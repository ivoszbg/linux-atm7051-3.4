/*
 * Contiguous Memory Allocator for DMA mapping framework
 * Copyright (c) 2010-2011 by Samsung Electronics.
 * Written by:
 *	Marek Szyprowski <m.szyprowski@samsung.com>
 *	Michal Nazarewicz <mina86@mina86.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License or (at your optional) any later version of the license.
 */

#define pr_fmt(fmt) "cma: " fmt

#ifdef CONFIG_CMA_DEBUG
#ifndef DEBUG
#  define DEBUG
#endif
#endif

#include <asm/page.h>
#include <asm/dma-contiguous.h>

#include <linux/memblock.h>
#include <linux/err.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/page-isolation.h>
#include <linux/slab.h>
#include <linux/swap.h>
#include <linux/mm_types.h>
#include <linux/dma-contiguous.h>

#ifndef SZ_1M
#define SZ_1M (1 << 20)
#endif

struct cma {
	unsigned long	base_pfn;
	unsigned long	count;
	unsigned long	*bitmap;
};

struct cma *dma_contiguous_default_area;

#ifdef CONFIG_CMA_SIZE_MBYTES
#define CMA_SIZE_MBYTES CONFIG_CMA_SIZE_MBYTES
#else
#define CMA_SIZE_MBYTES 0
#endif

/*
 * Default global CMA area size can be defined in kernel's .config.
 * This is usefull mainly for distro maintainers to create a kernel
 * that works correctly for most supported systems.
 * The size can be set in bytes or as a percentage of the total memory
 * in the system.
 *
 * Users, who want to set the size of global CMA area for their system
 * should use cma= kernel parameter.
 */
static const unsigned long size_bytes = CMA_SIZE_MBYTES * SZ_1M;
static long size_cmdline = -1;

static int __init early_cma(char *p)
{
	pr_debug("%s(%s)\n", __func__, p);
	size_cmdline = memparse(p, &p);
	return 0;
}
early_param("cma", early_cma);

#ifdef CONFIG_CMA_SIZE_PERCENTAGE

static unsigned long __init __maybe_unused cma_early_percent_memory(void)
{
	struct memblock_region *reg;
	unsigned long total_pages = 0;

	/*
	 * We cannot use memblock_phys_mem_size() here, because
	 * memblock_analyze() has not been called yet.
	 */
	for_each_memblock(memory, reg)
		total_pages += memblock_region_memory_end_pfn(reg) -
			       memblock_region_memory_base_pfn(reg);

	return (total_pages * CONFIG_CMA_SIZE_PERCENTAGE / 100) << PAGE_SHIFT;
}

#else

static inline __maybe_unused unsigned long cma_early_percent_memory(void)
{
	return 0;
}

#endif

/**
 * dma_contiguous_reserve() - reserve area for contiguous memory handling
 * @limit: End address of the reserved memory (optional, 0 for any).
 *
 * This function reserves memory from early allocator. It should be
 * called by arch specific code once the early allocator (memblock or bootmem)
 * has been activated and all other subsystems have already allocated/reserved
 * memory.
 */
void __init dma_contiguous_reserve(phys_addr_t limit)
{
	unsigned long selected_size = 0;

	pr_debug("%s(limit %08lx)\n", __func__, (unsigned long)limit);

	if (size_cmdline != -1) {
		selected_size = size_cmdline;
	} else {
#ifdef CONFIG_CMA_SIZE_SEL_MBYTES
		selected_size = size_bytes;
#elif defined(CONFIG_CMA_SIZE_SEL_PERCENTAGE)
		selected_size = cma_early_percent_memory();
#elif defined(CONFIG_CMA_SIZE_SEL_MIN)
		selected_size = min(size_bytes, cma_early_percent_memory());
#elif defined(CONFIG_CMA_SIZE_SEL_MAX)
		selected_size = max(size_bytes, cma_early_percent_memory());
#endif
	}
#if 1
	if(selected_size > 8*SZ_1M)
		selected_size = 8*SZ_1M;
	if (selected_size) {
		pr_debug("%s: reserving %ld MiB for global area\n", __func__,
			 selected_size / SZ_1M);

		dma_declare_contiguous(NULL, selected_size, 0, limit);
	}
#endif	
};

static DEFINE_MUTEX(cma_mutex);

static __init int cma_activate_area(unsigned long base_pfn, unsigned long count)
{
	unsigned long pfn = base_pfn;
	unsigned i = count >> pageblock_order;
	struct zone *zone;

	WARN_ON_ONCE(!pfn_valid(pfn));
	zone = page_zone(pfn_to_page(pfn));

	do {
		unsigned j;
		base_pfn = pfn;
		for (j = pageblock_nr_pages; j; --j, pfn++) {
			WARN_ON_ONCE(!pfn_valid(pfn));
			if (page_zone(pfn_to_page(pfn)) != zone)
				return -EINVAL;
		}
		init_cma_reserved_pageblock(pfn_to_page(base_pfn));
	} while (--i);
	return 0;
}
static __init struct cma *cma_create_area(unsigned long base_pfn,
				     unsigned long count)
{
	int bitmap_size = BITS_TO_LONGS(count) * sizeof(long);
	struct cma *cma;
	int ret = -ENOMEM;

	pr_debug("%s(base %08lx, count %lx)\n", __func__, base_pfn, count);

	cma = kmalloc(sizeof *cma, GFP_KERNEL);
	if (!cma)
		return ERR_PTR(-ENOMEM);

	cma->base_pfn = base_pfn;
	cma->count = count;
	cma->bitmap = kzalloc(bitmap_size, GFP_KERNEL);

	if (!cma->bitmap)
		goto no_mem;

	ret = cma_activate_area(base_pfn, count);
	if (ret)
		goto error;

	pr_debug("%s: returned %p\n", __func__, (void *)cma);
	return cma;

error:
	kfree(cma->bitmap);
no_mem:
	kfree(cma);
	return ERR_PTR(ret);
}

static struct cma_reserved {
	phys_addr_t start;
	unsigned long size;
	struct device *dev;
} cma_reserved[MAX_CMA_AREAS] __initdata;
static unsigned cma_reserved_count __initdata;

static struct cma_reserve_reserved {
	phys_addr_t start_pfn;
	phys_addr_t end_pfn;
	phys_addr_t limit_pfn;
	unsigned long sum_pages;
	struct device *dev;
} cma_reserve_reserved;

void register_cma_reserve(phys_addr_t startpfn, 
	unsigned long numpages,
	phys_addr_t limitpfn,
	struct device *dev){

	if(cma_reserve_reserved.dev)
		return;
	cma_reserve_reserved.dev = dev;
	cma_reserve_reserved.start_pfn = startpfn;
	cma_reserve_reserved.sum_pages = numpages;
	cma_reserve_reserved.end_pfn = startpfn + numpages;
	cma_reserve_reserved.limit_pfn = limitpfn;
	printk("dev: 0x%08x, startpfn: %d, endpfn: %d, sum: %d, limit: %d\n",
		dev, startpfn, startpfn + numpages, numpages, limitpfn);
}	
static struct page *
__first_valid_page(unsigned long pfn, unsigned long nr_pages)
{
	int i;
	for (i = 0; i < nr_pages; i++)
		if (pfn_valid_within(pfn + i))
			break;
	if (unlikely(i == nr_pages))
		return NULL;
	return pfn_to_page(pfn + i);
}
extern void set_pageblock_migratetype(struct page *page, int migratetype);
static void check_cma_reserve_cover(struct device *dev, phys_addr_t pfn, unsigned long  count){
	int startpfn;
	if(dev != cma_reserve_reserved.dev)
		return;
	if(cma_reserve_reserved.limit_pfn <=cma_reserve_reserved.end_pfn)	
		return;
	for(startpfn = cma_reserve_reserved.start_pfn; startpfn<cma_reserve_reserved.limit_pfn; startpfn+=pageblock_nr_pages){
			struct page *page =  __first_valid_page(startpfn, pageblock_nr_pages);
			if(!page)
				continue;
			if (startpfn < cma_reserve_reserved.end_pfn)
				set_pageblock_migratetype(page, MIGRATE_CMA_RESERVE);
			else
				set_pageblock_migratetype(page, MIGRATE_CMA);
	}
}

int check_cma_continued(struct page *page){
	int pfn = __page_to_pfn(page);
	//if(!cma_reserve_reserved.dev)
	//	return 0;
	if((pfn< cma_reserve_reserved.limit_pfn) && (pfn >= cma_reserve_reserved.start_pfn)) {
		return 1;
	}
	return 0;
}

int check_cma_reserve(struct page *page){
	int pfn = __page_to_pfn(page);
	//if(!cma_reserve_reserved.dev)
	//	return 0;
	if((pfn< cma_reserve_reserved.end_pfn) && (pfn >= cma_reserve_reserved.start_pfn)) {
		return 1;
	}
	return 0;
}
int check_cma_reserve_pfn(phys_addr_t pfn){
	if((pfn< cma_reserve_reserved.end_pfn) && (pfn >= cma_reserve_reserved.start_pfn)) {
		return 1;
	}
	return 0;
}

extern void dump_cma_pageblocks();
#define CMA_RESERVE_FOR_512M (1*SZ_1M/4/1024)
#define CMA_RESERVE_FOR_1024M (128*SZ_1M/4/1024)
#define PAGES_512M_SUM  (512*SZ_1M/4/1024)

static int cma_reserved_size_in_page = CMA_RESERVE_FOR_512M;
int get_cma_reserved_size_in_page()
{
		return cma_reserved_size_in_page;
}
int use_cma_alloc_first()
{
		//return (totalram_pages <= PAGES_512M_SUM);
		return 1;
}
static int calculate_cma_reserve() {
		if(totalram_pages <= PAGES_512M_SUM) {
				cma_reserved_size_in_page = CMA_RESERVE_FOR_512M;
		}
		else {
				//cma_reserved_size_in_page = CMA_RESERVE_FOR_1024M;
				/*all reserved memory reserved in cma*/
				//cma_reserved_size_in_page = PAGES_512M_SUM;
				cma_reserved_size_in_page = CMA_RESERVE_FOR_512M;
		}
}

static int __init cma_init_reserved_areas(void)
{
	struct cma_reserved *r = cma_reserved;
	unsigned i = cma_reserved_count;
	calculate_cma_reserve();
	for (; i; --i, ++r) {
		struct cma *cma;
		printk("cma area[%d], startpfn: %d, endpfn: %d\n", i, PFN_DOWN(r->start), PFN_DOWN(r->start)+(r->size >> PAGE_SHIFT));
		if(cma_reserved_size_in_page > (r->size >> PAGE_SHIFT)) {
				cma_reserved_size_in_page = (r->size >> PAGE_SHIFT);
		}
		if(r->dev)
			 register_cma_reserve(PFN_DOWN(r->start), 
				cma_reserved_size_in_page,
				PFN_DOWN(r->start) + (r->size >> PAGE_SHIFT),
				r->dev);
		cma = cma_create_area(PFN_DOWN(r->start),
				      r->size >> PAGE_SHIFT);
		if (!IS_ERR(cma))
			dev_set_cma_area(r->dev, cma);
	}
	dump_cma_pageblocks();
	
	printk("cma_reserve_reserved.dev: 0x%08x\n",	cma_reserve_reserved.dev);
	printk("cma_reserve_reserved.start_pfn: 0x%08x\n",	cma_reserve_reserved.start_pfn);
	printk("cma_reserve_reserved.end_pfn: 0x%08x\n",	cma_reserve_reserved.end_pfn);
	printk("cma_reserve_reserved.limit_pfn: 0x%08x\n",	cma_reserve_reserved.limit_pfn);
	printk("cma_reserve_reserved.sum_pages: 0x%08x\n",	cma_reserve_reserved.sum_pages);
	return 0;
}
core_initcall(cma_init_reserved_areas);

/**
 * dma_declare_contiguous() - reserve area for contiguous memory handling
 *			      for particular device
 * @dev:   Pointer to device structure.
 * @size:  Size of the reserved memory.
 * @base:  Start address of the reserved memory (optional, 0 for any).
 * @limit: End address of the reserved memory (optional, 0 for any).
 *
 * This function reserves memory for specified device. It should be
 * called by board specific code when early allocator (memblock or bootmem)
 * is still activate.
 */
int __init dma_declare_contiguous(struct device *dev, unsigned long size,
				  phys_addr_t base, phys_addr_t limit)
{
	struct cma_reserved *r = &cma_reserved[cma_reserved_count];
	unsigned long alignment;

	pr_debug("%s(size %lx, base %08lx, limit %08lx)\n", __func__,
		 (unsigned long)size, (unsigned long)base,
		 (unsigned long)limit);

	/* Sanity checks */
	if (cma_reserved_count == ARRAY_SIZE(cma_reserved)) {
		pr_err("Not enough slots for CMA reserved regions!\n");
		return -ENOSPC;
	}

	if (!size)
		return -EINVAL;

	/* Sanitise input arguments */
	alignment = PAGE_SIZE << max(MAX_ORDER, pageblock_order);
	base = ALIGN(base, alignment);
	size = ALIGN(size, alignment);
	limit &= ~(alignment - 1);

	/* Reserve memory */
	if (base) {
		if (memblock_is_region_reserved(base, size) ||
		    memblock_reserve(base, size) < 0) {
			base = -EBUSY;
			goto err;
		}
	} else {
		/*
		 * Use __memblock_alloc_base() since
		 * memblock_alloc_base() panic()s.
		 */
		phys_addr_t addr = __memblock_alloc_base(size, alignment, limit);
		if (!addr) {
			base = -ENOMEM;
			goto err;
		} else if (addr + size > ~(unsigned long)0) {
			memblock_free(addr, size);
			base = -EINVAL;
			goto err;
		} else {
			base = addr;
		}
	}

	/*
	 * Each reserved area must be initialised later, when more kernel
	 * subsystems (like slab allocator) are available.
	 */
	r->start = base;
	r->size = size;
	r->dev = dev;
	cma_reserved_count++;
	pr_info("CMA: reserved %ld MiB at %08lx\n", size / SZ_1M,
		(unsigned long)base);

	/* Architecture specific contiguous memory fixup. */
	dma_contiguous_early_fixup(base, size);
	return 0;
err:
	pr_err("CMA: failed to reserve %ld MiB\n", size / SZ_1M);
	return base;
}

/**
 * cma_mem_stat() - stat the cma free area.
 * 
*/

static void cma_mem_stat(struct cma *cma)
{
	unsigned long free_pages = 0;	//for checksum here.
	unsigned long mask = 0;
    unsigned long pfn, pageno, start = 0;
	unsigned long plast = bitmap_find_next_zero_area(cma->bitmap, cma->count,
						    0, 1, mask);
	unsigned long p = plast;

	WARN_ON(!cma);
	
    printk("cma: count %lu, base_pfn %lu, 1st free page %lu\n", cma->count, cma->base_pfn, plast);
	pageno = plast;
	for (;;) {
		pageno = bitmap_find_next_zero_area(cma->bitmap, cma->count,
					    pageno + 1, 1, mask);
		if (pageno >= cma->count) {
			printk("cma free: %06lu <-->%06lu, size: %luk\n", 
					plast, p , (p  - plast + 1) * 4 );
			printk("%s, %d, total_size: %lu, free_pages: %lu, used pages %lu\n", __FUNCTION__, __LINE__, cma->count, free_pages, __bitmap_weight(cma->bitmap, cma->count));
			return;
		}
		if((p + 1) != pageno){
			printk("cma free: %06lu <-->%06lu, size: %luk\n", 
				plast, p , (p  - plast + 1) * 4 );
			p  = pageno;
			plast = pageno;
		} else {
			p ++;
			free_pages++;
		}
	}
}


/**
 * dma_alloc_from_contiguous() - allocate pages from contiguous area
 * @dev:   Pointer to device for which the allocation is performed.
 * @count: Requested number of pages.
 * @align: Requested alignment of pages (in PAGE_SIZE order).
 *
 * This function allocates memory buffer for specified device. It uses
 * device specific contiguous memory area if available or the default
 * global one. Requires architecture specific get_dev_cma_area() helper
 * function.
 */
struct page *dma_alloc_from_contiguous(struct device *dev, int count,
				       unsigned int align)
{
	unsigned long mask, pfn, pageno, start = 0, last_pfn, step;
	struct cma *cma = dev_get_cma_area(dev);
	int ret = 0;
	struct timeval trace_start_tv;
	struct timeval trace_end_tv;

	
	if (!cma || !cma->count)
	{	
		printk("%s, %d\n", __FUNCTION__, __LINE__);
		return NULL;
	}

	if (align > CONFIG_CMA_ALIGNMENT)
		align = CONFIG_CMA_ALIGNMENT;

	cma_printk("%s(cma %p, count %d, align %d)\n", __func__, (void *)cma,
		 count, align);

	if (!count)
	{
		printk("%s, %d\n", __FUNCTION__, __LINE__);
		return NULL;
	}

	mask = (1 << align) - 1;

	mutex_lock(&cma_mutex);
	do_gettimeofday(&trace_start_tv);	
	for (;;) {
		pageno = bitmap_find_next_zero_area(cma->bitmap, cma->count,
						    start, count, mask);
		if (pageno >= cma->count) {
			ret = -ENOMEM;
			printk("%s, %d, fail alloc cma with size: %d\n", __FUNCTION__, __LINE__, count);
			cma_mem_stat(cma);
			goto error;
		}
		pfn = cma->base_pfn + pageno;
		if(check_cma_reserve_pfn(pfn))
			ret = alloc_contig_range(pfn, pfn + count, MIGRATE_CMA_RESERVE, &last_pfn);
		else
			ret = alloc_contig_range(pfn, pfn + count, MIGRATE_CMA, &last_pfn);
		check_cma_reserve_cover(dev, pfn, count);
		//printk("pageno: %d, testfpn: %d, size: %d, ret: %d\n", pageno, pfn, count, ret);
#if 0
{
		if(page_mapped(pfn_to_page(pfn)))
		{
			printk("erro still mapped dev: 0x%08x, pageno: %d, testfpn: %d, size: %d, ret: %d\n", dev, pageno, pfn, count, ret);
			ret = -1;
		}
}
#endif		
		if (ret == 0) {
			bitmap_set(cma->bitmap, pageno, count);
			break;
		} else if (ret != -EBUSY) {
			printk("%s, %d\n", __FUNCTION__, __LINE__);
			goto error;
		}
		pr_debug("%s(): memory range at %p is busy, retrying\n",
			 __func__, pfn_to_page(pfn));
		step = max(mask, last_pfn - pfn);
		/* try again with a bit different memory target */
		start = pageno + step + 1;
	}
	do_gettimeofday(&trace_end_tv);
	long time_us = (trace_end_tv.tv_sec - trace_start_tv.tv_sec)*1000000 + \
			 (trace_end_tv.tv_usec- trace_start_tv.tv_usec);
	if (time_us > 500*1000) 
		pr_err("[%s:%d],used time: %d ms\n", __func__, __LINE__, time_us/1000);
	mutex_unlock(&cma_mutex);

	cma_printk("%s(): returned %p\n", __func__, pfn_to_page(pfn));
	return pfn_to_page(pfn);
error:
	mutex_unlock(&cma_mutex);
	return NULL;
}
#if 0
void check_cma_alloced(unsigned long pfn)
{
	unsigned long pageno1, pageno2;
	struct cma *cma = dev_get_cma_area(NULL);

	pageno1 = pfn - cma->base_pfn;
	if(pageno1 > cma->count)
		return;
	
	pageno2 = bitmap_find_next_zero_area(cma->bitmap, cma->count,
						    pageno1, 1, 0);
	if(pageno2 != pageno1)
	{
		printk("logic wrong:\n");
		printk("pfn: %d is alloced by cma\n", pfn);
		printk("pageno1: %d,  pageno2: %d\n", pageno1, pageno2);
	}
		
}
#endif
/**
 * dma_release_from_contiguous() - release allocated pages
 * @dev:   Pointer to device for which the pages were allocated.
 * @pages: Allocated pages.
 * @count: Number of allocated pages.
 *
 * This function releases memory allocated by dma_alloc_from_contiguous().
 * It returns false when provided pages do not belong to contiguous area and
 * true otherwise.
 */
bool dma_release_from_contiguous(struct device *dev, struct page *pages,
				 int count)
{
	struct cma *cma = dev_get_cma_area(dev);
	unsigned long pfn;

	if (!cma || !pages)
		return false;

	pr_debug("%s(page %p)\n", __func__, (void *)pages);

	pfn = page_to_pfn(pages);

	if (pfn < cma->base_pfn || pfn >= cma->base_pfn + cma->count)
		return false;

	VM_BUG_ON(pfn + count > cma->base_pfn + cma->count);

	mutex_lock(&cma_mutex);
	bitmap_clear(cma->bitmap, pfn - cma->base_pfn, count);
	free_contig_range(pfn, count);
	mutex_unlock(&cma_mutex);

	return true;
}
