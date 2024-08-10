#include <linux/err.h>
#include <linux/ion.h>
#include <linux/asoc_ion.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include "../ion_priv.h"
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>  
#include <asm/mach/map.h>
#include <asm/cacheflush.h>

extern struct ion_device *idev;
extern struct ion_mapper *asoc_user_mapper;
extern int num_heaps;
extern struct ion_heap **heaps;

//#define pr_debug printk

#define asoc_printk(format, arg...)				\
({								\
	if (0)							\
		printk(format, ##arg);	\
})


struct ion_cma_heap {
	struct ion_heap heap;
	struct device *dev;
	int allocated;
	int has_outer_cache;
};

#define ION_CMA_ALLOCATE_FAILED -1
#define to_cma_heap(x) container_of(x, struct ion_cma_heap, heap)

struct ion_cma_buffer_info {
	void *cpu_addr;
	dma_addr_t handle;
	struct sg_table *table;
};

#define dev_dbg(dev, format, arg...)				\
({								\
	if (0)							\
		printk(format, ##arg);	\
})

/*
 * Create scatter-list for the already allocated DMA buffer.
 * This function could be replaced by dma_common_get_sgtable
 * as soon as it will avalaible.
 */
static struct device	*cma_dev = NULL;
void set_cma_dev(struct device	*dev)
{
		cma_dev = dev;
}
int ion_cma_get_sgtable(struct device *dev, struct sg_table *sgt,
			void *cpu_addr, dma_addr_t handle, size_t size)
{
	struct page *page = virt_to_page(cpu_addr);
	int ret;

	ret = sg_alloc_table(sgt, 1, GFP_KERNEL);
	if (unlikely(ret))
		return ret;

	sg_set_page(sgt->sgl, page, PAGE_ALIGN(size), 0);
	return 0;
}

/*
 * Create scatter-list for each page of the already allocated DMA buffer.
 */
int ion_cma_get_sgtable_per_page(struct device *dev, struct sg_table *sgt,
			void *cpu_addr, dma_addr_t handle, size_t size)
{
	struct page *page = virt_to_page(cpu_addr);
	int ret, i;
	struct scatterlist *sg;

	ret = sg_alloc_table(sgt, PAGE_ALIGN(size) / PAGE_SIZE, GFP_KERNEL);
	if (unlikely(ret))
		return ret;

	sg = sgt->sgl;
	for (i = 0; i < (PAGE_ALIGN(size) / PAGE_SIZE); i++) {
		page = virt_to_page(cpu_addr + (i * PAGE_SIZE));
		sg_set_page(sg, page, PAGE_SIZE, 0);
		sg = sg_next(sg);
	}
	return 0;
}

void create_ion_debugfs(struct device * cma_dev);
/* ION CMA heap operations functions */
static int ion_cma_allocate(struct ion_heap *heap, struct ion_buffer *buffer,
			    unsigned long len, unsigned long align,
			    unsigned long flags)
{
	struct ion_cma_heap *cma_heap = to_cma_heap(heap);
	struct device *dev = cma_heap->dev;
	struct ion_cma_buffer_info *info;
	asoc_printk("pid: %d, comm: %s, ion_cma_allocate, dev: 0x%08x\n", current->pid, current->comm, dev);
	//dump_stack();
	if(dev)
		asoc_printk("ion_cma_allocate, dev->cma_area: 0x%08x\n", dev->cma_area);
	else
		asoc_printk("ion_cma_allocate, dev is null\n");
        create_ion_debugfs(dev);

	asoc_printk("Request buffer allocation len %ld, cma_heap->allocated: %d\n", len, cma_heap->allocated);

	info = kzalloc(sizeof(struct ion_cma_buffer_info), GFP_KERNEL);
	if (!info) {
		dev_err(dev, "Can't allocate buffer info\n");
		dump_stack();
		return ION_CMA_ALLOCATE_FAILED;
	}
	asoc_printk("%s, %d\n", __FUNCTION__, __LINE__);
	info->cpu_addr = dma_alloc_coherent(dev, len, &(info->handle), 0);

	if (!info->cpu_addr) {
		dev_err(dev, "Fail to allocate buffer\n");
		goto err;
	}
	asoc_printk("%s, %d\n", __FUNCTION__, __LINE__);
	cma_heap->allocated += len;

	info->table = kmalloc(sizeof(struct sg_table), GFP_KERNEL);
	if (!info->table) {
		dev_err(dev, "Fail to allocate sg table\n");
		goto free_mem;
	}
	asoc_printk("%s, %d\n", __FUNCTION__, __LINE__);
	if (ion_buffer_fault_user_mappings(buffer)) {
		if (ion_cma_get_sgtable_per_page
			(dev, info->table, info->cpu_addr, info->handle, len))
			goto free_table;
	} else {
		if (ion_cma_get_sgtable
			(dev, info->table, info->cpu_addr, info->handle, len))
			goto free_table;
	}
	/* keep this for memory release */
	buffer->priv_virt = info;
	asoc_printk("%s, %d\n", __FUNCTION__, __LINE__);
	dev_dbg(dev, "Allocate buffer %p\n", buffer);
	return 0;

free_table:
	kfree(info->table);
	asoc_printk("%s, %d\n", __FUNCTION__, __LINE__);
free_mem:
	asoc_printk("%s, %d\n", __FUNCTION__, __LINE__);
	dma_free_coherent(dev, len, info->cpu_addr, info->handle);
err:
	asoc_printk("%s, %d\n", __FUNCTION__, __LINE__);
	kfree(info);
	dump_stack();
	return ION_CMA_ALLOCATE_FAILED;
}

static void ion_cma_free(struct ion_buffer *buffer)
{
	struct ion_cma_heap *cma_heap = to_cma_heap(buffer->heap);
	struct device *dev = cma_heap->dev;
	struct ion_cma_buffer_info *info = buffer->priv_virt;

	dev_dbg(dev, "Release buffer %p\n", buffer);
	/* release memory */
	dma_free_coherent(dev, buffer->size, info->cpu_addr, info->handle);
	cma_heap->allocated -= buffer->size;
	/* release sg table */
	sg_free_table(info->table);
	kfree(info->table);
	kfree(info);
}

/* return physical address in addr */
static int ion_cma_phys(struct ion_heap *heap, struct ion_buffer *buffer,
			ion_phys_addr_t *addr, size_t *len)
{
	struct ion_cma_heap *cma_heap = to_cma_heap(buffer->heap);
	struct device *dev = cma_heap->dev;
	struct ion_cma_buffer_info *info = buffer->priv_virt;

	dev_dbg(dev, "Return buffer %p physical address 0x%x\n", buffer,
		info->handle);

	*addr = info->handle;
	*len = buffer->size;

	return 0;
}

struct sg_table *ion_cma_heap_map_dma(struct ion_heap *heap,
					 struct ion_buffer *buffer)
{
	struct ion_cma_buffer_info *info = buffer->priv_virt;

	return info->table;
}

void ion_cma_heap_unmap_dma(struct ion_heap *heap,
			       struct ion_buffer *buffer)
{
	//printk("%s, %s, %d\n", current->comm, __FUNCTION__, __LINE__);
}

#define pgprot_dmacoherent_cache(prot) \
	__pgprot_modify(prot, L_PTE_MT_MASK, L_PTE_MT_BUFFERABLE | L_PTE_XN | L_PTE_MT_WRITEBACK)
	
static int ion_cma_mmap(struct ion_heap *mapper, struct ion_buffer *buffer,
			struct vm_area_struct *vma)
{
	struct ion_cma_heap *cma_heap = to_cma_heap(buffer->heap);
	struct device *dev = cma_heap->dev;
	struct ion_cma_buffer_info *info = buffer->priv_virt;

	if(!ION_IS_CACHED(buffer->flags))
	{
		return dma_mmap_coherent(dev, vma, info->cpu_addr, info->handle,
				 buffer->size);
	}
	else
	{
		int ret;
		unsigned long pfn = dma_to_pfn(dev, info->handle);
		vma->vm_page_prot = pgprot_dmacoherent_cache(vma->vm_page_prot);
		ret = remap_pfn_range(vma, vma->vm_start,
				      pfn + vma->vm_pgoff,
				      vma->vm_end - vma->vm_start,
				      vma->vm_page_prot);	
		if(ret)
			printk("ion_cma_mmap failed\n");
		return ret;
	}
}

void *ion_cma_map_kernel(struct ion_heap *heap, struct ion_buffer *buffer)
{
	struct ion_cma_buffer_info *info = buffer->priv_virt;
	/* kernel memory mapping has been done at allocation time */
	return info->cpu_addr;
}
static int ion_cma_cache_ops(struct ion_heap *heap, struct ion_buffer *buffer,
			void *vaddr, unsigned int offset, unsigned int length,
			unsigned int cmd)
{
	struct ion_cma_buffer_info *info = buffer->priv_virt;
	void (*outer_cache_op)(phys_addr_t, phys_addr_t);
	struct ion_cma_heap *cma_heap =
	     container_of(heap, struct  ion_cma_heap, heap);

    pr_debug("%s %d: vaddr %x, offset %x, length %x, cmd %x\n", __FUNCTION__, __LINE__, 
        vaddr, offset, length, cmd);

	switch (cmd) {
	case ION_IOC_CLEAN_CACHES:
        pr_debug("%s %d: ION_IOC_CLEAN_CACHES vaddr %x length %x\n", 
            __FUNCTION__, __LINE__, vaddr, length);

		dmac_clean_range(vaddr, vaddr + length);
		outer_cache_op = outer_clean_range;
		break;
	case ION_IOC_INV_CACHES:
        pr_debug("%s %d: ION_IOC_INV_CACHES vaddr %x length %x\n", 
            __FUNCTION__, __LINE__, vaddr, length);

		dmac_inv_range(vaddr, vaddr + length);
		outer_cache_op = outer_inv_range;
		break;
	case ION_IOC_CLEAN_INV_CACHES:
        pr_debug("%s %d: ION_IOC_CLEAN_INV_CACHES vaddr %x length %x\n", 
            __FUNCTION__, __LINE__, vaddr, length);

		dmac_flush_range(vaddr, vaddr + length);
		outer_cache_op = outer_flush_range;
		break;
	default:
		return -EINVAL;
	}

	if (cma_heap->has_outer_cache) {
		unsigned long pstart = info->handle + offset;

        pr_debug("%s %d: outer_cache_op pstart %x end %x\n", 
            __FUNCTION__, __LINE__, pstart, pstart + length);

		outer_cache_op(pstart, pstart + length);
	}
	return 0;
}
static void ion_cma_unmap_kernel(struct ion_heap *heap,
				  struct ion_buffer *buffer)
{
	//printk("%s, %s, %d\n", current->comm, __FUNCTION__, __LINE__);
}

static struct ion_heap_ops ion_cma_ops = {
	.allocate = ion_cma_allocate,
	.free = ion_cma_free,
	.map_dma = ion_cma_heap_map_dma,
	.unmap_dma = ion_cma_heap_unmap_dma,
	.phys = ion_cma_phys,
	.map_user = ion_cma_mmap,
	.map_kernel = ion_cma_map_kernel,
	.unmap_kernel = ion_cma_unmap_kernel,
	.cache_op = ion_cma_cache_ops,
};

struct ion_heap *ion_cma_heap_create(struct ion_platform_heap *data)
{
	struct ion_cma_heap *cma_heap;

	cma_heap = kzalloc(sizeof(struct ion_cma_heap), GFP_KERNEL);

	if (!cma_heap)
		return ERR_PTR(-ENOMEM);

	cma_heap->heap.ops = &ion_cma_ops;
	/* get device from private heaps data, later it will be
	 * used to make the link with reserved CMA memory */
	cma_heap->dev = cma_dev;
	cma_heap->heap.type = ION_HEAP_TYPE_CMA;
	cma_heap->allocated = 0;
	cma_heap->has_outer_cache = 1;
	
	asoc_printk("cma_dev: 0x%08x\n", cma_dev);
	if(cma_dev) 
	  asoc_printk("%s(), cma_dev: %s\n", __FUNCTION__, cma_dev->init_name);
	 else
	 	asoc_printk("%s(), cma_dev is null\n", __FUNCTION__); 
	 		
	return &cma_heap->heap;
}

void ion_cma_heap_destroy(struct ion_heap *heap)
{
	struct ion_cma_heap *cma_heap = to_cma_heap(heap);
	kfree(cma_heap);
}

