/*
 * drivers/gpu/ion/ion_carveout_heap.c
 *
 * Copyright (C) 2011 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
#define DEBUG
*/

#include <linux/spinlock.h>

#include <linux/err.h>
#include <linux/genalloc.h>
#include <linux/io.h>
#include <linux/ion.h>
#include <linux/mm.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/seq_file.h>
#include "ion_priv.h"

#include <asm/mach/map.h>
#include <asm/cacheflush.h>

struct ion_carveout_heap {
	struct ion_heap heap;
	struct gen_pool *pool;
	ion_phys_addr_t base;
	unsigned long allocated_bytes;
	unsigned long total_size;
	int (*request_region)(void *);
	int (*release_region)(void *);
	atomic_t map_count;
	void *bus_id;
	unsigned int has_outer_cache;
};

ion_phys_addr_t ion_carveout_allocate(struct ion_heap *heap,
				      unsigned long size,
				      unsigned long align)
{
	struct ion_carveout_heap *carveout_heap =
		container_of(heap, struct ion_carveout_heap, heap);
	unsigned long offset;

	ION_PRINT("%s %d: size %ld\n", __func__, __LINE__, size);

/*
	unsigned long offset = gen_pool_alloc_aligned(carveout_heap->pool,
							size, ilog2(align));
*/
	offset = gen_pool_alloc(carveout_heap->pool, size);

	if (!offset) {
		if ((carveout_heap->total_size -
		      carveout_heap->allocated_bytes) >= size)
			ION_PRINT("%s: heap %s has enough memory (%lx) but"
				" the allocation of size %lx still failed."
				" Memory is probably fragmented.",
				__func__, heap->name,
				carveout_heap->total_size -
				carveout_heap->allocated_bytes, size);
		return ION_CARVEOUT_ALLOCATE_FAIL;
	}

	carveout_heap->allocated_bytes += size;

	ION_PRINT("%s %d: return offset %ld\n", __func__, __LINE__, offset);

	return offset;
}

void ion_carveout_free(struct ion_heap *heap, ion_phys_addr_t addr,
		       unsigned long size)
{
	struct ion_carveout_heap *carveout_heap =
		container_of(heap, struct ion_carveout_heap, heap);

	ION_PRINT("%s %d: addr 0x%lx, size %ld\n",
		__func__, __LINE__, addr, size);

	if (addr == ION_CARVEOUT_ALLOCATE_FAIL)
		return;
	gen_pool_free(carveout_heap->pool, addr, size);
	carveout_heap->allocated_bytes -= size;
}

static int ion_carveout_heap_phys(struct ion_heap *heap,
				  struct ion_buffer *buffer,
				  ion_phys_addr_t *addr, size_t *len)
{
	*addr = buffer->priv_phys;
	*len = buffer->size;
	return 0;
}

static int ion_carveout_heap_allocate(struct ion_heap *heap,
				      struct ion_buffer *buffer,
				      unsigned long size, unsigned long align,
				      unsigned long flags)
{
	buffer->priv_phys = ion_carveout_allocate(heap, size, align);
	return buffer->priv_phys == ION_CARVEOUT_ALLOCATE_FAIL ? -ENOMEM : 0;
}

static void ion_carveout_heap_free(struct ion_buffer *buffer)
{
	struct ion_heap *heap = buffer->heap;

	ion_carveout_free(heap, buffer->priv_phys, buffer->size);
	buffer->priv_phys = ION_CARVEOUT_ALLOCATE_FAIL;
}

struct sg_table *ion_carveout_heap_map_dma(struct ion_heap *heap,
					      struct ion_buffer *buffer)
{
	struct sg_table *table;
	int ret;

	ION_PRINT("%s %d:\n", __func__, __LINE__);

	table = kzalloc(sizeof(struct sg_table), GFP_KERNEL);
	if (!table)
		return ERR_PTR(-ENOMEM);

	ret = sg_alloc_table(table, 1, GFP_KERNEL);
	if (ret)
		goto err0;

	sg_set_page(table->sgl, phys_to_page(buffer->priv_phys), buffer->size,
		    0);

	ION_PRINT("%s %d: length %x page_link %x\n", __func__, __LINE__,
        table->sgl->length,
        table->sgl->page_link);

	return table;

err0:
	kfree(table);
	return ERR_PTR(ret);
}

void ion_carveout_heap_unmap_dma(struct ion_heap *heap,
				 struct ion_buffer *buffer)
{
	ION_PRINT("%s %d:\n", __func__, __LINE__);

	if (buffer->sg_table) {
        ION_PRINT("%s %d: length %x page_link %x dma_address %x\n", __func__, __LINE__,
            buffer->sg_table->sgl->length,
            buffer->sg_table->sgl->page_link,
            buffer->sg_table->sgl->dma_address);

		sg_free_table(buffer->sg_table);
    }
	kfree(buffer->sg_table);
	buffer->sg_table = 0;
}

static int ion_carveout_request_region(struct ion_carveout_heap *carveout_heap)
{
	int ret_value = 0;
	if (atomic_inc_return(&carveout_heap->map_count) == 1) {
		if (carveout_heap->request_region) {
			ret_value = carveout_heap->request_region(
						carveout_heap->bus_id);
			if (ret_value) {
				pr_err("Unable to request SMI region");
				atomic_dec(&carveout_heap->map_count);
			}
		}
	}
	return ret_value;
}

static int ion_carveout_release_region(struct ion_carveout_heap *carveout_heap)
{
	int ret_value = 0;
	if (atomic_dec_and_test(&carveout_heap->map_count)) {
		if (carveout_heap->release_region) {
			ret_value = carveout_heap->release_region(
						carveout_heap->bus_id);
			if (ret_value)
				pr_err("Unable to release SMI region");
		}
	}
	return ret_value;
}

void *ion_carveout_heap_map_kernel(struct ion_heap *heap,
				   struct ion_buffer *buffer)
{
	struct ion_carveout_heap *carveout_heap =
		container_of(heap, struct ion_carveout_heap, heap);
	void *ret_value;

	ION_PRINT("%s %d:\n", __func__, __LINE__);

	if (ion_carveout_request_region(carveout_heap))
		return NULL;

	if (ION_IS_CACHED(buffer->flags))
		ret_value = ioremap_cached(buffer->priv_phys, buffer->size);
	else
		ret_value = ioremap(buffer->priv_phys, buffer->size);

	if (!ret_value)
		ion_carveout_release_region(carveout_heap);
	return ret_value;
}

void ion_carveout_heap_unmap_kernel(struct ion_heap *heap,
				    struct ion_buffer *buffer)
{
	struct ion_carveout_heap *carveout_heap =
		container_of(heap, struct ion_carveout_heap, heap);

	ION_PRINT("%s %d:\n", __func__, __LINE__);

	__arm_iounmap(buffer->vaddr);
	buffer->vaddr = NULL;

	ion_carveout_release_region(carveout_heap);
	return;
}

int ion_carveout_heap_map_user(struct ion_heap *heap, struct ion_buffer *buffer,
			       struct vm_area_struct *vma)
{
	struct ion_carveout_heap *carveout_heap =
		container_of(heap, struct ion_carveout_heap, heap);
	int ret_value = 0;

	ION_PRINT("%s %d: vma->vm_page_prot 0x%x\n",
		__func__, __LINE__, vma->vm_page_prot);

	if (ion_carveout_request_region(carveout_heap))
		return -EINVAL;

	if (!ION_IS_CACHED(buffer->flags))
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	ION_PRINT("%s %d: remap_pfn_range vm_page_prot 0x%x\n",
		__func__, __LINE__, vma->vm_page_prot);

	ret_value =  remap_pfn_range(vma, vma->vm_start,
			__phys_to_pfn(buffer->priv_phys) + vma->vm_pgoff,
			vma->vm_end - vma->vm_start,
			vma->vm_page_prot);

	if (ret_value)
		ion_carveout_release_region(carveout_heap);
	return ret_value;
}

void ion_carveout_heap_unmap_user(struct ion_heap *heap,
				    struct ion_buffer *buffer)
{
	struct ion_carveout_heap *carveout_heap =
		container_of(heap, struct ion_carveout_heap, heap);

	ION_PRINT("%s %d:\n", __func__, __LINE__);

	ion_carveout_release_region(carveout_heap);
}

int ion_carveout_cache_ops(struct ion_heap *heap, struct ion_buffer *buffer,
			void *vaddr, unsigned int offset, unsigned int length,
			unsigned int cmd)
{
	void (*outer_cache_op)(phys_addr_t, phys_addr_t);
	struct ion_carveout_heap *carveout_heap =
		container_of(heap, struct  ion_carveout_heap, heap);

	ION_PRINT("%s %d: vaddr %x, offset %x, length %x, cmd %x\n",
		__func__, __LINE__, (unsigned int)vaddr, offset, length, cmd);

	switch (cmd) {
	case ION_IOC_CLEAN_CACHES:
		ION_PRINT("ION_IOC_CLEAN_CACHES");
		dmac_clean_range(vaddr, vaddr + length);
		outer_cache_op = outer_clean_range;
		break;

	case ION_IOC_INV_CACHES:
		ION_PRINT("ION_IOC_INV_CACHES");
		dmac_inv_range(vaddr, vaddr + length);
		outer_cache_op = outer_inv_range;
		break;

	case ION_IOC_CLEAN_INV_CACHES:
		ION_PRINT("ION_IOC_CLEAN_INV_CACHES");
		dmac_flush_range(vaddr, vaddr + length);
		outer_cache_op = outer_flush_range;
		break;

	default:
		return -EINVAL;
	}

	if (carveout_heap->has_outer_cache) {
		unsigned long pstart = buffer->priv_phys + offset;

	ION_PRINT("%s %d: outer_cache_op pstart %lx end %lx\n",
		__func__, __LINE__, pstart, pstart + length);

		outer_cache_op(pstart, pstart + length);
	}
	return 0;
}

static int ion_carveout_print_debug(struct ion_heap *heap, struct seq_file *s,
				    const struct rb_root *mem_map)
{
#if 0
	struct ion_carveout_heap *carveout_heap =
		container_of(heap, struct ion_carveout_heap, heap);

	seq_printf(s, "total bytes currently allocated: %lx\n",
		carveout_heap->allocated_bytes);
	seq_printf(s, "total heap size: %lx\n", carveout_heap->total_size);

	if (mem_map) {
		unsigned long base = carveout_heap->base;
		unsigned long size = carveout_heap->total_size;
		unsigned long end = base+size;
		unsigned long last_end = base;
		struct rb_node *n;

		seq_printf(s, "\nMemory Map\n");
		seq_printf(s, "%16.s %14.s %14.s %14.s\n",
			   "client", "start address", "end address",
			   "size (hex)");

		for (n = rb_first(mem_map); n; n = rb_next(n)) {
			struct mem_map_data *data =
					rb_entry(n, struct mem_map_data, node);
			const char *client_name = "(null)";

			if (last_end < data->addr) {
				seq_printf(s, "%16.s %14lx %14lx %14lu (%lx)\n",
					   "FREE", last_end, data->addr-1,
					   data->addr-last_end,
					   data->addr-last_end);
			}

			if (data->client_name)
				client_name = data->client_name;

			seq_printf(s, "%16.s %14lx %14lx %14lu (%lx)\n",
				   client_name, data->addr,
				   data->addr_end,
				   data->size, data->size);
			last_end = data->addr_end+1;
		}
		if (last_end < end) {
			seq_printf(s, "%16.s %14lx %14lx %14lu (%lx)\n", "FREE",
				last_end, end-1, end-last_end, end-last_end);
		}
	}
#endif
	return 0;
}
static struct ion_heap_ops carveout_heap_ops = {
	.allocate = ion_carveout_heap_allocate,
	.free = ion_carveout_heap_free,
	.phys = ion_carveout_heap_phys,
	.map_user = ion_carveout_heap_map_user,
	.map_kernel = ion_carveout_heap_map_kernel,
	.unmap_user = ion_carveout_heap_unmap_user,
	.unmap_kernel = ion_carveout_heap_unmap_kernel,
	.map_dma = ion_carveout_heap_map_dma,
	.unmap_dma = ion_carveout_heap_unmap_dma,
	.cache_op = ion_carveout_cache_ops,
	.print_debug = ion_carveout_print_debug,
};

struct ion_heap *ion_carveout_heap_create(struct ion_platform_heap *heap_data)
{
	struct ion_carveout_heap *carveout_heap;
	int ret;

	ION_PRINT("%s %d:\n", __func__, __LINE__);

	carveout_heap = kzalloc(sizeof(struct ion_carveout_heap), GFP_KERNEL);
	if (!carveout_heap)
		return ERR_PTR(-ENOMEM);

	carveout_heap->pool = gen_pool_create(12, -1);
	if (!carveout_heap->pool) {
		kfree(carveout_heap);
		return ERR_PTR(-ENOMEM);
	}
	carveout_heap->base = heap_data->base;
	ret = gen_pool_add(carveout_heap->pool, carveout_heap->base,
			heap_data->size, -1);
	if (ret < 0) {
		gen_pool_destroy(carveout_heap->pool);
		kfree(carveout_heap);
		return ERR_PTR(-EINVAL);
	}
	carveout_heap->heap.ops = &carveout_heap_ops;
	carveout_heap->heap.type = ION_HEAP_TYPE_CARVEOUT;
	carveout_heap->allocated_bytes = 0;
	carveout_heap->total_size = heap_data->size;
	carveout_heap->has_outer_cache = heap_data->has_outer_cache;

	return &carveout_heap->heap;
}

void ion_carveout_heap_destroy(struct ion_heap *heap)
{
	struct ion_carveout_heap *carveout_heap =
	     container_of(heap, struct  ion_carveout_heap, heap);

	ION_PRINT("%s %d:\n", __func__, __LINE__);

	gen_pool_destroy(carveout_heap->pool);
	kfree(carveout_heap);
	carveout_heap = NULL;
}
