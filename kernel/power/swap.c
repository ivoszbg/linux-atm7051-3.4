/*
 * linux/kernel/power/swap.c
 *
 * This file provides functions for reading the suspend image from
 * and writing it to a swap partition.
 *
 * Copyright (C) 1998,2001-2005 Pavel Machek <pavel@ucw.cz>
 * Copyright (C) 2006 Rafael J. Wysocki <rjw@sisk.pl>
 * Copyright (C) 2010 Bojan Smojver <bojan@rexursive.com>
 *
 * This file is released under the GPLv2.
 *
 */

#include <linux/module.h>
#include <linux/file.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/genhd.h>
#include <linux/device.h>
#include <linux/bio.h>
#include <linux/blkdev.h>
#include <linux/swap.h>
#include <linux/swapops.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/lzo.h>
#include <linux/vmalloc.h>
#include <linux/cpumask.h>
#include <linux/atomic.h>
#include <linux/kthread.h>
#include <linux/crc32.h>
#include <linux/mm.h>
#include <asm/pgalloc.h>
#include <asm/idmap.h>
#include <linux/sched.h>

#include "power.h"
#include "asoc_ioctl.h"

#define HIBERNATE_SIG	"S1SUSPEND"

#define SAVE_RESERVE_HOLE		0
#define ENABLE_CRC				0

#define CPU_AFFINITY
#define rw_thread_prio		-20
#define main_thread_prio	-20
#define lzo_thread_prio		-20
#define rw_thread_cpu		1
#define main_thread_cpu		0
extern int hibernate_power_long_press(void);
int power_interrupt = 0;

#ifdef CPU_AFFINITY
static void set_kernel_thread_prop(int affinity_cpu, int nice)
{
	struct cpumask dstp;
	struct sched_param param;
	
	cpumask_empty(&dstp);

	if(affinity_cpu >= num_online_cpus())
		affinity_cpu = 0;
	
	cpumask_set_cpu(affinity_cpu, &dstp);
	if(sched_setaffinity(0, &dstp) < 0)
		printk("affinity set to cpu%d failed\n", affinity_cpu);
		
	set_user_nice(current, nice);
}
#else
#define set_kernel_thread_prop(v1, v2)	do{} while(0)
#endif

#ifndef CONFIG_FAST_HIBERNATE
/*
 *	The swap map is a data structure used for keeping track of each page
 *	written to a swap partition.  It consists of many swap_map_page
 *	structures that contain each an array of MAP_PAGE_ENTRIES swap entries.
 *	These structures are stored on the swap and linked together with the
 *	help of the .next_swap member.
 *
 *	The swap map is created during suspend.  The swap map pages are
 *	allocated and populated one at a time, so we only need one memory
 *	page to set up the entire structure.
 *
 *	During resume we pick up all swap_map_page structures into a list.
 */

#define MAP_PAGE_ENTRIES	(PAGE_SIZE / sizeof(sector_t) - 1)


/*
 * Number of free pages that are not high.
 */
static inline unsigned long low_free_pages(void)
{
	return nr_free_pages() - nr_free_highpages();
}

/*
 * Number of pages required to be kept free while writing the image. Always
 * half of all available low pages before the writing starts.
 */
static inline unsigned long reqd_free_pages(void)
{
	return low_free_pages() / 2;
}

struct swap_map_page {
	sector_t entries[MAP_PAGE_ENTRIES];
	sector_t next_swap;
};

struct swap_map_page_list {
	struct swap_map_page *map;
	struct swap_map_page_list *next;
};

/**
 *	The swap_map_handle structure is used for handling swap in
 *	a file-alike way
 */

struct swap_map_handle {
	struct swap_map_page *cur;
	struct swap_map_page_list *maps;
	sector_t cur_swap;
	sector_t first_sector;
	unsigned int k;
	unsigned long reqd_free_pages;
	u32 crc32;
};

struct swsusp_header {
	char reserved[PAGE_SIZE - 20 - sizeof(sector_t) - sizeof(int) -
	              sizeof(u32)];
	u32	crc32;
	sector_t image;
	unsigned int flags;	/* Flags to pass to the "boot" kernel */
	char	orig_sig[10];
	char	sig[10];
} __attribute__((packed));

static struct swsusp_header *swsusp_header;

/**
 *	The following functions are used for tracing the allocated
 *	swap pages, so that they can be freed in case of an error.
 */

struct swsusp_extent {
	struct rb_node node;
	unsigned long start;
	unsigned long end;
};

static struct rb_root swsusp_extents = RB_ROOT;

static int swsusp_extents_insert(unsigned long swap_offset)
{
	struct rb_node **new = &(swsusp_extents.rb_node);
	struct rb_node *parent = NULL;
	struct swsusp_extent *ext;

	/* Figure out where to put the new node */
	while (*new) {
		ext = container_of(*new, struct swsusp_extent, node);
		parent = *new;
		if (swap_offset < ext->start) {
			/* Try to merge */
			if (swap_offset == ext->start - 1) {
				ext->start--;
				return 0;
			}
			new = &((*new)->rb_left);
		} else if (swap_offset > ext->end) {
			/* Try to merge */
			if (swap_offset == ext->end + 1) {
				ext->end++;
				return 0;
			}
			new = &((*new)->rb_right);
		} else {
			/* It already is in the tree */
			return -EINVAL;
		}
	}
	/* Add the new node and rebalance the tree. */
	ext = kzalloc(sizeof(struct swsusp_extent), GFP_KERNEL);
	if (!ext)
		return -ENOMEM;

	ext->start = swap_offset;
	ext->end = swap_offset;
	rb_link_node(&ext->node, parent, new);
	rb_insert_color(&ext->node, &swsusp_extents);
	return 0;
}

/**
 *	alloc_swapdev_block - allocate a swap page and register that it has
 *	been allocated, so that it can be freed in case of an error.
 */

sector_t alloc_swapdev_block(int swap)
{
	unsigned long offset;

	offset = swp_offset(get_swap_page_of_type(swap));
	if (offset) {
		if (swsusp_extents_insert(offset))
			swap_free(swp_entry(swap, offset));
		else
			return swapdev_block(swap, offset);
	}
	return 0;
}

/**
 *	free_all_swap_pages - free swap pages allocated for saving image data.
 *	It also frees the extents used to register which swap entries had been
 *	allocated.
 */

void free_all_swap_pages(int swap)
{
	struct rb_node *node;

	while ((node = swsusp_extents.rb_node)) {
		struct swsusp_extent *ext;
		unsigned long offset;

		ext = container_of(node, struct swsusp_extent, node);
		rb_erase(node, &swsusp_extents);
		for (offset = ext->start; offset <= ext->end; offset++)
			swap_free(swp_entry(swap, offset));

		kfree(ext);
	}
}

int swsusp_swap_in_use(void)
{
	return (swsusp_extents.rb_node != NULL);
}

/*
 * General things
 */

static unsigned short root_swap = 0xffff;
struct block_device *hib_resume_bdev;

/*
 * Saving part
 */

static int mark_swapfiles(struct swap_map_handle *handle, unsigned int flags)
{
	int error;

	hib_bio_read_page(swsusp_resume_block, swsusp_header, NULL);
	if (!memcmp("SWAP-SPACE",swsusp_header->sig, 10) ||
	    !memcmp("SWAPSPACE2",swsusp_header->sig, 10)) {
		memcpy(swsusp_header->orig_sig,swsusp_header->sig, 10);
		memcpy(swsusp_header->sig, HIBERNATE_SIG, 10);
		swsusp_header->image = handle->first_sector;
		swsusp_header->flags = flags;
		if (flags & SF_CRC32_MODE)
			swsusp_header->crc32 = handle->crc32;
		error = hib_bio_write_page(swsusp_resume_block,
					swsusp_header, NULL);
	} else {
		printk(KERN_ERR "PM: Swap header not found!\n");
		error = -ENODEV;
	}
	return error;
}

/**
 *	swsusp_swap_check - check if the resume device is a swap device
 *	and get its index (if so)
 *
 *	This is called before saving image
 */
static int swsusp_swap_check(void)
{
	int res;

	res = swap_type_of(swsusp_resume_device, swsusp_resume_block,
			&hib_resume_bdev);
	if (res < 0)
		return res;

	root_swap = res;
	res = blkdev_get(hib_resume_bdev, FMODE_WRITE, NULL);
	if (res)
		return res;

	res = set_blocksize(hib_resume_bdev, PAGE_SIZE);
	if (res < 0)
		blkdev_put(hib_resume_bdev, FMODE_WRITE);

	return res;
}

/**
 *	write_page - Write one page to given swap location.
 *	@buf:		Address we're writing.
 *	@offset:	Offset of the swap page we're writing to.
 *	@bio_chain:	Link the next write BIO here
 */

static int write_page(void *buf, sector_t offset, struct bio **bio_chain)
{
	void *src;
	int ret;

	if (!offset)
		return -ENOSPC;

	if (bio_chain) {
		src = (void *)__get_free_page(__GFP_WAIT | __GFP_HIGH);
		if (src) {
			copy_page(src, buf);
		} else {
			ret = hib_wait_on_bio_chain(bio_chain); /* Free pages */
			if (ret)
				return ret;
			src = (void *)__get_free_page(__GFP_WAIT | __GFP_HIGH);
			if (src) {
				copy_page(src, buf);
			} else {
				WARN_ON_ONCE(1);
				bio_chain = NULL;	/* Go synchronous */
				src = buf;
			}
		}
	} else {
		src = buf;
	}
	return hib_bio_write_page(offset, src, bio_chain);
}

static void release_swap_writer(struct swap_map_handle *handle)
{
	if (handle->cur)
		free_page((unsigned long)handle->cur);
	handle->cur = NULL;
}

static int get_swap_writer(struct swap_map_handle *handle)
{
	int ret;

	ret = swsusp_swap_check();
	if (ret) {
		if (ret != -ENOSPC)
			printk(KERN_ERR "PM: Cannot find swap device, try "
					"swapon -a.\n");
		return ret;
	}
	handle->cur = (struct swap_map_page *)get_zeroed_page(GFP_KERNEL);
	if (!handle->cur) {
		ret = -ENOMEM;
		goto err_close;
	}
	handle->cur_swap = alloc_swapdev_block(root_swap);
	if (!handle->cur_swap) {
		ret = -ENOSPC;
		goto err_rel;
	}
	handle->k = 0;
	handle->reqd_free_pages = reqd_free_pages();
	handle->first_sector = handle->cur_swap;
	return 0;
err_rel:
	release_swap_writer(handle);
err_close:
	swsusp_close(FMODE_WRITE);
	return ret;
}

static int swap_write_page(struct swap_map_handle *handle, void *buf,
				struct bio **bio_chain, sector_t *offset_save)
{
	int error = 0;
	sector_t offset;

	if (!handle->cur)
		return -EINVAL;
	offset = alloc_swapdev_block(root_swap);
	error = write_page(buf, offset, bio_chain);
	if (error)
		return error;
	if(offset_save != NULL)
		*offset_save = offset;
		
	handle->cur->entries[handle->k++] = offset;
	if (handle->k >= MAP_PAGE_ENTRIES) {
		offset = alloc_swapdev_block(root_swap);
		if (!offset)
			return -ENOSPC;
		handle->cur->next_swap = offset;
		error = write_page(handle->cur, handle->cur_swap, bio_chain);
		if (error)
			goto out;
		clear_page(handle->cur);
		handle->cur_swap = offset;
		handle->k = 0;
	}
	if (bio_chain && low_free_pages() <= handle->reqd_free_pages) {
		error = hib_wait_on_bio_chain(bio_chain);
		if (error)
			goto out;
		handle->reqd_free_pages = reqd_free_pages();
	}
 out:
	return error;
}

static int flush_swap_writer(struct swap_map_handle *handle)
{
	if (handle->cur && handle->cur_swap)
		return write_page(handle->cur, handle->cur_swap, NULL);
	else
		return -EINVAL;
}

static int swap_writer_finish(struct swap_map_handle *handle,
		unsigned int flags, int error)
{
	if (!error) {
		flush_swap_writer(handle);
		printk(KERN_INFO "PM: S");
		error = mark_swapfiles(handle, flags);
		printk("|\n");
	}

	if (error)
		free_all_swap_pages(root_swap);
	release_swap_writer(handle);
	swsusp_close(FMODE_WRITE);

	return error;
}

/* We need to remember how much compressed data we need to read. */
#define LZO_HEADER	sizeof(size_t)

/* Number of pages/bytes we'll compress at one time. */
#define LZO_UNC_PAGES	32
#define LZO_UNC_SIZE	(LZO_UNC_PAGES * PAGE_SIZE)

/* Number of pages/bytes we need for compressed data (worst case). */
#define LZO_CMP_PAGES	DIV_ROUND_UP(lzo1x_worst_compress(LZO_UNC_SIZE) + \
			             LZO_HEADER, PAGE_SIZE)
#define LZO_CMP_SIZE	(LZO_CMP_PAGES * PAGE_SIZE)

/* Maximum number of threads for compression/decompression. */
#define LZO_THREADS	3

/* Maximum number of pages for read buffering. */
#define LZO_READ_PAGES	(MAP_PAGE_ENTRIES * 8)


/**
 *	save_image - save the suspend image data
 */

static int save_image(struct swap_map_handle *handle,
                      struct snapshot_handle *snapshot,
                      unsigned int nr_to_write)
{
	unsigned int m;
	int ret;
	int nr_pages;
	int err2;
	struct bio *bio;
	struct timeval start;
	struct timeval stop;

	printk(KERN_INFO "PM: Saving image data pages (%u pages) ...     ",
		nr_to_write);
	m = nr_to_write / 100;
	if (!m)
		m = 1;
	nr_pages = 0;
	bio = NULL;
	do_gettimeofday(&start);
	while (1) {
		ret = snapshot_read_next(snapshot);
		if (ret <= 0)
			break;
		ret = swap_write_page(handle, data_of(*snapshot), &bio, NULL);
		if (ret)
			break;
		if (!(nr_pages % m))
			printk(KERN_CONT "\b\b\b\b%3d%%", nr_pages / m);
		nr_pages++;
	}
	err2 = hib_wait_on_bio_chain(&bio);
	do_gettimeofday(&stop);
	if (!ret)
		ret = err2;
	if (!ret)
		printk(KERN_CONT "\b\b\b\bdone\n");
	else
		printk(KERN_CONT "\n");
	swsusp_show_speed(&start, &stop, nr_to_write, "Wrote");
	return ret;
}

/**
 * Structure used for CRC32.
 */
#if ENABLE_CRC
struct crc_data {
	struct task_struct *thr;                  /* thread */
	atomic_t ready;                           /* ready to start flag */
	atomic_t stop;                            /* ready to stop flag */
	unsigned run_threads;                     /* nr current threads */
	wait_queue_head_t go;                     /* start crc update */
	wait_queue_head_t done;                   /* crc update done */
	u32 *crc32;                               /* points to handle's crc32 */
	size_t *unc_len[LZO_THREADS];             /* uncompressed lengths */
	unsigned char *unc[LZO_THREADS];          /* uncompressed data */
};

/**
 * CRC32 update function that runs in its own thread.
 */
static int crc32_threadfn(void *data)
{
	struct crc_data *d = data;
	unsigned i;

	while (1) {
		wait_event(d->go, atomic_read(&d->ready) ||
		                  kthread_should_stop());
		if (kthread_should_stop()) {
			d->thr = NULL;
			atomic_set(&d->stop, 1);
			wake_up(&d->done);
			break;
		}
		atomic_set(&d->ready, 0);

		for (i = 0; i < d->run_threads; i++)
			*d->crc32 = crc32_le(*d->crc32,
			                     d->unc[i], *d->unc_len[i]);
		atomic_set(&d->stop, 1);
		wake_up(&d->done);
	}
	return 0;
}
#endif
/**
 * Structure used for LZO data compression.
 */
struct cmp_data {
	struct task_struct *thr;                  /* thread */
	atomic_t ready;                           /* ready to start flag */
	atomic_t stop;                            /* ready to stop flag */
	int ret;                                  /* return code */
	wait_queue_head_t go;                     /* start compression */
	wait_queue_head_t done;                   /* compression done */
	size_t unc_len;                           /* uncompressed length */
	size_t cmp_len;                           /* compressed length */
	unsigned char unc[LZO_UNC_SIZE];          /* uncompressed buffer */
	unsigned char cmp[LZO_CMP_SIZE];          /* compressed buffer */
	unsigned char wrk[LZO1X_1_MEM_COMPRESS];  /* compression workspace */
	int affinity_cpu;
};

/**
 * Compression function that runs in its own thread.
 */
static int lzo_compress_threadfn(void *data)
{
	struct cmp_data *d = data;
	
	set_kernel_thread_prop(d->affinity_cpu, lzo_thread_prio);
	while (1) {
		wait_event(d->go, atomic_read(&d->ready) ||
		                  kthread_should_stop());
		if (kthread_should_stop()) {
			d->thr = NULL;
			d->ret = -1;
			atomic_set(&d->stop, 1);
			wake_up(&d->done);
			break;
		}
		atomic_set(&d->ready, 0);

		d->ret = lzo1x_1_compress(d->unc, d->unc_len,
		                          d->cmp + LZO_HEADER, &d->cmp_len,
		                          d->wrk);
		atomic_set(&d->stop, 1);
		wake_up(&d->done);
	}
	return 0;
}

#if	(SAVE_RESERVE_HOLE==1)
extern unsigned int asoc_gpu_start, asoc_gpu_size;
extern unsigned int asoc_ion_start, asoc_ion_size;
extern unsigned int asoc_fb_start, asoc_fb_size;

static int reserve_start;
static int reserve_end;
static int reserve_pos;

static int reserve_page_save_prepare(void)
{
	int nr;
	reserve_pos = reserve_start = asoc_gpu_start;
	reserve_end = asoc_gpu_start + asoc_gpu_size;
	nr = ((reserve_end-reserve_start)/PAGE_SIZE);
	if(nr == 0)
		return 0;

	return nr;
}

static void reserve_page_save_end(void)
{
    
}

static int reserve_page_next(unsigned char **pageAddr)
{
	if(reserve_pos < reserve_end)
	{
		*pageAddr = (unsigned char*)ioremap(reserve_pos, PAGE_SIZE);
		reserve_pos += PAGE_SIZE;
		return PAGE_SIZE;
	}
	return 0;
}

static void reserve_page_commit(unsigned char *pageAddr)
{
    iounmap(pageAddr);
}

static int save_reserve_lzo(struct swap_map_handle *handle, unsigned long *swap_pages)
{
	unsigned int m;
	int ret = 0;
	int nr_pages;
	int err2;
	struct bio *bio;
	struct timeval start;
	struct timeval stop;
	size_t off;
	unsigned thr, run_threads, nr_threads;
	unsigned char *page = NULL;
	struct cmp_data *data = NULL;
	unsigned char *savable_page;
	int nr_to_write, swap_wirte_pages = 0;
	/*
	 * We'll limit the number of threads for compression to limit memory
	 * footprint.
	 */

	nr_to_write = reserve_page_save_prepare();
	if(nr_to_write == 0)
		return 0;
		
	nr_threads = num_online_cpus();
	nr_threads = clamp_val(nr_threads, 1, LZO_THREADS);
	printk("nr_threads=%d\n", nr_threads);

	page = (void *)__get_free_page(__GFP_WAIT | __GFP_HIGH);
	if (!page) {
		printk(KERN_ERR "PM: Failed to allocate LZO page\n");
		ret = -ENOMEM;
		goto out_clean;
	}

	data = vmalloc(sizeof(*data) * nr_threads);
	if (!data) {
		printk(KERN_ERR "PM: Failed to allocate LZO data\n");
		ret = -ENOMEM;
		goto out_clean;
	}
	for (thr = 0; thr < nr_threads; thr++)
		memset(&data[thr], 0, offsetof(struct cmp_data, go));

	/*
	 * Start the compression threads.
	 */
	for (thr = 0; thr < nr_threads; thr++) {
		init_waitqueue_head(&data[thr].go);
		init_waitqueue_head(&data[thr].done);

		data[thr].affinity_cpu = thr;
		data[thr].thr = kthread_run(lzo_compress_threadfn,
		                            &data[thr],
		                            "image_compress/%u", thr);
		if (IS_ERR(data[thr].thr)) {
			data[thr].thr = NULL;
			printk(KERN_ERR
			       "PM: Cannot start compression threads\n");
			ret = -ENOMEM;
			goto out_clean;
		}
	}

	/*
	 * Adjust number of free pages after all allocations have been done.
	 * We don't want to run out of pages when writing.
	 */
	handle->reqd_free_pages = reqd_free_pages();
	printk(KERN_INFO
		"PM: Using %u thread(s) for compression.\n"
		"PM: Compressing and saving image data (%u pages) ...     ",
		nr_threads, nr_to_write);
	m = nr_to_write / 100;
	if (!m)
		m = 1;
	nr_pages = 0;
	bio = NULL;
	do_gettimeofday(&start);
	for (;;) {
		for (thr = 0; thr < nr_threads; thr++) {
			for (off = 0; off < LZO_UNC_SIZE; off += PAGE_SIZE) {
				ret = reserve_page_next(&savable_page);
				//ret = snapshot_read_next(snapshot);
				if (ret < 0)
					goto out_finish;

				if (!ret)
					break;

				memcpy(data[thr].unc + off, savable_page, PAGE_SIZE);
                reserve_page_commit(savable_page);
                
				if (!(nr_pages % m))
					printk(KERN_CONT "\b\b\b\b%3d%%",
				               nr_pages / m);
				nr_pages++;
			}
			if (!off)
				break;

			data[thr].unc_len = off;

			atomic_set(&data[thr].ready, 1);
			wake_up(&data[thr].go);
		}

		if (!thr)
			break;
		for (run_threads = thr, thr = 0; thr < run_threads; thr++) {
			wait_event(data[thr].done,
			           atomic_read(&data[thr].stop));
			atomic_set(&data[thr].stop, 0);

			ret = data[thr].ret;

			if (ret < 0) {
				printk(KERN_ERR "PM: LZO compression failed\n");
				goto out_finish;
			}

			if (unlikely(!data[thr].cmp_len ||
			             data[thr].cmp_len >
			             lzo1x_worst_compress(data[thr].unc_len))) {
				printk(KERN_ERR
				       "PM: Invalid LZO compressed length\n");
				ret = -1;
				goto out_finish;
			}

			*(size_t *)data[thr].cmp = data[thr].cmp_len;

			/*
			 * Given we are writing one page at a time to disk, we
			 * copy that much from the buffer, although the last
			 * bit will likely be smaller than full page. This is
			 * OK - we saved the length of the compressed data, so
			 * any garbage at the end will be discarded when we
			 * read it.
			 */
			for (off = 0;
			     off < LZO_HEADER + data[thr].cmp_len;
			     off += PAGE_SIZE) {
				memcpy(page, data[thr].cmp + off, PAGE_SIZE);

				ret = swap_write_page(handle, page, &bio, NULL);
				if (ret)
					goto out_finish;
				swap_wirte_pages++;
			}
		}
	}

out_finish:
	*swap_pages = swap_wirte_pages;
	err2 = hib_wait_on_bio_chain(&bio);
	do_gettimeofday(&stop);
	if (!ret)
		ret = err2;
	if (!ret) {
		printk(KERN_CONT "\b\b\b\bdone\n");
	} else {
		printk(KERN_CONT "\n");
	}
	swsusp_show_speed(&start, &stop, nr_to_write, "Wrote");
out_clean:
	if (data) {
		for (thr = 0; thr < nr_threads; thr++)
			if (data[thr].thr)
				kthread_stop(data[thr].thr);
		vfree(data);
	}
	if (page) free_page((unsigned long)page);

	reserve_page_save_end();
	return ret;
}
#else
#define save_reserve_lzo(v1, v2)	0
#endif

/**
 * save_image_lzo - Save the suspend image data compressed with LZO.
 * @handle: Swap mam handle to use for saving the image.
 * @snapshot: Image to read data from.
 * @nr_to_write: Number of pages to save.
 */
static int save_image_lzo(struct swap_map_handle *handle,
                          struct snapshot_handle *snapshot,
                          unsigned long *swap_pages,
                          unsigned int nr_to_write)
{
	unsigned int m;
	int ret = 0;
	int nr_pages;
	int err2;
	struct bio *bio;
	struct timeval start;
	struct timeval stop;
	size_t off;
	unsigned thr, run_threads, nr_threads;
	unsigned char *page = NULL;
	struct cmp_data *data = NULL;
#if ENABLE_CRC
	struct crc_data *crc = NULL;
#endif
	int swap_wirte_pages = 0;
	/*
	 * We'll limit the number of threads for compression to limit memory
	 * footprint.
	 */
#if ENABLE_CRC
	nr_threads = num_online_cpus() - 1;
#else
	nr_threads = num_online_cpus();
#endif
	nr_threads = clamp_val(nr_threads, 1, LZO_THREADS);
	printk("nr_threads=%d\n", nr_threads);

	page = (void *)__get_free_page(__GFP_WAIT | __GFP_HIGH);
	if (!page) {
		printk(KERN_ERR "PM: Failed to allocate LZO page\n");
		ret = -ENOMEM;
		goto out_clean;
	}

	data = vmalloc(sizeof(*data) * nr_threads);
	if (!data) {
		printk(KERN_ERR "PM: Failed to allocate LZO data\n");
		ret = -ENOMEM;
		goto out_clean;
	}
	for (thr = 0; thr < nr_threads; thr++)
		memset(&data[thr], 0, offsetof(struct cmp_data, go));

#if ENABLE_CRC
	crc = kmalloc(sizeof(*crc), GFP_KERNEL);
	if (!crc) {
		printk(KERN_ERR "PM: Failed to allocate crc\n");
		ret = -ENOMEM;
		goto out_clean;
	}
	memset(crc, 0, offsetof(struct crc_data, go));
#endif

	/*
	 * Start the compression threads.
	 */
	for (thr = 0; thr < nr_threads; thr++) {
		init_waitqueue_head(&data[thr].go);
		init_waitqueue_head(&data[thr].done);

		data[thr].affinity_cpu = thr;
		data[thr].thr = kthread_run(lzo_compress_threadfn,
		                            &data[thr],
		                            "image_compress/%u", thr);
		if (IS_ERR(data[thr].thr)) {
			data[thr].thr = NULL;
			printk(KERN_ERR
			       "PM: Cannot start compression threads\n");
			ret = -ENOMEM;
			goto out_clean;
		}
	}

	/*
	 * Adjust number of free pages after all allocations have been done.
	 * We don't want to run out of pages when writing.
	 */
	handle->reqd_free_pages = reqd_free_pages();

#if ENABLE_CRC
	/*
	 * Start the CRC32 thread.
	 */
	init_waitqueue_head(&crc->go);
	init_waitqueue_head(&crc->done);

	handle->crc32 = 0;
	crc->crc32 = &handle->crc32;
	for (thr = 0; thr < nr_threads; thr++) {
		crc->unc[thr] = data[thr].unc;
		crc->unc_len[thr] = &data[thr].unc_len;
	}

	crc->thr = kthread_run(crc32_threadfn, crc, "image_crc32");
	if (IS_ERR(crc->thr)) {
		crc->thr = NULL;
		printk(KERN_ERR "PM: Cannot start CRC32 thread\n");
		ret = -ENOMEM;
		goto out_clean;
	}
#endif

	printk(KERN_INFO
		"PM: Using %u thread(s) for compression.\n"
		"PM: Compressing and saving image data (%u pages) ...     ",
		nr_threads, nr_to_write);
	m = nr_to_write / 100;
	if (!m)
		m = 1;
	nr_pages = 0;
	bio = NULL;
	do_gettimeofday(&start);
	for (;;) {
		for (thr = 0; thr < nr_threads; thr++) {
			for (off = 0; off < LZO_UNC_SIZE; off += PAGE_SIZE) {
				ret = snapshot_read_next(snapshot);
				if (ret < 0)
					goto out_finish;

				if (!ret)
					break;

				memcpy(data[thr].unc + off,
				       data_of(*snapshot), PAGE_SIZE);

				if (!(nr_pages % m))
					printk(KERN_CONT "\b\b\b\b%3d%%",
				               nr_pages / m);
				nr_pages++;
			}
			if (!off)
				break;

			data[thr].unc_len = off;

			atomic_set(&data[thr].ready, 1);
			wake_up(&data[thr].go);
		}

		if (!thr)
			break;
#if ENABLE_CRC
		crc->run_threads = thr;
		atomic_set(&crc->ready, 1);
		wake_up(&crc->go);
#endif
		for (run_threads = thr, thr = 0; thr < run_threads; thr++) {
			wait_event(data[thr].done,
			           atomic_read(&data[thr].stop));
			atomic_set(&data[thr].stop, 0);

			ret = data[thr].ret;

			if (ret < 0) {
				printk(KERN_ERR "PM: LZO compression failed\n");
				goto out_finish;
			}

			if (unlikely(!data[thr].cmp_len ||
			             data[thr].cmp_len >
			             lzo1x_worst_compress(data[thr].unc_len))) {
				printk(KERN_ERR
				       "PM: Invalid LZO compressed length\n");
				ret = -1;
				goto out_finish;
			}

			*(size_t *)data[thr].cmp = data[thr].cmp_len;

			/*
			 * Given we are writing one page at a time to disk, we
			 * copy that much from the buffer, although the last
			 * bit will likely be smaller than full page. This is
			 * OK - we saved the length of the compressed data, so
			 * any garbage at the end will be discarded when we
			 * read it.
			 */
			for (off = 0;
			     off < LZO_HEADER + data[thr].cmp_len;
			     off += PAGE_SIZE) {
				memcpy(page, data[thr].cmp + off, PAGE_SIZE);

				ret = swap_write_page(handle, page, &bio, NULL);
				if (ret)
					goto out_finish;
				swap_wirte_pages++;
			}
		}
#if ENABLE_CRC
		wait_event(crc->done, atomic_read(&crc->stop));
		atomic_set(&crc->stop, 0);
#endif
	}

out_finish:
	*swap_pages = swap_wirte_pages;
	err2 = hib_wait_on_bio_chain(&bio);
	do_gettimeofday(&stop);
	if (!ret)
		ret = err2;
	if (!ret) {
		printk(KERN_CONT "\b\b\b\bdone\n");
	} else {
		printk(KERN_CONT "\n");
	}
	swsusp_show_speed(&start, &stop, nr_to_write, "Wrote");
out_clean:
#if ENABLE_CRC
	if (crc) {
		if (crc->thr)
			kthread_stop(crc->thr);
		kfree(crc);
	}
#endif
	if (data) {
		for (thr = 0; thr < nr_threads; thr++)
			if (data[thr].thr)
				kthread_stop(data[thr].thr);
		vfree(data);
	}
	if (page) free_page((unsigned long)page);

	return ret;
}

/**
 *	enough_swap - Make sure we have enough swap to save the image.
 *
 *	Returns TRUE or FALSE after checking the total amount of swap
 *	space avaiable from the resume partition.
 */

static int enough_swap(unsigned int nr_pages, unsigned int flags)
{
	unsigned int free_swap = count_swap_pages(root_swap, 1);
	unsigned int required;

	pr_debug("PM: Free swap pages: %u\n", free_swap);

	required = PAGES_FOR_IO + nr_pages;
	return free_swap > required;
}

/**
 *	swsusp_write - Write entire image and metadata.
 *	@flags: flags to pass to the "boot" kernel in the image header
 *
 *	It is important _NOT_ to umount filesystems at this point. We want
 *	them synced (in case something goes wrong) but we DO not want to mark
 *	filesystem clean: it is not. (And it does not matter, if we resume
 *	correctly, we'll mark system clean, anyway.)
 */

int swsusp_write(unsigned int flags)
{
	struct swap_map_handle handle;
	struct snapshot_handle snapshot;
	struct swsusp_info *header = NULL;
	unsigned long pages, swap_pages;
	sector_t offset_save;
	int error;

	pages = snapshot_get_image_size();
	error = get_swap_writer(&handle);
	if (error) {
		printk(KERN_ERR "PM: Cannot get swap writer\n");
		return error;
	}
	if (flags & SF_NOCOMPRESS_MODE) {
		if (!enough_swap(pages, flags)) {
			printk(KERN_ERR "PM: Not enough free swap\n");
			error = -ENOSPC;
			goto out_finish;
		}
	}
	memset(&snapshot, 0, sizeof(struct snapshot_handle));
	error = snapshot_read_next(&snapshot);
	if (error < PAGE_SIZE) {
		if (error >= 0)
			error = -EFAULT;

		goto out_finish;
	}
	
	header = (struct swsusp_info *)kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (header == NULL) {
		goto out_finish;
	}
	memcpy(header, (char*)data_of(snapshot), sizeof(struct swsusp_info));
	error = swap_write_page(&handle, header, NULL, &offset_save);
	if (!error) {
		error = ((flags & SF_NOCOMPRESS_MODE) ?
			save_image(&handle, &snapshot, pages - 1) :
			save_image_lzo(&handle, &snapshot, &swap_pages, pages - 1));
		header->swap_image_pages = ((flags & SF_NOCOMPRESS_MODE) ? 
			pages - 1 : swap_pages);
		
		save_reserve_lzo(&handle, &swap_pages);
		header->swap_reserve_pages = swap_pages;

		printk("swap_image_pages=%d, swap_reserve_pages=%d\n", header->swap_image_pages, header->swap_reserve_pages);
	}
out_finish:
	error = swap_writer_finish(&handle, flags, error);
	if (header != NULL) {
		if(error == 0) {
			hib_bio_write_page(offset_save, header, NULL);
		}
		kfree(header);
	}
	return error;
}

/**
 *	The following functions allow us to read data using a swap map
 *	in a file-alike way
 */

static void release_swap_reader(struct swap_map_handle *handle)
{
	struct swap_map_page_list *tmp;

	while (handle->maps) {
		if (handle->maps->map)
			free_page((unsigned long)handle->maps->map);
		tmp = handle->maps;
		handle->maps = handle->maps->next;
		kfree(tmp);
	}
	handle->cur = NULL;
}

static int get_swap_reader(struct swap_map_handle *handle,
		unsigned int *flags_p)
{
	int error;
	struct swap_map_page_list *tmp, *last;
	sector_t offset;

	*flags_p = swsusp_header->flags;

	if (!swsusp_header->image) /* how can this happen? */
		return -EINVAL;

	handle->cur = NULL;
	last = handle->maps = NULL;
	offset = swsusp_header->image;
	while (offset) {
		tmp = kmalloc(sizeof(*handle->maps), GFP_KERNEL);
		if (!tmp) {
			release_swap_reader(handle);
			return -ENOMEM;
		}
		memset(tmp, 0, sizeof(*tmp));
		if (!handle->maps)
			handle->maps = tmp;
		if (last)
			last->next = tmp;
		last = tmp;

		tmp->map = (struct swap_map_page *)
		           __get_free_page(__GFP_WAIT | __GFP_HIGH);
		if (!tmp->map) {
			release_swap_reader(handle);
			return -ENOMEM;
		}

		error = hib_bio_read_page(offset, tmp->map, NULL);
		if (error) {
			release_swap_reader(handle);
			return error;
		}
		offset = tmp->map->next_swap;
	}
	handle->k = 0;
	handle->cur = handle->maps->map;
	return 0;
}

static int swap_read_page(struct swap_map_handle *handle, void *buf,
				struct bio **bio_chain)
{
	sector_t offset;
	int error;
	struct swap_map_page_list *tmp;

	if (!handle->cur)
		return -EINVAL;
	offset = handle->cur->entries[handle->k];
	if (!offset)
		return -EFAULT;
	error = hib_bio_read_page(offset, buf, bio_chain);
	if (error)
		return error;
	if (++handle->k >= MAP_PAGE_ENTRIES) {
		handle->k = 0;
		free_page((unsigned long)handle->maps->map);
		tmp = handle->maps;
		handle->maps = handle->maps->next;
		kfree(tmp);
		if (!handle->maps)
			release_swap_reader(handle);
		else
			handle->cur = handle->maps->map;
	}
	return error;
}

static int swap_reader_finish(struct swap_map_handle *handle)
{
	release_swap_reader(handle);

	return 0;
}

/**
 *	load_image - load the image using the swap map handle
 *	@handle and the snapshot handle @snapshot
 *	(assume there are @nr_pages pages to load)
 */

static int load_image(struct swap_map_handle *handle,
                      struct snapshot_handle *snapshot,
                      unsigned int nr_to_read)
{
	unsigned int m;
	int ret = 0;
	struct timeval start;
	struct timeval stop;
	struct bio *bio;
	int err2;
	unsigned nr_pages;

	printk(KERN_INFO "PM: Loading image data pages (%u pages) ...     ",
		nr_to_read);
	m = nr_to_read / 100;
	if (!m)
		m = 1;
	nr_pages = 0;
	bio = NULL;
	do_gettimeofday(&start);
	for ( ; ; ) {
		ret = snapshot_write_next(snapshot);
		if (ret <= 0)
			break;
		ret = swap_read_page(handle, data_of(*snapshot), &bio);
		if (ret)
			break;
		if (snapshot->sync_read)
			ret = hib_wait_on_bio_chain(&bio);
		if (ret)
			break;
		if (!(nr_pages % m))
			printk("\b\b\b\b%3d%%", nr_pages / m);
		nr_pages++;
	}
	err2 = hib_wait_on_bio_chain(&bio);
	do_gettimeofday(&stop);
	if (!ret)
		ret = err2;
	if (!ret) {
		printk("\b\b\b\bdone\n");
		snapshot_write_finalize(snapshot);
		if (!snapshot_image_loaded(snapshot))
			ret = -ENODATA;
	} else
		printk("\n");
	swsusp_show_speed(&start, &stop, nr_to_read, "Read");
	return ret;
}

/**
 * Structure used for LZO data decompression.
 */
struct dec_data {
	struct task_struct *thr;                  /* thread */
	atomic_t ready;                           /* ready to start flag */
	atomic_t stop;                            /* ready to stop flag */
	int ret;                                  /* return code */
	wait_queue_head_t go;                     /* start decompression */
	wait_queue_head_t done;                   /* decompression done */
	size_t unc_len;                           /* uncompressed length */
	size_t cmp_len;                           /* compressed length */
	unsigned char unc[LZO_UNC_SIZE];          /* uncompressed buffer */
	unsigned char cmp[LZO_CMP_SIZE];          /* compressed buffer */
	int affinity_cpu;
};

/**
 * Deompression function that runs in its own thread.
 */
static int lzo_decompress_threadfn(void *data)
{
	struct dec_data *d = data;
	
	set_kernel_thread_prop(d->affinity_cpu, lzo_thread_prio);
	while (1) {
		wait_event(d->go, atomic_read(&d->ready) ||
		                  kthread_should_stop());
		if (kthread_should_stop()) {
			d->thr = NULL;
			d->ret = -1;
			atomic_set(&d->stop, 1);
			wake_up(&d->done);
			break;
		}
		atomic_set(&d->ready, 0);

		d->unc_len = LZO_UNC_SIZE;
		d->ret = lzo1x_decompress_safe(d->cmp + LZO_HEADER, d->cmp_len,
		                               d->unc, &d->unc_len);
		atomic_set(&d->stop, 1);
		wake_up(&d->done);
	}
	return 0;
}

#if	(SAVE_RESERVE_HOLE==1)
static int load_reserve_lzo(struct swap_map_handle *handle, unsigned long swap_pages)
{
	unsigned int m;
	int ret = 0;
	int eof = 0;
	struct bio *bio;
	struct timeval start;
	struct timeval stop;
	unsigned nr_pages;
	size_t off;
	unsigned i, thr, run_threads, nr_threads;
	unsigned ring = 0, pg = 0, ring_size = 0,
	         have = 0, want, need, asked = 0;
	unsigned long read_pages;
	unsigned char **page = NULL;
	struct dec_data *data = NULL;
	unsigned char *savable_page;
	int nr_to_read, real_read_pages = 0;
	
	nr_to_read = reserve_page_save_prepare();
	if(nr_to_read == 0)
		return 0;

	/*
	 * We'll limit the number of threads for decompression to limit memory
	 * footprint.
	 */
	nr_threads = num_online_cpus();
	nr_threads = clamp_val(nr_threads, 1, LZO_THREADS);

	page = vmalloc(sizeof(*page) * LZO_READ_PAGES);
	if (!page) {
		printk(KERN_ERR "PM: Failed to allocate LZO page\n");
		ret = -ENOMEM;
		goto out_clean;
	}

	data = vmalloc(sizeof(*data) * nr_threads);
	if (!data) {
		printk(KERN_ERR "PM: Failed to allocate LZO data\n");
		ret = -ENOMEM;
		goto out_clean;
	}
	for (thr = 0; thr < nr_threads; thr++)
		memset(&data[thr], 0, offsetof(struct dec_data, go));
	/*
	 * Start the decompression threads.
	 */
	for (thr = 0; thr < nr_threads; thr++) {
		init_waitqueue_head(&data[thr].go);
		init_waitqueue_head(&data[thr].done);

		data[thr].affinity_cpu = thr;
		data[thr].thr = kthread_run(lzo_decompress_threadfn,
		                            &data[thr],
		                            "image_decompress/%u", thr);
		if (IS_ERR(data[thr].thr)) {
			data[thr].thr = NULL;
			printk(KERN_ERR
			       "PM: Cannot start decompression threads\n");
			ret = -ENOMEM;
			goto out_clean;
		}
	}

	/*
	 * Adjust number of pages for read buffering, in case we are short.
	 */
	read_pages = nr_free_pages() >> 1;
	read_pages = clamp_val(read_pages, LZO_CMP_PAGES, LZO_READ_PAGES);

	for (i = 0; i < read_pages; i++) {
		page[i] = (void *)__get_free_page(i < LZO_CMP_PAGES ?
		                                  __GFP_WAIT | __GFP_HIGH :
		                                  __GFP_WAIT);
		if (!page[i]) {
			if (i < LZO_CMP_PAGES) {
				ring_size = i;
				printk(KERN_ERR
				       "PM: Failed to allocate LZO pages\n");
				ret = -ENOMEM;
				goto out_clean;
			} else {
				break;
			}
		}
	}
	want = ring_size = i;

	printk(KERN_INFO
		"PM: Using %u thread(s) for decompression.\n"
		"PM: Loading and decompressing image data (%u pages) ...     ",
		nr_threads, nr_to_read);
	m = nr_to_read / 100;
	if (!m)
		m = 1;
	nr_pages = 0;
	bio = NULL;
	do_gettimeofday(&start);

	ret = reserve_page_next(&savable_page);
	if (ret <= 0)
		goto out_finish;

	for(;;) {
		for (i = 0; !eof && i < want; i++) {
			if(real_read_pages < swap_pages)
				ret = swap_read_page(handle, page[ring], &bio);
			else {
				eof = 1;
				break;
			}
			real_read_pages++;
			
			if (ret) {
				/*
				 * On real read error, finish. On end of data,
				 * set EOF flag and just exit the read loop.
				 */
				if (handle->cur &&
				    handle->cur->entries[handle->k]) {
					goto out_finish;
				} else {
					eof = 1;
					break;
				}
			}
			if (++ring >= ring_size)
				ring = 0;
		}
		asked += i;
		want -= i;

		/*
		 * We are out of data, wait for some more.
		 */
		if (!have) {
			if (!asked)
				break;

			ret = hib_wait_on_bio_chain(&bio);
			if (ret)
				goto out_finish;
			have += asked;
			asked = 0;
			if (eof)
				eof = 2;
		}
		for (thr = 0; have && thr < nr_threads; thr++) {
			data[thr].cmp_len = *(size_t *)page[pg];
			if (unlikely(!data[thr].cmp_len ||
			             data[thr].cmp_len >
			             lzo1x_worst_compress(LZO_UNC_SIZE))) {
				printk(KERN_ERR
				       "PM: Invalid LZO compressed length\n");
				ret = -1;
				goto out_finish;
			}

			need = DIV_ROUND_UP(data[thr].cmp_len + LZO_HEADER,
			                    PAGE_SIZE);
			if (need > have) {
				if (eof > 1) {
					ret = -1;
					goto out_finish;
				}
				break;
			}

			for (off = 0;
			     off < LZO_HEADER + data[thr].cmp_len;
			     off += PAGE_SIZE) {
				memcpy(data[thr].cmp + off,
				       page[pg], PAGE_SIZE);
				have--;
				want++;
				if (++pg >= ring_size)
					pg = 0;
			}

			atomic_set(&data[thr].ready, 1);
			wake_up(&data[thr].go);
		}

		/*
		 * Wait for more data while we are decompressing.
		 */
		if (have < LZO_CMP_PAGES && asked) {
			ret = hib_wait_on_bio_chain(&bio);
			if (ret)
				goto out_finish;
			have += asked;
			asked = 0;
			if (eof)
				eof = 2;
		}

		for (run_threads = thr, thr = 0; thr < run_threads; thr++) {
			wait_event(data[thr].done,
			           atomic_read(&data[thr].stop));
			atomic_set(&data[thr].stop, 0);

			ret = data[thr].ret;

			if (ret < 0) {
				printk(KERN_ERR
				       "PM: LZO decompression failed\n");
				goto out_finish;
			}

			if (unlikely(!data[thr].unc_len ||
			             data[thr].unc_len > LZO_UNC_SIZE ||
			             data[thr].unc_len & (PAGE_SIZE - 1))) {
				printk(KERN_ERR
				       "PM: Invalid LZO uncompressed length\n");
				ret = -1;
				goto out_finish;
			}

			for (off = 0;
			     off < data[thr].unc_len; off += PAGE_SIZE) {
				memcpy(savable_page, data[thr].unc + off, PAGE_SIZE);
                reserve_page_commit(savable_page);

				if (!(nr_pages % m))
					printk("\b\b\b\b%3d%%", nr_pages / m);
				nr_pages++;

				ret = reserve_page_next(&savable_page);
				if (ret <= 0) {
					goto out_finish;
				}
			}
		}
	}

out_finish:
	do_gettimeofday(&stop);
	if (!ret) {
		printk("\b\b\b\bdone\n");
	} else
		printk("\n");
	swsusp_show_speed(&start, &stop, nr_to_read, "Read");
out_clean:
	for (i = 0; i < ring_size; i++)
		free_page((unsigned long)page[i]);
	if (data) {
		for (thr = 0; thr < nr_threads; thr++)
			if (data[thr].thr)
				kthread_stop(data[thr].thr);
		vfree(data);
	}
	if (page) vfree(page);
	reserve_page_save_end();
	return ret;
}

#else
#define load_reserve_lzo(v1, v2)	0
#endif

/**
 * load_image_lzo - Load compressed image data and decompress them with LZO.
 * @handle: Swap map handle to use for loading data.
 * @snapshot: Image to copy uncompressed data into.
 * @nr_to_read: Number of pages to load.
 */
static int load_image_lzo(struct swap_map_handle *handle,
                          struct snapshot_handle *snapshot,
                          unsigned long swap_pages,
                          unsigned int nr_to_read)
{
	unsigned int m;
	int ret = 0;
	int eof = 0;
	struct bio *bio;
	struct timeval start;
	struct timeval stop;
	unsigned nr_pages;
	size_t off;
	unsigned i, thr, run_threads, nr_threads;
	unsigned ring = 0, pg = 0, ring_size = 0,
	         have = 0, want, need, asked = 0;
	unsigned long read_pages;
	unsigned char **page = NULL;
	struct dec_data *data = NULL;
#if ENABLE_CRC
	struct crc_data *crc = NULL;
#endif
	int real_read_pages = 0;
	/*
	 * We'll limit the number of threads for decompression to limit memory
	 * footprint.
	 */
#if ENABLE_CRC
	nr_threads = num_online_cpus() - 1;
#else
	nr_threads = num_online_cpus();
#endif
	nr_threads = clamp_val(nr_threads, 1, LZO_THREADS);

	page = vmalloc(sizeof(*page) * LZO_READ_PAGES);
	if (!page) {
		printk(KERN_ERR "PM: Failed to allocate LZO page\n");
		ret = -ENOMEM;
		goto out_clean;
	}

	data = vmalloc(sizeof(*data) * nr_threads);
	if (!data) {
		printk(KERN_ERR "PM: Failed to allocate LZO data\n");
		ret = -ENOMEM;
		goto out_clean;
	}
	for (thr = 0; thr < nr_threads; thr++)
		memset(&data[thr], 0, offsetof(struct dec_data, go));
#if ENABLE_CRC
	crc = kmalloc(sizeof(*crc), GFP_KERNEL);
	if (!crc) {
		printk(KERN_ERR "PM: Failed to allocate crc\n");
		ret = -ENOMEM;
		goto out_clean;
	}
	memset(crc, 0, offsetof(struct crc_data, go));
#endif
	/*
	 * Start the decompression threads.
	 */
	for (thr = 0; thr < nr_threads; thr++) {
		init_waitqueue_head(&data[thr].go);
		init_waitqueue_head(&data[thr].done);

		data[thr].affinity_cpu = thr;
		data[thr].thr = kthread_run(lzo_decompress_threadfn,
		                            &data[thr],
		                            "image_decompress/%u", thr);
		if (IS_ERR(data[thr].thr)) {
			data[thr].thr = NULL;
			printk(KERN_ERR
			       "PM: Cannot start decompression threads\n");
			ret = -ENOMEM;
			goto out_clean;
		}
	}

#if ENABLE_CRC
	/*
	 * Start the CRC32 thread.
	 */
	init_waitqueue_head(&crc->go);
	init_waitqueue_head(&crc->done);

	handle->crc32 = 0;
	crc->crc32 = &handle->crc32;
	for (thr = 0; thr < nr_threads; thr++) {
		crc->unc[thr] = data[thr].unc;
		crc->unc_len[thr] = &data[thr].unc_len;
	}

	crc->thr = kthread_run(crc32_threadfn, crc, "image_crc32");
	if (IS_ERR(crc->thr)) {
		crc->thr = NULL;
		printk(KERN_ERR "PM: Cannot start CRC32 thread\n");
		ret = -ENOMEM;
		goto out_clean;
	}
#endif

	/*
	 * Adjust number of pages for read buffering, in case we are short.
	 */
	read_pages = (nr_free_pages() - snapshot_get_image_size()) >> 1;
	read_pages = clamp_val(read_pages, LZO_CMP_PAGES, LZO_READ_PAGES);

	for (i = 0; i < read_pages; i++) {
		page[i] = (void *)__get_free_page(i < LZO_CMP_PAGES ?
		                                  __GFP_WAIT | __GFP_HIGH :
		                                  __GFP_WAIT);
		if (!page[i]) {
			if (i < LZO_CMP_PAGES) {
				ring_size = i;
				printk(KERN_ERR
				       "PM: Failed to allocate LZO pages\n");
				ret = -ENOMEM;
				goto out_clean;
			} else {
				break;
			}
		}
	}
	want = ring_size = i;

	printk(KERN_INFO
		"PM: Using %u thread(s) for decompression.\n"
		"PM: Loading and decompressing image data (%u pages) ...     ",
		nr_threads, nr_to_read);
	m = nr_to_read / 100;
	if (!m)
		m = 1;
	nr_pages = 0;
	bio = NULL;
	do_gettimeofday(&start);

	ret = snapshot_write_next(snapshot);
	if (ret <= 0)
		goto out_finish;

	for(;;) {
		for (i = 0; !eof && i < want; i++) {
			if(real_read_pages < swap_pages)
				ret = swap_read_page(handle, page[ring], &bio);
			else {
				eof = 1;
				break;
			}
			real_read_pages++;
			
			if (ret) {
				/*
				 * On real read error, finish. On end of data,
				 * set EOF flag and just exit the read loop.
				 */
				if (handle->cur &&
				    handle->cur->entries[handle->k]) {
					goto out_finish;
				} else {
					eof = 1;
					break;
				}
			}
			if (++ring >= ring_size)
				ring = 0;
		}
		asked += i;
		want -= i;

		/*
		 * We are out of data, wait for some more.
		 */
		if (!have) {
			if (!asked)
				break;

			ret = hib_wait_on_bio_chain(&bio);
			if (ret)
				goto out_finish;
			have += asked;
			asked = 0;
			if (eof)
				eof = 2;
		}
#if ENABLE_CRC
		if (crc->run_threads) {
			wait_event(crc->done, atomic_read(&crc->stop));
			atomic_set(&crc->stop, 0);
			crc->run_threads = 0;
		}
#endif
		for (thr = 0; have && thr < nr_threads; thr++) {
			data[thr].cmp_len = *(size_t *)page[pg];
			if (unlikely(!data[thr].cmp_len ||
			             data[thr].cmp_len >
			             lzo1x_worst_compress(LZO_UNC_SIZE))) {
				printk(KERN_ERR
				       "PM: Invalid LZO compressed length\n");
				ret = -1;
				goto out_finish;
			}

			need = DIV_ROUND_UP(data[thr].cmp_len + LZO_HEADER,
			                    PAGE_SIZE);
			if (need > have) {
				if (eof > 1) {
					ret = -1;
					goto out_finish;
				}
				break;
			}

			for (off = 0;
			     off < LZO_HEADER + data[thr].cmp_len;
			     off += PAGE_SIZE) {
				memcpy(data[thr].cmp + off,
				       page[pg], PAGE_SIZE);
				have--;
				want++;
				if (++pg >= ring_size)
					pg = 0;
			}

			atomic_set(&data[thr].ready, 1);
			wake_up(&data[thr].go);
		}

		/*
		 * Wait for more data while we are decompressing.
		 */
		if (have < LZO_CMP_PAGES && asked) {
			ret = hib_wait_on_bio_chain(&bio);
			if (ret)
				goto out_finish;
			have += asked;
			asked = 0;
			if (eof)
				eof = 2;
		}

		for (run_threads = thr, thr = 0; thr < run_threads; thr++) {
			wait_event(data[thr].done,
			           atomic_read(&data[thr].stop));
			atomic_set(&data[thr].stop, 0);

			ret = data[thr].ret;

			if (ret < 0) {
				printk(KERN_ERR
				       "PM: LZO decompression failed\n");
				goto out_finish;
			}

			if (unlikely(!data[thr].unc_len ||
			             data[thr].unc_len > LZO_UNC_SIZE ||
			             data[thr].unc_len & (PAGE_SIZE - 1))) {
				printk(KERN_ERR
				       "PM: Invalid LZO uncompressed length\n");
				ret = -1;
				goto out_finish;
			}

			for (off = 0;
			     off < data[thr].unc_len; off += PAGE_SIZE) {
				memcpy(data_of(*snapshot),
				       data[thr].unc + off, PAGE_SIZE);

				if (!(nr_pages % m))
					printk("\b\b\b\b%3d%%", nr_pages / m);
				nr_pages++;

				ret = snapshot_write_next(snapshot);
				if (ret <= 0) {
#if ENABLE_CRC
					crc->run_threads = thr + 1;
					atomic_set(&crc->ready, 1);
					wake_up(&crc->go);
#endif
					goto out_finish;
				}
			}
		}
#if ENABLE_CRC
		crc->run_threads = thr;
		atomic_set(&crc->ready, 1);
		wake_up(&crc->go);
#endif
	}

out_finish:
#if ENABLE_CRC
	if (crc->run_threads) {
		wait_event(crc->done, atomic_read(&crc->stop));
		atomic_set(&crc->stop, 0);
	}
#endif
	do_gettimeofday(&stop);
	if (!ret) {
		printk("\b\b\b\bdone\n");
		snapshot_write_finalize(snapshot);
		if (!snapshot_image_loaded(snapshot))
			ret = -ENODATA;
#if ENABLE_CRC
		if (!ret) {
			if (swsusp_header->flags & SF_CRC32_MODE) {
				if(handle->crc32 != swsusp_header->crc32) {
					printk(KERN_ERR
					       "PM: Invalid image CRC32!\n");
					ret = -ENODATA;
				}
			}
		}
#endif
	} else
		printk("\n");
	swsusp_show_speed(&start, &stop, nr_to_read, "Read");
out_clean:
	for (i = 0; i < ring_size; i++)
		free_page((unsigned long)page[i]);
#if ENABLE_CRC
	if (crc) {
		if (crc->thr)
			kthread_stop(crc->thr);
		kfree(crc);
	}
#endif
	if (data) {
		for (thr = 0; thr < nr_threads; thr++)
			if (data[thr].thr)
				kthread_stop(data[thr].thr);
		vfree(data);
	}
	if (page) vfree(page);

	return ret;
}

/**
 *	swsusp_read - read the hibernation image.
 *	@flags_p: flags passed by the "frozen" kernel in the image header should
 *		  be written into this memory location
 */

int swsusp_read(unsigned int *flags_p)
{
	int error;
	struct swap_map_handle handle;
	struct snapshot_handle snapshot;
	struct swsusp_info *header;
	unsigned long swap_image_pages, swap_reserve_pages;
	
	memset(&snapshot, 0, sizeof(struct snapshot_handle));
	error = snapshot_write_next(&snapshot);
	if (error < PAGE_SIZE)
		return error < 0 ? error : -EFAULT;
	header = (struct swsusp_info *)data_of(snapshot);
	error = get_swap_reader(&handle, flags_p);
	if (error)
		goto end;
	if (!error)
		error = swap_read_page(&handle, header, NULL);
	if (!error) {
		swap_image_pages = header->swap_image_pages;
		swap_reserve_pages = header->swap_reserve_pages;
		printk("swap_image_pages=%d, swap_reserve_pages=%d\n", swap_image_pages, swap_reserve_pages);
		
		error = ((*flags_p & SF_NOCOMPRESS_MODE) ?
			load_image(&handle, &snapshot, header->pages - 1) :
			load_image_lzo(&handle, &snapshot, swap_image_pages, header->pages - 1));
			
		load_reserve_lzo(&handle, swap_reserve_pages);
	}
	swap_reader_finish(&handle);
end:
	if (!error)
		pr_debug("PM: Image successfully loaded\n");
	else
		pr_debug("PM: Error %d resuming\n", error);
	return error;
}

/**
 *      swsusp_check - Check for swsusp signature in the resume device
 */

int swsusp_check(int swsup_flag)
{
	int error;

	hib_resume_bdev = blkdev_get_by_dev(swsusp_resume_device,
					    FMODE_READ, NULL);
	if (!IS_ERR(hib_resume_bdev)) {
		set_blocksize(hib_resume_bdev, PAGE_SIZE);
		clear_page(swsusp_header);
		error = hib_bio_read_page(swsusp_resume_block,
					swsusp_header, NULL);
		if (error)
			goto put;

		printk("HIBERNATE_SIG=%s\n", HIBERNATE_SIG);
		printk("swsusp_header->sig=%s\n", swsusp_header->sig);
		if (!memcmp(HIBERNATE_SIG, swsusp_header->sig, 10)) {
			memcpy(swsusp_header->sig, swsusp_header->orig_sig, 10);
			/* Reset swap signature now */
			if(swsup_flag == 0)
				error = hib_bio_write_page(swsusp_resume_block,
							swsusp_header, NULL);
		} else {
			error = -EINVAL;
		}

put:
		if (error)
			blkdev_put(hib_resume_bdev, FMODE_READ);
		else
			pr_debug("PM: Image signature found, resuming\n");
	} else {
		error = PTR_ERR(hib_resume_bdev);
	}

	if (error)
		pr_debug("PM: Image not found (code %d)\n", error);

	return error;
}

/**
 *	swsusp_close - close swap device.
 */

void swsusp_close(fmode_t mode)
{
	if (IS_ERR(hib_resume_bdev)) {
		pr_debug("PM: Image device not initialised\n");
		return;
	}

	blkdev_put(hib_resume_bdev, mode);
}

static int swsusp_header_init(void)
{
	swsusp_header = (struct swsusp_header*) __get_free_page(GFP_KERNEL);
	if (!swsusp_header)
		panic("Could not allocate memory for swsusp_header\n");
	return 0;
}

core_initcall(swsusp_header_init);

#else
/*
 * linux/kernel/power/swap.c
 *
 * This file provides functions for reading the suspend image from
 * and writing it to a swap partition.
 *
 * Copyright (C) 1998,2001-2005 Pavel Machek <pavel@ucw.cz>
 * Copyright (C) 2006 Rafael J. Wysocki <rjw@sisk.pl>
 * Copyright (C) 2010 Bojan Smojver <bojan@rexursive.com>
 *
 * This file is released under the GPLv2.
 *
 */


/*
 *	The swap map is a data structure used for keeping track of each page
 *	written to a swap partition.  It consists of many swap_map_page
 *	structures that contain each an array of MAP_PAGE_ENTRIES swap entries.
 *	These structures are stored on the swap and linked together with the
 *	help of the .next_swap member.
 *
 *	The swap map is created during suspend.  The swap map pages are
 *	allocated and populated one at a time, so we only need one memory
 *	page to set up the entire structure.
 *
 *	During resume we pick up all swap_map_page structures into a list.
 */

#define MAP_PAGE_ENTRIES	(PAGE_SIZE / sizeof(sector_t) - 1)

struct swsusp_header {
	char reserved[PAGE_SIZE - 20 - sizeof(sector_t) - sizeof(int) -
	              sizeof(u32)];
	u32	crc32;
	sector_t image;
	unsigned int flags;	/* Flags to pass to the "boot" kernel */
	char	orig_sig[10];
	char	sig[10];
} __attribute__((packed));

#define SWSUSP_RESUME_LBA 			0
#define STORAGE_SECTOR_SIZE 			512  //nand flash one sector has 512 bytes
#define SECTORS_PER_BLOCK 				(PAGE_SIZE / STORAGE_SECTOR_SIZE)

typedef struct {
	struct block_device *bdev;
	struct inode *bdev_inode;
	struct gendisk *disk;
	unsigned int cur_lba;
} swapdisk_handle_t;

static int get_swapdisk(swapdisk_handle_t *handle)
{
	struct block_device *bdev;
	int ret;

	handle->bdev = NULL;
	
	if(swsusp_resume_device == 0) {
		printk(KERN_ERR "PM: swsusp_resume_device not special, set to default: 93:80\n");
		swsusp_resume_device = MKDEV(93, 80);
	}
	
	printk("PM: Hibernation image partition %d:%d present\n",
		MAJOR(swsusp_resume_device), MINOR(swsusp_resume_device));

	bdev = bdget(swsusp_resume_device);
	if(bdev == NULL) {
		printk(KERN_ERR "bdget failed\n");
		return -ENODEV;
	}

	ret = blkdev_get(bdev, FMODE_WRITE | FMODE_READ, NULL);
	if(ret) {
		bdput(bdev);
		printk(KERN_ERR "blkdev_get failed\n");
		return -ENODEV;
	}
		
  	if(bdev->bd_disk == NULL  || bdev->bd_inode == NULL) {
  		blkdev_put(bdev, FMODE_WRITE | FMODE_READ);
		printk(KERN_ERR "PM: Cannot get swap device\n");
		return -ENODEV;
	}

	handle->bdev = bdev;
	handle->bdev_inode = bdev->bd_inode;
	handle->disk = bdev->bd_disk;
	handle->cur_lba = SWSUSP_RESUME_LBA + SECTORS_PER_BLOCK;

	return 0;
}

static int put_swapdisk(swapdisk_handle_t *handle)
{
	if(handle->bdev) {
		handle->disk->fops->flush_disk_cache();
		blkdev_put(handle->bdev, FMODE_WRITE | FMODE_READ);
		handle->bdev = NULL;
	}
	return 0;
}

static int check_swapdisk(swapdisk_handle_t *handle)
{
	if(handle->bdev && handle->disk->fops->ioctl) {
		handle->disk->fops->ioctl(handle->bdev, FMODE_WRITE | FMODE_READ, SELF_CHECK, 1);
	}
	return 0;
}

static int swapdisk_write_page(swapdisk_handle_t *handle, void *addr)
{
	unsigned int ret;

	ret = handle->disk->fops->blk_write(handle->cur_lba, SECTORS_PER_BLOCK, addr, handle->bdev_inode);
	handle->cur_lba += SECTORS_PER_BLOCK;

	if( ret == 0 ) 
		return 0;
	else {
		printk(KERN_ERR "PM: swapdisk_write_page failed\n");
		return -EIO;
	}
}


static int swapdisk_write_head_page(swapdisk_handle_t *handle, void *addr)
{
	unsigned int ret;

	ret = handle->disk->fops->blk_write(SWSUSP_RESUME_LBA, SECTORS_PER_BLOCK, addr, handle->bdev_inode);
	
	if( ret == 0 ) 
		return 0;
	else {
		printk(KERN_ERR "PM: swapdisk_write_page failed\n");
		return -EIO;
	}
	return 0;
}


static int swapdisk_read_page(swapdisk_handle_t *handle, void *addr)
{
	unsigned int ret;

	ret = handle->disk->fops->blk_read(handle->cur_lba, SECTORS_PER_BLOCK, addr, handle->bdev_inode);
	handle->cur_lba += SECTORS_PER_BLOCK;
	
	if( ret == 0 ) 
		return 0;
	else {
		printk(KERN_ERR "PM: swapdisk_read_page failed\n");
		return -EIO;
	}
}


static int swapdisk_read_head_page(swapdisk_handle_t *handle, void *addr)
{
	unsigned int ret;

	ret = handle->disk->fops->blk_read(SWSUSP_RESUME_LBA, SECTORS_PER_BLOCK, addr, handle->bdev_inode);

	if( ret == 0 ) 
		return 0;
	else {
		printk(KERN_ERR "PM: swapdisk_write_page failed\n");
		return -EIO;
	}
}

static int swapdisk_rewind(swapdisk_handle_t *handle)
{
	handle->cur_lba = SWSUSP_RESUME_LBA + SECTORS_PER_BLOCK;
	return 0;
}


#define ASYNC_PAGE_NUM		(16*SZ_1M/PAGE_SIZE)

typedef struct {
	struct task_struct *thr;                  /* thread */
	wait_queue_head_t go;                     /* start compression */
	wait_queue_head_t done;                   /* compression done */
	
	swapdisk_handle_t *handle;
	char *pages[ASYNC_PAGE_NUM];
	atomic_t page_num;
	atomic_t valid_num;
	int output_index;
	int input_index;
} async_thread_t;

static int async_write_threadfn(void *data)
{
	async_thread_t *d = data;
	
	set_kernel_thread_prop(rw_thread_cpu, rw_thread_prio);
	while (1) {
		wait_event(d->go, atomic_read(&d->page_num) ||kthread_should_stop());
		
		while(atomic_read(&d->page_num) > 0) {
			swapdisk_write_page(d->handle, d->pages[d->output_index]);
			d->output_index++;
			if(d->output_index == ASYNC_PAGE_NUM)
				d->output_index = 0;
			atomic_sub(1, &d->page_num);
			wake_up(&d->done);
            if( power_interrupt == 1 ) {
                return 0;
            }
		}

		if (kthread_should_stop()) {
			d->thr = NULL;
			break;
		}
	}
	return 0;
}

static void async_write_page(async_thread_t *d, void *addr)
{
	wait_event(d->done, atomic_read(&d->page_num) <ASYNC_PAGE_NUM );
	
	memcpy(d->pages[d->input_index], addr, PAGE_SIZE);
	atomic_add(1, &d->page_num);
	wake_up(&d->go);
	d->input_index++;
	if(d->input_index == ASYNC_PAGE_NUM)
		d->input_index = 0;
}

static int async_read_threadfn(void *data)
{
	async_thread_t *d = data;
	
	set_kernel_thread_prop(rw_thread_cpu, rw_thread_prio);
	while (1) {
		wait_event(d->go, (atomic_read(&d->page_num) <ASYNC_PAGE_NUM && atomic_read(&d->valid_num) > 0) 
			||kthread_should_stop());
		
		while(atomic_read(&d->page_num) <ASYNC_PAGE_NUM && atomic_read(&d->valid_num) > 0) {
			swapdisk_read_page(d->handle, d->pages[d->input_index]);
			d->input_index++;
			if(d->input_index == ASYNC_PAGE_NUM)
				d->input_index = 0;
			atomic_add(1, &d->page_num);
			atomic_sub(1, &d->valid_num);
			wake_up(&d->done);
		}

		if (kthread_should_stop()) {
			d->thr = NULL;
			break;
		}
	}
	return 0;
}

static int async_read_page(async_thread_t *d, void *addr)
{
	wait_event(d->done, atomic_read(&d->page_num) || atomic_read(&d->valid_num) == 0 );
	if(atomic_read(&d->page_num) > 0) {
		memcpy(addr, d->pages[d->output_index], PAGE_SIZE);
		atomic_sub(1, &d->page_num);
		wake_up(&d->go);
		d->output_index++;
		if(d->output_index == ASYNC_PAGE_NUM)
			d->output_index = 0;
		return 0;
	}
	else
		return -1;
}


static async_thread_t *async_thread_create(swapdisk_handle_t *handle, char mode, int readn)
{
	int i;
	async_thread_t *async_thread = NULL;

	async_thread = vmalloc(sizeof(async_thread_t));
	if(async_thread == NULL)
		goto end;
	memset(async_thread, 0, sizeof(async_thread_t));
	
	init_waitqueue_head(&async_thread->go);
	init_waitqueue_head(&async_thread->done);
	async_thread->handle = handle;

	for(i = 0; i < ASYNC_PAGE_NUM; i++) {
		async_thread->pages[i] = kmalloc(PAGE_SIZE, GFP_KERNEL);
		if(async_thread->pages[i] == NULL) 
			goto end;
	}

	if(mode == 'w')
		async_thread->thr = kthread_run(async_write_threadfn, async_thread, "async_write_threadfn");
	else {
		atomic_set(&async_thread->valid_num, readn);
		async_thread->thr = kthread_run(async_read_threadfn, async_thread, "async_read_threadfn");
	}
	if (IS_ERR(async_thread->thr)) {
		printk(KERN_ERR "PM: Cannot start async_write threads\n");
		goto end;
	}

	return async_thread;
	
end:
	printk(KERN_ERR "PM: Failed to allocate async_write_thread data\n");
	if(async_thread != NULL) {
		for(i = 0; i < ASYNC_PAGE_NUM; i++) {
			if(async_thread->pages[i] != NULL)
				kfree(async_thread->pages[i]);
		}
		vfree(async_thread);
	}
	return NULL;
}

static void async_thread_delete(async_thread_t *async_thread)
{
	int i;

	if(async_thread != NULL) {
		kthread_stop(async_thread->thr);
		for(i = 0; i < ASYNC_PAGE_NUM; i++) {
			if(async_thread->pages[i] != NULL)
				kfree(async_thread->pages[i]);
		}
		vfree(async_thread);
	}
}




/* We need to remember how much compressed data we need to read. */
#define LZO_HEADER	sizeof(size_t)

/* Number of pages/bytes we'll compress at one time. */
#define LZO_UNC_PAGES	32
#define LZO_UNC_SIZE	(LZO_UNC_PAGES * PAGE_SIZE)

/* Number of pages/bytes we need for compressed data (worst case). */
#define LZO_CMP_PAGES	DIV_ROUND_UP(lzo1x_worst_compress(LZO_UNC_SIZE) + \
			             LZO_HEADER, PAGE_SIZE)
#define LZO_CMP_SIZE	(LZO_CMP_PAGES * PAGE_SIZE)

/* Maximum number of threads for compression/decompression. */
#define LZO_THREADS	3

/* Maximum number of pages for read buffering. */
#define LZO_READ_PAGES	(MAP_PAGE_ENTRIES * 8)


/**
 * Structure used for LZO data compression.
 */
struct cmp_data {
	struct task_struct *thr;                  /* thread */
	atomic_t ready;                           /* ready to start flag */
	atomic_t stop;                            /* ready to stop flag */
	int ret;                                  /* return code */
	wait_queue_head_t go;                     /* start compression */
	wait_queue_head_t done;                   /* compression done */
	size_t unc_len;                           /* uncompressed length */
	size_t cmp_len;                           /* compressed length */
	unsigned char unc[LZO_UNC_SIZE];          /* uncompressed buffer */
	unsigned char cmp[LZO_CMP_SIZE];          /* compressed buffer */
	unsigned char wrk[LZO1X_1_MEM_COMPRESS];  /* compression workspace */
	int affinity_cpu;
};

/**
 * Compression function that runs in its own thread.
 */
static int lzo_compress_threadfn(void *data)
{
	struct cmp_data *d = data;
	
	set_kernel_thread_prop(d->affinity_cpu, lzo_thread_prio);
	while (1) {
		wait_event(d->go, atomic_read(&d->ready) ||
		                  kthread_should_stop());
		if (kthread_should_stop()) {
			d->thr = NULL;
			d->ret = -1;
			atomic_set(&d->stop, 1);
			wake_up(&d->done);
			break;
		}
		atomic_set(&d->ready, 0);

		d->ret = lzo1x_1_compress(d->unc, d->unc_len,
		                          d->cmp + LZO_HEADER, &d->cmp_len,
		                          d->wrk);
		atomic_set(&d->stop, 1);
		wake_up(&d->done);
	}
	return 0;
}

#if	(SAVE_RESERVE_HOLE==1)
extern unsigned int asoc_gpu_start, asoc_gpu_size;
extern unsigned int asoc_ion_start, asoc_ion_size;
extern unsigned int asoc_fb_start, asoc_fb_size;

static int reserve_start;
static int reserve_end;
static int reserve_pos;

static int reserve_page_save_prepare(void)
{
	int nr;
	reserve_pos = reserve_start = asoc_gpu_start;
	reserve_end = asoc_gpu_start + asoc_gpu_size;
	nr = ((reserve_end-reserve_start)/PAGE_SIZE);
	if(nr == 0)
		return 0;

	return nr;
}

static void reserve_page_save_end(void)
{
    
}

static int reserve_page_next(unsigned char **pageAddr)
{
	if(reserve_pos < reserve_end)
	{
		*pageAddr = (unsigned char*)ioremap(reserve_pos, PAGE_SIZE);
		reserve_pos += PAGE_SIZE;
		return PAGE_SIZE;
	}
	return 0;
}

static void reserve_page_commit(unsigned char *pageAddr)
{
    iounmap(pageAddr);
}

static int save_reserve_lzo(async_thread_t *async_thread, unsigned long *swap_pages)
{
	unsigned int m;
	int ret = 0;
	int nr_pages;
	struct bio *bio;
	struct timeval start;
	struct timeval stop;
	size_t off;
	unsigned thr, run_threads, nr_threads;
	struct cmp_data *data = NULL;
	unsigned char *savable_page;
	int nr_to_write, swap_wirte_pages = 0;
	/*
	 * We'll limit the number of threads for compression to limit memory
	 * footprint.
	 */

	nr_to_write = reserve_page_save_prepare();
	if(nr_to_write == 0)
		return 0;
		
	nr_threads = num_online_cpus();
	nr_threads = clamp_val(nr_threads, 1, LZO_THREADS);
	printk("nr_threads=%d\n", nr_threads);

	data = vmalloc(sizeof(*data) * nr_threads);
	if (!data) {
		printk(KERN_ERR "PM: Failed to allocate LZO data\n");
		ret = -ENOMEM;
		goto out_clean;
	}
	for (thr = 0; thr < nr_threads; thr++)
		memset(&data[thr], 0, offsetof(struct cmp_data, go));

	/*
	 * Start the compression threads.
	 */
	for (thr = 0; thr < nr_threads; thr++) {
		init_waitqueue_head(&data[thr].go);
		init_waitqueue_head(&data[thr].done);

		data[thr].affinity_cpu = thr;
		data[thr].thr = kthread_run(lzo_compress_threadfn,
		                            &data[thr],
		                            "image_compress/%u", thr);
		if (IS_ERR(data[thr].thr)) {
			data[thr].thr = NULL;
			printk(KERN_ERR
			       "PM: Cannot start compression threads\n");
			ret = -ENOMEM;
			goto out_clean;
		}
	}

	printk(KERN_INFO
		"PM: Using %u thread(s) for compression.\n"
		"PM: Compressing and saving image data (%u pages) ...     ",
		nr_threads, nr_to_write);
	m = nr_to_write / 100;
	if (!m)
		m = 1;
	nr_pages = 0;
	bio = NULL;
	do_gettimeofday(&start);
	for (;;) {
		for (thr = 0; thr < nr_threads; thr++) {
			for (off = 0; off < LZO_UNC_SIZE; off += PAGE_SIZE) {
				ret = reserve_page_next(&savable_page);
				//ret = snapshot_read_next(snapshot);
				if (ret < 0)
					goto out_finish;

				if (!ret)
					break;

				memcpy(data[thr].unc + off, savable_page, PAGE_SIZE);
                reserve_page_commit(savable_page);
                
				if (!(nr_pages % m))
					printk(KERN_CONT "\b\b\b\b%3d%%",
				               nr_pages / m);
				nr_pages++;
			}
			if (!off)
				break;

			data[thr].unc_len = off;

			atomic_set(&data[thr].ready, 1);
			wake_up(&data[thr].go);
		}

		if (!thr)
			break;
		for (run_threads = thr, thr = 0; thr < run_threads; thr++) {
			wait_event(data[thr].done,
			           atomic_read(&data[thr].stop));
			atomic_set(&data[thr].stop, 0);

			ret = data[thr].ret;

			if (ret < 0) {
				printk(KERN_ERR "PM: LZO compression failed\n");
				goto out_finish;
			}

			if (unlikely(!data[thr].cmp_len ||
			             data[thr].cmp_len >
			             lzo1x_worst_compress(data[thr].unc_len))) {
				printk(KERN_ERR
				       "PM: Invalid LZO compressed length\n");
				ret = -1;
				goto out_finish;
			}

			*(size_t *)data[thr].cmp = data[thr].cmp_len;

			/*
			 * Given we are writing one page at a time to disk, we
			 * copy that much from the buffer, although the last
			 * bit will likely be smaller than full page. This is
			 * OK - we saved the length of the compressed data, so
			 * any garbage at the end will be discarded when we
			 * read it.
			 */
			for (off = 0;
			     off < LZO_HEADER + data[thr].cmp_len;
			     off += PAGE_SIZE) {
				async_write_page(async_thread, data[thr].cmp + off);
				swap_wirte_pages++;
			}
		}
	}

out_finish:
	*swap_pages = swap_wirte_pages;
	do_gettimeofday(&stop);
	if (!ret) {
		printk(KERN_CONT "\b\b\b\bdone\n");
	} else {
		printk(KERN_CONT "\n");
	}
	swsusp_show_speed(&start, &stop, nr_to_write, "Wrote");
out_clean:
	if (data) {
		for (thr = 0; thr < nr_threads; thr++)
			if (data[thr].thr)
				kthread_stop(data[thr].thr);
		vfree(data);
	}
	reserve_page_save_end();
	return ret;
}
#else
#define save_reserve_lzo(v1, v2)	0
#endif

/**
 * save_image_lzo - Save the suspend image data compressed with LZO.
 * @handle: Swap mam handle to use for saving the image.
 * @snapshot: Image to read data from.
 * @nr_to_write: Number of pages to save.
 */
static int save_image_lzo(async_thread_t *async_thread,
                          struct snapshot_handle *snapshot,
                          unsigned long *swap_pages,
                          unsigned int nr_to_write)
{
	unsigned int m;
	int ret = 0;
	int nr_pages;
	struct bio *bio;
	struct timeval start;
	struct timeval stop;
	size_t off;
	unsigned thr, run_threads, nr_threads;
	struct cmp_data *data = NULL;
	int swap_wirte_pages = 0;
	/*
	 * We'll limit the number of threads for compression to limit memory
	 * footprint.
	 */
	nr_threads = num_online_cpus();
	nr_threads = clamp_val(nr_threads, 1, LZO_THREADS);
	printk("nr_threads=%d\n", nr_threads);

	data = vmalloc(sizeof(*data) * nr_threads);
	if (!data) {
		printk(KERN_ERR "PM: Failed to allocate LZO data\n");
		ret = -ENOMEM;
		goto out_clean;
	}
	for (thr = 0; thr < nr_threads; thr++)
		memset(&data[thr], 0, offsetof(struct cmp_data, go));

	/*
	 * Start the compression threads.
	 */
	for (thr = 0; thr < nr_threads; thr++) {
		init_waitqueue_head(&data[thr].go);
		init_waitqueue_head(&data[thr].done);

		data[thr].affinity_cpu = thr;
		data[thr].thr = kthread_run(lzo_compress_threadfn,
		                            &data[thr],
		                            "image_compress/%u", thr);
		if (IS_ERR(data[thr].thr)) {
			data[thr].thr = NULL;
			printk(KERN_ERR
			       "PM: Cannot start compression threads\n");
			ret = -ENOMEM;
			goto out_clean;
		}
	}

	printk(KERN_INFO
		"PM: Using %u thread(s) for compression.\n"
		"PM: Compressing and saving image data (%u pages) ...     ",
		nr_threads, nr_to_write);
	m = nr_to_write / 100;
	if (!m)
		m = 1;
	nr_pages = 0;
	bio = NULL;
	do_gettimeofday(&start);
	for (;;) {
		for (thr = 0; thr < nr_threads; thr++) {
			for (off = 0; off < LZO_UNC_SIZE; off += PAGE_SIZE) {
				ret = snapshot_read_next(snapshot);
				if (ret < 0)
					goto out_finish;

				if (!ret)
					break;

				memcpy(data[thr].unc + off,
				       data_of(*snapshot), PAGE_SIZE);

				if (!(nr_pages % m))
					printk(KERN_CONT "\b\b\b\b%3d%%",
				               nr_pages / m);
				nr_pages++;
			}
			if (!off)
				break;

			data[thr].unc_len = off;

			atomic_set(&data[thr].ready, 1);
			wake_up(&data[thr].go);
		}

		if (!thr)
			break;
		for (run_threads = thr, thr = 0; thr < run_threads; thr++) {
			wait_event(data[thr].done,
			           atomic_read(&data[thr].stop));
			atomic_set(&data[thr].stop, 0);

			ret = data[thr].ret;

			if (ret < 0) {
				printk(KERN_ERR "PM: LZO compression failed\n");
				goto out_finish;
			}

			if (unlikely(!data[thr].cmp_len ||
			             data[thr].cmp_len >
			             lzo1x_worst_compress(data[thr].unc_len))) {
				printk(KERN_ERR
				       "PM: Invalid LZO compressed length\n");
				ret = -1;
				goto out_finish;
			}

			*(size_t *)data[thr].cmp = data[thr].cmp_len;

			/*
			 * Given we are writing one page at a time to disk, we
			 * copy that much from the buffer, although the last
			 * bit will likely be smaller than full page. This is
			 * OK - we saved the length of the compressed data, so
			 * any garbage at the end will be discarded when we
			 * read it.
			 */
			for (off = 0;
			     off < LZO_HEADER + data[thr].cmp_len;
			     off += PAGE_SIZE) {
		        if( hibernate_power_long_press() == 1 ) {
		            power_interrupt = 1;
    				ret = -1;
    				goto out_finish;
		        }
				async_write_page(async_thread, data[thr].cmp + off);
				swap_wirte_pages++;
			}
		}
	}

out_finish:
	*swap_pages = swap_wirte_pages;
	do_gettimeofday(&stop);
	if (!ret) {
		printk(KERN_CONT "\b\b\b\bdone\n");
	} else {
		printk(KERN_CONT "\n");
	}
	swsusp_show_speed(&start, &stop, nr_to_write, "Wrote");
out_clean:
	if (data) {
		for (thr = 0; thr < nr_threads; thr++)
			if (data[thr].thr)
				kthread_stop(data[thr].thr);
		vfree(data);
	}

	return ret;
}


/**
 *	swsusp_write - Write entire image and metadata.
 *	@flags: flags to pass to the "boot" kernel in the image header
 *
 *	It is important _NOT_ to umount filesystems at this point. We want
 *	them synced (in case something goes wrong) but we DO not want to mark
 *	filesystem clean: it is not. (And it does not matter, if we resume
 *	correctly, we'll mark system clean, anyway.)
 */
extern unsigned int    pa_swsusp_save_sp;
extern unsigned int    pa_stext;
extern unsigned int    pa_etext;
extern unsigned int    pa_sdata;
extern unsigned int    pa_edata;
extern unsigned int    phy_cpu_reset;
extern unsigned int    phy_cpu_resume;
extern unsigned int    pa_nosave_begin;
extern unsigned int    pa_nosave_end;

void restore_resume_data(struct swsusp_info *header)
{
	void *nosave;
	
	pa_swsusp_save_sp = header->pa_swsusp_save_sp;
	pa_stext = header->pa_stext;
	pa_etext = header->pa_etext;
	pa_sdata = header->pa_sdata;
	pa_edata = header->pa_edata;
	phy_cpu_reset = header->phy_cpu_reset;
	phy_cpu_resume = header->phy_cpu_resume;
	pa_nosave_begin = header->pa_nosave_begin;
	pa_nosave_end = header->pa_nosave_end;
	nosave = ioremap(pa_nosave_begin, pa_nosave_end - pa_nosave_begin);
	memset(nosave, 0, pa_nosave_end - pa_nosave_begin);
	iounmap(nosave);
}

void save_resume_data(struct swsusp_info *header)
{
	header->pa_swsusp_save_sp = pa_swsusp_save_sp;
	header->pa_stext = pa_stext;
	header->pa_etext = pa_etext;
	header->pa_sdata = pa_sdata;
	header->pa_edata = pa_edata;
	header->phy_cpu_reset = phy_cpu_reset;
	header->phy_cpu_resume = phy_cpu_resume;
	header->pa_nosave_begin = pa_nosave_begin;
	header->pa_nosave_end = pa_nosave_end;
}

int swsusp_write(unsigned int flags)
{
	swapdisk_handle_t handle;
	async_thread_t *async_thread;
	struct snapshot_handle snapshot;
	struct swsusp_info *header = NULL;
	unsigned long pages, swap_pages;
	int error;

	error = get_swapdisk(&handle);
	if(error)
		goto out_finish;

    check_swapdisk(&handle);
    
	pages = snapshot_get_image_size();
	memset(&snapshot, 0, sizeof(struct snapshot_handle));
	error = snapshot_read_next(&snapshot);
	if (error < PAGE_SIZE) {
		if (error >= 0)
			error = -EFAULT;
		goto out_finish;
	}
	
	header = (struct swsusp_info *)kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (header == NULL) {
		goto out_finish;
	}

	async_thread = async_thread_create(&handle, 'w', 0);
	if(async_thread == NULL) {
		goto out_finish;
	}
	
	set_kernel_thread_prop(main_thread_cpu, main_thread_prio);

	memcpy(header, (char*)data_of(snapshot), sizeof(struct swsusp_info));
	async_write_page(async_thread, header);
	
	error = save_image_lzo(async_thread, &snapshot, &swap_pages, pages - 1);
	header->swap_image_pages = swap_pages;

	if (!error) {
		error = save_reserve_lzo(async_thread, &swap_pages);
		header->swap_reserve_pages = swap_pages;
	}
	
	printk("swap_image_pages=%d, swap_reserve_pages=%d\n", header->swap_image_pages, header->swap_reserve_pages);

	async_thread_delete(async_thread);

	
out_finish:
	if (header != NULL) {
		if(error == 0) {
			struct swsusp_header *swapdisk_header;
			swapdisk_header = (struct swsusp_header *)kmalloc(PAGE_SIZE, GFP_KERNEL);
			if(swapdisk_header == NULL)
				return -ENOMEM;
				
			save_resume_data(header);
		       
			swapdisk_rewind(&handle);
			swapdisk_write_page(&handle, header);

			memcpy(swapdisk_header->sig, HIBERNATE_SIG, 10);
			memcpy(swapdisk_header->orig_sig,"SWAPSPACE3", 10);
			printk(KERN_INFO "PM: S");
			swapdisk_write_head_page(&handle, swapdisk_header);
			printk("|\n");

			kfree(swapdisk_header);
		}
		kfree(header);
	}

	put_swapdisk(&handle);
	return error;
}




/**
 * Structure used for LZO data decompression.
 */
struct dec_data {
	struct task_struct *thr;                  /* thread */
	atomic_t ready;                           /* ready to start flag */
	atomic_t stop;                            /* ready to stop flag */
	int ret;                                  /* return code */
	wait_queue_head_t go;                     /* start decompression */
	wait_queue_head_t done;                   /* decompression done */
	size_t unc_len;                           /* uncompressed length */
	size_t cmp_len;                           /* compressed length */
	unsigned char unc[LZO_UNC_SIZE];          /* uncompressed buffer */
	unsigned char cmp[LZO_CMP_SIZE];          /* compressed buffer */
	int affinity_cpu;
};

/**
 * Deompression function that runs in its own thread.
 */

static int lzo_decompress_threadfn(void *data)
{
	struct dec_data *d = data;

	set_kernel_thread_prop(d->affinity_cpu, lzo_thread_prio);
	
	while (1) {
		wait_event(d->go, atomic_read(&d->ready) ||
		                  kthread_should_stop());
		if (kthread_should_stop()) {
			d->thr = NULL;
			d->ret = -1;
			atomic_set(&d->stop, 1);
			wake_up(&d->done);
			break;
		}
		atomic_set(&d->ready, 0);

		d->unc_len = LZO_UNC_SIZE;
		d->ret = lzo1x_decompress_safe(d->cmp + LZO_HEADER, d->cmp_len,
		                               d->unc, &d->unc_len);
		atomic_set(&d->stop, 1);
		wake_up(&d->done);
	}
	return 0;
}

#if	(SAVE_RESERVE_HOLE==1)
static int load_reserve_lzo(async_thread_t *async_thread, unsigned long swap_pages)
{
	unsigned int m;
	int ret = 0;
	struct timeval start;
	struct timeval stop;
	unsigned nr_pages;
	size_t off;
	unsigned thr, run_threads, nr_threads;
	struct dec_data *data = NULL;
	unsigned char *savable_page;
	int nr_to_read, real_read_pages = 0;
	
	nr_to_read = reserve_page_save_prepare();
	if(nr_to_read == 0)
		return 0;

	/*
	 * We'll limit the number of threads for decompression to limit memory
	 * footprint.
	 */
	nr_threads = num_online_cpus();
	nr_threads = clamp_val(nr_threads, 1, LZO_THREADS);

	data = vmalloc(sizeof(*data) * nr_threads);
	if (!data) {
		printk(KERN_ERR "PM: Failed to allocate LZO data\n");
		ret = -ENOMEM;
		goto out_clean;
	}
	for (thr = 0; thr < nr_threads; thr++)
		memset(&data[thr], 0, offsetof(struct dec_data, go));
	/*
	 * Start the decompression threads.
	 */
	for (thr = 0; thr < nr_threads; thr++) {
		init_waitqueue_head(&data[thr].go);
		init_waitqueue_head(&data[thr].done);

		data[thr].affinity_cpu = thr;
		data[thr].thr = kthread_run(lzo_decompress_threadfn,
		                            &data[thr],
		                            "image_decompress/%u", thr);
		if (IS_ERR(data[thr].thr)) {
			data[thr].thr = NULL;
			printk(KERN_ERR
			       "PM: Cannot start decompression threads\n");
			ret = -ENOMEM;
			goto out_clean;
		}
	}

	printk(KERN_INFO
		"PM: Using %u thread(s) for decompression.\n"
		"PM: Loading and decompressing image data (%u pages) ...     ",
		nr_threads, nr_to_read);
	m = nr_to_read / 100;
	if (!m)
		m = 1;
	nr_pages = 0;
	do_gettimeofday(&start);

	ret = reserve_page_next(&savable_page);
	if (ret <= 0)
		goto out_finish;

	for(;;) {
		for (thr = 0; thr < nr_threads; thr++) {
			for (off = 0; off < LZO_HEADER + data[thr].cmp_len; off += PAGE_SIZE) {
    			if(real_read_pages < swap_pages) {
        		    async_read_page(async_thread, data[thr].cmp + off);
        			real_read_pages++;
        		}
    			else
    				break;
    			
    			if(off == 0) {
    			    data[thr].cmp_len = *(size_t *)data[thr].cmp;
        			if (unlikely(!data[thr].cmp_len ||
        			             data[thr].cmp_len >
        			             lzo1x_worst_compress(LZO_UNC_SIZE))) {
        				printk(KERN_ERR
        				       "PM: Invalid LZO compressed length\n");
        				ret = -1;
        				goto out_finish;
        			}
    			}
			}
            
			atomic_set(&data[thr].ready, 1);
			wake_up(&data[thr].go);
		}

		for (run_threads = thr, thr = 0; thr < run_threads; thr++) {
			wait_event(data[thr].done,
			           atomic_read(&data[thr].stop));
			atomic_set(&data[thr].stop, 0);

			ret = data[thr].ret;

			if (ret < 0) {
				printk(KERN_ERR
				       "PM: LZO decompression failed\n");
				goto out_finish;
			}

			if (unlikely(!data[thr].unc_len ||
			             data[thr].unc_len > LZO_UNC_SIZE ||
			             data[thr].unc_len & (PAGE_SIZE - 1))) {
				printk(KERN_ERR
				       "PM: Invalid LZO uncompressed length\n");
				ret = -1;
				goto out_finish;
			}

			for (off = 0;
			     off < data[thr].unc_len; off += PAGE_SIZE) {
				memcpy(savable_page, data[thr].unc + off, PAGE_SIZE);
                reserve_page_commit(savable_page);

				if (!(nr_pages % m))
					printk("\b\b\b\b%3d%%", nr_pages / m);
				nr_pages++;

				ret = reserve_page_next(&savable_page);
				if (ret <= 0) {
					goto out_finish;
				}
			}
		}
	}

out_finish:
	do_gettimeofday(&stop);
	if (!ret) {
		printk("\b\b\b\bdone\n");
	} else
		printk("\n");
	swsusp_show_speed(&start, &stop, nr_to_read, "Read");
out_clean:
	if (data) {
		for (thr = 0; thr < nr_threads; thr++)
			if (data[thr].thr)
				kthread_stop(data[thr].thr);
		vfree(data);
	}
	reserve_page_save_end();
	return ret;
}

#else
#define load_reserve_lzo(v1, v2)	0
#endif

/**
 * load_image_lzo - Load compressed image data and decompress them with LZO.
 * @handle: Swap map handle to use for loading data.
 * @snapshot: Image to copy uncompressed data into.
 * @nr_to_read: Number of pages to load.
 */
static int load_image_lzo(async_thread_t *async_thread,
                          struct snapshot_handle *snapshot,
                          unsigned long swap_pages,
                          unsigned int nr_to_read)
{
	unsigned int m;
	int ret = 0;
	struct timeval start;
	struct timeval stop;
	unsigned nr_pages;
	size_t off;
	unsigned thr, run_threads, nr_threads;
	struct dec_data *data = NULL;
	int real_read_pages = 0;
	
	/*
	 * We'll limit the number of threads for decompression to limit memory
	 * footprint.
	 */
	nr_threads = num_online_cpus();
	nr_threads = clamp_val(nr_threads, 1, LZO_THREADS);

	data = vmalloc(sizeof(*data) * nr_threads);
	if (!data) {
		printk(KERN_ERR "PM: Failed to allocate LZO data\n");
		ret = -ENOMEM;
		goto out_clean;
	}
	for (thr = 0; thr < nr_threads; thr++)
		memset(&data[thr], 0, offsetof(struct dec_data, go));
	/*
	 * Start the decompression threads.
	 */
	for (thr = 0; thr < nr_threads; thr++) {
		init_waitqueue_head(&data[thr].go);
		init_waitqueue_head(&data[thr].done);

		data[thr].affinity_cpu = thr;
		data[thr].thr = kthread_run(lzo_decompress_threadfn,
		                            &data[thr],
		                            "image_decompress/%u", thr);
		if (IS_ERR(data[thr].thr)) {
			data[thr].thr = NULL;
			printk(KERN_ERR
			       "PM: Cannot start decompression threads\n");
			ret = -ENOMEM;
			goto out_clean;
		}
	}

	printk(KERN_INFO
		"PM: Using %u thread(s) for decompression.\n"
		"PM: Loading and decompressing image data (%u pages) ...     ",
		nr_threads, nr_to_read);
	m = nr_to_read / 100;
	if (!m)
		m = 1;
	nr_pages = 0;
	do_gettimeofday(&start);

	ret = snapshot_write_next(snapshot);
	if (ret <= 0)
		goto out_finish;

	for(;;) {
		for (thr = 0; thr < nr_threads; thr++) {
			for (off = 0; off < LZO_HEADER + data[thr].cmp_len; off += PAGE_SIZE) {
    			if(real_read_pages < swap_pages) {
        		    async_read_page(async_thread, data[thr].cmp + off);
        			real_read_pages++;
        		}
    			else
    				break;
    			
    			if(off == 0) {
    			    data[thr].cmp_len = *(size_t *)data[thr].cmp;
        			if (unlikely(!data[thr].cmp_len ||
        			             data[thr].cmp_len >
        			             lzo1x_worst_compress(LZO_UNC_SIZE))) {
        				printk(KERN_ERR
        				       "PM: Invalid LZO compressed length\n");
        				ret = -1;
        				goto out_finish;
        			}
    			}
			}
            
			atomic_set(&data[thr].ready, 1);
			wake_up(&data[thr].go);
		}

		for (run_threads = thr, thr = 0; thr < run_threads; thr++) {
			wait_event(data[thr].done,
			           atomic_read(&data[thr].stop));
			atomic_set(&data[thr].stop, 0);

			ret = data[thr].ret;

			if (ret < 0) {
				printk(KERN_ERR
				       "PM: LZO decompression failed\n");
				goto out_finish;
			}

			if (unlikely(!data[thr].unc_len ||
			             data[thr].unc_len > LZO_UNC_SIZE ||
			             data[thr].unc_len & (PAGE_SIZE - 1))) {
				printk(KERN_ERR
				       "PM: Invalid LZO uncompressed length\n");
				ret = -1;
				goto out_finish;
			}

			for (off = 0;
			     off < data[thr].unc_len; off += PAGE_SIZE) {
				memcpy(data_of(*snapshot),
				       data[thr].unc + off, PAGE_SIZE);

				if (!(nr_pages % m))
					printk("\b\b\b\b%3d%%", nr_pages / m);
				nr_pages++;

				ret = snapshot_write_next(snapshot);
				if (ret <= 0) {
					goto out_finish;
				}
			}
		}
	}

out_finish:
	do_gettimeofday(&stop);
	if (!ret) {
		printk("\b\b\b\bdone\n");
		snapshot_write_finalize(snapshot);
		if (!snapshot_image_loaded(snapshot))
			ret = -ENODATA;
	} else
		printk("\n");
	swsusp_show_speed(&start, &stop, nr_to_read, "Read");
out_clean:
	if (data) {
		for (thr = 0; thr < nr_threads; thr++)
			if (data[thr].thr)
				kthread_stop(data[thr].thr);
		vfree(data);
	}

	return ret;
}

/**
 *	swsusp_read - read the hibernation image.
 *	@flags_p: flags passed by the "frozen" kernel in the image header should
 *		  be written into this memory location
 */

int swsusp_read(unsigned int *flags_p)
{
	int error;
	swapdisk_handle_t handle;
	async_thread_t *async_thread;
	struct snapshot_handle snapshot;
	struct swsusp_info *header;
	unsigned long swap_image_pages, swap_reserve_pages;
	int readn;
	
	memset(&snapshot, 0, sizeof(struct snapshot_handle));
	error = snapshot_write_next(&snapshot);
	if (error < PAGE_SIZE)
		return error < 0 ? error : -EFAULT;
		
	header = (struct swsusp_info *)data_of(snapshot);
	error = get_swapdisk(&handle);
	if (error)
		goto end;

	swapdisk_read_page(&handle, header);
	restore_resume_data(header);

	swap_image_pages = header->swap_image_pages;
	swap_reserve_pages = header->swap_reserve_pages;
	readn = swap_image_pages + swap_reserve_pages;
	printk("swap_image_pages=%d, swap_reserve_pages=%d\n", swap_image_pages, swap_reserve_pages);

	async_thread = async_thread_create(&handle, 'r', readn);
	if(async_thread == NULL) {
		goto end;
	}
	
	set_kernel_thread_prop(main_thread_cpu, main_thread_prio);

	error = load_image_lzo(async_thread, &snapshot, swap_image_pages, header->pages - 1);
	if (!error)
		error = load_reserve_lzo(async_thread, swap_reserve_pages);

	async_thread_delete(async_thread);

end:
	put_swapdisk(&handle);

	if (!error)
		pr_debug("PM: Image successfully loaded\n");
	else
		pr_debug("PM: Error %d resuming\n", error);
	return error;
}

/**
 *      swsusp_check - Check for swsusp signature in the resume device
 */

int swsusp_check(int swsup_flag)
{
	int error = 0;
	swapdisk_handle_t handle;
	struct swsusp_header *swapdisk_header;

	error = get_swapdisk(&handle);
	if (error)
		goto end;
		
	swapdisk_header = (struct swsusp_header *)kmalloc(PAGE_SIZE, GFP_KERNEL);
	if(swapdisk_header == NULL)
		return -ENOMEM;

	swapdisk_read_head_page(&handle, swapdisk_header);
	printk("HIBERNATE_SIG=%s\n", HIBERNATE_SIG);
	printk("swsusp_header->sig=%s\n", swapdisk_header->sig);
	if (!memcmp(HIBERNATE_SIG, swapdisk_header->sig, 10)) {
		memcpy(swapdisk_header->sig, swapdisk_header->orig_sig, 10);
		if(swsup_flag == 0)
			swapdisk_write_head_page(&handle, swapdisk_header);
	} else {
		error = -EINVAL;
	}

	kfree(swapdisk_header);
	if (error)
		pr_debug("PM: Image not found (code %d)\n", error);

end:
	put_swapdisk(&handle);
	return error;
}



#endif
