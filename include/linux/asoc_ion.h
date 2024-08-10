/*
 * include/linux/asoc_ion.h
 *
 * Copyright 2012 Actions Semi Inc.
 * Author: Actions Semi, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/types.h>
#include <linux/ion.h>

#if !defined(__KERNEL__)
#define __user
#endif

#ifndef _LINUX_ASOC_ION_H
#define _LINUX_ASOC_ION_H

struct asoc_ion_phys_data {
	struct ion_handle *handle;
	unsigned long phys_addr;
	size_t size;
};

/* Custom Ioctl's. */
enum {
	ASOC_ION_GET_PHY = 0,
};

/**
 * These are the only ids that should be used for Ion heap ids.
 * The ids listed are the order in which allocation will be attempted
 * if specified. Don't swap the order of heap ids unless you know what
 * you are doing!
 * Id's are spaced by purpose to allow new Id's to be inserted in-between (for
 * possible fallbacks)
 */

enum ion_heap_ids {
	ION_HEAP_ID_INVALID = -1,
	ION_HEAP_ID_PMEM = 0,
	ION_HEAP_ID_FB = 8,
	ION_HEAP_ID_SYSTEM = 12,
	ION_HEAP_ID_RESERVED = 31 /** Bit reserved for ION_SECURE flag */
};

#endif /* _LINUX_ASOC_ION_H */
