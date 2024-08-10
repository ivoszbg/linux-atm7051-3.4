/*
 * arch/arm/mach-leopard/bootafinfo.c
 *
 * bootadfi for Actions SOC
 *
 * Copyright 2012 Actions Semi Inc.
 * Author: Actions Semi, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/module.h> 
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/bootmem.h>
#include <linux/memblock.h>
#include <asm/setup.h>
#include <asm/memblock.h>
#include <asm/idmap.h>
#include <mach/bootafinfo.h>
#include <mach/debug.h>

static unsigned int afinfo_buf_phys;
static int afinfo_buf_len = 0;
static unsigned char *afinfo_buf = NULL;

unsigned char *asoc_get_boot_afinfo(void)
{
    return afinfo_buf;
}
EXPORT_SYMBOL_GPL(asoc_get_boot_afinfo);

int asoc_get_boot_afinfo_len(void)
{
    return afinfo_buf_len;
}
EXPORT_SYMBOL_GPL(asoc_get_boot_afinfo_len);

void __init asoc_boot_afinfo_init(void)
{
	if (!afinfo_buf_phys)
		return;

	afinfo_buf = alloc_bootmem_align(afinfo_buf_len, PAGE_SIZE);

	printk("%s: afinfo_buf 0x%x, afinfo_buf_phys 0x%x, len %x\n", __FUNCTION__,
        afinfo_buf, afinfo_buf_phys, afinfo_buf_len);

	identity_mapping_add(swapper_pg_dir, afinfo_buf_phys, 
            afinfo_buf_phys + PFN_ALIGN(afinfo_buf_len));
	memcpy(afinfo_buf, afinfo_buf_phys, afinfo_buf_len);
	identity_mapping_del(swapper_pg_dir, afinfo_buf_phys, 
            afinfo_buf_phys + PFN_ALIGN(afinfo_buf_len));

	if(afinfo_buf_phys >= PHYS_OFFSET)
	    memblock_free(afinfo_buf_phys, afinfo_buf_len);

//    asoc_dump_mem(afinfo_buf, afinfo_buf_len, 0, 1);
}

void __init asoc_boot_afinfo_reserve(void)
{
	if (!afinfo_buf_phys)
		return;

	if(afinfo_buf_phys >= PHYS_OFFSET) {
    	memblock_reserve(afinfo_buf_phys, afinfo_buf_len);
		printk("memblock_reserve,0x%x:0x%x\n", afinfo_buf_phys, afinfo_buf_len);
    }
}

/* parse ATAG_BOOT_AFINFO*/
static int __init parse_tag_boot_afinfo(const struct tag *tag)
{
    afinfo_buf_phys = tag->u.afinfo.afinfo_buf_start;
    afinfo_buf_len = tag->u.afinfo.afinfo_buf_len;

	printk("tag: boot_afinfo 0x%x, len %d\n", afinfo_buf_phys, afinfo_buf_len);

	return 0;
}
__tagtable(ATAG_BOOT_AFINFO, parse_tag_boot_afinfo);

#if 1
static int bootafinfo_proc_show(struct seq_file *m, void *v)
{
	int i;
    if (afinfo_buf)
    {
    	printk("afi len=%x:\n",afinfo_buf_len);
        //asoc_dump_mem(afinfo_buf, afinfo_buf_len, 0, 1);        
        for ( i = 0 ; i < afinfo_buf_len; i++ )
        	seq_putc(m, (char)afinfo_buf[i]);
    }    
	return 0;
}

static int bootafinfo_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, bootafinfo_proc_show, NULL);
}

static const struct file_operations bootafinfo_proc_fops = {
	.open		= bootafinfo_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_bootafinfo_init(void)
{
	proc_create("bootafinfo", 0, NULL, &bootafinfo_proc_fops);
	return 0;
}
module_init(proc_bootafinfo_init);
#endif
