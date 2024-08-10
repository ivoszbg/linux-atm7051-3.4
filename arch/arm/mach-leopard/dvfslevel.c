/*
 * arch/arm/mach-leopard/dvfslevel.c
 *
 * dvfslevel & icversion for Actions SOC
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
#include <asm/setup.h>
#include <mach/dvfslevel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static int g_dvfslevel = -1;
static int g_icversion = -1;

int asoc_get_dvfslevel(void)
{
    return g_dvfslevel;
}
EXPORT_SYMBOL_GPL(asoc_get_dvfslevel);

int asoc_get_icversion(void)
{
	return g_icversion;
}
EXPORT_SYMBOL_GPL(asoc_get_icversion);

/* parse ATAG_DVFSLEVEL */
static int __init parse_tag_dvfslevel(const struct tag *tag)
{
	g_dvfslevel = tag->u.dvfslevel.dvfslevel;
	g_icversion = tag->u.dvfslevel.icversion;

//	fix_dvfslevel();

	printk("tag: dvfslevel: 0x%x\n", g_dvfslevel);
	printk("tag: icversion: 0x%x\n", g_icversion);

	return 0;
}

__tagtable(ATAG_DVFSLEVEL, parse_tag_dvfslevel);

static int dvfslevel_proc_show(struct seq_file *m, void *v)
{
    switch (ASOC_GET_IC(g_dvfslevel)) {
    case 0x7021:
    case 0x7023:
    case 0x7029:
	    seq_printf(m, "0x%x\n", ASOC_GET_IC(g_dvfslevel));
        break;
    default:
	    seq_printf(m, "unkown\n");
        break;
    }
    
	return 0;
}

static int dvfslevel_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, dvfslevel_proc_show, NULL);
}

static const struct file_operations dvfslevel_proc_fops = {
	.open		= dvfslevel_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_dvfslevel_init(void)
{
	proc_create("dvfslevel", 0, NULL, &dvfslevel_proc_fops);
	return 0;
}
module_init(proc_dvfslevel_init);