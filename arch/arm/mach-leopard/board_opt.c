/*
 * arch/arm/mach-leopard/board_opt.c
 *
 * board_opt for Actions SOC
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
#include <linux/proc_fs.h> 
#include <linux/seq_file.h>


static int g_board_opt = 0;
static int g_board_opt_flags = 0;

int asoc_get_board_opt(void)
{
    return g_board_opt;
}
EXPORT_SYMBOL_GPL(asoc_get_board_opt);

int asoc_get_board_opt_flags(void)
{
	return g_board_opt_flags;
}
EXPORT_SYMBOL_GPL(asoc_get_board_opt_flags);

/* parse ATAG_BOARD_OPTION */
static int __init parse_tag_board_opt(const struct tag *tag)
{
	g_board_opt = tag->u.board_opt.board_opt;
	g_board_opt_flags = tag->u.board_opt.board_opt_flags;

	printk("board_opt %d, flags 0x%x\n", g_board_opt, g_board_opt_flags);

	return 0;
}

__tagtable(ATAG_BOARD_OPTION, parse_tag_board_opt);

static int board_opt_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", g_board_opt);
	return 0;
}

static int board_opt_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, board_opt_proc_show, NULL);
}

static const struct file_operations board_opt_proc_fops = {
	.open		= board_opt_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_board_opt_init(void)
{
	proc_create("board_opt", 0, NULL, &board_opt_proc_fops);
	return 0;
}
module_init(proc_board_opt_init);
