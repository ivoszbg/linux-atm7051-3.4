/*
 *  Copyright (C) 2011, Actions Semiconductor
 *  GL5201 SoC clock support
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <mach/clock.h>


static struct dentry *asoc_clock_debugfs;

static int asoc_clock_debugfs_show_enabled(void *data, uint64_t *value)
{
    struct clk *clk = data;
    *value = clk_is_enabled(clk);

    return 0;
}

static int asoc_clock_debugfs_set_enabled(void *data, uint64_t value)
{
    struct clk *clk = data;

    if (value)
        return clk_enable(clk);
    else
        clk_disable(clk);

    return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(asoc_clock_debugfs_ops_enabled,
    asoc_clock_debugfs_show_enabled,
    asoc_clock_debugfs_set_enabled,
    "%llu\n");

static int asoc_clock_debugfs_show_rate(void *data, uint64_t *value)
{
    struct clk *clk = data;
    *value = clk_get_rate(clk);

    return 0;
}
static int asoc_clock_debugfs_set_rate(void *data, uint64_t value)
{
    struct clk *clk = data;
    return clk_set_rate(clk, value);
}

DEFINE_SIMPLE_ATTRIBUTE(asoc_clock_debugfs_ops_rate,
	asoc_clock_debugfs_show_rate,
	asoc_clock_debugfs_set_rate,
	"%llu\n");

void asoc_clock_debugfs_add_clk(struct clk *clk)
{
    struct clk *parent = NULL;
    
    if (!asoc_clock_debugfs)
        return;
    
    clk->debugfs_entry = debugfs_create_dir(clk->name, asoc_clock_debugfs);
    debugfs_create_file("rate", S_IWUGO | S_IRUGO, clk->debugfs_entry, clk,
                &asoc_clock_debugfs_ops_rate);
    debugfs_create_file("enabled", S_IRUGO, clk->debugfs_entry, clk,
                &asoc_clock_debugfs_ops_enabled);

    parent = clk_get_parent(clk);
    if (parent)
    {
        char parent_path[100];
        snprintf(parent_path, 100, "../%s", parent->name);
        clk->debugfs_parent_entry = debugfs_create_symlink("parent",
                clk->debugfs_entry, parent_path);
	}
}

/* TODO: Locking */
void asoc_clock_debugfs_update_parent(struct clk *clk)
{
    struct clk *parent = NULL;
    
    if (clk->debugfs_parent_entry)
        debugfs_remove(clk->debugfs_parent_entry);

    parent = clk_get_parent(clk);
    if (parent)
    {
        char parent_path[100];
        snprintf(parent_path, 100, "../%s", parent->name);
        clk->debugfs_parent_entry = debugfs_create_symlink("parent",
                    clk->debugfs_entry, parent_path);
    }
    else
    {
        clk->debugfs_parent_entry = NULL;
    }
}

void asoc_clock_debugfs_init(void)
{
    asoc_clock_debugfs = debugfs_create_dir("leopard-clock", NULL);
    if (IS_ERR(asoc_clock_debugfs))
        asoc_clock_debugfs = NULL;
}
