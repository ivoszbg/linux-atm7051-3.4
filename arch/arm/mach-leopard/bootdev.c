/*
 * arch/arm/mach-leopard/bootdev.c
 *
 * bootdev for Actions SOC
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
#include <linux/vmalloc.h>
#include <asm/setup.h>
#include <mach/bootdev.h>

#include <asm/uaccess.h>
#include <asm/mach/map.h>

static int g_boot_dev = 0;

int asoc_get_boot_dev(void)
{
    return g_boot_dev;
}
EXPORT_SYMBOL_GPL(asoc_get_boot_dev);

/* parse ATAG_BOOTDEV */
static int __init parse_tag_boot_dev(const struct tag *tag)
{
	g_boot_dev = tag->u.boot_dev.boot_dev;

	printk("tag: boot_dev %d\n", g_boot_dev);

	return 0;
}

__tagtable(ATAG_BOOT_DEV, parse_tag_boot_dev);


static int bootdev_proc_show(struct seq_file *m, void *v)
{
    switch (g_boot_dev) {
    case ASOC_BOOTDEV_NAND:
	    seq_printf(m, "nand\n");
        break;
    case ASOC_BOOTDEV_SD0:
	    seq_printf(m, "sd0\n");
        break;
    case ASOC_BOOTDEV_SD1:
	    seq_printf(m, "sd1\n");
        break;
    case ASOC_BOOTDEV_SD2:
	    seq_printf(m, "sd2\n");
        break;
    case ASOC_BOOTDEV_SD02NAND:
	    seq_printf(m, "sd02nand\n");
        break; 
    case ASOC_BOOTDEV_SD02SD2:
	    seq_printf(m, "sd02sd2\n");
        break;  		
    default:
	    seq_printf(m, "unkown\n");
        break;
    }
	return 0;
}

static char *get_user_string(const char __user *user_buf, size_t user_len)
{
	char *buffer;

	buffer = vmalloc(user_len + 1);
	if (buffer == NULL)
		return ERR_PTR(-ENOMEM);
	if (copy_from_user(buffer, user_buf, user_len) != 0) {
		vfree(buffer);
		return ERR_PTR(-EFAULT);
	}
	/* got the string, now strip linefeed. */
	if (buffer[user_len - 1] == '\n')
		buffer[user_len - 1] = 0;
	else
		buffer[user_len] = 0;
	return buffer;
}


static ssize_t bootdev_proc_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *ppos)
{
	char *buf, *str;
    buf = get_user_string(buffer, count);
	if (IS_ERR(buf))
		return PTR_ERR(buf);	
	str = skip_spaces(buf);
	if (!strcmp(str, "nand")) {
        g_boot_dev = ASOC_BOOTDEV_NAND;
    } else if (!strcmp(str, "sd0")) {
        g_boot_dev = ASOC_BOOTDEV_SD0;
    } else if (!strcmp(str, "sd1")) {
        g_boot_dev = ASOC_BOOTDEV_SD1;
    } else if (!strcmp(str, "sd2")) {
        g_boot_dev = ASOC_BOOTDEV_SD2;
    } else if (!strcmp(str, "sd02nand")) {
        g_boot_dev = ASOC_BOOTDEV_SD02NAND;
    } else if (!strcmp(str, "sd02sd2")) {
        g_boot_dev = ASOC_BOOTDEV_SD02SD2;
    } else {
        pr_warning("invalid boot device name\n");
    }

	vfree(buf);
	return count;
}


static int bootdev_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, bootdev_proc_show, NULL);
}

static const struct file_operations bootdev_proc_fops = {
	.open		= bootdev_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
    .write		= bootdev_proc_write,
};

static int __init proc_bootdev_init(void)
{
	proc_create("bootdev", S_IRUGO | S_IWUSR, NULL, &bootdev_proc_fops);
	return 0;
}
module_init(proc_bootdev_init);
