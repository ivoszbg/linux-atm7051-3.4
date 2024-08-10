/*
 * arch/arm/mach-leopard/sirq.c
 *
 * special external IRQs support for Actions SOC
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
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/debug.h>

/* FIXME: move symbol from adfus to kernel temporarily */
typedef unsigned int (*func_t)(unsigned int *, void *);

func_t AdfuUpdateMbrFromPhyToUsr;
EXPORT_SYMBOL(AdfuUpdateMbrFromPhyToUsr);

typedef void (*func_t1)(void);
func_t1 adfu_flush_nand_cache;
EXPORT_SYMBOL(adfu_flush_nand_cache);

typedef int (*func_t4)(unsigned long, unsigned long , void *, void *);

func_t4 adfus_nand_read;
func_t4 adfus_nand_write;
EXPORT_SYMBOL(adfus_nand_read);
EXPORT_SYMBOL(adfus_nand_write);

void asoc_dump_mem(void *startaddr, int size, void *showaddr, int show_bytes)
{
    int i, count, count_per_line;
    void *addr = startaddr;

    if ((show_bytes != 1) && (show_bytes != 2) && (show_bytes != 4))
    {
        printk("dump_mem: not support mode\n");
        return;
    }

    if (((int) startaddr & (show_bytes - 1)) || (size & (show_bytes - 1)))
    {
        printk("dump_mem: startaddr must be aligned by %d bytes!\n", show_bytes);
        return;
    }

    count = size / show_bytes;
    count_per_line = 16 / show_bytes; // 16 bytes per line

    printk("startaddr 0x%p, size 0x%x\n",
        startaddr, size);

    i = 0;
    while (i < count)
    {
        if ((i % count_per_line) == 0) {
            if (i != 0)
                printk("\n");

            printk("%08x: ", (unsigned int)showaddr + ((i / count_per_line) * 16));
        }
        switch (show_bytes) {
        case 1:
            printk("%02x ", *((unsigned char *) addr + i));
            break;
        case 2:
            printk("%04x ", *((unsigned short *) addr + i));
            break;
        case 4:
            printk("%08x ", *((unsigned int *) addr + i));
            break;
        default:
            printk("dump_mem: not support mode\n");
            return;
        }

        i++;
    }
    printk("\n");
}

EXPORT_SYMBOL(asoc_dump_mem);

void asoc_dump_reg(unsigned int addr, int size)
{
    int count, i = 0;

    if ((addr & 3) || (size & 3)) {
        printk("asoc_dump_reg: startaddr must be aligned by 4 bytes!\n");
        return;
    }

    count = size / 4;

    printk("addr 0x%08x, size 0x%x\n", addr, size);

    while (i < count) {
        if ((i % 4) == 0) {
            if (i != 0)
                printk("\n");

            printk("%08x: ", (unsigned int)addr + i * 4);
        }

        printk("%08x ", act_readl((unsigned int)addr + i * 4));

        i++;
    }
    printk("\n");
}

EXPORT_SYMBOL(asoc_dump_reg);

#ifdef CONFIG_DEBUG_FS

static ssize_t reg_read(struct file *filp, char __user *buffer,
        size_t count, loff_t *ppos)
{
    printk("read/write a register:\n");
    printk("  read : echo 0xb01c0000 > reg\n");
    printk("  write: echo 0xb01c00000=0x12345678 > reg\n");

    return 0;
}

static ssize_t reg_write(struct file *filp, const char __user *buffer,
        size_t count, loff_t *ppos)
{
    unsigned int reg, read_val, reg_val;
    char buf[32];
    char *end_ptr;
    int write = 0;

    if (*ppos != 0)
        return -EINVAL;
    if (count > 32)
        return -EINVAL;
    if (copy_from_user(buf, buffer, count))
        return -EFAULT;

    *ppos += count;

    reg = simple_strtoul(buf, &end_ptr, 16);

    if ((reg & 0x3) || (reg < ASOC_PA_REG_BASE) ||
        (reg >= ASOC_PA_REG_BASE + 6 * SZ_1M)) {
        printk("invalid register address\n");
        return -EINVAL;
    }

    if ((buf == end_ptr) )
        goto out;

    read_val = act_readl(reg);
    printk("[0x%08x]: 0x%08x\n", reg, read_val);

    if (*end_ptr++ == '=') {
        reg_val = simple_strtoul(end_ptr, NULL, 16);
        write = 1;
    }

    if (write) {
        act_writel(reg_val, reg);
        printk("[0x%08x] <- 0x%08x\n", reg, reg_val);
    }

out:
    return count;
}

static int reg_open(struct inode *inode, struct file *filp)
{
    filp->private_data = inode->i_private;
    return 0;
}

static struct file_operations reg_fops = {
    .open = reg_open,
    .read = reg_read,
    .write = reg_write,
};

int __init asoc_debug_init(void)
{
    struct dentry *dir;
    struct dentry *d;

    dir = debugfs_create_dir("asoc", NULL);
    if (!dir)
        return -ENOMEM;

    d = debugfs_create_file("reg", S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP, dir, NULL,
        &reg_fops);
    if (!d)
        return -ENOMEM;

    return 0;
}

arch_initcall(asoc_debug_init);

#endif /* CONFIG_DEBUG_FS */
