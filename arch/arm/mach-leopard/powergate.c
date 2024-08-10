/*
 * arch/arm/mach-leopard/powergate.c
 *
 * powergate support for Actions SOC
 *
 * Copyright 2012 Actions Semi Inc.
 * Author: Actions Semi, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

//#define DEBUG

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>

#include <mach/hardware.h>
#include <mach/powergate.h>
#include <mach/clock.h>


#define NO_RESET_ID                 0xffffffff

struct asoc_powergate_info
{
    char name[16];              /* name of powergate */
    unsigned int reset_id;      /* modules reset id*/
    int count;                  /* power on count */
    int init_power_off;         /* power off at boot init stage */
    int use_mutex;              /* need use mutex? */
    struct mutex mutex;         /* mutex of one powergate for multi muser */
};

static struct asoc_powergate_info powergate_info[] =
{
    [ASOC_POWERGATE_CPU1] = {
        .name = "cpu1",
        .reset_id = NO_RESET_ID,
        .init_power_off = 0,
        .use_mutex = 0,
        .count = 0,
    },

    [ASOC_POWERGATE_CPU2] = {
        .name = "cpu2",
        .reset_id = NO_RESET_ID,
        .init_power_off = 0,
        .use_mutex = 0,
        .count = 0,
    },

    [ASOC_POWERGATE_CPU3] = {
        .name = "cpu3",
        .reset_id = NO_RESET_ID,
        .init_power_off = 0,
        .use_mutex = 0,
        .count = 0,
    },

    [ASOC_POWERGATE_GPU] = {
        .name = "gpu",
        .reset_id = MODULE_RESET_GPU,
        .init_power_off = 1,
        .use_mutex = 1,
        .count = 0,
    },

    [ASOC_POWERGATE_VCE] = {
        .name = "vce",
        .reset_id = MODULE_RESET_VCE,
        .init_power_off = 1,
        .use_mutex = 1,
        .count = 0,
    },

    [ASOC_POWERGATE_VDE] = {
        .name = "vde",
        .reset_id = MODULE_RESET_VDE,
        .init_power_off = 1,
        .use_mutex = 1,
        .count = 0,
    },
};

static unsigned int asoc_cpu_domains[] = {
	0xffffffff,
	ASOC_POWERGATE_CPU1,
	ASOC_POWERGATE_CPU2,
	ASOC_POWERGATE_CPU3,
};

static DEFINE_SPINLOCK(asoc_powergate_lock);
static DEFINE_SPINLOCK(module_rst_lock);

static int __module_reset(enum module_reset_id reset_id, int holding, int holding_assert)
{
    unsigned int reg, regv, id;
    unsigned long flags;

    id = reset_id;

    pr_debug("%s(reset_id: %d)\n", __func__, reset_id);

    if (id < 32)
        reg = CMU_DEVRST0;
    else if (id < MODULE_RESET_ID_MAX) {
        reg = CMU_DEVRST1;
        id -= 32;
    } else
        return -1;

    spin_lock_irqsave(&module_rst_lock, flags);

    regv = act_readl(reg);

    if (!holding || holding_assert)
    {
        regv &= ~(1 << id);
        act_writel(regv, reg);
        regv = act_readl(reg);
    }

    if (!holding || !holding_assert)
    {
        regv |= (1 << id);
        act_writel(regv, reg);
        regv = act_readl(reg);
    }

    spin_unlock_irqrestore(&module_rst_lock, flags);

    pr_debug("%s(reset_id: %d) reg %x: %x\n", __func__, reset_id,
        reg, act_readl(reg));

    return 0;
}

static int asoc_module_reset_assert(enum module_reset_id reset_id)
{
    return __module_reset(reset_id, 1, 1);
}

static int asoc_module_reset_deassert(enum module_reset_id reset_id)
{
    return __module_reset(reset_id, 1, 0);
}

int module_reset(enum module_reset_id reset_id)
{
    return __module_reset(reset_id, 0, 0);
}
EXPORT_SYMBOL_GPL(module_reset);

static int asoc_powergate_set(enum asoc_powergate_id id, bool on)
{
    struct asoc_powergate_info *pgi = &powergate_info[id];
    bool ack_is_on;
	unsigned long val, flags;
    int timeout;

    pr_debug("[PowerGate] name: '%s', on: %d, SPS_PG_ACK: 0x%x\n", 
         pgi->name, on, act_readl(SPS_PG_ACK));

	spin_lock_irqsave(&asoc_powergate_lock, flags);

	ack_is_on = (act_readl(SPS_PG_ACK) & (1 << id));

	if (ack_is_on == on) {
		spin_unlock_irqrestore(&asoc_powergate_lock, flags);
		return 0;
	}

    /* assert modules reset before poweron */
    if (on) {
        if (pgi->reset_id != NO_RESET_ID)
            asoc_module_reset_assert(pgi->reset_id);
    }

	val = act_readl(SPS_PG_CTL);
    if (on)
        val |= (1 << id);
    else
        val &= ~(1 << id);
    act_writel(val, SPS_PG_CTL);

    if (on) {
        timeout = 5000;  /* 5ms */
        while (timeout > 0 && !asoc_powergate_is_powered(id)) {
            udelay(50);
            timeout -= 50;
        }
        if (timeout <= 0) {
            pr_err("[PowerGate] enable power for id %d timeout\n", id);
        }

        /* deasert modules reset after poweron */
        if (pgi->reset_id != NO_RESET_ID)
            asoc_module_reset_deassert(pgi->reset_id);
    } else {
        timeout = 5000;  /* 5ms */
        while (timeout > 0 && asoc_powergate_is_powered(id)) {
            udelay(50);
            timeout -= 50;
        }
        if (timeout <= 0) {
            pr_err("[PowerGate] disable power for id %d timeout\n", id);
        }
    }

	spin_unlock_irqrestore(&asoc_powergate_lock, flags);

	return 0;
}

int asoc_powergate_power_on(enum asoc_powergate_id id)
{
    struct asoc_powergate_info *pgi;
    int ret;

	if (id < 0 || id >= ASOC_POWERGATE_MAXID)
		return -EINVAL;

    pgi = &powergate_info[id];

    pr_debug("[PowerGate] %s(): '%s', count %d\n",
        __FUNCTION__, pgi->name, pgi->count);

    if (pgi->use_mutex)
        mutex_lock(&pgi->mutex);
    
    pgi->count++;

    if (asoc_powergate_is_powered(id) > 0) {
        if (pgi->use_mutex)
            pr_debug("[PowerGate] %s(): '%s', skip power on, new count %d\n",
                __FUNCTION__, pgi->name, pgi->count);

            mutex_unlock(&pgi->mutex);
        return 0;
    }

    ret = asoc_powergate_set(id, true);

    if (pgi->use_mutex)
    	mutex_unlock(&pgi->mutex);

	return ret;
}
EXPORT_SYMBOL(asoc_powergate_power_on);

int asoc_powergate_power_off(enum asoc_powergate_id id)
{
    struct asoc_powergate_info *pgi;
    int ret = 0;

	if (id < 0 || id >= ASOC_POWERGATE_MAXID)
		return -EINVAL;

    pgi = &powergate_info[id];

    pr_debug("[PowerGate] %s(): '%s', count %d\n",
        __FUNCTION__, pgi->name, pgi->count);

    if (pgi->use_mutex)
        mutex_lock(&pgi->mutex);

    if (WARN(pgi->count <= 0,
         "unbalanced power off for %s\n", pgi->name)) {
        if (pgi->use_mutex)
            mutex_unlock(&pgi->mutex);
        return -EIO;
    }

    pgi->count--;
    if (pgi->count == 0) {
        pr_debug("[PowerGate] %s(): '%s', count is 0, real power off\n",
            __FUNCTION__, pgi->name);

        ret = asoc_powergate_set(id, false);
    }

    if (pgi->use_mutex)
        mutex_unlock(&pgi->mutex);

    return ret;
}
EXPORT_SYMBOL(asoc_powergate_power_off);

int asoc_powergate_is_powered(enum asoc_powergate_id id)
{
    struct asoc_powergate_info *pgi;
	u32 status;

	if (id < 0 || id >= ASOC_POWERGATE_MAXID)
		return -EINVAL;

    pgi = &powergate_info[id];

	status = act_readl(SPS_PG_ACK) & (1 << id);

    pr_debug("[PowerGate] %s(): '%s', status %d\n",
        __FUNCTION__, pgi->name, !!status);

	return !!status;
}
EXPORT_SYMBOL(asoc_powergate_is_powered);

int asoc_cpu_powergate_id(int cpuid)
{
	if (cpuid > 0 && cpuid < ARRAY_SIZE(asoc_cpu_domains))
		return asoc_cpu_domains[cpuid];

	return -EINVAL;
}

#ifdef CONFIG_DEBUG_FS

static int powergate_show(struct seq_file *s, void *data)
{
    struct asoc_powergate_info *pgi;
	int i;

	seq_printf(s, " powergate powered\n");
	seq_printf(s, "------------------\n");

	seq_printf(s, "     name     status    count\n");

	for (i = 0; i < ASOC_POWERGATE_MAXID; i++) {
        pgi = &powergate_info[i];

		seq_printf(s, " %9s %7s %7d\n", 
            pgi->name,
			asoc_powergate_is_powered(i) ? "on" : "off",
            pgi->count);
    }

	return 0;
}

static int powergate_open(struct inode *inode, struct file *file)
{
	return single_open(file, powergate_show, inode->i_private);
}

static const struct file_operations powergate_fops = {
	.open		= powergate_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init powergate_debugfs_init(void)
{
    struct dentry *d;

	d = debugfs_create_file("powergate", S_IRUGO, NULL, NULL,
		&powergate_fops);
	if (!d)
		return -ENOMEM;

	return 0;
}

#else
static int powergate_debugfs_init(void)
{
    return 0;
}
#endif


int asoc_powergate_init(void)
{
    struct asoc_powergate_info *pgi;
	int i;

	pr_info("[PowerGate] init\n");

	for (i = 0; i < ASOC_POWERGATE_MAXID; i++) {
        pgi = &powergate_info[i];

        if (asoc_powergate_is_powered(i) > 0) {
            if (pgi->init_power_off) {
                pr_debug("[PowerGate] %s(): '%s', init power off\n",
                    __FUNCTION__, pgi->name);
                /* power off */
                asoc_powergate_set(i, false);
                pgi->count = 0;
            } else {
                pgi->count = 1;
            }
        } else {
            pgi->count = 0;
        }

        pr_debug("[PowerGate] %s(): '%s', init count %d\n",
            __FUNCTION__, pgi->name, pgi->count);

        if (pgi->use_mutex)
		    mutex_init(&pgi->mutex);
    }

    powergate_debugfs_init();

    return 0;
}

arch_initcall(asoc_powergate_init);
