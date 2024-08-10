/*
 * arch/arm/mach-leopard/dma.c
 *
 * DMA interface for Actions SOC 
 *
 * Copyright 2012 Actions Semi Inc.
 * Author: Actions Semi, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h> 
#include <linux/kernel.h> 
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>

#include <mach/dma.h>
#include <mach/irqs.h>
#include <mach/clock.h>

#ifdef DEBUG_DMA
#define DMSG_DMA(stuff...)  printk(stuff)
#else
#define DMSG_DMA(stuff...)  do{}while(0)
#endif


static irqreturn_t asoc_dma_irq(int irq, void *dev_id);

static struct irqaction dma_irqaction =
{
    .handler = asoc_dma_irq,
//  .mask = CPU_MASK_NONE,
    .name = "asoc-dma",
    .flags = IRQF_SHARED,
};

static DEFINE_SPINLOCK(dma_table_lock);
DEFINE_SPINLOCK(dma_regop_lock);

struct dma_chan asoc_dma_table[NUM_ASOC_DMA_CHANNELS] =
{
    {.is_used=0,.callback = NULL},
    {.is_used=0,.callback = NULL},
    {.is_used=0,.callback = NULL},
    {.is_used=0,.callback = NULL},
};

static irqreturn_t asoc_dma_irq(int irq, void *dev_id)
{
    int i;
    u32 tc_pending = act_readl(DMA_IRQPD);
    u32 tc_mask = act_readl(DMA_IRQEN);

    tc_mask &= tc_pending;
    //act_writel(tc_pending, DMA_IRQ_PEND);
    for (i = 0; i < NUM_ASOC_DMA_CHANNELS; i++)
    {
        if (tc_mask & DMA_IRQPD_DXTP(i))
        {
            /* BUG_ON(!(asoc_dma_table[i].callback)); */
            BUG_ON(!(asoc_dma_table[i].is_used));

            if (asoc_dma_table[i].callback != NULL)
            {
                asoc_dma_table[i].callback(i,
                    asoc_dma_table[i].irq_dev);
            }
            else
            {
                /* printk("bug on dma!\n"); */
            }
        }
    }
    /* act_writel(0xffff, DMA_IRQ_PEND); */
    return IRQ_HANDLED;
}

static int __init asoc_dma_init(void)
{
    struct clk *dma_clk;

    dma_clk = clk_get_sys(CLK_NAME_DMA_CLK, NULL);
    clk_enable(dma_clk);
    clk_reset(dma_clk);

    /* 200MHz */
    clk_set_rate(dma_clk, 200 * 1000 * 1000);

    printk("[asoc] dma clock rate %ld\n", clk_get_rate(dma_clk));

    act_writel(0xff0000, DMA_CTL);
    act_writel(0x0, DMA_IRQEN);
    act_writel(0xffff, DMA_IRQPD);

    setup_irq(IRQ_ASOC_DMA, &dma_irqaction);

    return 0;
}

arch_initcall(asoc_dma_init);

struct dma_chan *get_dma_chan(unsigned int dmanr)
{
    if ((dmanr >= NUM_ASOC_DMA_CHANNELS)
        || (asoc_dma_table[dmanr].is_used == 0))
        return NULL;

    return &asoc_dma_table[dmanr];
}

int request_asoc_dma(unsigned int chan_type, const char *dev_str,
        irqhandler_t irqhandler,
        unsigned long irqflags, void *irq_dev_id)
{
    struct dma_chan *chan;
    int i;
    unsigned long flag;

    if (chan_type != DMA_CHAN_TYPE_BUS)
        return -1;

    spin_lock_irqsave(&dma_table_lock, flag);
    for (i = 0; i < NUM_ASOC_DMA_CHANNELS; i++)
    if (!(asoc_dma_table[i].is_used))
        break;

    if (i == NUM_ASOC_DMA_CHANNELS) {
        spin_unlock_irqrestore(&dma_table_lock, flag);
        return -1;
    }

    chan = &asoc_dma_table[i];
    chan->is_used = 1;
    if (irqhandler) {
        chan->callback = (chan_callback_t)irqhandler;
        chan->irq_dev = irq_dev_id;
        clear_dma_tcirq_pend(i);
    } else {
        //  chan->irq = 0;
        chan->irq_dev = NULL;
    }

    enable_dma_tcirq(i);
    chan->io = DMA_CHANNEL_BASE + i * DMA_CHANNEL_LEN;
    chan->dev_str = dev_str;
    chan->chan_type = chan_type;
    spin_unlock_irqrestore(&dma_table_lock,flag);

    return i;
}

void free_asoc_dma(unsigned int dmanr)
{
    unsigned long flag;
    struct dma_chan *chan = get_dma_chan(dmanr);

    if (!chan) {
        DMSG_DMA("Trying to free DMA%d\n", dmanr);
        return;
    }

    spin_lock_irqsave(&dma_table_lock, flag);

    reset_dma(dmanr);
    disable_dma_tcirq(dmanr);
    clear_dma_tcirq_pend(dmanr);

    chan->callback = NULL;
    chan->irq_dev = NULL;
    chan->is_used = 0;

    spin_unlock_irqrestore(&dma_table_lock, flag);
}

#if defined(CONFIG_PROC_FS)
static int proc_dma_show(struct seq_file *m, void *v)
{
    int i;
    struct dma_chan *chan;
    int count, remain;

    for (i = 0 ; i < NUM_ASOC_DMA_CHANNELS ; i++) {
        chan = &asoc_dma_table[i];

        seq_printf(m, "%2d: ", i);
        seq_printf(m, " %14s", (chan->is_used)?chan->dev_str:"free");
        seq_printf(m, " %8s", dma_started(i)?"started":"stopped");
        seq_printf(m, " %8s", dma_paused(i)?"paused":"running");

        count = get_dma_count(i);
        seq_printf(m, " %10u", (count >= 0)?count:0);
        remain = get_dma_remain(i);
        seq_printf(m, " %10u", (remain >= 0)?remain:0);
        seq_putc(m, '\n');
    }
    return 0;
}

static int proc_dma_open(struct inode *inode, struct file *file)
{
    return single_open(file, proc_dma_show, NULL);
}

static const struct file_operations proc_dma_operations = {
    .open       = proc_dma_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};

static int __init proc_dma_init(void)
{
    printk(KERN_ALERT "PROC DMA INIT\n");
    proc_create("dma", 0, NULL, &proc_dma_operations);
    return 0;
}

__initcall(proc_dma_init);
#endif

EXPORT_SYMBOL(request_asoc_dma);
EXPORT_SYMBOL(free_asoc_dma);
EXPORT_SYMBOL(get_dma_chan);
EXPORT_SYMBOL(dma_regop_lock);
