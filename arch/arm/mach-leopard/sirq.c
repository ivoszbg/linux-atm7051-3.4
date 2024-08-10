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
#include <linux/init.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <mach/hardware.h>
#include <mach/irqs.h>

/* INTC_EXTCTL */
#define INTC_EXTCTL_E0EN            (0x1 << 21)
#define INTC_EXTCTL_E0PD            (0x1 << 16)
#define INTC_EXTCTL_E1EN            (0x1 << 13)
#define INTC_EXTCTL_E1PD            (0x1 << 8)
#define INTC_EXTCTL_E2EN            (0x1 << 5)
#define INTC_EXTCTL_E2PD            (0x1 << 0)

#define INTC_EXTCTL_E0TYPE(x)       (((x) & 0x03) << 22)
#define INTC_EXTCTL_E1TYPE(x)       (((x) & 0x03) << 14)
#define INTC_EXTCTL_E2TYPE(x)       (((x) & 0x03) << 6)

#define EXT_INT_TYPE_MASK           0x3
#define EXT_INT_TYPE_HIGH           0x0
#define EXT_INT_TYPE_LOW            0x1
#define EXT_INT_TYPE_RISING         0x2
#define EXT_INT_TYPE_FALLING        0x3

#define PAD_PULLCTL0_PSIRQ0P(x)     (((x) & 0x3) << 14)
#define PAD_PULLCTL0_PSIRQ1P(x)     (((x) & 0x3) << 12)
#define PAD_PULLCTL0_PSIRQ2P(x)     (((x) & 0x3) << 10)

static DEFINE_SPINLOCK(asoc_sirq_lock);

static void asoc_sirq_enable(struct irq_data *data)
{
    unsigned int irq = data->irq;
    unsigned int val;
    unsigned long irq_flags;

    spin_lock_irqsave(&asoc_sirq_lock, irq_flags);

    val = act_readl(INTC_EXTCTL);
    val &= ~(INTC_EXTCTL_E0PD | INTC_EXTCTL_E1PD | INTC_EXTCTL_E2PD);

    switch (irq) {
    case IRQ_SIRQ0:
        act_writel(val | INTC_EXTCTL_E0EN, INTC_EXTCTL);
        break;
    case IRQ_SIRQ1:
        act_writel(val| INTC_EXTCTL_E1EN, INTC_EXTCTL);
        break;
    case IRQ_SIRQ2:
        act_writel(val | INTC_EXTCTL_E2EN, INTC_EXTCTL);
        break;
    default:
        break;
    }

    spin_unlock_irqrestore(&asoc_sirq_lock, irq_flags);
}

static void asoc_sirq_disable(struct irq_data *data)
{
    unsigned int irq = data->irq;
    unsigned int val;
    unsigned long irq_flags;

    spin_lock_irqsave(&asoc_sirq_lock, irq_flags);

    val = act_readl(INTC_EXTCTL);
    val &= ~(INTC_EXTCTL_E0PD | INTC_EXTCTL_E1PD | INTC_EXTCTL_E2PD);

    switch (irq) {
    case IRQ_SIRQ0:
        act_writel(val & (~INTC_EXTCTL_E0EN), INTC_EXTCTL);
        break;
    case IRQ_SIRQ1:
        act_writel(val & (~INTC_EXTCTL_E1EN), INTC_EXTCTL);
        break;
    case IRQ_SIRQ2:
        act_writel(val & (~INTC_EXTCTL_E2EN), INTC_EXTCTL);
        break;
    default:
        break;
    }

    spin_unlock_irqrestore(&asoc_sirq_lock, irq_flags);
}

static int asoc_sirq_set_type(struct irq_data *data, unsigned int flow_type)
{
    unsigned int irq, regv, type;
    unsigned long irq_flags;

    irq = data->irq;
    flow_type &= IRQ_TYPE_SENSE_MASK;

    switch (flow_type) {
    case IRQ_TYPE_EDGE_RISING:  
        type = EXT_INT_TYPE_RISING;
        break;
    case IRQ_TYPE_EDGE_FALLING:  
        type = EXT_INT_TYPE_FALLING;
        break;
    case IRQ_TYPE_LEVEL_HIGH:  
        type = EXT_INT_TYPE_HIGH;
        break;
    case IRQ_TYPE_LEVEL_LOW:  
        type = EXT_INT_TYPE_LOW;
        break;
    default:
        pr_err("[SIRQ] %s: irq %d, unknow type %d\n",
            __FUNCTION__, irq, flow_type);
        return -1;
    }

    spin_lock_irqsave(&asoc_sirq_lock, irq_flags);

    switch (irq) {
    case IRQ_SIRQ0:
        regv = act_readl(INTC_EXTCTL);
        regv &= ~INTC_EXTCTL_E0TYPE(3);
        regv |= INTC_EXTCTL_E0TYPE(type);
        act_writel(regv, INTC_EXTCTL);

        regv = act_readl(PAD_PULLCTL0);
        regv &= ~PAD_PULLCTL0_PSIRQ0P(3);
        if (flow_type == IRQ_TYPE_LEVEL_HIGH
                || flow_type == IRQ_TYPE_EDGE_RISING)
            /* 100K pull-up disable and 100K pull-down enable */
            regv |= PAD_PULLCTL0_PSIRQ0P(2);
        if (flow_type == IRQ_TYPE_LEVEL_LOW
                || flow_type == IRQ_TYPE_EDGE_FALLING)
            /* 100K pull-up enable and 100K pull-down disable */
            regv |= PAD_PULLCTL0_PSIRQ0P(1);
        act_writel(regv, PAD_PULLCTL0);
        break;
    case IRQ_SIRQ1:
        regv = act_readl(INTC_EXTCTL);
        regv &= ~INTC_EXTCTL_E1TYPE(3);
        regv |= INTC_EXTCTL_E1TYPE(type);
        act_writel(regv, INTC_EXTCTL);

        regv = act_readl(PAD_PULLCTL0);
        regv &= ~PAD_PULLCTL0_PSIRQ1P(3);
        if (flow_type == IRQ_TYPE_LEVEL_HIGH
                || flow_type == IRQ_TYPE_EDGE_RISING)
             /* 100K pull-up disable and 100K pull-down enable */
            regv |= PAD_PULLCTL0_PSIRQ1P(2);
        if (flow_type == IRQ_TYPE_LEVEL_LOW
                || flow_type == IRQ_TYPE_EDGE_FALLING)
            /* 100K pull-up enable and 100K pull-down disable */
            regv |= PAD_PULLCTL0_PSIRQ1P(1);
        act_writel(regv, PAD_PULLCTL0);
        break;
    case IRQ_SIRQ2:
        regv = act_readl(INTC_EXTCTL);
        regv &= ~INTC_EXTCTL_E2TYPE(3);
        regv |= INTC_EXTCTL_E2TYPE(type);
        act_writel(regv, INTC_EXTCTL);

        regv = act_readl(PAD_PULLCTL0);
        regv &= ~PAD_PULLCTL0_PSIRQ2P(3);
        if (flow_type == IRQ_TYPE_LEVEL_HIGH
                || flow_type == IRQ_TYPE_EDGE_RISING)
             /* 100K pull-up disable and 100K pull-down enable */
            regv |= PAD_PULLCTL0_PSIRQ2P(2);
        if (flow_type == IRQ_TYPE_LEVEL_LOW
                || flow_type == IRQ_TYPE_EDGE_FALLING)
            /* 100K pull-up enable and 100K pull-down disable */
            regv |= PAD_PULLCTL0_PSIRQ2P(1);
        act_writel(regv, PAD_PULLCTL0);
        break;
    default:
        printk("[SIRQ] unsupported sirq: %d\n", irq);
        spin_unlock_irqrestore(&asoc_sirq_lock, irq_flags);
        return -1;
    }

    spin_unlock_irqrestore(&asoc_sirq_lock, irq_flags);

    return 0;
}

static struct irq_chip asoc_sirq_irq =
{
    .name = "asoc_sirq_irq",
    .irq_ack = asoc_sirq_disable,
    .irq_mask = asoc_sirq_disable,
    .irq_mask_ack = asoc_sirq_disable,
    .irq_unmask = asoc_sirq_enable,
    .irq_set_type = asoc_sirq_set_type,
};

static void asoc_sirq_handler(unsigned int irq, struct irq_desc *desc)
{
    unsigned int pending, casc_irq;
    struct irq_chip *chip = irq_get_chip(irq);
    unsigned long irq_flags;

    spin_lock_irqsave(&asoc_sirq_lock, irq_flags);

    pending = act_readl(INTC_EXTCTL);
    casc_irq = 0;

    switch (irq) {
    case IRQ_ASOC_SIRQ0:
        if (pending & INTC_EXTCTL_E0PD) {
            casc_irq = IRQ_SIRQ0;
            /* don't clear other sirq pending */
            pending &= ~(INTC_EXTCTL_E1PD | INTC_EXTCTL_E2PD);
        }
        break;
    case IRQ_ASOC_SIRQ1:
        if (pending & INTC_EXTCTL_E1PD) {
            casc_irq = IRQ_SIRQ1;
            /* don't clear other sirq pending */
            pending &= ~(INTC_EXTCTL_E0PD | INTC_EXTCTL_E2PD);
        }
        break;
    case IRQ_ASOC_SIRQ2:
        if (pending & INTC_EXTCTL_E2PD) {
            casc_irq = IRQ_SIRQ2;
            /* don't clear other sirq pending */
            pending &= ~(INTC_EXTCTL_E0PD | INTC_EXTCTL_E1PD);
        }
        break;
    default:
        printk("[SIRQ] %s(): invalid virtual sirq number %d\n", __FUNCTION__, irq);
        return;
    }

    /* clear pending */
    act_writel(pending, INTC_EXTCTL);

    spin_unlock_irqrestore(&asoc_sirq_lock, irq_flags);

    generic_handle_irq(casc_irq);

    if (chip->irq_ack)
        chip->irq_ack(&desc->irq_data);
    if (chip->irq_eoi)
        chip->irq_eoi(&desc->irq_data);

    chip->irq_unmask(&desc->irq_data);
}

int __init asoc_sirq_init(void)
{
    printk("[SIRQ] init sirqs\n");

    irq_set_chip_and_handler(IRQ_SIRQ0, &asoc_sirq_irq,
        handle_level_irq);
    set_irq_flags(IRQ_SIRQ0, IRQF_VALID);
    irq_set_chained_handler(IRQ_ASOC_SIRQ0, asoc_sirq_handler);

    irq_set_chip_and_handler(IRQ_SIRQ1, &asoc_sirq_irq,
        handle_level_irq);
    set_irq_flags(IRQ_SIRQ1, IRQF_VALID);
    irq_set_chained_handler(IRQ_ASOC_SIRQ1, asoc_sirq_handler);

    irq_set_chip_and_handler(IRQ_SIRQ2, &asoc_sirq_irq,
        handle_level_irq);
    set_irq_flags(IRQ_SIRQ2, IRQF_VALID);
    irq_set_chained_handler(IRQ_ASOC_SIRQ2, asoc_sirq_handler);

    return 0;
}

subsys_initcall(asoc_sirq_init);
