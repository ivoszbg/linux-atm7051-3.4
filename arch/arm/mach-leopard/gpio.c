/*
 * arch/arm/mach-leopard/hotplug.c
 *
 * GPIO interface for Actions SOC
 *
 * Copyright 2012 Actions Semi Inc.
 * Author: Actions Semi, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/pinctrl/consumer.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/gpio.h>

#define GPIO_REG_BASE               (GPIO_MFP_PWM_BASE)

#define GPIO_BANK(gpio)             ((gpio) / 32)
#define GPIO_IN_BANK(gpio)          ((gpio) % 32)
#define GPIO_BIT(gpio)              (1 << GPIO_IN_BANK(gpio))

#define GPIO_REG_OUTEN(gpio)        (GPIO_REG_BASE + GPIO_BANK(gpio) * 0xc + 0x0)
#define GPIO_REG_INEN(gpio)         (GPIO_REG_BASE + GPIO_BANK(gpio) * 0xc + 0x4)
#define GPIO_REG_DAT(gpio)          (GPIO_REG_BASE + GPIO_BANK(gpio) * 0xc + 0x8)
#define GPIO_REG_INTC_PD(gpio)      (GPIO_REG_BASE + GPIO_BANK(gpio) * 0x8 + 0x208)
#define GPIO_REG_INTC_MASK(gpio)    (GPIO_REG_BASE + GPIO_BANK(gpio) * 0x8 + 0x20c)

/* INTC_EXTCTL */
#define GPIO_INT_TYPE_MASK          0x3
#define GPIO_INT_TYPE_HIGH          0x0
#define GPIO_INT_TYPE_LOW           0x1
#define GPIO_INT_TYPE_RISING        0x2
#define GPIO_INT_TYPE_FALLING       0x3

static DEFINE_SPINLOCK(asoc_gpio_lock);

static int asoc_gpio_request(struct gpio_chip *chip, unsigned offset)
{
    int gpio = chip->base + offset;

    if (offset >= ASOC_NR_GPIO) {
        printk("%s: invalid gpio %u\n", __FUNCTION__, offset);
        return -1;
    }

#ifdef CONFIG_PINCTRL_ASOC
	/*
	 * Map back to global GPIO space and request muxing, the direction
	 * parameter does not matter for this controller.
	 */

	return pinctrl_request_gpio(gpio);
#else
    return 0;
#endif
}

static void asoc_gpio_free(struct gpio_chip *chip, unsigned offset)
{
    int gpio = chip->base + offset;

    if (offset >= ASOC_NR_GPIO) {
        printk("%s: invalid gpio %u\n", __FUNCTION__, offset);
        return;
    }

#ifdef CONFIG_PINCTRL_ASOC
	pinctrl_free_gpio(gpio);
#endif
}

static int asoc_gpio_get(struct gpio_chip *chip, unsigned offset)
{
    return act_readl(GPIO_REG_DAT(offset)) & GPIO_BIT(offset);
}

static void asoc_gpio_set(struct gpio_chip *chip, unsigned offset, int val)
{
    unsigned int dat;

    dat = act_readl(GPIO_REG_DAT(offset));

    if (val)
        dat |= GPIO_BIT(offset);
    else
        dat &= ~GPIO_BIT(offset);

    act_writel(dat, GPIO_REG_DAT(offset));
}

static int asoc_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
    unsigned long irq_flags;

    spin_lock_irqsave(&asoc_gpio_lock, irq_flags);

    act_writel(act_readl(GPIO_REG_OUTEN(offset)) & ~GPIO_BIT(offset),
        GPIO_REG_OUTEN(offset));

    act_writel(act_readl(GPIO_REG_INEN(offset)) | GPIO_BIT(offset),
        GPIO_REG_INEN(offset));

    spin_unlock_irqrestore(&asoc_gpio_lock, irq_flags);

    return 0;
}

static int asoc_gpio_direction_output(struct gpio_chip *chip,
                unsigned offset,
                int val)
{
    unsigned long irq_flags;

    spin_lock_irqsave(&asoc_gpio_lock, irq_flags);

    act_writel(act_readl(GPIO_REG_INEN(offset)) & ~GPIO_BIT(offset),
        GPIO_REG_INEN(offset));

    act_writel(act_readl(GPIO_REG_OUTEN(offset)) | GPIO_BIT(offset),
        GPIO_REG_OUTEN(offset));

    asoc_gpio_set(chip, offset, val);

    spin_unlock_irqrestore(&asoc_gpio_lock, irq_flags);
    return 0;
}

static int asoc_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
    return ASOC_GPIO_TO_IRQ(chip->base + offset);
}

static inline int asoc_irq_to_gpio(struct gpio_chip *chip, unsigned irq)
{
    return irq - ASOC_GPIO_TO_IRQ(chip->base);
}

static struct gpio_chip asoc_gpio_chip = {
    .label              = "asoc-gpio-chip",
    .base               = 0,
    .ngpio              = ASOC_NR_GPIO,
    .request            = asoc_gpio_request,
    .free               = asoc_gpio_free,
    .direction_input    = asoc_gpio_direction_input,
    .direction_output   = asoc_gpio_direction_output,
    .get                = asoc_gpio_get,
    .set                = asoc_gpio_set,
    .to_irq             = asoc_gpio_to_irq,
};


static void asoc_gpio_irq_mask(struct irq_data *d)
{
    int gpio = asoc_irq_to_gpio(&asoc_gpio_chip, d->irq);
    unsigned long irq_flags;
    unsigned int val;

    spin_lock_irqsave(&asoc_gpio_lock, irq_flags);

    val = act_readl(GPIO_REG_INTC_MASK(gpio));
    val &= ~GPIO_BIT(gpio);
    act_writel(val, GPIO_REG_INTC_MASK(gpio));

    if (val == 0) {
        val = act_readl(INTC_GPIOCTL);
        val &= ~(0x1 << (GPIO_BANK(gpio) * 4 + 1));
        act_writel(val, INTC_GPIOCTL);
    }

    spin_unlock_irqrestore(&asoc_gpio_lock, irq_flags);
}


static void asoc_gpio_irq_unmask(struct irq_data *d)
{
    int gpio = asoc_irq_to_gpio(&asoc_gpio_chip, d->irq);
    unsigned long irq_flags;
    unsigned int val;

    spin_lock_irqsave(&asoc_gpio_lock, irq_flags);

    val = act_readl(GPIO_REG_INTC_MASK(gpio));
    val |= GPIO_BIT(gpio);
    act_writel(val, GPIO_REG_INTC_MASK(gpio));

    val = act_readl(INTC_GPIOCTL);
    val |= 0x1 << (GPIO_BANK(gpio) * 4 + 1);
    act_writel(val, INTC_GPIOCTL);

    spin_unlock_irqrestore(&asoc_gpio_lock, irq_flags);
}

static void asoc_gpio_irq_ack(struct irq_data *d)
{
    unsigned long irq_flags;
    unsigned int val;

    spin_lock_irqsave(&asoc_gpio_lock, irq_flags);

    /* clear GPIO* IRQ pending */
    val = act_readl(INTC_GPIOCTL);
    act_writel(val, INTC_GPIOCTL);
    
    val = act_readl(INTC_GPIOCTL);
    
    spin_unlock_irqrestore(&asoc_gpio_lock, irq_flags);
}

static int asoc_gpio_irq_set_type(struct irq_data *d, unsigned int flow_type)
{
    int gpio = asoc_irq_to_gpio(&asoc_gpio_chip, d->irq);
    unsigned long irq_flags;
    unsigned int type, val;

    spin_lock_irqsave(&asoc_gpio_lock, irq_flags);

    if (flow_type & IRQ_TYPE_EDGE_BOTH)
        __irq_set_handler_locked(d->irq, handle_edge_irq);
    else
        __irq_set_handler_locked(d->irq, handle_level_irq);

    flow_type &= IRQ_TYPE_SENSE_MASK;

    switch (flow_type) {
    case IRQ_TYPE_EDGE_RISING:
        type = GPIO_INT_TYPE_RISING;
        break;
    case IRQ_TYPE_EDGE_FALLING:
        type = GPIO_INT_TYPE_FALLING;
        break;
    case IRQ_TYPE_LEVEL_HIGH:
        type = GPIO_INT_TYPE_HIGH;
        break;
    case IRQ_TYPE_LEVEL_LOW:
        type = GPIO_INT_TYPE_LOW;
        break;
    default:
        pr_err("[GPIO] %s: gpio %d, unknow irq type %d\n",
            __FUNCTION__, gpio, flow_type);
        return -1;
    }

    val = act_readl(INTC_GPIOCTL);
    val &= ~(0x3 << (GPIO_BANK(gpio) * 4 + 2));
    val |= type << (GPIO_BANK(gpio) * 4 + 2);
    act_writel(val, INTC_GPIOCTL);

    spin_unlock_irqrestore(&asoc_gpio_lock, irq_flags);

    return 0;
}

/*
 * When the summary IRQ is raised, any number of GPIO lines may be high.
 * It is the job of the summary handler to find all those GPIO lines
 * which have been set as summary IRQ lines and which are triggered,
 * and to call their interrupt handlers.
 */
static void asoc_gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
    unsigned long bank, gpio_in_bank, pending, msk, gpioctl;
    struct irq_chip *chip = irq_desc_get_chip(desc);

    chained_irq_enter(chip, desc);

    gpioctl = act_readl(INTC_GPIOCTL);

    for (bank = 0; bank < ASOC_GPIO_BANKS; bank++) {
        if (gpioctl & (1 << ((bank * 4) + 0))) {
            /* check pending status in one gpio bank  */
            pending = act_readl(GPIO_REG_INTC_PD(bank * 32));
            msk = act_readl(GPIO_REG_INTC_MASK(bank * 32));
            pending &= msk;
            while (pending != 0) {
                gpio_in_bank = ffs(pending) - 1;

                generic_handle_irq(asoc_gpio_to_irq(&asoc_gpio_chip,
                    bank * 32 + gpio_in_bank));

                pending &= ~GPIO_BIT(gpio_in_bank);
            }
        }
    }

    chained_irq_exit(chip, desc);
}

static struct irq_chip asoc_gpio_irq_chip = {
    .name           = "asoc-gpio-irq",
    .irq_mask       = asoc_gpio_irq_mask,
    .irq_unmask     = asoc_gpio_irq_unmask,
    .irq_ack        = asoc_gpio_irq_ack,
    .irq_set_type   = asoc_gpio_irq_set_type,
};


int __devinit asoc_gpio_init(void)
{
    int i, irq, ret;

    printk("asoc_gpio_init()\n");

    ret = gpiochip_add(&asoc_gpio_chip);
    if (ret < 0)
        return ret;

    for (i = 0; i < asoc_gpio_chip.ngpio; ++i) {
        irq = asoc_gpio_to_irq(&asoc_gpio_chip, i);
        irq_set_chip_and_handler(irq, &asoc_gpio_irq_chip,
                     handle_level_irq);
        set_irq_flags(irq, IRQF_VALID);
    }

    irq_set_chained_handler(IRQ_ASOC_GPIO,
                asoc_gpio_irq_handler);
    return 0;
}