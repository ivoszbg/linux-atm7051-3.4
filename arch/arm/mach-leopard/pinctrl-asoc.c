/*
 * arch/arm/mach-leopard/pinctrl-asoc.c
 *
 * Pinctrl driver for Actions SOC
 *
 * Copyright 2012 Actions Semi Inc.
 * Author: Actions Semi, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include "pinctrl-asoc.h"

#define GPIO_REG_BASE               (GPIO_MFP_PWM_BASE)

#define GPIO_BANK(gpio)             ((gpio) / 32)
#define GPIO_IN_BANK(gpio)          ((gpio) % 32)
#define GPIO_BIT(gpio)              (1 << GPIO_IN_BANK(gpio))

#define GPIO_REG_OUTEN(gpio)        (GPIO_REG_BASE + GPIO_BANK(gpio) * 0xc + 0x0)
#define GPIO_REG_INEN(gpio)         (GPIO_REG_BASE + GPIO_BANK(gpio) * 0xc + 0x4)
#define GPIO_REG_DAT(gpio)          (GPIO_REG_BASE + GPIO_BANK(gpio) * 0xc + 0x8)
#define GPIO_REG_INTC_PD(gpio)      (GPIO_REG_BASE + GPIO_BANK(gpio) * 0x8 + 0x208)
#define GPIO_REG_INTC_MASK(gpio)    (GPIO_REG_BASE + GPIO_BANK(gpio) * 0x8 + 0x20c)

struct asoc_pinctrl {
    struct device *dev;
    struct pinctrl_dev *pctl;
    const struct asoc_pinctrl_soc_info *info;
};

static DEFINE_SPINLOCK(mfpreg_lock);

static int asoc_list_groups(struct pinctrl_dev *pctldev,
            unsigned selector)
{
    struct asoc_pinctrl *apctl = pinctrl_dev_get_drvdata(pctldev);
    const struct asoc_pinctrl_soc_info *info = apctl->info;

    pr_debug("%s(selector:%d)\n", __FUNCTION__, selector);

    if (selector >= info->ngroups)
        return -EINVAL;

    return 0;
}

static const char *asoc_get_group_name(struct pinctrl_dev *pctldev,
            unsigned selector)
{
    struct asoc_pinctrl *apctl = pinctrl_dev_get_drvdata(pctldev);
    const struct asoc_pinctrl_soc_info *info = apctl->info;

    pr_debug("%s(selector:%d)\n", __FUNCTION__, selector);

    return info->groups[selector].name;
}

static int asoc_get_group_pins(struct pinctrl_dev *pctldev, unsigned selector,
            const unsigned **pins, unsigned *num_pins)
{
    struct asoc_pinctrl *apctl = pinctrl_dev_get_drvdata(pctldev);
    const struct asoc_pinctrl_soc_info *info = apctl->info;

    pr_debug("%s(selector:%d)\n", __FUNCTION__, selector);

    if (selector >= info->ngroups)
        return -EINVAL;

    *pins = info->groups[selector].pins;
    *num_pins = info->groups[selector].npins;

    return 0;
}

static void asoc_pin_dbg_show(struct pinctrl_dev *pctldev,
            struct seq_file *s, unsigned offset)
{
    struct asoc_pinctrl *apctl = pinctrl_dev_get_drvdata(pctldev);

    seq_printf(s, "%s", dev_name(apctl->dev));
}

static struct pinctrl_ops asoc_pinctrl_ops = {
    .list_groups = asoc_list_groups,
    .get_group_name = asoc_get_group_name,
    .get_group_pins = asoc_get_group_pins,
    .pin_dbg_show = asoc_pin_dbg_show,
};

static int asoc_pinmux_endisable(struct pinctrl_dev *pctldev,
            unsigned function, unsigned group, bool enable)
{
    struct asoc_pinctrl *apctl = pinctrl_dev_get_drvdata(pctldev);
    const struct asoc_pinctrl_soc_info *info = apctl->info;
    const struct asoc_pinmux_group *g;
    const struct asoc_regcfg *cfg;
    unsigned int i, val, offset;
    unsigned long flags;

    pr_info("%s(enable:%d) function:%d '%s', group:%d '%s'\n",
                __FUNCTION__, enable,
                function, info->functions[function].name,
                group, info->groups[group].name);

    g = &info->groups[group];
    if (g->nregcfgs == 0)
        return 0;

    spin_lock_irqsave(&mfpreg_lock, flags);
	if (enable) {
		pr_debug("for mfp, disable gpio:");
		for (i = 0; i < g->npins; i++) {
			offset = g->pins[i];
			pr_debug("%d,", offset);
			/* disable gpio output */
			act_writel(act_readl(GPIO_REG_OUTEN(offset)) & ~GPIO_BIT(offset),
				GPIO_REG_OUTEN(offset));

			/* disable gpio input */
			act_writel(act_readl(GPIO_REG_INEN(offset)) & ~GPIO_BIT(offset),
				GPIO_REG_INEN(offset));		
		}
		pr_debug("\n");
	}
    for (i = 0; i < g->nregcfgs; i++) {
        cfg = &g->regcfgs[i];

        if (cfg->reg && cfg->mask) {
            pr_debug("  %s() reg 0x%x, mask 0x%x, val 0x%x\n", __FUNCTION__,
                cfg->reg, cfg->mask, cfg->val);

            val = act_readl(cfg->reg);
            val &= ~(cfg->mask);

            if (enable)
                val |= cfg->val;

            act_writel(val, cfg->reg);

            pr_debug("  %s() reg [0x%x] = 0x%x\n", __FUNCTION__,
                cfg->reg, act_readl(cfg->reg));
        }
    }

    spin_unlock_irqrestore(&mfpreg_lock, flags);

    return 0;
}

static int asoc_pinmux_enable(struct pinctrl_dev *pctldev, unsigned function,
               unsigned group)
{
    return asoc_pinmux_endisable(pctldev, function, group, true);
}

static void asoc_pinmux_disable(struct pinctrl_dev *pctldev, unsigned function,
                 unsigned group)
{
    asoc_pinmux_endisable(pctldev, function, group, false);
}

static int asoc_pinmux_list_funcs(struct pinctrl_dev *pctldev,
            unsigned selector)
{
    struct asoc_pinctrl *apctl = pinctrl_dev_get_drvdata(pctldev);
    const struct asoc_pinctrl_soc_info *info = apctl->info;

    pr_debug("%s(selector:%d)\n", __FUNCTION__, selector);

    if (selector >= info->nfunctions)
        return -EINVAL;
    return 0;
}

static const char *asoc_pinmux_get_func_name(struct pinctrl_dev *pctldev,
            unsigned selector)
{
    struct asoc_pinctrl *apctl = pinctrl_dev_get_drvdata(pctldev);
    const struct asoc_pinctrl_soc_info *info = apctl->info;

    pr_debug("%s(selector:%d) name %s\n", __FUNCTION__, selector,
        info->functions[selector].name);

    return info->functions[selector].name;
}

static int asoc_pinmux_get_groups(struct pinctrl_dev *pctldev, unsigned selector,
            const char * const **groups,
            unsigned * const num_groups)
{
    struct asoc_pinctrl *apctl = pinctrl_dev_get_drvdata(pctldev);
    const struct asoc_pinctrl_soc_info *info = apctl->info;

    pr_debug("%s(selector:%d)\n", __FUNCTION__, selector);

    *groups = info->functions[selector].groups;
    *num_groups = info->functions[selector].ngroups;

    return 0;
}

static struct pinmux_ops asoc_pinmux_ops = {
    .list_functions = asoc_pinmux_list_funcs,
    .get_function_name = asoc_pinmux_get_func_name,
    .get_function_groups = asoc_pinmux_get_groups,
    .enable = asoc_pinmux_enable,
    .disable = asoc_pinmux_disable,
};

static struct pinctrl_gpio_range *asoc_match_gpio_range(
            struct pinctrl_dev *pctldev, unsigned pin)
{
    struct asoc_pinctrl *apctl = pinctrl_dev_get_drvdata(pctldev);
    int i;

    for (i = 0; i < apctl->info->gpio_num_ranges; i++) {
        struct pinctrl_gpio_range *range;

        range = &apctl->info->gpio_ranges[i];
        if (pin >= range->pin_base &&
            pin <= (range->pin_base + range->npins - 1))
            return range;
    }
    return NULL;
}

static int asoc_pin_config_get(struct pinctrl_dev *pctldev,
            unsigned pin, unsigned long *config)
{
    struct pinctrl_gpio_range *range = asoc_match_gpio_range(pctldev, pin);

    pr_debug("%s(pin:%d)\n", __FUNCTION__, pin);

    /* We get config for those pins we CAN get it for and that's it */
    if (!range)
        return -EINVAL;

    /* TODO */

    return -ENOTSUPP;
}

static int asoc_pin_config_set(struct pinctrl_dev *pctldev,
            unsigned pin, unsigned long config)
{
    struct pinctrl_gpio_range *range = asoc_match_gpio_range(pctldev, pin);

    pr_debug("%s(pin:%d, config:%ld)\n", __FUNCTION__, pin, config);

    if (!range)
        return -EINVAL;

    /* TODO */

    return -ENOTSUPP;
}

static struct pinconf_ops asoc_pinconf_ops = {
    .pin_config_get = asoc_pin_config_get,
    .pin_config_set = asoc_pin_config_set,
};

static struct pinctrl_desc asoc_pinctrl_desc = {
    .pctlops = &asoc_pinctrl_ops,
    .pmxops = &asoc_pinmux_ops,
    .confops = &asoc_pinconf_ops,
    .owner = THIS_MODULE,
};

int __devinit asoc_pinctrl_probe(struct platform_device *pdev,
                struct asoc_pinctrl_soc_info *info)
{
    struct asoc_pinctrl *apctl;
    int ret;
    int i;

    printk("Actions pinctrl probe\n");

    if (!info || !info->pins || !info->npins) {
        dev_err(&pdev->dev, "wrong pinctrl info\n");
        return -EINVAL;
    }
    info->dev = &pdev->dev;

    /* Create state holders etc for this driver */
    apctl = devm_kzalloc(&pdev->dev, sizeof(*apctl), GFP_KERNEL);
    if (!apctl)
        return -ENOMEM;

    asoc_pinctrl_desc.name = dev_name(&pdev->dev);
    asoc_pinctrl_desc.pins = info->pins;
    asoc_pinctrl_desc.npins = info->npins;

    apctl->info = info;
    apctl->dev = info->dev;

    pr_debug("%s() nfunctions %d, ngroups %d\n",
        __FUNCTION__, info->nfunctions, info->ngroups);

    apctl->pctl = pinctrl_register(&asoc_pinctrl_desc, &pdev->dev, apctl);
    if (!apctl->pctl) {
        dev_err(&pdev->dev, "could not register Actions SOC pinmux driver\n");
        ret = -EINVAL;
        goto out_no_pmx;
    }

    /* We will handle a range of GPIO pins */
    for (i = 0; i < info->gpio_num_ranges; i++)
        pinctrl_add_gpio_range(apctl->pctl, &info->gpio_ranges[i]);

    platform_set_drvdata(pdev, apctl);

    dev_info(&pdev->dev, "initialized Actions SOC pin control driver\n");

    return 0;

out_no_pmx:
    platform_set_drvdata(pdev, NULL);
    devm_kfree(&pdev->dev, apctl);
    return ret;
}

int __devexit asoc_pinctrl_remove(struct platform_device *pdev)
{
    struct asoc_pinctrl *apctl = platform_get_drvdata(pdev);
    int i;

    for (i = 0; i < apctl->info->gpio_num_ranges; i++)
        pinctrl_remove_gpio_range(apctl->pctl, &apctl->info->gpio_ranges[i]);

    pinctrl_unregister(apctl->pctl);
    platform_set_drvdata(pdev, NULL);
    devm_kfree(&pdev->dev, apctl);

    return 0;
}
