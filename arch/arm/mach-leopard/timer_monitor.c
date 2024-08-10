/*
 * monitor the timer0 to ensure it work well
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <mach/hardware.h>
#include <mach/irqs.h>

#define DRIVER_NAME     "timer_monitor"

static unsigned int last_t0_val = 0;

static void reset_timer0(void)
{
    unsigned int dat;

    pr_warning("[timer monitor] reset timer0\n");

    act_writel(0, T0_CTL);
    dat = act_readl(T0_CTL);
    act_writel(4, T0_CTL);
    dat = act_readl(T0_CTL);

    last_t0_val = act_readl(T0_VAL);
}

static int check_timer0(void)
{
    unsigned int ctl, val;

    ctl = act_readl(T0_CTL);
    val = act_readl(T0_VAL);

    if (!(ctl & 0x4)) {
        pr_warning("[timer monitor] Warning: timer0 not enabled! T0_CTL 0x%x, T0_VAL 0x%x\n",
            ctl, val);
        return -1;
    }

    if (last_t0_val == val) {
       pr_warning("[timer monitor] Warning: timer0 not running! T0_CTL 0x%x, T0_VAL 0x%x\n",
            ctl, val);
        return -1;
    }

    last_t0_val = val;

    return 0;
}

static irqreturn_t asoc_twohz_irq(int irq, void *dev_id)
{
    unsigned int val;

    /* clear IRQ pending */
    val = act_readl(TWOHZ0_CTL);
    act_writel(val, TWOHZ0_CTL);

    if (check_timer0()) {
        reset_timer0();
    };

    return IRQ_HANDLED;
}

static int twohz_start(void)
{
    int ret;
    unsigned int val;

    pr_info("%s: start timer monitor\n", __FUNCTION__);

    /* disable TwoHz */
    act_writel(0x0, TWOHZ0_CTL);
    val = act_readl(TWOHZ0_CTL);

    ret = request_irq(IRQ_ASOC_2HZ0, asoc_twohz_irq, IRQF_TRIGGER_HIGH, "twohz0", NULL);
    if ( ret ) {
        pr_err("%s: failure to requesting irq %d\n", __FUNCTION__, IRQ_ASOC_2HZ0);
        return -EBUSY;
    }

    /* enable TwoHz */
    act_writel(0x3, TWOHZ0_CTL);
    val = act_readl(TWOHZ0_CTL);

    return 0;
}

static void twohz_stop(void)
{
    unsigned int val;

    pr_info("%s: stop timer monitor\n", __FUNCTION__);

    /* disable TwoHz */
    act_writel(0x0, TWOHZ0_CTL);
    val = act_readl(TWOHZ0_CTL);

    free_irq(IRQ_ASOC_2HZ0, NULL);
}

static int timer_monitor_probe(struct platform_device *pdev)
{
    return twohz_start();
}

static int timer_monitor_remove(struct platform_device *pdev)
{
    twohz_stop();
    return 0;
}

#ifdef CONFIG_PM
static int timer_monitor_suspend(struct platform_device *pdev, pm_message_t state)
{
    twohz_stop();
    return 0;
}

static int timer_monitor_resume(struct platform_device *pdev)
{
    return twohz_start();
}
#endif

void timer_monitor_release(struct device *dev)
{
    return;
}

static struct platform_device timer_monitor_device = {
    .name   = DRIVER_NAME,
	.dev	= {
		.release = timer_monitor_release,
    }
};

static struct platform_driver timer_monitor_driver = {
    .driver     = {
        .name       = DRIVER_NAME,
        .owner      = THIS_MODULE,
    },
    .probe      = timer_monitor_probe,
    .remove     = timer_monitor_remove,
#ifdef CONFIG_PM
    .suspend = timer_monitor_suspend,
    .resume = timer_monitor_resume,
#endif
};

static int __init timer_monitor_init(void)
{
    pr_info("%s\n", __FUNCTION__);

    platform_device_register(&timer_monitor_device);
    return platform_driver_register(&timer_monitor_driver);
}

static void __exit timer_monitor_exit(void)
{
    pr_info("%s\n", __FUNCTION__);
    platform_driver_unregister(&timer_monitor_driver);
    platform_device_unregister(&timer_monitor_device);
}


module_init(timer_monitor_init);
module_exit(timer_monitor_exit);

MODULE_AUTHOR("Actions Semi Inc");
MODULE_DESCRIPTION("timer monitor");
MODULE_LICENSE("GPL");
