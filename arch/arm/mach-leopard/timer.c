/*
 * arch/arm/mach-leopard/timer.c
 *
 * time0 use as clocksource
 * timer1 for time tick at boot stage 
 *
 * Copyright 2012 Actions Semi Inc.
 * Author: Actions Semi, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <asm/sched_clock.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/clock.h>

/*
 * clocksource
 */
static cycle_t leopard_read_timer(struct clocksource *cs)
{
	return (cycle_t)act_readl(T0_VAL);
}

static cycle_t suspended_timer_count;

static void leopard_timer_suspend(struct clocksource *cs)
{
    suspended_timer_count = cs->read(cs);

    printk(KERN_DEBUG "%s: suspended_timer_count %x\n", 
        __FUNCTION__, suspended_timer_count);
}

static void leopard_timer_resume(struct clocksource *cs)
{
    printk(KERN_DEBUG "%s: suspended_timer_count %x\n", 
        __FUNCTION__, suspended_timer_count);

    // act_writel(0, T0_CTL);
    // act_writel((u32)suspended_timer_count, T0_VAL);
    // act_writel(4, T0_CTL);

    // pr_info("%s: T0_VAL %x\n", 
        // __FUNCTION__, act_readl(T0_VAL));
}

static struct clocksource leopard_clksrc = {
    .name		= "timer0",
    .rating		= 200,
    .read		= leopard_read_timer,
    .mask		= CLOCKSOURCE_MASK(32),
    .shift		= 20,
    .flags		= CLOCK_SOURCE_IS_CONTINUOUS,
    .suspend	= leopard_timer_suspend,
    .resume		= leopard_timer_resume,
};


/*
 * Using this local implementation sched_clock which uses timer0
 * to get some better resolution when scheduling the kernel.
 */
static u32 notrace asoc_read_sched_clock(void)
{
	return act_readl(T0_VAL);
}

/* Clockevent device: use one-shot mode */
static void leopard_clkevt_mode(enum clock_event_mode mode,
			     struct clock_event_device *dev)
{
	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		pr_err("%s: periodic mode not supported\n", __func__);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		act_writel(0, T1_CTL);
		act_writel(0, T1_VAL);
        act_writel(0, T1_CMP);
		break;
	case CLOCK_EVT_MODE_SHUTDOWN:
	case CLOCK_EVT_MODE_UNUSED:
		/* disable irq */
		act_writel(0, T1_CTL);
		break;
	case CLOCK_EVT_MODE_RESUME:
		break;
	}
}

static int leopard_clkevt_next(unsigned long evt, struct clock_event_device *ev)
{
    /* disable timer */
    act_writel(0x0, T1_CTL);

	/* writing the value has immediate effect */
    act_writel(0, T1_VAL);
    act_writel(evt, T1_CMP);

    /* enable timer & IRQ */
    act_writel(0x6, T1_CTL);

	return 0;
}

static struct clock_event_device leopard_clkevt = {
	.name		= "timer1",
	.features	= CLOCK_EVT_FEAT_ONESHOT,
    .rating     = 200,
	.set_mode	= leopard_clkevt_mode,
	.set_next_event	= leopard_clkevt_next,
};

/*
 * IRQ Handler for timer 1 of the MTU block.
 */
static irqreturn_t leopard_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evdev = dev_id;

	act_writel(1 << 0, T1_CTL); /* Interrupt clear reg */
	evdev->event_handler(evdev);

	return IRQ_HANDLED;
}

static struct irqaction leopard_timer_irq = {
	.name		= "timer1_tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER,
	.handler	= leopard_timer_interrupt,
	.dev_id		= &leopard_clkevt,
};

void __init leopard_gp_timer_init(void)
{
    struct clk *timer_clk;
	unsigned long rate;

    timer_clk = clk_get_sys(CLK_NAME_TIMER_CLK, NULL);
	if (IS_ERR(timer_clk)) { 
        printk("%s() cannot get timer clk, err %ld\n", 
            __FUNCTION__, PTR_ERR(timer_clk));
    }

    clk_enable(timer_clk);
    rate = clk_get_rate(timer_clk);

    printk("[LEOPARD] timer rate %ld Hz\n", rate);

	/* Timer 0 is the free running clocksource */
	act_writel(0, T0_CTL);
    act_writel(0, T0_VAL);
	act_writel(0, T0_CMP);	
	act_writel(4, T0_CTL);

    setup_sched_clock(asoc_read_sched_clock, 32, rate);
    clocksource_register_hz(&leopard_clksrc, rate);

	/* Timer 1 is used for events, fix according to rate */
	act_writel(0, T1_CTL);
    act_writel(0, T1_VAL);
	act_writel(0, T1_CMP);	

	setup_irq(IRQ_ASOC_TIMER1, &leopard_timer_irq);
    leopard_clkevt.cpumask = cpumask_of(0);
    clockevents_config_and_register(&leopard_clkevt, rate,
					0xf, 0xffffffff);
}

