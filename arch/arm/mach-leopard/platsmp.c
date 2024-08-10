/*
 * arch/arm/mach-leopard/platsmp.c
 *
 * Platform file needed for Leopard. This file is based on arm
 * realview smp platform.
 *
 * Copyright 2012 Actions Semi Inc.
 * Author: Actions Semi, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/smp.h>
#include <linux/io.h>

#include <asm/cacheflush.h>
#include <asm/localtimer.h>
#include <asm/smp_plat.h>
#include <asm/smp_scu.h>

#include <mach/hardware.h>
#include <mach/powergate.h>
#include <mach/smp.h>

#define CPUX_BOOT_FLAG                              (0x55aa)


static unsigned int cpux_flag_regs[CONFIG_NR_CPUS] = {
    0xb0158050,      /* CPU1 flag reg */
    0xb0158058,      /* CPU2 flag reg, modify for 5202c*/	
    0xb0158060,      /* CPU3 flag reg */    
};

/*
 * write the address of secondary startup into the boot ram register, 
 * and the flag to the boot ram register
 * at offset 0x200/0x300, which is what boot rom code is waiting for.
 * This would wake up the secondary core from WFE
 *
 * boot wakeup data:
 * flag_offs:      flag
 * flag_offs + 4:  boot function pointer 
 */
static int cpux_set_wakeup_data(unsigned int cpu, void *boot_func)
{
    unsigned int flag_offs, func_offs;
    
    if (cpu < 1 || cpu >= CONFIG_NR_CPUS)
        return -EINVAL;

    func_offs = cpux_flag_regs[cpu - 1];
    flag_offs = func_offs + 4;

    if (boot_func != NULL) {
        __raw_writel(virt_to_phys(boot_func), IO_ADDRESS(func_offs));
        __raw_writel(CPUX_BOOT_FLAG, IO_ADDRESS(flag_offs));

    } else {
        __raw_writel(0, IO_ADDRESS(func_offs));
        __raw_writel(0, IO_ADDRESS(flag_offs));
    }

    return 0;
}

#define CPUX_WFE_FLAG 0x2222
static int wait_cpux_wfe(unsigned int cpu)
{
    unsigned int flag_offs, func_offs;
	unsigned int flag_val = 1;
    
    if (cpu < 1 || cpu >= CONFIG_NR_CPUS)
        return -EINVAL;

    func_offs = cpux_flag_regs[cpu - 1];
    flag_offs = func_offs + 4;

	while (flag_val) {
		udelay(200);
		flag_val = __raw_readl(IO_ADDRESS(flag_offs));
		if (flag_val == CPUX_WFE_FLAG)
			flag_val = 0;
		else 
			flag_val = 1;
	}
    return 0;
}

static DEFINE_SPINLOCK(boot_lock);

static void __iomem *scu_base_addr(void)
{
    return (void __iomem *)IO_ADDRESS(ASOC_PA_SCU);
}

/*
 * control for which core is the next to come out of the secondary
 * boot "holding pen"
 */
volatile int pen_release = -1;

/*
 * Write pen_release in a way that is guaranteed to be visible to all
 * observers, irrespective of whether they're taking part in coherency
 * or not.  This is necessary for the hotplug code to work reliably.
 */
static void write_pen_release(int val)
{
    pen_release = val;
    smp_wmb();
    __cpuc_flush_dcache_area((void *)&pen_release, sizeof(pen_release));
    outer_clean_range(__pa(&pen_release), __pa(&pen_release + 1));
}

void __cpuinit platform_secondary_init(unsigned int cpu)
{
    trace_hardirqs_off();

    /*
     * if any interrupts are already enabled for the primary
     * core (e.g. timer irq), then they will not have been enabled
     * for us: do so
     */
    gic_secondary_init(0);

    /*
     * let the primary processor know we're out of the
     * pen, then head off into the C entry point
     */
    write_pen_release(-1);

    /*
     * Synchronise with the boot thread.
     */
    spin_lock(&boot_lock);
    spin_unlock(&boot_lock);
}

static void wakeup_secondary(unsigned int cpu)
{
    enum asoc_powergate_id pgid;

	if(chip_4core_test())
	{
		pgid = asoc_cpu_powergate_id(cpu);
		asoc_powergate_power_on(pgid);
    }
    
    /* wait CPUx run to WFE instruct */
    /* udelay(200); */
    wait_cpux_wfe(cpu);

    /* set wakeup register data */
    cpux_set_wakeup_data(cpu, asoc_secondary_startup);

    /*
     * Send a 'sev' to wake the secondary core from WFE.
     * Drain the outstanding writes to memory
     */
    dsb_sev();
    mb();
}

int __cpuinit boot_secondary(unsigned int cpu, struct task_struct *idle)
{
    unsigned long timeout;

    wakeup_secondary(cpu);

    /* wait for CPUx wakeup */
    udelay(10);

    /*
     * set synchronisation state between this boot processor
     * and the secondary one
     */
    spin_lock(&boot_lock);

    /*
     * The secondary processor is waiting to be released from
     * the holding pen - release it, then wait for it to flag
     * that it has been released by resetting pen_release.
     */
    write_pen_release(cpu_logical_map(cpu));
    smp_send_reschedule(cpu);

    timeout = jiffies + (1 * HZ);
    while (time_before(jiffies, timeout)) {
        if (pen_release == -1)
            break;
    }

    /* clear wakeup register data */
    cpux_set_wakeup_data(cpu, (void *)0);

    /*
     * now the secondary core is starting up let it run its
     * calibrations, then wait for it to finish
     */
    spin_unlock(&boot_lock);

    return pen_release != -1 ? -ENOSYS : 0;
}

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
unsigned int smp_ncores;
void __init smp_init_cpus(void)
{
    void __iomem *scu_base = scu_base_addr();
    unsigned int i, ncores;

    ncores = scu_base ? scu_get_core_count(scu_base) : 1;
	smp_ncores = ncores;
	printk("smp_ncores:%d\n", smp_ncores);
    /* sanity check */
    if (ncores > nr_cpu_ids) {
        ncores = nr_cpu_ids;
    }

    for (i = 0; i < ncores; i++)
        set_cpu_possible(i, true);

    set_smp_cross_call(gic_raise_softirq);
}

void __init platform_smp_prepare_cpus(unsigned int max_cpus)
{
    scu_enable(scu_base_addr());
}
