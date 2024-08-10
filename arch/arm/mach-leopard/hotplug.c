/*
 * arch/arm/mach-leopard/hotplug.c
 *
 * cpu hotplug stuff for Actions SOC
 *
 * Copyright 2012 Actions Semi Inc.
 * Author: Actions Semi, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h> 
#include <linux/errno.h>
#include <linux/smp.h>

#include <asm/cp15.h>
#include <asm/cacheflush.h>
#include <asm/smp_plat.h>
#include <asm/smp_scu.h>

#include <mach/hardware.h>
#include <mach/dvfslevel.h>

extern volatile int pen_release;
extern unsigned int scu_ncores;

static void __iomem *scu_base_addr(void)
{
    return (void __iomem *)IO_ADDRESS(ASOC_PA_SCU);
}

extern unsigned int smp_ncores;

int chip_4core_test(void)
{
	//printk("chip_4core_test: %d\n", ret);
/* 	int dvfslevel = ASOC_GET_IC(asoc_get_dvfslevel());
	switch(dvfslevel)
	{
		case 0x7029:
		case 0x7021:
			return 1;
		case 0x7023:
			return 0;
		default:
            printk("wrong dvfslevel: %d\n", dvfslevel);
           BUG();
	} */
	if (smp_ncores == 4)
		return 1;
	else if (smp_ncores == 2)
		return 0;
	else {
		printk("wrong smp_ncores: %d\n", smp_ncores);
		BUG();
	}
}

static inline void cpu_enter_lowpower_a5(void)
{
	unsigned int v;

	/*flush_cache_all();*/
	/*
	Invalidate all instruction caches to PoU. Also flushes branch target cache.
	Clean data cache line to PoC by VA.
	Disable data coherency with other cores in the Cortex-A5 MPCore processor.(ACTLR)
	Data caching disabled at all levels.(SCTLR)
	*/
	asm volatile(
	"	mcr	p15, 0, %1, c7, c5, 0\n"
	"	mcr	p15, 0, %1, c7, c10, 4\n"
	/*
	 * Turn off coherency
	 */
	"	mrc	p15, 0, %0, c1, c0, 1\n"
	"	bic	%0, %0, %3\n"
	"	mcr	p15, 0, %0, c1, c0, 1\n"
	"	mrc	p15, 0, %0, c1, c0, 0\n"
	"	bic	%0, %0, %2\n"
	"	mcr	p15, 0, %0, c1, c0, 0\n"
	  : "=&r" (v)
	  : "r" (0), "Ir" (CR_C), "Ir" (0x40)
	  : "cc");
}

static inline void cpu_leave_lowpower(void)
{
	unsigned int v;
	asm volatile(
	"mrc	p15, 0, %0, c1, c0, 0\n"
	"	orr	%0, %0, %1\n"
	"	mcr	p15, 0, %0, c1, c0, 0\n"
	"	mrc	p15, 0, %0, c1, c0, 1\n"
	"	orr	%0, %0, %2\n"
	"	mcr	p15, 0, %0, c1, c0, 1\n"
	  : "=&r" (v)
	  : "Ir" (CR_C), "Ir" (0x40)
	  : "cc");
}

extern void cpu_reset_to_broom( void );
static inline void platform_do_lowpower(unsigned int cpu)
{
	void __iomem *scu_base = scu_base_addr();
	/*cache clean*/
	flush_cache_all();
	
	/*exit smp mode*/
	cpu_enter_lowpower_a5();
	
	/* we put the platform to just WFI */
	for (;;) {
		/*require power off(0x3), and this cpu will shutdown at next wfi*/
		if ((cpu >= 1) && (cpu < NR_CPUS)) {
		  if(!chip_4core_test()) {
				cpu_reset_to_broom();
		  }
		  else {
				scu_power_mode(scu_base, 0x3);
		  }
		}
		
		__asm__ __volatile__("dsb\n\t" "wfi\n\t"
				: : : "memory");
					
		if (pen_release == cpu_logical_map(cpu)) {
			/*
			 * OK, proper wakeup, we're done
			 */
			break;
		}
	}
	cpu_leave_lowpower();
	printk("cpu[%d] power off failed\n", cpu);
}

int platform_cpu_kill(unsigned int cpu)
{
	return 1;
}

/*
 * platform-specific code to shutdown a CPU
 *
 * Called with IRQs disabled
 */
void platform_cpu_die(unsigned int cpu)
{
	/* directly enter low power state, skipping secure registers */
	platform_do_lowpower(cpu);
}

int platform_cpu_disable(unsigned int cpu)
{
	/*
	 * we don't allow CPU 0 to be shutdown (it is still too special
	 * e.g. clock tick interrupts)
	 */
	return cpu == 0 ? -EPERM : 0;
}
