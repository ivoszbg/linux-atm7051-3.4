/*
 * arch/arm/mach-leopard/cpu.c
 *
 * cpu peripheral init for Actions SOC
 *
 * Copyright 2012 Actions Semi Inc.
 * Author: Actions Semi, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/module.h> 
#include <asm/hardware/cache-l2x0.h>
#include <asm/hardware/gic.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/localtimer.h>
#include <asm/smp_twd.h>

#include <mach/hardware.h>
#include <mach/powergate.h>
#include <mach/irqs.h>
#include <mach/debug.h>

#define ASOC_PA_SUSPEND_ROM     (0xb4068000)
#define ASOC_VA_SUSPEND_ROM     (0xFFFF8000)
#define ASOC_SUSPEND_ROM_SIZE 	(SZ_4K)

#define ASOC_PA_BOOT_ROM        (0xFFFF0000)
#define ASOC_VA_BOOT_ROM        (ASOC_VA_SUSPEND_ROM + ASOC_SUSPEND_ROM_SIZE)
#define ASOC_BOOT_ROM_SIZE		(SZ_4K)  

#define ASOC_PA_DDR_RESERVE		(0x80000000)	//the bus phy address (0x8000_0000~0x8000_ffff) transformed into 0x0000_0000~0x0000_ffff in ddr by nic 
#define ASOC_VA_DDR_RESERVE		(ASOC_VA_BOOT_ROM + ASOC_BOOT_ROM_SIZE)
#define ASOC_DDR_RESERVE_SIZE	(SZ_4K)

extern void __init leopard_gp_timer_init(void);

static struct map_desc leopard_io_desc[] __initdata = {    
    {
        .virtual    = IO_ADDRESS(ASOC_PA_REG_BASE),
        .pfn        = __phys_to_pfn(ASOC_PA_REG_BASE),
        .length     = ASOC_PA_REG_SIZE,
        .type       = MT_DEVICE,
    },

	{
        .virtual    = ASOC_VA_DDR_RESERVE,
        .pfn        = __phys_to_pfn(ASOC_PA_DDR_RESERVE),
        .length     = ASOC_DDR_RESERVE_SIZE,
        .type       = MT_DEVICE,
    },
	
    /* for ic version*/
    {
        .virtual    = ASOC_VA_BOOT_ROM,
        .pfn        = __phys_to_pfn(ASOC_PA_BOOT_ROM),
        .length     = ASOC_BOOT_ROM_SIZE,
        .type       = MT_MEMORY,
    },
    
    /*for suspend , to load ddr ops code. by jlingzhang*/
#if 1 
    {
        .virtual    = ASOC_VA_SUSPEND_ROM,
        .pfn        = __phys_to_pfn(ASOC_PA_SUSPEND_ROM),
        .length     = ASOC_SUSPEND_ROM_SIZE,
        .type       = MT_MEMORY,
    },
#endif
    
};

DEFINE_SPINLOCK(leopard_dma_sync_lock);

#define DDR_RESERVE_FOR_DMA_COUNT 16
int *ddr_reserve_for_dma_start=ASOC_VA_DDR_RESERVE+0x400;	//total 2K, reserved for dma from 1K
char ddr_reserve_for_dma_flag[DDR_RESERVE_FOR_DMA_COUNT];

int * leopard_request_dma_sync_addr(void)
{
	int i=0;
	unsigned long flag;
	
	for (i=0; i<DDR_RESERVE_FOR_DMA_COUNT; i++)
	{
		if(ddr_reserve_for_dma_flag[i] == 0)
		{
//			printk("%s,i:%d\n", __FUNCTION__, i);
			spin_lock_irqsave(&leopard_dma_sync_lock, flag);
			ddr_reserve_for_dma_flag[i] = 1;
			spin_unlock_irqrestore(&leopard_dma_sync_lock, flag);
			return ddr_reserve_for_dma_start+i;
		}
	}
	return NULL;
}
EXPORT_SYMBOL(leopard_request_dma_sync_addr);

int leopard_free_dma_sync_addr(int *addr)
{
	int i;
	unsigned long flag;
	
	if(((int)addr < (int)(ddr_reserve_for_dma_start)) || ((int)addr > (int)(ddr_reserve_for_dma_start+DDR_RESERVE_FOR_DMA_COUNT)))
	{
		return -1;
	}
	
	i = ((int)addr - (int)ddr_reserve_for_dma_start)/4;
//	printk("%s,i:%d, addr:0x%x, ddr_reserve_for_dma_start:0x%x\n", __FUNCTION__, i, addr, ddr_reserve_for_dma_start);
	spin_lock_irqsave(&leopard_dma_sync_lock, flag);
	ddr_reserve_for_dma_flag[i] = 0;
	spin_unlock_irqrestore(&leopard_dma_sync_lock, flag);
	return 0;
}
EXPORT_SYMBOL(leopard_free_dma_sync_addr);

int leopard_dma_sync_ddr(int * addr)
{
	volatile int *dma_sync_addr=addr;
    *dma_sync_addr = 0x1;
    *dma_sync_addr = 0x2;
    *dma_sync_addr = 0x3;
    *dma_sync_addr = 0x4;
    *dma_sync_addr = 0x5;
    *dma_sync_addr = 0x55aa;	// fifo max to 6 layer in all master of 702x serial
    while (*dma_sync_addr != 0x55aa);

    return 0;	
}
EXPORT_SYMBOL(leopard_dma_sync_ddr);
	
void __init leopard_map_io(void)
{
    /* dma coherent allocate buffer: 2 ~ 14MB */
    init_consistent_dma_size(14 << 20);
    iotable_init(leopard_io_desc, ARRAY_SIZE(leopard_io_desc));
}

void __init gic_init_irq(void)
{
    gic_init(0, 29, IO_ADDRESS(ASOC_PA_GIC_DIST),
        IO_ADDRESS(ASOC_PA_GIC_CPU));
}

#ifdef CONFIG_CACHE_L2X0
static int __init leopard_l2x0_init(void)
{
    printk("%s()\n", __FUNCTION__);

    /* Instruction prefetch enable
       Data prefetch enable
       Round-robin replacement
       Use AWCACHE attributes for WA
       16KB way size, 16 way associativity
       disable exclusive cache
    */
    l2x0_init(IO_ADDRESS(ASOC_PA_L2CC), 0x3e000000, 0xc00f0fff);

    return 0;
}
early_initcall(leopard_l2x0_init);

/* for resume */
extern void  my_l2x0_init(void __iomem *base, u32 aux_val, u32 aux_mask);
int my_leopard_l2x0_init(void)
{
    printk(KERN_DEBUG "%s()\n", __FUNCTION__);

    /* Instruction prefetch enable
       Data prefetch enable
       Round-robin replacement
       Use AWCACHE attributes for WA
       16KB way size, 16 way associativity
       disable exclusive cache
    */
    my_l2x0_init(IO_ADDRESS(ASOC_PA_L2CC), 0x3e000000, 0xc00f0fff);

    return 0;
}
#endif

#ifdef CONFIG_HAVE_ARM_TWD
static DEFINE_TWD_LOCAL_TIMER(twd_local_timer,
			      ASOC_PA_TWD,
			      IRQ_LOCALTIMER);
static void __init leopard_twd_init(void)
{
    int err = twd_local_timer_register(&twd_local_timer);
    if (err)
        pr_err("twd_local_timer_register failed %d\n", err);
}
#else
#define leopard_twd_init()	do { } while(0)
#endif


static void __init leopard_timer_init(void)
{
    leopard_gp_timer_init();
    leopard_twd_init();
}

struct sys_timer leopard_timer = {
    .init   = leopard_timer_init,
};
