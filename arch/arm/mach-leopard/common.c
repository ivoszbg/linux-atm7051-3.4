/*
 * arch/arm/mach-leopard/common.c
 *
 * common device register for leopard
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
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/bootmem.h>
#include <linux/memblock.h>
#include <linux/ion.h>
#include <linux/asoc_ion.h>     /* ASOC_ION_HEAP_CARVEOUT */
#include <asm/setup.h>

#include <mach/hardware.h>
#include <mach/irqs.h>

#include <linux/module.h>
#include <linux/dma-contiguous.h>
extern phys_addr_t arm_lowmem_limit;

unsigned int asoc_ddr_size = 0;
unsigned int asoc_gpu_start, asoc_gpu_size;
unsigned int asoc_ion_start, asoc_ion_size;
unsigned int asoc_fb_start, asoc_fb_size;
unsigned int asoc_fb1_start, asoc_fb1_size;	
unsigned int asoc_qb_gpu_start, asoc_qb_gpu_size;

/* parse ATAG_PMEM_INFO */
extern void register_board_ddr_size(unsigned long size);
static int __init parse_tag_pmem_info(const struct tag *tag)
{
	asoc_ddr_size = tag->u.pmem_info.ddr_size;
	asoc_fb_size = tag->u.pmem_info.fb_size;
	asoc_fb1_size = tag->u.pmem_info.fb1_size;
	asoc_gpu_size = tag->u.pmem_info.gpu_size;
	asoc_ion_size = tag->u.pmem_info.ion_size;

	register_board_ddr_size(asoc_ddr_size);
	printk("ddr_size 0x%x, fb_size 0x%x, fb1_size 0x%x\n", asoc_ddr_size, asoc_fb_size, asoc_fb1_size);
	printk("gpu_size 0x%x, ion_size 0x%x\n", asoc_gpu_size, asoc_ion_size);

	return 0;
}

__tagtable(ATAG_PMEM_INFO, parse_tag_pmem_info);


#define ACTIONS_ION_HEAP_NUM	3
static struct ion_platform_data asoc_ion_data = {
	.nr = ACTIONS_ION_HEAP_NUM,
	.heaps = {
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = ION_HEAP_ID_FB,
			.name = "ion_fb",
			.base = 0,	/* Filled in by liger_reserve() */
			.size = 0,	/* Filled in by liger_reserve() */
			.has_outer_cache = 1,
		},
#ifdef CONFIG_CMA
		{
			.type = ION_HEAP_TYPE_CMA,
			.id = ION_HEAP_ID_PMEM,
			.name = "ion_pmem",
			.base = 0,	/* Filled in by liger_reserve() */
			.size = 0,	/* Filled in by liger_reserve() */
			.has_outer_cache = 1,
		},
#else		
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = ION_HEAP_ID_PMEM,
			.name = "ion_pmem",
			.base = 0,	/* Filled in by liger_reserve() */
			.size = 0,	/* Filled in by liger_reserve() */
			.has_outer_cache = 1,
		},
#endif		
		{
			.type = ION_HEAP_TYPE_SYSTEM,
			.id = ION_HEAP_ID_SYSTEM,
			.name = "ion_system",
			.has_outer_cache = 1,
		},
	},
};

static struct platform_device asoc_ion_device = {
    .name = "ion-asoc",
    .id = -1,
    .dev = {
        .platform_data = &asoc_ion_data,
        .coherent_dma_mask = -1,
    },
};

static const unsigned int ddr_cap[] =
    {32, 64, 128, 256, 512, 1024, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0};

unsigned int get_mem_size(void)
{
    unsigned int i, mem_size;

    i = (act_readl(DCU_FEA) >> 4) & 0xf;
    if (i >= sizeof(ddr_cap)) {
        printk("[leopard] invalid memory size, DDR_FEA: 0x%x\n",
            act_readl(DCU_FEA));
        BUG_ON(1);
    }

    mem_size = ddr_cap[i];

    printk("[leopard] memory size %d MB\n", mem_size);

    return (mem_size << 20);
}
EXPORT_SYMBOL(get_mem_size);

unsigned int asoc_get_ion_size(void)
{
    return asoc_ion_size;
}
EXPORT_SYMBOL(asoc_get_ion_size);

/* framebuffer0 */
static struct resource asoc_fb_res[] = {
    {
        .name   = "fbmem0",
        .start  = 0,    /* Filled in by leopard_reserve() */
        .end    = 0,    /* Filled in by leopard_reserve() */
        .flags  = IORESOURCE_MEM,
    },
    {
        .name   = "fbmem1",
        .start  = 0,    /* Filled in by leopard_reserve() */
        .end    = 0,    /* Filled in by leopard_reserve() */
        .flags  = IORESOURCE_MEM,
    },
};

static struct platform_device asoc_fb_device = {
    .name = "act_fb",
    .resource = asoc_fb_res,
    .num_resources = ARRAY_SIZE(asoc_fb_res),
};

/* GPU */
static struct resource asoc_gpu_res[] = {
    {
        .name   = "gpu_irq",
        .start = IRQ_ASOC_GPU_3D,
        .end = IRQ_ASOC_GPU_3D,
        .flags = IORESOURCE_IRQ,
    },
    {
        .name  = "gpu_base",
        .start = GPU_BASE,
        .end   = GPU_BASE + 0xffff,
        .flags = IORESOURCE_MEM,
    },
//    {
//        .name   = "gpu_mem",
//        .start  = 0,    /* Filled in by leopard_reserve() */
//        .end    = 0,    /* Filled in by leopard_reserve() */
//        .flags  = IORESOURCE_MEM,
//    },
};


static struct platform_device asoc_gpu_device = {
    .name = "pvrsrvkm",
    .resource = asoc_gpu_res,
    .num_resources = ARRAY_SIZE(asoc_gpu_res),
};

void __init leopard_fixup(struct tag *tags, char **from,
            struct meminfo *meminfo)
{
    u32 size;

    size = get_mem_size();
    
    meminfo->nr_banks=1;
    meminfo->bank[0].start = PHYS_OFFSET;
    printk("PHYS_OFFSET: 0x%x\n", PHYS_OFFSET);
    if(PHYS_OFFSET == 0)
        meminfo->bank[0].size = size-PHYS_OFFSET;
    else
        meminfo->bank[0].size = 64 * SZ_1M;
    printk("Total Memory Size: %uMB 0x%x\n",
        size >> 20, meminfo->bank[0].size );
}

void __init leopard_reserve(void)
{
    struct resource *res;
    bool is_qb_kernel = false;

    if(PHYS_OFFSET != 0)
        is_qb_kernel = true;

    if ( asoc_fb_size == 0 && asoc_ion_size == 0 ) // upgrade
		return ;

    if(!is_qb_kernel) {
        /* 
	 * framebuffer0
	 * asoc_fb_size parameter passed by the boot stage 
	 * */
        if (asoc_fb_size == 0)
            asoc_fb_size = CONFIG_FB_RESERVED_SIZE * SZ_1M;
    
        asoc_fb_start = memblock_end_of_DRAM() - asoc_fb_size;
    
        res = platform_get_resource_byname(&asoc_fb_device,
                         IORESOURCE_MEM, "fbmem0");
        if (res) {
            res->start = asoc_fb_start;
            res->end = asoc_fb_start + asoc_fb_size - 1;
        }
    
        memblock_remove(asoc_fb_start, asoc_fb_size);
    
	/* framebuffer1 */
	if (asoc_fb1_size > 0) {
		asoc_fb1_start = memblock_end_of_DRAM() - asoc_fb1_size;

		res = platform_get_resource_byname(&asoc_fb_device,
				IORESOURCE_MEM, "fbmem1");
		if (res) {
			res->start = asoc_fb1_start;
			res->end = asoc_fb1_start + asoc_fb1_size - 1;
		}
		memblock_remove(asoc_fb1_start, asoc_fb1_size);
	}

    	unsigned int mem_end_low;
	    /* GPU */
			
      /* ION */
      if (asoc_ion_size == 0) {
        if (get_mem_size() > (512 * SZ_1M))
            asoc_ion_size = 160 * SZ_1M;   /* 160MB */
        else
            asoc_ion_size = 102 * SZ_1M;   /* 102MB */
      }
      asoc_ion_size = ALIGN(asoc_ion_size,(8 * SZ_1M)); // base and size align 8MB
      asoc_ion_start = memblock_end_of_DRAM() - asoc_ion_size;

      asoc_ion_data.heaps[0].base = asoc_fb_start;
      asoc_ion_data.heaps[0].size = asoc_fb_size;
#ifdef CONFIG_CMA
      //mem_end_low = arm_lowmem_limit;
      //asoc_ion_size += 8 * SZ_1M;
      //asoc_ion_start = min(asoc_ion_start, mem_end_low) - asoc_ion_size - 16*SZ_1M;	//reserve 16M for base and size align.
      asoc_ion_start &= 0xff800000; // base and size align 8MB
      printk("pfn asoc_ion_start: 0x%x, asoc_ion_size: 0x%x\n", asoc_ion_start, asoc_ion_size);
      int ret = dma_declare_contiguous(&(asoc_ion_device.dev), asoc_ion_size,
		      asoc_ion_start, asoc_ion_start+asoc_ion_size);
      if(ret != 0) {
	      printk("dma_declare_contiguous failed to reserve mem base: 0x%08x, size: 0x%08x\n",
			      asoc_ion_start, asoc_ion_size);
      } else {
	      printk("dma_declare_contiguous successed to reserve mem base: 0x%08x, size: 0x%08x\n",
			      asoc_ion_start, asoc_ion_size);
      }
      printk("&asoc_ion_device: 0x%08x\n", &asoc_ion_device);
      asoc_ion_data.heaps[1].base = asoc_ion_start;
      asoc_ion_data.heaps[1].size = asoc_ion_size;
#else        
      asoc_ion_data.heaps[1].base = asoc_ion_start;
      asoc_ion_data.heaps[1].size = asoc_ion_size;
      memblock_remove(asoc_ion_start, asoc_ion_size);
#endif        
    } else {
      asoc_ion_size = 0;
      asoc_ion_start = memblock_end_of_DRAM() - asoc_ion_size;

      asoc_ion_data.heaps[0].base = asoc_fb_start;
      asoc_ion_data.heaps[0].size = asoc_fb_size;

      asoc_ion_data.heaps[1].base = asoc_ion_start;
      asoc_ion_data.heaps[1].size = asoc_ion_size;
    }

    printk("Reserved memory %uMB\n",
        (asoc_ion_size + asoc_fb_size) >> 20);
    printk("   ION:       0x%08x, %uMB\n"
           "    FB:       0x%08x, %uMB\n"
           "    FB1:       0x%08x, %uMB\n",
           asoc_ion_start, asoc_ion_size >> 20,
           asoc_fb_start, asoc_fb_size >> 20,
           asoc_fb1_start, asoc_fb1_size >> 20);
}

static struct platform_device *asoc_common_devices[] __initdata = {
    &asoc_ion_device,
    &asoc_fb_device,
    &asoc_gpu_device,
};


void __init asoc_common_init(void)
{
    printk("%s()\n", __FUNCTION__);

    platform_add_devices(asoc_common_devices,
                   ARRAY_SIZE(asoc_common_devices));
}

static  int boot_mode = BOOT_MODE_NORMAL;
static int __init boot_mode_process(char *str)
{
	printk("boot_mode_process\n");
	if (str == NULL || *str == '\0')
		return 0;
		
	if (strcmp(str, "upgrade") == 0) {
		printk("upgrade boot mode\n");
		boot_mode = BOOT_MODE_UPGRADE;
	}else if (strcmp(str, "charger") == 0) {
		printk("minicharger boot mode\n");
		boot_mode = BOOT_MODE_CHARGER;
	}else if(strcmp(str, "recovery1") == 0 || strcmp(str, "recovery2") == 0 ){
		printk("recovery boot mode\n");
		boot_mode = BOOT_MODE_RECOVERY;
	}else{
		printk("Normal boot mode\n");
		boot_mode = BOOT_MODE_NORMAL;
	}	
	printk("boot_mode=%d, str=%s\n", boot_mode, str);
	return 1;
}
__setup("androidboot.mode=", boot_mode_process);

/*return the mode
	0:normal
	1:produce
	2:minicharge
	4:recovery
*/
int asoc_get_boot_mode(void)
{
	//printk("boot mode = %d\n", boot_mode);
	return boot_mode;
}
EXPORT_SYMBOL_GPL(asoc_get_boot_mode);


arch_initcall(asoc_common_init);
