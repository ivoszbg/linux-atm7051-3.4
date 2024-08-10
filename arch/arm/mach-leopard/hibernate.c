/*
 * Hibernation support specific for ARM
 *
 * Derived from work on ARM hibernation support by:
 *
 * Ubuntu project, hibernation support for mach-dove
 * Copyright (C) 2010 Nokia Corporation (Hiroshi Doyu)
 * Copyright (C) 2010 Texas Instruments, Inc. (Teerth Reddy et al.)
 *	https://lkml.org/lkml/2010/6/18/4
 *	https://lists.linux-foundation.org/pipermail/linux-pm/2010-June/027422.html
 *	https://patchwork.kernel.org/patch/96442/
 *
 * Copyright (C) 2006 Rafael J. Wysocki <rjw@sisk.pl>
 *
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/suspend.h>
#include <asm/tlbflush.h>
#include <asm/cacheflush.h>
#include <asm/pgalloc.h>
#include <asm/sections.h>

#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/serial_core.h>
#include <linux/power_supply.h>
#include <linux/io.h>
#include <mach/atc260x/atc260x.h>
#include <mach/hardware.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <asm/idmap.h>

extern const void __nosave_begin, __nosave_end;
extern void cpu_resume(void);
extern unsigned long sleep_save_sp;
extern struct pbe *restore_pblist;

int pfn_is_nosave(unsigned long pfn)
{
	unsigned long nosave_begin_pfn = __pa_symbol(&__nosave_begin) >> PAGE_SHIFT;
	unsigned long nosave_end_pfn = PAGE_ALIGN(__pa_symbol(&__nosave_end)) >> PAGE_SHIFT;

	return (pfn >= nosave_begin_pfn) && (pfn < nosave_end_pfn);
}

void notrace save_processor_state(void)
{
	flush_thread();
	local_fiq_disable();
}

void notrace restore_processor_state(void)
{
	flush_tlb_all();
	flush_cache_all();
	local_fiq_enable();
}

u8 __swsusp_resume_stk[PAGE_SIZE/2] __nosavedata;
u32 __swsusp_save_sp;

unsigned int    pa_swsusp_save_sp;
unsigned int    pa_stext;
unsigned int    pa_etext;
unsigned int    pa_sdata;
unsigned int    pa_edata;
unsigned int    phy_cpu_reset;
unsigned int    phy_cpu_resume;
unsigned int    pa_nosave_begin;
unsigned int    pa_nosave_end;
unsigned int    pa_contin_pfn;

int __swsusp_arch_resume_finish(void)
{
	identity_mapping_del(swapper_pg_dir, __pa(_stext), __pa(_etext));
	identity_mapping_del(swapper_pg_dir, __pa(_sdata), __pa(_edata));
	identity_mapping_del(swapper_pg_dir, PFN_ALIGN(pa_swsusp_save_sp) - PAGE_SIZE, PFN_ALIGN(pa_swsusp_save_sp) + PAGE_SIZE);
	return 0;
}

extern void leopard_clean_l2_cache_all(void);
/*
 * The framework loads the hibernation image into a linked list anchored
 * at restore_pblist, for swsusp_arch_resume() to copy back to the proper
 * destinations.
 *
 * To make this work if resume is triggered from initramfs, the
 * pagetables need to be switched to allow writes to kernel mem.
 */
void __swsusp_arch_save_env(int contin_start_pfn)
{
    pa_swsusp_save_sp = __swsusp_save_sp;
    pa_stext = __pa(_stext);
    pa_etext = __pa(_etext);
    pa_sdata = __pa(_sdata);
    pa_edata = __pa(_edata);
    phy_cpu_reset = virt_to_phys(cpu_reset);
    phy_cpu_resume = virt_to_phys(cpu_resume);
    pa_nosave_begin = __pa_symbol(&__nosave_begin);
    pa_nosave_end = __pa_symbol(&__nosave_end);
    pa_contin_pfn = contin_start_pfn;
}

void notrace __swsusp_arch_restore_image(void)
{
	struct pbe *pbe;
	typeof(cpu_reset) *phys_reset = phy_cpu_reset;

	cpu_switch_mm(swapper_pg_dir, &init_mm);
	
	for (pbe = restore_pblist; pbe; pbe = pbe->next)
		copy_page(pbe->orig_address, pbe->address);
	flush_tlb_all();
	flush_cache_all();

	sleep_save_sp = pa_swsusp_save_sp;
	
	identity_mapping_add(swapper_pg_dir, pa_stext, pa_etext);
	identity_mapping_add(swapper_pg_dir, pa_sdata, pa_edata);
	identity_mapping_add(swapper_pg_dir, PFN_ALIGN(pa_swsusp_save_sp) - PAGE_SIZE, PFN_ALIGN(pa_swsusp_save_sp) + PAGE_SIZE);
       
	flush_tlb_all();
	flush_cache_all();
    leopard_clean_l2_cache_all();
    
#if 0
#ifndef CONFIG_SMP
	cpu_proc_fin();
	flush_tlb_all();
	flush_cache_all();
#endif
#endif

	phys_reset(phy_cpu_resume);
}


typedef struct{
	const char *name;
	struct regulator *regulator;
	int uv;
} asoc_regulator_t;

static asoc_regulator_t asoc_regulators[] = {
    {"vddcore", 0, 0},
    {"vddr", 0, 0},
    {"vcc", 0, 0},
    {"avddcore", 0, 0},
};

int hibernate_power_init(void)
{
    int i;

    for(i = 0; i < sizeof(asoc_regulators) / sizeof(asoc_regulator_t); i++)
        asoc_regulators[i].regulator = regulator_get(NULL, asoc_regulators[i].name);
    return 0;
}

int hibernate_power_remove(void)
{
    int i;

    for(i = 0; i < sizeof(asoc_regulators) / sizeof(asoc_regulator_t); i++)
    {
		if(IS_ERR(asoc_regulators[i].regulator)) {
			continue;
	    }
	    regulator_put(asoc_regulators[i].regulator);
    }
    return 0;
}

static int save_asoc_power_regs(void)
{
    int i;
    
    for(i = 0; i < sizeof(asoc_regulators) / sizeof(asoc_regulator_t); i++)
    {
		if(IS_ERR(asoc_regulators[i].regulator)) {
			continue;
	    }
	    asoc_regulators[i].uv = regulator_get_voltage(asoc_regulators[i].regulator);
    }
    
    return 0;
}


static int restore_asoc_power_regs(void)
{
    int i;
    
    for(i = 0; i < sizeof(asoc_regulators) / sizeof(asoc_regulator_t); i++)
    {
		if(IS_ERR(asoc_regulators[i].regulator)) {
			continue;
	    }	    
	    regulator_set_voltage(asoc_regulators[i].regulator, asoc_regulators[i].uv, asoc_regulators[i].uv);	    	    
    }
   
    return 0;
}

typedef struct {
	unsigned int reg_addr;
	unsigned int reg_val;
	unsigned int reg_mask;
} leopard_reg_t;

static leopard_reg_t leopard_regs[] = {
	/* <1> CMU register */
	{CMU_COREPLL, 0x0, ~(0x0)},
	{CMU_BUSCLK, 0x0, ~(0x0)},
    {CMU_DEVPLL, 0x0, ~(0x0)},
//    {CMU_NANDPLL, 0x0, ~(0x0)},
    {CMU_DISPLAYPLL, 0x0, ~(0x0)},
    {CMU_AUDIOPLL, 0x0, ~(0x0)},
    {CMU_TVOUTPLL, 0x0, ~(0x0)},
    {CMU_SENSORCLK, 0x0, ~(0x0)},
    {CMU_LCDCLK, 0x0, ~(0x0)},
    {CMU_DECLK, 0x0, ~(0x0)},
    {CMU_SICLK, 0x0, ~(0x0)},
    {CMU_VDECLK, 0x0, ~(0x0)},
    {CMU_VCECLK, 0x0, ~(0x0)},
    {CMU_GPUCLK, 0x0, ~(0x0)},
//    {CMU_NANDCCLK, 0x0, ~(0x0)},
    {CMU_SD0CLK, 0x0, ~(0x0)},
    {CMU_SD1CLK, 0x0, ~(0x0)},
    {CMU_SD2CLK, 0x0, ~(0x0)},
    {CMU_UART0CLK, 0x0, ~(0x0)},
    {CMU_UART1CLK, 0x0, ~(0x0)},
    {CMU_UART2CLK, 0x0, ~(0x0)},
    {CMU_DMACLK, 0x0, ~(0x0)},
    {CMU_PWM0CLK, 0x0, ~(0x0)},   
    {CMU_PWM1CLK, 0x0, ~(0x0)},
    {CMU_PWM2CLK, 0x0, ~(0x0)},
    {CMU_PWM3CLK, 0x0, ~(0x0)},
    {CMU_USBCLK, 0x0, ~(0x0)},
    {CMU_120MPLL, 0x0, ~(0x0)},
    {CMU_DEVCLKEN0, 0x0, ~(0x1 << 9)},
    {CMU_DEVCLKEN1, 0x0, ~(0x0)},
    {CMU_DEVRST0, 0x0, ~(0x0)},
    {CMU_DEVRST1, 0x0, ~(0x0)},
    {CMU_UART3CLK, 0x0, ~(0x0)},
    {CMU_UART4CLK, 0x0, ~(0x0)},
    {CMU_UART5CLK, 0x0, ~(0x0)},
 
    /*Timer*/
    {T0_CTL, 0x0, ~(0x0)},
	{T0_CMP, 0x0, ~(0x0)},
	{T0_VAL, 0x0, ~(0x0)},
	{TWOHZ0_CTL, 0x0, ~(0x0)},

    
	/* <3> DMA configuration register */
	{DMA_CTL, 0x0, ~(0x0)},
	{DMA_IRQEN, 0x0, ~(0x0)},
//	{DMA_IRQPD, 0x0, ~(0x0)},
	{DMA_PAUSE, 0x0, ~(0x0)},
//	{CMU_DMACLK, 0x0, ~(0x0)},

	/* <4> Multiplex pin control register */
	{GPIO_AOUTEN, 0x0, ~(0x0)},
	{GPIO_AINEN, 0x0, ~(0x0)},
	{GPIO_ADAT, 0x0, ~(0x0)},
	{GPIO_BOUTEN, 0x0, ~(0x0)},
	{GPIO_BINEN, 0x0, ~(0x0)},
	{GPIO_BDAT, 0x0, ~(0x0)},
	{GPIO_COUTEN, 0x0, ~(0x0)},
	{GPIO_CINEN, 0x0, ~(0x0)},
	{GPIO_CDAT, 0x0, ~(0x0)},
	{GPIO_DOUTEN, 0x0, ~(0x0)},
	{GPIO_DINEN, 0x0, ~(0x0)},
	{GPIO_DDAT, 0x0, ~(0x0)},
	{MFP_CTL0, 0x0, ~(0x0)},
	{MFP_CTL1, 0x0, ~(0x0)},
    {MFP_CTL2, 0x0, ~(0x0)},
    {MFP_CTL3, 0x0, ~(0x0)},
    {PAD_PULLCTL0, 0x0, ~(0x0)},
    {PAD_PULLCTL1, 0x0, ~(0x0)},
    {PAD_PULLCTL2, 0x0, ~(0x0)},

    /*INTC*/
    {INTC_EXTCTL, 0x0, ~(0x0)},
    {INTC_GPIOCTL, 0x0, ~(0x0)},
    {INTC_GPIOA_MSK, 0x0, ~(0x0)},
    {INTC_GPIOB_MSK, 0x0, ~(0x0)},
    {INTC_GPIOC_MSK, 0x0, ~(0x0)},
    {INTC_GPIOD_MSK, 0x0, ~(0x0)},

	/* <5> UART control register */
	{UART0_CTL, 0x0, ~(0x0)},
	{UART1_CTL, 0x0, ~(0x0)},
    {UART2_CTL, 0x0, ~(0x0)},
    {UART3_CTL, 0x0, ~(0x0)},
    {UART4_CTL, 0x0, ~(0x0)},
    {UART5_CTL, 0x0, ~(0x0)},
    
    /*Shareram CTL*/
    {SHARESRAM_CTL, 0x0, ~(0x0)},
    
    /*pad drv*/
    {PAD_DRV0, 0x0, ~(0x0)},
    {PAD_DRV1, 0x0, ~(0x0)},
    {PAD_DRV2, 0x0, ~(0x0)},
};

void hibernate_save_regs(void)
{
	/* save core register */
	int i;

	for(i = 0; i < sizeof(leopard_regs) / sizeof(leopard_reg_t); i++) {
		if (leopard_regs[i].reg_addr != 0) {
			leopard_regs[i].reg_val = act_readl(leopard_regs[i].reg_addr);
		}
	}

//    save_asoc_power_regs();
}

static void restore_base_pll(leopard_reg_t *leopard_base_reg)
{
    BUG_ON(leopard_base_reg[0].reg_addr != CMU_COREPLL);
    BUG_ON(leopard_base_reg[1].reg_addr != CMU_BUSCLK);
    BUG_ON(leopard_base_reg[2].reg_addr != CMU_DEVPLL);
    
    /*restore devpll first*/
    act_writel(leopard_base_reg[2].reg_val, CMU_DEVPLL);
    udelay(100);
    
//    /* cpuclk switch to host */
//    act_writel((act_readl(CMU_BUSCLK) & (~0x3)) | 0x1, CMU_BUSCLK);
//    udelay(1);
//    /* restore corepll register */
//    act_writel(leopard_base_reg[0].reg_val, CMU_COREPLL);
//    udelay(100);

    /* restore busclk register */
    act_writel(leopard_base_reg[1].reg_val, CMU_BUSCLK);
    act_readl(CMU_BUSCLK);
    udelay(10);
    
    printk("%s %d, corepll:0x%x, busclk:0x%x, devpll:0x%x", __FUNCTION__,__LINE__,  act_readl(CMU_COREPLL), act_readl(CMU_BUSCLK),  act_readl(CMU_DEVPLL));    	
}

void hibernate_restore_regs(int in_suspend)
{
    int i;
    unsigned int val;
    
//    restore_asoc_power_regs();
    
	/* restore core register */
    BUG_ON(sizeof(leopard_regs) / sizeof(leopard_reg_t) < 3);

    restore_base_pll(leopard_regs);

    for(i = 3; i < sizeof(leopard_regs) / sizeof(leopard_reg_t); i++) {
		if (leopard_regs[i].reg_addr != 0) {
            val = leopard_regs[i].reg_val & leopard_regs[i].reg_mask;
            val |= act_readl(leopard_regs[i].reg_addr) & (~leopard_regs[i].reg_mask);
            act_writel(val, leopard_regs[i].reg_addr);
		}
	}
}


int hibernate_reg_setmap(unsigned int reg, unsigned int bitmap)
{
    int i;
    
    for(i = 0; i < sizeof(leopard_regs) / sizeof(leopard_reg_t); i++) {
		if (leopard_regs[i].reg_addr != 0 && leopard_regs[i].reg_addr == reg) {
		    leopard_regs[i].reg_mask = ~(~leopard_regs[i].reg_mask | bitmap);
            break;
		}
	}
}

int hibernate_reg_clearmap(unsigned int reg, unsigned int bitmap)
{
    int i;

    for(i = 0; i < sizeof(leopard_regs) / sizeof(leopard_reg_t); i++) {
		if (leopard_regs[i].reg_addr != 0 && leopard_regs[i].reg_addr == reg) {
            leopard_regs[i].reg_mask |= bitmap;
            break;
		}
	}
}

EXPORT_SYMBOL(hibernate_reg_setmap);
EXPORT_SYMBOL(hibernate_reg_clearmap);
