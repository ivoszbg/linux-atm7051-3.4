/*
 * pm.c  --  common power management (suspend to ram) support for GL5201.
 *
 * Copyright 2011 Actions Semi Inc.
 * Author: Actions Semi, Inc.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/serial_core.h>
#include <linux/power_supply.h>
#include <linux/io.h>
#include <linux/module.h> 
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/uaccess.h>
//#include <asm/r4kcache.h>
//#include <asm/mach-actions/actions_soc.h>
#include <mach/atc260x/atc260x.h>
#include <mach/hardware.h>
#include <mach/powergate.h>
#include <mach/dvfslevel.h>
#include <asm/suspend.h>
#include <asm/cacheflush.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <asm/system_misc.h>
#include <asm/smp_scu.h>
#include <mach/hardware.h>


#define pm_debug(format, arg...)		\
	printk(KERN_DEBUG format , ## arg)

extern void leopard_cpu_resume(void);

u32 suspend_sp;
static struct pmic_suspend_ops   *pmic_suspend_ops = NULL;

int (* boot_info_read)(unsigned short reg);
int (* boot_info_write)(unsigned short reg, unsigned short val);

/*
    返回值定义如下：
         0 ---- 无任何adapter插入
         1 --- wall adapter 插入
         2 --- pc usb 插入
         3 --- usb adapter 插入
         4 --- wall & usb adapter同时插入
 */
#define ADAPTER_TYPE_NO_PLUGIN             0
#define ADAPTER_TYPE_WALL_PLUGIN           1 
#define ADAPTER_TYPE_PC_USB_PLUGIN         2
#define ADAPTER_TYPE_USB_ADAPTER_PLUGIN    3
#define ADAPTER_TYPE_USB_WALL_PLUGIN       4

/*新增判断usb type的接口*/
int (*judge_adapter_type)(void) = NULL;

int set_judge_adapter_type_handle(void* handle)
{
    judge_adapter_type = handle;
    return 0;
}
EXPORT_SYMBOL_GPL(set_judge_adapter_type_handle);

struct asoc_power_regulator_par{
	const char *regulator_name;
	int uv;
//	unsigned int mode;
};

struct asoc_power_regulator_par asoc_system_powers[] = {
    {"vddcore", 0},
    {"vddr",    0},
    {"vcc", 0},
    {"avddcore", 0},
};

int num_asoc_system_powers = ARRAY_SIZE(asoc_system_powers);
/*
 * Here we need save/restore the core registers that are
 * either volatile or reset to some state across a processor sleep.
 * If reading a register doesn't provide a proper result for a
 * later restore, we have to provide a function for loading that
 * register and save a copy.
 *
 * We only have to save/restore registers that aren't otherwise
 * done as part of a driver pm_* function.
 */

struct CORE_REG {
	u32 reg_addr;
	u32 reg_val;
};

static struct CORE_REG core_regs[] = {
	/* <1> CMU register */
	{CMU_COREPLL, 0x0},
	{CMU_BUSCLK, 0x0},
    {CMU_DEVPLL, 0x0},
//    {CMU_NANDPLL, 0x0},
    {CMU_DISPLAYPLL, 0x0},
    {CMU_AUDIOPLL, 0x0},
    {CMU_TVOUTPLL, 0x0},
    {CMU_SENSORCLK, 0x0},
    {CMU_LCDCLK, 0x0},
    {CMU_DECLK, 0x0},
    {CMU_SICLK, 0x0},
    {CMU_VDECLK, 0x0},
    {CMU_VCECLK, 0x0},
    {CMU_GPUCLK, 0x0},
//    {CMU_NANDCCLK, 0x0},
    {CMU_SD0CLK, 0x0},
    {CMU_SD1CLK, 0x0},
    {CMU_SD2CLK, 0x0},
    {CMU_UART0CLK, 0x0},
    {CMU_UART1CLK, 0x0},
    {CMU_UART2CLK, 0x0},
    {CMU_DMACLK, 0x0},
    {CMU_PWM0CLK, 0x0},   
    {CMU_PWM1CLK, 0x0},
    {CMU_PWM2CLK, 0x0},
    {CMU_PWM3CLK, 0x0},
    {CMU_USBCLK, 0x0},
    {CMU_120MPLL, 0x0},
    {CMU_DEVCLKEN0, 0x0},
    {CMU_DEVCLKEN1, 0x0},
    {CMU_DEVRST0, 0x0},
    {CMU_DEVRST1, 0x0},
    {CMU_UART3CLK, 0x0},
    {CMU_UART4CLK, 0x0},
    {CMU_UART5CLK, 0x0},
 
    /*Timer*/
	{T0_VAL, 0x0},    
    {T0_CTL, 0x0},
	{T0_CMP, 0x0},
	{TWOHZ0_CTL, 0x0},

    
	/* <3> DMA configuration register */
	{DMA_CTL, 0x0},
	{DMA_IRQEN, 0x0},
//	{DMA_IRQPD, 0x0},
	{DMA_PAUSE, 0x0},
//	{CMU_DMACLK, 0x0},

	/* <4> Multiplex pin control register */
	{GPIO_ADAT, 0x0},
	{GPIO_AOUTEN, 0x0},
	{GPIO_AINEN, 0x0},
	{GPIO_BDAT, 0x0},
	{GPIO_BOUTEN, 0x0},
	{GPIO_BINEN, 0x0},
	{GPIO_CDAT, 0x0},
	{GPIO_COUTEN, 0x0},
	{GPIO_CINEN, 0x0},
	{GPIO_DDAT, 0x0},
	{GPIO_DOUTEN, 0x0},
	{GPIO_DINEN, 0x0},

	/*PWM duty */
	{PWM_CTL0, 0x0},
	{PWM_CTL1, 0x0},
	{PWM_CTL2, 0x0},
	{PWM_CTL3, 0x0},
	
	{MFP_CTL0, 0x0},
	{MFP_CTL1, 0x0},
    {MFP_CTL2, 0x0},
    {MFP_CTL3, 0x0},
    {PAD_PULLCTL0, 0x0},
    {PAD_PULLCTL1, 0x0},
    {PAD_PULLCTL2, 0x0},


    /*INTC*/
    {INTC_EXTCTL, 0x0},
    {INTC_GPIOCTL, 0x0},
    {INTC_GPIOA_MSK, 0x0},
    {INTC_GPIOB_MSK, 0x0},
    {INTC_GPIOC_MSK, 0x0},
    {INTC_GPIOD_MSK, 0x0},

	/* <5> UART control register */
	{UART0_CTL, 0x0},
	{UART1_CTL, 0x0},
    {UART2_CTL, 0x0},
    {UART3_CTL, 0x0},
    {UART4_CTL, 0x0},
    {UART5_CTL, 0x0},
    
    /*Shareram CTL*/
    {SHARESRAM_CTL, 0x0},
    
    /* Pad_drv*/
    {PAD_DRV0, 0x0},
    {PAD_DRV1, 0x0},
    {PAD_DRV2, 0x0},

	/* Terminate tag */
	{0x0, 0x0}
};

void save_core_regs(void)
{
	/* save core register */
	s32 index = 0, num;
	num = sizeof(core_regs) / sizeof(core_regs[0]);

    pm_debug("enter save_core_regs\n");
	while (index < num) {
		if (core_regs[index].reg_addr != 0) {
			core_regs[index].reg_val =
			    act_readl(core_regs[index].reg_addr);
            pm_debug("register<%2d>[%#x] = 0%#x\n", index,
                core_regs[index].reg_addr,
                core_regs[index].reg_val);
		}
		index++;
	}
}

static void restore_corepll_busclk(unsigned int corepll_val, unsigned int busclk_val, unsigned int devpll_value)
{
    unsigned int val, tmp;

    pm_debug("\n %s corepll:0x%x, busclk:0x%x, devpll:0x%x", __FUNCTION__, act_readl(CMU_COREPLL), act_readl(CMU_BUSCLK),  act_readl(CMU_DEVPLL));    	
    pm_debug("\n [zjl] enter %s corepll_val:0x%x, busclk_val:0x%x, devpll_value:0x%x", __FUNCTION__, corepll_val, busclk_val, devpll_value);

    
    /*restore devpll first*/
    /*先enable devpll，然后delay 50us*/
    tmp = act_readl(CMU_DEVPLL);
    tmp &= ~(0x7f);
    tmp |= (devpll_value&0x7f);
    tmp |= (0x1<<8);
    act_writel(tmp, CMU_DEVPLL);
    udelay(100);
    
    
    /*恢复busclk中，除了bit[1:0]的部分*/
    tmp = act_readl(CMU_BUSCLK);
    tmp &= (0x3);
    tmp |= (busclk_val & (~0x3));
    act_writel(tmp, CMU_BUSCLK);
    
    act_writel(act_readl(CMU_DEVPLL)| (0x1<<12), CMU_DEVPLL);
    udelay(50);
    /*restore vce_clk_before_gating*/
    act_writel(0x0, CMU_VCECLK);
        
    /* cpuclk switch to vce_clk_before_gating */
    val = act_readl(CMU_BUSCLK);
    val &= ~(0x3<<0);
    val |= (0x3<<0);
    act_writel(val, CMU_BUSCLK);

    udelay(1);

    /* restore corepll register */
    act_writel(corepll_val, CMU_COREPLL);

    udelay(100);

    /* restore busclk register bit[1:0] */
    tmp = act_readl(CMU_BUSCLK);
    tmp &= (~0x3);
    tmp |= (busclk_val & 0x3);
    act_writel(tmp, CMU_BUSCLK);
    act_readl(CMU_BUSCLK);
    udelay(10);
    
    pm_debug("\n %s %d, corepll:0x%x, busclk:0x%x, devpll:0x%x", __FUNCTION__,__LINE__,  act_readl(CMU_COREPLL), act_readl(CMU_BUSCLK),  act_readl(CMU_DEVPLL));    	
}

void restore_core_regs(void)
{
	/* restore core register */
	s32 index = 0, num;
	num = sizeof(core_regs) / sizeof(core_regs[0]);

    pm_debug("enter restore_core_regs\n");

    BUG_ON(num < 3);
    BUG_ON(core_regs[0].reg_addr != CMU_COREPLL);
    BUG_ON(core_regs[1].reg_addr != CMU_BUSCLK);

    restore_corepll_busclk(core_regs[0].reg_val, core_regs[1].reg_val, core_regs[2].reg_val);

    index = 3;
	while (index < num) {
        pm_debug("\n reg[%d], reg_addr:0x%x, reg_val:0x%x", index, core_regs[index].reg_addr, core_regs[index].reg_val);
		if (core_regs[index].reg_addr != 0) {
			act_writel(core_regs[index].reg_val,
				   core_regs[index].reg_addr);

        pm_debug("register<%2d>[%#x] = %#x\n", index,
                core_regs[index].reg_addr,
                act_readl(core_regs[index].reg_addr));
		}
		index++;
	}
}

void gl5201_blast_cache(void)
{
#if 0
	pm_debug("enter gl5201_blast_cache");

	blast_dcache32();
	blast_icache32();
    blast_scache32();
#endif
}

unsigned short _GetCheckSum(void* buf,unsigned int len)
{
    unsigned int loop;
    unsigned short sum = 0;
    unsigned short* p_buf = (unsigned short*)buf;
    for(loop = 0; loop <len; loop++)
    {
        sum += p_buf[loop];
    }

    return sum;
}

/* suspend_func_size 大小暂定为0x1000*/
static unsigned int suspend_func_size = 0x1000; 
extern void leopard_finish_suspend(unsigned long pmu_type , unsigned long i2c_base);
extern int get_pmu_type(void);
extern int get_pmu_i2c_num(void);
typedef void (*finish_suspend_t)(unsigned long pnu_type, unsigned long i2c_base);

/*
	called by cpu_suspend。
	first should copy the real code to address 0xffff8000.
	the mem is configured to PT_MEM, and can run without DDR
  */
int asoc_finish_suspend(unsigned long pmu_type)
{
    
#if 0    
//    volatile unsigned int ii=1;
	finish_suspend_t func_run = (0xffff8000);
    pm_debug("\n[zjl]enter %s %d, leopard_finish_suspend:0x%x", __FUNCTION__, __LINE__, leopard_finish_suspend);
  	flush_cache_all();	
	/*将leopard_real_finish_suspend 代码copy到
	    0xffff8000 处执行
	  */
	  
  	memcpy((void*)func_run, (void*)leopard_finish_suspend, suspend_func_size);
    flush_cache_all();
	/*跳转到leopard_real_finish_suspend执行*/
  	func_run(pmu_type);
#else
	unsigned long i2c_base;
    /*测试使用shareram来实现*/
    finish_suspend_t func_run = (finish_suspend_t)0xffff8000;
    //switch_shareram_AHB();
    {
        /*
         将shareram切换到AHB通路访问
         注意shareram 可用条件：
         1. shareram power enable
         2. shareram clk enable
         3. shareram ctl switch ok
         */
        unsigned int dvfslevel = asoc_get_dvfslevel();
        
         
        /*
            1. check shareram power is enabled
            gl5207 shareram power is always enabled.
            gl5202c shareram power is controled by VDE power
         */
        pm_debug("\n dvfslevel:0x%x ", ASOC_GET_IC(dvfslevel));
        if(ASOC_GET_IC(dvfslevel) == 0x7029)
        {
            asoc_powergate_power_on(ASOC_POWERGATE_VDE);
        }
           
        /*2. check shareram clk is enabled */ 
        if((act_readl(CMU_DEVCLKEN0) & (0x1<<28)) == 0)
        {
            act_writel((act_readl(CMU_DEVCLKEN0) | (0x1<<28)), CMU_DEVCLKEN0);
        }
        
        /*3. switch module*/
        act_writel(0x0, SHARESRAM_CTL);
        
        pm_debug("\n %s SPS_PG_CTL:0x%x, CMU_DEVCLKEN0:0x%x, SHARESRAM_CTL:0x%x", __FUNCTION__, act_readl(SPS_PG_CTL), act_readl(CMU_DEVCLKEN0), act_readl(SHARESRAM_CTL));
    }
    
    flush_cache_all();	
    memcpy((void*)func_run, (void*)leopard_finish_suspend, suspend_func_size);
    flush_cache_all();
	/*跳转到leopard_real_finish_suspend执行*/


	pmu_type = get_pmu_type();
	if (pmu_type == 1)
		i2c_base = 0;
	else
		i2c_base = IO_ADDRESS(I2C0_BASE+get_pmu_i2c_num()*0x4000);

	printk("\n pmu_type=%d ,pmu_i2c_base=0x%x  %s %d\n",pmu_type,i2c_base,__FUNCTION__,__LINE__);
	func_run(pmu_type, i2c_base);

#endif  	
	return 0;
	
}

void asoc_sleep(void)
{
	int wakeup_src;
	int wakeup_mask;
//	int ddr_prio;
	int adapter_type = -1;

    
	if (pmic_suspend_ops && pmic_suspend_ops->set_wakeup_src) {
	    
	    if(judge_adapter_type != NULL)
	    {
	        adapter_type = (*judge_adapter_type)();
	        pr_info("\n %s adapter_type:%d", __FUNCTION__, adapter_type);
	        if(adapter_type<0)
	        {
	            pr_info("\n[%s] Error at judge_adapter_type func!", __FUNCTION__);
	            wakeup_src = (WAKEUP_SRC_ONOFF_SHORT | WAKEUP_SRC_ALARM);
	            wakeup_mask = wakeup_src;
	        }
	        else 
	        {
	            switch(adapter_type)
	            {
	            case ADAPTER_TYPE_NO_PLUGIN:
	                /*无任何adapter 插入*/
	                wakeup_src = (WAKEUP_SRC_ONOFF_SHORT | WAKEUP_SRC_ALARM);
	                wakeup_mask = wakeup_src;
	                break; 
	                
	            case ADAPTER_TYPE_WALL_PLUGIN:
	                /*wall 插入，且充满电*/
	                wakeup_mask = (WAKEUP_SRC_ONOFF_SHORT | WAKEUP_SRC_VBUS_IN |
									WAKEUP_SRC_WALL_IN | WAKEUP_SRC_ALARM |
									WAKEUP_SRC_WALL_OUT | WAKEUP_SRC_VBUS_OUT);
	                wakeup_src = (WAKEUP_SRC_ONOFF_SHORT | WAKEUP_SRC_VBUS_IN | WAKEUP_SRC_WALL_OUT);
	                break;
	            
	            case ADAPTER_TYPE_PC_USB_PLUGIN:
	                /* usb 连接 PC*/
	                wakeup_src = (WAKEUP_SRC_ONOFF_SHORT | WAKEUP_SRC_ALARM);
	                wakeup_mask = wakeup_src;
	                break;
	            case ADAPTER_TYPE_USB_ADAPTER_PLUGIN:
	                /*usb adapter 插入*/
	                wakeup_mask = (WAKEUP_SRC_ONOFF_SHORT | WAKEUP_SRC_VBUS_IN |
					WAKEUP_SRC_WALL_IN | WAKEUP_SRC_ALARM | WAKEUP_SRC_VBUS_OUT | WAKEUP_SRC_VBUS_OUT);
	                wakeup_src = (WAKEUP_SRC_ONOFF_SHORT | WAKEUP_SRC_WALL_IN | WAKEUP_SRC_VBUS_OUT);
	                break;
	            case ADAPTER_TYPE_USB_WALL_PLUGIN:
	                /*usb adapter & Wall adatper 同时插入*/
	                wakeup_mask = (WAKEUP_SRC_ONOFF_SHORT | WAKEUP_SRC_VBUS_IN | WAKEUP_SRC_WALL_IN |
								 WAKEUP_SRC_ALARM | WAKEUP_SRC_VBUS_OUT | WAKEUP_SRC_WALL_OUT);

	                wakeup_src = (WAKEUP_SRC_ONOFF_SHORT | WAKEUP_SRC_VBUS_OUT | WAKEUP_SRC_WALL_OUT);
	                break;
	            default :
	                /*something wrong*/
	                pr_info("\n [%s] adatper type is wrong.", __FUNCTION__);
	                wakeup_src = (WAKEUP_SRC_ONOFF_SHORT | WAKEUP_SRC_ALARM);
	                wakeup_mask = wakeup_src;
	            };
	            
	        }
	        
	    }
	    else
	    {
	        wakeup_src = (WAKEUP_SRC_ONOFF_SHORT | WAKEUP_SRC_ALARM);
	        wakeup_mask = wakeup_src;
	    }
	    
	    wakeup_src |= WAKEUP_SRC_ONOFF_LONG | WAKEUP_SRC_ALARM ;
		wakeup_mask |= WAKEUP_SRC_ONOFF_LONG | WAKEUP_SRC_ALARM;
		
		pr_info("\n %s wakesrc: 0x%x, wakeup_mask:0x%x\n", __FUNCTION__, wakeup_src, wakeup_mask);
		/*xyl :alarm wakesource has some problem, so forbidden it until it's OK*/		
		pmic_suspend_ops->set_wakeup_src(wakeup_mask, wakeup_src);
	}
	    
	//ddr_prio = asoc_get_ddr_prio();
	save_core_regs();
	/* save and restore the cpu core register */
	
	cpu_suspend(0, asoc_finish_suspend);	

	restore_core_regs();

#ifdef CONFIG_SMP	
    scu_enable((void *)IO_ADDRESS(ASOC_PA_SCU));
#endif

	//asoc_set_ddr_prio(ddr_prio);
	if (pmic_suspend_ops && pmic_suspend_ops->set_wakeup_src) {
		wakeup_src = 0;
		pm_debug("\nwakesrc: 0x%x\n", wakeup_src);
		/*xyl :alarm wakesource has some problem, so forbidden it until it's OK*/
		pmic_suspend_ops->set_wakeup_src(WAKEUP_SRC_ONOFF_SHORT, wakeup_src);
	}
}

/**
 *	pmic_suspend_set_ops - Set the global suspend method table.
 *	@ops:	Pointer to ops structure.
 */
void pmic_suspend_set_ops(struct pmic_suspend_ops *ops)
{
    pm_debug("[PM] set pmic suspend ops\n");

	pmic_suspend_ops = ops;
}
EXPORT_SYMBOL_GPL(pmic_suspend_set_ops);

static int asoc_pm_enter(suspend_state_t state)
{
    pm_debug("[PM] %s %d:\n", __FUNCTION__, __LINE__);

    if (!pmic_suspend_ops || !pmic_suspend_ops->enter)
        return -ENODEV;

    pmic_suspend_ops->enter(POWER_MODE_STANDBY);

    pm_debug("[PM] %s %d:\n", __FUNCTION__, __LINE__);

    asoc_sleep();

	return 0;
}

static int asoc_pm_prepare(void)
{
    pm_debug("[PM] %s %d:\n", __FUNCTION__, __LINE__);

    if (!pmic_suspend_ops)
        return -ENODEV;

    if (pmic_suspend_ops->prepare) 
        pmic_suspend_ops->prepare(POWER_MODE_STANDBY);

	return 0;
}

static void asoc_pm_wake(void)
{
    if (pmic_suspend_ops && pmic_suspend_ops->wake)
        pmic_suspend_ops->wake();
}


static void asoc_pm_finish(void)
{
    pm_debug("[PM] %s %d:\n", __FUNCTION__, __LINE__);

    if (pmic_suspend_ops && pmic_suspend_ops->finish)
    {
        pmic_suspend_ops->finish();
    }

}

static int asoc_pm_valid_state(suspend_state_t state)
{
	switch (state) {
//	case PM_SUSPEND_ON:
//	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		return 1;
	default:
		return 0;
	}
}

static struct platform_suspend_ops asoc_pm_ops = {
	.enter		= asoc_pm_enter,
	.prepare	= asoc_pm_prepare,
    .wake       = asoc_pm_wake,
	.finish		= asoc_pm_finish,
	.valid		= asoc_pm_valid_state,
};

int asoc_pm_wakeup_flag(void)
{
    int wakeup_flag;

    pm_debug("[PM] %s %d:\n", __FUNCTION__, __LINE__);

	/* Power off system */

    if (!pmic_suspend_ops || !pmic_suspend_ops->get_wakeup_flag)
        return -ENODEV;

    wakeup_flag = pmic_suspend_ops->get_wakeup_flag();
    if (wakeup_flag < 0)
        return -1;

	return wakeup_flag;
}
EXPORT_SYMBOL_GPL(asoc_pm_wakeup_flag);

int halt_with_power_wakeup=1;

/*
    set_halt_flag
    flag:0-halt without power wakeup
    flag:1-halt with power wakeup(default)
*/
int set_halt_flag(int flag)
{
    halt_with_power_wakeup = flag;
    return 0;
}
EXPORT_SYMBOL_GPL(set_halt_flag);


#if 0
/*获取电池充电容量，返回0表示没有电池或者充电满了*/
static int battery_get_cap(void)
{
	union power_supply_propval ret = {0,};
	 struct power_supply *psy;
 	 psy = power_supply_get_by_name("atc260x-battery");
	 if (psy == NULL ) {
	 	pr_info("get atc260x-battery fail\n");
	 	return 0;
	 }
	 if (psy->get_property(psy, POWER_SUPPLY_PROP_PRESENT, &ret)) {
		pr_info("get battery present fail\n");
		return 0;
	 }
	 pr_info("get battery present = %d\n", ret.intval);
	if (ret.intval == 0){ // 没电池
		pr_info("battery  not present\n");
		return 0;
	}

	if (psy->get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &ret)) {
		pr_info("get battery capcity fail\n");
		return 0;
	}
	pr_info("get battery capcity = %d\n", ret.intval);
	
	if (ret.intval == 100)// 充满电
		return 0;

	return -1;
}
#endif

unsigned int support_adaptor_type;
extern int hard_key_get_state(void);
void asoc_pm_halt(void)
{
    int mode, wakeup_src;
	int adapter_type = -1;
	
    pm_debug("[PM] %s %d:\n", __FUNCTION__, __LINE__);

    if (!pmic_suspend_ops || !pmic_suspend_ops->enter)
        return ;

    /* default sleep mode and wakeup source */
/*xyl :alarm wakesource has some problem, so forbidden it until it's OK*/
    mode = POWER_MODE_SLEEP;
    
    if(halt_with_power_wakeup)
    {
        wakeup_src = WAKEUP_SRC_RESET |
    //        WAKEUP_SRC_ALARM |
    //        WAKEUP_SRC_ONOFF_SHORT |
            WAKEUP_SRC_ONOFF_LONG |
         WAKEUP_SRC_WALL_IN | 
         WAKEUP_SRC_VBUS_IN;


		if (!power_supply_is_system_supplied()) {            
				printk("wall/usb not connect, ennter s4\n");
				mode = POWER_MODE_DEEP_SLEEP;
	    } else {
			int stat =hard_key_get_state( );
			printk("--wall/usb  connect, ennter s3, %d\n", stat);
				/* if wall/usb is connect, cannot enter S4, only can enter S3 */
			if ( stat > 0)  /*定时关机进充电模式*/
				boot_info_write(atc2603_PMU_FW_USE1, ((boot_info_read(atc2603_PMU_FW_USE1) & (~(0x1 << 4))) | (0x1 << 4)));
			else
				boot_info_write(atc2603_PMU_FW_USE1,  boot_info_read(atc2603_PMU_FW_USE1) & (~(0x1 << 4)) );

			#if 0
			if( battery_get_cap() ) {/*没有满电，重启进入充电模式*/
				printk("cap not full, enter chargr:\n");
				wakeup_src |= WAKEUP_SRC_WALL_IN | WAKEUP_SRC_VBUS_IN;
			} else {
				//满电了
				boot_info_write(atc2603_PMU_FW_USE1,  boot_info_read(atc2603_PMU_FW_USE1) & (~(0x1 << 4)) );
			}
			#endif
		}


		if(support_adaptor_type == 1)
		{
			/* if wall is connect, enter S3. otherwise, enter S4 */
			if(judge_adapter_type != NULL)
			{
				adapter_type = (*judge_adapter_type)();
				pr_info("\n %s adapter_type:%d", __FUNCTION__, adapter_type);
				if(adapter_type>=0)
				{
					switch(adapter_type)
					{
					case ADAPTER_TYPE_PC_USB_PLUGIN:/* usb 连接 PC*/
					case ADAPTER_TYPE_USB_ADAPTER_PLUGIN:/*usb adapter 插入*/
						wakeup_src &= ~WAKEUP_SRC_VBUS_IN;
						boot_info_write(atc2603_PMU_SYS_CTL5, boot_info_read(atc2603_PMU_SYS_CTL5) & ~(0x1 << 8));
						break;
					default :
						break;
					};
				}
			}	
		}
    }
    else
    {
        wakeup_src = WAKEUP_SRC_RESET |
           WAKEUP_SRC_ONOFF_LONG;
    }

	/* Power off system */
	pr_info("\nPowering off (wakesrc: 0x%x, mode:%d)\n",
        wakeup_src, mode);

    pmic_suspend_ops->set_wakeup_src(WAKEUP_SRC_ALL, 
        wakeup_src);
    pmic_suspend_ops->enter(mode);

    /* never return to here */
    pm_debug("[PM] %s %d:\n", __FUNCTION__, __LINE__);
}
EXPORT_SYMBOL_GPL(asoc_pm_halt);

void asoc_pm_halt_upgrade(void)
{
    int mode, wakeup_src;

    pm_debug("[PM] %s %d:\n", __FUNCTION__, __LINE__);

    if (!pmic_suspend_ops || !pmic_suspend_ops->enter)
        return ;

    /* default sleep mode and wakeup source */
/*xyl :alarm wakesource has some problem, so forbidden it until it's OK*/
    mode = POWER_MODE_SLEEP;
    wakeup_src = WAKEUP_SRC_RESET |
        WAKEUP_SRC_ONOFF_LONG;

	/* Power off system */
	pr_info("\nPowering off (wakesrc: 0x%x, mode:%d)\n",
        wakeup_src, mode);

    pmic_suspend_ops->set_wakeup_src(WAKEUP_SRC_ALL, 
        wakeup_src);
    pmic_suspend_ops->enter(mode);

    /* never return to here */
    pm_debug("[PM] %s %d:\n", __FUNCTION__, __LINE__);
}
EXPORT_SYMBOL_GPL(asoc_pm_halt_upgrade);

int asoc_pm_reset(void)
{
    #if 0
    if (!pmic_suspend_ops || !pmic_suspend_ops->reset)
        return -1;

    pmic_suspend_ops->reset();
   #endif
    return 0;
}
EXPORT_SYMBOL_GPL(asoc_pm_reset);


u32 get_phBaseAddr(void)
{
	u32 tmp;
	
	__asm__ __volatile__ ("mrc p15, 4, %0, c15, c0, 0":"=r"(tmp));
	return tmp;
}

// void init_WD_timer(unsigned int load_value, unsigned int auto_reload)
// Sets up the WD timer
// r0: initial load value
// r1:  IF 0 (AutoReload) ELSE (SingleShot)
void init_WD_timer(unsigned int load_value, unsigned int auto_reload)
{
	u32 phBaseAddr, wdCountAddr, wdModeAddr, tmp=0;
   	
   	phBaseAddr = get_phBaseAddr();
   	wdCountAddr = phBaseAddr+0x620;
   	wdModeAddr = phBaseAddr+0x628;
   	
   	act_writel(load_value, wdCountAddr);
   	if(auto_reload == 0)
   	{
   		tmp = 0x2;	
   	}
   	act_writel(tmp, wdModeAddr);
}

// void set_WD_mode(unsigned int mode)
// Sets up the WD timer  
// r0:  IF 0 (timer mode) ELSE (watchdog mode)
void set_WD_mode(unsigned int mode)
{
	u32 phBaseAddr, wdModeAddr;
	
	phBaseAddr = get_phBaseAddr();
   	wdModeAddr = phBaseAddr+0x628;
   	
   	if(mode == 0)
   	{
   		act_writel((act_readl(wdModeAddr) & 0xf7) | 0x4, wdModeAddr);
   	}
   	else
   	{
   		act_writel(act_readl(wdModeAddr) | 0x8, wdModeAddr);
   	}
}

// void start_WD_timer(void)
// Starts the WD timer
void start_WD_timer(void)
{
	u32 phBaseAddr, wdModeAddr;
	
	phBaseAddr = get_phBaseAddr();
   	wdModeAddr = phBaseAddr+0x628;
   	
	act_writel(act_readl(wdModeAddr) | 0x1, wdModeAddr);
}

int (*set_alarm_restart_time)(int);

int set_restart_handler(void* handler)
{
    set_alarm_restart_time = handler;
    return 0;
}

EXPORT_SYMBOL_GPL(set_restart_handler);
void cpu_reset_to_broom(void)
{
	local_fiq_disable();
	arch_local_irq_disable(); 	
	init_WD_timer(0xA, 0x01);
  set_WD_mode(1);
  start_WD_timer();
  while(1)
  {
  	asm volatile("wfe");
  }
}
int wd_restart(void)
{
#if 0    
    act_writel(act_readl(SPS_PG_CTL)|0x04000000, SPS_PG_CTL);
    init_WD_timer(0x98, 0x01);
    set_WD_mode(1);
    start_WD_timer();
    while(1)
    {
    };
#endif
    int wakeup_src = WAKEUP_SRC_RESET | WAKEUP_SRC_ALARM;
    
    if (!pmic_suspend_ops || !pmic_suspend_ops->set_wakeup_src || !pmic_suspend_ops->enter)
    {
        return -1;
    }
    pmic_suspend_ops->prepare(POWER_MODE_SLEEP);
    if(!power_supply_is_system_supplied())
    {
        wakeup_src |= (WAKEUP_SRC_WALL_IN | WAKEUP_SRC_VBUS_IN);
    }
    pmic_suspend_ops->set_wakeup_src(WAKEUP_SRC_ALL, wakeup_src);
    if(set_alarm_restart_time)
    {
        set_alarm_restart_time(2);
    }
    pr_info("\n######### %s start power down", __FUNCTION__);
    pmic_suspend_ops->enter(POWER_MODE_SLEEP);  
    
    return 0;
}
/*******************************************
*factory reset reg: atc2603_PMU_SYS_CTL2[2:1]
*set flag when restart with "recovery" cmd
*******************************************/

void asoc_pm_restart(char mode, const char *cmd)
{
    if(cmd != NULL)
    {
     pr_info("cmd:%s----restart------\n", cmd);
	 int pmu_type = get_pmu_type();
	 if (!strcmp(cmd, "recovery") ) 
	 {
		if (pmu_type==ATC2603A)
			boot_info_write(atc2603_PMU_SYS_CTL2, ((boot_info_read(atc2603_PMU_SYS_CTL2) & (~(0x3 << 1))) | (0x1 << 1)));
		if (pmu_type==ATC2603C)
			boot_info_write(atc2603_PMU_FW_USE1, ((boot_info_read(atc2603_PMU_FW_USE1) & (~(0x3 << 1))) | (0x1 << 1)));
	 } 
	 else if (!strcmp(cmd, "adfu"))
	 {
	 	pr_info("---adfu reboot--------\n");
		boot_info_write(atc2603_PMU_UV_Status, ((boot_info_read(atc2603_PMU_UV_Status) & (~(0x1<< 1))) | (0x1 << 1)));
	 } 
	 else 
	 {
        pr_info("-cmd=%s not support--normal reboot--\n", cmd);
        boot_info_write(atc2603_PMU_OC_Status, ((boot_info_read(atc2603_PMU_OC_Status) & (~(0x1 << 1))) | (0x1 << 1)));
	 }
    }
    else
    {
        pr_info("---normal reboot--------\n");
        boot_info_write(atc2603_PMU_OC_Status, ((boot_info_read(atc2603_PMU_OC_Status) & (~(0x1 << 1))) | (0x1 << 1)));
    }

    wd_restart();
    return ;
}

static int asoc_wakeup_src_init(void)
{
	int wakeup_src;

	if (pmic_suspend_ops && pmic_suspend_ops->set_wakeup_src) {
		wakeup_src = WAKEUP_SRC_RESET |
				//        WAKEUP_SRC_ALARM |
				//        WAKEUP_SRC_ONOFF_SHORT |
					WAKEUP_SRC_ONOFF_LONG |
					WAKEUP_SRC_WALL_IN | 
					WAKEUP_SRC_VBUS_IN;
		pmic_suspend_ops->set_wakeup_src(WAKEUP_SRC_ALL, 
			wakeup_src);
		return 0;
	} else {
		pr_info("[PM] FALL TO INIT WAKEUP SOURCE\n");
		return -EINVAL;
	}
}

int hibernate_power_init(void);
int hibernate_power_remove(void);

int get_support_adaptor_type(void)
{
	int ret;
	
    ret = get_config("charge.support_adaptor_type", (char *)(&support_adaptor_type), sizeof(unsigned int));
    if (ret == 0) {
        printk("charge.support_adaptor_type: %d\n", support_adaptor_type);
    }
	return ret;
}

static int __init asoc_pm_init(void)
{
	pr_info("[PM] ASOC Power Management\n");

	suspend_set_ops(&asoc_pm_ops);

#ifdef CONFIG_HIBERNATE
    hibernate_power_init();
#endif
    pm_power_off = asoc_pm_halt;
    arm_pm_restart = asoc_pm_restart;
	asoc_wakeup_src_init();
	get_support_adaptor_type();
	
	return 0;
}

static void __exit asoc_pm_remove(void)
{
#ifdef CONFIG_HIBERNATE
	hibernate_power_remove();
#endif
}

EXPORT_SYMBOL(power_kobj);
EXPORT_SYMBOL_GPL(leopard_cpu_resume);
EXPORT_SYMBOL_GPL(boot_info_read);
EXPORT_SYMBOL_GPL(boot_info_write);
late_initcall(asoc_pm_init);
module_exit(asoc_pm_remove);



static int upgrade_flag_proc_show(struct seq_file *m, void *v)
{
	int flag;
	flag =  (boot_info_read(atc2603_PMU_FW_USE1) & (0x3 << 1)) >> 1;
	printk("flag=%d\n", flag);
	if ( flag == 1 )
		seq_printf(m, "1\n");
	else
		seq_printf(m, "0\n");
	return 0;
}

static ssize_t upgrade_flag_proc_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *ppos)
{
	char buf[5];
	if (copy_from_user(buf, buffer, 1) != 0) {
		printk("upgrade_flag_proc_write: copy_from_user fail\n");
		return ERR_PTR(-EFAULT);
	}
	printk("upgrade_flag:wr=%c\n", buf[0]);
	if ( buf[0] == '0' )
		boot_info_write(atc2603_PMU_FW_USE1, 
			((boot_info_read(atc2603_PMU_FW_USE1) & (~(0x3 << 1))) | (0x0 << 1)));
	else
		boot_info_write(atc2603_PMU_FW_USE1, 
			((boot_info_read(atc2603_PMU_FW_USE1) & (~(0x3 << 1))) | (0x1 << 1)));
	
	return count;
}

static int upgrade_flag_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, upgrade_flag_proc_show, NULL);
}

static const struct file_operations upgrade_flag_proc_fops = {
	.open		= upgrade_flag_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
    .write		= upgrade_flag_proc_write,
};

static int __init proc_upgrade_flag_init(void)
{
	proc_create("upgrade_flag", S_IRUGO | S_IWUSR, NULL, &upgrade_flag_proc_fops);
	return 0;
}
module_init(proc_upgrade_flag_init);



