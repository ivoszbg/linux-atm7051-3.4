/*
 * atc260x_pm.c  --  ATC260X power management (suspend to ram) support.
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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/slab.h>

#include <mach/atc260x/atc260x_dev.h>
#include <asm/suspend.h>
#include <mach/hardware.h>
#include <mach/bootafinfo.h>
#include <linux/delay.h>
//#include <atc260x_pdata.h>

/* for saving ddr delay chain to atc2603 */
//#define CONFIG_ATC260X_SAVE_DDR_DELAY_CHAIN

//#ifdef CONFIG_ATC260X_SAVE_DDR_DELAY_CHAIN
//#include <asm/mach-actions/actions_soc.h>
//#endif
extern int get_pmu_type();
extern void leopard_cpu_resume(void);
extern void pmic_suspend_set_ops(struct pmic_suspend_ops *ops);

#define WAKEUP_SRC_OFFSET_IR            (0)
#define WAKEUP_SRC_OFFSET_RESET         (1)
#define WAKEUP_SRC_OFFSET_HDSW          (2)
#define WAKEUP_SRC_OFFSET_ALARM         (3)
#define WAKEUP_SRC_OFFSET_REMCON        (4)
#define WAKEUP_SRC_OFFSET_TP            (5)
#define WAKEUP_SRC_OFFSET_WKIRQ         (6)
#define WAKEUP_SRC_OFFSET_ONOFF_SHORT   (7)
#define WAKEUP_SRC_OFFSET_ONOFF_LONG    (8)
#define WAKEUP_SRC_OFFSET_WALL_IN       (9)
#define WAKEUP_SRC_OFFSET_VBUS_IN       (10)

/* atc2603_PMU_SYS_CTL0 register bits */
#define PMU_SYS_CTL0_WAKE_SRCS_SHIFT    (5)
#define PMU_SYS_CTL3_WAKE_SRCS_SHIFT    (11)
/* atc2603_PMU_SYS_CTL1 register bits */
#define PMU_SYS_CTL1_EN_S1              (1 << 0)
#define PMU_SYS_CTL1_LB_S4_SHIFT        (3) 
#define PMU_SYS_CTL1_LB_S4_MASK					(0x3 << PMU_SYS_CTL1_LB_S4_SHIFT)
#define PMU_SYS_CTL1_LB_S4_3_3V         (0x3 << PMU_SYS_CTL1_LB_S4_SHIFT)//3.3b 低电进S4电压
#define PMU_SYS_CTL1_WAKE_FLAG_SHIFT    (5)

/* atc2603_PMU_SYS_CTL2 register bits */
#define PMU_SYS_CTL2_S2_TIMER_SHIFT     (3)
#define PMU_SYS_CTL2_S2_TIMER_MASK      (0x7 << PMU_SYS_CTL2_S2_TIMER_SHIFT)
#define PMU_SYS_CTL2_S2_TIMER_EN        (1 << 6)

/* atc2603_PMU_SYS_CTL3 register bits */
#define PMU_SYS_CTL3_S3_TIMER_SHIFT     (10)
#define PMU_SYS_CTL3_S3_TIMER_MASK      (0x7 << PMU_SYS_CTL3_S3_TIMER_SHIFT)
#define PMU_SYS_CTL3_S3_TIMER_EN        (1 << 13)
#define PMU_SYS_CTL3_EN_S3              (1 << 14)
#define PMU_SYS_CTL3_EN_S2              (1 << 15)

/* atc2603_PMU_SYS_CTL5 register bits */
#define PMU_SYS_CTL5_DETECT_MASK        (0xf << 7)

#define PMU_SYS_CTL5_WALLWKDTEN         (1 << 7)
#define PMU_SYS_CTL5_VBUSWKDTEN         (1 << 8)
#define PMU_SYS_CTL5_REMCON_DECT_EN     (1 << 9)
#define PMU_SYS_CTL5_TP_DECT_EN         (1 << 10)

#define to_atc260x_pm_attr(_attr) \
	container_of(_attr, struct atc260x_pm_attribute, attr)
static unsigned int is_real_suspend = 0;
static const unsigned long s2tos3_timeout_table[8] = {
    6, 16, 31, 61, 91, 121, 151, 181
};

static const unsigned long s3tos4_timeout_table[8] = {
    6, 16, 31, 61, 91, 121, 151, 181
};

struct atc260x_pm {
    struct atc260x_dev  *atc260x;
};

struct atc260x_pm_attribute{
	struct kobj_attribute attr;
	int index;
};

static struct atc260x_pm atc260x_pm;  

/* ---------- pm interface ---------- */

static int atc260x_set_wakeup_src(int wakeup_mask, int wakeup_src)
{
    int val = 0, mask = 0;

    printk(KERN_DEBUG "[ATC260X PM]: wakeup_mask %x, wakeup_src %x\n", wakeup_mask, wakeup_src);

    if ((wakeup_mask & ~WAKEUP_SRC_ALL) || (wakeup_src & ~WAKEUP_SRC_ALL)) {
        pr_err("[ATC260X PM] invalid wake source\n");
        return -EINVAL;
    }

    atc260x_set_bits(atc260x_pm.atc260x,
        atc2603_PMU_SYS_CTL0,
        wakeup_mask << PMU_SYS_CTL0_WAKE_SRCS_SHIFT,
        wakeup_src << PMU_SYS_CTL0_WAKE_SRCS_SHIFT);

	atc260x_set_bits(atc260x_pm.atc260x,
        atc2603_PMU_SYS_CTL3,
        wakeup_mask >> PMU_SYS_CTL3_WAKE_SRCS_SHIFT,
        wakeup_src >> PMU_SYS_CTL3_WAKE_SRCS_SHIFT);
    /* set detect enable bit */

    if (wakeup_mask & WAKEUP_SRC_TP) {
        mask |= PMU_SYS_CTL5_TP_DECT_EN;
        if (wakeup_src & WAKEUP_SRC_TP){
            val |= PMU_SYS_CTL5_TP_DECT_EN;

            /* BUG74934: must eable TEN if TP as wakeup source */
           /* atc260x_set_bits(atc260x_pm.atc260x, 
                atc2603_TP_CTL1, 1, 1);*///PANGGA DELET
        }
    }

    if (wakeup_mask & WAKEUP_SRC_REMCON) {
        mask |= PMU_SYS_CTL5_REMCON_DECT_EN;
        if (wakeup_src & WAKEUP_SRC_REMCON)
            val |= PMU_SYS_CTL5_REMCON_DECT_EN;
    }

    /* for re-plugin wakeup, not disable wakeup detect for wall/usb cable */
#if 0
    if (wakeup_mask & WAKEUP_SRC_WALL_IN) {
        mask |= PMU_SYS_CTL5_WALLWKDTEN;
        if (wakeup_src & WAKEUP_SRC_WALL_IN)
            val |= PMU_SYS_CTL5_WALLWKDTEN;
    }

    if (wakeup_mask & WAKEUP_SRC_VBUS_IN) {
        mask |= PMU_SYS_CTL5_VBUSWKDTEN;
        if (wakeup_src & WAKEUP_SRC_VBUS_IN)
            val |= PMU_SYS_CTL5_VBUSWKDTEN;
    }
#endif
    atc260x_set_bits(atc260x_pm.atc260x, 
        atc2603_PMU_SYS_CTL5,
        mask, val);

    pr_debug("[ATC260X PM] PMU_SYS_CTL0:%x, PMU_SYS_CTL5:%x\n", 
        atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_SYS_CTL0),
        atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_SYS_CTL5));

	return 0;
}

static int atc260x_get_wakeup_src(void)
{
    int wakeup_src;

    pr_debug("[ATC260X PM] %s %d:\n", __FUNCTION__, __LINE__);

    pr_debug("[ATC260X PM] PMU_SYS_CTL0:%x, PMU_SYS_CTL5:%x\n", 
        atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_SYS_CTL0),
        atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_SYS_CTL5));

    wakeup_src = atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_SYS_CTL0);
    if (wakeup_src < 0)        
    {
        return wakeup_src;
    }
    
	return ((wakeup_src >> PMU_SYS_CTL0_WAKE_SRCS_SHIFT) & WAKEUP_SRC_ALL);
}

static int atc260x_get_wakeup_flag(void)
{
    int wakeup_flag;
    if(is_real_suspend == 1)
    {
    wakeup_flag = atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_SYS_CTL1);
    if (wakeup_flag < 0)
        return wakeup_flag;

    pr_debug("[ATC260X PM] %s(): PMU_SYS_CTL1:0x%x\n",
        __FUNCTION__, wakeup_flag);
        is_real_suspend = 0;
	return ((wakeup_flag >> PMU_SYS_CTL1_WAKE_FLAG_SHIFT) & WAKEUP_SRC_ALL);
    }
    else
    {
        return 0;
    }
}

static void atc260x_prepare_suspend(int mode)
{
    pr_debug("[ATC260X PM] %s %d: mode %d\n", __FUNCTION__, __LINE__, mode);

    /* avoid using standard SPI interface that maybe halt in suspend process */
    atc260x_set_access_mode(atc260x_pm.atc260x, 
        ATC260X_ACCESS_MODE_DIRECT);
}


/* 
 * for POWER_MODE_STANDBY only prepare the env, not disable S1 immediately 
 * other mode: disable S1 immediately
 */
static void atc260x_enter_suspend(int mode)
{
    int val, dat, tmp;
    void (*p_cpu_resume)(void);
    unsigned int resume_func_phy = 0;
    unsigned int high_bytes = 0;
	unsigned int timeout_cnt = 10, i=0;
	int pmu_type;
	int ver, val_dc2;
    
	pr_info("[ATC260X PM] %s %d: mode %d\n", __FUNCTION__, __LINE__, mode);
    p_cpu_resume = symbol_get(leopard_cpu_resume);
    if(!p_cpu_resume)
    {
        printk("\n[zjl] error at %s %d", __FUNCTION__, __LINE__);
        return ;
    }
		     pr_debug("[ATC260X PM] PMU_SYS_CTL0:%x, PMU_SYS_CTL3:%x, PMU_SYS_CTL5:%x DC2_CTL0:%x DC2_CTL1:%x\n", 
        atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_SYS_CTL0),
        atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_SYS_CTL3), 
        atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_SYS_CTL5),
		atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_DC2_CTL0),
		atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_DC2_CTL1)
		);
   pmu_type =get_pmu_type();
  printk("\n[zjl] leopard_cpu_resume : 0x%x, phy address:0x%x\n", (unsigned int)p_cpu_resume, virt_to_phys(p_cpu_resume));
    
    switch (mode) {
    case POWER_MODE_STANDBY:
        val = PMU_SYS_CTL3_EN_S2 | PMU_SYS_CTL3_FW_FLAG_S2;

        /* if S2->S3 timer is enabled, enable S3 state */
        dat = atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_SYS_CTL2);
        if (dat & PMU_SYS_CTL2_S2_TIMER_EN)
            val |= PMU_SYS_CTL3_EN_S3;

    /* 
          将leopard_cpu_resume 的物理地址存入到PMU_SYS_CTL8, PMU_SYS_CTL9
          PMU_SYS_CTL8 [15:0]--- resume_func_phy[15:0]
          PMU_SYS_CTL9 bit[7:0]--- resume_func_phy[23:16] 
	      PMU_SYS_CTL3 bit [9:6]  ---- resume_func_phy[27:24] 
	      PMU_OC_Status bit [5:2] ---- resume_func_phy[31:28]      
	*/
	 resume_func_phy = virt_to_phys(p_cpu_resume);
	 
	 if (pmu_type ==ATC2603C)
	 {
		high_bytes =  (resume_func_phy>>16);
		atc260x_reg_write(atc260x_pm.atc260x, atc2603_PMU_SYS_CTL8, (unsigned short) (resume_func_phy & 0xffff));
		atc260x_reg_write(atc260x_pm.atc260x, atc2603_PMU_FW_USE0, (unsigned short) (high_bytes & 0xffff));
		ver = (atc260x_reg_read(atc260x_pm.atc260x,atc2603_CHIP_VER)&0x7);
		if (ver == 0x0) {
			/* dc2 auto mode */
			val_dc2 = atc260x_reg_read(atc260x_pm.atc260x,atc2603_PMU_DC2_CTL0);
			val_dc2 = (val_dc2 & ~(0x3<<5)) | (0x1<<5);
			atc260x_reg_write(atc260x_pm.atc260x, atc2603_PMU_DC2_CTL0, val_dc2);

			atc260x_reg_write(atc260x_pm.atc260x, atc2603_PMU_DC2_CTL1,0XECAE);
		}
		if (ver == 0x1) {
			/* dc2 pfm mode */
			val_dc2 = atc260x_reg_read(atc260x_pm.atc260x,atc2603_PMU_DC2_CTL0);
			val_dc2 = val_dc2 & ~(0x3<<5);
			atc260x_reg_write(atc260x_pm.atc260x, atc2603_PMU_DC2_CTL0, val_dc2);
		}
	 }
	 if (pmu_type ==ATC2603A)
	 {
	 high_bytes =  (resume_func_phy>>16);
	 atc260x_reg_write(atc260x_pm.atc260x, atc2603_PMU_SYS_CTL8, (unsigned short) (resume_func_phy & 0xffff));
     atc260x_set_bits(atc260x_pm.atc260x,atc2603_PMU_SYS_CTL9,(0xff), high_bytes&0xff);
     
     high_bytes >>= 8;
     atc260x_set_bits(atc260x_pm.atc260x,atc2603_PMU_SYS_CTL3, (0xf<<6), (high_bytes&0xf)<<6);
     atc260x_set_bits(atc260x_pm.atc260x,atc2603_PMU_OC_Status, (0xf<<2), (high_bytes>>4)<<2);
	 }
	     pr_debug("[ATC260X PM] PMU_SYS_CTL0:%x, PMU_SYS_CTL3:%x, PMU_SYS_CTL5:%x DC2_CTL0:%x DC2_CTL1:%x\n", 
        atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_SYS_CTL0),
        atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_SYS_CTL3), 
        atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_SYS_CTL5),
		atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_DC2_CTL0),
		atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_DC2_CTL1)
		);
        break;
    case POWER_MODE_SLEEP:
        val = PMU_SYS_CTL3_EN_S3;
        break;
    case POWER_MODE_DEEP_SLEEP:
        val = 0;
        break;
    case POWER_MODE_WORKING:
        return;
    default:
        pr_err("[ATC260X PM] %s %d: invalid power mode\n", __FUNCTION__, __LINE__);
        return;
    }

    while(i<timeout_cnt)
    {
        i++;
        atc260x_set_bits(atc260x_pm.atc260x, 
        atc2603_PMU_SYS_CTL3,
        PMU_SYS_CTL3_EN_S2 | PMU_SYS_CTL3_EN_S3 | PMU_SYS_CTL3_FW_FLAG_S2, 
        val);
    
        mdelay(10);
		tmp = atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_SYS_CTL3) ;
        if(((tmp& PMU_SYS_CTL3_EN_S2)!=0 || (tmp & PMU_SYS_CTL3_EN_S3) != 0))
        {
            break;
        }
    }
    pr_debug("%s %d i=%d\n",__FUNCTION__,__LINE__,i);
    is_real_suspend = 1;
    
    pr_debug("[ATC260X PM] PMU_SYS_CTL0:%x, PMU_SYS_CTL3:%x, PMU_SYS_CTL5:%x DC2_CTL0:%x DC2_CTL1:%x\n", 
        atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_SYS_CTL0),
        atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_SYS_CTL3), 
        atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_SYS_CTL5),
		atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_DC2_CTL0),
		atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_DC2_CTL1)
		);


#ifdef CONFIG_ATC260X_SAVE_DDR_DELAY_CHAIN
    if (mode == POWER_MODE_STANDBY) {
        /* save delay chain value to atc2603 for mbrec init
          DCU_RD_DQS_DLY -- PMU_VBUS_CTL1[7:0]
          DCU_WR_DQS_DLY -- PMU_BAT_CTL1[7:0]
          DCU_CLK_DLY    -- PMU_WALL_CTL1[7:0]
         */

        /* only save one byte */
        tmp = atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_VBUS_CTL1) & (~0xff);
        val = act_readl(DCU_RD_DQS_DLY) & 0xff;
        tmp |= val;
        atc260x_reg_write(atc260x_pm.atc260x,
            atc2603_PMU_VBUS_CTL1,
            tmp);
        
        tmp = atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_BAT_CTL1) & (~0xff);
        val = act_readl(DCU_WR_DQS_DLY) & 0xff;
        tmp |= val;    
        
        atc260x_reg_write(atc260x_pm.atc260x,
            atc2603_PMU_BAT_CTL1,
            tmp);

        tmp = atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_WALL_CTL1) & (~0xff);
        val = act_readl(DCU_CLK_DLY) & 0xff;
        tmp |= val;   
        atc260x_reg_write(atc260x_pm.atc260x,
            atc2603_PMU_WALL_CTL1,
            tmp);
        
        printk("[ATC260X PM] DCU_RD_DQS_DLY:%x, DCU_WR_DQS_DLY:%x, atc2603_PMU_VBUS_CTL1:%x, atc2603_PMU_BAT_CTL1:0x%x\n",
            act_readl(DCU_RD_DQS_DLY),
            act_readl(DCU_WR_DQS_DLY),
            atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_VBUS_CTL1),
            atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_BAT_CTL1)
            );
    }
#endif

    /* enter S2 code in sleep.S */
    if (mode != POWER_MODE_STANDBY) {
        /*wurui 2014.1.27: clear unnormally shutdown flag */
        atc260x_set_bits(atc260x_pm.atc260x, 
            atc2603_PMU_SYS_CTL3,
            PMU_SYS_CTL3_FW_FLAG_Reset,
            0);
        /* disable S1 */
        atc260x_set_bits(atc260x_pm.atc260x, 
            atc2603_PMU_SYS_CTL1,
            PMU_SYS_CTL1_EN_S1,
            0);

        /* never come here */
        while(1);
    }
}

static void atc260x_wakeup(void)
{
    pr_info("wakeup,PM_SYS_CTL1:%x, CTL3:%x\n", 
        atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_SYS_CTL1),
        atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_SYS_CTL3));

#ifdef CONFIG_ATC260X_SAVE_DDR_DELAY_CHAIN
    pr_info("[ATC260X PM] DCU_RD_DQS_DLY:%x, DCU_WR_DQS_DLY:%x, atc2603_PMU_SYS_CTL8:%x\n",
        act_readl(DCU_RD_DQS_DLY),
        act_readl(DCU_WR_DQS_DLY),
        atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_SYS_CTL8));
#endif

    /* clear suspend flag */
    atc260x_set_bits(atc260x_pm.atc260x, 
        atc2603_PMU_SYS_CTL3,
        PMU_SYS_CTL3_FW_FLAG_S2,
        0);
}

static void atc260x_finsh_wakeup(void)
{
    pr_debug("[ATC260X PM] %s %d:\n", __FUNCTION__, __LINE__);

//    /* restore standard SPI interface */
//    atc260x_set_access_mode(atc260x_pm.atc260x, 
//        ATC260X_SPI_ACCESS_MODE_NORMAL);
}


static struct pmic_suspend_ops atc260x_pm_ops = {
	.set_wakeup_src	= atc260x_set_wakeup_src,
	.get_wakeup_src = atc260x_get_wakeup_src,
    .get_wakeup_flag = atc260x_get_wakeup_flag,
    .prepare = atc260x_prepare_suspend,
	.enter = atc260x_enter_suspend,
    .wake = atc260x_wakeup,
    .finish = atc260x_finsh_wakeup,
};

/* ---------- sysfs interface ---------- */

static ssize_t atc260x_wakeup_attr_shown(struct kobject *kobj, 
        struct kobj_attribute *attr, char *buf)
{
    int index = to_atc260x_pm_attr(attr)->index;
    int wakeup_src;

    pr_debug("%s %d: index %d\n", __FUNCTION__, __LINE__, index);
    
    wakeup_src = atc260x_get_wakeup_src();

    wakeup_src = (wakeup_src & (1 << index)) ? 1 : 0;

    return sprintf(buf, "%d\n", wakeup_src);
}

static ssize_t atc260x_wakeup_attr_store(struct kobject *kobj, 
        struct kobj_attribute *attr, const char *buf, size_t count)
{
    int index = to_atc260x_pm_attr(attr)->index;
    unsigned long val = 0;
    int ret;

    pr_debug("%s %d: index %d\n", __FUNCTION__, __LINE__, index);
    
    ret = strict_strtoul(buf, 0, &val);
    if (ret < 0)
        return ret;

    atc260x_set_wakeup_src(1 << index, (!!val) << index);

    return count;
}


static ssize_t atc260x_wakeup_srcs_attr_shown(struct kobject *kobj, 
        struct kobj_attribute *attr, char *buf)
{
    int wakeup_src;

    pr_debug("%s %d:\n", __FUNCTION__, __LINE__);
    
    wakeup_src = atc260x_get_wakeup_src();

    return sprintf(buf, "0x%x\n", wakeup_src);
}

static ssize_t atc260x_wakeup_srcs_attr_store(struct kobject *kobj, 
        struct kobj_attribute *attr, const char *buf, size_t count)
{
    unsigned long wakeup_src = 0;
    int ret;

    pr_debug("%s %d:\n", __FUNCTION__, __LINE__);
    
    ret = strict_strtoul(buf, 0, &wakeup_src);
    if (ret < 0)
        return ret;

    if (wakeup_src & ~WAKEUP_SRC_ALL)
        return -EINVAL;

    atc260x_set_wakeup_src(WAKEUP_SRC_ALL, wakeup_src);

    return count;
}

static ssize_t atc260x_s2tos3_timeout_shown(struct kobject *kobj, 
        struct kobj_attribute *attr, char *buf)
{
    int ret, i;
    int len = 0;

    pr_debug("%s %d:\n", __FUNCTION__, __LINE__);
    
    ret = atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_SYS_CTL2);

    if (ret & PMU_SYS_CTL2_S2_TIMER_EN) {
        i = (ret & PMU_SYS_CTL2_S2_TIMER_MASK) >> PMU_SYS_CTL2_S2_TIMER_SHIFT;
        len = sprintf(buf, "[%lu] ", s2tos3_timeout_table[i]);
    } else {
        len = sprintf(buf, "[%d] ", 0);
    }

    for (i = 0; i < ARRAY_SIZE(s2tos3_timeout_table); i++) {
        len += sprintf(buf + len, "%lu ", s2tos3_timeout_table[i]);
    }

    len += sprintf(buf + len, " (min)\n");

    return len;
}

static ssize_t atc260x_s2tos3_timeout_store(struct kobject *kobj, 
        struct kobj_attribute *attr, const char *instr, size_t bytes)
{
    unsigned long timeout = 0;
    int ret, i, tbl_size;

    pr_debug("%s %d:\n", __FUNCTION__, __LINE__);
    
    tbl_size = ARRAY_SIZE(s2tos3_timeout_table);

    ret = strict_strtoul(instr, 0, &timeout);
    if (ret)
        return ret;

    if (timeout == 0) {
        /* disable S2->S3 timer */
        ret = atc260x_set_bits(atc260x_pm.atc260x, 
            atc2603_PMU_SYS_CTL2,
            PMU_SYS_CTL2_S2_TIMER_MASK,
            0);

        return ret;
    }

    for (i = 0; i < tbl_size; i++) {
        if (timeout == s2tos3_timeout_table[i])
            break;
    }

    if (i == tbl_size)
        return -EINVAL;

    /* enable S2->S3 timer */
    ret = atc260x_set_bits(atc260x_pm.atc260x, 
        atc2603_PMU_SYS_CTL2,
        PMU_SYS_CTL2_S2_TIMER_MASK | PMU_SYS_CTL2_S2_TIMER_EN,
        (i << PMU_SYS_CTL2_S2_TIMER_SHIFT) | PMU_SYS_CTL2_S2_TIMER_EN);

    atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_SYS_CTL2);

    return bytes;
}


static ssize_t atc260x_s3tos4_timeout_shown(struct kobject *kobj, 
        struct kobj_attribute *attr, char *buf)
{
    int ret, i;
    int len = 0;

    pr_debug("%s %d:\n", __FUNCTION__, __LINE__);
    
    ret = atc260x_reg_read(atc260x_pm.atc260x, atc2603_PMU_SYS_CTL3);

    if (ret & PMU_SYS_CTL3_S3_TIMER_EN) {
        i = (ret & PMU_SYS_CTL3_S3_TIMER_MASK) >> PMU_SYS_CTL3_S3_TIMER_SHIFT;
        len = sprintf(buf, "[%lu] ", s2tos3_timeout_table[i]);
    } else {
        len = sprintf(buf, "[%d] ", 0);
    }

    for (i = 0; i < ARRAY_SIZE(s3tos4_timeout_table); i++) {
        len += sprintf(buf + len, "%lu ", s3tos4_timeout_table[i]);
    }

    len += sprintf(buf + len, " (min)\n");

    return len;
}

static ssize_t atc260x_s3tos4_timeout_store(struct kobject *kobj, 
        struct kobj_attribute *attr, const char *instr, size_t bytes)
{
    unsigned long timeout = 0;
    int ret, i, tbl_size;

    pr_debug("%s %d:\n", __FUNCTION__, __LINE__);
    
    tbl_size = ARRAY_SIZE(s3tos4_timeout_table);

    ret = strict_strtoul(instr, 0, &timeout);
    if (ret)
        return ret;

    if (timeout == 0) {
        /* disable S3->S4 timer */
        ret = atc260x_set_bits(atc260x_pm.atc260x, 
            atc2603_PMU_SYS_CTL3,
            PMU_SYS_CTL3_S3_TIMER_MASK,
            0);

        return ret;
    }

    for (i = 0; i < tbl_size; i++) {
        if (timeout == s3tos4_timeout_table[i])
            break;
    }

    if (i == tbl_size)
        return -EINVAL;

    ret = atc260x_set_bits(atc260x_pm.atc260x, 
        atc2603_PMU_SYS_CTL3,
        PMU_SYS_CTL3_S3_TIMER_MASK | PMU_SYS_CTL3_S3_TIMER_EN,
        (i << PMU_SYS_CTL3_S3_TIMER_SHIFT) | PMU_SYS_CTL3_S3_TIMER_EN);

    return bytes;
}

#define ATC260X_PM_ATTR(_name, _mode, _show, _store, _index)	\
	{ .attr = __ATTR(_name, _mode, _show, _store),	\
	  .index = _index }

#define WAKEUP_ATTR(_name, _index)	\
struct atc260x_pm_attribute atc260x_wakeup_attr_##_name		\
	= ATC260X_PM_ATTR(_name, S_IRUGO | S_IWUSR, atc260x_wakeup_attr_shown, atc260x_wakeup_attr_store, _index)

#define WAKEUP_ATTR_PTR(_name) \
    &atc260x_wakeup_attr_##_name.attr.attr

#define PM_ATTR(_name, _mode, _show, _store)	\
struct atc260x_pm_attribute atc260x_pm_attr_##_name		\
	= ATC260X_PM_ATTR(_name, _mode, _show, _store, 0)

#define PM_ATTR_PTR(_name)	\
    &atc260x_pm_attr_##_name.attr.attr

WAKEUP_ATTR(wken_ir, WAKEUP_SRC_OFFSET_IR);
WAKEUP_ATTR(wken_reset, WAKEUP_SRC_OFFSET_RESET);
WAKEUP_ATTR(wken_hdsw, WAKEUP_SRC_OFFSET_HDSW);
WAKEUP_ATTR(wken_alarm, WAKEUP_SRC_OFFSET_ALARM);
WAKEUP_ATTR(wken_remcon, WAKEUP_SRC_OFFSET_REMCON);
WAKEUP_ATTR(wken_tp, WAKEUP_SRC_OFFSET_TP);
WAKEUP_ATTR(wken_wkirq, WAKEUP_SRC_OFFSET_WKIRQ);
WAKEUP_ATTR(wken_onoff_short, WAKEUP_SRC_OFFSET_ONOFF_SHORT);
WAKEUP_ATTR(wken_onoff_long, WAKEUP_SRC_OFFSET_ONOFF_LONG);
WAKEUP_ATTR(wken_wall, WAKEUP_SRC_OFFSET_WALL_IN);
WAKEUP_ATTR(wken_vbus, WAKEUP_SRC_OFFSET_VBUS_IN);

PM_ATTR(wakeup_srcs, S_IRUGO | S_IWUSR, atc260x_wakeup_srcs_attr_shown, \
    atc260x_wakeup_srcs_attr_store);

PM_ATTR(s2tos3_timeout, S_IRUGO | S_IWUSR, atc260x_s2tos3_timeout_shown, \
    atc260x_s2tos3_timeout_store);
PM_ATTR(s3tos4_timeout, S_IRUGO | S_IWUSR, atc260x_s3tos4_timeout_shown, \
    atc260x_s3tos4_timeout_store);


static struct attribute *atc260x_pm_attrs[] = {
    WAKEUP_ATTR_PTR(wken_ir),
    WAKEUP_ATTR_PTR(wken_reset),
    WAKEUP_ATTR_PTR(wken_hdsw),
    WAKEUP_ATTR_PTR(wken_alarm),
    WAKEUP_ATTR_PTR(wken_remcon),
    WAKEUP_ATTR_PTR(wken_tp),
    WAKEUP_ATTR_PTR(wken_wkirq),
    WAKEUP_ATTR_PTR(wken_onoff_short),
    WAKEUP_ATTR_PTR(wken_onoff_long),
    WAKEUP_ATTR_PTR(wken_wall),
    WAKEUP_ATTR_PTR(wken_vbus),

    PM_ATTR_PTR(wakeup_srcs),  

    PM_ATTR_PTR(s2tos3_timeout),  
    PM_ATTR_PTR(s3tos4_timeout),  

    NULL,
};

static struct attribute_group asoc_pmattr_group = {
	.name = "wakeups",
	.attrs = atc260x_pm_attrs,
};

static __devinit int atc260x_pm_probe(struct platform_device *pdev)
{
    struct atc260x_dev *atc260x = dev_get_drvdata(pdev->dev.parent);
    struct atc260x_pm *pm = &atc260x_pm;
    int ret;

    printk("[ATC260X] Probing PM\n");

    pm->atc260x = atc260x;

    platform_set_drvdata(pdev, pm);

    pmic_suspend_set_ops(&atc260x_pm_ops);

		pr_debug("CTL0:%x, CTL1:%x, CTL2:%x, CTL3:%x, CTL8:%x\n", 
        atc260x_reg_read(atc260x, atc2603_PMU_SYS_CTL0),
        atc260x_reg_read(atc260x, atc2603_PMU_SYS_CTL1),
        atc260x_reg_read(atc260x, atc2603_PMU_SYS_CTL2),
        atc260x_reg_read(atc260x, atc2603_PMU_SYS_CTL3),
        atc260x_reg_read(atc260x, atc2603_PMU_SYS_CTL8));

	atc260x_set_bits(atc260x, atc2603_PMU_SYS_CTL1,PMU_SYS_CTL1_LB_S4_MASK,0);

    ret = sysfs_create_group(power_kobj, &asoc_pmattr_group);
    atc260x_set_wakeup_src(WAKEUP_SRC_ALL, 
            WAKEUP_SRC_RESET |WAKEUP_SRC_ONOFF_LONG 
            |WAKEUP_SRC_WALL_IN | WAKEUP_SRC_VBUS_IN);
    printk("\n #####%s atc2603_PMU_SYS_CTL0:0x%x ####", __FUNCTION__, atc260x_reg_read(atc260x, atc2603_PMU_SYS_CTL0));

    return ret;
}

static __devexit int atc260x_pm_remove(struct platform_device *pdev)
{
    sysfs_remove_group(power_kobj, &asoc_pmattr_group);

    platform_set_drvdata(pdev, NULL);

    return 0;
}

static struct platform_driver atc260x_pm_driver = {
    .probe = atc260x_pm_probe,
    .remove = __devexit_p(atc260x_pm_remove),
    .driver     = {
        .name   = "atc260x-pm",
        .owner  = THIS_MODULE,
    },
};

/*是否是闰年*/
int leap(int year)
{ 
    if(((year%4==0) && (year%100 != 0)) || (year%400==0))
    {
        return 1; 
    }
    else
    {  
        return 0;
    }
}

int month_day(int year, int month)
{
    int day_tab[12]={
//        {31,28,31,30,31,30,31,31,30,31,30,31},
        31,29,31,30,31,30,31,31,30,31,30,31};
    if(leap(year) && (month==2))
    {
        return 28;
    }
    
    return day_tab[month-1];
}

int adjust_time(unsigned short * rtc_ms_value, unsigned short* rtc_h_value, unsigned short* rtc_ymd_value, unsigned short timevalue)
{
    unsigned short min = 0, sec = 0, hour=0 , day=0, mon=0, year=0; 
    unsigned short rtc_ms=0, rtc_h=0, rtc_dc=0, rtc_ymd=0;
    
    if(!rtc_ms_value || !rtc_h_value || !rtc_ymd_value)
    {
        return -1;
    }
    
    rtc_ms = *rtc_ms_value;
    rtc_h = *rtc_h_value;
    rtc_ymd = *rtc_ymd_value;
    
    hour = (rtc_h & 0x1f);
    min = ((rtc_ms & (0x3f<<6))>>6);
    sec = (rtc_ms & 0x3f);
    day = (rtc_ymd & 0x1f);
    mon = ((rtc_ymd & (0xf<<5))>>5);
    year = ((rtc_ymd & (0x7f<<9))>>9);
    printk("\n %s %d year:%d, mon:%d, day:%d, hour:%d, min:%d, sec:%d", __FUNCTION__,__LINE__, year, mon, day, hour, min, sec);
    
    sec = sec+timevalue;
    if(sec>=60)
    {
        sec = sec-60; 
        min = min + 1;
        if(min>=60)
        {
            min = min-60;
            hour = hour+1;
            if(hour>=24)
            {
                hour = hour-24;
                day = day+1;
                if(day >= month_day(year, mon))
                {
                    day = day - month_day(year, mon);
                    mon= mon+1;
                    if(mon>=12)
                    {
                        mon = mon-12;
                        year = year+1;
                    }
                }
            }
        }
    }
  
   printk("\n %s %d year:%d, mon:%d, day:%d, hour:%d, min:%d, sec:%d", __FUNCTION__,__LINE__, year, mon, day, hour, min, sec);
   
   *rtc_ymd_value = (((year&0x7f)<<9)|((mon&0xf)<<5)|(day&0x1f));
   *rtc_h_value = (hour&0x1f);
   *rtc_ms_value = (((min&0x3f)<<6) | (sec)); 
   return 0;
    
}

int atc260x_set_alarm_restart_time(int timevalue)
{
    unsigned short msalm = 0;
    unsigned short halm = 0;
    unsigned short ymdalm = 0;
    unsigned short tmp = 0;
	int pmu_type;
    int ret =0;
//    unsigned short min = 0, sec = 0, hour=0 , day=0, mon=0, year=0; 
    
    unsigned short rtc_ms=0, rtc_h=0, rtc_dc=0, rtc_ymd=0;
    struct atc260x_dev *atc260x = atc260x_pm.atc260x;
    pmu_type =get_pmu_type();
    /*
      将当前alarm信息保存到pmu不掉电寄存器中
      //RTC_MSALM  ---  PMU_SYS_CLT9 bit[11:0]
      RTC_MSALM[7:0] -- PMU_SYS_CLT9 bit[7:0]
      RTC_MSALM[11:8] -- atc2603_PMU_SYS_CTL3[9:6]
      RTC_HALM   ---  PMU_VBUS_CTL1 bit[4:0]
      RTC_YMDALM  --- PMU_SYS_CTL8 bit[15:0] 
     */
    msalm = atc260x_reg_read(atc260x, atc2603_RTC_MSALM)&0xfff;
    halm = atc260x_reg_read(atc260x, atc2603_RTC_HALM)&0x1f;
    ymdalm = atc260x_reg_read(atc260x, atc2603_RTC_YMDALM); 
    
//    printk("\n %s atc2603_RTC_MSALM:0x%x, atc2603_RTC_HALM:0x%x, atc2603_RTC_YMDALM:0x%x", __FUNCTION__,
//        atc260x_reg_read(atc260x, atc2603_RTC_MSALM),
//        atc260x_reg_read(atc260x, atc2603_RTC_HALM),
//        atc260x_reg_read(atc260x, atc2603_RTC_YMDALM));
        
//    tmp = atc260x_reg_read(atc260x, atc2603_PMU_SYS_CTL9);
//    tmp &= ~(0xfff);
//    tmp |= msalm;
//    atc260x_reg_write(atc260x, atc2603_PMU_SYS_CTL9, tmp);
   if (pmu_type ==ATC2603A)
		atc260x_set_bits(atc260x, atc2603_PMU_SYS_CTL9, 0xff, msalm&0xff);
		atc260x_set_bits(atc260x, atc2603_PMU_SYS_CTL3, (0xf<<6), ((msalm>>8)&0xf)<<6);
	if (pmu_type==ATC2603C)
		atc260x_set_bits(atc260x, atc2603_PMU_FW_USE0, 0xFFF, msalm);
    
    tmp = atc260x_reg_read(atc260x, atc2603_PMU_VBUS_CTL1);
    tmp &= ~0x1f;
    tmp |= halm;
    atc260x_reg_write(atc260x, atc2603_PMU_VBUS_CTL1, tmp);
        
    atc260x_reg_write(atc260x, atc2603_PMU_SYS_CTL8, ymdalm);
    
//    printk("\n %s msalm:0x%x, halm:0x%x, ymdalm:0x%x", __FUNCTION__, msalm, halm, ymdalm);
    
//    printk("\n %s atc2603_PMU_SYS_CTL9:0x%x, atc2603_PMU_VBUS_CTL1:0x%x, atc2603_PMU_SYS_CTL8:0x%x", __FUNCTION__,
//        atc260x_reg_read(atc260x, atc2603_PMU_SYS_CTL9),
//        atc260x_reg_read(atc260x, atc2603_PMU_VBUS_CTL1),
//        atc260x_reg_read(atc260x, atc2603_PMU_SYS_CTL8));
        
    /*读取当前时间信息,设置alarm寄存器*/
    rtc_ms = atc260x_reg_read(atc260x, atc2603_RTC_MS);
    rtc_h  = atc260x_reg_read(atc260x, atc2603_RTC_H);
//    rtc_dc = atc260x_reg_read(atc260x, atc2603_RTC_DC);
    rtc_ymd = atc260x_reg_read(atc260x, atc2603_RTC_YMD);  
    
    ret = adjust_time(&rtc_ms, &rtc_h, &rtc_ymd, timevalue);
   
    if(ret != 0)
    {
        printk("\n########### ERROR at %s %d#########", __FUNCTION__, __LINE__);
        return -1;
    }
    
    atc260x_reg_write(atc260x, atc2603_RTC_YMDALM, rtc_ymd);
    atc260x_reg_write(atc260x, atc2603_RTC_HALM, rtc_h);
    atc260x_reg_write(atc260x, atc2603_RTC_MSALM, rtc_ms);
   
    return 0;
}
EXPORT_SYMBOL(atc260x_set_alarm_restart_time);

int set_restart_handler(void* handler);

static int __init atc260x_pm_init(void)
{
    int ret;
    set_restart_handler(atc260x_set_alarm_restart_time);
    ret = platform_driver_register(&atc260x_pm_driver);
    if (ret != 0)
        pr_err("Failed to register ATC260X PM driver: %d\n", ret);

    return 0;
}
module_init(atc260x_pm_init);

static void __exit atc260x_pm_exit(void)
{
    platform_driver_unregister(&atc260x_pm_driver);
}
module_exit(atc260x_pm_exit);

/* Module information */
MODULE_AUTHOR("Actions Semi, Inc");
MODULE_DESCRIPTION("ATC260X PM driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:atc260x-pm");
