/*
 * atc260x_dcdc.c  --  DCDC driver for ATC260X
 *
 * Copyright 2011 Actions Semi Inc.
 * Author: Actions Semi, Inc.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/slab.h>
//#include <linux/earlysuspend.h>
#include <linux/gpio.h>
#include <mach/hardware.h>
#include <mach/atc260x/atc260x.h>
#include <mach/atc260x/atc260x_pdata.h>
#include <mach/atc260x/atc260x_dev.h>
#include <linux/delay.h>

/* DCDC_CTL0 */
#define DCDC_CTL0_MODE                      (1 << 6)

/* DCDC_CTL1 */
#define DCDC_CTL1_EN_OCPL                   (1 << 15)


#define ATC260X_DCDC_BOOST_NUM              1
#define ATC260X_DCDC_BUCK_NUM               3           /* DCDC1~DCDC31 */

#define ATC260X_DCDC_BUCK_ENABLE            (1 << 15)   /* bit 15 */
#define ATC260X_DCDC_BOOST_ENABLE           (1 << 0)    /* bit 0 */

#define ATC260X_DCDC_MAX_NAME               8

// #define ATC260X_DCDC4_EXTERNAL              1

#ifdef ATC260X_DCDC4_EXTERNAL

#define MOD_ID_DCDC  MOD_ID_USBC0

//static int dcdc4_is_external = 1;

//static int gpio_status = 0 ;

//static struct gpio_pre_cfg external_dcdc4_gpio;

//static unsigned int  external_dcdc4_pin = 0;

//#define GPIO_EXTERNAL_DCDC4		"otg_vbus"

#endif

struct atc260x_dcdc_property
{
    int ctl_reg;
    unsigned int min_uv;            /* min voltage (uV) */  
    unsigned int step_uv;           /* step voltage (uV) */
    u8 step_num;
    u8 step_bits_mask;
    u8 step_bits_shift; 
    u8 bias_bit_shift;
    unsigned int opt;
};

struct atc260x_dcdc {
    char name[ATC260X_DCDC_MAX_NAME];
    struct regulator_desc desc;
    struct atc260x_dev *atc260x;
    struct regulator_dev *regulator;
    struct atc260x_dcdc_property    *property;
 //   struct early_suspend early_suspend;
};

#define GET_SELECTOR(property, vsel)        (((vsel) >> (property)->step_bits_shift) & (property)->step_bits_mask)
#define SET_SELECTOR(property, vsel)        (((vsel) & (property)->step_bits_mask) << (property)->step_bits_shift)

/* DCDC property options */
#define DCDC_OPT_ALWAYS_ON                  (1 << 0)        /* DCDC is always on, cannot control by software */

static struct atc260x_dcdc_property dcdcs_property[ATC260X_DCDC_MAX_NUM] = {

    /* ctl_reg              min_uv      step_uv   step_num  step_shift  bias_shift  opt */
    
    /* DCDC 1, 0.7~1.4v */
    {atc2603_PMU_DC1_CTL0,   700000,     25000,      29,     0x1f,   7,      0,      DCDC_OPT_ALWAYS_ON}, 

    /* DCDC 2, 1.3~2.2v */
    {atc2603_PMU_DC2_CTL0,   1300000,    50000,      32,     0xf,    8,      0,      0}, 

    /* DCDC 3, 2.6~3.3v */
    {atc2603_PMU_DC3_CTL0,   2600000,    100000,     8,      0x7,    9,      11,     DCDC_OPT_ALWAYS_ON}, 

    /* DCDC 5, 5v */
    {atc2603_PMU_DC4_CTL0,   5000000,    0,          0,      0x0,    0,      0,      0}, 
};


static int atc260x_buck_list_voltage(struct regulator_dev *rdev,
                      unsigned int selector)
{
    struct atc260x_dcdc *dcdc = rdev_get_drvdata(rdev);
    int id = rdev_get_id(rdev);

    if (id > ATC260X_DCDC_BUCK_NUM) {
        return -EINVAL; 
    }

    if (selector < dcdc->property->step_num)
        return dcdc->property->min_uv + (selector * dcdc->property->step_uv);

    return -EINVAL;
}

static int atc260x_buck_is_enabled(struct regulator_dev *rdev)
{
    struct atc260x_dcdc *dcdc = rdev_get_drvdata(rdev);
    struct atc260x_dev *atc260x = dcdc->atc260x;
    int ret;

    if (dcdc->property->opt & DCDC_OPT_ALWAYS_ON)
        return 1;
        
    

    ret = atc260x_reg_read(atc260x, dcdc->property->ctl_reg);
    if (ret < 0)
        return ret;

    if (ret & ATC260X_DCDC_BUCK_ENABLE)
        return 1;
    else
        return 0;
}

static int atc260x_buck_enable(struct regulator_dev *rdev)
{
    struct atc260x_dcdc *dcdc = rdev_get_drvdata(rdev);
    struct atc260x_dev *atc260x = dcdc->atc260x;
    
    if (dcdc->property->opt & DCDC_OPT_ALWAYS_ON)
        return 0;

    return atc260x_set_bits(atc260x, dcdc->property->ctl_reg, ATC260X_DCDC_BUCK_ENABLE, ATC260X_DCDC_BUCK_ENABLE);
}

static int atc260x_buck_disable(struct regulator_dev *rdev)
{
    struct atc260x_dcdc *dcdc = rdev_get_drvdata(rdev);
    struct atc260x_dev *atc260x = dcdc->atc260x;

    if (dcdc->property->opt & DCDC_OPT_ALWAYS_ON)
        return 0;

    return atc260x_set_bits(atc260x, dcdc->property->ctl_reg, ATC260X_DCDC_BUCK_ENABLE, 0);
}

static int atc260x_buck_set_voltage(struct regulator_dev *rdev,
                     int min_uV, int max_uV,unsigned* selector)
{
    struct atc260x_dcdc *dcdc = rdev_get_drvdata(rdev);
    struct atc260x_dev *atc260x = dcdc->atc260x;
    int ret, reg, vsel;

    reg = dcdc->property->ctl_reg;

    vsel = (min_uV - dcdc->property->min_uv) / dcdc->property->step_uv;
        
    ret = atc260x_buck_list_voltage(rdev, vsel);
    if (ret < 0)
        return ret;
    if (ret < min_uV || ret > max_uV)
        return -EINVAL;

    *selector = vsel; /*add by jlingzhang*/
    ret = atc260x_set_bits(atc260x, reg, 
        SET_SELECTOR(dcdc->property, dcdc->property->step_bits_mask), 
        SET_SELECTOR(dcdc->property, vsel));
	mdelay(1);
	return ret;
}

static int atc260x_buck_get_voltage(struct regulator_dev *rdev)
{
    struct atc260x_dcdc *dcdc = rdev_get_drvdata(rdev);
    struct atc260x_dev *atc260x = dcdc->atc260x;
    int ret;

    ret = atc260x_reg_read(atc260x, dcdc->property->ctl_reg);
    if (ret < 0)
        return ret;

    ret = GET_SELECTOR(dcdc->property, ret);

    return atc260x_buck_list_voltage(rdev, ret);;
}

unsigned int atc260x_buck_get_mode(struct regulator_dev *rdev)
{
    struct atc260x_dcdc *dcdc = rdev_get_drvdata(rdev);
    struct atc260x_dev *atc260x = dcdc->atc260x;
    int ret;

    pr_err("[DCDC] get mode\n");

    ret = atc260x_reg_read(atc260x, dcdc->property->ctl_reg);
    if (ret < 0)
        return ret;

    if (ret & DCDC_CTL0_MODE)
        return REGULATOR_MODE_NORMAL;
    else
        return REGULATOR_MODE_IDLE;
}

static struct regulator_ops atc260x_buck_ops = {
    .list_voltage = atc260x_buck_list_voltage,
    .get_voltage = atc260x_buck_get_voltage,
    .set_voltage = atc260x_buck_set_voltage,
    .get_mode = atc260x_buck_get_mode,
    .is_enabled = atc260x_buck_is_enabled,
    .enable = atc260x_buck_enable,
    .disable = atc260x_buck_disable,
};

/* DCDC3 always use PWM mode */
#if 0
/* FIXME: DCDC3 for EVB still use the LDO mode */
#ifndef CONFIG_GL5201_EVB
#ifdef CONFIG_HAS_EARLYSUSPEND
static void atc260x_dcdc3_early_suspend(struct early_suspend *h)
{
	struct atc260x_dcdc *dcdc = container_of(h, struct atc260x_dcdc, early_suspend);
    struct atc260x_dev *atc260x = dcdc->atc260x;
    
    pr_info("\n[DCDC3] early suspend\n");
    pr_info("[DCDC3] ctl0:%04x, ctl1:%04x\n",
        atc260x_reg_read(atc260x, dcdc->property->ctl_reg),
        atc260x_reg_read(atc260x, dcdc->property->ctl_reg + 1));

    atc260x_set_bits(atc260x, dcdc->property->ctl_reg,
        DCDC_CTL0_MODE, 0);

    atc260x_set_bits(atc260x, dcdc->property->ctl_reg + 1,
        DCDC_CTL1_EN_OCPL, DCDC_CTL1_EN_OCPL);
}

static void atc260x_dcdc3_late_resume(struct early_suspend *h)
{
	struct atc260x_dcdc *dcdc = container_of(h, struct atc260x_dcdc, early_suspend);
    struct atc260x_dev *atc260x = dcdc->atc260x;

    pr_info("\n[DCDC3] late resume\n");
    pr_info("[DCDC3] ctl0:%04x, ctl1:%04x\n",
        atc260x_reg_read(atc260x, dcdc->property->ctl_reg),
        atc260x_reg_read(atc260x, dcdc->property->ctl_reg + 1));

    atc260x_set_bits(atc260x, dcdc->property->ctl_reg + 1,
        DCDC_CTL1_EN_OCPL, 0);

    atc260x_set_bits(atc260x, dcdc->property->ctl_reg,
        DCDC_CTL0_MODE, DCDC_CTL0_MODE);
}
#endif
#endif

static int atc260x_dcdc_suspend(struct platform_device *pdev, pm_message_t m)
{
    struct atc260x_dcdc *dcdc = platform_get_drvdata(pdev);
    struct atc260x_dev *atc260x = dcdc->atc260x;

    dev_info(&pdev->dev, "atc260x_dcdc_resume() - DCDC%d\n",
        dcdc->desc.id + 1);

#ifndef CONFIG_GL5201_EVB
    /* switch DCDC3 to PWM mode.
     * PFM mode maybe not afford the heavy load while wakeup
     */
    if (dcdc->desc.id == 2) {
        pr_info("[DCDC3] ctl0:%04x, ctl1:%04x\n",
            atc260x_reg_read(atc260x, dcdc->property->ctl_reg),
            atc260x_reg_read(atc260x, dcdc->property->ctl_reg + 1));

        atc260x_set_bits(atc260x, dcdc->property->ctl_reg + 1,
            DCDC_CTL1_EN_OCPL, 0);

        atc260x_set_bits(atc260x, dcdc->property->ctl_reg,
            DCDC_CTL0_MODE, DCDC_CTL0_MODE);
    }
#endif

    return 0;
}

static int atc260x_dcdc_resume(struct platform_device *pdev)
{
    struct atc260x_dcdc *dcdc = platform_get_drvdata(pdev);

    dev_info(&pdev->dev, "atc260x_dcdc_resume() - DCDC%d\n",
        dcdc->desc.id + 1);

    return 0;
}
#endif

static __devinit int atc260x_buck_probe(struct platform_device *pdev)
{
    struct atc260x_dev *atc260x = dev_get_drvdata(pdev->dev.parent);
    struct atc260x_pdata *pdata = atc260x->dev->platform_data;
    int id = pdev->id % ARRAY_SIZE(pdata->dcdc);
    struct atc260x_dcdc *dcdc;
    int ret;

    printk("[ATC260X] Probing DCDC%d\n", id + 1);

    if (pdata == NULL || pdata->dcdc[id] == NULL)
        return -ENODEV;

    if (id >= ATC260X_DCDC_BUCK_NUM) {
        return -EINVAL; 
    }

    dcdc = kzalloc(sizeof(struct atc260x_dcdc), GFP_KERNEL);
    if (dcdc == NULL) {
        dev_err(&pdev->dev, "Unable to allocate private data\n");
        return -ENOMEM;
    }

    dcdc->atc260x = atc260x;

    dcdc->property = &dcdcs_property[id];

    snprintf(dcdc->name, sizeof(dcdc->name), "DCDC%d", id + 1);
    dcdc->desc.name = dcdc->name;
    dcdc->desc.id = id;
    dcdc->desc.type = REGULATOR_VOLTAGE;
    dcdc->desc.n_voltages = dcdc->property->step_num;
    dcdc->desc.ops = &atc260x_buck_ops;
    dcdc->desc.owner = THIS_MODULE;
		
    dcdc->regulator = regulator_register(&dcdc->desc, &pdev->dev,
                         pdata->dcdc[id], dcdc, NULL);
    if (IS_ERR(dcdc->regulator)) {
        ret = PTR_ERR(dcdc->regulator);
        dev_err(atc260x->dev, "Failed to register DCDC%d: %d\n",
            id + 1, ret);
        goto err;
    }

    platform_set_drvdata(pdev, dcdc);

#if 0
#ifndef CONFIG_GL5201_EVB
#ifdef CONFIG_HAS_EARLYSUSPEND
    if (id == 2) {
        pr_err("[DCDC3] register early suspend\n");
        /* 
         * switch dcdc3 to PFM mode after lcd closed, and 
         * switch dcdc3 to PWM mode before lcd open 
         */
        dcdc->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
        dcdc->early_suspend.suspend = atc260x_dcdc3_early_suspend;
        dcdc->early_suspend.resume = atc260x_dcdc3_late_resume;
        register_early_suspend(&dcdc->early_suspend);
    }
#endif
#endif
#endif

    return 0;

//err_regulator:
//  regulator_unregister(dcdc->regulator);
err:
    kfree(dcdc);
    return ret;
}

static __devexit int atc260x_buck_remove(struct platform_device *pdev)
{
    struct atc260x_dcdc *dcdc = platform_get_drvdata(pdev);

    platform_set_drvdata(pdev, NULL);

    regulator_unregister(dcdc->regulator);
    kfree(dcdc);

    return 0;
}


static struct platform_driver atc260x_buck_driver = {
    .probe = atc260x_buck_probe,
    .remove = __devexit_p(atc260x_buck_remove),
    .driver     = {
        .name   = "atc260x-buck",
        .owner  = THIS_MODULE,
    },
#if 0    
    .suspend     = atc260x_dcdc_suspend,
    .resume     = atc260x_dcdc_resume,
#endif    
};

/*
 * DCDC boost convertors
 */

static int atc260x_boost_is_enabled(struct regulator_dev *rdev)
{
    struct atc260x_dcdc *dcdc = rdev_get_drvdata(rdev);
    struct atc260x_dev *atc260x = dcdc->atc260x;
    int ret;
    
#ifdef ATC260X_DCDC4_EXTERNAL
    if(dcdc4_is_external){
    	if(gpio_status == external_dcdc4_gpio.active_level){
        	return 1;
      }else{
      	  return 0;
      }
    }
#endif
    ret = atc260x_reg_read(atc260x, dcdc->property->ctl_reg);
    if (ret < 0)
        return ret;

    if (ret & ATC260X_DCDC_BOOST_ENABLE)
        return 1;
    else
        return 0;
}

static int atc260x_boost_enable(struct regulator_dev *rdev)
{
    struct atc260x_dcdc *dcdc = rdev_get_drvdata(rdev);
    struct atc260x_dev *atc260x = dcdc->atc260x;    
#ifdef ATC260X_DCDC4_EXTERNAL
    if(dcdc4_is_external){
    	gpio_direction_output(external_dcdc4_pin,external_dcdc4_gpio.active_level);
    	gpio_status = external_dcdc4_gpio.active_level;
    	return 1;
    }
#endif
    return atc260x_set_bits(atc260x, dcdc->property->ctl_reg, 
        ATC260X_DCDC_BOOST_ENABLE, ATC260X_DCDC_BOOST_ENABLE);
}

static int atc260x_boost_disable(struct regulator_dev *rdev)
{
    struct atc260x_dcdc *dcdc = rdev_get_drvdata(rdev);
    struct atc260x_dev *atc260x = dcdc->atc260x;
    
#ifdef ATC260X_DCDC4_EXTERNAL
    if(dcdc4_is_external){
    	gpio_direction_output(external_dcdc4_pin,external_dcdc4_gpio.unactive_level);
    	gpio_status = external_dcdc4_gpio.unactive_level;
    	return 1;
    }
#endif

    return atc260x_set_bits(atc260x, dcdc->property->ctl_reg, 
        ATC260X_DCDC_BOOST_ENABLE, 0);
}

static int atc260x_boost_get_status(struct regulator_dev *rdev)
{
    struct atc260x_dcdc *dcdc = rdev_get_drvdata(rdev);
    struct atc260x_dev *atc260x = dcdc->atc260x;
    int ret, reg;
    
#ifdef ATC260X_DCDC4_EXTERNAL
    if(dcdc4_is_external){
    	if(gpio_status == external_dcdc4_gpio.active_level){
        	return 1;
      }else{
      	  return 0;
      }
    }
#endif
    /* First, check for errors */
    ret = atc260x_reg_read(atc260x, atc2603_PMU_UV_Status);
    if (ret < 0)
        return ret;

    if (ret & (1 << (15 - rdev_get_id(rdev)))) {
        dev_dbg(atc260x->dev, "DCDC%d under voltage\n",
            rdev_get_id(rdev) + 1);
        return REGULATOR_STATUS_ERROR;
    }

    /* Is the regulator on? */
    reg = dcdc->property->ctl_reg;
    
    ret = atc260x_reg_read(atc260x, reg);
    if (ret < 0)
        return ret;
    if (ret & ATC260X_DCDC_BOOST_ENABLE)
        return REGULATOR_STATUS_ON;
    else
        return REGULATOR_STATUS_OFF;
}

static struct regulator_ops atc260x_boost_ops = {
    .get_status = atc260x_boost_get_status,

    .is_enabled = atc260x_boost_is_enabled,
    .enable = atc260x_boost_enable,
    .disable = atc260x_boost_disable,
};

static __devinit int atc260x_boost_probe(struct platform_device *pdev)
{
    struct atc260x_dev *atc260x = dev_get_drvdata(pdev->dev.parent);
    struct atc260x_pdata *pdata = atc260x->dev->platform_data;
    int id = pdev->id % ARRAY_SIZE(pdata->dcdc);
    struct atc260x_dcdc *dcdc;
    int ret;

    printk("[ATC260X] Probing DCDC%d (5v)\n", id + 1);

    if (pdata == NULL || pdata->dcdc[id] == NULL)
        return -ENODEV;

    if (id != 3) {
        return -EINVAL; 
    }

    dcdc = kzalloc(sizeof(struct atc260x_dcdc), GFP_KERNEL);
    if (dcdc == NULL) {
        dev_err(&pdev->dev, "Unable to allocate private data\n");
        return -ENOMEM;
    }

    dcdc->atc260x = atc260x;

    dcdc->property = &dcdcs_property[id];

    snprintf(dcdc->name, sizeof(dcdc->name), "DCDC%d", id + 1);
    dcdc->desc.name = dcdc->name;
    dcdc->desc.id = id;
    dcdc->desc.type = REGULATOR_VOLTAGE;
    dcdc->desc.n_voltages = dcdc->property->step_num;
    dcdc->desc.ops = &atc260x_boost_ops;
    dcdc->desc.owner = THIS_MODULE;
    
#ifdef ATC260X_DCDC4_EXTERNAL
		if(get_config("dcdc_dcdc4", (char *)&dcdc4_is_external,4) != 0){
			printk("dcdc4 is not config and used  external as default  \n");
			dcdc4_is_external = 1;
		}		
		if(dcdc4_is_external){
			memset(&external_dcdc4_gpio, 0, sizeof(external_dcdc4_gpio));
		  gpio_get_pre_cfg(MOD_ID_DCDC, GPIO_EXTERNAL_DCDC4, &external_dcdc4_gpio);
		  external_dcdc4_pin = ASOC_GPIO_PORT(external_dcdc4_gpio.iogroup, external_dcdc4_gpio.pin_num);	
		  ret = gpio_request(external_dcdc4_pin, GPIO_EXTERNAL_DCDC4);
		  if(ret) {
					printk(KERN_ERR "external_dcdc4_pin request err ret:%d\n",ret);
					external_dcdc4_pin = 0;
			} else {
					gpio_direction_output(external_dcdc4_pin,external_dcdc4_gpio.active_level);
					gpio_status = external_dcdc4_gpio.active_level;
			}
		}
#endif

    dcdc->regulator = regulator_register(&dcdc->desc, &pdev->dev,
                         pdata->dcdc[id], dcdc, NULL);
    if (IS_ERR(dcdc->regulator)) {
        ret = PTR_ERR(dcdc->regulator);
        dev_err(atc260x->dev, "Failed to register DCDC%d: %d\n",
            id + 1, ret);
        goto err;
    }

    platform_set_drvdata(pdev, dcdc);

    return 0;

//err_regulator:
//  regulator_unregister(dcdc->regulator);
err:
    kfree(dcdc);
    return ret;
}

static __devexit int atc260x_boost_remove(struct platform_device *pdev)
{
    struct atc260x_dcdc *dcdc = platform_get_drvdata(pdev);

    platform_set_drvdata(pdev, NULL);

    regulator_unregister(dcdc->regulator);
    kfree(dcdc);

    return 0;
}

static struct platform_driver atc260x_boost_driver = {
    .probe = atc260x_boost_probe,
    .remove = __devexit_p(atc260x_boost_remove),
    .driver     = {
        .name   = "atc260x-boost",
        .owner  = THIS_MODULE,
    },
};


static int __init atc260x_dcdc_init(void)
{
    int ret;

    ret = platform_driver_register(&atc260x_buck_driver);
    if (ret != 0)
        pr_err("Failed to register ATC260X buck driver: %d\n", ret);

    ret = platform_driver_register(&atc260x_boost_driver);
    if (ret != 0)
        pr_err("Failed to register ATC260X boost driver: %d\n", ret);


    return 0;
}
subsys_initcall(atc260x_dcdc_init);

static void __exit atc260x_dcdc_exit(void)
{
    platform_driver_unregister(&atc260x_boost_driver);
    platform_driver_unregister(&atc260x_buck_driver);   
}
module_exit(atc260x_dcdc_exit);

/* Module information */
MODULE_AUTHOR("Actions Semi, Inc");
MODULE_DESCRIPTION("ATC260X DCDC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:atc260x-buck");
MODULE_ALIAS("platform:atc260x-boost");

