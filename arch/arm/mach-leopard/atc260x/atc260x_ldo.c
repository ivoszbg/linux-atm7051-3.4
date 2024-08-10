/*
 * atc260x_ldo.c  --  LDO driver for ATC260X
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

#include <mach/atc260x/atc260x.h>
#include <mach/atc260x/atc260x_pdata.h>
#include <mach/atc260x/atc260x_dev.h>
#include <linux/delay.h>

#define ATC260X_LDO_ENABLE              (1 << 0) /* bit 0 */

#define ATC260X_LDO_MAX_NAME            8

struct atc260x_ldo_property
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

struct atc260x_ldo {
    char name[ATC260X_LDO_MAX_NAME];
    struct regulator_desc desc;
    struct atc260x_dev *atc260x;
    struct regulator_dev *regulator;
    struct atc260x_ldo_property *property;
};

#define GET_SELECTOR(property, vsel)        (((vsel) >> (property)->step_bits_shift) & (property)->step_bits_mask)
#define SET_SELECTOR(property, vsel)        (((vsel) & (property)->step_bits_mask) << (property)->step_bits_shift)

/* LDO property options */
#define LDO_OPT_ALWAYS_ON               (1 << 0)        /* LDO is always on, cannot control by software */

static struct atc260x_ldo_property ldos_property[ATC260X_LDO_MAX_NUM] = {

    /* ctl_reg              min_uv      step_uv   step_num  step_shift  bias_shift  opt */
    
    /* LDO 1, 2.6~3.3v */
    {atc2603_PMU_LDO1_CTL,   2600000,    100000,     8,      0x7,    13,     11,     LDO_OPT_ALWAYS_ON},

    /* LDO 2, 2.6~3.3v */
    {atc2603_PMU_LDO2_CTL,   2600000,    100000,     8,      0x7,    13,     11,     LDO_OPT_ALWAYS_ON}, 

    /* LDO 3, 1.5~2.0v */
    {atc2603_PMU_LDO3_CTL,   1500000,    100000,     6,      0x7,    13,     11,     LDO_OPT_ALWAYS_ON}, 

    /* LDO 4, 2.8~3.5v */
    {atc2603_PMU_LDO4_CTL,   2800000,    100000,     8,      0x7,    13,     11,     0}, 

    /* LDO 5, 2.6~3.3v */
    {atc2603_PMU_LDO5_CTL,   2600000,    100000,     8,      0x7,    13,     11,     0}, 

    /* LDO 6, 0.7~1.4v */
    {atc2603_PMU_LDO6_CTL,   700000,     25000,      29,     0x1f,   11,     9,      LDO_OPT_ALWAYS_ON}, 

    /* LDO 7, 1.5~2.0v */
    {atc2603_PMU_LDO7_CTL,   1500000,    100000,     6,      0x7,    13,     11,     LDO_OPT_ALWAYS_ON}, 
            
    /* LDO 8, 2.3~3.3v */
    {atc2603_PMU_LDO8_CTL,   2300000,    100000,     11,     0xf,    12,     10,     0}, 
    
    /* LDO 9, 1.0~1.5v */
    {atc2603_PMU_LDO9_CTL,   1000000,    100000,     6,      0x7,    13,     11,     0}, 
    
    /* LDO 10, 2.3~3.3v */
    {atc2603_PMU_LDO10_CTL,  2300000,    100000,     11,     0xf,    12,     10,     0}, 
    
    /* LDO 11, 2.6~3.3v */
    {atc2603_PMU_LDO11_CTL,  2600000,    100000,     8,      0x7,    13,     0,      LDO_OPT_ALWAYS_ON},     /* no bias bit */

};


static int atc260x_ldo_list_voltage(struct regulator_dev *rdev,
                      unsigned int selector)
{
    struct atc260x_ldo *ldo = rdev_get_drvdata(rdev);
    int id = rdev_get_id(rdev);

    if (id > ATC260X_LDO_MAX_NUM) {
        return -EINVAL; 
    }

    if (selector < ldo->property->step_num)
        return ldo->property->min_uv + (selector * ldo->property->step_uv);

    return -EINVAL;
}

static int atc260x_ldo_is_enabled(struct regulator_dev *rdev)
{
    struct atc260x_ldo *ldo = rdev_get_drvdata(rdev);
    struct atc260x_dev *atc260x = ldo->atc260x;
    int ret;

    if (ldo->property->opt & LDO_OPT_ALWAYS_ON)
        return 1;

    ret = atc260x_reg_read(atc260x, ldo->property->ctl_reg);
    if (ret < 0)
        return ret;

    if (ret & ATC260X_LDO_ENABLE)
        return 1;
    else
        return 0;
}

static int atc260x_ldo_enable(struct regulator_dev *rdev)
{
    struct atc260x_ldo *ldo = rdev_get_drvdata(rdev);
    struct atc260x_dev *atc260x = ldo->atc260x;

    if (ldo->property->opt & LDO_OPT_ALWAYS_ON)
        return 0;

    return atc260x_set_bits(atc260x, ldo->property->ctl_reg, 
        ATC260X_LDO_ENABLE, ATC260X_LDO_ENABLE);
}

static int atc260x_ldo_disable(struct regulator_dev *rdev)
{
    struct atc260x_ldo *ldo = rdev_get_drvdata(rdev);
    struct atc260x_dev *atc260x = ldo->atc260x;

    if (ldo->property->opt & LDO_OPT_ALWAYS_ON)
        return 0;

    return atc260x_set_bits(atc260x, ldo->property->ctl_reg, 
        ATC260X_LDO_ENABLE, 0);
}

static int atc260x_ldo_set_voltage(struct regulator_dev *rdev,
                     int min_uV, int max_uV, unsigned* selector)
{
    struct atc260x_ldo *ldo = rdev_get_drvdata(rdev);
    struct atc260x_dev *atc260x = ldo->atc260x;
    int ret, vsel;

    vsel = (min_uV - ldo->property->min_uv) / ldo->property->step_uv;
        
    ret = atc260x_ldo_list_voltage(rdev, vsel);
    if (ret < 0)
        return ret;
    if (ret < min_uV || ret > max_uV)
        return -EINVAL;
    
    *selector = vsel; /*add by jlingzhang*/
    ret = atc260x_set_bits(atc260x, ldo->property->ctl_reg, 
        SET_SELECTOR(ldo->property, ldo->property->step_bits_mask), 
        SET_SELECTOR(ldo->property, vsel));
	mdelay(1);
	return ret;
}

static int atc260x_ldo_get_voltage(struct regulator_dev *rdev)
{
    struct atc260x_ldo *ldo = rdev_get_drvdata(rdev);
    struct atc260x_dev *atc260x = ldo->atc260x;
    int ret;

    ret = atc260x_reg_read(atc260x, ldo->property->ctl_reg);
    if (ret < 0)
        return ret;

    ret = GET_SELECTOR(ldo->property, ret);

    return atc260x_ldo_list_voltage(rdev, ret);;
}


static struct regulator_ops atc260x_ldo_ops = {
    .list_voltage   = atc260x_ldo_list_voltage,
    .get_voltage    = atc260x_ldo_get_voltage,
    .set_voltage    = atc260x_ldo_set_voltage,

    .is_enabled     = atc260x_ldo_is_enabled,
    .enable         = atc260x_ldo_enable,
    .disable        = atc260x_ldo_disable,
};


static __devinit int atc260x_ldo_probe(struct platform_device *pdev)
{
    struct atc260x_dev *atc260x = dev_get_drvdata(pdev->dev.parent);
    struct atc260x_pdata *pdata = atc260x->dev->platform_data;
    int id = pdev->id % ARRAY_SIZE(pdata->ldo);
    struct atc260x_ldo *ldo;
    int ret;

    printk("[ATC260X] Probing LDO%d\n", id + 1);

    dev_dbg(&pdev->dev, "Probing LDO%d\n", id + 1);

    if (pdata == NULL || pdata->ldo[id] == NULL)
        return -ENODEV;

    if (id > ATC260X_LDO_MAX_NUM) {
        return -EINVAL; 
    }

    ldo = kzalloc(sizeof(struct atc260x_ldo), GFP_KERNEL);
    if (ldo == NULL) {
        dev_err(&pdev->dev, "Unable to allocate private data\n");
        return -ENOMEM;
    }

    ldo->atc260x = atc260x;
    ldo->property = &ldos_property[id];

    snprintf(ldo->name, sizeof(ldo->name), "LDO%d", id + 1);
    ldo->desc.name = ldo->name;
    ldo->desc.id = id;
    ldo->desc.type = REGULATOR_VOLTAGE;
    ldo->desc.n_voltages = ldo->property->step_num;
    ldo->desc.ops = &atc260x_ldo_ops;
    ldo->desc.owner = THIS_MODULE;


    ldo->regulator = regulator_register(&ldo->desc, &pdev->dev,
                         pdata->ldo[id], ldo, NULL);
    if (IS_ERR(ldo->regulator)) {
        ret = PTR_ERR(ldo->regulator);
        dev_err(atc260x->dev, "Failed to register LDO%d: %d\n",
            id + 1, ret);
        goto err;
    }

    platform_set_drvdata(pdev, ldo);

    return 0;

//err_regulator:
//  regulator_unregister(ldo->regulator);
err:
    kfree(ldo);
    return ret;
}

static __devexit int atc260x_ldo_remove(struct platform_device *pdev)
{
    struct atc260x_ldo *ldo = platform_get_drvdata(pdev);

    platform_set_drvdata(pdev, NULL);

    regulator_unregister(ldo->regulator);
    kfree(ldo);

    return 0;
}


static struct platform_driver atc260x_ldo_driver = {
    .probe = atc260x_ldo_probe,
    .remove = __devexit_p(atc260x_ldo_remove),
    .driver     = {
        .name   = "atc260x-ldo",
        .owner  = THIS_MODULE,
    },
};

static int __init atc260x_ldo_init(void)
{
    int ret;

    ret = platform_driver_register(&atc260x_ldo_driver);
    if (ret != 0)
        pr_err("Failed to register ATC260X LDO driver: %d\n", ret);


    return 0;
}
subsys_initcall(atc260x_ldo_init);

static void __exit atc260x_ldo_exit(void)
{
    platform_driver_unregister(&atc260x_ldo_driver);
}
module_exit(atc260x_ldo_exit);

/* Module information */
MODULE_AUTHOR("Actions Semi, Inc");
MODULE_DESCRIPTION("ATC260X LDO driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:atc260x-ldo");
