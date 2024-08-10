/*
 * atc260x_switch_ldo.c  --  Switch LDO driver for ATC260X
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
#include <linux/regulator/machine.h>
#include <linux/slab.h>

#include <mach/atc260x/atc260x.h>
#include <mach/atc260x/atc260x_pdata.h>
#include <mach/atc260x/atc260x_dev.h>

#define ATC260X_SWITCH_LDO_MAX_NAME            12

#define PMU_SWITCH_CTL_SWITCH1_LDO_BIAS         (1 << 0)
#define PMU_SWITCH_CTL_SWITCH1_DISCHARGE_EN     (1 << 1)
#define PMU_SWITCH_CTL_SWITCH1_LDO_VOL(x)       (((x) & 0x3) << 3)
#define PMU_SWITCH_CTL_SWITCH1_LDO_MASK         PMU_SWITCH_CTL_SWITCH1_LDO_VOL(0x3)
#define PMU_SWITCH_CTL_SWITCH1_MODE_MASK        (1 << 5)
#define     PMU_SWITCH_CTL_SWITCH1_MODE_SWITCH      (0 << 5)
#define     PMU_SWITCH_CTL_SWITCH1_MODE_LDO         (1 << 5)
#define PMU_SWITCH_CTL_SWITCH2_LDO_BIAS         (1 << 6)
#define PMU_SWITCH_CTL_SWITCH2_LDO_VOL(x)       (((x) & 0xf) << 8)
#define PMU_SWITCH_CTL_SWITCH2_LDO_MASK         PMU_SWITCH_CTL_SWITCH1_LDO_VOL(0xf)

#define PMU_SWITCH_CTL_SWITCH2_LDO_VOL_SEL_MASK (1 << 12)
#define     PMU_SWITCH_CTL_SWITCH2_LDO_VOL_SEL_10_20V   (0 << 12)
#define     PMU_SWITCH_CTL_SWITCH2_LDO_VOL_SEL_23_33V   (1 << 12)
#define PMU_SWITCH_CTL_SWITCH2_MODE_SWITCH      (0 << 13)
#define PMU_SWITCH_CTL_SWITCH2_MODE_LDO         (1 << 13)
#define PMU_SWITCH_CTL_SWITCH2_EN               (1 << 14)
#define PMU_SWITCH_CTL_SWITCH1_EN               (1 << 15)

static int pmic_ver = 0;
extern int atc260x_get_version();
/* Supported voltage values for regulators (in milliVolts) */
static const unsigned int switch2_ldo_volcfg1_table[] = {
	1000000, 1100000, 1200000, 1300000,
	1400000, 1500000, 1600000, 1700000,
	1750000, 1800000, 1850000, 1900000,
	1950000, 2000000, 2000000, 2000000,
};

static const unsigned int switch2_ldo_volcfg2_table[] = {
	2300000, 2400000, 2500000, 2600000,
	2700000, 2800000, 2900000, 3000000,
	3050000, 3100000, 3150000, 3200000,
	3250000, 3300000, 3300000, 3300000,
};

static const unsigned int switch1_ldo_volcfg_table[] = {
	3000000, 3100000, 3200000, 3300000,
};

struct atc260x_switch_ldo_property
{
    int table_len;
    const unsigned int *table;
    unsigned int min_uv;            /* min voltage (uV) */  
    unsigned int max_uv;            /* step voltage (uV) */    
};


struct atc260x_switch_ldo {
    char name[ATC260X_SWITCH_LDO_MAX_NAME];
    struct regulator_desc desc;
    struct atc260x_dev *atc260x;
    struct regulator_dev *regulator;
    int id;
    struct atc260x_switch_ldo_property property;
};


static int atc260x_switch_ldo_list_voltage(struct regulator_dev *rdev,
                      unsigned int selector)
{
    struct atc260x_switch_ldo *ldo = rdev_get_drvdata(rdev);
    int id = rdev_get_id(rdev);

    if (id > ATC260X_SWITCH_LDO_MAX_NUM) {
        return -EINVAL; 
    }

    if (selector < ldo->property.table_len)
        return ldo->property.table[selector];

    return -EINVAL;
}

static int atc260x_switch_ldo_is_enabled(struct regulator_dev *rdev)
{
    struct atc260x_switch_ldo *ldo = rdev_get_drvdata(rdev);
    struct atc260x_dev *atc260x = ldo->atc260x;
    int ret, mask_bit;

    ret = atc260x_reg_read(atc260x, atc2603_PMU_SWITCH_CTL);
    if (ret < 0)
        return ret;

    if (ldo->id == 0) {
        mask_bit = PMU_SWITCH_CTL_SWITCH1_EN | PMU_SWITCH_CTL_SWITCH1_MODE_LDO;
        ret &= mask_bit;
        if(pmic_ver == 3)
        {
            /*ver D*/
            if(ret != 0)
            {
                return 0;
            }
            else
            {
                return 1;
            }
        }
        else
        {
            /*ver A B C*/
             return (ret == mask_bit);
        }
        
    } else {
        mask_bit = PMU_SWITCH_CTL_SWITCH2_EN | PMU_SWITCH_CTL_SWITCH2_MODE_LDO;
        ret &= mask_bit;
        return (ret == mask_bit);
    }
}


static int atc260x_switch_ldo_enable(struct regulator_dev *rdev)
{
    struct atc260x_switch_ldo *ldo = rdev_get_drvdata(rdev);
    struct atc260x_dev *atc260x = ldo->atc260x;
    int ret;

    if (ldo->id == 0) {
        
        if(pmic_ver == 3)
        {
            /*ver D, enable is 0, disable is 1*/
            ret = atc260x_set_bits(atc260x, atc2603_PMU_SWITCH_CTL, PMU_SWITCH_CTL_SWITCH1_EN, 0);
        }
        else
        {
            ret = atc260x_set_bits(atc260x, atc2603_PMU_SWITCH_CTL, PMU_SWITCH_CTL_SWITCH1_EN, PMU_SWITCH_CTL_SWITCH1_EN);
        }
        
        
    } else {
        ret = atc260x_set_bits(atc260x, atc2603_PMU_SWITCH_CTL, 
            PMU_SWITCH_CTL_SWITCH2_EN, PMU_SWITCH_CTL_SWITCH2_EN);
    }

    return ret;
}

static int atc260x_switch_ldo_disable(struct regulator_dev *rdev)
{
    struct atc260x_switch_ldo *ldo = rdev_get_drvdata(rdev);
    struct atc260x_dev *atc260x = ldo->atc260x;
    int ret;

    if (ldo->id == 0) {
        if(pmic_ver == 3)
        {
            /*ver D, enable is 0, disable is 1*/
            ret = atc260x_set_bits(atc260x, atc2603_PMU_SWITCH_CTL, 
                PMU_SWITCH_CTL_SWITCH1_EN, PMU_SWITCH_CTL_SWITCH1_EN);
        }
        else
        {
            ret = atc260x_set_bits(atc260x, atc2603_PMU_SWITCH_CTL, 
                PMU_SWITCH_CTL_SWITCH1_EN, 0);
        }
    } else {
        ret = atc260x_set_bits(atc260x, atc2603_PMU_SWITCH_CTL, 
            PMU_SWITCH_CTL_SWITCH2_EN, 0);
    }

    return ret;
}

static int atc260x_switch_ldo_set_voltage(struct regulator_dev *rdev,
                     int min_uV, int max_uV)
{
    struct atc260x_switch_ldo *ldo = rdev_get_drvdata(rdev);
    struct atc260x_dev *atc260x = ldo->atc260x;
    int ret, vsel;

	if (min_uV < ldo->property.min_uv || min_uV > ldo->property.max_uv)
		return -EINVAL;
	if (max_uV < ldo->property.min_uv || max_uV > ldo->property.max_uv)
		return -EINVAL;

	for (vsel = 0; vsel < ldo->property.table_len; vsel++) {
		int uV = ldo->property.table[vsel];

		/* Break at the first in-range value */
		if (min_uV <= uV && uV <= max_uV)
			break;
	}

	if (vsel == ldo->property.table_len)
		return -EINVAL;

    if (ldo->id == 0) {
        ret = atc260x_set_bits(atc260x, atc2603_PMU_SWITCH_CTL, 
            PMU_SWITCH_CTL_SWITCH1_LDO_MASK, PMU_SWITCH_CTL_SWITCH1_LDO_VOL(vsel));
    } else {
        ret = atc260x_set_bits(atc260x, atc2603_PMU_SWITCH_CTL, 
            PMU_SWITCH_CTL_SWITCH2_LDO_MASK, PMU_SWITCH_CTL_SWITCH2_LDO_VOL(vsel));
    }

    return ret;
}

static int atc260x_switch_ldo_get_voltage(struct regulator_dev *rdev)
{
    struct atc260x_switch_ldo *ldo = rdev_get_drvdata(rdev);
    struct atc260x_dev *atc260x = ldo->atc260x;
    int ret, vsel;

    ret = atc260x_reg_read(atc260x, atc2603_PMU_SWITCH_CTL);
    if (ret < 0)
        return ret;

    if (ldo->id == 0) {
        vsel = PMU_SWITCH_CTL_SWITCH1_LDO_VOL(ret);
    } else {
        vsel = PMU_SWITCH_CTL_SWITCH2_LDO_VOL(ret);
    }

    ret = ldo->property.table[vsel];

    return ret;
}


static struct regulator_ops atc260x_switch_ldo_ops = {
    .list_voltage   = atc260x_switch_ldo_list_voltage,
    .get_voltage    = atc260x_switch_ldo_get_voltage,
    .set_voltage    = atc260x_switch_ldo_set_voltage,

    .is_enabled     = atc260x_switch_ldo_is_enabled,
    .enable         = atc260x_switch_ldo_enable,
    .disable        = atc260x_switch_ldo_disable,
};

static void set_property(struct atc260x_switch_ldo_property *property, 
        const int *table, int table_len) {
    property->table = table;
    property->table_len = table_len;
    property->min_uv = table[0];
    property->max_uv = table[table_len - 1];
}

static int ldo_init(struct atc260x_switch_ldo *ldo,
        struct regulator_init_data *init_data) 
{
    int id = ldo->id;
    int min_uv =  init_data->constraints.min_uV;
    int max_uv =  init_data->constraints.min_uV;
    int ret = 0, ver=0;

    
    if (id == 0) {
        /* Switch LDO 1, keep enable status */
        atc260x_set_bits(ldo->atc260x, atc2603_PMU_SWITCH_CTL,
            PMU_SWITCH_CTL_SWITCH1_DISCHARGE_EN,
            0);

        if(pmic_ver == 3)
        {
            /*
                ver D, 0-- LDO, 1-- switch
            */
            atc260x_set_bits(ldo->atc260x, atc2603_PMU_SWITCH_CTL,
                PMU_SWITCH_CTL_SWITCH1_MODE_MASK,
                0);
        }
        else
        {
            atc260x_set_bits(ldo->atc260x, atc2603_PMU_SWITCH_CTL,
                PMU_SWITCH_CTL_SWITCH1_MODE_MASK,
                PMU_SWITCH_CTL_SWITCH1_MODE_LDO);
        }
        set_property(&ldo->property, (const int *)&switch1_ldo_volcfg_table, 
            ARRAY_SIZE(switch1_ldo_volcfg_table));
    } else if (id == 1) {
        /* Switch LDO 2 */
        int table1_len = ARRAY_SIZE(switch2_ldo_volcfg1_table);
        int table2_len = ARRAY_SIZE(switch2_ldo_volcfg2_table);

        atc260x_set_bits(ldo->atc260x, atc2603_PMU_SWITCH_CTL,
            PMU_SWITCH_CTL_SWITCH2_EN,
            0);

        if ((switch2_ldo_volcfg1_table[0] <= min_uv) && 
            (switch2_ldo_volcfg1_table[table1_len - 1] >= max_uv)) {

            set_property(&ldo->property, (const int *)&switch2_ldo_volcfg1_table, 
                table1_len);

            atc260x_set_bits(ldo->atc260x, atc2603_PMU_SWITCH_CTL,
                PMU_SWITCH_CTL_SWITCH2_LDO_VOL_SEL_MASK,
                PMU_SWITCH_CTL_SWITCH2_LDO_VOL_SEL_10_20V);
        } else if ((switch2_ldo_volcfg2_table[0] <= min_uv) && 
            (switch2_ldo_volcfg2_table[table2_len - 1] >= max_uv)) {

            set_property(&ldo->property, (const int *)&switch2_ldo_volcfg1_table, 
                table1_len);

            atc260x_set_bits(ldo->atc260x, atc2603_PMU_SWITCH_CTL,
                PMU_SWITCH_CTL_SWITCH2_LDO_VOL_SEL_MASK,
                PMU_SWITCH_CTL_SWITCH2_LDO_VOL_SEL_23_33V);
        } else {
            ret = -EINVAL; 
        }
    } else {
        ret = -EINVAL;
    }

    return ret;
}

static __devinit int atc260x_switch_ldo_probe(struct platform_device *pdev)
{
    struct atc260x_dev *atc260x = dev_get_drvdata(pdev->dev.parent);
    struct atc260x_pdata *pdata = atc260x->dev->platform_data;
    struct atc260x_switch_ldo *ldo;
    int id = pdev->id % ARRAY_SIZE(pdata->switch_ldo);
    int ret;

    printk("[ATC260X] Probing Switch/LDO%d\n", id + 1);

    if (pdata == NULL || pdata->switch_ldo[id] == NULL)
        return -ENODEV;

    if (id > ATC260X_SWITCH_LDO_MAX_NUM) {
        return -EINVAL; 
    }

    ldo = kzalloc(sizeof(struct atc260x_switch_ldo), GFP_KERNEL);
    if (ldo == NULL) {
        return -ENOMEM;
    }
    
    ldo->id = id;
    ldo->atc260x = atc260x;

    pmic_ver = atc260x_get_version();
    if(pmic_ver<0)
    {
        printk("\n %s atc260x_get_version failed\n",__FUNCTION__);
        return pmic_ver;
    }
    
    ret = ldo_init(ldo, pdata->switch_ldo[id]);
    if (ret) {
        pr_err("Faild to init Switch LDO%d", id + 1);
        goto err;
    }

    snprintf(ldo->name, sizeof(ldo->name), "Switch LDO%d", id + 1);
    ldo->desc.name = ldo->name;
    ldo->desc.id = id;
    ldo->desc.type = REGULATOR_VOLTAGE;
    ldo->desc.n_voltages = ldo->property.table_len;
    ldo->desc.ops = &atc260x_switch_ldo_ops;
    ldo->desc.owner = THIS_MODULE;

    ldo->regulator = regulator_register(&ldo->desc, &pdev->dev,
                         pdata->switch_ldo[id], ldo, NULL);
    if (IS_ERR(ldo->regulator)) {
        ret = PTR_ERR(ldo->regulator);
        pr_err("Failed to register Switch LDO%d: %d\n",
            id + 1, ret);
        goto err;
    }

    platform_set_drvdata(pdev, ldo);
    return 0;
err:
    kfree(ldo);
    return ret;
}

static __devexit int atc260x_switch_ldo_remove(struct platform_device *pdev)
{
    struct atc260x_switch_ldo *ldo = platform_get_drvdata(pdev);

    platform_set_drvdata(pdev, NULL);

    regulator_unregister(ldo->regulator);
    kfree(ldo);

    return 0;
}

static struct platform_driver atc260x_switch_ldo_driver = {
    .probe = atc260x_switch_ldo_probe,
    .remove = __devexit_p(atc260x_switch_ldo_remove),
    .driver     = {
        .name   = "atc260x-switch-ldo",
        .owner  = THIS_MODULE,
    },
};

static int __init atc260x_switch_ldo_init(void)
{
    int ret;

    ret = platform_driver_register(&atc260x_switch_ldo_driver);
    if (ret != 0)
        pr_err("Failed to register ATC260X Switch LDO driver: %d\n", ret);

    return 0;
}
subsys_initcall(atc260x_switch_ldo_init);

static void __exit atc260x_switch_ldo_exit(void)
{
    platform_driver_unregister(&atc260x_switch_ldo_driver);
}
module_exit(atc260x_switch_ldo_exit);

/* Module information */
MODULE_AUTHOR("Actions Semi, Inc");
MODULE_DESCRIPTION("ATC260X Switch LDO driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:atc260x-switch-ldo");
