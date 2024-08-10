/*
 * atc260x-hwmon.c  --  hardware monitoring for ATC260X
 *
 * Copyright 2011 Actions Semi Inc.
 * Author: Actions Semi, Inc.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/slab.h>

//#include <asm/mach-actions/atc260x.h>
//#include <asm/mach-actions/atc260x_pdata.h>
#include <mach/atc260x/atc260x.h>
#include <mach/atc260x/atc260x_pdata.h>

struct atc260x_hwmon {
    struct atc260x_dev *atc260x;
    struct device *classdev;
};

static ssize_t show_name(struct device *dev,
             struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "atc260x\n");
}

static ssize_t show_value(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct atc260x_hwmon *hwmon = dev_get_drvdata(dev);
    int channel = to_sensor_dev_attr(attr)->index;
    int ret;

    printk("%s_%d: channel %d\n", __FUNCTION__, __LINE__, channel);
    
    ret = atc260x_auxadc_read(hwmon->atc260x, channel);
    if (ret < 0)
        return ret;

    return sprintf(buf, "%d\n", ret);
}

static DEVICE_ATTR(name, S_IRUGO, show_name, NULL);

static SENSOR_DEVICE_ATTR(wall_voltage, S_IRUGO, show_value, NULL, ATC260X_AUX_WALLV);
static SENSOR_DEVICE_ATTR(vbus_voltage, S_IRUGO, show_value, NULL, ATC260X_AUX_VBUSV);
static SENSOR_DEVICE_ATTR(bat_voltage, S_IRUGO, show_value, NULL, ATC260X_AUX_BATV);
static SENSOR_DEVICE_ATTR(syspower_voltage, S_IRUGO, show_value, NULL, ATC260X_AUX_SYSPWRV);
static SENSOR_DEVICE_ATTR(backupbat_voltage, S_IRUGO, show_value, NULL, ATC260X_AUX_BAKBATV);

static SENSOR_DEVICE_ATTR(wall_current, S_IRUGO, show_value, NULL, ATC260X_AUX_WALLI);
static SENSOR_DEVICE_ATTR(vbus_current, S_IRUGO, show_value, NULL, ATC260X_AUX_VBUSI);
static SENSOR_DEVICE_ATTR(bat_current, S_IRUGO, show_value, NULL, ATC260X_AUX_BATI);
static SENSOR_DEVICE_ATTR(charge_current, S_IRUGO, show_value, NULL, ATC260X_AUX_CHGI);

static SENSOR_DEVICE_ATTR(current_ref, S_IRUGO, show_value, NULL, ATC260X_AUX_IREF);
static SENSOR_DEVICE_ATTR(ic_temperature, S_IRUGO, show_value, NULL, ATC260X_AUX_SYSPWRV);
static SENSOR_DEVICE_ATTR(remote_control, S_IRUGO, show_value, NULL, ATC260X_AUX_REMCON);

static SENSOR_DEVICE_ATTR(aux0, S_IRUGO, show_value, NULL, ATC260X_AUX_AUX0);
static SENSOR_DEVICE_ATTR(aux1, S_IRUGO, show_value, NULL, ATC260X_AUX_AUX1);
static SENSOR_DEVICE_ATTR(aux2, S_IRUGO, show_value, NULL, ATC260X_AUX_AUX2);
static SENSOR_DEVICE_ATTR(aux3, S_IRUGO, show_value, NULL, ATC260X_AUX_AUX3);

static struct attribute *atc260x_attributes[] = {
    &dev_attr_name.attr,
        
    &sensor_dev_attr_wall_voltage.dev_attr.attr,
    &sensor_dev_attr_vbus_voltage.dev_attr.attr,
    &sensor_dev_attr_bat_voltage.dev_attr.attr,
    &sensor_dev_attr_syspower_voltage.dev_attr.attr,
    &sensor_dev_attr_backupbat_voltage.dev_attr.attr,
    
    &sensor_dev_attr_wall_current.dev_attr.attr,
    &sensor_dev_attr_vbus_current.dev_attr.attr,
    &sensor_dev_attr_bat_current.dev_attr.attr,
    &sensor_dev_attr_charge_current.dev_attr.attr,
    
    &sensor_dev_attr_current_ref.dev_attr.attr,
    &sensor_dev_attr_ic_temperature.dev_attr.attr,
    &sensor_dev_attr_remote_control.dev_attr.attr,
    
    &sensor_dev_attr_aux0.dev_attr.attr,
    &sensor_dev_attr_aux1.dev_attr.attr,
    &sensor_dev_attr_aux2.dev_attr.attr,
    &sensor_dev_attr_aux3.dev_attr.attr,    
    NULL
};

static const struct attribute_group atc260x_attr_group = {
    .attrs  = atc260x_attributes,
};

static int __devinit atc260x_hwmon_probe(struct platform_device *pdev)
{
    struct atc260x_dev *atc260x = dev_get_drvdata(pdev->dev.parent);
    struct atc260x_hwmon *hwmon;
    int ret;

    printk("[ATC260X] Probing hwmon\n");

    hwmon = kzalloc(sizeof(struct atc260x_hwmon), GFP_KERNEL);
    if (!hwmon)
        return -ENOMEM;

    hwmon->atc260x = atc260x;

    ret = sysfs_create_group(&pdev->dev.kobj, &atc260x_attr_group);
    if (ret)
        goto err;

    hwmon->classdev = hwmon_device_register(&pdev->dev);
    if (IS_ERR(hwmon->classdev)) {
        ret = PTR_ERR(hwmon->classdev);
        goto err_sysfs;
    }

    platform_set_drvdata(pdev, hwmon);

    return 0;

err_sysfs:
    sysfs_remove_group(&pdev->dev.kobj, &atc260x_attr_group);
err:
    kfree(hwmon);
    return ret;
}

static int __devexit atc260x_hwmon_remove(struct platform_device *pdev)
{
    struct atc260x_hwmon *hwmon = platform_get_drvdata(pdev);

    hwmon_device_unregister(hwmon->classdev);
    sysfs_remove_group(&pdev->dev.kobj, &atc260x_attr_group);
    platform_set_drvdata(pdev, NULL);
    kfree(hwmon);

    return 0;
}

static struct platform_driver atc260x_hwmon_driver = {
    .probe = atc260x_hwmon_probe,
    .remove = __devexit_p(atc260x_hwmon_remove),
    .driver = {
        .name = "atc260x-hwmon",
        .owner = THIS_MODULE,
    },
};

static int __init atc260x_hwmon_init(void)
{
    return platform_driver_register(&atc260x_hwmon_driver);
}
module_init(atc260x_hwmon_init);

static void __exit atc260x_hwmon_exit(void)
{
    platform_driver_unregister(&atc260x_hwmon_driver);
}
module_exit(atc260x_hwmon_exit);

MODULE_AUTHOR("Actions Semi, Inc");
MODULE_DESCRIPTION("ATC260X Hardware Monitoring");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:atc260x-hwmon");

