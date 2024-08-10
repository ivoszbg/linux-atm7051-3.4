/*
 * Leopard thermal driver.
 *
 * Copyright (C) 2011-2012 ST Microelectronics
 * Author: Vincenzo Frascino <vincenzo.frascino@st.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/cpufreq.h>

#include <linux/of.h>
#include <linux/reboot.h>
#include <linux/suspend.h>
#include <mach/leopard_thermal.h>
#include <mach/atc260x/actions_reg_atc2603.h>

#define MAX_COOLING_DEVICE 4

#define PASSIVE_INTERVAL 5000
#define IDLE_INTERVAL 10000

/* CPU Zone information */
#define PANIC_ZONE      4
#define WARN_ZONE       3
#define MONITOR_ZONE    2
#define SAFE_ZONE       1

#define GET_ZONE(trip) (trip + 2)
#define GET_TRIP(zone) (zone - 2)

#define LEOPARD_ZONE_COUNT	3

/* Leopard Thermal Sensor Dev Structure */
struct leopard_thermal_dev {
	/* pointer to thermal flags */
	unsigned int flags;
};

extern int (* boot_info_read)(unsigned short reg);
static void leopard_unregister_thermal(void);

struct leopard_thermal_zone {
//	enum thermal_device_mode mode;
	struct thermal_zone_device *therm_dev;
	struct thermal_cooling_device *cool_dev[MAX_COOLING_DEVICE];
	unsigned int cool_dev_size;
	struct platform_device *leopard_dev;
};

static struct leopard_thermal_zone *th_zone;

unsigned int dbg_temp;

static inline int leopard_get_temp(struct thermal_zone_device *thermal,
				unsigned long *temp)
{
	unsigned int val;
	
	/*IC TEMP=0.1949*Register DATA[9：0]-44.899*/
	val = ((boot_info_read(atc2603_PMU_ICTEMPADC) & 0x3ff)*1949 - 448990)/10000;
	*temp = val;

	if(dbg_temp)
	{
		*temp  = dbg_temp;
	}

	printk(KERN_DEBUG "%s,val:%d, temp:%d\n", __FUNCTION__, val, *temp);
	return 0;
}

/* Get trip type callback functions for thermal zone */
static int leopard_get_trip_type(struct thermal_zone_device *thermal, int trip,
				 enum thermal_trip_type *type)
{
	switch (GET_ZONE(trip)) {
	case MONITOR_ZONE:
		*type = THERMAL_TRIP_ACTIVE;
		break;
	case WARN_ZONE:
		*type = THERMAL_TRIP_PASSIVE;
		break;
	case PANIC_ZONE:
		*type = THERMAL_TRIP_CRITICAL;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/* Get trip temperature callback functions for thermal zone */
static int leopard_get_trip_temp(struct thermal_zone_device *thermal, int trip,
				unsigned long *temp)
{
	struct leopard_tmu_platform_data *pdata = th_zone->leopard_dev->dev.platform_data;
	
	if (trip < GET_TRIP(MONITOR_ZONE) || trip > GET_TRIP(PANIC_ZONE))
		return -EINVAL;

	*temp = pdata->trigger_levels[trip];

	return 0;
}

static int leopard_set_trip_temp(struct thermal_zone_device *thermal, int trip,
				unsigned long temp)
{
	struct leopard_tmu_platform_data *pdata = th_zone->leopard_dev->dev.platform_data;

	if (trip < GET_TRIP(MONITOR_ZONE) || trip > GET_TRIP(PANIC_ZONE))
		return -EINVAL;

	pdata->trigger_levels[trip] = temp;

	return 0;
}


/* Bind callback functions for thermal zone */
static int leopard_bind(struct thermal_zone_device *thermal,
			struct thermal_cooling_device *cdev)
{
	int ret = 0, i;

	/* find the cooling device registered*/
	for (i = 0; i < th_zone->cool_dev_size; i++)
		if (cdev == th_zone->cool_dev[i])
			break;

	/*No matching cooling device*/
	if (i == th_zone->cool_dev_size)
		return 0;

	switch (GET_ZONE(i)) {
	case MONITOR_ZONE:
	case WARN_ZONE:
	case PANIC_ZONE:
		if (thermal_zone_bind_cooling_device(thermal, i, cdev)) {
			pr_err("error binding cooling dev inst 0\n");
			ret = -EINVAL;
		}
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

/* Unbind callback functions for thermal zone */
static int leopard_unbind(struct thermal_zone_device *thermal,
			struct thermal_cooling_device *cdev)
{
	int ret = 0, i;

	/* find the cooling device registered*/
	for (i = 0; i < th_zone->cool_dev_size; i++)
		if (cdev == th_zone->cool_dev[i])
			break;

	/*No matching cooling device*/
	if (i == th_zone->cool_dev_size)
		return 0;

	switch (GET_ZONE(i)) {
	case MONITOR_ZONE:
	case WARN_ZONE:
	case PANIC_ZONE:
		if (thermal_zone_unbind_cooling_device(thermal, i, cdev)) {
			pr_err("error unbinding cooling dev\n");
			ret = -EINVAL;
		}
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static struct thermal_zone_device_ops ops = {
	.bind = leopard_bind,
	.unbind = leopard_unbind,
	.get_temp = leopard_get_temp,
	.get_trip_type = leopard_get_trip_type,
	.get_trip_temp = leopard_get_trip_temp,
	.set_trip_temp = leopard_set_trip_temp,
};

#ifdef CONFIG_PM
static int leopard_thermal_suspend(struct device *dev)
{
	printk("%s\n", __FUNCTION__);
	return 0;
}

static int leopard_thermal_resume(struct device *dev)
{
	printk("%s\n", __FUNCTION__);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(leopard_thermal_pm_ops, leopard_thermal_suspend,
		leopard_thermal_resume);

int set_freq_clip_max(void)
{
	int i;
	struct leopard_tmu_platform_data *pdata = th_zone->leopard_dev->dev.platform_data;
	
	for (i = 0; i < pdata->freq_tab_count; i++) {
		pdata->freq_tab[i].freq_clip_max = get_voltage_best_freq(pdata->freq_tab[i].vdd_clip_max);
	}
	return 0;
}

static int leopard_thermal_probe(struct platform_device *pdev)
{
	int ret = 0, i;
	int count, tab_size, pos = 0;
	struct freq_clip_table *tab_ptr, *clip_data;
	struct leopard_tmu_platform_data *pdata = pdev->dev.platform_data;
	
	printk("%s\n", __FUNCTION__);

	th_zone = kzalloc(sizeof(struct leopard_thermal_zone), GFP_KERNEL);
	if (!th_zone)
		return -ENOMEM;
		
	th_zone->leopard_dev = pdev;
	for (i = 0; i < pdata->freq_tab_count; i++) {
		pdata->freq_tab[i].mask_val = cpu_all_mask;
//		pdata->freq_tab[i].freq_clip_max = get_best_freq(pdata->freq_tab[i].vdd_clip_max);
	}
	
	/* Register the cpufreq cooling device */
	tab_ptr = (struct freq_clip_table *)pdata->freq_tab;
	for (count = 0; count < LEOPARD_ZONE_COUNT; count++) {
		tab_size = pdata->size[count];
		if (tab_size == 0)
			continue;

		clip_data = (struct freq_clip_table *)&(tab_ptr[pos]);

		th_zone->cool_dev[count] = cpufreq_cooling_register(
						clip_data, tab_size);
		pos += tab_size;

		if (IS_ERR(th_zone->cool_dev[count])) {
			pr_err("Failed to register cpufreq cooling device\n");
			ret = -EINVAL;
			th_zone->cool_dev_size = count;
			goto err_unregister;
		}
	}
	th_zone->cool_dev_size = count;
	
#ifdef ENABLE_PASSIVE	
	th_zone->therm_dev = thermal_zone_device_register("leopard_thermal", LEOPARD_ZONE_COUNT,
				NULL, &ops, 1, 1, PASSIVE_INTERVAL, IDLE_INTERVAL);
#else
//by defining tc1 & tc2 as zero, cancel THERMAL_TRIP_PASSIVE
	th_zone->therm_dev = thermal_zone_device_register("leopard_thermal", LEOPARD_ZONE_COUNT,
				NULL, &ops, 0, 0, PASSIVE_INTERVAL, IDLE_INTERVAL);	
#endif				
	if (IS_ERR(th_zone->therm_dev)) {
		dev_err(&pdev->dev, "thermal zone device is NULL\n");
		ret = PTR_ERR(th_zone->therm_dev);
		goto err_unregister;
	}
	platform_set_drvdata(pdev, th_zone);
	pr_info("leopard: Kernel Thermal management registered\n");
	return 0;

err_unregister:
	leopard_unregister_thermal();
	return ret;
}

/* Un-Register with the in-kernel thermal management */
static void leopard_unregister_thermal(void)
{
	int i;

	for (i = 0; i < th_zone->cool_dev_size; i++) {
		if (th_zone && th_zone->cool_dev[i])
			cpufreq_cooling_unregister(th_zone->cool_dev[i]);
	}

	if (th_zone && th_zone->therm_dev)
		thermal_zone_device_unregister(th_zone->therm_dev);

	kfree(th_zone);

	pr_info("leopard: Kernel Thermal management unregistered\n");
}

static int leopard_thermal_exit(struct platform_device *pdev)
{
	leopard_unregister_thermal();
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver leopard_thermal_driver = {

	.driver = {
		.name = "leopard-tmu",
		.owner = THIS_MODULE,
		.pm = &leopard_thermal_pm_ops,
	},
	.probe = leopard_thermal_probe,
	.remove = leopard_thermal_exit,
};

module_platform_driver(leopard_thermal_driver);

MODULE_AUTHOR("Vincenzo Frascino <vincenzo.frascino@st.com>");
MODULE_DESCRIPTION("Leopard thermal driver");
MODULE_LICENSE("GPL");
