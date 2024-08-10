/* linux/arch/arm/mach-leopard/cpuidle.c
 *
 * Copyright (c) 2012 actions Electronics Co., Ltd.
 *		http://www.actions-semi.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cpuidle.h>
#include <linux/io.h>
#include <linux/suspend.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/irqflags.h>

#include <asm/cacheflush.h>
#include <asm/system.h>

static DEFINE_PER_CPU(struct cpuidle_device, leopard_cpuidle_device);

static int leopard_enter_idle(struct cpuidle_device *dev,
				struct cpuidle_driver *drv,
				int index);
static int leopard_enter_lowpower(struct cpuidle_device *dev,  struct cpuidle_state *state);

static struct cpuidle_state leopard_cpuidle_set[] = {
	[0] = {
		.enter			= leopard_enter_idle,
		.exit_latency		= 1,
		.target_residency	= 10000,
		.flags			= CPUIDLE_FLAG_TIME_VALID,
		.name			= "IDLE",
		.desc			= "ARM clock gating(WFI)",
	},
#ifdef CONFIG_LEOPARD_LOWPWR_IDLE
	[1] = {
		.enter			= leopard_enter_lowpower,
		.exit_latency		= 300,
		.target_residency	= 10000,
		.flags			= CPUIDLE_FLAG_TIME_VALID,
		.name			= "LOW_POWER",
		.desc			= "ARM power down",
	},
#endif
};

static DEFINE_PER_CPU(struct cpuidle_device, leopard_cpuidle_device);

static struct cpuidle_driver leopard_cpu_idle_driver = {
	.name		= "leopard_idle",
	.owner		= THIS_MODULE,
};

static void leopard_do_idle()
{
	extern void leopard_do_wfi();
	//flush_cache_all();/*no need according to arm cortex A5 spec/
	leopard_do_wfi();
}
static int leopard_enter_idle(struct cpuidle_device *dev,
				struct cpuidle_driver *drv,
				int index)
{
	struct timeval before, after;
	int idle_time;

	local_irq_disable();
	do_gettimeofday(&before);

	leopard_do_idle();

	do_gettimeofday(&after);
	local_irq_enable();
	idle_time = (after.tv_sec - before.tv_sec) * USEC_PER_SEC +
		    (after.tv_usec - before.tv_usec);

	dev->last_residency = idle_time;
	return index;
}


static int leopard_enter_lowpower(struct cpuidle_device *dev,  struct cpuidle_state *state)
{
	/*call pm idle?*/
	return 0;
}

static int leopard_cpuidle_notifier_event(struct notifier_block *this,
					  unsigned long event,
					  void *ptr)
{
	switch (event) {
	case PM_SUSPEND_PREPARE:
		disable_hlt();
		pr_debug("PM_SUSPEND_PREPARE for CPUIDLE\n");
		return NOTIFY_OK;
	case PM_POST_RESTORE:
	case PM_POST_SUSPEND:
		enable_hlt();
		pr_debug("PM_POST_SUSPEND for CPUIDLE\n");
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static struct notifier_block leopard_cpuidle_notifier = {
	.notifier_call = leopard_cpuidle_notifier_event,
};

static int __init leopard_init_cpuidle(void)
{
	int i, max_cpuidle_state, cpu_id, ret;
	struct cpuidle_device *device;
	struct cpuidle_driver *drv = &leopard_cpu_idle_driver;

	/* Setup cpuidle driver */
	drv->state_count = (sizeof(leopard_cpuidle_set) /sizeof(struct cpuidle_state));
	max_cpuidle_state = drv->state_count;
	for (i = 0; i < max_cpuidle_state; i++) {
		memcpy(&drv->states[i], &leopard_cpuidle_set[i],
				sizeof(struct cpuidle_state));
	}
	drv->safe_state_index = 0;
	printk("cpu idle drv->state_count %d, cpuidle_register_driver: 0x%08x\n",
	 drv->state_count, (u32)cpuidle_register_driver);
	ret = cpuidle_register_driver(&leopard_cpu_idle_driver);
	if(ret < 0){
		printk(KERN_ERR "cpu idle register driver failed\n");
		return ret;
	}
	/* Setup cpuidle device */
	for_each_cpu(cpu_id, cpu_online_mask) {
		device = &per_cpu(leopard_cpuidle_device, cpu_id);
		device->cpu = cpu_id;

		if (cpu_id == 0)
			device->state_count = ARRAY_SIZE(leopard_cpuidle_set);
		else
			device->state_count = 1;	/* Support IDLE only */

		if (cpuidle_register_device(device)) {
			cpuidle_unregister_driver(&leopard_cpu_idle_driver);
			printk(KERN_ERR "CPUidle register device failed\n,");
			return -EIO;
		}
	}
	register_pm_notifier(&leopard_cpuidle_notifier);
	
	return 0;
}
device_initcall(leopard_init_cpuidle);
