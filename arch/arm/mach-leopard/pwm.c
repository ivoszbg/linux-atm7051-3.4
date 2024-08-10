/* arch/arm/plat-s3c24xx/pwm.c
 *
 * Copyright (c) 2007 Ben Dooks
 * Copyright (c) 2008 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>, <ben-linux@fluff.org>
 *
 * S3C24XX PWM device core
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/pwm.h>
#include <linux/cpufreq.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <mach/hardware.h>
#include <mach/clock.h>

struct pwm_device
{
		struct list_head list;
		struct platform_device *pdev;
		
		unsigned int clk_div;
		unsigned int clk_source;
		unsigned int required_period_ns;
		unsigned int period_ns;
		unsigned int duty_ns;
		unsigned int polarity;
		
		unsigned char running;
		unsigned char use_count;
		unsigned char pwm_id;
		const char *label;
		
		struct clk *clk;
		int clk_status;
		
		struct notifier_block cpufreq_notif;
};

#define pwm_dbg(_pwm, msg...)   dev_dbg(&(_pwm)->pdev->dev, msg)

#define CLK_SOURCE_LOSC_32K     0
#define CLK_SOURCE_HOSC_24M     1


#define CMU_HOSC_FREQ                   25165824
#define CMU_LOSC_FREQ                   32768
#define MAX_PWM_CLK_DIV                 1024
static DEFINE_MUTEX(pwm_lock);
static LIST_HEAD(pwm_list);

/*
static void pwm_print_dbg(void)
{
	printk(KERN_INFO "MFP_CTL1 = 0x%x\n", act_readl(MFP_CTL1));
	printk(KERN_INFO "CMU_PWM2CLK = 0x%x\n", act_readl(CMU_PWM2CLK));
	printk(KERN_INFO "PWM2_CTL = 0x%x\n", act_readl(PWM2_CTL));
	printk(KERN_INFO "CMU_DEVCLKEN1_PWM2 = 0x%x\n",
		act_readl(CMU_DEVCLKEN1));
	printk(KERN_INFO "GPIO_BINEN = 0x%x\n", act_readl(GPIO_BINEN));
	printk(KERN_INFO "GPIO_BOUTEN = 0x%x\n", act_readl(GPIO_BOUTEN));
	printk(KERN_INFO "GPIO_BDAT = 0x%x\n", act_readl(GPIO_BDAT));
}*/

struct pwm_device *pwm_request(int pwm_id, const char *label)
{
	struct pwm_device *pwm;
	int retv, found = 0;
	
	//printk(KERN_INFO "pwm %d request\n", pwm_id);
	
	mutex_lock(&pwm_lock);
	list_for_each_entry(pwm, &pwm_list, list)
	{
		if (pwm->pwm_id == pwm_id)
		{
			found = 1;
			break;
		}
	}
	
	if (found)
	{
		if (pwm->use_count == 0)
		{
				pwm->use_count = 1;
				pwm->label = label;
		}
		else
		{
			pwm = ERR_PTR(-EBUSY);
		}
	}
	else
	{
		pwm = ERR_PTR(-ENOENT);
	}
	mutex_unlock(&pwm_lock);
	return pwm;
}
EXPORT_SYMBOL(pwm_request);

void pwm_free(struct pwm_device *pwm)
{
	pwm->label = NULL;
		
}
EXPORT_SYMBOL(pwm_free);

int pwm_enable(struct pwm_device *pwm)
{
//	u32 pwm_id = (u32) pwm->pwm_id;
	
	//act_setl(((1 << 23) << pwm_id), CMU_DEVCLKEN1);	
	if(pwm->clk_status == 0){
		clk_enable(pwm->clk);  /*enable clk*/
		pwm->clk_status = 1;
	}	
	//printk("pwm_enable pwm_id = %d CMU_DEVCLKEN1 = %x ",pwm_id, act_readl(CMU_DEVCLKEN1));
	pwm->running = 1;

	return 0;
}
EXPORT_SYMBOL(pwm_enable);

void pwm_disable(struct pwm_device *pwm)
{
//	u32 pwm_id = (u32) pwm->pwm_id;
	
	//act_clearl(((1 << 23) << pwm_id), CMU_DEVCLKEN1);
	if(pwm->clk_status == 1){
		clk_disable( pwm->clk);  /*enable clk*/
		pwm->clk_status = 0;
	}	
	//printk("pwm_disable pwm_id = %d CMU_DEVCLKEN1 = %x ",pwm_id, act_readl(CMU_DEVCLKEN1));
	pwm->running = 0;
}
EXPORT_SYMBOL(pwm_disable);

#define NS_IN_HZ (1000000000UL)

int pwm_config(struct pwm_device *pwm, int duty_ns, int period_ns)
{
	unsigned long source_period_ns;
	unsigned int tmp = 0, rate = 0 ,duty = 0 , pwm_clk_div = 0;
	struct clk * parent_clk = NULL; 
	
	/* We currently avoid using 64bit arithmetic by using the
	* fact that anything faster than 1Hz is easily representable
	* by 32bits. */
	
	if (period_ns > NS_IN_HZ || duty_ns > NS_IN_HZ  || period_ns <= 0)
		return -ERANGE;
	
	if (duty_ns > period_ns)
		return -EINVAL;
	
//	if (period_ns != pwm->required_period_ns)
	{
		rate = NS_IN_HZ / period_ns;
		pwm->required_period_ns = period_ns;
		
		//pwm实际频率为频率源/64/分频比,此处的条件判断为频率源/64
		if (rate < 32) {
			parent_clk = clk_get_sys(CLK_NAME_LOSC, NULL); 
			pwm_clk_div = MAX_PWM_CLK_DIV;
		} else if(rate <= CMU_HOSC_FREQ / MAX_PWM_CLK_DIV) {
			parent_clk = clk_get_sys(CLK_NAME_HOSC, NULL); 
			pwm_clk_div = MAX_PWM_CLK_DIV;
		} else if(rate > CMU_HOSC_FREQ / MAX_PWM_CLK_DIV) {
			parent_clk = clk_get_sys(CLK_NAME_HOSC, NULL); 
			pwm_clk_div = CMU_HOSC_FREQ / rate;
		}
		clk_set_parent(pwm->clk, parent_clk);
		//printk(KERN_ERR "parent clock = %d , target clock = %d \n",clk_get_rate(parent_clk), rate);				
		clk_set_rate(pwm->clk, rate * pwm_clk_div); 	
	}
    if(pwm_clk_div == 0){
       pwm_clk_div = ((act_readl(PWM_CTL0) >> 9) & 0x3ff) + 1;
    }
    duty = (duty_ns * (pwm_clk_div - 1)) / period_ns;
    duty = (duty) ? (duty-1) : 0;    
	//printk(KERN_ERR "duty_ns = %d , period_ns = %d duty = 0x%x \n",duty_ns , period_ns,duty);		
	tmp |= (duty << 19);
	tmp |= ((pwm_clk_div - 1) << 9);
	tmp |= (pwm->polarity << 8);
		
	act_writel(tmp, PWM_CTL0 + (pwm->pwm_id << 2));

/*      pwm_print_dbg();
*/
	return 0;
}
EXPORT_SYMBOL(pwm_config);

static int pwm_register(struct pwm_device *pwm)
{
/*	
printk(KERN_INFO "pwm %d register\n", pwm->pwm_id);
*/
	pwm->duty_ns = -1;
	pwm->period_ns = -1;
	pwm->required_period_ns = -1;
	pwm->polarity = 1;

	mutex_lock(&pwm_lock);
	list_add_tail(&pwm->list, &pwm_list);
	mutex_unlock(&pwm_lock);

	return 0;
}

void pwm_change_polarity(struct pwm_device *pwm, unsigned int polarity)
{
	pwm->polarity = polarity;
}
EXPORT_SYMBOL(pwm_change_polarity);

static int act_pwm_probe(struct platform_device *pdev)
{
		struct device *dev = &pdev->dev;
		struct pwm_device *pwm;
		
		unsigned int id = pdev->id;
		int ret;
		char buff[16];
		
		pwm = kzalloc(sizeof(struct pwm_device), GFP_KERNEL);
		if (pwm == NULL)
		{
			dev_err(dev, "failed to allocate pwm_device\n");
			return -ENOMEM;
		}
		
		pwm->pdev = pdev;
		pwm->pwm_id = id;
		
		ret = pwm_register(pwm);
		if (ret)
		{
			dev_err(dev, "failed to register pwm\n");
			goto err_alloc;
		}
		
		sprintf(buff, "pwm%d_clk", id);
		
		pwm->clk = clk_get_sys(buff, NULL);
		pwm->clk_status = 0;
		platform_set_drvdata(pdev, pwm);
		return 0;

err_alloc:
	kfree(pwm);
	return ret;
}

static int __devexit act_pwm_remove(struct platform_device *pdev)
{
	struct pwm_device *pwm = platform_get_drvdata(pdev);

	mutex_lock(&pwm_lock);
	list_del(&pwm->list);
	mutex_unlock(&pwm_lock);

	kfree(pwm);

	return 0;
}

static struct platform_driver act_pwm_driver =
{
	.driver =
    {
        .name = "asoc-pwm",
        .owner = THIS_MODULE,
    },
	.probe = act_pwm_probe,
	.remove = act_pwm_remove,
};

static struct platform_device act_pwm0_device = {
    .name   ="asoc-pwm",
    .id		= 0,
};

static struct platform_device act_pwm1_device = {
    .name   ="asoc-pwm",
    .id		= 1,
};

static struct platform_device act_pwm2_device = {
    .name   ="asoc-pwm",
    .id		= 2,
};

static struct platform_device act_pwm3_device = {
    .name   ="asoc-pwm",
    .id		= 3,
};

static struct platform_device *act_pwm_devices[] __initdata = {
    &act_pwm0_device,
    &act_pwm1_device,
    &act_pwm2_device,
    &act_pwm3_device,
};

static int __init pwm_init(void)
{ 
	int ret = 0;

    ret = platform_add_devices(act_pwm_devices, ARRAY_SIZE(act_pwm_devices));
    if(ret){
		 printk("platform register pwm device error =%d \n",ret);
		 return -1;
	}
    printk(KERN_INFO "platform register pwm driver: %s\n", act_pwm_driver.driver.name);
	return platform_driver_register(&act_pwm_driver);
}

arch_initcall(pwm_init);
