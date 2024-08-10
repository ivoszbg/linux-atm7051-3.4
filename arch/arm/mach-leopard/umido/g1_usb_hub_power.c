#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <asm/pgtable.h>
#include <asm/uaccess.h>		
#include <asm/processor.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/miscdevice.h>
#include <asm/gpio.h>
#include <linux/module.h>
#include <asm/io.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <mach/atc260x/atc260x.h>
#include <mach/bootafinfo.h>
#include <mach/atc260x/atc260x_pdata.h>

#include <mach/hardware.h>

#define DEV_NAME   "g1_usb_hub_power"
 
#define G1_HUB_POWER    0x0A01C000
#define G1_USB_SWITCH   0x0A00C000

static unsigned int gpio_hub_power;
static char* hub_power_name = "hub_power";

static unsigned int gpio_usb_switch;
static char* usb_switch_name = "usb_switch";

static int get_power_config(void)
{
	int ret = -1;
	struct gpio_pre_cfg pcfg;
	int i;
		
	memset((void *)&pcfg, 0, sizeof(struct gpio_pre_cfg));
	if (gpio_get_pre_cfg(hub_power_name, &pcfg))
	{
		printk("[%s] get hub power pin gpio %s failed!\n",__func__, hub_power_name);
		ret = -1;
		return ret;
	}
	else
	{
		gpio_hub_power = ASOC_GPIO_PORT(pcfg.iogroup, pcfg.pin_num);
		printk("[%s] %s: %d, get led pin gpio ok!\n",__func__, hub_power_name, gpio_hub_power);
		ret = 0;
	}
	
	memset((void *)&pcfg, 0, sizeof(struct gpio_pre_cfg));
	if (gpio_get_pre_cfg(usb_switch_name, &pcfg))
	{
		printk("[%s] get usb_switch pin gpio %s failed!\n",__func__, usb_switch_name);
		ret = -1;
	}
	else
	{
		gpio_usb_switch = ASOC_GPIO_PORT(pcfg.iogroup, pcfg.pin_num);
		printk("[%s] %s: %d, get usb switch pin gpio ok!\n",__func__, usb_switch_name, gpio_usb_switch);
		ret = 0;
	}
	
	return ret;
}

static int g1_usb_hub_power_ioctl(/*struct inode *inode,*/ struct file *file, unsigned int cmd, unsigned long arg)
{
	unsigned int val;
	int ret = 0;

	printk("ioctl cmd: %x\n", cmd);

	switch(cmd)
	{
		case G1_HUB_POWER:
			val = *(unsigned int*)arg;

			printk("ioctl hub power: %d\n", val);
			if(val == 0)
				gpio_direction_output(gpio_hub_power, 0);
			else
				gpio_direction_output(gpio_hub_power, 1);
			
			break;
		case G1_USB_SWITCH:
			val = *(unsigned int*)arg;

			printk("ioctl usb switch: %d\n", val);
			if(val == 0)
				gpio_direction_output(gpio_usb_switch, 0);
			else
				gpio_direction_output(gpio_usb_switch, 1);

			break;
		default:
			printk("No such G1 hub power ioctl command %#x!\n", cmd);
			ret = -1;
			break;
	}
		 
	return 0;
}

int g1_usb_hub_power_open(struct inode * inode, struct file * file)
{
	return 0;
}
 
int g1_usb_hub_power_close(struct inode * inode, struct file * file)
{
	return 0;
}
 
static struct file_operations g1_usb_hub_power_fops = {
	.owner             = THIS_MODULE,
	.unlocked_ioctl    = g1_usb_hub_power_ioctl,
//	.ioctl             = g1_usb_hub_power_ioctl,
	.open              = g1_usb_hub_power_open,
	.release           = g1_usb_hub_power_close
};
 
 
static struct miscdevice g1_usb_hub_power_dev = {
	.minor             = MISC_DYNAMIC_MINOR,                                                             .name              = DEV_NAME,
	.fops              = &g1_usb_hub_power_fops,
};


/***********************************/
static int __init g1_usb_hub_power_init(void)
{
	printk("%s %d\n", __FILE__, __LINE__);	
	int ret;
	ret = 0;
	
	ret = get_power_config();

	if(ret == 0)
	{
		if(gpio_request(gpio_hub_power, hub_power_name) == 0) 
			gpio_direction_output(gpio_hub_power, 0);
		else
			printk("request G1 USB POWER faild! \n");

		if(gpio_request(gpio_usb_switch, usb_switch_name) == 0) 
			gpio_direction_output(gpio_usb_switch, 0);
		else
			printk("request G1 USB SWITCH faild! \n");

		printk(" G1 USB POWER initialize success! \n");
	}else
		printk("register G1 USB POWER device failed!\n");

	ret = misc_register(&g1_usb_hub_power_dev);

	return ret;
}

static void __exit g1_usb_hub_power_uninit(void)
{
	misc_deregister(&g1_usb_hub_power_dev);
}


module_init(g1_usb_hub_power_init);
module_exit(g1_usb_hub_power_uninit);

MODULE_AUTHOR("frank");
MODULE_DESCRIPTION("G1 hub power driver");
MODULE_LICENSE("GPL");

