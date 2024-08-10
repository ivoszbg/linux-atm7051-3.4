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

#define DEV_NAME   "g1_led"
 
#define G1_LED_1   	 0x0A01C000
#define G1_LED_2  	 0x0A00C000
#define G1_CHIP_ID   0x0A00B000

#define LED_NUM 2

#define __SNK_BOARD_BDN__	1

extern unsigned char neogeoChipID[7];

static unsigned int led_gpio_pin[LED_NUM];
static char *led_name[LED_NUM] = {
	"led_1",
	"led_2",
};

static int get_led_config(void)
{
	int ret = -1;
	struct gpio_pre_cfg pcfg[LED_NUM];
	int i;
		
	for(i=0; i < LED_NUM; i++)
	{
		memset((void *)&pcfg[i], 0, sizeof(struct gpio_pre_cfg));
		if (gpio_get_pre_cfg(led_name[i], &pcfg[i]))
		{
			printk("[%s] get led pin gpio %s failed!\n",__func__, led_name[i]);
			ret = -1;
			break;
		}
		else
		{
			led_gpio_pin[i] = ASOC_GPIO_PORT(pcfg[i].iogroup, pcfg[i].pin_num);
			printk("[%s] %s: %d, get led pin gpio ok!\n",__func__, led_name[i], led_gpio_pin[i]);
			ret = 0;
		}
	}
	
	return ret;
}

static int g1_led_ioctl(/*struct inode *inode,*/ struct file *file, unsigned int cmd, unsigned long arg)
{
	unsigned int val;
	int ret = 0;

	switch(cmd)
	{
		case G1_LED_1:
			val = *(unsigned int*)arg;

//			printk("ioctl G1_LED_1 %d\n", val);
#ifndef __SNK_BOARD_BDN__
			if(val == 0)
				gpio_direction_output(led_gpio_pin[0], 0);
			else
				gpio_direction_output(led_gpio_pin[0], 1);
#else
			if(val == 0)
				gpio_direction_output(led_gpio_pin[0], 1);
			else
				gpio_direction_output(led_gpio_pin[0], 0);
#endif
			
			break;
		case G1_LED_2:
			val = *(unsigned int*)arg;

//			printk("ioctl G1_LED_POWER %d\n", val);
#ifndef __SNK_BOARD_BDN__
			if(val == 0)
				gpio_direction_output(led_gpio_pin[1], 0);
			else
				gpio_direction_output(led_gpio_pin[1], 1);
#else
			if(val == 0)
				gpio_direction_output(led_gpio_pin[1], 1);
			else
				gpio_direction_output(led_gpio_pin[1], 0);
#endif

			break;
		case G1_CHIP_ID:
			copy_to_user((unsigned char*)arg, neogeoChipID, 7);
			break;
		default:
			printk("No such G1 LED ioctl command %#x!\n", cmd);
			ret = -1;
			break;
	}
		 
	return 0;
}

int g1_led_open(struct inode * inode, struct file * file)
{
	return 0;
}
 
int g1_led_close(struct inode * inode, struct file * file)
{
	return 0;
}
 
static struct file_operations g1_led_fops = {
	.owner             = THIS_MODULE,
	.unlocked_ioctl    = g1_led_ioctl,
//	.ioctl             = g1_led_ioctl,
	.open              = g1_led_open,
	.release           = g1_led_close
};
 
 
static struct miscdevice g1_led_dev = {
	.minor             = MISC_DYNAMIC_MINOR,                                                             .name              = DEV_NAME,
	.fops              = &g1_led_fops,
};


/***********************************/
static int __init g1_led_init(void)
{
	printk("%s %d\n", __FILE__, __LINE__);	
	int ret;
	int i;
	ret = 0;

	ret = get_led_config();

	if(ret == 0)
	{
		for(i = 0; i < LED_NUM; ++i)
		{
			if(gpio_request(led_gpio_pin[i], led_name[i]) != 0) 
			{
				ret = -EIO;
				printk("== led name %s ==\n", led_name[i]);
				gpio_free(led_gpio_pin[i]);
				printk("%s:led  gpio_request() error\n", __FUNCTION__);
				continue;
			}else
			{
#ifndef __SNK_BOARD_BDN__
				if(i == 0)
					gpio_direction_output(led_gpio_pin[i], 0);
				else
					gpio_direction_output(led_gpio_pin[i], 1);
#else
				if(i == 0)
					gpio_direction_output(led_gpio_pin[i], 1);
				else
					gpio_direction_output(led_gpio_pin[i], 0);
#endif
			}

		}
	}else
	{
		ret = -EIO;
	}

	if(ret == 0)
		ret = misc_register(&g1_led_dev);

	if (ret < 0)
	{
		printk("register G1 LED device failed ...!\n");
		ret = -EIO;
	}else
		printk(" G1 LED initialize success..! \n");

	return ret;
}

static void __exit g1_led_uninit(void)
{
	misc_deregister(&g1_led_dev);
}


module_init(g1_led_init);
module_exit(g1_led_uninit);

MODULE_AUTHOR("frank");
MODULE_DESCRIPTION("G1 LED driver");
MODULE_LICENSE("GPL");

