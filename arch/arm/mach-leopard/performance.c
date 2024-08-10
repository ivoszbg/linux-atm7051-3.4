/*
 * performance driver.
 */
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/fcntl.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>

#define PERFORMANCE_MINOR	156
enum PERFORMANCE_CMD{
	DROP_CACHE=0,
	OPEN_PERFORMANCE,
	CLOSE_PERFORMANCE,
	CMD_ENDS,
};

extern void fs_drop_page_caches(void);
extern ssize_t store_scaling_governor(struct cpufreq_policy *policy,
					const char *buf, size_t count);

static __cpuinit void open_performance_mode()
{
	int i;
	int ret;
	struct cpufreq_policy *policy;
	for(i=0; i<CONFIG_NR_CPUS; i++)
	{
			if(cpu_possible(i) && (cpu_online(i) == 0))
				cpu_up(i);
	}
	for_each_online_cpu(i)
  {
      policy = cpufreq_cpu_get(i);
      ret = store_scaling_governor(policy, "performance", 12);
      cpufreq_cpu_put(policy);
      if(ret < 0)
      	printk("store_scaling_governor performance failed\n");	
   }
}

static __cpuinit void close_performance_mode()
{
	int i;
	int ret;
	struct cpufreq_policy *policy;
	for(i=0; i<CONFIG_NR_CPUS; i++)
	{
			if(cpu_possible(i) && (cpu_online(i) == 0))
			{
				cpu_up(i);
			}
	}
	for_each_online_cpu(i)
  {
      policy = cpufreq_cpu_get(i);
      ret = store_scaling_governor(policy, "interactive", 12);
      cpufreq_cpu_put(policy);
      if(ret < 0)
      	printk("store_scaling_governor performance failed\n");	
  }
}

static __cpuinit long performance_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	if(cmd != 0)
		return -1;
	switch(arg)
	{
		case DROP_CACHE:
			//printk("performance_ioctl DROP_CACHE cmd\n");	
			fs_drop_page_caches();
			break;
		case OPEN_PERFORMANCE:
			//printk("performance_ioctl OPEN_PERFORMANCE cmd\n");	
			open_performance_mode();
			break;	
		case CLOSE_PERFORMANCE:
			//printk("performance_ioctl CLOSE_PERFORMANCE cmd\n");	
			close_performance_mode();
			break;	
		default:
			printk("performance_ioctl undefined cmd\n");	
	}
	return ret;
}

static int performance_open( struct inode * inode, struct file * file )
{
	return 0;
}

const struct file_operations performance_fops = {
	.unlocked_ioctl	= performance_ioctl,
	.open						= performance_open,
};

static struct miscdevice performance_dev = {
	PERFORMANCE_MINOR,
	"performance",
	&performance_fops
};

static int __init performance_init(void)
{
	int a;
	int retval;
	retval = misc_register(&performance_dev);
	return 0;
}

static void __exit performance_exit(void)
{
	misc_deregister(&performance_dev);
}

module_init(performance_init);
module_exit(performance_exit);
