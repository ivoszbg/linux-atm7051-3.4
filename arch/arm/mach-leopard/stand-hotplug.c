/* linux/arch/arm/mach-leopard/stand-hotplug.c
 *
 * Copyright (c) 2012 actions Electronics Co., Ltd.
 *		http://www.actions-semi.com/
 *
 * gs702a - Dynamic CPU hotpluging
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/serial_core.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/cpu.h>
#include <linux/percpu.h>
#include <linux/ktime.h>
#include <linux/tick.h>
#include <linux/kernel_stat.h>
#include <linux/sched.h>
#include <linux/suspend.h>
#include <linux/reboot.h>
#include <linux/gpio.h>
#include <linux/cpufreq.h>
#include <linux/moduleparam.h>
#include <linux/cpufreq.h>
#include <linux/earlysuspend.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <asm-generic/cputime.h>
#include <mach/dvfslevel.h>

/*structures*/
struct freq_tran_work {
	struct delayed_work work;
	unsigned int new;
};

struct early_suspend_work_t {
	struct delayed_work work;
	unsigned int flag;
};

struct cpu_time_info {
	cputime64_t prev_cpu_idle;
	cputime64_t prev_cpu_wall;
	unsigned int load;
};

struct cpu_hotplug_info {
	unsigned long nr_running;
	pid_t tgid;
};

struct cpu_hotplug_rat{
	int up_rate;
	int down_rate;
};

enum flag{
	HOTPLUG_NOP,
	HOTPLUG_IN,
	HOTPLUG_OUT
};

enum SUSPEND_STAT{
		UP_CPU1=0x1,
		DOWN_CPU1=0x11,
};


/*consts and macros*/
#define INPUT_MODE_DETECT 0

/*
*currently, there are 1 cpus,  
*when load average over 20%(every one over 80%), to up one, then load for every online cpu over 40%. 
*/
#define UP_CPU_CUR_1CPU 20
/*
*currently, there are 2 cpus, 
*when load average below 15%(every one below 30%), to shut down one, then load average below 60%. 
*when load average over 30%(every one over 60%), to up one, then load average over 40%. 
*/
#define UP_CPU_CUR_2CPU   30
#define DOWN_CPU_CUR_2CPU 15

/*
*currently, there are 3 cpus, 
*when load average below 25%(every one below 33%), to shut down one, then load average below 50%. 
*when load average over 50%(every one over 66%), to up one, then load average over 37.5%. 
*/
#define UP_CPU_CUR_3CPU 50
#define DOWN_CPU_CUR_3CPU 25

/*
*currently, there are 4 cpus, 
*when load average below 36%(every one below 36%), to shut down one, then load average below 48%. 
*/
#define DOWN_CPU_CUR_4CPU 39
/*load threshold*/
static const unsigned int threshold_2core[][2] = {
		{0,  20},
		{15, 30},
		{25, 50},
		{39, 100},
		{100, 100}/*no use*/
};

static const unsigned int threshold_4core[][2] = {
		{0,  15},
		{10, 25},
		{20, 45},
		{30, 100},
		{100, 100}/*no use*/
};

static unsigned int threshold[5][2];

static void init_threshold(void)
{
    int dvfslevel=asoc_get_dvfslevel();
    
    switch (ASOC_GET_IC(dvfslevel))
    {
    case 0x7023:
    case 0x7021:    
        printk("%s,%d,size:0x%x\n", __FUNCTION__, __LINE__,sizeof(threshold_2core));
        memcpy(threshold, threshold_2core, sizeof(threshold_2core));
        
        break;        
    case 0x7029:
        printk("%s,%d,size:0x%x\n", __FUNCTION__, __LINE__,sizeof(threshold_4core));
        memcpy(threshold, threshold_4core, sizeof(threshold_4core));
        break;     
    default:
        printk("%s,%d,invalid dvfslevel\n", __FUNCTION__, __LINE__);
        break;
    }	
}

static int hot_plug_cpu_sum = CONFIG_NR_CPUS;
static void init_hot_plug_cpu_sum()
{
		int i;
		for(i=0; i<CONFIG_NR_CPUS; i++)
		{
				if(!cpu_possible(i))
					break;
		}
		hot_plug_cpu_sum = i;
}
/*更低的负载应该加速关闭cpu*/
static int down_count_convergence(int load)
{
		//todo, make a good convergence path
		if(load < 10)
			return 2;
		else
			return 1;			
}

#define BOOT_DELAY	60

#define CHECK_DELAY_ON	 (.3*HZ * 4)
#define CHECK_DELAY_OFF  (.3*HZ)

#define TRANS_RQ 2
#define TRANS_LOAD_RQ 20

#define CPU_OFF 0
#define CPU_ON  1

#define HOTPLUG_UNLOCKED 0
#define HOTPLUG_LOCKED 1
#define PM_HOTPLUG_DEBUG 0

#define cputime64_sub(__a, __b)		((__a) - (__b))

/*debug abouts*/
#define PRINT_PLUG_INFO 1
#if PRINT_PLUG_INFO
//static unsigned int g_avg_load = 0;
#endif

#define DBG_PRINT(fmt, ...)\
	if(PM_HOTPLUG_DEBUG)			\
		printk(pr_fmt(fmt), ##__VA_ARGS__)

#define LAOD_DEBUG_ON 0
#define DBG_LOAD(fmt, ...)\
	if(LAOD_DEBUG_ON)			\
		printk(pr_fmt(fmt), ##__VA_ARGS__)

#define ASSERT_HOTPLUG(v)\
if(v) printk("%s, %d erro\n", __FILE__, __LINE__)

	
/*static varables*/
#define MAX_DOWN_COUNT 12
#ifdef MAX_DOWN_COUNT
static int down_count = 0;
#endif

static unsigned int freq_min = 1000000;
static unsigned int freq_max = 0;
static unsigned int max_performance;

static struct workqueue_struct *hotplug_wq;
static struct delayed_work hotplug_work;
static struct early_suspend_work_t early_suspend_work;
static struct early_suspend early_suspend;
static struct freq_tran_work freq_trans_works;
static struct delayed_work detecting_freq_work;

static unsigned int fb_lock = 1;
static unsigned int user_lock = 1;
static unsigned int freq_lock = 1;
static unsigned int plug_mask = 0;
static unsigned int plug_test = 0;
#if INPUT_MODE_DETECT
static unsigned long input_interval = 0;
#endif

static int work_sequence_on = 0;
static unsigned int trans_rq= TRANS_RQ;
static unsigned int trans_load_rq = TRANS_LOAD_RQ;
static unsigned int hotpluging_rate = CHECK_DELAY_OFF;

static struct clk *cpu_clk;
static DEFINE_MUTEX(hotplug_lock);
static DEFINE_PER_CPU(struct cpu_time_info, hotplug_cpu_time);

#define COMMON_RATE  (.3*HZ * 4)
#define RATE_FOR_UP (.3*HZ)
#define RATE_FOR_DOWN (.3*HZ * 4)

static struct cpu_hotplug_rat cpu_plug_rate_def[] = {
		{
			.up_rate   = RATE_FOR_UP,  /*no use*/
			.down_rate = RATE_FOR_UP   /*no use*/
		},
		{
			.up_rate   = RATE_FOR_UP,   /*no use*/
			.down_rate = RATE_FOR_UP    /*only cpu0 left, check for up action*/
		},
		{
			.up_rate   = RATE_FOR_UP,   /*check to up next cpu*/
			.down_rate = RATE_FOR_DOWN  /*check to shut down next cpu*/
		},
		{
			.up_rate   = RATE_FOR_UP,   /*check to up next cpu*/
			.down_rate = RATE_FOR_DOWN  /*check to shut down next cpu*/ 
		},
		{
			.up_rate   = RATE_FOR_DOWN, /*all cpu online, check for shut down cpus*/ 
			.down_rate = RATE_FOR_DOWN  /*no use*/
		},
};



/*externs*/
extern unsigned long clk_get_rate(struct clk *clk);
extern struct clk *clk_get_sys(const char *dev_id, const char *con_id);
extern unsigned long get_cpu_nr_running(unsigned int cpu);
extern int __cpuinit cpu_up(unsigned int cpu);
extern int __ref cpu_down(unsigned int cpu);

static int test_cpu_not_mask(int cpu)
{
		if(cpu < hot_plug_cpu_sum)
			return (plug_mask & (1<<cpu)) == 0;
		return 1;	
}

#if INPUT_MODE_DETECT
/*
在有用户输入的情况下，为了减少震荡，提高开cpu门槛，降低关cpu门槛； 
*/
static int check_loading_for_up(unsigned int load, int online)
{
		if(time_after_eq(jiffies, input_interval))
		{
				printk("+");
				return load > threshold[online-1][1];
		}	
		else
		{
				printk("^");
				return load > threshold[online-1][1] + 5;
		}	
}
static int check_loading_for_down(unsigned int load, int online)
{
		if(time_after_eq(jiffies, input_interval))
		{
				printk("-");
				return load < threshold[online-1][0];
		}	
		else
		{
				printk("|");
				return load < threshold[online-1][0] - 10;
		}	
}
#else
static int check_loading_for_up(unsigned int load, int online)
{
		//printk("threshold[%d][1]: %d\n", online-1, threshold[online-1][1]);
		return load > threshold[online-1][1];
}
static int check_loading_for_down(unsigned int load, int online)
{
		//printk("threshold[%d][0]: %d\n", online-1, threshold[online-1][0]);
		return load < threshold[online-1][0];
}
#endif

/*kernel fuctions*/
static inline enum flag
standalone_hotplug(unsigned int load, unsigned long load_min, unsigned long load_min_rq)
{
	unsigned int cur_freq;
	unsigned int nr_online_cpu;
	unsigned int avg_load;

	cur_freq = clk_get_rate(cpu_clk) / 1000;
		
	nr_online_cpu = num_online_cpus();
	
	if((nr_online_cpu == 1) && (fb_lock == 1))
	{
		DBG_LOAD("IN on light screen\n");
		return HOTPLUG_IN;
	}
	avg_load = (unsigned int)((cur_freq * load) / max_performance);
	DBG_LOAD("avg_load=%d, cur_freq=%d, nr_running()=%d ", avg_load, cur_freq, nr_running());
	DBG_LOAD("nr_online_cpu: %d, load_min: %d, load_min_rq: %d\n", nr_online_cpu, load_min, load_min_rq);
	
	if (nr_online_cpu > 1 && \
		(check_loading_for_down(avg_load, nr_online_cpu) || cur_freq <= freq_min))
	{
		DBG_LOAD("-0");
		goto cpu_plug_out;
	} 
	/* If total nr_running is less than cpu(on-state) number, hotplug do not hotplug-in */
	else if (nr_running() > nr_online_cpu &&
		   check_loading_for_up(avg_load, nr_online_cpu)  && cur_freq > freq_min) 
	{
		down_count = 0;	
		DBG_LOAD("IN\n");
		return HOTPLUG_IN;
	} 
	else if ((nr_online_cpu > 1) && (load_min_rq < trans_rq)) 
	{
		/*If CPU(cpu_rq_min) load is less than trans_load_rq, hotplug-out*/
		if(load_min < trans_load_rq)
		{
			DBG_LOAD("-1");
			goto cpu_plug_out;
		}
	}

	return HOTPLUG_NOP;
	
cpu_plug_out:

	down_count +=down_count_convergence(avg_load);
	if(down_count > MAX_DOWN_COUNT)
	{	
			down_count = 0;
			DBG_LOAD("OUT\n");
	 		return HOTPLUG_OUT;
	}
	else
	{
			return HOTPLUG_NOP;
	}
}

static __cpuinit void cpu_hot_plug_test_for_stability(void)
{
		int cpu;
		unsigned nr_online_cpu = num_online_cpus();
		for(cpu=1; cpu<CONFIG_NR_CPUS; cpu++)
			if (cpu_possible(cpu) && test_cpu_not_mask(cpu)) 
			{
					if(cpu_online(cpu) == CPU_OFF)
					{	
							printk("cpu%d turning on start...", cpu);
							cpu_up(cpu);
							hotpluging_rate = cpu_plug_rate_def[nr_online_cpu].up_rate;
							printk("end\n");
					}
					else
					{
							printk("cpu%d turning off start...", cpu);
							cpu_down(cpu);	
							hotpluging_rate = cpu_plug_rate_def[nr_online_cpu-1].down_rate;
							printk("end\n");
					}
			}
}

static  __cpuinit int cpu_switch_logic(enum flag to_plug, int cpu)
{
		int i;
		int check_rate = RATE_FOR_DOWN;
		//printk("cpu_switch_logic to_plug: %d, cpu: %d\n", to_plug, cpu);	
		switch(to_plug)
		{
				case HOTPLUG_IN:
					for(i=hot_plug_cpu_sum-1; i>0; i--)
					{
							if(cpu_possible(i) && test_cpu_not_mask(i) && (cpu_online(i) == CPU_OFF))
							{
								printk("cpu%d turning on start...", i);
								cpu_up(i);
								printk("end\n");
								check_rate = cpu_plug_rate_def[num_online_cpus()].up_rate;
								break;
							}		
					}
					break;
				case HOTPLUG_OUT:
					/*for dual core case, if screent is on light, not shutdown the cpu1*/
					if((cpu==0) || (cpu_possible(2)  && fb_lock == 1 && num_online_cpus() <= 2))
						break;
					if(cpu_possible(cpu) && test_cpu_not_mask(cpu) && (cpu_online(cpu) == CPU_ON))
					{
						printk("cpu%d turning off start...", cpu);
						cpu_down(cpu);
						printk("end\n");
						check_rate = cpu_plug_rate_def[num_online_cpus()].down_rate;
						break;
					}		
					break;
				case HOTPLUG_NOP:
					break;	
				default:
					printk("wrong plug state: %d\n", to_plug);		
		}
		//printk("%s, %d\n", __FUNCTION__, __LINE__);
		return check_rate;
}

static __cpuinit void hotplug_timer(struct work_struct *work)
{
	int i;
	unsigned int load = 0;
	unsigned int load_min = 0;
	unsigned int cpu_rq_min = -1UL;
	unsigned int cpu_to_shut_down=0;
	enum flag flag_hotplug;
	mutex_lock(&hotplug_lock);
	//printk("%s, %d\n", __FUNCTION__, __LINE__);
	/*just for test system stability*/
	if(plug_test)
	{
			cpu_hot_plug_test_for_stability();
			goto no_hotplug;
	}		
	
	if((freq_lock == 1) && 
		 ( ((fb_lock != 0) && (num_online_cpus() <= 2)) || 
		   ((fb_lock == 0) && (num_online_cpus() == 1 )) ) 
		)
	{
		/*cancel the work sequence*/
		work_sequence_on = 0;
		mutex_unlock(&hotplug_lock);
		//printk("disable cpu_hot_plug\n");
		return;
	}
#if 0
	struct timeval trace_start_tv;
	struct timeval trace_end_tv;
	do_gettimeofday(&trace_start_tv);	
#endif	
	if (user_lock == 1)
	{
		goto no_hotplug;
	}

	for_each_online_cpu(i) {
		struct cpu_time_info *tmp_info;
		cputime64_t cur_wall_time, cur_idle_time;
		unsigned int idle_time, wall_time;

		tmp_info = &per_cpu(hotplug_cpu_time, i);

		cur_idle_time = get_cpu_idle_time_us(i, &cur_wall_time);

		idle_time = (unsigned int)cputime64_sub(cur_idle_time,
							tmp_info->prev_cpu_idle);
		tmp_info->prev_cpu_idle = cur_idle_time;

		wall_time = (unsigned int)cputime64_sub(cur_wall_time,
							tmp_info->prev_cpu_wall);
		tmp_info->prev_cpu_wall = cur_wall_time;

		if (wall_time < idle_time)
			goto no_hotplug;

		if (wall_time == 0)	wall_time++;

		tmp_info->load = 100 * (wall_time - idle_time) / wall_time;

		load += tmp_info->load;
		/*find the cpu which has fewest thread, this one is the best to kill*/
		if(i && (cpu_rq_min >  get_cpu_nr_running(i))){
				load_min = tmp_info->load;
				cpu_rq_min = get_cpu_nr_running(i);
				cpu_to_shut_down = i;
		}
	}

	/*standallone hotplug*/
	flag_hotplug = standalone_hotplug(load, load_min, cpu_rq_min);

	/*cpu hotplug*/
	hotpluging_rate = cpu_switch_logic(flag_hotplug, cpu_to_shut_down);

no_hotplug:
	queue_delayed_work_on(0, hotplug_wq, &hotplug_work, hotpluging_rate);
	mutex_unlock(&hotplug_lock);
#if 0	
	do_gettimeofday(&trace_end_tv);	
	long time_us = (trace_end_tv.tv_sec - trace_start_tv.tv_sec)*1000000 + \
			 (trace_end_tv.tv_usec- trace_start_tv.tv_usec);
	printk("used time: %d\n", time_us);	
#endif		
	//printk("%s, %d\n", __FUNCTION__, __LINE__);
}

/*notifiers*/
static int leopard_pm_hotplug_notifier_event(struct notifier_block *this,
					     unsigned long event, void *ptr)
{
	static unsigned user_lock_saved;
	printk(KERN_DEBUG "%s, %d\n", __FUNCTION__, __LINE__);
	switch (event) {
	case PM_SUSPEND_PREPARE:
	case PM_HIBERNATION_PREPARE:
	case PM_RESTORE_PREPARE:
		mutex_lock(&hotplug_lock);
		user_lock_saved = user_lock;
		user_lock = 1;
		DBG_PRINT("%s: saving pm_hotplug lock %x\n",
			__func__, user_lock_saved);
		mutex_unlock(&hotplug_lock);
		return NOTIFY_OK;
	case PM_POST_SUSPEND:
	case PM_POST_HIBERNATION:
	case PM_POST_RESTORE:
		mutex_lock(&hotplug_lock);
		DBG_PRINT("%s: restoring pm_hotplug lock %x\n",
			__func__, user_lock_saved);
		user_lock = user_lock_saved;
		mutex_unlock(&hotplug_lock);
		return NOTIFY_OK;
	}
	printk(KERN_DEBUG "%s, %d\n", __FUNCTION__, __LINE__);
	return NOTIFY_DONE;
}

static struct notifier_block leopard_pm_hotplug_notifier = {
	.notifier_call = leopard_pm_hotplug_notifier_event,
};

static int hotplug_reboot_notifier_call(struct notifier_block *this,
					unsigned long code, void *_cmd)
{
	printk("%s, %d\n", __FUNCTION__, __LINE__);
	mutex_lock(&hotplug_lock);
	DBG_PRINT("%s: disabling pm hotplug\n", __func__);
	user_lock = 1;
	mutex_unlock(&hotplug_lock);
	printk("%s, %d\n", __FUNCTION__, __LINE__);
	return NOTIFY_DONE;
}

static struct notifier_block hotplug_reboot_notifier = {
	.notifier_call = hotplug_reboot_notifier_call,
};



static void start_cpu_hot_plug(int delay)
{
	queue_delayed_work_on(0, hotplug_wq, &hotplug_work, delay);
}

static int cpufreq_stat_notifier_trans(struct notifier_block *nb,
		unsigned long val, void *data)
{
	//int orgize_todo = 0;
	struct cpufreq_freqs *freq = data;

	if ((val != CPUFREQ_POSTCHANGE) || (freq->cpu != 0) )
		return 0;
	freq_trans_works.new = freq->new;
	queue_delayed_work_on(0, hotplug_wq, &(freq_trans_works.work), 0);
	return 0;
}

static void adjust_hot_plug_with_freq(struct work_struct *work)
{
	struct freq_tran_work *freq_tran_workp = (struct freq_tran_work*)work;
	if (freq_tran_workp->new > freq_max/2)
	{
		mutex_lock(&hotplug_lock);
		if(work_sequence_on == 0)
				start_cpu_hot_plug(0);
		freq_lock = 0;
		work_sequence_on = 1;
		mutex_unlock(&hotplug_lock);
		//printk("enable cpu_hot_plug, freq: %d, thresh: %d\n", freq_tran_workp->new, freq_max/2);
	}
	else
	{
			mutex_lock(&hotplug_lock);
			freq_lock = 1;
			//printk( "freq_lock: %d, freq: %d, thresh: %d\n", freq_lock, freq_tran_workp->new, freq_max/2);	
			mutex_unlock(&hotplug_lock);
	}
	
}

static struct notifier_block notifier_freqtrans_block = {
	.notifier_call = cpufreq_stat_notifier_trans
};

static __cpuinit void early_suspend_process(struct work_struct *work)
{
		printk(KERN_DEBUG "%s, %d\n", __FUNCTION__, __LINE__);
		struct early_suspend_work_t *tmp = (struct early_suspend_work_t *)work;
		switch(tmp->flag)
		{
				case UP_CPU1:
					mutex_lock(&hotplug_lock);
					fb_lock = 1;
					mutex_unlock(&hotplug_lock);	
					break;
				case DOWN_CPU1:
					mutex_lock(&hotplug_lock);
					fb_lock = 0;
					mutex_unlock(&hotplug_lock);	
					break;
				default:		
					printk("wrong falg: 0x%08x\n", tmp->flag);
		}
		printk(KERN_DEBUG "%s, %d\n", __FUNCTION__, __LINE__);
}

static void hotplug_early_suspend(struct early_suspend *h)
{
		early_suspend_work.flag = DOWN_CPU1;
		printk( "hotplug_early_suspend\n");
		queue_delayed_work_on(0, hotplug_wq, &(early_suspend_work.work), 0);
}

static void hotplug_late_resume(struct early_suspend *h)
{
		early_suspend_work.flag = UP_CPU1;
		printk( "hotplug_late_resume\n");
		queue_delayed_work_on(0, hotplug_wq, &(early_suspend_work.work), 0);
}


#if INPUT_MODE_DETECT
/*input about*/
struct cpuplug_inputopen {
	struct input_handle *handle;
	struct work_struct inputopen_work;
};

static struct cpuplug_inputopen inputopen;
static void cpuplug_input_event(struct input_handle *handle,
					    unsigned int type,
					    unsigned int code, int value)
{
	input_interval = jiffies + 30*HZ;
}

static void cpuplug_input_open(struct work_struct *w)
{
	struct cpuplug_inputopen *io =
		container_of(w, struct cpuplug_inputopen,
			     inputopen_work);
	int error;

	error = input_open_device(io->handle);
	if (error)
		input_unregister_handle(io->handle);
}

static int cpuplug_input_connect(struct input_handler *handler,
					     struct input_dev *dev,
					     const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	printk("%s: connect to %s\n", __func__, dev->name);
	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cpuplug";

	error = input_register_handle(handle);
	if (error)
		goto err;

	inputopen.handle = handle;
	queue_work(hotplug_wq, &inputopen.inputopen_work);
	return 0;
	
err:
	kfree(handle);
	return error;
}

static void cpuplug_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id cpuplug_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			 INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
			    BIT_MASK(ABS_MT_POSITION_X) |
			    BIT_MASK(ABS_MT_POSITION_Y) },
	}, /* multi-touch touchscreen */
	{
		.flags = INPUT_DEVICE_ID_MATCH_KEYBIT |
			 INPUT_DEVICE_ID_MATCH_ABSBIT,
		.keybit = { [BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
		.absbit = { [BIT_WORD(ABS_X)] =
			    BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) },
	}, /* touchpad */
	{ },
};

static struct input_handler cpuplug_input_handler = {
	.event          = cpuplug_input_event,
	.connect       = cpuplug_input_connect,
	.disconnect   = cpuplug_input_disconnect,
	.name          = "cpuplug",
	.id_table      = cpuplug_ids,
};
#endif

static void set_cpu_frequence(struct work_struct *work)
{
	int i;
	unsigned int freq;
	struct cpufreq_frequency_table *table;
	table = cpufreq_frequency_get_table(0);
	if(NULL == table)
	{
		queue_delayed_work_on(0, hotplug_wq, &detecting_freq_work, HZ);
		DBG_PRINT(KERN_INFO "cpu hot plug set_cpu_frequence, cpufrequnce set failed\n");
		return;
	}
	for (i = 0; table[i].frequency != CPUFREQ_TABLE_END; i++) {
		freq = table[i].frequency;
		if(table[i].frequency == CPUFREQ_TABLE_END)
			break;
		if (freq > freq_max)
			freq_max = freq;
		if (freq_min > freq)
			freq_min = freq;
	}
	/*get max frequence*/
	max_performance = freq_max * hot_plug_cpu_sum;
	DBG_PRINT("freq_min %d\n", freq_min);
	DBG_PRINT("freq_max %d, HOTPLUG_CPU_SUM: %d\n", freq_max, hot_plug_cpu_sum);
	DBG_PRINT(KERN_INFO "cpu hot plug set_cpu_frequence, cpufrequnce set ok\n");

	start_cpu_hot_plug(BOOT_DELAY * HZ);
	
	/*enable cpu hot plug*/
	mutex_lock(&hotplug_lock);
	user_lock = 0;
	freq_lock = 0;
	
	/*on default, sequence work is on duty*/
	work_sequence_on = 1;
	mutex_unlock(&hotplug_lock);

	if (cpufreq_register_notifier(&notifier_freqtrans_block, CPUFREQ_TRANSITION_NOTIFIER)) 
	{
		printk("cpufreq_register_notifier failed\n");
		return;
	}
	
#if INPUT_MODE_DETECT
	if (input_register_handler(&cpuplug_input_handler))
		printk("%s: failed to register input handler\n",	__func__);
#endif
	return;
}

/*sys interface*/
static ssize_t show_usr_lock(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	return sprintf(buf, "%u\n", user_lock);
}

static ssize_t __ref store_usr_lock(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	ssize_t ret = count;
	switch (buf[0]) {
	case '0':
		mutex_lock(&hotplug_lock);
		user_lock = 0;
		mutex_unlock(&hotplug_lock);
		DBG_PRINT("%s: user_lock: %d\n", __func__, user_lock);
		break;
	case '1':
		mutex_lock(&hotplug_lock);
		user_lock = 1;
		mutex_unlock(&hotplug_lock);
		DBG_PRINT("%s: user_lock: %d\n", __func__, user_lock);
		break;
	default:
		DBG_PRINT("%s: user_lock: %d\n", __func__, user_lock);
		ret = -EINVAL;
	}
	return count;
}
static DEVICE_ATTR(usr_lock, 0644, show_usr_lock,  store_usr_lock);
static ssize_t show_freq_lock(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	return sprintf(buf, "%u\n", freq_lock);
}
static DEVICE_ATTR(freq_lock, 0644, show_freq_lock,  NULL);

static int time_flag = 0;
static int time_flag_2 = 0;

int get_time_flag(void)
{
		return time_flag;
}

static ssize_t show_time(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	return sprintf(buf, "%u\n", time_flag);
}
static ssize_t __ref store_time(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	ssize_t ret = count;
	switch (buf[0]) {
	case '0':
		time_flag = 0;
		//printk("case 0: %s: time_flag: %d\n", __func__, time_flag);
		break;
	case '1':
		time_flag = 1;
		//printk("case 1: %s: time_flag: %d\n", __func__, time_flag);
		break;
	default:
		//printk("default: %s: time_flag: %d\n", __func__, time_flag);
		ret = -EINVAL;
	}
	return count;
}

static ssize_t show_time_2(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	return sprintf(buf, "%u\n", time_flag_2);
}
static ssize_t __ref store_time_2(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	ssize_t ret = count;
	switch (buf[0]) {
	case '0':
		time_flag_2 = 0;
		//printk("case 0: %s: time_flag: %d\n", __func__, time_flag);
		break;
	case '1':
		time_flag_2 = 1;
		//printk("case 1: %s: time_flag: %d\n", __func__, time_flag);
		break;
	default:
		//printk("default: %s: time_flag: %d\n", __func__, time_flag);
		ret = -EINVAL;
	}
	return count;
}

static DEVICE_ATTR(opt, 0644, show_time,  store_time);
static DEVICE_ATTR(opt2, 0644, show_time_2,  store_time_2);

/*sys interface*/
static ssize_t show_plug_mask(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	return sprintf(buf, "0x%01x\n", plug_mask);
}
static ssize_t __ref store_plug_mask(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	mutex_lock(&hotplug_lock);
	sscanf(buf, "0x%01x", &plug_mask);
	mutex_unlock(&hotplug_lock);
	printk("plug_mask :0x%08x\n", plug_mask);
	return count;
}
static DEVICE_ATTR(plug_mask, 0644, show_plug_mask,  store_plug_mask);

static ssize_t show_plug_test(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	return sprintf(buf, "0x%01x\n", plug_test);
}

static ssize_t __ref store_plug_test(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	mutex_lock(&hotplug_lock);
	sscanf(buf, "0x%01x", &plug_test);
	mutex_unlock(&hotplug_lock);
	printk("plug_test :0x%08x\n", plug_test);
	return count;
}
static DEVICE_ATTR(plug_test, 0644, show_plug_test,  store_plug_test);
static struct attribute *cpuplug_lock_attrs[] = {
	&dev_attr_usr_lock.attr,
	&dev_attr_freq_lock.attr,
	&dev_attr_plug_mask.attr,
	&dev_attr_plug_test.attr,
	&dev_attr_opt.attr,
	&dev_attr_opt2.attr,
	NULL
};

static struct attribute_group cpuplug_attr_group = {
	.attrs = cpuplug_lock_attrs,
	.name = "autoplug",
};
/**
 * cpuplug_add_interface - add CPU global sysfs attributes
 */
int cpuplug_add_interface(struct device *dev)
{
	return sysfs_create_group(&dev->kobj, &cpuplug_attr_group);
}

/*inits*/
int leopard_pm_hotplug_init(void)
{
	//unsigned int i;
	//unsigned int freq;

	printk("leopard PM-hotplug init function\n");
	//hotplug_wq = create_workqueue("dynamic hotplug");
	hotplug_wq = alloc_workqueue("dynamic hotplug", 0, 0);
	if (!hotplug_wq) {
		DBG_PRINT(KERN_ERR "Creation of hotplug work failed\n");
		return -EFAULT;
	}
	/*early_suspend cpu hot plug*/
	INIT_DELAYED_WORK(&(early_suspend_work.work), early_suspend_process);
	early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	early_suspend.suspend = hotplug_early_suspend;
	early_suspend.resume  = hotplug_late_resume;
	register_early_suspend(&early_suspend);
	
	//INIT_DELAYED_WORK(&hotplug_work, hotplug_timer);
	INIT_DELAYED_WORK(&hotplug_work, hotplug_timer);
	INIT_DELAYED_WORK(&(freq_trans_works.work), adjust_hot_plug_with_freq);
	INIT_DELAYED_WORK(&detecting_freq_work, set_cpu_frequence);
#if INPUT_MODE_DETECT	
	INIT_WORK(&inputopen.inputopen_work, cpuplug_input_open);
#endif
	
	queue_delayed_work_on(0, hotplug_wq, &detecting_freq_work, HZ);

	/*default values when system booting*/
	user_lock = 1;
	freq_max = clk_get_rate(cpu_clk) / 1000;
	max_performance = freq_max * hot_plug_cpu_sum;


	DBG_PRINT("freq_min %d\n", freq_min);
	DBG_PRINT("freq_max %d, HOT_PLUG_CPU_SUM: %d\n", freq_max, hot_plug_cpu_sum);

	register_pm_notifier(&leopard_pm_hotplug_notifier);
	register_reboot_notifier(&hotplug_reboot_notifier);
	
	/*on default, sequence work of detecting cpufreq driver is on duty*/
	work_sequence_on = 1;
	init_hot_plug_cpu_sum();
	
	return 0;
}
late_initcall(leopard_pm_hotplug_init);

static struct platform_device leopard_pm_hotplug_device = {
	.name = "leopard-dynamic-cpu-hotplug",
	.id = -1,
};

extern int clk_enable(struct clk *c);
static int __init leopard_pm_hotplug_device_init(void)
{
	int ret;

	ret = platform_device_register(&leopard_pm_hotplug_device);
	if (ret) {
		DBG_PRINT(KERN_ERR "failed at(%d)\n", __LINE__);
		return ret;
	}

	ret = cpuplug_add_interface(cpu_subsys.dev_root);
	if (ret) {
		DBG_PRINT(KERN_ERR "failed at(%d)\n", __LINE__);
		return ret;
	}
	
	DBG_PRINT(KERN_INFO "leopard_pm_hotplug_device_init: %d\n", ret);
	
	cpu_clk = clk_get_sys("cpuclk", NULL);
	if (IS_ERR(cpu_clk))
	{
		DBG_PRINT(KERN_INFO "leopard_pm_hotplug_device_init clk_get_sys failed\n");
		return PTR_ERR(cpu_clk);
  }
	clk_enable(cpu_clk);
    init_threshold();    
	
	return ret;
}

late_initcall(leopard_pm_hotplug_device_init);


void set_cpu_hotplug_state(unsigned int lock)
{
    if(lock>1)
    {
        return ;
    }
    
    mutex_lock(&hotplug_lock);
	user_lock = lock;
	mutex_unlock(&hotplug_lock);
	
	return ;
}
EXPORT_SYMBOL(set_cpu_hotplug_state);
