/*
 * arch/arm/mach-leopard/board-leopardevb.c
 *
 * Board support file for Actions Leopard EVB
 *
 * Copyright 2012 Actions Semi Inc.
 * Author: Actions Semi, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/regulator/machine.h>
#include <linux/platform_device.h>
#include <linux/bootmem.h>
#include <linux/memblock.h>
#include <linux/spi/spi.h>
#include <linux/ion.h>
#include <linux/asoc_ion.h>     /* ASOC_ION_HEAP_CARVEOUT */
#include <linux/pinctrl/machine.h>
#include <linux/i2c.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <mach/spi.h>
#include <mach/atc260x/atc260x_pdata.h>
#include <mach/serial.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/hardware/gic.h>
#include <asm/setup.h>
#include <mach/clock.h>

extern void __init leopard_map_io(void);
extern void __init leopard_reserve(void);
extern void __init leopard_fixup(struct tag *tags, char **from,
            struct meminfo *meminfo);

extern void __init gic_init_irq(void);
struct sys_timer;
extern struct sys_timer leopard_timer;

/* ------------ ATC260x DCDC/LDO config ------------ */
/* 电源配置管理 
 * 新申请一路LDO 使用方式如下：
 *  regu = regulator_get(NULL, name);
 * name是指定LD0对应的别名,表示对应的ldo,在Platform.c有一些默认的名字，
 * 如果用户需要新添加一个新的别名就使用增加
 *　{
 *	.supply = "user name",
 * } 的方式来添加，在不添加的情况下使用Platform.c中的配置也可以。
 * 	.min_uV 	.max_uV  分别为输出电压的最小值和最大值的范围
 *　在没有设置的时候LDO 使用硬件默认的值。用户可以通过
 *  regulator_enable(regu) 使能LDO 
 *  regulator_disable(regu) 关闭LDO 
 *  regulator_set_voltage(regu, voltage, voltage) 设置电压值。
 *  regulator_get_voltage(regu)获取当前输出的电压值。
 */ 
/* DCDC1: SOC core */
 struct regulator_consumer_supply leopard_vddcore_consumers[] = {
	{
		.supply = "vddcore",
	}
};

 struct regulator_init_data leopard_vddcore = {
	.constraints = {
		.name = "vddcore",
		.min_uV = 700000,
		.max_uV = 1400000,
		.always_on = 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies = ARRAY_SIZE(leopard_vddcore_consumers),
	.consumer_supplies = leopard_vddcore_consumers,
};


/* DCDC2: DDR */
 struct regulator_consumer_supply leopard_vddr_consumers[] = {
	{
		.supply = "vddr",
	}
};

 struct regulator_init_data leopard_vddr = {
	.constraints = {
		.name = "vddr",
		.min_uV = 1400000,
		.max_uV = 2200000,
		.always_on = 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies = ARRAY_SIZE(leopard_vddr_consumers),
	.consumer_supplies = leopard_vddr_consumers,
};

/* DCDC3: VCC */
 struct regulator_consumer_supply leopard_vcc_consumers[] = {
	{
		.supply = "vcc",
	}
};

 struct regulator_init_data leopard_vcc = {
	.constraints = {
		.name = "vcc",
		.min_uV = 2600000,
		.max_uV = 3100000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies = ARRAY_SIZE(leopard_vcc_consumers),
	.consumer_supplies = leopard_vcc_consumers,
};

/* DCDC4: 5v for OTG, HDMI */
 struct regulator_consumer_supply leopard_vcc5v_consumers[] = {
	{
		.supply = "vcc5v",
	}
};

 struct regulator_init_data leopard_vcc5v = {
	.constraints = {
		.name = "vcc5v",
		.min_uV = 5000000,
		.max_uV = 5000000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(leopard_vcc5v_consumers),
	.consumer_supplies = leopard_vcc5v_consumers,
};


/* LDO1: AVCC for SOC */
 struct regulator_consumer_supply leopard_avcccore_consumers[] = {
	{
		.supply = "avcccore",
	}
};

 struct regulator_init_data leopard_avcccore = {
	.constraints = {
		.name = "avcccore",
		.min_uV = 2600000,
		.max_uV = 3300000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies = ARRAY_SIZE(leopard_avcccore_consumers),
	.consumer_supplies = leopard_avcccore_consumers,
};

/* LDO2: AVCC for ATC260x */
 struct regulator_consumer_supply leopard_avccpmic_consumers[] = {
	{
		.supply = "avccpmic",
	}
};

 struct regulator_init_data leopard_avccpmic = {
	.constraints = {
		.name = "avccpmic",
		.min_uV = 2600000,
		.max_uV = 3300000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies = ARRAY_SIZE(leopard_avccpmic_consumers),
	.consumer_supplies = leopard_avccpmic_consumers,
};

/* LDO3: VDD for ATC260x */
 struct regulator_consumer_supply leopard_vddpmic_consumers[] = {
	{
		.supply = "vddpmic",
	}
};

 struct regulator_init_data leopard_vddpmic = {
	.constraints = {
		.name = "vddpmic",
		.min_uV = 1500000,
		.max_uV = 2000000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies = ARRAY_SIZE(leopard_vddpmic_consumers),
	.consumer_supplies = leopard_vddpmic_consumers,
};

/* LDO4: WIFI3.3, Bluetooth */
 struct regulator_consumer_supply leopard_ldo4_consumers[] = {
	{
		.supply = "wifi33",
	},
	{
		.supply = "bluetooth",
	}
};

 struct regulator_init_data leopard_ldo4 = {
	.constraints = {
		.min_uV = 2800000,
		.max_uV = 3500000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(leopard_ldo4_consumers),
	.consumer_supplies = leopard_ldo4_consumers,
};


/* LDO5: Sensor 2.8V */
 struct regulator_consumer_supply leopard_ldo5_consumers[] = {
	{
		.supply = "sensor28",
	}
};

 struct regulator_init_data leopard_ldo5 = {
	.constraints = {
		.min_uV = 2600000,
		.max_uV = 3300000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(leopard_ldo5_consumers),
	.consumer_supplies = leopard_ldo5_consumers,
};

/* LDO6: AVDD_5201, SATA, MIPI */
 struct regulator_consumer_supply leopard_avddcore_consumers[] = {
	{
		.supply = "avddcore",
	},
	{
		.supply = "sata",
	},
	{
		.supply = "mipi",
	},
};

 struct regulator_init_data leopard_avddcore = {
	.constraints = {
		.name = "avddcore",
		.min_uV = 700000,
		.max_uV = 1400000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies = ARRAY_SIZE(leopard_avddcore_consumers),
	.consumer_supplies = leopard_avddcore_consumers,
};

/* LDO7: WIFI 1.8v */
struct regulator_consumer_supply leopard_ldo7_consumers[] = {
	{
		.supply = "wifi18",
	}
};

 struct regulator_init_data leopard_ldo7 = {
	.constraints = {
		.min_uV = 1500000,
		.max_uV = 2000000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(leopard_ldo7_consumers),
	.consumer_supplies = leopard_ldo7_consumers,
};

/* LDO8: FM, GPS */
 struct regulator_consumer_supply leopard_ldo8_consumers[] = {
	{
		.supply = "fm",
	},
	{
		.supply = "gps",
	}
};

 struct regulator_init_data leopard_ldo8 = {
	.constraints = {
		.min_uV = 2300000,
		.max_uV = 3300000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.boot_on = 1,
	},
	.num_consumer_supplies = ARRAY_SIZE(leopard_ldo8_consumers),
	.consumer_supplies = leopard_ldo8_consumers,
};

/* LDO9: WIFI 1.2V */
 struct regulator_consumer_supply leopard_ldo9_consumers[] = {
	{
		.supply = "wifi12",
	}
};

 struct regulator_init_data leopard_ldo9 = {
	.constraints = {
		.min_uV = 1000000,
		.max_uV = 1500000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(leopard_ldo9_consumers),
	.consumer_supplies = leopard_ldo9_consumers,
};

/* LDO10: SATA 2.5V */
 struct regulator_consumer_supply leopard_ldo10_consumers[] = {
	{
		.supply = "sata25",
	}
};

 struct regulator_init_data leopard_ldo10 = {
	.constraints = {
		.min_uV = 2300000,
		.max_uV = 3300000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(leopard_ldo10_consumers),
	.consumer_supplies = leopard_ldo10_consumers,
};

/* LDO11: SVCC */
 struct regulator_consumer_supply leopard_svcc_consumers[] = {
	{
		.supply = "svcc",
	}
};

 struct regulator_init_data leopard_svcc = {
	.constraints = {
		.name = "svcc",
		.min_uV = 2600000,
		.max_uV = 3300000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies = ARRAY_SIZE(leopard_svcc_consumers),
	.consumer_supplies = leopard_svcc_consumers,
};


/* Switch LDO1: SD/MMC */
 struct regulator_consumer_supply leopard_switch1_ldo_consumers[] = {
	{
		.supply = "sd_vcc",
	}
};

 struct regulator_init_data leopard_switch1_ldo = {
	.constraints = {
		.min_uV = 3000000,
		.max_uV = 3300000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(leopard_switch1_ldo_consumers),
	.consumer_supplies = leopard_switch1_ldo_consumers,
};

/* Battery */
/**
vbat_charge_start 电池开始充电的最低门限电压
vbat_charge_stop　电池充电完成的门限电压
vbat_low　　　　　低电警告电压
vbat_crit　　　　　自动关机电压
trickle_charge_current　　涓流充电电流
constant_charge_current　恒流充电电流
**/
struct atc260x_battery_pdata leopard_battery_info = {
	.vbat_charge_start = 4100,
	.vbat_charge_stop = 4200,
	.vbat_low = 3600,
	.vbat_crit = 3400,

	.trickle_charge_current = 100,
	.constant_charge_current = 1000,

	.batmon_interval = 0,

	.battery_low = NULL,
	.battery_critical = NULL,
};

/* Backup Battery */
struct atc260x_backup_battery_pdata leopard_backup_battery_info = {
	.vbat_charge_start = 4100,
	.vbat_charge_stop = 4200,

	.batmon_interval = 0,
};

/* uart2 for 3G module */
struct asoc_uart_platform_data uart2_platform_data = {
    .use_dma_tx = 1,
    .use_dma_rx = 1,
    .rx_poll_rate = 10, /* 10ms */
    .rx_timeout = 100,  /* 100ms */
};

/* uart3 for bluetooth */
struct asoc_uart_platform_data uart3_platform_data = {
    .use_dma_tx = 1,
    .use_dma_rx = 1,
    .rx_poll_rate = 10, /* 10ms */
    .rx_timeout = 100,  /* 100ms */
};

/* uart5 for console */
struct asoc_uart_platform_data uart5_platform_data = {
    .use_dma_tx = 0,
    .use_dma_rx = 0,
};

/* UART2 */
static struct resource asoc_res_uart2[] = {
    {
        .start = UART2_BASE,
        .end = UART2_BASE + 0xfff,
        .flags = IORESOURCE_MEM,
    },
    {
        .start = IRQ_ASOC_UART2,
        .end = IRQ_ASOC_UART2,
        .flags = IORESOURCE_IRQ,
    },
};

struct platform_device asoc_uart_device2 = {
    .name = "asoc-serial",
    .id = 2,
    .resource = asoc_res_uart2,
    .num_resources = ARRAY_SIZE(asoc_res_uart2),
	.dev	= {
		.platform_data = &uart2_platform_data,
	}
};

/* UART3 */
static struct resource asoc_res_uart3[] = {
    {
        .start = UART3_BASE,
        .end = UART3_BASE + 0xfff,
        .flags = IORESOURCE_MEM,
    },
    {
        .start = IRQ_ASOC_UART3,
        .end = IRQ_ASOC_UART3,
        .flags = IORESOURCE_IRQ,
    },
};

struct platform_device asoc_uart_device3 = {
    .name = "asoc-serial",
    .id = 3,
    .resource = asoc_res_uart3,
    .num_resources = ARRAY_SIZE(asoc_res_uart3),
	.dev	= {
		.platform_data = &uart3_platform_data,
	}
};

/* UART5 */
static struct resource asoc_res_uart5[] = {
    {
        .start = UART5_BASE,
        .end = UART5_BASE + 0xfff,
        .flags = IORESOURCE_MEM,
    },
    {
        .start = IRQ_ASOC_UART5,
        .end = IRQ_ASOC_UART5,
        .flags = IORESOURCE_IRQ,
    },
};

struct platform_device asoc_uart_device5 = {
    .name = "asoc-serial",
    .id = 5,
    .resource = asoc_res_uart5,
    .num_resources = ARRAY_SIZE(asoc_res_uart5),
	.dev	= {
		.platform_data = &uart5_platform_data,
	}
};

static struct resource asoc_mmc0_resources[] = {
	[0] = {
		.start  = SD0_BASE,
		.end    = SD0_BASE + 0x38,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
        .start  = IRQ_ASOC_SD0,
        .end  = IRQ_ASOC_SD0,
        .flags  = IORESOURCE_IRQ,
	},
	[3] = {
        .start  = IRQ_ASOC_SIRQ0,
        .end  = IRQ_ASOC_SIRQ0,
        .flags  = IORESOURCE_IRQ,
	},
	[4] = {
        .start  = IRQ_ASOC_SIRQ1,
        .end  = IRQ_ASOC_SIRQ1,
        .flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device asoc_mmc0_device = {
	.name =  "asoc-mmc0",
	.id = 0,
	.num_resources	= ARRAY_SIZE(asoc_mmc0_resources),
	.resource	= asoc_mmc0_resources,
};

static struct resource asoc_mmc1_resources[] = {
	[0] = {
		.start  = SD1_BASE,
		.end    = SD1_BASE + 0x38,
		.flags  = IORESOURCE_MEM,
	},
  [1] = {
        .start  = IRQ_ASOC_SD1,
        .end  = IRQ_ASOC_SD1,
        .flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device asoc_mmc1_device = {
	.name = "asoc-mmc1",
	.id = 0,
	.num_resources	= ARRAY_SIZE(asoc_mmc1_resources),
	.resource	= asoc_mmc1_resources,
};

static struct resource asoc_pmu_resource[] = {
	[0] = {
		.start	= IRQ_ASOC_PC0,
		.end	= IRQ_ASOC_PC0,
		.flags	= IORESOURCE_IRQ,
	},
	[1] = {
		.start	= IRQ_ASOC_PC1,
		.end	= IRQ_ASOC_PC1,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {	
		.start	= IRQ_ASOC_PC2,
		.end	= IRQ_ASOC_PC2,
		.flags	= IORESOURCE_IRQ,	
	},
};

struct platform_device asoc_pmu_device = {
    .name = "arm-pmu",
    .id = 0,
    .resource = asoc_pmu_resource,
    .num_resources = ARRAY_SIZE(asoc_pmu_resource),
};

#include <mach/leopard_thermal.h>
static struct leopard_tmu_platform_data leopard_tmu_data = {
	.trigger_levels[0] = 90,
	.trigger_levels[1] = 140,
	.trigger_levels[2] = 150,
	.trigger_level0_en = 1,
	.trigger_level1_en = 1,
	.trigger_level2_en = 1,
	.trigger_level3_en = 0,
	.freq_tab[0] = {
		.freq_clip_max = 600 * 1000,
		.temp_level = 90,
		.vdd_clip_max = 1050*1000,
	},
	.freq_tab[1] = {
		.freq_clip_max = 600 * 1000,
		.temp_level = 100,
		.vdd_clip_max = 1025*1000,
	},
	.freq_tab[2] = {
		.freq_clip_max = 600 * 1000,
		.temp_level = 140,
		.vdd_clip_max = 1000*1000,
	},
	// .freq_tab[3] = {
		// .freq_clip_max = 600 * 1000,
		// .temp_level = 110,
		// .vdd_clip_max = 1025*1000,
	// },
	.size[THERMAL_TRIP_ACTIVE] = 1,
	.size[THERMAL_TRIP_PASSIVE] = 1,//2,
	.size[THERMAL_TRIP_HOT] = 1,
	.freq_tab_count = 3,//4,
};

struct platform_device asoc_tmu_device = {
    .name = "leopard-tmu",
    .id = 0,
	.dev	= {
		.platform_data = &leopard_tmu_data,
	}	
};

/* wlan pm device */
static struct platform_device wlan_pm_device = {
        .name           ="wlan_pm_dev",
        .id             = 0,
};

static struct platform_device *asoc_platform_devices[] __initdata = {
    &asoc_uart_device2,
    &asoc_uart_device3,
    &asoc_uart_device5,

    &asoc_mmc0_device,
    &asoc_mmc1_device,
    &asoc_pmu_device,
	&asoc_tmu_device,
    &wlan_pm_device,
};

/* ATC260x platform data */
 struct atc260x_pdata leopard_fpga_atc260x_pdata = {
    .irq_base = IRQ_ATC260X_BASE,
     .dcdc = {
        &leopard_vddcore,    /* DCDC1 */
        &leopard_vddr,       /* DCDC2 */
        &leopard_vcc,        /* DCDC3 */
        &leopard_vcc5v,      /* DCDC4 */
    },
    .ldo = {
         &leopard_avcccore,   /* LDO1 */
         &leopard_avccpmic,   /* LDO2 */
         &leopard_vddpmic,    /* LDO3 */
         &leopard_ldo4,       /* LDO4 */
         &leopard_ldo5,       /* LDO5 */
         &leopard_avddcore,   /* LDO6 */
         &leopard_ldo7,       /* LDO7 */
         &leopard_ldo8,       /* LDO8 */
         &leopard_ldo9,       /* LDO9 */
         &leopard_ldo10,      /* LDO10 */
         &leopard_svcc,       /* LDO11 */
    },
    .switch_ldo = {
        &leopard_switch1_ldo, /* Switch LDO1 */
    },
   .battery = &leopard_battery_info,
   .backup_battery = &leopard_backup_battery_info,
};

static struct spi_board_info asoc_spi_board_info[] __initdata = {
	{
		.modalias = "atc260x",
		.max_speed_hz = 2000000,       /* max spi clock (SCK) speed in HZ */
		.bus_num = 1,
		.chip_select = 0,
        .mode = SPI_MODE_3,
        .irq = IRQ_SIRQ2,               /* IRQ number for ATC260x*/
        .platform_data = &leopard_fpga_atc260x_pdata,
	},
};

#ifdef CONFIG_PINCTRL_ASOC

#define PINMUX_DEV "pinctrl-asoc"

/* We use these to define hog settings that are always done on boot */
#define ASOC_MUX_HOG(group, func) \
    PIN_MAP_MUX_GROUP_HOG_DEFAULT(PINMUX_DEV, group, func)
#define ASOC_PIN_HOG(pin, conf) \
    PIN_MAP_CONFIGS_PIN_HOG_DEFAULT(PINMUX_DEV, pin, conf)

/* These are default states associated with device and changed runtime */
#define ASOC_MUX_DEV(group, func, dev) \
    PIN_MAP_MUX_GROUP_DEFAULT(dev, PINMUX_DEV, group, func)
#define ASOC_PIN_DEV(pin, conf, dev) \
    PIN_MAP_CONFIGS_PIN_DEFAULT(dev, PINMUX_DEV, pin, conf)

/* Pin control settings */
#if 0
static struct pinctrl_map __initdata leopardfpga_pinmux_map[] = {
    /* enable at boot stage */
    ASOC_MUX_HOG(NULL, "i2c0"),
    ASOC_MUX_HOG(NULL, "lcd0"),
    ASOC_MUX_HOG(NULL, "spi0"),
    ASOC_MUX_HOG(NULL, "sd0"),
    ASOC_MUX_HOG(NULL, "sens0"),
    ASOC_MUX_HOG(NULL, "uart0"),

    ASOC_MUX_HOG(NULL, "spi0"),

    /* choose gps pins from i2s pads */
    ASOC_MUX_HOG("gps_i2s", "gps"),

    /* matrix Key 4 * 2, for atc_keymx device */
    ASOC_MUX_DEV("key4x2", "key", "atc_keymx"),
};
#endif

unsigned int nmaps;
struct pinctrl_map *leopardfpga_pinmux_map;

//#define COMPARE_RESULT
#ifdef COMPARE_RESULT
struct pinctrl_map __initdata leopardfpga_pinmux_map_ori[] = {
    /* enable at boot stage */
    ASOC_MUX_HOG(NULL, "lvds"),
    ASOC_MUX_HOG(NULL, "spi1"),
//    ASOC_MUX_HOG(NULL, "sd0"),
    ASOC_MUX_HOG(NULL, "sens1"),

    /* pwm */
    ASOC_MUX_HOG("pwm01_ksin02", "pwm"),
    
    /* uart2 */
    ASOC_MUX_HOG(NULL, "uart2"),
       /* sd0 pin switch*/
    PIN_MAP_MUX_GROUP("asoc-mmc0.0","sd0_host",PINMUX_DEV,"sd0","sd0"),
    PIN_MAP_MUX_GROUP("asoc-mmc0.0","serial_port",PINMUX_DEV,"uart5_sd0","uart5"),

    /* sd1 pin*/
    PIN_MAP_MUX_GROUP("asoc-mmc1.0","sd1_wifi",PINMUX_DEV,"sd1","sd1"),
#if 1
    /* uart5 from sd0 */
//    ASOC_MUX_HOG("uart5_sd0", "uart5"),
#else
    /* uart5 from ks pads */
//    ASOC_MUX_HOG("uart5_ks", "uart5"),
#endif
};
#endif

static struct platform_device asoc_pinctrl_device = {
	.name = "pinctrl-asoc",
	.id = -1,
};

static struct platform_device *devices[] = {
	&asoc_pinctrl_device,
};

#ifdef COMPARE_RESULT    
void dump_pinctrl_mappings_ori(void)
{
	int ret=0;
	int i;
	
	printk("-------leopardfpga_pinmux_map_ori:%d-------\n", ARRAY_SIZE(leopardfpga_pinmux_map_ori));
	for(i=0; i<ARRAY_SIZE(leopardfpga_pinmux_map_ori); i++)
	{
		printk("----------leopardfpga_pinmux_map_ori[%d]----------\n", i);
		printk("dev_name:%s\n", leopardfpga_pinmux_map_ori[i].dev_name);
		printk("name:%s\n", leopardfpga_pinmux_map_ori[i].name);
		printk("type:%d\n", leopardfpga_pinmux_map_ori[i].type);
		printk("ctrl_dev_name:%s\n", leopardfpga_pinmux_map_ori[i].ctrl_dev_name);
		printk("group:%s\n", leopardfpga_pinmux_map_ori[i].data.mux.group);
		printk("function:%s\n", leopardfpga_pinmux_map_ori[i].data.mux.function);
	}	
}
#endif

void __init asoc_board_pinmux_init(void)
{
    printk("asoc_board_pinmux_init()\n");

	pinctrl_register_mappings(leopardfpga_pinmux_map, nmaps);

    platform_add_devices(devices, ARRAY_SIZE(devices));
}
#else
void __init asoc_board_pinmux_init(void) {}
#endif  /* CONFIG_PINCTRL_ASOC */

extern __init int leopard_cmu_init_early(void);

void __init leopard_init_early(void)
{
	leopard_cmu_init_early();
}

static int __init asoc_uart_platform_data_init(void)
{
    int ret;
    unsigned int use_dma_tx, use_dma_rx;
    unsigned int rx_poll_rate, rx_timeout;

    printk("asoc_uart_platform_data_init()\n");

    ret = get_config("uart2_port.use_dma_tx", (char *)(&use_dma_tx), sizeof(unsigned int));
    if (ret == 0) {
        printk("uart2_port.use_dma_tx: %d\n", use_dma_tx);
        uart2_platform_data.use_dma_tx = use_dma_tx;
    }

    ret = get_config("uart2_port.use_dma_rx", (char *)(&use_dma_rx), sizeof(unsigned int));
    if (ret == 0) {
        printk("uart2_port.use_dma_rx: %d\n", use_dma_rx);
        uart2_platform_data.use_dma_rx = use_dma_rx;
    }

    ret = get_config("uart2_port.rx_poll_rate", (char *)(&rx_poll_rate), sizeof(unsigned int));
    if (ret == 0) {
        printk("uart2_port.rx_poll_rate: %d\n", rx_poll_rate);
        uart2_platform_data.rx_poll_rate = rx_poll_rate;
    }

    ret = get_config("uart2_port.rx_timeout", (char *)(&rx_timeout), sizeof(unsigned int));
    if (ret == 0) {
        printk("uart2_port.rx_timeout: %d\n", rx_timeout);
        uart2_platform_data.rx_timeout = rx_timeout;
    }


    ret = get_config("uart3_port.use_dma_tx", (char *)(&use_dma_tx), sizeof(unsigned int));
    if (ret == 0) {
        printk("uart3_port.use_dma_tx: %d\n", use_dma_tx);
        uart3_platform_data.use_dma_tx = use_dma_tx;
    }

    ret = get_config("uart3_port.use_dma_rx", (char *)(&use_dma_rx), sizeof(unsigned int));
    if (ret == 0) {
        printk("uart3_port.use_dma_rx: %d\n", use_dma_rx);
        uart3_platform_data.use_dma_rx = use_dma_rx;
    }

    ret = get_config("uart3_port.rx_poll_rate", (char *)(&rx_poll_rate), sizeof(unsigned int));
    if (ret == 0) {
        printk("uart3_port.rx_poll_rate: %d\n", rx_poll_rate);
        uart3_platform_data.rx_poll_rate = rx_poll_rate;
    }

    ret = get_config("uart3_port.rx_timeout", (char *)(&rx_timeout), sizeof(unsigned int));
    if (ret == 0) {
        printk("uart3_port.rx_timeout: %d\n", rx_timeout);
        uart3_platform_data.rx_timeout = rx_timeout;
    }

    return 0;
}

#include <mach/dvfslevel.h>

struct leopard_dvfslevel_vdd_clip_max_table {
    int dvfslevel;
    unsigned int vdd_clip_max;
};

static struct leopard_dvfslevel_vdd_clip_max_table leopard_vdd_clip_max[] = {
    {ATM7021_L_1, 1000000},
    {ATM7021_L_2,  975000},
    {ATM7021_L_3, 1050000},
    /* version C */
    {ATM7029_L_21, 1000000},
    {ATM7029_L_22,  975000},
    {ATM7029_L_23,  950000},
	    

};

int update_vdd_clip_max(int dvfslevel)
{
    struct leopard_dvfslevel_vdd_clip_max_table* p_cur = NULL;
    int i=0;
    int len = ARRAY_SIZE(leopard_vdd_clip_max);

    for(i=0; i<len; i++)
    {
        p_cur = &(leopard_vdd_clip_max[i]);
        if(p_cur->dvfslevel == dvfslevel)
        {
			leopard_tmu_data.freq_tab[0].vdd_clip_max = p_cur->vdd_clip_max;
			printk("vdd_clip_max:%d\n", p_cur->vdd_clip_max);
			return 0;
        }
    }
    
    printk("[%s] error at dvfslevel:%d\n", __FUNCTION__, dvfslevel);
    return -1;    
}

static int __init asoc_tmu_platform_data_init(void)
{
    int ret;
    unsigned int trigger_temp;

    printk("asoc_tmu_platform_data_init()\n");

    ret = get_config("tmu.trigger_temp", (char *)(&trigger_temp), sizeof(unsigned int));
    if (ret == 0) {
        printk("tmu.trigger_temp: %d\n", trigger_temp);
		leopard_tmu_data.trigger_levels[0] = trigger_temp;
    }
	printk("leopard_tmu_data.trigger_levels[0]: %d\n", leopard_tmu_data.trigger_levels[0]);

	//ToDo:set vdd_clip_max accroding to dvfslevel
	update_vdd_clip_max(asoc_get_dvfslevel());
    return 0;
}

#define I2C_NUM   0

int enable_readahead;
int get_readahead_flag(void)
{
	int ret;

	ret = get_config("readadump.enable", (char *)(&enable_readahead), sizeof(int));
	if (ret == 0)
        printk("readadump.enable: %d\n", enable_readahead);
	return ret;
}

static void __init leopard_board_init(void)
{
    int ret;

    printk("%s()\n", __FUNCTION__);

    get_readahead_flag();

#ifdef COMPARE_RESULT    
	dump_pinctrl_mappings_ori();
#endif
	
    asoc_board_pinmux_init();

    asoc_uart_platform_data_init();
	asoc_tmu_platform_data_init();

    ret = platform_add_devices(asoc_platform_devices,
                   ARRAY_SIZE(asoc_platform_devices));

    asoc_gpio_init();

    printk("asoc_spi_devices_init()\n");
    spi_register_board_info(asoc_spi_board_info, ARRAY_SIZE(asoc_spi_board_info));
}

MACHINE_START(LEOPARD_EVB, "gs702c")
    /* Maintainer: ARM Ltd/Deep Blue Solutions Ltd */
    .atag_offset    = 0x00000100,
    .init_early     = leopard_init_early,
    .map_io         = leopard_map_io,
    .fixup          = leopard_fixup,
    .reserve        = leopard_reserve,
    .init_irq       = gic_init_irq,
    .handle_irq     = gic_handle_irq,
    .init_machine   = leopard_board_init,
    .timer          = &leopard_timer,
MACHINE_END
