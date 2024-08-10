/*
 * arch/arm/mach-leopard/pinctrl-asoc.h
 *
 * Pinctrl definitions for Actions SOC
 *
 * Copyright 2012 Actions Semi Inc.
 * Author: Actions Semi, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */
 
#ifndef __PINCTRL_ASOC_H__ 
#define __PINCTRL_ASOC_H__

#include <linux/platform_device.h>

/* Package definitions */
#define PINCTRL_NMK_STN8815	0
#define PINCTRL_NMK_DB8500	1

/**
 * struct asoc_regcfgs - describes register config for a pin group
 * @reg: a register for this specific pin group
 * @mask: register mask
 * @val: register value
 */
struct asoc_regcfg {
    unsigned int reg;
    unsigned int mask;
    unsigned int val;
};

/**
 * struct asoc_pinmux_group - describes a Actions SOC pin group
 * @name: the name of this specific pin group
 * @pins: an array of discrete physical pins used in this group, taken
 *	from the driver-local pin enumeration space
 * @num_pins: the number of pins in this group array, i.e. the number of
 *	elements in .pins so we can iterate over that array
 */
struct asoc_pinmux_group {
	const char *name;
	const unsigned int *pins;
	unsigned int npins;

    const struct asoc_regcfg *regcfgs;
	unsigned int nregcfgs;
};

/**
 * struct asoc_pinmux_func - Actions SOC pinctrl mux functions
 * @name: The name of the function, exported to pinctrl core.
 * @groups: An array of pin groups that may select this function.
 * @ngroups: The number of entries in @groups.
 */
struct asoc_pinmux_func {
	const char *name;
	const char * const *groups;
	unsigned ngroups;
};

/**
 * struct asoc_pinctrl_soc_data - Actions SOC pin controller per-SoC configuration
 * @gpio_ranges: An array of GPIO ranges for this SoC
 * @gpio_num_ranges: The number of GPIO ranges for this SoC
 * @pins:	An array describing all pins the pin controller affects.
 *		All pins which are also GPIOs must be listed first within the
 *		array, and be numbered identically to the GPIO controller's
 *		numbering.
 * @npins:	The number of entries in @pins.
 * @functions:	The functions supported on this SoC.
 * @nfunction:	The number of entries in @functions.
 * @groups:	An array describing all pin groups the pin SoC supports.
 * @ngroups:	The number of entries in @groups.
 */
struct asoc_pinctrl_soc_info {
	struct device *dev;
	struct pinctrl_gpio_range *gpio_ranges;
	unsigned gpio_num_ranges;
	const struct pinctrl_pin_desc *pins;
	unsigned npins;
	const struct asoc_pinmux_func *functions;
	unsigned nfunctions;
	const struct asoc_pinmux_group *groups;
	unsigned ngroups;
};

int __devinit asoc_pinctrl_probe(struct platform_device *pdev,
				struct asoc_pinctrl_soc_info *info);
int __devexit asoc_pinctrl_remove(struct platform_device *pdev);

#endif /* __PINCTRL_ASOC_H__ */
