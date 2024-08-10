/*
 * atc260x-mfp.c  --  Multifunction Pad interface for ATC260X PMIC
 *
 * Copyright 2011 Actions Semi Inc.
 * Author: Actions Semi, Inc.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <mach/atc260x/atc260x.h>
#include <mach/atc260x/atc260x_pdata.h>
#include <mach/atc260x/atc260x_dev.h>


int atc260x_mfp_lock (enum atc260x_mfp_mod_id mod_id, int opt, struct device *dev)
{
    return 0;
}
EXPORT_SYMBOL_GPL(atc260x_mfp_lock);

int atc260x_mfp_locked (enum atc260x_mfp_mod_id mod_id, int opt)
{
    return 0;
}
EXPORT_SYMBOL_GPL(atc260x_mfp_locked);

int atc260x_mfp_unlock (enum atc260x_mfp_mod_id mod_id, int opt)
{
    return 0;
}
EXPORT_SYMBOL_GPL(atc260x_mfp_unlock);


int atc260x_mfp_init(struct atc260x_dev *atc260x)
{
	return 0;
}

void atc260x_mfp_exit(struct atc260x_dev *atc260x)
{
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Actions Semi, Inc");
