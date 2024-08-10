/*
 * arch/arm/mach-leopard/include/mach/bootdev.h
 *
 * Boot device
 *
 * Copyright 2012 Actions Semi Inc.
 * Author: Actions Semi, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __ASM_ARCH_BOOTDEV_H
#define __ASM_ARCH_BOOTDEV_H

#define ASOC_BOOTDEV_NAND       (0x00)
#define ASOC_BOOTDEV_SD0        (0x20)
#define ASOC_BOOTDEV_SD1        (0x21)
#define ASOC_BOOTDEV_SD2        (0x22)
#define ASOC_BOOTDEV_SD02NAND   (0x30)   //nand for cardburn 
#define ASOC_BOOTDEV_SD02SD2    (0x31)	 //emmc for cardburn 

/*
 * get boot device name
 */
extern int asoc_get_boot_dev(void);

#endif /* __ASM_ARCH_BOOTDEV_H */
