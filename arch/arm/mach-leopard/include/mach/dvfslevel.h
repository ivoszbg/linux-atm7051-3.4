/*
 * arch/arm/mach-leopard/include/mach/dvfslevel.h
 *
 * Actions dvfs level
 *
 * Copyright 2012 Actions Semi Inc.
 * Author: Actions Semi, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __ASM_ARCH_DVFSLEVEL_H
#define __ASM_ARCH_DVFSLEVEL_H

#define DVFSLEVEL_MAGIC    0x47739582
#define ASOC_DVFSLEVEL(ic, version, level)   \
    (((((ic) & 0xffff) << 12) | (((version) & 0xf)) << 6 | ((level) & 0xf)) ^ DVFSLEVEL_MAGIC)

#define ASOC_GET_IC(dvfslevel)         ((((dvfslevel) ^ DVFSLEVEL_MAGIC) >> 12) & 0xffff)
#define ASOC_GET_VERSION(dvfslevel)    ((((dvfslevel) ^ DVFSLEVEL_MAGIC) >> 6) & 0xf)
#define ASOC_GET_LEVEL(dvfslevel)       (((dvfslevel) ^ DVFSLEVEL_MAGIC) & 0xf)

/* version A */
#define ATM7021_L_1              ASOC_DVFSLEVEL(0x7021, 0x0, 0x01)
#define ATM7021_L_2              ASOC_DVFSLEVEL(0x7021, 0x0, 0x02)
#define ATM7021_L_3              ASOC_DVFSLEVEL(0x7021, 0x0, 0x03)

/* version C */
#define ATM7029_L_21             ASOC_DVFSLEVEL(0x7029, 0x2, 0x01)
#define ATM7029_L_22             ASOC_DVFSLEVEL(0x7029, 0x2, 0x02)
#define ATM7029_L_23             ASOC_DVFSLEVEL(0x7029, 0x2, 0x03)

/* get dvfsleve */
extern int asoc_get_dvfslevel(void);
extern int asoc_get_icversion(void);

#endif /* __ASM_ARCH_DVFSLEVEL_H */
