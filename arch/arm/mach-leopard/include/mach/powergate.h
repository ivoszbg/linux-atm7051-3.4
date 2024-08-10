/*
 * arch/arm/mach-leopard/include/mach/powergate.h
 *
 * powergate definitions
 *
 * Copyright 2012 Actions Semi Inc.
 * Author: Actions Semi, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __ASM_ARCH_POWERGATE_H
#define __ASM_ARCH_POWERGATE_H

enum asoc_powergate_id {
    ASOC_POWERGATE_CPU1 = 0,
    ASOC_POWERGATE_CPU2,
    ASOC_POWERGATE_GPU,
    ASOC_POWERGATE_RESERVED3,
    ASOC_POWERGATE_VCE,
    ASOC_POWERGATE_VDE,
    ASOC_POWERGATE_RESERVED6,
    ASOC_POWERGATE_RESERVED7,
    ASOC_POWERGATE_CPU3,
    ASOC_POWERGATE_MAXID,
};

int  __init asoc_powergate_init(void);

int asoc_cpu_powergate_id(int cpuid);
int asoc_powergate_is_powered(enum asoc_powergate_id id);
int asoc_powergate_power_on(enum asoc_powergate_id id);
int asoc_powergate_power_off(enum asoc_powergate_id id);
int asoc_powergate_power_on_l2(void);

#endif /* __ASM_ARCH_POWERGATE_H */
