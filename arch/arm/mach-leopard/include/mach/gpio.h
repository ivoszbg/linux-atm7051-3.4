/*
 * arch/arm/mach-leopard/include/mach/gpio.h
 *
 * GPIO definitions
 *
 * Copyright 2012 Actions Semi Inc.
 * Author: Actions Semi, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __ASM_ARCH_GPIO_H
#define __ASM_ARCH_GPIO_H

#define ASOC_GPIO_BANKS         4
#define ASOC_GPIO_PER_BANK      32

/* GPIOA/B/C, GPIOD0~21 */
#define ASOC_NR_GPIO            (3 * 32 + 22)

#define ASOC_GPIO_PORTA(x)      ((x) + ASOC_GPIO_PER_BANK * 0)
#define ASOC_GPIO_PORTB(x)      ((x) + ASOC_GPIO_PER_BANK * 1)
#define ASOC_GPIO_PORTC(x)      ((x) + ASOC_GPIO_PER_BANK * 2)
#define ASOC_GPIO_PORTD(x)      ((x) + ASOC_GPIO_PER_BANK * 3)

#define ASOC_GPIO_PORT(iogroup, pin_num) ((pin_num) + ASOC_GPIO_PER_BANK * (iogroup))

extern int asoc_gpio_init(void);

/**
 * struct icpad - ic pad information
 * @pad_index: pad index for gpio
 * @pre_cfg: pre-configuration
 * @pad_name: name of pad
 * @status:
 * @dev: currently belongs to which device
 */
struct gpio_pre_cfg
{
    unsigned char iogroup;          // A, B, C, D, etc
    unsigned char pin_num;          // 0-31
    unsigned char gpio_dir;         // INPUT/OUTPUT
    unsigned char reserved_0;       // the initialize value
    unsigned char active_level;     // active level
    unsigned char reserved_1;   // unactive level
    unsigned char reserved[7];
    char name[64];
} __attribute__ ((packed));

extern int gpio_get_pre_cfg(char *gpio_name, struct gpio_pre_cfg *m_gpio);

#endif /* __ASM_ARCH_GPIO_H */

