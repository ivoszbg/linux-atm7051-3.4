/*
 * arch/arm/mach-leopard/pinctrl-leopard.c
 *
 * Pinctrl driver based on Actions SOC pinctrl
 *
 * Copyright 2012 Actions Semi Inc.
 * Author: Actions Semi, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <mach/dvfslevel.h>
#include "pinctrl-asoc.h"

#define DRIVER_NAME "pinctrl-asoc"

/*
 * Most pins affected by the pinmux can also be GPIOs. Define these first.
 * These must match how the GPIO driver names/numbers its pins.
 */
#define _GPIOA(offset)          (offset)
#define _GPIOB(offset)          (32 + (offset))
#define _GPIOC(offset)          (64 + (offset))
#define _GPIOD(offset)          (96 + (offset))

/* All non-GPIO pins follow */
#define NUM_GPIOS               (_GPIOD(21) + 1)
#define _PIN(offset)            (NUM_GPIOS + (offset))

/* I2C */
#define ATM7021_P_I2C1_SCLK             _GPIOA(8)
#define ATM7021_P_I2C1_SDATA            _GPIOA(9)
#define ATM7021_P_I2C2_SCLK             _GPIOA(10)
#define ATM7021_P_I2C2_SDATA            _GPIOA(11)
#define ATM7021_P_I2C0_SCLK             _GPIOA(12)
#define ATM7021_P_I2C0_SDATA            _GPIOA(13)

/* UART */
#define ATM7021_P_UART2_RX              _GPIOA(14)
#define ATM7021_P_UART2_TX              _GPIOA(15)
#define ATM7021_P_UART2_RTSB            _GPIOA(16)
#define ATM7021_P_UART2_CTSB            _GPIOA(17)

#define ATM7021_P_UART3_RX              _GPIOA(18)
#define ATM7021_P_UART3_TX              _GPIOA(19)
#define ATM7021_P_UART3_RTSB            _GPIOA(20)
#define ATM7021_P_UART3_CTSB            _GPIOA(21)

#define ATM7021_P_UART4_RX              _GPIOA(22)
#define ATM7021_P_UART4_TX              _GPIOA(23)

/* SIRQ */
#define ATM7021_P_SIRQ0                 _GPIOA(24)
#define ATM7021_P_SIRQ1                 _GPIOA(25)
#define ATM7021_P_SIRQ2                 _GPIOA(26)

/* I2S */
#define ATM7021_P_I2S_D0                _GPIOA(27)
#define ATM7021_P_I2S_BCLK0             _GPIOA(28)
#define ATM7021_P_I2S_LRCLK0            _GPIOA(29)
#define ATM7021_P_I2S_MCLK0             _GPIOA(30)
#define ATM7021_P_I2S_D1                _GPIOA(31)
#define ATM7021_P_I2S_BCLK1             _GPIOB(0)
#define ATM7021_P_I2S_LRCLK1            _GPIOB(1)
#define ATM7021_P_I2S_MCLK1             _GPIOB(2)

/* KEY */
#define ATM7021_P_KS_IN0                _GPIOB(3)
#define ATM7021_P_KS_IN1                _GPIOB(4)
#define ATM7021_P_KS_IN2                _GPIOB(5)
#define ATM7021_P_KS_IN3                _GPIOB(6)
#define ATM7021_P_KS_OUT0               _GPIOB(7)
#define ATM7021_P_KS_OUT1               _GPIOB(8)
#define ATM7021_P_KS_OUT2               _GPIOB(9)

/* LCD0 */
#define ATM7021_P_LVDS_OEP              _GPIOB(10)
#define ATM7021_P_LVDS_OEN              _GPIOB(11)
#define ATM7021_P_LVDS_ODP              _GPIOB(12)
#define ATM7021_P_LVDS_ODN              _GPIOB(13)
#define ATM7021_P_LVDS_OCP              _GPIOB(14)
#define ATM7021_P_LVDS_OCN              _GPIOB(15)
#define ATM7021_P_LVDS_OBP              _GPIOB(16)
#define ATM7021_P_LVDS_OBN              _GPIOB(17)
#define ATM7021_P_LVDS_OAP              _GPIOB(18)
#define ATM7021_P_LVDS_OAN              _GPIOB(19)
#define ATM7021_P_LVDS_EEP              _GPIOB(20)
#define ATM7021_P_LVDS_EEN              _GPIOB(21)
#define ATM7021_P_LVDS_EDP              _GPIOB(22)
#define ATM7021_P_LVDS_EDN              _GPIOB(23)
#define ATM7021_P_LVDS_ECP              _GPIOB(24)
#define ATM7021_P_LVDS_ECN              _GPIOB(25)
#define ATM7021_P_LVDS_EBP              _GPIOB(26)
#define ATM7021_P_LVDS_EBN              _GPIOB(27)
#define ATM7021_P_LVDS_EAP              _GPIOB(28)
#define ATM7021_P_LVDS_EAN              _GPIOB(29)
#define ATM7021_P_LCD0_D18              _GPIOB(30)
#define ATM7021_P_LCD0_D17              _GPIOB(31)
#define ATM7021_P_LCD0_D16              _GPIOC(0)
#define ATM7021_P_LCD0_D9               _GPIOC(1)
#define ATM7021_P_LCD0_D8               _GPIOC(2)
#define ATM7021_P_LCD0_D2               _GPIOC(3)
#define ATM7021_P_LCD0_D1               _GPIOC(4)
#define ATM7021_P_LCD0_D0               _GPIOC(5)

/* SD */
#define ATM7021_P_SD0_D0                _GPIOC(10)
#define ATM7021_P_SD0_D1                _GPIOC(11)
#define ATM7021_P_SD0_D2                _GPIOC(12)
#define ATM7021_P_SD0_D3                _GPIOC(13)
#define ATM7021_P_SD0_D4                _GPIOC(14)
#define ATM7021_P_SD0_D5                _GPIOC(15)
#define ATM7021_P_SD0_D6                _GPIOC(16)
#define ATM7021_P_SD0_D7                _GPIOC(17)
#define ATM7021_P_SD0_CMD               _GPIOC(18)
#define ATM7021_P_SD0_CLK               _GPIOC(19)
#define ATM7021_P_SD1_CMD               _GPIOC(20)
#define ATM7021_P_SD1_CLK               _GPIOC(21)

/* SPI */
#define ATM7021_P_SPI0_SCLK             _GPIOC(22)
#define ATM7021_P_SPI0_SS               _GPIOC(23)
#define ATM7021_P_SPI0_MISO             _GPIOC(24)
#define ATM7021_P_SPI0_MOSI             _GPIOC(25)

/* UART0 */
#define ATM7021_P_UART0_RX              _GPIOC(26)
#define ATM7021_P_UART0_TX              _GPIOC(27)

/* Sensor */
#define ATM7021_P_SENS1_VSYNC           _GPIOD(0)
#define ATM7021_P_SENS1_HSYNC           _GPIOD(1)
#define ATM7021_P_SENS1_D0              _GPIOD(2)
#define ATM7021_P_SENS1_D1              _GPIOD(3)
#define ATM7021_P_SENS1_D2              _GPIOD(4)
#define ATM7021_P_SENS1_D3              _GPIOD(5)
#define ATM7021_P_SENS1_D4              _GPIOD(6)
#define ATM7021_P_SENS1_D5              _GPIOD(7)
#define ATM7021_P_SENS1_D6              _GPIOD(8)
#define ATM7021_P_SENS1_D7              _GPIOD(9)
#define ATM7021_P_SENS1_CKOUT           _GPIOD(10)
#define ATM7021_P_SENS1_PCLK            _GPIOD(11)

/* NAND (1.8v / 3.3v) */
#define ATM7021_P_DNAND_D0              _PIN(0)
#define ATM7021_P_DNAND_D1              _PIN(1)
#define ATM7021_P_DNAND_D2              _PIN(2)
#define ATM7021_P_DNAND_D3              _PIN(3)
#define ATM7021_P_DNAND_D4              _PIN(4)
#define ATM7021_P_DNAND_D5              _PIN(5)
#define ATM7021_P_DNAND_D6              _PIN(6)
#define ATM7021_P_DNAND_D7              _PIN(7)
#define ATM7021_P_DNAND_WRB             _PIN(8)
#define ATM7021_P_DNAND_RB              _PIN(9)
#define ATM7021_P_DNAND_DQS             _GPIOC(28)
#define ATM7021_P_DNAND_RDB             _GPIOC(29)
#define ATM7021_P_DNAND_CLE             _GPIOC(30)
#define ATM7021_P_DNAND_ALE             _GPIOC(31)
#define ATM7021_P_DNAND_CEB0            _GPIOC(6)
#define ATM7021_P_DNAND_CEB1            _GPIOC(7)
#define ATM7021_P_DNAND_CEB2            _GPIOC(8)
#define ATM7021_P_DNAND_CEB3            _GPIOC(9)

/* HDMI */
#define ATM7021_P_HDMI_TNCK             _PIN(10)
#define ATM7021_P_HDMI_TPCK             _PIN(11)
#define ATM7021_P_HDMI_TXON0            _PIN(12)
#define ATM7021_P_HDMI_TXOP0            _PIN(13)
#define ATM7021_P_HDMI_TXON1            _PIN(14)
#define ATM7021_P_HDMI_TXOP1            _PIN(15)
#define ATM7021_P_HDMI_TXON2            _PIN(16)
#define ATM7021_P_HDMI_TXOP2            _PIN(17)

/* Pad names for the pinmux subsystem */
static const struct pinctrl_pin_desc atm7021_leopard_pads[] = {
    PINCTRL_PIN(ATM7021_P_I2C0_SCLK, "P_I2C0_SCLK"),
    PINCTRL_PIN(ATM7021_P_I2C0_SDATA, "P_I2C0_SDATA"),
    PINCTRL_PIN(ATM7021_P_I2C1_SCLK, "P_I2C1_SCLK"),
    PINCTRL_PIN(ATM7021_P_I2C1_SDATA, "P_I2C1_SDATA"),
    PINCTRL_PIN(ATM7021_P_I2C2_SCLK, "P_I2C2_SCLK"),
    PINCTRL_PIN(ATM7021_P_I2C2_SDATA, "P_I2C2_SDATA"),

    PINCTRL_PIN(ATM7021_P_UART0_RX, "P_UART0_RX"),
    PINCTRL_PIN(ATM7021_P_UART0_TX, "P_UART0_TX"),
    PINCTRL_PIN(ATM7021_P_UART2_RX, "P_UART2_RX"),
    PINCTRL_PIN(ATM7021_P_UART2_TX, "P_UART2_TX"),
    PINCTRL_PIN(ATM7021_P_UART2_RTSB, "P_UART2_RTSB"),
    PINCTRL_PIN(ATM7021_P_UART2_CTSB, "P_UART2_CTSB"),
    PINCTRL_PIN(ATM7021_P_UART3_RX, "P_UART3_RX"),
    PINCTRL_PIN(ATM7021_P_UART3_TX, "P_UART3_TX"),
    PINCTRL_PIN(ATM7021_P_UART3_RTSB, "P_UART3_RTSB"),
    PINCTRL_PIN(ATM7021_P_UART3_CTSB, "P_UART3_CTSB"),
    PINCTRL_PIN(ATM7021_P_UART4_RX, "P_UART4_RX"),
    PINCTRL_PIN(ATM7021_P_UART4_TX, "P_UART4_TX"),

    PINCTRL_PIN(ATM7021_P_I2S_D0, "P_I2S_D0"),
    PINCTRL_PIN(ATM7021_P_I2S_BCLK0, "P_I2S_BCLK0"),
    PINCTRL_PIN(ATM7021_P_I2S_LRCLK0, "P_I2S_LRCLK0"),
    PINCTRL_PIN(ATM7021_P_I2S_MCLK0, "P_I2S_MCLK0"),
    PINCTRL_PIN(ATM7021_P_I2S_D1, "P_I2S_D1"),
    PINCTRL_PIN(ATM7021_P_I2S_BCLK1, "P_I2S_BCLK1"),
    PINCTRL_PIN(ATM7021_P_I2S_LRCLK1, "P_I2S_LRCLK1"),
    PINCTRL_PIN(ATM7021_P_I2S_MCLK1, "P_I2S_MCLK1"),

    PINCTRL_PIN(ATM7021_P_SIRQ0, "P_SIRQ0"),
    PINCTRL_PIN(ATM7021_P_SIRQ1, "P_SIRQ1"),
    PINCTRL_PIN(ATM7021_P_SIRQ2, "P_SIRQ2"),

    PINCTRL_PIN(ATM7021_P_KS_IN0, "P_KS_IN0"),
    PINCTRL_PIN(ATM7021_P_KS_IN1, "P_KS_IN1"),
    PINCTRL_PIN(ATM7021_P_KS_IN2, "P_KS_IN2"),
    PINCTRL_PIN(ATM7021_P_KS_IN3, "P_KS_IN3"),
    PINCTRL_PIN(ATM7021_P_KS_OUT0, "P_KS_OUT0"),
    PINCTRL_PIN(ATM7021_P_KS_OUT1, "P_KS_OUT1"),
    PINCTRL_PIN(ATM7021_P_KS_OUT2, "P_KS_OUT2"),

    PINCTRL_PIN(ATM7021_P_LVDS_OEP, "P_LVDS_OEP"),
    PINCTRL_PIN(ATM7021_P_LVDS_OEN, "P_LVDS_OEN"),
    PINCTRL_PIN(ATM7021_P_LVDS_ODP, "P_LVDS_ODP"),
    PINCTRL_PIN(ATM7021_P_LVDS_ODN, "P_LVDS_ODN"),
    PINCTRL_PIN(ATM7021_P_LVDS_OCP, "P_LVDS_OCP"),
    PINCTRL_PIN(ATM7021_P_LVDS_OCN, "P_LVDS_OCN"),
    PINCTRL_PIN(ATM7021_P_LVDS_OBP, "P_LVDS_OBP"),
    PINCTRL_PIN(ATM7021_P_LVDS_OBN, "P_LVDS_OBN"),
    PINCTRL_PIN(ATM7021_P_LVDS_OAP, "P_LVDS_OAP"),
    PINCTRL_PIN(ATM7021_P_LVDS_OAN, "P_LVDS_OAN"),
    PINCTRL_PIN(ATM7021_P_LVDS_EEP, "P_LVDS_EEP"),
    PINCTRL_PIN(ATM7021_P_LVDS_EEN, "P_LVDS_EEN"),
    PINCTRL_PIN(ATM7021_P_LVDS_EDP, "P_LVDS_EDP"),
    PINCTRL_PIN(ATM7021_P_LVDS_EDN, "P_LVDS_EDN"),
    PINCTRL_PIN(ATM7021_P_LVDS_ECP, "P_LVDS_ECP"),
    PINCTRL_PIN(ATM7021_P_LVDS_ECN, "P_LVDS_ECN"),
    PINCTRL_PIN(ATM7021_P_LVDS_EBP, "P_LVDS_EBP"),
    PINCTRL_PIN(ATM7021_P_LVDS_EBN, "P_LVDS_EBN"),
    PINCTRL_PIN(ATM7021_P_LVDS_EAP, "P_LVDS_EAP"),
    PINCTRL_PIN(ATM7021_P_LVDS_EAN, "P_LVDS_EAN"),
    PINCTRL_PIN(ATM7021_P_LCD0_D18, "P_LCD0_D18"),
    PINCTRL_PIN(ATM7021_P_LCD0_D17, "P_LCD0_D17"),
    PINCTRL_PIN(ATM7021_P_LCD0_D16, "P_LCD0_D16"),
    PINCTRL_PIN(ATM7021_P_LCD0_D9, "P_LCD0_D9"),
    PINCTRL_PIN(ATM7021_P_LCD0_D8, "P_LCD0_D8"),
    PINCTRL_PIN(ATM7021_P_LCD0_D2, "P_LCD0_D2"),
    PINCTRL_PIN(ATM7021_P_LCD0_D1, "P_LCD0_D1"),
    PINCTRL_PIN(ATM7021_P_LCD0_D0, "P_LCD0_D0"),

    PINCTRL_PIN(ATM7021_P_SD0_D0, "P_SD0_D0"),
    PINCTRL_PIN(ATM7021_P_SD0_D1, "P_SD0_D1"),
    PINCTRL_PIN(ATM7021_P_SD0_D2, "P_SD0_D2"),
    PINCTRL_PIN(ATM7021_P_SD0_D3, "P_SD0_D3"),
    PINCTRL_PIN(ATM7021_P_SD0_D4, "P_SD0_D4"),
    PINCTRL_PIN(ATM7021_P_SD0_D5, "P_SD0_D5"),
    PINCTRL_PIN(ATM7021_P_SD0_D6, "P_SD0_D6"),
    PINCTRL_PIN(ATM7021_P_SD0_D7, "P_SD0_D7"),
    PINCTRL_PIN(ATM7021_P_SD0_CMD, "P_SD0_CMD"),
    PINCTRL_PIN(ATM7021_P_SD0_CLK, "P_SD0_CLK"),
    PINCTRL_PIN(ATM7021_P_SD1_CMD, "P_SD1_CMD"),
    PINCTRL_PIN(ATM7021_P_SD1_CLK, "P_SD1_CLK"),

    PINCTRL_PIN(ATM7021_P_SPI0_SCLK, "P_SPI0_SCLK"),
    PINCTRL_PIN(ATM7021_P_SPI0_SS, "P_SPI0_SS"),
    PINCTRL_PIN(ATM7021_P_SPI0_MISO, "P_SPI0_MISO"),
    PINCTRL_PIN(ATM7021_P_SPI0_MOSI, "P_SPI0_MOSI"),


    PINCTRL_PIN(ATM7021_P_SENS1_PCLK, "P_SENS1_PCLK"),
    PINCTRL_PIN(ATM7021_P_SENS1_VSYNC, "P_SENS1_VSYNC"),
    PINCTRL_PIN(ATM7021_P_SENS1_HSYNC, "P_SENS1_HSYNC"),
    PINCTRL_PIN(ATM7021_P_SENS1_D0, "P_SENS1_D0"),
    PINCTRL_PIN(ATM7021_P_SENS1_D1, "P_SENS1_D1"),
    PINCTRL_PIN(ATM7021_P_SENS1_D2, "P_SENS1_D2"),
    PINCTRL_PIN(ATM7021_P_SENS1_D3, "P_SENS1_D3"),
    PINCTRL_PIN(ATM7021_P_SENS1_D4, "P_SENS1_D4"),
    PINCTRL_PIN(ATM7021_P_SENS1_D5, "P_SENS1_D5"),
    PINCTRL_PIN(ATM7021_P_SENS1_D6, "P_SENS1_D6"),
    PINCTRL_PIN(ATM7021_P_SENS1_D7, "P_SENS1_D7"),
    PINCTRL_PIN(ATM7021_P_SENS1_CKOUT, "P_SENS1_CKOUT"),


    PINCTRL_PIN(ATM7021_P_DNAND_D0, "P_DNAND_D0"),
    PINCTRL_PIN(ATM7021_P_DNAND_D1, "P_DNAND_D1"),
    PINCTRL_PIN(ATM7021_P_DNAND_D2, "P_DNAND_D2"),
    PINCTRL_PIN(ATM7021_P_DNAND_D3, "P_DNAND_D3"),
    PINCTRL_PIN(ATM7021_P_DNAND_D4, "P_DNAND_D4"),
    PINCTRL_PIN(ATM7021_P_DNAND_D5, "P_DNAND_D5"),
    PINCTRL_PIN(ATM7021_P_DNAND_D6, "P_DNAND_D6"),
    PINCTRL_PIN(ATM7021_P_DNAND_D7, "P_DNAND_D7"),
    PINCTRL_PIN(ATM7021_P_DNAND_WRB, "P_DNAND_WRB"),
    PINCTRL_PIN(ATM7021_P_DNAND_RDB, "P_DNAND_RDB"),
    PINCTRL_PIN(ATM7021_P_DNAND_DQS, "P_DNAND_DQS"),
    PINCTRL_PIN(ATM7021_P_DNAND_RB, "P_DNAND_RB"),
    PINCTRL_PIN(ATM7021_P_DNAND_ALE, "P_DNAND_ALE"),
    PINCTRL_PIN(ATM7021_P_DNAND_CLE, "P_DNAND_CLE"),
    PINCTRL_PIN(ATM7021_P_DNAND_CEB0, "P_DNAND_CEB0"),
    PINCTRL_PIN(ATM7021_P_DNAND_CEB1, "P_DNAND_CEB1"),
    PINCTRL_PIN(ATM7021_P_DNAND_CEB2, "P_DNAND_CEB2"),
    PINCTRL_PIN(ATM7021_P_DNAND_CEB3, "P_DNAND_CEB3"),

    PINCTRL_PIN(ATM7021_P_HDMI_TNCK, "P_HDMI_TNCK"),
    PINCTRL_PIN(ATM7021_P_HDMI_TPCK, "P_HDMI_TPCK"),
    PINCTRL_PIN(ATM7021_P_HDMI_TXON0, "P_HDMI_TXON0"),
    PINCTRL_PIN(ATM7021_P_HDMI_TXOP0, "P_HDMI_TXOP0"),
    PINCTRL_PIN(ATM7021_P_HDMI_TXON1, "P_HDMI_TXON1"),
    PINCTRL_PIN(ATM7021_P_HDMI_TXOP1, "P_HDMI_TXOP1"),
    PINCTRL_PIN(ATM7021_P_HDMI_TXON2, "P_HDMI_TXON2"),
    PINCTRL_PIN(ATM7021_P_HDMI_TXOP2, "P_HDMI_TXOP2"),
};

/* Ethernet MAC */
#define ATM7029_P_ETH_TXD0             _GPIOA(14)
#define ATM7029_P_ETH_TXD1             _GPIOA(15)
#define ATM7029_P_ETH_TX_EN            _GPIOA(16)
#define ATM7029_P_ETH_RX_EN            _GPIOA(17)
#define ATM7029_P_ETH_CRS_DV           _GPIOA(18)
#define ATM7029_P_ETH_RXD1             _GPIOA(19)
#define ATM7029_P_ETH_RXD0             _GPIOA(20)
#define ATM7029_P_ETH_REF_CLK          _GPIOA(21)
#define ATM7029_P_ETH_MDC               _GPIOA(22)
#define ATM7029_P_ETH_MDIO              _GPIOA(23)

/* INTC */
#define ATM7029_P_SIRQ0                 _GPIOA(24)
#define ATM7029_P_SIRQ1                 _GPIOA(25)
#define ATM7029_P_SIRQ2                 _GPIOA(26)

/* I2S */
#define ATM7029_P_I2S_D0                _GPIOA(27)
#define ATM7029_P_I2S_BCLK0             _GPIOA(28)
#define ATM7029_P_I2S_LRCLK0            _GPIOA(29)
#define ATM7029_P_I2S_MCLK0             _GPIOA(30)
#define ATM7029_P_I2S_D1                _GPIOA(31)
#define ATM7029_P_I2S_BCLK1             _GPIOB(0)
#define ATM7029_P_I2S_LRCLK1            _GPIOB(1)
#define ATM7029_P_I2S_MCLK1             _GPIOB(2)

/* PCM1 */
#define ATM7029_P_PCM1_OUT              _GPIOA(0)
#define ATM7029_P_PCM1_SYNC             _GPIOA(1)
#define ATM7029_P_PCM1_CLK0             _GPIOA(2)
#define ATM7029_P_PCM1_IN               _GPIOA(3)

/* KEY */
#define ATM7029_P_KS_IN0                _GPIOB(3)
#define ATM7029_P_KS_IN1                _GPIOB(4)
#define ATM7029_P_KS_IN2                _GPIOB(5)
#define ATM7029_P_KS_IN3                _GPIOB(6)
#define ATM7029_P_KS_OUT0               _GPIOB(7)
#define ATM7029_P_KS_OUT1               _GPIOB(8)
#define ATM7029_P_KS_OUT2               _GPIOB(9)

/* LCD0 */
#define ATM7029_P_LVDS_OEP              _GPIOB(10)
#define ATM7029_P_LVDS_OEN              _GPIOB(11)
#define ATM7029_P_LVDS_ODP              _GPIOB(12)
#define ATM7029_P_LVDS_ODN              _GPIOB(13)
#define ATM7029_P_LVDS_OCP              _GPIOB(14)
#define ATM7029_P_LVDS_OCN              _GPIOB(15)
#define ATM7029_P_LVDS_OBP              _GPIOB(16)
#define ATM7029_P_LVDS_OBN              _GPIOB(17)
#define ATM7029_P_LVDS_OAP              _GPIOB(18)
#define ATM7029_P_LVDS_OAN              _GPIOB(19)
#define ATM7029_P_LVDS_EEP              _GPIOB(20)
#define ATM7029_P_LVDS_EEN              _GPIOB(21)
#define ATM7029_P_LVDS_EDP              _GPIOB(22)
#define ATM7029_P_LVDS_EDN              _GPIOB(23)
#define ATM7029_P_LVDS_ECP              _GPIOB(24)
#define ATM7029_P_LVDS_ECN              _GPIOB(25)
#define ATM7029_P_LVDS_EBP              _GPIOB(26)
#define ATM7029_P_LVDS_EBN              _GPIOB(27)
#define ATM7029_P_LVDS_EAP              _GPIOB(28)
#define ATM7029_P_LVDS_EAN              _GPIOB(29)
#define ATM7029_P_LCD0_D18              _GPIOB(30)
#define ATM7029_P_LCD0_D17              _GPIOB(31)
#define ATM7029_P_LCD0_D16              _GPIOC(0)
#define ATM7029_P_LCD0_D9               _GPIOC(1)
#define ATM7029_P_LCD0_D8               _GPIOC(2)
#define ATM7029_P_LCD0_D2               _GPIOC(3)
#define ATM7029_P_LCD0_D1               _GPIOC(4)
#define ATM7029_P_LCD0_D0               _GPIOC(5)
#define ATM7029_P_DSI_DP0               _GPIOC(6)
#define ATM7029_P_DSI_DN0               _GPIOC(7)
#define ATM7029_P_DSI_DP2               _GPIOC(8)
#define ATM7029_P_DSI_DN2               _GPIOC(9)

/* SD/MS */
#define ATM7029_P_SD0_D0                _GPIOC(10)
#define ATM7029_P_SD0_D1                _GPIOC(11)
#define ATM7029_P_SD0_D2                _GPIOC(12)
#define ATM7029_P_SD0_D3                _GPIOC(13)
#define ATM7029_P_SD0_D4                _GPIOC(14)
#define ATM7029_P_SD0_D5                _GPIOC(15)
#define ATM7029_P_SD0_D6                _GPIOC(16)
#define ATM7029_P_SD0_D7                _GPIOC(17)
#define ATM7029_P_SD0_CMD               _GPIOC(18)
#define ATM7029_P_SD0_CLK               _GPIOC(19)
#define ATM7029_P_SD1_CMD               _GPIOC(20)
#define ATM7029_P_SD1_CLK               _GPIOC(21)

/* SPI */
#define ATM7029_P_SPI0_SCLK             _GPIOC(22)
#define ATM7029_P_SPI0_SS               _GPIOC(23)
#define ATM7029_P_SPI0_MISO             _GPIOC(24)
#define ATM7029_P_SPI0_MOSI             _GPIOC(25)

/* UART for console */
#define ATM7029_P_UART0_RX              _GPIOC(26)
#define ATM7029_P_UART0_TX              _GPIOC(27)

/* UART for Bluetooth */
#define ATM7029_P_UART2_RX              _GPIOD(11)
#define ATM7029_P_UART2_TX              _GPIOD(12)
#define ATM7029_P_UART2_RTSB            _GPIOD(13)
#define ATM7029_P_UART2_CTSB            _GPIOD(20)

/* UART for 3G */
#define ATM7029_P_UART3_RX              _GPIOD(21)
#define ATM7029_P_UART3_TX              _GPIOA(11)
#define ATM7029_P_UART3_RTSB            _GPIOA(10)
#define ATM7029_P_UART3_CTSB            _GPIOA(9)

/* UART for GPS */
#define ATM7029_P_UART4_RX              _GPIOA(8)
#define ATM7029_P_UART4_TX              _GPIOA(7)

/* I2C */
#define ATM7029_P_I2C0_SCLK             _GPIOC(28)
#define ATM7029_P_I2C0_SDATA            _GPIOC(29)
#define ATM7029_P_I2C1_SCLK             _GPIOC(30)
#define ATM7029_P_I2C1_SDATA            _GPIOA(4)
#define ATM7029_P_I2C2_SCLK             _GPIOA(5)
#define ATM7029_P_I2C2_SDATA            _GPIOA(6)

/* Sensor */
#define ATM7029_P_SENS1_PCLK            _GPIOC(31)
#define ATM7029_P_SENS1_VSYNC           _GPIOD(0)
#define ATM7029_P_SENS1_HSYNC           _GPIOD(1)
#define ATM7029_P_SENS1_D0              _GPIOD(2)
#define ATM7029_P_SENS1_D1              _GPIOD(3)
#define ATM7029_P_SENS1_D2              _GPIOD(4)
#define ATM7029_P_SENS1_D3              _GPIOD(5)
#define ATM7029_P_SENS1_D4              _GPIOD(6)
#define ATM7029_P_SENS1_D5              _GPIOD(7)
#define ATM7029_P_SENS1_D6              _GPIOD(8)
#define ATM7029_P_SENS1_D7              _GPIOD(9)
#define ATM7029_P_SENS1_CKOUT           _GPIOD(10)

/* NAND (1.8v / 3.3v) */
#define ATM7029_P_DNAND_D0              _PIN(20)
#define ATM7029_P_DNAND_D1              _PIN(21)
#define ATM7029_P_DNAND_D2              _PIN(22)
#define ATM7029_P_DNAND_D3              _PIN(23)
#define ATM7029_P_DNAND_D4              _PIN(24)
#define ATM7029_P_DNAND_D5              _PIN(25)
#define ATM7029_P_DNAND_D6              _PIN(26)
#define ATM7029_P_DNAND_D7              _PIN(27)
#define ATM7029_P_DNAND_WRB             _PIN(28)
#define ATM7029_P_DNAND_RDB             _PIN(29)
#define ATM7029_P_DNAND_RDBN            _PIN(30)
#define ATM7029_P_DNAND_DQS             _GPIOA(12)
#define ATM7029_P_DNAND_DQSN            _GPIOA(13)
#define ATM7029_P_DNAND_RB              _PIN(31)
#define ATM7029_P_DNAND_ALE             _GPIOD(18)
#define ATM7029_P_DNAND_CLE             _GPIOD(19)
#define ATM7029_P_DNAND_CEB0            _GPIOD(14)
#define ATM7029_P_DNAND_CEB1            _GPIOD(15)
#define ATM7029_P_DNAND_CEB2            _GPIOD(16)
#define ATM7029_P_DNAND_CEB3            _GPIOD(17)

/* System */
#define ATM7029_P_TST_OUT               _PIN(32)

/* Pad names for the pinmux subsystem */
static const struct pinctrl_pin_desc atm7029_leopard_pads[] = {
    PINCTRL_PIN(ATM7029_P_ETH_TXD0, "P_ETH_TXD0"),
    PINCTRL_PIN(ATM7029_P_ETH_TXD1, "P_ETH_TXD1"),
    PINCTRL_PIN(ATM7029_P_ETH_TX_EN, "P_ETH_TX_EN"),
    PINCTRL_PIN(ATM7029_P_ETH_RX_EN, "P_ETH_RX_EN"),
    PINCTRL_PIN(ATM7029_P_ETH_CRS_DV, "P_ETH_CRS_DV"),
    PINCTRL_PIN(ATM7029_P_ETH_RXD1, "P_ETH_RXD1"),
    PINCTRL_PIN(ATM7029_P_ETH_RXD0, "P_ETH_RXD0"),
    PINCTRL_PIN(ATM7029_P_ETH_REF_CLK, "P_ETH_REF_CLK"),
    PINCTRL_PIN(ATM7029_P_ETH_MDC, "P_ETH_MDC"),
    PINCTRL_PIN(ATM7029_P_ETH_MDIO, "P_ETH_MDIO"),
    PINCTRL_PIN(ATM7029_P_SIRQ0, "P_SIRQ0"),
    PINCTRL_PIN(ATM7029_P_SIRQ1, "P_SIRQ1"),
    PINCTRL_PIN(ATM7029_P_SIRQ2, "P_SIRQ2"),
    PINCTRL_PIN(ATM7029_P_I2S_D0, "P_I2S_D0"),
    PINCTRL_PIN(ATM7029_P_I2S_BCLK0, "P_I2S_BCLK0"),
    PINCTRL_PIN(ATM7029_P_I2S_LRCLK0, "P_I2S_LRCLK0"),
    PINCTRL_PIN(ATM7029_P_I2S_MCLK0, "P_I2S_MCLK0"),
    PINCTRL_PIN(ATM7029_P_I2S_D1, "P_I2S_D1"),
    PINCTRL_PIN(ATM7029_P_I2S_BCLK1, "P_I2S_BCLK1"),
    PINCTRL_PIN(ATM7029_P_I2S_LRCLK1, "P_I2S_LRCLK1"),
    PINCTRL_PIN(ATM7029_P_I2S_MCLK1, "P_I2S_MCLK1"),
    PINCTRL_PIN(ATM7029_P_PCM1_IN, "P_PCM1_IN"),
    PINCTRL_PIN(ATM7029_P_PCM1_CLK0, "P_PCM1_CLK0"),
    PINCTRL_PIN(ATM7029_P_PCM1_SYNC, "P_PCM1_SYNC"),
    PINCTRL_PIN(ATM7029_P_PCM1_OUT, "P_PCM1_OUT"),
    PINCTRL_PIN(ATM7029_P_KS_IN0, "P_KS_IN0"),
    PINCTRL_PIN(ATM7029_P_KS_IN1, "P_KS_IN1"),
    PINCTRL_PIN(ATM7029_P_KS_IN2, "P_KS_IN2"),
    PINCTRL_PIN(ATM7029_P_KS_IN3, "P_KS_IN3"),
    PINCTRL_PIN(ATM7029_P_KS_OUT0, "P_KS_OUT0"),
    PINCTRL_PIN(ATM7029_P_KS_OUT1, "P_KS_OUT1"),
    PINCTRL_PIN(ATM7029_P_KS_OUT2, "P_KS_OUT2"),
    PINCTRL_PIN(ATM7029_P_LVDS_OEP, "P_LVDS_OEP"),
    PINCTRL_PIN(ATM7029_P_LVDS_OEN, "P_LVDS_OEN"),
    PINCTRL_PIN(ATM7029_P_LVDS_ODP, "P_LVDS_ODP"),
    PINCTRL_PIN(ATM7029_P_LVDS_ODN, "P_LVDS_ODN"),
    PINCTRL_PIN(ATM7029_P_LVDS_OCP, "P_LVDS_OCP"),
    PINCTRL_PIN(ATM7029_P_LVDS_OCN, "P_LVDS_OCN"),
    PINCTRL_PIN(ATM7029_P_LVDS_OBP, "P_LVDS_OBP"),
    PINCTRL_PIN(ATM7029_P_LVDS_OBN, "P_LVDS_OBN"),
    PINCTRL_PIN(ATM7029_P_LVDS_OAP, "P_LVDS_OAP"),
    PINCTRL_PIN(ATM7029_P_LVDS_OAN, "P_LVDS_OAN"),
    PINCTRL_PIN(ATM7029_P_LVDS_EEP, "P_LVDS_EEP"),
    PINCTRL_PIN(ATM7029_P_LVDS_EEN, "P_LVDS_EEN"),
    PINCTRL_PIN(ATM7029_P_LVDS_EDP, "P_LVDS_EDP"),
    PINCTRL_PIN(ATM7029_P_LVDS_EDN, "P_LVDS_EDN"),
    PINCTRL_PIN(ATM7029_P_LVDS_ECP, "P_LVDS_ECP"),
    PINCTRL_PIN(ATM7029_P_LVDS_ECN, "P_LVDS_ECN"),
    PINCTRL_PIN(ATM7029_P_LVDS_EBP, "P_LVDS_EBP"),
    PINCTRL_PIN(ATM7029_P_LVDS_EBN, "P_LVDS_EBN"),
    PINCTRL_PIN(ATM7029_P_LVDS_EAP, "P_LVDS_EAP"),
    PINCTRL_PIN(ATM7029_P_LVDS_EAN, "P_LVDS_EAN"),
    PINCTRL_PIN(ATM7029_P_LCD0_D18, "P_LCD0_D18"),
    PINCTRL_PIN(ATM7029_P_LCD0_D17, "P_LCD0_D17"),
    PINCTRL_PIN(ATM7029_P_LCD0_D16, "P_LCD0_D16"),
    PINCTRL_PIN(ATM7029_P_LCD0_D9, "P_LCD0_D9"),
    PINCTRL_PIN(ATM7029_P_LCD0_D8, "P_LCD0_D8"),
    PINCTRL_PIN(ATM7029_P_LCD0_D2, "P_LCD0_D2"),
    PINCTRL_PIN(ATM7029_P_LCD0_D1, "P_LCD0_D1"),
    PINCTRL_PIN(ATM7029_P_LCD0_D0, "P_LCD0_D0"),
    PINCTRL_PIN(ATM7029_P_DSI_DP0, "P_DSI_DP0"),
    PINCTRL_PIN(ATM7029_P_DSI_DN0, "P_DSI_DN0"),
    PINCTRL_PIN(ATM7029_P_DSI_DP2, "P_DSI_DP2"),
    PINCTRL_PIN(ATM7029_P_DSI_DN2, "P_DSI_DN2"),
    PINCTRL_PIN(ATM7029_P_SD0_D0, "P_SD0_D0"),
    PINCTRL_PIN(ATM7029_P_SD0_D1, "P_SD0_D1"),
    PINCTRL_PIN(ATM7029_P_SD0_D2, "P_SD0_D2"),
    PINCTRL_PIN(ATM7029_P_SD0_D3, "P_SD0_D3"),
    PINCTRL_PIN(ATM7029_P_SD0_D4, "P_SD0_D4"),
    PINCTRL_PIN(ATM7029_P_SD0_D5, "P_SD0_D5"),
    PINCTRL_PIN(ATM7029_P_SD0_D6, "P_SD0_D6"),
    PINCTRL_PIN(ATM7029_P_SD0_D7, "P_SD0_D7"),
    PINCTRL_PIN(ATM7029_P_SD0_CMD, "P_SD0_CMD"),
    PINCTRL_PIN(ATM7029_P_SD0_CLK, "P_SD0_CLK"),
    PINCTRL_PIN(ATM7029_P_SD1_CMD, "P_SD1_CMD"),
    PINCTRL_PIN(ATM7029_P_SD1_CLK, "P_SD1_CLK"),
    PINCTRL_PIN(ATM7029_P_SPI0_SCLK, "P_SPI0_SCLK"),
    PINCTRL_PIN(ATM7029_P_SPI0_SS, "P_SPI0_SS"),
    PINCTRL_PIN(ATM7029_P_SPI0_MISO, "P_SPI0_MISO"),
    PINCTRL_PIN(ATM7029_P_SPI0_MOSI, "P_SPI0_MOSI"),
    PINCTRL_PIN(ATM7029_P_UART0_RX, "P_UART0_RX"),
    PINCTRL_PIN(ATM7029_P_UART0_TX, "P_UART0_TX"),
    PINCTRL_PIN(ATM7029_P_UART2_RX, "P_UART2_RX"),
    PINCTRL_PIN(ATM7029_P_UART2_TX, "P_UART2_TX"),
    PINCTRL_PIN(ATM7029_P_UART2_RTSB, "P_UART2_RTSB"),
    PINCTRL_PIN(ATM7029_P_UART2_CTSB, "P_UART2_CTSB"),
    PINCTRL_PIN(ATM7029_P_UART3_RX, "P_UART3_RX"),
    PINCTRL_PIN(ATM7029_P_UART3_TX, "P_UART3_TX"),
    PINCTRL_PIN(ATM7029_P_UART3_RTSB, "P_UART3_RTSB"),
    PINCTRL_PIN(ATM7029_P_UART3_CTSB, "P_UART3_CTSB"),
    PINCTRL_PIN(ATM7029_P_UART4_RX, "P_UART4_RX"),
    PINCTRL_PIN(ATM7029_P_UART4_TX, "P_UART4_TX"),
    PINCTRL_PIN(ATM7029_P_I2C0_SCLK, "P_I2C0_SCLK"),
    PINCTRL_PIN(ATM7029_P_I2C0_SDATA, "P_I2C0_SDATA"),
    PINCTRL_PIN(ATM7029_P_I2C1_SCLK, "P_I2C1_SCLK"),
    PINCTRL_PIN(ATM7029_P_I2C1_SDATA, "P_I2C1_SDATA"),
    PINCTRL_PIN(ATM7029_P_I2C2_SCLK, "P_I2C2_SCLK"),
    PINCTRL_PIN(ATM7029_P_I2C2_SDATA, "P_I2C2_SDATA"),
    PINCTRL_PIN(ATM7029_P_SENS1_PCLK, "P_SENS1_PCLK"),
    PINCTRL_PIN(ATM7029_P_SENS1_VSYNC, "P_SENS1_VSYNC"),
    PINCTRL_PIN(ATM7029_P_SENS1_HSYNC, "P_SENS1_HSYNC"),
    PINCTRL_PIN(ATM7029_P_SENS1_D0, "P_SENS1_D0"),
    PINCTRL_PIN(ATM7029_P_SENS1_D1, "P_SENS1_D1"),
    PINCTRL_PIN(ATM7029_P_SENS1_D2, "P_SENS1_D2"),
    PINCTRL_PIN(ATM7029_P_SENS1_D3, "P_SENS1_D3"),
    PINCTRL_PIN(ATM7029_P_SENS1_D4, "P_SENS1_D4"),
    PINCTRL_PIN(ATM7029_P_SENS1_D5, "P_SENS1_D5"),
    PINCTRL_PIN(ATM7029_P_SENS1_D6, "P_SENS1_D6"),
    PINCTRL_PIN(ATM7029_P_SENS1_D7, "P_SENS1_D7"),
    PINCTRL_PIN(ATM7029_P_SENS1_CKOUT, "P_SENS1_CKOUT"),
    PINCTRL_PIN(ATM7029_P_DNAND_D0, "P_DNAND_D0"),
    PINCTRL_PIN(ATM7029_P_DNAND_D1, "P_DNAND_D1"),
    PINCTRL_PIN(ATM7029_P_DNAND_D2, "P_DNAND_D2"),
    PINCTRL_PIN(ATM7029_P_DNAND_D3, "P_DNAND_D3"),
    PINCTRL_PIN(ATM7029_P_DNAND_D4, "P_DNAND_D4"),
    PINCTRL_PIN(ATM7029_P_DNAND_D5, "P_DNAND_D5"),
    PINCTRL_PIN(ATM7029_P_DNAND_D6, "P_DNAND_D6"),
    PINCTRL_PIN(ATM7029_P_DNAND_D7, "P_DNAND_D7"),
    PINCTRL_PIN(ATM7029_P_DNAND_WRB, "P_DNAND_WRB"),
    PINCTRL_PIN(ATM7029_P_DNAND_RDB, "P_DNAND_RDB"),
    PINCTRL_PIN(ATM7029_P_DNAND_RDBN, "P_DNAND_RDBN"),
    PINCTRL_PIN(ATM7029_P_DNAND_DQS, "P_DNAND_DQS"),
    PINCTRL_PIN(ATM7029_P_DNAND_DQSN, "P_DNAND_DQSN"),
    PINCTRL_PIN(ATM7029_P_DNAND_RB, "P_DNAND_RB"),
    PINCTRL_PIN(ATM7029_P_DNAND_ALE, "P_DNAND_ALE"),
    PINCTRL_PIN(ATM7029_P_DNAND_CLE, "P_DNAND_CLE"),
    PINCTRL_PIN(ATM7029_P_DNAND_CEB0, "P_DNAND_CEB0"),
    PINCTRL_PIN(ATM7029_P_DNAND_CEB1, "P_DNAND_CEB1"),
    PINCTRL_PIN(ATM7029_P_DNAND_CEB2, "P_DNAND_CEB2"),
    PINCTRL_PIN(ATM7029_P_DNAND_CEB3, "P_DNAND_CEB3"),
    PINCTRL_PIN(ATM7029_P_TST_OUT, "P_TST_OUT"),
};

unsigned int nfunctions, ngroups;
struct asoc_pinmux_group *leopard_pingroups;
struct asoc_pinmux_func *leopard_functions;

static struct pinctrl_gpio_range asoc_gpio_ranges[] = {
    {
        .name = "asoc-gpio*",
        .id = 0,
        .base = 0,
        .pin_base = 0,
        .npins = ASOC_NR_GPIO,
    },
};

static struct asoc_pinctrl_soc_info leopard_pinctrl_info = {
    .gpio_ranges = asoc_gpio_ranges,
    .gpio_num_ranges = ARRAY_SIZE(asoc_gpio_ranges),
    // .pins = leopard_pads,
    // .npins = ARRAY_SIZE(leopard_pads),
//    .functions = leopard_functions,
//    .nfunctions = ARRAY_SIZE(leopard_functions),
//    .groups = leopard_pingroups,
//    .ngroups = ARRAY_SIZE(leopard_pingroups),
};

static void init_leopard_pads(void)
{
    int dvfslevel=asoc_get_dvfslevel();
    
    switch (ASOC_GET_IC(dvfslevel))
    {
    case 0x7023:
    case 0x7021:    
        printk("%s,%d\n", __FUNCTION__, __LINE__);
        leopard_pinctrl_info.pins = atm7021_leopard_pads;
        leopard_pinctrl_info.npins = ARRAY_SIZE(atm7021_leopard_pads);
        break;        
    case 0x7029:
        printk("%s,%d\n", __FUNCTION__, __LINE__);
        leopard_pinctrl_info.pins = atm7029_leopard_pads;
        leopard_pinctrl_info.npins = ARRAY_SIZE(atm7029_leopard_pads);
        break;     
    default:
        printk("%s,%d,invalid dvfslevel\n", __FUNCTION__, __LINE__);
        break;
    }	
}

extern int set_asoc_pinctrl_soc_info(struct asoc_pinctrl_soc_info *);
static int __devinit leopard_pinctrl_probe(struct platform_device *pdev)
{
	printk("leopard_pinctrl_probe\n");
	
	
	set_asoc_pinctrl_soc_info(&leopard_pinctrl_info);
    return asoc_pinctrl_probe(pdev, &leopard_pinctrl_info);
}

static struct platform_driver leopard_pinctrl_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .owner = THIS_MODULE,
    },
    .probe = leopard_pinctrl_probe,
    .remove = __devexit_p(asoc_pinctrl_remove),
};

static int __init leopard_pinctrl_init(void)
{
    init_leopard_pads();
    return platform_driver_register(&leopard_pinctrl_driver);
}
arch_initcall(leopard_pinctrl_init);

static void __exit leopard_pinctrl_exit(void)
{
    platform_driver_unregister(&leopard_pinctrl_driver);
}
module_exit(leopard_pinctrl_exit);

MODULE_AUTHOR("Actions Semi Inc.");
MODULE_DESCRIPTION("Pin control driver for Actions SOC");
MODULE_LICENSE("GPL");
