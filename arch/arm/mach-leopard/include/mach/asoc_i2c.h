/*
 * Copyright 2011 Actions Semi Inc.
 * Author: Actions Semi, Inc.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */
#ifndef __ASOC_I2C_H__
#define __ASOC_I2C_H__

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/completion.h>

/* don't use FIFO mode for HDMI HDCP because of incompatible timing */
#define ASOC_I2C_SUPPORT_HDMI_NOFIFO
//#define I2C_DEBUG_INFO
//#define I2C_DEBUG_WARNING

//regs.
#define I2C_CTL                     0x0000          /* I2C Control Register */
#define I2C_CLKDIV                  0x0004          /* I2C Clock Divide Register */
#define I2C_STAT                    0x0008          /* I2C Status Register */
#define I2C_ADDR                    0x000C          /* I2C Address Register */
#define I2C_TXDAT                   0x0010          /* I2C TX Data Register */
#define I2C_RXDAT                   0x0014          /* I2C RX Data Register */
#define I2C_CMD                     0x0018          /* I2C Command Register */
#define I2C_FIFOCTL                 0x001C          /* I2C FIFO control Register */
#define I2C_FIFOSTAT                0x0020          /* I2C FIFO status Register */
#define I2C_DATCNT                  0x0024          /* I2C Data transmit counter */
#define I2C_RCNT                    0x0028          /* I2C Data transmit remain counter */

/* I2Cx_CTL */
#define I2C_CTL_GRAS                (0x1 << 0)      /* Generate ACK or NACK Signal */
#define I2C_CTL_GRAS_ACK           0            /* generate the ACK signal at 9th clock of SCL */
#define I2C_CTL_GRAS_NACK          I2C_CTL_GRAS /* generate the NACK signal at 9th clock of SCL */
#define I2C_CTL_RB                  (0x1 << 1)     /* Release Bus */
#define I2C_CTL_GBCC_MASK           (0x3 << 2)     /* Loop Back Enable */
#define I2C_CTL_GBCC(x)             (((x) & 0x3) << 2)
#define I2C_CTL_GBCC_NONE           I2C_CTL_GBCC(0)
#define I2C_CTL_GBCC_START          I2C_CTL_GBCC(1)
#define I2C_CTL_GBCC_STOP           I2C_CTL_GBCC(2)
#define I2C_CTL_GBCC_RESTART        I2C_CTL_GBCC(3)
#define I2C_CTL_IRQE                (0x1 << 5)     /* IRQ Enable */
#define I2C_CTL_PUEN                (0x1 << 6)     /* Internal Pull-Up resistor (1.5k) enable. */
#define I2C_CTL_EN                  (0x1 << 7)     /* Enable. When enable, reset the status machine to IDLE */
#define I2C_CTL_AE                  (0x1 << 8)     /* Arbitor enable */

/* I2Cx_CLKDIV */
#define I2C_CLKDIV_DIV_MASK         (0xff << 0)     /* Clock Divider Factor (only for master mode). */
#define I2C_CLKDIV_DIV(x)           (((x) & 0xff) << 0)

/* I2Cx_STAT */
#define I2C_STAT_RACK               (0x1 << 0)      /* Receive ACK or NACK when transmit data or address */
#define I2C_STAT_BEB                (0x1 << 1)      /* IRQ Pending Bit, Write “1” to clear this bit */
#define I2C_STAT_IRQP               (0x1 << 2)      /* IRQ Pending Bit, Write “1” to clear this bit */
#define I2C_STAT_LAB                (0x1 << 3)      /* Lose arbitration bit, Write “1” to clear this bit */
#define I2C_STAT_STPD               (0x1 << 4)      /* Stop detect bit, Write “1” to clear this bit */
#define I2C_STAT_STAD               (0x1 << 5)      /* Start detect bit, Write “1” to clear this bit */
#define I2C_STAT_BBB                (0x1 << 6)      /* Bus busy bit */
#define I2C_STAT_TCB                (0x1 << 7)      /* Transfer complete bit */
#define I2C_STAT_LBST               (0x1 << 8)      /* Last Byte Status Bit, 0: address, 1: data */
#define I2C_STAT_SAMB               (0x1 << 9)      /* Slave address match bit */
#define I2C_STAT_SRGC               (0x1 << 10)     /* Slave receive general call */

#define I2C_BUS_ERR_MSK             ( I2C_STAT_LAB | I2C_STAT_BEB)

/* I2Cx_CMD */
#define I2C_CMD_SBE                 (0x1 << 0)      /* Start bit enable */
#define I2C_CMD_AS_MASK             (0x7 << 1)      /* Address select */
#define I2C_CMD_AS(x)               (((x) & 0x7) << 1)
#define I2C_CMD_RBE                 (0x1 << 4)      /* Restart bit enable */
#define I2C_CMD_SAS_MASK            (0x7 << 5)      /* Second Address select */
#define I2C_CMD_SAS(x)              (((x) & 0x7) << 5)
#define I2C_CMD_DE                  (0x1 << 8)      /* Data enable */
#define I2C_CMD_NS                  (0x1 << 9)      /* NACK select */
#define I2C_CMD_SE                  (0x1 << 10)     /* Stop enable */
#define I2C_CMD_MSS                 (0x1 << 11)     /* MSS Master or slave mode select */
#define I2C_CMD_WRS                 (0x1 << 12)     /* Write or Read select */
#define I2C_CMD_EXEC                (0x1 << 15)     /* Start to execute the command list */

/*FIFO mode write cmd 0x8d01 */
#define I2C_CMD_X	(\
	I2C_CMD_EXEC | I2C_CMD_MSS | \
	I2C_CMD_SE | I2C_CMD_DE | I2C_CMD_SBE)

/* I2Cx_FIFOCTL */
#define I2C_FIFOCTL_NIB             (0x1 << 0)      /* NACK Ignore Bit */
#define I2C_FIFOCTL_RFR             (0x1 << 1)      /* RX FIFO reset bit, Write 1 to reset RX FIFO */
#define I2C_FIFOCTL_TFR             (0x1 << 2)      /* TX FIFO reset bit, Write 1 to reset TX FIFO */

/* I2Cx_FIFOSTAT */
#define I2C_FIFOSTAT_CECB           (0x1 << 0)      /* command Execute Complete bit */
#define I2C_FIFOSTAT_RNB            (0x1 << 1)      /* Receive NACK Error bit */
#define I2C_FIFOSTAT_RFE            (0x1 << 2)      /* RX FIFO empty bit */
#define I2C_FIFOSTAT_RFF            (0x1 << 3)      /* RX FIFO full bit */
#define I2C_FIFOSTAT_TFE            (0x1 << 4)      /* TX FIFO empty bit */
#define I2C_FIFOSTAT_TFF            (0x1 << 5)      /* TX FIFO full bit */
#define I2C_FIFOSTAT_RFD_MASK       (0xff << 8)     /* Rx FIFO level display */
#define I2C_FIFOSTAT_RFD_SHIFT      (8)
#define I2C_FIFOSTAT_TFD_MASK       (0xff << 16)    /* Tx FIFO level display */
#define I2C_FIFOSTAT_TFD_SHIFT      (16)

#define I2C_CTL_START_CMD               (  \
          I2C_CTL_IRQE |   \
          I2C_CTL_EN |     \
          I2C_CTL_GBCC_START |    \
          I2C_CTL_PUEN |     \
          I2C_CTL_RB   \
         )

#define I2C_CTL_STOP_CMD                (  \
            I2C_CTL_EN |  \
            I2C_CTL_PUEN | \
            I2C_CTL_GBCC_STOP | \
            I2C_CTL_RB | \
            I2C_CTL_IRQE \
        )

#define I2C_CTL_RESTART_CMD             ( \
            I2C_CTL_IRQE | \
            I2C_CTL_EN | \
            I2C_CTL_GBCC_RESTART | \
            I2C_CTL_PUEN | \
            I2C_CTL_RB \
        )
#define I2C_MODULE_CLK		(100*1000*1000)
#define I2C_CLK_STD                     (100*1000) //khz
#define I2C_CLK_FAST                    (400*1000) //khz
#define I2C_CLK_HDMI                    (87*1000)
#define I2C_TRANS_TIMEOUT               (5*HZ)
#define I2C_STATE_DEFAULT "default"

enum i2c_state {
    INVALIDSTATE,
    RESTARTSTATE,
    STARTSTATE,
    WRITESTATE,
    READSTATE,
    STOPSTATE,

    FIFO_READSTATE,
    FIFO_WRITESTATE,
};
enum asoc_i2c_type {
	ASOC_I2C_GL5203 = 0,
	ASOC_I2C_GL5204,
	ASOC_I2C_GL5205,
};


struct asoc_i2c_bus_platform_data {
    u32                 clkrate;
};

struct asoc_i2c_dev {
    struct device       *dev;
    struct resource     *ioarea;
    struct i2c_adapter  adapter;
    struct completion   cmd_complete;
    struct i2c_msg      *msgs;
    struct mutex        mutex;
    wait_queue_head_t   waitqueue;
    unsigned int        msg_num;
    unsigned int        msg_idx;
    unsigned int        msg_ptr;
	int i2c_addr_type;	/*1: HDMI,0 else */

    spinlock_t          lock;
    enum i2c_state           state;
    void __iomem        *base;      /* virtual */
    unsigned long       phys;
    u32                 speed;      /* Speed of bus in Khz */
    u32                 i2c_freq;
    u32                 i2c_freq_cfg;
    struct clk          *clk;
	struct clk *i2c_clk;
    int                 irq;
    u8                  fifo_size;
    u8                  rev;
#ifdef ASOC_I2C_SUPPORT_HDMI_NOFIFO
    /* use FIFO mode for HDMI HDCP? */
    int                 hdmi_nofifo;
#endif
};
#if 0
static inline void asoc_i2c_writel(struct asoc_i2c_dev *i2c_dev, u32 val, int reg)
{
    //i2c_dbg("-->>write 0x%x to 0x%x",val, (u32)(i2c_dev->base +reg));
    __raw_writel(val, i2c_dev->base + reg);
}

static inline u32 asoc_i2c_readl(struct asoc_i2c_dev *i2c_dev, int reg)
{
    return __raw_readl(i2c_dev->base + reg);
}
#endif //0
#endif //__ASOC_I2C_H__


