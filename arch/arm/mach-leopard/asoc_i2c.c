/*
 * arch/arm/mach-leopard/asoc_i2c.c
 *
 * I2C master mode controller driver for Actions SOC
 *
 * Copyright 2012 Actions Semi Inc.
 * Author: Actions Semi, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/cpufreq.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/clock.h>
#include <mach/debug.h>

extern int get_config(const char *key, char *buff, int len);

//#define I2C_DEBUG_INFO
#define I2C_DEBUG_WARNING
#define asoc_dump_mem(a, b, c, d)           do {} while (0);

/* support FIFO read/write */
#define ASOC_I2C_SUPPORT_FIFO
#define ASOC_FIFO_MAX_DATA_LENGTH           120
#define ASOC_FIFO_MAX_INTER_ADDR_LENGTH     7

/* i2c speed limit config */
#define ASOC_I2C_SUPPORT_FREQ_LIMIT
#define NUM_OF_MAX_FREQ_LIMITS              16

/* don't use FIFO mode for HDMI HDCP because of incompatible timing */
#define ASOC_I2C_SUPPORT_HDMI_NOFIFO

#ifdef I2C_DEBUG_INFO
#define i2c_dbg(fmt, args...)   \
    printk(KERN_INFO fmt, ##args)
#else
#define i2c_dbg(fmt, args...)   \
    do {} while(0)
#endif

#ifdef I2C_DEBUG_WARNING
#define i2c_warn(fmt, args...)  \
    printk(KERN_WARNING "asoc_i2c: "fmt"\n", ##args)
#else
#define i2c_warn(fmt, args...)   \
    do {} while(0)
#endif


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
#define     I2C_CTL_GRAS_ACK           0            /* generate the ACK signal at 9th clock of SCL */
#define     I2C_CTL_GRAS_NACK          I2C_CTL_GRAS /* generate the NACK signal at 9th clock of SCL */
#define I2C_CTL_RB                  (0x1 << 1)     /* Release Bus */
#define I2C_CTL_GBCC_MASK           (0x3 << 2)     /* Loop Back Enable */
#define I2C_CTL_GBCC(x)             (((x) & 0x3) << 2)
#define     I2C_CTL_GBCC_NONE           I2C_CTL_GBCC(0)
#define     I2C_CTL_GBCC_START          I2C_CTL_GBCC(1)
#define     I2C_CTL_GBCC_STOP           I2C_CTL_GBCC(2)
#define     I2C_CTL_GBCC_RESTART        I2C_CTL_GBCC(3)
#define I2C_CTL_IRQE                (0x1 << 5)     /* IRQ Enable */
#define I2C_CTL_PUEN                (0x1 << 6)     /* Internal Pull-Up resistor (1.5k) enable. */
#define I2C_CTL_EN                  (0x1 << 7)     /* Enable. When enable, reset the status machine to IDLE */
#define I2C_CTL_AE                  (0x1 << 8)     /* Arbitor enable */

/* I2Cx_CLKDIV */
#define I2C_CLKDIV_DIV_MASK         (0xff << 0)     /* Clock Divider Factor (only for master mode). */
#define I2C_CLKDIV_DIV(x)           (((x) & 0xff) << 0)

/* I2Cx_STAT */
#define I2C_STAT_RACK               (0x1 << 0)      /* Receive ACK or NACK when transmit data or address */
#define I2C_STAT_BEB                (0x1 << 1)      /* IRQ Pending Bit, Write ¡°1¡± to clear this bit */
#define I2C_STAT_IRQP               (0x1 << 2)      /* IRQ Pending Bit, Write ¡°1¡± to clear this bit */
#define I2C_STAT_LAB                (0x1 << 3)      /* Lose arbitration bit, Write ¡°1¡± to clear this bit */
#define I2C_STAT_STPD               (0x1 << 4)      /* Stop detect bit, Write ¡°1¡± to clear this bit */
#define I2C_STAT_STAD               (0x1 << 5)      /* Start detect bit, Write ¡°1¡± to clear this bit */
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

#define I2C_CLK_STD                     (100*1000) //khz
#define I2C_CLK_FAST                    (400*1000) //khz
#define I2C_CLK_HDMI                    (87*1000)
#define I2C_TRANS_TIMEOUT               (5*HZ)

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

struct asoc_i2c_bus_platform_data {
    u32                 clkrate;
};

#ifdef ASOC_I2C_SUPPORT_FREQ_LIMIT
struct i2c_freq_limit {
    int                 chan;
    int                 addr;
    unsigned int        freq;
};
#endif

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

    spinlock_t          lock;
    enum i2c_state           state;
    void __iomem        *base;      /* virtual */
    unsigned long       phys;
    u32                 speed;      /* Speed of bus in Khz */
    u32                 i2c_freq;
    u32                 i2c_freq_cfg;

#ifdef ASOC_I2C_SUPPORT_FREQ_LIMIT
    struct i2c_freq_limit  freq_limits[NUM_OF_MAX_FREQ_LIMITS];
#endif

    struct clk          *clk;
    int                 irq;
    u8                  fifo_size;
    u8                  rev;
#ifdef ASOC_I2C_SUPPORT_HDMI_NOFIFO
    /* use FIFO mode for HDMI HDCP? */
    int                 hdmi_nofifo;
#endif
};

static inline void asoc_i2c_writel(struct asoc_i2c_dev *i2c_dev, u32 val, int reg)
{
    i2c_dbg("-->>write 0x%x to 0x%x",val, (u32)(i2c_dev->base +reg));
    __raw_writel(val, (u32)i2c_dev->base + reg);
}

static inline u32 asoc_i2c_readl(struct asoc_i2c_dev *i2c_dev, int reg)
{
    return __raw_readl((u32)(i2c_dev->base + reg));
}


static void asoc_i2c_printifo(struct asoc_i2c_dev * dev)
{
#if 1
    printk("====================================\n");
    printk("MFP_CTL2:       0x%x\n", act_readl(MFP_CTL2));
    printk("MFP_CTL3:       0x%x\n", act_readl(MFP_CTL3));
    printk("CMU_DEVCLKEN0:  0x%x\n", act_readl(CMU_DEVCLKEN0));
    printk("CMU_DEVCLKEN1:  0x%x\n", act_readl(CMU_DEVCLKEN1));
//  printk("I2C%d_CTL: 0x%x\n",act_readl(dev->base + I2C_CTL));
    printk("I2C_CTL:        0x%x\n", asoc_i2c_readl(dev, I2C_CTL));
    printk("I2C_CLKDIV:     0x%x\n", asoc_i2c_readl(dev, I2C_CLKDIV));
    printk("I2C_STAT:       0x%x\n", asoc_i2c_readl(dev, I2C_STAT));
    printk("I2C_ADDR:       0x%x\n", asoc_i2c_readl(dev, I2C_ADDR));
    printk("I2C_CMD:        0x%x\n", asoc_i2c_readl(dev, I2C_CMD));
    printk("I2C_FIFOCTL:    0x%x\n", asoc_i2c_readl(dev, I2C_FIFOCTL));
    printk("I2C_FIFOSTAT:   0x%x\n", asoc_i2c_readl(dev, I2C_FIFOSTAT));
    printk("I2C_DATCNT:     0x%x\n", asoc_i2c_readl(dev, I2C_DATCNT));
    printk("I2C_RCNT:       0x%x\n", asoc_i2c_readl(dev, I2C_RCNT));

    printk("====================================\n");
#endif

}
/*
 * Disable i2C controler <Disable its clock..??>
 */
static void asoc_i2c_disable(struct asoc_i2c_dev *dev)
{
    asoc_i2c_writel(dev,
        asoc_i2c_readl(dev, I2C_CTL) & ~I2C_CTL_EN, I2C_CTL);

    clk_disable(dev->clk);

    return;
}

#ifdef ASOC_I2C_SUPPORT_FREQ_LIMIT
static unsigned int asoc_i2c_get_speed_limit(struct asoc_i2c_dev *dev,  int addr)
{
    int i;

    for (i = 0; i < NUM_OF_MAX_FREQ_LIMITS; i++) {
        if ((addr == dev->freq_limits[i].addr) &&
            (dev->adapter.nr == dev->freq_limits[i].chan)) {
                i2c_dbg("%s: i2c%d: addr %d, freq_limit %d\n",
                    __FUNCTION__,
                    dev->freq_limits[i].chan,
                    dev->freq_limits[i].addr,
                    dev->freq_limits[i].freq);
            return dev->freq_limits[i].freq;
        }
    }

    return 0;
}
#endif

static void i2c_set_freq(struct asoc_i2c_dev *dev, u32 pclk, u32 i2c_freq)
{
    u32 div_factor;

    div_factor = (pclk + i2c_freq * 16 - 1) / (i2c_freq * 16);
    asoc_i2c_writel(dev, I2C_CLKDIV_DIV(div_factor), I2C_CLKDIV);

    i2c_dbg("iic clock divisor is %d!\n", div_factor);
    i2c_dbg("iic clock is %dHz!\n", pclk / (div_factor * 16));

    return;
}

/*
 * Initialize hardware registers.
 */
static void asoc_i2c_hwinit(struct asoc_i2c_dev *dev)
{
    struct clk *pclk;

#ifdef CONFIG_MACH_LEOPARD_FPGA
    asoc_i2c_writel(dev, 4, I2C_CLKDIV);    /* 400K */
    return;
#endif

    switch ((u32)dev->phys)
    {
    case I2C0_BASE:
        dev->clk = clk_get_sys(CLK_NAME_I2C0_CLK, NULL);
        break;
    case I2C1_BASE:
        dev->clk = clk_get_sys(CLK_NAME_I2C1_CLK, NULL);
        break;
    case I2C2_BASE:
        dev->clk = clk_get_sys(CLK_NAME_I2C2_CLK, NULL);
        break;
    default:
        i2c_warn("invalid i2c phys 0x%lx\n", dev->phys);
        return;
    }

    if (IS_ERR(dev->clk)) { 
        printk("%s() cannot get i2c clk, err %ld\n", 
            __FUNCTION__, PTR_ERR(dev->clk));
        return;
    }
    
    clk_enable(dev->clk);
    clk_reset(dev->clk);

    pclk = clk_get_sys(CLK_NAME_HOSC, NULL);
    if (IS_ERR(pclk)) { 
        printk("%s() cannot get pclk, err %ld\n", 
            __FUNCTION__, PTR_ERR(pclk));
    }

    i2c_set_freq(dev, clk_get_rate(pclk), dev->i2c_freq);
}

static void asoc_i2cdev_init(struct asoc_i2c_dev *dev)
{
//    u32 iiccon = I2C_CTL_EN | I2C_CTL_IRQE | I2C_CTL_PUEN;
    u32 iiccon = I2C_CTL_EN | I2C_CTL_IRQE | I2C_CTL_PUEN;

    i2c_dbg("asoc_i2cdev_init");
    asoc_i2c_hwinit(dev);
    asoc_i2c_writel(dev, 0xff, I2C_STAT);
    asoc_i2c_writel(dev, iiccon, I2C_CTL);
}

static inline void asoc_i2c_disable_irq(struct asoc_i2c_dev *dev)
{
    u32 tmp = asoc_i2c_readl(dev, I2C_CTL) & (~I2C_CTL_IRQE);
    asoc_i2c_writel(dev, tmp, I2C_CTL);
}

static inline void asoc_i2c_enable_irq(struct asoc_i2c_dev *dev)
{
    u32 tmp = asoc_i2c_readl(dev, I2C_CTL) | I2C_CTL_IRQE;
    asoc_i2c_writel(dev, tmp, I2C_CTL);
}

static inline void asoc_i2c_clear_tcb(struct asoc_i2c_dev *dev)
{
    volatile unsigned int tmp;
    int retry_times = 10;

    i2c_dbg("clear tcb");

    do
    {
        tmp = asoc_i2c_readl(dev, I2C_STAT) | I2C_STAT_IRQP;
        asoc_i2c_writel(dev, tmp, I2C_STAT);

        /* ensure write finished */
        tmp = asoc_i2c_readl(dev, I2C_STAT);
        if (!(tmp & I2C_STAT_IRQP))
            return;

        /*
         * Note: we cannot clear the IRQ pending if transfer bytes > fifo depth
         * so we skip the check if not all data in fifo
         */

        if ((dev->state == FIFO_READSTATE) &&
            (I2C_FIFOSTAT_RFE & asoc_i2c_readl(dev, I2C_FIFOSTAT)) &&
            (asoc_i2c_readl(dev, I2C_RCNT) != 0)) {

            i2c_dbg("%s(): [i2c%d] skip fifo read IRQ pending check\n",
                __FUNCTION__, dev->adapter.nr);
            return;
        }

        if ((dev->state == FIFO_WRITESTATE) &&
            (asoc_i2c_readl(dev, I2C_RCNT) != 0)) {

            i2c_dbg("%s(): [i2c%d] skip fifo write IRQ pending check\n",
                __FUNCTION__, dev->adapter.nr);
            return;
        }

        retry_times--;
    } while (retry_times > 0);

    /* clear IRQ pending timeout, we have to reset the I2C controller */

    i2c_warn("%s(): [i2c%d] clear IRQ pending error! reset controller\n",
        __FUNCTION__,
        dev->adapter.nr);

    asoc_i2c_printifo(dev);

    /* reset I2C controller */
    asoc_i2c_writel(dev, asoc_i2c_readl(dev, I2C_CTL) & ~I2C_CTL_EN, I2C_CTL);
    udelay(1);
    asoc_i2c_writel(dev, asoc_i2c_readl(dev, I2C_CTL) | I2C_CTL_EN, I2C_CTL);
    udelay(1);
}

static inline void asoc_i2c_reset_fifo(struct asoc_i2c_dev *dev)
{
    asoc_i2c_writel(dev, I2C_FIFOCTL_RFR | I2C_FIFOCTL_TFR, I2C_FIFOCTL);
}

static inline int isMsgEnd(struct asoc_i2c_dev *dev)
{
    return (dev->msg_ptr == (dev->msgs->len - 1));
}

static inline int isLastMsg(struct asoc_i2c_dev *dev)
{
    return (dev->msg_idx == (dev->msg_num - 1));
}

static inline int isMsgFinish(struct asoc_i2c_dev *dev)
{
    return (dev->msg_ptr >= dev->msgs->len);
}

static inline int asoc_i2c_reset(struct asoc_i2c_dev *dev)
{
    asoc_i2c_reset_fifo(dev);

    /* reset i2c controller */
    asoc_i2c_writel(dev,
        asoc_i2c_readl(dev, I2C_CTL) & ~I2C_CTL_EN, I2C_CTL);

    asoc_i2c_writel(dev,
        asoc_i2c_readl(dev, I2C_CTL) | I2C_CTL_EN, I2C_CTL);
}

static void asoc_master_trans_completion(struct asoc_i2c_dev *dev, int retval)
{
    i2c_dbg("I2C Trans complete %s", retval? "failed" : "successfully");

//    spin_lock_irq(&dev->lock);
    dev->msgs = NULL;
    dev->msg_num = 0;
    dev->msg_ptr = 0;
    dev->msg_idx++;
    if ( retval )
        dev->msg_idx = retval;
//    spin_unlock_irq(&dev->lock);

    wake_up(&dev->waitqueue);
}

static void asoc_i2c_message_start(struct asoc_i2c_dev *dev,
                                   struct i2c_msg *msg)
{
    u16 addr = (msg->addr & 0x7f) << 1;
    u32 ctl;

    i2c_dbg("%s()\n", __FUNCTION__);

    if ( msg->flags & I2C_M_RD ) {
        addr |= 1;
    }

#if 1
    dev->state = STARTSTATE;
    ctl = I2C_CTL_IRQE | I2C_CTL_EN | I2C_CTL_GBCC_START
        | I2C_CTL_PUEN | I2C_CTL_RB;
    asoc_i2c_writel(dev, addr, I2C_TXDAT);
    asoc_i2c_writel(dev, ctl, I2C_CTL);
#endif

#if 0

    char buf[64];
    int i;
    i2c_dbg("%s() raw write\n", __FUNCTION__);


    //module_reset(MOD_ID_I2C0);
    asoc_i2c_writel(dev, 0x40, I2C_CTL);
    mdelay(10);
    asoc_i2c_writel(dev, 0x80, I2C_CTL);
    mdelay(10);

    asoc_i2c_writel(dev, 0xa0, I2C_TXDAT);
    asoc_i2c_writel(dev, 0x86, I2C_CTL);
    mdelay(10);
    asoc_i2c_printifo(dev);

    asoc_i2c_writel(dev, 0xe5, I2C_STAT);
    asoc_i2c_printifo(dev);

    asoc_i2c_writel(dev, 0x00, I2C_TXDAT);
    asoc_i2c_writel(dev, 0x80, I2C_CTL);
    mdelay(10);
    asoc_i2c_printifo(dev);
#endif

#if 0

    i2c_dbg("%s() raw write\n", __FUNCTION__);

    char buf[64];
    int i;

    //module_reset(MOD_ID_I2C0);
    udelay(10);
    asoc_i2c_writel(dev, 0x40, I2C_CTL);
    udelay(10);
    asoc_i2c_writel(dev, 0xc0, I2C_CTL);
    asoc_i2c_writel(dev, 0x06, I2C_FIFOCTL);

    asoc_i2c_writel(dev, 64, I2C_DATCNT);
    asoc_i2c_writel(dev, 0xa0, I2C_TXDAT);
    asoc_i2c_writel(dev, 0x00, I2C_TXDAT);
    asoc_i2c_writel(dev, 0xa1, I2C_TXDAT);
    asoc_i2c_writel(dev, 0x8f35, I2C_CMD);

    asoc_i2c_printifo(dev);
    mdelay(100);
    asoc_i2c_printifo(dev);

    for (i = 0; i < 64; i++)
    {
        buf[i] = asoc_i2c_readl(dev, I2C_RXDAT);
    }

    dump_mem(buf, 64, 0, 1);
    i2c_dbg("%s() end\n", __FUNCTION__);
#endif
}

static void asoc_i2c_message_restart(struct asoc_i2c_dev *dev,
                                     struct i2c_msg *msg)
{
    u16 addr = (msg->addr & 0x7f) << 1;
    u32 ctl;

    i2c_dbg("%s()\n", __FUNCTION__);

    if (msg->flags & I2C_M_RD){
        addr |= 1;
    }

    ctl = (asoc_i2c_readl(dev, I2C_CTL) & ~I2C_CTL_GBCC_MASK)
        | I2C_CTL_GBCC_RESTART | I2C_CTL_RB;
    asoc_i2c_writel(dev, addr, I2C_TXDAT);
    asoc_i2c_writel(dev, ctl, I2C_CTL);
}

//Only to send stop command, but Not stop at all.
static void asoc_i2c_stop(struct asoc_i2c_dev *dev, int retval)
{
    u32 ctl;

    i2c_dbg("%s(): retval %d\n", __FUNCTION__, retval);

    dev->state = STOPSTATE;
    ctl = I2C_CTL_EN | I2C_CTL_GBCC_STOP | I2C_CTL_RB ; //stop cmd: 0xaa
    asoc_i2c_writel(dev, ctl, I2C_CTL);
    udelay(10);
    asoc_master_trans_completion(dev, retval);
}

#ifdef ASOC_I2C_SUPPORT_FIFO

static void asoc_i2c_message_fifo_start(struct asoc_i2c_dev *dev,
                                   struct i2c_msg *msgs, int num)
{
    u16 addr = (msgs[0].addr & 0x7f) << 1;
    struct i2c_msg *msg;
    int i, read = 1, addr_len;
    int fifo_cmd;

    i2c_dbg("%s() %d\n", __FUNCTION__, __LINE__);
    asoc_i2c_writel(dev, I2C_CTL_IRQE | I2C_CTL_EN, I2C_CTL);

#ifdef I2C_DEBUG_INFO
    asoc_i2c_printifo(dev);
#endif

    if (num == 1) {
        /* 1 message */
        if (!(msgs[0].flags & I2C_M_RD)) {
            /* for write to device */
            asoc_i2c_writel(dev, addr, I2C_TXDAT);
            read = 0;
        }

        addr_len = 1;
        msg = &msgs[0];
        dev->msg_idx = 0;
    } else {
        /* 2 messages for read from device */

        /* write address */
        asoc_i2c_writel(dev, addr, I2C_TXDAT);

        /* write internal register address */
        for (i = 0; i < msgs[0].len; i++)
            asoc_i2c_writel(dev, msgs[0].buf[i], I2C_TXDAT);

        /* internal register address length +  1 byte device address */
        addr_len = msgs[0].len + 1;
        msg = &msgs[1];
        dev->msg_idx = 1;
    }

    i2c_dbg("%s() %d: msg->len %d\n", __FUNCTION__, __LINE__, msg->len);
    asoc_i2c_writel(dev, msg->len, I2C_DATCNT);

    if (read) {
        /* write restart with WR address */
        asoc_i2c_writel(dev, addr | 1, I2C_TXDAT);

        /* read from device */
        fifo_cmd = I2C_CMD_EXEC | I2C_CMD_MSS | I2C_CMD_SE | I2C_CMD_NS | I2C_CMD_DE | \
                   I2C_CMD_AS(addr_len) | I2C_CMD_SBE;
        if (num == 2)
            fifo_cmd |= I2C_CMD_SAS(1) | I2C_CMD_RBE;       /* 0x8f35 */

        dev->state = FIFO_READSTATE;
    } else {
        /* write to device */
        while (!(I2C_FIFOSTAT_TFF & asoc_i2c_readl(dev, I2C_FIFOSTAT))) {
            if (dev->msg_ptr >= msg->len)
                break;

            i2c_dbg("%s(): [i2c%d] fifostat %x, write dev->msg_ptr %d: %x\n", __FUNCTION__, 
                dev->adapter.nr,
                asoc_i2c_readl(dev, I2C_FIFOSTAT),
                dev->msg_ptr,
                msg->buf[dev->msg_ptr]);

            asoc_i2c_writel(dev, msg->buf[dev->msg_ptr++], I2C_TXDAT);
        }

        fifo_cmd = I2C_CMD_EXEC | I2C_CMD_MSS | I2C_CMD_SE | I2C_CMD_NS | I2C_CMD_DE | \
                   I2C_CMD_AS(1) | I2C_CMD_SBE;             /* 0x8f03 */

        dev->state = FIFO_WRITESTATE;
    }

    /* don't care NACK for hdmi device */
    if (msg->flags & I2C_M_IGNORE_NAK)
        asoc_i2c_writel(dev,
            asoc_i2c_readl(dev, I2C_FIFOCTL) | I2C_FIFOCTL_NIB, I2C_FIFOCTL);
    else
        asoc_i2c_writel(dev,
            asoc_i2c_readl(dev, I2C_FIFOCTL) & ~I2C_FIFOCTL_NIB, I2C_FIFOCTL);

    /* write fifo command to start transfer */
    asoc_i2c_writel(dev, fifo_cmd, I2C_CMD);

    i2c_dbg("%s() %d\n", __FUNCTION__, __LINE__);

#ifdef I2C_DEBUG_INFO
    asoc_i2c_printifo(dev);
#endif

    i2c_dbg("%s() end\n", __FUNCTION__);
}

/*
 * check if we can use the fifo mode to transfer data
 *
 * For simplicity, only support the following data pattern:
 *  1) 1 message,  msg[0] write (MAX 120bytes)
 *  2) 1 message,  msg[0] read  (MAX 120bytes)
 *  3) 2 messages, msg[0] write (MAX 7bytes), msg[1] read (MAX 120bytes)
 */
static int can_use_fifo_trans(struct asoc_i2c_dev *dev,
                              struct i2c_msg *msgs, int num)
{
    int pass = 0;


    i2c_dbg("%s(): msgs[0].flags 0x%x, msgs[0].len %d\n", 
        __FUNCTION__, msgs[0].flags, msgs[0].len);

    switch (num) {
    case 1:
        /* 1 message must be write to device */
        pass = 1;
        break;
    case 2:
        /* 2 message must be read from device */
        if ((msgs[0].flags & I2C_M_RD) || !(msgs[1].flags & I2C_M_RD)) {
            i2c_warn("%s(): cannot use fifo mode, msgs[0].flags 0x%x, msgs[0].flags 0x%x\n", 
                __FUNCTION__, msgs[0].flags, msgs[1].flags);
            break;
        }
        pass = 1;
        break;
    default:
        i2c_dbg("%s(): skip msg (num %d)\n", __FUNCTION__, num);
    }

#ifdef ASOC_I2C_SUPPORT_HDMI_NOFIFO
    /* don't use FIFO mode for hdmi */
    if (dev->i2c_freq == I2C_CLK_HDMI && dev->hdmi_nofifo != 0) {
        pass = 0;
    }
#endif

    i2c_dbg("%s() pass %d\n", __FUNCTION__, pass);

    return pass;
}
#else
#define asoc_i2c_message_fifo_start(dev, msgs, num) do { } while (0)
#define can_use_fifo_trans(dev, msgs, num)          (0)
#endif


static int asoc_doxfer(struct asoc_i2c_dev *dev, struct i2c_msg *msgs, int num)
{
    unsigned long timeout;
    int ret = 0;
    int i;

    spin_lock_irq(&dev->lock);
    dev->state = STARTSTATE;
    dev->msgs = msgs;
    dev->msg_num = num;
    dev->msg_idx = dev->msg_ptr = 0;
    spin_unlock_irq(&dev->lock);

    i2c_dbg("%s(): msg num %d\n", __FUNCTION__, num);

    for (i = 0; i < num; i++) {
        i2c_dbg("  msg[%d]: addr 0x%x, len %d, flags 0x%x\n", 
            i, msgs[i].addr, msgs[i].len, msgs[i].flags);
        asoc_dump_mem(msgs[i].buf, msgs[i].len, 0, 1);
    }

    if (can_use_fifo_trans(dev, msgs, num))
        asoc_i2c_message_fifo_start(dev, msgs, num);
    else
        asoc_i2c_message_start(dev, msgs);

    timeout = wait_event_timeout(dev->waitqueue,
                dev->msg_num == 0, I2C_TRANS_TIMEOUT);
    if ( !timeout ) {
        ret = -EAGAIN;
        i2c_warn("Timedout..");
        goto out;
    }

    if ( dev->msg_idx < 0) {
        ret = -EAGAIN;
        asoc_i2cdev_init(dev);
        i2c_warn("Transition failed");
    } else {
        ret = dev->msg_idx;
    }

out:
    /* disable i2c after transfer */
    asoc_i2c_writel(dev, 0, I2C_CTL);

    return ret;
}

static int asoc_irq_nextbyte(struct asoc_i2c_dev *dev, unsigned long status)
{
    u8 byte, ctl_reg, spec = 0;
    int ret = 0;
    unsigned long flags;

    i2c_dbg("%s(): status 0x%x, dev->state 0x%x\n", __FUNCTION__, 
        status, dev->state);
#ifdef I2C_DEBUG_INFO
    asoc_i2c_printifo(dev);
#endif

    spin_lock_irqsave(&dev->lock, flags);
    switch (dev->state) {
        case STOPSTATE:
            i2c_dbg("%s(): STOPSTATE\n", __FUNCTION__);

//            asoc_i2c_disable_irq(dev);
            goto out;
        case STARTSTATE:
            i2c_dbg("%s(): STARTSTATE\n", __FUNCTION__);

            if ( dev->msgs->flags & I2C_M_RD ) {
                dev->state = READSTATE;
            } else {
                dev->state = WRITESTATE;
            }

            if ( dev->msgs->len == 0 ) {
                asoc_i2c_stop(dev, 0);
                goto out;
            }

        i2c_dbg("%s(): -> dev->state 0x%x\n", __FUNCTION__, 
            dev->state);

            if ( dev->state == READSTATE ) {
                goto pre_read;
            } else if ( dev->state == WRITESTATE ) {
                goto retry_write;
            }

        case READSTATE:

            i2c_dbg("%s(): READSTATE\n", __FUNCTION__);

            byte = asoc_i2c_readl(dev, I2C_RXDAT);
            dev->msgs->buf[dev->msg_ptr++] = byte;

pre_read:
            i2c_dbg("%s(): READSTATE - %d, %d, %d\n", __FUNCTION__,
                isMsgEnd(dev), isMsgFinish(dev), isLastMsg(dev));

            if ( isMsgEnd(dev) ) {
                spec = I2C_CTL_GRAS;
            } else if (isMsgFinish(dev)) {
                if ( !isLastMsg(dev) ) {
                    dev->msgs++;
                    dev->msg_idx++;
                    dev->msg_ptr = 0;
                    dev->state = STARTSTATE;
                    asoc_i2c_message_restart(dev, dev->msgs);
                    goto out;
                } else {
                    asoc_i2c_stop(dev, 0);
                    goto out;
                }
            }
            break;

        case WRITESTATE:
            i2c_dbg("%s(): WRITESTATE\n", __FUNCTION__);

retry_write:
            i2c_dbg("%s(): WRITESTATE - %d, %d, %d\n", __FUNCTION__,
                isMsgEnd(dev), isMsgFinish(dev), isLastMsg(dev));

            if (!isMsgFinish(dev)) {

#ifdef I2C_DEBUG_INFO
    asoc_i2c_printifo(dev);
#endif
                byte = dev->msgs->buf[dev->msg_ptr++];
                asoc_i2c_writel(dev, byte, I2C_TXDAT);

#ifdef I2C_DEBUG_INFO
    mdelay(1);
    asoc_i2c_printifo(dev);
#endif
            } else if (!isLastMsg(dev)) {
                dev->msgs++;
                dev->msg_idx++;
                dev->msg_ptr = 0;
                dev->state = STARTSTATE;
                asoc_i2c_message_restart(dev, dev->msgs);
                goto out;
            } else {
                asoc_i2c_stop(dev, 0);
                goto out;
            }

            break;
        default:
            i2c_warn("Invalid State..");
            ret = -EINVAL;
            break;
    }

    ctl_reg = (asoc_i2c_readl(dev, I2C_CTL) & ~I2C_CTL_GBCC_MASK)
                | I2C_CTL_GBCC_NONE | I2C_CTL_RB | spec;
    asoc_i2c_writel(dev, ctl_reg, I2C_CTL);

mdelay(1);
#ifdef I2C_DEBUG_INFO
    asoc_i2c_printifo(dev);
#endif
out:
    spin_unlock_irqrestore(&dev->lock, flags);
    return ret;
}

#ifdef ASOC_I2C_SUPPORT_FIFO

static int fifo_write_irq(struct asoc_i2c_dev *dev)
{
    struct i2c_msg *msg = &dev->msgs[dev->msg_idx];

    i2c_dbg("%s(): [i2c%d] fifo write, msg->len %d, dev->msg_ptr %d, fifostat %x, rcnt %d\n", __FUNCTION__, 
        dev->adapter.nr,
        msg->len, 
        dev->msg_ptr, 
        asoc_i2c_readl(dev, I2C_FIFOSTAT),
        asoc_i2c_readl(dev, I2C_RCNT));

    BUG_ON(msg->len < dev->msg_ptr);

    while (!(I2C_FIFOSTAT_TFF & asoc_i2c_readl(dev, I2C_FIFOSTAT))) {
        if (dev->msg_ptr >= msg->len)
            break;

        i2c_dbg("%s(): [i2c%d] fifostat %x, write dev->msg_ptr %d: %x\n", __FUNCTION__, 
            dev->adapter.nr,
            asoc_i2c_readl(dev, I2C_FIFOSTAT),
            dev->msg_ptr,
            msg->buf[dev->msg_ptr]);

        asoc_i2c_writel(dev, msg->buf[dev->msg_ptr++], I2C_TXDAT);
    }

    if (msg->len == dev->msg_ptr && (I2C_FIFOSTAT_CECB & asoc_i2c_readl(dev, I2C_FIFOSTAT))) {
        i2c_dbg("%s(): [i2c%d] end fifo write, msg->len %d, fifostat %x, %d/%d\n", __FUNCTION__, 
            dev->adapter.nr,
            msg->len, 
            asoc_i2c_readl(dev, I2C_FIFOSTAT),
            asoc_i2c_readl(dev, I2C_DATCNT) - asoc_i2c_readl(dev, I2C_RCNT),
            asoc_i2c_readl(dev, I2C_DATCNT));
        asoc_i2c_reset(dev);
        asoc_master_trans_completion(dev, 0);
    }

    return 0;
}


static int fifo_read_irq(struct asoc_i2c_dev *dev, int stop_detected)
{
    struct i2c_msg *msg = &dev->msgs[dev->msg_idx];
    int i, byte, len;
    unsigned int fifostat;

    i2c_dbg("%s(): [i2c%d] fifo read, msg->len %d, dev->msg_ptr %d, fifostat %x\n", __FUNCTION__, 
        dev->adapter.nr,
        msg->len, 
        dev->msg_ptr, 
        asoc_i2c_readl(dev, I2C_FIFOSTAT));

    BUG_ON(msg->len < dev->msg_ptr);

    if (msg->len > dev->msg_ptr) {
        while (I2C_FIFOSTAT_RFE & asoc_i2c_readl(dev, I2C_FIFOSTAT)) {
            i2c_dbg("%s(): [i2c%d] fifostat %x, stat %x\n", __FUNCTION__, 
                dev->adapter.nr,
                asoc_i2c_readl(dev, I2C_FIFOSTAT),
                asoc_i2c_readl(dev, I2C_STAT));

            byte = asoc_i2c_readl(dev, I2C_RXDAT);
            msg->buf[dev->msg_ptr++] = byte;

            i2c_dbg("%s(): [i2c%d] read to dev->msg_ptr %d: 0x%02x\n", __FUNCTION__, 
                dev->adapter.nr,
                dev->msg_ptr - 1,
                byte);
        }
    }

    if (msg->len == dev->msg_ptr && stop_detected) {
        i2c_dbg("%s(): [i2c%d] finish fifo read, msg->len %d, fifostat %x, stat %x, %d/%d\n", __FUNCTION__, 
            dev->adapter.nr,
            msg->len, 
            asoc_i2c_readl(dev, I2C_FIFOSTAT),
            asoc_i2c_readl(dev, I2C_STAT),
            asoc_i2c_readl(dev, I2C_DATCNT) - asoc_i2c_readl(dev, I2C_RCNT),
            asoc_i2c_readl(dev, I2C_DATCNT));

        asoc_i2c_reset(dev);
        asoc_master_trans_completion(dev, 0);
    }

    return 0;
}

static int asoc_fifo_irq(struct asoc_i2c_dev *dev, int stop_detected)
{
    struct i2c_msg *msg = &dev->msgs[dev->msg_idx];
    int i, byte, len;
    unsigned int fifostat;

    i2c_dbg("%s(): fifo mode, state %d, msg_idx %d, len %d, stop_detected %d\n", 
        __FUNCTION__, dev->state, dev->msg_idx, msg->len, stop_detected);

    if (dev->msg_idx >= 2) {
        i2c_warn("%s(): [i2c%d] i2c bus error! I2C_CTL 0x%x, I2C_STAT 0x%x, fifostat 0x%x\n", 
            __FUNCTION__,
            dev->adapter.nr,
            asoc_i2c_readl(dev, I2C_CTL),
            asoc_i2c_readl(dev, I2C_STAT),
            asoc_i2c_readl(dev, I2C_FIFOSTAT));

        asoc_i2c_reset(dev);
        asoc_master_trans_completion(dev, -ENXIO);
        return -1;
    }

    fifostat = asoc_i2c_readl(dev, I2C_FIFOSTAT);
    if (fifostat & I2C_FIFOSTAT_RNB) {
        i2c_warn("%s(): [i2c%d] no ACK, fifostat 0x%x\n", __FUNCTION__, 
            dev->adapter.nr,
            fifostat);
        asoc_i2c_reset(dev);
        asoc_master_trans_completion(dev, -ENXIO);
        return -1;
    }
    
    if (dev->state == FIFO_WRITESTATE)
        fifo_write_irq(dev);
    else if (dev->state == FIFO_READSTATE)
        fifo_read_irq(dev, stop_detected);
    else 
        BUG_ON(1);

    i2c_dbg("%s() %d:\n", __FUNCTION__, __LINE__);

    return 0;
}
#else
#define asoc_fifo_irq(dev)          do { } while (0)
#endif

static irqreturn_t asoc_i2c_interrupt(int irq, void *dev_id)
{
    struct asoc_i2c_dev *dev = dev_id;
    unsigned long status = 0, ctl_reg = 0;
    int flags, stop_detected;
    
    i2c_dbg("%s(): irq %d, I2C_STAT 0x%x, I2C_FIFOSTAT 0x%x, dev->state %d\n", 
        __FUNCTION__, 
        irq, asoc_i2c_readl(dev, I2C_STAT),
        asoc_i2c_readl(dev, I2C_FIFOSTAT), dev->state);

    stop_detected = (asoc_i2c_readl(dev, I2C_STAT) & I2C_STAT_STPD) ? 1 : 0;

    /* clear STPD/IRQP */
    asoc_i2c_clear_tcb(dev);

    if (dev->state == FIFO_READSTATE || dev->state == FIFO_WRITESTATE) {
        asoc_fifo_irq(dev, stop_detected);
        goto out;
    }

    status = asoc_i2c_readl(dev, I2C_STAT);
    if (status & I2C_BUS_ERR_MSK) {
        i2c_warn("I2C trans failed <stat: 0x%lx>", status);
        goto out;
    }

    if (dev->state == STOPSTATE)
        goto out;

    if (dev->msgs == NULL) {
        i2c_warn("I2C: skip spurious interrupt, status 0x%x\n", status);
        goto out;
    }

    flags = dev->msgs->flags;

    ctl_reg = asoc_i2c_readl(dev, I2C_CTL);
    if (!(flags & I2C_M_RD) && !(flags & I2C_M_IGNORE_NAK)) {
        if ( status & I2C_STAT_RACK) {
            i2c_dbg("ACK\n");                   
        } else {                    
            i2c_warn("No Ack\n");
            goto no_ack;
        }
    }

    asoc_irq_nextbyte(dev, status);

out:
    return IRQ_HANDLED;
no_ack:
    asoc_i2c_stop(dev, -ENXIO);
    goto out;
}

static int asoc_i2c_xfer(struct i2c_adapter *adap,
    struct i2c_msg *pmsg, int num)
{
    struct asoc_i2c_dev *dev = i2c_get_adapdata(adap);
    int ret = 0;
    unsigned int freq;
#ifdef ASOC_I2C_SUPPORT_FREQ_LIMIT
    unsigned int freq_limit;
#endif

    if (!adap || !pmsg)
        return -EINVAL;

    if (mutex_lock_interruptible(&dev->mutex) < 0)
        return -EAGAIN;

    /* Sometimes the TP i2c address is 0x30 at I2C1 */
    if ((adap->nr != 1) && 
        (((0x60 >> 1) == pmsg->addr) || 
        ((0xA0 >> 1) == pmsg->addr) || 
        ((0x74 >> 1) == pmsg->addr))) {
        if (I2C_CLK_HDMI != dev->i2c_freq) {
            dev->i2c_freq = I2C_CLK_HDMI;
        }
    } else {
        dev->i2c_freq = dev->i2c_freq_cfg;
    }

#ifdef ASOC_I2C_SUPPORT_FREQ_LIMIT
    freq_limit = asoc_i2c_get_speed_limit(dev, pmsg->addr);
    if (freq_limit > 0 && dev->i2c_freq > freq_limit) {
        i2c_dbg("%s: i2c%d: set limit addr %d, freq %d\n",
            __FUNCTION__,
            adap->nr,
            pmsg->addr, freq_limit);
        dev->i2c_freq = freq_limit;
    }
#endif

    asoc_i2cdev_init(dev);
    ret = asoc_doxfer(dev, pmsg, num);

    mutex_unlock(&dev->mutex);

    return ret;
}

static u32 asoc_i2c_func(struct i2c_adapter *adapter)
{
    return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static struct i2c_algorithm asoc_i2c_algo = {
    .master_xfer    = asoc_i2c_xfer,
    .functionality  = asoc_i2c_func,
};

static int asoc_i2c_probe(struct platform_device *pdev)
{
    struct asoc_i2c_dev *dev;
    struct i2c_adapter  *adap;
    struct resource     *mem, *irq, *ioarea;
    int ret = 0;
    unsigned int freq;
#ifdef ASOC_I2C_SUPPORT_HDMI_NOFIFO
    int hdcp_switch;
#endif
#ifdef ASOC_I2C_SUPPORT_FREQ_LIMIT
    unsigned int *cfg_array;
    int i;
#endif

    i2c_dbg("%s-%d", __func__, __LINE__);

    mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!mem) {
        dev_err(&pdev->dev, "no mem resource?\n");
        return -ENODEV;
    }

    irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if (!irq) {
        dev_err(&pdev->dev, "no irq resource?\n");
        return -ENODEV;
    }

    ioarea = request_mem_region(mem->start, resource_size(mem),
            pdev->name);
    if (!ioarea) {
        dev_err(&pdev->dev, "I2C region already claimed\n");
        return -EBUSY;
    }

    dev = (struct asoc_i2c_dev *)kzalloc(sizeof(struct asoc_i2c_dev), GFP_KERNEL);
    if ( !dev ) {
        i2c_warn("alloc i2c device failed");
        ret = -ENOMEM;
        goto err_release_region;
    }

    mutex_init(&dev->mutex);
    dev->dev = &pdev->dev;
    dev->irq = irq->start;

    spin_lock_init(&dev->lock);

    dev->phys = (unsigned long)mem->start;
    if (!dev->phys) {
        ret = -ENOMEM;
        goto err_free_mem;
    }
    dev->base = (void __iomem *)IO_ADDRESS(dev->phys);

    platform_set_drvdata(pdev, dev);
    init_waitqueue_head(&dev->waitqueue);

    ret = request_irq(dev->irq, asoc_i2c_interrupt, IRQF_DISABLED, pdev->name, dev);
    if ( ret ) {
        dev_err(dev->dev, "failure requesting irq %i\n", dev->irq);
        goto err_unuse_clocks;
    }

    adap = &dev->adapter;
    i2c_set_adapdata(adap, dev);
    adap->owner = THIS_MODULE;
    adap->class = I2C_CLASS_HWMON;
    strlcpy(adap->name, "ASOC I2C adapter", sizeof(adap->name));
    adap->algo = &asoc_i2c_algo;
    adap->dev.parent = &pdev->dev;

    adap->nr = pdev->id;

    /* use default freqence */
    dev->i2c_freq_cfg = I2C_CLK_FAST;
    switch(adap->nr) {
	    case 0:
        ret = get_config("i2c.i2c0_freq", (char *)(&freq), sizeof(unsigned int));
        if (ret == 0) {
            printk("i2c.i2c1_freq: %d\n", freq);
            dev->i2c_freq_cfg = freq;
        }
        break;
    case 1:
        ret = get_config("i2c.i2c1_freq", (char *)(&freq), sizeof(unsigned int));
        if (ret == 0) {
            printk("i2c.i2c1_freq: %d\n", freq);
            dev->i2c_freq_cfg = freq;
        }
        break;
    case 2:
        ret = get_config("i2c.i2c2_freq", (char *)(&freq), sizeof(unsigned int));
        if (ret == 0) {
            printk("i2c.i2c2_freq: %d\n", freq);
            dev->i2c_freq_cfg = freq;
        }
        break;
    default:
        i2c_warn("Invalid adapter..");
        return -ENODEV;
    }
    dev->i2c_freq = dev->i2c_freq_cfg;

#ifdef ASOC_I2C_SUPPORT_HDMI_NOFIFO
    ret = get_config("hdmi.hdcp_switch", (char *)(&hdcp_switch), sizeof(int));
    if (ret == 0) {
        printk("i2c: use no fifo mode for hdmi hdcp\n");
        dev->hdmi_nofifo = hdcp_switch;
    } else {
        dev->hdmi_nofifo = 0;
    }
#endif

#ifdef ASOC_I2C_SUPPORT_FREQ_LIMIT
    cfg_array = (unsigned int *)kzalloc(
        NUM_OF_MAX_FREQ_LIMITS * 3 * sizeof(unsigned int), GFP_KERNEL);
	ret = get_config("i2c.freq_limits", cfg_array,
        NUM_OF_MAX_FREQ_LIMITS * 3 * sizeof(unsigned int));
	if (ret == 0) {
        printk("i2c: i2c.freq_limits initialize\n");

        for (i = 0; i < NUM_OF_MAX_FREQ_LIMITS; i++) {

            if ((cfg_array[3 * i] > 0) && (cfg_array[3 * i] < 0x7f)) {
                dev->freq_limits[i].chan = cfg_array[3 * i];
                dev->freq_limits[i].addr = cfg_array[3 * i + 1];
                dev->freq_limits[i].freq = cfg_array[3 * i + 2];

                printk("    i2c%d: addr %d, freq_limit %d\n",
                    dev->freq_limits[i].chan,
                    dev->freq_limits[i].addr,
                    dev->freq_limits[i].freq);

            }
        }
	}
    kfree(cfg_array);
#endif

    asoc_i2cdev_init(dev);
    ret = i2c_add_numbered_adapter(adap);
    if (ret) {
        dev_err(dev->dev, "failure adding adapter\n");
        goto err_free_irq;
    }
    i2c_dbg("%s %d\n", __func__,__LINE__);
#ifdef I2C_DEBUG_INFO
    asoc_i2c_printifo(dev);
#endif

    return 0;

err_free_irq:
    free_irq(dev->irq, dev);
err_unuse_clocks:
//err_iounmap:
//    iounmap(dev->base);
err_free_mem:
    platform_set_drvdata(pdev, NULL);
    kfree(dev);
err_release_region:
    release_mem_region(mem->start, resource_size(mem));

    return ret;
}

static int asoc_i2c_remove(struct platform_device *pdev)
{
    struct  asoc_i2c_dev    *dev = platform_get_drvdata(pdev);
    struct resource     *mem;
    int rc;

    i2c_dbg("%s %d\n", __func__,__LINE__);
    platform_set_drvdata(pdev, NULL);
    free_irq(dev->irq, dev);
    rc=i2c_del_adapter(&dev->adapter);
    asoc_i2c_disable(dev);
    kfree(dev);
    mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    release_mem_region(mem->start, (mem->end - mem->start) + 1);

    return rc;
}
extern int get_pmu_i2c_num(void);
static int asoc_i2c_suspend(struct platform_device *pdev, pm_message_t state)
{
    struct asoc_i2c_dev *dev = platform_get_drvdata(pdev);

	//TODO: debug code here for on/off, skip i2c0 suspend.
	if (dev->adapter.nr == get_pmu_i2c_num())
	{
		printk("pmu i2c%d not suspend %s %d\n",dev->adapter.nr, __FUNCTION__,__LINE__);
		return 0;
	}
	
    if ( mutex_lock_interruptible(&dev->mutex) < 0 )
        return -EAGAIN;

    asoc_i2c_disable(dev);
    mutex_unlock(&dev->mutex);

    return 0;
}

static int asoc_i2c_resume(struct platform_device *pdev)
{
    struct asoc_i2c_dev *dev = platform_get_drvdata(pdev);

    if ( mutex_lock_interruptible(&dev->mutex) < 0 )
        return -EAGAIN;

    asoc_i2c_hwinit(dev);
    mutex_unlock(&dev->mutex);

    return 0;
}

static struct platform_driver asoc_i2c_driver = {
    .probe      = asoc_i2c_probe,
    .remove     = __devexit_p(asoc_i2c_remove),
    .suspend    = asoc_i2c_suspend,
    .resume     = asoc_i2c_resume,
    .driver     = {
        .name   = "asoc-i2c",
        .owner  = THIS_MODULE,
    },
};

static struct resource i2c_resources[][2] = {
    {
        {
            .start  = (I2C0_BASE),
            .end    = (I2C0_BASE) + 0xfff,
            .flags  = IORESOURCE_MEM,
        },
        {
            .start  = (IRQ_ASOC_I2C0),
            .flags  = IORESOURCE_IRQ,
        },
    },

    {
        {
            .start  = (I2C1_BASE),
            .end    = (I2C1_BASE) + 0xfff,
            .flags  = IORESOURCE_MEM,
        },
        {
            .start  = (IRQ_ASOC_I2C1),
            .flags  = IORESOURCE_IRQ,
        },
    },

    {
        {
            .start  = (I2C2_BASE),
            .end    = (I2C2_BASE) + 0xfff,
            .flags  = IORESOURCE_MEM,
        },
        {
            .start  = (IRQ_ASOC_I2C2),
            .flags  = IORESOURCE_IRQ,
        },
    },
};

static struct asoc_i2c_bus_platform_data i2c_pdata[ARRAY_SIZE(i2c_resources)];
static struct platform_device asoc_i2c_device[] = {
#if defined(CONFIG_I2C_ASOC_BUS0)
    {
        .name   = "asoc-i2c",
        .id =  0,
        .num_resources  = ARRAY_SIZE( i2c_resources[0]),
        .resource   = ( i2c_resources[0]),
        .dev        = {
            .platform_data  = &i2c_pdata[0],
        },
	},
#endif
#if defined(CONFIG_I2C_ASOC_BUS1)
    {
        .name   = "asoc-i2c",
        .id =  1,
        .num_resources  = ARRAY_SIZE( i2c_resources[1]),
        .resource   = ( i2c_resources[1]),
        .dev        = {
            .platform_data  = &i2c_pdata[1],
        },
      },
#endif
#if defined(CONFIG_I2C_ASOC_BUS2)
    {
        .name   = "asoc-i2c",
        .id = 2,
        .num_resources  = ARRAY_SIZE( i2c_resources[2]),
        .resource   = ( i2c_resources[2]),
        .dev        = {
            .platform_data  = &i2c_pdata[2],
        },
    },
#endif
};

#include <mach/gpio.h>
#include <linux/i2c-gpio.h>

#define I2C_GPIO_MAX 	3

unsigned int bus_count;
static struct i2c_gpio_platform_data i2c_gpio_data[I2C_GPIO_MAX];

static struct platform_device gpio_i2c_device[I2C_GPIO_MAX] = {
    {
        .name   = "i2c-gpio",
        .id = 10,
        .dev        = {
            .platform_data  = &i2c_gpio_data[0],
        },
    },		
    {
        .name   = "i2c-gpio",
        .id = 11,
        .dev        = {
            .platform_data  = &i2c_gpio_data[1],
        },
    },		
    {
        .name   = "i2c-gpio",
        .id = 12,
        .dev        = {
            .platform_data  = &i2c_gpio_data[2],
        },	
	},
};

int register_gpio_i2c(void)
{
	int ret;
	unsigned int i;
	char gpio_sda_name[]="i2c-gpio-10-sda";
	char gpio_scl_name[]="i2c-gpio-10-scl";
	struct gpio_pre_cfg pcfg;
	
	ret = get_config("i2c-gpio.count", (char *)(&bus_count), sizeof(unsigned int));
	if (ret != 0) {
		return -1;
	}	
	if(bus_count == 0)
	{
		return 0;
	}
	if(bus_count > I2C_GPIO_MAX)
	{
		printk("too many i2c-gpio:%d > max(%d)\n", bus_count, I2C_GPIO_MAX);
		return -2;
	}
	for(i=0; i<bus_count ;i++)
	{
		gpio_sda_name[10] = '0' + i;
		memset((void *)&pcfg, 0, sizeof(struct gpio_pre_cfg));
		gpio_get_pre_cfg(gpio_sda_name, &pcfg);
		i2c_gpio_data[i].sda_pin = ASOC_GPIO_PORT(pcfg.iogroup, pcfg.pin_num);		
		printk(KERN_DEBUG "i2c-gpio[%d]_sda_pin, pcfg.iogroup:%d,  pcfg.pin_num:%d\n", i, pcfg.iogroup, pcfg.pin_num);
		
		gpio_scl_name[10] = '0' + i;
		memset((void *)&pcfg, 0, sizeof(struct gpio_pre_cfg));
		gpio_get_pre_cfg(gpio_scl_name, &pcfg);
		i2c_gpio_data[i].scl_pin = ASOC_GPIO_PORT(pcfg.iogroup, pcfg.pin_num);	
		printk(KERN_DEBUG "i2c-gpio[%d]_scl_pin, pcfg.iogroup:%d,  pcfg.pin_num:%d\n", i, pcfg.iogroup, pcfg.pin_num);	

		platform_device_register(&gpio_i2c_device[i]);
	}
	return 0;
}

static int __init asoc_i2c_init(void)
{
    int ret, i;

    /* initialize mutex */
    for(i=0;i<ARRAY_SIZE(asoc_i2c_device);i++)
        platform_device_register(&asoc_i2c_device[i]);
	
	register_gpio_i2c();

    ret = platform_driver_register(&asoc_i2c_driver);

    return ret;
}

static void __exit asoc_i2c_exit(void)
{
    int i;

    platform_driver_unregister(&asoc_i2c_driver);
    for( i = 0; i < ARRAY_SIZE(asoc_i2c_device); i++)
        platform_device_unregister(&asoc_i2c_device[i]);
    for( i = 0; i < bus_count; i++)
        platform_device_unregister(&gpio_i2c_device[i]);		
}

MODULE_AUTHOR("lzhou");
MODULE_DESCRIPTION("I2C driver for Actions SOC");
MODULE_LICENSE("GPL");

subsys_initcall(asoc_i2c_init);
module_exit(asoc_i2c_exit);

