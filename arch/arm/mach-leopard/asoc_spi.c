/*
 * arch/arm/mach-leopard/asoc_spi.c
 *
 * serial driver for Actions SOC
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/dma-mapping.h>
#include <linux/spi/spi.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/clock.h>
#include <mach/spi.h>
#include <mach/dma.h>

extern int get_config(const char *key, char *buff, int len);

//#define DEBUG

#define ULOG(lev, aspi, fmt, args...) \
    do {                        \
        if (aspi->master->bus_num == 0)    \
            printk(lev "%s: [spi%d] "fmt, __FUNCTION__, aspi->master->bus_num, ## args); \
    } while (0)

#ifdef DEBUG
#define LOGD(aspi, fmt, args...)    ULOG(KERN_DEBUG, aspi, fmt, ## args)
#define LOGI(aspi, fmt, args...)    ULOG(KERN_INFO, aspi, fmt, ## args)
#else
#define LOGD(aspi, fmt, args...)    do {} while (0)
#define LOGI(aspi, fmt, args...)    do {} while (0)
#endif

#define LOGE(aspi, fmt, args...)    ULOG(KERN_ERR, aspi, fmt, ## args)


#define ASOC_MAX_SPI                    4

#define SPI_CTL                         0x0000
#define SPI_CLKDIV                      0x0004
#define SPI_STAT                        0x0008
#define SPI_RXDAT                       0x000C
#define SPI_TXDAT                       0x0010
#define SPI_TCNT                        0x0014
#define SPI_SEED                        0x0018
#define SPI_TXCR                        0x001C
#define SPI_RXCR                        0x0020

/* SPI_CTL */
#define SPI_CTL_SDT_MASK                (0x7 << 29)
#define SPI_CTL_SDT(x)                  (((x) & 0x7) << 29)
#define SPI_CTL_BM                      (0x1 << 28)
#define SPI_CTL_GM                      (0x1 << 27)
#define SPI_CTL_CEB                     (0x1 << 26)
#define SPI_CTL_RANEN(x)                (0x1 << 24)
#define SPI_CTL_RDIC_MASK               (0x3 << 22)
#define SPI_CTL_RDIC(x)                 (((x) & 0x3) << 22)
#define SPI_CTL_TDIC_MASK               (0x3 << 20)
#define SPI_CTL_TDIC(x)                 (((x) & 0x3) << 20)
#define SPI_CTL_TWME                    (0x1 << 19)
#define SPI_CTL_EN                      (0x1 << 18)
#define SPI_CTL_RWC_MASK                (0x3 << 16)
#define SPI_CTL_RWC(x)                  (((x) & 0x3) << 16)
#define SPI_CTL_DTS                     (0x1 << 15)
#define SPI_CTL_SSATEN                  (0x1 << 14)
#define SPI_CTL_DM_MASK                 (0x3 << 12)
#define SPI_CTL_DM(x)                   (((x) & 0x3) << 12)
#define SPI_CTL_LBT                     (0x1 << 11)
#define SPI_CTL_MS                      (0x1 << 10)
#define SPI_CTL_DAWS_MASK               (0x3 << 8)
#define SPI_CTL_DAWS(x)                 (((x) & 0x3) << 8)
#define     SPI_CTL_DAWS_8BIT               (SPI_CTL_DAWS(0))
#define     SPI_CTL_DAWS_16BIT              (SPI_CTL_DAWS(1))
#define     SPI_CTL_DAWS_32BIT              (SPI_CTL_DAWS(2))
#define SPI_CTL_CPOS_MASK               (0x3 << 6)
#define SPI_CTL_CPOS(x)                 (((x) & 0x3) << 6)
#define     SPI_CTL_CPOS_CPHA               (0x1 << 7)
#define     SPI_CTL_CPOS_CPOL               (0x1 << 6)
#define SPI_CTL_LMFS                    (0x1 << 5)
#define SPI_CTL_SSCO                    (0x1 << 4)
#define SPI_CTL_TIEN                    (0x1 << 3)
#define SPI_CTL_RIEN                    (0x1 << 2)
#define SPI_CTL_TDEN                    (0x1 << 1)
#define SPI_CTL_RDEN                    (0x1 << 0)

/* SPI_CLKDIV */
#define SPI_CLKDIV_CLKDIV_MASK          (0x3FF << 0)
#define SPI_CLKDIV_CLKDIV(x)            (((x) & 0x3FF) << 0)

/******************************************************************************/
/*SPI_STAT*/
/*bit 10-31 Reserved*/
#define SPI_STAT_TFEM                   (0x1 << 9)
#define SPI_STAT_RFFU                   (0x1 << 8)
#define SPI_STAT_TFFU                   (0x1 << 7)
#define SPI_STAT_RFEM                   (0x1 << 6)
#define SPI_STAT_TFER                   (0x1 << 5)
#define SPI_STAT_RFER                   (0x1 << 4)
#define SPI_STAT_BEB                    (0x1 << 3)
#define SPI_STAT_TCOM                   (0x1 << 2)
#define SPI_STAT_TIP                    (0x1 << 1)
#define SPI_STAT_PIP                    (0x1 << 0)

#define msecs_to_loops(t) (loops_per_jiffy / 1000 * HZ * t)

#define INVALID_DMA_ADDRESS	0xffffffff

struct asoc_spi_data {
    struct spi_master *master;
    struct platform_device  *pdev;
    spinlock_t lock;

    struct clk *clk;
    void __iomem *base;
    unsigned long phys;
    int dma_chan;
    int use_dma;
	int rt;
    struct completion xfer_completion;

    unsigned int default_cs_gpio;

    unsigned int cur_speed;
    unsigned int cur_mode;
    unsigned int cur_bits_per_word;
};

struct asoc_spi_slave_data {
    unsigned int cs_gpio;
};

static inline unsigned int asoc_spi_readl(struct asoc_spi_data *aspi,
                    unsigned int reg)
{
    return __raw_readl(aspi->base + reg);
}

static inline void asoc_spi_writel(struct asoc_spi_data *aspi,
                    unsigned int val,
                    unsigned int reg)
{
    __raw_writel(val, aspi->base + reg);
}

static inline void dump_spi_registers(struct asoc_spi_data *aspi)
{
    pr_info("asoc_spi: SPI_CTL(0x%x) = 0x%x\n",
            (unsigned int)aspi->phys + SPI_CTL,
            asoc_spi_readl(aspi, SPI_CTL));
    pr_info("asoc_spi: SPI_STAT(0x%x) = 0x%x\n",
            (unsigned int)aspi->phys + SPI_STAT,
            asoc_spi_readl(aspi, SPI_STAT));
    pr_info("asoc_spi: SPI_CLKDIV(0x%x) = 0x%x\n",
            (unsigned int)aspi->base + SPI_CLKDIV,
            asoc_spi_readl(aspi, SPI_CLKDIV));

    pr_info("asoc_spi: SPI_TCNT(0x%x) = 0x%x\n",
            (unsigned int)aspi->phys + SPI_TCNT,
            asoc_spi_readl(aspi, SPI_TCNT));
    pr_info("asoc_spi: SPI_RXCR(0x%x) = 0x%x\n",
            (unsigned int)aspi->phys + SPI_RXCR,
            asoc_spi_readl(aspi, SPI_RXCR));
    pr_info("asoc_spi: SPI_TXCR(0x%x) = 0x%x\n",
            (unsigned int)aspi->phys + SPI_TXCR,
            asoc_spi_readl(aspi, SPI_TXCR));
}

static int xfer_can_use_dma(struct asoc_spi_data *aspi, struct spi_transfer *xfer)
{
    if (aspi->use_dma && xfer->len > 64)
        return 1;

    return 0;
}

static int asoc_spi_map_transfer(struct asoc_spi_data *aspi,
						struct spi_transfer *xfer)
{
	struct device *dev = &aspi->master->dev;

	/* First mark xfer unmapped */
    xfer->rx_dma = INVALID_DMA_ADDRESS;
    xfer->tx_dma = INVALID_DMA_ADDRESS;

    if (xfer->tx_buf != NULL) {
        xfer->tx_dma = dma_map_single(dev,
                (void *)xfer->tx_buf, xfer->len,
                DMA_TO_DEVICE);
        if (dma_mapping_error(dev, xfer->tx_dma)) {
            LOGE(aspi, "dma_map_single Tx failed, tx_buf 0x%p, len 0x%x\n", 
                xfer->tx_buf, xfer->len);
            xfer->tx_dma = INVALID_DMA_ADDRESS;
            return -ENOMEM;
        }
    }

    if (xfer->rx_buf != NULL) {
        xfer->rx_dma = dma_map_single(dev, xfer->rx_buf,
                    xfer->len, DMA_FROM_DEVICE);
        if (dma_mapping_error(dev, xfer->rx_dma)) {
            LOGE(aspi, "dma_map_single Rx failed, rx_buf 0x%p, len 0x%x\n", 
                xfer->rx_buf, xfer->len);

            dma_unmap_single(dev, xfer->tx_dma,
                    xfer->len, DMA_TO_DEVICE);
            xfer->tx_dma = INVALID_DMA_ADDRESS;
            xfer->rx_dma = INVALID_DMA_ADDRESS;
            return -ENOMEM;
        }
    }

	return 0;
}

static void asoc_spi_unmap_transfer(struct asoc_spi_data *aspi,
						struct spi_transfer *xfer)
{
	struct device *dev = &aspi->master->dev;

    if (xfer->rx_buf != NULL
            && xfer->rx_dma != INVALID_DMA_ADDRESS)
        dma_unmap_single(dev, xfer->rx_dma,
                    xfer->len, DMA_FROM_DEVICE);

    if (xfer->tx_buf != NULL
            && xfer->tx_dma != INVALID_DMA_ADDRESS)
        dma_unmap_single(dev, xfer->tx_dma,
                    xfer->len, DMA_TO_DEVICE);
}

static void __set_spi_dma_mode(struct asoc_spi_data *aspi, unsigned int dmanr, int rx)
{
    unsigned int spi_drq_id[ASOC_MAX_SPI] = {
        7, 8, 14, 15};
    unsigned int mode;
    int bus_num;

    bus_num = aspi->master->bus_num;

    if (bus_num >= ASOC_MAX_SPI)
        return;

    if (rx) {
        /* fix src address, uart mode enable, dst is ddr */
        mode = spi_drq_id[bus_num] | (1 << 6) | (18 << 16); 
    } else {
        /* fix dst address, uart mode enable, src is ddr */
        mode = (spi_drq_id[bus_num] | (1 << 6)) << 16 | (18 << 0);
    }
    set_dma_mode(dmanr, mode);
}

static void asoc_spi_dma_callback(int irq, void *dev_id)
{
    struct asoc_spi_data *aspi = (struct asoc_spi_data *)dev_id;
    unsigned int dma_chan = irq;    /* here actually irq is dma channel */

    LOGD(aspi, "%d: \n", __LINE__);

    clear_dma_tcirq_pend(dma_chan);

    complete(&aspi->xfer_completion);
}

static int asoc_spi_request_dma(struct asoc_spi_data *aspi)
{
    int dma_chan;

    dma_chan = request_asoc_dma(DMA_CHAN_TYPE_BUS,
        dev_name(&aspi->master->dev), asoc_spi_dma_callback, 0, aspi);
    if (dma_chan < 0) {
        LOGE(aspi, "cannot request dma channel, ret %d\n",
            dma_chan);
        return -1;
    }

    return dma_chan;
}

static void asoc_spi_free_dma(struct asoc_spi_data *aspi)
{
    if (aspi->dma_chan >= 0)
        free_asoc_dma(aspi->dma_chan);
}

static int asoc_spi_init_dma(struct asoc_spi_data *aspi,
						struct spi_transfer *xfer)
{
    set_dma_count(aspi->dma_chan, xfer->len);

    if (xfer->rx_buf) {
        set_dma_src_addr(aspi->dma_chan, aspi->phys + SPI_RXDAT);
        set_dma_dst_addr(aspi->dma_chan, xfer->rx_dma);
        __set_spi_dma_mode(aspi, aspi->dma_chan, 1);
    } else {
        set_dma_src_addr(aspi->dma_chan, xfer->tx_dma);
        set_dma_dst_addr(aspi->dma_chan, aspi->phys + SPI_TXDAT);
        __set_spi_dma_mode(aspi, aspi->dma_chan, 0);
    }

    return 0;
}

static void enable_cs(struct spi_device *spi)
{
    struct asoc_spi_data *aspi = spi_master_get_devdata(spi->master);
    struct asoc_spi_slave_data *sdata = spi_get_ctldata(spi);
    unsigned int ret, val;

    if (spi->chip_select) {
        BUG_ON(!sdata || sdata->cs_gpio >= ASOC_NR_GPIO);

        /*
         * deselect the default cs
         * switch the default cs pin to gpio, and pull to high level
         */
        if (aspi->default_cs_gpio < ASOC_NR_GPIO) {
            ret = gpio_request(aspi->default_cs_gpio, dev_name(&spi->dev));
            if (ret) {
                dev_err(&spi->dev, "%s: cs %d, failed to request default_cs_gpio %d\n", 
                    __FUNCTION__,
                    spi->chip_select, aspi->default_cs_gpio);
            }
            gpio_direction_output(aspi->default_cs_gpio, 1);
        }

        /* select the cs by gpio  */
        gpio_set_value(sdata->cs_gpio, 0);
    }

    /* select the default cs  */
    val = asoc_spi_readl(aspi, SPI_CTL);
    val &= ~SPI_CTL_SSCO;
    asoc_spi_writel(aspi, val, SPI_CTL);
}

static void disable_cs(struct spi_device *spi)
{
    struct asoc_spi_data *aspi = spi_master_get_devdata(spi->master);
    struct asoc_spi_slave_data *sdata = spi_get_ctldata(spi);
    unsigned int val;

    if (spi->chip_select) {
        BUG_ON(!sdata || sdata->cs_gpio >= ASOC_NR_GPIO);

        /* deselect the cs by gpio */
        gpio_set_value(sdata->cs_gpio, 1);

        /* restore the default cs to normal mfp */
        if (aspi->default_cs_gpio < ASOC_NR_GPIO)
            gpio_free(aspi->default_cs_gpio);

    }

    /* deselect the default cs  */
    val = asoc_spi_readl(aspi, SPI_CTL);
    val |= SPI_CTL_SSCO;
    asoc_spi_writel(aspi, val, SPI_CTL);
}

static int asoc_spi_baudrate_set(struct asoc_spi_data *aspi, unsigned int speed)
{
    struct clk *hclk;

	u32 spi_source_clk_hz;
	u32 clk_div;

#ifdef CONFIG_MACH_LEOPARD_FPGA
    spi_source_clk_hz = 24000000;
#else
    hclk = clk_get_sys(CLK_NAME_HCLK, NULL);
	if (IS_ERR(hclk)) { 
        printk("%s() cannot get hclk, err %ld\n", 
            __FUNCTION__, PTR_ERR(hclk));
    }

	spi_source_clk_hz = clk_get_rate(hclk);
#endif

#ifdef CONFIG_MACH_LEOPARD_FPGA
    clk_div = 0x1f; /* for FPGA: hclk / 32 */
#else
	/* setup SPI clock register */
	/* SPICLK = HCLK/(CLKDIV*2) */
	clk_div = (spi_source_clk_hz + (2 * speed) - 1) / (speed) / 2;
	if (clk_div == 0)
		clk_div = 1;
#endif

	LOGD(aspi, "required speed = %d\n", speed);
	LOGD(aspi, "asoc_spi: spi clock = %d KHz(hclk = %d,clk_div = %d)\n",
	       spi_source_clk_hz / (clk_div * 2) / 1000, spi_source_clk_hz, clk_div);

	asoc_spi_writel(aspi, SPI_CLKDIV_CLKDIV(clk_div), SPI_CLKDIV);

	return 0;
}

static inline int asoc_spi_config(struct asoc_spi_data *aspi)
{
    unsigned int val, mode;

    mode = aspi->cur_mode;
    val = asoc_spi_readl(aspi, SPI_CTL);

    val &= ~(SPI_CTL_CPOS_MASK | SPI_CTL_LMFS | SPI_CTL_LBT | 
             SPI_CTL_DAWS_MASK | SPI_CTL_CEB);

	if (mode & SPI_CPOL)
		val |= SPI_CTL_CPOS_CPOL;

	if (mode & SPI_CPHA)
		val |= SPI_CTL_CPOS_CPHA;

	if(mode & SPI_LSB_FIRST)
		val |= SPI_CTL_LMFS;

	if(mode & SPI_LOOP)
		val |= SPI_CTL_LBT;

    switch (aspi->cur_bits_per_word)
    {
    case 16:
        val |= SPI_CTL_DAWS_16BIT;
        break;
    case 32:
        val |= SPI_CTL_DAWS_32BIT;
        break;
    default:
        val |= SPI_CTL_DAWS_8BIT;
        break;
    }

    asoc_spi_writel(aspi, val, SPI_CTL);

    asoc_spi_baudrate_set(aspi, aspi->cur_speed);

    return 0;
}

static inline void spi_clear_stat(struct asoc_spi_data *aspi)
{
	asoc_spi_writel(aspi,
          SPI_STAT_TFER	/* clear the rx FIFO */
            | SPI_STAT_RFER	/* clear the tx FIFO */
            | SPI_STAT_BEB	/* clear the Bus error bit */
            | SPI_STAT_TCOM	/* clear the transfer complete bit */
            | SPI_STAT_TIP	/* clear the tx IRQ pending bit */
            | SPI_STAT_PIP,	/* clear the rx IRQ pending bit */
        SPI_STAT);
}

static inline int asoc_spi_wait_tcom(struct asoc_spi_data *aspi)
{
    unsigned int stat;
    int timeout;

    /* transfer timeout: 500ms, 8 cycles per loop */
    timeout = msecs_to_loops(500 / 8);
    
    do {
        stat = asoc_spi_readl(aspi, SPI_STAT);
    } while (!(stat & SPI_STAT_TCOM) && timeout--);

    if (timeout < 0) {
        pr_err("Error: spi transfer wait complete timeout\n");
        dump_spi_registers(aspi);
        return -1;
    }
    
    /* clear transfer complete flag */
    asoc_spi_writel(aspi, stat | SPI_STAT_TCOM, SPI_STAT);

    return 0;
}

static int asoc_spi_write_read_8bit(struct asoc_spi_data *aspi,
                    struct spi_transfer *xfer)
{
    unsigned int ctl;
    unsigned int count = xfer->len;
    const u8 *tx_buf = xfer->tx_buf;
    u8 *rx_buf = xfer->rx_buf;

    ctl = asoc_spi_readl(aspi, SPI_CTL);
    ctl &= ~(SPI_CTL_RWC_MASK | SPI_CTL_TDIC_MASK | SPI_CTL_SDT_MASK | SPI_CTL_DTS |
              SPI_CTL_DAWS_MASK | SPI_CTL_TIEN | SPI_CTL_RIEN | SPI_CTL_TDEN | SPI_CTL_RDEN);
    ctl |= SPI_CTL_RWC(0) | SPI_CTL_DAWS(0);

    if (aspi->cur_speed > 20000000)
        ctl |= SPI_CTL_SDT(1);

    asoc_spi_writel(aspi, ctl, SPI_CTL);

    do {
        if (tx_buf)
            asoc_spi_writel(aspi, *tx_buf++, SPI_TXDAT);
        else
            asoc_spi_writel(aspi, 0, SPI_TXDAT);

        if (asoc_spi_wait_tcom(aspi) < 0) {
            pr_err("SPI: TXS timed out\n");
            return -ETIMEDOUT;
        }

        if (rx_buf)
            *rx_buf++ = asoc_spi_readl(aspi, SPI_RXDAT) & 0xff;

        count -= 1;
    } while(count);

    return 0;
}

static int asoc_spi_write_read_16bit(struct asoc_spi_data *aspi,
                    struct spi_transfer *xfer)
{
    unsigned int ctl;
    unsigned int count = xfer->len;
    const u16 *tx_buf = xfer->tx_buf;
    u16 *rx_buf = xfer->rx_buf;

    ctl = asoc_spi_readl(aspi, SPI_CTL);
    ctl &= ~(SPI_CTL_RWC_MASK | SPI_CTL_TDIC_MASK | SPI_CTL_SDT_MASK | SPI_CTL_DTS |
              SPI_CTL_DAWS_MASK | SPI_CTL_TIEN | SPI_CTL_RIEN | SPI_CTL_TDEN | SPI_CTL_RDEN);
    ctl |= SPI_CTL_RWC(0) | SPI_CTL_DAWS(1);

    if (aspi->cur_speed > 20000000)
        ctl |= SPI_CTL_SDT(1);

    asoc_spi_writel(aspi, ctl, SPI_CTL);

    do {
        if (tx_buf)
            asoc_spi_writel(aspi, *tx_buf++, SPI_TXDAT);
        else
            asoc_spi_writel(aspi, 0, SPI_TXDAT);

        if (asoc_spi_wait_tcom(aspi) < 0) {
            pr_err("SPI: TXS timed out\n");
            return -ETIMEDOUT;
        }

        if (rx_buf)
            *rx_buf++ = asoc_spi_readl(aspi, SPI_RXDAT) & 0xffff;

        count -= 2;
    } while(count);

    return 0;
}

static int asoc_spi_write_read_32bit(struct asoc_spi_data *aspi,
                    struct spi_transfer *xfer)
{
    unsigned int ctl;
    unsigned int count = xfer->len;
    const u32 *tx_buf = xfer->tx_buf;
    u32 *rx_buf = xfer->rx_buf;

    ctl = asoc_spi_readl(aspi, SPI_CTL);
    ctl &= ~(SPI_CTL_RWC_MASK | SPI_CTL_TDIC_MASK | SPI_CTL_SDT_MASK | SPI_CTL_DTS |
              SPI_CTL_TIEN | SPI_CTL_RIEN | SPI_CTL_TDEN | SPI_CTL_RDEN);
    ctl |= SPI_CTL_RWC(0) | SPI_CTL_DAWS(2);

    if (aspi->cur_speed > 20000000)
        ctl |= SPI_CTL_SDT(1);

    asoc_spi_writel(aspi, ctl, SPI_CTL);

    do {
        if (tx_buf)
            asoc_spi_writel(aspi, *tx_buf++, SPI_TXDAT);
        else
            asoc_spi_writel(aspi, 0, SPI_TXDAT);

        if (asoc_spi_wait_tcom(aspi) < 0) {
            pr_err("SPI: TXS timed out\n");
            return -ETIMEDOUT;
        }

        if (rx_buf)
            *rx_buf++ = asoc_spi_readl(aspi, SPI_RXDAT);

        count -= 4;
    } while(count);

    return 0;
}


static int asoc_spi_write_read(struct asoc_spi_data *aspi,
                    struct spi_transfer *xfer)
{
    int word_len, len, count;

    word_len = aspi->cur_bits_per_word;
    len = xfer->len;

    spi_clear_stat(aspi);

    switch (word_len)
    {
    case 8:
        count = asoc_spi_write_read_8bit(aspi, xfer);
        break;
    case 16:
        count = asoc_spi_write_read_16bit(aspi, xfer);
        break;
    case 32:
        count = asoc_spi_write_read_32bit(aspi, xfer);
        break;
    default:
        count = 0;
        break;
    }

    if (count < 0)
        return count;

    return len - count;
}

static int wait_for_xfer(struct asoc_spi_data *aspi,
                    struct spi_transfer *xfer)
{
    unsigned long timeout, ret;

    /* timeout: 2s */
    timeout = msecs_to_jiffies(2000) + 10;
    ret = wait_for_completion_timeout(&aspi->xfer_completion, timeout);
	if (!ret) {
		LOGE(aspi, "Xfer: wait dma finished timeout!\n");
        dump_spi_registers(aspi);
        asoc_dump_mem((void *)IO_ADDRESS(BDMA0_BASE + aspi->dma_chan * 0x30), 0x30, 0, 4);
        reset_dma(aspi->dma_chan);
		return -EIO;
    }

    if (asoc_spi_wait_tcom(aspi) < 0) {
        LOGE(aspi, "wait dma timed out\n");
        return -ETIMEDOUT;
    }

    return 0;
}

static int asoc_spi_write_read_seg_bydma(struct asoc_spi_data *aspi,
                    struct spi_transfer *xfer)
{
    int word_len, len;
    int ret, status = 0;
    unsigned int ctl;

    word_len = aspi->cur_bits_per_word;
    len = xfer->len;

    BUG_ON(xfer->tx_buf && xfer->rx_buf);
    BUG_ON(xfer->len % 4);

    spi_clear_stat(aspi);

    LOGD(aspi, "%d: rx_buf %p, tx_buf %p, len %d, SPI_CTL %x, SPI_STAT %x\n", 
        __LINE__, xfer->rx_buf, xfer->tx_buf, xfer->len, 
        asoc_spi_readl(aspi, SPI_CTL),
        asoc_spi_readl(aspi, SPI_STAT));

    ret = asoc_spi_map_transfer(aspi, xfer);
    if (ret) {
		LOGE(aspi, "Xfer: Unable to map message buffers!\n");
		status = -ENOMEM;
        goto out;
    }
    
    ctl = asoc_spi_readl(aspi, SPI_CTL);
    ctl &= ~(SPI_CTL_RWC_MASK | SPI_CTL_TDIC_MASK | SPI_CTL_SDT_MASK | SPI_CTL_DTS |
              SPI_CTL_TIEN | SPI_CTL_RIEN | SPI_CTL_TDEN | SPI_CTL_RDEN);
    ctl |= SPI_CTL_DAWS(2);
    
    if (aspi->cur_speed > 20000000)
        ctl |= SPI_CTL_SDT(1);

    if (xfer->rx_buf) {
        ctl |= SPI_CTL_RWC(2) | SPI_CTL_DTS | SPI_CTL_RDIC(2) | SPI_CTL_RDEN;
        asoc_spi_writel(aspi, xfer->len / 4, SPI_TCNT);
        asoc_spi_writel(aspi, xfer->len / 4, SPI_RXCR);
    } else {
        ctl |= SPI_CTL_RWC(1) | SPI_CTL_TDIC(2) | SPI_CTL_TDEN;
        asoc_spi_writel(aspi, xfer->len / 4, SPI_TXCR);
    }

    if (word_len != 32)
        ctl |= SPI_CTL_CEB;

    asoc_spi_writel(aspi, ctl, SPI_CTL);

    asoc_spi_init_dma(aspi, xfer);
//    asoc_dump_mem(0xf8220100 + aspi->dma_chan * 0x30, 0x30, 0, 4);

//    LOGD(aspi->master, "%d: rx_buf %p, tx_buf %p, len %d\n   SPI_CTL %x, SPI_STAT %x, SPI_RXCR %x\n", 
//        __LINE__, xfer->rx_buf, xfer->tx_buf, xfer->len, 
//        asoc_spi_readl(aspi, SPI_CTL),
//        asoc_spi_readl(aspi, SPI_STAT),
//        asoc_spi_readl(aspi, SPI_RXCR));

    start_dma(aspi->dma_chan);

    status = wait_for_xfer(aspi, xfer);
    if (status)
        goto out;

out:
	asoc_spi_unmap_transfer(aspi, xfer);

    if (status < 0)
        return status;

    LOGD(aspi->master, "%d: status %d\n", __LINE__, status);

    return xfer->len;
}

static int asoc_spi_write_read_bydma(struct asoc_spi_data *aspi,
                    struct spi_transfer *xfer)
{
    int ret, len;
    struct spi_transfer seg_xfer;

    BUG_ON(xfer->tx_buf && xfer->rx_buf);

    seg_xfer = *xfer;
    len = xfer->len;

    LOGD(aspi, "rx_buf %p, tx_buf %p, len %d\n", 
        xfer->rx_buf, xfer->tx_buf, xfer->len);

    do {
        if (len > 8192)
             seg_xfer.len = 8192;
        else {
            if (len % 4)
                 seg_xfer.len = len - (len % 4);
            else
                 seg_xfer.len = len;
        }
        
        if ( seg_xfer.len < 4)
            ret = asoc_spi_write_read(aspi, &seg_xfer);
        else
            ret = asoc_spi_write_read_seg_bydma(aspi, &seg_xfer);

        if (unlikely(ret < 0)) {
            LOGE(aspi, "failed to write seg_len %d, ret %d\n",  seg_xfer.len, ret);
            goto out;
        }

        if (xfer->rx_buf)
            seg_xfer.rx_buf += seg_xfer.len;
        else
            seg_xfer.tx_buf += seg_xfer.len;

        len -=  seg_xfer.len;
    } while (len > 0);

out:
    if (ret < 0)
        return ret;

    return xfer->len;
}


static int asoc_spi_handle_msg(struct asoc_spi_data *aspi,
                    struct spi_message *msg)
{
    struct spi_device *spi = msg->spi;
    struct spi_transfer *xfer = NULL;
    int ret, status = 0;
	u32 speed;
	u8 bits_per_word;

    LOGD(aspi->master, "%d: \n", __LINE__);

    msg->status = -EINPROGRESS;

    aspi->cur_bits_per_word = spi->bits_per_word;
    aspi->cur_speed = spi->max_speed_hz;
    aspi->cur_mode = spi->mode;
    asoc_spi_config(aspi);

    enable_cs(spi);

    list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		/* Only BPW and Speed may change across transfers */
		bits_per_word = xfer->bits_per_word ? : spi->bits_per_word;
		speed = xfer->speed_hz ? : spi->max_speed_hz;

        if ((bits_per_word != 8) && (bits_per_word != 16) &&
            (bits_per_word != 32)) {
            dev_err(&spi->dev, "invalid bits per word - %d!\n", bits_per_word);
            status = -EINVAL;
            goto out;
        }

        /*transfer length should be alignd according to bits_per_word*/
        if (xfer->len & ((bits_per_word >> 3) - 1)) {
            dev_err(&spi->dev, "bad transfer length - %d!\n", xfer->len);
            status = -EINVAL;
            goto out;
        }

		if (bits_per_word != aspi->cur_bits_per_word || speed != aspi->cur_speed) {
			aspi->cur_bits_per_word = bits_per_word;
			aspi->cur_speed = speed;
			asoc_spi_config(aspi);
		}

#if 0
        if (xfer->tx_buf && aspi->master->bus_num == 0) {
             asoc_dump_mem(xfer->tx_buf, xfer->len, xfer->tx_buf, 1);
        }
#endif

        if (xfer->cs_change)
            enable_cs(spi);

        if (xfer_can_use_dma(aspi, xfer)) {
            ret = asoc_spi_write_read_bydma(aspi, xfer);
            if (unlikely(ret < 0)) {
                status = ret;
                goto out;
            }
        } else {
            ret = asoc_spi_write_read(aspi, xfer);
            if (unlikely(ret < 0)) {
                status = ret;
                goto out;
            }
        }

#if 0
        if (xfer->rx_buf && aspi->master->bus_num == 0) {
            if (xfer->len > 64)  {
                asoc_dump_mem(xfer->rx_buf, 64, xfer->rx_buf, 1);
            } else {
                asoc_dump_mem(xfer->rx_buf, xfer->len, xfer->rx_buf, 1);
            }
        }
#endif

        msg->actual_length += ret;

        if (xfer->delay_usecs)
            udelay(xfer->delay_usecs);

        if (xfer->cs_change)
            disable_cs(spi);
    }

out:
    disable_cs(spi);

    msg->status = status;
    if (status < 0)
        dev_warn(&spi->dev, "spi transfer failed with %d\n", status);

    spi_finalize_current_message(aspi->master);

    LOGD(aspi->master, "%d: \n", __LINE__);

    return status;
}


static int asoc_spi_transfer_one_message(struct spi_master *master,
				      struct spi_message *msg)
{
    struct asoc_spi_data *aspi = spi_master_get_devdata(master);
    int ret;

	ret = asoc_spi_handle_msg(aspi, msg);

	return ret;
}


static int asoc_spi_prepare_transfer_hardware(struct spi_master *master)
{
	return 0;
}

static int asoc_spi_unprepare_transfer_hardware(struct spi_master *master)
{
	return 0;
}

static int asoc_spi_setup(struct spi_device *spi)
{
    struct asoc_spi_controller_data *cdata = spi->controller_data;
    struct asoc_spi_slave_data *sdata;
    int ret = 0;

	/* Only alloc slave data on first setup */
	sdata = spi_get_ctldata(spi);
	if (!sdata) {
		sdata = kzalloc(sizeof(*sdata), GFP_KERNEL);
		if (!sdata) {
            dev_err(&spi->dev, "cannot allocate chip data\n");
			return -ENOMEM;
        }

        if (spi->chip_select) {
            BUG_ON(!cdata || cdata->cs_gpio >= ASOC_NR_GPIO);

            /* request & disable cs */
            ret = gpio_request(cdata->cs_gpio, dev_name(&spi->dev));
            if (ret) {
                dev_err(&spi->dev, "cannot request gpio %d\n", cdata->cs_gpio);
                kfree(sdata);
                return ret;
            }
            gpio_direction_output(cdata->cs_gpio, 1);
            sdata->cs_gpio = cdata->cs_gpio;
        } else {
            sdata->cs_gpio = 0xffffffff;
        }

        spi_set_ctldata(spi, sdata);
    }

    if (!spi->bits_per_word)
        spi->bits_per_word = 8;

	dev_info(&spi->dev,
		"setup: %u Hz bpw %u mode 0x%x for cs %d(gpio %u)\n",
		spi->max_speed_hz, spi->bits_per_word, spi->mode, spi->chip_select, sdata->cs_gpio);

    return 0;
}

static void asoc_spi_cleanup(struct spi_device *spi)
{
    struct asoc_spi_slave_data *sdata = spi_get_ctldata(spi);

	if (!sdata)
		return;

    if (spi->chip_select)
        gpio_free(sdata->cs_gpio);

    kfree(sdata);

    spi_set_ctldata(spi, NULL);
}

/* init hardware by default value */
static int asoc_spi_hwinit(struct asoc_spi_data *aspi)
{
    unsigned int val;

    clk_reset(aspi->clk);

    val = asoc_spi_readl(aspi, SPI_CTL);
    val &= ~(SPI_CTL_RWC_MASK | SPI_CTL_TDIC_MASK | SPI_CTL_SDT_MASK | SPI_CTL_DTS |
              SPI_CTL_TIEN | SPI_CTL_RIEN | SPI_CTL_TDEN | SPI_CTL_RDEN);
    val |= SPI_CTL_EN | SPI_CTL_SSCO;
    asoc_spi_writel(aspi, val, SPI_CTL);

    spi_clear_stat(aspi);

    /* 2MHz by default */
    asoc_spi_baudrate_set(aspi, 2000000);

    return 0;
}

static int __devinit asoc_spi_probe(struct platform_device *pdev)
{
    struct asoc_spi_platform_data *aspi_pdata;
    struct spi_master *master;
    struct asoc_spi_data *aspi;
    struct resource *r;
    int ret = 0;
    char spi_clk[32];

    aspi_pdata = pdev->dev.platform_data;

    master = spi_alloc_master(&pdev->dev, sizeof *aspi);
    if (master == NULL) {
        dev_err(&pdev->dev, "master allocation failed\n");
        return -ENOMEM;
    }

    /* the spi->mode bits understood by this driver: */
    master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;
    master->flags = SPI_MASTER_HALF_DUPLEX; 

    master->bus_num = pdev->id;
    master->setup = asoc_spi_setup;
    master->cleanup = asoc_spi_cleanup;
    master->prepare_transfer_hardware = asoc_spi_prepare_transfer_hardware;
    master->transfer_one_message = asoc_spi_transfer_one_message;
    master->unprepare_transfer_hardware = asoc_spi_unprepare_transfer_hardware;

    dev_set_drvdata(&pdev->dev, master);
    aspi = spi_master_get_devdata(master);
    aspi->master = master;
    aspi->pdev = pdev;

    if (aspi_pdata) {
        master->num_chipselect = aspi_pdata->num_chipselect;
        aspi->default_cs_gpio = aspi_pdata->default_cs_gpio;
        aspi->use_dma = aspi_pdata->use_dma;
        aspi->rt = aspi_pdata->rt;
    } else {
        master->num_chipselect = 4;
        aspi->default_cs_gpio = 0xffffffff;
        aspi->use_dma = 0;
        aspi->rt = 0;
    }

    spin_lock_init(&aspi->lock);

    r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (r == NULL) {
        ret = -ENODEV;
        goto err0;
    }

    if (!request_mem_region(r->start, (r->end - r->start) + 1,
                dev_name(&pdev->dev))) {
        ret = -EBUSY;
        goto err0;
    }

    aspi->phys = r->start;
    aspi->base = (void __iomem *)IO_ADDRESS(r->start);

#ifdef CONFIG_HAVE_CLK
    sprintf(spi_clk, "spi%d_clk", pdev->id);
    aspi->clk = clk_get_sys(spi_clk, NULL);
    if (IS_ERR(aspi->clk)) {
        dev_err(&pdev->dev, "cannot get clock\n");
        ret = PTR_ERR(aspi->clk);
        goto err1;
    }
    clk_enable(aspi->clk);
#endif

    if (aspi->use_dma) {
        aspi->dma_chan = asoc_spi_request_dma(aspi);
        if (aspi->dma_chan < 0) {
            dev_err(&pdev->dev, "cannot request dma\n");
            goto err1;
        }

        dev_info(&pdev->dev, "request dma chan %d\n", aspi->dma_chan);

        master->rt = aspi->rt;
    } else {
        aspi->dma_chan = -1;
    }

    asoc_spi_hwinit(aspi);

    init_completion(&aspi->xfer_completion);

    ret = spi_register_master(master);
    if (ret < 0)
        goto err1;

    return ret;

err1:
    release_mem_region(r->start, (r->end - r->start) + 1);
err0:
    spi_master_put(master);
    return ret;
}

static int __devexit asoc_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct asoc_spi_data *aspi = spi_master_get_devdata(master);
    struct resource *r;
    
    if (aspi->use_dma && aspi->dma_chan >= 0) {
        asoc_spi_free_dma(aspi);
    }

#ifdef CONFIG_HAVE_CLK
    clk_disable(aspi->clk);
    clk_put(aspi->clk);
#endif

    r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    release_mem_region(r->start, (r->end - r->start) + 1);

    spi_unregister_master(master);

    return 0;
}

#ifdef CONFIG_PM
static int asoc_spi_suspend(struct device *dev)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct spi_master *master = platform_get_drvdata(pdev);
    struct asoc_spi_data *aspi = spi_master_get_devdata(master);

    dev_info(dev, "enter suspend\n");

    /* skip suspend for PMU spi bus */
    if (master->bus_num == 1)
        return 0;

    spi_master_suspend(master);

#ifdef CONFIG_HAVE_CLK
    clk_disable(aspi->clk);
#endif

	return 0;
}

static int asoc_spi_resume(struct device *dev)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct spi_master *master = platform_get_drvdata(pdev);
    struct asoc_spi_data *aspi = spi_master_get_devdata(master);

    dev_info(dev, "enter resume\n");

    /* skip suspend for PMU spi bus */
    if (master->bus_num == 1)
        return 0;

#ifdef CONFIG_HAVE_CLK
    if (master->bus_num != 1)
        clk_enable(aspi->clk);
#endif
    asoc_spi_hwinit(aspi);

    spi_master_resume(master); 

	return 0;
}

static const struct dev_pm_ops asoc_spi_pm_ops = {
	.suspend = asoc_spi_suspend,
	.resume = asoc_spi_resume,
};
#endif

static struct platform_driver asoc_spi_driver = {
    .driver = {
        .name = "asoc_spi",
        .owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm     = &asoc_spi_pm_ops,
#endif
    },
    .probe  = asoc_spi_probe,
    .remove = __devexit_p(asoc_spi_remove),
};


static struct resource asoc_spi_resources[ASOC_SPI_CHANNELS][1] = {
    /* SPI0 resources */
    {
        {
            .start		= SPI0_BASE,
            .end		= SPI0_BASE + 0x20,
            .flags		= IORESOURCE_MEM,
        },
    },

    /* SPI1 resources */
    {
        {
            .start		= SPI1_BASE,
            .end		= SPI1_BASE + 0x20,
            .flags		= IORESOURCE_MEM,
        },
    },

    /* SPI2 resources */
    {
        {
            .start		= SPI2_BASE,
            .end		= SPI2_BASE + 0x20,
            .flags		= IORESOURCE_MEM,
        },
    },

    /* SPI3 resources */
    {
        {
            .start		= SPI3_BASE,
            .end		= SPI3_BASE + 0x20,
            .flags		= IORESOURCE_MEM,
        },
    },
};

static struct asoc_spi_platform_data asoc_spi_pdata[ASOC_SPI_CHANNELS] = {
    /* spi0 */
    {
        .num_chipselect = 4,
        .default_cs_gpio = 0xffffffff,
    },
    /* spi1 */
    {
        .num_chipselect = 4,
        .default_cs_gpio = ASOC_GPIO_PORTC(27),
    },
    /* spi2 */
    {
        .num_chipselect = 4,
        .default_cs_gpio = 0xffffffff,
    },
    /* spi3 */
    {
        .num_chipselect = 4,
        .default_cs_gpio = 0xffffffff,
    },
};

static struct platform_device asoc_spi_device[ASOC_SPI_CHANNELS] = {
    /* spi0 */
    {
        .name = "asoc_spi",
        .id = 0,
        .num_resources = ARRAY_SIZE(asoc_spi_resources[0]),
        .resource = asoc_spi_resources[0],
        .dev = {
            .platform_data  = &asoc_spi_pdata[0],
        },
    },
    /* spi1 */
    {
        .name = "asoc_spi",
        .id = 1,
        .num_resources = ARRAY_SIZE(asoc_spi_resources[1]),
        .resource = asoc_spi_resources[1],
        .dev = {
            .platform_data  = &asoc_spi_pdata[1],
        },
    },
    /* spi2 */
    {
        .name = "asoc_spi",
        .id = 2,
        .num_resources = ARRAY_SIZE(asoc_spi_resources[2]),
        .resource = asoc_spi_resources[2],
        .dev = {
            .platform_data  = &asoc_spi_pdata[2],
        },
    },
    /* spi3 */
    {
        .name = "asoc_spi",
        .id = 3,
        .num_resources = ARRAY_SIZE(asoc_spi_resources[3]),
        .resource = asoc_spi_resources[3],
        .dev = {
            .platform_data  = &asoc_spi_pdata[3],
        },
    },
};

static int __init asoc_spi_add_devices(void)
{
    int ret, i;
    char spi_cfg[32];
    int enable, use_dma, rt;
    
    printk("asoc_spi: add controller devices()\n");

    for (i = 0; i < ASOC_SPI_CHANNELS; i++) {
        sprintf(spi_cfg, "spi%d_cfg.enable", i);

        ret = get_config(spi_cfg, (char *)(&enable), sizeof(unsigned int));
        if ((ret < 0) || enable == 0)
            continue;

        sprintf(spi_cfg, "spi%d_cfg.use_dma", i);
        ret = get_config(spi_cfg, (char *)(&use_dma), sizeof(unsigned int));
        if (ret < 0)
            use_dma = 0;

        sprintf(spi_cfg, "spi%d_cfg.rt", i);
        ret = get_config(spi_cfg, (char *)(&rt), sizeof(unsigned int));
        if (ret < 0)
            rt = 0;

        asoc_spi_pdata[i].use_dma = use_dma;
        asoc_spi_pdata[i].rt = rt;

        platform_device_register(&asoc_spi_device[i]);
    }

    return 0;
}

static int __init asoc_spi_init(void)
{
    printk(KERN_INFO "asoc_spi: driver initialized\n");

    platform_driver_register(&asoc_spi_driver);

    return asoc_spi_add_devices();
}

static void __exit asoc_spi_exit(void)
{
    platform_driver_unregister(&asoc_spi_driver);
}

subsys_initcall(asoc_spi_init);
module_exit(asoc_spi_exit);

MODULE_AUTHOR("Actions Semi Inc.");
MODULE_DESCRIPTION("SPI controller driver for Actions SOC");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:asoc_spi");

