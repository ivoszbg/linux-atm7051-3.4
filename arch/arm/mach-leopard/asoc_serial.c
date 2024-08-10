/*
 * arch/arm/mach-leopard/asoc_serial.c
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
#if defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif
#include <linux/module.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/hrtimer.h>
#include <linux/console.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/clock.h>
#include <mach/dma.h>
#include <mach/serial.h>
#include <mach/debug.h>

/* support dma transfer */
#define CONFIG_SERIAL_ASOC_DMA
//#define DEBUG
#define asoc_dump_mem

#define ASOC_MAX_UART                   6
#define ASOC_UART_MAX_BARD_RATE         3000000         /* HOSC / 8 */

/* UART name and device definitions */
#define ASOC_SERIAL_NAME                "ttyS"
#define ASOC_SERIAL_MAJOR               204
#define ASOC_SERIAL_MINOR               0

#define RX_DMA_BUFFER_SIZE              (4096 * 64)

#define UART_CTL                        0x0000
#define UART_RXDAT                      0x0004
#define UART_TXDAT                      0x0008
#define UART_STAT                       0x000C

/* UART0_CTL */
/*bit 31~23 reserved*/
#define UART_CTL_DTCR                   (0x1 << 22)     /* DMA TX counter reset */
#define UART_CTL_DRCR                   (0x1 << 21)     /* DMA RX counter reset */
#define UART_CTL_LBEN                   (0x1 << 20)     /* Loop Back Enable */
#define UART_CTL_TXIE                   (0x1 << 19)     /* TX IRQ Enable */
#define UART_CTL_RXIE                   (0x1 << 18)     /* RX IRQ Enable */
#define UART_CTL_TXDE                   (0x1 << 17)     /* TX DRQ Enable */
#define UART_CTL_RXDE                   (0x1 << 16)     /* RX DRQ Enable */
#define UART_CTL_EN                     (0x1 << 15)     /* UART0 Enable */
#define UART_CTL_TRFS                   (0x1 << 14)     /* UART0 TX/RX FIFO Enable */
#define     UART_CTL_TRFS_RX            (0x0 << 14)         /* select RX FIFO */
#define     UART_CTL_TRFS_TX            (0x1 << 14)         /* select TX FIFO */
/*bit 13 reserved*/
#define UART_CTL_AFE                    (0x1 << 12)     /* Autoflow Enable */
/*bit 11~7 reserved*/
#define UART_CTL_PRS_MASK               (0x7 << 4)
#define UART_CTL_PRS(x)                 (((x) & 0x7) << 4)
#define     UART_CTL_PRS_NONE               UART_CTL_PRS(0)
#define     UART_CTL_PRS_ODD                UART_CTL_PRS(4)
#define     UART_CTL_PRS_MARK               UART_CTL_PRS(5)
#define     UART_CTL_PRS_EVEN               UART_CTL_PRS(6)
#define     UART_CTL_PRS_SPACE              UART_CTL_PRS(7)
/*bit 3 reserved*/
#define UART_CTL_STPS                   (0x1 << 2)
#define     UART_CTL_STPS_1BITS             (0x0 << 2)
#define     UART_CTL_STPS_2BITS             (0x1 << 2)
#define UART_CTL_DWLS_MASK              (0x3 << 0)
#define UART_CTL_DWLS(x)                (((x) & 0x3) << 0)
#define     UART_CTL_DWLS_5BITS             UART_CTL_DWLS(0)
#define     UART_CTL_DWLS_6BITS             UART_CTL_DWLS(1)
#define     UART_CTL_DWLS_7BITS             UART_CTL_DWLS(2)
#define     UART_CTL_DWLS_8BITS             UART_CTL_DWLS(3)

/********************************************************************************/
/* UART0_RXDAT */
/*bit 31~8 reserved*/
#define UART_RXDAT_MASK                 (0xFF << 0)      /* Received Data */

/********************************************************************************/
/* UART0_TXDAT */
/*bit 31~8 reserved*/
#define UART_TXDAT_MASK                 (0xFF << 0)      /* Sending Data*/

/********************************************************************************/
/* UART_STAT */
/*bit 31~17 reserved*/
#define UART_STAT_UTBB                  (0x1 << 16)     /* UART0 TX busy bit */
#define UART_STAT_TRFL_MASK             (0x1F << 11)    /* TX/RX FIFO Level */
#define UART_STAT_TRFL_SET(x)           (((x) & 0x1F) << 11)
#define UART_STAT_TFES                  (0x1 << 10)     /* TX FIFO Empty Status */
#define UART_STAT_RFFS                  (0x1 << 9)      /* RX FIFO full Status */
#define UART_STAT_RTSS                  (0x1 << 8)      /* RTS status */
#define UART_STAT_CTSS                  (0x1 << 7)      /* CTS status */
#define UART_STAT_TFFU                  (0x1 << 6)      /* TX FIFO full Status */
#define UART_STAT_RFEM                  (0x1 << 5)      /* RX FIFO Empty Status */
#define UART_STAT_RXST                  (0x1 << 4)      /* Receive Status */
#define UART_STAT_TFER                  (0x1 << 3)      /* TX FIFO Erro */
#define UART_STAT_RXER                  (0x1 << 2)      /* RX FIFO Erro */
#define UART_STAT_TIP                   (0x1 << 1)      /* TX IRQ Pending Bit */
#define UART_STAT_RIP                   (0x1 << 0)      /* RX IRQ Pending Bit */

#define UART_TO_ASOC(uart_port) ((struct asoc_port *) uart_port)


#define ULOG(lev, port, fmt, args...) \
    do {                        \
        if (port->line != 5)    \
            printk(lev "%s: [uart%d] "fmt, __FUNCTION__, port->line, ## args); \
    } while (0)


#ifdef DEBUG
#define LOGD(port, fmt, args...)    ULOG(KERN_INFO, port, fmt, ## args)
#define LOGI(port, fmt, args...)    ULOG(KERN_DEBUG, port, fmt, ## args)
#else
#define LOGD(port, fmt, args...)    do {} while (0)
#define LOGI(port, fmt, args...)    do {} while (0)
#endif

#define LOGE(port, fmt, args...)    ULOG(KERN_ERR, port, fmt, ## args)


struct asoc_dma_buffer {
	unsigned char	*buf;
	dma_addr_t	dma_addr;
	unsigned int	dma_size;
	unsigned int	ofs;
};

struct asoc_uart_dma {
    /* enable dma transfer */
	int use_dma_rx;
	int use_dma_tx;

    /* dma is busy? */
    int rx_dma_used;
    int tx_dma_used;

    /* lock for struct */
	spinlock_t		tx_lock;
	spinlock_t		rx_lock;

    /* dma channel */
    int rx_dma_channel;
    int tx_dma_channel;

    /* dma buffer */
	struct asoc_dma_buffer rx_buf;
	struct asoc_dma_buffer tx_buf;

    unsigned int prev_rx_dma_pos;

	/* timer to poll activity on rx dma */
	struct hrtimer intr_timer;
	struct uart_port *port;
	//struct timer_list rx_timer;
	unsigned int rx_poll_rate_s;
	unsigned int rx_poll_rate_ns;
	//unsigned int rx_poll_rate;
	unsigned int rx_timeout;
};

struct asoc_port {
    struct uart_port port;
    char name[16];
    struct clk *clk;
    unsigned long phys_base;
    int sysrq_rx_index;
    unsigned long port_activity;
    struct asoc_uart_dma uart_dma;
};


/* Forward declaration of functions */
static void uart_tx_dma_callback(int irq, void *dev_id);
static void uart_rx_dma_callback(int irq, void *dev_id);

static struct asoc_port asoc_ports[ASOC_MAX_UART];

static inline struct uart_port *get_port_from_line(unsigned int line)
{
    return &asoc_ports[line].port;
}

static inline
void asoc_write(struct uart_port *port, unsigned int val, unsigned int off)
{
    __raw_writel(val, port->membase + off);
}

static inline
unsigned int asoc_read(struct uart_port *port, unsigned int off)
{
    return __raw_readl(port->membase + off);
}

#ifdef CONFIG_SERIAL_ASOC_DMA
static bool asoc_use_dma_rx(struct uart_port *port)
{
	struct asoc_port *asoc_port = UART_TO_ASOC(port);

    return asoc_port->uart_dma.use_dma_rx;
}

static bool asoc_use_dma_tx(struct uart_port *port)
{
	struct asoc_port *asoc_port = UART_TO_ASOC(port);

    return asoc_port->uart_dma.use_dma_tx;
}
#else
static bool asoc_use_dma_rx(struct uart_port *port)
{
	return 0;
}

static bool asoc_use_dma_tx(struct uart_port *port)
{
	return 0;
}
#endif

static int check_uart_error(struct uart_port *port)
{
    unsigned int stat = asoc_read(port, UART_STAT);

    if (stat & (UART_STAT_RXER | UART_STAT_TFER | UART_STAT_RXST))
        return 1;

    return 0;
}

static int clear_uart_error(struct uart_port *port)
{
    unsigned int stat = asoc_read(port, UART_STAT);

    /* clear error */
    stat |= (UART_STAT_RXER | UART_STAT_TFER | UART_STAT_RXST);
    /* reserve interrupt pending bits */
    stat &= ~(UART_STAT_RIP | UART_STAT_TIP);
    asoc_write(port, stat, UART_STAT);

    return 0;
}

/*
 * set uart dma mode register
 */
static void __set_uart_dma_mode(struct uart_port *port, unsigned int dmanr, int rx)
{
    unsigned int uart_drq_id[ASOC_MAX_UART] = {
        0, 1, 13, 27, 28, 29};
    unsigned int mode;

    if (port->line >= ASOC_MAX_UART)
        return;

    if (rx) {
        /* fix src address, uart mode enable, dst is ddr */
        mode = uart_drq_id[port->line] | (1 << 6) | (1 << 30) | (18 << 16); 
    } else {
        /* fix dst address, uart mode enable, src is ddr */
        mode = (uart_drq_id[port->line] | (1 << 6)) << 16 | (1 << 30) | (18 << 0); 
    }
    set_dma_mode(dmanr, mode);
}

/*
 * Start transmitting.
 */
static void asoc_start_tx(struct uart_port *port)
{
    struct asoc_port *asoc_port = UART_TO_ASOC(port);
    struct asoc_uart_dma *uart_dma = &asoc_port->uart_dma;
    struct asoc_dma_buffer *tx_buf = &uart_dma->tx_buf;
    struct circ_buf *xmit = &port->state->xmit;
    unsigned int data, count;

    LOGD(port, "%s: UART_CTL %x, UART_STAT %x, count %d\n", __FUNCTION__, 
        asoc_read(port, UART_CTL), asoc_read(port, UART_STAT),
        CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE));

    if (!asoc_use_dma_tx(port)) {
        /* normal transfer */
        data = asoc_read(port, UART_STAT);
        data |= UART_STAT_TIP;
        asoc_write(port, data, UART_STAT);

        data = asoc_read(port, UART_CTL);
        data |= UART_CTL_TXIE;
        asoc_write(port, data, UART_CTL);

        return;
    }

    /* tx dma is busy? */
    if (uart_dma->tx_dma_used)
        return;

    /* update tx busy flag */
	spin_lock(&(uart_dma->tx_lock));
	uart_dma->tx_dma_used = true;
	spin_unlock(&(uart_dma->tx_lock));

    /* check error */
    if (check_uart_error(port)) {
        LOGE(port, "transfer error UART_CTL %x, UART_STAT %x\n",
            asoc_read(port, UART_CTL), 
            asoc_read(port, UART_STAT));
        clear_uart_error(port);
    }

    /* reset uart TX dma counter */
    data = asoc_read(port, UART_CTL);
    data |= UART_CTL_DTCR;
    asoc_write(port, data, UART_CTL);

    /* flush data cache */
    dma_sync_single_for_device(port->dev,
                   tx_buf->dma_addr,
                   tx_buf->dma_size,
                   DMA_TO_DEVICE);

    count = CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE);
    tx_buf->ofs = count;

    reset_dma(uart_dma->tx_dma_channel);
    __set_uart_dma_mode(port, uart_dma->tx_dma_channel, 0);
    set_dma_dst_addr(uart_dma->tx_dma_channel, asoc_port->phys_base + UART_TXDAT);
    set_dma_src_addr(uart_dma->tx_dma_channel, tx_buf->dma_addr + xmit->tail);
    set_dma_count(uart_dma->tx_dma_channel, count);
    enable_dma_tcirq(uart_dma->tx_dma_channel);

    asoc_dump_mem(xmit->buf + xmit->tail, count, xmit->buf + xmit->tail, 1);

    start_dma(uart_dma->tx_dma_channel);

    /* enable TX DRQ for dma */
    data = asoc_read(port, UART_CTL);
    data &= ~UART_CTL_TXIE;
    data |= UART_CTL_TXDE;
    asoc_write(port, data, UART_CTL);    
}

/*
 * Stop transmitting.
 */
static void asoc_stop_tx(struct uart_port *port)
{
    struct asoc_port *asoc_port = UART_TO_ASOC(port);
    struct asoc_uart_dma *uart_dma = &asoc_port->uart_dma;
    unsigned int data;

    LOGD(port, "%s: UART_CTL %x, UART_STAT %x\n", __FUNCTION__, 
        asoc_read(port, UART_CTL), asoc_read(port, UART_STAT));

    if (asoc_use_dma_tx(port)) {
        stop_dma(uart_dma->tx_dma_channel);

        /* update the dma busy flag */
        spin_lock(&(uart_dma->tx_lock));
        uart_dma->tx_dma_used = false;
        spin_unlock(&(uart_dma->tx_lock));
    }

    data = asoc_read(port, UART_CTL);
    /* disable tx IRQ and tx DRQ */
    data &= ~(UART_CTL_TXIE | UART_CTL_TXDE);
    data |= UART_CTL_DTCR;
    asoc_write(port, data, UART_CTL);

    data = asoc_read(port, UART_STAT);
    data |= UART_STAT_TIP;
    /* don't override the rx irq pending flag */
    data &= ~UART_STAT_RIP;
    asoc_write(port, data, UART_STAT);
}

/*
 * Stop receiving - port is in process of being closed.
 */
static void asoc_stop_rx(struct uart_port *port)
{
    struct asoc_port *asoc_port = UART_TO_ASOC(port);
    struct asoc_uart_dma *uart_dma = &asoc_port->uart_dma;
    unsigned int data;

    LOGD(port, "%s\n", __FUNCTION__);

    if (asoc_use_dma_rx(port)) {
        //del_timer(&uart_dma->rx_timer);
		hrtimer_cancel(&uart_dma->intr_timer);
        stop_dma(uart_dma->rx_dma_channel);

        /* update the dma busy flag */
        spin_lock(&(uart_dma->rx_lock));
        uart_dma->rx_dma_used = false;
        spin_unlock(&(uart_dma->rx_lock));
    }

    data = asoc_read(port, UART_CTL);
	/* disable rx IRQ */
    data &= ~UART_CTL_RXIE;
    data |= UART_CTL_DRCR;
    asoc_write(port, data, UART_CTL);

    data = asoc_read(port, UART_STAT);
    data |= UART_STAT_RIP;
    /* don't override the tx irq pending flag */
    data &= ~UART_STAT_TIP;
    asoc_write(port, data, UART_STAT);
}

/*
 * Enable modem status interrupts
 */
static void asoc_enable_ms(struct uart_port *port)
{
}

#ifdef SUPPORT_SYSRQ
static int owl_is_break_button_down(struct uart_port *port, char c)
{
	const char breakbuf[] = {0x02, 0x12, 0x05, 0x01, 0x0b}; /* ctrl + break */
	//const char breakbuf[] = {'a', 'b', 'c', 'd', 'e'}; /* ctrl + break */
	struct asoc_port *aport = UART_TO_ASOC(port);;
	if (c == breakbuf[aport->sysrq_rx_index]) {
		aport->sysrq_rx_index++;
		if (aport->sysrq_rx_index == sizeof(breakbuf)) {
			aport->sysrq_rx_index = 0;
			return 1;
		}
	} else
		aport->sysrq_rx_index = 0;

	return 0;
}
#else
static int owl_is_break_button_down(struct uart_port *port, char c)
{
	return 0;
}
#endif
/*
 * Characters received (called from interrupt handler)
 */
static void asoc_rx_from_ring(struct uart_port *port)
{
    struct tty_struct *tty = port->state->port.tty;
    unsigned int stat, data;

    /* select RX FIFO */
    data = asoc_read(port, UART_CTL);
    data &= ~UART_CTL_TRFS;
    data |= UART_CTL_TRFS_RX;
    asoc_write(port, data, UART_CTL);

    /* and now the main RX loop */
    while (!((stat = asoc_read(port, UART_STAT)) & UART_STAT_RFEM)) {
        unsigned int c;
        char flag = TTY_NORMAL;

        c = asoc_read(port, UART_RXDAT);

        if (stat & UART_STAT_RXER)
            port->icount.overrun++;

        if (stat & UART_STAT_RXST) {
            /* we are not able to distinguish the error type */
            port->icount.brk++;
            port->icount.frame++;

            /* Mask conditions we're ignorning. */
            stat &= port->read_status_mask;
            if (stat & UART_STAT_RXST)
                flag = TTY_PARITY;
        } else {
            port->icount.rx++;
        }
#ifdef SUPPORT_SYSRQ
		if (owl_is_break_button_down(port, c)) {
			if (uart_handle_break(port))
				continue;
		}
#endif
        if (uart_handle_sysrq_char(port, c))
            continue;

        uart_insert_char(port, stat, stat & UART_STAT_RXER, c, flag);
    }

    tty_flip_buffer_push(tty);
}

static int asoc_rx_start_dma(struct uart_port *port)
{
    struct asoc_port *asoc_port = UART_TO_ASOC(port);
    struct asoc_uart_dma *uart_dma = &asoc_port->uart_dma;
    struct asoc_dma_buffer *rx_buf = &uart_dma->rx_buf;
	unsigned int data;
	int ret = 0;

    LOGD(port, "UART_CTL %x, UART_STAT %x\n", 
        asoc_read(port, UART_CTL), 
        asoc_read(port, UART_STAT));

    /* update the dma busy flag */
    spin_lock(&(uart_dma->rx_lock));
    uart_dma->rx_dma_used = true;
    spin_unlock(&(uart_dma->rx_lock));

    uart_dma->prev_rx_dma_pos = 0;

    /* enable RX DRQ for dma */
    data = asoc_read(port, UART_CTL);
    data &= ~UART_CTL_RXIE;
    data |= UART_CTL_RXDE | UART_CTL_DRCR;
    asoc_write(port, data, UART_CTL);

    if (dma_started(uart_dma->rx_dma_channel))
    {
        LOGE(port, "rx dma already stated\n");
    }

    /* set dma parameters */
    set_dma_dst_addr(uart_dma->rx_dma_channel, rx_buf->dma_addr);
    set_dma_count(uart_dma->rx_dma_channel, rx_buf->dma_size);
	start_dma(uart_dma->rx_dma_channel);
    
    /* reset rx poll timer */
	//mod_timer(&uart_dma->rx_timer, jiffies +
	//			uart_dma->rx_poll_rate);
	hrtimer_start(&uart_dma->intr_timer,
					ktime_set(uart_dma->rx_poll_rate_s, uart_dma->rx_poll_rate_ns), HRTIMER_MODE_REL);

	return ret;
}

static int asoc_uart_rx_dma_chars(struct uart_port *port, void *buf, unsigned int size)
{
    struct tty_struct *tty = port->state->port.tty;

    LOGD(port, "UART_CTL %x, UART_STAT %x\n", 
        asoc_read(port, UART_CTL), 
        asoc_read(port, UART_STAT));

    asoc_dump_mem(buf, size, buf, 1);

	port->icount.rx += size;
	tty_insert_flip_string(tty, buf, size);
	tty_flip_buffer_push(tty);

    return 0;
}

static enum hrtimer_restart asoc_rxdma_poll(struct hrtimer *hrtimer)
{
    //struct uart_port *port = (struct uart_port *)handle;
	struct asoc_uart_dma *uart_dma = container_of(hrtimer, struct asoc_uart_dma, intr_timer);
	struct uart_port *port = uart_dma->port;
    struct asoc_port *asoc_port = UART_TO_ASOC(port);
    //struct asoc_uart_dma *uart_dma = &asoc_port->uart_dma;
    struct asoc_dma_buffer *rx_buf = &uart_dma->rx_buf;
	unsigned int curr_dma_pos;
	unsigned int data;
	unsigned long flags;

    LOGD(port, "UART_CTL %x, UART_STAT %x\n", 
        asoc_read(port, UART_CTL), 
        asoc_read(port, UART_STAT));

    /* check error */
    if (check_uart_error(port)) {
        LOGE(port, "transfer error UART_CTL %x, UART_STAT %x\n",
            asoc_read(port, UART_CTL), 
            asoc_read(port, UART_STAT));
        clear_uart_error(port);
    }

/*     spin_lock_irqsave(&port->lock, flags); */
	if (!spin_trylock_irqsave(&port->lock, flags)) {
		hrtimer_start(&uart_dma->intr_timer,
				ktime_set(uart_dma->rx_poll_rate_s, uart_dma->rx_poll_rate_ns), HRTIMER_MODE_REL);
		return HRTIMER_NORESTART;
	}

	curr_dma_pos = get_dma_count(uart_dma->rx_dma_channel) -
            get_dma_remain(uart_dma->rx_dma_channel);

    if (curr_dma_pos > uart_dma->prev_rx_dma_pos) {
        LOGI(port, "new data: curr_dma_pos %d, prev_rx_dma_pos %d\n", 
            curr_dma_pos, uart_dma->prev_rx_dma_pos);

        /* recieve new data */
        asoc_uart_rx_dma_chars(port,
            rx_buf->buf + uart_dma->prev_rx_dma_pos, 
            curr_dma_pos - uart_dma->prev_rx_dma_pos);

        asoc_port->port_activity = jiffies;
    }

    /* receive the data in controller FIFO */
    if (uart_dma->rx_timeout != 0 &&
        time_is_before_jiffies(asoc_port->port_activity + uart_dma->rx_timeout)) {

        LOGI(port, "rx_timeout, stop dma\n");

        /* rx wait timeout */
    	stop_dma(uart_dma->rx_dma_channel);

        /* enable RX interrupt & disable RX DRQ */
        data = asoc_read(port, UART_CTL);
        data &= ~UART_CTL_RXDE;
        data |= UART_CTL_RXIE | UART_CTL_DRCR;
        asoc_write(port, data, UART_CTL);

        spin_unlock_irqrestore(&port->lock, flags);

        //return;
		return HRTIMER_NORESTART;
    }

    uart_dma->prev_rx_dma_pos = curr_dma_pos;
    if (curr_dma_pos == rx_buf->dma_size)
    {
        asoc_rx_start_dma(port);
    }

    //mod_timer(&uart_dma->rx_timer,
    //    jiffies + uart_dma->rx_poll_rate);
	hrtimer_start(&uart_dma->intr_timer,
					ktime_set(uart_dma->rx_poll_rate_s, uart_dma->rx_poll_rate_ns), HRTIMER_MODE_REL);

    spin_unlock_irqrestore(&port->lock, flags);

    //return;
	return HRTIMER_NORESTART;
}

static void uart_rx_dma_callback(int irq, void *dev_id)
{
    struct uart_port *port = dev_id;
    struct asoc_port *asoc_port = UART_TO_ASOC(port);
    struct asoc_uart_dma *uart_dma = &asoc_port->uart_dma;
    struct asoc_dma_buffer *rx_buf = &uart_dma->rx_buf;
	unsigned int curr_dma_pos;
	unsigned long flags;
    /* here actually irq is dma channel */
    unsigned int dma_chan = irq;

    /* clear dma irq pending */
    clear_dma_tcirq_pend(dma_chan);

    spin_lock_irqsave(&port->lock, flags);

	curr_dma_pos = get_dma_count(uart_dma->rx_dma_channel) -
            get_dma_remain(uart_dma->rx_dma_channel);

    asoc_uart_rx_dma_chars(port,
        rx_buf->buf + uart_dma->prev_rx_dma_pos, 
        curr_dma_pos - uart_dma->prev_rx_dma_pos);

    LOGI(port, "rx dma full, restart dma\n");

    asoc_rx_start_dma(port);

    spin_unlock_irqrestore(&port->lock, flags);
}


static void uart_tx_dma_callback(int irq, void *dev_id)
{
    struct uart_port *port = dev_id;
    struct asoc_port *asoc_port = UART_TO_ASOC(port);
    struct asoc_uart_dma *uart_dma = &asoc_port->uart_dma;
    struct asoc_dma_buffer *tx_buf = &uart_dma->tx_buf;
    struct circ_buf *xmit = &port->state->xmit;
    unsigned int dma_chan = irq;    /* here actually irq is dma channel */
    unsigned int data;
	unsigned long flags;

    LOGD(port, "UART_CTL %x, UART_STAT %x, count %d\n", 
        asoc_read(port, UART_CTL), 
        asoc_read(port, UART_STAT),
        CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE));

    /* check error */
    if (check_uart_error(port)) {
        LOGE(port, "transfer error UART_CTL %x, UART_STAT %x\n",
            asoc_read(port, UART_CTL), 
            asoc_read(port, UART_STAT));
        clear_uart_error(port);
    }

    clear_dma_tcirq_pend(dma_chan);

    spin_lock_irqsave(&port->lock, flags);

    /* the previous transfer is over, update ciruit buffer pointer */
	xmit->tail += tx_buf->ofs;
	xmit->tail &= (UART_XMIT_SIZE - 1);
	port->icount.tx += tx_buf->ofs;

	if (uart_circ_empty(xmit)) {
		asoc_stop_tx(port);
	} else {
        /* has new data to send */
        stop_dma(uart_dma->tx_dma_channel);

        /* reset uart TX dma counter */
        data = asoc_read(port, UART_CTL);
        data |= UART_CTL_DTCR;
        asoc_write(port, data, UART_CTL);

        dma_sync_single_for_device(port->dev,
                       tx_buf->dma_addr,
                       tx_buf->dma_size,
                       DMA_TO_DEVICE);

		tx_buf->ofs = CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE);

        set_dma_src_addr(uart_dma->tx_dma_channel, tx_buf->dma_addr + xmit->tail);
		set_dma_count(uart_dma->tx_dma_channel, tx_buf->ofs);

        asoc_dump_mem(xmit->buf + xmit->tail, tx_buf->ofs, xmit->buf + xmit->tail, 1);

        start_dma(uart_dma->tx_dma_channel);
	}

    asoc_port->port_activity = jiffies; 

    spin_unlock_irqrestore(&port->lock, flags);
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);
}

/*
 * transmit interrupt handler
 */
static void asoc_tx_chars(struct uart_port *port)
{
    struct circ_buf *xmit = &port->state->xmit;
    unsigned int data;

    if (port->x_char) {
        /* wait TX FIFO not full */
        while(asoc_read(port, UART_STAT) & UART_STAT_TFFU);
        asoc_write(port, port->x_char, UART_TXDAT);
        port->icount.tx++;
        port->x_char = 0;
    }

    /* select TX FIFO */
    data = asoc_read(port, UART_CTL);
    data &= ~UART_CTL_TRFS;
    data |= UART_CTL_TRFS_TX;
    asoc_write(port, data, UART_CTL);

    while (!(asoc_read(port, UART_STAT) & UART_STAT_TFFU)) {
        if (uart_circ_empty(xmit))
            break;

        asoc_write(port, xmit->buf[xmit->tail], UART_TXDAT);

        /* wait FIFO empty? */
        while(asoc_read(port, UART_STAT) & UART_STAT_TRFL_MASK);

        xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
        port->icount.tx++;
    }

    if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
        uart_write_wakeup(port);

    if (uart_circ_empty(xmit)) {
        asoc_stop_tx(port);
    }
}

/*
 * Interrupt handler
 */
static irqreturn_t asoc_irq(int irq, void *dev_id)
{
    struct uart_port *port = dev_id;
    struct asoc_port *asoc_port = UART_TO_ASOC(port);
    unsigned int stat;

    spin_lock(&port->lock);

    /* check error */
    if (check_uart_error(port)) {
        LOGE(port, "transfer error UART_CTL %x, UART_STAT %x\n",
            asoc_read(port, UART_CTL), 
            asoc_read(port, UART_STAT));
    }

    stat = asoc_read(port, UART_STAT);

    /* clear irq/error pending */
    asoc_write(port, stat, UART_STAT);

    if (stat & UART_STAT_RIP) {
        if (asoc_use_dma_rx(port))
            asoc_rx_start_dma(port);
        else
            asoc_rx_from_ring(port);
    }

    if (stat & UART_STAT_TIP) {
        if (!asoc_use_dma_tx(port))
            asoc_tx_chars(port);
    }

    asoc_port->port_activity = jiffies;

    spin_unlock(&port->lock);

    return IRQ_HANDLED;
}

/*
 * Return TIOCSER_TEMT when transmitter FIFO and Shift register is empty.
 */
static unsigned int asoc_tx_empty(struct uart_port *port)
{
    unsigned int data, ret;
    unsigned long flags;

    spin_lock_irqsave(&port->lock, flags);

    /* select TX FIFO */
    data = asoc_read(port, UART_CTL);
    data &= ~UART_CTL_TRFS;
    data |= UART_CTL_TRFS_TX;
    asoc_write(port, data, UART_CTL);

    /* check FIFO level */
    data = asoc_read(port, UART_STAT);
    ret = (data & UART_STAT_TRFL_MASK) ? 0 : TIOCSER_TEMT;

    spin_unlock_irqrestore(&port->lock, flags);

    return ret;
}

static unsigned int asoc_get_mctrl(struct uart_port *port)
{
    return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;
}

static void asoc_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static void asoc_break_ctl(struct uart_port *port, int break_ctl)
{
}

static int asoc_set_baud_rate(struct uart_port *port, unsigned int baud)
{
    struct asoc_port *asoc_port = UART_TO_ASOC(port);

#if 0
    unsigned int div;

    /* clock source: 24MHz HOSC */
    div = ((24 * 1000000) / (8 * baud)) - 1;
    __raw_writel(div, asoc_port->cmu_reg);
#endif

    clk_set_rate(asoc_port->clk, 8 * baud);

    return baud;
}

/*
 * Perform initialization and enable port for reception
 */
static int asoc_startup(struct uart_port *port)
{
    struct asoc_port *asoc_port = UART_TO_ASOC(port);
    unsigned int data;
    int ret;
    unsigned long flags;

    snprintf(asoc_port->name, sizeof(asoc_port->name),
         "asoc_serial%d", port->line);

    LOGD(port, "asoc_startup: asoc_serial%d", port->line);

	if (port->flags & ASYNC_INITIALIZED) {
        LOGE(port, "already open, skip startup again\n");
		return 0;
    }

    ret = request_irq(port->irq, asoc_irq, IRQF_TRIGGER_HIGH,
              asoc_port->name, port);
    if (unlikely(ret))
        return ret;

    if (asoc_port->clk) {
        clk_enable(asoc_port->clk);
        //clk_reset(asoc_port->clk);
    }

	/*
	 * Initialize DMA (if necessary)
	 */

	if (asoc_use_dma_tx(port)) {
		struct asoc_uart_dma *uart_dma = &asoc_port->uart_dma;
		struct asoc_dma_buffer *tx_buf = &uart_dma->tx_buf;
		struct circ_buf *xmit = &port->state->xmit;

        uart_dma->tx_dma_channel = request_asoc_dma(DMA_CHAN_TYPE_BUS,
            dev_name(asoc_port->port.dev), uart_tx_dma_callback, 0, port);
        if (uart_dma->tx_dma_channel < 0) {
            LOGE(port, "cannot request dma channel, ret %d\n",
                uart_dma->tx_dma_channel);
            return -1;
        }

	    LOGD(port, "use tx dma, dma_channel %d\n", uart_dma->tx_dma_channel);

		tx_buf->buf = xmit->buf;
		tx_buf->dma_addr = dma_map_single(port->dev,
					       tx_buf->buf,
					       UART_XMIT_SIZE,
					       DMA_TO_DEVICE);
		tx_buf->dma_size = UART_XMIT_SIZE;
		tx_buf->ofs = 0;
	}

	if (asoc_use_dma_rx(port)) {
		struct asoc_uart_dma *uart_dma = &asoc_port->uart_dma;
		struct asoc_dma_buffer *rx_buf = &uart_dma->rx_buf;

	    LOGD(port, "use rx dma\n");

        uart_dma->rx_dma_channel = request_asoc_dma(DMA_CHAN_TYPE_BUS,
            dev_name(asoc_port->port.dev), uart_rx_dma_callback, 0, port);
        if (uart_dma->rx_dma_channel < 0) {
            LOGE(port, "cannot request dma channel, ret %d\n",
                uart_dma->rx_dma_channel);
            return -EBUSY;
        }

	    LOGD(port, "use rx dma, dma_channel %d\n", uart_dma->rx_dma_channel);

		/* Currently the buffer size is 4KB. Can increase it */
		rx_buf->buf = dma_alloc_coherent(NULL,
			RX_DMA_BUFFER_SIZE,
			(dma_addr_t *)&(rx_buf->dma_addr), 0);
        rx_buf->dma_size = RX_DMA_BUFFER_SIZE;
        rx_buf->ofs = 0;

        set_dma_src_addr(uart_dma->rx_dma_channel, asoc_port->phys_base + UART_RXDAT);
        set_dma_dst_addr(uart_dma->rx_dma_channel, rx_buf->dma_addr);
        set_dma_count(uart_dma->rx_dma_channel, RX_DMA_BUFFER_SIZE);
        __set_uart_dma_mode(port, uart_dma->rx_dma_channel, 1);

		//init_timer(&(uart_dma->rx_timer));
		//uart_dma->rx_timer.function = asoc_rxdma_poll;
		//uart_dma->rx_timer.data = (unsigned long)port;
		uart_dma->port = port;
		hrtimer_init(&uart_dma->intr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		uart_dma->intr_timer.function = asoc_rxdma_poll;
	}

    spin_lock_irqsave(&port->lock, flags);

    /* clear IRQ pending */
    data = asoc_read(port, UART_STAT);
    data |= UART_STAT_RIP | UART_STAT_TIP;
    asoc_write(port, data, UART_STAT);

    /* enable module/IRQs */
    data = asoc_read(port, UART_CTL);
    data |= UART_CTL_RXIE | UART_CTL_EN;


#if 0
    /* enable loopback for test */
    if (port->line != 5)
        data |= UART_CTL_LBEN;
#endif

    asoc_write(port, data, UART_CTL);

	port->flags |= ASYNC_INITIALIZED;

    spin_unlock_irqrestore(&port->lock, flags);

    return 0;
}

/*
 * Disable the port
 */
static void asoc_shutdown(struct uart_port *port)
{
    struct asoc_port *asoc_port = UART_TO_ASOC(port);
    unsigned int data;
    unsigned long flags;

	if (!(port->flags & ASYNC_INITIALIZED)) {
        LOGE(port, "already shutdown, skip shutdown again\n");
		return;
    }

    spin_lock_irqsave(&port->lock, flags);

    /*
     * Ensure everything is stopped.
     */
    asoc_stop_rx(port);
    asoc_stop_tx(port);
	
    /* disable module/IRQs */
    data = asoc_read(port, UART_CTL);
    data &= ~(UART_CTL_RXIE | UART_CTL_TXIE | UART_CTL_EN);
    asoc_write(port, data, UART_CTL);

    spin_unlock_irqrestore(&port->lock, flags);

    /* fixme: don't disable clock because the console maybe still use this port */
//    if (asoc_port->clk)
//        clk_disable(asoc_port->clk);

    free_irq(port->irq, port);

	/*
	 * Shut-down the DMA.
	 */
	if (asoc_use_dma_rx(port)) {
		struct asoc_uart_dma *uart_dma = &asoc_port->uart_dma;
		struct asoc_dma_buffer *rx_buf = &uart_dma->rx_buf;

		dma_free_coherent(port->dev,
			rx_buf->dma_size, rx_buf->buf,
			rx_buf->dma_addr);
        rx_buf->buf = NULL;

        free_asoc_dma(uart_dma->rx_dma_channel);
        uart_dma->rx_dma_channel = -1;
	}

	if (asoc_use_dma_tx(port)) {
		struct asoc_uart_dma *uart_dma = &asoc_port->uart_dma;
		struct asoc_dma_buffer *tx_buf = &uart_dma->tx_buf;

		dma_unmap_single(port->dev,
				 tx_buf->dma_addr,
				 tx_buf->dma_size,
				 DMA_TO_DEVICE);

        free_asoc_dma(uart_dma->tx_dma_channel);
        uart_dma->tx_dma_channel = -1;
	}

    port->flags &= ~ASYNC_INITIALIZED;
}

/*
 * Change the port parameters
 */
static void asoc_set_termios(struct uart_port *port, struct ktermios *termios,
                  struct ktermios *old)
{
    unsigned long flags;
    unsigned int ctl, baud;

    spin_lock_irqsave(&port->lock, flags);

    baud = uart_get_baud_rate(port, termios, old, 0, ASOC_UART_MAX_BARD_RATE);
    baud = asoc_set_baud_rate(port, baud);
    /*
     * We don't support modem control lines.
     */
    termios->c_cflag &= ~(HUPCL | CMSPAR);
    termios->c_cflag |= CLOCAL;

    /*
     * We don't support BREAK character recognition.
     */
    termios->c_iflag &= ~(IGNBRK | BRKINT);

    ctl = asoc_read(port, UART_CTL);
    ctl &= ~(UART_CTL_DWLS_MASK | UART_CTL_STPS
            | UART_CTL_PRS_MASK | UART_CTL_AFE);

    /* byte size */
    ctl &= ~UART_CTL_DWLS_MASK;
    switch (termios->c_cflag & CSIZE) {
    case CS5:
        ctl |= UART_CTL_DWLS(0);
        break;
    case CS6:
        ctl |= UART_CTL_DWLS(1);
        break;
    case CS7:
        ctl |= UART_CTL_DWLS(2);
        break;
    case CS8:
    default:
        ctl |= UART_CTL_DWLS(3);
        break;
    }

    /* stop bits */
    if (termios->c_cflag & CSTOPB)
        ctl |= UART_CTL_STPS_2BITS;
    else
        ctl |= UART_CTL_STPS_1BITS;

    /* parity */
    if (termios->c_cflag & PARENB) {
        /* Mark or Space parity */
        if (termios->c_cflag & CMSPAR) {
            if (termios->c_cflag & PARODD)
                ctl |= UART_CTL_PRS_MARK;
            else
                ctl |= UART_CTL_PRS_SPACE;
        } else if (termios->c_cflag & PARODD)
            ctl |= UART_CTL_PRS_ODD;
        else
            ctl |= UART_CTL_PRS_EVEN;
    } else
        ctl |= UART_CTL_PRS_NONE;

    /* hardware handshake (RTS/CTS) */
    if (termios->c_cflag & CRTSCTS)
        ctl |= UART_CTL_AFE;

    asoc_write(port, ctl, UART_CTL);

    /* Configure status bits to ignore based on termio flags. */

    /*
     * Normally we need to mask the bits we do care about
     * as there is no hardware support for (termios->c_iflag & INPACK/BRKINT/PARMRK)
     * and it seems the interrupt happened only for tx/rx
     * we do nothing about the port.read_status_mask
     */
    port->read_status_mask |= UART_STAT_RXER;
    if (termios->c_iflag & INPCK)
        port->read_status_mask |= UART_STAT_RXST;

    /* update the per-port timeout */
    uart_update_timeout(port, termios->c_cflag, baud);

    spin_unlock_irqrestore(&port->lock, flags);
}

/*
 * Return string describing the specified port
 */
static const char *asoc_type(struct uart_port *port)
{
    return "ASOC_SERIAL";
}

/*
 * Release the memory region(s) being used by 'port'.
 */
static void asoc_release_port(struct uart_port *port)
{
    struct platform_device *pdev = to_platform_device(port->dev);
    struct resource *resource;
    resource_size_t size;

    resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (unlikely(!resource))
        return;
    size = resource->end - resource->start + 1;

    release_mem_region(port->mapbase, size);

    if (port->flags & UPF_IOREMAP) {
        iounmap(port->membase);
        port->membase = NULL;
    }
}

/*
 * Request the memory region(s) being used by 'port'.
 */
static int asoc_request_port(struct uart_port *port)
{
    struct platform_device *pdev = to_platform_device(port->dev);
    struct resource *resource;
    resource_size_t size;

    resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (unlikely(!resource))
        return -ENXIO;
    size = resource->end - resource->start + 1;

    if (!request_mem_region(port->mapbase, size, "asoc_serial"))
        return -EBUSY;

    if (port->flags & UPF_IOREMAP) {
        port->membase = ioremap(port->mapbase, size);
        if (port->membase == NULL) {
            release_mem_region(port->mapbase, size);
            return -ENOMEM;
        }
    }

    return 0;
}


/*
 * Configure/autoconfigure the port.
 */
static void asoc_config_port(struct uart_port *port, int flags)
{
    if (flags & UART_CONFIG_TYPE) {
        port->type = PORT_ASOC;
        asoc_request_port(port);
    }
}

static int asoc_verify_port(struct uart_port *port, struct serial_struct *ser)
{
    if (unlikely(ser->type != PORT_UNKNOWN && ser->type != PORT_ASOC))
        return -EINVAL;
    if (unlikely(port->irq != ser->irq))
        return -EINVAL;
    if ((void *)port->membase != ser->iomem_base)
        return -EINVAL;
    return 0;
}

#ifdef CONFIG_CONSOLE_POLL
static int asoc_poll_get_char(struct uart_port *port)
{
    unsigned int old_ctl, data;
    unsigned long flags;
    unsigned int ch = NO_POLL_CHAR;

    spin_lock_irqsave(&port->lock, flags);

    /* backup old control register */
    old_ctl = asoc_read(port, UART_CTL);

    /* select RX FIFO */
    data = old_ctl & (~(UART_CTL_TRFS));
    data = data | UART_CTL_TRFS_RX;

    /* disable IRQ */
    //data &= ~(UART_CTL_TXIE | UART_CTL_RXIE);
    asoc_write(port, data, UART_CTL);

    /* wait RX FIFO not emtpy */
    do {
        cpu_relax();
        /* Get the interrupts */
        data = asoc_read(port, UART_STAT);
    } while ((data & UART_STAT_RIP) == 0);

    while (!(data & UART_STAT_RFEM)) {
        ch = asoc_read(port, UART_RXDAT);
        data = asoc_read(port, UART_STAT);
    }

    /* clear IRQ pending */
    data = asoc_read(port, UART_STAT);
    //data |= UART_STAT_TIP | UART_STAT_RIP;
    data |= UART_STAT_RIP;
    asoc_write(port, data, UART_STAT);

    /* restore old ctl */
    asoc_write(port, old_ctl, UART_CTL);

    spin_unlock_irqrestore(&port->lock, flags);

    return ch;
}

static void asoc_poll_put_char(struct uart_port *port, unsigned char ch)
{
    unsigned int old_ctl, data;
    unsigned long flags;

    spin_lock_irqsave(&port->lock, flags);

    /* backup old control register */
    old_ctl = asoc_read(port, UART_CTL);

    /* select TX FIFO */
    data = old_ctl & (~(UART_CTL_TRFS));
    data = data | UART_CTL_TRFS_TX;

    /* disable IRQ */
    data &= ~(UART_CTL_TXIE | UART_CTL_RXIE);
    asoc_write(port, data, UART_CTL);

    /* wait TX FIFO not full */
    while(asoc_read(port, UART_STAT) & UART_STAT_TFFU)
        cpu_relax();

    asoc_write(port, ch, UART_TXDAT);

    /* wait until all content have been sent out
     * TODO:
     */
    while(asoc_read(port, UART_STAT) & UART_STAT_TRFL_MASK)
        cpu_relax();

    /* clear IRQ pending */
    data = asoc_read(port, UART_STAT);
    //data |= UART_STAT_TIP | UART_STAT_RIP;
    data |= UART_STAT_TIP;
    asoc_write(port, data, UART_STAT);
    data = asoc_read(port, UART_STAT);

    /* restore old ctl */
    asoc_write(port, old_ctl, UART_CTL);

    spin_unlock_irqrestore(&port->lock, flags);
    return;
}
#endif

static struct uart_ops asoc_uart_pops = {
    .tx_empty = asoc_tx_empty,
    .set_mctrl = asoc_set_mctrl,
    .get_mctrl = asoc_get_mctrl,
    .stop_tx = asoc_stop_tx,
    .start_tx = asoc_start_tx,
    .stop_rx = asoc_stop_rx,
    .enable_ms = asoc_enable_ms,
    .break_ctl = asoc_break_ctl,
    .startup = asoc_startup,
    .shutdown = asoc_shutdown,
    .set_termios = asoc_set_termios,
    .type = asoc_type,
    .release_port = asoc_release_port,
    .request_port = asoc_request_port,
    .config_port = asoc_config_port,
    .verify_port = asoc_verify_port,
#ifdef CONFIG_CONSOLE_POLL
    .poll_get_char  = asoc_poll_get_char,
    .poll_put_char  = asoc_poll_put_char,
#endif
    //.pm = asoc_power,
};

/*
 * Configure the port from the platform device resource info.
 */
static int __devinit asoc_init_port(struct asoc_port *asoc_port,
                      struct platform_device *pdev)
{
    struct uart_port *port = &asoc_port->port;
    struct asoc_uart_platform_data *pdata = pdev->dev.platform_data;
    struct resource *resource;
    int irq;

    port->iotype        = UPIO_MEM;
    port->flags     = UPF_BOOT_AUTOCONF;
    port->ops       = &asoc_uart_pops;
    port->fifosize      = 1;
    port->line      = pdev->id;
    port->dev       = &pdev->dev;

    resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (unlikely(!resource))
        return -ENXIO;
    port->membase = IO_ADDRESS(resource->start);
    port->mapbase = (resource_size_t)port->membase;
    asoc_port->phys_base = resource->start;

    if (pdata != NULL) {
        struct asoc_uart_dma *uart_dma = &asoc_port->uart_dma;

        uart_dma->use_dma_tx = pdata->use_dma_tx;
        uart_dma->use_dma_rx = pdata->use_dma_rx;

        if (pdata->use_dma_rx) {
            //uart_dma->rx_poll_rate = msecs_to_jiffies(pdata->rx_poll_rate);
            uart_dma->rx_poll_rate_s = pdata->rx_poll_rate/1000;
			uart_dma->rx_poll_rate_ns = (pdata->rx_poll_rate%1000)*1000000;
            uart_dma->rx_timeout = msecs_to_jiffies(pdata->rx_timeout);
            
            /* set default time */
            if (pdata->rx_poll_rate == 0) {
                //uart_dma->rx_poll_rate = msecs_to_jiffies(10);
				uart_dma->rx_poll_rate_s = 0;
				uart_dma->rx_poll_rate_ns = 10*1000000;
                uart_dma->rx_timeout = 0;
            }
        }
    }

    LOGI(port, "pdata->rx_timeout %d ms, uart_dma.rx_timeout %d jiffies", 
        pdata->rx_timeout,
        asoc_port->uart_dma.rx_timeout);

    spin_lock_init(&asoc_port->uart_dma.tx_lock);
    spin_lock_init(&asoc_port->uart_dma.rx_lock);

    irq = platform_get_irq(pdev, 0);
    if (unlikely(irq < 0))
        return -ENXIO;
    port->irq = irq;

    switch(pdev->id) {
    case 0:
        asoc_port->clk = clk_get_sys(CLK_NAME_UART0_CLK, NULL);
        break;
    case 1:
        asoc_port->clk = clk_get_sys(CLK_NAME_UART1_CLK, NULL);
        break;
    case 2:
        asoc_port->clk = clk_get_sys(CLK_NAME_UART2_CLK, NULL);
        break;
    case 3:
        asoc_port->clk = clk_get_sys(CLK_NAME_UART3_CLK, NULL);
        break;
    case 4:
        asoc_port->clk = clk_get_sys(CLK_NAME_UART4_CLK, NULL);
        break;
    case 5:
        asoc_port->clk = clk_get_sys(CLK_NAME_UART5_CLK, NULL);
        break;
    default:
        break;
    }

    if (IS_ERR(asoc_port->clk)) {
        LOGE(port, "cannot get clk\n");
        asoc_port->clk = NULL;
        return -1;
    }

    return 0;
}

#ifdef CONFIG_SERIAL_ASOC_CONSOLE
static void asoc_console_putchar(struct uart_port *port, int c)
{
    while(asoc_read(port, UART_STAT) & UART_STAT_TFFU)
        cpu_relax();
    asoc_write(port, c, UART_TXDAT);
}

static void asoc_console_write(struct console *co, const char *s,
                  unsigned int count)
{
    struct uart_port *port;
    unsigned int old_ctl, data;
    int locked = 1;

    BUG_ON(co->index < 0 || co->index >= ASOC_MAX_UART);

    port = get_port_from_line(co->index);

    if (port->sysrq)
        locked = 0;
    else
        spin_lock(&port->lock);


    /* backup old control register */
    old_ctl = asoc_read(port, UART_CTL);

    /* disable IRQ */
    data = old_ctl | UART_CTL_TRFS_TX | UART_CTL_EN;
    data &= ~(UART_CTL_TXIE | UART_CTL_RXIE);
    asoc_write(port, data, UART_CTL);

    uart_console_write(port, s, count, asoc_console_putchar);

    /* wait until all content have been sent out */
    while(asoc_read(port, UART_STAT) & UART_STAT_TRFL_MASK);

    /* clear IRQ pending */
    data = asoc_read(port, UART_STAT);
    data |= UART_STAT_TIP | UART_STAT_RIP;
    asoc_write(port, data, UART_STAT);

    /* restore old ctl */
    asoc_write(port, old_ctl, UART_CTL);
    if (locked)
        spin_unlock(&port->lock);

}

static int __init asoc_console_setup(struct console *co, char *options)
{
    struct uart_port *port;
    int baud = 115200;
    int bits = 8;
    int parity = 'n';
    int flow = 'n';

    if (unlikely(co->index >= ASOC_MAX_UART || co->index < 0))
        return -ENXIO;

    port = get_port_from_line(co->index);

    if (unlikely(!port->membase))
        return -ENXIO;

    port->cons = co;

    if (options)
        uart_parse_options(options, &baud, &parity, &bits, &flow);

    printk(KERN_INFO "asoc_serial: console setup on port #%d\n", port->line);

    return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct uart_driver asoc_uart_driver;

static struct console asoc_console = {
    .name = ASOC_SERIAL_NAME,
    .write = asoc_console_write,
    .device = uart_console_device,
    .setup = asoc_console_setup,
    .flags = CON_PRINTBUFFER,
    .index = -1,
    .data = &asoc_uart_driver,
};

#define ASOC_CONSOLE    (&asoc_console)

#else
#define ASOC_CONSOLE    NULL
#endif

static struct uart_driver asoc_uart_driver = {
    .owner = THIS_MODULE,
    .driver_name = "asoc_serial",
    .dev_name = ASOC_SERIAL_NAME,
    .nr = ASOC_MAX_UART,
    .cons = ASOC_CONSOLE,
    .major = ASOC_SERIAL_MAJOR,
    .minor = ASOC_SERIAL_MINOR,
};

#ifdef CONFIG_SERIAL_CORE_CONSOLE
#define uart_console(port)	((port)->cons && (port)->cons->index == (port)->line)
#else
#define uart_console(port)	(0)
#endif

static int asoc_serial_suspend(struct platform_device *pdev, pm_message_t state)
{
    struct uart_port *port = platform_get_drvdata(pdev);
    struct asoc_port *asoc_port = UART_TO_ASOC(port);

    if (port && !uart_console(port))
    {
        LOGI(port, "enter suspend\n");
        uart_suspend_port(&asoc_uart_driver, port);
    }

    return 0;
}

static int asoc_serial_resume(struct platform_device *pdev)
{
    struct uart_port *port = platform_get_drvdata(pdev);
    struct asoc_port *asoc_port = UART_TO_ASOC(port);

    if (port && !uart_console(port))
    {
        LOGI(port, "enter resume\n");
        uart_resume_port(&asoc_uart_driver, port);
    }

    return 0;
}

static int __init asoc_serial_probe(struct platform_device *pdev)
{
    struct asoc_port *asoc_port;
    struct uart_port *port;
    int ret;

    if (unlikely(pdev->id < 0 || pdev->id >= ASOC_MAX_UART))
        return -ENXIO;

    printk(KERN_INFO "asoc_serial: detected port #%d\n", pdev->id);

    port = get_port_from_line(pdev->id);
    asoc_port = UART_TO_ASOC(port);

    ret = asoc_init_port(asoc_port, pdev);
    if (ret)
        return ret;

    platform_set_drvdata(pdev, port);

    return uart_add_one_port(&asoc_uart_driver, port);
}

static int __devexit asoc_serial_remove(struct platform_device *pdev)
{
    struct uart_port *port = platform_get_drvdata(pdev);
    int ret = 0;

    device_init_wakeup(&pdev->dev, 0);
    platform_set_drvdata(pdev, NULL);

    ret = uart_remove_one_port(&asoc_uart_driver, port);

    /* "port" is allocated statically, so we shouldn't free it */

    return ret;
}

static struct platform_driver asoc_platform_driver = {
    .remove = asoc_serial_remove,
    .driver = {
        .name = "asoc-serial",
        .owner = THIS_MODULE,
    },
    .suspend	= asoc_serial_suspend,
    .resume		= asoc_serial_resume,
};

static int __init asoc_serial_init(void)
{
    int ret;

    ret = uart_register_driver(&asoc_uart_driver);
    if (unlikely(ret))
        return ret;

    ret = platform_driver_probe(&asoc_platform_driver, asoc_serial_probe);
    if (unlikely(ret))
        uart_unregister_driver(&asoc_uart_driver);

    printk(KERN_INFO "asoc_serial: driver initialized\n");

    return ret;
}

static void __exit asoc_serial_exit(void)
{
    platform_driver_unregister(&asoc_platform_driver);
    uart_unregister_driver(&asoc_uart_driver);
}

module_init(asoc_serial_init);
module_exit(asoc_serial_exit);

MODULE_AUTHOR("Actions Semi Inc.");
MODULE_DESCRIPTION("serial driver for Actions SOC");
MODULE_LICENSE("GPL");
