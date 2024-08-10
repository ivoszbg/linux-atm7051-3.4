/*
 * atc260x_irq.c  --  Interrupt controller support for ATC260X PMIC
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
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/mfd/core.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <mach/atc260x/atc260x.h>
#include <mach/atc260x/atc260x_pdata.h>
#include <mach/atc260x/atc260x_dev.h>

extern void set_irq_flags(unsigned int irq, unsigned int iflags);

//#undef dev_dbg(fmt...)
//#define dev_dbg(fmt...) do{}while(0)

#define ATC260X_INT_BIT(atc260x_internal_irq)       (1 << (atc260x_internal_irq))

/*
 * Since generic IRQs don't currently support interrupt controllers on
 * interrupt driven buses we don't use genirq but instead provide an
 * interface that looks very much like the standard ones.  This leads
 * to some bodges, including storing interrupt handler information in
 * the static irq_data table we use to look up the data for individual
 * interrupts, but hopefully won't last too long.
 */
static inline int irq_to_atc260x_irq(struct atc260x_dev *atc260x,
                            int irq)
{
    return (irq - atc260x->irq_base);
}

static void atc260x_irq_lock(struct irq_data *data)
{
    struct atc260x_dev *atc260x = irq_data_get_irq_chip_data(data);

    mutex_lock(&atc260x->irq_lock);
}

static void atc260x_irq_sync_unlock(struct irq_data *data)
{
    struct atc260x_dev *atc260x = irq_data_get_irq_chip_data(data);

    /* If there's been a change in the mask write it back
     * to the hardware. */
    if (atc260x->irq_masks_cur != atc260x->irq_masks_cache) {

        dev_dbg(atc260x->dev, "IRQ mask sync: %x = %x\n",
            atc2603_INTS_MSK,
            atc260x->irq_masks_cur);
        
        atc260x->irq_masks_cache = atc260x->irq_masks_cur;
        atc260x_reg_write(atc260x,
                 atc2603_INTS_MSK,
                 atc260x->irq_masks_cur);
    }

    mutex_unlock(&atc260x->irq_lock);
}


static void atc260x_irq_enable(struct irq_data *data)
{
    struct atc260x_dev *atc260x = irq_data_get_irq_chip_data(data);
    int internal_irq = irq_to_atc260x_irq(atc260x, data->irq);

    atc260x->irq_masks_cur |= ATC260X_INT_BIT(internal_irq);
}

static void atc260x_irq_disable(struct irq_data *data)
{
    struct atc260x_dev *atc260x = irq_data_get_irq_chip_data(data);
    int internal_irq = irq_to_atc260x_irq(atc260x, data->irq);

    atc260x->irq_masks_cur &= ~ATC260X_INT_BIT(internal_irq);
}

/* only support HIGH Level type interrupt */
static int atc260x_irq_set_type(struct irq_data *data, unsigned int type)
{
    /* do nothting */

    return 0;
}

static struct irq_chip atc260x_irq_chip = {
    .name                   = "atc260x_irq",
    .irq_bus_lock           = atc260x_irq_lock,
    .irq_bus_sync_unlock    = atc260x_irq_sync_unlock,
    .irq_enable             = atc260x_irq_enable,
    .irq_disable            = atc260x_irq_disable,
    .irq_set_type           = atc260x_irq_set_type,
};

/* The processing of the primary interrupt occurs in a thread so that
 * we can interact with the device over SPI. */
static irqreturn_t atc260x_irq_thread(int irq, void *data)
{
    struct atc260x_dev *atc260x = data;
    unsigned int i;
    int pending, mask;

    dev_dbg(atc260x->dev, "%s(irq:%d)\n", __FUNCTION__, irq);

    pending = atc260x_reg_read(atc260x, atc2603_INTS_PD);
    if (pending < 0) {
        dev_err(atc260x->dev, "Failed to read INTS_PD interrupt: %d\n",
            pending);
        goto out;
    }

    mask = atc260x_reg_read(atc260x, atc2603_INTS_MSK);
    if (mask < 0) {
        dev_err(atc260x->dev, "Failed to read INTS_MSK interrupt: %d\n",
            mask);
        goto out;
    }

    dev_dbg(atc260x->dev, "%s(irq:%d): pending %02x, mask %02x, irq_masks_cur %x\n", 
        __FUNCTION__, irq, pending, mask, atc260x->irq_masks_cur);

    /* Apply masking */
    mask &= atc260x->irq_masks_cur;
    atc260x_reg_write(atc260x, atc2603_INTS_MSK, mask);
    
    for (i = 0; i < ATC260X_IRQ_NUM; i++) {
        if (!(pending & ATC260X_INT_BIT(i)))
            continue;

        /* Report it if it isn't masked, or forget the status. */
        if (atc260x->irq_masks_cur & ATC260X_INT_BIT(i)) {
            dev_dbg(atc260x->dev, "%s(): call handle_nested_irq(%d)\n",
                __FUNCTION__, atc260x->irq_base + i);
            handle_nested_irq(atc260x->irq_base + i);
        }
    }

out:

    dev_dbg(atc260x->dev, "%s(irq:%d) return\n", __FUNCTION__, irq);

    return IRQ_HANDLED;
}

int atc260x_irq_init(struct atc260x_dev *atc260x, int irq)
{
    struct atc260x_pdata *pdata = atc260x->dev->platform_data;
    int cur_irq, ret;

    dev_info(atc260x->dev, "%s(irq:%d) irq_base:%d\n", 
        __FUNCTION__, irq, pdata->irq_base);
    
    mutex_init(&atc260x->irq_lock);

    /* Mask the individual interrupt sources */
    atc260x_reg_write(atc260x, atc2603_INTS_MSK, 0x0);

    atc260x->irq_masks_cur = 0x0;
    atc260x->irq_masks_cache = 0x0;

    if (!irq) {
        dev_warn(atc260x->dev,
             "No interrupt specified - functionality limited\n");
        return 0;
    }

    if (!pdata || !pdata->irq_base) {
        dev_err(atc260x->dev,
            "No interrupt base specified, no interrupts\n");
        return 0;
    }

    atc260x->irq = irq;
    atc260x->irq_base = pdata->irq_base;

    /* Register them with genirq */
    for (cur_irq = atc260x->irq_base;
         cur_irq < ATC260X_IRQ_NUM + atc260x->irq_base;
         cur_irq++) {
            
        irq_set_chip_data(cur_irq, atc260x);
        irq_set_chip_and_handler(cur_irq, &atc260x_irq_chip,
            handle_level_irq);
        irq_set_nested_thread(cur_irq, 1);

		/* ARM needs us to explicitly flag the IRQ as valid
		 * and will set them noprobe when we do so. */
#ifdef CONFIG_ARM
		set_irq_flags(cur_irq, IRQF_VALID);
#else
		irq_set_noprobe(cur_irq);
#endif
    }

    /* reset ATC260X INTC */
    atc260x_cmu_reset(atc260x, ATC260X_CMU_MODULE_INTS);

    /* Enable P_EXTIRQ pad */
    atc260x_set_bits(atc260x, atc2603_PAD_EN, 0x1, 0x1);

    ret = request_threaded_irq(irq, NULL, atc260x_irq_thread,
                   IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
                   "atc260x_irq", atc260x);
    if (ret != 0) {
        dev_err(atc260x->dev, "Failed to request IRQ %d: %d\n",
            irq, ret);
        return ret;
    }

    return 0;
}

void atc260x_irq_exit(struct atc260x_dev *atc260x)
{
    if (atc260x->irq)
        free_irq(atc260x->irq, atc260x);
}

