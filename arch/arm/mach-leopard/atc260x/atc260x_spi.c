/*
 * atc260x-spi.c  --  SPI access for Wolfson WM831x PMICs
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
#include <linux/spi/spi.h>

#include <mach/hardware.h>
#include <mach/atc260x/atc260x.h>
#include <mach/atc260x/atc260x_pdata.h>
#include <mach/atc260x/atc260x_dev.h>


#define delay_cell	0x0000000	//0x1000000:delay 1 hclk cycle;0x2000000:delay 2 hclk cycle
#define MAX_WAIT_TIME	0x500000
#define MAX_BUSY_WAIT_TIME 0xfffffff

#undef read_reg
#define read_reg(reg) act_readl(reg)

#undef write_reg
#define write_reg(reg, val) act_writel(val,reg)


/********************************************************************************
spi_shift_out: transmit the data out from SPI_TXDAT; here SPI FIFO is 16bit,halfword
        Input Parameter:      No
        Output Parameter:     0:timeout
        		      >0,shift OK	
********************************************************************************/ 
static int spi_shift_out(unsigned int var,unsigned int device)
{
	int temp;
	int wait_times = MAX_WAIT_TIME;
	if(device==0)
	{
		write_reg(SPI0_TXDAT,var);	
		do
		{
			temp =read_reg(SPI0_STAT);
			temp =temp&0x04;
		}
		while((temp==0)&&(--wait_times>0));	
		write_reg(SPI0_STAT,temp);	
	}
	if(device==1)
	{
		write_reg(SPI1_TXDAT,var);	
		do
		{
			temp =read_reg(SPI1_STAT);
			temp =temp&0x04;
		}
		while((temp==0)&&(--wait_times>0));	
		write_reg(SPI1_STAT,temp);	
	}
	return(temp);
}


//static int wait_transmit_complete(void)
//{
//    int timeout = 10000;
//
//    while((timeout > 0) && ((act_readl(SPI0_STAT) & 0x04) == 0))
//    {
//        timeout--;
//    }
//
//    if (timeout <= 0)
//    {
//        pr_err("[ATC260X] spi transfer timeout! (SPICTL:0x%08x, SPI0_STAT:0x%x)\n", 
//            act_readl(SPI0_CTL), act_readl(SPI0_STAT));
//        return -1;
//    }
//
//    act_writel(act_readl(SPI0_STAT) | 0x04, SPI0_STAT);
//
//    return 0;
//}

void spi_direct_write(unsigned short addr, unsigned short data)
{
	unsigned short temp;
	temp=read_reg(0xb01c00e0);
	if ((temp&0xf)==0x03)		//device is SPI0
	{
		write_reg(SPI0_CTL,0x401c0+delay_cell);  	//16bit mode,pull down spi_ss line			
		temp=(addr<<3)|0x8000;			//bit15=1 is write flag,low 15 bit is the register address		
		spi_shift_out(temp,0);			//send out the write flag and address
		spi_shift_out(data,0);			//send out the write data
		write_reg(SPI0_CTL,0x401d0+delay_cell);  //16bit mode,pull up spi_ss line
	}
	else				//device is SPI1
	{
		write_reg(SPI1_CTL,0x401c0+delay_cell);  	//16bit mode,pull down spi_ss line			
		temp=(addr<<3)|0x8000;			//bit15=1 is write flag,low 15 bit is the register address		
		spi_shift_out(temp,1);			//send out the write flag and address
		spi_shift_out(data,1);			//send out the write data
		write_reg(SPI1_CTL,0x401d0+delay_cell);  //16bit mode,pull up spi_ss line		
	}
}

unsigned int spi_direct_read(unsigned short addr)
{
	
	unsigned short temp;
	temp=read_reg(0xb01c00e0);
	if ((temp&0xf)==0x03)			//device is SPI0
	{
		write_reg(SPI0_CTL,0x401c0+delay_cell);  	//16bit mode,pull down spi_ss line
	
		temp=(addr<<3)&0x7fff;			//bit15=0 is read flag,low 15 bit is the register address		
		spi_shift_out(temp,0);			//send out the read flag and address
	
		write_reg(SPI0_STAT,0x30);  		//reset the fifo	
	
		spi_shift_out(0x0,0);			//send out read clock
		temp=read_reg(SPI0_RXDAT);			//read the register value
		write_reg(SPI0_CTL,0x401d0+delay_cell);  	//16bit mode,pull up spi_ss line	
		return(temp);
	}
	else					//device is SPI1
	{
		write_reg(SPI1_CTL,0x401c0+delay_cell);  	//16bit mode,pull down spi_ss line
	
		temp=(addr<<3)&0x7fff;			//bit15=0 is read flag,low 15 bit is the register address		
		spi_shift_out(temp,1);			//send out the read flag and address
	
		write_reg(SPI1_STAT,0x30);  		//reset the fifo	
	
		spi_shift_out(0x0,1);			//send out read clock
		temp=read_reg(SPI1_RXDAT);			//read the register value
		write_reg(SPI1_CTL,0x401d0+delay_cell);  	//16bit mode,pull up spi_ss line	
		return(temp);
	}
}


static int atc260x_spi_direct_read_device(struct atc260x_dev *atc260x, unsigned short reg,
                  int bytes, void *dest)
{
//    unsigned short txdat;
    unsigned short *d = dest;

    /* only support 2 bytes data */
    if ((dest == NULL) || (bytes != 2))
        return -EINVAL;

//    act_writel(0x1f, SPI0_CLKDIV);    //set spi clkdiv hclk / 32
//
//    act_writel(0x30, SPI0_STAT);    //clear SPI FIFO
//    act_writel(0x401c0, SPI0_CTL);  //16bit mode 3,SPI CS- low
//
//    txdat = ((reg & 0x0fff) << 3) & 0x7fff;
//
//    act_writel(txdat, SPI0_TXDAT); //16bit mode 3,SPI CS- low
//    wait_transmit_complete();
//
//    act_writel(0x30, SPI0_STAT);    //clear SPI FIFO
//
//    act_writel(0, SPI0_TXDAT);      //generate SPI read clock
//    wait_transmit_complete();
//
//    *d = act_readl(SPI0_RXDAT) & 0xffff;    //read data
//
//    act_writel(0x401d0, SPI0_CTL); //SPI cs pin high
    *d = spi_direct_read(reg);
    return 0;
}

static int atc260x_spi_direct_write_device(struct atc260x_dev *atc260x, unsigned short reg,
                   int bytes, void *src)
{
//    unsigned short txdat;
    unsigned short *s = src;

    /* only support 2 bytes data */
    if ((src == NULL) || (bytes != 2))
        return -EINVAL;

//    act_writel(0x1f, SPI0_CLKDIV);    //set spi clkdiv hclk / 32
//
//    act_writel(0x30, SPI0_STAT);    //clear SPI FIFO
//    act_writel(0x401c0, SPI0_CTL);  //16bit mode 3,SPI CS- low
//
//    txdat = ((reg & 0x0fff) << 3) | 0x8000;
//    act_writel(txdat, SPI0_TXDAT); //16bit mode 3,SPI CS- low
//    wait_transmit_complete();
//
//    act_writel(0x30, SPI0_STAT);    //clear SPI FIFO
//    act_writel(*s, SPI0_TXDAT);      //generate SPI read clock
//    wait_transmit_complete();
//
//    act_writel(0x401d0, SPI0_CTL); //SPI cs pin high
    spi_direct_write(reg, *s);

    return 0;
}

static int atc260x_spi_read_device(struct atc260x_dev *atc260x, unsigned short reg,
                  int bytes, void *dest)
{
    u16 tx_val;
    u16 *d = dest;
    int r, ret;

    /* Go register at a time */
    for (r = reg; r < reg + (bytes / 2); r++) {
        tx_val = (r << 3) & 0x7fff;

        ret = spi_write_then_read(atc260x->control_data,
                      (u8 *)&tx_val, 2, (u8 *)d, 2);
        if (ret != 0)
            return ret;

        *d = le16_to_cpu(*d);

        d++;
    }

    return 0;
}

static int atc260x_spi_write_device(struct atc260x_dev *atc260x, unsigned short reg,
                   int bytes, void *src)
{
    struct spi_device *spi = atc260x->control_data;
    u16 *s = src;
    u16 data[2];
    int ret, r;
    u16 tx_val;

    /* Go register at a time */
    for (r = reg; r < reg + (bytes / 2); r++) {
        
        tx_val = (r << 3) | 0x8000;

        data[0] = tx_val;
        data[1] = *s++;
        data[1] = cpu_to_le16(data[1]);

        ret = spi_write(spi, (char *)&data, sizeof(data));
        if (ret != 0)
            return ret;
    }

    return 0;
}

static int atc260x_spi_set_access_mode(struct atc260x_dev *atc260x, int mode)
{
    if (mode == ATC260X_ACCESS_MODE_NORMAL) {
        atc260x->read_dev = atc260x_spi_read_device;
        atc260x->write_dev = atc260x_spi_write_device;
    } else if (mode == ATC260X_ACCESS_MODE_DIRECT){
        atc260x->read_dev = atc260x_spi_direct_read_device;
        atc260x->write_dev = atc260x_spi_direct_write_device;
    } else {
        return -EINVAL;
    }

    return 0;
}

static int __devinit atc260x_spi_probe(struct spi_device *spi)
{
    struct atc260x_dev *atc260x;
    unsigned int temp;
    pr_info("%s()\n", __FUNCTION__);

    /* Currently SPI support for ID tables is unmerged, we're faking it */
    if (strcmp(spi->modalias, "atc260x") != 0) {
        dev_err(&spi->dev, "Unknown device type\n");
        return -EINVAL;
    }

    atc260x = kzalloc(sizeof(struct atc260x_dev), GFP_KERNEL);
    if (atc260x == NULL)
        return -ENOMEM;

    spi->bits_per_word = 16;
    spi->mode = SPI_MODE_3;

    dev_set_drvdata(&spi->dev, atc260x);
    atc260x->dev = &spi->dev;
    atc260x->control_data = spi;
    atc260x->read_dev = atc260x_spi_read_device;
    atc260x->write_dev = atc260x_spi_write_device;
    atc260x->set_access_mode = atc260x_spi_set_access_mode;
	
	temp=read_reg(CMU_DEVCLKEN0);
	act_writel(temp|0x00040000,CMU_DEVCLKEN0);
	
	temp=read_reg(PAD_CTL);
	act_writel(temp|0x02,PAD_CTL);
	
	
    temp=read_reg(MFP_CTL3);
	printk("mfp_ctl3=0x%x\n",temp);
	temp&=0x8fc0ffff;
	temp|=0xA0140000;
	act_writel(temp,MFP_CTL3);		
	write_reg(SPI1_CLKDIV,0xf);		//set SPI_CLK=H_clk/(CLKDIV*2)=3MHz , H_clk=60M
	act_writel(0x401d0,SPI1_CTL);

	temp=read_reg(SPI1_STAT);
	act_writel(temp,SPI1_STAT);		////clear SPI status register	
	
  //  atc260x_spi_set_access_mode(atc260x, ATC260X_SPI_ACCESS_MODE_DIRECT);
   		printk("MFP_CTL2:       0x%x\n", act_readl(MFP_CTL2));
		printk("spi1_ctl:       0x%x\n", act_readl(SPI1_CTL));
		printk("MFP_CTL3:       0x%x\n", act_readl(MFP_CTL3));
		printk("CMU_DEVCLKEN0:  0x%x\n", act_readl(CMU_DEVCLKEN0));
		printk("CMU_DEVCLKEN1:  0x%x\n", act_readl(CMU_DEVCLKEN1));
    return atc260x_spi_device_init(atc260x, spi->irq);
}

static int __devexit atc260x_spi_remove(struct spi_device *spi)
{
    struct atc260x_dev *atc260x = dev_get_drvdata(&spi->dev);

    atc260x_device_exit(atc260x);

    return 0;
}

static int atc260x_spi_suspend(struct spi_device *spi, pm_message_t m)
{
    struct atc260x_dev *atc260x = dev_get_drvdata(&spi->dev);

    return atc260x_device_suspend(atc260x);
}

static int atc260x_spi_resume(struct spi_device *spi)
{
    struct atc260x_dev *atc260x = dev_get_drvdata(&spi->dev);

    return atc260x_device_resume(atc260x);
}

static struct spi_driver atc260x_spi_driver = {
    .driver = {
        .name   = "atc260x",
        .bus    = &spi_bus_type,
        .owner  = THIS_MODULE,
    },
    .probe      = atc260x_spi_probe,
    .remove     = __devexit_p(atc260x_spi_remove),
	.suspend    = atc260x_spi_suspend,
	.resume     = atc260x_spi_resume,
};

static int __init atc260x_spi_init(void)
{
    int ret;
	int pmu_type ;
   pmu_type = get_pmu_type();
   if (pmu_type ==ATC2603C)
		return 0;
    ret = spi_register_driver(&atc260x_spi_driver);
    if (ret != 0)
        pr_err("Failed to register ATC260X SPI driver: %d\n", ret);

    return 0;
}
subsys_initcall(atc260x_spi_init);

static void __exit atc260x_spi_exit(void)
{
    spi_unregister_driver(&atc260x_spi_driver);
}
module_exit(atc260x_spi_exit);

MODULE_DESCRIPTION("SPI support for ATC260X PMIC");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Actions Semi, Inc");

