/*
 * atc260x-i2c.c  --  IIC access for Actions atc260x
 *
 * Copyright 2014 Actions Semi Inc.
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
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <mach/hardware.h>
#include <mach/asoc_i2c.h>
#include <linux/delay.h>
#include <mach/hardware.h>
#include <mach/atc260x/atc260x.h>
#include <mach/atc260x/atc260x_pdata.h>
#include <mach/atc260x/atc260x_dev.h>
#define PMU_NAME "act260x"

#define delay_cell	0x0000000	//0x1000000:delay 1 hclk cycle;0x2000000:delay 2 hclk cycle
#define MAX_WAIT_TIME	0x500000
#define MAX_BUSY_WAIT_TIME 0xfffffff

#undef read_reg
#define read_reg(reg) act_readl(reg)

#undef write_reg
#define write_reg(reg, val) act_writel(val,reg)

//#define ACT260X_I2C_ADAPTER 2
#define PMU_I2C_ADDRESS (0xCA>>1)
#define ATC260X_5307_SADDR (0xCA>>1)
static struct i2c_client * pClient = NULL;
extern struct  atc260x_pdata leopard_fpga_atc260x_pdata;

#if 0
#define BOOT_I2C_DBG(stuff...)      printk(stuff)
#define BOOT_I2C_ERR(stuff...)      printk(stuff)
#else
#define BOOT_I2C_DBG(stuff...)      do{}while(0)
#define BOOT_I2C_ERR(stuff...)      do{}while(0)
#endif

struct asoc_i2c_dev_direct
{
	__u32 base;
};
#if 1
static inline u32 asoc_i2c_readl(struct asoc_i2c_dev *i2c_dev, int reg)
{
    return act_readl((u32)(i2c_dev->base + reg));
}

static inline void asoc_i2c_writel(struct asoc_i2c_dev *i2c_dev, u32 val, int reg)
{
   // printk("-->>write 0x%x to 0x%x\n",val, (u32)(i2c_dev->base +reg));
    act_writel(val, (u32)i2c_dev->base + reg);
    act_readl((u32)i2c_dev->base + reg);
}
#endif//0
 
 
 
 #define I2C_WRITE	(0)
#define I2C_READ	(1)

#define I2C_OK			0
#define I2C_NOK			1
#define I2C_NACK		2
#define I2C_NOK_TOUT	3
#define I2C_TIMEOUT		1
/*
 * direct msg.
 */
typedef struct _i2c_dir_msg_t {
	u8	type;		//read or write
	u8	s_addr;		//slave address
	u8	r_addr;		//reg address
	u8  len;		//data length
	u8 * data;		//data buffer
	
}i2c_dmsg_t;
 
 
 
 static int _do_i2c_transfer(struct asoc_i2c_dev *dev,	i2c_dmsg_t * msg)
{	
	u32 val = 0, i = 0, result = I2C_OK;
	u32 i2c_cmd;
	u8 chip, addr_len, data_len;
	u8 *addr;
	u8 *data;
	
	chip = msg->s_addr;
	addr = &(msg->r_addr);
	addr_len = 1; //fix 
	data = msg->data;
	data_len = msg->len;

	BOOT_I2C_DBG("i2c_transfer: bad callmsg-¡·type %d\n",msg->type);
	//printk("i2c_transfer: bad callmsg-¡·type %d\n",msg->type);
	switch (msg->type) {
		
	case I2C_WRITE:

		/*1, enable i2c ,not enable interrupt*/
		asoc_i2c_writel(dev, 0x80, I2C_CTL);
		/*2, write data count*/
		asoc_i2c_writel(dev, data_len, I2C_DATCNT);
		/*3, write slave addr*/
		asoc_i2c_writel(dev, (chip << 1), I2C_TXDAT);
		/*4, write register addr*/
		for (i = 0; i < addr_len; i++)
			asoc_i2c_writel(dev, addr[i], I2C_TXDAT);
		/*5, write data*/
		for (i = 0; i < data_len; i++)
			asoc_i2c_writel(dev, data[i], I2C_TXDAT);
		/*6, write fifo command */
		i2c_cmd = I2C_CMD_X | I2C_CMD_AS((addr_len) + sizeof(chip));
		asoc_i2c_writel(dev, i2c_cmd, I2C_CMD);
		//udelay(10*1000);
		mdelay(1);
		val = asoc_i2c_readl(dev, I2C_FIFOSTAT);
		if (val & I2C_FIFOSTAT_RNB) {
			result = I2C_NACK;
			return result;
		}
		/*7, check the status.*/
		for (i = 0; i < 3; i++) {
			val = asoc_i2c_readl(dev, I2C_FIFOSTAT);
			if (val & I2C_FIFOSTAT_CECB) {
				result = I2C_OK;
				break;
			} else {
				result = I2C_NOK_TOUT;
				udelay(1000);
				continue;
			}
		}
		break;

	case I2C_READ:
		/*1, enable i2c ,not enable interrupt*/
		asoc_i2c_writel(dev, 0x80, I2C_CTL);
		/*2, write data count*/
		asoc_i2c_writel(dev, data_len, I2C_DATCNT);
		/*3, write slave addr*/
		asoc_i2c_writel(dev, (chip << 1), I2C_TXDAT);
		/*4, write register addr*/
		for (i = 0; i < addr_len; i++)
			asoc_i2c_writel(dev, addr[i], I2C_TXDAT);
		/*5, write slave addr | read_flag*/
		asoc_i2c_writel(dev, (chip << 1) | I2C_READ, I2C_TXDAT);
		/*6, write fifo command */

		i2c_cmd = I2C_CMD_X | I2C_CMD_RBE | I2C_CMD_NS \
			| I2C_CMD_SAS(sizeof(chip)) |\
			I2C_CMD_AS(sizeof(chip) + addr_len);
		//i2c_writel((u32)&i2c->cmd, i2c_cmd);
		asoc_i2c_writel(dev, i2c_cmd, I2C_CMD);
		//udelay(10*1000);
		mdelay(1);
		val = asoc_i2c_readl(dev, I2C_FIFOSTAT);
		if (val & I2C_FIFOSTAT_RNB) {
			result = I2C_NACK;
			return result;
		}
		/*7, check the status.*/
		for (i = 0; i < 3; i++) {
			val = asoc_i2c_readl(dev, I2C_FIFOSTAT);
			if (val & I2C_FIFOSTAT_CECB) {
				result = I2C_OK;
				break;
			} else {
				result = I2C_NOK_TOUT;
				udelay(1000);
				continue;
			}
		}
		/*8, Read data from rxdata*/
		for (i = 0; i < data_len; i++) {
			data[i] = asoc_i2c_readl(dev, I2C_RXDAT);
			//i2c_dbg("-->>Read data[%d] = 0x%02x\r\n", i, data[i]);
		}
		break;

	default:
		printk("i2c_transfer: bad call\n");
		result = I2C_NOK;
		break;
	}
	return result;
}

static int _i2c_transfer(struct i2c_adapter *adap,	i2c_dmsg_t * msg)
{
	struct asoc_i2c_dev *dev = i2c_get_adapdata(adap);
	u32 divide;
	u32 freq = 0;
	int ret;

   // dev->base = 0xb0180000;
    //BOOT_I2C_DBG("%s(): msg num , dev->base: 0x%08x\n", 
	//	__FUNCTION__, dev->base);
	
	
	
///BOOT_I2C_DBG(" %s %d\n",__FUNCTION__,__LINE__);
	freq = dev->i2c_freq;
		//BOOT_I2C_DBG(" %s %d\n",__FUNCTION__,__LINE__);
	divide = I2C_MODULE_CLK / (freq * 16);
		//BOOT_I2C_DBG(" %s %d\n",__FUNCTION__,__LINE__);
	mutex_lock(&dev->mutex);
		//	BOOT_I2C_DBG(" %s %d %p\n",__FUNCTION__,__LINE__,&dev->mutex);
	asoc_i2c_writel(dev, divide, I2C_CLKDIV);
    asoc_i2c_writel(dev, 0xff, I2C_STAT);//reset all the stats
    asoc_i2c_writel(dev, I2C_CTL_EN | I2C_CTL_PUEN, I2C_CTL);	//disable inttrupt.
	//BOOT_I2C_DBG(" %s %d\n",__FUNCTION__,__LINE__);
	ret = _do_i2c_transfer(dev, msg);
	//disable adapter.
	asoc_i2c_writel(dev, 0, I2C_CTL);
	mutex_unlock(&dev->mutex);
	return ret;
}
static int _i2c_read(struct i2c_adapter *adap, u8 addr, u8 reg, u8 *data, u32 len)
{
	int ret = 0;
	i2c_dmsg_t msg;
	
	msg.type = I2C_READ;
	msg.s_addr = addr;
	msg.r_addr = reg;
	msg.len = len;
	msg.data = data;
	BOOT_I2C_DBG(" %s %d adap=%p msg=%p\n",__FUNCTION__,__LINE__,adap,&msg);
	ret = _i2c_transfer(adap, &msg);
	if (ret != 0) {
		pr_err("I2c%d read: failed %d\n", adap->nr, ret);
		return 1;
	}
	return 0;
}

static int _i2c_write(struct i2c_adapter *adap, u8 addr, u8 reg, u8 *data, u32 len)
{
	i2c_dmsg_t msg;
	int ret = 0;

	msg.type = I2C_WRITE;
	msg.s_addr = addr;
	msg.r_addr = reg;
	msg.len = len;
	msg.data = data;
	 BOOT_I2C_DBG("reg0x22 reg=0x%0x\n",reg);
	ret = _i2c_transfer(adap, &msg);
	if (reg==0x22)
	  BOOT_I2C_DBG("reg0x22 ret=%d\n",ret);
	if (ret != 0) {
		pr_err("I2c%d write: failed %d\n",adap->nr,  ret);
		return 1;
	}
	return 0;
}

 
 
 static int atc260x_iic_read_device(struct atc260x_dev *atc260x, unsigned short reg,
                  int bytes, void *dest)
{
	struct i2c_client *client = to_i2c_client(atc260x->dev);
	struct i2c_adapter *adap = client->adapter;
    u16 *d = dest;
	struct i2c_msg msg[2];
    int ret;
	char buffer[2];
    //BOOT_I2C_DBG("TODO TEST %s \n",__FUNCTION__);
    msg[0].addr = ATC260X_5307_SADDR;
	msg[0].flags = 0;
	msg[0].buf = (u8 *)&reg;
	msg[0].len = 1;	//write 1 byte address

	msg[1].addr = ATC260X_5307_SADDR;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = buffer;
	msg[1].len = 2;	//read back 2 bytes data.
	
	ret = i2c_transfer(adap, msg, 2);
	if (ret > 0){
		*d = buffer[1] | (buffer[0] << 8);
		}
    return ret;
}


static int atc260x_iic_direct_read_device(struct atc260x_dev *atc260x, unsigned short reg,
                  int bytes, void *dest)
{
	int ret;
	struct i2c_client *client = to_i2c_client(atc260x->dev);
	struct i2c_adapter *adap = client->adapter;
	char buffer[2];
	u16 *d = dest;
	
	ret = _i2c_read(adap, ATC260X_5307_SADDR, reg, buffer, 2);
	if (!ret){
		*d = buffer[1] | (buffer[0] << 8);
		}
	return ret;
}
static int atc260x_iic_write_device(struct atc260x_dev *atc260x, unsigned short reg,
                   int bytes, void *src)
{
	struct i2c_client *client = to_i2c_client(atc260x->dev);
	struct i2c_adapter *adap = client->adapter;
	u16 *s = src;
    struct i2c_msg msg[1];
	int ret;
	char buffer[3];

	buffer[0] = reg;
	buffer[1] = (*s >> 8) & 0xFF;
	buffer[2] = *s & 0xFF;
	
    msg[0].addr = ATC260X_5307_SADDR;
	msg[0].flags = 0;
	msg[0].buf = buffer;
	msg[0].len = 3;	//write 2 bytes.
	
	ret = i2c_transfer(adap, msg, 1);
	//if (ret==1)
	//	return 0;
    return 0;//linux3.4
}

static int atc260x_iic_direct_write_device(struct atc260x_dev *atc260x, unsigned short reg,
                  int bytes, void *src)
{
	int ret;
	struct i2c_client *client = to_i2c_client(atc260x->dev);
	struct i2c_adapter *adap = client->adapter;
	char buffer[2];
	u16 *s = src;

	buffer[0] = (*s >> 8) & 0xFF;
	buffer[1] = *s & 0xFF;
	ret = _i2c_write(adap, ATC260X_5307_SADDR, reg, buffer, 2);

	return ret;
}


static int atc260x_iic_set_access_mode(struct atc260x_dev *atc260x, int mode)
{
	BOOT_I2C_DBG(" %s %d\n",__FUNCTION__,__LINE__);
	if (mode == ATC260X_ACCESS_MODE_NORMAL) {
		BOOT_I2C_DBG(" %s %d\n",__FUNCTION__,__LINE__);
		atc260x->read_dev = atc260x_iic_read_device;
		atc260x->write_dev = atc260x_iic_write_device;
	} else if (mode == ATC260X_ACCESS_MODE_DIRECT){
		BOOT_I2C_DBG(" %s %d\n",__FUNCTION__,__LINE__);
		atc260x->read_dev = atc260x_iic_direct_read_device;
		atc260x->write_dev = atc260x_iic_direct_write_device;
	} else {
		return -EINVAL;
	}
	return 0;
}

#if 0
static int s_jtag(void)
{
    unsigned int tmp = 0;
	unsigned int volatile ii = 1;
    /**/
    tmp = act_readl(GPIO_COUTEN);
    tmp &= (~0x000c3400);
    act_writel(tmp, GPIO_COUTEN);
    
    tmp = act_readl(GPIO_CINEN);
    tmp &= (~0x000c3400);
    act_writel(tmp, GPIO_CINEN);    

    tmp = act_readl(MFP_CTL1);
    tmp &= (~((0x7<<29) | (0x7<<26)));
    act_writel(tmp, MFP_CTL1);

    tmp = act_readl(MFP_CTL2);
    tmp &= (~((0x3<<5) | (0x3<<7) | (0x7<<11) | (0x7<<17)));
    tmp |= (0x2<<5) | (0x3<<7) | (0x3<<11) | (0x3<<17);
    
    act_writel(tmp, MFP_CTL2);

    while(ii);  
    return 0;
}

#endif //0 debug code
static int __devinit atc260x_i2c_probe(struct i2c_client *i2c,
    const struct i2c_device_id *id)
{
		int ret,temp;
    struct atc260x_dev *atc260x;
    //s_jtag();
    pr_info("%s()\n", __FUNCTION__);
    //printk("%s, %d\n", __FUNCTION__, __LINE__);

   if (strcmp(i2c->name, PMU_NAME) != 0) {
        dev_err(&i2c->dev, "Unknown device type\n");
        return -EINVAL;
    }
    //printk("%s, %d\n", __FUNCTION__, __LINE__);
    
    ret = i2c_check_functionality(i2c->adapter, I2C_FUNC_SMBUS_BYTE_DATA);
    if ( !ret ) {
        printk("I2c bus dosen't support");
        ret = -EFAULT;
        goto i2c_check_failed;
    }

    //printk("%s, %d\n", __FUNCTION__, __LINE__);


    atc260x = kzalloc(sizeof(struct atc260x_dev), GFP_KERNEL);
    if (atc260x == NULL)
        return -ENOMEM;

    i2c_set_clientdata(i2c, atc260x);
    printk("%s, %d\n", __FUNCTION__, __LINE__);

    atc260x->dev = &i2c->dev;
    atc260x->control_data = i2c;
    //atc260x->read_dev = atc260x_i2c_read_device;
   // atc260x->write_dev = atc260x_i2c_write_device;atc260x_iic_write_device   

   atc260x->read_dev = atc260x_iic_read_device;
    atc260x->write_dev = atc260x_iic_write_device;
   
   

    atc260x->set_access_mode = atc260x_iic_set_access_mode;
    //atc260x_spi_set_access_mode(atc260x, ATC260X_SPI_ACCESS_MODE_DIRECT);
	printk("i2c->irq=%d\n",i2c->irq);
    temp=act_readl(PAD_CTL);
	//enable pad
	act_writel(temp|0x02, PAD_CTL);

	temp=act_readl(MFP_CTL3);
	//  config 24M output to 530x
	act_writel(temp|0x80000000, MFP_CTL3);

	
     //config i2c0
	act_writel((act_readl(MFP_CTL3) & 0xfff8ffff), MFP_CTL3);
	//enable I2C0 clock
    act_writel(act_readl(CMU_DEVCLKEN1)|0x4000,CMU_DEVCLKEN1);
	//enable i2c0 pull-up
	act_writel((act_readl(PAD_PULLCTL0) | 0x0300), PAD_PULLCTL0);
    return atc260x_i2c_device_init(atc260x, i2c->irq);

i2c_check_failed:
		return ret;    
}

static int __devexit atc260x_i2c_remove(struct i2c_client *i2c)
{
    struct atc260x_dev *atc260x = (struct atc260x_dev *)i2c_get_clientdata(i2c);

    atc260x_device_exit(atc260x);

    return 0;
}

static int atc260x_i2c_suspend(struct i2c_client *i2c, pm_message_t m)
{
    struct atc260x_dev *atc260x = (struct atc260x_dev *)i2c_get_clientdata(i2c);

    return atc260x_device_suspend(atc260x);
}

static int atc260x_i2c_resume(struct i2c_client *i2c)
{
    struct atc260x_dev *atc260x = (struct atc260x_dev *)i2c_get_clientdata(i2c);

    return atc260x_device_resume(atc260x);
}

static struct i2c_board_info atc260x_info = {
    .type  = PMU_NAME,
    .addr  = PMU_I2C_ADDRESS,
    .irq    = IRQ_SIRQ2,
    .platform_data = &leopard_fpga_atc260x_pdata,
};	

static struct i2c_device_id atc260x_id[] = {
    {PMU_NAME, 0},
    {},
};
static struct i2c_driver atc260x_i2c_driver = {
    .driver = {
        .name   = PMU_NAME,
        .owner  = THIS_MODULE,
    },
    .probe      = atc260x_i2c_probe,
    .remove    = __devexit_p(atc260x_i2c_remove),
    .suspend   = atc260x_i2c_suspend,
    .resume    = atc260x_i2c_resume,
    .id_table	= atc260x_id,
};

#include <mach/bootafinfo.h>
int get_pmu_i2c_num(void)
{
	unsigned char *addr;
	addr = asoc_get_boot_afinfo();
	return *((unsigned char *)(addr+0xa));
}
EXPORT_SYMBOL_GPL(get_pmu_i2c_num);

static int __init atc260x_i2c_init(void)
{
   int ret;
   //return 0;
   int pmu_type, i2c_num;
   struct i2c_adapter *adap;
   
   pmu_type = get_pmu_type();
   //printk("__%s___ %d,pmu=%d\n",__FUNCTION__,__LINE__,pmu_type);
   if (pmu_type ==ATC2603A)
   {
       printk("pmu_type atc2603a not support i2c!\n");
		return -1;
   }
   i2c_num = get_pmu_i2c_num();
   if ( i2c_num == 2 )
   		adap = i2c_get_adapter(2); //i2c 2
   else
   		adap = i2c_get_adapter(0);//i2c 0
   
   printk("%s, %d, adap: i2c%d\n", __FUNCTION__, __LINE__, i2c_num);
   if (adap == NULL){
        ////printk("i2c_get_adapter failed: 0x%08x\n", (u32)adap);
	 return -1;	
    }	
   //switch_jtag();

    printk("%s, %d, adap->name: %s\n", __FUNCTION__, __LINE__, adap->name);
   
    pClient = i2c_new_device(adap, &atc260x_info); 
    if (pClient == NULL){
        printk("i2c_new_device failed: %d\n", ret);
	 return -1;	
    }
    BOOT_I2C_DBG("%s, %d\n", __FUNCTION__, __LINE__);		
    ret = i2c_add_driver(&atc260x_i2c_driver);
    if (ret != 0)
        printk("Failed to register ATC260X I2C driver: %d\n", ret);

    return 0;
}
subsys_initcall_sync(atc260x_i2c_init);

static void __exit atc260x_i2c_exit(void)
{
    i2c_del_driver(&atc260x_i2c_driver);
    i2c_unregister_device(pClient);	
}
module_exit(atc260x_i2c_exit);

MODULE_DESCRIPTION("SPI support for ATC260X PMIC");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Actions Semi, Inc");

