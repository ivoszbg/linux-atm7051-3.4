/*
 * tps65010 - driver for tps6501x power management chips
 *
 * Copyright (C) 2004 Texas Instruments
 * Copyright (C) 2004-2005 David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>

#include <asm/gpio.h>

#include <linux/random.h>
#include <linux/reboot.h>
#include "sw2001_aes.h"

//#define ENCRYPT_DEBUG

#ifdef ENCRYPT_DEBUG
/* If you are writing a driver, please use dev_dbg instead */
//#define PR_DEBUG(fmt, arg...) 	printk(KERN_INFO fmt, ##arg)
//#define PR_INFO(fmt, arg...)	printk(KERN_INFO fmt, ##arg)
#define PR_DEBUG(x...) 	printk(x)
#define PR_INFO(x...)	printk(x)
#else
#define PR_DEBUG(fmt, arg...) \
	({ if (0) printk(KERN_INFO fmt, ##arg); 0; })
#define PR_INFO(fmt, arg...) \
	({ if (0) printk(KERN_INFO fmt, ##arg); 0; })
#endif
/*-------------------------------------------------------------------------*/

#define	DRIVER_VERSION	"2015.09.21"
#define DEVICE_NAME	"umido" //"sw2001"
#define	DRIVER_NAME	(sw2001_driver.driver.name)

//MODULE_DESCRIPTION("SW2001 Encryption Driver");
MODULE_LICENSE("GPL");

static struct i2c_driver sw2001_driver;
static struct sw2001 *the_sw2001;

/*-------------------------------------------------------------------------*/
struct sw2001 {
	struct i2c_client	*client;
	struct mutex		lock;
	struct platform_device	*sw2001_device;
};

/*=============================SW2001 ENCRYPTION control===================================*/

/*-------------------------------------------------------------------------*/
/* sw2001_write parameter:
 * reg:  
 * data: 
 */
int sw2001_write(unsigned char reg, unsigned char *buf, int len)
{
	int	 status=0;

	if (!the_sw2001)
		return -ENODEV;

	mutex_lock(&the_sw2001->lock);

	//status = i2c_smbus_write_byte_data(the_sw2001->client, reg, data);
	status = i2c_smbus_write_i2c_block_data(the_sw2001->client, reg, len, buf);

	mutex_unlock(&the_sw2001->lock);
	
	PR_DEBUG("%s: sw2001_write(0x%x,0x%x)\n", DRIVER_NAME, reg, *buf);

	if(status)
		PR_INFO("error.. status=0x%x\n",status);
		
	if(status>=len) return 0;
	else return status;
}
EXPORT_SYMBOL(sw2001_write);

/* sw2001_read parameter:
 * reg:  
 * data: 
 */
int sw2001_read(unsigned char reg, unsigned char *buf, int len)
{
	int	 status=0;

	if (!the_sw2001)
		return -ENODEV;

	mutex_lock(&the_sw2001->lock);

	//status = i2c_smbus_read_byte_data(the_sw2001->client, reg);
	status = i2c_smbus_read_i2c_block_data(the_sw2001->client, reg, len, buf);

	mutex_unlock(&the_sw2001->lock);
	
	PR_DEBUG("%s: sw2001_read(0x%x) return 0x%x\n", DRIVER_NAME, reg, status);	
	
	PR_INFO("status=0x%x\n",status);	
	
	if(status>=len) return 0;
	else return status;	
}
EXPORT_SYMBOL(sw2001_read);

////////////////////////////////////////////////////////////////////////
//******************************************************************************
// function    : iDeviceInit(uint8_t device_addr, uint8_t speed)
// description : initial the i2c, including configure pin, device address and speed
// parameters  :
//               devaddr : device address for i2c
//               speed: interface speed for i2c
//               |       0 : 100k
//               |       1 : 400k
// return      : 
//               TRUE : sucess
//               FALSE : fail
//*******************************************************************************
BOOL iDeviceInit(uint8_t device_addr, uint8_t speed)
{
	return TRUE;
}

//*******************************************************************************
// function    : iDeviceDeInit()
// description : Deinitializes the i2c, release resource
// return      : 
//               TRUE : success
//               FALSE : fail
//********************************************************************************
BOOL iDeviceDeInit(void)
{
	return TRUE;
}

//********************************************************************************
// function    : iWriteByte(uint8_t addr, uint8_t data)
// description : write data to byte addr(internal register address in device)
// parameters  :
//                addr : register address in device
//                data:  data 
// return        : 
//               TRUE : success
//               FALSE : fail
//*********************************************************************************
BOOL iWriteByte(uint8_t addr, uint8_t data)
{
	sw2001_write(addr, &data, 1);
	return TRUE;
}

//*********************************************************************************
// function    : iReadByte(uint8_t addr, uint8_t data)
// description :  read data from addr(internal register address in device)
// parameters  :
//                  addr : register address in device
//                  data:  data 
// return        : 
//               TRUE : success
//               FALSE : fail
//*********************************************************************************
BOOL iReadByte(uint8_t addr, uint8_t *data)
{
	sw2001_read(addr, data, 1);
	return TRUE;
}

//*********************************************************************************
// function    : iSleep(uint8_t waittime)
// description :  host wait time, during this time  host cpu can do other thing
// parameters  :
//                  waittime : sleep time(ms)
// return        : 
//               TRUE : success
//               FALSE : fail
//*********************************************************************************
BOOL iSleep(uint8_t waittime)
{
	msleep(waittime);
	return TRUE;
}

//********************************************************************************
// function    : iWriteData(uint8_t addr, uint8_t *data, uint8_t len)
// description : write len bytes data to byte addr(internal address in device)
// parameters  :
//                addr : byte address in device
//                data:  data pointer
//                len  :  data length
// return        : 
//               TRUE : success
//               FALSE : fail
//*********************************************************************************
BOOL iWriteData(uint8_t addr, uint8_t *data, uint8_t len)
{
	uint8_t i;
	uint8_t *pt=data;
	for(i=0;i<len;i++)
	{
		if(iWriteByte(addr+i,*pt++)!=TRUE)
		{
			//printf("iwritedata function failed!");
			return FALSE;
		}
	}
	return TRUE;
}

//*********************************************************************************
// function    : iReadData(uint8_t addr, uint8_t *data, uint8_t len)
// description :  read len bytes from addr(internal address in device)
// parameters  :
//                  addr : byte address in device
//                  data:  data pointer
//                   len :  data length
// return        : 
//               TRUE : success
//               FALSE : fail
//*********************************************************************************
BOOL iReadData(uint8_t addr, uint8_t *data, uint8_t len)
{
	uint8_t i;
	uint8_t temp,*pt=data;
	for(i=0;i<len;i++)
	{
		if(iReadByte(addr+i,&temp) != TRUE)
		{
			//printf("ireaddata failed\n");
			return FALSE;
		}
		*pt++ = temp;
	}
	return TRUE;
}


//*********************************************************************************
// function    : iSetBit(uint8_t addr, uint8_t bit)
// description :  set 1 to bit in byte
// parameters  :
//                   addr : byte address in device
//                   bit:  position for set in byte
// return        : 
//               TRUE : success
//               FALSE : fail
//*********************************************************************************
BOOL iSetBits(uint8_t addr, uint8_t bit)
{
	uint8_t data;
	if(iReadByte(addr,&data)!=TRUE)
	{
		return FALSE;
	}
	data |= bit;
	if(iWriteByte(addr,data)!=TRUE)
	{
		return FALSE;
	}
	return TRUE;
}

//*********************************************************************************
// function    : iClearBit(uint8_t addr, uint8_t bit)
// description :  set 0 to bit in byte
// parameters  :
//                   addr : byte address in device
//                   bit:  position for clear in byte
// return        : 
//               TRUE : success
//               FALSE : fail
//*********************************************************************************
BOOL iClearBits(uint8_t addr, uint8_t bit)
{
	uint8_t data;
	if(iReadByte(addr,&data)!=TRUE)
	{
		return FALSE;
	}
	data &= ~bit;
	if(iWriteByte(addr,data)!=TRUE)
	{
		return FALSE;
	}
	return TRUE;
}

//*********************************************************************************
// function    : iCheckBits(uint8_t addr, uint8_t mask, uint8_t ref)
// description :  wait the value  of register equal to ref
// parameters  :
//                   addr : register address in device
//                   mask:  mask bit
//                   ref:  compare data
// return        : 
//               TRUE : success
//               FALSE : fail
//*********************************************************************************
BOOL iCheckBits(uint8_t addr, uint8_t mask, uint8_t ref)
{
	uint8_t data=0;
	iSleep(3);  //wait for aes complete
	do
	{
		if(iReadByte(addr,&data)!=TRUE)
		{
			return FALSE;
		}
	}while((data&mask) != ref);

	return TRUE;
}

unsigned char sw2001_rand(void)
{
  static unsigned long seed; // 2byte, must be a static variable
  unsigned long randNum;

  get_random_bytes(&randNum, sizeof(unsigned long));
  //printk("We get random number: %ld\n", randNum);
	
  seed = seed + randNum ;//get_random_int(); // rand(); <------------------ add time value
  seed =  seed * 1103515245 + 12345;

  return (seed/65536) % 32768;
}

static int hasReadChipID = 0;
uint8_t neogeoChipID[7] = {0xFF};
EXPORT_SYMBOL(neogeoChipID);

static int sw2001_process(void)
{
	uint8_t pt[16] ={0x32,0x43,0xf6,0xa8,0x88,0x5a,0x30,0x8d,0x31,0x31,0x98,0xa2,0xe0,0x37,0x07,0x34};
	uint8_t ct[16]; // = {0x39,0x25,0x84,0x1d,0x02,0xdc,0x09,0xfb,0xdc,0x11,0x85,0x97,0x19,0x6a,0x0b,0x32};
	uint8_t key[16] = {0x58,0x55,0x47,0x41,0x4d,0x45,0x50,0x4c,0x41,0x59,0x59,0x5a,0x4a,0x4a,0x43,0x58}; //user define according to sw2001 ic
	uint8_t result[16];
	uint8_t parity=0;
	int32 i;
	
	//init I2C, used to acess sw2001
	iDeviceInit(SW2001_DEVICE_ADDR, 0);   //sw2001 default device address=0x3C, speed support 100k/400k

	//generate random plait text
	for(i=0;i<16;i++)
	{
		pt[i] = sw2001_rand();
		//printk("---pt[%d] = %d \n",i,pt[i]);
	}

	//compute cipher text
	sw2001_aes_compute(key, pt, ct);
	
	//compute odd check bits
	sw2001_aes_parity(pt, &parity);

	//write cipher text to sw2001
	iWriteData(SW2001_REG_CIPHER_TEXT_ADDR, ct, 16);
	
	//set parity
	iWriteByte(SW2001_REG_DECRYPT_CTRL,parity);

	//start sw2001 aes decrypt
	iSetBits(SW2001_REG_DECRYPT_CTRL, SW2001_AES_ENABLE);

	//wait for sw2001 decrypt complete
	iCheckBits(SW2001_REG_DECRYPT_CTRL, SW2001_AES_STATUS, 0x0);

	//read decrypt result back
	iReadData(SW2001_REG_PLAINT_TEXT_ADDR, result, 16);

	//read chipID
	if(hasReadChipID == 0)
	{
		iReadData(SW2001_REG_CHIP_ID_ADDR, neogeoChipID, 7);
		hasReadChipID == 1;
	}
	PR_DEBUG("============================================================="); PR_DEBUG("\r\n");
	PR_DEBUG(" pt Data : "); for (i=0; i<16; i++) PR_DEBUG("0x%02x ", pt[i]); PR_DEBUG("\r\n");
	PR_DEBUG(" result Data : "); for (i=0; i<16; i++) PR_DEBUG("0x%02x ", result[i]); PR_DEBUG("\r\n");
	PR_DEBUG(" ct Data : "); for (i=0; i<16; i++) PR_DEBUG("0x%02x ", ct[i]); PR_DEBUG("\r\n");
//	PR_DEBUG(" chip ID : "); for (i=0; i<7; i++) PR_DEBUG("0x%02x ", neogeoChipID[i]); PR_DEBUG("\r\n");
	PR_DEBUG("============================================================="); PR_DEBUG("\r\n");
	PR_DEBUG("\r\n");
		
	//compare result
	for(i=0;i<16;i++)
	{
		if(pt[i] != result[i])
		{
			PR_DEBUG("verify failed!\n");
			return 1;
		}
	}
	PR_DEBUG("verify success!\n");
	
	//clear flag if need
	//iSetBits(SW2001_REG_FLAG_CTRL, SW2001_LOCK_CLEAR);
	
	//deinit I2C, release resource
	iDeviceDeInit();
	
	return 0;
}

/****************************** module control *******************************************/

int isvalid(void)
{
	unsigned char result;
	int j;
	
	//initial..
	for(j=0; j<10; j++)	{
		
		result = sw2001_process();

		if ( result ) PR_DEBUG(" SW2001 Test Fail!!");
		else PR_DEBUG(" SW2001 Test Success!!!");
		PR_DEBUG("\r\n");
		
		if (result==0) return 1; //valid.
	}
	
	//not valid.
	return 0;
}

/*-------------------------------------------------------------------------*/
static int __exit sw2001_remove(struct i2c_client *client)
{
	struct sw2001		*axp = (struct sw2001 *)i2c_get_clientdata(client);

	PR_DEBUG("%s\n",__func__);

	kfree(axp);
	i2c_set_clientdata(client, NULL);
	the_sw2001 = NULL;

	return 0;
}

void encryption_verify(void)
{
    if(!isvalid())  //illegal. power off.
    {
            //PR_DEBUG("%s: will shutdown.\n",__func__);
            printk("Invalid product.\n");
            msleep(500);
            
            //if(pm_power_off)
            //   pm_power_off();
			while(1);
    }
    else
		printk("%s: valid.\n",__func__);
}
EXPORT_SYMBOL(encryption_verify);

static int sw2001_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	int err = 0;
	struct sw2001		*pEnc;
	//struct device   	*dev=&client->dev;

	PR_DEBUG("%s\n",__func__);
	
	//print_client(client);
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EINVAL;

	pEnc = kzalloc(sizeof(struct sw2001), GFP_KERNEL); 
	if (!pEnc)
		return -ENOMEM;

	mutex_init(&pEnc->lock);
	pEnc->client = client;
	i2c_set_clientdata(client,pEnc);
	
	the_sw2001 = pEnc;

	encryption_verify();

	return err;
	
//fail1:
//	kfree(pEnc);
//	return status;
}

/*-------------------------------------------------------------------------*/
#define I2C_STATIC_BUS_NUM      (1)     //define which I2C bus to connect             
static struct i2c_board_info sw2001_i2c_boardinfo = {
    I2C_BOARD_INFO(DEVICE_NAME, SW2001_DEVICE_ADDR),
};
int i2c_static_add_device(struct i2c_board_info *info)
{
    struct i2c_adapter *adapter;
    struct i2c_client  *client;
    int    ret;

    adapter = i2c_get_adapter(I2C_STATIC_BUS_NUM);
    if (!adapter) {
        PR_INFO("%s: can't get i2c adapter\n", __func__);
        ret = -ENODEV;
        goto i2c_err;
    }

    client = i2c_new_device(adapter, info);
    if (!client) {
        PR_INFO("%s:  can't add i2c device at 0x%x\n",
                __FUNCTION__, (unsigned int)info->addr);
        ret = -ENODEV;
        goto i2c_err;
    }

    i2c_put_adapter(adapter);

    return 0;

i2c_err:
    return ret;
}

static const struct i2c_device_id sw2001_id[] = {
	{ DEVICE_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sw2001_id);

static const unsigned short normal_i2c[2] = {0x3C,I2C_CLIENT_END};

static struct i2c_driver sw2001_driver = {
	.driver = {
		.name	= DEVICE_NAME,
	},
	.probe	= sw2001_probe,
	.remove	= __exit_p(sw2001_remove),
	.id_table = sw2001_id,
	.address_list	= normal_i2c,
};


#if 1 //allen add
uint8_t srand[16];
uint8_t rand_flag=0;
static ssize_t reg_show(struct class *class, struct class_attribute *attr,	char *buf)
{
	int i,ret;
	uint8_t pt[16] ={0x32,0x43,0xf6,0xa8,0x88,0x5a,0x30,0x8d,0x31,0x31,0x98,0xa2,0xe0,0x37,0x07,0x34};
	uint8_t ct[16];
	
	ret = sw2001_process();
	if(ret)
	{
		printk("xxxx errr\n");
		while(1);
	}
	
	if(0 == rand_flag)
	{
		for(i=0;i<16;i++)
		{
			srand[i] = sw2001_rand();
			//printk("---pt[%d] = %d \n",i,pt[i]);
		}
	}
	rand_flag = 0;

	//compute cipher text
	sw2001_aes_compute(pt, srand, ct);
	
	//for(i=0;i<16;i++)
	//	printk("kernel ct: %d: %d \n",i,ct[i]);

	return snprintf(buf,PAGE_SIZE,"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		            ct[0],ct[1],ct[2],ct[3],ct[4],ct[5],ct[6],ct[7],
		            ct[8],ct[9],ct[10],ct[11],ct[12],ct[13],ct[14],ct[15]);
}

static ssize_t reg_store(struct class *class, struct class_attribute *attr,	const char *buf, size_t count)
{
	//int tmp;
	
	//tmp = simple_strtoul(buf, NULL, 10);
	//printk("--%s val = %d \n",__func__,tmp);
	//int i;
	//printk("kerenl: %s \n",buf);
	
	sscanf(buf,"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
	       &srand[0],&srand[1],&srand[2],&srand[3],&srand[4],&srand[5],&srand[6],&srand[7],
	       &srand[8],&srand[9],&srand[10],&srand[11],&srand[12],&srand[13],&srand[14],&srand[15]);

	//for(i=0;i<16;i++)
	//	printk("kernel rand: %d: %d \n",i,srand[i]);
		   
	rand_flag = 1;
	
	return count;
}

/***********************
/sys/class/umido/reg
************************/
static struct class_attribute umido_class_attrs[] = {
	__ATTR(reg,S_IRUGO|S_IWUSR,reg_show,reg_store),
	__ATTR_NULL
};

static struct class umido_class = {
    .name = "umido",
    .class_attrs = umido_class_attrs,
};
#endif

static int __init sw2001_init(void)
{
	int	status = -ENODEV;
#ifdef ENCRYPT_DEBUG	
	printk("%s: version %s\n", DRIVER_NAME, DRIVER_VERSION);
#endif	

	class_register(&umido_class);

    status = i2c_static_add_device(&sw2001_i2c_boardinfo);
    if (status < 0)
    {
        printk("%s: add i2c device error %d\n", __func__, status);
        return (status);
    }
	status = i2c_add_driver(&sw2001_driver);
	
	return status;
}

//subsys_initcall(sw2001_init);
module_init(sw2001_init);

static void __exit sw2001_exit(void)
{
	PR_INFO("%s\n",__func__);
	if(the_sw2001)
		i2c_unregister_device(the_sw2001->client);
	i2c_del_driver(&sw2001_driver);
}
module_exit(sw2001_exit);

