//*****************************************************************************
//	File name  : 				sw2001_aes.h
//
//	Description:  This file provides all functions prototypes of aes
//	
//  written by :  jason, iSmartware Technology Co.,LTD
//	History    :
//                 2015/04/13      jason       v0.1    Initial version
//                 2015/06/06      jason       v1.0    added flag control register
//******************************************************************************

#ifndef __SW2001_AES_H__
#define __SW2001_AES_H__
#include <linux/string.h>

typedef unsigned long       DWORD;
typedef int                 BOOL;
typedef unsigned char       BYTE;
typedef unsigned short      WORD;
typedef float               FLOAT;

typedef int                 INT;
typedef unsigned int        UINT;
typedef unsigned int        *PUINT;

typedef signed char         __s8;
typedef unsigned char       __u8;
typedef signed short        __s16;
typedef unsigned short      __u16;
typedef signed int          __s32;
typedef unsigned int        __u32;
//typedef signed long long    __s64;
//typedef unsigned long long  __u64;

typedef signed char         s8;
typedef unsigned char       u8;
typedef signed short        s16;
typedef unsigned short      u16;
typedef signed int          s32;
typedef unsigned int        u32;
//typedef signed long long    s64;
//typedef unsigned long long  u64;

typedef signed char         int8;
typedef unsigned char       uint8;
typedef signed short        int16;
typedef unsigned short      uint16;
typedef signed int          int32;
typedef unsigned int        uint32;

typedef signed char         int8_t;
typedef unsigned char       uint8_t;
typedef signed short        int16_t;
typedef unsigned short      uint16_t;
typedef signed int          int32_t;
typedef unsigned int        uint32_t;

typedef unsigned char __bool;

BOOL iDeviceInit(uint8_t device_addr, uint8_t speed);
BOOL iDeviceDeInit(void);
BOOL iWriteByte(uint8_t addr, uint8_t data);
BOOL iReadByte(uint8_t addr, uint8_t *data);
BOOL iSleep(uint8_t waittime);
BOOL iWriteData(uint8_t addr, uint8_t *data, uint8_t len);
BOOL iReadData(uint8_t addr, uint8_t *data, uint8_t len);
BOOL iSetBits(uint8_t addr, uint8_t bit);
BOOL iClearBits(uint8_t addr, uint8_t bit);
BOOL iCheckBits(uint8_t addr, uint8_t mask, uint8_t ref);

#define TRUE (0)
#define FALSE (-1)

//i2c device address
#define SW2001_DEVICE_ADDR 0x3C

//sw2001 cipher text start address
#define SW2001_REG_CIPHER_TEXT_ADDR 0x10

//sw2001 plaint text start address
#define SW2001_REG_PLAINT_TEXT_ADDR 0x10

#define SW2001_REG_CHIP_ID_ADDR 0xA1

//sw2001 aes control addr
#define SW2001_REG_DECRYPT_CTRL 0x00

#define SW2001_AES_ENABLE (0x01)

#define SW2001_AES_STATUS (0x20) 

//sw2001 lock 
#define SW2001_REG_FLAG_CTRL 0x01

#define SW2001_LOCK_CLEAR (0x40)

uint8_t sw2001_aes_parity(uint8_t *text, uint8_t *parity);
uint8_t sw2001_aes_compute(uint8_t *key, uint8_t *planit, uint8_t *cipher);

#endif
