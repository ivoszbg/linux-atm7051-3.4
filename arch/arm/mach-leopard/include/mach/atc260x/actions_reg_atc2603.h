#ifndef __TEST_H__


//atc2603_readll(int a);
enum reg 
{

	atc2603_PMU_SYS_CTL0 = 0,
	atc2603_PMU_SYS_CTL1,
	atc2603_PMU_SYS_CTL2,
        atc2603_PMU_SYS_CTL3,
        atc2603_PMU_SYS_CTL4,
        atc2603_PMU_SYS_CTL5,
        atc2603_PMU_SYS_CTL6,
	atc2603_PMU_SYS_CTL7,
        atc2603_PMU_SYS_CTL8,
        atc2603_PMU_SYS_CTL9,//9
	atc2603_PMU_BAT_CTL0,
	atc2603_PMU_BAT_CTL1,
	atc2603_PMU_VBUS_CTL0,
	atc2603_PMU_VBUS_CTL1,
        atc2603_PMU_WALL_CTL0,
        atc2603_PMU_WALL_CTL1,
        atc2603_PMU_SYS_Pending,
        atc2603_PMU_DC1_CTL0,
        atc2603_PMU_DC1_CTL1,
        atc2603_PMU_DC1_CTL2,//19
        atc2603_PMU_DC2_CTL0,
        atc2603_PMU_DC2_CTL1,
        atc2603_PMU_DC2_CTL2,
        atc2603_PMU_DC3_CTL0,
        atc2603_PMU_DC3_CTL1,
        atc2603_PMU_DC3_CTL2,
        atc2603_PMU_DC4_CTL0,
        atc2603_PMU_DC4_CTL1,
        atc2603_PMU_DC5_CTL0,
        atc2603_PMU_DC5_CTL1,//29
        atc2603_PMU_LDO1_CTL,
        atc2603_PMU_LDO2_CTL,
        atc2603_PMU_LDO3_CTL,
        atc2603_PMU_LDO4_CTL,
        atc2603_PMU_LDO5_CTL,
        atc2603_PMU_LDO6_CTL,
        atc2603_PMU_LDO7_CTL,
        atc2603_PMU_LDO8_CTL,
        atc2603_PMU_LDO9_CTL,
        atc2603_PMU_LDO10_CTL,//39
        atc2603_PMU_LDO11_CTL,
        atc2603_PMU_SWITCH_CTL,
        atc2603_PMU_OV_CTL0,
        atc2603_PMU_OV_CTL1,
        atc2603_PMU_OV_Status,
        atc2603_PMU_OV_EN,
        atc2603_PMU_OV_INT_EN,
        atc2603_PMU_OC_CTL,
        atc2603_PMU_OC_Status,
        atc2603_PMU_OC_EN,//49
        atc2603_PMU_OC_INT_EN,
        atc2603_PMU_UV_CTL0,
        atc2603_PMU_UV_CTL1,
        atc2603_PMU_UV_Status,
        atc2603_PMU_UV_EN,
        atc2603_PMU_UV_INT_EN,
        atc2603_PMU_OT_CTL,
        atc2603_PMU_CHARGER_CTL0,
        atc2603_PMU_CHARGER_CTL1,
        atc2603_PMU_CHARGER_CTL2,//59
        atc2603_PMU_BakCHARGER_CTL,
        atc2603_PMU_APDS_CTL,
        atc2603_PMU_AuxADC_CTL0,
        atc2603_PMU_AuxADC_CTL1,
        atc2603_PMU_BATVADC,
        atc2603_PMU_BATIADC,
        atc2603_PMU_WALLVADC,
        atc2603_PMU_WALLIADC,
        atc2603_PMU_VBUSVADC,
        atc2603_PMU_VBUSIADC,//69
        atc2603_PMU_SYSPWRADC,
        atc2603_PMU_REMCONADC,
        atc2603_PMU_RemConADC,
        atc2603_PMU_SVCCADC,
        atc2603_PMU_CHGIADC,
        atc2603_PMU_IREFADC,
        atc2603_PMU_BAKBATADC,
        atc2603_PMU_ICTEMPADC,
        atc2603_PMU_AuxADC0,
        atc2603_PMU_AuxADC1,//79
        atc2603_PMU_AuxADC2,
        atc2603_PMU_AuxADC3,
        atc2603_PMU_ICMADC,
        atc2603_PMU_BDG_CTL,
        atc2603_RTC_CTL,
        atc2603_RTC_MSALM,
        atc2603_RTC_HALM,
        atc2603_RTC_YMDALM,
        atc2603_RTC_MS,
        atc2603_RTC_H,//89
        atc2603_RTC_DC,       
 	atc2603_RTC_YMD,
 	atc2603_EFUSE_DAT,
 	atc2603_EFUSECRTL1,
 	atc2603_EFUSECRTL2,
 	atc2603_PMU_FW_USE0,
 	atc2603_PMU_FW_USE1,
 	atc2603_PMU_FW_USE2,
 	atc2603_PMU_FW_USE3,
 	atc2603_IRC_CTL,//99
 	atc2603_IRC_STAT,
 	atc2603_PMU_FW_USE4,
 	atc2603_PMU_Abnormal_Status,
 	atc2603_IRC_CC,
 	atc2603_IRC_KDC,
	atc2603_PMU_WALL_APDS_CTL,
 	atc2603_IRC_WK,
 	atc2603_PMU_REMCON_CTL0,
 	atc2603_PMU_REMCON_CTL1,
 	atc2603_PMU_MUX_CTL0,//109
 	atc2603_PMU_SGPIO_CTL0,
 	atc2603_PMU_SGPIO_CTL1,
 	atc2603_PMU_SGPIO_CTL2,
 	atc2603_PMU_SGPIO_CTL3,
 	atc2603_PMU_SGPIO_CTL4,
 	atc2603_PWMCLK_CLK,
 	atc2603_PWM0_CTL,
 	atc2603_PWM1_CTL,
 	atc2603_PMU_ADC_DBG0,
 	atc2603_PMU_ADC_DBG1,//119
 	atc2603_PMU_ADC_DBG2,
 	atc2603_PMU_ADC_DBG3,
 	atc2603_PMU_ADC_DBG4,
 	atc2603_IRC_RCC,
 	atc2603_IRC_FILTER,
 	atc2603_AUDIOINOUT_CTL,
 	atc2603_AUDIO_DEBUGOUTCTL,
	atc2603_DAC_FILTERCTL0,
	atc2603_DAC_FILTERCTL1,
 	atc2603_DAC_DIGITALCTL,//129
 	atc2603_DAC_VOLUMECTL0,
	atc2603_DAC_VOLUMECTL1,
	atc2603_DAC_VOLUMECTL2,
	atc2603_DAC_VOLUMECTL3,
 	atc2603_DAC_ANALOG0,
 	atc2603_DAC_ANALOG1,
 	atc2603_DAC_ANALOG2,
 	atc2603_DAC_ANALOG3,
	atc2603_DAC_ANALOG4,
 	atc2603_ADC_DIGITALCTL,//139
	atc2603_ADC_CTL,
 	atc2603_ADC_HPFCTL,
	atc2603_ADC0_DIGITALCTL,
	atc2603_ADC0_HPFCTL,
	atc2603_ADC0_CTL,
	atc2603_ADC1_DIGITALCTL,
	atc2603_ADC1_CTL,
	atc2603_AGC1_CTL0,
	atc2603_AGC1_CTL1,
	atc2603_AGC1_CTL2,//149
 	atc2603_AGC_CTL0,
 	atc2603_AGC_CTL1,
 	atc2603_AGC_CTL2,
	atc2603_AGC0_CTL0,
	atc2603_AGC0_CTL1,
	atc2603_AGC0_CTL2,
	atc2603_ADC_ANANLOG0,
	atc2603_ADC_ANANLOG1,
 	atc2603_ADC_ANALOG0,
 	atc2603_ADC_ANALOG1,//159
 	atc2603_PCM0_CTL,
 	atc2603_PCM1_CTL,
 	atc2603_PCM2_CTL,
 	atc2603_PCMIF_CTL,
	atc2603_CMU_HOSCCTL,
 	atc2603_CMU_DEVRST,
 	atc2603_INTS_PD,
 	atc2603_INTS_MSK,
	atc2603_MFP_CTL,
 	atc2603_PAD_VSEL,//169
	atc2603_GPIO_OUTEN,
 	atc2603_GPIO_INEN,
	atc2603_GPIO_DAT,
	atc2603_PAD_DRV,
	atc2603_PAD_EN,
	atc2603_DEBUG_SEL,
	atc2603_DEBUG_IE,
	atc2603_DEBUG_OE,
	atc2603_BIST_START,
	atc2603_BIST_RESULT,//179
	atc2603_CHIP_VER, 
	atc2603_CLASSD_CTL0,
	atc2603_CLASSD_CTL1,
	atc2603_CLASSD_CTL2,
	atc2603_MFP_CTL0,
	atc2603_MFP_CTL1,
	atc2603_GPIO_OUTEN0,
	atc2603_GPIO_OUTEN1,
	atc2603_GPIO_INEN0,
	atc2603_GPIO_INEN1,//189
	atc2603_GPIO_DAT0,
	atc2603_GPIO_DAT1,
	atc2603_GPIO_DRV0,
	atc2603_GPIO_DRV1,
	atc2603_TEST_MODE_CFG,
	atc2603_TEST_CFG_EN,
	atc2603_AUDIO_MEM_BIST_CTL,
	atc2603_AUDIO_MEM_BIST_RESULT,
	atc2603_CVER,//198
	atc2603_SLAVE_ADDRESS_REG,
};

#if 0
unsigned short  atc2603a[200] = {0x0000,0x0001,0x0002,0x0003,0x0004,0x0005,0x0006,0x0007,0x0008,0x0009,
								 0x000a,0x000b,0x000c,0x000d,0x000e,0x000f,0x0010,0x0011,0x0012,0x0013,
								 0x0014,0x0015,0x0016,0x0017,0x0018,0x0019,0x001a,0x001b,0x001c,0x001d,
								 0x001e,0x001f,0x0020,0x0021,0x0022,0x0023,0x0024,0x0025,0x0026,0x0027,
								 0x0028,0x0029,0x002a,0x002b,0x002c,0x002d,0x002e,0x002f,0x0030,0x0031,
								 0x0032,0x0033,0x0034,0x0035,0x0036,0x0037,0x0038,0x0039,0x003a,0x003b,
								 0x003c,0x003d,0x003e,0x003f,0x0040,0x0041,0x0042,0x0043,0x0044,0x0045,
								 0x0046,0xffff,0x0047,0x0048,0x0049,0x004a,0x004b,0x004c,0x004d,0x004e,
                                 0x004f,0x0050,0xffff,0x0051,0x0052,0x0053,0x0054,0x0055,0x0056,0x0057,
								 0x0058,0x0059,0x005a,0x005b,0x005c,0xffff,0xffff,0xffff,0xffff,0x0060,
								 0x0061,0xffff,0xffff,0x0062,0x0063,0xffff,0x0064,0xffff,0xffff,0xffff,
								 0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,
								 0xffff,0xffff,0xffff,0xffff,0xffff,0x0400,0x0401,0x0402,0x0403,0x0404,//13
								 0x0405,0x0406,0x0407,0x0408,0x0409,0x040a,0x040b,0x040c,0x040d,0xffff,
								 0xffff,0xffff,0x0411,0x0412,0x0413,0x0419,0x041a,0x041b,0x041c,0x041d,
                                 0xffff,0xffff,0xffff,0x0414,0x0415,0x0416,0x0417,0x0418,0xffff,0xffff,//16
			                     0xffff,0xffff,0xffff,0xffff,0x0100,0x0101,0x0200,0x0201,0xffff,0xffff,
			                     0xffff,0xffff,0xffff,0xffff,0x0322,0x0330,0x0331,0x0332,0xffff,0xffff,
                                 0xffff,0x040e,0x040f,0x0410,0x0300,0x0301,0x0310,0x0311,0x0312,0x0313,
			                     0x0314,0x0315,0x0320,0x0321,0x0700,0x0701,0x0702,0x0703,0x0704,0XFFFF
			       };
unsigned char atc2603c[200] ={ 0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,
							   0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,0x10,0x11,0x12,0x13,
                               0x14,0x15,0x16,0x17,0x18,0x19,0x1a,0x1b,0x1c,0x1d,
							   0x1e,0x1f,0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,
                               0x28,0x29,0x2a,0x2b,0x2c,0x2d,0x2e,0x2f,0x30,0x31,
                               0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x3a,0x3b,
                               0x3c,0x3d,0x3e,0x3f,0x40,0x41,0x42,0x43,0x44,0x45,
							   0x46,0x47,0xff,0x48,0x49,0x4a,0x4b,0x4c,0x4d,0x4e,
			                   0x4f,0xff,0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57,
                               0x58,0x59,0x5a,0x5b,0x5c,0x5d,0x5e,0x5f,0x60,0x80,
                               0x81,0x61,0x62,0x82,0x83,0x63,0x84,0x64,0x65,0x66,
		                       0x67,0x68,0x69,0x6a,0x6b,0x6c,0x6d,0x6e,0x70,0x71,
			                   0x72,0x73,0x74,0x85,0x86,0xa0,0xa1,0xff,0xff,0xa2,//13
			                   0xa3,0xff,0xff,0xff,0xa4,0xa5,0xa6,0xa7,0xff,0xa8,
                               0xaa,0xa9,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
                               0xab,0xac,0xad,0xff,0xff,0xff,0xff,0xff,0xae,0xaf,//16
                               0xb0,0xb1,0xb2,0xb3,0xff,0xc1,0xc8,0xc9,0xd0,0xd1,
                               0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,0xda,0xdb,
			                   0xdc,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
			                   0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0XF8
			     };

 #endif //0
#endif