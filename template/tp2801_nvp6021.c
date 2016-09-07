/* 
 *
 * Copyright (c) 2006 Hisilicon Co., Ltd. 
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307  USA
 *
 * 
 * History: 
 *      10-April-2006 create this file
 */


#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/mm.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/slab.h>
//#include <linux/smp_lock.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/system.h>

#include <linux/miscdevice.h>

#include <linux/delay.h>

#include <linux/proc_fs.h>
#include <linux/poll.h>

#include <mach/hardware.h>
#include <asm/bitops.h>
#include <asm/uaccess.h>
#include <asm/irq.h>

#include <linux/moduleparam.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/reboot.h>
#include <linux/notifier.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <linux/kfifo.h>


#include "tp2801.h"
#include "gpio_i2c.h"

#define GPIO_I2C
//=============================================================
#define GPIO13_IRQ_NR	(83)

#define GPIO12_IRQ_NR	(84)

#define COC_BUF_SIZE (32)
#define COC_BUF 1
static int det = DET_TVI;
static int tp2801_addr=0x88;
unsigned static  reset_num = 1;
static struct timer_list  isr_timer;
int gvideo_type=VIDEO_1080P25;
int coc_func (void);
struct timeval tv;

DECLARE_TASKLET(coc_task, coc_func, 0);
static DECLARE_KFIFO(coc_fifo,  COC_DATA_TYPE, COC_BUF_SIZE);

#define HW_REG(reg)             *((volatile unsigned int *)(reg))
#define MUXCTL_REGBASE      (0x200F0000)

#define MULCTL_REG106       IO_ADDRESS(MUXCTL_REGBASE + 0x1a8) //VI_VS GPIO15_2
#define MULCTL_REG110       IO_ADDRESS(MUXCTL_REGBASE + 0x1b8) //VI_DAT12 GPIO12_3
#define MULCTL_REG111       IO_ADDRESS(MUXCTL_REGBASE + 0x1bc) //VI_DAT12 GPIO12_4
#define MULCTL_REG115       IO_ADDRESS(MUXCTL_REGBASE + 0x1cc) //VI_DAT7 GPIO13_0
#define MULCTL_REG116       IO_ADDRESS(MUXCTL_REGBASE + 0x1D0) //VI_DAT6 GPIO13_1

#define MULCTL_REG4   IO_ADDRESS(MUXCTL_REGBASE + 0x010) //VO_DAT15
#define MULCTL_VO_REG(X) IO_ADDRESS(MUXCTL_REGBASE + 0x010+ 4*(X)) //VO_DATA[0...15]

#define GPIO12_BASE_ADDR     (0x20200000)  
#define GPIO12_DIR_REG       IO_ADDRESS(GPIO12_BASE_ADDR + 0x0400)
#define GPIO12_DATA_REG      IO_ADDRESS(GPIO12_BASE_ADDR + 0x03fc)
#define GPIO12_IS            IO_ADDRESS(GPIO12_BASE_ADDR + 0x0404)
#define GPIO12_IBE           IO_ADDRESS(GPIO12_BASE_ADDR + 0x0408)
#define GPIO12_IEV       	 IO_ADDRESS(GPIO12_BASE_ADDR + 0x040C)
#define GPIO12_IE      		 IO_ADDRESS(GPIO12_BASE_ADDR + 0x0410)
#define GPIO12_RIS       	 IO_ADDRESS(GPIO12_BASE_ADDR + 0x0414)
#define GPIO12_MIS      	 IO_ADDRESS(GPIO12_BASE_ADDR + 0x0418)
#define GPIO12_IC      		 IO_ADDRESS(GPIO12_BASE_ADDR + 0x041C)

#define GPIO13_BASE_ADDR     (0x20210000)  
#define GPIO13_DIR_REG       IO_ADDRESS(GPIO13_BASE_ADDR + 0x0400)
#define GPIO13_DATA_REG      IO_ADDRESS(GPIO13_BASE_ADDR + 0x03fc)
#define GPIO13_IS            IO_ADDRESS(GPIO13_BASE_ADDR + 0x0404)
#define GPIO13_IBE           IO_ADDRESS(GPIO13_BASE_ADDR + 0x0408)
#define GPIO13_IEV       	 IO_ADDRESS(GPIO13_BASE_ADDR + 0x040C)
#define GPIO13_IE      		 IO_ADDRESS(GPIO13_BASE_ADDR + 0x0410)
#define GPIO13_RIS       	 IO_ADDRESS(GPIO13_BASE_ADDR + 0x0414)
#define GPIO13_MIS      	 IO_ADDRESS(GPIO13_BASE_ADDR + 0x0418)
#define GPIO13_IC      		 IO_ADDRESS(GPIO13_BASE_ADDR + 0x041C)


#define GPIO15_BASE_ADDR     (0x20260000) 
#define GPIO15_DIR_REG       IO_ADDRESS(GPIO15_BASE_ADDR + 0x0400)
#define GPIO15_DATA_REG      IO_ADDRESS(GPIO15_BASE_ADDR + 0x03fc)

#define MULCTL_VOCLK          IO_ADDRESS(MUXCTL_REGBASE + 0x00C) 
#define CLK_CTL         IO_ADDRESS(0X20030000 + 0x0034) 
#define CHIP_NUM 1
#define DEV_NAME "tp2801_nvp6021"
#define DEBUG_LEVEL 1
#define DPRINTK(level,fmt,args...) do{ if(level < DEBUG_LEVEL)\
    printk(KERN_INFO "%s [%s ,%d]: " fmt "\n",DEV_NAME,__FUNCTION__,__LINE__,##args);\
}while(0)


COC_DEV coc_device;

static int gDet_tvi_ahd = DET_TVI; //1:tvi,  0 :ahd


static struct i2c_board_info tp2801_info = {
     I2C_BOARD_INFO("tp2801", TP2801_ADDR),
};
static struct i2c_board_info tp28012_info = {
     I2C_BOARD_INFO("tp2801", TP28012_ADDR),
};


static struct i2c_board_info nvp6021_info = {
     I2C_BOARD_INFO("nvp6021", NVP6021_ADDR),
};

static struct i2c_client *tp2801_client=NULL;
static struct i2c_client *nvp6021_client=NULL;

static unsigned int  open_cnt = 0;	
static int chip_count = 1;

int init_reg_hw(void);

static int i2c_client_init(void);

// 操作成功返回1，否则返回0;
unsigned char RegisterWrite(unsigned char RegisterAddr, unsigned char Data)
{
	int ret = 0;
	ret = tp2801_write(RegisterAddr, Data);
	
	return ret;
}

// 操作成功返回1，否则返回0;
unsigned char RegisterRead(unsigned char RegisterAddr, unsigned char *pData)
{
	int ret = 1;
	*pData =tp2801_read(RegisterAddr);

	if (*pData == 0xff)
		ret = 0;
	return ret;
}


static void tp2801_init_video(const TP2801_PARA_TYPE *pData, unsigned char BufLen)
{
	unsigned char i, Cnt;

    for (i= 0; i<BufLen; i++)
    {
    	for (Cnt=0; Cnt<3; Cnt++)
    	{
    		if (RegisterWrite(pData->RegAddr, pData->Data))
    		{
    			break;
    		}
    		mdelay(200);
    	}
    	pData++;
        udelay(20);
    }
}

static void tp2801_init_core(void)
{
    unsigned char i, Cnt;
	unsigned char value;
    const TP2801_PARA_TYPE *pData = TP2801_Comm;

     for (i= 0; i<33; i++)
    {
    	for (Cnt=0; Cnt<3; Cnt++)
    	{
    		if (RegisterWrite(pData->RegAddr, pData->Data))
    		{
    			break;
    		}
    		udelay(20);
    	}
    	pData++;
        udelay(20);
    }


	//RegisterRead(0x43, &value);
	//value |= 0x40;
	//RegisterWrite(0x43, value);
	//hold_cpu_delay_ms(100);
	//value &= 0xBF;
	//RegisterWrite(0x43, value);
	//hold_cpu_delay_ms(100);
	//RegisterWrite(0x06, 0x80);	// soft reset


	RegisterRead(0x43, &value);
	value &= 0xBF;
	RegisterWrite(0x43, value);
	mdelay(100);
	value |= 0x40;
	RegisterWrite(0x43, value);
	mdelay(100);
	RegisterWrite(0x06, 0x80);	// soft reset
}

void TviVideoInit(unsigned char Mode)
{
    switch(Mode)
    {
    case VIDEO_720P25:
        tp2801_init_video(TP2801_720P25, 10);
        break;
    case VIDEO_720P30:
        tp2801_init_video(TP2801_720P30, 10);
        break;
    case VIDEO_720P50:
        tp2801_init_video(TP2801_720P50, 10);
        break;
    case VIDEO_720P60:
        tp2801_init_video(TP2801_720P60, 10);
        break;
    case VIDEO_1080P25:
        tp2801_init_video(TP2801_1080P25, 10);
        break;
    case VIDEO_1080P30:
        tp2801_init_video(TP2801_1080P30, 10);
        break;
    default:
        break;
    }
 
    tp2801_init_core();
	
}

void TviCoaxSend(unsigned char *pBuf)
{
	RegisterWrite(0x4C, 0x80);
	udelay(20);
	RegisterWrite(0x4D, pBuf[4]);
	udelay(20);
	RegisterWrite(0x4E, pBuf[5]);
	udelay(20);
	RegisterWrite(0x4F, pBuf[6]);
	udelay(20);
	RegisterWrite(0x50, pBuf[7]);
	udelay(20);

	RegisterWrite(0x51, 0x80);
	udelay(20);
	RegisterWrite(0x52, pBuf[0]);
	udelay(20);
	RegisterWrite(0x53, pBuf[1]);
	udelay(20);
	RegisterWrite(0x54, pBuf[2]);
	udelay(20);
	RegisterWrite(0x55, pBuf[3]);
	udelay(20);
}

#define TVI_CMD_LENGTH		5


//==================================================================
// 操作成功返回1，否则返回0;
unsigned char nvp6021_write(unsigned char reg_addr, unsigned char value)
{
	int ret;
#ifdef GPIO_I2C
	ret = gpio_i2c_write(NVP6021_ADDR,reg_addr,value);
#else
	unsigned char buf[2];
	struct i2c_client *client = nvp6021_client;
	
	buf[0] = reg_addr;
	buf[1] = value;
	
	ret = i2c_master_send(client, buf, 2);
#endif
	return ret;
}
int nvp6021_read(unsigned char reg_addr)
{
    int ret_data = 0xFF;
	int ret;
#ifdef GPIO_I2C
	ret_data = gpio_i2c_read(NVP6021_ADDR,reg_addr);

#else
	struct i2c_client *client = nvp6021_client;
    unsigned char buf[2];

    buf[0] = reg_addr;
	ret = i2c_master_recv(client, buf, 1);
    if (ret >= 0)
    {
        ret_data = buf[0];
    }
#endif
	return ret_data;
}
void Init_TX_coax(void)
{
	nvp6021_write(0xFF,BANK2);
	nvp6021_write(0x63,0xFF);	// TX out-put Threshold
	nvp6021_write(0xFF,BANK3);

	nvp6021_write(0x20,0x2F);	// A-CP TX BOUD
	nvp6021_write(0x23,0x08);	// Line position1
	nvp6021_write(0x24,0x00);	// Line position2
	nvp6021_write(0x25,0x07);	// Lines count
	nvp6021_write(0x2B,0x10);	// A-CP mode choose
	//nvp6021_write(0x2D,0x00);	// Start point1
	nvp6021_write(0x2D,0x0D);	// Start point1
	nvp6021_write(0x2E,0x01);	// Start point2
	nvp6021_write(0xA9,0x00);	// i2c master mode off
		
	nvp6021_write(0x30,0x55);
	nvp6021_write(0x31,0x24);
	nvp6021_write(0x32,0x00);
	nvp6021_write(0x33,0x00);
	nvp6021_write(0x34,0x00);
	nvp6021_write(0x35,0x00);
	nvp6021_write(0x36,0x00);
	nvp6021_write(0x37,0x00);
	nvp6021_write(0x29,0x08);
}

void Init_RX_coax(void)
{
	nvp6021_write(0xFF, BANK0);
	nvp6021_write(0x17, 0x00); 	// interrupt type
	nvp6021_write(0xFF, BANK3);

	nvp6021_write(0x09, 0x00);
	nvp6021_write(0x80, 0x00);	// RX ID
	nvp6021_write(0x82, 0x10);
	nvp6021_write(0x83, 0x01);
	nvp6021_write(0x86, 0x80);
	nvp6021_write(0x87, 0x01);
	nvp6021_write(0x88, 0x40);

	nvp6021_write(0x30, 0x55); 	//TX ID
}
unsigned char acp_tx_command(unsigned char data0,unsigned char data1,unsigned char data2)
{
	nvp6021_write(0xFF,BANK3);	
	nvp6021_write(0x31,data0);
	nvp6021_write(0x32,data1);
	nvp6021_write(0x33,data2);
	nvp6021_write(0x29,0x08);

	nvp6021_write(0x29,0x00);

	return 0;
}

int nvp_video_init(int FirstInit, unsigned char VideoMode)
{
	unsigned char val=0;
	int ret = 0;
	switch(VideoMode)
	{
		case VIDEO_1080P30 : 
			nvp6021_write(0xFF, BANK0);
			nvp6021_write(0x01, 0xF0); 
			nvp6021_write(0x04, 0x00); 
			val = nvp6021_read(0x01);
			if (val != 0xF0) {
				ret = -1;
			}
			break;
		case VIDEO_1080P25 : 
			nvp6021_write(0xFF, BANK0);
			nvp6021_write(0x01, 0xF1); 
			val = nvp6021_read(0x01);
			if (val != 0xF1) {
				ret = -1;
			}
			nvp6021_write(0x04, 0x00); 
			break;
		default :
			break;
	}

	if(FirstInit)
	{
		nvp6021_write(0x00, 0x01); 	// must be set
	
		nvp6021_write(0xFF, BANK1);
		nvp6021_write(0x0E, 0x00); 	// Bit swap off
	
		nvp6021_write(0xFF, BANK2);
		nvp6021_write(0x00, 0xFE);
	
		nvp6021_write(0x02, 0x00);
	
		//nvp6021_write(0x04, 0x50);
		nvp6021_write(0x04, 0x80);//0x80);
		nvp6021_write(0x05, 0x00);	// swap cb-cr
		nvp6021_write(0x06, 0x10);
	
		nvp6021_write(0x0C, 0x04); 	// SYNC LEVEL
		nvp6021_write(0x0D, 0x3F); 	// BLACK LEVEL
		nvp6021_write(0x0E, 0x00);
		nvp6021_write(0x10, 0xEB);
		nvp6021_write(0x11, 0x10);
		nvp6021_write(0x12, 0xF0);
		nvp6021_write(0x13, 0x10);
		nvp6021_write(0x14, 0x01);
		nvp6021_write(0x15, 0x00);
		nvp6021_write(0x16, 0x00);
		nvp6021_write(0x17, 0x00);
		nvp6021_write(0x18, 0x00);
		nvp6021_write(0x19, 0x00);
	
		nvp6021_write(0x1C, 0x80); 	// y-scale
		nvp6021_write(0x1D, 0x80); 	// cb-scale
		nvp6021_write(0x1E, 0x80); 	// cr-scale
	
		nvp6021_write(0x20, 0x00);
		nvp6021_write(0x21, 0x00);
		nvp6021_write(0x22, 0x00);
		nvp6021_write(0x23, 0x00);
		nvp6021_write(0x24, 0x00);
		nvp6021_write(0x25, 0x00);
		nvp6021_write(0x26, 0x00);
		nvp6021_write(0x27, 0x00);
		nvp6021_write(0x28, 0x00);
		nvp6021_write(0x29, 0x00);
		nvp6021_write(0x2A, 0x00);
		nvp6021_write(0x2B, 0x00);
		nvp6021_write(0x2C, 0x00);
		nvp6021_write(0x2D, 0x00);
		nvp6021_write(0x2E, 0x00);
		nvp6021_write(0x2F, 0x00);
		nvp6021_write(0x30, 0x00);
		nvp6021_write(0x31, 0x00);
		nvp6021_write(0x32, 0x00);
		nvp6021_write(0x33, 0x00);
		nvp6021_write(0x34, 0x00);
	
		nvp6021_write(0x36, 0x01);
		nvp6021_write(0x37, 0x80);
	
		nvp6021_write(0x39, 0x00);
	
		nvp6021_write(0x3C, 0x00);
		nvp6021_write(0x3D, 0x00);
		nvp6021_write(0x3E, 0x00);
	
		nvp6021_write(0x40, 0x01);
		nvp6021_write(0x41, 0xFF);
		nvp6021_write(0x42, 0x80);
	
		nvp6021_write(0x48, 0x00);
		nvp6021_write(0x49, 0x00);
		nvp6021_write(0x4A, 0x00);
		nvp6021_write(0x4B, 0x00);
		nvp6021_write(0x4C, 0x00);
		nvp6021_write(0x4D, 0x00);
		nvp6021_write(0x4E, 0x00);
		nvp6021_write(0x4F, 0x00);
		nvp6021_write(0x50, 0x00);
		nvp6021_write(0x51, 0x00);
		nvp6021_write(0x52, 0x00);
		nvp6021_write(0x53, 0x00);
		nvp6021_write(0x54, 0x00);
		nvp6021_write(0x55, 0x00);
		nvp6021_write(0x56, 0x00);
		nvp6021_write(0x57, 0x00);
		nvp6021_write(0x59, 0x00);
		nvp6021_write(0x5A, 0x00);
		nvp6021_write(0x5C, 0x00);
		nvp6021_write(0x5D, 0x00);
		nvp6021_write(0x5E, 0x00);
		nvp6021_write(0x5F, 0x00);
	
	
		nvp6021_write(0x60, 0x80);
		nvp6021_write(0x61, 0x80);
		nvp6021_write(0x63, 0xE0);
		nvp6021_write(0x64, 0x19);
		nvp6021_write(0x65, 0x04);
		nvp6021_write(0x66, 0xEB);
		nvp6021_write(0x67, 0x60);
		nvp6021_write(0x68, 0x00);
		nvp6021_write(0x69, 0x00);
		nvp6021_write(0x6A, 0x00);
		nvp6021_write(0x6B, 0x00);
	}
	
	switch(VideoMode)
	{
		case VIDEO_1080P30 :
			nvp6021_write(0xFF, BANK2);		
			nvp6021_write(0x3A, 0x11);
			nvp6021_write(0x01, 0xF0); 
			break;
		case VIDEO_1080P25 :
			nvp6021_write(0xFF, BANK2);		
			nvp6021_write(0x3A, 0x11);
			nvp6021_write(0x01, 0xF1); 
			break;
		default :
			break;
	}
	
	switch(VideoMode)
	{
		case VIDEO_1080P30 :
			nvp6021_write(0xFF, BANK2);		
			nvp6021_write(0x3D, 0x80);   // pn enable

			nvp6021_write(0x5c, 0x52);   // pn value
			nvp6021_write(0x5d, 0xCA);   // pn value
			nvp6021_write(0x5e, 0xF0);   // pn value
			nvp6021_write(0x5f, 0x2C);   // pn value

			nvp6021_write(0x1D, 0xA0);   // cb scale - b0
			nvp6021_write(0x1E, 0xA0);   // cr scale - b0

			nvp6021_write(0x37, 0xA0);   // burst scale
			nvp6021_write(0x5A, 0x07);   // EXPANDER_MODE
			break;
		case VIDEO_1080P25 :
			nvp6021_write(0xFF, BANK2);		
			nvp6021_write(0x3D, 0x80);   // pn enable

			nvp6021_write(0x5c, 0x52);   // pn value
			nvp6021_write(0x5d, 0xC3);   // pn value
			nvp6021_write(0x5e, 0x7D);   // pn value
			nvp6021_write(0x5f, 0xC8);   // pn value

			nvp6021_write(0x1D, 0x80);   // cb scale -b0
			nvp6021_write(0x1E, 0x80);   // cr scale -b0

			nvp6021_write(0x37, 0x80);   // burst scale

			nvp6021_write(0x5A, 0x07);   // EXPANDER_MODE
			break;
		default :
			break;
	}

	return ret;
	
}
//==============================================================
static int tp2801_device_init(unsigned char type);
static int tp2801_device_exit(void);
static int nvp6021_device_init(unsigned char type);
static int nvp6021_device_exit(void);

int read_nvp(void)
{
	int i=0,j=0,val=0;
	for (i=0; i < 4; i++){
			nvp6021_write(0xFF, i);
			for (j = 0; j <= 0xff; j++){
				val = nvp6021_read(j);
				printk("---bank[%d],reg[0x%x]:0x%x\n",i,j,val);
			}
		}

}


int tp2801_write(unsigned char reg_addr,unsigned char value)
{
	int ret;
#ifdef GPIO_I2C
	ret = gpio_i2c_write(tp2801_addr,reg_addr,value);

#else
    unsigned char buf[2];
	struct i2c_client *client = tp2801_client;
    
    buf[0] = reg_addr;
    buf[1] = value;
    
	ret = i2c_master_send(client, buf, 2);
#endif

	return 1;
}

int tp2801_read(unsigned char reg_addr)
{
    int ret_data = 0xFF;
	int ret;
#ifdef GPIO_I2C
	ret_data = gpio_i2c_read(tp2801_addr,reg_addr);
#else
	struct i2c_client *client = tp2801_client;
    unsigned char buf[2];

    buf[0] = reg_addr;
	ret = i2c_master_recv(client, buf, 1);
    if (ret >= 0)
    {
        ret_data = buf[0];
    }
#endif
	return ret_data;
}
void tp2801_reg_dump(unsigned int reg_num)
{
    unsigned int i = 0;
	unsigned char reg[10]= {0x02,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0f,0x10,0x11};
	printk("tp2801 [reg]:[val]\n");
    for(i = 0;i < 10;i++)
    {
        printk("[0x%x] = 0x%x\n",reg[i],tp2801_read(reg[i]));
    }
	printk("\n");
}
void nvp6021_reg_dump(unsigned int reg_num)
{
    unsigned int i = 0;
	printk("nvp6021 reg:");
    for(i = 0;i < reg_num;i++)
    {
        printk("[%d] =%x,",i,nvp6021_read(i));
        if((i+1)%8==0)
        {
            printk("\n");
        }
    }
	printk("\n");
}

void __gpio_set(int addr, int pin, int val)
{
	unsigned char reg_val = 0;
	
	reg_val = HW_REG(addr);
	
	if (val){
		reg_val |= (0x1<<pin);	
	} else {
		reg_val &= ~(0x1<<pin);
	}
	
	HW_REG(addr) = reg_val;
	mdelay(10);
}


void reset_device(void)
{	
	int i=0;
	
	if (gDet_tvi_ahd == DET_AHD){
		__gpio_set(GPIO13_DATA_REG, 1, 0);
		rst_tvi_ahd(GPIO12_DATA_REG, 4); //ahd
	} else {
		__gpio_set(GPIO12_DATA_REG, 4, 0);
		for (i= 0; i < 3; i++) {
			rst_tvi_ahd(GPIO13_DATA_REG, 1);//tvi
		}
	}
}
    	
int coc_func (void)
{
	unsigned char RxIntStatus = 0;
	unsigned char RxBuffer[8];
	unsigned char tmp1=0, tmp2=0;
	unsigned char Sum; 
	int i,j,val=0,n,ret;
	COC_DEV* pdev;
	COC_DATA_TYPE tmp_data;
	static int count=0;
	static int rst_cnt=0;
	static int null_cnt=0;
	
	count++;
	printk("---isr,count:%d\n",count);
	if (count >= 60&& rst_cnt <= 5){
		rst_cnt++;
		count = 0;
		reset_device();
		
		if (gDet_tvi_ahd == DET_TVI){
			ret = i2c_client_init();
			if (ret){
				printk("---gpio i2c tp2801 init failed.\n");
			} 

			tp2801_device_init(gvideo_type);
		} else {
			nvp6021_device_init(gvideo_type);
		}
		kfifo_reset(&coc_fifo);
		return 0;
	}

	pdev = &coc_device;
	memset(RxBuffer,0x00,8);
	if(gDet_tvi_ahd == DET_TVI) {
		if (RegisterRead(0x69, &RxIntStatus) == 1)
		{
			if ((RxIntStatus & 0x01) != 0)	// 接收到数据
			{
				RegisterRead(0x5F, &tmp1);
				udelay(10);
				RegisterRead(0x60, &RxBuffer[0]);
				udelay(10);
				RegisterRead(0x61, &RxBuffer[1]);
				udelay(10);
				RegisterRead(0x62, &RxBuffer[2]);
				udelay(10);
				RegisterRead(0x63, &RxBuffer[3]);
				udelay(10);
				RegisterRead(0x64, &tmp2);
				udelay(10);
				RegisterRead(0x65, &RxBuffer[4]);
				udelay(10);
				RegisterRead(0x66, &RxBuffer[5]);
				udelay(10);
				RegisterRead(0x67, &RxBuffer[6]);
				udelay(10);
				RegisterRead(0x68, &RxBuffer[7]);

				if (((tmp1&0x03) == 0x02) && ((tmp2&0x03) == 0x02))	// check
				{
					Sum = 0;
					for (i=0; i<7; i++)
					{
						Sum += RxBuffer[i];
					}
					
					if ((RxBuffer[0] == 0xB5) && (Sum == RxBuffer[7]))		// check synch and checksum
					{
						pdev->coc_data.dev_type = DET_TVI;
						memcpy(&pdev->coc_data.data[0],&RxBuffer[2],5);

						if (kfifo_is_full(&coc_fifo)) {
							kfifo_out(&coc_fifo,&tmp_data,1);
						}
						n=kfifo_in(&coc_fifo,&pdev->coc_data,1);
						
						//printk("---kfifo_put:0x%x,element:%d\n",pdev->coc_data.data[0],n);
					#ifndef  COC_BUF
						if (pdev->coc_read_flag != COC_FLAG_READY){
							pdev->coc_read_flag = COC_FLAG_READY;
							wake_up(&pdev->coc_readque);
						}
					#endif
					}
				}
			}
		}
	}else {
		nvp6021_write(0xFF,BANK3);
		RxBuffer[0] = nvp6021_read(0x91);
		RxBuffer[1] = nvp6021_read(0x93);
		RxBuffer[2] = nvp6021_read(0x9a);
		RxBuffer[3] = nvp6021_read(0x9b);

		for(i = 0; i < 4; i++){
			if(RxBuffer[i]) count=0;
		}
		//printk("---rx_buf[0]:0x%x,0x%x,0x%x,0x%x\n",RxBuffer[0],RxBuffer[1],RxBuffer[2],RxBuffer[3]);

		if (RxBuffer[0] != 0xaa)
		{
			//==================================
			if (RxBuffer[0] == 0 && RxBuffer[1] == 0 && RxBuffer[2] == 0 && RxBuffer[3] == 0){
				null_cnt++;
			} else {
				null_cnt = 0;
			}
			if (null_cnt >= 2) {
				if (RxBuffer[0] == 0 && RxBuffer[1] == 0 && RxBuffer[2] == 0 && RxBuffer[3] == 0) {
					nvp6021_write(0x5A,0x01);	// RX Data register clear
					nvp6021_write(0x5A,0x00);
					nvp6021_write(0x7C,0x00);	// RX Data Read Done clear
					return 0;
				}	
			}
			pdev->coc_data.dev_type = DET_AHD;
			memcpy(&pdev->coc_data.data[0],&RxBuffer[0],4);

			if (kfifo_is_full(&coc_fifo)) {
				kfifo_out(&coc_fifo,&tmp_data,1);
			}
			n=kfifo_in(&coc_fifo,&pdev->coc_data,1);
				
#ifndef  COC_BUF
				if (pdev->coc_read_flag != COC_FLAG_READY){
					pdev->coc_read_flag = COC_FLAG_READY;
					wake_up(&pdev->coc_readque);
				}
#endif
		
		} else {
			memset(RxBuffer,0x00,8);
		}

		nvp6021_write(0x5A,0x01);	// RX Data register clear
		nvp6021_write(0x5A,0x00);
		nvp6021_write(0x7C,0x00);	// RX Data Read Done clear

	}

}

/*
 *	device open. set counter
 */
static int tp2801_nvp6021_open(struct inode * inode, struct file * file)
{
	open_cnt++;

	if(open_cnt > 1) {
		printk("---open coc device.\n");
	}
	
	return 0;    	
}

/*
 *	Close device, Do nothing!
 */
static int tp2801_nvp6021_close(struct inode *inode ,struct file *file)
{
	open_cnt--;

	return 0;
}

//static int tp2801_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
static long tp2801_nvp6021_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret=0;
	static int type=00;
	unsigned int __user *argp = (unsigned int __user *)arg;
	COC_DEV * pdev;
	COC_DATA_TYPE tmp_coc_data;
	int tdata=0;
	unsigned char val=0,reg=0,dev_flag=0,ahd_group=0;
	memset(&tmp_coc_data,0x00,sizeof(COC_DATA_TYPE));

	pdev = &coc_device;

	switch(cmd)
	{
		case COC_DEV_READ:
			if (kfifo_out(&coc_fifo,&tmp_coc_data,1)){
				tmp_coc_data.dev_type = gDet_tvi_ahd;
				if (copy_to_user(argp, &tmp_coc_data, sizeof(COC_DATA_TYPE)))
	            {
	                printk(KERN_INFO "copy lib infor to user failed!\n");
	                return -EFAULT;
	            }

			} else {
				return -EFAULT;
			}
		#ifndef COC_BUF
			if(pdev->coc_read_flag != COC_FLAG_READY){
				ret = wait_event_interruptible(pdev->coc_readque, ((pdev->coc_read_flag == COC_FLAG_READY)));
				if (ret) {
					printk("wait the interruptible timeout for read!\n");
					return -EFAULT;
				}	
			}
			pdev->coc_data.dev_type = gDet_tvi_ahd;
			if (copy_to_user(argp, &pdev->coc_data, sizeof(COC_DATA_TYPE)))
            {
                printk(KERN_INFO "copy lib infor to user failed!\n");
                return -EFAULT;
            }
			memset(&pdev->coc_data,0x00,sizeof(COC_DATA_TYPE));
			pdev->coc_read_flag = COC_FLAG_BUSY;
			
		#endif
		
			break;
			
		case COC_DEV_WRITE:
			if (copy_from_user(&pdev->coc_data, argp, sizeof(COC_DATA_TYPE)))
            {
                printk("copy isp lib info from user failed!\n");
                return -EFAULT;
            }
			break;
			
		case COC_GET_DEV:
			break;
			
		case COC_DEV_INIT:
			if (copy_from_user(&type, argp, sizeof(int)))
            {
                printk("copy isp lib info from user failed!\n");
               // return -EFAULT;
            }

			printk("---type:%d\n",type);
			gvideo_type = type;
			init_reg_hw();
			reset_device();

			if (gDet_tvi_ahd == DET_TVI){
				ret = i2c_client_init();
				if (ret){
					printk("---gpio i2c tp2801 init failed.\n");
				} 
		
				tp2801_device_init(gvideo_type);
			} else {
				nvp6021_device_init(gvideo_type);
			}
			kfifo_reset(&coc_fifo);
	
			//ret = mod_timer( &isr_timer, jiffies + HZ/2);
			//if (ret) printk("Error in mod_timer\n");
			break;

		case COC_DEV_RESET:
			reset_device();
			mdelay(100);
			printk("---read nvp6021:\n");
			//acp_tx_command(0x00,0x00,0x01);
			//nvp6021_write(0xFF, BANK2);
			//nvp6021_write(0x02, 0x00);
			nvp6021_device_init(type);
			//read_nvp();
			
			kfifo_reset(&coc_fifo);
			break;
		case TVI_IQ_W:
			if (copy_from_user(&tdata, argp, sizeof(int)))
            {
                printk("copy TVI_IQ_W info from user failed!\n");
               // return -EFAULT;
            }
			dev_flag = (tdata>>20)&0xff;
			ahd_group = (tdata>>16)&0xff;
			reg = (tdata>>8)&0xff;
			val = tdata&0xff;
			if (dev_flag == 0) { //tvi
				ret = RegisterWrite(reg, val);
				if (ret==0) {
					printk("---write reg[0x%x] failed.\n",reg);
				}
			} else {//ahd
				nvp6021_write(0xFF,ahd_group);
				ret = nvp6021_write(reg, val);
				if (ret==0) {
					printk("---write group[0x%x],reg[0x%x] failed.\n",ahd_group,reg);
				}
			}
			break;
		case TVI_IQ_R:
			if (copy_from_user(&tdata, argp, sizeof(int)))
            {
                printk("copy TVI_IQ_r info from user failed!\n");
               // return -EFAULT;
            }
			dev_flag = (tdata>>20)&0xff;
			ahd_group = (tdata>>16)&0xff;
			reg = (tdata>>8)&0xff;
			if (dev_flag == 0) {// tvi
				RegisterRead(reg, &val);
				tdata = (reg<<8)|val;
			} else { //ahd
				nvp6021_write(0xFF,ahd_group);
				val = nvp6021_read(reg);

				tdata |= val; 
			}
			if (copy_to_user(argp, &tdata, sizeof(COC_DATA_TYPE)))
            {
                printk(KERN_INFO "copy TVI_IQ_r info to user failed!");
                return -EFAULT;
            }
			break;
		default:
			printk("---coc cmd error...\n");
			break;
	}
    return ret;
}
/*
 *  The various file operations we support.
 */
 
static struct file_operations tp2801_nvp6021_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl		= tp2801_nvp6021_ioctl,
	.open		= tp2801_nvp6021_open,
	.release	= tp2801_nvp6021_close
};


static struct miscdevice tp2801_nvp6021_dev = {
	MISC_DYNAMIC_MINOR,
	DEV_NAME,
	&tp2801_nvp6021_fops,
};

static int tp2801_device_init(unsigned char type)
{
    /* inite codec configs.*/
    TviVideoInit(type);

    return 0;
}  

static int tp2801_device_exit(void)
{
        /* HPLOUT is mute */
	   //	tp2801_write(IIC_device_addr[num], 51, 0x04);

        /* HPROUT is mute */	
  		//tp2801_write(IIC_device_addr[num], 65, 0x04);

    	return 0;
}

static int nvp6021_device_init(unsigned char type)
{
	int ret=0;
	 
	nvp6021_write(0xFF,BANK3);
	nvp6021_write(0x78,0x00);
	mdelay(20);

	ret = nvp_video_init(1, type);
	Init_TX_coax();
	Init_RX_coax();
	if (ret < 0){
		printk("---gpio i2c nvp6021 init failed.\n");
	} 
	nvp6021_write(0xFF,BANK3);
	nvp6021_write(0x5a,0x01);
	mdelay(20);
	nvp6021_write(0x78,0x01);
	mdelay(20);
	nvp6021_write(0x5a,0x00);
	mdelay(20);
	return 0;
}  
static int nvp6021_device_exit(void)
{
        /* HPLOUT is mute */
	   //	tp2801_write(IIC_device_addr[num], 51, 0x04);

        /* HPROUT is mute */	
  		//tp2801_write(IIC_device_addr[num], 65, 0x04);

    	return 0;
}
static int i2c_client_init(void)
{
#ifdef GPIO_I2C
	unsigned char val=0;

	val = gpio_i2c_read(0x88,0xfe);
printk("---read tp2801 0x88 reg[0xfe]=0x%x\n",val);
	if (val == 0x28) {
		tp2801_addr = 0x88;
		return 0;
	}
	val = gpio_i2c_read(0x8a,0xfe);
printk("---read tp2801 0x8a reg[0xfe]=0x%x\n",val);
	if (val == 0x28) {
		tp2801_addr = 0x8a;
		return 0;
	}

	return -1;
	
#else
    struct i2c_adapter *i2c_adap;

    // use i2c2
    i2c_adap = i2c_get_adapter(2);
	tp2801_client = i2c_new_device(i2c_adap, &tp2801_info);
	if (tp2801_client == NULL) {
		mdelay(100);
		tp2801_client = i2c_new_device(i2c_adap, &tp28012_info);
		if (tp2801_client == NULL) {
			printk("---tp2801 for addr 0x8a failed.\n");
		} else {
			printk("---tp2801 for addr 0x8a ok.\n");
		}
	} else {
		printk("---tp2801 for addr 0x88 ok.\n");
	}
	i2c_put_adapter(i2c_adap);

	nvp6021_client = i2c_new_device(i2c_adap, &nvp6021_info);
	
	i2c_put_adapter(i2c_adap);
	return 0;
#endif
    
}

static void i2c_client_exit(void)
{
#ifndef GPIO_I2C
	if (tp2801_client)
    	i2c_unregister_device(tp2801_client);
	if (nvp6021_client)
		i2c_unregister_device(nvp6021_client);
#endif
}

void set_GPIO(unsigned int addr, unsigned int pin, unsigned int val)
{
	unsigned char reg_val = 0;

	if (val) {
		reg_val = HW_REG(addr);
		reg_val |= (0x1<<pin);
		HW_REG(addr) = reg_val;
	} else {
		mdelay(10);
		reg_val = HW_REG(addr);
		reg_val &= ~(0x1<<pin);
		HW_REG(addr) = reg_val;
	}
}

void rst_tvi_ahd(unsigned int addr, unsigned int pin)
{
	unsigned char reg_val = 0;

	reg_val = HW_REG(addr);
	reg_val |= (0x1<<pin);
	HW_REG(addr) = reg_val;
	mdelay(10);
	reg_val = HW_REG(addr);
	reg_val &= ~(0x1<<pin);
	HW_REG(addr) = reg_val;
	mdelay(200);
	reg_val = HW_REG(addr);
	reg_val |= (0x1<<pin);
	HW_REG(addr) = reg_val;

	mdelay(50);

}

#if 01
static inline irqreturn_t COC_ISR(int irq, void *dev)
{
	if (irq == GPIO13_IRQ_NR) {
		HW_REG(GPIO13_IC) = 0X1;//mask
		gDet_tvi_ahd = DET_TVI;
	} else {
		HW_REG(GPIO12_IC) = 0X1<<3;//mask
		gDet_tvi_ahd = DET_AHD;
	}
#ifndef GPIO_I2C
	if (tp2801_client || nvp6021_client)
#endif
		tasklet_schedule(&coc_task);
	
    return IRQ_HANDLED;
}
#endif
int init_reg_hw(void)
{
	unsigned int ret,i;
	unsigned char reg_val = 0;

		//FOR tvi/ahd vo
	HW_REG(MULCTL_REG106) = 0;//GPIO15_2  DET_TVI/AHD
	HW_REG(MULCTL_REG110) = 0;//GPIO12_3  INT_AHD
	HW_REG(MULCTL_REG111) = 0;//GPIO12_4  AHD_RST
	HW_REG(MULCTL_REG115) = 0;//GPIO13_0  INT_TVI
	HW_REG(MULCTL_REG116) = 0;//GPIO13_1  TVI_RST
	
	//HW_REG(MULCTL_VOCLK) = 0X1;//GPIO1_0	vo_clk

	

//set vo data[0...15]
	for(i=0; i <= 15; i++)
	{
		HW_REG(MULCTL_VO_REG(i)) = 0x1;

	}
//set output
	reg_val = HW_REG(GPIO12_DIR_REG);	
	reg_val |= (0x1<<4);//GPIO12_4 output
	HW_REG(GPIO12_DIR_REG) = reg_val;

	reg_val = HW_REG(GPIO13_DIR_REG);	
	reg_val |= (0x1<<1);//GPIO13_1output
	HW_REG(GPIO13_DIR_REG) = reg_val;
//====================================
//set input for COC's intrupter
	reg_val = HW_REG(GPIO13_DIR_REG);
	reg_val &= 0xfe;  //set gpio13_0 input
	HW_REG(GPIO13_DIR_REG) = reg_val;
	
	HW_REG(GPIO13_IS) = 0X0;//edge-sensitive
	HW_REG(GPIO13_IEV) = 0X1;//raisge
	HW_REG(GPIO13_IBE) = 0X0;//single
	HW_REG(GPIO13_IC) = 0X0;//mask
	HW_REG(GPIO13_IE) = 0X1;//enable
	
	

	reg_val = HW_REG(GPIO12_DIR_REG);
	reg_val &= 0xf7;  //set gpio12_3 input
	HW_REG(GPIO12_DIR_REG) = reg_val;
	HW_REG(GPIO12_IS) = 0X0;//edge-sensitive
	HW_REG(GPIO12_IEV) = 0X1<<3;//raisge
	HW_REG(GPIO12_IBE) = 0X0;//single
	HW_REG(GPIO12_IC) = 0X0;//mask
	HW_REG(GPIO12_IE) = 0X1<<3;//enable
//==================================

//set input for detect AHD/TVI
//	reg_val = HW_REG(GPIO15_DIR_REG);
//	reg_val &= 0xfb;  //set gpio15_2 input
//	HW_REG(GPIO15_DIR_REG) = reg_val;
//	mdelay(10);

//	reg_val = HW_REG(GPIO15_DATA_REG);
//	reg_val &= (0x1<<2);
//	if (reg_val == 0) {
//		gDet_tvi_ahd = DET_AHD;
//	} else {
//		gDet_tvi_ahd = DET_TVI;
//	}
//	printk("---gDet_tvi_ahd:%s\n",(gDet_tvi_ahd==DET_TVI)?"tvi":"ahd");
	
}

static void isr_timer_handle(unsigned long data)//定时器处理函数 
{
	int ret;
	struct timeval tmp_tv;
	printk( "---isr_timer_handle called (%lld).\n", jiffies );
	ret = mod_timer( &isr_timer, jiffies + 5*HZ);
	if (ret) printk("Error in mod_timer\n");
	do_gettimeofday(&tmp_tv);
	printk("---tv.sec:%ld,tv.usec:%ld\n",tmp_tv.tv_sec,tmp_tv.tv_usec);

}

static int __init tp2801_nvp6021_init(void)
{
	unsigned int ret=0,i;
	unsigned char reg_val = 0;
	COC_DEV * pdev;
	
	pdev = &coc_device;

//	init_reg_hw();
//if (reset_num == 1){
//	reset_device();
//}
	//set input for detect AHD/TVI
	reg_val = HW_REG(GPIO15_DIR_REG);
	reg_val &= 0xfb;  //set gpio15_2 input
	HW_REG(GPIO15_DIR_REG) = reg_val;
	mdelay(10);

	reg_val = HW_REG(GPIO15_DATA_REG);
	reg_val &= (0x1<<2);
	if (reg_val == 0) {
		gDet_tvi_ahd = DET_AHD;
	} else {
		gDet_tvi_ahd = DET_TVI;
	}
	printk("---gDet_tvi_ahd:%s\n",(gDet_tvi_ahd==DET_TVI)?"tvi":"ahd");
	gpio_i2c_pin_init();

	ret = i2c_client_init();
	if (ret){
		printk("---gpio i2c tp2801 init failed.\n");
	} 
	mdelay(100);
	init_waitqueue_head(&pdev->coc_readque);
	init_waitqueue_head(&pdev->coc_writeque);
	pdev->coc_read_flag = COC_FLAG_INIT;
	
	INIT_KFIFO(coc_fifo);

	ret = misc_register(&tp2801_nvp6021_dev);
	if(ret)
	{
		DPRINTK(0,"could not register tp2801_nvp6021 device");
		return -1;
	}
	if (gDet_tvi_ahd == DET_TVI){
		pdev->coc_data.dev_type = DET_TVI;
		ret = request_irq(GPIO13_IRQ_NR, COC_ISR, IRQF_SAMPLE_RANDOM, "TVI_DET", NULL);
		if(ret){
				printk(KERN_ERR  "gpio13 Register Interrupt Failed!,ret:%d\n",ret);
				return -EAGAIN;
		}
	} else {
		pdev->coc_data.dev_type = DET_AHD;
		ret = request_irq(GPIO12_IRQ_NR, COC_ISR, IRQF_SAMPLE_RANDOM, "AHD_DET", NULL);

	    if(ret){
	        printk(KERN_ERR  "gpio12 Register Interrupt Failed!,ret:%d\n",ret);
	        return -EAGAIN;
	    }
	}

	printk("Timer module installing\n");

	init_timer(&isr_timer);

	setup_timer( &isr_timer, isr_timer_handle, 0);

	DPRINTK(1,"tp2801_nvp6021 driver init successful!");
	printk("---date:%s %s load tp2801_nvp6021.ko for Hi3516A ok!\n",__DATE__,__TIME__);
	
	return ret;
	
init_fail:

    DPRINTK(0,"tp2801_nvp6021 device init fail,deregister it!");
    return -1;
}     

static void __exit tp2801_nvp6021_exit(void)
{
	unsigned int i;

	free_irq(GPIO13_IRQ_NR, NULL);

	free_irq(GPIO12_IRQ_NR, NULL);

	for(i = 0;i< chip_count;i++)
    {
        tp2801_device_exit();
		nvp6021_device_exit();
    }    
	
    misc_deregister(&tp2801_nvp6021_dev);

    i2c_client_exit();

	del_timer(&isr_timer);

    DPRINTK(1,"deregister tp2801_nvp6021");
	printk("rmmod tp2801_nvp6021.ko for Hi3516A ok!\n");
}

module_init(tp2801_nvp6021_init);
module_exit(tp2801_nvp6021_exit);
module_param(det, int ,S_IRUGO);
module_param(reset_num, uint, S_IRUGO);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hisilicon");
