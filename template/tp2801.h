#ifndef _TP2801_H_
#define _TP2801_H_


#define BANK0	0x00
#define BANK1	0x01
#define BANK2	0x02
#define BANK3	0x03

#define BANK4	0x04
#define BANK5	0x05
#define BANK6	0x06

#define TP2801_ADDR (0X88)
#define TP28012_ADDR (0X8a)

#define NVP6021_ADDR (0X60)
enum {
	DET_AHD,
	DET_TVI
};

typedef struct
{
    unsigned char RegAddr;
    unsigned char Data;
}TP2801_PARA_TYPE;

#define VIDEO_720P25        0
#define VIDEO_720P30        1
#define VIDEO_720P50        2
#define VIDEO_720P60        3
#define VIDEO_1080P25       4
#define VIDEO_1080P30       5

const TP2801_PARA_TYPE TP2801_720P60[10]=
{
	{0x02,0x8B},
	{0x08,0x6E},
	{0x09,0x28},
	{0x0A,0x28},
	{0x0B,0x05},
	{0x0C,0x04},
	{0x0D,0x04},
	{0x0F,0x72},
	{0x10,0x06},
	{0x11,0x82},
};

const TP2801_PARA_TYPE TP2801_720P50[10]=
{
	{0x02,0x9B},
	{0x08,0xB8},
	{0x09,0x28},
	{0x0A,0x28},
	{0x0B,0x15},
	{0x0C,0x04},
	{0x0D,0x04},
	{0x0F,0xBC},
	{0x10,0x17},
	{0x11,0xCC},
};

const TP2801_PARA_TYPE TP2801_720P30[10]=
{
	{0x02,0x8F},
	{0x08,0xE0},
	{0x09,0x28},
	{0x0A,0x28},
	{0x0B,0x45}, //0x0B,0x65,
	{0x0C,0x04},
	{0x0D,0x04},
	{0x0F,0xE4},
	{0x10,0x6C},
	{0x11,0xF4},
};

const TP2801_PARA_TYPE TP2801_720P25[10]=
{
	{0x02,0x9F},   
	{0x08,0x74},   
	{0x09,0x28},   
	{0x0A,0x28},   
	{0x0B,0x65}, //0x0B,0x95,  
	{0x0C,0x04},   
	{0x0D,0x04},   
	{0x0F,0x78},   
	{0x10,0x9F},   
	{0x11,0x88},
};

const TP2801_PARA_TYPE TP2801_1080P30[10]=
{
	//{0x02,0xC7},    //测试色条卡
	{0x02,0x87},
	{0x08,0x58},
	{0x09,0x2C},
	{0x0A,0x2C},
	{0x0B,0x00},
	{0x0C,0xC0},
	{0x0D,0xC0},
	{0x0F,0x98},
	{0x10,0x08},
	{0x11,0x6C},
};

const TP2801_PARA_TYPE TP2801_1080P25[10]=
{
	//{0x02,0xc7},   

	{0x02,0x97},    
	{0x08,0x10},    
	{0x09,0x2C},    
	{0x0A,0x2C},    
	{0x0B,0x20},	  
	{0x0C,0xC0},    
	{0x0D,0xC0},    
	{0x0F,0x50},    
	{0x10,0x2A},    
	{0x11,0x24},		
};

const TP2801_PARA_TYPE TP2801_Comm[33]=
{
	{0x1B, 0x80},//0xA4},	//0x90,	//新增
	{0x1C, 0x40},//0x7A},	//0x92,   //修改颜色，新增三个参数
	{0x1D, 0x60},//0xC0},	//0xCD,
	{0x1E, 0x55},//0xF0}, //0xA0,	//0x90,
  
	{0x20, 0x48},
	{0x21, 0xBB},	//0xBA,
	{0x22, 0x2E},
	{0x23, 0x8B},
  
	{0x41, 0x00},
	{0x42, 0x14},
	{0x43, 0x41}, //0x8843, 0x07,
	{0x44, 0x4A}, //0x8844, 0x49,
	{0x45, 0xCB},//0xCB
	{0x46, 0xAA},
	{0x47, 0x1F},
	{0x48, 0xFA},
	{0x49, 0x00},
	{0x4A, 0x07},
	{0x4B, 0x08},
  
	{0x56, 0xA7},
	{0x57, 0x04},
	{0x58, 0xEC}, //0x00,
	{0x59, 0x4E}, //0x00,
	{0x5A, 0x0A}, //0x10,
	{0x5E, 0xA1}, //0xA7,
	{0x5B, 0x00},
	{0x5D, 0x00}, //0x0B,
	{0x5C, 0x00}, //0x0A,
	{0x5D, 0x0C}, // hk:0x0B
	{0x5C, 0x0B}, // hk:0x0A
  
	{0x69, 0x03},
	{0x6A, 0xFA},
	{0x6B, 0xF0},
	//{0x6C, 0x00},
	//{0xFD, 0x00},
};	

typedef struct
{
    unsigned char dev_type;
    unsigned char data[8];
}COC_DATA_TYPE;

enum
{
	COC_FLAG_INIT = 0,
	COC_FLAG_BUSY,
	COC_FLAG_READY,
};

typedef struct
{
    int cnt;
	
    COC_DATA_TYPE coc_data;
	int coc_read_flag;
	int coc_write_flag;
	wait_queue_head_t coc_readque;
	wait_queue_head_t coc_writeque;
}COC_DEV;

#define COC_DEV_READ        0X01
#define COC_DEV_WRITE 	    0X02
#define COC_GET_DEV    		0X03
#define COC_SET_DEV_TYPE    0X04
#define COC_DEV_RESET    	0X05
#define COC_DEV_INIT    	0X06
#define TVI_IQ_W            0X07
#define TVI_IQ_R            0X08



unsigned char RegisterWrite(unsigned char RegisterAddr, unsigned char Data);		// 操作成功返回1，否则返回0;
unsigned char RegisterRead(unsigned char RegisterAddr, unsigned char *pData);		// 操作成功返回1，否则返回0;
void TviVideoInit(unsigned char Mode);
void TviCoaxRead(void);
void TviCoaxSend(unsigned char *pBuf);
void DecodeTviCoaxCmd(const unsigned char *RxBuffer);
void Tp2801Reset(void);
int tp2801_write(unsigned char reg_addr,unsigned char value);
int tp2801_read(unsigned char reg_addr);
void rst_tvi_ahd(unsigned int addr, unsigned int pin);


#endif

