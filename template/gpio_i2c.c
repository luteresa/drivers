
#include <linux/kernel.h>
#include <linux/version.h>

#include <linux/module.h>
//#include <asm/hardware.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/fcntl.h>

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/workqueue.h>

#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/io.h>

#include "gpio_i2c.h" 

spinlock_t  gpioi2c_lock;

//GPIO9_0/I2C2_SDA
//GPIO9_1/I2C2_SCL

#define  GPIO_CRTL_BASE_ADDR_REG    (IO_ADDRESS(0x200F0000))

#define  GPIO_CRTL_SDA2_ADDR_REG    (GPIO_CRTL_BASE_ADDR_REG+0x070)
#define  GPIO_CRTL_SCL2_ADDR_REG    (GPIO_CRTL_BASE_ADDR_REG+0x074)

#define GPIO_0_BASE 0x201D0000

#define GPIO_0_DIR IO_ADDRESS(GPIO_0_BASE + 0x400)

#define SCL_SHIFT_NUM   1
#define SDA_SHIFT_NUM   0

#define SCL                 (1 << SCL_SHIFT_NUM)    /* GPIO 0_0 */
#define SDA                 (1 << SDA_SHIFT_NUM)    /* GPIO 0_1 */
#define GPIO_I2C_SCL_REG    IO_ADDRESS(GPIO_0_BASE + (0x1<<(SCL_SHIFT_NUM+2)))
#define GPIO_I2C_SDA_REG    IO_ADDRESS(GPIO_0_BASE + (0x1<<(SDA_SHIFT_NUM+2)))

#define GPIO_I2C_SCLSDA_REG IO_ADDRESS(GPIO_0_BASE + ((0x1<<(SCL_SHIFT_NUM+2))+(0x1<<(SDA_SHIFT_NUM+2))))

#define HW_REG(reg)         *((volatile unsigned int *)(reg))
#define DELAY(us)           time_delay_us(us)


typedef struct _mul_i2c
{
	int addr;
	int val;
}multi_i2c_s;



/* 
 * I2C by GPIO simulated  clear 0 routine.
 *
 * @param whichline: GPIO control line
 *
 */
static void i2c_clr(unsigned char whichline)
{
	unsigned char regvalue;
	
	if(whichline == SCL)
	{
		regvalue = HW_REG(GPIO_0_DIR);
		regvalue |= SCL;
		HW_REG(GPIO_0_DIR) = regvalue;
		
		HW_REG(GPIO_I2C_SCL_REG) = 0;
		return;
	}
	else if(whichline == SDA)
	{
		regvalue = HW_REG(GPIO_0_DIR);
		regvalue |= SDA;
		HW_REG(GPIO_0_DIR) = regvalue;
		
		HW_REG(GPIO_I2C_SDA_REG) = 0;
		return;
	}
	else if(whichline == (SDA|SCL))
	{
		regvalue = HW_REG(GPIO_0_DIR);
		regvalue |= (SDA|SCL);
		HW_REG(GPIO_0_DIR) = regvalue;
		
		HW_REG(GPIO_I2C_SCLSDA_REG) = 0;
		return;
	}
	else
	{
		printk("Error input.\n");
		return;
	}
	
}

/* 
 * I2C by GPIO simulated  set 1 routine.
 *
 * @param whichline: GPIO control line
 *
 */
static void  i2c_set(unsigned char whichline)
{
	unsigned char regvalue;
	
	if(whichline == SCL)
	{
		regvalue = HW_REG(GPIO_0_DIR);
		regvalue |= SCL;
		HW_REG(GPIO_0_DIR) = regvalue;
		
		HW_REG(GPIO_I2C_SCL_REG) = SCL;
		return;
	}
	else if(whichline == SDA)
	{
		regvalue = HW_REG(GPIO_0_DIR);
		regvalue |= SDA;
		HW_REG(GPIO_0_DIR) = regvalue;
		
		HW_REG(GPIO_I2C_SDA_REG) = SDA;
		return;
	}
	else if(whichline == (SDA|SCL))
	{
		regvalue = HW_REG(GPIO_0_DIR);
		regvalue |= (SDA|SCL);
		HW_REG(GPIO_0_DIR) = regvalue;
		
		HW_REG(GPIO_I2C_SCLSDA_REG) = (SDA|SCL);
		return;
	}
	else
	{
		printk("Error input.\n");
		return;
	}
}

/*
 *  delays for a specified number of micro seconds rountine.
 *
 *  @param usec: number of micro seconds to pause for
 *
 */
void time_delay_us(unsigned int usec)
{
	volatile int i,j;
	
	for(i=0;i<usec * 3;i++)
	{
		for(j=0;j<47*2;j++)
		{;}
	}
}

/* 
 * I2C by GPIO simulated  read data routine.
 *
 * @return value: a bit for read 
 *
 */
 
static unsigned char i2c_data_read(void)
{
	unsigned char regvalue;
	
	regvalue = HW_REG(GPIO_0_DIR);
	regvalue &= (~SDA);
	HW_REG(GPIO_0_DIR) = regvalue;
	DELAY(1);
		
	regvalue = HW_REG(GPIO_I2C_SDA_REG);
	if((regvalue&SDA) != 0)
		return 1;
	else
		return 0;
}



/*
 * sends a start bit via I2C rountine.
 *
 */
static void i2c_start_bit(void)
{
        DELAY(1);
        i2c_set(SDA | SCL);
        DELAY(1);
        i2c_clr(SDA);
        DELAY(2);
}

/*
 * sends a stop bit via I2C rountine.
 *
 */
static void i2c_stop_bit(void)
{
        /* clock the ack */
        DELAY(1);
        i2c_set(SCL);
        DELAY(1); 
        i2c_clr(SCL);  

        /* actual stop bit */
        DELAY(1);
        i2c_clr(SDA);
        DELAY(1);
        i2c_set(SCL);
        DELAY(1);
        i2c_set(SDA);
        DELAY(1);
}

/*
 * sends a character over I2C rountine.
 *
 * @param  c: character to send
 *
 */
static void i2c_send_byte(unsigned char c)
{
    int i;
    local_irq_disable();
    for (i=0; i<8; i++)
    {
        DELAY(1);
        i2c_clr(SCL);
        DELAY(1);

        if (c & (1<<(7-i)))
            i2c_set(SDA);
        else
            i2c_clr(SDA);

        DELAY(1);
        i2c_set(SCL);
        DELAY(1);
        i2c_clr(SCL);
    }
    DELAY(1);
   // i2c_set(SDA);
    local_irq_enable();
}

/*  receives a character from I2C rountine.
 *
 *  @return value: character received
 *
 */
static unsigned char i2c_receive_byte(void)
{
    int j=0;
    int i;
    unsigned char regvalue;

    local_irq_disable();
    for (i=0; i<8; i++)
    {
        DELAY(1);
        i2c_clr(SCL);
        DELAY(2);
        i2c_set(SCL);
        
        regvalue = HW_REG(GPIO_0_DIR);
        regvalue &= (~SDA);
        HW_REG(GPIO_0_DIR) = regvalue;
        DELAY(1);
        
        if (i2c_data_read())
            j+=(1<<(7-i));

        DELAY(1);
        i2c_clr(SCL);
    }
    local_irq_enable();
    DELAY(1);
   // i2c_clr(SDA);
   // DELAY(1);

    return j;
}

/*  receives an acknowledge from I2C rountine.
 *
 *  @return value: 0--Ack received; 1--Nack received
 *          
 */
static int i2c_receive_ack(void)
{
    int nack;
    unsigned char regvalue;
    
    DELAY(1);
    
    regvalue = HW_REG(GPIO_0_DIR);
    regvalue &= (~SDA);
    HW_REG(GPIO_0_DIR) = regvalue;
    
    DELAY(1);
    i2c_clr(SCL);
    DELAY(1);
    i2c_set(SCL);
    DELAY(1);
    
    

    nack = i2c_data_read();

    DELAY(1);
    i2c_clr(SCL);
    DELAY(1);
  //  i2c_set(SDA);
  //  DELAY(1);

    if (nack == 0)
        return 1; 

    return 0;
}

#if 1

static void i2c_multi_start_bit(void)
{
        DELAY(1);
        //i2c_set(SDA | SCL);
        i2c_set(SDA );
        i2c_clr(SCL);
        DELAY(1);
        i2c_set(SCL);
        DELAY(1);
        i2c_clr(SDA);
        DELAY(2);
}

static void i2c_multi_stop_bit(void)
{
        /* clock the ack */
        DELAY(1); 
        i2c_clr(SCL);  

        /* actual stop bit */
        DELAY(1);
        i2c_clr(SDA);
        DELAY(1);
        i2c_set(SCL);
        DELAY(1);
        i2c_set(SDA);
        DELAY(1);
        DELAY(1); 
        i2c_clr(SCL);  
}

static void i2c_multi_send_byte(unsigned char c)
{
    int i;
    local_irq_disable();
    for (i=0; i<8; i++)
    {
        DELAY(1);
        i2c_clr(SCL);
        DELAY(1);

        if (c & (1<<(7-i)))
            i2c_set(SDA);
        else
            i2c_clr(SDA);

        DELAY(1);
        i2c_set(SCL);
        DELAY(1);
        i2c_clr(SCL);
    }
    DELAY(1);
   // i2c_set(SDA);
    local_irq_enable();
}


static unsigned char i2c_multi_data_read(void)
{
	unsigned char regvalue;
	
		
	regvalue = HW_REG(GPIO_I2C_SDA_REG);
	if((regvalue&SDA) != 0)
		return 1;
	else
		return 0;
}

static unsigned char i2c_multi_receive_byte(void)
{
    int j=0;
    int i;
    unsigned char regvalue;

    local_irq_disable();

    regvalue = HW_REG(GPIO_0_DIR);
    regvalue &= (~SDA);
    HW_REG(GPIO_0_DIR) = regvalue;

    for (i=0; i<8; i++)
    {
        DELAY(1);
        i2c_clr(SCL);

        DELAY(4);
        i2c_set(SCL);
        DELAY(2);
        
        
        if (i2c_multi_data_read())
            j+=(1<<(7-i));

        DELAY(1);
        i2c_clr(SCL);
    }

    local_irq_enable();
    DELAY(1);
   // i2c_clr(SDA);
   // DELAY(1);

    return j;
}

static int i2c_multi_receive_ack(void)
{
    int nack;
    unsigned char regvalue;
    
    DELAY(1);
    
    regvalue = HW_REG(GPIO_0_DIR);
    regvalue &= (~SDA);
    HW_REG(GPIO_0_DIR) = regvalue;
    
    DELAY(1);
    i2c_clr(SCL);
    DELAY(1);
    i2c_set(SCL);
    DELAY(1);
    
    

    nack = i2c_data_read();

    DELAY(1);
    i2c_clr(SCL);
    DELAY(1);
  //  i2c_set(SDA);
  //  DELAY(1);

    if (nack == 0)
        return 1; 

    return 0;
}




static void i2c_multi_send_ack(void)
{
    DELAY(1);
    i2c_clr(SCL);
    DELAY(1);
    i2c_clr(SDA);
    DELAY(1);
    i2c_set(SCL);
    DELAY(1);
    i2c_clr(SCL);
    DELAY(1);
    i2c_clr(SDA);
    DELAY(1);
       
}

#if 0
static void i2c_multi_send_nack(void)
{
//unsigned char regvalue;
    DELAY(1);
    i2c_clr(SCL);
    DELAY(1);
    i2c_set(SDA);
//	i2c_clr(SDA);
    DELAY(1);
    i2c_set(SCL);
    DELAY(1);
    i2c_clr(SCL);
    DELAY(1);
//    i2c_cx838_clr(SDA);
    DELAY(1);
}
#endif

#endif

EXPORT_SYMBOL(gpio_i2c_read);
unsigned char gpio_i2c_read(unsigned char devaddress, unsigned char address)
{
    int rxdata;
    
    spin_lock(&gpioi2c_lock);
    
    i2c_start_bit();
    i2c_send_byte((unsigned char)(devaddress));
    i2c_receive_ack();
    i2c_send_byte(address);
    i2c_receive_ack();   
    i2c_start_bit();
    i2c_send_byte((unsigned char)(devaddress) | 1);
    i2c_receive_ack();
    rxdata = i2c_receive_byte();
    //i2c_send_ack();
    i2c_stop_bit();
    
    spin_unlock(&gpioi2c_lock);

    return rxdata;
}


EXPORT_SYMBOL(gpio_i2c_write);
int gpio_i2c_write(unsigned char devaddress, unsigned char address, unsigned char data)
{
	int ret=0;
	spin_lock(&gpioi2c_lock);
		
    i2c_start_bit();
    i2c_send_byte((unsigned char)(devaddress));
    ret = i2c_receive_ack();
	//if (ret){
//		printk("---i2c device addr[0x%x] no ack.\n",devaddress);
//		spin_unlock(&gpioi2c_lock);
//		return 0;
//	}
//	
    i2c_send_byte(address);
    ret = i2c_receive_ack();
//	if (ret){
//		printk("---i2c device addr[0x%x] reg[0x%x] no ack.\n",devaddress,address);
//		spin_unlock(&gpioi2c_lock);
//		return 0;
//	}
	
    i2c_send_byte(data); 
   // ret = i2c_receive_ack();//add by hyping for tw2815
//    if (ret){
//		printk("---i2c device addr[0x%x] val[0x%x] no ack.\n",devaddress,data);
//		spin_unlock(&gpioi2c_lock);
//		return 0;
//	}
   
    i2c_stop_bit();
    
    spin_unlock(&gpioi2c_lock);

	return 1;
}

EXPORT_SYMBOL(gpio_i2c_multitvt_read);
void gpio_i2c_multitvt_read(unsigned char devaddress, unsigned char address, unsigned int *rxbuf, int  Nbytes)
{
	
    //int rxdata;
    int i;
    
    i2c_multi_start_bit();
    i2c_multi_send_byte((unsigned char)(devaddress));
    
    i2c_multi_receive_ack();
    i2c_multi_send_byte(address);
    
    i2c_multi_receive_ack();   
    i2c_multi_start_bit();
    i2c_multi_send_byte((unsigned char)(devaddress) | 1);
    
    i2c_multi_receive_ack(); 
    
    for(i=0; i<Nbytes-1; i++)
    {
        rxbuf[i] = i2c_multi_receive_byte();
		    i2c_multi_send_ack();
    }
    rxbuf[i] = i2c_multi_receive_byte();
    i2c_multi_stop_bit();
    
    
    return;
}


EXPORT_SYMBOL(gpio_i2c_multitvt_write);
void gpio_i2c_multitvt_write(unsigned char devaddress, unsigned char address, unsigned int *rxbuf, int  Nbytes)
{
		int i;
		
    i2c_multi_start_bit();
    i2c_multi_send_byte((unsigned char)(devaddress));
    i2c_multi_receive_ack();
    i2c_multi_send_byte(address);
    i2c_multi_receive_ack();

    for(i=0; i<Nbytes-1; i++)
    {
    	i2c_multi_send_byte( (unsigned char)rxbuf[i] );
    	i2c_multi_receive_ack(); 
    }
    i2c_multi_send_byte( (unsigned char)rxbuf[i] );
    i2c_multi_stop_bit();

}

spinlock_t* gpio_i2c_lock_addr(void);

EXPORT_SYMBOL(gpio_i2c_lock_addr);
spinlock_t* gpio_i2c_lock_addr(void)
{
	return &gpioi2c_lock;
}
int __init gpio_i2c_pin_init(void)
{
    int ret;

    printk( "---gpio_i2c version : 20160805\n" );

	//init multiple pin
	HW_REG(GPIO_CRTL_SDA2_ADDR_REG) = 0;
	HW_REG(GPIO_CRTL_SCL2_ADDR_REG) = 0;
 
    //set SCL and SDA
    i2c_set(SCL | SDA);

	spin_lock_init(&gpioi2c_lock);

    return 0;    
}



