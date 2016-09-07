#ifndef _GPIO_I2C_H
#define _GPIO_I2C_H

#define GPIO_I2C_READ   0x01
#define GPIO_I2C_WRITE  0x02

#define GPIO_I2C_MULTVT_READ   0x11
#define GPIO_I2C_MULTVT_WRITE  0x12

#define GPIO_I2C_READ16   0x03
#define GPIO_I2C_WRITE16  0x04

int __init gpio_i2c_pin_init(void);
unsigned char gpio_i2c_read(unsigned char devaddress, unsigned char address);
int gpio_i2c_write(unsigned char devaddress, unsigned char address, unsigned char data);

void gpio_i2c_multitvt_read(unsigned char devaddress, unsigned char address, unsigned int *rxbuf, int  Nbytes);
void gpio_i2c_multitvt_write(unsigned char devaddress, unsigned char address, unsigned int *rxbuf, int  Nbytes);

#endif

