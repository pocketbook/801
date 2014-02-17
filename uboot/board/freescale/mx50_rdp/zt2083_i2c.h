#ifndef __ZT2083_I2C_H__
#define __ZT2083_I2C_H__

//#include <linux/gpio.h>
//#include <linux/delay.h>
#include <asm/io.h>
#include <asm/arch/mx50.h>
#include <asm/arch/mx50_pins.h>
#include <asm/arch/iomux.h>  


#define uint8  unsigned char
#define int32 signed int
#define uint16  unsigned int
#define int16   unsigned  int
#define uint32  unsigned  int
#define int8  unsigned char

#define SDA    	1/*(3*32 + 24)	, GPIO_4_24 MX51_PAD_CSPI1_SS0__GPIO4_24 */
#define SCL    	0/*(3*32 + 27)	, GPIO_4_27 MX51_PAD_CSPI1_SCLK__GPIO4_27*/


void zt2083_SerialCommStart(void); 
void zt2083_SerialCommStop(void);
unsigned char zt2083_SerialCommTxByte(unsigned char data);
void zt2083_SerialCommRxByte(unsigned char *data, unsigned char ack);
int zt2083_gpio_init(void);

#endif

