
/***********************************************************************************
 *#defines
 ***********************************************************************************/
#include <linux/gpio.h>
#include <linux/delay.h>
#include "../../../arch/arm/mach-mx5/mx50_io_cfg.h"

//kernel_imx_imx508/drivers/hwmon/encrypt-gpio
//kernel_imx_imx508/arch/arm/mach-mx5/mx50_io_cfg.h
extern  int i2c_encrypt_setoutput(void);
extern  int i2c_encrypt_setinput(void);

//#define MX50_RDP_UART1_CTS_GPIO6_8_ENCRYPT_SDA  (5*32+8)
///#define MX50_RDP_EPITO_GPIO6_27_ENCRYPT_SCL    (5*32+27)

#define ENCRYPT_SDA  MX50_RDP_UART1_CTS_GPIO6_8_ENCRYPT_SDA
#define ENCRYPT_SCL   MX50_RDP_EPITO_GPIO6_27_ENCRYPT_SCL

#define I2C_DELAY		udelay(5)/*5*/
#define I2C_DELAY_LONG	udelay(100)/*100*/

#define SCL_LOW			gpio_set_value(ENCRYPT_SCL,0)  /*PORTC &= 0xF7*/
#define SCL_HIGH		gpio_set_value(ENCRYPT_SCL,1) /*PORTC &= 0xF7*/
#define SDA_IN			gpio_direction_input(ENCRYPT_SDA)
#define SDA_OUT		       gpio_direction_output(ENCRYPT_SDA,1)
#define SDA_LOW			gpio_set_value(ENCRYPT_SDA,0) /*PORTC &= 0xF7*/
#define SDA_HIGH		gpio_set_value(ENCRYPT_SDA,1)  /*PORTC &= 0xF7*/
#define SDA_DETECT		gpio_get_value(ENCRYPT_SDA)  /*PORTC &= 0xF7*/
 
#define ERROR_CODE_TRUE			0
#define ERROR_CODE_FALSE		1
#define ERROR_CODE_WRITE_ADDR	10
#define ERROR_CODE_WRITE_DATA	20
#define ERROR_CODE_READ_ADDR	30
#define ERROR_CODE_READ_DATA	40
#define ERROR_CODE_START_BIT	50
#define ERROR_CODE_APROCESS		60
#define ERROR_CODE_DENY			70

/***********************************************************************************
 * Variables
 ***********************************************************************************/

/***********************************************************************************
 * External Variables
 ***********************************************************************************/

/***********************************************************************************
 * Function Prototypes
 ***********************************************************************************/
void _i2c_start(void);
void _i2c_stop(void);
void _i2c_ack_send(void);
unsigned char _i2c_ack_detect(void);
unsigned char _i2c_write_byte(unsigned char);
unsigned char _i2c_read_byte(void);
unsigned char _i2c_write(unsigned char , unsigned char, unsigned char *, int);
unsigned char _i2c_read(unsigned char , unsigned char, unsigned char *, int);
  
/***********************************************************************************
 * External Function
 ***********************************************************************************/

/* EOF */

