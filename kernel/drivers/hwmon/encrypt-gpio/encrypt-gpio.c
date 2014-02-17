/************************************************************
Alpu-M Bypass Code v1.0
Test reference : Atmega128L
************************************************************/
#include <linux/delay.h>
#include <linux/gpio.h>
#include "encrypt-gpio.h"


#define printkMODE	0

#define DEVICE_ADDRESS	0x7a

unsigned char alpum_tx_data[8];
unsigned char alpum_rx_data[10];
unsigned char alpum_ex_data[8];



void _alpu_delay_ms(int i);
unsigned char _alpu_rand(void);

void _alpu_delay_ms(int i)
{
	udelay(i);
}

unsigned char _alpu_rand(void)
{
  static unsigned long seed; // 2byte, must be a static variable
  static unsigned char random=0;

 random = random * 17 + 41;
  seed = seed + random; // rand(); <------------------ add time value
  seed =  seed * 1103515245 + 12345;

  return (seed/65536) % 32768;
}


void _i2c_start(void)
{
	#if printkMODE
	printk("\r\n _i2c_start");
	#endif
	SDA_HIGH;
	I2C_DELAY;
	I2C_DELAY;
	SCL_HIGH;
	I2C_DELAY;
	I2C_DELAY;
	SDA_LOW;
	I2C_DELAY;
	SCL_LOW;
	I2C_DELAY;
}

void _i2c_stop(void)
{
	#if printkMODE
	printk("\r\n _i2c_stop");
	#endif
	SDA_LOW;
	I2C_DELAY;
	SCL_HIGH;
	I2C_DELAY;
	SDA_HIGH;
	I2C_DELAY_LONG;
}

unsigned char _i2c_write_byte(unsigned char data)
{
	unsigned char i;

	#if printkMODE
	printk("\r\n _i2c_write_byte");
	#endif

	for(i = 0; i< 8; i++)
	{
		if( (data << i) & 0x80) SDA_HIGH;
		else SDA_LOW;
		I2C_DELAY;
		SCL_HIGH;
		I2C_DELAY;
		SCL_LOW;
		I2C_DELAY;
	}

	if(_i2c_ack_detect()) {
		#if printkMODE
		printk("\n\r Acknoledge Error");
		#endif
		return ERROR_CODE_FALSE;
	}
	return ERROR_CODE_TRUE;
}

unsigned char _i2c_read_byte(void)
{
	unsigned char i, data;

	#if printkMODE
	printk("\r\n _i2c_read_byte");
	#endif

	data = 0;
	//20081016
	//SDA_HIGH;
	//end
	SDA_IN;
	for(i = 0; i< 8; i++){
		data <<= 1;
		I2C_DELAY;
		SCL_HIGH;
		I2C_DELAY;
		if (SDA_DETECT) data |= 0x01;
		SCL_LOW;
		I2C_DELAY;
	}
	_i2c_ack_send();
	return data;
}

unsigned char _i2c_ack_detect(void)
{
	#if printkMODE
	printk("\r\n _i2c_ack_detect");
	#endif

	//SDA_HIGH;
	SDA_IN;	// SDA Input Mode
	I2C_DELAY;
	SCL_HIGH;
	I2C_DELAY;

	if (SDA_DETECT)
	{
		SDA_OUT;
		#if printkMODE
		printk("\r\n Ack Error!");
		#endif
		return ERROR_CODE_FALSE; // false
	}

	I2C_DELAY;
	SCL_LOW;
	SDA_OUT;
	return ERROR_CODE_TRUE; // true
}

void _i2c_ack_send(void)
{
	#if printkMODE
	printk("\r\n _i2c_ack_send");
	#endif

	SDA_OUT;
	SDA_LOW;
	I2C_DELAY;
	SCL_HIGH;
	I2C_DELAY;
	SCL_LOW;
	I2C_DELAY;
}

unsigned char _i2c_write(unsigned char device_addr, unsigned char sub_addr, unsigned char *buff, int ByteNo)
{
	unsigned char i;

	#if printkMODE
	printk("\r\n_i2c_write");
	#endif

	_i2c_start();
	I2C_DELAY;
	if(_i2c_write_byte(device_addr)) {
		_i2c_stop();
		#if printkMODE
		printk("\n\r Write Error - Addr");
		#endif
		return ERROR_CODE_WRITE_ADDR;
	}
	if(_i2c_write_byte(sub_addr)) {
		_i2c_stop();
		#if printkMODE
		printk("\n\r Write Error - Addr");
		#endif
		return ERROR_CODE_WRITE_ADDR;
	}
	for(i = 0; i<ByteNo; i++) {
		if(_i2c_write_byte(buff[i])) {
			_i2c_stop();
			#if printkMODE
			printk("\n\r Write Error - Data");
			#endif
			return ERROR_CODE_WRITE_DATA;
		}
	}
	I2C_DELAY;
	_i2c_stop();
	I2C_DELAY_LONG;
	return ERROR_CODE_TRUE;
}

unsigned char _i2c_read(unsigned char device_addr, unsigned char sub_addr, unsigned char *buff, int ByteNo)
{
	unsigned char i;

	#if printkMODE
	printk("\r\n_i2c_read");
	#endif
	_i2c_start();
	I2C_DELAY;
	if(_i2c_write_byte(device_addr)) {
		_i2c_stop();
		#if printkMODE
		printk("\n\r Write Error - Addr");
		#endif
		return ERROR_CODE_READ_ADDR;
	}
	if(_i2c_write_byte(sub_addr)) {
		_i2c_stop();
		#if printkMODE
		printk("\n\r Write Error - Addr");
		#endif
		return ERROR_CODE_READ_ADDR;
	}
	_i2c_start();
	I2C_DELAY;
	if(_i2c_write_byte(device_addr+1)) {
		_i2c_stop();
		#if printkMODE
		printk("\n\r Write Error - Addr");
		#endif
		return ERROR_CODE_READ_ADDR;
	}
	for(i = 0; i<ByteNo; i++) buff[i] = _i2c_read_byte();
	I2C_DELAY;
	I2C_DELAY_LONG;
	_i2c_stop();
	I2C_DELAY_LONG;
	return ERROR_CODE_TRUE;
}

/* EOF */




void _alpum_bypass(void)
{
	int i;
	
	for ( i=0; i<8; i++ ) alpum_ex_data[i] =alpum_tx_data[i] ^ 0x01 ;
}

unsigned char _alpum_process(void)
{
	unsigned char error_code;
	int i;

	for (i=0; i<8; i++) alpum_tx_data[i] =_alpu_rand();

	error_code = _i2c_write(DEVICE_ADDRESS, 0x80, alpum_tx_data, 8);
	if (error_code) 
		return error_code;

	_alpu_delay_ms(1);////1

	error_code = _i2c_read(DEVICE_ADDRESS, 0x80, alpum_rx_data, 10);
	if (error_code) 
		return error_code;
  
     //for (i=0; i<8; i++) alpum_rx_data[i] =0;
	 
	_alpum_bypass();
	
	for (i=0; i<8; i++) {
		if (alpum_ex_data[i] != alpum_rx_data[i]) return 60;
	}
	return 0;
}


 int i2c_encrypt_init(void)
{
	int ret = -1;
       static bool bInited=false;

	 if(!bInited)
	 {
	       bInited = true;
		ret = gpio_request(ENCRYPT_SCL, "encrypt i2c request");
		if (ret)
		{
		       printk(" gpio_request :ENCRYPT_SCL error \r\n");
			goto out_1;
		}

		ret = gpio_request(ENCRYPT_SDA, "encrypt i2c request");
		if (ret)
		{
			 printk(" gpio_request :ENCRYPT_SDA error \r\n");
			goto out_2;
		}
		gpio_direction_output(ENCRYPT_SDA,1);
		gpio_direction_output(ENCRYPT_SCL,1);

	 }
      return 0;

out_2:
	gpio_free(ENCRYPT_SCL);
out_1:
	return ret;
}


unsigned char  encypt_main(void)
{
	unsigned char result=0;
	//int i;
	//initial..
	//other functions..
      i2c_encrypt_init();

	result = _alpum_process();
	
	if((result == 10) ||( result == 20))
	   result = 0;

	printk("=====================================================================\n\r");
        printk(" Error_code : %d \r\n", result);
	//result = 20;
	if ( result)
        {
             printk(" Enc check Fail!!\n\r");
             return result;
        }
	else 
        {
             printk(" Enc check Success!!!\n\r");
             return result;
        }
	return result;
	//other functions..
}


int EncrypICCheck(int x)
{
	 if(encypt_main()==0)
	     return 65531;
          else return 0;
}
int EncrypICCheck1(int x)
{
	 if(encypt_main()==0)
	     return 24531;
        else return 0;
}
int EncrypICCheck2(int x)
{
	 if(encypt_main()==0)
	     return 43531;
         else return 0;
}
int EncrypICCheck3(int x)
{
	 if(encypt_main()==0)
	     return 53431;
	 else return 0;
}	
int EncrypICCheck4(int x)
{
	 if(encypt_main()==0)
	     return 53431;
	 else
	 	return 0;
}	

int EncrypICCheck5(int x)
{
	 if(encypt_main()==0)
	     return 63421;
	 else
	 	return 0;
}		
int EncrypICCheck6(int x)
{
	 if(encypt_main()==0)
	     return 63223;
 else return 0;
}		
int EncrypICCheck7(int x)
{
	 if(encypt_main()==0)
	     return 62523;
 else return 0;
}		

int EncrypICCheck8(int x)
{
	 if(encypt_main()==0)
	     return 63423;
 else return 0;
}
/*
EXPORT_SYMBOL(EncrypICCheck);
EXPORT_SYMBOL(EncrypICCheck1);
EXPORT_SYMBOL(EncrypICCheck2);
EXPORT_SYMBOL(EncrypICCheck3);
EXPORT_SYMBOL(EncrypICCheck4);
EXPORT_SYMBOL(EncrypICCheck5);
EXPORT_SYMBOL(EncrypICCheck6);
EXPORT_SYMBOL(EncrypICCheck7);
EXPORT_SYMBOL(EncrypICCheck8);
*/
/* EOF */

