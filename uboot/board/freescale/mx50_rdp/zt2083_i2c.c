//#if defined(CONFIG_ZT2083_CHARGEMEASURE)

#include "zt2083_i2c.h"


void zt2083_setGPIO_input(void)
{ 
      unsigned int reg;
	//gpio_direction_input(GPIO);
	reg = readl(GPIO4_BASE_ADDR + 0x4); 
	reg &= ~(1<<11);	
	writel(reg, GPIO4_BASE_ADDR + 0x4);
	//reg = readl(GPIO4_BASE_ADDR + 0x0); 
	//reg &= ~(1<<24);
	//writel(reg, GPIO4_BASE_ADDR + 0x0); 
}
void zt2083_setGPIO_output(void)
{
	unsigned int reg;
	reg = readl(GPIO4_BASE_ADDR + 0x4); 
	reg |= (1<<11);	
	writel(reg, GPIO4_BASE_ADDR + 0x4);
	//set high 
	//reg = readl(GPIO4_BASE_ADDR + 0x0); 
	//reg |= (1<<24);
	//writel(reg, GPIO4_BASE_ADDR + 0x0); 

}


uint32_t zt2083_GPIO_Read(uint32  GPIO)
{
	//uint32 High_Low=0;
	unsigned int reg;
	
	reg = readl(GPIO4_BASE_ADDR + 0x0); 
	//reg &= ~(1<<24);
	//writel(reg, GPIO4_BASE_ADDR + 0x0); 
	if(reg&(1<<11))
	return 1;
	else
	return 0;
}
void gpio_set_value(unsigned gpio, int value)
{
	unsigned int reg;
	if(gpio==SDA)	
	{
	    if(value)
	      {
	              //set high 
		      reg = readl(GPIO4_BASE_ADDR + 0x0); 
		      reg |= (1<<11);
		       writel(reg, GPIO4_BASE_ADDR + 0x0); 
		}
		else
		{
                    //set low 
		      reg = readl(GPIO1_BASE_ADDR + 0x0); 
		      reg &= ~(1<<11);	
		      writel(reg, GPIO4_BASE_ADDR + 0x0); 
		}
	}
	else  //scl
	{
	    if(value)
	    {
	    	//set high 
		reg = readl(GPIO4_BASE_ADDR + 0x0); 
		reg |= (1<<8);
		writel(reg, GPIO4_BASE_ADDR + 0x0); 
	    }
	   else
	   {     //set low
		   reg = readl(GPIO4_BASE_ADDR + 0x0); 
		   reg &= ~(1<<8);
		   writel(reg, GPIO4_BASE_ADDR + 0x0); 
	   }
	}
}
void zt2083_SerialCommStart(void) /* start or re-start */
{
	zt2083_setGPIO_output();//GPIO_InitIO(1,SDA);
	gpio_set_value(SDA,1) ;//GPIO_WriteIO(1,SDA);
	gpio_set_value(SCL,1) ;//GPIO_WriteIO(1,SCL);
	udelay(1);
	gpio_set_value(SDA,0) ;//GPIO_WriteIO(0,SDA);   
	udelay(1);
	gpio_set_value(SCL,0) ;//GPIO_WriteIO(0,SCL);
	udelay(1);
}

void zt2083_SerialCommStop(void)
{
	gpio_set_value(SCL,0) ;//GPIO_WriteIO(0,SCL);
	udelay(2);
	zt2083_setGPIO_output();//GPIO_InitIO(1,SDA);
	gpio_set_value(SDA,0) ;//GPIO_WriteIO(0,SDA);
	udelay(1);
	gpio_set_value(SCL,1) ;//GPIO_WriteIO(1,SCL);
	udelay(1);
	gpio_set_value(SDA,1) ;//GPIO_WriteIO(1,SDA); /* stop condition */
	udelay(2);
}

uint8 zt2083_SerialCommTxByte(uint8 data) /* return 0 --> ack */
{
	int32 i;
	uint8 temp_value = 0;
	for(i=7; (i>=0)&&(i<=7); i--){
		gpio_set_value(SCL,0) ;//GPIO_WriteIO( 0, SCL); /* low */
		udelay(2);
		if(i==7) zt2083_setGPIO_output();//GPIO_InitIO(1,SDA);
		udelay(2);

		gpio_set_value(SDA,((data>>i)&0x01)); //GPIO_WriteIO(((data>>i)&0x01), SDA);
		udelay(1);
		gpio_set_value(SCL,1) ;//GPIO_WriteIO( 1, SCL); /* high */
		udelay(4);
	}
	gpio_set_value(SCL,0) ;//GPIO_WriteIO(0, SCL); /* low */
	udelay(2);
	//setGPIO_input(SDA);//GPIO_InitIO(0,SDA);/* input  */
	udelay(1);	
	gpio_set_value(SCL,1) ;// GPIO_WriteIO(1, SCL); /* high */
	udelay(1);	

	temp_value = zt2083_GPIO_Read(SDA);//GPIO_ReadIO(SDA);
	return temp_value;
}

void zt2083_SerialCommRxByte(uint8 *data, uint8 ack)
{
	int32 i;
	uint32 dataCache;

	dataCache = 0;
	for(i=7; (i>=0)&&(i<=7); i--){
		gpio_set_value(SCL,0) ;
		udelay(2);		
		if(i==7)zt2083_setGPIO_input();
		udelay(2);	
		gpio_set_value(SCL,1) ;
		udelay(1);			
		dataCache |= (zt2083_GPIO_Read(SDA)<<i);
		udelay(1);			
	}

	gpio_set_value(SCL,0) ;
	udelay(2);
	zt2083_setGPIO_output();
	gpio_set_value(SDA,ack) ;  //回ack信号
	udelay(1);
	gpio_set_value(SCL,1) ;
	udelay(1);	
	*data = (uint8)dataCache;//data为读走的数，存在datacache中
}

int zt2083_gpio_init(void)
{	
        unsigned int reg;
   	//SDA  gpio4_11   
	mxc_request_iomux(MX50_PIN_CSPI_SS0,
			IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX50_PIN_CSPI_SS0,
			PAD_CTL_HYS_NONE
			| PAD_CTL_PKE_NONE 
			| PAD_CTL_DRV_HIGH );
	reg = readl(GPIO4_BASE_ADDR + 0x4); 
	reg |= (1<<11);	
	writel(reg, GPIO4_BASE_ADDR + 0x4);
	//set high 
	reg = readl(GPIO4_BASE_ADDR + 0x0); 
	reg |= (1<<11);
	writel(reg, GPIO4_BASE_ADDR + 0x0); 

// SCL GPIO4_8
	mxc_request_iomux(MX50_PIN_CSPI_SCLK,
			IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX50_PIN_CSPI_SCLK,
			PAD_CTL_HYS_NONE
			| PAD_CTL_PKE_NONE 
			| PAD_CTL_DRV_HIGH );
	reg = readl(GPIO4_BASE_ADDR + 0x4); 
	reg |= (1<<8);	
	writel(reg, GPIO4_BASE_ADDR + 0x4);
		
	//set high 
	reg = readl(GPIO4_BASE_ADDR + 0x0); 
	reg |= (1<<8);
	writel(reg, GPIO4_BASE_ADDR + 0x0); 
}

//#endif
