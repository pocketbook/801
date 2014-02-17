//#if defined(CONFIG_ZT2083_CHARGEMEASURE)

#include "zt2083_drv.h"


#define ZT2083_IC
//================================================================================================
//ZT2083 I2C
#ifdef ZT2083_IC
#define ZT2083_CMD_PWRON    0xd0
#define ZT2083_CMD_READX     0xd0
#define ZT2083_CMD_READY     0xc0
uint8 ZT2083_SendCmd(uint8 cmd, uint8 numBytes)
{
	uint8 acknowledge=0;
	//2083

	zt2083_SerialCommStart();

	acknowledge = zt2083_SerialCommTxByte(0x90);
	acknowledge = zt2083_SerialCommTxByte(cmd); 			

	zt2083_SerialCommStop();

	return acknowledge;

}
uint8 ZT2083_ReadData(  uint8 *data, uint8 numBytes)
{
	uint8 j;
	uint8 acknowledge=0;
	udelay(2);
	zt2083_SerialCommStart();
	acknowledge = zt2083_SerialCommTxByte(0x91);


	for(j = 0; j < numBytes; j++, data++)
	{
		if(j == (numBytes -1))
			zt2083_SerialCommRxByte(data,1); 
		else
			zt2083_SerialCommRxByte(data,0); 
	}
	zt2083_SerialCommStop();

	return acknowledge;

}
#endif
/**************************************************************************
* 函数描述: 触摸屏上电初始化
* 入口参数: 无
* 出口参数: 无
* 返回值:	无
* Log	:
***************************************************************************/
void zt2083Touch_PowerOnInit(void)
{
     //printf("zt2083Touch_PowerOnInit\n");	 
     zt2083_gpio_init();
#ifdef ZT2083_IC
	ZT2083_SendCmd(ZT2083_CMD_PWRON,1);
#endif
}
/**************************************************************************
* 函数描述: 触摸屏获取像素点
* 入口参数: 
* 出口参数: 按下的情况下,输出新的触摸点坐标.
* 返回值:	TRUE:  触摸按下,
*			FALSE: 触摸弹起
* Log	:
***************************************************************************/
uint16_t  zt2083HWTouchGetPixelPoint( void)
{
	uint16 Temp;
	uint8 tmpbuf[2];
	
#ifdef ZT2083_IC
	ZT2083_SendCmd(ZT2083_CMD_READX,1);
	ZT2083_ReadData(tmpbuf,2);
	Temp =(tmpbuf[0]<<8) |tmpbuf[1];
	Temp >>= 4;	
	
	//printf("tmpbuf[0] = 0X%x,tmpbuf[1] = 0X%x,dc xtemp=0x%x bat=%d mv \n",tmpbuf[0],tmpbuf[1],Temp,Temp*2772*2/4096+350);
	return Temp*2772*2/4096+350;
#endif
}
/*----------------------------end of file-------------------------------------*/

//#endif

