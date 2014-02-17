#ifndef __ZT2083_DRV_H__
#define __ZT2083_DRV_H__

#include "zt2083_i2c.h"


uint8 ZT2083_SendCmd(uint8 cmd, uint8 numBytes);
uint8 ZT2083_ReadData(  uint8 *data, uint8 numBytes);
extern void zt2083Touch_PowerOnInit(void);
extern uint16_t   zt2083HWTouchGetPixelPoint(void );
#endif

