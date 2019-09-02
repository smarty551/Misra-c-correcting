/*
 * BLMGR_CFG.h
 *
 * Created: 28/02/2016 06:54:32 م
 *  Author: hossam
 */ 


#ifndef BLMGR_CFG_H_
#define BLMGR_CFG_H_

#include "Basic_Types.h"
typedef struct  
{
	u8 Portname;
	u8 PortMask;
}BLMGR_DioPinConfig;

#define BLOUETOOTH_ON 0xff
#define BUZEER_ON     0xff

 extern BLMGR_DioPinConfig BuzzerConfig;
 extern BLMGR_DioPinConfig BlueToothPwrConfig;
 extern BLMGR_DioPinConfig BluetoothKeyConfig;
 

#endif /* BLMGR_CFG_H_ */