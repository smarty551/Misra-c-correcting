/*
 * BLMGR_CFG.c
 *
 * Created: 28/02/2016 06:54:45 م
 *  Author: hossam
 */ 
#include "DIO.h"
#include "BLMGR_CFG.h"
BLMGR_DioPinConfig BuzzerConfig      = {PC,(1u)<<(4u)};
BLMGR_DioPinConfig BlueToothPwrConfig = {PC,(1u)<<(5u)};
BLMGR_DioPinConfig BluetoothKeyConfig = {PD,(1u)<<(2u)};
	
