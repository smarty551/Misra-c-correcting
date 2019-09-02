/*
 * BLMGR_CFG.c
 *
 * Created: 28/02/2016 06:54:45 م
 *  Author: hossam
 */ 
#include "DIO.h"
#include "BLMGR_CFG.h"
BLMGR_DioPinConfig BuzzerConfig      = {PC,1<<4};
BLMGR_DioPinConfig BlueToothPwrConfig = {PC,1<<5};
BLMGR_DioPinConfig BluetoothKeyConfig = {PD,1<<2};	
	