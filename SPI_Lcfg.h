/*
 * SPI_Lcfg.h
 *
 * Created: 19/09/2015 11:01:20 ص
 *  Author: hossam
 */ 


#ifndef SPI_LCFG_H_
#define SPI_LCFG_H_
#include "Basic_Types.h"
#include "DIO.h"

#define u8MODE_0 0x00
#define u8MODE_1 0x04
#define u8MODE_2 0x08
#define u8MODE_3 0x0C

#define u8DOR_MSB 0x00
#define u8DOR_LSB 0x20


typedef struct  
{
	u32 u32MaxFreq;
	u8 u8Mode;
	u8 u8DataOrder;
}SPI_ConfigType;
extern const SPI_ConfigType SPI_InitConfig;



#endif /* SPI_LCFG_H_ */