/*
 * UART_LCFG.h
 *
 * Created: 26/07/2015 06:33:52 م
 *  Author: hossam
 */ 


#ifndef UART_LCFG_H_
#define UART_LCFG_H_
#include "Basic_Types.h"

typedef struct 
{
	u32 BaudRate;
	u8 Parity;
	u8 stopBitSetting;
}UART_tConfig ;



extern const UART_tConfig UartConfig ;



#endif /* UART_LCFG_H_ */