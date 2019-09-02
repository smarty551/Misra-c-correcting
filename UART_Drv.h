/*
 * UART_Drv.h
 *
 * Created: 26/07/2015 05:24:02 م
 *  Author: hossam
 */ 


#ifndef UART_DRV_H_
#define UART_DRV_H_
#include "Basic_Types.h"
typedef void (*tCbkFn)(void);
void UART_Init(void);
void UART_TxBuffer(const u8* Buffer,u16 BufferLength);
void UART_StartReception(u8* Buffer,u16 BufferLength,tCbkFn RxcCbkFnPtr);
u16 UART_GetNumOfRxbytes(void);
void UART_StopRception(void);





#endif /* UART_DRV_H_ */