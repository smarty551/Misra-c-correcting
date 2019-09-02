/*
 * BLTD.c
 *
 * Created: 31/07/2015 12:35:22 ص
 *  Author: hossam
 */ 
#include "Basic_Types.h"
#include "UART_Drv.h"
#include "BLTD.h"

static u8 BTCommandBuffer[100];
static u8 IsRespRecieved = 0;
static u8 RxBuffer[100];
static void BTCommandSend(u8* Command,u16 CommandLength);
static  void MemCpy(u8 *Des,const u8 *Src,u16 Length);
static void RxcCallBackFun(void);
static u8 MemCmp(const u8 * Src1,const u8 *Src2,u16 CmpLength);
/***************************************************************************************************************/
void BLTD_SendInitCmd(void)
{	IsRespRecieved = 0;
	UART_StartReception(RxBuffer,4,RxcCallBackFun);
	BTCommandSend((u8*)"+INIT",5);
}
/***************************************************************************************************************/
void BLTD_SendInquireCmd(void)
{	IsRespRecieved = 0;
	UART_StartReception(RxBuffer,4,RxcCallBackFun);
	BTCommandSend((u8*)"+INQ",4);
}
u8 BLTD_CheckForResponse(u8* Response,u16 RespLength)
{
	u8 RespStatus;
	u8 IsEqual;
	if(IsRespRecieved == 1)
	{
		IsRespRecieved = 0;
		IsEqual = MemCmp(Response,RxBuffer,RespLength);
		if(IsEqual == 0)
		{
			RespStatus = BLTD_RESP_STATUS_OK;
			
		}
		else
		{
			RespStatus = BLTD_RESP_STATUS_NOK;
		}
		
	}
	else
	{

		RespStatus = BLTD_RESP_STATUS_NON;
	}
	return RespStatus;
}	
/***************************************************************************************************************/
void BLTD_StartWaitPairing(void)
{
	UART_StopRception();
	UART_StartReception(RxBuffer,4,RxcCallBackFun);
	//BTCommandSend(0,0);
	
}
/***************************************************************************************************************/	
void BLTD_SendMessage(u8* Message,u16 MsgLength)
{
	UART_TxBuffer(Message,MsgLength);
}	
/***************************************************************************************************************/
u8 BLTD_GetRecievedData(u8*Data, u16 Length)
{
	u8 RespStatus;
	u8 i;
	if(IsRespRecieved == 1)
	{
		IsRespRecieved = 0;
		RespStatus = BLTD_RESP_STATUS_OK;
		for( i = 0; i< Length ; i++)
		{
			Data[i] = RxBuffer[i];
		}
	}
	else
	{
		RespStatus = BLTD_RESP_STATUS_NON;
	}		
		
	return RespStatus;
}
/***************************************************************************************************************/
void BLTD_StartReceivingData(u8* DataBuffer,u16 BufferLength,CbkPfnType CbkFnPtr)
{
	UART_StartReception(DataBuffer,BufferLength,CbkFnPtr);
	
}
/***************************************************************************************************************/
u8 BLTD_CheckForData(u8* Data)
{
	u16 RxBytesNum;
	u8 IsReceived;
	RxBytesNum = UART_GetNumOfRxbytes();
	if(RxBytesNum > 0)
	{
		IsReceived = 0x01;
		*Data = RxBuffer[RxBytesNum+1];
		UART_StopRception();
		UART_StartReception(RxBuffer,100,RxcCallBackFun);
	}
	else
	{
		IsReceived = 0x00;
		*Data  = 0;
	}
	return IsReceived;
}

/***************************************************************************************************************/	
void BLTD_SenTestCmd(void)
{
	UART_StartReception(RxBuffer,4,RxcCallBackFun);
	BTCommandSend(0,0);
	
}
/***************************************************************************************************************/
static void BTCommandSend(u8* Command,u16 CommandLength)
	{
		BTCommandBuffer[0] = 'A';
		BTCommandBuffer[1] = 'T';
		MemCpy(&BTCommandBuffer[2],Command,CommandLength);
		BTCommandBuffer[CommandLength+2] = 0x0d;
		BTCommandBuffer[CommandLength+3] = 0x0a;
		UART_TxBuffer(BTCommandBuffer,CommandLength + 4);
		
	}
/***************************************************************************************************************/
static  void MemCpy(u8 *Des,const u8 *Src,u16 Length)
	{
	u16 i;
	for(i = 0 ; i<Length ; i++)
		{
		*(Des+i) = *(Src+i);
		}
	}
/***************************************************************************************************************/	
static void RxcCallBackFun(void)
{

	IsRespRecieved = 1;
}
/***************************************************************************************************************/
static u8 MemCmp(const u8 * Src1,const u8 *Src2,u16 CmpLength)
	{
	u8 RetVal = 0;
	u16 i;
	for(i = 0 ;(i < CmpLength); i++)
		{
		if(Src1[i] != Src2[i])
			{
			RetVal = 1;
			break;
			}
		}
	return RetVal;
	}
/***************************************************************************************************************/
