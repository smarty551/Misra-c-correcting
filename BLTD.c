/*
 * BLTD.c
 *
 * Created: 31/07/2015 12:35:22 ص
 *  Author: hossam
 */ 
#include "Basic_Types.h"
#include "UART_Drv.h"
#include "BLTD.h"

static u8 IsRespRecieved = 0;
static u8 RxBuffer[100];
static void BTCommandSend(const u8* Command,u16 CommandLength);
static  void MemCpy(u8 Des[],const u8 Src[],u16 Length);
static void RxcCallBackFun(void);
static u8 MemCmp(const u8  Src1[],const u8 Src2[],u16 CmpLength);
/***************************************************************************************************************/
void BLTD_SendInitCmd(void)
{	IsRespRecieved = 0U;
	UART_StartReception(RxBuffer,4U,&RxcCallBackFun);
	BTCommandSend((u8*)"+INIT",5U);
}
/***************************************************************************************************************/
void BLTD_SendInquireCmd(void)
{	IsRespRecieved = 0U;
	UART_StartReception(RxBuffer,4U,&RxcCallBackFun);
	BTCommandSend((u8*)"+INQ",4U);
}
u8 BLTD_CheckForResponse(const u8* Response,u16 RespLength)
{
	u8 RespStatus;
	u8 IsEqual;
	if(IsRespRecieved == 1U)
	{
		IsRespRecieved = 0U;
		IsEqual = MemCmp(Response,RxBuffer,RespLength);
		if(IsEqual == 0U)
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
	UART_StartReception(RxBuffer,4U,&RxcCallBackFun);
	/*BTCommandSend(0,0);*/
	
}
/***************************************************************************************************************/	
void BLTD_SendMessage(const u8* Message,u16 MsgLength)
{
	UART_TxBuffer(Message,MsgLength);
}	
/***************************************************************************************************************/
u8 BLTD_GetRecievedData( u8 Data[], u16 Length)
{
	u8 RespStatusr;
	u8 i;
	if(IsRespRecieved == 1U)
	{
		IsRespRecieved = 0U;
		RespStatusr = BLTD_RESP_STATUS_OK;
		for( i = 0U; i< Length ; i++)
		{
			Data[i] = RxBuffer[i];
		}
	}
	else
	{
		RespStatusr = BLTD_RESP_STATUS_NON;
	}		
		
	return RespStatusr;
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
	if(RxBytesNum > 0U)
	{
		IsReceived = 0x01U;
		*Data = RxBuffer[RxBytesNum+1U];
		UART_StopRception();
		UART_StartReception(RxBuffer,(u16)100U,&RxcCallBackFun);
	}
	else
	{
		IsReceived = 0x00U;
		*Data  = 0U;
	}
	return IsReceived;
}

/***************************************************************************************************************/	
void BLTD_SenTestCmd(void)
{
	UART_StartReception(RxBuffer,(u16)4U,&RxcCallBackFun);
	BTCommandSend((u8 *)0U,(u16)0U);
	
}
/***************************************************************************************************************/
static void BTCommandSend(const u8* Command,u16 CommandLength)
	{
     u8 BTCommandBuffer[100];
		BTCommandBuffer[0] = (u8)'A';
		BTCommandBuffer[1] = (u8)'T';
		MemCpy(&BTCommandBuffer[2],Command,CommandLength);
		BTCommandBuffer[CommandLength+2U] = 0x0dU;
		BTCommandBuffer[CommandLength+3U] = 0x0aU;
		UART_TxBuffer(BTCommandBuffer,CommandLength + 4U);
		
	}
/***************************************************************************************************************/
static  void MemCpy(u8 Des[],const u8 Src[],u16 Length)
	{
	u16 ii;
	for(ii = 0U ; ii<Length ; ii++)
		{
		Des[ii] = Src[ii];
		}
	}
/***************************************************************************************************************/	
static void RxcCallBackFun(void)
{

	IsRespRecieved = 1U;
}
/***************************************************************************************************************/
static u8 MemCmp(const u8  Src1[],const u8 Src2[],u16 CmpLength)
	{
	u8 RetVal = 0;
	u16 iii;
	for(iii = 0U ;(iii < CmpLength); iii++)
		{
		if(Src1[iii] != Src2[iii])
			{
			RetVal = 1U;
			break;
			}
		}
	return RetVal;
	}
/***************************************************************************************************************/
