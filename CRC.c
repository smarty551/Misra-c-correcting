#include "Basic_Types.h"
#include <stdlib.h>
/***************************************************************************************/
static u32 GetPower(u32 Base,u32 Pow);
static u16 gen_crc16(const u8 data[], u16 size, u32 CRC16);
/***************************************************************************************/
void SECR_CrcPolynomialGenerate(u32* PolynomialPtr,u8 CrcLengthInBits);
void SECR_CrcPolynomialGenerate(u32* PolynomialPtr,u8 CrcLengthInBits)
{
	u32 DevisorValue;
	DevisorValue = (u32)(GetPower(((u32)2),((u32)CrcLengthInBits))) - 1U;
	*PolynomialPtr = ((u32)rand() % DevisorValue) + (u32)0x10000U ;
}
/***************************************************************************************/
void SECR_GnerateCrc(const u8 PayloadPtr[],u16 PayloadLength, u16* CrcPtr, u32 CrcPoly);

void SECR_GnerateCrc(const u8 PayloadPtr[],u16 PayloadLength, u16* CrcPtr, u32 CrcPoly)
{
	u16 LoopIndex;
	static u8 InternalBuffer[8U];
	/*Copying data to internal buffer*/
	for (LoopIndex = 0U; LoopIndex < PayloadLength; LoopIndex ++)
	{
		InternalBuffer[LoopIndex] = PayloadPtr [LoopIndex];
	}
	/*perform bit wise invert on the data*/
	for (LoopIndex = 0U; LoopIndex < PayloadLength; LoopIndex ++)
	{
		InternalBuffer[LoopIndex]  ^= 0xffU;
	}
	/*Generate CRC*/
	*CrcPtr = gen_crc16(InternalBuffer,PayloadLength*8U,0x18005U);
}
/***************************************************************************************/
static u32 GetPower(u32 Base,u32 Pow)
{
	u32 result = 1;
	u32 LoopIndexh=0;
	for (LoopIndexh = 0U; LoopIndexh < Pow; LoopIndexh ++)
	{
		result *= Base;
	}
	return result;
}
/***************************************************************************************/
static u16 gen_crc16(const u8 data[], u16 size, u32 CRC16)
{
	u16 out = 0;
	u16 bits_read = 0, bit_flag;
	u16 i;
	u16 crc = 0;
	u16 j = 0x0001;
	u16 indexer=0;
	/* Sanity check: */
	if(data > 0)
	{


	while(size > 0U)
	{
		bit_flag = out >> 15;

		/* Get next bit: */
		out <<= 1;
		out |= ((data[indexer] >> bits_read) & 1U); /* item a) work from the least significant bits*/

		/* Increment bit counter: */
		bits_read++;
		if(bits_read > 7U)
		{
			bits_read = 0U;
			indexer++;
			size--;
		}

		/* Cycle check: */
		if(bit_flag){
		out ^= CRC16;
		}
		else{

		}
	}

	/* item b) "push out" the last 16 bits*/

	for (i = 0U; i < 16U; ++i) {
		bit_flag = out >> 15;
		out <<= 1;
		if(bit_flag){
		out ^= CRC16;
		}
		else{}
	}

	/* item c) reverse the bits*/



	for (i = 0x8000U; i != 0U; i >>=1U) {
	     j <<= 1U;
	    if (i & out){
		    crc |= j;
		}


		else {}
	}
}
	else{
	    crc=0U;
	}
	return crc;
}
/***************************************************************************************/
