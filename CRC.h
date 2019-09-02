/*
 * CRC.h
 *
 * Created: 28/02/2016 04:27:41 م
 *  Author: hossam
 */ 


#ifndef CRC_H_
#define CRC_H_
#include "Basic_Types.h"
void SECR_CrcPolynomialGenerate(u32* PolynomialPtr,u8 CrcLengthInBits);
void SECR_GnerateCrc(const u8* PayloadPtr,u16 PayloadLength, u16* CrcPtr, u32 CrcPoly);



#endif /* CRC_H_ */