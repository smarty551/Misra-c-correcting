/*
 * KEYPAD.c
 *
 * Created: 30/01/2016 06:38:37 م
 *  Author: hossam
 */ 
#include "DIO.h"

/*Local Symbols*/
#define KPD_COL_PORT PD
#define KPD_ROW_PORT PD
#define KPD_COL_MASK 0x70
#define KPD_ROW_MASK 0x0f
#define KPD_COL_PIN_NUM 4u
#define KPD_ROW_PIN_NUM 0u
/**************************************************/
#define KPD_COL_INIT() DIO_vidWritePortDirection(KPD_COL_PORT,KPD_COL_MASK,0x00)
#define KPD_ROW_INIT() DIO_vidWritePortDirection(KPD_ROW_PORT,KPD_ROW_MASK,0xff); \
                       DIO_vidWritePortData(KPD_ROW_PORT,KPD_ROW_MASK,0x00)

#define KPD_COL_READ(VALPTR) DIO_vidReadPortData(KPD_COL_PORT,KPD_COL_MASK,(VALPTR));\
                             *(VALPTR) = (*(VALPTR)) >> KPD_COL_PIN_NUM


#define KPD_ROW_WRITE(DATA) DIO_vidWritePortData(KPD_ROW_PORT,KPD_ROW_MASK,((DATA) << KPD_ROW_PIN_NUM))
 
static const unsigned char KeysLut[]= {'1' , '2' , '3' , '4' , '5' , '6','7' , '8' , '9','*' , '0' , '#'};
void KPD_Init(void)
{
	KPD_COL_INIT();
	KPD_ROW_INIT();
	
}
void KPD_ReadVal(char* ValuePtr)
{
	unsigned char Rowdata;
	unsigned char ColData;
	unsigned char LoopTermnate = 0;
	for(Rowdata = 0 ; (Rowdata < 4) & (LoopTermnate == 0) ; Rowdata ++)
	{
		KPD_ROW_WRITE((1<<Rowdata));
		KPD_COL_READ(&ColData);
		if(ColData != 0)
		{
			*ValuePtr = KeysLut[(Rowdata*3) + ColData/2];
			LoopTermnate = 1;
		}
		else
		{
			*ValuePtr = 'n';
		}
	}

	
	
}

