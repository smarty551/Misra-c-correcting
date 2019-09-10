/*
 * KEYPAD.c
 *
 * Created: 30/01/2016 06:38:37 م
 *  Author: hossam
 */ 
#include "DIO.h"

/*Local Symbols*/
#define KPD_COL_PORT (PD)
#define KPD_ROW_PORT (PD)
#define KPD_COL_MASK 0x70U
#define KPD_ROW_MASK 0x0fU
#define KPD_COL_PIN_NUM 4u
#define KPD_ROW_PIN_NUM 0u
/**************************************************/
void KPD_COL_INIT(void);

void KPD_ROW_INIT(void);

void KPD_COL_INIT(void)
{
    DIO_vidWritePortDirection(KPD_COL_PORT,KPD_COL_MASK,0x00);
}

void KPD_ROW_INIT(void) {
    DIO_vidWritePortDirection(KPD_ROW_PORT,KPD_ROW_MASK,0xff);
    DIO_vidWritePortData(KPD_ROW_PORT,KPD_ROW_MASK,0x00);
}
void KPD_COL_READ(u8 * VALPTR);
void KPD_COL_READ(u8 * VALPTR)
{
    DIO_vidReadPortData(KPD_COL_PORT,KPD_COL_MASK,(VALPTR));
    *(VALPTR) = (*(VALPTR)) >> KPD_COL_PIN_NUM;
}

void KPD_ROW_WRITE(u8 DATAl);
void KPD_ROW_WRITE(u8 DATAl)
{
    DIO_vidWritePortData(KPD_ROW_PORT,KPD_ROW_MASK,((DATAl) << KPD_ROW_PIN_NUM));
}
void KPD_Init(void);

void KPD_Init(void)
{
    KPD_COL_INIT();
    KPD_ROW_INIT();

}
void KPD_ReadVal(char* ValuePtr);

void KPD_ReadVal(char* ValuePtr)
{
    const u8 KeysLut[]= {'1' , '2' , '3' , '4' , '5' , '6','7' , '8' , '9','*' , '0' , '#'};

    unsigned char Rowdata;
    unsigned char ColData;
    unsigned char LoopTermnate = 0U;
    for(Rowdata = 0U ; (Rowdata < 4U) && (LoopTermnate == 0U) ; Rowdata++)
    {
        KPD_ROW_WRITE(((u8)(1U)<<(u8)Rowdata));
        KPD_COL_READ(&ColData);
        if(ColData != 0U)
        {
            *ValuePtr = (char)(KeysLut[((u8)(Rowdata*3u) + (u8)(ColData/2u))]);
            LoopTermnate = 1U;
        }
        else
        {
            *ValuePtr = 'n';
        }
    }



}

