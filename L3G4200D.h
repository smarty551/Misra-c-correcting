/*
 * L3G4200D.h
 *
 * Created: 24/09/2015 09:13:19 م
 *  Author: hossam
 */ 


#ifndef L3G4200D_H_
#define L3G4200D_H_

#include "Basic_Types.h"
/*Wakeup States*/
#define u8WAKEUP_DN 0x00
#define u8WAKEUP_IN_PROGRESS 0x01
#define u8WAKEUP_FAILED 0x02

/*Reading Status*/
#define u8READING_OK       0x00
#define u8DATA_OVERWRITTEN 0x01
#define u8AXIS_DISABLED    0x02
#define u8NO_DATA_UPDATED  0x03

/*Init Service*/
u8 GYHD_Init(void);

/*Power MAnagement Services*/
void GYHD_WakeUpModule(u8* u8WakeUpStatePtr);
void GYHD_SleepModule(void);

/*Data Read*/
u8 GYHD_ReadXDirection(u16* pu16Data,u8* pu8ReadingStatus);
u8 GYHD_ReadYDirection(u16* pu16Data,u8* pu8ReadingStatus);
u8 GYHD_ReadZDirection(u16* pu16Data,u8* pu8ReadingStatus);
#endif /* L3G4200D_H_ */