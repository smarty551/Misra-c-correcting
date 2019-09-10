/*
 * L3G4200D_Cfg.h
 *
 * Created: 24/09/2015 09:13:50 م
 *  Author: hossam
 */ 


#ifndef L3G4200D_CFG_H_
#define L3G4200D_CFG_H_
#include "Basic_Types.h"
#include "DIO.h"
#include "SPI.h"

/*SPI Communication Configuration*/
void GYHD_INIT_SLAVE_SELECT(void);
void GYHD_ACTIVATE_SLAVE_SELECT(void);
void GYHD_DEACTIVATE_SLAVE_SELECT(void);

/*Timeout Management Configuration*/
#define u8USE_DELAY (0x00U)
#define u8USE_TIMER (0x01U)

#define u8TIMEOUT_FUNCTION (u8USE_DELAY)

#if(u8TIMEOUT_FUNCTION == u8USE_DELAY)


void u8START_TIME_OUT_MS(u8 DELAY_MS, u8 * FLAG_PTR);
#else
#define u8START_TIME_OUT_MS(DELAY_MS,FLAG_PTR)
#endif

/*Self Axis Movement Detection Config*/
#define ON                  (0x01U)
#define u8SELF_AXIS_MOV  (ON)


#endif /* L3G4200D_CFG_H_ */
