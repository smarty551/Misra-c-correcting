/*
 * MTR.h
 *
 * Created: 11/07/2015 01:32:32 ص
 *  Author: hossam
 */ 


#ifndef MTR_H_
#define MTR_H_

void MTR_MoveFwd(u8 Duty);
void MTR_MoveBkrd(u8 Duty);
void MTR_MoveRht(u8 Duty);
void MTR_MoveLft(u8 Duty);
 void MotorRun(u8 Duty); 
void MTR_Stop(void);
void MTR_init(void);


#endif /* MTR_H_ */