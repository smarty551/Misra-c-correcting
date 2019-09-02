/*
 * BLMGR.h
 *
 * Created: 28/02/2016 04:20:32 م
 *  Author: hossam
 */ 


#ifndef BLMGR_H_
#define BLMGR_H_

#define SLAVE_COMM 0x00
#define MSTER_COMM 0x01
#define COMM_CINFIG MSTER_COMM

#define ROLE_CHAIR 0x02
#define ROLE_CAP  0x03
#define ROLE_MAPP 0x01

#define DEVICE_ROLE ROLE_CHAIR
void BLMGR_Test(void);
void BLMGR_BluetoothInit(void);
void BLMGR_BluetoothStateMachine(void);
void BLMGR_StartDevice(void);
void BLMGR_SetReceiver(u8 Receiver);
void BLMGR_SetDeviceName(u8 DeviceName[],u8 DeviceNameLength);
void BLMGR_SetBattLevel(u8 BattLevel);
#endif /* BLMGR_H_ */