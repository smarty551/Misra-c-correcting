


#include "Basic_Types.h"
#include "BLMGR.h"


#include "PWM.h"
#include "GPT.h"
u16 main(void);
void Cyclic30ms(void);
void Cyclic30ms(void)
{
	static u8 TimeoutCounter = 0U;
	TimeoutCounter ++;
	if(TimeoutCounter == 10U)
	{
		BLMGR_BluetoothStateMachine();
		TimeoutCounter = 0U;
		
		
	}
}
u16 main(void)
{
	u16 Count= 0U;
	u16 start = 0U;
	u16 Count2 = 0U;
 


GPT_Timer30msInit(&Cyclic30ms);
	
	BLMGR_BluetoothInit();
	BLMGR_SetReceiver(ROLE_MAPP);
    
							
	PWM_Init();
	sei();
	PWM_SetSpeed(25U);

	
	while(1)
	{
		Count2 = (u16)(((u16)Count2 +1U) %(u16) 20U);
		BLMGR_SetBattLevel((u8)(Count2 / 4U));
		
		_delay_ms(100);
		Count ++;
		if(start == 0U)
		{
					if(Count > 5U)
					{
						BLMGR_StartDevice();

						start = 1U;
					}
					
		}

	    
		
		
	}
	
	
}
