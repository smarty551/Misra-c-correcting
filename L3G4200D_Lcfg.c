/*
 * L3G4200D_Lcfg.c
 *
 * Created: 27/09/2015 02:11:03 م
 *  Author: hossam
 */ 
#include "L3G4200D_Lcfg.h"
#include "Basic_Types.h"
#include "L3G4200D_Lcfg.h"
#include "L3G4200D.h"

const L3G4200D_CfgType L3G4200D_ConfigParam = {.u8FullScaleValue                       = u8FS_250,
                                               .strAxisActivation.u8XAxisActivation    = u8X_AXIS_ACTIVE,
											   .strAxisActivation.u8YAxisActivation    = u8Y_AXIS_ACTIVE,
											   .strAxisActivation.u8ZAxisActivation    = u8Z_AXIS_ACTIVE,
											   .strFilterCfg.u8FilterOnData            = u8NO_FILTERS,
											   .strFilterCfg.u8FilterOnInterrupt       = u8NO_FILTERS,
											   .strIntCfg.u16ThresholdX                = 0X2CA4,
											   .strIntCfg.u16ThresholdY                = 0x2CA4,
											   .strIntCfg.u16ThresholdZ                = 0x2CA4,
											   .strIntCfg.u8IntDuration                = 0x00,
											   .strIntCfg.u8XAxisInterrupt             = u8INT1_X,
											   .strIntCfg.u8YAxisInterrupt             = u8INT1_Y,
											   .strIntCfg.u8ZAxisInterrupt             = u8INT1_Z};
											   