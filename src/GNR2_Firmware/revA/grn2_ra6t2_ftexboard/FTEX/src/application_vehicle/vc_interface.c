/**
  * @file    vc_interface.c
  * @brief   This module offers an interface to interact with vehicle properties
	*/
	
#include "vc_interface.h"

//TODO: Complete register table
int32_t VCI_ReadRegister(VCI_Handle_t* pHandle, uint16_t RegID)
{
	int32_t value = 0;
	
	switch (RegID)
	{
		// Vehicle parameters (0-255)	//
		case REG_PWRTRAIN_TYPE:
			value = pHandle->pPowertrain->sParameters.bDrivetrainType;
			break;
		case REG_DUALMOTOR:
			value = pHandle->pPowertrain->sParameters.bMode;
			break;
		case REG_MAINMOTOR:
			value = pHandle->pPowertrain->bMainMotor;
			break;
		case REG_MOTORSELECT_ENABLE:
			value = pHandle->pPowertrain->pMS->bMSEnable;
 			break;
		case REG_POWERLOCK_ENABLE:
			value = pHandle->pPowertrain->pPWREN->bUsePowerLock;
			break;
		case REG_FLDB_START_TEMP:
			break;
		case REG_FLDB_END_TEMP:
			break;
		case REG_REGEN_SPEED_THR:
			break;
		case REG_SPEED_LIMIT_RAMP:
			break;
		case REG_CTRL_MODE:
			value = pHandle->pPowertrain->sParameters.bCtrlType;
			break;
		case REG_TORQUESENSOR_FAULTTIME:
			break;
		case REG_TORQUESENSOR_VOLTAGEFAULT_THR:
			break;
		case REG_TORQUESENSOR_FILTER:
			break;
		case REG_MAXSPEED_THROTTLE:
			break;
		case REG_MAXSPEED_PAS:
			break;
		case REG_FLDB_END_VOLTAGE:
			value = pHandle->pPowertrain->DCVoltageFoldback.hDecreasingEndValue;
			break;
		case REG_FLDB_START_VOLTAGE:
			break;
		case REG_FLDB_END_SOC:
			break;
		case REG_FLDB_START_SOC:
			break;
		
	// Peripheral parameters (256-511)//
		case REG_THROTTLE_SLOPE:
			value = pHandle->pPowertrain->pThrottle->hParameters.bSlopeThrottle;
			break;
		case REG_THROTTLE_DIVISOR:
			value = pHandle->pPowertrain->pThrottle->hParameters.bDivisorThrottle;
			break;
		case REG_THROTTLE_OFFSET:
			value = pHandle->pPowertrain->pThrottle->hParameters.hOffsetThrottle;
			break;
		case REG_TORQE_RAMP_UP:
			break;
		case REG_TORQE_RAMP_DOWN:
			break;
		case REG_START_THROTTLE:
			value = pHandle->pPowertrain->sParameters.hStartingThrottle;
			break;
		case REG_STOP_THROTTLE:
			value = pHandle->pPowertrain->sParameters.hStoppingThrottle;
			break;
		case REG_THROTTLE_BW1:
			break;
		case REG_THROTTLE_BW2:
			break;
		case REG_THROTTLE_DEADBAND_THR:
			break;
		case REG_THROTTLE_FAULTRANGE:
			break;
		case REG_TORQUE_SLOPE:
			value = pHandle->pPowertrain->pThrottle->hParameters.bSlopeTorque;
			break;
		case REG_TORQUE_DIVISOR:
			value = pHandle->pPowertrain->pThrottle->hParameters.bDivisorTorque;
			break;
		case REG_TORQUE_OFFSET:
			value = pHandle->pPowertrain->pThrottle->hParameters.hOffsetTorque;
			break;
		case REG_TORQUESENSOR_HIGH_THR:
			break;
		case REG_TORQUESENSOR_LOW_THR:
			break;
		case REG_STOP_SPEED:
			value = pHandle->pPowertrain->sParameters.hStoppingSpeed;
			break;
		case REG_SPEED_OFFSET:
			value = pHandle->pPowertrain->pThrottle->hParameters.hOffsetSpeed;
			break;
		case REG_SPEED_SLOPE:
			value = pHandle->pPowertrain->pThrottle->hParameters.bSlopeSpeed;
			break;
		case REG_SPEED_DIVISOR:
			value = pHandle->pPowertrain->pThrottle->hParameters.bDivisorSpeed;
			break;
		case REG_SPEED_RAMP_UP:
			break;
		case REG_SPEED_RAMP_DOWN:
			break;
		
		case REG_PAS_ENABLE:
//			value = pHandle->pPowertrain->pPAS // To complete...			
			break;
		case REG_PAS_TYPE:
//			value = pHandle->pPowertrain->pPAS // To complete...
			break;
		case REG_PAS_MAXTORQUE:
//			value = pHandle->pPowertrain->pPAS // To complete...			
			break;
		case REG_PAS_LEVEL:
			//value = pHandle->pPowertrain->pPAS->bLevel;
			break;
		case REG_PAS_SENSE_DELAY:
//			value = pHandle->pPowertrain->pPAS // To complete...
			break;
		case REG_PAS_POWERGAIN:
//			value = pHandle->pPowertrain->pPAS // To complete...
			break;
		case REG_PAS_TORQUESENSOR_OFFSET:
			break;
		case REG_PAS_MAXLEVEL:
//			value = pHandle->pPowertrain->pPAS->bMaxLevel;
			break;

		
	// Battery parameters (512-767)//
		case REG_FAST_OVERVOLT_THR:
			break;
		case REG_FAST_UNDERVOLT_THR:
			break;
		case REG_BAT_CURRENT_MAX:
			break;
		case REG_SLOW_OVERVOLT_THR:
			break;
		case REG_SLOW_UNDERVOLT_THR:
			break;
		
	// Motor parameters (768-1023)//
		case REG_M1_ENABLE:
			value = pHandle->pPowertrain->sParameters.bUseMotorM1;
			break;
		case REG_M1_TORQUE_KP:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_TORQUE_KI:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FLUX_KP:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FLUX_KI:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_ANGLE_OFFSET:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FF_C1:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FF_C2:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FF_C3:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FW_KP:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FW_KI:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_MAXPHASECURR:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_MAXMOTORTEMP:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_WHEELDIA:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_GEARRATIO:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_POLE_PAIR:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_SENSORLESS:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_OBS_G1:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_OBS_G2:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_RS:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_LS:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_KE:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_RATEDCURR:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_RATEDPOWER_RACE_PAS:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_RATEDPOWER_THROTTLE_RACE_THROTTLE:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_RATEDPOWER_STREET_PAS:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_RATEDPOWER_STREET_THROTTLE:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_RATEDSPEED:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_SPEEDLIMITER_OUTPUT:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_MOTOR_TEMP_RAW:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->MDMeas.temp_motor; // To verify
			break;
		case REG_M1_AUTOTUNE_HALL_OFFSET:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_AUTOTUNE_LS:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_AUTOTUNE_RATEDSPEED:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_AUTOTUNE_RS:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case	REG_M1_AUTOTUNE_KE:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_HALL_INTERPOL_START:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_HALL_INTERPOL_STOP:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FWCURR_MAX:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FLDB_END_MOTTEMP:
			break;
		case REG_M1_FLDB_START_MOTTEMP:
			break;
		case REG_M1_OPENLOOP_ANGLE:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_OPENLOOP_CURR:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_OPENLOOP_FREQ:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_OVRLOAD_CONTINUOUS_CURR:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_OVRLOAD_COOLING_CURR:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_OVRLOAD_COOLING_TIME:
//			//			value = pHandle->pPowertrain->
			break;
		case REG_M1_OVRLOAD_FLDB_END:
//			value = pHandle->pPowertrain->
			break;
		case REG_M1_OVRLOAD_FLDB_START:
//			value = pHandle->pPowertrain->
			break;
		case REG_M1_OVRLOAD_HEATING_CURR:
//			value = pHandle->pPowertrain->
			break;
		case REG_M1_OVRLOAD_HEATING_TIME:
//			value = pHandle->pPowertrain->
			break;
		case REG_M1_SENSORLESS_PLL_KI:
//			value = pHandle->pPowertrain->
			break;
		case REG_M1_SENSORLESS_PLL_KP:
//			value = pHandle->pPowertrain->
			break;
		case 	REG_M1_RATEDFREQ:
//			value = pHandle->pPowertrain->
			break;
		case REG_M1_REGENCURR_MAX:
//			value = pHandle->pPowertrain->
			break;
		case REG_M1_SENSORLESS_OPENLOOP_CURR_RAMPTIME:
//			value = pHandle->pPowertrain->
			break;
		case REG_M1_SENSORLESS_TRANSITION_FREQ:
//			value = pHandle->pPowertrain->
			break;
		case REG_M1_SENSORLESS_OPENLOOP_CURR_HOLDTIME:
//			value = pHandle->pPowertrain->
			break;
		case REG_M1_SENSORLESS_OPENLOOP_FREQ_RAMPTIME:
//			value = pHandle->pPowertrain->
			break;
		case REG_M1_SENSORLESS_OPENLOOP_CURR:
//			value = pHandle->pPowertrain->
			break;
		case REG_M1_SPEEDCTRL_KI:
//			value = pHandle->pPowertrain->
			break;
		case REG_M1_SPEEDCTRL_KP:
//			value = pHandle->pPowertrain->
			break;
		case REG_M1_HEATSINK_TEMP_RAW:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->MDMeas.temp_hs;
			break;
		case REG_M1_HEATSINK_OVERTEMP_THR:
//			value = pHandle->pPowertrain->
			break;
		case REG_M1_REGENRATIO_MAX:
//			value = pHandle->pPowertrain->
			break;
		case REG_M1_FWRATIO_MAX:
//			value = pHandle->pPowertrain->
			break;
		case REG_M2_ENABLE:
			value = pHandle->pPowertrain->sParameters.bUseMotorM2;
			break;
		case REG_M2_TORQUE_KP:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_TORQUE_KI:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FLUX_KP:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FLUX_KI:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_DTHRESHOLD:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_DSLOPE:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_ANGLE_OFFSET:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FF_C1:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FF_C2:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FF_C3:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FW_KP:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FW_KI:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_MAXPHASECURR:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_MAXMOTORTEMP:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_WHEELDIA:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_GEARRATIO:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_POLE_PAIR:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SENSORLESS:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_NOMINALCURR:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OBS_G1:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OBS_G2:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RS:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_LS:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_KE:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RATEDCURR:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RATEDPOWER_RACE_PAS:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RATEDPOWER_THROTTLE_RACE_THROTTLE:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RATEDPOWER_STREET_PAS:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RATEDPOWER_STREET_THROTTLE:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RATEDSPEED:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SPEEDLIMITER_OUTPUT:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_MOTOR_TEMP_RAW:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]->MDMeas.temp_motor;
			break;
		case REG_M2_AUTOTUNE_HALL_OFFSET:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_AUTOTUNE_LS:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_AUTOTUNE_RATEDSPEED:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_AUTOTUNE_RS:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_AUTOTUNE_KE:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_HALL_INTERPOL_START:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_HALL_INTERPOL_STOP:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FWCURR_MAX:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FLDB_END_MOTTEMP:
			break;
		case REG_M2_FLDB_START_MOTTEMP:
			break;
		case REG_M2_OPENLOOP_ANGLE:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OPENLOOP_CURR:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OPENLOOP_FREQ:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OVRLOAD_CONTINUOUS_CURR:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OVRLOAD_COOLING_CURR:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OVRLOAD_COOLING_TIME:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OVRLOAD_FLDB_END:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OVRLOAD_FLDB_START:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OVRLOAD_HEATING_CURR:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OVRLOAD_HEATING_TIME:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SENSORLESS_PLL_KI:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
	  case REG_M2_SENSORLESS_PLL_KP:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RATEDFREQ:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_REGENCURR_MAX:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SENSORLESS_OPENLOOP_CURR_RAMPTIME:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SENSORLESS_TRANSITION_FREQ:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SENSORLESS_OPENLOOP_CURR_HOLDTIME:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SENSORLESS_OPENLOOP_FREQ_RAMPTIME:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SENSORLESS_OPENLOOP_CURR:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SPEEDCTRL_KI:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SPEEDCTRL_KP:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_HEATSINK_TEMP_RAW:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_HEATSINK_OVERTEMP_THR:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_REGENRATIO_MAX:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FWRATIO_MAX:
//			value = pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		
// Communication parameters (1024-1279)//
		case REG_UART_BAUDRATE:
//			value = pHandle->pPowertrain->
			break;
		case REG_CAN_BAUDRATE:
//			value = pHandle->pPowertrain->
			break;
		case REG_CAN_HEARTBEAT_PERIOD:
//			value = pHandle->pPowertrain->
			break;
		case REG_CAN_ID:
//			value = pHandle->pPowertrain->
			break;
		case REG_SLAVE_ID:
//			value = pHandle->pPowertrain->
			break;
		
	// Manufacturing parameters (1280-1535)//
	case REG_TEST_MODE:
//		value = pHandle->pPowertrain->
		break;
	case REG_FIRMVER:
//		value = pHandle->pPowertrain->
		break;
	case REG_BOOTCOUNTER:
//		value = pHandle->pPowertrain->
		break;
  case REG_DEVICE_ID_LOW:
//	    value = GetChipID(0);
    break;	
  case REG_DEVICE_ID_HIGH:
//		  value = GetChipID(1);
    break;		
	default:
		break;
	}
	return value;
}

//TODO: Complete register table
void VCI_SetRegister(VCI_Handle_t* pHandle, uint16_t RegID, int32_t value)
{
	switch (RegID)
	{
		case REG_PWRTRAIN_TYPE:
		{
			// cast to enum otherwise the compiler complains
			DrivetrainType_t castedValue = (DrivetrainType_t) value; 
			
			pHandle->pPowertrain->sParameters.bDrivetrainType = castedValue;
			break;
		}
		case REG_DUALMOTOR:
		{
			// cast to enum otherwise the compiler complains
			PowertrainMode_t castedValue = (PowertrainMode_t) value; 
			
			pHandle->pPowertrain->sParameters.bMode = castedValue;
			break;
		}
		case REG_MAINMOTOR:
			pHandle->pPowertrain->bMainMotor = (uint8_t)value;
			break;
		case REG_MOTORSELECT_ENABLE:
			pHandle->pPowertrain->pMS->bMSEnable = value;
			break;
		case REG_POWERLOCK_ENABLE:
			pHandle->pPowertrain->pPWREN->bUsePowerLock = value;
			break;
		case REG_FLDB_START_TEMP:
			break;
		case REG_FLDB_END_TEMP:
			break;
		case REG_REGEN_SPEED_THR:
			break;
		case REG_SPEED_LIMIT_RAMP:
			break;
		case REG_CTRL_MODE:
			pHandle->pPowertrain->sParameters.bCtrlType = (CtrlType_t)value;
			break;
		case REG_TORQUESENSOR_FAULTTIME:
			break;
		case REG_TORQUESENSOR_VOLTAGEFAULT_THR:
			break;
		case REG_TORQUESENSOR_FILTER:
			break;
		case REG_MAXSPEED_THROTTLE:
			break;
		case REG_MAXSPEED_PAS:
			break;
		case REG_FLDB_END_VOLTAGE:
			pHandle->pPowertrain->DCVoltageFoldback.hDecreasingEndValue = (int16_t) value;
			break;
		case REG_FLDB_START_VOLTAGE:
			break;
		case REG_FLDB_END_SOC:
			break;
		case REG_FLDB_START_SOC:
			break;
		
	// Peripheral parameters (256-511)//
		case REG_THROTTLE_SLOPE:
			pHandle->pPowertrain->pThrottle->hParameters.bSlopeThrottle = (uint16_t)value;
			break;
		case REG_THROTTLE_DIVISOR:
			pHandle->pPowertrain->pThrottle->hParameters.bDivisorThrottle = (uint16_t)value;
			break;
		case REG_THROTTLE_OFFSET:
			pHandle->pPowertrain->pThrottle->hParameters.hOffsetThrottle = (uint16_t)value;
			break;
		case REG_START_THROTTLE:
			pHandle->pPowertrain->sParameters.hStartingThrottle = (uint16_t)value;
			break;
		case REG_STOP_THROTTLE:
			pHandle->pPowertrain->sParameters.hStoppingThrottle = (uint16_t)value;
			break;
		case REG_THROTTLE_BW1:
			break;
		case REG_THROTTLE_BW2:
			break;
		case REG_THROTTLE_DEADBAND_THR:
			break;
		case REG_THROTTLE_FAULTRANGE:
			break;
		case REG_PAS_ENABLE:
//		pHandle->pPowertrain->pPAS // To complete...			
			break;
		case REG_PAS_TYPE:
//		pHandle->pPowertrain->pPAS // To complete...
			break;
		case REG_PAS_MAXTORQUE:
//		pHandle->pPowertrain->pPAS // To complete...			
			break;
		case REG_PAS_LEVEL:
//			pHandle->pPowertrain->pPAS->pLevel = (PAS_sLevel)value;
			break;
		case REG_PAS_SENSE_DELAY:
//		pHandle->pPowertrain->pPAS // To complete...
			break;
		case REG_PAS_POWERGAIN:
//		pHandle->pPowertrain->pPAS // To complete...
			break;
		case REG_PAS_TORQUESENSOR_OFFSET:
			break;
		case REG_PAS_MAXLEVEL:
//			pHandle->pPowertrain->pPAS->bMaxLevel = value;
			break;
		case REG_TORQUE_SLOPE:
			pHandle->pPowertrain->pThrottle->hParameters.bSlopeTorque = (int16_t) value;
			break;
		case REG_TORQUE_DIVISOR:
			pHandle->pPowertrain->pThrottle->hParameters.bDivisorTorque = (uint16_t)value;
			break;
		case REG_TORQUE_OFFSET:
			pHandle->pPowertrain->pThrottle->hParameters.hOffsetTorque = (uint16_t)value;
			break;
		case REG_TORQE_RAMP_DOWN:
			break;
		case REG_TORQE_RAMP_UP:
			break;
		case REG_TORQUESENSOR_HIGH_THR:
			break;
		case REG_TORQUESENSOR_LOW_THR:
			break;
		case REG_STOP_SPEED:
			pHandle->pPowertrain->sParameters.hStoppingSpeed = (uint16_t)value;
			break;
		case REG_SPEED_OFFSET:
			pHandle->pPowertrain->pThrottle->hParameters.hOffsetSpeed = (uint16_t)value;
			break;
		case REG_SPEED_SLOPE:
			pHandle->pPowertrain->pThrottle->hParameters.bSlopeSpeed = (int16_t) value;
			break;
		case REG_SPEED_DIVISOR:
			pHandle->pPowertrain->pThrottle->hParameters.bDivisorSpeed = (uint16_t)value;
			break;
		case REG_SPEED_RAMP_UP:
			break;
		case REG_SPEED_RAMP_DOWN:
			break;

	// Battery parameters (512-767)//
		case REG_FAST_OVERVOLT_THR:
			break;
		case REG_FAST_UNDERVOLT_THR:
			break;
		case REG_BAT_CURRENT_MAX:
			break;
		case REG_SLOW_OVERVOLT_THR:
			break;
		case REG_SLOW_UNDERVOLT_THR:
			break;
	// Motor parameters (768-1023)//
		case REG_M1_ENABLE:
			pHandle->pPowertrain->sParameters.bUseMotorM1 = value;
			break;
		case REG_M1_TORQUE_KP:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_TORQUE_KI:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FLUX_KP:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FLUX_KI:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_ANGLE_OFFSET:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FF_C1:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FF_C2:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FF_C3:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FW_KP:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FW_KI:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_MAXPHASECURR:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_MAXMOTORTEMP:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_WHEELDIA:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_GEARRATIO:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_POLE_PAIR:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_SENSORLESS:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_OBS_G1:
//			pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_OBS_G2:
//			pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_RS:
//			pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_LS:
//			pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_KE:
//			pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_RATEDCURR:
//			pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_RATEDPOWER_RACE_PAS:
//			pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_RATEDPOWER_THROTTLE_RACE_THROTTLE:
//			pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_RATEDPOWER_STREET_PAS:
//			pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_RATEDPOWER_STREET_THROTTLE:
//			pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_RATEDSPEED:
//			pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_SPEEDLIMITER_OUTPUT:
//			pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_MOTOR_TEMP_RAW:
//			pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->MDMeas.temp_motor = value; // To verify
			break;
		case REG_M1_AUTOTUNE_HALL_OFFSET:
//			pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_AUTOTUNE_LS:
//			pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_AUTOTUNE_RATEDSPEED:
//			pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_AUTOTUNE_RS:
//			pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case	REG_M1_AUTOTUNE_KE:
//			pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_HALL_INTERPOL_START:
//			pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_HALL_INTERPOL_STOP:
//			pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FWCURR_MAX:
//			pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FLDB_END_MOTTEMP:
			break;
		case REG_M1_FLDB_START_MOTTEMP:
			break;
		case REG_M1_OPENLOOP_ANGLE:
//			pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_OPENLOOP_CURR:
//			pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_OPENLOOP_FREQ:
//			pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_OVRLOAD_CONTINUOUS_CURR:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_OVRLOAD_COOLING_CURR:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_OVRLOAD_COOLING_TIME:
//		pHandle->pPowertrain->
			break;
		case REG_M1_OVRLOAD_FLDB_END:
//		pHandle->pPowertrain->
			break;
		case REG_M1_OVRLOAD_FLDB_START:
//		pHandle->pPowertrain->
			break;
		case REG_M1_OVRLOAD_HEATING_CURR:
//		pHandle->pPowertrain->
			break;
		case REG_M1_OVRLOAD_HEATING_TIME:
//		pHandle->pPowertrain->
			break;
		case REG_M1_SENSORLESS_PLL_KI:
//		pHandle->pPowertrain->
			break;
		case REG_M1_SENSORLESS_PLL_KP:
//		pHandle->pPowertrain->
			break;
		case 	REG_M1_RATEDFREQ:
//		pHandle->pPowertrain->
			break;
		case REG_M1_REGENCURR_MAX:
//		pHandle->pPowertrain->
			break;
		case REG_M1_SENSORLESS_OPENLOOP_CURR_RAMPTIME:
//		pHandle->pPowertrain->
			break;
		case REG_M1_SENSORLESS_TRANSITION_FREQ:
//		pHandle->pPowertrain->
			break;
		case REG_M1_SENSORLESS_OPENLOOP_CURR_HOLDTIME:
//		pHandle->pPowertrain->
			break;
		case REG_M1_SENSORLESS_OPENLOOP_FREQ_RAMPTIME:
//		pHandle->pPowertrain->
			break;
		case REG_M1_SENSORLESS_OPENLOOP_CURR:
//		pHandle->pPowertrain->
			break;
		case REG_M1_SPEEDCTRL_KI:
//		pHandle->pPowertrain->
			break;
		case REG_M1_SPEEDCTRL_KP:
//		pHandle->pPowertrain->
			break;
		case REG_M1_HEATSINK_TEMP_RAW:
//			pHandle->pPowertrain->pMDI->pMDC->pMD[M1]->MDMeas.temp_hs = value;
			break;
		case REG_M1_HEATSINK_OVERTEMP_THR:
//		pHandle->pPowertrain->
			break;
		case REG_M1_REGENRATIO_MAX:
//		pHandle->pPowertrain->
			break;
		case REG_M1_FWRATIO_MAX:
//		pHandle->pPowertrain->
			break;
		case REG_M2_ENABLE:
			value = pHandle->pPowertrain->sParameters.bUseMotorM2;
			break;
		case REG_M2_TORQUE_KP:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_TORQUE_KI:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FLUX_KP:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FLUX_KI:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_DTHRESHOLD:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_DSLOPE:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_ANGLE_OFFSET:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FF_C1:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FF_C2:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FF_C3:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FW_KP:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FW_KI:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_MAXPHASECURR:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_MAXMOTORTEMP:
//		Handle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_WHEELDIA:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_GEARRATIO:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_POLE_PAIR:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SENSORLESS:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_NOMINALCURR:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OBS_G1:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OBS_G2:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RS:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_LS:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_KE:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RATEDCURR:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RATEDPOWER_RACE_PAS:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RATEDPOWER_THROTTLE_RACE_THROTTLE:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RATEDPOWER_STREET_PAS:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RATEDPOWER_STREET_THROTTLE:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RATEDSPEED:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SPEEDLIMITER_OUTPUT:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_MOTOR_TEMP_RAW:
//			pHandle->pPowertrain->pMDI->pMDC->pMD[M2]->MDMeas.temp_motor = value;
			break;
		case REG_M2_AUTOTUNE_HALL_OFFSET:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_AUTOTUNE_LS:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_AUTOTUNE_RATEDSPEED:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_AUTOTUNE_RS:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_AUTOTUNE_KE:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_HALL_INTERPOL_START:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_HALL_INTERPOL_STOP:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FWCURR_MAX:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FLDB_END_MOTTEMP:
			break;
		case REG_M2_FLDB_START_MOTTEMP:
			break;
		case REG_M2_OPENLOOP_ANGLE:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OPENLOOP_CURR:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OPENLOOP_FREQ:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OVRLOAD_CONTINUOUS_CURR:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OVRLOAD_COOLING_CURR:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OVRLOAD_COOLING_TIME:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OVRLOAD_FLDB_END:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OVRLOAD_FLDB_START:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OVRLOAD_HEATING_CURR:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OVRLOAD_HEATING_TIME:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SENSORLESS_PLL_KI:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
	  case REG_M2_SENSORLESS_PLL_KP:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RATEDFREQ:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_REGENCURR_MAX:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SENSORLESS_OPENLOOP_CURR_RAMPTIME:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SENSORLESS_TRANSITION_FREQ:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SENSORLESS_OPENLOOP_CURR_HOLDTIME:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SENSORLESS_OPENLOOP_FREQ_RAMPTIME:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SENSORLESS_OPENLOOP_CURR:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SPEEDCTRL_KI:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SPEEDCTRL_KP:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_HEATSINK_TEMP_RAW:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_HEATSINK_OVERTEMP_THR:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_REGENRATIO_MAX:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FWRATIO_MAX:
//		pHandle->pPowertrain->pMDI->pMDC->pMD[M2]
			break;
	// Communication parameters (1024-1279)//
		case REG_UART_BAUDRATE:
//		pHandle->pPowertrain->
			break;
		case REG_CAN_BAUDRATE:
//		pHandle->pPowertrain->
			break;
		case REG_CAN_HEARTBEAT_PERIOD:
//		pHandle->pPowertrain->
			break;
		case REG_CAN_ID:
//		pHandle->pPowertrain->
			break;
		case REG_SLAVE_ID:
//		pHandle->pPowertrain->
			break;
		
		// Manufacturing parameters (1280-1535)//
		case REG_TEST_MODE:
//		pHandle->pPowertrain->
			break;
		case REG_FIRMVER:
//		pHandle->pPowertrain->
			break;
		case REG_BOOTCOUNTER:
//		pHandle->pPowertrain->
			break;		
		default:
			break;
		}
}
