/**
  ******************************************************************************
  * @file    vc_interface.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module offers an interface to interact with vehicle properties
	******************************************************************************
	*/
	
#include "vc_interface.h"

//TODO: Complete register table
int32_t VCI_ReadRegister(VCI_Handle_t* pHandle, uint16_t RegID)
{
	int32_t value = 0;
	
	switch (RegID)
	{
		// Vehicle parameters (0-255)	//
		case REG_DRVTRAIN_TYPE:
			value = pHandle->pDrivetrain->bDrivetrainType;
			break;
		case REG_DUALMOTOR:
			value = pHandle->pDrivetrain->bMode;
			break;
		case REG_MAINMOTOR:
			value = pHandle->pDrivetrain->bMainMotor;
			break;
		case REG_FLDB_START_TEMP:
			value = pHandle->pDrivetrain->sHeatsinkTempFoldback1.hStartValue; // To verify
			break;
		case REG_FLDB_END_TEMP:
			value = pHandle->pDrivetrain->sHeatsinkTempFoldback1.hEndValue;	  // To verify
			break;
		case REG_REGEN_SPEED_THR:
			break;
		case REG_SPEED_LIMIT_RAMP:
			break;
		case REG_SPEED_CTRL_MODE:
			value = pHandle->pDrivetrain->bCtrlType;
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
//			value = pHandle->pDrivetrain->pPAS
			break;
		case REG_FLDB_END_VOLTAGE:
			value = pHandle->pDrivetrain->sDCVoltageFoldback.hEndValue;
			break;
		case REG_FLDB_START_VOLTAGE:
			value = pHandle->pDrivetrain->sDCVoltageFoldback.hStartValue;
			break;
		case REG_FLDB_END_SOC:
			break;
		case REG_FLDB_START_SOC:
			break;
		
	// Peripheral parameters (256-511)//
		case REG_THROTTLE_SLOPE:
			value = pHandle->pDrivetrain->pThrottle->hParam.bSlopeThrottle;
			break;
		case REG_THROTTLE_DIVISOR:
			value = pHandle->pDrivetrain->pThrottle->hParam.bDivisorThrottle;
			break;
		case REG_THROTTLE_OFFSET:
			value = pHandle->pDrivetrain->pThrottle->hParam.hOffsetThrottle;
			break;
		case REG_PAS_ENABLE:
//			value = pHandle->pDrivetrain->pPAS // To complete...			
			break;
		case REG_PAS_TYPE:
//			value = pHandle->pDrivetrain->pPAS // To complete...
			break;
		case REG_PAS_MAXTORQUE:
//			value = pHandle->pDrivetrain->pPAS // To complete...			
			break;
		case REG_PAS_LEVEL:
			value = pHandle->pDrivetrain->pPAS->bLevel;
			break;
		case REG_PAS_SENSE_DELAY:
//			value = pHandle->pDrivetrain->pPAS // To complete...
			break;
		case REG_PAS_POWERGAIN:
//			value = pHandle->pDrivetrain->pPAS // To complete...
			break;
		case REG_PAS_TORQUESENSOR_OFFSET:
			break;
		case REG_PAS_MAXLEVEL:
			value = pHandle->pDrivetrain->pPAS->bMaxLevel;
			break;
		case REG_TORQUESENSOR_HIGH_THR:
			break;
		case REG_TORQUESENSOR_LOW_THR:
			break;
		case REG_THROTTLE_DEADBAND_THR:
			break;
		case REG_THROTTLE_FAULTRANGE:
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
		case REG_M1_TORQUE_KP:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_TORQUE_KI:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FLUX_KP:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FLUX_KI:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_ANGLE_OFFSET:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FF_C1:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FF_C2:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FF_C3:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FW_KP:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FW_KI:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_MAXPHASECURR:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_MAXMOTORTEMP:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_WHEELDIA:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_GEARRATIO:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_POLE_PAIR:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_SENSORLESS:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_OBS_G1:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_OBS_G2:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_RS:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_LS:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_KE:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_RATEDCURR:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_RATEDPOWER_RACE_PAS:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_RATEDPOWER_THROTTLE_RACE_THROTTLE:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_RATEDPOWER_STREET_PAS:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_RATEDPOWER_STREET_THROTTLE:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_RATEDSPEED:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_SPEEDLIMITER_OUTPUT:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_MOTOR_TEMP_RAW:
			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->MDMeas.temp_motor; // To verify
			break;
		case REG_M1_AUTOTUNE_HALL_OFFSET:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_AUTOTUNE_LS:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_AUTOTUNE_RATEDSPEED:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_AUTOTUNE_RS:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case	REG_M1_AUTOTUNE_KE:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_HALL_INTERPOL_START:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_HALL_INTERPOL_STOP:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FWCURR_MAX:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FLDB_END_MOTTEMP:
			value = pHandle->pDrivetrain->sMotorTempFoldback1.hEndValue;
			break;
		case REG_M1_FLDB_START_MOTTEMP:
			value = pHandle->pDrivetrain->sMotorTempFoldback1.hStartValue;
			break;
		case REG_M1_OPENLOOP_ANGLE:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_OPENLOOP_CURR:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_OPENLOOP_FREQ:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_OVRLOAD_CONTINUOUS_CURR:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_OVRLOAD_COOLING_CURR:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_OVRLOAD_COOLING_TIME:
//			//			value = pHandle->pDrivetrain->
			break;
		case REG_M1_OVRLOAD_FLDB_END:
//			value = pHandle->pDrivetrain->
			break;
		case REG_M1_OVRLOAD_FLDB_START:
//			value = pHandle->pDrivetrain->
			break;
		case REG_M1_OVRLOAD_HEATING_CURR:
//			value = pHandle->pDrivetrain->
			break;
		case REG_M1_OVRLOAD_HEATING_TIME:
//			value = pHandle->pDrivetrain->
			break;
		case REG_M1_SENSORLESS_PLL_KI:
//			value = pHandle->pDrivetrain->
			break;
		case REG_M1_SENSORLESS_PLL_KP:
//			value = pHandle->pDrivetrain->
			break;
		case 	REG_M1_RATEDFREQ:
//			value = pHandle->pDrivetrain->
			break;
		case REG_M1_REGENCURR_MAX:
//			value = pHandle->pDrivetrain->
			break;
		case REG_M1_SENSORLESS_OPENLOOP_CURR_RAMPTIME:
//			value = pHandle->pDrivetrain->
			break;
		case REG_M1_SENSORLESS_TRANSITION_FREQ:
//			value = pHandle->pDrivetrain->
			break;
		case REG_M1_SENSORLESS_OPENLOOP_CURR_HOLDTIME:
//			value = pHandle->pDrivetrain->
			break;
		case REG_M1_SENSORLESS_OPENLOOP_FREQ_RAMPTIME:
//			value = pHandle->pDrivetrain->
			break;
		case REG_M1_SENSORLESS_OPENLOOP_CURR:
//			value = pHandle->pDrivetrain->
			break;
		case REG_M1_SPEEDCTRL_KI:
//			value = pHandle->pDrivetrain->
			break;
		case REG_M1_SPEEDCTRL_KP:
//			value = pHandle->pDrivetrain->
			break;
		case REG_M1_HEATSINK_TEMP_RAW:
			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->MDMeas.temp_hs;
			break;
		case REG_M1_HEATSINK_OVERTEMP_THR:
//			value = pHandle->pDrivetrain->
			break;
		case REG_M1_REGENRATIO_MAX:
//			value = pHandle->pDrivetrain->
			break;
		case REG_M1_FWRATIO_MAX:
//			value = pHandle->pDrivetrain->
			break;
		case REG_M2_TORQUE_KP:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_TORQUE_KI:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FLUX_KP:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FLUX_KI:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_DTHRESHOLD:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_DSLOPE:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_ANGLE_OFFSET:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FF_C1:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FF_C2:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FF_C3:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FW_KP:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FW_KI:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_MAXPHASECURR:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_MAXMOTORTEMP:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_WHEELDIA:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_GEARRATIO:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_POLE_PAIR:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SENSORLESS:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_NOMINALCURR:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OBS_G1:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OBS_G2:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RS:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_LS:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_KE:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RATEDCURR:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RATEDPOWER_RACE_PAS:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RATEDPOWER_THROTTLE_RACE_THROTTLE:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RATEDPOWER_STREET_PAS:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RATEDPOWER_STREET_THROTTLE:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RATEDSPEED:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SPEEDLIMITER_OUTPUT:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_MOTOR_TEMP_RAW:
			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]->MDMeas.temp_motor;
			break;
		case REG_M2_AUTOTUNE_HALL_OFFSET:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_AUTOTUNE_LS:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_AUTOTUNE_RATEDSPEED:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_AUTOTUNE_RS:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_AUTOTUNE_KE:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_HALL_INTERPOL_START:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_HALL_INTERPOL_STOP:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FWCURR_MAX:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FLDB_END_MOTTEMP:
			value = pHandle->pDrivetrain->sMotorTempFoldback2.hEndValue;
			break;
		case REG_M2_FLDB_START_MOTTEMP:
			value = pHandle->pDrivetrain->sMotorTempFoldback2.hStartValue;
			break;
		case REG_M2_OPENLOOP_ANGLE:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OPENLOOP_CURR:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OPENLOOP_FREQ:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OVRLOAD_CONTINUOUS_CURR:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OVRLOAD_COOLING_CURR:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OVRLOAD_COOLING_TIME:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OVRLOAD_FLDB_END:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OVRLOAD_FLDB_START:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OVRLOAD_HEATING_CURR:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OVRLOAD_HEATING_TIME:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SENSORLESS_PLL_KI:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
	  case REG_M2_SENSORLESS_PLL_KP:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RATEDFREQ:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_REGENCURR_MAX:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SENSORLESS_OPENLOOP_CURR_RAMPTIME:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SENSORLESS_TRANSITION_FREQ:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SENSORLESS_OPENLOOP_CURR_HOLDTIME:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SENSORLESS_OPENLOOP_FREQ_RAMPTIME:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SENSORLESS_OPENLOOP_CURR:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SPEEDCTRL_KI:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SPEEDCTRL_KP:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_HEATSINK_TEMP_RAW:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_HEATSINK_OVERTEMP_THR:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_REGENRATIO_MAX:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FWRATIO_MAX:
//			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
	// Communication parameters (1024-1279)//
		case REG_UART_BAUDRATE:
//			value = pHandle->pDrivetrain->
			break;
		case REG_CAN_BAUDRATE:
//			value = pHandle->pDrivetrain->
			break;
		case REG_CAN_HEARTBEAT_PERIOD:
//			value = pHandle->pDrivetrain->
			break;
		case REG_CAN_ID:
//			value = pHandle->pDrivetrain->
			break;
		case REG_SLAVE_ID:
//			value = pHandle->pDrivetrain->
			break;
		
	// Manufacturing parameters (1280-1535)//
	case REG_TEST_MODE:
//		value = pHandle->pDrivetrain->
		break;
	case REG_FIRMVER:
//		value = pHandle->pDrivetrain->
		break;
	case REG_BOOTCOUNTER:
//		value = pHandle->pDrivetrain->
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
		case REG_DRVTRAIN_TYPE:
		{
			// cast to enum otherwise the compiler complains
			DRVT_Type_h castedValue = (DRVT_Type_h) value; 
			
			pHandle->pDrivetrain->bDrivetrainType = castedValue;
			break;
		}
		case REG_DUALMOTOR:
		{
			// cast to enum otherwise the compiler complains
			Motor_Mode_t castedValue = (Motor_Mode_t) value; 
			
			pHandle->pDrivetrain->bMode = castedValue;
			break;
		}
		case REG_MAINMOTOR:
			pHandle->pDrivetrain->bMainMotor = value;
			break;
		case REG_FLDB_START_TEMP:
			pHandle->pDrivetrain->sHeatsinkTempFoldback1.hStartValue = value; // To verify
			break;
		case REG_FLDB_END_TEMP:
			pHandle->pDrivetrain->sHeatsinkTempFoldback1.hEndValue = value;	  // To verify
			break;
		case REG_REGEN_SPEED_THR:
			break;
		case REG_SPEED_LIMIT_RAMP:
			break;
		case REG_SPEED_CTRL_MODE:
			pHandle->pDrivetrain->bCtrlType = (CTRL_Type_h)value;
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
			pHandle->pDrivetrain->sDCVoltageFoldback.hEndValue = value;
			break;
		case REG_FLDB_START_VOLTAGE:
			pHandle->pDrivetrain->sDCVoltageFoldback.hStartValue = value;
			break;
		case REG_FLDB_END_SOC:
			break;
		case REG_FLDB_START_SOC:
			break;
		
	// Peripheral parameters (256-511)//
		case REG_THROTTLE_SLOPE:
			pHandle->pDrivetrain->pThrottle->hParam.bSlopeThrottle = value;
			break;
		case REG_THROTTLE_DIVISOR:
			pHandle->pDrivetrain->pThrottle->hParam.bDivisorThrottle = value;
			break;
		case REG_THROTTLE_OFFSET:
			pHandle->pDrivetrain->pThrottle->hParam.hOffsetThrottle = value;
			break;
		case REG_PAS_ENABLE:
//		pHandle->pDrivetrain->pPAS // To complete...			
			break;
		case REG_PAS_TYPE:
//		pHandle->pDrivetrain->pPAS // To complete...
			break;
		case REG_PAS_MAXTORQUE:
//		pHandle->pDrivetrain->pPAS // To complete...			
			break;
		case REG_PAS_LEVEL:
			pHandle->pDrivetrain->pPAS->bLevel = value;
			break;
		case REG_PAS_SENSE_DELAY:
//		pHandle->pDrivetrain->pPAS // To complete...
			break;
		case REG_PAS_POWERGAIN:
//		pHandle->pDrivetrain->pPAS // To complete...
			break;
		case REG_PAS_TORQUESENSOR_OFFSET:
			break;
		case REG_PAS_MAXLEVEL:
			value = pHandle->pDrivetrain->pPAS->bMaxLevel;
			break;
		case REG_TORQUESENSOR_HIGH_THR:
			break;
		case REG_TORQUESENSOR_LOW_THR:
			break;
		case REG_THROTTLE_DEADBAND_THR:
			break;
		case REG_THROTTLE_FAULTRANGE:
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
		case REG_M1_TORQUE_KP:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_TORQUE_KI:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FLUX_KP:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FLUX_KI:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_ANGLE_OFFSET:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FF_C1:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FF_C2:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FF_C3:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FW_KP:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FW_KI:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_MAXPHASECURR:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_MAXMOTORTEMP:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_WHEELDIA:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_GEARRATIO:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_POLE_PAIR:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_SENSORLESS:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_OBS_G1:
//			pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_OBS_G2:
//			pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_RS:
//			pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_LS:
//			pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_KE:
//			pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_RATEDCURR:
//			pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_RATEDPOWER_RACE_PAS:
//			pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_RATEDPOWER_THROTTLE_RACE_THROTTLE:
//			pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_RATEDPOWER_STREET_PAS:
//			pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_RATEDPOWER_STREET_THROTTLE:
//			pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_RATEDSPEED:
//			pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_SPEEDLIMITER_OUTPUT:
//			pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_MOTOR_TEMP_RAW:
			value = pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->MDMeas.temp_motor; // To verify
			break;
		case REG_M1_AUTOTUNE_HALL_OFFSET:
//			pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_AUTOTUNE_LS:
//			pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_AUTOTUNE_RATEDSPEED:
//			pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_AUTOTUNE_RS:
//			pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case	REG_M1_AUTOTUNE_KE:
//			pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_HALL_INTERPOL_START:
//			pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_HALL_INTERPOL_STOP:
//			pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FWCURR_MAX:
//			pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_FLDB_END_MOTTEMP:
			pHandle->pDrivetrain->sMotorTempFoldback1.hEndValue = value;
			break;
		case REG_M1_FLDB_START_MOTTEMP:
			pHandle->pDrivetrain->sMotorTempFoldback1.hStartValue = value;
			break;
		case REG_M1_OPENLOOP_ANGLE:
//			pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_OPENLOOP_CURR:
//			pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_OPENLOOP_FREQ:
//			pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_OVRLOAD_CONTINUOUS_CURR:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_OVRLOAD_COOLING_CURR:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->
			break;
		case REG_M1_OVRLOAD_COOLING_TIME:
//		pHandle->pDrivetrain->
			break;
		case REG_M1_OVRLOAD_FLDB_END:
//		pHandle->pDrivetrain->
			break;
		case REG_M1_OVRLOAD_FLDB_START:
//		pHandle->pDrivetrain->
			break;
		case REG_M1_OVRLOAD_HEATING_CURR:
//		pHandle->pDrivetrain->
			break;
		case REG_M1_OVRLOAD_HEATING_TIME:
//		pHandle->pDrivetrain->
			break;
		case REG_M1_SENSORLESS_PLL_KI:
//		pHandle->pDrivetrain->
			break;
		case REG_M1_SENSORLESS_PLL_KP:
//		pHandle->pDrivetrain->
			break;
		case 	REG_M1_RATEDFREQ:
//		pHandle->pDrivetrain->
			break;
		case REG_M1_REGENCURR_MAX:
//		pHandle->pDrivetrain->
			break;
		case REG_M1_SENSORLESS_OPENLOOP_CURR_RAMPTIME:
//		pHandle->pDrivetrain->
			break;
		case REG_M1_SENSORLESS_TRANSITION_FREQ:
//		pHandle->pDrivetrain->
			break;
		case REG_M1_SENSORLESS_OPENLOOP_CURR_HOLDTIME:
//		pHandle->pDrivetrain->
			break;
		case REG_M1_SENSORLESS_OPENLOOP_FREQ_RAMPTIME:
//		pHandle->pDrivetrain->
			break;
		case REG_M1_SENSORLESS_OPENLOOP_CURR:
//		pHandle->pDrivetrain->
			break;
		case REG_M1_SPEEDCTRL_KI:
//		pHandle->pDrivetrain->
			break;
		case REG_M1_SPEEDCTRL_KP:
//		pHandle->pDrivetrain->
			break;
		case REG_M1_HEATSINK_TEMP_RAW:
			pHandle->pDrivetrain->pMDI->pMDC->pMD[M1]->MDMeas.temp_hs = value;
			break;
		case REG_M1_HEATSINK_OVERTEMP_THR:
//		pHandle->pDrivetrain->
			break;
		case REG_M1_REGENRATIO_MAX:
//		pHandle->pDrivetrain->
			break;
		case REG_M1_FWRATIO_MAX:
//		pHandle->pDrivetrain->
			break;
		case REG_M2_TORQUE_KP:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_TORQUE_KI:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FLUX_KP:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FLUX_KI:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_DTHRESHOLD:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_DSLOPE:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_ANGLE_OFFSET:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FF_C1:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FF_C2:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FF_C3:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FW_KP:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FW_KI:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_MAXPHASECURR:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_MAXMOTORTEMP:
//		Handle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_WHEELDIA:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_GEARRATIO:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_POLE_PAIR:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SENSORLESS:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_NOMINALCURR:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OBS_G1:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OBS_G2:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RS:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_LS:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_KE:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RATEDCURR:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RATEDPOWER_RACE_PAS:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RATEDPOWER_THROTTLE_RACE_THROTTLE:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RATEDPOWER_STREET_PAS:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RATEDPOWER_STREET_THROTTLE:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RATEDSPEED:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SPEEDLIMITER_OUTPUT:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_MOTOR_TEMP_RAW:
			pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]->MDMeas.temp_motor = value;
			break;
		case REG_M2_AUTOTUNE_HALL_OFFSET:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_AUTOTUNE_LS:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_AUTOTUNE_RATEDSPEED:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_AUTOTUNE_RS:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_AUTOTUNE_KE:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_HALL_INTERPOL_START:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_HALL_INTERPOL_STOP:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FWCURR_MAX:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FLDB_END_MOTTEMP:
			pHandle->pDrivetrain->sMotorTempFoldback2.hEndValue = value;
			break;
		case REG_M2_FLDB_START_MOTTEMP:
			pHandle->pDrivetrain->sMotorTempFoldback2.hStartValue = value;
			break;
		case REG_M2_OPENLOOP_ANGLE:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OPENLOOP_CURR:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OPENLOOP_FREQ:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OVRLOAD_CONTINUOUS_CURR:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OVRLOAD_COOLING_CURR:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OVRLOAD_COOLING_TIME:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OVRLOAD_FLDB_END:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OVRLOAD_FLDB_START:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OVRLOAD_HEATING_CURR:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_OVRLOAD_HEATING_TIME:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SENSORLESS_PLL_KI:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
	  case REG_M2_SENSORLESS_PLL_KP:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_RATEDFREQ:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_REGENCURR_MAX:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SENSORLESS_OPENLOOP_CURR_RAMPTIME:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SENSORLESS_TRANSITION_FREQ:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SENSORLESS_OPENLOOP_CURR_HOLDTIME:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SENSORLESS_OPENLOOP_FREQ_RAMPTIME:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SENSORLESS_OPENLOOP_CURR:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SPEEDCTRL_KI:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_SPEEDCTRL_KP:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_HEATSINK_TEMP_RAW:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_HEATSINK_OVERTEMP_THR:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_REGENRATIO_MAX:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
		case REG_M2_FWRATIO_MAX:
//		pHandle->pDrivetrain->pMDI->pMDC->pMD[M2]
			break;
	// Communication parameters (1024-1279)//
		case REG_UART_BAUDRATE:
//		pHandle->pDrivetrain->
			break;
		case REG_CAN_BAUDRATE:
//		pHandle->pDrivetrain->
			break;
		case REG_CAN_HEARTBEAT_PERIOD:
//		pHandle->pDrivetrain->
			break;
		case REG_CAN_ID:
//		pHandle->pDrivetrain->
			break;
		case REG_SLAVE_ID:
//		pHandle->pDrivetrain->
			break;
		
		// Manufacturing parameters (1280-1535)//
		case REG_TEST_MODE:
//		pHandle->pDrivetrain->
			break;
		case REG_FIRMVER:
//		pHandle->pDrivetrain->
			break;
		case REG_BOOTCOUNTER:
//		pHandle->pDrivetrain->
			break;		
		default:
			break;
		}
}

