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
	
	/*switch (RegID)
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
			value = 0;
			break;
		case REG_FLDB_END_TEMP:
			value = 0;
			break;
		case REG_REGEN_SPEED_THR:
			value = 0;
			break;
		case REG_SPEED_LIMIT_RAMP:
			value = 0;
			break;
		case REG_SPEED_CTRL_MODE:
			value = pHandle->pDrivetrain->bCtrlType;
			break;
		case REG_TORQUESENSOR_FAULTTIME:
			value = 0;
			break;
		case REG_TORQUESENSOR_VOLTAGEFAULT_THR:
			value = 0;
			break;
		case REG_TORQUESENSOR_FILTER:
			value = 0;
			break;
		case REG_MAXSPEED_THROTTLE:
			value = pHandle->pDrivetrain->aTorque
			break;
		case REG_MAXSPEED_PAS:
			value = pHandle->pDrivetrain->
			break;
		case REG_FLDB_END_VOLTAGE:
			value = pHandle->pDrivetrain->
			break;
		case REG_FLDB_START_VOLTAGE:
			value = pHandle->pDrivetrain->
			break;
		case REG_FLDB_END_SOC:
			value = pHandle->pDrivetrain->
			break;
		case REG_FLDB_START_SOC:
			value = pHandle->pDrivetrain->
			break;
		
	// Peripheral parameters (256-511)//
		case REG_THROTTLE_SLOPE:
			value = pHandle->pDrivetrain->pThrottle->hParam.m;
			break;
		case REG_THROTTLE_DIVISOR:
			value = pHandle->pDrivetrain->
			break;
		case REG_THROTTLE_OFFSET:
			value = pHandle->pDrivetrain->
			break;
		case REG_PAS_ENABLE:
			value = pHandle->pDrivetrain->
			break;
		case REG_PAS_TYPE:
			value = pHandle->pDrivetrain->
			break;
		case REG_PAS_MAXTORQUE:
			value = pHandle->pDrivetrain->
			break;
		case REG_PAS_LEVEL:
			value = pHandle->pDrivetrain->
			break;
		case REG_PAS_SENSE_DELAY:
			value = pHandle->pDrivetrain->
			break;
		case REG_PAS_POWERGAIN:
			value = pHandle->pDrivetrain->
			break;
		case REG_PAS_TORQUESENSOR_OFFSET
			value = pHandle->pDrivetrain->:
			break;
		case REG_PAS_MAXLEVEL:
			value = pHandle->pDrivetrain->
			break;
		case REG_TORQUESENSOR_HIGH_THR:
			value = pHandle->pDrivetrain->
			break;
		case REG_TORQUESENSOR_LOW_THR:
			value = pHandle->pDrivetrain->
			break;
		case REG_THROTTLE_DEADBAND_THR:
			value = pHandle->pDrivetrain->
			break;
		case REG_THROTTLE_FAULTRANGE:
			value = pHandle->pDrivetrain->
			break;
	// Battery parameters (512-767)//
		case REG_FAST_OVERVOLT_THR:
			value = pHandle->pDrivetrain->
			break;
		case REG_FAST_UNDERVOLT_THR:
			value = pHandle->pDrivetrain->
			break;
		case REG_BAT_CURRENT_MAX:
			value = pHandle->pDrivetrain->
			break;
		case REG_SLOW_OVERVOLT_THR:
			value = pHandle->pDrivetrain->
			break;
		case REG_SLOW_UNDERVOLT_THR:
			value = pHandle->pDrivetrain->
			break;
	// Motor parameters (768-1023)//
		case REG_M1_TORQUE_KP:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_TORQUE_KI:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_FLUX_KP:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_FLUX_KI:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_ANGLE_OFFSET:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_FF_C1:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_FF_C2:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_FF_C3:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_FW_KP:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_FW_KI:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_MAXPHASECURR:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_MAXMOTORTEMP:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_WHEELDIA:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_GEARRATIO:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_POLE_PAIR:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_SENSORLESS:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_OBS_G1:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_OBS_G2:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_RS:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_LS:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_KE:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_RATEDCURR:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_RATEDPOWER_RACE_PAS:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_RATEDPOWER_THROTTLE_
			value = pHandle->pDrivetrain->RACE_THROTTLE:
			break;
		case REG_M1_RATEDPOWER_STREET_PAS:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_RATEDPOWER_STREET_THROTTLE:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_RATEDSPEED:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_SPEEDLIMITER_OUTPUT:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_MOTOR_TEMP_RAW:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_AUTOTUNE_HALL_OFFSET
			value = pHandle->pDrivetrain->:
			break;
		case REG_M1_AUTOTUNE_LS:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_AUTOTUNE_RATEDSPEED:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_AUTOTUNE_RS:
			value = pHandle->pDrivetrain->
			break;
		case	REG_M1_AUTOTUNE_KE:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_HALL_INTERPOL_START:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_HALL_INTERPOL_STOP:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_FWCURR_MAX:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_FLDB_END_MOTTEMP:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_FLDB_START_MOTTEMP:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_OPENLOOP_ANGLE:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_OPENLOOP_CURR:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_OPENLOOP_FREQ:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_OVRLOAD_CONTINUOUS_CURR:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_OVRLOAD_COOLING_CURR:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_OVRLOAD_COOLING_TIME:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_OVRLOAD_FLDB_END:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_OVRLOAD_FLDB_START:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_OVRLOAD_HEATING_CURR:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_OVRLOAD_HEATING_TIME:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_SENSORLESS_PLL_KI:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_SENSORLESS_PLL_KP:
			value = pHandle->pDrivetrain->
			break;
		case 	REG_M1_RATEDFREQ:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_REGENCURR_MAX:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_SENSORLESS_OPENLOOP_CURR_RAMPTIME:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_SENSORLESS_TRANSITION_FREQ:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_SENSORLESS_OPENLOOP_CURR_HOLDTIME:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_SENSORLESS_OPENLOOP_FREQ_RAMPTIME:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_SENSORLESS_OPENLOOP_CURR:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_SPEEDCTRL_KI:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_SPEEDCTRL_KP:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_HEATSINK_TEMP_RAW:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_HEATSINK_OVERTEMP_THR:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_REGENRATIO_MAX:
			value = pHandle->pDrivetrain->
			break;
		case REG_M1_FWRATIO_MAX:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_TORQUE_KP:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_TORQUE_KI:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_FLUX_KP:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_FLUX_KI:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_DTHRESHOLD:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_DSLOPE:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_ANGLE_OFFSET:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_FF_C1:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_FF_C2:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_FF_C3:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_FW_KP:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_FW_KI:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_MAXPHASECURR:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_MAXMOTORTEMP:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_WHEELDIA:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_GEARRATIO:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_POLE_PAIR:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_SENSORLESS:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_NOMINALCURR:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_OBS_G1:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_OBS_G2:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_RS:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_LS:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_KE:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_RATEDCURR:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_RATEDPOWER_RACE_PAS:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_RATEDPOWER_THROTTLE_RACE_THROTTLE:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_RATEDPOWER_STREET_PAS:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_RATEDPOWER_STREET_THROTTLE:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_RATEDSPEED:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_SPEEDLIMITER_OUTPUT:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_MOTOR_TEMP_RAW:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_AUTOTUNE_HALL_OFFSET:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_AUTOTUNE_LS:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_AUTOTUNE_RATEDSPEED:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_AUTOTUNE_RS:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_AUTOTUNE_KE:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_HALL_INTERPOL_START:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_HALL_INTERPOL_STOP:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_FWCURR_MAX:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_FLDB_END_MOTTEMP:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_FLDB_START_MOTTEMP:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_OPENLOOP_ANGLE:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_OPENLOOP_CURR:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_OPENLOOP_FREQ:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_OVRLOAD_CONTINUOUS_CURR:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_OVRLOAD_COOLING_CURR:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_OVRLOAD_COOLING_TIME:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_OVRLOAD_FLDB_END:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_OVRLOAD_FLDB_START:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_OVRLOAD_HEATING_CURR:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_OVRLOAD_HEATING_TIME:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_SENSORLESS_PLL_KI:
			value = pHandle->pDrivetrain->
			break;
	  case REG_M2_SENSORLESS_PLL_KP:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_RATEDFREQ:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_REGENCURR_MAX:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_SENSORLESS_OPENLOOP_CURR_RAMPTIME:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_SENSORLESS_TRANSITION_FREQ:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_SENSORLESS_OPENLOOP_CURR_HOLDTIME:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_SENSORLESS_OPENLOOP_FREQ_RAMPTIME:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_SENSORLESS_OPENLOOP_CURR:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_SPEEDCTRL_KI:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_SPEEDCTRL_KP:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_HEATSINK_TEMP_RAW:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_HEATSINK_OVERTEMP_THR:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_REGENRATIO_MAX:
			value = pHandle->pDrivetrain->
			break;
		case REG_M2_FWRATIO_MAX:
			value = pHandle->pDrivetrain->
			break;
	// Communication parameters (1024-1279)//
		case REG_UART_BAUDRATE:
			value = pHandle->pDrivetrain->
			break;
		case REG_CAN_BAUDRATE:
			value = pHandle->pDrivetrain->
			break;
		case REG_CAN_HEARTBEAT_PERIOD:
			value = pHandle->pDrivetrain->
			break;
		case REG_CAN_ID:
			value = pHandle->pDrivetrain->
			break;
		case REG_SLAVE_ID:
			value = pHandle->pDrivetrain->
			break;
		
	// Manufacturing parameters (1280-1535)//
	case REG_TEST_MODE:
		value = pHandle->pDrivetrain->
		break;
	case REG_FIRMVER:
		value = pHandle->pDrivetrain->
		break;
	case REG_BOOTCOUNTER:
		value = pHandle->pDrivetrain->
		break;		
	default:
		break;
	}*/
	return value;
}

//TODO: Complete register table
void VCI_SetRegister(VCI_Handle_t* pHandle, uint16_t RegID, int32_t value)
{
	switch (RegID)
	{
		case REG_DRVTRAIN_TYPE:
			pHandle->pDrivetrain->bDrivetrainType = value;
			break;
		case REG_DUALMOTOR:
			pHandle->pDrivetrain->bMode = value;
			break;
		case REG_MAINMOTOR:
			pHandle->pDrivetrain->bMainMotor = value;
			break;
		case REG_FLDB_START_TEMP:
			break;
		case REG_FLDB_END_TEMP:
			break;
		case REG_REGEN_SPEED_THR:
			break;
		case REG_SPEED_LIMIT_RAMP:
			break;
		case REG_SPEED_CTRL_MODE:
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
			break;
		case REG_FLDB_START_VOLTAGE:
			break;
		case REG_FLDB_END_SOC:
			break;
		case REG_FLDB_START_SOC:
			break;
		
	/* Peripheral parameters (256-511)*/
		case REG_THROTTLE_SLOPE:
			break;
		case REG_THROTTLE_DIVISOR:
			break;
		case REG_THROTTLE_OFFSET:
			break;
		case REG_PAS_ENABLE:
			break;
		case REG_PAS_TYPE:
			break;
		case REG_PAS_MAXTORQUE:
			break;
		case REG_PAS_LEVEL:
			break;
		case REG_PAS_SENSE_DELAY:
			break;
		case REG_PAS_POWERGAIN:
			break;
		case REG_PAS_TORQUESENSOR_OFFSET:
			break;
		case REG_PAS_MAXLEVEL:
			break;
		case REG_TORQUESENSOR_HIGH_THR:
			break;
		case REG_TORQUESENSOR_LOW_THR:
			break;
		case REG_THROTTLE_DEADBAND_THR:
			break;
		case REG_THROTTLE_FAULTRANGE:
			break;
	/* Battery parameters (512-767)*/
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
	/* Motor parameters (768-1023)*/
		case REG_M1_TORQUE_KP:
			break;
		case REG_M1_TORQUE_KI:
			break;
		case REG_M1_FLUX_KP:
			break;
		case REG_M1_FLUX_KI:
			break;
		case REG_M1_ANGLE_OFFSET:
			break;
		case REG_M1_FF_C1:
			break;
		case REG_M1_FF_C2:
			break;
		case REG_M1_FF_C3:
			break;
		case REG_M1_FW_KP:
			break;
		case REG_M1_FW_KI:
			break;
		case REG_M1_MAXPHASECURR:
			break;
		case REG_M1_MAXMOTORTEMP:
			break;
		case REG_M1_WHEELDIA:
			break;
		case REG_M1_GEARRATIO:
			break;
		case REG_M1_POLE_PAIR:
			break;
		case REG_M1_SENSORLESS:
			break;
		case REG_M1_OBS_G1:
			break;
		case REG_M1_OBS_G2:
			break;
		case REG_M1_RS:
			break;
		case REG_M1_LS:
			break;
		case REG_M1_KE:
			break;
		case REG_M1_RATEDCURR:
			break;
		case REG_M1_RATEDPOWER_RACE_PAS:
			break;
		case REG_M1_RATEDPOWER_THROTTLE_RACE_THROTTLE:
			break;
		case REG_M1_RATEDPOWER_STREET_PAS:
			break;
		case REG_M1_RATEDPOWER_STREET_THROTTLE:
			break;
		case REG_M1_RATEDSPEED:
			break;
		case REG_M1_SPEEDLIMITER_OUTPUT:
			break;
		case REG_M1_MOTOR_TEMP_RAW:
			break;
		case REG_M1_AUTOTUNE_HALL_OFFSET:
			break;
		case REG_M1_AUTOTUNE_LS:
			break;
		case REG_M1_AUTOTUNE_RATEDSPEED:
			break;
		case REG_M1_AUTOTUNE_RS:
			break;
		case	REG_M1_AUTOTUNE_KE:
			break;
		case REG_M1_HALL_INTERPOL_START:
			break;
		case REG_M1_HALL_INTERPOL_STOP:
			break;
		case REG_M1_FWCURR_MAX:
			break;
		case REG_M1_FLDB_END_MOTTEMP:
			break;
		case REG_M1_FLDB_START_MOTTEMP:
			break;
		case REG_M1_OPENLOOP_ANGLE:
			break;
		case REG_M1_OPENLOOP_CURR:
			break;
		case REG_M1_OPENLOOP_FREQ:
			break;
		case REG_M1_OVRLOAD_CONTINUOUS_CURR:
			break;
		case REG_M1_OVRLOAD_COOLING_CURR:
			break;
		case REG_M1_OVRLOAD_COOLING_TIME:
			break;
		case REG_M1_OVRLOAD_FLDB_END:
			break;
		case REG_M1_OVRLOAD_FLDB_START:
			break;
		case REG_M1_OVRLOAD_HEATING_CURR:
			break;
		case REG_M1_OVRLOAD_HEATING_TIME:
			break;
		case REG_M1_SENSORLESS_PLL_KI:
			break;
		case REG_M1_SENSORLESS_PLL_KP:
			break;
		case 	REG_M1_RATEDFREQ:
			break;
		case REG_M1_REGENCURR_MAX:
			break;
		case REG_M1_SENSORLESS_OPENLOOP_CURR_RAMPTIME:
			break;
		case REG_M1_SENSORLESS_TRANSITION_FREQ:
			break;
		case REG_M1_SENSORLESS_OPENLOOP_CURR_HOLDTIME:
			break;
		case REG_M1_SENSORLESS_OPENLOOP_FREQ_RAMPTIME:
			break;
		case REG_M1_SENSORLESS_OPENLOOP_CURR:
			break;
		case REG_M1_SPEEDCTRL_KI:
			break;
		case REG_M1_SPEEDCTRL_KP:
			break;
		case REG_M1_HEATSINK_TEMP_RAW:
			break;
		case REG_M1_HEATSINK_OVERTEMP_THR:
			break;
		case REG_M1_REGENRATIO_MAX:
			break;
		case REG_M1_FWRATIO_MAX:
			break;
		case REG_M2_TORQUE_KP:
			break;
		case REG_M2_TORQUE_KI:
			break;
		case REG_M2_FLUX_KP:
			break;
		case REG_M2_FLUX_KI:
			break;
		case REG_M2_DTHRESHOLD:
			break;
		case REG_M2_DSLOPE:
			break;
		case REG_M2_ANGLE_OFFSET:
			break;
		case REG_M2_FF_C1:
			break;
		case REG_M2_FF_C2:
			break;
		case REG_M2_FF_C3:
			break;
		case REG_M2_FW_KP:
			break;
		case REG_M2_FW_KI:
			break;
		case REG_M2_MAXPHASECURR:
			break;
		case REG_M2_MAXMOTORTEMP:
			break;
		case REG_M2_WHEELDIA:
			break;
		case REG_M2_GEARRATIO:
			break;
		case REG_M2_POLE_PAIR:
			break;
		case REG_M2_SENSORLESS:
			break;
		case REG_M2_NOMINALCURR:
			break;
		case REG_M2_OBS_G1:
			break;
		case REG_M2_OBS_G2:
			break;
		case REG_M2_RS:
			break;
		case REG_M2_LS:
			break;
		case REG_M2_KE:
			break;
		case REG_M2_RATEDCURR:
			break;
		case REG_M2_RATEDPOWER_RACE_PAS:
			break;
		case REG_M2_RATEDPOWER_THROTTLE_RACE_THROTTLE:
			break;
		case REG_M2_RATEDPOWER_STREET_PAS:
			break;
		case REG_M2_RATEDPOWER_STREET_THROTTLE:
			break;
		case REG_M2_RATEDSPEED:
			break;
		case REG_M2_SPEEDLIMITER_OUTPUT:
			break;
		case REG_M2_MOTOR_TEMP_RAW:
			break;
		case REG_M2_AUTOTUNE_HALL_OFFSET:
			break;
		case REG_M2_AUTOTUNE_LS:
			break;
		case REG_M2_AUTOTUNE_RATEDSPEED:
			break;
		case REG_M2_AUTOTUNE_RS:
			break;
		case REG_M2_AUTOTUNE_KE:
			break;
		case REG_M2_HALL_INTERPOL_START:
			break;
		case REG_M2_HALL_INTERPOL_STOP:
			break;
		case REG_M2_FWCURR_MAX:
			break;
		case REG_M2_FLDB_END_MOTTEMP:
			break;
		case REG_M2_FLDB_START_MOTTEMP:
			break;
		case REG_M2_OPENLOOP_ANGLE:
			break;
		case REG_M2_OPENLOOP_CURR:
			break;
		case REG_M2_OPENLOOP_FREQ:
			break;
		case REG_M2_OVRLOAD_CONTINUOUS_CURR:
			break;
		case REG_M2_OVRLOAD_COOLING_CURR:
			break;
		case REG_M2_OVRLOAD_COOLING_TIME:
			break;
		case REG_M2_OVRLOAD_FLDB_END:
			break;
		case REG_M2_OVRLOAD_FLDB_START:
			break;
		case REG_M2_OVRLOAD_HEATING_CURR:
			break;
		case REG_M2_OVRLOAD_HEATING_TIME:
			break;
		case REG_M2_SENSORLESS_PLL_KI:
			break;
	  case REG_M2_SENSORLESS_PLL_KP:
			break;
		case REG_M2_RATEDFREQ:
			break;
		case REG_M2_REGENCURR_MAX:
			break;
		case REG_M2_SENSORLESS_OPENLOOP_CURR_RAMPTIME:
			break;
		case REG_M2_SENSORLESS_TRANSITION_FREQ:
			break;
		case REG_M2_SENSORLESS_OPENLOOP_CURR_HOLDTIME:
			break;
		case REG_M2_SENSORLESS_OPENLOOP_FREQ_RAMPTIME:
			break;
		case REG_M2_SENSORLESS_OPENLOOP_CURR:
			break;
		case REG_M2_SPEEDCTRL_KI:
			break;
		case REG_M2_SPEEDCTRL_KP:
			break;
		case REG_M2_HEATSINK_TEMP_RAW:
			break;
		case REG_M2_HEATSINK_OVERTEMP_THR:
			break;
		case REG_M2_REGENRATIO_MAX:
			break;
		case REG_M2_FWRATIO_MAX:
			break;
	/* Communication parameters (1024-1279)*/
		case REG_UART_BAUDRATE:
			break;
		case REG_CAN_BAUDRATE:
			break;
		case REG_CAN_HEARTBEAT_PERIOD:
			break;
		case REG_CAN_ID:
			break;
		case REG_SLAVE_ID:
			break;
		
		/* Manufacturing parameters (1280-1535)*/
		case REG_TEST_MODE:
			break;
		case REG_FIRMVER:
			break;
		case REG_BOOTCOUNTER:
			break;		
		default:
			break;
		}
}

