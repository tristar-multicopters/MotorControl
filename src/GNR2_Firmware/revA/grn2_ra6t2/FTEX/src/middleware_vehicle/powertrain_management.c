/**
  * @file    powertrain_management.c
  * @author  Sami Bouzid, FTEX
	* @author  Jorge Polo, FTEX
  * @brief   This module handles management of the vehicle powertrain
  *
*/

#include "powertrain_management.h"

#define OVERCURRENT_COUNTER 		0
#define STARTUP_COUNTER 				1
#define SPEEDFEEDBACK_COUNTER 	2

/* Functions ---------------------------------------------------- */

/**
	* @brief  Module initialization, to be called once before using it
	* @param  Powertrain handle
	* @retval None
	*/
void PWRT_Init(PWRT_Handle_t * pHandle, MotorControlInterfaceHandle_t * pMci_M1)
{
    MDI_Init(pHandle->pMDI, pMci_M1);
    THRO_Init(pHandle->pThrottle);
    BRK_Init(pHandle->pBrake);
    PSS_Init(pHandle->pPAS);
    MS_Init(pHandle->pMS);
    PWREN_Init(pHandle->pPWREN);

    pHandle->aTorque[M1] = 0; pHandle->aTorque[M2] = 0;
    pHandle->aSpeed[M1] = 0; pHandle->aSpeed[M2] = 0;
    pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M1] = 0; pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M2] = 0;
    pHandle->aFaultManagementCounters[STARTUP_COUNTER][M1] = 0; pHandle->aFaultManagementCounters[STARTUP_COUNTER][M2] = 0;
    pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M1] = 0; pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M2] = 0;
}

/**
	* @brief  Update current value of powertrain peripherals, such as throttle. To be called periodically.
	* @param  Powertrain handle
	* @retval None
	*/
void PWRT_UpdatePowertrainPeripherals(PWRT_Handle_t * pHandle)
{
    /* Update throttle sensor handle */
    THRO_CalcAvThrottleValue(pHandle->pThrottle);

    /* Update torque sensor handle */
    //PAS_CalcTSAvValue(pHandle->pPAS);
}

/**
	* @brief  Compute target torque and speed to be applied to each motors and store value into powertrain handle.
	* @param  Powertrain handle
	* @retval None
	*/
void PWRT_CalcMotorTorqueSpeed(PWRT_Handle_t * pHandle)
{
	bool bIsBrakePressed = BRK_IsPressed(pHandle->pBrake);
	bool bIsPwrEnabled = PWREN_IsPowerEnabled(pHandle->pPWREN);
	int32_t wSpeedM1 = MDI_GetAvrgMecSpeedUnit(pHandle->pMDI, M1);
	int32_t wSpeedM2 = MDI_GetAvrgMecSpeedUnit(pHandle->pMDI, M2);
	int32_t wSpeedMainMotor = MDI_GetAvrgMecSpeedUnit(pHandle->pMDI, pHandle->bMainMotor);
	int16_t hTempHeatSnkM1 = 0; //TODO
	int16_t hTempHeatSnkM2 = 0; //TODO
	int16_t hTempHeatSnkMainMotor = 0; //TODO

	MotorSelection_t bMotorSelection = MS_CheckSelection(pHandle->pMS); // Check which motor is selected
	int16_t hTorqueRef = 0; int32_t hSpeedRef = 0;
	int16_t hAux = 0;

	if (pHandle->pMS->bMSEnable)
	{
		switch (bMotorSelection) // Change powertrain mode and main motor depending on motor selection
		{
			case M1_SELECTED:
				pHandle->sParameters.bMode = SINGLE_MOTOR;
				pHandle->bMainMotor = M1;
				break;
			case M2_SELECTED:
				pHandle->sParameters.bMode = SINGLE_MOTOR;
				pHandle->bMainMotor = M2;
				break;
			case ALL_MOTOR_SELECTED:
				pHandle->sParameters.bMode = DUAL_MOTOR;
				pHandle->bMainMotor = pHandle->sParameters.bDefaultMainMotor;
				break;
			default:
				break;
		}
	}

	pHandle->aTorque[M1] = 0; pHandle->aTorque[M2] = 0;
	pHandle->aSpeed[M1] = 0; pHandle->aSpeed[M2] = 0;

	if (pHandle->sParameters.bCtrlType == TORQUE_CTRL) // If torque control
	{
		hTorqueRef = THRO_ThrottleToTorque(pHandle->pThrottle); // Translate throttle value to powertrain target torque value

		if (bIsBrakePressed)
		{
			hTorqueRef = 0;
		}
		if(pHandle->sParameters.bMode == SINGLE_MOTOR)
		{
      // Apply torque limitation based on heatsink temperature of M1
			hAux = FLDBK_ApplyTorqueLimitation(&pHandle->sHeatsinkTempFoldback[pHandle->bMainMotor], hTorqueRef, hTempHeatSnkMainMotor);
			pHandle->aTorque[pHandle->bMainMotor] = hAux; // Store powertrain target torque value in handle
		}
		if(pHandle->sParameters.bMode == DUAL_MOTOR)
		{
      // Apply torque limitation based on heatsink temperature of M1
			hAux = FLDBK_ApplyTorqueLimitation(&pHandle->sHeatsinkTempFoldback[M1], hTorqueRef, hTempHeatSnkM1);
			pHandle->aTorque[M1] = hAux;
      // Apply torque limitation based on heatsink temperature of M2
			hAux = FLDBK_ApplyTorqueLimitation(&pHandle->sHeatsinkTempFoldback[M2], hTorqueRef, hTempHeatSnkM2);
			pHandle->aTorque[M2] = pHandle->sParameters.bM2TorqueInversion ? -hAux : hAux; // Store powertrain target torque value in handle. Invert torque if needed.
		}
	}
	else if (pHandle->sParameters.bCtrlType == SPEED_CTRL)
	{
		/* SPEED CONTROL NOT IMPLEMENTED YET. JUST PLACEHOLDER CODE */
		hSpeedRef = THRO_ThrottleToSpeed(pHandle->pThrottle);
		if (bIsBrakePressed)
		{
			hSpeedRef = 0;
		}

		if(pHandle->sParameters.bMode == SINGLE_MOTOR)
		{
			pHandle->aSpeed[pHandle->bMainMotor] = hSpeedRef;
		}
		if(pHandle->sParameters.bMode == DUAL_MOTOR)
		{
			pHandle->aSpeed[M1] = hSpeedRef;
			pHandle->aSpeed[M2] = hSpeedRef;
		}
	}
	else {}
}

/**
	* @brief  Send torque and/or speed ramp commands to motors
	* @param  Powertrain handle
	* @retval None
	*/
void PWRT_ApplyMotorRamps(PWRT_Handle_t * pHandle)
{
	if (PWRT_IsMotor1Used(pHandle))
	{
		if (pHandle->sParameters.bCtrlType == TORQUE_CTRL)
		{
      MDI_ExecTorqueRamp(pHandle->pMDI, M1, pHandle->aTorque[M1]);
		}
		else if (pHandle->sParameters.bCtrlType == SPEED_CTRL)
		{
			MDI_ExecSpeedRamp(pHandle->pMDI, M1, pHandle->aSpeed[M1]);
		}
		else {}
	}
	if (PWRT_IsMotor2Used(pHandle))
	{
		if (pHandle->sParameters.bCtrlType == TORQUE_CTRL)
		{
			MDI_ExecTorqueRamp(pHandle->pMDI, M2, pHandle->aTorque[M2]);
		}
	}
}

/**
	* @brief  Send command to start motors
	* @param  Powertrain handle
	* @retval None
	*/
void PWRT_StartMotors(PWRT_Handle_t * pHandle)
{
	if (PWRT_IsMotor1Used(pHandle))
	{
		MDI_StartMotor(pHandle->pMDI, M1);
	}
	if (PWRT_IsMotor2Used(pHandle))
	{
		MDI_StartMotor(pHandle->pMDI, M2);
	}
}

/**
	* @brief  Send command to stop motors
	* @param  Powertrain handle
	* @retval None
	*/
void PWRT_StopMotors(PWRT_Handle_t * pHandle)
{
	if (PWRT_IsMotor1Used(pHandle))
	{
		MDI_StopMotor(pHandle->pMDI, M1);
	}
	if (PWRT_IsMotor2Used(pHandle))
	{
		MDI_StopMotor(pHandle->pMDI, M2);
	}
}

/**
	* @brief  Check for motor faults during standby state
	* @param  Powertrain handle
	* @retval Vehicle fault code
	*/
uint16_t PWRT_StandbyStateCheck(PWRT_Handle_t * pHandle)
{
	uint16_t hVehicleFault = 0;

	if (PWRT_IsMotor1Used(pHandle))
	{
		switch (MDI_GetSTMState(pHandle->pMDI, M1))
		{
			case M_IDLE:
			case M_STOP:
				break;
			case M_FAULT_NOW:
			case M_FAULT_OVER:
				hVehicleFault |= VC_M1_FAULTS;
				break;
			default:
				hVehicleFault |= VC_M1_UNEXPECTED_BEHAVIOR;
				break;
		}
	}
	if (PWRT_IsMotor2Used(pHandle))
	{
		switch (MDI_GetSTMState(pHandle->pMDI, M2))
		{
			case M_IDLE:
			case M_STOP:
				break;
			case M_FAULT_NOW:
			case M_FAULT_OVER:
				hVehicleFault |= VC_M2_FAULTS;
				break;
			default:
				hVehicleFault |= VC_M2_UNEXPECTED_BEHAVIOR;
				break;
		}
	}

	return hVehicleFault;
}

/**
	* @brief  Check for motor faults during start state
	* @param  Powertrain handle
	* @retval Vehicle fault code
	*/
uint16_t PWRT_StartStateCheck(PWRT_Handle_t * pHandle)
{
	uint16_t hVehicleFault = 0;

	if (PWRT_IsMotor1Used(pHandle))
	{
		switch (MDI_GetSTMState(pHandle->pMDI, M1))
		{
			case M_RUN:
				break;
			case M_FAULT_NOW:
			case M_FAULT_OVER:
				hVehicleFault |= VC_M1_FAULTS;
				break;
			default:
				break;
		}
	}
	if (PWRT_IsMotor2Used(pHandle))
	{
		switch (MDI_GetSTMState(pHandle->pMDI, M2))
		{
			case M_RUN:
				break;
			case M_FAULT_NOW:
			case M_FAULT_OVER:
				hVehicleFault |= VC_M2_FAULTS;
				break;
			default:
				break;
		}
	}

	return hVehicleFault;
}

/**
	* @brief  Check for motor faults during run state
	* @param  Powertrain handle
	* @retval Vehicle fault code
	*/
uint16_t PWRT_RunStateCheck(PWRT_Handle_t * pHandle)
{
	uint16_t hVehicleFault = 0;

	if (PWRT_IsMotor1Used(pHandle))
	{
		switch (MDI_GetSTMState(pHandle->pMDI, M1))
		{
			case M_RUN:
				break;
			case M_FAULT_NOW:
			case M_FAULT_OVER:
				hVehicleFault |= VC_M1_FAULTS;
				break;
			default:
				hVehicleFault |= VC_M1_UNEXPECTED_BEHAVIOR;
				break;
		}
	}
	if (PWRT_IsMotor2Used(pHandle))
	{
		switch (MDI_GetSTMState(pHandle->pMDI, M2))
		{
			case M_RUN:
				break;
			case M_FAULT_NOW:
			case M_FAULT_OVER:
				hVehicleFault |= VC_M2_FAULTS;
				break;
			default:
				hVehicleFault |= VC_M2_UNEXPECTED_BEHAVIOR;
				break;
		}
	}

	return hVehicleFault;
}

/**
	* @brief  Check for motor faults during stop state
	* @param  Powertrain handle
	* @retval Vehicle fault code
	*/
uint16_t PWRT_StopStateCheck(PWRT_Handle_t * pHandle)
{
	uint16_t hVehicleFault = 0;

	if (PWRT_IsMotor1Used(pHandle))
	{
		switch (MDI_GetSTMState(pHandle->pMDI, M1))
		{
			case M_RUN:
				break;
			case M_FAULT_NOW:
			case M_FAULT_OVER:
				hVehicleFault |= VC_M1_FAULTS;
				break;
			default:
				break;
		}
	}
	if (PWRT_IsMotor2Used(pHandle))
	{
		switch (MDI_GetSTMState(pHandle->pMDI, M2))
		{
			case M_RUN:
				break;
			case M_FAULT_NOW:
			case M_FAULT_OVER:
				hVehicleFault |= VC_M2_FAULTS;
				break;
			default:
				break;
		}
	}

	return hVehicleFault;
}

/**
	* @brief  Check if powertrain is active (e.g. motors are in running state)
	* @param  Powertrain handle
	* @retval Returns true if powertrain is active
	*/
bool PWRT_IsPowertrainActive(PWRT_Handle_t * pHandle)
{
	bool bActive = true;

	if (PWRT_IsMotor1Used(pHandle))
	{
		if (MDI_GetSTMState(pHandle->pMDI, M1) != M_RUN)
		{
			bActive = false;
		}
	}
	if (PWRT_IsMotor2Used(pHandle))
	{
		if (MDI_GetSTMState(pHandle->pMDI, M2) != M_RUN)
		{
			bActive = false;
		}
	}
	return bActive;
}

/**
	* @brief  Check if powertrain is stopped (e.g. motors are in idle state)
	* @param  Powertrain handle
	* @retval Returns true if powertrain is stopped
	*/
bool PWRT_IsPowertrainStopped(PWRT_Handle_t * pHandle)
{
	bool bStopped = true;

	if (PWRT_IsMotor1Used(pHandle))
	{
		switch (MDI_GetSTMState(pHandle->pMDI, M1))
		{
			case M_IDLE:
			case M_FAULT_NOW:
			case M_FAULT_OVER:
				break;
			default:
				bStopped = false;
				break;
		}
	}
	if (PWRT_IsMotor2Used(pHandle))
	{
		switch (MDI_GetSTMState(pHandle->pMDI, M2))
		{
			case M_IDLE:
			case M_FAULT_NOW:
			case M_FAULT_OVER:
				break;
			default:
				bStopped = false;
				break;
		}
	}
	return bStopped;
}

/**
	* @brief  Check if conditions to stop powertrain are met
	* @param  Powertrain handle
	* @retval Returns true if powertrain conditions to stop are met, false otherwise
	*/
bool PWRT_CheckStopConditions(PWRT_Handle_t * pHandle)
{
  /* bCheckStopX variable are stopping condition checks.
    By default they are false, but if a specific stopping condition is met they may toggle true. */
	bool bCheckStop1 = false;
	bool bCheckStop2 = false;
	bool bCheckStop3 = false;
	bool bCheckStop4 = false;
	//bool bCheckStop5 = false;
	bool bCheckStop6 = false;

	int32_t wSpeedM1 = MDI_GetAvrgMecSpeedUnit(pHandle->pMDI, M1);
	int32_t wSpeedM2 = MDI_GetAvrgMecSpeedUnit(pHandle->pMDI, M2);
	uint16_t hThrottleValue = THRO_GetAvThrottleValue(pHandle->pThrottle);

	if (PWRT_IsMotor1Used(pHandle))
	{
		if (abs(wSpeedM1) <= pHandle->sParameters.hStoppingSpeed) // If motor speed is lower than stopping speed parameter
		{
			bCheckStop1 = true;
		}
	}
	else
	{
		bCheckStop1 = true;
	}

	if (PWRT_IsMotor2Used(pHandle))
	{
		if (abs(wSpeedM2) <= pHandle->sParameters.hStoppingSpeed) // If motor speed is lower than stopping speed parameter
		{
			bCheckStop2 = true;
		}
	}
	else
	{
		bCheckStop2 = true;
	}

	if (!PWREN_IsPowerEnabled(pHandle->pPWREN)) // If power is enabled through power enable input
	{
		bCheckStop3 = true;
	}

	if (BRK_IsPressed(pHandle->pBrake)) // If brake is pressed
	{
		bCheckStop4 = true;
	}

	if (hThrottleValue < pHandle->sParameters.hStoppingThrottle) // If throttle value is lower than stopping throttle parameter
	{
		bCheckStop6 = true;
	}

	return (bCheckStop1 & bCheckStop2 & bCheckStop6) | bCheckStop3 | bCheckStop4; // Final logic to know if powertrain should be stopped.
}

/**
	* @brief  Check if conditions to start powertrain are met
	* @param  Powertrain handle
	* @retval Returns true if powertrain can be started
	*/
bool PWRT_CheckStartConditions(PWRT_Handle_t * pHandle)
{
  /* bCheckStartX variable are starting condition checks.
    By default they are false, but if a specific starting condition is met they may toggle true. */
	bool bCheckStart1 = false;
  bool bCheckStart2 = false;
  bool bCheckStart3 = false;

	uint16_t hThrottleValue = THRO_GetAvThrottleValue(pHandle->pThrottle);

	if (hThrottleValue > pHandle->sParameters.hStartingThrottle) // If throttle is higher than starting throttle parameter
	{
		bCheckStart1 = true;
	}
  if (PWREN_IsPowerEnabled(pHandle->pPWREN)) // If power is enabled through power enable input
	{
		bCheckStart2 = true;
	}
  if (!BRK_IsPressed(pHandle->pBrake)) // If brake is pressed
	{
		bCheckStart3 = true;
	}
	return bCheckStart1 && bCheckStart2 && bCheckStart3; // Final logic to know if powertrain should be started.
}

/**
	* @brief  Manage motor faults. Check if faults are still present and send motor fault acknowledge when faults are gone.
	* @param  Powertrain handle
	* @retval Returns true if a motor fault is still active, false if no more fault is present.
	*/
bool PWRT_MotorFaultManagement(PWRT_Handle_t * pHandle)
{
	uint16_t hM1FaultNowCode = MDI_GetCurrentFaults(pHandle->pMDI, M1);
	uint16_t hM1FaultOccurredCode = MDI_GetOccurredFaults(pHandle->pMDI, M1);
	uint16_t hM2FaultNowCode = MDI_GetCurrentFaults(pHandle->pMDI, M2);
	uint16_t hM2FaultOccurredCode = MDI_GetOccurredFaults(pHandle->pMDI, M2);

	bool bFaultNow = hM1FaultNowCode | hM2FaultNowCode;

	//If there's no current motor errors
	if (!bFaultNow)
	{
		if (PWRT_IsMotor1Used(pHandle))
		{// If there's an over current (OC) that has occurred but has already been cleared
			if (hM1FaultOccurredCode & MC_BREAK_IN)
			{
				if(pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M1] >= pHandle->sParameters.hFaultManagementTimeout)
				{// If the timer has timeout (500ms), clear the OC fault
					hM1FaultOccurredCode &= ~MC_BREAK_IN;
					pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M1] = 0;
				}
				else
				{//Increase the counter to count 20ms
					pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M1]++;
				}
			}

			if (hM1FaultOccurredCode & MC_SPEED_FDBK)
			{// If there's a speed feedback (SF) that has occurred but has already been cleared
				if(pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M1] >= pHandle->sParameters.hFaultManagementTimeout)
				{// If the timer has timeout (500ms), clear the SF fault
					hM1FaultOccurredCode &= ~MC_SPEED_FDBK;
					pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M1] = 0;
				}
				else
				{//Increase the counter to count 20ms
					pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M1]++;
				}
			}

			if (hM1FaultOccurredCode & MC_START_UP)
			{
				/* In case of motor startup failure... */
				if(pHandle->aFaultManagementCounters[STARTUP_COUNTER][M1] >= pHandle->sParameters.hFaultManagementTimeout)
				{// If the timer has timeout (500ms), clear the SF fault
					hM1FaultOccurredCode &= ~MC_START_UP;
					pHandle->aFaultManagementCounters[STARTUP_COUNTER][M1] = 0;
				}
				else
				{//Increase the counter to count 20ms
					pHandle->aFaultManagementCounters[STARTUP_COUNTER][M1]++;
				}
			}

			if (hM1FaultOccurredCode & MC_OVER_TEMP)
			{
				// In case of overtemperature, clear the OT fault
				hM1FaultOccurredCode &= ~MC_OVER_TEMP;
			}

			if (hM1FaultOccurredCode & MC_OVER_VOLT)
			{
				// In case of DCbus overvoltage, clear the OV fault
				hM1FaultOccurredCode &= ~MC_OVER_VOLT;
			}

			if (hM1FaultOccurredCode & MC_UNDER_VOLT)
			{
				// In case of DCbus undervoltage, clear the UV fault
				hM1FaultOccurredCode &= ~MC_UNDER_VOLT;
			}
		}

		if (PWRT_IsMotor2Used(pHandle))
		{
			if (hM2FaultOccurredCode & MC_BREAK_IN)
			{
				if(pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M2] >= pHandle->sParameters.hFaultManagementTimeout)
				{// If the timer has timeout (500ms), clear the OC fault
					hM2FaultOccurredCode &= ~MC_BREAK_IN;
					pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M2] = 0;
				}
				else
				{//Increase the counter to count 20ms
					pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M2]++;
				}
			}

			if (hM2FaultOccurredCode & MC_SPEED_FDBK)
			{// If there's a speed feedback (SF) that has occurred but has already been cleared
				if(pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M2] >= pHandle->sParameters.hFaultManagementTimeout)
				{// If the timer has timeout (500ms), clear the SF fault
					hM2FaultOccurredCode &= ~MC_SPEED_FDBK;
					pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M2] = 0;
				}
				else
				{//Increase the counter to count 20ms
					pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M2]++;
				}
			}

			if (hM2FaultOccurredCode & MC_START_UP)
			{// If there's a start-up (SU) that has occurred but has already been cleared
				if(pHandle->aFaultManagementCounters[STARTUP_COUNTER][M2] >= pHandle->sParameters.hFaultManagementTimeout)
				{// If the timer has timeout (500ms), clear the SF fault
					hM2FaultOccurredCode &= ~MC_START_UP;
					pHandle->aFaultManagementCounters[STARTUP_COUNTER][M2] = 0;
				}
				else
				{//Increase the counter to count 20ms
					pHandle->aFaultManagementCounters[STARTUP_COUNTER][M2]++;
				}
			}

			if (hM2FaultOccurredCode & MC_OVER_TEMP)
			{
				/* In case of overtemperature... */
				hM2FaultOccurredCode &= ~MC_OVER_TEMP;
			}

			if (hM2FaultOccurredCode & MC_OVER_VOLT)
			{
				/* In case of DCbus overvoltage... */
				hM2FaultOccurredCode &= ~MC_OVER_VOLT;
			}

			if (hM2FaultOccurredCode & MC_UNDER_VOLT)
			{
				/* In case of DCbus undervoltage... */
				hM2FaultOccurredCode &= ~MC_UNDER_VOLT;
			}
		}
	} // End of if (!bFaultNow)

	// Verify if all fault occured have been cleared
	bool bFaultOccured = (hM1FaultOccurredCode | hM2FaultOccurredCode);

	if (!bFaultOccured)
	{
		//todo: handle result from MDI_FaultAcknowledged below
		MDI_FaultAcknowledged(pHandle->pMDI, M1);
		MDI_FaultAcknowledged(pHandle->pMDI, M2);
	}
	return bFaultOccured;
}


//void PWRT_SetPASLevel(PWRT_Handle_t * pHandle, PAS_sLevel Level)
//{
//	 pHandle->pPAS->pLevel = Level;
//}

//PAS_sLevel PWRT_GetPASLevel (PWRT_Handle_t * pHandle)
//{
//	return pHandle->pPAS->pLevel;
//}

/**
	* @brief  Get main motor reference torque
	* @param  Powertrain handle
	* @retval Returns main motor reference torque
	*/
int16_t PWRT_GetTorqueRefMainMotor(PWRT_Handle_t * pHandle)
{
	return pHandle->aTorque[pHandle->bMainMotor];
}

/**
	* @brief  Get motor 1 reference torque
	* @param  Powertrain handle
	* @retval Returns motor 1 reference torque
	*/
int16_t PWRT_GetTorqueRefM1(PWRT_Handle_t * pHandle)
{
	return pHandle->aTorque[M1];
}

/**
	* @brief  Get motor 2 reference torque
	* @param  Powertrain handle
	* @retval Returns motor 2 reference torque
	*/
int16_t PWRT_GetTorqueRefM2(PWRT_Handle_t * pHandle)
{
	return pHandle->aTorque[M2];
}

/**
	* @brief  Get main motor speed reference
	* @param  Powertrain handle
	* @retval Returns main motor speed reference
	*/
int32_t PWRT_GetSpeedRefMainMotor(PWRT_Handle_t * pHandle)
{
	return pHandle->aSpeed[pHandle->bMainMotor];
}

/**
	* @brief  Get motor 1 speed reference
	* @param  Powertrain handle
	* @retval Returns motor 1 speed reference
	*/
int32_t PWRT_GetSpeedRefM1(PWRT_Handle_t * pHandle)
{
	return pHandle->aSpeed[M1];
}

/**
	* @brief  Get motor 2 speed reference
	* @param  Powertrain handle
	* @retval Returns motor 2 speed reference
	*/
int32_t PWRT_GetSpeedRefM2(PWRT_Handle_t * pHandle)
{
	return pHandle->aSpeed[M2];
}

/**
	* @brief  Check if motor 1 is used
	* @param  Powertrain handle
	* @retval Returns true if motor 1 is used
	*/
bool PWRT_IsMotor1Used(PWRT_Handle_t * pHandle)
{
	return pHandle->sParameters.bUseMotorM1;
}

/**
	* @brief  Check if motor 2 is used
	* @param  Powertrain handle
	* @retval Returns true if motor 2 is used
	*/
bool PWRT_IsMotor2Used(PWRT_Handle_t * pHandle)
{
	return pHandle->sParameters.bUseMotorM2;
}

///**
//	* @brief  Set Pedal Assist torque based on screen informations
//	* @param  Powertrain handle
//	* @retval pRefTorque in int16
//	*/
//int16_t PWRT_GetPASTorque(PWRT_Handle_t * pHandle)
//{
//	int16_t hRefTorque;
//	PAS_sLevel Got_Level;
//	Got_Level = PWRT_GetPASLevel(pHandle);
//
//	hRefTorque = (pHandle->sParameters.hPASMaxTorque / pHandle->pPAS->bMaxLevel) * Got_Level;
//
//	return hRefTorque;
//}

///**
//	* @brief  Set Pedal Assist speed based on screen informations
//	* @param  Powertrain handle
//	* @retval pRefTorque in int16
//	*/
//void PWRT_PASSetMaxSpeed(PWRT_Handle_t * pHandle)
//{
//	PAS_sLevel Got_Level;
//	Got_Level = PWRT_GetPASLevel(pHandle);
//
//	FLDBK_SetStartValue (&pHandle->sSpeedFoldback[M1], (pHandle->sParameters.hPASMaxSpeed / pHandle->pPAS->bMaxLevel) * Got_Level);
//	FLDBK_SetEndValue (&pHandle->sSpeedFoldback[M1], (pHandle->sParameters.hPASMaxSpeed / pHandle->pPAS->bMaxLevel) * Got_Level);

//	FLDBK_SetStartValue (&pHandle->sSpeedFoldback[M2], (pHandle->sParameters.hPASMaxSpeed / pHandle->pPAS->bMaxLevel) * Got_Level);
//	FLDBK_SetEndValue (&pHandle->sSpeedFoldback[M2], (pHandle->sParameters.hPASMaxSpeed / pHandle->pPAS->bMaxLevel) * Got_Level);
//}

///**
//	* @brief  Set Pedal Assist torque based on the Torque Sensor
//	* @param  Powertrain handle
//	* @retval pRefTorqueS in int16
//	*/
//int16_t PWRT_GetTorqueFromTS(PWRT_Handle_t * pHandle)
//{
//	int16_t hRefTorqueS, hReadTS;
//	PAS_sLevel Got_Level;
//	hReadTS = TS_ToMotorTorque(pHandle->pPAS->pTorque);
//	Got_Level = PWRT_GetPASLevel(pHandle);
//
//	hRefTorqueS = (hReadTS * Got_Level) / pHandle->pPAS->bMaxLevel;
//
//	if (hRefTorqueS < pHandle->pPAS->bMaxTorque)
//	{
//		hRefTorqueS = pHandle->pPAS->bMaxTorque;
//	}
//	return hRefTorqueS;
//}
///**
//	* @brief  Select Control assistance based on Throttle or PAS
//	* @param  Powertrain handle
//	* @retval pHandle->pTorqueSelect in int16
//	*/
//int16_t PWRT_CalcSelectedTorque(PWRT_Handle_t * pHandle)
//{
//
//	/* PAS and Throttle management */
//	if (PAS_IsPASDetected(pHandle->pPAS) && !THRO_IsThrottleDetected(pHandle->pThrottle))
//	{
//		/* Torque sensor enabled */
//		if (pHandle->pPAS->bTorqueSensorUse)
//		{
//				pHandle->hTorqueSelect = PWRT_GetTorqueFromTS(pHandle);
//				PWRT_PASSetMaxSpeed(pHandle);
//		}
//		else
//		{
//				pHandle->hTorqueSelect= PWRT_GetPASTorque(pHandle);
//				PWRT_PASSetMaxSpeed(pHandle);
//		}
//	}
//	else
//	{
//		/* Throttle value convert to torque */
//		pHandle->hTorqueSelect = THRO_ThrottleToTorque(pHandle->pThrottle);
//	}
//	return pHandle->hTorqueSelect;
//}
