/**
  * @file    powertrain_management.c
  * @author  Sami Bouzid, FTEX
  * @author  Jorge Polo, FTEX
  * @brief   This module handles management of the vehicle powertrain
  *
*/

#include "powertrain_management.h"

#define OVERCURRENT_COUNTER 		0
#define STARTUP_COUNTER					1
#define SPEEDFEEDBACK_COUNTER		2

/* Functions ---------------------------------------------------- */

/**
	* @brief  Module initialization, to be called once before using it
	* @param  Powertrain handle
	* @retval None
	*/
void PWRT_Init(PWRT_Handle_t * pHandle, MotorControlInterfaceHandle_t * pMci_M1)
{
    ASSERT(pHandle != NULL);
    MDI_Init(pHandle->pMDI, pMci_M1);
    Throttle_Init(pHandle->pThrottle);
    BRK_Init(pHandle->pBrake);
    MS_Init(pHandle->pMS);
    PWREN_Init(pHandle->pPWREN);
		
	PedalSpdSensor_Init(pHandle->pPSS);
	PedalTorqSensor_Init(pHandle->pPTS);
	WheelSpdSensor_Init(pHandle->pWSS);
	
    Foldback_Init(&pHandle->SpeedFoldbackStartupDualMotor);

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
    ASSERT(pHandle != NULL);
    /* Update throttle sensor handle */
    Throttle_CalcAvThrottleValue(pHandle->pThrottle);
}

/**
	* @brief  Compute target torque and speed to be applied to each motors and store value into powertrain handle.
	* @param  Powertrain handle
	* @retval None
	*/
void PWRT_CalcMotorTorqueSpeed(PWRT_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
	bool bIsBrakePressed = BRK_IsPressed(pHandle->pBrake);
    
	/*bool bIsPwrEnabled = PWREN_IsPowerEnabled(pHandle->pPWREN);*/
    int16_t hSpeedM1 = MDI_GetAvrgMecSpeedUnit(pHandle->pMDI, M1);
    int16_t hSpeedM2 = MDI_GetAvrgMecSpeedUnit(pHandle->pMDI, M2);

	MotorSelection_t bMotorSelection = MS_CheckSelection(pHandle->pMS); // Check which motor is selected
	int16_t hTorqueRef = 0; 
    int32_t hSpeedRef = 0;
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
    
    // Reset to zero before placing computed value again
    pHandle->aTorque[M1] = 0;
    pHandle->aTorque[M2] = 0;
    pHandle->aSpeed[M1] = 0;
    pHandle->aSpeed[M2] = 0;

	if (pHandle->sParameters.bCtrlType == TORQUE_CTRL) // If torque control
	{
		hTorqueRef = Throttle_ThrottleToTorque(pHandle->pThrottle); // Translate throttle value to powertrain target torque value
        hAux = hTorqueRef; //hAux is used as auxialiary variable for final torque computation. Will be reduced depending on foldback and brake state for example.
        
		if (bIsBrakePressed)
		{
			hAux = 0;
		}
		if(pHandle->sParameters.bMode == SINGLE_MOTOR)
		{
            if (pHandle->sParameters.bEnableDualMotorStartup)
            {
                // Execute dual motor startup strategy...
                // TODO: Following foldbacks are computed using speed of M1 or M2. Should use vehicle speed instead when it is ready.
                if (pHandle->bMainMotor == M1)
                {
                    pHandle->aTorque[M1] = hAux;
                    hAux = Foldback_ApplyFoldback(&pHandle->SpeedFoldbackStartupDualMotor, hAux, abs(hSpeedM1));
                    // Store powertrain target torque value in handle. Invert torque if needed.
                    pHandle->aTorque[M2] = pHandle->sParameters.bM2TorqueInversion ? -hAux : hAux;
                }
                else if (pHandle->bMainMotor == M2)
                {
                    // Store powertrain target torque value in handle. Invert torque if needed.
                    pHandle->aTorque[M2] = pHandle->sParameters.bM2TorqueInversion ? -hAux : hAux;
                    hAux = Foldback_ApplyFoldback(&pHandle->SpeedFoldbackStartupDualMotor, hAux, abs(hSpeedM2));
                    pHandle->aTorque[M1] = hAux;
                }
                else
                {
                }
            }
            else
            {
                pHandle->aTorque[pHandle->bMainMotor] = hAux; // Store powertrain target torque value in handle
            }
		}
		else if(pHandle->sParameters.bMode == DUAL_MOTOR)
		{
            // Apply same torque to both motors...
            pHandle->aTorque[M1] = hAux;
            // Store powertrain target torque value in handle. Invert torque if needed.
            pHandle->aTorque[M2] = pHandle->sParameters.bM2TorqueInversion ? -hAux : hAux;
        }
        else
        {
        }
	}
	else if (pHandle->sParameters.bCtrlType == SPEED_CTRL)
	{
		/* SPEED CONTROL NOT IMPLEMENTED YET. JUST PLACEHOLDER CODE */
		hSpeedRef = Throttle_ThrottleToSpeed(pHandle->pThrottle);
		if (bIsBrakePressed)
		{
			hSpeedRef = 0;
		}

		if(pHandle->sParameters.bMode == SINGLE_MOTOR)
		{
			pHandle->aSpeed[pHandle->bMainMotor] = (int16_t) hSpeedRef;
		}
		if(pHandle->sParameters.bMode == DUAL_MOTOR)
		{
			pHandle->aSpeed[M1] = (int16_t) hSpeedRef;
			pHandle->aSpeed[M2] = (int16_t) hSpeedRef;
		}
    }
    else
    {
    }
}

/**
	* @brief  Send torque and/or speed ramp commands to motors
	* @param  Powertrain handle
	* @retval None
	*/
void PWRT_ApplyMotorRamps(PWRT_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
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
    ASSERT(pHandle != NULL);
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
    ASSERT(pHandle != NULL);
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
    ASSERT(pHandle != NULL);
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
    ASSERT(pHandle != NULL);
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
    ASSERT(pHandle != NULL);
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
    ASSERT(pHandle != NULL);
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
    ASSERT(pHandle != NULL);
	bool bM1Active = false;
    bool bM2Active = false;

	if (PWRT_IsMotor1Used(pHandle))
	{
		if (MDI_GetSTMState(pHandle->pMDI, M1) == M_RUN)
		{
			bM1Active = true;
		}
	}
    else
    {
        bM1Active = false;
    }
	if (PWRT_IsMotor2Used(pHandle))
	{
		if (MDI_GetSTMState(pHandle->pMDI, M2) == M_RUN)
		{
			bM2Active = true;
		}
	}
    else
    {
        bM2Active = false;
    }
    
	return bM1Active & bM2Active;
}

/**
	* @brief  Check if powertrain is stopped (e.g. motors are in idle state)
	* @param  Powertrain handle
	* @retval Returns true if powertrain is stopped
	*/
bool PWRT_IsPowertrainStopped(PWRT_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
	bool bM1Stopped = false;
    bool bM2Stopped = false;

	if (PWRT_IsMotor1Used(pHandle))
	{
		switch (MDI_GetSTMState(pHandle->pMDI, M1))
		{
			case M_IDLE:
			case M_FAULT_NOW:
			case M_FAULT_OVER:
                bM1Stopped = true;
				break;
			default:
				
				break;
		}
	}
    else
    {
        bM1Stopped = true;
    }
	if (PWRT_IsMotor2Used(pHandle))
	{
		switch (MDI_GetSTMState(pHandle->pMDI, M2))
		{
			case M_IDLE:
			case M_FAULT_NOW:
			case M_FAULT_OVER:
                bM2Stopped = true;
				break;
			default:
				
				break;
		}
	}
    else
    {
        bM2Stopped = true;
    }
	return bM1Stopped & bM2Stopped;
}

/**
	* @brief  Check if conditions to stop powertrain are met
	* @param  Powertrain handle
	* @retval Returns true if powertrain conditions to stop are met, false otherwise
	*/
bool PWRT_CheckStopConditions(PWRT_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    /* bCheckStopX variable are stopping condition checks.
    By default they are false, but if a specific stopping condition is met they may toggle true. */
	bool bCheckStop1 = false;
	bool bCheckStop2 = false;
	bool bCheckStop3 = false;
	bool bCheckStop4 = false;
	bool bCheckStop5 = false;

	int32_t wSpeedM1 = MDI_GetAvrgMecSpeedUnit(pHandle->pMDI, M1);
	int32_t wSpeedM2 = MDI_GetAvrgMecSpeedUnit(pHandle->pMDI, M2);
	uint16_t hThrottleValue = Throttle_GetAvThrottleValue(pHandle->pThrottle);

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
		bCheckStop5 = true;
	}

	return (bCheckStop1 & bCheckStop2 & bCheckStop5) | bCheckStop3 | bCheckStop4; // Final logic to know if powertrain should be stopped.
}

/**
	* @brief  Check if conditions to start powertrain are met
	* @param  Powertrain handle
	* @retval Returns true if powertrain can be started
	*/
bool PWRT_CheckStartConditions(PWRT_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    /* bCheckStartX variable are starting condition checks.
    By default they are false, but if a specific starting condition is met they may toggle true. */
	bool bCheckStart1 = false;
    bool bCheckStart2 = false;
    bool bCheckStart3 = false;

	uint16_t hThrottleValue = Throttle_GetAvThrottleValue(pHandle->pThrottle);

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
	return bCheckStart1 & bCheckStart2 & bCheckStart3; // Final logic to know if powertrain should be started.
}

/**
	* @brief  Manage motor faults. Check if faults are still present and send motor fault acknowledge when faults are gone.
	* @param  Powertrain handle
	* @retval Returns true if a motor fault is still active, false if no more fault is present.
	*/
bool PWRT_MotorFaultManagement(PWRT_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
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
			if ((hM1FaultOccurredCode & MC_BREAK_IN) | (hM1FaultOccurredCode & MC_OCSP))
			{
				if(pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M1] >= pHandle->sParameters.hFaultManagementTimeout)
				{// If the timer has timeout, clear the OC fault
					hM1FaultOccurredCode &= ~MC_BREAK_IN;
                    hM1FaultOccurredCode &= ~MC_OCSP;
					pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M1] = 0;
				}
				else
				{//Increase the counter one more tick
					pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M1]++;
				}
			}

			if (hM1FaultOccurredCode & MC_SPEED_FDBK)
			{// If there's a speed feedback (SF) that has occurred but has already been cleared
				if(pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M1] >= pHandle->sParameters.hFaultManagementTimeout)
				{// If the timer has timeout, clear the SF fault
					hM1FaultOccurredCode &= ~MC_SPEED_FDBK;
					pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M1] = 0;
				}
				else
				{//Increase the counter one more tick
					pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M1]++;
				}
			}

			if (hM1FaultOccurredCode & MC_START_UP)
			{
				/* In case of motor startup failure... */
				if(pHandle->aFaultManagementCounters[STARTUP_COUNTER][M1] >= pHandle->sParameters.hFaultManagementTimeout)
				{// If the timer has timeout, clear the SF fault
					hM1FaultOccurredCode &= ~MC_START_UP;
					pHandle->aFaultManagementCounters[STARTUP_COUNTER][M1] = 0;
				}
				else
				{//Increase the counter one more tick
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
			if ((hM2FaultOccurredCode & MC_BREAK_IN) || (hM2FaultOccurredCode & MC_OCSP))
			{
				if(pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M2] >= pHandle->sParameters.hFaultManagementTimeout)
				{// If the timer has timeout, clear the OC fault
					hM2FaultOccurredCode &= ~MC_BREAK_IN;
                    hM2FaultOccurredCode &= ~MC_OCSP;
					pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M2] = 0;
				}
				else
				{//Increase the counter one more tick
					pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M2]++;
				}
			}

			if (hM2FaultOccurredCode & MC_SPEED_FDBK)
			{// If there's a speed feedback (SF) that has occurred but has already been cleared
				if(pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M2] >= pHandle->sParameters.hFaultManagementTimeout)
				{// If the timer has timeout, clear the SF fault
					hM2FaultOccurredCode &= ~MC_SPEED_FDBK;
					pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M2] = 0;
				}
				else
				{//Increase the counter one more tick
					pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M2]++;
				}
			}

			if (hM2FaultOccurredCode & MC_START_UP)
			{// If there's a start-up (SU) that has occurred but has already been cleared
				if(pHandle->aFaultManagementCounters[STARTUP_COUNTER][M2] >= pHandle->sParameters.hFaultManagementTimeout)
				{// If the timer has timeout, clear the SF fault
					hM2FaultOccurredCode &= ~MC_START_UP;
					pHandle->aFaultManagementCounters[STARTUP_COUNTER][M2] = 0;
				}
				else
				{//Increase the counter one more tick
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


void PWRT_SetAssistLevel(PWRT_Handle_t * pHandle, uint8_t bLevel)
{
    ASSERT(pHandle != NULL);
	pHandle->bCurrentAssistLevel = bLevel;
}

uint8_t PWRT_GetAssistLevel (PWRT_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
	return pHandle->bCurrentAssistLevel;
}

/**
	* @brief  Get main motor reference torque
	* @param  Powertrain handle
	* @retval Returns main motor reference torque
	*/
int16_t PWRT_GetTorqueRefMainMotor(PWRT_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
	return pHandle->aTorque[pHandle->bMainMotor];
}

/**
	* @brief  Get motor 1 reference torque
	* @param  Powertrain handle
	* @retval Returns motor 1 reference torque
	*/
int16_t PWRT_GetTorqueRefM1(PWRT_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
	return pHandle->aTorque[M1];
}

/**
	* @brief  Get motor 2 reference torque
	* @param  Powertrain handle
	* @retval Returns motor 2 reference torque
	*/
int16_t PWRT_GetTorqueRefM2(PWRT_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
	return pHandle->aTorque[M2];
}

/**
	* @brief  Get main motor speed reference
	* @param  Powertrain handle
	* @retval Returns main motor speed reference
	*/
int32_t PWRT_GetSpeedRefMainMotor(PWRT_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
	return pHandle->aSpeed[pHandle->bMainMotor];
}

/**
	* @brief  Get motor 1 speed reference
	* @param  Powertrain handle
	* @retval Returns motor 1 speed reference
	*/
int32_t PWRT_GetSpeedRefM1(PWRT_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
	return pHandle->aSpeed[M1];
}

/**
	* @brief  Get motor 2 speed reference
	* @param  Powertrain handle
	* @retval Returns motor 2 speed reference
	*/
int32_t PWRT_GetSpeedRefM2(PWRT_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
	return pHandle->aSpeed[M2];
}

/**
	* @brief  Check if motor 1 is used
	* @param  Powertrain handle
	* @retval Returns true if motor 1 is used
	*/
bool PWRT_IsMotor1Used(PWRT_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
	return pHandle->sParameters.bUseMotorM1;
}

/**
	* @brief  Check if motor 2 is used
	* @param  Powertrain handle
	* @retval Returns true if motor 2 is used
	*/
bool PWRT_IsMotor2Used(PWRT_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
	return pHandle->sParameters.bUseMotorM2;
}


