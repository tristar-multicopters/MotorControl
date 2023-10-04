/**
  * @file    powertrain_management.c
  * @author  FTEX
  * @brief   This module handles management of the vehicle powertrain
  *
  */

// ============================= Includes ================================ //
#include "powertrain_management.h"
#include "vc_tasks.h"
#include "firmware_update.h"
#include "vc_errors_management.h"
// ============================= Defines ================================ //
#define OVERCURRENT_COUNTER         0
#define STARTUP_COUNTER             1
#define SPEEDFEEDBACK_COUNTER       2
#define STUCK_REVERSE_COUNTER       3
#define UNDERVOLTAGE_COUNTER        4
#define MOTOR_ERROR_TYPE_COUNT      7

#define MAXCURRENT                  MAX_MEASURABLE_CURRENT /* Used for a generic conversion 
                                                              from current ref to actual amps */
// ============================= Variables ================================ //
                                  
bool isPWMCleared;
static Delay_Handle_t ThrottleDelay; // Delay for Throttle stuck check while initialization
static Delay_Handle_t PTSensorDelay; // Delay for Pedal Torque sensor stuck check while initialization
static Delay_Handle_t brakeDelay;    // Delay for Brake sensor stuck check while initialization

// ==================== Public function prototypes ======================== //

/**
  * @brief  Module initialization, to be called once before using it
  * @param  Powertrain handle
  * @retval None
  */
void PWRT_Init(PWRT_Handle_t * pHandle, MotorControlInterfaceHandle_t * pMci_M1, SlaveMotorHandle_t * pSlaveM2, Delay_Handle_t pDelayArray[])
{
    ASSERT(pHandle != NULL);     
    
    // Initialize Delays for stuck conditions
    ThrottleDelay = pDelayArray[THROTTLE_DELAY];
    PTSensorDelay = pDelayArray[PTS_DELAY];
    brakeDelay = pDelayArray[BRAKE_DELAY];

    // Initilaize peripherals
    Wheel_Init(WHEEL_DIAMETER_DEFAULT);
    MDI_Init(pHandle->pMDI, pMci_M1, pSlaveM2);
    Throttle_Init(pHandle->pThrottle,&ThrottleDelay);
    BRK_Init(pHandle->pBrake, &brakeDelay);
    BatMonitor_Init(pHandle->pBatMonitorHandle, pHandle->pMDI->pMCI);
    MS_Init(pHandle->pMS);
    PWREN_Init(pHandle->pPWREN);
    Light_Init(pHandle->pHeadLight);
    Light_Init(pHandle->pTailLight);
    PedalAssist_Init(pHandle->pPAS, &PTSensorDelay);    

    pHandle->aTorque[M1] = 0; pHandle->aTorque[M2] = 0;
    pHandle->aSpeed[M1] = 0; pHandle->aSpeed[M2] = 0;
    pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M1] = 0; pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M2] = 0;
    pHandle->aFaultManagementCounters[STARTUP_COUNTER][M1] = 0; pHandle->aFaultManagementCounters[STARTUP_COUNTER][M2] = 0;
    pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M1] = 0; pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M2] = 0;
    pHandle->aFaultManagementCounters[STUCK_REVERSE_COUNTER][M1] = 0; pHandle->aFaultManagementCounters[STUCK_REVERSE_COUNTER][M2] = 0;   
    
    pHandle->sParameters.CruiseForceDisengage = false;
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
    bool bIsBrakePressed = BRK_IsPressedSafety(pHandle->pBrake);
      
    MotorSelection_t bMotorSelection = MS_CheckSelection(pHandle->pMS); // Check which motor is selected
    int16_t hTorqueRef = 0; 
    uint16_t hSpeedRef = 0;
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
        PedalTorqSensor_CalcAvValue(pHandle->pPAS->pPTS); // Calculate the pedal assist torque sensor value
        
        hTorqueRef = PWRT_CalcSelectedTorque(pHandle); // Compute torque to motor depending on either throttle or PAS
        hAux = hTorqueRef; //hAux is used as auxialiary variable for final torque computation. Will be reduced depending on brake state.
                
        if (bIsBrakePressed)
        {
            hAux = 0;
            
            //Reset passed detection
            pHandle->pPAS->bPASDetected = false;
            
            // Reset All the Pedal Assist Parameters
            PedalAssist_ResetParameters(pHandle->pPAS);
            PWRT_ForceDisengageCruiseControl(pHandle);
        }
        
        if((pHandle->pPAS->bCurrentPasAlgorithm      == CadenceSensorUse) &&  // If the user pedals while were are in cruise
           (PedalAssist_IsPASDetected(pHandle->pPAS) == true) && 
           (PWRT_GetCruiseControlState(pHandle)      == true))
        {
            hAux = 0;                                  // Exit cruise control
            pHandle->pPAS->bPASDetected = false;
            PedalAssist_ResetParameters(pHandle->pPAS);
            PWRT_ForceDisengageCruiseControl(pHandle);
        }
        
        /* Throttle and walk mode always have higher priority over PAS but 
           the priority between walk mode and throttle depends on the value 
           of the parameter WalkmodeOverThrottle  */    
                 
            /* Using PAS or walk mode */
        if (((PedalAssist_IsPASDetected(pHandle->pPAS) && !Throttle_IsThrottleDetected(pHandle->pThrottle)) || 
             (PedalAssist_IsWalkModeDetected(pHandle->pPAS) && (!Throttle_IsThrottleDetected(pHandle->pThrottle) || pHandle->pPAS->sParameters.WalkmodeOverThrottle))))
        {             
            uint16_t TopSpeed = 0;
            PedalAssist_PASUpdateMaxSpeed(pHandle->pPAS); // Make sure we have the most up-to-date desired top speed
            
            TopSpeed = PedalAssist_GetPASMaxSpeed(pHandle->pPAS);
            PWRT_SetNewTopSpeed(pHandle,TopSpeed);        // Tell motor control what is our desired top speed       

            #if VEHICLE_SELECTION == VEHICLE_NIDEC
            if (pHandle->pPAS->bCurrentPasAlgorithm == TorqueSensorUse)
            {
                static int16_t PowerAvg = 0;                
                                   
                int16_t BandwidthUp = 35; //Bandwidth CANNOT be set to 0
                int16_t BandwidthDown = 75;    
                int16_t Bandwidth = 0;
                
                    
                if(abs(PowerAvg - hAux) > (pHandle->pPAS->sParameters.hPASMaxTorque/25)) // Sudden acceleration or decelration ? 
                {                                                                      
                    BandwidthUp = 60;
                    BandwidthDown = 30;                    
                }
                
                if(hAux >= PowerAvg)
                {
                    Bandwidth = BandwidthUp;
                }
                else
                {                    
                    Bandwidth = BandwidthDown;
                }
                
                PowerAvg = ((Bandwidth-1) * PowerAvg + hAux)/Bandwidth;
                                        
                hAux = PowerAvg;
            }
            #endif
        }                            
        else if(Throttle_IsThrottleDetected(pHandle->pThrottle))
        {
            uint16_t TopSpeed = 0;
                          
            TopSpeed = Throttle_GetMaxSpeed(pHandle->pThrottle);   // Get the current desired top speed for throttle
            PWRT_SetNewTopSpeed(pHandle,TopSpeed);                 // Tell motor control what is our desired top speed 
        }   
        
        // Store powertrain target torque value in handle
        pHandle->aTorque[pHandle->bMainMotor] = hAux;
        
        if(pHandle->sParameters.bMode == DUAL_MOTOR)
        {
            // Apply same torque to both motors..
            // Store powertrain target torque value in handle. Invert torque if needed.
            pHandle->aTorque[M2] = pHandle->sParameters.bM2TorqueInversion ? -hAux : hAux;
        }
    }
    else if (pHandle->sParameters.bCtrlType == SPEED_CTRL)
    {
        
        // get speed value from Throttle
        hSpeedRef = Throttle_ThrottleToSpeed(pHandle->pThrottle);
        
        // check if PAS detected and Throttle not detected
        if (PedalAssist_IsPASDetected(pHandle->pPAS) && !Throttle_IsThrottleDetected(pHandle->pThrottle)) 
        {
            /* this part commited because of a warning. TO DO: just get speed reference from PAS and convert it to Motor RPM

            
            // get speed value from PAS
            //uint16_t gearRatio = (uint16_t)(pHandle->sParameters.MotorToHubGearRatio >> 16);
            
            //uint16_t PASRequestedSpeed = (uint16_t)(UserConfigTask_GetCadenceHybridLevelSpeed(pHandle->pPAS->bCurrentAssistLevel)-1);
            //hSpeedRef = gearRatio * Wheel_GetWheelRpmFromSpeed(PASRequestedSpeed);
            */
        }
        
        // Check if the calculated speed is more than maximum allowed Speed set by LCD
        if (hSpeedRef > pHandle->pThrottle->hParameters.MaxThrottleSpeedKMH)
        {
            hSpeedRef = pHandle->pThrottle->hParameters.MaxThrottleSpeedKMH;
        }
        
        // set speed zero if brake is pressed
        if (bIsBrakePressed)
        {
            hSpeedRef = 0;
        }
        else
        {
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
        bM1Active = true;
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
        bM2Active = true;
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
    bool bCheckStop6 = false;
    
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

    if (!PWREN_IsPowerEnabled(pHandle->pPWREN) || BatMonitor_GetLowBatFlag(pHandle->pBatMonitorHandle)) // If power is not enabled through power enable input
    {                                                                                                   // Or the low battery flag is raised
        bCheckStop3 = true;
    }

    if (!pHandle->pPAS->bPASDetected && !PedalAssist_IsWalkModeDetected(pHandle->pPAS)) // If there is no PAS  or walk mode detected
    {
        bCheckStop5 = true;
    }
    
    if (hThrottleValue < pHandle->sParameters.hStoppingThrottle) // If throttle value is lower than stopping throttle parameter
    {
        bCheckStop6 = true;
    }

    return (bCheckStop1 & bCheckStop2 & bCheckStop5& bCheckStop6) | bCheckStop3 | bCheckStop4; // Final logic to know if powertrain should be stopped.
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

    //check if a firmware update is going. firmware update true block start condition.
    if ((hThrottleValue > pHandle->sParameters.hStartingThrottle) || (pHandle->pPAS->bPASDetected) || PedalAssist_IsWalkModeDetected(pHandle->pPAS)) // If throttle is higher than starting throttle parameter
    {
        bCheckStart1 = true;
    }
    
    //if a firmware update is running or device is going off stop motors to start.
    if(FirmwareUpdate_Running() || PWREN_GetGoingOffFlag(pHandle->pPWREN))
    {
        bCheckStart1 = false;
    }
    
    if (PWREN_IsPowerEnabled(pHandle->pPWREN) && !BatMonitor_GetLowBatFlag(pHandle->pBatMonitorHandle)) // If power is enabled through power enable input
    {                                                                                                   // And the low battery flag isn't raised  
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
    uint32_t wM1FaultOccurredCode = MDI_GetOccurredFaults(pHandle->pMDI, M1);
    uint32_t wM2FaultOccurredCode = MDI_GetOccurredFaults(pHandle->pMDI, M2);

    uint32_t wFaultOccurred = wM1FaultOccurredCode | wM2FaultOccurredCode;

    if (wFaultOccurred != MC_NO_ERROR)      // Rasie Motor current error to the LCD
    {
        if (((wFaultOccurred & MC_BREAK_IN) | (wFaultOccurred & MC_OCSP)) != MC_NO_ERROR )
        {
            VC_Errors_RaiseError(OVER_CURRENT, HOLD_UNTIL_CLEARED);
        }
        if ((wFaultOccurred & MC_OVER_TEMP_CONTROLLER) != MC_NO_ERROR)
        {
            VC_Errors_RaiseError(CONTROLLER_OT_PROTECT, DEFAULT_HOLD_FRAMES);
        }
        if ((wFaultOccurred & MC_OVER_TEMP_MOTOR) != MC_NO_ERROR)
        {
            VC_Errors_RaiseError(MOTOR_OT_PROTECT, DEFAULT_HOLD_FRAMES);
        }
        if ((wFaultOccurred & MC_OVER_VOLT)!= MC_NO_ERROR)
        {
            VC_Errors_RaiseError(OV_PROTECTION, DEFAULT_HOLD_FRAMES);
        }
        if ((wFaultOccurred & MC_UNDER_VOLT)!= MC_NO_ERROR)
        {
            VC_Errors_RaiseError(UV_PROTECTION, DEFAULT_HOLD_FRAMES);
        }
        if ((wFaultOccurred & MC_NTC_FREEZE_CONTROLLER)!= MC_NO_ERROR)
        {
            VC_Errors_RaiseError(UT_PROTECTION, DEFAULT_HOLD_FRAMES);
        }
				
    }
    if (PWRT_IsMotor1Used(pHandle))
    {// If there's an over current (OC) that has occurred but has already been cleared
        if ((wM1FaultOccurredCode & MC_BREAK_IN) | (wM1FaultOccurredCode & MC_OCSP))
        {
            if(pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M1] >= pHandle->sParameters.hFaultManagementTimeout)
            {// If the timer has timeout, clear the OC fault
                wM1FaultOccurredCode &= ~MC_BREAK_IN;
                wM1FaultOccurredCode &= ~MC_OCSP;
                pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M1] = 0;
                VC_Errors_ClearError(OVER_CURRENT);
            }
            else
            {//Increase the counter one more tick
                pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M1]++;
            }
        }

        if (wM1FaultOccurredCode & MC_SPEED_FDBK)
        {// If there's a speed feedback (SF) that has occurred but has already been cleared
            if(pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M1] >= pHandle->sParameters.hFaultManagementTimeout)
            {// If the timer has timeout, clear the SF fault
                wM1FaultOccurredCode &= ~MC_SPEED_FDBK;
                pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M1] = 0;
            }
            else
            {//Increase the counter one more tick
                pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M1]++;
            }
        }

        if ((wM1FaultOccurredCode & MC_OVER_TEMP_CONTROLLER) != 0)
        {
            // In case of controller overtemperature, clear the OT fault
            wM1FaultOccurredCode &= ~MC_OVER_TEMP_CONTROLLER;
        }
        
        if ((wM1FaultOccurredCode & MC_OVER_TEMP_MOTOR) != 0)
        {
            // In case of motor overtemperature, clear the OT fault
            wM1FaultOccurredCode &= ~MC_OVER_TEMP_MOTOR;
        }

        if ((wM1FaultOccurredCode & MC_OVER_VOLT) != 0)
        {
            // In case of DCbus overvoltage, clear the OV fault
            wM1FaultOccurredCode &= ~MC_OVER_VOLT;
        }

        if ((wM1FaultOccurredCode & MC_UNDER_VOLT) != 0)
        {
            if(pHandle->aFaultManagementCounters[UNDERVOLTAGE_COUNTER][M1] >= pHandle->sParameters.hFaultManagementTimeout)
            {
                // If the timer has timeout, clear the UV fault
                wM1FaultOccurredCode &= ~MC_UNDER_VOLT;
                pHandle->aFaultManagementCounters[UNDERVOLTAGE_COUNTER][M1] = 0;
            }
            else
            {//Increase the counter one more tick
                pHandle->aFaultManagementCounters[UNDERVOLTAGE_COUNTER][M1]++;
            }
        }

        if ((wM1FaultOccurredCode & MC_MSRP) != 0)
        {// If there's a Motor StuckReverse feedback (MSRP) that has occurred but has already been cleared
            if(pHandle->aFaultManagementCounters[STUCK_REVERSE_COUNTER][M1] >= pHandle->sParameters.hFaultManagementTimeout)
            {// If the timer has timeout, clear the MSRP fault
                wM1FaultOccurredCode &= ~MC_MSRP;
                pHandle->aFaultManagementCounters[STUCK_REVERSE_COUNTER][M1] = 0;
            }
            else
            {//Increase the counter one more tick
                pHandle->aFaultManagementCounters[STUCK_REVERSE_COUNTER][M1]++;
            }
        }

        if ((wM1FaultOccurredCode & MC_NTC_FREEZE_CONTROLLER) != 0)
        {
            wM1FaultOccurredCode &= ~MC_NTC_FREEZE_CONTROLLER;
        }

        if ((wM1FaultOccurredCode & MC_FOC_DURATION) != 0)
        {
            wM1FaultOccurredCode &= ~MC_FOC_DURATION;
        }
    }

    if (PWRT_IsMotor2Used(pHandle))
    {
        if ((wM2FaultOccurredCode & MC_BREAK_IN) || (wM2FaultOccurredCode & MC_OCSP))
        {
            if(pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M2] >= pHandle->sParameters.hFaultManagementTimeout)
            {// If the timer has timeout, clear the OC fault
                wM2FaultOccurredCode &= ~MC_BREAK_IN;
                wM2FaultOccurredCode &= ~MC_OCSP;
                pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M2] = 0;
                VC_Errors_ClearError(OVER_CURRENT);
            }
            else
            {//Increase the counter one more tick
                pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M2]++;
            }
        }

        if ((wM2FaultOccurredCode & MC_SPEED_FDBK) != 0)
        {// If there's a speed feedback (SF) that has occurred but has already been cleared
            if(pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M2] >= pHandle->sParameters.hFaultManagementTimeout)
            {// If the timer has timeout, clear the SF fault
                wM2FaultOccurredCode &= ~MC_SPEED_FDBK;
                pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M2] = 0;
            }
            else
            {//Increase the counter one more tick
                pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M2]++;
            }
        }

        if ((wM2FaultOccurredCode & MC_OVER_TEMP_CONTROLLER) != 0)
        {
            /* In case of controller overtemperature... */
            wM2FaultOccurredCode &= ~MC_OVER_TEMP_CONTROLLER;
        }
        
        if ((wM2FaultOccurredCode & MC_OVER_TEMP_MOTOR) != 0)
        {
            /* In case of motor overtemperature... */
            wM2FaultOccurredCode &= ~MC_OVER_TEMP_MOTOR;
        }

        if (wM2FaultOccurredCode & MC_OVER_VOLT)
        {
            /* In case of DCbus overvoltage... */
            wM2FaultOccurredCode &= ~MC_OVER_VOLT;
        }

        if ((wM2FaultOccurredCode & MC_UNDER_VOLT) != 0)
        {
            if(pHandle->aFaultManagementCounters[UNDERVOLTAGE_COUNTER][M2] >= pHandle->sParameters.hFaultManagementTimeout)
            {
                // If the timer has timeout, clear the UV fault
                wM2FaultOccurredCode &= ~MC_UNDER_VOLT;
                pHandle->aFaultManagementCounters[UNDERVOLTAGE_COUNTER][M2] = 0;
            }
            else
            {//Increase the counter one more tick
                pHandle->aFaultManagementCounters[UNDERVOLTAGE_COUNTER][M2]++;
            }
        }

        if ((wM2FaultOccurredCode & MC_MSRP) != 0)
        {// If there's a Motor StuckReverse feedback (MSRP) that has occurred but has already been cleared
            if(pHandle->aFaultManagementCounters[STUCK_REVERSE_COUNTER][M2] >= pHandle->sParameters.hFaultManagementTimeout)
            {// If the timer has timeout, clear the MSRP fault
                wM2FaultOccurredCode &= ~MC_MSRP;
                pHandle->aFaultManagementCounters[STUCK_REVERSE_COUNTER][M2] = 0;
            }
            else
            {//Increase the counter one more tick
                pHandle->aFaultManagementCounters[STUCK_REVERSE_COUNTER][M2]++;
            }
        }

        if ((wM2FaultOccurredCode & MC_NTC_FREEZE_CONTROLLER) != 0)
        {
            wM2FaultOccurredCode &= ~MC_NTC_FREEZE_CONTROLLER;
        }

        if ((wM2FaultOccurredCode & MC_FOC_DURATION) != 0)
        {
            wM2FaultOccurredCode &= ~MC_FOC_DURATION;
        }
    }

    // Verify if all fault occured have been cleared
    if (!wM1FaultOccurredCode)
    {
        //todo: handle result from MDI_FaultAcknowledged below
        MDI_FaultAcknowledged(pHandle->pMDI, M1);
    }

    if (!wM2FaultOccurredCode)
    {
        MDI_FaultAcknowledged(pHandle->pMDI, M2);
    }

    bool bFaultOccured = wM1FaultOccurredCode | wM2FaultOccurredCode;
    return bFaultOccured;
}

/**
  * @brief  Manage motor faults. Check if faults are still present and send motor fault acknowledge when faults are gone.
  * @param  Powertrain handle
  * @retval Returns true if a motor fault is still active, false if no more fault is present.
  */
void PWRT_MotorWarningManagement(PWRT_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    uint32_t wWarningOccurred = MDI_GetOccuredWarnings(pHandle->pMDI, M1);
    
    if ((wWarningOccurred & MC_PHASE_DISC) != MC_NO_ERROR )
    {
        VC_Errors_RaiseError(MOTOR_PHASE_ERROR, HOLD_UNTIL_CLEARED);
    }
    else
    {
        VC_Errors_ClearError(MOTOR_PHASE_ERROR);
    }
    
    if ((wWarningOccurred & MC_HALL_DISC) != MC_NO_ERROR )
    {
        VC_Errors_RaiseError(MOTOR_HALL_ERROR, HOLD_UNTIL_CLEARED);
    }
    else
    {
        VC_Errors_ClearError(MOTOR_HALL_ERROR);
    }
		
    if ((wWarningOccurred & MC_FOLDBACK_TEMP_MOTOR) != MC_NO_ERROR )
    {
        VC_Errors_RaiseError(MOTOR_FOLDBACK_TEMP, HOLD_UNTIL_CLEARED);
    }
    else
    {
        VC_Errors_ClearError(MOTOR_FOLDBACK_TEMP);
    }
    
    if ((wWarningOccurred & MC_FOLDBACK_TEMP_CONTROLLER) != MC_NO_ERROR )
    {
        VC_Errors_RaiseError(CONTROLLER_FOLDBACK_TEMP, HOLD_UNTIL_CLEARED);
    }
    else
    {
        VC_Errors_ClearError(CONTROLLER_FOLDBACK_TEMP);
    }
    
    if ((wWarningOccurred & MC_NTC_DISC_FREEZE_MOTOR) != MC_NO_ERROR )
    {
        VC_Errors_RaiseError(MOTOR_NTC_DISC_FREEZE, HOLD_UNTIL_CLEARED);
    }
    else
    {
        VC_Errors_ClearError(MOTOR_NTC_DISC_FREEZE);
    }
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

/**
  * @brief  Select Control assistance based on Throttle or PAS
  * @param  Powertrain handle
  * @retval pHandle->pTorqueSelect in int16                                                                                    
  */
int16_t PWRT_CalcSelectedTorque(PWRT_Handle_t * pHandle)
{      
    ASSERT(pHandle != NULL);
    
    /* Disable the throttle output if we need to when PAS level is 0 */
    if(pHandle->sParameters.bPAS0DisableThrottle && PedalAssist_GetAssistLevel(pHandle->pPAS) == 0)
    {
        Throttle_DisableThrottleOutput(pHandle->pThrottle);    
    }
    else
    {
        Throttle_EnableThrottleOutput(pHandle->pThrottle);
    }
    
    /* Throttle and walk mode always have higher priority over PAS but 
       the priority between walk mode and throttle depends on the value 
       of the parameter WalkmodeOverThrottle */     
   
    /* Using PAS or Walk mode 
       Conditions are 
        - PAS Detetect & No throttle 
        - Walk Mode detected & Walk Mode over Throttle detected | No Throttle detected */
        
    if ((PedalAssist_IsPASDetected(pHandle->pPAS) && !Throttle_IsThrottleDetected(pHandle->pThrottle)) || 
        (PedalAssist_IsWalkModeDetected(pHandle->pPAS) && (pHandle->pPAS->sParameters.WalkmodeOverThrottle || !Throttle_IsThrottleDetected(pHandle->pThrottle))))
    {
        /* Torque sensor enabled */
        if ((pHandle->pPAS->bCurrentPasAlgorithm == TorqueSensorUse) && !PedalAssist_IsWalkModeDetected(pHandle->pPAS))
        {
            pHandle->hTorqueSelect = PedalAssist_GetTorqueFromTS(pHandle->pPAS);
        }                
        /* Cadence sensor enabled */
        else 
        {
            pHandle->hTorqueSelect = PedalAssist_GetPASCadenceMotorTorque(pHandle->pPAS);
            PedalAssist_PASUpdateMaxSpeed(pHandle->pPAS); 
        }
    }        
    /* Using throttle */
    else 
    {        
        /* Throttle value convert to torque */        
        pHandle->hTorqueSelect = Throttle_ThrottleToTorque(pHandle->pThrottle);
    }
    
    return pHandle->hTorqueSelect;
}

/**
  * @brief  Get the total amount of current the vehicle is pushing
  * @param  Powertrain handle
  * @retval current in amps uin16_t                                                                                   
  */
uint16_t PWRT_GetTotalMotorsCurrent(PWRT_Handle_t * pHandle)
{  
    ASSERT(pHandle != NULL);    
    uint16_t TotalMotorCurrent = 0;
    uint16_t M1Current = 0;
    uint16_t M2Current = 0; 
    
    // Check if M1 is selected
    if (pHandle->pMS->bMotorSelection == ALL_MOTOR_SELECTED || pHandle->pMS->bMotorSelection == M1_SELECTED)
    {    
        int32_t temp = abs(pHandle->pMDI->pMCI->pFOCVars->Iqdref.q);
        
        // explicit cast to uint16 because abs returns an integer. Should not be an issue because q is an int16
        M1Current = (uint16_t)temp;
        
        M1Current = M1Current/(uint16_t)round((INT16_MAX/MAXCURRENT));  // Convert the Iq reference to an actual current value           
    }
    
    if (pHandle->pMS->bMotorSelection == ALL_MOTOR_SELECTED) // we assume m1 and m2 are the same
    {
        M2Current = M1Current;
    }
    
    // Check if M2 is selected
    if (pHandle->pMS->bMotorSelection == M2_SELECTED)
    {
        M2Current = (uint16_t) abs(pHandle->aTorque[M2]); // Get the current torque reference for M2
         
        M2Current = (M2Current * (uint16_t) pHandle->pMDI->pMCI->MCIConvFactors.Gain_Torque_IQRef);    // Convert it to a IQ current reference
        if (M2Current > INT16_MAX)
        {
            M2Current = INT16_MAX;
        }
        
        M2Current = M2Current/(uint16_t)round((INT16_MAX/MAXCURRENT));  // Convert the Iq refference to an actual current value 
    }
    
    TotalMotorCurrent =  M1Current + M2Current;  // Get the sum of the currents form both motors             
  
    return  TotalMotorCurrent;   
}

/**
  * Get the max safe current we can push                                                                                  
  */
uint16_t PWRT_GetMaxSafeCurrent(PWRT_Handle_t * pHandle)
{  
    ASSERT(pHandle != NULL);    
    uint16_t CurrentInAMPS;
    
    CurrentInAMPS = PWRT_ConvertDigitalCurrentToAMPS(pHandle,(uint16_t) MCInterface_GetMaxCurrent(pHandle->pMDI->pMCI));
    
    return  CurrentInAMPS;
}

/**
  * Get the ongoing max current we can push                                                                                   
  */
uint16_t PWRT_GetOngoingMaxCurrent(PWRT_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    uint16_t CurrentInAMPS;
    
    CurrentInAMPS = PWRT_ConvertDigitalCurrentToAMPS(pHandle,(uint16_t) MCInterface_GetOngoingMaxCurrent(pHandle->pMDI->pMCI));
    
    return  CurrentInAMPS;    
}

/**
  * Set the ongoing max current we want to push                                                                                 
  */
void PWRT_SetOngoingMaxCurrent(PWRT_Handle_t * pHandle, uint16_t aCurrent)
{
    ASSERT(pHandle != NULL);
    uint16_t DigitalConv;
    
    DigitalConv = PWRT_ConvertAMPSToDigitalCurrent(pHandle, aCurrent);   
    
    MCInterface_SetOngoingMaxCurrent(pHandle->pMDI->pMCI,(int16_t) DigitalConv);    
}

/**
  * Convert a digital current to a current in AMPs                                                                                   
  */
uint16_t PWRT_ConvertDigitalCurrentToAMPS(PWRT_Handle_t * pHandle, uint16_t aDigitalCurrent)
{
    ASSERT(pHandle != NULL);
    return  (uint16_t) round(aDigitalCurrent / (65535/(pHandle->pMDI->pMCI->MCIConvFactors.MaxMeasurableCurrent * 2)));
}

/**
  * Convert a current in AMPs to a digital current                                                                                   
  */
uint16_t PWRT_ConvertAMPSToDigitalCurrent(PWRT_Handle_t * pHandle, uint16_t aAMPSCurrent)
{
    ASSERT(pHandle != NULL); 
    return  (uint16_t) round(aAMPSCurrent * (65535/(pHandle->pMDI->pMCI->MCIConvFactors.MaxMeasurableCurrent * 2)));
}

/**
  * Setting a new top speed
  */
void PWRT_SetNewTopSpeed(PWRT_Handle_t * pHandle, uint16_t topSpeed)
{
    ASSERT(pHandle != NULL);
    ASSERT(pHandle->pMDI != NULL); 
    uint16_t NewTopSpeed = 0;
    
    if(topSpeed > pHandle->sParameters.VehicleMaxSpeed) // Safety measure to ensure vehicle max speed is always respected
    {
        NewTopSpeed = pHandle->sParameters.VehicleMaxSpeed;
    }
    else
    {
        NewTopSpeed = topSpeed;
    }        
    
    MDI_SetTorqueSpeedLimit(pHandle->pMDI,NewTopSpeed,pHandle->sParameters.TorqueSpeedLimitGain);
}


/**
 * Return the cruise control state
 */
bool PWRT_GetCruiseControlState(PWRT_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    ASSERT(pHandle->pThrottle != NULL);
    
    return Throttle_GetCruiseControlState(pHandle->pThrottle);    
}    

/**
 * Engage the cruise control feature and keep track of the PAS algorithm
 */
void PWRT_EngageCruiseControl(PWRT_Handle_t * pHandle, uint8_t aSpeed)
{
    ASSERT(pHandle != NULL);
    ASSERT(pHandle->pPAS != NULL);
    ASSERT(pHandle->pThrottle != NULL);
    
    if(pHandle->pThrottle->CruiseControlEnable == false)
    {    
        pHandle->sParameters.PreCruiseControlPAS = PedalAssist_GetPASAlgorithm(pHandle->pPAS);
        PedalAssist_SetPASAlgorithm(pHandle->pPAS,CadenceSensorUse);  // Force cadence while in cruise control
        Throttle_EngageCruiseControl(pHandle->pThrottle,aSpeed);
    }
}

/**
 * Disengage the cruise control feature and restore the PAS algorithm
 */
void PWRT_DisengageCruiseControl(PWRT_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    ASSERT(pHandle->pPAS != NULL);
    ASSERT(pHandle->pThrottle != NULL);
        
    if(pHandle->pThrottle->CruiseControlEnable == true)
    { 
        PedalAssist_SetPASAlgorithm(pHandle->pPAS,pHandle->sParameters.PreCruiseControlPAS); 
        Throttle_DisengageCruiseControl(pHandle->pThrottle);  
    }        
}

/**
 *  Force the disengage the cruise control feature
 *  No matter what the screen tells the controller
 */
void PWRT_ForceDisengageCruiseControl(PWRT_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
     
    if(pHandle->sParameters.CruiseForceDisengage == false)
    {
        pHandle->sParameters.CruiseForceDisengage = true;
        PWRT_DisengageCruiseControl(pHandle); 
    }
}

/**
 *  Get the state of the force disengage flag
 */
bool PWRT_GetForceDisengageState(PWRT_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->sParameters.CruiseForceDisengage;   
} 

/**
 *  Clear the flag when the forced disengage is complete
 */
void PWRT_ClearForceDisengage(PWRT_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    pHandle->sParameters.CruiseForceDisengage = false;
}
