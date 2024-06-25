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
#include "vc_defines.h"
#include "wheel.h"
#include "ramps.h"
#include "vc_constants.h"
#include "odometer.h"

// ============================= Defines ================================ //
#define OVERCURRENT_COUNTER         0
#define SPEEDFEEDBACK_COUNTER       1
#define PHASE_DISC_COUNTER          2

#define MAXCURRENT                  MAX_MEASURABLE_CURRENT /* Used for a generic conversion 
                                                              from current ref to actual amps */
// ============================= Variables ================================ //
                                  
bool isPWMCleared;
static Delay_Handle_t ThrottleDelay; // Delay for Throttle stuck check while initialization
static Delay_Handle_t PTSensorDelay; // Delay for Pedal Torque sensor stuck check while initialization
static Delay_Handle_t brakeDelay;    // Delay for Brake sensor stuck check while initialization
static Delay_Handle_t OdometerDelay; // Delay for the odometer

// ==================== Public function prototypes ======================== //

/**
  * @brief  Module initialization, to be called once before using it
  * @param  Powertrain handle
  * @retval None
  */
void PWRT_Init(PWRT_Handle_t * pHandle,Delay_Handle_t pDelayArray[])
{
    ASSERT(pHandle != NULL);     
    uint8_t motorWSSNbrPerRotation = MDI_GetWheelSpdSensorNbrPerRotation(pHandle->pMDI);
    
    // Initialize Delays for stuck conditions
    ThrottleDelay = pDelayArray[THROTTLE_DELAY];
    PTSensorDelay = pDelayArray[PTS_DELAY];
    brakeDelay    = pDelayArray[BRAKE_DELAY];
    OdometerDelay = pDelayArray[ODOMETER_DELAY];
    
    // Initilaize peripherals
    Wheel_Init();
    Throttle_Init(pHandle->pThrottle, &ThrottleDelay, MDI_GetStartingTorque(pHandle->pMDI));
    BRK_Init(pHandle->pBrake, &brakeDelay);
    BatMonitor_Init(pHandle->pBatMonitorHandle);
    MS_Init(pHandle->pMS);
    PWREN_Init(pHandle->pPWREN);
    Odometer_Init(&OdometerDelay,pHandle->pPAS->pWSS,1000); // Time interval is 1 sec for now
    Light_Init(pHandle->pHeadLight);
    Light_Init(pHandle->pTailLight);
    
    //if we want to use external wss or if wss of motor has 0 magnets use external wss nbr of magnets value
    if (pHandle->pPAS->pWSS->bWSSUseMotorPulsePerRotation == false || motorWSSNbrPerRotation <= 0)
    {
        //use starting torque for dual motors, nominal torque  for all other motors
        #if POWERTRAIN_DEFAULT_MODE == DUAL_MOTOR
            PedalAssist_Init(pHandle->pPAS, &PTSensorDelay, MDI_GetStartingTorque(pHandle->pMDI), EXTERNAL_WSS_NBR_PER_ROTATION);
        #else
            PedalAssist_Init(pHandle->pPAS, &PTSensorDelay, MDI_GetNominalTorque(pHandle->pMDI), EXTERNAL_WSS_NBR_PER_ROTATION, EXTERNAL_WSS_TIME_ON_ONE_MAGNET_PERCENT);    
        #endif
    }
    //if we want to use the motor's wss, use motor nbr of magnets value
    else
    {        
        //use starting torque for dual motors, nominal torque  for all other motors
        #if POWERTRAIN_DEFAULT_MODE == DUAL_MOTOR
            PedalAssist_Init(pHandle->pPAS, &PTSensorDelay, MDI_GetStartingTorque(pHandle->pMDI), motorWSSNbrPerRotation);
        #else
            PedalAssist_Init(pHandle->pPAS, &PTSensorDelay, MDI_GetNominalTorque(pHandle->pMDI), motorWSSNbrPerRotation);    
        #endif
    }

    pHandle->aTorque[M1] = 0; pHandle->aTorque[M2] = 0;
    pHandle->aSpeed[M1] = 0; pHandle->aSpeed[M2] = 0;
    pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M1] = 0; pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M2] = 0;
    pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M1] = 0; pHandle->aFaultManagementCounters[SPEEDFEEDBACK_COUNTER][M2] = 0;
    
    pHandle->sParameters.CruiseForceDisengage = false;
    
    pHandle->sParameters.ScreenMaxSpeed = pHandle->sParameters.VehicleMaxSpeed;
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
    
    //this variable was added to create a linear deceleration ramp
    //when PAS is off(bike is stoping).
    static int16_t lastTorque = 0;
    
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
        // Calculate the pedal assist torque sensor value
        PedalTorqSensor_CalcAvValue(pHandle->pPAS->pPTS, (uint8_t)Wheel_GetVehicleSpeedFromWSS(pHandle->pPAS->pWSS)); 
        
        hTorqueRef = PWRT_CalcSelectedTorque(pHandle); // Compute torque to motor depending on either throttle or PAS
        hAux = hTorqueRef; //hAux is used as auxialiary variable for final torque computation. Will be reduced depending on brake state.
                
        if (bIsBrakePressed)
        {
            hAux = 0;
            
            //Reset passed detection
            PedalAssist_ResetPASDetected(pHandle->pPAS);
            PedalAssist_ResetCadenceStartupPasDection(pHandle->pPAS);
            PedalAssist_ResetCadenceRunningPasDection(pHandle->pPAS);
            PedalAssist_ResetTorqueStartupPasDection(pHandle->pPAS);
            PedalAssist_ResetTorqueRunningPasDection(pHandle->pPAS);
            PedalAssist_ResetCadenceStatePasDection();
            
            // Reset All the Pedal Assist Parameters
            PedalAssist_ResetParameters(pHandle->pPAS);
            PWRT_ForceDisengageCruiseControl(pHandle);
        }
        
        // If the user pedals while were are in cruise
        if((PedalAssist_IsPASDetected(pHandle->pPAS) == true) && (PWRT_GetCruiseControlState(pHandle) == true))
        {
            hAux = 0;                                  // Exit cruise control
            PedalAssist_ResetPASDetected(pHandle->pPAS);
            PedalAssist_ResetCadenceStartupPasDection(pHandle->pPAS);
            PedalAssist_ResetCadenceRunningPasDection(pHandle->pPAS);
            PedalAssist_ResetTorqueStartupPasDection(pHandle->pPAS);
            PedalAssist_ResetTorqueRunningPasDection(pHandle->pPAS);
            PedalAssist_ResetCadenceStatePasDection();
            PedalAssist_ResetParameters(pHandle->pPAS);
            PWRT_ForceDisengageCruiseControl(pHandle);
        }
        
        /* Throttle and walk mode always have higher priority over PAS but 
           the priority between walk mode and throttle depends on the value 
           of the parameter WalkmodeOverThrottle  */    
                 
            /* Using PAS or walk mode */
        
        bool ThrottleDetected = Throttle_IsThrottleDetected(pHandle->pThrottle);
        bool PASDetected      = PedalAssist_IsPASDetected(pHandle->pPAS);
        bool WalkDetected     = PedalAssist_IsWalkModeDetected(pHandle->pPAS);
        bool WalkOverThrottle = pHandle->pPAS->sParameters.WalkmodeOverThrottle;
        bool PASOverThrottle  = pHandle->pPAS->sParameters.PASOverThrottle;
        
        if (((PASDetected || (hAux < lastTorque)) && (!ThrottleDetected || PASOverThrottle)) || 
             (WalkDetected && (!ThrottleDetected || WalkOverThrottle)))
        {             
            //get the last torque value.
            lastTorque = hAux;
            
            if (pHandle->sParameters.bEnableSpeedLimit)
            {
                 // Make sure we have the most up-to-date desired top speed
                uint16_t TopSpeed = PedalAssist_PASUpdateMaxSpeed(pHandle->pPAS);
                
                PWRT_SetNewTopSpeed(pHandle,TopSpeed);        // Tell motor control what is our desired top speed       
            }

            #if VEHICLE_SELECTION == VEHICLE_NIDEC || VEHICLE_SELECTION == VEHICLE_PEGATRON
            if (!PedalAssist_IsWalkModeDetected(pHandle->pPAS))
            {
                static int16_t PowerAvg = 0;                
                                   
                int16_t BandwidthUp = 35; //Bandwidth CANNOT be set to 0
                int16_t BandwidthDown = 75;    
                int16_t Bandwidth = 0;
                   
                if(abs(PowerAvg - hAux * 100) > ((pHandle->pPAS->sParameters.hPASMaxTorque * 100)/25)) // Sudden acceleration or decelration ? 
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
                
                PowerAvg = ((Bandwidth-1) * PowerAvg + (hAux * 100))/Bandwidth;
                                        
                hAux = PowerAvg/100;
            }
            #endif
        }                            
        else if(Throttle_IsThrottleDetected(pHandle->pThrottle) && pHandle->sParameters.bEnableSpeedLimit)
        {
            uint16_t TopSpeed = 0;
                          
            TopSpeed = Throttle_GetMaxSpeed(pHandle->pThrottle);   // Get the current desired top speed for throttle
            PWRT_SetNewTopSpeed(pHandle,TopSpeed);                 // Tell motor control what is our desired top speed 
        }   
        
        // Store powertrain target torque value in handle
        pHandle->aTorque[pHandle->bMainMotor] = hAux;
        
        pHandle->hOldTorqueSelect = hAux;
        
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
            if (MDI_GetMotorType(pHandle->pMDI) != DIRECT_DRIVE)
            {
                bCheckStop1 = true;
            }
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
    bool bCheckStart4 = false;

    uint16_t hThrottleValue = Throttle_GetAvThrottleValue(pHandle->pThrottle);
    uint16_t wheelSpeed = Wheel_GetSpeedFromWheelRpm(WheelSpdSensor_GetSpeedRPM(pHandle->pPAS->pWSS));
    
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
    if(wheelSpeed < pHandle->sParameters.VehicleMaxSpeed || !(pHandle->sParameters.bEnableSpeedLimit)) //Check if the wheel speed does not exceeds the speed limit
    {
        bCheckStart4 = true;
    }
    return bCheckStart1 & bCheckStart2 & bCheckStart3 & bCheckStart4; // Final logic to know if powertrain should be started.
}

/**
  * @brief  Manage motor critical faults. Check if critical faults are still present and send motor critical fault acknowledge when critical faults are gone.
  * @param  Powertrain handle
  * @retval Returns true if a motor critical fault is still active, false if no more critical fault is present.
  */

bool PWRT_MotorCriticalFaultManagement(PWRT_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    uint32_t wM1FaultOccurredCode = MDI_GetOccurredCriticalFaults(pHandle->pMDI, M1);
    uint32_t wM2FaultOccurredCode = MDI_GetOccurredCriticalFaults(pHandle->pMDI, M2);

    uint32_t wCriticalFaultOccurred = wM1FaultOccurredCode | wM2FaultOccurredCode;

    if (wCriticalFaultOccurred != MC_NO_FAULT)      // Raise Motor current error to the LCD
    {
        if ((wCriticalFaultOccurred & MC_OCD1)!= MC_NO_FAULT)
        {
            VC_Errors_RaiseError(OVER_CURRENT, HOLD_UNTIL_CLEARED);
        }
        if ((wCriticalFaultOccurred & MC_PHASE_DISC) != MC_NO_WARNING)
        {
            VC_Errors_RaiseError(MOTOR_PHASE_ERROR, DEFAULT_HOLD_FRAMES);
        }
        if ((wCriticalFaultOccurred & MC_OVER_VOLT)!= MC_NO_FAULT)
        {
            VC_Errors_RaiseError(OV_PROTECTION, DEFAULT_HOLD_FRAMES);
        }

                
    }
    if (PWRT_IsMotor1Used(pHandle))
    {// If there's an over current (OC) that has occurred but has already been cleared
        #if OCDX_POEG == OCD2_POEG
            if (wM1FaultOccurredCode & MC_OCD2)
            {
                if (pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M1] >= pHandle->sParameters.hFaultManagementTimeout)
                {// If the timer has timeout, clear the OC fault
                    wM1FaultOccurredCode &= ~MC_OCD2;
                    pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M1] = 0;
                    if ((wCriticalFaultOccurred & MC_OCD1) == 0)
                    {
                        VC_Errors_ClearError(OVER_CURRENT);
                    }
                }
                else
                {//Increase the counter one more tick
                    pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M1]++;
                }
            }
        #endif

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

        if ((wM1FaultOccurredCode & MC_OVER_VOLT) != 0)
        {
            // In case of DCbus overvoltage, clear the OV fault
            wM1FaultOccurredCode &= ~MC_OVER_VOLT;
        }

        if ((wM1FaultOccurredCode & MC_FOC_DURATION) != 0)
        {
            wM1FaultOccurredCode &= ~MC_FOC_DURATION;
        }
        if ((wM1FaultOccurredCode & MC_PHASE_DISC) != 0)
        {
            //if there's a phase disconection that occurred but has already been cleared
            if(pHandle->aFaultManagementCounters[PHASE_DISC_COUNTER][M1] >= pHandle->sParameters.hFaultManagementTimeout)
            {
                //if the phase disconnection timer has cleared out, clear the phase disconnection fault
                wM1FaultOccurredCode &= ~MC_PHASE_DISC;
                pHandle->aFaultManagementCounters[PHASE_DISC_COUNTER][M1] = 0;
            }
            else
            {
                //increase the counter one more tick
                pHandle->aFaultManagementCounters[PHASE_DISC_COUNTER][M1]++;
            }
        }
    }

    if (PWRT_IsMotor2Used(pHandle))
    {
        #if OCDX_POEG == OCD2_POEG
            if ((wM2FaultOccurredCode & MC_OCD2))
            {
                if(pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M2] >= pHandle->sParameters.hFaultManagementTimeout)
                {// If the timer has timeout, clear the OC fault
                    wM2FaultOccurredCode &= ~MC_OCD2;
                    pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M2] = 0;
                }
                else
                {//Increase the counter one more tick
                    pHandle->aFaultManagementCounters[OVERCURRENT_COUNTER][M2]++;
                }
            }
        #endif
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

        if (wM2FaultOccurredCode & MC_OVER_VOLT)
        {
            /* In case of DCbus overvoltage... */
            wM2FaultOccurredCode &= ~MC_OVER_VOLT;
        }

        if ((wM2FaultOccurredCode & MC_FOC_DURATION) != 0)
        {
            wM2FaultOccurredCode &= ~MC_FOC_DURATION;
        }
        if ((wM2FaultOccurredCode & MC_PHASE_DISC) != 0)
        {
            //if there's a phase disconnection that has occurred but has already been cleared
            if(pHandle->aFaultManagementCounters[PHASE_DISC_COUNTER][M2] >= pHandle->sParameters.hFaultManagementTimeout)
            {
                //if the phase disconnection timer has cleared out, clear the phase disconnection fault
                wM2FaultOccurredCode &= ~MC_PHASE_DISC;
                pHandle->aFaultManagementCounters[PHASE_DISC_COUNTER][M2] = 0;
            }
            else
            {
                //Increase the counter one more tick
                pHandle->aFaultManagementCounters[PHASE_DISC_COUNTER][M2]++;
            }
        }
    }

    // Verify if all fault occurred have been cleared
    if (!wM1FaultOccurredCode)
    {
        MDI_CriticalFaultAcknowledged(pHandle->pMDI, M1);
    }

    if (!wM2FaultOccurredCode)
    {
        MDI_CriticalFaultAcknowledged(pHandle->pMDI, M2);
    }
    

    bool bFaultOccurred = wM1FaultOccurredCode | wM2FaultOccurredCode;
    return bFaultOccurred;
}

/**
  * @brief  Manage motor errors. Check if errors are still present and send motor errors acknowledge when errors are gone.
  * @param  Powertrain handle
  * @retval Returns true if a motor error is still active, false if no more error is present.
  */
void PWRT_MotorErrorManagement(PWRT_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    uint32_t wM1ErrorOccurred = MDI_GetOccurredErrors(pHandle->pMDI, M1);
    uint32_t wM2ErrorOccurred = MDI_GetOccurredErrors(pHandle->pMDI, M2);
    
    uint32_t wErrorOccurred = wM1ErrorOccurred | wM2ErrorOccurred;
    
    //if OCSP error occurs, raise to vc layer
    if ((wErrorOccurred & MC_OCSP) != MC_NO_WARNING)
    {
        VC_Errors_RaiseError(OVERCURRENT_COUNTER, HOLD_UNTIL_CLEARED);
    }
    //else clear the error
    else
    {
        VC_Errors_ClearError(OVERCURRENT_COUNTER);
    }
    
    #if OCDX_POEG == OCD1_POEG && HARDWARE_OCD2 == OCD2_ENABLED
        //if OCD2 error occurs, raise to vc layer
        if ((wErrorOccurred & MC_OCD2) != MC_NO_WARNING)
        {
            VC_Errors_RaiseError(OVERCURRENT_COUNTER, HOLD_UNTIL_CLEARED);
        }
        //else clear the error
        else
        {
            VC_Errors_ClearError(OVERCURRENT_COUNTER);
        }
    #endif
    
    //if hall sensor error occurs, raise to vc layer
    if ((wErrorOccurred & MC_HALL_DISC) != MC_NO_WARNING)
    {
        VC_Errors_RaiseError(MOTOR_HALL_ERROR, HOLD_UNTIL_CLEARED);
    }
    //else clear the error
    else
    {
        VC_Errors_ClearError(MOTOR_HALL_ERROR);
    }
    
    //if over temp controller error occurs, raise to vc layer
    if ((wErrorOccurred & MC_OVER_TEMP_CONTROLLER)!= MC_NO_FAULT)
    {
        VC_Errors_RaiseError(CONTROLLER_OT_PROTECT, DEFAULT_HOLD_FRAMES);
    }
    //else clear the error
    else
    {
        VC_Errors_ClearError(CONTROLLER_OT_PROTECT);
    }
    
    //if under temp controller error occurs, raise to vc layer
    if ((wErrorOccurred & MC_NTC_FREEZE_CONTROLLER)!= MC_NO_FAULT)
    {
        VC_Errors_RaiseError(UT_PROTECTION, DEFAULT_HOLD_FRAMES);
    }
    //else clear the error
    else
    {
        VC_Errors_ClearError(UT_PROTECTION);
    }
    
    //if over temp motor error occurs, raise to vc layer
    if ((wErrorOccurred & MC_OVER_TEMP_MOTOR) != MC_NO_FAULT)
    {
        VC_Errors_RaiseError(MOTOR_OT_PROTECT, DEFAULT_HOLD_FRAMES);
    }
    //else clear the error
    else
    {
        VC_Errors_ClearError(MOTOR_OT_PROTECT);
    }
    
    //if undervoltage error occurs, raise to vc layer
    if ((wErrorOccurred & MC_UNDER_VOLT) != MC_NO_FAULT)
    {
        VC_Errors_RaiseError(UV_PROTECTION, DEFAULT_HOLD_FRAMES);
    }
    //else clear the error
    else
    {
        VC_Errors_ClearError(UV_PROTECTION);
    }
}

/**
  * @brief  Manage motor warnings. Check if faults are still present and send motor warnings acknowledge when warnings are gone.
  * @param  Powertrain handle
  * @retval Returns true if a motor warning is still active, false if no more warning is present.
  */
void PWRT_MotorWarningManagement(PWRT_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    uint32_t wM1WarningOccurred = MDI_GetOccurredWarnings(pHandle->pMDI, M1);
    uint32_t wM2WarningOccurred = MDI_GetOccurredWarnings(pHandle->pMDI, M2);
        
    uint32_t wWarningOccurred = wM1WarningOccurred | wM2WarningOccurred;
    
    //if motor temperature foldback warning occurs, raise to vc layer
    if ((wWarningOccurred & MC_FOLDBACK_TEMP_MOTOR) != MC_NO_WARNING)
    {
        VC_Errors_RaiseError(MOTOR_FOLDBACK_TEMP, HOLD_UNTIL_CLEARED);
    }
    //else clear the warning
    else
    {
        VC_Errors_ClearError(MOTOR_FOLDBACK_TEMP);
    }
    
    //if controller temperature foldback warning occurs, raise to vc layer
    if ((wWarningOccurred & MC_FOLDBACK_TEMP_CONTROLLER) != MC_NO_WARNING)
    {
        VC_Errors_RaiseError(CONTROLLER_FOLDBACK_TEMP, HOLD_UNTIL_CLEARED);
    }
    //else clear the warning
    else
    {
        VC_Errors_ClearError(CONTROLLER_FOLDBACK_TEMP);
    }
    
    //if motor disconnection or freeze warning occurs, raise to vc layer
    if ((wWarningOccurred & MC_NTC_DISC_FREEZE_MOTOR) != MC_NO_WARNING)
    {
        VC_Errors_RaiseError(MOTOR_NTC_DISC_FREEZE, HOLD_UNTIL_CLEARED);
    }
    //else clear the warning
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
    
    static bool PASWasDetected = false;
    
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
    bool ThrottleDetected = Throttle_IsThrottleDetected(pHandle->pThrottle);
    bool PASDetected      = PedalAssist_IsPASDetected(pHandle->pPAS);
    bool WalkDetected     = PedalAssist_IsWalkModeDetected(pHandle->pPAS);
    bool PowerEnable      = PedalAssist_IsPowerEnableDetected(pHandle->pPAS);
    bool WalkOverThrottle = pHandle->pPAS->sParameters.WalkmodeOverThrottle;
    bool PASOverThrottle  = pHandle->pPAS->sParameters.PASOverThrottle;
    
    if (((PASDetected || PowerEnable)  && (!ThrottleDetected || PASOverThrottle)) ||
        (WalkDetected && (!ThrottleDetected || WalkOverThrottle)))
    {
        PASWasDetected = true;
               
        /* Torque sensor enabled */
        if (!PedalAssist_IsWalkModeDetected(pHandle->pPAS))
        {
            pHandle->hTorqueSelect = PedalAssist_GetTorqueFromTS(pHandle->pPAS);
        }                
        else
        {
            pHandle->hTorqueSelect = PedalAssist_GetWalkmodeTorque(pHandle->pPAS);
        }
        
        // Link the correct PAS ramp as the ramp to apply
        //pSelectedRampHandle = PedalAssist_GetRamp(pHandle->pPAS, Direction);

        if(PedalAssist_IsCadenceDetected(pHandle->pPAS))
        {
            pHandle->hTorqueSelect = PWRT_EnableCadencePower(pHandle);
        }

        pHandle->hTorqueSelect = Ramps_ApplyRamp(PAS_RAMP_SELECTION, Wheel_GetVehicleSpeedFloatFromWSS(pHandle->pPAS->pWSS), pHandle->hTorqueSelect); 

    }
    /* Using throttle */
    else 
    {        
        /* Throttle value convert to torque */        
        pHandle->hTorqueSelect = Throttle_ThrottleToTorque(pHandle->pThrottle);    
    }

    // Check if there is any torque sensor issue detected
    pHandle->pPAS->bTorqueSensorIssue = PedalAssist_TorqueSensorIssueDetected(pHandle->pPAS);
    // If we have a torque sensor issue detected and we are supposed to give torque
    if(pHandle->pPAS->bTorqueSensorIssue && pHandle->hTorqueSelect != 0)
    {
        // Cut power because we have detected a torque sensor issue
        pHandle->hTorqueSelect = 0;
        VC_Errors_RaiseError(TORQUE_SENSOR_ERROR, HOLD_UNTIL_CLEARED);
    }
    // No issues detected, we clear the error
    else
    {
        VC_Errors_ClearError(TORQUE_SENSOR_ERROR);
    }
    return pHandle->hTorqueSelect;
}

/**
  * @brief  Get minimum power required on cadence power enable
  * @param  Powertrain handle
  * @retval Torque power value calculated according to min power                                                                                     
  */
int16_t PWRT_EnableCadencePower(PWRT_Handle_t *pHandle)
{
    int16_t maxTorquePower = pHandle->pPAS->sParameters.hPASMaxTorque;
    uint8_t currentPASLevel = pHandle->pPAS->bCurrentAssistLevel;
    float minPowerPercentage = pHandle->pPAS->sParameters.PASMinTorqRatiosInPercentage[currentPASLevel];
    int16_t tempPower = (int16_t)((float)maxTorquePower * (minPowerPercentage / 100));
    
    if(tempPower > pHandle->hTorqueSelect) return tempPower;
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
    uint16_t TotalMotorPower = PWRT_GetTotalMotorsPower(pHandle);
    
    float TotalMotorCurrent;
    
    if (pHandle->pBatMonitorHandle->VBatAvg > 0)
    {
        TotalMotorCurrent = (TotalMotorPower * 100)/ pHandle->pBatMonitorHandle->VBatAvg;
    }
    else
    {
        TotalMotorCurrent = 0;
    }
    return (uint16_t) round(TotalMotorCurrent);
}

/**
  * @brief  Get the total amount of power the vehicle is pushing
  * @param  Powertrain handle
  * @retval power in watts uin16_t                                                                                   
  */
uint16_t PWRT_GetTotalMotorsPower(PWRT_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL); 
    
    float TotalMotorPower = 0;
    uint16_t M1Rpm = (uint16_t) abs(MDI_GetAvrgMecSpeedUnit(pHandle->pMDI, M1));
    
    float M1TorqueRef = MDI_GetMotorTorqueReference(pHandle->pMDI, M1);
    
    TotalMotorPower  = M1Rpm * RPM_TO_RAD_PERSEC  * (M1TorqueRef/100);
    
    //if using dual motors add dual motor's power
    if(pHandle->sParameters.bMode == DUAL_MOTOR)
    {
        uint16_t M2Rpm = (uint16_t) abs(MDI_GetAvrgMecSpeedUnit(pHandle->pMDI, M2));
        float M2TorqueRef = MDI_GetMotorTorqueReference(pHandle->pMDI, M2);
        TotalMotorPower += M2Rpm * RPM_TO_RAD_PERSEC  * (M2TorqueRef/100);
    }
    
    return  (uint16_t)round(TotalMotorPower);   

}

/**
  * @brief  Get the approximate DC power (motor power + losses)
  * @param  Powertrain handle
  * @retval power in watts uin16_t                                                                                   
  */
uint16_t PWRT_GetDCPower(PWRT_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
    float Amps = 0;
    float Loss = 0;
    uint16_t IqRef = 0;
    qd_t IqdrefM1 = MDI_GetIqdref(pHandle->pMDI, M1);
        
    //if using dual add the dual motor's current
    if(pHandle->sParameters.bMode == DUAL_MOTOR)
    {
        qd_t IqdrefM2 = MDI_GetIqdref(pHandle->pMDI, M2);
        IqRef = (uint16_t) (abs(IqdrefM1.q) + abs (IqdrefM2.q));
    }
    else
    {
        // Get Iqref
        IqRef = (uint16_t) abs(IqdrefM1.q);
    }
    
    // Convert to amps using amps = iqref *(2 * MAX_MEASURABLE_CURRENT)/65535;
    Amps = (float)(IqRef *(2 * MAX_MEASURABLE_CURRENT)/65535);
    
    // Aprox motor loss with 3*Rs*amps^2;
    Loss = 3 * MDI_GetRS(pHandle->pMDI) * Amps * Amps;
    
    
    
    // Total power is mech power + loss    
    return (uint16_t) round(PWRT_GetTotalMotorsPower(pHandle) + Loss);
}

/**
  * @brief  Get the max DC power (motor power + losses)
  * @param  Powertrain handle
  * @retval power in watts uin16_t                                                                                   
  */
uint16_t PWRT_GetMaxDCPower(PWRT_Handle_t * pHandle)
{
  ASSERT(pHandle != NULL); 
  return MDI_GetMaxPositivePower(pHandle->pMDI);  
}

/**
  * @brief  Get the approximate DC current (motor current + losses)
  * @param  Powertrain handle
  * @retval power in watts uin16_t                                                                                   
  */
uint16_t PWRT_GetDCCurrent(PWRT_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);    
    uint16_t DCPower = PWRT_GetDCPower(pHandle);
    
    float DCCurrent;
    
    if (pHandle->pBatMonitorHandle->VBatAvg > 0)
    {
        DCCurrent = (DCPower * 100) / pHandle->pBatMonitorHandle->VBatAvg;
    }
    else
    {
        DCCurrent = 0;
    }
 
    return (uint16_t) round(DCCurrent);
}

/**
  * @brief  Get the total amount of torque the motors are pushing
  * @param  Powertrain handle
  * @retval torque in nm uin16_t                                                                                   
  */
uint16_t PWRT_GetTotalMotorsTorque(PWRT_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL); 
    
    float TotalMotorTorque = 0;
    
    uint16_t M1TorqueRef = MDI_GetMotorTorqueReference(pHandle->pMDI, M1);
    //uint16_t M2TorqueRef = MDI_GetMotorTorqueReference(pHandle->pMDI, M2);
    
    TotalMotorTorque  = M1TorqueRef;
                
   // TotalMotorTorque += M2TorqueRef/100; // For now not supporting dual
    
    return  (uint16_t)round(TotalMotorTorque);   

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
    uint16_t SmallestSpeedLimit = 0;
    
    // Check which of the speed limits is the most restrictive
    if (pHandle->sParameters.ScreenMaxSpeed > pHandle->sParameters.VehicleMaxSpeed)
    {
        SmallestSpeedLimit = pHandle->sParameters.VehicleMaxSpeed;
    }        
    else
    {
        SmallestSpeedLimit = pHandle->sParameters.ScreenMaxSpeed;
    }
    
        
    // Apply the speed limit restriction
    if (topSpeed > SmallestSpeedLimit) 
    {
        NewTopSpeed = SmallestSpeedLimit;
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
        pHandle->sParameters.PreCruiseControlStartupPASAlgo = PedalAssist_GetStartupPasAlgorithm(pHandle->pPAS);
        pHandle->sParameters.PreCruiseControlRunningPASAlgo = PedalAssist_GetRunningPasAlgorithm(pHandle->pPAS);
        PedalAssist_SetStartupPASAlgorithm(pHandle->pPAS,CadenceSensorUse);  // Force cadence while in cruise control
        PedalAssist_SetRunningPASAlgorithm(pHandle->pPAS,CadenceSensorUse);  // Force cadence while in cruise control
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
        PedalAssist_SetStartupPASAlgorithm(pHandle->pPAS,pHandle->sParameters.PreCruiseControlStartupPASAlgo); 
        PedalAssist_SetRunningPASAlgorithm(pHandle->pPAS,pHandle->sParameters.PreCruiseControlRunningPASAlgo); 
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

/**
 *  Updates wheel RPM to MC Layer
 */
void PWRT_SetWheelRPM(PWRT_Handle_t * pHandle)
{ 
    ASSERT(pHandle != NULL);
    uint16_t wheelRPM = WheelSpdSensor_GetSpeedRPM(pHandle->pPAS->pWSS);
    
    MDI_SetWheelRPM(pHandle->pMDI, wheelRPM);
}

/**
 *  Updates the top speed of the screen
 */
void PWRT_SetScreenMaxSpeed(PWRT_Handle_t * pHandle, uint8_t aSpeed)
{ 
    ASSERT(pHandle != NULL);
    pHandle->sParameters.ScreenMaxSpeed = aSpeed;
}

/**
 *  Get the bus voltage
 */
uint16_t PWRT_GetBusVoltagex100(PWRT_Handle_t * pHandle)
{
    return MDI_GetBusVoltageInVoltx100(pHandle->pMDI->pMCI);
}

/**
 *  Get the traveled distance
 */
uint32_t PWRT_GetDistanceTravelled()
{
    return Odometer_GetDistanceTravelled();
}
