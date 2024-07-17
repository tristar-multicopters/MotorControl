/**
    * @file      speed_torq_ctrl.c
    * @author    Sami Bouzid, FTEX inc
    * @brief     This file provides firmware functions that implement the following features
    *            of the Speed & Torque Control component of the Motor Control application.
    *
*/

/* Includes ------------------------------------------------------------------*/
#include "speed_torq_ctrl.h"
#include "speed_pos_fdbk.h"
#include "mc_type.h"
#include "gear_ratio_table.h"

#define GEAR_RATIO_LOWER_BOUND_MIN 1        //min gear ratio, cannot be 0
#define GEAR_RATIO_UPPER_BOUND_MAX 10       //max gear ratio = highest gear + GEAR_RATIO_UPPER_BOUND_MAX

#define SPEED_MARGIN  110
#define DIV_PERCENTAGE 100
#define DECC_RANGE_PW  100 //decceleration range
#define DCC_RANGE 12  //decceleration range for walk mode
static int16_t SpdTorqCtrl_ApplyTorqueFoldback(SpdTorqCtrlHandle_t * pHandle, int16_t hInputTorque, MotorParameters_t MotorParameters);
static int16_t SpdTorqCtrl_ApplyPowerLimitation(SpdTorqCtrlHandle_t * pHandle, int16_t hInputTorque);
void SpdTorqCtrl_SetGearRatio(SpdTorqCtrlHandle_t * pHandle, int16_t hMeasuredSpeed);

uint16_t M_STUCK_timer = 0;
uint16_t OCD_timer = 0;

/*
 * see function definition
 */
void SpdTorqCtrl_Init(SpdTorqCtrlHandle_t * pHandle, PIDHandle_t * pPI, SpdPosFdbkHandle_t * SPD_Handle,
                        MotorParameters_t MotorParameters)
{
    ASSERT(pHandle != NULL);
    
    //Init the foldbacks in speed torque ctrl
    Foldback_Init(&pHandle->FoldbackDynamicMaxTorque, MotorParameters.ParametersConversion.FoldbackInitTorque);
    Foldback_Init(&pHandle->FoldbackControllerTemperature, MotorParameters.ParametersConversion.FoldbackInitHeatsinkTemp);
    Foldback_Init(&pHandle->FoldbackMotorSpeed, MotorParameters.ParametersConversion.FoldbackInitSpeed);
    Foldback_Init(&pHandle->FoldbackMotorTemperature, MotorParameters.ParametersConversion.FoldbackInitMotorTemp);
    
    //Init the variables dependent on motor parameters
    pHandle->fGearRatio     = MotorParameters.ConfigParameters.fMotorGearRatio;
    pHandle->motorType      = MotorParameters.ConfigParameters.bMotorType;
    
    pHandle->hMaxAppPositiveMecSpeedUnit = MotorParameters.ParametersConversion.hMaxApplicationSpeedUnit;
    pHandle->hMaxAppNegativeMecSpeedUnit = (int16_t)(- MotorParameters.ParametersConversion.hMaxApplicationSpeedUnit);
    pHandle->hSpdLimit = (int16_t)(MotorParameters.ParametersConversion.hMaxApplicationSpeedUnit);
    pHandle->hSpdLimitWheelRpm = MotorParameters.ParametersConversion.hMaxApplicationSpeedUnit;
        
    pHandle->hMaxPositivePower = (uint16_t)(DEFAULT_MAX_APPLICATION_POSITIVE_POWER * MotorParameters.PowerParameters.hEstimatedEfficiency / 100);
    pHandle->hMinNegativePower = -(int16_t)(DEFAULT_MAX_APPLICATION_POSITIVE_POWER * MotorParameters.PowerParameters.hEstimatedEfficiency / 100);
    pHandle->hEstimatedEfficiencyPercent = MotorParameters.PowerParameters.hEstimatedEfficiency;
    
    pHandle->wTorqueSlopePerSecondUp = MotorParameters.RampManagerParameters.wDefaultTorqueSlopeUp;
    pHandle->wTorqueSlopePerSecondDown = MotorParameters.RampManagerParameters.wDefaultTorqueSlopeDown;
    pHandle->wSpeedSlopePerSecondUp = MotorParameters.RampManagerParameters.wDefaultSpeedSlopeUp;
    pHandle->wSpeedSlopePerSecondDown = MotorParameters.RampManagerParameters.wDefaultSpeedSlopeDown;
    
    pHandle->fGainTorqueIqref = MotorParameters.ParametersConversion.fGainTorqueIqRef;
    
    pHandle->hMaxPositiveTorque = MotorParameters.ParametersConversion.hNominalTorque;
    pHandle->hMinNegativeTorque = (int16_t)(-MotorParameters.ParametersConversion.hNominalTorque);
    
    pHandle->hStartingTorque = MotorParameters.ParametersConversion.hStartingTorque;
    
    pHandle->bFluxWeakeningEn = MotorParameters.FluxParameters.bFluxWeakeningEnable;
    
    pHandle->pPISpeed = pPI;
    pHandle->pSPD = SPD_Handle;
    pHandle->pSPD->gearMAFiltPos = 0;
    pHandle->pSPD->hTorqueRegenMax = 0;
    pHandle->pSPD->hDeltaT = REGEN_TORQUE_RAMP;
    pHandle->pSPD->hTorqueRegen = 0;
    pHandle->pSPD->hIdcRegen = MAX_NEG_DC_CURRENT;
    pHandle->pSPD->bActiveRegen = 0;
    for(uint16_t count = 0; count < GEAR_FILTER_SIZE; count++)
    {
        pHandle->pSPD->gearArray[count] = 0;
    }

    pHandle->Mode = pHandle->ModeDefault;
  
    if (NTCTempSensor_GetSensorType(NTC_CONTROLLER) == NO_SENSOR)
    {
        Foldback_DisableFoldback(&pHandle->FoldbackControllerTemperature);
    }
    if (NTCTempSensor_GetSensorType(NTC_MOTOR) == NO_SENSOR)
    {
        Foldback_DisableFoldback(&pHandle->FoldbackMotorTemperature);
    }
    
    //the foldback is used to control speed for direct drives instead of the pid
    if (pHandle->motorType == DIRECT_DRIVE)
    {
        Foldback_EnableFoldback(&pHandle->FoldbackLimitSpeed);
    }
   
    pHandle->TorqueRampMngr.wFrequencyHz = pHandle->hSTCFrequencyHz;
    pHandle->SpeedRampMngr.wFrequencyHz = pHandle->hSTCFrequencyHz;
    RampMngr_Init(&pHandle->TorqueRampMngr);
    RampMngr_Init(&pHandle->SpeedRampMngr);
    DynamicPower_Init(&pHandle->DynamicPowerHandle, (uint16_t)(DEFAULT_MAX_APPLICATION_POSITIVE_POWER), MotorParameters.PowerParameters.hEstimatedEfficiency);
    StuckProtection_Init(&pHandle->StuckProtection);
    
    //Init speed limit PID
    PID_Init(&pHandle->PISpeedLimit, MotorParameters.ParametersConversion.PIDInitSpeedLimit);
}

/*
 * see function definition
 */
void SpdTorqCtrl_PowerInit(SpdTorqCtrlHandle_t * pHandle, MC_Setup_t MCSetup, MotorParameters_t MotorParameters)
{
    ASSERT(pHandle != NULL);
    
    Foldback_Handle_t FoldbackInitPower;
    
    //initialize maximum power in watts that drive can push to the motor
    if (MCSetup.BatteryPowerSetup.hMaxApplicationPositivePower < DEFAULT_MAX_APPLICATION_POSITIVE_POWER)
    {
        pHandle->FoldbackDynamicMaxPower.hDefaultOutputLimitHigh = MCSetup.BatteryPowerSetup.hMaxApplicationPositivePower * MotorParameters.PowerParameters.hEstimatedEfficiency / 100;
        pHandle->DynamicPowerHandle.hDynamicMaxPower = MCSetup.BatteryPowerSetup.hMaxApplicationPositivePower * MotorParameters.PowerParameters.hEstimatedEfficiency / 100;
        pHandle->hMaxPositivePower = (uint16_t)(MCSetup.BatteryPowerSetup.hMaxApplicationPositivePower * MotorParameters.PowerParameters.hEstimatedEfficiency / 100);
    }
    
    //initialize maximum power in watts that drive can accept from the motor
    if (MCSetup.BatteryPowerSetup.hMaxApplicationNegativePower < DEFAULT_MAX_APPLICATION_NEGATIVE_POWER)
    {
        pHandle->hMinNegativePower = -(int16_t)MCSetup.BatteryPowerSetup.hMaxApplicationNegativePower * MotorParameters.PowerParameters.hEstimatedEfficiency / 100;
    }
    
    // Initialize maximum battery current in amps that drive can accept from the motor
    if (MCSetup.BatteryPowerSetup.hMaxApplicationCurrent < DEFAULT_MAX_APPLICATION_CURRENT)
    {
        pHandle->hMaxBusCurrent = MCSetup.BatteryPowerSetup.hMaxApplicationCurrent;
    }
    
    // Initializes the undervoltage threshold of the battery
    if (MCSetup.BatteryPowerSetup.hUndervoltageThreshold > UD_VOLTAGE_THRESHOLD_CONT_V)
    {
        pHandle->hBatteryLowVoltage = (uint16_t)(MCSetup.BatteryPowerSetup.hUndervoltageThreshold *(MCSetup.BatteryPowerSetup.hLowVoltageThresholdPercentage + 100))/100;
    }
    
    // Defines if the code should use MAX_APPLICATION_POSITIVE_POWER or MAX_APPLICATION_CURRENT
    pHandle->bPowerRef = MCSetup.BatteryPowerSetup.bPowerLimitRef;
    
    pHandle->bEnableLVtorqueLimit = MCSetup.BatteryPowerSetup.bEnableLVTorqueLimit;
    pHandle->hLowBatteryTorque = MCSetup.BatteryPowerSetup.hLowBatteryTorque;
    pHandle->hBatteryLowVoltage = (uint16_t)(UD_VOLTAGE_THRESHOLD_CONT_V * (MCSetup.BatteryPowerSetup.hLowVoltageThresholdPercentage + 100))/100;
    pHandle->hMaxContinuousPower = MCSetup.BatteryPowerSetup.hMaxBMSPositivePower * MotorParameters.PowerParameters.hEstimatedEfficiency / 100;
    pHandle->hMaxContinuousCurrent = MCSetup.BatteryPowerSetup.hMaxBMSContinuousCurrent;
    
    //Init power foldback
    FoldbackInitPower.bEnableFoldback = MCSetup.BatteryPowerSetup.bEnableMaxPowerLimit;
    FoldbackInitPower.hDefaultOutputLimitHigh = (uint16_t)(MCSetup.BatteryPowerSetup.hMaxApplicationPositivePower * MotorParameters.PowerParameters.hEstimatedEfficiency / 100);
    FoldbackInitPower.hDefaultOutputLimitLow = MCSetup.BatteryPowerSetup.hMaxBMSPositivePower * MotorParameters.PowerParameters.hEstimatedEfficiency / 100;
    FoldbackInitPower.hDecreasingEndValue = MCSetup.BatteryPowerSetup.wMaxTimeBMSTolerant;
    FoldbackInitPower.hDecreasingRange = MCSetup.BatteryPowerSetup.hMaxPowerLimitTimeout;
    
    Foldback_Init(&pHandle->FoldbackDynamicMaxPower, FoldbackInitPower);
}

void SpdTorqCtrl_SpeedLimitEnInit(SpdTorqCtrlHandle_t * pHandle, MC_Setup_t MCSetup)
{
    pHandle->bEnableSpdLimitControl = MCSetup.bEnSpeedLimit;
}

void SpdTorqCtrl_SetSpeedSensor(SpdTorqCtrlHandle_t * pHandle, SpdPosFdbkHandle_t * SPD_Handle)
{
    ASSERT(pHandle != NULL);
    pHandle->pSPD = SPD_Handle;
}


SpdPosFdbkHandle_t * SpdTorqCtrl_GetSpeedSensor(SpdTorqCtrlHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return (pHandle->pSPD);
}


void SpdTorqCtrl_Clear(SpdTorqCtrlHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    if (pHandle->Mode == STC_SPEED_MODE)
    {
        PID_SetIntegralTerm(pHandle->pPISpeed, 0);
    }
    
    PID_SetIntegralTerm(&pHandle->PISpeedLimit, pHandle->PISpeedLimit.wUpperIntegralLimit);

    RampMngr_Init(&pHandle->TorqueRampMngr);
    RampMngr_Init(&pHandle->SpeedRampMngr);
    SpdTorqCtrl_StopRamp(pHandle);
}


int16_t SpdTorqCtrl_GetMecSpeedRefUnit(SpdTorqCtrlHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return ((int16_t)(RampMngr_GetValue(&pHandle->SpeedRampMngr) / INT16_MAX));
}


int16_t SpdTorqCtrl_GetTorqueRef(SpdTorqCtrlHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return ((int16_t)(RampMngr_GetValue(&pHandle->TorqueRampMngr) / INT16_MAX));
}


void SpdTorqCtrl_SetControlMode(SpdTorqCtrlHandle_t * pHandle, STCModality_t bMode)
{
    ASSERT(pHandle != NULL);
    pHandle->Mode = bMode;
    SpdTorqCtrl_StopRamp(pHandle);
}


STCModality_t SpdTorqCtrl_GetControlMode(SpdTorqCtrlHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->Mode;
}


bool SpdTorqCtrl_ExecRamp(SpdTorqCtrlHandle_t * pHandle, int16_t hTargetFinal)
{
    ASSERT(pHandle != NULL);
    bool AllowedRange = true;
    int16_t hTargetFinalSat = hTargetFinal;

    /* Check if the hTargetFinal is out of bound of application. If it is out of bound,
         a limitation is applied. */
    if (pHandle->Mode == STC_TORQUE_MODE)
    {
        if ((int32_t)hTargetFinal > (int32_t)pHandle->hMaxPositiveTorque)
        {
            hTargetFinalSat = (int16_t) pHandle->hMaxPositiveTorque;
            AllowedRange = false;
        }
        if ((int32_t)hTargetFinal < (int32_t)pHandle->hMinNegativeTorque)
        {
            hTargetFinalSat = (int16_t) pHandle->hMinNegativeTorque;
            AllowedRange = false;
        }
    }
    else
    {
        if ((int32_t)hTargetFinal > (int32_t)pHandle->hMaxAppPositiveMecSpeedUnit)
        {
            hTargetFinalSat = (int16_t) pHandle->hMaxAppPositiveMecSpeedUnit;
            AllowedRange = false;
        }
        if (hTargetFinal < pHandle->hMinAppNegativeMecSpeedUnit)
        {
            AllowedRange = false;
            hTargetFinalSat = (int16_t) pHandle->hMinAppNegativeMecSpeedUnit;
        }
    }

    if (pHandle->Mode == STC_TORQUE_MODE)
    {
        pHandle->hFinalTorqueRef = hTargetFinalSat; // Store final torque value in handle
        if (abs(hTargetFinalSat) > abs(RampMngr_GetValue(&pHandle->TorqueRampMngr)))
        {
            RampMngr_ExecRamp(&pHandle->TorqueRampMngr, hTargetFinalSat, pHandle->wTorqueSlopePerSecondUp); // Setup torque ramp going up
        }
        else
        {
            RampMngr_ExecRamp(&pHandle->TorqueRampMngr, hTargetFinalSat, pHandle->wTorqueSlopePerSecondDown); // Setup torque ramp going down
        }
    }
    else
    {
        pHandle->hFinalSpeedRef = hTargetFinalSat; // Store final speed value in handle
        if (abs(hTargetFinalSat) > abs(RampMngr_GetValue(&pHandle->SpeedRampMngr)))
        {
            RampMngr_ExecRamp(&pHandle->SpeedRampMngr, hTargetFinalSat, pHandle->wSpeedSlopePerSecondUp); // Setup speed ramp going up
        }
        else
        {
            RampMngr_ExecRamp(&pHandle->SpeedRampMngr, hTargetFinalSat, pHandle->wSpeedSlopePerSecondDown); // Setup speed ramp going down
        }
    }

    return AllowedRange;
}


void SpdTorqCtrl_StopRamp(SpdTorqCtrlHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    RampMngr_StopRamp(&pHandle->TorqueRampMngr);
    RampMngr_StopRamp(&pHandle->SpeedRampMngr);
}

int16_t SpdTorqCtrl_CalcTorqueReference(SpdTorqCtrlHandle_t * pHandle, MotorParameters_t MotorParameters)
{
    ASSERT(pHandle != NULL);
    int16_t hTorqueReference = 0;
    int16_t hMeasuredSpeed;
    int16_t hTargetSpeed;
    int16_t hError;
    if (pHandle->Mode == STC_TORQUE_MODE)
    {
        hTorqueReference = (int16_t) (RampMngr_Calc(&pHandle->TorqueRampMngr)); // Apply torque ramp
        
          if (pHandle->motorType == DIRECT_DRIVE)
          {
              Foldback_UpdateLimitValue(&pHandle->FoldbackLimitSpeed, 0); // Update speed limit foldback
              Foldback_UpdateMaxValue(&pHandle->FoldbackLimitSpeed, hTorqueReference); // Update speed limit foldback
              Foldback_SetDecreasingEndValue(&pHandle->FoldbackLimitSpeed, (pHandle->hSpdLimit * SPEED_MARGIN/DIV_PERCENTAGE)); // Update speed limit by 10 percent
              Foldback_SetDecreasingRange(&pHandle->FoldbackLimitSpeed, DCC_RANGE); // Update speed limit foldback
          }
        if (pHandle->bEnableSpdLimitControl)
        {
            
            hMeasuredSpeed = (int16_t) abs(SpdPosFdbk_GetAvrgMecSpeedUnit(pHandle->pSPD)); // Speed is somehow negative when applying positive torque, need to figure out why.
            //bug filed for negative speed: https://tristarmulticopters.atlassian.net/browse/DEV-505
            //for mid-drives, get gear ratio if motor is spinning to set motor speed limit, else use wheel speed with gear ratio
            if (pHandle->motorType == MID_DRIVE)
            {
                if (pHandle->hCurrentTorqueRef > 0)
                {
                    SpdTorqCtrl_SetGearRatio(pHandle, hMeasuredSpeed);
                }
                else
                {
                    hMeasuredSpeed = (pHandle->pSPD->dynamic_gear)*(int16_t)(pHandle->pSPD->wheelRPM);
                }
            }
            
            if (pHandle->motorType == DIRECT_DRIVE)
            {
                hTorqueReference = Foldback_ApplyFoldback(&pHandle->FoldbackLimitSpeed, hTorqueReference, (int16_t) abs(hMeasuredSpeed));      // Apply speed limit foldback
            }
            else
            {
                hError = pHandle->hSpdLimit - hMeasuredSpeed; // Compute speed error
                pHandle->hTorqueReferenceSpdLim = PI_Controller(pHandle->pPISpeed, (int32_t)hError); // Compute torque value with PI controller
                if (pHandle->hTorqueReferenceSpdLim < hTorqueReference)
                {
                    hTorqueReference = pHandle->hTorqueReferenceSpdLim;
                }
            }
        }
        
        
        hTorqueReference = SpdTorqCtrl_ApplyPowerLimitation(pHandle, hTorqueReference); // Apply power limitation
        hTorqueReference = SpdTorqCtrl_ApplyTorqueFoldback(pHandle, hTorqueReference, MotorParameters); // Apply motor torque foldbacks
        /* Store values in handle */
        pHandle->hCurrentTorqueRef = hTorqueReference;

         if (pHandle->motorType == DIRECT_DRIVE)
          
         {
             if (pHandle->pSPD->hIdcRegen)
             {
                 if (pHandle->hCurrentTorqueRef == 0 && pHandle->pSPD->bActiveRegen)
                 {
                      if (abs(pHandle->pSPD->hAvrMecSpeedUnit) > MIN_REGEN_SPEED)
                      { 
                          pHandle->pSPD->hTorqueRegenMax = (int16_t)((pHandle->pSPD->hIdcRegen * pHandle->hBusVoltage * 100)/(abs(pHandle->pSPD->hAvrMecSpeedUnit)*PI_/30));
                          if (pHandle->pSPD->hTorqueRegen < pHandle->pSPD->hTorqueRegenMax)
                          {
                            pHandle->pSPD->hTorqueRegen = pHandle->pSPD->hTorqueRegenMax;  
                            
                            hTorqueReference = pHandle->pSPD->hTorqueRegen;
                          }
                          else
                          {
                            pHandle->pSPD->hTorqueRegen = pHandle->pSPD->hTorqueRegen + pHandle->pSPD->hDeltaT;
                            
                            hTorqueReference = pHandle->pSPD->hTorqueRegen;
                          }
                  
                  
                      }
                      else
                      {
                          pHandle->pSPD->hTorqueRegen = 0;
                      }
                 }
                
              }
        }  

    }
    else
    {
        hTargetSpeed = pHandle->hFinalSpeedRef;
        /*  the Ramp Manager bypassed in speed control because of an error caused by it
            Having two ramp manager (one for torque and one for speed over each other
            is not needed, so the bug 
            replace above line with below one to bring it back                           */
        /*  hTargetSpeed = (int16_t) RampMngr_Calc(&pHandle->SpeedRampMngr);             */
        
        /* Run the speed control loop */
        hMeasuredSpeed = -SpdPosFdbk_GetAvrgMecSpeedUnit(pHandle->pSPD); // Speed is somehow negative when applying positive torque, need to figure out why.
        hError = hTargetSpeed - hMeasuredSpeed; // Compute speed error
        hTorqueReference = PI_Controller(pHandle->pPISpeed, (int32_t)hError); // Compute final torque value with PI controller
        if (hTorqueReference < 0)
        {
            hTorqueReference = 0; // Hard-coded protection against regen during speed control. Todo: Make it more flexible.
        }
        
        // ramp manager for smooth Torque 
        if (abs(hTorqueReference) > abs(RampMngr_GetValue(&pHandle->TorqueRampMngr)))
        {
            RampMngr_ExecRamp(&pHandle->TorqueRampMngr, hTorqueReference, pHandle->wTorqueSlopePerSecondUp); // Setup torque ramp going up
        }
        else
        {
            RampMngr_ExecRamp(&pHandle->TorqueRampMngr, hTorqueReference, pHandle->wTorqueSlopePerSecondDown); // Setup torque ramp going down
        }
        hTorqueReference = (int16_t) RampMngr_Calc(&pHandle->TorqueRampMngr); // Apply torque ramp
        
        hTorqueReference = SpdTorqCtrl_ApplyPowerLimitation(pHandle, hTorqueReference); // Apply power limitation

        if (hTargetSpeed == 0)
        {
            pHandle->pPISpeed->wIntegralTerm = 0;   // reset integral term when target speed is zero tp prevent accumulation error
        } 
        
        /* Store values in handle */
        pHandle->hCurrentTorqueRef = hTorqueReference;
        pHandle->hCurrentSpeedRef = hTargetSpeed;
    }
    
    
    return hTorqueReference;
}

/*
    Calculates and sets the gear ratio
*/
void SpdTorqCtrl_SetGearRatio(SpdTorqCtrlHandle_t * pHandle, int16_t hMeasuredSpeed)
{
    int16_t gearTemp = 0;
    bool gearSame = true;
    
    //calculate gear ratio
    if (pHandle->pSPD->wheelRPM > 0)
    {
        gearTemp = (int16_t) hMeasuredSpeed/pHandle->pSPD->wheelRPM;
    }
    
    //compare calculated gear ratio with table
    for (uint8_t count = 0; count < NUM_GEARS; count++)
    {
        int16_t upperBound;
        int16_t lowerBound;
        
        //A gear ratio from the gear ratio table is chosen if the calculated gear ratio is between midpoint of the current gear and the previous gear,
        //and the midpoint of current gear and the next gear. For the lowest gear, the min calculated gear ratio considered is set to GEAR_RATIO_LOWER_BOUND_MIN,
        //For the highest gear, the max calculated gear ratio considered is set to the max gear ratio + GEAR_RATIO_UPPER_BOUND_MAX.
        if (count == 0)
        {
            upperBound = (gearRatioTable[count] + gearRatioTable[count + 1]) / 2;
            lowerBound = GEAR_RATIO_LOWER_BOUND_MIN;
        }
        else if (count == (NUM_GEARS - 1))
        {
            upperBound = gearRatioTable[count] + GEAR_RATIO_UPPER_BOUND_MAX;
            lowerBound = (gearRatioTable[count] + gearRatioTable[count - 1]) / 2;
        }
        else
        {
            upperBound = (gearRatioTable[count] + gearRatioTable[count + 1]) / 2;
            lowerBound = (gearRatioTable[count] + gearRatioTable[count - 1]) / 2;
        }
        
        //if gearTemp is within the bounds put the value in the filter
        if ((gearTemp <= upperBound) && (gearTemp >= lowerBound))
        {
            pHandle->pSPD->gearArray[pHandle->pSPD->gearMAFiltPos] = gearRatioTable[count];
            if (pHandle->pSPD->gearMAFiltPos < (GEAR_FILTER_SIZE - 1))
            {
                pHandle->pSPD->gearMAFiltPos++;
            }
            else
            {
                pHandle->pSPD->gearMAFiltPos = 0;
            }
        }
    }
    
    //if all numbers in filter are the same update gear ratio
    for(uint16_t count = 1; count < GEAR_FILTER_SIZE; count++)
    {
        if (pHandle->pSPD->gearArray[count] != pHandle->pSPD->gearArray[0])
        {
            gearSame = false;
        }
    }
                
    if (gearSame == true)
    {
        pHandle->pSPD->dynamic_gear = pHandle->pSPD->gearArray[pHandle->pSPD->gearMAFiltPos];
    }        
}


uint16_t SpdTorqCtrl_GetMaxAppPositiveMecSpeedUnit(SpdTorqCtrlHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->hMaxAppPositiveMecSpeedUnit;
}


int16_t SpdTorqCtrl_GetMinAppNegativeMecSpeedUnit(SpdTorqCtrlHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->hMinAppNegativeMecSpeedUnit;
}


bool SpdTorqCtrl_IsRampCompleted(SpdTorqCtrlHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    bool retVal = false;

    if (pHandle->Mode == STC_TORQUE_MODE)
    {
        retVal = RampMngr_IsRampCompleted(&pHandle->TorqueRampMngr);
    }
    else
    {
        retVal = RampMngr_IsRampCompleted(&pHandle->SpeedRampMngr);
    }

    return retVal;
}


void SpdTorqCtrl_ForceSpeedReferenceToCurrentSpeed(SpdTorqCtrlHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    RampMngr_ExecRamp(&pHandle->SpeedRampMngr, (int32_t)SpdPosFdbk_GetAvrgMecSpeedUnit(pHandle->pSPD) * (int32_t)65536, 0);
}

/*
    Set torque ramp slope values, for ramping up and ramping down.
*/
void SpdTorqCtrl_SetTorqueRampSlope(SpdTorqCtrlHandle_t * pHandle, uint32_t wSlopePerSecondUp, uint32_t wSlopePerSecondDown)
{
    ASSERT(pHandle != NULL);
    pHandle->wTorqueSlopePerSecondUp = wSlopePerSecondUp;
    pHandle->wTorqueSlopePerSecondDown = wSlopePerSecondDown;
}


/*
    Set speed ramp slope values, for ramping up and ramping down.
*/
void SpdTorqCtrl_SetSpeedRampSlope(SpdTorqCtrlHandle_t * pHandle, uint32_t wSlopePerSecondUp, uint32_t wSlopePerSecondDown)
{
    ASSERT(pHandle != NULL);
    pHandle->wSpeedSlopePerSecondUp = wSlopePerSecondUp;
    pHandle->wSpeedSlopePerSecondDown = wSlopePerSecondDown;
}

/*
    Apply all torque foldbacks and returns limited torque.
*/
static int16_t SpdTorqCtrl_ApplyTorqueFoldback(SpdTorqCtrlHandle_t * pHandle, int16_t hInputTorque, MotorParameters_t MotorParameters)
{
    
    ASSERT(pHandle != NULL);
        
    int16_t hMeasuredSpeed = 0;
    int16_t hMeasuredMotorTemp = 0;
    int16_t hMeasuredHeatsinkTemp = 0;

    hMeasuredSpeed = 10 * SpdPosFdbk_GetAvrgMecSpeedUnit(pHandle->pSPD);

    if (NTCTempSensor_GetSensorType(NTC_MOTOR) == NO_SENSOR)
    {
        hMeasuredMotorTemp = NTCTempSensor_GetAvTempCelcius(NTC_MOTOR) * 100;
    }
    if (NTCTempSensor_GetSensorType(NTC_CONTROLLER) == NO_SENSOR)
    {
        hMeasuredHeatsinkTemp = NTCTempSensor_GetAvTempCelcius(NTC_CONTROLLER) * 100;
    }
        
    int16_t hOutputTorque, hMaxTorque; 
    hMaxTorque = Foldback_ApplyFoldback(&pHandle->FoldbackDynamicMaxTorque, NULL,(int16_t) abs(hMeasuredSpeed) );
    Foldback_UpdateMaxValue(&pHandle->FoldbackMotorSpeed, hMaxTorque);
    Foldback_UpdateMaxValue(&pHandle->FoldbackMotorTemperature, hMaxTorque);
    Foldback_UpdateMaxValue(&pHandle->FoldbackControllerTemperature, hMaxTorque);
    
    //for mid-drives, we need to also set the max wheel speed since it can vary based on the gear with a constant motor speed
    if (pHandle->motorType == MID_DRIVE)
    {
    
        int16_t hMaxWheelSpeed = 0;
                
        hMaxWheelSpeed = (int16_t) pHandle->hSpdLimitWheelRpm;
        
        if (pHandle->pSPD->dynamic_gear == 0)
        {
            pHandle->pSPD->dynamic_gear = gearRatioTable[NUM_GEARS - 1];
        }

        pHandle->FoldbackMotorSpeed.hDecreasingEndValue = 10 * hMaxWheelSpeed * pHandle->pSPD->dynamic_gear; //multiplied by 10 for higher accuracy in foldback
        pHandle->FoldbackMotorSpeed.hDecreasingRange = (MotorParameters.SpeedParameters.hFoldbackSpeedInterval / MotorParameters.SpeedParameters.hMaxAppliationSpeedRPM) * (uint16_t) pHandle->FoldbackMotorSpeed.hDecreasingEndValue;
    }
    
    hOutputTorque = Foldback_ApplyFoldback(&pHandle->FoldbackMotorSpeed, hInputTorque,(int16_t)abs(hMeasuredSpeed)); 
    hOutputTorque = Foldback_ApplyFoldback(&pHandle->FoldbackMotorTemperature, hOutputTorque, hMeasuredMotorTemp);
    hOutputTorque = Foldback_ApplyFoldback(&pHandle->FoldbackControllerTemperature, hOutputTorque, hMeasuredHeatsinkTemp);
    
    //limit Torque when the Battery SoC is low to prevent UNDERVOLTAGE fault
    if(pHandle->bEnableLVtorqueLimit)
    {
        if (pHandle->hBusVoltage < pHandle->hBatteryLowVoltage && hOutputTorque > pHandle->hLowBatteryTorque)
        {
            hOutputTorque = pHandle->hLowBatteryTorque;
        }
    }
    
    
    return hOutputTorque;
}


int16_t SpdTorqCtrl_GetIqFromTorqueRef(SpdTorqCtrlHandle_t * pHandle, int16_t hTorqueRef)
{
    ASSERT(pHandle != NULL); 

    float fTemp;

    fTemp = (float) (hTorqueRef * pHandle->fGainTorqueIqref);
    if (fTemp > INT16_MAX)
    {
        fTemp = INT16_MAX;
    }
    if (fTemp < INT16_MIN)
    {
        fTemp = INT16_MIN;
    }

    return (int16_t) fTemp;
}

int16_t SpdTorqCtrl_GetIdFromTorqueRef(SpdTorqCtrlHandle_t * pHandle, int16_t hTorqueRef)
{
    float fTemp;

    fTemp = (float) (hTorqueRef * pHandle->fGainTorqueIdref);
    if (fTemp > INT16_MAX)
    {
        fTemp = INT16_MAX;
    }
    if (fTemp < INT16_MIN)
    {
        fTemp = INT16_MIN;
    }

    return (int16_t) fTemp;
}

/*
    Apply motor power limitation to torque reference
*/
static int16_t SpdTorqCtrl_ApplyPowerLimitation(SpdTorqCtrlHandle_t * pHandle, int16_t hInputTorque)
{
    ASSERT(pHandle != NULL); 
    
    int32_t wTorqueLimit = 0;
    int16_t hMeasuredSpeedTenthRadPerSec = 0;
    int16_t hMeasuredSpeedUnit = 0;
    int16_t hRetval = hInputTorque;
  
    hMeasuredSpeedUnit = SpdPosFdbk_GetAvrgMecSpeedUnit(pHandle->pSPD);
    hMeasuredSpeedTenthRadPerSec = (int16_t)((10*hMeasuredSpeedUnit*2*3.1416F)/SPEED_UNIT);


    // Limit MAX POWER by the foldback to prevent BMS shutdown
    Foldback_UpdateMaxValue(&pHandle->FoldbackDynamicMaxPower, (int16_t)pHandle->hMaxPositivePower);        // this foldback limits MAX POWER after a while
    Foldback_UpdateLimitValue(&pHandle->FoldbackDynamicMaxPower, (int16_t)pHandle->hMaxContinuousPower);      // this foldback limits MAX POWER immediately
    Foldback_SetDecreasingEndValue(&pHandle->FoldbackLimitSpeed, (pHandle->hSpdLimit * SPEED_MARGIN/DIV_PERCENTAGE)); // Update speed limit foldback
    Foldback_SetDecreasingRange(&pHandle->FoldbackLimitSpeed, DECC_RANGE_PW); // Update speed limit foldback
  
    pHandle->DynamicPowerHandle.hDynamicMaxPower = (uint16_t)Foldback_ApplyFoldback(&pHandle->FoldbackDynamicMaxPower, (int16_t)pHandle->hMaxPositivePower, (int16_t)pHandle->DynamicPowerHandle.hOverMaxPowerTimer);    

    if (hMeasuredSpeedUnit != 0)
    {
        if (hInputTorque > 0)
        {
                wTorqueLimit = 1000*pHandle->DynamicPowerHandle.hDynamicMaxPower/abs(hMeasuredSpeedTenthRadPerSec); // Torque limit in cNm. 1000 comes from 100*10
            
            if (hInputTorque > wTorqueLimit)
            {
                //limit the toque and start timer to count on max torque elapsed timer used by foldback to reduce power after a while
                hRetval = (int16_t) wTorqueLimit;
                /* Check if timer is not exceeds the end of foldback */
                if (pHandle->DynamicPowerHandle.hOverMaxPowerTimer < pHandle->FoldbackDynamicMaxPower.hDecreasingEndValue)
                {
                    pHandle->DynamicPowerHandle.hOverMaxPowerTimer++;
                }
            }
            else
            {
                // if the power is less that dynamic maximum power for a short time, then reset dynamic max power to positive max power
                pHandle->DynamicPowerHandle.hBelowMaxPowerTimer++;
                if (pHandle->DynamicPowerHandle.hBelowMaxPowerTimer > pHandle->DynamicPowerHandle.hBelowMaxPowerTimeout)
                {
                    // clear timers and put max power back to the default defined value
                    pHandle->DynamicPowerHandle.hOverMaxPowerTimer = 0;
                    pHandle->DynamicPowerHandle.hBelowMaxPowerTimer = 0;
                    pHandle->DynamicPowerHandle.hDynamicMaxPower = pHandle->hMaxPositivePower;
                }
            }
        }
        else
        {
            wTorqueLimit = 1000*pHandle->hMinNegativePower/abs(hMeasuredSpeedTenthRadPerSec); // Torque limit in cNm. 1000 comes from 100*10
            
            if (hInputTorque < wTorqueLimit)
            {
                hRetval = (int16_t) wTorqueLimit;
            }
        }
    }
    
    
    return hRetval;
}

/*
    Apply nominal current limit
*/
void SpdTorqCtrl_ApplyCurrentLimitation_Iq(qd_t * pHandle, int16_t NominalCurrent, int16_t userMaxCurrent)
{
    ASSERT(pHandle != NULL); 
    int16_t hMaxLimit = NominalCurrent;
    
    if (userMaxCurrent < NominalCurrent)
    {
        hMaxLimit = userMaxCurrent;
    }
    
    if (pHandle->q > hMaxLimit)
    {
        pHandle->q = hMaxLimit;
    }
    else if (pHandle->q < -hMaxLimit)
    {
        pHandle->q = -(int16_t)hMaxLimit;
    }
}

/*
    calculate and update Max Positive Power based on Bus Voltage in CURRENT LIMIT mode
*/
void MC_AdaptiveMaxPower(SpdTorqCtrlHandle_t * pHandle)
{
    #if POWER_LIMIT_REF == MAX_CURRENT_LIMIT
    pHandle->hMaxPositivePower = pHandle->hMaxBusCurrent * pHandle->hBusVoltage * pHandle->hEstimatedEfficiencyPercent / 100;
    pHandle->hMaxContinuousPower = pHandle->hMaxContinuousCurrent * pHandle->hBusVoltage * pHandle->hEstimatedEfficiencyPercent / 100;
    #else
        UNUSED_PARAMETER(pHandle);
        //do nothing - keep it at maximum defined level 
    #endif
}  

/*
    set speed limit of motor
*/
void SpdTorqCtrl_SetSpeedLimit(SpdTorqCtrlHandle_t * pHandle, int16_t hSpdLimUnit)
{
    ASSERT(pHandle != NULL);
    pHandle->hSpdLimit = hSpdLimUnit;
}

/*
    set speed limit of wheel
*/
void SpdTorqCtrl_SetSpeedLimitWheelRpm(SpdTorqCtrlHandle_t * pHandle, uint16_t hSpdLimUnit)
{
    ASSERT(pHandle != NULL);
    pHandle->hSpdLimitWheelRpm = hSpdLimUnit;
}

