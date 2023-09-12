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
#include "parameters_conversion.h"

static int16_t SpdTorqCtrl_ApplyTorqueFoldback(SpdTorqCtrlHandle_t * pHandle, int16_t hInputTorque);
static int16_t SpdTorqCtrl_ApplyPowerLimitation(SpdTorqCtrlHandle_t * pHandle, int16_t hInputTorque);
uint16_t M_STUCK_timer = 0;
uint16_t OCD_timer = 0;
    
void SpdTorqCtrl_Init(SpdTorqCtrlHandle_t * pHandle, PIDHandle_t * pPI, SpdPosFdbkHandle_t * SPD_Handle,
                        NTCTempSensorHandle_t* pTempSensorHS, NTCTempSensorHandle_t* pTempSensorMotor)
{
    ASSERT(pHandle != NULL);
    pHandle->pPISpeed = pPI;
    pHandle->pSPD = SPD_Handle;
    pHandle->pHeatsinkTempSensor = pTempSensorHS;
    pHandle->pMotorTempSensor = pTempSensorMotor;
    pHandle->Mode = pHandle->ModeDefault;
  
    if (pTempSensorHS == NULL)
    {
        Foldback_DisableFoldback(&pHandle->FoldbackHeatsinkTemperature);
    }
    if (pTempSensorMotor == NULL)
    {
        Foldback_DisableFoldback(&pHandle->FoldbackMotorTemperature);
    }
   
    pHandle->TorqueRampMngr.wFrequencyHz = pHandle->hSTCFrequencyHz;
    pHandle->SpeedRampMngr.wFrequencyHz = pHandle->hSTCFrequencyHz;
    RampMngr_Init(&pHandle->TorqueRampMngr);
    RampMngr_Init(&pHandle->SpeedRampMngr);
    DynamicPower_Init(&pHandle->DynamicPowerHandle);
    StuckProtection_Init(&pHandle->StuckProtection);
    
    PID_Init(&pHandle->PISpeedLimit);
    
#if HARDWARE_OCD == OCD_POWER_DERATING
    OCD2_Init(&pHandle->OCD2_Handle);
#else
    //do nothing here
#endif
    
    Foldback_Init(&pHandle->FoldbackDynamicMaxTorque);
    Foldback_Init(&pHandle->FoldbackHeatsinkTemperature);
    Foldback_Init(&pHandle->FoldbackMotorSpeed);
    Foldback_Init(&pHandle->FoldbackDynamicMaxPower);
    Foldback_Init(&pHandle->FoldbackMotorTemperature);
    
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


int16_t SpdTorqCtrl_CalcTorqueReference(SpdTorqCtrlHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    int16_t hTorqueReference = 0;
    int16_t hMeasuredSpeed;
    int16_t hTargetSpeed;
    int16_t hError;

    if (pHandle->Mode == STC_TORQUE_MODE)
    {
        hTorqueReference = (int16_t) (RampMngr_Calc(&pHandle->TorqueRampMngr)); // Apply torque ramp
        
        
        if (pHandle->bEnableSpdLimitControl)
        {
            hMeasuredSpeed = -SpdPosFdbk_GetAvrgMecSpeedUnit(pHandle->pSPD); // Speed is somehow negative when applying positive torque, need to figure out why.
            hError = pHandle->hSpdLimit - hMeasuredSpeed; // Compute speed error
            pHandle->hTorqueReferenceSpdLim = PI_Controller(pHandle->pPISpeed, (int32_t)hError); // Compute torque value with PI controller
            if (pHandle->hTorqueReferenceSpdLim < hTorqueReference)
            {
                hTorqueReference = pHandle->hTorqueReferenceSpdLim;
            }
        }
        
        hTorqueReference = SpdTorqCtrl_ApplyPowerLimitation(pHandle, hTorqueReference); // Apply power limitation
        hTorqueReference = SpdTorqCtrl_ApplyTorqueFoldback(pHandle, hTorqueReference); // Apply motor torque foldbacks
        /* Store values in handle */
        pHandle->hCurrentTorqueRef = hTorqueReference;
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

#if HARDWARE_OCD == OCD_POWER_DERATING
    //Here the OCD2 (PC14) is checked to see Over Current feedback is received from current sesnor
    hTorqueReference = SpdTorqCtrl_ApplyIncrementalPowerDerating(pHandle, hTorqueReference);
#else
    // nothing to do here

#endif
    
    return hTorqueReference;
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
static int16_t SpdTorqCtrl_ApplyTorqueFoldback(SpdTorqCtrlHandle_t * pHandle, int16_t hInputTorque)
{
    ASSERT(pHandle != NULL);
    
    int16_t hMeasuredSpeed = 0;
    int16_t hMeasuredMotorTemp = 0;
    int16_t hMeasuredHeatsinkTemp = 0;

    hMeasuredSpeed = 10 * SpdPosFdbk_GetAvrgMecSpeedUnit(pHandle->pSPD);
    if (pHandle->pMotorTempSensor != NULL)
    {
        hMeasuredMotorTemp = NTCTempSensor_GetAvTempCelcius(pHandle->pMotorTempSensor) * 100;
    }
    if (pHandle->pHeatsinkTempSensor != NULL)
    {
        hMeasuredHeatsinkTemp = NTCTempSensor_GetAvTempCelcius(pHandle->pHeatsinkTempSensor) * 100;
    }

    int16_t hOutputTorque, hMaxTorque; 
    hMaxTorque = Foldback_ApplyFoldback(&pHandle->FoldbackDynamicMaxTorque, NULL,(int16_t) abs(hMeasuredSpeed) );
    Foldback_UpdateMaxValue(&pHandle->FoldbackMotorSpeed, hMaxTorque);
    Foldback_UpdateMaxValue(&pHandle->FoldbackMotorTemperature, hMaxTorque);
    Foldback_UpdateMaxValue(&pHandle->FoldbackHeatsinkTemperature, hMaxTorque);
    
    hOutputTorque = Foldback_ApplyFoldback(&pHandle->FoldbackMotorSpeed, hInputTorque,(int16_t)(abs(hMeasuredSpeed)));
    hOutputTorque = Foldback_ApplyFoldback(&pHandle->FoldbackMotorTemperature, hOutputTorque, hMeasuredMotorTemp);
    hOutputTorque = Foldback_ApplyFoldback(&pHandle->FoldbackHeatsinkTemperature, hOutputTorque, hMeasuredHeatsinkTemp);
    
    #ifdef LOW_BATTERY_TORQUE
    //limit Torque when the Battery SoC is low to prevent UNDERVOLTAGE fault
    if (pHandle->hBusVoltage < pHandle->hBatteryLowVoltage && hOutputTorque > LOW_BATTERY_TORQUE)
    {
        hOutputTorque = LOW_BATTERY_TORQUE;
    }
    #endif 
    
    
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
    If Over Current Detected flag is raised, this function decreses torq until flag clear
*/
int16_t SpdTorqCtrl_ApplyIncrementalPowerDerating(SpdTorqCtrlHandle_t * pHandle, int16_t hInputTorque)
{
    ASSERT(pHandle != NULL); 
    
    float hNewTorque = hInputTorque;
    int16_t hRetval = 0;
        
    if (OCD2_IsEnabled(&pHandle->OCD2_Handle))
    {
        pHandle->HWOverCurrentDetection.bIsOverCurrentDetected = true;
        
        if (OCD_timer < pHandle->HWOverCurrentDetection.OCDTimeInterval)
        {
            OCD_timer++;
        }
        else
        {
            pHandle->HWOverCurrentDetection.OCDPowerDearatingGain *= pHandle->HWOverCurrentDetection.OCDPowerDeratingSlope;
            OCD_timer = 0;
        }
    }
    else 
    {
        OCD_timer = 0;
    }
    hRetval = (int16_t)(hNewTorque * pHandle->HWOverCurrentDetection.OCDPowerDearatingGain);
    return hRetval;
}

/*
    calculate and update Max Positive Power based on Bus Voltage in CURRENT LIMIT mode
*/
void MC_AdaptiveMaxPower(SpdTorqCtrlHandle_t * pHandle)
{
    #if POWER_LIMIT_REF == MAX_CURRENT_LIMIT
    pHandle->hMaxPositivePower = pHandle->hMaxBusCurrent * pHandle->hBusVoltage;
    #else
        UNUSED_PARAMETER(pHandle);
        //do nothing - keep it at maximum defined level 
    #endif
}  

void SpdTorqCtrl_SetSpeedLimit(SpdTorqCtrlHandle_t * pHandle, int16_t hSpdLimUnit)
{
    pHandle->hSpdLimit = hSpdLimUnit;
}

