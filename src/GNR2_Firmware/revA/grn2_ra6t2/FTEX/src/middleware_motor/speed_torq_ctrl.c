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

static int16_t SpdTorqCtrl_ApplyTorqueFoldback(SpeednTorqCtrlHandle_t * pHandle, int16_t hInputTorque);

void SpdTorqCtrl_Init(SpeednTorqCtrlHandle_t * pHandle, PIDHandle_t * pPI, SpeednPosFdbkHandle_t * SPD_Handle,
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
}


void SpdTorqCtrl_SetSpeedSensor(SpeednTorqCtrlHandle_t * pHandle, SpeednPosFdbkHandle_t * SPD_Handle)
{
    ASSERT(pHandle != NULL);
    pHandle->pSPD = SPD_Handle;
}


SpeednPosFdbkHandle_t * SpdTorqCtrl_GetSpeedSensor(SpeednTorqCtrlHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return (pHandle->pSPD);
}


void SpdTorqCtrl_Clear(SpeednTorqCtrlHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    if (pHandle->Mode == STC_SPEED_MODE)
    {
        PID_SetIntegralTerm(pHandle->pPISpeed, 0);
    }
    
    RampMngr_Init(&pHandle->TorqueRampMngr);
    RampMngr_Init(&pHandle->SpeedRampMngr);
    SpdTorqCtrl_StopRamp(pHandle);
}


int16_t SpdTorqCtrl_GetMecSpeedRefUnit(SpeednTorqCtrlHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return ((int16_t)(RampMngr_GetValue(&pHandle->SpeedRampMngr) / INT16_MAX));
}


int16_t SpdTorqCtrl_GetTorqueRef(SpeednTorqCtrlHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return ((int16_t)(RampMngr_GetValue(&pHandle->TorqueRampMngr) / INT16_MAX));
}


void SpdTorqCtrl_SetControlMode(SpeednTorqCtrlHandle_t * pHandle, STCModality_t bMode)
{
    ASSERT(pHandle != NULL);
    pHandle->Mode = bMode;
    SpdTorqCtrl_StopRamp(pHandle);
}


STCModality_t SpdTorqCtrl_GetControlMode(SpeednTorqCtrlHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->Mode;
}


bool SpdTorqCtrl_ExecRamp(SpeednTorqCtrlHandle_t * pHandle, int16_t hTargetFinal)
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
        else if (hTargetFinal < pHandle->hMinAppNegativeMecSpeedUnit)
        {
            AllowedRange = false;
            hTargetFinalSat = (int16_t) pHandle->hMinAppNegativeMecSpeedUnit;
        }
        else if ((int32_t)hTargetFinal < (int32_t)pHandle->hMinAppPositiveMecSpeedUnit)
        {
            if (hTargetFinal > pHandle->hMaxAppNegativeMecSpeedUnit)
            {
                if (hTargetFinal >= 0)
                {
                    hTargetFinalSat = (int16_t) pHandle->hMinAppPositiveMecSpeedUnit;
                    AllowedRange = false;
                }
            }
        }
        else if ((int32_t)hTargetFinal > (int32_t)pHandle->hMaxAppNegativeMecSpeedUnit)
        {
            if (hTargetFinal > pHandle->hMinAppPositiveMecSpeedUnit)
            {
                if (hTargetFinal < 0)
                {
                    hTargetFinalSat = (int16_t) pHandle->hMaxAppNegativeMecSpeedUnit;
                    AllowedRange = false;
                }
            }
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
        if (abs(hTargetFinalSat) > abs(RampMngr_GetValue(&pHandle->TorqueRampMngr)))
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


void SpdTorqCtrl_StopRamp(SpeednTorqCtrlHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    RampMngr_StopRamp(&pHandle->TorqueRampMngr);
    RampMngr_StopRamp(&pHandle->SpeedRampMngr);
}


int16_t SpdTorqCtrl_CalcTorqueReference(SpeednTorqCtrlHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    int16_t hTorqueReference = 0;
    int16_t hMeasuredSpeed;
    int16_t hTargetSpeed;
    int16_t hError;

    if (pHandle->Mode == STC_TORQUE_MODE)
    {
        hTorqueReference = (int16_t) (RampMngr_Calc(&pHandle->TorqueRampMngr)); // Apply torque ramp
        hTorqueReference = SpdTorqCtrl_ApplyTorqueFoldback(pHandle, hTorqueReference); // Apply motor torque foldbacks
        /* Store values in handle */
        pHandle->hCurrentTorqueRef = hTorqueReference;
    }
    else
    {
        hTargetSpeed = (int16_t) RampMngr_Calc(&pHandle->SpeedRampMngr);
        /* Run the speed control loop */
        hMeasuredSpeed = SpdPosFdbk_GetAvrgMecSpeedUnit(pHandle->pSPD);
        hError = hTargetSpeed - hMeasuredSpeed; // Compute speed error
        hTorqueReference = PI_Controller(pHandle->pPISpeed, (int32_t)hError); // Compute final torque value with PI controller
        if (abs(hTorqueReference) > abs(RampMngr_GetValue(&pHandle->TorqueRampMngr)))
        {
            RampMngr_ExecRamp(&pHandle->TorqueRampMngr, hTorqueReference, pHandle->wTorqueSlopePerSecondUp); // Setup torque ramp going up
        }
        else
        {
            RampMngr_ExecRamp(&pHandle->TorqueRampMngr, hTorqueReference, pHandle->wTorqueSlopePerSecondDown); // Setup torque ramp going down
        }
        hTorqueReference = (int16_t) RampMngr_Calc(&pHandle->TorqueRampMngr); // Apply torque ramp
        hTorqueReference = SpdTorqCtrl_ApplyTorqueFoldback(pHandle, hTorqueReference); // Apply motor torque foldbacks
        /* Store values in handle */
        pHandle->hCurrentTorqueRef = hTorqueReference;
        pHandle->hCurrentSpeedRef = hTargetSpeed;
    }

    return hTorqueReference;
}


uint16_t SpdTorqCtrl_GetMaxAppPositiveMecSpeedUnit(SpeednTorqCtrlHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->hMaxAppPositiveMecSpeedUnit;
}


int16_t SpdTorqCtrl_GetMinAppNegativeMecSpeedUnit(SpeednTorqCtrlHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->hMinAppNegativeMecSpeedUnit;
}


bool SpdTorqCtrl_IsRampCompleted(SpeednTorqCtrlHandle_t * pHandle)
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


void SpdTorqCtrl_ForceSpeedReferenceToCurrentSpeed(SpeednTorqCtrlHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    RampMngr_ExecRamp(&pHandle->SpeedRampMngr, (int32_t)SpdPosFdbk_GetAvrgMecSpeedUnit(pHandle->pSPD) * (int32_t)65536, 0);
}

/*
    Set torque ramp slope values, for ramping up and ramping down.
*/
void SpdTorqCtrl_SetTorqueRampSlope(SpeednTorqCtrlHandle_t * pHandle, uint32_t wSlopePerSecondUp, uint32_t wSlopePerSecondDown)
{
    ASSERT(pHandle != NULL);
    pHandle->wTorqueSlopePerSecondUp = wSlopePerSecondUp;
    pHandle->wTorqueSlopePerSecondDown = wSlopePerSecondDown;
}


/*
    Set speed ramp slope values, for ramping up and ramping down.
*/
void SpdTorqCtrl_SetSpeedRampSlope(SpeednTorqCtrlHandle_t * pHandle, uint32_t wSlopePerSecondUp, uint32_t wSlopePerSecondDown)
{
    ASSERT(pHandle != NULL);
    pHandle->wSpeedSlopePerSecondUp = wSlopePerSecondUp;
    pHandle->wSpeedSlopePerSecondDown = wSlopePerSecondDown;
}

/*
    Apply all torque foldbacks and returns limited torque.
*/
static int16_t SpdTorqCtrl_ApplyTorqueFoldback(SpeednTorqCtrlHandle_t * pHandle, int16_t hInputTorque)
{
    ASSERT(pHandle != NULL);
    int16_t hMeasuredSpeed = 0;
    int16_t hMeasuredMotorTemp = 0;
    int16_t hMeasuredHeatsinkTemp = 0;

    hMeasuredSpeed = SpdPosFdbk_GetAvrgMecSpeedUnit(pHandle->pSPD);
    if (pHandle->pMotorTempSensor != NULL)
    {
        hMeasuredMotorTemp = NTCTempSensor_GetAvTempCelcius(pHandle->pMotorTempSensor);
    }
    if (pHandle->pMotorTempSensor != NULL)
    {
        hMeasuredHeatsinkTemp = NTCTempSensor_GetAvTempCelcius(pHandle->pHeatsinkTempSensor);
    }

    int16_t hOutputTorque;
    hOutputTorque = Foldback_ApplyFoldback(&pHandle->FoldbackMotorSpeed, hInputTorque, abs(hMeasuredSpeed));
    hOutputTorque = Foldback_ApplyFoldback(&pHandle->FoldbackMotorTemperature, hOutputTorque, hMeasuredMotorTemp);
    hOutputTorque = Foldback_ApplyFoldback(&pHandle->FoldbackHeatsinkTemperature, hOutputTorque, hMeasuredHeatsinkTemp);

    return hOutputTorque;
}


int16_t SpdTorqCtrl_GetIqFromTorqueRef(SpeednTorqCtrlHandle_t * pHandle, int16_t hTorqueRef)
{
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

int16_t SpdTorqCtrl_GetIdFromTorqueRef(SpeednTorqCtrlHandle_t * pHandle, int16_t hTorqueRef)
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
    

