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
static int16_t SpdTorqCtrl_ApplyPowerLimitation(SpeednTorqCtrlHandle_t * pHandle, int16_t hInputTorque);

void SpdTorqCtrl_Init(SpeednTorqCtrlHandle_t * pHandle, PID_Handle_t * pPI, SpeednPosFdbk_Handle_t * SPD_Handle,
                        NTC_Handle_t* pTempSensorHS, NTC_Handle_t* pTempSensorMotor)
{
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
		
		SignalFiltering_Init(&pHandle->SpeedFilter);
    SignalFiltering_ConfigureRecursiveAverage(&pHandle->SpeedFilter,pHandle->hSpeedFilterLength );
}


void SpdTorqCtrl_SetSpeedSensor(SpeednTorqCtrlHandle_t * pHandle, SpeednPosFdbk_Handle_t * SPD_Handle)
{

    pHandle->pSPD = SPD_Handle;
}


SpeednPosFdbk_Handle_t * SpdTorqCtrl_GetSpeedSensor(SpeednTorqCtrlHandle_t * pHandle)
{

    return (pHandle->pSPD);
}


void SpdTorqCtrl_Clear(SpeednTorqCtrlHandle_t * pHandle)
{

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

    return ((int16_t)(RampMngr_GetValue(&pHandle->SpeedRampMngr) / INT16_MAX));
}


int16_t SpdTorqCtrl_GetTorqueRef(SpeednTorqCtrlHandle_t * pHandle)
{

    return ((int16_t)(RampMngr_GetValue(&pHandle->TorqueRampMngr) / INT16_MAX));
}


void SpdTorqCtrl_SetControlMode(SpeednTorqCtrlHandle_t * pHandle, STC_Modality_t bMode)
{

    pHandle->Mode = bMode;
    SpdTorqCtrl_StopRamp(pHandle);
}


STC_Modality_t SpdTorqCtrl_GetControlMode(SpeednTorqCtrlHandle_t * pHandle)
{

    return pHandle->Mode;
}


bool SpdTorqCtrl_ExecRamp(SpeednTorqCtrlHandle_t * pHandle, int16_t hTargetFinal)
{

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
        pHandle->hFinalTorque = hTargetFinalSat; // Store final torque value in handle
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
        pHandle->hFinalSpeed = hTargetFinalSat; // Store final speed value in handle
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

    RampMngr_StopRamp(&pHandle->TorqueRampMngr);
    RampMngr_StopRamp(&pHandle->SpeedRampMngr);
}


int16_t SpdTorqCtrl_CalcTorqueReference(SpeednTorqCtrlHandle_t * pHandle)
{

    int16_t hTorqueReference = 0;
    int16_t hMeasuredSpeed;
    int16_t hTargetSpeed;
    int16_t hError;

    if (pHandle->Mode == STC_TORQUE_MODE)
    {
        hTorqueReference = (int16_t) (RampMngr_Calc(&pHandle->TorqueRampMngr)); // Apply torque ramp
        hTorqueReference = SpdTorqCtrl_ApplyTorqueFoldback(pHandle, hTorqueReference); // Apply motor torque foldbacks
        hTorqueReference = SpdTorqCtrl_ApplyPowerLimitation(pHandle, hTorqueReference); // Apply power limitation
        /* Store values in handle */
        pHandle->hCurrentTorqueRef = hTorqueReference;
    }
    else
    {
        hTargetSpeed = (int16_t) RampMngr_Calc(&pHandle->SpeedRampMngr);
        /* Run the speed control loop */
        hMeasuredSpeed = SPD_GetAvrgMecSpeedUnit(pHandle->pSPD);
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
        hTorqueReference = SpdTorqCtrl_ApplyPowerLimitation(pHandle, hTorqueReference); // Apply power limitation
        /* Store values in handle */
        pHandle->hCurrentTorqueRef = hTorqueReference;
        pHandle->hCurrentSpeedRef = hTargetSpeed;
    }

    return hTorqueReference;
}


int16_t SpdTorqCtrl_GetMecSpeedRefUnitDefault(SpeednTorqCtrlHandle_t * pHandle)
{

    return pHandle->hMecSpeedRefUnitDefault;
}


uint16_t SpdTorqCtrl_GetMaxAppPositiveMecSpeedUnit(SpeednTorqCtrlHandle_t * pHandle)
{

    return pHandle->hMaxAppPositiveMecSpeedUnit;
}


int16_t SpdTorqCtrl_GetMinAppNegativeMecSpeedUnit(SpeednTorqCtrlHandle_t * pHandle)
{

    return pHandle->hMinAppNegativeMecSpeedUnit;
}


bool SpdTorqCtrl_IsRampCompleted(SpeednTorqCtrlHandle_t * pHandle)
{

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


qd_t SpdTorqCtrl_GetDefaultIqdref(SpeednTorqCtrlHandle_t * pHandle)
{

    qd_t IqdRefDefault;
    IqdRefDefault.q = pHandle->hTorqueRefDefault;
    IqdRefDefault.d = pHandle->hIdrefDefault;
    return IqdRefDefault;
}


void SpdTorqCtrl_SetNominalCurrent(SpeednTorqCtrlHandle_t * pHandle, uint16_t hNominalCurrent)
{

    pHandle->hMaxPositiveTorque = hNominalCurrent;
    pHandle->hMinNegativeTorque = -hNominalCurrent;
}


void SpdTorqCtrl_ForceSpeedReferenceToCurrentSpeed(SpeednTorqCtrlHandle_t * pHandle)
{

    RampMngr_ExecRamp(&pHandle->SpeedRampMngr, (int32_t)SPD_GetAvrgMecSpeedUnit(pHandle->pSPD) * (int32_t)65536, 0);
}

/*
    Set torque ramp slope values, for ramping up and ramping down.
*/
void SpdTorqCtrl_SetTorqueRampSlope(SpeednTorqCtrlHandle_t * pHandle, uint32_t wSlopePerSecondUp, uint32_t wSlopePerSecondDown)
{

    pHandle->wTorqueSlopePerSecondUp = wSlopePerSecondUp;
    pHandle->wTorqueSlopePerSecondDown = wSlopePerSecondDown;
}


/*
    Set speed ramp slope values, for ramping up and ramping down.
*/
void SpdTorqCtrl_SetSpeedRampSlope(SpeednTorqCtrlHandle_t * pHandle, uint32_t wSlopePerSecondUp, uint32_t wSlopePerSecondDown)
{

    pHandle->wSpeedSlopePerSecondUp = wSlopePerSecondUp;
    pHandle->wSpeedSlopePerSecondDown = wSlopePerSecondDown;
}

/*
    Apply all torque foldbacks and returns limited torque.
*/
static int16_t SpdTorqCtrl_ApplyTorqueFoldback(SpeednTorqCtrlHandle_t * pHandle, int16_t hInputTorque)
{
    int16_t hMeasuredSpeed = 0;
    int16_t hMeasuredMotorTemp = 0;
    int16_t hMeasuredHeatsinkTemp = 0;

    hMeasuredSpeed = SPD_GetAvrgMecSpeedUnit(pHandle->pSPD);
    if (pHandle->pMotorTempSensor != NULL)
    {
        hMeasuredMotorTemp = NTC_GetAvTemp_C(pHandle->pMotorTempSensor);
    }
    if (pHandle->pHeatsinkTempSensor != NULL)
    {
        hMeasuredHeatsinkTemp = NTC_GetAvTemp_C(pHandle->pHeatsinkTempSensor);
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

/*
    Apply motor power limitation to torque reference
*/
static int16_t SpdTorqCtrl_ApplyPowerLimitation(SpeednTorqCtrlHandle_t * pHandle, int16_t hInputTorque)
{
    int32_t wTorqueLimit = 0;
    int16_t hMeasuredSpeedTenthRadPerSec = 0;
    int16_t hMeasuredSpeedUnit = 0;
    int16_t hRetval = hInputTorque;

    hMeasuredSpeedUnit = SPD_GetAvrgMecSpeedUnit(pHandle->pSPD);
		hMeasuredSpeedUnit = SignalFiltering_CalcOutput(&pHandle->SpeedFilter,hMeasuredSpeedUnit);  // filter out the speed
	
    hMeasuredSpeedTenthRadPerSec = (int16_t)((10*hMeasuredSpeedUnit*2*3.1416F)/SPEED_UNIT);

    if (hMeasuredSpeedUnit != 0)
    {
        if (hInputTorque > 0)
        {
            wTorqueLimit = 1000*pHandle->hMaxPositivePower/abs(hMeasuredSpeedTenthRadPerSec); // Torque limit in cNm. 1000 comes from 100*10
            if (hInputTorque > wTorqueLimit)
            {
                hRetval = (int16_t) wTorqueLimit;
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
