/**
  * @file    virtual_speed_sensor.c
  * @brief   This file provides firmware functions that implement the features
  *          of the Virtual Speed Sensor component of the Motor Control application.
  *
*/

/* Includes ------------------------------------------------------------------*/
#include "virtual_speed_sensor.h"


void VirtualSpdSensor_Init(VirtualSpeedSensor_Handle_t * pHandle)
{
  VirtualSpdSensor_Clear(pHandle);
}


void VirtualSpdSensor_Clear(VirtualSpeedSensor_Handle_t * pHandle)
{

  pHandle->Super.bSpeedErrorNumber = 0u;
  pHandle->Super.hElAngle = 0;
  pHandle->Super.hMecAngle = 0;
  pHandle->Super.hAvrMecSpeedUnit = 0;
  pHandle->Super.hElSpeedDpp = 0;
  pHandle->Super.hMecAccelUnitP = 0;
  pHandle->Super.bSpeedErrorNumber = 0u;

  pHandle->wElAccDppP32 = 0;
  pHandle->wElSpeedDpp32 = 0;
  pHandle->hRemainingStep = 0u;
  pHandle->hElAngleAccu = 0;

  pHandle->bTransitionStarted = false;
  pHandle->bTransitionEnded = false;
  pHandle->hTransitionRemainingSteps = pHandle->hTransitionSteps;
  pHandle->bTransitionLocked = false;

  pHandle->bCopyObserver = false;
}


int16_t VirtualSpdSensor_CalcElAngle(VirtualSpeedSensor_Handle_t * pHandle, void * pInputVars_str)
{

  int16_t hRetAngle;
  int16_t hAngleDiff;
  int16_t hAngleCorr;
  int32_t wAux;
  int16_t hSignCorr = 1;

  if (pHandle->bCopyObserver == true)
  {
    hRetAngle = *(int16_t *)pInputVars_str;
  }
  else
  {
    pHandle->hElAngleAccu += pHandle->Super.hElSpeedDpp;

    pHandle->Super.hMecAngle += pHandle->Super.hElSpeedDpp /
                                 (int16_t)pHandle->Super.bElToMecRatio;

    if (pHandle->bTransitionStarted == true)
    {
      if (pHandle->hTransitionRemainingSteps == 0)
      {
        hRetAngle = *(int16_t *)pInputVars_str;
        pHandle->bTransitionEnded = true;
        pHandle->Super.bSpeedErrorNumber = 0u;
      }
      else
      {
        pHandle->hTransitionRemainingSteps--;

        if (pHandle->Super.hElSpeedDpp >= 0)
        {
          hAngleDiff = *(int16_t *)pInputVars_str - pHandle->hElAngleAccu;
        }
        else
        {
          hAngleDiff = pHandle->hElAngleAccu - *(int16_t *)pInputVars_str;
          hSignCorr = -1;
        }

        wAux = (int32_t)hAngleDiff * pHandle->hTransitionRemainingSteps;


        hAngleCorr = (int16_t)(wAux / pHandle->hTransitionSteps);


        hAngleCorr *= hSignCorr;

        if (hAngleDiff >= 0)
        {
          pHandle->bTransitionLocked = true;
          hRetAngle = *(int16_t *)pInputVars_str - hAngleCorr;
        }
        else
        {
          if (pHandle->bTransitionLocked == false)
          {
            hRetAngle = pHandle->hElAngleAccu;
          }
          else
          {
            hRetAngle = *(int16_t *)pInputVars_str + hAngleCorr;
          }
        }
      }
    }
    else
    {
      hRetAngle = pHandle->hElAngleAccu;
    }
  }

  pHandle->Super.hElAngle = hRetAngle;
  return hRetAngle;
}


bool VirtualSpdSensor_CalcAvrgMecSpeedUnit(VirtualSpeedSensor_Handle_t * pHandle, int16_t * pMecSpeedUnit)
{
  bool SpeedSensorReliability = false;

  if (pHandle->hRemainingStep > 1u)
  {
    pHandle->wElSpeedDpp32 += pHandle->wElAccDppP32;
    pHandle->Super.hElSpeedDpp = (int16_t)(pHandle->wElSpeedDpp32 / 65536);

    /* Convert dpp into MecUnit */
    *pMecSpeedUnit = (int16_t)(((int32_t)pHandle->Super.hElSpeedDpp *
                                    (int32_t)pHandle->Super.hMeasurementFrequency * SPEED_UNIT) /
                                  ((int32_t)pHandle->Super.DPPConvFactor * (int32_t)pHandle->Super.bElToMecRatio));

    pHandle->Super.hAvrMecSpeedUnit = *pMecSpeedUnit;

    pHandle->hRemainingStep--;
  }
  else if (pHandle->hRemainingStep == 1u)
  {
    *pMecSpeedUnit = pHandle->hFinalMecSpeedUnit;

    pHandle->Super.hAvrMecSpeedUnit = *pMecSpeedUnit;

    pHandle->Super.hElSpeedDpp = (int16_t)(((int32_t)(*pMecSpeedUnit) *
                                  (int32_t) (pHandle->Super.DPPConvFactor)) /
                                  ((int32_t)SPEED_UNIT * (int32_t)pHandle->Super.hMeasurementFrequency));

    pHandle->Super.hElSpeedDpp *= (int16_t)(pHandle->Super.bElToMecRatio);

    pHandle->hRemainingStep = 0u;
  }
  else
  {
    *pMecSpeedUnit = pHandle->Super.hAvrMecSpeedUnit;
  }
  /* If the transition is not done yet, we already know that speed is not reliable */
  if (pHandle->bTransitionEnded == false)
  {
    pHandle->Super.bSpeedErrorNumber = pHandle->Super.bMaximumSpeedErrorsNumber;
    SpeedSensorReliability = false;
  }
  else
  {
    SpeedSensorReliability = SpdPosFdbk_CalcReliability (&pHandle->Super, pMecSpeedUnit);
  }

  return (SpeedSensorReliability);
}


void VirtualSpdSensor_SetMecAngle(VirtualSpeedSensor_Handle_t * pHandle, int16_t hMecAngle)
{

  pHandle->hElAngleAccu = hMecAngle;
  pHandle->Super.hMecAngle = pHandle->hElAngleAccu / (int16_t)pHandle->Super.bElToMecRatio;
  pHandle->Super.hElAngle = hMecAngle;
}


void  VirtualSpdSensor_SetMecAcceleration(VirtualSpeedSensor_Handle_t * pHandle, int16_t  hFinalMecSpeedUnit,
                              uint16_t hDurationms)
{

  uint16_t hNbrStep;
  int16_t hCurrentMecSpeedDpp;
  int32_t wMecAccDppP32;
  int16_t hFinalMecSpeedDpp;

  if (pHandle->bTransitionStarted == false)
  {
    if (hDurationms == 0u)
    {
      pHandle->Super.hAvrMecSpeedUnit = hFinalMecSpeedUnit;

      pHandle->Super.hElSpeedDpp = (int16_t)(((int32_t)(hFinalMecSpeedUnit) *
                                    (int32_t)(pHandle->Super.DPPConvFactor)) /
                                    ((int32_t)SPEED_UNIT * (int32_t)pHandle->Super.hMeasurementFrequency));

      pHandle->Super.hElSpeedDpp *= (int16_t)(pHandle->Super.bElToMecRatio);

      pHandle->hRemainingStep = 0u;

      pHandle->hFinalMecSpeedUnit = hFinalMecSpeedUnit;
    }
    else
    {
      hNbrStep = (uint16_t)(((uint32_t)hDurationms *
                                 (uint32_t)pHandle->hSpeedSamplingFreqHz) /
                               1000u);

      hNbrStep++;

      pHandle->hRemainingStep = hNbrStep;

      hCurrentMecSpeedDpp = pHandle->Super.hElSpeedDpp /
                            (int16_t)pHandle->Super.bElToMecRatio;

      hFinalMecSpeedDpp = (int16_t)(((int32_t)hFinalMecSpeedUnit * (int32_t)(pHandle->Super.DPPConvFactor)) /
                                       ((int32_t)SPEED_UNIT * (int32_t)pHandle->Super.hMeasurementFrequency));

      wMecAccDppP32 = (((int32_t)hFinalMecSpeedDpp - (int32_t)hCurrentMecSpeedDpp) *
                        (int32_t)65536) / (int32_t)hNbrStep;

      pHandle->wElAccDppP32 = wMecAccDppP32 * (int16_t)pHandle->Super.bElToMecRatio;

      pHandle->hFinalMecSpeedUnit = hFinalMecSpeedUnit;

      pHandle->wElSpeedDpp32 = (int32_t)pHandle->Super.hElSpeedDpp * (int32_t)65536;
    }
  }
}


bool VirtualSpdSensor_IsRampCompleted(VirtualSpeedSensor_Handle_t * pHandle)
{
  bool retVal = false;
  if (pHandle->hRemainingStep == 0u)
  {
    retVal = true;
  }
  return retVal;
}


int16_t  VirtualSpdSensor_GetLastRampFinalSpeed(VirtualSpeedSensor_Handle_t * pHandle)
{
  return pHandle->hFinalMecSpeedUnit;
}


bool VirtualSpdSensor_SetStartTransition(VirtualSpeedSensor_Handle_t * pHandle, bool bCommand)
{
  bool bAux = true;
  if (bCommand == true)
  {
    pHandle->bTransitionStarted = true;

    if (pHandle->hTransitionSteps == 0)
    {
      pHandle->bTransitionEnded = true;
      pHandle->Super.bSpeedErrorNumber = 0u;
      bAux = false;
    }
  }
  return bAux;
}


bool VirtualSpdSensor_IsTransitionOngoing(VirtualSpeedSensor_Handle_t * pHandle)
{
  uint16_t hTS = 0u, hTE = 0u, hAux;
  bool retVal = false;
  if (pHandle->bTransitionStarted == true)
  {
    hTS = 1u;
  }
  if (pHandle->bTransitionEnded == true)
  {
    hTE = 1u;
  }
  hAux = hTS ^ hTE;
  if (hAux != 0u)
  {
    retVal = true;
  }
  return (retVal);
}

bool VirtualSpdSensor_TransitionEnded(VirtualSpeedSensor_Handle_t * pHandle)
{
    return pHandle->bTransitionEnded;
}


void VirtualSpdSensor_SetCopyObserver(VirtualSpeedSensor_Handle_t * pHandle)
{
  pHandle->bCopyObserver = true;
}


void VirtualSpdSensor_SetElAngle(VirtualSpeedSensor_Handle_t * pHandle, int16_t hElAngle)
{
  pHandle->hElAngleAccu = hElAngle;
  pHandle->Super.hElAngle = hElAngle;
}


