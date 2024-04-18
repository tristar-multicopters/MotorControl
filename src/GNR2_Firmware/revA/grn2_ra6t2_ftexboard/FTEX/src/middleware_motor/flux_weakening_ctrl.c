/**
  * @file    flux_weakening_ctrl.c
  * @brief   This file provides firmware functions that implement the Motor Control
  *          Control component of the Motor Control application.
  *
*/

/* Includes ------------------------------------------------------------------*/
#include "flux_weakening_ctrl.h"
#include "mc_math.h"

#include "mc_type.h"
#include "pid_regulator.h"
#define MARGIN_COEF     9  // voltage margin coefficient for flux weakening

void MotorControl_Init(MCConfigHandle_t * pHandle, PIDHandle_t * pPIDSpeed, PIDHandle_t * pPIDMotorControlHandle, MotorParameters_t MotorParameters)
{
    pHandle->hFwVoltRef = pHandle->hDefaultFwVoltRef;

    pHandle->pMotorControlPID = pPIDMotorControlHandle;

    pHandle->pSpeedPID = pPIDSpeed;

    pHandle->fRS = MotorParameters.ConfigParameters.fRS;
    
    pHandle->hNominalCurr = MotorParameters.ParametersConversion.hNominalPeakCurrent;
    pHandle->wNominalSqCurr = MotorParameters.ParametersConversion.hNominalPeakCurrent * MotorParameters.ParametersConversion.hNominalPeakCurrent;
    pHandle->wUsrMaxCurr = MotorParameters.ParametersConversion.hNominalPeakCurrent;
    
    pHandle->bWheelSpdSensorNbrPerRotation = MotorParameters.WheelSpeedSensorParameters.bWheelSpeedSensorNbrPerRotation;
}


void FluxWkng_Clear(MCConfigHandle_t * pHandle)
{
  qd_t V_null = {(int16_t)0, (int16_t)0};

  PID_SetIntegralTerm(pHandle->pMotorControlPID, (int32_t)0);
  pHandle->AvVoltQd = V_null;
  pHandle->AvVoltAmpl = (int16_t)0;
  pHandle->hIdRefOffset = (int16_t)0;
}


qd_t FluxWkng_CalcCurrRef(MCConfigHandle_t * pHandle, qd_t Iqdref)
{
  int32_t wIdRef, wIqSatSq, wIqSat, wAux1, wAux2;
  uint32_t wVoltLimit_Ref;
  int16_t hId_fw;

  /* Computation of the Id contribution coming from flux weakening algorithm */
  wVoltLimit_Ref = ((uint32_t)(pHandle->hFwVoltRef) * pHandle->hMaxModule * MARGIN_COEF)
                   / 10000u;
  wAux1 = (int32_t)(pHandle->AvVoltQd.q) *
          pHandle->AvVoltQd.q;
  wAux2 = (int32_t)(pHandle->AvVoltQd.d) *
          pHandle->AvVoltQd.d;
  wAux1 += wAux2;

  wAux1 = MCMath_Sqrt(wAux1);
  pHandle->AvVoltAmpl = (int16_t)wAux1;

  /* Just in case sqrt rounding exceeded INT16_MAX */
  if (wAux1 > INT16_MAX)
  {
    wAux1 = (int32_t)INT16_MAX;
  }

  hId_fw = PI_Controller(pHandle->pMotorControlPID, (int32_t)wVoltLimit_Ref - wAux1);

  /* If the Id coming from flux weakening algorithm (Id_fw) is positive, keep
  unchanged Idref, otherwise sum it to last Idref available when Id_fw was
  zero */
  if (hId_fw >= (int16_t)0)
  {
    pHandle->hIdRefOffset = Iqdref.d;
    wIdRef = (int32_t)Iqdref.d;
  }
  else
  {
    wIdRef = (int32_t)pHandle->hIdRefOffset + hId_fw;
  }

  /* Saturate new Idref to prevent the rotor from being demagnetized */
  if (wIdRef < pHandle->hDemagCurrent)
  {
    wIdRef =  pHandle->hDemagCurrent;
  }

  Iqdref.d = -(int16_t)wIdRef;

  /* New saturation for Iqref */
  wIqSatSq =  pHandle->wNominalSqCurr - wIdRef * wIdRef;
  wIqSat = MCMath_Sqrt(wIqSatSq);

  /* Iqref saturation value used for updating integral term limitations of
  speed PI */
  wAux1 = wIqSat * (int32_t)PID_GetKIDivisor(pHandle->pSpeedPID);

  PID_SetLowerIntegralTermLimit(pHandle->pSpeedPID, -wAux1);
  PID_SetUpperIntegralTermLimit(pHandle->pSpeedPID, wAux1);

  /* Iqref saturation value used for updating integral term limitations of
  speed PI */
  if (Iqdref.q > wIqSat)
  {
    Iqdref.q = (int16_t)wIqSat;
  }
  else if (Iqdref.q < -wIqSat)
  {
    Iqdref.q = -(int16_t)wIqSat;
  }
  else
  {
  }

  return (Iqdref);
}


void MC_DataProcess(MCConfigHandle_t * pHandle, qd_t Vqd)
{
  int32_t wAux;
  int32_t lowPassFilterBW = (int32_t)(pHandle->hVqdLowPassFilterBw) - (int32_t)1 ;
  
#ifdef FULL_MISRA_C_COMPLIANCY
  wAux = (int32_t)(pHandle->AvVoltQd.q) * lowPassFilterBW;
  wAux += Vqd.q;

  pHandle->AvVoltQd.q = (int16_t)(wAux /
                                     (int32_t)(pHandle->hVqdLowPassFilterBw));

  wAux = (int32_t)(pHandle->AvVoltQd.d) * lowPassFilterBW;
  wAux += Vqd.d;

  pHandle->AvVoltQd.d = (int16_t)(wAux /
                                     (int32_t)pHandle->hVqdLowPassFilterBw);
#else
  wAux = (int32_t)(pHandle->AvVoltQd.q) * lowPassFilterBW;
  wAux += Vqd.q;

  pHandle->AvVoltQd.q = (int16_t)(wAux >>
                                      pHandle->hVqdLowPassFilterBwLog);
  
  wAux = (int32_t)(pHandle->AvVoltQd.d) * lowPassFilterBW;
  wAux += Vqd.d;
  pHandle->AvVoltQd.d = (int16_t)(wAux >>
                                      pHandle->hVqdLowPassFilterBwLog);
  
#endif
  return;
}


void MC_SetVref(MCConfigHandle_t * pHandle, uint16_t hNewVref)
{
  pHandle->hFwVoltRef = hNewVref;
}

/**
  * @brief  It returns the present value of target voltage used by flux
  *         weakening algorihtm.
  * @param  pHandle Motor Control init strutcture.
  * @retval int16_t Present target voltage value expressed in tenth of
  *         percentage points of available voltage.
  */
uint16_t MC_GetVref(MCConfigHandle_t * pHandle)
{
  return (pHandle->hFwVoltRef);
}

/**
  * @brief  It returns the present value of voltage actually used by flux
  *         weakening algorihtm.
  * @param  pHandle Motor Control init strutcture.
  * @retval int16_t Present averaged phase stator voltage value, expressed
  *         in s16V (0-to-peak), where
  *         PhaseVoltage(V) = [PhaseVoltage(s16A) * Vbus(V)] /[sqrt(3) *32767].
  */
int16_t MC_GetAvVAmplitude(MCConfigHandle_t * pHandle)
{
  return (pHandle->AvVoltAmpl);
}

/**
  * @brief  It returns the measure of present voltage actually used by flux
  *         weakening algorihtm as percentage of available voltage.
  * @param  pHandle Motor Control init strutcture.
  * @retval uint16_t Present averaged phase stator voltage value, expressed in
  *         tenth of percentage points of available voltage.
  */
uint16_t MC_GetAvVPercentage(MCConfigHandle_t * pHandle)
{
  return (uint16_t)((uint32_t)(pHandle->AvVoltAmpl) * 1000u /
                       (uint32_t)(pHandle->hMaxModule));
}

