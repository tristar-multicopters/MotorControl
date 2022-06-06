/**
  * @file    speed_torq_ctrl.c
  * @brief   This file provides firmware functions that implement the following features
  *          of the Speed & Torque Control component of the Motor Control application.
  *
	*/

/* Includes ------------------------------------------------------------------*/
#include "speed_torq_ctrl.h"
#include "speed_pos_fdbk.h"

#include "mc_type.h"

#define CHECK_BOUNDARY


void SpdTorqCtrl_Init( SpeednTorqCtrlHandle_t * pHandle, PIDHandle_t * pPI, SpeednPosFdbkHandle_t * SPD_Handle )
{

  pHandle->pPISpeed = pPI;
  pHandle->pSPD = SPD_Handle;
  pHandle->Mode = pHandle->ModeDefault;
  pHandle->wSpeedRefUnitExt = ( int32_t )pHandle->hMecSpeedRefUnitDefault * 65536;
  pHandle->wTorqueRef = ( int32_t )pHandle->hTorqueRefDefault * 65536;
  pHandle->hTargetFinal = 0;
  pHandle->wRampRemainingStep = 0u;
  pHandle->wIncDecAmount = 0;
}


void SpdTorqCtrl_SetSpeedSensor( SpeednTorqCtrlHandle_t * pHandle, SpeednPosFdbkHandle_t * SPD_Handle )
{
  pHandle->pSPD = SPD_Handle;
}


SpeednPosFdbkHandle_t * SpdTorqCtrl_GetSpeedSensor( SpeednTorqCtrlHandle_t * pHandle )
{
  return ( pHandle->pSPD );
}


void SpdTorqCtrl_Clear( SpeednTorqCtrlHandle_t * pHandle )
{
  if ( pHandle->Mode == STC_SPEED_MODE )
  {
    PID_SetIntegralTerm( pHandle->pPISpeed, 0 );
  }
}


int16_t SpdTorqCtrl_GetMecSpeedRefUnit( SpeednTorqCtrlHandle_t * pHandle )
{
  return ( ( int16_t )( pHandle->wSpeedRefUnitExt / 65536 ) );
}


int16_t SpdTorqCtrl_GetTorqueRef( SpeednTorqCtrlHandle_t * pHandle )
{
  return ( ( int16_t )( pHandle->wTorqueRef / 65536 ) );
}


void SpdTorqCtrl_SetControlMode( SpeednTorqCtrlHandle_t * pHandle, STCModality_t bMode )
{
  pHandle->Mode = bMode;
  pHandle->wRampRemainingStep = 0u; /* Interrupts previous ramp. */
}


STCModality_t SpdTorqCtrl_GetControlMode( SpeednTorqCtrlHandle_t * pHandle )
{
  return pHandle->Mode;
}


bool SpdTorqCtrl_ExecRamp( SpeednTorqCtrlHandle_t * pHandle, int16_t hTargetFinal, uint32_t hDurationms )
{
  bool AllowedRange = true;
  uint32_t wAux;
  int32_t wAux1;
  int16_t hCurrentReference;

  /* Check if the hTargetFinal is out of the bound of application. */
  if ( pHandle->Mode == STC_TORQUE_MODE )
  {
    hCurrentReference = SpdTorqCtrl_GetTorqueRef( pHandle );
#ifdef CHECK_BOUNDARY
    if ( ( int32_t )hTargetFinal > ( int32_t )pHandle->hMaxPositiveTorque )
    {
      AllowedRange = false;
    }
    if ( ( int32_t )hTargetFinal < ( int32_t )pHandle->hMinNegativeTorque )
    {
      AllowedRange = false;
    }
#endif
  }
  else
  {
    hCurrentReference = ( int16_t )( pHandle->wSpeedRefUnitExt / 65536 );

#ifdef CHECK_BOUNDARY
    if ( ( int32_t )hTargetFinal > ( int32_t )pHandle->hMaxAppPositiveMecSpeedUnit )
    {
      AllowedRange = false;
    }
    else if ( hTargetFinal < pHandle->hMinAppNegativeMecSpeedUnit )
    {
      AllowedRange = false;
    }
    else if ( ( int32_t )hTargetFinal < ( int32_t )pHandle->hMinAppPositiveMecSpeedUnit )
    {
      if ( hTargetFinal > pHandle->hMaxAppNegativeMecSpeedUnit )
      {
        AllowedRange = false;
      }
    }
    else {}
#endif
  }

  if ( AllowedRange == true )
  {
    /* Interrupts the execution of any previous ramp command */
    if ( hDurationms == 0u )
    {
      if ( pHandle->Mode == STC_SPEED_MODE )
      {
        pHandle->wSpeedRefUnitExt = ( int32_t )hTargetFinal * 65536;
      }
      else
      {
        pHandle->wTorqueRef = ( int32_t )hTargetFinal * 65536;
      }
      pHandle->wRampRemainingStep = 0u;
      pHandle->wIncDecAmount = 0;
    }
    else
    {
      /* Store the hTargetFinal to be applied in the last step */
      pHandle->hTargetFinal = hTargetFinal;

      /* Compute the (wRampRemainingStep) number of steps remaining to complete
      the ramp. */
      wAux = ( uint32_t )hDurationms * ( uint32_t )pHandle->hSTCFrequencyHz;
      wAux /= 1000u;
      pHandle->wRampRemainingStep = wAux;
      pHandle->wRampRemainingStep++;

      /* Compute the increment/decrement amount (wIncDecAmount) to be applied to
      the reference value at each CalcTorqueReference. */
      wAux1 = ( ( int32_t )hTargetFinal - ( int32_t )hCurrentReference ) * 65536;
      wAux1 /= ( int32_t )pHandle->wRampRemainingStep;
      pHandle->wIncDecAmount = wAux1;
    }
  }

  return AllowedRange;
}


void SpdTorqCtrl_StopRamp( SpeednTorqCtrlHandle_t * pHandle )
{

  pHandle->wRampRemainingStep = 0u;
  pHandle->wIncDecAmount = 0;
}


int16_t SpdTorqCtrl_CalcTorqueReference( SpeednTorqCtrlHandle_t * pHandle )
{
  int32_t wCurrentReference;
  int16_t hTorqueReference = 0;
  int16_t hMeasuredSpeed;
  int16_t hTargetSpeed;
  int16_t hError;

  if ( pHandle->Mode == STC_TORQUE_MODE )
  {
    wCurrentReference = pHandle->wTorqueRef;
  }
  else
  {
    wCurrentReference = pHandle->wSpeedRefUnitExt;
  }

  /* Update the speed reference or the torque reference according to the mode
     and terminates the ramp if needed. */
  if ( pHandle->wRampRemainingStep > 1u )
  {
    /* Increment/decrement the reference value. */
    wCurrentReference += pHandle->wIncDecAmount;

    /* Decrement the number of remaining steps */
    pHandle->wRampRemainingStep--;
  }
  else if ( pHandle->wRampRemainingStep == 1u )
  {
    /* Set the backup value of hTargetFinal. */
    wCurrentReference = ( int32_t )pHandle->hTargetFinal * 65536;
    pHandle->wRampRemainingStep = 0u;
  }
  else
  {
    /* Do nothing. */
  }

  if ( pHandle->Mode == STC_SPEED_MODE )
  {
    /* Run the speed control loop */

    /* Compute speed error */
    hTargetSpeed = ( int16_t )( wCurrentReference / 65536 );
    hMeasuredSpeed = SpdPosFdbk_GetAvrgMecSpeedUnit( pHandle->pSPD );
    hError = hTargetSpeed - hMeasuredSpeed;
    hTorqueReference = PI_Controller( pHandle->pPISpeed, ( int32_t )hError );

    pHandle->wSpeedRefUnitExt = wCurrentReference;
    pHandle->wTorqueRef = ( int32_t )hTorqueReference * 65536;
  }
  else
  {
    pHandle->wTorqueRef = wCurrentReference;
    hTorqueReference = ( int16_t )( wCurrentReference / 65536 );
  }

  return hTorqueReference;
}


int16_t SpdTorqCtrl_GetMecSpeedRefUnitDefault( SpeednTorqCtrlHandle_t * pHandle )
{
  return pHandle->hMecSpeedRefUnitDefault;
}


uint16_t SpdTorqCtrl_GetMaxAppPositiveMecSpeedUnit( SpeednTorqCtrlHandle_t * pHandle )
{
  return pHandle->hMaxAppPositiveMecSpeedUnit;
}


int16_t SpdTorqCtrl_GetMinAppNegativeMecSpeedUnit( SpeednTorqCtrlHandle_t * pHandle )
{
  return pHandle->hMinAppNegativeMecSpeedUnit;
}


bool SpdTorqCtrl_RampCompleted( SpeednTorqCtrlHandle_t * pHandle )
{
  bool retVal = false;
  if ( pHandle->wRampRemainingStep == 0u )
  {
    retVal = true;
  }
  return retVal;
}


bool SpdTorqCtrl_StopSpeedRamp( SpeednTorqCtrlHandle_t * pHandle )
{
  bool retVal = false;
  if ( pHandle->Mode == STC_SPEED_MODE )
  {
    pHandle->wRampRemainingStep = 0u;
    retVal = true;
  }
  return retVal;
}


qd_t SpdTorqCtrl_GetDefaultIqdref( SpeednTorqCtrlHandle_t * pHandle )
{
  qd_t IqdRefDefault;
  IqdRefDefault.q = pHandle->hTorqueRefDefault;
  IqdRefDefault.d = pHandle->hIdrefDefault;
  return IqdRefDefault;
}


void SpdTorqCtrl_SetNominalCurrent( SpeednTorqCtrlHandle_t * pHandle, uint16_t hNominalCurrent )
{
  pHandle->hMaxPositiveTorque = hNominalCurrent;
  pHandle->hMinNegativeTorque = -hNominalCurrent;
}


void SpdTorqCtrl_ForceSpeedReferenceToCurrentSpeed( SpeednTorqCtrlHandle_t * pHandle )
{
  pHandle->wSpeedRefUnitExt = ( int32_t )SpdPosFdbk_GetAvrgMecSpeedUnit( pHandle->pSPD ) * ( int32_t )65536;
}



