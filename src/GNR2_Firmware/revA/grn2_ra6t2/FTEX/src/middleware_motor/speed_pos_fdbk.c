/**
  * @file    speed_pos_fdbk.c
  * @brief   This file provides firmware functions that implement the  features
  *          of the Speed & Position Feedback component of the Motor Control application.
  *
*/

/* Includes ------------------------------------------------------------------*/
#include "speed_pos_fdbk.h"


int16_t SpdPosFdbk_GetElAngle( SpeednPosFdbkHandle_t * pHandle )
{
  return ( pHandle->hElAngle );
}


int32_t SpdPosFdbk_GetMecAngle( SpeednPosFdbkHandle_t * pHandle )
{
  return ( pHandle->wMecAngle );
}


int16_t SpdPosFdbk_GetAvrgMecSpeedUnit( SpeednPosFdbkHandle_t * pHandle )
{
  return ( pHandle->hAvrMecSpeedUnit );
}


int16_t SpdPosFdbk_GetElSpeedDpp( SpeednPosFdbkHandle_t * pHandle )
{
  return ( pHandle->hElSpeedDpp );
}


int16_t SpdPosFdbk_GetInstElSpeedDpp( SpeednPosFdbkHandle_t * pHandle )
{
  return ( pHandle->InstantaneousElSpeedDpp );
}


bool SpdPosFdbk_GetReliability( SpeednPosFdbkHandle_t * pHandle )
{
  bool SpeedSensorReliability = true;
  if ( pHandle->bSpeedErrorNumber ==
       pHandle->bMaximumSpeedErrorsNumber )
  {
    SpeedSensorReliability = false;
  }
  return ( SpeedSensorReliability );
}


bool SpdPosFdbk_CalcReliability( SpeednPosFdbkHandle_t * pHandle, int16_t * pMecSpeedUnit )
{
  bool SpeedSensorReliability = true;
  uint8_t bSpeedErrorNumber;
  uint8_t bMaximumSpeedErrorsNumber = pHandle->bMaximumSpeedErrorsNumber;

  bool SpeedError = false;
  uint16_t hAbsMecSpeedUnit, hAbsMecAccelUnitP;
  int16_t hAux;

  bSpeedErrorNumber = pHandle->bSpeedErrorNumber;

  /* Compute absoulte value of mechanical speed */
  if ( *pMecSpeedUnit < 0 )
  {
    hAux = -( *pMecSpeedUnit );
    hAbsMecSpeedUnit = ( uint16_t )( hAux );
  }
  else
  {
    hAbsMecSpeedUnit = ( uint16_t )( *pMecSpeedUnit );
  }

  if ( hAbsMecSpeedUnit > pHandle->hMaxReliableMecSpeedUnit )
  {
    SpeedError = true;
  }

  if ( hAbsMecSpeedUnit < pHandle->hMinReliableMecSpeedUnit )
  {
    SpeedError = true;
  }

  /* Compute absoulte value of mechanical acceleration */
  if ( pHandle->hMecAccelUnitP < 0 )
  {
    hAux = -( pHandle->hMecAccelUnitP );
    hAbsMecAccelUnitP = ( uint16_t )( hAux );
  }
  else
  {
    hAbsMecAccelUnitP = ( uint16_t )( pHandle->hMecAccelUnitP );
  }

  if ( hAbsMecAccelUnitP > pHandle->hMaxReliableMecAccelUnitP )
  {
    SpeedError = true;
  }

  if ( SpeedError == true )
  {
    if ( bSpeedErrorNumber < bMaximumSpeedErrorsNumber )
    {
      bSpeedErrorNumber++;
    }
  }
  else
  {
    if ( bSpeedErrorNumber < bMaximumSpeedErrorsNumber )
    {
      bSpeedErrorNumber = 0u;
    }
  }

  if ( bSpeedErrorNumber == bMaximumSpeedErrorsNumber )
  {
    SpeedSensorReliability = false;
  }

  pHandle->bSpeedErrorNumber = bSpeedErrorNumber;

  return ( SpeedSensorReliability );
}


int16_t SpdPosFdbk_GetS16Speed( SpeednPosFdbkHandle_t * pHandle )
{
  int32_t wAux = ( int32_t ) pHandle->hAvrMecSpeedUnit;
  wAux *= INT16_MAX;
  wAux /= ( int16_t ) pHandle->hMaxReliableMecSpeedUnit;
  return ( int16_t )wAux;
}


uint8_t SpdPosFdbk_GetElToMecRatio( SpeednPosFdbkHandle_t * pHandle )
{
  return ( pHandle->bElToMecRatio );
}


void SpdPosFdbk_SetElToMecRatio( SpeednPosFdbkHandle_t * pHandle, uint8_t bPP )
{
  pHandle->bElToMecRatio = bPP;
}



