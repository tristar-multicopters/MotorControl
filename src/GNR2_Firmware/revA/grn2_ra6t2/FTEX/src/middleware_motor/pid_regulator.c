/**
 * @file    pid_regulator.c
 * @brief   This file provides firmware functions that implement the following features
 *          of the PID regulator component of the Motor Control application:
 *
 *           * proportional, integral and derivative computation funcions
 *           * read and write gain functions
 *
*/

/* Includes ------------------------------------------------------------------*/
#include "pid_regulator.h"

#include "mc_type.h"


void PID_HandleInit( PIDHandle_t * pHandle )
{
  pHandle->hKpGain =  pHandle->hDefKpGain;
  pHandle->hKiGain =  pHandle->hDefKiGain;
  pHandle->hKdGain =  pHandle->hDefKdGain;
  pHandle->wIntegralTerm = 0x00000000UL;
  pHandle->wPrevProcessVarError = 0x00000000UL;
}


void PID_SetKP( PIDHandle_t * pHandle, int16_t hKpGain )
{
  pHandle->hKpGain = hKpGain;
}


void PID_SetKI( PIDHandle_t * pHandle, int16_t hKiGain )
{
  pHandle->hKiGain = hKiGain;
}


int16_t PID_GetKP( PIDHandle_t * pHandle )
{
  return ( pHandle->hKpGain );
}


int16_t PID_GetKI( PIDHandle_t * pHandle )
{
  return ( pHandle->hKiGain );
}


int16_t PID_GetDefaultKP( PIDHandle_t * pHandle )
{
  return ( pHandle->hDefKpGain );
}


int16_t PID_GetDefaultKI( PIDHandle_t * pHandle )
{
  return ( pHandle->hDefKiGain );
}


void PID_SetIntegralTerm( PIDHandle_t * pHandle, int32_t wIntegralTermValue )
{
  pHandle->wIntegralTerm = wIntegralTermValue;

  return;
}


uint16_t PID_GetKPDivisor( PIDHandle_t * pHandle )
{
  return ( pHandle->hKpDivisor );
}


void PID_SetKPDivisorPOW2( PIDHandle_t * pHandle, uint16_t hKpDivisorPOW2 )
{
  pHandle->hKpDivisorPOW2 = hKpDivisorPOW2;
  pHandle->hKpDivisor = ( ( uint16_t )( 1u ) << hKpDivisorPOW2 );
}


uint16_t PID_GetKIDivisor( PIDHandle_t * pHandle )
{
  return ( pHandle->hKiDivisor );
}


void PID_SetKIDivisorPOW2( PIDHandle_t * pHandle, uint16_t hKiDivisorPOW2 )
{
  int32_t wKiDiv = ( ( int32_t )( 1u ) << hKiDivisorPOW2 );
  pHandle->hKiDivisorPOW2 = hKiDivisorPOW2;
  pHandle->hKiDivisor = ( uint16_t )( wKiDiv );
  PID_SetUpperIntegralTermLimit( pHandle, ( int32_t )INT16_MAX * wKiDiv );
  PID_SetLowerIntegralTermLimit( pHandle, ( int32_t ) - INT16_MAX * wKiDiv );
}


void PID_SetLowerIntegralTermLimit( PIDHandle_t * pHandle, int32_t wLowerLimit )
{
  pHandle->wLowerIntegralLimit = wLowerLimit;
}


void PID_SetUpperIntegralTermLimit( PIDHandle_t * pHandle, int32_t wUpperLimit )
{
  pHandle->wUpperIntegralLimit = wUpperLimit;
}


void PID_SetLowerOutputLimit( PIDHandle_t * pHandle, int16_t hLowerLimit )
{
  pHandle->hLowerOutputLimit = hLowerLimit;
}


void PID_SetUpperOutputLimit( PIDHandle_t * pHandle, int16_t hUpperLimit )
{
  pHandle->hUpperOutputLimit = hUpperLimit;
}


void PID_SetPrevError( PIDHandle_t * pHandle, int32_t wPrevProcessVarError )
{
  pHandle->wPrevProcessVarError = wPrevProcessVarError;
  return;
}


void PID_SetKD( PIDHandle_t * pHandle, int16_t hKdGain )
{
  pHandle->hKdGain = hKdGain;
}


 int16_t PID_GetKD( PIDHandle_t * pHandle )
{
  return pHandle->hKdGain;
}


uint16_t PID_GetKDDivisor( PIDHandle_t * pHandle )
{
  return ( pHandle->hKdDivisor );
}


 void PID_SetKDDivisorPOW2( PIDHandle_t * pHandle, uint16_t hKdDivisorPOW2 )
{
  pHandle->hKdDivisorPOW2 = hKdDivisorPOW2;
  pHandle->hKdDivisor = ( ( uint16_t )( 1u ) << hKdDivisorPOW2 );
}


int16_t PI_Controller( PIDHandle_t * pHandle, int32_t wProcessVarError )
{
  int32_t wProportional_Term, wIntegral_Term, wOutput_32, wIntegral_sum_temp;
  int32_t wDischarge = 0;
  int16_t hUpperOutputLimit = pHandle->hUpperOutputLimit;
  int16_t hLowerOutputLimit = pHandle->hLowerOutputLimit;

  /* Proportional term computation*/
  wProportional_Term = pHandle->hKpGain * wProcessVarError;

  /* Integral term computation */
  if ( pHandle->hKiGain == 0 )
  {
    pHandle->wIntegralTerm = 0;
  }
  else
  {
    wIntegral_Term = pHandle->hKiGain * wProcessVarError;
    wIntegral_sum_temp = pHandle->wIntegralTerm + wIntegral_Term;

    if ( wIntegral_sum_temp < 0 )
    {
      if ( pHandle->wIntegralTerm > 0 )
      {
        if ( wIntegral_Term > 0 )
        {
          wIntegral_sum_temp = INT32_MAX;
        }
      }
    }
    else
    {
      if ( pHandle->wIntegralTerm < 0 )
      {
        if ( wIntegral_Term < 0 )
        {
          wIntegral_sum_temp = -INT32_MAX;
        }
      }
    }

    if ( wIntegral_sum_temp > pHandle->wUpperIntegralLimit )
    {
      pHandle->wIntegralTerm = pHandle->wUpperIntegralLimit;
    }
    else if ( wIntegral_sum_temp < pHandle->wLowerIntegralLimit )
    {
      pHandle->wIntegralTerm = pHandle->wLowerIntegralLimit;
    }
    else
    {
      pHandle->wIntegralTerm = wIntegral_sum_temp;
    }
  }

#ifdef FULL_MISRA_C_COMPLIANCY
  wOutput_32 = ( wProportional_Term / ( int32_t )pHandle->hKpDivisor ) + ( pHandle->wIntegralTerm /
               ( int32_t )pHandle->hKiDivisor );
#else
  /* WARNING: the below instruction is not MISRA compliant, user should verify
             that Cortex-M3 assembly instruction ASR (arithmetic shift right)
             is used by the compiler to perform the shifts (instead of LSR
             logical shift right)*/
  wOutput_32 = ( wProportional_Term >> pHandle->hKpDivisorPOW2 ) + ( pHandle->wIntegralTerm >> pHandle->hKiDivisorPOW2 );
#endif

  if ( wOutput_32 > hUpperOutputLimit )
  {

    wDischarge = hUpperOutputLimit - wOutput_32;
    wOutput_32 = hUpperOutputLimit;
  }
  else if ( wOutput_32 < hLowerOutputLimit )
  {

    wDischarge = hLowerOutputLimit - wOutput_32;
    wOutput_32 = hLowerOutputLimit;
  }
  else { /* Nothing to do here */ }

  pHandle->wIntegralTerm += wDischarge;

  return ( ( int16_t )( wOutput_32 ) );
}


int16_t PID_Controller( PIDHandle_t * pHandle, int32_t wProcessVarError )
{
  int32_t wDifferential_Term;
  int32_t wDeltaError;
  int32_t wTemp_output;

  if ( pHandle->hKdGain != 0 ) /* derivative terms not used */
  {
    wDeltaError = wProcessVarError - pHandle->wPrevProcessVarError;
    wDifferential_Term = pHandle->hKdGain * wDeltaError;

#ifdef FULL_MISRA_C_COMPLIANCY
    wDifferential_Term /= ( int32_t )pHandle->hKdDivisor;
#else
    /* WARNING: the below instruction is not MISRA compliant, user should verify
    that Cortex-M3 assembly instruction ASR (arithmetic shift right)
    is used by the compiler to perform the shifts (instead of LSR
    logical shift right)*/
    wDifferential_Term >>= pHandle->hKdDivisorPOW2;
#endif

    pHandle->wPrevProcessVarError = wProcessVarError;

    wTemp_output = PI_Controller( pHandle, wProcessVarError ) + wDifferential_Term;

    if ( wTemp_output > pHandle->hUpperOutputLimit )
    {
      wTemp_output = pHandle->hUpperOutputLimit;
    }
    else if ( wTemp_output < pHandle->hLowerOutputLimit )
    {
      wTemp_output = pHandle->hLowerOutputLimit;
    }
    else
    {}
  }
  else
  {
    wTemp_output = PI_Controller( pHandle, wProcessVarError );
  }
  return ( ( int16_t ) wTemp_output );
}


