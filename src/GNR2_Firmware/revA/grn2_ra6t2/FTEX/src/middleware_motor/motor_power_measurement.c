/**
  * @file    motor_power_measurement.c
  * @brief   This file provides firmware functions that implement the features
  *          of the Motor Power Measurement component of the Motor Control application:
  *
  *           * Calculate power of the motor
  *           * Clear power measurement
  *           * Get Power of the motor
  *           * Get average Power of the motor
  *
*/


/* Includes ------------------------------------------------------------------*/
#include "motor_power_measurement.h"

#include "mc_type.h"


void MotorPowMeas_Clear( MotorPowerMeasHandle_t * pHandle )
{
  uint16_t i;
  for ( i = 0u; i < MPM_BUFFER_LENGHT; i++ )
  {
    pHandle->hMeasBuffer[i] = 0;
  }
  pHandle->hNextMeasBufferIndex = 0u;
  pHandle->hLastMeasBufferIndex = 0u;

}


int16_t MPM_CalcElMotorPower( MotorPowerMeasHandle_t * pHandle, int16_t CurrentMotorPower )
{
  uint16_t i;
  int32_t wAux = 0;

  /* Store the measured values in the buffer.*/
  pHandle->hMeasBuffer[pHandle->hNextMeasBufferIndex] = CurrentMotorPower;
  pHandle->hLastMeasBufferIndex = pHandle->hNextMeasBufferIndex;
  pHandle->hNextMeasBufferIndex++;
  if ( pHandle->hNextMeasBufferIndex >= MPM_BUFFER_LENGHT )
  {
    pHandle->hNextMeasBufferIndex = 0u;
  }
  /* Compute the average measured motor power */
  for ( i = 0u; i < MPM_BUFFER_LENGHT; i++ )
  {
    wAux += ( int32_t )( pHandle->hMeasBuffer[i] );
  }
  wAux /= ( int32_t )MPM_BUFFER_LENGHT;
  pHandle->hAvrgElMotorPowerW = ( int16_t )( wAux );
  /* Return the last measured motor power */
  return CurrentMotorPower;
}


int16_t MPM_GetElMotorPowerW( MotorPowerMeasHandle_t * pHandle )
{
  return ( pHandle->hMeasBuffer[pHandle->hLastMeasBufferIndex] );
}


int16_t MotorPowMeas_GetAvrgElMotorPowerW( MotorPowerMeasHandle_t * pHandle )
{
  return ( pHandle->hAvrgElMotorPowerW );
}


