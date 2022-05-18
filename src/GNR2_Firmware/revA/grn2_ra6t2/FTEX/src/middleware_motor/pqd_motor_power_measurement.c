/**
  ******************************************************************************
  * @file    pqd_motor_power_measurement.c
  * @author  FTEX inc
  * @brief   This file provides firmware functions that implement the following features
  *          of the PQD Motor Power Measurement component of the Motor Control SDK:
  *
  *           * Calculate power of the motor
  *           * Clear power measurement
  *           * Get Power of the motor
  *           * Get average Power of the motor
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/

#include "pqd_motor_power_measurement.h"

#include "mc_type.h"

/**
  * @brief  This method should be called with periodicity. It computes and
  *         returns the measured motor power expressed in watt. It is also used
  *         to fill, with that measure, the buffer used to compute the average
  *         motor power.
  * @param power handle.
  * @retval int16_t The measured motor power expressed in watt.
  */
void PQD_CalcElMotorPower( PQD_MotorPowMeas_Handle_t * pHandle )
{

  int32_t wAux, wAux2, wAux3;
  qd_t Iqd = pHandle->pFOCVars->Iqd;
  qd_t Vqd = pHandle->pFOCVars->Vqd;
  wAux = ( ( int32_t )Iqd.q * ( int32_t )Vqd.q ) +
         ( ( int32_t )Iqd.d * ( int32_t )Vqd.d );
  wAux /= 65536;

  wAux2 = pHandle->wConvFact * ( int32_t )VBS_GetAvBusVoltage_V( pHandle->pVBS );
  wAux2 /= 600; /* 600 is max bus voltage expressed in volt.*/

  wAux3 = wAux * wAux2;
  wAux3 *= 6; /* 6 is max bus voltage expressed in thousend of volt.*/
  wAux3 /= 10;
  wAux3 /= 65536;

  MPM_CalcElMotorPower( &pHandle->_super, wAux3 );

}


