/**
  * @file    pqd_motor_power_measurement.h
  * @brief   This file provides firmware functions that implement the following features
  *          of the PQD Motor Power Measurement component of the Motor Control application:
  *
  *           * Calculate power of the motor
  *           * Clear power measurement
  *           * Get Power of the motor
  *           * Get average Power of the motor
  *
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PQD_MOTORPOWERMEASUREMENT_H
#define __PQD_MOTORPOWERMEASUREMENT_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "motor_power_measurement.h"
#include "bus_voltage_sensor.h"

typedef struct
{
  MotorPowerMeasHandle_t Super;

  int32_t wConvFact; /* It is the conversion factor used to convert the
                         variables expressed in digit into variables expressed
                         in physical measurement unit. It is used to convert the
                         power in watts. It must be equal to
                         (1000 * 3 * Vddï¿½)/(sqrt(3) * Rshunt * Aop) */

  pFOCVars_t pFOCVars;    /*!< Pointer to FOC vars used by MPM.*/
  BusVoltageSensorHandle_t * pVBS;              /*!< Bus voltage sensor object used by MPM.*/
} MotorPowerQDHandle_t;



/**
  * @brief Implementation of derived class init method. It should be called before each motor restart.
  * @param pHandle related component instance.
  * @retval none.
  */
void MotorPowerQD_Clear(MotorPowerQDHandle_t * pHandle);

/**
  * @brief Implementation of derived class CalcElMotorPower.
  * @param pHandle related component instance.
  * @retval int16_t The measured motor power expressed in watt.
  */
void MotorPowerQD_CalcElMotorPower(MotorPowerQDHandle_t * pHandle);


#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __CCC_H */


