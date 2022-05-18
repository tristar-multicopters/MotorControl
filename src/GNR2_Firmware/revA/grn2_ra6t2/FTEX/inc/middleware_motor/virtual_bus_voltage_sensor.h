/**
  ******************************************************************************
  * @file    virtual_bus_voltage_sensor.h
  * @author  FTEX inc
  * @brief   This file contains all definitions and functions prototypes for the
  *          Virtual Bus Voltage Sensor component of the Motor Control SDK.
  ******************************************************************************
*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VIRTUAL_BUSVOLTAGESENSOR_H
#define __VIRTUAL_BUSVOLTAGESENSOR_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "bus_voltage_sensor.h"

/**
  * @brief  Virtual Vbus sensor class parameters definition
  */
typedef struct
{
  BusVoltageSensor_Handle_t _Super;

  uint16_t ExpectedVbus_d;            /*!< Expected Vbus voltage expressed in
                                           digital value
                                           hOverVoltageThreshold(digital value)=
                                           Over Voltage Threshold (V) * 65536
                                           / 500 */
} VirtualBusVoltageSensor_Handle_t;

/* Exported functions ------------------------------------------------------- */
void VVBS_Init( VirtualBusVoltageSensor_Handle_t * pHandle );
void VVBS_Clear( VirtualBusVoltageSensor_Handle_t * pHandle );
uint16_t VVBS_NoErrors( VirtualBusVoltageSensor_Handle_t * pHandle );


/** @} */
#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __CCC_H */



