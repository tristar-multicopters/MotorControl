/**
  * @file    virtual_bus_voltage_sensor.h
  * @brief   This file contains all definitions and functions prototypes for the
  *          Virtual Bus Voltage Sensor component of the Motor Control application.
*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VIRTUAL_BUSVOLTAGESENSOR_H
#define __VIRTUAL_BUSVOLTAGESENSOR_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "bus_voltage_sensor.h"
#include "ASSERT_FTEX.h"

/**
  * @brief  Virtual Vbus sensor class parameters definition
  */
typedef struct
{
  BusVoltageSensorHandle_t Super;

  uint16_t hExpectedVbusDigital;            /*!< Expected Vbus voltage expressed in
                                           digital value
                                           hOverVoltageThreshold(digital value)=
                                           Over Voltage Threshold (V) * 65536
                                           / 500 */
} VirtualBusVoltageSensorHandle_t;

/* Exported functions ------------------------------------------------------- */

/**
  * @brief  It initializes bus voltage conversion for virtual bus voltage sensor
  * @param  pHandle related Handle of VirtualBusVoltageSensorHandle_t
  * @retval none
  */
void VirtualBusVoltSensor_Init(VirtualBusVoltageSensorHandle_t * pHandle);

/**
  * @brief  It simply returns in virtual Vbus sensor implementation
  * @param  pHandle related Handle of VirtualBusVoltageSensorHandle_t
  * @retval none
  */
void VirtualBusVoltSensor_Clear(VirtualBusVoltageSensorHandle_t * pHandle);

/**
  * @brief  It returns MC_NO_FAULT
  * @param  pHandle related Handle of VirtualBusVoltageSensorHandle_t
* @retval uint16_t Fault code error: MC_NO_FAULT
  */
uint16_t VirtualBusVoltSensor_NoErrors(VirtualBusVoltageSensorHandle_t * pHandle);


#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __CCC_H */



