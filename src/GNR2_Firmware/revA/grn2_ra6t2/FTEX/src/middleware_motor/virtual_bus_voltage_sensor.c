/**
  ******************************************************************************
  * @file    virtual_bus_voltage_sensor.c
  * @author  FTEX inc
  * @brief   This file provides firmware functions that implement the features
  *          of the Virtual Bus Voltage Sensor component of the Motor Control SDK.
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "virtual_bus_voltage_sensor.h"

/**
  * @brief  It initializes bus voltage conversion for virtual bus voltage sensor
  * @param  pHandle related Handle of VirtualBusVoltageSensor_Handle_t
  * @retval none
  */
void VVBS_Init( VirtualBusVoltageSensor_Handle_t * pHandle )
{
  pHandle->_Super.FaultState = MC_NO_ERROR;
  pHandle->_Super.LatestConv = pHandle->ExpectedVbus_d;
  pHandle->_Super.AvBusVoltage_d = pHandle->ExpectedVbus_d;
}

/**
  * @brief  It simply returns in virtual Vbus sensor implementation
  * @param  pHandle related Handle of VirtualBusVoltageSensor_Handle_t
  * @retval none
  */
void VVBS_Clear( VirtualBusVoltageSensor_Handle_t * pHandle )
{
    if(pHandle)
    {
    }
  return;
}

/**
  * @brief  It returns MC_NO_ERROR
  * @param  pHandle related Handle of VirtualBusVoltageSensor_Handle_t
* @retval uint16_t Fault code error: MC_NO_ERROR
  */
uint16_t VVBS_NoErrors( VirtualBusVoltageSensor_Handle_t * pHandle )
{
    if(pHandle)
    {
    }
    return ( MC_NO_ERROR );
}


/** @} */



