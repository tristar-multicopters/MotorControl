/**
* @file   r_divider_bus_voltage_sensor.h
* @brief  This file provides firmware functions that implement the  features
*         of the Resistor Divider Bus Voltage Sensor component of the Motor
*         Control SDK:
*
*/

#ifndef __RDIVIDER_BUSVOLTAGESENSOR_H
#define __RDIVIDER_BUSVOLTAGESENSOR_H

#include "regular_conversion_manager.h"
#include "bus_voltage_sensor.h"

// ====== Structure used to configure resistor divider bus voltage sensor parameters ========== //
  
typedef struct
{
    BusVoltageSensor_Handle_t _Super;
    RegConv_t VbusRegConv; 
    uint16_t LowPassFilterBW;  // Use this number to configure the Vbus first order software filter bandwidth. hLowPassFilterBW = VBS_CalcBusReading call rate [Hz]/ FilterBandwidth[Hz].
    uint16_t OverVoltageThreshold;  // It represents the over voltage protection intervention threshold. To be expressed in digital value through formula: 
                                    // hOverVoltageThreshold (digital value) = Over Voltage Threshold (V) * 65536 / hConversionFactor
    uint16_t UnderVoltageThreshold;  // It represents the under voltage protection intervention threshold. To be expressed in digital value through formula:
                                     // hUnderVoltageThreshold (digital value)= Under Voltage Threshold (V) * 65536 / hConversionFactor.
    uint16_t *aBuffer;  // Buffer used to compute average value.
    uint8_t elem;  // Number of stored elements in the average buffer.
    uint8_t index;  // Index of last stored element in the average buffer.
    uint8_t convHandle;  // handle to the regular conversion.
} RDivider_Handle_t;

// ==================== Public function prototypes ========================= //

/**
* @brief  It initializes bus voltage conversion (ADC, ADC channel, conversion time. 
*         It must be called only after PWMC_Init.
* @param  pHandle related RDivider_Handle_t
* @retval none
*/
void RVBS_Init( RDivider_Handle_t * pHandle );

/**
* @brief  It clears bus voltage FW variable containing average bus voltage value
* @param  pHandle related RDivider_Handle_t
* @retval none
*/
void RVBS_Clear( RDivider_Handle_t * pHandle );

/**
* @brief  It actually reads the Vbus ADC conversion value, filters it and updates average value
* @param  pHandle related RDivider_Handle_t
* @retval uint16_t Fault code error
*/
uint16_t RVBS_CalcAvVbusFilt( RDivider_Handle_t * pHandle );

/**
* @brief  It actually reads the Vbus ADC conversion value and updates average value
* @param  pHandle related RDivider_Handle_t
* @retval uint16_t Fault code error
*/
uint16_t RVBS_CalcAvVbus( RDivider_Handle_t * pHandle );

/**
* @brief  It returns MC_OVER_VOLT, MC_UNDER_VOLT or MC_NO_ERROR depending on
*         bus voltage and protection threshold values
* @param  pHandle related RDivider_Handle_t
* @retval uint16_t Fault code error
*/
uint16_t RVBS_CheckFaultState( RDivider_Handle_t * pHandle );

#endif
