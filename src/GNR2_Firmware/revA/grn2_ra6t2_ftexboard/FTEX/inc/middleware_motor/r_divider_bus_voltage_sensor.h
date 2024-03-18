/**
  * @file    r_divider_bus_voltage_sensor.h
  * @brief   This file contains all definitions and functions prototypes for the
  *          Resistor Divider Bus Voltage Sensor component of the Motor Control application.
*/

#ifndef __RDIVIDER_BUSVOLTAGESENSOR_H
#define __RDIVIDER_BUSVOLTAGESENSOR_H

#include "regular_conversion_manager.h"
#include "bus_voltage_sensor.h"

/************** DEFINES************************/

//under voltage timeout to the system volatge to be stable.
#define UNDERVOLTAGE_INITTIMEOUT_MSEC  6000



// ====== Structure used to configure resistor divider bus voltage sensor parameters ========== //

typedef struct
{
  BusVoltageSensorHandle_t Super;

  RegConv_t      VbusRegConv;
  uint16_t       hLowPassFilterBw;       /*!< Use this number to configure the Vbus
                                             first order software filter bandwidth.
                                             hLowPassFilterBw = VBS_CalcBusReading
                                             call rate [Hz]/ FilterBandwidth[Hz] */
  uint16_t       hOverVoltageThreshold;  /*!< It represents the over voltage protection
                                             intervention threshold. To be expressed
                                             in digital value through formula:
                                             hOverVoltageThreshold (digital value) =
                                             Over Voltage Threshold (V) * 65536
                                             / hConversionFactor */
  uint16_t       hUnderVoltageThreshold; /*!< It represents the under voltage protection
                                             intervention threshold. To be expressed
                                             in digital value through formula:
                                             hUnderVoltageThreshold (digital value)=
                                             Under Voltage Threshold (V) * 65536
                                             / hConversionFactor */
  uint16_t       *aBuffer;                /*!< Buffer used to compute average value.*/
  uint8_t        bElem;                  /*!< Number of stored elements in the average buffer.*/
  uint8_t        bIndex;                 /*!< Index of last stored element in the average buffer.*/
  uint8_t        bConvHandle;            /*!< handle to the regular conversion */

} ResDivVbusSensorHandle_t;

/* Exported functions ------------------------------------------------------- */

/**
  * @brief  It initializes bus voltage conversion (ADC, ADC channel, conversion time.
    It must be called only after PWMCurrFdbk_Init.
  * @param  pHandle related ResDivVbusSensorHandle_t
  * @retval none
  */
void ResDivVbusSensor_Init(ResDivVbusSensorHandle_t * pHandle);

/**
  * @brief  It initializes the undervoltage threshold value from the battery parameters.
  * @param  pHandle related ResDivVbusSensorHandle_t
  *         UVThresh is the undervoltage threshold value from the battery parameters
  * @retval none
  */
void ResDivVbusSensor_UVInit(ResDivVbusSensorHandle_t * pResDivVbusSensor, MC_Setup_t MCSetup);

/**
  * @brief  It clears bus voltage FW variable containing average bus voltage
  *         value
  * @param  pHandle related ResDivVbusSensorHandle_t
  * @retval none
  */
void ResDivVbusSensor_Clear(ResDivVbusSensorHandle_t * pHandle);


/**
  * @brief  It actually performes the Vbus ADC conversion and updates average
  *         value
  * @param  pHandle related ResDivVbusSensorHandle_t
  * @retval uint16_t Fault code error
  */
uint32_t ResDivVbusSensor_CalcAvVbus(ResDivVbusSensorHandle_t * pHandle);

/**
  * @brief  It returns MC_OVER_VOLT, MC_UNDER_VOLT or MC_NO_ERROR depending on
  *         bus voltage and protection threshold values
  * @param  pHandle related ResDivVbusSensorHandle_t
  * @retval uint16_t Fault code error
  */
uint32_t ResDivVbusSensor_CheckFaultState(ResDivVbusSensorHandle_t * pHandle);

#endif
