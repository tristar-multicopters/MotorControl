/**
  * @file    odometer.h
  * @brief   This module handles odometer management
  *
  */
    
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ODOMETER_H
#define __ODOMETER_H

#include "stdbool.h"
#include "stdint.h"
#include "delay.h"
#include "wheel.h"

#define KM_H_TO_M_S 3.6f


/**
  * @brief Odometer_t structure used for odometer monitoring
  */
typedef struct
{                   
    uint16_t hInstADCValue;          // It contains latest available instantaneous ADC value.
    uint16_t hExtLatestVal;          // Contains the latest external throttle value 
    float   TimeIntervalMS;
    uint32_t OdometerDistance;
    WheelSpeedSensorHandle_t * pWSS;
    
    Delay_Handle_t * pOdometerDelay;
    
} OdometerHandle_t;


/**
 * @brief Initializes odometer module
 * @param  pHandle : Pointer on Handle of the odometer
 * @retval void
 */
void Odometer_Init(Delay_Handle_t * pOdometerDelay, WheelSpeedSensorHandle_t * pWSS, uint16_t TimeIntervalMS);

void Odometer_Update(void);

uint32_t Odometer_GetDistanceTravelled(void);

void Odometer_Save(void);
    
#endif /*__ODOMETER_H*/

