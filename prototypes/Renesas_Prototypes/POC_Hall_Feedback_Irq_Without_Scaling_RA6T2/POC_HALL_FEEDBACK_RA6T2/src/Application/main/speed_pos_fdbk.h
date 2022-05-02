/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPEEDNPOSFDBK_H
#define __SPEEDNPOSFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "hal_data.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/**
  * @brief  SpeednPosFdbk  handle definition
  */
typedef struct
{

  uint8_t bSpeedErrorNumber;

  uint8_t bElToMecRatio;  /*!< Coefficient used to transform electrical to
                               mechanical quantities and viceversa. It usually
                               coincides with motor pole pairs number*/
  uint8_t SpeedUnit; /*!< The speed unit value is defined into mc_stm_types.h*/   
  
  uint8_t bMaximumSpeedErrorsNumber; /*!< Maximum value of not valid measurements
                                        before an error is reported.*/    
  int16_t hElAngle;

  int16_t hMecAngle;
  
  int32_t wMecAngle;

  int16_t hAvrMecSpeedUnit;

  int16_t hElSpeedDpp;
  int16_t InstantaneousElSpeedDpp;

  int16_t hMecAccelUnitP;
                            
  uint16_t hMaxReliableMecSpeedUnit; /*!< Maximum value of measured mechanical speed that is
                                        considered to be valid. Expressed
                                        in the unit defined by #SPEED_UNIT.*/
  uint16_t hMinReliableMecSpeedUnit; /*!< Minimum value of measured mechanical speed that is
                                        considered to be valid. Expressed
                                        in the unit defined by #SPEED_UNIT.*/
  uint16_t hMaxReliableMecAccelUnitP; /*!< Maximum value of measured acceleration
                                        that is considered to be valid. Expressed in
                                        the unit defined by #SPEED_UNIT */
  uint16_t hMeasurementFrequency;  /*!< Frequency at which the user will request
                                    a measurement of the rotor electrical angle. Expressed in PWM_FREQ_SCALING*Hz.
                                    It is also used to convert measured speed from the unit
                                    defined by #SPEED_UNIT to dpp and viceversa.*/
  uint32_t DPPConvFactor; /* (65536/PWM_FREQ_SCALING) */
  

} SpeednPosFdbk_Handle_t;



int16_t SPD_GetElAngle( SpeednPosFdbk_Handle_t * pHandle );

int32_t SPD_GetMecAngle( SpeednPosFdbk_Handle_t * pHandle );

int16_t SPD_GetAvrgMecSpeedUnit( SpeednPosFdbk_Handle_t * pHandle );

int16_t SPD_GetElSpeedDpp( SpeednPosFdbk_Handle_t * pHandle );

int16_t SPD_GetInstElSpeedDpp ( SpeednPosFdbk_Handle_t * pHandle );

bool SPD_Check( SpeednPosFdbk_Handle_t * pHandle );

bool SPD_IsMecSpeedReliable( SpeednPosFdbk_Handle_t * pHandle, int16_t * pMecSpeedUnit );

int16_t SPD_GetS16Speed( SpeednPosFdbk_Handle_t * pHandle );

uint8_t SPD_GetElToMecRatio( SpeednPosFdbk_Handle_t * pHandle );

void SPD_SetElToMecRatio( SpeednPosFdbk_Handle_t * pHandle, uint8_t bPP );

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __SPEEDNPOSFDBK_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
