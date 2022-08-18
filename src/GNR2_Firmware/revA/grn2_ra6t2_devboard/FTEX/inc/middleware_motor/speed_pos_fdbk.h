/**
 * @file    speed_pos_fdbk.h
 * @brief   This file provides all definitions and functions prototypes
 *          of the Speed & Position Feedback component.
 *
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPEEDNPOSFDBK_H
#define __SPEEDNPOSFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "mc_type.h"


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
    uint8_t bSpeedUnit; /*!< The speed unit value is defined into mc_stm_types.h*/   

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


} SpeednPosFdbkHandle_t;

/**
  * @brief input structure type definition for SPD_CalcAngle
  */
typedef struct
{
  AlphaBeta_t  Valfa_beta;
  AlphaBeta_t  Ialfa_beta;
  uint16_t         Vbus;
} BemfObserverInputs_t;


/**
  * @brief  It returns the last computed rotor electrical angle, expressed in
  *         s16degrees. 1 s16degree = 360�/65536
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @retval int16_t rotor electrical angle (s16degrees)
  */
int16_t SpdPosFdbk_GetElAngle(SpeednPosFdbkHandle_t * pHandle);

/**
  * @brief  It returns the last computed rotor mechanical angle, expressed in
  *         s16degrees. Mechanical angle frame is based on parameter bElToMecRatio
  *         and, if occasionally provided - through function SPD_SetMecAngle -
  *         of a measured mechanical angle, on information computed thereof.
  * @note   both Hall sensor and Sensor-less do not implement either
  *         mechanical angle computation or acceleration computation.
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @retval int16_t rotor mechanical angle (s16degrees)
  */
int32_t SpdPosFdbk_GetMecAngle(SpeednPosFdbkHandle_t * pHandle);

/**
  * @brief  Returns the last computed average mechanical speed, expressed in
  *         the unit defined by #SPEED_UNIT.
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  */
int16_t SpdPosFdbk_GetAvrgMecSpeedUnit(SpeednPosFdbkHandle_t * pHandle);

/**
  * @brief  It returns the last computed electrical speed, expressed in Dpp.
  *         1 Dpp = 1 s16Degree/control Period. The control period is the period
  *         on which the rotor electrical angle is computed (through function
  *         SPD_CalcElectricalAngle).
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @retval int16_t rotor electrical speed (Dpp)
  */
int16_t SpdPosFdbk_GetElSpeedDpp(SpeednPosFdbkHandle_t * pHandle);

/**
  * @brief  It returns the last instantaneous computed electrical speed, expressed in Dpp.
  *         1 Dpp = 1 s16Degree/control Period. The control period is the period
  *         on which the rotor electrical angle is computed (through function
  *         SPD_CalcElectricalAngle).
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @retval int16_t rotor instantaneous electrical speed (Dpp)
  */
int16_t SpdPosFdbk_GetInstElSpeedDpp (SpeednPosFdbkHandle_t * pHandle);

/**
  * @brief  It returns the result of the last reliability check performed.
  *         Reliability is measured with reference to parameters
  *         hMaxReliableElSpeedUnit, hMinReliableElSpeedUnit,
  *         bMaximumSpeedErrorsNumber and/or specific parameters of the derived
  *         true = sensor information is reliable
  *         false = sensor information is not reliable
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @retval bool sensor reliability state
  */
bool SpdPosFdbk_GetReliability(SpeednPosFdbkHandle_t * pHandle);

/**
  * @brief  This method must be called - at least - with the same periodicity
  *         on which speed control is executed. It computes and returns - through
  *         parameter pMecSpeedUnit - the rotor average mechanical speed,
  *         expressed in the unit defined by #SPEED_UNIT. It computes and returns
  *         the reliability state of the sensor; reliability is measured with
  *         reference to parameters hMaxReliableElSpeedUnit, hMinReliableElSpeedUnit,
  *         bMaximumSpeedErrorsNumber and/or specific parameters of the derived
  *         true = sensor information is reliable
  *         false = sensor information is not reliable
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @param  pMecSpeedUnit pointer to int16_t, used to return the rotor average
  *         mechanical speed (expressed in the unit defined by #SPEED_UNIT)
  * @retval none
  */
bool SpdPosFdbk_CalcReliability(SpeednPosFdbkHandle_t * pHandle, int16_t * pMecSpeedUnit);

/**
  * @brief  This method returns the average mechanical rotor speed expressed in
  *         "S16Speed". It means that:\n
  *         - it is zero for zero speed,\n
  *         - it become INT16_MAX when the average mechanical speed is equal to
  *           hMaxReliableMecSpeedUnit,\n
  *         - it becomes -INT16_MAX when the average mechanical speed is equal to
  *         -hMaxReliableMecSpeedUnit.
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @retval int16_t The average mechanical rotor speed expressed in "S16Speed".
  */
int16_t SpdPosFdbk_GetS16Speed(SpeednPosFdbkHandle_t * pHandle);

/**
  * @brief  This method returns the coefficient used to transform electrical to
  *         mechanical quantities and viceversa. It usually coincides with motor
  *         pole pairs number.
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @retval uint8_t The motor pole pairs number.
  */
uint8_t SpdPosFdbk_GetElToMecRatio(SpeednPosFdbkHandle_t * pHandle);

/**
  * @brief  This method sets the coefficient used to transform electrical to
  *         mechanical quantities and viceversa. It usually coincides with motor
  *         pole pairs number.
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @param  bPP The motor pole pairs number to be set.
  */
void SpdPosFdbk_SetElToMecRatio(SpeednPosFdbkHandle_t * pHandle, uint8_t bPP);


#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __SPEEDNPOSFDBK_H */

