/**
* @file    speed_pos_fdbk.h
* @brief   Speed position feedback module for motor control application
* 
*This file provides all definitions and functions prototypes of the Speed & Position Feedback component.
*
*/

#ifndef __SPEEDNPOSFDBK_H
#define __SPEEDNPOSFDBK_H

#include "stdint.h"
#include "mc_type.h"

// ===== Structure used to configure speed position feedback handle ======== //

typedef struct  // Contain different parameters to configure speed position handle and variables to be used in realtime execution 
{
    uint8_t bSpeedErrorNumber;  // Variable to store number of errors that have occured while executing the algorithm. It is used for comparitive check with maximum permissible errors.
    uint8_t bElToMecRatio;  // Coefficient used to transform electrical to mechanical quantities and viceversa. It usually coincides with motor pole pairs number
    uint8_t SpeedUnit;  // It is a pseudo conversion factor used to calculate speed. The speed unit value is defined into mc_types.h   
    uint8_t bMaximumSpeedErrorsNumber;  // Maximum value of not valid measurements before an error is reported.*/    
    int16_t hElAngle;  // electrical angle specified in 32767 to -32768 which corresponds to 180 to -180 degrees
    int16_t hMecAngle;
    int32_t wMecAngle;
    int16_t hAvrMecSpeedUnit;
    int16_t hElSpeedDpp;  // Electrical speed specified in number of digital pulses per PWM pweriod. 10dpp is approx 3Hz in physical world.
    int16_t InstantaneousElSpeedDpp;
    int16_t hMecAccelUnitP;                            
    uint16_t hMaxReliableMecSpeedUnit;  // Maximum value of measured mechanical speed that is considered to be valid. Expressed in the unit defined by #SPEED_UNIT.*/
    uint16_t hMinReliableMecSpeedUnit;  // Minimum value of measured mechanical speed that is considered to be valid. Expressed in the unit defined by #SPEED_UNIT.*/
    uint16_t hMaxReliableMecAccelUnitP;  // Maximum value of measured acceleration that is considered to be valid. Expressed in the unit defined by #SPEED_UNIT */
    uint16_t hMeasurementFrequency;  // Frequency at which the user will request a measurement of the rotor electrical angle. Expressed in PWM_FREQ_SCALING*Hz. It is also used to convert measured speed from the unit defined by #SPEED_UNIT to dpp and viceversa.*/
    uint32_t DPPConvFactor;  // (65536/PWM_FREQ_SCALING) */
} SpeednPosFdbk_Handle_t;

typedef struct  // Structure that holds input variables if an angle observer is used.
{
    alphabeta_t Valfa_beta;
    alphabeta_t Ialfa_beta;
    uint16_t Vbus;
} Observer_Inputs_t;

// ==================== Public function prototypes ========================= //

/**
 * @brief  It returns the last computed rotor electrical angle, expressed in s16degrees. 180 to -180 corelates to 32767 to -32768
 * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
 * @retval int16_t rotor electrical angle (s16degrees)
 */
int16_t SPD_GetElAngle( SpeednPosFdbk_Handle_t * pHandle );

/**
* @brief  It returns the last computed rotor mechanical angle, expressed in
*         s16degrees. Mechanical angle frame is based on parameter bElToMecRatio
*         and, if occasionally provided - through function SPD_SetMecAngle -
*         of a measured mechanical angle, on information computed thereof.
* @note   both Hall sensor and Sensor-less do not implement either mechanical angle computation or acceleration computation.
* @param  pHandle: handler of the current instance of the SpeednPosFdbk component
* @retval int16_t rotor mechanical angle (s16degrees)
 */
int32_t SPD_GetMecAngle( SpeednPosFdbk_Handle_t * pHandle );

/**
* @brief  Returns the last computed average mechanical speed, expressed in the unit defined by #SPEED_UNIT.
* @param  pHandle: handler of the current instance of the SpeednPosFdbk component
*/
int16_t SPD_GetAvrgMecSpeedUnit( SpeednPosFdbk_Handle_t * pHandle );

/**
* @brief  It returns the last computed electrical speed, expressed in Dpp. 1 Dpp = 1 s16Degree/control Period. The control period is the period
*         on which the rotor electrical angle is computed (through function SPD_CalcElectricalAngle).
* @param  pHandle: handler of the current instance of the SpeednPosFdbk component
* @retval int16_t rotor electrical speed (Dpp)
*/
int16_t SPD_GetElSpeedDpp( SpeednPosFdbk_Handle_t * pHandle );

/**
 * @brief  It returns the last instantaneous computed electrical speed, expressed in Dpp. 1 Dpp = 1 s16Degree/control Period. The control period is the period
 *         on which the rotor electrical angle is computed (through function SPD_CalcElectricalAngle).
 * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
 * @retval int16_t rotor instantaneous electrical speed (Dpp)
 */
int16_t SPD_GetInstElSpeedDpp ( SpeednPosFdbk_Handle_t * pHandle );

/**
 * @brief  It returns the result of the last reliability check performed. Reliability is measured with reference to parameters
 *         hMaxReliableElSpeedUnit, hMinReliableElSpeedUnit, bMaximumSpeedErrorsNumber and/or specific parameters of the derived
 *         true = sensor information is reliable
 *         false = sensor information is not reliable
 * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
 * @retval bool sensor reliability state
 */
bool SPD_Check( SpeednPosFdbk_Handle_t * pHandle );

/**
 * @brief  This method must be called - at least - with the same periodicity on which speed control is executed. It computes and returns - through
 *         parameter pMecSpeedUnit - the rotor average mechanical speed, expressed in the unit defined by #SPEED_UNIT. It computes and returns
 *         the reliability state of the sensor; reliability is measured with reference to parameters hMaxReliableElSpeedUnit, hMinReliableElSpeedUnit,
 *         bMaximumSpeedErrorsNumber and/or specific parameters of the derived
 *         true = sensor information is reliable
 *         false = sensor information is not reliable
 * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
 * @param  pMecSpeedUnit pointer to int16_t, used to return the rotor average mechanical speed (expressed in the unit defined by #SPEED_UNIT)
 * @retval none
 */
bool SPD_IsMecSpeedReliable( SpeednPosFdbk_Handle_t * pHandle, int16_t * pMecSpeedUnit );

/**
 * @brief  This method returns the average mechanical rotor speed expressed in "S16Speed". It means that it is zero for zero speed,\n
 *         - it become INT16_MAX when the average mechanical speed is equal to hMaxReliableMecSpeedUnit,\n
 *         - it becomes -INT16_MAX when the average mechanical speed is equal to -hMaxReliableMecSpeedUnit.
 * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
 * @retval int16_t The average mechanical rotor speed expressed in "S16Speed".
 */
int16_t SPD_GetS16Speed( SpeednPosFdbk_Handle_t * pHandle );

/**
 * @brief  This method returns the coefficient used to transform electrical to
 *         mechanical quantities and viceversa. It usually coincides with motor pole pairs number.
 * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
 * @retval uint8_t The motor pole pairs number.
 */
uint8_t SPD_GetElToMecRatio( SpeednPosFdbk_Handle_t * pHandle );

/**
 * @brief  This method sets the coefficient used to transform electrical to
 *         mechanical quantities and viceversa. It usually coincides with motor pole pairs number.
 * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
 * @param  bPP The motor pole pairs number to be set.
 */
void SPD_SetElToMecRatio( SpeednPosFdbk_Handle_t * pHandle, uint8_t bPP );

#endif
