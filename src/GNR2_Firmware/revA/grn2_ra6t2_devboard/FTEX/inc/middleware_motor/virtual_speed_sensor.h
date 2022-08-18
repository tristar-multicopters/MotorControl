/**
  * @file    virtual_speed_sensor.h
  * @brief   This file contains all definitions and functions prototypes for the
  *          Virtual Speed Sensor component of the Motor Control application.
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _VIRTUALSPEEDSENSOR_H
#define _VIRTUALSPEEDSENSOR_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "speed_pos_fdbk.h"

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  This structure is used to handle an instance of the Virtual Speed
  *         sensor component
  */
typedef struct
{
  SpdPosFdbkHandle_t   Super;
  int32_t wElAccDppP32;   /*!< Delta electrical speed expressed in dpp per speed
                               sampling period to be appied each time is called
                               SPD_CalcAvrgMecSpeedUnit multiplied by scaling
                               factor of 65536.*/
  int32_t wElSpeedDpp32;  /*!< Electrical speed expressed in dpp multiplied by
                               scaling factor 65536.*/
  uint16_t hRemainingStep;/*!< Number of steps remaining to reach the final
                               speed.*/
  int16_t hFinalMecSpeedUnit;/*!< Backup of hFinalMecSpeedUnit to be applied in
                               the last step.*/
  bool bTransitionStarted;    /*!< Retaining information about Transition status.*/
  bool bTransitionEnded;      /*!< Retaining information about ransition status.*/
  int16_t hTransitionRemainingSteps;  /*!< Number of steps remaining to end
                               transition from CVirtualSpdSensor_SPD to other CSPD*/
  int16_t hElAngleAccu;        /*!< Electrical angle accumulator*/
  bool bTransitionLocked;      /*!< Transition acceleration started*/
  bool bCopyObserver;          /*!< Command to set VSPD output same as state observer*/

  uint16_t hSpeedSamplingFreqHz; /*!< Frequency (Hz) at which motor speed is to
                             be computed. It must be equal to the frequency
                             at which function SPD_CalcAvrgMecSpeedUnit
                             is called.*/
  int16_t hTransitionSteps; /*< Number of steps to perform the transition phase
                             from CVirtualSpdSensor_SPD to other CSPD; if the Transition PHase
                             should last TPH milliseconds, and the FOC Execution
                             Frequency is set to FEF kHz, then
                             hTransitionSteps = TPH * FEF*/

} VirtualSpeedSensor_Handle_t;

/**
* @brief  Software initialization of VirtualSpeedSensor component
* @param  pHandle: handler of the current instance of the VirtualSpeedSensor component
* @retval none
*/
void VirtualSpdSensor_Init(VirtualSpeedSensor_Handle_t * pHandle);

/**
* @brief  Software initialization of VSS object to be performed at each restart
*         of the motor.
* @param  pHandle: handler of the current instance of the VirtualSpeedSensor component
* @retval none
*/
void VirtualSpdSensor_Clear(VirtualSpeedSensor_Handle_t * pHandle);

/**
* @brief  Update the rotor electrical angle integrating the last setled
*         instantaneous electrical speed express in dpp.
* @param  pHandle: handler of the current instance of the VirtualSpeedSensor component
* @retval int16_t Measured electrical angle in s16degree format.
*/
int16_t VirtualSpdSensor_CalcElAngle(VirtualSpeedSensor_Handle_t * pHandle, void * pInputVars_str);

/**
  * @brief  This method must be called with the same periodicity
  *         on which speed control is executed.
  *         This method computes and stores rotor instantaneous el speed (express
  *         in dpp considering the measurement frequency) in order to provide it
  *         to SPD_CalcElAngle function and SpdPosFdbk_GetElAngle.
  *         Then compute store and return - through parameter
  *         pMecSpeedUnit - the rotor average mech speed, expressed in the unit
  *         defined by #SPEED_UNIT. Then return the reliability state of the
  *         sensor (always true).
  * @param  pHandle: handler of the current instance of the VirtualSpeedSensor component
  * @param  pMecSpeedUnit pointer to int16_t, used to return the rotor average
  *         mechanical speed (SPED_UNIT)
  * @retval true = sensor information is reliable
  *         false = sensor information is not reliable
  */
bool VirtualSpdSensor_CalcAvrgMecSpeedUnit(VirtualSpeedSensor_Handle_t * pHandle, int16_t * pMecSpeedUnit);

/**
  * @brief  It is used to set istantaneous information on VSS mechanical and
  *         electrical angle.
  * @param  pHandle: handler of the current instance of the VirtualSpeedSensor component
  * @param  hMecAngle istantaneous measure of rotor mechanical angle
  * @retval none
  */
void VirtualSpdSensor_SetMecAngle(VirtualSpeedSensor_Handle_t * pHandle, int16_t hMecAngle);

/**
  * @brief  Set the mechanical acceleration of virtual sensor. This acceleration
            is defined starting from current mechanical speed, final mechanical
            speed expressed in 0.1Hz and duration expressed in milliseconds.
  * @param  pHandle: handler of the current instance of the VirtualSpeedSensor component
  * @param  hFinalMecSpeedUnit mechanical speed  assumed by
            the virtual sensor at the end of the duration. Expressed in the unit defined
            by #SPEED_UNIT.
  * @param  hDurationms Duration expressed in ms. It can be 0 to apply
            instantaneous the final speed.
  * @retval none
  */
void  VirtualSpdSensor_SetMecAcceleration(VirtualSpeedSensor_Handle_t * pHandle, int16_t  hFinalMecSpeedUnit,
                              uint16_t hDurationms);

/**
  * @brief  Checks if the ramp executed after a VSPD_SetMecAcceleration command
  *         has been completed.
  * @param  pHandle: handler of the current instance of the VirtualSpeedSensor component
  * @retval bool true if the ramp is completed, otherwise false.
  */
bool VirtualSpdSensor_IsRampCompleted(VirtualSpeedSensor_Handle_t * pHandle);

/**
  * @brief  Get the final speed of last setled ramp of virtual sensor expressed
            in 0.1Hz.
  * @param  pHandle: handler of the current instance of the VirtualSpeedSensor component
  * @retval none
  */
int16_t  VirtualSpdSensor_GetLastRampFinalSpeed(VirtualSpeedSensor_Handle_t * pHandle);

/**
  * @brief  Set the command to Start the transition phase from VirtualSpeedSensor
            to other SpeedSensor.
            Transition is to be considered ended when Sensor information is
            declared 'Reliable' or if function returned value is false
  * @param  pHandle: handler of the current instance of the VirtualSpeedSensor component
  * @param  bool true to Start the transition phase, false has no effect
  * @retval bool true if Transition phase is enabled (started or not), false if
            transition has been triggered but it's actually disabled
            (parameter hTransitionSteps = 0)
  */
bool VirtualSpdSensor_SetStartTransition(VirtualSpeedSensor_Handle_t * pHandle, bool bCommand);

/**
  * @brief  Return the status of the transition phase.
  * @param  pHandle: handler of the current instance of the VirtualSpeedSensor component
  * @retval bool true if Transition phase is ongoing, false otherwise.
  */
bool VirtualSpdSensor_IsTransitionOngoing(VirtualSpeedSensor_Handle_t * pHandle);

bool VirtualSpdSensor_TransitionEnded(VirtualSpeedSensor_Handle_t * pHandle);

/**
  * @brief  It set istantaneous information on rotor electrical angle copied by state observer;
  * @param  pHandle: handler of the current instance of the VirtualSpeedSensor component
  * @retval none
  */
void VirtualSpdSensor_SetCopyObserver(VirtualSpeedSensor_Handle_t * pHandle);

/**
  * @brief  It  set istantaneous information on rotor electrical angle.
  * @param  pHandle: handler of the current instance of the VirtualSpeedSensor component
  * @param  hElAngle istantaneous measure of rotor electrical angle (s16degrees)
  * @retval none
  */
void VirtualSpdSensor_SetElAngle(VirtualSpeedSensor_Handle_t * pHandle, int16_t hElAngle);

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* _VIRTUALSPEEDSENSOR_H */


