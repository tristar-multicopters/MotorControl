/**
  * @file    autotune.h
  * @brief   This file contains the parameters and functions needed for AutoTune.
*/

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __AUTOTUNE_H
#define __AUTOTUNE_H

#include "pwm_curr_fdbk.h"
#include "ics_ra6t2_pwm_curr_fdbk.h"

typedef struct
{
    uint8_t NumPolePairs;
    float RatedMotorCurrent; // In Ampere peak
    float DCBusVoltage;
    float KnownRs;
    float KnownLd;
    float KnownLq;
    float KnownMagnetFlux;   // In Wb
} MotorTunerInput_t; /* Typedef for input values of motor identification procedure */

typedef enum
{
    ERROR_NONE =                 (0x0000),            /**< Defines non-error code */
    ERROR_OVER_CURRENT_HW =      (0x0001),            /**< Defines error code, value 1 */
    ERROR_OVER_CURRENT_SW =      (0x0002),            /**< Defines error code, value 2 */
    ERROR_OVER_VOLTAGE =         (0x0003),            /**< Defines error code, value 3 */
    ERROR_UNDER_VOLTAGE =        (0x0004),            /**< Defines error code, value 4 */
    ERROR_INPUT =                (0x0011),            /**< Defines error code, value 17 */
    ERROR_R_DIFF =               (0x0012),            /**< Defines error code, value 18 */
    ERROR_R_DFT =                (0x0013),            /**< Defines error code, value 19 */
    ERROR_R_RLS =                (0x0014),            /**< Defines error code, value 20 */
    ERROR_LD_DFT =               (0x0015),            /**< Defines error code, value 21 */
    ERROR_LD_RLS =               (0x0016),            /**< Defines error code, value 22 */
    ERROR_LQ_DFT =               (0x0017),            /**< Defines error code, value 23 */
    ERROR_LQ_RLS =               (0x0018),            /**< Defines error code, value 24 */
    ERROR_KE =                   (0x0019),            /**< Defines error code, value 25 */
    ERROR_J =                    (0x001A),            /**< Defines error code, value 26 */
    ERROR_D =                    (0x001B),            /**< Defines error code, value 27 */
    ERROR_J_STARTUP =            (0x001C),            /**< Defines error code, value 28 */
    ERROR_ASSERT_FAIL =          (0xFFFF)             /**< Defines error code, value 65535 */
}hErrorCode_t;  //enum used to manage autotune errors.

typedef struct
{
    bool bCompleted;            // True if identification procedure completed successfully.
    hErrorCode_t hErrorCode;    // Error code, see r_aid_core.h and r_aid_auto_identify.h for the list.
    float fProgress;            // Progress of the identification procedure, from 0 to 100.

    float fBatteryVoltage;      // Battery voltage used for calculations, should be rated DC bus voltage.
    int16_t RatedTorque;        // Nominal torque
    int16_t RatedSpeed;         // Motor max speed without flux weakening
    int16_t IqKp;               // PID Gains
    int16_t IqKi;               // PID Gains
    int16_t IdKp;               // PID Gains
    int16_t IdKi;               // PID Gains 
    float Rs;                   // Phase winding resistance
    float Ld;                   // d axis inductance
    float Lq;                   // q axis inductance
    float Ke;                   // Motor voltage coefficient
    float J;                    // Rotor inertia
    float Friction;             // Motor friction
} MotorTunerOutput_t; /* Typedef for results of motor identification procedure, ready to use for motor control algorithm */

typedef enum
{
    TUNING_NOT_STARTED =  0,
    TUNING_START =        1,
    TUNING_IN_PROGRESS =  2,
    TUNING_DONE =         3,
    TUNING_ERROR =        4
}TuningStatus_t;    //enum used to manage autotune status.

typedef struct
{   
    MotorTunerInput_t   MotorTunerInput;    //Autotune input values
    MotorTunerOutput_t  MotorTunerOutput;   //Autotune output values
    TuningStatus_t      TuningStatus;       //Autotune status
    bool bStartTuning;                      //If this variable is true this means we start autotune
    bool bEnAutotune;                        //This variable puts the bike in autotune mode
} AutoTune_Handle_t;

/**
  * @brief  Returns the phase current of the motor (in s16A unit).To call in order to compute Ia, Ib and Ic and store into handle.
  * @param  pHandle: handle on the target PWM component
  * @param  Iab: Pointer to the structure that will receive motor current
  *         of phase A and B in s16A unit.
  * @retval void
*/
void Autotune_GetPhaseCurrents( PWMCurrFdbkHandle_t * pHandle, ab_t * Iab );

/**
  * @brief Returns the filtered phase current of the motor (in s16A unit).
  * @param  pHandle: handle on the target PWM component
  * @param  IabFilt: Pointer to the structure that will receive motor filtered current
  *         of phase A and B in s16A unit.
*/
void Autotune_GetFiltPhaseCurrents( PWMCurrFdbkHandle_t * pHandle, ab_t * IabFilt );

/**
  * @brief  Manually set duty cycles.
  * @param  pHandle handler on the target PWM component.
  * @param  Duty cycle of each leg of the inverter (x3).
  *
  * The function programs the duty cycles in the related timer channels. It also sets the phase current
  * sampling point for the next PWM cycle accordingly.
  *
  * This function is used in the FOC frequency loop and needs to complete before the next PWM cycle starts
  * so that the duty cycles it computes can be taken into account. Failing to do so (for instance because
  * the PWM Frequency is too high) results in the functions returning MC_FOC_DURATION which entails a
  * motor Control fault that stops the motor.
  *
  * @retval Returns MC_NO_FAULT if no error occurred or MC_FOC_DURATION if the duty cycles were
  *         set too late for being taken into account in the next PWM cycle (overrun condition).
  */
uint32_t Autotune_SetDuties(PWMCurrFdbkHandle_t * pHandle,uint16_t hDutyA, uint16_t hDutyB, uint16_t hDutyC);

/**
  * @brief  Compute Ia, Ib and Ic, as well as filtered version, and store into handle.Must be called once per control period.
  * @param  pHandle handler on the target PWM component.
  * @retval void
*/
void Autotune_CalcPhaseCurrents(PWMCurrFdbkHandle_t * pHandle);

#endif