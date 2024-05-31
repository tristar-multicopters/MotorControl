/**
  * @file    flux_weakening_ctrl.h
  * @brief   This file provides firmware functions that implement the Flux Weakening
  *          Control component of the Motor Control application.
  *
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLUXWEAKENINGCTRL_H
#define __FLUXWEAKENINGCTRL_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "pid_regulator.h"
#include "motor_parameters.h"

/**
  * @brief  Flux Weakening Control Component handle structure
  */
typedef struct
{
    PIDHandle_t *       pMotorControlPID; /**< PI object used for flux weakening */
    PIDHandle_t *       pSpeedPID;         /**< PI object used for speed control */
    uint16_t        hFwVoltRef;              /**< Voltage reference, tenth of
                                                 percentage points */
    qd_t            AvVoltQd;              /**< Average stator voltage in qd
                                                 reference frame */
    int16_t         AvVoltAmpl;             /**< Average stator voltage amplitude */
    int16_t         hIdRefOffset;           /**< Id reference offset */
    uint16_t        hMaxModule;             /**< Circle limitation maximum allowed module */

    uint16_t        hDefaultFwVoltRef;       /**< Default flux weakening voltage reference,
                                               tenth of percentage points*/
    int16_t         hDemagCurrent;          /**< Demagnetization current in s16A:
                                               Current(Amp) = [Current(s16A) * Vdd micro]/
                                               [65536 * Rshunt * Aop] */
    int16_t         hNominalCurr;           /**< Squared motor nominal current in */
    int32_t         wNominalSqCurr;         /**< Squared motor nominal current in (s16A)^2
                                               where:
                                               Current(Amp) = [Current(s16A) * Vdd micro]/
                                               [65536 * Rshunt * Aop] */
    int16_t         wUsrMaxCurr;             /**< User Defined Maximum Curent comming from APT
                                               initial value is NominamMaxCurr but Vehicle layer
                                               can update the value */
    uint16_t        hVqdLowPassFilterBw;    /**< Use this parameter to configure the Vqd
                                               first order software filter bandwidth.
                                               hVqdLowPassFilterBw = FOC_CurrController
                                               call rate [Hz]/ FilterBandwidth[Hz] in
                                               case FULL_MISRA_COMPLIANCY is defined.
                                               On the contrary, if FULL_MISRA_COMPLIANCY
                                               is not defined, hVqdLowPassFilterBw is
                                               equal to log with base two of previous
                                               definition */
    uint16_t        hVqdLowPassFilterBwLog; /**< hVqdLowPassFilterBw expressed as power of 2.
                                               E.g. if gain divisor is 512 the value
                                               must be 9 because 2^9 = 512 */    
    float           fRS;                    /**< Stator resistance, in ohms */
    uint8_t         bWheelSpdSensorNbrPerRotation;      /**< Number of magnets on the wheel speed sensor */
    float           fMotorWSSTimeOnOneMagnetPercent;    /**< Percentage of time that the wheel speed sensor spends on each magnet */
    int16_t         hFlDir;                   /**< the dirction of demagnetization current Id */    
} MCConfigHandle_t;


/* Exported functions ------------------------------------------------------- */

/**
  * @brief  Initializes all the object variables, usually it has to be called
  *         once right after object creation.
  * @param  pHandle Flux weakening init strutcture.
  * @param  pPIDSpeed Speed PID structure.
  * @param  pPIDMotorControlHandle FW PID structure.
  * @retval none.
  */
void MotorControl_Init(MCConfigHandle_t * pHandle, PIDHandle_t * pPIDSpeed, PIDHandle_t * pPIDMotorControlHandle, MotorParameters_t MotorParameters);

/**
  * @brief  It should be called before each motor restart and clears the Flux
  *         weakening internal variables with the exception of the target
  *         voltage (hFwVoltRef).
  * @param  pHandle Flux weakening init strutcture.
  * @retval none
  */
void FluxWkng_Clear(MCConfigHandle_t * pHandle);

/**
  * @brief  It computes Iqdref according the flux weakening algorithm.  Inputs
  *         are the starting Iqref components.
  *         As soon as the speed increases beyond the nominal one, fluxweakening
  *         algorithm take place and handles Idref value. Finally, accordingly
  *         with new Idref, a new Iqref saturation value is also computed and
  *         put into speed PI.
  * @param  pHandle Flux weakening init strutcture.
  * @param  Iqdref The starting current components that have to be
  *         manipulated by the flux weakening algorithm.
  * @retval qd_t Computed Iqdref.
  */
qd_t FluxWkng_CalcCurrRef(MCConfigHandle_t * pHandle, qd_t Iqdref);

/**
  * @brief  It low-pass filters both the Vqd voltage components. Filter
  *         bandwidth depends on hVqdLowPassFilterBw parameter
  * @param  pHandle Flux weakening init strutcture.
  * @param  Vqd Voltage componets to be averaged.
  * @retval none
  */
void MC_DataProcess(MCConfigHandle_t * pHandle, qd_t Vqd);

/**
  * @brief  Use this method to set a new value for the voltage reference used by
  *         flux weakening algorithm.
  * @param  pHandle Flux weakening init strutcture.
  * @param  uint16_t New target voltage value, expressend in tenth of percentage
  *         points of available voltage.
  * @retval none
  */
void MC_SetVref(MCConfigHandle_t * pHandle, uint16_t hNewVref);

/**
  * @brief  It returns the present value of target voltage used by flux
  *         weakening algorihtm.
  * @param  pHandle Flux weakening init strutcture.
  * @retval int16_t Present target voltage value expressed in tenth of
  *         percentage points of available voltage.
  */
uint16_t MC_GetVref(MCConfigHandle_t * pHandle);

/**
  * @brief  It returns the present value of voltage actually used by flux
  *         weakening algorihtm.
  * @param  pHandle Flux weakening init strutcture.
  * @retval int16_t Present averaged phase stator voltage value, expressed
  *         in s16V (0-to-peak), where
  *         PhaseVoltage(V) = [PhaseVoltage(s16A) * Vbus(V)] /[sqrt(3) *32767].
  */
int16_t MC_GetAvVAmplitude(MCConfigHandle_t * pHandle);

/**
  * @brief  It returns the measure of present voltage actually used by flux
  *         weakening algorihtm as percentage of available voltage.
  * @param  pHandle Flux weakening init strutcture.
  * @retval uint16_t Present averaged phase stator voltage value, expressed in
  *         tenth of percentage points of available voltage.
  */
uint16_t MC_GetAvVPercentage(MCConfigHandle_t * pHandle);


#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __FLUXWEAKENINGCTRL_H */

