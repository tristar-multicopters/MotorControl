/**
  * @file    pwm_curr_fdbk.h
  * @author  Sami Bouzid, FTEX inc
  * @brief   This file provides firmware functions that implement the following features
  *          of the PWM & Current Feedback component:
  *
  *           * current sensing
  *           * regular ADC conversion execution
  *           * space vector modulation
  *
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PWMNCURRFDBK_H
#define PWMNCURRFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "signal_filtering.h"
#include "math.h"
#include "stdlib.h"

/* Exported defines ------------------------------------------------------------*/

#define SECTOR_1  0u
#define SECTOR_2  1u
#define SECTOR_3  2u
#define SECTOR_4  3u
#define SECTOR_5  4u
#define SECTOR_6  5u
#define SQRT3FACTOR (uint16_t) 0xDDB4 /* = (16384 * 1.732051 * 2)*/

/* Exported types ------------------------------------------------------------*/

/** @brief PWM & Current Sensing component handle type */
typedef struct PWMCurrFdbkHandle PWMCurrFdbkHandle_t;

/**
  * @brief Pointer on callback functions used by PWMC components
  *
  * This type is needed because the actual functions to use can change at run-time.
  *
  * See the following items:
  * - PWMCurrFdbkHandle::pFctSwitchOffPwm
  * - PWMCurrFdbkHandle::pFctSwitchOnPwm
  * - PWMCurrFdbkHandle::pFctCurrReadingCalib
  * - PWMCurrFdbkHandle::pFctTurnOnLowSides
  * - PWMCurrFdbkHandle::pFctRLDetectionModeEnable
  * - PWMCurrFdbkHandle::pFctRLDetectionModeDisable
  *
  */
typedef void (*PWMCurrFdbk_Generic_Cb_t)(PWMCurrFdbkHandle_t * pHandle);

/**
  * @brief Pointer on the interrupt handling function of the PWMC component instance.
  *
  * This type is needed because the actual function to use can change at run-time
  * (See PWMCurrFdbkHandle::pFctIrqHandler).
  *
  */
typedef void * (*PWMCurrFdbk_IrqHandler_Cb_t)(PWMCurrFdbkHandle_t * pHandle, unsigned char flag);

/**
  * @brief Pointer on the function provided by the PMWC component instance to get the phase current.
  *
  * This type is needed because the actual function to use can change at run-time
  * (See PWMCurrFdbkHandle::pFctGetPhaseCurrents).
  *
  */
typedef void (*PWMCurrFdbk_GetPhaseCurr_Cb_t)(PWMCurrFdbkHandle_t * pHandle, ab_t * Iab);

/**
  * @brief Pointer on the functions provided by the PMWC component instance to set the ADC sampling
  *        point for each sectors.
  *
  * This type is needed because the actual function to use can change at run-time. See:
  * - PWMCurrFdbkHandle::pFctSetADCSampPointSect1
  * - PWMCurrFdbkHandle::pFctSetADCSampPointSect2
  * - PWMCurrFdbkHandle::pFctSetADCSampPointSect3
  * - PWMCurrFdbkHandle::pFctSetADCSampPointSect4
  * - PWMCurrFdbkHandle::pFctSetADCSampPointSect5
  * - PWMCurrFdbkHandle::pFctSetADCSampPointSect6
  *
  */
typedef uint32_t (*PWMCurrFdbk_SetSampPointSectX_Cb_t)(PWMCurrFdbkHandle_t * pHandle);

/**
  * @brief Pointer on the function provided by the PMWC component instance to check if an over current
  *        condition has occured.
  *
  * This type is needed because the actual function to use can change at run-time
  * (See PWMCurrFdbkHandle::pFctIsOverCurrentOccurred).
  *
  */
typedef uint32_t (*PWMCurrFdbk_OverCurr_Cb_t)(PWMCurrFdbkHandle_t * pHandle);

/**
  * @brief Pointer on the function provided by the PMWC component instance to set the PWM duty cycle
  *        in RL detection mode.
  *
  * This type is needed because the actual function to use can change at run-time
  * (See PWMCurrFdbkHandle::pFctRLDetectionModeSetDuty).
  *
  */
typedef uint32_t (*PWMCurrFdbk_RLDetectSetDuty_Cb_t)(PWMCurrFdbkHandle_t * pHandle, uint16_t hDuty);

/**
  * @brief This structure is used to handle the data of an instance of the PWM & Current Feedback component
  *
  */
struct PWMCurrFdbkHandle
{
    PWMCurrFdbk_Generic_Cb_t                      pFctInitialize;              /**< pointer on the function the component instance uses to initialize PWM&Curr feedback module */
    PWMCurrFdbk_IrqHandler_Cb_t 					pFctIrqHandler;              /**< pointer on the interrupt handling function. */
    PWMCurrFdbk_GetPhaseCurr_Cb_t 				pFctGetPhaseCurrents;        /**< pointer on the function the component instance uses to retrieve pahse currents */
    PWMCurrFdbk_Generic_Cb_t 						pFctSwitchOffPwm;            /**< pointer on the function the component instance uses to switch PWM off */
    PWMCurrFdbk_Generic_Cb_t 						pFctSwitchOnPwm;             /**< pointer on the function the component instance uses to switch PWM on */
    PWMCurrFdbk_Generic_Cb_t 						pFctCurrReadingCalib;        /**< pointer on the function the component instance uses to calibrate the current reading ADC(s) */
    PWMCurrFdbk_Generic_Cb_t 						pFctTurnOnLowSides;          /**< pointer on the function the component instance uses to turn low sides on */
    PWMCurrFdbk_SetSampPointSectX_Cb_t 	pFctSetADCSampPointSectX;  	 /**< pointer on the function the component instance uses to set the ADC sampling point  */
    PWMCurrFdbk_OverCurr_Cb_t 						pFctIsOverCurrentOccurred;   /**< pointer on the function the component instance uses to return the over current status */
    PWMCurrFdbk_Generic_Cb_t 						pFctRLDetectionModeEnable;   /**< pointer on the function the component instance uses to enable RL detection mode */
    PWMCurrFdbk_Generic_Cb_t 						pFctRLDetectionModeDisable;  /**< pointer on the function the component instance uses to disable RL detection mode */
    PWMCurrFdbk_RLDetectSetDuty_Cb_t 		pFctRLDetectionModeSetDuty;  /**< pointer on the function the component instance uses to set the PWM duty cycle in RL detection mode */

    uint16_t  hT_Sqrt3;                                    /**< a constant utilized by PWM algorithm (@f$\sqrt{3}@f$) */
    uint16_t  hCntPhA;                                     /**< PWM Duty cycle for phase A */
    uint16_t  hCntPhB;                                     /**< PWM Duty cycle for phase B */
    uint16_t  hCntPhC;                                     /**< PWM Duty cycle for phase C */
    uint16_t  hSWerror;                                     /**< Contains status about SW error */
    uint8_t   Sector;                                     /**< the space vector sector number */
    uint16_t  hLowDuty;
    uint16_t  hMidDuty;
    uint16_t  hHighDuty;
    bool 			hTurnOnLowSidesAction;                  /**< true if TurnOnLowSides action is active,
                                                              false otherwise. */
    uint8_t   Motor;                                      /**< Motor reference number */
    bool      bRLDetectionMode;                           /**< true if enabled, false if disabled. */
    int16_t   Ia;                                         /**< Last @f$I_{A}@f$ measurement. */
    int16_t   Ib;                                         /**< Last @f$I_{B}@f$ measurement. */
    int16_t   Ic;                                         /**< Last @f$I_{C}@f$ measurement. */

    uint16_t hPWMperiod;                                   /**< PWM period expressed in timer clock cycles unit:
                                                           *  @f$hPWMPeriod = TimerFreq_{CLK} / F_{PWM}@f$    */

    SignalFilteringHandle_t IaFilter;                  /* Pointer to filter instance used for filtering Ia signal (only for software ocp) */
    SignalFilteringHandle_t IbFilter;                  /* Pointer to filter instance used for filtering Ib signal (only for software ocp) */
    float fCurrentFilterAlpha;
    float fCurrentFilterBeta;

    int16_t hSoftwareOCPMarginCurrent;                   /* Measured current amplitude can be until hSoftwareOCPMarginCurrent higher
                                                            than reference current before overcurrent software protection triggers */
    int16_t hSoftwareOCPMaximumCurrent;                   /* Max current that can be reached before triggering software overcurrent */

};


/**
  * @brief Initialize PWM&Current feedback module.
  * @param  pHandle: handle on the target PWMC component
  * @retval true if initialization is successful
*/
bool PWMCurrFdbk_Init( PWMCurrFdbkHandle_t * pHandle);

/**
  * @brief Returns the phase current of the motor (in s16A unit).
	*					To call in order to compute Ia, Ib and Ic and store into handle.
  * @param  pHandle: handle on the target PWMC component
  * @param  Iab: Pointer to the structure that will receive motor current
  *         of phase A and B in s16A unit.
*/
void PWMCurrFdbk_GetPhaseCurrents(PWMCurrFdbkHandle_t * pHandle,
                            ab_t * Iab);

/**
  * @brief Returns the current phase a
  * @param  pHandle: handle on the target PWMC component
  * @retval  Ia in s16A unit
*/
static inline int16_t PWMCurrFdbk_GetIa(PWMCurrFdbkHandle_t * pHandle)
{
    return pHandle->Ia;
}

/**
  * @brief Returns the current phase b
  * @param  pHandle: handle on the target PWMC component
  * @retval  Ib in s16A unit
*/
static inline int16_t PWMCurrFdbk_GetIb(PWMCurrFdbkHandle_t * pHandle)
{
    return pHandle->Ib;
}

/**
  * @brief Returns the current phase c
  * @param  pHandle: handle on the target PWMC component
  * @retval  Ic in s16A unit
*/
static inline int16_t PWMCurrFdbk_GetIc(PWMCurrFdbkHandle_t * pHandle)
{
    return pHandle->Ic;
}

/**
  * @brief  Execute software overcurrent protection algorithm. Must be called periodically to update current filters.
  * @param  pHandle: handle on the target PWMC component
  * @param  Iab: Pointer to the structure that contains Ia and Ib
  * @param  Iqdref: Pointer to the structure that contains Idref and Iqref
  * @retval True if overcurrent condition, false otherwise.
*/
uint32_t PWMCurrFdbk_CheckSoftwareOverCurrent( PWMCurrFdbkHandle_t * pHandle, const ab_t * Iab, const qd_t * Iqdref);

/**
  * @brief  Converts input voltages @f$ V_{\alpha} @f$ and @f$ V_{\beta} @f$ into PWM duty cycles
  *         and update duty cycle registers.
  * @param  pHandle handler on the target PWMC component.
  * @param  Valfa_beta Voltage Components expressed in the @f$(\alpha, \beta)@f$ reference frame
  *
  * This function computes the time during which the transistors of each phase are to be switched on in
  * a PWM cycle in order to achieve the reference phase voltage set by @p Valfa_beta. Then, the function
  * programs the resulting duty cycles in the related timer channels. It also sets the phase current
  * sampling point for the next PWM cycle accordingly.
  *
  * This function is used in the FOC frequency loop and needs to complete before the next PWM cycle starts
  * so that the duty cycles it computes can be taken into account. Failing to do so (for instance because
  * the PWM Frequency is too high) results in the functions returning #MC_FOC_DURATION which entails a
  * motor Control fault that stops the motor.
  *
  * @retval Returns #MC_NO_ERROR if no error occurred or #MC_FOC_DURATION if the duty cycles were
  *         set too late for being taken into account in the next PWM cycle (overrun condition).
  */
uint32_t PWMCurrFdbk_SetPhaseVoltage(PWMCurrFdbkHandle_t * pHandle,
                               AlphaBeta_t Valfa_beta);

/**
  * @brief  Switches PWM generation off
	* @param  pHandle: Handle on the target instance of the PWMC component
  */
void PWMCurrFdbk_SwitchOffPWM(PWMCurrFdbkHandle_t * pHandle);

/**
  * @brief  Switches PWM generation on
	* @param  pHandle: Handle on the target instance of the PWMC component
  */
void PWMCurrFdbk_SwitchOnPWM(PWMCurrFdbkHandle_t * pHandle);

/**
  * @brief  Calibrates ADC current conversions by reading the offset voltage
  *         present on ADC pins when no motor current is flowing in. This function
	*					should be called before each motor start-up.
	* @param  pHandle: Handle on the target instance of the PWMC component
  * @retval true if the current calibration has been completed, false if it is
  *         still ongoing.
  */
bool PWMCurrFdbk_CurrentReadingCalibr(PWMCurrFdbkHandle_t * pHandle);

/**
  * @brief  Switches power stage low sides transistors on.
  * This function is meant for charging boot capacitors of the driving
  * section. It has to be called on each motor start-up.
  * @param  pHandle: handle on the target instance of the PWMC component
  */
void PWMCurrFdbk_TurnOnLowSides(PWMCurrFdbkHandle_t * pHandle);

/** @brief Check if hardware overcurrent occured since last call.
 *  @param  pHandle: Handle on the target instance of the PWMC component
 *	@retval Returns #MC_OCD1 or MC_OCD2 if an over current condition was detected on the power stage
 *         controlled by the PWMC component pointed by  @p pHandle, since the last call to this function;
 *         returns #MC_NO_FAULTS otherwise. */
uint32_t PWMCurrFdbk_CheckOverCurrent(PWMCurrFdbkHandle_t * pHandle);

/**
  * @brief  It is used to retrieve the status of TurnOnLowSides action.
  * @param  pHandle: handler of the current instance of the PWMC component
  * @retval bool It returns the state of TurnOnLowSides action:
  *         true if TurnOnLowSides action is active, false otherwise.
  */
bool PWMCurrFdbk_GetTurnOnLowSidesAction(PWMCurrFdbkHandle_t * pHandle);

/** @brief Enables the RL detection mode on the power stage controlled by the @p pHandle PWMC component.
	* @param  pHandle: Handle on the target instance of the PWMC component
*/
void PWMCurrFdbk_RLDetectionModeEnable(PWMCurrFdbkHandle_t * pHandle);

/** @brief Disables the RL detection mode on the power stage controlled by the @p pHandle PWMC component.
	* @param  pHandle: Handle on the target instance of the PWMC component
*/
void PWMCurrFdbk_RLDetectionModeDisable(PWMCurrFdbkHandle_t * pHandle);

/**
  * @brief  Sets the PWM duty cycle to apply in the RL Detection mode.
  * @param  pHandle: handle on the target instance of the PWMC component
  * @param  hDuty Duty cycle to apply
  *
  * @retval If the Duty Cycle could be applied on time for the next PWM period,
  *         #MC_NO_ERROR is returned. Otherwise, #MC_FOC_DURATION is returned.
  */
uint32_t PWMCurrFdbk_RLDetectionModeSetDuty(PWMCurrFdbkHandle_t * pHandle,
                                      uint16_t hDuty);

/**
 * @brief Sets the Callback that the PWMC component shall invoke to get phases current.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMCurrFdbk_RegisterGetPhaseCurrentsCallBack(PWMCurrFdbk_GetPhaseCurr_Cb_t pCallBack,
    PWMCurrFdbkHandle_t * pHandle);

/**
 * @brief Sets the Callback that the PWMC component shall invoke to switch PWM
 *        generation off.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMCurrFdbk_RegisterSwitchOffPwmCallBack(PWMCurrFdbk_Generic_Cb_t pCallBack,
                                        PWMCurrFdbkHandle_t * pHandle);

/**
 * @brief Sets the Callback that the PWMC component shall invoke to switch PWM
 *        generation on.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMCurrFdbk_RegisterSwitchonPwmCallBack(PWMCurrFdbk_Generic_Cb_t pCallBack,
                                       PWMCurrFdbkHandle_t * pHandle);

/**
 * @brief Sets the Callback that the PWMC component shall invoke to execute a calibration
 *        of the current sensing system.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMCurrFdbk_RegisterReadingCalibrationCallBack(PWMCurrFdbk_Generic_Cb_t pCallBack,
    PWMCurrFdbkHandle_t * pHandle);

/**
 * @brief Sets the Callback that the PWMC component shall invoke to turn low sides on.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMCurrFdbk_RegisterTurnOnLowSidesCallBack(PWMCurrFdbk_Generic_Cb_t pCallBack,
    PWMCurrFdbkHandle_t * pHandle);

/**
 * @brief Sets the Callback that the PWMC component shall invoke to compute ADC sampling point
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMCurrFdbk_RegisterSampPointSectXCallBack(PWMCurrFdbk_SetSampPointSectX_Cb_t pCallBack,
    PWMCurrFdbkHandle_t * pHandle);


/**
 * @brief Sets the Callback that the PWMC component shall invoke to the overcurrent status
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMCurrFdbk_RegisterIsOverCurrentOccurredCallBack(PWMCurrFdbk_OverCurr_Cb_t pCallBack,
    PWMCurrFdbkHandle_t * pHandle);


/**
 * @brief Sets the Callback that the PWMC component shall invoke to enable the R/L detection mode
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMCurrFdbk_RegisterRLDetectionModeEnableCallBack(PWMCurrFdbk_Generic_Cb_t pCallBack,
    PWMCurrFdbkHandle_t * pHandle);

/**
 * @brief Sets the Callback wIch PWMC shall invoke to disable the R/L detection mode
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 */
void PWMCurrFdbk_RegisterRLDetectionModeDisableCallBack(PWMCurrFdbk_Generic_Cb_t pCallBack,
    PWMCurrFdbkHandle_t * pHandle);

/**
 * @brief Sets the Callback that the PWMC component shall invoke to set the duty cycle
 *        for the R/L detection mode
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMCurrFdbk_RegisterRLDetectionModeSetDutyCallBack(PWMCurrFdbk_RLDetectSetDuty_Cb_t pCallBack,
    PWMCurrFdbkHandle_t * pHandle);


#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* PWMNCURRFDBK_H */
