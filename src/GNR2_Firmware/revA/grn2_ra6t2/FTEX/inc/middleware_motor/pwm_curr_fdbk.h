/**
  ******************************************************************************
  * @file    pwm_curr_fdbk.h
  * @author  Sami Bouzid, FTEX inc
  * @brief   This file provides firmware functions that implement the following features
  *          of the PWM & Current Feedback component:
  *
  *           * current sensing
  *           * regular ADC conversion execution
  *           * space vector modulation
  *
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PWMNCURRFDBK_H
#define PWMNCURRFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"

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
typedef struct PWMC_Handle PWMC_Handle_t;

/**
  * @brief Pointer on callback functions used by PWMC components
  *
  * This type is needed because the actual functions to use can change at run-time.
  *
  * See the following items:
  * - PWMC_Handle::pFctSwitchOffPwm
  * - PWMC_Handle::pFctSwitchOnPwm
  * - PWMC_Handle::pFctCurrReadingCalib
  * - PWMC_Handle::pFctTurnOnLowSides
  * - PWMC_Handle::pFctRLDetectionModeEnable
  * - PWMC_Handle::pFctRLDetectionModeDisable
  *
  */
typedef void ( *PWMC_Generic_Cb_t )( PWMC_Handle_t * pHandle );

/**
  * @brief Pointer on the interrupt handling function of the PWMC component instance.
  *
  * This type is needed because the actual function to use can change at run-time
  * (See PWMC_Handle::pFctIrqHandler).
  *
  */
typedef void * ( *PWMC_IrqHandler_Cb_t )( PWMC_Handle_t * pHandle, unsigned char flag );

/**
  * @brief Pointer on the function provided by the PMWC component instance to get the phase current.
  *
  * This type is needed because the actual function to use can change at run-time
  * (See PWMC_Handle::pFctGetPhaseCurrents).
  *
  */
typedef void ( *PWMC_GetPhaseCurr_Cb_t )( PWMC_Handle_t * pHandle, ab_t * Iab );

/**
  * @brief Pointer on the functions provided by the PMWC component instance to set the ADC sampling
  *        point for each sectors.
  *
  * This type is needed because the actual function to use can change at run-time. See:
  * - PWMC_Handle::pFctSetADCSampPointSect1
  * - PWMC_Handle::pFctSetADCSampPointSect2
  * - PWMC_Handle::pFctSetADCSampPointSect3
  * - PWMC_Handle::pFctSetADCSampPointSect4
  * - PWMC_Handle::pFctSetADCSampPointSect5
  * - PWMC_Handle::pFctSetADCSampPointSect6
  *
  */
typedef uint16_t ( *PWMC_SetSampPointSectX_Cb_t )( PWMC_Handle_t * pHandle);

/**
  * @brief Pointer on the function provided by the PMWC component instance to check if an over current
  *        condition has occured.
  *
  * This type is needed because the actual function to use can change at run-time
  * (See PWMC_Handle::pFctIsOverCurrentOccurred).
  *
  */
typedef uint16_t ( *PWMC_OverCurr_Cb_t )( PWMC_Handle_t * pHandle );

/**
  * @brief Pointer on the function provided by the PMWC component instance to set the PWM duty cycle
  *        in RL detection mode.
  *
  * This type is needed because the actual function to use can change at run-time
  * (See PWMC_Handle::pFctRLDetectionModeSetDuty).
  *
  */
typedef uint16_t ( *PWMC_RLDetectSetDuty_Cb_t )( PWMC_Handle_t * pHandle, uint16_t hDuty );

/**
  * @brief This structure is used to handle the data of an instance of the PWM & Current Feedback component
  *
  */
struct PWMC_Handle
{
  /** @{ */
  PWMC_IrqHandler_Cb_t pFctIrqHandler;                   /**< pointer on the interrupt handling function. */
  PWMC_GetPhaseCurr_Cb_t
  pFctGetPhaseCurrents;           /**< pointer on the function the component instance uses to retrieve pahse currents */
  PWMC_Generic_Cb_t
  pFctSwitchOffPwm;                    /**< pointer on the function the component instance uses to switch PWM off */
  PWMC_Generic_Cb_t
  pFctSwitchOnPwm;                     /**< pointer on the function the component instance uses to switch PWM on */
  PWMC_Generic_Cb_t
  pFctCurrReadingCalib;                /**< pointer on the function the component instance uses to calibrate the current reading ADC(s) */
  PWMC_Generic_Cb_t
  pFctTurnOnLowSides;                  /**< pointer on the function the component instance uses to turn low sides on */
  PWMC_SetSampPointSectX_Cb_t
  pFctSetADCSampPointSectX;  /**< pointer on the function the component instance uses to set the ADC sampling point  */
  PWMC_OverCurr_Cb_t
  pFctIsOverCurrentOccurred;          /**< pointer on the function the component instance uses to return the over current status */
  PWMC_Generic_Cb_t
  pFctRLDetectionModeEnable;           /**< pointer on the function the component instance uses to enable RL detection mode */
  PWMC_Generic_Cb_t
  pFctRLDetectionModeDisable;          /**< pointer on the function the component instance uses to disable RL detection mode */
  PWMC_RLDetectSetDuty_Cb_t
  pFctRLDetectionModeSetDuty;  /**< pointer on the function the component instance uses to set the PWM duty cycle in RL detection mode */
  /** @} */
  uint16_t  hT_Sqrt3;                                    /**< a constant utilized by PWM algorithm (@f$\sqrt{3}@f$) */
  uint16_t  CntPhA;                                     /**< PWM Duty cycle for phase A */
  uint16_t  CntPhB;                                     /**< PWM Duty cycle for phase B */
  uint16_t  CntPhC;                                     /**< PWM Duty cycle for phase C */
  uint16_t  SWerror;                                     /**< Contains status about SW error */
  uint8_t   Sector;                                     /**< the space vector sector number */
  uint16_t  lowDuty;
  uint16_t  midDuty;
  uint16_t  highDuty;
  bool TurnOnLowSidesAction;                            /**< true if TurnOnLowSides action is active,
                                                              false otherwise. */
  uint8_t   Motor;                                      /**< Motor reference number */
  bool      RLDetectionMode;                             /**< true if enabled, false if disabled. */
  int16_t   Ia;                                         /**< Last @f$I_{A}@f$ measurement. */
  int16_t   Ib;                                         /**< Last @f$I_{B}@f$ measurement. */
  int16_t   Ic;                                         /**< Last @f$I_{C}@f$ measurement. */
 
  uint16_t PWMperiod;                                   /**< PWM period expressed in timer clock cycles unit:
                                                           *  @f$hPWMPeriod = TimerFreq_{CLK} / F_{PWM}@f$    */
};

/**
  * @brief Returns the phase current of the motor (in s16A unit)
  * @param  pHandle: handle on the target PWMC component
  * @param  Iab: Pointer to the structure that will receive motor current
  *         of phase A and B in s16A unit.
*/
void PWMC_GetPhaseCurrents( PWMC_Handle_t * pHandle,
                            ab_t * Iab );

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
uint16_t PWMC_SetPhaseVoltage( PWMC_Handle_t * pHandle,
                               alphabeta_t Valfa_beta );

/**
  * @brief  Switches PWM generation off
	* @param  pHandle: Handle on the target instance of the PWMC component
  */
void PWMC_SwitchOffPWM( PWMC_Handle_t * pHandle );

/**
  * @brief  Switches PWM generation on
	* @param  pHandle: Handle on the target instance of the PWMC component
  */
void PWMC_SwitchOnPWM( PWMC_Handle_t * pHandle );

/**
  * @brief  Calibrates ADC current conversions by reading the offset voltage
  *         present on ADC pins when no motor current is flowing in. This function
	*					should be called before each motor start-up.
	* @param  pHandle: Handle on the target instance of the PWMC component
  * @retval true if the current calibration has been completed, false if it is
  *         still ongoing.
  */
bool PWMC_CurrentReadingCalibr( PWMC_Handle_t * pHandle );

/**
  * @brief  Switches power stage low sides transistors on.
  * This function is meant for charging boot capacitors of the driving
  * section. It has to be called on each motor start-up.
  * @param  pHandle: handle on the target instance of the PWMC component
  */
void PWMC_TurnOnLowSides( PWMC_Handle_t * pHandle );

/** @brief Check if overcurrent occured since last call.
 *  @param  pHandle: Handle on the target instance of the PWMC component
 *	@retval Returns #MC_BREAK_IN if an over current condition was detected on the power stage
 *         controlled by the PWMC component pointed by  @p pHandle, since the last call to this function;
 *         returns #MC_NO_FAULTS otherwise. */
uint16_t PWMC_CheckOverCurrent( PWMC_Handle_t * pHandle );

/**
  * @brief  It is used to retrieve the status of TurnOnLowSides action.
  * @param  pHandle: handler of the current instance of the PWMC component
  * @retval bool It returns the state of TurnOnLowSides action:
  *         true if TurnOnLowSides action is active, false otherwise.
  */
bool PWMC_GetTurnOnLowSidesAction( PWMC_Handle_t * pHandle );

/** @brief Enables the RL detection mode on the power stage controlled by the @p pHandle PWMC component.
	* @param  pHandle: Handle on the target instance of the PWMC component
*/
void PWMC_RLDetectionModeEnable( PWMC_Handle_t * pHandle );

/** @brief Disables the RL detection mode on the power stage controlled by the @p pHandle PWMC component. 
	* @param  pHandle: Handle on the target instance of the PWMC component
*/
void PWMC_RLDetectionModeDisable( PWMC_Handle_t * pHandle );

/**
  * @brief  Sets the PWM duty cycle to apply in the RL Detection mode.
  * @param  pHandle: handle on the target instance of the PWMC component
  * @param  hDuty Duty cycle to apply
  *
  * @retval If the Duty Cycle could be applied on time for the next PWM period,
  *         #MC_NO_ERROR is returned. Otherwise, #MC_FOC_DURATION is returned.
  */
uint16_t PWMC_RLDetectionModeSetDuty( PWMC_Handle_t * pHandle,
                                      uint16_t hDuty );

/**
 * @brief Sets the Callback that the PWMC component shall invoke to get phases current.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterGetPhaseCurrentsCallBack( PWMC_GetPhaseCurr_Cb_t pCallBack,
    PWMC_Handle_t * pHandle );

/**
 * @brief Sets the Callback that the PWMC component shall invoke to switch PWM
 *        generation off.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterSwitchOffPwmCallBack( PWMC_Generic_Cb_t pCallBack,
                                        PWMC_Handle_t * pHandle );

/**
 * @brief Sets the Callback that the PWMC component shall invoke to switch PWM
 *        generation on.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterSwitchonPwmCallBack( PWMC_Generic_Cb_t pCallBack,
                                       PWMC_Handle_t * pHandle );

/**
 * @brief Sets the Callback that the PWMC component shall invoke to execute a calibration
 *        of the current sensing system.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterReadingCalibrationCallBack( PWMC_Generic_Cb_t pCallBack,
    PWMC_Handle_t * pHandle );

/**
 * @brief Sets the Callback that the PWMC component shall invoke to turn low sides on.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterTurnOnLowSidesCallBack( PWMC_Generic_Cb_t pCallBack,
    PWMC_Handle_t * pHandle );

/**
 * @brief Sets the Callback that the PWMC component shall invoke to compute ADC sampling point
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterSampPointSectXCallBack( PWMC_SetSampPointSectX_Cb_t pCallBack,
    PWMC_Handle_t * pHandle );


/**
 * @brief Sets the Callback that the PWMC component shall invoke to the overcurrent status
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterIsOverCurrentOccurredCallBack( PWMC_OverCurr_Cb_t pCallBack,
    PWMC_Handle_t * pHandle );


/**
 * @brief Sets the Callback that the PWMC component shall invoke to enable the R/L detection mode
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterRLDetectionModeEnableCallBack( PWMC_Generic_Cb_t pCallBack,
    PWMC_Handle_t * pHandle );

/**
 * @brief Sets the Callback wIch PWMC shall invoke to disable the R/L detection mode
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 */
void PWMC_RegisterRLDetectionModeDisableCallBack( PWMC_Generic_Cb_t pCallBack,
    PWMC_Handle_t * pHandle );

/**
 * @brief Sets the Callback that the PWMC component shall invoke to set the duty cycle
 *        for the R/L detection mode
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterRLDetectionModeSetDutyCallBack( PWMC_RLDetectSetDuty_Cb_t pCallBack,
    PWMC_Handle_t * pHandle );


#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* PWMNCURRFDBK_H */
