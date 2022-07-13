/**
  * @file    sto_pll_speed_pos_fdbk.h
  * @brief   This file contains all definitions and functions prototypes for the
  *          State Observer + PLL Speed & Position Feedback module
*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STO_PLL_SPEEDNPOSFDBK_H
#define __STO_PLL_SPEEDNPOSFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "speed_pos_fdbk.h"
#include "sto_speed_pos_fdbk.h"
#include "pid_regulator.h"

/* Exported types ------------------------------------------------------------*/

/**
  * @brief This structure is used to handle an instance of the BemfObs_SpeednPosFdbk component
  *
  */

typedef struct
{
  SpeednPosFdbkHandle_t Super;

  int16_t  hC1;                 /*!< Variable containing state observer constant
                                     C1 to speed-up computations */
  int16_t  hC2;                /*!< Variable containing state observer constant
                                     C2, it can be computed as F1 * K1/ State
                                     observer execution rate [Hz] being K1 one
                                     of the two observer gains   */
  int16_t  hC3;                 /*!< Variable containing state observer constant
                                     C3 */
  int16_t  hC4;                  /*!< State Observer constant C4, it can
                                      be computed as K2 * max measurable
                                      current (A) / (Max application speed
                                      [#SPEED_UNIT] * Motor B-emf constant
                                      [Vllrms/krpm] * sqrt(2) * F2 * State
                                      observer execution rate [Hz]) being
                                       K2 one of the two observer gains  */
  int16_t  hC5;                 /*!< Variable containing state observer constant
                                     C5 */
  int16_t  hC6;                 /*!< State observer constant C6, computed with a
                                    specific procedure starting from the other
                                    constants */
  int16_t  hF1;                 /*!< Variable containing state observer scaling
                                     factor F1 */
  int16_t  hF2;                 /*!< Variable containing state observer scaling factor F2*/
  int16_t  hF3;                  /*!< State observer scaling factor F3 */
  uint16_t hF3Pow2;              /*!< State observer scaling factor F3 expressed as power of 2.
                                     E.g. if gain divisor is 512 the value
                                     must be 9 because 2^9 = 512 */
  PIDHandle_t PIRegulator;     /*!< PI regulator component handle, used for
                                     PLL implementation */
  int32_t wIalfaEst;           /*!< Estimated Ialfa current in int32 format */
  int32_t wIbetaEst;           /*!< Estimated Ibeta current in int32 format */
  int32_t wBemfalfaEst;       /*!< Estimated B-emf alfa in int32_t format */
  int32_t wBemfbetaEst;       /*!< Estimated B-emf beta in int32_t format */
  int16_t hBemfalfaEst;       /*!< Estimated B-emf alfa in int16_t format */
  int16_t hBemfbetaEst;       /*!< Estimated B-emf beta in int16_t format */
  int16_t SpeedBuffer[64];    /*!< Estimated DPP speed FIFO, it contains latest
                                     bSpeedBufferSizeDpp speed measurements*/
  uint8_t bSpeedBufferIndex;  /*!< Position of latest speed estimation in
                                     estimated speed FIFO */
  bool bIsSpeedReliable;        /*!< Latest private speed reliability information,
                                     updated by SPD_CalcAvrgMecSpeedUnit, it is
                                     true if the speed measurement variance is
                                     lower then threshold corresponding to
                                     hVariancePercentage */
  uint8_t bConsistencyCounter;  /*!< Counter of passed tests for start-up
                                     validation */
  uint8_t bReliabilityCounter; /*!< Counter for reliability check */
  bool bIsAlgorithmConverged;   /*!< Boolean variable containing observer
                                     convergence information */
  bool bIsBemfConsistent;       /*!< Sensor reliability information, updated by
                                     SPD_CalcAvrgMecSpeedUnit, it is true if the
                                     observed back-emfs are consistent with
                                     expectation*/

  int32_t wObsBemfLevel;      /*!< Observed back-emf Level*/
  int32_t wEstBemfLevel;      /*!< Estimated back-emf Level*/
  bool bEnableDualCheck;        /*!< Consistency check enabler*/
  int32_t wDppBufferSum;        /*!< summation of speed buffer elements [dpp]*/
  int16_t hSpeedBufferOldestEl; /*!< Oldest element of the speed buffer*/

  uint8_t bSpeedBufferSizeUnit;       /*!< Depth of FIFO used to average
                                           estimated speed exported by
                                           SpdPosFdbk_GetAvrgMecSpeedUnit. It
                                           must be an integer number between 1
                                           and 64 */
  uint8_t bSpeedBufferSizeDpp;       /*!< Depth of FIFO used for both averaging
                                           estimated speed exported by
                                           SpdPosFdbk_GetElSpeedDpp and state
                                           observer equations. It must be an
                                           integer number between 1 and
                                           bSpeedBufferSizeUnit */
  uint16_t hVariancePercentage;        /*!< Parameter expressing the maximum
                                           allowed variance of speed estimation
                                           */
  uint8_t bSpeedValidationBandHigh;   /*!< It expresses how much estimated speed
                                           can exceed forced stator electrical
                                           frequency during start-up without
                                           being considered wrong. The
                                           measurement unit is 1/16 of forced
                                           speed */
  uint8_t bSpeedValidationBandLow;   /*!< It expresses how much estimated speed
                                           can be below forced stator electrical
                                           frequency during start-up without
                                           being considered wrong. The
                                           measurement unit is 1/16 of forced
                                           speed */
  uint16_t hMinStartUpValidSpeed;     /*!< Absolute value of minimum mechanical
                                            speed  required to validate the start-up.
                                            Expressed in the unit defined by #SPEED_UNIT. */
  uint8_t bStartUpConsistThreshold;   /*!< Number of consecutive tests on speed
                                           consistency to be passed before
                                           validating the start-up */
  uint8_t bReliabilityHysteresys;    /*!< Number of reliability failed
                                           consecutive tests before a speed
                                           check fault is returned to Super.bSpeedErrorNumber
                                           */
  uint8_t bBemfConsistencyCheck;      /*!< Degree of consistency of the observed
                                           back-emfs, it must be an integer
                                           number ranging from 1 (very high
                                           consistency) down to 64 (very poor
                                           consistency) */
  uint8_t bBemfConsistencyGain;       /*!< Gain to be applied when checking
                                           back-emfs consistency; default value
                                           is 64 (neutral), max value 105
                                           (x1.64 amplification), min value 1
                                           (/64 attenuation) */
  uint16_t hMaxAppPositiveMecSpeedUnit; /*!< Maximum positive value of rotor speed. Expressed in
                                             the unit defined by #SPEED_UNIT. It can be
                                             x1.1 greater than max application speed*/
  uint16_t hF1Log;                    /*!< F1 gain divisor expressed as power of 2.
                                            E.g. if gain divisor is 512 the value
                                            must be 9 because 2^9 = 512 */
  uint16_t hF2Log;                    /*!< F2 gain divisor expressed as power of 2.
                                            E.g. if gain divisor is 512 the value
                                            must be 9 because 2^9 = 512 */
  uint16_t hSpeedBufferSizeDppLog;    /*!< bSpeedBufferSizedpp expressed as power of 2.
                                            E.g. if gain divisor is 512 the value
                                            must be 9 because 2^9 = 512 */
  bool bForceConvergency;       /*!< Variable to force observer convergence.*/
  bool bForceConvergency2;      /*!< Variable to force observer convergence.*/

  int8_t hForcedDirection;
  
} BemfObserverPllHandle_t;


/* Exported functions ------------------------------------------------------- */

/**
  * @brief  It initializes the state observer component
  * @param  pHandle: handler of the current instance of the BemfObserver component
  * @retval none
  */
void BemfObsPll_Init(BemfObserverPllHandle_t * pHandle);

/**
  * @brief  It only returns, necessary to implement fictitious IRQ_Handler
  * @param  pHandle: handler of the current instance of the BemfObserver component
  * @param  uint8_t Fictitious interrupt flag
  * @retval none
  */
void BemfObsPll_Return(BemfObserverPllHandle_t * pHandle, uint8_t flag);

/**
  * @brief  It clears state observer component by re-initializing private variables
  * @param  pHandle related object of class STO
  * @retval none
  */
void BemfObsPll_Clear(BemfObserverPllHandle_t * pHandle);

/**
  * @brief  This method executes Luenberger state observer equations and calls
  *         PLL with the purpose of computing a new speed estimation and
  *         updating the estimated electrical angle.
  * @param  pHandle: handler of the current instance of the BemfObserver component
  * @param  pInputVars_str pointer to the observer inputs structure
  * @retval int16_t rotor electrical angle (s16Degrees)
  */
int16_t BemfObsPll_CalcElAngle(BemfObserverPllHandle_t * pHandle, BemfObserverInputs_t * pInputVars_str);

/**
  * @brief  This method must be called - at least - with the same periodicity
  *         on which speed control is executed. It computes and returns - through
  *         parameter pMecSpeedUnit - the rotor average mechanical speed,
  *         expressed in Unit. Average is computed considering a FIFO depth
  *         equal to bSpeedBufferSizeUnit. Moreover it also computes and returns
  *         the reliability state of the sensor.
  * @param  pHandle: handler of the current instance of the BemfObserver component
  * @param  pMecSpeedUnit pointer to int16_t, used to return the rotor average
  *         mechanical speed (expressed in the unit defined by #SPEED_UNIT)
  * @retval bool speed sensor reliability, measured with reference to parameters
  *         bReliability_hysteresys, hVariancePercentage and bSpeedBufferSize
  *         true = sensor information is reliable
  *         false = sensor information is not reliable
  */
bool BemfObsPll_CalcAvrgMecSpeedUnit(BemfObserverPllHandle_t * pHandle, int16_t * pMecSpeedUnit);

/**
  * @brief  It resets integral term of PLL during on-the-fly startup
  * @param  pHandle: handler of the current instance of the BemfObserver component
  * @retval none
  */
void BemfObsPll_OtfResetPLL(BemfObserver_t * pHandle);

/**
  * @brief  It resets integral term of PLL
  * @param  pHandle: handler of the current instance of the BemfObserver component
  * @retval none
  */
void BemfObsPll_ResetPLL(BemfObserverPllHandle_t * pHandle);

/**
  * @brief  It internally performs a set of checks necessary to state whether
  *         the state observer algorithm converged. To be periodically called
  *         during motor open-loop ramp-up (e.g. at the same frequency of
  *         SPD_CalcElAngle), it returns true if the estimated angle and speed
  *         can be considered reliable, false otherwise
  * @param  pHandle: handler of the current instance of the BemfObserver component
  * @param  hForcedMecSpeedUnit Mechanical speed in 0.1Hz unit as forced by VSS
  * @retval bool sensor reliability state
  */
bool BemfObsPll_IsObserverConverged(BemfObserverPllHandle_t * pHandle, int16_t hForcedMecSpeedUnit);

/**
  * @brief  This method must be called - at least - with the same periodicity
  *         on which speed control is executed. It computes and update component
  *         variable hElSpeedDpp that is estimated average electrical speed
  *         expressed in dpp used for instance in observer equations.
  *         Average is computed considering a FIFO depth equal to
  *         bSpeedBufferSizedpp.
  * @param  pHandle: handler of the current instance of the BemfObserver component
  * @retval none
  */
void BemfObsPll_CalcAvrgElSpeedDpp(BemfObserverPllHandle_t * pHandle);

/**
  * @brief  It exports estimated Bemf alpha-beta in AlphaBeta_t format
  * @param  pHandle: handler of the current instance of the BemfObserver component
  * @retval AlphaBeta_t Bemf alpha-beta
  */
AlphaBeta_t BemfObsPll_GetEstimatedBemf(BemfObserverPllHandle_t * pHandle);

/**
  * @brief  It exports the stator current alpha-beta as estimated by state
  *         observer
  * @param  pHandle: handler of the current instance of the BemfObserver component
  * @retval AlphaBeta_t State observer estimated stator current Ialpha-beta
  */
AlphaBeta_t BemfObsPll_GetEstimatedCurrent(BemfObserverPllHandle_t * pHandle);

/**
  * @brief  It allows setting new values for observer gains
  * @param  pHandle: handler of the current instance of the BemfObserver component
  * @param  wK1 new value for observer gain hhC1
  * @param  wK2 new value for observer gain hhC2
  * @retval none
  */
void BemfObsPll_SetObserverGains(BemfObserverPllHandle_t * pHandle, int16_t hC1, int16_t hC2);

/**
  * @brief  It exports current observer gains through parameters hhC2 and hhC4
  * @param  pHandle: handler of the current instance of the BemfObserver component
  * @param  phC2 pointer to int16_t used to return parameters hhC2
  * @param  phC4 pointer to int16_t used to return parameters hhC4
  * @retval none
  */
void BemfObsPll_GetObserverGains(BemfObserverPllHandle_t * pHandle, int16_t * pC2, int16_t * pC4);

/**
  * @brief  It exports current PLL gains through parameters pPgain and pIgain
  * @param  pHandle: handler of the current instance of the BemfObserver component
  * @param  pPgain pointer to int16_t used to return PLL proportional gain
  * @param  pIgain pointer to int16_t used to return PLL integral gain
  * @retval none
  */
void BemfObs_GetPLLGains(BemfObserverPllHandle_t * pHandle, int16_t * pPgain, int16_t * pIgain);

/**
  * @brief  It allows setting new values for PLL gains
  * @param  pHandle: handler of the current instance of the BemfObserver component
  * @param  hPgain new value for PLL proportional gain
  * @param  hIgain new value for PLL integral gain
  * @retval none
  */
void BemfObsPll_SetPLLGains(BemfObserverPllHandle_t * pHandle, int16_t hPgain, int16_t hIgain);

void BemfObsPll_SetMecAngle(BemfObserverPllHandle_t * pHandle, int16_t hMecAngle);

/**
  * @brief  It sends locking info for PLL
  * @param  pHandle: handler of the current instance of the BemfObserver component
  * @param  hElSpeedDpp:
  * @param  hElAngle:
  * @retval none
  */
void BemfObsPll_SetPLL(BemfObserverPllHandle_t * pHandle, int16_t hElSpeedDpp, int16_t hElAngle);

/**
  * @brief  It exports estimated Bemf squared level
  * @param  pHandle: handler of the current instance of the BemfObserver component
  * @retval int32_t
  */
int32_t BemfObsPll_GetEstimatedBemfLevel(BemfObserverPllHandle_t * pHandle);

/**
  * @brief  It exports observed Bemf squared level
  * @param  pHandle: handler of the current instance of the BemfObserver component
  * @retval int32_t
  */
int32_t BemfObsPll_GetObservedBemfLevel(BemfObserverPllHandle_t * pHandle);

/**
  * @brief  It enables/disables the bemf consistency check
  * @param  pHandle: handler of the current instance of the BemfObserver component
  * @param  bSel boolean; true enables check; false disables check
  */
void BemfObsPll_BemfConsistencyCheckSwitch(BemfObserverPllHandle_t * pHandle, bool bSel);

/**
  * @brief  It returns the result of the Bemf consistency check
  * @param  pHandle: handler of the current instance of the BemfObserver component
  * @retval bool Bemf consistency state
  */
bool BemfObsPll_IsBemfConsistent(BemfObserverPllHandle_t * pHandle);

/**
  * @brief  It returns the result of the last variance check
  * @param  pHandle: handler of the current instance of the BemfObserver component
  * @retval bool Variance state
  */
bool BemfObsPll_IsVarianceTight(const BemfObserver_t * pHandle);

/**
  * @brief  It forces the state-observer to declare convergency
  * @param  pHandle: handler of the current instance of the BemfObserver component
  */
void BemfObsPll_ForceConvergency1(BemfObserver_t * pHandle);

/**
  * @brief  It forces the state-observer to declare convergency
  * @param  pHandle: handler of the current instance of the BemfObserver component
  */
void BemfObsPll_ForceConvergency2(BemfObserver_t * pHandle);

/**
  * @brief  Set the Absolute value of minimum mechanical speed (expressed in
  *         the unit defined by #SPEED_UNIT) required to validate the start-up.
  * @param  pHandle: handler of the current instance of the BemfObserver component
  * @param  hMinStartUpValidSpeed: Absolute value of minimum mechanical speed
  */
void BemfObsPll_SetMinStartUpValidSpeedUnit(BemfObserverPllHandle_t * pHandle, uint16_t hMinStartUpValidSpeed);

/**
  * @brief  forces the rotation direction
  * @param  direction: imposed direction
  */
void BemfObsPll_SetDirection(BemfObserverPllHandle_t * pHandle, uint8_t direction);


#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*__STO_PLL_SPEEDNPOSFDBK_H*/

