/**
  * @file    feed_forward_ctrl.h
  * @brief   This file contains all definitions and functions prototypes for the
  *          Feed Forward Control component of the Motor Control application.
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FEEDFORWARDCTRL_H
#define __FEEDFORWARDCTRL_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "bus_voltage_sensor.h"
#include "speed_pos_fdbk.h"
#include "speed_torq_ctrl.h"

/**
  * @brief Handle structure of the Feed Forward Component
  */
typedef struct
{
  qd_t   Vqdff;                 /**< Feed Forward controller @f$I_{qd}@f$ contribution to @f$V_{qd}@f$ */
  qd_t   VqdPIout;              /**< @f$V_{qd}@f$ as output by PID controller */
  qd_t   VqdAvPIout;            /**< Averaged @f$V_{qd}@f$ as output by PID controller */
  int32_t  wConstant1D;                   /**< Feed forward default constant for the @f$d@f$ axis */
  int32_t  wConstant1Q;                   /**< Feed forward default constant for the @f$q@f$ axis */
  int32_t  wConstant2;                    /**< Default constant value used by Feed-Forward algorithm */
  BusVoltageSensorHandle_t * pBusSensor; /**< Related bus voltage sensor */
  PIDHandle_t * pPIDq;                   /*!< Related PI for @f$I_{q}@f$ regulator */
  PIDHandle_t * pPIDd;                   /*!< Related PI for @f$I_{d}@f$ regulator */
  uint16_t hVqdLowPassFilterBw;            /**< Configures the @f$V_{qd}@f$
                                                first order software filter bandwidth.

                                                If #FULL_MISRA_COMPLIANCY is defined, this field must be
                                                set to
                                                @f[
                                                \frac{FOC_CurrController
                                                call rate [Hz]}{FilterBandwidth[Hz]}
                                                @f]
                                                Otherwise, it must be set to:
                                                @f[
                                                log_2(\frac{FOC_CurrController
                                                call rate [Hz]}{FilterBandwidth[Hz]})
                                                @f]
                                                */
  int32_t  wDefConstant1D;                /**< Feed forward default constant for d axes */
  int32_t  wDefConstant1Q;                /**< Feed forward default constant for q axes */
  int32_t  wDefConstant2;                 /**< Default constant value used by
                                                Feed-Forward algorithm*/
  uint16_t hVqdLowPassFilterBwLog;         /**< hVqdLowPassFilterBw expressed as power of 2.
                                                E.g. if gain divisor is 512 the value
                                                must be 9 because 2^9 = 512 */

} FeedforwardHandle_t;


/**
  * @brief  Initializes all the component variables
  * @param  pHandle Feed forward init strutcture.
  * @param  pBusSensor VBus Sensor.
  * @param  pPIDId Id PID.
  * @param  pPIDIq Iq PID.
  * @retval none
  */
void Feedforward_Init( FeedforwardHandle_t * pHandle, BusVoltageSensorHandle_t * pBusSensor, PIDHandle_t * pPIDId,
              PIDHandle_t * pPIDIq );

/**
  * @brief  It should be called before each motor restart and clears the Flux
  *         weakening internal variables.
  * @param  pHandle Feed forward init strutcture.
  * @retval none
  */
void Feedforward_Clear( FeedforwardHandle_t * pHandle );

/**
  * @brief  It implements feed-forward controller by computing new Vqdff value.
  *         This will be then summed up to PI output in IMFF_VqdConditioning
  *         method.
  * @param  pHandle Feed forward  strutcture.
  * @param  Iqdref Idq reference componets used to calcupate the feed forward
  *         action.
  * @param  pSpeedTorqCtrl  Speed sensor.
  * @retval none
  */
void Feedforward_VqdffComputation( FeedforwardHandle_t * pHandle, qd_t Iqdref, SpeednTorqCtrlHandle_t * pSpeedTorqCtrl );

/**
  * @brief  It return the Vqd componets fed in input plus the feed forward
  *         action and store the last Vqd values in the internal variable.
  * @param  pHandle Feed forward  strutcture.
  * @param  Vqd Initial value of Vqd to be manipulated by FF.
  * @retval none
  */
qd_t Feedforward_VqdConditioning( FeedforwardHandle_t * pHandle, qd_t Vqd );

/**
  * @brief  It low-pass filters the Vqd voltage coming from the speed PI. Filter
  *         bandwidth depends on hVqdLowPassFilterBw parameter.
  * @param  pHandle Feed forward  strutcture.
  * @retval none
  */
void Feedforward_DataProcess( FeedforwardHandle_t * pHandle );

/**
  * @brief  Use this method to initialize FF vars in START_TO_RUN state.
  * @param  pHandle Feed forward  strutcture.
  * @retval none
  */
void Feedforward_InitFOCAdditionalMethods( FeedforwardHandle_t * pHandle );

/**
  * @brief  Use this method to set new values for the constants utilized by
  *         feed-forward algorithm.
  * @param  pHandle Feed forward  strutcture.
  * @param  sNewConstants The FeedforwardTuningStruct_t containing constants utilized by
  *         feed-forward algorithm.
  * @retval none
  */
void Feedforward_SetFFConstants( FeedforwardHandle_t * pHandle, FeedforwardTuningStruct_t sNewConstants );

/**
  * @brief  Use this method to get present values for the constants utilized by
  *         feed-forward algorithm.
  * @param  pHandle Feed forward  strutcture.
  * @retval FeedforwardTuningStruct_t Values of the constants utilized by
  *         feed-forward algorithm.
  */
FeedforwardTuningStruct_t Feedforward_GetFFConstants( FeedforwardHandle_t * pHandle );

/**
  * @brief  Use this method to get present values for the Vqd feed-forward
  *         components.
  * @param  pHandle Feed forward  strutcture.
  * @retval qd_t Vqd feed-forward components.
  */
qd_t Feedforward_GetVqdff( const FeedforwardHandle_t * pHandle );

/**
  * @brief  Use this method to get values of the averaged output of qd axes
  *         currents PI regulators.
  * @param  pHandle Feed forward  strutcture.
  * @retval qd_t Averaged output of qd axes currents PI regulators.
  */
qd_t Feedforward_GetVqdAvPIout( const FeedforwardHandle_t * pHandle );

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __FEEDFORWARDCTRL_H */

