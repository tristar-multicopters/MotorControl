/**
  * @file    sto_speed_pos_fdbk.h
  * @brief   This file contains definitions and functions prototypes common to all
  *          State Observer based Speed & Position Feedback components of the Motor
  *          Control application.
	*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STO_SPEEDNPOSFDBK_H
#define __STO_SPEEDNPOSFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "speed_pos_fdbk.h"


/* Exported types ------------------------------------------------------------*/

/** @brief PWM & Current Sensing component handle type */
typedef struct BemfObserverHandle BemfObserver_t;

typedef void (*BemfObs_ForceConvergency1_Cb_t)(BemfObserver_t * pHandle);
typedef void (*BemfObs_ForceConvergency2_Cb_t)(BemfObserver_t * pHandle);
typedef void (*BemfObs_OtfResetPLL_Cb_t)(BemfObserver_t * pHandle);
typedef bool (*BemfObs_SpeedReliabilityCheck_Cb_t)(const BemfObserver_t * pHandle);

/**
  * @brief  SpeednPosFdbk  handle definition
  */
struct BemfObserverHandle
{
  SpdPosFdbkHandle_t     *     Super;
  BemfObs_ForceConvergency1_Cb_t       pFctForceConvergency1;
  BemfObs_ForceConvergency2_Cb_t       pFctForceConvergency2;
  BemfObs_OtfResetPLL_Cb_t             pFctStoOtfResetPLL;
  BemfObs_SpeedReliabilityCheck_Cb_t   pFctBemfObserverSpeedReliabilityCheck;
};


#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*__STO_SPEEDNPOSFDBK_H*/

