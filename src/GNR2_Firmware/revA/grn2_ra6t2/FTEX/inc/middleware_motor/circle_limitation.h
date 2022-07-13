/**
  * @file    circle_limitation.h
  * @brief   This file contains all definitions and functions prototypes for the
  *          Circle Limitation component of the Motor Control application.
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CIRCLELIMITATION_H
#define __CIRCLELIMITATION_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"


typedef struct
{
  uint16_t hMaxModule;               /**<  Circle limitation maximum allowed
                                         module */
  uint16_t hMaxVd;                   /**<  Circle limitation maximum allowed
                                         module */
  uint16_t CircleLimitTable[87];  /**<  Circle limitation table */
  uint8_t  bStartIndex;             /**<  Circle limitation table indexing
                                         start */
} CircleLimitationHandle_t;

/* Exported functions ------------------------------------------------------- */

/**
  * @brief Check whether Vqd.q^2 + Vqd.d^2 <= 32767^2
  *        and if not it applies a limitation keeping constant ratio
  *        Vqd.q / Vqd.d
  * @param  pHandle pointer on the related component instance
  * @param  Vqd Voltage in qd reference frame
  * @retval qd_t Limited Vqd vector
  */
qd_t CircleLimitation(CircleLimitationHandle_t * pHandle, qd_t Vqd);


#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __Circle Limitation_H */

