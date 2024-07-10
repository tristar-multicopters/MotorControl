/**
  * @file    pwm_common.h
  * @author  Sami Bouzid, FTEX inc
  * @brief   This file provides firmware functions that implement common features
  *          of the PWM & Current Feedback component.
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PWMNCOMMON_H
#define __PWMNCOMMON_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "uCAL_GPIO.h"
#include "board_hardware.h"
#define NB_CONVERSIONS 16u


/* Exported functions --------------------------------------------------------*/

/**
 * @brief  It waits for the end of the polarization, i.e. when the counter value reach NB_CONVERSIONS
 * @param  cnt: polarization counter value
 * @retval none
 */
void waitForPolarizationEnd(volatile uint8_t *cnt);

/**
 * @brief  It disables the driver for the GANs
 * @param  Driver enable flag
 * @retval none
 */
void Driver_Disable(bool *bDriverEn);

/**
 * @brief  It enables the driver for the GANs
 * @param  Driver enable flag
 * @retval none
 */
void Driver_Enable(bool *bDriverEn);



#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __PWMNCOMMON_H */

