/**
  ******************************************************************************
  * @file    dynamic_power.h
  * @author  Behnam Shakibafar, FTEX inc
  * @brief   This file contains all definitions and functions prototypes for the
  *          Dynamic Power component of the Motor Control application.
  *
  ******************************************************************************
*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DYNAMICPOWER_H
#define __DYNAMICPOWER_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "ASSERT_FTEX.h"
#include "stdlib.h"

/* Exported types ------------------------------------------------------------*/

typedef struct
{
    uint16_t  hDynamicMaxPower;         /* new maximum power based on foldback function */
    uint16_t  hOverMaxPowerTimer;       /* timer to count time elapsed with power more that MAX */
    uint16_t  hBelowMaxPowerTimer;      /* timer to count time elapsed with power less that MAX */
    uint16_t  hOverMaxPowerTimeout;     /* timeout for hOverMaxPowerTimeout */
    uint16_t  hBelowMaxPowerTimeout;    /* timeout for hBelowMaxPowerTimeout */
} DynamicPowerHandle_t;


/* Exported functions ------------------------------------------------------- */

/**
  * @brief  It reset the state variable to zero.
  * @param  pHandle related Handle of struct RampMngr_Handle_t
  * @retval none.
  */
void DynamicPower_Init(DynamicPowerHandle_t * pHandle);

#endif /* __DYNAMICPOWER_H */



