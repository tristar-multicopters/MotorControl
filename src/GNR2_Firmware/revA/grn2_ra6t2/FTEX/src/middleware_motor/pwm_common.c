/**
  ******************************************************************************
  * @file    pwm_common.c
  * @author  FTEX inc
  * @brief   This file provides firmware functions that implement common features
  *          of the PWM & Current Feedback component of the Motor Control SDK:
  *
  *           * start timers (main and auxiliary) synchronously
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "pwm_common.h"

//#ifdef TIM2
///**
// * @brief  It perform the start of all the timers required by the control.
// *          It utilizes TIM2 as temporary timer to achieve synchronization between
// *          PWM signals.
// *          When this function is called, TIM1 and/or TIM8 must be in frozen state
// *          with CNT, ARR, REP RATE and trigger correctly set (these setting are
// *          usually performed in the Init method accordingly with the configuration)
// * @param  none
// * @retval none
// */
//void startTimers( void )
//{
//  uint32_t isTIM2ClockOn;
//  uint32_t trigOut;

//  isTIM2ClockOn = LL_APB1_GRP1_IsEnabledClock ( LL_APB1_GRP1_PERIPH_TIM2 );
//  if ( isTIM2ClockOn == 0 )
//  {
//    /* Temporary Enable TIM2 clock if not already on */
//    LL_APB1_GRP1_EnableClock ( LL_APB1_GRP1_PERIPH_TIM2 );
//    LL_TIM_GenerateEvent_UPDATE ( TIM2 );
//    LL_APB1_GRP1_DisableClock ( LL_APB1_GRP1_PERIPH_TIM2 );
//  }
//  else
//  {
//    trigOut = LL_TIM_ReadReg( TIM2, CR2 ) & TIM_CR2_MMS;
//    LL_TIM_SetTriggerOutput( TIM2, LL_TIM_TRGO_UPDATE );
//    LL_TIM_GenerateEvent_UPDATE ( TIM2 );
//    LL_TIM_SetTriggerOutput( TIM2, trigOut );
//  }
//}
//#endif

///**
// * @brief  It waits for the end of the polarization. If the polarization exceeds the
// *         the number of needed PWM cycles, it reports an error.
// * @param  TIMx: timer used to generate PWM
// *         SWerror: variable used to report a SW error
// *         repCnt: repetition counter value
// *         cnt: polarization counter value
// * @retval none
// */
//void waitForPolarizationEnd( TIM_TypeDef*  TIMx, uint16_t  *SWerror, uint8_t repCnt, volatile uint8_t *cnt )
//{
//  uint16_t hCalibrationPeriodCounter;
//  uint16_t hMaxPeriodsNumber;

//  if ( (LL_TIM_GetCounterMode(TIMx) & LL_TIM_COUNTERMODE_CENTER_UP_DOWN) == LL_TIM_COUNTERMODE_CENTER_UP_DOWN)
//  {
//    hMaxPeriodsNumber=(2*NB_CONVERSIONS)*(((uint16_t)repCnt+1u));
//  }
//  else
//  {
//	hMaxPeriodsNumber=(2*NB_CONVERSIONS)*(((uint16_t)repCnt+1u)>>1);
//  }

//  /* Wait for NB_CONVERSIONS to be executed */
//  LL_TIM_ClearFlag_CC1(TIMx);
//  hCalibrationPeriodCounter = 0u;
//  while (*cnt < NB_CONVERSIONS)
//  {
//    if (LL_TIM_IsActiveFlag_CC1(TIMx))
//    {
//      LL_TIM_ClearFlag_CC1(TIMx);
//      hCalibrationPeriodCounter++;
//      if (hCalibrationPeriodCounter >= hMaxPeriodsNumber)
//      {
//        if (*cnt < NB_CONVERSIONS)
//        {
//          *SWerror = 1u;
//          break;
//        }
//      }
//    }
//  }
//}


