/**
  * @file    pwm_common.c
  * @author  Sami Bouzid, FTEX inc
  * @brief   This file provides firmware functions that implement common features
  *          of the PWM & Current Feedback component.
  *
*/

/* Includes ------------------------------------------------------------------*/
#include "pwm_common.h"


void waitForPolarizationEnd( volatile uint8_t *cnt )
{
  while (*cnt < NB_CONVERSIONS)
  {

  }
}


