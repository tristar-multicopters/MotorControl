/**
  * @file    pwm_common.c
  * @author  Sami Bouzid, FTEX inc
  * @brief   This file provides firmware functions that implement common features
  *          of the PWM & Current Feedback component.
  *
*/

/* Includes ------------------------------------------------------------------*/
#include "pwm_common.h"
#include "common_data.h"

/* Functions ---------------------------------------------------- */


void waitForPolarizationEnd(volatile uint8_t *cnt)
{
  while (*cnt < NB_CONVERSIONS)
  {

  }
}

void Driver_Disable(bool *bDriverEn)
{
    *bDriverEn = false;
    //Disable OCD1
    R_ICU_ExternalIrqDisable(g_external_irq0.p_ctrl);
    //Writes a 0 to the pin 
    uCAL_GPIO_Reset(DRIVER_EN_PIN);
    
}

void Driver_Enable(bool *bDriverEn)
{
    *bDriverEn = true;
    //Writes a 1 to the pin
    uCAL_GPIO_Set(DRIVER_EN_PIN);
    //Enable OCD1
    R_ICU_ExternalIrqEnable(g_external_irq0.p_ctrl);
    
}


