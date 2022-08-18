/**
    * @file    foldback.h
	* @author  Jorge Andres Polo, FTEX
	* @author  Sami Bouzid, FTEX
    * @brief   This module implement a negative ramp to limit an input value based on another variable.
    *
*/

#ifndef __FOLDBACK_H
#define __FOLDBACK_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

typedef struct
{
	int16_t hDecreasingEndValue;
	uint16_t hDecreasingRange;
	int16_t hDefaultMaxOutput;
	
	bool bEnableFoldback;
	bool bIsInverted;
	
} Foldback_Handle_t;


/**
  * @brief  Function for applying the limitation based on the control variable
  * @param  pHandle: handler of the current instance of the Foldback component
  * @param  hInputVariable: Input variable before limitation
  * @param  hControlVariable: Variable that define the output limit
  * @retval Input variable after limitation
  */
int16_t Foldback_ApplyFoldback(Foldback_Handle_t * pHandle, int16_t hInputVariable, int16_t hControlVariable);

/**
  * @brief  Function for setting the range of the control variable from when output limit starts to decrease until zero
  * @param  pHandle: handler of the current instance of the Foldback component
  * @param  hDecreasingRange: Range of decreasing output limit
  * @retval None
  */
void Foldback_SetDecreasingRange(Foldback_Handle_t * pHandle, uint16_t hDecreasingRange);

/**
  * @brief  Function for setting the end value of the control variable, i.e. when output limit is zero
  * @param  pHandle: handler of the current instance of the Foldback component
  * @param  hDecreasingEndValue: End value of decreasing output limit
  * @retval None
  */
void Foldback_SetDecreasingEndValue(Foldback_Handle_t * pHandle, int16_t hDecreasingEndValue);

/**
  * @brief  Function for enabling foldback feature
  * @param  pHandle: handler of the current instance of the Foldback component
  * @retval None
  */
void Foldback_EnableFoldback(Foldback_Handle_t * pHandle);

/**
  * @brief  Function for disabling foldback feature
  * @param  pHandle: handler of the current instance of the Foldback component
  * @retval None
  */
void Foldback_DisableFoldback(Foldback_Handle_t * pHandle);

#endif

