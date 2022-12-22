/**
    * @file    foldback.h
	* @author  Jorge Andres Polo, FTEX
	* @author  Sami Bouzid, FTEX
    * @author  Ronak Nemade, FTEX
    * @brief   This module implement a negative ramp to limit an input value based on another variable.
    *
*/

#ifndef __FOLDBACK_H
#define __FOLDBACK_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h> // explicitely include stdlib since we're using the abs function

typedef enum    // Enumerated data type to set the configuration of foldback
{
    TRIM = 0,
    SET_THRESHOLD = 1
} FoldbackConfig_t;

typedef struct
{
    bool bEnableFoldback;               // Check flag if foldback is enabled.
    bool bIsInverted;                   // Check flag if input variables are negative.
    bool bEnableSlowStart;              // Flag for smooth start using PAS
    bool bRefreshSlowStart;             // Flag to refresh the smooth start using PAS
    uint16_t hSlowStartBandwidth;       // Slow start filter bandwidth coefficient for slow ramp to the PAS Control
    uint16_t hSlowStopBandwidth;        // Slow stop filter bandwidth coefficient for slow ramp to the PAS Control
    uint32_t wSlowStartTimeout;         // Fold Back slow start timeout   
    
    FoldbackConfig_t FoldbackConfig;                     // Check flag if the foldback handle is configured to trim the input or set the thresholds. 
    
    int16_t  hDecreasingEndValue;        // Maximum of control variable beyond which input variable is trimmed to lower limit. 
	uint16_t hDecreasingRange;          // Decreasing range of control variable.
    int16_t  hDecreasingInterval;       // Range of control variable beyond which input variable is trimmed to lower limit. 
	int16_t  hMaxOutputLimitHigh;       // Higher threshold of maximum value that input variable can take, this variable changes depending upon operating conditions.
    int16_t  hMaxOutputLimitLow;        // Lower threshold of maximum value that input variable can take, this variable changes depending upon operating conditions.
    uint16_t hDefaultOutputLimitHigh;   // Higher threshold of maximum value that input variable can take.
    uint16_t hDefaultOutputLimitLow;    // Lower threshold of maximum value that input variable can take.
    
} Foldback_Handle_t;

/**
  * @brief  Function for Initialializing foldback
  * @param  pHandle: handler of the current instance of the Foldback component
  */
void Foldback_Init( Foldback_Handle_t * pHandle );

/**
  * @brief  Function for applying the limitation based on the control variable
  * @param  pHandle: handler of the current instance of the Foldback component
  * @param  hInputVariable: Input variable before limitation
  * @param  hControlVariable: Variable that define the output limit
  * @retval Input variable after limitation
  */
int16_t Foldback_ApplyFoldback(Foldback_Handle_t * pHandle, int16_t hInputVariable, int16_t hControlVariable);

/**
  * @brief  Function for updating the thresholds between which the input needs to be trimmed. 
  * @param  pHandle: handler of the current instance of the Foldback component
  * @param  hDecreasingEndValue: End value of decreasing output limit
  * @retval None
  */
void Foldback_UpdateMaxValue(Foldback_Handle_t * pHandle, int16_t hMaxValue);

/**
  * @brief  Function for setting the range of the control variable from when output limit starts to decrease until lower limit
  * @param  pHandle: handler of the current instance of the Foldback component
  * @param  hDecreasingRange: Range of decreasing output limit
  * @retval None
  */
void Foldback_SetDecreasingRange(Foldback_Handle_t * pHandle, uint16_t hDecreasingRange);

/**
  * @brief  Function for setting the end value of the control variable, i.e. when output is equal to lower threshold value
  * @param  pHandle: handler of the current instance of the Foldback component
  * @param  hDecreasingEndValue: End value of decreasing output limit
  * @retval None
  */
void Foldback_SetDecreasingEndValue(Foldback_Handle_t * pHandle, int16_t hDecreasingRange);

/**
  * @brief  Function for setting the end value of the control variable based on an interval
  * @param  pHandle: handler of the current instance of the Foldback component
  * @param  hDecreasingEndValue: End value of decreasing output limit
  * @retval None
  */
void Foldback_SetDecreasingRangeEndValue(Foldback_Handle_t * pHandle, int16_t hDecreasingRange);

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

/**
  * @brief  Apply a low pass filter on the torque after for smooth 
  *         accelearation with Pedal Assist  
  * @param  Foldback handle and torque value
  * @retval Torque after passing it through the filter
  */
int16_t Foldback_ApplySlowStart(Foldback_Handle_t * pHandle, int16_t hTorque);

/**
  * @brief  Used to start or refresh a slow start
  * @param  Foldback handle 
  * @retval Nothing
  */
void Foldback_EnableSlowStart(Foldback_Handle_t * pHandle);

#endif

