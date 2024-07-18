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

#define MINIMUMVAL   1 // Define Minimum Range Value for Filtering as safety

// Defines for each foldback. If a foldback is added or removed the define FOLDBACK_NUMBER should be adjusted accordingly
#define FOLDBACK_MAX_MOTOR_SPEED    0
#define FOLDBACK_SPEED_LIMIT        1
#define FOLDBACK_MOTOR_TEMP         2
#define FOLDBACK_CONTROLLER_TEMP    3
#define FOLDBACK_POWER_LIMIT        4
#define FOLDBACK_MAX_TORQUE         5

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
    
    int32_t  hDecreasingEndValue;        // Maximum of control variable beyond which input variable is trimmed to lower limit. 
    uint16_t hDecreasingRange;          // Decreasing range of control variable.
    int16_t  hDecreasingInterval;       // Range of control variable beyond which input variable is trimmed to lower limit. 
    int16_t  hMaxOutputLimitHigh;       // Higher threshold of maximum value that input variable can take, this variable changes depending upon operating conditions.
    int16_t  hMaxOutputLimitLow;        // Lower threshold of maximum value that input variable can take, this variable changes depending upon operating conditions.
    uint16_t hDefaultOutputLimitHigh;   // Higher threshold of maximum value that input variable can take.
    uint16_t hDefaultOutputLimitLow;    // Lower threshold of maximum value that input variable can take.
    
} Foldback_Handle_t;

/**
  * @brief  Function for Initialializing foldback
  * @param  chosenFoldback: choose the foldback
  */
void Foldback_Init(uint8_t chosenFoldback, Foldback_Handle_t FoldbackInit);

/**
  * @brief  Function for applying the limitation based on the control variable
  * @param  chosenFoldback: choose the foldback
  * @param  hInputVariable: Input variable before limitation
  * @param  hControlVariable: Variable that define the output limit
  * @retval Input variable after limitation
  */
int16_t Foldback_ApplyFoldback(uint8_t chosenFoldback, int16_t hInputVariable, int16_t hValue);

/**
  * @brief  Function for updating the thresholds between which the input needs to be trimmed. 
  * @param  chosenFoldback: choose the foldback
  * @param  hDecreasingEndValue: End value of decreasing output limit
  * @retval None
  */
void Foldback_SetMaxOutputLimitHigh(uint8_t chosenFoldback, int16_t hMaxValue);

/**
  * @brief  Function for setting max threshold value
  * @param  chosenFoldback: choose the foldback
  * @param  hDecreasingEndValue: End value of decreasing output limit
  * @retval None
  */
void Foldback_SetDefaultMaxOutputLimitHigh(uint8_t chosenFoldback, uint16_t maxValue);

/**
  * @brief  Function for updating the thresholds between which the input needs to be trimmed. 
  * @param  chosenFoldback: choose the foldback
  * @param  hDefaultOutputLimitLow: End value of decreasing output limit
  * @retval None
  */
void Foldback_SetMaxOutputLimitLow(uint8_t chosenFoldback, int16_t hLimitValue);


/**
  * @brief  Function for setting the range of the control variable from when output limit starts to decrease until lower limit
  * @param  chosenFoldback: choose the foldback
  * @param  hDecreasingRange: Range of decreasing output limit
  * @retval None
  */
void Foldback_SetDecreasingRange(uint8_t chosenFoldback, uint16_t hDecreasingRange);

/**
  * @brief  Function for setting the end value of the control variable, i.e. when output is equal to lower threshold value
  * @param  chosenFoldback: choose the foldback
  * @param  hDecreasingEndValue: End value of decreasing output limit
  * @retval None
  */
void Foldback_SetDecreasingEndValue(uint8_t chosenFoldback, int32_t hDecreasingRange);

/**
  * @brief  Function for getting the end value of the control variable, i.e. when output is equal to lower threshold value
  * @param  chosenFoldback: choose the foldback
  * @retval End value of decreasing output limit
  */
int32_t Foldback_GetDecreasingEndValue(uint8_t chosenFoldback);

/**
  * @brief  Function for setting the end value of the control variable based on an interval
  * @param  chosenFoldback: choose the foldback
  * @param  hDecreasingEndValue: End value of decreasing output limit
  * @retval None
  */
void Foldback_SetDecreasingRangeEndValue(uint8_t chosenFoldback, int16_t hDecreasingRange);

/**
  * @brief  Function for enabling foldback feature
  * @param  chosenFoldback: choose the foldback
  * @retval None
  */
void Foldback_EnableFoldback(uint8_t chosenFoldback);

/**
  * @brief  Function for disabling foldback feature
  * @param  chosenFoldback: choose the foldback
  * @retval None
  */
void Foldback_DisableFoldback(uint8_t chosenFoldback);

/**
  * @brief  Apply a low pass filter on the torque after for smooth 
  *         accelearation with Pedal Assist  
  * @param  chosenFoldback: choose the foldback
  * @param  hTorque: torque value
  * @retval Torque after passing it through the filter
  */
int16_t Foldback_ApplySlowStart(uint8_t chosenFoldback, int16_t hTorque);

/**
  * @brief  Used to start or refresh a slow start
  * @param  chosenFoldback: choose the foldback
  * @retval Nothing
  */
void Foldback_EnableSlowStart(uint8_t chosenFoldback);

#endif

