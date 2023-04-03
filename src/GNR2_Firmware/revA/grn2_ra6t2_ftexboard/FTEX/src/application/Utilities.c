/**
  ******************************************************************************
  * @file    Utilities.c
  * @brief   Implementation of generic utility functions that can be reused across modules
             This file should not be storing anything
  ******************************************************************************
*/

#include "Utilities.h"

/**
  * @brief  Compute a linear equation y = mx + b using two points
  * @param  The two points (x1, y1) and (x2, y2)
  * @retval A pair representing the two equation y = mx+b parameters. Pair.First is m and Pair.Second is b
  */
Int32Pair Utilities_ComputeLinearEquation(int32_t x1, int32_t y1, int32_t x2, int32_t y2)
{    
    Int32Pair returnedPair;
    // Compute in float to preserve precision, but cast to int before returning
    float m = (float)(y2 - y1) / (float)(x2 - x1);
    returnedPair.First = (int32_t)m;
        
    // Compute in float to preserve precision, but cast to int before returning
    float b = (float)y2 - m * (float)x2;
    returnedPair.Second = (int32_t)b;
    
    return returnedPair;
}

