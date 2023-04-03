/**
  * @file    Utilities.h
  * @brief   This file contains general utility functions. This file should not store anything, it's just
  *          to be used for generic, reusable utility functions
  */
  
#ifndef __UTLITIES_H
#define __UTLITIES_H

#include "stdint.h"
#include "wheel_speed_sensor.h"
#include "ASSERT_FTEX.h"

// Generic pair of int32 elements
typedef struct Int32Pair
{
    int32_t First;
    int32_t Second;
} Int32Pair;
    
/**
  * @brief  Compute a linear equation y = mx + b using two points
  * @param  The two points (x1, y1) and (x2, y2)
  * @retval A pair representing the two equation y = mx+b parameters. Pair.First is m and Pair.Second is b
  */
Int32Pair Utilities_ComputeLinearEquation(int32_t x1, int32_t y1, int32_t x2, int32_t y2);



#endif