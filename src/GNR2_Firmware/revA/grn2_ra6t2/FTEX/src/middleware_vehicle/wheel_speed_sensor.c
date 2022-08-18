/**
  ******************************************************************************
  * @file    wheel_speed_sensor.c
  * @author  FTEX inc
  * @brief   This file defines the functions used in higher 
  *          level modules for wheel speed sensor
  ******************************************************************************
*/

#include "wheel_speed_sensor.h"
#include "ASSERT_FTEX.h"

// =============================== Defines ================================== //

#define WRPMCOEFF       60			// RPM multiplication for r/min
#define WPRECISIONCOEFF 1000	    // ms coefficient precision
#define WCOEFFREQ       1000000000	// Period coeff for usecond division



