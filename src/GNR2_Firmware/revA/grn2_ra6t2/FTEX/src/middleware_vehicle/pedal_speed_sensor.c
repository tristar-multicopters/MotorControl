/**
  ******************************************************************************
  * @file    pedal_speed_sensor.c
  * @author  FTEX inc
  * @brief   This file defines the functions used in higher 
  *          level modules for pedal speed sensor
  ******************************************************************************
*/

#include "pedal_speed_sensor.h"
#include "ASSERT_FTEX.h"

// =============================== Defines ================================== //

#define RPMCOEFF        60      // RPM multiplication for r/min
#define PRECISIONCOEFF	1000	// ms coefficient precision
#define COEFFREQ        1000000000	// Period coeff for usecond division

