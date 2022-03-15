/**
  ******************************************************************************
  * @file    saadc_test.h
  * @author  Jorge Andres Polo, FTEX
  * @brief   This module implements a saadc to sampling values from a potentiometer
	*          TESTING PURPOSES
  *
	******************************************************************************
	*/
#ifndef __SAADC_TEST_H
#define __SAADC_TEST_H

/* FreeRTOS lib */
#include "FreeRTOS.h"
#include "task.h"

#include "nrf_drv_saadc.h"
#include "md_comm.h"
#include <math.h>

// Samples are needed to be stored in a buffer, we define the length here
#define SAMPLE_BUFFER_LEN 1

// Save the samples in double buffer which is  a two dimentional array
static nrf_saadc_value_t m_buffer_pool[2][SAMPLE_BUFFER_LEN]; 
void saadc_callback_handler(nrf_drv_saadc_evt_t const * p_event);
void saadc_init(void);
void saadc_init_Task(void * pvParameter);

#endif
