/**
  ******************************************************************************
  * @file    pulse_frequency.h
  * @author  FTEX inc
  * @brief   Header for the Pulse frequency reading module
	*					 pulse_frequency_gpt.c using General Purpose Timer
  ******************************************************************************
*/

#ifndef __PULSE_FREQUENCY_H
#define __PULSE_FREQUENCY_H

// =============================== Includes ================================== //
#include "stdint.h"
#include "stdbool.h"
#include "r_gpt.h"
#include "r_agt.h"

// =============================== Defines ================================== //
#define RPMCOEFF        60      // RPM multiplication for r/min
#define PRECISIONCOEFF	1000    // ms coefficient precision
#define COEFFREQ        1000000000	// Period coeff for usecond division
#define MICRO_SEC       1000000 // us coefficient precision
#define PERIOD_FACTOR   2       // Period factor
	   
// =============================== Variables ================================== //

	
// ================= Structure used to configure a pin ===================== //
typedef enum
{
	GPT_TIMER = 0,
	AGT_TIMER,
} TimerType_t;

typedef struct
{
    const timer_instance_t* PF_Timer;
	
}  PulseFrequencyParam_t; 


typedef struct
{
    TimerType_t TimerType;
    const timer_instance_t* pTimer;
	
    uint32_t wCaptureCount; 		/* timer capture variable */
    uint32_t wCaptureOverflow; /* timer capture overflow variable */
    uint32_t wUsPeriod; 			/* timer us Period Detection */
    uint16_t wNumberOfPulse;   /* has the number of pulse detected by
                               /* the time on capture mode.*/
    volatile bool  start_measurement;		/* Flag start measurment */
    PulseFrequencyParam_t PulseFreqParam; /* AGT Parameters*/
	
} PulseFrequencyHandle_t; 

// ==================== Public function prototypes ========================= //

/**
  @brief  Function used to read the capture input from the Timer
  @param  PulseFrequencyHandle_t handle
  @return None
*/
void PulseFrequency_ReadInputCapture (PulseFrequencyHandle_t * pHandle);

/**
  @brief  Function used to return period of pulse signal
  @param  PulseFrequencyHandle_t handle
  @return Latest period measurement of pulse signal
*/
uint32_t PulseFrequency_GetPeriod(PulseFrequencyHandle_t * pHandle);

/**
  @brief  Function used to update the capture variables from the interrupt
  @param  PulseFrequencyHandle_t handle
  @return None
*/
void PulseFrequency_IsrCallUpdate(PulseFrequencyHandle_t * pHandle, uint32_t wCapture);

/**
  @brief  Function used to update the overflow variable from the interrupt
  @param  PulseFrequencyHandle_t handle
  @return None
*/
void PulseFrequency_ISROverflowUpdate(PulseFrequencyHandle_t * pHandle);


#endif /* __PULSE_FREQUENCY_H */

