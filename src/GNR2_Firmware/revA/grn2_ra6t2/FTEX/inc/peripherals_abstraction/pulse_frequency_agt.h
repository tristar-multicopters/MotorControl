/**
  ******************************************************************************
  * @file    pulse_frequency_agt.h
  * @author  FTEX inc
  * @brief   Header for the Pulse frequency reading module
	*					 pulse_frequency_agt.c using Low power Timer
  ******************************************************************************
*/

#ifndef __PULSE_FREQUENCY_AGT_H
#define __PULSE_FREQUENCY_AGT_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "hal_data.h"


// ============================ Defines ==================================== //


// ======================== Pin configuration enums ======================== //

// ================= Structure used to configure a pin ===================== //


typedef struct
{
    const timer_instance_t* PF_AGT_Timer;
	
} PulseFreqParam_AGT_t;


typedef struct
{
    uint64_t agt_capture_count;		 /* AGT timer capture variable */
    uint32_t agt_capture_overflow; /* AGT timer capture overflow variable */

    volatile bool  agt_start_measurement; /* AGT Flag start measurment */	

    PulseFreqParam_AGT_t PulseFreqParam_AGT; /* AGT Parameters*/
	
} PulseFrequency_Handle_AGT_t; 


// ============================== Variables ================================ //



// ==================== Public function prototypes ========================= //


/**
  @brief  Function used to read the capture input from the AGT
  @param  PulseFrequency_Handle_AGT_t handle
  @return None
*/
void PulseFrequency_ReadInputCapture_AGT (PulseFrequency_Handle_AGT_t * pHandle);

/**
  @brief  Function used to return the capture read input from the AGT
  @param  PulseFrequency_Handle_AGT_t handle
  @return None
*/
uint64_t PulseFrequency_InputCaptureValue_AGT (PulseFrequency_Handle_AGT_t * pHandle);

/**
  @brief  Function used to update the capture variables from the AGT interrupt
  @param  PulseFrequency_Handle_AGT_t handle
  @return None
*/
void PulseFrequency_IsrCallUpdate_AGT( PulseFrequency_Handle_AGT_t * pHandle , uint64_t  wCapture );

/**
  @brief  Function used to update the overflow variable from the AGT interrupt
  @param  PulseFrequency_Handle_AGT_t handle
  @return None
*/
void PulseFrequency_IsrOverFlowUpdate_AGT( PulseFrequency_Handle_AGT_t * pHandle );


#endif /* __GNR_MAIN_H */