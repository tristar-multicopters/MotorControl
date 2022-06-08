/**
  ******************************************************************************
  * @file    pulse_frequency_gpt.h
  * @author  FTEX inc
  * @brief   Header for the Pulse frequency reading module
	*					 pulse_frequency_gpt.c using General Purpose Timer
  ******************************************************************************
*/

#ifndef __PULSE_FREQUENCY_GPT_H
#define __PULSE_FREQUENCY_GPT_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "hal_data.h"


// ============================ Defines ==================================== //


// ======================== Pin configuration enums ======================== //

// ================= Structure used to configure a pin ===================== //
typedef struct
{
    const timer_instance_t* PF_GPT_Timer;
	
} PF_Param_GPT_t;


typedef struct
{
    uint64_t gpt_capture_count; 	/* GPT timer capture variable */
    uint32_t gpt_capture_overflow;/* GPT timer capture overflow variable */

    volatile bool  gpt_start_measurement;		/* GPT Flag start measurment */	
    PF_Param_GPT_t PF_Param_GPT;						/* GPT Parameters */
	
} PF_Handle_GPT_t; 


// ============================== Variables ================================ //



// ==================== Public function prototypes ========================= //

/**
  @brief  Function used to read the capture input from the GPT
  @param  PF_Handle_GPT_t handle
  @return None
*/
void PulseFrequency_ReadInputCapture_GPT (PF_Handle_GPT_t * pHandle);

/**
  @brief  Function used to return the capture read input from the GPT
  @param  PF_Handle_GPT_t handle
  @return None
*/
uint64_t PulseFrequency_InputCaptureValue_GPT (PF_Handle_GPT_t * pHandle);

/**
  @brief  Function used to update the capture variables from the GPT interrupt
  @param  PF_Handle_GPT_t handle
  @return None
*/
void PulseFrequency_IsrCallUpdate_GPT( PF_Handle_GPT_t * pHandle , uint64_t  wCapture );

/**
  @brief  Function used to update the overflow variable from the GPT interrupt
  @param  PF_Handle_GPT_t handle
  @return None
*/
void PulseFrequency_IsrOverFlowUpdate_GPT( PF_Handle_GPT_t * pHandle );
	



#endif /* __GNR_MAIN_H */