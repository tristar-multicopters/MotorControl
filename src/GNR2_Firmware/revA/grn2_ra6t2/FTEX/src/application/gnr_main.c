/**
  ******************************************************************************
  * @file    gnr_main.c
  * @author  FTEX inc
  * @brief   This file is the main application of the ganrunner motor controller firmware
  *
	******************************************************************************
	*/

#include "gnr_main.h"

extern void startMCSafetyTask(void * pvParameter);
extern void startMCMediumFrequencyTask(void * pvParameter);

static bool ADCInit(void);
static bool GPTInit(void);

osThreadId_t MC_MediumFrequencyTask_handle;
osThreadId_t MC_SafetyTask_handle;


static const osThreadAttr_t ThAtt_MC_SafetyTask = {
	.name = "MC_SafetyTask",
	.stack_size = 512,
	.priority = osPriorityAboveNormal3,
};

static const osThreadAttr_t ThAtt_MC_MediumFrequencyTask = {
	.name = "MC_MediumFrequencyTask",
	.stack_size = 512,
	.priority = osPriorityNormal2
};


void gnr_main(void)
{
	ADCInit();
	GPTInit();
	
	SystemCoreClockUpdate();
	
	osKernelInitialize();  // Initialise the kernel
	//EventRecorderInitialize(EventRecordAll,1U); // Initialise the events
	
	MC_MediumFrequencyTask_handle   = osThreadNew(startMCMediumFrequencyTask, 
																								NULL,
																								&ThAtt_MC_MediumFrequencyTask);
	 
	MC_SafetyTask_handle   					= osThreadNew(startMCSafetyTask, 
																								NULL,
																								&ThAtt_MC_SafetyTask);
	
	// Start thread execution
	if (osKernelGetState() == osKernelReady)
	{
    osKernelStart();                    								
  }

}

static bool ADCInit(void)
{
	bool err = false;
	
	err |= (bool)R_ADC_B_Open(g_adc.p_ctrl, g_adc.p_cfg);
	
	err |= (bool)R_ADC_B_Calibrate(g_adc.p_ctrl, NULL);

	/* Wait for calibration to complete */
	adc_status_t status = {.state = ADC_STATE_SCAN_IN_PROGRESS};
	while ((ADC_STATE_SCAN_IN_PROGRESS == status.state) &&
				 (FSP_SUCCESS == err))
	{
			err |= (bool)R_ADC_B_StatusGet(g_adc.p_ctrl, &status);
	}
	
	err |= R_ADC_B_ScanCfg(g_adc.p_ctrl, &g_adc_scan_cfg);
	
	return err;
}

static bool GPTInit(void)
{
	bool err = false;
	
	err |= R_GPT_THREE_PHASE_Open(g_three_phase0.p_ctrl, g_three_phase0.p_cfg);

	/* Start the timer. */
	err |= (bool)R_GPT_THREE_PHASE_Start(g_three_phase0.p_ctrl);
	
	return err;
}	

