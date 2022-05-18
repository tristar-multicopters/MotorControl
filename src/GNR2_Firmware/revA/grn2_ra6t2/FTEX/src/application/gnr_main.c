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
static bool GPT0Init(void);

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
    GPT0Init();
	
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
	bool bIsError = false;
	
	bIsError |= (bool)R_ADC_B_Open(g_adc.p_ctrl, g_adc.p_cfg);
	
	bIsError |= (bool)R_ADC_B_Calibrate(g_adc.p_ctrl, NULL);

	/* Wait for calibration to complete */
	adc_status_t status = {.state = ADC_STATE_SCAN_IN_PROGRESS};
	while ((ADC_STATE_SCAN_IN_PROGRESS == status.state) &&
				 (FSP_SUCCESS == bIsError))
	{
			bIsError |= (bool)R_ADC_B_StatusGet(g_adc.p_ctrl, &status);
	}
	
	bIsError |= R_ADC_B_ScanCfg(g_adc.p_ctrl, &g_adc_scan_cfg);
	
	return bIsError;
}

static bool GPTInit(void)
{
	bool bIsError = false;
	
	bIsError |= R_GPT_THREE_PHASE_Open(g_three_phase0.p_ctrl, g_three_phase0.p_cfg);
	
	/* Frequency setup */
	bIsError |= R_GPT_PeriodSet(g_three_phase0.p_cfg->p_timer_instance[THREE_PHASE_CHANNEL_U]->p_ctrl, PWM_PERIOD_CYCLES/2);
	bIsError |= R_GPT_PeriodSet(g_three_phase0.p_cfg->p_timer_instance[THREE_PHASE_CHANNEL_V]->p_ctrl, PWM_PERIOD_CYCLES/2);
	bIsError |= R_GPT_PeriodSet(g_three_phase0.p_cfg->p_timer_instance[THREE_PHASE_CHANNEL_W]->p_ctrl, PWM_PERIOD_CYCLES/2);
	
	/* Deatime setup */
	g_timer4_ctrl.p_reg->GTDVU = DEAD_TIME_COUNTS;
	g_timer4_ctrl.p_reg->GTDVD = DEAD_TIME_COUNTS;
	g_timer5_ctrl.p_reg->GTDVU = DEAD_TIME_COUNTS;
	g_timer5_ctrl.p_reg->GTDVD = DEAD_TIME_COUNTS;
	g_timer6_ctrl.p_reg->GTDVU = DEAD_TIME_COUNTS;
	g_timer6_ctrl.p_reg->GTDVD = DEAD_TIME_COUNTS;
	
	return bIsError;
}	

static bool GPT0Init(void)
{
    bool bIsError = false;
    /* Turn on timer0 to be used for hall capture compare */
    /* Capture Timer settings for HALL      */
    bIsError |=R_GPT_Open(g_timer0.p_ctrl, g_timer0.p_cfg);
    bIsError |=R_GPT_Enable(g_timer0.p_ctrl);
    bIsError |=R_GPT_PeriodSet(g_timer0.p_ctrl, 524287uL);
    bIsError |=R_GPT_Start(g_timer0.p_ctrl);
    /* Congigure external interrupts for HALL SENSING   */
    bIsError |=R_ICU_ExternalIrqOpen(g_external_irq0.p_ctrl,g_external_irq0.p_cfg);
    bIsError |=R_ICU_ExternalIrqOpen(g_external_irq1.p_ctrl,g_external_irq1.p_cfg);
    bIsError |=R_ICU_ExternalIrqOpen(g_external_irq2.p_ctrl,g_external_irq2.p_cfg);

    return bIsError;
}

