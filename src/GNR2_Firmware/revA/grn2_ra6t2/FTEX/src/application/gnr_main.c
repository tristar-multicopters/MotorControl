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
static bool POEGInit(void);
static bool DACInit(void);
static bool ICUInit(void);
static bool ELCInit(void);

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
	POEGInit();
	DACInit();
	ICUInit();
	ELCInit();

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

/**
  * @brief  Initialize ADC (Analog Digital Converter) hardware
  */
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

/**
  * @brief  Initialize GPT (General Purpose Timer) hardware
  */
static bool GPTInit(void)
{
	bool bIsError = false;

	/* ________________________
	 *		GPT4, GPT5, GPT6
	 * ________________________ */
	
	bIsError |= R_GPT_THREE_PHASE_Open(g_three_phase0.p_ctrl, g_three_phase0.p_cfg);

	/* Frequency setup of PWM timers */
	bIsError |= R_GPT_PeriodSet(g_three_phase0.p_cfg->p_timer_instance[THREE_PHASE_CHANNEL_U]->p_ctrl, PWM_PERIOD_CYCLES/2);
	bIsError |= R_GPT_PeriodSet(g_three_phase0.p_cfg->p_timer_instance[THREE_PHASE_CHANNEL_V]->p_ctrl, PWM_PERIOD_CYCLES/2);
	bIsError |= R_GPT_PeriodSet(g_three_phase0.p_cfg->p_timer_instance[THREE_PHASE_CHANNEL_W]->p_ctrl, PWM_PERIOD_CYCLES/2);

	/* Deadtime setup of PWM timers */
	g_timer4_ctrl.p_reg->GTDVU = DEAD_TIME_COUNTS;
	g_timer4_ctrl.p_reg->GTDVD = DEAD_TIME_COUNTS;
	g_timer5_ctrl.p_reg->GTDVU = DEAD_TIME_COUNTS;
	g_timer5_ctrl.p_reg->GTDVD = DEAD_TIME_COUNTS;
	g_timer6_ctrl.p_reg->GTDVU = DEAD_TIME_COUNTS;
	g_timer6_ctrl.p_reg->GTDVD = DEAD_TIME_COUNTS;

	/* ________________________
	 *		GPT0
	 * ________________________ */
	 
	/* Capture Timer settings for hall sensing      */
	bIsError |= R_GPT_Open(g_timer0.p_ctrl, g_timer0.p_cfg);
	bIsError |= R_GPT_Enable(g_timer0.p_ctrl);
	bIsError |= R_GPT_PeriodSet(g_timer0.p_ctrl, 524287uL);

	return bIsError;
}

/**
  * @brief  Initialize POEG (Port Output Enable for GPT) hardware
  */
static bool POEGInit(void)
{
	bool bIsError = false;

	bIsError |= R_POEG_Open(g_poeg1.p_ctrl, g_poeg1.p_cfg);

	return bIsError;
}

/**
  * @brief  Initialize DAC (Digital Analog Converter) hardware
  */
static bool DACInit(void)
{
	bool bIsError = false;

	bIsError |= R_DAC_Open(g_dac1.p_ctrl,g_dac1.p_cfg);
	bIsError |= R_DAC_Start(g_dac1.p_ctrl);

	bIsError |= R_DAC_Open(g_dac0.p_ctrl,g_dac0.p_cfg);
	bIsError |= R_DAC_Start(g_dac0.p_ctrl);

	return bIsError;
}

/**
  * @brief  Initialize ELC (Event Link Controller) hardware
  */
static bool ELCInit(void)
{
	bool bIsError = false;

	bIsError |= R_ELC_Open(g_elc.p_ctrl, g_elc.p_cfg);
	bIsError |= R_ELC_Enable(g_elc.p_ctrl);

	return bIsError;
}

/**
  * @brief  Initialize ICU (Interrupt Controller Unit) hardware
  */
static bool ICUInit(void)
{
	bool bIsError = false;

	/* Configure external interrupts for hall sensing  */
	bIsError |= R_ICU_ExternalIrqOpen(g_external_irq0.p_ctrl,g_external_irq0.p_cfg);
	bIsError |= R_ICU_ExternalIrqOpen(g_external_irq1.p_ctrl,g_external_irq1.p_cfg);
	bIsError |= R_ICU_ExternalIrqOpen(g_external_irq2.p_ctrl,g_external_irq2.p_cfg);

	return bIsError;
}

