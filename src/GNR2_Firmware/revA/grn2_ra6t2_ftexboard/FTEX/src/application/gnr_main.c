/**
  * @file    gnr_main.c
  * @brief   This file is the main application of the ganrunner motor controller firmware
  *
  */

#include "gnr_main.h"
#include "vc_tasks.h"
#include "mc_tasks.h"
#include "comm_tasks.h"

//****************** THREAD EXTERN FUNCTION PROTOTYPES ******************//

extern void startMCSafetyTask(void * pvParameter);
extern void startMCMediumFrequencyTask(void * pvParameter);
extern void THR_VC_MediumFreq(void * pvParameter);
extern void THR_VC_StateMachine(void * pvParameter);
extern void ProcessUARTFrames(void * pvParameter);
extern void CANOpenTask(void * pvParameter);

//****************** LOCAL FUNCTION PROTOTYPES ******************//

static bool ADCInit(void);
static bool GPTInit(void);
static bool POEGInit(void);
static bool DACInit(void);
static bool ICUInit(void);
static bool ELCInit(void);
static bool AGTInit(void);
static bool UARTInit(void);
static bool CANInit(void);
static bool IIRFAInit(void);


//****************** THREAD HANDLES ******************//

osThreadId_t MC_MediumFrequencyTask_handle;
osThreadId_t MC_SafetyTask_handle;
osThreadId_t THR_VC_MediumFreq_handle;
osThreadId_t THR_VC_StateMachine_handle;
osThreadId_t COMM_Uart_handle;
osThreadId_t CANOpenTaskHandle;
osThreadId_t CANRxFrameHandle;

//****************** THREAD ATTRIBUTES ******************//

//TODO: NEED TO ADJUST THREAD PRIORITIES

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

#if !DEBUGMODE_MOTOR_CONTROL
#if GNR_MASTER
static const osThreadAttr_t ThAtt_VC_MediumFrequencyTask = {
    .name = "VC_MediumFrequencyTask",
    .stack_size = 512,
    .priority = osPriorityAboveNormal2
};

static const osThreadAttr_t ThAtt_VehicleStateMachine = {
    .name = "VC_StateMachine",
    .stack_size = 512,
    .priority = osPriorityAboveNormal3,
};

static const osThreadAttr_t ThAtt_UART = {
	.name = "TSK_UART",
	.stack_size = 512,
	.priority = osPriorityBelowNormal
};
#endif

static const osThreadAttr_t ThAtt_CANOpen = {
    .name = "CANOpenTask",
    .stack_size = 512,
    .priority = osPriorityNormal
};

#endif

/**************************************************************/

/**
 * @brief Function for main application entry.
 */
void gnr_main(void)
{
    /* Hardware initialization */
    ADCInit();
    GPTInit();
    POEGInit();
    DACInit();
    ICUInit();
    ELCInit();
    AGTInit();
    UARTInit();
    CANInit();
    IIRFAInit();
    /* At this point, hardware should be ready to be used by application systems */

    MC_BootUp();
    #if GNR_MASTER
    VC_BootUp();
    #endif
    Comm_BootUp();

    SystemCoreClockUpdate(); // Standard ARM function to update clock settings

    osKernelInitialize();  // Initialise the kernel
    //EventRecorderInitialize(EventRecordAll,1U); // Initialise the event recorder

    /* Create the threads */
    MC_MediumFrequencyTask_handle   = osThreadNew(startMCMediumFrequencyTask,
                                      NULL,
                                      &ThAtt_MC_MediumFrequencyTask);

    MC_SafetyTask_handle            = osThreadNew(startMCSafetyTask,
                                      NULL,
                                      &ThAtt_MC_SafetyTask);

    #if !DEBUGMODE_MOTOR_CONTROL
    #if GNR_MASTER
    THR_VC_MediumFreq_handle        = osThreadNew(THR_VC_MediumFreq,
                                      NULL,
                                      &ThAtt_VC_MediumFrequencyTask);

    THR_VC_StateMachine_handle      = osThreadNew(THR_VC_StateMachine,
                                      NULL,
                                      &ThAtt_VehicleStateMachine);

   	COMM_Uart_handle                = osThreadNew(ProcessUARTFrames,
                                      NULL,
                                      &ThAtt_UART);
    #endif

    CANOpenTaskHandle                = osThreadNew(CANOpenTask,
                                      NULL,
                                      &ThAtt_CANOpen);
    #endif

    /* Start RTOS */
    if (osKernelGetState() == osKernelReady)
    {
    osKernelStart();
    }

    while (1)
    {
        // We should never get here...
    }

}

/**
  * @brief  Initialize ADC (Analog Digital Converter) hardware
  */
static bool ADCInit(void)
{
    bool bIsError = false;

    bIsError |= (bool)R_ADC_B_Open(g_adc0.p_ctrl, g_adc0.p_cfg);

    bIsError |= (bool)R_ADC_B_Calibrate(g_adc0.p_ctrl, NULL);

    /* Wait for calibration to complete */
    adc_status_t status = {.state = ADC_STATE_SCAN_IN_PROGRESS};
    while ((ADC_STATE_SCAN_IN_PROGRESS == status.state) &&
                 (FSP_SUCCESS == bIsError))
    {
        bIsError |= (bool)R_ADC_B_StatusGet(g_adc0.p_ctrl, &status);
    }

    bIsError |= R_ADC_B_ScanCfg(g_adc0.p_ctrl, &g_adc0_scan_cfg);

    return bIsError;
}

/**
  * @brief  Initialize GPT (General Purpose Timer) hardware
  */
static bool GPTInit(void)
{
    bool bIsError = false;

    /* ________________________
     *    GPT4, GPT5, GPT6
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
     *        GPT0
     * ________________________ */

    /* Capture timer settings for hall sensing      */
    bIsError |= R_GPT_Open(g_timer0.p_ctrl, g_timer0.p_cfg);
    bIsError |= R_GPT_Enable(g_timer0.p_ctrl);
    bIsError |= R_GPT_PeriodSet(g_timer0.p_ctrl, 524287uL);

//    /* ________________________
//     *        GPT2
//     * ________________________ */

//    /* Capture timer settings for wheel speed sensor   */
//    bIsError |= R_GPT_Open(g_timer2.p_ctrl, g_timer2.p_cfg);
//    bIsError |= R_GPT_Enable(g_timer2.p_ctrl);

    return bIsError;
}

/**
  * @brief  Initialize POEG (Port Output Enable for GPT) hardware
  */
static bool POEGInit(void)
{
    bool bIsError = false;

    bIsError |= R_POEG_Open(g_poeg0.p_ctrl, g_poeg0.p_cfg);

    return bIsError;
}

/**
  * @brief  Initialize DAC (Digital Analog Converter) hardware
  */
static bool DACInit(void)
{
    bool bIsError = false;

    bIsError |= R_DAC_Open(g_dac2.p_ctrl,g_dac2.p_cfg);
    bIsError |= R_DAC_Start(g_dac2.p_ctrl);

    bIsError |= R_DAC_Open(g_dac3.p_ctrl,g_dac3.p_cfg);
    bIsError |= R_DAC_Start(g_dac3.p_ctrl);

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
    bIsError |= R_ICU_ExternalIrqOpen(g_external_irq3.p_ctrl,g_external_irq3.p_cfg);
    bIsError |= R_ICU_ExternalIrqOpen(g_external_irq4.p_ctrl,g_external_irq4.p_cfg);
    bIsError |= R_ICU_ExternalIrqOpen(g_external_irq5.p_ctrl,g_external_irq5.p_cfg);

    return bIsError;
}

/**
  * @brief  Function used to initialize the low power timer
  */
static bool AGTInit(void)
{
    bool bIsError = false;
//    // Initialize the low power timer 0 for capture mode
//    bIsError |= R_AGT_Open(g_timer_a0.p_ctrl, g_timer_a0.p_cfg);
//    // Enable external event triggers that start the AGT
//    bIsError |= R_AGT_Enable(g_timer_a0.p_ctrl);

	  // Initialize the low power timer 1 as timer for CANOpen
    bIsError |= R_AGT_Open(g_timer_a1.p_ctrl, g_timer_a1.p_cfg);

    return bIsError;
}

/**
  * @brief  Function used to Initialize the UART
  */
static bool UARTInit(void)
{
    bool bIsError = false;

    // Initialise the UART
    bIsError |= R_SCI_B_UART_Open(&g_uart9_ctrl, &g_uart9_cfg);

    return bIsError;
}

/**
  * @brief  Function used to Initialize the CAN
  */
static bool CANInit(void)
{
    bool bIsError = false;

    // Initialise the CAN
    bIsError |= R_CANFD_Open(&g_canfd0_ctrl, &g_canfd0_cfg);

    return bIsError;
}

/**
  * @brief  Function used to Initialize IIR filter hardware accelerator peripheral
  */
static bool IIRFAInit(void)
{
    bool bIsError = false;

    bIsError |= R_IIRFA_Open(&g_iirfa0_ctrl, &g_iirfa0_cfg);
    bIsError |= R_IIRFA_Open(&g_iirfa1_ctrl, &g_iirfa1_cfg);

    return bIsError;
}
