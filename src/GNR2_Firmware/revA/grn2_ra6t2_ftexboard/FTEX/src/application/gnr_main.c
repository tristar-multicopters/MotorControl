/**
  * @file    gnr_main.c
  * @brief   This file is the main application of the ganrunner motor controller firmware
  *
  */

#include "gnr_main.h"
#include "vc_tasks.h"
#include "mc_tasks.h"
#include "comm_tasks.h"
#include "vc_config.h"
#include "firmware_update.h"
#include "watchdog.h"

// disable warning about user_config_task modifying the pragma pack value
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wpragma-pack"
#include "user_config_task.h"
#pragma clang diagnostic pop

//****************** THREAD EXTERN FUNCTION PROTOTYPES ******************//

extern void startMCSafetyTask(void * pvParameter);
extern void startMCMediumFrequencyTask(void * pvParameter);
extern void THR_VC_MediumFreq(void * pvParameter);
extern void THR_VC_StateMachine(void * pvParameter);
extern void ProcessUARTFrames(void * pvParameter);
extern void PowerOffSequence(void * pvParameter);
extern void Watchdog(void * pvParameter);

#if !GNR_MASTER
extern void PWREN_TurnoffSlaveTask (void * pvParameter);
#endif

//****************** LOCAL FUNCTION PROTOTYPES ******************//

static bool ADCInit(void);
static bool GPTInit(void);

#if HARDWARE_OCD == OCD_PWM_OFF
static bool POEGInit(void); //please read the describtion on function definition before enablenig this
#endif
static bool DACInit(void);
static bool ICUInit(void);
static bool ELCInit(void);
static bool AGTInit(void);
static bool UARTInit(void);
static bool CANInit(void);
static bool IIRFAInit(void);
static bool SPIInit(void);

//****************** THREAD HANDLES ******************//

osThreadId_t MC_MediumFrequencyTask_handle;
osThreadId_t MC_SafetyTask_handle;
osThreadId_t THR_VC_MediumFreq_handle;
osThreadId_t THR_VC_StateMachine_handle;
osThreadId_t COMM_Uart_handle;
osThreadId_t CANOpenTaskHandle;
osThreadId_t CANRxFrameHandle;
osThreadId_t PowerOffSequence_handle;
osThreadId_t Watchdog_handle;

#if !GNR_MASTER
osThreadId_t PWREN_TurnoffSlave_handle;
#endif

//****************** THREAD ATTRIBUTES ******************//

//TODO: NEED TO ADJUST THREAD PRIORITIES

static const osThreadAttr_t ThAtt_MC_SafetyTask = {
    .name = "MC_SafetyTask",
    .stack_size = 1024,
    .priority = osPriorityAboveNormal6,
};

static const osThreadAttr_t ThAtt_MC_MediumFrequencyTask = {
    .name = "MC_MediumFrequencyTask",
    .stack_size = 1024,
    .priority = osPriorityNormal5
};

static const osThreadAttr_t ThAtt_PowerOffSequence = {
	.name = "VC_PowerOffSequence",
	.stack_size = 512,
	.priority = osPriorityBelowNormal
};


#if !DEBUGMODE_MOTOR_CONTROL
#if GNR_MASTER
static const osThreadAttr_t ThAtt_VC_MediumFrequencyTask = {
    .name = "VC_MediumFrequencyTask",
    .stack_size = 1024,
    .priority = osPriorityAboveNormal3
};

static const osThreadAttr_t ThAtt_VehicleStateMachine = {
    .name = "VC_StateMachine",
    .stack_size = 1024,
    .priority = osPriorityAboveNormal2,
};
#if CANLOGGERTASK
static const osThreadAttr_t ThAtt_CANLogger = {
    .name = "CANLoggerTask",
    .stack_size = 512,
    .priority = osPriorityLow
};
#endif

#endif

static const osThreadAttr_t ThAtt_UART = {
	.name = "TSK_UART",
	.stack_size = 512,
	.priority = osPriorityBelowNormal
};

static const osThreadAttr_t ThAtt_Watchdog = {
	.name = "TSK_Watchdog",
	.stack_size = 256,
	.priority = osPriorityLow2
};

#endif

#if !GNR_MASTER
static const osThreadAttr_t ThAtt_PWREN_TurnoffSlaveTask = {
    .name = "PWREN_TurnoffSlaveTask",
    .stack_size = 512,
    .priority = osPriorityLow
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
#if HARDWARE_OCD == OCD_PWM_OFF
    POEGInit(); //Read the describtion on the Function before enabling it
#endif
    DACInit();
    ICUInit();
    ELCInit();
    AGTInit();
    UARTInit();
    CANInit();
    SPIInit();
    IIRFAInit();
    WatchdogInit();
    
    /* At this point, hardware should be ready to be used by application systems */
    
    //Initialize external flash memory used on DFU process.
    FirmwareUpdate_SerialFlashMemoryInit();
    
    // Bootloader app to marks the image with index 0 in the primary slot as confirmed
    boot_set_confirmed();

    //load user config from the data flash.
    UserConfigTask_InitUserConfigFromDataFlash(&UserConfigHandle,&DataFlashHandle,&VCInterfaceHandle);
    
    //fucntion to pass the user configuration read from the memory to
    //VCInterfaceHandle
    UserConfigTask_UpdateUserConfigData(&UserConfigHandle);

    MC_BootUp();
    #if GNR_MASTER
    VC_BootUp();
    #endif
    Comm_BootUp();
    
    //Initialize the CAN OPEN ID/object dictionary.
    CANOpenTask();

    SystemCoreClockUpdate(); // Standard ARM function to update clock settings
    // Initialise the kernel
    ASSERT(osKernelInitialize() == osOK);

    //EventRecorderInitialize(EventRecordAll,1U); // Initialise the event recorder

    /* Create the threads */
    MC_MediumFrequencyTask_handle   = osThreadNew(startMCMediumFrequencyTask,
                                      NULL,
                                      &ThAtt_MC_MediumFrequencyTask);
              
    //verify if the task was correctly created.              
    ASSERT(MC_MediumFrequencyTask_handle != NULL);

    MC_SafetyTask_handle            = osThreadNew(startMCSafetyTask,
                                      NULL,
                                      &ThAtt_MC_SafetyTask);
                                      
    //verify if the task was correctly created.              
    ASSERT(MC_SafetyTask_handle != NULL);
    
    PowerOffSequence_handle         = osThreadNew(PowerOffSequence, 
                                      NULL,
                                      &ThAtt_PowerOffSequence);
    
    //verify if the task was correctly created.              
    ASSERT(PowerOffSequence_handle != NULL);
    
    #if !DEBUGMODE_MOTOR_CONTROL
    #if GNR_MASTER
    THR_VC_MediumFreq_handle        = osThreadNew(THR_VC_MediumFreq,
                                      NULL,
                                      &ThAtt_VC_MediumFrequencyTask);
                                      
    //verify if the task was correctly created.              
    ASSERT(THR_VC_MediumFreq_handle != NULL);

    THR_VC_StateMachine_handle      = osThreadNew(THR_VC_StateMachine,
                                      NULL,
                                      &ThAtt_VehicleStateMachine);
                                      
    //verify if the task was correctly created.              
    ASSERT(THR_VC_StateMachine_handle != NULL);
                                      
    #if CANLOGGERTASK

    CANOpenTaskHandle               = osThreadNew(CANLoggerTask,
                                      NULL,
                                      &ThAtt_CANLogger);  
    
    //verify if the task was correctly created.              
    ASSERT(CANOpenTaskHandle != NULL);
    #endif

    #endif
    
    COMM_Uart_handle                = osThreadNew(ProcessUARTFrames,
                                      NULL,
                                      &ThAtt_UART);
                                      
    //verify if the task was correctly created.              
    ASSERT(COMM_Uart_handle != NULL);
	
    
    Watchdog_handle                 = osThreadNew(Watchdog,
                                      NULL,
                                      &ThAtt_Watchdog);
    //verify if the task was correctly created.              
    ASSERT(Watchdog_handle != NULL);
    
    #endif
    
    #if !GNR_MASTER
    //Create a nerw task.
    PWREN_TurnoffSlave_handle       = osThreadNew(PWREN_TurnoffSlaveTask, 
                                      &CONodeGNR,
                                      &ThAtt_PWREN_TurnoffSlaveTask);
    
    //verify if the task was correctly created.              
    ASSERT(PWREN_TurnoffSlave_handle != NULL);
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

    /* ________________________
     *        GPT9
     * ________________________ */

    /* Capture timer settings for wheel speed sensor   */
    bIsError |= R_GPT_Open(g_timer9.p_ctrl, g_timer9.p_cfg);
    bIsError |= R_GPT_Enable(g_timer9.p_ctrl);

    return bIsError;
}

/*
* @brief  Initialize POEG (Port Output Enable for GPT) hardware

* The POEG is going to stop PWM output right after a falling edge occures on POEG PIN 
* which is defined on PC14 on this code. Th reason that we a re not going to use 
* this fuction is that PC14 is connected to OCD2 pin of Current Sensor on PCB
* which is making alert signal when the current exceeds 82% of max thershold, where we
* only need to derate power, not shutting down the output.
* P.S. if for any reason you want to bring this feature back, do these stpe:
1- uncomment bool POEGInit() decleration in first rows of this file
2- uncomment following function definition 
3- Check if the PC14 is still defined as PEOG functionality
4- Check the PWMBreak1_IRQHandler() which is going to raise the MC_BREAK_IN error
5- Check the PWRT_MotorFaultManagement() if still includes handeling this error flag
6- Use the R_POEG_Open() function to bring back the PWM signal after error is handled
*/

#if HARDWARE_OCD == OCD_PWM_OFF
static bool POEGInit(void)
{
    bool bIsError = false;
    bIsError |= R_POEG_Open(g_poeg0.p_ctrl, g_poeg0.p_cfg);
    return bIsError;
}
#endif

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
	  /* ________________________
     *        AGT0
     * ________________________ */
    // Initialize the low power timer for capture mode
    bIsError |= R_AGT_Open(g_timer_a0.p_ctrl, g_timer_a0.p_cfg);
    // Enable external event triggers that start the AGT
    bIsError |= R_AGT_Enable(g_timer_a0.p_ctrl);
	  /* ________________________
     *        AGT1
     * ________________________ */
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

/**
  * @brief  Function used to Initialize the SPI
  */
static bool SPIInit(void)
{
    bool bIsError = false;

    // Initialise the SPI_B
    bIsError |= R_SPI_B_Open(&g_spi1_ctrl, &g_spi1_cfg);

    return bIsError;
}



