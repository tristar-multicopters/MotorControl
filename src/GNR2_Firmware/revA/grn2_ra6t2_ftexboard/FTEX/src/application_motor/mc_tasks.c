/**
 * @file  mc_tasks.c
 * @brief This file implements motor tasks definition
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "gnr_main.h"
#include "mc_type.h"
#include "mc_math.h"
#include "regular_conversion_manager.h"
#include "mc_interface.h"
#include "mc_tuning.h"
#include "mc_state_machine.h"
#include "pwm_common.h"
#include "board_hardware.h"
#include "uCAL_GPIO.h"
#include "log_high_speed.h"
#include "comm_config.h" // For dual

#include "mc_tasks.h"
#include "motor_parameters.h"
#include "gnr_parameters.h"
#include "vc_autodetermination.h"
#include "motor_signal_processing.h"

#if AUTOTUNE_ENABLE
#include "r_aid_auto_identify.h"
#include "r_aid_driver_if.h"
#include "r_aid_core.h"
#include "autotune.h"
AutoTune_Handle_t pAutoTune;
void MCTask_InitTuning();
bool MC_UpdateMotorTunerOutput(float fBatteryVoltage);
void MC_ConfigureMotorTuner(void);
#endif

/* Private define ------------------------------------------------------------*/

#define CHARGE_BOOT_CAP_MS 10
#define CHARGE_BOOT_CAP_MS2 10
#define OFFCALIBRWAIT_MS 0
#define OFFCALIBRWAIT_MS2 0
#define STOPPERMANENCY_MS 10
#define STOPPERMANENCY_MS2 10
#define CHARGE_BOOT_CAP_TICKS (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS) / 1000)
#define CHARGE_BOOT_CAP_TICKS2 (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS2) / 1000)
#define OFFCALIBRWAITTICKS (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS) / 1000)
#define OFFCALIBRWAITTICKS2 (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS2) / 1000)
#define STOPPERMANENCY_TICKS (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS) / 1000)
#define STOPPERMANENCY_TICKS2 (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS2) / 1000)
#define VBUS_TEMP_ERR_MASK (uint32_t) ~(0 | MC_NO_ERROR)
#define DEFAULT_TEMP_MOTOR 0xFFF
#define DEFAULT_TEMP_CONTROLLER 0x000
#define OCD2_MAX 7
#define OCD2_TIMER 10000 //~10 secs
#define DRIVER_TIMER 10000 //~.5 secs

/* Private variables----------------------------------------------------------*/
FOCVars_t FOCVars[NBR_OF_MOTORS];
MotorControlInterfaceHandle_t *oMCInterface[NBR_OF_MOTORS];
MotorControlInterfaceHandle_t MCInterface[NBR_OF_MOTORS];
MotorControlTuningHandle_t MCTuning[NBR_OF_MOTORS];
MotorStateMachineHandle_t MCStateMachine[NBR_OF_MOTORS];
SpdTorqCtrlHandle_t *pSpeedTorqCtrl[NBR_OF_MOTORS];
PIDHandle_t *pPIDSpeed[NBR_OF_MOTORS];
PIDHandle_t *pPIDIq[NBR_OF_MOTORS];
PIDHandle_t *pPIDId[NBR_OF_MOTORS];
ResDivVbusSensorHandle_t *pBusSensorM1;

NTCTempSensorHandle_t *pTemperatureSensorController[NBR_OF_MOTORS];
NTCTempSensorHandle_t *pTemperatureSensorMotor[NBR_OF_MOTORS];
PWMCurrFdbkHandle_t *pPWMCurrFdbk[NBR_OF_MOTORS];
MotorPowerQDHandle_t *pMotorPower[NBR_OF_MOTORS];
CircleLimitationHandle_t *pCircleLimitation[NBR_OF_MOTORS];
MCConfigHandle_t *pFieldWeakening[NBR_OF_MOTORS];
FeedforwardHandle_t *pFeedforward[NBR_OF_MOTORS];


static volatile uint16_t hMFTaskCounterM1 = 0;
static volatile uint16_t hBootCapDelayCounterM1 = 0;
static volatile uint16_t hStopPermanencyCounterM1 = 0;
volatile uint8_t bOCCheck = 0;
volatile uint16_t hOCCheckReset = 0;
volatile uint16_t hDriverCounter = 0;

uint8_t bMCBootCompleted = 0;

#if DEBUGMODE_MOTOR_CONTROL
bool bStartMotor = false;
int16_t hTorqueFinalValueTest = 0;
#endif
#if (BYPASS_POSITION_SENSOR || BYPASS_CURRENT_CONTROL)
int16_t hOpenloopTheta = 0;
int16_t hOpenloopSpeed = -10;
#endif

/* Private functions ---------------------------------------------------------*/
void MediumFrequencyTaskM1(void);
void FOC_Clear(uint8_t bMotor);
void FOC_UpdatePIDGains(uint8_t bMotor);
void FOC_InitAdditionalMethods(uint8_t bMotor);
void FOC_CalcCurrRef(uint8_t bMotor);
static uint32_t FOC_CurrControllerM1(void);
void SetChargeBootCapDelayM1(uint16_t hTickCount);
bool ChargeBootCapDelayHasElapsedM1(void);
void SetStopPermanencyTimeM1(uint16_t hTickCount);
bool StopPermanencyTimeHasElapsedM1(void);
void SafetyTask_PWMOFF(uint8_t motor);
void PWMCurrFdbk_IqdMovingAverage(FOCVars_t * pHandle);
bool IsPhaseCableDisconnected(FOCVars_t * pHandle, int16_t MechSpeed);

/**
 * @brief It initializes the whole MC core according to user defined
 *        parameters.
 * @retval None
 */
void MC_BootUp(void)
{
    
    MotorParameters_Init(&MotorParameters);
    
    /**************************************/
    /*    State machine initialization    */
    /**************************************/
    MCStateMachine_Init(&MCStateMachine[M1]);

    bMCBootCompleted = 0;
    pCircleLimitation[M1] = &CircleLimitationM1;
    pFieldWeakening[M1] = &MCConfig; /* only if M1 has FW */
    pFeedforward[M1] = &FeedforwardM1;      /* only if M1 has FF */

    /**********************************************************/
    /*     PWM and current sensing component initialization   */
    /**********************************************************/
    pPWMCurrFdbk[M1] = &PWMInsulCurrSensorFdbkHandleM1.Super;
    PWMInsulCurrSensorFdbk_Init(&PWMInsulCurrSensorFdbkHandleM1);

    /******************************************************/
    /*   PID component initialization: speed regulation   */
    /******************************************************/
    PID_Init(&PIDSpeedHandleM1, MotorParameters.ParametersConversion.PIDInitSpeed);
    pPIDSpeed[M1] = &PIDSpeedHandleM1;

    /******************************************************/
    /*     Main speed sensor component initialization     */
    /******************************************************/
    #if VEHICLE_SELECTION == VEHICLE_QUIETKAT || VEHICLE_SELECTION == VEHICLE_E_CELLS
    //used to update wTorqueSlopePerSecondUp with the correct
    //value, based on the auto master/slave detection.
    if (VcAutodeter_GetGnrState())
    {
        SpeednTorqCtrlM1.wTorqueSlopePerSecondUp = MASTER_DEFAULT_TORQUE_SLOPE_UP;
    }
    else
    {
        SpeednTorqCtrlM1.wTorqueSlopePerSecondUp = SLAVE_DEFAULT_TORQUE_SLOPE_UP;
    }
    #endif
    pSpeedTorqCtrl[M1] = &SpeednTorqCtrlM1;
    HallPosSensor_Init(&HallPosSensorM1, MotorParameters);
    RotorPosObs_Init(&RotorPosObsM1, MotorParameters);

    /******************************************************/
    /*   Speed & torque component initialization          */
    /******************************************************/
    SpdTorqCtrl_Init(pSpeedTorqCtrl[M1], pPIDSpeed[M1], &RotorPosObsM1.Super, &TempSensorControllerM1, &TempSensorMotorM1, MotorParameters);

    /******************************************************/
    /*  Auxiliary speed sensor component initialization   */
    /******************************************************/
    BemfObsPll_Init(&BemfObserverPllM1, MotorParameters);

    /********************************************************/
    /*     PID component initialization: current regulation */
    /********************************************************/
    PID_Init(&PIDIqHandleM1, MotorParameters.ParametersConversion.PIDInitIq);
    PID_Init(&PIDIdHandleM1, MotorParameters.ParametersConversion.PIDInitId);
    pPIDIq[M1] = &PIDIqHandleM1;
    pPIDId[M1] = &PIDIdHandleM1;

    /********************************************************/
    /*     Bus voltage sensor component initialization      */
    /********************************************************/
    pBusSensorM1 = &RealBusVoltageSensorParamsM1;
    ResDivVbusSensor_Init(pBusSensorM1);

    /*************************************************/
    /*   Power measurement component initialization  */
    /*************************************************/
    pMotorPower[M1] = &PQDMotorPowMeasM1;
    pMotorPower[M1]->pVBS = &(pBusSensorM1->Super);
    pMotorPower[M1]->pFOCVars = &FOCVars[M1];

    /*******************************************************/
    /*   Temperature measurement component initialization  */
    /*******************************************************/
    NTCTempSensor_Init(&TempSensorControllerM1, MotorParameters.ParametersConversion.HeatsinkNTCInit, DEFAULT_TEMP_CONTROLLER);
    pTemperatureSensorController[M1] = &TempSensorControllerM1;
    NTCTempSensor_Init(&TempSensorMotorM1, MotorParameters.ParametersConversion.MotorNTCInit, DEFAULT_TEMP_MOTOR);
    pTemperatureSensorMotor[M1] = &TempSensorMotorM1;
    initMotorMixedSignal(MotorParameters);
    /*******************************************************/
    /*     Motor Control component initialization         */
    /*******************************************************/
    PID_Init(&PIDMotorControlM1, MotorParameters.ParametersConversion.PIDInitMotorControl);
    MotorControl_Init(pFieldWeakening[M1], pPIDSpeed[M1], &PIDMotorControlM1, MotorParameters);

    /*******************************************************/
    /*     Feed forward component initialization           */
    /*******************************************************/
    Feedforward_Init(pFeedforward[M1], &(pBusSensorM1->Super), pPIDId[M1], pPIDIq[M1]);

    /*******************************************************/
    /*     MCTuning & FOC variables initialization         */
    /*******************************************************/
    FOC_Clear(M1);
    FOCVars[M1].bDriveInput = INTERNAL;
    oMCInterface[M1] = &MCInterface[M1];
    
    
    MCInterface_Init(oMCInterface[M1], &MCStateMachine[M1], pSpeedTorqCtrl[M1], &FOCVars[M1], pBusSensorM1, &MCConfig);
    
    /* Section where we initialise conversion factors that need to be available to vehicle control */
    oMCInterface[M1]->MCIConvFactors.Gain_Torque_IQRef = MotorParameters.ParametersConversion.fGainTorqueIqRef;
    oMCInterface[M1]->MCIConvFactors.MaxMeasurableCurrent = MAX_MEASURABLE_CURRENT;
    /***********************************************************************************************/
    
    MCTuning[M1].pPIDSpeed = pPIDSpeed[M1];
    MCTuning[M1].pPIDIq = pPIDIq[M1];
    MCTuning[M1].pPIDId = pPIDId[M1];
    MCTuning[M1].pPIDMotorControl = &PIDMotorControlM1; /* only if M1 has FW */
    MCTuning[M1].pPWMnCurrFdbk = pPWMCurrFdbk[M1];
    MCTuning[M1].pSpeedSensorMain = (SpdPosFdbkHandle_t *)&RotorPosObsM1;
    MCTuning[M1].pSpeedSensorAux = (SpdPosFdbkHandle_t *)&BemfObserverPllM1;
    MCTuning[M1].pSpeedSensorVirtual = MC_NULL;
    MCTuning[M1].pSpeednTorqueCtrl = pSpeedTorqCtrl[M1];
    MCTuning[M1].pStateMachine = &MCStateMachine[M1];
    MCTuning[M1].pTemperatureSensorController = (NTCTempSensorHandle_t *)pTemperatureSensorController[M1];
    MCTuning[M1].pTemperatureSensorMotor = (NTCTempSensorHandle_t *)pTemperatureSensorMotor[M1];
    MCTuning[M1].pBusVoltageSensor = &(pBusSensorM1->Super);
    MCTuning[M1].pMotorPower = (MotorPowerMeasHandle_t *)pMotorPower[M1];
    MCTuning[M1].pFieldWeakening = pFieldWeakening[M1];
    MCTuning[M1].pFeedforward = pFeedforward[M1];

    /*******************************************************/
    /*     Dynamic PI lookup table initialization         */
    /*******************************************************/
    LookupTable_Init(&LookupTableM1IqKp);
    LookupTable_Init(&LookupTableM1IqKi);
    LookupTable_Init(&LookupTableM1IdKp);
    LookupTable_Init(&LookupTableM1IdKi);
    
    /*******************************************************/
    /*     Motor tuner initialization         */
    /*******************************************************/
    #if AUTOTUNE_ENABLE
    R_AID_Init(REGULATION_EXECUTION_RATE, 1.0f/(float)SPEED_LOOP_FREQUENCY_HZ);
    R_AID_ConfigEnableVolterrID(); // Enable the voltage error measurement procedure.
    R_AID_ConfigSetVolterrCrntStep(10);       
    pAutoTune.TuningStatus=0;
    pAutoTune.bStartTuning=false;
    MCTask_InitTuning();
    #endif
    
    bMCBootCompleted = 1;
}

void MC_RunMotorControlTasks(void)
{
    if (bMCBootCompleted)
    {
        /* ** Medium Frequency Tasks ** */
        MC_Scheduler();
    }
}

void MC_Scheduler(void)
{
    if (bMCBootCompleted == 1)
    {
        if (hMFTaskCounterM1 > 0u)
        {
            hMFTaskCounterM1--;
        }
        else
        {
            //period of execution is 1 ms.
            MediumFrequencyTaskM1();
            hMFTaskCounterM1 = MF_TASK_OCCURENCE_TICKS;
        }
        if (hBootCapDelayCounterM1 > 0u)
        {
            hBootCapDelayCounterM1--;
        }
        if (hStopPermanencyCounterM1 > 0u)
        {
            hStopPermanencyCounterM1--;
        }
    }
    else
    {
    }
}

/**
 * @brief Executes medium frequency periodic Motor Control tasks
 *
 * This function performs some of the control duties on Motor 1 according to the
 * present state of its state machine. In particular, duties requiring a periodic
 * execution at a medium frequency rate (such as the speed controller for instance)
 * are executed here.
 */
void MediumFrequencyTaskM1(void)
{
    MotorState_t StateM1;
    int16_t wAux = 0;
    if (hOCCheckReset < OCD2_TIMER)
    {
        hOCCheckReset++;
    }
    else if (bOCCheck < OCD2_MAX)
    {
        hOCCheckReset = 0;
        bOCCheck = 0;
    }
    
    #if HSLOG_BUTTON_LOG
    (void) BemfObsPll_CalcAvrgMecSpeedUnit(&BemfObserverPllM1, &wAux);
        
     if (!uCAL_GPIO_Read(REVERSE_GPIO_PIN))
     {
        LogHS_StopLog(&LogHS_handle);
     }
    #endif

   (void)HallPosSensor_CalcAvrgMecSpeedUnit(&HallPosSensorM1, &wAux);
   bool bIsSpeedReliable = RotorPosObs_CalcMecSpeedUnit(&RotorPosObsM1, &wAux);
    MotorPowerQD_CalcElMotorPower(pMotorPower[M1]);
#if DYNAMIC_CURRENT_CONTROL_PID
    FOC_UpdatePIDGains(M1);
#endif

    StateM1 = MCStateMachine_GetState(&MCStateMachine[M1]);
    switch (StateM1)
    {
    case M_IDLE:
#if DEBUGMODE_MOTOR_CONTROL
        if (bStartMotor)
        {
            MCInterface_ExecTorqueRamp(oMCInterface[M1], hTorqueFinalValueTest);
            MCStateMachine_NextState(&MCStateMachine[M1], M_IDLE_START);
        }
#endif
#if AUTOTUNE_ENABLE
        if ((MotorParameters.bAutotuneEnable == true) && (pAutoTune.bStartTuning == 1))
        {      
            PWMInsulCurrSensorFdbk_TurnOnLowSides(pPWMCurrFdbk[M1]);
            PWMInsulCurrSensorFdbk_SwitchOnPWM(pPWMCurrFdbk[M1]);            
            MCInterface_StartMotorTuning(oMCInterface[M1]);
            PWMInsulCurrSensorFdbkHandleM1.wPhaseAOffset=UINT16_MAX/2;
            PWMInsulCurrSensorFdbkHandleM1.wPhaseBOffset=UINT16_MAX/2;
            pPWMCurrFdbk[M1]->IaFilter.pIIRFAInstance =NULL;
            pPWMCurrFdbk[M1]->IbFilter.pIIRFAInstance =NULL;
        }
 #endif    
        if (MCInterface->bDriverEn == true)
        {
            Driver_Disable(&MCInterface->bDriverEn);
        }
        
        //check for whether motor temp is in foldback region
        if (NTCTempSensor_CalcAvTemp(pTemperatureSensorMotor[M1]) == NTC_FOLDBACK)
        {
            MCStateMachine_WarningHandling(&MCStateMachine[M1], MC_FOLDBACK_TEMP_MOTOR, 0);    //Report the warning
        }
        else
        {
            MCStateMachine_WarningHandling(&MCStateMachine[M1], 0, MC_FOLDBACK_TEMP_MOTOR);    //Clear the warning
        }
        
        //check for whether controller temp is in foldback region
        if (NTCTempSensor_CalcAvTemp(pTemperatureSensorController[M1]) == NTC_FOLDBACK)
        {
            MCStateMachine_WarningHandling(&MCStateMachine[M1], MC_FOLDBACK_TEMP_CONTROLLER, 0);    //Report the warning
        }
        else
        {
            MCStateMachine_WarningHandling(&MCStateMachine[M1], 0, MC_FOLDBACK_TEMP_CONTROLLER);    //Clear the warning
        }
        
        //check if NTC is disconnected
        if (NTCTempSensor_CalcAvTemp(pTemperatureSensorMotor[M1]) == NTC_DISC)
        {
            MCStateMachine_WarningHandling(&MCStateMachine[M1], MC_NTC_DISC_FREEZE_MOTOR, 0);  //Report the warning
        }
        else
        {
            MCStateMachine_WarningHandling(&MCStateMachine[M1], 0, MC_NTC_DISC_FREEZE_MOTOR);  //Clear the warning
        }
        break;

    case M_IDLE_START:
        PWMInsulCurrSensorFdbk_TurnOnLowSides(pPWMCurrFdbk[M1]);
        SetChargeBootCapDelayM1(CHARGE_BOOT_CAP_TICKS);
        MCStateMachine_NextState(&MCStateMachine[M1], M_CHARGE_BOOT_CAP);
        break;

    case M_CHARGE_BOOT_CAP:
        if (ChargeBootCapDelayHasElapsedM1())
        {
            MCStateMachine_NextState(&MCStateMachine[M1], M_OFFSET_CALIB);
        }
        break;

    case M_OFFSET_CALIB:
        if (PWMCurrFdbk_CurrentReadingCalibr(pPWMCurrFdbk[M1]))
        {
            MCStateMachine_NextState(&MCStateMachine[M1], M_CLEAR);
        }
        break;

    case M_CLEAR:
        HallPosSensor_Clear(&HallPosSensorM1);
        BemfObsPll_Clear(&BemfObserverPllM1);
        RotorPosObs_Clear(&RotorPosObsM1);
        Clear_MotorStuckReverse(&pSpeedTorqCtrl[M1]->StuckProtection);
        if (MCStateMachine_NextState(&MCStateMachine[M1], M_START) == true)
        {
            FOC_Clear(M1);
            PWMInsulCurrSensorFdbk_SwitchOnPWM(pPWMCurrFdbk[M1]);
        }
        break;

    case M_START:
        MCStateMachine_NextState(&MCStateMachine[M1], M_START_RUN);
        break;

    case M_START_RUN:
        FOC_InitAdditionalMethods(M1);
        FOC_CalcCurrRef(M1);

        MCStateMachine_NextState(&MCStateMachine[M1], M_RUN);
        SpdTorqCtrl_ForceSpeedReferenceToCurrentSpeed(pSpeedTorqCtrl[M1]); /* Init the reference speed to current speed */
        MCInterface_ExecBufferedCommands(oMCInterface[M1]);                /* Exec the speed ramp after changing of the speed sensor */
#if !(BYPASS_POSITION_SENSOR)
        if (Check_MotorStuckReverse(&pSpeedTorqCtrl[M1]->StuckProtection, pSpeedTorqCtrl[M1]->hFinalTorqueRef, 
                                    pSpeedTorqCtrl[M1]->hBusVoltage, pSpeedTorqCtrl[M1]->pSPD->hAvrMecSpeedUnit)
                                    != MC_NO_FAULTS)
        {
            MCStateMachine_FaultProcessing(&MCStateMachine[M1], MC_MSRP, 0);    //Report the Fault and change bstate to FaultNow
        }
#endif
        
#if HSLOG_ZEROSPEED_LOG
        LogHS_StartOneShot(&LogHS_handle);
#endif

        break;

    case M_RUN:
#if DEBUGMODE_MOTOR_CONTROL
        if (!bStartMotor)
        {
            MCStateMachine_NextState(&MCStateMachine[M1], M_ANY_STOP);
        }
        MCInterface_ExecTorqueRamp(oMCInterface[M1], hTorqueFinalValueTest);
#endif
        MCInterface_ExecBufferedCommands(oMCInterface[M1]);
        FOC_CalcCurrRef(M1);

        if (IsPhaseCableDisconnected(MCInterface->pFOCVars, MCInterface->pSpeedTorqCtrl->pSPD->hAvrMecSpeedUnit) == true)
        {
            // raise MC_PHASE_DISC error if the ratio of measured Iqd and reference Iq is not reasoble
            MCStateMachine_WarningHandling(&MCStateMachine[M1], MC_PHASE_DISC, 0);    //Report the warning
        }
        else
        {
            MCStateMachine_WarningHandling(&MCStateMachine[M1], 0, MC_PHASE_DISC);    //Clear the warning
        }
        //check for whether motor temp is in foldback region
        if (NTCTempSensor_CalcAvTemp(pTemperatureSensorMotor[M1]) == NTC_FOLDBACK)
        {
            MCStateMachine_WarningHandling(&MCStateMachine[M1], MC_FOLDBACK_TEMP_MOTOR, 0);    //Report the warning
        }
        else
        {
            MCStateMachine_WarningHandling(&MCStateMachine[M1], 0, MC_FOLDBACK_TEMP_MOTOR);    //Clear the warning
        }
        
        //check for whether controller temp is in foldback region
        if (NTCTempSensor_CalcAvTemp(pTemperatureSensorController[M1]) == NTC_FOLDBACK)
        {
            MCStateMachine_WarningHandling(&MCStateMachine[M1], MC_FOLDBACK_TEMP_CONTROLLER, 0);    //Report the warning
        }
        else
        {
            MCStateMachine_WarningHandling(&MCStateMachine[M1], 0, MC_FOLDBACK_TEMP_CONTROLLER);    //Clear the warning
        }
        
        //check if NTC is disconnected
        if (NTCTempSensor_CalcAvTemp(pTemperatureSensorMotor[M1]) == NTC_DISC)
        {
            MCStateMachine_WarningHandling(&MCStateMachine[M1], MC_NTC_DISC_FREEZE_MOTOR, 0);    //Report the warning
        }
        else
        {
            MCStateMachine_WarningHandling(&MCStateMachine[M1], 0, MC_NTC_DISC_FREEZE_MOTOR);    //Clear the warning
        }

#if !(BYPASS_POSITION_SENSOR)    
        if (Check_MotorStuckReverse(&pSpeedTorqCtrl[M1]->StuckProtection, pSpeedTorqCtrl[M1]->hFinalTorqueRef, 
                                    pSpeedTorqCtrl[M1]->hBusVoltage, pSpeedTorqCtrl[M1]->pSPD->hAvrMecSpeedUnit)
                                    != MC_NO_FAULTS)  
        { 
            MCStateMachine_FaultProcessing(&MCStateMachine[M1], MC_MSRP, 0);    //Report the Fault and change bstate to FaultNow
        }            
#endif

#if !(BYPASS_POSITION_SENSOR || BYPASS_CURRENT_CONTROL)
        if (!bIsSpeedReliable)
        {
            //MCStateMachine_FaultProcessing(&MCStateMachine[M1], MC_SPEED_FDBK, 0);
        }
#endif
        break;

    case M_ANY_STOP:
        PWMInsulCurrSensorFdbk_SwitchOffPWM(pPWMCurrFdbk[M1]);
        if (MCInterface->bDriverEn == true)
        {
            Driver_Disable(&MCInterface->bDriverEn);
        }
        FOC_Clear(M1);
        MotorPowMeas_Clear((MotorPowerMeasHandle_t *)pMotorPower[M1]);
        SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);

        MCStateMachine_NextState(&MCStateMachine[M1], M_STOP);
        break;

    case M_STOP:
        if (StopPermanencyTimeHasElapsedM1())
        {
            MCStateMachine_NextState(&MCStateMachine[M1], M_STOP_IDLE);
        }
        break;

    case M_STOP_IDLE:
        MCStateMachine_NextState(&MCStateMachine[M1], M_IDLE);
        break;
#if AUTOTUNE_ENABLE
    case M_AUTOTUNE_ENTER_IDENTIFICATION:    
        MC_ConfigureMotorTuner();
    
        R_AID_CmdStart();
        pAutoTune.TuningStatus = TUNING_START;
        MCStateMachine_NextState(&MCStateMachine[M1], M_AUTOTUNE_IDENTIFICATION);
        break;
    
    case M_AUTOTUNE_IDENTIFICATION:
        R_AID_SpeedCtrlISR();
        pAutoTune.TuningStatus = TUNING_IN_PROGRESS;
        MC_UpdateMotorTunerOutput(pAutoTune.MotorTunerInput.DCBusVoltage);
        // Check if the autotune is completed or has an error.
        if (R_AID_GetSystemStatus() == AID_STATUS_COMPLETED || R_AID_GetSystemStatus() == AID_STATUS_ERROR )
        {
            MCInterface_StopMotorTuning(oMCInterface[M1]); // Return to normal motor control mode
            if (R_AID_GetSystemStatus() == AID_STATUS_ERROR)
            {
                pAutoTune.TuningStatus = TUNING_ERROR;
            }
        }
        
        break;
    
    case M_AUTOTUNE_ANY_STOP_IDENTIFICATION:
        R_AID_CmdStop();
        PWMInsulCurrSensorFdbk_SwitchOffPWM(pPWMCurrFdbk[M1]);
        SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
        MCStateMachine_NextState(&MCStateMachine[M1], M_AUTOTUNE_STOP_IDENTIFICATION);
        break;
    
    case M_AUTOTUNE_STOP_IDENTIFICATION:
        if (StopPermanencyTimeHasElapsedM1())
        {
            if (pAutoTune.TuningStatus != TUNING_ERROR)
            {
                pAutoTune.TuningStatus = TUNING_DONE;
            }
            pAutoTune.bStartTuning = false;
            MCStateMachine_NextState(&MCStateMachine[M1], M_IDLE);
        }
        break;
#endif
    default:
        break;
    }
}

/**
 * @brief It re-initializes the current and voltage variables. Moreover
 *        it clears qd currents PI controllers, voltage sensor and SpeednTorque
 *        controller. It must be called before each motor restart.
 *        It does not clear speed sensor.
 * @param  bMotor related motor it can be M1 or M2
 * @retval none
 */
void FOC_Clear(uint8_t bMotor)
{
    ab_t NULL_ab = {(int16_t)0, (int16_t)0};
    qd_t NULL_qd = {(int16_t)0, (int16_t)0};
    AlphaBeta_t NULL_alphabeta = {(int16_t)0, (int16_t)0};

    FOCVars[bMotor].Iab = NULL_ab;
    FOCVars[bMotor].Ialphabeta = NULL_alphabeta;
    FOCVars[bMotor].Iqd = NULL_qd;
    FOCVars[bMotor].Iqdref = NULL_qd;
    FOCVars[bMotor].hTeref = (int16_t)0;
    FOCVars[bMotor].Vqd = NULL_qd;
    FOCVars[bMotor].Valphabeta = NULL_alphabeta;
    FOCVars[bMotor].hElAngle = (int16_t)0;

    PID_SetIntegralTerm(pPIDIq[bMotor], (int32_t)0);
    PID_SetIntegralTerm(pPIDId[bMotor], (int32_t)0);

    SpdTorqCtrl_Clear(pSpeedTorqCtrl[bMotor]);

    PWMCurrFdbk_SwitchOffPWM(pPWMCurrFdbk[bMotor]);

    if (pFieldWeakening[bMotor])
    {
        FluxWkng_Clear(pFieldWeakening[bMotor]);
    }
    if (pFeedforward[bMotor])
    {
        Feedforward_Clear(pFeedforward[bMotor]);
    }
}

void FOC_UpdatePIDGains(uint8_t bMotor)
{
    SpdPosFdbkHandle_t *SpeedHandle;
    SpeedHandle = SpdTorqCtrl_GetSpeedSensor(pSpeedTorqCtrl[bMotor]);

    int16_t hM1SpeedUnit = SpdPosFdbk_GetAvrgMecSpeedUnit(SpeedHandle);

/*  The PI controller continues the PWM for few seconds after releasing the Throttle
    which can be seen when there is no load. So solve, we are updating PI parameters 
    to perevent such situation. for now only Velec. For other bikes first we should do
    the tests */
  
  
  

#if MOTOR_SELECTION == MOTOR_TSUGAWA_L13S5_350W    
    if (FOCVars[bMotor].hTeref == 0.0)
    {
        PID_SetKI(pPIDIq[bMotor], No_Load_PID_KIq_Gain);
        
    }
    else
    {
        PID_SetKI(pPIDIq[bMotor], (int16_t)LookupTable_CalcOutput(&LookupTableM1IqKi, abs(hM1SpeedUnit)));
    }
#endif  
    
    PID_SetKP(pPIDIq[bMotor], (int16_t)LookupTable_CalcOutput(&LookupTableM1IqKp, abs(hM1SpeedUnit)));
    PID_SetKI(pPIDIq[bMotor], (int16_t)LookupTable_CalcOutput(&LookupTableM1IqKi, abs(hM1SpeedUnit)));    
    PID_SetKP(pPIDId[bMotor], (int16_t)LookupTable_CalcOutput(&LookupTableM1IdKp, abs(hM1SpeedUnit)));
    PID_SetKI(pPIDId[bMotor], (int16_t)LookupTable_CalcOutput(&LookupTableM1IdKi, abs(hM1SpeedUnit)));
    
#if MOTOR_SELECTION == MOTOR_AKM_128SX_750W    
    // this PID update is for correct IqKI for imidietly stop when release trottle
    if (FOCVars[bMotor].hTeref == 0.0)
    {
        PID_SetKI(pPIDIq[bMotor], No_Load_PID_KIq_Gain);
    }
    else
    {
        PID_SetKI(pPIDIq[bMotor], (int16_t)LookupTable_CalcOutput(&LookupTableM1IqKi, abs(hM1SpeedUnit)));
    }
#endif
    
#if MOTOR_SELECTION == MOTOR_AKM_128SX_500W    
    // this PID update is for correct IqKI for imidietly stop when release trottle
    if (FOCVars[bMotor].hTeref == 0.0)
    {
        PID_SetKI(pPIDIq[bMotor], No_Load_PID_KIq_Gain);
    }
    else
    {
        PID_SetKI(pPIDIq[bMotor], (int16_t)LookupTable_CalcOutput(&LookupTableM1IqKi, abs(hM1SpeedUnit)));
    }
#endif
    
#if MOTOR_SELECTION == MOTOR_AKM_128SX_350W    
    // this PID update is for correct IqKI for imidietly stop when release trottle
    if (FOCVars[bMotor].hTeref == 0.0)
    {
        PID_SetKI(pPIDIq[bMotor], No_Load_PID_KIq_Gain);
    }
    else
    {
        PID_SetKI(pPIDIq[bMotor], (int16_t)LookupTable_CalcOutput(&LookupTableM1IqKi, abs(hM1SpeedUnit)));
    }
#endif

#if MOTOR_SELECTION == MOTOR_GHR_0194_DD    
    // this PID update is for correct IqKI for imidietly stop when release trottle
    if (FOCVars[bMotor].hTeref == 0.0)
    {
        PID_SetKI(pPIDIq[bMotor], No_Load_PID_KIq_Gain);
        PID_SetKP(pPIDIq[bMotor], No_Load_PID_KIq_Gain);
    }
    else
    {
        PID_SetKI(pPIDIq[bMotor], (int16_t)LookupTable_CalcOutput(&LookupTableM1IqKi, abs(hM1SpeedUnit)));
    }
#endif
}

/**
 * @brief  Use this method to initialize additional methods (if any) in
 *         START_TO_RUN state
 * @param  bMotor related motor it can be M1 or M2
 * @retval none
 */
void FOC_InitAdditionalMethods(uint8_t bMotor)
{
    if (pFeedforward[bMotor])
    {
        Feedforward_InitFOCAdditionalMethods(pFeedforward[bMotor]);
    }
}

/**
 * @brief It computes the new values of Iqdref (current references on qd
 *        reference frame) based on the required electrical torque information
 *        provided by oTSC object (internally clocked).
 *        If implemented in the derived class it executes flux weakening and/or
 *        MTPA algorithm(s). It must be called with the periodicity specified
 *        in oTSC parameters
 * @param  bMotor related motor it can be M1 or M2
 * @retval none
 */
void FOC_CalcCurrRef(uint8_t bMotor)
{
    qd_t IqdTmp;
    int16_t delta_t;
  
    delta_t = -2;
  
    /* update Max Power based on DC Voltage */
    pSpeedTorqCtrl[bMotor]->hBusVoltage = VbusSensor_GetAvBusVoltageVolt(pMotorPower[bMotor]->pVBS);
    MC_AdaptiveMaxPower(pSpeedTorqCtrl[bMotor]);    
    
    /* If current references iqref and idref are computed internally    */
    if (FOCVars[bMotor].bDriveInput == INTERNAL)
    {
        FOCVars[bMotor].hTeref = SpdTorqCtrl_CalcTorqueReference(pSpeedTorqCtrl[bMotor], MotorParameters);
        FOCVars[bMotor].Iqdref.q = SpdTorqCtrl_GetIqFromTorqueRef(pSpeedTorqCtrl[bMotor], FOCVars[bMotor].hTeref);
        FOCVars[bMotor].Iqdref.d = SpdTorqCtrl_GetIdFromTorqueRef(pSpeedTorqCtrl[bMotor], FOCVars[bMotor].hTeref);
        
        /* Use Flux Weakening to recalculate Iq and Id if it is enabled for motor */
        if (pSpeedTorqCtrl[bMotor]->bFluxWeakeningEn == true)    
        {
            IqdTmp.q = FOCVars[bMotor].Iqdref.q;
            IqdTmp.d = FOCVars[bMotor].Iqdref.d;
            FOCVars[bMotor].Iqdref = FluxWkng_CalcCurrRef(pFieldWeakening[bMotor],IqdTmp);
        }
        
        /* apply the maximum nominal limitation to the Iq */
        SpdTorqCtrl_ApplyCurrentLimitation_Iq(&FOCVars[bMotor].Iqdref, MCConfig.hNominalCurr, MCConfig.wUsrMaxCurr);
        
  //=================================APPLY REGENERATIVE CURRENT ========================================      
        if (pSpeedTorqCtrl[bMotor]->motorType == DIRECT_DRIVE) // providing negative current for regen
        {
            if ((MCInterface->Iqdref.q == 0) && (MCInterface->Iqdref.d == 0) && (MCInterface->hFinalTorque == 0) && (MCInterface->bDriverEn == true))
            {
                if (abs(pSpeedTorqCtrl[M1]->pSPD->hAvrMecSpeedUnit) > MIN_REGEN_SPEED)
                {
                    if (FOCVars[M1].I_regen < IQ_REGEN)
                    {
                        FOCVars[M1].I_regen = IQ_REGEN;  
                    }
                    else
                    {
                        FOCVars[M1].I_regen = FOCVars[M1].I_regen + delta_t;
                    }
                  
                    FOCVars[M1].Iqdref.q =  FOCVars[M1].I_regen;
                    FOCVars[M1].Iqdref.d = 0;
                }
                else
                {
                    FOCVars[M1].I_regen = 0;
                }
                
            }   
        }
 // ========================================================================================================
        
        if (pFeedforward[bMotor])
        {
            Feedforward_VqdffComputation(pFeedforward[bMotor], FOCVars[bMotor].Iqdref, pSpeedTorqCtrl[bMotor]);
        }
    }

    /* If current references iqref and idref are computed externaly    */
    else if (FOCVars[bMotor].bDriveInput == EXTERNAL)    //Added to calculate FlxWeakening parameters in SWD Debug mode
    {
        IqdTmp.q = FOCVars[bMotor].Iqdref.q;
        IqdTmp.d = FOCVars[bMotor].Iqdref.d;
        FOCVars[bMotor].Iqdref = FluxWkng_CalcCurrRef(pFieldWeakening[bMotor],IqdTmp);
    }
}

/**
 * @brief It set a counter intended to be used for counting the delay required
 *        for drivers boot capacitors charging of motor 1
 * @param  hTickCount number of ticks to be counted
 * @retval void
 */
void SetChargeBootCapDelayM1(uint16_t hTickCount)
{
    hBootCapDelayCounterM1 = hTickCount;
}

/**
 * @brief Use this function to know whether the time required to charge boot
 *        capacitors of motor 1 has elapsed
 * @param none
 * @retval bool true if time has elapsed, false otherwise
 */
bool ChargeBootCapDelayHasElapsedM1(void)
{
    bool retVal = false;
    if (hBootCapDelayCounterM1 == 0)
    {
        retVal = true;
    }
    return (retVal);
}

/**
 * @brief It set a counter intended to be used for counting the permanency
 *        time in STOP state of motor 1
 * @param  hTickCount number of ticks to be counted
 * @retval void
 */
void SetStopPermanencyTimeM1(uint16_t hTickCount)
{
    hStopPermanencyCounterM1 = hTickCount;
}

/**
 * @brief Use this function to know whether the permanency time in STOP state
 *        of motor 1 has elapsed
 * @param  none
 * @retval bool true if time is elapsed, false otherwise
 */
bool StopPermanencyTimeHasElapsedM1(void)
{
    bool retVal = false;
    if (hStopPermanencyCounterM1 == 0)
    {
        retVal = true;
    }
    return (retVal);
}

/**
 * @brief Executes the Motor Control duties that require a high frequency rate and a precise timing
 *
 * This is mainly the FOC current control loop. It is executed depending on the state of the Motor Control
 * subsystem (see the state machine(s)).
 *
 * @retval Number of the motor instance which FOC loop was executed.
 */
uint8_t MC_HighFrequencyTask(void)
{

    uint8_t bMotorNbr = 0;
    uint32_t wFOCreturn;

    BemfObserverInputs_t BemfObsInputs;
    BemfObsInputs.Valfa_beta = FOCVars[M1].Valphabeta;

    HallPosSensor_CalcElAngle(&HallPosSensorM1);
    RotorPosObs_CalcElAngle(&RotorPosObsM1, 0);
    
#if AUTOTUNE_ENABLE
    if(MotorParameters.bAutotuneEnable == true)
    {
        Autotune_CalcPhaseCurrents(pPWMCurrFdbk[M1]);
        Driver_Enable(&MCInterface->bDriverEn);
        MotorState_t StateM1;
        StateM1 = MCStateMachine_GetState(&MCStateMachine[M1]);
        if (StateM1 == M_AUTOTUNE_IDENTIFICATION || StateM1 == M_AUTOTUNE_ENTER_IDENTIFICATION || StateM1 == M_AUTOTUNE_ANY_STOP_IDENTIFICATION)
        {
            R_AID_CurrentCtrlISR();
        }
    }
#else
    if (false)
    {
    }
#endif
    else
    {
        // Check if the Hall Sensors are disconneted and raise the error
        if (HallSensor_IsDisconnected(&HallPosSensorM1) == true)
        {
            MCStateMachine_WarningHandling(&MCStateMachine[M1], MC_HALL_DISC, 0);
        }
        else
        {
            MCStateMachine_WarningHandling(&MCStateMachine[M1], 0, MC_HALL_DISC);
        }
        
        /* here the code is checking last 16 records of direction, 
        if all last 16 records show vibration, then rasie the stuck protection error */
        if ((HallPosSensorM1.wDirectionChangePattern & 0xFFFF) == VIBRATION_PATTERN)
        {
            MCStateMachine_FaultProcessing(&MCStateMachine[M1], MC_MSRP, 0);    //Report the Fault and change bstate to FaultNow
        }
        
        wFOCreturn = FOC_CurrControllerM1();
        if (wFOCreturn == MC_FOC_DURATION)
        {
            MCStateMachine_FaultProcessing(&MCStateMachine[M1], MC_FOC_DURATION, 0);
        }
        else
        {
            //        BemfObsInputs.Ialfa_beta = FOCVars[M1].Ialphabeta;
            //        BemfObsInputs.Vbus = VbusSensor_GetAvBusVoltageDigital(&(pBusSensorM1->Super));
            //        BemfObsPll_CalcElAngle (&BemfObserverPllM1, &BemfObsInputs);
            //        BemfObsPll_CalcAvrgElSpeedDpp (&BemfObserverPllM1);
        } 

        #if LOGMOTORVALS
        LogHS_LogMotorVals(&LogHS_handle); //High speed logging, if disable function does a run through
        #endif
        #if LOGMOTORVALSRES
        LogHS_LogMotorValsVarRes(&LogHS_handle); //High speed logging, if disable function does a run through
        #endif
    }
    
    return bMotorNbr;
}

/**
 * @brief It executes the core of FOC drive that is the controllers for Iqd
 *        currents regulation. Reference frame transformations are carried out
 *        accordingly to the active speed sensor. It must be called periodically
 *        when new motor currents have been converted
 * @param this related object of class CFOC.
 * @retval int16_t It returns MC_NO_FAULTS if the FOC has been ended before
 *                 next PWM Update event, MC_FOC_DURATION otherwise
 */
inline uint32_t FOC_CurrControllerM1(void)
{
    qd_t Iqd = {0}, Vqd = {0};
    ab_t Iab = {0};
    AlphaBeta_t Ialphabeta = {0}, Valphabeta = {0};

    int16_t hElAngle;
    uint32_t wCodeError = 0;
    SpdPosFdbkHandle_t *speedHandle;
    
    if (MCInterface->bDriverEn == false)
    {
        Driver_Enable(&MCInterface->bDriverEn);
    }
    
    speedHandle = SpdTorqCtrl_GetSpeedSensor(pSpeedTorqCtrl[M1]);
    hElAngle = SpdPosFdbk_GetElAngle(speedHandle);

#if (BYPASS_POSITION_SENSOR)
    hOpenloopTheta += hOpenloopSpeed;
    hElAngle = hOpenloopTheta;
#endif

    PWMCurrFdbk_GetPhaseCurrents(pPWMCurrFdbk[M1], &Iab);
    MotorState_t StateM1;
    StateM1 = MCStateMachine_GetState(&MCStateMachine[M1]);
    
    if (StateM1 == M_RUN || StateM1 == M_ANY_STOP)
    {
        Ialphabeta = MCMath_Clarke(Iab);
        Iqd = MCMath_Park(Ialphabeta, hElAngle);

        Vqd.q = PI_Controller(pPIDIq[M1],
                              (int32_t)(FOCVars[M1].Iqdref.q) - Iqd.q);

        Vqd.d = PI_Controller(pPIDId[M1],
                              (int32_t)(FOCVars[M1].Iqdref.d) - Iqd.d);

        Vqd = Feedforward_VqdConditioning(pFeedforward[M1], Vqd);

#if (BYPASS_CURRENT_CONTROL)
        Vqd.q = 2000;
        Vqd.d = 0;
#endif

        Vqd = CircleLimitation(pCircleLimitation[M1], Vqd);
        
    if (pSpeedTorqCtrl[M1]->motorType == DIRECT_DRIVE)     
    {      

      if ((MCInterface->Iqdref.q == 0) && (MCInterface->Iqdref.d == 0) && (MCInterface->hFinalTorque == 0) && (MCInterface->bDriverEn == true))
        {
            
          if (abs(pSpeedTorqCtrl[M1]->pSPD->hAvrMecSpeedUnit) < RESET_SPEED)
          {
            
            Driver_Disable(&MCInterface->bDriverEn);
            PID_SetIntegralTerm(pPIDIq[M1], (int32_t)0);
            PID_SetIntegralTerm(pPIDId[M1], (int32_t)0);
          }
          
        }
        else if ((MCInterface->bDriverEn == false) && (MCInterface->hFinalTorque != 0))
        {
            Driver_Enable(&MCInterface->bDriverEn);
        }
      }
        
       
        
        Valphabeta = MCMath_RevPark(Vqd, hElAngle);
        
        wCodeError = PWMCurrFdbk_SetPhaseVoltage(pPWMCurrFdbk[M1], Valphabeta);

        FOCVars[M1].Vqd = Vqd;
        FOCVars[M1].Iab = Iab;
        FOCVars[M1].Ialphabeta = Ialphabeta;
        FOCVars[M1].Iqd = Iqd;
        FOCVars[M1].Valphabeta = Valphabeta;
        FOCVars[M1].hElAngle = hElAngle;
        MC_DataProcess(pFieldWeakening[M1], Vqd);
        Feedforward_DataProcess(pFeedforward[M1]);

        // Check for overcurrent condition (software overcurrent protection)
        if (PWMCurrFdbk_CheckSoftwareOverCurrent(pPWMCurrFdbk[M1], &Iab, &FOCVars[M1].Iqdref))
        {
            PWMCurrFdbk_SwitchOffPWM(pPWMCurrFdbk[M1]);
            MCStateMachine_FaultProcessing(&MCStateMachine[M1], MC_OCSP, 0);
        }

#if ENABLE_MC_DAC_DEBUGGING
        R_DAC_Write((DEBUG1_DAC_HANDLE_ADDRESS)->p_ctrl, (uint16_t)FOCVars[M1].Iqd.q + INT16_MAX);
        R_DAC_Write((DEBUG2_DAC_HANDLE_ADDRESS)->p_ctrl, (uint16_t)FOCVars[M1].Iqd.d + INT16_MAX);
#endif
    }

    return (wCodeError);
}

/**
 * @brief  Executes safety checks (e.g. bus voltage and temperature) for all drive instances.
 *
 * Faults flags are updated here.
 */
void MC_SafetyTask(void)
{
    if (bMCBootCompleted == 1)
    {
        SafetyTask_PWMOFF(M1);
    }
}

/**
 * @brief  Safety task implementation if    MC.ON_OVER_VOLTAGE == TURN_OFF_PWM
 * @param  bMotor Motor reference number defined
 *         \link Motors_reference_number here \endlink
 * @retval None
 */
void SafetyTask_PWMOFF(uint8_t bMotor)
{
    uint32_t CodeReturn = MC_NO_ERROR;
    uint32_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK};

    // Check if Controller temperature is higher than the threshold, then raise the error
    if (NTCTempSensor_CalcAvTemp(pTemperatureSensorController[bMotor]) == NTC_OT)
    {
        CodeReturn |= errMask[bMotor] & MC_OVER_TEMP_CONTROLLER;
    }

    // Check if Motor temperature is higher than the threshold, then raise the error
    if (NTCTempSensor_CalcAvTemp(pTemperatureSensorMotor[bMotor]) == NTC_OT)
    {
        CodeReturn |= errMask[bMotor] & MC_OVER_TEMP_MOTOR;
    }
    
    if (NTCTempSensor_CalcAvTemp(pTemperatureSensorController[bMotor]) == NTC_FREEZE)
    {
        CodeReturn |= errMask[bMotor] & MC_NTC_FREEZE_CONTROLLER;
    }
    
    CodeReturn |= PWMCurrFdbk_CheckOverCurrent(pPWMCurrFdbk[bMotor]);               /* check for fault. It return MC_OCD1, MC_OCD2 or MC_NO_FAULTS
                                                                                    (for STM32F30x can return MC_OVER_VOLT in case of HW Overvoltage) */
    if (((CodeReturn & MC_OCD2) == MC_OCD2) && (bOCCheck < OCD2_MAX))
    {
        bOCCheck++;
        
        if (bOCCheck == OCD2_MAX)
        {
            CodeReturn |= errMask[bMotor] & MC_OCD1;
        }
    }
    
    if (bMotor == M1)
    {
        CodeReturn |= errMask[bMotor] & ResDivVbusSensor_CalcAvVbus(pBusSensorM1);
    }
    
    MCStateMachine_FaultProcessing(&MCStateMachine[bMotor], CodeReturn, ~CodeReturn); /* Update the MCStateMachine according error code */
    switch (MCStateMachine_GetState(&MCStateMachine[bMotor]))                         /* Acts on PWM outputs in case of faults */
    {
    case M_FAULT_NOW:
        PWMCurrFdbk_SwitchOffPWM(pPWMCurrFdbk[bMotor]);
        FOC_Clear(bMotor);
        MotorPowMeas_Clear((MotorPowerMeasHandle_t *)pMotorPower[bMotor]);
        break;
    case M_FAULT_OVER:
        PWMCurrFdbk_SwitchOffPWM(pPWMCurrFdbk[bMotor]);
        break;
    default:
        break;
    }
}

/**
  * @brief  Turns off the PWM for M1
    * @param    None
    * @retval None
  */
void MC_PWM_OFF_M1()
{
    PWMCurrFdbk_SwitchOffPWM(pPWMCurrFdbk[M1]);
}

/**
 * @brief  This function returns the reference of the MCInterface relative to
 *         the selected drive.
 * @param  bMotor Motor reference number defined
 *         \link Motors_reference_number here \endlink
 * @retval MotorControlInterfaceHandle_t * Reference to MCInterface relative to the selected drive.
 *         Note: it can be MC_NULL if MCInterface of selected drive is not
 *         allocated.
 */
MotorControlInterfaceHandle_t *GetMCI(uint8_t bMotor)
{
    MotorControlInterfaceHandle_t *retVal = MC_NULL;
    if (bMotor < NBR_OF_MOTORS)
    {
        retVal = oMCInterface[bMotor];
    }
    return retVal;
}

/**
 * @brief  This function returns the reference of the MCTuning relative to
 *         the selected drive.
 * @param  bMotor Motor reference number defined
 *         \link Motors_reference_number here \endlink
 * @retval MotorControlTuningHandle_t motor control tuning handler for the selected drive.
 *         Note: it can be MC_NULL if MCInterface of selected drive is not
 *         allocated.
 */
MotorControlTuningHandle_t *GetMCT(uint8_t bMotor)
{
    MotorControlTuningHandle_t *retVal = MC_NULL;
    if (bMotor < NBR_OF_MOTORS)
    {
        retVal = &MCTuning[bMotor];
    }
    return retVal;
}

void MC_HardwareFaultTask(void)
{
    PWMInsulCurrSensorFdbk_SwitchOffPWM(pPWMCurrFdbk[M1]);
    MCStateMachine_FaultProcessing(&MCStateMachine[M1], MC_SW_ERROR, 0);
}

/* startMCMediumFrequencyTask function */
__NO_RETURN void startMCMediumFrequencyTask(void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);

    /* Infinite loop */
    for (;;)
    {
        /* delay of 500us */
        osDelay(1);

        MC_RunMotorControlTasks();
    }
}

/* startMCSafetyTask function */
__NO_RETURN void startMCSafetyTask(void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);

    /* Infinite loop */
    for (;;)
    {
        /* delay of 500us */
        osDelay(1);

        MC_SafetyTask();
    }
}


/**
 * @brief  Calculate the moving average value of last CURRENT_AVG_WIN_SIZE current samples
 * @param  pHandle pointer on the handle structure of the PWMC instance
 * @retval None
**/
void PWMCurrFdbk_IqdMovingAverage(FOCVars_t * pHandle)
{
    static uint8_t AvgIndex = 0;
    static qd_t Iqd_Samples[CURRENT_AVG_WIN_SIZE];    
    int32_t sumIq = 0, sumId = 0; 
    
    // write latest CURRENT value on the last updated value in array
    Iqd_Samples[AvgIndex]= pHandle->Iqd;
    
    // update AvgIndex to the next last updated value
    if (AvgIndex >= (CURRENT_AVG_WIN_SIZE - 1))
    {
        AvgIndex = 0;
    }
    else
    {
        AvgIndex++;
    }
    
    // calculate the average
    for (uint8_t i = 0; i < CURRENT_AVG_WIN_SIZE; i++ )
    {
        sumIq += Iqd_Samples[i].q;
        sumId += Iqd_Samples[i].d;
    }
    pHandle->Iqd_avg.q = (int16_t)(sumIq / CURRENT_AVG_WIN_SIZE);   
    pHandle->Iqd_avg.d = (int16_t)(sumId / CURRENT_AVG_WIN_SIZE);
}

/**
 * @brief  Checks if the value of IQ measured is smaller than one PHASE_WIRE_DISCONNECT_RATIO of Iqref
 * @param  pHandle pointer on the handle structure of the FOCVars
 * @retval bool: true if diconnection detected,
**/

bool IsPhaseCableDisconnected(FOCVars_t * pHandle, int16_t MechSpeed)
{
    static uint16_t Timer_Disc, Timer_Conn;

    bool retVal = false;
    PWMCurrFdbk_IqdMovingAverage(pHandle);
    uint16_t MeanSquare = (uint16_t)sqrt((pHandle->Iqd_avg.q * pHandle->Iqd_avg.q) + (pHandle->Iqd_avg.d * pHandle->Iqd_avg.d));
    
    if ((MeanSquare < (abs(pHandle->Iqdref.q))) && (abs(MechSpeed) == 0))
    {
        Timer_Disc++;
        Timer_Conn = 0;
    }
    else
    {
        Timer_Disc = 0;
        Timer_Conn++;
    }
    
    if (Timer_Disc > PHASE_WIRE_DISCONNECT_WAIT_MCCYCLE)
    {
        retVal = true;
        Timer_Conn = 0;
    }
    else if (Timer_Conn > PHASE_WIRE_DISCONNECT_WAIT_MCCYCLE)
    {
        Timer_Disc = 0;
        retVal = false;
    }
    return retVal;
}

#if AUTOTUNE_ENABLE
/*************************** TUNING ************************************/
/**
  * @brief  This function fills the initiate values to the tuning variables
  */
void MCTask_InitTuning()
{
    pAutoTune.MotorTunerInput.NumPolePairs = 8;
    pAutoTune.MotorTunerInput.RatedMotorCurrent = 30;
    pAutoTune.MotorTunerInput.DCBusVoltage = 48;
}

/**
  * @brief  This function returns the results of the last identification procedure.
  * @param  fBatteryVoltage Rated DC voltage of battery used to operate the motor.
  * @retval True if last identification was successfully completed.
  */
bool MC_UpdateMotorTunerOutput(float fBatteryVoltage)
{
    bool bRetVal = false;
    st_aid_id_setting_t IdentificationSettings;
    
    pAutoTune.MotorTunerOutput.fProgress = R_AID_GetProgress();
    pAutoTune.MotorTunerOutput.hErrorCode = R_AID_GetErrorStatus();
    pAutoTune.MotorTunerOutput.bCompleted = false;
    
    if (R_AID_GetSystemStatus() == AID_STATUS_COMPLETED)
    {        
        pAutoTune.MotorTunerOutput.bCompleted = true;
        
        R_AID_GetIDSetting(&IdentificationSettings);
        //Make a coef to convert PID gains from the autotune library range to match the range expected by our firmware.
        float fScalingKpIq = (float)(fBatteryVoltage/SQRT_3*AMPLIFICATION_GAIN/ADC_REFERENCE_VOLTAGE*PID_GetKPDivisor(pPIDIq[M1]));
        float fScalingKiIq = (float)(fBatteryVoltage/SQRT_3*AMPLIFICATION_GAIN/ADC_REFERENCE_VOLTAGE*PID_GetKIDivisor(pPIDIq[M1]));
        float fScalingKpId = (float)(fBatteryVoltage/SQRT_3*AMPLIFICATION_GAIN/ADC_REFERENCE_VOLTAGE*PID_GetKPDivisor(pPIDId[M1]));
        float fScalingKiId = (float)(fBatteryVoltage/SQRT_3*AMPLIFICATION_GAIN/ADC_REFERENCE_VOLTAGE*PID_GetKIDivisor(pPIDId[M1]));
        
        pAutoTune.MotorTunerOutput.fBatteryVoltage = fBatteryVoltage;
        pAutoTune.MotorTunerOutput.Rs = R_AID_GetResistance();
        pAutoTune.MotorTunerOutput.Ld = R_AID_GetLd();
        pAutoTune.MotorTunerOutput.Lq = R_AID_GetLq();
        pAutoTune.MotorTunerOutput.J = R_AID_GetInertia();
        pAutoTune.MotorTunerOutput.Friction = R_AID_GetFriction();
        pAutoTune.MotorTunerOutput.Ke = R_AID_GetKe();
        pAutoTune.MotorTunerOutput.IqKp = (int16_t) (fScalingKpIq*aid_f4_kp_iq);
        pAutoTune.MotorTunerOutput.IqKi = (int16_t) (fScalingKiIq*aid_f4_ki_iq);
        pAutoTune.MotorTunerOutput.IdKp = (int16_t) (fScalingKpId*aid_f4_kp_id);
        pAutoTune.MotorTunerOutput.IdKi = (int16_t) (fScalingKiId*aid_f4_ki_id);
        pAutoTune.MotorTunerOutput.RatedSpeed =  (int16_t) (fBatteryVoltage/SQRT_3/R_AID_GetKe()/(2*PI_)*_RPM);
        pAutoTune.MotorTunerOutput.RatedTorque = (int16_t) (1.5f*IdentificationSettings.u2_num_pole_pairs*IdentificationSettings.f4_rated_current*R_AID_GetKe()*100.0f);
        bRetVal = true;
    }
    
    return bRetVal;
}

/**
  *  @brief This function configure motor information for tuning
  */
void MC_ConfigureMotorTuner(void)
{   
    R_AID_ConfigMotorPlate(pAutoTune.MotorTunerInput.RatedMotorCurrent, pAutoTune.MotorTunerInput.NumPolePairs);
    R_AID_SetInitElecParams(pAutoTune.MotorTunerInput.KnownRs,
                            pAutoTune.MotorTunerInput.KnownLd,
                            pAutoTune.MotorTunerInput.KnownLq,
                            pAutoTune.MotorTunerInput.KnownMagnetFlux);
}
#endif