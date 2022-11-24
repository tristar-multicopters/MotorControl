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
#include "current_pid_vs_speed_table.h"
#include "log_high_speed.h"
#include "comm_config.h"

#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "gnr_parameters.h"

/* Private define ------------------------------------------------------------*/

#define CHARGE_BOOT_CAP_MS            10
#define CHARGE_BOOT_CAP_MS2           10
#define OFFCALIBRWAIT_MS              0
#define OFFCALIBRWAIT_MS2             0
#define STOPPERMANENCY_MS             400
#define STOPPERMANENCY_MS2            400
#define CHARGE_BOOT_CAP_TICKS    (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS)/ 1000)
#define CHARGE_BOOT_CAP_TICKS2 (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS2)/ 1000)
#define OFFCALIBRWAITTICKS         (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS)/ 1000)
#define OFFCALIBRWAITTICKS2        (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS2)/ 1000)
#define STOPPERMANENCY_TICKS     (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS)/ 1000)
#define STOPPERMANENCY_TICKS2    (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS2)/ 1000)
#define VBUS_TEMP_ERR_MASK       (uint16_t) ~(0 | MC_UNDER_VOLT | 0)

/* Private variables----------------------------------------------------------*/
FOCVars_t FOCVars[NBR_OF_MOTORS];
MotorControlInterfaceHandle_t * oMCInterface[NBR_OF_MOTORS];
MotorControlInterfaceHandle_t MCInterface[NBR_OF_MOTORS];
MotorControlTuningHandle_t MCTuning[NBR_OF_MOTORS];
MotorStateMachineHandle_t MCStateMachine[NBR_OF_MOTORS];
SpdTorqCtrlHandle_t *pSpeedTorqCtrl[NBR_OF_MOTORS];
PIDHandle_t *pPIDSpeed[NBR_OF_MOTORS];
PIDHandle_t *pPIDIq[NBR_OF_MOTORS];
PIDHandle_t *pPIDId[NBR_OF_MOTORS];
ResDivVbusSensorHandle_t *pBusSensorM1;

NTCTempSensorHandle_t *pTemperatureSensor[NBR_OF_MOTORS];
PWMCurrFdbkHandle_t * pPWMCurrFdbk[NBR_OF_MOTORS];
MotorPowerQDHandle_t *pMotorPower[NBR_OF_MOTORS];
CircleLimitationHandle_t *pCircleLimitation[NBR_OF_MOTORS];
FluxWeakeningHandle_t *pFieldWeakening[NBR_OF_MOTORS];
FeedforwardHandle_t *pFeedforward[NBR_OF_MOTORS];

static volatile uint16_t hMFTaskCounterM1 = 0;
static volatile uint16_t hBootCapDelayCounterM1 = 0;
static volatile uint16_t hStopPermanencyCounterM1 = 0;

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
static uint16_t FOC_CurrControllerM1(void);
void SetChargeBootCapDelayM1(uint16_t hTickCount);
bool ChargeBootCapDelayHasElapsedM1(void);
void SetStopPermanencyTimeM1(uint16_t hTickCount);
bool StopPermanencyTimeHasElapsedM1(void);
void SafetyTask_PWMOFF(uint8_t motor);


/**
    * @brief It initializes the whole MC core according to user defined
    *        parameters.
    * @retval None
    */
void MC_BootUp(void)
{
    /**************************************/
    /*    State machine initialization    */
    /**************************************/
    MCStateMachine_Init(&MCStateMachine[M1]);

    bMCBootCompleted = 0;
    pCircleLimitation[M1] = &CircleLimitationM1;
    pFieldWeakening[M1] = &FluxWeakeningM1; /* only if M1 has FW */
    pFeedforward[M1] = &FeedforwardM1; /* only if M1 has FF */

    /**********************************************************/
    /*     PWM and current sensing component initialization   */
    /**********************************************************/
    pPWMCurrFdbk[M1] = &PWMInsulCurrSensorFdbkHandleM1.Super;
    PWMInsulCurrSensorFdbk_Init(&PWMInsulCurrSensorFdbkHandleM1);

    /******************************************************/
    /*   PID component initialization: speed regulation   */
    /******************************************************/
    PID_Init(&PIDSpeedHandleM1);
    pPIDSpeed[M1] = &PIDSpeedHandleM1;

    /******************************************************/
    /*     Main speed sensor component initialization     */
    /******************************************************/
    pSpeedTorqCtrl[M1] = &SpeednTorqCtrlM1;
    HallPosSensor_Init (&HallPosSensorM1);
    RotorPosObs_Init(&RotorPosObsM1);

    /******************************************************/
    /*   Speed & torque component initialization          */
    /******************************************************/
    SpdTorqCtrl_Init(pSpeedTorqCtrl[M1],pPIDSpeed[M1], &RotorPosObsM1.Super, &TempSensorParamsM1, NULL);

    /******************************************************/
    /*  Auxiliary speed sensor component initialization   */
    /******************************************************/
    BemfObsPll_Init (&BemfObserverPllM1);

    /********************************************************/
    /*     PID component initialization: current regulation */
    /********************************************************/
    PID_Init(&PIDIqHandleM1);
    PID_Init(&PIDIdHandleM1);
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
    NTCTempSensor_Init(&TempSensorParamsM1);
    pTemperatureSensor[M1] = &TempSensorParamsM1;
    /*******************************************************/
    /*     Flux weakening component initialization         */
    /*******************************************************/
    PID_Init(&PIDFluxWeakeningHandleM1);
    FluxWkng_Init(pFieldWeakening[M1],pPIDSpeed[M1],&PIDFluxWeakeningHandleM1);

    /*******************************************************/
    /*     Feed forward component initialization           */
    /*******************************************************/
    Feedforward_Init(pFeedforward[M1],&(pBusSensorM1->Super),pPIDId[M1],pPIDIq[M1]);

    /*******************************************************/
    /*     MCTuning & FOC variables initialization         */
    /*******************************************************/
    FOC_Clear(M1);
    FOCVars[M1].bDriveInput = INTERNAL;
    oMCInterface[M1] = & MCInterface[M1];
    MCInterface_Init(oMCInterface[M1], &MCStateMachine[M1], pSpeedTorqCtrl[M1], &FOCVars[M1],&(pBusSensorM1->Super), &TempSensorParamsM1);
    MCTuning[M1].pPIDSpeed = pPIDSpeed[M1];
    MCTuning[M1].pPIDIq = pPIDIq[M1];
    MCTuning[M1].pPIDId = pPIDId[M1];
    MCTuning[M1].pPIDFluxWeakening = &PIDFluxWeakeningHandleM1; /* only if M1 has FW */
    MCTuning[M1].pPWMnCurrFdbk = pPWMCurrFdbk[M1];
    MCTuning[M1].pSpeedSensorMain = (SpdPosFdbkHandle_t *) &RotorPosObsM1;
    MCTuning[M1].pSpeedSensorAux = (SpdPosFdbkHandle_t *) &BemfObserverPllM1;
    MCTuning[M1].pSpeedSensorVirtual = MC_NULL;
    MCTuning[M1].pSpeednTorqueCtrl = pSpeedTorqCtrl[M1];
    MCTuning[M1].pStateMachine = &MCStateMachine[M1];
    MCTuning[M1].pTemperatureSensor = (NTCTempSensorHandle_t *) pTemperatureSensor[M1];
    MCTuning[M1].pBusVoltageSensor = &(pBusSensorM1->Super);
    MCTuning[M1].pMotorPower =    (MotorPowerMeasHandle_t*)pMotorPower[M1];
    MCTuning[M1].pFieldWeakening = pFieldWeakening[M1];
    MCTuning[M1].pFeedforward = pFeedforward[M1];

    /*******************************************************/
    /*     Dynamic PI lookup table initialization         */
    /*******************************************************/
    LookupTable_Init(&LookupTableM1IqKp);
    LookupTable_Init(&LookupTableM1IqKi);
    LookupTable_Init(&LookupTableM1IdKp);
    LookupTable_Init(&LookupTableM1IdKi);

    bMCBootCompleted = 1;
}


void MC_RunMotorControlTasks(void)
{
    if (bMCBootCompleted) {
        /* ** Medium Frequency Tasks ** */
        MC_Scheduler();
    }
}


void MC_Scheduler(void)
{
    if (bMCBootCompleted == 1)
    {
        if(hMFTaskCounterM1 > 0u)
        {
            hMFTaskCounterM1--;
        }
        else
        {
            MediumFrequencyTaskM1();
            hMFTaskCounterM1 = MF_TASK_OCCURENCE_TICKS;
        }
        if(hBootCapDelayCounterM1 > 0u)
        {
            hBootCapDelayCounterM1--;
        }
        if(hStopPermanencyCounterM1 > 0u)
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

//    (void) BemfObsPll_CalcAvrgMecSpeedUnit(&BemfObserverPllM1, &wAux);
//     #if HSLOG_PROFILE == HSLOG_BUTTON_LOG 
//        if(!uCAL_GPIO_Read(REVERSE_GPIO_PIN))
//          {
//            LogHS_StopLog(&LogHS_handle);
//         }
//       #endif    
    
    (void) HallPosSensor_CalcAvrgMecSpeedUnit(&HallPosSensorM1, &wAux);
    bool bIsSpeedReliable = RotorPosObs_CalcMecSpeedUnit(&RotorPosObsM1, &wAux);
    MotorPowerQD_CalcElMotorPower(pMotorPower[M1]);
    #if DYNAMIC_CURRENT_CONTROL_PID
    FOC_UpdatePIDGains(M1);
    #endif

    RegConvMng_ExecuteGroupRegularConv(FIRST_REG_CONV_ADC_GROUP_MASK | SECOND_REG_CONV_ADC_GROUP_MASK);

    StateM1 = MCStateMachine_GetState(&MCStateMachine[M1]);
    switch (StateM1)
    {
        case M_IDLE:
            #if DEBUGMODE_MOTOR_CONTROL
            if (bStartMotor)
            {
                MCInterface_ExecTorqueRamp(oMCInterface[M1], hTorqueFinalValueTest);
                MCStateMachine_NextState( &MCStateMachine[M1], M_IDLE_START );
            }
            #endif
            break;

        case M_IDLE_START:
            PWMInsulCurrSensorFdbk_TurnOnLowSides(pPWMCurrFdbk[M1]);
            SetChargeBootCapDelayM1(CHARGE_BOOT_CAP_TICKS);
            MCStateMachine_NextState(&MCStateMachine[M1], M_CHARGE_BOOT_CAP);
            break;

        case M_CHARGE_BOOT_CAP:
            if (ChargeBootCapDelayHasElapsedM1())
            {
                MCStateMachine_NextState(&MCStateMachine[M1],M_OFFSET_CALIB);
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
            MCInterface_ExecBufferedCommands(oMCInterface[M1]); /* Exec the speed ramp after changing of the speed sensor */
        
            #if HSLOG_PROFILE == HSLOG_ZEROSPEED_LOG 
                LogHS_StartOneShot(&LogHS_handle);
            #endif
            
        break;

        case M_RUN:
            #if DEBUGMODE_MOTOR_CONTROL
            if (!bStartMotor)
            {
                MCStateMachine_NextState( &MCStateMachine[M1], M_ANY_STOP );
            }
            MCInterface_ExecTorqueRamp(oMCInterface[M1], hTorqueFinalValueTest);
            #endif

            MCInterface_ExecBufferedCommands(oMCInterface[M1]);
            FOC_CalcCurrRef(M1);

            #if !(BYPASS_POSITION_SENSOR || BYPASS_CURRENT_CONTROL)
            if(!bIsSpeedReliable)
            {
                //MCStateMachine_FaultProcessing(&MCStateMachine[M1], MC_SPEED_FDBK, 0);
            }
            #endif
            break;

        case M_ANY_STOP:
            PWMInsulCurrSensorFdbk_SwitchOffPWM(pPWMCurrFdbk[M1]);
            FOC_Clear(M1);
            MotorPowMeas_Clear((MotorPowerMeasHandle_t*) pMotorPower[M1]);
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
    SpdPosFdbkHandle_t * SpeedHandle;
    SpeedHandle = SpdTorqCtrl_GetSpeedSensor(pSpeedTorqCtrl[bMotor]);

    int16_t hM1SpeedUnit = SpdPosFdbk_GetAvrgMecSpeedUnit(SpeedHandle);

    PID_SetKP(pPIDIq[bMotor], (int16_t)LookupTable_CalcOutput(&LookupTableM1IqKp, abs(hM1SpeedUnit)));
    PID_SetKI(pPIDIq[bMotor], (int16_t)LookupTable_CalcOutput(&LookupTableM1IqKi, abs(hM1SpeedUnit)));
    PID_SetKP(pPIDId[bMotor], (int16_t)LookupTable_CalcOutput(&LookupTableM1IdKp, abs(hM1SpeedUnit)));
    PID_SetKI(pPIDId[bMotor], (int16_t)LookupTable_CalcOutput(&LookupTableM1IdKi, abs(hM1SpeedUnit)));
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

    /* If current references iqref and idref are computed internally    */
    if(FOCVars[bMotor].bDriveInput == INTERNAL)
    {
        FOCVars[bMotor].hTeref = SpdTorqCtrl_CalcTorqueReference(pSpeedTorqCtrl[bMotor]);
        FOCVars[bMotor].Iqdref.q = SpdTorqCtrl_GetIqFromTorqueRef(pSpeedTorqCtrl[bMotor], FOCVars[bMotor].hTeref);
        FOCVars[bMotor].Iqdref.d = SpdTorqCtrl_GetIdFromTorqueRef(pSpeedTorqCtrl[bMotor], FOCVars[bMotor].hTeref);

        if (pFieldWeakening[bMotor])
        {
            IqdTmp.q = FOCVars[bMotor].Iqdref.q;
            IqdTmp.d = FOCVars[bMotor].UserIdref;
            FOCVars[bMotor].Iqdref = FluxWkng_CalcCurrRef(pFieldWeakening[bMotor],IqdTmp);
        }
        if (pFeedforward[bMotor])
        {
            Feedforward_VqdffComputation(pFeedforward[bMotor], FOCVars[bMotor].Iqdref, pSpeedTorqCtrl[bMotor]);
        }
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
    uint16_t hFOCreturn;

    BemfObserverInputs_t BemfObsInputs;
    BemfObsInputs.Valfa_beta = FOCVars[M1].Valphabeta;

    HallPosSensor_CalcElAngle (&HallPosSensorM1);
    RotorPosObs_CalcElAngle(&RotorPosObsM1, 0);

    hFOCreturn = FOC_CurrControllerM1();
    if(hFOCreturn == MC_FOC_DURATION)
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

//  		LogHS_LogMotorVals(&LogHS_handle); //High speed logging, if disable function does a run through
	  	//LogHS_LogMotorValsVarRes(&LogHS_handle); //High speed logging, if disable function does a run through   
 
				
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
inline uint16_t FOC_CurrControllerM1(void)
{
    qd_t Iqd = {0}, Vqd = {0};
    ab_t Iab = {0};
    AlphaBeta_t Ialphabeta = {0}, Valphabeta = {0};

    int16_t hElAngle;
    uint16_t hCodeError = 0;
    SpdPosFdbkHandle_t *speedHandle;

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

        Vqd = Feedforward_VqdConditioning(pFeedforward[M1],Vqd);

        #if (BYPASS_CURRENT_CONTROL)
        Vqd.q = 2000;
        Vqd.d = 0;
        #endif

        Vqd = CircleLimitation(pCircleLimitation[M1], Vqd);
        Valphabeta = MCMath_RevPark(Vqd, hElAngle);

        hCodeError = PWMCurrFdbk_SetPhaseVoltage(pPWMCurrFdbk[M1], Valphabeta);

        FOCVars[M1].Vqd = Vqd;
        FOCVars[M1].Iab = Iab;
        FOCVars[M1].Ialphabeta = Ialphabeta;
        FOCVars[M1].Iqd = Iqd;
        FOCVars[M1].Valphabeta = Valphabeta;
        FOCVars[M1].hElAngle = hElAngle;
        FluxWkng_DataProcess(pFieldWeakening[M1], Vqd);
        Feedforward_DataProcess(pFeedforward[M1]);

        //Check for overcurrent condition (software overcurrent protection)
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

    return(hCodeError);
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
    uint16_t CodeReturn = MC_NO_ERROR;
    uint16_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK};

    CodeReturn |= errMask[bMotor] & NTCTempSensor_CalcAvTemp(pTemperatureSensor[bMotor]); /* check for fault if FW protection is activated. It returns MC_OVER_TEMP or MC_NO_ERROR */
    NTCTempSensor_GetAvTempCelcius(pTemperatureSensor[bMotor]);                           /* store into handle temperature in Celcius */
    CodeReturn |= PWMCurrFdbk_CheckOverCurrent(pPWMCurrFdbk[bMotor]);                     /* check for fault. It return MC_BREAK_IN or MC_NO_FAULTS
                                                                                            (for STM32F30x can return MC_OVER_VOLT in case of HW Overvoltage) */
    if(bMotor == M1)
    {
        CodeReturn |= errMask[bMotor] & ResDivVbusSensor_CalcAvVbus(pBusSensorM1);
    }

    MCStateMachine_FaultProcessing(&MCStateMachine[bMotor], CodeReturn, ~CodeReturn); /* Update the MCStateMachine according error code */
    switch (MCStateMachine_GetState(&MCStateMachine[bMotor])) /* Acts on PWM outputs in case of faults */
    {
    case M_FAULT_NOW:
        PWMCurrFdbk_SwitchOffPWM(pPWMCurrFdbk[bMotor]);
        FOC_Clear(bMotor);
        MotorPowMeas_Clear((MotorPowerMeasHandle_t*)pMotorPower[bMotor]);
        break;
    case M_FAULT_OVER:
        PWMCurrFdbk_SwitchOffPWM(pPWMCurrFdbk[bMotor]);
        break;
    default:
        break;
    }
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
MotorControlInterfaceHandle_t * GetMCI(uint8_t bMotor)
{
    MotorControlInterfaceHandle_t * retVal = MC_NULL;
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
MotorControlTuningHandle_t* GetMCT(uint8_t bMotor)
{
    MotorControlTuningHandle_t* retVal = MC_NULL;
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
__NO_RETURN void startMCMediumFrequencyTask(void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);

    /* Infinite loop */
    for(;;)
    {
        /* delay of 500us */
        osDelay(1);

        MC_RunMotorControlTasks();
    }
}

/* startMCSafetyTask function */
__NO_RETURN void startMCSafetyTask(void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);

    /* Infinite loop */
    for(;;)
    {
        /* delay of 500us */
        osDelay(1);

        MC_SafetyTask();
    }
}
