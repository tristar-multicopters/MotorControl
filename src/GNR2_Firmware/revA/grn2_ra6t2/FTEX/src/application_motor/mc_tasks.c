/**
  ******************************************************************************
  * @file    mc_tasks.c
  * @author  FTEX inc
  * @brief   This file implements motor tasks definition
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "gnr_main.h"
#include "mc_type.h"
#include "mc_math.h"
#include "regular_conversion_manager.h"
#include "mc_interface.h"
#include "mc_tuning.h"
#include "state_machine.h"
#include "pwm_common.h"
#include "mc_config.h"

#include "mc_tasks.h"
#include "parameters_conversion.h"

/* Private define ------------------------------------------------------------*/

#define CHARGE_BOOT_CAP_MS  10
#define CHARGE_BOOT_CAP_MS2 10
#define OFFCALIBRWAIT_MS     0
#define OFFCALIBRWAIT_MS2    0
#define STOPPERMANENCY_MS  400
#define STOPPERMANENCY_MS2 400
#define CHARGE_BOOT_CAP_TICKS  (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS)/ 1000)
#define CHARGE_BOOT_CAP_TICKS2 (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS2)/ 1000)
#define OFFCALIBRWAITTICKS     (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS)/ 1000)
#define OFFCALIBRWAITTICKS2    (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS2)/ 1000)
#define STOPPERMANENCY_TICKS   (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS)/ 1000)
#define STOPPERMANENCY_TICKS2  (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS2)/ 1000)

/* Un-Comment this macro define in order to activate the smooth
   braking action on over voltage */
/* #define  MC.SMOOTH_BRAKING_ACTION_ON_OVERVOLTAGE */

#define VBUS_TEMP_ERR_MASK ~(0 | MC_UNDER_VOLT | 0)

/* Private variables----------------------------------------------------------*/
FOCVars_t FOCVars[NBR_OF_MOTORS];
MCI_Handle_t Mci[NBR_OF_MOTORS];
MCI_Handle_t * oMCInterface[NBR_OF_MOTORS];
MCT_Handle_t MCT[NBR_OF_MOTORS];
STM_Handle_t STM[NBR_OF_MOTORS];
SpeednTorqCtrl_Handle_t *pSTC[NBR_OF_MOTORS];
PID_Handle_t *pPIDSpeed[NBR_OF_MOTORS];
PID_Handle_t *pPIDIq[NBR_OF_MOTORS];
PID_Handle_t *pPIDId[NBR_OF_MOTORS];
RDivider_Handle_t *pBusSensorM1;

NTC_Handle_t *pTemperatureSensor[NBR_OF_MOTORS];
PWMC_Handle_t * pwmcHandle[NBR_OF_MOTORS];
PQD_MotorPowMeas_Handle_t *pMPM[NBR_OF_MOTORS];
CircleLimitation_Handle_t *pCLM[NBR_OF_MOTORS];
FW_Handle_t *pFW[NBR_OF_MOTORS];     /* only if M1 or M2 has FW */
FF_Handle_t *pFF[NBR_OF_MOTORS];     /* only if M1 or M2 has FF */
RampExtMngr_Handle_t *pREMNG[NBR_OF_MOTORS];   /*!< Ramp manager used to modify the Iq ref
                                                    during the start-up switch over.*/

static volatile uint16_t hMFTaskCounterM1 = 0;
static volatile uint16_t hBootCapDelayCounterM1 = 0;
static volatile uint16_t hStopPermanencyCounterM1 = 0;

uint8_t bMCBootCompleted = 0;


int16_t hOpenLoopTheta = 0;
#define OPEN_LOOP_SPEED  20


/* Private functions ---------------------------------------------------------*/
void TSK_MediumFrequencyTaskM1(void);
void FOC_Clear(uint8_t bMotor);
void FOC_InitAdditionalMethods(uint8_t bMotor);
void FOC_CalcCurrRef(uint8_t bMotor);
static uint16_t FOC_CurrControllerM1(void);
void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount);
bool TSK_ChargeBootCapDelayHasElapsedM1(void);
void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount);
bool TSK_StopPermanencyTimeHasElapsedM1(void);
void TSK_SafetyTask_PWMOFF(uint8_t motor);
void UI_Scheduler(void);

/* USER CODE BEGIN Private Functions */

/* USER CODE END Private Functions */
/**
  * @brief  It initializes the whole MC core according to user defined
  *         parameters.
  * @param  pMCIList pointer to the vector of MCInterface objects that will be
  *         created and initialized. The vector must have length equal to the
  *         number of motor drives.
  * @param  pMCTList pointer to the vector of MCTuning objects that will be
  *         created and initialized. The vector must have length equal to the
  *         number of motor drives.
  * @retval None
  */
void MCboot(void)
{
  /**************************************/
  /*    State machine initialization    */
  /**************************************/
//  STM_Init(&STM[M1]);

  bMCBootCompleted = 0;
//  pCLM[M1] = &CircleLimitationM1;
//  pFW[M1] = &FW_M1; /* only if M1 has FW */
//  pFF[M1] = &FF_M1; /* only if M1 has FF */

  /**********************************************************/
  /*    PWM and current sensing component initialization    */
  /**********************************************************/
  pwmcHandle[M1] = &PWM_Handle_M1._Super;
  ICS_Init(&PWM_Handle_M1);

  /**************************************/
  /*    Start timers synchronously      */
  /**************************************/
  //startTimers();

//  /******************************************************/
//  /*   PID component initialization: speed regulation   */
//  /******************************************************/
//  PID_HandleInit(&PIDSpeedHandle_M1);
//  pPIDSpeed[M1] = &PIDSpeedHandle_M1;

//  /******************************************************/
//  /*   Main speed sensor component initialization       */
//  /******************************************************/
//  pSTC[M1] = &SpeednTorqCtrlM1;
//  HALL_Init (&HALL_M1);

//  /******************************************************/
//  /*   Speed & torque component initialization          */
//  /******************************************************/
//  STC_Init(pSTC[M1],pPIDSpeed[M1], &HALL_M1._Super);

//  /******************************************************/
//  /*   Auxiliary speed sensor component initialization  */
//  /******************************************************/
//  STO_PLL_Init (&STO_PLL_M1);

//  /********************************************************/
//  /*   PID component initialization: current regulation   */
//  /********************************************************/
//  PID_HandleInit(&PIDIqHandle_M1);
//  PID_HandleInit(&PIDIdHandle_M1);
//  pPIDIq[M1] = &PIDIqHandle_M1;
//  pPIDId[M1] = &PIDIdHandle_M1;

//  /********************************************************/
//  /*   Bus voltage sensor component initialization        */
//  /********************************************************/
//  pBusSensorM1 = &RealBusVoltageSensorParamsM1;
//  RVBS_Init(pBusSensorM1);

//  /*************************************************/
//  /*   Power measurement component initialization  */
//  /*************************************************/
//  pMPM[M1] = &PQD_MotorPowMeasM1;
//  pMPM[M1]->pVBS = &(pBusSensorM1->_Super);
//  pMPM[M1]->pFOCVars = &FOCVars[M1];

//  /*******************************************************/
//  /*   Temperature measurement component initialization  */
//  /*******************************************************/
//  NTC_Init(&TempSensorParamsM1);
//  pTemperatureSensor[M1] = &TempSensorParamsM1;

//  /*******************************************************/
//  /*   Flux weakening component initialization           */
//  /*******************************************************/
//  PID_HandleInit(&PIDFluxWeakeningHandle_M1);
//  FW_Init(pFW[M1],pPIDSpeed[M1],&PIDFluxWeakeningHandle_M1);

//  /*******************************************************/
//  /*   Feed forward component initialization             */
//  /*******************************************************/
//  FF_Init(pFF[M1],&(pBusSensorM1->_Super),pPIDId[M1],pPIDIq[M1]);

//  pREMNG[M1] = &RampExtMngrHFParamsM1;
//  REMNG_Init(pREMNG[M1]);

//  FOC_Clear(M1);
//  FOCVars[M1].bDriveInput = EXTERNAL;
//  FOCVars[M1].Iqdref = STC_GetDefaultIqdref(pSTC[M1]);
//  FOCVars[M1].UserIdref = STC_GetDefaultIqdref(pSTC[M1]).d;
//  oMCInterface[M1] = & Mci[M1];
//  MCI_Init(oMCInterface[M1], &STM[M1], pSTC[M1], &FOCVars[M1] );
//  MCI_ExecSpeedRamp(oMCInterface[M1],
//  STC_GetMecSpeedRefUnitDefault(pSTC[M1]),0); /*First command to STC*/
//  MCT[M1].pPIDSpeed = pPIDSpeed[M1];
//  MCT[M1].pPIDIq = pPIDIq[M1];
//  MCT[M1].pPIDId = pPIDId[M1];
//  MCT[M1].pPIDFluxWeakening = &PIDFluxWeakeningHandle_M1; /* only if M1 has FW */
//  MCT[M1].pPWMnCurrFdbk = pwmcHandle[M1];
//  MCT[M1].pSpeedSensorMain = (SpeednPosFdbk_Handle_t *) &HALL_M1;
//  MCT[M1].pSpeedSensorAux = (SpeednPosFdbk_Handle_t *) &STO_PLL_M1;
//  MCT[M1].pSpeedSensorVirtual = MC_NULL;
//  MCT[M1].pSpeednTorqueCtrl = pSTC[M1];
//  MCT[M1].pStateMachine = &STM[M1];
//  MCT[M1].pTemperatureSensor = (NTC_Handle_t *) pTemperatureSensor[M1];
//  MCT[M1].pBusVoltageSensor = &(pBusSensorM1->_Super);
//  MCT[M1].pMPM =  (MotorPowMeas_Handle_t*)pMPM[M1];
//  MCT[M1].pFW = pFW[M1];
//  MCT[M1].pFF = pFF[M1];

//	AO_Init( &AngleObserverM1 );

  bMCBootCompleted = 1;
}

/**
 * @brief Runs all the Tasks of the Motor Control cockpit
 *
 * This function is to be called periodically at least at the Medium Frequency task
 * rate (It is typically called on the Systick interrupt). Exact invokation rate is
 * the Speed regulator execution rate set in the Motor Contorl Workbench.
 *
 * The following tasks are executed in this order:
 *
 * - Medium Frequency Tasks of each motors
 * - Safety Task
 * - Power Factor Correction Task (if enabled)
 * - User Interface task.
 */
void MC_RunMotorControlTasks(void)
{
  if ( bMCBootCompleted ) {
    /* ** Medium Frequency Tasks ** */
    MC_Scheduler();
  }
}

/**
 * @brief  Executes the Medium Frequency Task functions for each drive instance.
 *
 * It is to be clocked at the Systick frequency.
 */
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
      TSK_MediumFrequencyTaskM1();
      /* USER CODE BEGIN MC_Scheduler 1 */

      /* USER CODE END MC_Scheduler 1 */
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
  /* USER CODE BEGIN MC_Scheduler 2 */

  /* USER CODE END MC_Scheduler 2 */
}

/**
  * @brief Executes medium frequency periodic Motor Control tasks
  *
  * This function performs some of the control duties on Motor 1 according to the
  * present state of its state machine. In particular, duties requiring a periodic
  * execution at a medium frequency rate (such as the speed controller for instance)
  * are executed here.
  */
void TSK_MediumFrequencyTaskM1(void)
{
  State_t StateM1;
  int16_t wAux = 0;

//  (void) STO_PLL_CalcAvrgMecSpeedUnit( &STO_PLL_M1, &wAux );
//  bool IsSpeedReliable = HALL_CalcAvrgMecSpeedUnit( &HALL_M1, &wAux );
//  PQD_CalcElMotorPower( pMPM[M1] );

  StateM1 = STM_GetState( &STM[M1] );

  switch ( StateM1 )
  {
  case IDLE_START:
//    ICS_TurnOnLowSides( pwmcHandle[M1] );
//    TSK_SetChargeBootCapDelayM1( CHARGE_BOOT_CAP_TICKS );
//    STM_NextState( &STM[M1], CHARGE_BOOT_CAP );
    break;

  case CHARGE_BOOT_CAP:
//    if ( TSK_ChargeBootCapDelayHasElapsedM1() )
//    {
//      PWMC_CurrentReadingCalibr( pwmcHandle[M1], CRC_START );

//      STM_NextState(&STM[M1],OFFSET_CALIB);
//    }
    break;

  case OFFSET_CALIB:
//    if ( PWMC_CurrentReadingCalibr( pwmcHandle[M1], CRC_EXEC ) )
//    {
//      STM_NextState( &STM[M1], CLEAR );
//    }
    break;

  case CLEAR:
//    HALL_Clear( &HALL_M1 );
//    STO_PLL_Clear( &STO_PLL_M1 );
//		AO_Clear( &AngleObserverM1 );
//    if ( STM_NextState( &STM[M1], START ) == true )
//    {
//      FOC_Clear( M1 );

//      ICS_SwitchOnPWM( pwmcHandle[M1] );
//    }
    break;

  case START:
//    STM_NextState( &STM[M1], START_RUN ); /* only for sensored*/
    break;

  case START_RUN:
//	  FOC_InitAdditionalMethods(M1);
//    FOC_CalcCurrRef( M1 );
//    STM_NextState( &STM[M1], RUN );
//    STC_ForceSpeedReferenceToCurrentSpeed( pSTC[M1] ); /* Init the reference speed to current speed */
//    MCI_ExecBufferedCommands( oMCInterface[M1] ); /* Exec the speed ramp after changing of the speed sensor */

    break;

  case RUN:
//    MCI_ExecBufferedCommands( oMCInterface[M1] );
//    FOC_CalcCurrRef( M1 );

//		#if !(POSITION_OPENLOOP || VOLTAGE_OPENLOOP)
//    if( !IsSpeedReliable )
//    {
//      STM_FaultProcessing( &STM[M1], MC_SPEED_FDBK, 0 );
//    }
//		#endif
    break;

  case ANY_STOP:
//    ICS_SwitchOffPWM( pwmcHandle[M1] );
//    FOC_Clear( M1 );
//    MPM_Clear( (MotorPowMeas_Handle_t*) pMPM[M1] );
//    TSK_SetStopPermanencyTimeM1( STOPPERMANENCY_TICKS );

//    STM_NextState( &STM[M1], STOP );
    break;

  case STOP:
//    if ( TSK_StopPermanencyTimeHasElapsedM1() )
//    {
//      STM_NextState( &STM[M1], STOP_IDLE );
//    }
    break;

  case STOP_IDLE:
//    STM_NextState( &STM[M1], IDLE );
    break;

  default:
    break;
  }
}

/**
  * @brief  It re-initializes the current and voltage variables. Moreover
  *         it clears qd currents PI controllers, voltage sensor and SpeednTorque
  *         controller. It must be called before each motor restart.
  *         It does not clear speed sensor.
  * @param  bMotor related motor it can be M1 or M2
  * @retval none
  */
void FOC_Clear(uint8_t bMotor)
{
//  ab_t NULL_ab = {(int16_t)0, (int16_t)0};
//  qd_t NULL_qd = {(int16_t)0, (int16_t)0};
//  alphabeta_t NULL_alphabeta = {(int16_t)0, (int16_t)0};

//  FOCVars[bMotor].Iab = NULL_ab;
//  FOCVars[bMotor].Ialphabeta = NULL_alphabeta;
//  FOCVars[bMotor].Iqd = NULL_qd;
//  FOCVars[bMotor].Iqdref = NULL_qd;
//  FOCVars[bMotor].hTeref = (int16_t)0;
//  FOCVars[bMotor].Vqd = NULL_qd;
//  FOCVars[bMotor].Valphabeta = NULL_alphabeta;
//  FOCVars[bMotor].hElAngle = (int16_t)0;

//  PID_SetIntegralTerm(pPIDIq[bMotor], (int32_t)0);
//  PID_SetIntegralTerm(pPIDId[bMotor], (int32_t)0);

//  STC_Clear(pSTC[bMotor]);

//  PWMC_SwitchOffPWM(pwmcHandle[bMotor]);

//  if (pFW[bMotor])
//  {
//    FW_Clear(pFW[bMotor]);
//  }
//  if (pFF[bMotor])
//  {
//    FF_Clear(pFF[bMotor]);
//  }
}

/**
  * @brief  Use this method to initialize additional methods (if any) in
  *         START_TO_RUN state
  * @param  bMotor related motor it can be M1 or M2
  * @retval none
  */
void FOC_InitAdditionalMethods(uint8_t bMotor)
{
//    if (pFF[bMotor])
//    {
//      FF_InitFOCAdditionalMethods(pFF[bMotor]);
//    }
}

/**
  * @brief  It computes the new values of Iqdref (current references on qd
  *         reference frame) based on the required electrical torque information
  *         provided by oTSC object (internally clocked).
  *         If implemented in the derived class it executes flux weakening and/or
  *         MTPA algorithm(s). It must be called with the periodicity specified
  *         in oTSC parameters
  * @param  bMotor related motor it can be M1 or M2
  * @retval none
  */
void FOC_CalcCurrRef(uint8_t bMotor)
{
//  qd_t IqdTmp;

//  if(FOCVars[bMotor].bDriveInput == INTERNAL)
//  {
//    FOCVars[bMotor].hTeref = STC_CalcTorqueReference(pSTC[bMotor]);
//    FOCVars[bMotor].Iqdref.q = FOCVars[bMotor].hTeref;

//    if (pFW[bMotor])
//    {
//      IqdTmp.q = FOCVars[bMotor].Iqdref.q;
//      IqdTmp.d = FOCVars[bMotor].UserIdref;
//      FOCVars[bMotor].Iqdref = FW_CalcCurrRef(pFW[bMotor],IqdTmp);
//    }
//    if (pFF[bMotor])
//    {
//      FF_VqdffComputation(pFF[bMotor], FOCVars[bMotor].Iqdref, pSTC[bMotor]);
//    }
//  }
}

/**
  * @brief  It set a counter intended to be used for counting the delay required
  *         for drivers boot capacitors charging of motor 1
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount)
{
//   hBootCapDelayCounterM1 = hTickCount;
}

/**
  * @brief  Use this function to know whether the time required to charge boot
  *         capacitors of motor 1 has elapsed
  * @param  none
  * @retval bool true if time has elapsed, false otherwise
  */
bool TSK_ChargeBootCapDelayHasElapsedM1(void)
{
//  bool retVal = false;
//  if (hBootCapDelayCounterM1 == 0)
//  {
//    retVal = true;
//  }
//  return (retVal);
}

/**
  * @brief  It set a counter intended to be used for counting the permanency
  *         time in STOP state of motor 1
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount)
{
//  hStopPermanencyCounterM1 = hTickCount;
}

/**
  * @brief  Use this function to know whether the permanency time in STOP state
  *         of motor 1 has elapsed
  * @param  none
  * @retval bool true if time is elapsed, false otherwise
  */
bool TSK_StopPermanencyTimeHasElapsedM1(void)
{
//  bool retVal = false;
//  if (hStopPermanencyCounterM1 == 0)
//  {
//    retVal = true;
//  }
//  return (retVal);
}

/**
  * @brief  Executes the Motor Control duties that require a high frequency rate and a precise timing
  *
  *  This is mainly the FOC current control loop. It is executed depending on the state of the Motor Control
  * subsystem (see the state machine(s)).
  *
  * @retval Number of the  motor instance which FOC loop was executed.
  */
uint8_t TSK_HighFrequencyTask(void)
{
  uint8_t bMotorNbr = 0;
  uint16_t hFOCreturn;

//  Observer_Inputs_t STO_aux_Inputs; /*  only if sensorless aux*/
//  STO_aux_Inputs.Valfa_beta = FOCVars[M1].Valphabeta;  /* only if sensorless*/

  //HALL_CalcElAngle (&HALL_M1);
	//AO_CalcElAngle(&AngleObserverM1, 0);
	
  hFOCreturn = FOC_CurrControllerM1();
//  if(hFOCreturn == MC_FOC_DURATION)
//  {
//    STM_FaultProcessing(&STM[M1], MC_FOC_DURATION, 0);
//  }
//  else
//  {
//    STO_aux_Inputs.Ialfa_beta = FOCVars[M1].Ialphabeta; /*  only if sensorless*/
//    STO_aux_Inputs.Vbus = VBS_GetAvBusVoltage_d(&(pBusSensorM1->_Super)); /*  only for sensorless*/
//    STO_PLL_CalcElAngle (&STO_PLL_M1, &STO_aux_Inputs);
//		STO_PLL_CalcAvrgElSpeedDpp (&STO_PLL_M1);
//  }

  return bMotorNbr;
}

/**
  * @brief It executes the core of FOC drive that is the controllers for Iqd
  *        currents regulation. Reference frame transformations are carried out
  *        accordingly to the active speed sensor. It must be called periodically
  *        when new motor currents have been converted
  * @param this related object of class CFOC.
  * @retval int16_t It returns MC_NO_FAULTS if the FOC has been ended before
  *         next PWM Update event, MC_FOC_DURATION otherwise
  */
inline uint16_t FOC_CurrControllerM1(void)
{
  qd_t Iqd, Vqd;
  ab_t Iab;
  alphabeta_t Ialphabeta, Valphabeta;

  int16_t hElAngle;
  uint16_t hCodeError = 0;
//  SpeednPosFdbk_Handle_t *speedHandle;

//  speedHandle = STC_GetSpeedSensor(pSTC[M1]);
//  hElAngle = SPD_GetElAngle(speedHandle);
//	//hElAngle = AO_GetElAngle(&AngleObserverM1);
//	#if (POSITION_OPENLOOP)
//	hOpenLoopTheta += OPEN_LOOP_SPEED;
//	hElAngle = hOpenLoopTheta;
//	#endif
	
  PWMC_GetPhaseCurrents(pwmcHandle[M1], &Iab);
//  RCM_ReadOngoingConv();
//  RCM_ExecNextConv();
//  Ialphabeta = MCM_Clarke(Iab);
//  Iqd = MCM_Park(Ialphabeta, hElAngle);
//  Vqd.q = PI_Controller(pPIDIq[M1],
//            (int32_t)(FOCVars[M1].Iqdref.q) - Iqd.q);

//  Vqd.d = PI_Controller(pPIDId[M1],
//            (int32_t)(FOCVars[M1].Iqdref.d) - Iqd.d);
//  Vqd = FF_VqdConditioning(pFF[M1],Vqd);
//	
//	#if (VOLTAGE_OPENLOOP)
//	Vqd.q = 1500;
//	Vqd.d = 0;
//	#endif

//  Vqd = Circle_Limitation(pCLM[M1], Vqd);
//  hElAngle += SPD_GetInstElSpeedDpp(speedHandle)*REV_PARK_ANGLE_COMPENSATION_FACTOR;
//  Valphabeta = MCM_Rev_Park(Vqd, hElAngle);
//  hCodeError = PWMC_SetPhaseVoltage(pwmcHandle[M1], Valphabeta);
//  FOCVars[M1].Vqd = Vqd;
//  FOCVars[M1].Iab = Iab;
//  FOCVars[M1].Ialphabeta = Ialphabeta;
//  FOCVars[M1].Iqd = Iqd;
//  FOCVars[M1].Valphabeta = Valphabeta;
//  FOCVars[M1].hElAngle = hElAngle;
//  FW_DataProcess(pFW[M1], Vqd);
//  FF_DataProcess(pFF[M1]);
  return(hCodeError);
}

/**
  * @brief  Executes safety checks (e.g. bus voltage and temperature) for all drive instances.
  *
  * Faults flags are updated here.
  */
void TSK_SafetyTask(void)
{
//  if (bMCBootCompleted == 1)
//  {
//    TSK_SafetyTask_PWMOFF(M1);
//    /* User conversion execution */
//    RCM_ExecUserConv ();
//  }
}

/**
  * @brief  Safety task implementation if  MC.ON_OVER_VOLTAGE == TURN_OFF_PWM
  * @param  bMotor Motor reference number defined
  *         \link Motors_reference_number here \endlink
  * @retval None
  */
void TSK_SafetyTask_PWMOFF(uint8_t bMotor)
{
//  uint16_t CodeReturn = MC_NO_ERROR;
//  uint16_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK};

//  CodeReturn |= errMask[bMotor] & NTC_CalcAvTemp(pTemperatureSensor[bMotor]); /* check for fault if FW protection is activated. It returns MC_OVER_TEMP or MC_NO_ERROR */
//  CodeReturn |= PWMC_CheckOverCurrent(pwmcHandle[bMotor]);                    /* check for fault. It return MC_BREAK_IN or MC_NO_FAULTS
//                                                                                 (for STM32F30x can return MC_OVER_VOLT in case of HW Overvoltage) */
//  if(bMotor == M1)
//  {
//    CodeReturn |=  errMask[bMotor] &RVBS_CalcAvVbus(pBusSensorM1);
//  }

//  STM_FaultProcessing(&STM[bMotor], CodeReturn, ~CodeReturn); /* Update the STM according error code */
//  switch (STM_GetState(&STM[bMotor])) /* Acts on PWM outputs in case of faults */
//  {
//  case FAULT_NOW:
//    PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
//    FOC_Clear(bMotor);
//    MPM_Clear((MotorPowMeas_Handle_t*)pMPM[bMotor]);
//    break;
//  case FAULT_OVER:
//    PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
//    break;
//  default:
//    break;
//  }
}

/**
  * @brief  This function returns the reference of the MCInterface relative to
  *         the selected drive.
  * @param  bMotor Motor reference number defined
  *         \link Motors_reference_number here \endlink
  * @retval MCI_Handle_t * Reference to MCInterface relative to the selected drive.
  *         Note: it can be MC_NULL if MCInterface of selected drive is not
  *         allocated.
  */
MCI_Handle_t * GetMCI(uint8_t bMotor)
{
  MCI_Handle_t * retVal = MC_NULL;
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
  * @retval MCT_Handle_t motor control tuning handler for the selected drive.
  *         Note: it can be MC_NULL if MCInterface of selected drive is not
  *         allocated.
  */
MCT_Handle_t* GetMCT(uint8_t bMotor)
{
  MCT_Handle_t* retVal = MC_NULL;
  if (bMotor < NBR_OF_MOTORS)
  {
    retVal = &MCT[bMotor];
  }
  return retVal;
}

/**
  * @brief  Puts the Motor Control subsystem in in safety conditions on a Hard Fault
  *
  *  This function is to be executed when a general hardware failure has been detected
  * by the microcontroller and is used to put the system in safety condition.
  */
void TSK_HardwareFaultTask(void)
{
//  ICS_SwitchOffPWM(pwmcHandle[M1]);
//  STM_FaultProcessing(&STM[M1], MC_SW_ERROR, 0);
}

/* startMCMediumFrequencyTask function */
__NO_RETURN void startMCMediumFrequencyTask(void * pvParameter)
{
	MCboot();
	
	ICS_SwitchOnPWM( &PWM_Handle_M1._Super );
	
  /* Infinite loop */
  for(;;)
  {
		R_GPT_THREE_PHASE_DutyCycleSet(g_three_phase0.p_ctrl, &PWM_Handle_M1.sDutyCycle);
		
    /* delay of 500us */
    osDelay(1);
  }
}

/* startMCSafetyTask function */
__NO_RETURN void startMCSafetyTask(void * pvParameter)
{
	osDelay(100);
	
  /* Infinite loop */
  for(;;)
  {
    /* delay of 500us */
    osDelay(1);
    //TSK_SafetyTask();
  }
}


