/**
  * @file    mc_interface.c
  * @author  Sami Bouzid, FTEX inc
  * @brief   This file contains all definitions and functions prototypes for the
  *          MC Interface component. It allow controlling a motor and get measurements using a simple api.
*/

/* Includes ------------------------------------------------------------------*/
#include "mc_math.h"
#include "speed_torq_ctrl.h"
#include "mc_interface.h"
#include "pwm_common.h"
#include "mc_config.h"


/* Private macros ------------------------------------------------------------*/
/**
  * @brief This macro converts the exported enum from the state machine to the corresponding bit field.
  */
#define BC(state) (1u<<((uint16_t)((uint8_t)(state))))

/* Functions -----------------------------------------------*/

/*
* see function definition
*/
void MCInterface_Init(MotorControlInterfaceHandle_t * pHandle, MotorStateMachineHandle_t * pSTM, SpdTorqCtrlHandle_t * pSpeedTorqCtrl, pFOCVars_t pFOCVars,
                        ResDivVbusSensorHandle_t * pResDivVbusSensor, MCConfigHandle_t *pMCConfig)
{
    ASSERT(pHandle != NULL);
    pHandle->pSTM = pSTM;
    pHandle->pSpeedTorqCtrl = pSpeedTorqCtrl;
    pHandle->pFOCVars = pFOCVars;
    pHandle->pResDivVbusSensor = pResDivVbusSensor;
    pHandle->pMCConfig = pMCConfig;

    /* Buffer related initialization */
    pHandle->LastCommand = MCI_NOCOMMANDSYET;
    pHandle->hFinalSpeed = 0;
    pHandle->hFinalTorque = 0;
    pHandle->CommandState = MCI_BUFFER_EMPTY;
    
    /*Initialize driver */
    Driver_Disable(&pHandle->bDriverEn);
    
}

/*
* see function definition
*/
void MCInterface_PowerInit(MotorControlInterfaceHandle_t * pHandle, MC_Setup_t MCSetup)
{
    ASSERT(pHandle != NULL);
    SpdTorqCtrl_PowerInit(pHandle->pSpeedTorqCtrl, MCSetup, MotorParameters);
    ResDivVbusSensor_UVInit(pHandle->pResDivVbusSensor, MCSetup);
}

/*
* see function definition
*/
void MCInterface_ExecSpeedRamp(MotorControlInterfaceHandle_t * pHandle,  int16_t hFinalSpeed)
{
  ASSERT(pHandle != NULL);
  pHandle->LastCommand = MCI_EXECSPEEDRAMP;
  pHandle->hFinalSpeed = hFinalSpeed;
  pHandle->CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
  pHandle->LastModalitySetByUser = STC_SPEED_MODE;
}

/*
* see function definition
*/
void MCInterface_ExecTorqueRamp(MotorControlInterfaceHandle_t * pHandle,  int16_t hFinalTorque)
{
  ASSERT(pHandle != NULL);
  pHandle->LastCommand = MCI_EXECTORQUERAMP;
  pHandle->hFinalTorque = hFinalTorque;
  pHandle->CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
  pHandle->LastModalitySetByUser = STC_TORQUE_MODE;
}

/*
* see function definition
*/
void MCInterface_SetCurrentReferences(MotorControlInterfaceHandle_t * pHandle, qd_t Iqdref)
{
  ASSERT(pHandle != NULL);
  pHandle->LastCommand = MCI_SETCURRENTREFERENCES;
  pHandle->Iqdref.q = Iqdref.q;
  pHandle->Iqdref.d = Iqdref.d;
  pHandle->CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
  pHandle->LastModalitySetByUser = STC_TORQUE_MODE;
}

/*
* see function definition
*/
bool MCInterface_StartMotor(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  bool bRetVal = false;

  if (MCStateMachine_GetState(pHandle->pSTM) == M_IDLE && !MCStateMachine_GetOccurredErrorState(pHandle->pSTM))
  {
      bRetVal = MCStateMachine_NextState(pHandle->pSTM, M_IDLE_START);
  }
  return bRetVal;
}

/*
* see function definition
*/
bool MCInterface_StopMotor(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  bool bRetVal = false;

  if (MCStateMachine_GetState(pHandle->pSTM) == M_RUN)
  {
      bRetVal = MCStateMachine_NextState(pHandle->pSTM, M_ANY_STOP);
  }
  return bRetVal;
}

/*
* see function definition
*/
bool MCInterface_CriticalFaultAcknowledged(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  return MCStateMachine_CriticalFaultAcknowledged(pHandle->pSTM);
}

/*
* see function definition
*/
void MCInterface_ExecBufferedCommands(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  if (pHandle != MC_NULL)
  {
    if (pHandle->CommandState == MCI_COMMAND_NOT_ALREADY_EXECUTED)
    {
      bool commandHasBeenExecuted = false;
      switch (pHandle->LastCommand)
      {
        case MCI_EXECSPEEDRAMP:
        {
          pHandle->pFOCVars->bDriveInput = INTERNAL;
          SpdTorqCtrl_SetControlMode(pHandle->pSpeedTorqCtrl, STC_SPEED_MODE);
          commandHasBeenExecuted = SpdTorqCtrl_ExecRamp(pHandle->pSpeedTorqCtrl, pHandle->hFinalSpeed);
        }
        break;
        case MCI_EXECTORQUERAMP:
        {
          pHandle->pFOCVars->bDriveInput = INTERNAL;
          SpdTorqCtrl_SetControlMode(pHandle->pSpeedTorqCtrl, STC_TORQUE_MODE);
          commandHasBeenExecuted = SpdTorqCtrl_ExecRamp(pHandle->pSpeedTorqCtrl, pHandle->hFinalTorque);
        }
        break;
        case MCI_SETCURRENTREFERENCES:
        {
          pHandle->pFOCVars->bDriveInput = EXTERNAL;
          pHandle->pFOCVars->Iqdref = pHandle->Iqdref;
          commandHasBeenExecuted = true;
        }
        break;
        default:
          break;
      }

      if (commandHasBeenExecuted)
      {
        pHandle->CommandState = MCI_COMMAND_EXECUTED_SUCCESFULLY;
      }
      else
      {
        pHandle->CommandState = MCI_COMMAND_EXECUTED_UNSUCCESFULLY;
      }
    }
  }
}

/*
* see function definition
*/
MCInterfaceCommandState_t  MCI_IsCommandAcknowledged(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  MCInterfaceCommandState_t retVal = pHandle->CommandState;

  if ((retVal == MCI_COMMAND_EXECUTED_SUCCESFULLY) |
       (retVal == MCI_COMMAND_EXECUTED_UNSUCCESFULLY))
  {
    pHandle->CommandState = MCI_BUFFER_EMPTY;
  }
  return retVal;
}

/*
* see function definition
*/
MotorState_t  MCInterface_GetSTMState(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  return MCStateMachine_GetState(pHandle->pSTM);
}

/*
* see function definition
*/
uint32_t MCInterface_GetOccurredCriticalFaults(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  return (uint32_t)(MCStateMachine_GetCriticalFaultState(pHandle->pSTM));
}

/*
* see function definition
*/
uint32_t MCInterface_GetCurrentErrors(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  return MCStateMachine_GetCurrentErrorState(pHandle->pSTM);
}

/*
* see function definition
*/
uint32_t MCInterface_GetOccurredErrors(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  return MCStateMachine_GetOccurredErrorState(pHandle->pSTM);
}

/*
* see function definition
*/
uint32_t MCInterface_GetOccurredWarning(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  return MCStateMachine_GetWarningState(pHandle->pSTM);
}

/*
* see function definition
*/
uint32_t MCInterface_GetCurrentCriticalFaults(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  return (uint32_t)(MCStateMachine_GetCriticalFaultState(pHandle->pSTM) >> 32);
}

/*
* see function definition
*/
STCModality_t MCInterface_GetControlMode(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  return pHandle->LastModalitySetByUser;
}

/*
* see function definition
*/
int16_t MCInterface_GetImposedMotorDirection(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  int16_t retVal = 1;

  switch (pHandle->LastCommand)
  {
    case MCI_EXECSPEEDRAMP:
      if (pHandle->hFinalSpeed < 0)
      {
        retVal = -1;
      }
      break;
    case MCI_EXECTORQUERAMP:
      if (pHandle->hFinalTorque < 0)
      {
        retVal = -1;
      }
      break;
    case MCI_SETCURRENTREFERENCES:
      if (pHandle->Iqdref.q < 0)
      {
        retVal = -1;
      }
      break;
    default:
      break;
  }
  return retVal;
}

/*
* see function definition
*/
int16_t MCInterface_GetLastRampFinalSpeed(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  int16_t hRetVal = 0;

  /* Examine the last buffered commands */
  if (pHandle->LastCommand == MCI_EXECSPEEDRAMP)
  {
    hRetVal = pHandle->hFinalSpeed;
  }
  return hRetVal;
}

/*
* see function definition
*/
bool MCInterface_IsRampCompleted(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  bool retVal = false;

  if ((MCStateMachine_GetState(pHandle->pSTM)) == M_RUN)
  {
    retVal = SpdTorqCtrl_IsRampCompleted(pHandle->pSpeedTorqCtrl);
  }

  return retVal;
}

/*
* see function definition
*/
void MCInterface_StopRamp(MotorControlInterfaceHandle_t * pHandle)
{
   ASSERT(pHandle != NULL);
   SpdTorqCtrl_StopRamp(pHandle->pSpeedTorqCtrl);
}

/*
* see function definition
*/
bool MCInterface_GetSpdSensorReliability(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  SpdPosFdbkHandle_t * SpeedSensor = SpdTorqCtrl_GetSpeedSensor(pHandle->pSpeedTorqCtrl);

  return (SpdPosFdbk_GetReliability(SpeedSensor));
}

/*
* see function definition
*/
int16_t MCInterface_GetAvrgMecSpeedUnit(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  SpdPosFdbkHandle_t * SpeedSensor = SpdTorqCtrl_GetSpeedSensor(pHandle->pSpeedTorqCtrl);
    
  return (SpdPosFdbk_GetAvrgMecSpeedUnit(SpeedSensor));
}

/*
* see function definition
*/
int16_t MCInterface_GetMecSpeedRefUnit(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  return (SpdTorqCtrl_GetMecSpeedRefUnit(pHandle->pSpeedTorqCtrl));
}

/*
* see function definition
*/
ab_t MCInterface_GetIab(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  ASSERT(pHandle->pFOCVars != NULL);
  return (pHandle->pFOCVars->Iab);
}

/*
* see function definition
*/
AlphaBeta_t MCInterface_GetIalphabeta(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  ASSERT(pHandle->pFOCVars != NULL);
  return (pHandle->pFOCVars->Ialphabeta);
}

/*
* see function definition
*/
qd_t MCInterface_GetIqd(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  ASSERT(pHandle->pFOCVars != NULL);
  return (pHandle->pFOCVars->Iqd);
}

/*
* see function definition
*/
qd_t MCInterface_GetIqdHF(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  ASSERT(pHandle->pFOCVars != NULL);
  return (pHandle->pFOCVars->IqdHF);
}

/*
* see function definition
*/
qd_t MCInterface_GetIqdref(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  ASSERT(pHandle->pFOCVars != NULL);
  return (pHandle->pFOCVars->Iqdref);
}

/*
* see function definition
*/
qd_t MCInterface_GetVqd(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  ASSERT(pHandle->pFOCVars != NULL);
  return (pHandle->pFOCVars->Vqd);
}

/*
* see function definition
*/
AlphaBeta_t MCInterface_GetValphabeta(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  ASSERT(pHandle->pFOCVars != NULL);
  return (pHandle->pFOCVars->Valphabeta);
}

/*
* see function definition
*/
int16_t MCInterface_GetElAngledpp(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  ASSERT(pHandle->pFOCVars != NULL);
  return (pHandle->pFOCVars->hElAngle);
}

/*
* see function definition
*/
int16_t MCInterface_GetTeref(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  ASSERT(pHandle->pFOCVars != NULL);
  return (pHandle->pFOCVars->hTeref);
}

/*
* see function definition
*/
int16_t MCInterface_GetPhaseCurrentAmplitude(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  ASSERT(pHandle->pFOCVars != NULL);
    
  AlphaBeta_t Local_Curr;
  int32_t wAux1, wAux2;

  Local_Curr = pHandle->pFOCVars->Ialphabeta;
  wAux1 = (int32_t)(Local_Curr.alpha) * Local_Curr.alpha;
  wAux2 = (int32_t)(Local_Curr.beta) * Local_Curr.beta;

  wAux1 += wAux2;
  wAux1 = MCMath_Sqrt(wAux1);

  if (wAux1 > INT16_MAX)
  {
    wAux1 = (int32_t) INT16_MAX;
  }

  return ((int16_t)wAux1);
}

/*
* see function definition
*/
int16_t MCInterface_GetPhaseVoltageAmplitude(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  ASSERT(pHandle->pFOCVars != NULL);
  
  AlphaBeta_t Local_Voltage;
  int32_t wAux1, wAux2;

  Local_Voltage = pHandle->pFOCVars->Valphabeta;
  wAux1 = (int32_t)(Local_Voltage.alpha) * Local_Voltage.alpha;
  wAux2 = (int32_t)(Local_Voltage.beta) * Local_Voltage.beta;

  wAux1 += wAux2;
  wAux1 = MCMath_Sqrt(wAux1);

  if (wAux1 > INT16_MAX)
  {
    wAux1 = (int32_t) INT16_MAX;
  }

  return ((int16_t) wAux1);
}

/**
  *  Converts the digital voltage of the bus to a value in volts * 100
  *  Function has been added to enable the battery monitoring module in 
  *  vehicle control to have acces to the bus voltage.
  */
uint16_t MCInterface_GetBusVoltageInVoltx100(MotorControlInterfaceHandle_t * pHandle)
{ 
    ASSERT(pHandle != NULL);
    
    uint32_t VoltageConverted = 0;
    uint16_t ConversionFactor = 0;
    
    ConversionFactor = pHandle->pResDivVbusSensor->Super.hConversionFactor;
    VoltageConverted = VbusSensor_GetAvBusVoltageDigital(&(pHandle->pResDivVbusSensor->Super));
    VoltageConverted = VoltageConverted * ConversionFactor * 100u;
    VoltageConverted = VoltageConverted/65536u;
         
   return (uint16_t) VoltageConverted; // Return voltage * 100 so 49.63 V will be 4963.0
                                       // This is done to keep precision 
}

/**
  *  Getting the controller NTC temperature value
  */
int16_t MCInterface_GetControllerTemp(MotorControlInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    ASSERT(pHandle->pSpeedTorqCtrl->pHeatsinkTempSensor != NULL);
    
    return pHandle->pSpeedTorqCtrl->pHeatsinkTempSensor->hAvTempCelcius;
}

/**
  *  Getting the motor NTC temperature value
  */
int16_t MCInterface_GetMotorTemp(MotorControlInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    ASSERT(pHandle->pSpeedTorqCtrl->pMotorTempSensor != NULL);
    
    return pHandle->pSpeedTorqCtrl->pMotorTempSensor->hAvTempCelcius;
}

/**
  *  Get the Max safe current from motor control.
  */
int16_t MCInterface_GetMaxCurrent(MotorControlInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    ASSERT(pHandle->pMCConfig != NULL);
    
    return pHandle->pMCConfig->hNominalCurr;
}

/**
  *  Get the maximum ongoing current,
  */
int16_t MCInterface_GetOngoingMaxCurrent(MotorControlInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    ASSERT(pHandle->pMCConfig != NULL);
    
    return pHandle->pMCConfig->wUsrMaxCurr;
}

/**
  *  Set the maximum ongoing current, 
  */
void MCInterface_SetOngoingMaxCurrent(MotorControlInterfaceHandle_t * pHandle, int16_t aCurrent)
{
    ASSERT(pHandle != NULL);
    ASSERT(pHandle->pMCConfig != NULL);
    
    pHandle->pMCConfig->wUsrMaxCurr = aCurrent;
}

/**
  *  Set the wheel RPM
  */
void  MCInterface_SetWheelRPM(MotorControlInterfaceHandle_t * pHandle, uint16_t aWheelRPM)
{
  ASSERT(pHandle != NULL);  
  pHandle->pSpeedTorqCtrl->pSPD->wheelRPM = aWheelRPM;
    
}   

/**
  *  Get the current torq reference 
  */
int16_t MCInterface_GetTorqueReference(MotorControlInterfaceHandle_t * pHandle, uint8_t Motor)
{
    if (Motor == 0)
    {
        return pHandle->pFOCVars[M1].hTeref;
    }
    else
    {
        return pHandle->pFOCVars[M2].hTeref;
    }          
}

/**
  *  Get the max application power
  */
uint16_t MCInterface_GetMaxPositivePower(MotorControlInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->pSpeedTorqCtrl->hMaxPositivePower;
}

/**
  *  enable the regen feature,
  */
void MCInterface_Enableregen(SpdTorqCtrlHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    ASSERT(pHandle->pSPD != NULL);
    
    pHandle->pSPD->bActiveRegen = true;
}

/**
  *  disable the regen feature,
  */
void MCInterface_Disableregen(SpdTorqCtrlHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    ASSERT(pHandle->pSPD != NULL);
    
    pHandle->pSPD->bActiveRegen = false;
}

/**
  *  Get the max negative battery current in amps
  */
int16_t MCInterface_GetMaxNegativeCurrent(SpdTorqCtrlHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->pSPD->hIdcRegen;
}

/**
  *  Set the max negative battery current in amps
  */
void MCInterface_SetMaxNegativeCurrent(SpdTorqCtrlHandle_t * pHandle, int16_t Idc_Negative)
{
    ASSERT(pHandle != NULL);
    pHandle->pSPD->hIdcRegen = Idc_Negative;
}

/**
  *  Get the rate of increasing the negative Torque in mili Nm per milisecond or Nm/sec
  */
int16_t MCInterface_GetMaxNegativeTorqueRate(SpdTorqCtrlHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->pSPD->hDeltaT;
}

/**
  *  Get the regenerative torque value in mili Nm
  */
int16_t MCInterface_GetRegenTorque(SpdTorqCtrlHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->pSPD->hTorqueRegen;
}
/**
  *  Get the motor gear ratio
  */
float MCInterface_GetMotorGearRatio(MotorControlInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->pSpeedTorqCtrl->fGearRatio;
}

/**
  *  Get the motor type
  */
MotorType_t MCInterface_GetMotorType(MotorControlInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->pSpeedTorqCtrl->motorType;
}

/**
  *  Get the nominal torque
  */
uint16_t MCInterface_GetNominalTorque(MotorControlInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->pSpeedTorqCtrl->hMaxPositiveTorque;
}

/**
  *  Get the starting torque
  */
uint16_t MCInterface_GetStartingTorque(MotorControlInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->pSpeedTorqCtrl->hStartingTorque;
}

/**
  *  Get the RS torque
  */
float MCInterface_GetRS(MotorControlInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->pMCConfig->fRS;
}

/**
  *  Get the number of magnets on the wheel speed sensor
  */
uint8_t MCInterface_GetWheelSpdSensorNbrPerRotation(MotorControlInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->pMCConfig->bWheelSpdSensorNbrPerRotation;
}

/**
  *  Get the percentage of time that the wheel speed sensor spends on each magnet
  */
float MCInterface_GetMotorWSSTimeOnOneMagnetPercent(MotorControlInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->pMCConfig->fMotorWSSTimeOnOneMagnetPercent;
}

/**
  *  Get whether the motor sensor type is mixed or not
  */
bool MCInterface_GetMotorTempSensorMixed(MotorControlInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->pSpeedTorqCtrl->pMotorTempSensor->bSensorMixed;
}

#if AUTOTUNE_ENABLE

/**
  *  This command enters motor tuning mode.
  *  Commands to the motor tuner are only processed in this mode.
  */
bool MCInterface_StartMotorTuning(MotorControlInterfaceHandle_t * pHandle)
{
  bool bRetVal = false;

  if (MCStateMachine_GetState(pHandle->pSTM) == M_IDLE)
  {
      bRetVal = MCStateMachine_NextState( pHandle->pSTM, M_AUTOTUNE_ENTER_IDENTIFICATION );
  }
  return bRetVal;
}

/**
  *  This command exits motor tuning mode.
  *  Commands to the motor tuner are only processed in this mode.
  */
bool MCInterface_StopMotorTuning(MotorControlInterfaceHandle_t * pHandle)
{
  bool bRetVal = false;

  if (MCStateMachine_GetState(pHandle->pSTM) == M_AUTOTUNE_IDENTIFICATION)
  {
      bRetVal = MCStateMachine_NextState( pHandle->pSTM, M_AUTOTUNE_ANY_STOP_IDENTIFICATION );
  }
  return bRetVal;
}
#endif

/**
  *  This command enables the flux weakening logic.
  */
void MCInterface_EnableFluxWeakening(SpdTorqCtrlHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    pHandle->bFluxWeakeningEn = true;
}

/**
  *  This command disables the flux weakening logic.
  */

void MCInterface_DisableFluxWeakening(SpdTorqCtrlHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    pHandle->bFluxWeakeningEn = false;
}