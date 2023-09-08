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



/* Private macros ------------------------------------------------------------*/
/**
  * @brief This macro converts the exported enum from the state machine to the corresponding bit field.
  */
#define BC(state) (1u<<((uint16_t)((uint8_t)(state))))

/* Functions -----------------------------------------------*/

/*
* see function definition
*/
void MCInterface_Init(MotorControlInterfaceHandle_t * pHandle, MotorStateMachineHandle_t * pSTM, SpdTorqCtrlHandle_t * pSpeedTorqCtrl, pFOCVars_t pFOCVars, BusVoltageSensorHandle_t * pBusVoltageSensor, MCConfigHandle_t *pMCConfig)
{
  ASSERT(pHandle != NULL);
  pHandle->pSTM = pSTM;
  pHandle->pSpeedTorqCtrl = pSpeedTorqCtrl;
  pHandle->pFOCVars = pFOCVars;
  pHandle->pBusVoltageSensor = pBusVoltageSensor;
  pHandle->pMCConfig = pMCConfig;
    
  /* Buffer related initialization */
  pHandle->LastCommand = MCI_NOCOMMANDSYET;
  pHandle->hFinalSpeed = 0;
  pHandle->hFinalTorque = 0;
  pHandle->CommandState = MCI_BUFFER_EMPTY;
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

  if (MCStateMachine_GetState(pHandle->pSTM) == M_IDLE)
  {
      bRetVal = MCStateMachine_NextState( pHandle->pSTM, M_IDLE_START );
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
      bRetVal = MCStateMachine_NextState( pHandle->pSTM, M_ANY_STOP );
  }
  return bRetVal;
}

/*
* see function definition
*/
bool MCInterface_FaultAcknowledged(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  return MCStateMachine_FaultAcknowledged(pHandle->pSTM);
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
uint16_t MCInterface_GetOccurredFaults(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  return (uint16_t)(MCStateMachine_GetFaultState(pHandle->pSTM));
}

/*
* see function definition
*/
uint16_t MCInterface_GetOccurredWarning(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  return (uint16_t)(MCStateMachine_GetWarningState(pHandle->pSTM));
}

/*
* see function definition
*/
uint16_t MCInterface_GetCurrentFaults(MotorControlInterfaceHandle_t * pHandle)
{
  ASSERT(pHandle != NULL);
  return (uint16_t)(MCStateMachine_GetFaultState(pHandle->pSTM) >> 16);
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
    
    ConversionFactor = pHandle->pBusVoltageSensor->hConversionFactor;
    VoltageConverted = VbusSensor_GetAvBusVoltageDigital(pHandle->pBusVoltageSensor);
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
    
    return pHandle->pMCConfig->wNominalCurr;
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
