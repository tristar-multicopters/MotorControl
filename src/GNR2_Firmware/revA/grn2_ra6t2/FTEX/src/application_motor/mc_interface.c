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

void MCInterface_Init(MotorControlInterfaceHandle_t * pHandle, MotorStateMachineHandle_t * pSTM, SpeednTorqCtrlHandle_t * pSpeedTorqCtrl, pFOCVars_t pFOCVars)
{
  pHandle->pSTM = pSTM;
  pHandle->pSpeedTorqCtrl = pSpeedTorqCtrl;
  pHandle->pFOCVars = pFOCVars;

  /* Buffer related initialization */
  pHandle->LastCommand = MCI_NOCOMMANDSYET;
  pHandle->hFinalSpeed = 0;
  pHandle->hFinalTorque = 0;
  pHandle->CommandState = MCI_BUFFER_EMPTY;
}


void MCInterface_ExecSpeedRamp(MotorControlInterfaceHandle_t * pHandle,  int16_t hFinalSpeed)
{
  pHandle->LastCommand = MCI_EXECSPEEDRAMP;
  pHandle->hFinalSpeed = hFinalSpeed;
  pHandle->CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
  pHandle->LastModalitySetByUser = STC_SPEED_MODE;
}


void MCInterface_ExecTorqueRamp(MotorControlInterfaceHandle_t * pHandle,  int16_t hFinalTorque)
{
  pHandle->LastCommand = MCI_EXECTORQUERAMP;
  pHandle->hFinalTorque = hFinalTorque;
  pHandle->CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
  pHandle->LastModalitySetByUser = STC_TORQUE_MODE;
}


void MCInterface_SetCurrentReferences(MotorControlInterfaceHandle_t * pHandle, qd_t Iqdref)
{
  pHandle->LastCommand = MCI_SETCURRENTREFERENCES;
  pHandle->Iqdref.q = Iqdref.q;
  pHandle->Iqdref.d = Iqdref.d;
  pHandle->CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
  pHandle->LastModalitySetByUser = STC_TORQUE_MODE;
}


bool MCInterface_StartMotor(MotorControlInterfaceHandle_t * pHandle)
{
  bool RetVal = MCStateMachine_NextState(pHandle->pSTM, M_IDLE_START);

  if (RetVal == true)
  {
    pHandle->CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
  }

  return RetVal;
}


bool MCInterface_StopMotor(MotorControlInterfaceHandle_t * pHandle)
{
  return MCStateMachine_NextState(pHandle->pSTM, M_ANY_STOP);
}


bool MCInterface_FaultAcknowledged(MotorControlInterfaceHandle_t * pHandle)
{
  return MCStateMachine_FaultAcknowledged(pHandle->pSTM);
}


void MCInterface_ExecBufferedCommands(MotorControlInterfaceHandle_t * pHandle)
{
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


MCInterfaceCommandState_t  MCI_IsCommandAcknowledged(MotorControlInterfaceHandle_t * pHandle)
{
  MCInterfaceCommandState_t retVal = pHandle->CommandState;

  if ((retVal == MCI_COMMAND_EXECUTED_SUCCESFULLY) |
       (retVal == MCI_COMMAND_EXECUTED_UNSUCCESFULLY))
  {
    pHandle->CommandState = MCI_BUFFER_EMPTY;
  }
  return retVal;
}


MotorState_t  MCInterface_GetSTMState(MotorControlInterfaceHandle_t * pHandle)
{
  return MCStateMachine_GetState(pHandle->pSTM);
}


uint16_t MCInterface_GetOccurredFaults(MotorControlInterfaceHandle_t * pHandle)
{
  return (uint16_t)(MCStateMachine_GetFaultState(pHandle->pSTM));
}


uint16_t MCInterface_GetCurrentFaults(MotorControlInterfaceHandle_t * pHandle)
{
  return (uint16_t)(MCStateMachine_GetFaultState(pHandle->pSTM) >> 16);
}


STCModality_t MCInterface_GetControlMode(MotorControlInterfaceHandle_t * pHandle)
{
  return pHandle->LastModalitySetByUser;
}


int16_t MCInterface_GetImposedMotorDirection(MotorControlInterfaceHandle_t * pHandle)
{
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


int16_t MCInterface_GetLastRampFinalSpeed(MotorControlInterfaceHandle_t * pHandle)
{
  int16_t hRetVal = 0;

  /* Examine the last buffered commands */
  if (pHandle->LastCommand == MCI_EXECSPEEDRAMP)
  {
    hRetVal = pHandle->hFinalSpeed;
  }
  return hRetVal;
}


bool MCInterface_IsRampCompleted(MotorControlInterfaceHandle_t * pHandle)
{
  bool retVal = false;

  if ((MCStateMachine_GetState(pHandle->pSTM)) == M_RUN)
  {
    retVal = SpdTorqCtrl_IsRampCompleted(pHandle->pSpeedTorqCtrl);
  }

  return retVal;
}


void MCInterface_StopRamp(MotorControlInterfaceHandle_t * pHandle)
{
   SpdTorqCtrl_StopRamp(pHandle->pSpeedTorqCtrl);
}


bool MCInterface_GetSpdSensorReliability(MotorControlInterfaceHandle_t * pHandle)
{
  SpeednPosFdbkHandle_t * SpeedSensor = SpdTorqCtrl_GetSpeedSensor(pHandle->pSpeedTorqCtrl);

  return (SpdPosFdbk_GetReliability(SpeedSensor));
}


int16_t MCInterface_GetAvrgMecSpeedUnit(MotorControlInterfaceHandle_t * pHandle)
{
  SpeednPosFdbkHandle_t * SpeedSensor = SpdTorqCtrl_GetSpeedSensor(pHandle->pSpeedTorqCtrl);

  return (SpdPosFdbk_GetAvrgMecSpeedUnit(SpeedSensor));
}


int16_t MCInterface_GetMecSpeedRefUnit(MotorControlInterfaceHandle_t * pHandle)
{
  return (SpdTorqCtrl_GetMecSpeedRefUnit(pHandle->pSpeedTorqCtrl));
}


ab_t MCInterface_GetIab(MotorControlInterfaceHandle_t * pHandle)
{
  return (pHandle->pFOCVars->Iab);
}


AlphaBeta_t MCInterface_GetIalphabeta(MotorControlInterfaceHandle_t * pHandle)
{
  return (pHandle->pFOCVars->Ialphabeta);
}


qd_t MCInterface_GetIqd(MotorControlInterfaceHandle_t * pHandle)
{
  return (pHandle->pFOCVars->Iqd);
}


qd_t MCInterface_GetIqdHF(MotorControlInterfaceHandle_t * pHandle)
{
  return (pHandle->pFOCVars->IqdHF);
}


qd_t MCInterface_GetIqdref(MotorControlInterfaceHandle_t * pHandle)
{
  return (pHandle->pFOCVars->Iqdref);
}


qd_t MCInterface_GetVqd(MotorControlInterfaceHandle_t * pHandle)
{
  return (pHandle->pFOCVars->Vqd);
}


AlphaBeta_t MCInterface_GetValphabeta(MotorControlInterfaceHandle_t * pHandle)
{
  return (pHandle->pFOCVars->Valphabeta);
}


int16_t MCInterface_GetElAngledpp(MotorControlInterfaceHandle_t * pHandle)
{
  return (pHandle->pFOCVars->hElAngle);
}


int16_t MCInterface_GetTeref(MotorControlInterfaceHandle_t * pHandle)
{
  return (pHandle->pFOCVars->hTeref);
}


int16_t MCInterface_GetPhaseCurrentAmplitude(MotorControlInterfaceHandle_t * pHandle)
{
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


int16_t MCInterface_GetPhaseVoltageAmplitude(MotorControlInterfaceHandle_t * pHandle)
{
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


void MCInterface_ClearIqdref(MotorControlInterfaceHandle_t * pHandle)
{
  pHandle->pFOCVars->Iqdref = SpdTorqCtrl_GetDefaultIqdref(pHandle->pSpeedTorqCtrl);
}
