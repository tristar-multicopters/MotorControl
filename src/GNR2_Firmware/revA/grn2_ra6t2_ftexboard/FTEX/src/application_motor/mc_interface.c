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
#include "mc_tasks.h"
#include "motor_signal_processing.h"
#include "Regen.h"

/* Private macros ------------------------------------------------------------*/
/**
  * @brief This macro converts the exported enum from the state machine to the corresponding bit field.
  */
#define BC(state) (1u<<((uint16_t)((uint8_t)(state))))

/* Functions -----------------------------------------------*/

/*
* see function definition
*/
void MCInterface_PowerInit(MC_Setup_t MCSetup)
{
    SpdTorqCtrl_PowerInit(MCInterface[M1].pSpeedTorqCtrl, MCSetup, MotorParameters);
    ResDivVbusSensor_UVInit(MCInterface[M1].pResDivVbusSensor, MCSetup);
}

/*
* see function definition
*/
void MCInterface_SpeedLimitEnInit(MC_Setup_t MCSetup)
{
    SpdTorqCtrl_SpeedLimitEnInit(MCInterface[M1].pSpeedTorqCtrl, MCSetup);
}

/*
* see function definition
*/
void MCInterface_ExecSpeedRamp(int16_t hFinalSpeed)
{
  MCInterface[M1].LastCommand = MCI_EXECSPEEDRAMP;
  MCInterface[M1].hFinalSpeed = hFinalSpeed;
  MCInterface[M1].CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
  MCInterface[M1].LastModalitySetByUser = STC_SPEED_MODE;
}

/*
* see function definition
*/
void MCInterface_ExecTorqueRamp(int16_t hFinalTorque)
{
  MCInterface[M1].LastCommand = MCI_EXECTORQUERAMP;
  MCInterface[M1].hFinalTorque = hFinalTorque;
  MCInterface[M1].CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
  MCInterface[M1].LastModalitySetByUser = STC_TORQUE_MODE;
}

/*
* see function definition
*/
void MCInterface_SetCurrentReferences(qd_t Iqdref)
{
  MCInterface[M1].LastCommand = MCI_SETCURRENTREFERENCES;
  MCInterface[M1].Iqdref.q = Iqdref.q;
  MCInterface[M1].Iqdref.d = Iqdref.d;
  MCInterface[M1].CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
  MCInterface[M1].LastModalitySetByUser = STC_TORQUE_MODE;
}

/*
* see function definition
*/
bool MCInterface_StartMotor()
{
  bool bRetVal = false;

  if (MCStateMachine_GetState(MCInterface[M1].pSTM) == M_IDLE && !MCStateMachine_GetOccurredErrorState(MCInterface[M1].pSTM))
  {
      bRetVal = MCStateMachine_NextState(MCInterface[M1].pSTM, M_IDLE_START);
  }
  return bRetVal;
}

/*
* see function definition
*/
bool MCInterface_StopMotor()
{
  bool bRetVal = false;

  if (MCStateMachine_GetState(MCInterface[M1].pSTM) == M_RUN)
  {
      bRetVal = MCStateMachine_NextState(MCInterface[M1].pSTM, M_ANY_STOP);
  }
  return bRetVal;
}

/*
* see function definition
*/
bool MCInterface_CriticalFaultAcknowledged()
{
  return MCStateMachine_CriticalFaultAcknowledged(MCInterface[M1].pSTM);
}

/*
* see function definition
*/
void MCInterface_ExecBufferedCommands()
{
    if (MCInterface[M1].CommandState == MCI_COMMAND_NOT_ALREADY_EXECUTED)
    {
        bool commandHasBeenExecuted = false;
        switch (MCInterface[M1].LastCommand)
        {
            case MCI_EXECSPEEDRAMP:
            {
                MCInterface[M1].pFOCVars->bDriveInput = INTERNAL;
                SpdTorqCtrl_SetControlMode(MCInterface[M1].pSpeedTorqCtrl, STC_SPEED_MODE);
                commandHasBeenExecuted = SpdTorqCtrl_ExecRamp(MCInterface[M1].pSpeedTorqCtrl, MCInterface[M1].hFinalSpeed);
            }                               
            break;
            case MCI_EXECTORQUERAMP:
            {
                MCInterface[M1].pFOCVars->bDriveInput = INTERNAL;
                SpdTorqCtrl_SetControlMode(MCInterface[M1].pSpeedTorqCtrl, STC_TORQUE_MODE);
                commandHasBeenExecuted = SpdTorqCtrl_ExecRamp(MCInterface[M1].pSpeedTorqCtrl, MCInterface[M1].hFinalTorque);
            }
            break;
            case MCI_SETCURRENTREFERENCES:
            {
                MCInterface[M1].pFOCVars->bDriveInput = EXTERNAL;
                MCInterface[M1].pFOCVars->Iqdref = MCInterface[M1].Iqdref;
                commandHasBeenExecuted = true;
            }
            break;
            default:
            break;
        }

        if (commandHasBeenExecuted)
        {
            MCInterface[M1].CommandState = MCI_COMMAND_EXECUTED_SUCCESFULLY;
        }
        else
        {
            MCInterface[M1].CommandState = MCI_COMMAND_EXECUTED_UNSUCCESFULLY;
        }
    }       
}

/*
* see function definition
*/
MotorState_t  MCInterface_GetSTMState()
{
    return MCStateMachine_GetState(MCInterface[M1].pSTM);
}

/*
* see function definition
*/
uint32_t MCInterface_GetOccurredCriticalFaults()
{
    return (uint32_t)(MCStateMachine_GetCriticalFaultState(MCInterface[M1].pSTM));
}

/*
* see function definition
*/
uint32_t MCInterface_GetCurrentErrors()
{
    return MCStateMachine_GetCurrentErrorState(MCInterface[M1].pSTM);
}

/*
* see function definition
*/
uint32_t MCInterface_GetOccurredErrors()
{
    return MCStateMachine_GetOccurredErrorState(MCInterface[M1].pSTM);
}

/*
* see function definition
*/
uint32_t MCInterface_GetOccurredWarning()
{
    return MCStateMachine_GetWarningState(MCInterface[M1].pSTM);
}

/*
* see function definition
*/
uint32_t MCInterface_GetCurrentCriticalFaults()
{
    return (uint32_t)(MCStateMachine_GetCriticalFaultState(MCInterface[M1].pSTM) >> 32);
}

/*
* see function definition
*/
STCModality_t MCInterface_GetControlMode()
{
    return MCInterface[M1].LastModalitySetByUser;
}

/*
* see function definition
*/
int16_t MCInterface_GetImposedMotorDirection()
{
    int16_t retVal = 1;

    switch (MCInterface[M1].LastCommand)
    {
        case MCI_EXECSPEEDRAMP:
          if (MCInterface[M1].hFinalSpeed < 0)
          {
            retVal = -1;
          }
          break;
        case MCI_EXECTORQUERAMP:
          if (MCInterface[M1].hFinalTorque < 0)
          {
            retVal = -1;
          }
          break;
        case MCI_SETCURRENTREFERENCES:
          if (MCInterface[M1].Iqdref.q < 0)
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
int16_t MCInterface_GetLastRampFinalSpeed()
{
    int16_t hRetVal = 0;

    /* Examine the last buffered commands */
    if (MCInterface[M1].LastCommand == MCI_EXECSPEEDRAMP)
    {
        hRetVal = MCInterface[M1].hFinalSpeed;
    }
    return hRetVal;
}

/*
* see function definition
*/
bool MCInterface_IsRampCompleted()
{
    bool retVal = false;

    if ((MCStateMachine_GetState(MCInterface[M1].pSTM)) == M_RUN)
    {
        retVal = SpdTorqCtrl_IsRampCompleted(MCInterface[M1].pSpeedTorqCtrl);
    }

    return retVal;
}

/*
* see function definition
*/
void MCInterface_StopRamp()
{
    SpdTorqCtrl_StopRamp(MCInterface[M1].pSpeedTorqCtrl);
}

/*
* see function definition
*/
bool MCInterface_GetSpdSensorReliability()
{
    SpdPosFdbkHandle_t * SpeedSensor = SpdTorqCtrl_GetSpeedSensor(MCInterface[M1].pSpeedTorqCtrl);

    return (SpdPosFdbk_GetReliability(SpeedSensor));
}

/*
* see function definition
*/
int16_t MCInterface_GetAvrgMecSpeedUnit()
{
    SpdPosFdbkHandle_t * SpeedSensor = SpdTorqCtrl_GetSpeedSensor(MCInterface[M1].pSpeedTorqCtrl);
    
    return (SpdPosFdbk_GetAvrgMecSpeedUnit(SpeedSensor));
}

/*
* see function definition
*/
int16_t MCInterface_GetMecSpeedRefUnit()
{
    return (SpdTorqCtrl_GetMecSpeedRefUnit(MCInterface[M1].pSpeedTorqCtrl));
}

/*
* see function definition
*/
ab_t MCInterface_GetIab()
{
    ASSERT(MCInterface[M1].pFOCVars != NULL);
    return (MCInterface[M1].pFOCVars->Iab);
}

/*
* see function definition
*/
AlphaBeta_t MCInterface_GetIalphabeta()
{
    ASSERT(MCInterface[M1].pFOCVars != NULL);
    return (MCInterface[M1].pFOCVars->Ialphabeta);
}

/*
* see function definition
*/
qd_t MCInterface_GetIqd()
{
    ASSERT(MCInterface[M1].pFOCVars != NULL);
    return (MCInterface[M1].pFOCVars->Iqd);
}

/*
* see function definition
*/
qd_t MCInterface_GetIqdHF()
{
    ASSERT(MCInterface[M1].pFOCVars != NULL);
    return (MCInterface[M1].pFOCVars->IqdHF);
}

/*
* see function definition
*/
qd_t MCInterface_GetIqdref()
{
    ASSERT(MCInterface[M1].pFOCVars != NULL);
    return (MCInterface[M1].pFOCVars->Iqdref);
}

/*
* see function definition
*/
qd_t MCInterface_GetVqd()
{
    ASSERT(MCInterface[M1].pFOCVars != NULL);
    return (MCInterface[M1].pFOCVars->Vqd);
}

/*
* see function definition
*/
AlphaBeta_t MCInterface_GetValphabeta()
{
    ASSERT(MCInterface[M1].pFOCVars != NULL);
    return (MCInterface[M1].pFOCVars->Valphabeta);
}

/*
* see function definition
*/
int16_t MCInterface_GetElAngledpp()
{
    ASSERT(MCInterface[M1].pFOCVars != NULL);
    return (MCInterface[M1].pFOCVars->hElAngle);
}

/*
* see function definition
*/
int16_t MCInterface_GetTeref()
{
    ASSERT(MCInterface[M1].pFOCVars != NULL);
    return (MCInterface[M1].pFOCVars->hTeref);
}

/*
* see function definition
*/
int16_t MCInterface_GetPhaseCurrentAmplitude()
{
    ASSERT(MCInterface[M1].pFOCVars != NULL);

    AlphaBeta_t Local_Curr;
    int32_t wAux1, wAux2;

    Local_Curr = MCInterface[M1].pFOCVars->Ialphabeta;
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
int16_t MCInterface_GetPhaseVoltageAmplitude()
{
      ASSERT(MCInterface[M1].pFOCVars != NULL);
      
      AlphaBeta_t Local_Voltage;
      int32_t wAux1, wAux2;

      Local_Voltage = MCInterface[M1].pFOCVars->Valphabeta;
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
uint16_t MCInterface_GetBusVoltageInVoltx100()
{   
    uint32_t VoltageConverted = 0;
    uint16_t ConversionFactor = 0;
    
    ConversionFactor = MCInterface[M1].pResDivVbusSensor->Super.hConversionFactor;
    VoltageConverted = VbusSensor_GetAvBusVoltageDigital(&(MCInterface[M1].pResDivVbusSensor->Super));
    VoltageConverted = VoltageConverted * ConversionFactor * 100u;
    VoltageConverted = VoltageConverted/65536u;
         
    return (uint16_t) VoltageConverted; // Return voltage * 100 so 49.63 V will be 4963.0
                                       // This is done to keep precision 
}

/**
  *  Getting the controller NTC temperature value
  */
int16_t MCInterface_GetControllerTemp()
{
    ASSERT(MCInterface[M1].pSpeedTorqCtrl->pHeatsinkTempSensor != NULL);
    
    return MCInterface[M1].pSpeedTorqCtrl->pHeatsinkTempSensor->hAvTempCelcius;
}

/**
  *  Getting the motor NTC temperature value
  */
int16_t MCInterface_GetMotorTemp()
{
    ASSERT(MCInterface[M1].pSpeedTorqCtrl->pMotorTempSensor != NULL);
    
    return MCInterface[M1].pSpeedTorqCtrl->pMotorTempSensor->hAvTempCelcius;
}

/**
  *  Get the Max safe current from motor control.
  */
int16_t MCInterface_GetMaxCurrent()
{
    ASSERT(MCInterface[M1].pMCConfig != NULL);
    
    return MCInterface[M1].pMCConfig->hNominalCurr;
}

/**
  *  Get the maximum ongoing current,
  */
int16_t MCInterface_GetOngoingMaxCurrent()
{
    ASSERT(MCInterface[M1].pMCConfig != NULL);
    
    return MCInterface[M1].pMCConfig->wUsrMaxCurr;
}

/**
  *  Set the maximum ongoing current, 
  */
void MCInterface_SetOngoingMaxCurrent(int16_t aCurrent)
{
    ASSERT(MCInterface[M1].pMCConfig != NULL);
    
    MCInterface[M1].pMCConfig->wUsrMaxCurr = aCurrent;
}

/**
  *  Set the wheel RPM
  */
void  MCInterface_SetWheelRPM(uint16_t aWheelRPM)
{
    MCInterface[M1].pSpeedTorqCtrl->pSPD->wheelRPM = aWheelRPM;
    
}   

/**
  *  Get the current torq reference 
  */
int16_t MCInterface_GetTorqueReference(uint8_t Motor)
{
    if (Motor == 0)
    {
        return MCInterface[M1].pFOCVars[M1].hTeref;
    }
    else
    {
        return MCInterface[M1].pFOCVars[M2].hTeref;
    }          
}

/**
  *  Get the max application power
  */
uint16_t MCInterface_GetMaxPositivePower()
{
    return MCInterface[M1].pSpeedTorqCtrl->hMaxPositivePower;
}


//******************************************* REGEN API */
/**
  *  enable the regen feature,
  */
void MCInterface_EnableRegen()
{   
    return RegenSetEnabled();
}

/**
  *  disable the regen feature,
  */
void MCInterface_DisableRegen()
{
    return RegenSetDisabled();
}

/**
  *  Get the regen enabled state
  */
bool MCInterface_GetRegenEnabled()
{
    return RegenGetEnabled();
}

/**
  *  Set the max negative battery current in amps
  */
bool MCInterface_SetRegenMaxNegativeCurrent(int16_t Idc_Negative)
{
    return RegenSetMaxCurrent(Idc_Negative); 
}

/**
  *  Get the max negative battery current in amps
  */
int16_t MCInterface_GetRegenMaxNegativeCurrent()
{
    return MCInterface[M1].pSpeedTorqCtrl->pSPD->hIdcRegen;
}


/**
  *  Set the min negative battery current in amps
  */
bool MCInterface_SetMinNegativeCurrent(int16_t MinCurrent)
{
    return RegenSetMinCurrent(-MinCurrent); 
}

/**
  *  Get the min negative battery current in amps
  */
int16_t MCInterface_GetMinNegativeCurrent()
{
    return RegenGetMinCurrent();
}

/**
  *  Set the rate of increasing the negative Torque in mili Nm per milisecond or Nm/sec
  */
bool MCInterface_SetRegenRampPercent(uint16_t Percent)
{
    return RegenSetRampPercent(Percent); 
}

/**
  *  Get the rate of increasing the negative Torque in mili Nm per milisecond or Nm/sec
  */
uint16_t MCInterface_GetRegenRampPercent()
{
    return RegenGetRampPercent();
}

/**
  *  Get the regenerative torque value in mili Nm
  */
bool MCInterface_SetRegenMaxVoltage(uint16_t hMaxVoltage)
{
    return RegenSetMaxVoltage(hMaxVoltage);
}

/**
  *  Get the regenerative torque value in mili Nm
  */
uint16_t MCInterface_GetRegenMaxVoltage()
{
    return RegenGetMaxVoltage();
}

/**
  *  Set the regen level in percent
  */
bool MCInterface_SetRegenLevelPercent(uint8_t Level)
{
    return RegenSetLevelPercent(Level);
}


// END OF REGEN API ********************************************/

/**
  *  Get the motor gear ratio
  */
float MCInterface_GetMotorGearRatio()
{
    return MCInterface[M1].pSpeedTorqCtrl->fGearRatio;
}

/**
  *  Get the motor type
  */
MotorType_t MCInterface_GetMotorType()
{
    return MCInterface[M1].pSpeedTorqCtrl->motorType;
}

/**
  *  Get the nominal torque
  */
uint16_t MCInterface_GetNominalTorque()
{
    return MCInterface[M1].pSpeedTorqCtrl->hMaxPositiveTorque;
}

/**
  *  Get the starting torque
  */
uint16_t MCInterface_GetStartingTorque()
{
    return MCInterface[M1].pSpeedTorqCtrl->hStartingTorque;
}

/**
  *  Get the RS torque
  */
float MCInterface_GetRS()
{
    return MCInterface[M1].pMCConfig->fRS;
}

/**
  *  Get the number of magnets on the wheel speed sensor
  */
uint8_t MCInterface_GetWheelSpdSensorNbrPerRotation()
{
    return MCInterface[M1].pMCConfig->bWheelSpdSensorNbrPerRotation;
}

/**
  *  Get whether the motor sensor type is mixed or not
  */
bool MCInterface_GetMotorTempSensorMixed()
{
    return MCInterface[M1].pSpeedTorqCtrl->pMotorTempSensor->bSensorMixed;
}

#if AUTOTUNE_ENABLE

/**
  *  This command enters motor tuning mode.
  *  Commands to the motor tuner are only processed in this mode.
  */
bool MCInterface_StartMotorTuning()
{
  bool bRetVal = false;

  if (MCStateMachine_GetState(MCInterface[M1].pSTM) == M_IDLE)
  {
      bRetVal = MCStateMachine_NextState( MCInterface[M1].pSTM, M_AUTOTUNE_ENTER_IDENTIFICATION );
  }
  return bRetVal;
}

/**
  *  This command exits motor tuning mode.
  *  Commands to the motor tuner are only processed in this mode.
  */
bool MCInterface_StopMotorTuning()
{
  bool bRetVal = false;

  if (MCStateMachine_GetState(MCInterface[M1].pSTM) == M_AUTOTUNE_IDENTIFICATION)
  {
      bRetVal = MCStateMachine_NextState( MCInterface[M1].pSTM, M_AUTOTUNE_ANY_STOP_IDENTIFICATION );
  }
  return bRetVal;
}
#endif

/**
  *  This command enables the flux weakening logic.
  */
void MCInterface_EnableFluxWeakening()
{
    MCInterface[M1].pSpeedTorqCtrl->bFluxWeakeningEn = true;
}

/**
  *  This command disables the flux weakening logic.
  */

void MCInterface_DisableFluxWeakening()
{
    MCInterface[M1].pSpeedTorqCtrl->bFluxWeakeningEn = false;
}

/*
    set speed limit of wheel
*/
void MCInterface_SetSpeedLimitWheelRpm(uint16_t wheelRpm)
{
    SpdTorqCtrl_SetSpeedLimitWheelRpm(MCInterface[M1].pSpeedTorqCtrl, wheelRpm);
}

/*
    set speed limit of motor
*/
void MCInterface_SetSpeedLimit(int16_t desiredMotorRPM)
{
    SpdTorqCtrl_SetSpeedLimit(MCInterface[M1].pSpeedTorqCtrl, desiredMotorRPM);
}

/*
    get max measurable current
*/
float MCInterface_GetMaxMeasurableCurrent()
{
    return MCInterface[M1].MCIConvFactors.MaxMeasurableCurrent;
}

/*
    set max bus current
*/
void MCInterface_SetMaxBusCurrent(uint16_t maxBusCurrent)
{
    MCInterface[M1].pSpeedTorqCtrl->hMaxBusCurrent = maxBusCurrent;
}

/*
    set max continuous current
*/
void MCInterface_SetMaxContinuousCurrent(uint16_t maxContinuousCurrent)
{
    MCInterface[M1].pSpeedTorqCtrl->hMaxContinuousCurrent = maxContinuousCurrent;
}

/*
    set power foldback end value
*/
void MCInterface_SetPowerFoldbackEndValue(int32_t endValue)
{
    MCInterface[M1].pSpeedTorqCtrl->FoldbackDynamicMaxPower.hDecreasingEndValue = endValue;
}

/*
    set power foldback range
*/
void MCInterface_SetPowerFoldbackRange(uint16_t range)
{
    MCInterface[M1].pSpeedTorqCtrl->FoldbackDynamicMaxPower.hDecreasingRange = range;
}

/*
    set motor temp sensor type
*/
void MCInterface_SetMotorTempSensorType(uint8_t sensorType)
{
    MCInterface[M1].pSpeedTorqCtrl->pMotorTempSensor->bSensorType = sensorType;
}

/*
    set motor NTC Beta coefficient
*/
void MCInterface_SetMotorNTCBetaCoef(uint16_t NTCBetaCoef)
{
    MCInterface[M1].pSpeedTorqCtrl->pMotorTempSensor->hNTCBetaCoef = NTCBetaCoef;
}

/*
    set motor NTC resistance coefficient
*/
void MCInterface_SetMotorNTCResistanceCoef(float NTCResCoef)
{
    MCInterface[M1].pSpeedTorqCtrl->pMotorTempSensor->hNTCResCoef = NTCResCoef;
}

/*
    set is motor signal mixed
*/
void MCInterface_SetIsMotorSignalMixed(bool value)
{
    updateisMotorMixedSignalValue(value);
}

/*
    set minimum signal threshold value
*/
void MCInterface_SetMinSignalThresholdValueMixed(uint16_t value)
{
    updateMinSignalThresholdValue(value);
}

/*
    set wheel speed period for mixed signal
*/
void MCInterface_SetMaxWheelSpeedPeriodUsValueMixed(uint32_t value)
{
    updateMaxWheelSpeedPeriodUsValue(value);
}

/*
    get extracted wheel speed from mixed signal
*/
float MCInterface_GetExtractedWheelSpeedMixed(void)
{
    return -getExtractedWheelSpeed();
}

/**
  *  Set the maximum regen current
  */
bool MCInterface_SetRegenMaxCurrent(int16_t maxCurrent)
{
    return RegenSetMaxCurrent(-maxCurrent);
}

/**
  *  Set the minimum speed for regen to start
  */
bool MCInterface_SetRegenMinSpeed(int16_t minSpeed)
{
    return RegenSetMinSpeed(minSpeed);
}
