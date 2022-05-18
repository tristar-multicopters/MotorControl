/**
  ******************************************************************************
  * @file    pwm_curr_fdbk.c
  * @author  FTEX inc
  * @brief   This file provides firmware functions that implement the following features
  *          of the PWM & Current Feedback component of the Motor Control SDK:
  *
  *           * current sensing
  *           * regular ADC conversion execution
  *           * space vector modulation
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "pwm_curr_fdbk.h"

#include "mc_type.h"

/**
  * @brief Returns the phase current of the motor as read by the ADC (in s16A unit)
  *
  * The function actually returns the current values of phase A & B. Phase C current
  * can be deduced thanks to the formula:
  *
  * @f[
  * I_{C} = -I_{A} - I_{C}
  * @f]
  *
  * @param  pHandle handle on the target PWMC component
  * @param  pStator_Currents Pointer to the structure that will receive motor current
  *         of phase A and B in ElectricalValue format.
*/
void PWMC_GetPhaseCurrents( PWMC_Handle_t * pHandle, ab_t * Iab )
{
  pHandle->pFctGetPhaseCurrents( pHandle, Iab );
}

/**
 * @brief Sets the PWM duty cycles
 *
 *
 */

/**
  * @brief  Converts input voltages @f$ V_{\alpha} @f$ and @f$ V_{\beta} @f$ into PWM duty cycles
  *         and feed them to the inverter.
  * @param  pHandle handler on the target PWMC component.
  * @param  Valfa_beta Voltage Components expressed in the @f$(\alpha, \beta)@f$ reference frame
  *
  * This function computes the the time during wIch the transistors of each phase are to be switched on in
  * a PWM cycle in order to achieve the reference phase voltage set by @p Valfa_beta. Then, the function
  * programs the resulting duty cycles in the related timer channels. It also sets the phase current
  * sampling point for the next PWM cycle accordingly.
  *
  * This function is used in the FOC frequency loop and needs to complete before the next PWM cycle starts
  * so that the duty cycles it computes can be taken into account. Failing to do so (for instance because
  * the PWM Frequency is too high) results in the functions returning #MC_FOC_DURATION wIch entails a
  * Motor Control Fault that stops the motor.
  *
  * @retval Returns #MC_NO_ERROR if no error occurred or #MC_FOC_DURATION if the duty cycles were
  *         set too late for being taken into account in the next PWM cycle.
  */
uint16_t PWMC_SetPhaseVoltage( PWMC_Handle_t * pHandle, alphabeta_t Valfa_beta )
{
  int32_t wX, wY, wZ, wUAlpha, wUBeta, wTimePhA, wTimePhB, wTimePhC;

  wUAlpha = Valfa_beta.alpha * ( int32_t )pHandle->hT_Sqrt3;
  wUBeta = -( Valfa_beta.beta * ( int32_t )( pHandle->PWMperiod ) ) * 2;

  wX = wUBeta;
  wY = ( wUBeta + wUAlpha ) / 2;
  wZ = ( wUBeta - wUAlpha ) / 2;

  /* Sector calculation from wX, wY, wZ */
  if ( wY < 0 )
  {
    if ( wZ < 0 )
    {
      pHandle->Sector = SECTOR_5;
      wTimePhA = ( int32_t )( pHandle->PWMperiod ) / 4 + ( ( wY - wZ ) / ( int32_t )262144 );
      wTimePhB = wTimePhA + wZ / 131072;
      wTimePhC = wTimePhA - wY / 131072;
      pHandle->lowDuty = wTimePhC;
      pHandle->midDuty = wTimePhA;
      pHandle->highDuty = wTimePhB;
    }
    else /* wZ >= 0 */
      if ( wX <= 0 )
      {
        pHandle->Sector = SECTOR_4;
        wTimePhA = ( int32_t )( pHandle->PWMperiod ) / 4 + ( ( wX - wZ ) / ( int32_t )262144 );
        wTimePhB = wTimePhA + wZ / 131072;
        wTimePhC = wTimePhB - wX / 131072;
        pHandle->lowDuty = wTimePhC;
        pHandle->midDuty = wTimePhB;
        pHandle->highDuty = wTimePhA;
      }
      else /* wX > 0 */
      {
        pHandle->Sector = SECTOR_3;
        wTimePhA = ( int32_t )( pHandle->PWMperiod ) / 4 + ( ( wY - wX ) / ( int32_t )262144 );
        wTimePhC = wTimePhA - wY / 131072;
        wTimePhB = wTimePhC + wX / 131072;
        pHandle->lowDuty = wTimePhB;
        pHandle->midDuty = wTimePhC;
        pHandle->highDuty = wTimePhA;
      }
  }
  else /* wY > 0 */
  {
    if ( wZ >= 0 )
    {
      pHandle->Sector = SECTOR_2;
      wTimePhA = ( int32_t )( pHandle->PWMperiod ) / 4 + ( ( wY - wZ ) / ( int32_t )262144 );
      wTimePhB = wTimePhA + wZ / 131072;
      wTimePhC = wTimePhA - wY / 131072;
      pHandle->lowDuty = wTimePhB;
      pHandle->midDuty = wTimePhA;
      pHandle->highDuty = wTimePhC;
    }
    else /* wZ < 0 */
      if ( wX <= 0 )
      {
        pHandle->Sector = SECTOR_6;
        wTimePhA = ( int32_t )( pHandle->PWMperiod ) / 4 + ( ( wY - wX ) / ( int32_t )262144 );
        wTimePhC = wTimePhA - wY / 131072;
        wTimePhB = wTimePhC + wX / 131072;
        pHandle->lowDuty = wTimePhA;
        pHandle->midDuty = wTimePhC;
        pHandle->highDuty = wTimePhB;
      }
      else /* wX > 0 */
      {
        pHandle->Sector = SECTOR_1;
        wTimePhA = ( int32_t )( pHandle->PWMperiod ) / 4 + ( ( wX - wZ ) / ( int32_t )262144 );
        wTimePhB = wTimePhA + wZ / 131072;
        wTimePhC = wTimePhB - wX / 131072;
        pHandle->lowDuty = wTimePhA;
        pHandle->midDuty = wTimePhB;
        pHandle->highDuty = wTimePhC;
      }
  }

  pHandle->CntPhA = (uint16_t)(MAX(wTimePhA,0));
  pHandle->CntPhB = (uint16_t)(MAX(wTimePhB,0));
  pHandle->CntPhC = (uint16_t)(MAX(wTimePhC,0));
  
  if ( pHandle->DTTest == 1u )
  {
    /* Dead time compensation */
    if ( pHandle->Ia > 0 )
    {
      pHandle->CntPhA += pHandle->DTCompCnt;
    }
    else
    {
      pHandle->CntPhA -= pHandle->DTCompCnt;
    }

    if ( pHandle->Ib > 0 )
    {
      pHandle->CntPhB += pHandle->DTCompCnt;
    }
    else
    {
      pHandle->CntPhB -= pHandle->DTCompCnt;
    }

    if ( pHandle->Ic > 0 )
    {
      pHandle->CntPhC += pHandle->DTCompCnt;
    }
    else
    {
      pHandle->CntPhC -= pHandle->DTCompCnt;
    }
  }

  return ( pHandle->pFctSetADCSampPointSectX( pHandle ) );
}

/**
  * @brief  Switches PWM generation off, inactivating the outputs.
  * @param  pHandle Handle on the target instance of the PWMC component
  */
void PWMC_SwitchOffPWM( PWMC_Handle_t * pHandle )
{
  pHandle->pFctSwitchOffPwm( pHandle );
}

/**
  * @brief  Switches PWM generation on
  * @param  pHandle Handle on the target instance of the PWMC component
  */
void PWMC_SwitchOnPWM( PWMC_Handle_t * pHandle )
{
  pHandle->pFctSwitchOnPwm( pHandle );
}

/**
  * @brief  Calibrates ADC current conversions by reading the offset voltage
  *         present on ADC pins when no motor current is flowing in.
  *
  * This function should be called before each motor start-up.
  *
  * @param  pHandle Handle on the target instance of the PWMC component
  * @param  action Can be #CRC_START to initialize the offset calibration or
  *         #CRC_EXEC to execute the offset calibration.
  * @retval true if the current calibration has been completed, false if it is
  *         still ongoing.
  */
bool PWMC_CurrentReadingCalibr( PWMC_Handle_t * pHandle, CRCAction_t action )
{
  bool retVal = false;
  if ( action == CRC_START )
  {
    PWMC_SwitchOffPWM( pHandle );
    pHandle->OffCalibrWaitTimeCounter = pHandle->OffCalibrWaitTicks;
    if ( pHandle->OffCalibrWaitTicks == 0u )
    {
      pHandle->pFctCurrReadingCalib( pHandle );
      retVal = true;
    }
  }
  else if ( action == CRC_EXEC )
  {
    if ( pHandle->OffCalibrWaitTimeCounter > 0u )
    {
      pHandle->OffCalibrWaitTimeCounter--;
      if ( pHandle->OffCalibrWaitTimeCounter == 0u )
      {
        pHandle->pFctCurrReadingCalib( pHandle );
        retVal = true;
      }
    }
    else
    {
      retVal = true;
    }
  }
  else
  {
  }
  return retVal;
}

/**
  * @brief  Switches power stage Low Sides transistors on.
  *
  * This function is meant for charging boot capacitors of the driving
  * section. It has to be called on each motor start-up when using high
  * voltage drivers.
  *
  * @param  pHandle: handle on the target instance of the PWMC component
  */
void PWMC_TurnOnLowSides( PWMC_Handle_t * pHandle )
{
  pHandle->pFctTurnOnLowSides( pHandle );
}


/** @brief Returns #MC_BREAK_IN if an over current condition was detected on the power stage
 *         controlled by the PWMC component pointed by  @p pHandle, since the last call to this function;
 *         returns #MC_NO_FAULTS otherwise. */
uint16_t PWMC_CheckOverCurrent( PWMC_Handle_t * pHandle )
{
  return pHandle->pFctIsOverCurrentOccurred( pHandle );
}

/**
  * @brief  Sets the over current threshold to be used
  *
  * The value to be set is relative to the VDD_DAC DAC reference voltage with
  * 0 standing for 0 V and 65536 standing for VDD_DAC.
  *
  * @param  pHandle handle on the target instance of the PWMC component
  * @param  hDACVref Value of DAC reference expressed as 16bit unsigned integer
  */
void PWMC_OCPSetReferenceVoltage( PWMC_Handle_t * pHandle, uint16_t hDACVref )
{
  if ( pHandle->pFctOCPSetReferenceVoltage )
  {
    pHandle->pFctOCPSetReferenceVoltage( pHandle, hDACVref );
  }
}

/**
  * @brief  It is used to retrieve the satus of TurnOnLowSides action.
  * @param  pHandle: handler of the current instance of the PWMC component
  * @retval bool It returns the state of TurnOnLowSides action:
  *         true if TurnOnLowSides action is active, false otherwise.
  */
/** @brief Returns the status of the "TurnOnLowSide" action on the power stage
 *         controlled by the @p pHandle PWMC component: true if it
 *         is active, false otherwise*/
bool PWMC_GetTurnOnLowSidesAction( PWMC_Handle_t * pHandle )
{
  return pHandle->TurnOnLowSidesAction;
}

/** @brief Enables the RL detection mode on the power stage controlled by the @p pHandle PWMC component. */
void PWMC_RLDetectionModeEnable( PWMC_Handle_t * pHandle )
{
  if ( pHandle->pFctRLDetectionModeEnable )
  {
    pHandle->pFctRLDetectionModeEnable( pHandle );
  }
}

/** @brief Disables the RL detection mode on the power stage controlled by the @p pHandle PWMC component. */
void PWMC_RLDetectionModeDisable( PWMC_Handle_t * pHandle )
{
  if ( pHandle->pFctRLDetectionModeDisable )
  {
    pHandle->pFctRLDetectionModeDisable( pHandle );
  }
}

/**
  * @brief  Sets the PWM duty cycle to apply in the RL Detection mode.
  * @param  pHandle: handle on the target instance of the PWMC component
  * @param  hDuty Duty cycle to apply
  *
  * @todo TODO: Describe the unit of the hDuty variable.
  *
  * @retval If the Duty Cycle could be applied on time for the next PWM period,
  *         #MC_NO_ERROR is returned. Otherwise, #MC_FOC_DURATION is returned.
  */
uint16_t PWMC_RLDetectionModeSetDuty( PWMC_Handle_t * pHandle, uint16_t hDuty )
{
  uint16_t hRetVal = MC_FOC_DURATION;
  if ( pHandle->pFctRLDetectionModeSetDuty )
  {
    hRetVal = pHandle->pFctRLDetectionModeSetDuty( pHandle, hDuty );
  }
  return hRetVal;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to get phases current.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterGetPhaseCurrentsCallBack( PWMC_GetPhaseCurr_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctGetPhaseCurrents = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to switch PWM
 *        generation off.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterSwitchOffPwmCallBack( PWMC_Generic_Cb_t pCallBack,
                                        PWMC_Handle_t * pHandle )
{
  pHandle->pFctSwitchOffPwm = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to switch PWM
 *        generation on.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterSwitchonPwmCallBack( PWMC_Generic_Cb_t pCallBack,
                                       PWMC_Handle_t * pHandle )
{
  pHandle->pFctSwitchOnPwm = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to execute a calibration
 *        of the current sensing system.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterReadingCalibrationCallBack( PWMC_Generic_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctCurrReadingCalib = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to turn low sides on.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterTurnOnLowSidesCallBack( PWMC_Generic_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctTurnOnLowSides = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to compute ADC sampling point
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterSampPointSectXCallBack( PWMC_SetSampPointSectX_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctSetADCSampPointSectX = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to the overcurrent status
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterIsOverCurrentOccurredCallBack( PWMC_OverCurr_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctIsOverCurrentOccurred = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to set the reference
 *        voltage for the over current protection
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterOCPSetRefVoltageCallBack( PWMC_SetOcpRefVolt_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctOCPSetReferenceVoltage = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to enable the R/L detection mode
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterRLDetectionModeEnableCallBack( PWMC_Generic_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctRLDetectionModeEnable = pCallBack;
}

/**
 * @brief Sets the Callback wIch PWMC shall invoke to disable the R/L detection mode
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterRLDetectionModeDisableCallBack( PWMC_Generic_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctRLDetectionModeDisable = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to set the duty cycle
 *        for the R/L detection mode
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterRLDetectionModeSetDutyCallBack( PWMC_RLDetectSetDuty_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctRLDetectionModeSetDuty = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to call PWMC instance IRQ handler
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 * @note this function is deprecated.
 */
void PWMC_RegisterIrqHandlerCallBack( PWMC_IrqHandler_Cb_t pCallBack,
                                      PWMC_Handle_t * pHandle )
{
  pHandle->pFctIrqHandler = pCallBack;
}


