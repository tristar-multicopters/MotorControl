/**
  * @file    pwm_curr_fdbk.c
  * @author  Sami Bouzid, FTEX inc
  * @brief   This file provides firmware functions that implement the following features
  *          of the PWM & Current Feedback component:
  *
  *           * current sensing
  *           * regular ADC conversion execution
  *           * space vector modulation
  *
*/

/* Includes ------------------------------------------------------------------*/
#include "pwm_curr_fdbk.h"

#include "mc_type.h"


void PWMCurrFdbk_GetPhaseCurrents( PWMCurrFdbkHandle_t * pHandle, ab_t * Iab )
{
  pHandle->pFctGetPhaseCurrents( pHandle, Iab );
}


uint16_t PWMCurrFdbk_SetPhaseVoltage( PWMCurrFdbkHandle_t * pHandle, AlphaBeta_t Valfa_beta )
{
  int32_t wX, wY, wZ, wUAlpha, wUBeta, wTimePhA, wTimePhB, wTimePhC;

  wUAlpha = Valfa_beta.alpha * ( int32_t )pHandle->hT_Sqrt3;
  wUBeta = -( Valfa_beta.beta * ( int32_t )( pHandle->hPWMperiod ) ) * 2;

  wX = wUBeta;
  wY = ( wUBeta + wUAlpha ) / 2;
  wZ = ( wUBeta - wUAlpha ) / 2;

  /* Sector calculation from wX, wY, wZ */
  if ( wY < 0 )
  {
    if ( wZ < 0 )
    {
      pHandle->Sector = SECTOR_5;
      wTimePhA = ( int32_t )( pHandle->hPWMperiod ) / 4 + ( ( wY - wZ ) / ( int32_t )262144 );
      wTimePhB = wTimePhA + wZ / 131072;
      wTimePhC = wTimePhA - wY / 131072;
      pHandle->hLowDuty = wTimePhC;
      pHandle->hMidDuty = wTimePhA;
      pHandle->hHighDuty = wTimePhB;
    }
    else /* wZ >= 0 */
      if ( wX <= 0 )
      {
        pHandle->Sector = SECTOR_4;
        wTimePhA = ( int32_t )( pHandle->hPWMperiod ) / 4 + ( ( wX - wZ ) / ( int32_t )262144 );
        wTimePhB = wTimePhA + wZ / 131072;
        wTimePhC = wTimePhB - wX / 131072;
        pHandle->hLowDuty = wTimePhC;
        pHandle->hMidDuty = wTimePhB;
        pHandle->hHighDuty = wTimePhA;
      }
      else /* wX > 0 */
      {
        pHandle->Sector = SECTOR_3;
        wTimePhA = ( int32_t )( pHandle->hPWMperiod ) / 4 + ( ( wY - wX ) / ( int32_t )262144 );
        wTimePhC = wTimePhA - wY / 131072;
        wTimePhB = wTimePhC + wX / 131072;
        pHandle->hLowDuty = wTimePhB;
        pHandle->hMidDuty = wTimePhC;
        pHandle->hHighDuty = wTimePhA;
      }
  }
  else /* wY > 0 */
  {
    if ( wZ >= 0 )
    {
      pHandle->Sector = SECTOR_2;
      wTimePhA = ( int32_t )( pHandle->hPWMperiod ) / 4 + ( ( wY - wZ ) / ( int32_t )262144 );
      wTimePhB = wTimePhA + wZ / 131072;
      wTimePhC = wTimePhA - wY / 131072;
      pHandle->hLowDuty = wTimePhB;
      pHandle->hMidDuty = wTimePhA;
      pHandle->hHighDuty = wTimePhC;
    }
    else /* wZ < 0 */
      if ( wX <= 0 )
      {
        pHandle->Sector = SECTOR_6;
        wTimePhA = ( int32_t )( pHandle->hPWMperiod ) / 4 + ( ( wY - wX ) / ( int32_t )262144 );
        wTimePhC = wTimePhA - wY / 131072;
        wTimePhB = wTimePhC + wX / 131072;
        pHandle->hLowDuty = wTimePhA;
        pHandle->hMidDuty = wTimePhC;
        pHandle->hHighDuty = wTimePhB;
      }
      else /* wX > 0 */
      {
        pHandle->Sector = SECTOR_1;
        wTimePhA = ( int32_t )( pHandle->hPWMperiod ) / 4 + ( ( wX - wZ ) / ( int32_t )262144 );
        wTimePhB = wTimePhA + wZ / 131072;
        wTimePhC = wTimePhB - wX / 131072;
        pHandle->hLowDuty = wTimePhA;
        pHandle->hMidDuty = wTimePhB;
        pHandle->hHighDuty = wTimePhC;
      }
  }

  pHandle->hCntPhA = (uint16_t)(MAX(wTimePhA,0));
  pHandle->hCntPhB = (uint16_t)(MAX(wTimePhB,0));
  pHandle->hCntPhC = (uint16_t)(MAX(wTimePhC,0));

  return ( pHandle->pFctSetADCSampPointSectX( pHandle ) );
}


void PWMCurrFdbk_SwitchOffPWM( PWMCurrFdbkHandle_t * pHandle )
{
  pHandle->pFctSwitchOffPwm( pHandle );
}


void PWMCurrFdbk_SwitchOnPWM( PWMCurrFdbkHandle_t * pHandle )
{
  pHandle->pFctSwitchOnPwm( pHandle );
}


bool PWMCurrFdbk_CurrentReadingCalibr( PWMCurrFdbkHandle_t * pHandle )
{
  bool retVal = true;
  
	PWMCurrFdbk_SwitchOffPWM( pHandle );
	
  pHandle->pFctCurrReadingCalib( pHandle );

  return retVal;
}


void PWMCurrFdbk_TurnOnLowSides( PWMCurrFdbkHandle_t * pHandle )
{
  pHandle->pFctTurnOnLowSides( pHandle );
}



uint16_t PWMCurrFdbk_CheckOverCurrent( PWMCurrFdbkHandle_t * pHandle )
{
  return pHandle->pFctIsOverCurrentOccurred( pHandle );
}


bool PWMCurrFdbk_GetTurnOnLowSidesAction( PWMCurrFdbkHandle_t * pHandle )
{
  return pHandle->hTurnOnLowSidesAction;
}


void PWMCurrFdbk_RLDetectionModeEnable( PWMCurrFdbkHandle_t * pHandle )
{
  if ( pHandle->pFctRLDetectionModeEnable )
  {
    pHandle->pFctRLDetectionModeEnable( pHandle );
  }
}


void PWMCurrFdbk_RLDetectionModeDisable( PWMCurrFdbkHandle_t * pHandle )
{
  if ( pHandle->pFctRLDetectionModeDisable )
  {
    pHandle->pFctRLDetectionModeDisable( pHandle );
  }
}


uint16_t PWMCurrFdbk_RLDetectionModeSetDuty( PWMCurrFdbkHandle_t * pHandle, uint16_t hDuty )
{
  uint16_t hRetVal = MC_FOC_DURATION;
  if ( pHandle->pFctRLDetectionModeSetDuty )
  {
    hRetVal = pHandle->pFctRLDetectionModeSetDuty( pHandle, hDuty );
  }
  return hRetVal;
}


void PWMCurrFdbk_RegisterGetPhaseCurrentsCallBack( PWMCurrFdbk_GetPhaseCurr_Cb_t pCallBack,
    PWMCurrFdbkHandle_t * pHandle )
{
  pHandle->pFctGetPhaseCurrents = pCallBack;
}


void PWMCurrFdbk_RegisterSwitchOffPwmCallBack( PWMCurrFdbk_Generic_Cb_t pCallBack,
                                        PWMCurrFdbkHandle_t * pHandle )
{
  pHandle->pFctSwitchOffPwm = pCallBack;
}


void PWMCurrFdbk_RegisterSwitchonPwmCallBack( PWMCurrFdbk_Generic_Cb_t pCallBack,
                                       PWMCurrFdbkHandle_t * pHandle )
{
  pHandle->pFctSwitchOnPwm = pCallBack;
}


void PWMCurrFdbk_RegisterReadingCalibrationCallBack( PWMCurrFdbk_Generic_Cb_t pCallBack,
    PWMCurrFdbkHandle_t * pHandle )
{
  pHandle->pFctCurrReadingCalib = pCallBack;
}


void PWMCurrFdbk_RegisterTurnOnLowSidesCallBack( PWMCurrFdbk_Generic_Cb_t pCallBack,
    PWMCurrFdbkHandle_t * pHandle )
{
  pHandle->pFctTurnOnLowSides = pCallBack;
}


void PWMCurrFdbk_RegisterSampPointSectXCallBack( PWMCurrFdbk_SetSampPointSectX_Cb_t pCallBack,
    PWMCurrFdbkHandle_t * pHandle )
{
  pHandle->pFctSetADCSampPointSectX = pCallBack;
}


void PWMCurrFdbk_RegisterIsOverCurrentOccurredCallBack( PWMCurrFdbk_OverCurr_Cb_t pCallBack,
    PWMCurrFdbkHandle_t * pHandle )
{
  pHandle->pFctIsOverCurrentOccurred = pCallBack;
}


void PWMCurrFdbk_RegisterRLDetectionModeEnableCallBack( PWMCurrFdbk_Generic_Cb_t pCallBack,
    PWMCurrFdbkHandle_t * pHandle )
{
  pHandle->pFctRLDetectionModeEnable = pCallBack;
}


void PWMCurrFdbk_RegisterRLDetectionModeDisableCallBack( PWMCurrFdbk_Generic_Cb_t pCallBack,
    PWMCurrFdbkHandle_t * pHandle )
{
  pHandle->pFctRLDetectionModeDisable = pCallBack;
}


void PWMCurrFdbk_RegisterRLDetectionModeSetDutyCallBack( PWMCurrFdbk_RLDetectSetDuty_Cb_t pCallBack,
    PWMCurrFdbkHandle_t * pHandle )
{
  pHandle->pFctRLDetectionModeSetDuty = pCallBack;
}



