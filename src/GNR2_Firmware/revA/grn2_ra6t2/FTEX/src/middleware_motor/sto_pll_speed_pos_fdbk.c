/**
  * @file    sto_speed_pos_fdbk.c
  * @brief   This file provides firmware functions that implement the features
  *          of the State Observer + PLL Speed & Position Feedback module
  *
	*/

/* Includes ------------------------------------------------------------------*/
#include "sto_pll_speed_pos_fdbk.h"
#include "mc_math.h"

/* Private defines -----------------------------------------------------------*/
#define C6_COMP_CONST1  (int32_t) 1043038
#define C6_COMP_CONST2  (int32_t) 10430

/* Private function prototypes -----------------------------------------------*/
static void BemfObs_Store_Rotor_Speed(BemfObserverPllHandle_t * pHandle, int16_t hRotor_Speed);
static int16_t BemfObs_ExecutePLL(BemfObserverPllHandle_t * pHandle, int16_t hBemfalfaEst,
                               int16_t hBemfbetaEst);
static void BemfObs_InitSpeedBuffer(BemfObserverPllHandle_t * pHandle);


void BemfObsPll_Init(BemfObserverPllHandle_t * pHandle)
{
  int16_t htempk;
  int32_t wAux;


  pHandle->bConsistencyCounter = pHandle->bStartUpConsistThreshold;
  pHandle->bEnableDualCheck = true;

  wAux = (int32_t)1;
  pHandle->hF3Pow2 = 0u;

  htempk = (int16_t)(C6_COMP_CONST1 / (pHandle->hF2));

  while (htempk != 0)
  {
    htempk /= (int16_t)2;
    wAux *= (int32_t)2;
    pHandle->hF3Pow2++;
  }

  pHandle->hF3 = (int16_t)wAux;
  wAux = (int32_t)(pHandle->hF2) * pHandle->hF3;
  pHandle->hC6 = (int16_t)(wAux / C6_COMP_CONST2);

  BemfObsPll_Clear(pHandle);

  PID_Init(& pHandle->PIRegulator);

  /* Acceleration measurement set to zero */
  pHandle->Super.hMecAccelUnitP = 0;

  return;
}


void BemfObsPll_Return(BemfObserverPllHandle_t * pHandle, uint8_t flag)
{
  return;
}


int16_t BemfObsPll_CalcElAngle(BemfObserverPllHandle_t * pHandle, BemfObserverInputs_t * pInputs)
{
  int32_t wAux, wDirection;
  int32_t wIalfa_est_Next, wIbeta_est_Next;
  int32_t wBemf_alfa_est_Next, wBemf_beta_est_Next;
  int16_t hAux, hAux_Alfa, hAux_Beta, hIalfa_err, hIbeta_err, hRotor_Speed,
          hValfa, hVbeta;


  if (pHandle->wBemfalfaEst > (int32_t)(pHandle->hF2)*INT16_MAX)
  {
    pHandle->wBemfalfaEst = INT16_MAX * (int32_t)(pHandle->hF2);
  }
  else if (pHandle->wBemfalfaEst <= -INT16_MAX * (int32_t)(pHandle->hF2))
  {
    pHandle->wBemfalfaEst = -INT16_MAX * (int32_t)(pHandle->hF2);
  }
  else
  {
  }
#ifdef FULL_MISRA_C_COMPLIANCY
  hAux_Alfa = (int16_t)(pHandle->wBemfalfaEst / pHandle->hF2);
#else
  hAux_Alfa = (int16_t)(pHandle->wBemfalfaEst >> pHandle->hF2Log);
#endif

  if (pHandle->wBemfbetaEst > INT16_MAX * (int32_t)(pHandle->hF2))
  {
    pHandle->wBemfbetaEst = INT16_MAX * (int32_t)(pHandle->hF2);
  }
  else if (pHandle->wBemfbetaEst <= -INT16_MAX * (int32_t)(pHandle->hF2))
  {
    pHandle->wBemfbetaEst = -INT16_MAX * (int32_t)(pHandle->hF2);
  }
  else
  {
  }
#ifdef FULL_MISRA_C_COMPLIANCY
  hAux_Beta = (int16_t)(pHandle->wBemfbetaEst / pHandle->hF2);
#else
  hAux_Beta = (int16_t)(pHandle->wBemfbetaEst >> pHandle->hF2Log);
#endif

  if (pHandle->wIalfaEst > INT16_MAX * (int32_t)(pHandle->hF1))
  {
    pHandle->wIalfaEst = INT16_MAX * (int32_t)(pHandle->hF1);
  }
  else if (pHandle->wIalfaEst <= -INT16_MAX * (int32_t)(pHandle->hF1))
  {
    pHandle->wIalfaEst = -INT16_MAX * (int32_t)(pHandle->hF1);
  }
  else
  {
  }

  if (pHandle->wIbetaEst > INT16_MAX * (int32_t)(pHandle->hF1))
  {
    pHandle->wIbetaEst = INT16_MAX * (int32_t)(pHandle->hF1);
  }
  else if (pHandle->wIbetaEst <= -INT16_MAX * (int32_t)(pHandle->hF1))
  {
    pHandle->wIbetaEst = -INT16_MAX * (int32_t)(pHandle->hF1);
  }
  else
  {
  }

#ifdef FULL_MISRA_C_COMPLIANCY
  hIalfa_err = (int16_t)(pHandle->wIalfaEst / pHandle->hF1);
#else
  hIalfa_err = (int16_t)(pHandle->wIalfaEst >> pHandle->hF1Log);
#endif

  hIalfa_err = hIalfa_err - pInputs->Ialfa_beta.alpha;

#ifdef FULL_MISRA_C_COMPLIANCY
  hIbeta_err = (int16_t)(pHandle->wIbetaEst / pHandle->hF1);
#else
  hIbeta_err = (int16_t)(pHandle->wIbetaEst >> pHandle->hF1Log);
#endif

  hIbeta_err = hIbeta_err - pInputs->Ialfa_beta.beta;

  wAux = (int32_t)(pInputs->Vbus) * pInputs->Valfa_beta.alpha;
#ifdef FULL_MISRA_C_COMPLIANCY
  hValfa = (int16_t) (wAux / 65536);
#else
  hValfa = (int16_t) (wAux >> 16);
#endif

  wAux = (int32_t)(pInputs->Vbus) * pInputs->Valfa_beta.beta;
#ifdef FULL_MISRA_C_COMPLIANCY
  hVbeta = (int16_t) (wAux / 65536);
#else
  hVbeta = (int16_t) (wAux >> 16);
#endif

  /*alfa axes observer*/
#ifdef FULL_MISRA_C_COMPLIANCY
  hAux = (int16_t) (pHandle->wIalfaEst / pHandle->hF1);
#else
  hAux = (int16_t) (pHandle->wIalfaEst >> pHandle->hF1Log);
#endif

  wAux = (int32_t) (pHandle->hC1) * hAux;
  wIalfa_est_Next = pHandle->wIalfaEst - wAux;

  wAux = (int32_t) (pHandle->hC2) * hIalfa_err;
  wIalfa_est_Next += wAux;

  wAux = (int32_t) (pHandle->hC5) * hValfa;
  wIalfa_est_Next += wAux;

  wAux = (int32_t)  (pHandle->hC3) * hAux_Alfa;
  wIalfa_est_Next -= wAux;

  wAux = (int32_t)(pHandle->hC4) * hIalfa_err;
  wBemf_alfa_est_Next = pHandle->wBemfalfaEst + wAux;

#ifdef FULL_MISRA_C_COMPLIANCY
  wAux = (int32_t) hAux_Beta / pHandle->hF3;
#else
  wAux = (int32_t) hAux_Beta >> pHandle->hF3Pow2;
#endif

  wAux = wAux * pHandle->hC6;
  wAux = pHandle->Super.hElSpeedDpp * wAux;
  wBemf_alfa_est_Next += wAux;

  /*beta axes observer*/
#ifdef FULL_MISRA_C_COMPLIANCY
  hAux = (int16_t) (pHandle->wIbetaEst / pHandle->hF1);
#else
  hAux = (int16_t) (pHandle->wIbetaEst >> pHandle->hF1Log);
#endif

  wAux = (int32_t)  (pHandle->hC1) * hAux;
  wIbeta_est_Next = pHandle->wIbetaEst - wAux;

  wAux = (int32_t) (pHandle->hC2) * hIbeta_err;
  wIbeta_est_Next += wAux;

  wAux = (int32_t) (pHandle->hC5) * hVbeta;
  wIbeta_est_Next += wAux;

  wAux = (int32_t)  (pHandle->hC3) * hAux_Beta;
  wIbeta_est_Next -= wAux;

  wAux = (int32_t)(pHandle->hC4) * hIbeta_err;
  wBemf_beta_est_Next = pHandle->wBemfbetaEst + wAux;

#ifdef FULL_MISRA_C_COMPLIANCY
  wAux = (int32_t)hAux_Alfa / pHandle->hF3;
#else
  wAux = (int32_t) hAux_Alfa >> pHandle->hF3Pow2;
#endif

  wAux = wAux * pHandle->hC6;
  wAux = pHandle->Super.hElSpeedDpp * wAux;
  wBemf_beta_est_Next -= wAux;

  /*Calls the PLL blockset*/
  pHandle->hBemfalfaEst = hAux_Alfa;
  pHandle->hBemfbetaEst = hAux_Beta;

  if (pHandle->hForcedDirection ==0)
  {
    /* we are in auxiliary mode, then rely on the speed detected */
    if(pHandle->Super.hElSpeedDpp >= 0)
    {
      wDirection = 1;
    }
    else
    {
      wDirection = -1;
    }
  }
  else
  {
    /* we are in main sensor mode, use a forced direction */
    wDirection = pHandle->hForcedDirection;
  }

  hAux_Alfa = (int16_t)(hAux_Alfa * wDirection );
  hAux_Beta = (int16_t)(hAux_Beta * wDirection );

  hRotor_Speed = BemfObs_ExecutePLL(pHandle, hAux_Alfa, -hAux_Beta);
  pHandle->Super.InstantaneousElSpeedDpp = hRotor_Speed;

  BemfObs_Store_Rotor_Speed(pHandle, hRotor_Speed);

  pHandle->Super.hElAngle += hRotor_Speed;

  /*storing previous values of currents and bemfs*/
  pHandle->wIalfaEst = wIalfa_est_Next;
  pHandle->wBemfalfaEst = wBemf_alfa_est_Next;

  pHandle->wIbetaEst = wIbeta_est_Next;
  pHandle->wBemfbetaEst = wBemf_beta_est_Next;

  return (pHandle->Super.hElAngle);
}


bool BemfObsPll_CalcAvrgMecSpeedUnit(BemfObserverPllHandle_t * pHandle, int16_t * pMecSpeedUnit)
{
  int32_t wAvrSpeed_dpp = (int32_t)0;
  int32_t wError, wAux, wAvrSquareSpeed, wAvrQuadraticError = 0;
  uint8_t i, bSpeedBufferSizeUnit = pHandle->bSpeedBufferSizeUnit;
  int32_t wObsBemf, wEstBemf;
  int32_t wObsBemfSq = 0, wEstBemfSq = 0;
  int32_t wEstBemfSqLo;

  bool bIs_Speed_Reliable = false, bAux = false;
  bool bIs_Bemf_Consistent = false;

  for (i = 0u; i < bSpeedBufferSizeUnit; i++)
  {
    wAvrSpeed_dpp += (int32_t)(pHandle->SpeedBuffer[i]);
  }

  wAvrSpeed_dpp = wAvrSpeed_dpp / (int16_t)bSpeedBufferSizeUnit;

  for (i = 0u; i < bSpeedBufferSizeUnit; i++)
  {
    wError = (int32_t)(pHandle->SpeedBuffer[i]) - wAvrSpeed_dpp;
    wError = (wError * wError);
    wAvrQuadraticError += wError;
  }

  /*It computes the measurement variance   */
  wAvrQuadraticError = wAvrQuadraticError / (int16_t)bSpeedBufferSizeUnit;

  /* The maximum variance acceptable is here calculated as a function of average
     speed                                                                    */
  wAvrSquareSpeed = wAvrSpeed_dpp * wAvrSpeed_dpp;
  wAvrSquareSpeed = (wAvrSquareSpeed * (int32_t)(pHandle->hVariancePercentage)) / (int16_t)128;

  if (wAvrQuadraticError < wAvrSquareSpeed)
  {
    bIs_Speed_Reliable = true;
  }

  /*Computation of Mechanical speed Unit*/
  wAux = wAvrSpeed_dpp * (int32_t)(pHandle->Super.hMeasurementFrequency);
  wAux = wAux * (int32_t) (pHandle->Super.bSpeedUnit);
  wAux = wAux / (int32_t)(pHandle->Super.DPPConvFactor);
  wAux = wAux / (int16_t)(pHandle->Super.bElToMecRatio);

  *pMecSpeedUnit = (int16_t)wAux;
  pHandle->Super.hAvrMecSpeedUnit = (int16_t)wAux;

  pHandle->bIsSpeedReliable = bIs_Speed_Reliable;

  /*Bemf Consistency Check algorithm*/
  if (pHandle->bEnableDualCheck == true) /*do algorithm if it's enabled*/
  {
    wAux = (wAux < 0 ? (-wAux) : (wAux)); /* wAux abs value   */
    if (wAux < (int32_t)(pHandle->hMaxAppPositiveMecSpeedUnit))
    {
      /*Computation of Observed back-emf*/
      wObsBemf = (int32_t)(pHandle->hBemfalfaEst);
      wObsBemfSq = wObsBemf * wObsBemf;
      wObsBemf = (int32_t)(pHandle->hBemfbetaEst);
      wObsBemfSq += wObsBemf * wObsBemf;

      /*Computation of Estimated back-emf*/
      wEstBemf = (wAux * 32767) / (int16_t)(pHandle->Super.hMaxReliableMecSpeedUnit);
      wEstBemfSq = (wEstBemf * (int32_t)(pHandle->bBemfConsistencyGain)) / 64;
      wEstBemfSq *= wEstBemf;

      /*Computation of threshold*/
      wEstBemfSqLo = wEstBemfSq -
                     (wEstBemfSq / 64) * (int32_t)(pHandle->bBemfConsistencyCheck);

      /*Check*/
      if (wObsBemfSq > wEstBemfSqLo)
      {
        bIs_Bemf_Consistent = true;
      }
    }

    pHandle->bIsBemfConsistent = bIs_Bemf_Consistent;
    pHandle->wObsBemfLevel = wObsBemfSq;
    pHandle->wEstBemfLevel = wEstBemfSq;
  }
  else
  {
    bIs_Bemf_Consistent = true;
  }

  /*Decision making*/
  if (pHandle->bIsAlgorithmConverged == false)
  {
    bAux = SpdPosFdbk_CalcReliability (&pHandle->Super, pMecSpeedUnit);
  }
  else
  {
    if ((pHandle->bIsSpeedReliable == false) || (bIs_Bemf_Consistent == false))
    {
      pHandle->bReliabilityCounter++;
      if (pHandle->bReliabilityCounter >= pHandle->bReliabilityHysteresys)
      {
        pHandle->bReliabilityCounter = 0u;
        pHandle->Super.bSpeedErrorNumber = pHandle->Super.bMaximumSpeedErrorsNumber;
        bAux = false;
      }
      else
      {
        bAux = SpdPosFdbk_CalcReliability (&pHandle->Super, pMecSpeedUnit);
      }
    }
    else
    {
      pHandle->bReliabilityCounter = 0u;
      bAux = SpdPosFdbk_CalcReliability (&pHandle->Super, pMecSpeedUnit);
    }
  }
  return (bAux);
}


void BemfObsPll_CalcAvrgElSpeedDpp(BemfObserverPllHandle_t * pHandle)
{
  int16_t hIndexNew = (int16_t)pHandle->bSpeedBufferIndex;
  int16_t hIndexOld;
  int16_t hIndexOldTemp;
  int32_t wSum = pHandle->wDppBufferSum;
  int32_t wAvrSpeed_dpp;
  int16_t hSpeedBufferSizedpp = (int16_t)(pHandle->bSpeedBufferSizeDpp);
  int16_t hSpeedBufferSizeUnit = (int16_t)(pHandle->bSpeedBufferSizeUnit);
  int16_t hBufferSizeDiff;

  hBufferSizeDiff = hSpeedBufferSizeUnit - hSpeedBufferSizedpp;

  if (hBufferSizeDiff == 0)
  {
    wSum = wSum + pHandle->SpeedBuffer[hIndexNew] -
           pHandle->hSpeedBufferOldestEl;
  }
  else
  {
    hIndexOldTemp = hIndexNew + hBufferSizeDiff;

    if (hIndexOldTemp >= hSpeedBufferSizeUnit)
    {
      hIndexOld = hIndexOldTemp - hSpeedBufferSizeUnit;
    }
    else
    {
      hIndexOld = hIndexOldTemp;
    }

    wSum = wSum + pHandle->SpeedBuffer[hIndexNew] -
           pHandle->SpeedBuffer[hIndexOld];
  }

#ifdef FULL_MISRA_C_COMPLIANCY
  wAvrSpeed_dpp = wSum / hSpeedBufferSizeDpp;
#else
  wAvrSpeed_dpp = wSum >> pHandle->hSpeedBufferSizeDppLog;
#endif

  pHandle->Super.hElSpeedDpp = (int16_t)wAvrSpeed_dpp;
  pHandle->wDppBufferSum = wSum;
}


void BemfObsPll_Clear(BemfObserverPllHandle_t * pHandle)
{
  pHandle->wIalfaEst = (int32_t)0;
  pHandle->wIbetaEst = (int32_t)0;
  pHandle->wBemfalfaEst = (int32_t)0;
  pHandle->wBemfbetaEst = (int32_t)0;
  pHandle->Super.hElAngle = (int16_t)0;
  pHandle->Super.hElSpeedDpp = (int16_t)0;
  pHandle->bConsistencyCounter = 0u;
  pHandle->bReliabilityCounter = 0u;
  pHandle->bIsAlgorithmConverged = false;
  pHandle->bIsBemfConsistent = false;
  pHandle->wObsBemfLevel = (int32_t)0;
  pHandle->wEstBemfLevel = (int32_t)0;
  pHandle->wDppBufferSum = (int32_t)0;
  pHandle->bForceConvergency = false;
  pHandle->bForceConvergency2 = false;

  BemfObs_InitSpeedBuffer(pHandle);
  PID_SetIntegralTerm(& pHandle->PIRegulator, (int32_t)0);
}

/**
  * @brief  It stores in estimated speed FIFO latest calculated value of motor
  *         speed
  * @param  pHandle: handler of the current instance of the BemfObserver component
  * @retval none
  */
inline static void BemfObs_Store_Rotor_Speed(BemfObserverPllHandle_t * pHandle, int16_t hRotor_Speed)
{

  uint8_t bBuffer_index = pHandle->bSpeedBufferIndex;

  bBuffer_index++;
  if (bBuffer_index == pHandle->bSpeedBufferSizeUnit)
  {
    bBuffer_index = 0u;
  }

  pHandle->hSpeedBufferOldestEl = pHandle->SpeedBuffer[bBuffer_index];

  pHandle->SpeedBuffer[bBuffer_index] = hRotor_Speed;
  pHandle->bSpeedBufferIndex = bBuffer_index;
}

/**
  * @brief  It executes PLL algorithm for rotor position extraction from B-emf
  *         alpha and beta
  * @param  pHandle: handler of the current instance of the BemfObserver component
  *         hBemfalfaEst estimated Bemf alpha on the stator reference frame
  *         hBemfbetaEst estimated Bemf beta on the stator reference frame
  * @retval none
  */
inline static int16_t BemfObs_ExecutePLL(BemfObserverPllHandle_t * pHandle, int16_t hBemfalfaEst, int16_t
                               hBemfbetaEst)
{
  int32_t wAlfa_Sin_tmp, wBeta_Cos_tmp;
  int16_t hOutput;
  TrigComponents_t Local_Components;
  int16_t hAux1, hAux2;

  Local_Components = MCMath_TrigFunctions(pHandle->Super.hElAngle);

  /* Alfa & Beta BEMF multiplied by Cos & Sin*/
  wAlfa_Sin_tmp = (int32_t)(hBemfalfaEst) * (int32_t)Local_Components.hSin;
  wBeta_Cos_tmp = (int32_t)(hBemfbetaEst) * (int32_t)Local_Components.hCos;

#ifdef FULL_MISRA_C_COMPLIANCY
  hAux1 = (int16_t)(wBeta_Cos_tmp / 32768);
#else
  hAux1 = (int16_t)(wBeta_Cos_tmp >> 15);
#endif

#ifdef FULL_MISRA_C_COMPLIANCY
  hAux2 = (int16_t)(wAlfa_Sin_tmp / 32768);
#else
  hAux2 = (int16_t)(wAlfa_Sin_tmp >> 15);
#endif

  /* Speed PI regulator */
  hOutput = PI_Controller(& pHandle->PIRegulator, (int32_t)(hAux1) - hAux2);

  return (hOutput);
}

/**
  * @brief  It clears the estimated speed buffer
  * @param  pHandle: handler of the current instance of the BemfObserver component
  * @retval none
  */
static void BemfObs_InitSpeedBuffer(BemfObserverPllHandle_t * pHandle)
{
  uint8_t b_i;
  uint8_t bSpeedBufferSize = pHandle->bSpeedBufferSizeUnit;

  /*init speed buffer*/
  for (b_i = 0u; b_i < bSpeedBufferSize; b_i++)
  {
    pHandle->SpeedBuffer[b_i] = (int16_t)0;
  }
  pHandle->bSpeedBufferIndex = 0u;
  pHandle->hSpeedBufferOldestEl = (int16_t)0;

  return;
}


bool BemfObsPll_IsObserverConverged(BemfObserverPllHandle_t * pHandle, int16_t hForcedMecSpeedUnit)
{
  int16_t hEstimatedSpeedUnit, hUpperThreshold, hLowerThreshold;
  int32_t wAux;
  bool bAux = false;
  int32_t wtemp;
  
  if (pHandle->bForceConvergency2 == true)
  {
    hForcedMecSpeedUnit = pHandle->Super.hAvrMecSpeedUnit;
  }

  if (pHandle->bForceConvergency == true)
  {
    bAux = true;
    pHandle->bIsAlgorithmConverged = true;
    pHandle->Super.bSpeedErrorNumber = 0u;
  }
  else
  {
    hEstimatedSpeedUnit = pHandle->Super.hAvrMecSpeedUnit;

    wtemp = (int32_t)hEstimatedSpeedUnit * (int32_t)hForcedMecSpeedUnit;

    if (wtemp > 0)
    {
      if (hEstimatedSpeedUnit < 0)
      {
        hEstimatedSpeedUnit = -hEstimatedSpeedUnit;
      }

      if (hForcedMecSpeedUnit < 0)
      {
        hForcedMecSpeedUnit = -hForcedMecSpeedUnit;
      }
      wAux = (int32_t) (hForcedMecSpeedUnit) * (int16_t)pHandle->bSpeedValidationBandHigh;
      hUpperThreshold = (int16_t)(wAux / (int32_t)16);

      wAux = (int32_t) (hForcedMecSpeedUnit) * (int16_t)pHandle->bSpeedValidationBandLow;
      hLowerThreshold = (int16_t)(wAux / (int32_t)16);

      /* If the variance of the estimated speed is low enough...*/
      if (pHandle->bIsSpeedReliable == true)
      {
        if ((uint16_t)hEstimatedSpeedUnit > pHandle->hMinStartUpValidSpeed)
        {
          /*...and the estimated value is quite close to the expected value... */
          if (hEstimatedSpeedUnit >= hLowerThreshold)
          {
            if (hEstimatedSpeedUnit <= hUpperThreshold)
            {
              pHandle->bConsistencyCounter++;

              /*... for hConsistencyThreshold consecutive times... */
              if (pHandle->bConsistencyCounter >=
                   pHandle->bStartUpConsistThreshold)
              {

                /* the algorithm converged.*/
                bAux = true;
                pHandle->bIsAlgorithmConverged = true;
                pHandle->Super.bSpeedErrorNumber = 0u;
              }
            }
            else
            {
              pHandle->bConsistencyCounter = 0u;
            }
          }
          else
          {
            pHandle->bConsistencyCounter = 0u;
          }
        }
        else
        {
          pHandle->bConsistencyCounter = 0u;
        }
      }
      else
      {
        pHandle->bConsistencyCounter = 0u;
      }
    }
  }

  return (bAux);
}


AlphaBeta_t BemfObsPll_GetEstimatedBemf(BemfObserverPllHandle_t * pHandle)
{
  AlphaBeta_t Vaux;
  Vaux.alpha = pHandle->hBemfalfaEst;
  Vaux.beta = pHandle->hBemfbetaEst;
  return (Vaux);
}


AlphaBeta_t BemfObsPll_GetEstimatedCurrent(BemfObserverPllHandle_t * pHandle)
{
  AlphaBeta_t Iaux;

#ifdef FULL_MISRA_C_COMPLIANCY
  Iaux.alpha = (int16_t)(pHandle->wIalfaEst / (pHandle->hF1));
#else
  Iaux.alpha = (int16_t)(pHandle->wIalfaEst >> pHandle->hF1Log);
#endif

#ifdef FULL_MISRA_C_COMPLIANCY
  Iaux.beta = (int16_t)(pHandle->wIbetaEst / (pHandle->hF1));
#else
  Iaux.beta = (int16_t)(pHandle->wIbetaEst >> pHandle->hF1Log);
#endif

  return (Iaux);
}


void BemfObsPll_GetObserverGains(BemfObserverPllHandle_t * pHandle, int16_t * phC2, int16_t * phC4)
{
  *phC2 = pHandle->hC2;
  *phC4 = pHandle->hC4;
}



void BemfObsPll_SetObserverGains(BemfObserverPllHandle_t * pHandle, int16_t hhC1, int16_t hhC2)
{

  pHandle->hC2 = hhC1;
  pHandle->hC4 = hhC2;
}


void BemfObs_GetPLLGains(BemfObserverPllHandle_t * pHandle, int16_t * pPgain, int16_t * pIgain)
{

  *pPgain = PID_GetKP(& pHandle->PIRegulator);
  *pIgain = PID_GetKI(& pHandle->PIRegulator);
}


void BemfObsPll_SetPLLGains(BemfObserverPllHandle_t * pHandle, int16_t hPgain, int16_t hIgain)
{
  PID_SetKP(& pHandle->PIRegulator, hPgain);
  PID_SetKI(& pHandle->PIRegulator, hIgain);
}


void BemfObsPll_OtfResetPLL(BemfObserver_t * pHandle)
{
  BemfObserverPllHandle_t * pHdl = (BemfObserverPllHandle_t *)pHandle->Super;
  PID_SetIntegralTerm(&pHdl->PIRegulator, (int32_t)0);
}


void BemfObsPll_ResetPLL(BemfObserverPllHandle_t * pHandle)
{
  PID_SetIntegralTerm(&pHandle->PIRegulator, (int32_t)0);
}


void BemfObsPll_SetPLL(BemfObserverPllHandle_t * pHandle, int16_t hElSpeedDpp, int16_t hElAngle)
{
  PID_SetIntegralTerm(& pHandle->PIRegulator,
                       (int32_t)hElSpeedDpp * (int32_t)PID_GetKIDivisor(& pHandle->PIRegulator));
  pHandle->Super.hElAngle = hElAngle;
}


int32_t BemfObsPll_GetEstimatedBemfLevel(BemfObserverPllHandle_t * pHandle)
{
  return (pHandle->wEstBemfLevel);
}


int32_t BemfObsPll_GetObservedBemfLevel(BemfObserverPllHandle_t * pHandle)
{
  return (pHandle->wObsBemfLevel);
}


void BemfObsPll_BemfConsistencyCheckSwitch(BemfObserverPllHandle_t * pHandle, bool bSel)
{
  pHandle->bEnableDualCheck = bSel;
}


bool BemfObsPll_IsBemfConsistent(BemfObserverPllHandle_t * pHandle)
{
  return (pHandle->bIsBemfConsistent);
}


bool BemfObsPll_IsVarianceTight(const BemfObserver_t * pHandle)
{
  BemfObserverPllHandle_t * pHdl = (BemfObserverPllHandle_t *)pHandle->Super;
  return (pHdl->bIsSpeedReliable);
}


void BemfObsPll_ForceConvergency1(BemfObserver_t * pHandle)
{
  BemfObserverPllHandle_t * pHdl = (BemfObserverPllHandle_t *)pHandle->Super;
  pHdl->bForceConvergency = true;
}


void BemfObsPll_ForceConvergency2(BemfObserver_t * pHandle)
{
  BemfObserverPllHandle_t * pHdl = (BemfObserverPllHandle_t *)pHandle->Super;
  pHdl->bForceConvergency2 = true;
}


void BemfObsPll_SetMinStartUpValidSpeedUnit(BemfObserverPllHandle_t * pHandle, uint16_t hMinStartUpValidSpeed)
{
  pHandle->hMinStartUpValidSpeed = hMinStartUpValidSpeed;
}


void BemfObsPll_SetDirection(BemfObserverPllHandle_t * pHandle, uint8_t direction)
{
  pHandle->hForcedDirection = direction;
}

