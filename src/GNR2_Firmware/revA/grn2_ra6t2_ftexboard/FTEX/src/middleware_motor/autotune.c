/**
  * @file    autotune.c
  * @brief   This file provides firmware functions that implement the feature
  *          of the autotune.
*/

/* Includes ------------------------------------------------------------------*/
#include "autotune.h"
#include "mc_math.h"

/*
    Returns the phase current of the motor (in s16A unit).
    To call in order to compute Ia, Ib and Ic and store into handle.
*/
void Autotune_GetPhaseCurrents( PWMCurrFdbkHandle_t * pHandle, ab_t * Iab )
{
    Iab->a = pHandle->Ia;
    Iab->b = pHandle->Ib;
}
/*
    Manually set duty cycles.
*/
uint32_t Autotune_SetDuties(PWMCurrFdbkHandle_t * pHandle,
                               uint16_t hDutyA, uint16_t hDutyB, uint16_t hDutyC)
{
	pHandle->hCntPhA = (UINT16_MAX-hDutyA) * pHandle->hPWMperiod / 2 / UINT16_MAX;
	pHandle->hCntPhB= (UINT16_MAX-hDutyB) * pHandle->hPWMperiod / 2 / UINT16_MAX;
	pHandle->hCntPhC = (UINT16_MAX-hDutyC) * pHandle->hPWMperiod / 2 / UINT16_MAX;
	
	return (pHandle->pFctSetADCSampPointSectX(pHandle));
}

/*
    Compute Ia, Ib and Ic, as well as filtered version, and store into handle.
    Must be called once per control period.
*/
void Autotune_CalcPhaseCurrents(PWMCurrFdbkHandle_t * pHandle)
{
    ab_t Iab; // Only used as dummy for next function.
    int16_t IaFiltered, IbFiltered, IcFiltered;
    
    pHandle->pFctGetPhaseCurrents(pHandle, &Iab);

    IaFiltered = SignalFiltering_CalcOutputI16(&pHandle->IaFilter, pHandle->Ia);
    IbFiltered = SignalFiltering_CalcOutputI16(&pHandle->IbFilter, pHandle->Ib);
    IcFiltered = -IaFiltered - IbFiltered;
    
    pHandle->IaFilt = IaFiltered;
    pHandle->IbFilt = IbFiltered;
    pHandle->IcFilt = IcFiltered;
}

/*
    Returns the filtered phase current of the motor
*/
void Autotune_GetFiltPhaseCurrents( PWMCurrFdbkHandle_t * pHandle, ab_t * IabFilt )
{
    IabFilt->a = pHandle->IaFilt;
    IabFilt->b = pHandle->IbFilt;
}