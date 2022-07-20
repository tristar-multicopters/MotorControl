/**
  * @file    signal_filtering.c
  * @author  Sami Bouzid, FTEX inc
  * @brief   This file provides firmware functions that implement various type of signal filters.
  *
*/

/* Includes ------------------------------------------------------------------*/
#include "signal_filtering.h"

static void SignalFiltering_PerformTimestep(SignalFilteringHandle_t * pHandle, int16_t hInputData, int16_t hOutputData);


void SignalFiltering_Init(SignalFilteringHandle_t * pHandle)
{
    pHandle->FilterType = NOT_CONFIGURED;
    
    pHandle->Coeffs.a1 = 0;
    pHandle->Coeffs.a2 = 0;
    pHandle->Coeffs.b0 = 0;
    pHandle->Coeffs.b1 = 0;
    pHandle->Coeffs.b2 = 0;
    
    pHandle->hPastInputs[0] = 0;
    pHandle->hPastInputs[1] = 0;
    pHandle->hPastOutputs[0] = 0;
    pHandle->hPastOutputs[1] = 0;
}

int16_t SignalFiltering_CalcOutput(SignalFilteringHandle_t * pHandle, int16_t hInputData)
{
    int16_t hRetVal = 0;
    float fSum;
    float fTemp1, fTemp2, fTemp3, fTemp4, fTemp5;
    
    if (pHandle->FilterType != NOT_CONFIGURED)
    {
        fTemp1 = pHandle->Coeffs.b0*hInputData;
        fTemp2 = pHandle->Coeffs.b1*pHandle->hPastInputs[0];
        fTemp3 = pHandle->Coeffs.b2*pHandle->hPastInputs[1];
        fTemp4 = pHandle->Coeffs.a1*pHandle->hPastOutputs[0];
        fTemp5 = pHandle->Coeffs.a2*pHandle->hPastOutputs[1];
        
        fSum = (fTemp1 + fTemp2 + fTemp3 + fTemp4 + fTemp5);
        if (fSum > INT16_MAX)
        {
            fSum = INT16_MAX;
        }
        if (fSum < INT16_MIN)
        {
            fSum = INT16_MIN;
        }
        hRetVal = (int16_t) fSum;
        SignalFiltering_PerformTimestep(pHandle, hInputData, hRetVal);
    }
    
    return hRetVal;
}

void SignalFiltering_ConfigureRecursiveAverage(SignalFilteringHandle_t * pHandle, uint16_t hCoeff)
{
    pHandle->FilterType = RECURSIVE_AVERAGE;
    
    float fCoeff = (float)hCoeff;
    
    /* Biquad coefficients calculation */
    pHandle->Coeffs.a1 = 1 - 1/fCoeff;
    pHandle->Coeffs.a2 = 0;
    pHandle->Coeffs.b0 = 1/fCoeff;
    pHandle->Coeffs.b1 = 0;
    pHandle->Coeffs.b2 = 0;
    
    pHandle->hPastInputs[0] = 0;
    pHandle->hPastInputs[1] = 0;
    pHandle->hPastOutputs[0] = 0;
    pHandle->hPastOutputs[1] = 0;
}

void SignalFiltering_ConfigureButterworthFOLP(SignalFilteringHandle_t * pHandle, float fAlpha, float fBeta)
{
    pHandle->FilterType = BUTTERWORTH_FO_LP;
    
    /* Biquad coefficients calculation */
    pHandle->Coeffs.a1 = (float) (-fBeta/fAlpha);
    pHandle->Coeffs.a2 = 0;
    pHandle->Coeffs.b0 = (float) (1/fAlpha);
    pHandle->Coeffs.b1 = (float) (1/fAlpha);
    pHandle->Coeffs.b2 = 0;
    
    pHandle->hPastInputs[0] = 0;
    pHandle->hPastInputs[1] = 0;
    pHandle->hPastOutputs[0] = 0;
    pHandle->hPastOutputs[1] = 0;
}

static void SignalFiltering_PerformTimestep(SignalFilteringHandle_t * pHandle, int16_t hInputData, int16_t hOutputData)
{
    pHandle->hPastInputs[1] = pHandle->hPastInputs[0];
    pHandle->hPastInputs[0] = hInputData;
    pHandle->hPastOutputs[1] = pHandle->hPastOutputs[0];
    pHandle->hPastOutputs[0] = hOutputData;
}


