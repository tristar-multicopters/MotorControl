/**
  * @file    signal_filtering.c
  * @author  Sami Bouzid, FTEX inc
  * @brief   This file provides firmware functions that implement various type of signal filters.
  *
*/

/* Includes ------------------------------------------------------------------*/
#include "signal_filtering.h"
#include "ASSERT_FTEX.h"

static void SignalFiltering_PerformTimestep(SignalFilteringHandle_t * pHandle, int16_t hInputData, int16_t hOutputData);

/*
    Initializes filter handle.
*/
void SignalFiltering_Init(SignalFilteringHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
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

/*
    Calculate filter output from provided input data. 
    Must be called periodically at desired filter sampling time.
*/
int16_t SignalFiltering_CalcOutput(SignalFilteringHandle_t * pHandle, int16_t hInputData)
{
    ASSERT(pHandle != NULL);
    int16_t hRetVal = 0;
    float fSum;
    float fTemp1, fTemp2, fTemp3, fTemp4, fTemp5;
    
    if (pHandle->FilterType != NOT_CONFIGURED) // Compute output only if filter is configured
    {
        if (pHandle->pIIRFAInstance == NULL)
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
        else
        {
            hRetVal = (int16_t) R_IIRFA_SingleFilter(pHandle->pIIRFAInstance->p_ctrl, (float) hInputData);
        }
    }
    
    return hRetVal;
}

/*
    Configure recursive average filter.
    Discrete equation: y[n] = ( (A-1)*y[n-1] + x[n] )/A, where A is hCoeff
*/
void SignalFiltering_ConfigureRecursiveAverage(SignalFilteringHandle_t * pHandle, uint16_t hCoeff)
{
    ASSERT(pHandle != NULL);
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
    
    if (pHandle->pIIRFAInstance == NULL)
    {
        
    }    
    else
    {
        /* Prepare biquad coefficients for hardware accelerator */
        iir_filter_coeffs_t gp_iirfa_filter_coeffs =
        {
            .a1 = pHandle->Coeffs.a1,
            .a2 = pHandle->Coeffs.a2,
            .b0 = pHandle->Coeffs.b0,
            .b1 = pHandle->Coeffs.b1,
            .b2 = pHandle->Coeffs.b2,
        };
        /* Biquad state data (clear to start) */
        iir_filter_state_t gp_iirfa_filter_state = {0};
        /* Hardware filter configuration */
        iir_filter_cfg_t g_iirfa_filter_cfg =
        {
            .p_filter_coeffs = &gp_iirfa_filter_coeffs, // Pointer to filter coefficient array
            .p_filter_state  = &gp_iirfa_filter_state,  // Pointer to filter state data array
            .stage_base      = 0,   // Which hardware biquad stage to start allocation from (0-31)
            .stage_num       = 1,                       // Number of stages to allocate
        };
        R_IIRFA_Configure(pHandle->pIIRFAInstance->p_ctrl, &g_iirfa_filter_cfg);
    }
}

/*
    Configure first order low pass butterworth filter.
    Discrete equation: y[n] = ( x[n]+x[n-1]-beta*y(n-1) )/alpha
*/
void SignalFiltering_ConfigureButterworthFOLP(SignalFilteringHandle_t * pHandle, float fAlpha, float fBeta)
{
    ASSERT(pHandle != NULL);
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
    
    if (pHandle->pIIRFAInstance == NULL)
    {
        
    }    
    else
    {
        /* Prepare biquad coefficients for hardware accelerator */
        iir_filter_coeffs_t gp_iirfa_filter_coeffs =
        {
            .a1 = pHandle->Coeffs.a1,
            .a2 = pHandle->Coeffs.a2,
            .b0 = pHandle->Coeffs.b0,
            .b1 = pHandle->Coeffs.b1,
            .b2 = pHandle->Coeffs.b2,
        };
        /* Biquad state data (clear to start) */
        iir_filter_state_t gp_iirfa_filter_state = {0};
        /* Hardware filter configuration */
        iir_filter_cfg_t g_iirfa_filter_cfg =
        {
            .p_filter_coeffs = &gp_iirfa_filter_coeffs, // Pointer to filter coefficient array
            .p_filter_state  = &gp_iirfa_filter_state,  // Pointer to filter state data array
            .stage_base      = 0,   // Which hardware biquad stage to start allocation from (0-31)
            .stage_num       = 1,                       // Number of stages to allocate
        };
        R_IIRFA_Configure(pHandle->pIIRFAInstance->p_ctrl, &g_iirfa_filter_cfg);
    }
}

/*
    Perform a timestep for calculating filter output, i.e. shifting past values.
*/
static void SignalFiltering_PerformTimestep(SignalFilteringHandle_t * pHandle, int16_t hInputData, int16_t hOutputData)
{
    ASSERT(pHandle != NULL);
    pHandle->hPastInputs[1] = pHandle->hPastInputs[0];
    pHandle->hPastInputs[0] = hInputData;
    pHandle->hPastOutputs[1] = pHandle->hPastOutputs[0];
    pHandle->hPastOutputs[0] = hOutputData;
}


