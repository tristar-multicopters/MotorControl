/**
  * @file    signal_filtering.h
  * @author  Sami Bouzid, FTEX inc
  * @brief   This file provides firmware functions that implement various type of signal filters.
  *
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SIGNALFILTERING_H
#define __SIGNALFILTERING_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "r_iirfa.h"


typedef enum
{   
    NOT_CONFIGURED,
    RECURSIVE_AVERAGE,
    BUTTERWORTH_FO_LP,
} FilterType_t;

typedef struct
{
    float a1;
    float a2;
    float b0;
    float b1;
    float b2;
    
} SignalFilteringCoeffs_t;

typedef struct
{
    FilterType_t FilterType;            /* Current configuration of the filter */
    int16_t hPastInputs[2];             /* Past inputs. hPastInputs[0] is the most recent */
    int16_t hPastOutputs[2];            /* Past outputs. hPastOutputs[0] is the most recent */
    SignalFilteringCoeffs_t Coeffs;     /* Biquad filter coefficients */
    
    const iirfa_instance_t * pIIRFAInstance;  /* Pointer to renesas IIR filter accelerator. 
                                                If software computation is preferred, initialize this pointer to NULL. */
    
} SignalFilteringHandle_t;


/* Exported functions ------------------------------------------------------- */


void SignalFiltering_Init(SignalFilteringHandle_t * pHandle);

int16_t SignalFiltering_CalcOutput(SignalFilteringHandle_t * pHandle, int16_t hInputData);

void SignalFiltering_ConfigureRecursiveAverage(SignalFilteringHandle_t * pHandle, uint16_t hCoeff);

void SignalFiltering_ConfigureButterworthFOLP(SignalFilteringHandle_t * pHandle, float fAlpha, float fBeta);



#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __SIGNALFILTERING_H */

