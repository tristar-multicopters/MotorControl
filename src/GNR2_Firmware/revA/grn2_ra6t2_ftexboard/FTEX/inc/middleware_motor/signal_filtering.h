/**
  * @file    signal_filtering.h
  * @author  Sami Bouzid, FTEX inc
  * @brief   This module was intended to provide a general way to implement different kinds of first
  *           and second order digital filters. The concept is based on the biquad digital filter,
  *           which can be configured by 5 coefficients into any first or second order IIR filter.
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

// Current filter type
typedef enum
{
    NOT_CONFIGURED,
    RECURSIVE_AVERAGE,
    BUTTERWORTH_FO_LP,
} FilterType_t;

// Biquad filter coefficients
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
    int32_t hPastInputs[2];             /* Past inputs. hPastInputs[0] is the most recent */
    int32_t hPastOutputs[2];            /* Past outputs. hPastOutputs[0] is the most recent */
    SignalFilteringCoeffs_t Coeffs;     /* Biquad filter coefficients */

    const iirfa_instance_t * pIIRFAInstance;  /* Pointer to renesas IIR filter accelerator.
                                                If software computation is preferred, initialize this pointer to NULL. */

} SignalFilteringHandle_t;


/* Exported functions ------------------------------------------------------- */

/**
  * @brief  Initializes filter handle.
  * @param  pHandle: handler of the current instance of the SignalFiltering component
  * @retval none.
  */
void SignalFiltering_Init(SignalFilteringHandle_t * pHandle);

/**
  * @brief  Clear filter handle, while keeping current configuration.
  * @param  pHandle: handler of the current instance of the SignalFiltering component
  * @retval none.
  */
void SignalFiltering_Clear(SignalFilteringHandle_t * pHandle);

/**
  * @brief  Calculate filter output from provided input data (uint16).
            Must be called periodically at desired filter sampling time.
  * @param  pHandle: handler of the current instance of the SignalFiltering component
  * @param  hInputData: Input data in uint16_t
  * @retval Output data in uint16_t
  */
uint16_t SignalFiltering_CalcOutputU16(SignalFilteringHandle_t * pHandle, uint16_t hInputData);

/**
  * @brief  Calculate filter output from provided input data (int16).
            Must be called periodically at desired filter sampling time.
  * @param  pHandle: handler of the current instance of the SignalFiltering component
  * @param  hInputData: Input data in int16_t
  * @retval Output data in int16_t
  */
int16_t SignalFiltering_CalcOutputI16(SignalFilteringHandle_t * pHandle, int16_t hInputData);

/**
  * @brief  Configure recursive average filter.
            Discrete equation: y[n] = ( (A-1)*y[n-1] + x[n] )/A, where A is hCoeff
  * @param  pHandle: handler of the current instance of the SignalFiltering component
  * @param  hCoeff: Coefficient A to configure filter bandwidth. Higher A means slower response.
  * @retval none.
  */
void SignalFiltering_ConfigureRecursiveAverage(SignalFilteringHandle_t * pHandle, uint16_t hCoeff);

/**
  * @brief  Configure first order low pass butterworth filter.
            Discrete equation: y[n] = ( x[n]+x[n-1]-beta*y(n-1) )/alpha
  * @param  pHandle: handler of the current instance of the SignalFiltering component
  * @param  fAlpha: Coefficient alpha as float
  * @param  fBeta: Coefficient beta as float
  * @retval none.
  */
void SignalFiltering_ConfigureButterworthFOLP(SignalFilteringHandle_t * pHandle, float fAlpha, float fBeta);



#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __SIGNALFILTERING_H */
