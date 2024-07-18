/**
    * @file    foldback.c
    * @author  Jorge Andres Polo, FTEX
    * @author  Sami Bouzid, FTEX
    * @author  Ronak NemaDE, FTEX
    * @brief   This module implement a negative ramp to limit an input value based on another variable.
    *
*/
    
#include "foldback.h"
#include "ASSERT_FTEX.h"

//Number of foldbacks. If another foldback is added this number should be increased by 1
#define FOLDBACK_NUMBER     6

//Struct that holds the foldbacks
Foldback_Handle_t Foldbacks[FOLDBACK_NUMBER];


/**
  * @brief  Function for computing maximum output value based on a control variable
  * @param  pHandle: handler of the current instance of the Foldback component
  * @param  hControlVariable: Variable that define the output limit
  * @retval Computed output limit
  */
static int16_t Foldback_GetMaxOutput(uint8_t chosenFoldback, int16_t hControlVariable);


/**
  * Refer to function definition
  **/

void Foldback_Init(uint8_t chosenFoldback, Foldback_Handle_t FoldbackInit)
{
    Foldbacks[chosenFoldback].bEnableFoldback = FoldbackInit.bEnableFoldback;
    Foldbacks[chosenFoldback].hDefaultOutputLimitHigh = FoldbackInit.hDefaultOutputLimitHigh;
    Foldbacks[chosenFoldback].hDefaultOutputLimitLow = FoldbackInit.hDefaultOutputLimitLow;
    Foldbacks[chosenFoldback].hDecreasingEndValue = FoldbackInit.hDecreasingEndValue;
    Foldbacks[chosenFoldback].hDecreasingRange = FoldbackInit.hDecreasingRange;
    Foldbacks[chosenFoldback].FoldbackConfig = FoldbackInit.FoldbackConfig;
    
    Foldbacks[chosenFoldback].hMaxOutputLimitHigh = (int16_t) Foldbacks[chosenFoldback].hDefaultOutputLimitHigh;
    Foldbacks[chosenFoldback].hMaxOutputLimitLow = (int16_t) Foldbacks[chosenFoldback].hDefaultOutputLimitLow;
}

int16_t Foldback_GetMaxOutput(uint8_t chosenFoldback, int16_t hValue)
{
     
    int16_t hMaxOutput = 0;
    int32_t hStartValue;
    uint32_t wAux;
    
    if (!Foldbacks[chosenFoldback].bIsInverted)
    {
        hStartValue = Foldbacks[chosenFoldback].hDecreasingEndValue - (int16_t) Foldbacks[chosenFoldback].hDecreasingRange;
    }
    else
    {
        hStartValue = Foldbacks[chosenFoldback].hDecreasingEndValue + (int16_t) Foldbacks[chosenFoldback].hDecreasingRange;
    }

    if (hStartValue > Foldbacks[chosenFoldback].hDecreasingEndValue)
    {
        Foldbacks[chosenFoldback].bIsInverted = true;
    }
    else
    {
        Foldbacks[chosenFoldback].bIsInverted = false;
    }
        
    if (!Foldbacks[chosenFoldback].bIsInverted)
    {
        // If hValue is in the foldback range
        if(hValue > hStartValue && hValue < Foldbacks[chosenFoldback].hDecreasingEndValue)
        { 
            // Find the max torque value according to the foldback settings
            wAux = (uint32_t)( ( Foldbacks[chosenFoldback].hMaxOutputLimitHigh - Foldbacks[chosenFoldback].hMaxOutputLimitLow )*( Foldbacks[chosenFoldback].hDecreasingEndValue - hValue ) );
            wAux /= (uint32_t)(Foldbacks[chosenFoldback].hDecreasingEndValue - hStartValue);
            wAux += (uint32_t) Foldbacks[chosenFoldback].hMaxOutputLimitLow;
            hMaxOutput = (int16_t)wAux;
        }
        else if (hValue <= hStartValue)
        {
            hMaxOutput = Foldbacks[chosenFoldback].hMaxOutputLimitHigh;    // Pass higher threshold if input is lower than control range
        }
        else
        {
            hMaxOutput = Foldbacks[chosenFoldback].hMaxOutputLimitLow;     // Pass lower threshold if input is higher than control range
        }
    }
    else
    {
        // If hValue is in the foldback range
        if(hValue < hStartValue && hValue > Foldbacks[chosenFoldback].hDecreasingEndValue)
        { 
            // Find the max torque value according to the foldback settings
            wAux = (uint32_t)( ( Foldbacks[chosenFoldback].hMaxOutputLimitHigh - Foldbacks[chosenFoldback].hDefaultOutputLimitLow )*( hValue - Foldbacks[chosenFoldback].hDecreasingEndValue ) );                
            wAux /= (uint32_t)(hStartValue - Foldbacks[chosenFoldback].hDecreasingEndValue);
            wAux += (uint32_t) Foldbacks[chosenFoldback].hMaxOutputLimitLow;
            hMaxOutput = (int16_t)wAux;
        }
        else if (hValue >= hStartValue)
        {
            hMaxOutput =  Foldbacks[chosenFoldback].hMaxOutputLimitHigh;       // Pass higher threshold if input is lower than control range
        }
        else
        {
            hMaxOutput =  Foldbacks[chosenFoldback].hMaxOutputLimitLow;        // Pass lower threshold if input is higher than control range
        }
    }
    
    return hMaxOutput;
}


int16_t Foldback_ApplyFoldback(uint8_t chosenFoldback, int16_t hInputVariable, int16_t hValue)
{
     
    int16_t hMaxOutput,hOutputVariable = 0;
    
    if (Foldbacks[chosenFoldback].bEnableFoldback)
    {
        if(Foldbacks[chosenFoldback].FoldbackConfig == TRIM) // Test if foldback instance is used to trim the inputs.
        {
            hMaxOutput =  Foldback_GetMaxOutput(chosenFoldback, hValue);
            if (hInputVariable > hMaxOutput) // If input is greater than maximum possible value 
            {
                hOutputVariable = hMaxOutput; 
            }
            else if (hInputVariable < - hMaxOutput) // If input is smaller than maximum possible value               
            { 
                hOutputVariable = - hMaxOutput;
            }
            else 
            { 
                hOutputVariable = hInputVariable;
            } // set output as input
        }    
        else
        {
            // Foldback instance is used to calculate dynamic thresholds.
            hOutputVariable = Foldback_GetMaxOutput(chosenFoldback, hValue); 
        }
    }
    else
    {
        hOutputVariable = hInputVariable;
    }
    return hOutputVariable;
}

/**
 * Function for setting the start speed limitation value
 **/
void Foldback_SetDecreasingRange(uint8_t chosenFoldback, uint16_t hDecreasingRange)
{
     
    Foldbacks[chosenFoldback].hDecreasingRange = hDecreasingRange;
}

/**
 * Function for setting the end limitation speed value
 **/
void Foldback_SetDecreasingEndValue(uint8_t chosenFoldback, int32_t hDecreasingEndValue)
{
     
    Foldbacks[chosenFoldback].hDecreasingEndValue = hDecreasingEndValue;
}

/**
 * Function for getting the end limitation speed value
 **/
int32_t Foldback_GetDecreasingEndValue(uint8_t chosenFoldback)
{
     
    return Foldbacks[chosenFoldback].hDecreasingEndValue;
}

/**
 * Function for setting the end limitation speed value
 **/
void Foldback_SetDecreasingRangeEndValue(uint8_t chosenFoldback, int16_t hDecreasingRange)
{
     
    int16_t hInterval, hEndval;
    /* Add the Interval Value to the Start Value */
    hInterval = Foldbacks[chosenFoldback].hDecreasingInterval;
    hEndval = hInterval + hDecreasingRange;
    Foldbacks[chosenFoldback].hDecreasingEndValue = hEndval;
}

/**
 * Check function definition
 **/
void Foldback_EnableFoldback(uint8_t chosenFoldback)
{
     
    Foldbacks[chosenFoldback].bEnableFoldback = true;
}

/**
 * Check function definition
 **/
void Foldback_DisableFoldback(uint8_t chosenFoldback)
{
     
    Foldbacks[chosenFoldback].bEnableFoldback = false;
}

void Foldback_SetDefaultMaxOutputLimitHigh(uint8_t chosenFoldback, uint16_t maxValue)
{
    Foldbacks[chosenFoldback].hDefaultOutputLimitHigh = maxValue;
}

/**
 * Check function definition
 **/
void Foldback_SetMaxOutputLimitHigh(uint8_t chosenFoldback, int16_t hMaxValue)
{
     
    if(Foldbacks[chosenFoldback].FoldbackConfig == TRIM)
    {
        Foldbacks[chosenFoldback].hMaxOutputLimitHigh = hMaxValue; 
    }
    
}

/**
 * Check function definition
 **/
void Foldback_SetMaxOutputLimitLow(uint8_t chosenFoldback, int16_t hLimitValue)
{
     
    if(Foldbacks[chosenFoldback].FoldbackConfig == TRIM)
    {
        Foldbacks[chosenFoldback].hMaxOutputLimitLow = hLimitValue; 
    }
    
}

/**
 * Apply a low pass filter on the torque after for smooth 
 * accelearation with Pedal Assist
 **/

int16_t Foldback_ApplySlowStart(uint8_t chosenFoldback, int16_t hTorque)
{
     
    static uint32_t wTimeCounter;
    static int16_t hAverageTorque;
           int32_t wTemp = 0;
           int16_t hTorqueOut;

    hTorqueOut = hTorque;
    uint16_t hBandwidth;

    if(Foldbacks[chosenFoldback].bRefreshSlowStart)  
    { // Used to reset the counter, the average torque and the 
      // slow start refresh for PAS speed limitation 
       wTimeCounter = 0; 
       hAverageTorque = 0; 
       Foldbacks[chosenFoldback].bRefreshSlowStart = false; 
    }    

    if(Foldbacks[chosenFoldback].bEnableSlowStart) //Check if a slow start was requested or is in progress
    {    
        if(abs(hTorque) > abs(hAverageTorque))
        {
            hBandwidth = Foldbacks[chosenFoldback].hSlowStartBandwidth;
        }
        else
        {
            hBandwidth = Foldbacks[chosenFoldback].hSlowStopBandwidth;
        }
            
            wTemp =  (hBandwidth - 1u); //Apply a low pass filter to the torque
            wTemp *= hAverageTorque;
            wTemp += hTorque;
            wTemp /= hBandwidth;
            // Add a foladback filtering check for value under 1
            if (wTemp<MINIMUMVAL)
            {
                wTemp = MINIMUMVAL;
            }

            hAverageTorque =  (int16_t) wTemp ;
            hTorqueOut = hAverageTorque;            

        if (wTimeCounter > Foldbacks[chosenFoldback].wSlowStartTimeout)          // Timeout condition is there to make sure we cant get stuck in a slow start
        {
            wTimeCounter = 0;
            hAverageTorque = 0; 
            Foldbacks[chosenFoldback].bEnableSlowStart = false;        
        }              
    }
    else
    {
      wTimeCounter = 0;
      hAverageTorque = 0;        
    }        

    return hTorqueOut;
}

/**
 * Used to start or refresh a slow start
**/
void Foldback_EnableSlowStart(uint8_t chosenFoldback)
{
         
    Foldbacks[chosenFoldback].bRefreshSlowStart = true;
    Foldbacks[chosenFoldback].bEnableSlowStart = true;   
} 