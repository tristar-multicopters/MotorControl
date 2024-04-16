/**
  ******************************************************************************
  * @file    pedal_torque_sensor.c
  * @author  FTEX Inc
  * @brief   This file defines the functions used in higher 
  *          level modules for pedal assist
  ******************************************************************************
*/

#include "pedal_torque_sensor.h"
#include "wheel.h"
#include "ASSERT_FTEX.h"
#include "vc_errors_management.h"



// ============================= Variables ================================ //
static uint16_t hSafeStartCounter = 0;
static bool PTSsensorStuck = false;

// ==================== Public function prototypes ======================== //


/**
    Pedal torque Sensor conversion Initialization
*/
void PedalTorqSensor_Init(PedalTorqSensorHandle_t * pHandle, Delay_Handle_t * pPTSstuckDelay, uint16_t maxTorque)
{    
    
    ASSERT(pHandle != NULL);
    
    //init pas max output
    pHandle->hParameters.PasMaxOutputTorque = maxTorque;
   
    pHandle->bSafeStart = false; 
        
    pHandle->pPTSstuckDelay = pPTSstuckDelay;
      
    ASSERT(pHandle->pPTSstuckDelay->DelayInitialized); /* Delay sohuld be initialized in the task to specify at which 
                                                          frequence the update delay function will be called */
    
    Delay_SetTime(pHandle->pPTSstuckDelay, 5, SEC); /* Set a 5 seconds delay to detect a stuck Pedal Torque Sensor */
    Delay_Reset(pHandle->pPTSstuckDelay);           /* Make sure the counter is reset */
    

    SignalFiltering_Init(&pHandle->TorqSensorFilter);
    SignalFiltering_ConfigureButterworthFOLP(&pHandle->TorqSensorFilter,
                                                pHandle->hParameters.fFilterAlpha,
                                                    pHandle->hParameters.fFilterBeta);  
    pHandle->bConvHandle = RegConvMng_RegisterRegConv(&pHandle->PTSRegConv);
    PedalTorqSensor_Clear(pHandle);
    
    PedalTorqSensor_ComputeSlopes(pHandle);
    
    pHandle->hParameters.hStartupOffsetMTSpeedRPM = Wheel_GetWheelRpmFromSpeed(pHandle->hParameters.hStartupOffsetMTSpeedKMH);

}


/**
    Pedal torque Sensor ADC hardware values clear
*/
void PedalTorqSensor_Clear(PedalTorqSensorHandle_t * pHandle)
{
    pHandle->hAvTorqueValue = 0u;
    pHandle->hAvADCValue = 0u;
}

/**
    Pedal torque Sensor ADC value calculation and filtering
*/
void PedalTorqSensor_CalcAvValue(PedalTorqSensorHandle_t * pHandle, uint8_t speed)
{
    uint32_t wAux;
    uint16_t hAux;
    uint16_t hBandwidth;
    
    //get the index of the bw to be used. this is speed dependent.
    uint8_t n = PedalTorqSensor_GetBwUsingSpeed(pHandle,speed);

    /* Use the Read conversion Manager for ADC read*/
    hAux = RegConvMng_ReadConv(pHandle->bConvHandle);
    pHandle->hInstTorque = hAux;
    /* Select the filter coefficient based on start or stop condition*/
    if (pHandle->hInstTorque > pHandle->hAvADCValue)
        hBandwidth = pHandle->hParameters.hLowPassFilterBW1[n];
    else
        hBandwidth = pHandle->hParameters.hLowPassFilterBW2[n];
    /* Check if the variable not exceeding the limit*/
    if (hAux == 0xFFFFu)
    {
        hAux = 0xFFFE;
    }
        
    wAux =  (uint32_t) (hBandwidth - 1u); // Affect Bandwidth to the output value
    wAux *= (uint32_t) (pHandle->hAvADCValue); // Multiply the Avrg value with the coefficient
    wAux += hAux;
    wAux /= ( uint32_t )( hBandwidth );// Devide the output value  with the coefficient for a new avrg value
    /* Affect the average value to the hAvADCValue */
    pHandle->hAvADCValue = (uint16_t) wAux;
    

    /* Compute torque sensor value (between 0 and 65535) */
    hAux = (pHandle->hAvADCValue > pHandle->hParameters.hOffsetPTS) ? 
    (pHandle->hAvADCValue - pHandle->hParameters.hOffsetPTS) : 0; //Substraction without overflow

    wAux = (uint32_t)(pHandle->hParameters.bSlopePTS * hAux);
    
    ASSERT(pHandle->hParameters.bDivisorPTS != 0);
    wAux /= pHandle->hParameters.bDivisorPTS;
    if (wAux > UINT16_MAX) 
    {    
        wAux = UINT16_MAX;
    }
    hAux = (uint16_t)wAux;

    pHandle->hAvTorqueValue = hAux;    

    /* Pedal Torque Sensor stuck on startup verification */
    if (!pHandle->bSafeStart) 
    {   
        uint16_t TorqueSens;
        uint32_t OffsetSafe;
        
        TorqueSens = hAux;
        
        // We include the startup threshold to check if throttle is detected
        // Be cause it is in % 
        OffsetSafe = (UINT16_MAX * pHandle->hParameters.hOffsetMTSafety)/PTS_PERCENTAGE;
        
        /* Pedal Torque Sensor is not Detected */
        if (TorqueSens < OffsetSafe) 
        { 
            hSafeStartCounter ++; 
            /* Launch Safe Start after a delay counter */
            if (hSafeStartCounter >= SAFE_TORQUE_COUNT_100MS) 
            {   
                pHandle->bSafeStart = true;
                /* Clear this error in case it was falsly flagged as stuck (user kept pedal pushing at max on boot) */
                VC_Errors_ClearError(PAS_BOOT_ERROR);
                Delay_Reset(pHandle->pPTSstuckDelay);
            }
            else
            {
                /* Push zero torque while no safe start */
                pHandle->hAvTorqueValue = 0; 
            }                     
        }
        /* Pedal Torque Sensor is detected */
        else     
        {
            hSafeStartCounter = 0;
            pHandle->hAvTorqueValue = 0; 

            if (!PTSsensorStuck)
            {
                /* Increase the counter for the error delay and check if the delay has been reached */
                if (Delay_Update(pHandle->pPTSstuckDelay)) 
                {
                    VC_Errors_RaiseError(PAS_BOOT_ERROR, HOLD_UNTIL_CLEARED); 
                    PTSsensorStuck = true;
                }
            }
        }    
    }
}


/**
    Pedal torque Sensor return ADC value
*/
uint16_t PedalTorqSensor_GetAvValue(PedalTorqSensorHandle_t * pHandle)
{
    return pHandle->hAvTorqueValue;
}

/**
    Pedal torque Sensor Reset ADC value
*/
void   PedalTorqSensor_ResetAvValue(PedalTorqSensorHandle_t * pHandle)
{
    pHandle->hAvTorqueValue = 0;
}

/**
    Pedal torque Sensor  Convert Torque sensor data to motor torque
*/
int16_t PedalTorqSensor_ToMotorTorque(PedalTorqSensorHandle_t * pHandle)
{
    int32_t tAux;
    
    /* Compute torque value (between -32768 and 32767) */
    tAux = (int32_t)(pHandle->hAvTorqueValue);
    if (tAux < 0)
    {
        tAux = 0;
    }
    
    tAux /= 2; // Convert from 0-65535 to 0-32767
    
    /* Use slope factor to translate the torque speed sensor to a motor torque */
    tAux = ((pHandle->hParameters.bSlopeMT) * tAux);
    
    ASSERT(pHandle->hParameters.bDivisorMT != 0);
    
    tAux /= pHandle->hParameters.bDivisorMT;
    
    /* Data limitation secure if there is any exceed */
    if (tAux > INT16_MAX)
    {
        tAux = INT16_MAX;
    }
    else if (tAux < INT16_MIN)
    {
        tAux = 0;
    }
    
    return (int16_t)tAux;
}

/**
    Return true if the Pedal Torque sensor is pressed (threshold is passed) 
  */
bool PedalTorqSensor_IsDetected (PedalTorqSensorHandle_t * pHandle) 
{
    ASSERT(pHandle != NULL);
    uint16_t hTorqueSens;
    hTorqueSens = PedalTorqSensor_GetAvValue(pHandle);
    if (hTorqueSens <= pHandle->hParameters.hOffsetPTS)
    {    
        return false;
    }    
    else
    {        
        return true;
    }   
}

/**
    Compute slope for torque sensor module
*/
void PedalTorqSensor_ComputeSlopes(PedalTorqSensorHandle_t * pHandle)
{
   float ADCSlope = 0;
   float TorqueSensor2Torq = 0;  
       
   ADCSlope = (pHandle->hParameters.hMax - pHandle->hParameters.hOffsetPTS); // Calculate the size of usable values received as an input
   
   ASSERT(ADCSlope >= 1);            
   ADCSlope =  UINT16_MAX/ADCSlope;   // Calculate the gain needed to scale that value to a 0-uint16(65535)
   ADCSlope *= PTS_SLOPE_FACTOR;      // Multiply by the factor to create the numerator of a fraction  
   
   pHandle->hParameters.bSlopePTS   = (uint16_t) round(ADCSlope);   // Save the numerator
   pHandle->hParameters.bDivisorPTS = PTS_SLOPE_FACTOR;             // and denominator
        
   TorqueSensor2Torq =  INT16_MAX; // Calculate the size of usable values received as an input
   
   ASSERT(TorqueSensor2Torq >= 1); 
   TorqueSensor2Torq =  pHandle->hParameters.PasMaxOutputTorque/TorqueSensor2Torq;    // Calculate the gain needed to scale that value to a 0-PasMaxOutputTorque
   TorqueSensor2Torq *= PTS_SLOPE_FACTOR;                                             // Multiply by the factor to create the numerator of a fraction    
   
   pHandle->hParameters.bSlopeMT   = (int16_t) round(TorqueSensor2Torq);    // Save the numerator
   pHandle->hParameters.bDivisorMT = PTS_SLOPE_FACTOR;                      // and denominator 
    
}

/**
    Select the index of the bw buffer based on the bike speed.
*/
uint8_t PedalTorqSensor_GetBwUsingSpeed(PedalTorqSensorHandle_t * pHandle, uint8_t speed)
{
    //get array size
    uint8_t size = sizeof(pHandle->hParameters.hFilterSpeed)/sizeof(pHandle->hParameters.hFilterSpeed[0]);
    
    //
    for (uint8_t n = 0; n < size; n++)
    {
        //edge case 1.
        if ((speed < pHandle->hParameters.hFilterSpeed[n]) && (n == 0))
        {
            return n;
        }
        
        //edge case 2.
        if ((n + 1) == (size))
        {
            return size;
        }
        
        //middle condtion.
        if ((speed >= pHandle->hParameters.hFilterSpeed[n]) && ((speed < pHandle->hParameters.hFilterSpeed[n + 1])))
        {
           return n + 1; 
        }
    }
    
    return 0;
}