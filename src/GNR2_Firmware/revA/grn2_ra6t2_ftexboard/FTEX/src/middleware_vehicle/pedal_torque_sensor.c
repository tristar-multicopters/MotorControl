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
#include "vc_parameters.h"
#include "hal_data.h"
#include "board_hardware.h"

// ============================= Variables ================================ //
static uint16_t hSafeStartCounter = 0;
static bool PTSsensorStuck = false;

PedalTorqueSensorHandle_t pts =
{
    .PTSRegConv =
    {
        .hChannel = PEDAL_TORQUE_SENSOR_ANALOG_CHANNEL,
    },
    .TorqSensorFilter = 
    {
        .pIIRFAInstance = NULL, // NULL to apply software filtering, no hardware accelerator
    },
    .parameters =
    {
        .filterAlpha = PTS_FILTER_ALPHA,
        .filterBeta = PTS_FILTER_BETA,
        .idleSensorOffset = PTS_OFFSET_ADC2PTS,
        .offsetMTStartup = TORQUE_STARTUP_VALUE_THRESHOLD,  
        .startupOffsetSpeedKMH = PTS_OFFSET_STARTUP_SPEED_KMH,
        .offsetMT = PTS_OFFSET_PTS2TORQUE,
        .offsetMTSafety = PTS_OFFSET_PTS2TORQUE_SAFETY,
        .maxTorqueValue = PTS_MAX_PTSVALUE,
        .filterSpeed[0] = PTS_SPEED_FILTER_1,
        .filterSpeed[1] = PTS_SPEED_FILTER_2,
        .lowPassFilterBW1[0] = PTS_FILTER_BW1_1,
        .lowPassFilterBW2[0] = PTS_FILTER_BW2_1,
        .lowPassFilterBW1[1] = PTS_FILTER_BW1_2,
        .lowPassFilterBW2[1] = PTS_FILTER_BW2_2,
        .lowPassFilterBW1[2] = PTS_FILTER_BW1_3,
        .lowPassFilterBW2[2] = PTS_FILTER_BW2_3,
    }
};


// ==================== Public function prototypes ======================== //
/**
    Pedal torque sensor conversion initialization
*/

void PedalTorqueSensor_Init(Delay_Handle_t * pPTSstuckDelay)
{     
    pts.safeStart = false; 
    pts.pPTSstuckDelay = pPTSstuckDelay;
  
    ASSERT(pts.pPTSstuckDelay->DelayInitialized); /* Delay sohuld be initialized in the task to specify at which 
                                                          frequence the update delay function will be called */
    
    Delay_SetTime(pts.pPTSstuckDelay, 5, SEC); /* Set a 5 seconds delay to detect a stuck Pedal Torque Sensor */
    Delay_Reset(pts.pPTSstuckDelay);           /* Make sure the counter is reset */
    

    SignalFiltering_Init(&pts.TorqSensorFilter);
    SignalFiltering_ConfigureButterworthFOLP(&pts.TorqSensorFilter,
                                                pts.parameters.filterAlpha,
                                                    pts.parameters.filterBeta);  
    pts.convHandle = RegConvMng_RegisterRegConv(&pts.PTSRegConv);
    PedalTorqueSensor_Clear();
 
    pts.parameters.startupOffsetSpeedRPM = Wheel_GetWheelRpmFromSpeed(pts.parameters.startupOffsetSpeedKMH);
}

/**
    The rest of pedal torque sensor conversion initialization after MC initialization
*/
void PedalTorqueSensor_InitTorque(uint16_t maxTorque)
{
    //init pas max output
    pts.parameters.PasMaxOutputTorque = maxTorque;
    PedalTorqueSensor_ComputeSlopes();
   
}

/**
    Pedal torque Sensor ADC hardware values clear
*/
void PedalTorqueSensor_Clear()
{
    pts.avgTorqueValue = 0u;
    pts.avgADCValue = 0u;
}

/**
    Pedal torque Sensor ADC value calculation and filtering
*/
void PedalTorqueSensor_CalcAvValue(uint8_t speed)
{
    uint32_t wAux;
    uint16_t hAux;
    uint16_t hBandwidth;
    
    //get the index of the bw to be used. this is speed dependent.
    uint8_t n = PedalTorqueSensor_GetBwUsingSpeed(speed);

    /* Use the Read conversion Manager for ADC read*/
    hAux = RegConvMng_ReadConv(pts.convHandle);
    pts.instTorque = hAux;
    
    /* Select the filter coefficient based on start or stop condition*/
    if (pts.instTorque > pts.avgADCValue)
        hBandwidth = pts.parameters.lowPassFilterBW1[n];
    else
        hBandwidth = pts.parameters.lowPassFilterBW2[n];
    
    /* Check if the variable not exceeding the limit*/
    if (hAux == 0xFFFFu)
    {
        hAux = 0xFFFE;
    }
        
    wAux =  (uint32_t) (hBandwidth - 1u); // Affect Bandwidth to the output value
    wAux *= (uint32_t) (pts.avgADCValue); // Multiply the Avrg value with the coefficient
    wAux += hAux;
    wAux /= (uint32_t)(hBandwidth);// Devide the output value  with the coefficient for a new avrg value
    /* Affect the average value to the hAvADCValue */
    pts.avgADCValue = (uint16_t) wAux;
    

    /* Compute torque sensor value (between 0 and 65535) */
    hAux = (pts.avgADCValue > pts.parameters.idleSensorOffset) ? 
    (pts.avgADCValue - pts.parameters.idleSensorOffset) : 0; //Substraction without overflow

    wAux = (uint32_t)(pts.parameters.sensorSlope * hAux);
    
    ASSERT(pts.parameters.sensorADCScale != 0);
    wAux /= pts.parameters.sensorADCScale;
    if (wAux > UINT16_MAX) 
    {    
        wAux = UINT16_MAX;
    }
    hAux = (uint16_t)wAux;

    pts.avgTorqueValue = hAux;    

    /* Pedal Torque Sensor stuck on startup verification */
    if (!pts.safeStart) 
    {   
        uint16_t TorqueSens;
        uint32_t OffsetSafe;
        
        TorqueSens = hAux;
        
        // We include the startup threshold to check if throttle is detected
        // Be cause it is in % 
        OffsetSafe = (UINT16_MAX * pts.parameters.offsetMTSafety)/PTS_PERCENTAGE;
        
        /* Pedal Torque Sensor is not Detected */
        if (TorqueSens < OffsetSafe) 
        { 
            hSafeStartCounter ++; 
            /* Launch Safe Start after a delay counter */
            if (hSafeStartCounter >= SAFE_TORQUE_COUNT_100MS) 
            {   
                pts.safeStart = true;
                /* Clear this error in case it was falsly flagged as stuck (user kept pedal pushing at max on boot) */
                VC_Errors_ClearError(PAS_BOOT_ERROR);
                Delay_Reset(pts.pPTSstuckDelay);
            }
            else
            {
                /* Push zero torque while no safe start */
                pts.avgTorqueValue = 0; 
            }                     
        }
        /* Pedal Torque Sensor is detected */
        else     
        {
            hSafeStartCounter = 0;
            pts.avgTorqueValue = 0; 

            if (!PTSsensorStuck)
            {
                /* Increase the counter for the error delay and check if the delay has been reached */
                if (Delay_Update(pts.pPTSstuckDelay)) 
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
uint16_t PedalTorqueSensor_GetAvValue()
{
    return pts.avgTorqueValue;
}

/**
  @brief  Pedal torque Sensor Value in percentage
  @param  PedalTorqSensorHandle_t handle
  @return Torque value in percentage in uin8_t format
*/
uint8_t PedalTorqueSensor_GetPercentTorqueValue()
{
    //Calculate the percentage according to torque sensor offset and max value
    uint32_t temp = (pts.avgADCValue - pts.parameters.idleSensorOffset) * PTS_PERCENTAGE;
    uint16_t denominator = pts.parameters.maxTorqueValue - pts.parameters.idleSensorOffset;
    //Prevent zero div.
    if (denominator > 0)
    {
        temp = temp/denominator;
    }
    else
    {
        temp = 0;
    }
    
    //Max value is 100 percent or more if hMax is lower than 65535 and av value > hMax, trim value
    if (temp > PTS_PERCENTAGE)
        temp = PTS_PERCENTAGE;
    //it is safe to cast to 8-bit value is between 0 - 100 %
    return (uint8_t) temp;
}

/**
    Pedal torque Sensor Reset ADC value
*/
void   PedalTorqueSensor_ResetAvValue()
{
    pts.avgTorqueValue = 0;
}

/**
    Pedal torque Sensor  Convert Torque sensor data to motor torque
*/
int16_t PedalTorqueSensor_ToMotorTorque()
{
    int32_t tAux;
    
    /* Compute torque value (between -32768 and 32767) */
    tAux = (int32_t)(pts.avgTorqueValue);
    if (tAux < 0)
    {
        tAux = 0;
    }
    
    tAux /= 2; // Convert from 0-65535 to 0-32767
    
    /* Use slope factor to translate the torque speed sensor to a motor torque */
    tAux = ((pts.parameters.slopeMT) * tAux);
    
    ASSERT(pts.parameters.divisorMT != 0);
    
    tAux /= pts.parameters.divisorMT;
    
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
bool PedalTorqueSensor_IsDetected () 
{
    uint16_t hTorqueSens;
    hTorqueSens = PedalTorqueSensor_GetAvValue();
    if (hTorqueSens <= pts.parameters.idleSensorOffset)
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
void PedalTorqueSensor_ComputeSlopes()
{
   float ADCSlope = 0;
   float TorqueSensor2Torq = 0;  
       
   ADCSlope = (pts.parameters.maxTorqueValue - pts.parameters.idleSensorOffset); // Calculate the size of usable values received as an input
   
   ASSERT(ADCSlope >= 1);            
   ADCSlope =  UINT16_MAX/ADCSlope;   // Calculate the gain needed to scale that value to a 0-uint16(65535)
   ADCSlope *= PTS_SLOPE_FACTOR;      // Multiply by the factor to create the numerator of a fraction  
   
   pts.parameters.sensorSlope = (uint16_t) round(ADCSlope);   // Save the numerator
   pts.parameters.sensorADCScale = PTS_SLOPE_FACTOR;             // and denominator
        
   TorqueSensor2Torq =  INT16_MAX; // Calculate the size of usable values received as an input
   
   ASSERT(TorqueSensor2Torq >= 1); 
   TorqueSensor2Torq =  pts.parameters.PasMaxOutputTorque/TorqueSensor2Torq;    // Calculate the gain needed to scale that value to a 0-PasMaxOutputTorque
   TorqueSensor2Torq *= PTS_SLOPE_FACTOR;                                             // Multiply by the factor to create the numerator of a fraction    
   
   pts.parameters.slopeMT   = (int16_t) round(TorqueSensor2Torq);    // Save the numerator
   pts.parameters.divisorMT = PTS_SLOPE_FACTOR;                      // and denominator 
    
}

/**
    Select the index of the bw buffer based on the bike speed.
*/
uint8_t PedalTorqueSensor_GetBwUsingSpeed(uint8_t speed)
{
    //get array size
    uint8_t size = sizeof(pts.parameters.filterSpeed)/sizeof(pts.parameters.filterSpeed[0]);
    
    for (uint8_t n = 0; n < size; n++)
    {
        //edge case 1.
        if ((speed < pts.parameters.filterSpeed[n]) && (n == 0))
        {
            return n;
        }
        
        //edge case 2.
        if ((n + 1) == (size))
        {
            return size;
        }
        
        //middle condtion.
        if ((speed >= pts.parameters.filterSpeed[n]) && ((speed < pts.parameters.filterSpeed[n + 1])))
        {
           return n + 1; 
        }
    }   
    return 0;
}

/**
    Getter for Startup offset Speed RPM
*/
uint16_t PedalTorqueSensor_GetStartupOffsetMTSpeedRPM(void)
{
    return pts.parameters.startupOffsetSpeedRPM;
}

/**
    Getter for Startup Offset
*/
uint16_t PedalTorqueSensor_GetOffsetMTStartup(void)
{
    return pts.parameters.offsetMTStartup;
}

/**
    Getter for Running Offset
*/
uint16_t PedalTorqueSensor_GetOffsetMT(void)
{
    return pts.parameters.offsetMT;
}

/**
    Getter for Idle Sensor Offset
*/
uint16_t PedalTorqueSensor_GetSensorOffset(void)
{
    return pts.parameters.idleSensorOffset;
}

/**
    Getter for Max Torque Value
*/
uint16_t PedalTorqueSensor_GetMaxTorqueValue(void)
{
    return pts.parameters.maxTorqueValue;
}

/**
    Setter for Startup Offset Speed KM/H
*/
void PedalTorqueSensor_SetStartupOffsetMTSpeedKMH(uint16_t value)
{
    pts.parameters.startupOffsetSpeedKMH = value;
}

/**
    Setter for Startup MT Offset 
*/
void PedalTorqueSensor_SetOffsetMTStartup(uint16_t value)
{
    pts.parameters.offsetMTStartup = value;
}

/**
    Setter for Running MT Offset 
*/
void PedalTorqueSensor_SetOffsetMT(uint16_t value)
{
    pts.parameters.offsetMT = value;
}

/**
    Setter for Idle Sensor Offset
*/
void PedalTorqueSensor_SetSensorOffset(uint16_t value)
{
    pts.parameters.idleSensorOffset = value;
}

/**
    Setter for Max Torque Value 
*/
void PedalTorqueSensor_SetMaxTorqueValue(uint16_t value)
{
    pts.parameters.maxTorqueValue = value;
}

/**
    Setter for Speed Filter 
*/
void PedalTorqueSensor_SetFilterSpeed(uint8_t value, uint8_t index)
{
    pts.parameters.filterSpeed[index] = value;
}

/**
    Setter for First Butterworth Filter  
*/
void PedalTorqueSensor_SetBWFilter1(uint16_t value, uint8_t index)
{
    pts.parameters.lowPassFilterBW1[index] = value;
}

/**
    Setter for Second Butterworth Filter  
*/
void PedalTorqueSensor_SetBWFilter2(uint16_t value, uint8_t index)
{
    pts.parameters.lowPassFilterBW2[index] = value;
}
