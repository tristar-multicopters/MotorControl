/**
  ******************************************************************************
  * @file    wheel_speed_sensor.c
  * @author  FTEX inc
  * @brief   This file defines the functions used in higher 
  *          level modules for wheel speed sensor
  ******************************************************************************
*/
#include "wheel_speed_sensor.h"
#include "ASSERT_FTEX.h"
#include "motor_signal_processing.h"
#include "vc_parameters.h"
#include "hal_data.h"
#include "board_hardware.h"

PulseFrequencyHandle_t wssPulseFrequency = 
{
    .TimerType = GPT_TIMER,
    .measuring = false,
    .wCaptureCount = 0,
    .wCaptureOverflow = 0,
    .PulseFreqParam = 
    {
        .PF_Timer = WHEEL_SPEED_SENSOR_TIMER_HANDLE_ADDRESS,
    }
};

WheelSpeedSensorHandle_t wss = 
{
    .pulseFrequency = &wssPulseFrequency,     
    .useMotorPulsePerRotation = WWS_USE_MOTOR_NBR_PER_ROTATION,
};


/**
    Wheel Speed Sensor Initialization
*/
void WSS_Init(uint8_t magnetsPerRotation)
{
    wss.pulsePerRotation = magnetsPerRotation;
    //get GPT timer information
    PulseFrequency_GetTimerInfo(wss.pulseFrequency);
}

/**
    Wheel Speed Sensor calculate periode value
*/
void WSS_CalculatePeriodValue(bool motorTempSensorMixed)
{
    //if motor doesn't have a mixed temperature/wheelspeed siganl,
    //get wheel speed period from the capture timer. 
    if (motorTempSensorMixed == false)
    {
        //verify if the timer is measuring.
        //if yes, initialise the flag to wait for the timer
        //to set the flag again and indicating it still working.
        if (wss.pulseFrequency->measuring == true)
        {
            wss.pulseFrequency->measuring = false;
            wss.timeout = 0;
        }
        else
        {
            //avoid variable overflow
            if (wss.timeout < WHEELSPEED_TIMEOUT_MS)
            {   
                //increment the timeout variable
                wss.timeout = wss.timeout + WHEELSPEED_TIME_INCREMENT_MS;
            }
        }
    
        // Detect if the timer did more than 2 overflows or timer is not running anymore, speed must to be set to zero.
        if ((wss.pulseFrequency->wCaptureOverflow > MAXNUMBER_OVERFLOW_WHEELSPEED) || ((wss.pulseFrequency->measuring == false) && (wss.timeout >= WHEELSPEED_TIMEOUT_MS)))
        {
            //Wheel speed is zero becacuse the timer overflowed
            //the time period determine the maximum time to the
            //wheel complete a full revolution.
            //this means that:
            wss.periodValue = 0; 
        }
        else
        {
            //update basic wheel speed information.
            PulseFrequency_ReadInputCapture (wss.pulseFrequency); 
            //get the total period time including when the wheel is on the magnet
            wss.periodValue = wss.pulseFrequency->wSecondPeriod / ((100 - (wss.timeOnOneMagnetPercent * wss.pulsePerRotation)) / 100);
        }
    }
    else
    {
        //get directly from the mixed signal, using analogic input.
        wss.periodValue = (float)getExtractedWheelSpeed();
    }
}

/**
    Wheel Speed Sensor Get periode value in seconds.
*/
float WSS_GetPeriodValue()
{
    return wss.periodValue;
}

/**
    Wheel Speed Sensor retrun RPM in tr/min
*/
uint16_t WSS_GetSpeedRPM()
{
    float wSpeed = WSS_GetPeriodValue();
    
    if (wSpeed != 0.0f) // Div by 0 protection
    {
        wss.speedRPM = (int32_t)(((float)(RPMCOEFF))/(wSpeed * wss.pulsePerRotation));
    }
    else
    {
        wss.speedRPM = 0;
    } 
    return (uint16_t)round(wss.speedRPM);
}

/**
    Update the pulse capture value coming from the ISR
*/
void WSS_UpdatePulseFromISR(uint32_t capture)
{
	PulseFrequency_IsrCallUpdate(wss.pulseFrequency, capture);
}

/**
    Update the overflow coming from ISR
*/
void WSS_OverflowPulseFromISR()
{
    PulseFrequency_ISROverflowUpdate(wss.pulseFrequency); 
}

/**
    Getter for WSS flag : useMotorPulsePerRotation
*/
bool WSS_GetUseMotorPulsePerRotation(void)
{
    return wss.useMotorPulsePerRotation;
}