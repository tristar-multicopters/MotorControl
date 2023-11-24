/**
  * @file    wheel.c
  * @brief   This module handles wheel speed calculation
  *
  */

#include "wheel.h"
#include "vc_constants.h"
#include "ASSERT_FTEX.h"
#include "vc_parameters.h"

// Internal variable used to store the wheel diameter

static uint8_t WheelDiameter = WHEEL_DIAMETER;

static bool InternalDiameterUpdate = false; // Indicates if the change of diameter of the wheel comes from the vehicle
                                            // Used to coordinate with a change from CAN
                                            
// Formula to convert RPM to KM using the wheel diameter
static float RpmToKmFormula = 0;
float GetRpmFormula(uint8_t diameterInInches);

// Initialize the wheel module
void Wheel_Init(void)
{
    RpmToKmFormula = GetRpmFormula(WheelDiameter);
}

// Return the wheel diameter in inches
uint8_t Wheel_GetWheelDiameter()
{
    return WheelDiameter;
}

// Set the wheel diameter to a value in inches
void Wheel_SetWheelDiameter(uint8_t diameterInInches)
{
    
    if (diameterInInches == WheelDiameter || diameterInInches == 0)
    {
        // no change
        return;
    }
    
    WheelDiameter = diameterInInches;
    
    // recompute the formula since diameter changed
    RpmToKmFormula = GetRpmFormula(WheelDiameter);
    
    InternalDiameterUpdate = true;
}

// Set the wheel diameter to a value in inches by external factor 
// we override the flag to false
void Wheel_ExternalSetWheelDiameter(uint8_t diameterInInches)
{
    Wheel_SetWheelDiameter(diameterInInches);   
    InternalDiameterUpdate = false;
}

// Check if we had an internal change of the wheel diameter
bool Wheel_CheckInternalUpdateFlag(void)
{
    return InternalDiameterUpdate;
}

// Clear the internal update flag after processing the change in CAN 
void Wheel_ClearInternalUpdateFlag(void)
{
    InternalDiameterUpdate = false;
}

// Compute the speed in km/h from the wheel rpm
uint16_t Wheel_GetSpeedFromWheelRpm(uint16_t wheelRpm)
{
    return (uint16_t)((float)wheelRpm * RpmToKmFormula);  
}

// Compute the wheel rpm from the speed in km/h
uint16_t Wheel_GetWheelRpmFromSpeed(uint16_t speed)
{
    return (uint16_t)((float)speed / RpmToKmFormula);  
}

// Internal utility function to compute the rpm to km formula
float GetRpmFormula(uint8_t diameterInInches)
{
    return FTEX_PI * diameterInInches * MINUTES_PER_HOUR / FTEX_KM_TO_INCH;
}

/**
  * @brief  Get the vehicle speed in kmh using the wheel speed sensor
  * @param  Handle of the wheel speed sensor
  * @retval Speed in km/h
  */
uint16_t Wheel_GetVehicleSpeedFromWSS(WheelSpeedSensorHandle_t * pHandle)
{
    ASSERT(pHandle!= NULL);

    // Get the RPM from the wheel speed sensor module
    uint16_t rpm = WheelSpdSensor_GetSpeedRPM(pHandle);
    
    // Convert the measurement in km/h;
    uint16_t speed = Wheel_GetSpeedFromWheelRpm(rpm);

    return speed;
}
