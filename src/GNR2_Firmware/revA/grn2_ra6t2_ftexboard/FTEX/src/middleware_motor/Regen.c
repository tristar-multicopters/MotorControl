#include "ASSERT_FTEX.h"
#include "Regen.h"
#include "motor_parameters.h"
#include "hardware_parameters.h"
#include <stdint.h>

RegenHandle_t RegenHandler;

/**
    * Set the Regen enabled state
 */ 
void RegenSetEnabled()
{
    RegenHandler.bRegenEnabled = true;
}

/**
    * Set the Regen disabled state
 */
void RegenSetDisabled()
{
    RegenHandler.bRegenEnabled = false;
}

/**
    * Get the Regen enabled states
 */
bool RegenGetEnabled()
{
    return RegenHandler.bRegenEnabled;
}

/**
    * Apply the Regen
 */
uint16_t ApplyRegen(int16_t hMotorSpeed, uint16_t hBusVoltage)
{
    if (RegenHandler.bRegenEnabled == false)
    {
        return false;                       // return false if regen is not enabled
    }
    
    if (RegenHandler.hMaxCurrent > 0)
    {
        return false;                       // return false if the maximum current is positive
    }

    if ((abs)(hMotorSpeed) < RegenHandler.hMinSpeed)
    {
        return false;                       // return false if the speed is lower than the minimum speed
    }

    if (hBusVoltage > RegenHandler.hMaxVoltage)
    {
        return false;                       // return false if the bus voltage is lower than the maximum voltage
    }

    float RegenLevel = (float)(RegenHandler.bRegenLevelPercent/100);
    RegenHandler.hRegenTorque = (uint16_t)((RegenLevel * RegenHandler.hMaxCurrent * hBusVoltage * 100)/(abs(hMotorSpeed)*PI_/30));

    return RegenHandler.hRegenTorque;
}

bool RegenSetLevelPercent(uint8_t bRegenLevelPercent)
{
    if (bRegenLevelPercent > 100)
    {
        bRegenLevelPercent = 100;                       
    }
    RegenHandler.bRegenLevelPercent = bRegenLevelPercent;
    return true;
}

/**
    * Set the Regen maximum current
 */
bool RegenSetMaxCurrent(int16_t hMaxCurrent)
{
    if (hMaxCurrent > MAX_NEG_DC_CURRENT)   
    {
        return false;                       // retun false if the current is greater than the maximum negative DC current 
    }
    RegenHandler.hMaxCurrent = hMaxCurrent;
    return true;
}

/**
    * Get the Regen maximum current
 */
int16_t RegenGetMaxCurrent()
{
    return RegenHandler.hMaxCurrent;
}

/** 
    * Set the Regen minimum current
 */
bool RegenSetMinCurrent(int16_t hMinCurrent)
{
    if (hMinCurrent > 0)   
    {
        return false;                       // retun false if the current is positive
    }
    RegenHandler.hMinCurrent = hMinCurrent;
    return true;
}

/**
    * Get the Regen minimum current
 */
int16_t RegenGetMinCurrent()
{
    return RegenHandler.hMinCurrent;
}

/**
    * Set the Regen ramp duration
 */
bool RegenSetRampDurationMs(uint16_t hRampDurationMs)
{
    if (hRampDurationMs < REGEN_MIN_DURATION_MS)   
    {
        return false;                       // retun false if the duration is greater than 1000ms
    }
    RegenHandler.hRampDurationMs = hRampDurationMs;
    return true;
}

/**
    * Get the Regen ramp duration
 */
uint16_t RegenGetRampDurationMs()
{
    return RegenHandler.hRampDurationMs;
}

/**
    * Set the Regen minimum speed
 */
bool RegenSetMaxVoltage(uint16_t hMaxVoltage)
{
    if (hMaxVoltage < UD_VOLTAGE_THRESHOLD_CONT_V)   
    {
        return false;                       // retun false if lower than hardware minimum acceptable voltage
    }
    RegenHandler.hMaxVoltage = hMaxVoltage;
    return true;
}

/**
    * Get the Regen minimum speed
 */
uint16_t RegenGetMaxVoltage()
{
    return RegenHandler.hMaxVoltage;
}

/**
    * Set the Regen minimum speed
 */
bool RegenSetMinSpeed(int16_t hMinSpeed)
{
    if (abs(hMinSpeed) < REGEN_MIN_SPEED)   
    {
        return false;                       // retun false if lower than the minimum speed
    }
    RegenHandler.hMinSpeed = (uint16_t)abs(hMinSpeed);
    return true;
}

/**
    * Get the Regen minimum speed
 */
uint16_t RegenGetMinSpeed()
{
    return RegenHandler.hMinSpeed;
}
