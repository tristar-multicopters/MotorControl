#include "ASSERT_FTEX.h"
#include "Regen.h"
#include "motor_parameters.h"
#include "hardware_parameters.h"
#include <stdint.h>

RegenHandle_t RegenHandler;

/**
    * Initialize the Regen
 */
void RegenInit(void)
{
    #ifdef REGEN_ENABLE
    RegenHandler.bRegenEnabled = false;
    RegenHandler.hMaxCurrent = 0;
    RegenHandler.hMinCurrent = 0;
    RegenHandler.hRampPercent = REGEN_MIN_RAMP_PERCENT;
    RegenHandler.hMaxVoltage = 0;
    RegenHandler.hMinSpeed = REGEN_MIN_SPEED_RPM;
    RegenHandler.hRegenTorqueMax = 0;
    RegenHandler.bRegenLevelPercent = 0;
    RegenHandler.hRegenTorque = 0;
    RegenHandler.fRampCoEff = 0;
    RegenHandler.VoltageMax = UD_VOLTAGE_THRESHOLD_CONT_V;
    #endif
}

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
    RegenHandler.hRegenTorque = 0;      // reste regen torque
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
int16_t ApplyRegen(int16_t hMotorSpeed, uint16_t hBusVoltage)
{
    if (RegenHandler.bRegenEnabled == false)
    {
        return 0;                       // set regen torque to zero if regen is not enabled
    }
    
    if (RegenHandler.hMaxCurrent > 0)
    {
        return 0;                       // set regen torque to zero if the maximum current is positive
    }

    if (hBusVoltage > RegenHandler.hMaxVoltage)
    {
        return 0;                       // set regen torque to zero if the bus voltage is lower than the maximum voltage
    }

    if (hMotorSpeed < 0)                // set regen torque to zero if motor is moving backward
    {
        return 0;
    }
    
    RegenHandler.hRegenTorqueMax = (int16_t)((RegenHandler.bRegenLevelPercent * RegenHandler.hMaxCurrent * hBusVoltage)/(abs(hMotorSpeed)*PI_/30));

    if (abs(hMotorSpeed) > RegenHandler.hMinSpeed * 1.5)
    {    
        int16_t newRegenTorque = RegenHandler.hRegenTorque + (int16_t)(RegenHandler.hRegenTorqueMax * RegenHandler.fRampCoEff);
        if (newRegenTorque < RegenHandler.hRegenTorqueMax)
        {
            RegenHandler.hRegenTorque = newRegenTorque ;
        }
        else 
        {
            RegenHandler.hRegenTorque = RegenHandler.hRegenTorqueMax;
        }
    }
    else if (abs(hMotorSpeed) < RegenHandler.hMinSpeed)
    {
        int16_t newRegenTorque = RegenHandler.hRegenTorque - (int16_t)(RegenHandler.hRegenTorqueMax * RegenHandler.fRampCoEff);
        if (newRegenTorque < 0)
        {
            RegenHandler.hRegenTorque = newRegenTorque;
        }
        else 
        {
            RegenHandler.hRegenTorque = 0;
        }
    }

    return RegenHandler.hRegenTorque;
}

/**
    * Set the Regen level in percent
 */
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
    if (hMaxCurrent > 0)   
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
    return -RegenHandler.hMinCurrent;
}

/**
    * Set the Regen ramp in percent
 */
bool RegenSetRampPercent(uint16_t hRampPercent)
{
    if (hRampPercent < RegenHandler.hRampPercent)   
    {
        hRampPercent = RegenHandler.hRampPercent;
        return false;                       // retun false if the percent is greater than 1000ms
    }
    RegenHandler.hRampPercent = hRampPercent;
    RegenHandler.fRampCoEff = (float)hRampPercent / 100;
    return true;
}

/**
    * Get the Regen ramp percent
 */
uint16_t RegenGetRampPercent()
{
    return RegenHandler.hRampPercent;
}

/**
    * Set the Regen minimum speed
 */
bool RegenSetMaxVoltage(uint16_t hMaxVoltage)
{
    if (hMaxVoltage < RegenHandler.VoltageMax)   
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
    if (abs(hMinSpeed) < RegenHandler.hMinSpeed)   
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
