/**
    * @file    Regen.h
    * @brief   This file contains all definitions and functions prototypes for the
    *          MC Interface component. It allows controlling a motor and getting measurements using a simple API.
    * @author  Behnam S, FTEX inc
    */

#ifndef __REGEN_H
#define __REGEN_H

#include <stdint.h>
/* Includes ------------------------------------------------------------------*/

typedef struct
{
        uint8_t bRegenEnabled; /**< Flag indicating whether regen is enabled or disabled. */
        int16_t hMaxCurrent; /**< Maximum current for regen operation. */
        int16_t hMinCurrent; /**< Minimum current for regen operation. */
        uint16_t hRampPercent; /**< Ramp duration for regen operation in milliseconds. */
        uint16_t hMaxVoltage; /**< Maximum voltage for regen operation. */
        uint16_t hMinSpeed; /**< Minimum speed for regen operation. */
        int16_t hRegenTorqueMax; /**< Maximum torque for regen operation. */
        uint8_t bRegenLevelPercent; /**< Regen level as a percentage. */
        int16_t hRegenTorque; /**< Regen torque value. */
        float fRampCoEff; /**< Ramp coefficient value. */
} RegenHandle_t;

/**
    * @brief   Initialize regen operation.
    * @return none
 */
void RegenInit(void);

/**
    * @brief   Enable regen operation.
    * @return none
    */
void RegenSetEnabled();

/**
    * @brief  Disable regen operation.
    * @return none
    */
void RegenSetDisabled();

/**
    * @brief   Get the status of regen operation.
    * @return  True if regen is enabled, false otherwise.
    */
bool RegenGetEnabled();

/**
    * @brief   Apply regen operation to the motor speed and bus voltage.
    * @param   hMotorSpeed: The motor speed value.
    * @param   hBusVoltage: The bus voltage value.
    * @return  The updated motor speed after applying regen operation.
    */
int16_t ApplyRegen(int16_t hMotorSpeed, uint16_t hBusVoltage);

/**
    * @brief   Set the regen level as a percentage.
    * @param   bRegenLevelPercent: The regen level as a percentage.
    * @return  True if the regen level was set successfully, false otherwise.
    */
bool RegenSetLevelPercent(uint8_t bRegenLevelPercent);

/**
    * @brief   Set the maximum current for regen operation.
    * @param   hMaxCurrent: The maximum current value.
    * @return  True if the maximum current was set successfully, false otherwise.
    */
bool RegenSetMaxCurrent(int16_t hMaxCurrent);

/**
    * @brief   Get the maximum current for regen operation.
    * @return  The maximum current value.
    */
int16_t RegenGetMaxCurrent();

/**
    * @brief   Set the minimum current for regen operation.
    * @param   hMinCurrent: The minimum current value.
    * @return  True if the minimum current was set successfully, false otherwise.
    */
bool RegenSetMinCurrent(int16_t hMinCurrent);

/**
    * @brief   Get the minimum current for regen operation.
    * @return  The minimum current value.
    */
int16_t RegenGetMinCurrent();

/**
    * @brief   Set the ramp duration for regen operation.
    * @param   hRampDurationMs: The ramp duration value in milliseconds.
    * @return  True if the ramp duration was set successfully, false otherwise.
    */
bool RegenSetRampPercent(uint16_t hRampPercent);

/**
    * @brief   Get the ramp duration for regen operation.
    * @return  The ramp duration value in milliseconds.
    */
uint16_t RegenGetRampPercent();

/**
    * @brief   Set the maximum voltage for regen operation.
    * @param   hMaxVoltage: The maximum voltage value.
    * @return  True if the maximum voltage was set successfully, false otherwise.
    */
bool RegenSetMaxVoltage(uint16_t hMaxVoltage);

/**
    * @brief   Get the maximum voltage for regen operation.
    * @return  The maximum voltage value.
    */
uint16_t RegenGetMaxVoltage();

/**
    * @brief   Set the minimum speed for regen operation.
    * @param   hMinSpeed: The minimum speed value.
    * @return  True if the minimum speed was set successfully, false otherwise.
    */
bool RegenSetMinSpeed(int16_t hMinSpeed);

/**
    * @brief   Get the minimum speed for regen operation.
    * @return  The minimum speed value.
    */
uint16_t RegenGetMinSpeed();

#endif /* __REGEN_H */