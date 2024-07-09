/**
  ******************************************************************************
  * @file    pedal_speed_sensor.h
  * @author  FTEX inc
  * @brief   This file defines the handles, constantas and function prototypes
  *           used in higher level modules for pedal speed sensor
  *
    ******************************************************************************
*/

#ifndef __PEDAL_SPEED_SENSOR_H
#define __PEDAL_SPEED_SENSOR_H

// =============================== Includes ================================== //
#include "pulse_frequency.h"

// ================= Structure used to configure a pin ===================== //
typedef struct 
{    
    PulseFrequencyHandle_t * pPulseFrequency;   /* Pointer to pedal handle */
    uint32_t minPulsesStartup;                  //It has the minimum number of pulses, on startup, 
                                                //to detect PAS from cadence sensor
    uint16_t windowStartup;                     //Maximum time, on ms, to verify the Detected Number of pulses
                                                //from the cadence sensor when starts to pedal.
    uint32_t minPulsesRunning;                  //It has the minimum number of pulses, on running, 
                                                //to detect PAS from cadence sensor
    uint16_t windowRunning;                     //Maximum time, on ms, to verify the Detected Number of pulses
                                                //from the cadence sensor when on run mode.
    bool resetWindowsFlag;                      
    uint16_t numberOfPulses;                    /*Detected Number of pulses from the cadence signal*/
    uint16_t previousNumberOfPulses;            // Number of pulses since the last pedalling activity check
    uint16_t RPM;                               /* Pedal Speed sensor RPM calculated value */
    uint8_t magnetsCount;                       // Number of magnets or encoder resolution
} PedalSpeedSensorHandle_t;

// ==================== Public function prototypes ========================= //
/**
  @brief  Function to initialize pedal speed sensor handle
  @param  PedalSpeedSensorHandle_t handle
  @return None
*/
void PSS_Init(void);

/**
  @brief  Function to capture pedal speed sensor Periode
  @param  PedalSpeedSensorHandle_t handle
  @return None
*/
void PSS_ReadNumberOfPulses(void);

/**
  @brief  Function to Get the Pedal Speed Sensor Periode
  @param  PedalSpeedSensorHandle_t handle
  @return wPedal_Sensor_Read in unit32_t
*/
uint16_t PSS_GetNumberOfPulses(void);

/**
  @brief  Function to reset all variables used to hold the number of pulses measured
          by the AGT timer.
  @param  PedalSpeedSensorHandle_t handle
  @return None
*/
void PSS_ResetValue(void);

/**
  @brief  Set windows reset flag
  @param  PedalSpeedSensorHandle_t handle
  @return None
*/
void PSS_SetWindowsFlag(void);

/**
  @brief  Clear windows reset flag
  @param  PedalSpeedSensorHandle_t handle
  @return None
*/
void PSS_ClearWindowsFlag(void);

/**
  @brief  Get windows reset flag
  @param  PedalSpeedSensorHandle_t handle
  @return bool resetWindowsFlag
*/
bool PSS_GetWindowsFlag(void);

/**
  @brief  Function to return speed in rpm
  @param  PedalSpeedSensorHandle_t handle
  @retval Speed in rpm
*/
uint16_t PSS_GetSpeedRPM(void);

/**
  @brief Check if we have a new number of pulses detected 
         from the previous number of pulses detected 
  @param  PedalSpeedSensorHandle_t handle
  @return True if new pedal sensor pulses were detected since last function call
*/
bool PSS_NewPedalPulsesDetected(void);

/**
   Pedal Speed Sensor calculate RPM
*/
void PSS_CalculateRPM(void);

/**
 * @brief Getter for the pss startup window
 * @return Value of the startup window
 */
uint16_t PSS_GetStartupWindow(void);

/**
 * @brief Getter for the number of pulses
 *        required for startup activation
 * @return Number of required pulses count
 */
uint32_t PSS_GetStartupPulsesCount(void);

/**
 * @brief Getter for the pss running window
 * @return Value of the running window
 */
uint16_t PSS_GetRunningWindow(void);

/**
 * @brief Getter for the number of pulses
 *        required for running activation
 * @return Number of required pulses count
 */
uint32_t PSS_GetRunningPulsesCount(void);

/**
 * @brief Setter for the pss startup window
 * @param value : New updated value to set
 * @return None
 */
void PSS_SetStartupWindow(uint16_t value);

/**
 * @brief Setter for the number of pulses
 *        required for startup activation
 * @param value : New updated value to set
 * @return None
 */
void PSS_SetStartupPulsesCount(uint32_t value);

/**
 * @brief Setter for the pss running window
 * @param value : New updated value to set
 * @return None
 */
void PSS_SetRunningWindow(uint16_t value);

/**
 * @brief Setter for the number of pulses
 *        required for running activation
 * @param value : New updated value to set
 * @return None
 */
void PSS_SetRunningPulsesCount(uint32_t value);

/**
 * @brief Setter to update the number of magnet
 * @param value : New updated value to set
 * @return None
 */
void PSS_SetNumberOfMagnets(uint8_t value);

/**
  @brief  Update the pulse capture value coming from the ISR
  @param  Capture : Value capture by the ISR
  @return None
*/
void PSS_UpdatePulseFromISR(uint32_t capture);

/**
  @brief  Update the overflow coming from ISR
  @return None
*/
void PSS_OverflowPulseFromISR(void);

#endif
