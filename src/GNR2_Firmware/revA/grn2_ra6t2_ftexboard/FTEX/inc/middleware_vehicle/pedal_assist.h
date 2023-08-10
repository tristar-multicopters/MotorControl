/**
  * @file    pedal_assist.h
  * @brief   This module handles management of the vehicle pedal assist
  *
  */
    
    /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PEDAL_ASSIST_H
#define __PEDAL_ASSIST_H


#include "pedal_speed_sensor.h"
#include "pedal_torque_sensor.h"
#include "wheel_speed_sensor.h"

// ============================== Defines =============================== // 
#define PAS_PERCENTAGE          (uint8_t)100    /* Percentage for PAS use */

#define TORQUE_THRESHOLD_AVG_NB   10  /* Number of values we use to do an average to check for Torque PAS threshold */ 

// ======================== Configuration enums ======================== // 
typedef enum
{
    PAS_LEVEL_0 = 0,
    PAS_LEVEL_1,
    PAS_LEVEL_2,
    PAS_LEVEL_3,
    PAS_LEVEL_4,
    PAS_LEVEL_5,
    PAS_LEVEL_6,
    PAS_LEVEL_7,
    PAS_LEVEL_8,
    PAS_LEVEL_9,
    PAS_LEVEL_WALK = 0xF,
} PasLevel_t;


#define DEFAULT_PAS_LEVEL PAS_LEVEL_1

typedef enum
{
    TorqueSensorUse = 0,    // Torque sensor use define
    CadenceSensorUse,       // Cadence sensor use define
}PasAlgorithm_t;

// ======================== Configuration structures ======================== // 
typedef struct
{ 
    int16_t  hPASMaxTorque;                 // PAS Maximum given torque
    uint16_t hPASMaxSpeed;                  // PAS Maximum given speed
    
    uint16_t TorquePasMaxSpeed;             // Maxiumumu speed in torque pas (not level specific for now)
    uint8_t  bMaxLevel;                     // PAS maximum given Level
    uint8_t  bTorqueGain;                   // User gain used to affect the torque ramp in %
    int16_t  hMaxTorqueRatio;               // PAS maximum torque ratio
    
    bool WalkmodeOverThrottle;              // Flag used to decide if walk mode has higher priority than throttle          
    
    uint8_t bPASCountSafe;                  // Counter for safe detection of the PAS after one pedaling
    uint8_t bPASCountActivation;            // Counter for slow PAS detection over than 700ms periode

    int16_t PASCTorqRatiosInPercentage[10]; // Cadence Torque ratio in % for each level
    int16_t PASTTorqRatiosInPercentage[10]; // Torque Torque ratio in % for each level
    int16_t walkModeTorqueRatio;            // Torque ratio in % for walk mode
} PAS_Parameters_t;

typedef struct
{   
    int16_t hTorqueSelect;                        // Select torque to feed for motor control
    PasLevel_t bCurrentAssistLevel;               // Current pedal assist level
    PasAlgorithm_t  bCurrentPasAlgorithm;         // Current PAS used Algorithm
    uint16_t hPASSelectedSpeed;                   // current PAS speed selected by user 
   
    bool bPASDetected;                            // Use PAS flag  for detection
    
    PedalSpeedSensorHandle_t * pPSS;              // Pointer to Pedal Speed Sensor handle
    PedalTorqSensorHandle_t * pPTS;               // Pointer to Pedal Torque Sensor handle   
    WheelSpeedSensorHandle_t * pWSS;              // Pointer to Wheel Speed Sensor handle
        
    PAS_Parameters_t sParameters;                 // Structure for powertrain parameters
    
} PAS_Handle_t;

// ======================== Public Functions ======================== //
/**
    * @brief  Module initialization, to be called once before using it
    * @param  Pedal Assist handle & Delay Handle
    * @retval None
    */
void PedalAssist_Init(PAS_Handle_t * pHandle, Delay_Handle_t * pPTSstuckDelay);

/**
    * @brief  Set pedal assist level
    * @param  Pedal Assist handle
    * @param  bLevel: Desired pedal assist level
    * @retval None
    */
void PedalAssist_SetAssistLevel(PAS_Handle_t * pHandle, uint8_t bLevel);

/**
    * @brief  Get pedal assist level
    * @param  Pedal Assist handle
    * @retval Current pedal assist level in uint8_t format
    */
uint8_t PedalAssist_GetAssistLevel(PAS_Handle_t * pHandle);

/**
    * @brief  Set Pedal Assist standard torque based on screen informations
    * @param  Pedal Assist handle
    * @retval pRefTorque in int16
    */
int16_t PedalAssist_GetPASTorque(PAS_Handle_t * pHandle);

/**
    * @brief  Set Pedal Assist standard torque based on screen informations
    *         for Cadence PAS base
    * @param  Pedal Assist handle
    * @retval pRefTorque in int16
    */
int16_t PedalAssist_GetPASCadenceMotorTorque(PAS_Handle_t * pHandle);

/**
    * @brief  Update Pedal Assist standard speed based on pas level
    * @param  Pedal Assist handle
    * @retval nothing
    */
void PedalAssist_PASUpdateMaxSpeed(PAS_Handle_t * pHandle);

/**
    * @brief  Get the Pedal Assist standard speed based on pas level
    * @param  Pedal Assist handle
    * @retval current PAS speed limit
    */
uint16_t PedalAssist_GetPASMaxSpeed(PAS_Handle_t * pHandle);

/**
    * @brief  Set the generic top speed for torque pas (not level specific) 
    * @param  Pedal Assist handle
    * @retval desired PAS speed limit
    */
void PedalAssist_SetTorquePASMaxSpeed(PAS_Handle_t * pHandle, uint16_t topSpeed);

/**
    * @brief  Set Pedal Assist torque based on the pedal Torque Sensor
    * @param  Pedal Assist handle
    * @retval pRefTorqueS in int16
    */
int16_t PedalAssist_GetTorqueFromTS(PAS_Handle_t * pHandle);

/**
    * @brief  Check the PAS Presence Flag
    * @param  Pedal Assist handle
    * @retval None
    */
void PedalAssist_UpdatePASDetection(PAS_Handle_t * pHandle);

/**
    * @brief  Check the PAS Presence Flag for slow PAS detection sensors
    * @param  Pedal Assist handle
    * @retval None
    */
void PedalAssist_UpdatePASDetectionCall(PAS_Handle_t * pHandle);

/**
    * @brief  Set Pedal Assist standard speed based on screen informations
    * @param  Pedal Assist handle
    * @retval pRefTorque in int16
    */
void PedalAssist_PASSetMaxSpeed_Standard(PAS_Handle_t * pHandle);

/**
    * @brief  Return if pedals are moving or not
    * @param  Pedal Assist handle
    * @retval True if pedal movement is detected, false otherwise
    */
bool PedalAssist_IsPASDetected(PAS_Handle_t * pHandle);

/**
    * @brief  Return if walk mode is active
    * @param  Pedal Assist handle
    * @retval True if walk mode is detected, false otherwise
    */
bool PedalAssist_IsWalkModeDetected(PAS_Handle_t * pHandle);

/**
    * @brief  Reset the PAS Prameters
    * @param  Pedal Assist handle
    * @retval None
    */
void PedalAssist_ResetParameters (PAS_Handle_t * pHandle);

#endif /*__PEDAL_ASSIST_H*/

