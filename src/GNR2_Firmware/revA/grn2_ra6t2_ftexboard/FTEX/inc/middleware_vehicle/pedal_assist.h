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
#include "ramps.h"

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


#define DEFAULT_PAS_LEVEL PAS_LEVEL_5

typedef enum
{
    noSensorUse = 0,
    TorqueSensorUse,    // Torque sensor use define
    CadenceSensorUse,   // Cadence sensor use define
    HybridAndSensorUse, // Torque AND Cadence sensor use define
    HybridOrSensorUse,  // Torque OR Cadence sensor use define
}PasAlgorithm_t;

//emum used to control the PAS cadence detection
//on differents situation.
typedef enum 
{
    
  CADENCE_DETECTION_STARTUP, 
  CADENCE_DETECTION_RUNNING,    
    
}PasCadenceState_t;

// ======================== Configuration structures ======================== // 
typedef struct
{ 
    int16_t  hPASMaxTorque;                 // PAS Maximum given torque
    
    uint16_t PasMaxSpeed;                   // Maxiumumu Pas speed in torque/cadence.
    uint8_t  bMaxLevel;                     // PAS maximum given Level
    uint16_t bTorqueGain[10];                   // User gain used to affect the torque ramp in %
    int16_t  hMaxTorqueRatio;               // PAS maximum torque ratio
    
    bool WalkmodeOverThrottle;              // Flag used to decide if walk mode has higher priority than throttle          

    uint8_t PASMaxSpeed[10];                  // Max speed to each PAS level.
    uint8_t PASMinTorqRatiosInPercentage[10]; // Min PAS Torque ratio in % for each level
    uint8_t PASMaxTorqRatiosInPercentage[10]; // Max PAS Torque ratio in % for each level
    int16_t walkModeTorqueRatio;              // Torque ratio in % for walk mode
    
    Ramps_Handle_t PasRamps[2][10];           // Acceleration and deceleration ramps for each pas level
    Ramps_Handle_t PasWalkmodeRamp;   
    
} PAS_Parameters_t;

typedef struct
{   
    int16_t hTorqueSelect;                        // Select torque to feed for motor control
    PasLevel_t bCurrentAssistLevel;               // Current pedal assist level
    PasAlgorithm_t  bPasPowerAlgorithm;           // Current PAS used Algorithm
    PasAlgorithm_t  bStartupPasAlgorithm;         // Current PAS used Algorithm
    PasAlgorithm_t  bRunningPasAlgorithm;         // Current PAS used Algorithm
    uint16_t hPASSelectedSpeed;                   // current PAS speed selected by user 
   
    bool bPASDetected;                            // Use PAS flag  for detection
    bool bCadenceStartupPASDetected;              // Flag used to Cadence PAS detection on startup state.
    bool bCadenceRunningPASDetected;              // Flag used to Cadence PAS detection on running state. 
    bool bTorqueStartupPASDetected;               // Flag used to Torque PAS detection on startup state.
    bool bTorqueRunningPASDetected;               // Flag used to Torque PAS detection on running state. 
    
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
    * @retval current PAS speed limit
    */
uint16_t PedalAssist_PASUpdateMaxSpeed(PAS_Handle_t * pHandle);

/**
    * @brief  Set the generic top speed for torque pas (not level specific) 
    * @param  Pedal Assist handle
    * @retval desired PAS speed limit
    */
void PedalAssist_SetPASMaxSpeed(PAS_Handle_t * pHandle, uint16_t topSpeed);

/**
    * @brief  Set Pedal Assist torque based on the pedal Torque Sensor
    * @param  Pedal Assist handle
    * @retval pRefTorqueS in int16
    */
int16_t PedalAssist_GetTorqueFromTS(PAS_Handle_t * pHandle);

/**
    * @brief  Check the PAS Presence Flag based on torque detection
    * @param  Pedal Assist handle
    * @retval None
    */
void PedalAssist_TorquePASDetection(PAS_Handle_t * pHandle);

/**
    * @brief  Detect the PAS based on cadence detection
    * @param  Pedal Assist handle
    * @param  Increment time used by the windowsDetectionLimite in ms.
    * @retval None
    */
void PedalAssist_CadencePASDetection (PAS_Handle_t * pHandle, uint16_t windowsIncrementTimeMs);

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
    * @brief  Reset PAS detected flag
    * @param  Pedal Assist handle
    * @retval none
    */
void PedalAssist_ResetPASDetected(PAS_Handle_t * pHandle);

/**
    * @brief  Set PAS detected flag
    * @param  Pedal Assist handle
    * @retval none
    */
void PedalAssist_SetPASDetected(PAS_Handle_t * pHandle);

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

/**
    * @brief  Get the Startup Pas algorithm
    * @param  Pedal Assist handle,
    * @retval PasAlgorithm_t
    */
PasAlgorithm_t PedalAssist_GetStartupPasAlgorithm(PAS_Handle_t * pHandle);

/**
    * @brief  Get the Running Pas algorithm
    * @param  Pedal Assist handle,
    * @retval PasAlgorithm_t
    */
PasAlgorithm_t PedalAssist_GetRunningPasAlgorithm(PAS_Handle_t * pHandle);

/**
    * @brief  Set startup PAS algorithm
    * @param  Pedal Assist handle, new pas algorithm
    * @retval None
    */
void PedalAssist_SetStartupPASAlgorithm(PAS_Handle_t * pHandle, PasAlgorithm_t aPASAlgo);

/**
    * @brief  Set running PAS algorithm
    * @param  Pedal Assist handle, new pas algorithm
    * @retval None
    */
void PedalAssist_SetRunningPASAlgorithm(PAS_Handle_t * pHandle, PasAlgorithm_t aPASAlgo);

/**
    * @brief  Get the ramp that should be applied
    * @param  Pedal Assist handle, Ramp to apply
    * @retval void
    */
Ramps_Handle_t * PedalAssist_GetRamp(PAS_Handle_t * pHandle, uint8_t Direction);

/**
    * @brief  Reset Cadence State Pas Dection
    * @param  None
    * @retval None
    */
void PedalAssist_ResetCadenceStatePasDection(void);

/**
    * @brief  Try to detect PAS
    * @param  Pedal Assist handle
    * @retval None
    */
void PedalAssist_PasDetection(PAS_Handle_t * pHandle);

/**
    * @brief  Reset Pas detection flag from Cadence on startup state.
    * @param  Pedal Assist handle
    * @retval None
    */
void PedalAssist_ResetCadenceStartupPasDection(PAS_Handle_t * pHandle);

/**
    * @brief  Set Pas detection flag from Cadence on startup state.
    * @param  Pedal Assist handle
    * @retval None
    */
void PedalAssist_SetCadenceStartupPasDection(PAS_Handle_t * pHandle);

/**
    * @brief  Reset Pas detection flag from Cadence on running state.
    * @param  Pedal Assist handle
    * @retval None
    */
void PedalAssist_ResetCadenceRunningPasDection(PAS_Handle_t * pHandle);

/**
    * @brief  Set Pas detection flag on Cadence on running state.
    * @param  Pedal Assist handle
    * @retval None
    */
void PedalAssist_SetCadenceRunningPasDection(PAS_Handle_t * pHandle);

/**
    * @brief  Reset Pas detection flag from Torque on startup state.
    * @param  Pedal Assist handle
    * @retval None
    */
void PedalAssist_ResetTorqueStartupPasDection(PAS_Handle_t * pHandle);

/**
    * @brief  Set Pas detection flag from Torque on startup state.
    * @param  Pedal Assist handle
    * @retval None
    */
void PedalAssist_SetTorqueStartupPasDection(PAS_Handle_t * pHandle);

/**
    * @brief  Reset Pas detection flag from Torque on running state.
    * @param  Pedal Assist handle
    * @retval None
    */
void PedalAssist_ResetTorqueRunningPasDection(PAS_Handle_t * pHandle);

/**
    * @brief  Set Pas detection flag from Torque on running state.
    * @param  Pedal Assist handle
    * @retval None
    */
void PedalAssist_SetTorqueRunningPasDection(PAS_Handle_t * pHandle);

#endif /*__PEDAL_ASSIST_H*/

