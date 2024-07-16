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
#include "wheel.h"

// ============================== Defines =============================== // 
#define PAS_PERCENTAGE          (uint8_t)100    /* Percentage for PAS use */
#define TORQUE_THRESHOLD_AVG_NB   10  /* Number of values we use to do an average to check for Torque PAS threshold */
#define TORQUE_SENSOR_TIMEOUT_THRESHOLD 800  // Number of tick before signaling no pedal activity (Approx. 800*5ms = 4sec).

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


#define DEFAULT_PAS_LEVEL PAS_LEVEL_0

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
    bool PASOverThrottle;                   // Flag used to decide if pas has higher priority than throttle
    
    uint8_t PASMaxSpeed[10];                  // Max speed to each PAS level.
    uint8_t PASMinTorqRatiosInPercentage[10]; // Min PAS Torque ratio in % for each level
    uint8_t PASMaxTorqRatiosInPercentage[10]; // Max PAS Torque ratio in % for each level
    int16_t walkModeTorqueRatio;              // Torque ratio in % for walk mode    
} PAS_Parameters_t;

typedef struct
{   
    int16_t hTorqueSelect;                        // Select torque to feed for motor control
    PasLevel_t bCurrentAssistLevel;               // Current pedal assist level
    PasAlgorithm_t  bPasPowerAlgorithm;           // Current PAS used Algorithm
    PasAlgorithm_t  bStartupPasAlgorithm;         // Current PAS used Algorithm
    PasAlgorithm_t  bRunningPasAlgorithm;         // Current PAS used Algorithm
    uint16_t hPASSelectedSpeed;                   // current PAS speed selected by user 

    bool bPASPowerEnable;                         // Flag used to validate if PAS needs to push power
    bool bPASCadenceRunningOverride;              // Flag used to override the power delivery mechanism on cadence
    bool bPASTorqueRunningOverride;               // Flag used to override the power delivery mechanism on torque
    bool cadenceAndOrTorqueFlag;

    uint16_t torqueSensorIssueTimer;              // Number of tick before we trigger the torque sensor issue flag
    bool bTorqueSensorIssue;                      // Flag used to detect if there is torque sensor issue detected.

    PAS_Parameters_t sParameters;                 // Structure for powertrain parameter 
} PAS_Handle_t;

// ======================== Public Functions ======================== //
/**
    * @brief  Module initialization, to be called once before using it
    * @param  Pedal Assist handle & Delay Handle
    * @retval None
    */
void PedalAssist_Init(PAS_Handle_t * pHandle, Delay_Handle_t * pPTSstuckDelay);

/**
    * @brief  Rest of module initialization after MC has been initialized, to be called once before using it
    * @param  Pedal Assist handle, max torque & wheel speed sensor number of magnets per rotation
    * @retval None
    */
void PedalAssist_InitTorqueAndWheelSpeedSensor(PAS_Handle_t * pHandle, uint16_t maxTorque, uint8_t wheelSpdSensorNbrPerRotation);
    
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
    * @brief  Update the PAS Handle power according to Cadence and Torque overrides
    * @param  Pedal Assist handle
    */
void PedalAssist_PASPowerDetection(PAS_Handle_t *pHandle);

/**
    * @brief  Check if the user is currently in cadence state, is currently pedalling
    * @param  Pedal Assist handle
    * @retval True if cadence(pedalling activity) is detected
    */
bool PedalAssist_IsCadenceDetected(PAS_Handle_t *pHandle);

/**
    * @brief  Set Pedal Assist standard torque based on screen informations
    * @param  Pedal Assist handle
    * @retval pRefTorque in int16
    */
int16_t PedalAssist_GetPASTorque(PAS_Handle_t * pHandle);

/**
    * @brief  Set Pedal Assist standard cadence based on screen informations
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
    * @brief  Set Pedal Assist torque based on the pedal Torque Sensor
    * @param  Pedal Assist handle
    * @retval pRefTorqueS in int16
    */
int16_t PedalAssist_GetTorqueFromTS(PAS_Handle_t * pHandle);

/**
    * @brief  Get walkmode torque
    * @param  Pedal Assist handle
    * @retval pRefTorqueS in int16
    */
int16_t PedalAssist_GetWalkmodeTorque(PAS_Handle_t * pHandle);

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
    * @brief  Return if walk mode is active
    * @param  Pedal Assist handle
    * @retval True if walk mode is detected, false otherwise
    */
bool PedalAssist_IsWalkModeDetected(PAS_Handle_t * pHandle);

/**
    * @brief  Return if PAS Power Enable is active
    * @param  Pedal Assist handle
    * @retval True if PAS Power Enable is detected, false otherwise
    */
bool PedalAssist_IsPowerEnableDetected(PAS_Handle_t *pHandle);

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
    * @brief  Set a new PAS algorithm
    * @param  Pedal Assist handle, new pas algorithm
    * @retval None
    */
void PedalAssist_SetPASAlgorithm(PAS_Handle_t * pHandle, PasAlgorithm_t aPASAlgo);

/**
    * @brief  Check if a torque sensor issue is detected
    * @param  Pedal Assist handle
    * @retval True if torque sensor issue is detected
    */
bool PedalAssist_TorqueSensorIssueDetected(PAS_Handle_t * pHandle);

#endif /*__PEDAL_ASSIST_H*/
