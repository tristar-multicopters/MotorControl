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
#include "foldback.h"

// ============================== Defines =============================== // 
#define PAS_PERCENTAGE          (uint8_t)100    /* Percentage for PAS use */

#define PAS_LEVEL_SPEED_0       (uint8_t)0      /* Maximum Speed for PAS Level 0 in Km/h */
#define PAS_LEVEL_SPEED_1       (uint8_t)10     /* Maximum Speed for PAS Level 1 in Km/h */
#define PAS_LEVEL_SPEED_2       (uint8_t)15     /* Maximum Speed for PAS Level 2 in Km/h */
#define PAS_LEVEL_SPEED_3       (uint8_t)20     /* Maximum Speed for PAS Level 3 in Km/h */
#define PAS_LEVEL_SPEED_4       (uint8_t)25     /* Maximum Speed for PAS Level 4 in Km/h */
#define PAS_LEVEL_SPEED_5       (uint8_t)30     /* Maximum Speed for PAS Level 5 in Km/h */

#define PAS_LEVEL_SPEED_WALK    (uint8_t)3     /* Maximum Speed for PAS Level 5 in Km/h */

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
    PAS_LEVEL_WALK,
} PasLevel_t;


// ======================== Configuration structures ======================== // 
typedef struct
{ 
    
    int16_t  hPASMaxTorque;               /* PAS Maximum given torque*/
    uint16_t hPASMaxSpeed;                /* PAS Maximum given speed */
    uint16_t hPASMaxKmSpeed;              /* PAS Maximum Km/h speed */
    
    uint8_t  bMaxLevel;                   /* PAS maximum given Level */
    uint8_t  bCoeffLevel;                 /*  User Coefficient used to multiply the Torque ramp if needed */
    int16_t  hMaxTorqueRatio;             /* PAS maximum torque ratio */
    uint16_t hMaxSpeedRatio;              /* PAS maximum speed ratio */
    
    bool bTorqueSensorUse;                /* Torque sensor use flag */
    bool WalkmodeOverThrottle;            /* Flag used to decide if walk mode has higher priority than throttle */
    
    uint8_t bPASCountSafe;                /* Counter for safe detection of the PAS after one pedaling*/
        
} PAS_Parameters_t;

typedef struct
{
   
    int16_t hTorqueSelect;                        /* Select torque to feed for motor control */
    PasLevel_t bCurrentAssistLevel;               /* Current pedal assist level */
    bool bPASDetected;                            /* Use PAS flag  for detection */
    
    PedalSpeedSensorHandle_t * pPSS;              /* Pointer to Pedal Speed Sensor handle */
    PedalTorqSensorHandle_t * pPTS;               /* Pointer to Pedal Torque Sensor handle */    
    WheelSpeedSensorHandle_t * pWSS;              /* Pointer to Wheel Speed Sensor handle */
    
    Foldback_Handle_t DCVoltageFoldback;                /* Foldback handle using DCbus voltage */
    Foldback_Handle_t *SpeedFoldbackStartupDualMotorPAS;    /* Foldback handle using speed for dual motor control */
    
    PAS_Parameters_t sParameters;                /* Structure for powertrain parameters */
    
} PAS_Handle_t;

// ======================== Public Functions ======================== //
/**
    * @brief  Module initialization, to be called once before using it
    * @param  Pedal Assist handle
    * @retval None
    */
void PedalAssist_Init(PAS_Handle_t * pHandle);

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
int16_t PedalAssist_GetPASTorqueSpeed(PAS_Handle_t * pHandle);

/**
    * @brief  Set Pedal Assist standard speed based on screen informations
    * @param  Pedal Assist handle
    * @retval pRefTorque in int16
    */
void PedalAssist_PASSetMaxSpeed(PAS_Handle_t * pHandle);
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
    * @brief  Return if pedals are moving or not
    * @param  Pedal Assist handle
    * @retval True if pedal movement is detected, false otherwise
    */
bool PedalAssist_IsPASDetected(PAS_Handle_t * pHandle);


bool PedalAssist_IsWalkModeDetected(PAS_Handle_t * pHandle);

#endif /*__PEDAL_ASSIST_H*/

