/**
  * @file    powertrain_management.h
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles management of the vehicle powertrain
  *
    */
    
    /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __POWERTRAIN_MANAGEMENT_H
#define __POWERTRAIN_MANAGEMENT_H

#include "md_interface.h"
#include "throttle.h"
#include "brake.h"
#include "lights.h"
#include "motor_selection.h"
#include "vc_defines.h"
#include "foldback.h"
#include "power_enable.h"
#include "battery_monitoring.h"

#include "pedal_speed_sensor.h"
#include "pedal_torque_sensor.h"
#include "wheel_speed_sensor.h"

// ============================== Defines =============================== // 
#define PAS_PERCENTAGE	    (uint8_t)100	/* Percentage for PAS use */

#define PAS_LEVEL_SPEED_0   (uint8_t)0      /* Maximum Speed for PAS Level 0 in Km/h */
#define PAS_LEVEL_SPEED_1   (uint8_t)10     /* Maximum Speed for PAS Level 1 in Km/h */
#define PAS_LEVEL_SPEED_2   (uint8_t)15     /* Maximum Speed for PAS Level 2 in Km/h */
#define PAS_LEVEL_SPEED_3   (uint8_t)20     /* Maximum Speed for PAS Level 3 in Km/h */
#define PAS_LEVEL_SPEED_4   (uint8_t)25     /* Maximum Speed for PAS Level 4 in Km/h */
#define PAS_LEVEL_SPEED_5   (uint8_t)30     /* Maximum Speed for PAS Level 5 in Km/h */

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
} PasLevel_t;

typedef enum
{
    HUB,
    MIDDRIVE,
} DrivetrainType_t;

typedef enum
{
    SINGLE_MOTOR,
    DUAL_MOTOR,
} PowertrainMode_t;

typedef enum
{
    TORQUE_CTRL,
    SPEED_CTRL,
} CtrlType_t;

// ======================== Configuration structures ======================== // 
typedef struct
{ 
    bool bUseMotorM1;                    /* To set once, true if motor 1 is used */
    bool bUseMotorM2;                    /* To set once, true if motor 2 is used */
    
    bool bM2TorqueInversion;             /* Set true if M2 torque sign is different than M1  */
    
    DrivetrainType_t bDrivetrainType;    /* Vehicle drivetrain type (i.e. hub, middrive, ...) */
    PowertrainMode_t bMode;              /* Single or dual motor. It is updated by user using motor selector switch */
    uint8_t bDefaultMainMotor;           /* Default main motor selection */
    CtrlType_t bCtrlType;                /* Torque or speed control */
    bool bEnableDualMotorStartup;        /* When in single motor mode, second motor will assist during startup until a certain speed if enabled. */
    
    uint16_t hStartingThrottle;          /* Minimum torque to start powertrain */
    uint16_t hStoppingThrottle;          /* Minimum torque to stop powertrain */
    uint16_t hStoppingSpeed;             /* Minimum speed to stop powertrain */
    
    int16_t hPASMaxTorque;               /* PAS Maximum given torque*/
    uint16_t hPASMaxSpeed;               /* PAS Maximum given speed */
    uint16_t hPASMaxKmSpeed;              /* PAS Maximum Km/h speed */
		
    uint8_t bMaxLevel;                   /* PAS maximum given Level */
    uint8_t bCoeffLevel;                 /*  User Coefficient used to multiply the Torque ramp if needed */
    int16_t	hMaxTorqueRatio;             /* PAS maximum torque ratio */
    uint16_t hMaxSpeedRatio;             /* PAS maximum speed ratio */
	
    bool bTorqueSensorUse;               /* Torque sensor use flag */
	
    uint32_t MotorToHubGearRatio;        /* Gear ratio of the motor Top 16 bits is numerator bottom 16 bits is denominator of ratio ex 3/2 would be 0x0003 0002 */
    uint16_t hFaultManagementTimeout;    /* Number of ticks the state machine should stay on fault state before restart */
    
    uint8_t bPASCountSafe;               /* Counter for safe detection of the PAS after one pedaling*/
		
} PWRT_Parameters_t;

typedef struct
{
    MultipleDriveInterfaceHandle_t * pMDI;        /* Pointer to MDI handle */
    ThrottleHandle_t * pThrottle;                 /* Pointer to throttle handle */
    BRK_Handle_t * pBrake;                        /* Pointer to brake handle */
    Light_Handle_t * pHeadLight;                  /* Pointer to front light handle */
    Light_Handle_t * pTailLight;                  /* Pointer to rear light handle */
    BatMonitor_Handle_t * pBatMonitorHandle;      /* Pointer to Battery monitor */    
    MS_Handle_t * pMS;                            /* Pointer to motor selector handle */
    PWREN_Handle_t * pPWREN;                      /* Pointer to power enable pin handle */
    PedalSpeedSensorHandle_t * pPSS;              /* Pointer to Pedal Speed Sensor handle */
    PedalTorqSensorHandle_t * pPTS;               /* Pointer to Pedal Torque Sensor handle */	
    WheelSpeedSensorHandle_t * pWSS;              /* Pointer to Wheel Speed Sensor handle */
    
    uint8_t bMainMotor;                           /* Main motor selection. It is updated by user using motor selector switch */
    int16_t aTorque[2];                           /* Array of torque reference, first element is for M1, second is for M2 */
    int16_t aSpeed[2];                            /* Array of speed reference, first element is for M1, second is for M2 */
    
    int16_t hTorqueSelect;                        /* Select torque to feed for motor control */
    PasLevel_t bCurrentAssistLevel;                  /* Current pedal assist level */
    bool bPASDetected;                            /* Use PAS flag  for detection */
	
    Foldback_Handle_t DCVoltageFoldback;          /* Foldback handle using DCbus voltage */
    Foldback_Handle_t SpeedFoldbackStartupDualMotor;     /* Foldback handle using speed for dual motor control */
    
    uint16_t aFaultManagementCounters[3][2];      /* Array of counter before acknowledging motor faults. First dimension is
                                                   fault type in this order: Over current, startup, and speed feedback. 
                                                   Second dimension is for motor number in this order: M1 and M2 */

    PWRT_Parameters_t sParameters;                /* Structure for powertrain parameters */
    
} PWRT_Handle_t;

// ======================== Public Functions ======================== //
/**
    * @brief  Module initialization, to be called once before using it
    * @param  Powertrain handle
    *    @param     Motor control interface handle for M1
    * @retval None
    */
void PWRT_Init(PWRT_Handle_t * pHandle, MotorControlInterfaceHandle_t * pMci_M1, SlaveMotorHandle_t * pSlaveM2);

/**
    * @brief  Update current value of powertrain peripherals, such as throttle. To be called periodically.
    * @param  Powertrain handle
    * @retval None
    */
void PWRT_UpdatePowertrainPeripherals(PWRT_Handle_t * pHandle);

/**
    * @brief  Compute target torque and speed to be applied to each motors.
    * @param  Powertrain handle
    * @retval None
    */
void PWRT_CalcMotorTorqueSpeed(PWRT_Handle_t * pHandle);

/**
    * @brief  Send torque and/or speed ramp commands to motors
    * @param  Powertrain handle
    * @retval None
    */
void PWRT_ApplyMotorRamps(PWRT_Handle_t * pHandle);

/**
    * @brief  Send command to start motors
    * @param  Powertrain handle
    * @retval None
    */
void PWRT_StartMotors(PWRT_Handle_t * pHandle);

/**
    * @brief  Send command to stop motors
    * @param  Powertrain handle
    * @retval None
    */
void PWRT_StopMotors(PWRT_Handle_t * pHandle);

/**
    * @brief  Check for motor faults during standby state
    * @param  Powertrain handle
    * @retval Vehicle fault code
    */
uint16_t PWRT_StandbyStateCheck(PWRT_Handle_t * pHandle);

/**
    * @brief  Check for motor faults during start state
    * @param  Powertrain handle
    * @retval Vehicle fault code
    */
uint16_t PWRT_StartStateCheck(PWRT_Handle_t * pHandle);

/**
    * @brief  Check for motor faults during run state
    * @param  Powertrain handle
    * @retval Vehicle fault code
    */
uint16_t PWRT_RunStateCheck(PWRT_Handle_t * pHandle);

/**
    * @brief  Check for motor faults during stop state
    * @param  Powertrain handle
    * @retval Vehicle fault code
    */
uint16_t PWRT_StopStateCheck(PWRT_Handle_t * pHandle);

/**
    * @brief  Check if powertrain is active (e.g. motors are in running state)
    * @param  Powertrain handle
    * @retval Returns true if powertrain is active
    */
bool PWRT_IsPowertrainActive(PWRT_Handle_t * pHandle);

/**
    * @brief  Check if powertrain is stopped (e.g. motors are in idle state)
    * @param  Powertrain handle
    * @retval Returns true if powertrain is stopped
    */
bool PWRT_IsPowertrainStopped(PWRT_Handle_t * pHandle);

/**
    * @brief  Check if conditions to stop powertrain are met
    * @param  Powertrain handle
    * @retval Returns true if powertrain can be stopped
    */
bool PWRT_CheckStopConditions(PWRT_Handle_t * pHandle);

/**
    * @brief  Check if conditions to start powertrain are met
    * @param  Powertrain handle
    * @retval Returns true if powertrain can be started
    */
bool PWRT_CheckStartConditions(PWRT_Handle_t * pHandle);

/**
    * @brief  Manage motor faults. Check if faults are still present and send motor fault acknowledge when faults are gone.
    * @param  Powertrain handle
    * @retval Returns true if a motor fault is still active, false if no more fault is present.
    */
bool PWRT_MotorFaultManagement(PWRT_Handle_t * pHandle);

/**
    * @brief  Get main motor reference torque
    * @param  Powertrain handle
    * @retval Returns main motor reference torque
    */
int16_t PWRT_GetTorqueRefMainMotor(PWRT_Handle_t * pHandle);
/**
    * @brief  Get motor 1 reference torque
    * @param  Powertrain handle
    * @retval Returns motor 1 reference torque
    */
int16_t PWRT_GetTorqueRefM1(PWRT_Handle_t * pHandle);

/**
    * @brief  Get motor 2 reference torque
    * @param  Powertrain handle
    * @retval Returns motor 2 reference torque
    */
int16_t PWRT_GetTorqueRefM2(PWRT_Handle_t * pHandle);

/**
    * @brief  Get main motor speed reference
    * @param  Powertrain handle
    * @retval Returns main motor speed reference
    */
int32_t PWRT_GetSpeedRefMainMotor(PWRT_Handle_t * pHandle);

/**
    * @brief  Get motor 1 speed reference
    * @param  Powertrain handle
    * @retval Returns motor 1 speed reference
    */
int32_t PWRT_GetSpeedRefM1(PWRT_Handle_t * pHandle);

/**
    * @brief  Get motor 2 speed reference
    * @param  Powertrain handle
    * @retval Returns motor 2 speed reference
    */
int32_t PWRT_GetSpeedRefM2(PWRT_Handle_t * pHandle);

/**
    * @brief  Check if motor 1 is used
    * @param  Powertrain handle
    * @retval Returns true if motor 1 is used
    */
bool PWRT_IsMotor1Used(PWRT_Handle_t * pHandle);

/**
    * @brief  Check if motor 2 is used
    * @param  Powertrain handle
    * @retval Returns true if motor 2 is used
    */
bool PWRT_IsMotor2Used(PWRT_Handle_t * pHandle);

/**
    * @brief  Set pedal assist level
    * @param  pHandle: powertrain handle
    * @param  bLevel: Desired pedal assist level
    * @retval None
    */
void PWRT_SetAssistLevel(PWRT_Handle_t * pHandle, uint8_t bLevel);

/**
    * @brief  Get pedal assist level
    * @param  pHandle: powertrain handle
    * @retval Current pedal assist level in uint8_t format
    */
uint8_t PWRT_GetAssistLevel(PWRT_Handle_t * pHandle);

/**
	* @brief  Set PAS level
	* @param  Powertrain handle
	* @param  PAS level handle
	* @retval None
	*/
void PWRT_SetAssistLevel(PWRT_Handle_t * pHandle, PasLevel_t bLevel);

/**
	* @brief  Get PAS level
	* @param  Powertrain handle
	* @retval Current Pedal assit level
	*/
PasLevel_t PWRT_GetAssistLevel (PWRT_Handle_t * pHandle);

/**
	* @brief  Set Pedal Assist standard torque based on screen informations
	* @param  Powertrain handle
	* @retval pRefTorque in int16
	*/
int16_t PWRT_GetPASTorque(PWRT_Handle_t * pHandle);

/**
	* @brief  Set Pedal Assist standard torque based on screen informations
    *         for Cadence PAS base
	* @param  Powertrain handle
	* @retval pRefTorque in int16
	*/
int16_t PWRT_GetPASTorqueSpeed(PWRT_Handle_t * pHandle);

/**
	* @brief  Set Pedal Assist standard speed based on screen informations
	* @param  Powertrain handle
	* @retval pRefTorque in int16
	*/
void PWRT_PASSetMaxSpeed(PWRT_Handle_t * pHandle);
/**
	* @brief  Set Pedal Assist torque based on the pedal Torque Sensor
	* @param  Powertrain handle
	* @retval pRefTorqueS in int16
	*/
int16_t PWRT_GetTorqueFromTS(PWRT_Handle_t * pHandle);

/**
	* @brief  Check the PAS Presence Flag
	* @param  Powertrain handle
	* @retval None
	*/
void PWRT_UpdatePASDetection (PWRT_Handle_t * pHandle);

/**
	* @brief  Return if pedals are moving or not
	* @param  Powertrain handle
	* @retval True if pedal movement is detected, false otherwise
	*/
bool PWRT_IsPASDetected(PWRT_Handle_t * pHandle);

/**
	* @brief  Select Control assistance based on Throttle or PAS
	* @param  Powertrain handle
	* @retval pHandle->pTorqueSelect in int16                                                                                    
	*/
int16_t PWRT_CalcSelectedTorque(PWRT_Handle_t * pHandle);


#endif /*__POWERTRAIN_MANAGEMENT_H*/

