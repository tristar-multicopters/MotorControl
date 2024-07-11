/**
  * @file    powertrain_management.h
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles management of the vehicle powertrain
  *
    */
    
    /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __POWERTRAIN_MANAGEMENT_H
#define __POWERTRAIN_MANAGEMENT_H

#include "stdbool.h"
#include "stdint.h"
#include "md_interface.h"
#include "pedal_assist.h"
#include "throttle.h"
#include "brake.h"
#include "lights.h"
#include "motor_selection.h"
#include "battery_monitoring.h"
#include "power_enable.h"

// ============================== Defines =============================== // 

#define NBR_CRITICAL_FAULT_COUNTERS 3
#define NBR_MOTORS                  2
#define TORQUE_DECAY                1000

// ======================== Configuration enums ======================== // 
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

typedef enum 
{
  THROTTLE_DELAY = 0,
  PTS_DELAY,
  BRAKE_DELAY,
  ODOMETER_DELAY,  
} PowertrainDelays_t;

// ======================== Configuration structures ======================== // 
typedef struct
{ 
    bool bUseMotorM1;                    // To set once, true if motor 1 is used
    bool bUseMotorM2;                    // To set once, true if motor 2 is used
    
    bool bM2TorqueInversion;             // Set true if M2 torque sign is different than M1
   
    DrivetrainType_t bDrivetrainType;    // Vehicle drivetrain type (i.e. hub, middrive, ...)
    PowertrainMode_t bMode;              // Single or dual motor. It is updated by user using motor selector switch
    uint8_t bDefaultMainMotor;           // Default main motor selection
    CtrlType_t bCtrlType;                // Torque or speed control
    
    bool bPAS0DisableThrottle;           // Will disable the throttle when we are in PAS level 0
    bool bTopSpeedRestrictionEnable;     // Will determine if we must restrict vehicle speed
    
    bool CruiseForceDisengage;
    PasAlgorithm_t PreCruiseControlStartupPASAlgo;  // Keeps track of the startup pas algorithmed used when we engaged cruise control
    PasAlgorithm_t PreCruiseControlRunningPASAlgo;  // Keeps track of the run time pas algorithmed used when we engaged cruise control
    
    uint16_t hStartingThrottle;          // Minimum torque to start powertrain
    uint16_t hStoppingThrottle;          // Minimum torque to stop powertrain
    uint16_t hStoppingSpeed;             // Minimum speed to stop powertrain
    
    uint16_t hFaultManagementTimeout;    // Number of ticks the state machine should stay on fault state before restart
    
    bool bEnableSpeedLimit;              // Enable or disable speed limit
    
    uint16_t VehicleMaxSpeed;            // Contains the max speed that the vehicle can push power (no matter what)
    uint16_t ScreenMaxSpeed;             // Contains the max speed specified by the screen   
        
} PWRT_Parameters_t;

typedef struct
{
    MultipleDriveInterfaceHandle_t * pMDI;        // Pointer to MDI handle
    ThrottleHandle_t * pThrottle;                 // Pointer to throttle handle
    BRK_Handle_t * pBrake;                        // Pointer to brake handle
    Light_Handle_t * pHeadLight;                  // Pointer to front light handle
    Light_Handle_t * pTailLight;                  // Pointer to rear light handle
    BatMonitor_Handle_t * pBatMonitorHandle;      // Pointer to Battery monitor  
    MS_Handle_t * pMS;                            // Pointer to motor selector handle
    PAS_Handle_t *pPAS;
    PWREN_Handle_t * pPWREN;                      // Pointer to power enable pin handle

    uint8_t bMainMotor;                           // Main motor selection. It is updated by user using motor selector switch
    int16_t aTorque[NBR_MOTORS];                  // Array of torque reference, first element is for M1, second is for M2
    int16_t aSpeed[NBR_MOTORS];                   // Array of speed reference, first element is for M1, second is for M2
    
    int16_t hTorqueSelect;                        // Select torque to feed for motor control
    int16_t hOldTorqueSelect;                     // Contaisn the preivous value of torque select that we sent, value can only be updated right before we send it to the MC layer
    
    uint16_t aFaultManagementCounters[NBR_CRITICAL_FAULT_COUNTERS][NBR_MOTORS];      /* Array of counter before acknowledging motor faults. First dimension is
                                                     fault type in this order: Over current, startup, and speed feedback, Stuck Reverse. 
                                                     Second dimension is for motor number in this order: M1 and M2 */

    PWRT_Parameters_t sParameters;                // Structure for powertrain parameters
    
} PWRT_Handle_t;

// ======================== Public Functions ======================== //
/**
  * @brief  Module initialization, to be called once before using it
  * @param  Powertrain handle
  * @param  Motor control interface handle for M1
  * @retval None
  */
void PWRT_Init(PWRT_Handle_t * pHandle, Delay_Handle_t pDelayArray[]);

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
bool PWRT_MotorCriticalFaultManagement(PWRT_Handle_t * pHandle);

/**
  * @brief  Manage motor errors.
  * @param  Powertrain handle
  * @retval none.
  */
void PWRT_MotorErrorManagement(PWRT_Handle_t * pHandle);

/**
  * @brief  Manage motor warnings.
  * @param  Powertrain handle
  * @retval none.
  */
void PWRT_MotorWarningManagement(PWRT_Handle_t * pHandle);

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
  * @brief  Select Control assistance based on Throttle or PAS
  * @param  Powertrain handle
  * @retval pHandle->pTorqueSelect in int16                                                                                    
  */
int16_t PWRT_CalcSelectedTorque(PWRT_Handle_t * pHandle);

/**
  * @brief  Create a smooth transition between Startup and Runtime
  * @param  inputTorque : Power delivered before the smoothing as input
  * @param  cadenceDetected : Flag if the cadence activity is detected
  * @param  PASPowerEnable : Flag if PAS power is enabled
  * @param  throttleOverride : Flag if throttle is activated
  * @param  walkOverPAS : Flag if walk mode is activated
  * @retval pHandle->pTorqueSelect in int16
  */
int16_t PWRT_TransitionStartupRuntimeTorque(int16_t inputTorque, bool cadenceDetected, bool PASPowerEnable,
                                            bool throttleOverride, bool walkOverPAS);

/**
  * @brief  Scale the input torque according to the current pedal RPM
  * @param  inputTorque : Torque value to scale
  * @param  currentPedalRPM : Current pedal RPM measured
  * @retval Output torque scaled from the pedal RPM
  */
int16_t PWRT_ApplyTorqueGainScaling(int16_t inputTorque, uint16_t currentPedalRPM);

/**
  * @brief  Get minimum power required on cadence power enable
  * @param  Powertrain handle
  * @retval Torque power value calculated according to min power                                                                                     
  */
int16_t PWRT_EnableCadencePower(PWRT_Handle_t *pHandle);

/**
  * @brief  Periodic check of the powerlock signal
  * @param  Powertrain handle
  * @retval None                                                                                  
  */
void PWRT_CheckPwrEnable(PWRT_Handle_t * pHandle);

/**
  * @brief  Get the total amount of current the vehicle is pushing
  * @param  Powertrain handle
  * @retval current in amps uin16_t                                                                                   
  */
uint16_t PWRT_GetTotalMotorsCurrent(PWRT_Handle_t * pHandle);

/**
  * @brief  Get the total amount of power the motors are pushing
  * @param  Powertrain handle
  * @retval power in watts uin16_t                                                                                   
  */
uint16_t PWRT_GetTotalMotorsPower(PWRT_Handle_t * pHandle);

/**
  * @brief  Get the approximate DC power (motor power + losses)
  * @param  Powertrain handle
  * @retval power in watts uin16_t                                                                                   
  */
uint16_t PWRT_GetDCPower(PWRT_Handle_t * pHandle);

/**
  * @brief  Get the max DC power (motor power + losses)
  * @param  Powertrain handle
  * @retval power in watts uin16_t                                                                                   
  */
uint16_t PWRT_GetMaxDCPower(PWRT_Handle_t * pHandle);

/**
  * @brief  Get the approximate DC current (motor current + losses)
  * @param  Powertrain handle
  * @retval power in watts uin16_t                                                                                   
  */
uint16_t PWRT_GetDCCurrent(PWRT_Handle_t * pHandle);

/**
  * @brief  Get the total amount of torque the motors are pushing
  * @param  Powertrain handle
  * @retval power in nm uin16_t                                                                                   
  */
uint16_t PWRT_GetTotalMotorsTorque(PWRT_Handle_t * pHandle);

/**
  * @brief  Get the max safe current we can push
  * @param  Powertrain handle
  * @retval current in amps uin16_t                                                                                   
  */
uint16_t PWRT_GetMaxSafeCurrent(PWRT_Handle_t * pHandle);

/**
  * @brief  Get the ongoing max current we can push
  * @param  Powertrain handle
  * @retval current in amps uin16_t                                                                                   
  */
uint16_t PWRT_GetOngoingMaxCurrent(PWRT_Handle_t * pHandle);

/**
  * @brief  Set the ongoing max current we want to push
  * @param  Powertrain handle, the desired max current value
  * @retval current in amps uin16_t                                                                                   
  */
void PWRT_SetOngoingMaxCurrent(PWRT_Handle_t * pHandle, uint16_t aCurrent);

/**
  * @brief  Convert a digital current to a current in AMPs
  * @param  Powertrain handle, the digital current to convert
  * @retval current in amps uin16_t                                                                                   
  */
uint16_t PWRT_ConvertDigitalCurrentToAMPS(PWRT_Handle_t * pHandle, uint16_t aDigitalCurrent);

/**
  * @brief  Convert a current in AMPs to a digital current
  * @param  Powertrain handle, the current in AMPs to convert
  * @retval digital current uin16_t                                                                                   
  */
uint16_t PWRT_ConvertAMPSToDigitalCurrent(PWRT_Handle_t * pHandle, uint16_t aAMPSCurrent);

/**
  * @brief  Setting a new top speed
  * @param  Powertrain handle
  * @param  New top speed in KMH
  * @retval nothing                                                                                
  */
void PWRT_SetNewTopSpeed(PWRT_Handle_t * pHandle, uint16_t topSpeed);

/**
 * @brief  Return the cruise control state
 * @param  Powertrain handle 
 * @retval bool state of the curise control
 */
bool PWRT_GetCruiseControlState(PWRT_Handle_t * pHandle);

/**
 * @brief  Engage the cruise control feature and keep track of the PAS algorithm
 * @param  Powertrain handle, Desired cruise speed 
 * @retval nothing 
 */
void PWRT_EngageCruiseControl(PWRT_Handle_t * pHandle, uint8_t aSpeed);

/**
 * @brief  Disengage the cruise control feature and restore the PAS algorithm
 * @param  Powertrain handle
 * @retval nothing
 */
void PWRT_DisengageCruiseControl(PWRT_Handle_t * pHandle);

/**
 * @brief  Force the disengage the cruise control feature
 *         No matter what the screen tells the controller
 * @param  Powertrain handle
 * @retval nothing
 */
void PWRT_ForceDisengageCruiseControl(PWRT_Handle_t * pHandle);

/**
 * @brief  Get the state of the force disengage flag
 * @param  Powertrain handle
 * @retval bool state of the flag
 */
bool PWRT_GetForceDisengageState(PWRT_Handle_t * pHandle);

/**
 * @brief  Clear the flag when the forced disengage is complete
 * @param  Powertrain handle
 * @retval nothing 
 */
void PWRT_ClearForceDisengage(PWRT_Handle_t * pHandle);

/**
 * @brief  Update the wheel RPM to the MC layer
 * @param  Powertrain handle
 * @retval nothing 
 */
void PWRT_SetWheelRPM(PWRT_Handle_t * pHandle);

/**
 * @brief  Updates the top speed of the screen
 * @param  Powertrain handle
 * @retval nothing 
 */
void PWRT_SetScreenMaxSpeed(PWRT_Handle_t * pHandle, uint8_t aSpeed);

/**
 * @brief  Get the bus voltage
 * @param  Powertrain handle
 * @retval Bus voltage x100 
 */ 
uint16_t PWRT_GetBusVoltagex100();

/**
 * @brief  Get the travelled distance
 * @retval the travelled distance
 */ 
uint32_t PWRT_GetDistanceTravelled();

#endif /*__POWERTRAIN_MANAGEMENT_H*/

