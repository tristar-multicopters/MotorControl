/**
  ******************************************************************************
  * @file    drivetrain_management.h
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles management of the vehicle drivetrain
  *
	******************************************************************************
	*/
	
	/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRIVETRAIN_MANAGEMENT_H
#define __DRIVETRAIN_MANAGEMENT_H

#include "md_interface.h"
#include "throttle.h"
#include "pedal_assist.h"
#include "brake.h"
#include "motor_selection.h"
#include "vc_defines.h"
#include "foldback.h"
#include "power_enable.h"
#include "torque_sensor.h"
#include "speed_pulse_read.h"
#include "wheel_speed_sensor.h"

typedef enum
{
	HUB_SINGLE,
	HUB_DUAL,
	MIDDRIVE,
} DRVT_Type_h;

typedef enum
{
	SINGLE_MOTOR,
	DUAL_MOTOR,
} Motor_Mode_t;

typedef enum
{
	TORQUE_CTRL,
	SPEED_CTRL,
} CTRL_Type_h;

typedef struct
{
	DRVT_Type_h bDrivetrainType;	  /* Vehicle drivetrain type (i.e. hub, middrive, ...) */
	Motor_Mode_t bMode;						  /* Single or dual motor. It is updated by user using motor selector switch */
	uint8_t bMainMotor;						  /* Main motor selection. It is updated by user using motor selector switch */
	uint8_t bDefaultMainMotor;		  /* Default main motor selection */
	CTRL_Type_h bCtrlType;				  /* Torque or speed control */
	bool bUseMotorM1;							  /* To set once, true if motor 1 is used */
	bool bUseMotorM2;							  /* To set once, true if motor 2 is used */
	
	int16_t aTorque[2];						  /* Array of torque reference, first element is for M1, second is for M2 */
	int16_t aSpeed[2];						  /* Array of speed reference, first element is for M1, second is for M2 */
	
	MDI_Handle_t * pMDI;					  /* Pointer to MDI handle */
	THRO_Handle_t * pThrottle;		  /* Pointer to throttle handle */
	PAS_Handle_t * pPAS;					  /* Pointer to PAS handle */
	BRK_Handle_t * pBrake;				  /* Pointer to brake handle */
	MS_Handle_t * pMS;						  /* Pointer to motor selector handle */
	PWREN_Handle_t * pPWREN;			  /* Pointer to power enable pin handle */
	WSS_Handle_t 	* pWSS;				    /* Pointer to Wheel speed handle */

	int16_t			 	pRefTorque;			/* Torque reference, first element is for M1, second is for M2 */
	int16_t			 	pTorqueSelect;	/* Torque reference, first element is for M1, second is for M2 */
	bool					bUsePAS;
		
	FLDBK_Handle_t sHeatsinkTempFoldback1;		/* Foldback handle using M1 heatsink temperature */
	FLDBK_Handle_t sHeatsinkTempFoldback2;		/* Foldback handle using M2 heatsink temperature */
	FLDBK_Handle_t sMotorTempFoldback1;				/* Foldback handle using M1 motor temperature */
	FLDBK_Handle_t sMotorTempFoldback2;				/* Foldback handle using M2 motor temperature */
	FLDBK_Handle_t sSpeedFoldback1;						/* Foldback handle using M1 speed */
	FLDBK_Handle_t sSpeedFoldback2;						/* Foldback handle using M2 speed */
	FLDBK_Handle_t sDCVoltageFoldback;				/* Foldback handle using DCbus voltage */
	
	uint16_t hSpeedRampTimeUp;			/* Speed ramp time in millisecond when controller is ramping UP */
	uint16_t hSpeedRampTimeDown;		/* Speed ramp time in millisecond when controller is ramping DOWN */
	uint16_t hTorqueRampTimeUp;		  /* Torque ramp time in millisecond when controller is ramping UP */
	uint16_t hTorqueRampTimeDown;		/* Torque ramp time in millisecond when controller is ramping DOWN */
	uint16_t hStartingThrottle;		  /* Minimum torque to start drivetrain */
	uint16_t hStoppingThrottle;		  /* Minimum torque to stop drivetrain */
	uint16_t hStoppingSpeed;			  /* Minimum speed to stop drivetrain */
	
	// Fault handlers //
	uint16_t hOCcounter[2];   			/* Over current counters. First element would	be for M1, second for M2 */
	uint16_t hSUcounter[2];   			/* Start-up counters. First element would	be for M1, second for M2     */
	uint16_t hSFcounter[2];   			/* Speed feedback current counters. First element would	be for M1, second for M2*/
	uint8_t fault_timeout; 					/* Number of times the VC state machine should be stayed on fault
																		 before clear a Over current, start-up or Speed back fault */
} DRVT_Handle_t;

/**
	* @brief  Module initialization, to be called once before using it
	* @param  Drivetrain handle
	* @retval None
	*/
void DRVT_Init(DRVT_Handle_t * pHandle);

/**
	* @brief  Compute target torque and speed to be applied to each motors. To be called periodically.
	* @param  Drivetrain handle
	* @retval None
	*/
void DRVT_CalcTorqueSpeed(DRVT_Handle_t * pHandle);

/**
	* @brief  Send torque and/or speed ramp commands to motors
	* @param  Drivetrain handle
	* @retval None
	*/
void DRVT_UpdateMotorRamps(DRVT_Handle_t * pHandle);

/**
	* @brief  Send command to start motors
	* @param  Drivetrain handle
	* @retval None
	*/
void DRVT_StartMotors(DRVT_Handle_t * pHandle);

/**
	* @brief  Send command to stop motors
	* @param  Drivetrain handle
	* @retval None
	*/
void DRVT_StopMotors(DRVT_Handle_t * pHandle);

/**
	* @brief  Check for motor faults during standby state
	* @param  Drivetrain handle
	* @retval Vehicle fault code
	*/
uint16_t DRVT_StandbyStateCheck(DRVT_Handle_t * pHandle);

/**
	* @brief  Check for motor faults during start state
	* @param  Drivetrain handle
	* @retval Vehicle fault code
	*/
uint16_t DRVT_StartStateCheck(DRVT_Handle_t * pHandle);

/**
	* @brief  Check for motor faults during run state
	* @param  Drivetrain handle
	* @retval Vehicle fault code
	*/
uint16_t DRVT_RunStateCheck(DRVT_Handle_t * pHandle);

/**
	* @brief  Check for motor faults during stop state
	* @param  Drivetrain handle
	* @retval Vehicle fault code
	*/
uint16_t DRVT_StopStateCheck(DRVT_Handle_t * pHandle);

/**
	* @brief  Check if drivetrain is active (e.g. motors are in running state)
	* @param  Drivetrain handle
	* @retval Returns true if drivetrain is active
	*/
bool DRVT_IsDrivetrainActive(DRVT_Handle_t * pHandle);

/**
	* @brief  Check if drivetrain is stopped (e.g. motors are in idle state)
	* @param  Drivetrain handle
	* @retval Returns true if drivetrain is stopped
	*/
bool DRVT_IsDrivetrainStopped(DRVT_Handle_t * pHandle);

/**
	* @brief  Check if conditions to stop drivetrain are met
	* @param  Drivetrain handle
	* @retval Returns true if drivetrain can be stopped
	*/
bool DRVT_CheckStopConditions(DRVT_Handle_t * pHandle);

/**
	* @brief  Check if conditions to start drivetrain are met
	* @param  Drivetrain handle
	* @retval Returns true if drivetrain can be started
	*/
bool DRVT_CheckStartConditions(DRVT_Handle_t * pHandle);

/**
	* @brief  Manage motor faults. Check if faults are still present and send motor fault acknowledge when faults are gone.
	* @param  Drivetrain handle
	* @retval Returns true if a motor fault is still active, false if no more fault is present.
	*/
bool DRVT_MotorFaultManagement(DRVT_Handle_t * pHandle);

/**
	* @brief  Set PAS level
	* @param  Drivetrain handle
	* @param  PAS level
	* @retval None
	*/
PAS_sLevel DRVT_SetPASLevel(DRVT_Handle_t * pHandle, PAS_sLevel level);
/**
	* @brief  Get main motor reference torque
	* @param  Drivetrain handle
	* @retval Returns main motor reference torque
	*/
int16_t DRVT_GetTorqueRefMainMotor(DRVT_Handle_t * pHandle);

/**
	* @brief  Get motor 1 reference torque
	* @param  Drivetrain handle
	* @retval Returns motor 1 reference torque
	*/
int16_t DRVT_GetTorqueRefM1(DRVT_Handle_t * pHandle);

/**
	* @brief  Get motor 2 reference torque
	* @param  Drivetrain handle
	* @retval Returns motor 2 reference torque
	*/
int16_t DRVT_GetTorqueRefM2(DRVT_Handle_t * pHandle);

/**
	* @brief  Get main motor speed reference
	* @param  Drivetrain handle
	* @retval Returns main motor speed reference
	*/
int32_t DRVT_GetSpeedRefMainMotor(DRVT_Handle_t * pHandle);

/**
	* @brief  Get motor 1 speed reference
	* @param  Drivetrain handle
	* @retval Returns motor 1 speed reference
	*/
int32_t DRVT_GetSpeedRefM1(DRVT_Handle_t * pHandle);

/**
	* @brief  Get motor 2 speed reference
	* @param  Drivetrain handle
	* @retval Returns motor 2 speed reference
	*/
int32_t DRVT_GetSpeedRefM2(DRVT_Handle_t * pHandle);

/**
	* @brief  Check if motor 1 is used
	* @param  Drivetrain handle
	* @retval Returns true if motor 1 is used
	*/
bool DRVT_IsMotor1Used(DRVT_Handle_t * pHandle);

/**
	* @brief  Check if motor 2 is used
	* @param  Drivetrain handle
	* @retval Returns true if motor 2 is used
	*/
bool DRVT_IsMotor2Used(DRVT_Handle_t * pHandle);

/**
	* @brief  Set Pedal Assist Level based on the screen information
	* @param  Drivetrain handle
	* @retval pRefTorque in int16
	*/
int16_t DRVT_PasSetLevel(DRVT_Handle_t * pHandle);
/**
	* @brief  Select Control assistance based on Throttle or PAS
	* @param  Drivetrain handle
	* @retval RefTorque in int16                                                                                    
	*/
int16_t DRVT_ControlSelect(DRVT_Handle_t * pHandle);
/**
	* @brief  Set Pedal Assist Level based on the screen information
	* @param  Drivetrain handle
	* @retval pRefTorque in int16
	*/
bool DRVT_PASpresence (DRVT_Handle_t * pHandle);

#endif /*__DRIVETRAIN_MANAGEMENT_H*/

