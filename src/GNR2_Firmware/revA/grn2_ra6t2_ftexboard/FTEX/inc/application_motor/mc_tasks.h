/**
  * @file    mc_tasks.h
  * @brief   This file implementes tasks definition.
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MCTASKS_H
#define __MCTASKS_H

/* Includes ------------------------------------------------------------------*/
#include "mc_tuning.h"
#include "mc_config.h"

typedef enum
{
    MCI_BUFFER_EMPTY,                  /*!< If no buffered command has been
                                            called.*/
    MCI_COMMAND_NOT_ALREADY_EXECUTED,  /*!< If the buffered command condition
                                            hasn't already occurred.*/
    MCI_COMMAND_EXECUTED_SUCCESFULLY,  /*!< If the buffered command has been
                                            executed successfully.*/
    MCI_COMMAND_EXECUTED_UNSUCCESFULLY /*!< If the buffered command has been
                                            executed unsuccessfully.*/
} MCInterfaceCommandState_t ;

typedef enum
{
    MCI_NOCOMMANDSYET,        /*!< No command has been set by the user.*/
    MCI_EXECSPEEDRAMP,        /*!< ExecSpeedRamp command coming from the user.*/
    MCI_EXECTORQUERAMP,       /*!< ExecTorqueRamp command coming from the user.*/
    MCI_SETCURRENTREFERENCES, /*!< SetCurrentReferences command coming from the
                                 user.*/
} MCInterfaceUserCommands_t;

typedef struct
{
    SpdTorqCtrlHandle_t * pSpeedTorqCtrl;         /*!< Speed and torque controller object used by MCI.*/
    pFOCVars_t pFOCVars;                          /*!< Pointer to FOC vars used by MCI.*/
    ResDivVbusSensorHandle_t  *pResDivVbusSensor; /*!< Used to raise the resistor dividor bus voltage sensor to the vehicle layer*/
    MCInterfaceUserCommands_t LastCommand;        /*!< Last command coming from the user.*/
    MCConfigHandle_t          *pMCConfig;

    int16_t hFinalSpeed;        /*!< Final speed of last ExecSpeedRamp command.*/
    int16_t hFinalTorque;       /*!< Final torque of last ExecTorqueRamp command.*/
                                   
    qd_t Iqdref;                /*!< Current component of last
                                   SetCurrentReferences command.*/

    bool bDriverEn;             /*!< Status of Driver Enable pin */

    MCInterfaceCommandState_t CommandState; /*!< The status of the buffered command.*/
    STCModality_t LastModalitySetByUser;    /*!< The last STCModality_t set by the user. */
                                             
} MotorControlInterfaceHandle_t;


extern MotorControlInterfaceHandle_t MCInterface[NBR_OF_MOTORS];

/**
  * @brief  Initializes the Motor subsystem core according to user defined parameters.
    * @param    None
    * @retval None
  */
void MC_BootUp(void);


/**
  * @brief  Runs motor control tasks depending on motor state, such as speed control, current reference calculation, etc.
    * @param    None
    * @retval None
  */
void MC_RunMotorControlTasks(void);


/**
  * @brief  Executes the Medium Frequency Task functions for each drive instance.
    * @param    None
    * @retval None
  */
void MC_Scheduler(void);


/**
  * @brief  Executes safety checks (e.g. bus voltage and temperature) for all drive instances
    * @param    None
    * @retval None
  */
void MC_SafetyTask(void);


/**
  * @brief  Executes the Motor Control duties that require a high frequency rate and a precise timing.
    * @param    None
    * @retval None
  */
uint8_t MC_HighFrequencyTask(void);


/**
  * @brief  Reserves FOC execution on ADC ISR half a PWM period in advance
    * @param    None
    * @retval None
  */
void MC_DualDriveFIFOUpdate(uint8_t Motor);


/**
  * @brief  Puts the Motor Control subsystem in in safety conditions on a Hard Fault.
    * @param    None
    * @retval None
  */
void MC_HardwareFaultTask(void);


/**
  * @brief  Turns off the PWM for M1
    * @param    None
    * @retval None
  */
void MC_PWM_OFF_M1(void);

#endif /* __MCTASKS_H */
