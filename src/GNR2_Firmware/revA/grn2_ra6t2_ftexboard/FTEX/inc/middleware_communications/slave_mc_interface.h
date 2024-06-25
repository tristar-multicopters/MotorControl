/**
  * @file    slave_mc_interface.h
  * @author  Sami Bouzid, FTEX
  * @brief   Module that provides an interface to control a slave motor with CANOpen.
                Slave is a motor controller not present locally in this ganrunner device.
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SLAVE_MC_INTERFACE_H
#define __SLAVE_MC_INTERFACE_H

#include "mc_interface.h"
#include "co_core.h"
#include "vc_state_machine.h"
#include "gnr_parameters.h"

/************* DEFINES ****************/


/**************************************/


/*
*  Structure used to hold feedback from slave motor
*/
typedef struct
{
    MotorState_t bState;
    uint32_t wOccurredFaults;
    uint32_t wCurrentFaults;
    uint32_t wCurrentErrorsNow;
    uint32_t wOccurredErrors;
    uint32_t wWarnings;
    int16_t hMotorSpeed;
    int16_t hMotorTemp;
    int16_t hHeatsinkTemp;
    uint16_t hBusVoltage;
    int16_t hIqRef;
    int16_t hIdRef;
} SlaveMotorFeedback_t;

/*
*  Structure used to hold register addresses of slave motor. Use CO_DEV() macro with index and subindex.
*/
typedef struct
{
    uint32_t wRegAddrState;             // Object dictionnary address to get slave motor state
    uint32_t wRegAddrCurrentFaults;     // Object dictionnary address to get slave current faults
    uint32_t wRegAddrOccurredFaults;    // Object dictionnary address to get slave occurred faults   
    uint32_t wRegAddrCurrentErrorsNow;  // Object dictionnary address to get slave errors now   
    uint32_t wRegAddrOccurredErrors;    // Object dictionnary address to get slave errors occurred
    uint32_t wRegAddrWarnings;          // Object dictionnary address to get slave warnings   
    uint32_t wRegAddrMotorSpeed;        // Object dictionnary address to get slave motor speed
    uint32_t wRegAddrMotorTemp;         // Object dictionnary address to get slave motor temperature
    uint32_t wRegAddrHeatsinkTemp;      // Object dictionnary address to get slave heatsink temperature
    uint32_t wRegAddrBusVoltage;        // Object dictionnary address to get slave bus voltage
    uint32_t wRegAddrIq;                // Object dictionnary address to get slave Iq
    uint32_t wRegAddrId;                // Object dictionnary address to get slave Id
    uint32_t wRegAddrStartMotor;        // Object dictionnary address to write slave motor start bit
    uint32_t wRegAddrTorqueRef;         // Object dictionnary address to write slave motor torque reference
    uint32_t wRegAddrFaultAck;          // Object dictionnary address to write slave motor fault acknowledge bit
} SlaveMotorRegisterAddr_t;


/*
*  Structure used to hold data of a slave motor controller,
    i.e. motor controller not locally present on this ganrunner device.
*/
typedef struct
{
    CO_NODE *pCONode;
    
    SlaveMotorFeedback_t Feedback;
    SlaveMotorRegisterAddr_t RegisterAddr;
} SlaveMotorHandle_t;


/**
  * @brief  This function initialize the slave motor interface module.
  * @param  pHandle Pointer on the component instance to operate on.
  * @param  pNode Pointer on the CANOpen node to operate on.
  * @param  RegisterAddr Structure holding the CANopen register addresses. See SlaveMotorRegisterAddr_t definition.
  * @retval none.
  */
void SlaveMCInterface_Init(SlaveMotorHandle_t * pHandle, CO_NODE * pNode, SlaveMotorRegisterAddr_t RegisterAddr);

/**
  * @brief  This function is used to update the motor handle feedback by reading object dictionnary
  * @param  pHandle Pointer on the component instance to operate on.
  * @retval none.
  */
void SlaveMCInterface_UpdateFeedback(SlaveMotorHandle_t * pHandle);

/**
  * @brief  This command write to the object dictionnary a new reference torque. 
  *          The data must then be sent periodically (e.g. by PDO) to the slave.
  * @param  pHandle Pointer on the component instance to work on.
  * @param  hFinalTorque is the value of motor torque reference at the end of
  *         the ramp in cNm (Nm/100).
  * @retval none.
  */
void SlaveMCInterface_ExecTorqueRamp(SlaveMotorHandle_t * pHandle, int16_t hFinalTorque);


/**
  * @brief  This command write in the object dictionnary a boolean "true" to start the slave motor. 
  *          The data must then be sent periodically (e.g. by PDO) to the slave.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval bool It returns true if the command is successfully executed
  *         otherwise it return false.
  */
bool SlaveMCInterface_StartMotor(SlaveMotorHandle_t * pHandle);

/**
  * @brief  This command write in the object dictionnary a boolean "false" to stop the slave motor. 
  *          The data must then be sent periodically (e.g. by PDO) to the slave.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval bool It returns true if the command is successfully executed
  *         otherwise it return false.
  */
bool SlaveMCInterface_StopMotor(SlaveMotorHandle_t * pHandle);

/**
  * @brief  This command send an SDO to acknowledge faults in slave motor.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval bool It returns true if the command is successfully executed
  *         otherwise it return false.
  */
bool SlaveMCInterface_CriticalFaultAcknowledged(SlaveMotorHandle_t * pHandle);

/**
  * @brief  It returns information about the state of the related pSTM object.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval MotorState_t It returns the current state of the related pSTM object.
  */
MotorState_t  SlaveMCInterface_GetSTMState(SlaveMotorHandle_t * pHandle);

/**
  * @brief It returns a 16 bit fields containing information about critical faults
  *        historically occurred since the state machine has been moved into
  *        FAULT_NOW state.
  * \n\link Fault_generation_error_codes Returned critical fault codes are listed here \endlink
  * @param pHandle Pointer on the component instance to work on.
  * @retval uint16_t  16 bit fields with information about the critical faults
  *         historically occurred since the state machine has been moved into
  *         FAULT_NOW state.
  * \n\link Fault_generation_error_codes Returned critical fault codes are listed here \endlink
  */
uint32_t SlaveMCInterface_GetOccurredCriticalFaults(SlaveMotorHandle_t * pHandle);

/**
  * @brief It returns a 16 bit fields containing information about current errors
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
  * @param pHandle Pointer on the component instance to work on.
  * @retval uint16_t  16 bit fields with information about the errors
  */
uint32_t SlaveMCInterface_GetCurrentErrors(SlaveMotorHandle_t * pHandle);

/**
  * @brief It returns a 16 bit fields containing information about errors
  *        historically occurred 
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
  * @param pHandle Pointer on the component instance to work on.
  * @retval uint16_t  16 bit fields with information about the errors
  */
uint32_t SlaveMCInterface_GetOccurredErrors(SlaveMotorHandle_t * pHandle);

/**
  * @brief It returns a 16 bit fields containing information about warnings
  * \n\link Fault_generation_error_codes Returned warning codes are listed here \endlink
  * @param pHandle Pointer on the component instance to work on.
  * @retval uint16_t  16 bit fields with information about the warnings
  */
uint32_t SlaveMCInterface_GetOccurredWarnings(SlaveMotorHandle_t * pHandle);

/**
  * @brief It returns a 16 bit fields containing information about faults
  *        currently present.
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
  * @param pHandle Pointer on the component instance to work on.
  * @retval uint16_t  16 bit fields with information about about currently
  *         present faults.
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
  */
uint32_t SlaveMCInterface_GetCurrentCriticalFaults(SlaveMotorHandle_t * pHandle);

/**
  * @brief  Returns the last computed average mechanical speed, expressed in
  *         the unit defined by #SPEED_UNIT and related to the sensor actually
  *         used by FOC algorithm
  * @param  pHandle Pointer on the component instance to work on.
  */
int16_t SlaveMCInterface_GetAvrgMecSpeedUnit(SlaveMotorHandle_t * pHandle);

/**
  * @brief  Returns the Iq of the slave
  * @param  pHandle Pointer on the component instance to work on.
  */
int16_t SlaveMCInterface_GetIqRef(SlaveMotorHandle_t * pHandle);

/**
  * @brief  Returns the Id of the slave
  * @param  pHandle Pointer on the component instance to work on.
  */
int16_t SlaveMCInterface_GetIdRef(SlaveMotorHandle_t * pHandle);

#endif /* __SLAVE_MC_INTERFACE_H */
