/**
  * @file    power_enable.h
  * @brief   This module handles power enable pin
  *
*/
    
    /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __POWER_ENABLE_H
#define __POWER_ENABLE_H

#include "stdbool.h"
#include "uCAL_GPIO.h"
#include "ASSERT_FTEX.h"

// CANOpen includes
#include "co_core.h"
#include "co_gnr2_specs.h"

//#include "comm_config.h"

#define POWEROFFSEQUENCE_FLAG  0x13  // OS flag used to block the power enable task
                                     // TODO move to a dedicated flag file

// value used to check if a firmware update request was made by SDO service.
#define FIRMWARE_UPDATE_REQUEST  0x01

//timeout to wait for the firmware update command.
//Timeout == (FIRMWAREUPDATE_TIMEOUT*25ms) seconds. 
//Timeout == 200*25ms = 5 seconds.
#define FIRMWAREUPDATE_TIMEOUT  200 

//define the sdo timeout(ms) used on the power off sequency.
#define SDO_TIMEOUT             50

//define the maximum number of attempts to send a sdo download service.
#define SDODOWNLOAD_MAXATTEMPTS  2

//define power sequency wait time. use to wait all node be turnned off.
#define POWEROFF_WAITTIME      100

//define the system ready timeout(1500ms).
#define SYSTEMREADY_TIMEOUT   3000

//define IOT sdo donwload msg period(1000ms).
#define IOTMSG_PERIOD         40

typedef struct
{          
    bool bInitalized;       /* Tells use if the PWREN struct was initialized or not.*/
    bool bUsePowerLock;     /* Tells use if this bike uses a power lock */
    bool bIsPowerEnabled;   /* Tells us if the power is currently enable (we can push power) */
    bool bIsInvertedLogic;  /* Tells use if the pin used to check the state of the power is inverted */
    bool bInitialPowerLockState;       /* Tells us who has woke up the GNR*/ 
    bool bWakeUpSDOCommand; /* Tells us if the device was woke up by a SDO wake up command.*/
    bool bWakeUpCommandChecked;
    bool bIotStoped; // Confirm IOT knows the system will turn off.
    bool bSlaveOff; // Confirm Slave knows the system will turn off.
    bool bGoingOff; // flag used to inform to the device the system is going turn off.
    bool bSdoResponseReceived;
    uint8_t bSdoData;//used to send data to IOT/SLAVE.
    uint8_t bSdoAttempts;//used to control the number of attempts when send a sdo download.
    bool bSystemReady;//used to let the system know that the can communication and normal turn off sequency can be done.
    uint16_t bSytemReadyTimeout;//used to control the time necessary to the system be ready to used CANOPEN communication.
    
} PWREN_Handle_t;  

//enum used to control the power off sequency of the module.
typedef enum 
{
  PWREN_IDLE,
  PWREN_STOPIOT,
  PWREN_WAITIOT,
  PWREN_TURNOFFSLAVE,
  PWREN_WAITTURNOFFSLAVE,
  PWREN_TURNOFF,
} PWREN_PowerOffSequencyState_t;

/**
  @brief Function used to initialise the GPIO pins used for the power enable module
  @param pHandle : Pointer on Handle structure of PWREN module
  @return void
*/
void PWREN_Init(PWREN_Handle_t * pHandle);

/**
  @brief Function used to check the state of the Power Enable signal
  @param pHandle : Pointer on Handle structure of PWREN module
  @return bool
*/
bool PWREN_IsPowerEnabled(PWREN_Handle_t * pHandle);

/**
  @brief Function used to monitor the current state of the Power Enable signal 
         and trigger the power off sequence if needed
    
  @param pHandle : Pointer on Handle structure of PWREN module
  @return void
*/
void PWREN_MonitorPowerEnable(PWREN_Handle_t * pHandle);

/**
  @brief Function used to turn off the power to the controller
    
  @param pHandle : Pointer on Handle structure of PWREN module
  @return void
*/
void PWREN_StopPower(PWREN_Handle_t * pHandle);

/**
  @brief Function to check if the firmware update command was received.
  @param PWREN_Handle_t * pHandle pointer to the struct responsible
         to give access to the flags used to know if the device was turned on
         by firmware command received by CAN or was a false wake up.
  @param uint8_t UpdateCommand used to pass the value read from the CAN dictionary 
         address responsible to hold the firmware update command.
  @return void
*/
void PWREN_CheckFirmwareUpdateCommand(PWREN_Handle_t * pHandle, uint8_t UpdateCommand);

/**
  @brief Function used to turn off the slaver motor, if it's present,
         and inform IOT module the device is going to turn off.
  @return void
*/
void PWREN_TurnOffSystem(CO_NODE  *pNode, PWREN_Handle_t * pHandle);

/**
  @brief SDO transfer finalization callback function.
  @param PWREN_Handle_t * pHandle pointer to the struct responsible
         to give access to the flags used to know if the device was turned on
         by firmware command received by CAN or was a false wake up.
  @param CO_CSDO *csdo Pointer to SDO client object
  @param uint16_t index address of the sdo service.
  @param uint8_t sub subindex number of the sdo service.
  @param uint32_t code error code from the SDO service.
  @return void
*/
void IOT_CallbackSDODownloadFinish(CO_CSDO *csdo, uint16_t index, uint8_t sub, uint32_t code);

/**
  @brief SDO transfer finalization callback function used when
         sending sdodownload command to SLAVE module.
  @param PWREN_Handle_t * pHandle pointer to the struct responsible
         to give access to the flags used to know if the device was turned on
         by firmware command received by CAN or was a false wake up.
  @param CO_CSDO *csdo Pointer to SDO client object
  @param uint16_t index address of the sdo service.
  @param uint8_t sub subindex number of the sdo service.
  @param uint32_t code error code from the SDO service.
  @return void
*/
void Slave_CallbackSDODownloadFinish(CO_CSDO *csdo, uint16_t index, uint8_t sub, uint32_t code);

/**
  @brief Function used to set the going off flag.
  @param PWREN_Handle_t * pHandle pointer to the struct responsible
         to give access to the flags used to know if the device was turned on
         by firmware command received by CAN or was a false wake up.
  @return void
*/
void PWREN_SetGoingOffFlag(PWREN_Handle_t * pHandle);

/**
  @brief Function used to get the value of the going off flag.
  @param PWREN_Handle_t * pHandle pointer to the struct responsible
         to give access to the flags used to know if the device was turned on
         by firmware command received by CAN or was a false wake up.
  @return bool return the going off flag value.
*/
bool PWREN_GetGoingOffFlag(PWREN_Handle_t * pHandle);

/**
  @brief Function used to wait the system be ready to start can open communication
         This function set a flag that control the ready or not state of the CANOPEN
         communication and say if the system can do a normal power off sequency or not.
  @param PWREN_Handle_t * pHandle pointer to the struct responsible
         to give access to the flags used to know if the device was turned on
         by firmware command received by CAN or was a false wake up.
  @return void
*/
void PWREN_ManageSystemReadyFlag(CO_NODE  *pNode, PWREN_Handle_t * pHandle);

/**
  @brief Function used to change the IOT OD inform him that GNR is on.
         communication and say if the system can do a normal power off sequency or not.
  @param CO_NODE  *pNode pointer to the CAN node used on the CANOPEN communication.
  @param PWREN_Handle_t * pHandle pointer to the struct responsible
         to give access to the flags used to know if the device was turned on
         by firmware command received by CAN or was a false wake up.
  @return void
*/
void PWREN_SetIotSystemIsOn(CO_NODE  *pNode, PWREN_Handle_t * pHandle);

/**
  @brief Function used to get the system ready flag.
  @param PWREN_Handle_t * pHandle pointer to the struct responsible
         to give access to the flags used to know if the device was turned on
         by firmware command received by CAN or was a false wake up.
  @return bool return the system ready value.
*/
bool PWREN_GetSystemReadyFlag(PWREN_Handle_t * pHandle);

/**
  @brief SDO transfer finalization callback function used when
         sending sdodownload command to inform OIT module
         that the GNR is on.
  @param PWREN_Handle_t * pHandle pointer to the struct responsible
         to give access to the flags used to know if the device was turned on
         by firmware command received by CAN or was a false wake up.
  @param CO_CSDO *csdo Pointer to SDO client object
  @param uint16_t index address of the sdo service.
  @param uint8_t sub subindex number of the sdo service.
  @param uint32_t code error code from the SDO service.
  @return void
*/
void GnrOn_CallbackSDODownloadFinish(CO_CSDO *csdo, uint16_t index, uint8_t sub, uint32_t code);

#endif /*__POWER_ENABLE_H*/

