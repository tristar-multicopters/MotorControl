/**
  * @file    battery_monitoring.h
  * @brief   This module handles monitoring the battery voltage (also called bus voltage or vbus)
  *          and calculationg an approximate battery state of charge (SOC)
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BAT_MONITOR_H
#define __BAT_MONITOR_H

#include "stdbool.h"
#include "uCAL_GPIO.h"
#include "mc_interface.h"

#define NB_SAMPLES_4_AVG 20 // Obtained through trial and error
                            // Keeps the SOC stable  
/**
  * @brief BatMonitor_Handle_t structure used for battery monitoring
  *
  */
typedef struct
{          
  uint16_t VBatMin; // Value in volts that the battery has when it's empty
  uint16_t VBatMax; // Value in volts that the battery has when it's fully charged
  uint16_t VBatAvg; // Value in volts of the average voltage present in the battery.  
  
  MotorControlInterfaceHandle_t * pMCI;  
    
  uint16_t VBatLog[NB_SAMPLES_4_AVG]; // Used to calculate the last NB_SAMPLES_4_AVG values of VBat to get an average. 
  uint16_t ValCount;                  // Used as a cursor for keep track of where we are in VBatLog
  uint16_t Transition_count;          // Used to determin when the SOC changes
  uint16_t SOC;                       // Contains the state of charge of the battery
  bool StartupDone;                   // Tells us if we have read less than NB_SAMPLES_4_AVG values
  
  bool LowBattery;                    // Flag that when set to true indicates a low battery    
  uint16_t LowBatSOC;                 // Tells us the value of SOC that when reached means we have a low battery
  uint16_t RechargedBatSOC;           // Tells us the upper SOC value where the low battery flag should be cleared                 
    
} BatMonitor_Handle_t;

/**
 * @brief Initializes the battery monitoring module
 * @param pHandle : Pointer on Handle structure of the battery monitoring module
 */
void BatMonitor_Init(BatMonitor_Handle_t * pHandle, MotorControlInterfaceHandle_t *pMCI);

/**
 * @brief Updates the VBat average with a new value
 * @param pHandle : Pointer on Handle structure of the battery monitoring module
 */
void BatMonitor_UpdateAvg(BatMonitor_Handle_t * pHandle);

/**
 * @brief Calculate the SOC based on the voltage average and checks the value over
 *        numerous calls of this function to determine if we need to update the SOC
 * @param pHandle : Pointer on Handle structure of the battery monitoring module
 */
void BatMonitor_ComputeSOC(BatMonitor_Handle_t * pHandle);

/**
 * @brief Updates the SOC of the battery
 *        (simply calls updateAvg and ComputeSOC)
 * @param pHandle : Pointer on Handle structure of the battery monitoring module
 */
void BatMonitor_UpdateSOC(BatMonitor_Handle_t * pHandle);

/**
 * @brief Get the current SOC value
 * @param pHandle : Pointer on Handle structure of the battery monitoring module
 */
uint16_t BatMonitor_GetSOC(BatMonitor_Handle_t * pHandle);

/**
 *  @brief Get the low battery flag
 *  @param pHandle : Pointer on Handle structure of the battery monitoring module
 */
uint16_t BatMonitor_GetLowBatFlag(BatMonitor_Handle_t * pHandle);

#endif /*__BAT_MONITOR_H*/

