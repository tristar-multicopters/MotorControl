#ifndef MOTOR_SIGNAL_PROCESSING_H_
#define MOTOR_SIGNAL_PROCESSING_H_

/**
*  @file motor_signa_processing.c
*  @brief Middleware layer used to processing 
*         signal comming from the motor.
*/


/*********************************************
                  Includes                       
*********************************************/
#include <stdint.h>
#include "regular_conversion_manager.h"
#include "motor_parameters.h"

/*********************************************
                  Defines
*********************************************/
//tick, on microseconds, used to sampling the mixged signal
//by the processingMotorMixedSignal.
//If the call period og the processingMotorMixedSignal is change,
//this value must to be changed too.
#define ALGORITHM_TICK_US     500

//when initializing the system, this will be the default temperature.
#define INITIAL_TEMP_MOTOR    0x7D00

//the minimum number of values above the minimum temperature raw value.
//this defines wroks as a simple filter to try to reduce the noise
//when the signal goes on low state.
#define TEMPERATURE_MAX_PERIOD_US  1000000    

//max filter index
#define MAX_AVG_FILTER_SIZE        6

/*********************************************
                 Structs
*********************************************/

//struct used to hold all necessary information when 
//processing the signal comming from the motor.
typedef struct
{
    bool adcChannelConfigured;
    bool isMotorMixedSignal;
    RegConv_t mixedSignalRegConv;
    uint8_t mixedSignalIndex;
    uint16_t algorithmTick; 
    uint16_t motorTemperature;
    float wheelSpeedPeriod;
    uint32_t maxWheelSpeedPeriodUs;
    uint16_t minSignalThreshold;
    
}processingMotorSignal_t;


/****************************************************************
                Public Functions 
*****************************************************************/

/**
  @brief Function used to initialize the motor signal.
  @param MotorParameters: inital motor parameters.
  @return void.
*/
void initMotorMixedSignal(MotorParameters_t MotorParameters);
    
/**
  @brief Function used to get, process and extract motor temperature
         and wheel speed the mixed signal comming from the motor.
  @param uint16_t signal original raw signal commign from ADC..
  @return void.
*/

void processingMotorMixedSignal(void);

/**
  @brief Function used to get, process and extract motor temperature
         and wheel speed the mixed signal comming from the motor.
  @param none.
  @return uint16_t raw motor temperature signal.
*/

uint16_t getExtractedMotorTemperature(void);

/**
  @brief Function used to get the mixedSignalRegConv index.
  @param none.
  @return uint8_t mixedSignalIndex.
*/

uint8_t getMixedSignalRegConvIndex(void);

/**
  @brief Function used to get, process and extract wheel speed
         period on micro seconds from the mixed signal comming from the motor.
  @param none.
  @return float wheel speed period on seconds.
*/

float getExtractedWheelSpeed(void);

/**
  @brief Function used to know if the current motor has 
         temperature and wheel speed mixed.
  @param none.
  @return bool true if motor has mixed temperature and wheel speed signal
          false if not.
*/

bool isMotorMixedSignal(void);

/**
  @brief Function used change the flag isMotorMixedSignal.
  @param bool new value, true or false.
  @return none.
*/

void updateisMotorMixedSignalValue(bool value);

/**
  @brief Function used change the value of the minSignalThreshold.
  @param uint16_t value the new value to be used. 
  @return none.
*/

void updateMinSignalThresholdValue(uint16_t value);

/**
  @brief Function used change the value of the MaxWheelSpeedPeriodUs.
  @param uint32_t value the new value to be used and represented micro seconds.
  @return none.
*/

void updateMaxWheelSpeedPeriodUsValue(uint32_t value);

#endif