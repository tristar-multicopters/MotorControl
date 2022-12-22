#ifndef USER_CONFIG_DATA_H_
#define USER_CONFIG_DATA_H_

/*********************************************
                  Includes                       
*********************************************/

#include <stdint.h>

/*********************************************
                  Defines
*********************************************/

//define the lenght on bytes of the User_ConfigData_t.
#define USER_DATA_CONFIG_LENGTH   sizeof(User_ConfigData_t)

/*********************************************
            Data Struct Definition
*********************************************/

//force compiler to use 1 byte packaging
//avoid padding bytes.
#pragma pack(1)

//struct used to hold the configuration
//associated with PAS(pedal Assist System).
typedef struct 
{
    uint8_t pasAlgorithm;
    uint8_t numberOfPasLevels;
    uint8_t pasMaxPower;
    uint8_t torqueMinimumThreshold;
    uint8_t torqueSensorMultiplier;
    uint8_t cadenceHybridLeve1Speed;
    uint8_t cadenceHybridLeve2Speed;
    uint8_t cadenceHybridLeve3Speed;
    uint8_t cadenceHybridLeve4Speed;
    uint8_t cadenceHybridLeve5Speed;
    uint8_t cadenceHybridLeve6Speed;
    uint8_t cadenceHybridLeve7Speed;
    uint8_t cadenceHybridLeve8Speed;
    uint8_t cadenceHybridLeve9Speed;
    uint8_t cadenceHybridLeve10Speed;
} PAS_ConfigData_t;

//struct used to hold the configuration
//associated with Throttle.
typedef struct
{
    uint8_t maxSpeed;
    uint8_t walkModeSpeed;
} Throttle_ConfigData_t;

//struct used to hold all structs associated 
//with user configuration data.
typedef struct 
{
    uint8_t dataHeader[2];
    uint8_t vehicle;
    PAS_ConfigData_t PAS_ConfigData;
    Throttle_ConfigData_t Throttle_ConfigData;
} User_ConfigData_t;

//ends 1 bytes alignment.
#pragma pack(0)

#endif