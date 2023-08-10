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
    
//add pad bytes(zeros) to make the define be a multiple of 4(this is necessary to write correctly in 
//the user data flash). write operation on data flash must be to be multiple of 4.
#define NUMBER_OF_BYTES_MULT_4_TO_BE_WRITTEN  (((USER_DATA_CONFIG_LENGTH%4) == (0)) ? (USER_DATA_CONFIG_LENGTH) : (USER_DATA_CONFIG_LENGTH + 4 - (USER_DATA_CONFIG_LENGTH%4)))
    
//defines used to access cadenceLevelSpeed[] array
//each position in the array is associated with a PAS level
//index 0 PAS level 0 and etc.
#define PAS_0  0
#define PAS_1  1
#define PAS_2  2
#define PAS_3  3
#define PAS_4  4
#define PAS_5  5
#define PAS_6  6
#define PAS_7  7
#define PAS_8  8
#define PAS_9  9

//defines used to access torqueLevelPower[] array
//each position in the array is associated with a torque level
#define TORQUE_LEVEL_0  0
#define TORQUE_LEVEL_1  1
#define TORQUE_LEVEL_2  2
#define TORQUE_LEVEL_3  3
#define TORQUE_LEVEL_4  4
#define TORQUE_LEVEL_5  5
#define TORQUE_LEVEL_6  6
#define TORQUE_LEVEL_7  7
#define TORQUE_LEVEL_8  8
#define TORQUE_LEVEL_9  9


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
    uint8_t torqueMinimumThresholdStartup;    
    uint8_t startupTorqueMinimumThresholdSpeed;
    uint8_t torqueMinimumThreshold;
    uint8_t torqueSensorMultiplier;
    uint8_t torqueMaxSpeed;
    uint8_t cadenceLevelSpeed[10];
    uint8_t torqueLevelPower[10];
    
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
    uint16_t crc;
} User_ConfigData_t;

//ends 1 bytes alignment.
#pragma pack(0)

#endif