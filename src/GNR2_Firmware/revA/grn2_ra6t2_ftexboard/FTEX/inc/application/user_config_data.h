#ifndef USER_CONFIG_DATA_H_
#define USER_CONFIG_DATA_H_

/*********************************************
                  Includes                       
*********************************************/

#include <stdint.h>

/*********************************************
                  Defines
*********************************************/

//define the length on bytes of the User_ConfigData_t.
#define USER_DATA_CONFIG_LENGTH   sizeof(User_ConfigData_t)
    
//add pad bytes(zeros) to make the define be a multiple of 4(this is necessary to write correctly in 
//the user data flash). write operation on data flash must be to be multiple of 4.
#define NUMBER_OF_BYTES_MULT_4_TO_BE_WRITTEN  (((USER_DATA_CONFIG_LENGTH%4) == (0)) ? (USER_DATA_CONFIG_LENGTH) : (USER_DATA_CONFIG_LENGTH + 4 - (USER_DATA_CONFIG_LENGTH%4)))
    
//defines used to access PasLevelSpeed[] array
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

//
#define BW_ARRAY_SIZE        3

#define FILTERSPEED_ARRAY_SIZE       (BW_ARRAY_SIZE - 1)

#define DIGITAL33_0_7_VOLTS 13900 // Value of 0.7 volts when you have a 0-65535 digital value where 65535 = 3.3V
#define DIGITAL5_0_8_VOLTS  10500 // Value of 0.8 volts when you have a 0-65535 digital value where 65535 = 5.0V

//it need to be updated if this struct is modified : PasAlgorithm_t
#define MAX_PASALGORITHM_VALUE    4

/*********************************************
            Data Struct Definition
*********************************************/

//force compiler to use 1 byte packaging
//avoid padding bytes.
#pragma pack(1)

typedef struct
{
    uint16_t pasLowPassFilterBW1[BW_ARRAY_SIZE];
    uint16_t pasLowPassFilterBW2[BW_ARRAY_SIZE];
    uint8_t  FilterSpeed[FILTERSPEED_ARRAY_SIZE];
}PAS_Torque_Filter_Configuration_t;

typedef struct
{
    uint16_t pasPedalRPM;           // PlaceHolder Currently not supported          
    uint8_t  pasTorqueForcePercent; // PlaceHolder Currently not supported  
    uint16_t pasTorqueForceWatts;   // PlaceHolder Currently not supported  
    uint8_t  pasNbMagnetsPerTurn;      
    uint16_t pasTorqueInputMin;  
    uint16_t pasTorqueInputMax;
} PAS_Sensor_Configuration_t;

typedef struct
{
    uint8_t pasTorqueStartupSpeed;
    uint8_t pasTorqueStartupThreshold;    
    uint8_t pasCadenceStartupNumbPulses;
    uint16_t pasCadenceStartupWindows;
    uint8_t PasAlgorithmStartup;
} PAS_Startup_Detection_t;

typedef struct
{
    uint8_t pasTorqueRunningThreshold;    
    uint8_t pasCadenceRunningNumbPulses;
    uint16_t pasCadenceRunningWindows;
    uint8_t PasAlgorithmRunning;
} PAS_Running_Detection_t;


//struct used to hold the configuration
//associated with PAS(pedal Assist System).
typedef struct 
{
    uint8_t pasAlgorithm;
    uint8_t numberOfPasLevels;
    uint8_t pasMaxTorqueRatio;
    PAS_Startup_Detection_t PAS_Startup_Detection;
    PAS_Running_Detection_t PAS_Running_Detection;
    uint16_t torqueSensorMultiplier[10];
    uint8_t PasLevelSpeed[10];
    uint8_t pasLevelMaxTorque[10];
    uint8_t pasLevelMinTorque[10];
    PAS_Torque_Filter_Configuration_t PAS_Torque_Filter_Configuration;
    PAS_Sensor_Configuration_t PasSensorConfig;
    
} PAS_ConfigData_t;

//struct used to hold the configuration
//associated with Throttle.
typedef struct
{
    uint16_t AdcOffset;
    uint16_t AdcMax; 
    uint8_t  ThrottleBlock;
    uint8_t  MaxSpeed;
} Throttle_ConfigData_t;

typedef struct
{
    uint16_t FullVoltage;                 // In Volts x100
    uint16_t EmptyVoltage;                // In Volts x100
    uint16_t MaxPeakDCCurrent;            // In Amps x100
    uint16_t ContinuousDCCurrent;         // in Amps x100
    uint16_t PeakCurrentMaxDuration;      // In Seconds x10
    uint16_t PeakCurrentDeratingDuration; // In seconds x10
} Battery_ConfigData_t;



//struct used to hold the configuration
//associated with the Vehicle.
typedef struct
{
    uint8_t walkModeSpeed;
    uint8_t maxSpeed;
    uint8_t WheelDiameter;
    uint8_t ScreenProtocol;
    uint8_t HeadLightDefault;      // Contains the default state of the head light (on or off)
    uint8_t TailLightDefault;      // Contains the default state of the tail light (on or off)      
    uint8_t TailLightBlinkOnBrake;    
    Throttle_ConfigData_t Throttle_ConfigData;
} Screen_ConfigData_t;

//struct used to hold all structs associated 
//with user configuration data.
typedef struct 
{
    uint8_t dataHeader[2];
    uint8_t vehicle;
    PAS_ConfigData_t      PAS_ConfigData;
    Battery_ConfigData_t  Battery_ConfigData; 
    Screen_ConfigData_t   Screen_ConfigData;
    uint16_t crc;
} User_ConfigData_t;

//ends 1 bytes alignment.
#pragma pack(0)

#endif