/**
*  co_gnr2_specs.c
*  Module for defining the CANOpen specs of GNR2
*/

// ==================== INCLUDES ======================== //

#include "co_gnr2_specs.h"
#include "gnr_parameters.h"
#include "pedal_assist.h"

#include "co_can_ra6t2.h"           /* CAN driver                  */
#include "co_timer_ra6t2.h"         /* Timer driver                */
#include "co_nvm_ra6t2.h"           /* NVM driver                  */

// ==================== PRIVATE DEFINES ======================== //

//by defaul make master id the node id.
#define GNR2_NODE_ID       GNR2_MASTER_NODE_ID          /* CANopen node ID             */

#define GNR2_BAUDRATE      500000u             /* CAN baudrate                */
#define GNR2_TMR_N         16u                 /* Number of software timers   */
#define GNR2_TICKS_PER_SEC 2000u               /* Timer clock frequency in Hz */
#define GNR2_OBJ_N         512u                /* Object dictionary max size  */

//Master configuration - used by a master
#define MASTER_GENERATE_SYNC    1           /* Generate or not the SYNC frame. 1 for yes, 0 for no. */
#define STD_ID_SYNC             0x80        /* SYNC frame standard ID */
#define MASTER_SYNC_PERIOD_US   50000       /* SYNC frame period in us */

#define USE_EMCY                0           /* Use emergency frame */
#define STD_ID_EMCY             0           /* Emergency frame standard ID */

//increased heartbeat period on produced side
//to syncronize power off sequency.
#define HEARTBEAT_PRODUCE_PERIOD_MS         2500                     /* Period of the heartbeat frame */
#define HEARTBEAT_CONSUME_PERIOD_MS         500                     /* Max time to wait for receiving next heartbeat */  

#define USE_RPDO                1           /* Use or not RPDO. 1 for yes, 0 for no. */
#define MASTER_STD_ID_RPDO1     0x300       /* Standard ID of RPDO 1 */
#define MASTER_STD_ID_RPDO2     0x301       /* Standard ID of TPDO 2 */
#define MASTER_RPDO_TRANSMISSION_TYPE  254   /* RPDO transmission type. Event-driven (manufacturer-specific) is chosen */
#define MASTER_RPDO_PERIOD_MS          50          /* Period in ms for RPDO processing */

#define USE_TPDO                1           /* Use or not TPDO. 1 for yes, 0 for no. */
#define MASTER_STD_ID_TPDO1     0x400        /* Standard ID of TPDO 1 */
#define MASTER_TPDO_TRANSMISSION_TYPE  254         /* TPDO transmission type. Event-driven (manufacturer-specific) is chosen */
#define MASTER_TPDO_PERIOD_MS          50          /* Period in ms for TPDO transmission */

#define USE_SSDO                1           /* Use or not SSDO. 1 for yes, 0 for no. */
#define USE_CSDO                1           /* Use or not CSDO. 1 for yes, 0 for no. */

//Slave configuration - used by a slave.
#define SLAVE_GENERATE_SYNC           0           /* Generate or not the SYNC frame. 1 for yes, 0 for no. */
#define SLAVE_SYNC_PERIOD_US          0           /* SYNC frame period in us */

#define SLAVE_STD_ID_RPDO1            0x400        /* Standard ID of RPDO 1 */
#define SLAVE_RPDO_TRANSMISSION_TYPE  0           /* RPDO transmission type. Synchronous after SYNC is chosen */
#define SLAVE_RPDO_PERIOD_MS          0           /* Period in ms for RPDO processing. Not applicable for synchronous transmission type */

#define SLAVE_STD_ID_TPDO1      0x300        /* Standard ID of TPDO 1 */
#define SLAVE_TPDO_TRANSMISSION_TYPE  1           /* Period in ms for TPDO transmission. Not applicable for synchronous transmission type */
#define SLAVE_TPDO_PERIOD_MS          0           /* Period in ms for RPDO processing. Not applicable for synchronous transmission type� */


// ==================== PRIVATE VARIABLES ======================== //

//array used to hold the MASTER + IOT OD configuration.
const uint16_t masterIotSetup[MASTER_IOT_SIZE] = {
CO_OD_REG_DEVICE_TYPE, 
CO_OD_REG_ERROR, 
CO_OD_REG_SYNC_MESSAGE,
CO_OD_REG_SYNC_PERIOD, 
CO_OD_REG_EMCY_MSG, 
CO_OD_REG_HB_TIME, 
CO_OD_IDENTITY_OBJECT, 
CO_OD_SDO_SERVER,
CO_OD_SDO_CLIENT_01, 
CO_OD_SDO_CLIENT_02,
CO_OD_COMMUM_ENTRIES};

//array used to hold the MASTER + IOT + SLAVE OD configuration.
const uint16_t masterIotSlaveSetup[MASTER_IOT_SLAVE_SIZE] = {
CO_OD_REG_DEVICE_TYPE, 
CO_OD_REG_ERROR, 
CO_OD_REG_SYNC_MESSAGE,
CO_OD_REG_SYNC_PERIOD,
CO_OD_REG_EMCY_MSG,
CO_OD_REG_HB_TIME, 
CO_OD_IDENTITY_OBJECT, 
CO_OD_SDO_SERVER,
CO_OD_SDO_CLIENT_01, 
CO_OD_SDO_CLIENT_02, 
CO_OD_RPOD, 
CO_OD_RPOD_MAPPING, 
CO_OD_TPDO_COMMUNICATION, 
CO_OD_TPOD_MAPPING, 
CO_OD_COMMUM_ENTRIES};

//array used to hold the MASTER <--> SLAVE OD configuration.
const uint16_t masterSlaveSetup[MASTER_SLAVE_SIZE] = {
CO_OD_REG_DEVICE_TYPE, 
CO_OD_REG_ERROR, 
CO_OD_REG_SYNC_MESSAGE,
CO_OD_REG_SYNC_PERIOD, 
CO_OD_REG_EMCY_MSG, 
CO_OD_REG_HB_TIME, 
CO_OD_IDENTITY_OBJECT, 
CO_OD_SDO_SERVER, 
CO_OD_SDO_CLIENT_01, 
CO_OD_RPOD, 
CO_OD_RPOD_MAPPING, 
CO_OD_TPDO_COMMUNICATION, 
CO_OD_TPOD_MAPPING, 
CO_OD_COMMUM_ENTRIES};

//array used to hold the SLAVE <--> MASTER OD configuration.
const uint16_t slaveMasterSetup[SLAVE_MASTER_SIZE] = {
CO_OD_REG_DEVICE_TYPE, 
CO_OD_REG_ERROR, 
CO_OD_REG_SYNC_MESSAGE,
CO_OD_REG_SYNC_PERIOD, 
CO_OD_REG_EMCY_MSG, 
CO_OD_REG_HB_TIME, 
CO_OD_IDENTITY_OBJECT, 
CO_OD_SDO_SERVER, 
CO_OD_SDO_CLIENT_01,
CO_OD_RPOD, 
CO_OD_RPOD_MAPPING, 
CO_OD_TPDO_COMMUNICATION, 
CO_OD_TPOD_MAPPING, 
CO_OD_COMMUM_ENTRIES};

/* Allocate global variables for runtime value of objects */
CO_HBCONS AppHbConsumer_1 = {
    .Time = HEARTBEAT_CONSUME_PERIOD_MS,
    .NodeId = GNR2_MASTER_NODE_ID,
};

uint16_t hObjDataProdHbTime       = HEARTBEAT_PRODUCE_PERIOD_MS;
uint8_t  hObjDataErrorRegister    = 0;

/*****Allocate global variables for GNR objects*****/
int16_t  hObjDataMotor1SpeedMeas            = 0;
uint16_t hObjDataMotor1BusVoltage           = 0;
int16_t  hObjDataMotor1Temp                      = 0;
int16_t hObjDataHeatsink1Temp                  = 0;
uint16_t hObjDataMotor1State                     = 0;
uint16_t hObjDataMotor1OccuredFaults        = 0;
uint16_t hObjDataMotor1CurrentFaults        = 0;

int16_t  hObjDataMotor2SpeedMeas            = 0;
uint16_t hObjDataMotor2BusVoltage           = 0;
int16_t  hObjDataMotor2Temp                 = 0;
int16_t  hObjDataHeatsink2Temp              = 0;
uint16_t hObjDataMotor2State                = 0;
uint16_t hObjDataMotor2OccuredFaults        = 0;
uint16_t hObjDataMotor2CurrentFaults        = 0;

uint8_t  bObjDataMotor1Start                = 0;
int16_t  hObjDataMotor1TorqRef              = 0;
uint8_t  bObjDataMotor1FaultAck             = 0;

uint8_t  bObjDataMotor2Start                   = 0;
int16_t  hObjDataMotor2TorqRef              = 0;
uint8_t  bObjDataMotor2FaultAck             = 0;
    
/*****Allocate global variables for GNR-IOT objects*****/
uint8_t  bObjDataSpeedMeas                  = 0;
uint8_t  bObjDataSOC                        = 0;
uint8_t  bObjDataPAS                        = DEFAULT_PAS_LEVEL; // The default PAS level should be 1
uint8_t  bObjDataMaxPAS                     = 0;
uint32_t hObjDataFwVersion                  = 0;
uint16_t hObjDataDCPowerMeas                = 0;
uint16_t hObjDataTorqueMeas                 = 0; 
uint16_t hObjDataPowerMeas                  = 0;
uint16_t hObjDataMaxDCPower                 = 0;
uint32_t hObjDataErrorState                 = 0;
uint32_t wObjDataSerialNbL                  = 0;
uint32_t wObjDataSerialNbH                  = 0;

/***************************************************************/

/*****Allocate global variables for data flash update Gnr objects*****/

//variable associated with CO_OD_REG_DEVICE_TURNNING_OFF .
uint8_t bObjDataDeviceTurnningOff           = 0;

//variable associated with CO_OD_REG_KEY_USER_DATA_CONFIG.
uint16_t bObjDataKeyUserDataConfig          = 0;

/*****Allocate global variables for Throttle/Pedal Assist Gnr objects*****/

//variable associated with CO_OD_REG_PAS_MAX_TORQUE_RATIO.
uint8_t bObjDataPasMaxTorqueRatio                 = 0; 

//variable associated with CO_OD_REG_PAS_DETECTION_STARTUP subindex 0
uint8_t bObjDataPasTorqueStartupSpeed = 0;
//variable associated with CO_OD_REG_PAS_DETECTION_STARTUP subindex 1
uint8_t bObjDataPasTorqueStartupThreshold = 0;
//variable associated with CO_OD_REG_PAS_DETECTION_STARTUP subindex 2
uint8_t bObjDataPasCadenceStartupNumbPulses = 0;
//variable associated with CO_OD_REG_PAS_DETECTION_STARTUP subindex 3
uint16_t bObjDataPasCadenceStartupWindows = 0;
//variable associated with CO_OD_REG_PAS_DETECTION_RUNNING subindex 4
uint8_t bObjDataPasAlgorithmStartup = 0;

//variable associated with CO_OD_REG_PAS_ACCEL_RAMP subindex 0
uint8_t bObjDataPas1AccelRampType = 0;
//variable associated with CO_OD_REG_PAS_ACCEL_RAMP subindex 1
uint16_t bObjDataPas1AccelRampArg1 = 0;
//variable associated with CO_OD_REG_PAS_ACCEL_RAMP subindex 2
uint8_t bObjDataPas2AccelRampType = 0;
//variable associated with CO_OD_REG_PAS_ACCEL_RAMP subindex 3
uint16_t bObjDataPas2AccelRampArg1 = 0;
//variable associated with CO_OD_REG_PAS_ACCEL_RAMP subindex 4
uint8_t bObjDataPas3AccelRampType = 0;
//variable associated with CO_OD_REG_PAS_ACCEL_RAMP subindex 5
uint16_t bObjDataPas3AccelRampArg1 = 0;
//variable associated with CO_OD_REG_PAS_ACCEL_RAMP subindex 6
uint8_t bObjDataPas4AccelRampType = 0;
//variable associated with CO_OD_REG_PAS_ACCEL_RAMP subindex 7
uint16_t bObjDataPas4AccelRampArg1 = 0;
//variable associated with CO_OD_REG_PAS_ACCEL_RAMP subindex 8
uint8_t bObjDataPas5AccelRampType = 0;
//variable associated with CO_OD_REG_PAS_ACCEL_RAMP subindex 9
uint16_t bObjDataPas5AccelRampArg1 = 0;
//variable associated with CO_OD_REG_PAS_ACCEL_RAMP subindex 10
uint8_t bObjDataPas6AccelRampType = 0;
//variable associated with CO_OD_REG_PAS_ACCEL_RAMP subindex 11
uint16_t bObjDataPas6AccelRampArg1 = 0;
//variable associated with CO_OD_REG_PAS_ACCEL_RAMP subindex 12
uint8_t bObjDataPas7AccelRampType = 0;
//variable associated with CO_OD_REG_PAS_ACCEL_RAMP subindex 13
uint16_t bObjDataPas7AccelRampArg1 = 0;
//variable associated with CO_OD_REG_PAS_ACCEL_RAMP subindex 14
uint8_t bObjDataPas8AccelRampType = 0;
//variable associated with CO_OD_REG_PAS_ACCEL_RAMP subindex 15
uint16_t bObjDataPas8AccelRampArg1 = 0;
//variable associated with CO_OD_REG_PAS_ACCEL_RAMP subindex 16
uint8_t bObjDataPas9AccelRampType = 0;
//variable associated with CO_OD_REG_PAS_ACCEL_RAMP subindex 17
uint16_t bObjDataPas9AccelRampArg1 = 0;

//variable associated with CO_OD_REG_PAS_DECEL_RAMP subindex 0
uint8_t bObjDataPas1DecelRampType = 0;
//variable associated with CO_OD_REG_PAS_DECEL_RAMP subindex 1
uint16_t bObjDataPas1DecelRampArg1 = 0;
//variable associated with CO_OD_REG_PAS_DECEL_RAMP subindex 2
uint8_t bObjDataPas2DecelRampType = 0;
//variable associated with CO_OD_REG_PAS_DECEL_RAMP subindex 3
uint16_t bObjDataPas2DecelRampArg1 = 0;
//variable associated with CO_OD_REG_PAS_DECEL_RAMP subindex 4
uint8_t bObjDataPas3DecelRampType = 0;
//variable associated with CO_OD_REG_PAS_DECEL_RAMP subindex 5
uint16_t bObjDataPas3DecelRampArg1 = 0;
//variable associated with CO_OD_REG_PAS_DECEL_RAMP subindex 6
uint8_t bObjDataPas4DecelRampType = 0;
//variable associated with CO_OD_REG_PAS_DECEL_RAMP subindex 7
uint16_t bObjDataPas4DecelRampArg1 = 0;
//variable associated with CO_OD_REG_PAS_DECEL_RAMP subindex 8
uint8_t bObjDataPas5DecelRampType = 0;
//variable associated with CO_OD_REG_PAS_DECEL_RAMP subindex 9
uint16_t bObjDataPas5DecelRampArg1 = 0;
//variable associated with CO_OD_REG_PAS_DECEL_RAMP subindex 10
uint8_t bObjDataPas6DecelRampType = 0;
//variable associated with CO_OD_REG_PAS_DECEL_RAMP subindex 11
uint16_t bObjDataPas6DecelRampArg1 = 0;
//variable associated with CO_OD_REG_PAS_DECEL_RAMP subindex 12
uint8_t bObjDataPas7DecelRampType = 0;
//variable associated with CO_OD_REG_PAS_DECEL_RAMP subindex 13
uint16_t bObjDataPas7DecelRampArg1 = 0;
//variable associated with CO_OD_REG_PAS_DECEL_RAMP subindex 14
uint8_t bObjDataPas8DecelRampType = 0;
//variable associated with CO_OD_REG_PAS_DECEL_RAMP subindex 15
uint16_t bObjDataPas8DecelRampArg1 = 0;
//variable associated with CO_OD_REG_PAS_DECEL_RAMP subindex 16
uint8_t bObjDataPas9DecelRampType = 0;
//variable associated with CO_OD_REG_PAS_DECEL_RAMP subindex 17
uint16_t bObjDataPas9DecelRampArg1 = 0;

//variable associated with CO_OD_REG_TORQUE_SENSOR_MULTIPLIER.
uint16_t bObjDataTorqueSensorMultiplier[10] = {0};

//variable associated with CO_OD_REG_PAS_MIN_TORQUE.
uint8_t bObjDataPasLevelMinTorque[10]              = {0};

//variable associated with CO_OD_REG_PAS_LEVEL_SPEED.
uint8_t bObjDataPasLeveSpeed[10]  = {0};

//variable associated with CO_OD_REG_PAS_MAX_TORQUE.
uint8_t bObjDataPasLevelMaxTorque[10]        = {0};

//variable associated with CO_OD_REG_MAX_SPEED.
uint8_t bObjDataMaxSpeed                    = 0;

//variable associated with CO_OD_REG_WALK_MODE_SPEED.
uint8_t bObjDataWalkModeSpeed               = 0;

//variable associated with CO_OD_REG_WALK_MODE_SPEED.
uint8_t bObjDataWalkModeMaxTorque               = 0;

//variable associated with CO_OD_REG_WALK_MODE_SPEED.
uint8_t bObjDataWalkModeAccelRampType               = 0;

//variable associated with CO_OD_REG_WALK_MODE_SPEED.
uint16_t bObjDataWalkModeAccelRampArg1              = 0;


//variable associated with CO_OD_REG_BATTERY_VOLTAGE 0
uint16_t bObjDataBatteryFullVoltage         = 0;
//variable associated with CO_OD_REG_BATTERY_VOLTAGE 1
uint16_t bObjDataBatteryEmptyVoltage        = 0;

//variable associated with CO_OD_REG_WHEELS_DIAMETER 0
uint8_t bObjDataWheelDiameter               = 0;
//variable associated with CO_OD_REG_WHEELS_DIAMETER 1 
uint8_t bObjDataWheelPulsePerRotation       = 0;
//variable associated with CO_OD_REG_WHEELS_DIAMETER 2 
uint8_t bObjDataWheelDiameterDefault        = 0;

//variable associated with CO_OD_REG_VEHICLE_FRONT_LIGHT 0
uint8_t bObjDataFrontLightState             = 0;

//variable associated with CO_OD_REG_VEHICLE_FRONT_LIGHT 1
uint8_t bObjDataFrontLightDefaultState      = 0;

//variable associated with CO_OD_REG_VEHICLE_REAR_LIGHT  0
uint8_t bObjDataRearLightState              = 0;

//variable associated with CO_OD_REG_VEHICLE_REAR_LIGHT  1
uint8_t bObjDataRearLightDefaultState       = 0;

//variable associated with CO_OD_REG_VEHICLE_REAR_LIGHT 2
uint8_t bObjDataRearLightBlinkOnBrake       = 0;

//variable associated with CO_OD_REG_VEHICLE_REAR_LIGHT 3 PLACEHOLDER
uint16_t bObjDataRearLightBlinkPeriod        = 0; // in ms

//variable associated with CO_OD_REG_VEHICLE_REAR_LIGHT 4 PLACEHOLDER
uint8_t bObjDataRearLightBlinkDutyCycle     = 0; // % on

//variable associated with CO_OD_REG_MASTER_SLAVE_PRESENT
uint8_t bObjDataMasterSlavePresent          = 0;

//variable associated with COD_OD_REG_PAS_SENSOR 0 (Placeholder currently not implemented) 
uint16_t bObjDataPasPedalRPM                = 0;
//variable associated with COD_OD_REG_PAS_SENSOR 1 (Placeholder currently not implemented) 
uint8_t  bObjDataPasTorqueForcePercent      = 0;
//variable associated with COD_OD_REG_PAS_SENSOR 2 (Placeholder currently not implemented) 
uint16_t bObjDataPasTorqueForceWatts       = 0;
//variable associated with COD_OD_REG_PAS_SENSOR 3
uint8_t  bObjDataPasNbMagnetsPerTurn       = 0;  
//variable associated with COD_OD_REG_PAS_SENSOR 4
uint16_t bObjDataPasTorqueInputMin         = 0;  
//variable associated with COD_OD_REG_PAS_SENSOR 5
uint16_t bObjDataPasTorqueInputMax         = 0;

//variable associated with CO_OD_CONFIG_SCREEN_PROTOCOL
uint8_t bObjDataConfigScreenProtocol        = 0;
//variable associated with CO_OD_REG_PAS_DETECTION_RUNNING subindex 0
uint8_t bObjDataPasTorqueRunningThreshold = 0;
//variable associated with CO_OD_REG_PAS_DETECTION_RUNNING subindex 1
uint8_t bObjDataPasCadenceRunningNumbPulses = 0;
//variable associated with CO_OD_REG_PAS_DETECTION_RUNNING subindex 2
uint16_t bObjDataPasCadenceRunningWindows = 0;
//variable associated with CO_OD_REG_PAS_DETECTION_RUNNING subindex 3
uint8_t bObjDataPasAlgorithmRunning = 0;

//variable associated with  CO_OD_REG_BATTERY_DC_CURRENT subindex 0
uint16_t bObjDataConfigBatteryMaxPeakDCCurrent             = 0;
//variable associated with  CO_OD_REG_BATTERY_DC_CURRENT subindex 1
uint16_t bObjDataConfigBatteryContinuousDCCurrent          = 0;
//variable associated with  CO_OD_REG_BATTERY_DC_CURRENT subindex 2
uint16_t bObjDataConfigBatteryPeakCurrentDuration          = 0;
//variable associated with  CO_OD_REG_BATTERY_DC_CURRENT subindex 3
uint16_t bObjDataConfigBatteryPeakCurrentDeratingDuration  = 0;

//variable associated with CO_OD_REG_CONTROLLER_THROTTLE subindex 0
uint16_t bObjDataConfigThrottleAdcValue = 0;

//variable associated with CO_OD_REG_CONTROLLER_THROTTLE subindex 1
uint8_t  bObjDataConfigThrottleGetSetValue = 0;

//variable associated with CO_OD_REG_CONTROLLER_THROTTLE subindex 2
uint16_t bObjDataConfigThrottleAdcOffset    = 0;

//variable associated with CO_OD_REG_CONTROLLER_THROTTLE subindex 3
uint16_t bObjDataConfigThrottleAdcMax       = 0;

//variable associated with CO_OD_REG_CONTROLLER_THROTTLE subindex 4
uint8_t  bObjDataConfigThrottleBlockOff = 0;

//variable associated with CO_OD_REG_CONTROLLER_THROTTLE subindex 5
uint8_t  bObjDataConfigThrottleMaxSpeed = 0;

//variable associated with CO_OD_REG_CONTROLLER_THROTTLE subindex 6
uint8_t  bObjDataConfigThrottleAccelRampType= 0;

//variable associated with CO_OD_REG_CONTROLLER_THROTTLE subindex 7
uint16_t  bObjDataConfigThrottleAccelRampArg1 = 0;

//variable associated with CO_OD_CONFIG_SPEED_FOR_TORQUE_FILTER.
uint16_t bObjDataConfigSpeedForTorqueFilter[2]    = {0};

//variable associated with CO_OD_CONFIG_TORQUE_FILTER_FOR_SPEED.
uint16_t bObjDataConfigTorqueFilterForSpeed[6]    = {0};

//variable associated with CO_OD_REG_FIRMWAREUPDATE_MEMORY subindex 0
uint8_t bObjOtaCommand = 0;

//variable associated with CO_OD_REG_FIRMWAREUPDATE_MEMORY subindex 1
uint8_t bObjOtaStatus = 0;

//variable associated with CO_OD_REG_FIRMWAREUPDATE_MEMORY subindex 3
uint16_t bObjOtaFrameCount = 0;

//array that will be linked with domain object below.
uint8_t bObjfirmwareUpdateBuffer[FIRMWAREUPDATE_MEMORYSIZE];

//variable of domain type associated with CO_OD_REG_FIRMWAREUPDATE_MEMORY subindex 2
//Declare and initialize a struct that holds a array used 
//to receive the firmware data frame(134 bytes) at the same index and subindex.
//This is the way to have one variable in the object dictionary 
//with more than 4 bytes.
CO_OBJ_DOM bObjFirmwareUpdateDomain = {
  0,                              /* variable for read position     */
  FIRMWAREUPDATE_MEMORYSIZE,      /* size of domain memory          */
  &bObjfirmwareUpdateBuffer[0]        /* start address of domain memory */
};

//Declate the id used to receive all canopen configuration to this node.
struct CO_OBJ_T GNR2_OD[GNR2_OBJ_N];

/* Each software timer needs some memory for managing
 * the lists and states of the timed action events.
 */
static CO_TMR_MEM TmrMem[GNR2_TMR_N];

/* Each SDO server needs memory for the segmented or
 * block transfer requests.
 */
static uint8_t SdoSrvMem[CO_SSDO_N][CO_SDO_BUF_BYTE];

/* Select the drivers for your application. For possible
 * selections, see the directory /drivers. In this example
 * we select the driver templates. You may fill them with
 * your specific hardware functionality.
 */
static struct CO_IF_DRV_T CoGnrDriver = {
  &CoCanDriver,
  &CoTimerDriver,
  &CoNvmDriver
};

/* Specify the EMCY error codes with the corresponding
 * error register bit. There is a collection of defines
 * for the predefined emergency codes CO_EMCY_CODE...
 * and for the error register bits CO_EMCY_REG... for
 * readability. You can use plain numbers, too.
 */
static CO_EMCY_TBL AppEmcyTbl[APP_ERR_ID_NUM] = {
    { CO_EMCY_REG_GENERAL, CO_EMCY_CODE_GEN_ERR          }, /* APP_ERR_CODE_SOMETHING */
    { CO_EMCY_REG_TEMP   , CO_EMCY_CODE_TEMP_AMBIENT_ERR }  /* APP_ERR_CODE_HAPPENS   */
};

/******************************************************************************
* PUBLIC VARIABLES
******************************************************************************/

/* Collect all node specification settings in a single
 * structure for initializing the node easily.
 */
struct CO_NODE_SPEC_T GnR2ModuleSpec = {
    GNR2_NODE_ID,             /* default Node-Id                */
    GNR2_BAUDRATE,            /* default Baudrate               */
    &GNR2_OD[0],              /* pointer to object dictionary   */
    GNR2_OBJ_N,               /* object dictionary max length   */
    &AppEmcyTbl[0],           /* EMCY code & register bit table */
    &TmrMem[0],               /* pointer to timer memory blocks */
    GNR2_TMR_N,               /* number of timer memory blocks  */
    GNR2_TICKS_PER_SEC,       /* timer clock frequency in Hz    */
    &CoGnrDriver,             /* select drivers for application */
    (uint8_t*)&SdoSrvMem[0]             /* SDO Transfer Buffer Memory     */
};

/**************************************************************
*      Private Functions Declaration                          *
**************************************************************/
/**
  @brief Function to add a new Object to the CANOPEN OD.
                 This function must be called only by the 
                 CO_GnrOdSetup function.
  @param uint16_t objId address of the objtect or entrie 
         to be add in the OD.
  @param bool deviceType indicate if the device is master(true)
         or slave(false).
  @retval none
 */
static void CO_addObj(uint16_t objId, bool deviceType);

/**
  @brief Function used to add a specific setup in the OD.
  @param const uint16_t * arraySetup pointe to the array
         that has all address to be added on one specifc 
         setup(master, master + iot, and etc).
  @param bool deviceType indicate if the device is master(true)
         or slave(false).
  @retval none
 */
static void CO_GnrOdSetup(const uint16_t arraySetup, bool deviceType);


/*********************************************************************************
*                       Public Function                        
**********************************************************************************/

/**
  Function used to config the OD as master or slave.
  Must be called before initialise CANOPEN Node.
  Must be called only once.
 */
void CO_SelecOdSetup(bool deviceFunction)
{
    
    //check if the device must be configured as master or slave.
    //true is master, false is slave.
    if (deviceFunction)
    {
        #if GNR_IOT == 1 && SUPPORT_SLAVE_ON_IOT == 0
            for(uint8_t index = 0; index < MASTER_IOT_SIZE; index++)
            {
                CO_GnrOdSetup(masterIotSetup[index],deviceFunction);
            }
        #endif
            
        #if GNR_IOT == 1 && SUPPORT_SLAVE_ON_IOT == 1
            for(uint8_t index = 0; index < MASTER_IOT_SLAVE_SIZE; index++)
            {
                CO_GnrOdSetup(masterIotSlaveSetup[index],deviceFunction);
            }
        #endif
        
        //master to dual communication
        #if GNR_IOT == 0
            for(uint8_t index = 0; index < MASTER_SLAVE_SIZE; index++)
            {
                CO_GnrOdSetup(masterSlaveSetup[index],deviceFunction);
            }
            
        #endif   
    }
    else
    {
        //Slave setup
        for(uint8_t index = 0; index < SLAVE_MASTER_SIZE; index++)
        {
            CO_GnrOdSetup(slaveMasterSetup[index],deviceFunction);
        }
    }
    
}

/*********************************************************************************
*                       Private Function                        
**********************************************************************************/

/**
  Function to add a new Object to the CANOPEN OD.
  This function must be called before initialise 
  the CANOPEN NODE.
 */
static void CO_addObj(uint16_t objId, bool deviceType)
{   
    
    //used to control the entrie position in the GNR2_OD[index] array.
    static uint16_t index = 0;
    
    //state machine resposible to config the CANOPEN OD.
    switch(objId)
    {
        //Device Type
        case CO_OD_REG_DEVICE_TYPE:
            
            //Add Device Type
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_DEVICE_TYPE, 0, CO_OBJ_D___R_), CO_TUNSIGNED32, (CO_DATA)0};
            
            //move the indext to the next free position in the array.
            index++;
            
        break;
            
        //Error register
        case CO_OD_REG_ERROR:
            
            //Add error Register
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_ERROR, 0, CO_OBJ_____R_), CO_TUNSIGNED8, (CO_DATA)&hObjDataErrorRegister};
            
            //move the indext to the next free position in the array.
            index++;
            
        break;
            
        //master needs to send a SYNC message to make slave node 
        //respond to the master by TPDO at the very same time.
        //when using the macro CO_COBID_SYNC_STD the GENERATE_SYNC define
        //configure the device to produces SYNC messages.
        case CO_OD_REG_SYNC_MESSAGE:
            
            //master is configured to produce SYNC message.
            //it's a dual configuration + iot.
            if ( (GNR_IOT == 1) && (SUPPORT_SLAVE_ON_IOT == 1))
            {
                //Add COB-ID SYNC Message.
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_SYNC_MESSAGE, 0, CO_OBJ_D___R_), CO_TSYNC_ID, (CO_DATA)CO_COBID_SYNC_STD(MASTER_GENERATE_SYNC, STD_ID_SYNC)};
            } 
            else
            {
                //master is not configured to produce SYNC message.
                //it's a single GRN + IOT
                if ((GNR_IOT == 1) && (SUPPORT_SLAVE_ON_IOT == 0))
                {
                    //on this case the device is configured to cosumes SYNC message. 
                    //but IOT module is not sending SYNC messages.
                    //Add COB-ID SYNC Message. 
                    GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_SYNC_MESSAGE, 0, CO_OBJ_D___R_), CO_TSYNC_ID, (CO_DATA)STD_ID_SYNC};
                }
                else
                {
                    //master is configured to produce SYNC message.
                    //it's a dual config, master + slave, no 
                    if (deviceType == true)
                    {
                        //on this case the device is configured to cosumes SYNC message. 
                        //but IOT module is not sending SYNC messages.
                        //Add COB-ID SYNC Message.
                        GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_SYNC_MESSAGE, 0, CO_OBJ_D___R_), CO_TSYNC_ID, (CO_DATA)CO_COBID_SYNC_STD(MASTER_GENERATE_SYNC, STD_ID_SYNC)};
                    }
                    else
                    {
                        //slave is not configured to produce SYNC message.
                        //it's a dual config but it is a slave device. 
                        //Add COB-ID SYNC Message.
                        GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_SYNC_MESSAGE, 0, CO_OBJ_D___R_), CO_TSYNC_ID, (CO_DATA)CO_COBID_SYNC_STD(SLAVE_GENERATE_SYNC, STD_ID_SYNC)};
                    }
                }
            }
            
            //move the indext to the next free position in the array.
            index++;
        
        break;
            
        //add the syn period to master or slave.
        case CO_OD_REG_SYNC_PERIOD:
            
            //master is has diferent syn time 
            if (deviceType == true)
            {
                //Add SYNC period
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_SYNC_PERIOD, 0, CO_OBJ_D___R_), CO_TSYNC_CYCLE, (CO_DATA)MASTER_SYNC_PERIOD_US};
            }
            else
            {
                //Add SYNC period
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_SYNC_PERIOD, 0, CO_OBJ_D___R_), CO_TSYNC_CYCLE, (CO_DATA)SLAVE_SYNC_PERIOD_US};
            }
            
            //move the indext to the next free position in the array.
            index++;
                
        break;
              
        //Emergency message is add.
        case CO_OD_REG_EMCY_MSG:
            
            //Add COB-ID EMCY Message
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_EMCY_MSG, 0, CO_OBJ_D___R_), CO_TUNSIGNED32, (CO_DATA)CO_COBID_EMCY_STD(USE_EMCY, STD_ID_EMCY)};
        
            //move the indext to the next free position in the array.
            index++;
            
        break;
            
        //Producer Heartbeat period
        case CO_OD_REG_HB_TIME:
            
            // Producer Heartbeat Time
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_HB_TIME, 0, CO_OBJ_____RW), CO_THB_PROD, (CO_DATA)&hObjDataProdHbTime};
            
            //move the indext to the next free position in the array.
            index++;
            
        break;
            
        //add the Identity Object(like Vendor ID,Product code, Revision number and Serial number.
        case CO_OD_IDENTITY_OBJECT:
            
            // Identity - Highest Sub Index
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_IDENTITY_OBJECT, 0, CO_OBJ_D___R_), CO_TUNSIGNED8, (CO_DATA)4};	
        
            //move the indext to the next free position in the array.
            index++;
            
            // Identity - Vendor ID
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_IDENTITY_OBJECT, 1, CO_OBJ_D___R_), CO_TUNSIGNED32, (CO_DATA)0};
        
            //move the indext to the next free position in the array.
            index++;
            
            // Identity - Product Code
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_IDENTITY_OBJECT, 2, CO_OBJ_D___R_), CO_TUNSIGNED32, (CO_DATA)0};
            
            //move the indext to the next free position in the array.
            index++;
            
            // Identity - Revision Number        
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_IDENTITY_OBJECT, 3, CO_OBJ_D___R_), CO_TUNSIGNED32, (CO_DATA)0};
          
            //move the indext to the next free position in the array.
            index++;
            
            // Identity - Serial Number
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_IDENTITY_OBJECT, 4, CO_OBJ_D___R_), CO_TUNSIGNED32, (CO_DATA)0};
        
            //move the indext to the next free position in the array.
            index++;
            
        break;
            
        //Communication Object SDO Server. define server coib-id.
        case CO_OD_SDO_SERVER:
            
            //server sdo
            // SDO Srv Parameter - Highest Sub Index
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_SDO_SERVER, 0, CO_OBJ_D___R_), CO_TUNSIGNED8, (CO_DATA)2};
            
            //move to next OD index
            index++;
            
            // SDO Srv Parameter - COB-ID Client to Server        
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_SDO_SERVER, 1, CO_OBJ_DN__R_), CO_TUNSIGNED32, CO_COBID_SDO_REQUEST()};
            
            //move to next OD index
            index++;
            
            // SDO Srv Parameter - COB-ID Server to Client  
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_SDO_SERVER, 2, CO_OBJ_DN__R_), CO_TUNSIGNED32, CO_COBID_SDO_RESPONSE()};	
            
            //move to next OD index
            index++;
            
        break;
            
        //SDO Client 01 - a device to request information
        case CO_OD_SDO_CLIENT_01:
            
            //if true use master config(dual or single/iot).
            if (deviceType == true)
            {
                //SDO Client Parameter - Highest Sub Index
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_SDO_CLIENT_01, 0, CO_OBJ_D___R_), CO_TUNSIGNED8, (CO_DATA)3};
                
                //move to next OD index
                index++;
                
                // SDO Client Parameter - COB-ID Client to Server        
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_SDO_CLIENT_01, 1, CO_OBJ_D___R_), CO_TUNSIGNED32, CO_COBID_SDO_REQUEST()};
                
                //move to next OD index
                index++;
                
                // SDO Client Parameter - COB-ID Server to Client
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_SDO_CLIENT_01, 2, CO_OBJ_D___R_), CO_TUNSIGNED32, CO_COBID_SDO_RESPONSE()};
                
                //move to next OD index
                index++;
                
                //Node-ID of the SDO server        
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_SDO_CLIENT_01, 3, CO_OBJ_D___R_), CO_TUNSIGNED8, (CO_DATA)GNR2_SLAVE_NODE_ID};
                //move to next OD index
                index++;
            }
            else
            {
                // SDO Client Parameter - Highest Sub Index  
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_SDO_CLIENT_01, 0, CO_OBJ_D___R_), CO_TUNSIGNED8, (CO_DATA)3};
                
                //move to next OD index
                index++;
        
                // SDO Client Parameter - COB-ID Client to Server     
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_SDO_CLIENT_01, 1, CO_OBJ_D___R_), CO_TUNSIGNED32, CO_COBID_SDO_REQUEST()};
        
                //move to next OD index
                index++;
        
                // SDO Client Parameter - COB-ID Server to Client
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_SDO_CLIENT_01, 2, CO_OBJ_D___R_), CO_TUNSIGNED32, CO_COBID_SDO_RESPONSE()};
        
                //move to next OD index
                index++;
        
                //Node-ID of the SDO server        
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_SDO_CLIENT_01, 3, CO_OBJ_D___R_), CO_TUNSIGNED8, (CO_DATA)GNR2_MASTER_NODE_ID};
        
                //move to next OD index
                index++;   
            }
            
        break;
            
        //SDO Client 02 - a device to request information
        case CO_OD_SDO_CLIENT_02:
            
            // SDO Client Parameter - Highest Sub Index
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_SDO_CLIENT_02, 0, CO_OBJ_D___R_), CO_TUNSIGNED8, (CO_DATA)3};
        
            //move to next OD index
            index++;
            
            // SDO Client Parameter - COB-ID Client to Server       
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_SDO_CLIENT_02, 1, CO_OBJ_D___R_), CO_TUNSIGNED32, CO_COBID_SDO_REQUEST()};	
        
            //move to next OD index
            index++;
            
            // SDO Client Parameter - COB-ID Server to Client        
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_SDO_CLIENT_02, 2, CO_OBJ_D___R_), CO_TUNSIGNED32, CO_COBID_SDO_RESPONSE()};
        
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_SDO_CLIENT_02, 3, CO_OBJ_D___R_), CO_TUNSIGNED8, (CO_DATA)IOT_NODE_ID};
            
            //move to next OD index
            index++;
            
        break;
            
        //RPDO communication parameter
        case CO_OD_RPOD:
            
            //if true use master config
            if (deviceType == true)
            { 
                // RPDO1 Parameter - Highest Sub Index
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_RPOD, 0, CO_OBJ_D___R_), CO_TUNSIGNED8, (CO_DATA)5};
        
                //move to next OD index
                index++;
        
                // RPDO1 Parameter - COB-ID RPDO        
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_RPOD, 1, CO_OBJ_D___R_), CO_TUNSIGNED32, CO_COBID_RPDO_STD(USE_RPDO, MASTER_STD_ID_RPDO1)};
        
                //move to next OD index
                index++;
        
                // RPDO1 Parameter - Transmission type
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_RPOD, 2, CO_OBJ_D___R_), CO_TUNSIGNED8, MASTER_RPDO_TRANSMISSION_TYPE};	
        
                //move to next OD index
                index++;// RPDO1 Parameter - Event period
        
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_RPOD, 5, CO_OBJ_D___R_), CO_TUNSIGNED16, MASTER_RPDO_PERIOD_MS};
        
                //move to next OD index
                index++; 
            }
            else
            {
                // RPDO1 Parameter - Highest Sub Index
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_RPOD, 0, CO_OBJ_D___R_), CO_TUNSIGNED8, (CO_DATA)5};
        
                //move to next OD index
                index++;
        
                // RPDO1 Parameter - COB-ID RPDO        
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_RPOD, 1, CO_OBJ_D___R_), CO_TUNSIGNED32, CO_COBID_RPDO_STD(USE_RPDO, SLAVE_STD_ID_RPDO1)};
        
                //move to next OD index
                index++;
        
                // RPDO1 Parameter - Transmission type
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_RPOD, 2, CO_OBJ_D___R_), CO_TUNSIGNED8, SLAVE_RPDO_TRANSMISSION_TYPE};	
        
                //move to next OD index
                index++;
                
                // RPDO1 Parameter - Event period
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_RPOD, 5, CO_OBJ_D___R_), CO_TUNSIGNED16, SLAVE_RPDO_PERIOD_MS};
        
                //move to next OD index
                index++;
            }
            
        break;
            
        //RPDO mapping parameter
        case CO_OD_RPOD_MAPPING:
            
            //if true use master config
            if (deviceType == true)
            { 
                // RPDO1 Mapping - Number of mapped object in PDO
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_RPOD_MAPPING, 0, CO_OBJ_D___R_), CO_TUNSIGNED8, (CO_DATA)4};	
        
                //move to next OD index
                index++;
        
                // RPDO1 Mapping - Object 1
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_RPOD_MAPPING, 1, CO_OBJ_D___R_), CO_TUNSIGNED32, CO_LINK(CO_OD_REG_MOTOR_STATE, 1, 16)};	
        
                //move to next OD index
                index++;
        
                // RPDO1 Mapping - Object 2
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_RPOD_MAPPING, 2, CO_OBJ_D___R_), CO_TUNSIGNED32, CO_LINK(CO_OD_REG_MOTOR_OCC_FAULTS, 1, 16)};	
        
                //move to next OD index
                index++;
        
                // RPDO1 Mapping - Object 3
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_RPOD_MAPPING, 3, CO_OBJ_D___R_), CO_TUNSIGNED32, CO_LINK(CO_OD_REG_MOTOR_CUR_FAULTS, 1, 16)};			
                
                //move to next OD index
                index++;
                
                // RPDO1 Mapping - Object 4
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_RPOD_MAPPING, 4, CO_OBJ_D___R_), CO_TUNSIGNED32, CO_LINK(CO_OD_REG_MOTOR_SPEED, 1, 16)};		
                
                //move to next OD index
                index++;
            }
            else
            {
                 // RPDO1 Mapping - Number of mapped object in PDO
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_RPOD_MAPPING, 0, CO_OBJ_D___R_), CO_TUNSIGNED8, (CO_DATA)2};
        
                //move to next OD index
                index++;
        
                // RPDO1 Mapping - Object 1
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_RPOD_MAPPING, 1, CO_OBJ_D___R_), CO_TUNSIGNED32, CO_LINK(CO_OD_REG_MOTOR_TORQUE_REF, 1, 16)};	
        
                //move to next OD index
                index++;
        
                // RPDO1 Mapping - Object 2		
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_RPOD_MAPPING, 2, CO_OBJ_D___R_), CO_TUNSIGNED32, CO_LINK(CO_OD_REG_MOTOR_START, 1, 8)};			    
        
                //move to next OD index
                index++;
            }
            
        break;
            
        //TPDO communication parameter
        case CO_OD_TPDO_COMMUNICATION:
            
            //if true use master config
            if (deviceType == true)
            {
                // TPDO 1
                // TPDO1 Parameter - Highest Sub Index
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_TPDO_COMMUNICATION, 0, CO_OBJ_D___R_), CO_TUNSIGNED8, (CO_DATA)6};
                
                //move to next OD index
                index++;
                
                // TPDO1 Parameter - COB-ID TPDO
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_TPDO_COMMUNICATION, 1, CO_OBJ_D___R_), CO_TUNSIGNED32, CO_COBID_TPDO_STD(USE_TPDO, MASTER_STD_ID_TPDO1)};
                
                //move to next OD index
                index++;
                
                // TPDO1 Parameter - Transmission type
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_TPDO_COMMUNICATION, 2, CO_OBJ_D___R_), CO_TUNSIGNED8, MASTER_TPDO_TRANSMISSION_TYPE};
                
                //move to next OD index
                index++;
                
                // TPDO1 Parameter - Transmission type
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_TPDO_COMMUNICATION, 5, CO_OBJ_D___R_), CO_TUNSIGNED16, MASTER_TPDO_PERIOD_MS};
                
                //move to next OD index
                index++;
        
                // TPDO1 Parameter - SYNC start value
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_TPDO_COMMUNICATION, 6, CO_OBJ_D___R_), CO_TUNSIGNED8, 0};		
                
                //move to next OD index
                index++;
            }
            else
            {
                // TPDO 1
                // TPDO1 Parameter - Highest Sub Index
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_TPDO_COMMUNICATION, 0, CO_OBJ_D___R_), CO_TUNSIGNED8, (CO_DATA)6};
                
                //move to next OD index
                index++;
        
                // TPDO1 Parameter - COB-ID TPDO
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_TPDO_COMMUNICATION, 1, CO_OBJ_D___R_), CO_TUNSIGNED32, CO_COBID_TPDO_STD(USE_TPDO, SLAVE_STD_ID_TPDO1)};
                
                //move to next OD index
                index++;
                
                // TPDO1 Parameter - Transmission type
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_TPDO_COMMUNICATION, 2, CO_OBJ_D___R_), CO_TUNSIGNED8, SLAVE_TPDO_TRANSMISSION_TYPE};
                
                //move to next OD index
                index++;
                
                // TPDO1 Parameter - Transmission type
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_TPDO_COMMUNICATION, 5, CO_OBJ_D___R_), CO_TUNSIGNED16, SLAVE_TPDO_PERIOD_MS};
                
                //move to next OD index
                index++;
                
                // TPDO1 Parameter - SYNC start value
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_TPDO_COMMUNICATION, 6, CO_OBJ_D___R_), CO_TUNSIGNED8, 0};		
                
                //move to next OD index
                index++;
            }
        
        break;
            
        //TPDO mapping parameter
        case CO_OD_TPOD_MAPPING:
            
            //if true use master config
            if (deviceType == true)
            {
                // RPDO1 Mapping - Number of mapped object in PDO
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_TPOD_MAPPING, 0, CO_OBJ_D___R_), CO_TUNSIGNED8, (CO_DATA)2};
                //move to next OD index
                index++;
                // RPDO1 Mapping - Object 1
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_TPOD_MAPPING, 1, CO_OBJ_D___R_), CO_TUNSIGNED32, CO_LINK(CO_OD_REG_MOTOR_TORQUE_REF, 1, 16)};	
                //move to next OD index
                index++;
                // RPDO1 Mapping - Object 2		
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_TPOD_MAPPING, 2, CO_OBJ_D___R_), CO_TUNSIGNED32, CO_LINK(CO_OD_REG_MOTOR_START, 1, 8)};			    
                //move to next OD index
                index++;
                
            }
            else
            {
                // TPDO1 Mapping - Number of mapped object in PDO
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_TPOD_MAPPING, 0, CO_OBJ_D___R_), CO_TUNSIGNED8, (CO_DATA)4};	
                //move to next OD index
                index++;
                // TPDO1 Mapping - Object 1
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_TPOD_MAPPING, 1, CO_OBJ_D___R_), CO_TUNSIGNED32, CO_LINK(CO_OD_REG_MOTOR_STATE, 1, 16)};	
                //move to next OD index
                index++;
                // TPDO1 Mapping - Object 2
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_TPOD_MAPPING, 2, CO_OBJ_D___R_), CO_TUNSIGNED32, CO_LINK(CO_OD_REG_MOTOR_OCC_FAULTS, 1, 16)};	
                //move to next OD index
                index++;
                // TPDO1 Mapping - Object 3
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_TPOD_MAPPING, 3, CO_OBJ_D___R_), CO_TUNSIGNED32, CO_LINK(CO_OD_REG_MOTOR_CUR_FAULTS, 1, 16)};			
                //move to next OD index
                index++;
                // TPDO1 Mapping - Object 4
                GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_TPOD_MAPPING, 4, CO_OBJ_D___R_), CO_TUNSIGNED32, CO_LINK(CO_OD_REG_MOTOR_SPEED, 1, 16)};		
                //move to next OD index
                index++;
                
            }
        
        break;
          
        //commum parameters
        case CO_OD_COMMUM_ENTRIES:
            
            /**********************GNR2-IOT OBJECTS MODULE*******************************************/
            // Application - Inst Speed
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_SPEED_MEASURE, 0, CO_OBJ_____R_), CO_TUNSIGNED8, (CO_DATA)&bObjDataSpeedMeas}; 
            //move to next OD index
            index++;
            
            // Application - Inst Power
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_POWER_MEASURE, 0, CO_OBJ_____R_), CO_TUNSIGNED16, (CO_DATA)&hObjDataDCPowerMeas};  
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_POWER_MEASURE, 1, CO_OBJ_____R_), CO_TUNSIGNED16, (CO_DATA)&hObjDataTorqueMeas};  
            //move to next OD index
            index++;
                  
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_POWER_MEASURE, 2, CO_OBJ_____R_), CO_TUNSIGNED16, (CO_DATA)&hObjDataPowerMeas};  
            //move to next OD index
            index++;
            
            // Application - State of Charge
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_SOC,           0, CO_OBJ_____R_), CO_TUNSIGNED8, (CO_DATA)&bObjDataSOC}; 
            //move to next OD index
            index++;
            // Application - PAS Level
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_LEVEL,     0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPAS};
            //move to next OD index
            index++;
            // Application - Max PAS Level
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_MAX_PAS,       0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataMaxPAS};	
            //move to next OD index
            index++;
            // Application - Max Power
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_MAX_DCPOWER,   0, CO_OBJ_____RW), CO_TSIGNED16, (CO_DATA)&hObjDataMaxDCPower}; 
            //move to next OD index
            index++;
            // Application - Error State
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_ERR_STATE,     0, CO_OBJ_____R_), CO_TUNSIGNED32, (CO_DATA)&hObjDataErrorState};
            //move to next OD index
            index++;
            // Application - Serial Number High side
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_SERIAL_NB,     0, CO_OBJ_____R_), CO_TUNSIGNED32, (CO_DATA)&wObjDataSerialNbH};  
            //move to next OD index
            index++;
            // Application - Serial Number Low side
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_SERIAL_NB,     1, CO_OBJ_____R_), CO_TUNSIGNED32, (CO_DATA)&wObjDataSerialNbL};
            //move to next OD index
            index++;
            // Application - Firmware Version
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_FW_VERSION,    0, CO_OBJ_____R_), CO_TUNSIGNED32, (CO_DATA)&hObjDataFwVersion};   
            //move to next OD index
            index++;
        
            /************************GNR2 Motor parameters OBJECTS MODULE******************************/
            //master must to have this variables to exchange information with the slave if
            //dual motor setup is enabled.
            // Application - Measured motor speed of master
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_MOTOR_SPEED, 0, CO_OBJ_____R_), CO_TSIGNED16, (CO_DATA)&hObjDataMotor1SpeedMeas};
            //move to next OD index
            index++;
            // Application - Measured motor speed of slave 1
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_MOTOR_SPEED, 1, CO_OBJ_____R_), CO_TSIGNED16, (CO_DATA)&hObjDataMotor2SpeedMeas};			        
            //move to next OD index
            index++;
        
            // Application - Measured bus voltage of master
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_BUS_VOLTAGE, 0, CO_OBJ_____R_), CO_TUNSIGNED16, (CO_DATA)&hObjDataMotor1BusVoltage};	
            //move to next OD index
            index++;
            // Application - Measured bus voltage of slave 1
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_BUS_VOLTAGE, 1, CO_OBJ_____R_), CO_TUNSIGNED16, (CO_DATA)&hObjDataMotor2BusVoltage};			    
            //move to next OD index
            index++;
        
            // Application - Measured motor Iq of master
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_MOTOR_TEMP, 0, CO_OBJ_____R_), CO_TSIGNED16, (CO_DATA)&hObjDataMotor1Temp};
            //move to next OD index
            index++;
            // Application - Measured motor Iq of slave 1
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_MOTOR_TEMP, 1, CO_OBJ_____R_), CO_TSIGNED16, (CO_DATA)&hObjDataMotor2Temp};			                
            //move to next OD index
            index++;
        
            // Application - Measured motor Id of master
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_HEATSINK_TEMP, 0, CO_OBJ_____R_), CO_TSIGNED16, (CO_DATA)&hObjDataHeatsink1Temp};
            //move to next OD index
            index++;
            // Application - Measured motor Id of slave 1        
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_HEATSINK_TEMP, 1, CO_OBJ_____R_), CO_TSIGNED16, (CO_DATA)&hObjDataHeatsink2Temp};			        
            //move to next OD index
            index++;
        
            // Application - Motor state of master
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_MOTOR_STATE, 0, CO_OBJ_____R_), CO_TUNSIGNED16, (CO_DATA)&hObjDataMotor1State};
            //move to next OD index
            index++;
            // Application - Motor state of slave 1        
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_MOTOR_STATE, 1, CO_OBJ_____R_), CO_TUNSIGNED16, (CO_DATA)&hObjDataMotor2State};			            
            //move to next OD index
            index++;
        
            // Application - Motor faults of master
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_MOTOR_OCC_FAULTS, 0, CO_OBJ_____R_), CO_TUNSIGNED16, (CO_DATA)&hObjDataMotor1OccuredFaults};
            //move to next OD index
            index++;
            // Application - Motor faults of slave 1
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_MOTOR_OCC_FAULTS, 1, CO_OBJ_____R_), CO_TUNSIGNED16, (CO_DATA)&hObjDataMotor2OccuredFaults};		
            //move to next OD index
            index++;
        
            // Application - Motor faults of master
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_MOTOR_CUR_FAULTS, 0, CO_OBJ_____R_), CO_TUNSIGNED16, (CO_DATA)&hObjDataMotor1CurrentFaults};
            //move to next OD index
            index++;
            // Application - Motor faults of slave 1
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_MOTOR_CUR_FAULTS, 1, CO_OBJ_____R_), CO_TUNSIGNED16, (CO_DATA)&hObjDataMotor2CurrentFaults};		
            //move to next OD index
            index++;
        
            // Application - Reference torque to master motor
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_MOTOR_TORQUE_REF, 0, CO_OBJ_____RW), CO_TSIGNED16, (CO_DATA)&hObjDataMotor1TorqRef};
            //move to next OD index
            index++;
            // Application - Reference torque to slave motor 1        
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_MOTOR_TORQUE_REF, 1, CO_OBJ_____RW), CO_TSIGNED16, (CO_DATA)&hObjDataMotor2TorqRef};
            //move to next OD index
            index++;
        
            // Application - Start bit to activate master
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_MOTOR_START, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataMotor1Start};
            //move to next OD index
            index++;
            // Application - Start bit to activate slave motor 1
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_MOTOR_START, 1, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataMotor2Start};		            
            //move to next OD index
            index++;
        
            // Application - Bit to acknowledge motor fault master
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_FAULT_ACK, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataMotor1FaultAck};
            //move to next OD index
            index++;
            // Application - Bit to acknowledge motor fault slave 1
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_FAULT_ACK, 1, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataMotor2FaultAck};		            
            //move to next OD index
            index++;
        
            /***********GNR2 User Data configuration OBJECTS MODULE********************************/
    
            //Application - Inform what user data was upadted
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_DEVICE_TURNNING_OFF, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataDeviceTurnningOff};
            //move to next OD index
            index++;
        
            //Application - Inform if user data was upadted or is being upadted.
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_KEY_USER_DATA_CONFIG, 0, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataKeyUserDataConfig};
            //move to next OD index
            index++;
        
            /************************GNR2-Throttle/Pedal Assist OBJECTS MODULE*************************/ 
            // Here they are being linked in the OD.
            // IN the dictionary they are reponsible to hold the configuration
            // of the Throttle/Pedal Assist parameters.
            // Theses configuration can be read and write using SDO services.
        
            //Application - Percentage of the available max motor power that the PAS algorithm can use. 
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_MAX_TORQUE_RATIO, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasMaxTorqueRatio};
            //move to next OD index
            index++;
        
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_DETECTION_STARTUP, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasTorqueStartupSpeed};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_DETECTION_STARTUP, 1, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasTorqueStartupThreshold};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_DETECTION_STARTUP, 2, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasCadenceStartupNumbPulses};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_DETECTION_STARTUP, 3, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataPasCadenceStartupWindows};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_DETECTION_STARTUP, 4, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasAlgorithmStartup};
            //move to next OD index
            index++;
                                          
            //Application - How much the motor multiplies the torque sensor input from user. Only relevant for torque PAS.
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_TORQUE_SENSOR_MULTIPLIER, 0, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataTorqueSensorMultiplier[0]};
            //move to next OD index
            index++;
            //Application - How much the motor multiplies the torque sensor input from user. Only relevant for torque PAS.
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_TORQUE_SENSOR_MULTIPLIER, 1, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataTorqueSensorMultiplier[1]};
            //move to next OD index
            index++;
            //Application - How much the motor multiplies the torque sensor input from user. Only relevant for torque PAS.
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_TORQUE_SENSOR_MULTIPLIER, 2, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataTorqueSensorMultiplier[2]};
            //move to next OD index
            index++;
            //Application - How much the motor multiplies the torque sensor input from user. Only relevant for torque PAS.
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_TORQUE_SENSOR_MULTIPLIER, 3, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataTorqueSensorMultiplier[3]};
            //move to next OD index
            index++;
            //Application - How much the motor multiplies the torque sensor input from user. Only relevant for torque PAS.
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_TORQUE_SENSOR_MULTIPLIER, 4, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataTorqueSensorMultiplier[4]};
            //move to next OD index
            index++;
            //Application - How much the motor multiplies the torque sensor input from user. Only relevant for torque PAS.
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_TORQUE_SENSOR_MULTIPLIER, 5, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataTorqueSensorMultiplier[5]};
            //move to next OD index
            index++;
            //Application - How much the motor multiplies the torque sensor input from user. Only relevant for torque PAS.
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_TORQUE_SENSOR_MULTIPLIER, 6, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataTorqueSensorMultiplier[6]};
            //move to next OD index
            index++;
            //Application - How much the motor multiplies the torque sensor input from user. Only relevant for torque PAS.
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_TORQUE_SENSOR_MULTIPLIER, 7, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataTorqueSensorMultiplier[7]};
            //move to next OD index
            index++;
            //Application - How much the motor multiplies the torque sensor input from user. Only relevant for torque PAS.
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_TORQUE_SENSOR_MULTIPLIER, 8, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataTorqueSensorMultiplier[8]};
            //move to next OD index
            index++;

            
            //Application - The min power on PAS level. 
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_MIN_TORQUE, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasLevelMinTorque[0]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_MIN_TORQUE, 1, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasLevelMinTorque[1]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_MIN_TORQUE, 2, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasLevelMinTorque[2]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_MIN_TORQUE, 3, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasLevelMinTorque[3]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_MIN_TORQUE, 4, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasLevelMinTorque[4]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_MIN_TORQUE, 5, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasLevelMinTorque[5]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_MIN_TORQUE, 6, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasLevelMinTorque[6]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_MIN_TORQUE, 7, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasLevelMinTorque[7]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_MIN_TORQUE, 8, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasLevelMinTorque[8]};
            //move to next OD index
            index++;
        
            //Application - The speed up to which this PAS level will give motor assistance. 
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_LEVEL_SPEED, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasLeveSpeed[0]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_LEVEL_SPEED, 1, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasLeveSpeed[1]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_LEVEL_SPEED, 2, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasLeveSpeed[2]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_LEVEL_SPEED, 3, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasLeveSpeed[3]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_LEVEL_SPEED, 4, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasLeveSpeed[4]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_LEVEL_SPEED, 5, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasLeveSpeed[5]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_LEVEL_SPEED, 6, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasLeveSpeed[6]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_LEVEL_SPEED, 7, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasLeveSpeed[7]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_LEVEL_SPEED, 8, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasLeveSpeed[8]};
            //move to next OD index
            index++;

        
            //Application - The speed up to which this PAS level will give motor assistance. 
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_MAX_TORQUE, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasLevelMaxTorque[0]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_MAX_TORQUE, 1, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasLevelMaxTorque[1]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_MAX_TORQUE, 2, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasLevelMaxTorque[2]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_MAX_TORQUE, 3, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasLevelMaxTorque[3]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_MAX_TORQUE, 4, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasLevelMaxTorque[4]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_MAX_TORQUE, 5, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasLevelMaxTorque[5]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_MAX_TORQUE, 6, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasLevelMaxTorque[6]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_MAX_TORQUE, 7, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasLevelMaxTorque[7]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_MAX_TORQUE, 8, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasLevelMaxTorque[8]};
            //move to next OD index
            index++;

        
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_MAX_SPEED, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataMaxSpeed};
            //move to next OD index
            index++;
        
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_WALK_MODE_SPEED, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataWalkModeSpeed};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_WALK_MODE_SPEED, 1, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataWalkModeMaxTorque};
            //move to next OD index
            index++;
                     
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_WALK_MODE_SPEED, 2, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataWalkModeAccelRampType};
            //move to next OD index
            index++;

            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_WALK_MODE_SPEED, 3, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataWalkModeAccelRampArg1};
            //move to next OD index
            index++;            
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_BATTERY_VOLTAGE, 0, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataBatteryFullVoltage};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_BATTERY_VOLTAGE, 1, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataBatteryEmptyVoltage};
            //move to next OD index
            index++;
                       
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_WHEELS_DIAMETER, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataWheelDiameter};    
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_WHEELS_DIAMETER, 1, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataWheelPulsePerRotation};    
            //move to next OD index
            index++;
                    
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_WHEELS_DIAMETER, 2, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataWheelDiameterDefault};    
            //move to next OD index
            index++;
            
        
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_VEHICLE_FRONT_LIGHT, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataFrontLightState};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_VEHICLE_FRONT_LIGHT, 1, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataFrontLightDefaultState};
            //move to next OD index
            index++;
        
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_VEHICLE_REAR_LIGHT, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataRearLightState};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_VEHICLE_REAR_LIGHT, 1, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataRearLightDefaultState};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_VEHICLE_REAR_LIGHT, 2, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataRearLightBlinkOnBrake};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_VEHICLE_REAR_LIGHT, 3, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataRearLightBlinkPeriod};
            //move to next OD index
            index++;
                        
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_VEHICLE_REAR_LIGHT, 4, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataRearLightBlinkDutyCycle};
            //move to next OD index
            index++;
            
            //User to master and/or slave write showing it is present(no lost on master/slave communication).
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_MASTER_SLAVE_PRESENT, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataMasterSlavePresent};
            //move to next OD index
            index++;
            
            //User to master and/or slave write showing it is present(no lost on master/slave communication).
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(COD_OD_REG_PAS_SENSOR, 0, CO_OBJ_____R_), CO_TUNSIGNED16, (CO_DATA)&bObjDataPasPedalRPM};
            //move to next OD index
            index++;
            
            //User to master and/or slave write showing it is present(no lost on master/slave communication).
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(COD_OD_REG_PAS_SENSOR, 1, CO_OBJ_____R_), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasTorqueForcePercent};
            //move to next OD index
            index++;
            
            //User to master and/or slave write showing it is present(no lost on master/slave communication).
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(COD_OD_REG_PAS_SENSOR, 2, CO_OBJ_____R_), CO_TUNSIGNED16, (CO_DATA)&bObjDataPasTorqueForceWatts};
            //move to next OD index
            index++;
            
            //User to master and/or slave write showing it is present(no lost on master/slave communication).
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(COD_OD_REG_PAS_SENSOR, 3, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasNbMagnetsPerTurn};
            //move to next OD index
            index++;
            
            //User to master and/or slave write showing it is present(no lost on master/slave communication).
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(COD_OD_REG_PAS_SENSOR, 4, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataPasTorqueInputMin};
            //move to next OD index
            index++;
            
            //User to master and/or slave write showing it is present(no lost on master/slave communication).
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(COD_OD_REG_PAS_SENSOR, 5, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataPasTorqueInputMax};
            //move to next OD index
            index++;
            
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_CONFIG_SCREEN_PROTOCOL, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataConfigScreenProtocol};
            //move to next OD index
            index++;

            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_BATTERY_DC_CURRENT, 0, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataConfigBatteryMaxPeakDCCurrent};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_BATTERY_DC_CURRENT, 1, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataConfigBatteryContinuousDCCurrent};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_BATTERY_DC_CURRENT, 2, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataConfigBatteryPeakCurrentDuration};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_BATTERY_DC_CURRENT, 3, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataConfigBatteryPeakCurrentDeratingDuration};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_CONTROLLER_THROTTLE, 0, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataConfigThrottleAdcValue};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_CONTROLLER_THROTTLE, 1, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataConfigThrottleGetSetValue};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_CONTROLLER_THROTTLE, 2, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataConfigThrottleAdcOffset};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_CONTROLLER_THROTTLE, 3, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataConfigThrottleAdcMax};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_CONTROLLER_THROTTLE, 4, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataConfigThrottleBlockOff};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_CONTROLLER_THROTTLE, 5, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataConfigThrottleMaxSpeed};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_CONTROLLER_THROTTLE, 6, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataConfigThrottleAccelRampType};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_CONTROLLER_THROTTLE, 7, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataConfigThrottleAccelRampArg1};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_DETECTION_RUNNING, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasTorqueRunningThreshold};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_DETECTION_RUNNING, 1, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasCadenceRunningNumbPulses};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_DETECTION_RUNNING, 2, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataPasCadenceRunningWindows};
			//move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_DETECTION_RUNNING, 3, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasAlgorithmRunning};
			//move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_ACCEL_RAMP, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPas1AccelRampType};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_ACCEL_RAMP, 1, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataPas1AccelRampArg1};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_ACCEL_RAMP, 2, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPas2AccelRampType};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_ACCEL_RAMP, 3, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataPas2AccelRampArg1};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_ACCEL_RAMP, 4, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPas3AccelRampType};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_ACCEL_RAMP, 5, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataPas3AccelRampArg1};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_ACCEL_RAMP, 6, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPas4AccelRampType};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_ACCEL_RAMP, 7, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataPas4AccelRampArg1};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_ACCEL_RAMP, 8, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPas5AccelRampType};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_ACCEL_RAMP, 9, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataPas5AccelRampArg1};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_ACCEL_RAMP, 10, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPas6AccelRampType};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_ACCEL_RAMP, 11, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataPas6AccelRampArg1};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_ACCEL_RAMP, 12, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPas7AccelRampType};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_ACCEL_RAMP, 13, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataPas7AccelRampArg1};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_ACCEL_RAMP, 14, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPas8AccelRampType};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_ACCEL_RAMP, 15, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataPas8AccelRampArg1};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_ACCEL_RAMP, 16, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPas9AccelRampType};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_ACCEL_RAMP, 17, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataPas9AccelRampArg1};
            //move to next OD index
            index++;
            
             
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_DECEL_RAMP, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPas1DecelRampType};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_DECEL_RAMP, 1, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataPas1DecelRampArg1};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_DECEL_RAMP, 2, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPas2DecelRampType};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_DECEL_RAMP, 3, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataPas2DecelRampArg1};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_DECEL_RAMP, 4, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPas3DecelRampType};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_DECEL_RAMP, 5, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataPas3DecelRampArg1};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_DECEL_RAMP, 6, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPas4DecelRampType};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_DECEL_RAMP, 7, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataPas4DecelRampArg1};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_DECEL_RAMP, 8, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPas5DecelRampType};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_DECEL_RAMP, 9, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataPas5DecelRampArg1};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_DECEL_RAMP, 10, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPas6DecelRampType};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_DECEL_RAMP, 11, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataPas6DecelRampArg1};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_DECEL_RAMP, 12, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPas7DecelRampType};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_DECEL_RAMP, 13, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataPas7DecelRampArg1};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_DECEL_RAMP, 14, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPas8DecelRampType};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_DECEL_RAMP, 15, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataPas8DecelRampArg1};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_DECEL_RAMP, 16, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPas9DecelRampType};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_DECEL_RAMP, 17, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataPas9DecelRampArg1};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_CONFIG_SPEED_FOR_TORQUE_FILTER, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataConfigSpeedForTorqueFilter[0]};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_CONFIG_SPEED_FOR_TORQUE_FILTER, 1, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataConfigSpeedForTorqueFilter[1]};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_CONFIG_TORQUE_FILTER_FOR_SPEED, 0, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataConfigTorqueFilterForSpeed[0]};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_CONFIG_TORQUE_FILTER_FOR_SPEED, 1, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataConfigTorqueFilterForSpeed[1]};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_CONFIG_TORQUE_FILTER_FOR_SPEED, 2, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataConfigTorqueFilterForSpeed[2]};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_CONFIG_TORQUE_FILTER_FOR_SPEED, 3, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataConfigTorqueFilterForSpeed[3]};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_CONFIG_TORQUE_FILTER_FOR_SPEED, 4, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataConfigTorqueFilterForSpeed[4]};
            //move to next OD index
            index++;
            
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_CONFIG_TORQUE_FILTER_FOR_SPEED, 5, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&bObjDataConfigTorqueFilterForSpeed[5]};
            //move to next OD index
            index++;
            
            
            
            //Application - Used to control the firmware update procedure.
            //subindex 0 is used to receive command from the IOT module to control the DFU process.
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_FIRMWAREUPDATE_MEMORY, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)(&bObjOtaCommand)};
            //move to next OD index
            index++;
            //Used to inform about the ongoing state of the DFU process and report any error.
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_FIRMWAREUPDATE_MEMORY, 1, CO_OBJ_____R_), CO_TUNSIGNED8, (CO_DATA)(&bObjOtaStatus)};
            //move to next OD index
            index++;
            //Used to receive the data frame(part of the firware file).
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_FIRMWAREUPDATE_MEMORY, 2, CO_OBJ_____RW), CO_TDOMAIN, (CO_DATA)(&bObjFirmwareUpdateDomain)};
            //move to next OD index
            index++;
            //Used to inform the number of the last data frame received.
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_FIRMWAREUPDATE_MEMORY, 3, CO_OBJ_____R_), CO_TUNSIGNED16, (CO_DATA)(&bObjOtaFrameCount)}; 
            //move to next OD index
            index++;
          
            //mark end of used objects
            GNR2_OD[index] = (struct CO_OBJ_T)CO_OBJ_DICT_ENDMARK;
            
            //add the CAOPEN ID : GNR2_MASTER_NODE_ID or GNR2_SLAVE_NODE_ID.
            //if true use master config, MASTER_ID == 0x01(CANOPEN 0x601)
            if (deviceType == true)
            {
                //Master device must use master node id.
                GnR2ModuleSpec.NodeId = GNR2_MASTER_NODE_ID;
            }
            else
            {
                //Slave device must use slave node id.
                GnR2ModuleSpec.NodeId = GNR2_SLAVE_NODE_ID;
            }
        
        break;
            
        //don't add an entrie.
        default:
            
        
        break;
        
    }
    
}

/**
  Function used to add a specific setup in the OD.
 */
static void CO_GnrOdSetup(const uint16_t arraySetup, bool deviceType)
{
    
    //add the Object ID to the OD entrie.
    CO_addObj(arraySetup, deviceType);
    
}
