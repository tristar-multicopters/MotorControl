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
#define GNR2_OBJ_N         128u                /* Object dictionary max size  */

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
int16_t  hObjDataMotor1Temp                 = 0;
int16_t  hObjDataHeatsink1Temp              = 0;
uint16_t hObjDataMotor1State                = 0;
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

uint8_t  bObjDataMotor2Start                = 0;
int16_t  hObjDataMotor2TorqRef              = 0;
uint8_t  bObjDataMotor2FaultAck             = 0;
	
/*****Allocate global variables for GNR-IOT objects*****/
int16_t  bObjDataSpeedMeas                  = 0;
uint8_t  bObjDataSOC                        = 0;
uint8_t  bObjDataPAS                        = DEFAULT_PAS_LEVEL; // The default PAS level should be 1
uint8_t  bObjDataMaxPAS                     = 0;
uint32_t hObjDataFwVersion                  = 0;
uint16_t hObjDataPowerMeas                  = 0;
uint16_t hObjDataMaxPower                   = 0;
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

//variable associated with CO_OD_REG_PAS_ALGORITHM.
uint8_t bObjDataPasAlgorithm                = 0;

//variable associated with CO_OD_REG_PAS_MAX_POWER.
uint8_t bObjDataPasMaxPower                 = 0;

//variable associated with CO_OD_REG_TORQUE_MINIMUM_THRESHOLD.
uint8_t bObjDataTorqueMinimumThreshold      = 0;

//variable associated with CO_OD_REG_TORQUE_MINIMUM_THRESHOLD.
uint8_t bObjDataTorqueMinimumThresholdStartup  = 0;                     

//variable associated with CO_OD_REG_TORQUE_MINIMUM_THRESHOLD.
uint8_t bObjDataTorqueStartupSpeed           = 0;

//variable associated with CO_OD_REG_TORQUE_SENSOR_MULTIPLIER.
uint8_t bObjDataTorqueSensorMultiplier      = 0;

//variable associated with CO_OD_REG_TORQUE_MAX_SPEED.
uint8_t bObjDataTorqueMaxSpeed              = 0;

//variable associated with CO_OD_REG_CADENCE_HYBRID_LEVEL.
uint8_t bObjDataCadenceHybridLeveSpeed[10]  = {0};

//variable associated with CO_OD_REG_TORQUE_LEVEL_POWER.
uint8_t bObjDataTorqueLevelPower[10]        = {0};

//variable associated with CO_OD_REG_MAX_SPEED.
uint8_t bObjDataMaxSpeed                    = 0;

//variable associated with CO_OD_REG_WALK_MODE_SPEED.
uint8_t bObjDataWalkModeSpeed               = 0;

//variable associated with CO_OD_REG_WHEELS_DIAMETER
uint8_t bObjDataWheelDiamater               = 0;

//variable associated with CO_OD_REG_VEHICLE_FRONT_LIGHT
uint8_t bObjDataFrontLightState             = 0;

//variable associated with CO_OD_REG_VEHICLE_REAR_LIGHT
uint8_t bObjDataRearLightState              = 0;

//variable associated with CO_OD_REG_MASTER_SLAVE_PRESENT
uint8_t bObjDataMasterSlavePresent = 0;

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
    static uint8_t index = 0;
    
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
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_SPEED_MEASURE, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataSpeedMeas}; 
            //move to next OD index
            index++;
            // Application - Inst Power
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_POWER_MEASURE, 0, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA)&hObjDataPowerMeas};  
            //move to next OD index
            index++;
            // Application - State of Charge
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_SOC,           0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataSOC}; 
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
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_MAX_POWER,     0, CO_OBJ_____RW), CO_TSIGNED16, (CO_DATA)&hObjDataMaxPower}; 
            //move to next OD index
            index++;
            // Application - Error State
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_ERR_STATE,     0, CO_OBJ_____RW), CO_TUNSIGNED32, (CO_DATA)&hObjDataErrorState};
            //move to next OD index
            index++;
            // Application - Serial Number High side
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_SERIAL_NB,     0, CO_OBJ_____RW), CO_TUNSIGNED32, (CO_DATA)&wObjDataSerialNbH};  
            //move to next OD index
            index++;
            // Application - Serial Number Low side
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_SERIAL_NB,     1, CO_OBJ_____RW), CO_TUNSIGNED32, (CO_DATA)&wObjDataSerialNbL};
            //move to next OD index
            index++;
            // Application - Firmware Version
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_FW_VERSION,    0, CO_OBJ_____RW), CO_TUNSIGNED32, (CO_DATA)&hObjDataFwVersion};   
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
    
            // Application - 
            // Torque: based on a multiplier of the torque input. 
            // Cadence: based on pedaling speed.
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_ALGORITHM, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasAlgorithm};
            //move to next OD index
            index++;
        
            //Application - Percentage of the available max motor power that the PAS algorithm can use. 
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_PAS_MAX_POWER, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataPasMaxPower};
            //move to next OD index
            index++;
        
            //Application - Torque threshold for starting motor assistance from 0 speed. Only relevant for torque/hybrid PAS. 
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_TORQUE_MINIMUM_THRESHOLD, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataTorqueMinimumThreshold};
            //move to next OD index
            index++;
        
            //Application - Offset for pedal torque sensor to torque linear transformation during the startup in %
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_TORQUE_MINIMUM_THRESHOLD, 1, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataTorqueMinimumThresholdStartup};
            //move to next OD index
            index++;
        
            //Application - Speed under which the Startup pedal torque sensor offset is used in km/h 
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_TORQUE_MINIMUM_THRESHOLD, 2, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataTorqueStartupSpeed};
            //move to next OD index
            index++;
        
            //Application - How much the motor multiplies the torque sensor input from user. Only relevant for torque/hybrid PAS. 
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_TORQUE_SENSOR_MULTIPLIER, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataTorqueSensorMultiplier};
            //move to next OD index
            index++;
        
            //Application - not defined. to do. 
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_TORQUE_MAX_SPEED, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataTorqueMaxSpeed};
            //move to next OD index
            index++;
        
            //Application - The speed up to which this PAS level will give motor assistance. 
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataCadenceHybridLeveSpeed[0]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 1, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataCadenceHybridLeveSpeed[1]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 2, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataCadenceHybridLeveSpeed[2]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 3, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataCadenceHybridLeveSpeed[3]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 4, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataCadenceHybridLeveSpeed[4]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 5, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataCadenceHybridLeveSpeed[5]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 6, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataCadenceHybridLeveSpeed[6]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 7, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataCadenceHybridLeveSpeed[7]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 8, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataCadenceHybridLeveSpeed[8]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 9, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataCadenceHybridLeveSpeed[9]};
            //move to next OD index
            index++;
        
            //Application - The speed up to which this PAS level will give motor assistance. 
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataTorqueLevelPower[0]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 1, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataTorqueLevelPower[1]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 2, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataTorqueLevelPower[2]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 3, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataTorqueLevelPower[3]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 4, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataTorqueLevelPower[4]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 5, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataTorqueLevelPower[5]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 6, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataTorqueLevelPower[6]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 7, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataTorqueLevelPower[7]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 8, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataTorqueLevelPower[8]};
            //move to next OD index
            index++;
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 9, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataTorqueLevelPower[9]};
            //move to next OD index
            index++;
        
            //Application - The max speed that the throttle will bring the vehicle to. 
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_MAX_SPEED, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataMaxSpeed};
            //move to next OD index
            index++;
        
            //Application - Speed that the walk mode of the vehicle goes up to. 
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_WALK_MODE_SPEED, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataWalkModeSpeed};
            //move to next OD index
            index++;
    
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_WHEELS_DIAMETER, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataWheelDiamater};    
            //move to next OD index
            index++;
        
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_VEHICLE_FRONT_LIGHT, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataFrontLightState};
            //move to next OD index
            index++;
        
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_VEHICLE_REAR_LIGHT, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataRearLightState};
            //move to next OD index
            index++;
        
            //User to master and/or slave write showing it is present(no lost on master/slave communication).
            GNR2_OD[index] = (struct CO_OBJ_T){CO_KEY(CO_OD_REG_MASTER_SLAVE_PRESENT, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA)&bObjDataMasterSlavePresent};
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
