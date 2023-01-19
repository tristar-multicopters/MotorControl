/**
*  co_gnr2_specs.c
*  Module for defining the CANOpen specs of GNR2
*/

// ==================== INCLUDES ======================== //

#include "co_gnr2_specs.h"
#include "gnr_parameters.h"

#include "co_can_ra6t2.h"           /* CAN driver                  */
#include "co_timer_ra6t2.h"         /* Timer driver                */
#include "co_nvm_ra6t2.h"           /* NVM driver                  */

// ==================== PRIVATE DEFINES ======================== //

#if GNR_MASTER
#define GNR2_NODE_ID       GNR2_MASTER_NODE_ID          /* CANopen node ID             */
#else
#define GNR2_NODE_ID       GNR2_SLAVE_NODE_ID           /* CANopen node ID             */
#endif

#define GNR2_BAUDRATE      500000u             /* CAN baudrate                */
#define GNR2_TMR_N         16u                 /* Number of software timers   */
#define GNR2_TICKS_PER_SEC 2000u               /* Timer clock frequency in Hz */
#define GNR2_OBJ_N         128u                /* Object dictionary max size  */

#if GNR_MASTER

#define GENERATE_SYNC           1           /* Generate or not the SYNC frame. 1 for yes, 0 for no. */
#define STD_ID_SYNC             0x80        /* SYNC frame standard ID */
#define SYNC_PERIOD_US          50000       /* SYNC frame period in us */

#define USE_EMCY                0           /* Use emergency frame */
#define STD_ID_EMCY             0           /* Emergency frame standard ID */

#define HEARTBEAT_PRODUCE_PERIOD_MS         250                     /* Period of the heartbeat frame */
#define HEARTBEAT_CONSUME_PERIOD_MS         500                     /* Max time to wait for receiving next heartbeat */  
#define HEARTBEAT_CONSUME_NODE_ID           GNR2_SLAVE_NODE_ID      /* Node ID of the monitored heartbeat */

#define USE_RPDO                1           /* Use or not RPDO. 1 for yes, 0 for no. */
#define STD_ID_RPDO1            0x300        /* Standard ID of RPDO 1 */
#define STD_ID_RPDO2            0x301        /* Standard ID of TPDO 2 */
#define RPDO_TRANSMISSION_TYPE  254         /* RPDO transmission type. Event-driven (manufacturer-specific) is chosen */
#define RPDO_PERIOD_MS          50          /* Period in ms for RPDO processing */

#define USE_TPDO                1           /* Use or not TPDO. 1 for yes, 0 for no. */
#define STD_ID_TPDO1            0x400        /* Standard ID of TPDO 1 */
#define STD_ID_TPDO2            0x401        /* Standard ID of TPDO 2 */
#define TPDO_TRANSMISSION_TYPE  254         /* TPDO transmission type. Event-driven (manufacturer-specific) is chosen */
#define TPDO_PERIOD_MS          50          /* Period in ms for TPDO transmission */

#define USE_SSDO                1           /* Use or not SSDO. 1 for yes, 0 for no. */
#define USE_CSDO                1           /* Use or not CSDO. 1 for yes, 0 for no. */

#else

#define GENERATE_SYNC           0           /* Generate or not the SYNC frame. 1 for yes, 0 for no. */
#define STD_ID_SYNC             0x80        /* SYNC frame standard ID */
#define SYNC_PERIOD_US          0           /* SYNC frame period in us */

#define USE_EMCY                0           /* Use emergency frame */
#define STD_ID_EMCY             0           /* Emergency frame standard ID */

#define HEARTBEAT_PRODUCE_PERIOD_MS         250                     /* Period of the heartbeat frame */
#define HEARTBEAT_CONSUME_PERIOD_MS         500                     /* Max time to wait for receiving next heartbeat */  
#define HEARTBEAT_CONSUME_NODE_ID           GNR2_MASTER_NODE_ID     /* Node ID of the monitored heartbeat */

#define USE_RPDO                1           /* Use or not RPDO. 1 for yes, 0 for no. */
#define STD_ID_RPDO1            0x400        /* Standard ID of RPDO 1 */
#define STD_ID_RPDO2            0x401        /* Standard ID of TPDO 2 */
#define RPDO_TRANSMISSION_TYPE  0           /* RPDO transmission type. Synchronous after SYNC is chosen */
#define RPDO_PERIOD_MS          0           /* Period in ms for RPDO processing. Not applicable for synchronous transmission type */

#define USE_TPDO                1           /* Use or not TPDO. 1 for yes, 0 for no. */
#define STD_ID_TPDO1            0x300        /* Standard ID of TPDO 1 */
#define STD_ID_TPDO2            0x301        /* Standard ID of TPDO 2 */
#define TPDO_TRANSMISSION_TYPE  1           /* Period in ms for TPDO transmission. Not applicable for synchronous transmission type */
#define TPDO_PERIOD_MS          0           /* Period in ms for RPDO processing. Not applicable for synchronous transmission type� */

#define USE_SSDO                1           /* Use or not SSDO. 1 for yes, 0 for no. */
#define USE_CSDO                1           /* Use or not CSDO. 1 for yes, 0 for no. */

#endif


// ==================== PRIVATE VARIABLES ======================== //

/* Allocate global variables for runtime value of objects */
CO_HBCONS AppHbConsumer_1 = {
    .Time = HEARTBEAT_CONSUME_PERIOD_MS,
    .NodeId = HEARTBEAT_CONSUME_NODE_ID,
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
uint8_t  bObjDataPAS                        = 0;
uint8_t  bObjDataMaxPAS                     = 0;
uint16_t hObjDataFwVersion                  = 0;
uint16_t hObjDataPowerMeas                  = 0;
uint16_t hObjDataMaxPower                   = 0;
uint16_t hObjDataErrorState                 = 0;
uint32_t wObjDataSerialNbL                  = 0;
uint32_t wObjDataSerialNbH                  = 0;

/*****Allocate global variables for data flash update Gnr objects*****/

//variable associated with CO_OD_REG_USER_DATA_CONFIG_BIKE_MODEL .
uint16_t bObjDataUserDataConfig             = 0;

//variable associated with CO_OD_REG_KEY_USER_DATA_CONFIG.
uint16_t bObjDataKeyUserDataConfig          = 0;

/*****Allocate global variables for Throttle/Pedal Assist Gnr objects*****/

//variable associated with CO_OD_REG_PAS_ALGORITHM.
uint8_t bObjDataPasAlgorithm                = 0;

//variable associated with CO_OD_REG_PAS_MAX_POWER.
uint8_t bObjDataPasMaxPower                 = 0;

//variable associated with CO_OD_REG_TORQUE_MINIMUM_THRESHOLD.
uint8_t bObjDataTorqueMinimumThreshold      = 0;

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

/* define the static object dictionary */
#if GNR_IOT
struct CO_OBJ_T GNR2_OD[GNR2_OBJ_N] = {
    // Mandatory entries
	  {CO_KEY(0x1000, 0, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (uintptr_t)0},							    // Device Type
    {CO_KEY(0x1001, 0, CO_UNSIGNED8 |CO_OBJ____R_), 0, (uintptr_t)&hObjDataErrorRegister},				    // Error Register
    
    //master needs to send a SYNC message to make slave node 
    //respond to the master by TPDO at the very same time.
    //when using the macro CO_COBID_SYNC_STD the GENERATE_SYNC define
    //configured the device to produces SYNC messages.
    #if SUPPORT_SLAVE
    {CO_KEY(0x1005, 0, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (uintptr_t)CO_COBID_SYNC_STD(GENERATE_SYNC, STD_ID_SYNC)},		// COB-ID SYNC Message
    #else
    //on this case the device is configured to cosumes SYNC message. 
    //but IOT module is not sending SYNC messages.
    {CO_KEY(0x1005, 0, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (uintptr_t)STD_ID_SYNC},							// COB-ID SYNC Message
    #endif
    
    #if SUPPORT_SLAVE
    {CO_KEY(0x1006, 0, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (uintptr_t)SYNC_PERIOD_US},                                      // SYNC period
    //the idea of this service is to allow onde node to send a fatal error to the rest of the network.
    //but application needs to detect this error and send it. this is not send automatically 
    //by the can layer.
    //on this configuration EMCY message is not enabled.
    {CO_KEY(0x1014, 0, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (uintptr_t)CO_COBID_EMCY_STD(USE_EMCY, STD_ID_EMCY)},		    // COB-ID EMCY Message
    {CO_KEY(0x1016, 0, CO_UNSIGNED8|CO_OBJ_D__R_), 0, (uintptr_t)1},				                        // Consumer Heartbeat max index    
    {CO_KEY(0x1016, 1,               CO_OBJ____RW), CO_THB_CONS, (uintptr_t)&AppHbConsumer_1},				// Consumer Heartbeat parameters
    #endif
    
    {CO_KEY(0x1017, 0, CO_UNSIGNED16|CO_OBJ____RW), 0, (uintptr_t)&hObjDataProdHbTime},							// Producer Heartbeat Time
    {CO_KEY(0x1018, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (uintptr_t)4},								// Identity - Highest Sub Index
    {CO_KEY(0x1018, 1, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (uintptr_t)0},								// Identity - Vendor ID
    {CO_KEY(0x1018, 2, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (uintptr_t)0},								// Identity - Product Code
    {CO_KEY(0x1018, 3, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (uintptr_t)0},								// Identity - Revision Number
    {CO_KEY(0x1018, 4, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (uintptr_t)0},								// Identity - Serial Number    
 
    //server sdo
    {CO_KEY(0x1200, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (uintptr_t)2},								// SDO Srv Parameter - Highest Sub Index
    {CO_KEY(0x1200, 1, CO_UNSIGNED32|CO_OBJ_DN_R_), 0, CO_COBID_SDO_REQUEST()},						// SDO Srv Parameter - COB-ID Client to Server
    {CO_KEY(0x1200, 2, CO_UNSIGNED32|CO_OBJ_DN_R_), 0, CO_COBID_SDO_RESPONSE()},					// SDO Srv Parameter - COB-ID Server to Client  
    
    // SDO Client - second client to request data from the slaver.
    #if SUPPORT_SLAVE
    {CO_KEY(0x1280, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (uintptr_t)3},						                // SDO Client Parameter - Highest Sub Index
    {CO_KEY(0x1280, 1, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_COBID_SDO_REQUEST()},			                    // SDO Client Parameter - COB-ID Client to Server
    {CO_KEY(0x1280, 2, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_COBID_SDO_RESPONSE()},			                // SDO Client Parameter - COB-ID Server to Client
    {CO_KEY(0x1280, 3, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (uintptr_t)GNR2_SLAVE_NODE_ID},
    
    //necessary to allow master read slaver.
    // RPDO 1
    {CO_KEY(0x1400, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (uintptr_t)5},						                // RPDO1 Parameter - Highest Sub Index
    {CO_KEY(0x1400, 1, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_COBID_RPDO_STD(USE_RPDO, STD_ID_RPDO1)},			// RPDO1 Parameter - COB-ID RPDO
    {CO_KEY(0x1400, 2, CO_UNSIGNED8|CO_OBJ_D__R_), 0, RPDO_TRANSMISSION_TYPE},			                    // RPDO1 Parameter - Transmission type
    {CO_KEY(0x1400, 5, CO_UNSIGNED16|CO_OBJ_D__R_), 0, RPDO_PERIOD_MS},			                            // RPDO1 Parameter - Event period
    {CO_KEY(0x1600, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (uintptr_t)4},						                // RPDO1 Mapping - Number of mapped object in PDO
    {CO_KEY(0x1600, 1, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_LINK(CO_OD_REG_MOTOR_STATE, 1, 16)},			    // RPDO1 Mapping - Object 1
    {CO_KEY(0x1600, 2, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_LINK(CO_OD_REG_MOTOR_OCC_FAULTS, 1, 16)},			// RPDO1 Mapping - Object 2
    {CO_KEY(0x1600, 3, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_LINK(CO_OD_REG_MOTOR_CUR_FAULTS, 1, 16)},			// RPDO1 Mapping - Object 2
    {CO_KEY(0x1600, 4, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_LINK(CO_OD_REG_MOTOR_SPEED, 1, 16)},			    // RPDO1 Mapping - Object 2
    // TPDO 1
    {CO_KEY(0x1800, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (uintptr_t)6},						                // TPDO1 Parameter - Highest Sub Index
    {CO_KEY(0x1800, 1, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_COBID_TPDO_STD(USE_TPDO, STD_ID_TPDO1)},			// TPDO1 Parameter - COB-ID TPDO
    {CO_KEY(0x1800, 2, CO_UNSIGNED8|CO_OBJ_D__R_), 0, TPDO_TRANSMISSION_TYPE},			                    // TPDO1 Parameter - Transmission type
    {CO_KEY(0x1800, 5, CO_UNSIGNED16|CO_OBJ_D__R_), 0, TPDO_PERIOD_MS},			                            // TPDO1 Parameter - Transmission type
    {CO_KEY(0x1800, 6, CO_UNSIGNED8|CO_OBJ_D__R_), 0, 0},			                                        // TPDO1 Parameter - SYNC start value
    {CO_KEY(0x1A00, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (uintptr_t)2},						                // RPDO1 Mapping - Number of mapped object in PDO
    {CO_KEY(0x1A00, 1, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_LINK(CO_OD_REG_MOTOR_TORQUE_REF, 1, 16)},			// RPDO1 Mapping - Object 1
    {CO_KEY(0x1A00, 2, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_LINK(CO_OD_REG_MOTOR_START, 1, 8)},			    // RPDO1 Mapping - Object 2
    #endif
    
    /**********************GNR2-IOT OBJECTS MODULE*******************************************/
    {CO_KEY(CO_OD_REG_SPEED_MEASURE, 0, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataSpeedMeas},   // Application - Inst Speed
    {CO_KEY(CO_OD_REG_POWER_MEASURE, 0, CO_UNSIGNED16  |CO_OBJ____RW), 0, (uintptr_t)&hObjDataPowerMeas},   // Application - Inst Power
    {CO_KEY(CO_OD_REG_SOC,           0, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataSOC},         // Application - State of Charge
    {CO_KEY(CO_OD_REG_PAS_LEVEL,     0, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataPAS},	        // Application - PAS Level
    {CO_KEY(CO_OD_REG_MAX_PAS,       0, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataMaxPAS},	    // Application - Max PAS Level
    {CO_KEY(CO_OD_REG_MAX_POWER,     0, CO_SIGNED16    |CO_OBJ____RW), 0, (uintptr_t)&hObjDataMaxPower},   // Application - Max Power
    {CO_KEY(CO_OD_REG_ERR_STATE,     0, CO_SIGNED16    |CO_OBJ____RW), 0, (uintptr_t)&hObjDataErrorState},  // Application - Error State
    {CO_KEY(CO_OD_REG_SERIAL_NB,     0, CO_UNSIGNED32  |CO_OBJ____RW), 0, (uintptr_t)&wObjDataSerialNbH},   // Application - Serial Number High side
    {CO_KEY(CO_OD_REG_SERIAL_NB,     1, CO_UNSIGNED32  |CO_OBJ____RW), 0, (uintptr_t)&wObjDataSerialNbL},   // Application - Serial Number Low side
    {CO_KEY(CO_OD_REG_FW_VERSION,    0, CO_UNSIGNED16  |CO_OBJ____RW), 0, (uintptr_t)&hObjDataFwVersion},   // Application - Firmware Version
     
    /************************GNR2 Motor parameters OBJECTS MODULE******************************/
    //master must to have this variables to exchange information with the slave if
    //dual motor setup is enabled.
    {CO_KEY(CO_OD_REG_MOTOR_SPEED, 0, CO_SIGNED16|CO_OBJ____R_), 0, (uintptr_t)&hObjDataMotor1SpeedMeas},			        // Application - Measured motor speed of master
    {CO_KEY(CO_OD_REG_MOTOR_SPEED, 1, CO_SIGNED16|CO_OBJ____R_), 0, (uintptr_t)&hObjDataMotor2SpeedMeas},			        // Application - Measured motor speed of slave 1

    {CO_KEY(CO_OD_REG_BUS_VOLTAGE, 0, CO_UNSIGNED16|CO_OBJ____R_), 0, (uintptr_t)&hObjDataMotor1BusVoltage},			    // Application - Measured bus voltage of master
    {CO_KEY(CO_OD_REG_BUS_VOLTAGE, 1, CO_UNSIGNED16|CO_OBJ____R_), 0, (uintptr_t)&hObjDataMotor2BusVoltage},			    // Application - Measured bus voltage of slave 1

    {CO_KEY(CO_OD_REG_MOTOR_TEMP, 0, CO_SIGNED16|CO_OBJ____R_), 0, (uintptr_t)&hObjDataMotor1Temp},			                // Application - Measured motor Iq of master
    {CO_KEY(CO_OD_REG_MOTOR_TEMP, 1, CO_SIGNED16|CO_OBJ____R_), 0, (uintptr_t)&hObjDataMotor2Temp},			                // Application - Measured motor Iq of slave 1

    {CO_KEY(CO_OD_REG_HEATSINK_TEMP, 0, CO_SIGNED16|CO_OBJ____R_), 0, (uintptr_t)&hObjDataHeatsink1Temp},			        // Application - Measured motor Id of master
    {CO_KEY(CO_OD_REG_HEATSINK_TEMP, 1, CO_SIGNED16|CO_OBJ____R_), 0, (uintptr_t)&hObjDataHeatsink2Temp},			        // Application - Measured motor Id of slave 1

    {CO_KEY(CO_OD_REG_MOTOR_STATE, 0, CO_UNSIGNED16|CO_OBJ____R_), 0, (uintptr_t)&hObjDataMotor1State},			            // Application - Motor state of master
    {CO_KEY(CO_OD_REG_MOTOR_STATE, 1, CO_UNSIGNED16|CO_OBJ____R_), 0, (uintptr_t)&hObjDataMotor2State},			            // Application - Motor state of slave 1

    {CO_KEY(CO_OD_REG_MOTOR_OCC_FAULTS, 0, CO_UNSIGNED16|CO_OBJ____R_), 0, (uintptr_t)&hObjDataMotor1OccuredFaults},	    // Application - Motor faults of master
    {CO_KEY(CO_OD_REG_MOTOR_OCC_FAULTS, 1, CO_UNSIGNED16|CO_OBJ____R_), 0, (uintptr_t)&hObjDataMotor2OccuredFaults},		// Application - Motor faults of slave 1

    {CO_KEY(CO_OD_REG_MOTOR_CUR_FAULTS, 0, CO_UNSIGNED16|CO_OBJ____R_), 0, (uintptr_t)&hObjDataMotor1CurrentFaults},		// Application - Motor faults of master
    {CO_KEY(CO_OD_REG_MOTOR_CUR_FAULTS, 1, CO_UNSIGNED16|CO_OBJ____R_), 0, (uintptr_t)&hObjDataMotor2CurrentFaults},		// Application - Motor faults of slave 1

    {CO_KEY(CO_OD_REG_MOTOR_TORQUE_REF, 0, CO_SIGNED16|CO_OBJ____RW), 0, (uintptr_t)&hObjDataMotor1TorqRef},		        // Application - Reference torque to master motor
    {CO_KEY(CO_OD_REG_MOTOR_TORQUE_REF, 1, CO_SIGNED16|CO_OBJ____RW), 0, (uintptr_t)&hObjDataMotor2TorqRef},		        // Application - Reference torque to slave motor 1

    {CO_KEY(CO_OD_REG_MOTOR_START, 0, CO_UNSIGNED8|CO_OBJ____RW), 0, (uintptr_t)&bObjDataMotor1Start},		            // Application - Start bit to activate master
    {CO_KEY(CO_OD_REG_MOTOR_START, 1, CO_UNSIGNED8|CO_OBJ____RW), 0, (uintptr_t)&bObjDataMotor2Start},		            // Application - Start bit to activate slave motor 1

    {CO_KEY(CO_OD_REG_FAULT_ACK, 0, CO_UNSIGNED8|CO_OBJ____RW), 0, (uintptr_t)&bObjDataMotor1FaultAck},		            // Application - Bit to acknowledge motor fault master
    {CO_KEY(CO_OD_REG_FAULT_ACK, 1, CO_UNSIGNED8|CO_OBJ____RW), 0, (uintptr_t)&bObjDataMotor2FaultAck},		            // Application - Bit to acknowledge motor fault slave 1
    
    /***********GNR2 User Data configuration OBJECTS MODULE********************************/
    
    //Application - Informe what user data was upadted
    {CO_KEY(CO_OD_REG_USER_DATA_CONFIG_BIKE_MODEL, 0, CO_UNSIGNED16|CO_OBJ____RW), 0, (uintptr_t)&bObjDataUserDataConfig},
    
    //Application - Informe if user data was upadted or is being upadted.
    {CO_KEY(CO_OD_REG_KEY_USER_DATA_CONFIG, 0, CO_UNSIGNED16|CO_OBJ____RW), 0, (uintptr_t)&bObjDataKeyUserDataConfig},
    
    /************************GNR2-Throttle/Pedal Assist OBJECTS MODULE*************************/ 
    // Here they are being linked in the OD.
    // IN the dictionary they are reponsible to hold the configuration
    // of the Throttle/Pedal Assist parameters.
    // Theses configuration can be read and write using SDO services.
    
    // Application - 
    // Torque: based on a multiplier of the torque input. 
    // Cadence: based on pedaling speed.
    // Hybrid (recommended): Combination of torque + pedaling speed as algorithm inputs for motor assistance.
    {CO_KEY(CO_OD_REG_PAS_ALGORITHM, 0, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataPasAlgorithm},
    
    //Application - Percentage of the available max motor power that the PAS algorithm can use. 
    {CO_KEY(CO_OD_REG_PAS_MAX_POWER, 0, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataPasMaxPower},
    
    //Application - Torque threshold for starting motor assistance from 0 speed. Only relevant for torque/hybrid PAS. 
    {CO_KEY(CO_OD_REG_TORQUE_MINIMUM_THRESHOLD, 0, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataTorqueMinimumThreshold},
    
    //Application - How much the motor multiplies the torque sensor input from user. Only relevant for torque/hybrid PAS. 
    {CO_KEY(CO_OD_REG_TORQUE_SENSOR_MULTIPLIER, 0, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataTorqueSensorMultiplier},
    
    //Application - not defined. to do. 
    {CO_KEY(CO_OD_REG_TORQUE_MAX_SPEED, 0, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataTorqueMaxSpeed},
    
     //Application - The speed up to which this PAS level will give motor assistance. 
    {CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 0, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataCadenceHybridLeveSpeed[0]},
    {CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 1, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataCadenceHybridLeveSpeed[1]},
    {CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 2, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataCadenceHybridLeveSpeed[2]},
    {CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 3, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataCadenceHybridLeveSpeed[3]},
    {CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 4, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataCadenceHybridLeveSpeed[4]},
    {CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 5, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataCadenceHybridLeveSpeed[5]},
    {CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 6, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataCadenceHybridLeveSpeed[6]},
    {CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 7, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataCadenceHybridLeveSpeed[7]},
    {CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 8, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataCadenceHybridLeveSpeed[8]},
    {CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 9, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataCadenceHybridLeveSpeed[9]},
    
    //Application - The speed up to which this PAS level will give motor assistance. 
    {CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 0, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataTorqueLevelPower[0]},
    {CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 1, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataTorqueLevelPower[1]},
    {CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 2, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataTorqueLevelPower[2]},
    {CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 3, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataTorqueLevelPower[3]},
    {CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 4, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataTorqueLevelPower[4]},
    {CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 5, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataTorqueLevelPower[5]},
    {CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 6, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataTorqueLevelPower[6]},
    {CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 7, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataTorqueLevelPower[7]},
    {CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 8, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataTorqueLevelPower[8]},
    {CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 9, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataTorqueLevelPower[9]},
    
    //Application - The max speed that the throttle will bring the vehicle to. 
    {CO_KEY(CO_OD_REG_MAX_SPEED, 0, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataMaxSpeed},
    
    //Application - Speed that the walk mode of the vehicle goes up to. 
    {CO_KEY(CO_OD_REG_WALK_MODE_SPEED, 0, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataWalkModeSpeed},
    
    CO_OBJ_DIR_ENDMARK  /* mark end of used objects */
};
    #else
    // Mandatory entries
struct CO_OBJ_T GNR2_OD[GNR2_OBJ_N] = {
	  {CO_KEY(0x1000, 0, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (uintptr_t)0},						                            // Device Type
    {CO_KEY(0x1001, 0, CO_UNSIGNED8 |CO_OBJ____R_), 0, (uintptr_t)&hObjDataErrorRegister},				                    // Error Register
    {CO_KEY(0x1005, 0, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (uintptr_t)CO_COBID_SYNC_STD(GENERATE_SYNC, STD_ID_SYNC)},		// COB-ID SYNC Message
    {CO_KEY(0x1006, 0, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (uintptr_t)SYNC_PERIOD_US},                                      // SYNC period
    {CO_KEY(0x1014, 0, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (uintptr_t)CO_COBID_EMCY_STD(USE_EMCY, STD_ID_EMCY)},		    // COB-ID EMCY Message
    {CO_KEY(0x1016, 0, CO_UNSIGNED8|CO_OBJ_D__R_), 0, (uintptr_t)1},				                        // Consumer Heartbeat max index    
    {CO_KEY(0x1016, 1,               CO_OBJ____RW), CO_THB_CONS, (uintptr_t)&AppHbConsumer_1},				// Consumer Heartbeat parameters  
    {CO_KEY(0x1017, 0, CO_UNSIGNED16|CO_OBJ____RW), 0, (uintptr_t)&hObjDataProdHbTime},				        // Producer Heartbeat Time 
    {CO_KEY(0x1018, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (uintptr_t)4},						                // Identity - Highest Sub Index
    {CO_KEY(0x1018, 1, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (uintptr_t)0},						                // Identity - Vendor ID
    {CO_KEY(0x1018, 2, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (uintptr_t)0},						                // Identity - Product Code
    {CO_KEY(0x1018, 3, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (uintptr_t)0},						                // Identity - Revision Number
    {CO_KEY(0x1018, 4, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (uintptr_t)0},						                // Identity - Serial Number

    // SDO Server
    {CO_KEY(0x1200, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (uintptr_t)2},						                            // SDO Srv Parameter - Highest Sub Index
    #if GNR_MASTER
    {CO_KEY(0x1200, 1, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_COBID_SDO_REQUEST()},		    // SDO Srv Parameter - COB-ID Client to Server
    {CO_KEY(0x1200, 2, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_COBID_SDO_RESPONSE()},		// SDO Srv Parameter - COB-ID Server to Client
    #else
    {CO_KEY(0x1200, 1, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_COBID_SDO_STD(USE_SSDO, 0, 0x600+GNR2_SLAVE_NODE_ID)},		                                    // SDO Srv Parameter - COB-ID Client to Server
    {CO_KEY(0x1200, 2, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_COBID_SDO_STD(USE_SSDO, 0, 0x580+GNR2_SLAVE_NODE_ID)},		                                // SDO Srv Parameter - COB-ID Server to Client
    #endif

    // SDO Client
    {CO_KEY(0x1280, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (uintptr_t)3},						                // SDO Client Parameter - Highest Sub Index
    {CO_KEY(0x1280, 1, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_COBID_SDO_REQUEST()},			                    // SDO Client Parameter - COB-ID Client to Server
    {CO_KEY(0x1280, 2, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_COBID_SDO_RESPONSE()},			                // SDO Client Parameter - COB-ID Server to Client
    {CO_KEY(0x1280, 3, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (uintptr_t)GNR2_SLAVE_NODE_ID},

    // RPDO 1
    {CO_KEY(0x1400, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (uintptr_t)5},						                // RPDO1 Parameter - Highest Sub Index
    {CO_KEY(0x1400, 1, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_COBID_RPDO_STD(USE_RPDO, STD_ID_RPDO1)},			// RPDO1 Parameter - COB-ID RPDO
    {CO_KEY(0x1400, 2, CO_UNSIGNED8|CO_OBJ_D__R_), 0, RPDO_TRANSMISSION_TYPE},			                    // RPDO1 Parameter - Transmission type
    {CO_KEY(0x1400, 5, CO_UNSIGNED16|CO_OBJ_D__R_), 0, RPDO_PERIOD_MS},			                            // RPDO1 Parameter - Event period
    #if GNR_MASTER
    {CO_KEY(0x1600, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (uintptr_t)4},						                // RPDO1 Mapping - Number of mapped object in PDO
    {CO_KEY(0x1600, 1, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_LINK(CO_OD_REG_MOTOR_STATE, 1, 16)},			    // RPDO1 Mapping - Object 1
    {CO_KEY(0x1600, 2, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_LINK(CO_OD_REG_MOTOR_OCC_FAULTS, 1, 16)},			// RPDO1 Mapping - Object 2
    {CO_KEY(0x1600, 3, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_LINK(CO_OD_REG_MOTOR_CUR_FAULTS, 1, 16)},			// RPDO1 Mapping - Object 2
    {CO_KEY(0x1600, 4, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_LINK(CO_OD_REG_MOTOR_SPEED, 1, 16)},			    // RPDO1 Mapping - Object 2
    #else
    {CO_KEY(0x1600, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (uintptr_t)2},						                // RPDO1 Mapping - Number of mapped object in PDO
    {CO_KEY(0x1600, 1, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_LINK(CO_OD_REG_MOTOR_TORQUE_REF, 1, 16)},			// RPDO1 Mapping - Object 1
    {CO_KEY(0x1600, 2, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_LINK(CO_OD_REG_MOTOR_START, 1, 8)},			    // RPDO1 Mapping - Object 2
    #endif

//    // RPDO 2
//    {CO_KEY(0x1401, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (uintptr_t)5},						                // RPDO1 Parameter - Highest Sub Index
//    {CO_KEY(0x1401, 1, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_COBID_RPDO_STD(USE_RPDO, STD_ID_RPDO2)},			// RPDO1 Parameter - COB-ID RPDO
//    {CO_KEY(0x1401, 2, CO_UNSIGNED8|CO_OBJ_D__R_), 0, RPDO_TRANSMISSION_TYPE},			                    // RPDO1 Parameter - Transmission type
//    {CO_KEY(0x1401, 5, CO_UNSIGNED16|CO_OBJ_D__R_), 0, RPDO_PERIOD_MS},			                            // RPDO1 Parameter - Event period
//    #if GNR_MASTER
//    {CO_KEY(0x1601, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (uintptr_t)4},						                // RPDO1 Mapping - Number of mapped object in PDO
//    {CO_KEY(0x1601, 1, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_LINK(0x2000, 1, 16)},			                    // RPDO1 Mapping - Object 1
//    {CO_KEY(0x1601, 2, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_LINK(0x2001, 1, 16)},			                    // RPDO1 Mapping - Object 2
//    {CO_KEY(0x1601, 3, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_LINK(0x2002, 1, 16)},			                    // RPDO1 Mapping - Object 3
//    {CO_KEY(0x1601, 4, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_LINK(0x2003, 1, 16)},			                    // RPDO1 Mapping - Object 4
//    #else
//    {CO_KEY(0x1601, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (uintptr_t)0},						                // RPDO1 Mapping - Number of mapped object in PDO
//    #endif

    // TPDO 1
    {CO_KEY(0x1800, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (uintptr_t)6},						                // TPDO1 Parameter - Highest Sub Index
    {CO_KEY(0x1800, 1, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_COBID_TPDO_STD(USE_TPDO, STD_ID_TPDO1)},			// TPDO1 Parameter - COB-ID TPDO
    {CO_KEY(0x1800, 2, CO_UNSIGNED8|CO_OBJ_D__R_), 0, TPDO_TRANSMISSION_TYPE},			                    // TPDO1 Parameter - Transmission type
    {CO_KEY(0x1800, 5, CO_UNSIGNED16|CO_OBJ_D__R_), 0, TPDO_PERIOD_MS},			                            // TPDO1 Parameter - Transmission type
    {CO_KEY(0x1800, 6, CO_UNSIGNED8|CO_OBJ_D__R_), 0, 0},			                                        // TPDO1 Parameter - SYNC start value
    #if GNR_MASTER
    {CO_KEY(0x1A00, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (uintptr_t)2},						                // RPDO1 Mapping - Number of mapped object in PDO
    {CO_KEY(0x1A00, 1, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_LINK(CO_OD_REG_MOTOR_TORQUE_REF, 1, 16)},			// RPDO1 Mapping - Object 1
    {CO_KEY(0x1A00, 2, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_LINK(CO_OD_REG_MOTOR_START, 1, 8)},			    // RPDO1 Mapping - Object 2
    #else
    {CO_KEY(0x1A00, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (uintptr_t)4},						                // RPDO1 Mapping - Number of mapped object in PDO
    {CO_KEY(0x1A00, 1, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_LINK(CO_OD_REG_MOTOR_STATE, 1, 16)},			    // RPDO1 Mapping - Object 1
    {CO_KEY(0x1A00, 2, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_LINK(CO_OD_REG_MOTOR_OCC_FAULTS, 1, 16)},			// RPDO1 Mapping - Object 2
    {CO_KEY(0x1A00, 3, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_LINK(CO_OD_REG_MOTOR_CUR_FAULTS, 1, 16)},			// RPDO1 Mapping - Object 2
    {CO_KEY(0x1A00, 4, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_LINK(CO_OD_REG_MOTOR_SPEED, 1, 16)},			    // RPDO1 Mapping - Object 2
    #endif

//    // TPDO 2
//    {CO_KEY(0x1801, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (uintptr_t)5},						                // TPDO1 Parameter - Highest Sub Index
//    {CO_KEY(0x1801, 1, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_COBID_TPDO_STD(USE_TPDO, STD_ID_TPDO2)},			// TPDO1 Parameter - COB-ID TPDO
//    {CO_KEY(0x1801, 2, CO_UNSIGNED8|CO_OBJ_D__R_), 0, TPDO_TRANSMISSION_TYPE},			                    // TPDO1 Parameter - Transmission type
//    {CO_KEY(0x1801, 5, CO_UNSIGNED16|CO_OBJ_D__R_), 0, TPDO_PERIOD_MS},			                            // TPDO1 Parameter - Transmission type
//    #if GNR_MASTER
//    {CO_KEY(0x1A01, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (uintptr_t)0},						                // RPDO1 Mapping - Number of mapped object in PDO
//    #else
//    {CO_KEY(0x1A01, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (uintptr_t)4},						                // RPDO1 Mapping - Number of mapped object in PDO
//    {CO_KEY(0x1A01, 1, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_LINK(0x2000, 1, 16)},			                    // RPDO1 Mapping - Object 1
//    {CO_KEY(0x1A01, 2, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_LINK(0x2001, 1, 16)},			                    // RPDO1 Mapping - Object 2
//    {CO_KEY(0x1A01, 3, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_LINK(0x2002, 1, 16)},			                    // RPDO1 Mapping - Object 3
//    {CO_KEY(0x1A01, 4, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_LINK(0x2003, 1, 16)},			                    // RPDO1 Mapping - Object 4
//    #endif

    // GNR2-IOT OBJECTS MODULE
    {CO_KEY(CO_OD_REG_SPEED_MEASURE, 0, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataSpeedMeas},   // Application - Inst Speed
    {CO_KEY(CO_OD_REG_POWER_MEASURE, 0, CO_UNSIGNED16  |CO_OBJ____RW), 0, (uintptr_t)&hObjDataPowerMeas},   // Application - Inst Power
    {CO_KEY(CO_OD_REG_SOC,           0, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataSOC},         // Application - State of Charge
    {CO_KEY(CO_OD_REG_PAS_LEVEL,     0, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataPAS},	        // Application - PAS Level
    {CO_KEY(CO_OD_REG_MAX_PAS,       0, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataMaxPAS},	    // Application - Max PAS Level
    {CO_KEY(CO_OD_REG_MAX_POWER,     0, CO_SIGNED16    |CO_OBJ____RW), 0, (uintptr_t)&hObjDataMaxPower},   // Application - Max Power
    {CO_KEY(CO_OD_REG_ERR_STATE,     0, CO_SIGNED16    |CO_OBJ____RW), 0, (uintptr_t)&hObjDataErrorState},  // Application - Error State
    {CO_KEY(CO_OD_REG_SERIAL_NB,     0, CO_UNSIGNED32  |CO_OBJ____RW), 0, (uintptr_t)&wObjDataSerialNbH},   // Application - Serial Number High side
    {CO_KEY(CO_OD_REG_SERIAL_NB,     1, CO_UNSIGNED32  |CO_OBJ____RW), 0, (uintptr_t)&wObjDataSerialNbL},   // Application - Serial Number Low side
    {CO_KEY(CO_OD_REG_FW_VERSION,    0, CO_UNSIGNED16  |CO_OBJ____RW), 0, (uintptr_t)&hObjDataFwVersion},   // Application - Firmware Version

    //GNR2 Objects
    {CO_KEY(CO_OD_REG_MOTOR_SPEED, 0, CO_SIGNED16|CO_OBJ____R_), 0, (uintptr_t)&hObjDataMotor1SpeedMeas},			        // Application - Measured motor speed of master
    {CO_KEY(CO_OD_REG_MOTOR_SPEED, 1, CO_SIGNED16|CO_OBJ____R_), 0, (uintptr_t)&hObjDataMotor2SpeedMeas},			        // Application - Measured motor speed of slave 1

    {CO_KEY(CO_OD_REG_BUS_VOLTAGE, 0, CO_UNSIGNED16|CO_OBJ____R_), 0, (uintptr_t)&hObjDataMotor1BusVoltage},			    // Application - Measured bus voltage of master
    {CO_KEY(CO_OD_REG_BUS_VOLTAGE, 1, CO_UNSIGNED16|CO_OBJ____R_), 0, (uintptr_t)&hObjDataMotor2BusVoltage},			    // Application - Measured bus voltage of slave 1

    {CO_KEY(CO_OD_REG_MOTOR_TEMP, 0, CO_SIGNED16|CO_OBJ____R_), 0, (uintptr_t)&hObjDataMotor1Temp},			                // Application - Measured motor Iq of master
    {CO_KEY(CO_OD_REG_MOTOR_TEMP, 1, CO_SIGNED16|CO_OBJ____R_), 0, (uintptr_t)&hObjDataMotor2Temp},			                // Application - Measured motor Iq of slave 1

    {CO_KEY(CO_OD_REG_HEATSINK_TEMP, 0, CO_SIGNED16|CO_OBJ____R_), 0, (uintptr_t)&hObjDataHeatsink1Temp},			        // Application - Measured motor Id of master
    {CO_KEY(CO_OD_REG_HEATSINK_TEMP, 1, CO_SIGNED16|CO_OBJ____R_), 0, (uintptr_t)&hObjDataHeatsink2Temp},			        // Application - Measured motor Id of slave 1

    {CO_KEY(CO_OD_REG_MOTOR_STATE, 0, CO_UNSIGNED16|CO_OBJ____R_), 0, (uintptr_t)&hObjDataMotor1State},			            // Application - Motor state of master
    {CO_KEY(CO_OD_REG_MOTOR_STATE, 1, CO_UNSIGNED16|CO_OBJ____R_), 0, (uintptr_t)&hObjDataMotor2State},			            // Application - Motor state of slave 1

    {CO_KEY(CO_OD_REG_MOTOR_OCC_FAULTS, 0, CO_UNSIGNED16|CO_OBJ____R_), 0, (uintptr_t)&hObjDataMotor1OccuredFaults},	    // Application - Motor faults of master
    {CO_KEY(CO_OD_REG_MOTOR_OCC_FAULTS, 1, CO_UNSIGNED16|CO_OBJ____R_), 0, (uintptr_t)&hObjDataMotor2OccuredFaults},		// Application - Motor faults of slave 1

    {CO_KEY(CO_OD_REG_MOTOR_CUR_FAULTS, 0, CO_UNSIGNED16|CO_OBJ____R_), 0, (uintptr_t)&hObjDataMotor1CurrentFaults},		// Application - Motor faults of master
    {CO_KEY(CO_OD_REG_MOTOR_CUR_FAULTS, 1, CO_UNSIGNED16|CO_OBJ____R_), 0, (uintptr_t)&hObjDataMotor2CurrentFaults},		// Application - Motor faults of slave 1

    {CO_KEY(CO_OD_REG_MOTOR_TORQUE_REF, 0, CO_SIGNED16|CO_OBJ____RW), 0, (uintptr_t)&hObjDataMotor1TorqRef},		        // Application - Reference torque to master motor
    {CO_KEY(CO_OD_REG_MOTOR_TORQUE_REF, 1, CO_SIGNED16|CO_OBJ____RW), 0, (uintptr_t)&hObjDataMotor2TorqRef},		        // Application - Reference torque to slave motor 1

    {CO_KEY(CO_OD_REG_MOTOR_START, 0, CO_UNSIGNED8|CO_OBJ____RW), 0, (uintptr_t)&bObjDataMotor1Start},		            // Application - Start bit to activate master
    {CO_KEY(CO_OD_REG_MOTOR_START, 1, CO_UNSIGNED8|CO_OBJ____RW), 0, (uintptr_t)&bObjDataMotor2Start},		            // Application - Start bit to activate slave motor 1

    {CO_KEY(CO_OD_REG_FAULT_ACK, 0, CO_UNSIGNED8|CO_OBJ____RW), 0, (uintptr_t)&bObjDataMotor1FaultAck},		            // Application - Bit to acknowledge motor fault master
    {CO_KEY(CO_OD_REG_FAULT_ACK, 1, CO_UNSIGNED8|CO_OBJ____RW), 0, (uintptr_t)&bObjDataMotor2FaultAck},		            // Application - Bit to acknowledge motor fault slave 1

    /***********GNR2 User Data configuration OBJECTS MODULE********************************/
    
    //Application - Informe what user data was upadted
    {CO_KEY(CO_OD_REG_USER_DATA_CONFIG_BIKE_MODEL, 0, CO_UNSIGNED16|CO_OBJ____RW), 0, (uintptr_t)&bObjDataUserDataConfig},
    
    //Application - Informe if user data was upadted or is being upadted.
    {CO_KEY(CO_OD_REG_KEY_USER_DATA_CONFIG, 0, CO_UNSIGNED16|CO_OBJ____RW), 0, (uintptr_t)&bObjDataKeyUserDataConfig},
    
    /************************GNR2-Throttle/Pedal Assist OBJECTS MODULE*************************/ 
    // Here they are being linked in the OD.
    // IN the dictionary they are reponsible to hold the configuration
    // of the Throttle/Pedal Assist parameters.
    // Theses configuration can be read and write using SDO services.
    
    // Application - 
    // Torque: based on a multiplier of the torque input. 
    // Cadence: based on pedaling speed.
    // Hybrid (recommended): Combination of torque + pedaling speed as algorithm inputs for motor assistance.
    {CO_KEY(CO_OD_REG_PAS_ALGORITHM, 0, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataPasAlgorithm},
    
    //Application - Percentage of the available max motor power that the PAS algorithm can use. 
    {CO_KEY(CO_OD_REG_PAS_MAX_POWER, 0, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataPasMaxPower},
    
    //Application - Torque threshold for starting motor assistance from 0 speed. Only relevant for torque/hybrid PAS. 
    {CO_KEY(CO_OD_REG_TORQUE_MINIMUM_THRESHOLD, 0, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataTorqueMinimumThreshold},
    
    //Application - How much the motor multiplies the torque sensor input from user. Only relevant for torque/hybrid PAS. 
    {CO_KEY(CO_OD_REG_TORQUE_SENSOR_MULTIPLIER, 0, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataTorqueSensorMultiplier},
    
    //Application - not defined. to do. 
    {CO_KEY(CO_OD_REG_TORQUE_MAX_SPEED, 0, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataTorqueMaxSpeed},
    
     //Application - The speed up to which this PAS level will give motor assistance. 
    {CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 0, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataCadenceHybridLeveSpeed[0]},
    {CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 1, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataCadenceHybridLeveSpeed[1]},
    {CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 2, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataCadenceHybridLeveSpeed[2]},
    {CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 3, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataCadenceHybridLeveSpeed[3]},
    {CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 4, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataCadenceHybridLeveSpeed[4]},
    {CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 5, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataCadenceHybridLeveSpeed[5]},
    {CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 6, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataCadenceHybridLeveSpeed[6]},
    {CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 7, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataCadenceHybridLeveSpeed[7]},
    {CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 8, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataCadenceHybridLeveSpeed[8]},
    {CO_KEY(CO_OD_REG_CADENCE_HYBRID_LEVEL, 9, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataCadenceHybridLeveSpeed[9]},
    
    //Application - The speed up to which this PAS level will give motor assistance. 
    {CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 0, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataTorqueLevelPower[0]},
    {CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 1, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataTorqueLevelPower[1]},
    {CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 2, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataTorqueLevelPower[2]},
    {CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 3, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataTorqueLevelPower[3]},
    {CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 4, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataTorqueLevelPower[4]},
    {CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 5, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataTorqueLevelPower[5]},
    {CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 6, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataTorqueLevelPower[6]},
    {CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 7, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataTorqueLevelPower[7]},
    {CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 8, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataTorqueLevelPower[8]},
    {CO_KEY(CO_OD_REG_TORQUE_LEVEL_POWER, 9, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataTorqueLevelPower[9]},
    
    //Application - The max speed that the throttle will bring the vehicle to. 
    {CO_KEY(CO_OD_REG_MAX_SPEED, 0, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataMaxSpeed},
    
    //Application - Speed that the walk mode of the vehicle goes up to. 
    {CO_KEY(CO_OD_REG_WALK_MODE_SPEED, 0, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&bObjDataWalkModeSpeed},

    CO_OBJ_DIR_ENDMARK  /* mark end of used objects */
};
    #endif


/* Each software timer needs some memory for managing
 * the lists and states of the timed action events.
 */
static CO_TMR_MEM TmrMem[GNR2_TMR_N];

/* Each SDO server needs memory for the segmented or
 * block transfer requests.
 */
static uint8_t SdoSrvMem[CO_SSDO_N * CO_SDO_BUF_BYTE];

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
    &SdoSrvMem[0]             /* SDO Transfer Buffer Memory     */
};
