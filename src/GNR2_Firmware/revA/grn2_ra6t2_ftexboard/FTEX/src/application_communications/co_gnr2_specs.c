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

#define GNR2_MASTER_NODE_ID        0x01 
#define GNR2_SLAVE_NODE_ID         0x02 

#if GNR_MASTER
#define GNR2_NODE_ID       GNR2_MASTER_NODE_ID          /* CANopen node ID             */
#else
#define GNR2_NODE_ID       GNR2_SLAVE_NODE_ID           /* CANopen node ID             */
#endif

#define GNR2_BAUDRATE      500000u             /* CAN baudrate                */
#define GNR2_TMR_N         16u                 /* Number of software timers   */
#define GNR2_TICKS_PER_SEC 5000u               /* Timer clock frequency in Hz */
#define GNR2_OBJ_N         128u                /* Object dictionary max size  */

#if GNR_MASTER

#define GENERATE_SYNC           1           /* Generate or not the SYNC frame. 1 for yes, 0 for no. */
#define STD_ID_SYNC             0x80        /* SYNC frame standard ID */
#define SYNC_PERIOD_US          20000       /* SYNC frame period in us */

#define USE_EMCY                0           /* Use emergency frame */
#define STD_ID_EMCY             0           /* Emergency frame standard ID */

#define HEARTBEAT_PERIOD_MS     250         /* Period of the heartbeat frame */

#define USE_RPDO                1           /* Use or not RPDO. 1 for yes, 0 for no. */
#define STD_ID_RPDO1            0x20        /* Standard ID of RPDO 1 */
#define STD_ID_RPDO2            0x21        /* Standard ID of TPDO 2 */
#define RPDO_TRANSMISSION_TYPE  254         /* RPDO transmission type. Event-driven (manufacturer-specific) is chosen */
#define RPDO_PERIOD_MS          20          /* Period in ms for RPDO processing */

#define USE_TPDO                1           /* Use or not TPDO. 1 for yes, 0 for no. */
#define STD_ID_TPDO1            0x17        /* Standard ID of TPDO 1 */
#define STD_ID_TPDO2            0x18        /* Standard ID of TPDO 2 */
#define TPDO_TRANSMISSION_TYPE  254         /* TPDO transmission type. Event-driven (manufacturer-specific) is chosen */
#define TPDO_PERIOD_MS          20          /* Period in ms for TPDO transmission */

#define USE_SSDO                1           /* Use or not SSDO. 1 for yes, 0 for no. */
#define USE_CSDO                1           /* Use or not CSDO. 1 for yes, 0 for no. */

#else

#define GENERATE_SYNC           0           /* Generate or not the SYNC frame. 1 for yes, 0 for no. */
#define STD_ID_SYNC             0x80        /* SYNC frame standard ID */
#define SYNC_PERIOD_US          0           /* SYNC frame period in us */

#define USE_EMCY                0           /* Use emergency frame */
#define STD_ID_EMCY             0           /* Emergency frame standard ID */

#define HEARTBEAT_PERIOD_MS     250         /* Period of the heartbeat frame */

#define USE_RPDO                1           /* Use or not RPDO. 1 for yes, 0 for no. */
#define STD_ID_RPDO1            0x17        /* Standard ID of RPDO 1 */
#define STD_ID_RPDO2            0x18        /* Standard ID of TPDO 2 */
#define RPDO_TRANSMISSION_TYPE  0           /* RPDO transmission type. Synchronous after SYNC is chosen */
#define RPDO_PERIOD_MS          0           /* Period in ms for RPDO processing. Not applicable for synchronous transmission type */

#define USE_TPDO                1           /* Use or not TPDO. 1 for yes, 0 for no. */
#define STD_ID_TPDO1            0x20        /* Standard ID of TPDO 1 */
#define STD_ID_TPDO2            0x21        /* Standard ID of TPDO 2 */
#define TPDO_TRANSMISSION_TYPE  1           /* Period in ms for TPDO transmission. Not applicable for synchronous transmission type */
#define TPDO_PERIOD_MS          0           /* Period in ms for RPDO processing. Not applicable for synchronous transmission typeé */

#define USE_SSDO                1           /* Use or not SSDO. 1 for yes, 0 for no. */
#define USE_CSDO                1           /* Use or not CSDO. 1 for yes, 0 for no. */

#endif


// ==================== PRIVATE VARIABLES ======================== //

/* Allocate global variables for runtime value of objects */
uint16_t hObjDataHbTime           = HEARTBEAT_PERIOD_MS;
uint8_t  hObjDataErrorRegister    = 0;

/* Allocate global variables for GNR objects */
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

uint32_t wObjDataSerialNbL                  = 0;
uint32_t wObjDataSerialNbH                  = 0;
uint16_t hObjDataFwVersion                  = 0;

/* define the static object dictionary */
struct CO_OBJ_T GNR2_OD[GNR2_OBJ_N] = {
    // Mandatory entries
	{CO_KEY(0x1000, 0, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (uintptr_t)0},						                            // Device Type
    {CO_KEY(0x1001, 0, CO_UNSIGNED8 |CO_OBJ____R_), 0, (uintptr_t)&hObjDataErrorRegister},				                    // Error Register
    {CO_KEY(0x1005, 0, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (uintptr_t)CO_COBID_SYNC_STD(GENERATE_SYNC, STD_ID_SYNC)},		// COB-ID SYNC Message
    {CO_KEY(0x1006, 0, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (uintptr_t)SYNC_PERIOD_US},                                      // SYNC period
    {CO_KEY(0x1014, 0, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (uintptr_t)CO_COBID_EMCY_STD(USE_EMCY, STD_ID_EMCY)},		    // COB-ID EMCY Message
    {CO_KEY(0x1017, 0, CO_UNSIGNED16|CO_OBJ____RW), 0, (uintptr_t)&hObjDataHbTime},				                // Producer Heartbeat Time
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
    {CO_KEY(0x1800, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (uintptr_t)5},						                // TPDO1 Parameter - Highest Sub Index
    {CO_KEY(0x1800, 1, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_COBID_TPDO_STD(USE_TPDO, STD_ID_TPDO1)},			// TPDO1 Parameter - COB-ID TPDO
    {CO_KEY(0x1800, 2, CO_UNSIGNED8|CO_OBJ_D__R_), 0, TPDO_TRANSMISSION_TYPE},			                    // TPDO1 Parameter - Transmission type
    {CO_KEY(0x1800, 5, CO_UNSIGNED16|CO_OBJ_D__R_), 0, TPDO_PERIOD_MS},			                            // TPDO1 Parameter - Transmission type
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
    
    {CO_KEY(0x200E, 0, CO_UNSIGNED32|CO_OBJ____R_), 0, (uintptr_t)&wObjDataSerialNbH},			            // Application - Serial number high side
    {CO_KEY(0x200E, 1, CO_UNSIGNED32|CO_OBJ____R_), 0, (uintptr_t)&wObjDataSerialNbL},			            // Application - Serial number low side
    
    {CO_KEY(0x200F, 0, CO_UNSIGNED16|CO_OBJ____R_), 0, (uintptr_t)&hObjDataFwVersion},		                // Application - Firmware version
    CO_OBJ_DIR_ENDMARK  /* mark end of used objects */
};

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

