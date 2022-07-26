/**
*  co_gnr2_specs.c
*  Module for defining the CANOpen specs of GNR2
*/ 

// ==================== INCLUDES ======================== //

#include "co_gnr2_specs.h"
#include "gnr_parameters.h" // To use ENABLE_CAN_LOGGER
#if !ENABLE_CAN_LOGGER
                                    /* select application drivers: */
#include "co_can_ra6t2.h"               /* CAN driver                  */
#include "co_timer_ra6t2.h"              /* Timer driver                */
#include "co_nvm_ra6t2.h"               /* NVM driver                  */

// ==================== PRIVATE DEFINES ======================== //

/* Define some default values for our CANopen node: */
#define GNR2_NODE_ID       0x01                /* CANopen node ID             */
#define GNR2_BAUDRATE      500000u             /* CAN baudrate                */
#define GNR2_TMR_N         16u                 /* Number of software timers   */
#define GNR2_TICKS_PER_SEC 1000u               /* Timer clock frequency in Hz */
#define GNR2_OBJ_N         64u                 /* Object dictionary max size  */

// ==================== PRIVATE VARIABLES ======================== //

/* allocate global variables for runtime value of objects */
/* allocate global variables for runtime value of objects */
static uint8_t  	Obj1001_00_08 = 0;
static uint16_t  	HbTime = 0xFA;			    // 250ms

/* Allocate global variables for GNR objects */
static int16_t  objSpeedMeas = 0;
static uint8_t  objSOC = 0;
static uint8_t  objPAS = 0;
static uint8_t  objMaxPAS = 0;
static uint16_t  objFwVersion = 0;
static uint16_t objPowerMeas = 0;
static uint16_t objMaxPower = 0;
static uint16_t objErrorState = 0;
static uint64_t objserialNb = 0;

/* define the static object dictionary */
static struct CO_OBJ_T GNR2_OD[GNR2_OBJ_N] = {
    // Mandatory entries
	{CO_KEY(0x1000, 0, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (uintptr_t)0},							    // Device Type
    {CO_KEY(0x1001, 0, CO_UNSIGNED8 |CO_OBJ____R_), 0, (uintptr_t)&Obj1001_00_08},				    // Error Register
    {CO_KEY(0x1005, 0, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (uintptr_t)0x80},							// COB-ID SYNC Message
    {CO_KEY(0x1017, 0, CO_UNSIGNED16|CO_OBJ____RW), 0, (uintptr_t)&HbTime},							// Producer Heartbeat Time
    {CO_KEY(0x1018, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (uintptr_t)4},								// Identity - Highest Sub Index
    {CO_KEY(0x1018, 1, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (uintptr_t)0},								// Identity - Vendor ID
    {CO_KEY(0x1018, 2, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (uintptr_t)0},								// Identity - Product Code
    {CO_KEY(0x1018, 3, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (uintptr_t)0},								// Identity - Revision Number
    {CO_KEY(0x1018, 4, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (uintptr_t)0},								// Identity - Serial Number

    // SDO Server
    {CO_KEY(0x1200, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (uintptr_t)2},								// SDO Srv Parameter - Highest Sub Index
    {CO_KEY(0x1200, 1, CO_UNSIGNED32|CO_OBJ_DN_R_), 0, CO_COBID_SDO_REQUEST()},						// SDO Srv Parameter - COB-ID Client to Server
    {CO_KEY(0x1200, 2, CO_UNSIGNED32|CO_OBJ_DN_R_), 0, CO_COBID_SDO_RESPONSE()},					// SDO Srv Parameter - COB-ID Server to Client

    // SDO Client
    {CO_KEY(0x1280, 0, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (uintptr_t)3},								// SDO Client Parameter - Highest Sub Index
    {CO_KEY(0x1280, 1, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_COBID_SDO_REQUEST()},						// SDO Client Parameter - COB-ID Client to Server
    {CO_KEY(0x1280, 2, CO_UNSIGNED32|CO_OBJ_D__R_), 0, CO_COBID_SDO_RESPONSE()},					// SDO Client Parameter - COB-ID Server to Client
    {CO_KEY(0x1280, 3, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (uintptr_t)1},								// SDO Client Parameter - Client Node ID

    //GNR2 OBJECTS
    {CO_KEY(0x2000, 0, CO_SIGNED16    |CO_OBJ____R_), 0, (uintptr_t)&objSpeedMeas},			        // Application - Inst Speed
    {CO_KEY(0x2001, 0, CO_UNSIGNED16  |CO_OBJ____R_), 0, (uintptr_t)&objPowerMeas},			        // Application - Inst Power
    {CO_KEY(0x2002, 0, CO_UNSIGNED8   |CO_OBJ____R_), 0, (uintptr_t)&objSOC},					    // Application - State of Charge
    {CO_KEY(0x2003, 0, CO_UNSIGNED8   |CO_OBJ____RW), 0, (uintptr_t)&objPAS},						// Application - PAS Level
    {CO_KEY(0x2004, 0, CO_UNSIGNED8   |CO_OBJ____R_), 0, (uintptr_t)&objMaxPAS},					// Application - Max PAS Level
    {CO_KEY(0x2005, 0, CO_SIGNED16    |CO_OBJ____R_), 0, (uintptr_t)&objMaxPower},				    // Application - Max Power
    {CO_KEY(0x2006, 0, CO_SIGNED16    |CO_OBJ____R_), 0, (uintptr_t)&objErrorState},				// Application - Error State
    {CO_KEY(0x2007, 0, CO_UNSIGNED32  |CO_OBJ____R_), 0, (uintptr_t)&objserialNb},			        // Application - Serial Number
    {CO_KEY(0x2008, 0, CO_UNSIGNED16   |CO_OBJ____RW), 0, (uintptr_t)&objFwVersion},				    // Application - Firmware Version
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
#endif