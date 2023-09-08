/**
* @file   firmware_update.h
* @author Bruno Alves
* @brief  Application layer used to handle DFU process.
*
*
*/

#ifndef FIRMWARE_UPDATE_H_
#define FIRMWARE_UPDATE_H_

/*********************************************
               Includes                       
*********************************************/

#include "ASSERT_FTEX.h"
#include "vc_config.h"
#include "co_gnr2_specs.h"
#include "serial_flash_storage.h"
#include "lfs.h"
#include "comm_config.h"

/*********************************************
                Defines
*********************************************/

/************** OTA Commands ****************/
#define FORCE_STATUS_UPDATE        0x00
#define PREPARE_FILE_TRANSFER      0x01
#define FORCE_ROLLBACK             0x04
#define ERASE_FILE                 0x05
#define ABORT_FILE_TRANSFER        0x06
#define EXIT_DFU                   0x07
#define NO_COMMAND                 0xFF

/************** OTA Status ******************/
#define NO_ERROS                        0x00
#define READY_FILE_TRANSFER             0x01
#define FILE_TRANSFER_COMPLETED         0x02
#define APPLYING_DFU                    0x05
#define DFU_INITIALIZATION_FAILED       0xF0
#define DFU_ERROR_IMAGE_NOT_VALID       0xFA
#define DFU_ERROR_BAD_FW_VERSION        0xFB
#define TRANSFER_INTERNAL_ERROR         0xFD
#define TRANSFER_CORRUPTED_FRAME        0xFE
#define TRANSFER_MISSING_FRAME          0xFF

//timeout in steps of 33 ms. 1 = 33 ms, 2 = 66ms and etc.
#define OTA_TIMEOUT                     100 

//number of bytes of the firmware data frame
#define NUMBER_OF_DATA_FRAME_BYTES      134

//define data frame preambule.
#define PREAMBULE_BYTE                  0xAA

//define frame control type - First Frame
#define FIRST_FRAME                     0x00

//define frame control type - First Frame
#define INTERMEDIATE_FRAME              0x01

//define frame control type - First Frame
#define LAST_FRAME                      0x02

//define the maximum number of attempts.
#define MAXIMUM_ATTEMPTS                0x05

//define the position, in the data frame, of the Preamb byte.
#define PREAMB                          0x00

//define the position, in the data frame, of the msb byte of the frame count.
#define FRAMECOUNT_MSB                  0x01

//define the position, in the data frame, of the lsb byte of the frame count.
#define FRAMECOUNT_LSB                  0x02

//define the position, in the data frame, of the frame control byte.
#define FRAME_CONTROL                   0x03

////define the position, in the data frame, of the data size.
#define DATAFRAME_SIZE                  0x04

//define the position, in the data frame, of the beginning of the data payload.
#define DATAPAYLOAD_START               0x05

//define the postion, in the data frame, of the CRC byte.
#define DATAFRAME_CRC                   2

//define the data frame header size
#define DATAFRAME_HEADERSIZE            0x05

//define data frame crc size
#define DATAFRAME_CRCSIZE               0x01

//define the minimum battery charge, on %,
//where the device can start a DFU.
#define DFU_MINIMUMBATERRY_CHARGE       25

/************** CRC32 Macros ******************/
//
#define FLASH_READ_DWORD(x) (*(uint32_t*)(x))

/*********************************************
          Data Struct Definition
*********************************************/

//used by the firmware update state machine
typedef enum
{
    CHECK_START, //state where the device check if a command to start a DFU arrived.
    PREPARE_SYSTEM_TO_UPDATE,//state where force motor go to idle state.
    OPEN_DFU_FILE,//state to prepare file to receive the new firmware.
    READY_TO_FIRMWARE_UPDATE,// state to received, check and register the new firmware.
    FIRMWARE_UPDATE_FINISHED,//state to finish DFU, reset to call bootloader application.
    FIRMWARE_UPDATE_ERROR,//state to erase the memory because the firmware was corrupted.
    
}FirmwareUpdateStateMachine_t;

//struct to used by handle the firmware update process/
typedef struct
{
    uint8_t otaCommand;//used to received the dfu commands.
    uint8_t otaStatus;//used to inform IOT about the dfu state.
    bool updating;//used to inform the system if a update is running or not.
    bool firstDataFrameReceived;//flag to confirm that the data frame zero was received.
    short int otaTimeOut;//used to control the timeout to reception and retrys.
    uint16_t frameCount;//used to inform IOT about the last frame number received.
    uint8_t frameControl;//used to receive the frame type from teh data frame.
    uint8_t frameDataSize;//used to received the data frame payload size.
    uint8_t data[NUMBER_OF_DATA_FRAME_BYTES];//array to hold the data frame(134 bytes).
    bool serialFlashInit;//flag to indicate if the external memory was correctly initialized.
    uint8_t serialFlashReponse;//used to now if a flash operation failled or not.
    bool dataFrameReceived;//flag used to indicate if a new data frame was received.
    uint8_t crc;//used to CRC check from the data frame.
    uint32_t crc32;
    
}FirmwareUpdateControl_t;

//constant table used to calculate the crc
static unsigned char const crc8_table[] = {
	0x00,0x07,0x0e,0x09,0x1c,0x1b,0x12,0x15,
	0x38,0x3f,0x36,0x31,0x24,0x23,0x2a,0x2d,
	0x70,0x77,0x7e,0x79,0x6c,0x6b,0x62,0x65,
	0x48,0x4f,0x46,0x41,0x54,0x53,0x5a,0x5d,
	0xe0,0xe7,0xee,0xe9,0xfc,0xfb,0xf2,0xf5,
	0xd8,0xdf,0xd6,0xd1,0xc4,0xc3,0xca,0xcd,
	0x90,0x97,0x9e,0x99,0x8c,0x8b,0x82,0x85,
	0xa8,0xaf,0xa6,0xa1,0xb4,0xb3,0xba,0xbd,
	0xc7,0xc0,0xc9,0xce,0xdb,0xdc,0xd5,0xd2,
	0xff,0xf8,0xf1,0xf6,0xe3,0xe4,0xed,0xea,
	0xb7,0xb0,0xb9,0xbe,0xab,0xac,0xa5,0xa2,
	0x8f,0x88,0x81,0x86,0x93,0x94,0x9d,0x9a,
	0x27,0x20,0x29,0x2e,0x3b,0x3c,0x35,0x32,
	0x1f,0x18,0x11,0x16,0x03,0x04,0x0d,0x0a,
	0x57,0x50,0x59,0x5e,0x4b,0x4c,0x45,0x42,
	0x6f,0x68,0x61,0x66,0x73,0x74,0x7d,0x7a,
	0x89,0x8e,0x87,0x80,0x95,0x92,0x9b,0x9c,
	0xb1,0xb6,0xbf,0xb8,0xad,0xaa,0xa3,0xa4,
	0xf9,0xfe,0xf7,0xf0,0xe5,0xe2,0xeb,0xec,
	0xc1,0xc6,0xcf,0xc8,0xdd,0xda,0xd3,0xd4,
	0x69,0x6e,0x67,0x60,0x75,0x72,0x7b,0x7c,
	0x51,0x56,0x5f,0x58,0x4d,0x4a,0x43,0x44,
	0x19,0x1e,0x17,0x10,0x05,0x02,0x0b,0x0c,
	0x21,0x26,0x2f,0x28,0x3d,0x3a,0x33,0x34,
	0x4e,0x49,0x40,0x47,0x52,0x55,0x5c,0x5b,
	0x76,0x71,0x78,0x7f,0x6a,0x6d,0x64,0x63,
	0x3e,0x39,0x30,0x37,0x22,0x25,0x2c,0x2b,
	0x06,0x01,0x08,0x0f,0x1a,0x1d,0x14,0x13,
	0xae,0xa9,0xa0,0xa7,0xb2,0xb5,0xbc,0xbb,
	0x96,0x91,0x98,0x9f,0x8a,0x8d,0x84,0x83,
	0xde,0xd9,0xd0,0xd7,0xc2,0xc5,0xcc,0xcb,
	0xe6,0xe1,0xe8,0xef,0xfa,0xfd,0xf4,0xf3
};

//constant table used to calculate the crc32.
static const uint32_t crc32_table[] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

//========================= EXTERN TYPES ==========================//


// ==================== Public function prototypes ========================= //

/**
  @brief function used to manage the firmware update procedure.
  @param CO_NODE  *pNode pointer to the CAN node used on the CANOPEN communication.
  @param VCI_Handle_t * pVCI pointer to the struct responsible to handle vehicle control interface.
  @return void
*/
void FirmwareUpdate_Control (CO_NODE  *pNode, VCI_Handle_t * pVCI);

/**
  @brief return a flag, true or false, to indicating if the system
         is doing a firmware update.
  @return bool return true if the system is doing a firmware upadte.
*/
bool FirmwareUpdate_Running(void);

/**
  @brief used to initialize the external flash memory.
         The flash memory must to be initialized at the 
         firmware start, because initialization process
         requires some seconds sometimes.
  @return void
*/
void FirmwareUpdate_SerialFlashMemoryInit(void);

/**
  @brief used to initialize the external flash memory.
         The flash memory must to be initialized at the 
         firmware start, because initialization process
         requires some seconds sometimes.
  @return void
*/
void FirmwareUpdate_CheckDataFrame(VCI_Handle_t * pVCI);


 /*******************************************************************************************************************//**
 * @brief Calculate CRC-8 using polynomial X^8 + X^2 + X + 1 (0b10001100)
 * @param		vptr				Data over which the crc is computed
 * @param		len					size of the data over which the crc is computed
 * @return  uint8_t 		return the crc8 of the array given in parameter
 **********************************************************************************************************************/
uint8_t crc8(const void* vptr, int len);

/**********************************************************************************************************************
 * @brief Reset crcState variable to 0xFFFFFFFF
 **********************************************************************************************************************/
void FirmwareUpdate_Crc32Reset(void) ;

/**********************************************************************************************************************
 * @brief Calculate CRC-32 using polynomial x32 + x26 + x23 + x22 + x16 + x12 + x11 + x10 + x8 + x7 + x5 + x4 + x2 + x1 + 1
 * @param const uint8_t data Data over which the crc is computed
 **********************************************************************************************************************/
void FirmwareUpdate_Crc32Update(const uint8_t data);

/**********************************************************************************************************************
 * @brief Return crc 32 value.
 *        This operation is done using a separate function because we are calculating the CRC on the fly
 * @return  uint32_t return the crc32 calculated value.
 **********************************************************************************************************************/
uint32_t FirmwareUpdate_Crc32Result(void);

#endif