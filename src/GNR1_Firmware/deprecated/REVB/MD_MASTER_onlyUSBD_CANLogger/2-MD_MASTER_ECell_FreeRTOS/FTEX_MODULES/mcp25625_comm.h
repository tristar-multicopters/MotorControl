/**
  ******************************************************************************
  * @file    mcp25625_comm.h
	* @author  Kian Gheitasi, FTEX
  * @author  Sami Bouzid, FTEX
  * @brief   This module manages communications with mcp25625 (SPI CAN-controller)
  *
	******************************************************************************
	*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MCP25625_COMM_H
#define __MCP25625_COMM_H

#include "spi_interface.h"
#include "nrf_drv_spi.h"
#include "board_hardware.h"

/* MCP25625 Defines ------------------------------------------------------- */

// Typedef for MCP25625 register addresses

typedef enum 
{
	MCP_RXF0SIDH =   0x00,
	MCP_RXF0SIDL =   0x01,
	MCP_RXF0EID8 =   0x02,
	MCP_RXF0EID0 =   0x03,
	MCP_RXF1SIDH =   0x04,
	MCP_RXF1SIDL =   0x05,
	MCP_RXF1EID8 =   0x06,
	MCP_RXF1EID0 =   0x07,
	MCP_RXF2SIDH =   0x08,
	MCP_RXF2SIDL =   0x09,
	MCP_RXF2EID8 =   0x0A,
	MCP_RXF2EID0 =   0x0B,
	MCP_BFPCTRL  =   0x0C,
	MCP_TXRTSCTRL=   0x0D,
	MCP_CANSTAT  =   0x0E,
	MCP_CANCTRL  =   0x0F,
	MCP_RXF3SIDH =   0x10,
	MCP_RXF3SIDL =   0x11,
	MCP_RXF3EID8 =   0x12,
	MCP_RXF3EID0 =   0x13,
	MCP_RXF4SIDH =   0x14,
	MCP_RXF4SIDL =   0x15,
	MCP_RXF4EID8 =   0x16,
	MCP_RXF4EID0 =   0x17,
	MCP_RXF5SIDH =   0x18,
	MCP_RXF5SIDL =   0x19,
	MCP_RXF5EID8 =   0x1A,
	MCP_RXF5EID0 =   0x1B,
	MCP_TEC      =   0x1C,
	MCP_REC      =   0x1D,
	MCP_RXM0SIDH =   0x20,
	MCP_RXM0SIDL =   0x21,
	MCP_RXM0EID8 =   0x22,
	MCP_RXM0EID0 =   0x23,
	MCP_RXM1SIDH =   0x24,
	MCP_RXM1SIDL =   0x25,
	MCP_RXM1EID8 =   0x26,
	MCP_RXM1EID0 =   0x27,
  MCP_CNF3     =   0x28,
	MCP_CNF2     =   0x29,
	MCP_CNF1     =   0x2A,
	MCP_CANINTE  =   0x2B,
	MCP_CANINTF  =   0x2C,
	MCP_EFLG     =   0x2D,
	
	MCP_TXB0CTRL =   0x30,
	MCP_TXB0SIDH =   0x31,
	MCP_TXB0SIDL =   0x32,
	MCP_TXB0EID8 =   0x33,
	MCP_TXB0EID0 =   0x34,
	MCP_TXB0DLC  =   0x35,
	MCP_TXB0D0	 =	 0x36,
	MCP_TXB0D1   =   0x37,
	MCP_TXB0D2	 = 	 0x38,
	MCP_TXB0D3   =   0x39,
	MCP_TXB0D4   =   0x3A,
	MCP_TXB0D5   =   0x3B,
	MCP_TXB0D6   =   0x3C,
	MCP_TXB0D7   =   0x3D,
	
	MCP_TXB1CTRL =   0x40,
	MCP_TXB1SIDH =   0x41,
	MCP_TXB1SIDL =	 0x42,
	MCP_TXB1EID8 =   0x43,
	MCP_TXB1EID0 =   0x44,
	MCP_TXB1DLC  =   0x45,
	MCP_TXB1D0	 =	 0x46,
	MCP_TXB1D1   =   0x47,
	MCP_TXB1D2	 = 	 0x48,
	MCP_TXB1D3   =   0x49,
	MCP_TXB1D4   =   0x4A,
	MCP_TXB1D5   =   0x4B,
	MCP_TXB1D6   =   0x4C,
	MCP_TXB1D7   =   0x4D,
	
	MCP_TXB2CTRL =   0x50,
	MCP_TXB2SIDH =   0x51,
	MCP_TXB2SIDL =	 0x52,
	MCP_TXB2EID8 =   0x53,
	MCP_TXB2EID0 =   0x54,
	MCP_TXB2DLC  =   0x55,
	MCP_TXB2D0	 =	 0x56,
	MCP_TXB2D1   =   0x57,
	MCP_TXB2D2	 = 	 0x58,
	MCP_TXB2D3   =   0x59,
	MCP_TXB2D4   =   0x5A,
	MCP_TXB2D5   =   0x5B,
	MCP_TXB2D6   =   0x5C,
	MCP_TXB2D7   =   0x5D,
	
	MCP_RXB0CTRL =   0x60,
	MCP_RXB0SIDH =   0x61,
	MCP_RXB0SIDL =	 0x62,
	MCP_RXB0EID8 =   0x63,
	MCP_RXB0EID0 =   0x64,
	MCP_RXB0DLC  =   0x65,
	MCP_RXB0D0	 =	 0x66,
	MCP_RXB0D1   =   0x67,
	MCP_RXB0D2	 = 	 0x68,
	MCP_RXB0D3   =   0x69,
	MCP_RXB0D4   =   0x6A,
	MCP_RXB0D5   =   0x6B,
	MCP_RXB0D6   =   0x6C,
	MCP_RXB0D7   =   0x6D,
	
	MCP_RXB1CTRL =   0x70,
	MCP_RXB1SIDH =   0x71,
	MCP_RXB1SIDL =	 0x72,
	MCP_RXB1EID8 =   0x73,
	MCP_RXB1EID0 =   0x74,
	MCP_RXB1DLC  =   0x75,
	MCP_RXB1D0	 =	 0x76,
	MCP_RXB1D1   =   0x77,
	MCP_RXB1D2	 = 	 0x78,
	MCP_RXB1D3   =   0x79,
	MCP_RXB1D4   =   0x7A,
	MCP_RXB1D5   =   0x7B,
	MCP_RXB1D6   =   0x7C,
	MCP_RXB1D7   =   0x7D
}MCP_reg_addr_t;

typedef enum
{
	MCP_IDLE,
	MCP_TX0_RECEIVED,
	MCP_RX0_RECEIVED,
	MCP_RX1_RECEIVED,
	MCP_ERROR_MERRF,
	MCP_ERROR_ERRIF
} MCP_state_t;

typedef enum
{
	CAN_STATUS_VC_ID      = 0x1,
  CAN_THROTTLE_ID			  = 0x2,
	CAN_VBUS_ID					  = 0x10,
	
	CAN_STATUS_M1_ID 		  = 0xA0,
  CAN_CURRENT_M1_ID			= 0x20,
	CAN_SPEED_M1_ID				= 0x30,
	CAN_TEMPERATURE_M1_ID	= 0x40,
	
	CAN_STATUS_M2_ID	    = 0xA1,
	CAN_CURRENT_M2_ID	    = 0x21,
	CAN_SPEED_M2_ID       = 0x31,
	CAN_TEMPERATURE_M2_ID = 0x41,
	
	CAN_LOCK   					  = 0x123
}	MCP_CAN_id_t;

#define MCP_CANMSG_SIZE   64
// Interrupts definitions
#define MCP_RX_INT      0x03                                    // Enable receive interrupts
#define MCP_TX0_INT     0x04																		// Enable TXB0 interrupt
#define MCP_TX_INT      0x1C                                    // Enable all transmit interrup ts
#define MCP_NO_INT      0x00                                    // Disable all interrupts
////////////////////////////

// Define SPI Instruction Set
#define MCP_WRITE       0x02
#define MCP_READ        0x03
#define MCP_BITMOD      0x05
#define MCP_LOAD_TX0    0x40
#define MCP_LOAD_TX1    0x42
#define MCP_LOAD_TX2    0x44
#define MCP_READ_RX0    0x90
#define MCP_READ_RX1    0x94
#define MCP_READ_STATUS 0xA0
#define MCP_RX_STATUS   0xB0
#define MCP_RESET       0xC0

// CANCTRL Register Values
#define MODE_NORMAL     0x00
#define MODE_SLEEP      0x20
#define MODE_LOOPBACK   0x40
#define MODE_LISTENONLY 0x60
#define MODE_CONFIG     0x80
#define MODE_POWERUP    0xE0
#define MODE_MASK       0xE0
#define ABORT_TX        0x10
#define MODE_ONESHOT    0x08
#define CLKOUT_ENABLE   0x04
#define CLKOUT_DISABLE  0x00
#define CLKOUT_PS1      0x00
#define CLKOUT_PS2      0x01
#define CLKOUT_PS4      0x02
#define CLKOUT_PS8      0x03

// CNF1 Register Values
#define SJW1            0x00
#define SJW2            0x40
#define SJW3            0x80
#define SJW4            0xC0

//  CNF2 Register Values
#define BTLMODE         0x80
#define SAMPLE_1X       0x00
#define SAMPLE_3X       0x40

// CNF3 Register Values
#define SOF_ENABLE      0x80
#define SOF_DISABLE     0x00
#define WAKFIL_ENABLE   0x40
#define WAKFIL_DISABLE  0x00

// CANINTF Register Bits
#define MCP_RX0IF       0x01
#define MCP_RX1IF       0x02
#define MCP_TX0IF       0x04
#define MCP_TX1IF       0x08
#define MCP_TX2IF       0x10
#define MCP_ERRIF       0x20
#define MCP_WAKIF       0x40
#define MCP_MERRF       0x80

// BFPCTRL Register Bits
#define B1BFS           0x20
#define B0BFS           0x10
#define B1BFE           0x08
#define B0BFE           0x04
#define B1BFM           0x02
#define B0BFM           0x01

// TXRTCTRL Register Bits
#define B2RTS           0x20
#define B1RTS           0x10
#define B0RTS           0x08
#define B2RTSM          0x04
#define B1RTSM          0x02
#define B0RTSM          0x01

// Function Returns
#define CAN_OK              (0)
#define CAN_FAILINIT        (1)	

// Begin mt

#define MCP_EXIDE_MASK 			0x08                                        // Enables Extended identifier
#define MCP_DLC_MASK        0x0F                                        // 4 LSBits
#define MCP_RTR_MASK        0x40                                        // (1<<6) Bit 6

#define MCP_RXB_RX_ANY      0x60
#define MCP_RXB_RX_STD      0x20
#define MCP_RXB_RX_MASK     0x60
#define MCP_RXB_BUKT_MASK   (1<<2)

// Values for Baudrate
#define MCP_20MHz_1000kBPS_CFG1 (0x00)
#define MCP_20MHz_1000kBPS_CFG2 (0x89)
#define MCP_20MHz_1000kBPS_CFG3 (0x04)

#define MCP_20MHz_500kBPS_CFG1 (0x00)
#define MCP_20MHz_500kBPS_CFG2 (0xAC)
#define MCP_20MHz_500kBPS_CFG3 (0x07)

#define MCP_20MHz_250kBPS_CFG1 (0x01)
#define MCP_20MHz_250kBPS_CFG2 (0xAC)
#define MCP_20MHz_250kBPS_CFG3 (0x07)

// For MCP2515
#define MCP_24MHz_250kBPS_CFG1 (0x41)
#define MCP_24MHz_250kBPS_CFG2 (0xBE)
#define MCP_24MHz_250kBPS_CFG3 (0x07)

#define MCP_32MHz_250kBPS_CFG1 (0x43)
#define MCP_32MHz_250kBPS_CFG2 (0x9A)
#define MCP_32MHz_250kBPS_CFG3 (0x07)



/* Typedefs ------------------------------------------------------- */

typedef struct
{
	MCP_CAN_id_t id;								// The ID of the CAN Message
	bool ext;             				  // 0x01 for Extended identifier and 0x00 for a standard identifier
	bool rtr;										    // 0x01 if it is REMOTE TX REQUEST, 0x00 otherwise
	uint8_t length;									// Number of data bytes in the CAN message
	uint8_t data[8];								// CAN message

} CAN_Message_t;

typedef struct
{
	uint8_t size;
	uint8_t tx_buffer[15];
} CAN_toSend_t;

typedef enum
{
	CAN_TX_BUFFER_0 = 0,
	CAN_TX_BUFFER_1,
	CAN_TX_BUFFER_2,
	
} CAN_TX_Buffer_t;

typedef enum
{
	CAN_RX_BUFFER_0 = 0,
	CAN_RX_BUFFER_1,
	
} CAN_RX_Buffer_t;

typedef enum
{
	CAN_250KBS = 0,
	CAN_500KBS,
	CAN_1000KBS,
	
} CAN_BaudRate_t;

typedef struct
{
	uint8_t reg_table[127];
	SPI_Handle_t *pSPI;
	SPIdevice_t device;
	CAN_BaudRate_t baudrate;
	uint8_t int_pin;
	uint8_t res120_pin;
	const nrf_queue_t* MCP_queue;
	bool IT_received;
	bool ongoing_transfer;
	//For debugging purposes
	MCP_state_t state;
} MCP25625_Handle_t;

/* Functions ------------------------------------------------------- */
/**@brief Function for initialise the MCP instance
	 @p p_Handle: Handle of the MCP
*/
uint8_t MCP25625_Init(MCP25625_Handle_t* p_Handle);
/**@brief Function for write a CAN message on the MCP device
	 @p p_Handle: Handle of the MCP
   @out returns message to send with the size of elements to send
*/
CAN_toSend_t MCP25625_WriteCANmsg(MCP25625_Handle_t* p_Handle, CAN_TX_Buffer_t buffer, CAN_Message_t* pMsg);
void MCP25625_LoadTxBuffer(MCP25625_Handle_t* p_Handle, uint8_t length, uint8_t *buffer_tx);
void MCP25625_ReadRxBuffer(MCP25625_Handle_t* p_Handle, uint8_t buffer_ID);
void MCP25625_SendCANmsg(MCP25625_Handle_t* p_Handle, CAN_TX_Buffer_t buffer);
void MCP25625_manage_IT( void );
void MCP25625_check_errors( void );
/**@brief Function for pull a CAN message from the queue
	 @p message: Pointer to the message destination
*/
ret_code_t MCP25625_popCANmsg( CAN_Message_t * message );

/**@brief Task for managing the CAN messages reception and transmission
	 @p pvParameter: Unused parameter
*/
void TSK_CANmsg(void * pvParameter);
////////////////////////////////////////////////////////
//uint8_t MCP25625_Requesttoreceive(MCP25625_Handle_t* p_Handle, uint8_t buffer);
//uint8_t MCP25625_SetFilter(MCP25625_Handle_t* p_Handle, uint8_t identifier, uint8_t filter_index, uint16_t filter_value);

#endif /*__MCP25625_COMM_H*/
