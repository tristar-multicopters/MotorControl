/**
  ******************************************************************************
  * @file    lcd_comm_defines.h
  * @author  Jorge Polo, FTEX
  * @brief   This header gives the definitions used by the protocol comm. of the
	*					 LCD displays compatible with the RoadRunner
  *
	******************************************************************************
	*/

#ifndef	__LCD_COMM_DEFINES_H
#define	__LCD_COMM_DEFINES_H
/************************************** BAFANG **************************************/

// Commands
#define BAF_CMD_READ      0x11
#define BAF_CMD_WRITE     0x16

//Display Read Cmd
#define SKIP          0x00
#define R_VERSION     0x90
#define R_STATUS      0x08
#define R_WORKSTATUS  0x31 
#define R_CURRENT     0x0A
#define R_BATCAP      0x11
#define R_RSPEED      0x20
#define R_LIGHTSTATUS 0x1B
#define R_PHOTTHRESH  0x1C
//Display Write Cmd
#define W_SPEED_LIMIT 0x1F
#define W_ASSIST      0x0B 

//Assist levels
#define A_0      0x00 //No assist
#define A_1      0x01
#define A_2      0x0B //First(5)  
#define A_3      0x0C
#define A_4      0x0D //Seconde(5)
#define A_5      0x02  
#define A_6      0x15 //Third(5)
#define A_7      0x16
#define A_8      0x17 //Fourth(5) 
#define A_9      0x03 //Max assist
#define A_PUSH   0x06 //Push bike 
#define A_LSPEED 0x0A //Limit speed

typedef enum
{
  UFCP_REGULAR,
	UFCP_BAFANG

} ufcp_protocol;

/**********************************************************************************/
#endif
