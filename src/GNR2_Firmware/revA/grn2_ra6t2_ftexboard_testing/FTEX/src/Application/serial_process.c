/**
  ******************************************************************************
  * @file    serial_process.c
  * @brief   This file contain the serial process function for interaction 
  *          witht the ICT.
  ******************************************************************************
*/

// =============================== Includes ================================ //
#include "serial_process.h"

// ========================== Public Variables ============================ //

extern uint8_t bRecivedFrame[DATA_LENGTH];
extern volatile uint8_t g_data_received_flag;
data rxData;

// ==================== Private function prototypes ======================== //

/**
  * @brief  Clean received data frame 
  * @param  None
  * @return None
  */
void SerialProcess_CleanFrame()
{
    // Clear the received data frame
    for (uint8_t i = 0; i<DATA_LENGTH; i++)
    {
        bRecivedFrame[i]=0;
    }
}

/**
  * @brief  Get Message Length  
  * @param  None
  * @return return message Length
  */
uint8_t SerialProcess_MessageLenght ()
{
    uint8_t input_length = RESET_VALUE;

    /* Calculate bFrame_Buffer length */
    input_length = ((uint8_t)(strlen((char *) &bRecivedFrame)));
	return input_length;
}

/**
  * @brief  Sort received data frame  
  * @param  None
  * @return None
  */
void SerialProcess_SortData()
{
	rxData.startByte =  bRecivedFrame[0];
	rxData.device 	 = 	bRecivedFrame[1];
	rxData.type 	 =	bRecivedFrame[2];
	rxData.code		 =	bRecivedFrame[3];
	rxData.msgLength = 	(bRecivedFrame[4]-0x32);

	/* If the message is empty */
	if(rxData.msgLength == 0)
	{
        /*Clear the old message value of rxData*/
        for(int i = 0; i<20; i++ )
        {	
            rxData.message[i] = 0 ;
        }
    }
	/* If the message is not empty */
	else
	{
        /* Get the message bytes if we get a message !=0 */
        for(int i = 0; i< rxData.msgLength; i++ )
		{	
            rxData.message[i] = bRecivedFrame[5+i];
        }
	}
	/* Get the confByte */
	rxData.confByte = bRecivedFrame[5 + rxData.msgLength];
	/* Get the message length */
	rxData.dataLength = SerialProcess_MessageLenght();
}

/**
  * @brief  Sort received data frame  
  * @param  None
  * @return TRUE for success and FALSE for fail
  */
bool SerialProcess_TypeCommand_Valid()
{
	uint8_t RxCodeType = rxData.code & 0xF0 ;

	switch(RxCodeType)
	{
		/* CONTROLLER_GET commands */
		case CONTROLLER_GET :
			/* GET commands existence*/
			switch(rxData.code)
			{	
                case GET_SERIAL :
                    return true;
                    break;
				default :
					return false;
                    break;
            }
		/* CONTROLLER_SET commands */
		case CONTROLLER_SET :
			/* SET commands existence*/
			switch(rxData.code)
			{	
                case SET_SERIAL :
                    return true;
                    break;
				default :
					return false;
                    break;	
            }
		/* CONTROLLER_TEST commands */
		case CONTROLLER_TEST :
			/* TEST commands existence*/
			switch(rxData.code)
			{	
                case TEST_FLASH :
                    return true;
                    break;
				default :
					return false;
                    break;	
            }
		default :
			return false;
            break;
	}
}

/**
  * @brief  Validate the Frame length
  * @param  None
  * @return TRUE for success and FALSE for fail
  */
bool SerialProcess_FrameLength_Valid()
{
	/* See if rxData frame length is valid with the message length */
    /* 6 is the number of byte without any message reception */
	if(rxData.msgLength + 6 == SerialProcess_MessageLenght())
		return true;
	else
		return false;
}

/**
  * @brief  Calculate the Confirmation Byte 
  * @param  None
  * @return confirmation byte %256
  */
uint8_t SerialProcess_ConfirmationByte(uint8_t Temp_Frame[], uint8_t Temp_Frame_Size)
{
	uint8_t Sum_Byte = 0;
	for (int i=0; i < Temp_Frame_Size; i++ ) // before we add -1 if we have problem to reverify //
		{
			Sum_Byte = Sum_Byte + Temp_Frame[i];
			// Add delay for safety
		}
		Sum_Byte = Sum_Byte % 256;
		// Add delay for safety
	return Sum_Byte;
}

/**
  * @brief  Verify data frame 
  * @param  None
  * @return TRUE for success and FALSE for fail
  */
bool SerialProcess_VerifyDataFrame(void)
{
	/*Sort Data frame from Received String packet */
	SerialProcess_SortData();
	/* if rxData Length is invalid  */
	if(!SerialProcess_FrameLength_Valid())
		return false;
	/*	if rxData frame is corrupted  */
	else if(rxData.confByte != '!') 
		return false;
	/* Verify if rxData device C for Controller */
	else if((rxData.device != CONTROLLER_DEVICE))
		return false;
	/* Verify if the Type Command Data exist*/
	else if(!SerialProcess_TypeCommand_Valid())
		return false;
	/* rxData is valid, send acknowledge */
	else
	{
		return true;
	}
}

// ==================== Public function prototypes ======================== //


/**
  * Execute commands codes for test procedure
  */
void SerialProcess_ExecuteCommand ()
{
    uint8_t RxCode = rxData.code & 0xF0;
    switch(RxCode)
    {
/*---------------------------------------------------------------------*/
        /*----------------------- Test CASES -------------------------*/
        case CONTROLLER_TEST :
            /* GET commands existence */
            switch(rxData.code)
			{
                /* Format External flash  */
                case TEST_FLASH :
                {      
                    if (SF_format() == true)
                        Serial_Acknowledge(TEST_FLASH);
                    else
                        Serial_Error(FRAME_ERR);   
				}
                break;
			}
        /*----------------------- GET CASES -------------------------*/
        case CONTROLLER_GET :
            /* GET commands existence */
			switch(rxData.code)
			{
                /* Get Serial Number from Internal flash Memory */
                case GET_SERIAL :
                    InternalMemory_ReadSerialNumber();
                    Serial_Acknowledge(GET_SERIAL);
                    SerialProcess_CleanFrame();
                    break;
			}
		/*----------------------- SET CASES -------------------------*/
		case CONTROLLER_SET :
            switch(rxData.code)
            {
                /* Set Serial Number into Internal flash Memory */
                case SET_SERIAL:
                    InternalMemory_WriteSerialNumber(rxData.message);
                    Serial_Acknowledge(CONTROLLER_SET);
                    SerialProcess_CleanFrame();
                    break;
            }
/*---------------------------------------------------------------------*/
			break;
			default :

			Serial_Error(FRAME_ERR);
			break;
		}
}

/**
  * Serial Process that be runnig on main 
  */
void SerialProcess_Main()
{
	if (g_data_received_flag)
	{
        /* if the received frame is valid */
		if(SerialProcess_VerifyDataFrame()) 
		{
            SerialProcess_ExecuteCommand();
			g_data_received_flag = false;
            // Add Delay TODO
		}
		else
		{
            SerialProcess_CleanFrame();
			g_data_received_flag = false;
			Serial_Error(FRAME_ERR);
            // Add Delay TODO
        }
	}
}
