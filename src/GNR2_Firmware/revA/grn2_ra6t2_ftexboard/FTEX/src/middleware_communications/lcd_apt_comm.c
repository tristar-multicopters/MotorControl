/**
  ******************************************************************************
  * @file    lcd_apt_comm.c
  * @author  Andy Beaudoin, FTEX
  * @brief   High level module that describes APT LCD communication protocol
  *
  ******************************************************************************
*/
    
#include "lcd_apt_comm.h"
#include "ASSERT_FTEX.h"

extern osThreadId_t COMM_Uart_handle; // Task Id for UART

/**
 *  Function for initializing the LCD module with the APT protocol
 *  It links its own custom interrupt routines with the UART callbacks using
 *  a void pointer. 
 */
void LCD_APT_init(APT_Handle_t *pHandle,VCI_Handle_t *pVCIHandle, UART_Handle_t *pUARTHandle)
{
    ASSERT(pHandle     != NULL);
    ASSERT(pVCIHandle  != NULL);
    ASSERT(pUARTHandle != NULL);
    
    pHandle->pVController = pVCIHandle;        // Pointer to VController
    pHandle->pUART_handle = pUARTHandle;       // Pointer to UART instance  
    pHandle->pUART_handle->Super = pHandle;    // Initialise the super pointer that the UART needs   
    
    pHandle->pUART_handle->pRxCallback = &LCD_APT_RX_IRQ_Handler;   // Link the interrupts from the UART instance to this module
    pHandle->pUART_handle->pTxCallback = &LCD_APT_TX_IRQ_Handler; 
    

    pHandle->pUART_handle->UARTBaudrate = BAUD9600;

    LCD_APT_ClearAllErrors(pHandle);    
    
    pHandle->pUART_handle->UARTBaudrate = BAUD9600;
    uCAL_UART_Init(pHandle->pUART_handle);  // Initialise the UART module with the baudrate that APT screen needs  
    uCAL_UART_Receive(pHandle->pUART_handle, &(pHandle->RxByte), sizeof(pHandle->RxByte));  // Start the first reception   
}

/** 
 *  Function for building a frame specific to the APT protocol
 *  It is interrupt driven until a frame is completed where it  
 *  unblocks a comm task to process the frame.
 *  It is based on a byte by byte reception 
 */
void LCD_APT_RX_IRQ_Handler(void *pVoidHandle)
{     
    ASSERT(pVoidHandle != NULL);    
    APT_Handle_t *pHandle = pVoidHandle;  // Convert the void handle pointer to a handle pointer
    
    if(pHandle->RxCount >= RX_BYTE_BUFFER_SIZE) //Overflow safety
    {
       pHandle->RxCount = 0;
    }
    
    
    pHandle->RxBuffer[pHandle->RxCount] = pHandle->RxByte;
    pHandle->RxCount++;
    
    osThreadFlagsSet(COMM_Uart_handle, UART_FLAG); // Notify task that a byte has been received 
    uCAL_UART_Receive(pHandle->pUART_handle, &(pHandle->RxByte), sizeof(pHandle->RxByte));    
}

/**
 *  Function for sending a frame specific to the APT protocol
 *  once a frame has been received and processed this function
 *  sends the response made byt the frame process function.  
 *  It is based on a byte by byte transmission.   
 */
void LCD_APT_TX_IRQ_Handler(void *pVoidHandle)
{
    ASSERT(pVoidHandle != NULL);    
    APT_Handle_t *pHandle = pVoidHandle;  
   
    uint8_t tx_data;

    if(pHandle->tx_frame.ByteCnt < pHandle->tx_frame.Size) // Do we hav bytes to send ?
    {
        tx_data = pHandle->tx_frame.Buffer[pHandle->tx_frame.ByteCnt];
        pHandle->tx_frame.ByteCnt ++;
        // Send the data byte 
        uCAL_UART_Transmit(pHandle->pUART_handle, &tx_data, 1); 
    }
    else //If not then resume recepetion for the next frame
    {             
        pHandle->tx_frame.ByteCnt = 0;
        uCAL_UART_Receive(pHandle->pUART_handle, &(pHandle->RxByte), sizeof(pHandle->RxByte));
    }
}

/**
 *  Task for the APT screen
 *
 *
 */
void LCD_APT_Task(APT_Handle_t *pHandle)
{
   
    uint8_t ByteCount = pHandle->rx_frame.ByteCnt;
    uint8_t NextByte;    
    uint8_t BytesReceived[16];
    uint8_t NumbByteRx;
    
    
    NumbByteRx = pHandle->RxCount; //Save the number of bytes we have received but reset the variable so we dont miss any new bytes
    pHandle->RxCount = 0;
    
    for(int BIdx = 0; BIdx < NumbByteRx; BIdx++)
    {
        BytesReceived[BIdx] = pHandle->RxBuffer[BIdx]; //Save a local copy of the bytes we have received       
    }    
    
    for(int i = 0; i < NumbByteRx; i++) //Continue building the frame with the bytes we've received   
    {
        NextByte = BytesReceived[i];
        
        if(ByteCount == 0) // Each case represents the number of the byte received in a frame.                    
        {
             if(NextByte == APT_START) // Is it the proper start byte ?
             {
                 pHandle->rx_frame.Buffer[ByteCount] = NextByte;
                 pHandle->rx_frame.ByteCnt ++;                 
             }
             else // If not, its a bad cmd
             {
                 pHandle->rx_frame.ByteCnt = 0;
             }
             
        }
        else if(ByteCount > 0 && ByteCount < 8)
        {
            // Byte 1 to byte 7 contain various information
            // They dont need to be checked because as long as 
            // the checksum is good then they are valid
            // The checksum is verified in the frame process function
                 
            pHandle->rx_frame.Buffer[ByteCount] = NextByte;
            pHandle->rx_frame.ByteCnt ++;
                
        }
        else if(ByteCount == 8) // Every frame has a lenght of 9 bytes (0-8)
        {
                                        
            if(NextByte == APT_END) // Is it the proper end byte ?
            {
                pHandle->rx_frame.Buffer[ByteCount] = NextByte;
                
                LCD_APT_ProcessFrame(pHandle); //We have received a full frame so do the frame processing
            }
            pHandle->rx_frame.ByteCnt = 0;
        }
        else
        {            
            // We should never get here but if we do trash the frame
            pHandle->rx_frame.ByteCnt = 0;                    
        }
    }        
}


/**
 *  Function for decoding a received frame (previously built in the RxCallback function)
 *  according to the APT screen protocol.
 *  This is executed in a comm task that gets unblocked when a complete frame is received.
 *
 */
void LCD_APT_ProcessFrame(APT_Handle_t *pHandle)
{
    APT_frame_t replyFrame = {0};
     
    int32_t  toSend      = 0;
    uint32_t Check       = 0;
    uint16_t Merge       = 0;
    uint8_t  PassLvl     = 0;
    uint8_t  LightStatus = 0;
     
    
    //Verification of the checksum
    for(int i = 0; i < 6; i += 2) //Checksum is the sum of double bytes into a 16 bits
    {                             //Single bytes need to be paired into a single 16 bits before being summed    
        Merge = (uint16_t) (pHandle->rx_frame.Buffer[i] << 8) +  pHandle->rx_frame.Buffer[i+1];
        Check += Merge;
    }
     
    if(Check > 0x0000FFFF) // Small patch to accomodate bug in APT screen
    {
        Check += 0x0000FFFF & (Check >> 16);      
    }
     
    Check = (Check & 0x0000FFFF); //Protection in case of overflow
    
    //Check if the CRC is good
    if(Check == (uint32_t)(pHandle->rx_frame.Buffer[CHECK + 1] + (pHandle->rx_frame.Buffer[CHECK] << 8)))
    {
         
        LightStatus = ((pHandle->rx_frame.Buffer[PAS]) & 0x10);
         
        if(LightStatus) //Operate the light according to what the screne tells us
        {
            Light_Enable(pHandle->pVController->pPowertrain->pHeadLight);  
            Light_Enable(pHandle->pVController->pPowertrain->pTailLight); 
        }
        else
        {
            Light_Disable(pHandle->pVController->pPowertrain->pHeadLight); 
            Light_Disable(pHandle->pVController->pPowertrain->pTailLight);              
        }   
         
        //Reading the Pass
        PassLvl = (pHandle->rx_frame.Buffer[PAS] & 0x0F); //Only the 4 LSB contain the pass level
        
    
        if(PassLvl != PAS_UNCHANGED)
        {                  
            PedalAssist_SetAssistLevel(pHandle->pVController->pPowertrain->pPAS,LCD_APT_ConvertPASLevelFromAPT(PassLvl,pHandle->pVController->pPowertrain->pPAS->sParameters.bMaxLevel));         
            pHandle->OldPAS = PassLvl;
            pHandle->APTChangePasFlag = true;
        }                      
        
        //Reading the Speed   limit  TBA
        
        #ifdef SCREENPOWERCONTROL
        
        // Reading the Current limit          
        uint16_t CurrentLimit;
        CurrentLimit = pHandle->rx_frame.Buffer[CURRENTL];
        
        PWRT_SetOngoingMaxCurrent(pHandle->pVController->pPowertrain, CurrentLimit);
        
        #endif
         
        //Reading and updating the Wheel diameter 
        pHandle->WheelDiameter = LCD_APT_CalculateWheelDiameter(pHandle->rx_frame.Buffer[WHEELD]);
        CanVehiInterface_UpdateWheelDiamater(pHandle->WheelDiameter);
    
        //For APT protocol, LSB is sent first for multi-bytes values
        replyFrame.Size = 13;
    
        replyFrame.Buffer[ 0] = APT_START; //Start
      
        //Get the amount of amps we are currently pushing 
        toSend = PWRT_GetTotalMotorsCurrent(pHandle->pVController->pPowertrain);
         
        toSend = LCD_APT_ApplyPowerFilter((uint16_t)toSend);  
          
        toSend = toSend * 2; //Covert from amps to 0.5 amps; 
        
        replyFrame.Buffer[1] = (toSend & 0x000000FF); //Power 0.5 A/unit         

        /* Condition use for wheel speed sensor rpm to send */
        toSend = WheelSpdSensor_GetSpeedRPM(pHandle->pVController->pPowertrain->pPAS->pWSS); // Getting RPM from Wheel Speed Module
                   
        toSend =  LCD_APT_ApplySpeedFilter((uint16_t) toSend);
                  
        toSend = toSend * 500; //Converion from RPM to period in ms, 500 is used as scaling to conserve precision          
         
        if(toSend != 0) //verify toSend value to avoid a division by zero.
        {
            toSend = 500000/(toSend/60); // Descaling here with the 500000 to return to original unit            
        }
            
         
        replyFrame.Buffer[2] = (toSend & 0x00FF);      // Wheel speed Low half 
        replyFrame.Buffer[3] = (toSend & 0xFF00) >> 8; // Wheel speed High half
      
        replyFrame.Buffer[4] = (uint8_t) LCD_APT_CycleError(pHandle); // Error Code
      
        replyFrame.Buffer[5] = 0x04; // Brake Code bXXXX 0100 means the motor is working
                      
        replyFrame.Buffer[6] = 0x00;
         
        // If we want to change the PAS level we need to change it here
        replyFrame.Buffer[7] = PAS_UNCHANGED; // Send 0x0A unless we want to change the PAS on the screen  
         
        if(pHandle->CanChangePasFlag || pHandle->OldPAS != LCD_APT_ConvertPASLevelToAPT(PedalAssist_GetAssistLevel(pHandle->pVController->pPowertrain->pPAS)))
        {
             pHandle->CanChangePasFlag = false;
             replyFrame.Buffer[ 7] =  LCD_APT_ConvertPASLevelToAPT(PedalAssist_GetAssistLevel(pHandle->pVController->pPowertrain->pPAS)); 
             pHandle->OldPAS = LCD_APT_ConvertPASLevelToAPT(PedalAssist_GetAssistLevel(pHandle->pVController->pPowertrain->pPAS));
        }
                                       
        replyFrame.Buffer[8] = 0x00;
        replyFrame.Buffer[9] = 0x00;
    
        //Calculate checksum
        //Sum of paired bytes
        Merge = 0;
        Check = 0;
        for(uint16_t i = 0; i < NUMBER_OF_CRC_BYTES; i += BYTE_PER_PAIR)
        {
             Merge = (uint16_t) (replyFrame.Buffer[i+1] << 8) + replyFrame.Buffer[i];
             Check += Merge;
        }
        
        replyFrame.Buffer[10] =  (0x000000FF & Check);       // Checksum Low  Half
        replyFrame.Buffer[11] = ((0x0000FF00 & Check) >> 8); // Checksum High Half    
            
        replyFrame.Buffer[12] = APT_END; // End
            
        replyFrame.ByteCnt = 0;   
        
        pHandle->tx_frame = replyFrame; 
             
        LCD_APT_TX_IRQ_Handler((void *) pHandle); // Start the transmission of the answer frame
            
    }// End of CRC check
       
}


/**  
 *  Function used to apply a filter on the speed we send to the screen 
 */
uint16_t LCD_APT_ApplySpeedFilter(uint16_t aInstantSpeedInRPM)
{
   static uint16_t SpeedAvg; 
   uint16_t Bandwidth = 3; //Bandwidth CANNOT be set to 0
    
   SpeedAvg = ((Bandwidth-1) * SpeedAvg + aInstantSpeedInRPM)/Bandwidth;
   
   return (SpeedAvg); 
}    

/**  
 *  Function used to apply a filter on the power we send to the screen 
 */
uint16_t LCD_APT_ApplyPowerFilter(uint16_t aInstantPowerInAmps)
{
   static uint16_t PowerAvg; 
   uint16_t Bandwidth = 3; //Bandwidth CANNOT be set to 0
        
   PowerAvg = ((Bandwidth-1) * PowerAvg + aInstantPowerInAmps)/Bandwidth;
   
   return (PowerAvg); 
} 

/** Function used to translate the PAS level received from the APT  
 *  screen standard to the FTEX standard
 */
uint8_t LCD_APT_ConvertPASLevelToAPT(PasLevel_t aPAS_Level)
{
   uint8_t PAS_Out = PAS_LEVEL_0; 
   switch(aPAS_Level)
   {
       case PAS_LEVEL_0:
           PAS_Out = 0x0;
           break;
       case PAS_LEVEL_1:
           PAS_Out = 0x1;
           break;
       case PAS_LEVEL_2:
           PAS_Out = 0x2;
           break;
       case PAS_LEVEL_3:
           PAS_Out = 0x3;
           break;
       case PAS_LEVEL_4:
           PAS_Out = 0x4;
           break;
       case PAS_LEVEL_5:
           PAS_Out = 0x5;
           break;
       case PAS_LEVEL_6:
           PAS_Out = 0x6;
           break;
       case PAS_LEVEL_7:
           PAS_Out = 0x7;
           break;
       case PAS_LEVEL_8:
           PAS_Out = 0x8;
           break;
       case PAS_LEVEL_9:
           PAS_Out = 0x9;
           break;
       case PAS_LEVEL_WALK: // For now we cant control walk mode
       default:  
           PAS_Out = 0x0;           
           break;
       
   }
   return PAS_Out;
}

/** Function used to translate the FTEX standard PAS level to the APT  
 *  screen standard.(is not the same as when we receive a PAS level from the APT screen)
 */
PasLevel_t LCD_APT_ConvertPASLevelFromAPT(uint8_t aPAS_Level, uint8_t aNumberOfLevels)
{
   PasLevel_t PAS_Out = 0x0;
   
    bool ExtendedPASLevels;
    
   if (aNumberOfLevels == 5)  // Check if we need to use the 5 PAS config
   {
       ExtendedPASLevels = false;
   }    
   else if (aNumberOfLevels == 9) // Or the 9 PAS config
   {
       ExtendedPASLevels = true; 
   }       
   else
   {
       ASSERT(false); // Number of PAS levels not compatible with APT screen  
   }       
   
   //PAS level from APT 0x02,0x04,0x06,0x08 are only for 9 PAS levels mode
   
   switch(aPAS_Level)
   {
       case 0x0:
           PAS_Out = PAS_LEVEL_0;
           break;
       case 0x1:
           PAS_Out = PAS_LEVEL_1;
           break;
       case 0x2:
           if(ExtendedPASLevels)
               PAS_Out = PAS_LEVEL_2;    
       case 0x3:
           if(ExtendedPASLevels)
               PAS_Out = PAS_LEVEL_3;
           else
               PAS_Out = PAS_LEVEL_2;
           break;
       case 0x4:
            if (ExtendedPASLevels)
               PAS_Out = PAS_LEVEL_4;
           break;           
       case 0x5:
           if (ExtendedPASLevels)
               PAS_Out = PAS_LEVEL_5; 
           else    
               PAS_Out = PAS_LEVEL_3;
           break;
       case 0x6:
           if (ExtendedPASLevels)
               PAS_Out = PAS_LEVEL_6;            
           break;           
       case 0x7:
           if(ExtendedPASLevels)
               PAS_Out = PAS_LEVEL_7;
           else           
               PAS_Out = PAS_LEVEL_4;
           break;
       case 0x8:
           if (ExtendedPASLevels)
               PAS_Out = PAS_LEVEL_8; 
       case 0x9:
           if (ExtendedPASLevels)
               PAS_Out = PAS_LEVEL_9; 
           else    
               PAS_Out = PAS_LEVEL_5;
           break;
       case 0xF:
           PAS_Out = PAS_LEVEL_WALK;
           break;
       default:
           PAS_Out = PAS_LEVEL_0;
           break;
       
   }

   return PAS_Out;
}

/** Function used to translate the FTEX standard PAS level to the APT  
 *  screen standard.(is not the same as when we receive a PAS level from the APT screen)
 */
uint8_t LCD_APT_CalculateWheelDiameter(uint16_t aValue)
{
   uint8_t WheelDiameter; 
    
   if(aValue <= APT_WHEEL_DIAM_INCHES_MAX && aValue > 0)
   {
       WheelDiameter = (uint8_t) aValue;
   } 
   else if (aValue > APT_WHEEL_CIRCUMFERENCE_MIN)
   {
       WheelDiameter = (uint8_t) round(((float) (aValue/FTEX_PI)/ (float) FTEX_INCH_TO_CM)); // Convert from circumference in cm to diamater in inches     
   }
   else
   {
       WheelDiameter = WHEEL_DIAMETER_DEFAULT;   
   }   

   return WheelDiameter;
}

/**
 *  Function used to raise a specific error on the screen, cannot raise the same error twice. 
 */
void LCD_APT_RaiseError(APT_Handle_t *pHandle, APT_ErrorCodes_t aError)
{
    bool ErrorAlreadyRaised = false;
    
    for (uint16_t u = 0; u < APT_ERROR_BUFFER_SIZE; u ++) // Check if the error is already raised
    {
       if (pHandle->ErrorCodes[u] == aError)
       {
           ErrorAlreadyRaised = true; 
       }
    }    
    
    if (ErrorAlreadyRaised == false) // If the error is already raised there is not point in raising it again
    {        
        if (pHandle->NumberErrors < APT_ERROR_BUFFER_SIZE) // If the error buffer isn't full 
        {
            pHandle->NumberErrors ++; // increment the counter
        }
        else // If its full erase the oldlest error and add the new one
        {
            for (uint16_t i = 0; i < APT_ERROR_BUFFER_SIZE - 1 ; i ++) // Shift the other ones to make room at the end of the buffer
            {
                pHandle->ErrorCodes[i] = pHandle->ErrorCodes[i + 1];        
            }                        
        }
    
        pHandle->ErrorCodes[pHandle->NumberErrors-1] = aError; //Add the new error
    }
}

/**
 *  Function used to clear a specific error on the screen.
 */
void LCD_APT_ClearError(APT_Handle_t *pHandle, APT_ErrorCodes_t aError)
{
    bool ErrorFound = false;
    bool ErrorCleared = false;
    
    for (uint16_t i = 0; i < pHandle->NumberErrors; i++) // Cycle through the buffer once we find the error 
    {                                                     // shift the others to ensure there is no empty spot 
                                                         // in the middle of the buffer.
    
        if (pHandle->ErrorCodes[i] == aError) // Is this error raised in the buffer ?
        {
            ErrorFound = true;  
        }            
        
        if (ErrorFound) // If the error is present in the buffer
        {
            if (i < pHandle->NumberErrors - 1) // Making sure we are not on the last error present in the buffer
            {
                pHandle->ErrorCodes[i] = pHandle->ErrorCodes[i+1]; // If not shift the other errors to ensure
            }                                                      // we don't have an empty spot in the buffer
            else // If we are on the last error just clear it
            {
                pHandle->ErrorCodes[i] = NO_ERROR;
                ErrorCleared = true;                
            }                 
        }      
    }
    
    if (ErrorCleared) // If an error was cleared reflect the change in the error counter
    {
        pHandle->NumberErrors --;
    }
}

/**
 *  Function used to clear all of the errors present in the buffer
 */
void LCD_APT_ClearAllErrors(APT_Handle_t *pHandle)
{
    for (uint16_t i = 0; i < pHandle->NumberErrors; i++)
    {
       pHandle->ErrorCodes[i] = NO_ERROR; // Clear all of the raised errors
    }
    
    pHandle->NumberErrors = 0; // Reset the error counter
    
}   


/** Function used to cycle the next error to show on the screen 
 *  The speed at which the error cylce is define by a constant in the function
 */
APT_ErrorCodes_t LCD_APT_CycleError(APT_Handle_t *pHandle)
{
    const  uint8_t CycleLenght = 6; // Number of cycles that we show an error before switching
                                    // A cycle is about 150 ms 
    
    static uint8_t CycleTracking;   // Keeps track how many cycles has the current error been shown for
    
    
    static  uint16_t ShownErrorIndex;
    static APT_ErrorCodes_t ErrorShown; 
    
    
    if (CycleTracking >= CycleLenght) // Check if we need to cycle to show the next error
    {
        CycleTracking = 0;
        
        if (pHandle->NumberErrors <= 0) // If the error(s) got cleared since the last cycle
        {
            ErrorShown = NO_ERROR;     // Show no error  
            ShownErrorIndex = 0;
        }       
        else 
        {    
            if (ShownErrorIndex >= pHandle->NumberErrors - 1) // reset the cycle if we have shown all of them
            {
                ShownErrorIndex = 0;        
            }   
            else   // If not incremente the index to shw the next error in line
            {
                ShownErrorIndex ++;                        
            }
        
            ErrorShown = pHandle->ErrorCodes[ShownErrorIndex];
        }
    }
    else
    {
        CycleTracking ++; 
        
    }        
   
    return ErrorShown;
}

