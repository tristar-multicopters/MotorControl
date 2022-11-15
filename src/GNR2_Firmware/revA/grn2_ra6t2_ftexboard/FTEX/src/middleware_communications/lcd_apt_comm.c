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

#define APTMAXCURRENT 75  //Should be replaced with a dynamic value

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
                
                LCD_APT_frame_Process(pHandle); //We have received a full frame so do the frame processing
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
void LCD_APT_frame_Process(APT_Handle_t *pHandle)
{
     APT_frame_t replyFrame = {0};
     
     int32_t  toSend      = 0;
     uint32_t Check       = 0;
     uint16_t Merge       = 0;
     uint8_t  PassLvl     = 0;
     uint8_t  LightStatus = 0;
     bool ChangePasFlag = false;      
    
     //Verification of the checksum
     for(int i = 0; i < 6; i += 2) //Checksum is the sum of double bytes into a 16 bits
     {                             //Single bytes need to be paired into a single 16 bits before being summed    
         Merge = (uint16_t) (pHandle->rx_frame.Buffer[i] << 8) +  pHandle->rx_frame.Buffer[i+1];
         Check += Merge;
     }
     
     Check = (Check & 0x0000FFFF); //Protection in case of overflow
    
     //Check if the CRC is good
     if(Check == (uint32_t)(pHandle->rx_frame.Buffer[CHECK + 1] + (pHandle->rx_frame.Buffer[CHECK] << 8)))
     {
         
         LightStatus = ((pHandle->rx_frame.Buffer[PASS]) & 0x10);
         
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
         PassLvl = (pHandle->rx_frame.Buffer[PASS] & 0x0F); //Only the 4 LSB contain the pass level
        
    
          if(PassLvl != PAS_UNCHANGED)
          {                     
              PWRT_SetAssistLevel(pHandle->pVController->pPowertrain,LCD_APT_ConvertPASLevelFromAPT(PassLvl,pHandle->pVController->pPowertrain->sParameters.bMaxLevel));         
          }                      
        
         //Reading the Speed   limit  TBA
         //Reading the Current limit  TBA        
         //Reading the Wheel diameter TBA       
     
    
         //For APT protocol, LSB is sent first for multi-bytes values
         replyFrame.Size = 13;
    
         replyFrame.Buffer[ 0] = APT_START; //Start
      
         //Add up power from both
         toSend = abs(pHandle->pVController->pPowertrain->pMDI->pMCI->pFOCVars->Iqdref.q);
                
         toSend = toSend/(0x7FFF/APTMAXCURRENT); //Conversion from relative current to actual amps
         toSend = toSend * 2;                    //Covert from amps to 0.5 amps; 
        
         replyFrame.Buffer[ 1] = (toSend & 0x000000FF); //Power 0.5 A/unit         

         /* Condition use for wheel speed sensor rpm to send */
         toSend = WheelSpdSensor_GetSpeedRPM(pHandle->pVController->pPowertrain->pWSS); // Getting RPM from Wheel Speed Module
         toSend = toSend * 500;                                                         // Converion from RPM to period in ms 

         toSend = 500000/(toSend/60);
     
         replyFrame.Buffer[ 2] = (toSend & 0x00FF);      // Wheel speed Low half 
         replyFrame.Buffer[ 3] = (toSend & 0xFF00) >> 8; // Wheel speed High half
      
         replyFrame.Buffer[ 4] = 0x00; // Error Code
      
         replyFrame.Buffer[ 5] = 0x04; // Brake Code bXXXX 0100 means the motor is working
                      
         replyFrame.Buffer[ 6] = 0x00;
         
         // If we want to change the PAS level we need to change it here
         replyFrame.Buffer[ 7] = PAS_UNCHANGED; // Send 0x0A unless we want to change the PAS on the screen  
         
         if(ChangePasFlag)
         {
           //change avlue of replyFrame.Buffer[ 7] using LCD_APT_ConvertPASLevelToAPT  
         }
                                       
         replyFrame.Buffer[ 8] = 0x00;
         replyFrame.Buffer[ 9] = 0x00;
    
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
