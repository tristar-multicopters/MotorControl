/**
  ******************************************************************************
  * @file    lcd_KD718.c
  * @author  Andy Beaudoin, FTEX
  * @brief   High level module that describes KD718 LCD communication protocol
  *
  ******************************************************************************
*/
    
#include "lcd_KD718.h"
#include "ASSERT_FTEX.h"

extern osThreadId_t COMM_Uart_handle; // Task Id for UART

/**
 *  Function for initializing the LCD module with the KD718 protocol
 *  It links its own custom interrupt routines with the UART callbacks using
 *  a void pointer. 
 */
void LCD_KD718_init(KD718_Handle_t *pHandle,VCI_Handle_t *pVCIHandle, UART_Handle_t *pUARTHandle)
{
    ASSERT(pHandle     != NULL);
    ASSERT(pVCIHandle  != NULL);
    ASSERT(pUARTHandle != NULL);
    
    pHandle->pVController = pVCIHandle;        // Pointer to VController
    pHandle->pUART_handle = pUARTHandle;       // Pointer to UART instance  
    pHandle->pUART_handle->Super = pHandle;    // Initialise the super pointer that the UART needs 

    pHandle->EcoMode = false;
    pHandle->EcoModePowerPerCent = DEFAULT_ECOMODE_POWER;    
    
    pHandle->pUART_handle->pRxCallback = &LCD_KD718_RX_IRQ_Handler;   // Link the interrupts from the UART instance to this module
    pHandle->pUART_handle->pTxCallback = &LCD_KD718_TX_IRQ_Handler; 
        
    VC_Errors_SetCycleLength(0);
    
    pHandle->pUART_handle->UARTBaudrate = BAUD1200;
    uCAL_UART_Init(pHandle->pUART_handle);  // Initialise the UART module with the baudrate that KD718 screen needs  
    uCAL_UART_Receive(pHandle->pUART_handle, &(pHandle->RxByte), sizeof(pHandle->RxByte));  // Start the first reception   
}

/** 
 *  Function for building a frame specific to the KD718KD718 protocol
 *  It is interrupt driven until a frame is completed where it  
 *  unblocks a comm task to process the frame.
 *  It is based on a byte by byte reception 
 */
void LCD_KD718_RX_IRQ_Handler(void *pVoidHandle)
{     
    ASSERT(pVoidHandle != NULL);    
    KD718_Handle_t *pHandle = pVoidHandle;  // Convert the void handle pointer to a handle pointer
    
    if(pHandle->RxCount >= KD718_RX_BYTE_BUFFER_SIZE) //Overflow safety
    {
       pHandle->RxCount = 0;
    }
    
    
    pHandle->RxBuffer[pHandle->RxCount] = pHandle->RxByte;
    pHandle->RxCount++;
    
    osThreadFlagsSet(COMM_Uart_handle, UART_FLAG); // Notify task that a byte has been received 
    uCAL_UART_Receive(pHandle->pUART_handle, &(pHandle->RxByte), sizeof(pHandle->RxByte));    
}

/**
 *  Function for sending a frame specific to the KD718 protocol
 *  once a frame has been received and processed this function
 *  sends the response made byt the frame process function.  
 *  It is based on a byte by byte transmission.   
 */
void LCD_KD718_TX_IRQ_Handler(void *pVoidHandle)
{
    ASSERT(pVoidHandle != NULL);    
    KD718_Handle_t *pHandle = pVoidHandle;  
   
    uint8_t tx_data;

    if(pHandle->tx_frame.ByteCnt < pHandle->tx_frame.Size) // Do we have bytes to send ?
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
 *  Task for the KD718 screen
 *
 *
 */
void LCD_KD718_Task(KD718_Handle_t *pHandle)
{
    ASSERT(pHandle != NULL);  
    uint8_t ByteCount = pHandle->rx_frame.ByteCnt;
    uint8_t NextByte;    
    uint8_t BytesReceived[16];
    uint8_t NumbByteRx;
    static uint8_t NbBytePerFrame;
    
    
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
             if(NextByte == READ_COMMAND) // 
             {
                 NbBytePerFrame = 2; // Read commands are composed of 2 bytes
                 pHandle->rx_frame.Buffer[ByteCount] = NextByte;
                 pHandle->rx_frame.ByteCnt ++;                 
             }
             else if (NextByte == WRITE_COMMAND)
             {
                 pHandle->rx_frame.Buffer[ByteCount] = NextByte;
                 pHandle->rx_frame.ByteCnt ++;            
             }
             else // If not, its a bad cmd
             {
                 pHandle->rx_frame.ByteCnt = 0;
                 NbBytePerFrame = 0;
                 uCAL_UART_SendDefault(pHandle->pUART_handle);
             }
             
        }
        else if(ByteCount > 0 && ByteCount < 5) // The command was received but we are waiting for other remaining bytes
        {                 
            pHandle->rx_frame.Buffer[ByteCount] = NextByte;
            
            if (pHandle->rx_frame.Buffer[0] == READ_COMMAND) // Do we all the bytes for this frame ?
            {
                if (NbBytePerFrame == ByteCount + 1)
                {
                    pHandle->rx_frame.ByteCnt = 0;
                    LCD_KD718_ProcessFrame(pHandle); 
                }
                else
                {
                    pHandle->rx_frame.ByteCnt ++;
                }              
            }
            else if (pHandle->rx_frame.Buffer[0] == WRITE_COMMAND) // Is it a write cmd
            {
                if ((pHandle->rx_frame.Buffer[1] == W_PAS)    || (pHandle->rx_frame.Buffer[1] == W_PWRMODE) || 
                    (pHandle->rx_frame.Buffer[1] == W_LIGHTS) || (pHandle->rx_frame.Buffer[1] == W_SPEED_LIMIT)) // Is it writiing a PAS level or Power mode
                {
                    if (ByteCount == 1) 
                    {   
                        if ((pHandle->rx_frame.Buffer[1] == W_PAS) || (pHandle->rx_frame.Buffer[1] == W_PWRMODE))
                        {
                            NbBytePerFrame = 4; // If we are receiving a PAS level  or power mode, frame length is 4 
                        }
                        else if (pHandle->rx_frame.Buffer[1] == W_LIGHTS)
                        {
                            NbBytePerFrame = 3; // If we are receiving lights state, frame length is 3   
                        }
                        else if (pHandle->rx_frame.Buffer[1] == W_SPEED_LIMIT)
                        {
                            NbBytePerFrame = 5; // If we are receiving speed limit and wheel diam, frame length is 5
                        }                                                                                                                         
                    }                   
                    else if (NbBytePerFrame == ByteCount + 1)
                    {
                        pHandle->rx_frame.ByteCnt = 0;
                        LCD_KD718_ProcessFrame(pHandle);
                    }
                   
                    pHandle->rx_frame.ByteCnt ++;
                }
                else // Trash the frame for unkown write sub command
                {   
                    NbBytePerFrame = 0;
                    pHandle->rx_frame.ByteCnt = 0;
                    uCAL_UART_SendDefault(pHandle->pUART_handle);
                }              
            }
            else // Trash the frame for unkwon write command
            {
                 NbBytePerFrame = 0;
                 pHandle->rx_frame.ByteCnt = 0;
                 uCAL_UART_SendDefault(pHandle->pUART_handle);
            }                   
        }
        else
        {            
            // We should never get here but if we do trash the frame
            NbBytePerFrame = 0;
            pHandle->rx_frame.ByteCnt = 0; 
            uCAL_UART_SendDefault(pHandle->pUART_handle);            
        }
    }        
}


/**
 *  Function for decoding a received frame (previously built in the RxCallback function)
 *  according to the KD718 screen protocol.
 *  This is executed in a comm task that gets unblocked when a complete frame is received.
 *
 */
void LCD_KD718_ProcessFrame(KD718_Handle_t *pHandle)
{
    ASSERT(pHandle != NULL);
    KD718_frame_t replyFrame = {0};    
    int32_t toSend      = 0;
    uint8_t Status_to_Send = 0;

  
    if(pHandle->rx_frame.Buffer[0] == READ_COMMAND)
    {
        switch(pHandle->rx_frame.Buffer[1])
        {
            case R_PROTOCOL:
                replyFrame.Size = 3;
                replyFrame.Buffer[0] = 0x90;
                replyFrame.Buffer[1] = 0x40;
                replyFrame.Buffer[2] = 0xD0;
              break;
            case R_STATUS:            
                replyFrame.Size = 1;
                Status_to_Send = LCD_KD718_ErrorConversionFTEXToKD718(VC_Errors_CycleError());            
                replyFrame.Buffer[0] = Status_to_Send;               
              break;
            case R_PWN:
                replyFrame.Size = 1;
                replyFrame.Buffer[0] = 0x00; // PWM 0%
              break;
            case R_MOTOR_COMU:
              break;
            case R_VOLTAGE:
                replyFrame.Size = 2;
                replyFrame.Buffer[0] = 0x02; // Should be 36.02V
                replyFrame.Buffer[1] = 0x9E;
              break;                                                                      
            case R_CURRENT:
                         //Get the amount of amps we are currently pushing 
                toSend = PWRT_GetTotalMotorsCurrent(pHandle->pVController->pPowertrain); 
          
                //No filtering needed for this LCD
            
                toSend = toSend * 2; //Covert from amps to 0.5 amps;
                
                replyFrame.Size = 2;
                replyFrame.Buffer[0] = (toSend & 0x000000FF); // Send current
                replyFrame.Buffer[1] = (toSend & 0x000000FF);
            
              break;                                                                      
            case R_MODE:
              break;
            case R_THROTTLE_TARGET:
              break;
            case R_BRAKE_STATE:
                replyFrame.Size = 1;
                replyFrame.Buffer[0] = 0x00; // Brake off ?                
              break;
            case R_ACC_PULSE_NB:
              break;
            case R_SPD_HALL_CYCLE:                
              break;
            case R_SPEED:
                replyFrame.Size = 3;
            
                toSend = WheelSpdSensor_GetSpeedRPM(pHandle->pVController->pPowertrain->pPAS->pWSS); // Getting RPM from Wheel Speed Module
                  
                //No filtering needed for this LCD
            
                replyFrame.Buffer[0] = (toSend & 0xFF00) >> 8;
                replyFrame.Buffer[1] = (toSend & 0x00FF);            
                replyFrame.Buffer[2] = (0x20 + (toSend & 0x00FF) + ((toSend & 0xFF00) >> 8)) & 0x00FF;
               break; 
            default:
              break;
        }
        
    }
    else if (pHandle->rx_frame.Buffer[0] == WRITE_COMMAND)
    {
        replyFrame.Size = 0;
        uint8_t CRC;
        switch(pHandle->rx_frame.Buffer[1])
        {        
            case W_PAS: // We receive a new PAS level
                    
                CRC = 0;                           // This frame has a CRC so check it
                CRC += pHandle->rx_frame.Buffer[0];
                CRC += pHandle->rx_frame.Buffer[1];
                CRC += pHandle->rx_frame.Buffer[2];
            
                if (CRC == pHandle->rx_frame.Buffer[3]) // If the CRC is good update the new PAS level
                { 
                    uint8_t PASLvl = 0; 
                
                    PASLvl = pHandle->rx_frame.Buffer[2];
                
                    PedalAssist_SetAssistLevel(pHandle->pVController->pPowertrain->pPAS,
                    LCD_KD718_ConvertPASLevelFromKD718(PASLvl,pHandle->pVController->pPowertrain->pPAS->sParameters.bMaxLevel));   
                }                    
              break;
            case W_LIGHTS:
                
                if((pHandle->rx_frame.Buffer[2] & 0x01)) //Operate the light according to what the screne tells us
                {
                    Light_Enable(pHandle->pVController->pPowertrain->pHeadLight);  
                    Light_Enable(pHandle->pVController->pPowertrain->pTailLight); 
                }
                else
                {
                    Light_Disable(pHandle->pVController->pPowertrain->pHeadLight); 
                    Light_Disable(pHandle->pVController->pPowertrain->pTailLight);              
                }  
                
              break;
            case W_PWRMODE:
                
            #ifdef SCREENPOWERCONTROL // Safety to control if the screen is allowed to change the power of the vehicle
                CRC = 0;                           // This frame has a CRC so check it
                CRC += pHandle->rx_frame.Buffer[0];
                CRC += pHandle->rx_frame.Buffer[1];
                CRC += pHandle->rx_frame.Buffer[2];
            
                if (CRC == pHandle->rx_frame.Buffer[3]) // If the CRC is good update the new PAS level
                { 
                    if (pHandle->rx_frame.Buffer[2] == 0x02 || pHandle->rx_frame.Buffer[2] == 0x04)
                    {
                        replyFrame.ByteCnt = 4;       // Filling the acknowledge frame
                        replyFrame.Buffer[0] = 0x16;
                        replyFrame.Buffer[1] = 0x0C;
                        
                        if (pHandle->rx_frame.Buffer[2] == 0x02)
                        {
                            replyFrame.Buffer[2] = 0x02;  // Value expected when we acknowledge Eco mode change
                            
                            LCD_KD718_EnableEcoMode(pHandle);                            
                        }
                        else
                        {
                            replyFrame.Buffer[2] = 0x04;  // Value expected when we acknowledge Sport mode change
                            
                            LCD_KD718_DisableEcoMode(pHandle);
                        }
                        // Calculating the CRC for the acknowledge frame                       
                        replyFrame.Buffer[3] = 0x00FF & (replyFrame.Buffer[0] + replyFrame.Buffer[1] + replyFrame.Buffer[2]);                    
                    }                    
                }                    
            #endif
              break;
            case W_SPEED_LIMIT:
                CRC = 0;   // This frame has a CRC so check it
                CRC += pHandle->rx_frame.Buffer[0];
                CRC += pHandle->rx_frame.Buffer[1];
                CRC += pHandle->rx_frame.Buffer[2];
                CRC += pHandle->rx_frame.Buffer[3];
                 
                if (CRC == pHandle->rx_frame.Buffer[4]) // If the CRC is good update the new Speed limit
                {
                    #if DYNAMIC_SPEED_LIMITATION
                    uint16_t SpeedLimit = 0;
                    SpeedLimit += (pHandle->rx_frame.Buffer[2] << 8);
                    SpeedLimit +=  pHandle->rx_frame.Buffer[3];
                    
                    SpeedLimit = Wheel_GetSpeedFromWheelRpm(SpeedLimit);
                    
                    // setting the max for any speed limits
                    Throttle_SetMaxSpeed(pHandle->pVController->pPowertrain->pThrottle,SpeedLimit); 
                    PedalAssist_SetTorquePASMaxSpeed(pHandle->pVController->pPowertrain->pPAS,SpeedLimit); 
                    #endif                     
                }
              break;                
          default:
              break;
        }
    }                   
    
    replyFrame.ByteCnt = 0;   
        
    pHandle->tx_frame = replyFrame; 
             
    LCD_KD718_TX_IRQ_Handler((void *) pHandle); // Start the transmission of the answer frame     
}

/** Function used to translate the PAS level received from the KD718  
 *  screen standard to the FTEX standard
 */
uint8_t LCD_KD718_ConvertPASLevelToKD718(PasLevel_t aPAS_Level)
{
   uint8_t PAS_Out = PAS_LEVEL_0; 
    
   switch(aPAS_Level)
   {
       case PAS_LEVEL_0: //This function is a place holer, will have to be readjusted when controler can set PAS on KD718
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

/** Function used to translate the FTEX standard PAS level to the KD718  
 *  screen standard.(is not the same as when we receive a PAS level from the KD718 screen)
 */
PasLevel_t LCD_KD718_ConvertPASLevelFromKD718(uint8_t aPAS_Level, uint8_t aNumberOfLevels)
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
        ASSERT(false); // Number of PAS levels not compatible with KD718 screen 
        ExtendedPASLevels = false;        
    }        
   
    if (ExtendedPASLevels)
    {
         ASSERT(false); // Extended not yet supported
    } 
    
    switch(aPAS_Level)
    {
        case 0x0:
            PAS_Out = PAS_LEVEL_0;
          break;
        case 0x0B:
            PAS_Out = PAS_LEVEL_1;
          break;       
        case 0x0D:
            PAS_Out = PAS_LEVEL_2;
          break;           
        case 0x15:
            PAS_Out = PAS_LEVEL_3;            
          break;           
        case 0x17:
            PAS_Out = PAS_LEVEL_4;
          break;           
        case 0x03:  
            PAS_Out = PAS_LEVEL_5;
          break;
        case 0x6:
            PAS_Out = PAS_LEVEL_WALK;
          break;
        default:
            PAS_Out = PAS_LEVEL_0;
          break;      
   }

   return PAS_Out;
}

/** 
 *  Function used to enable Eco mode and change the max current fo the vehicle      
 */
void LCD_KD718_EnableEcoMode(KD718_Handle_t *pHandle)
{    
    ASSERT(pHandle != NULL);
    
    if (!pHandle->EcoMode) // Make sure ecomode isn't already enabled 
    {   
        uint16_t VehicleMaxCurrent;
        uint16_t VehicleMaxSafeCurrent;
        
        pHandle->EcoMode = true; // Update the variable state

        VehicleMaxSafeCurrent = PWRT_GetMaxSafeCurrent(pHandle->pVController->pPowertrain); // Get the maximum safe current
        
        VehicleMaxCurrent = (VehicleMaxSafeCurrent * pHandle->EcoModePowerPerCent) / 100; // Apply the Eco mode reduction
                
        PWRT_SetOngoingMaxCurrent(pHandle->pVController->pPowertrain,VehicleMaxCurrent); // Apply the new max ongoing current
    }
}

/**  
 *  Function used to disable Eco mode and change the max current fo the vehicle   
 */
void LCD_KD718_DisableEcoMode(KD718_Handle_t *pHandle)
{
    ASSERT(pHandle != NULL);
    
    if (pHandle->EcoMode) // Make sure ecomode is currently enabled
    {
        uint16_t VehicleMaxSafeCurrent;
        
        pHandle->EcoMode = false; // Update the variable state
        
        VehicleMaxSafeCurrent = PWRT_GetMaxSafeCurrent(pHandle->pVController->pPowertrain);  // Get the maximum safe current
        
        PWRT_SetOngoingMaxCurrent(pHandle->pVController->pPowertrain,VehicleMaxSafeCurrent); // Apply the new max ongoing current   
    }
}

/**
 * Function used to convert from standard FTEX error codes to KD718 error codes     
 */
uint8_t LCD_KD718_ErrorConversionFTEXToKD718(uint8_t aError)
{
    uint8_t ConvertedError;
    switch(aError)
    {
        case NO_ERROR:
            ConvertedError = KD718_NORMAL_STATUS; 
          break;
        case THROTTLE_STUCK:
            ConvertedError = KD718_THROTTLE_STUCK;
          break;  
        case UV_PROTECTION:
        case BATT_LOW:    
            ConvertedError = KD718_LOW_VOLTAGE;
          break;
        case OV_PROTECTION:
            ConvertedError = KD718_HIGH_VOLTAGE;
          break;
        case MOTOR_HALL_ERROR:
            ConvertedError = KD718_HALL_SIGNAL_FAULT;
          break;
        case MOTOR_PHASE_ERROR:
            ConvertedError = KD718_MOTOR_PHASE_FAULT;
          break;        
        case CONTROLLER_OT_PROTECT:
				case MOTOR_FOLDBACK_TEMP: // Generic temp error
            ConvertedError = KD718_OVER_TEMP;
            break;
        case IOT_COMM_ERROR: // Generic comm error
        case DUAL_COMM_ERROR:    
            ConvertedError = KD718_HDQ_COMM_FAYLT;
            break;
        case UNMAPPED_ERROR: // KD718 error that we con't currently support
            ConvertedError = KD718_THROTTLE_FAULT;
            ConvertedError = KD718_BRAKE_ENGAGED;
            ConvertedError = KD718_TEMP_SENSOR_FAULT;
            ConvertedError = KD718_CURRENT_SENSOR_FAULT;
            ConvertedError = KD718_SPEED_SENSOR_FAULT;
            ConvertedError = KD718_HEADLIGHT_FAULT;
            ConvertedError = KD718_HEADLIGHT_SENSOR_FAULT;
            break;
        default: // 
            ConvertedError = KD718_NORMAL_STATUS; 
          break; 
    }
    
    return ConvertedError;
}
