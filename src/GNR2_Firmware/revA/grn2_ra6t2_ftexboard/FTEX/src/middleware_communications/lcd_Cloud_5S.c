/**
  ******************************************************************************
  * @file    lcd_Cloud_5S.c
  * @author  Andy Beaudoin, FTEX
  * @brief   High level module that describes APT LCD communication protocol
  *
  ******************************************************************************
*/
    
#include "lcd_Cloud_5S.h"
#include "ASSERT_FTEX.h"
#include "wheel.h"
#include "gnr_main.h"
#include "vc_errors_management.h"
#include "vc_constants.h"
#include "vc_parameters.h"
#include "can_vehicle_interface.h"

extern osThreadId_t COMM_Uart_handle; // Task Id for UART

const uint8_t Cloud_HandshakeRef[64] =   // This array is iused as a handshake,
    {137,159,134,249, 88, 11,250, 61,    // the screen will ask us the content of an element
      33,150,  3,193,118,141,209, 94,    // and we must answer with the right corresponding value
     226, 68,146,158,145,127,216, 62,    // IE screen sends us 52, we need to answer with 36
     116,230,101,211,251, 54,229,247,
      20,222, 59, 63, 35,252,142,238,
      23,197, 84, 77,147,173,210, 57,
     142,223,157, 97, 36,160,229,237,
      75, 80, 37,113,154, 88, 23,120};

const uint8_t Cloud_5S_WheelDiameterInch[20] =  // [14] sohuld represent 700c wheel but we aprox to 27 inch    
   {  5,  6,  7,  8, 10, 12, 14, 16, 18, 20,   
     22, 24, 26, 27, 27, 28, 29, 30, 31, 32};  

/**
 *  Function for initializing the LCD module with the Cloud 5S protocol
 *  It links its own custom interrupt routines with the UART callbacks using
 *  a void pointer. 
 */
void LCD_Cloud_5S_init(Cloud_5S_Handle_t *pHandle,VCI_Handle_t *pVCIHandle, UART_Handle_t *pUARTHandle)
{
    ASSERT(pHandle     != NULL);
    ASSERT(pVCIHandle  != NULL);
    ASSERT(pUARTHandle != NULL);
    pHandle->isScreenSlave = true;  // Start as a slave to force pas to start at default pas on boot.
    pHandle->pasController = 0;
    
    pHandle->pVController = pVCIHandle;        // Pointer to VController
    pHandle->pUART_handle = pUARTHandle;       // Pointer to UART instance  
    pHandle->pUART_handle->Super = pHandle;    // Initialise the super pointer that the UART needs   
    
    pHandle->pUART_handle->pRxCallback = &LCD_Cloud_5S_RX_IRQ_Handler;   // Link the interrupts from the UART instance to this module
    pHandle->pUART_handle->pTxCallback = &LCD_Cloud_5S_TX_IRQ_Handler; 
            
    Throttle_SetupExternal(pHandle->pVController->pPowertrain->pThrottle,CLOUD_X_THROTTLE_MAX,CLOUD_X_THROTTLE_OFFSET);
    
    VC_Errors_SetCycleLength(8); // To adjust
    
    pHandle->pUART_handle->UARTBaudrate = BAUD9600;
    uCAL_UART_Init(pHandle->pUART_handle);  // Initialise the UART module with the baudrate that APT screen needs  
    uCAL_UART_Receive(pHandle->pUART_handle, &(pHandle->RxByte), sizeof(pHandle->RxByte));  // Start the first reception   
}

/** 
 *  Function for building a frame specific to the Cloud 5S protocol
 *  It is interrupt driven until a frame is completed where it  
 *  unblocks a comm task to process the frame.
 *  It is based on a byte by byte reception 
 */
void LCD_Cloud_5S_RX_IRQ_Handler(void *pVoidHandle)
{     
    ASSERT(pVoidHandle != NULL);    
    Cloud_5S_Handle_t *pHandle = pVoidHandle;  // Convert the void handle pointer to a handle pointer
    
    if (pHandle->RxCount >= CLOUD_RX_BUFFER_SIZE) //Overflow safety
    {
       pHandle->RxCount = 0;
    }
        
    pHandle->RxBuffer[pHandle->RxCount] = pHandle->RxByte;
    pHandle->RxCount++;
    
    uCAL_UART_SetTaskFlag(pHandle->pUART_handle);  // Tell the task that we unblocked the task
    osThreadFlagsSet(COMM_Uart_handle, UART_FLAG); // Notify task that a byte has been received 
    uCAL_UART_Receive(pHandle->pUART_handle, &(pHandle->RxByte), sizeof(pHandle->RxByte));    
}

/**
 *  Function for sending a frame specific to the Cloud 5S protocol
 *  once a frame has been received and processed this function
 *  sends the response made byt the frame process function.  
 *  It is based on a byte by byte transmission.   
 */
void LCD_Cloud_5S_TX_IRQ_Handler(void *pVoidHandle)
{
    ASSERT(pVoidHandle != NULL);    
    Cloud_5S_Handle_t *pHandle = pVoidHandle;  
   
    uint8_t tx_data;

    if (pHandle->tx_frame.ByteCnt < pHandle->tx_frame.Size) // Do we hav bytes to send ?
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
 *  Task for the Cloud 5S screen
 *
 *
 */
void LCD_Cloud_5S_Task(Cloud_5S_Handle_t *pHandle)
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
        
        if (ByteCount == 0) // Each case represents the number of the byte received in a frame.                    
        {
             
             if (NextByte == CLOUD_START) // Is it the proper start byte ?
             {
                 pHandle->rx_frame.Buffer[ByteCount] = NextByte;
                 pHandle->rx_frame.ByteCnt ++;                 
             }
             else // If not, its a bad cmd
             {
                 pHandle->rx_frame.ByteCnt = 0;
                 uCAL_UART_SendDefault(pHandle->pUART_handle);
             }           
        }
        else if (ByteCount == 1)
        {
             if (NextByte == CLOUD_SLAVE_ID) // Is it the proper address ?
             {
                 pHandle->rx_frame.Buffer[ByteCount] = NextByte;
                 pHandle->rx_frame.ByteCnt ++;                 
             }
             else // If not, its a bad cmd
             {
                 pHandle->rx_frame.ByteCnt = 0;
                 uCAL_UART_SendDefault(pHandle->pUART_handle);
             }
        }
        else if (ByteCount == 2)
        {
             if ((NextByte == CLOUD_RUNTIME) || (NextByte == CLOUD_SYSTEM)) // is it a valid command
             {
                 if (NextByte == CLOUD_RUNTIME)
                 {
                     pHandle->rx_frame.Size = CLOUD_RUNTIME_FRAME_LENGTH;    
                 }
                 else
                 {
                     pHandle->rx_frame.Size = CLOUD_SYSTEM_FRAME_LENGTH;   
                 }
                 
                 pHandle->rx_frame.Buffer[ByteCount] = NextByte;
                 pHandle->rx_frame.ByteCnt ++;   
             }
             else
             {
                pHandle->rx_frame.ByteCnt = 0;
                uCAL_UART_SendDefault(pHandle->pUART_handle);
             }    
 
        }
        else if ((ByteCount > 2) && (ByteCount < (pHandle->rx_frame.Size - 1)))
        {   
            // From byte 2 to the seconde to last byte
            // They dont need to be checked because as long as 
            // the checksum is good then they are valid
            // The checksum is verified in the frame process function
                 
            pHandle->rx_frame.Buffer[ByteCount] = NextByte;
            pHandle->rx_frame.ByteCnt ++;
                
        }
        else if (ByteCount == (pHandle->rx_frame.Size - 1)) // 
        {
                                        
            if (NextByte == CLOUD_END_CODE_2) // Is it the proper end byte ?
            {
                pHandle->rx_frame.Buffer[ByteCount] = NextByte;
                
                LCD_Cloud_5S_ProcessFrame(pHandle); //We have received a full frame so do the frame processing
            }
            
            pHandle->rx_frame.ByteCnt = 0;
        }
        else
        {            
            // We should never get here but if we do trash the frame
            pHandle->rx_frame.ByteCnt = 0;
            uCAL_UART_SendDefault(pHandle->pUART_handle);            
        }
    }        
}

/**
 *  Function for decoding a received frame (previously built in the RxCallback function)
 *  according to the Cloud 5S screen protocol.
 *  This is executed in a comm task that gets unblocked when a complete frame is received.
 *
 */ 

void LCD_Cloud_5S_ProcessFrame(Cloud_5S_Handle_t * pHandle)
{
    Cloud_5S_frame_t replyFrame = {0};
    replyFrame.Size = 14;
    
    int32_t  toSend         = 0;
    uint8_t  WheelDiameter  = 0;    
    uint8_t  pasLevel;
    uint8_t  LightStatus    = 0;
    uint8_t  HandshakeIndex = 0; 
    uint8_t  Check1         = 0;
    uint8_t  Check2         = 0; 
    uint8_t  AssistType; 
    uint8_t  CruiseCtrlState;
    
    PWRT_Handle_t *pPowertrainHandle = (pHandle->pVController->pPowertrain);
    
    ThrottleHandle_t *pThrottleHandle = (pHandle->pVController->pPowertrain->pThrottle);
    
    PAS_Handle_t *pPASHandle = (pHandle->pVController->pPowertrain->pPAS);
    
    
    LCD_Cloud_5S_ComputeChecksum(pHandle->rx_frame,&Check1,&Check2);
    
    
    //Check if the CRC is good
    if ((Check1 == (pHandle->rx_frame.Buffer[pHandle->rx_frame.Size - 4])) && 
       (Check2 == (pHandle->rx_frame.Buffer[pHandle->rx_frame.Size - 3])))
    {
        if (pThrottleHandle->extThrottleEnable == false)
        {
            Throttle_SetupExternal(pThrottleHandle,CLOUD_X_THROTTLE_MAX,CLOUD_X_THROTTLE_OFFSET);
        }
        
        if (pHandle->rx_frame.Buffer[2] == CLOUD_SYSTEM) // Check if its a system frame
        {
            //pHandle->rx_frame.Buffer[3] // data length
            AssistType = pHandle->rx_frame.Buffer[4]; /* assistance sensor bit7: Assistance signal type selection, 0-Cadence, 1-Torque
                                                                           bit6: 0-Assistance signal is positive, 1-Assistance signal is negative
                                                                           bit5-0: (Assistance signal will be effective after passing through how many magnets), 0~31, usually
                                                                                   is no less than 2.*/ 

            AssistType = AssistType & CLOUD_ASSIST_TYPE; // Extract only the assistance type from the byte
            
            if(AssistType == CLOUD_TORQ_ASSIST_TYPE) // Check if we want torque or cadence PAS and apply the change to the module
            {
                CanVehiInterface_SetAlgorithm(pHandle->pVController, TorqueSensorUse);
            }
            else if (AssistType == CLOUD_CADE_ASSIST_TYPE)
            {
                CanVehiInterface_SetAlgorithm(pHandle->pVController, CadenceSensorUse);
            }
            else
            {
                ASSERT(false); // unexpected AssistType
            }        

            //pHandle->rx_frame.Buffer[5] // assitance sensor type
            //pHandle->rx_frame.Buffer[6] // Magnets/ low speed behavior
            //pHandle->rx_frame.Buffer[7] // System undervoltage value
            //pHandle->rx_frame.Buffer[8] // Max PAS / Max Volts
            
            HandshakeIndex = pHandle->rx_frame.Buffer[9]; // handshake byte     
            
        #if DYNAMIC_SPEED_LIMITATION
            uint8_t  speedLimit    = 0;
            //Reading the Speed limit
            speedLimit = pHandle->rx_frame.Buffer[10]; // speed limit       
            
            // setting the max RPMs for any speed limits
            PWRT_SetScreenMaxSpeed(pHandle->pVController->pPowertrain,speedLimit);        
        #endif    
           
        #ifdef SCREENPOWERCONTROL        
            // Reading the Current limit          
            uint16_t CurrentLimit;
            CurrentLimit = pHandle->rx_frame.Buffer[11];  // current limit
            
            CurrentLimit /=2; // Transform from 0.5 amps perunit to 1 amps per unit 
            
            PWRT_SetOngoingMaxCurrent(pHandle->pVController->pPowertrain, CurrentLimit);        
        #endif
           
            WheelDiameter = Cloud_5S_WheelDiameterInch[pHandle->rx_frame.Buffer[12]]; // wheel diameter 
                                                                                    
            
            Wheel_SetWheelDiameter(WheelDiameter);
            
            replyFrame.Buffer[0] = CLOUD_START;
            replyFrame.Buffer[1] = CLOUD_SLAVE_ID;
            replyFrame.Buffer[2] = CLOUD_SYSTEM;
            replyFrame.Buffer[3] = 6; // Data length of this frame
            replyFrame.Buffer[4] = 0;
            replyFrame.Buffer[5] = 0;
            replyFrame.Buffer[6] = 0;
            replyFrame.Buffer[7] = Cloud_HandshakeRef[HandshakeIndex];
            replyFrame.Buffer[8] = 0;
            replyFrame.Buffer[9] = 0;
            
            LCD_Cloud_5S_ComputeChecksum(replyFrame,&Check1,&Check2); 
           
            replyFrame.Buffer[10] = Check1;
            replyFrame.Buffer[11] = Check2;
           
            replyFrame.Buffer[12] = CLOUD_END_CODE_1;
            replyFrame.Buffer[13] = CLOUD_END_CODE_2;          
                       
        }
        else //If its not a system frame it must be a runtime frame
        {
            
            uint8_t PasMaxLevel = pPASHandle->sParameters.bMaxLevel;   
            
            pasLevel = LCD_Cloud_5S_ConvertPASLevelFromCloud_5S(pHandle->rx_frame.Buffer[4],PasMaxLevel); // PAS level
            uint8_t currentPAS = PedalAssist_GetAssistLevel(pPASHandle);
            // Screen is set to slave when there's a new pas level from the app
            // Therefore we skip setting the Cloud Drive PAS to our PAS because 
            // we will be updating the Cloud Drive right after parsing this message
            // Otherwise check if the pas has changed to update the app and set the new pas
            // As long as the app pas has been confirmed.
            if(pHandle->isScreenSlave && (currentPAS == pHandle->pasController))
            {
                pHandle->isScreenSlave = false;
            }

            if (!pHandle->isScreenSlave)
            {
                if (pasLevel != currentPAS)
                {
                    PedalAssist_SetAssistLevel(pPASHandle,pasLevel); 
                    pHandle->cloud5SChangePasFlag = true;                    
                }
            }
            
            LightStatus = pHandle->rx_frame.Buffer[5]; 
            
            LightStatus = (LightStatus & 0x80); // bit7: headlight. 0: light off, 1: light on
            
            if (LightStatus > 0) //Operate the light according to what the screen tells us
            {
                Light_Enable(pHandle->pVController->pPowertrain->pHeadLight);  
                Light_Enable(pHandle->pVController->pPowertrain->pTailLight); 
            }
            else
            {
                Light_Disable(pHandle->pVController->pPowertrain->pHeadLight); 
                Light_Disable(pHandle->pVController->pPowertrain->pTailLight);              
            }
            
                          
            CruiseCtrlState = (((pHandle->rx_frame.Buffer[5]) & 0x40) >> 5); // bit6: Cruise, 0 = Off 1 = On
        
            if((CruiseCtrlState > 0) && (PWRT_GetForceDisengageState(pPowertrainHandle) == false))
            {
                uint8_t currentSpeed = (uint8_t)Wheel_GetSpeedFromWheelRpm(WSSGetSpeedRPM());
                
                //check if the throttle max speed is inside of the max speed limit
                //if not used MaxThrottleSpeedKMH as cruise control speed.
                if (currentSpeed > pHandle->pVController->pPowertrain->pThrottle->hParameters.MaxThrottleSpeedKMH)
                {
                    currentSpeed = (uint8_t)pHandle->pVController->pPowertrain->pThrottle->hParameters.MaxThrottleSpeedKMH;
                }
                
                PWRT_EngageCruiseControl(pPowertrainHandle,currentSpeed);
            }    
            else
            {
                PWRT_DisengageCruiseControl(pPowertrainHandle);
                if(CruiseCtrlState == 0)
                {    
                    PWRT_ClearForceDisengage(pPowertrainHandle); 
                }                    
            }    
            
            if (((pHandle->rx_frame.Buffer[5]) & 0x10) > 0)  //  bit4: walk function. 0: no, 1: yes 
            {
                PedalAssist_SetAssistLevel(pPASHandle, PAS_LEVEL_WALK);            
            }   
            
            uint8_t LatestThrottle = 0;                
                                         /* 
                                            bit5: battery capacity. 0: normal, 1: low
                                            bit3: reserved
                                            bit2: reserved
                                            bit1: reserved
                                            bit0: over speed. 0: normal speed, 1: over speed */
            LatestThrottle = pHandle->rx_frame.Buffer[6]; 
            Throttle_UpdateExternal(pThrottleHandle,LatestThrottle);
            
                                         /* Throttle value 0V-5V corresponds to 255 data, normal voltage range is 0.8V-4.1V, 
                                            the throttle¡¯s default working voltage is 1.3V-3.5V. Send 51 when voltage is 0.8-1.3V, 
                                            send data 76-190 when voltage is 1.3-3.5V, 
                                            send maximum data 190 when voltage is 3.5-4.1,
                                            send 51 and throttle error together when voltage is more than 4.1V */
           
            replyFrame.Buffer[0] = CLOUD_START;
            replyFrame.Buffer[1] = CLOUD_SLAVE_ID;
            replyFrame.Buffer[2] = CLOUD_RUNTIME;
            replyFrame.Buffer[3] = 6; // Data length of this frame
            uint8_t controllerPas = PedalAssist_GetAssistLevel(pPASHandle);
            replyFrame.Buffer[4] = LCD_Cloud_5S_ConvertPASLevelToCloud_5S(controllerPas); // PAS 
            if (pHandle->isScreenSlave == true)
            {
                pHandle->pasController = controllerPas;
            }

            toSend = PWRT_GetDCCurrent(pHandle->pVController->pPowertrain);             
            toSend = LCD_Cloud_5S_ApplyPowerFilter((uint16_t)toSend);  
          
            toSend = toSend * 2; //Covert from amps to 0.5 amps; 
        
            replyFrame.Buffer[5] = (toSend & 0x000000FF); //Power 0.5 A/unit maxed out at 80 = 0x50 (40 amps)
           
            /* Condition use for wheel speed sensor rpm to send */
            toSend = WSSGetSpeedRPM(); // Getting RPM from Wheel Speed Module
                  
            toSend = toSend * 500; //Converion from RPM to period in ms, 500 is used as scaling to conserve precision          
         
            if ((toSend != 0) && ((toSend/60) != 0)) //verify toSend value to avoid a division by zero.
            {
                toSend = 500000/(toSend/60); // Descaling here with the 500000 to return to original unit            
            }
            
            if (toSend <= 80) // if the wheel isn't spinning or we get an unrealistic speed (0ver 100 km/h)
            {
                toSend = CLOUD_DEFAULT_SPEED_PERIOD; // Send the default value
            }
                        
            replyFrame.Buffer[6] = (toSend & 0xFF00) >> 8;    // Wheel periode high byte
            replyFrame.Buffer[7] = (toSend & 0x00FF);         // Wheel periode low byte
             
           
            replyFrame.Buffer[8] = LCD_Cloud_5S_ErrorConversionFTEXToCloud_5S(VC_Errors_CycleError()); // Error code
            
            if((PWRT_GetCruiseControlState(pPowertrainHandle) == true)   // If cruise control is still active and the                 
            && (PWRT_GetForceDisengageState(pPowertrainHandle) == false))  // controller doesn't want to for a disengage
            {
                replyFrame.Buffer[9] = CLOUD_CC_ON; //Cruise control status set to on 
            }
            else
            {
                replyFrame.Buffer[9] = CLOUD_CC_OFF; //Cruise control status set to off
            }
            
            
            LCD_Cloud_5S_ComputeChecksum(replyFrame,&Check1,&Check2); 
           
            replyFrame.Buffer[10] = Check1;
            replyFrame.Buffer[11] = Check2;
           
            replyFrame.Buffer[12] = CLOUD_END_CODE_1;
            replyFrame.Buffer[13] = CLOUD_END_CODE_2;                                
        }

        replyFrame.ByteCnt = 0;   
        
        pHandle->tx_frame = replyFrame; 
             
        LCD_Cloud_5S_TX_IRQ_Handler((void *) pHandle); // Start the transmission of the answer frame        

    }// End of CRC check      
}    

/**  
 *  Function used to apply a filter on the power we send to the screen 
 */
uint16_t LCD_Cloud_5S_ApplyPowerFilter(uint16_t aInstantPowerInAmps)
{
   static uint16_t PowerAvg; 
   uint16_t BandwidthUp   = 1; //Bandwidth CANNOT be set to 0
   uint16_t BandwidthDown = 5;   

   uint16_t Bandwidth = 0;

   if (aInstantPowerInAmps >= PowerAvg)
   {
       Bandwidth = BandwidthUp;
   }
   else
   {
       Bandwidth = BandwidthDown;
   }       
    
   PowerAvg = (((Bandwidth-1) * PowerAvg + aInstantPowerInAmps)/Bandwidth);
   
   return (PowerAvg); 
} 

/** Function used to translate the PAS level received from the Cloud 5S  
 *  screen standard to the FTEX standard
 */
uint8_t LCD_Cloud_5S_ConvertPASLevelToCloud_5S(PasLevel_t aPAS_Level)
{
    uint8_t PAS_Out = PAS_LEVEL_0; 
    switch(aPAS_Level)
    {
       case PAS_LEVEL_0:
           PAS_Out = 0;
           break;
       case PAS_LEVEL_1:
           PAS_Out = 51;
           break;
       case PAS_LEVEL_2:
           PAS_Out = 102;
           break;
       case PAS_LEVEL_3:
           PAS_Out = 153;
           break;
       case PAS_LEVEL_4:
           PAS_Out = 204;
           break;
       case PAS_LEVEL_5:
           PAS_Out = 255;
           break;
       default:  
           PAS_Out = 0;           
           break;       
    }
    return PAS_Out;
}

/** Function used to translate the FTEX standard PAS level to the Cloud 5S  
 *  screen standard.(is not the same as when we receive a PAS level from the Cloud 5S screen)
 */
PasLevel_t LCD_Cloud_5S_ConvertPASLevelFromCloud_5S(uint8_t aPAS_Level, uint8_t aNumberOfLevels)
{
    PasLevel_t PAS_Out = 0x0;
   
    
    if (aNumberOfLevels != 5)  // Check if we need to use the 5 PAS config
    {
        ASSERT(false); // Number of PAS levels not compatible with APT screen  
    }       
   
    switch(aPAS_Level)
    {
        case 0:
            PAS_Out = PAS_LEVEL_0;
           break;
        case 51:
            PAS_Out = PAS_LEVEL_1;
           break;
        case 102:
            PAS_Out = PAS_LEVEL_2;
           break;       
        case 153:
            PAS_Out = PAS_LEVEL_3;
           break;
        case 204:
            PAS_Out = PAS_LEVEL_4;
           break;           
        case 255:
            PAS_Out = PAS_LEVEL_5; 
           break;
        default:
            PAS_Out = PAS_LEVEL_0;
           break;     
    }

    return PAS_Out;
}

/**
 * Function used to convert from standard FTEX error codes to Cloud 5S error codes    
 */
uint8_t LCD_Cloud_5S_ErrorConversionFTEXToCloud_5S(uint8_t aError)
{    
    //Initialized to no error to avoid a warning here
    uint8_t ConvertedError = CLOUD_5S_NO_ERROR;
    switch(aError)
    {
        case NO_ERROR:
            ConvertedError = CLOUD_5S_NO_ERROR; 
            break;
        // TODO : Split those error when we have a screen protocol redifinition
        case PAS_BOOT_ERROR:
        case TORQUE_SENSOR_ERROR:
            ConvertedError = CLOUD_5S_PAS_BOOT_ERROR; 
            break;
        case CONTROLLER_ERROR:
            ConvertedError = CLOUD_5S_CNTL_ERROR; 
            break;
        case MOTOR_PHASE_ERROR:
            ConvertedError = CLOUD_5S_PHASE_ERROR; 
            break;
        case UV_PROTECTION:
            ConvertedError = CLOUD_5S_UVP;
            break;
        case BRAKE_ERROR:
            ConvertedError = CLOUD_5S_BRAKE_ERROR;
            break;
        case MOTOR_HALL_ERROR:
            ConvertedError = CLOUD_5S_HALL_ERROR;
            break;
        case THROTTLE_STUCK:
            ConvertedError = CLOUD_5S_THROTTLE_ERROR;
            break;            
        case UT_PROTECTION:
            ConvertedError = CLOUD_5S_UT_ERROR;
            break;            
        case CONTROLLER_OT_PROTECT:
        case CONTROLLER_FOLDBACK_TEMP:
            ConvertedError = CLOUD_5S_OT_ERROR;
            break;            
        case IOT_COMM_ERROR:
            ConvertedError = CLOUD_5S_IOT_COMM_ERROR;
            break;   
        case MOTOR_OT_PROTECT:
        case MOTOR_FOLDBACK_TEMP:
            ConvertedError = CLOUD_5S_MOT_ERROR;
            break;            
        case OV_PROTECTION:
            ConvertedError = CLOUD_5S_OV_ERROR;
            break;            
        case BATT_LOW:
            ConvertedError = CLOUD_5S_LOW_BAT;
            break;            
        case MOTOR_NTC_DISC_FREEZE:
            ConvertedError = CLOUD_5S_TEMP_DISC;
            break;
        case OVER_CURRENT:
            ConvertedError = CLOUD_5S_CNTL_ERROR;
            
        case UNMAPPED_ERROR: // Errors that Cloud 5S has but that we currently don't flag
            break;        
        default: // Cloud drive doesn't supports sending other error codes
            ConvertedError = CLOUD_5S_NO_ERROR;         
            break;
    }

    return ConvertedError;
}


/**
 * Function used to compute the checksum values to verify frames    
 */

void LCD_Cloud_5S_ComputeChecksum(Cloud_5S_frame_t aFrame, uint8_t *pCheck1, uint8_t *pCheck2)
{
    uint8_t FrameSize = aFrame.Size;
    uint16_t CheckCalc = 0;
    //Verification of the checksum
    for(int i = 1; i < FrameSize - CLOUD_POST_CRC_BYTE_NB; i ++) //Checksum is the sum of all bytes except for: start code, end code and checksum bytes
    {                              
         CheckCalc += aFrame.Buffer[i];
    }
    
    *pCheck1 = (uint8_t) (CheckCalc%256); // These two values are used for checksum verification
    *pCheck2 = (uint8_t) (CheckCalc/256);
}
