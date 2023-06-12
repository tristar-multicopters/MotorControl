/**
  * @file    vc_tasks.c
  * @brief   This module gather all tasks of vehicule control application
  *
    */

#include "vc_config.h"
#include "comm_config.h"
#include "vc_tasks.h"
#include "mc_tasks.h"
#include "comm_tasks.h"

#include "mc_interface.h"

#include "gnr_main.h"

//#include "slave_mc_interface.h"


/************* DEBUG ****************/

/* This structure is used in swd interface to manually control motors */
#if SWD_CONTROL_ENABLE
struct {
    qd_t hIqdRef;
    int16_t hTorqRef;
    int32_t hSpeedRef;
    bool bStartM1;
    bool bStartM2;
    bool FaultAck;
} sDebugVariables;
#endif


/************* DEFINES ****************/

#define DELAY_AFTER_BOOTUP                   500    /* 500 RTOS ticks delay to prevent starting motors just after system bootup */
#define TASK_VCFASTLOOP_SAMPLE_TIME_TICK     10     /* VC_FastLoop execute every 10 ticks */
#define TASK_VCSTM_SAMPLE_TIME_TICK          10     /* VC_StateMachine execute every 10 ticks */
#define TASK_VCSLOWLOOP_SAMPLE_TIME_TICK     50     /* VC_SlowLoop execute every (50*10*0.5 ticks)= 250ms called in the MedFreq Task*/
#define RETURN_TO_STANDBY_LOOPTICKS          0      // Max number of ticks to stay in run while stop conditions are met
#define START_MOTORS_LOOPTICKS               2      // Max number of ticks to stay in standby while start conditions are met
#define START_LOOPTICKS                      500    // Max number of ticks to stay in start state
#define STOP_LOOPTICKS                       500    // Max number of ticks to stay in stop state

/************* VARIABLES ****************/
uint16_t TASK_VCSLOWLOOP_SAMPLE_LOOP_COUNT = 0;

/************* TASKS ****************/

/**
  * @brief  It initializes the vehicle control application. Needs to be called before using
    *                    vehicle control related modules.
  * @retval None
  */
void VC_BootUp(void)
{    
    VCI_Handle_t * pVCI = &VCInterfaceHandle;
    
    Delay_Handle_t DelayArray[2];
    
    Delay_Init(&DelayArray[THROTTLE_DELAY],TASK_VCFASTLOOP_SAMPLE_TIME_TICK * 500,MIC_SEC); // Initialising the time base for the throttle stuck delay
    Delay_Init(&DelayArray[PTS_DELAY],TASK_VCFASTLOOP_SAMPLE_TIME_TICK * 500,MIC_SEC); // // Initialize the time base for the Pedal Torque sensor stuck delay
    
    /* Initialize vehicle controller state machine and powertrain components */
    VCSTM_Init(pVCI->pStateMachine);
    PWRT_Init(pVCI->pPowertrain, &MCInterface[M1], &SlaveM2, DelayArray);
}

/**
  * @brief  Task to periodically update vehicle control components
  * @retval None
  */
__NO_RETURN void THR_VC_MediumFreq (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    
    VCI_Handle_t * pVCI = &VCInterfaceHandle;
    
    uint32_t xLastWakeTime = osKernelGetTickCount();
    
    if (pVCI->pPowertrain->pPWREN->bInitialPowerLockState == true) // If we have bene powered on by the screen
    {
        Light_PowerOnSequence(pVCI->pPowertrain->pHeadLight); // Setup the lights with their default values
        Light_PowerOnSequence(pVCI->pPowertrain->pTailLight);
    }
    
    while (true)
    {
        PWRT_UpdatePowertrainPeripherals(pVCI->pPowertrain);
        PWRT_CalcMotorTorqueSpeed(pVCI->pPowertrain);
        
        // VC_SlowLoop execute in the MediumFreq loop
        TASK_VCSLOWLOOP_SAMPLE_LOOP_COUNT++;
        if (TASK_VCSLOWLOOP_SAMPLE_LOOP_COUNT > TASK_VCSLOWLOOP_SAMPLE_TIME_TICK)
        {
            // Check PAS activation based on torque or cadence
            PedalAssist_UpdatePASDetectionCall(pVCI->pPowertrain->pPAS);
            // Pedal Assist Cadence reading period
            PedalSpdSensor_CalculateSpeed(pVCI->pPowertrain->pPAS->pPSS);
            // Wheel Speed sensor reading period
            WheelSpdSensor_CalculatePeriodValue(pVCI->pPowertrain->pPAS->pWSS);

            // Check if we still have power enabled
            PWREN_MonitorPowerEnable(pVCI->pPowertrain->pPWREN);          
            
            // Update the SOC voltage reference
            BatMonitor_UpdateSOC(pVCI->pPowertrain->pBatMonitorHandle);
            
            if (BRK_IsPressed(pVCI->pPowertrain->pBrake))  //Blink the tail light when we brake
            {
                Light_SetBlink(pVCI->pPowertrain->pTailLight,true);  
            }
            else
            {
                Light_SetBlink(pVCI->pPowertrain->pTailLight,false);  
            }    
            
            //reset the count loop           
            TASK_VCSLOWLOOP_SAMPLE_LOOP_COUNT = NULL;
        }
        
        // Update Light if Blinking
        Light_Blink(pVCI->pPowertrain->pTailLight);
        
        
        #if ENABLE_VC_DAC_DEBUGGING
        R_DAC_Write((DEBUG1_DAC_HANDLE_ADDRESS)->p_ctrl, pVCI->pPowertrain->pThrottle->hInstADCValue);
        R_DAC_Write((DEBUG2_DAC_HANDLE_ADDRESS)->p_ctrl, pVCI->pPowertrain->pThrottle->hAvADCValue);
        #endif
        xLastWakeTime += TASK_VCFASTLOOP_SAMPLE_TIME_TICK;
        osDelayUntil(xLastWakeTime);
    }
}

/* 
   Task to control vehicle depending on its current state.
   Flowchart describing how this state machine works available on the FTEX drive
   https://drive.google.com/file/d/1PoqcsODmJ9KeqMJPuf6NUA2KDo6AWH9R/view?usp=sharing
*/
__NO_RETURN void THR_VC_StateMachine (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    
    VCI_Handle_t * pVCI = &VCInterfaceHandle;
    
    #if SWD_CONTROL_ENABLE
    sDebugVariables.bStartM1 = false;
    sDebugVariables.bStartM2 = false;
    sDebugVariables.FaultAck = false;
    sDebugVariables.hTorqRef = 0;
    sDebugVariables.hIqdRef.q = 0;
    sDebugVariables.hIqdRef.d = 0;
    sDebugVariables.hSpeedRef = 0;
    #else
    uint32_t wCounter;
    uint16_t hVehicleFault;
    VC_State_t StateVC;
    #endif
    
    osDelay(DELAY_AFTER_BOOTUP); //Simple delay to prevent starting motors just after system bootup
    
    uint32_t xLastWakeTime = osKernelGetTickCount();
    while (true)
    {    
        #if SWD_CONTROL_ENABLE
        
        /* for directly input Torque */
        MDI_ExecTorqueRamp(pVCI->pPowertrain->pMDI, M1, sDebugVariables.hTorqRef);
        MDI_ExecTorqueRamp(pVCI->pPowertrain->pMDI, M2, sDebugVariables.hTorqRef);
        
        /* for directly inout Iq and Id */
        //MDI_SetCurrentReferences(pVCI->pPowertrain->pMDI, M1, sDebugVariables.hIqdRef);
        
        sDebugVariables.bStartM1 ? MDI_StartMotor(pVCI->pPowertrain->pMDI, M1) : MDI_StopMotor(pVCI->pPowertrain->pMDI, M1);
        sDebugVariables.bStartM2 ? MDI_StartMotor(pVCI->pPowertrain->pMDI, M2) : MDI_StopMotor(pVCI->pPowertrain->pMDI, M2);
        
        if (sDebugVariables.FaultAck)
        {
            sDebugVariables.FaultAck = false;
            MDI_FaultAcknowledged(pVCI->pPowertrain->pMDI, M1);
            MDI_FaultAcknowledged(pVCI->pPowertrain->pMDI, M2);
        }
        
        #else
        StateVC = VCSTM_GetState(pVCI->pStateMachine);
        switch (StateVC)
        {
            case V_IDLE:
            /* Vehicule is idle, does not do anything */
                    osDelay(100);
                    wCounter = 0;
                    VCSTM_NextState(pVCI->pStateMachine, V_STANDBY);
                    break;
            
            case V_STANDBY:
            /* Vehicule is in standby, waiting for a condition to start powertrain */
                    hVehicleFault = PWRT_StandbyStateCheck(pVCI->pPowertrain); // Check state of motors. Return error if they are not into the state they are supposed to be.
                    VCSTM_FaultProcessing(pVCI->pStateMachine, hVehicleFault, 0); // If motor state check fails, triggers vehicle fault.
                    if (PWRT_CheckStartConditions(pVCI->pPowertrain)) // If powertrain start conditions are met, increase counter.
                    {
                        wCounter++;
                    }
                    else
                    {
                        wCounter = 0;
                    }
                    if (wCounter > START_MOTORS_LOOPTICKS) // If counter reach target, start powertrain. 
                    {
                        wCounter = 0;
                        VCSTM_NextState(pVCI->pStateMachine, V_STANDBY_START);
                    }
                    break;
            
            case V_STANDBY_START:
            /* Temporary state in between standby and start */
                    wCounter = 0;
                    VCSTM_NextState(pVCI->pStateMachine, V_START);
                    break;
            
            case V_START:
            /* Vehicle is starting powertrain */
                    PWRT_ApplyMotorRamps(pVCI->pPowertrain); // Send target ramp (torque or speed) to motors
                    PWRT_StartMotors(pVCI->pPowertrain); // Send start command to motors
                    hVehicleFault = PWRT_StartStateCheck(pVCI->pPowertrain); // Check state of motors. Return error if they are not into the state they are supposed to be.
                    VCSTM_FaultProcessing(pVCI->pStateMachine, hVehicleFault, 0); // If motor state check fails, triggers vehicle fault.
                    if (PWRT_IsPowertrainActive(pVCI->pPowertrain)) // If powertrain is active, go to run state.
                    {
                        wCounter = 0;
                        VCSTM_NextState(pVCI->pStateMachine, V_RUN);
                    }
                    wCounter++;
                    if (wCounter > START_LOOPTICKS) // If counter reach target, it means start vehicle state takes too long and vehicle fault is triggered.
                    {
                        wCounter = 0;
                        VCSTM_FaultProcessing(pVCI->pStateMachine, VC_START_TIMEOUT, 0);
                    }
                    break;
                    
            case V_RUN:
            /* Vehicle is running, motors are active */
                    hVehicleFault = PWRT_RunStateCheck(pVCI->pPowertrain); // Check state of motors. Return error if they are not into the state they are supposed to be.
                    VCSTM_FaultProcessing(pVCI->pStateMachine, hVehicleFault, 0); // If motor state check fails, triggers vehicle fault.
                    PWRT_ApplyMotorRamps(pVCI->pPowertrain); // Update ramp command to motors            
                    if (PWRT_CheckStopConditions(pVCI->pPowertrain)) // If powertrain stop conditions are met, increase counter.
                    {
                        wCounter++;
                    }
                    else
                    {
                        wCounter = 0;
                    }

                    if (wCounter > RETURN_TO_STANDBY_LOOPTICKS) // If counter reach target, stop powertrain. 
                    {
                        wCounter = 0;
                        VCSTM_NextState(pVCI->pStateMachine, V_ANY_STOP);
                    }
                    break;
                    
            case V_ANY_STOP:
            /* Temporary state before stopping vehicle */
                    wCounter = 0;
                    VCSTM_NextState(pVCI->pStateMachine, V_STOP);                    
                    break;
                    
            case V_STOP:
            /* Vehicle is getting stopped */
                    PWRT_StopMotors(pVCI->pPowertrain); // Send command to stop powertrain
                    hVehicleFault = PWRT_StopStateCheck(pVCI->pPowertrain); // Check state of motors. Return error if they are not into the state they are supposed to be.
                    VCSTM_FaultProcessing(pVCI->pStateMachine, hVehicleFault, 0); // If motor state check fails, triggers vehicle fault.
                    if (PWRT_IsPowertrainStopped(pVCI->pPowertrain)) // If powertrain is stopped, go back to idle/standby state.
                    {
                        wCounter = 0;
                        VCSTM_NextState(pVCI->pStateMachine, V_IDLE);
                    }
                    wCounter++;
                    if (wCounter > STOP_LOOPTICKS) // If counter reach target, it means stop vehicle state takes too long and vehicle fault is triggered.
                    {
                        wCounter = 0;
                        VCSTM_FaultProcessing(pVCI->pStateMachine, VC_STOP_TIMEOUT, 0);
                    }
                    break;
                    
            case V_FAULT_NOW:
            /* A vehicle fault happened and not processed yet */
                    PWRT_StopMotors(pVCI->pPowertrain); // Stop powertrain when fault happens
                    if (PWRT_IsPowertrainStopped(pVCI->pPowertrain)) // If powertrain is stopped, do fault management strategy.
                    {
                        if (!PWRT_MotorFaultManagement(pVCI->pPowertrain)) // If motor fault management is successful, remove vehicle faults related to motors.
                        {
                            VCSTM_FaultProcessing(pVCI->pStateMachine, 0, VC_M1_FAULTS); // Remove VC_M1_FAULTS flag
                            VCSTM_FaultProcessing(pVCI->pStateMachine, 0, VC_M2_FAULTS); // Remove VC_M2_FAULTS flag
                            //try to clear VC faults.
                            VCFaultManagment_Processing(pVCI->pStateMachine, &CONodeGNR);
                        }
                    }
                    break;
                    
            case V_FAULT_OVER:
            /* Faults have been managed and vehicle is now in a safe state */
                    VCSTM_FaultAcknowledged(pVCI->pStateMachine); // Acknowledge vehicle faults, which means return to idle state. 
                    break;
            
            default:
                /* Raise a software error */
                VCSTM_FaultProcessing(pVCI->pStateMachine, VC_SW_ERROR, 0u);
            break;
        }
        #endif
    
        xLastWakeTime += TASK_VCSTM_SAMPLE_TIME_TICK;
        osDelayUntil(xLastWakeTime);
    }
}


/*
   Task takes care of the power down sequence of the controller. 
   Whatever the source of it may be, this module makes sure the motor is stopped
   before setting a pin to low to disable the power supply.
*/

__NO_RETURN void PowerOffSequence (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
    
    VCI_Handle_t * pVCI = &VCInterfaceHandle;
    
    while(true)
    {
        osThreadFlagsWait(POWEROFFSEQUENCE_FLAG, osFlagsWaitAny, osWaitForever); // Task is blocked until we have to power down        
        MDI_StopMotor(pVCI->pPowertrain->pMDI,M1);
        //if motor is not in idle state , wait.
        while(MCInterface_GetSTMState(pVCI->pPowertrain->pMDI->pMCI) != M_IDLE)
        {    
            osDelay(STOP_LOOPTICKS);
        }
        
        Light_PowerOffSequence(pVCI->pPowertrain->pHeadLight); // Make sure we turn off the lights before powering down
        Light_PowerOffSequence(pVCI->pPowertrain->pTailLight);
        
//is we have dual motor configuration, send a command to stop the slaver motor
//and wait until motor stops to run.
#if (SUPPORT_SLAVE_ON_IOT == 1 && GNR_IOT == 1)|| (GNR_MASTER == 1 && GNR_IOT == 0)
        MDI_StopMotor(pVCI->pPowertrain->pMDI,M2);
        while (MDI_GetSTMState(pVCI->pPowertrain->pMDI,M2) != M_IDLE)
        {
            osDelay(STOP_LOOPTICKS);
        }
        // TO DO
        // a sequence is needed to TURN OFF the slave. If not, the slave continues sending messages on CAN and prevents Master to turn off
        // reporrted bug on the Jira: https://tristarmulticopters.atlassian.net/browse/EGNR-2531
#endif
        //set the going off flag to indicate the system is going turn off.
        PWREN_SetGoingOffFlag(pVCI->pPowertrain->pPWREN);
        
        //while to wait until the turn off sequency to be finished.
        while(1)
        {
            //if system is ready to a normal power off sequency(stop IOT and turn off slave).
            //if they are present.
            if(PWREN_GetSystemReadyFlag(VCInterfaceHandle.pPowertrain->pPWREN) == true)
            {
                //Call the function responsible to manage the power off 
                PWREN_TurnOffSystem(&CONodeGNR, pVCI->pPowertrain->pPWREN);
            }
            else
            {
                //turn off himself 
                PWREN_StopPower(pVCI->pPowertrain->pPWREN);
            }
        }
    }
}