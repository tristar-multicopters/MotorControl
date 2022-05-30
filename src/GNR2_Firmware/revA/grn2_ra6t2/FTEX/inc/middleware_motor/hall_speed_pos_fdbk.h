/**
* @file    hall_speed_pos_fdbk.h
* @brief   This file provides firmware functions that implement the features of the Hall Speed & Position Feedback component.
*
*/

#ifndef __HALL_SPEEDNPOSFDBK_H
#define __HALL_SPEEDNPOSFDBK_H

#include "speed_pos_fdbk.h"
#include "r_gpt.h"
#include "r_timer_api.h"

// ====================== HALL speed sensing defines ===================== //

#define HALL_SPEED_FIFO_SIZE  ((uint8_t)18)
#define DEGREES_120 0u
#define DEGREES_60 1u

// ============== Structure used to define HALL components =============== //

typedef struct
{
    SpeednPosFdbk_Handle_t _Super;  // Speed position feedback handle as sub part of HALL angle
    /* SW Settings */
    uint8_t  SensorPlacement;  // Define here the mechanical position of the sensors with reference to an electrical cycle. Allowed values are: DEGREES_120 or DEGREES_60.
    int16_t  PhaseShift;  // Define here in s16degree the electrical phase shift between the low to high transition of signal H1 and the maximum of the Bemf induced on phase A.
    uint16_t SpeedSamplingFreqHz;  // Frequency (Hz) at which motor speed is to be computed. It must be equal to the frequency at which function SPD_CalcAvrgMecSpeedUnit is called.
    uint8_t  SpeedBufferSize;  // Size of the buffer used to calculate the average speed. It must be less than 18.*/
    /* HW Settings */
    uint32_t TIMClockFreq;  // Timer clock frequency express in Hz.
    const timer_instance_t  * TIMx;  // Timer used for HALL sensor management.
    bsp_io_port_pin_t H1PortPin;
    bsp_io_port_pin_t H2PortPin;
    bsp_io_port_pin_t H3PortPin;
    bool SensorIsReliable;  // Flag to indicate a wrong configuration of the Hall sensor signanls.
    volatile uint8_t FirstCapt;  // Flag used to discard first capture for the speed measurement
    volatile uint8_t BufferFilled;  // Indicate the number of speed measuremt present in the buffer from the start. It will be max bSpeedBufferSize and it is used to validate the start of speed averaging.
    volatile uint16_t OVFCounter;  // Count overflows if prescaler is too low
    uint32_t SensorPeriod[HALL_SPEED_FIFO_SIZE];  // Holding the last period captures  
    uint8_t SpeedFIFOIdx;  // Pointer of next element to be stored in the speed sensor buffer
    uint32_t  ElPeriodSum;  // Period accumulator used to speed up the average speed computation
    uint32_t  MaxElSum;  // maximum sum of periods, used to check if timoout has occured or not  
    int16_t PrevRotorFreq;  // Used to store the last valid rotor electrical speed in dpp used when HALL_MAX_PSEUDO_SPEED is detected
    int8_t Direction;  // Instantaneous direction of rotor between two captures
    int16_t AvrElSpeedDpp;  // It is the averaged rotor electrical speed express in s16degree per current control period.
    uint8_t HallState;  // Current HALL state configuration
    int16_t DeltaAngle;  // Delta angle at the Hall sensor signal edge between current electrical rotor angle of synchronism. It is in s16degrees.
    int16_t MeasuredElAngle;  // This is the electrical angle  measured at each Hall sensor signal edge. It is considered the best measurement of electrical rotor angle.*/
    int16_t CompSpeed;  // Speed compensation factor used to syncronize the current electrical angle with the target electrical angle.
    uint16_t HALLMaxRatio;  // Max TIM prescaler ratio defining the lowest expected speed feedback.
    uint16_t SatSpeed;  // Returned value if the measured speed is above the maximum realistic.
    uint32_t PseudoFreqConv; // Conversion factor between time interval Delta T between HALL sensors captures, express in timer counts, and electrical rotor speed express in dpp. Ex. Rotor speed (dpp) = wPseudoFreqConv / Delta T It will be ((CKTIM / 6) / (SAMPLING_FREQ)) * 65536.*/
    uint32_t MaxPeriod;  /* Time delay between two sensor edges when the speed of the rotor is the minimum realistic in the
                         application: this allows to discriminate too low freq for instance. This period shoud be expressed in timer counts and it will be:
                         wMaxPeriod = ((10 * CKTIM) / 6) / MinElFreq(0.1Hz).*/
    uint32_t MinPeriod;
                        /* Time delay between two sensor edges when the speed of the rotor is the maximum realistic in the application: this allows discriminating glitches for instance.
                        This period shoud be expressed in timer counts and it will be: wSpeedOverflow = ((10 * CKTIM) / 6) / MaxElFreq(0.1Hz).*/
    uint16_t HallTimeout;  // Max delay between two Hall sensor signal to assert zero speed express in milliseconds.
    uint16_t OvfFreq;  // Frequency of timer overflow (from 0 to 0x10000) it will be: hOvfFreq = CKTIM /65536.
    uint16_t PWMNbrPSamplingFreq;  // Number of current control periods inside each speed control periods it will be: (hMeasurementFrequency / hSpeedSamplingFreqHz) - 1.
    uint8_t PWMFreqScaling;  // Scaling factor to allow to store a PWMFrequency greater than 16 bits 
    bool HallMtpa;  // if true at each sensor toggling, the true angle is set without ramp
    int16_t Sector_Start_Angle[7];  // Array that stores start confinement angle
    int16_t Sector_Destination_Angle[7];  // Array that stores destination confinement angle
    int16_t Sector_Middle_Angle[7];  // Array that stores middle angle for condition check
    int8_t DirectionChangeCounter;  // Variable that stores number of change in direction if one has cocured
    uint32_t Period;  // Variable thar stores timer period value. Should be integral multiples of 65536. lets say 8*65536 = 524287 is equivalent to prescaler set to 8.
} HALL_Handle_t;

// ====================  Interrupt service routine  prototypes ========================= //

/**
* @brief Interrupt service routine which needs to execute when there is timer over run. Here, overflow counter is incremented to add up total overruns when calculating capture value.   
* 
* @param Pointer to Hall speed position feedback handle  
* @return void 
*/
void * HALL_TIMx_UP_IRQHandler( void * pHandleVoid );

/**
* @brief Interrupt service routine which needs to execute when there is a capture compare event. Capture compare event occurs at every edge detection on any three hall speed sensor pins.   
* 
* @param Pointer to Hall speed position feedback handle and captured period register.
* @return void  
*/
void * HALL_TIMx_CC_IRQHandler( void * pHandleVoid, uint32_t* pCapture );

// ==================== Public function prototypes ========================= // 

/**
* @brief  It initializes the hardware peripherals (Timer, GPIO and NVIC)
*         required for the speed position sensor management using HALL sensors.
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
* @retval void
*/
void HALL_Init( HALL_Handle_t * pHandle );

/**
* @brief  Clear instantenous components in hall_speed_pos_fdbk
*         
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
* @retval void.
*/
void HALL_Clear( HALL_Handle_t * pHandle );

/**
* @brief  Update the rotor electrical angle integrating the last measured
*         instantaneous electrical speed express in dpp.
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
* @retval int16_t Measured electrical angle in s16degree format.
*/
int16_t HALL_CalcElAngle( HALL_Handle_t * pHandle );

/**
* @brief  This method must be called - at least - with the same periodicity
*         on which speed control is executed.
*         This method compute and store rotor istantaneous el speed (express
*         in dpp considering the measurement frequency) in order to provide it
*         to HALL_CalcElAngle function and SPD_GetElAngle.
*         Then compute rotor average el speed (express in dpp considering the
*         measurement frequency) based on the buffer filled by IRQ, then - as
*         a consequence - compute, store and return - through parameter
*         hMecSpeedUnit - the rotor average mech speed, expressed in Unit.
*         Then check, store and return the reliability state of
*         the sensor; in this function the reliability is measured with
*         reference to specific parameters of the derived
*         sensor (HALL) through internal variables managed by IRQ.
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
* @param  hMecSpeedUnit pointer to int16_t, used to return the rotor average
*         mechanical speed (expressed in the unit defined by #SPEED_UNIT)
* @retval true = sensor information is reliable
*         false = sensor information is not reliable
*/
bool HALL_CalcAvrgMecSpeedUnit( HALL_Handle_t * pHandle, int16_t * hMecSpeedUnit );

/**
* @brief  It could be used to set istantaneous information on rotor mechanical
*         angle.
*         Note: Mechanical angle management is not implemented in this
*         version of Hall sensor class.
* @param  pHandle pointer on related component instance
* @param  hMecAngle istantaneous measure of rotor mechanical angle
* @retval none
*/
void HALL_SetMecAngle( HALL_Handle_t * pHandle, int16_t hMecAngle );

#endif
