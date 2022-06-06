/**
  * @file    hall_speed_pos_fdbk.h
  * @brief   This file contains all definitions and functions prototypes for the
  *          Hall Speed & Position Feedback component of the Motor Control application.
*/

#ifndef __HALL_SPEEDNPOSFDBK_H
#define __HALL_SPEEDNPOSFDBK_H

#include "speed_pos_fdbk.h"
#include "r_gpt.h"
#include "r_timer_api.h"


#define HALL_SPEED_FIFO_SIZE  ((uint8_t)18)
#define DEGREES_120 0u
#define DEGREES_60 1u


/* Exported types ------------------------------------------------------------*/

typedef struct
{
    SpeednPosFdbkHandle_t Super; /* Base class module */

    uint8_t  bSensorPlacement; /*!< Define here the mechanical position of the sensors
                     with reference to an electrical cycle.
                     Allowed values are: DEGREES_120 or DEGREES_60.*/

    int16_t  hPhaseShift;  /*!< Define here in s16degree the electrical phase shift
                     between the low to high transition of signal H1 and
                     the maximum of the Bemf induced on phase A.*/

    uint16_t hSpeedSamplingFreqHz; /*!< Frequency (Hz) at which motor speed is to
                     be computed. It must be equal to the frequency
                     at which function SPD_CalcAvrgMecSpeedUnit
                     is called.*/

    uint8_t  bSpeedBufferSize; /*!< Size of the buffer used to calculate the average
                     speed. It must be less than 18.*/

    /* HW Settings */
    uint32_t wTIMClockFreq; /*!< Timer clock frequency express in Hz.*/

    const timer_instance_t  * TIMx;   /*!< Timer used for HALL sensor management.*/

    bsp_io_port_pin_t H1PortPin;
    bsp_io_port_pin_t H2PortPin;
    bsp_io_port_pin_t H3PortPin;

    bool bSensorIsReliable;            /*!< Flag to indicate a wrong configuration
                                 of the Hall sensor signanls.*/

    volatile bool bRatioDec;           /*!< Flag to avoid consecutive prescaler
                                 decrement.*/
    volatile bool bRatioInc;           /*!< Flag to avoid consecutive prescaler
                                 increment.*/
    volatile uint8_t bFirstCapt;      /*!< Flag used to discard first capture for
                                 the speed measurement*/
    volatile uint8_t bBufferFilled;   /*!< Indicate the number of speed measuremt
                                 present in the buffer from the start.
                                 It will be max bSpeedBufferSize and it
                                 is used to validate the start of speed
                                 averaging. If bBufferFilled is below
                                 bSpeedBufferSize the instantaneous
                                 measured speed is returned as average
                                 speed.*/
    volatile uint16_t hOVFCounter;     /*!< Count overflows if prescaler is too low
                                 */

    uint32_t SensorPeriod[HALL_SPEED_FIFO_SIZE];/*!< Holding the last
                                 period captures */

    uint8_t bSensorPeriod;/*!< Pointer of next element to be stored in
                                 the speed sensor buffer*/

    uint32_t  wElPeriodSum; /* Period accumulator used to speed up the average speed computation*/

    uint32_t  wMaxElSum; /* maximum sum of periods, used to check if timoout has occured or not */

    int16_t hPrevRotorFreq; /*!< Used to store the last valid rotor electrical
                       speed in dpp used when HALL_MAX_PSEUDO_SPEED
                       is detected */
    int8_t bDirection;          /*!< Instantaneous direction of rotor between two
                       captures*/

    int16_t hAvrElSpeedDpp; /*!< It is the averaged rotor electrical speed express
                       in s16degree per current control period.*/

    uint8_t bHallState;     /*!< Current HALL state configuration */

    int16_t hDeltaAngle;    /*!< Delta angle at the Hall sensor signal edge between
                       current electrical rotor angle of synchronism.
                       It is in s16degrees.*/
    int16_t hMeasuredElAngle;/*!< This is the electrical angle  measured at each
                       Hall sensor signal edge. It is considered the
                       best measurement of electrical rotor angle.*/

    int16_t hCompSpeed;     /*!< Speed compensation factor used to syncronize
                       the current electrical angle with the target
                       electrical angle. */

    uint16_t hHallMaxRatio; /*!< Max TIM prescaler ratio defining the lowest
                     expected speed feedback.*/
    uint16_t hSaturationSpeed;     /*!< Returned value if the measured speed is above the
                     maximum realistic.*/
    uint32_t wPseudoFreqConv;/*!< Conversion factor between time interval Delta T
                     between HALL sensors captures, express in timer
                     counts, and electrical rotor speed express in dpp.
                     Ex. Rotor speed (dpp) = wPseudoFreqConv / Delta T
                     It will be ((CKTIM / 6) / (SAMPLING_FREQ)) * 65536.*/

    uint32_t wMaxPeriod;  /*!< Time delay between two sensor edges when the speed
                     of the rotor is the minimum realistic in the
                     application: this allows to discriminate too low
                     freq for instance.
                     This period shoud be expressed in timer counts and
                     it will be:
                     wMaxPeriod = ((10 * CKTIM) / 6) / MinElFreq(0.1Hz).*/

    uint32_t wMinPeriod;
    /*!< Time delay between two sensor edges when the speed
    of the rotor is the maximum realistic in the
    application: this allows discriminating glitches
    for instance.
    This period shoud be expressed in timer counts and
    it will be:
    wSpeedOverflow = ((10 * CKTIM) / 6) / MaxElFreq(0.1Hz).*/

    uint16_t hHallTimeout;/*!< Max delay between two Hall sensor signal to assert
                     zero speed express in milliseconds.*/

    uint16_t hOvfFreq;   /*!< Frequency of timer overflow (from 0 to 0x10000)
                     it will be: hOvfFreq = CKTIM /65536.*/
    uint16_t hPWMNbrPSamplingFreq; /*!< Number of current control periods inside
                     each speed control periods it will be:
                     (hMeasurementFrequency / hSpeedSamplingFreqHz) - 1.*/
    uint8_t bPWMFreqScaling; /*!< Scaling factor to allow to store a PWMFrequency greater than 16 bits */

    bool bHallMtpa; /* if true at each sensor toggling, the true angle is set without ramp*/

    int16_t SectorStartAngle[7];
    int16_t SectorDestinationAngle[7];
    int16_t SectorMiddleAngle[7];
    int8_t bDirectionChangeCounter;

} HallPosSensorHandle_t;


/**
* @brief  Method of the class HALL to implement an MC IRQ function
*         to be called when TIMx update event occurs
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
* @retval none
*/
void * HallPosSensor_TIMx_UP_IRQHandler( void * pHandleVoid );

/**
* @brief  Method of the class HALL to implement an MC IRQ function
*         to be called when TIMx capture compare event occurs
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
* @retval none
*/
void * HallPosSensor_TIMx_CC_IRQHandler( void * pHandleVoid, uint32_t* pCapture );

/**
  * @brief  It initializes the hardware peripherals (Timer, GPIO and NVIC)
            required for the speed position sensor management using HALL
            sensors.
  * @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
  * @retval none
  */
void HallPosSensor_Init( HallPosSensorHandle_t * pHandle );


void HallPosSensor_Clear( HallPosSensorHandle_t * pHandle );

/**
* @brief  Update the rotor electrical angle integrating the last measured
*         instantaneous electrical speed express in dpp.
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
* @retval int16_t Measured electrical angle in s16degree format.
*/
int16_t HallPosSensor_CalcElAngle( HallPosSensorHandle_t * pHandle );

/**
  * @brief  This method must be called - at least - with the same periodicity
  *         on which speed control is executed.
  *         This method compute and store rotor istantaneous el speed (express
  *         in dpp considering the measurement frequency) in order to provide it
  *         to HallPosSensor_CalcElAngle function and SpdPosFdbk_GetElAngle.
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
bool HallPosSensor_CalcAvrgMecSpeedUnit( HallPosSensorHandle_t * pHandle, int16_t * hMecSpeedUnit );


void HallPosSensor_SetMecAngle( HallPosSensorHandle_t * pHandle, int16_t hMecAngle );



#endif /*__HALL_SPEEDNPOSFDBK_H*/
