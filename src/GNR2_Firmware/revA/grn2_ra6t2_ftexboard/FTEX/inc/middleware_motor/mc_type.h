/**
  * @file    mc_type.h
  * @brief   Motor Control global types definitions
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_TYPE_H
#define __MC_TYPE_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/**
 * Each of the following symbols defines a rotation speed unit that can be used by the
 * functions of the API for their speed parameter. Each Unit is defined by expressing
 * the value of 1 Hz in this unit.
 *
 * These symbols can be used to set the #SPEED_UNIT macro which defines the rotation speed
 * unit used by the functions of the API.
 *
*/
/** Revolutions Per Minute: 1 Hz is 60 RPM */
#define _RPM 60
/** Tenth of Hertz: 1 Hz is 10 01Hz */
#define _01HZ 10
/** Hundreth of Hertz: 1 Hz is 100 001Hz */
#define _001HZ 100
/** @} */

/* Definitions placed here will not be erased by code generation */
/**
 * @brief Rotation speed unit used at the interface with the application
 *
 * This symbols defines the value of 1 Hertz in the unit used by the functions of the API for
 * their speed parameters.
 *
 * For instance, if the chosen unit is the RPM, SPEED_UNIT is defined to 60, since 1 Hz is 60 RPM.
 *
 * @note This symbol should not be set to a literal numeric value. Rather, it should be set to one
 *       of the symbols predefined for that purpose such as #_RPM, #_01HZ,... See @ref bSpeedUnit for
 *       more details.
 */
#define SPEED_UNIT _RPM

/**
 * @brief use a circle limitation that privileges Vd component instead of Vdq angle (uses more MIPS)
 *
 *        to use a circle limitation that keeps Vdq angle uncomment the define below
 *        (Beware: this uses more MIPS)
 */
/*#define CIRCLE_LIMITATION_VD*/


/** @name Macros to use bit banding capability */
/** @{ */
#define BB_REG_BIT_SET(regAddr,bit_number) *(uint32_t *) (0x42000000+(((uint32_t)regAddr - 0x40000000)<<5) + (bit_number <<2)) = (uint32_t)(0x1u)
#define BB_REG_BIT_CLR(regAddr,bit_number) (*(uint32_t *) (0x42000000+(((uint32_t)regAddr - 0x40000000)<<5) + (bit_number <<2)) = (uint32_t)(0x0u))
#define BB_REG_BIT_READ(regAddr,bit_number) (*(uint32_t *) (0x42000000+(((uint32_t)regAddr - 0x40000000)<<5) + (bit_number <<2)))
/** @} */

/** @brief Macro to calculate absolute value of a number */
#define ABSOLUTE(x) (x < 0) ? -x : x

/** @brief Not initialized pointer */
#define MC_NULL    (void *)(0x0)

/** @name Motor identification macros */
/** @{ */
#define M1      (uint8_t)(0x0)  /*!< Motor 1.*/
#define M2      (uint8_t)(0x1)  /*!< Motor 2.*/
#define M_NONE  (uint8_t)(0xFF) /*!< None motor.*/
/** @} */

/** @name Fault source error codes */
/** @{ */
#define  MC_NO_ERROR  (uint16_t)(0x0000u)      /**< @brief No error.*/
#define  MC_NO_FAULTS  (uint16_t)(0x0000u)     /**< @brief No error.*/
#define  MC_FOC_DURATION  (uint16_t)(0x0001u)  /**< @brief Error: FOC rate to high.*/
#define  MC_OVER_VOLT  (uint16_t)(0x0002u)     /**< @brief Error: Software over voltage.*/
#define  MC_UNDER_VOLT  (uint16_t)(0x0004u)    /**< @brief Error: Software under voltage.*/
#define  MC_OVER_TEMP  (uint16_t)(0x0008u)     /**< @brief Error: Software over temperature.*/
#define  MC_START_UP  (uint16_t)(0x0010u)      /**< @brief Error: Startup failed.*/
#define  MC_SPEED_FDBK  (uint16_t)(0x0020u)    /**< @brief Error: Speed feedback.*/
#define  MC_BREAK_IN  (uint16_t)(0x0040u)      /**< @brief Error: Emergency input (Over current).*/
#define  MC_SW_ERROR  (uint16_t)(0x0080u)      /**< @brief Software Error.*/
#define  MC_OCSP  		(uint16_t)(0x0100u)    /**< @brief Error: Overcurrent software protection.*/
#define  MC_MSRP		(uint16_t)(0x0200u)    /**< @brief Error: Motor Stuck & Reverse Protection.*/

/** @} */

/**
  * @brief Two components q, d type definition
  */
typedef struct
{
  int16_t q;
  int16_t d;
} qd_t;

/**
  * @brief Two components a,b type definition
  */
typedef struct
{
  int16_t a;
  int16_t b;
} ab_t;

/**
  * @brief Two components alpha, beta type definition
  */
typedef struct
{
  int16_t alpha;
  int16_t beta;
} AlphaBeta_t;


/**
  * @brief  SensorType_t type definition, it's used in BusVoltageSensor and TemperatureSensor component parameters structures
  *       to specify whether the sensor is real or emulated by SW
  */
typedef enum
{
  REAL_SENSOR, VIRTUAL_SENSOR
} SensorType_t;


/**
  * @brief  STCModality_t type definition, it's used by SpdTorqCtrl_SetControlMode and SpdTorqCtrl_GetControlMode methods in
  *         SpeednTorqCtrl class to specify the control modality type
  */
typedef enum
{
  STC_TORQUE_MODE, /**< @brief Torque mode.*/
  STC_SPEED_MODE   /**< @brief Speed mode.*/
} STCModality_t;


/**
  * @brief Structure type definition for feed-forward constants tuning
  */
typedef struct
{
  int32_t wConst1D;
  int32_t wConst1Q;
  int32_t wConst2;
} FeedforwardTuningStruct_t;

/**
  * @brief  Current references source type, internal or external to FOCDriveClass
  */
typedef enum
{
  INTERNAL, EXTERNAL
} CurrRefSource_t ;

/**
  * @brief  FOC variables structure
  */
typedef struct
{
  ab_t Iab;                    /**< @brief Stator current on stator reference frame abc */
  AlphaBeta_t Ialphabeta;      /**< @brief Stator current on stator reference frame alfa-beta*/
  qd_t IqdHF;                  /**< @brief Stator current on stator reference frame alfa-beta*/
  qd_t Iqd;                    /**< @brief Stator current on rotor reference frame qd */
  qd_t Iqdref;                 /**< @brief Stator current on rotor reference frame qd */
  int16_t UserIdref;           /**< @brief User value for the Idref stator current */
  qd_t Vqd;                    /**< @brief Phase voltage on rotor reference frame qd */
  AlphaBeta_t Valphabeta;      /**< @brief Phase voltage on stator reference frame alpha-beta*/
  int16_t hTeref;              /**< @brief Reference torque */
  int16_t hElAngle;            /**< @brief Electrical angle used for reference frame transformation  */
  uint16_t hCodeError;         /**< @brief error message */
  CurrRefSource_t bDriveInput; /**< @brief It specifies whether the current reference source must be
                                 *         #INTERNAL or #EXTERNAL*/
} FOCVars_t, *pFOCVars_t;


/** @name Utility macros definitions */
/** @{ */
#define RPM2MEC01HZ(rpm) (int16_t)((int32_t)(rpm)/6)
#define MAX(a,b) (((a)>(b))?(a):(b))
/** @} */


#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __MC_TYPE_H */
