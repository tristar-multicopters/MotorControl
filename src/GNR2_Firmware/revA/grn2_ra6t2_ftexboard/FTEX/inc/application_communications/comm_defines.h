/**
  * @file    comm_defines.h
  * @author  FTEX
  * @brief   This header is used for general definitions used by communication application.
  *
    */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COMM_DEFINES_H
#define __COMM_DEFINES_H

/** @name Communication error codes */
/** @{ */
#define  COMM_NO_ERROR               (uint16_t)(0x0000u)      /**< @brief No error. */
#define  CANBUS_HARD_ERROR          (uint16_t)(0x0001u)      /**< @brief Hardware error on CANbus. */
#define  CANOPEN_ERROR              (uint16_t)(0x0002u)      /**< @brief CANOpen general error. */
#define  MASTER_SLAVE_NO_HEARTBEAT  (uint16_t)(0x0004u)      /**< @brief No heartbeat from master or slave */
/** @} */

#endif /*__COMM_DEFINES_H*/
