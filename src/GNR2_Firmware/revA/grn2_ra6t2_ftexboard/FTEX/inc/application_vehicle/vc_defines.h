/**
  * @file    vc_defines.h
  * @author  Sami Bouzid, FTEX
  * @brief   This header is used for general definitions used by vehicle control API.
  *
	*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VC_DEFINES_H
#define __VC_DEFINES_H

#include "stdint.h"

/** @name Vehicle Fault source error codes */
/** @{ */
#define  VC_NO_ERROR   								(uint16_t)(0x0000u)      /**< @brief No error.*/
#define  VC_NO_FAULTS  								(uint16_t)(0x0000u)     /**< @brief No error.*/
#define  VC_M1_FAULTS  								(uint16_t)(0x0001u)  /**< @brief Error: Fault with motor 1.*/
#define  VC_M2_FAULTS  								(uint16_t)(0x0002u)     /**< @brief Error: Fault with motor 2.*/
#define	 VC_SW_ERROR   								(uint16_t)(0x0004u)     /**< @brief Error: Vehicle software error.*/
#define	 VC_MC_COMM_ERROR   				 	(uint16_t)(0x0008u)     /**< @brief Error: Communication with motor controller.*/
#define	 VC_M1_UNEXPECTED_BEHAVIOR   	(uint16_t)(0x0020u)     /**< @brief Error: Unexpected behavior of M1, i.e. when state is not what it's supposed to be*/
#define	 VC_M2_UNEXPECTED_BEHAVIOR   	(uint16_t)(0x0040u)     /**< @brief Error: Unexpected behavior of M2, i.e. when state is not what it's supposed to be*/
#define	 VC_START_TIMEOUT				   		(uint16_t)(0x0080u)     /**< @brief Error: Startup procedure reached timeout.*/
#define	 VC_STOP_TIMEOUT				   		(uint16_t)(0x0080u)     /**< @brief Error: Stop procedure reached timeout.*/
/** @} */


#endif /*__MC_DEFINES_H*/
