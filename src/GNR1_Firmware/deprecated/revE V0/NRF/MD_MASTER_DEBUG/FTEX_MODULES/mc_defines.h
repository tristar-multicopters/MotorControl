/**
  ******************************************************************************
  * @file    mc_defines.h
  * @author  Sami Bouzid, FTEX
  * @brief   This header defines the registers and codes used by STM32 Motor Control firmware.
  *
	******************************************************************************
	*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_DEFINES_H
#define __MC_DEFINES_H

#include "stdint.h"

typedef enum
{
  MC_PROTOCOL_REG_TARGET_MOTOR,          /* 0   */
  MC_PROTOCOL_REG_FLAGS,                 /* 1   */
  MC_PROTOCOL_REG_STATUS,                /* 2   */
  MC_PROTOCOL_REG_CONTROL_MODE,          /* 3   */
  MC_PROTOCOL_REG_SPEED_REF,             /* 4   */
  MC_PROTOCOL_REG_SPEED_KP,              /* 5   */
  MC_PROTOCOL_REG_SPEED_KI,              /* 6   */
  MC_PROTOCOL_REG_SPEED_KD,              /* 7   */
  MC_PROTOCOL_REG_TORQUE_REF,            /* 8   */
  MC_PROTOCOL_REG_TORQUE_KP,             /* 9   */
  MC_PROTOCOL_REG_TORQUE_KI,             /* 10  */
  MC_PROTOCOL_REG_TORQUE_KD,             /* 11  */
  MC_PROTOCOL_REG_FLUX_REF,              /* 12  */
  MC_PROTOCOL_REG_FLUX_KP,               /* 13  */
  MC_PROTOCOL_REG_FLUX_KI,               /* 14  */
  MC_PROTOCOL_REG_FLUX_KD,               /* 15  */
  MC_PROTOCOL_REG_OBSERVER_C1,           /* 16  */
  MC_PROTOCOL_REG_OBSERVER_C2,           /* 17  */
  MC_PROTOCOL_REG_OBSERVER_CR_C1,        /* 18  */
  MC_PROTOCOL_REG_OBSERVER_CR_C2,        /* 19  */
  MC_PROTOCOL_REG_PLL_KI,                /* 20  */
  MC_PROTOCOL_REG_PLL_KP,                /* 21  */
  MC_PROTOCOL_REG_FLUXWK_KP,             /* 22  */
  MC_PROTOCOL_REG_FLUXWK_KI,             /* 23  */
  MC_PROTOCOL_REG_FLUXWK_BUS,            /* 24  */
  MC_PROTOCOL_REG_BUS_VOLTAGE,           /* 25  */
  MC_PROTOCOL_REG_HEATS_TEMP,            /* 26  */
  MC_PROTOCOL_REG_MOTOR_POWER,           /* 27  */
  MC_PROTOCOL_REG_DAC_OUT1,              /* 28  */
  MC_PROTOCOL_REG_DAC_OUT2,              /* 29  */
  MC_PROTOCOL_REG_SPEED_MEAS,            /* 30  */
  MC_PROTOCOL_REG_TORQUE_MEAS,           /* 31  */
  MC_PROTOCOL_REG_FLUX_MEAS,             /* 32  */
  MC_PROTOCOL_REG_FLUXWK_BUS_MEAS,       /* 33  */
  MC_PROTOCOL_REG_RUC_STAGE_NBR,         /* 34  */
  MC_PROTOCOL_REG_I_A,                   /* 35  */
  MC_PROTOCOL_REG_I_B,                   /* 36  */
  MC_PROTOCOL_REG_I_ALPHA,               /* 37  */
  MC_PROTOCOL_REG_I_BETA,                /* 38  */
  MC_PROTOCOL_REG_I_Q,                   /* 39  */
  MC_PROTOCOL_REG_I_D,                   /* 40  */
  MC_PROTOCOL_REG_I_Q_REF,               /* 41  */
  MC_PROTOCOL_REG_I_D_REF,               /* 42  */
  MC_PROTOCOL_REG_V_Q,                   /* 43  */
  MC_PROTOCOL_REG_V_D,                   /* 44  */
  MC_PROTOCOL_REG_V_ALPHA,               /* 45  */
  MC_PROTOCOL_REG_V_BETA,                /* 46  */
  MC_PROTOCOL_REG_MEAS_EL_ANGLE,         /* 47  */
  MC_PROTOCOL_REG_MEAS_ROT_SPEED,        /* 48  */
  MC_PROTOCOL_REG_OBS_EL_ANGLE,          /* 49  */
  MC_PROTOCOL_REG_OBS_ROT_SPEED,         /* 50  */
  MC_PROTOCOL_REG_OBS_I_ALPHA,           /* 51  */
  MC_PROTOCOL_REG_OBS_I_BETA,            /* 52  */
  MC_PROTOCOL_REG_OBS_BEMF_ALPHA,        /* 53  */
  MC_PROTOCOL_REG_OBS_BEMF_BETA,         /* 54  */
  MC_PROTOCOL_REG_OBS_CR_EL_ANGLE,       /* 55  */
  MC_PROTOCOL_REG_OBS_CR_ROT_SPEED,      /* 56  */
  MC_PROTOCOL_REG_OBS_CR_I_ALPHA,        /* 57  */
  MC_PROTOCOL_REG_OBS_CR_I_BETA,         /* 58  */
  MC_PROTOCOL_REG_OBS_CR_BEMF_ALPHA,     /* 59  */
  MC_PROTOCOL_REG_OBS_CR_BEMF_BETA,      /* 60  */
  MC_PROTOCOL_REG_DAC_USER1,             /* 61  */
  MC_PROTOCOL_REG_DAC_USER2,             /* 62  */
  MC_PROTOCOL_REG_MAX_APP_SPEED,         /* 63  */
  MC_PROTOCOL_REG_MIN_APP_SPEED,         /* 64  */
  MC_PROTOCOL_REG_IQ_SPEEDMODE,          /* 65  */
  MC_PROTOCOL_REG_EST_BEMF_LEVEL,        /* 66  */
  MC_PROTOCOL_REG_OBS_BEMF_LEVEL,        /* 67  */
  MC_PROTOCOL_REG_EST_CR_BEMF_LEVEL,     /* 68  */
  MC_PROTOCOL_REG_OBS_CR_BEMF_LEVEL,     /* 69  */
  MC_PROTOCOL_REG_FF_1Q,                 /* 70  */
  MC_PROTOCOL_REG_FF_1D,                 /* 71  */
  MC_PROTOCOL_REG_FF_2,                  /* 72  */
  MC_PROTOCOL_REG_FF_VQ,                 /* 73  */
  MC_PROTOCOL_REG_FF_VD,                 /* 74  */
  MC_PROTOCOL_REG_FF_VQ_PIOUT,           /* 75  */
  MC_PROTOCOL_REG_FF_VD_PIOUT,           /* 76  */
  MC_PROTOCOL_REG_PFC_STATUS,            /* 77  */
  MC_PROTOCOL_REG_PFC_FAULTS,            /* 78  */
  MC_PROTOCOL_REG_PFC_DCBUS_REF,         /* 79  */
  MC_PROTOCOL_REG_PFC_DCBUS_MEAS,        /* 80  */
  MC_PROTOCOL_REG_PFC_ACBUS_FREQ,        /* 81  */
  MC_PROTOCOL_REG_PFC_ACBUS_RMS,         /* 82  */
  MC_PROTOCOL_REG_PFC_I_KP,              /* 83  */
  MC_PROTOCOL_REG_PFC_I_KI,              /* 84  */
  MC_PROTOCOL_REG_PFC_I_KD,              /* 85  */
  MC_PROTOCOL_REG_PFC_V_KP,              /* 86  */
  MC_PROTOCOL_REG_PFC_V_KI,              /* 87  */
  MC_PROTOCOL_REG_PFC_V_KD,              /* 88  */
  MC_PROTOCOL_REG_PFC_STARTUP_DURATION,  /* 89  */
  MC_PROTOCOL_REG_PFC_ENABLED,           /* 90  */
  MC_PROTOCOL_REG_RAMP_FINAL_SPEED,      /* 91  */
  MC_PROTOCOL_REG_RAMP_DURATION,         /* 92  */
  MC_PROTOCOL_REG_HFI_EL_ANGLE,          /* 93  */
  MC_PROTOCOL_REG_HFI_ROT_SPEED,         /* 94  */
  MC_PROTOCOL_REG_HFI_CURRENT,           /* 95  */
  MC_PROTOCOL_REG_HFI_INIT_ANG_PLL,      /* 96  */
  MC_PROTOCOL_REG_HFI_INIT_ANG_SAT_DIFF, /* 97  */
  MC_PROTOCOL_REG_HFI_PI_PLL_KP,         /* 98  */
  MC_PROTOCOL_REG_HFI_PI_PLL_KI,         /* 99  */
  MC_PROTOCOL_REG_HFI_PI_TRACK_KP,       /* 100 */
  MC_PROTOCOL_REG_HFI_PI_TRACK_KI,       /* 101 */
  MC_PROTOCOL_REG_SC_CHECK,              /* 102 */
  MC_PROTOCOL_REG_SC_STATE,              /* 103 */
  MC_PROTOCOL_REG_SC_RS,                 /* 104 */
  MC_PROTOCOL_REG_SC_LS,                 /* 105 */
  MC_PROTOCOL_REG_SC_KE,                 /* 106 */
  MC_PROTOCOL_REG_SC_VBUS,               /* 107 */
  MC_PROTOCOL_REG_SC_MEAS_NOMINALSPEED,  /* 108 */
  MC_PROTOCOL_REG_SC_STEPS,              /* 109 */
  MC_PROTOCOL_REG_SPEED_KP_DIV,          /* 110 */
  MC_PROTOCOL_REG_SPEED_KI_DIV,          /* 111 */
  MC_PROTOCOL_REG_UID,                   /* 112 */
  MC_PROTOCOL_REG_HWTYPE,                /* 113 */
  MC_PROTOCOL_REG_CTRBDID,               /* 114 */
  MC_PROTOCOL_REG_PWBDID,                /* 115 */
  MC_PROTOCOL_REG_SC_PP,                 /* 116 */
  MC_PROTOCOL_REG_SC_CURRENT,            /* 117 */
  MC_PROTOCOL_REG_SC_SPDBANDWIDTH,       /* 118 */
  MC_PROTOCOL_REG_SC_LDLQRATIO,          /* 119 */
  MC_PROTOCOL_REG_SC_NOMINAL_SPEED,      /* 120 */
  MC_PROTOCOL_REG_SC_CURRBANDWIDTH,      /* 121 */
  MC_PROTOCOL_REG_SC_J,                  /* 122 */
  MC_PROTOCOL_REG_SC_F,                  /* 123 */
  MC_PROTOCOL_REG_SC_MAX_CURRENT,        /* 124 */
  MC_PROTOCOL_REG_SC_STARTUP_SPEED,      /* 125 */
  MC_PROTOCOL_REG_SC_STARTUP_ACC,        /* 126 */
  MC_PROTOCOL_REG_SC_PWM_FREQUENCY,      /* 127 */
  MC_PROTOCOL_REG_SC_FOC_REP_RATE,       /* 128 */
  MC_PROTOCOL_REG_PWBDID2,               /* 129 */
  MC_PROTOCOL_REG_SC_COMPLETED,          /* 130 */
  MC_PROTOCOL_REG_CURRENT_POSITION,      /* 131 */
  MC_PROTOCOL_REG_TARGET_POSITION,       /* 132 */
  MC_PROTOCOL_REG_MOVE_DURATION,         /* 133 */
  MC_PROTOCOL_REG_POSITION_KP,           /* 134 */
  MC_PROTOCOL_REG_POSITION_KI,           /* 135 */
  MC_PROTOCOL_REG_POSITION_KD,           /* 136 */
  MC_PROTOCOL_REG_UNDEFINED
} MC_Protocol_REG_t;

typedef enum
{
    MC_PROTOCOL_CONFIGREG_TORQUE_KP_INIT,    /* 0  */
    MC_PROTOCOL_CONFIGREG_TORQUE_KI_INIT,    /* 1  */
    MC_PROTOCOL_CONFIGREG_FLUX_KI_INIT,      /* 2  */
    MC_PROTOCOL_CONFIGREG_FLUX_KP_INIT,      /* 3  */
    MC_PROTOCOL_CONFIGREG_TEMPERATURE_MAX,   /* 4  */
    MC_PROTOCOL_CONFIGREG_MAX_DC_VOLTAGE,    /* 5  */
    MC_PROTOCOL_CONFIGREG_MIN_DC_VOLTAGE, 	 /* 6  */
    MC_PROTOCOL_CONFIGREG_MAX_PHASE_CURRENT, /* 7  */
    MC_PROTOCOL_CONFIGREG_MOTOR_PHASE_SHIFT, /* 8  */
    MC_PROTOCOL_CONFIGREG_MOTOR_POLE_PAIRS,  /* 9  */
	  MC_PROTOCOL_CONFIGREG_CURRENT_DC,		     /* 10 */
    MC_PROTOCOL_CONFIGREG_WH_DIAMETER,		   /* 11 */
    MC_PROTOCOL_CONFIGREG_WH_MAXSPEED,       /* 12 */
	  MC_PROTOCOL_CONFIGREG_WH_GRATIO,		     /* 13 */
	  MC_PROTOCOL_CONFIGREG_TH_SAADC,          /* 14 */
    MC_PROTOCOL_CONFIGREG_TH_TORQUE,   		   /* 15 */
	  MC_PROTOCOL_CONFIGREG_TH_VOFFSET, 	     /* 16 */
    MC_PROTOCOL_CONFIGREG_TH_SENSIBILITY,    /* 17 */
	  MC_PROTOCOL_CONFIGREG_TH_TORQUERESP      /* 18 */
} MC_Protocol_CONFIGREG_t;

#define MC_PROTOCOL_CODE_SET_REG          0x01
#define MC_PROTOCOL_CODE_GET_REG          0x02
#define MC_PROTOCOL_CODE_EXECUTE_CMD      0x03
#define MC_PROTOCOL_CODE_STORE_TOADDR     0x04
#define MC_PROTOCOL_CODE_LOAD_FROMADDR    0x05
#define MC_PROTOCOL_CODE_GET_BOARD_INFO   0x06
#define MC_PROTOCOL_CODE_SET_SPEED_RAMP   0x07
#define MC_PROTOCOL_CODE_GET_REVUP_DATA   0x08
#define MC_PROTOCOL_CODE_SET_REVUP_DATA   0x09
#define MC_PROTOCOL_CODE_SET_CURRENT_REF  0x0A
#define MC_PROTOCOL_CODE_GET_MP_INFO      0x0B
#define MC_PROTOCOL_CODE_GET_FW_VERSION   0x0C
#define MC_PROTOCOL_CODE_SET_TORQUE_RAMP  0x0D
#define MC_PROTOCOL_CODE_SET_POSITION_CMD 0x12

#define MC_PROTOCOL_CODE_SET_CONFIG_MD  	0X13
#define MC_PROTOCOL_CODE_GET_CONFIG_MD  	0X14

#define MC_PROTOCOL_CMD_START_MOTOR       0x01
#define MC_PROTOCOL_CMD_STOP_MOTOR        0x02
#define MC_PROTOCOL_CMD_STOP_RAMP         0x03
#define MC_PROTOCOL_CMD_RESET             0x04
#define MC_PROTOCOL_CMD_PING              0x05
#define MC_PROTOCOL_CMD_START_STOP        0x06
#define MC_PROTOCOL_CMD_FAULT_ACK         0x07
#define MC_PROTOCOL_CMD_ENCODER_ALIGN     0x08
#define MC_PROTOCOL_CMD_IQDREF_CLEAR      0x09
#define MC_PROTOCOL_CMD_PFC_ENABLE        0x0A
#define MC_PROTOCOL_CMD_PFC_DISABLE       0x0B
#define MC_PROTOCOL_CMD_PFC_FAULT_ACK     0x0C
#define MC_PROTOCOL_CMD_SC_START          0x0D
#define MC_PROTOCOL_CMD_SC_STOP           0x0E

#define MC_PROTOCOL_CMD_OVERWRITE_FLASH   0x20
#define MC_PROTOCOL_CMD_ENTER_CONFIG      0x21
#define MC_PROTOCOL_CMD_EXIT_CONFIG       0x22

#define CTRBDID 29
#define PWBDID 0
#define MC_UID 883328122

#define ACK_NOERROR 0xF0
#define ACK_ERROR   0xFF
#define ATR_FRAME_START 0xE0

#define MC_PROTOCOL_CODE_NONE        0x00

/* List of error codes */
typedef enum ERROR_CODE_e
{
	ERROR_NONE = 0,             /**<  0x00 - No error */
	ERROR_BAD_FRAME_ID,         /**<  0x01 - BAD Frame ID. The Frame ID has not been recognized by the firmware. */
	ERROR_CODE_SET_READ_ONLY,   /**<  0x02 - Write on read-only. The master wants to write on a read-only register. */
	ERROR_CODE_GET_WRITE_ONLY,  /**<  0x03 - Read not allowed. The value cannot be read. */
	ERROR_CODE_NO_TARGET_DRIVE, /**<  0x04 - Bad target drive. The target motor is not supported by the firmware. */
	ERROR_CODE_WRONG_SET,       /**<  0x05 - Value used in the frame is out of range expected by the FW. */
	ERROR_CODE_CMD_ID,          /**<  0x06 - NOT USED */
	ERROR_CODE_WRONG_CMD,       /**<  0x07 - Bad command ID. The command ID has not been recognized. */
	ERROR_CODE_OVERRUN,         /**<  0x08 - Overrun error. Transmission speed too fast, frame not received correctly */
	ERROR_CODE_TIMEOUT,         /**<  0x09 - Timeout error. Received frame corrupted or unrecognized by the FW. */
	ERROR_CODE_BAD_CRC,         /**<  0x0A - The computed CRC is not equal to the received CRC byte. */
	ERROR_BAD_MOTOR_SELECTED,   /**<  0x0B - Bad target drive. The target motor is not supported by the firmware. */
	ERROR_MP_NOT_ENABLED        /**<  0x0C - Motor Profiler not enabled. */
} ERROR_CODE;


typedef enum
{
  M_TORQUE_MODE, /**< Torque mode.*/
  M_SPEED_MODE   /**< Speed mode.*/
} MC_Modality_t;

typedef enum
{
  M_ICLWAIT = 12,         /*!< Persistent state, the system is waiting for ICL
                           deactivation. Is not possible to run the motor if
                           ICL is active. Until the ICL is active the state is
                           forced to ICLWAIT, when ICL become inactive the state
                           is moved to IDLE */
  M_IDLE = 0,             /*!< Persistent state, following state can be IDLE_START
                           if a start motor command has been given or
                           IDLE_ALIGNMENT if a start alignment command has been
                           given */
  M_IDLE_ALIGNMENT = 1,   /*!< "Pass-through" state containg the code to be executed
                           only once after encoder alignment command.
                           Next states can be ALIGN_CHARGE_BOOT_CAP or
                           ALIGN_OFFSET_CALIB according the configuration. It
                           can also be ANY_STOP if a stop motor command has been
                           given. */
  M_ALIGN_CHARGE_BOOT_CAP = 13,/*!< Persistent state where the gate driver boot
                           capacitors will be charged. Next states will be
                           ALIGN_OFFSET_CALIB. It can also be ANY_STOP if a stop
                           motor command has been given. */
  M_ALIGN_OFFSET_CALIB = 14,/*!< Persistent state where the offset of motor currents
                           measurements will be calibrated. Next state will be
                           ALIGN_CLEAR. It can also be ANY_STOP if a stop motor
                           command has been given. */
  M_ALIGN_CLEAR = 15,     /*!< "Pass-through" state in which object is cleared and
                           set for the startup.
                           Next state will be ALIGNMENT. It can also be ANY_STOP
                           if a stop motor command has been given. */
  M_ALIGNMENT = 2,        /*!< Persistent state in which the encoder are properly
                           aligned to set mechanical angle, following state can
                           only be ANY_STOP */
  M_IDLE_START = 3,       /*!< "Pass-through" state containg the code to be executed
                           only once after start motor command.
                           Next states can be CHARGE_BOOT_CAP or OFFSET_CALIB
                           according the configuration. It can also be ANY_STOP
                           if a stop motor command has been given. */
  M_CHARGE_BOOT_CAP = 16, /*!< Persistent state where the gate driver boot
                           capacitors will be charged. Next states will be
                           OFFSET_CALIB. It can also be ANY_STOP if a stop motor
                           command has been given. */
  M_OFFSET_CALIB = 17,    /*!< Persistent state where the offset of motor currents
                           measurements will be calibrated. Next state will be
                           CLEAR. It can also be ANY_STOP if a stop motor
                           command has been given. */
  M_CLEAR = 18,           /*!< "Pass-through" state in which object is cleared and
                           set for the startup.
                           Next state will be START. It can also be ANY_STOP if
                           a stop motor command has been given. */
  M_START = 4,            /*!< Persistent state where the motor start-up is intended
                           to be executed. The following state is normally
                           SWITCH_OVER or RUN as soon as first validated speed is
                           detected. Another possible following state is
                           ANY_STOP if a stop motor command has been executed */
  M_SWITCH_OVER = 19,     /**< TBD */
  M_START_RUN = 5,        /*!< "Pass-through" state, the code to be executed only
                           once between START and RUN states it’s intended to be
                           here executed. Following state is normally  RUN but
                           it can also be ANY_STOP  if a stop motor command has
                           been given */
  M_RUN = 6,              /*!< Persistent state with running motor. The following
                           state is normally ANY_STOP when a stop motor command
                           has been executed */
  M_ANY_STOP = 7,         /*!< "Pass-through" state, the code to be executed only
                           once between any state and STOP it’s intended to be
                           here executed. Following state is normally STOP */
  M_STOP = 8,             /*!< Persistent state. Following state is normally
                           STOP_IDLE as soon as conditions for moving state
                           machine are detected */
  M_STOP_IDLE = 9,        /*!< "Pass-through" state, the code to be executed only
                           once between STOP and IDLE it’s intended to be here
                           executed. Following state is normally IDLE */
  M_FAULT_NOW = 10,       /*!< Persistent state, the state machine can be moved from
                           any condition directly to this state by
                           STM_FaultProcessing method. This method also manage
                           the passage to the only allowed following state that
                           is FAULT_OVER */
  M_FAULT_OVER = 11,       /*!< Persistent state where the application is intended to
                          stay when the fault conditions disappeared. Following
                          state is normally STOP_IDLE, state machine is moved as
                          soon as the user has acknowledged the fault condition.
                      */
  M_WAIT_STOP_MOTOR = 20

} MC_State_t;

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
/** @} */


#endif /*__MC_DEFINES_H*/
