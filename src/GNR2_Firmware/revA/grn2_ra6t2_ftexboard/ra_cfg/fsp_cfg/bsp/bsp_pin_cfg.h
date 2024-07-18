/* generated configuration header file - do not edit */
#ifndef BSP_PIN_CFG_H_
#define BSP_PIN_CFG_H_
#include "r_ioport.h"

/* Common macro for FSP header files. There is also a corresponding FSP_FOOTER macro at the end of this file. */
FSP_HEADER

#define M2_MOTOR_SELECT (BSP_IO_PORT_00_PIN_01)
#define M1_CURR_U (BSP_IO_PORT_10_PIN_00)
#define M1_CURR_V (BSP_IO_PORT_10_PIN_02)
#define VBUS_SENSE_M1 (BSP_IO_PORT_10_PIN_06)
#define M1_MOTOR_SELECT (BSP_IO_PORT_10_PIN_07)
#define CAN_RES (BSP_IO_PORT_10_PIN_09)
#define M1_HS_TEMP (BSP_IO_PORT_11_PIN_00)
#define THROTTLE (BSP_IO_PORT_11_PIN_01)
#define CAN_STBY_N (BSP_IO_PORT_11_PIN_04)
#define M1_ENC_U (BSP_IO_PORT_11_PIN_05)
#define M1_ENC_V (BSP_IO_PORT_11_PIN_06)
#define M1_ENC_W (BSP_IO_PORT_11_PIN_07)
#define PWREN_GPIO_PIN (BSP_IO_PORT_11_PIN_08)
#define DRIVER_EN (BSP_IO_PORT_11_PIN_10)
#define BRAKE (BSP_IO_PORT_12_PIN_00)
#define DEBUG1_DAC (BSP_IO_PORT_12_PIN_04)
#define DEBUG2_DAC (BSP_IO_PORT_12_PIN_05)
#define OCD2_U (BSP_IO_PORT_12_PIN_14)
#define OCD2_V (BSP_IO_PORT_12_PIN_15)
#define SPI_CS (BSP_IO_PORT_13_PIN_00)
#define LIGHT_FRONT (BSP_IO_PORT_13_PIN_04)
#define CAN_EN (BSP_IO_PORT_13_PIN_07)
#define LIGHT_BACK (BSP_IO_PORT_13_PIN_10)
#define STATUS1 (BSP_IO_PORT_14_PIN_00)
#define STATUS2 (BSP_IO_PORT_14_PIN_01)
#define PAS_TORQUE (BSP_IO_PORT_14_PIN_08)
#define PWM_U_P (BSP_IO_PORT_14_PIN_10)
#define PWM_V_P (BSP_IO_PORT_14_PIN_11)
#define PWM_W_P (BSP_IO_PORT_14_PIN_12)
#define PWM_U_N (BSP_IO_PORT_14_PIN_13)
#define PWM_V_N (BSP_IO_PORT_14_PIN_14)
#define PWM_W_N (BSP_IO_PORT_14_PIN_15)
extern const ioport_cfg_t g_bsp_pin_cfg; /* R7FA6T2BD3CFP.pincfg */

void BSP_PinConfigSecurityInit();

/* Common macro for FSP header files. There is also a corresponding FSP_HEADER macro at the top of this file. */
FSP_FOOTER

#endif /* BSP_PIN_CFG_H_ */
