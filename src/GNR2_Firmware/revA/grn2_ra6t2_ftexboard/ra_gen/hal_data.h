/* generated HAL header file - do not edit */
#ifndef HAL_DATA_H_
#define HAL_DATA_H_
#include <stdint.h>
#include "bsp_api.h"
#include "common_data.h"
#include "r_flash_hp.h"
#include "r_flash_api.h"
#include "bootutil/bootutil.h"
#include "r_dtc.h"
#include "r_transfer_api.h"
#include "r_spi_b.h"
#include "r_gpt.h"
#include "r_timer_api.h"
#include "r_agt.h"
#include "r_timer_api.h"
#include "r_canfd.h"
#include "r_can_api.h"
#include "r_sci_b_uart.h"
            #include "r_uart_api.h"
#include "r_iirfa.h"
#include "r_iir_api.h"
#include "r_poeg.h"
#include "r_poeg_api.h"
#include "r_dac.h"
#include "r_dac_api.h"
#include "r_adc_b.h"
                      #include "r_adc_api.h"
#include "r_gpt_three_phase.h"
            #include "r_three_phase_api.h"
FSP_HEADER
/* Flash on Flash HP Instance */
extern const flash_instance_t g_flash0;

/** Access the Flash HP instance using these structures when calling API functions directly (::p_api is not used). */
extern flash_hp_instance_ctrl_t g_flash0_ctrl;
extern const flash_cfg_t g_flash0_cfg;

#ifndef NULL
void NULL(flash_callback_args_t * p_args);
#endif
void mcuboot_image_utilities_quick_setup();
/* Transfer on DTC Instance. */
extern const transfer_instance_t g_transfer1;

/** Access the DTC instance using these structures when calling API functions directly (::p_api is not used). */
extern dtc_instance_ctrl_t g_transfer1_ctrl;
extern const transfer_cfg_t g_transfer1_cfg;
/* Transfer on DTC Instance. */
extern const transfer_instance_t g_transfer0;

/** Access the DTC instance using these structures when calling API functions directly (::p_api is not used). */
extern dtc_instance_ctrl_t g_transfer0_ctrl;
extern const transfer_cfg_t g_transfer0_cfg;
/** SPI on SPI Instance. */
extern const spi_instance_t g_spi1;

/** Access the SPI instance using these structures when calling API functions directly (::p_api is not used). */
extern spi_b_instance_ctrl_t g_spi1_ctrl;
extern const spi_cfg_t g_spi1_cfg;

/** Callback used by SPI Instance. */
#ifndef spi_callback
void spi_callback(spi_callback_args_t * p_args);
#endif


#define RA_NOT_DEFINED (1)
#if (RA_NOT_DEFINED == g_transfer0)
    #define g_spi1_P_TRANSFER_TX (NULL)
#else
    #define g_spi1_P_TRANSFER_TX (&g_transfer0)
#endif
#if (RA_NOT_DEFINED == g_transfer1)
    #define g_spi1_P_TRANSFER_RX (NULL)
#else
    #define g_spi1_P_TRANSFER_RX (&g_transfer1)
#endif
#undef RA_NOT_DEFINED
/** Timer on GPT Instance. */
extern const timer_instance_t g_timer9;

/** Access the GPT instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_instance_ctrl_t g_timer9_ctrl;
extern const timer_cfg_t g_timer9_cfg;

#ifndef WheelSpeedTimer_IRQHandler
void WheelSpeedTimer_IRQHandler(timer_callback_args_t * p_args);
#endif
/** AGT Timer Instance */
extern const timer_instance_t g_timer_a0;

/** Access the AGT instance using these structures when calling API functions directly (::p_api is not used). */
extern agt_instance_ctrl_t g_timer_a0_ctrl;
extern const timer_cfg_t g_timer_a0_cfg;

#ifndef PedalSpeedTimer_IRQHandler
void PedalSpeedTimer_IRQHandler(timer_callback_args_t * p_args);
#endif
/** AGT Timer Instance */
extern const timer_instance_t g_timer_a1;

/** Access the AGT instance using these structures when calling API functions directly (::p_api is not used). */
extern agt_instance_ctrl_t g_timer_a1_ctrl;
extern const timer_cfg_t g_timer_a1_cfg;

#ifndef CANTimer_IRQHandler
void CANTimer_IRQHandler(timer_callback_args_t * p_args);
#endif
/** CANFD on CANFD Instance. */
extern const can_instance_t g_canfd0;
/** Access the CANFD instance using these structures when calling API functions directly (::p_api is not used). */
extern canfd_instance_ctrl_t g_canfd0_ctrl;
extern const can_cfg_t g_canfd0_cfg;
extern const canfd_extended_cfg_t g_canfd0_cfg_extend;

#ifndef CANFD_IRQhandler
void CANFD_IRQhandler(can_callback_args_t * p_args);
#endif

/* Global configuration (referenced by all instances) */
extern canfd_global_cfg_t g_canfd_global_cfg;
/** UART on SCI Instance. */
            extern const uart_instance_t      g_uart9;

            /** Access the UART instance using these structures when calling API functions directly (::p_api is not used). */
            extern sci_b_uart_instance_ctrl_t     g_uart9_ctrl;
            extern const uart_cfg_t g_uart9_cfg;
            extern const sci_b_uart_extended_cfg_t g_uart9_cfg_extend;

            #ifndef UART_IRQHandler
            void UART_IRQHandler(uart_callback_args_t * p_args);
            #endif
/** Timer on GPT Instance. */
extern const timer_instance_t g_timer0;

/** Access the GPT instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_instance_ctrl_t g_timer0_ctrl;
extern const timer_cfg_t g_timer0_cfg;

#ifndef HallTimer_IRQHandler
void HallTimer_IRQHandler(timer_callback_args_t * p_args);
#endif
/** IIRFA Instance. */
extern const iirfa_instance_t g_iirfa1;

/** Access the IIRFA instance using these structures when calling API functions directly (::p_api is not used). */
extern iirfa_instance_ctrl_t g_iirfa1_ctrl;
extern const iir_cfg_t g_iirfa1_cfg;
/** IIRFA Instance. */
extern const iirfa_instance_t g_iirfa0;

/** Access the IIRFA instance using these structures when calling API functions directly (::p_api is not used). */
extern iirfa_instance_ctrl_t g_iirfa0_ctrl;
extern const iir_cfg_t g_iirfa0_cfg;
/** POEG Instance. */
extern const poeg_instance_t g_poeg1;

/** Access the POEG instance using these structures when calling API functions directly (::p_api is not used). */
extern poeg_instance_ctrl_t g_poeg1_ctrl;
extern const poeg_cfg_t g_poeg1_cfg;

#ifndef PWMBreak2_IRQHandler
void PWMBreak2_IRQHandler(poeg_callback_args_t * p_args);
#endif
/** POEG Instance. */
extern const poeg_instance_t g_poeg0;

/** Access the POEG instance using these structures when calling API functions directly (::p_api is not used). */
extern poeg_instance_ctrl_t g_poeg0_ctrl;
extern const poeg_cfg_t g_poeg0_cfg;

#ifndef PWMBreak1_IRQHandler
void PWMBreak1_IRQHandler(poeg_callback_args_t * p_args);
#endif
/** DAC on DAC Instance. */
extern const dac_instance_t g_dac3;

/** Access the DAC instance using these structures when calling API functions directly (::p_api is not used). */
extern dac_instance_ctrl_t g_dac3_ctrl;
extern const dac_cfg_t g_dac3_cfg;
/** DAC on DAC Instance. */
extern const dac_instance_t g_dac2;

/** Access the DAC instance using these structures when calling API functions directly (::p_api is not used). */
extern dac_instance_ctrl_t g_dac2_ctrl;
extern const dac_cfg_t g_dac2_cfg;
/** ADC on ADC_B instance. */
                    extern const adc_instance_t g_adc0;

                    /** Access the ADC_B instance using these structures when calling API functions directly (::p_api is not used). */
                    extern adc_b_instance_ctrl_t g_adc0_ctrl;
                    extern const adc_cfg_t g_adc0_cfg;
                    extern const adc_b_scan_cfg_t g_adc0_scan_cfg;

                    #ifndef ADC_IRQHandler
                    void ADC_IRQHandler(adc_callback_args_t * p_args);
                    #endif
/** Timer on GPT Instance. */
extern const timer_instance_t g_timer6;

/** Access the GPT instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_instance_ctrl_t g_timer6_ctrl;
extern const timer_cfg_t g_timer6_cfg;

#ifndef NULL
void NULL(timer_callback_args_t * p_args);
#endif
/** Timer on GPT Instance. */
extern const timer_instance_t g_timer5;

/** Access the GPT instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_instance_ctrl_t g_timer5_ctrl;
extern const timer_cfg_t g_timer5_cfg;

#ifndef NULL
void NULL(timer_callback_args_t * p_args);
#endif
/** Timer on GPT Instance. */
extern const timer_instance_t g_timer4;

/** Access the GPT instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_instance_ctrl_t g_timer4_ctrl;
extern const timer_cfg_t g_timer4_cfg;

#ifndef PWMTimer_IRQHandler
void PWMTimer_IRQHandler(timer_callback_args_t * p_args);
#endif
/** GPT Three-Phase Instance. */
extern const three_phase_instance_t g_three_phase0;

/** Access the GPT Three-Phase instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_three_phase_instance_ctrl_t g_three_phase0_ctrl;
extern const three_phase_cfg_t g_three_phase0_cfg;
void hal_entry(void);
void g_hal_init(void);
FSP_FOOTER
#endif /* HAL_DATA_H_ */
