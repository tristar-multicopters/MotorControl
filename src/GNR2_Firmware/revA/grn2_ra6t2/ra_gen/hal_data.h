/* generated HAL header file - do not edit */
#ifndef HAL_DATA_H_
#define HAL_DATA_H_
#include <stdint.h>
#include "bsp_api.h"
#include "common_data.h"
#include "r_agt.h"
#include "r_timer_api.h"
#include "r_canfd.h"
#include "r_can_api.h"
#include "r_iirfa.h"
#include "r_iir_api.h"
#include "r_sci_b_uart.h"
            #include "r_uart_api.h"
#include "r_gpt.h"
#include "r_timer_api.h"
#include "r_icu.h"
#include "r_external_irq_api.h"
#include "r_dac.h"
#include "r_dac_api.h"
#include "r_poeg.h"
#include "r_poeg_api.h"
#include "r_adc_b.h"
                      #include "r_adc_api.h"
#include "r_gpt_three_phase.h"
            #include "r_three_phase_api.h"
FSP_HEADER
/** AGT Timer Instance */
extern const timer_instance_t g_timer1;

/** Access the AGT instance using these structures when calling API functions directly (::p_api is not used). */
extern agt_instance_ctrl_t g_timer1_ctrl;
extern const timer_cfg_t g_timer1_cfg;

#ifndef g_co_TIM_callback
void g_co_TIM_callback(timer_callback_args_t * p_args);
#endif
/** CANFD on CANFD Instance. */
extern const can_instance_t g_canfd0;
/** Access the CANFD instance using these structures when calling API functions directly (::p_api is not used). */
extern canfd_instance_ctrl_t g_canfd0_ctrl;
extern const can_cfg_t g_canfd0_cfg;
extern const canfd_extended_cfg_t g_canfd0_cfg_extend;

#ifndef canfd0_callback
void canfd0_callback(can_callback_args_t * p_args);
#endif

/* Global configuration (referenced by all instances) */
extern canfd_global_cfg_t g_canfd_global_cfg;
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
/** UART on SCI Instance. */
            extern const uart_instance_t      g_uart0;

            /** Access the UART instance using these structures when calling API functions directly (::p_api is not used). */
            extern sci_b_uart_instance_ctrl_t     g_uart0_ctrl;
            extern const uart_cfg_t g_uart0_cfg;
            extern const sci_b_uart_extended_cfg_t g_uart0_cfg_extend;

            #ifndef UART_IRQHandler
            void UART_IRQHandler(uart_callback_args_t * p_args);
            #endif
/** Timer on GPT Instance. */
extern const timer_instance_t g_timer8;

/** Access the GPT instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_instance_ctrl_t g_timer8_ctrl;
extern const timer_cfg_t g_timer8_cfg;

#ifndef WFREQ_TIM_Callback
void WFREQ_TIM_Callback(timer_callback_args_t * p_args);
#endif
/** AGT Timer Instance */
extern const timer_instance_t ag_timer0;

/** Access the AGT instance using these structures when calling API functions directly (::p_api is not used). */
extern agt_instance_ctrl_t ag_timer0_ctrl;
extern const timer_cfg_t ag_timer0_cfg;

#ifndef PFREQ_TIM_Callback
void PFREQ_TIM_Callback(timer_callback_args_t * p_args);
#endif
/** External IRQ on ICU Instance. */
extern const external_irq_instance_t g_external_irq2;

/** Access the ICU instance using these structures when calling API functions directly (::p_api is not used). */
extern icu_instance_ctrl_t g_external_irq2_ctrl;
extern const external_irq_cfg_t g_external_irq2_cfg;

#ifndef NULL
void NULL(external_irq_callback_args_t * p_args);
#endif
/** External IRQ on ICU Instance. */
extern const external_irq_instance_t g_external_irq1;

/** Access the ICU instance using these structures when calling API functions directly (::p_api is not used). */
extern icu_instance_ctrl_t g_external_irq1_ctrl;
extern const external_irq_cfg_t g_external_irq1_cfg;

#ifndef NULL
void NULL(external_irq_callback_args_t * p_args);
#endif
/** External IRQ on ICU Instance. */
extern const external_irq_instance_t g_external_irq0;

/** Access the ICU instance using these structures when calling API functions directly (::p_api is not used). */
extern icu_instance_ctrl_t g_external_irq0_ctrl;
extern const external_irq_cfg_t g_external_irq0_cfg;

#ifndef NULL
void NULL(external_irq_callback_args_t * p_args);
#endif
/** Timer on GPT Instance. */
extern const timer_instance_t g_timer0;

/** Access the GPT instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_instance_ctrl_t g_timer0_ctrl;
extern const timer_cfg_t g_timer0_cfg;

#ifndef HALL_TIM_UP_CC_IRQHandler
void HALL_TIM_UP_CC_IRQHandler(timer_callback_args_t * p_args);
#endif
/** DAC on DAC Instance. */
extern const dac_instance_t g_dac0;

/** Access the DAC instance using these structures when calling API functions directly (::p_api is not used). */
extern dac_instance_ctrl_t g_dac0_ctrl;
extern const dac_cfg_t g_dac0_cfg;
/** DAC on DAC Instance. */
extern const dac_instance_t g_dac1;

/** Access the DAC instance using these structures when calling API functions directly (::p_api is not used). */
extern dac_instance_ctrl_t g_dac1_ctrl;
extern const dac_cfg_t g_dac1_cfg;
/** POEG Instance. */
extern const poeg_instance_t g_poeg1;

/** Access the POEG instance using these structures when calling API functions directly (::p_api is not used). */
extern poeg_instance_ctrl_t g_poeg1_ctrl;
extern const poeg_cfg_t g_poeg1_cfg;

#ifndef PWM_TIM_BRK_IRQHandler
void PWM_TIM_BRK_IRQHandler(poeg_callback_args_t * p_args);
#endif
/** ADC on ADC_B instance. */
                    extern const adc_instance_t g_adc;

                    /** Access the ADC_B instance using these structures when calling API functions directly (::p_api is not used). */
                    extern adc_b_instance_ctrl_t g_adc_ctrl;
                    extern const adc_cfg_t g_adc_cfg;
                    extern const adc_b_scan_cfg_t g_adc_scan_cfg;

                    #ifndef CS_ADC_IRQHandler
                    void CS_ADC_IRQHandler(adc_callback_args_t * p_args);
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

#ifndef PWM_TIM_UP_IRQHandler
void PWM_TIM_UP_IRQHandler(timer_callback_args_t * p_args);
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
