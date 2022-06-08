/* generated HAL source file - do not edit */
#include "hal_data.h"


/* Macros to tie dynamic ELC links to adc_b_trigger_sync_elc option in adc_b_trigger_t. */
                     #define ADC_B_TRIGGER_ADC_B0        ADC_B_TRIGGER_SYNC_ELC
                     #define ADC_B_TRIGGER_ADC_B0_B      ADC_B_TRIGGER_SYNC_ELC
                     #define ADC_B_TRIGGER_ADC_B1        ADC_B_TRIGGER_SYNC_ELC
                     #define ADC_B_TRIGGER_ADC_B1_B      ADC_B_TRIGGER_SYNC_ELC

gpt_instance_ctrl_t g_timer8_ctrl;
#if 0
const gpt_extended_pwm_cfg_t g_timer8_pwm_extend =
{
    .trough_ipl          = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT8_COUNTER_UNDERFLOW)
    .trough_irq          = VECTOR_NUMBER_GPT8_COUNTER_UNDERFLOW,
#else
    .trough_irq          = FSP_INVALID_VECTOR,
#endif
    .poeg_link           = GPT_POEG_LINK_POEG0,
    .output_disable      =  GPT_OUTPUT_DISABLE_NONE,
    .adc_trigger         =  GPT_ADC_TRIGGER_NONE,
    .dead_time_count_up  = 0,
    .dead_time_count_down = 0,
    .adc_a_compare_match = 0,
    .adc_b_compare_match = 0,
    .interrupt_skip_source = GPT_INTERRUPT_SKIP_SOURCE_NONE,
    .interrupt_skip_count  = GPT_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_adc    = GPT_INTERRUPT_SKIP_ADC_NONE,
    .gtioca_disable_setting = GPT_GTIOC_DISABLE_PROHIBITED,
    .gtiocb_disable_setting = GPT_GTIOC_DISABLE_PROHIBITED,
};
#endif
const gpt_extended_cfg_t g_timer8_extend =
{
    .gtioca = { .output_enabled = false,
                .stop_level     = GPT_PIN_LEVEL_LOW
              },
    .gtiocb = { .output_enabled = false,
                .stop_level     = GPT_PIN_LEVEL_LOW
              },
    .start_source        = (gpt_source_t) (GPT_SOURCE_GTIOCB_RISING_WHILE_GTIOCA_LOW |  GPT_SOURCE_NONE),
    .stop_source         = (gpt_source_t) (GPT_SOURCE_GTIOCB_FALLING_WHILE_GTIOCA_LOW |  GPT_SOURCE_NONE),
    .clear_source        = (gpt_source_t) (GPT_SOURCE_GTIOCB_RISING_WHILE_GTIOCA_LOW |  GPT_SOURCE_NONE),
    .count_up_source     = (gpt_source_t) ( GPT_SOURCE_NONE),
    .count_down_source   = (gpt_source_t) ( GPT_SOURCE_NONE),
    .capture_a_source    = (gpt_source_t) ( GPT_SOURCE_NONE),
    .capture_b_source    = (gpt_source_t) (GPT_SOURCE_GTIOCB_FALLING_WHILE_GTIOCA_LOW |  GPT_SOURCE_NONE),
    .capture_a_ipl       = (BSP_IRQ_DISABLED),
    .capture_b_ipl       = (12),
#if defined(VECTOR_NUMBER_GPT8_CAPTURE_COMPARE_A)
    .capture_a_irq       = VECTOR_NUMBER_GPT8_CAPTURE_COMPARE_A,
#else
    .capture_a_irq       = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_GPT8_CAPTURE_COMPARE_B)
    .capture_b_irq       = VECTOR_NUMBER_GPT8_CAPTURE_COMPARE_B,
#else
    .capture_b_irq       = FSP_INVALID_VECTOR,
#endif
    .capture_filter_gtioca       = GPT_CAPTURE_FILTER_NONE,
    .capture_filter_gtiocb       = GPT_CAPTURE_FILTER_NONE,
#if 0
    .p_pwm_cfg                   = &g_timer8_pwm_extend,
#else
    .p_pwm_cfg                   = NULL,
#endif
#if 0
    .gtior_setting.gtior_b.gtioa  = (0U << 4U) | (0U << 2U) | (0U << 0U),
    .gtior_setting.gtior_b.oadflt = (uint32_t) GPT_PIN_LEVEL_LOW,
    .gtior_setting.gtior_b.oahld  = 0U,
    .gtior_setting.gtior_b.oae    = (uint32_t) false,
    .gtior_setting.gtior_b.oadf   = (uint32_t) GPT_GTIOC_DISABLE_PROHIBITED,
    .gtior_setting.gtior_b.nfaen  = ((uint32_t) GPT_CAPTURE_FILTER_NONE & 1U),
    .gtior_setting.gtior_b.nfcsa  = ((uint32_t) GPT_CAPTURE_FILTER_NONE >> 1U),
    .gtior_setting.gtior_b.gtiob  = (0U << 4U) | (0U << 2U) | (0U << 0U),
    .gtior_setting.gtior_b.obdflt = (uint32_t) GPT_PIN_LEVEL_LOW,
    .gtior_setting.gtior_b.obhld  = 0U,
    .gtior_setting.gtior_b.obe    = (uint32_t) false,
    .gtior_setting.gtior_b.obdf   = (uint32_t) GPT_GTIOC_DISABLE_PROHIBITED,
    .gtior_setting.gtior_b.nfben  = ((uint32_t) GPT_CAPTURE_FILTER_NONE & 1U),
    .gtior_setting.gtior_b.nfcsb  = ((uint32_t) GPT_CAPTURE_FILTER_NONE >> 1U),
#else
    .gtior_setting.gtior = 0U,
#endif
};
const timer_cfg_t g_timer8_cfg =
{
    .mode                = TIMER_MODE_PERIODIC,
    /* Actual period: 35.791394133333334 seconds. Actual duty: 50%. */ .period_counts = (uint32_t) 0x100000000, .duty_cycle_counts = 0x80000000, .source_div = (timer_source_div_t)0,
    .channel             = 8,
    .p_callback          = WFREQ_TIM_Callback,
    /** If NULL then do not add & */
#if defined(NULL)
    .p_context           = NULL,
#else
    .p_context           = &NULL,
#endif
    .p_extend            = &g_timer8_extend,
    .cycle_end_ipl       = (12),
#if defined(VECTOR_NUMBER_GPT8_COUNTER_OVERFLOW)
    .cycle_end_irq       = VECTOR_NUMBER_GPT8_COUNTER_OVERFLOW,
#else
    .cycle_end_irq       = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const timer_instance_t g_timer8 =
{
    .p_ctrl        = &g_timer8_ctrl,
    .p_cfg         = &g_timer8_cfg,
    .p_api         = &g_timer_on_gpt
};
agt_instance_ctrl_t ag_timer0_ctrl;
const agt_extended_cfg_t ag_timer0_extend =
{
    .count_source            = AGT_CLOCK_PCLKB,
    .agto                    = AGT_PIN_CFG_DISABLED,
    .agtoab_settings_b.agtoa = AGT_PIN_CFG_DISABLED,
    .agtoab_settings_b.agtob = AGT_PIN_CFG_DISABLED,
    .measurement_mode        = AGT_MEASURE_PULSE_PERIOD,
    .agtio_filter            = AGT_AGTIO_FILTER_NONE,
    .enable_pin              = AGT_ENABLE_PIN_NOT_USED,
    .trigger_edge            = AGT_TRIGGER_EDGE_FALLING,
};
const timer_cfg_t ag_timer0_cfg =
{
    .mode                = TIMER_MODE_PERIODIC,
    /* Actual period: 0.0010922666666666667 seconds. Actual duty: 50%. */ .period_counts = (uint32_t) 0x10000, .duty_cycle_counts = 0x8000, .source_div = (timer_source_div_t)0,
    .channel             = 0,
    .p_callback          = PFREQ_TIM_Callback,
    /** If NULL then do not add & */
#if defined(NULL)
    .p_context           = NULL,
#else
    .p_context           = &NULL,
#endif
    .p_extend            = &ag_timer0_extend,
    .cycle_end_ipl       = (12),
#if defined(VECTOR_NUMBER_AGT0_INT)
    .cycle_end_irq       = VECTOR_NUMBER_AGT0_INT,
#else
    .cycle_end_irq       = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const timer_instance_t ag_timer0 =
{
    .p_ctrl        = &ag_timer0_ctrl,
    .p_cfg         = &ag_timer0_cfg,
    .p_api         = &g_timer_on_agt
};
icu_instance_ctrl_t g_external_irq2_ctrl;
const external_irq_cfg_t g_external_irq2_cfg =
{
    .channel             = 1,
    .trigger             = EXTERNAL_IRQ_TRIG_BOTH_EDGE,
    .filter_enable       = true,
    .pclk_div            = EXTERNAL_IRQ_PCLK_DIV_BY_64,
    .p_callback          = NULL,
    /** If NULL then do not add & */
#if defined(NULL)
    .p_context           = NULL,
#else
    .p_context           = &NULL,
#endif
    .p_extend            = NULL,
    .ipl                 = (15),
#if defined(VECTOR_NUMBER_ICU_IRQ1)
    .irq                 = VECTOR_NUMBER_ICU_IRQ1,
#else
    .irq                 = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const external_irq_instance_t g_external_irq2 =
{
    .p_ctrl        = &g_external_irq2_ctrl,
    .p_cfg         = &g_external_irq2_cfg,
    .p_api         = &g_external_irq_on_icu
};
icu_instance_ctrl_t g_external_irq1_ctrl;
const external_irq_cfg_t g_external_irq1_cfg =
{
    .channel             = 11,
    .trigger             = EXTERNAL_IRQ_TRIG_BOTH_EDGE,
    .filter_enable       = true,
    .pclk_div            = EXTERNAL_IRQ_PCLK_DIV_BY_64,
    .p_callback          = NULL,
    /** If NULL then do not add & */
#if defined(NULL)
    .p_context           = NULL,
#else
    .p_context           = &NULL,
#endif
    .p_extend            = NULL,
    .ipl                 = (15),
#if defined(VECTOR_NUMBER_ICU_IRQ11)
    .irq                 = VECTOR_NUMBER_ICU_IRQ11,
#else
    .irq                 = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const external_irq_instance_t g_external_irq1 =
{
    .p_ctrl        = &g_external_irq1_ctrl,
    .p_cfg         = &g_external_irq1_cfg,
    .p_api         = &g_external_irq_on_icu
};
icu_instance_ctrl_t g_external_irq0_ctrl;
const external_irq_cfg_t g_external_irq0_cfg =
{
    .channel             = 10,
    .trigger             = EXTERNAL_IRQ_TRIG_BOTH_EDGE,
    .filter_enable       = true,
    .pclk_div            = EXTERNAL_IRQ_PCLK_DIV_BY_64,
    .p_callback          = NULL,
    /** If NULL then do not add & */
#if defined(NULL)
    .p_context           = NULL,
#else
    .p_context           = &NULL,
#endif
    .p_extend            = NULL,
    .ipl                 = (15),
#if defined(VECTOR_NUMBER_ICU_IRQ10)
    .irq                 = VECTOR_NUMBER_ICU_IRQ10,
#else
    .irq                 = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const external_irq_instance_t g_external_irq0 =
{
    .p_ctrl        = &g_external_irq0_ctrl,
    .p_cfg         = &g_external_irq0_cfg,
    .p_api         = &g_external_irq_on_icu
};
gpt_instance_ctrl_t g_timer0_ctrl;
#if 0
const gpt_extended_pwm_cfg_t g_timer0_pwm_extend =
{
    .trough_ipl          = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT0_COUNTER_UNDERFLOW)
    .trough_irq          = VECTOR_NUMBER_GPT0_COUNTER_UNDERFLOW,
#else
    .trough_irq          = FSP_INVALID_VECTOR,
#endif
    .poeg_link           = GPT_POEG_LINK_POEG0,
    .output_disable      =  GPT_OUTPUT_DISABLE_NONE,
    .adc_trigger         =  GPT_ADC_TRIGGER_NONE,
    .dead_time_count_up  = 0,
    .dead_time_count_down = 0,
    .adc_a_compare_match = 0,
    .adc_b_compare_match = 0,
    .interrupt_skip_source = GPT_INTERRUPT_SKIP_SOURCE_NONE,
    .interrupt_skip_count  = GPT_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_adc    = GPT_INTERRUPT_SKIP_ADC_NONE,
    .gtioca_disable_setting = GPT_GTIOC_DISABLE_PROHIBITED,
    .gtiocb_disable_setting = GPT_GTIOC_DISABLE_PROHIBITED,
};
#endif
const gpt_extended_cfg_t g_timer0_extend =
{
    .gtioca = { .output_enabled = false,
                .stop_level     = GPT_PIN_LEVEL_LOW
              },
    .gtiocb = { .output_enabled = false,
                .stop_level     = GPT_PIN_LEVEL_LOW
              },
    .start_source        = (gpt_source_t) ( GPT_SOURCE_NONE),
    .stop_source         = (gpt_source_t) ( GPT_SOURCE_NONE),
    .clear_source        = (gpt_source_t) (GPT_SOURCE_GPT_A |  GPT_SOURCE_NONE),
    .count_up_source     = (gpt_source_t) ( GPT_SOURCE_NONE),
    .count_down_source   = (gpt_source_t) ( GPT_SOURCE_NONE),
    .capture_a_source    = (gpt_source_t) (GPT_SOURCE_GPT_H | GPT_SOURCE_GPT_G | GPT_SOURCE_GPT_F |  GPT_SOURCE_NONE),
    .capture_b_source    = (gpt_source_t) ( GPT_SOURCE_NONE),
    .capture_a_ipl       = (5),
    .capture_b_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT0_CAPTURE_COMPARE_A)
    .capture_a_irq       = VECTOR_NUMBER_GPT0_CAPTURE_COMPARE_A,
#else
    .capture_a_irq       = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_GPT0_CAPTURE_COMPARE_B)
    .capture_b_irq       = VECTOR_NUMBER_GPT0_CAPTURE_COMPARE_B,
#else
    .capture_b_irq       = FSP_INVALID_VECTOR,
#endif
    .capture_filter_gtioca       = GPT_CAPTURE_FILTER_NONE,
    .capture_filter_gtiocb       = GPT_CAPTURE_FILTER_NONE,
#if 0
    .p_pwm_cfg                   = &g_timer0_pwm_extend,
#else
    .p_pwm_cfg                   = NULL,
#endif
#if 0
    .gtior_setting.gtior_b.gtioa  = (0U << 4U) | (0U << 2U) | (0U << 0U),
    .gtior_setting.gtior_b.oadflt = (uint32_t) GPT_PIN_LEVEL_LOW,
    .gtior_setting.gtior_b.oahld  = 0U,
    .gtior_setting.gtior_b.oae    = (uint32_t) false,
    .gtior_setting.gtior_b.oadf   = (uint32_t) GPT_GTIOC_DISABLE_PROHIBITED,
    .gtior_setting.gtior_b.nfaen  = ((uint32_t) GPT_CAPTURE_FILTER_NONE & 1U),
    .gtior_setting.gtior_b.nfcsa  = ((uint32_t) GPT_CAPTURE_FILTER_NONE >> 1U),
    .gtior_setting.gtior_b.gtiob  = (0U << 4U) | (0U << 2U) | (0U << 0U),
    .gtior_setting.gtior_b.obdflt = (uint32_t) GPT_PIN_LEVEL_LOW,
    .gtior_setting.gtior_b.obhld  = 0U,
    .gtior_setting.gtior_b.obe    = (uint32_t) false,
    .gtior_setting.gtior_b.obdf   = (uint32_t) GPT_GTIOC_DISABLE_PROHIBITED,
    .gtior_setting.gtior_b.nfben  = ((uint32_t) GPT_CAPTURE_FILTER_NONE & 1U),
    .gtior_setting.gtior_b.nfcsb  = ((uint32_t) GPT_CAPTURE_FILTER_NONE >> 1U),
#else
    .gtior_setting.gtior = 0U,
#endif
};
const timer_cfg_t g_timer0_cfg =
{
    .mode                = TIMER_MODE_PERIODIC,
    /* Actual period: 0.0043690583333333335 seconds. Actual duty: 49.99990463238646%. */ .period_counts = (uint32_t) 0x7ffff, .duty_cycle_counts = 0x3ffff, .source_div = (timer_source_div_t)0,
    .channel             = 0,
    .p_callback          = HALL_TIM_UP_CC_IRQHandler,
    /** If NULL then do not add & */
#if defined(NULL)
    .p_context           = NULL,
#else
    .p_context           = &NULL,
#endif
    .p_extend            = &g_timer0_extend,
    .cycle_end_ipl       = (5),
#if defined(VECTOR_NUMBER_GPT0_COUNTER_OVERFLOW)
    .cycle_end_irq       = VECTOR_NUMBER_GPT0_COUNTER_OVERFLOW,
#else
    .cycle_end_irq       = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const timer_instance_t g_timer0 =
{
    .p_ctrl        = &g_timer0_ctrl,
    .p_cfg         = &g_timer0_cfg,
    .p_api         = &g_timer_on_gpt
};
dac_instance_ctrl_t g_dac0_ctrl;
const dac_extended_cfg_t g_dac0_ext_cfg =
{
    .enable_charge_pump   = 0,
    .data_format         = DAC_DATA_FORMAT_FLUSH_LEFT,
    .output_amplifier_enabled = 0,
    .internal_output_enabled = false,
};
const dac_cfg_t g_dac0_cfg =
{
    .channel             = 0,
    .ad_da_synchronized  = false,
    .p_extend            = &g_dac0_ext_cfg
};
/* Instance structure to use this module. */
const dac_instance_t g_dac0 =
{
    .p_ctrl    = &g_dac0_ctrl,
    .p_cfg     = &g_dac0_cfg,
    .p_api     = &g_dac_on_dac
};
dac_instance_ctrl_t g_dac1_ctrl;
const dac_extended_cfg_t g_dac1_ext_cfg =
{
    .enable_charge_pump   = 0,
    .data_format         = DAC_DATA_FORMAT_FLUSH_LEFT,
    .output_amplifier_enabled = 0,
    .internal_output_enabled = false,
};
const dac_cfg_t g_dac1_cfg =
{
    .channel             = 1,
    .ad_da_synchronized  = false,
    .p_extend            = &g_dac1_ext_cfg
};
/* Instance structure to use this module. */
const dac_instance_t g_dac1 =
{
    .p_ctrl    = &g_dac1_ctrl,
    .p_cfg     = &g_dac1_cfg,
    .p_api     = &g_dac_on_dac
};
poeg_instance_ctrl_t g_poeg1_ctrl;
const poeg_cfg_t g_poeg1_cfg =
{
    .trigger             = (poeg_trigger_t) (POEG_TRIGGER_PIN |  POEG_TRIGGER_SOFTWARE),
    .polarity            = POEG_GTETRG_POLARITY_ACTIVE_LOW,
    .noise_filter        = POEG_GTETRG_NOISE_FILTER_PCLKB_DIV_32,
    .channel             = 1,
    .ipl                 = (0),
    .p_callback          = PWM_TIM_BRK_IRQHandler,
    .p_context           = NULL,
#if defined(VECTOR_NUMBER_POEG1_EVENT)
    .irq       = VECTOR_NUMBER_POEG1_EVENT,
#else
    .irq       = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const poeg_instance_t g_poeg1 =
{
    .p_ctrl        = &g_poeg1_ctrl,
    .p_cfg         = &g_poeg1_cfg,
    .p_api         = &g_poeg_on_poeg
};
#define RA_NOT_DEFINED (0) // TODO: Remove this after implementing all channels and groups.

                               #if (1) // Define Virtual Channel 0 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_0_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_0,

                                   .channel_cfg_bits.group             = (1),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_4),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_4) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_16_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_4) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (1) // Define Virtual Channel 1 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_1_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_1,

                                   .channel_cfg_bits.group             = (1),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_2),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_2) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_16_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_2) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 2 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_2_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_2,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 3 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_3_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_3,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 4 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_4_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_4,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 5 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_5_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_5,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 6 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_6_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_6,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 7 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_7_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_7,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 8 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_8_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_8,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 9 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_9_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_9,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 10 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_10_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_10,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 11 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_11_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_11,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 12 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_12_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_12,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 13 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_13_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_13,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 14 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_14_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_14,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 15 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_15_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_15,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 16 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_16_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_16,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 17 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_17_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_17,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 18 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_18_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_18,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 19 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_19_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_19,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 20 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_20_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_20,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 21 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_21_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_21,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 22 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_22_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_22,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 23 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_23_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_23,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 24 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_24_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_24,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 25 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_25_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_25,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 26 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_26_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_26,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 27 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_27_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_27,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 28 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_28_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_28,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 29 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_29_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_29,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 30 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_30_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_30,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 31 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_31_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_31,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 32 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_32_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_32,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 33 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_33_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_33,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 34 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_34_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_34,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 35 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_35_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_35,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (0) // Define Virtual Channel 36 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_36_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_36,

                                   .channel_cfg_bits.group             = (0),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_12_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif
#if (((1) == 1)||((1) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1))
const adc_b_virtual_channel_cfg_t *const group_0_virtual_channels[] = {
                                  #if ((1) == 1)
                                      &virtual_channel_0_cfg,
                                  #endif

                                  #if ((1) == 1)
                                      &virtual_channel_1_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_2_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_3_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_4_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_5_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_6_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_7_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_8_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_9_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_10_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_11_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_12_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_13_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_14_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_15_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_16_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_17_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_18_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_19_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_20_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_21_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_22_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_23_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_24_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_25_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_26_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_27_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_28_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_29_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_30_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_31_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_32_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_33_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_34_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_35_cfg,
                                  #endif

                                  #if ((0) == 1)
                                      &virtual_channel_36_cfg,
                                  #endif
                                  };
                                  #endif
#if (((1) == 2)||((1) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2))
const adc_b_virtual_channel_cfg_t *const group_1_virtual_channels[] = {
                                  #if ((1) == 2)
                                      &virtual_channel_0_cfg,
                                  #endif

                                  #if ((1) == 2)
                                      &virtual_channel_1_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_2_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_3_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_4_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_5_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_6_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_7_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_8_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_9_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_10_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_11_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_12_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_13_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_14_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_15_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_16_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_17_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_18_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_19_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_20_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_21_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_22_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_23_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_24_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_25_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_26_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_27_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_28_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_29_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_30_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_31_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_32_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_33_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_34_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_35_cfg,
                                  #endif

                                  #if ((0) == 2)
                                      &virtual_channel_36_cfg,
                                  #endif
                                  };
                                  #endif
#if (((1) == 3)||((1) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3))
const adc_b_virtual_channel_cfg_t *const group_2_virtual_channels[] = {
                                  #if ((1) == 3)
                                      &virtual_channel_0_cfg,
                                  #endif

                                  #if ((1) == 3)
                                      &virtual_channel_1_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_2_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_3_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_4_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_5_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_6_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_7_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_8_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_9_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_10_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_11_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_12_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_13_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_14_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_15_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_16_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_17_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_18_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_19_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_20_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_21_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_22_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_23_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_24_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_25_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_26_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_27_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_28_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_29_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_30_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_31_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_32_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_33_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_34_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_35_cfg,
                                  #endif

                                  #if ((0) == 3)
                                      &virtual_channel_36_cfg,
                                  #endif
                                  };
                                  #endif
#if (((1) == 4)||((1) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4))
const adc_b_virtual_channel_cfg_t *const group_3_virtual_channels[] = {
                                  #if ((1) == 4)
                                      &virtual_channel_0_cfg,
                                  #endif

                                  #if ((1) == 4)
                                      &virtual_channel_1_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_2_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_3_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_4_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_5_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_6_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_7_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_8_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_9_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_10_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_11_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_12_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_13_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_14_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_15_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_16_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_17_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_18_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_19_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_20_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_21_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_22_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_23_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_24_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_25_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_26_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_27_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_28_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_29_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_30_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_31_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_32_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_33_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_34_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_35_cfg,
                                  #endif

                                  #if ((0) == 4)
                                      &virtual_channel_36_cfg,
                                  #endif
                                  };
                                  #endif
#if (((1) == 5)||((1) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5))
const adc_b_virtual_channel_cfg_t *const group_4_virtual_channels[] = {
                                  #if ((1) == 5)
                                      &virtual_channel_0_cfg,
                                  #endif

                                  #if ((1) == 5)
                                      &virtual_channel_1_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_2_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_3_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_4_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_5_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_6_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_7_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_8_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_9_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_10_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_11_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_12_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_13_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_14_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_15_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_16_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_17_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_18_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_19_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_20_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_21_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_22_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_23_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_24_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_25_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_26_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_27_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_28_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_29_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_30_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_31_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_32_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_33_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_34_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_35_cfg,
                                  #endif

                                  #if ((0) == 5)
                                      &virtual_channel_36_cfg,
                                  #endif
                                  };
                                  #endif
#if (((1) == 6)||((1) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6))
const adc_b_virtual_channel_cfg_t *const group_5_virtual_channels[] = {
                                  #if ((1) == 6)
                                      &virtual_channel_0_cfg,
                                  #endif

                                  #if ((1) == 6)
                                      &virtual_channel_1_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_2_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_3_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_4_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_5_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_6_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_7_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_8_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_9_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_10_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_11_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_12_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_13_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_14_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_15_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_16_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_17_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_18_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_19_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_20_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_21_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_22_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_23_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_24_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_25_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_26_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_27_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_28_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_29_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_30_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_31_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_32_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_33_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_34_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_35_cfg,
                                  #endif

                                  #if ((0) == 6)
                                      &virtual_channel_36_cfg,
                                  #endif
                                  };
                                  #endif
#if (((1) == 7)||((1) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7))
const adc_b_virtual_channel_cfg_t *const group_6_virtual_channels[] = {
                                  #if ((1) == 7)
                                      &virtual_channel_0_cfg,
                                  #endif

                                  #if ((1) == 7)
                                      &virtual_channel_1_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_2_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_3_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_4_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_5_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_6_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_7_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_8_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_9_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_10_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_11_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_12_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_13_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_14_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_15_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_16_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_17_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_18_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_19_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_20_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_21_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_22_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_23_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_24_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_25_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_26_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_27_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_28_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_29_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_30_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_31_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_32_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_33_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_34_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_35_cfg,
                                  #endif

                                  #if ((0) == 7)
                                      &virtual_channel_36_cfg,
                                  #endif
                                  };
                                  #endif
#if (((1) == 8)||((1) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8))
const adc_b_virtual_channel_cfg_t *const group_7_virtual_channels[] = {
                                  #if ((1) == 8)
                                      &virtual_channel_0_cfg,
                                  #endif

                                  #if ((1) == 8)
                                      &virtual_channel_1_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_2_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_3_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_4_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_5_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_6_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_7_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_8_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_9_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_10_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_11_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_12_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_13_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_14_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_15_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_16_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_17_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_18_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_19_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_20_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_21_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_22_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_23_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_24_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_25_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_26_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_27_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_28_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_29_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_30_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_31_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_32_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_33_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_34_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_35_cfg,
                                  #endif

                                  #if ((0) == 8)
                                      &virtual_channel_36_cfg,
                                  #endif
                                  };
                                  #endif
#if (((1) == 9)||((1) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9))
const adc_b_virtual_channel_cfg_t *const group_8_virtual_channels[] = {
                                  #if ((1) == 9)
                                      &virtual_channel_0_cfg,
                                  #endif

                                  #if ((1) == 9)
                                      &virtual_channel_1_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_2_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_3_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_4_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_5_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_6_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_7_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_8_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_9_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_10_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_11_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_12_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_13_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_14_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_15_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_16_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_17_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_18_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_19_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_20_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_21_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_22_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_23_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_24_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_25_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_26_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_27_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_28_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_29_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_30_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_31_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_32_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_33_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_34_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_35_cfg,
                                  #endif

                                  #if ((0) == 9)
                                      &virtual_channel_36_cfg,
                                  #endif
                                  };
                                  #endif

                     #if (1) // Define Scan Group 0 if it's enabled
                     const adc_b_group_cfg_t adc_b_group_0_cfg =
                     {
                         .scan_group_id                   = ADC_GROUP_ID_0,
                         .converter_selection             = (0),
                         .scan_group_enable               = (1),
                         .scan_end_interrupt_enable       = (1),
                         .external_trigger_enable_mask    = ( ADC_B_EXTERNAL_TRIGGER_NONE),
                         .elc_trigger_enable_mask         = ( 0x00),
                         .gpt_trigger_enable_mask         = (0x10 |  ADC_B_GPT_TRIGGER_NONE),

                         .self_diagnosis_mask             = (ADC_B_SELF_DIAGNOSIS_DISABLED << R_ADC_B0_ADSGDCR0_DIAGVAL_Pos),

                         .limit_clip_interrupt_enable     = (0),
                         .virtual_channel_count           = 0 + (
                         ((1) == 1)+((1) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)),
#if (((1) == 1)||((1) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1))

                         .p_virtual_channels = (adc_b_virtual_channel_cfg_t**)group_0_virtual_channels,
                         #else
                         .p_virtual_channels = NULL,
                         #endif
                     };
                     #endif

                     #if (0) // Define Scan Group 1 if it's enabled
                     const adc_b_group_cfg_t adc_b_group_1_cfg =
                     {
                         .scan_group_id                   = ADC_GROUP_ID_1,
                         .converter_selection             = (0),
                         .scan_group_enable               = (0),
                         .scan_end_interrupt_enable       = (1),
                         .external_trigger_enable_mask    = ( ADC_B_EXTERNAL_TRIGGER_NONE),
                         .elc_trigger_enable_mask         = ( 0x00),
                         .gpt_trigger_enable_mask         = ( ADC_B_GPT_TRIGGER_NONE),

                         .self_diagnosis_mask             = (ADC_B_SELF_DIAGNOSIS_DISABLED << R_ADC_B0_ADSGDCR0_DIAGVAL_Pos),

                         .limit_clip_interrupt_enable     = (1),
                         .virtual_channel_count           = 0 + (
                         ((1) == 2)+((1) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)),
#if (((1) == 2)||((1) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2))

                         .p_virtual_channels = (adc_b_virtual_channel_cfg_t**)group_1_virtual_channels,
                         #else
                         .p_virtual_channels = NULL,
                         #endif
                     };
                     #endif

                     #if (0) // Define Scan Group 2 if it's enabled
                     const adc_b_group_cfg_t adc_b_group_2_cfg =
                     {
                         .scan_group_id                   = ADC_GROUP_ID_2,
                         .converter_selection             = (0),
                         .scan_group_enable               = (0),
                         .scan_end_interrupt_enable       = (1),
                         .external_trigger_enable_mask    = ( ADC_B_EXTERNAL_TRIGGER_NONE),
                         .elc_trigger_enable_mask         = ( 0x00),
                         .gpt_trigger_enable_mask         = ( ADC_B_GPT_TRIGGER_NONE),

                         .self_diagnosis_mask             = (ADC_B_SELF_DIAGNOSIS_DISABLED << R_ADC_B0_ADSGDCR0_DIAGVAL_Pos),

                         .limit_clip_interrupt_enable     = (1),
                         .virtual_channel_count           = 0 + (
                         ((1) == 3)+((1) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)),
#if (((1) == 3)||((1) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3))

                         .p_virtual_channels = (adc_b_virtual_channel_cfg_t**)group_2_virtual_channels,
                         #else
                         .p_virtual_channels = NULL,
                         #endif
                     };
                     #endif

                     #if (0) // Define Scan Group 3 if it's enabled
                     const adc_b_group_cfg_t adc_b_group_3_cfg =
                     {
                         .scan_group_id                   = ADC_GROUP_ID_3,
                         .converter_selection             = (0),
                         .scan_group_enable               = (0),
                         .scan_end_interrupt_enable       = (1),
                         .external_trigger_enable_mask    = ( ADC_B_EXTERNAL_TRIGGER_NONE),
                         .elc_trigger_enable_mask         = ( 0x00),
                         .gpt_trigger_enable_mask         = ( ADC_B_GPT_TRIGGER_NONE),

                         .self_diagnosis_mask             = (ADC_B_SELF_DIAGNOSIS_DISABLED << R_ADC_B0_ADSGDCR0_DIAGVAL_Pos),

                         .limit_clip_interrupt_enable     = (1),
                         .virtual_channel_count           = 0 + (
                         ((1) == 4)+((1) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)),
#if (((1) == 4)||((1) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4))

                         .p_virtual_channels = (adc_b_virtual_channel_cfg_t**)group_3_virtual_channels,
                         #else
                         .p_virtual_channels = NULL,
                         #endif
                     };
                     #endif

                     #if (0) // Define Scan Group 4 if it's enabled
                     const adc_b_group_cfg_t adc_b_group_4_cfg =
                     {
                         .scan_group_id                   = ADC_GROUP_ID_4,
                         .converter_selection             = (0),
                         .scan_group_enable               = (0),
                         .scan_end_interrupt_enable       = (1),
                         .external_trigger_enable_mask    = ( ADC_B_EXTERNAL_TRIGGER_NONE),
                         .elc_trigger_enable_mask         = ( 0x00),
                         .gpt_trigger_enable_mask         = ( ADC_B_GPT_TRIGGER_NONE),

                         .self_diagnosis_mask             = (ADC_B_SELF_DIAGNOSIS_DISABLED << R_ADC_B0_ADSGDCR0_DIAGVAL_Pos),

                         .limit_clip_interrupt_enable     = (1),
                         .virtual_channel_count           = 0 + (
                         ((1) == 5)+((1) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)),
#if (((1) == 5)||((1) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5))

                         .p_virtual_channels = (adc_b_virtual_channel_cfg_t**)group_4_virtual_channels,
                         #else
                         .p_virtual_channels = NULL,
                         #endif
                     };
                     #endif

                     #if (0) // Define Scan Group 5 if it's enabled
                     const adc_b_group_cfg_t adc_b_group_5_cfg =
                     {
                         .scan_group_id                   = ADC_GROUP_ID_5,
                         .converter_selection             = (0),
                         .scan_group_enable               = (0),
                         .scan_end_interrupt_enable       = (1),
                         .external_trigger_enable_mask    = ( ADC_B_EXTERNAL_TRIGGER_NONE),
                         .elc_trigger_enable_mask         = ( 0x00),
                         .gpt_trigger_enable_mask         = ( ADC_B_GPT_TRIGGER_NONE),

                         .self_diagnosis_mask             = (ADC_B_SELF_DIAGNOSIS_DISABLED << R_ADC_B0_ADSGDCR0_DIAGVAL_Pos),

                         .limit_clip_interrupt_enable     = (1),
                         .virtual_channel_count           = 0 + (
                         ((1) == 6)+((1) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)),
#if (((1) == 6)||((1) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6))

                         .p_virtual_channels = (adc_b_virtual_channel_cfg_t**)group_5_virtual_channels,
                         #else
                         .p_virtual_channels = NULL,
                         #endif
                     };
                     #endif

                     #if (0) // Define Scan Group 6 if it's enabled
                     const adc_b_group_cfg_t adc_b_group_6_cfg =
                     {
                         .scan_group_id                   = ADC_GROUP_ID_6,
                         .converter_selection             = (0),
                         .scan_group_enable               = (0),
                         .scan_end_interrupt_enable       = (1),
                         .external_trigger_enable_mask    = ( ADC_B_EXTERNAL_TRIGGER_NONE),
                         .elc_trigger_enable_mask         = ( 0x00),
                         .gpt_trigger_enable_mask         = ( ADC_B_GPT_TRIGGER_NONE),

                         .self_diagnosis_mask             = (ADC_B_SELF_DIAGNOSIS_DISABLED << R_ADC_B0_ADSGDCR0_DIAGVAL_Pos),

                         .limit_clip_interrupt_enable     = (1),
                         .virtual_channel_count           = 0 + (
                         ((1) == 7)+((1) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)),
#if (((1) == 7)||((1) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7))

                         .p_virtual_channels = (adc_b_virtual_channel_cfg_t**)group_6_virtual_channels,
                         #else
                         .p_virtual_channels = NULL,
                         #endif
                     };
                     #endif

                     #if (0) // Define Scan Group 7 if it's enabled
                     const adc_b_group_cfg_t adc_b_group_7_cfg =
                     {
                         .scan_group_id                   = ADC_GROUP_ID_7,
                         .converter_selection             = (0),
                         .scan_group_enable               = (0),
                         .scan_end_interrupt_enable       = (1),
                         .external_trigger_enable_mask    = ( ADC_B_EXTERNAL_TRIGGER_NONE),
                         .elc_trigger_enable_mask         = ( 0x00),
                         .gpt_trigger_enable_mask         = ( ADC_B_GPT_TRIGGER_NONE),

                         .self_diagnosis_mask             = (ADC_B_SELF_DIAGNOSIS_DISABLED << R_ADC_B0_ADSGDCR0_DIAGVAL_Pos),

                         .limit_clip_interrupt_enable     = (1),
                         .virtual_channel_count           = 0 + (
                         ((1) == 8)+((1) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)),
#if (((1) == 8)||((1) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8))

                         .p_virtual_channels = (adc_b_virtual_channel_cfg_t**)group_7_virtual_channels,
                         #else
                         .p_virtual_channels = NULL,
                         #endif
                     };
                     #endif

                     #if (0) // Define Scan Group 8 if it's enabled
                     const adc_b_group_cfg_t adc_b_group_8_cfg =
                     {
                         .scan_group_id                   = ADC_GROUP_ID_8,
                         .converter_selection             = (0),
                         .scan_group_enable               = (0),
                         .scan_end_interrupt_enable       = (1),
                         .external_trigger_enable_mask    = ( ADC_B_EXTERNAL_TRIGGER_NONE),
                         .elc_trigger_enable_mask         = ( 0x00),
                         .gpt_trigger_enable_mask         = ( ADC_B_GPT_TRIGGER_NONE),

                         .self_diagnosis_mask             = (ADC_B_SELF_DIAGNOSIS_DISABLED << R_ADC_B0_ADSGDCR0_DIAGVAL_Pos),

                         .limit_clip_interrupt_enable     = (1),
                         .virtual_channel_count           = 0 + (
                         ((1) == 9)+((1) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)),
#if (((1) == 9)||((1) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9))

                         .p_virtual_channels = (adc_b_virtual_channel_cfg_t**)group_8_virtual_channels,
                         #else
                         .p_virtual_channels = NULL,
                         #endif
                     };
                     #endif
#if ((1)||(0)||(0)||(0)||(0)||(0)||(0)||(0)||(0))
const adc_b_group_cfg_t * const adc_b_scan_cfg_groups[] = {
                           #if (0 != (1))
                           &adc_b_group_0_cfg,
                           #endif

                           #if (0 != (0))
                           &adc_b_group_1_cfg,
                           #endif

                           #if (0 != (0))
                           &adc_b_group_2_cfg,
                           #endif

                           #if (0 != (0))
                           &adc_b_group_3_cfg,
                           #endif

                           #if (0 != (0))
                           &adc_b_group_4_cfg,
                           #endif

                           #if (0 != (0))
                           &adc_b_group_5_cfg,
                           #endif

                           #if (0 != (0))
                           &adc_b_group_6_cfg,
                           #endif

                           #if (0 != (0))
                           &adc_b_group_7_cfg,
                           #endif

                           #if (0 != (0))
                           &adc_b_group_8_cfg,
                           #endif
                           };
                           #endif

                       const adc_b_scan_cfg_t g_adc_scan_cfg =
                       {
                       .group_count = ( 0 +
                       (0 != (1)) + (0 != (0)) + (0 != (0)) + (0 != (0)) + (0 != (0)) + (0 != (0)) + (0 != (0)) + (0 != (0)) + (0 != (0))),
#if ((0 != (1))||(0 != (0))||(0 != (0))||(0 != (0))||(0 != (0))||(0 != (0))||(0 != (0))||(0 != (0))||(0 != (0)))
                       .p_adc_groups = (adc_b_group_cfg_t**)adc_b_scan_cfg_groups,
                       #else
                       .p_adc_groups = NULL,
                       #endif
                       };

     const adc_b_isr_cfg_t g_adc_isr_cfg =
{
    .calibration_end_ipl_adc_0 =  (BSP_IRQ_DISABLED),
    .calibration_end_ipl_adc_1 =  (BSP_IRQ_DISABLED),
    .limit_clip_ipl           =  (BSP_IRQ_DISABLED),
    .conversion_error_ipl_adc_0 = (BSP_IRQ_DISABLED),
    .conversion_error_ipl_adc_1 = (BSP_IRQ_DISABLED),
    .overflow_error_ipl_adc_0 =   (BSP_IRQ_DISABLED),
    .overflow_error_ipl_adc_1 =   (BSP_IRQ_DISABLED),

    .scan_end_ipl_group_0 = (2),
    .scan_end_ipl_group_1 = (BSP_IRQ_DISABLED),
    .scan_end_ipl_group_2 = (BSP_IRQ_DISABLED),
    .scan_end_ipl_group_3 = (BSP_IRQ_DISABLED),
    .scan_end_ipl_group_4 = (BSP_IRQ_DISABLED),
    .scan_end_ipl_group_5678 = (BSP_IRQ_DISABLED),
    .fifo_overflow_ipl = (BSP_IRQ_DISABLED),
    .fifo_read_ipl_group_0 = (BSP_IRQ_DISABLED),
    .fifo_read_ipl_group_1 = (BSP_IRQ_DISABLED),
    .fifo_read_ipl_group_2 = (BSP_IRQ_DISABLED),
    .fifo_read_ipl_group_3 = (BSP_IRQ_DISABLED),
    .fifo_read_ipl_group_4 = (BSP_IRQ_DISABLED),
    .fifo_read_ipl_group_5678 = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_ADC12_CALEND0)
    .calibration_end_irq_adc_0 = VECTOR_NUMBER_ADC12_CALEND0,
#else
    .calibration_end_irq_adc_0 = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_ADC12_CALEND1)
    .calibration_end_irq_adc_1 = VECTOR_NUMBER_ADC12_CALEND1,
#else
    .calibration_end_irq_adc_1 = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_ADC12_LIMCLPI)
    .limit_clip_irq = VECTOR_NUMBER_ADC12_LIMCLPI,
#else
    .limit_clip_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_ADC12_ERR0)
    .conversion_error_irq_adc_0 = VECTOR_NUMBER_ADC12_ERR0,
#else
    .conversion_error_irq_adc_0 = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_ADC12_ERR1)
    .conversion_error_irq_adc_1 = VECTOR_NUMBER_ADC12_ERR1,
#else
    .conversion_error_irq_adc_1 = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_ADC12_RESOVF0)
    .overflow_error_irq_adc_0 = VECTOR_NUMBER_ADC12_RESOVF0,
#else
    .overflow_error_irq_adc_0 = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_ADC12_RESOVF1)
    .overflow_error_irq_adc_1 = VECTOR_NUMBER_ADC12_RESOVF1,
#else
    .overflow_error_irq_adc_1 = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_ADC12_ADI0)
    .scan_end_irq_group_0 = VECTOR_NUMBER_ADC12_ADI0,
#else
    .scan_end_irq_group_0 = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_ADC12_ADI1)
    .scan_end_irq_group_1 = VECTOR_NUMBER_ADC12_ADI1,
#else
    .scan_end_irq_group_1 = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_ADC12_ADI2)
    .scan_end_irq_group_2 = VECTOR_NUMBER_ADC12_ADI2,
#else
    .scan_end_irq_group_2 = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_ADC12_ADI3)
    .scan_end_irq_group_3 = VECTOR_NUMBER_ADC12_ADI3,
#else
    .scan_end_irq_group_3 = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_ADC12_ADI4)
    .scan_end_irq_group_4 = VECTOR_NUMBER_ADC12_ADI4,
#else
    .scan_end_irq_group_4 = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_ADC12_ADI5678)
    .scan_end_irq_group_5678 = VECTOR_NUMBER_ADC12_ADI5678,
#else
    .scan_end_irq_group_5678 = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_ADC12_FIFOOVF)
    .fifo_overflow_irq = VECTOR_NUMBER_ADC12_FIFOOVF,
#else
    .fifo_overflow_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_ADC12_FIFOREQ0)
    .fifo_read_irq_group_0 = VECTOR_NUMBER_ADC12_FIFOREQ0,
#else
    .fifo_read_irq_group_0 = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_ADC12_FIFOREQ1)
    .fifo_read_irq_group_1 = VECTOR_NUMBER_ADC12_FIFOREQ1,
#else
    .fifo_read_irq_group_1 = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_ADC12_FIFOREQ2)
    .fifo_read_irq_group_2 = VECTOR_NUMBER_ADC12_FIFOREQ2,
#else
    .fifo_read_irq_group_2 = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_ADC12_FIFOREQ3)
    .fifo_read_irq_group_3 = VECTOR_NUMBER_ADC12_FIFOREQ3,
#else
    .fifo_read_irq_group_3 = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_ADC12_FIFOREQ4)
    .fifo_read_irq_group_4 = VECTOR_NUMBER_ADC12_FIFOREQ4,
#else
    .fifo_read_irq_group_4 = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_ADC12_FIFOREQ5678)
    .fifo_read_irq_group_5678 = VECTOR_NUMBER_ADC12_FIFOREQ5678,
#else
    .fifo_read_irq_group_5678 = FSP_INVALID_VECTOR,
#endif
};

const adc_b_extended_cfg_t g_adc_cfg_extend =
{
    .clock_control_data     = ((ADC_B_CLOCK_SOURCE_PCLKC << R_ADC_B0_ADCLKCR_CLKSEL_Pos) |
                               (ADC_B_CLOCK_DIV_1 << R_ADC_B0_ADCLKCR_DIVR_Pos)),
    .sync_operation_control = (((1) << R_ADC_B0_ADSYCR_ADSYDIS0_Pos) |
                               ((1) << R_ADC_B0_ADSYCR_ADSYDIS1_Pos) |
                               (100 << R_ADC_B0_ADSYCR_ADSYCYC_Pos)),
    .adc_b_mode =  (((0) << R_ADC_B0_ADMDR_ADMD0_Pos) | ((0) << R_ADC_B0_ADMDR_ADMD1_Pos)),
    .converter_selection_0 = (((0) << R_ADC_B0_ADSGCR0_SGADS0_Pos) |
                              ((0) << R_ADC_B0_ADSGCR0_SGADS1_Pos) |
                              ((0) << R_ADC_B0_ADSGCR0_SGADS2_Pos) |
                              ((0) << R_ADC_B0_ADSGCR0_SGADS3_Pos)),
    .converter_selection_1 = (((0) << R_ADC_B0_ADSGCR1_SGADS4_Pos) |
                              ((0) << R_ADC_B0_ADSGCR1_SGADS5_Pos) |
                              ((0) << R_ADC_B0_ADSGCR1_SGADS6_Pos) |
                              ((0) << R_ADC_B0_ADSGCR1_SGADS7_Pos)),
    .converter_selection_2 =  ((0) << R_ADC_B0_ADSGCR2_SGADS8_Pos),

    .fifo_enable_mask     = (((1) << R_ADC_B0_ADFIFOCR_FIFOEN0_Pos) |
                             ((0) << R_ADC_B0_ADFIFOCR_FIFOEN1_Pos) |
                             ((0) << R_ADC_B0_ADFIFOCR_FIFOEN2_Pos) |
                             ((0) << R_ADC_B0_ADFIFOCR_FIFOEN3_Pos) |
                             ((0) << R_ADC_B0_ADFIFOCR_FIFOEN4_Pos) |
                             ((0) << R_ADC_B0_ADFIFOCR_FIFOEN5_Pos) |
                             ((0) << R_ADC_B0_ADFIFOCR_FIFOEN6_Pos) |
                             ((0) << R_ADC_B0_ADFIFOCR_FIFOEN7_Pos) |
                             ((0) << R_ADC_B0_ADFIFOCR_FIFOEN8_Pos)),
    .fifo_interrupt_enable_mask = (((0) << R_ADC_B0_ADFIFOINTCR_FIFOIE0_Pos) |
                                   ((0) << R_ADC_B0_ADFIFOINTCR_FIFOIE1_Pos) |
                                   ((0) << R_ADC_B0_ADFIFOINTCR_FIFOIE2_Pos) |
                                   ((0) << R_ADC_B0_ADFIFOINTCR_FIFOIE3_Pos) |
                                   ((0) << R_ADC_B0_ADFIFOINTCR_FIFOIE4_Pos) |
                                   ((0) << R_ADC_B0_ADFIFOINTCR_FIFOIE5_Pos) |
                                   ((0) << R_ADC_B0_ADFIFOINTCR_FIFOIE6_Pos) |
                                   ((0) << R_ADC_B0_ADFIFOINTCR_FIFOIE7_Pos) |
                                   ((0) << R_ADC_B0_ADFIFOINTCR_FIFOIE8_Pos)),
    .fifo_interrupt_level0 = ((0 << R_ADC_B0_ADFIFOINTLR0_FIFOILV0_Pos) |
                              (0 << R_ADC_B0_ADFIFOINTLR0_FIFOILV1_Pos)),
    .fifo_interrupt_level1 = ((0 << R_ADC_B0_ADFIFOINTLR1_FIFOILV2_Pos) |
                              (0 << R_ADC_B0_ADFIFOINTLR1_FIFOILV3_Pos)),
    .fifo_interrupt_level2 = ((0 << R_ADC_B0_ADFIFOINTLR2_FIFOILV4_Pos) |
                              (0 << R_ADC_B0_ADFIFOINTLR2_FIFOILV5_Pos)),
    .fifo_interrupt_level3 = ((0 << R_ADC_B0_ADFIFOINTLR3_FIFOILV6_Pos) |
                              (0 << R_ADC_B0_ADFIFOINTLR3_FIFOILV7_Pos)),
    .fifo_interrupt_level4 = (0 << R_ADC_B0_ADFIFOINTLR4_FIFOILV8_Pos),

    .start_trigger_delay_0 = ((0 << R_ADC_B0_ADTRGDLR0_TRGDLY0_Pos) |
                              (0 << R_ADC_B0_ADTRGDLR0_TRGDLY1_Pos)),
    .start_trigger_delay_1 = ((0 << R_ADC_B0_ADTRGDLR1_TRGDLY2_Pos) |
                              (0 << R_ADC_B0_ADTRGDLR1_TRGDLY3_Pos)),
    .start_trigger_delay_2 = ((0 << R_ADC_B0_ADTRGDLR2_TRGDLY4_Pos) |
                              (0 << R_ADC_B0_ADTRGDLR2_TRGDLY5_Pos)),
    .start_trigger_delay_3 = ((0 << R_ADC_B0_ADTRGDLR3_TRGDLY6_Pos) |
                              (0 << R_ADC_B0_ADTRGDLR3_TRGDLY7_Pos)),
    .start_trigger_delay_4 = ((0 << R_ADC_B0_ADTRGDLR4_TRGDLY8_Pos)),
    .calibration_adc_state = ((95 << R_ADC_B0_ADCALSTCR_CALADSST_Pos) |
                              (5 << R_ADC_B0_ADCALSTCR_CALADCST_Pos)),
    .calibration_sample_and_hold = ((95 << R_ADC_B0_ADCALSHCR_CALSHSST_Pos) |
                                    (5 << R_ADC_B0_ADCALSHCR_CALSHHST_Pos)),
   .p_isr_cfg = &g_adc_isr_cfg,
   .sampling_state_tables = {
                            ((95  << R_ADC_B0_ADSSTR0_SST0_Pos)  | (95  << R_ADC_B0_ADSSTR0_SST1_Pos)),
                            ((95  << R_ADC_B0_ADSSTR1_SST2_Pos)  | (95  << R_ADC_B0_ADSSTR1_SST3_Pos)),
                            ((95  << R_ADC_B0_ADSSTR2_SST4_Pos)  | (95  << R_ADC_B0_ADSSTR2_SST5_Pos)),
                            ((95  << R_ADC_B0_ADSSTR3_SST6_Pos)  | (95  << R_ADC_B0_ADSSTR3_SST7_Pos)),
                            ((95  << R_ADC_B0_ADSSTR4_SST8_Pos)  | (95  << R_ADC_B0_ADSSTR4_SST9_Pos)),
                            ((95 << R_ADC_B0_ADSSTR5_SST10_Pos) | (95 << R_ADC_B0_ADSSTR5_SST11_Pos)),
                            ((95 << R_ADC_B0_ADSSTR6_SST12_Pos) | (95 << R_ADC_B0_ADSSTR6_SST13_Pos)),
                            ((95 << R_ADC_B0_ADSSTR7_SST14_Pos) | (95 << R_ADC_B0_ADSSTR7_SST15_Pos)),
                            },
    .sample_and_hold_enable_mask = ( ADC_B_SAMPLE_AND_HOLD_MASK_NONE),
    .sample_and_hold_config_012 = ((95 << R_ADC_B0_ADSHSTR1_SHSST_Pos )|
                                   (5 << R_ADC_B0_ADSHSTR0_SHHST_Pos )),
    .sample_and_hold_config_456 = ((95 << R_ADC_B0_ADSHSTR1_SHSST_Pos) |
                                   (5 << R_ADC_B0_ADSHSTR1_SHHST_Pos)),
    .conversion_state = ((5 << R_ADC_B0_ADCNVSTR_CST0_Pos) |
                        (5 << R_ADC_B0_ADCNVSTR_CST1_Pos)),
    .user_offset_tables = {
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          },
    .user_gain_tables = {
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        },
    .limiter_clip_interrupt_enable_mask = ( 0x00),
    .limiter_clip_tables = {
                           (0 | 0 << R_ADC_B0_ADLIMTR0_LIMU_Pos),
                           (0 | 0 << R_ADC_B0_ADLIMTR1_LIMU_Pos),
                           (0 | 0 << R_ADC_B0_ADLIMTR2_LIMU_Pos),
                           (0 | 0 << R_ADC_B0_ADLIMTR3_LIMU_Pos),
                           (0 | 0 << R_ADC_B0_ADLIMTR4_LIMU_Pos),
                           (0 | 0 << R_ADC_B0_ADLIMTR5_LIMU_Pos),
                           (0 | 0 << R_ADC_B0_ADLIMTR6_LIMU_Pos),
                           (0 | 0 << R_ADC_B0_ADLIMTR7_LIMU_Pos),
                           },

    #if (1 == 0)
    .pga_gain[0] = ADC_B_PGA_GAIN_SINGLE_ENDED_2_500,
    #elif (2 == 0)
    .pga_gain[0] = ADC_B_PGA_GAIN_DIFFERENTIAL_1_500,
    #else
    .pga_gain[0] = ADC_B_PGA_GAIN_DISABLED,
    #endif

    #if (1 == 0)
    .pga_gain[1] = ADC_B_PGA_GAIN_SINGLE_ENDED_2_500,
    #elif (2 == 0)
    .pga_gain[1] = ADC_B_PGA_GAIN_DIFFERENTIAL_1_500,
    #else
    .pga_gain[1] = ADC_B_PGA_GAIN_DISABLED,
    #endif

    #if (1 == 0)
    .pga_gain[2] = ADC_B_PGA_GAIN_SINGLE_ENDED_2_500,
    #elif (2 == 0)
    .pga_gain[2] = ADC_B_PGA_GAIN_DIFFERENTIAL_1_500,
    #else
    .pga_gain[2] = ADC_B_PGA_GAIN_DISABLED,
    #endif

    #if (1 == 0)
    .pga_gain[3] = ADC_B_PGA_GAIN_SINGLE_ENDED_2_500,
    #elif (2 == 0)
    .pga_gain[3] = ADC_B_PGA_GAIN_DIFFERENTIAL_1_500,
    #else
    .pga_gain[3] = ADC_B_PGA_GAIN_DISABLED,
    #endif

    /* For debug only! Prolonged use of PGA Monitor function may deteriorate PGA characteristics. See user manual for more information.*/
    .pga_debug_monitor_mask_b.unit_0 = 0,
    .pga_debug_monitor_mask_b.unit_1 = 0,
    .pga_debug_monitor_mask_b.unit_2 = 0,
    .pga_debug_monitor_mask_b.unit_3 = 0,
};


const adc_cfg_t g_adc_cfg =
{
    .unit                = 0xFFFC,
    .mode                = (adc_mode_t) 0, // Unused
    .resolution          = (adc_resolution_t) 0, // Unused
    .alignment           = ADC_ALIGNMENT_RIGHT,
    .trigger             = (adc_trigger_t) 0, // Unused
    .p_callback          = CS_ADC_IRQHandler,
#if defined(NULL)
    .p_context           = NULL,
#else
    .p_context           = &NULL,
#endif
    .p_extend            = &g_adc_cfg_extend,

    .scan_end_irq        = FSP_INVALID_VECTOR,
    .scan_end_ipl        = BSP_IRQ_DISABLED,
    .scan_end_b_irq      = FSP_INVALID_VECTOR,
    .scan_end_b_ipl      = BSP_IRQ_DISABLED,
};


adc_b_instance_ctrl_t g_adc_ctrl;


const adc_instance_t g_adc =
{
    .p_ctrl    = &g_adc_ctrl,
    .p_cfg = &g_adc_cfg,
    .p_channel_cfg = &g_adc_scan_cfg,
    .p_api     = &g_adc_on_adc_b,
};


    #undef RA_NOT_DEFINED
gpt_instance_ctrl_t g_timer6_ctrl;
#if 1
const gpt_extended_pwm_cfg_t g_timer6_pwm_extend =
{
    .trough_ipl          = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT6_COUNTER_UNDERFLOW)
    .trough_irq          = VECTOR_NUMBER_GPT6_COUNTER_UNDERFLOW,
#else
    .trough_irq          = FSP_INVALID_VECTOR,
#endif
    .poeg_link           = GPT_POEG_LINK_POEG1,
    .output_disable      =  GPT_OUTPUT_DISABLE_NONE,
    .adc_trigger         =  GPT_ADC_TRIGGER_NONE,
    .dead_time_count_up  = 50,
    .dead_time_count_down = 50,
    .adc_a_compare_match = 0,
    .adc_b_compare_match = 0,
    .interrupt_skip_source = GPT_INTERRUPT_SKIP_SOURCE_NONE,
    .interrupt_skip_count  = GPT_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_adc    = GPT_INTERRUPT_SKIP_ADC_NONE,
    .gtioca_disable_setting = GPT_GTIOC_DISABLE_SET_HI_Z,
    .gtiocb_disable_setting = GPT_GTIOC_DISABLE_SET_HI_Z,
};
#endif
const gpt_extended_cfg_t g_timer6_extend =
{
    .gtioca = { .output_enabled = true,
                .stop_level     = GPT_PIN_LEVEL_LOW
              },
    .gtiocb = { .output_enabled = true,
                .stop_level     = GPT_PIN_LEVEL_LOW
              },
    .start_source        = (gpt_source_t) ( GPT_SOURCE_NONE),
    .stop_source         = (gpt_source_t) ( GPT_SOURCE_NONE),
    .clear_source        = (gpt_source_t) ( GPT_SOURCE_NONE),
    .count_up_source     = (gpt_source_t) ( GPT_SOURCE_NONE),
    .count_down_source   = (gpt_source_t) ( GPT_SOURCE_NONE),
    .capture_a_source    = (gpt_source_t) ( GPT_SOURCE_NONE),
    .capture_b_source    = (gpt_source_t) ( GPT_SOURCE_NONE),
    .capture_a_ipl       = (BSP_IRQ_DISABLED),
    .capture_b_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT6_CAPTURE_COMPARE_A)
    .capture_a_irq       = VECTOR_NUMBER_GPT6_CAPTURE_COMPARE_A,
#else
    .capture_a_irq       = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_GPT6_CAPTURE_COMPARE_B)
    .capture_b_irq       = VECTOR_NUMBER_GPT6_CAPTURE_COMPARE_B,
#else
    .capture_b_irq       = FSP_INVALID_VECTOR,
#endif
    .capture_filter_gtioca       = GPT_CAPTURE_FILTER_NONE,
    .capture_filter_gtiocb       = GPT_CAPTURE_FILTER_NONE,
#if 1
    .p_pwm_cfg                   = &g_timer6_pwm_extend,
#else
    .p_pwm_cfg                   = NULL,
#endif
#if 1
    .gtior_setting.gtior_b.gtioa  = (0U << 4U) | (1U << 2U) | (3U << 0U),
    .gtior_setting.gtior_b.oadflt = (uint32_t) GPT_PIN_LEVEL_LOW,
    .gtior_setting.gtior_b.oahld  = 0U,
    .gtior_setting.gtior_b.oae    = (uint32_t) true,
    .gtior_setting.gtior_b.oadf   = (uint32_t) GPT_GTIOC_DISABLE_SET_HI_Z,
    .gtior_setting.gtior_b.nfaen  = ((uint32_t) GPT_CAPTURE_FILTER_NONE & 1U),
    .gtior_setting.gtior_b.nfcsa  = ((uint32_t) GPT_CAPTURE_FILTER_NONE >> 1U),
    .gtior_setting.gtior_b.gtiob  = (1U << 4U) | (2U << 2U) | (3U << 0U),
    .gtior_setting.gtior_b.obdflt = (uint32_t) GPT_PIN_LEVEL_LOW,
    .gtior_setting.gtior_b.obhld  = 0U,
    .gtior_setting.gtior_b.obe    = (uint32_t) true,
    .gtior_setting.gtior_b.obdf   = (uint32_t) GPT_GTIOC_DISABLE_SET_HI_Z,
    .gtior_setting.gtior_b.nfben  = ((uint32_t) GPT_CAPTURE_FILTER_NONE & 1U),
    .gtior_setting.gtior_b.nfcsb  = ((uint32_t) GPT_CAPTURE_FILTER_NONE >> 1U),
#else
    .gtior_setting.gtior = 0U,
#endif
};
const timer_cfg_t g_timer6_cfg =
{
    .mode                = TIMER_MODE_TRIANGLE_WAVE_SYMMETRIC_PWM,
    /* Actual period: 0.00016666666666666666 seconds. Actual duty: 50%. */ .period_counts = (uint32_t) 0x2710, .duty_cycle_counts = 0x1388, .source_div = (timer_source_div_t)0,
    .channel             = 6,
    .p_callback          = NULL,
    /** If NULL then do not add & */
#if defined(NULL)
    .p_context           = NULL,
#else
    .p_context           = &NULL,
#endif
    .p_extend            = &g_timer6_extend,
    .cycle_end_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT6_COUNTER_OVERFLOW)
    .cycle_end_irq       = VECTOR_NUMBER_GPT6_COUNTER_OVERFLOW,
#else
    .cycle_end_irq       = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const timer_instance_t g_timer6 =
{
    .p_ctrl        = &g_timer6_ctrl,
    .p_cfg         = &g_timer6_cfg,
    .p_api         = &g_timer_on_gpt
};
gpt_instance_ctrl_t g_timer5_ctrl;
#if 1
const gpt_extended_pwm_cfg_t g_timer5_pwm_extend =
{
    .trough_ipl          = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT5_COUNTER_UNDERFLOW)
    .trough_irq          = VECTOR_NUMBER_GPT5_COUNTER_UNDERFLOW,
#else
    .trough_irq          = FSP_INVALID_VECTOR,
#endif
    .poeg_link           = GPT_POEG_LINK_POEG1,
    .output_disable      =  GPT_OUTPUT_DISABLE_NONE,
    .adc_trigger         =  GPT_ADC_TRIGGER_NONE,
    .dead_time_count_up  = 50,
    .dead_time_count_down = 50,
    .adc_a_compare_match = 0,
    .adc_b_compare_match = 0,
    .interrupt_skip_source = GPT_INTERRUPT_SKIP_SOURCE_NONE,
    .interrupt_skip_count  = GPT_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_adc    = GPT_INTERRUPT_SKIP_ADC_NONE,
    .gtioca_disable_setting = GPT_GTIOC_DISABLE_SET_HI_Z,
    .gtiocb_disable_setting = GPT_GTIOC_DISABLE_SET_HI_Z,
};
#endif
const gpt_extended_cfg_t g_timer5_extend =
{
    .gtioca = { .output_enabled = true,
                .stop_level     = GPT_PIN_LEVEL_LOW
              },
    .gtiocb = { .output_enabled = true,
                .stop_level     = GPT_PIN_LEVEL_LOW
              },
    .start_source        = (gpt_source_t) ( GPT_SOURCE_NONE),
    .stop_source         = (gpt_source_t) ( GPT_SOURCE_NONE),
    .clear_source        = (gpt_source_t) ( GPT_SOURCE_NONE),
    .count_up_source     = (gpt_source_t) ( GPT_SOURCE_NONE),
    .count_down_source   = (gpt_source_t) ( GPT_SOURCE_NONE),
    .capture_a_source    = (gpt_source_t) ( GPT_SOURCE_NONE),
    .capture_b_source    = (gpt_source_t) ( GPT_SOURCE_NONE),
    .capture_a_ipl       = (BSP_IRQ_DISABLED),
    .capture_b_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT5_CAPTURE_COMPARE_A)
    .capture_a_irq       = VECTOR_NUMBER_GPT5_CAPTURE_COMPARE_A,
#else
    .capture_a_irq       = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_GPT5_CAPTURE_COMPARE_B)
    .capture_b_irq       = VECTOR_NUMBER_GPT5_CAPTURE_COMPARE_B,
#else
    .capture_b_irq       = FSP_INVALID_VECTOR,
#endif
    .capture_filter_gtioca       = GPT_CAPTURE_FILTER_NONE,
    .capture_filter_gtiocb       = GPT_CAPTURE_FILTER_NONE,
#if 1
    .p_pwm_cfg                   = &g_timer5_pwm_extend,
#else
    .p_pwm_cfg                   = NULL,
#endif
#if 1
    .gtior_setting.gtior_b.gtioa  = (0U << 4U) | (1U << 2U) | (3U << 0U),
    .gtior_setting.gtior_b.oadflt = (uint32_t) GPT_PIN_LEVEL_LOW,
    .gtior_setting.gtior_b.oahld  = 0U,
    .gtior_setting.gtior_b.oae    = (uint32_t) true,
    .gtior_setting.gtior_b.oadf   = (uint32_t) GPT_GTIOC_DISABLE_SET_HI_Z,
    .gtior_setting.gtior_b.nfaen  = ((uint32_t) GPT_CAPTURE_FILTER_NONE & 1U),
    .gtior_setting.gtior_b.nfcsa  = ((uint32_t) GPT_CAPTURE_FILTER_NONE >> 1U),
    .gtior_setting.gtior_b.gtiob  = (1U << 4U) | (2U << 2U) | (3U << 0U),
    .gtior_setting.gtior_b.obdflt = (uint32_t) GPT_PIN_LEVEL_LOW,
    .gtior_setting.gtior_b.obhld  = 0U,
    .gtior_setting.gtior_b.obe    = (uint32_t) true,
    .gtior_setting.gtior_b.obdf   = (uint32_t) GPT_GTIOC_DISABLE_SET_HI_Z,
    .gtior_setting.gtior_b.nfben  = ((uint32_t) GPT_CAPTURE_FILTER_NONE & 1U),
    .gtior_setting.gtior_b.nfcsb  = ((uint32_t) GPT_CAPTURE_FILTER_NONE >> 1U),
#else
    .gtior_setting.gtior = 0U,
#endif
};
const timer_cfg_t g_timer5_cfg =
{
    .mode                = TIMER_MODE_TRIANGLE_WAVE_SYMMETRIC_PWM,
    /* Actual period: 0.00016666666666666666 seconds. Actual duty: 50%. */ .period_counts = (uint32_t) 0x2710, .duty_cycle_counts = 0x1388, .source_div = (timer_source_div_t)0,
    .channel             = 5,
    .p_callback          = NULL,
    /** If NULL then do not add & */
#if defined(NULL)
    .p_context           = NULL,
#else
    .p_context           = &NULL,
#endif
    .p_extend            = &g_timer5_extend,
    .cycle_end_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT5_COUNTER_OVERFLOW)
    .cycle_end_irq       = VECTOR_NUMBER_GPT5_COUNTER_OVERFLOW,
#else
    .cycle_end_irq       = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const timer_instance_t g_timer5 =
{
    .p_ctrl        = &g_timer5_ctrl,
    .p_cfg         = &g_timer5_cfg,
    .p_api         = &g_timer_on_gpt
};
gpt_instance_ctrl_t g_timer4_ctrl;
#if 1
const gpt_extended_pwm_cfg_t g_timer4_pwm_extend =
{
    .trough_ipl          = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT4_COUNTER_UNDERFLOW)
    .trough_irq          = VECTOR_NUMBER_GPT4_COUNTER_UNDERFLOW,
#else
    .trough_irq          = FSP_INVALID_VECTOR,
#endif
    .poeg_link           = GPT_POEG_LINK_POEG1,
    .output_disable      =  GPT_OUTPUT_DISABLE_NONE,
    .adc_trigger         = GPT_ADC_TRIGGER_DOWN_COUNT_START_ADC_A |  GPT_ADC_TRIGGER_NONE,
    .dead_time_count_up  = 50,
    .dead_time_count_down = 50,
    .adc_a_compare_match = 0,
    .adc_b_compare_match = 0,
    .interrupt_skip_source = GPT_INTERRUPT_SKIP_SOURCE_NONE,
    .interrupt_skip_count  = GPT_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_adc    = GPT_INTERRUPT_SKIP_ADC_NONE,
    .gtioca_disable_setting = GPT_GTIOC_DISABLE_SET_HI_Z,
    .gtiocb_disable_setting = GPT_GTIOC_DISABLE_SET_HI_Z,
};
#endif
const gpt_extended_cfg_t g_timer4_extend =
{
    .gtioca = { .output_enabled = true,
                .stop_level     = GPT_PIN_LEVEL_LOW
              },
    .gtiocb = { .output_enabled = true,
                .stop_level     = GPT_PIN_LEVEL_LOW
              },
    .start_source        = (gpt_source_t) ( GPT_SOURCE_NONE),
    .stop_source         = (gpt_source_t) ( GPT_SOURCE_NONE),
    .clear_source        = (gpt_source_t) ( GPT_SOURCE_NONE),
    .count_up_source     = (gpt_source_t) ( GPT_SOURCE_NONE),
    .count_down_source   = (gpt_source_t) ( GPT_SOURCE_NONE),
    .capture_a_source    = (gpt_source_t) ( GPT_SOURCE_NONE),
    .capture_b_source    = (gpt_source_t) ( GPT_SOURCE_NONE),
    .capture_a_ipl       = (BSP_IRQ_DISABLED),
    .capture_b_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT4_CAPTURE_COMPARE_A)
    .capture_a_irq       = VECTOR_NUMBER_GPT4_CAPTURE_COMPARE_A,
#else
    .capture_a_irq       = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_GPT4_CAPTURE_COMPARE_B)
    .capture_b_irq       = VECTOR_NUMBER_GPT4_CAPTURE_COMPARE_B,
#else
    .capture_b_irq       = FSP_INVALID_VECTOR,
#endif
    .capture_filter_gtioca       = GPT_CAPTURE_FILTER_NONE,
    .capture_filter_gtiocb       = GPT_CAPTURE_FILTER_NONE,
#if 1
    .p_pwm_cfg                   = &g_timer4_pwm_extend,
#else
    .p_pwm_cfg                   = NULL,
#endif
#if 1
    .gtior_setting.gtior_b.gtioa  = (0U << 4U) | (1U << 2U) | (3U << 0U),
    .gtior_setting.gtior_b.oadflt = (uint32_t) GPT_PIN_LEVEL_LOW,
    .gtior_setting.gtior_b.oahld  = 0U,
    .gtior_setting.gtior_b.oae    = (uint32_t) true,
    .gtior_setting.gtior_b.oadf   = (uint32_t) GPT_GTIOC_DISABLE_SET_HI_Z,
    .gtior_setting.gtior_b.nfaen  = ((uint32_t) GPT_CAPTURE_FILTER_NONE & 1U),
    .gtior_setting.gtior_b.nfcsa  = ((uint32_t) GPT_CAPTURE_FILTER_NONE >> 1U),
    .gtior_setting.gtior_b.gtiob  = (1U << 4U) | (2U << 2U) | (3U << 0U),
    .gtior_setting.gtior_b.obdflt = (uint32_t) GPT_PIN_LEVEL_LOW,
    .gtior_setting.gtior_b.obhld  = 0U,
    .gtior_setting.gtior_b.obe    = (uint32_t) true,
    .gtior_setting.gtior_b.obdf   = (uint32_t) GPT_GTIOC_DISABLE_SET_HI_Z,
    .gtior_setting.gtior_b.nfben  = ((uint32_t) GPT_CAPTURE_FILTER_NONE & 1U),
    .gtior_setting.gtior_b.nfcsb  = ((uint32_t) GPT_CAPTURE_FILTER_NONE >> 1U),
#else
    .gtior_setting.gtior = 0U,
#endif
};
const timer_cfg_t g_timer4_cfg =
{
    .mode                = TIMER_MODE_TRIANGLE_WAVE_SYMMETRIC_PWM,
    /* Actual period: 0.00016666666666666666 seconds. Actual duty: 50%. */ .period_counts = (uint32_t) 0x2710, .duty_cycle_counts = 0x1388, .source_div = (timer_source_div_t)0,
    .channel             = 4,
    .p_callback          = PWM_TIM_UP_IRQHandler,
    /** If NULL then do not add & */
#if defined(NULL)
    .p_context           = NULL,
#else
    .p_context           = &NULL,
#endif
    .p_extend            = &g_timer4_extend,
    .cycle_end_ipl       = (1),
#if defined(VECTOR_NUMBER_GPT4_COUNTER_OVERFLOW)
    .cycle_end_irq       = VECTOR_NUMBER_GPT4_COUNTER_OVERFLOW,
#else
    .cycle_end_irq       = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const timer_instance_t g_timer4 =
{
    .p_ctrl        = &g_timer4_ctrl,
    .p_cfg         = &g_timer4_cfg,
    .p_api         = &g_timer_on_gpt
};
gpt_three_phase_instance_ctrl_t g_three_phase0_ctrl;
const three_phase_cfg_t g_three_phase0_cfg =
{
    .buffer_mode         = (THREE_PHASE_BUFFER_MODE_SINGLE),
    .p_timer_instance    =
    {
        &g_timer4,
        &g_timer5,
        &g_timer6
    },
    .callback_ch         = THREE_PHASE_CHANNEL_U,
    .channel_mask        = (1 << 4) | (1 << 5) | (1 << 6),
    .p_context           = NULL,
    .p_extend            = NULL,
};
/* Instance structure to use this module. */
const three_phase_instance_t g_three_phase0 =
{
    .p_ctrl        = &g_three_phase0_ctrl,
    .p_cfg         = &g_three_phase0_cfg,
    .p_api         = &g_gpt_three_phase_on_gpt_three_phase
};
void g_hal_init(void) {
g_common_init();
}
