/* generated HAL source file - do not edit */
#include "hal_data.h"


/* Macros to tie dynamic ELC links to adc_b_trigger_sync_elc option in adc_b_trigger_t. */
                     #define ADC_B_TRIGGER_ADC_B0        ADC_B_TRIGGER_SYNC_ELC
                     #define ADC_B_TRIGGER_ADC_B0_B      ADC_B_TRIGGER_SYNC_ELC
                     #define ADC_B_TRIGGER_ADC_B1        ADC_B_TRIGGER_SYNC_ELC
                     #define ADC_B_TRIGGER_ADC_B1_B      ADC_B_TRIGGER_SYNC_ELC

dtc_instance_ctrl_t g_transfer1_ctrl;

transfer_info_t g_transfer1_info =
{
    .transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED,
    .transfer_settings_word_b.repeat_area    = TRANSFER_REPEAT_AREA_DESTINATION,
    .transfer_settings_word_b.irq            = TRANSFER_IRQ_END,
    .transfer_settings_word_b.chain_mode     = TRANSFER_CHAIN_MODE_DISABLED,
    .transfer_settings_word_b.src_addr_mode  = TRANSFER_ADDR_MODE_FIXED,
    .transfer_settings_word_b.size           = TRANSFER_SIZE_2_BYTE,
    .transfer_settings_word_b.mode           = TRANSFER_MODE_NORMAL,
    .p_dest                                  = (void *) NULL,
    .p_src                                   = (void const *) NULL,
    .num_blocks                              = 0,
    .length                                  = 0,
};

const dtc_extended_cfg_t g_transfer1_cfg_extend =
{
    .activation_source   = VECTOR_NUMBER_SPI1_RXI,
};
const transfer_cfg_t g_transfer1_cfg =
{
    .p_info              = &g_transfer1_info,
    .p_extend            = &g_transfer1_cfg_extend,
};

/* Instance structure to use this module. */
const transfer_instance_t g_transfer1 =
{
    .p_ctrl        = &g_transfer1_ctrl,
    .p_cfg         = &g_transfer1_cfg,
    .p_api         = &g_transfer_on_dtc
};
dtc_instance_ctrl_t g_transfer0_ctrl;

transfer_info_t g_transfer0_info =
{
    .transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED,
    .transfer_settings_word_b.repeat_area    = TRANSFER_REPEAT_AREA_SOURCE,
    .transfer_settings_word_b.irq            = TRANSFER_IRQ_END,
    .transfer_settings_word_b.chain_mode     = TRANSFER_CHAIN_MODE_DISABLED,
    .transfer_settings_word_b.src_addr_mode  = TRANSFER_ADDR_MODE_INCREMENTED,
    .transfer_settings_word_b.size           = TRANSFER_SIZE_2_BYTE,
    .transfer_settings_word_b.mode           = TRANSFER_MODE_NORMAL,
    .p_dest                                  = (void *) NULL,
    .p_src                                   = (void const *) NULL,
    .num_blocks                              = 0,
    .length                                  = 0,
};

const dtc_extended_cfg_t g_transfer0_cfg_extend =
{
    .activation_source   = VECTOR_NUMBER_SPI1_TXI,
};
const transfer_cfg_t g_transfer0_cfg =
{
    .p_info              = &g_transfer0_info,
    .p_extend            = &g_transfer0_cfg_extend,
};

/* Instance structure to use this module. */
const transfer_instance_t g_transfer0 =
{
    .p_ctrl        = &g_transfer0_ctrl,
    .p_cfg         = &g_transfer0_cfg,
    .p_api         = &g_transfer_on_dtc
};
spi_b_instance_ctrl_t g_spi1_ctrl;

/** SPI extended configuration for SPI HAL driver */
const spi_b_extended_cfg_t g_spi1_ext_cfg =
{
    .spi_clksyn         = SPI_B_SSL_MODE_CLK_SYN,
    .spi_comm           = SPI_B_COMMUNICATION_FULL_DUPLEX,
    .ssl_polarity        = SPI_B_SSLP_LOW,
    .ssl_select          = SPI_B_SSL_SELECT_SSL0,
    .mosi_idle           = SPI_B_MOSI_IDLE_VALUE_FIXING_DISABLE,
    .parity              = SPI_B_PARITY_MODE_DISABLE,
    .byte_swap           = SPI_B_BYTE_SWAP_DISABLE,
    .clock_source        = SPI_B_CLOCK_SOURCE_SCISPICLK,
    .spck_div            = {
        /* Actual calculated bitrate: 15000000. */ .spbr = 0, .brdv = 0
    },
    .spck_delay          = SPI_B_DELAY_COUNT_1,
    .ssl_negation_delay  = SPI_B_DELAY_COUNT_1,
    .next_access_delay   = SPI_B_DELAY_COUNT_1,

 };

/** SPI configuration for SPI HAL driver */
const spi_cfg_t g_spi1_cfg =
{
    .channel             = 1,

#if defined(VECTOR_NUMBER_SPI1_RXI)
    .rxi_irq             = VECTOR_NUMBER_SPI1_RXI,
#else
    .rxi_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SPI1_TXI)
    .txi_irq             = VECTOR_NUMBER_SPI1_TXI,
#else
    .txi_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SPI1_TEI)
    .tei_irq             = VECTOR_NUMBER_SPI1_TEI,
#else
    .tei_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SPI1_ERI)
    .eri_irq             = VECTOR_NUMBER_SPI1_ERI,
#else
    .eri_irq             = FSP_INVALID_VECTOR,
#endif

    .rxi_ipl             = (2),
    .txi_ipl             = (2),
    .tei_ipl             = (2),
    .eri_ipl             = (2),

    .operating_mode      = SPI_MODE_MASTER,

    .clk_phase           = SPI_CLK_PHASE_EDGE_EVEN,
    .clk_polarity        = SPI_CLK_POLARITY_HIGH,

    .mode_fault          = SPI_MODE_FAULT_ERROR_DISABLE,
    .bit_order           = SPI_BIT_ORDER_MSB_FIRST,
    .p_transfer_tx       = g_spi1_P_TRANSFER_TX,
    .p_transfer_rx       = g_spi1_P_TRANSFER_RX,
    .p_callback          = spi_callback,

    .p_context           = NULL,
    .p_extend            = (void *)&g_spi1_ext_cfg,
};

/* Instance structure to use this module. */
const spi_instance_t g_spi1 =
{
    .p_ctrl        = &g_spi1_ctrl,
    .p_cfg         = &g_spi1_cfg,
    .p_api         = &g_spi_on_spi_b
};
gpt_instance_ctrl_t g_timer9_ctrl;
#if 0
const gpt_extended_pwm_cfg_t g_timer9_pwm_extend =
{
    .trough_ipl          = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT9_COUNTER_UNDERFLOW)
    .trough_irq          = VECTOR_NUMBER_GPT9_COUNTER_UNDERFLOW,
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
const gpt_extended_cfg_t g_timer9_extend =
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
#if defined(VECTOR_NUMBER_GPT9_CAPTURE_COMPARE_A)
    .capture_a_irq       = VECTOR_NUMBER_GPT9_CAPTURE_COMPARE_A,
#else
    .capture_a_irq       = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_GPT9_CAPTURE_COMPARE_B)
    .capture_b_irq       = VECTOR_NUMBER_GPT9_CAPTURE_COMPARE_B,
#else
    .capture_b_irq       = FSP_INVALID_VECTOR,
#endif
    .capture_filter_gtioca       = GPT_CAPTURE_FILTER_NONE,
    .capture_filter_gtiocb       = GPT_CAPTURE_FILTER_NONE,
#if 0
    .p_pwm_cfg                   = &g_timer9_pwm_extend,
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
const timer_cfg_t g_timer9_cfg =
{
    .mode                = TIMER_MODE_PERIODIC,
    /* Actual period: 35.791394133333334 seconds. Actual duty: 50%. */ .period_counts = (uint32_t) 0x100000000, .duty_cycle_counts = 0x80000000, .source_div = (timer_source_div_t)0,
    .channel             = 9,
    .p_callback          = WheelSpeedTimer_IRQHandler,
    /** If NULL then do not add & */
#if defined(NULL)
    .p_context           = NULL,
#else
    .p_context           = &NULL,
#endif
    .p_extend            = &g_timer9_extend,
    .cycle_end_ipl       = (12),
#if defined(VECTOR_NUMBER_GPT9_COUNTER_OVERFLOW)
    .cycle_end_irq       = VECTOR_NUMBER_GPT9_COUNTER_OVERFLOW,
#else
    .cycle_end_irq       = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const timer_instance_t g_timer9 =
{
    .p_ctrl        = &g_timer9_ctrl,
    .p_cfg         = &g_timer9_cfg,
    .p_api         = &g_timer_on_gpt
};
agt_instance_ctrl_t g_timer_a0_ctrl;
const agt_extended_cfg_t g_timer_a0_extend =
{
    .count_source            = AGT_CLOCK_PCLKB,
    .agto                    = AGT_PIN_CFG_DISABLED,
    .agtoab_settings_b.agtoa = AGT_PIN_CFG_DISABLED,
    .agtoab_settings_b.agtob = AGT_PIN_CFG_DISABLED,
    .measurement_mode        = AGT_MEASURE_PULSE_PERIOD,
    .agtio_filter            = AGT_AGTIO_FILTER_PCLKB_DIV_32,
    .enable_pin              = AGT_ENABLE_PIN_NOT_USED,
    .trigger_edge            = AGT_TRIGGER_EDGE_RISING,
};
const timer_cfg_t g_timer_a0_cfg =
{
    .mode                = TIMER_MODE_PERIODIC,
    /* Actual period: 0.065536 seconds. Actual duty: 50%. */ .period_counts = (uint32_t) 0x3c0000, .duty_cycle_counts = 0x1e0000, .source_div = (timer_source_div_t)0,
    .channel             = 0,
    .p_callback          = PedalSpeedTimer_IRQHandler,
    /** If NULL then do not add & */
#if defined(NULL)
    .p_context           = NULL,
#else
    .p_context           = &NULL,
#endif
    .p_extend            = &g_timer_a0_extend,
    .cycle_end_ipl       = (12),
#if defined(VECTOR_NUMBER_AGT0_INT)
    .cycle_end_irq       = VECTOR_NUMBER_AGT0_INT,
#else
    .cycle_end_irq       = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const timer_instance_t g_timer_a0 =
{
    .p_ctrl        = &g_timer_a0_ctrl,
    .p_cfg         = &g_timer_a0_cfg,
    .p_api         = &g_timer_on_agt
};
agt_instance_ctrl_t g_timer_a1_ctrl;
const agt_extended_cfg_t g_timer_a1_extend =
{
    .count_source            = AGT_CLOCK_PCLKB,
    .agto                    = AGT_PIN_CFG_DISABLED,
    .agtoab_settings_b.agtoa = AGT_PIN_CFG_DISABLED,
    .agtoab_settings_b.agtob = AGT_PIN_CFG_DISABLED,
    .measurement_mode        = AGT_MEASURE_DISABLED,
    .agtio_filter            = AGT_AGTIO_FILTER_NONE,
    .enable_pin              = AGT_ENABLE_PIN_NOT_USED,
    .trigger_edge            = AGT_TRIGGER_EDGE_RISING,
};
const timer_cfg_t g_timer_a1_cfg =
{
    .mode                = TIMER_MODE_PERIODIC,
    /* Actual period: 0.0010922666666666667 seconds. Actual duty: 50%. */ .period_counts = (uint32_t) 0x10000, .duty_cycle_counts = 0x8000, .source_div = (timer_source_div_t)0,
    .channel             = 1,
    .p_callback          = CANTimer_IRQHandler,
    /** If NULL then do not add & */
#if defined(NULL)
    .p_context           = NULL,
#else
    .p_context           = &NULL,
#endif
    .p_extend            = &g_timer_a1_extend,
    .cycle_end_ipl       = (12),
#if defined(VECTOR_NUMBER_AGT1_INT)
    .cycle_end_irq       = VECTOR_NUMBER_AGT1_INT,
#else
    .cycle_end_irq       = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const timer_instance_t g_timer_a1 =
{
    .p_ctrl        = &g_timer_a1_ctrl,
    .p_cfg         = &g_timer_a1_cfg,
    .p_api         = &g_timer_on_agt
};
/* Nominal and Data bit timing configuration */

can_bit_timing_cfg_t g_canfd0_bit_timing_cfg =
{
    /* Actual bitrate: 500000 Hz. Actual sample point: 75 %. */
    .baud_rate_prescaler = 2,
    .time_segment_1 = 29,
    .time_segment_2 = 10,
    .synchronization_jump_width = 4
};

can_bit_timing_cfg_t g_canfd0_data_timing_cfg =
{
    /* Actual bitrate: 500000 Hz. Actual sample point: 75 %. */
    .baud_rate_prescaler = 2,
    .time_segment_1 = 29,
    .time_segment_2 = 10,
    .synchronization_jump_width = 4
};


extern const canfd_afl_entry_t CANFD_FilterListArray[CANFD_CFG_AFL_CH0_RULE_NUM];

#ifndef CANFD_PRV_GLOBAL_CFG
#define CANFD_PRV_GLOBAL_CFG
canfd_global_cfg_t g_canfd_global_cfg =
{
    .global_interrupts = CANFD_CFG_GLOBAL_ERR_SOURCES,
    .global_config     = (CANFD_CFG_TX_PRIORITY | CANFD_CFG_DLC_CHECK | (BSP_CFG_CANFDCLK_SOURCE == BSP_CLOCKS_SOURCE_CLOCK_MAIN_OSC ? R_CANFD_CFDGCFG_DCS_Msk : 0U) | CANFD_CFG_FD_OVERFLOW),
    .rx_mb_config      = (CANFD_CFG_RXMB_NUMBER | (CANFD_CFG_RXMB_SIZE << R_CANFD_CFDRMNB_RMPLS_Pos)),
    .global_err_ipl = CANFD_CFG_GLOBAL_ERR_IPL,
    .rx_fifo_ipl    = CANFD_CFG_RX_FIFO_IPL,
    .rx_fifo_config    =
    {
        ((CANFD_CFG_RXFIFO0_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO0_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO0_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO0_INT_MODE) | (CANFD_CFG_RXFIFO0_ENABLE)),
        ((CANFD_CFG_RXFIFO1_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO1_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO1_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO1_INT_MODE) | (CANFD_CFG_RXFIFO1_ENABLE)),
#if !BSP_FEATURE_CANFD_LITE
        ((CANFD_CFG_RXFIFO2_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO2_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO2_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO2_INT_MODE) | (CANFD_CFG_RXFIFO2_ENABLE)),
        ((CANFD_CFG_RXFIFO3_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO3_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO3_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO3_INT_MODE) | (CANFD_CFG_RXFIFO3_ENABLE)),
        ((CANFD_CFG_RXFIFO4_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO4_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO4_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO4_INT_MODE) | (CANFD_CFG_RXFIFO4_ENABLE)),
        ((CANFD_CFG_RXFIFO5_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO5_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO5_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO5_INT_MODE) | (CANFD_CFG_RXFIFO5_ENABLE)),
        ((CANFD_CFG_RXFIFO6_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO6_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO6_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO6_INT_MODE) | (CANFD_CFG_RXFIFO6_ENABLE)),
        ((CANFD_CFG_RXFIFO7_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO7_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO7_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO7_INT_MODE) | (CANFD_CFG_RXFIFO7_ENABLE)),
#endif
    },
};
#endif

canfd_extended_cfg_t g_canfd0_extended_cfg =
{
    .p_afl              = CANFD_FilterListArray,
    .txmb_txi_enable    = ((1ULL << 0) |  0ULL),
    .error_interrupts   = ( 0U),
    .p_data_timing      = &g_canfd0_data_timing_cfg,
    .delay_compensation = (1),
    .p_global_cfg       = &g_canfd_global_cfg,
};

canfd_instance_ctrl_t g_canfd0_ctrl;
const can_cfg_t g_canfd0_cfg =
{
    .channel                = 0,
    .p_bit_timing           = &g_canfd0_bit_timing_cfg,
    .p_callback             = CANFD_IRQhandler,
    .p_extend               = &g_canfd0_extended_cfg,
    .p_context              = NULL,
    .ipl                    = (7),
#if defined(VECTOR_NUMBER_CAN0_TX)
    .tx_irq             = VECTOR_NUMBER_CAN0_TX,
#else
    .tx_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_CAN0_CHERR)
    .error_irq             = VECTOR_NUMBER_CAN0_CHERR,
#else
    .error_irq             = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const can_instance_t g_canfd0 =
{
    .p_ctrl        = &g_canfd0_ctrl,
    .p_cfg         = &g_canfd0_cfg,
    .p_api         = &g_canfd_on_canfd
};
sci_b_uart_instance_ctrl_t     g_uart9_ctrl;

            sci_b_baud_setting_t               g_uart9_baud_setting =
            {
                /* Baud rate calculated with 1.725% error. */ .baudrate_bits_b.abcse = 0, .baudrate_bits_b.abcs = 0, .baudrate_bits_b.bgdm = 1, .cks = 0, .brr = 15, .mddr = (uint8_t) 256, .baudrate_bits_b.brme = false
            };

            /** UART extended configuration for UARTonSCI HAL driver */
            const sci_b_uart_extended_cfg_t g_uart9_cfg_extend =
            {
                .clock                = SCI_B_UART_CLOCK_INT,
                .rx_edge_start          = SCI_B_UART_START_BIT_FALLING_EDGE,
                .noise_cancel         = SCI_B_UART_NOISE_CANCELLATION_DISABLE,
                .rx_fifo_trigger        = SCI_B_UART_RX_FIFO_TRIGGER_MAX,
                .p_baud_setting         = &g_uart9_baud_setting,
                .flow_control           = SCI_B_UART_FLOW_CONTROL_RTS,
                #if 0xFF != 0xFF
                .flow_control_pin       = BSP_IO_PORT_FF_PIN_0xFF,
                #else
                .flow_control_pin       = (bsp_io_port_pin_t) UINT16_MAX,
                #endif
                .rs485_setting          = {
                    .enable = SCI_B_UART_RS485_DISABLE,
                    .polarity = SCI_B_UART_RS485_DE_POLARITY_HIGH,
                    .assertion_time = 1,
                    .negation_time = 1,
                }
            };

            /** UART interface configuration */
            const uart_cfg_t g_uart9_cfg =
            {
                .channel             = 9,
                .data_bits           = UART_DATA_BITS_8,
                .parity              = UART_PARITY_OFF,
                .stop_bits           = UART_STOP_BITS_1,
                .p_callback          = UART_IRQHandler,
                .p_context           = NULL,
                .p_extend            = &g_uart9_cfg_extend,
#define RA_NOT_DEFINED (1)
#if (RA_NOT_DEFINED == RA_NOT_DEFINED)
                .p_transfer_tx       = NULL,
#else
                .p_transfer_tx       = &RA_NOT_DEFINED,
#endif
#if (RA_NOT_DEFINED == RA_NOT_DEFINED)
                .p_transfer_rx       = NULL,
#else
                .p_transfer_rx       = &RA_NOT_DEFINED,
#endif
#undef RA_NOT_DEFINED
                .rxi_ipl             = (12),
                .txi_ipl             = (12),
                .tei_ipl             = (12),
                .eri_ipl             = (12),
#if defined(VECTOR_NUMBER_SCI9_RXI)
                .rxi_irq             = VECTOR_NUMBER_SCI9_RXI,
#else
                .rxi_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI9_TXI)
                .txi_irq             = VECTOR_NUMBER_SCI9_TXI,
#else
                .txi_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI9_TEI)
                .tei_irq             = VECTOR_NUMBER_SCI9_TEI,
#else
                .tei_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI9_ERI)
                .eri_irq             = VECTOR_NUMBER_SCI9_ERI,
#else
                .eri_irq             = FSP_INVALID_VECTOR,
#endif
            };

/* Instance structure to use this module. */
const uart_instance_t g_uart9 =
{
    .p_ctrl        = &g_uart9_ctrl,
    .p_cfg         = &g_uart9_cfg,
    .p_api         = &g_uart_on_sci_b
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
    .clear_source        = (gpt_source_t) (GPT_SOURCE_GPT_H | GPT_SOURCE_GPT_G | GPT_SOURCE_GPT_F |  GPT_SOURCE_NONE),
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
    .p_callback          = HallTimer_IRQHandler,
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
icu_instance_ctrl_t g_external_irq5_ctrl;
const external_irq_cfg_t g_external_irq5_cfg =
{
    .channel             = 5,
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
#if defined(VECTOR_NUMBER_ICU_IRQ5)
    .irq                 = VECTOR_NUMBER_ICU_IRQ5,
#else
    .irq                 = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const external_irq_instance_t g_external_irq5 =
{
    .p_ctrl        = &g_external_irq5_ctrl,
    .p_cfg         = &g_external_irq5_cfg,
    .p_api         = &g_external_irq_on_icu
};
icu_instance_ctrl_t g_external_irq4_ctrl;
const external_irq_cfg_t g_external_irq4_cfg =
{
    .channel             = 4,
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
#if defined(VECTOR_NUMBER_ICU_IRQ4)
    .irq                 = VECTOR_NUMBER_ICU_IRQ4,
#else
    .irq                 = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const external_irq_instance_t g_external_irq4 =
{
    .p_ctrl        = &g_external_irq4_ctrl,
    .p_cfg         = &g_external_irq4_cfg,
    .p_api         = &g_external_irq_on_icu
};
icu_instance_ctrl_t g_external_irq3_ctrl;
const external_irq_cfg_t g_external_irq3_cfg =
{
    .channel             = 3,
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
#if defined(VECTOR_NUMBER_ICU_IRQ3)
    .irq                 = VECTOR_NUMBER_ICU_IRQ3,
#else
    .irq                 = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const external_irq_instance_t g_external_irq3 =
{
    .p_ctrl        = &g_external_irq3_ctrl,
    .p_cfg         = &g_external_irq3_cfg,
    .p_api         = &g_external_irq_on_icu
};
iirfa_instance_ctrl_t g_iirfa1_ctrl;

const iir_cfg_t g_iirfa1_cfg =
{
    .channel         = 1,
    .p_extend        = NULL,
};

/* Instance structure to use this module. */
const iirfa_instance_t g_iirfa1 =
{
    .p_ctrl          = &g_iirfa1_ctrl,
    .p_cfg           = &g_iirfa1_cfg,
    .p_api           = &g_iir_on_iirfa
};
iirfa_instance_ctrl_t g_iirfa0_ctrl;

const iir_cfg_t g_iirfa0_cfg =
{
    .channel         = 0,
    .p_extend        = NULL,
};

/* Instance structure to use this module. */
const iirfa_instance_t g_iirfa0 =
{
    .p_ctrl          = &g_iirfa0_ctrl,
    .p_cfg           = &g_iirfa0_cfg,
    .p_api           = &g_iir_on_iirfa
};
poeg_instance_ctrl_t g_poeg1_ctrl;
const poeg_cfg_t g_poeg1_cfg =
{
    .trigger             = (poeg_trigger_t) ( POEG_TRIGGER_SOFTWARE),
    .polarity            = POEG_GTETRG_POLARITY_ACTIVE_HIGH,
    .noise_filter        = POEG_GTETRG_NOISE_FILTER_PCLKB_DIV_32,
    .channel             = 1,
    .ipl                 = (0),
    .p_callback          = PWMBreak2_IRQHandler,
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
poeg_instance_ctrl_t g_poeg0_ctrl;
const poeg_cfg_t g_poeg0_cfg =
{
    .trigger             = (poeg_trigger_t) (POEG_TRIGGER_PIN |  POEG_TRIGGER_SOFTWARE),
    .polarity            = POEG_GTETRG_POLARITY_ACTIVE_LOW,
    .noise_filter        = POEG_GTETRG_NOISE_FILTER_PCLKB_DIV_32,
    .channel             = 0,
    .ipl                 = (0),
    .p_callback          = PWMBreak1_IRQHandler,
    .p_context           = NULL,
#if defined(VECTOR_NUMBER_POEG0_EVENT)
    .irq       = VECTOR_NUMBER_POEG0_EVENT,
#else
    .irq       = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const poeg_instance_t g_poeg0 =
{
    .p_ctrl        = &g_poeg0_ctrl,
    .p_cfg         = &g_poeg0_cfg,
    .p_api         = &g_poeg_on_poeg
};
dac_instance_ctrl_t g_dac3_ctrl;
const dac_extended_cfg_t g_dac3_ext_cfg =
{
    .enable_charge_pump   = 0,
    .data_format         = DAC_DATA_FORMAT_FLUSH_LEFT,
    .output_amplifier_enabled = 0,
    .internal_output_enabled = false,
};
const dac_cfg_t g_dac3_cfg =
{
    .channel             = 3,
    .ad_da_synchronized  = false,
    .p_extend            = &g_dac3_ext_cfg
};
/* Instance structure to use this module. */
const dac_instance_t g_dac3 =
{
    .p_ctrl    = &g_dac3_ctrl,
    .p_cfg     = &g_dac3_cfg,
    .p_api     = &g_dac_on_dac
};
dac_instance_ctrl_t g_dac2_ctrl;
const dac_extended_cfg_t g_dac2_ext_cfg =
{
    .enable_charge_pump   = 0,
    .data_format         = DAC_DATA_FORMAT_FLUSH_LEFT,
    .output_amplifier_enabled = 0,
    .internal_output_enabled = false,
};
const dac_cfg_t g_dac2_cfg =
{
    .channel             = 2,
    .ad_da_synchronized  = false,
    .p_extend            = &g_dac2_ext_cfg
};
/* Instance structure to use this module. */
const dac_instance_t g_dac2 =
{
    .p_ctrl    = &g_dac2_ctrl,
    .p_cfg     = &g_dac2_cfg,
    .p_api     = &g_dac_on_dac
};
#define RA_NOT_DEFINED (0) // TODO: Remove this after implementing all channels and groups.

                               #if (1) // Define Virtual Channel 0 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_0_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_0,

                                   .channel_cfg_bits.group             = (1),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_0),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_0) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_16_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_0) != ADC_CHANNEL_SELF_DIAGNOSIS),
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

                               #if (2) // Define Virtual Channel 2 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_2_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_2,

                                   .channel_cfg_bits.group             = (2),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_6),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_6) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_16_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_6) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (2) // Define Virtual Channel 3 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_3_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_3,

                                   .channel_cfg_bits.group             = (2),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_7),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_7) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_16_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_7) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (2) // Define Virtual Channel 4 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_4_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_4,

                                   .channel_cfg_bits.group             = (2),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_8),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_8) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_16_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_8) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (2) // Define Virtual Channel 5 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_5_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_5,

                                   .channel_cfg_bits.group             = (2),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_9),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_9) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_16_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_9) != ADC_CHANNEL_SELF_DIAGNOSIS),
                               };
                               #endif

                               #if (2) // Define Virtual Channel 6 if it's assigned to a scan group.
                               const adc_b_virtual_channel_cfg_t virtual_channel_6_cfg =
                               {
                                   .channel_id             = ADC_B_VIRTUAL_CHANNEL_6,

                                   .channel_cfg_bits.group             = (2),
                                   .channel_cfg_bits.channel           = (ADC_CHANNEL_20),
                                   .channel_cfg_bits.self_diag_enabled = ((ADC_CHANNEL_20) == ADC_CHANNEL_SELF_DIAGNOSIS),
                                   .channel_cfg_bits.sample_table_id   = ADC_B_SAMPLING_STATE_TABLE_0,

                                   .channel_control_a_bits.offset_table_id = ADC_B_USER_OFFSET_TABLE_SELECTION_DISABLED,
                                   .channel_control_a_bits.gain_table_id = ADC_B_USER_GAIN_TABLE_SELECTION_DISABLED,

                                   .channel_control_b_bits.addition_average_mode = (ADC_B_ADD_AVERAGE_OFF),
                                   .channel_control_b_bits.addition_average_count = ADC_B_ADD_AVERAGE_1,
                                   .channel_control_b_bits.compare_match_enable = false,

                                   .channel_control_c_bits.limiter_clip_table_id = ADC_B_LIMIT_CLIP_TABLE_SELECTION_NONE,
                                   .channel_control_c_bits.channel_resolution = (ADC_B_RESOLUTION_16_BIT),
                                   .channel_control_c_bits.data_sign_selection = ((ADC_CHANNEL_20) != ADC_CHANNEL_SELF_DIAGNOSIS),
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
#if (((1) == 1)||((1) == 1)||((2) == 1)||((2) == 1)||((2) == 1)||((2) == 1)||((2) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1))
const adc_b_virtual_channel_cfg_t *const group_0_virtual_channels[] = {
                                  #if ((1) == 1)
                                      &virtual_channel_0_cfg,
                                  #endif

                                  #if ((1) == 1)
                                      &virtual_channel_1_cfg,
                                  #endif

                                  #if ((2) == 1)
                                      &virtual_channel_2_cfg,
                                  #endif

                                  #if ((2) == 1)
                                      &virtual_channel_3_cfg,
                                  #endif

                                  #if ((2) == 1)
                                      &virtual_channel_4_cfg,
                                  #endif

                                  #if ((2) == 1)
                                      &virtual_channel_5_cfg,
                                  #endif

                                  #if ((2) == 1)
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
#if (((1) == 2)||((1) == 2)||((2) == 2)||((2) == 2)||((2) == 2)||((2) == 2)||((2) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2))
const adc_b_virtual_channel_cfg_t *const group_1_virtual_channels[] = {
                                  #if ((1) == 2)
                                      &virtual_channel_0_cfg,
                                  #endif

                                  #if ((1) == 2)
                                      &virtual_channel_1_cfg,
                                  #endif

                                  #if ((2) == 2)
                                      &virtual_channel_2_cfg,
                                  #endif

                                  #if ((2) == 2)
                                      &virtual_channel_3_cfg,
                                  #endif

                                  #if ((2) == 2)
                                      &virtual_channel_4_cfg,
                                  #endif

                                  #if ((2) == 2)
                                      &virtual_channel_5_cfg,
                                  #endif

                                  #if ((2) == 2)
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
#if (((1) == 3)||((1) == 3)||((2) == 3)||((2) == 3)||((2) == 3)||((2) == 3)||((2) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3))
const adc_b_virtual_channel_cfg_t *const group_2_virtual_channels[] = {
                                  #if ((1) == 3)
                                      &virtual_channel_0_cfg,
                                  #endif

                                  #if ((1) == 3)
                                      &virtual_channel_1_cfg,
                                  #endif

                                  #if ((2) == 3)
                                      &virtual_channel_2_cfg,
                                  #endif

                                  #if ((2) == 3)
                                      &virtual_channel_3_cfg,
                                  #endif

                                  #if ((2) == 3)
                                      &virtual_channel_4_cfg,
                                  #endif

                                  #if ((2) == 3)
                                      &virtual_channel_5_cfg,
                                  #endif

                                  #if ((2) == 3)
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
#if (((1) == 4)||((1) == 4)||((2) == 4)||((2) == 4)||((2) == 4)||((2) == 4)||((2) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4))
const adc_b_virtual_channel_cfg_t *const group_3_virtual_channels[] = {
                                  #if ((1) == 4)
                                      &virtual_channel_0_cfg,
                                  #endif

                                  #if ((1) == 4)
                                      &virtual_channel_1_cfg,
                                  #endif

                                  #if ((2) == 4)
                                      &virtual_channel_2_cfg,
                                  #endif

                                  #if ((2) == 4)
                                      &virtual_channel_3_cfg,
                                  #endif

                                  #if ((2) == 4)
                                      &virtual_channel_4_cfg,
                                  #endif

                                  #if ((2) == 4)
                                      &virtual_channel_5_cfg,
                                  #endif

                                  #if ((2) == 4)
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
#if (((1) == 5)||((1) == 5)||((2) == 5)||((2) == 5)||((2) == 5)||((2) == 5)||((2) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5))
const adc_b_virtual_channel_cfg_t *const group_4_virtual_channels[] = {
                                  #if ((1) == 5)
                                      &virtual_channel_0_cfg,
                                  #endif

                                  #if ((1) == 5)
                                      &virtual_channel_1_cfg,
                                  #endif

                                  #if ((2) == 5)
                                      &virtual_channel_2_cfg,
                                  #endif

                                  #if ((2) == 5)
                                      &virtual_channel_3_cfg,
                                  #endif

                                  #if ((2) == 5)
                                      &virtual_channel_4_cfg,
                                  #endif

                                  #if ((2) == 5)
                                      &virtual_channel_5_cfg,
                                  #endif

                                  #if ((2) == 5)
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
#if (((1) == 6)||((1) == 6)||((2) == 6)||((2) == 6)||((2) == 6)||((2) == 6)||((2) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6))
const adc_b_virtual_channel_cfg_t *const group_5_virtual_channels[] = {
                                  #if ((1) == 6)
                                      &virtual_channel_0_cfg,
                                  #endif

                                  #if ((1) == 6)
                                      &virtual_channel_1_cfg,
                                  #endif

                                  #if ((2) == 6)
                                      &virtual_channel_2_cfg,
                                  #endif

                                  #if ((2) == 6)
                                      &virtual_channel_3_cfg,
                                  #endif

                                  #if ((2) == 6)
                                      &virtual_channel_4_cfg,
                                  #endif

                                  #if ((2) == 6)
                                      &virtual_channel_5_cfg,
                                  #endif

                                  #if ((2) == 6)
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
#if (((1) == 7)||((1) == 7)||((2) == 7)||((2) == 7)||((2) == 7)||((2) == 7)||((2) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7))
const adc_b_virtual_channel_cfg_t *const group_6_virtual_channels[] = {
                                  #if ((1) == 7)
                                      &virtual_channel_0_cfg,
                                  #endif

                                  #if ((1) == 7)
                                      &virtual_channel_1_cfg,
                                  #endif

                                  #if ((2) == 7)
                                      &virtual_channel_2_cfg,
                                  #endif

                                  #if ((2) == 7)
                                      &virtual_channel_3_cfg,
                                  #endif

                                  #if ((2) == 7)
                                      &virtual_channel_4_cfg,
                                  #endif

                                  #if ((2) == 7)
                                      &virtual_channel_5_cfg,
                                  #endif

                                  #if ((2) == 7)
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
#if (((1) == 8)||((1) == 8)||((2) == 8)||((2) == 8)||((2) == 8)||((2) == 8)||((2) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8))
const adc_b_virtual_channel_cfg_t *const group_7_virtual_channels[] = {
                                  #if ((1) == 8)
                                      &virtual_channel_0_cfg,
                                  #endif

                                  #if ((1) == 8)
                                      &virtual_channel_1_cfg,
                                  #endif

                                  #if ((2) == 8)
                                      &virtual_channel_2_cfg,
                                  #endif

                                  #if ((2) == 8)
                                      &virtual_channel_3_cfg,
                                  #endif

                                  #if ((2) == 8)
                                      &virtual_channel_4_cfg,
                                  #endif

                                  #if ((2) == 8)
                                      &virtual_channel_5_cfg,
                                  #endif

                                  #if ((2) == 8)
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
#if (((1) == 9)||((1) == 9)||((2) == 9)||((2) == 9)||((2) == 9)||((2) == 9)||((2) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9))
const adc_b_virtual_channel_cfg_t *const group_8_virtual_channels[] = {
                                  #if ((1) == 9)
                                      &virtual_channel_0_cfg,
                                  #endif

                                  #if ((1) == 9)
                                      &virtual_channel_1_cfg,
                                  #endif

                                  #if ((2) == 9)
                                      &virtual_channel_2_cfg,
                                  #endif

                                  #if ((2) == 9)
                                      &virtual_channel_3_cfg,
                                  #endif

                                  #if ((2) == 9)
                                      &virtual_channel_4_cfg,
                                  #endif

                                  #if ((2) == 9)
                                      &virtual_channel_5_cfg,
                                  #endif

                                  #if ((2) == 9)
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
                         ((1) == 1)+((1) == 1)+((2) == 1)+((2) == 1)+((2) == 1)+((2) == 1)+((2) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)+((0) == 1)),
#if (((1) == 1)||((1) == 1)||((2) == 1)||((2) == 1)||((2) == 1)||((2) == 1)||((2) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1)||((0) == 1))

                         .p_virtual_channels = (adc_b_virtual_channel_cfg_t**)group_0_virtual_channels,
                         #else
                         .p_virtual_channels = NULL,
                         #endif
                     };
                     #endif

                     #if (1) // Define Scan Group 1 if it's enabled
                     const adc_b_group_cfg_t adc_b_group_1_cfg =
                     {
                         .scan_group_id                   = ADC_GROUP_ID_1,
                         .converter_selection             = (1),
                         .scan_group_enable               = (1),
                         .scan_end_interrupt_enable       = (0),
                         .external_trigger_enable_mask    = ( ADC_B_EXTERNAL_TRIGGER_NONE),
                         .elc_trigger_enable_mask         = ( 0x00),
                         .gpt_trigger_enable_mask         = ( ADC_B_GPT_TRIGGER_NONE),

                         .self_diagnosis_mask             = (ADC_B_SELF_DIAGNOSIS_DISABLED << R_ADC_B0_ADSGDCR0_DIAGVAL_Pos),

                         .limit_clip_interrupt_enable     = (0),
                         .virtual_channel_count           = 0 + (
                         ((1) == 2)+((1) == 2)+((2) == 2)+((2) == 2)+((2) == 2)+((2) == 2)+((2) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)+((0) == 2)),
#if (((1) == 2)||((1) == 2)||((2) == 2)||((2) == 2)||((2) == 2)||((2) == 2)||((2) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2)||((0) == 2))

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
                         ((1) == 3)+((1) == 3)+((2) == 3)+((2) == 3)+((2) == 3)+((2) == 3)+((2) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)+((0) == 3)),
#if (((1) == 3)||((1) == 3)||((2) == 3)||((2) == 3)||((2) == 3)||((2) == 3)||((2) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3)||((0) == 3))

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
                         ((1) == 4)+((1) == 4)+((2) == 4)+((2) == 4)+((2) == 4)+((2) == 4)+((2) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)+((0) == 4)),
#if (((1) == 4)||((1) == 4)||((2) == 4)||((2) == 4)||((2) == 4)||((2) == 4)||((2) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4)||((0) == 4))

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
                         ((1) == 5)+((1) == 5)+((2) == 5)+((2) == 5)+((2) == 5)+((2) == 5)+((2) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)+((0) == 5)),
#if (((1) == 5)||((1) == 5)||((2) == 5)||((2) == 5)||((2) == 5)||((2) == 5)||((2) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5)||((0) == 5))

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
                         ((1) == 6)+((1) == 6)+((2) == 6)+((2) == 6)+((2) == 6)+((2) == 6)+((2) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)+((0) == 6)),
#if (((1) == 6)||((1) == 6)||((2) == 6)||((2) == 6)||((2) == 6)||((2) == 6)||((2) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6)||((0) == 6))

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
                         ((1) == 7)+((1) == 7)+((2) == 7)+((2) == 7)+((2) == 7)+((2) == 7)+((2) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)+((0) == 7)),
#if (((1) == 7)||((1) == 7)||((2) == 7)||((2) == 7)||((2) == 7)||((2) == 7)||((2) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7)||((0) == 7))

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
                         ((1) == 8)+((1) == 8)+((2) == 8)+((2) == 8)+((2) == 8)+((2) == 8)+((2) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)+((0) == 8)),
#if (((1) == 8)||((1) == 8)||((2) == 8)||((2) == 8)||((2) == 8)||((2) == 8)||((2) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8)||((0) == 8))

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
                         ((1) == 9)+((1) == 9)+((2) == 9)+((2) == 9)+((2) == 9)+((2) == 9)+((2) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)+((0) == 9)),
#if (((1) == 9)||((1) == 9)||((2) == 9)||((2) == 9)||((2) == 9)||((2) == 9)||((2) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9)||((0) == 9))

                         .p_virtual_channels = (adc_b_virtual_channel_cfg_t**)group_8_virtual_channels,
                         #else
                         .p_virtual_channels = NULL,
                         #endif
                     };
                     #endif
#if ((1)||(1)||(0)||(0)||(0)||(0)||(0)||(0)||(0))
const adc_b_group_cfg_t * const adc_b_scan_cfg_groups[] = {
                           #if (0 != (1))
                           &adc_b_group_0_cfg,
                           #endif

                           #if (0 != (1))
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

                       const adc_b_scan_cfg_t g_adc0_scan_cfg =
                       {
                       .group_count = ( 0 +
                       (0 != (1)) + (0 != (1)) + (0 != (0)) + (0 != (0)) + (0 != (0)) + (0 != (0)) + (0 != (0)) + (0 != (0)) + (0 != (0))),
#if ((0 != (1))||(0 != (1))||(0 != (0))||(0 != (0))||(0 != (0))||(0 != (0))||(0 != (0))||(0 != (0))||(0 != (0)))
                       .p_adc_groups = (adc_b_group_cfg_t**)adc_b_scan_cfg_groups,
                       #else
                       .p_adc_groups = NULL,
                       #endif
                       };

     const adc_b_isr_cfg_t g_adc0_isr_cfg =
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

const adc_b_extended_cfg_t g_adc0_cfg_extend =
{
    .clock_control_data     = ((ADC_B_CLOCK_SOURCE_PCLKC << R_ADC_B0_ADCLKCR_CLKSEL_Pos) |
                               (ADC_B_CLOCK_DIV_1 << R_ADC_B0_ADCLKCR_DIVR_Pos)),
    .sync_operation_control = (((1) << R_ADC_B0_ADSYCR_ADSYDIS0_Pos) |
                               ((1) << R_ADC_B0_ADSYCR_ADSYDIS1_Pos) |
                               (100 << R_ADC_B0_ADSYCR_ADSYCYC_Pos)),
    .adc_b_mode =  (((0) << R_ADC_B0_ADMDR_ADMD0_Pos) | ((0) << R_ADC_B0_ADMDR_ADMD1_Pos)),
    .converter_selection_0 = (((0) << R_ADC_B0_ADSGCR0_SGADS0_Pos) |
                              ((1) << R_ADC_B0_ADSGCR0_SGADS1_Pos) |
                              ((0) << R_ADC_B0_ADSGCR0_SGADS2_Pos) |
                              ((0) << R_ADC_B0_ADSGCR0_SGADS3_Pos)),
    .converter_selection_1 = (((0) << R_ADC_B0_ADSGCR1_SGADS4_Pos) |
                              ((0) << R_ADC_B0_ADSGCR1_SGADS5_Pos) |
                              ((0) << R_ADC_B0_ADSGCR1_SGADS6_Pos) |
                              ((0) << R_ADC_B0_ADSGCR1_SGADS7_Pos)),
    .converter_selection_2 =  ((0) << R_ADC_B0_ADSGCR2_SGADS8_Pos),

    .fifo_enable_mask     = (((0) << R_ADC_B0_ADFIFOCR_FIFOEN0_Pos) |
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
   .p_isr_cfg = &g_adc0_isr_cfg,
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


const adc_cfg_t g_adc0_cfg =
{
    .unit                = 0xFFFC,
    .mode                = (adc_mode_t) 0, // Unused
    .resolution          = (adc_resolution_t) 0, // Unused
    .alignment           = ADC_ALIGNMENT_RIGHT,
    .trigger             = (adc_trigger_t) 0, // Unused
    .p_callback          = ADC_IRQHandler,
#if defined(NULL)
    .p_context           = NULL,
#else
    .p_context           = &NULL,
#endif
    .p_extend            = &g_adc0_cfg_extend,

    .scan_end_irq        = FSP_INVALID_VECTOR,
    .scan_end_ipl        = BSP_IRQ_DISABLED,
    .scan_end_b_irq      = FSP_INVALID_VECTOR,
    .scan_end_b_ipl      = BSP_IRQ_DISABLED,
};


adc_b_instance_ctrl_t g_adc0_ctrl;


const adc_instance_t g_adc0 =
{
    .p_ctrl    = &g_adc0_ctrl,
    .p_cfg = &g_adc0_cfg,
    .p_channel_cfg = &g_adc0_scan_cfg,
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
    .gtioca_disable_setting = GPT_GTIOC_DISABLE_LEVEL_LOW,
    .gtiocb_disable_setting = GPT_GTIOC_DISABLE_LEVEL_LOW,
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
    .gtior_setting.gtior_b.oadf   = (uint32_t) GPT_GTIOC_DISABLE_LEVEL_LOW,
    .gtior_setting.gtior_b.nfaen  = ((uint32_t) GPT_CAPTURE_FILTER_NONE & 1U),
    .gtior_setting.gtior_b.nfcsa  = ((uint32_t) GPT_CAPTURE_FILTER_NONE >> 1U),
    .gtior_setting.gtior_b.gtiob  = (1U << 4U) | (2U << 2U) | (3U << 0U),
    .gtior_setting.gtior_b.obdflt = (uint32_t) GPT_PIN_LEVEL_LOW,
    .gtior_setting.gtior_b.obhld  = 0U,
    .gtior_setting.gtior_b.obe    = (uint32_t) true,
    .gtior_setting.gtior_b.obdf   = (uint32_t) GPT_GTIOC_DISABLE_LEVEL_LOW,
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
    .gtioca_disable_setting = GPT_GTIOC_DISABLE_LEVEL_LOW,
    .gtiocb_disable_setting = GPT_GTIOC_DISABLE_LEVEL_LOW,
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
    .gtior_setting.gtior_b.oadf   = (uint32_t) GPT_GTIOC_DISABLE_LEVEL_LOW,
    .gtior_setting.gtior_b.nfaen  = ((uint32_t) GPT_CAPTURE_FILTER_NONE & 1U),
    .gtior_setting.gtior_b.nfcsa  = ((uint32_t) GPT_CAPTURE_FILTER_NONE >> 1U),
    .gtior_setting.gtior_b.gtiob  = (1U << 4U) | (2U << 2U) | (3U << 0U),
    .gtior_setting.gtior_b.obdflt = (uint32_t) GPT_PIN_LEVEL_LOW,
    .gtior_setting.gtior_b.obhld  = 0U,
    .gtior_setting.gtior_b.obe    = (uint32_t) true,
    .gtior_setting.gtior_b.obdf   = (uint32_t) GPT_GTIOC_DISABLE_LEVEL_LOW,
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
    .poeg_link           = GPT_POEG_LINK_POEG0,
    .output_disable      =  GPT_OUTPUT_DISABLE_NONE,
    .adc_trigger         = GPT_ADC_TRIGGER_DOWN_COUNT_START_ADC_A |  GPT_ADC_TRIGGER_NONE,
    .dead_time_count_up  = 0,
    .dead_time_count_down = 0,
    .adc_a_compare_match = 0,
    .adc_b_compare_match = 0,
    .interrupt_skip_source = GPT_INTERRUPT_SKIP_SOURCE_NONE,
    .interrupt_skip_count  = GPT_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_adc    = GPT_INTERRUPT_SKIP_ADC_NONE,
    .gtioca_disable_setting = GPT_GTIOC_DISABLE_LEVEL_LOW,
    .gtiocb_disable_setting = GPT_GTIOC_DISABLE_LEVEL_LOW,
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
    .gtior_setting.gtior_b.oadf   = (uint32_t) GPT_GTIOC_DISABLE_LEVEL_LOW,
    .gtior_setting.gtior_b.nfaen  = ((uint32_t) GPT_CAPTURE_FILTER_NONE & 1U),
    .gtior_setting.gtior_b.nfcsa  = ((uint32_t) GPT_CAPTURE_FILTER_NONE >> 1U),
    .gtior_setting.gtior_b.gtiob  = (1U << 4U) | (2U << 2U) | (3U << 0U),
    .gtior_setting.gtior_b.obdflt = (uint32_t) GPT_PIN_LEVEL_LOW,
    .gtior_setting.gtior_b.obhld  = 0U,
    .gtior_setting.gtior_b.obe    = (uint32_t) true,
    .gtior_setting.gtior_b.obdf   = (uint32_t) GPT_GTIOC_DISABLE_LEVEL_LOW,
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
    .p_callback          = PWMTimer_IRQHandler,
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
