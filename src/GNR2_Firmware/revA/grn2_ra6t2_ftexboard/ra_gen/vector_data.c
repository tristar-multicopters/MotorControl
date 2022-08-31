/* generated vector source file - do not edit */
        #include "bsp_api.h"
        /* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
        #if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_MAX_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = gpt_counter_overflow_isr, /* GPT4 COUNTER OVERFLOW (Overflow) */
            [1] = adc_b_adi0_isr, /* ADC0 ADI0 (End of A/D scanning operation(Gr.0)) */
            [2] = poeg_event_isr, /* POEG0 EVENT (Port Output disable interrupt A) */
            [3] = poeg_event_isr, /* POEG1 EVENT (Port Output disable interrupt B) */
            [4] = r_icu_isr, /* ICU IRQ3 (External pin interrupt 3) */
            [5] = r_icu_isr, /* ICU IRQ4 (External pin interrupt 4) */
            [6] = r_icu_isr, /* ICU IRQ5 (External pin interrupt 5) */
            [7] = gpt_counter_overflow_isr, /* GPT0 COUNTER OVERFLOW (Overflow) */
            [8] = gpt_capture_a_isr, /* GPT0 CAPTURE COMPARE A (Compare match A) */
            [9] = canfd_error_isr, /* CAN0 CHERR (Channel error) */
            [10] = canfd_channel_tx_isr, /* CAN0 TX (Transmit interrupt) */
            [11] = canfd_error_isr, /* CAN GLERR (Global error) */
            [12] = canfd_rx_fifo_isr, /* CAN RXF (Global recieve FIFO interrupt) */
            [13] = agt_int_isr, /* AGT1 INT (AGT interrupt) */
            [14] = agt_int_isr, /* AGT0 INT (AGT interrupt) */
            [15] = gpt_counter_overflow_isr, /* GPT2 COUNTER OVERFLOW (Overflow) */
            [16] = gpt_capture_a_isr, /* GPT2 CAPTURE COMPARE A (Compare match A) */
            [17] = sci_b_uart_rxi_isr, /* SCI9 RXI (Received data full) */
            [18] = sci_b_uart_txi_isr, /* SCI9 TXI (Transmit data empty) */
            [19] = sci_b_uart_tei_isr, /* SCI9 TEI (Transmit end) */
            [20] = sci_b_uart_eri_isr, /* SCI9 ERI (Receive error) */
        };
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_MAX_ENTRIES] =
        {
            [0] = BSP_PRV_IELS_ENUM(EVENT_GPT4_COUNTER_OVERFLOW), /* GPT4 COUNTER OVERFLOW (Overflow) */
            [1] = BSP_PRV_IELS_ENUM(EVENT_ADC12_ADI0), /* ADC0 ADI0 (End of A/D scanning operation(Gr.0)) */
            [2] = BSP_PRV_IELS_ENUM(EVENT_POEG0_EVENT), /* POEG0 EVENT (Port Output disable interrupt A) */
            [3] = BSP_PRV_IELS_ENUM(EVENT_POEG1_EVENT), /* POEG1 EVENT (Port Output disable interrupt B) */
            [4] = BSP_PRV_IELS_ENUM(EVENT_ICU_IRQ3), /* ICU IRQ3 (External pin interrupt 3) */
            [5] = BSP_PRV_IELS_ENUM(EVENT_ICU_IRQ4), /* ICU IRQ4 (External pin interrupt 4) */
            [6] = BSP_PRV_IELS_ENUM(EVENT_ICU_IRQ5), /* ICU IRQ5 (External pin interrupt 5) */
            [7] = BSP_PRV_IELS_ENUM(EVENT_GPT0_COUNTER_OVERFLOW), /* GPT0 COUNTER OVERFLOW (Overflow) */
            [8] = BSP_PRV_IELS_ENUM(EVENT_GPT0_CAPTURE_COMPARE_A), /* GPT0 CAPTURE COMPARE A (Compare match A) */
            [9] = BSP_PRV_IELS_ENUM(EVENT_CAN0_CHERR), /* CAN0 CHERR (Channel error) */
            [10] = BSP_PRV_IELS_ENUM(EVENT_CAN0_TX), /* CAN0 TX (Transmit interrupt) */
            [11] = BSP_PRV_IELS_ENUM(EVENT_CAN_GLERR), /* CAN GLERR (Global error) */
            [12] = BSP_PRV_IELS_ENUM(EVENT_CAN_RXF), /* CAN RXF (Global recieve FIFO interrupt) */
            [13] = BSP_PRV_IELS_ENUM(EVENT_AGT1_INT), /* AGT1 INT (AGT interrupt) */
            [14] = BSP_PRV_IELS_ENUM(EVENT_AGT0_INT), /* AGT0 INT (AGT interrupt) */
            [15] = BSP_PRV_IELS_ENUM(EVENT_GPT2_COUNTER_OVERFLOW), /* GPT2 COUNTER OVERFLOW (Overflow) */
            [16] = BSP_PRV_IELS_ENUM(EVENT_GPT2_CAPTURE_COMPARE_A), /* GPT2 CAPTURE COMPARE A (Compare match A) */
            [17] = BSP_PRV_IELS_ENUM(EVENT_SCI9_RXI), /* SCI9 RXI (Received data full) */
            [18] = BSP_PRV_IELS_ENUM(EVENT_SCI9_TXI), /* SCI9 TXI (Transmit data empty) */
            [19] = BSP_PRV_IELS_ENUM(EVENT_SCI9_TEI), /* SCI9 TEI (Transmit end) */
            [20] = BSP_PRV_IELS_ENUM(EVENT_SCI9_ERI), /* SCI9 ERI (Receive error) */
        };
        #endif