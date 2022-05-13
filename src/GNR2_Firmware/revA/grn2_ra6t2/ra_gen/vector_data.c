/* generated vector source file - do not edit */
        #include "bsp_api.h"
        /* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
        #if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_MAX_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = gpt_counter_overflow_isr, /* GPT4 COUNTER OVERFLOW (Overflow) */
            [1] = adc_b_limclpi_isr, /* ADC0 LIMCLPI (Limiter clip interrupt with the limit table 0 to 7) */
            [2] = adc_b_err0_isr, /* ADC0 ERR0 (A/D converter unit 0 Error) */
            [3] = adc_b_err1_isr, /* ADC0 ERR1 (A/D converter unit 1 Error) */
            [4] = adc_b_resovf0_isr, /* ADC0 RESOVF0 (A/D conversion overflow on A/D converter unit 0) */
            [5] = adc_b_resovf1_isr, /* ADC0 RESOVF1 (A/D conversion overflow on A/D converter unit 1) */
            [6] = adc_b_calend0_isr, /* ADC0 CALEND0 (End of calibration of A/D converter unit 0) */
            [7] = adc_b_calend1_isr, /* ADC0 CALEND1 (End of calibration of A/D converter unit 1) */
            [8] = adc_b_adi0_isr, /* ADC0 ADI0 (End of A/D scanning operation(Gr.0)) */
            [9] = adc_b_adi1_isr, /* ADC0 ADI1 (End of A/D scanning operation(Gr.1)) */
            [10] = adc_b_adi2_isr, /* ADC0 ADI2 (End of A/D scanning operation(Gr.2)) */
            [11] = adc_b_adi3_isr, /* ADC0 ADI3 (End of A/D scanning operation(Gr.3)) */
            [12] = adc_b_adi4_isr, /* ADC0 ADI4 (End of A/D scanning operation(Gr.4)) */
            [13] = adc_b_adi5678_isr, /* ADC0 ADI5678 (End of A/D scanning operation(Gr.5 to 8)) */
            [14] = adc_b_fifoovf_isr, /* ADC0 FIFOOVF (FIFO data overflow) */
            [15] = adc_b_fiforeq0_isr, /* ADC0 FIFOREQ0 (FIFO data read request interrupt(Gr.0)) */
            [16] = adc_b_fiforeq1_isr, /* ADC0 FIFOREQ1 (FIFO data read request interrupt(Gr.1)) */
            [17] = adc_b_fiforeq2_isr, /* ADC0 FIFOREQ2 (FIFO data read request interrupt(Gr.2)) */
            [18] = adc_b_fiforeq3_isr, /* ADC0 FIFOREQ3 (FIFO data read request interrupt(Gr.3)) */
            [19] = adc_b_fiforeq4_isr, /* ADC0 FIFOREQ4 (FIFO data read request interrupt(Gr.4)) */
            [20] = adc_b_fiforeq5678_isr, /* ADC0 FIFOREQ5678 (FIFO data read request interrupt(Gr.5 to 8)) */
        };
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_MAX_ENTRIES] =
        {
            [0] = BSP_PRV_IELS_ENUM(EVENT_GPT4_COUNTER_OVERFLOW), /* GPT4 COUNTER OVERFLOW (Overflow) */
            [1] = BSP_PRV_IELS_ENUM(EVENT_ADC12_LIMCLPI), /* ADC0 LIMCLPI (Limiter clip interrupt with the limit table 0 to 7) */
            [2] = BSP_PRV_IELS_ENUM(EVENT_ADC12_ERR0), /* ADC0 ERR0 (A/D converter unit 0 Error) */
            [3] = BSP_PRV_IELS_ENUM(EVENT_ADC12_ERR1), /* ADC0 ERR1 (A/D converter unit 1 Error) */
            [4] = BSP_PRV_IELS_ENUM(EVENT_ADC12_RESOVF0), /* ADC0 RESOVF0 (A/D conversion overflow on A/D converter unit 0) */
            [5] = BSP_PRV_IELS_ENUM(EVENT_ADC12_RESOVF1), /* ADC0 RESOVF1 (A/D conversion overflow on A/D converter unit 1) */
            [6] = BSP_PRV_IELS_ENUM(EVENT_ADC12_CALEND0), /* ADC0 CALEND0 (End of calibration of A/D converter unit 0) */
            [7] = BSP_PRV_IELS_ENUM(EVENT_ADC12_CALEND1), /* ADC0 CALEND1 (End of calibration of A/D converter unit 1) */
            [8] = BSP_PRV_IELS_ENUM(EVENT_ADC12_ADI0), /* ADC0 ADI0 (End of A/D scanning operation(Gr.0)) */
            [9] = BSP_PRV_IELS_ENUM(EVENT_ADC12_ADI1), /* ADC0 ADI1 (End of A/D scanning operation(Gr.1)) */
            [10] = BSP_PRV_IELS_ENUM(EVENT_ADC12_ADI2), /* ADC0 ADI2 (End of A/D scanning operation(Gr.2)) */
            [11] = BSP_PRV_IELS_ENUM(EVENT_ADC12_ADI3), /* ADC0 ADI3 (End of A/D scanning operation(Gr.3)) */
            [12] = BSP_PRV_IELS_ENUM(EVENT_ADC12_ADI4), /* ADC0 ADI4 (End of A/D scanning operation(Gr.4)) */
            [13] = BSP_PRV_IELS_ENUM(EVENT_ADC12_ADI5678), /* ADC0 ADI5678 (End of A/D scanning operation(Gr.5 to 8)) */
            [14] = BSP_PRV_IELS_ENUM(EVENT_ADC12_FIFOOVF), /* ADC0 FIFOOVF (FIFO data overflow) */
            [15] = BSP_PRV_IELS_ENUM(EVENT_ADC12_FIFOREQ0), /* ADC0 FIFOREQ0 (FIFO data read request interrupt(Gr.0)) */
            [16] = BSP_PRV_IELS_ENUM(EVENT_ADC12_FIFOREQ1), /* ADC0 FIFOREQ1 (FIFO data read request interrupt(Gr.1)) */
            [17] = BSP_PRV_IELS_ENUM(EVENT_ADC12_FIFOREQ2), /* ADC0 FIFOREQ2 (FIFO data read request interrupt(Gr.2)) */
            [18] = BSP_PRV_IELS_ENUM(EVENT_ADC12_FIFOREQ3), /* ADC0 FIFOREQ3 (FIFO data read request interrupt(Gr.3)) */
            [19] = BSP_PRV_IELS_ENUM(EVENT_ADC12_FIFOREQ4), /* ADC0 FIFOREQ4 (FIFO data read request interrupt(Gr.4)) */
            [20] = BSP_PRV_IELS_ENUM(EVENT_ADC12_FIFOREQ5678), /* ADC0 FIFOREQ5678 (FIFO data read request interrupt(Gr.5 to 8)) */
        };
        #endif