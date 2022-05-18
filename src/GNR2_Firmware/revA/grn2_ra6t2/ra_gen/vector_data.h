/* generated vector header file - do not edit */
        #ifndef VECTOR_DATA_H
        #define VECTOR_DATA_H
        #ifdef __cplusplus
        extern "C" {
        #endif
                /* Number of interrupts allocated */
        #ifndef VECTOR_DATA_IRQ_COUNT
        #define VECTOR_DATA_IRQ_COUNT    (26)
        #endif
        /* ISR prototypes */
        void gpt_counter_overflow_isr(void);
        void adc_b_limclpi_isr(void);
        void adc_b_err0_isr(void);
        void adc_b_err1_isr(void);
        void adc_b_resovf0_isr(void);
        void adc_b_resovf1_isr(void);
        void adc_b_calend0_isr(void);
        void adc_b_calend1_isr(void);
        void adc_b_adi0_isr(void);
        void adc_b_adi1_isr(void);
        void adc_b_adi2_isr(void);
        void adc_b_adi3_isr(void);
        void adc_b_adi4_isr(void);
        void adc_b_adi5678_isr(void);
        void adc_b_fifoovf_isr(void);
        void adc_b_fiforeq0_isr(void);
        void adc_b_fiforeq1_isr(void);
        void adc_b_fiforeq2_isr(void);
        void adc_b_fiforeq3_isr(void);
        void adc_b_fiforeq4_isr(void);
        void adc_b_fiforeq5678_isr(void);
        void r_icu_isr(void);
        void gpt_capture_a_isr(void);

        /* Vector table allocations */
        #define VECTOR_NUMBER_GPT4_COUNTER_OVERFLOW ((IRQn_Type) 0) /* GPT4 COUNTER OVERFLOW (Overflow) */
        #define GPT4_COUNTER_OVERFLOW_IRQn          ((IRQn_Type) 0) /* GPT4 COUNTER OVERFLOW (Overflow) */
        #define VECTOR_NUMBER_ADC12_LIMCLPI ((IRQn_Type) 1) /* ADC0 LIMCLPI (Limiter clip interrupt with the limit table 0 to 7) */
        #define ADC12_LIMCLPI_IRQn          ((IRQn_Type) 1) /* ADC0 LIMCLPI (Limiter clip interrupt with the limit table 0 to 7) */
        #define VECTOR_NUMBER_ADC12_ERR0 ((IRQn_Type) 2) /* ADC0 ERR0 (A/D converter unit 0 Error) */
        #define ADC12_ERR0_IRQn          ((IRQn_Type) 2) /* ADC0 ERR0 (A/D converter unit 0 Error) */
        #define VECTOR_NUMBER_ADC12_ERR1 ((IRQn_Type) 3) /* ADC0 ERR1 (A/D converter unit 1 Error) */
        #define ADC12_ERR1_IRQn          ((IRQn_Type) 3) /* ADC0 ERR1 (A/D converter unit 1 Error) */
        #define VECTOR_NUMBER_ADC12_RESOVF0 ((IRQn_Type) 4) /* ADC0 RESOVF0 (A/D conversion overflow on A/D converter unit 0) */
        #define ADC12_RESOVF0_IRQn          ((IRQn_Type) 4) /* ADC0 RESOVF0 (A/D conversion overflow on A/D converter unit 0) */
        #define VECTOR_NUMBER_ADC12_RESOVF1 ((IRQn_Type) 5) /* ADC0 RESOVF1 (A/D conversion overflow on A/D converter unit 1) */
        #define ADC12_RESOVF1_IRQn          ((IRQn_Type) 5) /* ADC0 RESOVF1 (A/D conversion overflow on A/D converter unit 1) */
        #define VECTOR_NUMBER_ADC12_CALEND0 ((IRQn_Type) 6) /* ADC0 CALEND0 (End of calibration of A/D converter unit 0) */
        #define ADC12_CALEND0_IRQn          ((IRQn_Type) 6) /* ADC0 CALEND0 (End of calibration of A/D converter unit 0) */
        #define VECTOR_NUMBER_ADC12_CALEND1 ((IRQn_Type) 7) /* ADC0 CALEND1 (End of calibration of A/D converter unit 1) */
        #define ADC12_CALEND1_IRQn          ((IRQn_Type) 7) /* ADC0 CALEND1 (End of calibration of A/D converter unit 1) */
        #define VECTOR_NUMBER_ADC12_ADI0 ((IRQn_Type) 8) /* ADC0 ADI0 (End of A/D scanning operation(Gr.0)) */
        #define ADC12_ADI0_IRQn          ((IRQn_Type) 8) /* ADC0 ADI0 (End of A/D scanning operation(Gr.0)) */
        #define VECTOR_NUMBER_ADC12_ADI1 ((IRQn_Type) 9) /* ADC0 ADI1 (End of A/D scanning operation(Gr.1)) */
        #define ADC12_ADI1_IRQn          ((IRQn_Type) 9) /* ADC0 ADI1 (End of A/D scanning operation(Gr.1)) */
        #define VECTOR_NUMBER_ADC12_ADI2 ((IRQn_Type) 10) /* ADC0 ADI2 (End of A/D scanning operation(Gr.2)) */
        #define ADC12_ADI2_IRQn          ((IRQn_Type) 10) /* ADC0 ADI2 (End of A/D scanning operation(Gr.2)) */
        #define VECTOR_NUMBER_ADC12_ADI3 ((IRQn_Type) 11) /* ADC0 ADI3 (End of A/D scanning operation(Gr.3)) */
        #define ADC12_ADI3_IRQn          ((IRQn_Type) 11) /* ADC0 ADI3 (End of A/D scanning operation(Gr.3)) */
        #define VECTOR_NUMBER_ADC12_ADI4 ((IRQn_Type) 12) /* ADC0 ADI4 (End of A/D scanning operation(Gr.4)) */
        #define ADC12_ADI4_IRQn          ((IRQn_Type) 12) /* ADC0 ADI4 (End of A/D scanning operation(Gr.4)) */
        #define VECTOR_NUMBER_ADC12_ADI5678 ((IRQn_Type) 13) /* ADC0 ADI5678 (End of A/D scanning operation(Gr.5 to 8)) */
        #define ADC12_ADI5678_IRQn          ((IRQn_Type) 13) /* ADC0 ADI5678 (End of A/D scanning operation(Gr.5 to 8)) */
        #define VECTOR_NUMBER_ADC12_FIFOOVF ((IRQn_Type) 14) /* ADC0 FIFOOVF (FIFO data overflow) */
        #define ADC12_FIFOOVF_IRQn          ((IRQn_Type) 14) /* ADC0 FIFOOVF (FIFO data overflow) */
        #define VECTOR_NUMBER_ADC12_FIFOREQ0 ((IRQn_Type) 15) /* ADC0 FIFOREQ0 (FIFO data read request interrupt(Gr.0)) */
        #define ADC12_FIFOREQ0_IRQn          ((IRQn_Type) 15) /* ADC0 FIFOREQ0 (FIFO data read request interrupt(Gr.0)) */
        #define VECTOR_NUMBER_ADC12_FIFOREQ1 ((IRQn_Type) 16) /* ADC0 FIFOREQ1 (FIFO data read request interrupt(Gr.1)) */
        #define ADC12_FIFOREQ1_IRQn          ((IRQn_Type) 16) /* ADC0 FIFOREQ1 (FIFO data read request interrupt(Gr.1)) */
        #define VECTOR_NUMBER_ADC12_FIFOREQ2 ((IRQn_Type) 17) /* ADC0 FIFOREQ2 (FIFO data read request interrupt(Gr.2)) */
        #define ADC12_FIFOREQ2_IRQn          ((IRQn_Type) 17) /* ADC0 FIFOREQ2 (FIFO data read request interrupt(Gr.2)) */
        #define VECTOR_NUMBER_ADC12_FIFOREQ3 ((IRQn_Type) 18) /* ADC0 FIFOREQ3 (FIFO data read request interrupt(Gr.3)) */
        #define ADC12_FIFOREQ3_IRQn          ((IRQn_Type) 18) /* ADC0 FIFOREQ3 (FIFO data read request interrupt(Gr.3)) */
        #define VECTOR_NUMBER_ADC12_FIFOREQ4 ((IRQn_Type) 19) /* ADC0 FIFOREQ4 (FIFO data read request interrupt(Gr.4)) */
        #define ADC12_FIFOREQ4_IRQn          ((IRQn_Type) 19) /* ADC0 FIFOREQ4 (FIFO data read request interrupt(Gr.4)) */
        #define VECTOR_NUMBER_ADC12_FIFOREQ5678 ((IRQn_Type) 20) /* ADC0 FIFOREQ5678 (FIFO data read request interrupt(Gr.5 to 8)) */
        #define ADC12_FIFOREQ5678_IRQn          ((IRQn_Type) 20) /* ADC0 FIFOREQ5678 (FIFO data read request interrupt(Gr.5 to 8)) */
        #define VECTOR_NUMBER_ICU_IRQ1 ((IRQn_Type) 21) /* ICU IRQ1 (External pin interrupt 1) */
        #define ICU_IRQ1_IRQn          ((IRQn_Type) 21) /* ICU IRQ1 (External pin interrupt 1) */
        #define VECTOR_NUMBER_ICU_IRQ10 ((IRQn_Type) 22) /* ICU IRQ10 (External pin interrupt 10) */
        #define ICU_IRQ10_IRQn          ((IRQn_Type) 22) /* ICU IRQ10 (External pin interrupt 10) */
        #define VECTOR_NUMBER_ICU_IRQ11 ((IRQn_Type) 23) /* ICU IRQ11 (External pin interrupt 11) */
        #define ICU_IRQ11_IRQn          ((IRQn_Type) 23) /* ICU IRQ11 (External pin interrupt 11) */
        #define VECTOR_NUMBER_GPT0_COUNTER_OVERFLOW ((IRQn_Type) 24) /* GPT0 COUNTER OVERFLOW (Overflow) */
        #define GPT0_COUNTER_OVERFLOW_IRQn          ((IRQn_Type) 24) /* GPT0 COUNTER OVERFLOW (Overflow) */
        #define VECTOR_NUMBER_GPT0_CAPTURE_COMPARE_A ((IRQn_Type) 25) /* GPT0 CAPTURE COMPARE A (Compare match A) */
        #define GPT0_CAPTURE_COMPARE_A_IRQn          ((IRQn_Type) 25) /* GPT0 CAPTURE COMPARE A (Compare match A) */
        #ifdef __cplusplus
        }
        #endif
        #endif /* VECTOR_DATA_H */