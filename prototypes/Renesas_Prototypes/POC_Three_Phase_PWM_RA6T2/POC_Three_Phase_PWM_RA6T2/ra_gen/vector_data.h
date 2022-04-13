/* generated vector header file - do not edit */
        #ifndef VECTOR_DATA_H
        #define VECTOR_DATA_H
                /* Number of interrupts allocated */
        #ifndef VECTOR_DATA_IRQ_COUNT
        #define VECTOR_DATA_IRQ_COUNT    (2)
        #endif
        /* ISR prototypes */
        void poeg_event_isr(void);
        void adc_b_adi0_isr(void);

        /* Vector table allocations */
        #define VECTOR_NUMBER_POEG3_EVENT ((IRQn_Type) 0) /* POEG3 EVENT (Port Output disable interrupt D) */
        #define POEG3_EVENT_IRQn          ((IRQn_Type) 0) /* POEG3 EVENT (Port Output disable interrupt D) */
        #define VECTOR_NUMBER_ADC12_ADI0 ((IRQn_Type) 1) /* ADC0 ADI0 (End of A/D scanning operation(Gr.0)) */
        #define ADC12_ADI0_IRQn          ((IRQn_Type) 1) /* ADC0 ADI0 (End of A/D scanning operation(Gr.0)) */
        #endif /* VECTOR_DATA_H */