/* generated vector header file - do not edit */
        #ifndef VECTOR_DATA_H
        #define VECTOR_DATA_H
        #ifdef __cplusplus
        extern "C" {
        #endif
                /* Number of interrupts allocated */
        #ifndef VECTOR_DATA_IRQ_COUNT
        #define VECTOR_DATA_IRQ_COUNT    (8)
        #endif
        /* ISR prototypes */
        void spi_b_rxi_isr(void);
        void spi_b_txi_isr(void);
        void spi_b_tei_isr(void);
        void spi_b_eri_isr(void);
        void sci_b_uart_rxi_isr(void);
        void sci_b_uart_txi_isr(void);
        void sci_b_uart_tei_isr(void);
        void sci_b_uart_eri_isr(void);

        /* Vector table allocations */
        #define VECTOR_NUMBER_SPI1_RXI ((IRQn_Type) 0) /* SPI1 RXI (Receive buffer full) */
        #define SPI1_RXI_IRQn          ((IRQn_Type) 0) /* SPI1 RXI (Receive buffer full) */
        #define VECTOR_NUMBER_SPI1_TXI ((IRQn_Type) 1) /* SPI1 TXI (Transmit buffer empty) */
        #define SPI1_TXI_IRQn          ((IRQn_Type) 1) /* SPI1 TXI (Transmit buffer empty) */
        #define VECTOR_NUMBER_SPI1_TEI ((IRQn_Type) 2) /* SPI1 TEI (Transmission complete event) */
        #define SPI1_TEI_IRQn          ((IRQn_Type) 2) /* SPI1 TEI (Transmission complete event) */
        #define VECTOR_NUMBER_SPI1_ERI ((IRQn_Type) 3) /* SPI1 ERI (Error) */
        #define SPI1_ERI_IRQn          ((IRQn_Type) 3) /* SPI1 ERI (Error) */
        #define VECTOR_NUMBER_SCI1_RXI ((IRQn_Type) 4) /* SCI1 RXI (Received data full) */
        #define SCI1_RXI_IRQn          ((IRQn_Type) 4) /* SCI1 RXI (Received data full) */
        #define VECTOR_NUMBER_SCI1_TXI ((IRQn_Type) 5) /* SCI1 TXI (Transmit data empty) */
        #define SCI1_TXI_IRQn          ((IRQn_Type) 5) /* SCI1 TXI (Transmit data empty) */
        #define VECTOR_NUMBER_SCI1_TEI ((IRQn_Type) 6) /* SCI1 TEI (Transmit end) */
        #define SCI1_TEI_IRQn          ((IRQn_Type) 6) /* SCI1 TEI (Transmit end) */
        #define VECTOR_NUMBER_SCI1_ERI ((IRQn_Type) 7) /* SCI1 ERI (Receive error) */
        #define SCI1_ERI_IRQn          ((IRQn_Type) 7) /* SCI1 ERI (Receive error) */
        #ifdef __cplusplus
        }
        #endif
        #endif /* VECTOR_DATA_H */