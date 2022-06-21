#ifndef H_MCRO_DECLARE
#define H_MCRO_DECLARE

void delay(uint32_t num_tick);

void GPIOx_Init(GPIO_TypeDef* GPIOx,
                uint32_t GPIOx_CLK_EN_BIT_MASK,
                uint32_t GPIOx_pin_number,
                uint32_t GPIOx_mode,
                uint32_t periph_AF_number,
                uint32_t GPIOx_output_type,
                uint32_t GPIOx_output_speed,
                uint32_t GPIOx_pupd);

void USARTx_Init(uint32_t USART_num,
                 USART_TypeDef * USARTx,
                 uint32_t USARTx_CLK_EN_bit_mask,
                 uint32_t word_length,
                 uint32_t num_stop_bits,
                 uint32_t parity,
                 uint32_t parity_select,
                 uint32_t oversampling,
                 uint32_t baud_rate,
                 uint32_t dma_tx,
                 uint32_t dma_rx,
                 uint32_t interrupt_mask,
                 uint32_t USARTx_IRQn,
                 uint32_t priority);

void USARTx_Write_Str(USART_TypeDef * USARTx, uint8_t *str);

void USART2_transmits_message(void);

void TIMx_Init_Capture_Compare(TIM_TypeDef * TIMx,
                               uint32_t APB_number,
                               uint32_t TIMx_CLK_EN_bit_mask,
                               uint32_t TIMx_PSC_val,
                               uint32_t TIMx_ARR_val,
                               uint32_t TIMx_ch_number,
                               uint32_t TIMx_CCMRy_mask,
                               uint32_t TIMx_CCRy_val,
                               uint32_t TIMx_CCER_mask,
                               uint32_t TIMx_SMCR_mask,
                               uint32_t TIMx_EGR_mask,
                               uint32_t TIMx_DIER_mask,
                               uint32_t TIMx_IRQn,
                               uint32_t TIMx_interrupt_priority);

int HRM_analysis(void);

void LCD_Display(void);

void I2Cx_Init(I2C_TypeDef * I2Cx,
               uint32_t I2Cx_CLK_EN_bit_mask,
               uint32_t I2Cx_CR2_mask,
               uint32_t I2Cx_speed,
               uint32_t I2Cx_rise_time,
               uint32_t I2Cx_IRQn,
               uint32_t I2Cx_priority);
     
void LCD_write_instruction(I2C_TypeDef * I2Cx,
                           uint8_t cmd);


#endif
