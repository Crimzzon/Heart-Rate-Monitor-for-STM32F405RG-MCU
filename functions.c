

/*---------------------------------------------------------------------------------
    
    FILENAME:       functions.c

    HEADER:         __cross_studio_io.h, stm32f4xx.h

    DESCRIPTION:   This file contains the definitions of all the functions that are used to control 
                   input and output pins by writing to or reading from GPIOx registers.

                   This file contains the definitions of all the functions that are used for transmitting
                   receiving data using USART.

                   This file contains the definitions of all the functions that are used to 
                   configure and initialize the Timer in different modes: timer mode, 
                   input capture mode, and output compare mode, both with interrupts and without interrupts.

    REFERENCES:    STM32F405 Datasheet
    
    AUTHORS:        Habib Islam
                    Michael Reid
                    Grant Tofani

---------------------------------------------------------------------------------*/

#include "include.h"

/*------------------------------------------------------------------------------------------------
    
    FUNCTION:       void GPIOx_Init(GPIO_TypeDef* GPIOx, 
                                    uint32_t GPIOx_CLK_EN_bit_mask, 
                                    uint32_t GPIOx_pin_number,
                                    uint32_t GPIOx_mode,  
                                    uint32_t periph_AF_number, 
                                    uint32_t GPIOx_output_type,
                                    uint32_t GPIOx_output_speed,
                                    uint32_t GPIOx_pupd)

    DESCRIPTION:    This function configures GPIOx (x = A, B, C, D, etc.) pins in an arbitrary GPIO mode
                    so that digital I/O pins can be used by any arbitrary STM32F405 peripheral.

    PARAMETERS:     GPIOx - Pointer to the base address of GPIOC
                    GPIOx_pin_number - GPIOx pin number
                    GPIOx_mode - Selected Mode (e.g. Input, Output, Analog, Alternate Function)
                    periph_AF_number - AF number associated with the selected peripheral
                    GPIOx_output_type - GPIOx output type (e.g. push-pull, open-drain)
                    GPIOx_output_speed - GPIOx output speed (e.g. low, medium, high)
                    GPIOx_pupd - GPIOx pull-up/pull-down mode

    RETURNS:        None

    AUTHOR:         Habib Islam
    
-------------------------------------------------------------------------------------------------*/

void GPIOx_Init(GPIO_TypeDef* GPIOx, 
                uint32_t GPIOx_CLK_EN_bit_mask, 
                uint32_t GPIOx_pin_number,
                uint32_t GPIOx_mode,  
                uint32_t periph_AF_number, 
                uint32_t GPIOx_output_type,
                uint32_t GPIOx_output_speed,
                uint32_t GPIOx_pupd)
{
 /* Enable GPIOx clock by setting corresponding EN bit in RCC_AHB1ENR register */
 RCC->AHB1ENR |= GPIOx_CLK_EN_bit_mask;
 
 /* Clear mode of the selected pin by clearing corresponding bit in MODER register */
 GPIOx->MODER &= ~(3 << 2*GPIOx_pin_number);
 
 /* Set the desired mode for the selected GPIOx pin */
 GPIOx->MODER |= GPIOx_mode << 2*GPIOx_pin_number;
 
 /* If GPIOx mode is Alternate Function, configure GPIOx_AFRL (AFR[0]) 
    and GPIOx_AFRH (AFR[1]) registers for the GPIOx pin */
 if (GPIOx_mode == 2)
 {
  /* If the GPIOx pin number is 0 to 7 */
  if ((GPIOx_pin_number >= 0) && (GPIOx_pin_number <= 7))
  {
   /* Clear the 4 bits in AFRL register associated with the GPIOx pin */
   GPIOx->AFR[0] &= ~(0xF << 4*GPIOx_pin_number);

   /* Write the AF number in corresponding 4 bits in GPIOx_AFRL register */
   GPIOx->AFR[0] |= periph_AF_number << 4*GPIOx_pin_number;
  }

  /* If the GPIOx pin number is 8 to 16*/
  else if ((GPIOx_pin_number >= 8) && (GPIOx_pin_number <= 16))
  {
   /* Clear the 4 bits in AFRL register associated with the GPIOx pin */
   GPIOx->AFR[1] &= ~(0xF << (4*GPIOx_pin_number - 32));

   /* Write the AF number in corresponding 4 bits in GPIOx_AFRL register */
   GPIOx->AFR[1] |= periph_AF_number << (4*GPIOx_pin_number - 32);
  }
 }
 
 /* Clear output type by clearing the bit associated with GPIOx pin in GPIOx_OTYPER register */
 GPIOx->OTYPER &= ~(1 << GPIOx_pin_number);

 /* Select the desired output type by setting the corresponding bit in GPIOx_OTYPER register */
 GPIOx->OTYPER |= GPIOx_output_type << GPIOx_pin_number;
 
 /* Clear output speed by clearing the two bits associated with GPIOx pin in GPIOx_OSPEEDR register */
 GPIOx->OSPEEDR &= ~(3 << 2*GPIOx_pin_number);

  /* Select the desired output speed by writing the desired speed to the corresponding bits in GPIOx_OTYPER register */
 GPIOx->OSPEEDR |= GPIOx_output_speed << 2*GPIOx_pin_number;
 
 /* Clear pull-up/pull-down by clearing the two bits associated with GPIOx pin in GPIOx_PUPDR register */
 GPIOx->PUPDR &= ~(3 << 2*GPIOx_pin_number);

  /* Select the desired pull-up/pull-down type by writing the desired type to the corresponding bits in GPIOx_OTYPER register */
 GPIOx->PUPDR |= GPIOx_pupd << 2*GPIOx_pin_number;

} /* End of GPIOx_Init */

/*---------------------------------------------------------------------------------
    FUNCTION:       void USARTx_Init(uint32_t USART_num, 
                                     USART_TypeDef* USARTx,  
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
                                     uint32_t priority)

    DESCRIPTION:    Configures and Initializes any USART using a configurable interrupt

    PARAMETERS:     USART_num - number index for USART in STM32F405RG, number = 1, 2, 3, .......etc
                    USARTx - pointer to the base address of USARTx, x = 1, 2, 3, etc.
                    USARTx_CLK_EN_bit_mask - Enables USARTx clock
                    word_length - character length
                    number_stop_bits - number of stop bits
                    parity - parity enable/disable
                    parity_select - even/odd parity
                    oversampling - 8x/16x oversampling
                    baud_rate - baud rate
                    dma_tx - enables/disables DMA transmit
                    dma_rx - enables/disables DMA receive
                    interrupt_mask - USARTx bit mask for enabling desired interrupt
                    USARTx_IRQn - USARTx IRQ number
                    priority - Desired interrupt priority for USARTx
    
    RETURNS:        None
---------------------------------------------------------------------------------*/

/* Function prototype declaration */
void USARTx_Init(uint32_t USART_num, 
                 USART_TypeDef* USARTx,  
                 uint32_t USARTx_CLK_EN_bit_mask, 
                 uint32_t word_length,            /* 00 = 8 data bits, 01 = 9 data bits */
                 uint32_t num_stop_bits,          /* 00 = 1 stop bit, 01 = 0.5 stop bit, 10 = 2 stop bits, 11 = 1.5 stop bits */
                 uint32_t parity,                 /* 0 = parity disabled, 1 = parity enabled */
                 uint32_t parity_select,          /* 0 = even parity, 1 = odd parity */
                 uint32_t oversampling,           /* 0 = 16x oversampling, 1 = 8x oversampling */
                 uint32_t baud_rate,              /* Baud Rate = f_PCLK/(8*(2-OVER8)*USARTDIV) */
                 uint32_t dma_tx,                 /* 0 = TX DMA disabled, 1 = TX DMA enabled */
                 uint32_t dma_rx,                 /* 0 = RX DMA disabled, 1 = RX DMA enabled */
                 uint32_t interrupt_mask,         /* TXE = Bit[7], TC = Bit[6], RXNE = Bit[5] */
                 uint32_t USARTx_IRQn, 
                 uint32_t priority
                )

{
 /* If USART2 or USART3 is selected, enable its clock by setting 
    the corresponding bit in RCC_APB1ENR register */
 if ((USART_num == 2)|(USART_num == 3))
 {
  RCC->APB1ENR |= USARTx_CLK_EN_bit_mask;
 }
 
 /* If USART1 or USART6 is selected, enable its clock by setting 
    the corresponding bit in RCC_APB2ENR register */
 else if ((USART_num == 1)|(USART_num == 6))
 {
  RCC->APB2ENR |= USARTx_CLK_EN_bit_mask;
 }

 USARTx->CR1 &= ~(1 << 13); // Disable USART

 USARTx->CR1 &= ~(3 << 12); // clear M filed

 USARTx->CR1 |= word_length << 12; // set the word length

 USARTx->CR2 &= ~(3 << 12); // clear STOP field (Bits[13:12])

 USARTx->CR2 |= num_stop_bits << 12; // set the number of stop bits

 USARTx->CR1 &= ~(1 << 10); // clear PCE field (Bit[10])

 USARTx->CR1 |= parity << 10; // enable/disable parity control

 USARTx->CR1 &= ~(1 << 9); // clear PS field (Bit[9])

 USARTx->CR1 |= parity_select << 9; // select the desired parity 

 USARTx->CR1 &= ~(1 << 15); // clear OVER8 field (Bit[15])

 USARTx->CR1 |= oversampling << 15; // set the desired oversampling

 USARTx->BRR |= baud_rate; // set the desired baud rate

 USARTx->CR3 &= ~(1 << 7); // cleart DMAT field (Bit[7])

 USARTx->CR3 |= dma_tx << 7; // enable/disable DMA for transmit

 USARTx->CR3 &= ~(1 << 6); // cleart DMAR field (Bit[6])

 USARTx->CR3 |= dma_rx << 6; // enable/disable DMA for receive

 USARTx->CR1 |= 3 << 2; // Enable TX and RX

 USARTx->CR1 |= 1 << 13; // Enable USART

 USARTx->CR1 &= ~interrupt_mask; // clear desired interrupt bit fields
 
 USARTx->CR1 |= interrupt_mask; // enable desired interrupt

 NVIC_SetPriority(USARTx_IRQn, priority); // set the priority

 NVIC_EnableIRQ(USARTx_IRQn); // Enable IRQ for USARTx
}

/*---------------------------------------------------------------------------------
    FUNCTION:       void USARTx_Write_Str(USART_TypeDef * USARTx, uint8_t * str)

    DESCRIPTION:    Transmits a string of characters using USART

    PARAMETERS:     USARTx - Pointer to the base address of USARx
                    str - Pointer to the string to be sent

    RETURNS:        None

    AUTHOR:         Habib Islam
---------------------------------------------------------------------------------*/

/* Function prototype declaration */
void USARTx_Write_Str(USART_TypeDef *USARTx, uint8_t *str)

/* Beginning of function definition */
{
 uint32_t i = 0; /* Declare and initialize an index for the string elements */

 for(i = 0; i < strlen(str); i++)
 {
  /* Wait until TC (Transmission Complete) bit (Bit[6]) in USART_SR register is set */
     
  while(!(USARTx->SR & USART_SR_TC)); /* while((USARTx->SR & (1 << 6) == 0) */

  /* Write a character to be transmitted to the Data Register (USART_DR) */

  USARTx->DR = (str[i] & 0xFF);
 
 } /* End of for(i = 0; i < strlen(str); i++) loop */

}   /* End of USARTx_Write_Str */



/* FUNCTION:     void TIMx_Init_Timer(TIM_TypeDef * TIMx,
                                      uint32_t APB_number,
                                      uint32_t TIMx_CLK_EN_bit_mask,
                                      uint32_t TIMx_PSC_val,
                                      uint32_t TIMx_ARR_val,
                                      uint32_t TIMx_DIER_mask,
                                      uint32_t TIMx_IRQn,
                                      uint32_t TIMx_interrupt_priority)
   
   DESCRIPTION:   Configures and initializes any Timer channel to
                  operate in timer mode only. These timers include TIM2, TIM3, TIM4, 
                  TIM5, TIM6, TIM7, TIM12, TIM13, and TIM14. This function can be used to
                  generate a specified amount of delay or create a timer. It does not require
                  to configure any GPIOx pin as a Timer pin.

   PARAMETERS:    TIMx - Base Address of selected timer is a pointer to TC_TypeDef structure
                  APB_number - APB bus number (APB1 or APB2) the selected timer is connected with
                  TIMx_CLK_EN_bit_mask - RCC_APB1 bit mask to enable the selected timer
                  TIMx_PSC_val - Prescaler value
                  TIMx_ARR_val - Auto Reload Register (ARR) value
                  TIMx_DIER_mask - Desired interrupt mask for the selected timer
                  TIMx_IRQn - IRQ number of the selected timer
                  TIMx_interrupt_priority - Desired interrupt priority level for the selected timer
     
   RETURNS:       None

----------------------------------------------------------------------------------------------*/

//#include "mcro_include.h"

/* Function prototype */

void TIMx_Init_Timer(TIM_TypeDef * TIMx,
                     uint32_t APB_number,
                     uint32_t TIMx_CLK_EN_bit_mask,
                     uint32_t TIMx_PSC_val,
                     uint32_t TIMx_ARR_val,
                     uint32_t TIMx_DIER_mask,
                     uint32_t TIMx_IRQn,
                     uint32_t TIMx_interrupt_priority)
{
  /* Function definition starts */
 
 /* If the selected timer is connected to APB1 bus */
 if(APB_number == APB1)
 {
  /* Enable TIMx clock by setting the corresponding bit in RCC_APB1ENR register */
  RCC->APB1ENR |= TIMx_CLK_EN_bit_mask;
 }

 /* Else if the selected timer is connected to APB2 bus */
 if(APB_number == APB2)
 {
  /* Enable TIMx clock by setting the corresponding bit in RCC_APB1ENR register */
  RCC->APB2ENR |= TIMx_CLK_EN_bit_mask;
 }

 /* Disable TIMx before configuring its register by clearing CEN (Bit[0]) in TIMx_CR1 register */
 TIMx->CR1 &= ~TIM_CR1_CEN;

 /* Write the desired prescaler value in TIMx_PSC register */
 TIMx->PSC = TIMx_PSC_val;

 /* Write the desired ARR value to TIMx_ARR register */
 TIMx->ARR = TIMx_ARR_val;

 /* Enable the desired interrupts of the selected timer by setting the 
    bits associated with the desired interrupts in TIMx_DIER register */
 TIMx->DIER |= TIMx_DIER_mask;

 /* Clear the current value of the counter */
 TIMx->CNT = 0;

 /* Enable the timer channel by setting the CEN (Bit[0]) in TIMx_CR1 register */
 TIMx->CR1 |= TIM_CR1_CEN;
 
 /* Set the interrupt priority of the selected timer */
 NVIC_SetPriority(TIMx_IRQn, TIMx_interrupt_priority);

 /* Enable the interrupt of the selected timer */
 NVIC_EnableIRQ(TIMx_IRQn);

} /* End of TIMx_Init_Timer() */

/*--------------------------------------------------------------------------------------------------------------------------------------------------*/

/* FUNCTION:     void TIMx_Init_Capture_Compare(TIM_TypeDef * TIMx,
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
                                                uint32_t TIMx_interrupt_priority)
   
   DESCRIPTION:   Configures and initializes any Timer channel to operate both in
                  input capture and outpue compare modes. These timers include TIM2, TIM3, TIM4, 
                  TIM5, TIM6, TIM7, TIM12, TIM13, and TIM14. This function can be used for any
                  applications of input capture or output compare.

   PARAMETERS:    TIMx - Base Address of selected timer is a pointer to TC_TypeDef structure
                  APB_number - APB bus number (APB1 or APB2) the selected timer is connected with
                  TIMx_CLK_EN_bit_mask - RCC_APB1ENR bit mask for enabling/disabling TIMx clock
                  TIMx_PSC_val - Prescaler value to be written to TIMx_PSC register
                  TIMx_ARR_val - Auto Reload Register (ARR) value to be written to TIMx_ARR register
                  TIMx_ch_num - Channel number for the selected timer
                  TIMx_CCMRy_mask - TIMx_CCMRy bit masks (y = 1, 2)
                  TIMx_CCRy_val - Value to be written to TIMx_CCRy register (y = 1, 2, 3, 4)
                  TIMx_CCER_mask - TIMx_CCER bit mask to enable/disable input/output channel
                  TIMx_SMCR_mask - TIMx_SMCR bit mask to enable/disable external clock
                  TIMx_EGR_mask - TIMx_EGR bit mask to enable/disable even generation in PWM
                  TIMx_DIER_mask - TIMx_DIER bit mask for the the selected interrupts
                  TIMx_IRQn - IRQ number of the selected timer
                  TIMx_interrupt_priority - Desired interrupt priority level for the selected timer
     
   RETURNS:       None

------------------------------------------------------------------------------------------------------------------------------------------------*/

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
                               uint32_t TIMx_interrupt_priority) 
{
 /* If TIMx is connected to APB1 */
 if(APB_number == 1)
 {
  /* Enable TIMx clock using RCC_APB1ENR register */
  RCC->APB1ENR |= TIMx_CLK_EN_bit_mask;
 }

 /* If TIMx is connected to APB2 */
 else if(APB_number == 2)
 {
  /* Enable TIMx clock using RCC_APB2ENR register */
  RCC->APB2ENR |= TIMx_CLK_EN_bit_mask;
 }

 /* Disable TIMx before configuring its registers */
 TIMx->CR1 &= ~TIM_CR1_CEN;

 /* Write the desired PSC value to TIMx_PSC register */
 TIMx->PSC = TIMx_PSC_val;

 /* Write the desired ARR value to TIMx_PSC register */
 TIMx->ARR = TIMx_ARR_val;
 
 /* If TIMx channel number is TIMx_CH1 */
 if (TIMx_ch_number == 1)
 {
  /* Clear OC1M, CC1S, IC1F, and IC1PSC bits in TIMx_CCMR1 register */
  TIMx->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_CC1S | TIM_CCMR1_IC1F | TIM_CCMR1_IC1PSC);
  
  /* Configure CCMR1 register with desired parameters */
  TIMx->CCMR1 |= TIMx_CCMRy_mask;

  /* Clear CC1NP (Bit[3]) in TIMx_CCER register */
  TIMx->CCER &= ~TIM_CCER_CC1NP;
  
  /* Write desired value for TIMx_CCR register */
  TIMx->CCR1 = TIMx_CCRy_val;
 
 }
 
 /* If TIMx channel number is TIMx_CH2 */
 else if (TIMx_ch_number == 2)
 {
  /* Clear OC1M, CC1S, IC1F, and IC1PSC bits in TIMx_CCMR1 register */
  TIMx->CCMR1 &= ~(TIM_CCMR1_OC2M | TIM_CCMR1_CC2S | TIM_CCMR1_IC2F | TIM_CCMR1_IC2PSC);
  
  /* Configure CCMR1 register with desired parameters */
  TIMx->CCMR1 |= TIMx_CCMRy_mask;

  /* Clear CC1NP (Bit[3]) in TIMx_CCER register */
  TIMx->CCER &= ~TIM_CCER_CC2NP;
  
  /* Write desired value for TIMx_CCR register */
  TIMx->CCR2 = TIMx_CCRy_val;
 
 }
 
 /* If TIMx channel number is TIMx_CH3 */
 else if (TIMx_ch_number == 3)
 {
  /* Clear OC1M, CC1S, IC1F, and IC1PSC bits in TIMx_CCMR1 register */
  TIMx->CCMR2 &= ~(TIM_CCMR2_OC3M | TIM_CCMR2_CC3S | TIM_CCMR2_IC3F | TIM_CCMR2_IC3PSC);
  
  /* Configure CCMR1 register with desired parameters */
  TIMx->CCMR2 |= TIMx_CCMRy_mask;

  /* Clear CC1NP (Bit[3]) in TIMx_CCER register */
  TIMx->CCER &= ~TIM_CCER_CC3NP;
  
  /* Write desired value for TIMx_CCR register */
  TIMx->CCR3 = TIMx_CCRy_val;
 
 } 
 
 /* If TIMx channel number is TIMx_CH4 */
 else if (TIMx_ch_number == 4)
 {
  /* Clear OC1M, CC1S, IC1F, and IC1PSC bits in TIMx_CCMR1 register */
  TIMx->CCMR2 &= ~(TIM_CCMR2_OC4M | TIM_CCMR2_CC4S | TIM_CCMR2_IC4F | TIM_CCMR2_IC4PSC);
  
  /* Configure CCMR1 register with desired parameters */
  TIMx->CCMR2 |= TIMx_CCMRy_mask;

  /* Clear CC1NP (Bit[3]) in TIMx_CCER register */
  TIMx->CCER &= ~TIM_CCER_CC4NP;
  
  /* Write desired value for TIMx_CCR register */
  TIMx->CCR4 = TIMx_CCRy_val;
 
 }

 /* Clear TIMx_CNT register */
 TIMx->CNT = 0;
 
 /* Clear desired bits in TIMx_SMCR */
 TIMx->SMCR &= ~TIMx_SMCR_mask;
 
 /* Set desired bits in TIMx_SMCR */
 TIMx->SMCR |= TIMx_SMCR_mask;
 
 /* Clear desired bits in TIMx_EGR */
 TIMx->EGR &= ~TIMx_EGR_mask;
 
 /* Set desired bits in TIMx_EGR */
 TIMx->EGR |= TIMx_EGR_mask;
 
 /* Clear desired bits in TIMx_CCER */
 TIMx->CCER &= ~TIMx_CCER_mask;
 
 /* Set desired bits in TIMx_CCER */
 TIMx->CCER |= TIMx_CCER_mask;
 
 /* Clear desired bits in TIMx_DIER */
 TIMx->DIER &= ~TIMx_DIER_mask;

 /* Configure TIMx_DIER to enable the desired interrupts */
 TIMx->DIER |= TIMx_DIER_mask;

 /* Enable TIMx */
 TIMx->CR1 |= TIM_CR1_CEN;
 
 /* Set the desired priority for TIMx */
 NVIC_SetPriority(TIMx_IRQn, TIMx_interrupt_priority);
 
 /* Enable NVIC level interrupt */
 NVIC_EnableIRQ(TIMx_IRQn);

} /* End of TIMx_Init_Capture_Compare() */

/*------------------------------------------------------------------------------------------------
   
   FUNCTION:    void TIM3_IRQHandler(void)

   DESCRIPTION: Defines the Interrupt Service Routine for TIM3 for measuring
                pulse width, period, and frequency of an input signal. 
   
                The function checks the UIF flag is TIM3_SR register and
                every time this flag is set, it generates an interrupt
                and increments counter overflow count.
                
                The function also checks the CC3IF flag in TIM3_SR register.
                When the flag is set, the function reads the the counter
                value loaded into TIM3_CCR3 register. This counter value
                is added with the overflow count to calculate the total
                number of counter counts during a pulse duration by
                subtracting the previous count from the current count.
                             
   HARDWARE:    Olimex prototyping board with STM32F405RG microcontroller.

------------------------------------------------------------------------------------------------*/

/* Initialize the gloabl variable g_tim3_total_count to 0 */
uint32_t g_tim3_total_count = 0;

/* Initialize the flag g_new_captured_data to 0 */
uint32_t g_new_captured_data = 0;

void TIM3_IRQHandler(void)
{
 /* Initialize the static variable prev_count to 0 */
 static uint32_t prev_count = 0;
 
 /* Intialize the static variable overflow_count to 0 */
 static uint32_t overflow_count = 0;
 
 /* Initialize the static variable signal_polarity to 0.
    This flag is used to make sure that the calcualtion
    of the difference of total counter counts is done 
    between a rising edge and the next falling edge (or
    between two rising edges), NOT between a falling edge 
    and the next rising edge. */
 static uint32_t signal_polarity = 0;
 
 /* Define the integer variable ccr_val to store total counter counts */
 uint32_t ccr_val;
 
 /* If the UIF flag is set */
 if(TIM3->SR & TIM_SR_UIF)
 {
  /* Clear UIF flag by clearing Bit[0] of TIM3_SR */
  TIM3->SR &= ~TIM_SR_UIF;

  /* Increment the overflow count */
  overflow_count++;
 
 } /* End of if(TIM3->SR & TIM_SR_UIF) */
 
 /* If CC3IF flag is set */
 if(TIM3->SR & TIM_SR_CC3IF)
 {
  /* Read the value of TIM3_CCR3 register into ccr_val */
  ccr_val = TIM3->CCR3;

  /* Add overflow count to ccr_val */
  ccr_val |= overflow_count << 16;

  /* Change the signal polarity flag */
  signal_polarity = 1 - signal_polarity;

  /* Calculate the difference in counter values only
     when the current input is low */
  if(signal_polarity == 0)
  {
   /* Subtract previous count from current count */
   g_tim3_total_count = ccr_val - prev_count;
  
  } /* End of if(signal_polarity == 0) */

  /* Make the current count as a previous count */
  prev_count = ccr_val;

  /* Set the flag for new measurement */
  g_new_captured_data = 1;
 
 } /* End of if(TIM3->SR & TIM_SR_CC3IF) */

}  /* End of TIM3_IRQHandler() */



void I2Cx_Init(I2C_TypeDef * I2Cx,
               uint32_t I2Cx_CLK_EN_bit_mask,
               uint32_t I2Cx_CR2_mask,
               uint32_t I2Cx_speed,
               uint32_t I2Cx_rise_time,
               uint32_t I2Cx_IRQn,
               uint32_t I2Cx_priority)

{
 RCC->APB1ENR |= I2Cx_CLK_EN_bit_mask;

 I2Cx->CR1 |= I2C_CR1_SWRST;   // reset the software

 I2Cx->CR1 &= ~I2C_CR1_SWRST;   // Set the software

 I2Cx->CR1 &= ~I2C_CR1_PE;    // disable I2Cx

 I2Cx->CR1 &= ~I2C_CR1_SMBUS;  // select I2C mode

 I2Cx->CR2 &= ~I2C_CR2_FREQ;   // clear frequency

 I2Cx->CR2 |= I2Cx_CR2_mask;   // set the desired frequency

 I2Cx->CCR &= ~I2C_CCR_CCR;    // clear SCL clok

 I2Cx->CCR |= I2Cx_speed;      // set the desired clock

 I2Cx->TRISE &= ~I2C_TRISE_TRISE;  // clear rise time

 I2Cx->TRISE |= I2Cx_rise_time;   // set the desired rise time

 I2Cx->CR1 |= I2C_CR1_PE;      // enable I2Cx

 NVIC_SetPriority(I2Cx_IRQn, I2Cx_priority); // set interrupt priority

 NVIC_EnableIRQ(I2Cx_IRQn);  // enable interrupt
}

/*------------------------------------------------------------------------------------------------
   
   FUNCTION:    void LCD_write_instruction(I2C_TypeDef * I2Cx,
                           uint8_t cmd)

   DESCRIPTION: Function used to communicate with the Display to make sending desired data to 
                the LCD easier. Description is as follows in the comments
                             
   HARDWARE:    Olimex prototyping board with STM32F405RG microcontroller.

------------------------------------------------------------------------------------------------*/
void LCD_write_instruction(I2C_TypeDef * I2Cx,
                           uint8_t cmd) {
  
  volatile uint32_t dump;

  while(I2Cx->SR2 & I2C_SR2_BUSY); // wait until the bus is free

  I2Cx->CR1 |= I2C_CR1_START;     // send the start bit

  while(!(I2Cx->SR1 & I2C_SR1_SB)); // wait until start bit flag is set

  I2Cx->DR = LCD_ID<<1;  // send the device address

  while(!(I2Cx->SR1 & I2C_SR1_ADDR)); // wait until the address flag is set

  dump = I2Cx->SR2;  // clear address flag

  while(!(I2Cx->SR1 & I2C_SR1_TXE));  // wait until the data register is empty

  I2Cx->DR = cmd;   // send the first byte of data

  while(!(I2Cx->SR1 & I2C_SR1_BTF)); // wait until byte transfer is complete

  I2Cx->CR1 |= I2C_CR1_STOP;  // send the stop bit

  delay(0xff);
}