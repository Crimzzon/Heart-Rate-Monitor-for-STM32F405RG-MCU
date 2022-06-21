/*------------------------------------------------------------------------------------------------------------------------------
    FILENAME:       main.c

    DESCRIPTION:   The file main.c is the entry point to the application.
                   
                   It simply enables the global inturrup handler.

                   Then goes the the LCD_display function.

   HARDWARE:       Olimex prototyping board with STM32F405RG microcontroller

   AUTHOR:         Iouri Kourilov

   Modified and commented by Habib Islam and Michael Reid

------------------------------------------------------------------------------------------------------------------------------------*/

/* Include the header file mcro350_main.h that includes all the required header files */

#include "include.h"

int32_t main(void)
{
 /* Configure GPIOA registers to initialize PA2 as USART2 TxD pin */
 
 GPIOx_Init(GPIOA,                 /* Define a pointer that points to the base address of GPIOA */

           GPIOA_CLK_EN_BIT_MASK, /* Enable the clock of GPIOA by setting Bit[1] in RCC_AHB1ENR */

           2,                     /* GPIOA pin number 2 */

           2,                     /* Set pin 10 in AF mode by writing 2 to Bits[21:20] in GPIOB_MODER */

           7,                     /* AF number is 7 */

           0,                     /* Set output type to push-pull by clearing Bit[0] in GPIOA_OTYPER */

           2,                     /* Set GPIO speed as HIGH speed by writing 0b10 to Bits[1:0] in GPIOA_OSPEEDR */

           0                      /* Select "no pull-up pull-down by clearing Bits[1:0] in GPIOA_PUPDR */
           
           );
 
 /* Configure GPIOA registers to initialize PA3 as USART2 RxD pin */

 GPIOx_Init(GPIOA,                 /* Define a pointer that points to the base address of GPIOA */

           GPIOA_CLK_EN_BIT_MASK, /* Enable the clock of GPIOA by setting Bit[1] in RCC_AHB1ENR */

           3,                     /* GPIOA pin number 3 */

           2,                     /* Set pin 10 in AF mode by writing 2 to Bits[21:20] in GPIOB_MODER */

           7,                     /* AF number is 7 */

           0,                     /* Set output type to push-pull by clearing Bit[0] in GPIOA_OTYPER */

           2,                     /* Set GPIO speed as HIGH speed by writing 0b10 to Bits[1:0] in GPIOA_OSPEEDR */

           0                      /* Select "no pull-up pull-down by clearing Bits[1:0] in GPIOA_PUPDR */
           
           );

 
 /* Configure and initialize USART2 by calling USARTx_Init() function with the desired parameters*/

 USARTx_Init(2,                       /* Select USART2 */

             USART2,                  /* Pointer to the base address of USART2 */

             USART2_CLK_EN_BIT_MASK,  /* Set Bit[17] in RCC_APB1ENR to turn on USART2 peripheral clock */

             0,                       /* Configure for 8-bits of character length */

             0,                       /* Configure for 1 stop bit */

             0,                       /* Disable parity control */

             0,                       /* Doesn't matter since parity control is disabled */

             0,                       /* Configure for 16x oversampling */

             (273 << 4) | (7 << 0),   /* USARTDIV = f_PCLK/(8*(2-OVER8)*baud_rate) = 42 MHz/(8*(2-0)*9600 bps) = 273.4375
                                         Write Mantissa 273 to Bits[15:4] and fraction 0.4375*16 = 7 to Bits[3:0] in USART2_BRR */

             0,                       /* Disalbe DMA transmit */

             0,                       /* Disable DMA receive */

             USART_CR1_RXNEIE,        /* Configure for RXNE interrupt */

             USART2_IRQn,             /* IRQ number for USART2: doesn't matter since no interrupt is configured */

             0                        /* Highest priority */
            
            );
 
 __enable_irq();  // Enable global interrupt
   LCD_Display(); // Go to LCD_diplsay function

} /* End of main(void) */


/* Define the delay function. This function asks the processor 
   to wait for a specified amount of time before executing the
   next instruction. In the while loop, the processor wait
   until the selected number 'num_tick' counts down to 0. */

void delay(uint32_t num_tick)
{
 /* Wait until 'num_tick' counts down to 0 */
 while(num_tick--);
}















