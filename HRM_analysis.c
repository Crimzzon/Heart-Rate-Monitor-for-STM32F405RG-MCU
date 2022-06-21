/*-------------------------------------------------------------------------------------------------------------------------
  FILE NAME:   HRM_analysis.c

  HEADER:      mcro_include.h

  DESCRIPTION: Function is used to gather heart rate data using capture and compare method
               First pin PC8 is set to be an input then the input capture compare speed is set.

               New data is gathered and time is taken from the inout capture compare clock then is 
               converted to period and then frequency.

               Frequency is the converted to BPM (HRM)

               Irregular data is denied. Under 50 and above 150 are not within the scope of our 
               project and they only showed up as noise so tehy are simply ignored.

               A counter is incremented every time a valid measurment is stored in the 
               HRM_A[] array to a maximum of 10. Once ten samples is reached a for loop averages
               the results.

               The loop is broken and the counter is set back to zero. The USART display is used
               purely for trouble shooting.

               Once the data is stored in the output variable (PREV_HRM) it returns that value back 
               to the LCD_Display function.



  REFERENCES:  STM32F405xx data sheet

  AUTHOR:      Habib Islam

  Modifed and commented by Michael Reid

/*-----------------------------------------------------------------------------------------------------------*/

#include "include.h"

int HRM_analysis(void)
{
 /* Message prompt for the user */
 uint8_t msg[] = "\r\n\n\tSample Collection...\r\n\n";
 
 /* Display the message on Teraterm */
 USARTx_Write_Str(USART2, msg);
 
 //Vaiables are not global becasue reseting all of the is imperative to the accuracy of the results.

 double period = 0.0; 

 double frequency = 0.0;

 double HRM = 0.0;

 double HRM_A[50] = {0};

 double PREV_HRM = 60; // value used for trouble shooting purposes
 
 int count = 0;

 int i = 0;

 /* Initialize the period of TIM3 counter clock */
 double tim3_clock_period = 0.0;

 /* Initialize the char array for displaying the data */
 uint8_t display[MAX_STR_LEN] = {0};

 /* Configure PC8 as TIM3 input pin */

 GPIOx_Init(GPIOC,                 /* Base address of GPIOC is a pointer to GPIO_TypeDef structure */

            RCC_AHB1ENR_GPIOCEN,   /* Enable GPIOC clock by setting Bit[2] of RCC_AHB1ENR register */

            PC8,                   /* Select GPIOx pin to be PC8 */

            GPIO_MODE_AF,          /* Configure alternate function mode by writing 0b10 to Bits[17:16] in GPIOC_MODER register */

            2,                     /* Select AF2 for TIM3 by writing 0b0010 to Bits[3:0] in GPIOC_AFRH register */

            GPIO_OTYPE_PUSH_PULL,  /* Configure PC8 output type as PUSH-PULL by writing 0 to Bit[8] in GPIOC_OTYPER register */

            GPIO_OSPEED_HIGH,      /* Configure PC8 output speed HIGH by writing 0b10 to Bits[17:16] in GPIOC_OSPEEDR register */

            GPIO_PUPD_NO_PUPD      /* Select NO PULL-UP PULL-DOWN by writing 0b00 to Bits[17:16] in GPIOC_PUPDR register */
           );

 /* Initialize TIM3_CH3 in input capture mode */

 TIMx_Init_Capture_Compare(TIM3,                      /* TIM_TypeDef* TIMx = TIM3. Base address of TIM3 is a pointer
                                                         to TIM_TypeDef structure*/

                           APB1,                      /* APB_number = APB1 */

                           RCC_APB1ENR_TIM3EN,        /* Enable TIM3 clock by writing 1 to Bit[1] in RCC_APB1ENR register */

                           419,                       /* Configure TIM3 conter clock frequency to be 100 kHz by writing 419 to TIM3_PSC register. 
                                                         Counter clock frequency = 42 MHz/(419 + 1) = 100 kHz, 
                                                         Counter clock period = 1/(100 kHz) = 10 micro seconds*/

                           0xFFFF,                    /* Configure counter overflow duration to be 3.57 micro seconds by writing 149 to TIM3_ARR register 
                                                         Overflow duration = counter clock period * (ARR + 1) = 10 micro seconds * 65536 = 655.36 ms */

                           3,                         /* TIMx_ch_num = TIM3_CH3 = 3 */

                           1 << 0,                    /* TIMx_CCMRy_mask = 1 << 0 */
                                                      /* Map CC3 on TI3 by setting CC3S (Bit[0]) of TIM3_CCMR2 register */

                           
                           0,                         /* TIMx_CCRy_val = 0. (In input capture mode CCR is read-only) */

                           0 << 9 | 0 << 11 |         /* TIMx_CCER_mask = 1 << 9 | 1 << 11. Configure edge detection to 
                                                        capture on both rising and falling edges by setting
                                                        CC3P (Bit[9]) and CC3NP (Bit[11]) in TIM3_CCER register */

                           TIM_CCER_CC3E,             /* Enable TIM3_CH3 capture/compare by setting Bit[8] in TIM3_CCER register */



                           0,                         /* Configure for no external clock by clearing all bits in TIM3_SMCR register */

                           0,                        /* TIMx_EGR_mask = 0. Configure for no PWM Update Generation by 
                                                        clearing Bit[0] in TIM3_EGR register */

                           TIM_DIER_UIE|             /* TIMx_DIER_mask = 1 << 0. Enable Update Interrupt Enable flag
                                                        by setting UIE (Bit[0]) of TIM3_DIER register */

                           TIM_DIER_CC3IE,           /* TIMx_DIER_mask = 1 << 3. Enable Capture Compare 3 Interrupt Enable flag
                                                        by setting CC3IE (Bit[3]) of TIM3_CCER register */

                           TIM3_IRQn,                /* TIMx_IRQn = TIM3_IRQn. Enable interrupt for TIM3. */

                           0                          /* TIMx_interrupt_priority = 0 (highest priority) */
                          
                          ); 
 
 /* Calculate TIM3 clock period */
 tim3_clock_period = (double)(419 + 1)/APB1_CLK;

 int toggle = 1;

 while(toggle)
 {
  /* If new measurement is done, i.e. the g_new_captured_data flag is set */
  delay(0xFF);

  if(g_new_captured_data)
  {
   /* Clear g_new_captured_data flag */
   g_new_captured_data = 0; 

   /* Calculate the pulse width */
   period = (double)(tim3_clock_period*g_tim3_total_count);

   frequency = 1.0/period;

   HRM = frequency * 60.0;
   
   if(HRM>50 && HRM<150)
   {
    HRM_A[i] = HRM;
    i++;
   }

   if(i==1)
   {

    for(count=0;count<10;count++)
    {
      HRM = HRM_A[count] + HRM;
    }

    HRM=HRM/11;
    PREV_HRM=HRM;
    i=0;
    toggle = 0;

   }
   /* Convert the integer data into string to display on Teraterm */
   sprintf(display, "\r\t%f",PREV_HRM);
   
   /* Display pulse width information on teraterm */
   USARTx_Write_Str(USART2, display);
  
  } /* End of if(g_new_captured_data) */
 
 }  /* End of while(1) */

 return PREV_HRM;

}   /* End of TIM3_measures_pulse_width() */










































































