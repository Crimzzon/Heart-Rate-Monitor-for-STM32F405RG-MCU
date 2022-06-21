/*------------------------------------------------------------------------------------------------------------------------------
    FILENAME:       LCD_Display.c

    DESCRIPTION:   The file LCD_Display handles the display ofthe information
                   on the LCD.
                   
                   It first initializes the pins for data output using I2C
                   PB10 is used for the Clock signal
                   PB11 is used for the data signal

   HARDWARE:       Olimex prototyping board with STM32F405RG microcontroller

   AUTHOR:         Michael Reid 
   
   Assistance given by Colton Tible

------------------------------------------------------------------------------------------------------------------------------------*/
#include "include.h"

void LCD_Display(void) 
{
  /* PB10 SCL */ 
   GPIOx_Init(GPIOB, RCC_AHB1ENR_GPIOBEN, 6, 2, 4, 1, 0, 0); // Setup PB10 to be the clock signal output using I2C
    
   /* PB11 SDA */ 
   GPIOx_Init(GPIOB, RCC_AHB1ENR_GPIOBEN, 7, 2, 4, 1, 0, 0); // Setup PB11 to be the data signal output using I2C


  I2Cx_Init(I2C1,                         // I2C1 SCL UEXT-5 & I2C1 SDA UEXT-6
            1 << 21,                      // I2C1 clock enabled
            42,                           // 41MHz peripheral clock
            I2C_CCR_FS | 0x24,            // 390KHz for the I2C clock
            0x0A,                         // 300ns rise time
            0,                            // no interrupt
            0);                           // N/A

  int HRM_DATA = 100; // Set initial value for HRM_DATA for trouble shooting purposes
  int toggle = 1; // define integer variable used to enable or disable while loop
  int modifier = 0; // to ensure the display does not have black box's a modifier is used 
                    // to change the display output signal by 1 or zero. This is needed
                    // becasue of the herat rate data being either 2 or 3 digits long
  uint8_t display[80] = {0}; // define number of characters max for LAC display size
  uint8_t display_format[] = "|-HeartRate %d"; // define message to be displayed
  while(toggle)
  {
    HRM_DATA = HRM_analysis(); // get value from HRM_analysis(); and store in HRM_DATA 
    delay(0xFF); // delay is set just so the display has time to be ready
    sprintf(display,display_format, HRM_DATA);//display format
    if(HRM_DATA>99)modifier=1; // 2 or 3 digit check
    else modifier = 0; // 2 or 3 digit check
    for(int i = 0; i<(sizeof(display_format)/sizeof(char))-1+modifier;i++) // for loop for printing character  string to LCD
    {
      LCD_write_instruction(I2C1,display[i]);  //writing each character to LCD
    }
  //toggle = 0;
  }
}
