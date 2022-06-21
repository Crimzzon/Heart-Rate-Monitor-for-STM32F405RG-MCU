#ifndef H_MCRO_DEFINE
#define H_MCRO_DEFINE

#define ESC  (27)

/* Declare the mask for the bit corresponding to pin connected with board LED.
   Board LED is connected with PC12 */
#define LED_MASK (1 << 12)

/* Declare the mask for the bit corresponding to the pin connected with board push button */
/* The only push button on the board is connected with PB0 */
#define PUSH_BUTTON_MASK (1 << 0)

/* Declare the mask for the bit associated with turning on/off GPIOC peripheral clock */
/* Bit[2] of RCC_AHB1ENR register is associated with GPIOC */
#define GPIOC_CLK_EN_BIT_MASK (1 << 2)

/* Declare the mask for the bit associated with turning on/off GPIOA peripheral clock */
/* Bit[0] of RCC_AHB1ENR register is associated with GPIOC */
#define GPIOA_CLK_EN_BIT_MASK (1 << 0)

#define GPIOB_CLK_EN_BIT_MASK (1 << 1)

#define USART2_CLK_EN_BIT_MASK (1 << 17)

#define SPI_MASTER_MODE (1 << 2)

#define SPI_SLAVE_MODE (0 << 2)

#define SPI_CPOL1 (1 << 1)

#define SPI_CPOL0 (0 << 1)

#define SPI_CPHASE1 (1 << 0)

#define SPI_CPHASE0 (0 << 0)

#define SPI_LSBFIRST (1 << 7)

#define SPI_MSBFIRST (0 << 7)

#define SPI_SSM (1 << 9)

#define SPI_SSI (1 << 8)

#define SPI_DFF8 (0 << 11)

#define SPI_DFF16 (1 << 11)

#define SPI_RXNE_MASK (1 << 0)

#define SPI_MASTER (0)

#define SPI_SLAVE (1)

#define VALID_STATE (4)

#define BUF_LEN (256)
#define MAX_STR_LEN (50)

#define APB1  (1)
#define APB2  (2)

#define APB1_CLK (42000000)
#define APB2_CLK (84000000)
#define AHB_CLK (168000000)

#define ADC_VREF (3.3)
#define ADC_RES_12 (0xFFF)
#define ADC_RES_10 (0x3FF)
#define ADC_RES_8 (0xFF)
#define ADC_RES_6 (0x3F)

#define SAMPLE_SIZE (256)
#define PI (3.14159265358979323)

#define GPIO_MODE_INPUT               (0)
#define GPIO_MODE_OUTPUT              (1)
#define GPIO_MODE_AF                  (2)
#define GPIO_MODE_ANALOG              (3)

#define GPIO_OTYPE_PUSH_PULL          (0)
#define GPIO_OTYPE_OPEN_DRAIN         (1)

#define GPIO_OSPEED_LOW               (0)
#define GPIO_OSPEED_MEDIUM            (1)
#define GPIO_OSPEED_HIGH              (2)
#define GPIO_OSPEED_VERY_HIGH         (3)

#define GPIO_PUPD_NO_PUPD             (0)
#define GPIO_PUPD_PULL_UP             (1)
#define GPIO_PUPD_PULL_DOWN           (2)

/* Define port pins */

#define PA0       (0)
#define PA1       (1)
#define PA2       (2)
#define PA3       (3)
#define PA8       (8)

#define PB0       (0)
#define PB1       (1)
#define PB2       (2)
#define PB5       (5)
#define PB8       (8)
#define PB9       (9)
#define PB10      (10)
#define PB11      (11)
#define PB12      (12)
#define PB13      (13)
#define PB14      (14)
#define PB15      (15)

#define PC0       (0)
#define PC1       (1)
#define PC2       (2)
#define PC3       (3)
#define PC4       (4)
#define PC5       (5)
#define PC6       (6)
#define PC7       (7)
#define PC8       (8)
#define PC9       (9)
#define PC10      (10)
#define PC12      (12)
#define PC13      (13)

#define PD2       (2)

#define LCD_ID  (0x72)

#define NUM_BYTES (4)

#endif
