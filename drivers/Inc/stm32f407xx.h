/*
 * stm32f407xx.h
 *
 *  Created on: Apr 16, 2025
 *      Author: doanh
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>


#define _vo volatile

/* start : processor specific details
 * arm cotex Mx processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0   ((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1   ((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2   ((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3   ((volatile uint32_t*)0xE000E10C)


/*
 *  ARM cortex Mx processor NVIC ICERx register addresses
 */
#define NVIC_ICER0    ((_vo uint32_t*)0xE000E180)
#define NVIC_ICER1    ((_vo uint32_t*)0xE000E184)
#define NVIC_ICER2    ((_vo uint32_t*)0xE000E188)
#define NVIC_ICER3    ((_vo uint32_t*)0xE000E18C)


/*
 *  ARM Cortex Mx processor priority register address calculation
 */
#define NVIC_PR_BASE_ADDR  ((_vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED        4

/*
 * base addresses of flash and SRAM memories
 */

#define FLASH_BASEADDR  0X08000000U
#define SRAM1_BASEADDR  0X20000000U
#define SRAM2_BASEADDR  0x20001C00U
#define ROM_BASEADDR    0x1FFF0000U
#define SRAM            SRAM1_BASEADDR


/*
 * AHBx and APBx Bus peripheral base addresses
 */

#define PERIPH_BASE  0X40000000U
#define APB1PERIPH_BASE  PERIPH_BASE
#define APB2PERIPH_BASE  0x40010000U
#define AHB1PERIPH_BASE  0x40020000U
#define AHB2PERIPH_BASE  0x50000000U

/*
 * base addresses of peripherals which are hanging on AHB1 bus
 * TODO : complete for all other peripherals
 */

#define GPIOA_BASEADDR  (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR  (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR  (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR  (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR  (AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR  (AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR  (AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR  (AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR  (AHB1PERIPH_BASE + 0X2000)
#define RCC_BASEADDR    (AHB1PERIPH_BASE + 0X3800)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * TODO : complete for all peripherals
 */
#define I2C1_BASEADDR  (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR  (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR  (APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR  (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR  (APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR  (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR  (APB1PERIPH_BASEADDR + 0x4800)
#define USART4_BASEADDR  (APB1PERIPH_BASEADDR + 0x4C00)
#define USART5_BASEADDR  (APB1PERIPH_BASEADDR + 0x5000)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * TODO : Complete for all other peripherals
 */

#define EXTI_BASEADDR  (APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR  (APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR  (APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR  (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR  (APB2PERIPH_BASEADDR + 0x1400)



/* peripheral register definitoin structure*/


typedef struct
{
	_vo uint32_t MODER;         /*!< give a short description,      address offset: 0x00 */
	_vo uint32_t OTYPER;        /*!< TODO,                          address offset: 0x04 */
	_vo uint32_t OSPEEDR;
	_vo uint32_t PUPDR;
	_vo uint32_t IDR;
	_vo uint32_t ODR;
	_vo uint32_t BSRR;
	_vo uint32_t LCKR;
	_vo uint32_t AFR[2];        /*!< AFR[0] : GPIO alternate function low register; AF[1] : GPIO alternate function high register    address offset : 0x20-0x24 */
}GPIO_RegDef_t;

/*
 * peripheral register definition structure for RCC
 */

typedef struct
{
	_vo uint32_t CR;
	_vo uint32_t PLLCFRG;
	_vo uint32_t CFGR;
	_vo uint32_t AHB1RSTR;
	_vo uint32_t AHB2RSTR;
	_vo uint32_t AHB3RSTR;
	uint32_t     RESERVED0;
	_vo uint32_t APB1RSTR;
	_vo uint32_t APB2RSTR;
	uint32_t     RESERVED1[2];
	_vo uint32_t AHB1ENR;
	_vo uint32_t AHB2ENR;
	_vo uint32_t AHB3ENR;
	uint32_t     RESERVED2;
	_vo uint32_t APB1ENR;
	_vo uint32_t APB2ENR;
	uint32_t     RESERVED3[2];
	_vo uint32_t AHB1LPENR;
	_vo uint32_t AHB2LPENR;
	_vo uint32_t AHB3LPENR;
	uint32_t     RESERVED4;
	_vo uint32_t APB1LPENR;
	_vo uint32_t APB2LPENR;
	uint32_t     RESERVED5[2];
	_vo uint32_t BDCR;
	_vo uint32_t CSR;
	uint32_t     RESERVED6[2];
	_vo uint32_t SSCGR;
	_vo uint32_t PLLI2SCFGR;
	_vo uint32_t PLLSAICFGR;
	_vo uint32_t DCKCFGR;
	_vo uint32_t CKGATENR;
	_vo uint32_t DCKCFGR2;

}RCC_RegDef_t;



/*
 * peripheral register definition structure for EXTI
 */

typedef struct
{
	_vo uint32_t IMR;         /*!< give a short description,      address offset: 0x00 */
	_vo uint32_t EMR;        /*!< TODO,                          address offset: 0x04 */
	_vo uint32_t RTSR;
	_vo uint32_t FTSR;
	_vo uint32_t SWIER;
	_vo uint32_t PR;

}EXTI_RegDef_t;


/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	_vo uint32_t MEMRMP;         /*!< give a short description,      address offset: 0x00 */
	_vo uint32_t PMC;        /*!< TODO,                          address offset: 0x04 */
	_vo uint32_t EXTICR[4];
	uint32_t     RESERVED1[2];
	_vo uint32_t CMPCR;
	uint32_t     RESERVED2[2];
	_vo uint32_t CFGR;

} SYSCFG_RegDef_t;


/*
 * peripheral definitions (peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA  ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB  ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF  ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG  ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH  ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI  ((GPIO_RegDef_t*)GPIOI_BASEADDR)


#define RCC  ((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI ((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/*
 * clock enable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()  ( RCC->AHB1ENR |= ( 1 << 0) )
#define GPIOB_PCLK_EN()  ( RCC->AHB1ENR |= ( 1 << 1) )
#define GPIOC_PCLK_EN()  ( RCC->AHB1ENR |= ( 1 << 2) )
#define GPIOD_PCLK_EN()  ( RCC->AHB1ENR |= ( 1 << 3) )
#define GPIOE_PCLK_EN()  ( RCC->AHB1ENR |= ( 1 << 4) )
#define GPIOF_PCLK_EN()  ( RCC->AHB1ENR |= ( 1 << 5) )
#define GPIOG_PCLK_EN()  ( RCC->AHB1ENR |= ( 1 << 6) )
#define GPIOH_PCLK_EN()  ( RCC->AHB1ENR |= ( 1 << 7) )
#define GPIOI_PCLK_EN()  ( RCC->AHB1ENR |= ( 1 << 8) )

/*
 * clock enable macros for I2Cx peripherals
 */

#define I2C1_PCKC_EN()    ( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCKC_EN()    ( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCKC_EN()    ( RCC->APB1ENR |= (1 << 23) )

/*
 * clock enable macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()   ( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()   ( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()   ( RCC->APB1ENR |= (1 << 15) )
#define SPI4_PCLK_EN()   ( RCC->APB2ENR |= (1 << 13) )

/*
 * clock enable macros for USARTx peripherals
 */
#define USART1_PCCK_EN()  (RCC->APB2ENR |= (1 << 4))
#define USART2_PCCK_EN()  (RCC->APB1ENR |= (1 << 17))
#define USART3_PCCK_EN()  (RCC->APB1ENR |= (1 << 18))
#define USART4_PCCK_EN()  (RCC->APB1ENR |= (1 << 19))
#define USART5_PCCK_EN()  (RCC->APB1ENR |= (1 << 20))
#define USART6_PCCK_EN()  (RCC->APB1ENR |= (1 << 5))
/*
 * clock enable macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN()  (RCC->APB2ENR |= (1 << 14) )


/*
 * clock enable macros for GPIOx peripherals
 */


/*
 * clock disable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()


/*
 * clock disable macros for I2Cx peripherals
 */


/*
 * clock disable macros for SPIx peripherals
 */


/*
 * clock disable macros for USARTx peripherals
 */

/*
 * clock disable macros for SYSCFG peripheral
 */


/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()      do{ ( RCC->AHB1RSTR |= (1 << 0));    ( RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOB_REG_RESET()      do{ ( RCC->AHB1RSTR |= (1 << 1));    ( RCC->AHB1RSTR &= ~(1 << 1));} while(0)
#define GPIOC_REG_RESET()      do{ ( RCC->AHB1RSTR |= (1 << 2));    ( RCC->AHB1RSTR &= ~(1 << 2));} while(0)
#define GPIOD_REG_RESET()      do{ ( RCC->AHB1RSTR |= (1 << 3));    ( RCC->AHB1RSTR &= ~(1 << 3));} while(0)
#define GPIOE_REG_RESET()      do{ ( RCC->AHB1RSTR |= (1 << 4));    ( RCC->AHB1RSTR &= ~(1 << 4));} while(0)
#define GPIOF_REG_RESET()      do{ ( RCC->AHB1RSTR |= (1 << 5));    ( RCC->AHB1RSTR &= ~(1 << 5));} while(0)
#define GPIOG_REG_RESET()      do{ ( RCC->AHB1RSTR |= (1 << 6));    ( RCC->AHB1RSTR &= ~(1 << 6));} while(0)
#define GPIOH_REG_RESET()      do{ ( RCC->AHB1RSTR |= (1 << 7));    ( RCC->AHB1RSTR &= ~(1 << 7));} while(0)
#define GPIOI_REG_RESET()      do{ ( RCC->AHB1RSTR |= (1 << 8));    ( RCC->AHB1RSTR &= ~(1 << 8));} while(0)

/*
 * this macros return a code (between 0 to 7) for a given GPIO base address(x)
 */

#define GPIO_BASEADDR_TO_CODE(x)   (  (x == GPIOA) ? 0 : \
		                             (x == GPIOB) ? 1 : \
		                             (x == GPIOC) ? 2 : \
		                             (x == GPIOD) ? 3 : \
		                             (x == GPIOE) ? 4 : \
		                             (x == GPIOF) ? 5 : \
		                             (x == GPIOG) ? 6 : \
		                             (x == GPIOH) ? 7 : \
                                     (x == GPIOH) ? 8 : 0)

/*
 * IRQ number of stm32f407x MCU
 * NOTE : update these macros with valid values according to your MCU
 * TODO : you may complete this list for other peripherals
 */

#define IRQ_NO_EXTI0      6
#define IRQ_NO_EXTI1      7
#define IRQ_NO_EXTI2      8
#define IRQ_NO_EXTI3      9
#define IRQ_NO_EXTI4      10
#define IRQ_NO_EXTI9_5    23
#define IRQ_NO_EXTI15_10  40

/*
 * macros for all the possible priority  levels
 */
#define NVIC_IRQ_PRI0 0
#define NVIC_IRQ_PRI15 15
#define NVIC_IRQ_PRI9_5 13

/*
 * Macros for all the posssible priority levels
 */
#define NVIC_IRQ_PRI0 0
#define NVIC_IRQ_PRI15 15



// NVIC function declarations
void NVIC_EnableIRQ(uint8_t IRQn);
void NVIC_SetPriority(uint8_t IRQn, uint8_t priority);


// some generic macros

#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET
#define FLAG_SET SET
#define FLAG_SET SET

#define APB2PERIPH_BASEADDR     0x40010000U

/*
 * Bit position definitions of SPI peripheral
 */


/*
 *  Bit position definitions SPI_CR1
 */

#define SPI_CR1_CPHA      0
#define SPI_CR1_CPOL      1
#define SPI_CR1_MSTR      2
#define SPI_CR1_BR        3
#define SPI_CR1_SPE       6
#define SPI_CR1_LSBFIRST  7
#define SPI_CR1_SSI       8
#define SPI_CR1_SSM       9
#define SPI_CR1_RXONLY    10
#define SPI_CR1_DFF       11
#define SPI_CR1_CRCNEXT   12
#define SPI_CR1_CRCEN     13
#define SPI_CR1_BIDIOE    14
#define SPI_CR1_BIDIMODE  15


/*
 *  Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN    0
#define SPI_CR2_TXDMAEN    1
#define SPI_CR2_SSOE       2
#define SPI_CR2_FRF        4
#define SPI_CR2_ERRIE      5
#define SPI_CR2_RXNEIE     6
#define SPI_CR2_TXEIE      7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE        0
#define SPI_SR_TXE         1
#define SPI_SR_CHSIDE      2
#define SPI_SR_UDR         3
#define SPI_SR_CRCERR      4
#define SPI_SR_MODF        5
#define SPI_SR_OVR         6
#define SPI_SR_BSY         7
#define SPI_SR_FRE         8


#include "stm32f407xx_gpio_driver.h"


#endif /* INC_STM32F407XX_H_ */
