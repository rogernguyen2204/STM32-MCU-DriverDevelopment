/*
 * stm32f4xx.h
 *
 *  Created on: Mar 28, 2023
 *      Author: nguye
 */

#ifndef INC_STM32F4XX_H_
#define INC_STM32F4XX_H_

#include <stddef.h>
#include <stdint.h>

#define __vo volatile
#define __weak     __attribute__((weak))

/**************************************** Processor Specific Details - ARM Cortex M4 ***********************
 *
 * ARM Cortex M4 Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0        ((__vo uint32_t*)0xE000E100UL)
#define NVIC_ISER1        ((__vo uint32_t*)0xE000E104UL)
#define NVIC_ISER2        ((__vo uint32_t*)0xE000E108UL)
#define NVIC_ISER3        ((__vo uint32_t*)0xE000E10CUL)

/*
 * ARM Cortex M4 Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0        ((__vo uint32_t*)0XE000E180UL)
#define NVIC_ICER1        ((__vo uint32_t*)0xE000E184UL)
#define NVIC_ICER2        ((__vo uint32_t*)0xE000E188UL)
#define NVIC_ICER3        ((__vo uint32_t*)0xE000E18CUL)

/*
 * ARM Cortex M4 Processor Priority Register Addresses Calculation
 */

#define NVIC_PR_BASE_ADDR ((__vo uint32_t*)0xE000E400UL)

#define NO_PR_BITS_IMPLEMENTED        (4)

// base addresses of FLASH and SRAM memory
#define FLASH_BASEADDR          0x08000000UL      /*Flash base address*/
#define SRAM1_BASEADDR          0x20000000UL      /*SRAM1 base address*/
#define SRAM2                   0x20001C00UL     /*SRAM2 base address (12kb after SRAM1*/
#define SRAM                    SRAM1_BASEADDR   /*SRAM = SRAM1*/
#define ROM_BASEADDR            0x1FFF0000UL      /*System memory/ROM base address*/

// macros for different bus domain for the micro controller

#define PERIPH_BASEADDR         0x40000000UL
#define APB1PERIPH_BASEADDR     PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR     0x40010000UL
#define AHB1PERIPH_BASEADDR	    0x40020000UL
#define AHB2PERIPH_BASEADDR     0x50000000UL

// define base address of peripherals which are hanging on AHB1 bus (GPIO ONLY)
#define GPIOA_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR          (AHB1PERIPH_BASEADDR + 0x2000)

#define RCC_BASEADDR            (AHB1PERIPH_BASEADDR + 0x3800)

// define addresses of peripherals which are hanging on APB1 bus
#define I2C1_BASEADDR           (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR           (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR           (APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR           (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR           (APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR         (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR         (APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR          (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR          (APB1PERIPH_BASEADDR + 0x5000)

// define address of peripherals which are hanging on APB2 bus

#define EXTI_BASEADDR           (APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR           (APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR         (APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR         (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR         (APB2PERIPH_BASEADDR + 0x1400)


/*********************peripheral register definition structures*****************/

// 1. GPIO (General purpose Input/Output) peripheral register definition structures

typedef struct{
	__vo uint32_t MODER;       // GPIO port mode register (GPIOx_MODER)
	__vo uint32_t OTYPER;      // GPIO port output type register (GPIOx_OTYPER)
	__vo uint32_t OSPEEDR;     // GPIO port output speed register (GPIOx_OSPEEDR)
	__vo uint32_t PUPDR;       // GPIO port pull-up/pull-down register (GPIOx_PUPDR)
	__vo uint32_t IDR;         // GPIO port input data register (GPIOx_IDR)
	__vo uint32_t ODR;         // GPIO port output data register (GPIOx_ODR)
	__vo uint32_t BSRR;        // GPIO port bit set/reset register (GPIOx_BSRR)
	__vo uint32_t LCKR;        // GPIO port configuration lock register (GPIOx_LCKR)
	__vo uint32_t AFR[2];      // GPIO alternate function  register (GPIOx_AFRL & GPIOx_AFRH)
}GPIO_RegDef_t;

//2. RCC (Reset and Clock control) peripheral register definition structures
typedef struct{
   __vo uint32_t RCC_CR;            // RCC clock control register 0x00
   __vo uint32_t RCC_PLLCFGR;       // RCC PLL configuration register 0x04
   __vo uint32_t RCC_CFGR;          // RCC clock configuration register 0x08
   __vo uint32_t RCC_CIR;           // RCC clock interrupt register 0x0C
   __vo uint32_t RCC_AHB1RSTR;      // RCC AHB1 peripheral reset register 0x10
   __vo uint32_t RCC_AHB2RSTR;      // RCC AHB2 peripheral reset register 0x14
   __vo uint32_t RCC_AHB3RSTR;      // RCC AHB3 peripheral reset register 0x18
        uint32_t RESERVED1;         // 0x1C
   __vo uint32_t RCC_APB1RSTR;      // RCC APB1 peripheral reset register 0x20
   __vo uint32_t RCC_APB2RSTR;      // RCC APB2 peripheral reset register 0x24
        uint32_t RESERVED2;         // 0x28
        uint32_t RESERVED3;         // 0x2C
   __vo uint32_t RCC_AHB1ENR;       // RCC AHB1 clock enable register 0x30
   __vo uint32_t RCC_AHB2ENR;       // RCC AHB2 clock enable register 0x34
   __vo uint32_t RCC_AHB3ENR;       // RCC AHB3 clock enable register 0x38
        uint32_t RESERVED4;         // 0x3C
   __vo uint32_t RCC_APB1ENR;       // RCC APB1 clock enable register 0x40
   __vo uint32_t RCC_APB2ENR;       // RCC APB2 clock enable register 0x444
        uint32_t RESERVED5;         // 0x48
        uint32_t RESERVED6;         // 0x4C
   __vo uint32_t RCC_AHB1LPENR;     // RCC AHB1 clock enable low power mode 0x50
   __vo uint32_t RCC_AHB2LPENR;     // RCC AHB2 clock enable low power mode 0x54
   __vo uint32_t RCC_AHB3LPENR;     // RCC AHB3 clock enable low power mode 0x58
        uint32_t RESERVED7;         // 0x5C
   __vo uint32_t RCC_APB1LPENR;     // RCC APB1 clock enable low power mode 0x60
   __vo uint32_t RCC_APB2LPENR;     // RCC APB2 clock enable low power mode 0x64
        uint32_t RESERVED8;         // 0x68
        uint32_t RESERVED9;         // 0x6C
   __vo uint32_t RCC_BDCR;          // RCC Backup domain control register 0x70
   __vo uint32_t RCC_CSR;           // RCC clock control and status register 0x74
        uint32_t RESERVED10;        // 0x78
        uint32_t RESERVED11;        //0x7C
   __vo uint32_t RCC_SSCGR;         // RCC spread spectrum clock generation register 0x80
   __vo uint32_t RCC_PLLI2SCFGR;    // RCC PLLI2S 0x84
   __vo uint32_t RCC_PLLSAICFGR;    // Address offset: 0x88
   __vo uint32_t RCC_DCKCFGR;       // Address offset: 0x8C
   __vo uint32_t RCC_CKGATENR;      // Address offset: 0x90
   __vo uint32_t RCC_DCKCFGR2;      // Address offset: 0x94

}RCC_RegDef_t;

// 3. EXTI (External Interrupt Event Controller) peripheral register structure

typedef struct{
	__vo uint32_t EXTI_IMR;          // Interrupt mask register 0x00
    __vo uint32_t EXTI_EMR;          // Event mask register 0x04
	__vo uint32_t EXTI_RTSR;         // Rising trigger selection register 0x08
	__vo uint32_t EXTI_FTSR;         // Falling trigger selection register 0x0C
	__vo uint32_t EXTI_SWIER;        // Software interrupt event register 0x10
	__vo uint32_t EXTI_PR;           // Pending register 0x14

}EXTI_RegDef_t;

// 4. SYSCFG (System configuration controller) peripheral register structure

typedef struct{
	__vo uint32_t SYSCFG_MEMRMP;          // Memory remap register 0x00
	__vo uint32_t SYSCFG_PMC;             // Peripheral mode configuration register 0x04
	__vo uint32_t SYSCFG_EXTICR[4];       // External interrupt configuration register 0x08-0x14
	__vo uint32_t SYSCFG_CMPCR;           // Compensation cell control register

}SYSCFG_RegDef_t;

// 5. SPI (Serial Peripheral Interface) peripheral register structure
typedef struct {
	__vo uint32_t SPI_CR1;           // SPI control register 1 0x00
	__vo uint32_t SPI_CR2;           // SPI control register 2 0x04
	__vo uint32_t SPI_SR;            // SPI status control register 0x08
	__vo uint32_t SPI_DR;            // SPI data register 0x0C
	__vo uint32_t SPI_CRCPR;         // SPI CRC polynomial register (not used in I2S mode) 0x10
	__vo uint32_t SPI_RXCRCR;        // SPI RX CRC register (not used in I2S mode) 0x14
	__vo uint32_t SPI_TXCRCR;        // SPI TX CRC register (not used in I2S mode) 0x18
	__vo uint32_t SPI_I2SCFGR;       // SPI I2S configuration register 0x1C
	__vo uint32_t SPI_I2SPR;         // SPI_I2S prescaler register 0x20
}SPI_RegDef_t;


//6 . I2C (Inter-Integrated Circuit) peripheral register structure
typedef struct {
	__vo uint32_t I2C_CR1;          // I2C control register 1 0x00
	__vo uint32_t I2C_CR2;          // I2C control register 2 0x04
	__vo uint32_t I2C_OAR1;         // I2C own address register 1 0x08
	__vo uint32_t I2C_OAR2;         // I2C own address register 2 0x0C
	__vo uint32_t I2C_DR;           // I2C data register 0x10
	__vo uint32_t I2C_SR1;          // I2C status register 1 0x14
	__vo uint32_t I2C_SR2;          // I2C status register 2 0x18
	__vo uint32_t I2C_CCR;          // I2C clock control register 0x1C
	__vo uint32_t I2C_TRISE;        // I2C TRISE register 0x20
	__vo uint32_t I2C_FLTR;         // I2C FLTR register 0x24
}I2C_RegDef_t;

/*********************Peripheral definitions*****************/

// peripheral definitions (peripheral base address typecasted to xxx_RegDef_t

#define GPIOA        ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB        ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC        ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD        ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE        ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF        ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG        ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH        ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI        ((GPIO_RegDef_t*)GPIOI_BASEADDR)

// RCC peripheral definitions (peripheral base address typecasted to xxx_RegDef_t)

#define RCC          ((RCC_RegDef_t*) RCC_BASEADDR)

// EXTI peripheral definitions (peripheral base address typecasted to xxx_RegDef_t)
#define EXTI         ((EXTI_RegDef_t*) EXTI_BASEADDR)

// SYSCFG peripheral definitions (peripheral base address typecasted to xxx_RegDef_t)
#define SYSCFG       ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

// SPI peripheral definitions (peripheral base address typecasted to xxx_RegDef_t)
#define SPI1         ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2         ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3         ((SPI_RegDef_t*)SPI3_BASEADDR)

// I2C peripheral definitions (peripheral base address typecasted to xxx_RegDef_t)
#define I2C1         ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2         ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3         ((I2C_RegDef_t*)I2C3_BASEADDR)
/*********************Clock enable macros*****************/

//Clock enable macros for GPIOx

#define GPIOA_PCLK_EN    (RCC->RCC_AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN    (RCC->RCC_AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN    (RCC->RCC_AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN    (RCC->RCC_AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN    (RCC->RCC_AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN    (RCC->RCC_AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN    (RCC->RCC_AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN    (RCC->RCC_AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN    (RCC->RCC_AHB1ENR |= (1 << 8))

// Clock enable macros for I2C peripheral
#define I2C1_PCLK_EN       (RCC->RCC_APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN       (RCC->RCC_APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN       (RCC->RCC_APB1ENR |= (1 << 23))
// Clock enable macros for SPI peripheral
#define SPI1_PCLK_EN       (RCC->RCC_APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN       (RCC->RCC_APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN       (RCC->RCC_APB1ENR |= (1 << 15))
// Clock enable macros for USART peripheral
#define USART2_PCLK_EN     (RCC->RCC_APB1ENR |= (1 << 17))
#define USART3_PCLK_EN     (RCC->RCC_APB1ENR |= (1 << 18))
#define UART4_PCLK_EN      (RCC->RCC_APB1ENR |= (1 << 19))
#define UART5_PCLK_EN      (RCC->RCC_APB1ENR |= (1 << 20))
// Clock enable macros for SYSCFG peripheral
#define SYSCFG_PCLK_EN     (RCC->RCC_APB2ENR |= (1 << 14))


/*********************Clock disable macros*****************/
//Clock disable macros for GPIOx
#define GPIOA_PCLK_DI     (RCC->RCC_AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI     (RCC->RCC_AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI     (RCC->RCC_AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI     (RCC->RCC_AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI     (RCC->RCC_AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI     (RCC->RCC_AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI     (RCC->RCC_AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI     (RCC->RCC_AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI     (RCC->RCC_AHB1ENR &= ~(1 << 8))

// Clock disable macros for I2C
#define I2C1_PCLK_DI      (RCC->RCC_APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI      (RCC->RCC_APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI      (RCC->RCC_APB1ENR &= ~(1 << 23))

// Clock disable macros for SPI
#define SPI1_PCLK_DI      (RCC->RCC_APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI      (RCC->RCC_APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI      (RCC->RCC_APB1ENR &= ~(1 << 15))

// Clock disable macros for USART peripheral
#define USART2_PCLK_DI    (RCC->RCC_APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI    (RCC->RCC_APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI     (RCC->RCC_APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI     (RCC->RCC_APB1ENR &= ~(1 << 20))

// Clock disable macros for SYSCFG
#define SYSCFG_PCLK_DI    (RCC->RCC_APB2ENR &= ~(1 << 14))


// Macros to reset the GPIO peripherals
#define GPIOA_REG_RESET        do{(RCC->RCC_AHB1RSTR |= (1<<0)); (RCC->RCC_AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET        do{(RCC->RCC_AHB1RSTR |= (1<<1)); (RCC->RCC_AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET        do{(RCC->RCC_AHB1RSTR |= (1<<2)); (RCC->RCC_AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET        do{(RCC->RCC_AHB1RSTR |= (1<<3)); (RCC->RCC_AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET        do{(RCC->RCC_AHB1RSTR |= (1<<4)); (RCC->RCC_AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOF_REG_RESET        do{(RCC->RCC_AHB1RSTR |= (1<<5)); (RCC->RCC_AHB1RSTR &= ~(1<<5));}while(0)
#define GPIOG_REG_RESET        do{(RCC->RCC_AHB1RSTR |= (1<<6)); (RCC->RCC_AHB1RSTR &= ~(1<<6));}while(0)
#define GPIOH_REG_RESET        do{(RCC->RCC_AHB1RSTR |= (1<<7)); (RCC->RCC_AHB1RSTR &= ~(1<<7));}while(0)
#define GPIOI_REG_RESET        do{(RCC->RCC_AHB1RSTR |= (1<<8)); (RCC->RCC_AHB1RSTR &= ~(1<<8));}while(0)


// Macros to reset the SPI peripherals
#define SPI1_REG_RESET         do{(RCC->RCC_APB2RSTR |= (1<<12)); (RCC->RCC_APB2RSTR &= ~(1<<12));}while(0)
#define SPI2_REG_RESET         do{(RCC->RCC_APB1RSTR |= (1<<14)); (RCC->RCC_APB1RSTR &= ~(1<<14));}while(0)
#define SPI3_REG_RESET         do{(RCC->RCC_APB1RSTR |= (1<<15)); (RCC->RCC_APB1RSTR &= ~(1<<15));}while(0)

#define GPIO_BASEADDR_TO_CODE(x)         ((x==GPIOA)? 0 :\
		                                  (x==GPIOB)? 1 :\
										  (x==GPIOC)? 2 :\
										  (x==GPIOD)? 3 :\
										  (x==GPIOE)? 4 :\
										  (x==GPIOF)? 5 :\
										  (x==GPIOG)? 6 :\
									      (x==GPIOH)? 7 :\
										  (x==GPIOI)? 8 :0)

// Macros to reset I2C peripherals
#define I2C1_REG_RESET         do{(RCC->RCC_APB1RSTR |= (1<<21)); (RCC->RCC_APB1RSTR &= ~(1<<21));}while(0)
#define I2C2_REG_RESET         do{(RCC->RCC_APB1RSTR |= (1<<22)); (RCC->RCC_APB1RSTR &= ~(1<<22));}while(0)
#define I2C3_REG_RESET         do{(RCC->RCC_APB1RSTR |= (1<<23)); (RCC->RCC_APB1RSTR &= ~(1<<23));}while(0)
/*
 * IRQ (Interrupt Request) Numbers of STM32F407x MCU
 * NOTE: These macros can be updated for each MCU
 */

#define IRQ_NO_EXTI0           6
#define IRQ_NO_EXTI1           7
#define IRQ_NO_EXTI2           8
#define IRQ_NO_EXTI3           9
#define IRQ_NO_EXTI4           10
#define IRQ_NO_EXTI9_5         23
#define IRQ_NO_EXTI15_10       40
//SPI interrupt
#define IRQ_NO_SPI1            42
#define IRQ_NO_SPI2            43
#define IRQ_NO_SPI3            51

/*
 * macros for all the possible priority number
 */
#define NVIC_IRQ_PRI0          0
#define NVIC_IRQ_PRI15         15



// some generic macros
#define ENABLE                 1
#define DISABLE                0
#define SET                    ENABLE
#define RESET                  DISABLE
#define GPIO_PIN_SET           SET
#define GPIO_PIN_RESET         RESET
#define FLAG_RESET             RESET
#define FLAG_SET               SET


/************************************************************************
 *            BIT position definitions of SPI peripheral
 *************************************************************************/
// 1. SPI_CR1 register

#define SPI_CR1_CPHA        0
#define SPI_CR1_CPOL        1
#define SPI_CR1_MSTR        2
#define SPI_CR1_BR          3
#define SPI_CR1_SPE         6
#define SPI_CR1_LSB_FIRST   7
#define SPI_CR1_SSI         8
#define SPI_CR1_SSM         9
#define SPI_CR1_RXONLY      10
#define SPI_CR1_DFF         11
#define SPI_CR1_CRC_NEXT    12
#define SPI_CR1_CRC_EN      13
#define SPI_CR1_BIDIOE      14
#define SPI_CR1_BIDIMODE    15

// 2. SPI_CR2 regiter

#define SPI_CR2_RXDMAEN     0
#define SPI_CR2_TXDMAEN     1
#define SPI_CR2_SSOE        2
#define SPI_CR2_FRF         4
#define SPI_CR2_ERRIE       5
#define SPI_CR2_RXNEIE      6
#define SPI_CR2_TXEIE       7

// 3. SPI_SR regiter

#define SPI_SR_RXNE         0
#define SPI_SR_TXE          1
#define SPI_SR_CHSIDE       2
#define SPI_SR_UDR          3
#define SPI_SR_CRCERR       4
#define SPI_SR_MODF         5
#define SPI_SR_OVR          6
#define SPI_SR_BSY          7
#define SPI_SR_FRE          8

/************************************************************************
 *            BIT position definitions of I2C peripheral
 *************************************************************************/
//1. I2C_CR1

#define I2C_CR1_PE          0
#define I2C_CR1_NOSTRETCH   7
#define I2C_CR1_START       8
#define I2C_CR1_STOP        9
#define I2C_CR1_ACK         10
#define I2C_CR1_SWRST       15

//2. I2C_CR2

#define I2C_CR2_FREQ        0
#define I2C_CR2_ITERREN     8
#define I2C_CR2_ITEVTEN     9
#define I2C_CR2_ITBUFEN     10

//3. I2C_OAR1

#define I2C_OAR1_ADD0       0
#define I2C_OAR1_ADD71      1
#define I2C_OAR1_ADD98      8
#define I2C_OAR1_ANDMODE    15

//4. I2C_SR1
#define I2C_SR1_SB          0
#define I2C_SR1_ADDR        1
#define I2C_SR1_BTF         2
#define I2C_SR1_ADD10       3
#define I2C_SR1_STOPF       4
#define I2C_SR1_RXNE        6
#define I2C_SR1_TXE         7
#define I2C_SR1_BERR        8
#define I2C_SR1_ARLO        9
#define I2C_SR1_AF          10
#define I2C_SR1_OVR         11
#define I2C_SR1_PECERR      12
#define I2C_SR1_TIMEOUT     14

//5. I2C_SR2
#define I2C_SR2_MSL         0
#define I2C_SR2_BUSY        1
#define I2C_SR2_TRA         2
#define I2C_SR2_GENCALL     4
#define I2C_SR2_DUALF       7

//6. I2C_CCR
#define I2C_CCR_CRR         0
#define I2C_CCR_DUTY        14
#define I2C_CCR_FS          15




#endif /* INC_STM32F4XX_H_ */
