/*
 * stm32f401xx.h
 *
 *  Created on: Jun 16, 2025
 *      Author: Nikola Sokolović
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

#include <stdint.h>

#define __vo volatile

//Defining base addresses of Flash and SRAM
#define DRV_FLASH_BASEADDR					0x08000000U		//defining the base address of FLASH/Main memory
#define DRV_SRAM1_BASEADDR					0x20000000U		//defining the base address of SRAM1
#define DRV_ROM_BASEADDR					0x1FFF0000U		//defining the base address of ROM/System memory
#define DRV_SRAM 						DRV_SRAM1_BASEADDR	//there is only one SRAM on this controller


//Defining AHBx and APBx bus peripherals
#define DRV_PERIPH_BASEADDR					0x40000000U		//defining the base address of peripherals
#define DRV_APB1PERIPH_BASEADDR					DRV_PERIPH_BASEADDR	//APB1 is first
#define DRV_APB2PERIPH_BASEADDR					0x40010000U		//defining the base address for APB2
#define DRV_AHB1PERIPH_BASEADDR					0x40020000U		//defining the base address for AHB1
#define DRV_AHB2PERIPH_BASEADDR					0x50000000U		//defining the base address for AHB2


//Defining base addresses of peripherals connected to AHB1
//We define the addresses as the AHB1 base address + the offset for each GPIO
#define DRV_GPIOA_BASEADDR					(DRV_AHB1PERIPH_BASEADDR + 0x0000)
#define DRV_GPIOB_BASEADDR					(DRV_AHB1PERIPH_BASEADDR + 0x0400)
#define DRV_GPIOC_BASEADDR					(DRV_AHB1PERIPH_BASEADDR + 0x0800)
#define DRV_GPIOD_BASEADDR					(DRV_AHB1PERIPH_BASEADDR + 0x0C00)
#define DRV_GPIOE_BASEADDR					(DRV_AHB1PERIPH_BASEADDR + 0x1000)
#define DRV_GPIOH_BASEADDR					(DRV_AHB1PERIPH_BASEADDR + 0x1C00)
#define DRV_RCC_BASEADDR					(DRV_AHB1PERIPH_BASEADDR + 0x3800)


//Defining the base addresses of peripherals that are connected to the APB1 bus
#define DRV_I2C1_BASEADDR					(DRV_APB1PERIPH_BASEADDR + 0x5400)
#define DRV_I2C2_BASEADDR					(DRV_APB1PERIPH_BASEADDR + 0x5800)
#define DRV_I2C3_BASEADDR					(DRV_APB1PERIPH_BASEADDR + 0x5C00)
#define DRV_SPI2_BASEADDR					(DRV_APB1PERIPH_BASEADDR + 0x3800)
#define DRV_SPI3_BASEADDR					(DRV_APB1PERIPH_BASEADDR + 0x3C00)
#define DRV_USART2_BASEADDR					(DRV_APB1PERIPH_BASEADDR + 0x4400)


//Defining the base addresses of peripherals that are connected to the APB2 bus
#define DRV_SPI1_BASEADDR					(DRV_APB2PERIPH_BASEADDR + 0x3000)
#define DRV_SPI4_BASEADDR					(DRV_APB2PERIPH_BASEADDR + 0x3400)
#define DRV_USART1_BASEADDR					(DRV_APB2PERIPH_BASEADDR + 0x1000)
#define DRV_USART6_BASEADDR					(DRV_APB2PERIPH_BASEADDR + 0x1400)
#define DRV_SYSCFG_BASEADDR					(DRV_APB2PERIPH_BASEADDR + 0x3C00)
#define DRV_EXTI_BASEADDR					(DRV_APB2PERIPH_BASEADDR + 0x3C00)


/*************************** Peripheral register definition structure *********************************/
//Defining GPIO peripheral register structure
typedef struct
{
	__vo uint32_t MODER;				//Defining the port mode register					Address offset: 0x00
	__vo uint32_t OTYPER;				//Defining the output type of I/O port					Address offset: 0x04
	__vo uint32_t OSPEEDR;				//Defining the I/O output speed						Address offset: 0x08
	__vo uint32_t PUPDR;				//Defining the I/O as pull up/down 					Address offset: 0x0C
	__vo uint32_t IDR;				//These registers can only be read					Address offset: 0x10
	__vo uint32_t ODR;				//to be added								Address offset: 0x14
	__vo uint32_t BSRR;				//to be added								Address offset: 0x18
	__vo uint32_t LCKR;				//Defining the port configuration locked reg				Address offset: 0x1C
	__vo uint32_t AFR[2];				//AFR[0] is the alternate function low reg, 				Address offset: 0x20
							//AFR[1] is the alternate function high reg  				Address offset: 0x24
}GPIO_RegDef_t;


/************************** RCC register definition structure ***********************************************/
typedef struct
{
	__vo uint32_t CR;				//									Address offset: 0x00
	__vo uint32_t PLLCFGR;				//									Address offset: 0x04
	__vo uint32_t CFGR;				//									Address offset: 0x08
	__vo uint32_t CIR;				//									Address offset: 0x0C
	__vo uint32_t AHB1RSTR;				//									Address offset: 0x10
	__vo uint32_t AHB2RSTR;				//									Address offset: 0x14
	uint32_t RESERVED0[2];				//Reserved register							Address offset: 0x18 - 0x1C
	__vo uint32_t APB1RSTR;				//									Address offset: 0x20
	__vo uint32_t APB2RSTR;				//									Address offset: 0x24
	uint32_t RESERVED1[2];				//Reserved register							Address offset: 0x28 - 0x2C
	__vo uint32_t AHB1ENR;				//									Address offset: 0x30
	__vo uint32_t AHB2ENR;				//									Address offset: 0x34
	uint32_t RESERVED2[2];				//Reserved register							Address offset: 0x38 - 0x3C
	__vo uint32_t APB1ENR;				//									Address offset: 0x40
	__vo uint32_t APB2ENR;				//									Address offset: 0x44
	uint32_t RESERVED3[2];				//Reserved register							Address offset: 0x48 - 0x4C
	__vo uint32_t AHB1LPENR;			//									Address offset: 0x50
	__vo uint32_t AHB2LPENR;			//									Address offset: 0x54
	uint32_t RESERVED4[2];				//Reserved register							Address offset: 0x58 - 0x5C
	__vo uint32_t APB1LPENR;			//									Address offset: 0x60
	__vo uint32_t APB2LPENR;			//									Address offset: 0x64
	uint32_t RESERVED5[2];				//Reserved register							Address offset: 0x68 - 0x6C
	__vo uint32_t BDCR;				//									Address offset: 0x70
	__vo uint32_t CSR;				//									Address offset: 0x74
	uint32_t RESERVED6[2];				//Reserved register							Address offset: 0x78 - 0x7C
	__vo uint32_t SSCGR;				//									Address offset: 0x80
	__vo uint32_t PLLI2SCFGR;			//									Address offset: 0x84
	uint32_t RESERVED7;				//Reserved register							Address offset: 0x88
	__vo uint32_t DCKCFGR;				//TIMPRE bit 24 for use, everything else is reserved			Address offset: 0x8C


}RCC_RegDef_t;


//Peripheral definitions
#define DRV_GPIOA						((GPIO_RegDef_t*) DRV_GPIOA_BASEADDR)
#define DRV_GPIOB						((GPIO_RegDef_t*) DRV_GPIOB_BASEADDR)
#define DRV_GPIOC						((GPIO_RegDef_t*) DRV_GPIOC_BASEADDR)
#define DRV_GPIOD						((GPIO_RegDef_t*) DRV_GPIOD_BASEADDR)
#define DRV_GPIOE						((GPIO_RegDef_t*) DRV_GPIOE_BASEADDR)
#define DRV_GPIOH						((GPIO_RegDef_t*) DRV_GPIOH_BASEADDR)

//Defining the Reset and clock control
#define DRV_RCC							((RCC_RegDef_t*) DRV_RCC_BASEADDR)

/***************************** Defining peripheral clock enable macros *********************************/

//Clock enable macros for GPIOx peripherals
#define DRV_GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0));
#define DRV_GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1));
#define DRV_GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2));
#define DRV_GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3));
#define DRV_GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4));
#define DRV_GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7));

//Defining I2Cx clock enable macros
#define DRV_I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21));
#define DRV_I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22));
#define DRV_I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23));

//Defining SPI clock enable macros
#define DRV_SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14));
#define DRV_SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15));
#define DRV_SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12));
#define DRV_SPI4_PCLK_EN()		(RCC->APB2ENR |= (1 << 13));

//Defining USART clock enable macros
#define DRV_USART2_PCLK_EN()		(RCC->APB1ENR |= (1 << 17));
#define DRV_USART1_PCLK_EN()		(RCC->APB2ENR |= (1 << 4));
#define DRV_USART6_PCLK_EN()		(RCC->APB2ENR |= (1 << 5));

//Defining SYSCFG clock enable macros
#define DRV_SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1 << 13));


/***************************** Defining peripheral clock disable macros *********************************/

//Clock disable macros for GPIOx peripherals
#define DRV_GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0));
#define DRV_GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1));
#define DRV_GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2));
#define DRV_GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3));
#define DRV_GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4));
#define DRV_GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7));

//Defining I2Cx clock disable macros
#define DRV_I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21));
#define DRV_I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22));
#define DRV_I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 23));

//Defining SPI clock disable macros
#define DRV_SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14));
#define DRV_SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15));
#define DRV_SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12));
#define DRV_SPI4_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 13));

//Defining USART clock enable macros
#define DRV_USART2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 17));
#define DRV_USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 4));
#define DRV_USART6_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 5));

//Defining SYSCFG clock disable macros
#define DRV_SYSCFG_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 13));



#endif /* INC_STM32F401XX_H_ */
