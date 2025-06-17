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
#define DRV_SRAM 							DRV_SRAM1_BASEADDR	//there is only one SRAM on this controller


//Defining AHBx and APBx bus peripherals
#define DRV_PERIPH_BASEADDR					0x40000000U		//defining the base address of peripherals
#define DRV_APB1PERIPH_BASEADDR				DRV_PERIPH_BASEADDR	//APB1 is first
#define DRV_APB2PERIPH_BASEADDR				0x40010000U		//defining the base address for APB2
#define DRV_AHB1PERIPH_BASEADDR				0x40020000U		//defining the base address for AHB1
#define DRV_AHB2PERIPH_BASEADDR				0x50000000U		//defining the base address for AHB2


//Defining base addresses of peripherals connected to AHB1
//We define the addresses as the AHB1 base address + the offset for each GPIO
#define DRV_GPIOA_BASEADDR					(DRV_AHB1PERIPH_BASEADDR + 0x0000)
#define DRV_GPIOB_BASEADDR					(DRV_AHB1PERIPH_BASEADDR + 0x0400)
#define DRV_GPIOC_BASEADDR					(DRV_AHB1PERIPH_BASEADDR + 0x0800)
#define DRV_GPIOD_BASEADDR					(DRV_AHB1PERIPH_BASEADDR + 0x0C00)
#define DRV_GPIOE_BASEADDR					(DRV_AHB1PERIPH_BASEADDR + 0x1000)
#define DRV_GPIOH_BASEADDR					(DRV_AHB1PERIPH_BASEADDR + 0x1C00)


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
	__vo uint32_t MODER;			//Defining the port mode register							Address offset: 0x00
	__vo uint32_t OTYPER;			//Defining the output type of I/O port						Address offset: 0x04
	__vo uint32_t OSPEEDR;			//Defining the I/O output speed								Address offset: 0x08
	__vo uint32_t PUPDR;			//Defining the I/O as pull up/down 							Address offset: 0x0C
	__vo uint32_t IDR;				//These registers can only be read							Address offset: 0x10
	__vo uint32_t ODR;				//to be added												Address offset: 0x14
	__vo uint32_t BSRR;				//to be added												Address offset: 0x18
	__vo uint32_t LCKR;				//Defining the port configuration locked reg				Address offset: 0x1C
	__vo uint32_t AFR[2];			//AFR[0] is the alternate function low reg, 				Address offset: 0x20
									//AFR[1] is the alternate function low reg  				Address offset: 0x24
}GPIO_RegDef_t;




#endif /* INC_STM32F401XX_H_ */
