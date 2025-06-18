/*
 * stm32f401xx_gpio_driver.c
 *
 *  Created on: Jun 18, 2025
 *      Author: Nikola Sokolović
 */

#include "stm32f401xx_gpio_driver.h"


/*************************************************************
 * @Function:			GPIO_PeriClockControl
 *
 * @Description:		This function enables or disables the clock for the GPIO port
 *
 * @Parameter[in]		Address to the GPIO peripheral
 * @Parameter[in]		ENABLE or DISABLE macros
 * @Parameter[in]
 *
 * @Return:				None
 *
 * @Note:				None
 *
 */
//Peripheral clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pGPIOx == DRV_GPIOA)
		{
			DRV_GPIOA_PCLK_EN();
		}
		else if (pGPIOx == DRV_GPIOB)
		{
			DRV_GPIOB_PCLK_EN();
		}
		else if (pGPIOx == DRV_GPIOC)
		{
			DRV_GPIOC_PCLK_EN();
		}
		else if (pGPIOx == DRV_GPIOD)
		{
			DRV_GPIOD_PCLK_EN();
		}
		else if (pGPIOx == DRV_GPIOE)
		{
			DRV_GPIOE_PCLK_EN();
		}
		else
		{
			DRV_GPIOH_PCLK_EN();
		}
	}
	else{
		if (pGPIOx == DRV_GPIOA)
		{
			DRV_GPIOA_PCLK_DI();
		}
		else if (pGPIOx == DRV_GPIOB)
		{
			DRV_GPIOB_PCLK_DI();
		}
		else if (pGPIOx == DRV_GPIOC)
		{
			DRV_GPIOC_PCLK_DI();
		}
		else if (pGPIOx == DRV_GPIOD)
		{
			DRV_GPIOD_PCLK_DI();
		}
		else if (pGPIOx == DRV_GPIOE)
		{
			DRV_GPIOE_PCLK_DI();
		}
		else
		{
			DRV_GPIOH_PCLK_DI();
		}
	}

	return;
}


//Init and De-Init of GPIO
/*************************************************************
 * @Function:			GPIO_Init
 *
 * @Description:		This function initializes a GPIO pin
 *
 * @Parameter[in]		Address to the GPIO peripheral
 * @Parameter[in]
 * @Parameter[in]
 *
 * @Return:				None
 *
 * @Note:				None
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{

	uint32_t temp = 0;

	//configuring the mode of GPIO
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp;	//setting
	}
	else
	{
		//interrupt mode
	}

	//configuring the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed  << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	//configure pull up/down settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl  << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	//configuring the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType  <<  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	//configuring the alternate functionality mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure the alt function reg
		uint8_t temp1, temp2; //we use temp1 and temp2 to identify the location of the register we need

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8; //if 1 it is the upper 8, if 0 the it is in the lower 8
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;	//
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}


	return;
}

/*************************************************************
 * @Function:			GPIO_DeInit
 *
 * @Description:		This function initializes a GPIO pin
 *
 * @Parameter[in]		Address to the GPIO port
 * @Parameter[in]
 * @Parameter[in]
 *
 * @Return:				None
 *
 * @Note:				None
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == DRV_GPIOA)
	{
		DRV_GPIOA_REG_RST();
	}
	else if (pGPIOx == DRV_GPIOB)
	{
		DRV_GPIOB_REG_RST();
	}
	else if (pGPIOx == DRV_GPIOC)
	{
		DRV_GPIOC_REG_RST();
	}
	else if (pGPIOx == DRV_GPIOD)
	{
		DRV_GPIOD_REG_RST();
	}
	else if (pGPIOx == DRV_GPIOE)
	{
		DRV_GPIOE_REG_RST();
	}
	else
	{
		DRV_GPIOH_REG_RST();
	}
}

//Data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

//IRQ configuration and ISR handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);
