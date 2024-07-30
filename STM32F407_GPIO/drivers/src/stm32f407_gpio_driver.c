/*
 * stm32f407_gpio_driver.c
 *
 *  Created on: Jul 17, 2024
 *      Author: JESTIN
 */

#include "stm32f407_gpio_driver.h"
//#include "stm32f407_device.h"

/*
 * Function Name:GPIO_PeriClkControl
 * Arguments	:GPIO and Enable/disable
 * Return type	:None
 * Description	:To enable a particular GPIO port
 *
 */


void GPIO_PeriClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
	}
	else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}

	}
}

/*
 * Function Name:GPIO_Init
 * Arguments	:GPIO handle structure (base address of GPIO registers and initializing values)
 * Return type	:None
 * Description	:To initialize GPIO port/pin
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	//GPIO mode configuration
	uint32_t temp=0;
	if((pGPIOHandle->GPIO_Pin_Config.GPIO_PinMode)<=GPIO_MODE_ANALOG)
	{
		temp=pGPIOHandle->GPIO_Pin_Config.GPIO_PinMode <<(2*pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(0X3<<(2*pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |=temp;


	}
	else
	{
		//code to handle the interrupt features

		//1. set the bit corresponding to the pin in the IMR register
		EXTI->EXTI_IMR |= (1<<pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);

		//2.Set the corresponding bits in FTSR/RTSR based on rising edge or falling edge

		if(pGPIOHandle->GPIO_Pin_Config.GPIO_PinMode==GPIO_MODE_IT_FT)
		{
			//configure the EXTI
			EXTI->EXTI_FTSR |= (1<<pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);

			//corresponding bit in the RTSR is cleared to avoid rising edge selection
			EXTI->EXTI_RTSR &= ~(1<<pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);

		}
		else if(pGPIOHandle->GPIO_Pin_Config.GPIO_PinMode==GPIO_MODE_IT_RT)
		{
			//configure the EXTI
			EXTI->EXTI_RTSR |= (1<<pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);

			//corresponding bit in the FTSR is cleared to avoid rising edge selection
			EXTI->EXTI_FTSR &= ~(1<<pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);

		}
		else if(pGPIOHandle->GPIO_Pin_Config.GPIO_PinMode==GPIO_MODE_IT_RFT)
		{
			//configure the EXTI
			EXTI->EXTI_FTSR |= (1<<pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);
			EXTI->EXTI_RTSR |= (1<<pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);

		}

		//3. setup the corresponding SYSCFG register to choose the the port corresponding to the interrupt pin

		//select the corresponding register
		uint8_t temp1 = pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber/4;
		//position in the register
		uint8_t temp2 = pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber%4;

		//enable the SYSCFG peripheral clock
		SYSCFG_PCLK_EN();

		SYSCFG->SYSCFG_EXTICR[temp1] |= (BASE_ADD_TO_CODE(pGPIOHandle->pGPIOx)<<(temp2*4));
	}


	//GPIO speed configuration
	temp=0;
	temp=pGPIOHandle->GPIO_Pin_Config.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3<<(2*pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	// gpio port output type register config
	temp=0;
	temp=pGPIOHandle->GPIO_Pin_Config.GPIO_PinOPType << (pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1<<(pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	//gpio port pull up/pull down reg configuration
	temp=0;
	temp=pGPIOHandle->GPIO_Pin_Config.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x1<<(pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	//gpio alternate function register coniguration
	temp=0;
	if(pGPIOHandle->GPIO_Pin_Config.GPIO_PinMode ==GPIO_MODE_ALT_FN)
	{
		if(pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber <=7)
		{
			temp=pGPIOHandle->GPIO_Pin_Config.GPIO_PinAltFunMode << (4*(pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->AFRL &= ~(0xf << (4*pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->AFRL |= temp;
		}
		else
		{
			temp=pGPIOHandle->GPIO_Pin_Config.GPIO_PinAltFunMode << (4*(pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber % 4));
			pGPIOHandle->pGPIOx->AFRH &= ~(0xf << (4*pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->AFRL |= temp;
		}
	}

}


/*
 * Function Name:GPIO_DeInit
 * Arguments	:GPIO base address
 * Return type	:None
 * Description	:To reset a GPIO port/pin
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_REST();
	}
	else if (pGPIOx ==GPIOB)
	{
		GPIOB_REG_REST();
	}
	else if(pGPIOx ==GPIOC)
	{
		GPIOC_REG_REST();
	}
	else if (pGPIOx ==GPIOD)
	{
		GPIOD_REG_REST();
	}
}

/*
 * Function Name:GPIO_ReadFromInputPin
 * Arguments	:GPIO port and pin number
 * Return type	:value read from the pin
 * Description	:To read from a GPIO pin
 *
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	uint8_t value;
	value = (uint8_t) ((pGPIOx->IDR >>pinNumber)&(0x000000001));
	return value;
}

/*
 * Function Name:GPIO_ReadFromInputPort
 * Arguments	:GPIO port
 * Return type	:value read from the port
 * Description	:To read from a GPIO port
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t) (pGPIOx->IDR);
	return value;
}

/*
 * Function Name:GPIO_WriteToOutputPin
 * Arguments	:GPIO port, pin number, write value
 * Return type	:none
 * Description	:To write to an output pin
 *
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1<<pinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1<<pinNumber);
	}
}

/*
 * Function Name:GPIO_WriteToOutputPort
 * Arguments	:GPIO port, write value
 * Return type	:none
 * Description	:To write to an output port
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

/*
 * Function Name:GPIO_ToggleOuputPin
 * Arguments	:GPIO port, pin nmber
 * Return type	:none
 * Description	:To toggle a pin
 *
 */

void GPIO_ToggleOuputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	pGPIOx->ODR  ^=(1<<pinNumber);
}

/*
 * Function Name:GPIO_IRQInterruptConfig
 * Arguments	:IRQ number, Enable/Disable
 * Return type	:none
 * Description	:To enable the corresponding TRQ by configuring the corresponding NVIC registers
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi)
{
	if(EnorDi ==ENABLE)
	{
		if (IRQNumber <= 31)
		{
			(*NVIC_ISER0) |= (1<<IRQNumber);
		}
		else if (IRQNumber>31 && IRQNumber <64)
		{
			(*NVIC_ISER1) |= (1<<(IRQNumber%32));

		}
		else if (IRQNumber>=64 && IRQNumber <=96)
		{
			(*NVIC_ISER2) |= (IRQNumber%64);

		}
		else if (IRQNumber>=96 && IRQNumber <128)
		{
			(*NVIC_ISER3) |= (1<<(IRQNumber%96));

		}

	}
	else
	{
		if (IRQNumber <= 31)
		{
			(*NVIC_ICER0) |= (1<<IRQNumber);
		}
		else if (IRQNumber>31 && IRQNumber <64)
		{
			(*NVIC_ICER1) |= (1<<(IRQNumber%32));

		}
		else if (IRQNumber>=64 && IRQNumber <=96)
		{
			(*NVIC_ICER2) |= (IRQNumber%64);

		}
		else if (IRQNumber>=96 && IRQNumber <128)
		{
			(*NVIC_ICER3) |= (1<<(IRQNumber%96));

		}
	}

}

	/*
	 * Function Name:GPIO_IRQPriorityConfig
	 * Arguments	:IRQ number, priority(0 to 15)
	 * Return type	:none
	 * Description	:To set a priority for the external interrupt line
	 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//there are 60 priority set registers in the NVIC to handle 240 interrupts
	//Each register handles 4 IRQ and 8 bits assigned to each

	//calculation of priority register corresponding to the IRq
	uint8_t register_pos = IRQNumber/4;

	//each register is of 4 bytes so the corresponding memory offset is temp(since the base address is of uint32,each increment actually adds 4 to the base address)
	//eventhough there are 8 bits assigned for each IRQ, in the given processor
	//only the higher four(PRIORITY_BITS_ASSIGNED) bits are valid(priority form 0 to 15)
	//so the location within the register where the priority value is to be written is

	uint8_t bit_shift = ((IRQNumber%4)*8)+(8-PRIORITY_BITS_ASSIGNED);

	*(NVIC_IPR+(register_pos)) |= (IRQPriority<bit_shift);
}

/*
 * Function Name:GPIO_IRQHandling
 * Arguments	:pin number
 * Return type	:none
 * Description	:To clear the pending register after handling the interrupt
 */

void GPIO_IRQHandling(uint8_t pinNumber)
{
	if((EXTI->EXTI_PR & (1<<pinNumber)))
	{
		EXTI->EXTI_PR |=(1<<pinNumber);
	}
}

