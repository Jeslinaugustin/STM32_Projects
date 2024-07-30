/*
 * 004LEDPattern.c
 *
 *  Created on: Jul 19, 2024
 *      Author: JESLIN
 */

#include "stm32f407_device.h"
#include "stm32f407_gpio_driver.h"

__vo uint8_t mode =1;

//toggle delay
void delay()
{
	for (uint32_t i=0;i<500000;i++);
}

//delay to eliminate button debouncing
void delay_debounce()
{
	for (uint32_t i=0;i<50000;i++);
}

/*
 * LED Pattern 1
 */


void mode1Function(GPIO_Handle_t LED1,GPIO_Handle_t LED2,GPIO_Handle_t LED3,GPIO_Handle_t LED4,GPIO_Handle_t LED5)
{
	GPIO_WriteToOutputPin(LED1.pGPIOx,LED1.GPIO_Pin_Config.GPIO_PinNumber,  SET);
	GPIO_WriteToOutputPin(LED2.pGPIOx, LED2.GPIO_Pin_Config.GPIO_PinNumber,  SET);
	GPIO_WriteToOutputPin(LED3.pGPIOx, LED3.GPIO_Pin_Config.GPIO_PinNumber, SET);
	GPIO_WriteToOutputPin(LED4.pGPIOx, LED4.GPIO_Pin_Config.GPIO_PinNumber,SET);
	GPIO_WriteToOutputPin(LED5.pGPIOx,LED5.GPIO_Pin_Config.GPIO_PinNumber,  SET);

	delay();

	GPIO_WriteToOutputPin(LED1.pGPIOx, LED1.GPIO_Pin_Config.GPIO_PinNumber,  RESET);
	GPIO_WriteToOutputPin(LED2.pGPIOx, LED2.GPIO_Pin_Config.GPIO_PinNumber,  RESET);
	GPIO_WriteToOutputPin(LED3.pGPIOx, LED3.GPIO_Pin_Config.GPIO_PinNumber, RESET);
	GPIO_WriteToOutputPin(LED4.pGPIOx, LED4.GPIO_Pin_Config.GPIO_PinNumber,RESET);
	GPIO_WriteToOutputPin(LED5.pGPIOx,LED5.GPIO_Pin_Config.GPIO_PinNumber,  RESET);

	delay();

}

/*
 * LED Pattern 1
 */

void mode2Function(GPIO_Handle_t LED1,GPIO_Handle_t LED2,GPIO_Handle_t LED3,GPIO_Handle_t LED4,GPIO_Handle_t LED5)
{
	GPIO_WriteToOutputPin(LED1.pGPIOx,LED1.GPIO_Pin_Config.GPIO_PinNumber,  SET);
	GPIO_WriteToOutputPin(LED2.pGPIOx, LED2.GPIO_Pin_Config.GPIO_PinNumber,  RESET);
	GPIO_WriteToOutputPin(LED3.pGPIOx, LED3.GPIO_Pin_Config.GPIO_PinNumber, SET);
	GPIO_WriteToOutputPin(LED4.pGPIOx, LED4.GPIO_Pin_Config.GPIO_PinNumber,RESET);
	GPIO_WriteToOutputPin(LED5.pGPIOx,LED5.GPIO_Pin_Config.GPIO_PinNumber,  SET);

	delay();

	GPIO_WriteToOutputPin(LED1.pGPIOx,LED1.GPIO_Pin_Config.GPIO_PinNumber,  RESET);
	GPIO_WriteToOutputPin(LED2.pGPIOx, LED2.GPIO_Pin_Config.GPIO_PinNumber,  SET);
	GPIO_WriteToOutputPin(LED3.pGPIOx, LED3.GPIO_Pin_Config.GPIO_PinNumber, RESET);
	GPIO_WriteToOutputPin(LED4.pGPIOx, LED4.GPIO_Pin_Config.GPIO_PinNumber,SET);
	GPIO_WriteToOutputPin(LED5.pGPIOx,LED5.GPIO_Pin_Config.GPIO_PinNumber,  RESET);

	delay();

}

/*
 * Toggle the LED connected to PIN12 of port D
 */
int main()
{
	GPIO_Handle_t LED1,LED2,LED3,LED4,LED5, inputBtn;

	LED1.pGPIOx =GPIOD;
	LED1.GPIO_Pin_Config.GPIO_PinNumber=PIN11;
	LED1.GPIO_Pin_Config.GPIO_PinMode=GPIO_MODE_OUT;
	LED1.GPIO_Pin_Config.GPIO_PinOPType=GPIO_OUT_PP;
	LED1.GPIO_Pin_Config.GPIO_PinSpeed=GPIO_OUT_HIGSPEED;
	LED1.GPIO_Pin_Config.GPIO_PinPuPdControl=GPIO_NO_PU_PD;

	LED2.pGPIOx =GPIOD;
	LED2.GPIO_Pin_Config.GPIO_PinNumber=PIN12;
	LED2.GPIO_Pin_Config.GPIO_PinMode=GPIO_MODE_OUT;
	LED2.GPIO_Pin_Config.GPIO_PinOPType=GPIO_OUT_PP;
	LED2.GPIO_Pin_Config.GPIO_PinSpeed=GPIO_OUT_HIGSPEED;
	LED2.GPIO_Pin_Config.GPIO_PinPuPdControl=GPIO_NO_PU_PD;

	LED3.pGPIOx =GPIOD;
	LED3.GPIO_Pin_Config.GPIO_PinNumber=PIN13;
	LED3.GPIO_Pin_Config.GPIO_PinMode=GPIO_MODE_OUT;
	LED3.GPIO_Pin_Config.GPIO_PinOPType=GPIO_OUT_PP;
	LED3.GPIO_Pin_Config.GPIO_PinSpeed=GPIO_OUT_HIGSPEED;
	LED3.GPIO_Pin_Config.GPIO_PinPuPdControl=GPIO_NO_PU_PD;

	LED4.pGPIOx =GPIOD;
	LED4.GPIO_Pin_Config.GPIO_PinNumber=PIN14;
	LED4.GPIO_Pin_Config.GPIO_PinMode=GPIO_MODE_OUT;
	LED4.GPIO_Pin_Config.GPIO_PinOPType=GPIO_OUT_PP;
	LED4.GPIO_Pin_Config.GPIO_PinSpeed=GPIO_OUT_HIGSPEED;
	LED4.GPIO_Pin_Config.GPIO_PinPuPdControl=GPIO_NO_PU_PD;

	LED5.pGPIOx =GPIOD;
	LED5.GPIO_Pin_Config.GPIO_PinNumber=PIN15;
	LED5.GPIO_Pin_Config.GPIO_PinMode=GPIO_MODE_OUT;
	LED5.GPIO_Pin_Config.GPIO_PinOPType=GPIO_OUT_PP;
	LED5.GPIO_Pin_Config.GPIO_PinSpeed=GPIO_OUT_HIGSPEED;
	LED5.GPIO_Pin_Config.GPIO_PinPuPdControl=GPIO_NO_PU_PD;

	inputBtn.pGPIOx =GPIOA;
	inputBtn.GPIO_Pin_Config.GPIO_PinNumber = PIN8;
	inputBtn.GPIO_Pin_Config.GPIO_PinMode=GPIO_MODE_IT_FT;
	inputBtn.GPIO_Pin_Config.GPIO_PinOPType=GPIO_OUT_PP;
	inputBtn.GPIO_Pin_Config.GPIO_PinPuPdControl=GPIO_PU;
	inputBtn.GPIO_Pin_Config.GPIO_PinSpeed=GPIO_OUT_HIGSPEED;

	GPIO_PeriClkControl(GPIOD, ENABLE); //enable peripheral clock
	GPIO_Init(&LED1);
	GPIO_Init(&LED2);
	GPIO_Init(&LED3);
	GPIO_Init(&LED4);
	GPIO_Init(&LED5);

	GPIO_PeriClkControl(GPIOA, ENABLE);
	GPIO_Init(&inputBtn);

	//interrupt configuration
	//setting priority is optional if there is only one interrupt

	GPIO_IRQInterruptConfig(IRQ_EXTIEXTI9_5, ENABLE); // pin number 8
	GPIO_IRQPriorityConfig(IRQ_EXTIEXTI9_5, 15);


	while(1)
	{
		if(mode==1)
		{
			//mode1Function(*LED1,*LED2,*LED3,*LED4,*LED5);
			mode1Function(LED1,LED2,LED3,LED4,LED5);
		}
		else
		{
			//mode2Function(*LED1,*LED2,*LED3,*LED4,*LED5);
			mode2Function(LED1,LED2,LED3,LED4,LED5);
		}
	}
	return 0;

}

void EXTI9_5_IRQHandler()
{
	GPIO_IRQHandling(PIN8);
	delay_debounce();

	if(mode ==1)
	{
		mode =2;
	}
	else
	{
		mode=1;
	}
}

