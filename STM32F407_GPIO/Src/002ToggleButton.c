/*
 * 002ToggleButton.c
 *
 *  Created on: Jul 17, 2024
 *      Author: JESLIN
 *
 *      On board LED Toggles on button press
 */

#include "stm32f407_device.h"
#include "stm32f407_gpio_driver.h"

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
 * Toggle the LED connected to PIN12 of port D
 */
int main()
{
	GPIO_Handle_t toggleLED, inputBtn;

	toggleLED.pGPIOx =GPIOD;
	toggleLED.GPIO_Pin_Config.GPIO_PinNumber=PIN12;
	toggleLED.GPIO_Pin_Config.GPIO_PinMode=GPIO_MODE_OUT;
	toggleLED.GPIO_Pin_Config.GPIO_PinOPType=GPIO_OUT_PP;
	toggleLED.GPIO_Pin_Config.GPIO_PinSpeed=GPIO_OUT_HIGSPEED;
	toggleLED.GPIO_Pin_Config.GPIO_PinPuPdControl=GPIO_NO_PU_PD;

	inputBtn.pGPIOx =GPIOA;
	inputBtn.GPIO_Pin_Config.GPIO_PinNumber = PIN8;
	inputBtn.GPIO_Pin_Config.GPIO_PinMode=GPIO_MODE_IN;
	inputBtn.GPIO_Pin_Config.GPIO_PinOPType=GPIO_OUT_PP;
	inputBtn.GPIO_Pin_Config.GPIO_PinPuPdControl=GPIO_PU;
	inputBtn.GPIO_Pin_Config.GPIO_PinSpeed=GPIO_OUT_HIGSPEED;

	GPIO_PeriClkControl(GPIOD, ENABLE); //enable peripheral clock
	GPIO_Init(&toggleLED);

	GPIO_PeriClkControl(GPIOA, ENABLE);
	GPIO_Init(&inputBtn);


	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA, PIN8)==0)
		{
			delay_debounce();
			GPIO_ToggleOuputPin(GPIOD, PIN12);
			delay();
		}

	}

}
