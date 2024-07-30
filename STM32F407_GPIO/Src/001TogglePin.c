/*
 * 001TogglePin.c
 *
 *  Created on: Jul 17, 2024
 *      Author: JESLIN
 */
#include "stm32f407_device.h"
#include "stm32f407_gpio_driver.h"

//toggle delay
void delay()
{
	for (uint32_t i=0;i<500000;i++);
}

/*
 * Toggle the LED connected to PIN12 of port D
 */
int main()
{
	GPIO_Handle_t toggleLED;

	toggleLED.pGPIOx =GPIOD;
	toggleLED.GPIO_Pin_Config.GPIO_PinNumber=PIN12;
	toggleLED.GPIO_Pin_Config.GPIO_PinMode=GPIO_MODE_OUT;
	toggleLED.GPIO_Pin_Config.GPIO_PinOPType=GPIO_OUT_PP;
	toggleLED.GPIO_Pin_Config.GPIO_PinSpeed=GPIO_OUT_HIGSPEED;
	toggleLED.GPIO_Pin_Config.GPIO_PinPuPdControl=GPIO_NO_PU_PD;

	GPIO_PeriClkControl(GPIOD, ENABLE); //enable peripheral clock
	GPIO_Init(&toggleLED);

	while(1)
	{
		GPIO_ToggleOuputPin(GPIOD, PIN12);
		delay();
	}

}
