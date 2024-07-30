/*
 * stm32f407_gpio_driver.h
 *
 *  Created on: Jul 17, 2024
 *      Author: JESTIN
 */

#ifndef INC_STM32F407_GPIO_DRIVER_H_
#define INC_STM32F407_GPIO_DRIVER_H_

#include <stdint.h>
#include "stm32f407_device.h"


//structure defines different configurations of an I/O pin
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_Pin_Config_t;

//structure to handle a GPIO pin
//contains base address to the gpio specific registers and configuration structure
//that is used to initialize the registers
typedef struct
{
	GPIO_RegDef_t* pGPIOx;
	GPIO_Pin_Config_t GPIO_Pin_Config;
}GPIO_Handle_t;

//GPIO pin specific macros

/*
 * Macros to specify GPIO output types
 */

#define GPIO_OUT_PP 0
#define GPIO_OUT_OD 1


/*
 * Macros to specify GPIO speed
 */

#define GPIO_OUT_LOWSPEED 0
#define GPIO_MED_LOWSPEED 1
#define GPIO_OUT_HIGSPEED 2
#define GPIO_OUT_VHIGHSPEED 3

/*
 * gpio pull up and pulldown config
 */

#define GPIO_NO_PU_PD	0
#define GPIO_PU			1
#define GPIO_PD			2


/*
 * GPIO pin numbers
 */


#define PIN0	0
#define PIN1	1
#define PIN2	2
#define PIN3	3
#define PIN4	4
#define PIN5	5
#define PIN6	6
#define PIN7	7
#define PIN8	8
#define PIN9	9
#define PIN10	10
#define PIN11	11
#define PIN12	12
#define PIN13	13
#define PIN14	14
#define PIN15	15

//macros to specify port pin modes
#define GPIO_MODE_IN 0
#define GPIO_MODE_OUT 1
#define GPIO_MODE_ALT_FN 2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FT	4 //interrupt rising edge
#define GPIO_MODE_IT_RT 5
#define GPIO_MODE_IT_RFT 6 // interrupt rising or falling edge trigger

/*
 * ########APIs supported by the driver######
 * function declarations only here definition in the .c file
 */
void GPIO_PeriClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx); // to reset there is  RCC peripheral reset register no need to reset all the registers separately
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value);
void GPIO_ToggleOuputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

/*
 * The following APIs are used to handle the external interrupts at the processor side
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t pinNumber);


#endif /* INC_STM32F407_GPIO_DRIVER_H_ */
