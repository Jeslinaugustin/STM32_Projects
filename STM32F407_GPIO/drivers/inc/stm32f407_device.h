/*
 * stm32f407_device.h
 *
 *  Created on: Jul 17, 2024
 *      Author: JESTIN
 */

#ifndef INC_STM32F407_DEVICE_H_
#define INC_STM32F407_DEVICE_H_

#include <stdint.h>

#define __vo volatile

/*
 * Processor specific details
 * Included the details to deal with interrupts
 * In the given microcontroller a few interrupts are used than in the processor
 * Therefore the addresses of the registers required in the application is only added
 * If want add the remaining address
 */

#define NVIC_ISER0	((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1	((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2	((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3	((__vo uint32_t*)0xE000E10C)

#define NVIC_ICER0	((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1	((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2	((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3	((__vo uint32_t*)0xE000E18C)

#define NVIC_IPR	((__vo uint32_t*)0xE000E400)

/*
 * Controller Specific details
 */
#define FLASH_BASE_ADDRESS (0x08000000U)
#define SRAM_BASE_ADDRESS	(0x20000000U)	//SRAM1 is commonly called as SRAM
#define ROM_BASE_ADDRESS	(0x1FFF0000U)	//system memory base address

//Bus peripheral base address

#define PERIPH_BASE_ADDRESS			(0x40000000U)
#define APB1_BASE_ADDRESS			PERIPH_BASE
#define APB2_BASE_ADDRESS			(0x40010000U)
#define AHB1_BASE_ADDRESS			(0x40020000U)
#define AHB2_BASE_ADDRESS			(0x50000000U)

//base addresses of peripherals hanging on AHB1 bus

#define GPIOA_BASE_ADDRESS		(AHB1_BASE_ADDRESS+0x0000)
#define GPIOB_BASE_ADDRESS		(AHB1_BASE_ADDRESS+0x0400)
#define GPIOC_BASE_ADDRESS		(AHB1_BASE_ADDRESS+0x0800)
#define GPIOD_BASE_ADDRESS		(AHB1_BASE_ADDRESS+0x0C00)
#define GPIOE_BASE_ADDRESS		(AHB1_BASE_ADDRESS+0x1000)
#define GPIOF_BASE_ADDRESS		(AHB1_BASE_ADDRESS+0x1400)
#define GPIOG_BASE_ADDRESS		(AHB1_BASE_ADDRESS+0x1800)
#define GPIOH_BASE_ADDRESS		(AHB1_BASE_ADDRESS+0x1C00)
#define GPIOI_BASE_ADDRESS		(AHB1_BASE_ADDRESS+0x2000)
#define GPIOJ_BASE_ADDRESS		(AHB1_BASE_ADDRESS+0x2400)
#define GPIOK_BASE_ADDRESS		(AHB1_BASE_ADDRESS+0x2800)

#define RCC_BASE_ADDRESS		(AHB1_BASE_ADDRESS+0x3800)

//base addresses of peripherals hanging on APB2
//only the peripherals that are used in the application are included

#define SYSCFG_BASE_ADDRESS	(APB2_BASE_ADDRESS+0x3800)
#define EXTI_BASE_ADDRESS	(APB2_BASE_ADDRESS+0x3C00)

//GPIO register definition structure
typedef struct
{
	__vo uint32_t  MODER;
	__vo uint32_t  OTYPER;
	__vo uint32_t  OSPEEDR;
	__vo uint32_t  PUPDR;
	__vo uint32_t  IDR ;
	__vo uint32_t  ODR;
	__vo uint32_t  BSRR;
	__vo uint32_t  LCKR ;
	__vo uint32_t  AFRL ;
	__vo uint32_t  AFRH ;
}GPIO_RegDef_t;

//RCC registers definition structure
typedef struct
{
	__vo uint32_t RCC_CR;
	__vo uint32_t RCC_PLLCFGR;
	__vo uint32_t RCC_CFGR;
	__vo uint32_t RCC_CIR;
	__vo uint32_t RCC_AHB1RSTR;
	__vo uint32_t RCC_AHB2RSTR;
	__vo uint32_t RCC_AHB3RSTR;
	uint32_t Reserved1;
	__vo uint32_t RCC_APB1RSTR;
	__vo uint32_t RCC_APB2RSTR;
	uint32_t Reserved2;
	uint32_t Reserved3;
	__vo uint32_t RCC_AHB1ENR;
	__vo uint32_t RCC_AHB2ENR;
	__vo uint32_t RCC_AHB3ENR;
	__vo uint32_t RCC_APB1ENR;
	__vo uint32_t RCC_APB2ENR;
	uint32_t Reserved4;
	uint32_t Reserved5;
	__vo uint32_t RCC_AHB1LPENR;
	__vo uint32_t RCC_AHB2LPENR;
	__vo uint32_t RCC_AHB3LPENR;
	uint32_t Reserved6;
	__vo uint32_t RCC_APB1LPENR;
	__vo uint32_t RCC_APB2LPENR;
	uint32_t Reserved7;
	uint32_t Reserved8;
	__vo uint32_t RCC_BDCR;
	__vo uint32_t RCC_CSR;
	uint32_t reserved9;
	uint32_t reserved10;
	__vo uint32_t RCC_SSCGR;
	__vo uint32_t RCC_PLLI2SCFGR;
}RCC_RegDef_t;

/*
 * Type defintion structure for EXTI registers
 * To set up the gpio pin to accept the interrupt,to select rising edge/falling edge/both
 */

typedef struct
{
	__vo uint32_t  EXTI_IMR;
	__vo uint32_t  EXTI_EMR;
	__vo uint32_t  EXTI_RTSR;
	__vo uint32_t  EXTI_FTSR;
	__vo uint32_t  EXTI_SWIER ;
	__vo uint32_t  EXTI_PR;
}EXTI_RegDef_t;

typedef struct
{
	__vo uint32_t  SYSCFG_MEMRMP;
	__vo uint32_t  SYSCFG_PMC;
	__vo uint32_t  SYSCFG_EXTICR[4];
	__vo uint32_t reserved[2];
	__vo uint32_t  SYSCFG_CMPCR;

}SYSCFG_RegDef_t;

//peripheral definitions
#define GPIOA  ((GPIO_RegDef_t*)GPIOA_BASE_ADDRESS)
#define GPIOB  ((GPIO_RegDef_t*)GPIOB_BASE_ADDRESS)
#define GPIOC  ((GPIO_RegDef_t*)GPIOC_BASE_ADDRESS)
#define GPIOD  ((GPIO_RegDef_t*)GPIOD_BASE_ADDRESS)
#define GPIOE  ((GPIO_RegDef_t*)GPIOE_BASE_ADDRESS)
#define GPIOF  ((GPIO_RegDef_t*)GPIOF_BASE_ADDRESS)
#define GPIOG  ((GPIO_RegDef_t*)GPIOG_BASE_ADDRESS)
#define GPIOH  ((GPIO_RegDef_t*)GPIOH_BASE_ADDRESS)
#define GPIOI  ((GPIO_RegDef_t*)GPIOI_BASE_ADDRESS)

#define RCC  ((RCC_RegDef_t*)RCC_BASE_ADDRESS)

#define SYSCFG ((SYSCFG_RegDef_t*)SYSCFG_BASE_ADDRESS)
#define EXTI ((EXTI_RegDef_t*)EXTI_BASE_ADDRESS)

//define peripheral clock enable macros

#define GPIOA_PCLK_EN()	(RCC->RCC_AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()	(RCC->RCC_AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()	(RCC->RCC_AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()	(RCC->RCC_AHB1ENR |= (1<<3))
//define peripheral clock disable macros

//SYSCFG peripheral clock enable macros
#define SYSCFG_PCLK_EN()(RCC->RCC_APB2ENR |=(1<<14))

#define GPIOA_PCLK_DI()	(RCC->RCC_AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()	(RCC->RCC_AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()	(RCC->RCC_AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()	(RCC->RCC_AHB1ENR &= ~(1<<3))

//macros to rest the GPIOx peripheral registers
#define GPIOA_REG_REST() do{RCC->RCC_AHB1RSTR |= (1<<0); RCC->RCC_AHB1RSTR &= ~(1<<0);}while(0)
#define GPIOB_REG_REST() do{RCC->RCC_AHB1RSTR |= (1<<1); RCC->RCC_AHB1RSTR &= ~(1<<1);}while(0)
#define GPIOC_REG_REST() do{RCC->RCC_AHB1RSTR |= (1<<2); RCC->RCC_AHB1RSTR &= ~(1<<2);}while(0)
#define GPIOD_REG_REST() do{RCC->RCC_AHB1RSTR |= (1<<3); RCC->RCC_AHB1RSTR &= ~(1<<3);}while(0)


//some useful macros
#define ENABLE	1
#define DISABLE	0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET

/*
 * macro specify the number of priority bits assigned
 * controller specific
 */
#define PRIORITY_BITS_ASSIGNED 4

/*
 * IRQ numbers for different Interrupts(controller specific)
 */

#define IRQ_EXTI0	6
#define IRQ_EXTI1	7
#define IRQ_EXTI2	8
#define IRQ_EXTI3	9
#define IRQ_EXTI4	10
#define IRQ_EXTIEXTI9_5	23
#define IRQ_EXTIEXTI15_10	40

/*
 * Macro function to choose the port corresponding to an EXTI interrupt line
 * used to set the SYSCFG register
 */
#define BASE_ADD_TO_CODE(x)	( (x==GPIOA)?0:\
							(x==GPIOB)?1:\
							(x==GPIOC)?2:\
							(x==GPIOD)?3:\
							(x==GPIOE)?4:\
							(x==GPIOF)?5:\
							(x==GPIOG)?6:\
							(x==GPIOH)?7:\
							(x==GPIOI)?8:0)
#endif /* INC_STM32F407_DEVICE_H_ */
