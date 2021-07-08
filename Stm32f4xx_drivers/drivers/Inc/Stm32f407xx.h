/*
 * Stm32f407xx.h
 *
 *  Created on: Jul 5, 2021
 *      Author: enesayanoglu
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_
#define __vo volatile
#include <stdint.h>

/*
 * ARM cortex Mx Processor NVIC ISERx register Adrresses
 */
#define NVIC_ISER0		((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1		((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2		((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3		((__vo uint32_t*)0xE000E10C)
#define NVIC_ISER4		((__vo uint32_t*)0xE000E110)
#define NVIC_ISER5		((__vo uint32_t*)0xE000E114)
#define NVIC_ISER6		((__vo uint32_t*)0xE000E118)
#define NVIC_ISER7		((__vo uint32_t*)0xE000E11C)
/*
 * ARM cortex Mx Processor NVIC ICERx register Adrresses
 */
#define NVIC_ICER0     	((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1     	((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2     	((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3     	((__vo uint32_t*)0xE000E18C)
#define NVIC_ICER4     	((__vo uint32_t*)0xE000E190)
#define NVIC_ICER5     	((__vo uint32_t*)0xE000E194)
#define NVIC_ICER6     	((__vo uint32_t*)0xE000E198)
#define NVIC_ICER7     	((__vo uint32_t*)0xE000E19C)

#define NVIC_PR_BASE_ADDR ((__vo uint32_t*)0xE000E400)
/*
#define NVIC_IPR0		((__vo uint32_t*)0xE000E400)
#define NVIC_IPR1		((__vo uint32_t*)0xE000E404)
#define NVIC_IPR2		((__vo uint32_t*)0xE000E408)
#define NVIC_IPR3		((__vo uint32_t*)0xE000E40C)
#define NVIC_IPR4		((__vo uint32_t*)0xE000E410)
#define NVIC_IPR5		((__vo uint32_t*)0xE000E414)
#define NVIC_IPR6		((__vo uint32_t*)0xE000E418)
#define NVIC_IPR7		((__vo uint32_t*)0xE000E41C)
#define NVIC_IPR8		((__vo uint32_t*)0xE000E420)
#define NVIC_IPR9		((__vo uint32_t*)0xE000E424)
#define NVIC_IPR10		((__vo uint32_t*)0xE000E428)
#define NVIC_IPR11		((__vo uint32_t*)0xE000E42C)
#define NVIC_IPR12		((__vo uint32_t*)0xE000E430)
#define NVIC_IPR13		((__vo uint32_t*)0xE000E424)
#define NVIC_IPR14		((__vo uint32_t*)0xE000E438)
#define NVIC_IPR15		((__vo uint32_t*)0xE000E43C)
#define NVIC_IPR16		((__vo uint32_t*)0xE000E440)
#define NVIC_IPR17		((__vo uint32_t*)0xE000E444)
#define NVIC_IPR18		((__vo uint32_t*)0xE000E448)
#define NVIC_IPR19		((__vo uint32_t*)0xE000E44C)
#define NVIC_IPR20		((__vo uint32_t*)0xE000E450)
#define NVIC_IPR21		((__vo uint32_t*)0xE000E454)
#define NVIC_IPR22		((__vo uint32_t*)0xE000E458)
#define NVIC_IPR23		((__vo uint32_t*)0xE000E45C)
*/
#define NO_PR_BITS_IMPLEMENTED 4




/*
 * base addresses of Flash and SRAM memories
 */

#define FLASH_BA SEADDR						0x08000000U
#define SRAM1_BASEADDR						0x20000000U
#define SRAM2_BASEADDR						0x2001C000U
#define ROM_BASEADDR						0x1FFF0000U
#define SRAM 								SRAM1_BASEADDR


/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR 						0x40000000U
#define APB1PERIPH_BASEADDR						PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR						0x40010000U
#define AHB1PERIPH_BASEADDR						0x40020000U
#define AHB2PERIPH_BASEADDR						0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR                     (AHB1PERIPH_BASEADDR + 0x3800)
#define FIR_BASEADDR					 (AHB1PERIPH_BASEADDR + 0x3C00) //Flash interface register
#define BKPSRAM_BASEADDR				 (AHB1PERIPH_BASEADDR + 0x4000)
#define DMA1_BASEADDR                    (AHB1PERIPH_BASEADDR + 0x6000)
#define DMA2_BASEADDR                    (AHB1PERIPH_BASEADDR + 0x6400)
#define ETH_BASEADDR                     (AHB1PERIPH_BASEADDR + 0x8000) //ETHERNET MAC base address
#define DMA2D_BASEADDR                   (AHB1PERIPH_BASEADDR + 0xB000)
#define USBOTGHS_BASEADDR                (AHB1PERIPH_BASEADDR + 0x20000)


/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * TODO : Complete for all other peripherals
 */
#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR						(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR						(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR						(APB1PERIPH_BASEADDR + 0x5000)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * TODO : Complete for all other peripherals
 */
#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR        				(APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400)

typedef struct{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFRL[2];

}GPIO_RegDef_t;
typedef struct{
	__vo uint32_t IMR;		//Interrupt mask register
	__vo uint32_t EMR;		//Event mask register
	__vo uint32_t RTSR;		//Rising trigger selection register
	__vo uint32_t FTSR;		//Falling trigger selection register
	__vo uint32_t SWIER;	//Software interrupt event register
	__vo uint32_t PR;		//Pending register

}EXTI_RegDef_t;
typedef struct{
	__vo uint32_t MEMRMP;   //SYSCFG memory remap register
	__vo uint32_t PMC;   //SYSCFG peripheral mode configuration register
	 uint32_t EXTICR[3];   //SYSCFG external interrupt configuration register 1,2,3,4
	 __vo uint32_t Reserved1[2];
	 __vo uint32_t CMPCR;   //Compensation cell control register
}SYSCFG_RegDef_t;

typedef struct{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	 uint32_t Reserved1;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	 uint32_t Reserved2;
	 uint32_t Reserved3;
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	 uint32_t Reserved4;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	 uint32_t Reserved5;
	 uint32_t Reserved6;
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	 uint32_t Reserved7;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	 uint32_t Reserved8;
	 uint32_t Reserved9;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	 uint32_t Reserved10;
	 uint32_t Reserved11;
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
}RCC_RegDef_t;
//Peripheral base addresses type casted to xxx_RegDef_t
#define GPIOA  			(( GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB  			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF  			((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG  			((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH  			((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI  			((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC 			((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI			((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG			((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

//Clock enable for GPIOx

#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))

#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 8))

#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1<<23))

#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))

#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 5))
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))

#define GPIOA_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 0));	(RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 1));	(RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 2));	(RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 3));	(RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 4));	(RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 5));	(RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 6));	(RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 7));	(RCC->AHB1RSTR &= ~(1 << 7));}while(0)
#define GPIOI_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 8));	(RCC->AHB1RSTR &= ~(1 << 8));}while(0)

#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOF)?5:\
								        (x == GPIOG)?6:\
								        (x == GPIOH)?7: \
								        (x == GPIOI)?8:0)
/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 */

#define IRQ_NO_WWDG  				0
#define IRQ_NO_PVD					1
#define IRQ_NO_TAMP_STAMP			2
#define IRQ_NO_RTC_WKUP				3
#define IRQ_NO_FLASH				4
#define IRQ_NO_RCC					5
#define IRQ_NO_EXTI0				6
#define IRQ_NO_EXTI1				7
#define IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3				9
#define IRQ_NO_EXTI4				10
#define IRQ_NO_DMA1_Stream0			11
#define IRQ_NO_DMA1_Stream1			12
#define IRQ_NO_DMA1_Stream2			13
#define IRQ_NO_DMA1_Stream3			14
#define IRQ_NO_DMA1_Stream4			15
#define IRQ_NO_DMA1_Stream5			16
#define IRQ_NO_DMA1_Stream6			17
#define IRQ_NO_ADC					18
#define IRQ_NO_CAN1_TX				19
#define IRQ_NO_CAN1_RX0				20
#define IRQ_NO_CAN1_RX1				21
#define IRQ_NO_CAN1_SCE				22
#define IRQ_NO_EXTI9_5				23
#define IRQ_NO_TIM1_BRK_TIM9		24
#define IRQ_NO_TIM1_UP_TIM10		25
#define IRQ_NO_TIM1_TRG_COM_TIM11	26
#define IRQ_NO_TIM1_CC				27
#define IRQ_NO_TIM2					28
#define IRQ_NO_TIM3					29
#define IRQ_NO_TIM4					30
#define IRQ_NO_I2C1_EV				31
#define IRQ_NO_I2C1_ER				32
#define IRQ_NO_I2C2_EV				33
#define IRQ_NO_I2C2_ER				34
#define IRQ_NO_SPI1					35
#define IRQ_NO_SPI2					36
#define IRQ_NO_USART1				37
#define IRQ_NO_USART2				38
#define IRQ_NO_USART3				39
#define IRQ_NO_EXTI15_10			40
#define IRQ_NO_RTC_Alarm			41
#define IRQ_NO_OTG_FS_WKUP			42
#define IRQ_NO_TIM8_BRK_TIM12		43
#define IRQ_NO_TIM8_UP_TIM13		44
#define IRQ_NO_TIM8_TRG_COM_TIM14	45
#define IRQ_NO_TIM8_CC				46
#define IRQ_NO_DMA1_Stream7			47
#define IRQ_NO_FSMC					48
#define IRQ_NO_SDIO					49
#define IRQ_NO_TIM5					50
#define IRQ_NO_SPI3					51
#define IRQ_NO_UART4				52
#define IRQ_NO_UART5				53
#define IRQ_NO_TIM6_DAC				54
#define IRQ_NO_TIM7					55
#define IRQ_NO_DMA2_Stream0			56
#define IRQ_NO_DMA2_Stream1			57
#define IRQ_NO_DMA2_Stream2			58
#define IRQ_NO_DMA2_Stream3			59
#define IRQ_NO_DMA2_Stream4			60
#define IRQ_NO_ETH					61
#define IRQ_NO_ETH_WKUP				62
#define IRQ_NO_CAN2_TX				63
#define IRQ_NO_CAN2_RX0				64
#define IRQ_NO_CAN2_RX1				65
#define IRQ_NO_CAN2_SCE				66
#define IRQ_NO_OTG_FS				67
#define IRQ_NO_DMA2_Stream5			68
#define IRQ_NO_DMA2_Stream6			69
#define IRQ_NO_DMA2_Stream7			70
#define IRQ_NO_USART6				71
#define IRQ_NO_I2C3_EV				72
#define IRQ_NO_I2C3_ER				73
#define IRQ_NO_OTG_HS_EP1_OUT		74
#define IRQ_NO_OTG_HS_EP1_IN		75
#define IRQ_NO_OTG_HS_WKUP			76
#define IRQ_NO_OTG_HS				77
#define IRQ_NO_DCMI					78
#define IRQ_NO_CRYP					79
#define IRQ_NO_HASH_RNG				80
#define IRQ_NO_FPU					81

#define NVIC_IRQ_PRI0 	0
#define NVIC_IRQ_PRI1 	1
#define NVIC_IRQ_PRI2 	2
#define NVIC_IRQ_PRI3 	3
#define NVIC_IRQ_PRI4 	4
#define NVIC_IRQ_PRI5 	5
#define NVIC_IRQ_PRI6 	6
#define NVIC_IRQ_PRI7 	7
#define NVIC_IRQ_PRI8 	8
#define NVIC_IRQ_PRI9 	9
#define NVIC_IRQ_PRI10 	10
#define NVIC_IRQ_PRI11	11
#define NVIC_IRQ_PRI12	12
#define NVIC_IRQ_PRI13	13
#define NVIC_IRQ_PRI14	14
#define NVIC_IRQ_PRI15	15





#define ENABLE 			1
#define DISABLE			0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET

#include "stm32f407xx_gpio_driver.h"

#endif /* INC_STM32F407XX_H_ */
