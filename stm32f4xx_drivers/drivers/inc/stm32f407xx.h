/*
 * stm32f407xx.h
 *
 *  Created on: Jun 26, 2024
 *      Author:
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>


#define __vo volatile
#define ENABLE 			1
#define DISABLE			0
#define SET 			ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET


/* macros for Resetting x bit Masks */

#define RESET_MASK_2_BIT	0b11
#define RESET_MASK_4_BIT	0b1111



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/******************************* Processor Specific Details (ARM Cortex Mx) (see the Processor Documents)******************************************/

 /* ARM Cortex Mx Processor NVIC ISERx register Addresses
  */

#define NVIC_ISER0_BASEADDR				( (__vo uint32_t *)  0xE000E100 )	/* NVIC Interrupt Set-Enable Register 0 */
#define NVIC_ISER1_BASEADDR				( (__vo uint32_t *)  0xE000E104 )	/* NVIC Interrupt Set-Enable Register 1 */
#define NVIC_ISER2_BASEADDR				( (__vo uint32_t *)  0xE000E108 )	/* NVIC Interrupt Set-Enable Register 2 */
#define NVIC_ISER3_BASEADDR				( (__vo uint32_t *)  0xE000E10C )	/* NVIC Interrupt Set-Enable Register 3 */


/* ARM Cortex Mx Processor NVIC ICERx register Addresses
  */

#define NVIC_ICER0_BASEADDR				( (__vo uint32_t *)  0XE000E180 )	/* NVIC Interrupt Clear-Enable Register 0 */
#define NVIC_ICER1_BASEADDR				( (__vo uint32_t *)  0XE000E184 )	/* NVIC Interrupt Clear-Enable Register 1 */
#define NVIC_ICER2_BASEADDR				( (__vo uint32_t *)  0XE000E188 )	/* NVIC Interrupt Clear-Enable Register 2 */
#define NVIC_ICER3_BASEADDR				( (__vo uint32_t *)  0XE000E18C )	/* NVIC Interrupt Clear-Enable Register 3 */


/* ARM Cortex Mx Processor NVIC IPRx register Addresses
  */

#define NVIC_IPR_BASEADDR				( (__vo uint32_t *)  0xE000E400 )	/* NVIC Interrupt Priority Register */

#define NUMBER_OF_PRIORITY_BITS_IMPLEMENTED			4			// number of Priority Bits Implemented in NVIC IPR Register (Interrupt Priority Register) (it's Different from Processor to another)







///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/***************************************  		Micro-controller Specific Details	(STM32F407)	****************************************/

/*********************** the base Addresses of the Flash & SRAM memories of the MCU *******************/


#define FLASH_BASEADDR				0x08000000U 		/* (Main Memory)  Flash Base Address*/
#define SRAM1_BASEADDR				0x20000000U  		/* SRAM1 Base Address*/
#define SRAM						SRAM1_BASEADDR
#define SRAM2_BASEADDR				0x2001C000U			/* SRAM2 Base Address*/
#define ROM_BASEADDR 				0x1FFF0000U			/* System Memory (ROM) Base Address*/





/* ******************** AHBx and APBx Bus Peripheral Base Addresses ************************ */

#define PERIPH_BASEADDR				0x40000000U		/* Peripheral Base Address*/
#define APB1PERIPH_BASEADDR			PERIPH_BASEADDR 	/* APB 1 Peripherals Bus Base Address */
#define APB2PERIPH_BASEADDR			0x40010000U		/* APB 2 Peripherals Bus Base Address */
#define AHB1PERIPH_BASEADDR			0x40020000U		/* AHB 1 Peripherals Bus Base Address */
#define AHB2PERIPH_BASEADDR			0x50000000U		/* AHB 2 Peripherals Bus Base Address */


/* **************************** AHB1 Bus -> Base Addresses of Peripherals *****************************
 * */

#define GPIOA_BASEADDR				( AHB1PERIPH_BASEADDR + 0x0000 )  	/*  GPIO Port A Base Address */
#define GPIOB_BASEADDR				( AHB1PERIPH_BASEADDR + 0x0400 )	/*  GPIO Port B Base Address */
#define GPIOC_BASEADDR				( AHB1PERIPH_BASEADDR + 0x0800 )	/*  GPIO Port C Base Address */
#define GPIOD_BASEADDR				( AHB1PERIPH_BASEADDR + 0x0C00 )	/*  GPIO Port D Base Address */
#define GPIOE_BASEADDR				( AHB1PERIPH_BASEADDR + 0x1000 )	/*  GPIO Port E Base Address */
#define GPIOF_BASEADDR				( AHB1PERIPH_BASEADDR + 0x1400 )	/*  GPIO Port F Base Address */
#define GPIOG_BASEADDR				( AHB1PERIPH_BASEADDR + 0x1800 )	/*  GPIO Port G Base Address */
#define GPIOH_BASEADDR				( AHB1PERIPH_BASEADDR + 0x1C00 )	/*  GPIO Port H Base Address */
#define GPIOI_BASEADDR				( AHB1PERIPH_BASEADDR + 0x2000 )	/*  GPIO Port I Base Address */

#define RCC_BASEADDR 				( AHB1PERIPH_BASEADDR + 0x3800 )	/*  RCC Base Address */


/* **************************** APB1 Bus -> Base Addresses of Peripherals******************
 * */

/*we are interested in those peripherals: I2C1 , I2C2 , I2C3 , SPI2 , SPI3 , USART2 , USART3 , UART4 , UART5 */

#define SPI2_BASEADDR				( APB1PERIPH_BASEADDR + 0x3800 )  /*  SPI 2 Base Address */
#define SPI3_BASEADDR				( APB1PERIPH_BASEADDR + 0x3C00 )  /*  SPI 3 Base Address */

#define USART2_BASEADDR				( APB1PERIPH_BASEADDR + 0x4400 )  /*  USART 2 Base Address */
#define USART3_BASEADDR				( APB1PERIPH_BASEADDR + 0x4800 )  /*  USART 3 Base Address */
#define UART4_BASEADDR				( APB1PERIPH_BASEADDR + 0x4C00 )  /*  UART 4 Base Address */
#define UART5_BASEADDR				( APB1PERIPH_BASEADDR + 0x5000 )  /*  UART 5 Base Address */

#define I2C1_BASEADDR				( APB1PERIPH_BASEADDR + 0x5400 )  /*  I2C 1 Base Address */
#define I2C2_BASEADDR				( APB1PERIPH_BASEADDR + 0x5800 )  /*  I2C 2 Base Address */
#define I2C3_BASEADDR				( APB1PERIPH_BASEADDR + 0x5C00 )  /*  I2C 3 Base Address */






/**************************** APB2 Bus  ->  Base Addresses of Peripherals**********************
 * */

/*we are interested in those peripherals: SPI1 , USART1 , USART6 , EXTI , SYSCFG */



#define USART1_BASEADDR				( APB2PERIPH_BASEADDR + 0x1000 )	/*USART 1 Base Address*/
#define USART6_BASEADDR				( APB2PERIPH_BASEADDR + 0x1400 )	/*USART 6 Base Address*/

#define SPI1_BASEADDR				( APB2PERIPH_BASEADDR + 0x3000 )	/*SPI 1 Base Address*/
#define SPI4_BASEADDR				( APB2PERIPH_BASEADDR + 0x3400 )	/*SPI 4 Base Address*/
#define SPI5_BASEADDR				( APB2PERIPH_BASEADDR + 0x5000 )	/*SPI 5 Base Address*/
#define SPI6_BASEADDR				( APB2PERIPH_BASEADDR + 0x5400 )	/*SPI 6 Base Address*/

#define SYSCFG_BASEADDR				( APB2PERIPH_BASEADDR + 0x3800 )	/*System configuration Peripheral Base Address*/

#define EXTI_BASEADDR				( APB2PERIPH_BASEADDR + 0x3C00 )	/*EXTI Base Address*/










/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/************ Peripheral Definitions ( Peripheral base Addresses type-casted to xyz_RegDef_t )************ */



#define GPIOA		((GPIO_RegDef_t *)GPIOA_BASEADDR)    /*to be used here:  GPIO_RegDef_t *pGPIOA = (GPIO_RegDef_t *)GPIOA_BASEADDR;     */
#define GPIOB 		((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC	 	((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD	 	((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE	 	((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF	 	((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG	 	((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH	 	((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI	 	((GPIO_RegDef_t *)GPIOI_BASEADDR)


#define RCC	 		((RCC_RegDef_t *)RCC_BASEADDR)

#define EXTI		((EXTI_RegDef_t *)EXTI_BASEADDR)

#define SYSCFG		((SYSCFG_RegDef_t *) SYSCFG_BASEADDR)

#define SPI1		((SPI_RegDef_t *)	SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t *)	SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t *)	SPI3_BASEADDR)
#define SPI4		((SPI_RegDef_t *)	SPI4_BASEADDR)
#define SPI5		((SPI_RegDef_t *)	SPI5_BASEADDR)
#define SPI6		((SPI_RegDef_t *)	SPI6_BASEADDR)


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/******************************* Peripheral Register Definition Structures ***************************************/


/*
 * Note: Registers of a Peripheral  are Exclusively specific to each MCU
 * e.g.: Number of Registers of SPI peripheral of STM32F4x family of MCUs may be different (more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please Check your Device's R.M. (Reference Manual)
 *
 */



/* GPIO  Peripheral Register Definition Structures*/

typedef struct
{
	__vo uint32_t MODER;	/* GPIO port mode register , 					Address offset : 0x00 */
	__vo uint32_t OTYPER;	/* GPIO port output type register , 			Address offset : 0x04 */
	__vo uint32_t OSPEEDR;	/* GPIO port output speed register ,			Address offset : 0x08 */
	__vo uint32_t PUPDR;	/* GPIO port pull-up/pull-down register ,		Address offset : 0x0C */
	__vo uint32_t IDR;		/* GPIO port input data register , 				Address offset : 0x10 */
	__vo uint32_t ODR;		/* GPIO port output data register , 			Address offset : 0x14 */
	__vo uint32_t BSRR;		/* GPIO port bit set/reset register , 			Address offset : 0x18 */
	__vo uint32_t LCKR;		/* GPIO port configuration lock register , 		Address offset : 0x1C */
	__vo uint32_t AFRL;		/* GPIO alternate function low register , 		Address offset : 0x20 */
	__vo uint32_t AFRH;		/* GPIO alternate function high register , 		Address offset : 0x24 */

}GPIO_RegDef_t;


/* RCC Peripheral Register Definition Structures*/


typedef struct
{

	__vo uint32_t   CR;				/* RCC clock control register    , 								 Address offset: 0x00 */
	__vo uint32_t   PLLCFGR;		/* RCC PLL configuration register    , 							 Address offset: 0x04 */
	__vo uint32_t 	CFGR;			/* RCC clock configuration register    , 						 Address offset: 0x08 */
	__vo uint32_t 	CIR;			/* RCC clock interrupt register    , 							 Address offset: 0x0C */
	__vo uint32_t 	AHB1RSTR;		/* RCC AHB1 peripheral reset register    ,      				 Address offset: 0x10 */
	__vo uint32_t 	AHB2RSTR;		/* RCC AHB2 peripheral reset register    , 						 Address offset: 0x14 */
	__vo uint32_t   AHB3RSTR;		/* RCC AHB3 peripheral reset register    , 						 Address offset: 0x18 */
	     uint32_t	RESERVED0;				/* Reserved    , Address offset: 0x1C */
	__vo uint32_t   APB1RSTR;		/* RCC APB1 peripheral reset register    , 						 Address offset: 0x20 */
	__vo uint32_t   APB2RSTR;		/* RCC APB2 peripheral reset register    ,						 Address offset: 0x24 */
		 uint32_t  	RESERVED1;				/* Reserved    , Address offset: 0x28 */
		 uint32_t   RESERVED2;				/* Reserved    , Address offset: 0x2C */
	__vo uint32_t   AHB1ENR;		/* RCC AHB1 peripheral clock enable register    ,				 Address offset: 0x30 */
	__vo uint32_t   AHB2ENR;		/* RCC AHB2 peripheral clock enable register    ,				 Address offset: 0x34 */
	__vo uint32_t   AHB3ENR;		/* RCC AHB3 peripheral clock enable register    ,				 Address offset: 0x38 */
		 uint32_t	RESERVED3;				/* Reserved    , Address offset: 0x3C */
	__vo uint32_t   APB1ENR;		/* RCC APB1 peripheral clock enable register    , 				 Address offset: 0x40 */
	__vo uint32_t   APB2ENR;		/* RCC APB2 peripheral clock enable register    ,				 Address offset: 0x44 */
		 uint32_t	RESERVED4;				/* Reserved    , Address offset: 0x48 */
		 uint32_t	RESERVED5;				/* Reserved    , Address offset: 0x4C */
	__vo uint32_t  	AHB1LPENR;		/* RCC AHB1 peripheral clock enable in low power mode register , Address offset: 0x50 */
	__vo uint32_t   AHB2LPENR;		/* RCC AHB2 peripheral clock enable in low power mode register , Address offset: 0x54 */
	__vo uint32_t   AHB3LPENR;		/* RCC AHB3 peripheral clock enable in low power mode register , Address offset: 0x58 */
		 uint32_t  	RESERVED6;				/* Reserved    , Address offset: 0x5C */
	__vo uint32_t   APB1LPENR;		/* RCC APB1 peripheral clock enable in low power mode register , Address offset: 0x60 */
	__vo uint32_t   APB2LPENR;		/* RCC APB2 peripheral clock enabled in low power mode register, Address offset: 0x64 */
		 uint32_t   RESERVED7;				/* Reserved    , Address offset: 0x68 */
		 uint32_t   RESERVED8;				/* Reserved    , Address offset: 0x6C */
	__vo uint32_t   BDCR;			/* RCC Backup domain control register    , 						 Address offset: 0x70 */
	__vo uint32_t   CSR;			/* RCC clock control & status register    , 					 Address offset: 0x74 */
		 uint32_t   RESERVED9;				/* Reserved    , Address offset: 0x78 */
		 uint32_t   RESERVED10;				/* Reserved    , Address offset: 0x7C */
	__vo uint32_t   SSCGR;			/* RCC spread spectrum clock generation register    , 			 Address offset: 0x80 */
	__vo uint32_t   PLLI2SCFGR;		/* RCC PLLI2S configuration register    , 						 Address offset: 0x84 */


}RCC_RegDef_t;


/* EXTI Peripheral Register Definition Structures*/

typedef struct
{
	__vo uint32_t IMR ; 			/* EXTI Interrupt mask register    , 				Address offset: 0x00 */
	__vo uint32_t EMR ;				/* EXTI Event mask register    , 				 	Address offset: 0x04 */
	__vo uint32_t RTSR ;			/* EXTI Rising trigger selection register    , 		Address offset: 0x08 */
	__vo uint32_t FTSR ;			/* EXTI Falling trigger selection register    , 	Address offset: 0x0C */
	__vo uint32_t SWIER ;			/* EXTI Software interrupt event register    , 		Address offset: 0x10 */
	__vo uint32_t PR ;				/* EXTI Pending register    , 						Address offset: 0x14 */


}EXTI_RegDef_t ;


/* SYSCFG Peripheral Register Definition Structures*/

typedef struct
{
	__vo uint32_t  MEMRMP ; 		/* SYSCFG memory remap register    , 											Address offset: 0x00 */
	__vo uint32_t  PMC ; 			/* SYSCFG peripheral mode configuration register , 								Address offset: 0x04 */
	__vo uint32_t  EXTICR[4] ; 		/* EXTICR[0] = EXTICR1 SYSCFG external interrupt configuration register 1    , 	Address offset: 0x08 */
									/* EXTICR[1] = EXTICR2 SYSCFG external interrupt configuration register 2    , 	Address offset: 0x0C */
									/* EXTICR[2] = EXTICR3 SYSCFG external interrupt configuration register 3    ,	Address offset: 0x10 */
									/* EXTICR[3] = EXTICR4 SYSCFG external interrupt configuration register 4    , 	Address offset: 0x14 */

		 uint32_t	Reserved0[2];	/* 			Reserved    , 												Address offset:  0x18 , 0x1C */
	__vo uint32_t  CMPCR ; 			/* SYSCFG Compensation cell control register    , 								Address offset: 0x20 */



}SYSCFG_RegDef_t;



/* SPI Peripheral Register Definition Structures*/

typedef struct

{
	__vo uint32_t CR1;				/* SPI control register 1 (not used in I2S mode)   , 				Address offset: 0x00 */
	__vo uint32_t CR2;				/* SPI control register 2   , 										Address offset: 0x04 */
	__vo uint32_t SR;				/* SPI status register    , 										Address offset: 0x08 */
	__vo uint32_t DR;				/* SPI data register    , 											Address offset: 0x0C */
	__vo uint32_t CRCPR;				/* SPI CRC polynomial register (not used in I2S mode)   , 			Address offset: 0x10 */
	__vo uint32_t RXCRCR;			/* SPI RX CRC register (not used in I2S mode)   ,  					Address offset: 0x14 */
	__vo uint32_t TXCRCR;			/* SPI TX CRC register (not used in I2S mode)   , 					Address offset: 0x18 */
	__vo uint32_t I2SCFGR;			/* SPI_I2S configuration register    , 								Address offset: 0x1C */
	__vo uint32_t I2SPR;				/* SPI_I2S prescaler register    , 									Address offset: 0x20 */


}SPI_RegDef_t;














//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/**************************** Clock Enable & Disable Macros *********************************/





/* GPIOx peripherals Clock Enable Macros
 */

#define GPIOA_PCLK_EN()			( RCC->AHB1ENR |= (1<<0) )		/* Enable GPIO port A Peripheral Clock */
#define GPIOB_PCLK_EN()			( RCC->AHB1ENR |= (1<<1) )		/* Enable GPIO port B Peripheral Clock */
#define GPIOC_PCLK_EN()			( RCC->AHB1ENR |= (1<<2) )		/* Enable GPIO port C Peripheral Clock */
#define GPIOD_PCLK_EN()			( RCC->AHB1ENR |= (1<<3) )		/* Enable GPIO port D Peripheral Clock */
#define GPIOE_PCLK_EN()			( RCC->AHB1ENR |= (1<<4) )		/* Enable GPIO port E Peripheral Clock */
#define GPIOF_PCLK_EN()			( RCC->AHB1ENR |= (1<<5) )		/* Enable GPIO port F Peripheral Clock */
#define GPIOG_PCLK_EN()			( RCC->AHB1ENR |= (1<<6) )		/* Enable GPIO port G Peripheral Clock */
#define GPIOH_PCLK_EN()			( RCC->AHB1ENR |= (1<<7) )		/* Enable GPIO port H Peripheral Clock */
#define GPIOI_PCLK_EN()			( RCC->AHB1ENR |= (1<<8) )		/* Enable GPIO port I Peripheral Clock */


/* GPIOx peripherals Clock Disable Macros
 */
#define GPIOA_PCLK_DI()			( RCC->AHB1ENR &= ~(1<<0) )		/* Disable GPIO port A Peripheral Clock */
#define GPIOB_PCLK_DI()			( RCC->AHB1ENR &= ~(1<<1) )		/* Disable GPIO port B Peripheral Clock */
#define GPIOC_PCLK_DI()			( RCC->AHB1ENR &= ~(1<<2) )		/* Disable GPIO port C Peripheral Clock */
#define GPIOD_PCLK_DI()			( RCC->AHB1ENR &= ~(1<<3) )		/* Disable GPIO port D Peripheral Clock */
#define GPIOE_PCLK_DI()			( RCC->AHB1ENR &= ~(1<<4) )		/* Disable GPIO port E Peripheral Clock */
#define GPIOF_PCLK_DI()			( RCC->AHB1ENR &= ~(1<<5) )		/* Disable GPIO port F Peripheral Clock */
#define GPIOG_PCLK_DI()			( RCC->AHB1ENR &= ~(1<<6) )		/* Disable GPIO port G Peripheral Clock */
#define GPIOH_PCLK_DI()			( RCC->AHB1ENR &= ~(1<<7) )		/* Disable GPIO port H Peripheral Clock */
#define GPIOI_PCLK_DI()			( RCC->AHB1ENR &= ~(1<<8) )		/* Disable GPIO port I Peripheral Clock */






/* I2Cx peripherals Clock Enable Macros
 */


#define I2C1_PCLK_EN()			( RCC->APB1ENR |= (1<<21) )		/* Enable I2C1 Peripheral Clock */
#define I2C2_PCLK_EN()			( RCC->APB1ENR |= (1<<22) )		/* Enable I2C2 Peripheral Clock */
#define I2C3_PCLK_EN()			( RCC->APB1ENR |= (1<<23) )		/* Enable I2C3 Peripheral Clock */



/* I2Cx peripherals Clock Disable Macros
 */

#define I2C1_PCLK_DI()			( RCC->APB1ENR &= ~(1<<21) )		/* Disable I2C1 Peripheral Clock */
#define I2C2_PCLK_DI()			( RCC->APB1ENR &= ~(1<<22) )		/* Disable I2C2 Peripheral Clock */
#define I2C3_PCLK_DI()			( RCC->APB1ENR &= ~(1<<23) )		/* Disable I2C3 Peripheral Clock */



/* SPIx peripherals Clock Enable Macros
 */

#define SPI1_PCLK_EN()			( RCC->APB2ENR |= (1<<12) )		/* Enable SPI1 Peripheral Clock */
#define SPI2_PCLK_EN()			( RCC->APB1ENR |= (1<<14) )		/* Enable SPI2 Peripheral Clock */
#define SPI3_PCLK_EN()			( RCC->APB1ENR |= (1<<15) )		/* Enable SPI3 Peripheral Clock */
#define SPI4_PCLK_EN()			( RCC->APB2ENR |= (1<<13) )		/* Enable SPI4 Peripheral Clock */
#define SPI5_PCLK_EN()			( RCC->APB2ENR |= (1<<20) )		/* Enable SPI5 Peripheral Clock */
#define SPI6_PCLK_EN()			( RCC->APB2ENR |= (1<<21) )		/* Enable SPI6 Peripheral Clock */



/* SPIx peripherals Clock Disable Macros
 */

#define SPI1_PCLK_DI()			( RCC->APB2ENR &= ~(1<<12) )		/* Disable SPI1 Peripheral Clock */
#define SPI2_PCLK_DI()			( RCC->APB1ENR &= ~(1<<14) )		/* Disable SPI2 Peripheral Clock */
#define SPI3_PCLK_DI()			( RCC->APB1ENR &= ~(1<<15) )		/* Disable SPI3 Peripheral Clock */
#define SPI4_PCLK_DI()			( RCC->APB2ENR &= ~(1<<13) )		/* Disable SPI4 Peripheral Clock */
#define SPI5_PCLK_DI()			( RCC->APB2ENR &= ~(1<<20) )		/* Disable SPI5 Peripheral Clock */
#define SPI6_PCLK_DI()			( RCC->APB2ENR &= ~(1<<21) )		/* Disable SPI6 Peripheral Clock */



/* USARTx peripherals Clock Enable Macros */

#define USART1_PCLK_EN()			( RCC->APB2ENR |= (1<<4) )	    /* Enable USART1 Peripheral Clock */
#define USART2_PCLK_EN()			( RCC->APB1ENR |= (1<<17) )		/* Enable USART2 Peripheral Clock */
#define USART3_PCLK_EN()			( RCC->APB1ENR |= (1<<18) )		/* Enable USART3 Peripheral Clock */
#define UART4_PCLK_EN()				( RCC->APB1ENR |= (1<<19) )		/* Enable UART4 Peripheral Clock */
#define UART5_PCLK_EN()				( RCC->APB1ENR |= (1<<20) )		/* Enable UART5 Peripheral Clock */
#define USART6_PCLK_EN()			( RCC->APB2ENR |= (1<<5) )		/* Enable USART6 Peripheral Clock */



/* USARTx peripherals Clock Disable Macros */

#define USART1_PCLK_DI()			( RCC->APB2ENR &= ~(1<<4) )	  		/* Disable USART1 Peripheral Clock */
#define USART2_PCLK_DI()			( RCC->APB1ENR &= ~(1<<17) )		/* Disable USART2 Peripheral Clock */
#define USART3_PCLK_DI()			( RCC->APB1ENR &= ~(1<<18) )		/* Disable USART3 Peripheral Clock */
#define UART4_PCLK_DI()				( RCC->APB1ENR &= ~(1<<19) )		/* Disable UART4 Peripheral Clock */
#define UART5_PCLK_DI()				( RCC->APB1ENR &= ~(1<<20) )		/* Disable UART5 Peripheral Clock */
#define USART6_PCLK_DI()			( RCC->APB2ENR &= ~(1<<5) )			/* Disable USART6 Peripheral Clock */






/* SYSCFG peripherals Clock Enable Macros */

#define SYSCFG_PCLK_EN()			( RCC->APB2ENR |= (1<<14) )	    /* Enable SYSCFG Peripheral Clock */


/* SYSCFG peripherals Clock Disable Macros */

#define SYSCFG_PCLK_DI()			( RCC->APB2ENR &= ~(1<<14) )	/* Disable SYSCFG Peripheral Clock */



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*********************************** Macros to Reset Peripherals Registers ***********************************/
// to reset a peripheral you should Set then clear its corresponding bit in RCC reset registers



/* do{ Statement1; Statement2; } while(0);
 *
 * do .. while condition 0 loop:
 * this is a technique in C Programming to Execute Multiple Statements using single Macro
 *  */


/* Macros to RESET GPIOx Registers */

#define GPIOA_REGISTERS_RESET()			do{  ( RCC->AHB1RSTR |= (1<<0)) ; ( RCC->AHB1RSTR &= ~(1<<0)) ; }while(0)
//don't put a semicolon ; here at the End of the macro because we will put a semicolon after we call this macro

#define GPIOB_REGISTERS_RESET()			do{  ( RCC->AHB1RSTR |= (1<<1)) ; ( RCC->AHB1RSTR &= ~(1<<1)) ; }while(0)
#define GPIOC_REGISTERS_RESET()			do{  ( RCC->AHB1RSTR |= (1<<2)) ; ( RCC->AHB1RSTR &= ~(1<<2)) ; }while(0)
#define GPIOD_REGISTERS_RESET()			do{  ( RCC->AHB1RSTR |= (1<<3)) ; ( RCC->AHB1RSTR &= ~(1<<3)) ; }while(0)
#define GPIOE_REGISTERS_RESET()			do{  ( RCC->AHB1RSTR |= (1<<4)) ; ( RCC->AHB1RSTR &= ~(1<<4)) ; }while(0)
#define GPIOF_REGISTERS_RESET()			do{  ( RCC->AHB1RSTR |= (1<<5)) ; ( RCC->AHB1RSTR &= ~(1<<5)) ; }while(0)
#define GPIOG_REGISTERS_RESET()			do{  ( RCC->AHB1RSTR |= (1<<6)) ; ( RCC->AHB1RSTR &= ~(1<<6)) ; }while(0)
#define GPIOH_REGISTERS_RESET()			do{  ( RCC->AHB1RSTR |= (1<<7)) ; ( RCC->AHB1RSTR &= ~(1<<7)) ; }while(0)
#define GPIOI_REGISTERS_RESET()			do{  ( RCC->AHB1RSTR |= (1<<8)) ; ( RCC->AHB1RSTR &= ~(1<<8)) ; }while(0)




/* Macros to RESET SPIx Registers */

#define SPI1_REGISTERS_RESET()			do{  ( RCC->APB2RSTR |= (1<<12)) ; ( RCC->APB2RSTR &= ~(1<<12)) ; }while(0)
#define SPI2_REGISTERS_RESET()			do{  ( RCC->APB1RSTR |= (1<<14)) ; ( RCC->APB1RSTR &= ~(1<<14)) ; }while(0)
#define SPI3_REGISTERS_RESET()			do{  ( RCC->APB1RSTR |= (1<<15)) ; ( RCC->APB1RSTR &= ~(1<<15)) ; }while(0)





///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/********************************* IRQ ( Interrupt Request ) of STM32F407x MCU *******************************/



/* IRQ ( Interrupt Request ) that goes through EXTI line Number of STM32F407x MCU
 *
 * @IRQ_NUMBER_EXTI
 *
 */
#define IRQ_NUMBER_EXTI0		6
#define IRQ_NUMBER_EXTI1		7
#define IRQ_NUMBER_EXTI2		8
#define IRQ_NUMBER_EXTI3		9
#define IRQ_NUMBER_EXTI4		10
#define IRQ_NUMBER_EXTI5_9		23
#define IRQ_NUMBER_EXTI10_15	40



/* SPI peripheral  IRQ ( Interrupt Request )
 *
 * @IRQ_NUMBER_SPI
 *
 */

#define IRQ_NUMBER_SPI1		35		// SPI1 global interrupt IRQ Number
#define IRQ_NUMBER_SPI2		36		// SPI2 global interrupt IRQ Number
#define IRQ_NUMBER_SPI3		51		// SPI3 global interrupt IRQ Number








/***************************************  NVIC Priority Number of an IRQ ************************************************/


/*
 * NVIC Priority Number of an IRQ
 *
 * @NVIC_PRIORITY_NUMBER
 * */

/*Generated by using Excel sheet
 * Using this Function
 	 ="#define" &" "& "NVIC_IRQ_PRIORITY_" & B1 &"                "& B1
 * and set B column to be from 0 to 100 using {0 , B1+1 , Drag the Plus sign in the down right corner until you reach row 100}
 *  */

#define NVIC_IRQ_PRIORITY_0                	0
#define NVIC_IRQ_PRIORITY_1               	1
#define NVIC_IRQ_PRIORITY_2                	2
#define NVIC_IRQ_PRIORITY_3                	3
#define NVIC_IRQ_PRIORITY_4                	4
#define NVIC_IRQ_PRIORITY_5                	5
#define NVIC_IRQ_PRIORITY_6                	6
#define NVIC_IRQ_PRIORITY_7                	7
#define NVIC_IRQ_PRIORITY_8                	8
#define NVIC_IRQ_PRIORITY_9                	9
#define NVIC_IRQ_PRIORITY_10                10
#define NVIC_IRQ_PRIORITY_11                11
#define NVIC_IRQ_PRIORITY_12                12
#define NVIC_IRQ_PRIORITY_13                13
#define NVIC_IRQ_PRIORITY_14                14
#define NVIC_IRQ_PRIORITY_15                15
#define NVIC_IRQ_PRIORITY_16                16
#define NVIC_IRQ_PRIORITY_17                17
#define NVIC_IRQ_PRIORITY_18                18
#define NVIC_IRQ_PRIORITY_19                19
#define NVIC_IRQ_PRIORITY_20                20
#define NVIC_IRQ_PRIORITY_21                21
#define NVIC_IRQ_PRIORITY_22                22
#define NVIC_IRQ_PRIORITY_23                23
#define NVIC_IRQ_PRIORITY_24                24
#define NVIC_IRQ_PRIORITY_25                25
#define NVIC_IRQ_PRIORITY_26                26
#define NVIC_IRQ_PRIORITY_27                27
#define NVIC_IRQ_PRIORITY_28                28
#define NVIC_IRQ_PRIORITY_29                29
#define NVIC_IRQ_PRIORITY_30                30
#define NVIC_IRQ_PRIORITY_31                31
#define NVIC_IRQ_PRIORITY_32                32
#define NVIC_IRQ_PRIORITY_33                33
#define NVIC_IRQ_PRIORITY_34                34
#define NVIC_IRQ_PRIORITY_35                35
#define NVIC_IRQ_PRIORITY_36                36
#define NVIC_IRQ_PRIORITY_37                37
#define NVIC_IRQ_PRIORITY_38                38
#define NVIC_IRQ_PRIORITY_39                39
#define NVIC_IRQ_PRIORITY_40                40
#define NVIC_IRQ_PRIORITY_41                41
#define NVIC_IRQ_PRIORITY_42                42
#define NVIC_IRQ_PRIORITY_43                43
#define NVIC_IRQ_PRIORITY_44                44
#define NVIC_IRQ_PRIORITY_45                45
#define NVIC_IRQ_PRIORITY_46                46
#define NVIC_IRQ_PRIORITY_47                47
#define NVIC_IRQ_PRIORITY_48                48
#define NVIC_IRQ_PRIORITY_49                49
#define NVIC_IRQ_PRIORITY_50                50
#define NVIC_IRQ_PRIORITY_51                51
#define NVIC_IRQ_PRIORITY_52                52
#define NVIC_IRQ_PRIORITY_53                53
#define NVIC_IRQ_PRIORITY_54                54
#define NVIC_IRQ_PRIORITY_55                55
#define NVIC_IRQ_PRIORITY_56                56
#define NVIC_IRQ_PRIORITY_57                57
#define NVIC_IRQ_PRIORITY_58                58
#define NVIC_IRQ_PRIORITY_59                59
#define NVIC_IRQ_PRIORITY_60                60
#define NVIC_IRQ_PRIORITY_61                61
#define NVIC_IRQ_PRIORITY_62                62
#define NVIC_IRQ_PRIORITY_63                63
#define NVIC_IRQ_PRIORITY_64                64
#define NVIC_IRQ_PRIORITY_65                65
#define NVIC_IRQ_PRIORITY_66                66
#define NVIC_IRQ_PRIORITY_67                67
#define NVIC_IRQ_PRIORITY_68                68
#define NVIC_IRQ_PRIORITY_69                69
#define NVIC_IRQ_PRIORITY_70                70
#define NVIC_IRQ_PRIORITY_71                71
#define NVIC_IRQ_PRIORITY_72                72
#define NVIC_IRQ_PRIORITY_73                73
#define NVIC_IRQ_PRIORITY_74                74
#define NVIC_IRQ_PRIORITY_75                75
#define NVIC_IRQ_PRIORITY_76                76
#define NVIC_IRQ_PRIORITY_77                77
#define NVIC_IRQ_PRIORITY_78                78
#define NVIC_IRQ_PRIORITY_79                79
#define NVIC_IRQ_PRIORITY_80                80
#define NVIC_IRQ_PRIORITY_81                81
#define NVIC_IRQ_PRIORITY_82                82
#define NVIC_IRQ_PRIORITY_83                83
#define NVIC_IRQ_PRIORITY_84                84
#define NVIC_IRQ_PRIORITY_85                85
#define NVIC_IRQ_PRIORITY_86                86
#define NVIC_IRQ_PRIORITY_87                87
#define NVIC_IRQ_PRIORITY_88                88
#define NVIC_IRQ_PRIORITY_89                89
#define NVIC_IRQ_PRIORITY_90                90
#define NVIC_IRQ_PRIORITY_91                91
#define NVIC_IRQ_PRIORITY_92                92
#define NVIC_IRQ_PRIORITY_93                93
#define NVIC_IRQ_PRIORITY_94                94
#define NVIC_IRQ_PRIORITY_95                95
#define NVIC_IRQ_PRIORITY_96                96
#define NVIC_IRQ_PRIORITY_97                97
#define NVIC_IRQ_PRIORITY_98                98
#define NVIC_IRQ_PRIORITY_99                99
#define NVIC_IRQ_PRIORITY_100               100















#endif /* INC_STM32F407XX_H_ */
