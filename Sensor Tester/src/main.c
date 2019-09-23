//********************************************************************
//*             EEE3099S Sensor Tester Code                          *
//*==================================================================*
//* WRITTEN BY: Tonderai Said and Zayd Osman     	                 *
//* DATE CREATED: 19 September 2019                                  *
//* MODIFIED:                                                        *
//*==================================================================*
//* PROGRAMMED IN: Eclipse Luna Service Release 1 (4.4.1)            *
//* DEV. BOARD:    UCT STM32 Development Board                       *
//* TARGET:	   STMicroelectronics STM32F051C6                    	 *
//*==================================================================*
//* DESCRIPTION:                                                 	 *
//*                                                                  *
//********************************************************************
// INCLUDE FILES
//====================================================================
#include <stdio.h>
#include "stm32f0xx.h"
//====================================================================
//Constants




#define	lineRightPin				GPIO_IDR_15		//Right Line Sensor
#define	lineCenterPin				GPIO_IDR_14		//Center Line Sensor
#define	lineLeftPin					GPIO_IDR_13		//Left Line Sensor
#define wideLeftPin					GPIO_IDR_12		//left sensor
#define wideRightPin				GPIO_IDR_10		//right sensor
#define startDelay					1000000			//Delay after the Go Button is released
#define flashDelay					1500000			//Delay used when flashing the Status LE

//global variables

    uint8_t mode = 0;				//0=ERROR, 1=READY, 2=DRIVE, 3=Right Turn, 4=Left Turn, 5=STOP
    uint16_t leftDuty;
    uint16_t rightDuty;
    uint16_t MAX_DUTY = 50;
    uint16_t STEP_DOWN_1 = 20;


//Function Declarations
 //maze mapping functions


 //General robot functions
    void init_Ports(void);


//Hardware interfacing functions-------------------------------------------------------------------------------------
void init_Ports() {

    	//Enable Clock for Port A
    	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    	//Enable the clock for Port B
    	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    	//Set B0 (Go Button), B4-B7 and B9 (line Sensors) as logic inputs
    	GPIOB->MODER &= ~(GPIO_MODER_MODER15|
				GPIO_MODER_MODER14|
				GPIO_MODER_MODER13|
				GPIO_MODER_MODER12|
				GPIO_MODER_MODER10);

    	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR15_1;
    	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR14_1;
    	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR13_1;
    	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR12_1;
    	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR10_1;

    	//Set PB7 as an output (Status LED)
    	GPIOB->MODER |= GPIO_MODER_MODER7_0;
    	GPIOB->MODER |= GPIO_MODER_MODER6_0;
    	GPIOB->MODER |= GPIO_MODER_MODER5_0;
    	GPIOB->MODER |= GPIO_MODER_MODER4_0;
    	GPIOB->MODER |= GPIO_MODER_MODER3_0;


    	GPIOB->ODR &= 0;


}


//Maze Mapping Functions--------------------------------------------------------------------------------------



//-----------------------------------------------------------------------------------------------------------

// Main function
void main(void) {

	//Initialization
	init_Ports();


    //Loop forever
    for(;;)

    	if (GPIOB->IDR & wideRightPin){

    				//Turn on LED
    				GPIOB->ODR |= GPIO_ODR_7;

    			}

    			else if(GPIOB->IDR & wideLeftPin){

    				//Turn on Status LED
    				GPIOB->ODR |= GPIO_ODR_6;

    			}

    			else if(GPIOB->IDR & lineCenterPin){

    				//Turn on Status LED
    				GPIOB->ODR |= GPIO_ODR_5;

    			}

    			else if(GPIOB->IDR & lineRightPin){

    				//Turn on Status LED
    				GPIOB->ODR |= GPIO_ODR_4;

    			}

    			else if(GPIOB->IDR & lineLeftPin){

    				//Turn on Status LED
    				GPIOB->ODR |= GPIO_ODR_3;

    			}

    			else{

    				GPIOB->ODR &= 0;

    			}

    ;



}



