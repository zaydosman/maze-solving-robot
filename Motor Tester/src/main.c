//********************************************************************
//*             EEE3099S Motor Tester Code                           *
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


#define Delay					1000000			//Delay after the Go Button is released
#define flashDelay					1500000			//Delay used when flashing the Status LE

//global variables

    uint8_t mode = 0;				//0=ERROR, 1=READY, 2=DRIVE, 3=Right Turn, 4=Left Turn, 5=STOP
    uint16_t leftDuty=0;
    uint16_t increment=10;
    uint16_t rightDuty=0;
    uint16_t MAX_DUTY = 60;
    uint16_t STEP_UP_1 = 20;
    uint16_t STEP_DOWN_1 = 20;


//Function Declarations
 //maze mapping functions


 //General robot functions
    void init_Ports(void);
    void setDutyCycle(void);
    void init_Timer(void);
    void setDirectionForward(void);

//Hardware interfacing functions-------------------------------------------------------------------------------------
void init_Ports() {

    	//Enable Clock for Port A
    	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    	//Enable the clock for Port B
    	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    	//set switch buttons 0-3 to input mode
    	    		    GPIOA->MODER &= ~(GPIO_MODER_MODER0|
    	    					GPIO_MODER_MODER1|
    	    					GPIO_MODER_MODER2|
    	    					GPIO_MODER_MODER3);
    	        //set pull up resistors for pushbuttons
    	    		   GPIOA->PUPDR |= 0b00100100000000000000000001010101;

    	//Set A6-A7 as alternate function pins for PWM
    		GPIOA->MODER |= (GPIO_MODER_MODER6_1|
    						GPIO_MODER_MODER7_1);


    		   //Set B10-B13 as outputs (DIRECTION)
    		   	GPIOB->MODER |= (GPIO_MODER_MODER13_0|
    		   					GPIO_MODER_MODER12_0|
								GPIO_MODER_MODER11_0|
		    					GPIO_MODER_MODER10_0);
}

void init_Timer() {


	//Set B4-B5 alternate function as TIM3
	GPIOA->AFR[0] |= (1<<(6*4) | 1<<(7*4));

	//Enable clock for TIM3
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	//Configure TIM3 for output compare mode
	TIM3 -> CCMR1 &= ~TIM_CCMR1_CC1S;
	TIM3 -> CCMR1 &= ~TIM_CCMR1_CC2S;

	//Select PWM Mode 1
	TIM3 -> CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);
	TIM3 -> CCMR1 &= ~TIM_CCMR1_OC1M_0;

	TIM3 -> CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);
	TIM3 -> CCMR1 &= ~TIM_CCMR1_OC2M_0;

	//Set appropriate values for PSC, ARR AND CCR3
	TIM3 -> PSC = 0; 		//Write the value ‘0’ to TIM3_PSC
	TIM3 -> ARR = 47999; 	//Write the value ‘47999’ to TIM3_ARR
	TIM3 -> CCR1 = 0*480; 	//Initial Duty Cycle = 0
	TIM3 -> CCR2 = 0*480; 	//Initial Duty Cycle = 0

	//Set Edge aligned PWM
	TIM3 -> CR1 &= ~TIM_CR1_CMS;

	//Set Channel 1 & 2 as active high
	TIM3 -> CCER &= ~TIM_CCER_CC1P;
	TIM3 -> CCER &= ~TIM_CCER_CC2P;

	//Clear CC1NP & CC2NP bit for OC
	TIM3 -> CCER &= ~TIM_CCER_CC1NP;
	TIM3 -> CCER &= ~TIM_CCER_CC2NP;

	//Enable output compare for channel 1 & 2
	TIM3 -> CCER |= TIM_CCER_CC1E;
	TIM3 -> CCER |= TIM_CCER_CC2E;

	//Enable counter for TIM3
	TIM3 -> CR1 |= TIM_CR1_CEN;

}

void setDutyCycle() {

	//Set the duty cycle for TIM3
	TIM3 -> CCR1 = leftDuty*480;
	TIM3 -> CCR2 = rightDuty*480;
}
//Maze Mapping Functions--------------------------------------------------------------------------------------



//-----------------------------------------------------------------------------------------------------------

// Main function
void main(void) {

	//Initialization
	init_Ports();
	init_Timer();

    //Loop forever
    while(1)
    {
    	//set motors in forward direction, max duty cycle
		if((GPIOA->IDR & GPIO_IDR_0)==0)
		{
			GPIOB->ODR &= 0;

			for (int delay = 0; delay <Delay; delay++);

			GPIOB->ODR |= GPIO_ODR_13;
			GPIOB->ODR |= GPIO_ODR_12;

			rightDuty=MAX_DUTY;
			leftDuty=MAX_DUTY;

		}

		//set motors in reverse direction, max duty cycle
		else if((GPIOA->IDR & GPIO_IDR_1)==0)
		{
			GPIOB->ODR &= 0;

			for (int delay = 0; delay <Delay; delay++);

			GPIOB->ODR |= GPIO_ODR_11;
		    GPIOB->ODR |= GPIO_ODR_10;

		    rightDuty=MAX_DUTY;
		    leftDuty=MAX_DUTY;

		}
		//step left motor PWM down
		else if((GPIOA->IDR & GPIO_IDR_2)==0)
		{
			for (int delay = 0; delay <Delay; delay++);
			rightDuty-=increment;

		}
		//step right motor PWM down
		else if((GPIOA->IDR & GPIO_IDR_3)==0)
	    {
			for (int delay = 0; delay <Delay; delay++);
			rightDuty+=increment;

	    }

		setDutyCycle();
    }




}




