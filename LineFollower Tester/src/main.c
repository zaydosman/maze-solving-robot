//********************************************************************
//*             EEE3099S Line Follower Tester Code                   *
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




#define Delay  1000000			     //Delay after the Go Button is released


//global variables

    uint16_t leftDuty;
    uint16_t rightDuty;
    uint16_t MAX_DUTY = 80;
    uint16_t STEP_DOWN_1 = 60;
    int goPressed=0;


//Function Declarations
 //maze mapping functions


 //General robot functions
    void init_Ports(void);
    void init_Timer(void);
    void setDutyCycle(void);
    void followLine(void);
    void setDirection(void);
    void setStatusLED(void);


//Hardware interfacing functions-------------------------------------------------------------------------------------
void init_Ports() {

    	//Enable Clock for Port A
    	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    	//Enable the clock for Port B
    	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    	//set A3 Go button as logic input
    	GPIOA->MODER &= ~(GPIO_MODER_MODER3);

    	//Set B3-7 (line Sensors) as logic inputs
    	GPIOB->MODER &= ~(GPIO_MODER_MODER3|
				GPIO_MODER_MODER4|
				GPIO_MODER_MODER5|
				GPIO_MODER_MODER6|
				GPIO_MODER_MODER7);

    	//set A6 and A7 to alternate function mode for PWM
    	GPIOA->MODER |= (GPIO_MODER_MODER6_1|
    	    	    	 GPIO_MODER_MODER7_1);


    	//Set B13-B12 as outputs for motor direction control, B10-B11 for sensor status LEDs
    	GPIOB->MODER |= (GPIO_MODER_MODER13_0|
    	    	         GPIO_MODER_MODER12_0|
    	    			 GPIO_MODER_MODER11_0|
    	    			 GPIO_MODER_MODER10_0);

    	//set pull up resistor for go button
    	GPIOA->PUPDR |= 0b00100100000000000000000001000000;

    	//set pull down resistors for sensor input pins
    	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR3_1;
    	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR4_1;
    	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR5_1;
    	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_1;
    	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR7_1;



    	GPIOB->ODR &= 0;


}

void init_Timer() {


	//Set A6-A7 alternate function as TIM3
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

void setDirection() {
	//Set both motors to forward
	GPIOB->ODR |= GPIO_ODR_13;
	GPIOB->ODR |= GPIO_ODR_12;
}
//Maze Mapping Functions--------------------------------------------------------------------------------------

void followLine() {

	//only left sensor on
	if ((GPIOB->IDR & GPIO_IDR_3) && !(GPIOB->IDR & GPIO_IDR_4)) {

		leftDuty = STEP_DOWN_1;
	//only right sensor on
	} else if (!(GPIOB->IDR & GPIO_IDR_3) && (GPIOB->IDR & GPIO_IDR_4)){

		rightDuty = STEP_DOWN_1;
	} else {
		//both sensors on
		leftDuty = MAX_DUTY;
		rightDuty = MAX_DUTY;
	}



}


void setStatusLED() {

	if(GPIOB->IDR & GPIO_IDR_3){

		GPIOB->ODR |= GPIO_ODR_10;

	}

	else if(!(GPIOB->IDR & GPIO_IDR_3)){

		GPIOB->ODR &=~GPIO_ODR_10;
	}

	if(GPIOB->IDR & GPIO_IDR_4){

		GPIOB->ODR |= GPIO_ODR_11;

	}

	else if(!(GPIOB->IDR & GPIO_IDR_4)){

		GPIOB->ODR &=~GPIO_ODR_11;
	}

}

//-----------------------------------------------------------------------------------------------------------

// Main function
void main(void) {

	//Initialization
	init_Ports();
	init_Timer();
	setDirection();


    //Loop forever
    while(1){

    	if((GPIOA->IDR & GPIO_IDR_3)==0){

    		goPressed=1;
    		//delay for debounce
    		for (int delay = 0; delay <Delay; delay++);

    	}

    	else if (goPressed==1){

    		followLine();
    		setDutyCycle();
    		setStatusLED();
    	}


    }



}



