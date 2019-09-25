//********************************************************************
//*             EEE3099S Maze Robot Final     Code                   *
//*==================================================================*
//* WRITTEN BY: Tonderai Saidi and Zayd Osman     	                 *
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
#define TurnDelay 1000000
#define leftTurnDelay 10000000
#define rightTurnDelay 10000000
#define uTurnDelay 	   10000000
#define circleDelay 1000000
#define forwardDelay 780000


//global variables

    uint16_t leftDuty;
    uint16_t rightDuty;
    //uint16_t MAX_DUTY = 70;
    uint16_t MAX_DUTY_L = 75;
    uint16_t MAX_DUTY_R = 70;

    uint16_t STEP_RIGHT_UP_1 = 80;
    uint16_t STEP_LEFT_UP_1 = 90;
    uint16_t STEP_RIGHT_DOWN_1 =50 ;
    uint16_t STEP_LEFT_DOWN_1 = 50;

    int goPressed=0;
    int detected=0;
    int deadEnd=0;
    int deadEndDone=0;
    int circle=0;
    int index0 = 0;
    int turns=0;
    int nextDirection=0;
    int totalturns=0;
    int map[30];
    int optimisedMap[30];
    int flag;

    //optimisation cases
    int case1[]={3,4,1};
    int case2[]={3,4,2};
    int case3[]={3,4,3};
    int case4[]={2,4,3};
    int case5[]={2,4,2};
    int case6[]={1,4,3};




//Function Declarations
 //maze mapping functions





 //General robot functions
    void init_Ports(void);
    void init_Timer(void);
    void setDutyCycle(void);
    void followLine(void);
    void setDirection(void);//sets both motors forward
    void setStatusLED(void);
    //void init_NVIC(void);
    //void init_EXTI(void);
    void stop(void);
    void detectLR(void);
    void detectDeadEnd(void);
    void turnLeft(void);
    void turnRight(void);
    void uTurn(void);
    void indexShift(void);
    int direction(void);
    void changeDirection(void);
    void optimiser(void);

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


    	//Set B13-B12 as outputs for motor direction control, B2 and B11 for sensor status LEDs
    	GPIOB->MODER |= (GPIO_MODER_MODER15_0|    //motor driver 2
    			         GPIO_MODER_MODER14_0|	  //motor driver 3
    					 GPIO_MODER_MODER13_0|    //motor driver 1
    	    	         GPIO_MODER_MODER12_0|    //motor driver 4
    	    			 GPIO_MODER_MODER11_0|
    	    			 GPIO_MODER_MODER2_0);

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

/*void init_NVIC() {
	//Enable IRQs
	NVIC_EnableIRQ(EXTI4_15_IRQn);
}
void init_EXTI() {
	//Enable clock
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
	//Set EXTIs for all inputs
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI5_PB; 	//wide left sensor
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI6_PB; 	//wide right sensor
	//Set falling edge triggers
	EXTI->FTSR |= EXTI_FTSR_TR5;
	EXTI->FTSR |= EXTI_FTSR_TR6;
	//Set rising edge triggers
	EXTI->RTSR |= EXTI_RTSR_TR5;
	EXTI->RTSR |= EXTI_RTSR_TR6;
	//Note: Other interrupts will only be unmasked once the Go Button has been pressed
}
void EXTI4_15_IRQHandler() {
	//This IRQ handler deals with all sensor triggers
	//Clear the interrupt event
	EXTI->PR |= EXTI_PR_PR0;
	stop();
}
*/
//Maze Mapping Functions--------------------------------------------------------------------------------------

void followLine() {

	//only left sensor on
	if ((GPIOB->IDR & GPIO_IDR_3) && !(GPIOB->IDR & GPIO_IDR_4)) {

		leftDuty = STEP_LEFT_DOWN_1;
		rightDuty = STEP_RIGHT_UP_1;

	//only right sensor on
	} else if (!(GPIOB->IDR & GPIO_IDR_3) && (GPIOB->IDR & GPIO_IDR_4)){
		leftDuty = STEP_LEFT_UP_1;
		rightDuty = STEP_RIGHT_DOWN_1;
	} else {
		//both sensors on
		leftDuty = MAX_DUTY_L;
		rightDuty = MAX_DUTY_R;
	}



}

void setStatusLED() {

	if(GPIOB->IDR & GPIO_IDR_7){

		GPIOB->ODR |= GPIO_ODR_11;

	}

	else if(!(GPIOB->IDR & GPIO_IDR_7)){

		GPIOB->ODR &=~GPIO_ODR_11;
	}

	if(GPIOB->IDR & GPIO_IDR_4){

		GPIOB->ODR |= GPIO_ODR_2;

	}

	else if(!(GPIOB->IDR & GPIO_IDR_4)){

		GPIOB->ODR &=~GPIO_ODR_2;
	}

}

void stop() {
	//Stop both motors immediately
	TIM3 -> CCR1 = 0;
	TIM3 -> CCR2 = 0;
	leftDuty = 0;
	rightDuty = 0;


}

void detectLR(){

	if ((GPIOB->IDR & GPIO_IDR_5) || (GPIOB->IDR & GPIO_IDR_6)) {


			stop();
			detected=1;
			nextDirection=direction();
			for (int delay = 0; delay <250000; delay++);

			rightDuty=52;
			leftDuty=64;
			setDutyCycle();
			for (int delay = 0; delay <forwardDelay; delay++);
			stop();

	}
	else
	{
		//DONOTHING
	}


	//detectCircle();
	//changeDirection();


}

void changeDirection(){

	if(circle==1){

			stop();
			setStatusLED();

		}

		else if(nextDirection==1){

			turnRight();
			map[totalturns]=1;

		}

		else if(nextDirection==2){

			detected=0;


		}

		else if(nextDirection==3){

			turnLeft();

		}


}

void detectDeadEnd(){

	if (!(GPIOB->IDR & GPIO_IDR_5) && !(GPIOB->IDR & GPIO_IDR_6) && !(GPIOB->IDR & GPIO_IDR_7) &&(detected==0)) {

				stop();
				deadEnd=1;




			}

}

void detectCircle(){


	if((GPIOB->IDR & GPIO_IDR_5) && (GPIOB->IDR & GPIO_IDR_6) && (GPIOB->IDR & GPIO_IDR_7)){

		circle=1;

	}

}

void turnLeft(){


	GPIOB->ODR &= 0;
	GPIOB->ODR |= GPIO_ODR_15;
	GPIOB->ODR |= GPIO_ODR_12;

	leftDuty=50;
	rightDuty=50;
	setDutyCycle();


	while(!(GPIOB->IDR & GPIO_IDR_5)){

	}


	while(!(GPIOB->IDR & GPIO_IDR_7)){

	}
	detected=0;
    stop();
	GPIOB->ODR &= 0;
	setDirection();

}

void turnRight(){

	GPIOB->ODR &= 0;
	GPIOB->ODR |= GPIO_ODR_13;
	GPIOB->ODR |= GPIO_ODR_14;

	leftDuty=55;
	rightDuty=50;
	setDutyCycle();




	while(!(GPIOB->IDR & GPIO_IDR_6)){

	}


	while(!(GPIOB->IDR & GPIO_IDR_7)){

	}
	detected=0;
	stop();
	GPIOB->ODR &= 0;
	setDirection();


}

void uTurn(){

	GPIOB->ODR &= 0;
	GPIOB->ODR |= GPIO_ODR_15;
	GPIOB->ODR |= GPIO_ODR_12;

	leftDuty=50;
	rightDuty=50;
	setDutyCycle();

	while(!(GPIOB->IDR & GPIO_IDR_5)){

	}


	while(!(GPIOB->IDR & GPIO_IDR_7)){

	}
	stop();
	deadEnd=0;

	GPIOB->ODR &= 0;
	setDirection();




}

void indexShift(){

	//Select direction for RHR
	//using global turns variable
	//RFLB sequence
	//using directionIndex variable

	index0++;
	if(index0>2){
		index0 =0;
	}
	else if(index0 <0){
		index0=2;

	}

}

int direction(){

	//Method returns a integer value telling you the direction that the robot should move in

	int a[3] = {1,2,3};   //RFL
	int i=-1;
	int rev=0;
	int c=0;
	while(i==-1 && rev==0){
	switch(index0){

		 case 0:
				c++;
				  if((turns!=3) && (GPIOB->IDR & GPIO_IDR_6)){
				//move to the right
				i=0;
				turns++;
				indexShift();
				break;
				}
				  indexShift();
				break;

		case 1:
			c++;
			//move forward
			turns =0;
			if(GPIOB->IDR & GPIO_IDR_7){

				indexShift();
				i=1;
				break;
			}
			indexShift();
			 break;
		case 2:
			c++;
			// move left
			turns =0;
			if(GPIOB->IDR& GPIO_IDR_5){
				//pwm
				 indexShift();
				i=2;
			}
			 indexShift();
			break;

				}

	}



	return a[i];
}

/*
bool negatives(int arr[]){
	for(int i=0;i<3;i++){
		if(arr[i]<0){
			return true;
		}
	}
	return false;
}
bool match(int arr1[],int arr2[]){
	int c=0;
	for(int i=0;i<3;i++){
		if(arr1[i]==arr2[i]){
			c++;
		}
	}
	if(c==3){
		return true;
	}
	return false;
}
void optimiser(){
	int n = totalpoints-1;
	int temp[3]={-1,-1,-1};
	for(int i=0;i<totalturns;i+=3){
		for(int j=i;j<j+3;j++){
			temp[n]=map[j];
			n++;
		}
		if(matches(temp,case1)){
		}
		temp[0]=-1;
		temp[1]=-1;
		temp[2]=-1;
	}
}
*/
//-----------------------------------------------------------------------------------------------------------

// Main function
void main(void) {

	//Initialization
	init_Ports();
	init_Timer();
	setDirection();
	//init_NVIC();
	//init_EXTI();

	/*
	//unmask interrupt bits for sensors
	EXTI->IMR |= EXTI_IMR_MR5;	//Line sensor - Left
    EXTI->IMR |= EXTI_IMR_MR6;	//Line sensor - Center
    EXTI->IMR |= EXTI_IMR_MR7;	//Line sensor - Right
*/
    //Loop forever
    while(1){

    	setStatusLED();

    	if((GPIOA->IDR & GPIO_IDR_3)==0){

    		goPressed=1;
    		//delay for debounce
    		for (int delay = 0; delay <Delay; delay++);

    	}

    	else if (goPressed==1 && detected==0 && deadEnd==0){



    		followLine();
    		setDutyCycle();
    		detectLR();
    		detectDeadEnd();

    	}

    	else if (goPressed==1&&detected==1&&deadEnd==0){
    		//turnLeft();
    		turnRight();
    	}

    	else if(goPressed==1&&deadEnd==1&&detected==0){

    		uTurn();


    	}




    }



}





