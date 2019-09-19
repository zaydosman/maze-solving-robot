//********************************************************************
//*             EEE3099S Line Following Robot Code                   *
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
#include <cxxabi.h>

#define	goButton					GPIO_IDR_0		//Go Button
#define statusLED					GPIO_ODR_6		//Status LED on pin 6 of ord being used
#define	lineRightPin				GPIO_IDR_4		//Right Line Sensor
#define	lineCenterPin				GPIO_IDR_5		//Center Line Sensor
#define	lineLeftPin					GPIO_IDR_6		//Left Line Sensor
#define wideLeftPin					GPIO_IDR_7		//left sensor
#define wideRightPin				GPIO_IDR_9		//right sensor
#define startDelay					1000000			//Delay after the Go Button is released
#define flashDelay					1500000			//Delay used when flashing the Status LE

//global variables

    uint8_t mode = 0;				//0=ERROR, 1=READY, 2=DRIVE, 3=Right Turn, 4=Left Turn, 5=STOP
    uint16_t leftDuty;
    uint16_t rightDuty;
    uint16_t MAX_DUTY = 50;
    uint16_t STEP_DOWN_1 = 20;
    int total_points;
    int direction;				//stores direction that robot is facing 1234-->ENWS
    int time_distance;			//stores time based distance in x or y direction
    int coordinate[2][30];      //stores time based x and y co-ordinates of points
    int point[30];				//stores integer number that identifies point
    int type[30];				//stores the type of point, equal to number of paths leading to point -1
    int explored[30];           //stores the number of paths already explored leading to point
    int unexplored[4]={0,0,0,0};//stores the available unexplored paths that may be taken


//Function Declarations
 //Initialisation functions
    void initPorts(void);
    void init_NVIC(void);
    void init_EXTI(void);
    void update_data(void);   //updates data arrays
    int calc_distance(int);	  //calculates distance traveled from previous point
    int sel_direction(void);  //selects direction to move next based on order of preference
    void change_direction(int);//change direction based on input selected direction
    void follow_line(void);   //set motors to follow line in direction robot is facing
    void set_start(void);     //sets data values for starting point
    void check_explored(void); //sets global array of available unexplored paths
    int check_paths(void);    //returns number of paths surrounding the current point - 1



 //Interrupt Handler Functions
    void Go_Button_EXTI0_1_IRQHandler(void);
    void Sensor_EXTI_IRQHandler(void); // choose ports numbers for 14 and 15

 //General robot functions
    void stop(void); // code to make robot pause either at end of maze or at intersection for processing
    void move(void); // keep moving forward when on blacklike and no intersection is found
    void direction(void); // register direction when turning
    void init_Ports(void);
    void setDirection(void);

//Hardware interfacing functions-------------------------------------------------------------------------------------
void init_Ports() {

    	//Enable Clock for Port A
    	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    	//Enable the clock for Port B
    	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    	//Set A10-A11 as outputs (DIRECTION)
    	GPIOA->MODER |= (GPIO_MODER_MODER10_0|
    					GPIO_MODER_MODER11_0);

    	//Set B0 (Go Button), B4-B7 and B9 (line Sensors) as logic inputs
    	GPIOB->MODER &= ~(GPIO_MODER_MODER0|
    					GPIO_MODER_MODER4|
    					GPIO_MODER_MODER5|
    					GPIO_MODER_MODER6|
    					GPIO_MODER_MODER7|
    					GPIO_MODER_MODER9);

    	//Set PB6 as an output (Status LED)
    	GPIOB->MODER |= GPIO_MODER_MODER6_0;

    	//Enable pull up resistor for PB0 (Go Button)
    	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR0_0;

}

//attach edges to pushbutton(s) and configure external interrupt
void init_EXTI(void){



    RCC-> APB2ENR |=RCC_APB2ENR_SYSCFGCOMPEN;//ENABLE CLOCK

    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PB; 	//Go Button
    SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI4_PB; 	//Front sensor
    SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI5_PB; 	//Centre 1
    SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI6_PB; 	//Centre 2
    SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI7_PB; 	//Wide left
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI9_PB; 	//Wide right



    //attach edges FTSR=falling ede RTSR=rising edge
    //Both edges attatched for flexibility (Black->White surface) and (White->Black)
    EXTI-> FTSR =EXTI_FTSR_TRO;
    EXTI-> FTSR =EXTI_FTSR_TR4;
    EXTI-> FTSR =EXTI_FTSR_TR5;
    EXTI-> FTSR =EXTI_FTSR_TR6;
    EXTI-> FTSR =EXTI_FTSR_TR7;
    EXTI-> FTSR =EXTI_FTSR_TR9;

    EXTI-> RTSR =EXTI_RTSR_TR0;
    EXTI-> RTSR =EXTI_RTSR_TR4;
    EXTI-> RTSR =EXTI_RTSR_TR5;
    EXTI-> RTSR =EXTI_RTSR_TR6;
    EXTI-> RTSR =EXTI_RTSR_TR7;
    EXTI-> RTSR =EXTI_RTSR_TR9;



    //Unmask interrupt for Go Button
    EXTI->IMR |= EXTI_IMR_MR0;




}

void init_Timer() {


	//Set B4-B5 as alternate function pins
	GPIOB->MODER |= (GPIO_MODER_MODER4_1|
					GPIO_MODER_MODER5_1);

	//Set B4-B5 alternate function as TIM3
	GPIOB->AFR[0] |= (1<<(4*4) | 1<<(5*4));

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

void setDirection() {
	//Set both motors to forward
	GPIOA->ODR &=~ (GPIO_ODR_10|GPIO_ODR_11) ;
}

void followLine() {

	//center is on
	if ((GPIOB->IDR & lineLeftPin) && !(GPIOB->IDR & lineRightPin)) {
		//only left sensor on
		leftDuty = STEP_DOWN_1;
	} else if (!(GPIOB->IDR & lineLeftPin) && (GPIOB->IDR & lineRightPin)){
		//only right sensor on
		rightDuty = STEP_DOWN_1;
	} else {
		//all are on, or only center
		leftDuty = MAX_DUTY;
		rightDuty = MAX_DUTY;
	}

	//Set the duty cycle for TIM3
	TIM3 -> CCR1 = leftDuty*480;
	TIM3 -> CCR2 = rightDuty*480;

}
//Maze Mapping Functions--------------------------------------------------------------------------------------
void set_start(void){

	point[0]=0;
	coordinate[0][0]=0;
	coordinate[1][0]=0;
	Type[0]=0;
	explored[0]=check_paths();
	direction=2;
	total_points=1;


}


int check_paths(){

	int paths=0;

	paths = wideRightPin+WideleftPin+lineCentrePin;

	return paths;


}

void check_explored(){
	//sets available paths
	if (GPIO->IDR &right_wide){
		unexplored[1]=1;
	}

	if (GPIO->IDR &left_wide){
		unexplored[1]=1;
	}

	if (GPIO->IDR &right_wide){
		unexplored[1]=1;
	}

	if (GPIO->IDR &right_wide){
		unexplored[1]=1;
	}

	//sets unexplored available paths



	//corresponds to compass directions, sets array accordingly

}

// returns direction in terms of absolute NWSE So if you want to turn left then it tells you left corresponds to either NSWE depending on the orientation of robot
int choose_direction(int chosen_Direction){
	int absolute[] ={1,2,3,4} ;//NSWE �FBLR
	int relative[][] = [  [1,2,3,4], [3,4,2,1], [2,1,4,3], [4,3,1,2] ];

	//uses global index variable for axis

	int temp = Array.indexOf(relative[index],chosen_Direction);

	int absoluteChoice= absolute[temp];

	return absoluteChoice;

	/*if chosen direction is indeed taken then index variable must me updated
	Increament for Right turn and Decrease for left turn
	*/
}

//-----------------------------------------------------------------------------------------------------------

// Main function
void main(void) {

	//Initialization
		init_Ports();
		init_Timer();
		init_NVIC();
		init_EXTI();

    //Turn on Status LED on pin 6 of ODR
    GPIOB->ODR |= statusLED;

    //Loop forever
    for(;;);



}



//Interrupt handler for go button which is connected to GPIO B

void Go_Button_EXTI0_IRQHandler(void){
    /*apply debouncing later
     *
     *
     *

     * */

    EXTI->PR |= EXTI_PR_PR0; // clear interrupt pending bit since button was pressed


    EXTI-> IMR| EXTI0_IMR_MR0; // unmasking interrupt bit for button

    //Unmask interrupts for sensors
    EXTI->IMR |=EXT_IMR_MR4; //front sensor
    EXTI->IMR |=EXT_IMR_MR5;//centre sensor1
    EXTI->IMR |=EXT_IMR_MR6;//centresensor 2
    EXTI->IMR |=EXT_IMR_MR7;//left wide sensir
    EXTI->IMR |=EXT_IMR_MR8;//right wide sensor

    mode=1; // now ready to move

}

//Interrupt handler for sensor detection of new point
void Sensor_EXTI_IRQHandler(void){

    EXTI->PR |= EXTI_PR_PR0;// removes interrupt pending state




}

