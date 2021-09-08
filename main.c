// Ultrasonic Sensor Interface using Timer Input Capture
//Connections:
//VCC - 5V
//Trig - PA5
//Echo - PA0
//GND - GND

//Header Files
#include "stm32f4xx.h"
#define ARM_MATH_CM4

//User-Defined Function declarations
void GPIO_Init(void);
void TIM2_capture(void);
void NVIC_Init(void);
void TIM2_IRQHandler(void);
void TIM4_us_delay(int n);

//User-defined variables
uint32_t pw = 0; //pulse width
uint32_t lc = 0; //last captured counter value
uint32_t cc = 0; //current captured counter value
uint32_t sp = 0; //signal polarity
double time,dist;

void GPIO_Init(){
	//Enable clock for port A
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	//Configure PA0 in alternate function mode, and PA5 in output mode
	GPIOA->MODER |= (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER5_0);

	//Configure PA0 in AF1 (See page no.272, of the data sheet)
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL0_0;
}

void TIM2_capture(){
	//1.Enable the clock for Timer 2
	RCC->APB1ENR  |= 1;

	//2.Set CC1S[1:0] as 01 to map IC1 to TI1, and configure CC1 as input
	TIM2->CCMR1 |= 1;

	//3.Disable sampling, since we need every event captured
	TIM2->CCMR1 &= ~TIM_CCMR1_IC1F;

	//4. Set the polarity of the signal for the triggering capture
	//(CC1NP/CC1P is set as 11, for sensitivity to both rising and falling edge
	TIM2->CCER |= ( (1<<1) | (1<<3));

	//5. Disable the input pre-scaler to capture at each valid transition
	//Writing 00 to IC1PS bits
	TIM2->CCMR1 &= ~TIM_CCMR1_IC1PSC;

	//6.Enable the capture from the counter into Capture register
	TIM2->CCER |= TIM_CCER_CC1E;

	//7. Enable the interrupt for the timer
	TIM2->DIER |= TIM_DIER_CC1IE;

	//8. Enable the Counter
	TIM2->CR1 |= TIM_CR1_CEN;
}

void NVIC_Init(){
	NVIC_SetPriority(TIM2_IRQn, 0);
	NVIC->ISER[0] |= 1<<28; //Enable global interrupt for TIM2
}

void TIM2_IRQHandler(){
	if((TIM2->SR & TIM_SR_CC1IF) != 0){
		cc = TIM2->CCR1; //read the capture value
		sp = 1-sp;// toggle the polarity flag

		if (sp==0){ //Calculate only when signal is low
			pw = cc-lc; //Calculate the pulse-width
		}

		lc = cc; //Update the last captured value

		if((TIM2->SR & TIM_SR_UIF) != 0) { //Check if overflow has taken place
			TIM2->SR &= ~TIM_SR_UIF; // Clear UIF flag to prevent re-entering
		}
	}
}


void TIM4_us_delay(int n){
	RCC->APB1ENR |=(1<<2); //Start the clock for the timer peripheral
	TIM4->ARR = (int)(n/0.0625); // Total period of the timer
	TIM4->CNT = 0;
	TIM4->CR1 |= 1; //Start the Timer
	while(!(TIM4->SR & TIM_SR_UIF)){} //Polling the update interrupt flag
	TIM4->SR &= ~(0x0001); //Reset the update interrupt flag
}


int main(){
	GPIO_Init();
	TIM2_capture();
	NVIC_Init();
	while(1){
		//1. Sending 10us pulse to
		GPIOA->BSRR &= 0x00000000; //PA5 is low
		TIM4_us_delay(2);
		GPIOA->BSRR |= 0x00000020;//PA5 set to High
		TIM4_us_delay(10);// wait for 10us
		GPIOA->BSRR |= 0x00200000;// Make PA5 low again

		//2.Converting the gathered data into distance in cm
		if (pw>0){
			time = pw*(0.0625*0.000001);
			dist = ((time*340)/2)*100;
		}
	}
}
