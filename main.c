#include "stm32f4xx.h"
#include "pi.h"
#define ARM_MATH_CM4
#define CPR 1024

uint16_t ticks;

// PI Gain Values Kp = 2.3169, Ki = 79.9409; Sampling Time iteration_time = 20ms;
float kp = 2.3169;
float ki = 79.9409;
float iteration_time = 20e-3;

float speedRad = 0.0; // Real speed in rad/s
float speedRpm = 0.0; // Real speed in rpm
float pos=0.0;
float ref_speedRad = 2.0; // Setpoint speed in rad/s
float duty = 0.0; // initial duty
PI myPI;

void msDelay(int);

void msDelay(int msTime)
{
	for(int i = 0; i<msTime*1333; i++);
}

/Enabling Clocks/
void clocksEnable(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;     //GPIO Port E for PE9, PE11
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;      //TIM1
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;      //TIM2
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
}


/Configuring GPIO for input capture/
void configGPIO(void)
{
	GPIOE->MODER |= ((2<<18)|(2<<22));      //AF for PE9, PE11
	GPIOE->AFR[1] |= ((1<<4)|(1<<12));      //AF1 for TIM1_CH1, TIM1_CH2

	GPIOD->MODER |= (2<<24);				//For PWM on PD12
	GPIOD->AFR[1] |= (2<<16);
}


/Configuring TIM1 in Encoder Interface Mode/
void configTIM1(void)
{
	TIM1->PSC = 0x0000;                //Prescalar = 0
	TIM1->ARR = 0xFFFF;                //Period = 65536
	//TIM1->CR1 &= ~(1<<4);            //Upcounting Mode

	TIM1->SMCR |= (3<<0);	//Counting on both TI1 and TI2 edges i.e CH1 PE9 and CH2 PE11

	/*
	 * IC1F=IC2F = 0b0000, IC1PSC=IC2PSC = 0b00, CC1S=CC2S = 0b01
	 * TIM1->CCMR1 = 0b0000000100000001;
	 */

	TIM1->CCMR1 |= ((0x1<<0)|(0x1<<8));	    //CC1 and CC2 are input. CC1 = TI1, CC2 = TI2
	TIM1->CCMR1 &= ~((0x3<<2)|(0x3<<10));	//Prescalars are 0 for IC1PSC and IC2PSC
	TIM1->CCMR1 &= ~((0xF<<4)|(0xF<<12));	//Filters are 0 for IC1F and IC2F
	TIM1->CCER |= ((0x1<<0)|(0x1<<4));		//CC1E AND CC2E ARE ENABLED
}


//Configuring TIM2 for an interrupt every 0.02s
void configTIM2(void)
{
	TIM2->CR1 &= ~(0x0010);    					//Set the mode to Count up
	TIM2->PSC = 1600-1;						    //Set the Prescalar
	TIM2->ARR = 200-1; 						    //Set period (Auto reload) to 1000
	TIM2->SR &= ~(0x1<<0);						//Clear Update interrupt flag
}

void configTIM4(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	TIM4->CR1 &= ~(0x0010);    		//Set the mode to Count up
	TIM4->PSC = 1-1;				//Set the Prescalar
	TIM4->ARR = 320-1; 				//Set period (Auto reload)
	TIM4->CCMR1 |=(6UL<<4); 		//PWM mode
	TIM4->CCER |=(0x1<<0);
	TIM4->CCR1= 0;					//Duty Ratio
}


int main(void)
{
	SCB->CPACR|=(0xF<<20);
	clocksEnable();
	configGPIO();
	configTIM1();
	configTIM2();
	configTIM4();

	NVIC->ISER[0] |= 1<<28;
	TIM2->DIER |=(1<<0);

	TIM1->CR1 |= (1<<0);
	TIM2->CR1 |= (1<<0);
	TIM4->CR1 |= (1<<0);

	setPI(kp, ki, iteration_time, &myPI);
	//msDelay(5000);			//time to open ST Studio

	while(1)
	{

	}
}

void TIM2_IRQHandler()
{
	  ticks = TIM1->CNT;

	  speedRad = (float) (ticks/iteration_time)*(2*3.14)/(4*CPR); //rad/s for clockwise
	  speedRpm = 60*speedRad/(2*3.14);
	  pos += speedRpm*(ticks/iteration_time);

	  TIM1->CNT=0;

	  duty = outputPI(ref_speedRad, speedRad, &myPI); // pi output
	  if(duty>= 320)
		  duty = 320;
	  TIM4->CCR1 = (uint16_t) (duty - 1); // update duty
	  TIM2->SR &= ~(0x1<<0);
}
