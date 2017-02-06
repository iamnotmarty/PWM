//Imported files
#include <stdio.h>
#include <stdbool.h>
#include "stm32f10x.h"
#include "stm32f10x_conf.h"

/* STM32F103 SPI2 output pin definition */
#define SPI_GPIO_Port	GPIOB
#define SPI_CSN				GPIO_Pin_12
#define SPI_SCK 			GPIO_Pin_13
#define SPI_MISO 			GPIO_Pin_14
#define SPI_MOSI 			GPIO_Pin_15
#define SPI_CE 				GPIO_Pin_10


/* Motor control pins declaration */
#define motorL1_port	GPIOB
#define motorL1_pin		GPIO_Pin_0

#define motorL2_port	GPIOB
#define motorL2_pin		GPIO_Pin_1

#define motorLEN_port	GPIOA
#define motorLEN_pin	GPIO_Pin_6

#define motorR1_port	GPIOB
#define motorR1_pin		GPIO_Pin_8

#define motorR2_port	GPIOB
#define motorR2_pin		GPIO_Pin_9

#define motorREN_port	GPIOA
#define motorREN_pin	GPIO_Pin_7


/* Initialization functions declaration */
static void GPIO_initialize(void);
void TIM_initialize(void);
void NVIC_Configuration(void);
static void RCC_initialize(void);
void Motor_PWM_initialize(void);
static void Motor_ControlPin_initialize(void);
static void SetDutyCycle_LeftMotor(uint8_t);
static void SetDutyCycle_RightMotor(uint8_t);


//Type Define
GPIO_InitTypeDef  GPIO_InitStructure;
TIM_TimeBaseInitTypeDef timerInitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
TIM_OCInitTypeDef outputChannelInit = {0,};	

//Variables
uint16_t CCR_reset = 45;
uint16_t PrescalerValue = 0;


/*APB1 -> 36MHz
	APB2 -> 72MHz
	AHB  -> 72MHz*/

/* -----------------------------------------------------------------------
    TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles:
    The TIM3CLK frequency is set to SystemCoreClock (Hz), to get TIM3 counter
    clock at 24 MHz the Prescaler is computed as following:
     - Prescaler = (TIM3CLK / TIM3 counter clock) - 1

    SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
    and Connectivity line devices and to 24 MHz for Low-Density Value line and
    Medium-Density Value line devices
		
    The TIM3 is running at 36 KHz: TIM3 Frequency = TIM3 counter clock/(ARR + 1)
                                                  = 24 MHz / 666 = 36 KHz
																									
    TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
    TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
    TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
    TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 12.5%
  ----------------------------------------------------------------------- */


int main(void) {
	
	RCC_initialize();
	GPIO_initialize();
	NVIC_Configuration();	
	TIM_initialize();	
	Motor_PWM_initialize();
	Motor_ControlPin_initialize();
	
  // Start Timer 3 PWM output
	TIM_Cmd(TIM3, ENABLE);
	TIM_CtrlPWMOutputs(TIM3,ENABLE);	
	
  // Start Timer 2 interrupt
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);

	// LOOOOOOOOP
	while(1) {
		
	
  }		
}

/* Initialize MISC GPIO pins */
void GPIO_initialize(void){
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
}

/* Initialize timer */
void TIM_initialize(void){
		
	// Initialize Timer 2
	timerInitStructure.TIM_Prescaler = 14400;
  timerInitStructure.TIM_CounterMode = TIM_CounterMode_Down;
  timerInitStructure.TIM_Period = 10000;
  timerInitStructure.TIM_ClockDivision = 0;
  timerInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, &timerInitStructure);
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

	// Configure so that the interrupt flag is only set upon overflow
	TIM_UpdateRequestConfig(TIM2, TIM_UpdateSource_Regular);

	// Enable the TIM2 Update Interrupt type
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	
	//	Initialize Timer 3 -> PWM
	timerInitStructure.TIM_Prescaler = 7200;
  timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  timerInitStructure.TIM_Period = 100;
  timerInitStructure.TIM_ClockDivision = 0;
  timerInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM3, &timerInitStructure);
	

}

/* RCC clock initialize */
void RCC_initialize(void){
	
	//GPIO A
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
	
	//TIMER 2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
  //TIMER 4
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	//TIMER 4
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
}

/* Initialize nested vector interrupt control on timer 2*/
void NVIC_Configuration(void){
		
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;   
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;      
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
	
}

/* Initialize timer 3 PWM function for motor control */
void Motor_PWM_initialize(void){
		
    // Left Motor PWM Enable pins output channels initialize
	  outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputChannelInit.TIM_Pulse = CCR_reset;
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
 
    TIM_OC1Init(TIM3, &outputChannelInit);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
		TIM_ARRPreloadConfig(TIM3, ENABLE);
	
	  // Right Motor PWM Enable pins output channels initialize
		outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputChannelInit.TIM_Pulse = CCR_reset;
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
 
    TIM_OC2Init(TIM3, &outputChannelInit);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
		TIM_ARRPreloadConfig(TIM3, ENABLE);
}

/* Initialize left and right motor control pins */
void Motor_ControlPin_initialize(void){
	
	//Left pin 1
	GPIO_InitStructure.GPIO_Pin = motorL1_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(motorL1_port, &GPIO_InitStructure);
	
	//Left pin 2
	GPIO_InitStructure.GPIO_Pin = motorL2_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(motorL2_port, &GPIO_InitStructure);	
	
	//Left PWM pin
	GPIO_InitStructure.GPIO_Pin = motorLEN_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(motorLEN_port, &GPIO_InitStructure);
	
	// Right pin 1
	GPIO_InitStructure.GPIO_Pin = motorR1_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(motorR1_port, &GPIO_InitStructure);	
	
	// Right pin 2
	GPIO_InitStructure.GPIO_Pin = motorR2_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(motorR2_port, &GPIO_InitStructure);	
	
	// RIght PWM pin
	GPIO_InitStructure.GPIO_Pin = motorREN_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(motorREN_port, &GPIO_InitStructure);
}

/* Updates left motor PWM duty cycle */
void SetDutyCycle_LeftMotor(uint8_t percent){
	
	outputChannelInit.TIM_Pulse = percent;
	TIM_OC1Init(TIM3, &outputChannelInit);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
}

/* Updates right motor PWM duty cycle */
void SetDutyCycle_RightMotor(uint8_t percent){
	
	outputChannelInit.TIM_Pulse = percent;
	TIM_OC2Init(TIM3, &outputChannelInit);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
}


/* Interrupt Handling */
void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		
    
  }
		
} 


