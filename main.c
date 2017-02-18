//Imported files
#include <stdio.h>
#include "stdlib.h"
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

/* I2C1 pin out*/
#define I2C1_GPIO_Port 	GPIOB
#define I2C1_SCL				GPIO_Pin_6
#define I2C1_SDA				GPIO_Pin_7

/* Constant definition */



/* Motor control pins definition */
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

/* Encoder pins definition*/
#define encoderL1_port	GPIOA
#define encoderL1_pin		GPIO_Pin_4

#define encoderL2_port	GPIOA
#define encoderL2_pin		GPIO_Pin_5

#define encoderR1_port	GPIOB
#define encoderR1_pin		GPIO_Pin_0

#define encoderR2_port	GPIOB
#define encoderR2_pin		GPIO_Pin_1

/* Initialization functions declaration */
void GPIO_initialize(void);
void TIM_initialize(void);
void NVIC_Configuration(void);
void RCC_initialize(void);
void Motor_PWM_initialize(void);
void Motor_ControlPin_initialize(void);
void Encoder_Pin_initialize(void);
void SPI_Pin_initialize(void);
static void SetDutyCycle_LeftMotor(uint8_t);
static void SetDutyCycle_RightMotor(uint8_t);


//Type Define
GPIO_InitTypeDef  GPIO_InitStructure;
TIM_TimeBaseInitTypeDef timerInitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
TIM_OCInitTypeDef outputChannelInit = {0,};	

//Variables
uint16_t CCR_reset = 45;
uint8_t RPM_flag = 1;
uint8_t PID_flag = 1;


int16_t encoderL_counter = 0;
uint8_t encoderL1_current_state = 0;
uint8_t encoderL2_current_state = 0;
uint8_t encoderL1_past_state = 0;
uint8_t encoderL2_past_state = 0;
int16_t motorL_RPM = 0;

int16_t encoderR_counter = 0;
uint8_t encoderR1_current_state = 0;
uint8_t encoderR2_current_state = 0;
uint8_t encoderR1_past_state = 0;
uint8_t encoderR2_past_state = 0;
int16_t motorR_RPM = 0;

int main(void) {
	
	RCC_initialize();
	GPIO_initialize();
	NVIC_Configuration();	
	TIM_initialize();	
	Motor_PWM_initialize();
	Motor_ControlPin_initialize();
	Encoder_Pin_initialize();
	SPI_Pin_initialize();
	
  // Start Timer 3 PWM output
	TIM_Cmd(TIM3, ENABLE);
	TIM_CtrlPWMOutputs(TIM3,ENABLE);	
	
  // Start Timer 2 interrupt
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);

	while(1) {
		
		/* Update left and right motor encoder counter */
		encoderL1_current_state = GPIO_ReadInputDataBit(encoderL1_port, encoderL1_pin);
		encoderL2_current_state = GPIO_ReadInputDataBit(encoderL2_port, encoderL2_pin);
		
		encoderR1_current_state = GPIO_ReadInputDataBit(encoderR1_port, encoderR1_pin);
		encoderR2_current_state = GPIO_ReadInputDataBit(encoderR2_port, encoderR2_pin);		
		
		// Determine turning direction on two phase encoder and add to counter
		if ((encoderL1_past_state == 0) && (encoderL2_past_state == 0) && (encoderL1_current_state == 1)){
			encoderL_counter++;
		} else if ((encoderL1_past_state == 0) && (encoderL2_past_state == 0) && (encoderL2_current_state == 1)){
			encoderL_counter--;		
		}
		
		if ((encoderR1_past_state == 0) && (encoderR2_past_state == 0) && (encoderR2_current_state == 1)){
			encoderR_counter++;		
		}	else if ((encoderL1_past_state == 0) && (encoderL2_past_state == 0) && (encoderL2_current_state == 1)){
			encoderR_counter--;
		}
		
		// Save current state value in past state variable
		encoderL1_past_state = encoderL1_current_state;
		encoderL2_past_state = encoderL2_current_state;
		
		encoderR1_past_state = encoderR1_current_state;
		encoderR2_past_state = encoderR2_current_state;
		
		
		/* Get tranceiver data */
		
		
		
		
			
		
		
		/* Run RPM update at 20Hz interval (50ms)
			 RPM_update_period >> loop period */
		if (RPM_flag == 1){
			
			// 100 = (20Hz)*(60s/min)/(12inc/rev) 
			motorL_RPM = encoderL_counter*100;
			motorR_RPM = encoderR_counter*100;
		
			// Reset the encoder counters
			encoderL_counter = 0;
			encoderR_counter = 0;
			RPM_flag = 0;
		}
		
				
		
		/* Run PWM PID output update at 20Hz interval (50ms)
			 PID_update_period >> loop period */
		if (PID_flag == 1){
		
		
		
			PID_flag = 0;
		}

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
	timerInitStructure.TIM_Prescaler = 7200;
  timerInitStructure.TIM_CounterMode = TIM_CounterMode_Down;
  timerInitStructure.TIM_Period = 500;
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
	
	//GPIO B
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
	
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


/* Initialize encoder pins */
void Encoder_Pin_initialize(void){
	
	GPIO_InitStructure.GPIO_Pin = encoderL1_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(encoderL1_port, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = encoderL2_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(encoderL2_port, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = encoderR1_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(encoderR1_port, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = encoderR2_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(encoderR2_port, &GPIO_InitStructure);

}

/* Initialize SPI communication pins*/
void SPI_Pin_initialize(void){

	GPIO_InitStructure.GPIO_Pin = SPI_SCK;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(SPI_GPIO_Port, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI_MOSI;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(SPI_GPIO_Port, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SPI_MISO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(SPI_GPIO_Port, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI_CSN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(SPI_GPIO_Port, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SPI_CE;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(SPI_GPIO_Port, &GPIO_InitStructure);
}


/* Interrupt Handling @ 100Hz 10ms interval */
void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		
		RPM_flag = 1;
		PID_flag = 1;		
    
  }
		
} 


