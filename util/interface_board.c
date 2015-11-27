/*
 * File: interface_board.c
 *
 * Model version      : 
 * C/C++ source code written by  :  8-30-2015, Alireza Ramezani, Champaign, IL
 *
 * Embedded hardware selection: STMicroelectronics->STM32F4xx 32-bit Cortex-M4
 * Code generation objectives:
 *    1. 
 *    2. 
 *    3. 
 *    4. 
 *    5. 
 *    6. 
 *    7. 
 *    8. 
 * Validation result: 
 *
 *
 *
 * ******************************************************************************
 * * attention
 * *
 * * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * *
 * ******************************************************************************
 */
/* includes */
#include "interface_board.h"

/* Global Variable Definition for TIM2 Configuration */
TIM_TimeBaseInitTypeDef TIM2_TimeBaseStructure;
TIM_OCInitTypeDef TIM2_OCInitStructure;

/* Global Variable Definition for TIM3 Configuration */
TIM_TimeBaseInitTypeDef TIM3_TimeBaseStructure;
TIM_OCInitTypeDef TIM3_OCInitStructure;

/* Global Variable Definition for TIM9 Configuration */
TIM_TimeBaseInitTypeDef TIM9_TimeBaseStructure;
TIM_OCInitTypeDef TIM9_OCInitStructure;

/*Global variable for APB1/APB2 prescaler from RCC_Configuration.c file */
extern uint32_t RCC_APB1_Prescaler;
extern uint32_t RCC_APB2_Prescaler;


/* extern vars */
int16_T _pwm_pa3 = 0;
int16_T _pwm_pb1 = 0;
int16_T _pwm_pe5 = 0;

/*******************************************************************************
 * Function Name  : INTERFACE_BOARD_initialize
 * Description    : initialize the interface board related peripherials
 * Input          : -
 *******************************************************************************/
void INTERFACE_BOARD_initialize(void)
{
	/* initialize timers(3,4) to run DRV835*/
	INTERFACE_BOARD_TIM2_Configuration();
	INTERFACE_BOARD_TIM3_Configuration();
	
	/* initialize timers(9) TMS320*/
	INTERFACE_BOARD_TIM9_Configuration();
	
}

/*******************************************************************************
 * Function Name  : INTERFACE_BOARD_TIM2_Configuration
 * Description    : TIM2 CH(1,2,3,4) PWM output or Input_Capture Configuration
 * Input          : -
 *******************************************************************************/
void INTERFACE_BOARD_TIM2_Configuration(void)
{
   /* Time base configuration */
	switch (RCC_APB1_Prescaler) {
	case RCC_HCLK_Div4:
		TIM2_TimeBaseStructure.TIM_Prescaler = (uint16_t)((SystemCoreClock / 2) /
		  TIMER2_COUNTER_CLOCK_FREQ) - 1;
		break;

	case RCC_HCLK_Div8:
		TIM2_TimeBaseStructure.TIM_Prescaler = (uint16_t)((SystemCoreClock / 4) /
		  TIMER2_COUNTER_CLOCK_FREQ) - 1;
		break;

	case RCC_HCLK_Div16:
		TIM2_TimeBaseStructure.TIM_Prescaler = (uint16_t)((SystemCoreClock / 8) /
		  TIMER2_COUNTER_CLOCK_FREQ) - 1;
		break;

	default:
		TIM2_TimeBaseStructure.TIM_Prescaler = (uint16_t)(SystemCoreClock / TIMER2_COUNTER_CLOCK_FREQ)
		  - 1;
		break;
	}

	TIM2_TimeBaseStructure.TIM_Period = TIMER2_COUNTER_CLOCK_FREQ / TIMER2_PWM_FREQ - 1;
	TIM2_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM2_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM2_TimeBaseStructure);

	  /* PWM output mode configuration: Channel CH1*/
	TIM2_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM2_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM2_OCInitStructure.TIM_Pulse = 50 * TIM2_TimeBaseStructure.TIM_Period / 100;
	TIM2_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM2, &TIM2_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

	  /* PWM output mode configuration: Channel CH2*/
	TIM2_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM2_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM2_OCInitStructure.TIM_Pulse = 50 * TIM2_TimeBaseStructure.TIM_Period / 100;
	TIM2_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC2Init(TIM2, &TIM2_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

	  /* PWM output mode configuration: Channel CH3*/
	TIM2_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM2_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM2_OCInitStructure.TIM_Pulse = 50 * TIM2_TimeBaseStructure.TIM_Period / 100;
	TIM2_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC3Init(TIM2, &TIM2_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	 /* PWM output mode configuration: Channel CH4*/
	TIM2_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM2_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM2_OCInitStructure.TIM_Pulse = 50 * TIM2_TimeBaseStructure.TIM_Period / 100;
	TIM2_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC4Init(TIM2, &TIM2_OCInitStructure);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM2, ENABLE);

	  /* TIM2 enable counter */
	TIM_Cmd(TIM2, ENABLE);
}

/*******************************************************************************
 * Function Name  : INTERFACE_BOARD_TIM3_Configuration
 * Description    : TIM3 CH(1,2,3,4) PWM output or Input_Capture Configuration
 * Input          : -
 *******************************************************************************/
void INTERFACE_BOARD_TIM3_Configuration(void)
{
 /* Time base configuration */
	switch (RCC_APB1_Prescaler) {
	case RCC_HCLK_Div4:
		TIM3_TimeBaseStructure.TIM_Prescaler = (uint16_t)((SystemCoreClock / 2) /
		  TIMER3_COUNTER_CLOCK_FREQ) - 1;
		break;

	case RCC_HCLK_Div8:
		TIM3_TimeBaseStructure.TIM_Prescaler = (uint16_t)((SystemCoreClock / 4) /
		  TIMER3_COUNTER_CLOCK_FREQ) - 1;
		break;

	case RCC_HCLK_Div16:
		TIM3_TimeBaseStructure.TIM_Prescaler = (uint16_t)((SystemCoreClock / 8) /
		  TIMER3_COUNTER_CLOCK_FREQ) - 1;
		break;

	default:
		TIM3_TimeBaseStructure.TIM_Prescaler = (uint16_t)(SystemCoreClock / TIMER3_COUNTER_CLOCK_FREQ)
		  - 1;
		break;
	}

	TIM3_TimeBaseStructure.TIM_Period = TIMER3_COUNTER_CLOCK_FREQ / TIMER3_PWM_FREQ - 1;
	TIM3_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM3_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM3_TimeBaseStructure);

	  /* PWM output mode configuration: Channel CH1*/
	TIM3_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM3_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM3_OCInitStructure.TIM_Pulse = 50 * TIM3_TimeBaseStructure.TIM_Period / 100;
	TIM3_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM3, &TIM3_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	  /* PWM output mode configuration: Channel CH2*/
	TIM3_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM3_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM3_OCInitStructure.TIM_Pulse = 50 * TIM3_TimeBaseStructure.TIM_Period / 100;
	TIM3_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC2Init(TIM3, &TIM3_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

	  /* PWM output mode configuration: Channel CH3*/
	TIM3_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM3_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM3_OCInitStructure.TIM_Pulse = 50 * TIM3_TimeBaseStructure.TIM_Period / 100;
	TIM3_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC3Init(TIM3, &TIM3_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	 /* PWM output mode configuration: Channel CH3*/
	TIM3_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM3_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM3_OCInitStructure.TIM_Pulse = 50 * TIM3_TimeBaseStructure.TIM_Period / 100;
	TIM3_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC4Init(TIM3, &TIM3_OCInitStructure);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);

	  /* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);
}

/*******************************************************************************
 * Function Name  : INTERFACE_BOARD_TIM9_Configuration
 * Description    : TIM3 CH(1,2,3,4) PWM output or Input_Capture Configuration
 * Input          : -
 *******************************************************************************/
void INTERFACE_BOARD_TIM9_Configuration(void)
{
 /* Time base configuration */
	switch (RCC_APB2_Prescaler) {
	case RCC_HCLK_Div4:
		TIM9_TimeBaseStructure.TIM_Prescaler = (uint16_t)((SystemCoreClock / 2) /
		  TIMER9_COUNTER_CLOCK_FREQ) - 1;
		break;

	case RCC_HCLK_Div8:
		TIM9_TimeBaseStructure.TIM_Prescaler = (uint16_t)((SystemCoreClock / 4) /
		  TIMER9_COUNTER_CLOCK_FREQ) - 1;
		break;

	case RCC_HCLK_Div16:
		TIM9_TimeBaseStructure.TIM_Prescaler = (uint16_t)((SystemCoreClock / 8) /
		  TIMER9_COUNTER_CLOCK_FREQ) - 1;
		break;

	default:
		TIM9_TimeBaseStructure.TIM_Prescaler = (uint16_t)(SystemCoreClock / TIMER9_COUNTER_CLOCK_FREQ)
		  - 1;
		break;
	}

	TIM9_TimeBaseStructure.TIM_Period = TIMER9_COUNTER_CLOCK_FREQ / TIMER9_PWM_FREQ - 1;
	TIM9_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM9_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM9, &TIM9_TimeBaseStructure);

	  /* PWM output mode configuration: Channel CH1*/
	TIM9_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM9_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM9_OCInitStructure.TIM_Pulse = 50 * TIM9_TimeBaseStructure.TIM_Period / 100;
	TIM9_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM9, &TIM9_OCInitStructure);
	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);

	  /* PWM output mode configuration: Channel CH2*/
	TIM9_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM9_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM9_OCInitStructure.TIM_Pulse = 50 * TIM9_TimeBaseStructure.TIM_Period / 100;
	TIM9_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC2Init(TIM9, &TIM9_OCInitStructure);
	TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);

	  /* PWM output mode configuration: Channel CH3*/
	TIM9_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM9_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM9_OCInitStructure.TIM_Pulse = 50 * TIM9_TimeBaseStructure.TIM_Period / 100;
	TIM9_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC3Init(TIM9, &TIM9_OCInitStructure);
	TIM_OC3PreloadConfig(TIM9, TIM_OCPreload_Enable);
	
	 /* PWM output mode configuration: Channel CH4*/
	TIM9_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM9_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM9_OCInitStructure.TIM_Pulse = 50 * TIM9_TimeBaseStructure.TIM_Period / 100;
	TIM9_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC4Init(TIM9, &TIM9_OCInitStructure);
	TIM_OC4PreloadConfig(TIM9, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM9, ENABLE);

	  /* TIM9 enable counter */
	TIM_Cmd(TIM9, ENABLE);
}

/*******************************************************************************
 * Function Name  : INTERFACE_BOARD_pwmGenerator
 * Description    : TIM3,TIM4 CH4 PWM output generator
 * Input          : -
 *******************************************************************************/
void INTERFACE_BOARD_pwmGenerator(void)
{
	 /* Timer frequency is an input port */
	TIM2->ARR = TIMER2_COUNTER_CLOCK_FREQ / TIMER2_PWM_FREQ - 1;
	if (_pwm_pa3 < 0) {
	  /* Disable output and complementary output */
	  //            TIM2->BDTR &= 0x7FFF;  //MOE = 0
		TIM2->BDTR |= 0x8000;              //MOE = 1
		TIM2->BDTR &= 0xF7FF;              //OSSR = 0

	}
	else {
	  // Enable output and complementary output and update dutyCycle
		TIM2->BDTR |= 0x8000;              //MOE = 1
		TIM2->CCER |= 0x5;                 //CC4NE = 1 and CC4E = 1

		    // Channel1 duty cycle is an input port
		TIM2->CCR4 = _pwm_pa3 * TIM2->ARR / 100;
	}
	
	
	 /* Timer frequency is an input port */
	TIM3->ARR = TIMER3_COUNTER_CLOCK_FREQ / TIMER3_PWM_FREQ - 1;
	if (_pwm_pb1 < 0) {
	  /* Disable output and complementary output */
	  //            TIM2->BDTR &= 0x7FFF;  //MOE = 0
		TIM3->BDTR |= 0x8000;              //MOE = 1
		TIM3->BDTR &= 0xF7FF;              //OSSR = 0

	}
	else {
	  // Enable output and complementary output and update dutyCycle
		TIM3->BDTR |= 0x8000;              //MOE = 1
		TIM3->CCER |= 0x5;                 //CC4NE = 1 and CC4E = 1

		    // Channel1 duty cycle is an input port
		TIM3->CCR4 = _pwm_pb1 * TIM3->ARR / 100;
	}
	
	/* Timer frequency is an input port */
	TIM9->ARR = TIMER9_COUNTER_CLOCK_FREQ / TIMER9_PWM_FREQ - 1;
	if (_pwm_pe5 < 0) {
	  /* Disable output and complementary output */
	  //            TIM2->BDTR &= 0x7FFF;  //MOE = 0
		TIM9->BDTR |= 0x8000;              //MOE = 1
		TIM9->BDTR &= 0xF7FF;              //OSSR = 0

	}
	else {
	  // Enable output and complementary output and update dutyCycle
		TIM9->BDTR |= 0x8000;              //MOE = 1
		TIM9->CCER |= 0x5;                 //CC4NE = 1 and CC4E = 1

		    // Channel1 duty cycle is an input port
		TIM9->CCR1 = _pwm_pe5 * TIM9->ARR / 100;
	}
	
	/* Timer frequency is an input port */
	TIM9->ARR = TIMER9_COUNTER_CLOCK_FREQ / TIMER9_PWM_FREQ - 1;
	if (_pwm_pe6 < 0) {
	  /* Disable output and complementary output */
	  //            TIM2->BDTR &= 0x7FFF;  //MOE = 0
		TIM9->BDTR |= 0x8000;              //MOE = 1
		TIM9->BDTR &= 0xF7FF;              //OSSR = 0

	}
	else {
	  // Enable output and complementary output and update dutyCycle
		TIM9->BDTR |= 0x8000;              //MOE = 1
		TIM9->CCER |= 0x5;                 //CC4NE = 1 and CC4E = 1

		    // Channel1 duty cycle is an input port
		TIM9->CCR2 = _pwm_pe6 * TIM9->ARR / 100;
	}
}

