/*
 * File: daq.c
 *
 * Code generated for Simulink model :daq.
 *
 * Model version      : 1.40
 * Simulink Coder version    : 8.6 (R2014a) 27-Dec-2013
 * TLC version       : 8.6 (Jan 30 2014)
 * C/C++ source code generated on  : Tue Jun 23 20:05:13 2015
 *
 * Target selection: stm32F4xx.tlc
 * Embedded hardware selection: STMicroelectronics->STM32F4xx 32-bit Cortex-M4
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. Debugging
 * Validation result: Not run
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

#include "daq.h"
#include "daq_private.h"
#define USART1_RX_BUFF_SIZE            500
#define USART1_RX_STRING_SIZE          500
#define USART3_RX_BUFF_SIZE            1000
#define USART3_RX_STRING_SIZE          100
#define USART6_RX_BUFF_SIZE            1000
#define USART6_RX_STRING_SIZE          100
#define USART2_RX_BUFF_SIZE            18
#define USART2_RX_STRING_SIZE          18

/* Global Variable Definition for TIM4 Configuration */
TIM_TimeBaseInitTypeDef TIM4_TimeBaseStructure;
TIM_OCInitTypeDef TIM4_OCInitStructure;

/*Global variable for APB1/APB2 prescaler from RCC_Configuration.c file */
extern uint32_t RCC_APB1_Prescaler;
extern uint32_t RCC_APB2_Prescaler;

/* Global Variable Definition for TIM5 Configuration */
TIM_TimeBaseInitTypeDef TIM5_TimeBaseStructure;
TIM_OCInitTypeDef TIM5_OCInitStructure;

/*Global variable for APB1/APB2 prescaler from RCC_Configuration.c file */
extern uint32_t RCC_APB1_Prescaler;
extern uint32_t RCC_APB2_Prescaler;

/* Global Variable Definition for ADC1 Configuration */
ADC_InitTypeDef ADC1_InitStructure;

#ifdef ADC1_NB_CH
#undef ADC1_NB_CH
#endif

#define ADC1_NB_CH                     (2 + 0)
#ifndef ADC2_NB_CH
#define ADC2_NB_CH                     0
#endif

#ifndef ADC3_NB_CH
#define ADC3_NB_CH                     0
#endif

/* Global Variable Definition for ADC Configuration */
ADC_CommonInitTypeDef ADC_CommonInitStructure;
DMA_InitTypeDef ADC_DMA2_InitStructure;

/* ADC mode definition is used to know DMA channel to use */
#define ADC_COMMON_MODE                ADC_Mode_Independent

/* DMA access mode definition is used to know data size */
#define ADC_DMA2_ACCESS_MODE           ADC_DMAAccessMode_Disabled

/* ADC1/2/3 DR address and common CCR address */
#define ADC1_DR_ADDRESS                ((uint32_t)0x4001204C)
#define ADC2_DR_ADDRESS                ((uint32_t)0x4001214C)
#define ADC3_DR_ADDRESS                ((uint32_t)0x4001224C)
#define ADC_CCR_ADDRESS                ((uint32_t)0x40012308)

/* Global Variable Definition for GPIOA Configuration */
GPIO_InitTypeDef GPIOA_InitStructure;

/* Global Variable Definition for GPIOB Configuration */
GPIO_InitTypeDef GPIOB_InitStructure;

/* Global Variable Definition for GPIOC Configuration */
GPIO_InitTypeDef GPIOC_InitStructure;

/* Global Variable Definition for GPIOD Configuration */
GPIO_InitTypeDef GPIOD_InitStructure;

/* Global Variable Definition for GPIOE Configuration */
GPIO_InitTypeDef GPIOE_InitStructure;

/* Global Variable Definition for GPIOF Configuration */
GPIO_InitTypeDef GPIOF_InitStructure;

/* Global Variable Definition for GPIOG Configuration */
GPIO_InitTypeDef GPIOG_InitStructure;

/* Global Variable Definition for GPIOH Configuration */
GPIO_InitTypeDef GPIOH_InitStructure;

/* Global Variable Definition for GPIOI Configuration */
GPIO_InitTypeDef GPIOI_InitStructure;

#define USART6_CONFIG_NAME             USART6
#define USART6_CONFIG_PIN_RX           GPIO_Pin_7
#define USART6_CONFIG_PIN_TX           GPIO_Pin_6
#define USART6_CONFIG_PORT_TX          GPIOC
#define USART6_CONFIG_PORT_RX          GPIOC

/* Enable usart peripheral clocks */
#define RCC_USART6_ClockCmd()          RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE)

/* Enable GPIO clocks */
#define USART6_RCC_GPIORx_ClockCmd()   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE)
#define USART6_RCC_GPIOTx_ClockCmd()

/* Configure AFIO Alternate fonction */
#define USART6_GPIOTx_PinAFConfig()    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6)
#define USART6_GPIORx_PinAFConfig()    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6)

/* Global Variable Definition for GPIO Configuration */
static GPIO_InitTypeDef GPIO_InitStructure;

/* Global Variable Definition for UART Configuration */
static USART_InitTypeDef USART6_InitStructure;
static USART_ClockInitTypeDef USART6_ClockInitStructure;

/* USART6 interrupt receive */
#define USART6_IT_RCV                  1

/* USART6 interrupt send */
#define USART6_IT_SEND                 1

NVIC_InitTypeDef NVIC_USART6_InitStructure;
static volatile unsigned int USART6_NbrOfDataInBuff = 0;
static char USART6_RxBuffer[USART6_RX_BUFF_SIZE];
static char USART6_RxOutputDataBuffer[USART6_RX_STRING_SIZE] = { 0 };

static char* USART6_RxOutputDataBufferPt = USART6_RxOutputDataBuffer;
static char* USART6_ReadPt = USART6_RxBuffer;
static char* USART6_WritePt = USART6_RxBuffer;
static char* USART6_BufferSendPt;
static uint16_t USART6_NbCharToSend = 0;
static uint16_t USART6_NbCharSent = 0;

#define USART2_CONFIG_NAME             USART2
#define USART2_CONFIG_PIN_RX           GPIO_Pin_3
#define USART2_CONFIG_PIN_TX           GPIO_Pin_2
#define USART2_CONFIG_PORT_TX          GPIOA
#define USART2_CONFIG_PORT_RX          GPIOA

/* Enable usart peripheral clocks */
#define RCC_USART2_ClockCmd()          RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE)

/* Enable GPIO clocks */
#define USART2_RCC_GPIORx_ClockCmd()   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE)
#define USART2_RCC_GPIOTx_ClockCmd()

/* Configure AFIO Alternate fonction */
#define USART2_GPIOTx_PinAFConfig()    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2)
#define USART2_GPIORx_PinAFConfig()    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2)

/* Global Variable Definition for GPIO Configuration */
static GPIO_InitTypeDef GPIO_InitStructure;

/* Global Variable Definition for UART Configuration */
static USART_InitTypeDef USART2_InitStructure;
static USART_ClockInitTypeDef USART2_ClockInitStructure;

/* USART2 interrupt receive */
#define USART2_IT_RCV                  1

/* USART2 interrupt send */
#define USART2_IT_SEND                 1

NVIC_InitTypeDef NVIC_USART2_InitStructure;
static volatile unsigned int USART2_NbrOfDataInBuff = 0;
static char USART2_RxBuffer[USART2_RX_BUFF_SIZE];
static char USART2_RxOutputDataBuffer[USART2_RX_STRING_SIZE] = { 0 };

static char* USART2_RxOutputDataBufferPt = USART2_RxOutputDataBuffer;
static char* USART2_ReadPt = USART2_RxBuffer;
static char* USART2_WritePt = USART2_RxBuffer;
static char* USART2_BufferSendPt;
static uint16_t USART2_NbCharToSend = 0;
static uint16_t USART2_NbCharSent = 0;

#define USART1_CONFIG_NAME             USART1
#define USART1_CONFIG_PIN_RX           GPIO_Pin_10
#define USART1_CONFIG_PIN_TX           GPIO_Pin_9
#define USART1_CONFIG_PORT_TX          GPIOA
#define USART1_CONFIG_PORT_RX          GPIOA

/* Enable usart peripheral clocks */
#define RCC_USART1_ClockCmd()          RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE)

/* Enable GPIO clocks */
#define USART1_RCC_GPIORx_ClockCmd()   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE)
#define USART1_RCC_GPIOTx_ClockCmd()

/* Configure AFIO Alternate fonction */
#define USART1_GPIOTx_PinAFConfig()    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1)
#define USART1_GPIORx_PinAFConfig()    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1)

/* Global Variable Definition for GPIO Configuration */
static GPIO_InitTypeDef GPIO_InitStructure;

/* Global Variable Definition for UART Configuration */
static USART_InitTypeDef USART1_InitStructure;
static USART_ClockInitTypeDef USART1_ClockInitStructure;

/* USART1 interrupt receive */
#define USART1_IT_RCV                  1

/* USART1 interrupt send */
#define USART1_IT_SEND                 1

NVIC_InitTypeDef NVIC_USART1_InitStructure;
static volatile unsigned int USART1_NbrOfDataInBuff = 0;
static char USART1_RxBuffer[USART1_RX_BUFF_SIZE];
static char USART1_RxOutputDataBuffer[USART1_RX_STRING_SIZE] = { 0 };

static char* USART1_RxOutputDataBufferPt = USART1_RxOutputDataBuffer;
static char* USART1_ReadPt = USART1_RxBuffer;
static char* USART1_WritePt = USART1_RxBuffer;
static char* USART1_BufferSendPt;
static uint16_t USART1_NbCharToSend = 0;
static uint16_t USART1_NbCharSent = 0;

#define USART3_CONFIG_NAME             USART3
#define USART3_CONFIG_PIN_RX           GPIO_Pin_11
#define USART3_CONFIG_PIN_TX           GPIO_Pin_10
#define USART3_CONFIG_PORT_TX          GPIOC
#define USART3_CONFIG_PORT_RX          GPIOC

/* Enable usart peripheral clocks */
#define RCC_USART3_ClockCmd()          RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE)

/* Enable GPIO clocks */
#define USART3_RCC_GPIORx_ClockCmd()   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE)
#define USART3_RCC_GPIOTx_ClockCmd()

/* Configure AFIO Alternate fonction */
#define USART3_GPIOTx_PinAFConfig()    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3)
#define USART3_GPIORx_PinAFConfig()    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3)

/* Global Variable Definition for GPIO Configuration */
static GPIO_InitTypeDef GPIO_InitStructure;

/* Global Variable Definition for UART Configuration */
static USART_InitTypeDef USART3_InitStructure;
static USART_ClockInitTypeDef USART3_ClockInitStructure;

/* USART3 interrupt receive */
#define USART3_IT_RCV                  1

/* USART3 interrupt send */
#define USART3_IT_SEND                 1

NVIC_InitTypeDef NVIC_USART3_InitStructure;
static volatile unsigned int USART3_NbrOfDataInBuff = 0;
static char USART3_RxBuffer[USART3_RX_BUFF_SIZE];
static char USART3_RxOutputDataBuffer[USART3_RX_STRING_SIZE] = { 0 };

static char* USART3_RxOutputDataBufferPt = USART3_RxOutputDataBuffer;
static char* USART3_ReadPt = USART3_RxBuffer;
static char* USART3_WritePt = USART3_RxBuffer;
static char* USART3_BufferSendPt;
static uint16_t USART3_NbCharToSend = 0;
static uint16_t USART3_NbCharSent = 0;

/* Block signals (auto storage) */
B_daq daq_B;

/* External inputs (root inport signals with auto storage) */
ExtU_daq daq_U;

/* External outputs (root outports fed by signals with auto storage) */
ExtY_daq daq_Y;

/* Real-time model */
RT_MODEL_daq daq_M_;
RT_MODEL_daq *const daq_M = &daq_M_;

/* Function Declaration for TIM4 Configuration */
void TIM4_Configuration(void);

/* Function Declaration for TIM5 Configuration */
void TIM5_Configuration(void);

/* Function Declaration for ADC1 Configuration */
void ADC1_Init(void);
void ADC1_Start(void);

/* Function Declaration for DMA Configuration used for ADC*/
void ADC_DMA_Common_Init(void);

/* Function Declaration for ADC Common Configuration */
void ADC_Common_Init(void);

/* Function Declaration for GPIOA Configuration */
void GPIOA_Configuration(void);

/* Function Declaration for GPIOB Configuration */
void GPIOB_Configuration(void);

/* Function Declaration for GPIOC Configuration */
void GPIOC_Configuration(void);

/* Function Declaration for GPIOD Configuration */
void GPIOD_Configuration(void);

/* Function Declaration for GPIOE Configuration */
void GPIOE_Configuration(void);

/* Function Declaration for GPIOF Configuration */
void GPIOF_Configuration(void);

/* Function Declaration for GPIOG Configuration */
void GPIOG_Configuration(void);

/* Function Declaration for GPIOH Configuration */
void GPIOH_Configuration(void);

/* Function Declaration for GPIOI Configuration */
void GPIOI_Configuration(void);

/* Function Declaration for USART6 Configuration */
void USART6_Config(void);

/* Function Declaration for USART2 Configuration */
void USART2_Config(void);

/* Function Declaration for USART1 Configuration */
void USART1_Config(void);

/* Function Declaration for USART3 Configuration */
void USART3_Config(void);

/*******************************************************************************
 * Function Name  : TIM4_Configuration
 * Description    : TIM4 PWM output or Input_Capture Configuration
 * Input          : -
 *******************************************************************************/
void TIM4_Configuration(void)
{
  /* Time base configuration */
  switch (RCC_APB1_Prescaler) {
   case RCC_HCLK_Div4:
    TIM4_TimeBaseStructure.TIM_Prescaler = (uint16_t) ((SystemCoreClock /2) /
      21000000) - 1;
    break;

   case RCC_HCLK_Div8:
    TIM4_TimeBaseStructure.TIM_Prescaler = (uint16_t) ((SystemCoreClock /4) /
      21000000) - 1;
    break;

   case RCC_HCLK_Div16:
    TIM4_TimeBaseStructure.TIM_Prescaler = (uint16_t) ((SystemCoreClock /8) /
      21000000) - 1;
    break;

   default:
    TIM4_TimeBaseStructure.TIM_Prescaler = (uint16_t)(SystemCoreClock / 21000000)
      - 1;
    break;
  }

  TIM4_TimeBaseStructure.TIM_Period = 21000000 / 100000 -1;
  TIM4_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM4_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM4_TimeBaseStructure);

  /* PWM output mode configuration: Channel CH1*/
  TIM4_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM4_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM4_OCInitStructure.TIM_Pulse = 50 * TIM4_TimeBaseStructure.TIM_Period / 100;
  TIM4_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC1Init(TIM4, &TIM4_OCInitStructure);
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM output mode configuration: Channel CH2*/
  TIM4_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM4_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM4_OCInitStructure.TIM_Pulse = 50 * TIM4_TimeBaseStructure.TIM_Period / 100;
  TIM4_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC2Init(TIM4, &TIM4_OCInitStructure);
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM output mode configuration: Channel CH3*/
  TIM4_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM4_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM4_OCInitStructure.TIM_Pulse = 50 * TIM4_TimeBaseStructure.TIM_Period / 100;
  TIM4_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC3Init(TIM4, &TIM4_OCInitStructure);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM4, ENABLE);

  /* TIM4 enable counter */
  TIM_Cmd(TIM4, ENABLE);
}

/*******************************************************************************
 * Function Name  : TIM5_Configuration
 * Description    : TIM5 PWM output or Input_Capture Configuration
 * Input          : -
 *******************************************************************************/
void TIM5_Configuration(void)
{
  /* Time base configuration */
  switch (RCC_APB1_Prescaler) {
   case RCC_HCLK_Div4:
    TIM5_TimeBaseStructure.TIM_Prescaler = (uint16_t) ((SystemCoreClock /2) /
      21000000) - 1;
    break;

   case RCC_HCLK_Div8:
    TIM5_TimeBaseStructure.TIM_Prescaler = (uint16_t) ((SystemCoreClock /4) /
      21000000) - 1;
    break;

   case RCC_HCLK_Div16:
    TIM5_TimeBaseStructure.TIM_Prescaler = (uint16_t) ((SystemCoreClock /8) /
      21000000) - 1;
    break;

   default:
    TIM5_TimeBaseStructure.TIM_Prescaler = (uint16_t)(SystemCoreClock / 21000000)
      - 1;
    break;
  }

  TIM5_TimeBaseStructure.TIM_Period = 21000000 / 100000 -1;
  TIM5_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM5_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM5, &TIM5_TimeBaseStructure);

  /* PWM output mode configuration: Channel CH1*/
  TIM5_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM5_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM5_OCInitStructure.TIM_Pulse = 50 * TIM5_TimeBaseStructure.TIM_Period / 100;
  TIM5_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM5, &TIM5_OCInitStructure);
  TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);

  /* PWM output mode configuration: Channel CH2*/
  TIM5_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM5_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM5_OCInitStructure.TIM_Pulse = 50 * TIM5_TimeBaseStructure.TIM_Period / 100;
  TIM5_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC2Init(TIM5, &TIM5_OCInitStructure);
  TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM5, ENABLE);

  /* TIM5 enable counter */
  TIM_Cmd(TIM5, ENABLE);
}

/*******************************************************************************
 * Function Name  : ADC1_Init
 * Description    : ADC1 Initialisation
 * Input          : -
 *******************************************************************************/
void ADC1_Init(void)
{
  /* Enable ADC1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  ADC1_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC1_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC1_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC1_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC1_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC1_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC1_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC1_InitStructure);

  /* ADC1 channel configuration *************************************/
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_3Cycles);

  /* ADC1 channel configuration *************************************/
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_3Cycles);

#ifdef ADC1_USE_DMA

  /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

#endif

#ifndef ADC1_USE_DMA
#ifdef ADC_USE_DMA

  /* Enable DMA request after last transfer (multi-ADC mode) ******************/
  ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

#endif
#endif

}

/*******************************************************************************
 * Function Name  : ADC1_Start
 * Description    : Enable and start convertion for ADC1
 * Input          : -
 *******************************************************************************/
void ADC1_Start(void)
{

#ifdef ADC1_USE_DMA

  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);

#endif

  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Start ADC1 Regular Software Conversion */
  ADC_SoftwareStartConv(ADC1);
}

#if ( defined (ADC_USE_DMA) || defined (ADC1_USE_DMA) || defined (ADC2_USE_DMA) || defined (ADC3_USE_DMA) )

/*******************************************************************************
 * Function Name  : ADC_DMA_Common_Init
 * Description    : DMA configuration for ADC
 * Input          : -
 *******************************************************************************/
void ADC_DMA_Common_Init(void)
{
  /* Enable DMA2 peripheral clocks for ADCs*/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

  /* DMA2 configuration */
  ADC_DMA2_InitStructure.DMA_Channel = DMA2_CHANNEL_DEF;
  ADC_DMA2_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC_DR_CCR_ADDRESS;
  ADC_DMA2_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_DMA_BUFFER;
  ADC_DMA2_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  ADC_DMA2_InitStructure.DMA_BufferSize = ADC_NUMBER_OF_CHANNELS;
  ADC_DMA2_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  if (ADC_NUMBER_OF_CHANNELS <= 1) {
    ADC_DMA2_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  } else {
    ADC_DMA2_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  }

  ADC_DMA2_InitStructure.DMA_PeripheralDataSize =
    DMA_PeripheralDataSize_HalfWord;
  ADC_DMA2_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  ADC_DMA2_InitStructure.DMA_Mode = DMA_Mode_Circular;
  ADC_DMA2_InitStructure.DMA_Priority = DMA_Priority_High;
  ADC_DMA2_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  ADC_DMA2_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  ADC_DMA2_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  ADC_DMA2_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_STREAM_DEF, &ADC_DMA2_InitStructure);

  /* DMA2 enable */
  DMA_Cmd(DMA2_STREAM_DEF, ENABLE);
}

#endif

/*******************************************************************************
 * Function Name  : ADC_Common_Init
 * Description    : Parameter Initialisation for all ADCs
 * Input          : -
 *******************************************************************************/
void ADC_Common_Init(void)
{
  /* ADC Common Init */
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);
}

/*******************************************************************************
 * Function Name  : GPIOA_Configuration
 * Description    : GPIO PortA Configuration
 * Input          : -
 *******************************************************************************/
void GPIOA_Configuration(void)
{
  /* Enable GPIOA clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* GPIO default Value */
  GPIO_DeInit(GPIOA);
  GPIOA_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIOA_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOA_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOA_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIOA_InitStructure);
  GPIOA_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIOA_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOA_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOA_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIOA_InitStructure);
  GPIOA_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIOA_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOA_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOA_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIOA_InitStructure);
  GPIOA_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIOA_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOA_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOA_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIOA_InitStructure);
  GPIOA_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIOA_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIOA_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIOA_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(GPIOA, &GPIOA_InitStructure);
  GPIOA_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIOA_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOA_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOA_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIOA_InitStructure);
  GPIOA_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIOA_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOA_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOA_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIOA_InitStructure);
  GPIOA_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIOA_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIOA_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOA_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIOA_InitStructure);
  GPIOA_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIOA_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOA_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOA_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIOA_InitStructure);
  GPIOA_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIOA_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOA_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOA_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIOA_InitStructure);
  GPIOA_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIOA_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOA_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOA_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIOA_InitStructure);
  GPIOA_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIOA_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOA_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOA_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIOA_InitStructure);
  GPIOA_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIOA_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOA_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOA_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIOA_InitStructure);
  GPIO_Write(GPIOA,0);
}

/*******************************************************************************
 * Function Name  : GPIOB_Configuration
 * Description    : GPIO PortB Configuration
 * Input          : -
 *******************************************************************************/
void GPIOB_Configuration(void)
{
  /* Enable GPIOB clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  /* GPIO default Value */
  GPIO_DeInit(GPIOB);
  GPIOB_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIOB_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOB_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOB_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIOB_InitStructure);
  GPIOB_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIOB_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOB_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOB_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIOB_InitStructure);
  GPIOB_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIOB_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOB_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOB_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIOB_InitStructure);
  GPIOB_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIOB_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOB_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOB_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIOB_InitStructure);
  GPIOB_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIOB_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOB_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOB_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIOB_InitStructure);
  GPIOB_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIOB_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOB_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOB_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIOB_InitStructure);
  GPIOB_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIOB_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOB_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOB_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIOB_InitStructure);
  GPIOB_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIOB_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOB_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOB_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIOB_InitStructure);
  GPIOB_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIOB_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOB_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOB_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIOB_InitStructure);
  GPIOB_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIOB_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOB_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOB_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIOB_InitStructure);
  GPIOB_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIOB_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOB_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOB_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIOB_InitStructure);
  GPIOB_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIOB_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOB_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOB_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIOB_InitStructure);
  GPIOB_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIOB_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOB_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOB, &GPIOB_InitStructure);

  /*Alternate function configuration */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  GPIOB_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIOB_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOB_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOB_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIOB_InitStructure);
  GPIO_Write(GPIOB,0);
}

/*******************************************************************************
 * Function Name  : GPIOC_Configuration
 * Description    : GPIO PortC Configuration
 * Input          : -
 *******************************************************************************/
void GPIOC_Configuration(void)
{
  /* Enable GPIOC clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  /* GPIO default Value */
  GPIO_DeInit(GPIOC);
  GPIOC_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIOC_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOC_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOC_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIOC_InitStructure);
  GPIOC_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIOC_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOC_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOC_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIOC_InitStructure);
  GPIOC_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIOC_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOC_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOC_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIOC_InitStructure);
  GPIOC_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIOC_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOC_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOC_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIOC_InitStructure);
  GPIOC_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIOC_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOC_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOC_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIOC_InitStructure);
  GPIOC_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIOC_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOC_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOC_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIOC_InitStructure);
  GPIOC_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIOC_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOC_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOC_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIOC_InitStructure);
  GPIOC_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIOC_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOC_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOC_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIOC_InitStructure);
  GPIOC_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIOC_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOC_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOC_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIOC_InitStructure);
  GPIOC_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIOC_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOC_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOC_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIOC_InitStructure);
  GPIOC_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIOC_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOC_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOC_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIOC_InitStructure);
  GPIOC_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIOC_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOC_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOC_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIOC_InitStructure);
  GPIOC_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIOC_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOC_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOC_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIOC_InitStructure);
  GPIOC_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIOC_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOC_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOC_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIOC_InitStructure);
  GPIOC_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIOC_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOC_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOC_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIOC_InitStructure);
  GPIOC_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIOC_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOC_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOC_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIOC_InitStructure);
  GPIO_Write(GPIOC,0);
}

/*******************************************************************************
 * Function Name  : GPIOD_Configuration
 * Description    : GPIO PortD Configuration
 * Input          : -
 *******************************************************************************/
void GPIOD_Configuration(void)
{
  /* Enable GPIOD clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  /* GPIO default Value */
  GPIO_DeInit(GPIOD);
  GPIOD_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIOD_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOD_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOD_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIOD_InitStructure);
  GPIOD_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIOD_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOD_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOD_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIOD_InitStructure);
  GPIOD_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIOD_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOD_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOD_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIOD_InitStructure);
  GPIOD_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIOD_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOD_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOD_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIOD_InitStructure);
  GPIOD_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIOD_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOD_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOD_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIOD_InitStructure);
  GPIOD_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIOD_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOD_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOD, &GPIOD_InitStructure);

  /*Alternate function configuration */
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  GPIOD_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIOD_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOD_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOD, &GPIOD_InitStructure);

  /*Alternate function configuration */
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  GPIOD_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIOD_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOD_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOD_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIOD_InitStructure);
  GPIOD_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIOD_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOD_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOD_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIOD_InitStructure);
  GPIOD_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIOD_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOD_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOD_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIOD_InitStructure);
  GPIOD_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIOD_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOD_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOD_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIOD_InitStructure);
  GPIOD_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIOD_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOD_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOD_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIOD_InitStructure);
  GPIOD_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIOD_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOD_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOD_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIOD_InitStructure);
  GPIOD_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIOD_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOD_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOD_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIOD_InitStructure);
  GPIOD_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIOD_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOD_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOD, &GPIOD_InitStructure);

  /*Alternate function configuration */
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  GPIOD_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIOD_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOD_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOD_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIOD_InitStructure);
  GPIO_Write(GPIOD,0);
}

/*******************************************************************************
 * Function Name  : GPIOE_Configuration
 * Description    : GPIO PortE Configuration
 * Input          : -
 *******************************************************************************/
void GPIOE_Configuration(void)
{
  /* Enable GPIOE clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

  /* GPIO default Value */
  GPIO_DeInit(GPIOE);
  GPIOE_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIOE_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOE_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOE_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIOE_InitStructure);
  GPIOE_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIOE_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOE_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOE_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIOE_InitStructure);
  GPIOE_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIOE_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOE_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOE_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIOE_InitStructure);
  GPIOE_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIOE_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOE_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOE_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIOE_InitStructure);
  GPIOE_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIOE_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOE_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOE_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIOE_InitStructure);
  GPIOE_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIOE_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOE_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOE_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIOE_InitStructure);
  GPIOE_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIOE_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOE_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOE_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIOE_InitStructure);
  GPIOE_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIOE_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOE_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOE_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIOE_InitStructure);
  GPIOE_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIOE_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOE_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOE_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIOE_InitStructure);
  GPIOE_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIOE_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOE_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOE_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIOE_InitStructure);
  GPIOE_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIOE_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOE_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOE_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIOE_InitStructure);
  GPIOE_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIOE_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOE_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOE_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIOE_InitStructure);
  GPIOE_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIOE_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOE_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOE_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIOE_InitStructure);
  GPIOE_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIOE_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOE_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOE_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIOE_InitStructure);
  GPIOE_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIOE_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOE_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOE_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIOE_InitStructure);
  GPIOE_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIOE_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOE_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOE_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIOE_InitStructure);
  GPIO_Write(GPIOE,0);
}

/*******************************************************************************
 * Function Name  : GPIOF_Configuration
 * Description    : GPIO PortF Configuration
 * Input          : -
 *******************************************************************************/
void GPIOF_Configuration(void)
{
  /* Enable GPIOF clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

  /* GPIO default Value */
  GPIO_DeInit(GPIOF);
  GPIOF_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIOF_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOF_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOF_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOF, &GPIOF_InitStructure);
  GPIOF_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIOF_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOF_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOF_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOF, &GPIOF_InitStructure);
  GPIOF_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIOF_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOF_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOF_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOF, &GPIOF_InitStructure);
  GPIOF_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIOF_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOF_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOF_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOF, &GPIOF_InitStructure);
  GPIOF_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIOF_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOF_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOF_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOF, &GPIOF_InitStructure);
  GPIOF_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIOF_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOF_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOF_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOF, &GPIOF_InitStructure);
  GPIOF_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIOF_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOF_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOF_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOF, &GPIOF_InitStructure);
  GPIOF_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIOF_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOF_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOF_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOF, &GPIOF_InitStructure);
  GPIOF_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIOF_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOF_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOF_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOF, &GPIOF_InitStructure);
  GPIOF_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIOF_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOF_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOF_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOF, &GPIOF_InitStructure);
  GPIOF_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIOF_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOF_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOF_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOF, &GPIOF_InitStructure);
  GPIOF_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIOF_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOF_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOF_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOF, &GPIOF_InitStructure);
  GPIOF_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIOF_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOF_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOF_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOF, &GPIOF_InitStructure);
  GPIOF_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIOF_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOF_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOF_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOF, &GPIOF_InitStructure);
  GPIOF_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIOF_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOF_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOF_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOF, &GPIOF_InitStructure);
  GPIOF_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIOF_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOF_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOF_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOF, &GPIOF_InitStructure);
  GPIO_Write(GPIOF,0);
}

/*******************************************************************************
 * Function Name  : GPIOG_Configuration
 * Description    : GPIO PortG Configuration
 * Input          : -
 *******************************************************************************/
void GPIOG_Configuration(void)
{
  /* Enable GPIOG clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

  /* GPIO default Value */
  GPIO_DeInit(GPIOG);
  GPIOG_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIOG_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOG_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOG_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOG, &GPIOG_InitStructure);
  GPIOG_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIOG_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOG_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOG_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOG, &GPIOG_InitStructure);
  GPIOG_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIOG_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOG_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOG_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOG, &GPIOG_InitStructure);
  GPIOG_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIOG_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOG_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOG_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOG, &GPIOG_InitStructure);
  GPIOG_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIOG_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOG_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOG_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOG, &GPIOG_InitStructure);
  GPIOG_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIOG_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOG_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOG_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOG, &GPIOG_InitStructure);
  GPIOG_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIOG_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOG_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOG_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOG, &GPIOG_InitStructure);
  GPIOG_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIOG_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOG_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOG_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOG, &GPIOG_InitStructure);
  GPIOG_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIOG_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOG_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOG_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOG, &GPIOG_InitStructure);
  GPIOG_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIOG_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOG_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOG_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOG, &GPIOG_InitStructure);
  GPIOG_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIOG_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOG_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOG_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOG, &GPIOG_InitStructure);
  GPIOG_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIOG_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOG_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOG_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOG, &GPIOG_InitStructure);
  GPIOG_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIOG_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOG_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOG_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOG, &GPIOG_InitStructure);
  GPIOG_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIOG_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOG_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOG_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOG, &GPIOG_InitStructure);
  GPIOG_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIOG_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOG_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOG_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOG, &GPIOG_InitStructure);
  GPIOG_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIOG_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOG_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOG_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOG, &GPIOG_InitStructure);
  GPIO_Write(GPIOG,0);
}

/*******************************************************************************
 * Function Name  : GPIOH_Configuration
 * Description    : GPIO PortH Configuration
 * Input          : -
 *******************************************************************************/
void GPIOH_Configuration(void)
{
  /* Enable GPIOH clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);

  /* GPIO default Value */
  GPIO_DeInit(GPIOH);
  GPIOH_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIOH_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOH_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOH_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOH, &GPIOH_InitStructure);
  GPIOH_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIOH_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOH_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOH_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOH, &GPIOH_InitStructure);
  GPIOH_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIOH_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOH_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOH_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOH, &GPIOH_InitStructure);
  GPIOH_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIOH_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOH_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOH, &GPIOH_InitStructure);

  /*Alternate function configuration */
  GPIO_PinAFConfig(GPIOH, GPIO_PinSource4, GPIO_AF_I2C2);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
  GPIOH_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIOH_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOH_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOH, &GPIOH_InitStructure);

  /*Alternate function configuration */
  GPIO_PinAFConfig(GPIOH, GPIO_PinSource5, GPIO_AF_I2C2);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
  GPIOH_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIOH_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOH_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOH_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOH, &GPIOH_InitStructure);
  GPIOH_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIOH_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOH_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOH_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOH, &GPIOH_InitStructure);
  GPIOH_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIOH_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOH_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOH_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOH, &GPIOH_InitStructure);
  GPIOH_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIOH_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOH_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOH_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOH, &GPIOH_InitStructure);
  GPIOH_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIOH_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOH_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOH_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOH, &GPIOH_InitStructure);
  GPIOH_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIOH_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOH_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOH, &GPIOH_InitStructure);

  /*Alternate function configuration */
  GPIO_PinAFConfig(GPIOH, GPIO_PinSource11, GPIO_AF_TIM5);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
  GPIOH_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIOH_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOH_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIOH_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(GPIOH, &GPIOH_InitStructure);
  GPIOH_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIOH_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOH_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOH_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOH, &GPIOH_InitStructure);
  GPIOH_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIOH_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOH_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOH_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOH, &GPIOH_InitStructure);
  GPIOH_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIOH_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOH_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOH_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOH, &GPIOH_InitStructure);
  GPIO_Write(GPIOH,0);
}

/*******************************************************************************
 * Function Name  : GPIOI_Configuration
 * Description    : GPIO PortI Configuration
 * Input          : -
 *******************************************************************************/
void GPIOI_Configuration(void)
{
  /* Enable GPIOI clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);

  /* GPIO default Value */
  GPIO_DeInit(GPIOI);
  GPIOI_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIOI_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOI_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOI_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOI, &GPIOI_InitStructure);
  GPIOI_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIOI_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOI_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOI, &GPIOI_InitStructure);

  /*Alternate function configuration */
  GPIO_PinAFConfig(GPIOI, GPIO_PinSource1, GPIO_AF_SPI2);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  GPIOI_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIOI_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOI_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOI_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOI, &GPIOI_InitStructure);
  GPIOI_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIOI_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOI_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOI, &GPIOI_InitStructure);

  /*Alternate function configuration */
  GPIO_PinAFConfig(GPIOI, GPIO_PinSource3, GPIO_AF_SPI2);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  GPIOI_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIOI_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOI_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOI_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOI, &GPIOI_InitStructure);
  GPIOI_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIOI_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOI_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOI_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOI, &GPIOI_InitStructure);
  GPIOI_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIOI_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOI_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOI_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOI, &GPIOI_InitStructure);
  GPIOI_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIOI_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOI_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOI_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOI, &GPIOI_InitStructure);
  GPIOI_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIOI_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOI_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOI_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOI, &GPIOI_InitStructure);
  GPIOI_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIOI_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOI_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOI_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOI, &GPIOI_InitStructure);
  GPIOI_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIOI_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOI_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOI_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOI, &GPIOI_InitStructure);
  GPIOI_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIOI_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOI_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOI_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOI, &GPIOI_InitStructure);
  GPIO_Write(GPIOI,0);
}

/*******************************************************************************
 * Function Name  : USART6_Config
 * Description    : USART Configuration
 * Input          : -
 *******************************************************************************/
void USART6_Config(void)
{
  // USART6 IO configuration ------------------------------------//
  // Enable GPIO clock
  USART6_RCC_GPIORx_ClockCmd();
  USART6_RCC_GPIOTx_ClockCmd();

  // Set USART_TX as PP AF
  GPIO_InitStructure.GPIO_Pin = USART6_CONFIG_PIN_TX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(USART6_CONFIG_PORT_TX, &GPIO_InitStructure);

  // Set USART_RX as Pull-Up
  GPIO_InitStructure.GPIO_Pin = USART6_CONFIG_PIN_RX;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(USART6_CONFIG_PORT_RX, &GPIO_InitStructure);

  // configure AFIO
  USART6_GPIOTx_PinAFConfig();
  USART6_GPIORx_PinAFConfig();

  //USART6 clock enable
  RCC_USART6_ClockCmd();
  USART6_InitStructure.USART_BaudRate = 921600;
  USART6_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART6_InitStructure.USART_StopBits = USART_StopBits_1;
  USART6_InitStructure.USART_Parity = USART_Parity_No;
  USART6_InitStructure.USART_HardwareFlowControl =
    USART_HardwareFlowControl_None;
  USART6_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART6_ClockInitStructure.USART_Clock = USART_Clock_Disable;
  USART6_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
  USART6_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
  USART6_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
  USART_Init(USART6_CONFIG_NAME, &USART6_InitStructure);
  USART_ClockInit(USART6_CONFIG_NAME, &USART6_ClockInitStructure);

  // Enable USART6 selected from PIL config model.
  USART_Cmd(USART6_CONFIG_NAME, ENABLE);
}

/*******************************************************************************
 * Function Name  : USART6_IRQHandler
 * Description    : USART interrupt reception management.
 * Input          : -
 *******************************************************************************/
void USART6_IRQHandler(void);
void USART6_IRQHandler(void)
{
  if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET) {
    /* Read one byte from the receive data register */
    if (USART6_NbrOfDataInBuff < USART6_RX_BUFF_SIZE) {
      *USART6_WritePt++ = USART6->DR;
      USART6_NbrOfDataInBuff++;
      if (USART6_WritePt > &USART6_RxBuffer[USART6_RX_BUFF_SIZE])
        USART6_WritePt = USART6_RxBuffer;
    } else {
      /* Take care: Char is lost. Increase Buffer size.*/
      (void)USART6->DR;
    }
  }

  if (USART_GetITStatus(USART6, USART_IT_TXE) != RESET) {
    USART_SendData(USART6, *USART6_BufferSendPt++);
    USART6_NbCharSent++;
    if (--USART6_NbCharToSend <= 0)
      USART_ITConfig(USART6, USART_IT_TXE, DISABLE);
  }
}

/*******************************************************************************
 * Function Name  : USART2_Config
 * Description    : USART Configuration
 * Input          : -
 *******************************************************************************/
void USART2_Config(void)
{
  // USART2 IO configuration ------------------------------------//
  // Enable GPIO clock
  USART2_RCC_GPIORx_ClockCmd();
  USART2_RCC_GPIOTx_ClockCmd();

  // Set USART_TX as PP AF
  GPIO_InitStructure.GPIO_Pin = USART2_CONFIG_PIN_TX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(USART2_CONFIG_PORT_TX, &GPIO_InitStructure);

  // Set USART_RX as Pull-Up
  GPIO_InitStructure.GPIO_Pin = USART2_CONFIG_PIN_RX;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(USART2_CONFIG_PORT_RX, &GPIO_InitStructure);

  // configure AFIO
  USART2_GPIOTx_PinAFConfig();
  USART2_GPIORx_PinAFConfig();

  //USART2 clock enable
  RCC_USART2_ClockCmd();
  USART2_InitStructure.USART_BaudRate = 115200;
  USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART2_InitStructure.USART_StopBits = USART_StopBits_1;
  USART2_InitStructure.USART_Parity = USART_Parity_No;
  USART2_InitStructure.USART_HardwareFlowControl =
    USART_HardwareFlowControl_None;
  USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART2_ClockInitStructure.USART_Clock = USART_Clock_Disable;
  USART2_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
  USART2_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
  USART2_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
  USART_Init(USART2_CONFIG_NAME, &USART2_InitStructure);
  USART_ClockInit(USART2_CONFIG_NAME, &USART2_ClockInitStructure);

  // Enable USART2 selected from PIL config model.
  USART_Cmd(USART2_CONFIG_NAME, ENABLE);
}

/*******************************************************************************
 * Function Name  : USART2_IRQHandler
 * Description    : USART interrupt reception management.
 * Input          : -
 *******************************************************************************/
void USART2_IRQHandler(void);
void USART2_IRQHandler(void)
{
  if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
    /* Read one byte from the receive data register */
    if (USART2_NbrOfDataInBuff < USART2_RX_BUFF_SIZE) {
      *USART2_WritePt++ = USART2->DR;
      USART2_NbrOfDataInBuff++;
      if (USART2_WritePt > &USART2_RxBuffer[USART2_RX_BUFF_SIZE])
        USART2_WritePt = USART2_RxBuffer;
    } else {
      /* Take care: Char is lost. Increase Buffer size.*/
      (void)USART2->DR;
    }
  }

  if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET) {
    USART_SendData(USART2, *USART2_BufferSendPt++);
    USART2_NbCharSent++;
    if (--USART2_NbCharToSend <= 0)
      USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
  }
}

/*******************************************************************************
 * Function Name  : USART1_Config
 * Description    : USART Configuration
 * Input          : -
 *******************************************************************************/
void USART1_Config(void)
{
  // USART1 IO configuration ------------------------------------//
  // Enable GPIO clock
  USART1_RCC_GPIORx_ClockCmd();
  USART1_RCC_GPIOTx_ClockCmd();

  // Set USART_TX as PP AF
  GPIO_InitStructure.GPIO_Pin = USART1_CONFIG_PIN_TX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(USART1_CONFIG_PORT_TX, &GPIO_InitStructure);

  // Set USART_RX as Pull-Up
  GPIO_InitStructure.GPIO_Pin = USART1_CONFIG_PIN_RX;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(USART1_CONFIG_PORT_RX, &GPIO_InitStructure);

  // configure AFIO
  USART1_GPIOTx_PinAFConfig();
  USART1_GPIORx_PinAFConfig();

  //USART1 clock enable
  RCC_USART1_ClockCmd();
  USART1_InitStructure.USART_BaudRate = 921600;
  USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART1_InitStructure.USART_StopBits = USART_StopBits_1;
  USART1_InitStructure.USART_Parity = USART_Parity_No;
  USART1_InitStructure.USART_HardwareFlowControl =
    USART_HardwareFlowControl_None;
  USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART1_ClockInitStructure.USART_Clock = USART_Clock_Disable;
  USART1_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
  USART1_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
  USART1_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
  USART_Init(USART1_CONFIG_NAME, &USART1_InitStructure);
  USART_ClockInit(USART1_CONFIG_NAME, &USART1_ClockInitStructure);

  // Enable USART1 selected from PIL config model.
  USART_Cmd(USART1_CONFIG_NAME, ENABLE);
}

/*******************************************************************************
 * Function Name  : USART1_IRQHandler
 * Description    : USART interrupt reception management.
 * Input          : -
 *******************************************************************************/
void USART1_IRQHandler(void);
void USART1_IRQHandler(void)
{
  if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
    /* Read one byte from the receive data register */
    if (USART1_NbrOfDataInBuff < USART1_RX_BUFF_SIZE) {
      *USART1_WritePt++ = USART1->DR;
      USART1_NbrOfDataInBuff++;
      if (USART1_WritePt > &USART1_RxBuffer[USART1_RX_BUFF_SIZE])
        USART1_WritePt = USART1_RxBuffer;
    } else {
      /* Take care: Char is lost. Increase Buffer size.*/
      (void)USART1->DR;
    }
  }

  if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET) {
    USART_SendData(USART1, *USART1_BufferSendPt++);
    USART1_NbCharSent++;
    if (--USART1_NbCharToSend <= 0)
      USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
  }
}

/*******************************************************************************
 * Function Name  : USART3_Config
 * Description    : USART Configuration
 * Input          : -
 *******************************************************************************/
void USART3_Config(void)
{
  // USART3 IO configuration ------------------------------------//
  // Enable GPIO clock
  USART3_RCC_GPIORx_ClockCmd();
  USART3_RCC_GPIOTx_ClockCmd();

  // Set USART_TX as PP AF
  GPIO_InitStructure.GPIO_Pin = USART3_CONFIG_PIN_TX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(USART3_CONFIG_PORT_TX, &GPIO_InitStructure);

  // Set USART_RX as Pull-Up
  GPIO_InitStructure.GPIO_Pin = USART3_CONFIG_PIN_RX;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(USART3_CONFIG_PORT_RX, &GPIO_InitStructure);

  // configure AFIO
  USART3_GPIOTx_PinAFConfig();
  USART3_GPIORx_PinAFConfig();

  //USART3 clock enable
  RCC_USART3_ClockCmd();
  USART3_InitStructure.USART_BaudRate = 921600;
  USART3_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART3_InitStructure.USART_StopBits = USART_StopBits_1;
  USART3_InitStructure.USART_Parity = USART_Parity_No;
  USART3_InitStructure.USART_HardwareFlowControl =
    USART_HardwareFlowControl_None;
  USART3_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART3_ClockInitStructure.USART_Clock = USART_Clock_Disable;
  USART3_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
  USART3_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
  USART3_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
  USART_Init(USART3_CONFIG_NAME, &USART3_InitStructure);
  USART_ClockInit(USART3_CONFIG_NAME, &USART3_ClockInitStructure);

  // Enable USART3 selected from PIL config model.
  USART_Cmd(USART3_CONFIG_NAME, ENABLE);
}

/*******************************************************************************
 * Function Name  : USART3_IRQHandler
 * Description    : USART interrupt reception management.
 * Input          : -
 *******************************************************************************/
void USART3_IRQHandler(void);
void USART3_IRQHandler(void)
{
  if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
    /* Read one byte from the receive data register */
    if (USART3_NbrOfDataInBuff < USART3_RX_BUFF_SIZE) {
      *USART3_WritePt++ = USART3->DR;
      USART3_NbrOfDataInBuff++;
      if (USART3_WritePt > &USART3_RxBuffer[USART3_RX_BUFF_SIZE])
        USART3_WritePt = USART3_RxBuffer;
    } else {
      /* Take care: Char is lost. Increase Buffer size.*/
      (void)USART3->DR;
    }
  }

  if (USART_GetITStatus(USART3, USART_IT_TXE) != RESET) {
    USART_SendData(USART3, *USART3_BufferSendPt++);
    USART3_NbCharSent++;
    if (--USART3_NbCharToSend <= 0)
      USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
  }
}

/*******************************************************************************
 * Function Name  : TIM4_IRQHandler
 * Description    : This function handles TIM4 interrupt request.
 * Input          : -
 *******************************************************************************/
void TIM4_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : TIM5_IRQHandler
 * Description    : This function handles TIM5 interrupt request.
 * Input          : -
 *******************************************************************************/
void TIM5_IRQHandler(void)
{
}

/* Model step function */
void daq_step(void)
{
  {
    /* user code (Output function Header) */
#ifndef USART1_IT_SEND

    u16 i = 0;
    char* charToSend;

#endif

    u16 NbData_Read = 0;               //Nb of data copied into the output data buffer
    int i;                             //Loop counter

#ifndef USART3_IT_SEND

    u16 i = 0;
    char* charToSend;

#endif
//
    //u16 NbData_Read = 0;               //Nb of data copied into the output data buffer
    //int i;                             //Loop counter

#ifndef USART6_IT_SEND

    u16 i = 0;
    char* charToSend;

#endif

    //u16 NbData_Read = 0;               //Nb of data copied into the output data buffer
    //int i;                             //Loop counter
    int OffsetADC1_L = 0;
    //u16 NbData_Read = 0;               //Nb of data copied into the output data buffer
    //int i;                             //Loop counter

#ifndef USART2_IT_SEND

    u16 i = 0;
    char* charToSend;

#endif

    /* user code (Output function Body) */
#ifndef USART1_IT_RCV

    /* Pulling reception: Get received char while any and less than buff size*/
    while ((USART1->SR & USART_FLAG_RXNE) != (uint16_t)RESET &&
           USART1_NbrOfDataInBuff < 500) {
      /* Read one byte from the receive data register */
      *USART1_WritePt++ = USART1->DR;
      USART1_NbrOfDataInBuff++;
      if (USART1_WritePt > &USART1_RxBuffer[500])
        USART1_WritePt = USART1_RxBuffer;
    }

#endif

    /* Disable Rx interrupt during processing */
    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    /* Processing when data has been received only*/
    if (USART1_NbrOfDataInBuff ) {
      for (i = 0; i< 500;i++) {
        USART1_RxOutputDataBuffer[i] = *USART1_ReadPt++;
        USART1_NbrOfDataInBuff--;
        NbData_Read++;
        if (USART1_ReadPt > &USART1_RxBuffer[500]) {
          USART1_ReadPt = USART1_RxBuffer;
        }

        if (USART1_NbrOfDataInBuff == 0) {
          break;
        }
      }
    }

    daq_Y.usart1_NbRcv = NbData_Read;

    /* Enable Rx interrupt after processing */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART1_RxOutputDataBufferPt = USART1_RxOutputDataBuffer;

    {
      int_T i1;
      uint8_T *y1 = &daq_Y.usart1_RcvVal[0];
      for (i1=0; i1 < 500; i1++) {
        y1[i1] = *USART1_RxOutputDataBufferPt++;
      }
    }

#ifndef USART3_IT_RCV

    /* Pulling reception: Get received char while any and less than buff size*/
    while ((USART3->SR & USART_FLAG_RXNE) != (uint16_t)RESET &&
           USART3_NbrOfDataInBuff < 1000) {
      /* Read one byte from the receive data register */
      *USART3_WritePt++ = USART3->DR;
      USART3_NbrOfDataInBuff++;
      if (USART3_WritePt > &USART3_RxBuffer[1000])
        USART3_WritePt = USART3_RxBuffer;
    }

#endif

    /* Disable Rx interrupt during processing */
    USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);

    /* Processing when data has been received only*/
    if (USART3_NbrOfDataInBuff ) {
      for (i = 0; i< 100;i++) {
        USART3_RxOutputDataBuffer[i] = *USART3_ReadPt++;
        USART3_NbrOfDataInBuff--;
        NbData_Read++;
        if (USART3_ReadPt > &USART3_RxBuffer[1000]) {
          USART3_ReadPt = USART3_RxBuffer;
        }

        if (USART3_NbrOfDataInBuff == 0) {
          break;
        }
      }
    }

    daq_Y.usart3_NbRcv = NbData_Read;

    /* Enable Rx interrupt after processing */
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART3_RxOutputDataBufferPt = USART3_RxOutputDataBuffer;

    {
      int_T i1;
      uint8_T *y1 = &daq_Y.usart3_RcvVal[0];
      for (i1=0; i1 < 100; i1++) {
        y1[i1] = *USART3_RxOutputDataBufferPt++;
      }
    }

#ifndef USART6_IT_RCV

    /* Pulling reception: Get received char while any and less than buff size*/
    while ((USART6->SR & USART_FLAG_RXNE) != (uint16_t)RESET &&
           USART6_NbrOfDataInBuff < 1000) {
      /* Read one byte from the receive data register */
      *USART6_WritePt++ = USART6->DR;
      USART6_NbrOfDataInBuff++;
      if (USART6_WritePt > &USART6_RxBuffer[1000])
        USART6_WritePt = USART6_RxBuffer;
    }

#endif

    /* Disable Rx interrupt during processing */
    USART_ITConfig(USART6, USART_IT_RXNE, DISABLE);

    /* Processing when data has been received only*/
    if (USART6_NbrOfDataInBuff ) {
      for (i = 0; i< 100;i++) {
        USART6_RxOutputDataBuffer[i] = *USART6_ReadPt++;
        USART6_NbrOfDataInBuff--;
        NbData_Read++;
        if (USART6_ReadPt > &USART6_RxBuffer[1000]) {
          USART6_ReadPt = USART6_RxBuffer;
        }

        if (USART6_NbrOfDataInBuff == 0) {
          break;
        }
      }
    }

    daq_Y.usart6_NbRcv = NbData_Read;

    /* Enable Rx interrupt after processing */
    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
    USART6_RxOutputDataBufferPt = USART6_RxOutputDataBuffer;

    {
      int_T i1;
      uint8_T *y1 = &daq_Y.usart6_RcvVal[0];
      for (i1=0; i1 < 100; i1++) {
        y1[i1] = *USART6_RxOutputDataBufferPt++;
      }
    }

#ifdef ADC1_USE_DMA

    daq_Y.ADC123_IN0 = ADCConvertedValue[0];

#elif defined (ADC_USE_DMA)

    daq_Y.ADC123_IN0 = ADCConvertedValue[OffsetADC1_L + 0];

#else

    daq_Y.ADC123_IN0 = *(uint16_t*)ADC1_DR_ADDRESS;

#endif

#ifdef ADC1_USE_DMA

    daq_Y.ADC123_IN3 = ADCConvertedValue[1];

#elif defined (ADC_USE_DMA)

    daq_Y.ADC123_IN3 = ADCConvertedValue[OffsetADC1_L + 1];

#else

    daq_Y.ADC123_IN3 = *(uint16_t*)ADC1_DR_ADDRESS;

#endif

#ifndef USART2_IT_RCV

    /* Pulling reception: Get received char while any and less than buff size*/
    while ((USART2->SR & USART_FLAG_RXNE) != (uint16_t)RESET &&
           USART2_NbrOfDataInBuff < 18) {
      /* Read one byte from the receive data register */
      *USART2_WritePt++ = USART2->DR;
      USART2_NbrOfDataInBuff++;
      if (USART2_WritePt > &USART2_RxBuffer[18])
        USART2_WritePt = USART2_RxBuffer;
    }

#endif

    /* Disable Rx interrupt during processing */
    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);

    /* Processing when data has been received only*/
    if (USART2_NbrOfDataInBuff ) {
      for (i = 0; i< 18;i++) {
        USART2_RxOutputDataBuffer[i] = *USART2_ReadPt++;
        USART2_NbrOfDataInBuff--;
        NbData_Read++;
        if (USART2_ReadPt > &USART2_RxBuffer[18]) {
          USART2_ReadPt = USART2_RxBuffer;
        }

        if (USART2_NbrOfDataInBuff == 0) {
          break;
        }
      }
    }

    daq_Y.usart2_NbRcv = NbData_Read;

    /* Enable Rx interrupt after processing */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART2_RxOutputDataBufferPt = USART2_RxOutputDataBuffer;

    {
      int_T i1;
      uint8_T *y1 = &daq_Y.usart2_RcvVal[0];
      for (i1=0; i1 < 18; i1++) {
        y1[i1] = *USART2_RxOutputDataBufferPt++;
      }
    }

    daq_B.GPIO_Read1 = GPIO_ReadInputData(GPIOB);
    daq_B.GPIO_Read2 = GPIO_ReadInputData(GPIOA);
    daq_B.GPIO_Read4 = GPIO_ReadInputData(GPIOI);
    daq_B.GPIO_Read5 = GPIO_ReadInputData(GPIOH);

    /* DigitalClock: '<S2>/Digital Clock' */
    daq_Y.time = ((daq_M->Timing.clockTick0) * 0.2);

    /* S-Function Block: <S9>/USART_Send4 */

    /* S-Function Block: <S9>/USART_Receive4 */

    /* S-Function Block: <S11>/USART_Send4 */

    /* S-Function Block: <S11>/USART_Receive4 */

    /* S-Function Block: <S12>/USART_Send4 */

    /* S-Function Block: <S12>/USART_Receive4 */

    /* S-Function Block: <S5>/ADC_Read */

    /* Outport: '<Root>/Estop' */
    daq_Y.Estop = 0.0;

    /* Outport: '<Root>/do1' */
    daq_Y.do1 = 0.0;

    /* Outport: '<Root>/do2' */
    daq_Y.do2 = 0.0;

    /* Outport: '<Root>/do3' */
    daq_Y.do3 = 0.0;

    /* Outport: '<Root>/do4' */
    daq_Y.do4 = 0.0;

    /* Outport: '<Root>/do5' */
    daq_Y.do5 = 0.0;

    /* S-Function Block: <S10>/USART_Receive4 */

    /* S-Function Block: <S10>/USART_Send4 */

    /* S-Function Block: <S6>/GPIO_Read1 */

    /* S-Function Block: <S6>/GPIO_Read2 */

    /* S-Function Block: <S6>/GPIO_Read4 */

    /* S-Function Block: <S6>/GPIO_Read5 */

    /* S-Function Block: <S6>/GPIO_Write */

    /* S-Function Block: <S6>/GPIO_Write1 */

    /* S-Function Block: <S6>/GPIO_Write2 */

    /* S-Function Block: <S6>/GPIO_Write3 */

    /* S-Function Block: <S6>/GPIO_Write4 */

    /* S-Function Block: <S6>/GPIO_Write5 */

    /* Outport: '<Root>/imuData' */
    daq_Y.imuData = 0.0;

    /* user code (Output function Trailer) */
#ifdef USART1_IT_SEND

    USART1_BufferSendPt = (char *)&daq_U.usart1_SendVal[0];
    USART1_NbCharToSend = daq_U.usart1_Nb2Send;
    if (USART1_NbCharToSend) {
      /* Initialize nb of char sent to 0 */
      USART1_NbCharSent = 0;

      /* Send char is allowed */
      USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    }

    daq_Y.usart1_NbSent = USART1_NbCharSent;

#endif

#ifndef USART1_IT_SEND

    charToSend = (char *)&daq_U.usart1_SendVal[0];
    daq_Y.usart1_NbSent = 0;
    for (i=0; i<daq_U.usart1_Nb2Send; i++) {
      while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) ;
      USART_SendData(USART1, *charToSend++);
      daq_Y.usart1_NbSent++;
    }

#endif

#ifdef USART3_IT_SEND

    USART3_BufferSendPt = (char *)&daq_U.usart3_SendVal[0];
    USART3_NbCharToSend = daq_U.usart3_Nb2Send;
    if (USART3_NbCharToSend) {
      /* Initialize nb of char sent to 0 */
      USART3_NbCharSent = 0;

      /* Send char is allowed */
      USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
    }

    daq_Y.usart3_NbSent = USART3_NbCharSent;

#endif

#ifndef USART3_IT_SEND

    charToSend = (char *)&daq_U.usart3_SendVal[0];
    daq_Y.usart3_NbSent = 0;
    for (i=0; i<daq_U.usart3_Nb2Send; i++) {
      while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET) ;
      USART_SendData(USART3, *charToSend++);
      daq_Y.usart3_NbSent++;
    }

#endif

#ifdef USART6_IT_SEND

    USART6_BufferSendPt = (char *)&daq_U.usart6_SendVal[0];
    USART6_NbCharToSend = daq_U.usart6_Nb2Send;
    if (USART6_NbCharToSend) {
      /* Initialize nb of char sent to 0 */
      USART6_NbCharSent = 0;

      /* Send char is allowed */
      USART_ITConfig(USART6, USART_IT_TXE, ENABLE);
    }

    daq_Y.usart6_NbSent = USART6_NbCharSent;

#endif

#ifndef USART6_IT_SEND

    charToSend = (char *)&daq_U.usart6_SendVal[0];
    daq_Y.usart6_NbSent = 0;
    for (i=0; i<daq_U.usart6_Nb2Send; i++) {
      while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET) ;
      USART_SendData(USART6, *charToSend++);
      daq_Y.usart6_NbSent++;
    }

#endif

#ifdef USART2_IT_SEND

    USART2_BufferSendPt = (char *)&daq_U.usart2_SendVal[0];
    USART2_NbCharToSend = daq_U.usart2_Nb2Send;
    if (USART2_NbCharToSend) {
      /* Initialize nb of char sent to 0 */
      USART2_NbCharSent = 0;

      /* Send char is allowed */
      USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
    }

    daq_B.USART_Send4_m = USART2_NbCharSent;

#endif

#ifndef USART2_IT_SEND

    charToSend = (char *)&daq_U.usart2_SendVal[0];
    daq_B.USART_Send4_m = 0;
    for (i=0; i<daq_U.usart2_Nb2Send; i++) {
      while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) ;
      USART_SendData(USART2, *charToSend++);
      daq_B.USART_Send4_m++;
    }

#endif

    GPIO_ToggleBits(GPIOD,GPIO_Pin_13);
    GPIO_ToggleBits(GPIOD,GPIO_Pin_14);
  }

  /* Update for S-Function (TIMERS_Config): '<S7>/Timers' */

  /* Timer frequency is an input port */
  TIM4->ARR = 21000000 / daq_P.pwm5_freq_Value -1;
  if (0 < 0) {
    /* Disable output and complementary output */
    //            TIM4->BDTR &= 0x7FFF;  //MOE = 0
    TIM4->BDTR |= 0x8000;              //MOE = 1
    TIM4->BDTR &= 0xF7FF;              //OSSR = 0

    //            TIM4->CCER |= 0x4;     //CC1NE = 1
    //            TIM4->CCER &= 0xFFFE;  //CC1E = 0
    TIM4->CCER &= 0xFFFA;              //CC1E = 0 CC1NE = 0
    TIM4->CR2 &= 0xFCFF;               //OIS1 = 0 OIS1N = 0
  } else {
    // Enable output and complementary output and update dutyCycle
    TIM4->BDTR |= 0x8000;              //MOE = 1
    TIM4->CCER |= 0x5;                 //CC1NE = 1 and CC1E = 1

    // Channel1 duty cycle is an input port
    TIM4->CCR1 = 0 * TIM4->ARR / 100;
  }

  if (0 < 0) {
    /* Disable output and complementary output */
    //            TIM4->BDTR &= 0x7FFF;  //MOE = 0
    TIM4->BDTR |= 0x8000;              //MOE = 1
    TIM4->BDTR &= 0xF7FF;              //OSSR = 0

    //            TIM4->CCER |= 0x40;    //CC2NE = 1
    //            TIM4->CCER &= 0xFFEF;  //CC2E = 0
    TIM4->CCER &= 0xFFAF;              //CC2E = 0 CC2NE = 0
    TIM4->CR2 &= 0xF3FF;               //OIS2 = 0 OIS2N = 0
  } else {
    /* Enable output and complementary output and update dutyCycle*/
    TIM4->BDTR |= 0x8000;              //MOE = 1
    TIM4->CCER |= 0x50;                //CC2NE = 1 and CC2E = 1

    /* Channel2 duty cycle is an input port */
    TIM4->CCR2 = 0 * TIM4->ARR / 100;
  }

  if (daq_U.pwm5_1 < 0) {
    /* Disable output and complementary output */
    //            TIM4->BDTR &= 0x7FFF;  //MOE = 0
    TIM4->BDTR |= 0x8000;              //MOE = 1
    TIM4->BDTR &= 0xF7FF;              //OSSR = 0

    //            TIM4->CCER |= 0x400;   //CC3NE = 1
    //            TIM4->CCER &= 0xFEFF;  //CC3E = 0
    TIM4->CCER &= 0xFAFF;              //CC3E = 0 CC3NE = 0
    TIM4->CR2 &= 0xCFFF;               //OIS3 = 0 OIS3N = 0
  } else {
    /* Enable output and complementary output and update dutyCycle*/
    TIM4->BDTR |= 0x8000;              //MOE = 1
    TIM4->CCER |= 0x500;               //CC3NE = 1 and CC3E = 1

    /* Channel3 duty cycle is an input port */
    TIM4->CCR3 = daq_U.pwm5_1 * TIM4->ARR / 100;
  }

  /* Update for S-Function (TIMERS_Config): '<S7>/Timers1' */

  /* Timer frequency is an input port */
  TIM5->ARR = 21000000 / daq_P.pwm4_freq_Value -1;
  if (0 < 0) {
    /* Disable output and complementary output */
    //            TIM5->BDTR &= 0x7FFF;  //MOE = 0
    TIM5->BDTR |= 0x8000;              //MOE = 1
    TIM5->BDTR &= 0xF7FF;              //OSSR = 0

    //            TIM5->CCER |= 0x4;     //CC1NE = 1
    //            TIM5->CCER &= 0xFFFE;  //CC1E = 0
    TIM5->CCER &= 0xFFFA;              //CC1E = 0 CC1NE = 0
    TIM5->CR2 &= 0xFCFF;               //OIS1 = 0 OIS1N = 0
  } else {
    // Enable output and complementary output and update dutyCycle
    TIM5->BDTR |= 0x8000;              //MOE = 1
    TIM5->CCER |= 0x5;                 //CC1NE = 1 and CC1E = 1

    // Channel1 duty cycle is an input port
    TIM5->CCR1 = 0 * TIM5->ARR / 100;
  }

  if (daq_U.pwm5_2 < 0) {
    /* Disable output and complementary output */
    //            TIM5->BDTR &= 0x7FFF;  //MOE = 0
    TIM5->BDTR |= 0x8000;              //MOE = 1
    TIM5->BDTR &= 0xF7FF;              //OSSR = 0

    //            TIM5->CCER |= 0x40;    //CC2NE = 1
    //            TIM5->CCER &= 0xFFEF;  //CC2E = 0
    TIM5->CCER &= 0xFFAF;              //CC2E = 0 CC2NE = 0
    TIM5->CR2 &= 0xF3FF;               //OIS2 = 0 OIS2N = 0
  } else {
    /* Enable output and complementary output and update dutyCycle*/
    TIM5->BDTR |= 0x8000;              //MOE = 1
    TIM5->CCER |= 0x50;                //CC2NE = 1 and CC2E = 1

    /* Channel2 duty cycle is an input port */
    TIM5->CCR2 = daq_U.pwm5_2 * TIM5->ARR / 100;
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.2, which is the step size
   * of the task. Size of "clockTick0" ensures timer will not overflow during the
   * application lifespan selected.
   */
  daq_M->Timing.clockTick0++;
}

/* Model initialize function */
void daq_initialize(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)daq_M, 0,
                sizeof(RT_MODEL_daq));

  /* block I/O */
  (void) memset(((void *) &daq_B), 0,
                sizeof(B_daq));

  /* external inputs */
  (void) memset((void *)&daq_U, 0,
                sizeof(ExtU_daq));

  /* external outputs */
  (void) memset((void *)&daq_Y, 0,
                sizeof(ExtY_daq));

  /* user code (Start function Body) */

  /* ADC1 Initialization */
  ADC1_Init();

#if ( defined (ADC_USE_DMA) || defined (ADC1_USE_DMA) || defined (ADC2_USE_DMA) || defined (ADC3_USE_DMA) )

  /*ADC common DMA configuration*/
  ADC_DMA_Common_Init();

#endif

  /* ADC Initialization */
  ADC_Common_Init();

  /* GPIOA Configuration */
  GPIOA_Configuration();

  /* GPIOB Configuration */
  GPIOB_Configuration();

  /* GPIOC Configuration */
  GPIOC_Configuration();

  /* GPIOD Configuration */
  GPIOD_Configuration();

  /* GPIOE Configuration */
  GPIOE_Configuration();

  /* GPIOF Configuration */
  GPIOF_Configuration();

  /* GPIOG Configuration */
  GPIOG_Configuration();

  /* GPIOH Configuration */
  GPIOH_Configuration();

  /* GPIOI Configuration */
  GPIOI_Configuration();

  /* Usart configuration */
  USART6_Config();

  /* Enable USART6 Rcv interrupt */
  NVIC_USART6_InitStructure.NVIC_IRQChannel = USART6_IRQn ;
  NVIC_USART6_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_USART6_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_USART6_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_USART6_InitStructure);
  USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

  /* Usart configuration */
  USART2_Config();

  /* Enable USART2 Rcv interrupt */
  NVIC_USART2_InitStructure.NVIC_IRQChannel = USART2_IRQn ;
  NVIC_USART2_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_USART2_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_USART2_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_USART2_InitStructure);
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

  /* Usart configuration */
  USART1_Config();

  /* Enable USART1 Rcv interrupt */
  NVIC_USART1_InitStructure.NVIC_IRQChannel = USART1_IRQn ;
  NVIC_USART1_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_USART1_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_USART1_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_USART1_InitStructure);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

  /* Usart configuration */
  USART3_Config();

  /* Enable USART3 Rcv interrupt */
  NVIC_USART3_InitStructure.NVIC_IRQChannel = USART3_IRQn ;
  NVIC_USART3_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_USART3_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_USART3_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_USART3_InitStructure);
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

  /* Start for S-Function (TIMERS_Config): '<S7>/Timers' */
  ;

  /* Start for S-Function (TIMERS_Config): '<S7>/Timers1' */
  ;

  /* Start for S-Function (ADC_Init): '<S1>/ADC_Init' */
  ;
  ;

  /* Start for S-Function (ADC_Common_Init): '<S1>/STM32_ADC_Common_Init' */
  ;

  /* Start for S-Function (GPIOA_Config): '<S3>/GPIOA_Config' */
  ;

  /* Start for S-Function (GPIOB_Config): '<S3>/GPIOB_Config' */
  ;

  /* Start for S-Function (GPIOC_Config): '<S3>/GPIOC_Config' */
  ;

  /* Start for S-Function (GPIOD_Config): '<S3>/GPIOD_Config' */
  ;

  /* Start for S-Function (GPIOE_Config): '<S3>/GPIOE_Config' */
  ;

  /* Start for S-Function (GPIOF_Config): '<S3>/GPIOF_Config' */
  ;

  /* Start for S-Function (GPIOG_Config): '<S3>/GPIOG_Config1' */
  ;

  /* Start for S-Function (GPIOH_Config): '<S3>/GPIOH_Config' */
  ;

  /* Start for S-Function (GPIOI_Config): '<S3>/GPIOI_Config' */
  ;

  /* Start for S-Function (USART_Config): '<S4>/USART_Config' */
  ;

  /* Start for S-Function (USART_Config): '<S4>/USART_Config1' */
  ;

  /* Start for S-Function (USART_Config): '<S4>/USART_Config2' */
  ;

  /* Start for S-Function (USART_Config): '<S4>/USART_Config3' */
  ;

  /* user code (Start function Trailer) */

  /* TIM4 Configuration */
  TIM4_Configuration();

  /* TIM5 Configuration */
  TIM5_Configuration();

  /* ADC1 START */
  ADC1_Start();
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF] daq.c
 */
