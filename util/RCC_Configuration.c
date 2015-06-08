/*
 * File: RCC_Configuration.c
 *
 * Code example for RCC clock configuration
 *
 *
 * Target : stm32F4xx
 * Embedded hardware selection: STMicroelectronics->STM32F4xx 32-bit Cortex-M4
 * Validation result: Not run
 *
 * Configuration :
*   168MHz clock
 *  Based on HSI 16MHz  
 */

#include "stm32f4xx.h"

/* Function Declaration for RCC Configuration */
void RCC_Configuration(void);

/*Global variable used for Timers clock configuration */
uint32_t RCC_APB1_Prescaler = RCC_HCLK_Div1;
uint32_t RCC_APB2_Prescaler = RCC_HCLK_Div2;

/*******************************************************************************
 * Function Name  : RCC_Configuration
 * Description    : System Clocks Configuration
 * Input          : -
 *******************************************************************************/
void RCC_Configuration(void)
{
  ErrorStatus HSEStartUpStatus = SUCCESS;

  //--------------------------------------------------------
  //             CLK configuration (168.0MHz)
  //--------------------------------------------------------
  RCC_DeInit();

  /* %warning "---------->Start RCC_Init Clock source : HSI " */
  // Enable HSI (High Speed Internal Oscillation)
  RCC_HSICmd(ENABLE);

  // Wait for stable HSI clk
  while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET) ;

  // is HSE/HSI stable?
  if (HSEStartUpStatus != ERROR) {
    // HSE/HSI stable
    /* Voltage range V2v7_to_3v6 */
    // Flash 5 wait states
    FLASH_SetLatency(FLASH_Latency_5);
  }

  // HCLK Config AHB prescaler
  RCC_HCLKConfig(RCC_SYSCLK_Div1);

  // PCLK1 Config APB1 prescaler
  RCC_PCLK1Config(RCC_APB1_Prescaler);

  // PCLK2 Config APB2 prescaler
  RCC_PCLK2Config(RCC_APB2_Prescaler);
  
  // PLL configuration
  RCC_PLLConfig(RCC_PLLSource_HSI, 8, 168, 2, 7);
  // Enable PLL
  RCC_PLLCmd(ENABLE);
  // Wait till PLL is ready - PLL_RDY @ bit 25
  while ((RCC->CR & (1<<25)) == 0) ;
  // Select PLL as system clock source - RCC_SYSCLKSource_PLLCLK
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
  // Wait till PLL is used as system clock source
  while (RCC_GetSYSCLKSource() != 0x08) ;

  // SYSCFG Periph clock enable
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  // Enable PWR and BKP clock
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  // Enable access to the backup register, so LSE can be enabled
  PWR_BackupAccessCmd(ENABLE);

  //Update system core clock value
  SystemCoreClockUpdate();
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF] RCC_Process.c
 */
