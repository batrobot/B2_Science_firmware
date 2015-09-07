/*
 * File: pca9626.h
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
 * ******************************************************************************/
#ifndef PCA9626_H_
#define PCA9626_H_

/* includes */
#include <stdio.h>
#include "stddef.h"
#include "stm32f4xx.h"
#include "daq.h"
#include "rtwtypes.h"

/* Macros that used to initialize I2C for PCA9626 (on MCB)*/
#define PCA9626_RCC_APBPeriph RCC_APB1Periph_I2C2
#define PCA9626_RCC_AHBPeriph_GPIO RCC_AHB1Periph_GPIOH
#define PCA9626_GPIO_Init GPIO_InitH
#define PCA9626_I2C_Initx I2C_Init2
#define PCA9626_GPIO GPIOH
#define PCA9626_GPIO_Pin_CLK GPIO_Pin_4
#define PCA9626_GPIO_Pin_SDA GPIO_Pin_5
#define PCA9626_GPIO_PinSourceCLK GPIO_PinSource4
#define PCA9626_GPIO_PinSourceSDA GPIO_PinSource5
#define PCA9626_GPIO_Speed GPIO_Speed_100MHz
#define PCA9626_GPIO_AF_I2Cx GPIO_AF_I2C2
#define PCA9626_I2Cx I2C2
#define PCA9626_CLOCKSPEED 100000

/* pca9626 on mcb: bus address */
#define PCA9626_MCB_ADR 0x60 
/* pca9626 on interface board: bus address */
#define PCA9626_INT_ADR 0x00 

#define PCA9626_LED_ALLCALL_ADR 0xE0
#define PCA9626_LED_SUBCALL1_ADR 0xE2
#define PCA9626_LED_SUBCALL2_ADR 0xE4
#define PCA9626_LED_SUBCALL3_ADR 0xE6
#define PCA9626_SOFTWARE_RESET_ADR 0x06

/* following acknowledgement of slave add, the byte that is sent to slave is stored in Control register */



#endif