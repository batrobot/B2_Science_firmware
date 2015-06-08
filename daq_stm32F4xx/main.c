/*
 * File: main.c
 *
 * Code generated for Simulink model :daq.
 *
 * Model version      : 1.18
 * Simulink Coder version    : 8.6 (R2014a) 27-Dec-2013
 * TLC version       : 8.6 (Jan 30 2014)
 * C/C++ source code generated on  : Sun Jun 07 16:10:57 2015
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

#include <stdio.h>
#include "daq.h"                       /* Model's header file */
#include "rtwtypes.h"                  /* MathWorks types */
#include "stm32f4xx.h"

/* RCC_Configuration defined in RCC_Configuration.c file */
/* Clock configuration function provided as example */
extern void RCC_Configuration(void);

/* Real-time model */
extern RT_MODEL_daq *const daq_M;

/* Set which subrates need to run this base step (base rate always runs).*/
/* Defined in daq.c file */
extern void daq_SetEventsForThisBaseStep(boolean_T*);

/* Flags for taskOverrun */
static boolean_T OverrunFlags[1];

/* Number of auto reload timer rotation computed */
static uint32_t autoReloadTimerLoopVal_S = 1;

/* Remaining number of auto reload timer rotation to do */
static uint32_t remainAutoReloadTimerLoopVal_S = 1;

/****************************************************
   SysTick_Handler function
   This handler is called every tick and schedules tasks
 *****************************************************/
void SysTick_Handler(void);
void SysTick_Handler(void)
{
  /* Manage nb of loop before interrupt has to be processed */
  remainAutoReloadTimerLoopVal_S--;
  if (remainAutoReloadTimerLoopVal_S) {
    return;
  }

  remainAutoReloadTimerLoopVal_S = autoReloadTimerLoopVal_S;

  /* Check base rate for overrun */
  if (OverrunFlags[0]) {
    rtmSetErrorStatus(daq_M, "Overrun");
    return;
  }

  OverrunFlags[0] = TRUE;

  /* Step the model for base rate */
  daq_step();

  /* Get model outputs here */

  /* Indicate task for base rate complete */
  OverrunFlags[0] = FALSE;
}

/****************************************************
   main function
   Example of main :
   - Clock configuration
   - call Initialize
   - Wait for systick (infinite loop)
 *****************************************************/
int main (void)
{
  /* Data initialization */
  int_T i;
  for (i=0;i<1;i++) {
    OverrunFlags[i] = 0;
  }

  /* Clock has to be configured first*/
  RCC_Configuration();

  /* Model initialization call */
  daq_initialize();

  /* Systick configuration and enable SysTickHandler interrupt */
  if (SysTick_Config((uint32_t)(SystemCoreClock * 0.001))) {
    autoReloadTimerLoopVal_S = 1;
    do {
      autoReloadTimerLoopVal_S++;
    } while ((uint32_t)(SystemCoreClock * 0.001)/autoReloadTimerLoopVal_S >
             SysTick_LOAD_RELOAD_Msk);

    SysTick_Config((uint32_t)(SystemCoreClock * 0.001)/autoReloadTimerLoopVal_S);
  }

  remainAutoReloadTimerLoopVal_S = autoReloadTimerLoopVal_S;//Set nb of loop to do

  /* Infinite loop */
  /* Real time from systickHandler */
  while (1) {
    /* Add code here */
  }
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF] main.c
 */
