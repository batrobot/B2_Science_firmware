 /*
 * File: interface_board.h
 *
 * Model version      : 
 * C/C++ source code written by  :  8-28-2015, Alireza Ramezani, Champaign, IL
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
#ifndef INTERFACE_BOARD_H_
#define INTERFACE_BOARD_H_

/* includes */
#include "daq.h"
#include "daq_private.h"

/* timer configuration params */
#define TIMER2_COUNTER_CLOCK_FREQ  1312500
#define TIMER3_COUNTER_CLOCK_FREQ  1312500
#define TIMER9_COUNTER_CLOCK_FREQ  1312500

/* PWM output frequency */
#define TIMER2_PWM_FREQ  1000
#define TIMER3_PWM_FREQ  1000
#define TIMER9_PWM_FREQ  500


/* PWM duty cicle to PA3, PB1, PE5, PE6 */
extern int16_T _pwm_pa3;
extern int16_T _pwm_pb1;
extern int16_T _pwm_pe5;
extern int16_T _pwm_pe6;

/* Public functions */
void INTERFACE_BOARD_initialize(void);
void INTERFACE_BOARD_TIM2_Configuration(void);
void INTERFACE_BOARD_TIM3_Configuration(void);
void INTERFACE_BOARD_TIM9_Configuration(void);
void INTERFACE_BOARD_pwmGenerator(void);

#endif