/*
 * File: debug.h
 *
 * Model version      : 
 * C/C++ source code generated on  : 
 *
 * Target selection: stm32F4xx.tlc
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

#ifndef DEBUG_h_
#define DEBUG_h_

#include "rtwtypes.h"
#include <stdio.h>

/* debuging defs */
#define _REC_SD_CARD	// buffer data on SD card
//#define _BLUETOOTH		// buffer data over bluetooth


/* Tuning increments */
#define INC_KP					10.0
#define INC_KD					0.1
#define INC_KI					5.0
#define INC_POS					10
#define INC_POS_TOL				0.01
#define INC_AWU					100
#define INC_PWM_CMD				5
#define INC_FLIGHT_PARAM		0.5
#define INC_ANGLE_MAX_MIN		1

/* global vars */


/* global functions */
void debug_write_params(void);
void debug_write_data(void);
void debug_initialize_files(void);
boolean_T debug_check_bluetooth(void);
void debug_bat_robot(void);



#endif