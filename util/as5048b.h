/*
 * File: as5048.h
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
#ifndef AS5048B_H_
#define AS5048B_H_

/* includes */
#include <stdio.h>
#include "stddef.h"
#include "stm32f4xx.h"
#include "daq.h"
#include "rtwtypes.h"

/* Macros */
#define AS5048B_RCC_APBPeriph RCC_APB1Periph_I2C1
#define AS5048B_RCC_AHBPeriph_GPIO RCC_AHB1Periph_GPIOB
#define AS5048B_GPIO_Init GPIO_InitB
#define AS5048B_I2C_Initx I2C_Init1
#define AS5048B_GPIO GPIOB
#define AS5048B_GPIO_Pin_CLK GPIO_Pin_6
#define AS5048B_GPIO_Pin_SDA GPIO_Pin_7
#define AS5048B_GPIO_PinSourceCLK GPIO_PinSource6
#define AS5048B_GPIO_PinSourceSDA GPIO_PinSource7
#define AS5048B_GPIO_Speed GPIO_Speed_100MHz
#define AS5048B_GPIO_AF_I2Cx GPIO_AF_I2C1
#define AS5048B_I2Cx I2C1
#define AS5048B_CLOCKSPEED 100000

/* MACROS */
#define PI 3.141592654
#define DEG2RAD(x) x*PI/180
#define RAD2DEG(x) x*180/PI
#define TIME_OUT_DELAY 1000 // time out delay for I2C talks

// Default addresses for AS5048B
#define AS5048B_ADDRESS_FACTORY 0x80 // 10000 + ( A1 & A2 to GND)
#define AS5048B_ADDRESS_A 0x81 // 10000 + ( A1 High & A2 to GND)
#define AS5048B_ADDRESS_B 0x82 // 10000 + ( A1 GND & A2 to High)
#define AS5048B_ADDRESS_C 0x83 // 10000 + ( A1 High & A2 to High)
#define AS5048B_PROG_REG 0x03
#define AS5048B_ADDR_REG 0x15
#define AS5048B_ZEROMSB_REG 0x16 //bits 0..7
#define AS5048B_ZEROLSB_REG 0x17 //bits 0..5
#define AS5048B_GAIN_REG 0xFA
#define AS5048B_DIAG_REG 0xFB
#define AS5048B_MAGNMSB_REG 0xFC //bits 0..7
#define AS5048B_MAGNLSB_REG 0xFD //bits 0..5
#define AS5048B_ANGLMSB_REG 0xFE //bits 0..7
#define AS5048B_ANGLLSB_REG 0xFF //bits 0..5
#define AS5048B_RESOLUTION 16384.0 //14 bits

//unit consts - just to make the units more readable
#define U_RAW 1
#define U_TRN 2
#define U_DEG 3
#define U_RAD 4
#define U_GRAD 5
#define U_MOA 6
#define U_SOA 7
#define U_MILNATO 8
#define U_MILSE 9
#define U_MILRU 10

/* externs vars */
extern boolean_T TIME_OUT_ERR;
extern uint8_t _chipAddress;
extern boolean_T _clockWise;


/* routines */
void AS5048B_initialize_I2C(void);
void AS5048B_readBodyAngles(uint16_t *angleReg, uint8_t *autoGain, uint8_t *diag, uint16_t *magnitude, double *angle);
void AS5048B_initialize(void);
void AS5048B_setClockWise(boolean_T cw); 
void AS5048B_addressRegW(uint8_t regVal); 
uint8_t	AS5048B_addressRegR(void); 
void AS5048B_setZeroReg(void); 
void AS5048B_zeroRegW(uint16_t regVal); 
uint16_t AS5048B_zeroRegR(void); 
uint16_t AS5048B_angleRegR(void); 
uint8_t	AS5048B_getAutoGain(void); 
uint8_t	AS5048B_getDiagReg(void); 
uint16_t AS5048B_magnitudeR(void); 
double AS5048B_angleR(int unit, boolean_T newVal); 











#endif