/*
 * File: debug.cpp
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

/**/

#include "debug.h"
#include "stm32f4xx_conf.h"
#include "daq.h"


#define DEBUG_NbRcv daq__Y.usart2_NbRcv
#define DEBUG_RcvVal daq_Y.usart2_RcvVal

/**/
#define DEBUG_Nb2Send daq_U.usart2_Nb2Send
#define DEBUG_SendVal daq_U.usart2_SendVal


/*******************************************************************************
* Function Name  : debug_printf
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

uint8_T debug_printf(const char *buff, unsigned int buffLength){
	uint8_T i;
	
	for(i=0; i<buffLength; i++){
		DEBUG_SendVal[i] = buff[i];
	}
	DEBUG_Nb2Send = buffLength;
	return  buffLength;

}



