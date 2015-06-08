/******************** (C) COPYRIGHT 2009 STMicroelectronics ********************
* File Name          : stm32f4xx_conf.h
* Author             : MCD Application Team
* Version            : V0.0.1
* Date               : 06/08/2009
* Description        : Library configuration file.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __stm32f4xx_CONF_H
#define __stm32f4xx_CONF_H

/* Includes ------------------------------------------------------------------*/
/* Uncomment the line below to enable peripheral header file inclusion */
/* #include "stm32f4xx_bkp.h" */
 #include "stm32f4xx_can.h" 
/* #include "stm32f4xx_crc.h" */
/* #include "stm32f4xx_dac.h" */
/* #include "stm32f4xx_dbgmcu.h" */
#include "stm32f4xx_dma.h" 
#include "stm32f4xx_exti.h"
#include "stm32f4xx_flash.h" 
//#include "stm32f4xx_fsmc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h" 
/*#ifdef stm32f4xx
#include "stm32f4xx_i2c.h" 
#else
#include "stm32f4xx_adc.h"
#endif*/
#include "stm32f4xx_adc.h"
/* #include "stm32f4xx_iwdg.h" */
#include "stm32f4xx_pwr.h" 
#include "stm32f4xx_rcc.h"
/* #include "stm32f4xx_rtc.h" */
#include "stm32f4xx_sdio.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_tim.h" 
#include "stm32f4xx_usart.h"
/* #include "stm32f4xx_wwdg.h" */
#include "stm32f4xx_dma2d.h"
#include "stm32f4xx_ltdc.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h" /* High level functions for NVIC and SysTick (add-on to CMSIS functions) */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Uncomment the line below to expanse the "assert_param" macro in the 
   Standard Peripheral Library drivers code */
/* #define USE_FULL_ASSERT    1 
//#define UCB

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/*******************************************************************************
* Macro Name     : assert_param
* Description    : The assert_param macro is used for function's parameters check.
*                  It is used only if the library is compiled in DEBUG mode. 
* Input          : - expr: If expr is false, it calls assert_failed function
*                    which reports the name of the source file and the source
*                    line number of the call that failed. 
*                    If expr is true, it returns no value.
* Return         : None
*******************************************************************************/ 
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((u8 *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(u8* file, u32 line);
#else
  //#define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#endif /* __stm32f4xx_CONF_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
