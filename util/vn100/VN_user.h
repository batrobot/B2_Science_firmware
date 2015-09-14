/***************** (C) COPYRIGHT 2009 VectorNav Technologies *******************
* File Name          : VN_user.h
* Author             : John Brashear
* Version            : V1.0.0
* Date               : 09/26/2009
* Description        : This is the user configuration header file. It contains
*                    : setup parameters and all the function prototypes for the 
*                    : methods that are hardware specific. These methods need to
*                    : be modified by the user in the file VN_user.c to be
*                    : compatible with their specific hardware architecture.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
* CUSTOMERS WITH EXAMPLE CODE IN ORDER TO SAVE THEM TIME. AS A RESULT,
* VECTORNAV SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR 
* CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE 
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE 
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VN_USER_H
#define __VN_USER_H 

/* Includes ------------------------------------------------------------------*/
#include "VN_type.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Comment the lines below to disable the specific device inclusion */

/*********************************** VN-100 ***********************************/
#define _VN100

/* Timer */
#define VN100_TIM TIM7
#define VN100_RCC_APBPeriph RCC_APB1Periph_TIM7

/* vn100 SPI Interface pins */
#define VN100_SPI                       SPI4
#define VN100_SPI_CLK                   RCC_APB2Periph_SPI4

#define VN100_SPI_SCK_PIN               GPIO_Pin_2                  /* PE.02 */
#define VN100_SPI_SCK_GPIO_PORT         GPIOE                       /* GPIOE */
#define VN100_SPI_SCK_GPIO_CLK          RCC_AHB1Periph_GPIOE
#define VN100_SPI_SCK_SOURCE            GPIO_PinSource2
#define VN100_SPI_SCK_AF                GPIO_AF_SPI4

#define VN100_SPI_MISO_PIN              GPIO_Pin_5                  /* PE.5 */
#define VN100_SPI_MISO_GPIO_PORT        GPIOE                       /* GPIOE */
#define VN100_SPI_MISO_GPIO_CLK         RCC_AHB1Periph_GPIOE
#define VN100_SPI_MISO_SOURCE           GPIO_PinSource5
#define VN100_SPI_MISO_AF               GPIO_AF_SPI4

#define VN100_SPI_MOSI_PIN              GPIO_Pin_6                  /* PE.06 */
#define VN100_SPI_MOSI_GPIO_PORT        GPIOE                       /* GPIOE */
#define VN100_SPI_MOSI_GPIO_CLK         RCC_AHB1Periph_GPIOE
#define VN100_SPI_MOSI_SOURCE           GPIO_PinSource6
#define VN100_SPI_MOSI_AF               GPIO_AF_SPI4

#define VN100_SPI_CS_PIN                GPIO_Pin_12                  /* PI.00 */
#define VN100_SPI_CS_GPIO_PORT          GPIOH                       /* GPIOI */
#define VN100_SPI_CS_GPIO_CLK           RCC_AHB1Periph_GPIOH


/* Chip select macros */
#define VN100_CSSetBits GPIO_SetBits(VN100_SPI_CS_GPIO_PORT,VN100_SPI_CS_PIN)
#define VN100_CSResetBits GPIO_ResetBits(VN100_SPI_CS_GPIO_PORT,VN100_SPI_CS_PIN)


/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
#define VN100_FLAG_TIMEOUT             ((uint32_t)0x100)			


/* Exported functions ------------------------------------------------------- */
void VN_SPI_SetSS(unsigned char sensorID, VN_PinState state);
unsigned long VN_SPI_SendReceive(unsigned long data);
void VN_Delay(unsigned long delay_uS);
void SPI_initialize(void);		

#endif /* __VN_USER_H */

/******************* (C) COPYRIGHT 2009 VectorNav Technologies *****************
***********************************END OF FILE*********************************/
