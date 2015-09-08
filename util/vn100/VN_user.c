/***************** (C) COPYRIGHT 2009 VectorNav Technologies *******************
* File Name          : VN_user.c
* Author             : John Brashear
* Version            : V1.0.0
* Date               : 09/26/2009
* Description        : This file contains all the functions that are hardware 
*                    : specific. These functions need to be modified by the 
*                    : user to be compatible with their hardware architecture.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
* CUSTOMERS WITH EXAMPLE CODE IN ORDER TO SAVE THEM TIME. AS A RESULT,
* VECTORNAV SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR 
* CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE 
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE 
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
//#include "stm32f10x_lib.h"
#include "VN_user.h"
#include "VN_lib.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
__IO uint32_t  VN100_Timeout = VN100_FLAG_TIMEOUT;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
uint8_t VN100_transcieve_byte(uint8_t data);
double VN100_convert2milimeter(uint16_t value, uint8_t shift, double sensitivity);
void VN100_delay_us(unsigned long us);
void VN100_delay_ms(unsigned long ms);
void VN100_initialize_timer(void);


/*******************************************************************************
	* Function Name  : VN100_initialize
	* Input          : 
	* Output         : None
	* Return         : 
	* Description    : M
	*******************************************************************************/
void SPI_initialize(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;


			/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(VN100_SPI_SCK_GPIO_CLK, ENABLE);

			/* GPIO default Value */
	//GPIO_DeInit(VN100_SPI_CS_GPIO_PORT);
	GPIO_DeInit(VN100_SPI_SCK_GPIO_PORT);
	//GPIO_DeInit(VN100_SPI_MISO_GPIO_PORT);
	GPIO_DeInit(VN100_SPI_MOSI_GPIO_PORT);

//
	//GPIO_InitStructure.GPIO_Pin = VN100_SPI_CS_PIN;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//GPIO_Init(VN100_SPI_CS_GPIO_PORT, &GPIO_InitStructure);
//
	GPIO_InitStructure.GPIO_Pin = VN100_SPI_SCK_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(VN100_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(VN100_SPI_SCK_GPIO_PORT, VN100_SPI_SCK_SOURCE, VN100_SPI_SCK_AF);
	RCC_APB2PeriphClockCmd(VN100_SPI_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = VN100_SPI_MISO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(VN100_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(VN100_SPI_MISO_GPIO_PORT, VN100_SPI_MISO_SOURCE, VN100_SPI_MISO_AF);
	RCC_APB2PeriphClockCmd(VN100_SPI_CLK, ENABLE);


	GPIO_InitStructure.GPIO_Pin = VN100_SPI_MOSI_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(VN100_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(VN100_SPI_MOSI_GPIO_PORT, VN100_SPI_MOSI_SOURCE, VN100_SPI_MOSI_AF);
	RCC_APB2PeriphClockCmd(VN100_SPI_CLK, ENABLE);


			/*SPI initialization*/
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                 
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;   
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(VN100_SPI, &SPI_InitStructure);

			/*Enable SPI*/
	SPI_Cmd(VN100_SPI, ENABLE);

			/* Deselect : Chip Select high */
	GPIO_ResetBits(VN100_SPI_CS_GPIO_PORT, VN100_SPI_CS_PIN);


			/*TIM7 initialization*/
	VN100_initialize_timer();
	VN100_delay_ms(10);

}

/*******************************************************************************
	* Function Name  : VN100_transcieve_byte
	* Input          : 
	* Output         : None
	* Return         : 
	* Description    : 
	*******************************************************************************/

uint8_t VN100_transcieve_byte(uint8_t data) {

		/*Write data to be transmitted to the SPI data register*/
	VN100_SPI->DR = data; 
		
	/*Wait until transmit complete*/
	VN100_Timeout = VN100_FLAG_TIMEOUT;
	while (!(VN100_SPI->SR & SPI_I2S_FLAG_TXE)) {
		if ((VN100_Timeout--) == 0) return 0;
	} 
		
	/*Wait until receive complete*/
	VN100_Timeout = VN100_FLAG_TIMEOUT;
	while (!(VN100_SPI->SR & SPI_I2S_FLAG_RXNE)) {
		if ((VN100_Timeout--) == 0) return 0;
	}
		
	/*Wait until SPI is not busy anymore*/
	VN100_Timeout = VN100_FLAG_TIMEOUT;
	while (VN100_SPI->SR & SPI_I2S_FLAG_BSY) {
		if ((VN100_Timeout--) == 0) return 0;
	} 

			/*Return received data from SPI data register*/
	return VN100_SPI->DR; 
}



/*******************************************************************************
* Function Name  : VN_SPI_SetSS(unsigned char sensorID, bool LineState)
* Description    : This is a generic function that will set the SPI slave select
*                  line for the given sensor. This function needs to be added by
*                  the user with the logic specific to their hardware to perform
*                  the necessary actions to either raise or lower the slave
*                  select line for the given sensor.  If a multiplexer is used
*                  then the logic/communications neccessary to perform the
*                  actions should be placed here.                                        
* Input          : sensorID  -> The sensor to set the slave select line for.
*                : state -   -> The state to set the slave select to.
* Output         : None
* Return         : None
*******************************************************************************/
void VN_SPI_SetSS(unsigned char sensorID, VN_PinState state) {

if (state == VN_PIN_LOW) {
			/* Start SPI Transaction - Pull SPI CS line low */
	VN100_CSResetBits;
}
else {
	/* End SPI transaction - Pull SPI CS line high */
	VN100_CSSetBits;
}
}

/*******************************************************************************
* Function Name  : VN_SPI_SendReceiveWord(unsigned long data)
* Description    : Transmits the given 32-bit word on the SPI bus. The user needs
*                  to place their hardware specific logic here to send 4 bytes
*                  out the SPI bus. The slave select line is controlled by the 
*                  function VN_SPI_SetSS given above, so the user only needs
*                  to deal with sending the data out the SPI bus with this
*                  function.
* Input          : data -> The 32-bit data to send over the SPI bus
* Output         : None
* Return         : The data received on the SPI bus
*******************************************************************************/
unsigned long VN_SPI_SendReceive(unsigned long data) {

/* User code to send out 4 bytes over SPI goes here */
	unsigned long i;
	unsigned long ret = 0;
  
	for (i = 0;i < 4;i++) {
	  /* Wait for VN100_SPI Tx buffer empty */
		while (SPI_I2S_GetFlagStatus(VN100_SPI, SPI_I2S_FLAG_TXE) == RESET)
			;
  
		/* Send VN100_SPI requests */
		SPI_I2S_SendData(VN100_SPI, VN_BYTE(data, i));
  
		/* Wait for response from VN-100 */
		while (SPI_I2S_GetFlagStatus(VN100_SPI, SPI_I2S_FLAG_RXNE) == RESET)
			;

			/* Save received data in buffer */
		ret |= ((unsigned long)SPI_I2S_ReceiveData(VN100_SPI) << (8*i));    
	}
  
	return ret;
}

/*******************************************************************************
* Function Name  : VN_Delay(unsigned long delay_uS)
* Description    : Delay the processor for deltaT time in microseconds.  The user
*                  needs to place the hardware specific code here necessary to 
*                  delay the processor for the time span given by delay_uS
*                  measured in micro seconds. This function doesn't need to be
*                  ultra precise. The only requirement on this function is that
*                  the processor is delayed a time NO LESS THAN 90% of the time 
*                  given by the variable delay_uS in microseconds. The minimum
*                  timespan that is used by the VectorNav library code is 50uS so
*                  the function call shouldn't affect the timing accuracy much.
*                  If you decide to modify this library or wish to have more
*                  precision on this delay function then you can comment out this
*                  function and replace it with an optimized macro instead. Many
*                  compilers have their own delay routines or macros so make sure
*                  you check your compiler documentation before attempting to
*                  write your own.
* Input          : delay_uS -> Time to delay the processor in microseconds
* Output         : None
* Return         : None
*******************************************************************************/
void VN_Delay(unsigned long delay_uS) {

	VN100_delay_us(delay_uS);
}

/*******************************************************************************
	* Function Name  : VN100_delay_us
	* Input          : 
	* Output         : None
	* Return         : 
	* Description    : 
	*******************************************************************************/
void VN100_delay_us(unsigned long us)
{
	uint16_t t0 = VN100_TIM->CNT;
	for (;;) {
		int t = VN100_TIM->CNT;
		if (t < t0)
			t += 0x10000;

		if (us < t - t0)
			return;

		us -= t - t0;
		t0 = t;
	}
}


	/*******************************************************************************
	* Function Name  : VN100_delay_ms
	* Input          : 
	* Output         : None
	* Return         : 
	* Description    : 
	*******************************************************************************/
void VN100_delay_ms(unsigned long ms)
{
	VN100_delay_us(ms * 1000);
}



	/*******************************************************************************
	* Function Name  : VN100_initialize_timer
	* Input          : 
	* Output         : None
	* Return         : 
	* Description    : 
	*******************************************************************************/
void VN100_initialize_timer()
{
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);

	RCC->APB1ENR |= VN100_RCC_APBPeriph;
	VN100_TIM->PSC = (RCC_Clocks.PCLK1_Frequency / 1000000) - 1;
	VN100_TIM->ARR = 0xFFFF;
	VN100_TIM->CR1 = TIM_CR1_CEN;
}



/******************* (C) COPYRIGHT 2009 VectorNav Technologies *****************
***********************************END OF FILE*********************************/
