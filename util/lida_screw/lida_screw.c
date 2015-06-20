#include <stdio.h>
#include "stddef.h"
#include "stm32f4xx.h"
#include "daq.h"
#include "vn100/vn100.h"
#include "rtwtypes.h"
#include "lida_screw.h"
/*******************************************************************************
* Function Name  : PWM_IC_init
* Input          : 
* Output         : 
* Return         : 
* Description    : 
*******************************************************************************/

void PWM_IC_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	/* TIM2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	/* GPIOB clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  
	/* TIM2 chennel2 configuration : PB.03 */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
  
	/* Connect TIM pin to AF2 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);

	/* Enable the TIM2 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/*TIM2 Config*/
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;

	TIM_PWMIConfig(TIM2, &TIM_ICInitStructure);

	/* Select the TIM2 Input Trigger: TI2FP2 */
	TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);

	/* Select the slave Mode: Reset Mode */
	TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
	TIM_SelectMasterSlaveMode(TIM2,TIM_MasterSlaveMode_Enable);

	/* TIM enable counter */
	TIM_Cmd(TIM2, ENABLE);

	/* Enable the CC2 Interrupt Request */
	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);

}

/*******************************************************************************
* Function Name  : PID_generic
* Input          : PID-related parameter structure: param; 
				   input error: error; 
				   pointer to the PID-related data structure: PID_data;
* Output         : Updating PID_data
* Return         : controlled value
* Description    : This function is a generic PID controller with limit on integral
*******************************************************************************/
int16_t PID_generic (PID_param_t param, PID_data_t* data, int32_t measured, int32_t expected)
{
	int32_t error = expected - measured;

	int32_t D_error = error - (data->prev_error);
	data->prev_error = error;

	if(param.Ki != 0)
	{
		data->integral += error;
		if(data->integral >= param.integral_max)
		{
			data->integral = param.integral_max;
		}
		if(data->integral <= param.integral_min)
		{
			data->integral = param.integral_min;
		}
		return (int16_t)(param.Kp*error + param.Ki*(data->integral) + param.Kd*D_error);
	}

	return (int16_t)(param.Kp*error + param.Kd*D_error);
}

/*******************************************************************************
* Function Name  : screw_init
* Input          : 
* Output         : 
* Return         : 
* Description    : 
*******************************************************************************/
void screw_init(screw_position_t* position)
{
	//uint16_t read_temp;

	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN ;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_SetBits(GPIOD, GPIO_Pin_15);
	daq_U.pwm5_3 = 30;
	
	while(!GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_13));
	daq_U.pwm5_3 = 0;
	
	while((read_temp = MX_I2C_READ()) == 65535)
	{
		MX_I2C_Init();
	}
	position->range = read_temp;
	GPIO_ResetBits(GPIOD, GPIO_Pin_15);
	daq_U.pwm5_3 = 30;
	
	while(!GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_14))
	{
		while((read_temp = MX_I2C_READ()) == 65535)
		{
			MX_I2C_Init();
		}
		if(read_temp > 180+position->degree)
			position->range += 360;
		position->degree = read_temp;
	}
	daq_U.pwm5_3 = 0;
	GPIO_SetBits(GPIOD, GPIO_Pin_15);
	position->range -= position->degree;
	position->round = 0;
	position->init_degree = position->degree;
}

/*******************************************************************************
* Function Name  : MX_I2C_Init
* Input          : None
* Output         : None
* Return         : 
* Description    : I2C Initialization
*******************************************************************************/
void MX_I2C_Init(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef GPIO_InitB;
	GPIO_InitB.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitB.GPIO_OType = GPIO_OType_OD;
	GPIO_InitB.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitB.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitB.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitB);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);

	I2C_InitTypeDef I2C_Init1;
	I2C_Init1.I2C_Ack = I2C_Ack_Disable;
	I2C_Init1.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init1.I2C_ClockSpeed = 100000;
	I2C_Init1.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_Init1.I2C_Mode = I2C_Mode_I2C;
	I2C_Init1.I2C_OwnAddress1 = 0;

	I2C_Init(I2C1, &I2C_Init1);
}


/*******************************************************************************
* Function Name  : MX_I2C_READ
* Input          : None
* Output         : Angle Measure
* Return         :
* Description    : I2C Read Angle Measure
*******************************************************************************/
uint16_t MX_I2C_READ(void){

	uint8_t HIGH = 0x00;
	uint8_t LOW = 0x00;
	uint16_t temp1 = 0x0000;
	uint16_t temp2 = 0x0000;
	uint16_t ANGLE = 0x0000;

	int counter = 0;
	int FAILFLAG = 0;
	int WAITTIME = 1000;

	//Read Angle Information from I2C Registers
	// Angle[6-13]
	I2C_GenerateSTART(I2C1, ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)){
		counter++;
		if (counter == WAITTIME){
			FAILFLAG = 1;
			break;
		}
	}
	counter = 0;
	I2C_Send7bitAddress(I2C1, 0x80, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
		counter++;
		if (counter == WAITTIME){
			FAILFLAG = 1;
			break;
		}
	}
	counter = 0;
	I2C_SendData(I2C1, 0xFE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
		counter++;
		if (counter == WAITTIME){
			FAILFLAG = 1;
			break;
		}
	}
	counter = 0;
	I2C_GenerateSTART(I2C1, ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)){
		counter++;
		if (counter == WAITTIME){
			FAILFLAG = 1;
			break;
		}
	}
	counter = 0;
	I2C_Send7bitAddress(I2C1, 0x80, I2C_Direction_Receiver);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){
		counter++;
		if (counter == WAITTIME){
			FAILFLAG = 1;
			break;
		}
	}
	counter = 0;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)){
		counter++;
		if (counter == WAITTIME){
			FAILFLAG = 1;
			break;
		}
	}
	counter = 0;
	HIGH = I2C_ReceiveData(I2C1);
	I2C_GenerateSTOP(I2C1, ENABLE);

	// Angle[0-5]
	I2C_GenerateSTART(I2C1, ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)){
		counter++;
		if (counter == WAITTIME){
			FAILFLAG = 1;
			break;
		}
	}
	counter = 0;
	I2C_Send7bitAddress(I2C1, 0x80, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
		counter++;
		if (counter == WAITTIME){
			FAILFLAG = 1;
			break;
		}
	}
	counter = 0;
	I2C_SendData(I2C1, 0xFF);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
		counter++;
		if (counter == WAITTIME){
			FAILFLAG = 1;
			break;
		}
	}
	counter = 0;
	I2C_GenerateSTART(I2C1, ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)){
		counter++;
		if (counter == WAITTIME){
			FAILFLAG = 1;
			break;
		}
	}
	counter = 0;
	I2C_Send7bitAddress(I2C1, 0x80, I2C_Direction_Receiver);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){
		counter++;
		if (counter == WAITTIME){
			FAILFLAG = 1;
			break;
		}
	}
	counter = 0;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)){
		counter++;
		if (counter == WAITTIME){
			FAILFLAG = 1;
			break;
		}
	}
	counter = 0;

	LOW = I2C_ReceiveData(I2C1);
	I2C_GenerateSTOP(I2C1, ENABLE);

	//Data Manipulation
	temp1 = ((uint16_t)HIGH);
	temp1 = (temp1 << 6) & 0x3FC0;
	temp2 = (uint16_t)LOW;
	temp2 = temp2 & 0x003F;
	ANGLE = temp1 | temp2;

	if (FAILFLAG){
		return -1;
	}
	//Return Angle
	return ANGLE / 45.52;
}