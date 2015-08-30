/*
 * File: as5048.c
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
 * ******************************************************************************
 */
/* includes */
#include "as5048b.h"


/* Private functions */
uint8_t		AS5048B_readReg8(uint8_t address);
void		AS5048B_writeReg(uint8_t address, uint8_t value);
double		AS5048B_convertAngle(int unit, double angle); 

/* externs vars */
uint8_t _chipAddress = AS5048B_ADDRESS_FACTORY;
boolean_T TIME_OUT_ERR = false;
boolean_T _clockWise = false;


/*******************************************************************************
* Function Name  : AS5048B_initialize_I2C
* Input          : None
* Output         : None
* Return         : 
* Description    : I2C Initialization
*******************************************************************************/
void AS5048B_initialize_I2C(void){
	RCC_APB1PeriphClockCmd(AS5048B_RCC_APBPeriph, ENABLE);

	RCC_AHB1PeriphClockCmd(AS5048B_RCC_AHBPeriph_GPIO, ENABLE);

	GPIO_InitTypeDef AS5048B_GPIO_Init;
	AS5048B_GPIO_Init.GPIO_Mode = GPIO_Mode_AF;
	AS5048B_GPIO_Init.GPIO_OType = GPIO_OType_OD;
	AS5048B_GPIO_Init.GPIO_Pin = AS5048B_GPIO_Pin_CLK | AS5048B_GPIO_Pin_SDA;
	AS5048B_GPIO_Init.GPIO_PuPd = GPIO_PuPd_UP;
	AS5048B_GPIO_Init.GPIO_Speed = AS5048B_GPIO_Speed;
	GPIO_Init(AS5048B_GPIO, &AS5048B_GPIO_Init);

	GPIO_PinAFConfig(AS5048B_GPIO, AS5048B_GPIO_PinSourceCLK, AS5048B_GPIO_AF_I2Cx);
	GPIO_PinAFConfig(AS5048B_GPIO, AS5048B_GPIO_PinSourceSDA, AS5048B_GPIO_AF_I2Cx);

	I2C_InitTypeDef AS5048B_I2C_Initx;
	AS5048B_I2C_Initx.I2C_Ack = I2C_Ack_Disable;
	AS5048B_I2C_Initx.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	AS5048B_I2C_Initx.I2C_ClockSpeed = AS5048B_CLOCKSPEED;
	AS5048B_I2C_Initx.I2C_DutyCycle = I2C_DutyCycle_2;
	AS5048B_I2C_Initx.I2C_Mode = I2C_Mode_I2C;
	AS5048B_I2C_Initx.I2C_OwnAddress1 = 0;

	I2C_Init(AS5048B_I2Cx, &AS5048B_I2C_Initx);
}


/*******************************************************************************
* Function Name  : AS5048B_readBodyAngles
* Input          : None
* Output         : uint16_t *angleReg
*				 : uint8_t *autoGain
*				 : uint8_t *diag
*                : uint16_t *magnitude
*                : double *angle
* Return         : None
* Description    : read all the encoders in robot
*******************************************************************************/
void AS5048B_readBodyAngles(uint16_t *angleReg, uint8_t *autoGain, uint8_t *diag, uint16_t *magnitude,double *angle)
{
	uint8_t numEncoders = 4;
	uint8_t	encAddressList[4] = {AS5048B_ADDRESS_FACTORY, AS5048B_ADDRESS_A, AS5048B_ADDRESS_B, AS5048B_ADDRESS_C};
	uint8_t i;
	
	for (i = 0;i < numEncoders;i++)
	{
		_chipAddress = encAddressList[i];		
		angleReg[i] = AS5048B_angleRegR(); 
		autoGain[i] = AS5048B_getAutoGain();
		diag[i] = AS5048B_getDiagReg();
		magnitude[i] = AS5048B_magnitudeR();
		angle[i] = AS5048B_angleR(U_DEG, true);		
	}
	
}
	
	

/*******************************************************************************
* Function Name  : AS5048B_initialize
* Input          : None
* Output         : None
* Return         : None
* Description    : init values and overall behaviors for AS5948B use
*******************************************************************************/
void AS5048B_initialize(void) {
	uint16_t zeroRegVal;
	uint8_t chipAddress;
	uint16_t angleReg;
	uint8_t autoGain;
	uint8_t diag;
	uint16_t magnitude;
	double angle;
	
	/* init I2C */
	AS5048B_initialize_I2C();
	
	// set current position zero
	//AS5048B_setZeroReg();
	
	// set clockwise readings
	AS5048B_setClockWise(_clockWise); 
	
	// set address
	chipAddress = 1;
	//AS5048B_addressRegW(chipAddress); 
	chipAddress = AS5048B_addressRegR(); 
	
	return;
}

/*******************************************************************************
* Function Name  : AS5048B_setClockWise
* Input          : boolean cw - true: CW, false: CCW
* Output         : None
* Return         : None
* Description    : Set / unset clock wise counting - sensor counts CCW natively
*******************************************************************************/
void AS5048B_setClockWise(boolean_T cw){
	
	_clockWise = cw;
	return;
}

/*******************************************************************************
* Function Name  : AS5048B_addressRegW
* Input          : unit8_t register value
* Output         : None
* Return         : None
* Description    : write I2C address value (5 bits) into the address register - not done 
*******************************************************************************/
void AS5048B_addressRegW(uint8_t regVal) {

	AS5048B_writeReg(AS5048B_ADDR_REG, regVal);
	
	return;
}

/*******************************************************************************
* Function Name  : AS5048B_addressRegR
* Input          : None
* Output         : None
* Return         : uint8_t register value
* Description    : reads I2C address register value
*******************************************************************************/
uint8_t AS5048B_addressRegR(void) {

	return AS5048B_readReg8(AS5048B_ADDR_REG);
}

/*******************************************************************************
* Function Name  : AS5048B_setZeroReg
* Input          : None
* Output         : None
* Return         : None
* Description    : sets current angle as the zero position
*******************************************************************************/
void AS5048B_setZeroReg(void) {
	
	uint16_t angle;
	uint16_t zeroAngle;
	angle = AS5048B_angleRegR();
	
	AS5048B_zeroRegW(angle);
	return;
}

/*******************************************************************************
* Function Name  : AS5048B_zeroRegW
* Input          : unit16_t register value
* Output         : None
* Return         : None
* Description    : writes the 2 bytes Zero position register value
*******************************************************************************/
void AS5048B_zeroRegW(uint16_t regVal) {

	AS5048B_writeReg(AS5048B_ZEROMSB_REG, (uint8_t)(regVal >> 6));
	AS5048B_writeReg(AS5048B_ZEROLSB_REG, (uint8_t)(regVal & 0x3F));
	return;
}

/*******************************************************************************
* Function Name  : AS5048B_zeroRegR
* Input          : None
* Output         : None
* Return         : uint16_t register value trimmed on 14 bits
* Description    : reads the 2 bytes Zero position register value
*******************************************************************************/
uint16_t AS5048B_zeroRegR(void) {
	uint16_t readValue;
	uint8_t readArray[2];
	readArray[0] = AS5048B_readReg8(AS5048B_ZEROMSB_REG);
	readArray[1] = AS5048B_readReg8(AS5048B_ZEROLSB_REG);
	
	readValue = (((uint16_t) readArray[0]) << 6);
	readValue += (readArray[1] & 0x3F);
	return readValue;
}

/*******************************************************************************
* Function Name  : AS5048B_magnitudeR
* Input          : None
* Output         : None
* Return         : uint16_t register value trimmed on 14 bits
* Description    : reads the 2 bytes magnitude register value
*******************************************************************************/
uint16_t AS5048B_magnitudeR(void) {
	uint16_t readValue;
	uint8_t readArray[2];
	readArray[0] = AS5048B_readReg8(AS5048B_MAGNMSB_REG);
	readArray[1] = AS5048B_readReg8(AS5048B_MAGNLSB_REG);
	
	readValue = (((uint16_t) readArray[0]) << 6);
	readValue += (readArray[1] & 0x3F);
	return readValue;
}

/*******************************************************************************
* Function Name  : AS5048B_angleRegR
* Input          : None
* Output         : None
* Return         : uint16_t register value trimmed on 14 bits
* Description    : reads the 2 bytes angle register value
*******************************************************************************/
uint16_t AS5048B_angleRegR(void) {
	int16_t readValue;
	uint8_t readArray[2];
	readArray[0] = AS5048B_readReg8(AS5048B_ANGLMSB_REG);
	readArray[1] = AS5048B_readReg8(AS5048B_ANGLLSB_REG);
	
	readValue = (((uint16_t) readArray[0]) << 6);
	readValue += (readArray[1] & 0x3F);
	return readValue;
}

/*******************************************************************************
* Function Name  : AS5048B_getAutoGain
* Input          : None
* Output         : None
* Return         : uint8_t register value 
* Description    : reads the 1 bytes auto gain register value
*******************************************************************************/
uint8_t AS5048B_getAutoGain(void) {

	return AS5048B_readReg8(AS5048B_GAIN_REG);
}

/*******************************************************************************
* Function Name  : 
* Input          : None
* Output         : None
* Return         : uint8_t register value 
* Description    : reads the 1 bytes diagnostic register value
*******************************************************************************/
uint8_t AS5048B_getDiagReg(void) {

	return AS5048B_readReg8(AS5048B_DIAG_REG);
}

/*******************************************************************************
* Function Name  : AS5048B_angleR
* Input          : String unit : string expressing the unit of the angle. Sensor raw value as default
*				 : Boolean newVal : have a new measurement or use the last read one. True as default
* Output         : None
* Return         : Double angle value converted into the desired unit
* Description    : reads current angle value and converts it into the desired unit
*******************************************************************************/
double AS5048B_angleR(int unit, boolean_T newVal) {

	double angleRaw;
	int16_t readValue;
	uint8_t readArray[2];
	readArray[0] = AS5048B_readReg8(AS5048B_ANGLMSB_REG);
	readArray[1] = AS5048B_readReg8(AS5048B_ANGLLSB_REG);
	
	readValue = (((uint16_t) readArray[0]) << 6);
	readValue += (readArray[1] & 0x3F);
	
	
	if (newVal) {
		if (_clockWise) {
			angleRaw = (double)(0b11111111111111 - readValue);
		}
		else {
			angleRaw = (double) readValue;
		}
	}
	else {
		angleRaw = 0.0;
	}

	return AS5048B_convertAngle(unit, angleRaw);
}

/*========================================================================*/
/*                           PRIVATE FUNCTIONS                            */
/*========================================================================*/

/*******************************************************************************
* Function Name  : AS5048B_readReg8
* Input          : uint8_t address
* Output         : uint8_t address
* Return         : None
* Description    : read register for given address
*******************************************************************************/
uint8_t AS5048B_readReg8(uint8_t address) {
	
	uint8_t readValue;
	int counter = 0;
	
	// wait until I2C is not busy any more
	//while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) {
		//counter++;
		//if (counter == TIME_OUT_DELAY) {
			//TIME_OUT_ERR = true;
			//break;
		//}
	//}
		
	// send I2C START condition 
	counter = 0;
	I2C_GenerateSTART(I2C1, ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
		counter++;
		if (counter == TIME_OUT_DELAY) {
			TIME_OUT_ERR = true;
			break;
		}
	}

	//send slave address for write 
	counter = 0;
	I2C_Send7bitAddress(I2C1, _chipAddress, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
		counter++;
		if (counter == TIME_OUT_DELAY) {
			TIME_OUT_ERR = true;
			break;
		}
	}
	
	//send slave register address for write
	counter = 0;
	I2C_SendData(I2C1, address);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		counter++;
		if (counter == TIME_OUT_DELAY) {
			TIME_OUT_ERR = true;
			break;
		}
	}
	
	 //stop
	//I2C_GenerateSTOP(I2C1, ENABLE);
	
	// wait until I2C is not busy any more
	//counter = 0;
	//while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) {
		//counter++;
		//if (counter == TIME_OUT_DELAY) {
			//TIME_OUT_ERR = true;
			//break;
		//}
	//}
		
	// send I2C START condition 
	I2C_GenerateSTART(I2C1, ENABLE);
	
	counter = 0;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
		counter++;
		if (counter == TIME_OUT_DELAY) {
			TIME_OUT_ERR = true;
			break;
		}
	}
	
	// send slave address to read
	counter = 0;
	I2C_Send7bitAddress(I2C1, _chipAddress, I2C_Direction_Receiver);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
		counter++;
		if (counter == TIME_OUT_DELAY) {
			TIME_OUT_ERR = true;
			break;
		}
	}
	
	// enable acknowledge of received data
	//I2C_AcknowledgeConfig(I2C1, ENABLE);
	
	// wait until one byte has been received
	counter = 0;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
		counter++;
		if (counter == TIME_OUT_DELAY) {
			TIME_OUT_ERR = true;
			break;
		}
	}
	
	// read data from I2C data register and return data byte
	readValue = I2C_ReceiveData(I2C1);
	
	// stop
	I2C_GenerateSTOP(I2C1, ENABLE);
	
	
	return readValue;
}

/*******************************************************************************
* Function Name  : AS5048B_writeReg
* Input          : uint8_t address
*                : uint8_t value
* Output         : None
* Return         : None
* Description    : 
*******************************************************************************/
void AS5048B_writeReg(uint8_t address, uint8_t value) {
	
	int counter = 0;
	
	// wait until I2C is not busy any more
	//while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) {
		//counter++;
		//if (counter == TIME_OUT_DELAY) {
			//TIME_OUT_ERR = true;
			//break;
		//}
	//}
	
	// start I2C
	counter = 0;
	I2C_GenerateSTART(I2C1, ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
		counter++;
		if (counter == TIME_OUT_DELAY) {
			TIME_OUT_ERR = true;
			break;
		}
	}

	//send slave address for write 
	counter = 0;
	I2C_Send7bitAddress(I2C1, _chipAddress, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
		counter++;
		if (counter == TIME_OUT_DELAY) {
			TIME_OUT_ERR = true;
			break;
		}
	}
	
	//send register address for write 
	counter = 0;
	I2C_SendData(I2C1, address);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		counter++;
		if (counter == TIME_OUT_DELAY) {
			TIME_OUT_ERR = true;
			break;
		}
	}
	
	//send value for write 
	counter = 0;
	I2C_SendData(I2C1, value);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		counter++;
		if (counter == TIME_OUT_DELAY) {
			TIME_OUT_ERR = true;
			break;
		}
	}
	
	// stop 
	I2C_GenerateSTOP(I2C1, ENABLE);

	return;
}

/*******************************************************************************
* Function Name  : AS5048B_convertAngle
* Input          : int unit
*                : double angle
* Output         : None
* Return         : double
* Description    : RAW, TRN, DEG, RAD, GRAD, MOA, SOA, MILNATO, MILSE, MILRU
*******************************************************************************/
double AS5048B_convertAngle(int unit, double angle) {
	
	// convert raw sensor reading into angle unit
	
	double angleConv;
	
	switch (unit) {
	case U_RAW:
		//Sensor raw measurement
		angleConv = angle;
		break;
	case U_TRN:
		//full turn ratio
		angleConv = (angle / AS5048B_RESOLUTION);
		break;
	case U_DEG:
		//degree
		angleConv = (angle / AS5048B_RESOLUTION) * 360.0;
		break;
	case U_RAD:
		//Radian
		angleConv = (angle / AS5048B_RESOLUTION) * 2 * PI;
		break;
	case U_MOA:
		//minute of arc
		angleConv = (angle / AS5048B_RESOLUTION) * 60.0 * 360.0;
		break;
	case U_SOA:
		//second of arc
		angleConv = (angle / AS5048B_RESOLUTION) * 60.0 * 60.0 * 360.0;
		break;
	case U_GRAD:
		//grade
		angleConv = (angle / AS5048B_RESOLUTION) * 400.0;
		break;
	case U_MILNATO:
		//NATO MIL
		angleConv = (angle / AS5048B_RESOLUTION) * 6400.0;
		break;
	case U_MILSE:
		//Swedish MIL
		angleConv = (angle / AS5048B_RESOLUTION) * 6300.0;
		break;
	case U_MILRU:
		//Russian MIL
		angleConv = (angle / AS5048B_RESOLUTION) * 6000.0;
		break;
	default:
		//no conversion => raw angle
		angleConv = angle;
		break;	
	}	
	return angleConv;
}

/**/
