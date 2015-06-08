/*
* File: vn100.cpp
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

#include <stdlib.h>
#include <stdio.h>
#include "vn100.h"
#include "../debug.h"
#include "stm32f4xx.h"
#include "daq.h"
#include "vn100_data.h"


/* MACROS */
//#define DEBUG_VN100
//#define WRITE_SETTING_TO_EEPROM_VN100

#define VN100_BR 921600 		// Baudrate for VN100 Comms

#define VN100_BUFFER_SIZE 256 // The number of bytes from the serial port to cache while waiting for a complete message to arrive
#define VN100_MESSAGE_SIZE 42 // The length of the binary imu message, including header
#define VN100_HEADER_0 0xFA  // Header byte
#define VN100_HEADER_1 0x01  // Header byte
#define VN100_HEADER_2 0x08  // Header byte
#define VN100_HEADER_3 0x02  // Header byte
#define VN100_MAX_COMMAND_SIZE	256

// VN100 timer setup
#define VN100_TIM TIM7
#define VN100_RCC_APBPeriph RCC_APB1Periph_TIM7


// VN100 USART setup
#define VN100_NbRcv daq_Y.usart2_NbRcv
#define VN100_RcvVal daq_Y.usart2_RcvVal
#define VN100_Nb2Send daq_U.usart2_Nb2Send
#define VN100_SendVal daq_U.usart2_SendVal

/**/
ExternalInputs_vn100 vn100_U;

/**/
ExternalOutputs_vn100 vn100_Y;

/**/
Parameters_vn100 vn100_P;



/*******************************************************************************
* Function Name  : vn100_checksum_compute
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

unsigned char vn100_checksum_compute(const char* cmdToCheck)
{
	int i;
	unsigned char xorVal = 0;
	int cmdLength;

	cmdLength = strlen(cmdToCheck);

	for (i = 0; i < cmdLength; i++)
		xorVal ^= (unsigned char) cmdToCheck[i];

	return xorVal;
}


/*******************************************************************************
* Function Name  : vn100_checksum_computeAndReturnAsHex
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_checksum_computeAndReturnAsHex(const char* cmdToCheck, char* checksum)
{
	unsigned char cs;
	char tempChecksumHolder[3];

	cs = vn100_checksum_compute(cmdToCheck);

	/* We cannot sprintf into the parameter checksum because sprintf always
	appends a null at the end. */
	sprintf(tempChecksumHolder, "%X", cs);

	checksum[0] = tempChecksumHolder[0];
	checksum[1] = tempChecksumHolder[1];
}

/*******************************************************************************
* Function Name  : vn100_writeOutCommand
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_writeOutCommand(const char* cmdToSend)
{
	char packetTail[] = "*FF\r\n";

	/* We add one to the cmdToSend pointer to skip over the '$' at the beginning. */
	/* We add one to the packetTail pointer so the "FF" string is overwritten with the checksum. */
	vn100_checksum_computeAndReturnAsHex(cmdToSend + 1, packetTail + 1);

	vn100_writeData_threadSafe(cmdToSend, strlen(cmdToSend));
	vn100_writeData_threadSafe(packetTail, strlen(packetTail));

}


/*******************************************************************************
* Function Name  : vn100_writeData_threadSafe
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_writeData_threadSafe(const char* dataToSend, unsigned int dataLength)
{
	uint8_T i;
	for(i=0; i<dataLength; i++){
		VN100_SendVal[i] = ' ';
	}

	for(i=0; i<dataLength; i++){
		VN100_SendVal[i] = dataToSend[i];
	}

	VN100_Nb2Send = dataLength;

	daq_step();
	vn100_delay_ms(100);
}

/*******************************************************************************
* Function Name  : vn100_reset
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_reset(void)
{
	char *cmdToSend;
	cmdToSend = "$VNRST";
	vn100_writeOutCommand(cmdToSend);
	vn100_delay_ms(500);
}


/*******************************************************************************
* Function Name  : vn100_getSerialBaudRate
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_getSerialBaudRate(unsigned int* serialBaudrate)
{
	const char* cmdToSend = "$VNRRG,05";
	char delims[] = ",*";
	const char* responseMatch = "$VNRRG,05";
	char* tmp1;

	vn100_writeOutCommand(cmdToSend);

	vn100_delay_ms(500);
	daq_step();

	tmp1 = strstr((char*) VN100_RcvVal, responseMatch);
	tmp1 = tmp1 + strlen(responseMatch);
	++tmp1;
	tmp1 = strtok(tmp1, delims);
	*serialBaudrate = (unsigned int) atoi(tmp1);
}

/*******************************************************************************
* Function Name  : vn100_setSerialBaudRate
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_setSerialBaudRate(unsigned int serialBaudrate)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN100_MAX_COMMAND_SIZE];

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,5,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", serialBaudrate);

	cmdToSendBuilder[curBufLoc] = '\0';

	vn100_writeOutCommand(cmdToSendBuilder);
}


/*******************************************************************************
* Function Name  : vn100_getAsynchronousDataOutputType
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_getAsynchronousDataOutputType(unsigned int* asyncDataOutputType)
{
	const char* cmdToSend ="$VNRRG,6";
	const char* responseMatch = "$VNRRG,6";
	char delims[] = ",*";
	char* tmp1;

	vn100_writeOutCommand(cmdToSend);

	vn100_delay_ms(500);
	daq_step();

	tmp1 = strstr((char*) VN100_RcvVal, responseMatch);
	tmp1 = tmp1 + strlen(responseMatch);
	++tmp1;
	tmp1 = strtok(tmp1, delims);
	*asyncDataOutputType = (unsigned int) atoi(tmp1);
}


/*******************************************************************************
* Function Name  : vn100_setAsynchronousDataOutputType
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_setAsynchronousDataOutputType(unsigned int asyncDataOutputType){
	int curBufLoc = 0;
	char cmdToSendBuilder[VN100_MAX_COMMAND_SIZE];

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,6,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", asyncDataOutputType);

	cmdToSendBuilder[curBufLoc] = '\0';

	vn100_writeOutCommand(cmdToSendBuilder);
}


/*******************************************************************************
* Function Name  : vn100_getAsynchronousDataOutputFrequency
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_getAsynchronousDataOutputFrequency(unsigned int* asyncDataOutputFrequency){
	const char* cmdToSend = "$VNRRG,07";
	char delims[] = ",*";
	char* tmp1;
	const char* responseMatch = "$VNRRG,07";
	vn100_writeOutCommand(cmdToSend);

	vn100_delay_ms(500);
	daq_step();

	tmp1 = strstr((char*) VN100_RcvVal, responseMatch);
	tmp1 = tmp1 + strlen(responseMatch);
	++tmp1;
	tmp1 = strtok(tmp1, delims);
	*asyncDataOutputFrequency = (unsigned int) atoi(tmp1);

}


/*******************************************************************************
* Function Name  : vn100_setAsynchronousDataOutputFrequency
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_setAsynchronousDataOutputFrequency(unsigned int asyncDataOutputFrequency){
	int curBufLoc = 0;
	char cmdToSendBuilder[VN100_MAX_COMMAND_SIZE];

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,7,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", asyncDataOutputFrequency);

	cmdToSendBuilder[curBufLoc] = '\0';

	vn100_writeOutCommand(cmdToSendBuilder);
}



/*******************************************************************************
* Function Name  : vn100_getVpeControl
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_getVpeControl(unsigned char* enable, 
	unsigned char* headingMode, unsigned char* filteringMode, unsigned char* tuningMode){
		const char* cmdToSend = "$VNRRG,35";
		char delims[] = ",*";
		char* result;
		const char* responseMatch = "$VNRRG,35";

		vn100_delay_ms(500);
		daq_step();

		result = strstr((char*) VN100_RcvVal, responseMatch);
		result = result + strlen(responseMatch);
		++result;
		result = strtok(result, delims);
		*enable = (unsigned char) atoi(result);

		result = strtok(result, delims);	
		*headingMode = (unsigned char) atoi(result);

		result = strtok(result, delims);
		*filteringMode = (unsigned char) atoi(result);

		result = strtok(result, delims);
		*tuningMode = (unsigned char) atoi(result);
}

/*******************************************************************************
* Function Name  : vn100_setAsynchronousDataOutputFrequency
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_setVpeControl(unsigned char enable, unsigned char headingMode, 
	unsigned char filteringMode, unsigned char tuningMode){
		int curBufLoc = 0;
		char cmdToSendBuilder[VN100_MAX_COMMAND_SIZE];


		curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,35,");

		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", enable);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", headingMode);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", filteringMode);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", tuningMode);

		cmdToSendBuilder[curBufLoc] = '\0';

		vn100_writeOutCommand(cmdToSendBuilder);
}

/*******************************************************************************
* Function Name  : vn100_getVpeMagnetometerBasicTuning
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_getVpeMagnetometerBasicTuning(VnVector3* baseTuning, VnVector3* adaptiveTuning, VnVector3* adaptiveFiltering)
{
	const char* cmdToSend = "$VNRRG,36";
	char delims[] = ",*";
	char* result;
	const char* responseMatch = "VNRRG,36";


	vn100_delay_ms(500);
	daq_step();

	result = strstr((char*) VN100_RcvVal, responseMatch);
	result = result + strlen(responseMatch);
	++result;

	result = strtok(result, delims);
	baseTuning->c0 = atof(result);
	result = strtok(result, delims);
	baseTuning->c1 = atof(result);
	result = strtok(result, delims);
	baseTuning->c2 = atof(result);

	result = strtok(result, delims);
	adaptiveTuning->c0 = atof(result);
	result = strtok(result, delims);
	adaptiveTuning->c1 = atof(result);
	result = strtok(result, delims);
	adaptiveTuning->c2 = atof(result);

	result = strtok(result, delims);
	adaptiveFiltering->c0 = atof(result);
	result = strtok(result, delims);
	adaptiveFiltering->c1 = atof(result);
	result = strtok(result, delims);
	adaptiveFiltering->c2 = atof(result);
}

/*******************************************************************************
* Function Name  : vn100_setVpeMagnetometerBasicTuning
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_setVpeMagnetometerBasicTuning(VnVector3 baseTuning, VnVector3 adaptiveTuning, VnVector3 adaptiveFiltering){
	int curBufLoc = 0;
	char cmdToSendBuilder[VN100_MAX_COMMAND_SIZE];

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,36,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", baseTuning.c0, baseTuning.c1, baseTuning.c2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", adaptiveTuning.c0, adaptiveTuning.c1, adaptiveTuning.c2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", adaptiveFiltering.c0, adaptiveFiltering.c1, adaptiveFiltering.c2);

	cmdToSendBuilder[curBufLoc] = '\0';

	vn100_writeOutCommand(cmdToSendBuilder);
}

/*******************************************************************************
* Function Name  : vn100_getVpeMagnetometerAdvancedTuning
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_getVpeMagnetometerAdvancedTuning(VnVector3* minimumFiltering, VnVector3* maximumFiltering, 
	float* maximumAdaptRate, float* disturbanceWindow, float* maximumTuning)
{
	const char* cmdToSend = "$VNRRG,37";
	char delims[] = ",*";
	char* result;
	const char* responseMatch = "$VNRRG,37";

	vn100_delay_ms(500);
	daq_step();

	result = strstr((char*) VN100_RcvVal, responseMatch);
	result = result + strlen(responseMatch);
	++result;	

	result = strtok(result, delims);
	minimumFiltering->c0 = atof(result);
	result = strtok(result, delims);
	minimumFiltering->c1 = atof(result);
	result = strtok(result, delims);
	minimumFiltering->c2 = atof(result);

	result = strtok(result, delims);
	maximumFiltering->c0 = atof(result);
	result = strtok(result, delims);
	maximumFiltering->c1 = atof(result);
	result = strtok(result, delims);
	maximumFiltering->c2 = atof(result);

	result = strtok(result, delims);
	*maximumAdaptRate = (float) atof(result);
	result = strtok(result, delims);
	*disturbanceWindow = (float) atof(result);
	result = strtok(result, delims);
	*maximumTuning = (float) atof(result);
}

/*******************************************************************************
* Function Name  : vn100_setVpeMagnetometerAdvancedTuning
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_setVpeMagnetometerAdvancedTuning(VnVector3 minimumFiltering, VnVector3 maximumFiltering,
	float maximumAdaptRate, float disturbanceWindow, float maximumTuning){

		int curBufLoc = 0;
		char cmdToSendBuilder[VN100_MAX_COMMAND_SIZE];

		curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,37,");

		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", minimumFiltering.c0, minimumFiltering.c1, minimumFiltering.c2);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", maximumFiltering.c0, maximumFiltering.c1, maximumFiltering.c2);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", maximumAdaptRate);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", disturbanceWindow);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", maximumTuning);

		cmdToSendBuilder[curBufLoc] = '\0';

		vn100_writeOutCommand(cmdToSendBuilder);
}

/*******************************************************************************
* Function Name  : vn100_getVpeAccelerometerBasicTuning
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_getVpeAccelerometerBasicTuning(VnVector3* baseTuning, 
	VnVector3* adaptiveTuning, VnVector3* adaptiveFiltering){
		const char* cmdToSend = "$VNRRG,38";
		char delims[] = ",*";
		char* result;

		const char* responseMatch = "$VNRRG,38";
		vn100_delay_ms(500);
		daq_step();

		result = strstr((char*) VN100_RcvVal, responseMatch);
		result = result + strlen(responseMatch);
		++result;

		result = strtok(result, delims);
		baseTuning->c0 = atof(result);
		baseTuning->c1 = atof(result);
		baseTuning->c2 = atof(result);
		result = strtok(0, delims);

		result = strtok(result, delims);
		adaptiveTuning->c0 = atof(result);
		result = strtok(result, delims);
		adaptiveTuning->c1 = atof(result);
		result = strtok(result, delims);
		adaptiveTuning->c2 = atof(result);

		result = strtok(result, delims);
		adaptiveFiltering->c0 = atof(result);
		result = strtok(result, delims);
		adaptiveFiltering->c1 = atof(result);
		result = strtok(result, delims);
		adaptiveFiltering->c2 = atof(result);

}

/*******************************************************************************
* Function Name  : vn100_setAsynchronousDataOutputFrequency
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_setVpeAccelerometerBasicTuning(VnVector3 baseTuning, VnVector3 adaptiveTuning,
	VnVector3 adaptiveFiltering){
		int curBufLoc = 0;
		char cmdToSendBuilder[VN100_MAX_COMMAND_SIZE];

		curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,38,");

		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", baseTuning.c0, baseTuning.c1, baseTuning.c2);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", adaptiveTuning.c0, adaptiveTuning.c1, adaptiveTuning.c2);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", adaptiveFiltering.c0, adaptiveFiltering.c1, adaptiveFiltering.c2);

		cmdToSendBuilder[curBufLoc] = '\0';

		vn100_writeOutCommand(cmdToSendBuilder);
}

/*******************************************************************************
* Function Name  : vn100_getVpeAccelerometerAdvancedTuning
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_getVpeAccelerometerAdvancedTuning(VnVector3* minimumFiltering, VnVector3* maximumFiltering,
	float* maximumAdaptRate, float* disturbanceWindow, float* maximumTuning){
		const char* cmdToSend = "$VNRRG,39";
		char delims[] = ",*";
		char* result;
		const char* responseMatch = "$VNRRG,39";

		vn100_delay_ms(500);
		daq_step();

		result = strstr((char*) VN100_RcvVal, responseMatch);
		result = result + strlen(responseMatch);
		++result;

		result = strtok(result, delims);
		minimumFiltering->c0 = atof(result);
		result = strtok(result, delims);
		minimumFiltering->c1 = atof(result);
		result = strtok(result, delims);
		minimumFiltering->c2 = atof(result);

		result = strtok(result, delims);
		maximumFiltering->c0 = atof(result);
		result = strtok(result, delims);
		maximumFiltering->c1 = atof(result);
		result = strtok(result, delims);
		maximumFiltering->c2 = atof(result);

		result = strtok(result, delims);
		*maximumAdaptRate = (float) atof(result);
		result = strtok(result, delims);
		*disturbanceWindow = (float) atof(result);
		result = strtok(result, delims);
		*maximumTuning = (float) atof(result);

}

/*******************************************************************************
* Function Name  : vn100_setVpeAccelerometerAdvancedTuning
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_setVpeAccelerometerAdvancedTuning(VnVector3 minimumFiltering, VnVector3 maximumFiltering,
	float maximumAdaptRate, float disturbanceWindow, float maximumTuning){
		int curBufLoc = 0;
		char cmdToSendBuilder[VN100_MAX_COMMAND_SIZE];

		curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,39,");

		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", minimumFiltering.c0, minimumFiltering.c1, minimumFiltering.c2);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", maximumFiltering.c0, maximumFiltering.c1, maximumFiltering.c2);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", maximumAdaptRate);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", disturbanceWindow);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", maximumTuning);

		cmdToSendBuilder[curBufLoc] = '\0';

		vn100_writeOutCommand(cmdToSendBuilder);
}

/*******************************************************************************
* Function Name  : vn100_getVpeGyroBasicTuning
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_getVpeGyroBasicTuning(VnVector3* varianceAngularWalk, VnVector3* baseTuning, 
	VnVector3* adaptiveTuning)
{
	const char* cmdToSend = "$VNRRG,40";
	char delims[] = ",*";
	char* result;
	const char* responseMatch = "$VNRRG,40";

	vn100_delay_ms(500);
	daq_step();

	result = strstr((char*) VN100_RcvVal, responseMatch);
	result = result + strlen(responseMatch);
	++result;

	result = strtok(result, delims);
	varianceAngularWalk->c0 = atof(result);
	result = strtok(result, delims);
	varianceAngularWalk->c1 = atof(result);
	result = strtok(result, delims);
	varianceAngularWalk->c2 = atof(result);

	result = strtok(result, delims);
	baseTuning->c0 = atof(result);
	result = strtok(result, delims);
	baseTuning->c1 = atof(result);
	result = strtok(result, delims);
	baseTuning->c2 = atof(result);

	result = strtok(result, delims);
	adaptiveTuning->c0 = atof(result);
	result = strtok(result, delims);
	adaptiveTuning->c1 = atof(result);
	result = strtok(result, delims);
	adaptiveTuning->c2 = atof(result);

}

/*******************************************************************************
* Function Name  : vn100_setVpeGyroBasicTuning
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_setVpeGyroBasicTuning(VnVector3 varianceAngularWalk, VnVector3 baseTuning, 
	VnVector3 adaptiveTuning){
		int curBufLoc = 0;
		char cmdToSendBuilder[VN100_MAX_COMMAND_SIZE];

		curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,40,");

		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", varianceAngularWalk.c0, varianceAngularWalk.c1, varianceAngularWalk.c2);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", baseTuning.c0, baseTuning.c1, baseTuning.c2);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", adaptiveTuning.c0, adaptiveTuning.c1, adaptiveTuning.c2);

		cmdToSendBuilder[curBufLoc] = '\0';

		vn100_writeOutCommand(cmdToSendBuilder);
}

/*******************************************************************************
* Function Name  : vn100_getCalculatedMagnetometerCalibration
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_getCalculatedMagnetometerCalibration(VnMatrix3x3* c, VnVector3* b){
	const char* cmdToSend = "$VNRRG,47";
	char delims[] = ",*";
	char* result;
	const char* responseMatch = "$VNRRG,47";

	vn100_delay_ms(500);
	daq_step();

	result = strstr((char*) VN100_RcvVal, responseMatch);
	result = result + strlen(responseMatch);
	++result;
	result = strtok(result, delims);
	c->c00 = atof(result);
	result = strtok(result, delims);
	c->c01 = atof(result);
	result = strtok(result, delims);
	c->c02 = atof(result);
	result = strtok(result, delims);
	c->c10 = atof(result);
	result = strtok(result, delims);
	c->c11 = atof(result);
	result = strtok(result, delims);
	c->c12 = atof(result);
	result = strtok(result, delims);
	c->c20 = atof(result);
	result = strtok(result, delims);
	c->c21 = atof(result);
	result = strtok(result, delims);
	c->c22 = atof(result);

	result = strtok(result, delims);
	b->c0 = atof(result);
	result = strtok(result, delims);
	b->c1 = atof(result);
	result = strtok(result, delims);
	b->c2 = atof(result);

}

/*******************************************************************************
* Function Name  : vn100_getFilterStartupGyroBias
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/
void vn100_getFilterStartupGyroBias(VnVector3* gyroBias)
{
	const char* cmdToSend = "$VNRRG,43";
	char delims[] = ",*";
	char* result;
	const char* responseMatch = "$VNRRG,43";

	vn100_delay_ms(500);
	daq_step();

	result = strstr((char*) VN100_RcvVal, responseMatch);
	result = result + strlen(responseMatch);
	++result;

	result = strtok(result, delims);
	gyroBias->c0 = atof(result);
	result = strtok(result, delims);
	gyroBias->c1 = atof(result);
	result = strtok(result, delims);
	gyroBias->c2 = atof(result);

}

/*******************************************************************************
* Function Name  : vn100_setFilterStartupGyroBias
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_setFilterStartupGyroBias(VnVector3 gyroBias)
{

	int curBufLoc = 0;
	char cmdToSendBuilder[VN100_MAX_COMMAND_SIZE];

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,43,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", gyroBias.c0, gyroBias.c1, gyroBias.c2);

	cmdToSendBuilder[curBufLoc] = '\0';

	vn100_writeOutCommand(cmdToSendBuilder);
}

/*******************************************************************************
* Function Name  : vn100_getFilterMeasurementVarianceParameters
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_getFilterMeasurementVarianceParameters(double* angularWalkVariance, VnVector3* angularRateVariance, VnVector3* magneticVariance, VnVector3* accelerationVariance)
{
	const char* cmdToSend = "$VNRRG,22";
	char delims[] = ",*";
	char* result;
	const char* responseMatch = "$VNRRG,22";

	vn100_delay_ms(500);
	daq_step();

	result = strstr((char*) VN100_RcvVal, responseMatch);
	result = result + strlen(responseMatch);
	++result;
	result = strtok(result, delims);
	*angularWalkVariance = atof(result);
	result = strtok(result, delims);
	angularRateVariance->c0 = atof(result);
	result = strtok(result, delims);
	angularRateVariance->c1 = atof(result);
	result = strtok(result, delims);
	angularRateVariance->c2 = atof(result);

	result = strtok(result, delims);
	magneticVariance->c0 = atof(result);
	result = strtok(result, delims);
	magneticVariance->c1 = atof(result);
	result = strtok(result, delims);
	magneticVariance->c2 = atof(result);

	result = strtok(result, delims);
	accelerationVariance->c0 = atof(result);
	result = strtok(result, delims);
	accelerationVariance->c1 = atof(result);
	result = strtok(result, delims);
	accelerationVariance->c2 = atof(result);

}

/*******************************************************************************
* Function Name  : vn100_setFilterMeasurementVarianceParameters
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_setFilterMeasurementVarianceParameters(double angularWalkVariance, VnVector3 angularRateVariance,
	VnVector3 magneticVariance, VnVector3 accelerationVariance){
		int curBufLoc = 0;
		char cmdToSendBuilder[VN100_MAX_COMMAND_SIZE];


		curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,22,");

		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", angularWalkVariance);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", angularRateVariance.c0, angularRateVariance.c1, angularRateVariance.c2);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", magneticVariance.c0, magneticVariance.c1, magneticVariance.c2);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", accelerationVariance.c0, accelerationVariance.c1, accelerationVariance.c2);

		cmdToSendBuilder[curBufLoc] = '\0';

		vn100_writeOutCommand(cmdToSendBuilder);
}

/*******************************************************************************
* Function Name  : vn100_getFilterActiveTuningParameters
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_getFilterActiveTuningParameters(double* magneticGain, double* accelerationGain, 
	double* magneticMemory, double* accelerationMemory){
		const char* cmdToSend = "$VNRRG,24";
		char delims[] = ",*";
		char* result;
		const char* responseMatch = "$VNRRG,24";

		vn100_delay_ms(500);
		daq_step();

		result = strstr((char*) VN100_RcvVal, responseMatch);
		result = result + strlen(responseMatch);
		++result;
		result = strtok(result, delims);
		*magneticGain = atof(result);
		result = strtok(0, delims);

		result = strtok(result, delims);
		*accelerationGain = atof(result);

		result = strtok(result, delims);
		*magneticMemory = atof(result);

		result = strtok(result, delims);
		*accelerationMemory = atof(result);

}

/*******************************************************************************
* Function Name  : vn100_setFilterActiveTuningParameters
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_setFilterActiveTuningParameters(double magneticGain, double accelerationGain, 
	double magneticMemory, double accelerationMemory){
		int curBufLoc = 0;
		char cmdToSendBuilder[VN100_MAX_COMMAND_SIZE];

		curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,24,");

		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", magneticGain);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", accelerationGain);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", magneticMemory);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", accelerationMemory);

		cmdToSendBuilder[curBufLoc] = '\0';

		vn100_writeOutCommand(cmdToSendBuilder);
}

/*******************************************************************************
* Function Name  : vn100_getSynchronizationControl
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_getSynchronizationControl(unsigned char* syncInMode, unsigned char* syncInEdge, 
	unsigned short* syncInSkipFactor, unsigned int* reserved0, unsigned char* syncOutMode, 
	unsigned char* syncOutPolarity, unsigned short* syncOutSkipFactor, 
	unsigned int* syncOutPulseWidth, unsigned int* reserved1){
		const char* cmdToSend = "$VNRRG,32";
		char delims[] = ",*";
		char* result;
		const char* responseMatch = "$VNRRG,32";

		vn100_delay_ms(500);
		daq_step();

		result = strstr((char*) VN100_RcvVal, responseMatch);
		result = result + strlen(responseMatch);
		++result;
		result = strtok(result, delims);
		*syncInMode = (unsigned char) atoi(result);

		result = strtok(result, delims);
		*syncInEdge = (unsigned char) atoi(result);

		result = strtok(result, delims);
		*syncInSkipFactor = (unsigned short) atoi(result);

		result = strtok(result, delims);
		*reserved0 = (unsigned int) atoi(result);

		result = strtok(result, delims);
		*syncOutMode = (unsigned char) atoi(result);

		result = strtok(result, delims);
		*syncOutPolarity = (unsigned char) atoi(result);

		result = strtok(result, delims);
		*syncOutSkipFactor = (unsigned short) atoi(result);

		result = strtok(result, delims);
		*syncOutPulseWidth = (unsigned int) atoi(result);

		result = strtok(result, delims);
		*reserved1 = (unsigned int) atoi(result);

}

/*******************************************************************************
* Function Name  : vn100_setSynchronizationControl
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_setSynchronizationControl(unsigned char syncInMode, unsigned char syncInEdge,
	unsigned short syncInSkipFactor, unsigned int reserved0, unsigned char syncOutMode, 
	unsigned char syncOutPolarity, unsigned short syncOutSkipFactor, 
	unsigned int syncOutPulseWidth, unsigned int reserved1){
		int curBufLoc = 0;
		char cmdToSendBuilder[VN100_MAX_COMMAND_SIZE];

		curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,32,");

		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", syncInMode);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", syncInEdge);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", syncInSkipFactor);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", reserved0);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", syncOutMode);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", syncOutPolarity);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", syncOutSkipFactor);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", syncOutPulseWidth);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", reserved1);

		cmdToSendBuilder[curBufLoc] = '\0';

		vn100_writeOutCommand(cmdToSendBuilder);
}

/*******************************************************************************
* Function Name  : vn100_getYawPitchRoll
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_getYawPitchRoll(void)
{
	char packetToSend[16];
	char *cmdToSend = "$VNRRG,08";
	char packetTail[] = "*FF\r\n";
	const char* responseMatch = "$VNRRG,08";
	

	char delims[] = ",*";
	char* result;

	vn100_checksum_computeAndReturnAsHex(cmdToSend + 1, packetTail + 1);
	strcpy(packetToSend,cmdToSend);
	strcat(packetToSend,packetTail);

	uint8_T i;
	for(i=0; i<strlen(packetToSend); i++){
		VN100_SendVal[i] = ' ';
	}

	for(i=0; i<strlen(packetToSend); i++){
		VN100_SendVal[i] = packetToSend[i];
	}

	VN100_Nb2Send = strlen(packetToSend);


	result = strstr((char*) VN100_RcvVal, responseMatch);
	
	if(vn100_checksum_compute(result)){
		result = result + strlen(responseMatch);
		++result;
		result = strtok(result, delims);
		vn100_U.yaw = atof(result);
		result = strtok(0, delims);
		vn100_U.pitch = atof(result);
		result = strtok(0, delims);
		vn100_U.roll = atof(result);
	}

#ifdef DEBUG_VN100

	char tmpBuff[40];
	char *Buff;


	Buff =  "Roll = %f, Pitch = %f, Yaw = %f \r\n";
	//uint8_T i;
	for(i=0; i<strlen(tmpBuff); i++){
		tmpBuff[i] = ' ';
	}
	sprintf(tmpBuff, Buff, vn100_U.roll, vn100_U.pitch, vn100_U.yaw);
	debug_printf(tmpBuff, strlen(tmpBuff));

#endif




}


/*******************************************************************************
* Function Name  : vn100_getFilterBasicControl
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_getFilterBasicControl(unsigned char* magneticMode, 
	unsigned char* externalMagnetometerMode, 
	unsigned char* externalAccelerometerMode, 
	unsigned char* externalGyroscopeMode, VnVector3* angularRateLimit)
{
	const char* cmdToSend = "$VNRRG,34";
	char delims[] = ",*";
	char* result;
	const char* responseMatch = "VNRRG,";

	vn100_delay_ms(500);
	daq_step();

	result = strstr((char*) VN100_RcvVal, responseMatch);
	result = result + strlen(responseMatch);
	++result;
	result = strtok(result, delims);

	*magneticMode = (unsigned char) atoi(result);
	result = strtok(result, delims);
	*externalMagnetometerMode = (unsigned char) atoi(result);
	result = strtok(result, delims);
	*externalAccelerometerMode = (unsigned char) atoi(result);
	result = strtok(result, delims);
	*externalGyroscopeMode = (unsigned char) atoi(result);
	result = strtok(result, delims);
	angularRateLimit->c0 = atof(result);
	result = strtok(result, delims);
	angularRateLimit->c1 = atof(result);
	result = strtok(result, delims);
	angularRateLimit->c2 = atof(result);

}

/*******************************************************************************
* Function Name  : vn100_setFilterBasicControl
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_setFilterBasicControl(unsigned char magneticMode, 
	unsigned char externalMagnetometerMode, 
	unsigned char externalAccelerometerMode, 
	unsigned char externalGyroscopeMode, 
	VnVector3 angularRateLimit){

		int curBufLoc = 0;
		char cmdToSendBuilder[VN100_MAX_COMMAND_SIZE];


		curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,34,");

		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", magneticMode);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", externalMagnetometerMode);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", externalAccelerometerMode);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", externalGyroscopeMode);
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", angularRateLimit.c0, angularRateLimit.c1, angularRateLimit.c2);

		cmdToSendBuilder[curBufLoc] = '\0';

		vn100_writeOutCommand(cmdToSendBuilder);
}


/*******************************************************************************
* Function Name  : vn100_writeSettingsToVolatileMemory
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_writeSettingsToVolatileMemory(void){ 

		int curBufLoc = 0;
		char cmdToSendBuilder[VN100_MAX_COMMAND_SIZE];

		curBufLoc = sprintf(cmdToSendBuilder, "$VNWNV");
		cmdToSendBuilder[curBufLoc] = '\0';

		vn100_writeOutCommand(cmdToSendBuilder);
}



/*******************************************************************************
* Function Name  : vn100_confBinaryOutputMessage
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_confBinaryOutputMessage(BinaryMsg_vn100 msg){ 
	int curBufLoc = 0;
	char cmdToSendBuilder[VN100_MAX_COMMAND_SIZE];
	unsigned char serialPort;
	unsigned char devisor;
	unsigned char reserved = 1;
	uint16_T bitOffset;
	MsgType_vn100 msgType;

	serialPort = msg.serialPort;
	devisor = msg.devisor;
	msgType = msg.msg;


	bitOffset = vn100_bitOffset_compute(msgType);

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,77,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", serialPort);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", devisor);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", reserved);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", bitOffset);
	
	cmdToSendBuilder[curBufLoc] = '\0';

	vn100_writeOutCommand(cmdToSendBuilder);
}

/*******************************************************************************
* Function Name  : vn100_bitOffset_compute
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

uint16_T vn100_bitOffset_compute(MsgType_vn100 msgType){
	uint16_T bitOffset;

	bitOffset = msgType.TimeStartUp;
	bitOffset += msgType.TimeGPS<<1;
	bitOffset += msgType.TimeSyncIn<<2;
	bitOffset += msgType.Ypr<<3;
	bitOffset += msgType.Qtn<<4;
	bitOffset += msgType.AngularRate<<5;
	bitOffset += msgType.Position<<6;
	bitOffset += msgType.Velocity<<7;
	bitOffset += msgType.Accel<<8;
	bitOffset += msgType.Imu<<9;
	bitOffset += msgType.MagPres<<10;


	return bitOffset;
}

/*******************************************************************************
* Function Name  : vn100_initialize_defaults
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_initialize_defaults(void) {
	char *cmdToSend;

	/* Initialize timer for IMU delays.*/
	vn100_initialize_us_timer();

	
	/* Stop asynch output*/
	//vn100_AsyncOutputPause();
	
	
	/* Reset IMU.*/
	vn100_reset();

	/* Check 1000 Hz operation,*/
	unsigned int dataOutputFreq;
	vn100_getAsynchronousDataOutputFrequency(&dataOutputFreq);
	if(dataOutputFreq!=1000) 
		vn100_setAsynchronousDataOutputFrequency(1000);



	/* Check 921600 baud rate.*/
	unsigned int serialBaudRate;
	vn100_getSerialBaudRate(&serialBaudRate);
	if(serialBaudRate!=VN100_BR)	
		vn100_setSerialBaudRate(VN100_BR);

	/* Filter params.*/
	unsigned char enable = 1;
	unsigned char headingMode = 2;
	unsigned char filteringMode = 1;
	unsigned char tuningMode = 1;
	vn100_setVpeControl(enable, headingMode, filteringMode, tuningMode);


	VnVector3 baseTuning = {5,5,5};
	VnVector3 adaptiveTuning = {5,5,5};
	VnVector3 adaptiveFiltering = {5.5,5.5,5.5};
	vn100_setVpeMagnetometerBasicTuning(baseTuning,adaptiveTuning,adaptiveFiltering);

	VnVector3 minimumFiltering = {0,0,0};
	VnVector3 maximumFiltering = {6,6,6};
	float maximumAdaptRate = 0.4;
	float disturbanceWindow = 8.0;
	float maximumTuning = 1000;
	vn100_setVpeMagnetometerAdvancedTuning(minimumFiltering,maximumFiltering,
		maximumAdaptRate, disturbanceWindow, maximumTuning);


	baseTuning.c0 = 6;baseTuning.c1 = 6;baseTuning.c2 = 6;
	adaptiveTuning.c0 = 3;adaptiveTuning.c1 = 3;adaptiveTuning.c2 = 3;
	adaptiveFiltering.c0 = 5;adaptiveFiltering.c1 = 5;adaptiveFiltering.c2 = 5;
	vn100_setVpeAccelerometerBasicTuning(baseTuning,adaptiveTuning,adaptiveFiltering);


	minimumFiltering.c0 = 0;minimumFiltering.c1 = 0;minimumFiltering.c2 = 0;
	maximumFiltering.c0 = 6;maximumFiltering.c1 = 6;maximumFiltering.c2 = 6;
	maximumAdaptRate = 0.4;
	disturbanceWindow = 5; 
	maximumTuning = 1000;
	vn100_setVpeAccelerometerAdvancedTuning(minimumFiltering,maximumFiltering,
		maximumAdaptRate,disturbanceWindow, maximumTuning);


	VnVector3 varianceAngularWalk = {7,7,7};
	baseTuning.c0 = 3;baseTuning.c1 = 3;baseTuning.c2 = 3;
	adaptiveTuning.c0 = 0;adaptiveTuning.c1 = 0;adaptiveTuning.c2 = 0;
	vn100_setVpeGyroBasicTuning(varianceAngularWalk,baseTuning,adaptiveTuning);
/*
	cmdToSend = "$VNWRG,47,0.5,0"; 
	vn100_writeOutCommand(cmdToSend);
*/

	unsigned char magneticMode = 0; 
	unsigned char externalMagnetometerMode = 0; 
	unsigned char externalAccelerometerMode = 0;
	unsigned char externalGyroscopeMode = 0; 
	VnVector3 angularRateLimit = {16.581,16.581,16.581};
	vn100_setFilterBasicControl(magneticMode, 
				externalMagnetometerMode, 
				externalAccelerometerMode, 
				externalGyroscopeMode, 
				angularRateLimit);


	VnVector3 gyroBias = {0.0, 0.0, 0.0};
	vn100_setFilterStartupGyroBias(gyroBias);


	double angularWalkVariance = 1E-10;
	VnVector3 angularRateVariance = {1E-06,1E-06,1E-06};
	VnVector3 magneticVariance = {1E-02,1E-02,1E-02}; 
	VnVector3 accelerationVariance = {1E-02,1E-02,1E-02};
	vn100_setFilterMeasurementVarianceParameters(angularWalkVariance,angularRateVariance,
		magneticVariance,accelerationVariance);


	double magneticGain = 0; 
	double accelerationGain = 0; 
	double magneticMemory = 0.99;
	double accelerationMemory = 0.9;
	vn100_setFilterActiveTuningParameters(magneticGain, accelerationGain, 
	magneticMemory, accelerationMemory);


	/*Turn off ASCII data packs.*/
	unsigned int asyncDataOutputType = 0;
	vn100_setAsynchronousDataOutputType(asyncDataOutputType);

	/* Configure binary data packs.*/
	vn100_confBinaryOutputMessage(vn100_bmsg);

	/*IMU set to stream mode.*/
	unsigned char syncInMode = 3; 
	unsigned char syncInEdge = 0;
	unsigned short syncInSkipFactor = 0; 
	unsigned int reserved0 = 0;
	unsigned char syncOutMode = 3;
	unsigned char syncOutPolarity = 0; 
	unsigned short syncOutSkipFactor = 0;
	unsigned int syncOutPulseWidth = 500000;
	unsigned int reserved1 = 0;
	vn100_setSynchronizationControl(syncInMode, syncInEdge,
					syncInSkipFactor, reserved0, syncOutMode, 
					syncOutPolarity, syncOutSkipFactor, 
					syncOutPulseWidth, reserved1);

	/* Resume asynch output*/
	//vn100_AsyncOutputResume();
	
#ifdef WRITE_SETTING_TO_EEPROM_VN100
	/*Write setting to IMU.*/
vn100_writeSettingsToVolatileMemory();
#endif

}


/*******************************************************************************
* Function Name  : vn100_initialize
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_initialize(void) {
	char *cmdToSend;

	/* Initialize timer for IMU delays.*/
	vn100_initialize_us_timer();


	/* Reset IMU.*/
	vn100_reset();

	/* Check 1000 Hz operation,*/
	unsigned int dataOutputFreq;
	vn100_getAsynchronousDataOutputFrequency(&dataOutputFreq);
	if(dataOutputFreq!=1000) 
		vn100_setAsynchronousDataOutputFrequency(1000);



	/* Check 921600 baud rate.*/
	unsigned int serialBaudRate;
	vn100_getSerialBaudRate(&serialBaudRate);
	if(serialBaudRate!=VN100_BR)	
		vn100_setSerialBaudRate(VN100_BR);

	/* Filter params.*/
	unsigned char enable = 1;
	unsigned char headingMode = 1;
	unsigned char filteringMode = 1;
	unsigned char tuningMode = 1;
	vn100_setVpeControl(enable, headingMode, filteringMode, tuningMode);


	VnVector3 baseTuning = {0,0,0};
	VnVector3 adaptiveTuning = {0,0,0};
	VnVector3 adaptiveFiltering = {0,0,0};
	vn100_setVpeMagnetometerBasicTuning(baseTuning,adaptiveTuning,adaptiveFiltering);

	VnVector3 minimumFiltering = {0,0,0};
	VnVector3 maximumFiltering = {6,6,6};
	float maximumAdaptRate = 0.4;
	float disturbanceWindow = 2;
	float maximumTuning = 1000;
	vn100_setVpeMagnetometerAdvancedTuning(minimumFiltering,maximumFiltering,
		maximumAdaptRate, disturbanceWindow, maximumTuning);


	baseTuning.c0 = 6;baseTuning.c1 = 6;baseTuning.c2 = 6;
	adaptiveTuning.c0 = 3;adaptiveTuning.c1 = 3;adaptiveTuning.c2 = 3;
	adaptiveFiltering.c0 = 5;adaptiveFiltering.c1 = 5;adaptiveFiltering.c2 = 5;
	vn100_setVpeAccelerometerBasicTuning(baseTuning,adaptiveTuning,adaptiveFiltering);


	minimumFiltering.c0 = 0;minimumFiltering.c1 = 0;minimumFiltering.c2 = 0;
	maximumFiltering.c0 = 6;maximumFiltering.c1 = 6;maximumFiltering.c2 = 6;
	maximumAdaptRate = 0.4;
	disturbanceWindow = 5; 
	maximumTuning = 1000;
	vn100_setVpeAccelerometerAdvancedTuning(minimumFiltering,maximumFiltering,
		maximumAdaptRate,disturbanceWindow, maximumTuning);


	VnVector3 varianceAngularWalk = {7,7,7};
	baseTuning.c0 = 4;baseTuning.c1 = 4;baseTuning.c2 = 4;
	adaptiveTuning.c0 = 0;adaptiveTuning.c1 = 0;adaptiveTuning.c2 = 0;
	vn100_setVpeGyroBasicTuning(varianceAngularWalk,baseTuning,adaptiveTuning);
/*
	cmdToSend = "$VNWRG,47,0.5,0"; 
	vn100_writeOutCommand(cmdToSend);
*/

	unsigned char magneticMode = 0; 
	unsigned char externalMagnetometerMode = 0; 
	unsigned char externalAccelerometerMode = 0;
	unsigned char externalGyroscopeMode = 0; 
	VnVector3 angularRateLimit = {40,40,40};
	vn100_setFilterBasicControl(magneticMode, 
				externalMagnetometerMode, 
				externalAccelerometerMode, 
				externalGyroscopeMode, 
				angularRateLimit);


	VnVector3 gyroBias = {0.0, 0.0, 0.0};
	vn100_setFilterStartupGyroBias(gyroBias);


	double angularWalkVariance = 1E-07;
	VnVector3 angularRateVariance = {1E-07,1E-07,1E-07};
	VnVector3 magneticVariance = {0.1,0.1,0.1}; 
	VnVector3 accelerationVariance = {0.001,0.001,0.001};
	vn100_setFilterMeasurementVarianceParameters(angularWalkVariance,angularRateVariance,
		magneticVariance,accelerationVariance);


	double magneticGain = 0; 
	double accelerationGain = 0; 
	double magneticMemory = 0.99;
	double accelerationMemory = 0.9;
	vn100_setFilterActiveTuningParameters(magneticGain, accelerationGain, 
	magneticMemory, accelerationMemory);


	/*Turn off ASCII data packs.*/
	unsigned int asyncDataOutputType = 0;
	vn100_setAsynchronousDataOutputType(asyncDataOutputType);

	/* Configure binary data packs.*/
	vn100_confBinaryOutputMessage(vn100_bmsg);

	/*IMU set to async transfer on falling edge.*/
	unsigned char syncInMode = 5; 
	unsigned char syncInEdge = 1;
	unsigned short syncInSkipFactor = 0; 
	unsigned int reserved0 = 0;
	unsigned char syncOutMode = 3;
	unsigned char syncOutPolarity = 0; 
	unsigned short syncOutSkipFactor = 0;
	unsigned int syncOutPulseWidth = 500000;
	unsigned int reserved1 = 0;
	vn100_setSynchronizationControl(syncInMode, syncInEdge,
					syncInSkipFactor, reserved0, syncOutMode, 
					syncOutPolarity, syncOutSkipFactor, 
					syncOutPulseWidth, reserved1);

#ifdef WRITE_SETTING_TO_EEPROM_VN100
	/*Write setting to IMU.*/
vn100_writeSettingsToVolatileMemory();
#endif

	syncInMode = 3;
	syncInEdge = 0;
	vn100_setSynchronizationControl(syncInMode, syncInEdge,
					syncInSkipFactor, reserved0, syncOutMode, 
					syncOutPolarity, syncOutSkipFactor, 
					syncOutPulseWidth, reserved1);

}


/*******************************************************************************
* Function Name  : vn100_AsyncOutputPause
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_AsyncOutputPause(void) { 

	int curBufLoc = 0;
	char cmdToSendBuilder[VN100_MAX_COMMAND_SIZE];

	curBufLoc = sprintf(cmdToSendBuilder, "$VNASY,0");
	cmdToSendBuilder[curBufLoc] = '\0';

	vn100_writeOutCommand(cmdToSendBuilder);
}

/*******************************************************************************
* Function Name  : vn100_writeSettingsToVolatileMemory
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/

void vn100_AsyncOutputResume(void) { 

	int curBufLoc = 0;
	char cmdToSendBuilder[VN100_MAX_COMMAND_SIZE];

	curBufLoc = sprintf(cmdToSendBuilder, "$VNASY,1");
	cmdToSendBuilder[curBufLoc] = '\0';

	vn100_writeOutCommand(cmdToSendBuilder);
}


/*******************************************************************************
* Function Name  : check_crc
* Input          : byte message from the IMU
* Output         : None
* Return         : true if CRC confirms message bytes
* Description    : Checks all of the message bytes against the IMU-provided CRC
*                  code to validate the transfer
*******************************************************************************/

uint8_T vn100_check_crc(const char *message) {
	unsigned short crc = 0;
	uint32_T i;
	for( i = 1; i < VN100_MESSAGE_SIZE; i++) {
		crc = (unsigned char)(crc >> 8) | (crc << 8);
		crc ^= message[i];
		crc ^= (unsigned char)(crc & 0xff) >> 4;
		crc ^= (crc << 8) << 4;
		crc ^= ((crc & 0xff) << 4) << 1;
	}

	return crc == 0x0000;
}



/*******************************************************************************
* Function Name  : vn100_delay_us
* Input          : 
* Output         : None
* Return         : 
* Description    : 
*******************************************************************************/

void vn100_delay_us(unsigned long us)
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
* Function Name  : vn100_delay_ms
* Input          : 
* Output         : None
* Return         : 
* Description    : 
*******************************************************************************/

void vn100_delay_ms(unsigned long ms)
{
	vn100_delay_us(ms * 1000);
}


/*******************************************************************************
* Function Name  : vn100_initialize_us_timer
* Input          : 
* Output         : None
* Return         : 
* Description    : 
*******************************************************************************/

void vn100_initialize_us_timer()
{
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);

	RCC->APB1ENR |= VN100_RCC_APBPeriph;
	VN100_TIM->PSC = (RCC_Clocks.PCLK1_Frequency / 1000000) - 1;
	VN100_TIM->ARR = 0xFFFF;
	VN100_TIM->CR1 = TIM_CR1_CEN;
}
