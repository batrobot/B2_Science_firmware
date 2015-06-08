/*
 * File: vn100_data.h
 *
 * Model version      : 
 * C/C++ source code generated on  : 
 *
 * Target selection: stm32F4xx.tlc
 * Embedded hardware selection: STMicroelectronics->STM32F4xx 32-bit Cortex-M4
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

#ifndef VN100_DATA_h_
#define VN100_DATA_h_

#include "vn100.h"

/**/
ExternalInputs_vn100 vn100_U = {
	{0},
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0
};

/**/
ExternalOutputs_vn100 vn100_Y = {
	{0},
	0
};

/**/
Parameters_vn100 vn100_P = {

};


/**/
BinaryMsg_vn100 vn100_bmsg = {
	1, // Serial port
	1, // devisor
	{false, // Time Startup
	false, // TimeGPS
	false, //TimeSyncIn
	true,	// Ypr
	false,	// Qtn
	false,	// Angular rate
	false,	// Position
	false,	// Velocity
	false,	// Accel
	false,	// Imu
	false}	// Magpres
};


#endif
