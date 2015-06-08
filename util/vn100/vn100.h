/*
 * File: vn100.h
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

#ifndef VN100_h_
#define VN100_h_

#include "vn100_types.h"
#include "rtwtypes.h"



/**/
struct ExternalInputs_vn100_{
	uint8_T buffer[40];
	int16_T ptr;
	uint8_T chk_sum;
	uint8_T crc;
	uint8_T header;
	/*real_T roll;
	real_T pitch;
	real_T yaw;
	real_T gyrox;
	real_T gyroy;
	real_T gyroz;
	real_T accellx;
	real_T accelly;
	real_T accellz;*/
	float roll;
	float pitch;
	float yaw;
	float gyrox;
	float gyroy;
	float gyroz;
	float accellx;
	float accelly;
	float accellz;


};

/**/
struct ExternalOutputs_vn100_{
	uint8_T buffer[40];
	int16_T ptr;
};

/**/
struct Parameters_vn100_{
	
};


typedef struct{
	unsigned char TimeStartUp;
	unsigned char TimeGPS;
	unsigned char TimeSyncIn;
	unsigned char Ypr;
	unsigned char Qtn;
	unsigned char AngularRate;
	unsigned char Position;
	unsigned char Velocity;
	unsigned char Accel;
	unsigned char Imu;
	unsigned char MagPres;
} MsgType_vn100;

/**/
typedef struct{
	unsigned char serialPort;
	unsigned char devisor;
	MsgType_vn100 msg;
}BinaryMsg_vn100;


/**/
typedef struct {
	double	c0;		/**< Component 0 */
	double	c1;		/**< Component 1 */
	double	c2;		/**< Component 2 */
} VnVector3;

/**/
typedef struct {
	double c00;		/**< Component 0,0 */
	double c01;		/**< Component 0,1 */
	double c02;		/**< Component 0,2 */
	double c10;		/**< Component 1,0 */
	double c11;		/**< Component 1,1 */
	double c12;		/**< Component 1,2 */
	double c20;		/**< Component 2,0 */
	double c21;		/**< Component 2,1 */
	double c22;		/**< Component 2,2 */
} VnMatrix3x3;

/**/
extern ExternalInputs_vn100 vn100_U;

/**/
extern ExternalOutputs_vn100 vn100_Y;

/**/
extern Parameters_vn100 vn100_P;

/**/
void vn100_delay_us(unsigned long us);
void vn100_delay_ms(unsigned long ms);
void vn100_initialize_us_timer(void);


/**/
unsigned char vn100_checksum_compute(const char* cmdToCheck);
/**/
void vn100_checksum_computeAndReturnAsHex(const char* cmdToCheck, char* checksum);
/**/
void vn100_writeOutCommand(const char* cmdToSend);
/**/
void vn100_writeData_threadSafe(const char* dataToSend, unsigned int dataLength);
/**/
extern void vn100_reset(void);
/**/
void vn100_getSerialBaudRate(unsigned int* serialBaudrate);
/**/
void vn100_setSerialBaudRate(unsigned int serialBaudrate);
/**/
void vn100_getAsynchronousDataOutputType(unsigned int* asyncDataOutputType);
/**/
void vn100_setAsynchronousDataOutputType(unsigned int asyncDataOutputType);
/**/
void vn100_getAsynchronousDataOutputFrequency(unsigned int* asyncDataOutputFrequency);
/**/
void vn100_setAsynchronousDataOutputFrequency(unsigned int asyncDataOutputFrequency);
/**/
void vn100_getVpeControl(unsigned char* enable, 
	unsigned char* headingMode, unsigned char* filteringMode, unsigned char* tuningMode);
/**/
void vn100_setVpeControl(unsigned char enable, unsigned char headingMode, 
	unsigned char filteringMode, unsigned char tuningMode);
/**/
void vn100_getVpeMagnetometerBasicTuning(VnVector3* baseTuning, VnVector3* adaptiveTuning, VnVector3* adaptiveFiltering);
/**/
void vn100_setVpeMagnetometerBasicTuning(VnVector3 baseTuning, VnVector3 adaptiveTuning, VnVector3 adaptiveFiltering);
/**/
void vn100_getVpeMagnetometerAdvancedTuning(VnVector3* minimumFiltering, VnVector3* maximumFiltering, 
	float* maximumAdaptRate, float* disturbanceWindow, float* maximumTuning);
/**/
void vn100_setVpeMagnetometerAdvancedTuning(VnVector3 minimumFiltering, VnVector3 maximumFiltering,
	float maximumAdaptRate, float disturbanceWindow, float maximumTuning);
/**/
void vn100_getVpeAccelerometerBasicTuning(VnVector3* baseTuning, 
	VnVector3* adaptiveTuning, VnVector3* adaptiveFiltering);
/**/
void vn100_setVpeAccelerometerBasicTuning(VnVector3 baseTuning, VnVector3 adaptiveTuning,
	VnVector3 adaptiveFiltering);
/**/
void vn100_getVpeAccelerometerAdvancedTuning(VnVector3* minimumFiltering, VnVector3* maximumFiltering,
	float* maximumAdaptRate, float* disturbanceWindow, float* maximumTuning);
/**/
void vn100_setVpeAccelerometerAdvancedTuning(VnVector3 minimumFiltering, VnVector3 maximumFiltering,
	float maximumAdaptRate, float disturbanceWindow, float maximumTuning);
/**/
void vn100_getVpeGyroBasicTuning(VnVector3* varianceAngularWalk, VnVector3* baseTuning, 
	VnVector3* adaptiveTuning);
/**/
void vn100_setVpeGyroBasicTuning(VnVector3 varianceAngularWalk, VnVector3 baseTuning, 
	VnVector3 adaptiveTuning);
/**/
void vn100_getCalculatedMagnetometerCalibration(VnMatrix3x3* c, VnVector3* b);
/**/
void vn100_getFilterStartupGyroBias(VnVector3* gyroBias);
/**/
void vn100_setFilterStartupGyroBias(VnVector3 gyroBias);
/**/
void vn100_getFilterMeasurementVarianceParameters(double* angularWalkVariance, VnVector3* angularRateVariance, VnVector3* magneticVariance, VnVector3* accelerationVariance);
/**/
void vn100_setFilterMeasurementVarianceParameters(double angularWalkVariance, VnVector3 angularRateVariance,
	VnVector3 magneticVariance, VnVector3 accelerationVariance);
/**/
void vn100_getFilterActiveTuningParameters(double* magneticGain, double* accelerationGain, 
	double* magneticMemory, double* accelerationMemory);
/**/
void vn100_setFilterActiveTuningParameters(double magneticGain, double accelerationGain, 
	double magneticMemory, double accelerationMemory);
/**/

void vn100_getSynchronizationControl(unsigned char* syncInMode, unsigned char* syncInEdge, 
	unsigned short* syncInSkipFactor, unsigned int* reserved0, unsigned char* syncOutMode, 
	unsigned char* syncOutPolarity, unsigned short* syncOutSkipFactor, 
	unsigned int* syncOutPulseWidth, unsigned int* reserved1);
/**/

void vn100_setSynchronizationControl(unsigned char syncInMode, unsigned char syncInEdge,
	unsigned short syncInSkipFactor, unsigned int reserved0, unsigned char syncOutMode, 
	unsigned char syncOutPolarity, unsigned short syncOutSkipFactor, 
	unsigned int syncOutPulseWidth, unsigned int reserved1);

/**/
extern void vn100_getYawPitchRoll(void);
/**/
void vn100_getFilterBasicControl(unsigned char* magneticMode, 
	unsigned char* externalMagnetometerMode, 
	unsigned char* externalAccelerometerMode, 
	unsigned char* externalGyroscopeMode, VnVector3* angularRateLimit);
/**/
void vn100_setFilterBasicControl(unsigned char magneticMode, 
	unsigned char externalMagnetometerMode, 
	unsigned char externalAccelerometerMode, 
	unsigned char externalGyroscopeMode, 
	VnVector3 angularRateLimit);
/**/
void vn100_writeSettingsToVolatileMemory(void);
/**/
void vn100_confBinaryOutputMessage(BinaryMsg_vn100 msg);
/**/
uint16_T vn100_bitOffset_compute(MsgType_vn100 msgType);
/**/
extern void vn100_AsyncOutputPause(void);
/**/
extern void vn100_AsyncOutputResume(void);
/**/
extern void vn100_initialize(void);	
/**/
extern void vn100_initialize_defaults(void);
/**/
extern uint8_T vn100_check_crc(const char *message);
/**/

#endif



