/*
 * File: daq_types.h
 *
 * Code generated for Simulink model :daq.
 *
 * Model version      : 1.18
 * Simulink Coder version    : 8.6 (R2014a) 27-Dec-2013
 * TLC version       : 8.6 (Jan 30 2014)
 * C/C++ source code generated on  : Sun Jun 07 16:10:57 2015
 *
 * Target selection: stm32F4xx.tlc
 * Embedded hardware selection: STMicroelectronics->STM32F4xx 32-bit Cortex-M4
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. Debugging
 * Validation result: Not run
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

#ifndef RTW_HEADER_daq_types_h_
#define RTW_HEADER_daq_types_h_
#include "rtwtypes.h"

/* Custom Type definition for MATLAB Function: '<S9>/parse_imu' */
#ifndef struct_sNTFgGeQIsf0onppazMo7jE
#define struct_sNTFgGeQIsf0onppazMo7jE

struct sNTFgGeQIsf0onppazMo7jE
{
  real_T timeStartup;
  real_T TimeGPS;
  real_T TimeSyncIn;
  real_T Ypr;
  real_T Qtn;
  real_T AngRate;
  real_T Pos;
  real_T Vel;
  real_T Accel;
  real_T Imu;
  real_T Magpres;
  real_T rcvMsgSize;
  real_T sentMsgSize;
};

#endif                                 /*struct_sNTFgGeQIsf0onppazMo7jE*/

#ifndef typedef_sNTFgGeQIsf0onppazMo7jE_daq
#define typedef_sNTFgGeQIsf0onppazMo7jE_daq

typedef struct sNTFgGeQIsf0onppazMo7jE sNTFgGeQIsf0onppazMo7jE_daq;

#endif                                 /*typedef_sNTFgGeQIsf0onppazMo7jE_daq*/

#ifndef struct_skRxmPEbR1piRwKCnTJxF4D
#define struct_skRxmPEbR1piRwKCnTJxF4D

struct skRxmPEbR1piRwKCnTJxF4D
{
  char_T timeStartup[6];
  char_T TimeGPS[6];
  char_T TimeSyncIn[6];
  char_T Ypr[6];
  char_T Qtn[6];
  char_T AngRate[6];
  char_T Pos[6];
  char_T Vel[6];
  char_T Accel[6];
  char_T Imu[6];
  char_T Magpres[6];
};

#endif                                 /*struct_skRxmPEbR1piRwKCnTJxF4D*/

#ifndef typedef_skRxmPEbR1piRwKCnTJxF4D_daq
#define typedef_skRxmPEbR1piRwKCnTJxF4D_daq

typedef struct skRxmPEbR1piRwKCnTJxF4D skRxmPEbR1piRwKCnTJxF4D_daq;

#endif                                 /*typedef_skRxmPEbR1piRwKCnTJxF4D_daq*/

#ifndef struct_sDkgQgznLaVlrrSNjbnyqtF
#define struct_sDkgQgznLaVlrrSNjbnyqtF

struct sDkgQgznLaVlrrSNjbnyqtF
{
  boolean_T timeStartup;
  boolean_T TimeGPS;
  boolean_T TimeSyncIn;
  boolean_T Ypr;
  boolean_T Qtn;
  boolean_T AngRate;
  boolean_T Pos;
  boolean_T Vel;
  boolean_T Accel;
  boolean_T Imu;
  boolean_T Magpres;
  sNTFgGeQIsf0onppazMo7jE_daq size;
  skRxmPEbR1piRwKCnTJxF4D_daq type;
  uint8_T header[4];
};

#endif                                 /*struct_sDkgQgznLaVlrrSNjbnyqtF*/

#ifndef typedef_sDkgQgznLaVlrrSNjbnyqtF_daq
#define typedef_sDkgQgznLaVlrrSNjbnyqtF_daq

typedef struct sDkgQgznLaVlrrSNjbnyqtF sDkgQgznLaVlrrSNjbnyqtF_daq;

#endif                                 /*typedef_sDkgQgznLaVlrrSNjbnyqtF_daq*/

#ifndef struct_ssaMv3AXPYjdjqzhWTEFbaC
#define struct_ssaMv3AXPYjdjqzhWTEFbaC

struct ssaMv3AXPYjdjqzhWTEFbaC
{
  real_T serialPort;
  real_T devisor;
  sDkgQgznLaVlrrSNjbnyqtF_daq msgType;
};

#endif                                 /*struct_ssaMv3AXPYjdjqzhWTEFbaC*/

#ifndef typedef_ssaMv3AXPYjdjqzhWTEFbaC_daq
#define typedef_ssaMv3AXPYjdjqzhWTEFbaC_daq

typedef struct ssaMv3AXPYjdjqzhWTEFbaC ssaMv3AXPYjdjqzhWTEFbaC_daq;

#endif                                 /*typedef_ssaMv3AXPYjdjqzhWTEFbaC_daq*/

/* Parameters (auto storage) */
typedef struct P_daq_ P_daq;

/* Forward declaration for rtModel */
typedef struct tag_RTM_daq RT_MODEL_daq;

#endif                                 /* RTW_HEADER_daq_types_h_ */

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF] daq_types.h
 */
