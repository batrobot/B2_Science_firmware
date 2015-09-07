/*
 * File: daq.h
 *
 * Code generated for Simulink model :daq.
 *
 * Model version      : 1.65
 * Simulink Coder version    : 8.6 (R2014a) 27-Dec-2013
 * TLC version       : 8.6 (Jan 30 2014)
 * C/C++ source code generated on  : Thu Sep 03 12:59:16 2015
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

#ifndef RTW_HEADER_daq_h_
#define RTW_HEADER_daq_h_
#include <string.h>
#include "stm32f4xx.h"
#ifndef daq_COMMON_INCLUDES_
# define daq_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* daq_COMMON_INCLUDES_ */

#include "daq_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((void*) 0)
#endif

/* Block signals (auto storage) */
typedef struct {
  uint16_T GPIO_Read3;                 /* '<S6>/GPIO_Read3' */
  uint16_T GPIO_Read1;                 /* '<S6>/GPIO_Read1' */
  uint16_T GPIO_Read2;                 /* '<S6>/GPIO_Read2' */
  uint16_T GPIO_Read4;                 /* '<S6>/GPIO_Read4' */
  uint16_T GPIO_Read5;                 /* '<S6>/GPIO_Read5' */
} B_daq;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  int16_T pwm_pa0;                     /* '<Root>/pwm_pa0' */
  int16_T pwm_pa1;                     /* '<Root>/pwm_pa1' */
  int16_T pwm_pa2;                     /* '<Root>/pwm_pa2' */
  uint16_T usart1_Nb2Send;             /* '<Root>/usart1_Nb2Send' */
  uint8_T usart1_SendVal[1000];        /* '<Root>/usart1_SendVal' */
  uint16_T usart3_Nb2Send;             /* '<Root>/usart3_Nb2Send' */
  uint8_T usart3_SendVal[1000];        /* '<Root>/usart3_SendVal' */
  uint16_T usart6_Nb2Send;             /* '<Root>/usart6_Nb2Send' */
  uint8_T usart6_SendVal[1000];        /* '<Root>/usart6_SendVal' */
  real_T di1;                          /* '<Root>/di1' */
  real_T di2;                          /* '<Root>/di2' */
  real_T di3;                          /* '<Root>/di3' */
  real_T di4;                          /* '<Root>/di4' */
  real_T di5;                          /* '<Root>/di5' */
  real_T di6;                          /* '<Root>/di6' */
  real_T di7;                          /* '<Root>/di7' */
  real_T di8;                          /* '<Root>/di8' */
  real_T di9;                          /* '<Root>/di9' */
  real_T di10;                         /* '<Root>/di10' */
  real_T di11;                         /* '<Root>/di11' */
  real_T di12;                         /* '<Root>/di12' */
  real_T di13;                         /* '<Root>/di13' */
  real_T di14;                         /* '<Root>/di14' */
  real_T di15;                         /* '<Root>/di15' */
  real_T usart2_Nb2Send;               /* '<Root>/usart2_Nb2Send' */
  real_T usart2_SendVal;               /* '<Root>/usart2_SendVal' */
  int16_T pwm_pa6;                     /* '<Root>/pwm_pa6' */
  int16_T pwm_pa7;                     /* '<Root>/pwm_pa7' */
  int16_T pwm_pb0;                     /* '<Root>/pwm_pb0' */
} ExtU_daq;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real_T time;                         /* '<Root>/time' */
  uint16_T usart1_NbSent;              /* '<Root>/	usart1_NbSent' */
  real_T usart1_NbRcv;                 /* '<Root>/usart1_NbRcv' */
  uint8_T usart1_RcvVal[500];          /* '<Root>/usart1_RcvVal' */
  uint16_T usart3_NbSent;              /* '<Root>/	usart3_NbSent' */
  real_T usart3_NbRcv;                 /* '<Root>/usart3_NbRcv' */
  uint8_T usart3_RcvVal[100];          /* '<Root>/usart3_RcvVal' */
  uint16_T usart6_NbSent;              /* '<Root>/	usart6_NbSent' */
  real_T usart6_NbRcv;                 /* '<Root>/usart6_NbRcv' */
  uint8_T usart6_RcvVal[100];          /* '<Root>/usart6_RcvVal' */
  real_T ADC123_IN0;                   /* '<Root>/ADC123_IN0' */
  real_T ADC123_IN3;                   /* '<Root>/ADC123_IN3' */
  real_T Estop;                        /* '<Root>/Estop' */
  real_T do1;                          /* '<Root>/do1' */
  real_T do2;                          /* '<Root>/do2' */
  real_T imuData;                      /* '<Root>/imuData' */
  real_T usart2_NbRcv;                 /* '<Root>/usart2_NbRcv' */
  real_T usart2_RcvVal;                /* '<Root>/usart2_RcvVal' */
  uint16_T pe2;                        /* '<Root>/pe2' */
  uint16_T pe3;                        /* '<Root>/pe3' */
  uint16_T pe4;                        /* '<Root>/pe4' */
} ExtY_daq;

/* Parameters (auto storage) */
struct P_daq_ {
  real_T pwm_freq_Value;               /* Expression: 1000
                                        * Referenced by: '<S2>/pwm_freq'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_daq {
  const char_T *errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
  } Timing;
};

/* Block parameters (auto storage) */
extern P_daq daq_P;

/* Block signals (auto storage) */
extern B_daq daq_B;

/* External inputs (root inport signals with auto storage) */
extern ExtU_daq daq_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExtY_daq daq_Y;

/* Model entry point functions */
extern void daq_initialize(void);
extern void daq_step(void);

/* Real-time Model object */
extern RT_MODEL_daq *const daq_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'daq'
 * '<S1>'   : 'daq/ADC Config'
 * '<S2>'   : 'daq/DAQ'
 * '<S3>'   : 'daq/GPIO Config'
 * '<S4>'   : 'daq/USART Config'
 * '<S5>'   : 'daq/DAQ/ADC'
 * '<S6>'   : 'daq/DAQ/GPIO'
 * '<S7>'   : 'daq/DAQ/PWM'
 * '<S8>'   : 'daq/DAQ/USART'
 * '<S9>'   : 'daq/DAQ/GPIO/MATLAB Function'
 * '<S10>'  : 'daq/DAQ/USART/USART1'
 * '<S11>'  : 'daq/DAQ/USART/USART3'
 * '<S12>'  : 'daq/DAQ/USART/USART6'
 */
#endif                                 /* RTW_HEADER_daq_h_ */

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF] daq.h
 */
