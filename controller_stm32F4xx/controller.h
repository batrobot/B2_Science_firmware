/*
 * File: controller.h
 *
 * Code generated for Simulink model :controller.
 *
 * Model version      : 1.62
 * Simulink Coder version    : 8.6 (R2014a) 27-Dec-2013
 * TLC version       : 8.6 (Jan 30 2014)
 * C/C++ source code generated on  : Sat Jul 11 01:47:53 2015
 *
 * Target selection: stm32F4xx.tlc
 * Embedded hardware selection: STMicroelectronics->STM32F4xx 32-bit Cortex-M4
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. Debugging
 * Validation result: Passed (9), Warnings (3), Error (0)
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

#ifndef RTW_HEADER_controller_h_
#define RTW_HEADER_controller_h_
#ifndef controller_COMMON_INCLUDES_
# define controller_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* controller_COMMON_INCLUDES_ */

#include "controller_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((void*) 0)
#endif

/* Block signals and states (auto storage) for system '<Root>' */
typedef struct {
  real_T IC[2];                        /* '<Root>/IC' */
  real_T UD_DSTATE;                    /* '<S3>/UD' */
  real_T UD_DSTATE_e;                  /* '<S4>/UD' */
  real_T UD_DSTATE_o;                  /* '<S5>/UD' */
  boolean_T UnitDelay_DSTATE[2];       /* '<S1>/Unit Delay' */
  boolean_T IC_FirstOutputTime;        /* '<Root>/IC' */
} DW_controller;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real_T roll;                         /* '<Root>/roll' */
  real_T pitch;                        /* '<Root>/pitch' */
  real_T yaw;                          /* '<Root>/yaw' */
  real_T alpha[2];                     /* '<Root>/alpha' */
} ExtU_controller;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real_T left_w;                       /* '<Root>/left_w' */
  real_T right_w;                      /* '<Root>/right_w' */
  real_T left_t;                       /* '<Root>/left_t' */
  real_T right_t;                      /* '<Root>/right_t' */
  real_T rpy[3];                       /* '<Root>/rpy' */
  real_T ctrl[4];                      /* '<Root>/ctrl' */
} ExtY_controller;

/* Block signals and states (auto storage) */
extern DW_controller controller_DW;

/* External inputs (root inport signals with auto storage) */
extern ExtU_controller controller_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExtY_controller controller_Y;

/* Model entry point functions */
extern void controller_initialize(void);
extern void controller_step(void);

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
 * '<Root>' : 'controller'
 * '<S1>'   : 'controller/compute_rpy'
 * '<S2>'   : 'controller/func_controller'
 * '<S3>'   : 'controller/compute_rpy/Difference'
 * '<S4>'   : 'controller/compute_rpy/Difference1'
 * '<S5>'   : 'controller/compute_rpy/Difference2'
 * '<S6>'   : 'controller/compute_rpy/func_imu_rotation'
 */
#endif                                 /* RTW_HEADER_controller_h_ */

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF] controller.h
 */
