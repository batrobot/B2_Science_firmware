/*
 * File: controller.c
 *
 * Code generated for Simulink model :controller.
 *
 * Model version      : 1.63
 * Simulink Coder version    : 8.6 (R2014a) 27-Dec-2013
 * TLC version       : 8.6 (Jan 30 2014)
 * C/C++ source code generated on  : Wed Jul 29 19:06:07 2015
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

#include "controller.h"
#include "controller_private.h"

/* Block signals and states (auto storage) */
DW_controller controller_DW;

/* External inputs (root inport signals with auto storage) */
ExtU_controller controller_U;

/* External outputs (root outports fed by signals with auto storage) */
ExtY_controller controller_Y;

/* Model step function */
void controller_step(void)
{
  real_T u_r;
  real_T rtb_TmpSignalConversionAtSFun_0;
  boolean_T rtb_rollover_q_idx_0;
  boolean_T rtb_rollover_q_idx_1;
  real_T rtb_y_idx_1;
  real_T rtb_y_idx_2;
  real_T rtb_y_idx_3;

  /* SignalConversion: '<S6>/TmpSignal ConversionAt SFunction Inport2' incorporates:
   *  Inport: '<Root>/roll'
   *  MATLAB Function: '<S1>/func_imu_rotation'
   *  Sum: '<S3>/Diff'
   *  UnitDelay: '<S3>/UD'
   */
  rtb_TmpSignalConversionAtSFun_0 = controller_U.roll - controller_DW.UD_DSTATE;

  /* MATLAB Function: '<S1>/func_imu_rotation' incorporates:
   *  Inport: '<Root>/roll'
   *  SignalConversion: '<S6>/TmpSignal ConversionAt SFunction Inport1'
   *  UnitDelay: '<S1>/Unit Delay'
   */
  rtb_rollover_q_idx_0 = controller_DW.UnitDelay_DSTATE[0];
  rtb_rollover_q_idx_1 = controller_DW.UnitDelay_DSTATE[1];

  /* MATLAB Function 'compute_rpy/func_imu_rotation': '<S6>:1' */
  /*  func_imu_rotation */
  /*  u: imu measurments [roll, pitch,yaw].'        */
  /*  du: imu measurments @ (t+1) - imu measurments @ (t)          */
  /*  y: transformed measurments [roll,pitch,yaw].' */
  /*  NOTE: IMU is installed such that: */
  /*        x-axis is towards left arm wing (roll) */
  /*        y-axis is towards tail (pitch) */
  /*        z-axis is downwards (yaw) */
  /*  desired transformation: */
  /*        x-axis is forwards (roll) */
  /*        y-axis is towards left arm wing (pitch) */
  /*        z-axis is upwards (yaw) */
  /*  (rollover_p, rollover_q) internal variable */
  /*        rollover(1): when true rollover from 180 to -180 */
  /*        rollover(2): when true rollover from -180 to 180 */
  /* '<S6>:1:18' y = zeros(3,1); */
  /* '<S6>:1:20' u_r = u(1); */
  u_r = controller_U.roll;

  /* '<S6>:1:21' u_p = u(2); */
  /* '<S6>:1:22' u_y = u(3); */
  /* %% anti rollover for roll agnle %%%%% */
  /* '<S6>:1:26' threshold = 2*180 - 5; */
  /* '<S6>:1:27' if(du(1)>threshold) */
  if (rtb_TmpSignalConversionAtSFun_0 > 355.0) {
    /* '<S6>:1:28' rollover_p = [true,false].'; */
    rtb_rollover_q_idx_0 = true;
    rtb_rollover_q_idx_1 = false;
  }

  /* '<S6>:1:31' if(du(1)<-threshold) */
  if (rtb_TmpSignalConversionAtSFun_0 < -355.0) {
    /* '<S6>:1:32' rollover_p = [false,true].'; */
    rtb_rollover_q_idx_0 = false;
    rtb_rollover_q_idx_1 = true;
  }

  /* '<S6>:1:35' if rollover_p(1) */
  if (rtb_rollover_q_idx_0) {
    /* '<S6>:1:36' u_r = u_r - 180; */
    u_r = controller_U.roll - 180.0;
  }

  /* '<S6>:1:39' if rollover_p(2) */
  if (rtb_rollover_q_idx_1) {
    /* '<S6>:1:40' u_r = u_r + 180; */
    u_r += 180.0;
  }

  /* InitialCondition: '<Root>/IC' incorporates:
   *  Inport: '<Root>/alpha'
   */
  /* %% anti rollover for roll agnle %%%%% */
  /* '<S6>:1:44' y_r = u_p; */
  /* '<S6>:1:45' y_p = u_r; */
  /* '<S6>:1:46' y_y = -u_y; */
  /* '<S6>:1:48' y(1) = y_r; */
  /* '<S6>:1:49' y(2) = y_p; */
  /* '<S6>:1:50' y(3) = y_y; */
  /* '<S6>:1:52' rollover_q = rollover_p; */
  if (controller_DW.IC_FirstOutputTime) {
    controller_DW.IC_FirstOutputTime = false;
    controller_DW.IC[0] = 1.0;
    controller_DW.IC[1] = 1.0;
  } else {
    controller_DW.IC[0] = controller_U.alpha[0];
    controller_DW.IC[1] = controller_U.alpha[1];
  }

  /* End of InitialCondition: '<Root>/IC' */

  /* MATLAB Function: '<Root>/func_controller' incorporates:
   *  Inport: '<Root>/pitch'
   *  MATLAB Function: '<S1>/func_imu_rotation'
   *  SignalConversion: '<S6>/TmpSignal ConversionAt SFunction Inport1'
   */
  /* MATLAB Function 'func_controller': '<S2>:1' */
  /*  func_controller */
  /*  u: rpy, e.g., [roll,pitch,yaw].' */
  /*  alpha: control params, e.g., [roll_gain,pitch_gain] */
  /*  y: duty-cycle, e.g., [left_wing, right_wing, left_tail, right_tail] */
  /* '<S2>:1:7' y = zeros(4,1); */
  /* '<S2>:1:8' offset = 7.5; */
  /*  middle point of actuators */
  /*  left wing extends when roll < 0 */
  /* '<S2>:1:11' y(1) = alpha(1)*u(1)+offset; */
  rtb_TmpSignalConversionAtSFun_0 = controller_DW.IC[0] * controller_U.pitch +
    7.5;

  /*  right wing extends when roll > 0 */
  /* '<S2>:1:14' y(2) = -alpha(1)*u(1)+offset; */
  rtb_y_idx_1 = -controller_DW.IC[0] * controller_U.pitch + 7.5;

  /*  tails extend during pitch-down */
  /* '<S2>:1:17' y(3) = -alpha(2)*u(2)+offset; */
  rtb_y_idx_2 = -controller_DW.IC[1] * u_r + 7.5;

  /* '<S2>:1:18' y(4) = y(3); */
  rtb_y_idx_3 = rtb_y_idx_2;

  /* Saturate: '<Root>/duty-cycle(5-10%)' */
  if (rtb_TmpSignalConversionAtSFun_0 > 10.0) {
    rtb_TmpSignalConversionAtSFun_0 = 10.0;
  } else {
    if (rtb_TmpSignalConversionAtSFun_0 < 5.0) {
      rtb_TmpSignalConversionAtSFun_0 = 5.0;
    }
  }

  if (rtb_y_idx_1 > 10.0) {
    rtb_y_idx_1 = 10.0;
  } else {
    if (rtb_y_idx_1 < 5.0) {
      rtb_y_idx_1 = 5.0;
    }
  }

  if (rtb_y_idx_2 > 10.0) {
    rtb_y_idx_2 = 10.0;
  } else {
    if (rtb_y_idx_2 < 5.0) {
      rtb_y_idx_2 = 5.0;
    }
  }

  if (rtb_y_idx_3 > 10.0) {
    rtb_y_idx_3 = 10.0;
  } else {
    if (rtb_y_idx_3 < 5.0) {
      rtb_y_idx_3 = 5.0;
    }
  }

  /* End of Saturate: '<Root>/duty-cycle(5-10%)' */

  /* Outport: '<Root>/left_w' */
  controller_Y.left_w = rtb_TmpSignalConversionAtSFun_0;

  /* Outport: '<Root>/right_w' */
  controller_Y.right_w = rtb_y_idx_1;

  /* Outport: '<Root>/left_t' */
  controller_Y.left_t = rtb_y_idx_2;

  /* Outport: '<Root>/right_t' */
  controller_Y.right_t = rtb_y_idx_3;

  /* Outport: '<Root>/rpy' incorporates:
   *  Inport: '<Root>/pitch'
   *  Inport: '<Root>/yaw'
   *  MATLAB Function: '<S1>/func_imu_rotation'
   *  SignalConversion: '<S6>/TmpSignal ConversionAt SFunction Inport1'
   */
  controller_Y.rpy[0] = controller_U.pitch;
  controller_Y.rpy[1] = u_r;
  controller_Y.rpy[2] = -controller_U.yaw;

  /* Outport: '<Root>/ctrl' */
  controller_Y.ctrl[0] = rtb_TmpSignalConversionAtSFun_0;
  controller_Y.ctrl[1] = rtb_y_idx_1;
  controller_Y.ctrl[2] = rtb_y_idx_2;
  controller_Y.ctrl[3] = rtb_y_idx_3;

  /* Update for UnitDelay: '<S3>/UD' incorporates:
   *  Inport: '<Root>/roll'
   */
  controller_DW.UD_DSTATE = controller_U.roll;

  /* Update for UnitDelay: '<S4>/UD' incorporates:
   *  Inport: '<Root>/pitch'
   */
  controller_DW.UD_DSTATE_e = controller_U.pitch;

  /* Update for UnitDelay: '<S5>/UD' incorporates:
   *  Inport: '<Root>/yaw'
   */
  controller_DW.UD_DSTATE_o = controller_U.yaw;

  /* Update for UnitDelay: '<S1>/Unit Delay' */
  controller_DW.UnitDelay_DSTATE[0] = rtb_rollover_q_idx_0;
  controller_DW.UnitDelay_DSTATE[1] = rtb_rollover_q_idx_1;
}

/* Model initialize function */
void controller_initialize(void)
{
  /* Start for InitialCondition: '<Root>/IC' */
  controller_DW.IC[0] = 1.0;
  controller_DW.IC[1] = 1.0;
  controller_DW.IC_FirstOutputTime = true;
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF] controller.c
 */
