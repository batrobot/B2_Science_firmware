/*
 * File: controller.c
 *
 * Code generated for Simulink model :controller.
 *
 * Model version      : 1.136
 * Simulink Coder version    : 8.6 (R2014a) 27-Dec-2013
 * TLC version       : 8.6 (Jan 30 2014)
 * C/C++ source code generated on  : Sun Sep 06 11:06:16 2015
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

/* Real-time model */
RT_MODEL_controller controller_M_;
RT_MODEL_controller *const controller_M = &controller_M_;

/* Model step function */
void controller_step(void)
{
  /* local block i/o variables */
  real_T rtb_DigitalClock;
  real_T sensitivity_matrix[16];
  int32_T argout;
  int32_T b_argout;
  int32_T c_argout;
  int32_T d_argout;
  int32_T e_argout;
  int32_T f_argout;
  int32_T g_argout;
  int32_T h_argout;
  static const int8_T b[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

  real_T rtb_us[8];
  real_T rtb_q[12];
  real_T rtb_uf[4];
  real_T tmp[4];
  real_T tmp_0[16];
  real_T tmp_1[16];
  real_T tmp_2[4];
  int8_T argout_0[32];
  real_T rtb_uf_a;
  real_T max_angle_idx_0;
  real_T max_angle_idx_1;
  real_T max_angle_idx_2;
  real_T max_angle_idx_3;
  real_T u_pid_idx_0;
  real_T u_pid_idx_1;
  real_T u_pid_idx_2;
  real_T u_pid_idx_3;
  real_T angle_difference_idx_0;
  real_T angle_difference_idx_1;
  real_T angle_difference_idx_2;
  real_T angle_difference_idx_3;
  real_T angle_aro_idx_0;
  real_T angle_aro_idx_1;
  real_T angle_aro_idx_2;
  real_T angle_aro_idx_3;
  real_T rtb_uf_idx_1;

  /* MATLAB Function: '<S4>/func_state_estimator' incorporates:
   *  Inport: '<Root>/pitch'
   *  Inport: '<Root>/roll'
   *  Inport: '<Root>/yaw'
   *  SignalConversion: '<S8>/TmpSignal ConversionAt SFunction Inport1'
   */
  /* MATLAB Function 'state estimator/func_state_estimator': '<S8>:1' */
  /*  state estimator */
  /*  u: imu measurments  */
  /*     imu_roll [rad] */
  /*     imu_pitch */
  /*     imu_yaw */
  /*  du: imu measurments @ (t+1) - imu measurments @ (t) */
  /*  q: estimated state  */
  /*    qr: vehicle roll [rad] */
  /*    qp: vehicle pitch */
  /*    qy: vehicle yaw */
  /*    px: vehicle x-pos [m] */
  /*    py: vehicle y-pos */
  /*    pz: vehicle z-pos */
  /*    dqr: vehicle roll rate [rad\sec] */
  /*    dqp: vehicle pitch rate */
  /*    dqy: vehicle yaw rate */
  /*    dpx: vehicle x-vel [m\sec] */
  /*    dpy: vehicle y-vel */
  /*    dpz: vehicle z-vel */
  /*  NOTE: IMU is installed such that: */
  /*        x-axis is towards left arm wing (pitch) */
  /*        y-axis is towards head (roll) */
  /*        z-axis is downwards (yaw) */
  /*  By Alireza Ramezani, 9-5-2015, Champaign, IL */
  /* '<S8>:1:27' q = zeros(12,1); */
  /* '<S8>:1:28' DEG2RAD = pi/180; */
  /*  rad\deg */
  /* '<S8>:1:30' q = DEG2RAD*q; */
  memset(&rtb_q[0], 0, 12U * sizeof(real_T));

  /*  roll */
  /* '<S8>:1:33' q(1) = u(2); */
  rtb_q[0] = controller_U.pitch;

  /*  pitch */
  /* '<S8>:1:36' q(2) = u(1); */
  rtb_q[1] = controller_U.roll;

  /*  yaw  */
  /* '<S8>:1:39' q(3) = u(3); */
  rtb_q[2] = controller_U.yaw;

  /* Outport: '<Root>/q' */
  /*  % roll rate */
  /*  q(7) = du(2); */
  /*   */
  /*  % pitch rate */
  /*  q(8) = du(1); */
  /*   */
  /*  % yaw rate */
  /*  q(9) = du(3); */
  memcpy(&controller_Y.q[0], &rtb_q[0], 12U * sizeof(real_T));

  /* InitialCondition: '<Root>/IC' incorporates:
   *  Inport: '<Root>/flight_ctrl_params'
   */
  if (controller_DW.IC_FirstOutputTime) {
    controller_DW.IC_FirstOutputTime = false;
    memset(&controller_DW.IC[0], 0, 10U * sizeof(real_T));
  } else {
    memcpy(&controller_DW.IC[0], &controller_U.flight_ctrl_params[0], 10U *
           sizeof(real_T));
  }

  /* End of InitialCondition: '<Root>/IC' */

  /* MATLAB Function: '<Root>/func_flight_controller' incorporates:
   *  Inport: '<Root>/pitch'
   *  Inport: '<Root>/roll'
   *  MATLAB Function: '<S4>/func_state_estimator'
   *  SignalConversion: '<S8>/TmpSignal ConversionAt SFunction Inport1'
   */
  /* MATLAB Function 'func_flight_controller': '<S3>:1' */
  /*  func_flight_controller */
  /*  q: vehicle state, e.g., [qr,qp,qy,px,py,pz,dqr,dqp,dqy,dpx,dpy,dpz].' */
  /*    qr: roll [rad] */
  /*    qp: pitch [rad] */
  /*    qy: yaw [rad] */
  /*    px: x-position [m] */
  /*    py: y-position [m] */
  /*    pz: z-position [m] */
  /*    dqr: roll rate [rad\sec] */
  /*    dqp: pitch rate [rad\sec] */
  /*    dqy: yaw rate [rad\sec] */
  /*    dpx: x-vel [m/s] */
  /*    dpy: y-vel [m/s] */
  /*    dpz: z-vel [m/s] */
  /*  */
  /*  flight_ctrl_params: controller params, e.g.,  */
  /*    flight_ctrl_params(1): Roll sensitivity */
  /*    flight_ctrl_params(2): Pitch sensitivity */
  /*    flight_ctrl_params(3): max movement for left and right forelimb (NOTE: from equilibrium) */
  /*    flight_ctrl_params(4): min movement for left and right forelimb (NOTE: from equilibrium) */
  /*    flight_ctrl_params(5): max movement for left and right leg (NOTE: from equilibrium) */
  /*    flight_ctrl_params(6): min movement for left and right leg (NOTE: from equilibrium) */
  /*    flight_ctrl_params(7): equilbirium for right forelimb */
  /*    flight_ctrl_params(8): equilbirium for left forelimb */
  /*    flight_ctrl_params(9): equilbirium for right leg */
  /*    flight_ctrl_params(10): equilbirium for left leg */
  /*  */
  /*  uf: flight controller output, e.g., position of actuators */
  /*    uf(1): Right forelimb RP angle */
  /*    uf(2): Left forelimb RP angle */
  /*    uf(3): Right leg DV angle */
  /*    uf(4): Left leg DV angle */
  /*   */
  /*  By Alireza Ramezani, 9-5-2015, Champaign, IL */
  /* '<S3>:1:37' uf = zeros(4,1); */
  /* '<S3>:1:39' DEG2RAD = pi/180; */
  /*  rad\deg */
  /* '<S3>:1:41' ROLL_SENSITIVITY = flight_ctrl_params(1); */
  /* '<S3>:1:42' PITCH_SENSITIVITY = flight_ctrl_params(2); */
  /* '<S3>:1:43' MAX_FORELIMB_ANGLE = flight_ctrl_params(3); */
  /* '<S3>:1:44' MIN_FORELIMB_ANGLE = flight_ctrl_params(4); */
  /* '<S3>:1:45' MAX_LEG_ANGLE = flight_ctrl_params(5); */
  /* '<S3>:1:46' MIN_LEG_ANGLE = flight_ctrl_params(6); */
  /* '<S3>:1:47' R_foreq = flight_ctrl_params(7); */
  /* '<S3>:1:48' L_foreq = flight_ctrl_params(8); */
  /* '<S3>:1:49' R_leq = flight_ctrl_params(9); */
  /* '<S3>:1:50' L_leq = flight_ctrl_params(10); */
  /*  equilibrium */
  /* '<S3>:1:53' eq = DEG2RAD*[R_foreq,L_foreq,R_leq,L_leq].'; */
  /*  limits */
  /* '<S3>:1:56' max_angle = DEG2RAD*[MAX_FORELIMB_ANGLE,MAX_FORELIMB_ANGLE,MAX_LEG_ANGLE,MAX_LEG_ANGLE].'; */
  /* '<S3>:1:57' min_angle = DEG2RAD*[MIN_FORELIMB_ANGLE,MIN_FORELIMB_ANGLE,MIN_LEG_ANGLE,MIN_LEG_ANGLE].'; */
  /*  comptue control */
  /* '<S3>:1:60' sensitivity_matrix = diag([ROLL_SENSITIVITY,ROLL_SENSITIVITY,PITCH_SENSITIVITY,PITCH_SENSITIVITY]); */
  memset(&sensitivity_matrix[0], 0, sizeof(real_T) << 4U);
  sensitivity_matrix[0] = controller_DW.IC[0];
  sensitivity_matrix[5] = controller_DW.IC[0];
  sensitivity_matrix[10] = controller_DW.IC[1];
  sensitivity_matrix[15] = controller_DW.IC[1];

  /*  roll */
  /*  pitch */
  /* '<S3>:1:66' angle = [q(1),q(1),q(2),q(2)].'; */
  /* '<S3>:1:67' uf = sensitivity_matrix*angle + eq; */
  tmp[0] = controller_DW.IC[6];
  tmp[1] = controller_DW.IC[7];
  tmp[2] = controller_DW.IC[8];
  tmp[3] = controller_DW.IC[9];
  for (argout = 0; argout < 4; argout++) {
    rtb_uf[argout] = (((sensitivity_matrix[argout + 4] * controller_U.pitch +
                        sensitivity_matrix[argout] * controller_U.pitch) +
                       sensitivity_matrix[argout + 8] * controller_U.roll) +
                      sensitivity_matrix[argout + 12] * controller_U.roll) +
      0.017453292519943295 * tmp[argout];
  }

  /* End of MATLAB Function: '<Root>/func_flight_controller' */

  /* Outport: '<Root>/flight_ctrl' */
  /*  limit control effort */
  /*  for i=1:4 */
  /*      if(uf(i)>max_angle(i)) */
  /*          uf(i) = max_angle(i); */
  /*      end */
  /*      if(uf(i)<min_angle(i)) */
  /*          uf(i) = min_angle(i); */
  /*      end */
  /*  end */
  controller_Y.flight_ctrl[0] = rtb_uf[0];
  controller_Y.flight_ctrl[1] = rtb_uf[1];
  controller_Y.flight_ctrl[2] = rtb_uf[2];
  controller_Y.flight_ctrl[3] = rtb_uf[3];

  /* InitialCondition: '<Root>/IC2' incorporates:
   *  Inport: '<Root>/actuator_ctrl_params'
   */
  if (controller_DW.IC2_FirstOutputTime) {
    controller_DW.IC2_FirstOutputTime = false;
    memcpy(&controller_DW.IC2[0], &controller_ConstP.IC2_Value[0], 11U * sizeof
           (real_T));
  } else {
    memcpy(&controller_DW.IC2[0], &controller_U.actuator_ctrl_params[0], 11U *
           sizeof(real_T));
  }

  /* End of InitialCondition: '<Root>/IC2' */

  /* MATLAB Function: '<Root>/func_actuator_controller' incorporates:
   *  Inport: '<Root>/angle'
   *  Inport: '<Root>/pid_gian'
   *  UnitDelay: '<Root>/Unit Delay'
   *  UnitDelay: '<Root>/Unit Delay1'
   *  UnitDelay: '<Root>/Unit Delay2'
   */
  /* MATLAB Function 'func_actuator_controller': '<S2>:1' */
  /*  func_actuator_controller: uses a PID scheme to position wing and tail */
  /*  actuators */
  /*  */
  /*  angle: encoder measurments, in deg, measurements are not filters, 4-by-1 */
  /*  vecot */
  /*    angle(1): right forelimb */
  /*    angle(2): left forelimb */
  /*    angle(3): right leg */
  /*    angle(4): left leg */
  /*  angle_prev: memory for previous raw measurments, 4-by-1 vector */
  /*    angle_prev(1): right forelimb */
  /*    angle_prev(2): left forelimb */
  /*    angle_prev(3): right leg */
  /*    angle_prev(4): left leg */
  /*  angle_aro_prev: memory for previous anti rolled over mearumetns, 4-by-n */
  /*  vector */
  /*    angle_aro_prev(1,i): right forelimb @ time(i) */
  /*    angle_aro_prev(2,i): left forelimb @ time(i) */
  /*    angle_aro_prev(3,i): right leg @ time(i) */
  /*    angle_aro_prev(4,i): left leg @ time(i) */
  /*  uf: flight control inputs, 4-by-1 vector */
  /*    uf(1): right forelimb */
  /*    uf(2): left forelimb */
  /*    uf(3): right leg */
  /*    uf(4): left leg */
  /*  prev_err: memory for err vector 4-by-1 vec */
  /*    prev_err(1): right forelimb */
  /*    prev_err(2): left forelimb */
  /*    prev_err(3): right leg */
  /*    prev_err(4): left leg */
  /*  gain: control gain vector 2-by-3 vec */
  /*    gain(1,1:2): right forelimb Kp and Kd */
  /*    gain(1,1:2): left forelimb  Kp and Kd */
  /*    gain(2,1:2): right leg Kp and Kd */
  /*    gain(2,1:2): left leg Kp and Kd */
  /*  ctrl_param: params */
  /*    ctrl_param(1): PID_SATURATION_THRESHOLD (control sat, used in pid func) */
  /*    ctrl_param(2): MAX_ANGLE_DIFFERENCE (deg\sec, used in anti-roll-over func) */
  /*    ctrl_param(3): ANTI_ROLLOVER_CORRECTION (deg, used in anti-roll-over func) */
  /*    ctrl_param(4): MAX_RP_ANGLE (deg, same for left and right armwings) */
  /*    ctrl_param(5): MAX_DV_ANGLE (deg, same for left and right legs) */
  /*    ctrl_param(6): MIN_RP_ANGLE (deg, same for left and right armwings) */
  /*    ctrl_param(7): MIN_DV_ANGLE (deg, same for left and right legs) */
  /*  us: micro actuator control inputs, 4-by-1 vector */
  /*    us(1): right forelimb */
  /*    us(2): left forelimb */
  /*    us(3): right leg */
  /*    us(4): left leg */
  /*  angle2mem: to keep angle in memory for one sample time */
  /*  */
  /*  angle_aro2mem: to keep angle_aro in memory for one sample time */
  /*  */
  /*  err2mem: to keep error in memory for one sample time */
  /*  */
  /*  debug: debugging channel, i.e., used for debugging purpose. */
  /*  */
  /*  by ALireza Ramezani, 8-31-2015, Champaign, IL */
  /* '<S2>:1:60' us = zeros(8,1); */
  /* '<S2>:1:61' angle_aro = zeros(4,1); */
  /*  anti rolled-over */
  /* '<S2>:1:62' angle_f = zeros(4,1); */
  /*  filtered */
  /* '<S2>:1:63' u_pid = zeros(4,1); */
  /*  computer pid */
  /* '<S2>:1:64' angle2mem = zeros(4,1); */
  /* '<S2>:1:65' angle_aro2mem = zeros(4,1); */
  /* '<S2>:1:66' err2mem = zeros(4,1); */
  /* '<S2>:1:67' debug = zeros(4,1); */
  /*  global vars */
  /* '<S2>:1:82' PID_SATURATION_THRESHOLD = ctrl_param(1); */
  controller_DW.PID_SATURATION_THRESHOLD = controller_DW.IC2[0];

  /* '<S2>:1:83' MAX_ANGLE_DIFFERENCE = ctrl_param(2); */
  controller_DW.MAX_ANGLE_DIFFERENCE = controller_DW.IC2[1];

  /* '<S2>:1:84' ANTI_ROLLOVER_CORRECTION = ctrl_param(3); */
  controller_DW.ANTI_ROLLOVER_CORRECTION = controller_DW.IC2[2];

  /* '<S2>:1:85' MAX_RP_ANGLE_RIGHT = ctrl_param(4); */
  controller_DW.MAX_RP_ANGLE_RIGHT = controller_DW.IC2[3];

  /* '<S2>:1:86' MAX_DV_ANGLE_RIGHT = ctrl_param(5); */
  controller_DW.MAX_DV_ANGLE_RIGHT = controller_DW.IC2[4];

  /* '<S2>:1:87' MIN_RP_ANGLE_RIGHT = ctrl_param(6); */
  controller_DW.MIN_RP_ANGLE_RIGHT = controller_DW.IC2[5];

  /* '<S2>:1:88' MIN_DV_ANGLE_RIGHT = ctrl_param(7); */
  controller_DW.MIN_DV_ANGLE_RIGHT = controller_DW.IC2[6];

  /* '<S2>:1:89' MAX_RP_ANGLE_LEFT = ctrl_param(8); */
  controller_DW.MAX_RP_ANGLE_LEFT = controller_DW.IC2[7];

  /* '<S2>:1:90' MAX_DV_ANGLE_LEFT = ctrl_param(9); */
  controller_DW.MAX_DV_ANGLE_LEFT = controller_DW.IC2[8];

  /* '<S2>:1:91' MIN_RP_ANGLE_LEFT = ctrl_param(10); */
  controller_DW.MIN_RP_ANGLE_LEFT = controller_DW.IC2[9];

  /* '<S2>:1:92' MIN_DV_ANGLE_LEFT = ctrl_param(11); */
  controller_DW.MIN_DV_ANGLE_LEFT = controller_DW.IC2[10];

  /*  angle preprocessing */
  /*  NOTE: output is rad. */
  /* '<S2>:1:96' angle_aro = func_anti_rollOver(angle,angle_prev); */
  /*  func_anti_rollOver: prevent roll-over in as5048B (it rolles over at 360 deg) */
  /*  angle: encoder measurments, in deg, measurements are not filters, 4-by-1 */
  /*  vecot */
  /*    angle(1): right forelimb */
  /*    angle(2): left forelimb */
  /*    angle(3): right leg */
  /*    angle(4): left leg */
  /*  angle_prev: measurments from previous sample time */
  /*    angle_prev(1): right forelimb */
  /*    angle_prev(2): left forelimb */
  /*    angle_prev(3): right leg */
  /*    angle_prev(4): left leg */
  /*  angle_aro: anti-rolled over angles */
  /*  vecot */
  /*    angle_aro(1): right forelimb */
  /*    angle_aro(2): left forelimb */
  /*    angle_aro(3): right leg */
  /*    angle_aro(4): left leg */
  /*  by ALireza Ramezani, 8-31-2015, Champaign, IL */
  /* 'func_anti_rollOver:22' angle_difference = zeros(4,1); */
  /* 'func_anti_rollOver:23' angle_aro = zeros(4,1); */
  /*  guess max angular changes during one sample time */
  /* 'func_anti_rollOver:38' DEG2RAD = pi/180; */
  /*  rad\deg */
  /*  rollover matrix */
  /* 'func_anti_rollOver:41' ANTI_ROLLOVER_CORRECTION_MATRIX = ANTI_ROLLOVER_CORRECTION*eye(4,4); */
  /*  initiate angle aro */
  /* 'func_anti_rollOver:44' angle_aro = angle; */
  /*  ANTI_ROLLOVER_THRESHOLD = ANTI_ROLLOVER_CORRECTION - MAX_ANGLE_DIFFERENCE; % deg */
  /*  comptue the difference */
  /* 'func_anti_rollOver:49' angle_difference = angle - angle_prev; */
  angle_difference_idx_0 = controller_U.angle[0] -
    controller_DW.UnitDelay_DSTATE[0];
  angle_difference_idx_1 = controller_U.angle[1] -
    controller_DW.UnitDelay_DSTATE[1];
  angle_difference_idx_2 = controller_U.angle[2] -
    controller_DW.UnitDelay_DSTATE[2];
  angle_difference_idx_3 = controller_U.angle[3] -
    controller_DW.UnitDelay_DSTATE[3];

  /*  for i=1:4 */
  /*      if (angle_difference(i) > -ANTI_ROLLOVER_THRESHOLD)||(angle_difference(i) < ANTI_ROLLOVER_THRESHOLD) */
  /*          if(angle_difference(i)<0) */
  /*              angle_aro(i) = angle(i) + ANTI_ROLLOVER_CORRECTION; */
  /*          else */
  /*              angle_aro(i) = angle(i) - ANTI_ROLLOVER_CORRECTION; */
  /*          end */
  /*      end */
  /*  end */
  /* 'func_anti_rollOver:63' for i=1:4 */
  /* 'func_anti_rollOver:64' if (angle_difference(i) > MAX_ANGLE_DIFFERENCE)||(angle_difference(i) < -MAX_ANGLE_DIFFERENCE) */
  if ((angle_difference_idx_0 > controller_DW.MAX_ANGLE_DIFFERENCE) ||
      (angle_difference_idx_0 < -controller_DW.MAX_ANGLE_DIFFERENCE)) {
    /* 'func_anti_rollOver:65' if(angle_difference(i)<0) */
    if (angle_difference_idx_0 < 0.0) {
      /* 'func_anti_rollOver:66' ROLLOVER_FLAG(i) = ROLLOVER_FLAG(i) + 1; */
      controller_DW.ROLLOVER_FLAG[0]++;
    } else {
      /* 'func_anti_rollOver:67' else */
      /* 'func_anti_rollOver:68' ROLLOVER_FLAG(i) = ROLLOVER_FLAG(i) - 1; */
      controller_DW.ROLLOVER_FLAG[0]--;
    }
  }

  /* 'func_anti_rollOver:64' if (angle_difference(i) > MAX_ANGLE_DIFFERENCE)||(angle_difference(i) < -MAX_ANGLE_DIFFERENCE) */
  if ((angle_difference_idx_1 > controller_DW.MAX_ANGLE_DIFFERENCE) ||
      (angle_difference_idx_1 < -controller_DW.MAX_ANGLE_DIFFERENCE)) {
    /* 'func_anti_rollOver:65' if(angle_difference(i)<0) */
    if (angle_difference_idx_1 < 0.0) {
      /* 'func_anti_rollOver:66' ROLLOVER_FLAG(i) = ROLLOVER_FLAG(i) + 1; */
      controller_DW.ROLLOVER_FLAG[1]++;
    } else {
      /* 'func_anti_rollOver:67' else */
      /* 'func_anti_rollOver:68' ROLLOVER_FLAG(i) = ROLLOVER_FLAG(i) - 1; */
      controller_DW.ROLLOVER_FLAG[1]--;
    }
  }

  /* 'func_anti_rollOver:64' if (angle_difference(i) > MAX_ANGLE_DIFFERENCE)||(angle_difference(i) < -MAX_ANGLE_DIFFERENCE) */
  if ((angle_difference_idx_2 > controller_DW.MAX_ANGLE_DIFFERENCE) ||
      (angle_difference_idx_2 < -controller_DW.MAX_ANGLE_DIFFERENCE)) {
    /* 'func_anti_rollOver:65' if(angle_difference(i)<0) */
    if (angle_difference_idx_2 < 0.0) {
      /* 'func_anti_rollOver:66' ROLLOVER_FLAG(i) = ROLLOVER_FLAG(i) + 1; */
      controller_DW.ROLLOVER_FLAG[2]++;
    } else {
      /* 'func_anti_rollOver:67' else */
      /* 'func_anti_rollOver:68' ROLLOVER_FLAG(i) = ROLLOVER_FLAG(i) - 1; */
      controller_DW.ROLLOVER_FLAG[2]--;
    }
  }

  /* 'func_anti_rollOver:64' if (angle_difference(i) > MAX_ANGLE_DIFFERENCE)||(angle_difference(i) < -MAX_ANGLE_DIFFERENCE) */
  if ((angle_difference_idx_3 > controller_DW.MAX_ANGLE_DIFFERENCE) ||
      (angle_difference_idx_3 < -controller_DW.MAX_ANGLE_DIFFERENCE)) {
    /* 'func_anti_rollOver:65' if(angle_difference(i)<0) */
    if (angle_difference_idx_3 < 0.0) {
      /* 'func_anti_rollOver:66' ROLLOVER_FLAG(i) = ROLLOVER_FLAG(i) + 1; */
      controller_DW.ROLLOVER_FLAG[3]++;
    } else {
      /* 'func_anti_rollOver:67' else */
      /* 'func_anti_rollOver:68' ROLLOVER_FLAG(i) = ROLLOVER_FLAG(i) - 1; */
      controller_DW.ROLLOVER_FLAG[3]--;
    }
  }

  /* 'func_anti_rollOver:73' angle_aro = angle + ANTI_ROLLOVER_CORRECTION_MATRIX*ROLLOVER_FLAG; */
  /*  convert angles to radian */
  /* 'func_anti_rollOver:77' angle_aro = DEG2RAD*angle_aro; */
  /*  calibration materials */
  /* 'func_anti_rollOver:80' max_angle = DEG2RAD*[MAX_RP_ANGLE_RIGHT,MAX_RP_ANGLE_LEFT,MAX_DV_ANGLE_RIGHT,MAX_DV_ANGLE_LEFT].'; */
  /* 'func_anti_rollOver:81' min_angle = DEG2RAD*[MIN_RP_ANGLE_RIGHT,MIN_RP_ANGLE_LEFT,MIN_DV_ANGLE_RIGHT,MIN_DV_ANGLE_LEFT].'; */
  /* 'func_anti_rollOver:82' delta_angle_matrix = diag(max_angle-min_angle); */
  /*  calibrate angles */
  /* 'func_anti_rollOver:85' angle_aro = angle_aro-min_angle; */
  for (argout = 0; argout < 4; argout++) {
    tmp[argout] = ((((real_T)b[argout + 4] *
                     controller_DW.ANTI_ROLLOVER_CORRECTION *
                     controller_DW.ROLLOVER_FLAG[1] +
                     controller_DW.ANTI_ROLLOVER_CORRECTION * (real_T)b[argout] *
                     controller_DW.ROLLOVER_FLAG[0]) + (real_T)b[argout + 8] *
                    controller_DW.ANTI_ROLLOVER_CORRECTION *
                    controller_DW.ROLLOVER_FLAG[2]) + (real_T)b[argout + 12] *
                   controller_DW.ANTI_ROLLOVER_CORRECTION *
                   controller_DW.ROLLOVER_FLAG[3]) + controller_U.angle[argout];
  }

  angle_aro_idx_0 = 0.017453292519943295 * tmp[0] - 0.017453292519943295 *
    controller_DW.MIN_RP_ANGLE_RIGHT;
  angle_aro_idx_1 = 0.017453292519943295 * tmp[1] - 0.017453292519943295 *
    controller_DW.MIN_RP_ANGLE_LEFT;
  angle_aro_idx_2 = 0.017453292519943295 * tmp[2] - 0.017453292519943295 *
    controller_DW.MIN_DV_ANGLE_RIGHT;
  angle_aro_idx_3 = 0.017453292519943295 * tmp[3] - 0.017453292519943295 *
    controller_DW.MIN_DV_ANGLE_LEFT;

  /*  filtering */
  /* '<S2>:1:99' angle_f = func_lowpass_filter(angle_aro,angle_aro_prev); */
  /*  func_lowpass_filter: this is a simple moving average filter(n: window size) */
  /*  angle: 4-by-1 vec */
  /*    angle(1): right forelimb */
  /*    angle(2): left forelimb */
  /*    angle(3): right leg */
  /*    angle(4): left leg */
  /*  angle_prev: from n-previous sample times, a 4-by-n vector */
  /*    angle_prev(1,i): right forelimb @ time(i) */
  /*    angle_prev(2,i): left forelimb  @ time(i) */
  /*    angle_prev(3,i): right leg @ time(i) */
  /*    angle_prev(4,i): left leg @ time(i) */
  /*  angle_f: filtered data, 4-by-1 vec */
  /*    angle_f(1): right forelimb */
  /*    angle_f(2): left forelimb */
  /*    angle_f(3): right leg */
  /*    angle_f(4): left leg */
  /*  by ALireza Ramezani, 8-31-2015, Champaign, IL */
  /* 'func_lowpass_filter:20' angle_f = zeros(4,1); */
  /* 'func_lowpass_filter:21' angle_sum = zeros(4,1); */
  /*  find the length of moving ave window */
  /* 'func_lowpass_filter:24' [~,n] = size(angle_prev); */
  /* 'func_lowpass_filter:24' ~ */
  /* 'func_lowpass_filter:26' for i=1:n */
  /* 'func_lowpass_filter:27' angle_sum = angle_sum + angle_prev(:,i); */
  /* 'func_lowpass_filter:30' angle_sum = angle_sum + angle; */
  /* 'func_lowpass_filter:32' angle_f = angle_sum/(n+1); */
  angle_difference_idx_0 = (controller_DW.UnitDelay1_DSTATE[0] + angle_aro_idx_0)
    / 2.0;
  angle_difference_idx_1 = (controller_DW.UnitDelay1_DSTATE[1] + angle_aro_idx_1)
    / 2.0;
  angle_difference_idx_2 = (controller_DW.UnitDelay1_DSTATE[2] + angle_aro_idx_2)
    / 2.0;
  angle_difference_idx_3 = (controller_DW.UnitDelay1_DSTATE[3] + angle_aro_idx_3)
    / 2.0;

  /*  end of code */
  /*  PID scheme */
  /* '<S2>:1:102' [u_pid,err] = func_pid_controller(angle_f,uf,prev_err,pid_gain); */
  /*  func_pid_controller: pd position controller */
  /*  angle: 4-by-1 vec */
  /*    angle(1): right forelimb */
  /*    angle(2): left forelimb */
  /*    angle(3): right leg */
  /*    angle(4): left leg */
  /*  des_angle: desired position 4-by-1 vec */
  /*    des_angle(1): right forelimb */
  /*    des_angle(2): left forelimb */
  /*    des_angle(3): right leg */
  /*    des_angle(4): left leg */
  /*  prev_err: memory for err vector 4-by-1 vec */
  /*    prev_err(1): right forelimb */
  /*    prev_err(2): left forelimb */
  /*    prev_err(3): right leg */
  /*    prev_err(4): left leg */
  /*  gain: control gain vector 2-by-2 vec */
  /*    gain(1,1:2): right forelimb Kp and Kd */
  /*    gain(1,1:2): left forelimb  Kp and Kd */
  /*    gain(2,1:2): right leg Kp and Kd */
  /*    gain(2,1:2): left leg Kp and Kd */
  /*  u: control action, 4-by-1 vec */
  /*    u(1): right forelimb */
  /*    u(2): left forelimb */
  /*    u(3): right leg */
  /*    u(4): left leg */
  /*  err: to save err vec in memory */
  /*  by ALireza Ramezani, 8-31-2015, Champaign, IL */
  /* 'func_pid_controller:31' u = zeros(4,1); */
  /* 'func_pid_controller:32' err = zeros(4,1); */
  /*  err */
  /* 'func_pid_controller:33' derr = zeros(4,1); */
  /*  derivative of err */
  /* 'func_pid_controller:34' serr = zeros(4,1); */
  /*  integration of err */
  /*  global vars */
  /* 'func_pid_controller:46' Kp = [gain(1,1),0,0,0;... */
  /* 'func_pid_controller:47'       0, gain(1,1),0,0;... */
  /* 'func_pid_controller:48'       0, 0,gain(2,1),0;... */
  /* 'func_pid_controller:49'       0, 0, 0,gain(2,1)]; */
  /* 'func_pid_controller:51' Kd = [gain(1,2),0,0,0;... */
  /* 'func_pid_controller:52'       0, gain(1,2),0,0;... */
  /* 'func_pid_controller:53'       0, 0,gain(2,2),0;... */
  /* 'func_pid_controller:54'       0, 0, 0,gain(2,2)]; */
  /*  compute error and derr */
  /* 'func_pid_controller:57' err = angle - des_angle; */
  rtb_uf[0] = angle_difference_idx_0 - rtb_uf[0];
  rtb_uf[1] = angle_difference_idx_1 - rtb_uf[1];
  rtb_uf[2] = angle_difference_idx_2 - rtb_uf[2];
  rtb_uf_a = angle_difference_idx_3 - rtb_uf[3];

  /* 'func_pid_controller:58' derr = err-prev_err; */
  /*  computer u */
  /* 'func_pid_controller:61' u = -Kp*err -Kd*derr; */
  tmp_0[0] = controller_U.pid_gian[0];
  tmp_0[4] = 0.0;
  tmp_0[8] = 0.0;
  tmp_0[12] = 0.0;
  tmp_0[1] = 0.0;
  tmp_0[5] = controller_U.pid_gian[0];
  tmp_0[9] = 0.0;
  tmp_0[13] = 0.0;
  tmp_0[2] = 0.0;
  tmp_0[6] = 0.0;
  tmp_0[10] = controller_U.pid_gian[1];
  tmp_0[14] = 0.0;
  tmp_0[3] = 0.0;
  tmp_0[7] = 0.0;
  tmp_0[11] = 0.0;
  tmp_0[15] = controller_U.pid_gian[1];
  for (argout = 0; argout < 4; argout++) {
    sensitivity_matrix[argout << 2] = -tmp_0[argout << 2];
    sensitivity_matrix[1 + (argout << 2)] = -tmp_0[(argout << 2) + 1];
    sensitivity_matrix[2 + (argout << 2)] = -tmp_0[(argout << 2) + 2];
    sensitivity_matrix[3 + (argout << 2)] = -tmp_0[(argout << 2) + 3];
  }

  tmp_1[0] = controller_U.pid_gian[2];
  tmp_1[4] = 0.0;
  tmp_1[8] = 0.0;
  tmp_1[12] = 0.0;
  tmp_1[1] = 0.0;
  tmp_1[5] = controller_U.pid_gian[2];
  tmp_1[9] = 0.0;
  tmp_1[13] = 0.0;
  tmp_1[2] = 0.0;
  tmp_1[6] = 0.0;
  tmp_1[10] = controller_U.pid_gian[3];
  tmp_1[14] = 0.0;
  tmp_1[3] = 0.0;
  tmp_1[7] = 0.0;
  tmp_1[11] = 0.0;
  tmp_1[15] = controller_U.pid_gian[3];
  max_angle_idx_3 = rtb_uf[0] - controller_DW.UnitDelay2_DSTATE[0];
  rtb_uf_idx_1 = rtb_uf[1] - controller_DW.UnitDelay2_DSTATE[1];
  max_angle_idx_2 = rtb_uf[2] - controller_DW.UnitDelay2_DSTATE[2];
  max_angle_idx_1 = rtb_uf_a - controller_DW.UnitDelay2_DSTATE[3];
  for (argout = 0; argout < 4; argout++) {
    max_angle_idx_0 = sensitivity_matrix[argout + 12] * rtb_uf_a +
      (sensitivity_matrix[argout + 8] * rtb_uf[2] + (sensitivity_matrix[argout +
        4] * rtb_uf[1] + sensitivity_matrix[argout] * rtb_uf[0]));
    tmp[argout] = max_angle_idx_0;
  }

  for (argout = 0; argout < 4; argout++) {
    max_angle_idx_0 = tmp_1[argout + 12] * max_angle_idx_1 + (tmp_1[argout + 8] *
      max_angle_idx_2 + (tmp_1[argout + 4] * rtb_uf_idx_1 + tmp_1[argout] *
                         max_angle_idx_3));
    tmp_2[argout] = max_angle_idx_0;
  }

  u_pid_idx_0 = tmp[0] - tmp_2[0];
  u_pid_idx_1 = tmp[1] - tmp_2[1];
  u_pid_idx_2 = tmp[2] - tmp_2[2];
  u_pid_idx_3 = tmp[3] - tmp_2[3];

  /*  turn-off the controller immediately when hit the limits */
  /* 'func_pid_controller:64' DEG2RAD = pi/180; */
  /*  rad\deg */
  /* 'func_pid_controller:65' max_angle = DEG2RAD*[MAX_RP_ANGLE_RIGHT,MAX_RP_ANGLE_LEFT,MAX_DV_ANGLE_RIGHT,MAX_DV_ANGLE_LEFT].'; */
  /* 'func_pid_controller:66' min_angle = DEG2RAD*[MIN_RP_ANGLE_RIGHT,MIN_RP_ANGLE_LEFT,MIN_DV_ANGLE_RIGHT,MIN_DV_ANGLE_LEFT].'; */
  /* 'func_pid_controller:67' delta_max_angle = max_angle-min_angle; */
  /* 'func_pid_controller:68' for i=1:4 */
  /* 'func_pid_controller:69' if((angle(i)>delta_max_angle(i))||(angle(i)<0)) */
  if ((angle_difference_idx_0 > 0.017453292519943295 *
       controller_DW.MAX_RP_ANGLE_RIGHT - 0.017453292519943295 *
       controller_DW.MIN_RP_ANGLE_RIGHT) || (angle_difference_idx_0 < 0.0)) {
    /* 'func_pid_controller:70' u(i) = 0; */
    u_pid_idx_0 = 0.0;
  }

  /* 'func_pid_controller:69' if((angle(i)>delta_max_angle(i))||(angle(i)<0)) */
  if ((angle_difference_idx_1 > 0.017453292519943295 *
       controller_DW.MAX_RP_ANGLE_LEFT - 0.017453292519943295 *
       controller_DW.MIN_RP_ANGLE_LEFT) || (angle_difference_idx_1 < 0.0)) {
    /* 'func_pid_controller:70' u(i) = 0; */
    u_pid_idx_1 = 0.0;
  }

  /* 'func_pid_controller:69' if((angle(i)>delta_max_angle(i))||(angle(i)<0)) */
  if ((angle_difference_idx_2 > 0.017453292519943295 *
       controller_DW.MAX_DV_ANGLE_RIGHT - 0.017453292519943295 *
       controller_DW.MIN_DV_ANGLE_RIGHT) || (angle_difference_idx_2 < 0.0)) {
    /* 'func_pid_controller:70' u(i) = 0; */
    u_pid_idx_2 = 0.0;
  }

  /* 'func_pid_controller:69' if((angle(i)>delta_max_angle(i))||(angle(i)<0)) */
  if ((angle_difference_idx_3 > 0.017453292519943295 *
       controller_DW.MAX_DV_ANGLE_LEFT - 0.017453292519943295 *
       controller_DW.MIN_DV_ANGLE_LEFT) || (angle_difference_idx_3 < 0.0)) {
    /* 'func_pid_controller:70' u(i) = 0; */
    u_pid_idx_3 = 0.0;
  }

  /*  map pid efforts to servo */
  /* '<S2>:1:105' us = func_map_pid_to_servo(u_pid); */
  /*  func_map_pid_to_servo: map controller input to drv8835 commands */
  /*  u: control action, 4-by-1 vec */
  /*    u(1): right forelimb */
  /*    u(2): left forelimb */
  /*    u(3): right leg */
  /*    u(4): left leg */
  /*  u_drv: drv8835 control command, 8-by-1 vec */
  /*    u_drv(1): right forelimb */
  /*    u_drv(2): left forelimb */
  /*    u_drv(3): right leg */
  /*    u_drv(4): left leg */
  /*  by ALireza Ramezani, 8-31-2015, Champaign, IL */
  /* 'func_map_pid_to_servo:15' u_drv = zeros(8,1); */
  /* 'func_map_pid_to_servo:16' H = zeros(8,4); */
  /*  used for saturating control command to drv8835 */
  /*  map */
  /* 'func_map_pid_to_servo:22' H = [func_heaviside(u(1)),0,0,0;... */
  /* 'func_map_pid_to_servo:23'      func_heaviside(-u(1)),0,0,0;... */
  /* 'func_map_pid_to_servo:24'      0,func_heaviside(u(2)),0,0;... */
  /* 'func_map_pid_to_servo:25'      0,func_heaviside(-u(2)),0,0;... */
  /* 'func_map_pid_to_servo:26'      0,0,func_heaviside(u(3)),0;... */
  /* 'func_map_pid_to_servo:27'      0,0,func_heaviside(-u(3)),0;... */
  /* 'func_map_pid_to_servo:28'      0,0,0,func_heaviside(u(4));... */
  /* 'func_map_pid_to_servo:29'      0,0,0,func_heaviside(-u(4))]; */
  /*  func_heaviside: heaviside function */
  /*  by Alireza Ramezani,9-1-2015, Champaign, IL */
  /* 'func_heaviside:5' argout = 0; */
  argout = 0;

  /* 'func_heaviside:7' if argin>=0 */
  if (u_pid_idx_0 >= 0.0) {
    /* 'func_heaviside:8' argout = 1; */
    argout = 1;
  }

  /*  end of code */
  /*  func_heaviside: heaviside function */
  /*  by Alireza Ramezani,9-1-2015, Champaign, IL */
  /* 'func_heaviside:5' argout = 0; */
  b_argout = 0;

  /* 'func_heaviside:7' if argin>=0 */
  if (-u_pid_idx_0 >= 0.0) {
    /* 'func_heaviside:8' argout = 1; */
    b_argout = 1;
  }

  /*  end of code */
  /*  func_heaviside: heaviside function */
  /*  by Alireza Ramezani,9-1-2015, Champaign, IL */
  /* 'func_heaviside:5' argout = 0; */
  c_argout = 0;

  /* 'func_heaviside:7' if argin>=0 */
  if (u_pid_idx_1 >= 0.0) {
    /* 'func_heaviside:8' argout = 1; */
    c_argout = 1;
  }

  /*  end of code */
  /*  func_heaviside: heaviside function */
  /*  by Alireza Ramezani,9-1-2015, Champaign, IL */
  /* 'func_heaviside:5' argout = 0; */
  d_argout = 0;

  /* 'func_heaviside:7' if argin>=0 */
  if (-u_pid_idx_1 >= 0.0) {
    /* 'func_heaviside:8' argout = 1; */
    d_argout = 1;
  }

  /*  end of code */
  /*  func_heaviside: heaviside function */
  /*  by Alireza Ramezani,9-1-2015, Champaign, IL */
  /* 'func_heaviside:5' argout = 0; */
  e_argout = 0;

  /* 'func_heaviside:7' if argin>=0 */
  if (u_pid_idx_2 >= 0.0) {
    /* 'func_heaviside:8' argout = 1; */
    e_argout = 1;
  }

  /*  end of code */
  /*  func_heaviside: heaviside function */
  /*  by Alireza Ramezani,9-1-2015, Champaign, IL */
  /* 'func_heaviside:5' argout = 0; */
  f_argout = 0;

  /* 'func_heaviside:7' if argin>=0 */
  if (-u_pid_idx_2 >= 0.0) {
    /* 'func_heaviside:8' argout = 1; */
    f_argout = 1;
  }

  /*  end of code */
  /*  func_heaviside: heaviside function */
  /*  by Alireza Ramezani,9-1-2015, Champaign, IL */
  /* 'func_heaviside:5' argout = 0; */
  g_argout = 0;

  /* 'func_heaviside:7' if argin>=0 */
  if (u_pid_idx_3 >= 0.0) {
    /* 'func_heaviside:8' argout = 1; */
    g_argout = 1;
  }

  /*  end of code */
  /*  func_heaviside: heaviside function */
  /*  by Alireza Ramezani,9-1-2015, Champaign, IL */
  /* 'func_heaviside:5' argout = 0; */
  h_argout = 0;

  /* 'func_heaviside:7' if argin>=0 */
  if (-u_pid_idx_3 >= 0.0) {
    /* 'func_heaviside:8' argout = 1; */
    h_argout = 1;
  }

  /*  end of code */
  /*  saturate u */
  /* 'func_map_pid_to_servo:32' for i=1:4 */
  rtb_uf_idx_1 = u_pid_idx_0;

  /* 'func_map_pid_to_servo:33' if u(i)> PID_SATURATION_THRESHOLD */
  if (u_pid_idx_0 > controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_servo:34' u(i) = PID_SATURATION_THRESHOLD; */
    rtb_uf_idx_1 = controller_DW.PID_SATURATION_THRESHOLD;
  }

  /* 'func_map_pid_to_servo:37' if u(i)< -PID_SATURATION_THRESHOLD */
  if (rtb_uf_idx_1 < -controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_servo:38' u(i) = -PID_SATURATION_THRESHOLD; */
    rtb_uf_idx_1 = -controller_DW.PID_SATURATION_THRESHOLD;
  }

  angle_difference_idx_0 = rtb_uf_idx_1;
  rtb_uf_idx_1 = u_pid_idx_1;

  /* 'func_map_pid_to_servo:33' if u(i)> PID_SATURATION_THRESHOLD */
  if (u_pid_idx_1 > controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_servo:34' u(i) = PID_SATURATION_THRESHOLD; */
    rtb_uf_idx_1 = controller_DW.PID_SATURATION_THRESHOLD;
  }

  /* 'func_map_pid_to_servo:37' if u(i)< -PID_SATURATION_THRESHOLD */
  if (rtb_uf_idx_1 < -controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_servo:38' u(i) = -PID_SATURATION_THRESHOLD; */
    rtb_uf_idx_1 = -controller_DW.PID_SATURATION_THRESHOLD;
  }

  angle_difference_idx_1 = rtb_uf_idx_1;
  rtb_uf_idx_1 = u_pid_idx_2;

  /* 'func_map_pid_to_servo:33' if u(i)> PID_SATURATION_THRESHOLD */
  if (u_pid_idx_2 > controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_servo:34' u(i) = PID_SATURATION_THRESHOLD; */
    rtb_uf_idx_1 = controller_DW.PID_SATURATION_THRESHOLD;
  }

  /* 'func_map_pid_to_servo:37' if u(i)< -PID_SATURATION_THRESHOLD */
  if (rtb_uf_idx_1 < -controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_servo:38' u(i) = -PID_SATURATION_THRESHOLD; */
    rtb_uf_idx_1 = -controller_DW.PID_SATURATION_THRESHOLD;
  }

  angle_difference_idx_2 = rtb_uf_idx_1;
  rtb_uf_idx_1 = u_pid_idx_3;

  /* 'func_map_pid_to_servo:33' if u(i)> PID_SATURATION_THRESHOLD */
  if (u_pid_idx_3 > controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_servo:34' u(i) = PID_SATURATION_THRESHOLD; */
    rtb_uf_idx_1 = controller_DW.PID_SATURATION_THRESHOLD;
  }

  /* 'func_map_pid_to_servo:37' if u(i)< -PID_SATURATION_THRESHOLD */
  if (rtb_uf_idx_1 < -controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_servo:38' u(i) = -PID_SATURATION_THRESHOLD; */
    rtb_uf_idx_1 = -controller_DW.PID_SATURATION_THRESHOLD;
  }

  /* 'func_map_pid_to_servo:42' u_drv = H*abs(u); */
  max_angle_idx_0 = fabs(angle_difference_idx_0);
  max_angle_idx_1 = fabs(angle_difference_idx_1);
  max_angle_idx_2 = fabs(angle_difference_idx_2);
  max_angle_idx_3 = fabs(rtb_uf_idx_1);

  /*  end of code */
  argout_0[0] = (int8_T)argout;
  argout_0[8] = 0;
  argout_0[16] = 0;
  argout_0[24] = 0;
  argout_0[1] = (int8_T)b_argout;
  argout_0[9] = 0;
  argout_0[17] = 0;
  argout_0[25] = 0;
  argout_0[2] = 0;
  argout_0[10] = (int8_T)c_argout;
  argout_0[18] = 0;
  argout_0[26] = 0;
  argout_0[3] = 0;
  argout_0[11] = (int8_T)d_argout;
  argout_0[19] = 0;
  argout_0[27] = 0;
  argout_0[4] = 0;
  argout_0[12] = 0;
  argout_0[20] = (int8_T)e_argout;
  argout_0[28] = 0;
  argout_0[5] = 0;
  argout_0[13] = 0;
  argout_0[21] = (int8_T)f_argout;
  argout_0[29] = 0;
  argout_0[6] = 0;
  argout_0[14] = 0;
  argout_0[22] = 0;
  argout_0[30] = (int8_T)g_argout;
  argout_0[7] = 0;
  argout_0[15] = 0;
  argout_0[23] = 0;
  argout_0[31] = (int8_T)h_argout;
  for (argout = 0; argout < 8; argout++) {
    rtb_uf_idx_1 = (real_T)argout_0[argout + 24] * max_angle_idx_3 + ((real_T)
      argout_0[argout + 16] * max_angle_idx_2 + ((real_T)argout_0[argout + 8] *
      max_angle_idx_1 + (real_T)argout_0[argout] * max_angle_idx_0));
    rtb_us[argout] = rtb_uf_idx_1;
  }

  /* Update for UnitDelay: '<Root>/Unit Delay1' incorporates:
   *  MATLAB Function: '<Root>/func_actuator_controller'
   */
  /*  some update for memory */
  /* '<S2>:1:108' angle2mem = angle; */
  /* '<S2>:1:109' angle_aro2mem = angle_aro; */
  controller_DW.UnitDelay1_DSTATE[0] = angle_aro_idx_0;
  controller_DW.UnitDelay1_DSTATE[1] = angle_aro_idx_1;
  controller_DW.UnitDelay1_DSTATE[2] = angle_aro_idx_2;
  controller_DW.UnitDelay1_DSTATE[3] = angle_aro_idx_3;

  /* Update for UnitDelay: '<Root>/Unit Delay2' incorporates:
   *  MATLAB Function: '<Root>/func_actuator_controller'
   */
  /* '<S2>:1:110' err2mem = err; */
  controller_DW.UnitDelay2_DSTATE[0] = rtb_uf[0];
  controller_DW.UnitDelay2_DSTATE[1] = rtb_uf[1];
  controller_DW.UnitDelay2_DSTATE[2] = rtb_uf[2];
  controller_DW.UnitDelay2_DSTATE[3] = rtb_uf_a;

  /* Outport: '<Root>/debug' incorporates:
   *  MATLAB Function: '<Root>/func_actuator_controller'
   */
  /* '<S2>:1:111' debug = u_pid; */
  controller_Y.debug[0] = u_pid_idx_0;
  controller_Y.debug[1] = u_pid_idx_1;
  controller_Y.debug[2] = u_pid_idx_2;
  controller_Y.debug[3] = u_pid_idx_3;

  /* Outport: '<Root>/M0A' */
  /*  end of code */
  controller_Y.M0A = rtb_us[0];

  /* Outport: '<Root>/M0B' */
  controller_Y.M0B = rtb_us[1];

  /* Outport: '<Root>/M1A' */
  controller_Y.M1A = rtb_us[2];

  /* Outport: '<Root>/M1B' */
  controller_Y.M1B = rtb_us[3];

  /* Outport: '<Root>/M2A' */
  controller_Y.M2A = rtb_us[4];

  /* Outport: '<Root>/M2B' */
  controller_Y.M2B = rtb_us[5];

  /* Outport: '<Root>/M3A' */
  controller_Y.M3A = rtb_us[6];

  /* Outport: '<Root>/M3B' */
  controller_Y.M3B = rtb_us[7];

  /* DigitalClock: '<Root>/Digital Clock' */
  rtb_DigitalClock = ((controller_M->Timing.clockTick0) * 0.05);

  /* MATLAB Function: '<Root>/fcn_timer' */
  /* MATLAB Function 'fcn_timer': '<S1>:1' */
  /*  fcn_timer */
  /*  tin: time */
  /*  tou: time */
  /*  by Alireza Ramezani, 9-5-2015, Champaign, IL */
  /* '<S1>:1:9' if(tin < 1) */
  if (rtb_DigitalClock < 1.0) {
    /* '<S1>:1:10' ROLLOVER_FLAG = [0,0,0,0].'; */
    controller_DW.ROLLOVER_FLAG[0] = 0.0;
    controller_DW.ROLLOVER_FLAG[1] = 0.0;
    controller_DW.ROLLOVER_FLAG[2] = 0.0;
    controller_DW.ROLLOVER_FLAG[3] = 0.0;
  }

  /* Outport: '<Root>/time' incorporates:
   *  MATLAB Function: '<Root>/fcn_timer'
   */
  /* '<S1>:1:13' tout = tin; */
  controller_Y.time = rtb_DigitalClock;

  /* Update for UnitDelay: '<Root>/Unit Delay' incorporates:
   *  Inport: '<Root>/angle'
   *  MATLAB Function: '<Root>/func_actuator_controller'
   */
  controller_DW.UnitDelay_DSTATE[0] = controller_U.angle[0];
  controller_DW.UnitDelay_DSTATE[1] = controller_U.angle[1];
  controller_DW.UnitDelay_DSTATE[2] = controller_U.angle[2];
  controller_DW.UnitDelay_DSTATE[3] = controller_U.angle[3];

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.05, which is the step size
   * of the task. Size of "clockTick0" ensures timer will not overflow during the
   * application lifespan selected.
   */
  controller_M->Timing.clockTick0++;
}

/* Model initialize function */
void controller_initialize(void)
{
  /* Start for InitialCondition: '<Root>/IC' */
  controller_DW.IC_FirstOutputTime = true;

  /* Start for InitialCondition: '<Root>/IC2' */
  memcpy(&controller_DW.IC2[0], &controller_ConstP.IC2_Value[0], 11U * sizeof
         (real_T));
  controller_DW.IC2_FirstOutputTime = true;
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF] controller.c
 */
