/*
 * File: controller.c
 *
 * Code generated for Simulink model :controller.
 *
 * Model version      : 1.200
 * Simulink Coder version    : 8.6 (R2014a) 27-Dec-2013
 * TLC version       : 8.6 (Jan 30 2014)
 * C/C++ source code generated on  : Thu Mar 03 17:19:19 2016
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
  static const int8_T a[12] = { 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 };

  real_T u_pid[4];
  int32_T argout;
  int32_T b_argout;
  int32_T c_argout;
  int32_T d_argout;
  int32_T e_argout;
  int32_T f_argout;
  int32_T g_argout;
  int32_T h_argout;
  real_T rtb_us[8];
  real_T rtb_q[12];
  real_T tmp[8];
  real_T a_0[2];
  real_T tmp_0[8];
  real_T a_1[2];
  real_T tmp_1[4];
  real_T tmp_2[4];
  real_T tmp_3[16];
  real_T tmp_4[16];
  real_T tmp_5[16];
  real_T tmp_6[16];
  int8_T argout_0[32];
  real_T rtb_TSamp_idx_0;
  real_T rtb_TSamp_idx_1;
  real_T rtb_TSamp_idx_2;
  real_T delta_max_angle_idx_0;
  real_T delta_max_angle_idx_1;
  real_T delta_max_angle_idx_2;
  real_T delta_max_angle_idx_3;
  real_T angle_aro_idx_0;
  real_T angle_aro_idx_1;
  real_T angle_aro_idx_2;
  real_T angle_aro_idx_3;
  real_T rtb_uf_idx_0;
  real_T rtb_uf_idx_1;
  real_T rtb_uf_idx_2;
  real_T rtb_uf_idx_3;
  real_T err_idx_0;
  real_T err_idx_1;
  real_T err_idx_2;
  real_T err_idx_3;
  real_T err_idx_0_0;

  /* SampleTimeMath: '<S5>/TSamp' incorporates:
   *  Inport: '<Root>/pitch'
   *  Inport: '<Root>/roll'
   *  Inport: '<Root>/yaw'
   *
   * About '<S5>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  rtb_TSamp_idx_0 = controller_U.roll * 500.0;
  rtb_TSamp_idx_1 = controller_U.pitch * 500.0;
  rtb_TSamp_idx_2 = controller_U.yaw * 500.0;

  /* MATLAB Function: '<S4>/func_state_estimator' incorporates:
   *  Inport: '<Root>/pitch'
   *  Inport: '<Root>/roll'
   *  Inport: '<Root>/yaw'
   *  SignalConversion: '<S6>/TmpSignal ConversionAt SFunction Inport1'
   *  Sum: '<S5>/Diff'
   *  UnitDelay: '<S5>/UD'
   */
  /* MATLAB Function 'state estimator/func_state_estimator': '<S6>:1' */
  /*  state estimator */
  /*  u: imu measurments  */
  /*     roll [deg] */
  /*     pitch */
  /*     yaw */
  /*  du: imu measurments, rate angles */
  /*     roll rate [rad/sec] */
  /*     pitch rate */
  /*     yaw rate */
  /*  accel: imu measurments, accelerations */
  /*     accel X [m/s^2] */
  /*     accel Y */
  /*     accel Z */
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
  /*        x-axis is towards right arm wing (pitch) */
  /*        y-axis is towards tail (roll) */
  /*        z-axis is downwards (yaw) */
  /*  By Alireza Ramezani, 9-5-2015, Champaign, IL */
  /* '<S6>:1:34' q = zeros(12,1); */
  /* '<S6>:1:35' DEG2RAD = pi/180; */
  /*  rad\deg */
  /*  roll */
  /* '<S6>:1:38' q(1) = DEG2RAD*u(2); */
  rtb_q[0] = 0.017453292519943295 * controller_U.pitch;

  /*  pitch */
  /* '<S6>:1:41' q(2) = DEG2RAD*u(1); */
  rtb_q[1] = 0.017453292519943295 * controller_U.roll;

  /*  yaw  */
  /* '<S6>:1:44' q(3) = DEG2RAD*u(3); */
  rtb_q[2] = 0.017453292519943295 * controller_U.yaw;

  /*  px */
  /* '<S6>:1:47' q(4) = 0; */
  rtb_q[3] = 0.0;

  /*  py */
  /* '<S6>:1:50' q(5) = 0; */
  rtb_q[4] = 0.0;

  /*  pz */
  /* '<S6>:1:53' q(6) = 0; */
  rtb_q[5] = 0.0;

  /*  roll rate */
  /* '<S6>:1:56' q(7) = DEG2RAD*du(2); */
  rtb_q[6] = (rtb_TSamp_idx_1 - controller_DW.UD_DSTATE[1]) *
    0.017453292519943295;

  /*  pitch rate */
  /* '<S6>:1:59' q(8) = DEG2RAD*du(1); */
  rtb_q[7] = (rtb_TSamp_idx_0 - controller_DW.UD_DSTATE[0]) *
    0.017453292519943295;

  /*  yaw rate  */
  /* '<S6>:1:62' q(9) = DEG2RAD*du(3); */
  rtb_q[8] = (rtb_TSamp_idx_2 - controller_DW.UD_DSTATE[2]) *
    0.017453292519943295;

  /*  vx */
  /* '<S6>:1:65' q(10) = 0; */
  rtb_q[9] = 0.0;

  /*  vy */
  /* '<S6>:1:68' q(11) = 0; */
  rtb_q[10] = 0.0;

  /*  vz */
  /* '<S6>:1:71' q(12) = 0; */
  rtb_q[11] = 0.0;

  /* Outport: '<Root>/q' */
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
   *  Inport: '<Root>/xd'
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
  /*  xd: desired values [rad] */
  /*    qr: roll */
  /*    qp: pitch */
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
  /* '<S3>:1:39' uf = zeros(4,1); */
  /* '<S3>:1:41' DEG2RAD = pi/180; */
  /*  rad\deg */
  /* '<S3>:1:43' ROLL_wing_kp = flight_ctrl_params(1); */
  /* '<S3>:1:44' ROLL_leg_kp = flight_ctrl_params(2); */
  /* '<S3>:1:45' PITCH_leg_kp = flight_ctrl_params(3); */
  /* '<S3>:1:46' ROLL_wing_kd = flight_ctrl_params(4); */
  /* '<S3>:1:47' ROLL_leg_kd = flight_ctrl_params(5); */
  /* '<S3>:1:48' PITCH_leg_kd = flight_ctrl_params(6); */
  /* '<S3>:1:49' R_foreq = flight_ctrl_params(7); */
  /* '<S3>:1:50' L_foreq = flight_ctrl_params(8); */
  /* '<S3>:1:51' R_leq = flight_ctrl_params(9); */
  /* '<S3>:1:52' L_leq = flight_ctrl_params(10); */
  /*  feed-forward */
  /* '<S3>:1:55' fwd = DEG2RAD*[R_foreq,L_foreq,R_leq,L_leq].'; */
  /*  comptue control */
  /* '<S3>:1:59' x = q(1:6); */
  /* '<S3>:1:60' dx = q(7:end); */
  /* '<S3>:1:62' kp = [ROLL_wing_kp,0;... */
  /* '<S3>:1:63'      -ROLL_wing_kp,0;... */
  /* '<S3>:1:64'       ROLL_leg_kp,PITCH_leg_kp;... */
  /* '<S3>:1:65'       ROLL_leg_kp,-PITCH_leg_kp]; */
  /* '<S3>:1:67' kd = [ROLL_wing_kd,0;... */
  /* '<S3>:1:68'      -ROLL_wing_kd,0;... */
  /* '<S3>:1:69'       ROLL_leg_kd,PITCH_leg_kd;... */
  /* '<S3>:1:70'       ROLL_leg_kd,-PITCH_leg_kd]; */
  /* '<S3>:1:72' H = [1,0,0,0,0,0;... */
  /* '<S3>:1:73'      0,1,0,0,0,0]; */
  /* '<S3>:1:75' y = H*x-xd; */
  /* '<S3>:1:77' dy = H*dx; */
  /* '<S3>:1:79' uf = -kp*y -kd*dy + fwd; */
  rtb_us[0] = controller_DW.IC[0];
  rtb_us[4] = 0.0;
  rtb_us[1] = -controller_DW.IC[0];
  rtb_us[5] = 0.0;
  rtb_us[2] = controller_DW.IC[1];
  rtb_us[6] = controller_DW.IC[2];
  rtb_us[3] = controller_DW.IC[1];
  rtb_us[7] = -controller_DW.IC[2];
  for (argout = 0; argout < 2; argout++) {
    tmp[argout << 2] = -rtb_us[argout << 2];
    tmp[1 + (argout << 2)] = -rtb_us[(argout << 2) + 1];
    tmp[2 + (argout << 2)] = -rtb_us[(argout << 2) + 2];
    tmp[3 + (argout << 2)] = -rtb_us[(argout << 2) + 3];
  }

  for (argout = 0; argout < 2; argout++) {
    delta_max_angle_idx_3 = 0.0;
    for (b_argout = 0; b_argout < 6; b_argout++) {
      delta_max_angle_idx_3 += (real_T)a[(b_argout << 1) + argout] *
        rtb_q[b_argout];
    }

    a_0[argout] = delta_max_angle_idx_3 - controller_U.xd[argout];
  }

  tmp_0[0] = controller_DW.IC[3];
  tmp_0[4] = 0.0;
  tmp_0[1] = -controller_DW.IC[3];
  tmp_0[5] = 0.0;
  tmp_0[2] = controller_DW.IC[4];
  tmp_0[6] = controller_DW.IC[5];
  tmp_0[3] = controller_DW.IC[4];
  tmp_0[7] = -controller_DW.IC[5];
  for (argout = 0; argout < 2; argout++) {
    a_1[argout] = 0.0;
    for (b_argout = 0; b_argout < 6; b_argout++) {
      a_1[argout] += (real_T)a[(b_argout << 1) + argout] * rtb_q[6 + b_argout];
    }
  }

  for (argout = 0; argout < 4; argout++) {
    tmp_1[argout] = tmp[argout + 4] * a_0[1] + tmp[argout] * a_0[0];
  }

  for (argout = 0; argout < 4; argout++) {
    tmp_2[argout] = tmp_0[argout + 4] * a_1[1] + tmp_0[argout] * a_1[0];
  }

  rtb_uf_idx_0 = (tmp_1[0] - tmp_2[0]) + 0.017453292519943295 *
    controller_DW.IC[6];
  rtb_uf_idx_1 = (tmp_1[1] - tmp_2[1]) + 0.017453292519943295 *
    controller_DW.IC[7];
  rtb_uf_idx_2 = (tmp_1[2] - tmp_2[2]) + 0.017453292519943295 *
    controller_DW.IC[8];
  rtb_uf_idx_3 = (tmp_1[3] - tmp_2[3]) + 0.017453292519943295 *
    controller_DW.IC[9];

  /* End of MATLAB Function: '<Root>/func_flight_controller' */

  /* Outport: '<Root>/flight_ctrl' */
  controller_Y.flight_ctrl[0] = rtb_uf_idx_0;
  controller_Y.flight_ctrl[1] = rtb_uf_idx_1;
  controller_Y.flight_ctrl[2] = rtb_uf_idx_2;
  controller_Y.flight_ctrl[3] = rtb_uf_idx_3;

  /* InitialCondition: '<Root>/IC2' incorporates:
   *  Inport: '<Root>/actuator_ctrl_params'
   */
  if (controller_DW.IC2_FirstOutputTime) {
    controller_DW.IC2_FirstOutputTime = false;
    memcpy(&controller_DW.IC2[0], &controller_ConstP.IC2_Value[0], 14U * sizeof
           (real_T));
  } else {
    memcpy(&controller_DW.IC2[0], &controller_U.actuator_ctrl_params[0], 14U *
           sizeof(real_T));
  }

  /* End of InitialCondition: '<Root>/IC2' */

  /* MATLAB Function: '<Root>/func_actuator_controller' incorporates:
   *  Inport: '<Root>/angle'
   *  Inport: '<Root>/pid_gian'
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
  /*  gain: control gain vector 4-by-1 vec */
  /*    gain(1): right-left forelimb Kp */
  /*    gain(2): right-left leg Kp */
  /*    gain(3): right-left forelimb Kd */
  /*    gain(4): right-left leg Kd */
  /*  ctrl_param: params */
  /*    ctrl_param(1): PID_SATURATION_THRESHOLD (control sat, used in pid func) */
  /*    ctrl_param(2): MAX_ANGLE_DIFFERENCE (deg\sec, used in anti-roll-over func) */
  /*    ctrl_param(3): ANTI_ROLLOVER_CORRECTION (deg, used in anti-roll-over func) */
  /*    ctrl_param(4): MAX_RP_ANGLE_RIGHT (deg, same for left and right armwings) */
  /*    ctrl_param(5): MAX_DV_ANGLE_RIGHT (deg, same for left and right legs) */
  /*    ctrl_param(6): MIN_RP_ANGLE_RIGHT (deg, same for left and right armwings) */
  /*    ctrl_param(7): MIN_DV_ANGLE_RIGHT (deg, same for left and right legs) */
  /*    ctrl_param(8): MAX_RP_ANGLE_LEFT (deg, same for left and right armwings) */
  /*    ctrl_param(9): MAX_DV_ANGLE_LEFT (deg, same for left and right legs) */
  /*    ctrl_param(10): MIN_RP_ANGLE_LEFT (deg, same for left and right armwings) */
  /*    ctrl_param(11): MIN_DV_ANGLE_LEFT (deg, same for left and right legs) */
  /*    ctrl_param(12): SAMPLING_INTERVAL (sampling interval in sec, e.g., 0.1 for 100 m.sec) */
  /*    ctrl_param(13): PID_TRACKING_PRECISION_THRESHOLD (within this bound turn off the motors) */
  /*    ctrl_param(14): ANTI_WINDUP_THRESHOLD (anti-wind-up for the integrator in the PID controller) */
  /*  us: micro actuator control inputs, 4-by-1 vector (interprets control inputs for drv motor controllers) */
  /*    us(1): right forelimb */
  /*    us(2): left forelimb */
  /*    us(3): right leg */
  /*    us(4): left leg */
  /*  ubldc: micro actuator control inputs, 4-by-1 vector (interprets control inputs for tms320 motor controllers) */
  /*    ubldc(1): right forelimb */
  /*    ubldc(2): left forelimb */
  /*    ubldc(3): right leg */
  /*    ubldc(4): left leg */
  /*  angle2mem: to keep angle in memory for one sample time */
  /*  */
  /*  angle_aro2mem: to keep angle_aro in memory for one sample time */
  /*  */
  /*  err2mem: to keep error in memory for one sample time */
  /*  */
  /*  debug: debugging channel, i.e., used for debugging purpose. */
  /*  */
  /*  by ALireza Ramezani, 8-31-2015, Champaign, IL */
  /* '<S2>:1:72' us = zeros(8,1); */
  /* '<S2>:1:73' ubldc = zeros(4,1); */
  /* '<S2>:1:74' angle_aro = zeros(4,1); */
  /*  anti rolled-over */
  /* '<S2>:1:75' angle_f = zeros(4,1); */
  /*  filtered */
  /* '<S2>:1:76' u_pid = zeros(4,1); */
  /*  computer pid */
  /* '<S2>:1:77' angle2mem = zeros(4,1); */
  /* '<S2>:1:78' angle_aro2mem = zeros(4,1); */
  /* '<S2>:1:79' err2mem = zeros(4,1); */
  /* '<S2>:1:80' debug = zeros(4,1); */
  /*  global vars */
  /* '<S2>:1:100' PID_SATURATION_THRESHOLD = ctrl_param(1); */
  controller_DW.PID_SATURATION_THRESHOLD = controller_DW.IC2[0];

  /* '<S2>:1:101' MAX_ANGLE_DIFFERENCE = ctrl_param(2); */
  controller_DW.MAX_ANGLE_DIFFERENCE = controller_DW.IC2[1];

  /* '<S2>:1:102' ANTI_ROLLOVER_CORRECTION = ctrl_param(3); */
  controller_DW.ANTI_ROLLOVER_CORRECTION = controller_DW.IC2[2];

  /* '<S2>:1:103' MAX_RP_ANGLE_RIGHT = ctrl_param(4); */
  controller_DW.MAX_RP_ANGLE_RIGHT = controller_DW.IC2[3];

  /* '<S2>:1:104' MAX_DV_ANGLE_RIGHT = ctrl_param(5); */
  controller_DW.MAX_DV_ANGLE_RIGHT = controller_DW.IC2[4];

  /* '<S2>:1:105' MIN_RP_ANGLE_RIGHT = ctrl_param(6); */
  controller_DW.MIN_RP_ANGLE_RIGHT = controller_DW.IC2[5];

  /* '<S2>:1:106' MIN_DV_ANGLE_RIGHT = ctrl_param(7); */
  controller_DW.MIN_DV_ANGLE_RIGHT = controller_DW.IC2[6];

  /* '<S2>:1:107' MAX_RP_ANGLE_LEFT = ctrl_param(8); */
  controller_DW.MAX_RP_ANGLE_LEFT = controller_DW.IC2[7];

  /* '<S2>:1:108' MAX_DV_ANGLE_LEFT = ctrl_param(9); */
  controller_DW.MAX_DV_ANGLE_LEFT = controller_DW.IC2[8];

  /* '<S2>:1:109' MIN_RP_ANGLE_LEFT = ctrl_param(10); */
  controller_DW.MIN_RP_ANGLE_LEFT = controller_DW.IC2[9];

  /* '<S2>:1:110' MIN_DV_ANGLE_LEFT = ctrl_param(11); */
  controller_DW.MIN_DV_ANGLE_LEFT = controller_DW.IC2[10];

  /* '<S2>:1:111' SAMPLING_INTERVAL = ctrl_param(12); */
  controller_DW.SAMPLING_INTERVAL = controller_DW.IC2[11];

  /* '<S2>:1:112' PID_TRACKING_PRECISION_THRESHOLD = ctrl_param(13); */
  controller_DW.PID_TRACKING_PRECISION_THRESHOL = controller_DW.IC2[12];

  /* '<S2>:1:113' ANTI_WINDUP_THRESHOLD = ctrl_param(14); */
  controller_DW.ANTI_WINDUP_THRESHOLD = controller_DW.IC2[13];

  /*  angle preprocessing */
  /*  NOTE: output is rad. */
  /* '<S2>:1:117' angle_aro = func_anti_rollOver(angle,angle_prev); */
  /*  angle_difference = zeros(4,1); */
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
  /*  for i=1:4 */
  /*      if (angle_difference(i) > -ANTI_ROLLOVER_THRESHOLD)||(angle_difference(i) < ANTI_ROLLOVER_THRESHOLD) */
  /*          if(angle_difference(i)<0) */
  /*              angle_aro(i) = angle(i) + ANTI_ROLLOVER_CORRECTION; */
  /*          else */
  /*              angle_aro(i) = angle(i) - ANTI_ROLLOVER_CORRECTION; */
  /*          end */
  /*      end */
  /*  end */
  /*  for i=1:4 */
  /*      if (angle_difference(i) > MAX_ANGLE_DIFFERENCE)||(angle_difference(i) < -MAX_ANGLE_DIFFERENCE) */
  /*          if(angle_difference(i)<0) */
  /*              ROLLOVER_FLAG(i) = ROLLOVER_FLAG(i) + 1; */
  /*          else */
  /*              ROLLOVER_FLAG(i) = ROLLOVER_FLAG(i) - 1; */
  /*          end */
  /*      end */
  /*  end */
  /*   */
  /*  angle_aro = angle + ANTI_ROLLOVER_CORRECTION_MATRIX*ROLLOVER_FLAG; */
  /*  convert angles to radian */
  /* 'func_anti_rollOver:77' angle_aro = DEG2RAD*angle_aro; */
  /*  calibration materials */
  /*  max_angle = DEG2RAD*[MAX_RP_ANGLE_RIGHT,MAX_RP_ANGLE_LEFT,MAX_DV_ANGLE_RIGHT,MAX_DV_ANGLE_LEFT].'; */
  /* 'func_anti_rollOver:81' min_angle = DEG2RAD*[MIN_RP_ANGLE_RIGHT,MIN_RP_ANGLE_LEFT,MIN_DV_ANGLE_RIGHT,MIN_DV_ANGLE_LEFT].'; */
  /*  delta_angle_matrix = diag(max_angle-min_angle); */
  /*  calibrate angles */
  /* 'func_anti_rollOver:85' angle_aro = angle_aro-min_angle; */
  angle_aro_idx_0 = 0.017453292519943295 * controller_U.angle[0] -
    0.017453292519943295 * controller_DW.MIN_RP_ANGLE_RIGHT;
  angle_aro_idx_1 = 0.017453292519943295 * controller_U.angle[1] -
    0.017453292519943295 * controller_DW.MIN_RP_ANGLE_LEFT;
  angle_aro_idx_2 = 0.017453292519943295 * controller_U.angle[2] -
    0.017453292519943295 * controller_DW.MIN_DV_ANGLE_RIGHT;
  angle_aro_idx_3 = 0.017453292519943295 * controller_U.angle[3] -
    0.017453292519943295 * controller_DW.MIN_DV_ANGLE_LEFT;

  /*  filtering */
  /*  angle_f = func_lowpass_filter(angle_aro,angle_aro_prev); */
  /* '<S2>:1:121' angle_f = angle_aro; */
  /*  PID scheme */
  /* '<S2>:1:124' [u_pid,err] = func_pid_controller(angle_f,uf,prev_err,pid_gain); */
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
  /*  gain: control gain vector 4-by-1 vec */
  /*    gain(1): right-left forelimb Kp */
  /*    gain(2): right-left leg Kp */
  /*    gain(3): right-left forelimb Kd */
  /*    gain(4): right-left leg Kd */
  /*    gain(5): right-left forelimb Ki */
  /*    gain(6): right-left leg Ki */
  /*  u: control action, 4-by-1 vec */
  /*    u(1): right forelimb */
  /*    u(2): left forelimb */
  /*    u(3): right leg */
  /*    u(4): left leg */
  /*  err: to save err vec in memory */
  /*  by ALireza Ramezani, 8-31-2015, Champaign, IL */
  /* 'func_pid_controller:33' u = zeros(4,1); */
  /* 'func_pid_controller:34' err = zeros(4,1); */
  /*  err */
  /* 'func_pid_controller:35' derr = zeros(4,1); */
  /*  derivative of err */
  /* 'func_pid_controller:36' serr = zeros(4,1); */
  /*  integration of err */
  /*  global vars */
  /* 'func_pid_controller:52' Kp = [gain(1),0,0,0;... */
  /* 'func_pid_controller:53'       0, gain(1),0,0;... */
  /* 'func_pid_controller:54'       0, 0,gain(2),0;... */
  /* 'func_pid_controller:55'       0, 0, 0,gain(2)]; */
  /* 'func_pid_controller:57' Kd = [gain(3),0,0,0;... */
  /* 'func_pid_controller:58'       0, gain(3),0,0;... */
  /* 'func_pid_controller:59'       0, 0,gain(4),0;... */
  /* 'func_pid_controller:60'       0, 0, 0,gain(4)]; */
  /* 'func_pid_controller:62' Ki = [gain(5),0,0,0;... */
  /* 'func_pid_controller:63'       0, gain(5),0,0;... */
  /* 'func_pid_controller:64'       0, 0,gain(6),0;... */
  /* 'func_pid_controller:65'       0, 0, 0,gain(6)]; */
  /*  compute error and derr */
  /* 'func_pid_controller:68' err = angle - des_angle; */
  err_idx_0 = angle_aro_idx_0 - rtb_uf_idx_0;
  err_idx_1 = angle_aro_idx_1 - rtb_uf_idx_1;
  err_idx_2 = angle_aro_idx_2 - rtb_uf_idx_2;
  err_idx_3 = angle_aro_idx_3 - rtb_uf_idx_3;

  /* 'func_pid_controller:69' derr = (err-prev_err)/SAMPLING_INTERVAL; */
  /* 'func_pid_controller:70' ERR_INTEGRALE = ERR_INTEGRALE + err; */
  controller_DW.ERR_INTEGRALE[0] += err_idx_0;
  controller_DW.ERR_INTEGRALE[1] += err_idx_1;
  controller_DW.ERR_INTEGRALE[2] += err_idx_2;
  controller_DW.ERR_INTEGRALE[3] += err_idx_3;

  /* 'func_pid_controller:72' for i=1:4 */
  /* 'func_pid_controller:73' if ERR_INTEGRALE(i)>ANTI_WINDUP_THRESHOLD */
  if (controller_DW.ERR_INTEGRALE[0] > controller_DW.ANTI_WINDUP_THRESHOLD) {
    /* 'func_pid_controller:74' ERR_INTEGRALE(i) = ANTI_WINDUP_THRESHOLD; */
    controller_DW.ERR_INTEGRALE[0] = controller_DW.ANTI_WINDUP_THRESHOLD;
  }

  /* 'func_pid_controller:76' if ERR_INTEGRALE(i)<-ANTI_WINDUP_THRESHOLD */
  if (controller_DW.ERR_INTEGRALE[0] < -controller_DW.ANTI_WINDUP_THRESHOLD) {
    /* 'func_pid_controller:77' ERR_INTEGRALE(i) = -ANTI_WINDUP_THRESHOLD; */
    controller_DW.ERR_INTEGRALE[0] = -controller_DW.ANTI_WINDUP_THRESHOLD;
  }

  /* 'func_pid_controller:73' if ERR_INTEGRALE(i)>ANTI_WINDUP_THRESHOLD */
  if (controller_DW.ERR_INTEGRALE[1] > controller_DW.ANTI_WINDUP_THRESHOLD) {
    /* 'func_pid_controller:74' ERR_INTEGRALE(i) = ANTI_WINDUP_THRESHOLD; */
    controller_DW.ERR_INTEGRALE[1] = controller_DW.ANTI_WINDUP_THRESHOLD;
  }

  /* 'func_pid_controller:76' if ERR_INTEGRALE(i)<-ANTI_WINDUP_THRESHOLD */
  if (controller_DW.ERR_INTEGRALE[1] < -controller_DW.ANTI_WINDUP_THRESHOLD) {
    /* 'func_pid_controller:77' ERR_INTEGRALE(i) = -ANTI_WINDUP_THRESHOLD; */
    controller_DW.ERR_INTEGRALE[1] = -controller_DW.ANTI_WINDUP_THRESHOLD;
  }

  /* 'func_pid_controller:73' if ERR_INTEGRALE(i)>ANTI_WINDUP_THRESHOLD */
  if (controller_DW.ERR_INTEGRALE[2] > controller_DW.ANTI_WINDUP_THRESHOLD) {
    /* 'func_pid_controller:74' ERR_INTEGRALE(i) = ANTI_WINDUP_THRESHOLD; */
    controller_DW.ERR_INTEGRALE[2] = controller_DW.ANTI_WINDUP_THRESHOLD;
  }

  /* 'func_pid_controller:76' if ERR_INTEGRALE(i)<-ANTI_WINDUP_THRESHOLD */
  if (controller_DW.ERR_INTEGRALE[2] < -controller_DW.ANTI_WINDUP_THRESHOLD) {
    /* 'func_pid_controller:77' ERR_INTEGRALE(i) = -ANTI_WINDUP_THRESHOLD; */
    controller_DW.ERR_INTEGRALE[2] = -controller_DW.ANTI_WINDUP_THRESHOLD;
  }

  /* 'func_pid_controller:73' if ERR_INTEGRALE(i)>ANTI_WINDUP_THRESHOLD */
  if (controller_DW.ERR_INTEGRALE[3] > controller_DW.ANTI_WINDUP_THRESHOLD) {
    /* 'func_pid_controller:74' ERR_INTEGRALE(i) = ANTI_WINDUP_THRESHOLD; */
    controller_DW.ERR_INTEGRALE[3] = controller_DW.ANTI_WINDUP_THRESHOLD;
  }

  /* 'func_pid_controller:76' if ERR_INTEGRALE(i)<-ANTI_WINDUP_THRESHOLD */
  if (controller_DW.ERR_INTEGRALE[3] < -controller_DW.ANTI_WINDUP_THRESHOLD) {
    /* 'func_pid_controller:77' ERR_INTEGRALE(i) = -ANTI_WINDUP_THRESHOLD; */
    controller_DW.ERR_INTEGRALE[3] = -controller_DW.ANTI_WINDUP_THRESHOLD;
  }

  /*  computer u */
  /* 'func_pid_controller:81' u = -Kp*err -Kd*derr -Ki*ERR_INTEGRALE; */
  tmp_3[0] = controller_U.pid_gian[0];
  tmp_3[4] = 0.0;
  tmp_3[8] = 0.0;
  tmp_3[12] = 0.0;
  tmp_3[1] = 0.0;
  tmp_3[5] = controller_U.pid_gian[0];
  tmp_3[9] = 0.0;
  tmp_3[13] = 0.0;
  tmp_3[2] = 0.0;
  tmp_3[6] = 0.0;
  tmp_3[10] = controller_U.pid_gian[1];
  tmp_3[14] = 0.0;
  tmp_3[3] = 0.0;
  tmp_3[7] = 0.0;
  tmp_3[11] = 0.0;
  tmp_3[15] = controller_U.pid_gian[1];
  for (argout = 0; argout < 4; argout++) {
    tmp_4[argout << 2] = -tmp_3[argout << 2];
    tmp_4[1 + (argout << 2)] = -tmp_3[(argout << 2) + 1];
    tmp_4[2 + (argout << 2)] = -tmp_3[(argout << 2) + 2];
    tmp_4[3 + (argout << 2)] = -tmp_3[(argout << 2) + 3];
  }

  tmp_5[0] = controller_U.pid_gian[2];
  tmp_5[4] = 0.0;
  tmp_5[8] = 0.0;
  tmp_5[12] = 0.0;
  tmp_5[1] = 0.0;
  tmp_5[5] = controller_U.pid_gian[2];
  tmp_5[9] = 0.0;
  tmp_5[13] = 0.0;
  tmp_5[2] = 0.0;
  tmp_5[6] = 0.0;
  tmp_5[10] = controller_U.pid_gian[3];
  tmp_5[14] = 0.0;
  tmp_5[3] = 0.0;
  tmp_5[7] = 0.0;
  tmp_5[11] = 0.0;
  tmp_5[15] = controller_U.pid_gian[3];
  err_idx_0_0 = (err_idx_0 - controller_DW.UnitDelay2_DSTATE[0]) /
    controller_DW.SAMPLING_INTERVAL;
  delta_max_angle_idx_0 = (err_idx_1 - controller_DW.UnitDelay2_DSTATE[1]) /
    controller_DW.SAMPLING_INTERVAL;
  delta_max_angle_idx_1 = (err_idx_2 - controller_DW.UnitDelay2_DSTATE[2]) /
    controller_DW.SAMPLING_INTERVAL;
  delta_max_angle_idx_2 = (err_idx_3 - controller_DW.UnitDelay2_DSTATE[3]) /
    controller_DW.SAMPLING_INTERVAL;
  for (argout = 0; argout < 4; argout++) {
    delta_max_angle_idx_3 = tmp_4[argout + 12] * err_idx_3 + (tmp_4[argout + 8] *
      err_idx_2 + (tmp_4[argout + 4] * err_idx_1 + tmp_4[argout] * err_idx_0));
    tmp_1[argout] = delta_max_angle_idx_3;
  }

  for (argout = 0; argout < 4; argout++) {
    delta_max_angle_idx_3 = tmp_5[argout + 12] * delta_max_angle_idx_2 +
      (tmp_5[argout + 8] * delta_max_angle_idx_1 + (tmp_5[argout + 4] *
        delta_max_angle_idx_0 + tmp_5[argout] * err_idx_0_0));
    tmp_2[argout] = delta_max_angle_idx_3;
  }

  tmp_6[0] = controller_U.pid_gian[4];
  tmp_6[4] = 0.0;
  tmp_6[8] = 0.0;
  tmp_6[12] = 0.0;
  tmp_6[1] = 0.0;
  tmp_6[5] = controller_U.pid_gian[4];
  tmp_6[9] = 0.0;
  tmp_6[13] = 0.0;
  tmp_6[2] = 0.0;
  tmp_6[6] = 0.0;
  tmp_6[10] = controller_U.pid_gian[5];
  tmp_6[14] = 0.0;
  tmp_6[3] = 0.0;
  tmp_6[7] = 0.0;
  tmp_6[11] = 0.0;
  tmp_6[15] = controller_U.pid_gian[5];
  for (argout = 0; argout < 4; argout++) {
    u_pid[argout] = (tmp_1[argout] - tmp_2[argout]) - (((tmp_6[argout + 4] *
      controller_DW.ERR_INTEGRALE[1] + tmp_6[argout] *
      controller_DW.ERR_INTEGRALE[0]) + tmp_6[argout + 8] *
      controller_DW.ERR_INTEGRALE[2]) + tmp_6[argout + 12] *
      controller_DW.ERR_INTEGRALE[3]);
  }

  /*  turn-off the controller immediately when hit the limits */
  /* 'func_pid_controller:84' DEG2RAD = pi/180; */
  /*  rad\deg */
  /* 'func_pid_controller:85' max_angle = DEG2RAD*[MAX_RP_ANGLE_RIGHT,MAX_RP_ANGLE_LEFT,MAX_DV_ANGLE_RIGHT,MAX_DV_ANGLE_LEFT].'; */
  /* 'func_pid_controller:86' min_angle = DEG2RAD*[MIN_RP_ANGLE_RIGHT,MIN_RP_ANGLE_LEFT,MIN_DV_ANGLE_RIGHT,MIN_DV_ANGLE_LEFT].'; */
  /* 'func_pid_controller:87' delta_max_angle = max_angle-min_angle; */
  delta_max_angle_idx_0 = 0.017453292519943295 *
    controller_DW.MAX_RP_ANGLE_RIGHT - 0.017453292519943295 *
    controller_DW.MIN_RP_ANGLE_RIGHT;
  delta_max_angle_idx_1 = 0.017453292519943295 * controller_DW.MAX_RP_ANGLE_LEFT
    - 0.017453292519943295 * controller_DW.MIN_RP_ANGLE_LEFT;
  delta_max_angle_idx_2 = 0.017453292519943295 *
    controller_DW.MAX_DV_ANGLE_RIGHT - 0.017453292519943295 *
    controller_DW.MIN_DV_ANGLE_RIGHT;
  delta_max_angle_idx_3 = 0.017453292519943295 * controller_DW.MAX_DV_ANGLE_LEFT
    - 0.017453292519943295 * controller_DW.MIN_DV_ANGLE_LEFT;

  /* 'func_pid_controller:88' for i=1:4 */
  /* 'func_pid_controller:89' if((angle(i)>delta_max_angle(i))||(angle(i)<0)) */
  if (((angle_aro_idx_0 > delta_max_angle_idx_0) || (angle_aro_idx_0 < 0.0)) &&
      ((rtb_uf_idx_0 < 0.0) || (rtb_uf_idx_0 > delta_max_angle_idx_0))) {
    /* 'func_pid_controller:90' if (des_angle(i) < 0 || des_angle(i) > delta_max_angle(i)) */
    /* 'func_pid_controller:91' u(i) = 0; */
    u_pid[0] = 0.0;
  }

  /* 'func_pid_controller:89' if((angle(i)>delta_max_angle(i))||(angle(i)<0)) */
  if (((angle_aro_idx_1 > delta_max_angle_idx_1) || (angle_aro_idx_1 < 0.0)) &&
      ((rtb_uf_idx_1 < 0.0) || (rtb_uf_idx_1 > delta_max_angle_idx_1))) {
    /* 'func_pid_controller:90' if (des_angle(i) < 0 || des_angle(i) > delta_max_angle(i)) */
    /* 'func_pid_controller:91' u(i) = 0; */
    u_pid[1] = 0.0;
  }

  /* 'func_pid_controller:89' if((angle(i)>delta_max_angle(i))||(angle(i)<0)) */
  if (((angle_aro_idx_2 > delta_max_angle_idx_2) || (angle_aro_idx_2 < 0.0)) &&
      ((rtb_uf_idx_2 < 0.0) || (rtb_uf_idx_2 > delta_max_angle_idx_2))) {
    /* 'func_pid_controller:90' if (des_angle(i) < 0 || des_angle(i) > delta_max_angle(i)) */
    /* 'func_pid_controller:91' u(i) = 0; */
    u_pid[2] = 0.0;
  }

  /* 'func_pid_controller:89' if((angle(i)>delta_max_angle(i))||(angle(i)<0)) */
  if (((angle_aro_idx_3 > delta_max_angle_idx_3) || (angle_aro_idx_3 < 0.0)) &&
      ((rtb_uf_idx_3 < 0.0) || (rtb_uf_idx_3 > delta_max_angle_idx_3))) {
    /* 'func_pid_controller:90' if (des_angle(i) < 0 || des_angle(i) > delta_max_angle(i)) */
    /* 'func_pid_controller:91' u(i) = 0; */
    u_pid[3] = 0.0;
  }

  /*  for i=1:4 */
  /*      if(des_angle(i) > 0 && des_angle(i) < delta_max_angle(i)) */
  /*          LOCK_MOTORS(i) = false; */
  /*      end */
  /*  end */
  /*  for i=1:4 */
  /*      if(LOCK_MOTORS(i)) */
  /*          u(i) = 0; */
  /*      end */
  /*  end */
  /*  map pid efforts to drv (leg actuators are dc motors and armwing actuators */
  /*  previously were dc motors) */
  /* '<S2>:1:128' us = func_map_pid_to_servo(u_pid,err); */
  /*  func_map_pid_to_servo: map controller input to drv8835 commands */
  /*  u: control action, 4-by-1 vec */
  /*    u(1): right forelimb */
  /*    u(2): left forelimb */
  /*    u(3): right leg */
  /*    u(4): left leg */
  /*  error: pos error, 4-by-1 vec */
  /*    error(1): right forelimb */
  /*    error(2): left forelimb */
  /*    error(3): right leg */
  /*    error(4): left leg */
  /*  u_drv: drv8835 control command, 8-by-1 vec */
  /*    u_drv(1): right forelimb */
  /*    u_drv(2): left forelimb */
  /*    u_drv(3): right leg */
  /*    u_drv(4): left leg */
  /*  by ALireza Ramezani, 8-31-2015, Champaign, IL */
  /* 'func_map_pid_to_servo:20' u_drv = zeros(8,1); */
  /* 'func_map_pid_to_servo:21' H = zeros(8,4); */
  /*  used for saturating control command to drv8835 */
  /*  map */
  /* 'func_map_pid_to_servo:29' H = [func_heaviside(u(1)),0,0,0;... */
  /* 'func_map_pid_to_servo:30'      func_heaviside(-u(1)),0,0,0;... */
  /* 'func_map_pid_to_servo:31'      0,func_heaviside(u(2)),0,0;... */
  /* 'func_map_pid_to_servo:32'      0,func_heaviside(-u(2)),0,0;... */
  /* 'func_map_pid_to_servo:33'      0,0,func_heaviside(u(3)),0;... */
  /* 'func_map_pid_to_servo:34'      0,0,func_heaviside(-u(3)),0;... */
  /* 'func_map_pid_to_servo:35'      0,0,0,func_heaviside(u(4));... */
  /* 'func_map_pid_to_servo:36'      0,0,0,func_heaviside(-u(4))]; */
  /*  func_heaviside: heaviside function */
  /*  by Alireza Ramezani,9-1-2015, Champaign, IL */
  /* 'func_heaviside:5' argout = 0; */
  argout = 0;

  /* 'func_heaviside:7' if argin>=0 */
  if (u_pid[0] >= 0.0) {
    /* 'func_heaviside:8' argout = 1; */
    argout = 1;
  }

  /*  end of code */
  /*  func_heaviside: heaviside function */
  /*  by Alireza Ramezani,9-1-2015, Champaign, IL */
  /* 'func_heaviside:5' argout = 0; */
  b_argout = 0;

  /* 'func_heaviside:7' if argin>=0 */
  if (-u_pid[0] >= 0.0) {
    /* 'func_heaviside:8' argout = 1; */
    b_argout = 1;
  }

  /*  end of code */
  /*  func_heaviside: heaviside function */
  /*  by Alireza Ramezani,9-1-2015, Champaign, IL */
  /* 'func_heaviside:5' argout = 0; */
  c_argout = 0;

  /* 'func_heaviside:7' if argin>=0 */
  if (u_pid[1] >= 0.0) {
    /* 'func_heaviside:8' argout = 1; */
    c_argout = 1;
  }

  /*  end of code */
  /*  func_heaviside: heaviside function */
  /*  by Alireza Ramezani,9-1-2015, Champaign, IL */
  /* 'func_heaviside:5' argout = 0; */
  d_argout = 0;

  /* 'func_heaviside:7' if argin>=0 */
  if (-u_pid[1] >= 0.0) {
    /* 'func_heaviside:8' argout = 1; */
    d_argout = 1;
  }

  /*  end of code */
  /*  func_heaviside: heaviside function */
  /*  by Alireza Ramezani,9-1-2015, Champaign, IL */
  /* 'func_heaviside:5' argout = 0; */
  e_argout = 0;

  /* 'func_heaviside:7' if argin>=0 */
  if (u_pid[2] >= 0.0) {
    /* 'func_heaviside:8' argout = 1; */
    e_argout = 1;
  }

  /*  end of code */
  /*  func_heaviside: heaviside function */
  /*  by Alireza Ramezani,9-1-2015, Champaign, IL */
  /* 'func_heaviside:5' argout = 0; */
  f_argout = 0;

  /* 'func_heaviside:7' if argin>=0 */
  if (-u_pid[2] >= 0.0) {
    /* 'func_heaviside:8' argout = 1; */
    f_argout = 1;
  }

  /*  end of code */
  /*  func_heaviside: heaviside function */
  /*  by Alireza Ramezani,9-1-2015, Champaign, IL */
  /* 'func_heaviside:5' argout = 0; */
  g_argout = 0;

  /* 'func_heaviside:7' if argin>=0 */
  if (u_pid[3] >= 0.0) {
    /* 'func_heaviside:8' argout = 1; */
    g_argout = 1;
  }

  /*  end of code */
  /*  func_heaviside: heaviside function */
  /*  by Alireza Ramezani,9-1-2015, Champaign, IL */
  /* 'func_heaviside:5' argout = 0; */
  h_argout = 0;

  /* 'func_heaviside:7' if argin>=0 */
  if (-u_pid[3] >= 0.0) {
    /* 'func_heaviside:8' argout = 1; */
    h_argout = 1;
  }

  /*  end of code */
  /*  saturate u */
  /* 'func_map_pid_to_servo:39' for i=1:4 */
  rtb_uf_idx_3 = u_pid[0];

  /* 'func_map_pid_to_servo:40' if u(i)> PID_SATURATION_THRESHOLD */
  if (u_pid[0] > controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_servo:41' u(i) = PID_SATURATION_THRESHOLD; */
    rtb_uf_idx_3 = controller_DW.PID_SATURATION_THRESHOLD;
  }

  /* 'func_map_pid_to_servo:44' if u(i)< -PID_SATURATION_THRESHOLD */
  if (rtb_uf_idx_3 < -controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_servo:45' u(i) = -PID_SATURATION_THRESHOLD; */
    rtb_uf_idx_3 = -controller_DW.PID_SATURATION_THRESHOLD;
  }

  rtb_uf_idx_0 = rtb_uf_idx_3;
  rtb_uf_idx_3 = u_pid[1];

  /* 'func_map_pid_to_servo:40' if u(i)> PID_SATURATION_THRESHOLD */
  if (u_pid[1] > controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_servo:41' u(i) = PID_SATURATION_THRESHOLD; */
    rtb_uf_idx_3 = controller_DW.PID_SATURATION_THRESHOLD;
  }

  /* 'func_map_pid_to_servo:44' if u(i)< -PID_SATURATION_THRESHOLD */
  if (rtb_uf_idx_3 < -controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_servo:45' u(i) = -PID_SATURATION_THRESHOLD; */
    rtb_uf_idx_3 = -controller_DW.PID_SATURATION_THRESHOLD;
  }

  rtb_uf_idx_1 = rtb_uf_idx_3;
  rtb_uf_idx_3 = u_pid[2];

  /* 'func_map_pid_to_servo:40' if u(i)> PID_SATURATION_THRESHOLD */
  if (u_pid[2] > controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_servo:41' u(i) = PID_SATURATION_THRESHOLD; */
    rtb_uf_idx_3 = controller_DW.PID_SATURATION_THRESHOLD;
  }

  /* 'func_map_pid_to_servo:44' if u(i)< -PID_SATURATION_THRESHOLD */
  if (rtb_uf_idx_3 < -controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_servo:45' u(i) = -PID_SATURATION_THRESHOLD; */
    rtb_uf_idx_3 = -controller_DW.PID_SATURATION_THRESHOLD;
  }

  rtb_uf_idx_2 = rtb_uf_idx_3;
  rtb_uf_idx_3 = u_pid[3];

  /* 'func_map_pid_to_servo:40' if u(i)> PID_SATURATION_THRESHOLD */
  if (u_pid[3] > controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_servo:41' u(i) = PID_SATURATION_THRESHOLD; */
    rtb_uf_idx_3 = controller_DW.PID_SATURATION_THRESHOLD;
  }

  /* 'func_map_pid_to_servo:44' if u(i)< -PID_SATURATION_THRESHOLD */
  if (rtb_uf_idx_3 < -controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_servo:45' u(i) = -PID_SATURATION_THRESHOLD; */
    rtb_uf_idx_3 = -controller_DW.PID_SATURATION_THRESHOLD;
  }

  /*  if position err is too small: set ctrl effort to zero */
  /* 'func_map_pid_to_servo:51' for i=1:4 */
  /* 'func_map_pid_to_servo:52' if abs(error(i))<PID_TRACKING_PRECISION_THRESHOLD */
  if (fabs(err_idx_0) < controller_DW.PID_TRACKING_PRECISION_THRESHOL) {
    /* 'func_map_pid_to_servo:53' u(i) = 0; */
    rtb_uf_idx_0 = 0.0;
  }

  /* 'func_map_pid_to_servo:52' if abs(error(i))<PID_TRACKING_PRECISION_THRESHOLD */
  if (fabs(err_idx_1) < controller_DW.PID_TRACKING_PRECISION_THRESHOL) {
    /* 'func_map_pid_to_servo:53' u(i) = 0; */
    rtb_uf_idx_1 = 0.0;
  }

  /* 'func_map_pid_to_servo:52' if abs(error(i))<PID_TRACKING_PRECISION_THRESHOLD */
  if (fabs(err_idx_2) < controller_DW.PID_TRACKING_PRECISION_THRESHOL) {
    /* 'func_map_pid_to_servo:53' u(i) = 0; */
    rtb_uf_idx_2 = 0.0;
  }

  /* 'func_map_pid_to_servo:52' if abs(error(i))<PID_TRACKING_PRECISION_THRESHOLD */
  if (fabs(err_idx_3) < controller_DW.PID_TRACKING_PRECISION_THRESHOL) {
    /* 'func_map_pid_to_servo:53' u(i) = 0; */
    rtb_uf_idx_3 = 0.0;
  }

  /* 'func_map_pid_to_servo:58' u_drv = H*abs(u); */
  delta_max_angle_idx_0 = fabs(rtb_uf_idx_0);
  delta_max_angle_idx_1 = fabs(rtb_uf_idx_1);
  delta_max_angle_idx_2 = fabs(rtb_uf_idx_2);
  delta_max_angle_idx_3 = fabs(rtb_uf_idx_3);

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
    err_idx_0_0 = (real_T)argout_0[argout + 24] * delta_max_angle_idx_3 +
      ((real_T)argout_0[argout + 16] * delta_max_angle_idx_2 + ((real_T)
        argout_0[argout + 8] * delta_max_angle_idx_1 + (real_T)argout_0[argout] *
        delta_max_angle_idx_0));
    rtb_us[argout] = err_idx_0_0;
  }

  /*  map pid effort to tm320 (now, armwing actuators are bldc) */
  /* '<S2>:1:131' ubldc = func_map_pid_to_tms320(u_pid); */
  /*  func_map_pid_to_servo: map controller input to tms320 commands */
  /*  u: control action, 4-by-1 vec */
  /*    u(1): right forelimb */
  /*    u(2): left forelimb */
  /*    u(3): right leg */
  /*    u(4): left leg */
  /*  u_drv: drv8835 control command, 4-by-1 vec */
  /*    u_tms(1): right forelimb */
  /*    u_tms(2): left forelimb */
  /*    u_tms(3): right leg */
  /*    u_tms(4): left leg */
  /*  */
  /*  NOTE: MCB communicates with TMS320 over PWM, there are three states: */
  /*   1) 50(+-2)% duty cycle: stop */
  /*   2) 0(+-2)-to-50(+-2)% reverse */
  /*   3) 50(+-2)-to-100(+-2)% forward */
  /*  by ALireza Ramezani, 8-31-2015, Champaign, IL */
  /* 'func_map_pid_to_tms320:20' PWM_MAX_DC = 98; */
  /*  max duty cycle */
  /* 'func_map_pid_to_tms320:21' PWM_MIN_DC = 2; */
  /*  min duty cycle */
  /* 'func_map_pid_to_tms320:22' PWM_ZERO_DC = 50; */
  /*  zero command duty cycle */
  /* 'func_map_pid_to_tms320:24' u_tms = zeros(4,1); */
  /* 'func_map_pid_to_tms320:25' s = PWM_ZERO_DC*ones(4,1); */
  /*  used for saturating control command to tms320 */
  /*  saturate u */
  /* 'func_map_pid_to_tms320:30' for i=1:4 */
  err_idx_0_0 = u_pid[0];

  /* 'func_map_pid_to_tms320:31' if u(i)> PID_SATURATION_THRESHOLD */
  if (u_pid[0] > controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_tms320:32' u(i) = PID_SATURATION_THRESHOLD; */
    err_idx_0_0 = controller_DW.PID_SATURATION_THRESHOLD;
  }

  /* 'func_map_pid_to_tms320:35' if u(i)< -PID_SATURATION_THRESHOLD */
  if (err_idx_0_0 < -controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_tms320:36' u(i) = -PID_SATURATION_THRESHOLD; */
    err_idx_0_0 = -controller_DW.PID_SATURATION_THRESHOLD;
  }

  u_pid[0] = err_idx_0_0;
  err_idx_0_0 = u_pid[1];

  /* 'func_map_pid_to_tms320:31' if u(i)> PID_SATURATION_THRESHOLD */
  if (u_pid[1] > controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_tms320:32' u(i) = PID_SATURATION_THRESHOLD; */
    err_idx_0_0 = controller_DW.PID_SATURATION_THRESHOLD;
  }

  /* 'func_map_pid_to_tms320:35' if u(i)< -PID_SATURATION_THRESHOLD */
  if (err_idx_0_0 < -controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_tms320:36' u(i) = -PID_SATURATION_THRESHOLD; */
    err_idx_0_0 = -controller_DW.PID_SATURATION_THRESHOLD;
  }

  u_pid[1] = err_idx_0_0;
  err_idx_0_0 = u_pid[2];

  /* 'func_map_pid_to_tms320:31' if u(i)> PID_SATURATION_THRESHOLD */
  if (u_pid[2] > controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_tms320:32' u(i) = PID_SATURATION_THRESHOLD; */
    err_idx_0_0 = controller_DW.PID_SATURATION_THRESHOLD;
  }

  /* 'func_map_pid_to_tms320:35' if u(i)< -PID_SATURATION_THRESHOLD */
  if (err_idx_0_0 < -controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_tms320:36' u(i) = -PID_SATURATION_THRESHOLD; */
    err_idx_0_0 = -controller_DW.PID_SATURATION_THRESHOLD;
  }

  u_pid[2] = err_idx_0_0;
  err_idx_0_0 = u_pid[3];

  /* 'func_map_pid_to_tms320:31' if u(i)> PID_SATURATION_THRESHOLD */
  if (u_pid[3] > controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_tms320:32' u(i) = PID_SATURATION_THRESHOLD; */
    err_idx_0_0 = controller_DW.PID_SATURATION_THRESHOLD;
  }

  /* 'func_map_pid_to_tms320:35' if u(i)< -PID_SATURATION_THRESHOLD */
  if (err_idx_0_0 < -controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_tms320:36' u(i) = -PID_SATURATION_THRESHOLD; */
    err_idx_0_0 = -controller_DW.PID_SATURATION_THRESHOLD;
  }

  /* 'func_map_pid_to_tms320:40' u_tms = u + s; */
  u_pid[0] += 50.0;
  u_pid[1] += 50.0;
  u_pid[2] += 50.0;

  /* 'func_map_pid_to_tms320:42' for i=1:4 */
  delta_max_angle_idx_0 = u_pid[0];

  /* 'func_map_pid_to_tms320:43' if u_tms(i)> PWM_MAX_DC */
  if (u_pid[0] > 98.0) {
    /* 'func_map_pid_to_tms320:44' u_tms(i) = PWM_MAX_DC; */
    delta_max_angle_idx_0 = 98.0;
  }

  /* 'func_map_pid_to_tms320:47' if u_tms(i)< PWM_MIN_DC */
  if (delta_max_angle_idx_0 < 2.0) {
    /* 'func_map_pid_to_tms320:48' u_tms(i) = PWM_MIN_DC; */
    delta_max_angle_idx_0 = 2.0;
  }

  u_pid[0] = delta_max_angle_idx_0;
  delta_max_angle_idx_0 = u_pid[1];

  /* 'func_map_pid_to_tms320:43' if u_tms(i)> PWM_MAX_DC */
  if (u_pid[1] > 98.0) {
    /* 'func_map_pid_to_tms320:44' u_tms(i) = PWM_MAX_DC; */
    delta_max_angle_idx_0 = 98.0;
  }

  /* 'func_map_pid_to_tms320:47' if u_tms(i)< PWM_MIN_DC */
  if (delta_max_angle_idx_0 < 2.0) {
    /* 'func_map_pid_to_tms320:48' u_tms(i) = PWM_MIN_DC; */
    delta_max_angle_idx_0 = 2.0;
  }

  u_pid[1] = delta_max_angle_idx_0;
  delta_max_angle_idx_0 = u_pid[2];

  /* 'func_map_pid_to_tms320:43' if u_tms(i)> PWM_MAX_DC */
  if (u_pid[2] > 98.0) {
    /* 'func_map_pid_to_tms320:44' u_tms(i) = PWM_MAX_DC; */
    delta_max_angle_idx_0 = 98.0;
  }

  /* 'func_map_pid_to_tms320:47' if u_tms(i)< PWM_MIN_DC */
  if (delta_max_angle_idx_0 < 2.0) {
    /* 'func_map_pid_to_tms320:48' u_tms(i) = PWM_MIN_DC; */
    delta_max_angle_idx_0 = 2.0;
  }

  u_pid[2] = delta_max_angle_idx_0;
  delta_max_angle_idx_0 = err_idx_0_0 + 50.0;

  /* 'func_map_pid_to_tms320:43' if u_tms(i)> PWM_MAX_DC */
  if (err_idx_0_0 + 50.0 > 98.0) {
    /* 'func_map_pid_to_tms320:44' u_tms(i) = PWM_MAX_DC; */
    delta_max_angle_idx_0 = 98.0;
  }

  /* 'func_map_pid_to_tms320:47' if u_tms(i)< PWM_MIN_DC */
  if (delta_max_angle_idx_0 < 2.0) {
    /* 'func_map_pid_to_tms320:48' u_tms(i) = PWM_MIN_DC; */
    delta_max_angle_idx_0 = 2.0;
  }

  /* Outport: '<Root>/ubldc' incorporates:
   *  MATLAB Function: '<Root>/func_actuator_controller'
   */
  /*  end of code */
  controller_Y.ubldc[0] = u_pid[0];
  controller_Y.ubldc[1] = u_pid[1];
  controller_Y.ubldc[2] = u_pid[2];
  controller_Y.ubldc[3] = delta_max_angle_idx_0;

  /* Update for UnitDelay: '<Root>/Unit Delay2' incorporates:
   *  MATLAB Function: '<Root>/func_actuator_controller'
   */
  /*  some update for memory */
  /* '<S2>:1:134' angle2mem = angle; */
  /* '<S2>:1:135' angle_aro2mem = angle_aro; */
  /* '<S2>:1:136' err2mem = err; */
  controller_DW.UnitDelay2_DSTATE[0] = err_idx_0;
  controller_DW.UnitDelay2_DSTATE[1] = err_idx_1;
  controller_DW.UnitDelay2_DSTATE[2] = err_idx_2;
  controller_DW.UnitDelay2_DSTATE[3] = err_idx_3;

  /* Outport: '<Root>/debug' incorporates:
   *  MATLAB Function: '<Root>/func_actuator_controller'
   */
  /* '<S2>:1:137' debug = angle_f; */
  controller_Y.debug[0] = angle_aro_idx_0;
  controller_Y.debug[1] = angle_aro_idx_1;
  controller_Y.debug[2] = angle_aro_idx_2;
  controller_Y.debug[3] = angle_aro_idx_3;

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
  rtb_DigitalClock = ((controller_M->Timing.clockTick0) * 0.002);

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

  /* Update for UnitDelay: '<S5>/UD' */
  controller_DW.UD_DSTATE[0] = rtb_TSamp_idx_0;
  controller_DW.UD_DSTATE[1] = rtb_TSamp_idx_1;
  controller_DW.UD_DSTATE[2] = rtb_TSamp_idx_2;

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.002, which is the step size
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
  memcpy(&controller_DW.IC2[0], &controller_ConstP.IC2_Value[0], 14U * sizeof
         (real_T));
  controller_DW.IC2_FirstOutputTime = true;
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF] controller.c
 */
