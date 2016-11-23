/*
 * File: controller.c
 *
 * Code generated for Simulink model :controller.
 *
 * Model version      : 1.334
 * Simulink Coder version    : 8.6 (R2014a) 27-Dec-2013
 * TLC version       : 8.6 (Jan 30 2014)
 * C/C++ source code generated on  : Sat May 14 13:49:26 2016
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
  real_T dangle[3];
  real_T dtmp;
  real_T tmp2[3];
  real_T R[9];
  static const int8_T a[9] = { 0, -1, 0, 1, 0, 0, 0, 0, 1 };

  real_T fwd[4];
  static const int8_T a_0[12] = { 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 };

  real_T u_pid[4];
  int32_T b_argout;
  int32_T c_argout;
  int32_T d_argout;
  int32_T e_argout;
  int32_T f_argout;
  int32_T g_argout;
  int32_T h_argout;
  real_T rtb_us[8];
  real_T rtb_ramp_gain[6];
  real_T rtb_x[12];
  real_T rtb_par[16];
  real_T rtb_delta_par[16];
  real_T rtb_delta_x[12];
  int32_T i;
  static const real_T tmp[12] = { 0.0, -0.2251, 0.0, 0.0, 0.0, 0.0, 0.0, -0.2608,
    0.0, 0.8556, 0.0, 0.0 };

  static const real_T tmp_0[16] = { 10.0, 1.0839, 0.2665, -0.2956, 0.15, 1.5363,
    0.0022, 0.15, 1.5415, -0.0004, 0.05, -1.5708, 0.1, 0.05, -1.5708, 0.1 };

  real_T a_1[3];
  real_T a_2[2];
  real_T tmp_1[8];
  real_T a_3[2];
  real_T rtb_ramp_gain_0[16];
  real_T rtb_ramp_gain_1[16];
  real_T rtb_ramp_gain_2[16];
  real_T rtb_ramp_gain_3[4];
  real_T rtb_ramp_gain_4[4];
  real_T rtb_ramp_gain_5[16];
  int8_T argout[32];
  real_T rtb_out;
  real_T u_pid_0;
  real_T rtb_aroAngle_idx_0;
  real_T rtb_aroAngle_idx_2;
  real_T rtb_aroAngle_idx_1;
  real_T rtb_uf_idx_0;
  real_T rtb_uf_idx_1;
  real_T rtb_uf_idx_2;
  real_T rtb_uf_idx_3;
  real_T rtb_out_idx_0;
  real_T rtb_aro_angle_idx_0;
  real_T rtb_out_idx_1;
  real_T rtb_aro_angle_idx_1;
  real_T rtb_out_idx_2;
  real_T rtb_aro_angle_idx_3;
  real_T rtb_deg2rad_idx_0;
  real_T rtb_deg2rad_idx_1;
  real_T rtb_deg2rad_idx_2;
  real_T rtb_deg2rad_idx_3;
  real_T err_idx_0;
  real_T err_idx_1;
  real_T err_idx_2;
  real_T err_idx_3;

  /* MATLAB Function: '<S10>/aro' incorporates:
   *  Inport: '<Root>/pitch'
   *  Inport: '<Root>/roll'
   *  Inport: '<Root>/yaw'
   *  SignalConversion: '<S12>/TmpSignal ConversionAt SFunction Inport1'
   *  UnitDelay: '<S10>/Unit Delay1'
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  /* MATLAB Function 'state estimator/aro/aro': '<S12>:1' */
  /*  angle: euler angles from IMU in deg., e.g., [roll,pitch,yaw]  */
  /*  */
  /*  prevAngle: variable to keep euler angles from previous samples */
  /*  */
  /*  prevAroAngle: variable to keep aro euler angles from previous samples */
  /*  */
  /*  angleRate: angular velocity in body frame [wx,wy,wz] */
  /*  */
  /*  aroAngle: aro euler angle */
  /*  */
  /*  angle2Mem: keep euler angles from previous samples */
  /*  */
  /*  aroAngle2Mem: keep aro euler angles from previous samples */
  /*  */
  /*  IMU has the following ranges: */
  /*  */
  /*    roll:            -180<= qx <=180      */
  /*    pitch:           -90<= qy <=90 */
  /*    heading:         -180<= qz <=180 */
  /*  */
  /*  check the following events: */
  /*  */
  /*    roll,yaw:       -180    ->       180   */
  /*    roll,yaw:        180     ->      -180 */
  /*    pitch < -90:          -90 -89 -88 -87 etc.      */
  /*    pitch > 90:          90 89 88 87 etc.      */
  /*   */
  /*  pitch anti roll over is not finished yet!!!!!! */
  /*  */
  /*  By Alireza Ramezani, 4-16-2016, Champaign, IL */
  /* '<S12>:1:34' aroAngle = zeros(3,1); */
  /* '<S12>:1:35' angle2Mem = zeros(3,1); */
  /* '<S12>:1:36' aroAngle2Mem = zeros(3,1); */
  /*  ???? */
  /* '<S12>:1:39' ROLL_OVER_JUMP = 357; */
  /*  max. jumps for roll,pitch and yaw */
  /* '<S12>:1:42' maxDeltaAngle = 100; */
  /* '<S12>:1:44' corr = 360; */
  /*  comptue the difference */
  /* '<S12>:1:47' dangle = angle - prevAngle; */
  dangle[0] = controller_U.roll - controller_DW.UnitDelay3_DSTATE[0];
  dangle[1] = controller_U.pitch - controller_DW.UnitDelay3_DSTATE[1];
  dangle[2] = controller_U.yaw - controller_DW.UnitDelay3_DSTATE[2];

  /*  anti rollover for roll angle */
  /* '<S12>:1:50' i = 1; */
  /* '<S12>:1:51' dtmp = dangle(i); */
  dtmp = dangle[0];

  /* '<S12>:1:52' if dangle(i) > ROLL_OVER_JUMP */
  if (dangle[0] > 357.0) {
    /* '<S12>:1:53' dtmp = dangle(i) - corr; */
    dtmp = dangle[0] - 360.0;
  } else {
    if (dangle[0] > 100.0) {
      /* '<S12>:1:54' elseif dangle(i) > maxDeltaAngle */
      /* '<S12>:1:55' dtmp = 0; */
      dtmp = 0.0;
    }
  }

  /* '<S12>:1:58' if dangle(i) < -ROLL_OVER_JUMP */
  if (dangle[0] < -357.0) {
    /* '<S12>:1:59' dtmp = dangle(i) + corr; */
    dtmp = dangle[0] + 360.0;
  } else {
    if (dangle[0] < -100.0) {
      /* '<S12>:1:60' elseif dangle(i) < -maxDeltaAngle */
      /* '<S12>:1:61' dtmp = 0; */
      dtmp = 0.0;
    }
  }

  /*  integrate aro angle */
  /* '<S12>:1:65' aroAngle2Mem(i) = prevAroAngle(i) + dtmp; */
  rtb_aroAngle_idx_0 = controller_DW.UnitDelay1_DSTATE[0] + dtmp;

  /*  anti rollover for yaw angle */
  /* '<S12>:1:68' i = 3; */
  /* '<S12>:1:69' dtmp = dangle(i); */
  dtmp = dangle[2];

  /* '<S12>:1:70' if dangle(i) > ROLL_OVER_JUMP */
  if (dangle[2] > 357.0) {
    /* '<S12>:1:71' dtmp = dangle(i) - corr; */
    dtmp = dangle[2] - 360.0;
  } else {
    if (dangle[2] > 100.0) {
      /* '<S12>:1:72' elseif dangle(i) > maxDeltaAngle */
      /* '<S12>:1:73' dtmp = 0; */
      dtmp = 0.0;
    }
  }

  /* '<S12>:1:76' if dangle(i) < -ROLL_OVER_JUMP */
  if (dangle[2] < -357.0) {
    /* '<S12>:1:77' dtmp = dangle(i) + corr; */
    dtmp = dangle[2] + 360.0;
  } else {
    if (dangle[2] < -100.0) {
      /* '<S12>:1:78' elseif dangle(i) < -maxDeltaAngle */
      /* '<S12>:1:79' dtmp = 0; */
      dtmp = 0.0;
    }
  }

  /*  integrate aro angle */
  /* '<S12>:1:83' aroAngle2Mem(i) = prevAroAngle(i) + dtmp; */
  rtb_aroAngle_idx_2 = controller_DW.UnitDelay1_DSTATE[2] + dtmp;

  /*  anti rollover for pitch angle */
  /* '<S12>:1:87' i = 2; */
  /* '<S12>:1:88' dtmp = dangle(i); */
  dtmp = dangle[1];

  /*  ?????? */
  /*  if angleRate(i) > 0 && dtmp < 0 */
  /*      dtmp = -dtmp; */
  /*  end */
  /*   */
  /*  if angleRate(i) < 0 && dtmp > 0 */
  /*      dtmp = -dtmp;  */
  /*  end */
  /* '<S12>:1:99' if abs(dtmp) > maxDeltaAngle */
  if (fabs(dangle[1]) > 100.0) {
    /* '<S12>:1:100' dtmp = 0; */
    dtmp = 0.0;
  }

  /*  integrate aro angle */
  /* '<S12>:1:104' aroAngle2Mem(i) = prevAroAngle(i) + dtmp; */
  rtb_aroAngle_idx_1 = controller_DW.UnitDelay1_DSTATE[1] + dtmp;

  /* Gain: '<S7>/Gain' */
  /*  update  */
  /* '<S12>:1:107' angle2Mem = angle; */
  /*  output */
  /* '<S12>:1:110' aroAngle = aroAngle2Mem; */
  dangle[0] = 0.017453292519943295 * rtb_aroAngle_idx_0;
  dangle[1] = 0.017453292519943295 * rtb_aroAngle_idx_1;
  dangle[2] = 0.017453292519943295 * rtb_aroAngle_idx_2;

  /* MATLAB Function: '<S7>/func_state_estimator' incorporates:
   *  Inport: '<Root>/accelX'
   *  Inport: '<Root>/accelY'
   *  Inport: '<Root>/accelZ'
   *  Inport: '<Root>/wx'
   *  Inport: '<Root>/wy'
   *  Inport: '<Root>/wz'
   *  SignalConversion: '<S11>/TmpSignal ConversionAt SFunction Inport2'
   *  SignalConversion: '<S11>/TmpSignal ConversionAt SFunction Inport3'
   */
  /* MATLAB Function 'state estimator/func_state_estimator': '<S11>:1' */
  /*  state estimator */
  /*  */
  /*  euler: euler angle measurments (AHRS)(IMU coordinate frame) */
  /*     roll [rad] */
  /*     pitch */
  /*     yaw */
  /*  omega: imu measurments, angular velocity (IMU coordinate frame) */
  /*     p [rad/sec] */
  /*     q rate */
  /*     r rate */
  /*  accel: imu measurments, accelerations (IMU coordinate frame) */
  /*     accel X [m/s^2] */
  /*     accel Y */
  /*     accel Z */
  /*  x: estimated state  */
  /*    1) qx: roll [rad] */
  /*    2) qy: pitch */
  /*    3) qz: yaw */
  /*    4) px: x-pos [m] */
  /*    5) py: y-pos */
  /*    6) pz: z-pos */
  /*    7) wx:  angular velocity around x   [rad\sec] */
  /*    8) wy:  angular velocity around y */
  /*    9) wz:  angular velocity around z */
  /*    10) dpx: x-vel [m\sec] */
  /*    11) dpy: y-vel */
  /*    12) dpz: z-vel */
  /*  NOTE:  */
  /*        IMU is installed such that: */
  /*        x-axis is towards right arm wing (pitch) */
  /*        y-axis is towards head (roll) */
  /*        z-axis is upwards (yaw) */
  /*  */
  /*        IMU has the following ranges: */
  /*        roll:            -180<= qx <=180      */
  /*        pitch:           -90<= qy <=90 */
  /*        heading:         -180<= qz <=180 */
  /*  By Alireza Ramezani, 4-16-2016, Champaign, IL */
  /* '<S11>:1:42' x = zeros(12,1); */
  /* '<S11>:1:43' tmp1 = zeros(3,1); */
  /* '<S11>:1:44' tmp2 = zeros(3,1); */
  /* '<S11>:1:45' tmp3 = zeros(3,1); */
  /* '<S11>:1:46' R = zeros(3,3); */
  /* '<S11>:1:47' E = zeros(3,3); */
  /* '<S11>:1:48' dPos = zeros(3,1); */
  /* '<S11>:1:49' dvel = zeros(3,1); */
  /* '<S11>:1:50' dq = zeros(3,1); */
  /*  Computing position and velocity in the world frame */
  /*   */
  /*  Note: */
  /*            Pos = (Px,Py,Pz).': pos in world frame */
  /*            Vel = (Vx,Vy,Vz); vel in world frame */
  /*            v = (vx,vy,vz).': vel in body frame */
  /*            R: rotation matrix body->world */
  /*            a = (ax,ay,az).': acceleration in body frame */
  /*            omega: angular velocity in body frame */
  /*  */
  /*            integrate (     d(Pos)/dt = R*v     ) */
  /*            integrate (     d(v)/dt = a - omega x v     ) */
  /*  map from IMU coordinate to body coordinate: */
  /*  */
  /*  NOTE: IMU axeses are  not alligned with body coordinate frame, */
  /*        this map is not used for AHRS euler angles ????? */
  /* '<S11>:1:70' A = [0 1 0;... */
  /* '<S11>:1:71'     -1, 0, 0;... */
  /* '<S11>:1:72'     0, 0, 1]; */
  /*  euler angles */
  /* '<S11>:1:75' tmp1 = q; */
  /*  roll */
  /* '<S11>:1:78' x(1) = -tmp1(2); */
  rtb_x[0] = -dangle[1];

  /*  pitch */
  /* '<S11>:1:81' x(2) = -tmp1(1); */
  rtb_x[1] = -dangle[0];

  /*  yaw  */
  /* '<S11>:1:84' x(3) = -tmp1(3); */
  rtb_x[2] = -dangle[2];

  /*  angular rate and acceleration  in body frame  */
  /* '<S11>:1:87' tmp2 = A * omega; */
  for (i = 0; i < 3; i++) {
    tmp2[i] = (real_T)a[i + 6] * controller_U.wz + ((real_T)a[i + 3] *
      controller_U.wy + (real_T)a[i] * controller_U.wx);
  }

  /* '<S11>:1:88' tmp3 = A * accel; */
  /*  rotation body->world */
  /* '<S11>:1:91' qx = tmp1(1); */
  /* '<S11>:1:92' qy = tmp1(2); */
  /* '<S11>:1:93' qz = tmp1(3); */
  /* '<S11>:1:94' R(1,1) = cos(qy)*cos(qz); */
  R[0] = cos(dangle[1]) * cos(dangle[2]);

  /* '<S11>:1:95' R(1,2) = cos(qz)*sin(qx)*sin(qy) - cos(qx)*sin(qz); */
  R[3] = cos(dangle[2]) * sin(dangle[0]) * sin(dangle[1]) - cos(dangle[0]) * sin
    (dangle[2]);

  /* '<S11>:1:96' R(1,3) = sin(qx)*sin(qz) + cos(qx)*cos(qz)*sin(qy); */
  R[6] = cos(dangle[0]) * cos(dangle[2]) * sin(dangle[1]) + sin(dangle[0]) * sin
    (dangle[2]);

  /* '<S11>:1:97' R(2,1) = cos(qy)*sin(qz); */
  R[1] = cos(dangle[1]) * sin(dangle[2]);

  /* '<S11>:1:98' R(2,2) = cos(qx)*cos(qz) + sin(qx)*sin(qy)*sin(qz); */
  R[4] = sin(dangle[0]) * sin(dangle[1]) * sin(dangle[2]) + cos(dangle[0]) * cos
    (dangle[2]);

  /* '<S11>:1:99' R(2,3) = cos(qx)*sin(qy)*sin(qz) - cos(qz)*sin(qx); */
  R[7] = cos(dangle[0]) * sin(dangle[1]) * sin(dangle[2]) - cos(dangle[2]) * sin
    (dangle[0]);

  /* '<S11>:1:100' R(3,1) = -sin(qy); */
  R[2] = -sin(dangle[1]);

  /* '<S11>:1:101' R(3,2) = cos(qy)*sin(qx); */
  R[5] = cos(dangle[1]) * sin(dangle[0]);

  /* '<S11>:1:102' R(3,3) = cos(qx)*cos(qy); */
  R[8] = cos(dangle[0]) * cos(dangle[1]);

  /*  map: (dqx,dqy,dqz) -> omega */
  /* '<S11>:1:105' E(1,1) = cos(qy)*cos(qz); */
  /* '<S11>:1:106' E(1,2) = -sin(qz); */
  /* '<S11>:1:107' E(1,3) = 0; */
  /* '<S11>:1:108' E(2,1) = cos(qy)*sin(qz); */
  /* '<S11>:1:109' E(2,2) = cos(qz); */
  /* '<S11>:1:110' E(2,3) = 0; */
  /* '<S11>:1:111' E(3,1) = -sin(qy); */
  /* '<S11>:1:112' E(3,2) = 0; */
  /* '<S11>:1:113' E(3,3) = 1; */
  /*  Compute displacement vector over a sample time  */
  /* '<S11>:1:116' dPos =  R*vel; */
  for (i = 0; i < 3; i++) {
    dangle[i] = R[i + 6] * controller_DW.vel[2] + (R[i + 3] * controller_DW.vel
      [1] + R[i] * controller_DW.vel[0]);
  }

  /* '<S11>:1:117' dvel = tmp3 - cross(tmp2,vel); */
  /*  Update */
  /* '<S11>:1:120' Pos = Pos + dPos*SAMPLING_INTERVAL; */
  controller_DW.Pos[0] += dangle[0] * controller_DW.SAMPLING_INTERVAL;
  controller_DW.Pos[1] += dangle[1] * controller_DW.SAMPLING_INTERVAL;
  controller_DW.Pos[2] += dangle[2] * controller_DW.SAMPLING_INTERVAL;

  /* '<S11>:1:121' vel = vel + dvel*SAMPLING_INTERVAL; */
  rtb_aro_angle_idx_0 = controller_DW.vel[0];
  rtb_aro_angle_idx_1 = controller_DW.vel[2];
  dtmp = controller_DW.vel[1];
  u_pid_0 = controller_DW.vel[0];
  for (i = 0; i < 3; i++) {
    a_1[i] = (real_T)a[i + 6] * controller_U.accelZ + ((real_T)a[i + 3] *
      controller_U.accelY + (real_T)a[i] * controller_U.accelX);
  }

  controller_DW.vel[0] += (a_1[0] - (tmp2[1] * controller_DW.vel[2] - tmp2[2] *
    controller_DW.vel[1])) * controller_DW.SAMPLING_INTERVAL;
  controller_DW.vel[1] += (a_1[1] - (tmp2[2] * rtb_aro_angle_idx_0 - tmp2[0] *
    rtb_aro_angle_idx_1)) * controller_DW.SAMPLING_INTERVAL;
  controller_DW.vel[2] += (a_1[2] - (tmp2[0] * dtmp - tmp2[1] * u_pid_0)) *
    controller_DW.SAMPLING_INTERVAL;

  /*  px */
  /* '<S11>:1:124' x(4) = Pos(1); */
  rtb_x[3] = controller_DW.Pos[0];

  /*  py */
  /* '<S11>:1:127' x(5) = Pos(2); */
  rtb_x[4] = controller_DW.Pos[1];

  /*  pz */
  /* '<S11>:1:130' x(6) = Pos(3); */
  rtb_x[5] = controller_DW.Pos[2];

  /*  Compute (dqx,dqy,dqz) from omega */
  /* '<S11>:1:133' dq = E\tmp2; */
  /*  % roll rate */
  /*  x(7) = dq(1);  */
  /*   */
  /*  % pitch rate */
  /*  x(8) = dq(2); */
  /*   */
  /*  % yaw rate  */
  /*  x(9) = dq(3); */
  /*  wx */
  /* '<S11>:1:145' x(7) = tmp2(1); */
  rtb_x[6] = tmp2[0];

  /*  wy */
  /* '<S11>:1:148' x(8) = tmp2(2); */
  rtb_x[7] = tmp2[1];

  /*  wz  */
  /* '<S11>:1:151' x(9) = tmp2(3); */
  rtb_x[8] = tmp2[2];

  /*  vx */
  /* '<S11>:1:154' x(10) = dPos(1); */
  rtb_x[9] = dangle[0];

  /*  vy */
  /* '<S11>:1:157' x(11) = dPos(2); */
  rtb_x[10] = dangle[1];

  /*  vz */
  /* '<S11>:1:160' x(12) = dPos(3); */
  rtb_x[11] = dangle[2];

  /* End of MATLAB Function: '<S7>/func_state_estimator' */

  /* Outport: '<Root>/x' */
  memcpy(&controller_Y.x[0], &rtb_x[0], 12U * sizeof(real_T));

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

  /* DigitalClock: '<Root>/Digital Clock' */
  rtb_DigitalClock = ((controller_M->Timing.clockTick0) * 0.01);

  /* MATLAB Function: '<Root>/fcn_timer' */
  /* MATLAB Function 'fcn_timer': '<S3>:1' */
  /*  fcn_timer */
  /*  tin: time */
  /*  tou: time */
  /*  by Alireza Ramezani, 9-5-2015, Champaign, IL */
  /*  ROLLOVER_ACTIVATE (2-by-1) global variable vector  */
  /*    ROLLOVER_ACTIVATE(1): if 1 activate rollover for encoders */
  /*    ROLLOVER_ACTIVATE(2): if 1 activate rollover for imu euler angles */
  /*   */
  /*  some time related events... */
  /* '<S3>:1:16' if(tin < 1) */
  if (rtb_DigitalClock < 1.0) {
    /*  ??? */
    /* '<S3>:1:18' ROLLOVER_FLAG = [0,0,0,0].'; */
    controller_DW.ROLLOVER_FLAG[0] = 0.0;
    controller_DW.ROLLOVER_FLAG[1] = 0.0;
    controller_DW.ROLLOVER_FLAG[2] = 0.0;
    controller_DW.ROLLOVER_FLAG[3] = 0.0;

    /*  During the 1-sec time envelope keep reseting the anti rollover functions. */
    /* '<S3>:1:21' ROLLOVER_ACTIVATE = [0,0].'; */
    controller_DW.ROLLOVER_ACTIVATE[0] = 0.0;
    controller_DW.ROLLOVER_ACTIVATE[1] = 0.0;
  } else {
    /* '<S3>:1:22' else */
    /* '<S3>:1:23' ROLLOVER_ACTIVATE(2) = true; */
    controller_DW.ROLLOVER_ACTIVATE[1] = 1.0;
  }

  /* MATLAB Function: '<Root>/func_flight_controller' incorporates:
   *  Inport: '<Root>/event_based'
   *  Inport: '<Root>/xd'
   *  MATLAB Function: '<Root>/comput parameters for  event-based controller'
   *  MATLAB Function: '<Root>/fcn_timer'
   */
  /* '<S3>:1:26' tout = tin; */
  /*  from fixed-point */
  /*  xs: [12-by-1] */
  /*    1) qx: roll [rad] */
  /*    2) qy: pitch */
  /*    3) qz: yaw */
  /*    4) px: x-pos [m] */
  /*    5) py: y-pos */
  /*    6) pz: z-pos */
  /*    7) dqx:  rate of changes in roll[rad\sec] */
  /*    8) dqy: rate of changes in pitch */
  /*    9) dqz: rate of changes in yaw */
  /*    10) dpx: x-vel [m\sec] */
  /*    11) dpy: y-vel */
  /*    12) dpz: z-vel */
  /* MATLAB Function 'comput parameters for  event-based controller': '<S2>:1' */
  /*  ???????? */
  /*  xs: fixed_point (states at pioncare section)  [12-by-1] */
  /*    1) qx: roll [rad] */
  /*    2) qy: pitch */
  /*    3) qz: yaw */
  /*    4) px: x-pos [m] */
  /*    5) py: y-pos */
  /*    6) pz: z-pos */
  /*    7) dqx:  rate of changes in roll[rad\sec] */
  /*    8) dqy: rate of changes in pitch */
  /*    9) dqz: rate of changes in yaw */
  /*    10) dpx: x-vel [m\sec] */
  /*    11) dpy: y-vel */
  /*    12) dpz: z-vel */
  /*  */
  /*  pars: fixed-point (parameteres)   [16-by-1] */
  /*   freq */
  /*  */
  /*   flapping:  */
  /*            m_phi, -> fixed (mechanical restriction)    ??????? */
  /*            zeta_phi, -> fixed (mechanical restriction) ???????? */
  /*            phi_0 -> fixed (mechanical restriction)     ?????????? */
  /*   Right feathering:  */
  /*            R_m_theta,  */
  /*            R_zeta_theta,  */
  /*            R_theta_0  */
  /*   Left feathering:  */
  /*            L_m_theta, */
  /*            L_zeta_theta, */
  /*            L_theta_0  */
  /*   Right folding-unfolding:  */
  /*            R_m_eta,  */
  /*            R_zeta_eta,  */
  /*            R_eta_0 */
  /*   Left folding-unfolding:  */
  /*            L_m_eta,  */
  /*            L_zeta_eta,  */
  /*            L_eta_0 */
  /*  */
  /*  K: state feedback matrix */
  /*              % -12.6 degree (pitch-up) */
  /*              % -14.8 degree/s angular rate */
  /*               % 0.85 m/s forward velocity */
  /* '<S2>:1:59' xs = [0,... */
  /* '<S2>:1:60'     -0.2251,...             % -12.6 degree (pitch-up) */
  /* '<S2>:1:61'     0,... */
  /* '<S2>:1:62'     0,... */
  /* '<S2>:1:63'     0,... */
  /* '<S2>:1:64'     0,... */
  /* '<S2>:1:65'     0,... */
  /* '<S2>:1:66'     -0.2608,...             % -14.8 degree/s angular rate */
  /* '<S2>:1:67'     0,...   */
  /* '<S2>:1:68'     0.8556,...              % 0.85 m/s forward velocity */
  /* '<S2>:1:69'     0,... */
  /* '<S2>:1:70'     0].'; */
  /*  pars: */
  /*   freq */
  /*  */
  /*   flapping:  */
  /*            m_phi, -> fixed (mechanical restriction)    ??????? */
  /*            zeta_phi, -> fixed (mechanical restriction) ???????? */
  /*            phi_0 -> fixed (mechanical restriction)     ?????????? */
  /*   Right feathering:  */
  /*            R_m_theta,  */
  /*            R_zeta_theta,  */
  /*            R_theta_0  */
  /*   Left feathering:  */
  /*            L_m_theta, */
  /*            L_zeta_theta, */
  /*            L_theta_0  */
  /*   Right folding-unfolding:  */
  /*            R_m_eta,  */
  /*            R_zeta_eta,  */
  /*            R_eta_0 */
  /*   Left folding-unfolding:  */
  /*            L_m_eta,  */
  /*            L_zeta_eta,  */
  /*            L_eta_0 */
  /*                % 10.0860 Hz is too fast, servos cannot keep up with it. */
  /*               % 21 degree */
  /*               % 5 cm */
  /* '<S2>:1:95' pars = [10,...               % 10.0860 Hz is too fast, servos cannot keep up with it. */
  /* '<S2>:1:96'     1.0839,... */
  /* '<S2>:1:97'     0.2665,... */
  /* '<S2>:1:98'     -0.2956,...              */
  /* '<S2>:1:99'     0.15,...              % 21 degree  */
  /* '<S2>:1:100'     1.5363,... */
  /* '<S2>:1:101'     0.0022,... */
  /* '<S2>:1:102'     0.15,...              % 21 degree  */
  /* '<S2>:1:103'     1.5415,... */
  /* '<S2>:1:104'     -0.0004,... */
  /* '<S2>:1:105'     0.0500,...              % 5 cm  */
  /* '<S2>:1:106'     -1.5708,... */
  /* '<S2>:1:107'     0.1000,... */
  /* '<S2>:1:108'     0.0500,...              % 5 cm  */
  /* '<S2>:1:109'     -1.5708,... */
  /* '<S2>:1:110'     0.1000].'; */
  /*   */
  /* '<S2>:1:113' K = zeros(16,12); */
  /*  */
  /* MATLAB Function 'func_flight_controller': '<S5>:1' */
  /*  func_flight_controller */
  /*  */
  /*  x: estimated state  */
  /*    1) qx: roll [rad] */
  /*    2) qy: pitch */
  /*    3) qz: yaw */
  /*    4) px: x-pos [m] */
  /*    5) py: y-pos */
  /*    6) pz: z-pos */
  /*    7) dqx:  rate of changes in roll[rad\sec] */
  /*    8) dqy: rate of changes in pitch */
  /*    9) dqz: rate of changes in yaw */
  /*    10) dpx: x-vel [m\sec] */
  /*    11) dpy: y-vel */
  /*    12) dpz: z-vel */
  /*  */
  /*  hd: desired values [rad] */
  /*    qx: roll */
  /*    qy: pitch */
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
  /*  t: time */
  /*  */
  /*  xs: fixed_point (states at pioncare section)  [12-by-1] */
  /*    1) qx: roll [rad] */
  /*    2) qy: pitch */
  /*    3) qz: yaw */
  /*    4) px: x-pos [m] */
  /*    5) py: y-pos */
  /*    6) pz: z-pos */
  /*    7) wx:  angular velocity around x   [rad\sec] */
  /*    8) wy:  angular velocity around y */
  /*    9) wz:  angular velocity around z */
  /*    10) dpx: x-vel [m\sec] */
  /*    11) dpy: y-vel */
  /*    12) dpz: z-vel */
  /*  */
  /*  pars: fixed-point (parameteres)   [16-by-1] */
  /*   freq */
  /*  */
  /*   flapping:  */
  /*            m_phi, -> fixed (mechanical restriction)    ??????? */
  /*            zeta_phi, -> fixed (mechanical restriction) ???????? */
  /*            phi_0 -> fixed (mechanical restriction)     ?????????? */
  /*   Right feathering:  */
  /*            R_m_theta,  */
  /*            R_zeta_theta,  */
  /*            R_theta_0  */
  /*   Left feathering:  */
  /*            L_m_theta, */
  /*            L_zeta_theta, */
  /*            L_theta_0  */
  /*   Right folding-unfolding:  */
  /*            R_m_eta,  */
  /*            R_zeta_eta,  */
  /*            R_eta_0 */
  /*   Left folding-unfolding:  */
  /*            L_m_eta,  */
  /*            L_zeta_eta,  */
  /*            L_eta_0 */
  /*  */
  /*  K: state feedback gain [16-by-12] ??????????? */
  /*  */
  /*  event_based_scheme: if true uses event-based controller, otherwise PD */
  /*  controller is used. */
  /*      */
  /*  uf: flight controller output, e.g., position of actuators */
  /*    uf(1): Right forelimb RP angle */
  /*    uf(2): Left forelimb RP angle */
  /*    uf(3): Right leg DV angle */
  /*    uf(4): Left leg DV angle */
  /*  */
  /*  par: update params at each event  [16-by-1] */
  /*   freq */
  /*  */
  /*   flapping:  */
  /*            m_phi, -> fixed (mechanical restriction)    ??????? */
  /*            zeta_phi, -> fixed (mechanical restriction) ???????? */
  /*            phi_0 -> fixed (mechanical restriction)     ?????????? */
  /*   Right feathering:  */
  /*            R_m_theta,  */
  /*            R_zeta_theta,  */
  /*            R_theta_0  */
  /*   Left feathering:  */
  /*            L_m_theta, */
  /*            L_zeta_theta, */
  /*            L_theta_0  */
  /*   Right folding-unfolding:  */
  /*            R_m_eta,  */
  /*            R_zeta_eta,  */
  /*            R_eta_0 */
  /*   Left folding-unfolding:  */
  /*            L_m_eta,  */
  /*            L_zeta_eta,  */
  /*            L_eta_0 */
  /*  */
  /*  delta_par: changes in par at each event */
  /*  */
  /*  delta_x: changes in state at each event  [12-by-1] */
  /*    1) qx: roll [rad] */
  /*    2) qy: pitch */
  /*    3) qz: yaw */
  /*    4) px: x-pos [m] */
  /*    5) py: y-pos */
  /*    6) pz: z-pos */
  /*    7) dqx:  rate of changes in roll[rad\sec] */
  /*    8) dqy: rate of changes in pitch */
  /*    9) dqz: rate of changes in yaw */
  /*    10) dpx: x-vel [m\sec] */
  /*    11) dpy: y-vel */
  /*    12) dpz: z-vel */
  /*  */
  /*  By Alireza Ramezani, 9-5-2015, Champaign, IL */
  /* '<S5>:1:129' uf = zeros(4,1); */
  /*   */
  /* '<S5>:1:132' par = zeros(16,1); */
  /*   */
  /* '<S5>:1:135' delta_par = par; */
  for (i = 0; i < 16; i++) {
    rtb_par[i] = 0.0;
    rtb_delta_par[i] = 0.0;
  }

  /*   */
  /* '<S5>:1:138' delta_x = zeros(12,1); */
  memset(&rtb_delta_x[0], 0, 12U * sizeof(real_T));

  /*  */
  /* '<S5>:1:141' DEG2RAD = pi/180; */
  /*  rad\deg */
  /* '<S5>:1:143' ROLL_wing_kp = flight_ctrl_params(1); */
  /* '<S5>:1:144' ROLL_leg_kp = flight_ctrl_params(2); */
  /* '<S5>:1:145' PITCH_leg_kp = flight_ctrl_params(3); */
  /* '<S5>:1:146' ROLL_wing_kd = flight_ctrl_params(4); */
  /* '<S5>:1:147' ROLL_leg_kd = flight_ctrl_params(5); */
  /* '<S5>:1:148' PITCH_leg_kd = flight_ctrl_params(6); */
  /* '<S5>:1:149' R_qRP_offset = flight_ctrl_params(7); */
  /* '<S5>:1:150' L_qRP_offset = flight_ctrl_params(8); */
  /* '<S5>:1:151' R_qDV_offset = flight_ctrl_params(9); */
  /* '<S5>:1:152' L_qDV_offset = flight_ctrl_params(10); */
  /*  feed-forward (just an offset) */
  /* '<S5>:1:155' fwd = DEG2RAD*[R_qRP_offset,L_qRP_offset,R_qDV_offset,L_qDV_offset].'; */
  fwd[0] = 0.017453292519943295 * controller_DW.IC[6];
  fwd[1] = 0.017453292519943295 * controller_DW.IC[7];
  fwd[2] = 0.017453292519943295 * controller_DW.IC[8];
  fwd[3] = 0.017453292519943295 * controller_DW.IC[9];

  /*  positions and velocities */
  /* '<S5>:1:158' q = x(1:6); */
  /* '<S5>:1:159' dq = x(7:end); */
  /*  ???? */
  /* '<S5>:1:162' if ~event_based_scheme */
  if (!controller_U.event_based) {
    /* '<S5>:1:163' kp = [ROLL_wing_kp,0;... */
    /* '<S5>:1:164'          -ROLL_wing_kp,0;... */
    /* '<S5>:1:165'           0,PITCH_leg_kp;... */
    /* '<S5>:1:166'           0,PITCH_leg_kp]; */
    /* '<S5>:1:168' kd = [ROLL_wing_kd,0;... */
    /* '<S5>:1:169'          -ROLL_wing_kd,0;... */
    /* '<S5>:1:170'           0,PITCH_leg_kd;... */
    /* '<S5>:1:171'           0,PITCH_leg_kd]; */
    /* '<S5>:1:173' H = [1,0,0,0,0,0;... */
    /* '<S5>:1:174'          0,1,0,0,0,0]; */
    /* '<S5>:1:176' y = H*q-hd; */
    /* '<S5>:1:178' dy = H*dq; */
    /* '<S5>:1:180' uf = kp*y + kd*dy + fwd; */
    rtb_us[0] = controller_DW.IC[0];
    rtb_us[4] = 0.0;
    rtb_us[1] = -controller_DW.IC[0];
    rtb_us[5] = 0.0;
    rtb_us[2] = 0.0;
    rtb_us[6] = controller_DW.IC[2];
    rtb_us[3] = 0.0;
    rtb_us[7] = controller_DW.IC[2];
    for (i = 0; i < 2; i++) {
      rtb_aro_angle_idx_0 = 0.0;
      for (b_argout = 0; b_argout < 6; b_argout++) {
        rtb_aro_angle_idx_0 += (real_T)a_0[(b_argout << 1) + i] * rtb_x[b_argout];
      }

      a_2[i] = rtb_aro_angle_idx_0 - controller_U.xd[i];
    }

    tmp_1[0] = controller_DW.IC[3];
    tmp_1[4] = 0.0;
    tmp_1[1] = -controller_DW.IC[3];
    tmp_1[5] = 0.0;
    tmp_1[2] = 0.0;
    tmp_1[6] = controller_DW.IC[5];
    tmp_1[3] = 0.0;
    tmp_1[7] = controller_DW.IC[5];
    for (i = 0; i < 2; i++) {
      a_3[i] = 0.0;
      for (b_argout = 0; b_argout < 6; b_argout++) {
        a_3[i] += (real_T)a_0[(b_argout << 1) + i] * rtb_x[6 + b_argout];
      }
    }

    for (i = 0; i < 4; i++) {
      rtb_ramp_gain_3[i] = rtb_us[i + 4] * a_2[1] + rtb_us[i] * a_2[0];
    }

    for (i = 0; i < 4; i++) {
      rtb_ramp_gain_4[i] = tmp_1[i + 4] * a_3[1] + tmp_1[i] * a_3[0];
    }

    rtb_uf_idx_0 = (rtb_ramp_gain_3[0] + rtb_ramp_gain_4[0]) + fwd[0];
    rtb_uf_idx_1 = (rtb_ramp_gain_3[1] + rtb_ramp_gain_4[1]) + fwd[1];
    rtb_uf_idx_2 = (rtb_ramp_gain_3[2] + rtb_ramp_gain_4[2]) + fwd[2];
    rtb_uf_idx_3 = (rtb_ramp_gain_3[3] + rtb_ramp_gain_4[3]) + fwd[3];
  } else {
    /* '<S5>:1:181' else */
    /*  compute wing traj */
    /*  t : time [s] */
    /*  f : freq [hz] */
    /*  par: parameter vector, dim: (1 by 15) */
    /*      flapping: m_phi, zeta_phi, phi_0  */
    /*      Right feathering: R_m_theta, R_zeta_theta, R_theta_0  */
    /*      Left feathering: L_m_theta, L_zeta_theta, L_theta_0  */
    /*      Right folding-unfolding: R_m_eta, R_zeta_eta, R_eta_0 */
    /*      Left folding-unfolding: L_m_eta, L_zeta_eta, L_eta_0 */
    /*  phi: */
    /*  theta_R: */
    /*  theta_L: */
    /*  eta_R: */
    /*  eta_L: */
    /*  dphi: */
    /*  dtheta_R: */
    /*  dtheta_L: */
    /*  deta_R: */
    /*  deta_L: */
    /*  ddphi: */
    /*  ddtheta_R: */
    /*  ddtheta_L: */
    /*  ddeta_R: */
    /*  ddeta_L: */
    /*  compute drift from x */
    /* '<S5>:1:208' delta_x = x - xs; */
    for (i = 0; i < 12; i++) {
      rtb_delta_x[i] = rtb_x[i] - tmp[i];
    }

    /*  feedback */
    /* '<S5>:1:211' delta_par = -K*delta_x; */
    /*  update params */
    /* '<S5>:1:214' par = pars + delta_par; */
    for (i = 0; i < 16; i++) {
      rtb_delta_par[i] = 0.0;
      rtb_par[i] = tmp_0[i] + rtb_delta_par[i];
    }

    /*  compute desired traj to morphing servos... */
    /* '<S5>:1:217' f = par(1); */
    /* '<S5>:1:218' param = par(2:end); */
    /* '<S5>:1:219' [phi,theta_R,theta_L,eta_R,eta_L,dphi,dtheta_R,dtheta_L,deta_R,deta_L,ddphi,ddtheta_R,ddtheta_L,ddeta_R,ddeta_L] = func_wing_traj(t,f,param); */
    /*  plot wing kinematic traj: */
    /*  t : time [s] */
    /*  f : freq [hz] */
    /*  par: parameter vector, dim: (1 by 15) */
    /*      flapping: m_phi, zeta_phi, phi_0  */
    /*      Right feathering: R_m_theta, R_zeta_theta, R_theta_0  */
    /*      Left feathering: L_m_theta, L_zeta_theta, L_theta_0  */
    /*      Right folding-unfolding: R_m_eta, R_zeta_eta, R_eta_0 */
    /*      Left folding-unfolding: L_m_eta, L_zeta_eta, L_eta_0 */
    /*  phi: */
    /*  theta_R: */
    /*  theta_L: */
    /*  eta_R: */
    /*  eta_L: */
    /*  dphi: */
    /*  dtheta_R: */
    /*  dtheta_L: */
    /*  deta_R: */
    /*  deta_L: */
    /*  ddphi: */
    /*  ddtheta_R: */
    /*  ddtheta_L: */
    /*  ddeta_R: */
    /*  ddeta_L: */
    /*  by Alireza Ramezani, 1-8-2015, champaign-IL */
    /* 'func_wing_traj:28' m_phi = par(1); */
    /* 'func_wing_traj:29' zeta_phi = par(2); */
    /* 'func_wing_traj:30' phi_0 = par(3); */
    /* 'func_wing_traj:31' R_m_theta = par(4); */
    /* 'func_wing_traj:32' R_zeta_theta = par(5); */
    /* 'func_wing_traj:33' R_theta_0 = par(6); */
    /* 'func_wing_traj:34' L_m_theta = par(7); */
    /* 'func_wing_traj:35' L_zeta_theta = par(8); */
    /* 'func_wing_traj:36' L_theta_0 = par(9); */
    /* 'func_wing_traj:37' R_m_eta = par(10); */
    /* 'func_wing_traj:38' R_zeta_eta = par(11); */
    /* 'func_wing_traj:39' R_eta_0 = par(12); */
    /* 'func_wing_traj:40' L_m_eta = par(13); */
    /* 'func_wing_traj:41' L_zeta_eta = par(14); */
    /* 'func_wing_traj:42' L_eta_0 = par(15); */
    /*  T = 1/f; */
    /*   */
    /*  %  */
    /*  n = floor(t/T); */
    /*   */
    /*  % */
    /*  t = t-n*T; */
    /*  flapping traj */
    /* 'func_wing_traj:54' phi = phi_0 + m_phi*cos(zeta_phi + 2*pi*f*t); */
    /* 'func_wing_traj:55' dphi = -2*pi*f*m_phi*sin(zeta_phi + 2*pi*f*t); */
    /* 'func_wing_traj:56' ddphi = -4*pi^2*f^2*m_phi*cos(zeta_phi + 2*pi*f*t); */
    /*  right wing feathering traj */
    /* 'func_wing_traj:59' theta_0 = R_theta_0; */
    /* 'func_wing_traj:60' m_theta = R_m_theta; */
    /* 'func_wing_traj:61' zeta_theta = R_zeta_theta; */
    /* 'func_wing_traj:62' theta_R = theta_0 + m_theta*cos(zeta_theta + 2*pi*f*t); */
    /* 'func_wing_traj:63' dtheta_R = -2*pi*f*m_theta*sin(zeta_theta + 2*pi*f*t); */
    /* 'func_wing_traj:64' ddtheta_R = -4*pi^2*f^2*m_theta*cos(zeta_theta + 2*pi*f*t); */
    /*  left wing feathering traj */
    /* 'func_wing_traj:67' theta_0 = L_theta_0; */
    /* 'func_wing_traj:68' m_theta = L_m_theta; */
    /* 'func_wing_traj:69' zeta_theta = L_zeta_theta; */
    /* 'func_wing_traj:70' theta_L = theta_0 + m_theta*cos(zeta_theta + 2*pi*f*t); */
    /* 'func_wing_traj:71' dtheta_L = -2*pi*f*m_theta*sin(zeta_theta + 2*pi*f*t); */
    /* 'func_wing_traj:72' ddtheta_L = -4*pi^2*f^2*m_theta*cos(zeta_theta + 2*pi*f*t); */
    /*  right wing folding-unfolding traj */
    /* 'func_wing_traj:75' eta_0 = R_eta_0; */
    /* 'func_wing_traj:76' m_eta = R_m_eta; */
    /* 'func_wing_traj:77' zeta_eta = R_zeta_eta; */
    /* 'func_wing_traj:78' eta_R = eta_0 + m_eta*cos(zeta_eta + 2*pi*f*t); */
    /* 'func_wing_traj:79' deta_R = -2*pi*f*m_eta*sin(zeta_eta + 2*pi*f*t); */
    /* 'func_wing_traj:80' ddeta_R = -4*pi^2*f^2*m_eta*cos(zeta_eta + 2*pi*f*t); */
    /*  left wing folding-unfolding traj */
    /* 'func_wing_traj:83' eta_0 = L_eta_0; */
    /* 'func_wing_traj:84' m_eta = L_m_eta; */
    /* 'func_wing_traj:85' zeta_eta = L_zeta_eta; */
    /* 'func_wing_traj:86' eta_L = eta_0 + m_eta*cos(zeta_eta + 2*pi*f*t); */
    /* 'func_wing_traj:87' deta_L = -2*pi*f*m_eta*sin(zeta_eta + 2*pi*f*t); */
    /* 'func_wing_traj:88' ddeta_L = -4*pi^2*f^2*m_eta*cos(zeta_eta + 2*pi*f*t); */
    /* '<S5>:1:221' uf = [eta_R,eta_L,theta_R,theta_L].'+fwd; */
    rtb_uf_idx_0 = (cos(6.2831853071795862 * rtb_par[0] * rtb_DigitalClock +
                        rtb_par[11]) * rtb_par[10] + rtb_par[12]) + fwd[0];
    rtb_uf_idx_1 = (cos(6.2831853071795862 * rtb_par[0] * rtb_DigitalClock +
                        rtb_par[14]) * rtb_par[13] + rtb_par[15]) + fwd[1];
    rtb_uf_idx_2 = (cos(6.2831853071795862 * rtb_par[0] * rtb_DigitalClock +
                        rtb_par[5]) * rtb_par[4] + rtb_par[6]) + fwd[2];
    rtb_uf_idx_3 = (cos(6.2831853071795862 * rtb_par[0] * rtb_DigitalClock +
                        rtb_par[8]) * rtb_par[7] + rtb_par[9]) + fwd[3];
  }

  /* End of MATLAB Function: '<Root>/func_flight_controller' */

  /* Outport: '<Root>/flight_ctrl' */
  controller_Y.flight_ctrl[0] = rtb_uf_idx_0;
  controller_Y.flight_ctrl[1] = rtb_uf_idx_1;
  controller_Y.flight_ctrl[2] = rtb_uf_idx_2;
  controller_Y.flight_ctrl[3] = rtb_uf_idx_3;

  /* Gain: '<S1>/deg2rad' incorporates:
   *  Inport: '<Root>/angle'
   */
  rtb_deg2rad_idx_0 = 0.017453292519943295 * controller_U.angle[0];
  rtb_deg2rad_idx_1 = 0.017453292519943295 * controller_U.angle[1];
  rtb_deg2rad_idx_2 = 0.017453292519943295 * controller_U.angle[2];
  rtb_deg2rad_idx_3 = 0.017453292519943295 * controller_U.angle[3];

  /* MATLAB Function: '<S1>/aro' incorporates:
   *  UnitDelay: '<S1>/Unit Delay3'
   */
  /*  max angular changes during one sample time */
  /* MATLAB Function 'aro/aro': '<S8>:1' */
  /*  ?????? handles jumps in measurements??? */
  /*  */
  /*  angle: encoder measurments 4-by-1 [rad]  */
  /*    angle(1): right forelimb */
  /*    angle(2): left forelimb */
  /*    angle(3): right leg */
  /*    angle(4): left leg */
  /*  */
  /*  prev_angle: measurments from previous sample time [rad] */
  /*    prev_angle(1): right forelimb */
  /*    prev_angle(2): left forelimb */
  /*    prev_angle(3): right leg */
  /*    prev_angle(4): left leg */
  /*  */
  /*  angle2mem */
  /*  */
  /*  aro_angle: output angle    [rad] */
  /*    aro_angle(1): right forelimb */
  /*    aro_angle(2): left forelimb */
  /*    aro_angle(3): right leg */
  /*    aro_angle(4): left leg */
  /*  */
  /*  by ALireza Ramezani, 8-31-2015, Champaign, IL */
  /*  two cases: */
  /*  if 0->360 then ANTI_ROLLOVER_CORRECTION = -360 */
  /*  if 360->0 then ANTI_ROLLOVER_CORRECTION = 360 */
  /* '<S8>:1:34' DEG2RAD = pi/180; */
  /* '<S8>:1:36' aro_angle = angle; */
  rtb_aro_angle_idx_0 = rtb_deg2rad_idx_0;
  rtb_aro_angle_idx_1 = rtb_deg2rad_idx_1;
  dtmp = rtb_deg2rad_idx_2;
  rtb_aro_angle_idx_3 = rtb_deg2rad_idx_3;

  /*  comptue the difference */
  /* '<S8>:1:39' dangle = angle - prev_angle; */
  fwd[0] = rtb_deg2rad_idx_0 - controller_DW.UnitDelay3_DSTATE_b[0];
  fwd[1] = rtb_deg2rad_idx_1 - controller_DW.UnitDelay3_DSTATE_b[1];
  fwd[2] = rtb_deg2rad_idx_2 - controller_DW.UnitDelay3_DSTATE_b[2];
  fwd[3] = rtb_deg2rad_idx_3 - controller_DW.UnitDelay3_DSTATE_b[3];

  /* '<S8>:1:41' for i=1:4 */
  for (i = 0; i < 4; i++) {
    /*  ???????????? */
    /* '<S8>:1:44' bootup_jump_thresh = 200; */
    /* '<S8>:1:45' if (abs(dangle(i)) > DEG2RAD*bootup_jump_thresh) */
    if (fabs(fwd[i]) > 3.4906585039886591) {
      /* '<S8>:1:46' aro_angle = prev_angle; */
      rtb_aro_angle_idx_0 = controller_DW.UnitDelay3_DSTATE_b[0];
      rtb_aro_angle_idx_1 = controller_DW.UnitDelay3_DSTATE_b[1];
      dtmp = controller_DW.UnitDelay3_DSTATE_b[2];
      rtb_aro_angle_idx_3 = controller_DW.UnitDelay3_DSTATE_b[3];
    }

    /*  ??????????????? */
    /* '<S8>:1:51' if (abs(dangle(i)) > DEG2RAD*MAX_ANGLE_DIFFERENCE) */
    if (fabs(fwd[i]) > 0.017453292519943295 * controller_DW.MAX_ANGLE_DIFFERENCE)
    {
      /* '<S8>:1:52' aro_angle = prev_angle; */
      rtb_aro_angle_idx_0 = controller_DW.UnitDelay3_DSTATE_b[0];
      rtb_aro_angle_idx_1 = controller_DW.UnitDelay3_DSTATE_b[1];
      dtmp = controller_DW.UnitDelay3_DSTATE_b[2];
      rtb_aro_angle_idx_3 = controller_DW.UnitDelay3_DSTATE_b[3];
    }
  }

  /* MATLAB Function: '<S1>/normalize angle' */
  /* '<S8>:1:57' angle2mem = angle; */
  /* MATLAB Function 'aro/normalize angle': '<S9>:1' */
  /*   */
  /*  */
  /*  ???? */
  /*  in:  */
  /*    angle(1): right forelimb */
  /*    angle(2): left forelimb */
  /*    angle(3): right leg */
  /*    angle(4): left leg */
  /*  out:  */
  /*    angle(1): right forelimb */
  /*    angle(2): left forelimb */
  /*    angle(3): right leg */
  /*    angle(4): left leg */
  /*  */
  /*  ?????? */
  /* '<S9>:1:28' DEG2RAD = pi/180; */
  /* '<S9>:1:30' out = zeros(4,1); */
  /*  if postive rotation of the links is measured negative by the encoders, */
  /*  then              maximum angle - minimum angle <0 */
  /*  this is used to correct for this sign issue */
  /*  NOTE: maximum angle for arm refers to fully stretched and for legs */
  /*  referes to fully up. */
  /* '<S9>:1:38' servo_range = [MAX_RP_ANGLE_RIGHT,MAX_RP_ANGLE_LEFT,MAX_DV_ANGLE_RIGHT,MAX_DV_ANGLE_LEFT].'-[MIN_RP_ANGLE_RIGHT,MIN_RP_ANGLE_LEFT,MIN_DV_ANGLE_RIGHT,MIN_DV_ANGLE_LEFT].'; */
  /* '<S9>:1:39' servo_range = DEG2RAD*servo_range; */
  /* '<S9>:1:41' lb = DEG2RAD*[MIN_RP_ANGLE_RIGHT,MIN_RP_ANGLE_LEFT,MIN_DV_ANGLE_RIGHT,MIN_DV_ANGLE_LEFT].'; */
  /* '<S9>:1:43' out =  in-lb; */
  rtb_out_idx_0 = rtb_aro_angle_idx_0 - 0.017453292519943295 *
    controller_DW.MIN_RP_ANGLE_RIGHT;
  rtb_out_idx_1 = rtb_aro_angle_idx_1 - 0.017453292519943295 *
    controller_DW.MIN_RP_ANGLE_LEFT;
  rtb_out_idx_2 = dtmp - 0.017453292519943295 * controller_DW.MIN_DV_ANGLE_RIGHT;
  rtb_aro_angle_idx_0 = rtb_aro_angle_idx_3 - 0.017453292519943295 *
    controller_DW.MIN_DV_ANGLE_LEFT;

  /*  if postive rotation of the link is measured negative, this corrects that. */
  /* '<S9>:1:47' for i=1:4 */
  rtb_out = rtb_out_idx_0;

  /* '<S9>:1:48' if servo_range(i) < 0 */
  if ((controller_DW.MAX_RP_ANGLE_RIGHT - controller_DW.MIN_RP_ANGLE_RIGHT) *
      0.017453292519943295 < 0.0) {
    /* '<S9>:1:49' out(i) = -out(i); */
    rtb_out = -rtb_out_idx_0;
  }

  rtb_out_idx_0 = rtb_out;
  rtb_out = rtb_out_idx_1;

  /* '<S9>:1:48' if servo_range(i) < 0 */
  if ((controller_DW.MAX_RP_ANGLE_LEFT - controller_DW.MIN_RP_ANGLE_LEFT) *
      0.017453292519943295 < 0.0) {
    /* '<S9>:1:49' out(i) = -out(i); */
    rtb_out = -rtb_out_idx_1;
  }

  rtb_out_idx_1 = rtb_out;
  rtb_out = rtb_out_idx_2;

  /* '<S9>:1:48' if servo_range(i) < 0 */
  if ((controller_DW.MAX_DV_ANGLE_RIGHT - controller_DW.MIN_DV_ANGLE_RIGHT) *
      0.017453292519943295 < 0.0) {
    /* '<S9>:1:49' out(i) = -out(i); */
    rtb_out = -rtb_out_idx_2;
  }

  rtb_out_idx_2 = rtb_out;
  rtb_out = rtb_aro_angle_idx_0;

  /* '<S9>:1:48' if servo_range(i) < 0 */
  if ((controller_DW.MAX_DV_ANGLE_LEFT - controller_DW.MIN_DV_ANGLE_LEFT) *
      0.017453292519943295 < 0.0) {
    /* '<S9>:1:49' out(i) = -out(i); */
    rtb_out = -rtb_aro_angle_idx_0;
  }

  /* End of MATLAB Function: '<S1>/normalize angle' */

  /* Gain: '<Root>/Gain' incorporates:
   *  MATLAB Function: '<Root>/fcn_timer'
   */
  dtmp = 300.0 * rtb_DigitalClock;

  /* MATLAB Function: '<Root>/gain-ramp' incorporates:
   *  Inport: '<Root>/pid_gian'
   */
  /* MATLAB Function 'gain-ramp': '<S6>:1' */
  /*  Ramps up the control gains to avoid jumps at startups. */
  /*  u: ramp signal, scalar */
  /*  */
  /*  gain: control gain vector 4-by-1 vec */
  /*    gain(1): right-left forelimb Kp */
  /*    gain(2): right-left leg Kp */
  /*    gain(3): right-left forelimb Kd */
  /*    gain(4): right-left leg Kd */
  /*    gain(5): right-left forelimb Ki */
  /*    gain(6): right-left leg Ki */
  /*   */
  /*  ramp_gain: gain */
  /*  */
  /*  by Alireza Ramezani, 5-8-2016 */
  /* '<S6>:1:18' ramp_gain = zeros(6,1); */
  /* '<S6>:1:20' for i=1:6 */
  for (i = 0; i < 6; i++) {
    /* '<S6>:1:21' ramp_gain(i) = u; */
    u_pid_0 = dtmp;

    /* '<S6>:1:23' if ramp_gain(i) > gain(i) */
    if (dtmp > controller_U.pid_gian[i]) {
      /* '<S6>:1:24' ramp_gain(i) = gain(i); */
      u_pid_0 = controller_U.pid_gian[i];
    }

    rtb_ramp_gain[i] = u_pid_0;
  }

  /* End of MATLAB Function: '<Root>/gain-ramp' */

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
   *  UnitDelay: '<Root>/Unit Delay2'
   */
  /* MATLAB Function 'func_actuator_controller': '<S4>:1' */
  /*  func_actuator_controller: uses a PID scheme to position wing and tail */
  /*  actuators */
  /*  */
  /*  uf: flight control inputs, 4-by-1 vector */
  /*    uf(1): right forelimb des. pos. */
  /*    uf(2): left forelimb des. pos. */
  /*    uf(3): right leg des. pos. */
  /*    uf(4): left leg des. pos. */
  /*  */
  /*  angle: 4-by-1 */
  /*    angle(1): right forelimb */
  /*    angle(2): left forelimb */
  /*    angle(3): right leg */
  /*    angle(4): left leg */
  /*  */
  /*  prev_err: memory for err vector 4-by-1 vec */
  /*    prev_err(1): right forelimb */
  /*    prev_err(2): left forelimb */
  /*    prev_err(3): right leg */
  /*    prev_err(4): left leg */
  /*  */
  /*  gain: control gain vector 6-by-1 vec */
  /*    gain(1): right-left forelimb Kp */
  /*    gain(2): right-left leg Kp */
  /*    gain(3): right-left forelimb Kd */
  /*    gain(4): right-left leg Kd */
  /*    gain(5): right-left forelimb Ki */
  /*    gain(6): right-left leg Ki */
  /*  */
  /*  ctrl_param: params */
  /*    ctrl_param(1): PID_SATURATION_THRESHOLD (control sat, used in pid func) */
  /*    ctrl_param(2): MAX_ANGLE_DIFFERENCE (deg/sample, used in anti-roll-over func) */
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
  /*  */
  /*  us: micro actuator control inputs, 4-by-1 vector (interprets control inputs for drv motor controllers) */
  /*    us(1): right forelimb */
  /*    us(2): left forelimb */
  /*    us(3): right leg */
  /*    us(4): left leg */
  /*  */
  /*  err2mem: to keep error in memory for one sample time */
  /*  */
  /*  derr: time derivative of error */
  /*   */
  /*  interr: integration of error */
  /*  */
  /*  debug: debugging channel, i.e., used for debugging purpose. */
  /*  */
  /*  by ALireza Ramezani, 8-31-2015, Champaign, IL */
  /*  */
  /* '<S4>:1:64' us = zeros(8,1); */
  /* '<S4>:1:65' u_pid = zeros(4,1); */
  /* '<S4>:1:66' angle2mem = zeros(4,1); */
  /* '<S4>:1:67' err2mem = zeros(4,1); */
  /* '<S4>:1:68' debug = zeros(4,1); */
  /*  global vars */
  /* '<S4>:1:88' PID_SATURATION_THRESHOLD = ctrl_param(1); */
  controller_DW.PID_SATURATION_THRESHOLD = controller_DW.IC2[0];

  /* '<S4>:1:89' MAX_ANGLE_DIFFERENCE = ctrl_param(2); */
  controller_DW.MAX_ANGLE_DIFFERENCE = controller_DW.IC2[1];

  /* '<S4>:1:90' ANTI_ROLLOVER_CORRECTION = ctrl_param(3); */
  controller_DW.ANTI_ROLLOVER_CORRECTION = controller_DW.IC2[2];

  /* '<S4>:1:91' MAX_RP_ANGLE_RIGHT = ctrl_param(4); */
  controller_DW.MAX_RP_ANGLE_RIGHT = controller_DW.IC2[3];

  /* '<S4>:1:92' MAX_DV_ANGLE_RIGHT = ctrl_param(5); */
  controller_DW.MAX_DV_ANGLE_RIGHT = controller_DW.IC2[4];

  /* '<S4>:1:93' MIN_RP_ANGLE_RIGHT = ctrl_param(6); */
  controller_DW.MIN_RP_ANGLE_RIGHT = controller_DW.IC2[5];

  /* '<S4>:1:94' MIN_DV_ANGLE_RIGHT = ctrl_param(7); */
  controller_DW.MIN_DV_ANGLE_RIGHT = controller_DW.IC2[6];

  /* '<S4>:1:95' MAX_RP_ANGLE_LEFT = ctrl_param(8); */
  controller_DW.MAX_RP_ANGLE_LEFT = controller_DW.IC2[7];

  /* '<S4>:1:96' MAX_DV_ANGLE_LEFT = ctrl_param(9); */
  controller_DW.MAX_DV_ANGLE_LEFT = controller_DW.IC2[8];

  /* '<S4>:1:97' MIN_RP_ANGLE_LEFT = ctrl_param(10); */
  controller_DW.MIN_RP_ANGLE_LEFT = controller_DW.IC2[9];

  /* '<S4>:1:98' MIN_DV_ANGLE_LEFT = ctrl_param(11); */
  controller_DW.MIN_DV_ANGLE_LEFT = controller_DW.IC2[10];

  /* '<S4>:1:99' SAMPLING_INTERVAL = ctrl_param(12); */
  controller_DW.SAMPLING_INTERVAL = controller_DW.IC2[11];

  /* '<S4>:1:100' PID_TRACKING_PRECISION_THRESHOLD = ctrl_param(13); */
  controller_DW.PID_TRACKING_PRECISION_THRESHOL = controller_DW.IC2[12];

  /* '<S4>:1:101' ANTI_WINDUP_THRESHOLD = ctrl_param(14); */
  controller_DW.ANTI_WINDUP_THRESHOLD = controller_DW.IC2[13];

  /*  PID scheme */
  /* '<S4>:1:105' [u_pid,err,derr,interr] = func_pid_controller(angle,uf,prev_err,pid_gain); */
  /*  func_pid_controller: pd position controller */
  /*  */
  /*  angle: 4-by-1 vec */
  /*    angle(1): right forelimb */
  /*    angle(2): left forelimb */
  /*    angle(3): right leg */
  /*    angle(4): left leg */
  /*  */
  /*  des_angle: desired position 4-by-1 vec */
  /*    des_angle(1): right forelimb */
  /*    des_angle(2): left forelimb */
  /*    des_angle(3): right leg */
  /*    des_angle(4): left leg */
  /*  */
  /*  prev_err: memory for err vector 4-by-1 vec */
  /*    prev_err(1): right forelimb */
  /*    prev_err(2): left forelimb */
  /*    prev_err(3): right leg */
  /*    prev_err(4): left leg */
  /*  */
  /*  gain: control gain vector 4-by-1 vec */
  /*    gain(1): right-left forelimb Kp */
  /*    gain(2): right-left leg Kp */
  /*    gain(3): right-left forelimb Kd */
  /*    gain(4): right-left leg Kd */
  /*    gain(5): right-left forelimb Ki */
  /*    gain(6): right-left leg Ki */
  /*  */
  /*  u: control action, 4-by-1 vec */
  /*    u(1): right forelimb */
  /*    u(2): left forelimb */
  /*    u(3): right leg */
  /*    u(4): left leg */
  /*  */
  /*  err: to save err vec in memory */
  /*  */
  /*  derr: derivative of error */
  /*  */
  /*  interr: integration of error */
  /*  */
  /*  by ALireza Ramezani, 8-31-2015, Champaign, IL */
  /* 'func_pid_controller:44' u = zeros(4,1); */
  /* 'func_pid_controller:45' err = zeros(4,1); */
  /*  err */
  /* 'func_pid_controller:46' derr = zeros(4,1); */
  /*  derivative of err */
  /* 'func_pid_controller:47' interr = zeros(4,1); */
  /*  integration of err */
  /* 'func_pid_controller:48' serr = zeros(4,1); */
  /*  integration of err */
  /*  global vars */
  /* 'func_pid_controller:64' DEG2RAD = pi/180; */
  /*  rad\deg */
  /* 'func_pid_controller:65' max_angle = DEG2RAD*[MAX_RP_ANGLE_RIGHT,MAX_RP_ANGLE_LEFT,MAX_DV_ANGLE_RIGHT,MAX_DV_ANGLE_LEFT].'; */
  /* 'func_pid_controller:66' min_angle = DEG2RAD*[MIN_RP_ANGLE_RIGHT,MIN_RP_ANGLE_LEFT,MIN_DV_ANGLE_RIGHT,MIN_DV_ANGLE_LEFT].'; */
  /* 'func_pid_controller:68' Kp = [gain(1),0,0,0;... */
  /* 'func_pid_controller:69'       0, gain(1),0,0;... */
  /* 'func_pid_controller:70'       0, 0,gain(2),0;... */
  /* 'func_pid_controller:71'       0, 0, 0,gain(2)]; */
  /* 'func_pid_controller:73' Kd = [gain(3),0,0,0;... */
  /* 'func_pid_controller:74'       0, gain(3),0,0;... */
  /* 'func_pid_controller:75'       0, 0,gain(4),0;... */
  /* 'func_pid_controller:76'       0, 0, 0,gain(4)]; */
  /* 'func_pid_controller:78' Ki = [gain(5),0,0,0;... */
  /* 'func_pid_controller:79'       0, gain(5),0,0;... */
  /* 'func_pid_controller:80'       0, 0,gain(6),0;... */
  /* 'func_pid_controller:81'       0, 0, 0,gain(6)]; */
  /*  turn-off the motors immediately when hit the limits */
  /* 'func_pid_controller:84' delta_max_angle = max_angle-min_angle; */
  fwd[0] = 0.017453292519943295 * controller_DW.MAX_RP_ANGLE_RIGHT -
    0.017453292519943295 * controller_DW.MIN_RP_ANGLE_RIGHT;
  fwd[1] = 0.017453292519943295 * controller_DW.MAX_RP_ANGLE_LEFT -
    0.017453292519943295 * controller_DW.MIN_RP_ANGLE_LEFT;
  fwd[2] = 0.017453292519943295 * controller_DW.MAX_DV_ANGLE_RIGHT -
    0.017453292519943295 * controller_DW.MIN_DV_ANGLE_RIGHT;
  fwd[3] = 0.017453292519943295 * controller_DW.MAX_DV_ANGLE_LEFT -
    0.017453292519943295 * controller_DW.MIN_DV_ANGLE_LEFT;

  /*  compute error and derr */
  /* 'func_pid_controller:87' err = angle - des_angle; */
  err_idx_0 = rtb_out_idx_0 - rtb_uf_idx_0;
  err_idx_1 = rtb_out_idx_1 - rtb_uf_idx_1;
  err_idx_2 = rtb_out_idx_2 - rtb_uf_idx_2;
  err_idx_3 = rtb_out - rtb_uf_idx_3;

  /* 'func_pid_controller:88' derr = (err-prev_err)/SAMPLING_INTERVAL; */
  rtb_aro_angle_idx_0 = (err_idx_0 - controller_DW.UnitDelay2_DSTATE[0]) /
    controller_DW.SAMPLING_INTERVAL;
  rtb_aro_angle_idx_1 = (err_idx_1 - controller_DW.UnitDelay2_DSTATE[1]) /
    controller_DW.SAMPLING_INTERVAL;
  dtmp = (err_idx_2 - controller_DW.UnitDelay2_DSTATE[2]) /
    controller_DW.SAMPLING_INTERVAL;
  rtb_aro_angle_idx_3 = (err_idx_3 - controller_DW.UnitDelay2_DSTATE[3]) /
    controller_DW.SAMPLING_INTERVAL;

  /* 'func_pid_controller:89' ERR_INTEGRALE = ERR_INTEGRALE + err*SAMPLING_INTERVAL; */
  controller_DW.ERR_INTEGRALE[0] += err_idx_0 * controller_DW.SAMPLING_INTERVAL;
  controller_DW.ERR_INTEGRALE[1] += err_idx_1 * controller_DW.SAMPLING_INTERVAL;
  controller_DW.ERR_INTEGRALE[2] += err_idx_2 * controller_DW.SAMPLING_INTERVAL;
  controller_DW.ERR_INTEGRALE[3] += err_idx_3 * controller_DW.SAMPLING_INTERVAL;

  /* 'func_pid_controller:91' for i=1:4 */
  /* 'func_pid_controller:92' if ERR_INTEGRALE(i)>ANTI_WINDUP_THRESHOLD */
  if (controller_DW.ERR_INTEGRALE[0] > controller_DW.ANTI_WINDUP_THRESHOLD) {
    /* 'func_pid_controller:93' ERR_INTEGRALE(i) = ANTI_WINDUP_THRESHOLD; */
    controller_DW.ERR_INTEGRALE[0] = controller_DW.ANTI_WINDUP_THRESHOLD;
  }

  /* 'func_pid_controller:95' if ERR_INTEGRALE(i)<-ANTI_WINDUP_THRESHOLD */
  if (controller_DW.ERR_INTEGRALE[0] < -controller_DW.ANTI_WINDUP_THRESHOLD) {
    /* 'func_pid_controller:96' ERR_INTEGRALE(i) = -ANTI_WINDUP_THRESHOLD; */
    controller_DW.ERR_INTEGRALE[0] = -controller_DW.ANTI_WINDUP_THRESHOLD;
  }

  /* 'func_pid_controller:92' if ERR_INTEGRALE(i)>ANTI_WINDUP_THRESHOLD */
  if (controller_DW.ERR_INTEGRALE[1] > controller_DW.ANTI_WINDUP_THRESHOLD) {
    /* 'func_pid_controller:93' ERR_INTEGRALE(i) = ANTI_WINDUP_THRESHOLD; */
    controller_DW.ERR_INTEGRALE[1] = controller_DW.ANTI_WINDUP_THRESHOLD;
  }

  /* 'func_pid_controller:95' if ERR_INTEGRALE(i)<-ANTI_WINDUP_THRESHOLD */
  if (controller_DW.ERR_INTEGRALE[1] < -controller_DW.ANTI_WINDUP_THRESHOLD) {
    /* 'func_pid_controller:96' ERR_INTEGRALE(i) = -ANTI_WINDUP_THRESHOLD; */
    controller_DW.ERR_INTEGRALE[1] = -controller_DW.ANTI_WINDUP_THRESHOLD;
  }

  /* 'func_pid_controller:92' if ERR_INTEGRALE(i)>ANTI_WINDUP_THRESHOLD */
  if (controller_DW.ERR_INTEGRALE[2] > controller_DW.ANTI_WINDUP_THRESHOLD) {
    /* 'func_pid_controller:93' ERR_INTEGRALE(i) = ANTI_WINDUP_THRESHOLD; */
    controller_DW.ERR_INTEGRALE[2] = controller_DW.ANTI_WINDUP_THRESHOLD;
  }

  /* 'func_pid_controller:95' if ERR_INTEGRALE(i)<-ANTI_WINDUP_THRESHOLD */
  if (controller_DW.ERR_INTEGRALE[2] < -controller_DW.ANTI_WINDUP_THRESHOLD) {
    /* 'func_pid_controller:96' ERR_INTEGRALE(i) = -ANTI_WINDUP_THRESHOLD; */
    controller_DW.ERR_INTEGRALE[2] = -controller_DW.ANTI_WINDUP_THRESHOLD;
  }

  /* 'func_pid_controller:92' if ERR_INTEGRALE(i)>ANTI_WINDUP_THRESHOLD */
  if (controller_DW.ERR_INTEGRALE[3] > controller_DW.ANTI_WINDUP_THRESHOLD) {
    /* 'func_pid_controller:93' ERR_INTEGRALE(i) = ANTI_WINDUP_THRESHOLD; */
    controller_DW.ERR_INTEGRALE[3] = controller_DW.ANTI_WINDUP_THRESHOLD;
  }

  /* 'func_pid_controller:95' if ERR_INTEGRALE(i)<-ANTI_WINDUP_THRESHOLD */
  if (controller_DW.ERR_INTEGRALE[3] < -controller_DW.ANTI_WINDUP_THRESHOLD) {
    /* 'func_pid_controller:96' ERR_INTEGRALE(i) = -ANTI_WINDUP_THRESHOLD; */
    controller_DW.ERR_INTEGRALE[3] = -controller_DW.ANTI_WINDUP_THRESHOLD;
  }

  /*  computer u */
  /* 'func_pid_controller:100' interr = ERR_INTEGRALE; */
  /* 'func_pid_controller:101' u = -Kp*err -Kd*derr -Ki*interr; */
  rtb_ramp_gain_0[0] = rtb_ramp_gain[0];
  rtb_ramp_gain_0[4] = 0.0;
  rtb_ramp_gain_0[8] = 0.0;
  rtb_ramp_gain_0[12] = 0.0;
  rtb_ramp_gain_0[1] = 0.0;
  rtb_ramp_gain_0[5] = rtb_ramp_gain[0];
  rtb_ramp_gain_0[9] = 0.0;
  rtb_ramp_gain_0[13] = 0.0;
  rtb_ramp_gain_0[2] = 0.0;
  rtb_ramp_gain_0[6] = 0.0;
  rtb_ramp_gain_0[10] = rtb_ramp_gain[1];
  rtb_ramp_gain_0[14] = 0.0;
  rtb_ramp_gain_0[3] = 0.0;
  rtb_ramp_gain_0[7] = 0.0;
  rtb_ramp_gain_0[11] = 0.0;
  rtb_ramp_gain_0[15] = rtb_ramp_gain[1];
  for (i = 0; i < 4; i++) {
    rtb_ramp_gain_1[i << 2] = -rtb_ramp_gain_0[i << 2];
    rtb_ramp_gain_1[1 + (i << 2)] = -rtb_ramp_gain_0[(i << 2) + 1];
    rtb_ramp_gain_1[2 + (i << 2)] = -rtb_ramp_gain_0[(i << 2) + 2];
    rtb_ramp_gain_1[3 + (i << 2)] = -rtb_ramp_gain_0[(i << 2) + 3];
  }

  rtb_ramp_gain_2[0] = rtb_ramp_gain[2];
  rtb_ramp_gain_2[4] = 0.0;
  rtb_ramp_gain_2[8] = 0.0;
  rtb_ramp_gain_2[12] = 0.0;
  rtb_ramp_gain_2[1] = 0.0;
  rtb_ramp_gain_2[5] = rtb_ramp_gain[2];
  rtb_ramp_gain_2[9] = 0.0;
  rtb_ramp_gain_2[13] = 0.0;
  rtb_ramp_gain_2[2] = 0.0;
  rtb_ramp_gain_2[6] = 0.0;
  rtb_ramp_gain_2[10] = rtb_ramp_gain[3];
  rtb_ramp_gain_2[14] = 0.0;
  rtb_ramp_gain_2[3] = 0.0;
  rtb_ramp_gain_2[7] = 0.0;
  rtb_ramp_gain_2[11] = 0.0;
  rtb_ramp_gain_2[15] = rtb_ramp_gain[3];
  for (i = 0; i < 4; i++) {
    u_pid_0 = rtb_ramp_gain_1[i + 12] * err_idx_3 + (rtb_ramp_gain_1[i + 8] *
      err_idx_2 + (rtb_ramp_gain_1[i + 4] * err_idx_1 + rtb_ramp_gain_1[i] *
                   err_idx_0));
    rtb_ramp_gain_3[i] = u_pid_0;
  }

  for (i = 0; i < 4; i++) {
    u_pid_0 = rtb_ramp_gain_2[i + 12] * rtb_aro_angle_idx_3 + (rtb_ramp_gain_2[i
      + 8] * dtmp + (rtb_ramp_gain_2[i + 4] * rtb_aro_angle_idx_1 +
                     rtb_ramp_gain_2[i] * rtb_aro_angle_idx_0));
    rtb_ramp_gain_4[i] = u_pid_0;
  }

  rtb_ramp_gain_5[0] = rtb_ramp_gain[4];
  rtb_ramp_gain_5[4] = 0.0;
  rtb_ramp_gain_5[8] = 0.0;
  rtb_ramp_gain_5[12] = 0.0;
  rtb_ramp_gain_5[1] = 0.0;
  rtb_ramp_gain_5[5] = rtb_ramp_gain[4];
  rtb_ramp_gain_5[9] = 0.0;
  rtb_ramp_gain_5[13] = 0.0;
  rtb_ramp_gain_5[2] = 0.0;
  rtb_ramp_gain_5[6] = 0.0;
  rtb_ramp_gain_5[10] = rtb_ramp_gain[5];
  rtb_ramp_gain_5[14] = 0.0;
  rtb_ramp_gain_5[3] = 0.0;
  rtb_ramp_gain_5[7] = 0.0;
  rtb_ramp_gain_5[11] = 0.0;
  rtb_ramp_gain_5[15] = rtb_ramp_gain[5];
  for (i = 0; i < 4; i++) {
    u_pid[i] = (rtb_ramp_gain_3[i] - rtb_ramp_gain_4[i]) - (((rtb_ramp_gain_5[i
      + 4] * controller_DW.ERR_INTEGRALE[1] + rtb_ramp_gain_5[i] *
      controller_DW.ERR_INTEGRALE[0]) + rtb_ramp_gain_5[i + 8] *
      controller_DW.ERR_INTEGRALE[2]) + rtb_ramp_gain_5[i + 12] *
      controller_DW.ERR_INTEGRALE[3]);
  }

  /* 'func_pid_controller:103' for i=1:4 */
  u_pid_0 = u_pid[0];

  /* 'func_pid_controller:104' if delta_max_angle(i)<0 */
  if (fwd[0] < 0.0) {
    /* 'func_pid_controller:105' u(i) = -u(i); */
    u_pid_0 = -u_pid[0];
  }

  u_pid[0] = u_pid_0;
  u_pid_0 = u_pid[1];

  /* 'func_pid_controller:104' if delta_max_angle(i)<0 */
  if (fwd[1] < 0.0) {
    /* 'func_pid_controller:105' u(i) = -u(i); */
    u_pid_0 = -u_pid[1];
  }

  u_pid[1] = u_pid_0;
  u_pid_0 = u_pid[2];

  /* 'func_pid_controller:104' if delta_max_angle(i)<0 */
  if (fwd[2] < 0.0) {
    /* 'func_pid_controller:105' u(i) = -u(i); */
    u_pid_0 = -u_pid[2];
  }

  u_pid[2] = u_pid_0;
  u_pid_0 = u_pid[3];

  /* 'func_pid_controller:104' if delta_max_angle(i)<0 */
  if (fwd[3] < 0.0) {
    /* 'func_pid_controller:105' u(i) = -u(i); */
    u_pid_0 = -u_pid[3];
  }

  u_pid[3] = u_pid_0;

  /* 'func_pid_controller:110' for i=1:4 */
  /*  resume operation if commands are legitimate */
  /* 'func_pid_controller:112' if((angle(i) > abs(delta_max_angle(i))) || (angle(i) <0 )) */
  if (((rtb_out_idx_0 > fabs(fwd[0])) || (rtb_out_idx_0 < 0.0)) &&
      ((rtb_uf_idx_0 > fabs(fwd[0])) || (rtb_uf_idx_0 < 0.0))) {
    /* 'func_pid_controller:113' if ((des_angle(i) > abs(delta_max_angle(i))) || (des_angle(i) < 0)) */
    /* 'func_pid_controller:114' u(i) = 0; */
    u_pid[0] = 0.0;
  }

  /*  resume operation if commands are legitimate */
  /* 'func_pid_controller:112' if((angle(i) > abs(delta_max_angle(i))) || (angle(i) <0 )) */
  if (((rtb_out_idx_1 > fabs(fwd[1])) || (rtb_out_idx_1 < 0.0)) &&
      ((rtb_uf_idx_1 > fabs(fwd[1])) || (rtb_uf_idx_1 < 0.0))) {
    /* 'func_pid_controller:113' if ((des_angle(i) > abs(delta_max_angle(i))) || (des_angle(i) < 0)) */
    /* 'func_pid_controller:114' u(i) = 0; */
    u_pid[1] = 0.0;
  }

  /*  resume operation if commands are legitimate */
  /* 'func_pid_controller:112' if((angle(i) > abs(delta_max_angle(i))) || (angle(i) <0 )) */
  if (((rtb_out_idx_2 > fabs(fwd[2])) || (rtb_out_idx_2 < 0.0)) &&
      ((rtb_uf_idx_2 > fabs(fwd[2])) || (rtb_uf_idx_2 < 0.0))) {
    /* 'func_pid_controller:113' if ((des_angle(i) > abs(delta_max_angle(i))) || (des_angle(i) < 0)) */
    /* 'func_pid_controller:114' u(i) = 0; */
    u_pid[2] = 0.0;
  }

  /*  resume operation if commands are legitimate */
  /* 'func_pid_controller:112' if((angle(i) > abs(delta_max_angle(i))) || (angle(i) <0 )) */
  if (((rtb_out > fabs(fwd[3])) || (rtb_out < 0.0)) && ((rtb_uf_idx_3 > fabs
        (fwd[3])) || (rtb_uf_idx_3 < 0.0))) {
    /* 'func_pid_controller:113' if ((des_angle(i) > abs(delta_max_angle(i))) || (des_angle(i) < 0)) */
    /* 'func_pid_controller:114' u(i) = 0; */
    u_pid[3] = 0.0;
  }

  /* Outport: '<Root>/servo_derr' incorporates:
   *  MATLAB Function: '<Root>/func_actuator_controller'
   */
  controller_Y.servo_derr[0] = rtb_aro_angle_idx_0;
  controller_Y.servo_derr[1] = rtb_aro_angle_idx_1;
  controller_Y.servo_derr[2] = dtmp;
  controller_Y.servo_derr[3] = rtb_aro_angle_idx_3;

  /* Outport: '<Root>/servo_interr' incorporates:
   *  MATLAB Function: '<Root>/func_actuator_controller'
   */
  controller_Y.servo_interr[0] = controller_DW.ERR_INTEGRALE[0];
  controller_Y.servo_interr[1] = controller_DW.ERR_INTEGRALE[1];
  controller_Y.servo_interr[2] = controller_DW.ERR_INTEGRALE[2];
  controller_Y.servo_interr[3] = controller_DW.ERR_INTEGRALE[3];

  /* MATLAB Function: '<Root>/func_actuator_controller' */
  /*  map pid efforts to drv  */
  /* '<S4>:1:108' us = func_map_pid_to_servo(u_pid,err); */
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
  i = 0;

  /* 'func_heaviside:7' if argin>=0 */
  if (u_pid[0] >= 0.0) {
    /* 'func_heaviside:8' argout = 1; */
    i = 1;
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
  u_pid_0 = u_pid[0];

  /* 'func_map_pid_to_servo:40' if u(i)> PID_SATURATION_THRESHOLD */
  if (u_pid[0] > controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_servo:41' u(i) = PID_SATURATION_THRESHOLD; */
    u_pid_0 = controller_DW.PID_SATURATION_THRESHOLD;
  }

  /* 'func_map_pid_to_servo:44' if u(i)< -PID_SATURATION_THRESHOLD */
  if (u_pid_0 < -controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_servo:45' u(i) = -PID_SATURATION_THRESHOLD; */
    u_pid_0 = -controller_DW.PID_SATURATION_THRESHOLD;
  }

  u_pid[0] = u_pid_0;
  u_pid_0 = u_pid[1];

  /* 'func_map_pid_to_servo:40' if u(i)> PID_SATURATION_THRESHOLD */
  if (u_pid[1] > controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_servo:41' u(i) = PID_SATURATION_THRESHOLD; */
    u_pid_0 = controller_DW.PID_SATURATION_THRESHOLD;
  }

  /* 'func_map_pid_to_servo:44' if u(i)< -PID_SATURATION_THRESHOLD */
  if (u_pid_0 < -controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_servo:45' u(i) = -PID_SATURATION_THRESHOLD; */
    u_pid_0 = -controller_DW.PID_SATURATION_THRESHOLD;
  }

  u_pid[1] = u_pid_0;
  u_pid_0 = u_pid[2];

  /* 'func_map_pid_to_servo:40' if u(i)> PID_SATURATION_THRESHOLD */
  if (u_pid[2] > controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_servo:41' u(i) = PID_SATURATION_THRESHOLD; */
    u_pid_0 = controller_DW.PID_SATURATION_THRESHOLD;
  }

  /* 'func_map_pid_to_servo:44' if u(i)< -PID_SATURATION_THRESHOLD */
  if (u_pid_0 < -controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_servo:45' u(i) = -PID_SATURATION_THRESHOLD; */
    u_pid_0 = -controller_DW.PID_SATURATION_THRESHOLD;
  }

  u_pid[2] = u_pid_0;
  u_pid_0 = u_pid[3];

  /* 'func_map_pid_to_servo:40' if u(i)> PID_SATURATION_THRESHOLD */
  if (u_pid[3] > controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_servo:41' u(i) = PID_SATURATION_THRESHOLD; */
    u_pid_0 = controller_DW.PID_SATURATION_THRESHOLD;
  }

  /* 'func_map_pid_to_servo:44' if u(i)< -PID_SATURATION_THRESHOLD */
  if (u_pid_0 < -controller_DW.PID_SATURATION_THRESHOLD) {
    /* 'func_map_pid_to_servo:45' u(i) = -PID_SATURATION_THRESHOLD; */
    u_pid_0 = -controller_DW.PID_SATURATION_THRESHOLD;
  }

  u_pid[3] = u_pid_0;

  /*  if position err is too small: set ctrl effort to zero */
  /* 'func_map_pid_to_servo:51' for i=1:4 */
  /* 'func_map_pid_to_servo:52' if abs(error(i))<PID_TRACKING_PRECISION_THRESHOLD */
  if (fabs(err_idx_0) < controller_DW.PID_TRACKING_PRECISION_THRESHOL) {
    /* 'func_map_pid_to_servo:53' u(i) = 0; */
    u_pid[0] = 0.0;

    /* 'func_map_pid_to_servo:54' ERR_INTEGRALE(i) = 0; */
    controller_DW.ERR_INTEGRALE[0] = 0.0;
  }

  /* 'func_map_pid_to_servo:52' if abs(error(i))<PID_TRACKING_PRECISION_THRESHOLD */
  if (fabs(err_idx_1) < controller_DW.PID_TRACKING_PRECISION_THRESHOL) {
    /* 'func_map_pid_to_servo:53' u(i) = 0; */
    u_pid[1] = 0.0;

    /* 'func_map_pid_to_servo:54' ERR_INTEGRALE(i) = 0; */
    controller_DW.ERR_INTEGRALE[1] = 0.0;
  }

  /* 'func_map_pid_to_servo:52' if abs(error(i))<PID_TRACKING_PRECISION_THRESHOLD */
  if (fabs(err_idx_2) < controller_DW.PID_TRACKING_PRECISION_THRESHOL) {
    /* 'func_map_pid_to_servo:53' u(i) = 0; */
    u_pid[2] = 0.0;

    /* 'func_map_pid_to_servo:54' ERR_INTEGRALE(i) = 0; */
    controller_DW.ERR_INTEGRALE[2] = 0.0;
  }

  /* 'func_map_pid_to_servo:52' if abs(error(i))<PID_TRACKING_PRECISION_THRESHOLD */
  if (fabs(err_idx_3) < controller_DW.PID_TRACKING_PRECISION_THRESHOL) {
    /* 'func_map_pid_to_servo:53' u(i) = 0; */
    u_pid[3] = 0.0;

    /* 'func_map_pid_to_servo:54' ERR_INTEGRALE(i) = 0; */
    controller_DW.ERR_INTEGRALE[3] = 0.0;
  }

  /* 'func_map_pid_to_servo:59' u_drv = H*abs(u); */
  fwd[0] = fabs(u_pid[0]);
  fwd[1] = fabs(u_pid[1]);
  fwd[2] = fabs(u_pid[2]);
  fwd[3] = fabs(u_pid[3]);

  /*  end of code */
  argout[0] = (int8_T)i;
  argout[8] = 0;
  argout[16] = 0;
  argout[24] = 0;
  argout[1] = (int8_T)b_argout;
  argout[9] = 0;
  argout[17] = 0;
  argout[25] = 0;
  argout[2] = 0;
  argout[10] = (int8_T)c_argout;
  argout[18] = 0;
  argout[26] = 0;
  argout[3] = 0;
  argout[11] = (int8_T)d_argout;
  argout[19] = 0;
  argout[27] = 0;
  argout[4] = 0;
  argout[12] = 0;
  argout[20] = (int8_T)e_argout;
  argout[28] = 0;
  argout[5] = 0;
  argout[13] = 0;
  argout[21] = (int8_T)f_argout;
  argout[29] = 0;
  argout[6] = 0;
  argout[14] = 0;
  argout[22] = 0;
  argout[30] = (int8_T)g_argout;
  argout[7] = 0;
  argout[15] = 0;
  argout[23] = 0;
  argout[31] = (int8_T)h_argout;
  for (i = 0; i < 8; i++) {
    rtb_aro_angle_idx_1 = (real_T)argout[i + 24] * fwd[3] + ((real_T)argout[i +
      16] * fwd[2] + ((real_T)argout[i + 8] * fwd[1] + (real_T)argout[i] * fwd[0]));
    rtb_us[i] = rtb_aro_angle_idx_1;
  }

  /* Outport: '<Root>/debug' incorporates:
   *  MATLAB Function: '<Root>/func_actuator_controller'
   */
  /*  some update for memory */
  /* '<S4>:1:111' err2mem = err; */
  /* '<S4>:1:112' debug = err; */
  controller_Y.debug[0] = err_idx_0;
  controller_Y.debug[1] = err_idx_1;
  controller_Y.debug[2] = err_idx_2;
  controller_Y.debug[3] = err_idx_3;

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

  /* Outport: '<Root>/time' incorporates:
   *  MATLAB Function: '<Root>/fcn_timer'
   */
  controller_Y.time = rtb_DigitalClock;

  /* Outport: '<Root>/jointAngles' */
  controller_Y.jointAngles[0] = rtb_out_idx_0;
  controller_Y.jointAngles[1] = rtb_out_idx_1;
  controller_Y.jointAngles[2] = rtb_out_idx_2;
  controller_Y.jointAngles[3] = rtb_out;

  /* Outport: '<Root>/servo_err' incorporates:
   *  MATLAB Function: '<Root>/func_actuator_controller'
   */
  controller_Y.servo_err[0] = err_idx_0;
  controller_Y.servo_err[1] = err_idx_1;
  controller_Y.servo_err[2] = err_idx_2;
  controller_Y.servo_err[3] = err_idx_3;
  for (i = 0; i < 16; i++) {
    /* Outport: '<Root>/flight_ctrl1' */
    controller_Y.flight_ctrl1[i] = rtb_par[i];

    /* Outport: '<Root>/flight_ctrl2' */
    controller_Y.flight_ctrl2[i] = rtb_delta_par[i];
  }

  /* Outport: '<Root>/flight_ctrl3' */
  memcpy(&controller_Y.flight_ctrl3[0], &rtb_delta_x[0], 12U * sizeof(real_T));

  /* Update for UnitDelay: '<S10>/Unit Delay3' incorporates:
   *  Inport: '<Root>/pitch'
   *  Inport: '<Root>/roll'
   *  Inport: '<Root>/yaw'
   *  MATLAB Function: '<S10>/aro'
   *  SignalConversion: '<S12>/TmpSignal ConversionAt SFunction Inport1'
   */
  controller_DW.UnitDelay3_DSTATE[0] = controller_U.roll;
  controller_DW.UnitDelay3_DSTATE[1] = controller_U.pitch;
  controller_DW.UnitDelay3_DSTATE[2] = controller_U.yaw;

  /* Update for UnitDelay: '<S10>/Unit Delay1' incorporates:
   *  MATLAB Function: '<S10>/aro'
   */
  controller_DW.UnitDelay1_DSTATE[0] = rtb_aroAngle_idx_0;
  controller_DW.UnitDelay1_DSTATE[1] = rtb_aroAngle_idx_1;
  controller_DW.UnitDelay1_DSTATE[2] = rtb_aroAngle_idx_2;

  /* Update for UnitDelay: '<S1>/Unit Delay3' incorporates:
   *  MATLAB Function: '<S1>/aro'
   */
  controller_DW.UnitDelay3_DSTATE_b[0] = rtb_deg2rad_idx_0;
  controller_DW.UnitDelay3_DSTATE_b[1] = rtb_deg2rad_idx_1;
  controller_DW.UnitDelay3_DSTATE_b[2] = rtb_deg2rad_idx_2;
  controller_DW.UnitDelay3_DSTATE_b[3] = rtb_deg2rad_idx_3;

  /* Update for UnitDelay: '<Root>/Unit Delay2' incorporates:
   *  MATLAB Function: '<Root>/func_actuator_controller'
   */
  controller_DW.UnitDelay2_DSTATE[0] = err_idx_0;
  controller_DW.UnitDelay2_DSTATE[1] = err_idx_1;
  controller_DW.UnitDelay2_DSTATE[2] = err_idx_2;
  controller_DW.UnitDelay2_DSTATE[3] = err_idx_3;

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.01, which is the step size
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
