/*
 * File: controller.h
 *
 * Code generated for Simulink model :controller.
 *
 * Model version      : 1.334
 * Simulink Coder version    : 8.6 (R2014a) 27-Dec-2013
 * TLC version       : 8.6 (Jan 30 2014)
 * C/C++ source code generated on  : Mon May 09 21:55:17 2016
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
#include <math.h>
#include <string.h>
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
  real_T IC[10];                       /* '<Root>/IC' */
  real_T IC2[14];                      /* '<Root>/IC2' */
  real_T UnitDelay3_DSTATE[3];         /* '<S10>/Unit Delay3' */
  real_T UnitDelay1_DSTATE[3];         /* '<S10>/Unit Delay1' */
  real_T UnitDelay3_DSTATE_b[4];       /* '<S1>/Unit Delay3' */
  real_T UnitDelay2_DSTATE[4];         /* '<Root>/Unit Delay2' */
  real_T ERR_INTEGRALE[4];             /* '<Root>/Data Store Memory13' */
  real_T LOCK_MOTORS[4];               /* '<Root>/Data Store Memory16' */
  real_T Pos[3];                       /* '<Root>/Data Store Memory17' */
  real_T vel[3];                       /* '<Root>/Data Store Memory18' */
  real_T ROLLOVER_ACTIVATE[2];         /* '<Root>/Data Store Memory19' */
  real_T ROLLOVER_FLAG[4];             /* '<Root>/Data Store Memory3' */
  real_T ANTI_ROLLOVER_CORRECTION;     /* '<Root>/Data Store Memory' */
  real_T PID_SATURATION_THRESHOLD;     /* '<Root>/Data Store Memory1' */
  real_T MAX_DV_ANGLE_LEFT;            /* '<Root>/Data Store Memory10' */
  real_T MIN_DV_ANGLE_LEFT;            /* '<Root>/Data Store Memory11' */
  real_T SAMPLING_INTERVAL;            /* '<Root>/Data Store Memory12' */
  real_T PID_TRACKING_PRECISION_THRESHOL;/* '<Root>/Data Store Memory14' */
  real_T ANTI_WINDUP_THRESHOLD;        /* '<Root>/Data Store Memory15' */
  real_T MAX_ANGLE_DIFFERENCE;         /* '<Root>/Data Store Memory2' */
  real_T MAX_RP_ANGLE_RIGHT;           /* '<Root>/Data Store Memory4' */
  real_T MIN_RP_ANGLE_RIGHT;           /* '<Root>/Data Store Memory5' */
  real_T MAX_DV_ANGLE_RIGHT;           /* '<Root>/Data Store Memory6' */
  real_T MIN_DV_ANGLE_RIGHT;           /* '<Root>/Data Store Memory7' */
  real_T MAX_RP_ANGLE_LEFT;            /* '<Root>/Data Store Memory8' */
  real_T MIN_RP_ANGLE_LEFT;            /* '<Root>/Data Store Memory9' */
  boolean_T IC_FirstOutputTime;        /* '<Root>/IC' */
  boolean_T IC2_FirstOutputTime;       /* '<Root>/IC2' */
} DW_controller;

/* Constant parameters (auto storage) */
typedef struct {
  /* Expression: [0,20,360,1,1,-1,-1,1,1,-1,-1,0.01,0,0].'
   * Referenced by: '<Root>/IC2'
   */
  real_T IC2_Value[14];
} ConstP_controller;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real_T roll;                         /* '<Root>/roll' */
  real_T pitch;                        /* '<Root>/pitch' */
  real_T yaw;                          /* '<Root>/yaw' */
  real_T flight_ctrl_params[10];       /* '<Root>/flight_ctrl_params' */
  uint16_T rawAngle;                   /* '<Root>/rawAngle' */
  uint8_T magneticField;               /* '<Root>/magneticField' */
  real_T angle[4];                     /* '<Root>/angle' */
  real_T pid_gian[6];                  /* '<Root>/pid_gian' */
  real_T actuator_ctrl_params[14];     /* '<Root>/actuator_ctrl_params' */
  real_T wx;                           /* '<Root>/wx' */
  real_T wy;                           /* '<Root>/wy' */
  real_T wz;                           /* '<Root>/wz' */
  real_T accelX;                       /* '<Root>/accelX' */
  real_T accelY;                       /* '<Root>/accelY' */
  real_T accelZ;                       /* '<Root>/accelZ' */
  real_T xd[2];                        /* '<Root>/xd' */
  boolean_T event_based;               /* '<Root>/event_based' */
} ExtU_controller;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real_T x[12];                        /* '<Root>/x' */
  real_T flight_ctrl[4];               /* '<Root>/flight_ctrl' */
  real_T M0A;                          /* '<Root>/M0A' */
  real_T M0B;                          /* '<Root>/M0B' */
  real_T M1A;                          /* '<Root>/M1A' */
  real_T M1B;                          /* '<Root>/M1B' */
  real_T M2A;                          /* '<Root>/M2A' */
  real_T M2B;                          /* '<Root>/M2B' */
  real_T M3A;                          /* '<Root>/M3A' */
  real_T M3B;                          /* '<Root>/M3B' */
  real_T debug[4];                     /* '<Root>/debug' */
  real_T time;                         /* '<Root>/time' */
  real_T jointAngles[4];               /* '<Root>/jointAngles' */
  real_T servo_derr[4];                /* '<Root>/servo_derr' */
  real_T servo_interr[4];              /* '<Root>/servo_interr' */
  real_T servo_err[4];                 /* '<Root>/servo_err' */
  real_T flight_ctrl1[16];             /* '<Root>/flight_ctrl1' */
  real_T flight_ctrl2[16];             /* '<Root>/flight_ctrl2' */
  real_T flight_ctrl3[12];             /* '<Root>/flight_ctrl3' */
} ExtY_controller;

/* Real-time Model Data Structure */
struct tag_RTM_controller {
  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
  } Timing;
};

/* Block signals and states (auto storage) */
extern DW_controller controller_DW;

/* External inputs (root inport signals with auto storage) */
extern ExtU_controller controller_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExtY_controller controller_Y;

/* Constant parameters (auto storage) */
extern const ConstP_controller controller_ConstP;

/* Model entry point functions */
extern void controller_initialize(void);
extern void controller_step(void);

/* Real-time Model object */
extern RT_MODEL_controller *const controller_M;

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
 * '<S1>'   : 'controller/aro'
 * '<S2>'   : 'controller/comput parameters for  event-based controller'
 * '<S3>'   : 'controller/fcn_timer'
 * '<S4>'   : 'controller/func_actuator_controller'
 * '<S5>'   : 'controller/func_flight_controller'
 * '<S6>'   : 'controller/gain-ramp'
 * '<S7>'   : 'controller/state estimator'
 * '<S8>'   : 'controller/aro/aro'
 * '<S9>'   : 'controller/aro/normalize angle'
 * '<S10>'  : 'controller/state estimator/aro'
 * '<S11>'  : 'controller/state estimator/func_state_estimator'
 * '<S12>'  : 'controller/state estimator/aro/aro'
 */
#endif                                 /* RTW_HEADER_controller_h_ */

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF] controller.h
 */
