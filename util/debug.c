/*
 * File: debug.cpp
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

/* Include headers */
#include "debug.h"
#include "stm32f4xx_conf.h"
#include "daq.h"
#include  "controller.h"

/* Some macros */
#define DEBUG_NbRcv daq_Y.usart1_NbRcv
#define DEBUG_RcvVal daq_Y.usart1_RcvVal
#define DEBUG_Nb2Send daq_U.usart1_Nb2Send
#define DEBUG_SendVal daq_U.usart1_SendVal

/* Private vars */
char sendbuff[1000];
uint32_T rcvbyte;
char rcvbuff[1];
boolean_T tune_servo_params = false;
boolean_T tune_flight_ctrl_params = false;	
boolean_T calibrate_encs = false;
boolean_T save_param_on_SD = false;

// servo number, e.g., 0,1,2,3
// armwing, e.g. 0 or leg, e.g. 1 
uint8_T servo_num[2] = {0, 0};												

uint8_T flight_param_num = 0;
uint8_T servo_param_num = 0;
double flight_params[2] = {0, 0};
double servo_params[6] = {0, 0, 0, 0, 0, 0};
double motor_cmd[4] = {0, 0, 0, 0};
double servo_param_inc[6] = {INC_KP, INC_KD, INC_KI, INC_POS, INC_POS_TOL, INC_AWU};	
double motor_cmd_inc = INC_PWM_CMD;
double flight_param_inc = INC_FLIGHT_PARAM;

/* extern vars */
extern uint16_t angleReg[4];
extern uint8_t autoGain[4];
extern uint8_t diag[4];
extern uint16_t magnitude[4];
extern double angle[4];

/* Private functions */
uint8_T debug_printf(const char *buff, unsigned int buffLength);
uint32_T debug_scanf(char *buff);

/*******************************************************************************
* Function Name  : debug_bat_robot
* Input          : None
* Output         : None
* Return         : None
* Description    : 
*******************************************************************************/
void debug_bat_robot(void)
{	
	/* read debug */
	rcvbyte = debug_scanf(rcvbuff);
	
	if (!tune_servo_params && !tune_flight_ctrl_params && !calibrate_encs)
	{		
		if (strcmp(rcvbuff, "h") == 0)
		{
			sprintf(sendbuff, "please type in: q(quit), h(help), w(write SD), f(fl par), s(sens calib), c(ctrl), i(imu), m(magnet), e(enc), d(enc diag), 1, 2, 3, or 4 \r\n");
			debug_printf(sendbuff, strlen(sendbuff));
		}
		else if ((strcmp(rcvbuff, "1") == 0) || strcmp(rcvbuff, "2") == 0 || (strcmp(rcvbuff, "3") == 0) || strcmp(rcvbuff, "4") == 0)
		{
			// tuning serovs
			tune_servo_params = true;
			tune_flight_ctrl_params = false;
			calibrate_encs = false;
			if (strcmp(rcvbuff, "1") == 0)
			{
				servo_num[0] = 0;
				servo_num[1] = 0;
			}
			else if (strcmp(rcvbuff, "2") == 0)
			{
				servo_num[0] = 1;
				servo_num[1] = 0;
			}
			else if (strcmp(rcvbuff, "3") == 0)
			{
				servo_num[0] = 2;
				servo_num[1] = 1;
			}
			else if (strcmp(rcvbuff, "4") == 0)
			{
				servo_num[0] = 3;
				servo_num[1] = 1;
			}
		}
		else if ((strcmp(rcvbuff, "w") == 0))
		{
			save_param_on_SD = true;
			rcvbuff[0] = 0;
			sprintf(sendbuff,"Param saved! \r\n");
			debug_printf(sendbuff, strlen(sendbuff));		
		}
		else if (strcmp(rcvbuff, "f") == 0)
		{
			tune_servo_params = false;
			tune_flight_ctrl_params = true;
			calibrate_encs = false;			
		}
		else if (strcmp(rcvbuff, "s") == 0)
		{
			tune_servo_params = false;
			tune_flight_ctrl_params = false;
			calibrate_encs = true;			
		}
		else if (strcmp(rcvbuff, "c") == 0)
		{
			sprintf(sendbuff, "ctrl1 = %f, ctrl2 = %f, ctrl3 = %f,  ctrl4 = %f \r\n", controller_Y.flight_ctrl[0], controller_Y.flight_ctrl[1], 
			        controller_Y.flight_ctrl[2], controller_Y.flight_ctrl[3]);
			debug_printf(sendbuff, strlen(sendbuff));
		}
		else if (strcmp(rcvbuff, "i") == 0)
		{
			sprintf(sendbuff, "roll = %f, pitch = %f, yaw = %f \r\n", controller_Y.q[0], controller_Y.q[1], controller_Y.q[2]);
			debug_printf(sendbuff, strlen(sendbuff)); 	
		}
		else if (strcmp(rcvbuff, "m") == 0)
		{
			sprintf(sendbuff, "mag1 = %d, mag2 = %d, mag3 = %d, mag4 = %d \r\n", autoGain[0], autoGain[1], autoGain[2], 0);
			debug_printf(sendbuff, strlen(sendbuff));
		}
		else if (strcmp(rcvbuff, "e") == 0)
		{
			sprintf(sendbuff, "angle1 = %f, angle2 = %f, angle3 = %f,  angle4 = %f \r\n", angle[0], angle[1], angle[2], 0);
			debug_printf(sendbuff, strlen(sendbuff));
		}
		else if (strcmp(rcvbuff, "d") == 0)
		{
			sprintf(sendbuff, "Encdiag1 = %d, Encdiag2 = %d, Encdiag3 = %d,  Encdiag4 = %d \r\n", diag[0], diag[1], diag[2], 0);
			debug_printf(sendbuff, strlen(sendbuff));
		}
		else
		{
			sprintf(sendbuff, "Internal Clock = %f \r\n", controller_Y.time);
			debug_printf(sendbuff, strlen(sendbuff));
		}
	}
	else if (tune_servo_params)
	{			
		// Specify servo param, KP, KD, KI, SERVO POS, POS TOL, AWU
		if (strcmp(rcvbuff, "p") == 0)
		{
			servo_param_num = 0;
		}
		else if (strcmp(rcvbuff, "d") == 0)
		{
			servo_param_num = 1;
		}
		else if (strcmp(rcvbuff, "i") == 0)
		{
			servo_param_num = 2;
		}
		else if (strcmp(rcvbuff, "o") == 0)
		{
			servo_param_num = 3;
		}
		else if (strcmp(rcvbuff, "t") == 0)
		{
			servo_param_num = 4;
		}
		else if (strcmp(rcvbuff, "a") == 0)
		{
			servo_param_num = 5;
		}

		// ...
		servo_params[0] = controller_U.pid_gian[2 * 0 + servo_num[1]]; // Kp
		servo_params[1] = controller_U.pid_gian[2 * 1 + servo_num[1]]; // Kd
		servo_params[2] = controller_U.pid_gian[2 * 2 + servo_num[1]]; // Ki				
		servo_params[3] = controller_U.flight_ctrl_params[6 + servo_num[0]]; // SERVO POS
		servo_params[4] = controller_U.actuator_ctrl_params[12]; // POS TOL
		servo_params[5] = controller_U.actuator_ctrl_params[13]; // ANTI-WIND-UP THRESHOLD
		
		// Update the specfied param
		if (strcmp(rcvbuff, "w") == 0) 
		{
			servo_params[servo_param_num] += servo_param_inc[servo_param_num];
			rcvbuff[0] = 0;		
		}
		else if (strcmp(rcvbuff, "s") == 0) 
		{
			servo_params[servo_param_num] -= servo_param_inc[servo_param_num];
			rcvbuff[0] = 0;			
		}
		
		// ...
		controller_U.pid_gian[2 * 0 + servo_num[1]] = servo_params[0]; // Kp
		controller_U.pid_gian[2 * 1 + servo_num[1]] = servo_params[1]; // Kd
		controller_U.pid_gian[2 * 2 + servo_num[1]] = servo_params[2]; // Ki				
		controller_U.flight_ctrl_params[6 + servo_num[0]] = servo_params[3]; // SERVO POS
		controller_U.actuator_ctrl_params[12] = servo_params[4]; // POS TOL
		controller_U.actuator_ctrl_params[13] = servo_params[5]; // ANTI-WIND-UP THRESHOLD
		

		// Now deal with servo tuning...
		sprintf(sendbuff, "Kp = %f, Kd = %f, Ki = %f, Pos = %f, tol = %f, awu = %f \r\n",
			controller_U.pid_gian[2 * 0 + servo_num[1]], controller_U.pid_gian[2 * 1 + servo_num[1]], controller_U.pid_gian[2 * 2 + servo_num[1]],
		        controller_U.flight_ctrl_params[6 + servo_num[0]], controller_U.actuator_ctrl_params[12], controller_U.actuator_ctrl_params[13]);
		debug_printf(sendbuff, strlen(sendbuff));
	}
	else if (tune_flight_ctrl_params)
	{
		// Tune flight params,  ROLL_SENSITIVITY, PITCH_SENSITIVITY
		// specify the param
		if (strcmp(rcvbuff, "r") == 0)	// roll
		{
			flight_param_num = 0;
		}
		else if (strcmp(rcvbuff, "p") == 0) // pitch
		{
			flight_param_num = 1;
		}
		//else
		//{
			//sprintf(sendbuff, "press (r, or p) !!! \r\n");
			//debug_printf(sendbuff, strlen(sendbuff));
		//}
		
		// Update PWM command to the specified motor
		if (strcmp(rcvbuff, "w") == 0) 
		{
			controller_U.flight_ctrl_params[flight_param_num] += flight_param_inc;
			rcvbuff[0] = 0;
		}
		else if (strcmp(rcvbuff, "s") == 0) 
		{
			controller_U.flight_ctrl_params[flight_param_num] -= flight_param_inc;
			rcvbuff[0] = 0;
		}
		
		sprintf(sendbuff, "Roll Sens. = %f, Pitch Sens. = %f \r\n", controller_U.flight_ctrl_params[0], controller_U.flight_ctrl_params[1]);
		debug_printf(sendbuff, strlen(sendbuff));		
	}
	else if (calibrate_encs)
	{
		// Specify the servo 
		if (strcmp(rcvbuff, "1") == 0)
		{
			servo_num[0] = 0;
		}
		else if (strcmp(rcvbuff, "2") == 0)
		{
			servo_num[0] = 1;
		}
		else if (strcmp(rcvbuff, "3") == 0)
		{
			servo_num[0] = 2;
		}
		else if (strcmp(rcvbuff, "4") == 0)
		{
			servo_num[0] = 3;
		}
		//else
		//{
			//sprintf(sendbuff, "Not a valid servo number!!! \r\n");
			//debug_printf(sendbuff, strlen(sendbuff));
		//}

		// Update PWM command to the specified motor
		if (strcmp(rcvbuff, "w") == 0) 
		{
			motor_cmd[servo_num[0]] += motor_cmd_inc;
			rcvbuff[0] = 0;
		}
		else if (strcmp(rcvbuff, "s") == 0) 
		{
			motor_cmd[servo_num[0]] -= motor_cmd_inc;
			rcvbuff[0] = 0;
		}		
		sprintf(sendbuff, "M1 = %f, M2 = %f, M3 = %f, M4 = %f \r\n", motor_cmd[0], motor_cmd[1], motor_cmd[2], motor_cmd[3]);
		debug_printf(sendbuff, strlen(sendbuff));		
	}
	

	if (strcmp(rcvbuff, "q") == 0)
	{
		tune_servo_params = false;
		tune_flight_ctrl_params = false;
		calibrate_encs = false;
	}
}

/*******************************************************************************
* Function Name  : debug_printf
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/
uint8_T debug_printf(const char *buff, unsigned int buffLength){
	uint8_T i;
	
	for(i=0; i<buffLength; i++){
		DEBUG_SendVal[i] = buff[i];
	}
	DEBUG_Nb2Send = buffLength;
	return  buffLength;
}

/*******************************************************************************
* Function Name  : debug_scanf
* Input          : None
* Output         : None
* Return         : None
* Description    : **
*******************************************************************************/
uint32_T debug_scanf(char *buff) {
	uint32_T i;
	
	for (i = 0; i < DEBUG_NbRcv; i++) {
		buff[i] = DEBUG_RcvVal[i];
	}
	return DEBUG_NbRcv;
}



