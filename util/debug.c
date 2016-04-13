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
#include "stm32f4_sdio_sd.h"
#include "sdio_debug.h"
#include "diskio.h"
#include "ff.h"

/* Some macros */
#ifdef _BLUETOOTH
#define DEBUG_NbRcv daq_Y.usart6_NbRcv
#define DEBUG_RcvVal daq_Y.usart6_RcvVal
#define DEBUG_Nb2Send daq_U.usart6_Nb2Send
#define DEBUG_SendVal daq_U.usart6_SendVal
#else
#define DEBUG_NbRcv daq_Y.usart1_NbRcv
#define DEBUG_RcvVal daq_Y.usart1_RcvVal
#define DEBUG_Nb2Send daq_U.usart1_Nb2Send
#define DEBUG_SendVal daq_U.usart1_SendVal
#endif

/* Private vars */
char sendbuff[1000];
uint32_T rcvbyte;
char rcvbuff[1];
boolean_T tune_servo_params = false;
boolean_T tune_flight_ctrl_params = false;	
boolean_T calibrate_encs = false;
boolean_T save_param_on_SD = false;
FATFS fs;
FIL	data;							// File object 
FIL	param;
FRESULT	fresult;					// FatFs return code 
FILINFO info;
SD_Error SDstatus;
char_T buffstr[1000];			// Line buffer 
char_T filename[16];			// File name buffer
char_T line[1000];				// Line buffer 
UINT bw, strlength;

// servo number, e.g., 0,1,2,3
// armwing, e.g. 0 or leg, e.g. 1 
uint8_T servo_num[2] = {0, 0};	
uint8_T actuator_ctrl_params_index[2] = {0,0};

uint8_T flight_param_num = 0;
uint8_T servo_param_num = 0;
double flight_params[2] = {0, 0};
double servo_params[6] = {0, 0, 0, 0, 0, 0};
double motor_cmd[4] = {0, 0, 0, 0};
double servo_param_inc[6] = {INC_KP, INC_KD, INC_KI, INC_POS, INC_POS_TOL, INC_AWU};	
double motor_cmd_inc = INC_PWM_CMD;
double flight_param_inc = INC_FLIGHT_PARAM;
double angle_max_min_inc = INC_ANGLE_MAX_MIN; 

/* extern vars */
extern uint16_t angleReg[4];
extern uint8_t autoGain[4];
extern uint8_t diag[4];
extern uint16_t magnitude[4];
extern double angle[4];

extern float yaw, pitch, roll;
extern float accel[3];
extern float rates[3];

boolean_T handshakeRcvd = false;

/* Private functions */
uint8_T debug_printf(const char *buff, unsigned int buffLength);
uint32_T debug_scanf(char *buff);


/*******************************************************************************
* Function Name  : debug_write_params
* Input          : None
* Output         : None
* Return         : None
* Description    : writes control params on SD card
					// forelimb Kp 
					// leg Kp
					// forelimb Kd 
					// leg Kd
					// forelimb Ki 
					// leg Ki 
					// PID_SATURATION_THRESHOLD [pwm] (control sat, used in pid func)
					// MAX_ANGLE_DIFFERENCE [deg\sec] (used in anti-roll-over func)
					// ANTI_ROLLOVER_CORRECTION [deg] (used in anti-roll-over func)
					// MAX_RP_ANGLE_RIGHT [deg] 
					// MAX_DV_ANGLE_RIGHT [deg]
					// MIN_RP_ANGLE_RIGHT [deg]
					// MIN_DV_ANGLE_RIGHT [deg]
					// MAX_RP_ANGLE_LEFT [deg]
					// MAX_DV_ANGLE_LEFT [deg]
					// MIN_RP_ANGLE_LEFT [deg]
					// MIN_DV_ANGLE_LEFT [deg]
					// Sample interval
					// PID_TRACKING_PRECISION_THRESHOLD
					// ANTI_WINDUP_THRESHOLD
					// ROLL_SENSITIVITY [-] -0.01
					// PITCH_SENSITIVITY [-] 0.001
					// MAX_FORELIMB_ANGLE [deg] (n.a.)
					// MIN_FORELIMB_ANGLE [deg] (n.a.)
					// MAX_LEG_ANGLE [deg] (n.a.)
					// MIN_LEG_ANGLE [deg] (n.a.)
					// R_foreq  [deg]
					// L_foreq [deg]
					// R_leq [deg]
					// L_leq [deg]
*******************************************************************************/
void debug_write_params(void)
{
	if (save_param_on_SD)
	{
		/* Save params as param.txt file on SD card. */ 	
		sprintf(filename, "param.txt"); 
		/* open file on SD card */
		while (f_open(&param, filename, FA_OPEN_ALWAYS | FA_WRITE) != FR_OK)
			;
			
		/* close file */
		f_close(&param);
			
		/* Flight params are saved on SD card per request... */
		// 1st col: .....
	
			/* find the length of the datalog file */
		fresult = f_stat("param.txt", &info);  
		
		fresult = f_open(&param, filename, FA_OPEN_ALWAYS | FA_WRITE);
		
	/* If the file existed seek to the end */
	//if (fresult == FR_OK) f_lseek(&param, info.fsize);
		sprintf(buffstr,
			"%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f,\r\n", 
			controller_U.pid_gian[0],
			controller_U.pid_gian[1],
			controller_U.pid_gian[2],
			controller_U.pid_gian[3],
			controller_U.pid_gian[4],
			controller_U.pid_gian[5],
			controller_U.actuator_ctrl_params[0],
			controller_U.actuator_ctrl_params[1],
			controller_U.actuator_ctrl_params[2],
			controller_U.actuator_ctrl_params[3],
			controller_U.actuator_ctrl_params[4],
			controller_U.actuator_ctrl_params[5],
			controller_U.actuator_ctrl_params[6],
			controller_U.actuator_ctrl_params[7],
			controller_U.actuator_ctrl_params[8],
			controller_U.actuator_ctrl_params[9],
			controller_U.actuator_ctrl_params[10],
			controller_U.actuator_ctrl_params[11],
			controller_U.actuator_ctrl_params[12],
			controller_U.actuator_ctrl_params[13],
			controller_U.flight_ctrl_params[0],
			controller_U.flight_ctrl_params[1],
			controller_U.flight_ctrl_params[2],
			controller_U.flight_ctrl_params[3],
			controller_U.flight_ctrl_params[4],
			controller_U.flight_ctrl_params[5],
			controller_U.flight_ctrl_params[6],
			controller_U.flight_ctrl_params[7],
			controller_U.flight_ctrl_params[8],
			controller_U.flight_ctrl_params[9]);
		fresult = f_write(&param, buffstr, strlen(buffstr), &bw);
		f_close(&param); 
			
		// update 
		save_param_on_SD = false;
	}
}

/*******************************************************************************
* Function Name  : debug_write_data
* Input          : None
* Output         : None
* Return         : None
* Description    : writes flight data on SD card
* Data structure is as following:
		// 1st col: time	2nd col: roll	3rd col: pitch	4th col: yaw	5th col: enc1	
		// 6th col: enc2	7th col: enc3	8th col:enc4	9th col: u1	10th col: u2	11th col: u3	12th col: u4
*******************************************************************************/
void debug_write_data(void)
{
		/* find the length of the datalog file */
	fresult = f_stat("data.txt", &info);  
 
	sprintf(filename, "data.txt"); 
	fresult = f_open(&data, filename, FA_OPEN_ALWAYS | FA_WRITE);
		
	/* If the file existed seek to the end */
	if (fresult == FR_OK) f_lseek(&data, info.fsize);
		
	sprintf(buffstr,
		"%f, %f, %f, %f, %f, %f, %f, %f \r\n",
		controller_Y.time,
		controller_Y.q[0],
		controller_Y.q[1], 
		controller_Y.q[2],
		controller_U.angle[0],
		controller_U.angle[1],
		controller_U.angle[2],
		controller_U.angle[3]);
	fresult = f_write(&data, buffstr, strlen(buffstr), &bw);
		
	f_close(&data); 
}
	
/*******************************************************************************
* Function Name  : debug_initialize_files
* Input          : None
* Output         : None
* Return         : None
* Description    : initilaizes documents on SD card
*******************************************************************************/
void debug_initialize_files(void)
{
	/////////////////////////////// Make a quick header for SD log file ///////////////////////////////////
	
	/* Register work area to the default drive */
	f_mount(0, &fs);
	
	/* Save as data.txt file on SD card. */ 
	sprintf(filename, "data.txt"); 
	
	/* open file on SD card */
	while (f_open(&data, filename, FA_OPEN_ALWAYS | FA_WRITE) != FR_OK)
		;
	
		/* write the header to file */
	char *header = "Hello World! This is B2 talking! I am saving data on SD card. \r\n";
	while (f_write(&data, header, strlen(header), &bw) != FR_OK)	// Write data to the file 
		;	
			
		/* close file */
	f_close(&data);
	
	/* open file on SD card */
	char *pch;
	float var, varVector[30];
	uint8_T index = 0;
		
	/* Read param.txt file on SD card. */ 
	sprintf(filename, "param.txt"); 
	if (f_open(&param, filename, FA_READ) == FR_OK)
	{
		/* Read param line */
		f_gets(line, sizeof line, &param);
		pch = strtok(line, " ,");
		while (pch != NULL)
		{
			sscanf(pch, "%f", &var);
			pch = strtok(NULL, " ,");
			varVector[index] = var;
			index++;
		}
		
	/* Write servo params */
		for (index = 0;index < 6;index++)
		{
			controller_U.pid_gian[index] = varVector[index];
		}
		for (index = 0;index < 14;index++)
		{
			controller_U.actuator_ctrl_params[index] = varVector[index + 6];
		}
		/* Write fligth control params */
		for (index = 0;index < 10;index++)
		{
			controller_U.flight_ctrl_params[index] = varVector[index + 20];
		}
	}
	else
	{
		/* Write servo params */
		for (index = 0;index < 6;index++)
		{
			controller_U.pid_gian[index] = 0;
		}
		for (index = 0;index < 14;index++)
		{
			controller_U.actuator_ctrl_params[index] = 0;
		}
		/* Write fligth control params */
		for (index = 0;index < 10;index++)
		{
			controller_U.flight_ctrl_params[index] = 0;
		}
	}
			
	/* Close the file */
	f_close(&param);
		
	/////////////////////////////// Make a quick header for SD log file ///////////////////////////////////	
}

/*******************************************************************************
* Function Name  : debug_initialize_bluetooth
* Input          : None
* Output         : None
* Return         : None
* Description    : initilaizes bluetooth module
*******************************************************************************/
boolean_T debug_check_bluetooth(void)
{
	char sendBuff[100];
	char receiveBuff[100];
	char handshakeStr [] = "OPEN_OK 15 SPP\r";
	char cmdStr [] = "ENTER_DATA_MODE 15\r";
	char ackStr [] = "OK\r";

	/* step1: wait for "OPEN_OK 15 SPP\r" message*/
	debug_scanf(receiveBuff);
	if (strstr(receiveBuff, handshakeStr) != NULL)
	{
		/* step2: if step1 passed send "ENTER_DATA_MODE 15\r" */
		sprintf(sendBuff, cmdStr);
		debug_printf(sendBuff, strlen(sendBuff));
	}
	
	/* step3: wait for "OK \r" */						
	debug_scanf(receiveBuff);
	if (strstr(receiveBuff, ackStr) != NULL)
	{
		/* initialization is done!! */
		handshakeRcvd =  true;
	}
	
	return handshakeRcvd;
		
}

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
			sprintf(sendbuff, "please type in: q(quit), h(help), r(RF), w(write SD), f(fl par), s(sens calib), c(ctrl), i(imu), m(magnet), e(enc), d(enc diag), 1, 2, 3, or 4 \r\n");
			debug_printf(sendbuff, strlen(sendbuff));
		}
		else if ((strcmp(rcvbuff, "1") == 0) || strcmp(rcvbuff, "2") == 0 || (strcmp(rcvbuff, "3") == 0) || strcmp(rcvbuff, "4") == 0 || strcmp(rcvbuff, "5") == 0)
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
			else if (strcmp(rcvbuff, "5") == 0)
			{
				servo_num[0] = 4;
				servo_num[1];
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
			servo_num[0] = 0;
			uint8_t i;
			for (i = 0;i < 4;i++)
			{
				motor_cmd[i] = 0;
			}
		}
		else if (strcmp(rcvbuff, "c") == 0)
		{
			sprintf(sendbuff, "ctrl1 = %f, ctrl2 = %f, ctrl3 = %f,  ctrl4 = %f \r\n", controller_Y.flight_ctrl[0], controller_Y.flight_ctrl[1], 
					controller_Y.flight_ctrl[2], controller_Y.flight_ctrl[3]);
			debug_printf(sendbuff, strlen(sendbuff));
		}
		else if (strcmp(rcvbuff, "b") == 0)
		{
			sprintf(sendbuff,
				"debug1 = %f, debug2 = %f, degbug3 = %f,  debug4 = %f \r\n",
				controller_Y.debug[0],
				controller_Y.debug[1], 
				controller_Y.debug[2],
				controller_Y.debug[3]);
			debug_printf(sendbuff, strlen(sendbuff));
		}
		else if (strcmp(rcvbuff, "i") == 0)
		{
			sprintf(sendbuff, "roll = %f, pitch = %f, yaw = %f \r\n", controller_Y.q[0], controller_Y.q[1], controller_Y.q[2]);
			debug_printf(sendbuff, strlen(sendbuff)); 	
		}
		else if (strcmp(rcvbuff, "g") == 0)
		{
			sprintf(sendbuff, "rateX = %f, rateY = %f, rateZ = %f \r\n", controller_Y.q[6], controller_Y.q[7], controller_Y.q[8]);
			debug_printf(sendbuff, strlen(sendbuff)); 	
		}
		else if (strcmp(rcvbuff, "a") == 0)
		{
			sprintf(sendbuff, "accelX = %f, accelY = %f, accelZ = %f \r\n", accel[0], accel[1], accel[2]);
			debug_printf(sendbuff, strlen(sendbuff)); 	
		}
		else if (strcmp(rcvbuff, "m") == 0)
		{
			sprintf(sendbuff, "mag1 = %d, mag2 = %d, mag3 = %d, mag4 = %d \r\n", autoGain[0], autoGain[1], autoGain[2], autoGain[3]);
			debug_printf(sendbuff, strlen(sendbuff));
		}
		else if (strcmp(rcvbuff, "e") == 0)
		{
			sprintf(sendbuff, "angle1 = %f, angle2 = %f, angle3 = %f,  angle4 = %f \r\n", controller_U.angle[0], controller_U.angle[1], controller_U.angle[2], controller_U.angle[3]);
			debug_printf(sendbuff, strlen(sendbuff));
		}
		else if (strcmp(rcvbuff, "d") == 0)
		{
			sprintf(sendbuff, "Encdiag1 = %d, Encdiag2 = %d, Encdiag3 = %d,  Encdiag4 = %d \r\n", diag[0], diag[1], diag[2], diag[3]);
			debug_printf(sendbuff, strlen(sendbuff));
		}
		else if (strcmp(rcvbuff, "r") == 0)
		{
			sprintf(sendbuff, "CH1 = %f, CH2 = %f, CH3 = %f \r\n", daq_Y.dc1, daq_Y.dc2, daq_Y.dc3);
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
		if (servo_num[0] == 4)
		{
			servo_param_num = 3; // Position

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
			controller_U.flight_ctrl_params[6] = servo_params[3]; // SERVO POS
			controller_U.flight_ctrl_params[7] = servo_params[3]; // SERVO POS
			controller_U.flight_ctrl_params[8] = servo_params[3]; // SERVO POS
			controller_U.flight_ctrl_params[9] = servo_params[3]; // SERVO POS
			

					// Now deal with servo tuning...
			sprintf(sendbuff,
				"Pos1 = %f, Pos2 = %f, Pos3 = %f, Pos4 = %f \r\n",
				controller_U.flight_ctrl_params[6],
				controller_U.flight_ctrl_params[7],
				controller_U.flight_ctrl_params[8],
				controller_U.flight_ctrl_params[9]);
			debug_printf(sendbuff, strlen(sendbuff));
		}
		else
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
			sprintf(sendbuff,
				"Kp = %f, Kd = %f, Ki = %f, Pos = %f, tol = %f, awu = %f \r\n",
				controller_U.pid_gian[2 * 0 + servo_num[1]],
				controller_U.pid_gian[2 * 1 + servo_num[1]],
				controller_U.pid_gian[2 * 2 + servo_num[1]],
				controller_U.flight_ctrl_params[6 + servo_num[0]],
				controller_U.actuator_ctrl_params[12],
				controller_Y.debug[3]);
			debug_printf(sendbuff, strlen(sendbuff));
		}
	}
	else if (tune_flight_ctrl_params)
	{
		// Tune flight params,  ROLL_wing_kp, ROLL_leg_kp, PITCH_leg_kp, ROLL_wing_kd, ROLL_leg_kd, PITCH_leg_kd
		// specify the param
		if (strcmp(rcvbuff, "1") == 0)	// ROLL_wing_kp
		{
			flight_param_num = 0;
		}
		else if (strcmp(rcvbuff, "2") == 0) // ROLL_leg_kp
		{
			flight_param_num = 1;
		}
		else if (strcmp(rcvbuff, "3") == 0) // PITCH_leg_kp
		{
			flight_param_num = 2;
		}
		else if (strcmp(rcvbuff, "4") == 0) // ROLL_wing_kd
		{
			flight_param_num = 3;
		}
		else if (strcmp(rcvbuff, "5") == 0) // ROLL_leg_kd
		{
			flight_param_num = 4;
		}
		else if (strcmp(rcvbuff, "6") == 0) // PITCH_leg_kd
		{
			flight_param_num = 5;
		}
		
		
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
		
		sprintf(sendbuff, "R-w-kp. = %f, R-l-kp. = %f, P-l-kp. = %f, R-w-kd. = %f, R-l-kd. = %f, P-l-kd. = %f \r\n", controller_U.flight_ctrl_params[0], controller_U.flight_ctrl_params[1], 
		        controller_U.flight_ctrl_params[2], controller_U.flight_ctrl_params[3], controller_U.flight_ctrl_params[4], controller_U.flight_ctrl_params[5]);
		debug_printf(sendbuff, strlen(sendbuff));		
	}
	else if (calibrate_encs)
	{
		//servo_num[0] = 0;

		// Specify the servo 
		if (strcmp(rcvbuff, "1") == 0)
		{
			servo_num[0] = 0;
			actuator_ctrl_params_index[0] = 3;	// Max Right RP
			actuator_ctrl_params_index[1] = 5;	// Min Right RP
		}
		else if (strcmp(rcvbuff, "2") == 0)
		{
			servo_num[0] = 1;
			actuator_ctrl_params_index[0] = 7;	// Max Left RP
			actuator_ctrl_params_index[1] = 9;	// Min Left RP
		}
		else if (strcmp(rcvbuff, "3") == 0)
		{
			servo_num[0] = 2;
			actuator_ctrl_params_index[0] = 4;	// Max Right DV
			actuator_ctrl_params_index[1] = 6;	// Min Right DV
		}
		else if (strcmp(rcvbuff, "4") == 0)
		{
			servo_num[0] = 3;
			actuator_ctrl_params_index[0] = 8;	// Max Left DV
			actuator_ctrl_params_index[1] = 10;	// Min Left DV
		}
		else
		{
			//servo_num[0] = 0;
			//actuator_ctrl_params_index[0] = 3;	// Max Right RP
			//actuator_ctrl_params_index[1] = 5;	// Min Right RP
		}		
		
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
		
		// Update max/min angles
		if (strcmp(rcvbuff, "e") == 0) 
		{
			controller_U.actuator_ctrl_params[actuator_ctrl_params_index[0]] += angle_max_min_inc;
			rcvbuff[0] = 0;
		}
		else if (strcmp(rcvbuff, "d") == 0) 
		{
			controller_U.actuator_ctrl_params[actuator_ctrl_params_index[0]] -= angle_max_min_inc;
			rcvbuff[0] = 0;
		}
		
		// Update max/min angles
		if (strcmp(rcvbuff, "r") == 0) 
		{
			controller_U.actuator_ctrl_params[actuator_ctrl_params_index[1]] += angle_max_min_inc;
			rcvbuff[0] = 0;
		}
		else if (strcmp(rcvbuff, "f") == 0) 
		{
			controller_U.actuator_ctrl_params[actuator_ctrl_params_index[1]] -= angle_max_min_inc;
			rcvbuff[0] = 0;
		}
		
		sprintf(sendbuff, "M_CMD = %f, Max = %f, Min = %f \r\n", motor_cmd[servo_num[0]], controller_U.actuator_ctrl_params[actuator_ctrl_params_index[0]], controller_U.actuator_ctrl_params[actuator_ctrl_params_index[1]]);
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



