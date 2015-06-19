#ifndef LIDA_SCREW_H_
#define LIDA_SCREW_H_

//void main_PWM_IC_init(void); //PWM input, no use for screw programs
/* PID */
typedef struct PID_param_t
{
	float Kp;
	float Ki;
	float Kd;
	int32_t integral_max;
	int32_t integral_min;
} PID_param_t;

typedef struct PID_data_t
{
	int32_t integral;
	int32_t prev_error;
} PID_data_t;

/* Screw_driver */
typedef struct screw_position_t
{
	uint16_t round;
	uint16_t degree;
	uint16_t init_degree;
	uint32_t range;
} screw_position_t;

int16_t PID_generic (PID_param_t param, PID_data_t* data, int32_t measured, int32_t expected);
void screw_init(screw_position_t* position);
void MX_I2C_Init();
uint16_t MX_I2C_READ();

//global variables for testing
screw_position_t screw_position;
PID_param_t PID_param;
PID_data_t PID_data;
uint16_t expected_position; //in percentage
int16_t motor_control; //duty cycle
uint16_t read_temp;

#endif