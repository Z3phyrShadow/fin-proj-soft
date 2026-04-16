/*
 * orientation_ctrl_utils.c
 *
 *  ORCAS v2 – Orientation control utility.
 *  Uses step counting for tilt (accelerometer disabled by default).
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include "main.h"
#include "orientation_ctrl_utils.h"
#include "uart_cmds_handle.h"
#include "error_code.h"

#ifdef TILT_USE_ACCEL
#include "mma845x.h"
#endif

#define ORIENTATION_CTRL_ROUTINES_GRP1_PERIOD_US 200000
#define ORIENTATION_CTRL_ROUTINES_GRP2_PERIOD_US ORIENTATION_CTRL_ROUTINES_MIN_PERIOD_US
uint16_t orientation_ctrl_perodic_routines_timer = 0;

#define ORIENTATION_CTRL_STATUS__MMA845X_COMM_FAIL		0x01
uint8_t orientation_ctrl_status = 0;

#ifdef TILT_USE_ACCEL
#define ACCELEROMETER_ROTATE_ANGLE 	120
extern I2C_HandleTypeDef hi2c2;
#endif

#define TILT_UP_ENDSTOP_ANGLE		40
#define TILT_DOWN_ENDSTOP_ANGLE		(-15)

uint16_t curr_pan_angle_in_steps = 0;
float curr_tilt_angle = 0;

typedef struct stepper_motor_handle {
	#define STATUS__STEP_HIGH				0x01
	#define STATUS__CURR_DIR_CCW 			0x02
	#define STATUS__CURR_TILT_UP			STATUS__CURR_DIR_CCW
	#define STATUS__CURR_PAN_LEFT			STATUS__CURR_DIR_CCW
	#define STATUS__DISPATCHED_DIR_CCW		0x04
	#define STATUS__DISPATCHED_TILT_UP		STATUS__DISPATCHED_DIR_CCW
	#define STATUS__DISPATCHED_PAN_LEFT		STATUS__DISPATCHED_DIR_CCW
	uint8_t status;

	#define STATE__STOP			0
	#define STATE__ACC			1
	#define STATE__DEC			2
	#define STATE__CONST		3
	#define STATE__TURN_AROUND	4
	uint8_t state;

	uint8_t acc_dec_steps;
	uint8_t* acc_dec_half_period_lookup;

	uint8_t half_period_index;
	uint8_t half_period_timer;
	uint16_t dispatched_steps;
	uint16_t remain_steps;
	GPIO_TypeDef* dir_gpio_port;
	uint16_t dir_gpio_pin;
	GPIO_TypeDef* step_gpio_port;
	uint16_t step_gpio_pin;
} stepper_motor_handle_t;

/* Hardcoded steps since we use step-counting mode by default */
uint16_t pan_steps_per_round = 8009;
float pan_angle_per_step = 0.045;
float tilt_angle_per_step = 0.104;

#define PAN_ACC_DEC_STEPS	148
uint8_t pan_acc_dec_half_period_lookup[PAN_ACC_DEC_STEPS] = {
	2,
	3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
	4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
	5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
	6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
	7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
	8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
	9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
    10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10
};
#define TILT_ACC_DEC_STEPS 64
uint8_t tilt_acc_dec_half_period_lookup[TILT_ACC_DEC_STEPS] = {
	5,
	6, 6, 6, 6, 6, 6,
	7, 7, 7, 7, 7,
	8, 8, 8,
	9, 9, 9, 9,
	10, 10,
	11, 11, 11,
	12, 12, 12,
	13, 13,
	14, 14,
	15, 15, 15,
	16, 16,
	17, 17,
	18, 18, 18,
	19, 19, 19,
	20, 20,
	21, 21, 21, 21,
	22, 22, 22,
	23, 23, 23, 23, 23,
	24, 24, 24, 24, 24, 24,
};

stepper_motor_handle_t pan_motor_handle = {
	0,								// status
	STATE__STOP,					// state
	0,								// acc_dec_steps
	pan_acc_dec_half_period_lookup,	// acc_dec_half_period_lookup
	0, 								// half_period_index
	0, 								// half_period_timer
	0,								// dispatched_steps
	0, 								// remain_stapes
	PAN_MOTOR_DIR_GPIO_Port,		// dir_gpio_port
	PAN_MOTOR_DIR_Pin,				// dir_gpio_pin
	PAN_MOTOR_STEP_GPIO_Port,		// step_gpio_port
	PAN_MOTOR_STEP_Pin,				// step_gpio_pin
};

stepper_motor_handle_t tilt_motor_handle = {
	0,								// status
	STATE__STOP,					// state
	0, 								// acc_dec_steps
	tilt_acc_dec_half_period_lookup,// acc_dec_half_period_lookup
	0, 								// half_period_index
	0, 								// half_period_timer
	0,								// dispatched_steps
	0, 								// remain_stapes
	TILT_MOTOR_DIR_GPIO_Port,		// dir_gpio_port
	TILT_MOTOR_DIR_Pin,				// dir_gpio_pin
	TILT_MOTOR_STEP_GPIO_Port,		// step_gpio_port
	TILT_MOTOR_STEP_Pin,			// step_gpio_pin
};

static uint8_t motor_handles_init();
#ifdef TILT_USE_ACCEL
static uint8_t get_tilt_angle(float* angle);
#endif
static uint8_t pan_tilt_ctrl(stepper_motor_handle_t* handle);


uint8_t orientation_ctrl_init(){
	uint8_t ret = 0;
	uint16_t i = 0, j = 0;
#ifdef TILT_USE_ACCEL
	float angle_1, angle_2;
#endif

#ifdef PAN_CTRL
	#define PAN_INIT_STATE__LEAVE_ORIGIN	0
	#define PAN_INIT_STATE__ORIGIN_SEARCH_1	1
	#define PAN_INIT_STATE__ORIGIN_SEARCH_2	2
	#define PAN_INIT_STATE__COMPLETE		3
	uint8_t pan_init_state = PAN_INIT_STATE__LEAVE_ORIGIN;

	HAL_GPIO_WritePin(PAN_MOTOR_DIR_GPIO_Port, PAN_MOTOR_DIR_Pin, GPIO_PIN_RESET);
	while(!ret && (pan_init_state < PAN_INIT_STATE__COMPLETE)){
		switch(pan_init_state){
		case PAN_INIT_STATE__LEAVE_ORIGIN:
			if(HAL_GPIO_ReadPin(PAN_MOTOR_ORIGIN_GPIO_Port, PAN_MOTOR_ORIGIN_Pin) == GPIO_PIN_SET){
				i = 0;
				pan_init_state = PAN_INIT_STATE__ORIGIN_SEARCH_1;
			}else{
				for(i = 0; i < 100; i++){
					for(j = 0; j < 400; j++){};
					HAL_GPIO_WritePin(PAN_MOTOR_STEP_GPIO_Port, PAN_MOTOR_STEP_Pin, GPIO_PIN_SET);
					for(j = 0; j < 400; j++){};
					HAL_GPIO_WritePin(PAN_MOTOR_STEP_GPIO_Port, PAN_MOTOR_STEP_Pin, GPIO_PIN_RESET);
				}
				if(HAL_GPIO_ReadPin(PAN_MOTOR_ORIGIN_GPIO_Port, PAN_MOTOR_ORIGIN_Pin) == GPIO_PIN_RESET){
					ret = ERR__PAN_MOTOR_STEP_INIT_FAIL;
				}
			}
			break;
		case PAN_INIT_STATE__ORIGIN_SEARCH_1:
			if(HAL_GPIO_ReadPin(PAN_MOTOR_ORIGIN_GPIO_Port, PAN_MOTOR_ORIGIN_Pin) == GPIO_PIN_RESET){
				for(i = 0; i < 100; i++){
					for(j = 0; j < 400; j++){};
					HAL_GPIO_WritePin(PAN_MOTOR_STEP_GPIO_Port, PAN_MOTOR_STEP_Pin, GPIO_PIN_SET);
					for(j = 0; j < 400; j++){};
					HAL_GPIO_WritePin(PAN_MOTOR_STEP_GPIO_Port, PAN_MOTOR_STEP_Pin, GPIO_PIN_RESET);
				}
				pan_init_state = PAN_INIT_STATE__ORIGIN_SEARCH_2;
			}else{
				if(i < 10000){
					for(j = 0; j < 400; j++){};
					HAL_GPIO_WritePin(PAN_MOTOR_STEP_GPIO_Port, PAN_MOTOR_STEP_Pin, GPIO_PIN_SET);
					for(j = 0; j < 400; j++){};
					HAL_GPIO_WritePin(PAN_MOTOR_STEP_GPIO_Port, PAN_MOTOR_STEP_Pin, GPIO_PIN_RESET);
					i++;
				}else{
					ret = ERR__PAN_MOTOR_STEP_INIT_FAIL;
				}
			}
			break;
		case PAN_INIT_STATE__ORIGIN_SEARCH_2:
			if(HAL_GPIO_ReadPin(PAN_MOTOR_ORIGIN_GPIO_Port, PAN_MOTOR_ORIGIN_Pin) == GPIO_PIN_RESET){
				pan_steps_per_round = i;
				pan_angle_per_step = (float)360 / pan_steps_per_round;
				curr_pan_angle_in_steps = 0;
				pan_init_state = PAN_INIT_STATE__COMPLETE;
			}else{
				if(i < 10000){
					for(j = 0; j < 400; j++){};
					HAL_GPIO_WritePin(PAN_MOTOR_STEP_GPIO_Port, PAN_MOTOR_STEP_Pin, GPIO_PIN_SET);
					for(j = 0; j < 400; j++){};
					HAL_GPIO_WritePin(PAN_MOTOR_STEP_GPIO_Port, PAN_MOTOR_STEP_Pin, GPIO_PIN_RESET);
					i++;
				}else{
					ret = ERR__PAN_MOTOR_STEP_INIT_FAIL;
				}
			}
			break;
		default:
			break;
		}
	}
	if(ret){
		return ret;
	}
#endif // PAN_CTRL

#ifdef TILT_CTRL
#ifdef TILT_USE_ACCEL
	HAL_Delay(1000);
	ret = mma845x_init(&hi2c2);
	if(ret){
		orientation_ctrl_status |= ORIENTATION_CTRL_STATUS__MMA845X_COMM_FAIL;
		return ret;
	}

	HAL_Delay(500);
	get_tilt_angle(&angle_1);
	if(ret){
		orientation_ctrl_status |= ORIENTATION_CTRL_STATUS__MMA845X_COMM_FAIL;
		return ret;
	}
	HAL_GPIO_WritePin(TILT_MOTOR_DIR_GPIO_Port, TILT_MOTOR_DIR_Pin, GPIO_PIN_SET);		// Tilt up
	for(i = 0; i < 100; i++){
		HAL_Delay(2);
		HAL_GPIO_WritePin(TILT_MOTOR_STEP_GPIO_Port, TILT_MOTOR_STEP_Pin, GPIO_PIN_SET);
		HAL_Delay(2);
		HAL_GPIO_WritePin(TILT_MOTOR_STEP_GPIO_Port, TILT_MOTOR_STEP_Pin, GPIO_PIN_RESET);
	}

	HAL_Delay(500);
	get_tilt_angle(&angle_2);
	if(ret){
		orientation_ctrl_status |= ORIENTATION_CTRL_STATUS__MMA845X_COMM_FAIL;
		return ret;
	}
	tilt_angle_per_step = fabs(angle_2 - angle_1) / 100;

	if(angle_2 > 0){
		tilt_motor_handle.status &= ~STATUS__DISPATCHED_TILT_UP;
	}else{
		tilt_motor_handle.status |= STATUS__DISPATCHED_TILT_UP;
	}
	tilt_motor_handle.dispatched_steps = (uint16_t)(fabs(angle_2) / tilt_angle_per_step);
#else
	// No accelerometer, assume zeroed initially. Tilt angle per step is hardcoded.
	curr_tilt_angle = 0;
#endif // TILT_USE_ACCEL
#endif // TILT_CTRL

	motor_handles_init();

	return 0;
}

uint8_t orientation_ctrl_cmds_dispatch(uint8_t* cmd_buf, uint8_t* ack_buf){
	float angle;

	switch(cmd_buf[CMD_CODE_INDEX]){
	case CMD_CODE__GET_PAN_TILT_STEP_ANGLE:
		ack_buf[ACK_CODE_INDEX] = ACK_CODE__SUCCESS;
		ack_buf[ACK_PARAMS_INDEX] = (uint8_t)(((int16_t)(pan_angle_per_step * 1000)) >> 8);
		ack_buf[ACK_PARAMS_INDEX + 1] = (uint8_t)((int16_t)(pan_angle_per_step * 1000)) & 0x00FF;
		ack_buf[ACK_PARAMS_INDEX + 2] = (uint8_t)(((int16_t)(tilt_angle_per_step * 1000)) >> 8);
		ack_buf[ACK_PARAMS_INDEX + 3] = (uint8_t)((int16_t)(tilt_angle_per_step * 1000)) & 0x00FF;
		break;
	case CMD_CODE__GET_PAN_TILT_CURR_ANGLE:
		ack_buf[ACK_CODE_INDEX] = ACK_CODE__SUCCESS;
#ifdef PAN_CTRL
		if(!pan_angle_per_step){
			ack_buf[ACK_CODE_INDEX] = ERR__PAN_MOTOR_STEP_INIT_FAIL;
		}
#endif
#ifdef TILT_CTRL
#ifdef TILT_USE_ACCEL
		if(orientation_ctrl_status & ORIENTATION_CTRL_STATUS__MMA845X_COMM_FAIL){
			ack_buf[ACK_CODE_INDEX] = ERR__MMA845X_COMM_FAIL;
			orientation_ctrl_status &= ~ORIENTATION_CTRL_STATUS__MMA845X_COMM_FAIL;
		}
#endif
#endif
		angle = (curr_pan_angle_in_steps * pan_angle_per_step);
		if(angle > 180){angle = angle - 360;}
		ack_buf[ACK_PARAMS_INDEX] = (uint8_t)(((int16_t)(angle * 10)) >> 8);;
		ack_buf[ACK_PARAMS_INDEX + 1] = (uint8_t)((int16_t)(angle * 10)) & 0x00FF;;
		ack_buf[ACK_PARAMS_INDEX + 2] = (uint8_t)(((int16_t)(curr_tilt_angle * 10)) >> 8);
		ack_buf[ACK_PARAMS_INDEX + 3] = (uint8_t)((int16_t)(curr_tilt_angle * 10)) & 0x00FF;

		break;
	case CMD_CODE__SET_PAN_TILT_ROTATE_ANGLE:
		ack_buf[ACK_CODE_INDEX] = ACK_CODE__SUCCESS;
#ifdef PAN_CTRL
		if(!pan_angle_per_step){
			ack_buf[ACK_CODE_INDEX] = ERR__PAN_MOTOR_STEP_INIT_FAIL;
		}else{
			angle = (float)((int16_t)((cmd_buf[CMD_PARAMS_INDEX] << 8) | cmd_buf[CMD_PARAMS_INDEX + 1])) / 10;

			if(angle > 0){
				pan_motor_handle.status &= ~STATUS__DISPATCHED_PAN_LEFT;
			}else{
				pan_motor_handle.status |= STATUS__DISPATCHED_PAN_LEFT;
			}
			pan_motor_handle.dispatched_steps = (uint16_t)fabs(angle / pan_angle_per_step);
		}
#endif
#ifdef TILT_CTRL
		if(!tilt_angle_per_step){
			ack_buf[ACK_CODE_INDEX] = ERR__TILT_MOTOR_STEP_INIT_FAIL;
#ifdef TILT_USE_ACCEL
		}else if(orientation_ctrl_status & ORIENTATION_CTRL_STATUS__MMA845X_COMM_FAIL){
			ack_buf[ACK_CODE_INDEX] = ERR__MMA845X_COMM_FAIL;
			orientation_ctrl_status &= ~ORIENTATION_CTRL_STATUS__MMA845X_COMM_FAIL;
#endif
		}else{
			angle = (float)((int16_t)((cmd_buf[CMD_PARAMS_INDEX + 2] << 8) | cmd_buf[CMD_PARAMS_INDEX + 3])) / 10;
			if(angle > 0){
				if(curr_tilt_angle >= TILT_UP_ENDSTOP_ANGLE){
					ack_buf[ACK_CODE_INDEX] = ERR__TILT_ENDSTOP_REACHED;
					break;
				}else{
					tilt_motor_handle.status |= STATUS__DISPATCHED_TILT_UP;
					if(curr_tilt_angle + angle > TILT_UP_ENDSTOP_ANGLE){
						angle = TILT_UP_ENDSTOP_ANGLE - curr_tilt_angle;
					}
				}
			}else{
				if(curr_tilt_angle <= TILT_DOWN_ENDSTOP_ANGLE){
					ack_buf[ACK_CODE_INDEX] = ERR__TILT_ENDSTOP_REACHED;
					break;
				}else{
					tilt_motor_handle.status &= ~STATUS__DISPATCHED_TILT_UP;
					if(curr_tilt_angle + angle < TILT_DOWN_ENDSTOP_ANGLE){
						angle = TILT_DOWN_ENDSTOP_ANGLE - curr_tilt_angle;
					}
				}
			}
			tilt_motor_handle.dispatched_steps = (uint16_t)fabs(angle / tilt_angle_per_step);
		}
#endif
		break;
	default:
		break;
	}

	return 0;
}

uint8_t orientation_ctrl_perodic_routines(){
	uint8_t ret = 0;

	orientation_ctrl_perodic_routines_timer++;
	if(orientation_ctrl_perodic_routines_timer == (ORIENTATION_CTRL_ROUTINES_GRP1_PERIOD_US / ORIENTATION_CTRL_ROUTINES_MIN_PERIOD_US)){
		orientation_ctrl_perodic_routines_timer = 0;
	}

#ifdef TILT_CTRL
#ifdef TILT_USE_ACCEL
	if(orientation_ctrl_perodic_routines_timer % (ORIENTATION_CTRL_ROUTINES_GRP1_PERIOD_US / ORIENTATION_CTRL_ROUTINES_MIN_PERIOD_US) == 0){
		ret = get_tilt_angle(&curr_tilt_angle);
		if(ret){
			orientation_ctrl_status |= ORIENTATION_CTRL_STATUS__MMA845X_COMM_FAIL;
			MX_I2C_ForceClearBusyFlag(&hi2c2, I2C2_SDA_GPIO_Port, I2C2_SDA_Pin, I2C2_SCL_GPIO_Port, I2C2_SCL_Pin);
		}
	}
#endif
#endif

	if(orientation_ctrl_perodic_routines_timer % (ORIENTATION_CTRL_ROUTINES_GRP2_PERIOD_US / ORIENTATION_CTRL_ROUTINES_MIN_PERIOD_US) == 0){
#ifdef PAN_CTRL
		pan_tilt_ctrl(&pan_motor_handle);
#endif
#ifdef TILT_CTRL
		pan_tilt_ctrl(&tilt_motor_handle);
#endif
	}

	return 0;
}

uint8_t orientation_ctrl_isr(uint16_t GPIO_Pin){
#ifdef PAN_CTRL
	if(GPIO_Pin == PAN_MOTOR_ORIGIN_Pin){
		if(!(pan_motor_handle.status & STATUS__CURR_PAN_LEFT)){
			curr_pan_angle_in_steps = 0;
		}
	}
#endif
	return 0;
}

static uint8_t motor_handles_init(){
	pan_motor_handle.acc_dec_steps = PAN_ACC_DEC_STEPS;
	pan_motor_handle.half_period_index = PAN_ACC_DEC_STEPS - 1;
	tilt_motor_handle.acc_dec_steps = TILT_ACC_DEC_STEPS;
	tilt_motor_handle.half_period_index = TILT_ACC_DEC_STEPS - 1;
	return 0;
}

#ifdef TILT_USE_ACCEL
static uint8_t get_tilt_angle(float* angle){
	uint8_t ret = 0;
	int16_t x, y, z;
	float a;

	ret = mma845x_get_xyz(&hi2c2, &x, &y, &z);
	if(!ret){
		a = atan2(x, y) * 180 / M_PI;
		if(a < 0){a += 360;}
		*angle = a - ACCELEROMETER_ROTATE_ANGLE;
	}

	return ret;
}
#endif

static uint8_t pan_tilt_ctrl(stepper_motor_handle_t* handle){
	switch(handle->state){
	case STATE__STOP:
		if(handle->dispatched_steps){
			handle->remain_steps = handle->dispatched_steps;
			handle->dispatched_steps = 0;
			handle->half_period_index = handle->acc_dec_steps - 1;
			if(handle->status & STATUS__DISPATCHED_DIR_CCW){
				HAL_GPIO_WritePin(handle->dir_gpio_port, handle->dir_gpio_pin, GPIO_PIN_SET);
				handle->status |= STATUS__CURR_DIR_CCW;
			}else{
				HAL_GPIO_WritePin(handle->dir_gpio_port, handle->dir_gpio_pin, GPIO_PIN_RESET);
				handle->status &= ~STATUS__CURR_DIR_CCW;
			}
			handle->state = STATE__ACC;
		}
		break;

	case STATE__ACC:
	case STATE__CONST:
	case STATE__DEC:
	case STATE__TURN_AROUND:
		if(++handle->half_period_timer == handle->acc_dec_half_period_lookup[handle->half_period_index]){
			handle->half_period_timer = 0;
			if(handle->status & STATUS__STEP_HIGH){
				HAL_GPIO_WritePin(handle->step_gpio_port, handle->step_gpio_pin, GPIO_PIN_RESET);
				handle->status &= ~STATUS__STEP_HIGH;

				if(handle == &pan_motor_handle){
					if((handle->status) & STATUS__CURR_DIR_CCW){
						if(curr_pan_angle_in_steps){
							curr_pan_angle_in_steps--;
						}else{
							curr_pan_angle_in_steps = pan_steps_per_round;
						}
					}else{
						if(curr_pan_angle_in_steps < pan_steps_per_round){
							curr_pan_angle_in_steps++;
						}else{
							curr_pan_angle_in_steps = 0;
						}
					}
				}

#ifndef TILT_USE_ACCEL
				// Step counting mode for tilt
				if(handle == &tilt_motor_handle){
					if(handle->status & STATUS__CURR_TILT_UP){
						curr_tilt_angle += tilt_angle_per_step;
					}else{
						curr_tilt_angle -= tilt_angle_per_step;
					}
				}
#endif

				if(handle->remain_steps > 0){
					handle->remain_steps--;
				}

				if(handle->dispatched_steps){
					if(((handle->status & STATUS__DISPATCHED_DIR_CCW) && (handle->status & STATUS__CURR_DIR_CCW)) ||
						(!(handle->status & STATUS__DISPATCHED_DIR_CCW) && !(handle->status & STATUS__CURR_DIR_CCW))){
						handle->remain_steps = handle->dispatched_steps;
						handle->dispatched_steps = 0;
					}else{
						handle->state = STATE__TURN_AROUND;
					}
				}

				switch(handle->state){
				case STATE__ACC:
					if(handle->half_period_index > 0){
						handle->half_period_index--;
						if(handle->remain_steps > (handle->acc_dec_steps - handle->half_period_index)){
							handle->state = STATE__ACC;
						}else{
							handle->state = STATE__DEC;
						}
					}else{
						handle->state = STATE__CONST;
					}
					break;
				case STATE__CONST:
					if(handle->remain_steps > handle->acc_dec_steps){
						handle->state = STATE__CONST;
					}else{
						handle->state = STATE__DEC;
					}
					break;
				case STATE__DEC:
					if(handle->half_period_index < (handle->acc_dec_steps - 1)){
						handle->half_period_index++;
						if(handle->remain_steps < (handle->acc_dec_steps - handle->half_period_index)){
							handle->state = STATE__DEC;
						}else{
							handle->state = STATE__ACC;
						}
					}else{
						handle->state = STATE__STOP;
					}
					break;
				case STATE__TURN_AROUND:
					if(handle->half_period_index < (handle->acc_dec_steps - 1)){
						handle->half_period_index--;
						handle->dispatched_steps++;
					}else{
						handle->state = STATE__STOP;
					}
					break;
				default:
					break;
				}
			}else{
				HAL_GPIO_WritePin(handle->step_gpio_port, handle->step_gpio_pin, GPIO_PIN_SET);
				handle->status |= STATUS__STEP_HIGH;
			}
		}
		break;
	default:
		break;
	}

	return 0;
}
