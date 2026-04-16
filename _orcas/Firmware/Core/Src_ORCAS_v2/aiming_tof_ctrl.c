/*
 * aiming_tof_ctrl.c
 *
 *  ORCAS v2 – TOF Distance Tracking.
 *  Uses GY_TOF10M sensor on I2C1 to track distance.
 */

#include "main.h"
#include "error_code.h"
#include "uart_cmds_handle.h"
#include "aiming_tof_ctrl.h"
#include "gy_tof10m.h"

#ifdef AIMING_TOF_CTRL

extern I2C_HandleTypeDef hi2c1;

#define AIMING_TOF_CTRL_ROUTINES_PERIOD_US 100000

uint16_t aiming_tof_perodic_routines_timer = 0;

#define STATUS__GY_TOF10M_INIT_FAIL			0x01
#define STATUS__GY_TOF10M_COMM_FAIL			0x02
uint8_t aiming_tof_ctrl_status = 0;

uint8_t aiming_distance = 0;

static uint8_t get_aiming_distance();

uint8_t aiming_tof_ctrl_init(){
	uint8_t ret = 0;

	ret = gy_tof10m__init(&hi2c1);
	if(ret){
		aiming_tof_ctrl_status |= STATUS__GY_TOF10M_INIT_FAIL;
		return ERR__GY_TOF10M_INIT_FAIL;
	}

	return 0;
}

uint8_t aiming_tof_ctrl_cmds_dispatch(uint8_t* cmd_buf, uint8_t* ack_buf){
	switch(cmd_buf[CMD_CODE_INDEX]){
	case CMD_CODE__GET_AIMING_DISTANCE:
		if(aiming_tof_ctrl_status & STATUS__GY_TOF10M_INIT_FAIL){
			ack_buf[ACK_CODE_INDEX] = ERR__GY_TOF10M_INIT_FAIL;
		}else if(aiming_tof_ctrl_status & STATUS__GY_TOF10M_COMM_FAIL){
			aiming_tof_ctrl_status &= ~STATUS__GY_TOF10M_COMM_FAIL;
			ack_buf[ACK_CODE_INDEX] = ERR__GY_TOF10M_COMM_FAIL;
		}else{
			ack_buf[ACK_CODE_INDEX] = ACK_CODE__SUCCESS;
		}
		ack_buf[ACK_PARAMS_INDEX] = aiming_distance;
		break;
	default:
		break;
	}

	return 0;
}

uint8_t aiming_tof_ctrl_perodic_routines(){
	uint8_t ret;

	aiming_tof_perodic_routines_timer++;

	if(aiming_tof_perodic_routines_timer % (AIMING_TOF_CTRL_ROUTINES_PERIOD_US / AIMING_TOF_CTRL_MIN_PERIOD_US) == 0){
		aiming_tof_perodic_routines_timer = 0;
		ret = get_aiming_distance();
		if(ret & STATUS__GY_TOF10M_COMM_FAIL){
			MX_I2C_ForceClearBusyFlag(&hi2c1, I2C1_SDA_GPIO_Port, I2C1_SDA_Pin, I2C1_SCL_GPIO_Port, I2C1_SCL_Pin);
		}
	}

	return 0;
}

static uint8_t get_aiming_distance(){
	uint16_t range;

	if(gy_tof10m__get_range(&hi2c1, &range)){
		aiming_tof_ctrl_status |= STATUS__GY_TOF10M_COMM_FAIL;
		return STATUS__GY_TOF10M_COMM_FAIL;
	}

	aiming_distance = (range / 10) / 4;
	return 0;
}

#endif // AIMING_TOF_CTRL
