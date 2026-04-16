/*
 * uart_cmds_handle.c
 *
 *  ORCAS v2 – UART command dispatch.
 */

#include <string.h>
#include "main.h"
#include "error_code.h"
#include "orientation_ctrl_utils.h"
#include "uart_cmds_handle.h"
#ifdef AIMING_TOF_CTRL
#include "aiming_tof_ctrl.h"
#endif

uint8_t uart_rx_index = 0;
#define UART_RX_BUF_SIZE	32
uint8_t uart_rx_buf[UART_RX_BUF_SIZE] = {0};

#define CMD_RECIVING_STATE__HEADER		0
#define CMD_RECIVING_STATE__CODE		1
#define CMD_RECIVING_STATE__PARAMETERS	2
#define CMD_RECIVING_STATE__CHECKSUM	3
uint8_t cmd_receiving_state = CMD_RECIVING_STATE__HEADER;

uint8_t cmd_params_length[] = {0,
                               CMD_PARAMS_LENGTH__GET_FW_VER,
                               0, // GET_FIRE_CTRL_STATUS
                               0, // SET_FIRE_COUNT
                               0, // SET_FIRE_CTRL_CONFIG
                               0, // SET_FIRE_SAFETY
                               0, // SET_AMMO_FEEDER_CONFIG
                               0, // SET_AMMO_FEEDING
                               CMD_PARAMS_LENGTH__GET_PAN_TILT_STEP_ANGLE,
                               CMD_PARAMS_LENGTH__GET_PAN_TILT_CURR_ANGLE,
                               CMD_PARAMS_LENGTH__SET_PAN_TILT_ROTATE_ANGLE,
                               0, // GET_TOF_SENSROS_CTRL_STATUS
                               0, // SET_RADAR_EN
                               0, // SET_RADAR_CONFIG
                               0, // GET_RADAR_RANGING
                               0, // GET_AIMING_DISTANCE
                               CMD_PARAMS_LENGTH__SET_SEARCHLIGHT_PWM};

#define UART_TX_BUF_SIZE	64
uint8_t uart_tx_buf[UART_TX_BUF_SIZE] = {0};

#define UART_CMDS_WATCHDOG_TIMEOUT_MS	1000
uint16_t uart_cmds_watchdog_timeout = 0;

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim3;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	uint8_t i = 0, j = 0;
	uint8_t checksum = 0;

	if(huart == &huart1){
		switch(cmd_receiving_state){
		case CMD_RECIVING_STATE__HEADER:
			if(uart_rx_buf[CMD_HEADER_INDEX] == CMD_HEADER_CHAR){
				cmd_receiving_state = CMD_RECIVING_STATE__CODE;
				uart_rx_index++;
				uart_cmds_watchdog_timeout = 0;
			}else{
				uart_rx_index = 0;
			}
			break;
		case CMD_RECIVING_STATE__CODE:
			switch(uart_rx_buf[CMD_CODE_INDEX]){
			case CMD_CODE__GET_FW_VER:
			case CMD_CODE__GET_PAN_TILT_CURR_ANGLE:
			case CMD_CODE__GET_PAN_TILT_STEP_ANGLE:
			case CMD_CODE__GET_AIMING_DISTANCE:
				uart_rx_index++;
				cmd_receiving_state = CMD_RECIVING_STATE__CHECKSUM;
				break;
			case CMD_CODE__SET_PAN_TILT_ROTATE_ANGLE:
			case CMD_CODE__SET_SEARCHLIGHT_PWM:
				uart_rx_index++;
				cmd_receiving_state = CMD_RECIVING_STATE__PARAMETERS;
				break;
			default:
				memset(uart_tx_buf, 0, UART_TX_BUF_SIZE);
				checksum = 0;
				i = 0;
				uart_tx_buf[i++] = ACK_HEADER_CHAR;
				uart_tx_buf[i++] = ERR__INVALID_UART_CMD_CODE;
				for(j = 0; j < i; j++){checksum += uart_tx_buf[j];}
				uart_tx_buf[i++] = checksum;
				HAL_UART_Transmit_IT(&huart1, uart_tx_buf, i);
				uart_rx_index = 0;
				cmd_receiving_state = CMD_RECIVING_STATE__HEADER;
				break;
			}
			break;
		case CMD_RECIVING_STATE__PARAMETERS:
			uart_rx_index++;
			if(uart_rx_index == CMD_PARAMS_INDEX + cmd_params_length[uart_rx_buf[CMD_CODE_INDEX]]){
				cmd_receiving_state = CMD_RECIVING_STATE__CHECKSUM;
			}
			break;
		case CMD_RECIVING_STATE__CHECKSUM:
			for(i = 0; i < uart_rx_index; i++){
				checksum += uart_rx_buf[i];
			}
			if(checksum == uart_rx_buf[uart_rx_index]){
				switch(uart_rx_buf[CMD_CODE_INDEX]){
				case CMD_CODE__GET_FW_VER:
				case CMD_CODE__GET_PAN_TILT_STEP_ANGLE:
				case CMD_CODE__GET_PAN_TILT_CURR_ANGLE:
				case CMD_CODE__GET_AIMING_DISTANCE:
					memset(uart_tx_buf, 0, UART_TX_BUF_SIZE);
					checksum = 0;
					i = 0;
					uart_tx_buf[i++] = ACK_HEADER_CHAR;
					switch(uart_rx_buf[CMD_CODE_INDEX]){
					case CMD_CODE__GET_FW_VER:
						uart_tx_buf[i] = ACK_CODE__SUCCESS;
						memcpy(&uart_tx_buf[i + 1], FW_VER, cmd_params_length[CMD_CODE__GET_FW_VER]);
						break;
					case CMD_CODE__GET_PAN_TILT_STEP_ANGLE:
					case CMD_CODE__GET_PAN_TILT_CURR_ANGLE:
						orientation_ctrl_cmds_dispatch(uart_rx_buf, uart_tx_buf);
						break;
					case CMD_CODE__GET_AIMING_DISTANCE:
#ifdef AIMING_TOF_CTRL
						aiming_tof_ctrl_cmds_dispatch(uart_rx_buf, uart_tx_buf);
#endif
						break;
					default:
						break;
					}
					i = i + 1 + cmd_params_length[uart_rx_buf[CMD_CODE_INDEX]];
					for(j = 0; j < i; j++){checksum += uart_tx_buf[j];}
					uart_tx_buf[i++] = checksum;
					HAL_UART_Transmit_IT(&huart1, uart_tx_buf, i);
					uart_rx_index = 0;
					cmd_receiving_state = CMD_RECIVING_STATE__HEADER;
					break;
				case CMD_CODE__SET_PAN_TILT_ROTATE_ANGLE:
				case CMD_CODE__SET_SEARCHLIGHT_PWM:
					memset(uart_tx_buf, 0, UART_TX_BUF_SIZE);
					checksum = 0;
					i = 0;
					uart_tx_buf[i++] = ACK_HEADER_CHAR;

					switch(uart_rx_buf[CMD_CODE_INDEX]){
					case CMD_CODE__SET_PAN_TILT_ROTATE_ANGLE:
						orientation_ctrl_cmds_dispatch(uart_rx_buf, uart_tx_buf);
						break;
					case CMD_CODE__SET_SEARCHLIGHT_PWM:
						if(uart_rx_buf[CMD_PARAMS_INDEX] <= 100){
							TIM_OC_InitTypeDef sConfigOC = {0};
							sConfigOC.OCMode = TIM_OCMODE_PWM1;
							sConfigOC.Pulse = uart_rx_buf[CMD_PARAMS_INDEX];
							sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
							sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
							HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);
							HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
						}
						uart_tx_buf[i] = ACK_CODE__SUCCESS;
						break;
					default:
						break;
					}
					i++; // ACK_CODE_INDEX

					for(j = 0; j < i; j++){checksum += uart_tx_buf[j];}
					uart_tx_buf[i++] = checksum;
					HAL_UART_Transmit_IT(&huart1, uart_tx_buf, i);
					uart_rx_index = 0;
					cmd_receiving_state = CMD_RECIVING_STATE__HEADER;
					break;
				default:
					break;
				}
			}else{
				memset(uart_tx_buf, 0, UART_TX_BUF_SIZE);
				checksum = 0;
				i = 0;
				uart_tx_buf[i++] = ACK_HEADER_CHAR;
				uart_tx_buf[i++] = ERR__INVALID_UART_CMD_CHECKSUM;
				for(j = 0; j < i; j++){checksum += uart_tx_buf[j];}
				uart_tx_buf[i++] = checksum;
				HAL_UART_Transmit_IT(&huart1, uart_tx_buf, i);
				uart_rx_index = 0;
				cmd_receiving_state = CMD_RECIVING_STATE__HEADER;
				break;
			}
			break;
		default:
			break;
		}
		HAL_UART_Receive_IT(&huart1, &uart_rx_buf[uart_rx_index], 1);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	if(huart == &huart1){
		if((huart1.ErrorCode & HAL_UART_ERROR_ORE)){
			READ_REG(huart1.Instance->SR);
			READ_REG(huart1.Instance->DR);
		}
	}
}

uint8_t uart_cmds_handle_perodic_routines(){
	uart_cmds_watchdog_timeout += (UART_CMDS_HANDLE_ROUTINES_MIN_PERIOD_US / 1000);
	if(uart_cmds_watchdog_timeout >= UART_CMDS_WATCHDOG_TIMEOUT_MS){
		uart_rx_index = 0;
		HAL_UART_Receive_IT(&huart1, &uart_rx_buf[uart_rx_index], 1);
		cmd_receiving_state = CMD_RECIVING_STATE__HEADER;
		uart_cmds_watchdog_timeout = 0;
	}
	return 0;
}
