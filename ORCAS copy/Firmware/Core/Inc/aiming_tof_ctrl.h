/*
 * aiming_tof_ctrl.h
 *
 *  ORCAS v2 – TOF Distance Tracking.
 *  Uses GY_TOF10M sensor on I2C1 to track distance.
 */

#ifndef INC_AIMING_TOF_CTRL_H_
#define INC_AIMING_TOF_CTRL_H_

#define AIMING_TOF_CTRL_MIN_PERIOD_US	10000

uint8_t aiming_tof_ctrl_init();
uint8_t aiming_tof_ctrl_cmds_dispatch(uint8_t* cmd_buf, uint8_t* ack_buf);
uint8_t aiming_tof_ctrl_perodic_routines();

#endif /* INC_AIMING_TOF_CTRL_H_ */
