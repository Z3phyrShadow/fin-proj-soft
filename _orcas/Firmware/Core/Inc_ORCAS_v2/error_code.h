/*
 * error_code.h
 *
 *  ORCAS v2 – error / ACK codes.
 *  AEG / radar error codes are retained for binary compatibility but will
 *  never be emitted in a build without FIRE_CTRL / TOF_SENSORS_CTRL.
 */

#ifndef INC_ERROR_CODE_H_
#define INC_ERROR_CODE_H_

/* ── UART framing errors ─────────────────────────────────────────────────── */
#define ERR__INVALID_UART_CMD_CODE         0x01
#define ERR__INVALID_UART_CMD_CHECKSUM     0x02

/* ── Motor / orientation errors ──────────────────────────────────────────── */
#define ERR__MMA845X_COMM_FAIL             0x10  /* accelerometer I2C fail   */
#define ERR__TILT_MOTOR_STEP_INIT_FAIL     0x11
#define ERR__TILT_ENDSTOP_REACHED          0x12
#define ERR__PAN_MOTOR_STEP_INIT_FAIL      0x13

/* ── Legacy codes (unused in v2 build, kept for protocol compatibility) ───  */
#define ERR__RADAR_MOTOR_STEP_INIT_FAIL    0x20
#define ERR__GY_US42_COMM_FAIL             0x21
#define ERR__GY_TOF10M_INIT_FAIL           0x22
#define ERR__GY_TOF10M_COMM_FAIL           0x23
#define ERR__AEG_PISTON_PULLING_TIMEOUT    0x30

#endif /* INC_ERROR_CODE_H_ */
