/*
 * h_bridge_driver.h
 *
 *  Created on: Sep 26, 2024
 *      Author: xana
 */

#ifndef INC_H_BRIDGE_DRIVER_H_
#define INC_H_BRIDGE_DRIVER_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// Toggle which platform is this library on
#define SENSOR_PLATFORM_SENSOR
//#define SENSOR_PLATFORM_SCREEN
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#if !defined(SENSOR_PLATFORM_SCREEN) && !defined(SENSOR_PLATFORM_SENSOR)
#error "Please define which platform this library is used with using the above define statements"
#elif defined(SENSOR_PLATFORM_SCREEN) && defined(SENSOR_PLATFORM_SENSOR)
#warning "Please define only one platform, only test codes should enable both"
#endif

/* Public typedef -----------------------------------------------------------*/

/*
 * Type of sensor to address
 */
typedef enum {
    SENSOR_DEVICE_POWER = 0x01,
    SENSOR_DEVICE_RFID = 0x02,
    SENSOR_DEVICE_IR_TELEMETERS = 0x03,
    SENSOR_DEVICE_LINE_FOLLOWERS = 0x04,
    SENSOR_DEVICE_LEDS = 0x81,
    SENSOR_DEVICE_TOOL_TABLE = 0x83,
    SENSOR_DEVICE_SERVO_MOTORS = 0x84,
    SENSOR_DEVICE_ARM_STATE = 0x85,
#ifdef UART_ENABLE_ERROR_FEEDBACK
SENSOR_DEVICE_ERRORS = 0xFF
#endif
} SENSOR_DEVICE;

typedef enum {
    ARM_STATE_CUSTOM = 0x00,
    ARM_STATE_IDLE = 0x01,
    ARM_STATE_READY = 0x02,
    ARM_STATE_GRAB = 0x04,
    ARM_STATE_DISPLAY = 0x08
} ARM_STATE;

typedef struct {
    bool is_write;
    bool is_answer;
    bool is_spont_answ;
    bool echo_write;
} SENSOR_COMM_OPTIONS;

typedef enum {
    SENSOR_ERROR_OK,
    SENSOR_ERROR_UART_ERROR,
    SENSOR_ERROR_UNKNOWN_DEVICE,
    SENSOR_ERROR_PAYLOAD_TOO_LONG,
    SENSOR_ERROR_PAYLOAD_INVALID_VALUE,
    SENSOR_ERROR_NULL_POINTER
} SENSOR_ERROR;

typedef enum {
    SENSOR_PARSING_OK,
    SENSOR_PARSING_NULL_POINTER,
    SENSOR_PARSING_INVALID_FRAME_START,
    SENSOR_PARSING_INVALID_FRAME_END,
    SENSOR_PARSING_INCOMPLETE_FRAME,
    SENSOR_PARSING_INCORRECT_PAYLOAD_LENGTH,
    SENSOR_PARSING_UNKNOWN_DEVICE
} SENSOR_PARSING_ERROR;

// ------------------ MASTER/CONTROLLER FUNCTIONS ------------------


/* Public define ------------------------------------------------------------*/

#define SENSOR_UART_MAX_DATA_LENGTH         (8u)
#define SENSOR_MAX_TOOL_ID                  (7u)
#define SENSOR_MAX_LED_ID                   (16u)
#define SENSOR_MAX_SERVO_ID                 (3u)
#define SENSOR_MAX_IR_TELEM_ID              (4u)
#define SENSOR_MAX_LINE_FOLLOW_ID           (3u)

/* Public macro -------------------------------------------------------------*/

/* Public variables ---------------------------------------------------------*/

/* Global extern variables ---------------------------------------------------*/

/* Public function prototypes -----------------------------------------------*/

uint8_t sensor_format_led_value(uint8_t r, uint8_t g, uint8_t b);

SENSOR_ERROR sensor_send_cmd(SENSOR_DEVICE device, SENSOR_COMM_OPTIONS options, uint8_t *data, uint8_t data_len);

#ifdef SENSOR_PLATFORM_SCREEN
SENSOR_ERROR sensor_uart_query_rfid();
SENSOR_ERROR sensor_uart_query_battery_level();
SENSOR_ERROR query_uart_query_ir_telemeters(uint8_t ir_telem_id);
SENSOR_ERROR query_uart_query_line_follower(uint8_t line_follow_id);
SENSOR_ERROR sensor_uart_get_set_tool(bool set, uint8_t tool_id);
SENSOR_ERROR sensor_uart_get_set_led(bool set,uint8_t led_id, uint8_t value);
SENSOR_ERROR sensor_uart_get_set_robot_servo(bool set, uint8_t motor_id, uint8_t position);
SENSOR_ERROR sensor_uart_get_set_arm_state(bool set, ARM_STATE state);
#endif

SENSOR_PARSING_ERROR sensor_uart_rx_cb(uint8_t *bytes, size_t bytes_len);

#endif /* INC_H_BRIDGE_DRIVER_H_ */
