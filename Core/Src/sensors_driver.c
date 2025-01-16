/*
 * sensors_driver.c
 *
     *  Created on: Oct 6, 2024
 *      Author: xana
 */

/* Includes ------------------------------------------------------------------*/
#include <assert.h>
#include "sensors_driver.h"
#include "string.h"
#include "stdbool.h"
#include "stddef.h"

/* Private typedef -----------------------------------------------------------*/

typedef union {
    struct {
        // LSB
        uint8_t RW: 1;
        uint8_t RA: 1;
        uint8_t SPONT: 1;
        uint8_t ECHOW: 1;
        uint8_t Reserved: 3;
        uint8_t DIFF: 1;
        // MSB
    };
    uint8_t as_uint8;
} SENSOR_FRAME_TYPE;

typedef struct {
    SENSOR_FRAME_TYPE TYPE;
    SENSOR_DEVICE DEVICE;
    uint8_t *data;
    uint8_t data_len;
} SENSOR_UART_FRAME;

#define RX_READER_BUFFER_LEN      (16u)

typedef enum {
    READER_STATE_IDLE,
    READER_STATE_READING_HEADER,
    READER_STATE_READING_DATA,
    READER_STATE_READING_FOOTER
} RX_READER_STATE;

// Rx State Machine Accumulator
typedef struct {
    RX_READER_STATE state;
    uint8_t bytes_left;
    uint8_t bytes_idx;
    uint8_t bytes[RX_READER_BUFFER_LEN];
} RX_READER;

/* Private define ------------------------------------------------------------*/

#define START_BYTE_VALUE            (0x8A)
#define STOP_BYTE_VALUE             (0x51)
#define UART_HEADER_LENGTH          (3u)
#define UART_FOOTER_LENGTH          (1u)
#define UART_OVERHEAD_LENGTH        (UART_HEADER_LENGTH + UART_FOOTER_LENGTH)
#define UART_MIN_FRAME_LENGTH       (UART_OVERHEAD_LENGTH)

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static RX_READER rx_reader = {0};

/* Global extern variables ---------------------------------------------------*/
// Need to have these functions defined somewhere else in the codebase

// ------------------ TX FUNCTIONS ------------------
extern bool device_send_uart(uint8_t *bytes, uint8_t bytes_len);

// ------------------ RX FUNCTIONS ------------------

#ifdef SENSOR_PLATFORM_SCREEN
// UART Callbacks
extern void sensor_cb_get_battery_level(uint8_t battery_level);
extern void sensor_cb_get_rfid(uint32_t rfid);
extern void sensor_cb_get_ir_telemeter(uint8_t ir_telem_id, uint8_t ir_telem_distance);
extern void sensor_cb_get_line_follower(uint8_t line_follow_id, bool line_detected);
extern void sensor_cb_get_set_led_value(bool set, uint8_t led_id, uint8_t led_value);
extern void sensor_cb_get_set_tool_selection(bool set, uint8_t tool_id);
extern void sensor_cb_get_set_robot_servo(bool set, uint8_t servo_id, uint8_t servo_angle);
extern void sensor_cb_get_set_arm_state(bool set, ARM_STATE state);
#ifdef UART_ENABLE_ERROR_FEEDBACK
extern void sensor_cb_error(bool spontaneous, uint8_t error);
#endif
#endif

// Hardware IO accessors
#ifdef SENSOR_PLATFORM_SENSOR
extern uint8_t sensor_hw_get_battery_level();
extern uint32_t sensor_hw_get_rfid();
extern uint8_t sensor_hw_read_ir_telemeter(uint8_t ir_telem_id);
extern bool sensor_hw_read_line_follower(uint8_t line_follow_id);
extern uint8_t sensor_hw_get_set_led_value(bool set, uint8_t led_id, uint8_t led_value);
extern uint8_t sensor_hw_get_set_tool_selection(bool set, uint8_t tool_id);
extern uint8_t sensor_hw_robot_get_set_servo(bool set, uint8_t servo_id, uint8_t servo_angle);
extern uint8_t sensor_hw_robot_get_set_state(bool set, ARM_STATE state);
#endif

/* Private function prototypes -----------------------------------------------*/

static uint8_t get_frame_length(SENSOR_UART_FRAME frame);
static uint8_t get_device_data_length(SENSOR_DEVICE dev);
static bool device_exists(SENSOR_DEVICE dev);
static bool valid_arm_state(ARM_STATE state);
static SENSOR_PARSING_ERROR uart_parse_frame(uint8_t *bytes, uint8_t bytes_len);
static void dispatch_uart_cb(SENSOR_UART_FRAME frame);
static SENSOR_PARSING_ERROR sensor_uart_parse_byte(uint8_t byte);
static void store_byte(uint8_t byte);

/* Private static functions ---------------------------------------------------------*/

static uint8_t get_frame_length(SENSOR_UART_FRAME frame) {
    return frame.data_len + UART_OVERHEAD_LENGTH;
}

static uint8_t get_device_data_length(SENSOR_DEVICE dev) {
    switch (dev) {
        case SENSOR_DEVICE_POWER:
            return 1;
        case SENSOR_DEVICE_RFID:
            return 4;
        case SENSOR_DEVICE_IR_TELEMETERS:
            return 2;
        case SENSOR_DEVICE_LINE_FOLLOWERS:
            return 2;
        case SENSOR_DEVICE_LEDS:
            return 2;
        case SENSOR_DEVICE_TOOL_TABLE:
            return 1;
        case SENSOR_DEVICE_SERVO_MOTORS:
            return 2;
        case SENSOR_DEVICE_ARM_STATE:
            return 1;
#ifdef UART_ENABLE_ERROR_FEEDBACK
        case SENSOR_DEVICE_ERRORS:
            return 1;
#endif
        default:
            return 0;
    }
}

static bool device_exists(SENSOR_DEVICE dev) {
    return get_device_data_length(dev) != 0xFF;
}

static bool valid_arm_state(ARM_STATE state) {
    switch (state) {
        case ARM_STATE_CUSTOM:
        case ARM_STATE_IDLE:
        case ARM_STATE_READY:
        case ARM_STATE_GRAB:
        case ARM_STATE_DISPLAY:
            return true;
        default:
            return false;
    }
}

// RX PARSER -----------------------

static void reset_reader() {
    memset(rx_reader.bytes, 0, sizeof rx_reader.bytes);
    rx_reader.bytes_left = 0;
    rx_reader.bytes_idx = 0;
    rx_reader.state = READER_STATE_IDLE;
}

static SENSOR_PARSING_ERROR uart_parse_frame(uint8_t *bytes, uint8_t bytes_len) {
    if (bytes == NULL) return SENSOR_PARSING_NULL_POINTER;
    if (bytes_len < UART_MIN_FRAME_LENGTH) return SENSOR_PARSING_INCOMPLETE_FRAME;
    if (bytes[0] != START_BYTE_VALUE) return SENSOR_PARSING_INVALID_FRAME_START;
    if (bytes[bytes_len - 1] != STOP_BYTE_VALUE) return SENSOR_PARSING_INVALID_FRAME_END;

    SENSOR_DEVICE dev = bytes[2];
    if (!device_exists(dev)) return SENSOR_PARSING_UNKNOWN_DEVICE;
    if ((bytes_len - UART_OVERHEAD_LENGTH) != get_device_data_length(dev)) {
        return SENSOR_PARSING_INCORRECT_PAYLOAD_LENGTH;
    }

    SENSOR_UART_FRAME frame = {
            .TYPE = {.as_uint8 = bytes[1]},
            .DEVICE = bytes[2],
            .data_len = get_device_data_length(dev),
            .data = &bytes[3]
    };
    dispatch_uart_cb(frame);
    return SENSOR_PARSING_OK;
}

static void dispatch_uart_cb(SENSOR_UART_FRAME frame) {
    SENSOR_COMM_OPTIONS options = {
            .is_write = frame.TYPE.RW,
            .is_answer = frame.TYPE.RA,
            .echo_write = frame.TYPE.ECHOW,
            .is_spont_answ = frame.TYPE.SPONT,
    };

    switch (frame.DEVICE) {
        case SENSOR_DEVICE_POWER: {
#ifdef SENSOR_PLATFORM_SCREEN
            if (options.is_answer) {
                sensor_cb_get_battery_level(frame.data[0]);
            }
#endif
#ifdef SENSOR_PLATFORM_SENSOR
            if (!options.is_answer && !options.is_write) {
                options.is_answer = true;
                uint8_t battery_level = sensor_hw_get_battery_level();
                uint8_t answer[1] = {battery_level};
                sensor_send_cmd(SENSOR_DEVICE_POWER, options, answer, 1);
            }
#endif
            break;
        }
        case SENSOR_DEVICE_RFID: {
#ifdef SENSOR_PLATFORM_SCREEN
            if (options.is_answer) {
                sensor_cb_get_rfid(frame.data[3] << 24 | frame.data[2] << 16 | frame.data[1] << 8 | frame.data[0]);
            }
#endif
#ifdef SENSOR_PLATFORM_SENSOR
            if (!options.is_answer && !options.is_write) {
                options.is_answer = true;
                uint32_t rfid = sensor_hw_get_rfid();
                uint8_t answer[4];
                answer[0] = (rfid >> 0) & 0xFF;
                answer[1] = (rfid >> 8) & 0xFF;
                answer[2] = (rfid >> 16) & 0xFF;
                answer[3] = (rfid >> 24) & 0xFF;
                sensor_send_cmd(SENSOR_DEVICE_RFID, options, answer, 4);
            }
#endif
            break;
        }
        case SENSOR_DEVICE_IR_TELEMETERS: {
#ifdef SENSOR_PLATFORM_SCREEN
            if (options.is_answer) {
                sensor_cb_get_ir_telemeter(frame.data[0], frame.data[1]);
            }
#endif
#ifdef SENSOR_PLATFORM_SENSOR
            if (!options.is_answer && !options.is_write) {
                options.is_answer = true;
                uint8_t ir_telem_id = frame.data[0];
                uint8_t ir_telem_distance = sensor_hw_read_ir_telemeter(ir_telem_id);
                uint8_t answer[2] = {ir_telem_id, ir_telem_distance};
                sensor_send_cmd(SENSOR_DEVICE_IR_TELEMETERS, options, answer, 2);
            }
#endif
            break;
        }

        case SENSOR_DEVICE_LINE_FOLLOWERS: {
#ifdef SENSOR_PLATFORM_SCREEN
            if (options.is_answer) {
                sensor_cb_get_line_follower(frame.data[0], frame.data[1] != 0);
            }
#endif
#ifdef SENSOR_PLATFORM_SENSOR
            if (!options.is_answer && !options.is_write) {
                options.is_answer = true;
                uint8_t line_follow_id = frame.data[0];
                bool line_follow_state = sensor_hw_read_line_follower(line_follow_id);
                uint8_t answer[2] = {line_follow_id,line_follow_state != 0};
                sensor_send_cmd(SENSOR_DEVICE_LINE_FOLLOWERS, options, answer, 2);
            }
#endif
            break;
        }
        case SENSOR_DEVICE_LEDS: {
#ifdef SENSOR_PLATFORM_SCREEN
            if (options.is_answer) {
                sensor_cb_get_set_led_value(options.is_write, frame.data[0], frame.data[1]);
            }
#endif
#ifdef SENSOR_PLATFORM_SENSOR
            if (!options.is_answer) {
                options.is_answer = true;
                uint8_t led_id = frame.data[0];
                uint8_t led_value = frame.data[1];
                led_value = sensor_hw_get_set_led_value(options.is_write, led_id, led_value);
                uint8_t answer[2] = {led_id, led_value};
                if (options.echo_write || !options.is_write) {
                    sensor_send_cmd(SENSOR_DEVICE_LEDS, options, answer, 2);
                }
            }
#endif
            break;
        }
        case SENSOR_DEVICE_TOOL_TABLE: {
#ifdef SENSOR_PLATFORM_SCREEN
            if (options.is_answer) {
                sensor_cb_get_set_tool_selection(options.is_write, frame.data[0]);
            }
#endif
#ifdef SENSOR_PLATFORM_SENSOR
            if (!options.is_answer) {
                options.is_answer = true;
                uint8_t tool_id = frame.data[0];
                tool_id = sensor_hw_get_set_tool_selection(options.is_write, tool_id);
                uint8_t answer[1] = {tool_id};
                if (options.echo_write || !options.is_write) {
                    sensor_send_cmd(SENSOR_DEVICE_TOOL_TABLE, options, answer, 1);
                }
            }
#endif
            break;
        }
        case SENSOR_DEVICE_SERVO_MOTORS: {
#ifdef SENSOR_PLATFORM_SCREEN
            if (options.is_answer) {
                sensor_cb_get_set_robot_servo(options.is_write, frame.data[0], frame.data[1]);
            }
#endif
#ifdef SENSOR_PLATFORM_SENSOR
            if (!options.is_answer) {
                options.is_answer = true;
                uint8_t servo_id = frame.data[0];
                uint8_t servo_angle = frame.data[1];
                servo_angle = sensor_hw_robot_get_set_servo(options.is_write, servo_id, servo_angle);
                uint8_t answer[2] = {servo_id, servo_angle};
                if (options.echo_write || !options.is_write) {
                    sensor_send_cmd(SENSOR_DEVICE_SERVO_MOTORS, options, answer, 2);
                }
            }
#endif
            break;
        }
        case SENSOR_DEVICE_ARM_STATE: {
#ifdef SENSOR_PLATFORM_SCREEN
            if (options.is_answer) {
                sensor_cb_get_set_arm_state(options.is_write, frame.data[0]);
            }
#endif
#ifdef SENSOR_PLATFORM_SENSOR
            if (!options.is_answer) {
                options.is_answer = true;
                ARM_STATE arm_state = frame.data[0];
                arm_state = sensor_hw_robot_get_set_state(options.is_write, arm_state);
                uint8_t answer[2] = {arm_state};
                if (options.echo_write || !options.is_write) {
                    sensor_send_cmd(SENSOR_DEVICE_ARM_STATE, options, answer, 1);
                }
            }
#endif
            break;
        }
#ifdef SENSOR_PLATFORM_SCREEN
#ifdef UART_ENABLE_ERROR_FEEDBACK
        case SENSOR_DEVICE_ERRORS: {
            uint8_t error_id = frame.data[0];
            sensor_cb_error(options.is_spont_answ, error_id);
            break;
        }
#endif
#endif
        default:
            break;
    }
}

static void store_byte(uint8_t byte) {
    rx_reader.bytes[rx_reader.bytes_idx] = byte;
    rx_reader.bytes_idx = (rx_reader.bytes_idx + 1) % RX_READER_BUFFER_LEN;
    if (rx_reader.bytes_left > 0) rx_reader.bytes_left--;
}

static SENSOR_PARSING_ERROR sensor_uart_parse_byte(uint8_t byte) {
    if (rx_reader.bytes_left > 0) {
        store_byte(byte);
    }

    switch (rx_reader.state) {
        case READER_STATE_IDLE: {
            if (rx_reader.bytes_left == 0 && byte == START_BYTE_VALUE) {
                reset_reader();
                rx_reader.state = READER_STATE_READING_HEADER;
                rx_reader.bytes_left = UART_HEADER_LENGTH;
                store_byte(byte);
            }
            break;
        }

        case READER_STATE_READING_HEADER:
            if (rx_reader.bytes_left == 0) {
                rx_reader.bytes_left = get_device_data_length(byte);
                rx_reader.state = READER_STATE_READING_DATA;
            }

            break;

        case READER_STATE_READING_DATA:
            if (rx_reader.bytes_left == 0) {
                rx_reader.bytes_left = UART_FOOTER_LENGTH;
                rx_reader.state = READER_STATE_READING_FOOTER;
            }
            break;

        case READER_STATE_READING_FOOTER:
            if (rx_reader.bytes_left == 0) {
                rx_reader.state = READER_STATE_IDLE;
                return uart_parse_frame(rx_reader.bytes, rx_reader.bytes_idx);
            }
            break;

        default:
            break;
    }
    return SENSOR_PARSING_OK;
}

/* Public functions ---------------------------------------------------------*/

SENSOR_ERROR sensor_send_cmd(SENSOR_DEVICE device, SENSOR_COMM_OPTIONS options, uint8_t *data, uint8_t data_len) {
    if (data == NULL) return SENSOR_ERROR_NULL_POINTER;
    if (data_len > SENSOR_UART_MAX_DATA_LENGTH) return SENSOR_ERROR_PAYLOAD_TOO_LONG;
    if (!device_exists(device)) return SENSOR_ERROR_UNKNOWN_DEVICE;

    SENSOR_UART_FRAME frame = {
            .DEVICE = device,
            .TYPE = {
                    .RW = options.is_write,
                    .RA = options.is_answer,
                    .SPONT = options.is_spont_answ,
                    .ECHOW = options.echo_write,
                    .DIFF = 1
            },
            .data = data,
            .data_len = data_len
    };

    uint8_t uart_data_len = get_frame_length(frame);
    uint8_t uart_data[uart_data_len];
    memset(uart_data, 0, sizeof uart_data);
    uart_data[0] = START_BYTE_VALUE;
    uart_data[1] = frame.TYPE.as_uint8;
    uart_data[2] = frame.DEVICE;
    memcpy(&uart_data[3], frame.data, frame.data_len);
    uart_data[uart_data_len - 1] = STOP_BYTE_VALUE;

    if (!device_send_uart(uart_data,  uart_data_len)) {
        return SENSOR_ERROR_UART_ERROR;
    }
    return SENSOR_ERROR_OK;
}

#ifdef SENSOR_PLATFORM_SCREEN
SENSOR_ERROR sensor_uart_get_set_tool(bool set, uint8_t tool_id) {
    if (SENSOR_MAX_TOOL_ID < tool_id) return SENSOR_ERROR_PAYLOAD_INVALID_VALUE;
    SENSOR_COMM_OPTIONS options = {
            .is_write = set,
            .is_answer = false,
            .is_spont_answ = false,
            .echo_write = set,
    };
    uint8_t data[] = {tool_id};
    assert(sizeof(data) == get_device_data_length(SENSOR_DEVICE_TOOL_TABLE));
    return sensor_send_cmd(SENSOR_DEVICE_TOOL_TABLE, options, data, sizeof data);
}
#endif

uint8_t sensor_format_led_value(uint8_t r, uint8_t g, uint8_t b) {
    return (0x3 & r) << 4 | (0x3 & g) << 2 | (0x3 & b);
}

#ifdef SENSOR_PLATFORM_SCREEN
SENSOR_ERROR sensor_uart_get_set_led(bool set, uint8_t led_id, uint8_t freq) {
    if (SENSOR_MAX_LED_ID < led_id) return SENSOR_ERROR_PAYLOAD_INVALID_VALUE;

    SENSOR_COMM_OPTIONS options = {
            .is_write = set,
            .is_answer = false,
            .is_spont_answ = false,
            .echo_write = set,
    };
    uint8_t data[] = {led_id, freq};
    assert(sizeof(data) == get_device_data_length(SENSOR_DEVICE_LEDS));
    return sensor_send_cmd(SENSOR_DEVICE_LEDS, options, data, sizeof data);
}
#endif

#ifdef SENSOR_PLATFORM_SCREEN
SENSOR_ERROR sensor_uart_get_set_robot_servo(bool set, uint8_t motor_id, uint8_t position) {
    if (SENSOR_MAX_SERVO_ID < motor_id) return SENSOR_ERROR_PAYLOAD_INVALID_VALUE;

    SENSOR_COMM_OPTIONS options = {
            .is_write = set,
            .is_answer = false,
            .is_spont_answ = false,
            .echo_write = set,
    };
    uint8_t data[] = {motor_id, position};
    uint8_t data_len = get_device_data_length(SENSOR_DEVICE_SERVO_MOTORS);
    assert(sizeof(data) == get_device_data_length(SENSOR_DEVICE_SERVO_MOTORS));
    return sensor_send_cmd(SENSOR_DEVICE_SERVO_MOTORS, options, data, data_len);
}
#endif


#ifdef SENSOR_PLATFORM_SCREEN
SENSOR_ERROR sensor_uart_get_set_arm_state(bool set, ARM_STATE state) {
    if (!valid_arm_state(state)) return SENSOR_ERROR_PAYLOAD_INVALID_VALUE;

    SENSOR_COMM_OPTIONS options = {
            .is_write = set,
            .is_answer = false,
            .is_spont_answ = false,
            .echo_write = set,
    };
    uint8_t data[] = {state};
    uint8_t data_len = get_device_data_length(SENSOR_DEVICE_ARM_STATE);
    assert(sizeof(data) == get_device_data_length(SENSOR_DEVICE_ARM_STATE));
    return sensor_send_cmd(SENSOR_DEVICE_ARM_STATE, options, data, data_len);
}
#endif

#ifdef SENSOR_PLATFORM_SCREEN
SENSOR_ERROR sensor_uart_query_battery_level() {
    SENSOR_COMM_OPTIONS options = {
            .is_write = false,
            .is_answer = false,
            .is_spont_answ = false,
            .echo_write = false,
    };
    uint8_t data[] = {0xFF};
    assert(sizeof(data) == get_device_data_length(SENSOR_DEVICE_POWER));
    return sensor_send_cmd(SENSOR_DEVICE_POWER, options, data, sizeof data);
}
#endif

#ifdef SENSOR_PLATFORM_SCREEN
SENSOR_ERROR query_uart_query_ir_telemeters(uint8_t ir_telem_id) {
    if (SENSOR_MAX_IR_TELEM_ID < ir_telem_id) return SENSOR_ERROR_PAYLOAD_INVALID_VALUE;
    SENSOR_COMM_OPTIONS options = {
            .is_write = false,
            .is_answer = false,
            .is_spont_answ = false,
            .echo_write = false,
    };
    uint8_t data[] = {ir_telem_id, 0xFF};
    assert(sizeof(data) == get_device_data_length(SENSOR_DEVICE_IR_TELEMETERS));
    return sensor_send_cmd(SENSOR_DEVICE_IR_TELEMETERS, options, data, sizeof data);
}
#endif

#ifdef SENSOR_PLATFORM_SCREEN
SENSOR_ERROR query_uart_query_line_follower(uint8_t line_follow_id) {
    if (SENSOR_MAX_LINE_FOLLOW_ID < line_follow_id) return SENSOR_ERROR_PAYLOAD_INVALID_VALUE;
    SENSOR_COMM_OPTIONS options = {
            .is_write = false,
            .is_answer = false,
            .is_spont_answ = false,
            .echo_write = false,
    };
    uint8_t data[] = {line_follow_id, 0xFF};
    assert(sizeof(data) == get_device_data_length(SENSOR_DEVICE_LINE_FOLLOWERS));
    return sensor_send_cmd(SENSOR_DEVICE_LINE_FOLLOWERS, options, data, sizeof data);
}
#endif

#ifdef SENSOR_PLATFORM_SCREEN
SENSOR_ERROR sensor_uart_query_rfid() {
    SENSOR_COMM_OPTIONS options = {
            .is_write = false,
            .is_answer = false,
            .is_spont_answ = false,
            .echo_write = false,
    };
    uint8_t data[] = {0xFF, 0xFF, 0xFF, 0xFF};
    assert(sizeof(data) == get_device_data_length(SENSOR_DEVICE_RFID));
    return sensor_send_cmd(SENSOR_DEVICE_RFID, options, data, sizeof data);
}
#endif

// RX Functions
SENSOR_PARSING_ERROR sensor_uart_rx_cb(uint8_t *bytes, size_t bytes_len) {
    SENSOR_PARSING_ERROR status = SENSOR_PARSING_OK;
    for (size_t i=0; i<bytes_len; i++) {
        uint8_t byte = bytes[i];
        status = sensor_uart_parse_byte(byte);
        if (status != SENSOR_PARSING_OK) return status;
    }
    return SENSOR_PARSING_OK;
}



