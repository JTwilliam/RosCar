#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <stdint.h>

/*
 * 机器人公共数据类型。
 *
 * 这里的结构体用于任务队列和状态快照，不能直接强制转换成字节流发送。
 * STM32 与树莓派之间的线协议必须在 RobotProtocol/SerialTask 中按字段、
 * 按小端顺序显式序列化，避免编译器填充和 ABI 差异造成解析错误。
 */

/* 以下长度限制针对 COBS 编码前的解码数据。 */
#define ROBOT_PROTOCOL_VERSION                 1U
#define ROBOT_PROTOCOL_MAX_PAYLOAD             128U
#define ROBOT_PROTOCOL_MAX_PAYLOAD_BYTES       ROBOT_PROTOCOL_MAX_PAYLOAD
#define ROBOT_PROTOCOL_MAX_DECODED_FRAME_BYTES 256U

#define ROBOT_COMMAND_TIMEOUT_DEFAULT_MS 200U
#define ROBOT_COMMAND_TIMEOUT_MIN_MS      50U
#define ROBOT_COMMAND_TIMEOUT_MAX_MS      1000U

#define ROBOT_HEALTH_STACK_COUNT 5U

typedef uint8_t RobotHealthCounter_t;

#define ROBOT_HEALTH_COUNTER_UART_FRAMING       ((RobotHealthCounter_t)0U)
#define ROBOT_HEALTH_COUNTER_UART_CRC           ((RobotHealthCounter_t)1U)
#define ROBOT_HEALTH_COUNTER_UART_OVERFLOW      ((RobotHealthCounter_t)2U)
#define ROBOT_HEALTH_COUNTER_UART_RESTART       ((RobotHealthCounter_t)3U)
#define ROBOT_HEALTH_COUNTER_I2C_TIMEOUT        ((RobotHealthCounter_t)4U)
#define ROBOT_HEALTH_COUNTER_I2C_NACK           ((RobotHealthCounter_t)5U)
#define ROBOT_HEALTH_COUNTER_COMMAND_TIMEOUT    ((RobotHealthCounter_t)6U)
#define ROBOT_HEALTH_COUNTER_ARM_REJECTION      ((RobotHealthCounter_t)7U)
#define ROBOT_HEALTH_COUNTER_PS2_ESTOP          ((RobotHealthCounter_t)8U)
#define ROBOT_HEALTH_COUNTER_BOOTLOADER_REJECT  ((RobotHealthCounter_t)9U)
#define ROBOT_HEALTH_COUNTER_TASK_HEARTBEAT_MISS ((RobotHealthCounter_t)10U)
#define ROBOT_HEALTH_COUNTER_COUNT              ((RobotHealthCounter_t)11U)

typedef uint8_t SafetyState_t;

/* 安全状态是运动许可的唯一依据，FAULT 和急停状态均禁止执行运动命令。 */
#define SAFETY_STARTUP       ((SafetyState_t)0U)
#define SAFETY_NORMAL        ((SafetyState_t)1U)
#define SAFETY_STOPPING      ((SafetyState_t)2U)
#define SAFETY_SAFE_IDLE     ((SafetyState_t)3U)
#define SAFETY_ESTOP_LATCHED ((SafetyState_t)4U)
#define SAFETY_ESTOP_READY   ((SafetyState_t)5U)
#define SAFETY_BOOT_PENDING  ((SafetyState_t)6U)
#define SAFETY_FAULT         ((SafetyState_t)7U)

typedef uint8_t RobotMessageType_t;

#define ROBOT_MSG_HELLO_QUERY      ((RobotMessageType_t)0x01U)
#define ROBOT_MSG_STATE_QUERY      ((RobotMessageType_t)0x02U)
#define ROBOT_MSG_CMD_VELOCITY     ((RobotMessageType_t)0x10U)
#define ROBOT_MSG_CMD_ARM          ((RobotMessageType_t)0x11U)
#define ROBOT_MSG_STOP_ALL         ((RobotMessageType_t)0x12U)
#define ROBOT_MSG_ENTER_BOOTLOADER ((RobotMessageType_t)0x13U)
#define ROBOT_MSG_HELLO_STATE      ((RobotMessageType_t)0x80U)
#define ROBOT_MSG_CHASSIS_STATE    ((RobotMessageType_t)0x81U)
#define ROBOT_MSG_ARM_STATE        ((RobotMessageType_t)0x82U)
#define ROBOT_MSG_HEALTH_STATE     ((RobotMessageType_t)0x83U)
#define ROBOT_MSG_EVENT_STATE      ((RobotMessageType_t)0x84U)
#define ROBOT_MSG_ACK              ((RobotMessageType_t)0xF0U)
#define ROBOT_MSG_NACK             ((RobotMessageType_t)0xF1U)

typedef uint8_t RobotAckCode_t;

#define ROBOT_ACK_OK                      ((RobotAckCode_t)0U)
#define ROBOT_ACK_BUSY                    ((RobotAckCode_t)1U)
#define ROBOT_ACK_INVALID_PAYLOAD         ((RobotAckCode_t)2U)
#define ROBOT_ACK_BUSY_NOT_SAFE           ((RobotAckCode_t)3U)
#define ROBOT_ACK_STOP_HANDSHAKE_REQUIRED ((RobotAckCode_t)4U)
#define ROBOT_ACK_ESTOP_LATCHED           ((RobotAckCode_t)5U)
#define ROBOT_ACK_DEVICE_FAULT            ((RobotAckCode_t)6U)

typedef uint16_t RobotFaultFlags_t;

/* 故障位采用锁存累积方式，同一次运行期间不会自动清除历史故障。 */
#define ROBOT_FAULT_NONE            ((RobotFaultFlags_t)0x0000U)
#define ROBOT_FAULT_COMMAND_TIMEOUT ((RobotFaultFlags_t)0x0001U)
#define ROBOT_FAULT_SERIAL_OVERFLOW ((RobotFaultFlags_t)0x0002U)
#define ROBOT_FAULT_PROTOCOL         ((RobotFaultFlags_t)0x0004U)
#define ROBOT_FAULT_ESTOP            ((RobotFaultFlags_t)0x0008U)
#define ROBOT_FAULT_MOTOR            ((RobotFaultFlags_t)0x0010U)
#define ROBOT_FAULT_ARM              ((RobotFaultFlags_t)0x0020U)
#define ROBOT_FAULT_HEALTH           ((RobotFaultFlags_t)0x0040U)
#define ROBOT_FAULT_INTERNAL         ((RobotFaultFlags_t)0x8000U)

typedef uint16_t Ps2ButtonMask_t;

#define PS2_BUTTON_TRIANGLE ((Ps2ButtonMask_t)0x0001U)
#define PS2_BUTTON_CIRCLE   ((Ps2ButtonMask_t)0x0002U)
#define PS2_BUTTON_CROSS    ((Ps2ButtonMask_t)0x0004U)
#define PS2_BUTTON_SQUARE   ((Ps2ButtonMask_t)0x0008U)
#define PS2_BUTTON_L1       ((Ps2ButtonMask_t)0x0010U)
#define PS2_BUTTON_R1       ((Ps2ButtonMask_t)0x0020U)
#define PS2_BUTTON_L2       ((Ps2ButtonMask_t)0x0040U)
#define PS2_BUTTON_R2       ((Ps2ButtonMask_t)0x0080U)
#define PS2_BUTTON_SELECT   ((Ps2ButtonMask_t)0x0100U)
#define PS2_BUTTON_START    ((Ps2ButtonMask_t)0x0200U)
#define PS2_BUTTON_L3       ((Ps2ButtonMask_t)0x0400U)
#define PS2_BUTTON_R3       ((Ps2ButtonMask_t)0x0800U)
#define PS2_BUTTON_MODE     ((Ps2ButtonMask_t)0x1000U)

typedef struct {
    int16_t vx_mm_s;
    int16_t vy_mm_s;
    int16_t wz_mrad_s;
    uint16_t timeout_ms;
    uint16_t sequence;
} ChassisCommand_t;

typedef struct {
    uint32_t timestamp_ms;
    int32_t encoder[4];
    int8_t motor_command[4];
    uint16_t battery_mv;
    uint16_t fault_flags;
    uint8_t safety_state;
    uint8_t reserved;
    uint16_t sequence;
} ChassisState_t;

typedef struct {
    uint8_t command;
    uint8_t length;
    uint8_t payload[24];
    uint16_t sequence;
} ArmCommand_t;

typedef struct {
    uint32_t timestamp_ms;
    int16_t joint_position[6];
    uint16_t fault_flags;
    uint8_t active_command;
    uint8_t valid_mask;
    uint16_t sequence;
} ArmState_t;

typedef struct {
    uint32_t timestamp_ms;
    uint32_t minimum_free_heap_bytes;
    uint16_t stack_free_words[ROBOT_HEALTH_STACK_COUNT];
    uint16_t counters[ROBOT_HEALTH_COUNTER_COUNT];
    uint16_t fault_flags;
    SafetyState_t safety_state;
    uint8_t reserved;
    /* Extension schema 1; logical fields, explicitly encoded by RobotHealthWire. */
    uint8_t schema;
    uint8_t heartbeat_valid_mask;
    uint16_t active_faults;
    uint16_t historical_faults;
    uint16_t last_fault_bits;
    uint32_t last_fault_timestamp;
    uint16_t heartbeat_age_ms[ROBOT_HEALTH_STACK_COUNT];
    uint16_t chassis_sample_age_ms;
    uint16_t i2c_consecutive_fail;
    uint16_t command_age_ms;
    uint8_t command_valid;
    uint8_t arm_status;
    uint8_t arm_last_query;
    uint8_t owner;
    uint16_t last_event;
    uint32_t last_event_timestamp;
    uint32_t uptime_ms;
    uint32_t serial_tx_drop;
} HealthState_t;

typedef struct {
    uint32_t timestamp_ms;
    uint16_t buttons;
    uint8_t left_x;
    uint8_t left_y;
    uint8_t right_x;
    uint8_t right_y;
    uint8_t valid;
    uint8_t reserved;
} Ps2InputSnapshot_t;

/* Queue payload layout checks only; the serial protocol is field encoded. */
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(ChassisCommand_t) == 10U, "ChassisCommand_t wire size");
_Static_assert(sizeof(ArmCommand_t) == 28U, "ArmCommand_t wire size");
#endif

#endif /* ROBOT_TYPES_H */
