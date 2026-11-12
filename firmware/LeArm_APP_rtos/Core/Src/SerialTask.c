#include "SerialTask.h"

#include "RobotProtocol.h"
#include "RobotHealth.h"
#include "RobotHealthWire.h"
#include "RobotRtos.h"
#include "RobotSafety.h"
#include "SerialTransport.h"
#include "boot_entry.h"
#include "usart.h"

#include "cmsis_os.h"
#include "stm32f1xx_hal.h"

#include <stdbool.h>
#include <string.h>

#define SERIAL_PROTOCOL_HEADER_LENGTH 12U
#define SERIAL_TX_TIMEOUT_MS          30U
#define SERIAL_CHASSIS_PERIOD_MS      20U
#define SERIAL_HEALTH_PERIOD_MS       100U
#define SERIAL_RX_CHUNK_SIZE          64U

#define SERIAL_CAPABILITY_CHASSIS   (1U << 0)
#define SERIAL_CAPABILITY_ARM       (1U << 1)
#define SERIAL_CAPABILITY_BOOTLOADER (1U << 2)
#define SERIAL_CAPABILITY_PS2_ESTOP (1U << 3)

#define SERIAL_ARM_CMD_RESET  0x01U
#define SERIAL_ARM_CMD_JOINT  0x02U
#define SERIAL_ARM_CMD_ACTION 0x03U

/*
 * SerialTask 负责“字节到业务命令”的完整边界：协议校验成功后才允许命令
 * 进入 RTOS 队列。USART1 上禁止混入 printf 文本，否则会破坏 COBS 帧流。
 */
static RobotProtocolParser_t serial_parser;
static uint8_t serial_rx_chunk[SERIAL_RX_CHUNK_SIZE];
static uint16_t serial_tx_sequence;
static bool serial_stop_pending;
static uint16_t serial_stop_sequence;

static uint16_t serial_get_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static int16_t serial_get_i16_le(const uint8_t *data)
{
    return (int16_t)serial_get_u16_le(data);
}

static void serial_put_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8U);
}

static void serial_put_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
    data[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static void serial_put_i16_le(uint8_t *data, int16_t value)
{
    serial_put_u16_le(data, (uint16_t)value);
}

static void serial_put_i32_le(uint8_t *data, int32_t value)
{
    serial_put_u32_le(data, (uint32_t)value);
}

static bool serial_send_frame(uint8_t message_type,
                              const uint8_t *payload,
                              uint16_t payload_length,
                              uint32_t timestamp_ms)
{
    RobotFrame_t frame;
    uint8_t encoded[ROBOT_PROTOCOL_MAX_ENCODED_FRAME];
    size_t encoded_length;

    if ((payload_length > ROBOT_PROTOCOL_MAX_PAYLOAD) ||
        ((payload == NULL) && (payload_length != 0U))) {
        return false;
    }

    /* 所有字段先构造成逻辑帧，再统一执行 CRC32C 和 COBS 编码。 */
    (void)memset(&frame, 0, sizeof(frame));
    frame.version = ROBOT_PROTOCOL_VERSION;
    frame.message_type = message_type;
    frame.header_length = SERIAL_PROTOCOL_HEADER_LENGTH;
    frame.sequence = serial_tx_sequence++;
    frame.payload_length = payload_length;
    frame.timestamp_ms = timestamp_ms;
    if (payload_length != 0U) {
        (void)memcpy(frame.payload, payload, payload_length);
    }

    encoded_length = robot_protocol_encode(&frame, encoded, sizeof(encoded));
    bool sent = (encoded_length != 0U) &&
                (serial_transport_send(encoded, encoded_length, SERIAL_TX_TIMEOUT_MS) == HAL_OK);
    if (!sent) robot_health_tx_drop();
    return sent;
}

static bool serial_send_ack(uint16_t request_sequence,
                            RobotAckCode_t code,
                            uint32_t now_ms)
{
    uint8_t payload[3];
    uint8_t message_type = (code == ROBOT_ACK_OK) ? ROBOT_MSG_ACK : ROBOT_MSG_NACK;

    serial_put_u16_le(payload, request_sequence);
    payload[2] = code;
    return serial_send_frame(message_type, payload, sizeof(payload), now_ms);
}

static RobotAckCode_t serial_safety_rejection_code(void)
{
    SafetyState_t state = robot_safety_get_state();

    if ((state == SAFETY_ESTOP_LATCHED) || (state == SAFETY_ESTOP_READY)) {
        return ROBOT_ACK_ESTOP_LATCHED;
    }
    if (state == SAFETY_FAULT) {
        return ROBOT_ACK_DEVICE_FAULT;
    }
    return ROBOT_ACK_BUSY_NOT_SAFE;
}

static void serial_clear_stop_events(void)
{
    if (robotEventFlagsHandle != NULL) {
        (void)osEventFlagsClear(robotEventFlagsHandle,
                                ROBOT_EVT_STOP_REQUESTED |
                                ROBOT_EVT_CHASSIS_STOPPED |
                                ROBOT_EVT_ARM_STOPPED |
                                ROBOT_EVT_ESTOP);
    }
}

static void serial_request_owner_stop(void)
{
    robot_rtos_clear_motion_commands();
    if (robotEventFlagsHandle != NULL) {
        (void)osEventFlagsClear(robotEventFlagsHandle,
                                ROBOT_EVT_CHASSIS_STOPPED |
                                ROBOT_EVT_ARM_STOPPED);
        (void)osEventFlagsSet(robotEventFlagsHandle, ROBOT_EVT_STOP_REQUESTED);
    }
}

static void serial_dispatch_velocity(const RobotFrame_t *frame, uint32_t now_ms)
{
    ChassisCommand_t command;

    if (frame->payload_length != 8U) {
        (void)serial_send_ack(frame->sequence, ROBOT_ACK_INVALID_PAYLOAD, now_ms);
        return;
    }

    command.vx_mm_s = serial_get_i16_le(&frame->payload[0]);
    command.vy_mm_s = serial_get_i16_le(&frame->payload[2]);
    command.wz_mrad_s = serial_get_i16_le(&frame->payload[4]);
    command.timeout_ms = serial_get_u16_le(&frame->payload[6]);
    command.sequence = frame->sequence;
    if (command.timeout_ms < ROBOT_COMMAND_TIMEOUT_MIN_MS) {
        command.timeout_ms = ROBOT_COMMAND_TIMEOUT_MIN_MS;
    } else if (command.timeout_ms > ROBOT_COMMAND_TIMEOUT_MAX_MS) {
        command.timeout_ms = ROBOT_COMMAND_TIMEOUT_MAX_MS;
    }

    /* 协议合法不等于允许运动，仍需通过安全状态机的运动许可。 */
    if (!robot_safety_accept_motion(frame->sequence, now_ms)) {
        (void)serial_send_ack(frame->sequence,
                              serial_safety_rejection_code(),
                              now_ms);
        return;
    }

    serial_clear_stop_events();
    if (!robot_rtos_publish_chassis_command(&command)) {
        (void)robot_safety_request_stop(frame->sequence, now_ms);
        serial_request_owner_stop();
        (void)serial_send_ack(frame->sequence, ROBOT_ACK_BUSY, now_ms);
        return;
    }
    (void)serial_send_ack(frame->sequence, ROBOT_ACK_OK, now_ms);
}

static void serial_dispatch_arm(const RobotFrame_t *frame, uint32_t now_ms)
{
    ArmCommand_t command;
    uint8_t command_length;

    if (frame->payload_length < 2U) {
        (void)serial_send_ack(frame->sequence, ROBOT_ACK_INVALID_PAYLOAD, now_ms);
        return;
    }

    command_length = frame->payload[1];
    if ((command_length > sizeof(command.payload)) ||
        (frame->payload_length != (uint16_t)(2U + command_length))) {
        (void)serial_send_ack(frame->sequence, ROBOT_ACK_INVALID_PAYLOAD, now_ms);
        return;
    }

    (void)memset(&command, 0, sizeof(command));
    command.command = frame->payload[0];
    command.length = command_length;
    command.sequence = frame->sequence;
    if (command_length != 0U) {
        (void)memcpy(command.payload, &frame->payload[2], command_length);
    }

    if (((command.command == SERIAL_ARM_CMD_RESET) &&
         (command.length != 2U)) ||
        ((command.command == SERIAL_ARM_CMD_JOINT) &&
         ((command.length != 5U) || (command.payload[0] < 1U) ||
          (command.payload[0] > 6U))) ||
        ((command.command == SERIAL_ARM_CMD_ACTION) &&
         (command.length != 2U)) ||
        ((command.command != SERIAL_ARM_CMD_RESET) &&
         (command.command != SERIAL_ARM_CMD_JOINT) &&
         (command.command != SERIAL_ARM_CMD_ACTION))) {
        robot_health_count(ROBOT_HEALTH_COUNTER_ARM_REJECTION);
        (void)serial_send_ack(frame->sequence,
                              ROBOT_ACK_INVALID_PAYLOAD,
                              now_ms);
        return;
    }

    /* 机械臂命令与底盘命令共享同一安全许可，不能绕过急停/故障状态。 */
    if (!robot_safety_accept_motion(frame->sequence, now_ms)) {
        (void)serial_send_ack(frame->sequence,
                              serial_safety_rejection_code(),
                              now_ms);
        return;
    }

    serial_clear_stop_events();
    if (!robot_rtos_enqueue_arm_command(&command)) {
        (void)serial_send_ack(frame->sequence, ROBOT_ACK_BUSY, now_ms);
        return;
    }
    (void)serial_send_ack(frame->sequence, ROBOT_ACK_OK, now_ms);
}

static void serial_dispatch_stop(const RobotFrame_t *frame, uint32_t now_ms)
{
    if (frame->payload_length != 0U) {
        (void)serial_send_ack(frame->sequence, ROBOT_ACK_INVALID_PAYLOAD, now_ms);
        return;
    }
    /* STOP ACK 延迟到两个执行器 owner 都确认停止之后发送。 */
    if (serial_stop_pending ||
        !robot_safety_request_stop(frame->sequence, now_ms)) {
        (void)serial_send_ack(frame->sequence,
                              serial_safety_rejection_code(),
                              now_ms);
        return;
    }

    serial_stop_sequence = frame->sequence;
    serial_stop_pending = true;
    serial_request_owner_stop();
}

static RobotAckCode_t serial_boot_rejection_code(SafetyBootDecision_t decision)
{
    switch (decision) {
    case BOOT_DENIED_ESTOP:
        return ROBOT_ACK_ESTOP_LATCHED;
    case BOOT_DENIED_FAULT:
        return ROBOT_ACK_DEVICE_FAULT;
    case BOOT_DENIED_STOP_REQUIRED:
        return ROBOT_ACK_STOP_HANDSHAKE_REQUIRED;
    case BOOT_DENIED_BUSY:
    default:
        return ROBOT_ACK_BUSY_NOT_SAFE;
    }
}

static void serial_dispatch_boot(const RobotFrame_t *frame, uint32_t now_ms)
{
    uint16_t stop_sequence;
    SafetyBootDecision_t decision;

    if (frame->payload_length != 2U) {
        (void)serial_send_ack(frame->sequence, ROBOT_ACK_INVALID_PAYLOAD, now_ms);
        return;
    }

    stop_sequence = serial_get_u16_le(frame->payload);
    /* Boot 请求必须携带已完成 STOP 事务的同一序列号，防止旧授权被重放。 */
    decision = robot_safety_request_boot(stop_sequence, now_ms);
    if (decision != BOOT_ALLOWED) {
        robot_health_count(ROBOT_HEALTH_COUNTER_BOOTLOADER_REJECT);
        (void)serial_send_ack(frame->sequence,
                              serial_boot_rejection_code(decision),
                              now_ms);
        return;
    }

    if (!serial_send_ack(frame->sequence, ROBOT_ACK_OK, now_ms) ||
        (robot_safety_get_state() != SAFETY_BOOT_PENDING)) {
        robot_safety_cancel_boot(HAL_GetTick());
        return;
    }

    if (serial_transport_shutdown() != HAL_OK) {
        robot_safety_cancel_boot(HAL_GetTick());
        (void)serial_transport_init(&huart1);
        return;
    }

    if (robot_safety_get_state() != SAFETY_BOOT_PENDING) {
        robot_safety_cancel_boot(HAL_GetTick());
        (void)serial_transport_init(&huart1);
        return;
    }

    (void)boot_entry_execute();
}

static void serial_send_hello(uint32_t now_ms)
{
    uint8_t payload[10];
    uint16_t capabilities = SERIAL_CAPABILITY_CHASSIS |
                            SERIAL_CAPABILITY_ARM |
                            SERIAL_CAPABILITY_BOOTLOADER |
                            SERIAL_CAPABILITY_PS2_ESTOP;

    payload[0] = ROBOT_PROTOCOL_VERSION;
    payload[1] = robot_safety_get_state();
    serial_put_u16_le(&payload[2], capabilities);
    serial_put_u32_le(&payload[4], now_ms);
    serial_put_u16_le(&payload[8], ROBOT_PROTOCOL_MAX_PAYLOAD);
    (void)serial_send_frame(ROBOT_MSG_HELLO_STATE,
                            payload,
                            sizeof(payload),
                            now_ms);
}

static void serial_send_chassis_state(void)
{
    ChassisState_t state;
    uint8_t payload[32];
    uint8_t index;

    /* 读取 ChassisTask 发布的一致快照，不在串口任务中直接访问 I2C。 */
    robot_rtos_get_chassis_state(&state);
    serial_put_u32_le(&payload[0], state.timestamp_ms);
    for (index = 0U; index < 4U; ++index) {
        serial_put_i32_le(&payload[4U + ((uint16_t)index * 4U)],
                          state.encoder[index]);
        payload[20U + index] = (uint8_t)state.motor_command[index];
    }
    serial_put_u16_le(&payload[24], state.battery_mv);
    serial_put_u16_le(&payload[26], state.fault_flags);
    payload[28] = state.safety_state;
    payload[29] = 0U;
    serial_put_u16_le(&payload[30], state.sequence);
    (void)serial_send_frame(ROBOT_MSG_CHASSIS_STATE,
                            payload,
                            sizeof(payload),
                            state.timestamp_ms);
}

static void serial_send_arm_state(void)
{
    ArmState_t state;
    uint8_t payload[22];
    uint8_t index;

    robot_rtos_get_arm_state(&state);
    serial_put_u32_le(&payload[0], state.timestamp_ms);
    for (index = 0U; index < 6U; ++index) {
        serial_put_i16_le(&payload[4U + ((uint16_t)index * 2U)],
                          state.joint_position[index]);
    }
    serial_put_u16_le(&payload[16], state.fault_flags);
    payload[18] = state.active_command;
    payload[19] = state.valid_mask;
    serial_put_u16_le(&payload[20], state.sequence);
    (void)serial_send_frame(ROBOT_MSG_ARM_STATE,
                            payload,
                            sizeof(payload),
                            state.timestamp_ms);
}

static void serial_send_health_state(void)
{
    HealthState_t state = {0};
    uint8_t payload[ROBOT_HEALTH_WIRE_SIZE];

    /* Never publish an uninitialized stack snapshot if startup regresses. */
    if (!robot_health_get_snapshot(&state)) return;
    if (robot_health_wire_encode(&state, payload, sizeof(payload)) == 0U) return;
    (void)serial_send_frame(ROBOT_MSG_HEALTH_STATE,
                            payload,
                            sizeof(payload),
                            state.timestamp_ms);
}

static void serial_dispatch_frame(const RobotFrame_t *frame)
{
    uint32_t now_ms = HAL_GetTick();

    switch (frame->message_type) {
    case ROBOT_MSG_HELLO_QUERY:
        if (frame->payload_length == 0U) {
            serial_send_hello(now_ms);
        } else {
            (void)serial_send_ack(frame->sequence,
                                  ROBOT_ACK_INVALID_PAYLOAD,
                                  now_ms);
        }
        break;

    case ROBOT_MSG_STATE_QUERY:
        if (frame->payload_length == 0U) {
            serial_send_chassis_state();
            serial_send_arm_state();
            serial_send_health_state();
        } else {
            (void)serial_send_ack(frame->sequence,
                                  ROBOT_ACK_INVALID_PAYLOAD,
                                  now_ms);
        }
        break;

    case ROBOT_MSG_CMD_VELOCITY:
        serial_dispatch_velocity(frame, now_ms);
        break;

    case ROBOT_MSG_CMD_ARM:
        serial_dispatch_arm(frame, now_ms);
        break;

    case ROBOT_MSG_STOP_ALL:
        serial_dispatch_stop(frame, now_ms);
        break;

    case ROBOT_MSG_ENTER_BOOTLOADER:
        serial_dispatch_boot(frame, now_ms);
        break;

    default:
        (void)serial_send_ack(frame->sequence,
                              ROBOT_ACK_INVALID_PAYLOAD,
                              now_ms);
        break;
    }
}

static void serial_check_stop_completion(uint32_t now_ms)
{
    SafetyState_t state;

    if (!serial_stop_pending) {
        return;
    }
    if (robot_safety_stop_completed(serial_stop_sequence)) {
        if (serial_send_ack(serial_stop_sequence, ROBOT_ACK_OK, now_ms)) {
            serial_stop_pending = false;
            if (robotEventFlagsHandle != NULL) {
                (void)osEventFlagsClear(robotEventFlagsHandle,
                                        ROBOT_EVT_STOP_REQUESTED);
            }
        }
        return;
    }

    state = robot_safety_get_state();
    if ((state == SAFETY_ESTOP_LATCHED) || (state == SAFETY_ESTOP_READY) ||
        (state == SAFETY_FAULT) || (state == SAFETY_SAFE_IDLE)) {
        (void)serial_send_ack(serial_stop_sequence,
                              serial_safety_rejection_code(),
                              now_ms);
        serial_stop_pending = false;
    }
}

void serial_task_run(void *argument)
{
    uint32_t next_chassis_ms;
    uint32_t next_health_ms;

    (void)argument;
    /* 本任务独占 parser，ISR 只负责把原始字节搬入传输层环形缓冲。 */
    robot_protocol_parser_init(&serial_parser);
    serial_tx_sequence = 0U;
    serial_stop_pending = false;
    serial_stop_sequence = 0U;

    if (serial_transport_init(&huart1) != HAL_OK) {
        robot_safety_latch_fault(ROBOT_FAULT_PROTOCOL, HAL_GetTick());
        if (robotEventFlagsHandle != NULL) {
            (void)osEventFlagsSet(robotEventFlagsHandle,
                                  ROBOT_EVT_FAULT | ROBOT_EVT_STOP_REQUESTED);
        }
        for (;;) {
            (void)osDelay(100U);
        }
    }

    if (robotEventFlagsHandle != NULL) {
        (void)osEventFlagsSet(robotEventFlagsHandle, ROBOT_EVT_SERIAL_READY);
    }

    next_chassis_ms = HAL_GetTick() + SERIAL_CHASSIS_PERIOD_MS;
    next_health_ms = HAL_GetTick() + SERIAL_HEALTH_PERIOD_MS;
    for (;;) {
        size_t received = serial_transport_read(serial_rx_chunk,
                                                sizeof(serial_rx_chunk));
        size_t index;
        uint32_t now_ms;

        robot_health_heartbeat(ROBOT_TASK_SERIAL, HAL_GetTick());

        for (index = 0U; index < received; ++index) {
            RobotFrame_t frame;
            uint32_t crc_before = serial_parser.crc_error_count;
            RobotProtocolResult_t result =
                robot_protocol_consume_byte(&serial_parser,
                                            serial_rx_chunk[index],
                                            &frame);
            if (serial_parser.crc_error_count != crc_before) {
                robot_health_count(ROBOT_HEALTH_COUNTER_UART_CRC);
            }
            if (result == ROBOT_PROTOCOL_FRAME_READY) {
                serial_dispatch_frame(&frame);
            } else if ((result == ROBOT_PROTOCOL_FRAME_DROPPED) &&
                       (serial_parser.last_drop_was_crc == 0U)) {
                /* CRC drops have their own counter; do not double-count framing. */
                robot_health_count(ROBOT_HEALTH_COUNTER_UART_FRAMING);
            }
        }

        now_ms = HAL_GetTick();
        serial_check_stop_completion(now_ms);
        if ((int32_t)(now_ms - next_chassis_ms) >= 0) {
            serial_send_chassis_state();
            next_chassis_ms += SERIAL_CHASSIS_PERIOD_MS;
            if ((int32_t)(now_ms - next_chassis_ms) >=
                (int32_t)SERIAL_CHASSIS_PERIOD_MS) {
                next_chassis_ms = now_ms + SERIAL_CHASSIS_PERIOD_MS;
            }
        }
        if ((int32_t)(now_ms - next_health_ms) >= 0) {
            serial_send_health_state();
            next_health_ms += SERIAL_HEALTH_PERIOD_MS;
            if ((int32_t)(now_ms - next_health_ms) >=
                (int32_t)SERIAL_HEALTH_PERIOD_MS) {
                next_health_ms = now_ms + SERIAL_HEALTH_PERIOD_MS;
            }
        }
        (void)osDelay(1U);
    }
}
