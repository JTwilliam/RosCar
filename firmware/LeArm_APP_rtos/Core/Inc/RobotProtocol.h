#ifndef ROBOT_PROTOCOL_H
#define ROBOT_PROTOCOL_H

#include "RobotTypes.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 串口协议层只负责分帧、长度检查、COBS 和 CRC32C，不解释运动语义。
 * 完整发送帧由非零的 COBS 数据和一个结尾 0x00 组成。
 */
#define ROBOT_PROTOCOL_MAX_ENCODED_FRAME 258U

/* 解码后的逻辑帧；payload 仍需由 SerialTask 按消息类型逐字段解释。 */
typedef struct {
    uint8_t version;
    uint8_t message_type;
    uint8_t flags;
    uint8_t header_length;
    uint16_t sequence;
    uint16_t payload_length;
    uint32_t timestamp_ms;
    uint8_t payload[ROBOT_PROTOCOL_MAX_PAYLOAD];
} RobotFrame_t;

typedef struct {
    /* 尚未遇到 0x00 的候选帧暂存在此处，缓冲区满后丢弃到下一分隔符。 */
    uint8_t encoded[ROBOT_PROTOCOL_MAX_ENCODED_FRAME];
    uint16_t encoded_length;
    uint8_t discard_until_delimiter;
    uint8_t reserved;
    uint32_t overflow_count;
    /* Saturating checksum-mismatch count; protocol stays RTOS-independent. */
    uint32_t crc_error_count;
    uint8_t last_drop_was_crc; /* Remains accurate even after counter saturation. */
} RobotProtocolParser_t;

typedef enum {
    ROBOT_PROTOCOL_INCOMPLETE = 0,
    ROBOT_PROTOCOL_FRAME_READY,
    ROBOT_PROTOCOL_FRAME_DROPPED
} RobotProtocolResult_t;

void robot_protocol_parser_init(RobotProtocolParser_t *parser);
/*
 * 逐字节喂入串口数据。超长候选帧进入“丢弃直到下一个 0x00”状态，
 * 防止噪声造成缓冲区越界，同时保证后续合法帧能够重新同步。
 */
RobotProtocolResult_t robot_protocol_consume_byte(RobotProtocolParser_t *parser,
                                                   uint8_t byte,
                                                   RobotFrame_t *frame_out);

/* 返回可直接发送的完整长度，其中包含最后一个 0x00 分隔符。 */
size_t robot_protocol_encode(const RobotFrame_t *frame, uint8_t *encoded_out,
                             size_t encoded_capacity);

/* 只有 length 为 0 时才允许 data 为 NULL。 */
uint32_t robot_protocol_crc32c(const uint8_t *data, size_t length);

/* 有界 COBS 辅助函数不处理末尾 0x00，主要供协议实现和主机单元测试使用。 */
int robot_protocol_cobs_encode(const uint8_t *input, size_t input_length,
                               uint8_t *output, size_t output_capacity,
                               size_t *output_length);
int robot_protocol_cobs_decode(const uint8_t *input, size_t input_length,
                               uint8_t *output, size_t output_capacity,
                               size_t *output_length);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_PROTOCOL_H */
