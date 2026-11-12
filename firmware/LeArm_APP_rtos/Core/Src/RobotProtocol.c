#include "RobotProtocol.h"

#include <limits.h>
#include <string.h>

/*
 * 解码后固定头为 12 字节，末尾 4 字节为 CRC32C。
 * 所有多字节字段都显式按小端读写，禁止依赖结构体布局。
 */
#define ROBOT_PROTOCOL_HEADER_LENGTH 12U
#define ROBOT_PROTOCOL_CRC_LENGTH 4U
#define ROBOT_PROTOCOL_MIN_DECODED_FRAME \
    (ROBOT_PROTOCOL_HEADER_LENGTH + ROBOT_PROTOCOL_CRC_LENGTH)

static uint16_t robot_protocol_get_u16_le(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static uint32_t robot_protocol_get_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

static void robot_protocol_put_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8U);
}

static void robot_protocol_put_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
    data[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

uint32_t robot_protocol_crc32c(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFFU;
    size_t index;

    if ((data == NULL) && (length != 0U)) {
        return 0U;
    }

    /* Castagnoli 反射多项式，初值和结果均进行异或。 */
    for (index = 0U; index < length; ++index) {
        uint8_t bit;

        crc ^= (uint32_t)data[index];
        for (bit = 0U; bit < 8U; ++bit) {
            if ((crc & 1U) != 0U) {
                crc = (crc >> 1U) ^ 0x82F63B78U;
            } else {
                crc >>= 1U;
            }
        }
    }

    return crc ^ 0xFFFFFFFFU;
}

int robot_protocol_cobs_encode(const uint8_t *input, size_t input_length,
                               uint8_t *output, size_t output_capacity,
                               size_t *output_length)
{
    size_t input_index = 0U;
    size_t code_index = 0U;
    size_t output_index = 1U;
    uint8_t code = 1U;

    if ((output == NULL) || (output_length == NULL) ||
        ((input == NULL) && (input_length != 0U)) || (output_capacity == 0U)) {
        return 0;
    }

    /* code_index 指向当前非零数据块的长度码位置。 */
    while (input_index < input_length) {
        if (input[input_index] == 0U) {
            output[code_index] = code;
            code_index = output_index;
            if (output_index >= output_capacity) {
                return 0;
            }
            ++output_index;
            code = 1U;
        } else {
            if (output_index >= output_capacity) {
                return 0;
            }
            output[output_index] = input[input_index];
            ++output_index;
            ++code;
            if (code == 0xFFU) {
                output[code_index] = code;
                code_index = output_index;
                if (output_index >= output_capacity) {
                    return 0;
                }
                ++output_index;
                code = 1U;
            }
        }
        ++input_index;
    }

    output[code_index] = code;
    *output_length = output_index;
    return 1;
}

int robot_protocol_cobs_decode(const uint8_t *input, size_t input_length,
                               uint8_t *output, size_t output_capacity,
                               size_t *output_length)
{
    size_t input_index = 0U;
    size_t output_index = 0U;

    if ((input == NULL) || (output == NULL) || (output_length == NULL) ||
        (input_length == 0U)) {
        return 0;
    }

    while (input_index < input_length) {
        uint8_t code = input[input_index];
        size_t copy_length;
        size_t copy_index;

        if (code == 0U) {
            return 0;
        }
        ++input_index;
        copy_length = (size_t)code - 1U;
        if (copy_length > (input_length - input_index) ||
            copy_length > (output_capacity - output_index)) {
            return 0;
        }
        for (copy_index = 0U; copy_index < copy_length; ++copy_index) {
            output[output_index] = input[input_index];
            ++output_index;
            ++input_index;
        }
        if ((code != 0xFFU) && (input_index < input_length)) {
            if (output_index >= output_capacity) {
                return 0;
            }
            output[output_index] = 0U;
            ++output_index;
        }
    }

    *output_length = output_index;
    return 1;
}

void robot_protocol_parser_init(RobotProtocolParser_t *parser)
{
    if (parser != NULL) {
        (void)memset(parser, 0, sizeof(*parser));
    }
}

static void robot_protocol_note_overflow(RobotProtocolParser_t *parser)
{
    if (parser->overflow_count != UINT32_MAX) {
        ++parser->overflow_count;
    }
}

static RobotProtocolResult_t robot_protocol_decode_candidate(
    RobotProtocolParser_t *parser, RobotFrame_t *frame_out)
{
    uint8_t decoded[ROBOT_PROTOCOL_MAX_DECODED_FRAME_BYTES];
    RobotFrame_t decoded_frame = {0};
    size_t decoded_length = 0U;
    uint16_t payload_length;
    size_t expected_length;
    uint32_t expected_crc;
    uint32_t received_crc;

    if ((frame_out == NULL) ||
        (robot_protocol_cobs_decode(parser->encoded, parser->encoded_length,
                                    decoded, sizeof(decoded), &decoded_length) == 0)) {
        return ROBOT_PROTOCOL_FRAME_DROPPED;
    }
    /* 先验证长度，再读取 payload 和 CRC，避免畸形帧越界访问。 */
    if (decoded_length < ROBOT_PROTOCOL_MIN_DECODED_FRAME ||
        decoded_length > ROBOT_PROTOCOL_MAX_DECODED_FRAME_BYTES) {
        return ROBOT_PROTOCOL_FRAME_DROPPED;
    }
    if ((decoded[0] != ROBOT_PROTOCOL_VERSION) ||
        (decoded[3] != ROBOT_PROTOCOL_HEADER_LENGTH)) {
        return ROBOT_PROTOCOL_FRAME_DROPPED;
    }

    payload_length = robot_protocol_get_u16_le(&decoded[6]);
    if (payload_length > ROBOT_PROTOCOL_MAX_PAYLOAD) {
        return ROBOT_PROTOCOL_FRAME_DROPPED;
    }
    expected_length = ROBOT_PROTOCOL_HEADER_LENGTH + (size_t)payload_length +
                      ROBOT_PROTOCOL_CRC_LENGTH;
    if (decoded_length != expected_length) {
        return ROBOT_PROTOCOL_FRAME_DROPPED;
    }

    expected_crc = robot_protocol_crc32c(decoded,
                                         ROBOT_PROTOCOL_HEADER_LENGTH + (size_t)payload_length);
    received_crc = robot_protocol_get_u32_le(&decoded[ROBOT_PROTOCOL_HEADER_LENGTH +
                                                      (size_t)payload_length]);
    if (received_crc != expected_crc) {
        if (parser->crc_error_count < UINT32_MAX) ++parser->crc_error_count;
        parser->last_drop_was_crc = 1U;
        return ROBOT_PROTOCOL_FRAME_DROPPED;
    }

    decoded_frame.version = decoded[0];
    decoded_frame.message_type = decoded[1];
    decoded_frame.flags = decoded[2];
    decoded_frame.header_length = decoded[3];
    decoded_frame.sequence = robot_protocol_get_u16_le(&decoded[4]);
    decoded_frame.payload_length = payload_length;
    decoded_frame.timestamp_ms = robot_protocol_get_u32_le(&decoded[8]);
    if (payload_length != 0U) {
        (void)memcpy(decoded_frame.payload, &decoded[ROBOT_PROTOCOL_HEADER_LENGTH], payload_length);
    }

    *frame_out = decoded_frame;
    return ROBOT_PROTOCOL_FRAME_READY;
}

RobotProtocolResult_t robot_protocol_consume_byte(RobotProtocolParser_t *parser,
                                                   uint8_t byte,
                                                   RobotFrame_t *frame_out)
{
    RobotProtocolResult_t result;

    if (parser == NULL) {
        return ROBOT_PROTOCOL_FRAME_DROPPED;
    }

    parser->last_drop_was_crc = 0U;
    /* 超长帧后必须等到 0x00 才恢复，防止把同一坏帧的尾部当成新帧。 */
    if (parser->discard_until_delimiter != 0U) {
        if (byte == 0U) {
            parser->discard_until_delimiter = 0U;
            parser->encoded_length = 0U;
            return ROBOT_PROTOCOL_FRAME_DROPPED;
        }
        return ROBOT_PROTOCOL_INCOMPLETE;
    }

    if (byte != 0U) {
        if (parser->encoded_length >= ROBOT_PROTOCOL_MAX_ENCODED_FRAME) {
            parser->encoded_length = 0U;
            parser->discard_until_delimiter = 1U;
            robot_protocol_note_overflow(parser);
            return ROBOT_PROTOCOL_INCOMPLETE;
        }
        parser->encoded[parser->encoded_length] = byte;
        ++parser->encoded_length;
        return ROBOT_PROTOCOL_INCOMPLETE;
    }

    if (parser->encoded_length == 0U) {
        return ROBOT_PROTOCOL_INCOMPLETE;
    }

    result = robot_protocol_decode_candidate(parser, frame_out);
    parser->encoded_length = 0U;
    return result;
}

size_t robot_protocol_encode(const RobotFrame_t *frame, uint8_t *encoded_out,
                             size_t encoded_capacity)
{
    uint8_t decoded[ROBOT_PROTOCOL_MAX_DECODED_FRAME_BYTES];
    uint8_t cobs_encoded[ROBOT_PROTOCOL_MAX_ENCODED_FRAME];
    size_t decoded_length;
    size_t cobs_length = 0U;
    uint32_t crc;

    if ((frame == NULL) || (encoded_out == NULL) ||
        (frame->version != ROBOT_PROTOCOL_VERSION) ||
        (frame->header_length != ROBOT_PROTOCOL_HEADER_LENGTH) ||
        (frame->payload_length > ROBOT_PROTOCOL_MAX_PAYLOAD)) {
        return 0U;
    }

    decoded_length = ROBOT_PROTOCOL_HEADER_LENGTH + (size_t)frame->payload_length +
                     ROBOT_PROTOCOL_CRC_LENGTH;
    if (decoded_length > sizeof(decoded)) {
        return 0U;
    }

    decoded[0] = frame->version;
    decoded[1] = frame->message_type;
    decoded[2] = frame->flags;
    decoded[3] = frame->header_length;
    robot_protocol_put_u16_le(&decoded[4], frame->sequence);
    robot_protocol_put_u16_le(&decoded[6], frame->payload_length);
    robot_protocol_put_u32_le(&decoded[8], frame->timestamp_ms);
    if (frame->payload_length != 0U) {
        (void)memcpy(&decoded[ROBOT_PROTOCOL_HEADER_LENGTH], frame->payload,
                     frame->payload_length);
    }
    crc = robot_protocol_crc32c(decoded,
                                 ROBOT_PROTOCOL_HEADER_LENGTH + (size_t)frame->payload_length);
    robot_protocol_put_u32_le(&decoded[ROBOT_PROTOCOL_HEADER_LENGTH +
                                      (size_t)frame->payload_length], crc);

    if (robot_protocol_cobs_encode(decoded, decoded_length, cobs_encoded, sizeof(cobs_encoded),
                                   &cobs_length) == 0 ||
        (cobs_length >= encoded_capacity)) {
        return 0U;
    }
    (void)memcpy(encoded_out, cobs_encoded, cobs_length);
    encoded_out[cobs_length] = 0U;
    return cobs_length + 1U;
}
