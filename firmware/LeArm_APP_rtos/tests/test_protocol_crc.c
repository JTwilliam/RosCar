#include "RobotProtocol.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

static RobotProtocolResult_t feed(RobotProtocolParser_t *parser, const uint8_t *bytes, size_t length)
{
    RobotFrame_t frame;
    RobotProtocolResult_t result = ROBOT_PROTOCOL_INCOMPLETE;
    size_t i;
    for (i = 0; i < length; ++i) result = robot_protocol_consume_byte(parser, bytes[i], &frame);
    return result;
}

int main(void)
{
    RobotProtocolParser_t parser;
    RobotFrame_t frame;
    uint8_t good[258], bad[258], raw[256];
    size_t good_length, raw_length, bad_length;
    (void)memset(&frame, 0, sizeof(frame));
    frame.version = 1; frame.header_length = 12;
    frame.message_type = ROBOT_MSG_HELLO_QUERY;
    frame.sequence = 123; frame.timestamp_ms = 500;
    good_length = robot_protocol_encode(&frame, good, sizeof(good));
    assert(good_length > 0);
    assert(robot_protocol_cobs_decode(good, good_length - 1, raw, sizeof(raw), &raw_length));
    raw[raw_length - 1] ^= 1; /* Only CRC changes; length/version stay valid. */
    assert(robot_protocol_cobs_encode(raw, raw_length, bad, sizeof(bad) - 1, &bad_length));
    bad[bad_length++] = 0;
    robot_protocol_parser_init(&parser);
    assert(feed(&parser, good, good_length) == ROBOT_PROTOCOL_FRAME_READY);
    assert(parser.crc_error_count == 0);
    assert(feed(&parser, bad, bad_length) == ROBOT_PROTOCOL_FRAME_DROPPED);
    assert(parser.crc_error_count == 1 && parser.last_drop_was_crc == 1);
    assert(feed(&parser, good, good_length) == ROBOT_PROTOCOL_FRAME_READY);
    assert(parser.crc_error_count == 1 && parser.last_drop_was_crc == 0);
    /* A malformed COBS candidate remains framing, not CRC. */
    { const uint8_t malformed[] = {0xFF, 0};
      assert(feed(&parser, malformed, sizeof(malformed)) == ROBOT_PROTOCOL_FRAME_DROPPED); }
    assert(parser.crc_error_count == 1 && parser.last_drop_was_crc == 0);
    parser.crc_error_count = UINT32_MAX;
    assert(feed(&parser, bad, bad_length) == ROBOT_PROTOCOL_FRAME_DROPPED);
    assert(parser.crc_error_count == UINT32_MAX && parser.last_drop_was_crc == 1);
    robot_protocol_parser_init(&parser);
    assert(parser.crc_error_count == 0 && parser.last_drop_was_crc == 0);
    puts("PASS CRC mismatch counter, saturation, framing separation and recovery");
    return 0;
}
