#include "RobotHealthWire.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static unsigned get16(const uint8_t *p) { return p[0] | ((unsigned)p[1] << 8); }
int main(int argc, char **argv)
{
    HealthState_t s;
    uint8_t out[130];
    unsigned i;
    (void)memset(&s, 0, sizeof(s));
    (void)memset(out, 0xAA, sizeof(out));
    s.timestamp_ms = 0x12345678U;
    s.minimum_free_heap_bytes = 2048U;
    for (i = 0; i < 5; ++i) { s.stack_free_words[i] = (uint16_t)(100U + i); s.heartbeat_age_ms[i] = (uint16_t)(10U + i); }
    for (i = 0; i < 11; ++i) s.counters[i] = (uint16_t)(200U + i);
    s.fault_flags = 0x18U; s.safety_state = SAFETY_ESTOP_LATCHED;
    s.heartbeat_valid_mask = 0x1FU;
    s.active_faults = ROBOT_FAULT_MOTOR;
    s.historical_faults = ROBOT_FAULT_MOTOR | ROBOT_FAULT_COMMAND_TIMEOUT;
    s.last_fault_bits = ROBOT_FAULT_MOTOR;
    s.last_fault_timestamp = 900U;
    s.heartbeat_age_ms[4] = 65535U;
    s.chassis_sample_age_ms = 65534U;
    s.i2c_consecutive_fail = 7U;
    s.command_age_ms = 65535U;
    s.command_valid = 0U; s.arm_status = 0U; s.arm_last_query = 0U; s.owner = 0U;
    s.last_event = 6U; s.last_event_timestamp = 800U;
    s.uptime_ms = 1000U; s.serial_tx_drop = 3U;
    assert(robot_health_wire_encode(&s, out, 127) == 0U);
    assert(out[0] == 0xAA);
    assert(robot_health_wire_encode(NULL, out, 128) == 0U);
    assert(robot_health_wire_encode(&s, NULL, 128) == 0U);
    assert(robot_health_wire_encode(&s, out, 128) == 128U);
    assert(out[0] == 0x78 && out[3] == 0x12);
    assert(get16(out + 8) == 100 && get16(out + 18) == 200);
    assert(get16(out + 40) == 0x18 && out[42] == 4 && out[43] == 0);
    assert(out[44] == 1 && out[45] == 31 && get16(out + 46) == 16);
    assert(get16(out + 48) == 17 && get16(out + 50) == 16);
    assert(get16(out + 64) == 65535 && get16(out + 66) == 65534);
    assert(get16(out + 68) == 7 && get16(out + 70) == 65535);
    assert(get16(out + 76) == 6 && get16(out + 78) == 0);
    assert(memcmp(out + 92, "freertos-health1", 16) == 0);
    for (i = 108; i < 128; ++i) assert(out[i] == 0);
    assert(out[128] == 0xAA && out[129] == 0xAA);
    if (argc > 1) {
        FILE *file = fopen(argv[1], "wb");
        assert(file != NULL);
        assert(fwrite(out, 1, 128, file) == 128);
        assert(fclose(file) == 0);
    }
    puts("PASS health schema1 128-byte fixture");
    return 0;
}
