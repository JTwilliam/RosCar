#include "RobotHealthWire.h"
#include <string.h>

static void put16(uint8_t *p, uint16_t v)
{ p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8U); }
static void put32(uint8_t *p, uint32_t v)
{ p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8U); p[2] = (uint8_t)(v >> 16U); p[3] = (uint8_t)(v >> 24U); }

size_t robot_health_wire_encode(const HealthState_t *s, uint8_t *out, size_t capacity)
{
    uint8_t i;
    static const char firmware_id[] = "freertos-health1";
    if ((s == NULL) || (out == NULL) || (capacity < ROBOT_HEALTH_WIRE_SIZE)) return 0U;
    (void)memset(out, 0, ROBOT_HEALTH_WIRE_SIZE);
    put32(out, s->timestamp_ms);
    put32(out + 4, s->minimum_free_heap_bytes);
    for (i = 0U; i < ROBOT_HEALTH_STACK_COUNT; ++i) put16(out + 8U + 2U * i, s->stack_free_words[i]);
    for (i = 0U; i < ROBOT_HEALTH_COUNTER_COUNT; ++i) put16(out + 18U + 2U * i, s->counters[i]);
    put16(out + 40, s->fault_flags);
    out[42] = s->safety_state;
    out[44] = ROBOT_HEALTH_SCHEMA;
    out[45] = (uint8_t)(s->heartbeat_valid_mask & 0x1FU);
    put16(out + 46, s->active_faults);
    put16(out + 48, s->historical_faults);
    put16(out + 50, s->last_fault_bits);
    put32(out + 52, s->last_fault_timestamp);
    for (i = 0U; i < ROBOT_HEALTH_STACK_COUNT; ++i) put16(out + 56U + 2U * i, s->heartbeat_age_ms[i]);
    put16(out + 66, s->chassis_sample_age_ms);
    put16(out + 68, s->i2c_consecutive_fail);
    put16(out + 70, s->command_age_ms);
    out[72] = s->command_valid;
    out[73] = s->arm_status;
    out[74] = s->arm_last_query;
    out[75] = s->owner;
    put16(out + 76, s->last_event);
    put32(out + 80, s->last_event_timestamp);
    put32(out + 84, s->uptime_ms);
    put32(out + 88, s->serial_tx_drop);
    (void)memcpy(out + 92, firmware_id, sizeof(firmware_id) - 1U);
    return ROBOT_HEALTH_WIRE_SIZE;
}
