/* Host-only observation tests. No production source or reset path is changed. */
#include <assert.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "RobotHealth.h"

static uint32_t host_tick;
static unsigned critical_depth;
static unsigned critical_entries;

void health_host_enter_critical(void) { ++critical_depth; ++critical_entries; }
void health_host_exit_critical(void) { assert(critical_depth > 0U); --critical_depth; }
uint32_t HAL_GetTick(void) { return host_tick; }

/* White-box access seeds the otherwise impractical 32-bit saturation boundary.
 * The compiled implementation is the actual firmware file, not a rewritten model.
 * Reset-only HAL symbols deliberately have no definitions: --gc-sections must
 * remove that uncalled path, and accidental use therefore cannot silently reset. */
#include "../Core/Src/RobotHealth.c"

static HealthState_t snapshot(void)
{
    HealthState_t result;
    (void)memset(&result, 0xA5, sizeof(result));
    assert(robot_health_get_snapshot(&result));
    assert(critical_depth == 0U);
    return result;
}

static void fresh(uint32_t now)
{
    assert(critical_depth == 0U);
    host_tick = now;
    robot_health_init(now);
}

static void test_snapshot_requires_initialization(void)
{
    HealthState_t output, expected;
    (void)memset(&output, 0xA5, sizeof(output));
    expected = output;
    assert(!robot_health_initialized);
    assert(!robot_health_get_snapshot(&output));
    assert(memcmp(&output, &expected, sizeof(output)) == 0);
    assert(!robot_health_get_snapshot(NULL));
    fresh(123U);
    assert(robot_health_get_snapshot(&output));
    assert(output.timestamp_ms == 123U && output.schema == 1U);
}

static void test_init_unknown(void)
{
    HealthState_t s;
    unsigned i;
    fresh(123U);
    s = snapshot();
    assert(s.schema == 1U && s.timestamp_ms == 123U && s.uptime_ms == 123U);
    assert(s.safety_state == SAFETY_STARTUP);
    assert(s.heartbeat_valid_mask == 0U && s.active_faults == 0U);
    assert(s.historical_faults == 0U && s.last_fault_bits == 0U);
    assert(s.chassis_sample_age_ms == UINT16_MAX);
    assert(s.command_age_ms == UINT16_MAX && s.command_valid == 0U);
    for (i = 0U; i < ROBOT_TASK_COUNT; ++i) {
        assert(s.heartbeat_age_ms[i] == UINT16_MAX);
        assert(!robot_health_heartbeat_expired((RobotTaskId_t)i, 10000U, 1U));
    }
    assert(robot_health_heartbeat_expired(ROBOT_TASK_COUNT, 123U, 100U));
    robot_health_heartbeat(ROBOT_TASK_COUNT, 123U);
    robot_health_set_current_faults(ROBOT_HEALTH_SOURCE_COUNT, ROBOT_FAULT_MOTOR, 123U);
    assert(snapshot().active_faults == 0U);
    robot_health_get_snapshot(NULL);
}

static void test_heartbeat_updates_expiry_and_recovery(void)
{
    HealthState_t s;
    fresh(1000U);
    robot_health_heartbeat(ROBOT_TASK_CHASSIS, 1000U);
    host_tick = 1050U;
    s = snapshot();
    assert(s.heartbeat_valid_mask == (1U << ROBOT_TASK_CHASSIS));
    assert(s.heartbeat_age_ms[ROBOT_TASK_CHASSIS] == 50U);
    assert((s.active_faults & ROBOT_FAULT_HEALTH) == 0U);
    assert(!robot_health_heartbeat_expired(ROBOT_TASK_CHASSIS, 1050U, 50U));
    host_tick = 1051U;
    assert(robot_health_heartbeat_expired(ROBOT_TASK_CHASSIS, host_tick, 50U));
    s = snapshot();
    assert((s.active_faults & ROBOT_FAULT_HEALTH) != 0U);
    assert(s.last_fault_bits == ROBOT_FAULT_HEALTH && s.last_fault_timestamp == 1051U);
    robot_health_heartbeat(ROBOT_TASK_CHASSIS, 1051U);
    s = snapshot();
    assert(s.heartbeat_age_ms[ROBOT_TASK_CHASSIS] == 0U);
    assert((s.active_faults & ROBOT_FAULT_HEALTH) == 0U);
    assert((s.historical_faults & ROBOT_FAULT_HEALTH) != 0U);
    host_tick = 1051U + 70000U;
    assert(snapshot().heartbeat_age_ms[ROBOT_TASK_CHASSIS] == 65534U);
}

static void test_i2c_recovery_and_repeated_rising_fault(void)
{
    HealthState_t s;
    fresh(200U);
    robot_health_chassis_sample(false, 200U);
    s = snapshot();
    assert(s.i2c_consecutive_fail == 1U && s.chassis_sample_age_ms == UINT16_MAX);
    assert(s.active_faults == ROBOT_FAULT_MOTOR && s.historical_faults == ROBOT_FAULT_MOTOR);
    assert(s.last_fault_bits == ROBOT_FAULT_MOTOR && s.last_fault_timestamp == 200U);
    host_tick = 205U;
    robot_health_chassis_sample(false, 199U); /* already active: not another rising edge */
    assert(snapshot().last_fault_timestamp == 200U);
    host_tick = 210U;
    robot_health_chassis_sample(true, 210U);
    s = snapshot();
    assert(s.i2c_consecutive_fail == 0U && s.chassis_sample_age_ms == 0U);
    assert(s.active_faults == 0U && s.historical_faults == ROBOT_FAULT_MOTOR);
    host_tick = 220U;
    robot_health_chassis_sample(false, 215U);
    s = snapshot();
    assert(s.last_fault_bits == ROBOT_FAULT_MOTOR && s.last_fault_timestamp == 220U);
    assert(s.historical_faults == ROBOT_FAULT_MOTOR); /* recurrence is still observable */
    host_tick = 225U;
    robot_health_chassis_sample(true, 225U);
    host_tick = 325U;
    assert((snapshot().active_faults & ROBOT_FAULT_MOTOR) == 0U);
    host_tick = 326U;
    assert((snapshot().active_faults & ROBOT_FAULT_MOTOR) != 0U);
}

static void test_hal_clock_overrides_stale_call_timestamp(void)
{
    HealthState_t s;
    fresh(1000U);
    host_tick = 1100U;
    robot_health_heartbeat(ROBOT_TASK_DEFAULT, 1100U);
    robot_health_chassis_sample(true, 1100U);
    robot_health_command_received(1100U, 50U);
    /* An older caller sampled now before preemption; unsigned subtraction must
     * not turn it into a huge age or a current fault in refresh_observation. */
    robot_health_set_current_faults(ROBOT_HEALTH_SOURCE_RUNTIME, 0U, 1000U);
    s = snapshot();
    assert(s.active_faults == 0U && s.historical_faults == 0U);
    assert(s.heartbeat_age_ms[ROBOT_TASK_DEFAULT] == 0U);
    assert(s.chassis_sample_age_ms == 0U && s.command_age_ms == 0U);
    assert(s.command_valid == 1U && s.uptime_ms == 1100U);
    host_tick = 1101U;
    robot_health_set_current_faults(ROBOT_HEALTH_SOURCE_ARM, ROBOT_FAULT_ARM, 500U);
    s = snapshot();
    assert(s.last_fault_timestamp == 1101U); /* actual HAL clock, not 500 */
    assert(s.active_faults == ROBOT_FAULT_ARM);
}

static void test_uint32_tick_wrap_and_age_clamp(void)
{
    const uint32_t start = UINT32_MAX - 15U;
    HealthState_t s;
    fresh(start);
    robot_health_heartbeat(ROBOT_TASK_DEFAULT, start);
    robot_health_chassis_sample(true, start);
    robot_health_command_received(start, 50U);
    host_tick = 16U; /* modulo elapsed = 32 ms */
    s = snapshot();
    assert(s.heartbeat_age_ms[ROBOT_TASK_DEFAULT] == 32U);
    assert(s.chassis_sample_age_ms == 32U && s.command_age_ms == 32U);
    assert(s.active_faults == 0U && s.command_valid == 1U);
    assert(!robot_health_heartbeat_expired(ROBOT_TASK_DEFAULT, host_tick, 100U));
    host_tick = 34U; /* elapsed == timeout */
    assert(snapshot().command_valid == 2U);
    host_tick = 85U; /* elapsed == 101; heartbeat and sample now expire */
    s = snapshot();
    assert((s.active_faults & ROBOT_FAULT_HEALTH) != 0U);
    assert((s.active_faults & ROBOT_FAULT_MOTOR) != 0U);
    assert(robot_health_heartbeat_expired(ROBOT_TASK_DEFAULT, host_tick, 100U));
    fresh(100U);
    robot_health_chassis_sample(true, 100U);
    robot_health_command_received(100U, 1000U);
    host_tick = 70100U;
    s = snapshot();
    assert(s.chassis_sample_age_ms == 65534U && s.command_age_ms == 65534U);
}

static void test_counter_saturation(void)
{
    HealthState_t s;
    unsigned i;
    fresh(0U);
    for (i = 0U; i < ROBOT_HEALTH_COUNTER_COUNT; ++i) {
        robot_health_count_by((RobotHealthCounter_t)i, 65534U);
        robot_health_count((RobotHealthCounter_t)i);
        robot_health_count_by((RobotHealthCounter_t)i, UINT32_MAX);
    }
    robot_health_count_by(ROBOT_HEALTH_COUNTER_COUNT, UINT32_MAX);
    robot_health_count_by(ROBOT_HEALTH_COUNTER_UART_CRC, 0U);
    for (i = 0U; i < 65540U; ++i) robot_health_chassis_sample(false, 0U);
    /* Public API would require >4 billion calls. Seed only this boundary. */
    robot_health_state.serial_tx_drop = UINT32_MAX - 1U;
    robot_health_tx_drop();
    robot_health_tx_drop();
    s = snapshot();
    for (i = 0U; i < ROBOT_HEALTH_COUNTER_COUNT; ++i) assert(s.counters[i] == UINT16_MAX);
    assert(s.i2c_consecutive_fail == UINT16_MAX);
    assert(s.serial_tx_drop == UINT32_MAX);
    robot_health_chassis_sample(true, 0U);
    assert(snapshot().i2c_consecutive_fail == 0U);
}

static void test_resources_timestamp_freezes_without_health_task(void)
{
    const uint16_t stacks[ROBOT_HEALTH_STACK_COUNT] = {100U, 200U, 300U, 400U, 500U};
    HealthState_t s;
    fresh(10U);
    host_tick = 20U;
    robot_health_heartbeat(ROBOT_TASK_HEALTH, 20U);
    robot_health_update_resources(20U, 2048U, stacks, ROBOT_FAULT_HEALTH, SAFETY_SAFE_IDLE);
    host_tick = 21U;
    robot_health_count(ROBOT_HEALTH_COUNTER_UART_CRC);
    robot_health_arm_observation(3U, 4U);
    robot_health_safety_event(6U, 19U);
    robot_health_note_history(ROBOT_FAULT_ARM, 18U);
    robot_health_chassis_sample(true, 21U);
    robot_health_command_received(21U, 1000U);
    robot_health_set_current_faults(ROBOT_HEALTH_SOURCE_SERIAL, ROBOT_FAULT_PROTOCOL, 1U);
    s = snapshot();
    assert(s.timestamp_ms == 20U && s.uptime_ms == 21U);
    assert(s.minimum_free_heap_bytes == 2048U);
    assert(memcmp(s.stack_free_words, stacks, sizeof(stacks)) == 0);
    assert(s.arm_status == 3U && s.arm_last_query == 4U);
    assert(s.last_event == 6U && s.last_event_timestamp == 19U);
    /* HEALTHTASK no longer calls heartbeat/update_resources. Observers continue. */
    host_tick = 271U;
    s = snapshot();
    assert(s.timestamp_ms == 20U && s.uptime_ms == 271U);
    assert(s.heartbeat_age_ms[ROBOT_TASK_HEALTH] == 251U);
    assert((s.active_faults & ROBOT_FAULT_HEALTH) != 0U);
    robot_health_update_resources(999U, 1U, NULL, 0U, SAFETY_NORMAL);
    assert(snapshot().timestamp_ms == 20U); /* rejected resource update */
    robot_health_update_resources(272U, 1024U, stacks, 0U, SAFETY_NORMAL);
    assert(snapshot().timestamp_ms == 272U);
}

static void test_estop_is_safety_state_not_current_fault(void)
{
    const uint16_t stacks[ROBOT_HEALTH_STACK_COUNT] = {1U, 2U, 3U, 4U, 5U};
    HealthState_t s;
    fresh(0U);
    robot_health_update_resources(0U, 1024U, stacks, ROBOT_FAULT_ESTOP, SAFETY_ESTOP_LATCHED);
    robot_health_set_current_faults(ROBOT_HEALTH_SOURCE_RUNTIME, ROBOT_FAULT_ESTOP, 0U);
    robot_health_note_history(ROBOT_FAULT_ESTOP, 0U);
    s = snapshot();
    assert(s.safety_state == SAFETY_ESTOP_LATCHED);
    assert(s.fault_flags == ROBOT_FAULT_ESTOP); /* legacy safety prefix is retained */
    assert(s.active_faults == 0U && s.historical_faults == 0U && s.last_fault_bits == 0U);
    robot_health_set_current_faults(ROBOT_HEALTH_SOURCE_ARM,
        ROBOT_FAULT_ARM | ROBOT_FAULT_ESTOP, 0U);
    assert(snapshot().active_faults == ROBOT_FAULT_ARM);
}

static void test_command_unknown_valid_expired_and_recovery(void)
{
    HealthState_t s;
    fresh(100U);
    assert(snapshot().command_valid == 0U);
    host_tick = 500U;
    assert(snapshot().command_age_ms == UINT16_MAX);
    robot_health_command_received(500U, 50U);
    s = snapshot();
    assert(s.command_valid == 1U && s.command_age_ms == 0U && s.active_faults == 0U);
    host_tick = 549U;
    assert(snapshot().command_valid == 1U);
    host_tick = 550U;
    s = snapshot();
    assert(s.command_valid == 2U && s.command_age_ms == 50U);
    assert(s.active_faults == ROBOT_FAULT_COMMAND_TIMEOUT);
    assert(s.last_fault_timestamp == 550U);
    host_tick = 560U;
    robot_health_command_received(560U, 50U);
    s = snapshot();
    assert(s.command_valid == 1U && s.active_faults == 0U);
    assert(s.historical_faults == ROBOT_FAULT_COMMAND_TIMEOUT);
    host_tick = 610U;
    assert(snapshot().last_fault_timestamp == 610U);
    fresh(0U);
    robot_health_command_received(0U, 0U);
    assert(snapshot().command_valid == 2U); /* API does not clamp caller timeout */
}

#define RUN(test) do { test(); assert(critical_depth == 0U); puts("PASS " #test); } while (0)
int main(void)
{
    RUN(test_snapshot_requires_initialization);
    RUN(test_init_unknown);
    RUN(test_heartbeat_updates_expiry_and_recovery);
    RUN(test_i2c_recovery_and_repeated_rising_fault);
    RUN(test_hal_clock_overrides_stale_call_timestamp);
    RUN(test_uint32_tick_wrap_and_age_clamp);
    RUN(test_counter_saturation);
    RUN(test_resources_timestamp_freezes_without_health_task);
    RUN(test_estop_is_safety_state_not_current_fault);
    RUN(test_command_unknown_valid_expired_and_recovery);
    assert(critical_entries > 0U);
    puts("PASS RobotHealth host observation state: 10 groups; reset never called");
    return 0;
}
