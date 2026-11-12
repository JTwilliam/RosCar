#include "RobotHealth.h"

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f1xx_hal.h"

#include <limits.h>
#include <string.h>

static HealthState_t robot_health_state;
static uint32_t robot_health_last_heartbeat[ROBOT_TASK_COUNT];
static uint8_t robot_health_heartbeat_valid_mask;
static bool robot_health_initialized;
static uint16_t current_sources[ROBOT_HEALTH_SOURCE_COUNT];
static uint32_t last_chassis_sample_ms;
static bool chassis_sample_valid;
static uint32_t last_command_ms;
static uint16_t last_command_timeout_ms;
static bool command_seen;

static uint16_t age16(uint32_t age)
{ return (age > 65534U) ? 65534U : (uint16_t)age; }

static uint32_t elapsed(uint32_t now_ms, uint32_t then_ms)
{
    uint32_t delta = now_ms - then_ms;
    return (delta < 0x80000000UL) ? delta : 0U;
}

/* Caller holds the critical section: one coherent observation, never safety input. */
static void refresh_observation(uint32_t now_ms)
{
    static const uint16_t deadlines[ROBOT_TASK_COUNT] = {100U, 50U, 100U, 500U, 250U};
    uint8_t i;
    uint16_t active = 0U;
    uint16_t rising;
    /* Caller timestamp may predate preemption/I2C work; sample clock under the same lock. */
    now_ms = HAL_GetTick();
    for (i = 0U; i < ROBOT_HEALTH_SOURCE_COUNT; ++i) active |= current_sources[i];
    robot_health_state.heartbeat_valid_mask = robot_health_heartbeat_valid_mask;
    for (i = 0U; i < ROBOT_TASK_COUNT; ++i) {
        bool valid = (robot_health_heartbeat_valid_mask & (uint8_t)(1U << i)) != 0U;
        uint32_t age = elapsed(now_ms, robot_health_last_heartbeat[i]);
        robot_health_state.heartbeat_age_ms[i] = valid ? age16(age) : UINT16_MAX;
        if (valid && (age > deadlines[i])) active |= ROBOT_FAULT_HEALTH;
    }
    robot_health_state.chassis_sample_age_ms = chassis_sample_valid ?
        age16(elapsed(now_ms, last_chassis_sample_ms)) : UINT16_MAX;
    if (chassis_sample_valid && (elapsed(now_ms, last_chassis_sample_ms) > 100U))
        active |= ROBOT_FAULT_MOTOR;
    robot_health_state.command_age_ms = command_seen ? age16(elapsed(now_ms, last_command_ms)) : UINT16_MAX;
    robot_health_state.command_valid = command_seen ?
        ((elapsed(now_ms, last_command_ms) >= last_command_timeout_ms) ? 2U : 1U) : 0U;
    if (robot_health_state.command_valid == 2U) active |= ROBOT_FAULT_COMMAND_TIMEOUT;
    active = (uint16_t)(active & (uint16_t)~ROBOT_FAULT_ESTOP);
    rising = (uint16_t)(active & (uint16_t)~robot_health_state.active_faults);
    if (rising != 0U) {
        robot_health_state.last_fault_bits = rising;
        robot_health_state.last_fault_timestamp = now_ms;
    }
    robot_health_state.active_faults = active;
    robot_health_state.historical_faults |= active;
    robot_health_state.uptime_ms = now_ms;
}

/* 初始化仅建立软件状态，不启动看门狗或操作执行器。 */
void robot_health_init(uint32_t now_ms)
{
    RobotTaskId_t task;

    (void)memset(&robot_health_state, 0, sizeof(robot_health_state));
    (void)memset(robot_health_last_heartbeat, 0,
                 sizeof(robot_health_last_heartbeat));
    robot_health_state.timestamp_ms = now_ms;
    robot_health_state.safety_state = SAFETY_STARTUP;
    robot_health_heartbeat_valid_mask = 0U;
    (void)memset(current_sources, 0, sizeof(current_sources));
    chassis_sample_valid = false;
    command_seen = false;
    robot_health_state.schema = 1U;
    robot_health_state.chassis_sample_age_ms = UINT16_MAX;
    robot_health_state.command_age_ms = UINT16_MAX;
    for (task = 0U; task < ROBOT_TASK_COUNT; ++task) robot_health_state.heartbeat_age_ms[task] = UINT16_MAX;
    for (task = 0U; task < ROBOT_TASK_COUNT; ++task) {
        robot_health_last_heartbeat[task] = now_ms;
    }
    robot_health_initialized = true;
}

void robot_health_heartbeat(RobotTaskId_t task, uint32_t now_ms)
{
    if (!robot_health_initialized || (task >= ROBOT_TASK_COUNT)) {
        return;
    }

    /* 心跳时间和有效位必须作为一个一致更新提交。 */
    taskENTER_CRITICAL();
    robot_health_last_heartbeat[task] = now_ms;
    robot_health_heartbeat_valid_mask =
        (uint8_t)(robot_health_heartbeat_valid_mask | (uint8_t)(1U << task));
    taskEXIT_CRITICAL();
}

bool robot_health_heartbeat_expired(RobotTaskId_t task,
                                    uint32_t now_ms,
                                    uint32_t deadline_ms)
{
    uint32_t last_heartbeat;
    uint8_t valid_mask;

    if (!robot_health_initialized || (task >= ROBOT_TASK_COUNT)) {
        return true;
    }

    taskENTER_CRITICAL();
    last_heartbeat = robot_health_last_heartbeat[task];
    valid_mask = robot_health_heartbeat_valid_mask;
    taskEXIT_CRITICAL();

    if ((valid_mask & (uint8_t)(1U << task)) == 0U) {
        return false;
    }
    /* 无符号减法在合理截止时间范围内可自然处理 32 位 tick 回绕。 */
    return (uint32_t)(now_ms - last_heartbeat) > deadline_ms;
}

void robot_health_count_by(RobotHealthCounter_t counter, uint32_t amount)
{
    uint32_t value;

    if (!robot_health_initialized || (counter >= ROBOT_HEALTH_COUNTER_COUNT) ||
        (amount == 0U)) {
        return;
    }

    taskENTER_CRITICAL();
    value = robot_health_state.counters[counter];
    value = (amount > (uint32_t)UINT16_MAX - value) ? UINT16_MAX : value + amount;
    /* 饱和而不回绕，避免长期运行后严重故障重新看起来像零。 */
    robot_health_state.counters[counter] =
        (value > UINT16_MAX) ? UINT16_MAX : (uint16_t)value;
    taskEXIT_CRITICAL();
}

void robot_health_count(RobotHealthCounter_t counter)
{
    robot_health_count_by(counter, 1U);
}

void robot_health_update_resources(uint32_t now_ms,
                                   uint32_t minimum_free_heap_bytes,
                                   const uint16_t stack_free_words[ROBOT_HEALTH_STACK_COUNT],
                                   RobotFaultFlags_t fault_flags,
                                   SafetyState_t safety_state)
{
    if (!robot_health_initialized || (stack_free_words == NULL)) {
        return;
    }

    taskENTER_CRITICAL();
    robot_health_state.timestamp_ms = now_ms;
    robot_health_state.minimum_free_heap_bytes = minimum_free_heap_bytes;
    (void)memcpy(robot_health_state.stack_free_words,
                 stack_free_words,
                 sizeof(robot_health_state.stack_free_words));
    robot_health_state.fault_flags = fault_flags;
    robot_health_state.safety_state = safety_state;
    robot_health_state.reserved = 0U;
    taskEXIT_CRITICAL();
}

bool robot_health_get_snapshot(HealthState_t *snapshot_out)
{
    if (!robot_health_initialized || (snapshot_out == NULL)) {
        return false;
    }

    taskENTER_CRITICAL();
    refresh_observation(HAL_GetTick());
    *snapshot_out = robot_health_state;
    taskEXIT_CRITICAL();
    return true;
}

void robot_health_set_current_faults(uint8_t source, uint16_t bits, uint32_t now_ms)
{
    if (!robot_health_initialized || source >= ROBOT_HEALTH_SOURCE_COUNT) return;
    taskENTER_CRITICAL();
    current_sources[source] = (uint16_t)(bits & (uint16_t)~ROBOT_FAULT_ESTOP);
    refresh_observation(now_ms);
    taskEXIT_CRITICAL();
}

void robot_health_chassis_sample(bool success, uint32_t now_ms)
{
    if (!robot_health_initialized) return;
    taskENTER_CRITICAL();
    if (success) {
        chassis_sample_valid = true;
        last_chassis_sample_ms = now_ms;
        robot_health_state.i2c_consecutive_fail = 0U;
        current_sources[ROBOT_HEALTH_SOURCE_MOTOR_SAMPLE] = 0U;
    } else {
        if (robot_health_state.i2c_consecutive_fail < UINT16_MAX) ++robot_health_state.i2c_consecutive_fail;
        current_sources[ROBOT_HEALTH_SOURCE_MOTOR_SAMPLE] = ROBOT_FAULT_MOTOR;
    }
    refresh_observation(now_ms);
    taskEXIT_CRITICAL();
}

void robot_health_command_received(uint32_t now_ms, uint16_t timeout_ms)
{
    if (!robot_health_initialized) return;
    taskENTER_CRITICAL();
    command_seen = true; last_command_ms = now_ms; last_command_timeout_ms = timeout_ms;
    refresh_observation(now_ms);
    taskEXIT_CRITICAL();
}

void robot_health_arm_observation(uint8_t status, uint8_t last_query)
{
    if (!robot_health_initialized) return;
    taskENTER_CRITICAL();
    robot_health_state.arm_status = status;
    robot_health_state.arm_last_query = last_query;
    taskEXIT_CRITICAL();
}

void robot_health_safety_event(uint16_t reason, uint32_t timestamp_ms)
{
    if (!robot_health_initialized) return;
    taskENTER_CRITICAL();
    robot_health_state.last_event = reason;
    robot_health_state.last_event_timestamp = timestamp_ms;
    taskEXIT_CRITICAL();
}

void robot_health_note_history(uint16_t bits, uint32_t timestamp_ms)
{
    uint16_t fresh;
    if (!robot_health_initialized) return;
    bits = (uint16_t)(bits & (uint16_t)~ROBOT_FAULT_ESTOP);
    taskENTER_CRITICAL();
    fresh = (uint16_t)(bits & (uint16_t)~robot_health_state.historical_faults);
    robot_health_state.historical_faults |= bits;
    if (fresh != 0U) {
        robot_health_state.last_fault_bits = fresh;
        robot_health_state.last_fault_timestamp = timestamp_ms;
    }
    taskEXIT_CRITICAL();
}

void robot_health_tx_drop(void)
{
    if (!robot_health_initialized) return;
    taskENTER_CRITICAL();
    if (robot_health_state.serial_tx_drop < UINT32_MAX) ++robot_health_state.serial_tx_drop;
    taskEXIT_CRITICAL();
}

void robot_health_reset_with_reason(uint16_t reason_code)
{
    /* 异常上下文只保存最小复位原因，不再调用串口、Flash等复杂驱动。 */
    __disable_irq();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_BKP_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    WRITE_REG(BKP->DR2, reason_code);
    HAL_PWR_DisableBkUpAccess();
    __DSB();
    __ISB();
    NVIC_SystemReset();
    for (;;) {
    }
}
