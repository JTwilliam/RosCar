#include "RobotSafety.h"
#include "RobotHealth.h"

#include "cmsis_os.h"
#include "stm32f1xx.h"

/*
 * SafetyManager 的 FreeRTOS 门面。纯状态机由互斥锁保护，所有 API 只允许
 * 在任务上下文使用；ISR 必须先通知任务，再由任务调用安全接口。
 */
static SafetyManager_t robot_safety_manager;
static osMutexId_t robot_safety_mutex;
static bool robot_safety_initialized;
static volatile bool robot_safety_internal_fault_pending;
static volatile bool robot_safety_pending_time_valid;
static volatile uint32_t robot_safety_pending_time_ms;
static const osMutexAttr_t robot_safety_mutex_attributes = {
    .name = "robotSafety"
};

/* Observation only, invoked while existing safety mutex is held. */
static void robot_safety_publish_health(void)
{
    robot_health_safety_event((uint16_t)robot_safety_manager.last_transition.reason,
        robot_safety_manager.last_transition.timestamp_ms);
    robot_health_note_history(robot_safety_manager.fault_flags,
        robot_safety_manager.last_transition.timestamp_ms);
}

static bool robot_safety_is_task_context(void)
{
    return __get_IPSR() == 0U;
}

static void robot_safety_note_internal_fault(uint32_t now_ms, bool time_valid)
{
    /* 无法安全访问状态机时采用 fail-closed：记录待锁存内部故障并禁止运动。 */
    if (time_valid) {
        robot_safety_pending_time_ms = now_ms;
        robot_safety_pending_time_valid = true;
    }
    robot_safety_internal_fault_pending = true;
}

static void robot_safety_latch_pending_fault(void)
{
    uint32_t timestamp_ms = 0U;

    if (!robot_safety_internal_fault_pending) {
        return;
    }
    if (robot_safety_pending_time_valid) {
        timestamp_ms = robot_safety_pending_time_ms;
    }
    safety_manager_latch_fault(&robot_safety_manager, ROBOT_FAULT_INTERNAL, timestamp_ms);
    robot_safety_internal_fault_pending = false;
    robot_safety_pending_time_valid = false;
}

static bool robot_safety_lock(uint32_t now_ms, bool time_valid)
{
    /* 未初始化、ISR误调用或互斥锁失败都不能被当作“允许继续运动”。 */
    if (!robot_safety_is_task_context() || !robot_safety_initialized ||
        (robot_safety_mutex == NULL)) {
        robot_safety_note_internal_fault(now_ms, time_valid);
        return false;
    }
    if (osMutexAcquire(robot_safety_mutex, osWaitForever) != osOK) {
        robot_safety_note_internal_fault(now_ms, time_valid);
        return false;
    }

    robot_safety_latch_pending_fault();
    return true;
}

bool robot_safety_init(uint32_t now_ms)
{
    /* CMSIS permits mutex creation in Ready state. Initialize before any
     * worker can run; runtime reinitialization must never clear a fault. */
    if (!robot_safety_is_task_context() ||
        (osKernelGetState() != osKernelReady)) {
        robot_safety_note_internal_fault(now_ms, true);
        return false;
    }
    if (robot_safety_initialized) {
        return false;
    }

    robot_safety_mutex = osMutexNew(&robot_safety_mutex_attributes);
    if (robot_safety_mutex == NULL) {
        robot_safety_note_internal_fault(now_ms, true);
        return false;
    }

    safety_manager_init(&robot_safety_manager, now_ms);
    robot_safety_initialized = true;
    robot_safety_latch_pending_fault();
    return true;
}

void robot_safety_report_startup_ready(uint32_t now_ms)
{
    if (robot_safety_lock(now_ms, true)) {
        safety_manager_report_startup_ready(&robot_safety_manager, now_ms);
        robot_safety_publish_health();
        (void)osMutexRelease(robot_safety_mutex);
    }
}

bool robot_safety_accept_motion(uint16_t sequence, uint32_t now_ms)
{
    bool accepted = false;

    if (robot_safety_internal_fault_pending) {
        return accepted;
    }
    if (robot_safety_lock(now_ms, true)) {
        accepted = safety_manager_accept_motion(&robot_safety_manager, sequence, now_ms);
        robot_safety_publish_health();
        (void)osMutexRelease(robot_safety_mutex);
    }
    return accepted;
}

SafetyState_t robot_safety_get_state(void)
{
    SafetyState_t state = SAFETY_FAULT;

    if (robot_safety_internal_fault_pending) {
        return state;
    }
    if (robot_safety_lock(0U, false)) {
        state = safety_manager_get_state(&robot_safety_manager);
        robot_safety_publish_health();
        (void)osMutexRelease(robot_safety_mutex);
    }
    return state;
}

void robot_safety_on_ps2_sample(const Ps2InputSnapshot_t *sample, uint32_t now_ms)
{
    if (robot_safety_lock(now_ms, true)) {
        safety_manager_on_ps2_sample(&robot_safety_manager, sample, now_ms);
        robot_safety_publish_health();
        (void)osMutexRelease(robot_safety_mutex);
    }
}

bool robot_safety_request_stop(uint16_t sequence, uint32_t now_ms)
{
    bool accepted = false;

    if (robot_safety_lock(now_ms, true)) {
        accepted = safety_manager_request_stop(&robot_safety_manager, sequence, now_ms);
        robot_safety_publish_health();
        (void)osMutexRelease(robot_safety_mutex);
    }
    return accepted;
}

void robot_safety_report_chassis_stopped(bool stopped, uint32_t now_ms)
{
    if (robot_safety_lock(now_ms, true)) {
        safety_manager_report_chassis_stopped(&robot_safety_manager, stopped, now_ms);
        robot_safety_publish_health();
        (void)osMutexRelease(robot_safety_mutex);
    }
}

void robot_safety_report_arm_stopped(bool stopped, uint32_t now_ms)
{
    if (robot_safety_lock(now_ms, true)) {
        safety_manager_report_arm_stopped(&robot_safety_manager, stopped, now_ms);
        robot_safety_publish_health();
        (void)osMutexRelease(robot_safety_mutex);
    }
}

SafetyBootDecision_t robot_safety_request_boot(uint16_t stop_sequence, uint32_t now_ms)
{
    SafetyBootDecision_t decision = BOOT_DENIED_FAULT;

    if (robot_safety_internal_fault_pending) {
        return decision;
    }
    if (robot_safety_lock(now_ms, true)) {
        decision = safety_manager_request_boot(&robot_safety_manager, stop_sequence, now_ms);
        robot_safety_publish_health();
        (void)osMutexRelease(robot_safety_mutex);
    }
    return decision;
}

bool robot_safety_stop_completed(uint16_t stop_sequence)
{
    bool completed = false;

    if (robot_safety_internal_fault_pending) {
        return false;
    }
    if (robot_safety_lock(0U, false)) {
        completed = safety_manager_stop_completed(&robot_safety_manager,
                                                  stop_sequence);
        (void)osMutexRelease(robot_safety_mutex);
    }
    return completed;
}

void robot_safety_cancel_boot(uint32_t now_ms)
{
    if (robot_safety_lock(now_ms, true)) {
        safety_manager_cancel_boot(&robot_safety_manager, now_ms);
        robot_safety_publish_health();
        (void)osMutexRelease(robot_safety_mutex);
    }
}

void robot_safety_latch_fault(uint16_t fault_flags, uint32_t now_ms)
{
    if (robot_safety_lock(now_ms, true)) {
        safety_manager_latch_fault(&robot_safety_manager, fault_flags, now_ms);
        robot_safety_publish_health();
        (void)osMutexRelease(robot_safety_mutex);
    }
}

bool robot_safety_motion_allowed(void)
{
    bool allowed = false;

    if (robot_safety_internal_fault_pending) {
        return allowed;
    }
    if (robot_safety_lock(0U, false)) {
        allowed = safety_manager_motion_allowed(&robot_safety_manager);
        (void)osMutexRelease(robot_safety_mutex);
    }
    return allowed;
}

bool robot_safety_stop_required(void)
{
    bool required = true;

    if (robot_safety_internal_fault_pending) {
        return required;
    }
    if (robot_safety_lock(0U, false)) {
        required = safety_manager_stop_required(&robot_safety_manager);
        (void)osMutexRelease(robot_safety_mutex);
    }
    return required;
}
