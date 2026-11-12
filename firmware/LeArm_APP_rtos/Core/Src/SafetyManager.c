#include "SafetyManager.h"

#include <string.h>

#define SAFETY_AXIS_NEUTRAL_MIN 115U
#define SAFETY_AXIS_NEUTRAL_MAX 139U
#define SAFETY_ESTOP_NEUTRAL_MS 500U

/* 所有状态变化统一经过该函数，便于诊断时保留上一次转换记录。 */
static void safety_manager_transition(SafetyManager_t *manager, SafetyState_t new_state,
                                      SafetyTransitionReason_t reason, uint32_t now_ms,
                                      uint16_t related_sequence)
{
    manager->last_transition.old_state = manager->state;
    manager->state = new_state;
    manager->last_transition.new_state = new_state;
    manager->last_transition.reason = reason;
    manager->last_transition.timestamp_ms = now_ms;
    manager->last_transition.related_sequence = related_sequence;
    manager->last_transition.fault_flags = manager->fault_flags;
}

static void safety_manager_clear_boot_authorization(SafetyManager_t *manager)
{
    manager->boot_authorized = false;
    manager->boot_authorized_sequence = 0U;
}

static void safety_manager_clear_motion_command(SafetyManager_t *manager)
{
    manager->last_motion_sequence = 0U;
    manager->motion_sequence_valid = false;
}

static bool safety_manager_axis_is_neutral(uint8_t axis)
{
    return (axis >= SAFETY_AXIS_NEUTRAL_MIN) && (axis <= SAFETY_AXIS_NEUTRAL_MAX);
}

static bool safety_manager_axes_are_neutral(const Ps2InputSnapshot_t *sample)
{
    return safety_manager_axis_is_neutral(sample->left_x) &&
           safety_manager_axis_is_neutral(sample->left_y) &&
           safety_manager_axis_is_neutral(sample->right_x) &&
           safety_manager_axis_is_neutral(sample->right_y);
}

static bool safety_manager_other_ps2_activity(const Ps2InputSnapshot_t *sample)
{
    return ((sample->buttons & (Ps2ButtonMask_t)~PS2_BUTTON_MODE) != 0U) ||
           !safety_manager_axes_are_neutral(sample);
}

static void safety_manager_latch_estop(SafetyManager_t *manager, uint32_t now_ms)
{
    /* 急停会同时废止旧运动命令、STOP 事务和 Bootloader 授权。 */
    safety_manager_clear_boot_authorization(manager);
    safety_manager_clear_motion_command(manager);
    manager->stop_transaction_valid = false;
    manager->chassis_stopped = false;
    manager->arm_stopped = false;
    manager->neutral_timer_valid = false;
    safety_manager_transition(manager, SAFETY_ESTOP_LATCHED, SAFETY_TRANSITION_ESTOP_ACTIVITY,
                              now_ms, 0U);
}

void safety_manager_init(SafetyManager_t *manager, uint32_t now_ms)
{
    if (manager == NULL) {
        return;
    }

    (void)memset(manager, 0, sizeof(*manager));
    manager->state = SAFETY_STARTUP;
    manager->last_transition.old_state = SAFETY_STARTUP;
    manager->last_transition.new_state = SAFETY_STARTUP;
    manager->last_transition.reason = SAFETY_TRANSITION_INIT;
    manager->last_transition.timestamp_ms = now_ms;
    manager->last_transition.related_sequence = 0U;
    manager->last_transition.fault_flags = ROBOT_FAULT_NONE;
}

void safety_manager_report_startup_ready(SafetyManager_t *manager, uint32_t now_ms)
{
    if ((manager != NULL) && (manager->state == SAFETY_STARTUP)) {
        safety_manager_transition(manager, SAFETY_SAFE_IDLE, SAFETY_TRANSITION_STARTUP_READY,
                                  now_ms, 0U);
    }
}

bool safety_manager_accept_motion(SafetyManager_t *manager, uint16_t sequence, uint32_t now_ms)
{
    if ((manager == NULL) || (manager->fault_flags != ROBOT_FAULT_NONE) ||
        ((manager->state != SAFETY_SAFE_IDLE) && (manager->state != SAFETY_NORMAL))) {
        return false;
    }

    safety_manager_clear_boot_authorization(manager);
    manager->stop_transaction_valid = false;
    manager->chassis_stopped = false;
    manager->arm_stopped = false;
    manager->last_motion_sequence = sequence;
    manager->motion_sequence_valid = true;
    safety_manager_transition(manager, SAFETY_NORMAL, SAFETY_TRANSITION_MOTION_ACCEPTED,
                              now_ms, sequence);
    return true;
}

SafetyState_t safety_manager_get_state(const SafetyManager_t *manager)
{
    return (manager == NULL) ? SAFETY_STARTUP : manager->state;
}

void safety_manager_on_ps2_sample(SafetyManager_t *manager, const Ps2InputSnapshot_t *sample,
                                  uint32_t now_ms)
{
    bool mode_pressed;
    bool other_activity;
    bool fully_neutral;

    if ((manager == NULL) || (sample == NULL) || (sample->valid == 0U)) {
        return;
    }

    mode_pressed = (sample->buttons & PS2_BUTTON_MODE) != 0U;
    other_activity = safety_manager_other_ps2_activity(sample);
    fully_neutral = (sample->buttons == 0U) && safety_manager_axes_are_neutral(sample);

    if (manager->state == SAFETY_FAULT) {
        manager->previous_mode_pressed = mode_pressed;
        return;
    }

    /*
     * 急停恢复分两步：先连续 500 ms 完全中立，再等待一次新的 MODE 上升沿。
     * MODE 一直按住不会跨越两个阶段直接恢复。
     */
    if (manager->state == SAFETY_ESTOP_LATCHED) {
        if (fully_neutral) {
            if (!manager->neutral_timer_valid) {
                manager->neutral_timer_valid = true;
                manager->neutral_start_ms = now_ms;
            } else if ((uint32_t)(now_ms - manager->neutral_start_ms) >= SAFETY_ESTOP_NEUTRAL_MS) {
                safety_manager_transition(manager, SAFETY_ESTOP_READY,
                                          SAFETY_TRANSITION_ESTOP_NEUTRAL_READY, now_ms, 0U);
                manager->neutral_timer_valid = false;
            }
        } else {
            manager->neutral_timer_valid = false;
        }
        manager->previous_mode_pressed = mode_pressed;
        return;
    }

    if (manager->state == SAFETY_ESTOP_READY) {
        if (other_activity) {
            safety_manager_latch_estop(manager, now_ms);
        } else if (mode_pressed && !manager->previous_mode_pressed) {
            safety_manager_clear_boot_authorization(manager);
            manager->stop_transaction_valid = false;
            manager->chassis_stopped = false;
            manager->arm_stopped = false;
            safety_manager_transition(manager, SAFETY_SAFE_IDLE,
                                      SAFETY_TRANSITION_ESTOP_MODE_RECOVERED, now_ms, 0U);
        }
        manager->previous_mode_pressed = mode_pressed;
        return;
    }

    if (other_activity || mode_pressed) {
        safety_manager_latch_estop(manager, now_ms);
    }
    manager->previous_mode_pressed = mode_pressed;
}

bool safety_manager_request_stop(SafetyManager_t *manager, uint16_t sequence, uint32_t now_ms)
{
    if ((manager == NULL) || (manager->fault_flags != ROBOT_FAULT_NONE) ||
        ((manager->state != SAFETY_SAFE_IDLE) && (manager->state != SAFETY_NORMAL))) {
        return false;
    }

    safety_manager_clear_boot_authorization(manager);
    safety_manager_clear_motion_command(manager);
    manager->stop_sequence = sequence;
    manager->stop_transaction_valid = true;
    manager->chassis_stopped = false;
    manager->arm_stopped = false;
    safety_manager_transition(manager, SAFETY_STOPPING, SAFETY_TRANSITION_STOP_REQUESTED,
                              now_ms, sequence);
    return true;
}

static void safety_manager_finish_stop_if_confirmed(SafetyManager_t *manager, uint32_t now_ms)
{
    uint16_t completed_sequence;

    /* 底盘和机械臂必须确认同一个 STOP 事务后才能授权进入 Bootloader。 */
    if ((manager->state != SAFETY_STOPPING) || !manager->stop_transaction_valid ||
        !manager->chassis_stopped || !manager->arm_stopped) {
        return;
    }

    completed_sequence = manager->stop_sequence;
    manager->stop_transaction_valid = false;
    safety_manager_clear_motion_command(manager);
    manager->boot_authorized_sequence = completed_sequence;
    manager->boot_authorized = true;
    safety_manager_transition(manager, SAFETY_SAFE_IDLE, SAFETY_TRANSITION_STOP_CONFIRMED,
                              now_ms, completed_sequence);
}

void safety_manager_report_chassis_stopped(SafetyManager_t *manager, bool stopped,
                                           uint32_t now_ms)
{
    if (manager == NULL) {
        return;
    }

    if (!stopped) {
        manager->chassis_stopped = false;
        return;
    }
    if ((manager->state == SAFETY_STOPPING) && manager->stop_transaction_valid) {
        manager->chassis_stopped = true;
        safety_manager_finish_stop_if_confirmed(manager, now_ms);
    }
}

void safety_manager_report_arm_stopped(SafetyManager_t *manager, bool stopped, uint32_t now_ms)
{
    if (manager == NULL) {
        return;
    }

    if (!stopped) {
        manager->arm_stopped = false;
        return;
    }
    if ((manager->state == SAFETY_STOPPING) && manager->stop_transaction_valid) {
        manager->arm_stopped = true;
        safety_manager_finish_stop_if_confirmed(manager, now_ms);
    }
}

SafetyBootDecision_t safety_manager_request_boot(SafetyManager_t *manager,
                                                 uint16_t stop_sequence, uint32_t now_ms)
{
    if (manager == NULL) {
        return BOOT_DENIED_STOP_REQUIRED;
    }
    if (manager->state == SAFETY_FAULT) {
        return BOOT_DENIED_FAULT;
    }
    if ((manager->state == SAFETY_ESTOP_LATCHED) || (manager->state == SAFETY_ESTOP_READY)) {
        return BOOT_DENIED_ESTOP;
    }
    if ((manager->state == SAFETY_NORMAL) || (manager->state == SAFETY_STOPPING) ||
        (manager->state == SAFETY_BOOT_PENDING) || (manager->state == SAFETY_STARTUP)) {
        return BOOT_DENIED_BUSY;
    }
    if (!manager->boot_authorized || (manager->boot_authorized_sequence != stop_sequence)) {
        return BOOT_DENIED_STOP_REQUIRED;
    }

    safety_manager_clear_boot_authorization(manager);
    safety_manager_clear_motion_command(manager);
    safety_manager_transition(manager, SAFETY_BOOT_PENDING, SAFETY_TRANSITION_BOOT_AUTHORIZED,
                              now_ms, stop_sequence);
    return BOOT_ALLOWED;
}

bool safety_manager_stop_completed(const SafetyManager_t *manager,
                                   uint16_t stop_sequence)
{
    return (manager != NULL) && (manager->state == SAFETY_SAFE_IDLE) &&
           manager->boot_authorized &&
           (manager->boot_authorized_sequence == stop_sequence);
}

void safety_manager_cancel_boot(SafetyManager_t *manager, uint32_t now_ms)
{
    if ((manager == NULL) || (manager->state != SAFETY_BOOT_PENDING)) {
        return;
    }

    safety_manager_clear_boot_authorization(manager);
    safety_manager_transition(manager, SAFETY_SAFE_IDLE,
                              SAFETY_TRANSITION_BOOT_CANCELLED,
                              now_ms, 0U);
}

void safety_manager_latch_fault(SafetyManager_t *manager, uint16_t fault_flags, uint32_t now_ms)
{
    if (manager == NULL) {
        return;
    }

    /* 故障位锁存累积；本阶段不在运行中自动清除故障。 */
    manager->fault_flags = (RobotFaultFlags_t)(manager->fault_flags | fault_flags);
    safety_manager_clear_boot_authorization(manager);
    safety_manager_clear_motion_command(manager);
    manager->stop_transaction_valid = false;
    manager->chassis_stopped = false;
    manager->arm_stopped = false;
    manager->neutral_timer_valid = false;
    safety_manager_transition(manager, SAFETY_FAULT, SAFETY_TRANSITION_FAULT_LATCHED,
                              now_ms, 0U);
}

bool safety_manager_motion_allowed(const SafetyManager_t *manager)
{
    return (manager != NULL) && (manager->state == SAFETY_NORMAL) &&
           (manager->fault_flags == ROBOT_FAULT_NONE);
}

bool safety_manager_stop_required(const SafetyManager_t *manager)
{
    return (manager == NULL) || (manager->state != SAFETY_SAFE_IDLE) || !manager->boot_authorized;
}
