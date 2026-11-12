#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include "RobotTypes.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 与 HAL/FreeRTOS 无关的纯安全状态机。
 * 该层只记录状态和转换条件，不直接访问电机、舵机、串口或复位寄存器，
 * 因而可在主机端独立测试全部状态转换。
 */

typedef uint8_t SafetyBootDecision_t;

#define BOOT_ALLOWED               ((SafetyBootDecision_t)0U)
#define BOOT_DENIED_BUSY           ((SafetyBootDecision_t)1U)
#define BOOT_DENIED_STOP_REQUIRED  ((SafetyBootDecision_t)2U)
#define BOOT_DENIED_ESTOP          ((SafetyBootDecision_t)3U)
#define BOOT_DENIED_FAULT          ((SafetyBootDecision_t)4U)

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(SafetyBootDecision_t) == 1U,
               "SafetyBootDecision_t must remain one byte");
#endif

typedef enum {
    SAFETY_TRANSITION_NONE = 0,
    SAFETY_TRANSITION_INIT,
    SAFETY_TRANSITION_STARTUP_READY,
    SAFETY_TRANSITION_MOTION_ACCEPTED,
    SAFETY_TRANSITION_STOP_REQUESTED,
    SAFETY_TRANSITION_STOP_CONFIRMED,
    SAFETY_TRANSITION_ESTOP_ACTIVITY,
    SAFETY_TRANSITION_ESTOP_NEUTRAL_READY,
    SAFETY_TRANSITION_ESTOP_MODE_RECOVERED,
    SAFETY_TRANSITION_BOOT_AUTHORIZED,
    SAFETY_TRANSITION_BOOT_CANCELLED,
    SAFETY_TRANSITION_FAULT_LATCHED
} SafetyTransitionReason_t;

typedef struct {
    SafetyState_t old_state;
    SafetyState_t new_state;
    SafetyTransitionReason_t reason;
    uint32_t timestamp_ms;
    uint16_t related_sequence;
    RobotFaultFlags_t fault_flags;
} SafetyTransition_t;

/* 完整暴露内部状态，便于测试精确验证每次状态转换及其触发原因。 */
typedef struct {
    SafetyState_t state;
    bool previous_mode_pressed; /* 用于识别 MODE 新上升沿，避免长按误恢复。 */
    bool neutral_timer_valid;
    uint32_t neutral_start_ms;
    uint16_t stop_sequence;
    bool stop_transaction_valid; /* STOP 事务必须由底盘和机械臂共同确认。 */
    bool chassis_stopped;
    bool arm_stopped;
    uint16_t boot_authorized_sequence;
    bool boot_authorized; /* 只授权与 stop_sequence 匹配的一次 Boot 请求。 */
    RobotFaultFlags_t fault_flags;
    uint16_t last_motion_sequence;
    bool motion_sequence_valid;
    SafetyTransition_t last_transition;
} SafetyManager_t;

void safety_manager_init(SafetyManager_t *manager, uint32_t now_ms);
void safety_manager_report_startup_ready(SafetyManager_t *manager, uint32_t now_ms);
bool safety_manager_accept_motion(SafetyManager_t *manager, uint16_t sequence, uint32_t now_ms);
SafetyState_t safety_manager_get_state(const SafetyManager_t *manager);
void safety_manager_on_ps2_sample(SafetyManager_t *manager,
                                  const Ps2InputSnapshot_t *sample,
                                  uint32_t now_ms);
bool safety_manager_request_stop(SafetyManager_t *manager, uint16_t sequence, uint32_t now_ms);
void safety_manager_report_chassis_stopped(SafetyManager_t *manager, bool stopped,
                                           uint32_t now_ms);
void safety_manager_report_arm_stopped(SafetyManager_t *manager, bool stopped,
                                       uint32_t now_ms);
SafetyBootDecision_t safety_manager_request_boot(SafetyManager_t *manager,
                                                 uint16_t stop_sequence,
                                                 uint32_t now_ms);
bool safety_manager_stop_completed(const SafetyManager_t *manager,
                                   uint16_t stop_sequence);
void safety_manager_cancel_boot(SafetyManager_t *manager, uint32_t now_ms);
void safety_manager_latch_fault(SafetyManager_t *manager, uint16_t fault_flags,
                                uint32_t now_ms);
bool safety_manager_motion_allowed(const SafetyManager_t *manager);
bool safety_manager_stop_required(const SafetyManager_t *manager);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_MANAGER_H */
