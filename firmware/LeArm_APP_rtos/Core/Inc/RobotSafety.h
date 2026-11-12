#ifndef ROBOT_SAFETY_H
#define ROBOT_SAFETY_H

#include "SafetyManager.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * init 只在 osKernelInitialize 后、osKernelStart 前的 Ready 状态调用一次，
 * 由 MX_FREERTOS_Init 在创建任何业务任务前完成；失败时不得启动调度器。
 * 其余接口仅允许任务上下文调用；ISR 必须先把事件交给任务。
 */
bool robot_safety_init(uint32_t now_ms);
void robot_safety_report_startup_ready(uint32_t now_ms);
bool robot_safety_accept_motion(uint16_t sequence, uint32_t now_ms);
SafetyState_t robot_safety_get_state(void);
void robot_safety_on_ps2_sample(const Ps2InputSnapshot_t *sample, uint32_t now_ms);
bool robot_safety_request_stop(uint16_t sequence, uint32_t now_ms);
void robot_safety_report_chassis_stopped(bool stopped, uint32_t now_ms);
void robot_safety_report_arm_stopped(bool stopped, uint32_t now_ms);
SafetyBootDecision_t robot_safety_request_boot(uint16_t stop_sequence, uint32_t now_ms);
bool robot_safety_stop_completed(uint16_t stop_sequence);
void robot_safety_cancel_boot(uint32_t now_ms);
void robot_safety_latch_fault(uint16_t fault_flags, uint32_t now_ms);
bool robot_safety_motion_allowed(void);
bool robot_safety_stop_required(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_SAFETY_H */
