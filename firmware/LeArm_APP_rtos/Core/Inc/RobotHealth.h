#ifndef ROBOT_HEALTH_H
#define ROBOT_HEALTH_H

#include "RobotTypes.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 健康监控接口：任务周期性提交心跳，HealthTask 汇总栈、堆和通信错误。
 * 健康模块只上报和触发安全故障，不直接操作电机或舵机。
 */
typedef uint8_t RobotTaskId_t;

#define ROBOT_TASK_DEFAULT ((RobotTaskId_t)0U)
#define ROBOT_TASK_CHASSIS ((RobotTaskId_t)1U)
#define ROBOT_TASK_SERIAL  ((RobotTaskId_t)2U)
#define ROBOT_TASK_ARM     ((RobotTaskId_t)3U)
#define ROBOT_TASK_HEALTH  ((RobotTaskId_t)4U)
#define ROBOT_TASK_COUNT   ((RobotTaskId_t)5U)

void robot_health_init(uint32_t now_ms);
void robot_health_heartbeat(RobotTaskId_t task, uint32_t now_ms);
bool robot_health_heartbeat_expired(RobotTaskId_t task,
                                    uint32_t now_ms,
                                    uint32_t deadline_ms);
void robot_health_count(RobotHealthCounter_t counter);
void robot_health_count_by(RobotHealthCounter_t counter, uint32_t amount);
void robot_health_update_resources(uint32_t now_ms,
                                   uint32_t minimum_free_heap_bytes,
                                   const uint16_t stack_free_words[ROBOT_HEALTH_STACK_COUNT],
                                   RobotFaultFlags_t fault_flags,
                                   SafetyState_t safety_state);
/* Observation only: these APIs never grant motion or clear safety latches. */
#define ROBOT_HEALTH_SOURCE_RUNTIME 0U
#define ROBOT_HEALTH_SOURCE_MOTOR_SAMPLE 1U
#define ROBOT_HEALTH_SOURCE_MOTOR_OUTPUT 2U
#define ROBOT_HEALTH_SOURCE_ARM 3U
#define ROBOT_HEALTH_SOURCE_SERIAL 4U
#define ROBOT_HEALTH_SOURCE_COUNT 5U
void robot_health_set_current_faults(uint8_t source, uint16_t bits, uint32_t now_ms);
void robot_health_chassis_sample(bool success, uint32_t now_ms);
void robot_health_command_received(uint32_t now_ms, uint16_t timeout_ms);
void robot_health_arm_observation(uint8_t status, uint8_t last_query);
void robot_health_safety_event(uint16_t reason, uint32_t timestamp_ms);
void robot_health_note_history(uint16_t bits, uint32_t timestamp_ms);
void robot_health_tx_drop(void);
/* False before initialization or for NULL; caller must not serialize output. */
bool robot_health_get_snapshot(HealthState_t *snapshot_out);
void robot_health_reset_with_reason(uint16_t reason_code);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_HEALTH_H */
