#ifndef ROBOT_RTOS_H
#define ROBOT_RTOS_H

#include "RobotTypes.h"

#include "cmsis_os.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ROBOT_RTOS_CHASSIS_COMMAND_QUEUE_DEPTH 1U
#define ROBOT_RTOS_ARM_REQUEST_QUEUE_DEPTH      4U

/*
 * 底盘命令只保留最新值，因此使用深度 1 的覆盖邮箱；机械臂命令需要保持
 * 执行顺序，因此使用深度 4 的 FIFO。事件标志只表示条件，不携带消息数据。
 */
#define ROBOT_EVT_STOP_REQUESTED  (1UL << 0)
#define ROBOT_EVT_CHASSIS_STOPPED (1UL << 1)
#define ROBOT_EVT_ARM_STOPPED     (1UL << 2)
#define ROBOT_EVT_ESTOP           (1UL << 3)
#define ROBOT_EVT_BOOT_PENDING    (1UL << 4)
#define ROBOT_EVT_FAULT           (1UL << 5)
#define ROBOT_EVT_CHASSIS_READY   (1UL << 6)
#define ROBOT_EVT_ARM_READY       (1UL << 7)
#define ROBOT_EVT_SERIAL_READY    (1UL << 8)
#define ROBOT_EVT_PS2_READY       (1UL << 9)

/* 由 MX_FREERTOS_Init() 在任务启动前创建。 */
extern osEventFlagsId_t robotEventFlagsHandle;

/*
 * 所有 RobotRtos API 仅允许在任务上下文调用。ISR 应先通过 FromISR 接口或
 * 通知把数据交给任务，不能直接调用这里的队列和快照函数。
 */
bool robot_rtos_publish_chassis_command(const ChassisCommand_t *command);
bool robot_rtos_take_chassis_command(ChassisCommand_t *command);
bool robot_rtos_enqueue_arm_command(const ArmCommand_t *command);
bool robot_rtos_take_arm_command(ArmCommand_t *command);

/*
 * 这里只清除调用瞬间已经存在的命令，不构成原子急停屏障。正确顺序是先由
 * SafetyManager 禁止运动，再清队列，最后由资源 owner 再次检查安全状态。
 */
void robot_rtos_clear_motion_commands(void);

/* 仅当两个队列容量和事件标志句柄都符合配置时返回 true。 */
bool robot_rtos_is_ready(void);

/* ChassisTask 是底盘快照的唯一写者，其他任务只能读取副本。 */
void robot_rtos_set_chassis_state(const ChassisState_t *state);
void robot_rtos_get_chassis_state(ChassisState_t *state);

/* ArmTask 是机械臂快照的唯一写者，其他任务只能读取副本。 */
void robot_rtos_set_arm_state(const ArmState_t *state);
void robot_rtos_get_arm_state(ArmState_t *state);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_RTOS_H */
