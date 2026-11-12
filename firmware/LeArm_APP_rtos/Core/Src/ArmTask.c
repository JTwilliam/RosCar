#include "ArmTask.h"

#include "RobotHealth.h"
#include "RobotRtos.h"
#include "RobotSafety.h"
#include "robot_arm.h"

#include "cmsis_os.h"
#include "stm32f1xx_hal.h"

#include <string.h>

#define ARM_CMD_RESET  0x01U
#define ARM_CMD_JOINT  0x02U
#define ARM_CMD_ACTION 0x03U
#define ARM_CMD_STOP   0x04U

/*
 * ArmTask 是舵机、动作组和机械臂算法的唯一 owner。
 * SerialTask/PS2 映射层只能投递 ArmCommand_t，不能直接调用舵机接口。
 */
static uint16_t get_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static void arm_stop_all(uint32_t now_ms)
{
    uint8_t id;
    /* 先发出协作式中止，再停止动作组和各关节，最后确认机械臂已停。 */
    robot_arm_request_abort();
    robot_rtos_clear_motion_commands();
    action_group_stop();
    for (id = 1U; id <= MAX_SERVOS_NUM; ++id) robot_arm_knot_stop(id);
    robot_safety_report_arm_stopped(true, now_ms);
    if (robotEventFlagsHandle != NULL) {
        (void)osEventFlagsSet(robotEventFlagsHandle, ROBOT_EVT_ARM_STOPPED);
    }
}

static bool arm_dispatch(const ArmCommand_t *command)
{
    if (command == NULL) return false;
    switch (command->command) {
    case ARM_CMD_RESET:
        if (command->length != 2U) return false;
        robot_arm_reset(get_u16_le(command->payload));
        return true;
    case ARM_CMD_JOINT:
        if ((command->length != 5U) || (command->payload[0] < 1U) ||
            (command->payload[0] > MAX_SERVOS_NUM)) return false;
        robot_arm_knot_run(command->payload[0],
                           (int)get_u16_le(&command->payload[1]),
                           get_u16_le(&command->payload[3]));
        return true;
    case ARM_CMD_ACTION:
        if (command->length != 2U) return false;
        /* 动作组分片执行，每 1 ms 让出 CPU，并允许急停请求及时打断。 */
        while (!robot_arm_abort_requested() &&
               !action_group_run(command->payload[0], command->payload[1])) {
            (void)osDelay(1U);
        }
        action_group_reset();
        return !robot_arm_abort_requested();
    case ARM_CMD_STOP:
        return command->length == 0U;
    default:
        return false;
    }
}

void arm_task_run(void *argument)
{
    ArmState_t state;
    (void)argument;
    (void)memset(&state, 0, sizeof(state));
    /*
     * robot_arm_init() 内部包含舵机初始化、外部 Flash 检查和上电回原位。
     * 任一步失败都保持本任务存活并锁存故障，但不会继续接受运动命令。
     */
    if (!robot_arm_init()) {
        robot_health_arm_observation(3U, 0U);
        robot_health_set_current_faults(ROBOT_HEALTH_SOURCE_ARM, ROBOT_FAULT_ARM, HAL_GetTick());
        robot_safety_latch_fault(ROBOT_FAULT_ARM, HAL_GetTick());
        if (robotEventFlagsHandle != NULL) {
            (void)osEventFlagsSet(robotEventFlagsHandle,
                                  ROBOT_EVT_FAULT | ROBOT_EVT_STOP_REQUESTED);
        }
        for (;;) {
            robot_health_heartbeat(ROBOT_TASK_ARM, HAL_GetTick());
            (void)osDelay(100U);
        }
    }
    robot_arm_clear_abort();
    if (robotEventFlagsHandle != NULL) {
        (void)osEventFlagsSet(robotEventFlagsHandle, ROBOT_EVT_ARM_READY);
    }

    for (;;) {
        uint32_t now_ms = HAL_GetTick();
        uint32_t events = (robotEventFlagsHandle != NULL) ?
                          osEventFlagsGet(robotEventFlagsHandle) : ROBOT_EVT_FAULT;
        ArmCommand_t command;

        robot_health_heartbeat(ROBOT_TASK_ARM, now_ms);

        /* 安全许可撤销时，不论队列中是否还有命令，都先执行停止路径。 */
        if (!robot_safety_motion_allowed() ||
            ((events & (ROBOT_EVT_STOP_REQUESTED | ROBOT_EVT_ESTOP |
                        ROBOT_EVT_FAULT)) != 0U)) {
            arm_stop_all(now_ms);
            (void)osDelay(10U);
            continue;
        }

        if (robot_rtos_take_arm_command(&command)) {
            bool accepted;
            robot_arm_clear_abort();
            state.active_command = command.command;
            state.sequence = command.sequence;
            robot_health_arm_observation(2U, 0U);
            accepted = arm_dispatch(&command);
            /* No position/status query exists: dispatch completion is not proof of physical idle. */
            robot_health_arm_observation(0U, 0U);
            if (!accepted) {
                state.fault_flags |= ROBOT_FAULT_ARM;
                robot_health_count(ROBOT_HEALTH_COUNTER_ARM_REJECTION);
            }
            state.active_command = 0U;
        }

        state.timestamp_ms = now_ms;
        robot_rtos_set_arm_state(&state);
        (void)osDelay(10U);
    }
}
