#include "DefaultTask.h"

#include "RobotRtos.h"
#include "RobotHealth.h"
#include "RobotSafety.h"
#include "ps2_porting.h"

#include "cmsis_os.h"
#include "stm32f1xx_hal.h"

#define DEFAULT_TASK_PERIOD_MS 5U

/*
 * DefaultTask 只负责接收 PS2 快照并驱动安全状态机。
 * 当前版本不会在这里直接控制底盘或机械臂，避免多个任务同时操作执行器。
 */
static uint32_t default_task_period_ticks(void)
{
    uint32_t ticks = (osKernelGetTickFreq() * DEFAULT_TASK_PERIOD_MS + 999U) /
                     1000U;
    return (ticks == 0U) ? 1U : ticks;
}

void default_task_run(void *argument)
{
    uint32_t next_wake;
    const uint32_t period_ticks = default_task_period_ticks();
    Ps2InputSnapshot_t snapshot;
    SafetyState_t previous_state = SAFETY_STARTUP;

    (void)argument;
    /* USART3 DMA 和 PS2 解析器由本任务初始化并独占轮询。 */
    ps2_init();
    if (robotEventFlagsHandle != NULL) {
        (void)osEventFlagsSet(robotEventFlagsHandle, ROBOT_EVT_PS2_READY);
    }
    next_wake = osKernelGetTickCount();

    for (;;) {
        robot_health_heartbeat(ROBOT_TASK_DEFAULT, HAL_GetTick());
        /* 一次周期内消费所有已经完整接收的 PS2 帧。 */
        while (ps2_poll(&snapshot)) {
            SafetyState_t state;
            uint32_t event_bits = 0U;

            /* 无效或空闲帧不会触发急停；有效按键/摇杆活动由状态机判定。 */
            robot_safety_on_ps2_sample(&snapshot, snapshot.timestamp_ms);
            state = robot_safety_get_state();

            if (((state == SAFETY_ESTOP_LATCHED) ||
                 (state == SAFETY_ESTOP_READY)) &&
                ((previous_state != SAFETY_ESTOP_LATCHED) &&
                 (previous_state != SAFETY_ESTOP_READY))) {
                robot_health_count(ROBOT_HEALTH_COUNTER_PS2_ESTOP);
            }
            previous_state = state;

            if ((state == SAFETY_ESTOP_LATCHED) ||
                (state == SAFETY_ESTOP_READY)) {
                event_bits = ROBOT_EVT_ESTOP | ROBOT_EVT_STOP_REQUESTED;
            } else if (state == SAFETY_FAULT) {
                event_bits = ROBOT_EVT_FAULT | ROBOT_EVT_STOP_REQUESTED;
            }

            if ((event_bits != 0U) && (robotEventFlagsHandle != NULL)) {
                (void)osEventFlagsSet(robotEventFlagsHandle, event_bits);
            }
        }

        /* 使用绝对周期，避免处理时间逐次累积成调度漂移。 */
        next_wake += period_ticks;
        (void)osDelayUntil(next_wake);
    }
}
