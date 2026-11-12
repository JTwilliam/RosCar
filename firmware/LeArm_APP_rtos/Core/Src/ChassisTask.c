#include "ChassisTask.h"

#include "ChassisController.h"
#include "RobotHealth.h"
#include "RobotRtos.h"
#include "RobotSafety.h"
#include "mecanum_chassis.h"
#include "motor_module.h"

#include "cmsis_os.h"
#include "stm32f1xx_hal.h"

#include <string.h>

#define CHASSIS_CONTROL_PERIOD_MS 10U
#define CHASSIS_SAMPLE_PERIOD_MS  20U
#define CHASSIS_I2C_TIMEOUT_MS     5U
#define CHASSIS_STOP_HOLD_PERIODS  2U

/*
 * ChassisTask 是底盘电机输出和电机模块 I2C 采样的唯一 owner。
 * 控制周期为 10 ms，编码器/电压采样周期为 20 ms；其他任务只能投递命令。
 */
static uint32_t ms_to_ticks(uint32_t milliseconds)
{
    uint32_t ticks = (osKernelGetTickFreq() * milliseconds + 999U) / 1000U;
    return (ticks == 0U) ? 1U : ticks;
}

static uint16_t command_timeout_ms(uint16_t requested)
{
    if (requested < ROBOT_COMMAND_TIMEOUT_MIN_MS) return ROBOT_COMMAND_TIMEOUT_MIN_MS;
    if (requested > ROBOT_COMMAND_TIMEOUT_MAX_MS) return ROBOT_COMMAND_TIMEOUT_MAX_MS;
    return requested;
}

void chassis_task_run(void *argument)
{
    ChassisCommand_t active_command;
    ChassisState_t state;
    uint32_t command_received_ms = 0U;
    uint32_t next_wake;
    uint32_t next_sample_ms;
    uint8_t stop_hold_periods = 0U;
    bool command_valid = false;

    (void)argument;
    (void)memset(&active_command, 0, sizeof(active_command));
    (void)memset(&state, 0, sizeof(state));
    mecanum_chassis_init();

    /* 硬件初始化可能阻塞，因此初始化完成后再建立绝对周期基准。 */
    next_wake = osKernelGetTickCount();
    next_sample_ms = HAL_GetTick();

    if (robotEventFlagsHandle != NULL) {
        (void)osEventFlagsSet(robotEventFlagsHandle, ROBOT_EVT_CHASSIS_READY);
    }

    for (;;) {
        ChassisCommand_t newest;
        uint32_t now_ms = HAL_GetTick();
        uint32_t events = (robotEventFlagsHandle != NULL) ?
                          osEventFlagsGet(robotEventFlagsHandle) : ROBOT_EVT_FAULT;
        bool must_stop;

        robot_health_heartbeat(ROBOT_TASK_CHASSIS, now_ms);

        /* 深度 1 邮箱只取最新命令，不回放已经过期的速度指令。 */
        if (robot_rtos_take_chassis_command(&newest)) {
            active_command = newest;
            active_command.timeout_ms = command_timeout_ms(newest.timeout_ms);
            command_received_ms = now_ms;
            command_valid = true;
            robot_health_command_received(now_ms, active_command.timeout_ms);
        }

        /* 树莓派停止刷新命令时，在命令有效期到期后主动请求安全停车。 */
        if (command_valid &&
            ((uint32_t)(now_ms - command_received_ms) >= active_command.timeout_ms)) {
            command_valid = false;
            state.fault_flags |= ROBOT_FAULT_COMMAND_TIMEOUT;
            robot_health_count(ROBOT_HEALTH_COUNTER_COMMAND_TIMEOUT);
            (void)robot_safety_request_stop(active_command.sequence, now_ms);
        }

        /* 任一条件不满足都输出零速度，安全状态优先于普通控制命令。 */
        must_stop = !command_valid || !robot_safety_motion_allowed() ||
                    ((events & (ROBOT_EVT_STOP_REQUESTED | ROBOT_EVT_ESTOP |
                                ROBOT_EVT_FAULT)) != 0U);

        if (must_stop) {
            bool output_ok = chassis_controller_stop();
            robot_health_set_current_faults(ROBOT_HEALTH_SOURCE_MOTOR_OUTPUT,
                output_ok ? ROBOT_FAULT_NONE : ROBOT_FAULT_MOTOR, now_ms);
            if (stop_hold_periods < CHASSIS_STOP_HOLD_PERIODS) stop_hold_periods++;
            /* 连续保持两个控制周期的零输出后，再向 STOP 事务确认已停车。 */
            if (stop_hold_periods >= CHASSIS_STOP_HOLD_PERIODS) {
                robot_safety_report_chassis_stopped(true, now_ms);
                if (robotEventFlagsHandle != NULL) {
                    (void)osEventFlagsSet(robotEventFlagsHandle, ROBOT_EVT_CHASSIS_STOPPED);
                }
            }
        } else {
            ChassisLegacyCommand_t legacy;
            stop_hold_periods = 0U;
            robot_safety_report_chassis_stopped(false, now_ms);
            if (robotEventFlagsHandle != NULL) {
                (void)osEventFlagsClear(robotEventFlagsHandle, ROBOT_EVT_CHASSIS_STOPPED);
            }
            chassis_controller_map(&active_command, &legacy);
            bool output_ok = chassis_controller_apply(&legacy);
            robot_health_set_current_faults(ROBOT_HEALTH_SOURCE_MOTOR_OUTPUT,
                output_ok ? ROBOT_FAULT_NONE : ROBOT_FAULT_MOTOR, now_ms);
            if (!output_ok) {
                state.fault_flags |= ROBOT_FAULT_MOTOR;
                robot_safety_latch_fault(ROBOT_FAULT_MOTOR, now_ms);
            }
        }

        if ((int32_t)(now_ms - next_sample_ms) >= 0) {
            bool encoder_ok = get_motor_encoder_timeout(state.encoder, CHASSIS_I2C_TIMEOUT_MS);
            bool voltage_ok = get_motor_module_voltage_timeout(&state.battery_mv,
                                                               CHASSIS_I2C_TIMEOUT_MS);
            robot_health_chassis_sample(encoder_ok && voltage_ok, HAL_GetTick());
            if (!encoder_ok || !voltage_ok) {
                state.fault_flags |= ROBOT_FAULT_MOTOR;
                robot_health_count(ROBOT_HEALTH_COUNTER_I2C_TIMEOUT);
            }
            motor_module_get_last_speed(state.motor_command);
            state.timestamp_ms = HAL_GetTick();
            state.safety_state = robot_safety_get_state();
            state.sequence = active_command.sequence;
            robot_rtos_set_chassis_state(&state);
            next_sample_ms += CHASSIS_SAMPLE_PERIOD_MS;
        }

        {
            uint32_t current_tick;

            next_wake += ms_to_ticks(CHASSIS_CONTROL_PERIOD_MS);
            current_tick = osKernelGetTickCount();

            if ((int32_t)(next_wake - current_tick) > 0) {
                (void)osDelayUntil(next_wake);
            } else {
                /*
                 * Resynchronize after an overrun and block for one tick so
                 * lower-priority communication and monitoring tasks can run.
                 */
                next_wake = current_tick;
                (void)osDelay(1U);
            }
        }
    }
}
