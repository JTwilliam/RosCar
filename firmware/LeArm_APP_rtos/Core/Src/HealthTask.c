#include "HealthTask.h"

#include "RobotHealth.h"
#include "RobotRtos.h"
#include "RobotSafety.h"
#include "SerialTransport.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal.h"
#include "task.h"

#include <stdbool.h>
#include <stdint.h>

#define HEALTH_PERIOD_MS              100U
#define HEALTH_MIN_HEAP_BYTES         1024U
#define HEALTH_CHASSIS_STOP_TIMEOUT_MS 100U
#define HEALTH_ARM_STOP_TIMEOUT_MS     500U

#define HEALTH_DEFAULT_DEADLINE_MS 100U
#define HEALTH_CHASSIS_DEADLINE_MS  50U
#define HEALTH_SERIAL_DEADLINE_MS  100U
#define HEALTH_ARM_DEADLINE_MS     500U
#define HEALTH_SELF_DEADLINE_MS    250U

/*
 * HealthTask 统一检查任务心跳、栈余量、最小空闲堆和通信错误计数。
 * 它只发布故障/停止事件，不直接操作底盘或机械臂。
 */
extern osThreadId_t defaultTaskHandle;
extern osThreadId_t chassisTaskHandle;
extern osThreadId_t serialTaskHandle;
extern osThreadId_t armTaskHandle;
extern osThreadId_t healthTaskHandle;

static uint16_t health_stack_words(osThreadId_t thread)
{
    UBaseType_t words;

    if (thread == NULL) {
        return 0U;
    }
    /* 返回的是历史最小剩余栈，Cortex-M3 端口单位为 word。 */
    words = uxTaskGetStackHighWaterMark((TaskHandle_t)thread);
    return (words > UINT16_MAX) ? UINT16_MAX : (uint16_t)words;
}

static void health_add_transport_delta(uint32_t current,
                                       uint32_t *previous,
                                       RobotHealthCounter_t counter)
{
    uint32_t delta;

    /* 只累计本周期新增量，底层统计清零/回绕时从当前值重新开始。 */
    if (current >= *previous) {
        delta = current - *previous;
    } else {
        delta = current;
    }
    robot_health_count_by(counter, delta);
    *previous = current;
}

static bool health_heartbeat_failed(uint32_t now_ms)
{
    return robot_health_heartbeat_expired(ROBOT_TASK_DEFAULT,
                                          now_ms,
                                          HEALTH_DEFAULT_DEADLINE_MS) ||
           robot_health_heartbeat_expired(ROBOT_TASK_CHASSIS,
                                          now_ms,
                                          HEALTH_CHASSIS_DEADLINE_MS) ||
           robot_health_heartbeat_expired(ROBOT_TASK_SERIAL,
                                          now_ms,
                                          HEALTH_SERIAL_DEADLINE_MS) ||
           robot_health_heartbeat_expired(ROBOT_TASK_ARM,
                                          now_ms,
                                          HEALTH_ARM_DEADLINE_MS) ||
           robot_health_heartbeat_expired(ROBOT_TASK_HEALTH,
                                          now_ms,
                                          HEALTH_SELF_DEADLINE_MS);
}

void health_task_run(void *argument)
{
    static const uint16_t stack_warning_words[ROBOT_HEALTH_STACK_COUNT] = {
        32U, 64U, 80U, 80U, 40U
    };
    uint32_t previous_overflow = 0U;
    uint32_t previous_uart_error = 0U;
    uint32_t previous_restart = 0U;
    uint32_t previous_tx_timeout = 0U;
    uint32_t stop_started_ms = 0U;
    bool stop_tracking = false;
    bool startup_reported = false;
    bool health_fault_latched = false;

    (void)argument;

    for (;;) {
        uint32_t now_ms = HAL_GetTick();
        uint32_t events = (robotEventFlagsHandle != NULL) ?
                          osEventFlagsGet(robotEventFlagsHandle) :
                          ROBOT_EVT_FAULT;
        uint16_t stack_words[ROBOT_HEALTH_STACK_COUNT];
        uint32_t minimum_heap;
        RobotFaultFlags_t fault_flags = ROBOT_FAULT_NONE;
        SerialTransportStats_t transport_stats;
        ChassisState_t chassis_state;
        ArmState_t arm_state;
        uint8_t index;

        robot_health_heartbeat(ROBOT_TASK_HEALTH, now_ms);

        /* 四个硬件/通信 owner 都 READY 后，系统才允许离开 STARTUP。 */
        if (!startup_reported &&
            ((events & (ROBOT_EVT_CHASSIS_READY |
                        ROBOT_EVT_ARM_READY |
                        ROBOT_EVT_SERIAL_READY |
                        ROBOT_EVT_PS2_READY)) ==
             (ROBOT_EVT_CHASSIS_READY |
              ROBOT_EVT_ARM_READY |
              ROBOT_EVT_SERIAL_READY |
              ROBOT_EVT_PS2_READY))) {
            robot_safety_report_startup_ready(now_ms);
            startup_reported = true;
        }

        if ((events & ROBOT_EVT_STOP_REQUESTED) != 0U) {
            if (!stop_tracking) {
                stop_started_ms = now_ms;
                stop_tracking = true;
            }
            if ((events & ROBOT_EVT_CHASSIS_STOPPED) != 0U) {
                robot_safety_report_chassis_stopped(true, now_ms);
            }
            if ((events & ROBOT_EVT_ARM_STOPPED) != 0U) {
                robot_safety_report_arm_stopped(true, now_ms);
            }
            if ((events & (ROBOT_EVT_CHASSIS_STOPPED | ROBOT_EVT_ARM_STOPPED)) ==
                (ROBOT_EVT_CHASSIS_STOPPED | ROBOT_EVT_ARM_STOPPED)) {
                stop_tracking = false;
            } else if ((((events & ROBOT_EVT_CHASSIS_STOPPED) == 0U) &&
                        ((uint32_t)(now_ms - stop_started_ms) >
                         HEALTH_CHASSIS_STOP_TIMEOUT_MS)) ||
                       (((events & ROBOT_EVT_ARM_STOPPED) == 0U) &&
                        ((uint32_t)(now_ms - stop_started_ms) >
                         HEALTH_ARM_STOP_TIMEOUT_MS))) {
                fault_flags = (RobotFaultFlags_t)(fault_flags | ROBOT_FAULT_HEALTH);
            }
        } else {
            stop_tracking = false;
        }

        stack_words[ROBOT_TASK_DEFAULT] = health_stack_words(defaultTaskHandle);
        stack_words[ROBOT_TASK_CHASSIS] = health_stack_words(chassisTaskHandle);
        stack_words[ROBOT_TASK_SERIAL] = health_stack_words(serialTaskHandle);
        stack_words[ROBOT_TASK_ARM] = health_stack_words(armTaskHandle);
        stack_words[ROBOT_TASK_HEALTH] = health_stack_words(healthTaskHandle);
        minimum_heap = xPortGetMinimumEverFreeHeapSize();

        if (minimum_heap < HEALTH_MIN_HEAP_BYTES) {
            fault_flags = (RobotFaultFlags_t)(fault_flags | ROBOT_FAULT_HEALTH);
        }
        for (index = 0U; index < ROBOT_HEALTH_STACK_COUNT; ++index) {
            if ((stack_words[index] != 0U) &&
                (stack_words[index] < stack_warning_words[index])) {
                fault_flags = (RobotFaultFlags_t)(fault_flags | ROBOT_FAULT_HEALTH);
            }
        }
        /* 任一关键任务超期都会锁存健康故障并要求所有执行器停车。 */
        if (startup_reported && health_heartbeat_failed(now_ms)) {
            fault_flags = (RobotFaultFlags_t)(fault_flags | ROBOT_FAULT_HEALTH);
            if (!health_fault_latched) {
                robot_health_count(ROBOT_HEALTH_COUNTER_TASK_HEARTBEAT_MISS);
            }
        }

        /* Dynamic heartbeat expiry is derived atomically at snapshot time, so recovery
         * is visible immediately; historical stack/heap minima stay prefix-only. */
        robot_health_set_current_faults(ROBOT_HEALTH_SOURCE_RUNTIME, ROBOT_FAULT_NONE, now_ms);
        robot_rtos_get_chassis_state(&chassis_state);
        robot_rtos_get_arm_state(&arm_state);
        fault_flags = (RobotFaultFlags_t)(fault_flags |
                                          chassis_state.fault_flags |
                                          arm_state.fault_flags);
        if ((robot_safety_get_state() == SAFETY_ESTOP_LATCHED) ||
            (robot_safety_get_state() == SAFETY_ESTOP_READY)) {
            fault_flags = (RobotFaultFlags_t)(fault_flags | ROBOT_FAULT_ESTOP);
        }

        serial_transport_get_stats(&transport_stats);
        health_add_transport_delta(transport_stats.rx_overflow_count,
                                   &previous_overflow,
                                   ROBOT_HEALTH_COUNTER_UART_OVERFLOW);
        health_add_transport_delta(transport_stats.uart_error_count,
                                   &previous_uart_error,
                                   ROBOT_HEALTH_COUNTER_UART_FRAMING);
        health_add_transport_delta(transport_stats.rx_restart_count,
                                   &previous_restart,
                                   ROBOT_HEALTH_COUNTER_UART_RESTART);
        health_add_transport_delta(transport_stats.tx_timeout_count,
                                   &previous_tx_timeout,
                                   ROBOT_HEALTH_COUNTER_UART_RESTART);

        if ((fault_flags & ROBOT_FAULT_HEALTH) != 0U) {
            if (!health_fault_latched) {
                robot_safety_latch_fault(ROBOT_FAULT_HEALTH, now_ms);
                if (robotEventFlagsHandle != NULL) {
                    (void)osEventFlagsSet(robotEventFlagsHandle,
                                          ROBOT_EVT_FAULT |
                                          ROBOT_EVT_STOP_REQUESTED);
                }
                health_fault_latched = true;
            }
        }

        robot_health_update_resources(now_ms,
                                      minimum_heap,
                                      stack_words,
                                      fault_flags,
                                      robot_safety_get_state());
        (void)osDelay(HEALTH_PERIOD_MS);
    }
}
