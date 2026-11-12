#include "RobotRtos.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* CubeMX 在 freertos.c 中创建对象；CMSIS 队列句柄底层对应 FreeRTOS Queue。 */
extern osMessageQueueId_t chassisCommandQueueHandle;
extern osMessageQueueId_t armRequestQueueHandle;

static ChassisState_t robot_rtos_chassis_state;
static ArmState_t robot_rtos_arm_state;

static bool robot_rtos_queue_has_capacity(osMessageQueueId_t queue_handle,
                                          uint32_t expected_capacity)
{
    return (queue_handle != NULL) &&
           (osMessageQueueGetCapacity(queue_handle) == expected_capacity);
}

static QueueHandle_t robot_rtos_chassis_command_queue(void)
{
    return (QueueHandle_t)chassisCommandQueueHandle;
}

static QueueHandle_t robot_rtos_arm_request_queue(void)
{
    return (QueueHandle_t)armRequestQueueHandle;
}

bool robot_rtos_publish_chassis_command(const ChassisCommand_t *command)
{
    QueueHandle_t queue_handle;

    if ((command == NULL) ||
        !robot_rtos_queue_has_capacity(chassisCommandQueueHandle,
                                       ROBOT_RTOS_CHASSIS_COMMAND_QUEUE_DEPTH)) {
        return false;
    }

    queue_handle = robot_rtos_chassis_command_queue();
    if (queue_handle == NULL) {
        return false;
    }

    /* 底盘只关心最新速度，覆盖旧命令可避免执行过期轨迹。 */
    return xQueueOverwrite(queue_handle, command) == pdPASS;
}

bool robot_rtos_take_chassis_command(ChassisCommand_t *command)
{
    QueueHandle_t queue_handle;

    if ((command == NULL) ||
        !robot_rtos_queue_has_capacity(chassisCommandQueueHandle,
                                       ROBOT_RTOS_CHASSIS_COMMAND_QUEUE_DEPTH)) {
        return false;
    }

    queue_handle = robot_rtos_chassis_command_queue();
    if (queue_handle == NULL) {
        return false;
    }

    return xQueueReceive(queue_handle, command, 0U) == pdPASS;
}

bool robot_rtos_enqueue_arm_command(const ArmCommand_t *command)
{
    QueueHandle_t queue_handle;

    if ((command == NULL) ||
        !robot_rtos_queue_has_capacity(armRequestQueueHandle,
                                       ROBOT_RTOS_ARM_REQUEST_QUEUE_DEPTH)) {
        return false;
    }

    queue_handle = robot_rtos_arm_request_queue();
    if (queue_handle == NULL) {
        return false;
    }

    /* 机械臂命令必须保持顺序；队列满时立即失败，由协议层回复 BUSY。 */
    return xQueueSend(queue_handle, command, 0U) == pdPASS;
}

bool robot_rtos_take_arm_command(ArmCommand_t *command)
{
    QueueHandle_t queue_handle;

    if ((command == NULL) ||
        !robot_rtos_queue_has_capacity(armRequestQueueHandle,
                                       ROBOT_RTOS_ARM_REQUEST_QUEUE_DEPTH)) {
        return false;
    }

    queue_handle = robot_rtos_arm_request_queue();
    if (queue_handle == NULL) {
        return false;
    }

    return xQueueReceive(queue_handle, command, 0U) == pdPASS;
}

void robot_rtos_clear_motion_commands(void)
{
    QueueHandle_t queue_handle;

    /* 调用者必须先关闭安全许可；单独清队列无法阻止并发生产者再次写入。 */
    if (robot_rtos_queue_has_capacity(chassisCommandQueueHandle,
                                      ROBOT_RTOS_CHASSIS_COMMAND_QUEUE_DEPTH)) {
        queue_handle = robot_rtos_chassis_command_queue();
        (void)xQueueReset(queue_handle);
    }

    if (robot_rtos_queue_has_capacity(armRequestQueueHandle,
                                      ROBOT_RTOS_ARM_REQUEST_QUEUE_DEPTH)) {
        queue_handle = robot_rtos_arm_request_queue();
        (void)xQueueReset(queue_handle);
    }
}

bool robot_rtos_is_ready(void)
{
    return robot_rtos_queue_has_capacity(chassisCommandQueueHandle,
                                         ROBOT_RTOS_CHASSIS_COMMAND_QUEUE_DEPTH) &&
           robot_rtos_queue_has_capacity(armRequestQueueHandle,
                                         ROBOT_RTOS_ARM_REQUEST_QUEUE_DEPTH) &&
           (robotEventFlagsHandle != NULL);
}

void robot_rtos_set_chassis_state(const ChassisState_t *state)
{
    if (state == NULL) {
        return;
    }

    /* 快照很小，临界区保证读者不会看到一半新、一半旧的数据。 */
    taskENTER_CRITICAL();
    robot_rtos_chassis_state = *state;
    taskEXIT_CRITICAL();
}

void robot_rtos_get_chassis_state(ChassisState_t *state)
{
    if (state == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    *state = robot_rtos_chassis_state;
    taskEXIT_CRITICAL();
}

void robot_rtos_set_arm_state(const ArmState_t *state)
{
    if (state == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    robot_rtos_arm_state = *state;
    taskEXIT_CRITICAL();
}

void robot_rtos_get_arm_state(ArmState_t *state)
{
    if (state == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    *state = robot_rtos_arm_state;
    taskEXIT_CRITICAL();
}
