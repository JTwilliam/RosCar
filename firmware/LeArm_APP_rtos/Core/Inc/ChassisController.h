#ifndef CHASSIS_CONTROLLER_H
#define CHASSIS_CONTROLLER_H

#include "RobotTypes.h"

#include <stdbool.h>
#include <stdint.h>

/* 原机械底盘接口使用角度/百分比，结构体用于承接速度命令转换结果。 */
typedef struct
{
    uint16_t angle_deg;
    uint8_t speed;
    int8_t rotation;
} ChassisLegacyCommand_t;

void chassis_controller_map(const ChassisCommand_t *command,
                            ChassisLegacyCommand_t *legacy_out);
/* 只有 ChassisTask 应调用 apply/stop，保持电机驱动的单一 owner。 */
bool chassis_controller_apply(const ChassisLegacyCommand_t *command);
bool chassis_controller_stop(void);

#endif /* CHASSIS_CONTROLLER_H */
