#include "ChassisController.h"

#include "mecanum_chassis.h"

#include <math.h>
#include <stdio.h>
#define CHASSIS_MAX_LINEAR_MM_S     500.0f
#define CHASSIS_MAX_ANGULAR_MRAD_S 2000.0f
#define CHASSIS_MAX_LEGACY_OUTPUT    80.0f
#define CHASSIS_RAD_TO_DEG           57.2957795f

/*
 * 将上位机使用的车体速度转换成原工程 mecanum_chassis_run() 的
 * “方向角 + 百分比速度 + 旋转量”接口，并在转换前进行限幅。
 */
static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

void chassis_controller_map(const ChassisCommand_t *command,
                            ChassisLegacyCommand_t *legacy_out)
{
    float vx;
    float vy;
    float magnitude;
    float angle;
    float rotation;

    if (legacy_out == NULL) {
        return;
    }

    legacy_out->angle_deg = 0U;
    legacy_out->speed = 0U;
    legacy_out->rotation = 0;
    if (command == NULL) {
        return;
    }

    /* 平移向量的模长映射到旧接口 0..80，方向使用 atan2 计算。 */
    vx = clamp_float((float)command->vx_mm_s, -CHASSIS_MAX_LINEAR_MM_S,
                     CHASSIS_MAX_LINEAR_MM_S);
    vy = clamp_float((float)command->vy_mm_s, -CHASSIS_MAX_LINEAR_MM_S,
                     CHASSIS_MAX_LINEAR_MM_S);
    magnitude = sqrtf(vx * vx + vy * vy);
    magnitude = clamp_float(magnitude, 0.0f, CHASSIS_MAX_LINEAR_MM_S);

    if (magnitude > 0.0f) {
        angle = atan2f(vy, vx) * CHASSIS_RAD_TO_DEG;
        if (angle < 0.0f) {
            angle += 360.0f;
        }
        legacy_out->angle_deg = (uint16_t)(angle + 0.5f);
        legacy_out->speed = (uint8_t)((magnitude * CHASSIS_MAX_LEGACY_OUTPUT /
                                       CHASSIS_MAX_LINEAR_MM_S) + 0.5f);
    }

    rotation = clamp_float((float)command->wz_mrad_s,
                           -CHASSIS_MAX_ANGULAR_MRAD_S,
                           CHASSIS_MAX_ANGULAR_MRAD_S);
    legacy_out->rotation = (int8_t)(rotation * CHASSIS_MAX_LEGACY_OUTPUT /
                                    CHASSIS_MAX_ANGULAR_MRAD_S);
}

bool chassis_controller_apply(const ChassisLegacyCommand_t *command)
{
    if (command == NULL) {
        return false;
    }
    mecanum_chassis_run(command->angle_deg, command->speed,
                        command->rotation, false);
    return true;
}

bool chassis_controller_stop(void)
{
    /* 所有安全停车路径最终都调用同一个零输出接口。 */
    mecanum_chassis_run(0U, 0U, 0, false);
    return true;
}
