#ifndef ROBOT_HEALTH_WIRE_H
#define ROBOT_HEALTH_WIRE_H
#include "RobotTypes.h"
#include <stddef.h>
#include <stdint.h>
#define ROBOT_HEALTH_WIRE_SIZE 128U
#define ROBOT_HEALTH_SCHEMA 1U
#ifdef __cplusplus
extern "C" {
#endif
/* Returns 128, or zero for invalid pointers/capacity; no RTOS/HAL dependency. */
size_t robot_health_wire_encode(const HealthState_t *state, uint8_t *out, size_t capacity);
#ifdef __cplusplus
}
#endif
#endif
