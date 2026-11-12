#ifndef SAFETY_HOST_CMSIS_OS_H
#define SAFETY_HOST_CMSIS_OS_H
#include <stddef.h>
#include <stdint.h>

/* Only the CMSIS interface used by the actual startup/safety files is mocked. */
typedef void *osMutexId_t;
typedef void *osThreadId_t;
typedef void *osMessageQueueId_t;
typedef void *osEventFlagsId_t;
typedef int32_t osStatus_t;
typedef enum { osKernelInactive, osKernelReady, osKernelRunning,
               osKernelLocked } osKernelState_t;
typedef enum { osPriorityBelowNormal = 16, osPriorityNormal = 24,
               osPriorityAboveNormal = 32, osPriorityHigh = 40 } osPriority_t;
typedef struct { const char *name; uint32_t attr_bits; void *cb_mem;
                 uint32_t cb_size; } osMutexAttr_t;
typedef struct { const char *name; uint32_t attr_bits; void *cb_mem;
                 uint32_t cb_size; void *stack_mem; uint32_t stack_size;
                 osPriority_t priority; uint32_t tz_module; uint32_t reserved;
               } osThreadAttr_t;
typedef struct { const char *name; uint32_t attr_bits; void *cb_mem;
                 uint32_t cb_size; void *mq_mem; uint32_t mq_size;
               } osMessageQueueAttr_t;
typedef osMutexAttr_t osEventFlagsAttr_t;
#define osOK ((osStatus_t)0)
#define osError ((osStatus_t)-1)
#define osWaitForever UINT32_MAX

osKernelState_t osKernelGetState(void);
osMutexId_t osMutexNew(const osMutexAttr_t *attr);
osStatus_t osMutexAcquire(osMutexId_t mutex, uint32_t timeout);
osStatus_t osMutexRelease(osMutexId_t mutex);
osThreadId_t osThreadNew(void (*function)(void *), void *argument,
                         const osThreadAttr_t *attr);
void osThreadExit(void);
osMessageQueueId_t osMessageQueueNew(uint32_t count, uint32_t size,
                                     const osMessageQueueAttr_t *attr);
osEventFlagsId_t osEventFlagsNew(const osEventFlagsAttr_t *attr);
#endif
