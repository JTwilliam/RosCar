#ifndef HEALTH_HOST_FREERTOS_H
#define HEALTH_HOST_FREERTOS_H

typedef void *xTaskHandle;

/* Single-threaded observation tests; verify balanced critical sections. */
void health_host_enter_critical(void);
void health_host_exit_critical(void);
#define taskENTER_CRITICAL() health_host_enter_critical()
#define taskEXIT_CRITICAL() health_host_exit_critical()

#endif
