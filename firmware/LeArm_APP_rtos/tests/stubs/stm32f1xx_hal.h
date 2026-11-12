#ifndef HEALTH_HOST_STM32F1XX_HAL_H
#define HEALTH_HOST_STM32F1XX_HAL_H
#include <stdint.h>

uint32_t HAL_GetTick(void);

/* Compile-only declarations: reset code is never invoked and is linker-GC'd. */
typedef struct { volatile uint32_t DR2; } HealthHostBkpRegisters;
extern HealthHostBkpRegisters *health_host_bkp;
#define BKP health_host_bkp
void health_host_disable_irq(void);
void health_host_pwr_clock_enable(void);
void health_host_bkp_clock_enable(void);
void HAL_PWR_EnableBkUpAccess(void);
void HAL_PWR_DisableBkUpAccess(void);
void health_host_dsb(void);
void health_host_isb(void);
void NVIC_SystemReset(void);
#define __disable_irq() health_host_disable_irq()
#define __HAL_RCC_PWR_CLK_ENABLE() health_host_pwr_clock_enable()
#define __HAL_RCC_BKP_CLK_ENABLE() health_host_bkp_clock_enable()
#define WRITE_REG(reg, value) ((reg) = (value))
#define __DSB() health_host_dsb()
#define __ISB() health_host_isb()

#endif
