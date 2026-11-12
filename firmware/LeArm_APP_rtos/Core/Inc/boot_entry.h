#ifndef BOOT_ENTRY_H
#define BOOT_ENTRY_H

#include "stm32f1xx_hal.h"

/* 写入备份寄存器魔数并软件复位；调用前必须由安全状态机完成 STOP 授权。 */
HAL_StatusTypeDef boot_entry_execute(void);

#endif
