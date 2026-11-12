#include "boot_entry.h"

#define BOOT_REQUEST_MAGIC ((uint16_t)0xB007U)

/* 备份寄存器在软件复位后保留，用于通知启动代码进入应用 Bootloader。 */
static void boot_entry_store_request(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_BKP_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    WRITE_REG(BKP->DR1, BOOT_REQUEST_MAGIC);
    HAL_PWR_DisableBkUpAccess();
}

HAL_StatusTypeDef boot_entry_execute(void)
{
    /* SerialTask 必须先确认 ACK 已发送完毕并再次核对安全状态。 */
    boot_entry_store_request();
    __DSB();
    __ISB();
    NVIC_SystemReset();
    return HAL_OK;
}
