#ifndef SERIAL_TRANSPORT_H
#define SERIAL_TRANSPORT_H

#include "stm32f1xx_hal.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * USART1 字节传输层：DMA/ISR 只搬运字节并通知 SerialTask，不解析协议。
 * TX 使用传输层自有静态缓冲区，保证 DMA 发送期间数据生命周期有效。
 */
typedef struct
{
    uint32_t rx_overflow_count;
    uint32_t uart_error_count;
    uint32_t rx_restart_count;
    uint32_t tx_timeout_count;
} SerialTransportStats_t;

HAL_StatusTypeDef serial_transport_init(UART_HandleTypeDef *uart);
/* 从软件环形缓冲中非阻塞读取，返回实际取得的字节数。 */
size_t serial_transport_read(uint8_t *destination, size_t capacity);
HAL_StatusTypeDef serial_transport_send(const uint8_t *data,
                                        size_t length,
                                        uint32_t timeout_ms);
HAL_StatusTypeDef serial_transport_shutdown(void);
void serial_transport_get_stats(SerialTransportStats_t *stats_out);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_TRANSPORT_H */
