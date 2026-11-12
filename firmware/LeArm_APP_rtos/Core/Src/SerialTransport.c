#include "SerialTransport.h"

#include "RobotProtocol.h"

#include "cmsis_os.h"

#include <stdbool.h>
#include <string.h>

#define SERIAL_RX_DMA_BUFFER_SIZE 128U
#define SERIAL_RX_RING_SIZE       512U

#define SERIAL_FLAG_RX_ACTIVITY (1UL << 0)
#define SERIAL_FLAG_TX_COMPLETE (1UL << 1)
#define SERIAL_FLAG_TX_ERROR    (1UL << 2)

/*
 * DMA 循环缓冲区由硬件写入，回调只把新增区间复制到软件环形缓冲。
 * SerialTask 是软件环形缓冲的唯一消费者和 TX 的唯一调用者。
 */
static UART_HandleTypeDef *serial_uart;
static osThreadId_t serial_owner_thread;
static uint8_t serial_rx_dma_buffer[SERIAL_RX_DMA_BUFFER_SIZE];
static uint8_t serial_rx_ring[SERIAL_RX_RING_SIZE];
static uint8_t serial_tx_buffer[ROBOT_PROTOCOL_MAX_ENCODED_FRAME];
static volatile uint16_t serial_rx_head;
static volatile uint16_t serial_rx_tail;
static volatile uint16_t serial_rx_last_position;
static volatile bool serial_rx_restart_pending;
static volatile bool serial_transport_initialized;
static volatile SerialTransportStats_t serial_stats;

static uint32_t serial_transport_ms_to_ticks(uint32_t milliseconds)
{
    uint32_t frequency = osKernelGetTickFreq();
    uint32_t ticks = (frequency * milliseconds + 999U) / 1000U;

    return (ticks == 0U) ? 1U : ticks;
}

static void serial_transport_notify(uint32_t flags)
{
    if (serial_owner_thread != NULL) {
        (void)osThreadFlagsSet(serial_owner_thread, flags);
    }
}

static void serial_transport_ring_write(const uint8_t *data, uint16_t length)
{
    uint16_t index;

    for (index = 0U; index < length; ++index) {
        uint16_t next = (uint16_t)((serial_rx_head + 1U) % SERIAL_RX_RING_SIZE);

        /* 环满时保留已有数据并计数，不覆盖尚未被任务消费的字节。 */
        if (next == serial_rx_tail) {
            if (serial_stats.rx_overflow_count != UINT32_MAX) {
                ++serial_stats.rx_overflow_count;
            }
            break;
        }
        serial_rx_ring[serial_rx_head] = data[index];
        serial_rx_head = next;
    }
}

static void serial_transport_rx_event_callback(UART_HandleTypeDef *uart,
                                               uint16_t position)
{
    uint16_t last;

    if ((uart != serial_uart) || (position > SERIAL_RX_DMA_BUFFER_SIZE)) {
        return;
    }

    /* position 是 DMA 当前写位置，不是协议帧边界；必须处理回绕区间。 */
    last = serial_rx_last_position;
    if (position > last) {
        serial_transport_ring_write(&serial_rx_dma_buffer[last],
                                    (uint16_t)(position - last));
    } else if (position < last) {
        serial_transport_ring_write(&serial_rx_dma_buffer[last],
                                    (uint16_t)(SERIAL_RX_DMA_BUFFER_SIZE - last));
        if (position != 0U) {
            serial_transport_ring_write(serial_rx_dma_buffer, position);
        }
    }

    serial_rx_last_position = (position == SERIAL_RX_DMA_BUFFER_SIZE) ? 0U : position;
    serial_transport_notify(SERIAL_FLAG_RX_ACTIVITY);
}

static void serial_transport_tx_complete_callback(UART_HandleTypeDef *uart)
{
    if (uart == serial_uart) {
        serial_transport_notify(SERIAL_FLAG_TX_COMPLETE);
    }
}

static void serial_transport_error_callback(UART_HandleTypeDef *uart)
{
    if (uart != serial_uart) {
        return;
    }

    if (serial_stats.uart_error_count != UINT32_MAX) {
        ++serial_stats.uart_error_count;
    }
    /* 回调不在此处做阻塞恢复，只置位并唤醒 owner 任务。 */
    serial_rx_restart_pending = true;
    serial_transport_notify(SERIAL_FLAG_RX_ACTIVITY | SERIAL_FLAG_TX_ERROR);
}

static HAL_StatusTypeDef serial_transport_start_rx(void)
{
    HAL_StatusTypeDef status;

    serial_rx_last_position = 0U;
    status = HAL_UARTEx_ReceiveToIdle_DMA(serial_uart,
                                         serial_rx_dma_buffer,
                                         SERIAL_RX_DMA_BUFFER_SIZE);
    if (status == HAL_OK) {
        serial_rx_restart_pending = false;
    }
    return status;
}

static void serial_transport_recover_rx(void)
{
    if (!serial_rx_restart_pending || (serial_uart == NULL)) {
        return;
    }

    (void)HAL_UART_AbortReceive(serial_uart);
    serial_rx_tail = serial_rx_head;
    if (serial_transport_start_rx() == HAL_OK) {
        if (serial_stats.rx_restart_count != UINT32_MAX) {
            ++serial_stats.rx_restart_count;
        }
    }
}

HAL_StatusTypeDef serial_transport_init(UART_HandleTypeDef *uart)
{
    if ((uart == NULL) || (uart->Instance != USART1) ||
        (osKernelGetState() != osKernelRunning)) {
        return HAL_ERROR;
    }

    serial_uart = uart;
    serial_owner_thread = osThreadGetId();
    serial_rx_head = 0U;
    serial_rx_tail = 0U;
    serial_rx_last_position = 0U;
    serial_rx_restart_pending = false;
    serial_transport_initialized = false;
    serial_stats.rx_overflow_count = 0U;
    serial_stats.uart_error_count = 0U;
    serial_stats.rx_restart_count = 0U;
    serial_stats.tx_timeout_count = 0U;

    if ((HAL_UART_RegisterCallback(serial_uart,
                                   HAL_UART_TX_COMPLETE_CB_ID,
                                   serial_transport_tx_complete_callback) != HAL_OK) ||
        (HAL_UART_RegisterCallback(serial_uart,
                                   HAL_UART_ERROR_CB_ID,
                                   serial_transport_error_callback) != HAL_OK) ||
        (HAL_UART_RegisterRxEventCallback(serial_uart,
                                          serial_transport_rx_event_callback) != HAL_OK)) {
        serial_uart = NULL;
        serial_owner_thread = NULL;
        return HAL_ERROR;
    }

    if (serial_transport_start_rx() != HAL_OK) {
        serial_uart = NULL;
        serial_owner_thread = NULL;
        return HAL_ERROR;
    }

    serial_transport_initialized = true;
    return HAL_OK;
}

size_t serial_transport_read(uint8_t *destination, size_t capacity)
{
    size_t count = 0U;

    if (!serial_transport_initialized || (destination == NULL) ||
        (capacity == 0U) || (osThreadGetId() != serial_owner_thread)) {
        return 0U;
    }

    serial_transport_recover_rx();
    (void)osThreadFlagsClear(SERIAL_FLAG_RX_ACTIVITY);
    while ((count < capacity) && (serial_rx_tail != serial_rx_head)) {
        destination[count] = serial_rx_ring[serial_rx_tail];
        serial_rx_tail = (uint16_t)((serial_rx_tail + 1U) % SERIAL_RX_RING_SIZE);
        ++count;
    }
    return count;
}

HAL_StatusTypeDef serial_transport_send(const uint8_t *data,
                                        size_t length,
                                        uint32_t timeout_ms)
{
    uint32_t flags;
    HAL_StatusTypeDef status;

    if (!serial_transport_initialized || (data == NULL) || (length == 0U) ||
        (length > sizeof(serial_tx_buffer)) ||
        (osThreadGetId() != serial_owner_thread)) {
        return HAL_ERROR;
    }

    /* 复制到静态缓冲，避免 DMA 引用调用者已经失效的栈内存。 */
    (void)memcpy(serial_tx_buffer, data, length);
    (void)osThreadFlagsClear(SERIAL_FLAG_TX_COMPLETE | SERIAL_FLAG_TX_ERROR);
    status = HAL_UART_Transmit_DMA(serial_uart, serial_tx_buffer, (uint16_t)length);
    if (status != HAL_OK) {
        return status;
    }

    flags = osThreadFlagsWait(SERIAL_FLAG_TX_COMPLETE | SERIAL_FLAG_TX_ERROR,
                              osFlagsWaitAny,
                              serial_transport_ms_to_ticks(timeout_ms));
    if ((flags & osFlagsError) != 0U) {
        (void)HAL_UART_AbortTransmit(serial_uart);
        if (serial_stats.tx_timeout_count != UINT32_MAX) {
            ++serial_stats.tx_timeout_count;
        }
        return HAL_TIMEOUT;
    }
    if ((flags & SERIAL_FLAG_TX_ERROR) != 0U) {
        (void)HAL_UART_AbortTransmit(serial_uart);
        return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef serial_transport_shutdown(void)
{
    HAL_StatusTypeDef status;

    if (!serial_transport_initialized || (serial_uart == NULL) ||
        (osThreadGetId() != serial_owner_thread)) {
        return HAL_ERROR;
    }

    status = HAL_UART_DMAStop(serial_uart);
    __HAL_UART_DISABLE_IT(serial_uart, UART_IT_IDLE);
    serial_transport_initialized = false;
    serial_rx_restart_pending = false;
    return status;
}

void serial_transport_get_stats(SerialTransportStats_t *stats_out)
{
    if (stats_out != NULL) {
        uint32_t primask = __get_PRIMASK();

        __disable_irq();
        stats_out->rx_overflow_count = serial_stats.rx_overflow_count;
        stats_out->uart_error_count = serial_stats.uart_error_count;
        stats_out->rx_restart_count = serial_stats.rx_restart_count;
        stats_out->tx_timeout_count = serial_stats.tx_timeout_count;
        if (primask == 0U) {
            __enable_irq();
        }
    }
}
