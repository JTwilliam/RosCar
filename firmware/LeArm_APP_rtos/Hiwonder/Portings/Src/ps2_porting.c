#include "ps2_porting.h"

#include "usart.h"

#include <string.h>

/*
 * 本文件保留原手柄接收协议，但职责缩减为“收包 -> 输入快照”。
 * 不在 DMA 回调或解析函数中直接控制舵机/电机，避免破坏 RTOS owner 边界。
 */
static PS2HandleTypeDef ps2;

static void packet_uart_error_callback(UART_HandleTypeDef *huart);
static void packet_dma_receive_event_callback(UART_HandleTypeDef *huart,
                                              uint16_t length);

static void receive_data(uint8_t *data, uint16_t size)
{
    (void)HAL_UART_AbortReceive(&huart3);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart3, data, size);
}

static void packet_start_receive(void)
{
    (void)HAL_UART_RegisterCallback(&huart3, HAL_UART_ERROR_CB_ID,
                                    packet_uart_error_callback);
    (void)HAL_UART_RegisterRxEventCallback(&huart3,
                                           packet_dma_receive_event_callback);
    ps2.receive_data(ps2.rx_dma_buf, (uint16_t)sizeof(ps2.rx_dma_buf));
}

static void packet_dma_receive_event_callback(UART_HandleTypeDef *huart,
                                              uint16_t length)
{
    if ((huart == NULL) || (huart->Instance != USART3) || (length == 0U)) {
        return;
    }

    /* 回调只搬运本次 DMA 收到的字节，完整帧解析留给 DefaultTask。 */
    (void)lwrb_write(&ps2.rb, ps2.rx_dma_buf, length);
    ps2.receive_data(ps2.rx_dma_buf, (uint16_t)sizeof(ps2.rx_dma_buf));
}

static void packet_uart_error_callback(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart->Instance == USART3)) {
        packet_start_receive();
    }
}

void ps2_init(void)
{
    (void)memset(&ps2, 0, sizeof(ps2));
    ps2.receive_data = receive_data;
    ps2.packet_status = PS2_PACKET_HEADER_1;
    (void)lwrb_init(&ps2.rb, ps2.rx_fifo, sizeof(ps2.rx_fifo));
    packet_start_receive();
}

static void ps2_reset_candidate(PS2HandleTypeDef *self)
{
    self->packet_status = PS2_PACKET_HEADER_1;
    self->payload_index = 0U;
    self->expected_length = 0U;
}

static void ps2_fill_snapshot(const PS2HandleTypeDef *self,
                              Ps2InputSnapshot_t *snapshot)
{
    /*
     * 保留原接收帧格式：55 55 0A，后跟 9 字节负载。
     * payload[2]：三角/圆/叉/方块/L1/R1/L2/R2。
     * payload[3]：低 5 位依次为 SELECT/START/L3/R3/MODE。
     * payload[5..8]：左 X/Y、右 X/Y 摇杆原始值。
     */
    snapshot->timestamp_ms = HAL_GetTick();
    snapshot->buttons = (uint16_t)self->payload[2] |
                        ((uint16_t)(self->payload[3] & 0x1FU) << 8U);
    snapshot->left_x = self->payload[5];
    snapshot->left_y = self->payload[6];
    snapshot->right_x = self->payload[7];
    snapshot->right_y = self->payload[8];
    snapshot->valid = 1U;
    snapshot->reserved = 0U;
}

bool ps2_poll(Ps2InputSnapshot_t *snapshot_out)
{
    uint8_t byte;

    if (snapshot_out == NULL) {
        return false;
    }

    /* 按状态机逐字节寻找帧头、长度和固定长度负载，错误后重新找帧头。 */
    while (lwrb_read(&ps2.rb, &byte, 1U) == 1U) {
        switch (ps2.packet_status) {
        case PS2_PACKET_HEADER_1:
            if (byte == PS2_PACKET_HEADER) {
                ps2.packet_status = PS2_PACKET_HEADER_2;
            }
            break;

        case PS2_PACKET_HEADER_2:
            ps2.packet_status = (byte == PS2_PACKET_HEADER) ?
                                PS2_PACKET_DATA_LENGTH :
                                PS2_PACKET_HEADER_1;
            break;

        case PS2_PACKET_DATA_LENGTH:
            if (byte == PS2_PACKET_LENGTH) {
                ps2.expected_length = PS2_PACKET_PAYLOAD_LENGTH;
                ps2.payload_index = 0U;
                ps2.packet_status = PS2_PACKET_DATA;
            } else {
                ps2_reset_candidate(&ps2);
            }
            break;

        case PS2_PACKET_DATA:
            if (ps2.payload_index >= sizeof(ps2.payload)) {
                ps2_reset_candidate(&ps2);
                break;
            }

            ps2.payload[ps2.payload_index++] = byte;
            if (ps2.payload_index == ps2.expected_length) {
                Ps2InputSnapshot_t snapshot;

                ps2_fill_snapshot(&ps2, &snapshot);
                ps2_reset_candidate(&ps2);
                *snapshot_out = snapshot;
                return true;
            }
            break;

        default:
            ps2_reset_candidate(&ps2);
            break;
        }
    }

    return false;
}
