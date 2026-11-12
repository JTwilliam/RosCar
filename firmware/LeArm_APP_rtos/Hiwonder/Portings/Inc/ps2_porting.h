#ifndef __PS2_PORTING_H__
#define __PS2_PORTING_H__

#include "RobotTypes.h"
#include "lwrb.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * PS2 接收层只负责 USART3 帧解析并生成输入快照。
 * 当前版本不在本模块中直接调用底盘或机械臂控制函数。
 */
#define PS2_PACKET_HEADER         0x55U
#define PS2_PACKET_LENGTH         0x0AU
#define PS2_PACKET_PAYLOAD_LENGTH (PS2_PACKET_LENGTH - 1U)

typedef enum
{
    PS2_PACKET_HEADER_1 = 0,
    PS2_PACKET_HEADER_2,
    PS2_PACKET_DATA_LENGTH,
    PS2_PACKET_DATA
} PS2PackStatusTypeDef;

typedef struct
{
    uint8_t rx_dma_buf[128];
    uint8_t rx_fifo[256];
    uint8_t payload[PS2_PACKET_PAYLOAD_LENGTH];
    uint8_t payload_index;
    uint8_t expected_length;
    PS2PackStatusTypeDef packet_status;
    lwrb_t rb;
    void (*receive_data)(uint8_t *data, uint16_t size);
} PS2HandleTypeDef;

void ps2_init(void);
/* 每成功解析一帧返回 true；没有完整帧时立即返回 false。 */
bool ps2_poll(Ps2InputSnapshot_t *snapshot_out);

#ifdef __cplusplus
}
#endif

#endif /* __PS2_PORTING_H__ */
