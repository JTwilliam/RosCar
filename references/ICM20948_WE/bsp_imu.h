#ifndef __BSP_IMU_H
#define __BSP_IMU_H

#include <stdint.h>

#ifndef BSP_IMU_GYRO_Z_DEADBAND_RAD_S
#define BSP_IMU_GYRO_Z_DEADBAND_RAD_S (0.015f)
#endif

/* Calibration values measured with the final robot wiring and payload installed. */
#ifndef BSP_IMU_MAG_BIAS_X
#define BSP_IMU_MAG_BIAS_X    (-54.655f)
#endif
#ifndef BSP_IMU_MAG_BIAS_Y
#define BSP_IMU_MAG_BIAS_Y    (40.150f)
#endif
#ifndef BSP_IMU_MAG_BIAS_Z
#define BSP_IMU_MAG_BIAS_Z    (-19.885f)
#endif
#ifndef BSP_IMU_MAG_SCALE_X
#define BSP_IMU_MAG_SCALE_X   (1.0148f)
#endif
#ifndef BSP_IMU_MAG_SCALE_Y
#define BSP_IMU_MAG_SCALE_Y   (0.9233f)
#endif
#ifndef BSP_IMU_MAG_SCALE_Z
#define BSP_IMU_MAG_SCALE_Z   (1.0736f)
#endif

typedef struct
{ // 中间私有变量
    float x;
    float y;
    float z;
} PrivateBuf_t;

typedef struct
{
    PrivateBuf_t gyro;  // 3轴角速度
    PrivateBuf_t accel; // 3轴加速度
    PrivateBuf_t magn;  // 3轴磁力计
} IMU_DATA_t;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} ATTITUDE_DATA_t;

typedef enum
{
    IMU_DEVICE_ICM20948 = 0,
    IMU_DEVICE_MPU6500  = 1,
} IMUDevice_t;

typedef enum
{
    BSP_IMU_INIT_STAGE_IDLE = 0,
    BSP_IMU_INIT_STAGE_DETECT,
    BSP_IMU_INIT_STAGE_RESET,
    BSP_IMU_INIT_STAGE_CONFIG,
    BSP_IMU_INIT_STAGE_MAG,
    BSP_IMU_INIT_STAGE_READY,
} BSP_IMU_INIT_STAGE_t;

typedef struct
{
    IMUDevice_t           selected_device;
    BSP_IMU_INIT_STAGE_t  init_stage;
    uint8_t               last_addr_7bit;
    uint8_t               last_who_am_i;
    uint8_t               last_mag_wia;
    uint8_t               mag_available;
    uint8_t               init_ok;
} bsp_imu_diag_t;

#ifndef BSP_IMU_DEFAULT_DEVICE
#define BSP_IMU_DEFAULT_DEVICE IMU_DEVICE_ICM20948
#endif

/* 修改 BSP_IMU_DEFAULT_DEVICE 即可切换 IMU 型号。 */

// IMU操作接口
typedef struct
{
    uint8_t (*Init)(void);   // 返回值：1错误 0无异常
    uint8_t (*DeInit)(void); // 返回值：1错误 0无异常

    void (*UpdateZeroPoint_axis)(const IMU_DATA_t *point);                 // 更新9轴数据零点
    void (*UpdateZeroPoint_attitude)(const ATTITUDE_DATA_t *attitude);     // 更新姿态零点
    void (*Update_9axisVal)(IMU_DATA_t *imudata);                          // 更新9轴数据
    void (*UpdateAttitude)(IMU_DATA_t imudata, ATTITUDE_DATA_t *attitude); // 更新姿态角

} IMUInterface_t, *pIMUInterface_t;

void bsp_imu_select_device(IMUDevice_t device);
IMUDevice_t bsp_imu_get_selected_device(void);
const char * bsp_imu_get_device_name(IMUDevice_t device);
pIMUInterface_t bsp_imu_get_interface(void);
void bsp_imu_get_diag(bsp_imu_diag_t * p_diag);
bool bsp_imu_get_attitude(ATTITUDE_DATA_t * p_attitude);
bool bsp_imu_get_yaw(float * p_yaw_rad);
bool bsp_imu_get_magnetometer(PrivateBuf_t * p_magnetometer);

extern IMUInterface_t UserICM20948;
extern IMUInterface_t UserMPU9250;

#endif /* __BSP_IMU_H */
