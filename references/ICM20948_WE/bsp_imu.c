#include <BSP/IIC/bsp_iic.h>
#include <BSP/IMU/bsp_imu.h>
#include <BSP/IMU/icm_20948_reg.h>
#include <BSP/IMU/mpu9250_reg.h>
#include "bsp_api.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

typedef struct
{
    IMU_DATA_t        zero_point;
    ATTITUDE_DATA_t   zero_attitude;
    IMU_DATA_t        lpf_state;
    uint8_t           lpf_ready;
    float             e_int[3];
    float             q[4];
    float             mag_reference_norm;
    uint32_t          last_tick_ms;
    ATTITUDE_DATA_t   latest_attitude;
    uint8_t           attitude_ready;
    uint8_t           latest_attitude_valid;
} imu_runtime_state_t;

static imu_runtime_state_t g_icm20948_state = {0};
static imu_runtime_state_t g_mpu9250_state = {0};
static IMUDevice_t g_selected_device = BSP_IMU_DEFAULT_DEVICE;
static BSP_IMU_INIT_STAGE_t g_icm20948_init_stage = BSP_IMU_INIT_STAGE_IDLE;
static uint8_t g_icm20948_who_am_i = 0U;
static uint8_t g_icm20948_mag_wia = 0U;
static uint8_t g_icm20948_mag_available = 0U;
static BSP_IMU_INIT_STAGE_t g_mpu9250_init_stage = BSP_IMU_INIT_STAGE_IDLE;
static uint8_t g_mpu9250_addr_7bit = MPU9250_DEV_ADDR;
static uint8_t g_mpu9250_who_am_i = 0U;
static uint8_t g_mpu9250_mag_available = 0U;
static uint8_t g_mpu9250_mag_wia = 0U;

/* IMU sampling LPF: higher alpha means stronger smoothing. */
#define IMU_GYRO_LPF_ALPHA     (0.80f)
#define IMU_ACCEL_LPF_ALPHA    (0.70f)
#define IMU_MAG_LPF_ALPHA      (0.85f)
#define IMU_EINT_LIMIT         (2.0f)
#define IMU_MIN_DT_S           (0.001f)
#define IMU_MAX_DT_S           (0.050f)
#define IMU_DEFAULT_DT_S       (0.020f)
#define IMU_ACCEL_SCALE_2G     (0.00059814453125f)
#define IMU_GYRO_SCALE_2000DPS (0.06103515625f)
#define IMU_DEG_TO_RAD         (0.01745329252f)
#define IMU_MAG_SCALE_UT       (0.1495361328125f)

static float imu_clampf(float value, float min_v, float max_v)
{
    if (value < min_v)
    {
        return min_v;
    }
    if (value > max_v)
    {
        return max_v;
    }
    return value;
}

static float imu_lpf(float prev, float cur, float alpha)
{
    return alpha * prev + (1.0f - alpha) * cur;
}

static float imu_get_delta_time_s(imu_runtime_state_t * p_state)
{
    uint32_t now_ms;
    uint32_t delta_ms;

    if (NULL == p_state)
    {
        return IMU_DEFAULT_DT_S;
    }

    now_ms = (uint32_t) (xTaskGetTickCount() * portTICK_PERIOD_MS);

    if (0U == p_state->last_tick_ms)
    {
        p_state->last_tick_ms = now_ms;
        return IMU_DEFAULT_DT_S;
    }

    delta_ms = now_ms - p_state->last_tick_ms;
    p_state->last_tick_ms = now_ms;

    if (0U == delta_ms)
    {
        return IMU_MIN_DT_S;
    }

    if (delta_ms > (uint32_t) (IMU_MAX_DT_S * 1000.0f))
    {
        return IMU_MAX_DT_S;
    }

    return (float) delta_ms * 0.001f;
}

static void imu_runtime_reset(imu_runtime_state_t * p_state)
{
    if (NULL == p_state)
    {
        return;
    }

    memset(p_state, 0, sizeof(*p_state));
    p_state->q[0] = 1.0f;
}

static void imu_init_attitude_from_accel(imu_runtime_state_t * p_state, IMU_DATA_t const * p_data)
{
    float roll;
    float pitch;
    float yaw = 0.0f;
    float norm;
    float cr;
    float sr;
    float cp;
    float sp;
    float cy;
    float sy;

    if ((NULL == p_state) || (NULL == p_data))
    {
        return;
    }

    norm = sqrtf((p_data->accel.x * p_data->accel.x) +
                 (p_data->accel.y * p_data->accel.y) +
                 (p_data->accel.z * p_data->accel.z));
    if (norm < 0.001f)
    {
        return;
    }

    roll = atan2f(p_data->accel.y, p_data->accel.z);
    pitch = atan2f(-p_data->accel.x,
                   sqrtf((p_data->accel.y * p_data->accel.y) +
                         (p_data->accel.z * p_data->accel.z)));

    cr = cosf(roll * 0.5f);
    sr = sinf(roll * 0.5f);
    cp = cosf(pitch * 0.5f);
    sp = sinf(pitch * 0.5f);

    cy = cosf(yaw * 0.5f);
    sy = sinf(yaw * 0.5f);
    p_state->q[0] = (cr * cp * cy) + (sr * sp * sy);
    p_state->q[1] = (sr * cp * cy) - (cr * sp * sy);
    p_state->q[2] = (cr * sp * cy) + (sr * cp * sy);
    p_state->q[3] = (cr * cp * sy) - (sr * sp * cy);
    p_state->e_int[0] = 0.0f;
    p_state->e_int[1] = 0.0f;
    p_state->e_int[2] = 0.0f;
    p_state->last_tick_ms = 0U;
    p_state->attitude_ready = 1U;
}

static void imu_apply_lpf(IMU_DATA_t * p_data, imu_runtime_state_t * p_state)
{
    if ((NULL == p_data) || (NULL == p_state))
    {
        return;
    }

    if (0U == p_state->lpf_ready)
    {
        p_state->lpf_state = *p_data;
        p_state->lpf_ready = 1U;
        return;
    }

    p_state->lpf_state.accel.x = imu_lpf(p_state->lpf_state.accel.x, p_data->accel.x, IMU_ACCEL_LPF_ALPHA);
    p_state->lpf_state.accel.y = imu_lpf(p_state->lpf_state.accel.y, p_data->accel.y, IMU_ACCEL_LPF_ALPHA);
    p_state->lpf_state.accel.z = imu_lpf(p_state->lpf_state.accel.z, p_data->accel.z, IMU_ACCEL_LPF_ALPHA);

    p_state->lpf_state.gyro.x = imu_lpf(p_state->lpf_state.gyro.x, p_data->gyro.x, IMU_GYRO_LPF_ALPHA);
    p_state->lpf_state.gyro.y = imu_lpf(p_state->lpf_state.gyro.y, p_data->gyro.y, IMU_GYRO_LPF_ALPHA);
    p_state->lpf_state.gyro.z = imu_lpf(p_state->lpf_state.gyro.z, p_data->gyro.z, IMU_GYRO_LPF_ALPHA);

    p_state->lpf_state.magn.x = imu_lpf(p_state->lpf_state.magn.x, p_data->magn.x, IMU_MAG_LPF_ALPHA);
    p_state->lpf_state.magn.y = imu_lpf(p_state->lpf_state.magn.y, p_data->magn.y, IMU_MAG_LPF_ALPHA);
    p_state->lpf_state.magn.z = imu_lpf(p_state->lpf_state.magn.z, p_data->magn.z, IMU_MAG_LPF_ALPHA);

    *p_data = p_state->lpf_state;
}

static void imu_update_zero_point_axis(imu_runtime_state_t * p_state, const IMU_DATA_t * p_point)
{
    if ((NULL == p_state) || (NULL == p_point))
    {
        return;
    }

    memcpy(&p_state->zero_point, p_point, sizeof(IMU_DATA_t));
    p_state->lpf_ready = 0U;
}

static void imu_update_zero_point_attitude(imu_runtime_state_t * p_state, const ATTITUDE_DATA_t * p_attitude)
{
    if ((NULL == p_state) || (NULL == p_attitude))
    {
        return;
    }

    memcpy(&p_state->zero_attitude, p_attitude, sizeof(ATTITUDE_DATA_t));
}

static float imu_q_rsqrt(float number)
{
    long i;
    float x2;
    float y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    y = number;
    i = *(long *) &y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *) &i;
    y = y * (threehalfs - (x2 * y * y));
    return y;
}

static void imu_update_attitude(imu_runtime_state_t * p_state,
                                 IMU_DATA_t imudata,
                                 ATTITUDE_DATA_t * p_attitude)
{
    static const float accel_kp = 0.8f;
    static const float accel_ki = 0.0001f;
    float norm;
    float vx;
    float vy;
    float vz;
    float ex;
    float ey;
    float q0;
    float q1;
    float q2;
    float q3;
    float q0_t;
    float q1_t;
    float q2_t;
    float q3_t;
    float dt_s;
    float halftime;
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
    float q0q1;
    float q0q2;
    float q1q1;
    float q1q3;
    float q2q2;
    float q2q3;

    if ((NULL == p_state) || (NULL == p_attitude))
    {
        return;
    }

    dt_s = imu_get_delta_time_s(p_state);
    if (dt_s < IMU_MIN_DT_S)
    {
        dt_s = IMU_MIN_DT_S;
    }
    else if (dt_s > IMU_MAX_DT_S)
    {
        dt_s = IMU_MAX_DT_S;
    }

    halftime = dt_s * 0.5f;

    if (0U == p_state->attitude_ready)
    {
        imu_init_attitude_from_accel(p_state, &imudata);
    }

    gx = imudata.gyro.x;
    gy = imudata.gyro.y;
    gz = imudata.gyro.z;
    ax = imudata.accel.x;
    ay = imudata.accel.y;
    az = imudata.accel.z;

    if ((0.0f == p_state->q[0]) &&
        (0.0f == p_state->q[1]) &&
        (0.0f == p_state->q[2]) &&
        (0.0f == p_state->q[3]))
    {
        p_state->q[0] = 1.0f;
    }

    q0 = p_state->q[0];
    q1 = p_state->q[1];
    q2 = p_state->q[2];
    q3 = p_state->q[3];

    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q1q1 = q1 * q1;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;

    vx = 2.0f * (q1q3 - q0q2);
    vy = 2.0f * (q0q1 + q2q3);
    vz = 1.0f - 2.0f * (q1q1 + q2q2);

    norm = imu_q_rsqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;

    p_state->e_int[0] += ex;
    p_state->e_int[1] += ey;
    p_state->e_int[2] = 0.0f;

    p_state->e_int[0] = imu_clampf(p_state->e_int[0], -IMU_EINT_LIMIT, IMU_EINT_LIMIT);
    p_state->e_int[1] = imu_clampf(p_state->e_int[1], -IMU_EINT_LIMIT, IMU_EINT_LIMIT);

    gx += accel_kp * ex + accel_ki * p_state->e_int[0];
    gy += accel_kp * ey + accel_ki * p_state->e_int[1];

    q0_t = (-q1 * gx - q2 * gy - q3 * gz) * halftime;
    q1_t = (q0 * gx + q2 * gz - q3 * gy) * halftime;
    q2_t = (q0 * gy - q1 * gz + q3 * gx) * halftime;
    q3_t = (q0 * gz + q1 * gy - q2 * gx) * halftime;

    q0 += q0_t;
    q1 += q1_t;
    q2 += q2_t;
    q3 += q3_t;

    norm = imu_q_rsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;

    p_state->q[0] = q0;
    p_state->q[1] = q1;
    p_state->q[2] = q2;
    p_state->q[3] = q3;

    p_attitude->roll = atan2f(2.0f * q2 * q3 + 2.0f * q0 * q1, -2.0f * q1 * q1 - 2.0f * q2 * q2 + 1.0f);
    p_attitude->pitch = asinf(-2.0f * q1 * q3 + 2.0f * q0 * q2);
    p_attitude->yaw = atan2f(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);

    p_attitude->roll -= p_state->zero_attitude.roll;
    p_attitude->pitch -= p_state->zero_attitude.pitch;
    p_attitude->yaw -= p_state->zero_attitude.yaw;

    p_state->latest_attitude = *p_attitude;
    p_state->latest_attitude_valid = 1U;
}

static imu_runtime_state_t const * bsp_imu_get_runtime_state(void)
{
    switch (g_selected_device)
    {
        case IMU_DEVICE_MPU6500:
            return &g_mpu9250_state;
        case IMU_DEVICE_ICM20948:
        default:
            return &g_icm20948_state;
    }
}

static uint8_t mpu9250_is_supported_id(uint8_t who_am_i)
{
    return (uint8_t) ((MPU9250_WHO_AM_I_VAL == who_am_i) ||
                      (MPU6500_WHO_AM_I_VAL == who_am_i) ||
                      (MPU9255_WHO_AM_I_VAL == who_am_i));
}

static uint16_t mpu9250_dev_addr_8bit(uint8_t addr_7bit)
{
    return (uint16_t) ((uint16_t) addr_7bit << 1U);
}

static uint8_t mpu9250_detect_device(uint8_t * p_addr_7bit, uint8_t * p_who_am_i)
{
    pIICInterface_t iicdev = &UserII2Dev;
    const uint8_t candidate_addr[] = {MPU9250_DEV_ADDR, MPU9250_DEV_ADDR_ALT};
    uint32_t index;
    uint8_t who_am_i = 0U;

    for (index = 0U; index < (sizeof(candidate_addr) / sizeof(candidate_addr[0])); index++)
    {
        who_am_i = 0U;
        if (IIC_OK != iicdev->read_reg(mpu9250_dev_addr_8bit(candidate_addr[index]),
                                       MPU9250_WHO_AM_I,
                                       &who_am_i,
                                       1,
                                       100))
        {
            continue;
        }

        if (0U != mpu9250_is_supported_id(who_am_i))
        {
            if (NULL != p_addr_7bit)
            {
                *p_addr_7bit = candidate_addr[index];
            }

            if (NULL != p_who_am_i)
            {
                *p_who_am_i = who_am_i;
            }

            return 1U;
        }
    }

    return 0U;
}

static void mpu9250_configure_mag(void)
{
    pIICInterface_t iicdev = &UserII2Dev;
    uint8_t writebuf = 0U;
    uint8_t retry;

    g_mpu9250_mag_available = 0U;
    g_mpu9250_mag_wia = 0U;

    writebuf = 0x02U;
    (void) iicdev->write_reg(mpu9250_dev_addr_8bit(g_mpu9250_addr_7bit), MPU9250_INT_PIN_CFG, &writebuf, 1, 100);
    iicdev->delay_ms(10U);

    for (retry = 0U; retry < 10U; retry++)
    {
        g_mpu9250_mag_wia = 0U;
        if ((IIC_OK == iicdev->read_reg(AK8963_DEV_ADDR << 1, AK8963_WIA, &g_mpu9250_mag_wia, 1, 100)) &&
            (AK8963_WIA_VAL == g_mpu9250_mag_wia))
        {
            g_mpu9250_mag_available = 1U;
            break;
        }

        iicdev->delay_ms(10U);
    }

    if (0U == g_mpu9250_mag_available)
    {
        return;
    }

    writebuf = 0x01U;
    (void) iicdev->write_reg(AK8963_DEV_ADDR << 1, AK8963_CNTL2, &writebuf, 1, 100);
    iicdev->delay_ms(10U);

    writebuf = 0x00U;
    (void) iicdev->write_reg(AK8963_DEV_ADDR << 1, AK8963_CNTL1, &writebuf, 1, 100);
    iicdev->delay_ms(10U);

    writebuf = 0x16U;
    (void) iicdev->write_reg(AK8963_DEV_ADDR << 1, AK8963_CNTL1, &writebuf, 1, 100);
    iicdev->delay_ms(10U);
}

static uint8_t icm20948_init(void)
{
    pIICInterface_t iicdev = &UserII2Dev;
    uint8_t writebuf = 0U;
    uint8_t who_am_i = 0U;
    uint8_t mag_wia = 0U;
    uint8_t mag_ok = 0U;
    uint8_t retry;

    g_icm20948_init_stage = BSP_IMU_INIT_STAGE_DETECT;
    g_icm20948_who_am_i = 0U;
    g_icm20948_mag_wia = 0U;
    g_icm20948_mag_available = 0U;

    writebuf = REG_VAL_SELECT_BANK_0;
    (void) iicdev->write_reg(ICM20948_DEV << 1, REG_BANK_SEL, &writebuf, 1, 500);

    (void) iicdev->read_reg(ICM20948_DEV << 1, WHO_AM_I, &who_am_i, 1, 500);
    g_icm20948_who_am_i = who_am_i;
    if (0xEAU != who_am_i)
    {
        return 1U;
    }

    g_icm20948_init_stage = BSP_IMU_INIT_STAGE_RESET;
    writebuf = (1U << 7);
    (void) iicdev->write_reg(ICM20948_DEV << 1, PWR_MGMT_1, &writebuf, 1, 500);
    iicdev->delay_ms(100U);

    writebuf = 0x00U;
    (void) iicdev->write_reg(ICM20948_DEV << 1, USER_CTRL, &writebuf, 1, 500);

    writebuf = 0x01U;
    (void) iicdev->write_reg(ICM20948_DEV << 1, PWR_MGMT_1, &writebuf, 1, 100);

    g_icm20948_init_stage = BSP_IMU_INIT_STAGE_CONFIG;
    writebuf = REG_VAL_SELECT_BANK_2;
    (void) iicdev->write_reg(ICM20948_DEV << 1, REG_BANK_SEL, &writebuf, 1, 100);

    writebuf = 0x04U;
    (void) iicdev->write_reg(ICM20948_DEV << 1, GYRO_SMPLRT_DIV, &writebuf, 1, 100);

    writebuf = (3U << 1) | (1U << 0) | (3U << 3);
    (void) iicdev->write_reg(ICM20948_DEV << 1, GYRO_CONFIG_1, &writebuf, 1, 100);

    writebuf = 0x04U;
    (void) iicdev->write_reg(ICM20948_DEV << 1, ACCEL_SMPLRT_DIV_2, &writebuf, 1, 100);

    writebuf = (0U << 1) | (1U << 0) | (5U << 3);
    (void) iicdev->write_reg(ICM20948_DEV << 1, ACCEL_CONFIG, &writebuf, 1, 100);

    writebuf = REG_VAL_SELECT_BANK_0;
    (void) iicdev->write_reg(ICM20948_DEV << 1, REG_BANK_SEL, &writebuf, 1, 100);

    writebuf = (1U << 1);
    (void) iicdev->write_reg(ICM20948_DEV << 1, INT_PIN_CFG, &writebuf, 1, 100);

    g_icm20948_init_stage = BSP_IMU_INIT_STAGE_MAG;
    for (retry = 0U; retry < 5U; retry++)
    {
        (void) iicdev->read_reg(AK09916_DEV << 1, WIA, &mag_wia, 1, 100);
        g_icm20948_mag_wia = mag_wia;
        if (0x09U == mag_wia)
        {
            mag_ok = 1U;
            break;
        }
        iicdev->delay_ms(10U);
    }

    if (0U != mag_ok)
    {
        g_icm20948_mag_available = 1U;
        writebuf = (1U << 3);
        (void) iicdev->write_reg(AK09916_DEV << 1, CNTL2, &writebuf, 1, 100);
    }

    imu_runtime_reset(&g_icm20948_state);
    g_icm20948_init_stage = BSP_IMU_INIT_STAGE_READY;
    return 0U;
}

static uint8_t icm20948_deinit(void)
{
    pIICInterface_t iicdev = &UserII2Dev;
    uint8_t writebuf = 0U;
    IIC_Status_t check_state = IIC_OK;

    writebuf = REG_VAL_SELECT_BANK_0;
    check_state += iicdev->write_reg(ICM20948_DEV << 1, REG_BANK_SEL, &writebuf, 1, 500);
    check_state += iicdev->read_reg(ICM20948_DEV << 1, WHO_AM_I, &writebuf, 1, 500);
    if (0xEAU != writebuf)
    {
        return 1U;
    }

    writebuf = (1U << 6);
    check_state += iicdev->write_reg(ICM20948_DEV << 1, PWR_MGMT_1, &writebuf, 1, 500);

    imu_runtime_reset(&g_icm20948_state);
    g_icm20948_init_stage = BSP_IMU_INIT_STAGE_IDLE;
    g_icm20948_mag_available = 0U;
    return (0 != check_state) ? 1U : 0U;
}

static void icm20948_update_9axis(IMU_DATA_t * p_data)
{
    pIICInterface_t iicdev = &UserII2Dev;
    uint8_t tmpbuf[12];
    uint8_t magnbuf[8];

    if (NULL == p_data)
    {
        return;
    }

    memset(tmpbuf, 0, sizeof(tmpbuf));
    memset(magnbuf, 0, sizeof(magnbuf));

    (void) iicdev->read_reg(ICM20948_DEV << 1, ACCEL_XOUT_H, tmpbuf, 12, 100);

    p_data->accel.x = (float) (short) (tmpbuf[0] << 8 | tmpbuf[1]) * IMU_ACCEL_SCALE_2G;
    p_data->accel.y = (float) (short) (tmpbuf[2] << 8 | tmpbuf[3]) * IMU_ACCEL_SCALE_2G;
    p_data->accel.z = (float) (short) (tmpbuf[4] << 8 | tmpbuf[5]) * IMU_ACCEL_SCALE_2G;

    p_data->gyro.x = (float) (short) (tmpbuf[6] << 8 | tmpbuf[7]) * IMU_GYRO_SCALE_2000DPS * IMU_DEG_TO_RAD;
    p_data->gyro.y = (float) (short) (tmpbuf[8] << 8 | tmpbuf[9]) * IMU_GYRO_SCALE_2000DPS * IMU_DEG_TO_RAD;
    p_data->gyro.z = (float) (short) (tmpbuf[10] << 8 | tmpbuf[11]) * IMU_GYRO_SCALE_2000DPS * IMU_DEG_TO_RAD;

    p_data->gyro.x -= g_icm20948_state.zero_point.gyro.x;
    p_data->gyro.y -= g_icm20948_state.zero_point.gyro.y;
    p_data->gyro.z -= g_icm20948_state.zero_point.gyro.z;

    if (fabsf(p_data->gyro.z) < BSP_IMU_GYRO_Z_DEADBAND_RAD_S)
    {
        p_data->gyro.z = 0.0f;
    }

    (void) iicdev->read_reg(AK09916_DEV << 1, HXL, magnbuf, 8, 100);

    if (0U == ((magnbuf[7] >> 3) & 0x01U))
    {
        p_data->magn.x = (float) (short) (magnbuf[1] << 8 | magnbuf[0]) * IMU_MAG_SCALE_UT;
        p_data->magn.y = (float) (short) (magnbuf[3] << 8 | magnbuf[2]) * IMU_MAG_SCALE_UT;
        p_data->magn.z = (float) (short) (magnbuf[5] << 8 | magnbuf[4]) * IMU_MAG_SCALE_UT;
    }

    imu_apply_lpf(p_data, &g_icm20948_state);
}

static void icm20948_update_zero_axis(const IMU_DATA_t * p_point)
{
    imu_update_zero_point_axis(&g_icm20948_state, p_point);
}

static void icm20948_update_zero_attitude(const ATTITUDE_DATA_t * p_attitude)
{
    imu_update_zero_point_attitude(&g_icm20948_state, p_attitude);
}

static void icm20948_update_attitude(IMU_DATA_t imudata, ATTITUDE_DATA_t * p_attitude)
{
    imu_update_attitude(&g_icm20948_state, imudata, p_attitude);
}

static uint8_t mpu9250_init(void)
{
    pIICInterface_t iicdev = &UserII2Dev;
    uint8_t writebuf;
    uint8_t who_am_i = 0U;
    uint8_t detected_addr = MPU9250_DEV_ADDR;

    g_mpu9250_init_stage = BSP_IMU_INIT_STAGE_DETECT;
    g_mpu9250_mag_available = 0U;
    g_mpu9250_mag_wia = 0U;
    g_mpu9250_who_am_i = 0U;

    if (0U == mpu9250_detect_device(&detected_addr, &who_am_i))
    {
        return 1U;
    }

    g_mpu9250_addr_7bit = detected_addr;
    g_mpu9250_who_am_i = who_am_i;

    g_mpu9250_init_stage = BSP_IMU_INIT_STAGE_RESET;
    writebuf = 0x80U;
    if (IIC_OK != iicdev->write_reg(mpu9250_dev_addr_8bit(g_mpu9250_addr_7bit), MPU9250_PWR_MGMT_1, &writebuf, 1, 100))
    {
        return 1U;
    }
    iicdev->delay_ms(100U);

    writebuf = 0x00U;
    if (IIC_OK != iicdev->write_reg(mpu9250_dev_addr_8bit(g_mpu9250_addr_7bit), MPU9250_PWR_MGMT_2, &writebuf, 1, 100))
    {
        return 1U;
    }

    writebuf = 0x01U;
    if (IIC_OK != iicdev->write_reg(mpu9250_dev_addr_8bit(g_mpu9250_addr_7bit), MPU9250_PWR_MGMT_1, &writebuf, 1, 100))
    {
        return 1U;
    }
    iicdev->delay_ms(10U);

    writebuf = 0x00U;
    if (IIC_OK != iicdev->write_reg(mpu9250_dev_addr_8bit(g_mpu9250_addr_7bit), MPU9250_USER_CTRL, &writebuf, 1, 100))
    {
        return 1U;
    }

    g_mpu9250_init_stage = BSP_IMU_INIT_STAGE_CONFIG;
    writebuf = 0x03U;
    if (IIC_OK != iicdev->write_reg(mpu9250_dev_addr_8bit(g_mpu9250_addr_7bit), MPU9250_CONFIG, &writebuf, 1, 100))
    {
        return 1U;
    }

    writebuf = 0x04U;
    if (IIC_OK != iicdev->write_reg(mpu9250_dev_addr_8bit(g_mpu9250_addr_7bit), MPU9250_SMPLRT_DIV, &writebuf, 1, 100))
    {
        return 1U;
    }

    writebuf = 0x18U;
    if (IIC_OK != iicdev->write_reg(mpu9250_dev_addr_8bit(g_mpu9250_addr_7bit), MPU9250_GYRO_CONFIG, &writebuf, 1, 100))
    {
        return 1U;
    }

    writebuf = 0x00U;
    if (IIC_OK != iicdev->write_reg(mpu9250_dev_addr_8bit(g_mpu9250_addr_7bit), MPU9250_ACCEL_CONFIG, &writebuf, 1, 100))
    {
        return 1U;
    }

    writebuf = 0x03U;
    if (IIC_OK != iicdev->write_reg(mpu9250_dev_addr_8bit(g_mpu9250_addr_7bit), MPU9250_ACCEL_CONFIG_2, &writebuf, 1, 100))
    {
        return 1U;
    }

    g_mpu9250_init_stage = BSP_IMU_INIT_STAGE_MAG;
    mpu9250_configure_mag();

    imu_runtime_reset(&g_mpu9250_state);
    g_mpu9250_init_stage = BSP_IMU_INIT_STAGE_READY;
    return 0U;
}

static uint8_t mpu9250_deinit(void)
{
    pIICInterface_t iicdev = &UserII2Dev;
    uint8_t writebuf = 0U;
    uint8_t who_am_i = 0U;

    if (IIC_OK != iicdev->read_reg(mpu9250_dev_addr_8bit(g_mpu9250_addr_7bit), MPU9250_WHO_AM_I, &who_am_i, 1, 100))
    {
        return 1U;
    }

    if (0U == mpu9250_is_supported_id(who_am_i))
    {
        return 1U;
    }

    if (0U != g_mpu9250_mag_available)
    {
        writebuf = 0x00U;
        (void) iicdev->write_reg(AK8963_DEV_ADDR << 1, AK8963_CNTL1, &writebuf, 1, 100);
    }

    writebuf = 0x40U;
    (void) iicdev->write_reg(mpu9250_dev_addr_8bit(g_mpu9250_addr_7bit), MPU9250_PWR_MGMT_1, &writebuf, 1, 100);

    imu_runtime_reset(&g_mpu9250_state);
    g_mpu9250_init_stage = BSP_IMU_INIT_STAGE_IDLE;
    g_mpu9250_mag_available = 0U;
    return 0U;
}

static void mpu9250_update_9axis(IMU_DATA_t * p_data)
{
    pIICInterface_t iicdev = &UserII2Dev;
    uint8_t tmpbuf[14];
    uint8_t st1 = 0U;
    uint8_t magbuf[7];

    if (NULL == p_data)
    {
        return;
    }

    memset(tmpbuf, 0, sizeof(tmpbuf));
    memset(magbuf, 0, sizeof(magbuf));

    (void) iicdev->read_reg(mpu9250_dev_addr_8bit(g_mpu9250_addr_7bit), MPU9250_ACCEL_XOUT_H, tmpbuf, 14, 100);

    p_data->accel.x = (float) (short) (tmpbuf[0] << 8 | tmpbuf[1]) * IMU_ACCEL_SCALE_2G;
    p_data->accel.y = (float) (short) (tmpbuf[2] << 8 | tmpbuf[3]) * IMU_ACCEL_SCALE_2G;
    p_data->accel.z = (float) (short) (tmpbuf[4] << 8 | tmpbuf[5]) * IMU_ACCEL_SCALE_2G;

    p_data->gyro.x = (float) (short) (tmpbuf[8] << 8 | tmpbuf[9]) * IMU_GYRO_SCALE_2000DPS * IMU_DEG_TO_RAD;
    p_data->gyro.y = (float) (short) (tmpbuf[10] << 8 | tmpbuf[11]) * IMU_GYRO_SCALE_2000DPS * IMU_DEG_TO_RAD;
    p_data->gyro.z = (float) (short) (tmpbuf[12] << 8 | tmpbuf[13]) * IMU_GYRO_SCALE_2000DPS * IMU_DEG_TO_RAD;

    p_data->gyro.x -= g_mpu9250_state.zero_point.gyro.x;
    p_data->gyro.y -= g_mpu9250_state.zero_point.gyro.y;
    p_data->gyro.z -= g_mpu9250_state.zero_point.gyro.z;

    if (fabsf(p_data->gyro.z) < BSP_IMU_GYRO_Z_DEADBAND_RAD_S)
    {
        p_data->gyro.z = 0.0f;
    }

    if (0U != g_mpu9250_mag_available)
    {
        (void) iicdev->read_reg(AK8963_DEV_ADDR << 1, AK8963_ST1, &st1, 1, 100);
        if ((st1 & 0x01U) != 0U)
        {
            (void) iicdev->read_reg(AK8963_DEV_ADDR << 1, AK8963_HXL, magbuf, 7, 100);
            if (0U == ((magbuf[6] >> 3) & 0x01U))
            {
                p_data->magn.x = (float) (short) (magbuf[1] << 8 | magbuf[0]) * IMU_MAG_SCALE_UT;
                p_data->magn.y = (float) (short) (magbuf[3] << 8 | magbuf[2]) * IMU_MAG_SCALE_UT;
                p_data->magn.z = (float) (short) (magbuf[5] << 8 | magbuf[4]) * IMU_MAG_SCALE_UT;
            }
        }
    }

    imu_apply_lpf(p_data, &g_mpu9250_state);
}

static void mpu9250_update_zero_axis(const IMU_DATA_t * p_point)
{
    imu_update_zero_point_axis(&g_mpu9250_state, p_point);
}

static void mpu9250_update_zero_attitude(const ATTITUDE_DATA_t * p_attitude)
{
    imu_update_zero_point_attitude(&g_mpu9250_state, p_attitude);
}

static void mpu9250_update_attitude(IMU_DATA_t imudata, ATTITUDE_DATA_t * p_attitude)
{
    imu_update_attitude(&g_mpu9250_state, imudata, p_attitude);
}

void bsp_imu_select_device(IMUDevice_t device)
{
    if ((IMU_DEVICE_ICM20948 == device) || (IMU_DEVICE_MPU6500 == device))
    {
        g_selected_device = device;
    }
}

IMUDevice_t bsp_imu_get_selected_device(void)
{
    return g_selected_device;
}

const char * bsp_imu_get_device_name(IMUDevice_t device)
{
    switch (device)
    {
        case IMU_DEVICE_ICM20948:
            return "ICM20948";
        case IMU_DEVICE_MPU6500:
            return "MPU6500";
        default:
            return "UNKNOWN";
    }
}

pIMUInterface_t bsp_imu_get_interface(void)
{
    switch (g_selected_device)
    {
        case IMU_DEVICE_MPU6500:
            return &UserMPU9250;
        case IMU_DEVICE_ICM20948:
        default:
            return &UserICM20948;
    }
}

void bsp_imu_get_diag(bsp_imu_diag_t * p_diag)
{
    if (NULL == p_diag)
    {
        return;
    }

    p_diag->selected_device = g_selected_device;
    p_diag->init_ok = 0U;
    p_diag->init_stage = BSP_IMU_INIT_STAGE_IDLE;
    p_diag->last_addr_7bit = 0U;
    p_diag->last_who_am_i = 0U;
    p_diag->last_mag_wia = 0U;
    p_diag->mag_available = 0U;

    switch (g_selected_device)
    {
        case IMU_DEVICE_MPU6500:
            p_diag->init_ok = (uint8_t) (BSP_IMU_INIT_STAGE_READY == g_mpu9250_init_stage);
            p_diag->init_stage = g_mpu9250_init_stage;
            p_diag->last_addr_7bit = g_mpu9250_addr_7bit;
            p_diag->last_who_am_i = g_mpu9250_who_am_i;
            p_diag->last_mag_wia = g_mpu9250_mag_wia;
            p_diag->mag_available = g_mpu9250_mag_available;
            break;
        case IMU_DEVICE_ICM20948:
            p_diag->init_ok = (uint8_t) (BSP_IMU_INIT_STAGE_READY == g_icm20948_init_stage);
            p_diag->init_stage = g_icm20948_init_stage;
            p_diag->last_addr_7bit = ICM20948_DEV;
            p_diag->last_who_am_i = g_icm20948_who_am_i;
            p_diag->last_mag_wia = g_icm20948_mag_wia;
            p_diag->mag_available = g_icm20948_mag_available;
            break;
        default:
            break;
    }
}

bool bsp_imu_get_attitude(ATTITUDE_DATA_t * p_attitude)
{
    imu_runtime_state_t const * p_state = bsp_imu_get_runtime_state();

    if ((NULL == p_attitude) || (NULL == p_state) || (0U == p_state->latest_attitude_valid))
    {
        return false;
    }

    *p_attitude = p_state->latest_attitude;
    return true;
}

bool bsp_imu_get_yaw(float * p_yaw_rad)
{
    ATTITUDE_DATA_t attitude;

    if ((NULL == p_yaw_rad) || !bsp_imu_get_attitude(&attitude))
    {
        return false;
    }

    *p_yaw_rad = attitude.yaw;
    return true;
}

bool bsp_imu_get_magnetometer(PrivateBuf_t * p_magnetometer)
{
    imu_runtime_state_t const * p_state = bsp_imu_get_runtime_state();

    if ((NULL == p_magnetometer) || (NULL == p_state) || (0U == p_state->lpf_ready))
    {
        return false;
    }

    *p_magnetometer = p_state->lpf_state.magn;
    return true;
}

IMUInterface_t UserICM20948 = {
    .Init = icm20948_init,
    .DeInit = icm20948_deinit,
    .UpdateZeroPoint_axis = icm20948_update_zero_axis,
    .UpdateZeroPoint_attitude = icm20948_update_zero_attitude,
    .Update_9axisVal = icm20948_update_9axis,
    .UpdateAttitude = icm20948_update_attitude,
};

IMUInterface_t UserMPU9250 = {
    .Init = mpu9250_init,
    .DeInit = mpu9250_deinit,
    .UpdateZeroPoint_axis = mpu9250_update_zero_axis,
    .UpdateZeroPoint_attitude = mpu9250_update_zero_attitude,
    .Update_9axisVal = mpu9250_update_9axis,
    .UpdateAttitude = mpu9250_update_attitude,
};
