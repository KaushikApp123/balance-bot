#include "imu_mpu6050.h"

#include <math.h>
#include <string.h>

#include "board.h"
#include "kalman.h"
#include "shared.h"

/* Registers */
#define REG_SMPLRT_DIV    0x19u
#define REG_CONFIG        0x1Au
#define REG_GYRO_CONFIG   0x1Bu
#define REG_ACCEL_CONFIG  0x1Cu
#define REG_INT_PIN_CFG   0x37u
#define REG_INT_ENABLE    0x38u
#define REG_ACCEL_XOUT_H  0x3Bu
#define REG_PWR_MGMT_1    0x6Bu
#define REG_WHO_AM_I      0x75u

#define I2C_TIMEOUT_MS    10u

/* Mounting: chip X forward, Y left, Z up. If the IMU sits rotated on the PCB,
 * change only these four lines. Index into r[] = ax ay az temp gx gy gz. */
#define AX_FWD(r)     ((float)(r)[0])
#define AX_SIDE(r)    ((float)(r)[1])
#define AX_UP(r)      ((float)(r)[2])
#define GYRO_PITCH(r) ((float)(r)[5])    /* rotation about +Y tips Z toward +X */

static uint8_t s_dma_buf[14];
static uint8_t s_latest[14];
static volatile bool s_have_sample;

static bool wr(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT,
                             &val, 1, I2C_TIMEOUT_MS) == HAL_OK;
}

static bool rd(uint8_t reg, uint8_t *buf, uint16_t n)
{
    return HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT,
                            buf, n, I2C_TIMEOUT_MS) == HAL_OK;
}

static void delay_ms(uint32_t ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) vTaskDelay(pdMS_TO_TICKS(ms));
    else HAL_Delay(ms);
}

bool imu_init(void)
{
    uint8_t who = 0;
    if (!rd(REG_WHO_AM_I, &who, 1) || who != 0x68u) return false;

    if (!wr(REG_PWR_MGMT_1, 0x80u)) return false;       /* device reset */
    delay_ms(100);
    bool ok = true;
    ok &= wr(REG_PWR_MGMT_1, 0x01u);      /* wake, clock = PLL on gyro X (more stable than the 8 MHz RC) */
    ok &= wr(REG_CONFIG, 0x03u);          /* DLPF 44 Hz accel / 42 Hz gyro, 1 kHz internal rate */
    ok &= wr(REG_SMPLRT_DIV, 0x01u);      /* 1 kHz / (1 + 1) = 500 Hz */
    ok &= wr(REG_GYRO_CONFIG, 0x08u);     /* +-500 dps */
    ok &= wr(REG_ACCEL_CONFIG, 0x08u);    /* +-4 g */
    ok &= wr(REG_INT_PIN_CFG, 0x10u);     /* active high, push-pull, 50 us pulse, any read clears */
    ok &= wr(REG_INT_ENABLE, 0x01u);      /* data ready */
    return ok;
}

static void parse(const uint8_t *b, int16_t r[7])
{
    for (int i = 0; i < 7; i++) r[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);
}

/* r[] = ax ay az temp gx gy gz, as read from the chip */
static void to_sample(const int16_t r[7], imu_sample_t *s)
{
    s->a_fwd  = AX_FWD(r)  / MPU_ACCEL_LSB_PER_G;
    s->a_side = AX_SIDE(r) / MPU_ACCEL_LSB_PER_G;
    s->a_up   = AX_UP(r)   / MPU_ACCEL_LSB_PER_G;
    s->gyro_pitch = DEG2RAD(GYRO_PITCH(r) / MPU_GYRO_LSB_PER_DPS);
    s->temp_c = (float)r[3] / 340.0f + 36.53f;
}

bool imu_calibrate(float *gyro_bias, float *angle0, uint32_t n)
{
    /* Average the gyro (bias) and the accel (start angle) with the robot
     * held still. Polled reads, run once before the control tasks start. */
    float g_sum = 0.0f, g_sq = 0.0f, f_sum = 0.0f, u_sum = 0.0f;
    uint8_t b[14];
    int16_t r[7];
    imu_sample_t s;
    for (uint32_t i = 0; i < n; i++) {
        if (!rd(REG_ACCEL_XOUT_H, b, sizeof b)) return false;
        parse(b, r);
        to_sample(r, &s);
        g_sum += s.gyro_pitch;
        g_sq  += s.gyro_pitch * s.gyro_pitch;
        f_sum += s.a_fwd;
        u_sum += s.a_up;
        delay_ms(2);
    }
    const float mean = g_sum / (float)n;
    const float var = g_sq / (float)n - mean * mean;
    *gyro_bias = mean;
    *angle0 = accel_pitch(f_sum / (float)n, u_sum / (float)n);
    /* If it was moving, the bias is junk; let the Kalman filter find it. */
    if (var > DEG2RAD(1.0f) * DEG2RAD(1.0f)) {
        *gyro_bias = 0.0f;
        return false;
    }
    return true;
}

void imu_start(void)
{
    __HAL_GPIO_EXTI_CLEAR_IT(MPU_INT_PIN);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void imu_stop(void)
{
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
}

bool imu_read_latest(imu_sample_t *out)
{
    if (!s_have_sample) return false;
    uint8_t b[14];
    taskENTER_CRITICAL();
    memcpy(b, s_latest, sizeof b);
    s_have_sample = false;
    taskEXIT_CRITICAL();
    int16_t r[7];
    parse(b, r);
    to_sample(r, out);
    return true;
}

bool imu_bus_recover(void)
{
    /* A slave reset mid-transfer can hold SDA low forever. Free it by clocking
     * SCL up to 9 times, then send a STOP and bring I2C1 back up. */
    imu_stop();
    HAL_I2C_DeInit(&hi2c1);

    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    g.Mode = GPIO_MODE_OUTPUT_OD;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);

    for (int i = 0; i < 9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        for (volatile int d = 0; d < 300; d++) { }
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        for (volatile int d = 0; d < 300; d++) { }
    }
    /* STOP: SDA low -> high while SCL high */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    for (volatile int d = 0; d < 300; d++) { }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

    /* Pulse the peripheral reset: the F4 I2C can latch BUSY after a glitch. */
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) return false;   /* MspInit restores AF pins + DMA */

    g_stats.i2c_recoveries++;
    bool ok = imu_init();
    if (ok) imu_start();
    return ok;
}

/* ---------------- Interrupt side ---------------- */

void imu_on_data_ready_isr(void)
{
    if (hi2c1.State != HAL_I2C_STATE_READY) {
        g_stats.imu_overruns++;          /* previous read still running */
        return;
    }
    if (HAL_I2C_Mem_Read_DMA(&hi2c1, MPU_ADDR << 1, REG_ACCEL_XOUT_H,
                             I2C_MEMADD_SIZE_8BIT, s_dma_buf, sizeof s_dma_buf) != HAL_OK) {
        g_stats.i2c_errors++;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (pin == MPU_INT_PIN) imu_on_data_ready_isr();
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *h)
{
    if (h->Instance != I2C1) return;
    memcpy(s_latest, s_dma_buf, sizeof s_latest);
    s_have_sample = true;
    BaseType_t woken = pdFALSE;
    if (h_sensor) vTaskNotifyGiveFromISR(h_sensor, &woken);
    portYIELD_FROM_ISR(woken);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *h)
{
    if (h->Instance == I2C1) g_stats.i2c_errors++;
}
