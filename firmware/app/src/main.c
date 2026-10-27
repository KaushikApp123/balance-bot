/*
 * main.c - Bring-up order:
 *   1. Clocks and peripherals
 *   2. Configure the IMU and calibrate the gyro while the robot is held still
 *   3. Shared RTOS objects, then saved gains from flash (defaults if none)
 *   4. Create the six tasks, start the watchdog, start the scheduler
 */
#include "board.h"
#include "gains_store.h"
#include "imu_mpu6050.h"
#include "shared.h"
#include "tasks.h"

typedef struct {
    TaskFunction_t fn;
    const char *name;
    uint16_t stack_words;
    UBaseType_t prio;
    TaskHandle_t *handle;
} task_def_t;

static const task_def_t TASKS[] = {
    { task_safety,    "safety",    256, PRIO_SAFETY,    &h_safety    },
    { task_motor,     "motor",     256, PRIO_MOTOR,     &h_motor     },
    { task_sensor,    "sensor",    384, PRIO_SENSOR,    &h_sensor    },
    { task_balance,   "balance",   384, PRIO_BALANCE,   &h_balance   },
    { task_telemetry, "telemetry", 512, PRIO_TELEMETRY, &h_telemetry },
    { task_logger,    "logger",    768, PRIO_LOGGER,    &h_logger    },
};

int main(void)
{
    HAL_Init();
    board_clock_init();
    SystemCoreClockUpdate();
    board_init();

    /* IMU setup uses HAL_Delay, so it runs before the first FreeRTOS call.
     * FreeRTOS deliberately masks interrupts (including SysTick) from the first
     * API call until the scheduler starts, which would freeze HAL_Delay. */
    bool imu_ok = false;
    for (int attempt = 0; attempt < 3 && !imu_ok; attempt++) {
        imu_ok = imu_init();
        if (!imu_ok) HAL_Delay(50);
    }
    if (imu_ok) {
        HAL_Delay(100);   /* let the DLPF settle */
        (void)imu_calibrate(&g_boot_gyro_bias, &g_boot_angle, 500);   /* 1 s, robot still */
    }
    /* If the IMU is missing the sensor task keeps retrying and the safety
     * task holds FAULT_IMU, so the robot can never arm without it. */

    ctrl_gains_t g;
    const bool have_saved = gains_store_load(&g);
    board_watchdog_start();
    shared_init(have_saved ? &g : NULL);   /* first FreeRTOS call */

    for (size_t i = 0; i < sizeof TASKS / sizeof TASKS[0]; i++) {
        const task_def_t *t = &TASKS[i];
        BaseType_t ok = xTaskCreate(t->fn, t->name, t->stack_words, NULL, t->prio, t->handle);
        configASSERT(ok == pdPASS);
    }

    vTaskStartScheduler();
    board_fatal();   /* only reached if the idle task could not be created */
}

/* ---------------- FreeRTOS hooks ---------------- */

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    (void)name;
    board_fatal();
}

void vApplicationMallocFailedHook(void)
{
    board_fatal();
}

void vAssertCalled(const char *file, int line)
{
    (void)file;
    (void)line;
    board_fatal();
}
