/*
 * stm32f4xx_it.c - Interrupt vectors. Each one hands off to the HAL, which
 * calls back into imu_mpu6050.c and task_telemetry.c.
 */
#include "board.h"
#include "FreeRTOS.h"
#include "task.h"

extern void xPortSysTickHandler(void);

void NMI_Handler(void)        { board_fatal(); }
void HardFault_Handler(void)  { board_fatal(); }
void MemManage_Handler(void)  { board_fatal(); }
void BusFault_Handler(void)   { board_fatal(); }
void UsageFault_Handler(void) { board_fatal(); }
void DebugMon_Handler(void)   { }

/* One 1 kHz SysTick drives both the HAL timebase and the FreeRTOS tick. */
void SysTick_Handler(void)
{
    HAL_IncTick();
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}

void EXTI9_5_IRQHandler(void)    { HAL_GPIO_EXTI_IRQHandler(MPU_INT_PIN); }
void DMA1_Stream0_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_i2c1_rx); }
void I2C1_EV_IRQHandler(void)    { HAL_I2C_EV_IRQHandler(&hi2c1); }
void I2C1_ER_IRQHandler(void)    { HAL_I2C_ER_IRQHandler(&hi2c1); }
void DMA2_Stream7_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart1_tx); }
void DMA2_Stream2_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart1_rx); }
void USART1_IRQHandler(void)     { HAL_UART_IRQHandler(&huart1); }
