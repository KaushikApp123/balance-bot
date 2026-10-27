/*
 * board.h - Pin map and peripheral handles for the carrier PCB.
 *
 *  Function            Pin     Peripheral
 *  ------------------  ------  -------------------------------------------
 *  IMU SCL / SDA       PB8/PB9 I2C1 @ 400 kHz, RX on DMA1 Stream0 Ch1
 *  IMU INT (data rdy)  PB5     EXTI5, rising edge
 *  Left  motor EN/PWM  PA6     TIM3_CH1 @ 20 kHz  -> DRV8874 #1 EN
 *  Right motor EN/PWM  PA7     TIM3_CH2 @ 20 kHz  -> DRV8874 #2 EN
 *  Left  motor PH      PB0     GPIO (direction)
 *  Right motor PH      PB1     GPIO (direction)
 *  Drivers nSLEEP      PB12    GPIO, low = outputs off (shared)
 *  Drivers nFAULT      PB13    GPIO in, open-drain wired-OR, pull-up
 *  Left  encoder A/B   PA15/PB3 TIM2 encoder mode
 *  Right encoder A/B   PB6/PB7 TIM4 encoder mode
 *  BLE bridge TX/RX    PA9/PA10 USART1 @ 460800, TX DMA2 S7, RX DMA2 S2 (circular)
 *  Debug console TX/RX PA2/PA3 USART2 @ 115200
 *  Battery sense       PA4     ADC1_IN4, 100k/22k divider
 *  Motor current       PA0/PA1 ADC1_IN0/IN1 from DRV8874 IPROPI (routed, not used yet)
 *  Status LED          PC13    on the BlackPill, active low
 */
#ifndef BOARD_H
#define BOARD_H

#include "stm32f4xx_hal.h"

#define MPU_INT_PIN        GPIO_PIN_5
#define MPU_INT_PORT       GPIOB

#define MOT_PH_L_PIN       GPIO_PIN_0
#define MOT_PH_R_PIN       GPIO_PIN_1
#define MOT_PH_PORT        GPIOB
#define MOT_SLEEP_PIN      GPIO_PIN_12
#define MOT_FAULT_PIN      GPIO_PIN_13
#define MOT_CTRL_PORT      GPIOB

#define LED_PIN            GPIO_PIN_13
#define LED_PORT           GPIOC

#define PWM_FREQ_HZ        20000u
#define PWM_PERIOD         (100000000u / PWM_FREQ_HZ)   /* TIM3 clock = 100 MHz -> 5000 */

#define BLE_BAUD           460800u
#define DEBUG_BAUD         115200u

/* Battery divider: Vbatt * 22k / 122k on a 3.3 V, 12-bit ADC */
#define VBATT_SCALE        ((3.3f / 4095.0f) * (122.0f / 22.0f))

/* NVIC priorities (lower number = more urgent). All are >= 5 so they may use
 * FreeRTOS FromISR calls. The IMU path is the most time critical. */
#define IRQ_PRIO_IMU_EXTI  5
#define IRQ_PRIO_I2C_DMA   5
#define IRQ_PRIO_I2C       5
#define IRQ_PRIO_UART_BLE  6
#define IRQ_PRIO_UART_DMA  6

extern I2C_HandleTypeDef  hi2c1;
extern DMA_HandleTypeDef  hdma_i2c1_rx;
extern TIM_HandleTypeDef  htim2, htim3, htim4;
extern UART_HandleTypeDef huart1, huart2;
extern DMA_HandleTypeDef  hdma_usart1_tx, hdma_usart1_rx;
extern ADC_HandleTypeDef  hadc1;
extern IWDG_HandleTypeDef hiwdg;

void board_clock_init(void);
void board_init(void);
void board_watchdog_start(void);
void board_led(int on);
void board_cycle_counter_init(void);
static inline uint32_t board_cycles(void) { return DWT->CYCCNT; }
void board_fatal(void);     /* motors off, blink fast, wait for watchdog */

#endif /* BOARD_H */
