#include "motor.h"
#include "board.h"

void motor_enable(bool on)
{
    HAL_GPIO_WritePin(MOT_CTRL_PORT, MOT_SLEEP_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint32_t duty_to_ccr(float d)
{
    if (d < 0.0f) d = -d;
    if (d > 1.0f) d = 1.0f;
    return (uint32_t)(d * (float)PWM_PERIOD + 0.5f);
}

static void set_one(float duty, uint16_t ph_pin, volatile uint32_t *ccr, int invert)
{
    if (invert) duty = -duty;
    /* PH/EN mode: PH picks the direction, EN carries the PWM. */
    HAL_GPIO_WritePin(MOT_PH_PORT, ph_pin, duty >= 0.0f ? GPIO_PIN_SET : GPIO_PIN_RESET);
    *ccr = duty_to_ccr(duty);
}

void motor_set(float left, float right)
{
    set_one(left,  MOT_PH_L_PIN, &TIM3->CCR1, MOTOR_LEFT_INVERT);
    set_one(right, MOT_PH_R_PIN, &TIM3->CCR2, MOTOR_RIGHT_INVERT);
}

void motor_stop(void)
{
    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;
}

bool motor_driver_fault(void)
{
    return HAL_GPIO_ReadPin(MOT_CTRL_PORT, MOT_FAULT_PIN) == GPIO_PIN_RESET;
}

uint16_t encoder_left_raw(void)
{
    uint16_t c = (uint16_t)TIM2->CNT;
    return ENC_LEFT_INVERT ? (uint16_t)(0u - c) : c;
}

uint16_t encoder_right_raw(void)
{
    uint16_t c = (uint16_t)TIM4->CNT;
    return ENC_RIGHT_INVERT ? (uint16_t)(0u - c) : c;
}
