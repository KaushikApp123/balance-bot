/*
 * board.c - Clock tree, GPIO and peripheral setup, plus the HAL MSP hooks.
 * Written by hand instead of CubeMX output so every setting is visible.
 */
#include "board.h"

I2C_HandleTypeDef  hi2c1;
DMA_HandleTypeDef  hdma_i2c1_rx;
TIM_HandleTypeDef  htim2, htim3, htim4;
UART_HandleTypeDef huart1, huart2;
DMA_HandleTypeDef  hdma_usart1_tx, hdma_usart1_rx;
ADC_HandleTypeDef  hadc1;
IWDG_HandleTypeDef hiwdg;

static void check(HAL_StatusTypeDef st)
{
    if (st != HAL_OK) board_fatal();
}

/* 25 MHz HSE -> PLL -> 100 MHz SYSCLK. APB1 = 50 MHz (timers 100 MHz), APB2 = 100 MHz. */
void board_clock_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSI;
    osc.HSEState = RCC_HSE_ON;
    osc.LSIState = RCC_LSI_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 25;
    osc.PLL.PLLN = 200;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 4;
    check(HAL_RCC_OscConfig(&osc));

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    check(HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3));
}

void board_cycle_counter_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void gpio_init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Motor drivers start asleep with direction low. */
    HAL_GPIO_WritePin(MOT_CTRL_PORT, MOT_SLEEP_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOT_PH_PORT, MOT_PH_L_PIN | MOT_PH_R_PIN, GPIO_PIN_RESET);
    g.Pin = MOT_PH_L_PIN | MOT_PH_R_PIN | MOT_SLEEP_PIN;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);

    g.Pin = MOT_FAULT_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(MOT_CTRL_PORT, &g);

    /* IMU data-ready. The EXTI line is unmasked later by imu_start(). */
    g.Pin = MPU_INT_PIN;
    g.Mode = GPIO_MODE_IT_RISING;
    g.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(MPU_INT_PORT, &g);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, IRQ_PRIO_IMU_EXTI, 0);

    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);   /* off (active low) */
    g.Pin = LED_PIN;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(LED_PORT, &g);
}

static void i2c_init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    check(HAL_I2C_Init(&hi2c1));
}

static void pwm_init(void)
{
    TIM_OC_InitTypeDef oc = {0};
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = PWM_PERIOD - 1u;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    check(HAL_TIM_PWM_Init(&htim3));

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    check(HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_1));
    check(HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_2));
    check(HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1));
    check(HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2));
}

static void encoder_init_one(TIM_HandleTypeDef *h, TIM_TypeDef *inst)
{
    TIM_Encoder_InitTypeDef e = {0};
    h->Instance = inst;
    h->Init.Prescaler = 0;
    h->Init.CounterMode = TIM_COUNTERMODE_UP;
    h->Init.Period = 0xFFFF;          /* 16-bit wrap on both TIM2 and TIM4 */
    h->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    h->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    e.EncoderMode = TIM_ENCODERMODE_TI12;  /* count both edges of A and B: 4x */
    e.IC1Polarity = TIM_ICPOLARITY_RISING;
    e.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    e.IC1Prescaler = TIM_ICPSC_DIV1;
    e.IC1Filter = 6;                  /* reject glitches shorter than ~0.4 us */
    e.IC2Polarity = TIM_ICPOLARITY_RISING;
    e.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    e.IC2Prescaler = TIM_ICPSC_DIV1;
    e.IC2Filter = 6;
    check(HAL_TIM_Encoder_Init(h, &e));
    check(HAL_TIM_Encoder_Start(h, TIM_CHANNEL_ALL));
}

static void uart_init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = BLE_BAUD;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    check(HAL_UART_Init(&huart1));

    huart2.Instance = USART2;
    huart2.Init = huart1.Init;
    huart2.Init.BaudRate = DEBUG_BAUD;
    check(HAL_UART_Init(&huart2));
}

static void adc_init(void)
{
    ADC_ChannelConfTypeDef ch = {0};
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;    /* 25 MHz */
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    check(HAL_ADC_Init(&hadc1));

    ch.Channel = ADC_CHANNEL_4;
    ch.Rank = 1;
    ch.SamplingTime = ADC_SAMPLETIME_480CYCLES;   /* 18k source impedance */
    check(HAL_ADC_ConfigChannel(&hadc1, &ch));
}

void board_init(void)
{
    board_cycle_counter_init();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    gpio_init();
    i2c_init();
    pwm_init();
    encoder_init_one(&htim2, TIM2);
    encoder_init_one(&htim4, TIM4);
    uart_init();
    adc_init();
}

void board_watchdog_start(void)
{
    /* LSI ~32 kHz / 32 = 1 kHz, reload 1000 -> ~1 s. Safety task kicks it at
     * 100 Hz, and only while every control task is alive. Frozen under a
     * debugger so breakpoints do not reset the board. */
    __HAL_DBGMCU_FREEZE_IWDG();
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
    hiwdg.Init.Reload = 1000;
    check(HAL_IWDG_Init(&hiwdg));
}

void board_led(int on)
{
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void board_fatal(void)
{
    __disable_irq();
    MOT_CTRL_PORT->BSRR = (uint32_t)MOT_SLEEP_PIN << 16;   /* drivers off */
    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;
    for (;;) {
        LED_PORT->ODR ^= LED_PIN;
        for (volatile uint32_t i = 0; i < 2000000u; i++) { }
    }
}

/* ------------------------------------------------------------------------- */
/* HAL MSP callbacks: clocks, pins, DMA and NVIC for each peripheral.         */
/* ------------------------------------------------------------------------- */

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *h)
{
    if (h->Instance != I2C1) return;
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    g.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    g.Mode = GPIO_MODE_AF_OD;
    g.Pull = GPIO_PULLUP;              /* 2.2k externals on the PCB do the real work */
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &g);
    __HAL_RCC_I2C1_CLK_ENABLE();

    hdma_i2c1_rx.Instance = DMA1_Stream0;
    hdma_i2c1_rx.Init.Channel = DMA_CHANNEL_1;
    hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_i2c1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    check(HAL_DMA_Init(&hdma_i2c1_rx));
    __HAL_LINKDMA(h, hdmarx, hdma_i2c1_rx);

    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, IRQ_PRIO_I2C_DMA, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, IRQ_PRIO_I2C, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, IRQ_PRIO_I2C, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *h)
{
    if (h->Instance != I2C1) return;
    __HAL_RCC_I2C1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);
    HAL_DMA_DeInit(h->hdmarx);
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *h)
{
    if (h->Instance != TIM3) return;
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_TIM3_CLK_ENABLE();
    g.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &g);
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *h)
{
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    if (h->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();
        g.Alternate = GPIO_AF1_TIM2;
        g.Pin = GPIO_PIN_15;          /* JTDI, free because we only use SWD */
        HAL_GPIO_Init(GPIOA, &g);
        g.Pin = GPIO_PIN_3;           /* JTDO/SWO */
        HAL_GPIO_Init(GPIOB, &g);
    } else if (h->Instance == TIM4) {
        __HAL_RCC_TIM4_CLK_ENABLE();
        g.Alternate = GPIO_AF2_TIM4;
        g.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        HAL_GPIO_Init(GPIOB, &g);
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef *h)
{
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    if (h->Instance == USART1) {
        __HAL_RCC_USART1_CLK_ENABLE();
        g.Pin = GPIO_PIN_9 | GPIO_PIN_10;
        g.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &g);

        hdma_usart1_tx.Instance = DMA2_Stream7;
        hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
        hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart1_tx.Init.Mode = DMA_NORMAL;
        hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        check(HAL_DMA_Init(&hdma_usart1_tx));
        __HAL_LINKDMA(h, hdmatx, hdma_usart1_tx);

        hdma_usart1_rx.Instance = DMA2_Stream2;
        hdma_usart1_rx.Init = hdma_usart1_tx.Init;
        hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
        check(HAL_DMA_Init(&hdma_usart1_rx));
        __HAL_LINKDMA(h, hdmarx, hdma_usart1_rx);

        HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, IRQ_PRIO_UART_DMA, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
        HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, IRQ_PRIO_UART_DMA, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
        HAL_NVIC_SetPriority(USART1_IRQn, IRQ_PRIO_UART_BLE, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    } else if (h->Instance == USART2) {
        __HAL_RCC_USART2_CLK_ENABLE();
        g.Pin = GPIO_PIN_2 | GPIO_PIN_3;
        g.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &g);
    }
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *h)
{
    if (h->Instance != ADC1) return;
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_ADC1_CLK_ENABLE();
    g.Pin = GPIO_PIN_4;
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &g);
}
