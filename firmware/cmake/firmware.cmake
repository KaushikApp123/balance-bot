# Firmware build for the STM32F411CEU6 (WeAct BlackPill).
# Vendor code is pulled at configure time and pinned to tags.

include(FetchContent)
set(FETCHCONTENT_QUIET ON)

FetchContent_Declare(freertos
    GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel.git
    GIT_TAG        V11.1.0
    GIT_SHALLOW    TRUE)
FetchContent_Declare(hal
    GIT_REPOSITORY https://github.com/STMicroelectronics/stm32f4xx-hal-driver.git
    GIT_TAG        v1.8.5
    GIT_SHALLOW    TRUE)
FetchContent_Declare(cmsis_device
    GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis-device-f4.git
    GIT_TAG        v2.6.9
    GIT_SHALLOW    TRUE)
FetchContent_Declare(cmsis_core
    GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis-core.git
    GIT_TAG        v5.9.0
    GIT_SHALLOW    TRUE)

# Populate only; we pick the files ourselves instead of using their CMake.
foreach(dep freertos hal cmsis_device cmsis_core)
    FetchContent_GetProperties(${dep})
    if(NOT ${dep}_POPULATED)
        FetchContent_Populate(${dep})
    endif()
endforeach()

set(HAL_SRC ${hal_SOURCE_DIR}/Src)
set(HAL_MODULES
    stm32f4xx_hal.c stm32f4xx_hal_cortex.c stm32f4xx_hal_rcc.c stm32f4xx_hal_rcc_ex.c
    stm32f4xx_hal_gpio.c stm32f4xx_hal_dma.c stm32f4xx_hal_i2c.c stm32f4xx_hal_tim.c
    stm32f4xx_hal_tim_ex.c stm32f4xx_hal_uart.c stm32f4xx_hal_adc.c stm32f4xx_hal_adc_ex.c
    stm32f4xx_hal_flash.c stm32f4xx_hal_flash_ex.c stm32f4xx_hal_iwdg.c
    stm32f4xx_hal_pwr.c stm32f4xx_hal_pwr_ex.c stm32f4xx_hal_exti.c)
list(TRANSFORM HAL_MODULES PREPEND ${HAL_SRC}/)

set(RTOS_SRC
    ${freertos_SOURCE_DIR}/tasks.c
    ${freertos_SOURCE_DIR}/queue.c
    ${freertos_SOURCE_DIR}/list.c
    ${freertos_SOURCE_DIR}/stream_buffer.c
    ${freertos_SOURCE_DIR}/timers.c
    ${freertos_SOURCE_DIR}/portable/GCC/ARM_CM4F/port.c
    ${freertos_SOURCE_DIR}/portable/MemMang/heap_4.c)

set(APP_SRC
    app/src/main.c
    app/src/board.c
    app/src/stm32f4xx_it.c
    app/src/shared.c
    app/src/imu_mpu6050.c
    app/src/motor.c
    app/src/gains_store.c
    app/src/task_safety.c
    app/src/task_sensor.c
    app/src/task_balance.c
    app/src/task_motor.c
    app/src/task_telemetry.c
    app/src/task_logger.c
    app/src/syscalls.c)

set(VENDOR_INC
    ${hal_SOURCE_DIR}/Inc
    ${cmsis_device_SOURCE_DIR}/Include
    ${cmsis_core_SOURCE_DIR}/Include
    ${freertos_SOURCE_DIR}/include
    ${freertos_SOURCE_DIR}/portable/GCC/ARM_CM4F)

# Vendor code builds with its own (quieter) warnings.
add_library(vendor STATIC
    ${HAL_MODULES} ${RTOS_SRC}
    ${cmsis_device_SOURCE_DIR}/Source/Templates/system_stm32f4xx.c
    ${cmsis_device_SOURCE_DIR}/Source/Templates/gcc/startup_stm32f411xe.s)
target_include_directories(vendor PUBLIC app/inc core/inc ${VENDOR_INC})
target_compile_definitions(vendor PUBLIC STM32F411xE USE_HAL_DRIVER HSE_VALUE=25000000U)
target_compile_options(vendor PRIVATE -w)

add_executable(balance_bot.elf ${APP_SRC} ${CORE_SOURCES})
target_link_libraries(balance_bot.elf PRIVATE vendor m)
target_compile_options(balance_bot.elf PRIVATE
    -Wall -Wextra -Wshadow -Wdouble-promotion -Wno-unused-parameter -Werror
    $<$<CONFIG:Debug>:-Og -g3>
    $<$<NOT:$<CONFIG:Debug>>:-O2 -g>)

set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker/STM32F411CEUx_FLASH.ld)
target_link_options(balance_bot.elf PRIVATE
    -T${LINKER_SCRIPT}
    -Wl,-Map=${CMAKE_BINARY_DIR}/balance_bot.map,--cref
    -Wl,--print-memory-usage
    -u _printf_float)
set_target_properties(balance_bot.elf PROPERTIES LINK_DEPENDS ${LINKER_SCRIPT})

# No .bin on purpose: a raw image spans the gap over the gains sector and
# flashing it would wipe saved gains. Flash the .elf or .hex instead.
add_custom_command(TARGET balance_bot.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex balance_bot.elf balance_bot.hex
    COMMAND ${CMAKE_SIZE} balance_bot.elf
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

# cmake --build build --target flash   (ST-Link + OpenOCD)
add_custom_target(flash
    COMMAND openocd -f interface/stlink.cfg -f target/stm32f4x.cfg
            -c "program balance_bot.elf verify reset exit"
    DEPENDS balance_bot.elf
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
