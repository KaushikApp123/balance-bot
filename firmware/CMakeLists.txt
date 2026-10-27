cmake_minimum_required(VERSION 3.20)

# Two builds from one tree:
#   Firmware : cmake -S firmware -B build -DCMAKE_TOOLCHAIN_FILE=firmware/cmake/arm-none-eabi.cmake
#   Host     : cmake -S firmware -B build-host      (unit tests + libbbcore for the sim)

project(balance_bot C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CORE_SOURCES
    core/src/kalman.c
    core/src/pid.c
    core/src/controller.c
    core/src/speed_est.c
    core/src/protocol.c
)

if(CMAKE_CROSSCOMPILING)
    include(cmake/firmware.cmake)
else()
    enable_testing()
    set(WARN -Wall -Wextra -Wshadow -Wdouble-promotion -Wno-unused-parameter)

    add_library(bbcore_static STATIC ${CORE_SOURCES})
    target_include_directories(bbcore_static PUBLIC core/inc)
    target_compile_options(bbcore_static PRIVATE ${WARN} -Werror)
    set_target_properties(bbcore_static PROPERTIES POSITION_INDEPENDENT_CODE ON)
    if(NOT MSVC)
        target_link_libraries(bbcore_static PUBLIC m)
    endif()

    # Shared library loaded by the Python SIL sim through ctypes.
    add_library(bbcore SHARED core/src/sil.c)
    target_link_libraries(bbcore PRIVATE bbcore_static)
    target_compile_options(bbcore PRIVATE ${WARN})
    set_target_properties(bbcore PROPERTIES C_VISIBILITY_PRESET hidden)

    add_executable(test_core test/test_core.c)
    target_link_libraries(test_core PRIVATE bbcore_static)
    target_compile_options(test_core PRIVATE ${WARN} -Wno-double-promotion)
    add_test(NAME core COMMAND test_core)
endif()
