# ---------------------------------------------------------------------------
# POSIX 模拟构建 —— 本机（x86_64 Linux）用 gcc 编译同一份业务逻辑，
# 验证 nav / avoid / 机构 / 状态机在脱离 STM32 后仍能编过并跑多任务。
#
# 与 STM32 构建的差异：
#   1. FreeRTOS 端口换成 portable/ThirdParty/GCC/Posix（任务跑在 pthread 上）
#   2. 硬件实现文件（board.c/can.c/input.c/servo.c/oled.c/delay.c/
#      stm32f4xx_it.c/system_stm32f4xx.c）不编译，用 posix/stub/ 空壳替代
#   3. 用 posix/stub/stm32f4xx.h 与 posix/FreeRTOSConfig.h 替代真实 CMSIS 头
#
# 构建：
#   cmake -S posix -B posix/build
#   cmake --build posix/build
#   ./posix/build/posix_sim
# ---------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.20)
project(POSIX_SIM LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "构建类型" FORCE)
endif()

set(PROJ_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")

# ---------------------------------------------------------------------------
# FreeRTOS 内核（复用 STM32 工程同一份 V11.3.0 内核源码）
# ---------------------------------------------------------------------------
set(FREERTOS_ROOT "${PROJ_ROOT}/FreeRTOS")

set(FREERTOS_SOURCES
    "${FREERTOS_ROOT}/tasks.c"
    "${FREERTOS_ROOT}/queue.c"
    "${FREERTOS_ROOT}/list.c"
    "${FREERTOS_ROOT}/timers.c"
    "${FREERTOS_ROOT}/event_groups.c"
    "${FREERTOS_ROOT}/stream_buffer.c"
    "${FREERTOS_ROOT}/heap_4.c"
    # POSIX 端口（本目录）
    "${CMAKE_CURRENT_SOURCE_DIR}/freertos-port/port.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/freertos-port/utils/wait_for_event.c"
)

# ---------------------------------------------------------------------------
# 业务逻辑层（与 STM32 工程共用同一份源码，零改动）
# ---------------------------------------------------------------------------
set(APP_SOURCES
    "${PROJ_ROOT}/CONTRACT/contract.c"
    "${PROJ_ROOT}/CONTRACT/PID_Mod.c"
    "${PROJ_ROOT}/DEVICE/chassis_drive.c"
    "${PROJ_ROOT}/DEVICE/lift_axis.c"
    "${PROJ_ROOT}/DEVICE/grip_seq.c"
    "${PROJ_ROOT}/DEVICE/vision.c"
    "${PROJ_ROOT}/APP_LOG/nav.c"
    "${PROJ_ROOT}/APP_LOG/nav_seg.c"
    "${PROJ_ROOT}/APP_LOG/nav_align.c"
    "${PROJ_ROOT}/APP_LOG/nav_util.c"
    "${PROJ_ROOT}/APP_LOG/avoid.c"
    "${PROJ_ROOT}/IFACE/auxiliary.c"
    "${PROJ_ROOT}/IFACE/screen.c"
    "${PROJ_ROOT}/BSP/Emm_V5.c"
    "${PROJ_ROOT}/DRIVERS/fifo.c"
)

# ---------------------------------------------------------------------------
# 硬件空壳（替代真实 STM32 实现）
# ---------------------------------------------------------------------------
set(STUB_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/stub/board_stub.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/stub/can_stub.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/stub/input_stub.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/stub/servo_stub.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/stub/oled_stub.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/stub/delay_stub.c"
)

# ---------------------------------------------------------------------------
# Linux 原生底座（替换 STM32 外设的 socketcan / i2c / serial 等实现）
# ---------------------------------------------------------------------------
set(DEVICE_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/device/socketcan.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/device/pca9685.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/device/serial.c"
)

# ---------------------------------------------------------------------------
# 模拟入口
# ---------------------------------------------------------------------------
set(MAIN_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/posix_main.c")

add_executable(posix_sim
    ${MAIN_SOURCE}
    ${APP_SOURCES}
    ${STUB_SOURCES}
    ${DEVICE_SOURCES}
    ${FREERTOS_SOURCES}
)

# ---------------------------------------------------------------------------
# 头文件搜索路径：stub 头必须排在真实 CMSIS 头之前
# ---------------------------------------------------------------------------
target_include_directories(posix_sim PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"                      # posix/（FreeRTOSConfig.h）
    "${CMAKE_CURRENT_SOURCE_DIR}/stub"                 # stub/stm32f4xx.h 优先于 CMSIS
    "${CMAKE_CURRENT_SOURCE_DIR}/device"               # socketcan.h 等 Linux 底座
    "${CMAKE_CURRENT_SOURCE_DIR}/freertos-port"        # portmacro.h
    "${CMAKE_CURRENT_SOURCE_DIR}/freertos-port/utils"
    "${PROJ_ROOT}/CONTRACT"
    "${PROJ_ROOT}/DEVICE"
    "${PROJ_ROOT}/APP_LOG"
    "${PROJ_ROOT}/IFACE"
    "${PROJ_ROOT}/APP"
    "${PROJ_ROOT}/BSP"
    "${PROJ_ROOT}/DRIVERS"
    "${FREERTOS_ROOT}"
    "${FREERTOS_ROOT}/include"
)

# ---------------------------------------------------------------------------
# 预处理宏：无 STM32 相关宏（已彻底脱离 STM32 依赖）
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# 编译选项：本机 gcc，-funsigned-char 与实机一致
# ---------------------------------------------------------------------------
target_compile_options(posix_sim PRIVATE
    -funsigned-char
    -Wall
    $<$<CONFIG:Debug>:-Og;-g3>
    $<$<CONFIG:Release>:-O2>
)

# 链接 pthread（FreeRTOS POSIX 端口依赖）
target_link_libraries(posix_sim PRIVATE pthread m)

# PID_Mod.h 把内部辅助函数前置声明为 static，任何包含它的编译单元都会撞
# -Wunused-function；只对包含方 chassis_drive.c 关闭该告警（与 STM32 构建一致）
set_source_files_properties(
    "${PROJ_ROOT}/DEVICE/chassis_drive.c"
    PROPERTIES COMPILE_OPTIONS "-Wno-unused-function"
)
