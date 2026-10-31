# ---------------------------------------------------------------------------
# POSIX 构建 —— 本机（x86_64 Linux）用 g++ 编译业务逻辑，跑多线程。
#
# 工程已全面转向 Linux/鲁班猫5，STM32 与 FreeRTOS 均已移除：
#   - 任务模型：std::thread / std::mutex / condition_variable
#   - 硬件实现：posix/stub/ 空壳（board/can/input/servo/oled/delay）
#   - CMSIS 头：posix/stub/stm32f4xx.h（全工程唯一）
#
# 构建：
#   cmake -S posix -B posix/build
#   cmake --build posix/build
#   ./posix/build/posix_sim
# ---------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.20)
project(POSIX_SIM LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "构建类型" FORCE)
endif()

set(PROJ_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")

# ---------------------------------------------------------------------------
# 业务逻辑层（与 STM32 工程共用同一份源码，零改动）
# ---------------------------------------------------------------------------
set(APP_SOURCES
    "${PROJ_ROOT}/CONTRACT/contract.cpp"
    "${PROJ_ROOT}/CONTRACT/PID_Mod.c"
    "${PROJ_ROOT}/DEVICE/chassis_drive.cpp"
    "${PROJ_ROOT}/DEVICE/lift_axis.cpp"
    "${PROJ_ROOT}/DEVICE/grip_seq.cpp"
    "${PROJ_ROOT}/DEVICE/vision.cpp"
    "${PROJ_ROOT}/APP_LOG/nav.cpp"
    "${PROJ_ROOT}/APP_LOG/nav_seg.cpp"
    "${PROJ_ROOT}/APP_LOG/nav_align.cpp"
    "${PROJ_ROOT}/APP_LOG/nav_util.cpp"
    "${PROJ_ROOT}/APP_LOG/avoid.cpp"
    "${PROJ_ROOT}/IFACE/auxiliary.cpp"
    "${PROJ_ROOT}/IFACE/screen.cpp"
    "${PROJ_ROOT}/BSP/Emm_V5.cpp"
    "${PROJ_ROOT}/DRIVERS/fifo.cpp"
)

# ---------------------------------------------------------------------------
# 硬件空壳（替代真实 STM32 实现）
# ---------------------------------------------------------------------------
set(STUB_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/stub/board_stub.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/stub/can_stub.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/stub/input_stub.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/stub/servo_stub.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/stub/oled_stub.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/stub/delay_stub.cpp"
)

# ---------------------------------------------------------------------------
# Linux 原生底座（替换 STM32 外设的 socketcan / i2c / serial 等实现）
# ---------------------------------------------------------------------------
set(DEVICE_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/device/socketcan.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/device/pca9685.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/device/serial.cpp"
)

# ---------------------------------------------------------------------------
# 模拟入口
# ---------------------------------------------------------------------------
set(MAIN_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/posix_main.cpp")

add_executable(posix_sim
    ${MAIN_SOURCE}
    ${APP_SOURCES}
    ${STUB_SOURCES}
    ${DEVICE_SOURCES}
)

# ---------------------------------------------------------------------------
# 头文件搜索路径：stub 头必须排在真实 CMSIS 头之前
# ---------------------------------------------------------------------------
target_include_directories(posix_sim PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"                      # posix/
    "${CMAKE_CURRENT_SOURCE_DIR}/stub"                 # 唯一的 stm32f4xx.h
    "${CMAKE_CURRENT_SOURCE_DIR}/device"               # socketcan.h 等 Linux 底座
    "${PROJ_ROOT}/CONTRACT"
    "${PROJ_ROOT}/DEVICE"
    "${PROJ_ROOT}/APP_LOG"
    "${PROJ_ROOT}/IFACE"
    "${PROJ_ROOT}/BSP"
    "${PROJ_ROOT}/DRIVERS"
)

# ---------------------------------------------------------------------------
# 编译选项：本机 g++，-funsigned-char 与实机一致
# ---------------------------------------------------------------------------
target_compile_options(posix_sim PRIVATE
    -funsigned-char
    -Wall
    $<$<CONFIG:Debug>:-Og;-g3>
    $<$<CONFIG:Release>:-O2>
)

# 链接 pthread（std::thread 依赖）
target_link_libraries(posix_sim PRIVATE pthread m)

# PID_Mod.h 把内部辅助函数前置声明为 static，任何包含它的编译单元都会撞
# -Wunused-function；只对包含方 chassis_drive.cpp 关闭该告警（与 STM32 构建一致）
set_source_files_properties(
    "${PROJ_ROOT}/DEVICE/chassis_drive.cpp"
    PROPERTIES COMPILE_OPTIONS "-Wno-unused-function"
)
