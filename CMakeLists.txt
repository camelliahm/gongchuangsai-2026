# ---------------------------------------------------------------------------
# 工程构建 —— 本机（x86_64 Linux / 鲁班猫5）用 g++ 编译，跑多线程。
#
# 已迁离 STM32 与 FreeRTOS，纯 C++：
#   - 任务模型：std::thread / std::mutex / condition_variable
#   - 硬件底座：socketcan（CAN）/ serial（串口）/ pca9685（舵机），见 BSP/ 与 DEVICE/
#   - 基础类型头：include/types.h（全工程唯一）
#
# 构建：
#   ./build.sh
#   或 cmake -S . -B build && cmake --build build
#   ./build/posix_sim
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

# ---------------------------------------------------------------------------
# 源文件清单（按分层归位，均以工程根为基准）
# ---------------------------------------------------------------------------
set(APP_SOURCES
    CONTRACT/contract.cpp
    CONTRACT/PID_Mod.c
    CONTRACT/oled.cpp
    DEVICE/chassis_drive.cpp
    DEVICE/lift_axis.cpp
    DEVICE/grip_seq.cpp
    DEVICE/vision.cpp
    DEVICE/input.cpp
    DEVICE/servo.cpp
    APP_LOG/nav.cpp
    APP_LOG/nav_seg.cpp
    APP_LOG/nav_align.cpp
    APP_LOG/nav_util.cpp
    APP_LOG/avoid.cpp
    IFACE/auxiliary.cpp
    IFACE/screen.cpp
    BSP/Emm_V5.cpp
    BSP/can.cpp
    BSP/board.cpp
    DRIVERS/fifo.cpp
)

# Linux 原生底座（socketcan / i2c / serial）
set(DEVICE_SOURCES
    BSP/socketcan.cpp
    DEVICE/pca9685.cpp
    DEVICE/serial.cpp
)

# 入口
set(MAIN_SOURCE main.cpp)

add_executable(posix_sim
    ${MAIN_SOURCE}
    ${APP_SOURCES}
    ${DEVICE_SOURCES}
)

# ---------------------------------------------------------------------------
# 头文件搜索路径
# ---------------------------------------------------------------------------
target_include_directories(posix_sim PRIVATE
    include
    CONTRACT
    DEVICE
    APP_LOG
    IFACE
    BSP
    DRIVERS
)

# ---------------------------------------------------------------------------
# 编译选项：g++，-funsigned-char 与实机一致
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
# -Wunused-function；只对包含方 chassis_drive.cpp 关闭该告警
set_source_files_properties(
    DEVICE/chassis_drive.cpp
    PROPERTIES COMPILE_OPTIONS "-Wno-unused-function"
)
