cmake_minimum_required(VERSION 3.20)

# ---------------------------------------------------------------------------
# STM32F407VETx + FreeRTOS —— GCC (arm-none-eabi) + CMake 构建
#
# 与 Keil（PRJ/STM32_CAN_CMD.uvprojx）、EIDE（PRJ/.eide/eide.yml）并行，
# 三条链路互不干扰：
#   * Keil          : PRJ/Listings/            （ARMCC + RVDS 移植层 + CMSIS/startup_stm32f40_41xxx.s）
#   * EIDE          : PRJ/build/V1.0/
#   * 本文件        : build-gcc/{Debug,Release}/（GCC + GCC 移植层 + cmake/startup_stm32f40_41xxx.S）
#
# FreeRTOS-Kernel V11.3.0：内核 FreeRTOS/*.c，GCC 端口 FreeRTOS/portable/GCC/ARM_CM4F。
#
# 构建：
#   cmake --preset debug   && cmake --build --preset debug
#   cmake --preset release && cmake --build --preset release
# ---------------------------------------------------------------------------

project(STM32_CAN_CMD LANGUAGES C ASM)

# 未指定构建类型时默认 Debug（-Og -g3），与 Keil -O0 语义接近
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "构建类型" FORCE)
endif()

set(PROJ_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")

# ---------------------------------------------------------------------------
# 源文件清单（四层结构；与 Keil 工程收录的 .c 逐个对齐）
# ---------------------------------------------------------------------------
set(APP_SOURCES
    "${PROJ_ROOT}/APP/main.c"
    "${PROJ_ROOT}/APP/stm32f4xx_it.c"
)

set(BSP_SOURCES
    "${PROJ_ROOT}/BSP/board.c"
    "${PROJ_ROOT}/BSP/can.c"
    "${PROJ_ROOT}/BSP/Emm_V5.c"
)

set(CONTRACT_SOURCES
    "${PROJ_ROOT}/CONTRACT/contract.c"
    "${PROJ_ROOT}/CONTRACT/PID_Mod.c"
    "${PROJ_ROOT}/CONTRACT/oled.c"
)

set(DEVICE_SOURCES
    "${PROJ_ROOT}/DEVICE/chassis_drive.c"
    "${PROJ_ROOT}/DEVICE/lift_axis.c"
    "${PROJ_ROOT}/DEVICE/grip_seq.c"
    "${PROJ_ROOT}/DEVICE/servo.c"
    "${PROJ_ROOT}/DEVICE/input.c"
    "${PROJ_ROOT}/DEVICE/vision.c"
)

set(APPLOG_SOURCES
    "${PROJ_ROOT}/APP_LOG/nav.c"
    "${PROJ_ROOT}/APP_LOG/nav_seg.c"
    "${PROJ_ROOT}/APP_LOG/nav_align.c"
    "${PROJ_ROOT}/APP_LOG/nav_util.c"
    "${PROJ_ROOT}/APP_LOG/avoid.c"
)

set(IFACE_SOURCES
    "${PROJ_ROOT}/IFACE/auxiliary.c"
    "${PROJ_ROOT}/IFACE/screen.c"
)

set(CMSIS_SOURCES
    "${PROJ_ROOT}/CMSIS/system_stm32f4xx.c"
)

set(DRIVERS_SOURCES
    "${PROJ_ROOT}/DRIVERS/delay.c"
    "${PROJ_ROOT}/DRIVERS/fifo.c"
)

# 标准外设库：只收工程实际用到的 7 个外设文件
set(LIB_SOURCES
    "${PROJ_ROOT}/LIB/STM32F4xx_StdPeriph_Driver/src/misc.c"
    "${PROJ_ROOT}/LIB/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c"
    "${PROJ_ROOT}/LIB/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c"
    "${PROJ_ROOT}/LIB/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_can.c"
    "${PROJ_ROOT}/LIB/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.c"
    "${PROJ_ROOT}/LIB/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.c"
    "${PROJ_ROOT}/LIB/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c"
)

# FreeRTOS V11.3.0 内核 + GCC/ARM_CM4F 端口
# croutine.c 未纳入：configUSE_CO_ROUTINES = 0，V11.x 已标记弃用
set(FREERTOS_ROOT "${PROJ_ROOT}/FreeRTOS")

set(FREERTOS_SOURCES
    "${FREERTOS_ROOT}/tasks.c"
    "${FREERTOS_ROOT}/queue.c"
    "${FREERTOS_ROOT}/list.c"
    "${FREERTOS_ROOT}/timers.c"
    "${FREERTOS_ROOT}/event_groups.c"
    "${FREERTOS_ROOT}/stream_buffer.c"
    "${FREERTOS_ROOT}/heap_4.c"
    "${FREERTOS_ROOT}/portable/GCC/ARM_CM4F/port.c"
)

# GCC 语法启动文件（向量表逐项对齐 ARMCC 版）
set(STARTUP_SOURCE "${PROJ_ROOT}/cmake/startup_stm32f40_41xxx.S")

# newlib 裸机系统调用桩（替代 --specs=nosys.specs，避免其 4 条链接告警）
set(SYSCALLS_SOURCE "${PROJ_ROOT}/cmake/syscalls.c")

set(LINKER_SCRIPT "${PROJ_ROOT}/cmake/STM32F407VETx_FLASH.ld")

add_executable(${PROJECT_NAME}
    ${APP_SOURCES}
    ${BSP_SOURCES}
    ${CONTRACT_SOURCES}
    ${DEVICE_SOURCES}
    ${APPLOG_SOURCES}
    ${IFACE_SOURCES}
    ${CMSIS_SOURCES}
    ${DRIVERS_SOURCES}
    ${LIB_SOURCES}
    ${FREERTOS_SOURCES}
    ${STARTUP_SOURCE}
    ${SYSCALLS_SOURCE}
)

# ---------------------------------------------------------------------------
# 头文件搜索路径
# ---------------------------------------------------------------------------
target_include_directories(${PROJECT_NAME} PRIVATE
    "${PROJ_ROOT}/CONTRACT"
    "${PROJ_ROOT}/DEVICE"
    "${PROJ_ROOT}/APP_LOG"
    "${PROJ_ROOT}/IFACE"
    "${PROJ_ROOT}/APP"
    "${PROJ_ROOT}/BSP"
    "${PROJ_ROOT}/CMSIS"
    "${PROJ_ROOT}/DRIVERS"
    "${FREERTOS_ROOT}"
    "${FREERTOS_ROOT}/include"
    "${FREERTOS_ROOT}/portable/GCC/ARM_CM4F"
    "${PROJ_ROOT}/LIB/STM32F4xx_StdPeriph_Driver/inc"
)

# ---------------------------------------------------------------------------
# 预处理宏（与 Keil 的 Define 一致）
# ---------------------------------------------------------------------------
target_compile_definitions(${PROJECT_NAME} PRIVATE
    STM32F40_41xxx
    USE_STDPERIPH_DRIVER
)

# ---------------------------------------------------------------------------
# 目标架构与编译选项
#   -funsigned-char : ARMCC 默认 char 为无符号，GCC 默认为有符号；
#                     不加这一条，oled_font / 字符串处理会出现行为差异
# ---------------------------------------------------------------------------
set(CPU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)

target_compile_options(${PROJECT_NAME} PRIVATE
    ${CPU_FLAGS}
    $<$<COMPILE_LANGUAGE:C>:-std=gnu99>
    $<$<COMPILE_LANGUAGE:C>:-ffunction-sections>
    $<$<COMPILE_LANGUAGE:C>:-fdata-sections>
    $<$<COMPILE_LANGUAGE:C>:-fno-common>
    $<$<COMPILE_LANGUAGE:C>:-funsigned-char>
    $<$<COMPILE_LANGUAGE:C>:-Wall>
    $<$<COMPILE_LANGUAGE:C>:$<$<CONFIG:Debug>:-Og;-g3;-gdwarf-2>>
    $<$<COMPILE_LANGUAGE:C>:$<$<CONFIG:Release>:-Os>>
    $<$<COMPILE_LANGUAGE:C>:$<$<CONFIG:RelWithDebInfo>:-O2;-g3;-gdwarf-2>>
    $<$<COMPILE_LANGUAGE:C>:$<$<CONFIG:MinSizeRel>:-Os>>
)

# -Wmisleading-indentation 只对 ST 原厂文件关闭，其余文件保留该告警（0 Warning 验收口径）
set_source_files_properties(
    "${PROJ_ROOT}/CMSIS/system_stm32f4xx.c"
    PROPERTIES COMPILE_OPTIONS "-Wno-misleading-indentation"
)

# oled_font.h 是二维字库表，缺内层花括号属 GCC 的 -Wmissing-braces 提示，ARMCC 不报
set_source_files_properties(
    "${PROJ_ROOT}/CONTRACT/oled.c"
    PROPERTIES COMPILE_OPTIONS "-Wno-missing-braces"
)

# PID_Mod.h 把内部辅助函数全部前置声明为 static、实现留在 PID_Mod.c，
# 任何包含它的编译单元都会撞 -Wunused-function；只对包含方关闭该告警
set_source_files_properties(
    "${PROJ_ROOT}/DEVICE/chassis_drive.c"
    PROPERTIES COMPILE_OPTIONS "-Wno-unused-function"
)

# ---------------------------------------------------------------------------
# 链接选项
#   nano.specs            : 等价 Keil 侧 MicroLIB
#   syscalls.c            : 自带桩，不引 nosys.specs
#   no-warn-rwx-segments  : .data 初值镜像与 .text 同在 Flash，ld 会把该 LOAD 段标 RWE
# ---------------------------------------------------------------------------
target_link_options(${PROJECT_NAME} PRIVATE
    ${CPU_FLAGS}
    "-T${LINKER_SCRIPT}"
    --specs=nano.specs
    -Wl,--gc-sections
    -Wl,-Map=${PROJECT_NAME}.map,--cref
    -Wl,--print-memory-usage
    -Wl,--no-warn-rwx-segments
)

# 数学库必须走 target_link_libraries：-lm 若放进 link_options 会排在目标文件之前，
# ld 单遍扫描时 cosf/sinf 尚未被引用，库里成员不会被拉入 → undefined reference。
target_link_libraries(${PROJECT_NAME} PRIVATE m)

# ---------------------------------------------------------------------------
# 产物：elf / hex / bin + 尺寸报告
# ---------------------------------------------------------------------------
set_target_properties(${PROJECT_NAME} PROPERTIES
    OUTPUT_NAME "${PROJECT_NAME}"
    SUFFIX ".elf"
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND "${CMAKE_OBJCOPY}" -O ihex   "$<TARGET_FILE:${PROJECT_NAME}>" "${PROJECT_NAME}.hex"
    COMMAND "${CMAKE_OBJCOPY}" -O binary "$<TARGET_FILE:${PROJECT_NAME}>" "${PROJECT_NAME}.bin"
    COMMAND "${CMAKE_SIZE}" "$<TARGET_FILE:${PROJECT_NAME}>"
    COMMENT "生成 ${PROJECT_NAME}.hex / ${PROJECT_NAME}.bin 并打印尺寸"
    VERBATIM
)

# ---------------------------------------------------------------------------
# 配置摘要
# ---------------------------------------------------------------------------
message(STATUS "----------------------------------------")
message(STATUS " 构建类型  : ${CMAKE_BUILD_TYPE}")
message(STATUS " C 编译器  : ${CMAKE_C_COMPILER}")
message(STATUS " 链接脚本  : ${LINKER_SCRIPT}")
message(STATUS " 输出目录  : ${CMAKE_BINARY_DIR}")
message(STATUS "----------------------------------------")
