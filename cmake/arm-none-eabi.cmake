# ---------------------------------------------------------------------------
# CMake 工具链文件：arm-none-eabi-gcc 裸机（bare-metal）交叉编译
#
# 用法：
#   cmake -S . -B build-gcc/Debug -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
# 或直接用 CMakePresets.json 里的预设（已内置该路径）。
#
# 工具链目录可覆盖：
#   -DARM_TOOLCHAIN_DIR="D:/path/to/bin"
# 留空则退回从 PATH 查找 arm-none-eabi-gcc。
# ---------------------------------------------------------------------------

set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# 本机默认使用官方 Arm GNU Toolchain；不存在时自动退回 PATH
set(ARM_TOOLCHAIN_DIR
    "C:/Program Files (x86)/Arm/GNU Toolchain mingw-w64-i686-arm-none-eabi/bin"
    CACHE PATH "arm-none-eabi 工具链 bin 目录；留空表示从 PATH 查找")

if(ARM_TOOLCHAIN_DIR AND NOT EXISTS "${ARM_TOOLCHAIN_DIR}/arm-none-eabi-gcc.exe")
    message(STATUS "工具链目录 ${ARM_TOOLCHAIN_DIR} 下未找到 arm-none-eabi-gcc.exe，改为从 PATH 查找")
    set(ARM_TOOLCHAIN_DIR "" CACHE PATH "" FORCE)
endif()

# 用 find_program 解析出带可执行后缀的绝对路径（Windows 下必须带 .exe，
# 直接写 "arm-none-eabi-gcc" 会被 CMake 判定为"非存在的编译器"）
macro(_resolve_tool _outvar _name)
    if(ARM_TOOLCHAIN_DIR)
        find_program(${_outvar} NAMES ${_name} HINTS "${ARM_TOOLCHAIN_DIR}" NO_DEFAULT_PATH)
    endif()
    if(NOT ${_outvar})
        find_program(${_outvar} NAMES ${_name})
    endif()
    if(NOT ${_outvar})
        message(FATAL_ERROR "未找到 ${_name}，请用 -DARM_TOOLCHAIN_DIR=<工具链 bin 目录> 指定")
    endif()
endmacro()

_resolve_tool(_TOOLCHAIN_C_COMPILER arm-none-eabi-gcc)
_resolve_tool(_TOOLCHAIN_OBJCOPY      arm-none-eabi-objcopy)
_resolve_tool(_TOOLCHAIN_SIZE         arm-none-eabi-size)

set(CMAKE_C_COMPILER   "${_TOOLCHAIN_C_COMPILER}")
set(CMAKE_ASM_COMPILER "${_TOOLCHAIN_C_COMPILER}")
set(CMAKE_OBJCOPY      "${_TOOLCHAIN_OBJCOPY}")
set(CMAKE_SIZE         "${_TOOLCHAIN_SIZE}")

# 交叉编译环境下不做"能否链接"的探测（裸机没有默认可执行文件）
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# 只在工具链目录内查找库与头文件
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
