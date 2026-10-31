#!/usr/bin/env bash
# 一键编译（本机 g++，多线程；已迁离 STM32/FreeRTOS）
# 用法：./build.sh [debug|release]   （缺省 debug）
#       ./build.sh run               （编译后直接运行）

set -euo pipefail
cd "$(dirname "$0")"

BUILD_DIR="build"
LOG="/tmp/build_posix.log"

# 支持 debug / release / run 三种参数
case "${1:-debug}" in
    debug)
        BT="Debug";;
    release)
        BT="Release";;
    run)
        BT="Debug"; RUN=1;;
    *)
        echo "用法：./build.sh [debug|release|run]" >&2
        exit 1;;
esac

echo "==> 配置构建（${BT}）..."
cmake -S . -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE="${BT}" > "${LOG}" 2>&1

echo "==> 编译中..."
cmake --build "${BUILD_DIR}" --clean-first >> "${LOG}" 2>&1

ERR=$(grep -c 'error:' "${LOG}" || true)
WARN=$(grep -c 'warning:' "${LOG}" || true)

echo ""
echo "==> 完成  Error=${ERR}  Warning=${WARN}"
echo "----------------------------------------"
tail -8 "${LOG}"

if [ "${RUN:-0}" = "1" ]; then
    echo ""
    echo "==> 运行 posix_sim（Ctrl+C 停止）..."
    exec "${BUILD_DIR}/posix_sim"
fi
