#!/bin/sh
# 装依赖。只需要跑一次。
#   · arm_sdk 本体 —— 从本包自带的 wheel 装（免编译、免联网）
#   · pyserial    —— 只有走串口路线（relay / 固件直连）才需要
#   · numpy       —— 只有导出/读取 .npz 轨迹才需要
#
#   chmod +x install_deps.sh && ./install_deps.sh

cd "$(dirname "$0")" || exit 1

PYTHON="${PYTHON:-python3}"
command -v "$PYTHON" >/dev/null 2>&1 || { echo "  [错误] 找不到 $PYTHON" >&2; exit 2; }
echo "  解释器: $PYTHON"

echo ""
echo "  [1/2] 装 arm_sdk（优先用包内 wheel）"
if [ -f ../dist/arm_sdk-2.0.0-py3-none-any.whl ]; then
    "$PYTHON" -m pip install --upgrade --force-reinstall ../dist/arm_sdk-2.0.0-py3-none-any.whl
else
    echo "  包内没找到 wheel，改用源码安装…"
    (cd .. && "$PYTHON" -m pip install -e .)
fi

echo ""
echo "  [2/2] 选装 pyserial / numpy"
"$PYTHON" -m pip install pyserial numpy

echo ""
echo "  装完自检："
"$PYTHON" -m arm_sdk selftest
