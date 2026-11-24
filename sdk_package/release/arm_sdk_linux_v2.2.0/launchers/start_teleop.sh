#!/bin/sh
# 主从臂遥操 —— 一键启动（Linux / macOS）
#
#   ./start_teleop.sh                      # 用 arm_link_config.json 的地址
#   ./start_teleop.sh --preflight-only     # 只连通性/预检，不动臂
#   ./start_teleop.sh --sec 30
#   ./start_teleop.sh --leader sim:// --follower sim:// --sec 8 --yes
#
# 首次使用先给执行权限：chmod +x start_teleop.sh

cd "$(dirname "$0")" || exit 1

PYTHON="${PYTHON:-python3}"
if ! command -v "$PYTHON" >/dev/null 2>&1; then
    echo "  [错误] 找不到 $PYTHON。装一个 Python 3.8+，或用 PYTHON=/path/to/python3 指定。" >&2
    exit 2
fi

echo "  使用解释器: $PYTHON ($("$PYTHON" -c 'import sys;print(sys.version.split()[0])'))"
exec "$PYTHON" -X utf8 launch_teleop.py "$@"
