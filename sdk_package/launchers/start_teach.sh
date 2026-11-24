#!/bin/sh
# 遥操示教（录制 / 检查 / 回放）—— 一键启动（Linux / macOS）
#
#   ./start_teach.sh --mode list
#   ./start_teach.sh --mode record --sec 30 --name demo1
#   ./start_teach.sh --mode info trajectories/demo1.json
#   ./start_teach.sh --mode play trajectories/demo1.json --speed 0.5
#
# 首次使用先给执行权限：chmod +x start_teach.sh

cd "$(dirname "$0")" || exit 1

PYTHON="${PYTHON:-python3}"
if ! command -v "$PYTHON" >/dev/null 2>&1; then
    echo "  [错误] 找不到 $PYTHON。装一个 Python 3.8+，或用 PYTHON=/path/to/python3 指定。" >&2
    exit 2
fi

echo "  使用解释器: $PYTHON ($("$PYTHON" -c 'import sys;print(sys.version.split()[0])'))"
exec "$PYTHON" -X utf8 launch_teach.py "$@"
