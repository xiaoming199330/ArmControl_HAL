@echo off
chcp 65001 >nul 2>&1
setlocal
cd /d "%~dp0"

rem ============================================================
rem  装依赖。只需要跑**一次**。
rem    · arm_sdk 本体 —— 从本包自带的 wheel 装（免编译、免联网）
rem    · pyserial    —— 只有走串口路线（relay / 固件直连）才需要
rem    · numpy       —— 只有导出/读取 .npz 轨迹才需要
rem ============================================================

set PYTHONUTF8=1
set PYTHONIOENCODING=utf-8

set PY=
where py >nul 2>&1 && set "PY=py -3"
if not defined PY (
  where python >nul 2>&1 && set "PY=python"
)
if not defined PY (
  echo   [错误] 找不到 Python。先装 Python 3.8+ 并勾选 Add to PATH。& pause & exit /b 2
)

echo   解释器: %PY%
echo.
echo   [1/2] 装 arm_sdk（优先用包内 wheel）
if exist "..\dist\arm_sdk-2.0.0-py3-none-any.whl" (
  %PY% -m pip install --upgrade --force-reinstall "..\dist\arm_sdk-2.0.0-py3-none-any.whl"
) else (
  echo   包内没找到 wheel，改用源码安装（需要联网拿构建依赖）…
  pushd ..
  %PY% -m pip install -e .
  popd
)

echo.
echo   [2/2] 选装 pyserial / numpy（串口路线 & .npz 轨迹才需要）
%PY% -m pip install pyserial numpy

echo.
echo   装完自检：
%PY% -m arm_sdk selftest

echo.
echo   ---- 按任意键关闭 ----
pause >nul
endlocal
