@echo off
chcp 65001 >nul 2>&1
setlocal
cd /d "%~dp0"

rem ===========================================================================
rem  主从臂遥操 —— 双击即用
rem
rem  参数会原样透传给 launch_teleop.py：
rem      start_teleop.bat --preflight-only      只验连通性，不动臂
rem      start_teleop.bat --sec 30               跑 30 秒
rem      start_teleop.bat --leader sim:// --follower sim:// --sec 8 --yes
rem
rem  ⚠ 为什么开头要 chcp 65001：本目录路径含中文，且 Python 的输出里有
rem     ✓ ✗ ⚠ ═ ┌ │ 这些 GBK 里没有的字符 —— 不切 UTF-8 会在 Windows 控制台
rem     报编码错。Python 侧也会自己再设一次（双保险）。
rem ===========================================================================

set PYTHONUTF8=1
set PYTHONIOENCODING=utf-8

set "PY="
where py >nul 2>&1 && set "PY=py -3"
if not defined PY (
  where python >nul 2>&1 && set "PY=python"
)
if not defined PY (
  where python3 >nul 2>&1 && set "PY=python3"
)

if not defined PY (
  echo.
  echo   [错误] 这台机器上找不到 Python。
  echo   装一个 Python 3.8+，安装时勾选 "Add python.exe to PATH"，然后重开本窗口。
  echo   下载：https://www.python.org/downloads/
  echo   —— 或者直接在命令行里跑：python launch_teleop.py
  echo.
  pause
  exit /b 2
)

echo   解释器 : %PY%
%PY% -X utf8 launch_teleop.py %*
set RC=%ERRORLEVEL%

echo.
if "%RC%"=="0" (
  echo   [退出码 0] 正常结束
) else (
  echo   [退出码 %RC%]  0 正常 / 2 配置或预检不过 / 3 链路丢失 / 4 急停中止 / 5 未确认安全
)
echo   ---- 按任意键关闭窗口 ----
pause >nul
endlocal
exit /b %RC%
