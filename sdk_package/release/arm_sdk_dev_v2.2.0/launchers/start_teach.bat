@echo off
chcp 65001 >nul 2>&1
setlocal
cd /d "%~dp0"

rem ===========================================================================
rem  遥操示教（录制 / 检查 / 回放）—— 双击即用
rem
rem  参数会原样透传给 launch_teach.py：
rem      start_teach.bat --mode list
rem      start_teach.bat --mode record --sec 30 --name demo1
rem      start_teach.bat --mode info trajectories/demo1.json
rem      start_teach.bat --mode play trajectories/demo1.json --speed 0.5
rem
rem  不带参数双击 = --mode record（用配置文件里的时长与名字）
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
  echo   —— 或者直接在命令行里跑：python launch_teach.py
  echo.
  pause
  exit /b 2
)

echo   解释器 : %PY%
%PY% -X utf8 launch_teach.py %*
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
