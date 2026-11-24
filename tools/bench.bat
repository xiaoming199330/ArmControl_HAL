@echo off
chcp 65001 >nul
set PYTHONUTF8=1
set PYTHONIOENCODING=utf-8
setlocal

rem ============================================================
rem  UDP vs HTTP 对照基准 —— 一键启动
rem  用法:
rem     bench.bat                        用默认 IP 192.168.7.120
rem     bench.bat 192.168.7.121          指定目标臂 IP
rem     bench.bat --mode rtt             只测往返延迟（最快，~20s）
rem     bench.bat --selftest             离线自检（不连板子）
rem ============================================================

set "SCRIPT=%~dp0udp_vs_http_bench.py"
set "IP=192.168.7.120"

rem 第一个参数如果是 IP（形如 192.x）就取走当目标
set "REST="
:parse
if "%~1"=="" goto run
echo %~1| findstr /r /c:"^[0-9][0-9]*\.[0-9]" >nul && (set "IP=%~1") || (set "REST=%REST% %~1")
shift
goto parse

:run
rem 三级解释器探测
set "PY=python"
where py >nul 2>nul && set "PY=py -3"

echo.
echo   目标臂 : %IP%:5000  (UDP)
echo   解释器 : %PY%
echo.
echo   [提示] 网口需先 `link net` -^> `link save` -^> 按 RESET 才通。
echo          探针只用 PING(0x01)，只读、不动臂。
echo.

if "%REST%"=="" (
    %PY% -X utf8 "%SCRIPT%" --mode all --target-ip %IP% --n 500 --rates 50,100,200,400,800,1200 --sec 3 --burst 400 --with-burst
) else (
    if "%IP%"=="192.168.7.120" (
        %PY% -X utf8 "%SCRIPT%" --target-ip %IP% %REST%
    ) else (
        %PY% -X utf8 "%SCRIPT%" --mode all --target-ip %IP% --n 500 --rates 50,100,200,400,800,1200 --sec 3 --burst 400 --with-burst %REST%
    )
)

set "RC=%ERRORLEVEL%"
echo.
echo   退出码 = %RC%   (0=正常  2=参数/环境错  3=一个字节都不回，先查网口)
pause
endlocal
