# ============================================================
#  UDP vs HTTP 对照基准 —— PowerShell 入口
#
#  ⚠ 为什么需要这个脚本：
#     PowerShell 不支持 bash 的反斜杠 `\` 续行（那是 bash/POSIX 语法）。
#     写成多行时，第二行会被当成新语句，`--n` 里的 `--` 被判成
#     "一元运算符 -- 后面缺少表达式" 而报 ParserError。
#     PowerShell 的续行符是反引号 ` ，但更省事的办法就是用本脚本。
#
#  用法:
#     .\bench.ps1                          默认 IP 192.168.7.120，跑全套
#     .\bench.ps1 -TargetIp 192.168.7.121
#     .\bench.ps1 -Mode rtt                只测往返延迟（最快）
#     .\bench.ps1 -Mode rtt -N 1000
#     .\bench.ps1 -SelfTest                离线自检（不连板子）
#     .\bench.ps1 -TargetIp 192.168.7.120 -Extra @("--quiet")
# ============================================================

param(
    [string]$TargetIp = "192.168.7.120",
    [ValidateSet("rtt", "sweep", "burst", "all")]
    [string]$Mode = "all",
    [int]$N = 500,
    [string]$Rates = "50,100,200,400,800,1200",
    [double]$Sec = 3,
    [int]$Burst = 400,
    [string]$Json = "",
    [string[]]$Extra = @(),
    [switch]$SelfTest
)

$ErrorActionPreference = "Stop"

# ---- UTF-8 输出：否则含 ✓ ✗ ⚠ ═ 的输出会在中文 Windows 上 UnicodeEncodeError
$env:PYTHONUTF8 = "1"
$env:PYTHONIOENCODING = "utf-8"
try { [Console]::OutputEncoding = [Text.Encoding]::UTF8 } catch { }

# ---- 找到解释器（优先 py -3，其次 python）
$py = $null
if (Get-Command py -ErrorAction SilentlyContinue) { $py = @("py", "-3") }
elseif (Get-Command python -ErrorAction SilentlyContinue) { $py = @("python") }
else { Write-Error "找不到 Python。装一个，或把 python.exe 加进 PATH。"; exit 2 }

$script = Join-Path $PSScriptRoot "udp_vs_http_bench.py"
if (-not (Test-Path $script)) { Write-Error "找不到 $script"; exit 2 }

$argv = @("-X", "utf8", $script)

if ($SelfTest) {
    $argv += @("--selftest")
} else {
    $argv += @("--mode", $Mode, "--target-ip", $TargetIp,
               "--n", "$N", "--rates", $Rates,
               "--sec", "$Sec", "--burst", "$Burst")
    if ($Mode -in @("burst", "all")) { $argv += "--with-burst" }
    if ($Json) { $argv += @("--json", $Json) }
}
if ($Extra.Count -gt 0) { $argv += $Extra }

Write-Host ""
Write-Host "  目标臂 : $TargetIp`:5000  (UDP)" -ForegroundColor Cyan
Write-Host "  模式   : $Mode" -ForegroundColor Cyan
Write-Host "  解释器 : $($py -join ' ')" -ForegroundColor DarkGray
Write-Host ""
Write-Host "  [提示] 网口需先 ``link net`` -> ``link save`` -> 按 RESET 才通。" -ForegroundColor DarkGray
Write-Host "         探针只用 PING(0x01)，只读、不动臂。" -ForegroundColor DarkGray
Write-Host ""

& $py[0] ($py[1..($py.Count-1)] + $argv)
$rc = $LASTEXITCODE

Write-Host ""
Write-Host "  退出码 = $rc   (0=正常  2=参数/环境错  3=一个字节都不回，先查网口)" -ForegroundColor $(if ($rc -eq 0) { "Green" } else { "Yellow" })
exit $rc
