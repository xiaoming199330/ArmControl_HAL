#!/usr/bin/env python3
"""
ArmControl_HAL C 代码诊断日志注入器
向 C 源文件注入 cprint 风格的诊断日志（嵌入式/ISR 安全守则内置）。

用法:
    python c_log_injector.py --list-rules
    python c_log_injector.py --target-file Core/Src/main.c \
        --rules entry,branch,corr-fsm --prefix "[DIAG]" --in-place --backup
    python c_log_injector.py --target-file Core/Src/kinematics.c \
        --rules entry,data-transform -o /tmp/kinematics.instrumented.c

守则（脚本自动执行）:
  - 不向 ISR / 200Hz 实时函数注入 entry/exit/branch（见 REALTIME_SKIP）
  - 不向 cprint 自身注入（防递归）
  - traj-step 规则输出分频采样（默认 40 分频 = 5Hz）
  - 目标文件无 cprint 时自动插入 DIAG_PRINT 宏钩子（需用户映射到控制台打印）
"""

import argparse
import re
import shutil
import sys
from pathlib import Path

RULES = {
    "entry":         "函数入口日志（跳过 ISR/实时函数）",
    "exit":          "return 前出口日志（每函数上限3处）",
    "branch":        "if 分支进入日志（全文件上限8处）",
    "dxl-call":      "DXL/STS3215 总线调用前日志（上限10处）",
    "traj-step":     "200Hz 轨迹插值分频采样（5Hz）",
    "corr-fsm":      "自适应修正状态机日志",
    "cmd-parse":     "命令解析入口日志",
    "data-transform":"角度/量纲转换点日志（上限8处）",
    "safety":        "安全检查分支日志（E-Stop/钳制，上限6处）",
    "flash":         "Flash 加载/保存结果日志",
}

# ISR / 200Hz 实时路径 / 主循环高频函数：禁止注入普通日志
REALTIME_SKIP = {
    "USART6_IRQHandler", "HAL_TIM_PeriodElapsedCallback", "Error_Handler",
    "assert_failed", "cprint", "s_acc", "Trajectory_Interpolate",
    "gravity_comp_j2", "DXL_UpdateCRC",
    # 200Hz 插值路径上的热函数
    "servo_deg_to_rad", "rad_to_servo_deg", "deg_to_sts3215_raw", "sts3215_raw_to_deg",
    "AngleToRaw", "RawToAngle", "STS3215_SetPositionFast", "DXL_WriteNoReply",
    "DXL_Transmit", "DXL_SetDirection", "DXL_FlushErrors",
    # 每次主循环迭代都执行
    "cmdMultiPointService", "Console_TryReadLine", "IsEStop", "IsLimit", "clamp_angle",
}

FUNC_SIG = re.compile(
    r"^((?:static\s+)?(?:inline\s+)?(?:void|int|float|double|uint8_t|uint16_t|uint32_t|int8_t|int16_t|int32_t|uint64_t|HAL_StatusTypeDef|bool|char)\s+"
    r"(\w+)\s*\([^;{}]*\)\s*\{)", re.M)


def find_functions(content):
    """返回 [(name, body_start, body_end)] — 基于单行签名 + 花括号配对。"""
    funcs = []
    for m in FUNC_SIG.finditer(content):
        name = m.group(2)
        depth, i = 0, m.end() - 1
        while i < len(content):
            c = content[i]
            if c == "{":
                depth += 1
            elif c == "}":
                depth -= 1
                if depth == 0:
                    break
            i += 1
        funcs.append((name, m.start(), m.end(), i))
    return funcs


class CLogInjector:
    def __init__(self, prefix="[DIAG]", decimate=40):
        self.prefix = prefix
        self.decimate = decimate
        self.count = 0
        self.has_cprint = False

    def logfn(self):
        return "cprint" if self.has_cprint else "DIAG_PRINT"

    # ---------- 函数级规则 ----------
    def inject_entry(self, content):
        funcs = find_functions(content)
        out, last = [], 0
        for name, sig_start, sig_end, _ in funcs:
            if name in REALTIME_SKIP:
                continue
            out.append(content[last:sig_end])
            out.append(f'\n    {self.logfn()}("{self.prefix} {name} ENTRY\\r\\n");')
            self.count += 1
            last = sig_end
        out.append(content[last:])
        return "".join(out)

    def inject_exit(self, content):
        funcs = find_functions(content)
        out, last = [], 0
        for name, sig_start, sig_end, body_end in funcs:
            out.append(content[last:sig_end])
            body = content[sig_end:body_end]
            if name not in REALTIME_SKIP:
                n = [0]
                def repl(m):
                    if n[0] >= 3:
                        return m.group(0)
                    n[0] += 1
                    self.count += 1
                    indent = m.group(1)
                    return (f'\n{indent}{self.logfn()}("{self.prefix} {name} EXIT\\r\\n");'
                            f'{m.group(0)}')
                body = re.sub(r"\n([ \t]*)return\b", lambda m: repl(m), body)
            out.append(body)
            last = body_end
        out.append(content[last:])
        return "".join(out)

    def inject_branch(self, content):
        n = [0]
        def repl(m):
            if n[0] >= 8:
                return m.group(0)
            n[0] += 1
            self.count += 1
            indent = m.group(1)
            cond = m.group(2).strip().replace('"', "'")
            return (f'{m.group(0)}\n{indent}    {self.logfn()}('
                    f'"{self.prefix} BRANCH if({cond})\\r\\n");')
        return re.sub(r"\n([ \t]*)if\s*\(([^(){}]*)\)\s*\{", repl, content)

    # ---------- 行级规则 ----------
    def _inject_before_lines(self, content, pattern, mklog, cap):
        lines = content.split("\n")
        out, n = [], 0
        for line in lines:
            if n < cap and re.search(pattern, line) and self.prefix not in line:
                stripped = line.lstrip()
                if stripped.startswith(("//", "*", "/*")):
                    out.append(line)
                    continue
                indent = line[: len(line) - len(stripped)]
                out.append(indent + mklog(line) + ";")
                n += 1
                self.count += 1
            out.append(line)
        return "\n".join(out)

    def _inject_after_lines(self, content, pattern, mklog, cap):
        lines = content.split("\n")
        out, n = [], 0
        for line in lines:
            out.append(line)
            if n < cap and re.search(pattern, line) and self.prefix not in line:
                stripped = line.lstrip()
                if stripped.startswith(("//", "*", "/*")):
                    continue
                indent = line[: len(line) - len(stripped)]
                out.append(indent + mklog(line) + ";")
                n += 1
                self.count += 1
        return "\n".join(out)

    def inject_dxl_call(self, content):
        return self._inject_before_lines(
            content,
            r"\b(DXL_Read|DXL_Write|DXL_TransmitReceive|DXL_Ping|STS3215_SetPosition|STS3215_GetPosition)\s*\(",
            lambda l: f'{self.logfn()}("{self.prefix} DXL-CALL %s:%d\\r\\n", __FILE__, __LINE__)',
            10)

    def inject_traj_step(self, content):
        pat = r"Trajectory_Interpolate\s*\("
        block = (
            f'{{ static uint16_t diag_div = 0; if (++diag_div >= {self.decimate}) {{ diag_div = 0;\n'
            f'        {self.logfn()}("{self.prefix} TRAJ t=%lu a=%d,%d,%d,%d,%d,%d\\r\\n", '
            f'(unsigned long)traj_tick, (int)angles[0], (int)angles[1], (int)angles[2], '
            f'(int)angles[3], (int)angles[4], (int)angles[5]); }} }}')
        return self._inject_after_lines(content, pat, lambda l: block, 3)

    def inject_corr_fsm(self, content):
        pat = r"correction_target\[j\]\s*-\s*actual"
        return self._inject_after_lines(
            content, pat,
            lambda l: (f'{self.logfn()}("{self.prefix} CORR J%d err=%d gain=%d%% osc=%d stall=%d\\r\\n", '
                       f'j+1, (int)error, (int)(corr_gain[j]*100), oscillation_count[j], stall_count[j])'),
            3)

    def inject_cmd_parse(self, content):
        pat = r"char\s+cmd\s*=\s*tokens\[0\]\[0\]"
        return self._inject_after_lines(
            content, pat,
            lambda l: f'{self.logfn()}("{self.prefix} CMD ntok=%d cmd=%c\\r\\n", ntok, cmd)',
            1)

    def inject_data_transform(self, content):
        return self._inject_after_lines(
            content,
            r"=\s*(rad_to_servo_deg|servo_deg_to_rad|deg_to_sts3215_raw|sts3215_raw_to_deg)\s*\(",
            lambda l: f'{self.logfn()}("{self.prefix} XFORM %s:%d\\r\\n", __FILE__, __LINE__)',
            8)

    def inject_safety(self, content):
        return self._inject_after_lines(
            content,
            r"if\s*\(\s*IsEStop\s*\(\s*\)\s*\)\s*\{",
            lambda l: f'{self.logfn()}("{self.prefix} SAFETY E-Stop branch %s:%d\\r\\n", __FILE__, __LINE__)',
            6)

    def inject_flash(self, content):
        return self._inject_after_lines(
            content,
            r"PID_Flash(Save|Load)\s*\(",
            lambda l: f'{self.logfn()}("{self.prefix} FLASH op %s:%d\\r\\n", __FILE__, __LINE__)',
            4)

    def ensure_print_hook(self, content):
        """无 cprint 的文件插入 DIAG_PRINT 宏钩子（默认空操作，避免编译失败）。"""
        if self.has_cprint or "DIAG_PRINT" in content:
            return content
        hook = (
            "\n/* [DIAG-INJECT] 调试打印钩子：默认空操作。\n"
            "   发布前请移除或将 DIAG_PRINT 映射到控制台输出（如 extern 的 uart printf）。 */\n"
            "#ifndef DIAG_PRINT\n#define DIAG_PRINT(...)  ((void)0)\n#endif\n")
        includes = list(re.finditer(r"^#include[^\n]*\n", content, re.M))
        if includes:
            pos = includes[-1].end()
            return content[:pos] + hook + content[pos:]
        return hook + content

    def process(self, content, rules):
        self.count = 0
        self.has_cprint = "cprint" in content
        if "all" in rules:
            rules = list(RULES.keys())
        methods = {
            "entry": self.inject_entry, "exit": self.inject_exit,
            "branch": self.inject_branch, "dxl-call": self.inject_dxl_call,
            "traj-step": self.inject_traj_step, "corr-fsm": self.inject_corr_fsm,
            "cmd-parse": self.inject_cmd_parse, "data-transform": self.inject_data_transform,
            "safety": self.inject_safety, "flash": self.inject_flash,
        }
        for r in rules:
            m = methods.get(r)
            if m:
                content = m(content)
        content = self.ensure_print_hook(content)
        return content


def main():
    ap = argparse.ArgumentParser(description="ArmControl_HAL C 代码诊断日志注入器")
    ap.add_argument("--target-file", help="单个目标文件")
    ap.add_argument("--target-files", help="逗号分隔的多个目标文件")
    ap.add_argument("--rules", default="", help="逗号分隔规则; all 全选")
    ap.add_argument("--prefix", default="[DIAG]", help="日志前缀")
    ap.add_argument("--decimate", type=int, default=40, help="traj-step 分频(默认40=5Hz)")
    ap.add_argument("--output", "-o", help="输出文件")
    ap.add_argument("--in-place", action="store_true", help="原地修改")
    ap.add_argument("--backup", action="store_true", help="原地修改时创建 .bak")
    ap.add_argument("--list-rules", action="store_true", help="列出规则")
    args = ap.parse_args()

    if args.list_rules:
        print("可用注入规则:")
        for k, v in RULES.items():
            print(f"  {k:15} - {v}")
        print(f"\n实时/ISR 保护名单(不注入): {', '.join(sorted(REALTIME_SKIP))}")
        return

    if not args.rules:
        print("错误: 需要 --rules（用 --list-rules 查看）", file=sys.stderr)
        sys.exit(1)

    rules = [r.strip() for r in args.rules.split(",") if r.strip()]
    invalid = [r for r in rules if r not in RULES and r != "all"]
    if invalid:
        print(f"错误: 无效规则 {invalid}", file=sys.stderr)
        sys.exit(1)

    files = []
    if args.target_file:
        files.append(args.target_file)
    elif args.target_files:
        files.extend(f.strip() for f in args.target_files.split(","))
    else:
        print("错误: 需要 --target-file 或 --target-files", file=sys.stderr)
        sys.exit(1)

    for f in files:
        p = Path(f)
        if not p.exists():
            print(f"  错误: 文件不存在 {f}")
            continue
        injector = CLogInjector(prefix=args.prefix, decimate=args.decimate)
        content = p.read_text(encoding="utf-8")
        result = injector.process(content, rules)
        if injector.count == 0:
            print(f"  跳过: {f}（无注入点）")
            continue
        if args.in_place:
            if args.backup:
                shutil.copy2(p, p.with_suffix(p.suffix + ".bak"))
            out = p
        elif args.output:
            out = Path(args.output)
        else:
            out = p.with_suffix(".instrumented" + p.suffix)
        out.write_text(result, encoding="utf-8")
        hook_note = "" if injector.has_cprint else "（已插入 DIAG_PRINT 空宏钩子，需映射到打印函数）"
        print(f"  {f}: 注入 {injector.count} 处 → {out} {hook_note}")


if __name__ == "__main__":
    main()
