#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""全指令逐条测试器 —— 38 条网口二进制 + 114 条控制台文本，一条不落。

设计前提：**只测不改**（用户要求"不要改里面的参数"）
------------------------------------------------------------------
  本脚本**物理上不含**任何改参数的执行路径。命令分四档，只有前两档会真的下发：

    read      纯查询。零副作用。默认就跑这一档。
    motion    会动电机 —— 但参数全部取自"**读回当前值再原样写回**"，
              臂不产生位移（少数必然动的命令单列 MOVE，要额外开关）。
    write     会改运行期参数 / 落 Flash / 烧 EEPROM。**不下发**，只进报告。
    forbidden 会改量纲基准 / 松力矩 / 卡死主循环 / 抢 ETH 引脚。**不下发**。

  所以报告里的 write/forbidden 行是"已识别但按你的要求跳过"，附带原因。
  任何没被本文件分级过的命令，一律按 forbidden 处理（fail-safe：宁可漏测）。

三种后端
------------------------------------------------------------------
  --sim            离线内存假臂（arm_sdk.sim）。只能测 38 条二进制 ——
                   Python 版模拟器的文本解释器只覆盖自检用到的少数命令，
                   跑 114 条文本会得到一堆假的 "Unknown command"。
  --host IP        真机 UDP（需板子跑 NET 固件）。二进制走原生帧；
                   文本经 TEXT_EXEC 透传（124 条里 113 条可下发）。
  --serial COMx    真机串口裸文本，**像人在 COM11 上敲键**：
                   连字符回显、行尾 \r\n、执行后的 "> " 提示符一起测。
                   这是唯一能覆盖全部 124 条文本命令的路径。

用法
------------------------------------------------------------------
  py -3 tools/test_all_commands.py                       # 离线，read 档
  py -3 tools/test_all_commands.py --plan                # 只列清单，不下发
  py -3 tools/test_all_commands.py --host 192.168.7.120  # 真机网口
  py -3 tools/test_all_commands.py --serial COM11        # 真机串口（最全）
  py -3 tools/test_all_commands.py --host ... --level motion --i-know-it-moves
  py -3 tools/test_all_commands.py --only 0x01,s,f,FK_QUERY
  py -3 tools/test_all_commands.py --host ... --md outputs/test_all.md --json outputs/test_all.json

⚠ motion 档之前请先确认：臂周围无人、E-Stop 可及、臂已回中位、
  固件 fw-tag 与本次预期一致（报告开头会打出来）。
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
from dataclasses import asdict, dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SDK_DIR = os.path.join(ROOT, "sdk_package")
if SDK_DIR not in sys.path:
    sys.path.insert(0, SDK_DIR)

from arm_sdk import protocol as P                                  # noqa: E402
from arm_sdk.client import ArmClient                               # noqa: E402
from arm_sdk.transport import make_transport                       # noqa: E402

CONFIG = os.path.join(ROOT, "config", "arm_config.json")

LEVELS = ("read", "motion", "write", "forbidden")
LEVEL_ORDER = {lv: i for i, lv in enumerate(LEVELS)}


class Skip(Exception):
    """构造参数时判定「这条不该测」⇒ 记 SKIP 而不是 FAIL（带原因）。"""


# ==========================================================================
# 结果
# ==========================================================================

@dataclass
class Result:
    key: str
    kind: str                     # "bin" | "text"
    level: str
    sent: str                     # 实际下发的内容（未下发为空）
    status: str = "SKIP"          # PASS / PASS* / GATED / SKIP / FAIL / TIMEOUT / LISTED
    detail: str = ""
    ms: float = 0.0
    bytes_out: int = 0
    bytes_in: int = 0

    @property
    def executed(self) -> bool:
        return self.status not in ("LISTED", "SKIP")


@dataclass
class Report:
    backend: str = ""
    target: str = ""
    level: str = "read"
    started: str = ""
    config_total: int = 0
    text_registered: int = 0
    unclassified: List[str] = field(default_factory=list)
    preflight: List[str] = field(default_factory=list)
    results: List[Result] = field(default_factory=list)

    def add(self, r: Result) -> Result:
        self.results.append(r)
        return r

    def summary(self) -> Dict[str, int]:
        c: Dict[str, int] = {}
        for r in self.results:
            c[r.status] = c.get(r.status, 0) + 1
        return c


# ==========================================================================
# 命令表：网口二进制（38 条）
# ==========================================================================
# payload 用 lambda(ctx) -> bytes|None；返回 None 表示"前置条件不足，跳过"

BinFn = Callable[[Any], Optional[bytes]]

BIN_SPECS: List[Tuple[int, str, str, BinFn, str, str]] = [
    # ---- read 档（21 条）：纯查询，参数全部取自读回值 ----
    (P.CMD_PING, "PING", "read",
     lambda c: b"", "ACK", "连通性基线；不带 payload"),
    (P.CMD_GET_STATUS, "GET_STATUS", "read",
     lambda c: b"", "STATUS", "六轴 raw/pos/load/temp/err 快照"),
    (P.CMD_GET_JOINT_DETAIL, "GET_JOINT_DETAIL", "read",
     lambda c: P.payload_get_joint_detail(0xFF), "JOINT_DETAIL", "六轴详情 62B"),
    (P.CMD_GET_LIMITS, "GET_LIMITS", "read",
     lambda c: b"", "LIMITS", "运行时关节限位"),
    (P.CMD_GET_INFO, "GET_INFO", "read",
     lambda c: b"", "INFO", "fw-tag / 构建 / 链路（判版本就靠它）"),
    (P.CMD_GET_FLAGS, "GET_FLAGS", "read",
     lambda c: b"", "FLAGS", "estop/traj/stream 状态位"),
    (P.CMD_TEXT_EXEC, "TEXT_EXEC", "read",
     lambda c: P.payload_text_exec("s"), "TEXT_OUT", "文本透传：下发只读命令 s"),
    (P.CMD_TEXT_FETCH, "TEXT_FETCH", "read",
     lambda c: P.payload_text_fetch(0, P.TEXT_CHUNK), "TEXT_FETCH",
     "分页取上一条的剩余输出；无前置缓冲时回 NACK(PARAM) 也算连通"),
    (P.CMD_STOP, "STOP", "read",
     lambda c: b"", "ACK", "停机指令；臂静止时下发无副作用（放本档最后）"),
    (P.CMD_FK_QUERY, "FK_QUERY", "read",
     lambda c: b"", "FK", "正解位姿"),
    (P.CMD_IK_QUERY, "IK_QUERY", "read",
     lambda c: c.ik_payload_nonmoving(), "IK_QUERY",
     "只用算不动：目标取 FK 读回的**当前位姿** ⇒ 原地解"),
    (P.CMD_GET_GRAVITY, "GET_GRAVITY", "read",
     lambda c: b"", "GRAVITY", "重力前馈系数"),
    (P.CMD_GET_CORRECTION, "GET_CORRECTION", "read",
     lambda c: b"", "CORRECTION", "自适应校正配置"),
    (P.CMD_GET_PID, "GET_PID", "read",
     lambda c: P.payload_get_pid(0), "PID", "J1 的 PID"),
    (P.CMD_PARAM_READ, "PARAM_READ", "read",
     lambda c: P.payload_param_read(1), "PARAM_READ", "按 id 读单个运行期参数"),
    (P.CMD_PARAM_COUNT, "PARAM_COUNT", "read",
     lambda c: b"", "PARAM_COUNT", "运行期参数条数"),
    (P.CMD_PARAM_LIST, "PARAM_LIST", "read",
     lambda c: P.payload_param_list(0), "PARAM_LIST", "参数元数据首页（id/范围/单位）"),
    (P.CMD_TEACH_LIST, "TEACH_LIST", "read",
     lambda c: P.payload_teach_list(0), "TEACH_LIST", "示教文件 0 的元数据"),
    (P.CMD_TEACH_READ, "TEACH_READ", "read",
     lambda c: P.payload_teach_read(0, 0), "TEACH_READ", "示教文件 0 的前 5 个样本"),
    (P.CMD_TEACH_TOTAL, "TEACH_TOTAL", "read",
     lambda c: b"", "TEACH_TOTAL", "示教存储用量"),
    (P.CMD_CALIB_ENTER, "CALIB_ENTER", "read",
     lambda c: P.payload_magic(), "NACK", "一律 NACK(ERR_PARAM) 是**设计**："
     "交互式校准从链路进会卡死主循环，本工程无看门狗"),

    # ---- motion 档（4 条）：参数全部读回原样写回 ⇒ 原地 ----
    (P.CMD_MOVE_JOINTS, "MOVE_JOINTS", "motion",
     lambda c: c.move_joints_inplace(), "ACK", "六轴目标=当前 servo 角 ⇒ 原地"),
    (P.CMD_STREAM_JOINTS, "STREAM_JOINTS", "motion",
     lambda c: c.stream_inplace(), "ACK", "流目标=当前逻辑角 ⇒ 原地"),
    (P.CMD_MOVE_JOINT_ONE, "MOVE_JOINT_ONE", "motion",
     lambda c: c.move_one_inplace(0), "ACK", "J1 目标=当前角 ⇒ 原地"),
    (P.CMD_MOVE_CART, "MOVE_CART", "motion",
     lambda c: c.move_cart_inplace(), "ACK", "笛卡尔目标=FK 当前位姿 ⇒ 原地"),

    # ---- write 档（9 条）：不下发 ----
    (P.CMD_LOG_SET_LEVEL, "LOG_SET_LEVEL", "write",
     lambda c: P.payload_log_set_level(2), "ACK", "改运行期日志级别"),
    (P.CMD_SET_TORQUE, "SET_TORQUE", "write",
     lambda c: P.payload_set_torque(0, False), "ACK", "关扭矩 ⇒ 臂会下沉"),
    (P.CMD_SET_BUS_MODE, "SET_BUS_MODE", "write",
     lambda c: P.payload_set_bus_mode(0), "ACK", "改总线分组方式"),
    (P.CMD_SET_GRAVITY, "SET_GRAVITY", "write",
     lambda c: P.payload_set_gravity(0.0), "ACK", "改重力前馈系数"),
    (P.CMD_SET_CORRECTION, "SET_CORRECTION", "write",
     lambda c: P.payload_set_correction(True, 2.0, 3), "ACK", "改自适应校正配置"),
    (P.CMD_PARAM_WRITE, "PARAM_WRITE", "write",
     lambda c: P.payload_param_write(1, 0.0), "ACK", "写运行期参数"),
    (P.CMD_PARAM_SAVE, "PARAM_SAVE", "write",
     lambda c: P.payload_magic(), "ACK", "参数落 Flash 扇区 11"),
    (P.CMD_TEACH_DELETE, "TEACH_DELETE", "write",
     lambda c: P.payload_teach_delete(0), "ACK", "删示教文件（tombstone）"),
    (P.CMD_TEACH_COMPACT, "TEACH_COMPACT", "write",
     lambda c: b"", "ACK", "回收示教 tombstone 空间"),

    # ---- forbidden（4 条）：不下发 ----
    (P.CMD_SERVO_INIT, "SERVO_INIT", "forbidden",
     lambda c: P.payload_servo_init(0xFF), "ACK", "重写 PID/限位/中位并 RESET"),
    (P.CMD_CENTER_CALIB, "CENTER_CALIB", "forbidden",
     lambda c: P.payload_magic(), "ACK", "一键中位 = 标定，会废掉量纲基准"),
    (P.CMD_ZERO_RESET, "ZERO_RESET", "forbidden",
     lambda c: P.payload_zero_reset(False), "ACK", "复位 Home 零点基准"),
    (P.CMD_SYS_RESET, "SYS_RESET", "forbidden",
     lambda c: P.payload_magic(), "ACK", "软复位；复位丢力矩、臂下沉"),
]


# ==========================================================================
# 命令表：控制台文本（114 条登记项）
# ==========================================================================
# key 与 config/arm_config.json 的 commands[].name **逐字一致**；
# value = (level, 实际下发的命令行, 备注)
#
# ⚠ 没出现在本表里的 name 一律按 forbidden 处理，绝不执行（fail-safe）。

TEXT_SPECS: Dict[str, Tuple[str, Optional[str], str]] = {
    # ---------- read：只查询，零副作用 ----------
    "n":            ("read", "n", "扫描 Bus A/B 看 6/6"),
    "Z":            ("read", "Z", "全 ID 扫描 1..253：几十秒，--include-slow 才跑"),
    "s":            ("read", "s", "六轴状态表"),
    "p J":          ("read", "@expand", "逐轴读位置（展开成 p 0..p 5）"),
    "T":            ("read", "T", "六轴温度"),
    "O":            ("read", "O", "六轴负载"),
    "E":            ("read", "E", "六轴硬件错误位"),
    "f":            ("read", "f", "正解：末端位姿"),
    "C":            ("read", "C", "显示世界坐标偏移"),
    "L":            ("read", "L", "显示关节限位"),
    "K":            ("read", "K", "显示六轴 P 增益"),
    "I":            ("read", "I", "显示六轴 I 增益"),
    "D":            ("read", "D", "显示六轴 D 增益"),
    "N":            ("read", "N 0", "显示 J1 完整 PID"),
    "V":            ("read", "V", "显示舵机内部速度"),
    "A":            ("read", "A", "显示自适应校正配置"),
    "G":            ("read", "G", "显示重力前馈 K"),
    "LP":           ("read", "LP", "列出全部算法参数"),
    "LL":           ("read", "LL", "显示 runlog 级别"),
    "d":            ("read", "d", "CAN 总线诊断"),
    "h":            ("read", "h", "分组帮助（5406B，会被 2048B 截断，正常）"),
    "H":            ("read", "H", "完整命令表（固件侧标注 planned）"),
    "ms":           ("read", "ms", "关节角快照 + 主臂 TX 计数"),
    "lk":           ("read", "lk", "主从链路状态"),
    "mf":           ("read", "mf", "显示主臂转发开关"),
    "ma":           ("read", "ma", "显示主臂主动模式"),
    "sf":           ("read", "sf", "显示从臂跟随开关"),
    "zi":           ("read", "zi", "零点/IK 限位状态"),
    "link":         ("read", "link", "显示链路偏好（不带参数=查询）"),
    "files":        ("read", "files", "list 的别名"),
    "list":         ("read", "list", "列出示教文件"),
    "rec list":     ("read", "rec list", "未保存录制摘要"),
    "teach dump":   ("read", "teach dump", "打印全部零力参数"),
    "teach pwm":    ("read", "teach pwm", "PWM 零力模式状态"),
    "teach p":      ("read", "teach p", "显示软跟随 PID"),
    "teach i":      ("read", "teach i", "显示重力平衡积分增益"),
    "teach g":      ("read", "teach g", "显示 PWM 重力增益"),
    "teach v":      ("read", "teach v", "显示 PWM 速度阻尼"),
    "teach c":      ("read", "teach c", "显示库仑摩擦补偿"),
    "teach d":      ("read", "teach d", "显示静摩擦抖动"),
    "teach o":      ("read", "teach o", "显示恒定偏置占空比"),
    "teach cap":    ("read", "teach cap", "显示每轴占空比上限"),
    "teach b":      ("read", "teach b", "显示死区规避"),
    "teach test":   ("read", "teach test", "显示单轴测试模式"),
    "S":            ("read", "S", "停机（放本档最后：会中断正在跑的轨迹）"),
    # 主从 / CAN / NET 只读
    "ct":           ("read", "ct", "CAN 链路测试计数"),
    "cn":           ("read", "cn", "CAN 状态：计数器/TEC/REC/LEC"),
    "nx":           ("read", "nx", "NET 状态 / 调试开关 / 扫描"),
    "np":           ("read", "np", "NET 引脚探针（只读取证）"),
    "nm":           ("read", "nm", "MIIM 软探针（MDIO 线健康）"),
    "nt":           ("read", "nt", "REF_CLK 时钟探针"),
    "rc":           ("read", "rc", "RC 接收机捕获（只读）"),
    # 这几条在 NET 构建下会被固件拦下 —— 这里正是要验"拦截是否生效"
    "can":          ("read", "can", "⚠ NET 构建下抢引脚，预期被拦"),
    "cx":           ("read", "cx", "⚠ 抢引脚，预期被拦"),
    "cp":           ("read", "cp", "⚠ 抢引脚，预期被拦"),
    "cy":           ("read", "cy", "⚠ 抢引脚，预期被拦"),
    "cz":           ("read", "cz", "⚠ 抢引脚，预期被拦"),
    "cw":           ("read", "cw", "⚠ 抢引脚，预期被拦"),
    "ps":           ("read", "ps", "⚠ 抢引脚，预期被拦"),

    # ---------- motion：会动/改运行期状态，需显式开关 ----------
    "m":            ("motion", "@inplace-m", "六目标=当前角 ⇒ 原地"),
    "a J deg":      ("motion", "@inplace-a", "单轴写回当前角 ⇒ 原地"),
    "r J raw":      ("motion", "@inplace-r", "单轴写回当前 raw ⇒ 原地"),
    "g":            ("motion", "@inplace-g", "笛卡尔写回当前位姿 ⇒ 原地"),
    "pv ms":        ("motion", "@stream-pv", "开 200ms 再关（pv 0）"),
    "pf ms":        ("motion", "@stream-pf", "开 200ms 再关（pf 0）"),
    "tm ms [f]":    ("motion", "@stream-tm", "开 200ms 再关（tm 0）"),
    "P":            ("motion", "P 0", "⚠ MOVE：回 HOME 预设姿态"),
    "z J":          ("motion", "@home-j", "⚠ MOVE：单轴回中"),
    "z a":          ("motion", "z a", "⚠ MOVE：六轴回中"),
    "w J":          ("motion", "w 0", "⚠ MOVE：J1 扫描测试"),
    "B":            ("motion", None, "⚠ MOVE：两点往返，需自备参数"),
    "BG":           ("motion", None, "⚠ MOVE：笛卡尔往返，需自备参数"),
    "QL":           ("motion", None, "⚠ MOVE：多点循环，需已存路点"),
    "QLC":          ("motion", None, "⚠ MOVE：连续循环，需已存路点"),
    "W":            ("motion", "W 1", "⚠ MOVE：挥手 1 次"),
    "play":         ("motion", None, "⚠ MOVE：回放录制"),

    # ---------- write：改参数/落 Flash/烧 EEPROM（不下发） ----------
    "b":            ("write", "b 0", "切总线模式：部分舵机会失联"),
    "F":            ("write", "F", "PID/校正/速度/参数落 Flash"),
    "X":            ("write", "X", "恢复默认 PID 到 RAM + 舵机"),
    "t J 0/1":      ("write", "t 0 0", "关扭矩 ⇒ 臂下沉"),
    "q":            ("write", "q 1 180 180 180 180 180 180", "写路点"),
    "qg":           ("write", "qg 1 0 0 200", "写 XYZ 路点（经 IK）"),
    "qv":           ("write", "qv 1 50", "改段速度"),
    "qd":           ("write", "qd 1 500", "改驻留时间"),
    "C dx dy dz":   ("write", "C 0 0 0", "设世界坐标偏移"),
    "L J min max":  ("write", "L 0 63 303", "改运行时限位"),
    "Y":            ("write", "Y 0", "烧 PID 进舵机 EEPROM"),
    "R":            ("write", "R", "复位自适应状态"),
    "SP":           ("write", "SP 1 0", "按 id 写运行期参数"),
    "U":            ("write", "U", "寄存器读写诊断：语法不明，保守跳过"),
    "e":            ("write", "e", "CAN 回环自检：会发帧"),
    "ec":           ("write", "ec 1", "控制台回显开关：关了会静音 COM11"),
    "sp":           ("write", "sp 100", "主臂位置流"),
    "#P":           ("write", "#P 180 180 180 180 180 180", "PC 中继位置行接收"),
    "save":         ("write", "save", "示教录制存 Flash"),
    "del":          ("write", "del 0", "删示教文件"),
    "compact":      ("write", "compact", "回收示教空间"),
    "rec clear":    ("write", "rec clear", "丢弃未保存录制"),
    "teach exit":   ("write", "teach exit", "退出示教并恢复扭矩/速度/PID"),
    "teach lock":   ("write", "teach lock", "锁定当前姿态"),
    "teach f":      ("write", "teach f", "翻转 PWM 重力方向"),

    # ---------- forbidden：改量纲/松力矩/卡死主循环/抢引脚（不下发） ----------
    "i":            ("forbidden", "i", "初始化会重写 PID/限位/中位"),
    "i J":          ("forbidden", "i 0", "同上，单轴"),
    "M a":          ("forbidden", "M a", "一键中位=标定，废掉量纲基准"),
    "M J":          ("forbidden", "M 0", "单轴中位，同 M a 的量纲风险"),
    "c":            ("forbidden", "c", "交互式校准：for(;;) 等人按键，会卡死主循环"),
    "zr":           ("forbidden", "zr", "复位 Home 零点基准"),
    "teach":        ("forbidden", "teach", "进零力示教：松力矩，臂会掉"),
    "teach free":   ("forbidden", "teach free", "全轴松力矩"),
    "teach rec":    ("forbidden", "teach rec", "开始跟随录制"),
    "ml":           ("forbidden", "ml 1", "主臂 GPIO 回环：抢引脚"),
    "mg":           ("forbidden", "mg", "主臂 GPIO 回环：抢引脚"),
    "mp":           ("forbidden", "mp", "主臂引脚 DC 探针：抢引脚"),
    "mo":           ("forbidden", "mo r", "主臂引脚保持：抢引脚"),
}


# ==========================================================================
# 上下文：为 motion 档构造"原地"参数
# ==========================================================================

class Ctx:
    """给 payload 构造器提供"读回的当前值"，保证回写不产生位移。"""

    def __init__(self, client: ArmClient, verbose: bool = False) -> None:
        self.client = client
        self.verbose = verbose
        self._detail: Optional[List[Any]] = None
        self._pose: Optional[Any] = None

    # ---- 基础读取 ----
    def detail(self) -> Optional[List[Any]]:
        if self._detail is None:
            try:
                self._detail = self.client.get_joint_detail(0xFF)
            except Exception:                                       # noqa: BLE001
                self._detail = []
        return self._detail or None

    def pose(self) -> Optional[Any]:
        if self._pose is None:
            try:
                self._pose = self.client.fk_query()
            except Exception:                                       # noqa: BLE001
                self._pose = False
        return self._pose or None

    @staticmethod
    def _pick(obj: Any, *names: str, default: float = 0.0) -> float:
        """按候选字段名取值（SDK 各版本的字段名可能微调）。"""
        for n in names:
            v = getattr(obj, n, None)
            if isinstance(v, (int, float)):
                return float(v)
        return default

    def servo_deg(self) -> Optional[List[float]]:
        """六轴**舵机度**（0~360），由 pos_raw 换算：deg = raw * 360 / 4095。

        ⚠ 任一轴丢读（`ok=False`）就整条放弃 ⇒ 返回 None，让调用方 SKIP。
          绝不能拿 0 或丢读值当目标角 —— 那是"臂突然甩到 0°"的来源。
        """
        d = self.detail()
        if not d or len(d) < 6:
            return None
        out: List[float] = []
        for j in d:
            if not bool(getattr(j, "ok", True)):
                return None
            raw = self._pick(j, "pos_raw", "raw", "raw_pos", "position",
                             default=-1.0)
            if raw < 0:
                return None
            out.append(raw * 360.0 / 4095.0)
        return out

    def raw(self) -> Optional[List[float]]:
        d = self.detail()
        if not d or len(d) < 6:
            return None
        out: List[float] = []
        for j in d:
            if not bool(getattr(j, "ok", True)):
                return None
            raw = self._pick(j, "pos_raw", "raw", "raw_pos", "position",
                             default=-1.0)
            if raw < 0:
                return None
            out.append(raw)
        return out

    def logical_deg(self) -> Optional[List[float]]:
        """STREAM_JOINTS 要的是**逻辑角**，需要 calib(zero/offset/dir) 换算。

        本脚本**不猜**这个量纲：拿不到就返回 None ⇒ STREAM_JOINTS 自动 SKIP。
        量纲用错的后果是臂按错误的关节角运动，比不测严重得多。
        """
        raise Skip("STREAM_JOINTS 要的是逻辑角，需 calib(zero/offset/dir) 换算；"
                   "本脚本不猜量纲 ⇒ 跳过（要在真机上测请先用 client.logical_angles()）")

    # ---- 网口二进制 payload ----
    def move_joints_inplace(self) -> Optional[bytes]:
        s = self.servo_deg()
        if not s:
            return None
        return P.payload_move_joints(s, v=15.0)

    def stream_inplace(self) -> Optional[bytes]:
        s = self.logical_deg()
        if not s:
            return None
        return P.payload_stream_joints(s)

    def move_one_inplace(self, joint: int) -> Optional[bytes]:
        s = self.servo_deg()
        if not s:
            return None
        return P.payload_move_joint_one(joint, s[joint], v=15.0)

    def move_cart_inplace(self) -> Optional[bytes]:
        p = self.pose()
        if not p:
            return None
        return P.payload_move_cart(self._pick(p, "x"), self._pick(p, "y"),
                                   self._pick(p, "z"), self._pick(p, "pitch_deg", "pitch"))

    def ik_payload_nonmoving(self) -> Optional[bytes]:
        p = self.pose()
        if not p:
            return None
        return P.payload_ik_query(self._pick(p, "x"), self._pick(p, "y"),
                                  self._pick(p, "z"), self._pick(p, "pitch_deg", "pitch"))

    # ---- 真机 UDP：文本经 TEXT_EXEC ----
    def text_via_link(self, line: str) -> Tuple[str, bool, bool]:
        """返回 (输出, 被闸门拒绝, 被截断)。自己翻页以保留 gated 标志。"""
        r = self.client.request(P.CMD_TEXT_EXEC, P.payload_text_exec(line),
                                label="TEXT_EXEC %r" % line, idempotent=False)
        if not r.ok:
            raise RuntimeError("TEXT_EXEC 失败: %s" % (r.desc or r))
        first = P.decode_text_out(r.payload)
        if first.gated:
            return first.text, True, first.truncated
        parts = [first.raw]
        off = first.off + len(first.raw)
        truncated = first.truncated
        guard = P.TEXT_BUF_SZ // P.TEXT_CHUNK + 2
        while off < first.total and len(parts) < guard:
            r2 = self.client.request(P.CMD_TEXT_FETCH,
                                     P.payload_text_fetch(off, P.TEXT_CHUNK),
                                     label="TEXT_FETCH @%d" % off)
            if not r2.ok:
                break
            page = P.decode_text_out(r2.payload)
            if not page.raw:
                break
            parts.append(page.raw)
            off += len(page.raw)
            truncated = truncated or page.truncated
        return (b"".join(parts).decode("utf-8", "replace"), False, truncated)


# ==========================================================================
# 串口裸文本通道（像人在 COM11 上敲键）
# ==========================================================================

class RawSerial:
    """直接往 USART6 写文本行，读回显直到提示符 "> "。"""

    def __init__(self, port: str, baud: int = 115200, timeout: float = 2.0) -> None:
        import serial                                            # 延迟 import
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.timeout = timeout
        self.ser.reset_input_buffer()
        time.sleep(0.2)

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:                                           # noqa: BLE001
            pass

    def exec_line(self, line: str) -> Tuple[str, bool]:
        self.ser.reset_input_buffer()
        self.ser.write((line + "\r").encode("latin-1", "replace"))
        self.ser.flush()
        buf = bytearray()
        t0 = time.time()
        idle = 0.0
        while time.time() - t0 < self.timeout:
            chunk = self.ser.read(256)
            if chunk:
                buf.extend(chunk)
                idle = 0.0
                if buf.rstrip().endswith(b">"):
                    break
            else:
                idle += 0.2
                if buf and idle >= 0.6:
                    break
        text = bytes(buf).decode("utf-8", "replace")
        # 去掉首行回显 + 末尾提示符，只留正文
        lines = text.replace("\r\n", "\n").split("\n")
        if lines and lines[0].strip() == line:
            lines = lines[1:]
        body = "\n".join(lines).rstrip()
        if body.endswith(">"):
            body = body[:-1].rstrip()
        return body, False


# ==========================================================================
# 用例生成
# ==========================================================================

@dataclass
class Case:
    key: str
    kind: str
    level: str
    payload: Optional[bytes] = None
    line: Optional[str] = None
    expect: str = ""
    note: str = ""
    sent_label: str = ""


def build_bin_cases() -> List[Tuple[int, str, str, BinFn, str, str]]:
    return BIN_SPECS


def build_text_cases(include_slow: bool) -> List[Case]:
    out: List[Case] = []
    for name, (level, argv, note) in TEXT_SPECS.items():
        if argv is None:
            out.append(Case(key=name, kind="text", level=level, note=note + "（无安全参数 ⇒ 不自动构造）"))
            continue
        if argv == "@expand":                     # p J -> p 0..p 5
            for j in range(6):
                out.append(Case(key="p %d" % j, kind="text", level=level,
                                line="p %d" % j, note="逐轴读位置"))
            continue
        if argv.startswith("@"):                  # 让 Ctx 现算
            out.append(Case(key=name, kind="text", level=level, line=argv, note=note))
            continue
        if name == "Z" and not include_slow:
            out.append(Case(key=name, kind="text", level=level, line="Z",
                            note="跳过快命令（--include-slow 才跑）：" + note))
            continue
        out.append(Case(key=name, kind="text", level=level, line=argv, note=note))
    # ---- 兜底：接口真值源里还有哪些 name 没被我分级？一律按 forbidden 列出 ----
    for name in load_config_names():
        if name in TEXT_SPECS:
            continue
        out.append(Case(key=name, kind="text", level="forbidden",
                        note="⚠ 本脚本未分级 ⇒ 按禁止处理（fail-safe）；"
                             "若确认安全请补进 TEXT_SPECS"))
    return out


def load_config_names() -> List[str]:
    """从接口真值源读 114 条登记项的名字（用于覆盖率兜底）。"""
    try:
        with open(CONFIG, encoding="utf-8") as f:
            d = json.load(f)
        return [c["name"] for c in d["arm_config"]["commands"]]
    except Exception:                                               # noqa: BLE001
        return []


# ==========================================================================
# 执行
# ==========================================================================

GATED_HINTS = ("被拦", "拒绝", "禁用", "抢", "不可用", "NET", "闸门", "not allowed",
               "disabled", "blocked")

TEXT_GATED_KEYS = {"can", "cx", "cp", "cy", "cz", "cw", "ps", "c"}


def classify_text(text: str, gated: bool) -> Tuple[str, str]:
    """给一条文本命令的输出判定状态。"""
    t = text.strip()
    if gated:
        return "GATED", "被固件闸门拒绝（预期行为）：" + t[:80]
    if not t:
        return "PASS*", "无输出（部分命令本来就是静默成功）"
    low = t.lower()
    if "unknown command" in low or "unknown cmd" in low:
        return "FAIL", "固件不认这条命令: " + t[:80]
    if "err" in low and "error bitfield" not in low:
        for h in GATED_HINTS:
            if h.lower() in low:
                return "GATED", t[:80]
    return "PASS", t.split("\n")[0][:100]


def run_bin(report: Report, client: ArmClient, ctx: Ctx, cmd: int, key: str,
            level: str, fn: BinFn, expect: str, note: str,
            timeout: float) -> Result:
    r = Result(key=key, kind="bin", level=level, sent="", status="SKIP")
    if level in ("write", "forbidden"):
        return report.add(Result(key=key, kind="bin", level=level, sent="",
                                 status="LISTED", detail=note))
    try:
        pl = fn(ctx)
    except Skip as e:
        return report.add(Result(key=key, kind="bin", level=level, sent="",
                                 status="SKIP", detail=str(e)))
    except Exception as e:                                          # noqa: BLE001
        return report.add(Result(key=key, kind="bin", level=level, sent="",
                                 status="FAIL", detail="构造 payload 失败: %s" % e))
    if pl is None:
        return report.add(Result(key=key, kind="bin", level=level, sent="",
                                 status="SKIP", detail="前置读数不可用（臂未校准/离线）"))
    r.sent = "0x%02X + %dB" % (cmd, len(pl))
    r.bytes_out = len(pl)
    rep = client.request(cmd, pl, label=key, timeout=timeout,
                         idempotent=(level == "read"))
    r.ms = rep.latency_ms
    r.bytes_in = len(rep.payload or b"")
    if rep.timeout:
        r.status = "TIMEOUT"
        r.detail = "%.0f ms 无响应" % rep.latency_ms
    elif rep.cmd == P.RSP_NACK:
        if expect == "NACK":
            r.status = "PASS"
            r.detail = "如预期 NACK(%s)：%s" % (rep.err_name, note)
        else:
            r.status = "FAIL"
            r.detail = "NACK(%s) %s" % (rep.err_name, rep.desc)
    elif rep.ok:
        r.status = "PASS"
        r.detail = "%s %dB" % (P.cmd_name(rep.cmd), len(rep.payload or b""))
    else:
        r.status = "FAIL"
        r.detail = rep.desc
    if note and r.status in ("PASS", "PASS*"):
        r.detail += "  | " + note
    return report.add(r)


INPLACE_TEXT = {
    "@inplace-m": lambda ctx: _inplace_m(ctx),
    "@inplace-a": lambda ctx: _inplace_a(ctx),
    "@inplace-r": lambda ctx: _inplace_r(ctx),
    "@inplace-g": lambda ctx: _inplace_g(ctx),
    "@home-j": lambda ctx: "z 0",
    "@stream-pv": lambda ctx: "pv 200",
    "@stream-pf": lambda ctx: "pf 200",
    "@stream-tm": lambda ctx: "tm 200",
}
STREAM_STOP = {"@stream-pv": "pv 0", "@stream-pf": "pf 0", "@stream-tm": "tm 0"}


def _fmt(vals: Sequence[float]) -> str:
    return " ".join("%.1f" % v for v in vals)


def _inplace_m(ctx: Ctx) -> Optional[str]:
    s = ctx.servo_deg()
    return None if not s else "m %s 15" % _fmt(s)


def _inplace_a(ctx: Ctx) -> Optional[str]:
    s = ctx.servo_deg()
    return None if not s else "a 0 %.1f" % s[0]


def _inplace_r(ctx: Ctx) -> Optional[str]:
    r = ctx.raw()
    return None if not r else "r 0 %d" % int(round(r[0]))


def _inplace_g(ctx: Ctx) -> Optional[str]:
    p = ctx.pose()
    if not p:
        return None
    return "g %.1f %.1f %.1f %.1f" % (Ctx._pick(p, "x"), Ctx._pick(p, "y"),
                                      Ctx._pick(p, "z"),
                                      Ctx._pick(p, "pitch_deg", "pitch"))


def run_text(report: Report, ctx: Ctx, backend: str, raw: Optional[RawSerial],
             case: Case, timeout: float) -> Result:
    if case.level in ("write", "forbidden"):
        return report.add(Result(key=case.key, kind="text", level=case.level,
                                 sent="", status="LISTED", detail=case.note))
    line = case.line or ""
    if line.startswith("@"):
        fn = INPLACE_TEXT.get(line)
        if fn is None:
            return report.add(Result(key=case.key, kind="text", level=case.level,
                                     sent="", status="SKIP", detail="无安全参数构造器"))
        try:
            got = fn(ctx)
        except Exception as e:                                      # noqa: BLE001
            return report.add(Result(key=case.key, kind="text", level=case.level,
                                     sent="", status="FAIL", detail="参数构造失败: %s" % e))
        if not got:
            return report.add(Result(key=case.key, kind="text", level=case.level,
                                     sent="", status="SKIP",
                                     detail="读不到当前值（臂离线/未校准）⇒ 不猜参数"))
        line = got
    if not line:
        return report.add(Result(key=case.key, kind="text", level=case.level,
                                 sent="", status="SKIP", detail=case.note))

    r = Result(key=case.key, kind="text", level=case.level, sent=line)
    t0 = time.time()
    try:
        if backend == "serial" and raw is not None:
            out, gated = raw.exec_line(line)
            truncated = False
        else:
            if backend == "sim":
                # sim 的解释器只覆盖自检用到的少数命令
                if case.key not in TEXT_GATED_KEYS and case.key not in ("s", "h", "zi", "lk", "f"):
                    return report.add(Result(key=case.key, kind="text", level=case.level,
                                             sent=line, status="SKIP",
                                             detail="Python 模拟器无完整文本解释器 ⇒ 需真机"))
            out, gated, truncated = ctx.text_via_link(line)
        r.ms = (time.time() - t0) * 1000.0
        r.bytes_in = len(out.encode("utf-8", "replace"))
        status, detail = classify_text(out, gated)
        if case.key in TEXT_GATED_KEYS and status == "PASS":
            status, detail = "PASS*", "未被闸门拦（本构建可能允许）：" + detail
        if truncated:
            detail += "  [输出被 2048B 截断]"
        r.status, r.detail = status, detail
        if case.note and status in ("PASS", "PASS*"):
            r.detail += "  | " + case.note
    except Exception as e:                                          # noqa: BLE001
        r.status, r.detail = "FAIL", str(e)[:160]

    # 流类：立刻关掉
    stop = STREAM_STOP.get(line if line.startswith("@") else case.line or "")
    if stop and r.status in ("PASS", "PASS*"):
        try:
            if backend == "serial" and raw is not None:
                raw.exec_line(stop)
            else:
                ctx.text_via_link(stop)
            r.detail += "  → 已发 %r 停流" % stop
        except Exception as e:                                      # noqa: BLE001
            r.detail += "  ⚠ 停流失败: %s" % e
    return report.add(r)


# ==========================================================================
# 报告
# ==========================================================================

ICON = {"PASS": "✓", "PASS*": "✓", "GATED": "⊘", "SKIP": "-",
        "FAIL": "✗", "TIMEOUT": "✗", "LISTED": "·"}


def render(report: Report, verbose: bool) -> str:
    L: List[str] = []
    L.append("=" * 96)
    L.append("全指令逐条测试 —— 只测不改（write/forbidden 档不下发）")
    L.append("=" * 96)
    L.append("后端      : %s" % report.backend)
    L.append("目标      : %s" % report.target)
    L.append("安全档    : %s" % report.level)
    L.append("开始时间  : %s" % report.started)
    if report.config_total:
        cov = report.text_registered * 100.0 / report.config_total
        L.append("文本覆盖率: %d/%d 条登记项已分级（%.0f%%）%s"
                 % (report.text_registered, report.config_total, cov,
                    "" if not report.unclassified
                    else "  未分级: " + ", ".join(report.unclassified)))
    for line in report.preflight:
        L.append("预检      : %s" % line)
    L.append("")

    L.append("%-4s %-22s %-9s %-9s %s" % ("", "命令", "档位", "结果", "详情"))
    L.append("-" * 96)
    for r in report.results:
        if not verbose and r.status == "LISTED":
            continue
        L.append("%-4s %-22s %-9s %-9s %s" % (ICON.get(r.status, "?"), r.key,
                                              r.level, r.status, r.detail))
    L.append("-" * 96)

    s = report.summary()
    total = len(report.results)
    listed = s.get("LISTED", 0)
    ran = total - listed
    L.append("")
    L.append("汇总：共登记 %d 条 | 本次下发 %d 条 | 未下发（按「只测不改」要求跳过）%d 条"
             % (total, ran, listed))
    if ran == 0 and total:
        L.append("    （本次是 --plan 预览，一条都没下发）")
    for k in ("PASS", "PASS*", "GATED", "SKIP", "FAIL", "TIMEOUT", "LISTED"):
        if s.get(k):
            L.append("    %-9s %d" % (k, s[k]))
    bad = s.get("FAIL", 0) + s.get("TIMEOUT", 0)
    L.append("")
    L.append("结论：%s" % ("✗ 有 %d 条失败/超时，见上表" % bad if bad
                          else "✓ 已下发的命令全部有正确响应"))
    if listed:
        L.append("未下发清单（不执行是**设计**，不是漏测）：")
        for r in report.results:
            if r.status == "LISTED":
                L.append("    %-22s %-9s %s" % (r.key, r.level, r.detail))
    return "\n".join(L)


def dump_json(report: Report, path: str) -> None:
    d = {
        "backend": report.backend, "target": report.target, "level": report.level,
        "started": report.started, "preflight": report.preflight,
        "summary": report.summary(),
        "results": [asdict(r) for r in report.results],
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(d, f, ensure_ascii=False, indent=2)


# ==========================================================================
# main
# ==========================================================================

def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="全指令逐条测试（默认只读，绝不改参数）",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--sim", action="store_true", help="离线内存假臂（默认）")
    g.add_argument("--host", metavar="IP", help="真机 UDP，例 192.168.7.120")
    g.add_argument("--serial", metavar="COM", help="真机串口裸文本，例 COM11")
    ap.add_argument("--port", type=int, default=5000, help="UDP 端口（默认 5000）")
    ap.add_argument("--baud", type=int, default=115200, help="串口波特率")
    ap.add_argument("--level", choices=("read", "motion"), default="read",
                    help="执行档位；write/forbidden 永不下发")
    ap.add_argument("--i-know-it-moves", action="store_true",
                    help="motion 档必须显式加这个才跑（会发运动指令）")
    ap.add_argument("--only", metavar="KEY,...", help="只测这些 key（逗号分隔，支持 0x01 形式）")
    ap.add_argument("--skip", metavar="KEY,...", help="跳过这些 key")
    ap.add_argument("--include-slow", action="store_true", help="含慢命令（Z 全 ID 扫描）")
    ap.add_argument("--timeout", type=float, default=0.4, help="单条超时秒（默认 0.4）")
    ap.add_argument("--plan", action="store_true", help="只列清单，不下发")
    ap.add_argument("--verbose", action="store_true", help="连未下发的行也逐条打印")
    ap.add_argument("--md", metavar="FILE", help="同时写 Markdown 报告")
    ap.add_argument("--json", metavar="FILE", help="同时写 JSON 报告")
    return ap.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    a = parse_args(argv)
    started = time.strftime("%Y-%m-%d %H:%M:%S")

    backend = "serial" if a.serial else ("net" if a.host else "sim")
    target = a.serial or (a.host or "sim(内存假臂)")

    report = Report(backend=backend, target=target, level=a.level, started=started)

    # ---- 连接 ----
    client: Optional[ArmClient] = None
    raw: Optional[RawSerial] = None
    if not a.plan:
        if backend == "serial":
            try:
                raw = RawSerial(a.serial, a.baud)
                report.preflight.append("串口 %s @%d 已打开（裸文本模式）" % (a.serial, a.baud))
            except Exception as e:                                  # noqa: BLE001
                print("✗ 串口打开失败: %s" % e)
                return 3
            # 串口同时也能跑二进制帧 ⇒ 也给 client 一份
            client = ArmClient(make_transport("serial", port=a.serial, baudrate=a.baud))
        else:
            client = ArmClient(make_transport(
                "sim" if backend == "sim" else "net",
                **({"ip": a.host, "port": a.port} if backend == "net" else {})))
        try:
            client.open()
        except Exception as e:                                      # noqa: BLE001
            print("✗ 连接失败: %s" % e)
            return 3

    # ---- 预检：拿 fw-tag / 状态位（让报告能自我说明"测的是哪一版"）----
    ctx = Ctx(client) if client else None                            # type: ignore[arg-type]
    if client and not a.plan:
        try:
            rep = client.request(P.CMD_GET_INFO, b"", timeout=max(a.timeout, 0.6),
                                 label="预检 GET_INFO")
            if rep.ok:
                info = P.decode_info(rep.payload)
                report.preflight.append("固件: %s" % getattr(info, "fw_tag", "?"))
            else:
                report.preflight.append("GET_INFO 无响应（%s）" % rep.desc)
        except Exception as e:                                      # noqa: BLE001
            report.preflight.append("GET_INFO 异常: %s" % e)
        try:
            rep = client.request(P.CMD_GET_FLAGS, b"", timeout=max(a.timeout, 0.6),
                                 label="预检 GET_FLAGS")
            if rep.ok:
                fl = P.decode_flags(rep.payload)
                report.preflight.append("状态位: %s" % fl)
        except Exception as e:                                      # noqa: BLE001
            report.preflight.append("GET_FLAGS 异常: %s" % e)

    # ---- 清洗 --only / --skip ----
    only = set()
    if a.only:
        for k in a.only.split(","):
            k = k.strip()
            if not k:
                continue
            if k.lower().startswith("0x"):
                only.add(str(int(k, 16)))
            else:
                only.add(k)
    skip = {k.strip() for k in (a.skip or "").split(",") if k.strip()}

    def wanted(key: str, cmd: Optional[int] = None) -> bool:
        if key in skip or (cmd is not None and str(cmd) in skip):
            return False
        if not only:
            return True
        return key in only or (cmd is not None and
                               (str(cmd) in only or ("0x%02X" % cmd) in only or ("0x%02x" % cmd) in only))

    max_level = 1 if a.level == "motion" else 0
    motion_ok = a.level == "motion" and a.i_know_it_moves

    # ---- 二进制 38 条 ----
    for cmd, key, level, fn, expect, note in BIN_SPECS:
        if not wanted(key, cmd):
            continue
        if level == "motion" and not (motion_ok and max_level >= 1):
            report.add(Result(key=key, kind="bin", level=level, sent="", status="LISTED",
                              detail=note + "（需 --level motion --i-know-it-moves）"))
            continue
        if a.plan:
            report.add(Result(key=key, kind="bin", level=level,
                              sent=("0x%02X" % cmd), status="LISTED", detail=note))
            continue
        run_bin(report, client, ctx, cmd, key, level, fn, expect, note, a.timeout)  # type: ignore[arg-type]

    # ---- 文本 114 条 ----
    _names = load_config_names()
    report.config_total = len(_names)
    report.unclassified = [n for n in _names if n not in TEXT_SPECS]
    report.text_registered = report.config_total - len(report.unclassified)

    for case in build_text_cases(a.include_slow):
        if not wanted(case.key):
            continue
        if case.key == "Z" and not a.include_slow:
            report.add(Result(key=case.key, kind="text", level=case.level, sent="Z",
                              status="LISTED", detail=case.note))
            continue
        if case.level == "motion" and not (motion_ok and max_level >= 1):
            report.add(Result(key=case.key, kind="text", level=case.level, sent="",
                              status="LISTED",
                              detail=case.note + "（需 --level motion --i-know-it-moves）"))
            continue
        if a.plan:
            report.add(Result(key=case.key, kind="text", level=case.level,
                              sent=case.line or "", status="LISTED", detail=case.note))
            continue
        run_text(report, ctx, backend, raw, case, a.timeout)         # type: ignore[arg-type]

    # ---- 收尾 ----
    if client and not a.plan:
        try:
            client.close()
        except Exception:                                           # noqa: BLE001
            pass
    if raw is not None:
        raw.close()

    text = render(report, a.verbose or a.plan)
    print(text)
    if a.md:
        os.makedirs(os.path.dirname(os.path.abspath(a.md)), exist_ok=True)
        with open(a.md, "w", encoding="utf-8") as f:
            f.write("```\n%s\n```\n" % text)
        print("\n（Markdown 报告已写入 %s）" % a.md)
    if a.json:
        os.makedirs(os.path.dirname(os.path.abspath(a.json)), exist_ok=True)
        dump_json(report, a.json)
        print("（JSON 报告已写入 %s）" % a.json)

    s = report.summary()
    return 0 if (s.get("FAIL", 0) + s.get("TIMEOUT", 0)) == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
