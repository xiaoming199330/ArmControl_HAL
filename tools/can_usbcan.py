#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
can_usbcan.py — 厂商私有 USB-CAN 适配器（ZLG `VCI_*` 兼容 API）的 ctypes 后端

为什么需要这个文件
------------------
有些国产 USB-CAN（如本机装的 LYS 系列，DLL = `lyscan.dll`）不提供 python-can
内置接口，只给一个私有 DLL。好在它的导出表是 **ZLG USBCAN 那套标准 VCI API**：

    VCI_OpenDevice / VCI_CloseDevice / VCI_InitCAN / VCI_StartCAN
    VCI_Transmit / VCI_Receive / VCI_ClearBuffer / VCI_ReadBoardInfo / VCI_GetReceiveNum

（本机 `lyscan.dll` 的 100 个导出里逐个核实过，签名即上表；另有 LCAN_* 是本厂商自有扩展。）

⚠ **位数是第一杀手**：`lyscan.dll` 是 **x86（32 位）**，必须用 **32 位 Python** 运行本文件。
   64 位 Python 加载会报 "不是有效的 Win32 应用程序" —— 那跟代码对不对无关。

用法
----
    python tools/can_usbcan.py --scan              # 暴力扫 DevType, 认卡(插着卡才能扫到)
    python tools/can_usbcan.py --selftest          # 只验证 DLL 能加载 + 导出齐全

    # 从 can_host.py 使用（同一套协议代码, 只换承载）:
    python tools/can_host.py --backend usbcan --dll "C:\\Program Files (x86)\\CanAst\\lyscan.dll" \
        --devtype 4 --smoke

DevType 取值各家不同（ZLG 原厂: 3=USBCAN-I, 4=USBCAN-II），用 `--scan` 问出来即可。
"""

import argparse
import ctypes
import os
import sys
import time
from ctypes import wintypes

# 默认 DLL 搜索路径（按"最可能"排序）
DEFAULT_DLLS = [
    r"C:\Program Files (x86)\CanAst\lyscan.dll",
    r"C:\Program Files (x86)\LysUsbCanDriver\lyscan.dll",
    r"C:\Program Files\ZLG\lyscan.dll",
    "lyscan.dll",
]

# SJA1000 波特率寄存器表（ZLG/LAWICEL 通用的 16MHz 表）
# 500k = (0x00, 0x1C) —— 与机械臂侧默认 500 kbit/s 对上
BAUD_TABLE = {
    1000: (0x00, 0x14),
    800:  (0x00, 0x16),
    500:  (0x00, 0x1C),
    250:  (0x01, 0x1C),
    125:  (0x03, 0x1C),
    100:  (0x04, 0x1C),
    50:   (0x09, 0x1C),
    20:   (0x18, 0x1C),
    10:   (0x31, 0x1C),
}


# ============================================================
# 结构体（与 ZLG USBCAN 头文件一致，字段顺序/宽度不能改）
# ============================================================
class VCI_INIT_CONFIG(ctypes.Structure):
    _fields_ = [("AccCode", wintypes.DWORD),
                ("AccMask", wintypes.DWORD),
                ("Reserved", wintypes.DWORD),
                ("Filter", ctypes.c_ubyte),
                ("Timing0", ctypes.c_ubyte),
                ("Timing1", ctypes.c_ubyte),
                ("Mode", ctypes.c_ubyte)]          # 0=正常 1=只听


class VCI_CAN_OBJ(ctypes.Structure):
    _fields_ = [("ID", wintypes.UINT),
                ("TimeStamp", wintypes.UINT),
                ("TimeFlag", ctypes.c_ubyte),
                ("SendType", ctypes.c_ubyte),      # 0=正常发送 1=单次 2=自发自收
                ("RemoteFlag", ctypes.c_ubyte),    # 0=数据帧
                ("ExternFlag", ctypes.c_ubyte),    # 0=标准帧
                ("DataLen", ctypes.c_ubyte),
                ("Data", ctypes.c_ubyte * 8),
                ("Reserved", ctypes.c_ubyte * 3)]


class VCI_BOARD_INFO(ctypes.Structure):
    _fields_ = [("hw_Version", ctypes.c_ushort),
                ("fw_Version", ctypes.c_ushort),
                ("dr_Version", ctypes.c_ushort),
                ("in_Version", ctypes.c_ushort),
                ("irq_Num", ctypes.c_ushort),
                ("can_Num", ctypes.c_ubyte),
                ("str_Serial_Num", ctypes.c_char * 20),
                ("str_hw_Type", ctypes.c_char * 40),
                ("Reserved", ctypes.c_ubyte * 4)]


REQUIRED_EXPORTS = ("VCI_OpenDevice", "VCI_CloseDevice", "VCI_InitCAN", "VCI_StartCAN",
                    "VCI_Transmit", "VCI_Receive", "VCI_ClearBuffer")


def find_dll(explicit=None):
    for p in ([explicit] if explicit else []) + DEFAULT_DLLS:
        if p and os.path.exists(p):
            return p
    return None


def load_dll(path=None):
    """加载 DLL 并绑定签名。返回 (dll, path)。失败抛 RuntimeError（带位数提示）。"""
    p = find_dll(path)
    if not p:
        raise RuntimeError("找不到厂商 DLL，用 --dll 指定路径。\n"
                           "  常见位置: " + " / ".join(DEFAULT_DLLS))
    try:
        dll = ctypes.WinDLL(p)
    except OSError as e:
        raise RuntimeError(
            "加载 %s 失败: %s\n"
            "  ⇒ 若是 '不是有效的 Win32 应用程序'/'%%1 is not a valid Win32 application'，\n"
            "    说明 DLL 位数与本解释器不一致（本机 lyscan.dll 是 32 位）:\n"
            "    请用 32 位 Python 运行本脚本。当前解释器: %s"
            % (p, e, sys.executable))
    dll.VCI_OpenDevice.argtypes = [wintypes.DWORD, wintypes.DWORD, wintypes.DWORD]
    dll.VCI_OpenDevice.restype = wintypes.DWORD
    dll.VCI_CloseDevice.argtypes = [wintypes.DWORD, wintypes.DWORD]
    dll.VCI_CloseDevice.restype = wintypes.DWORD
    dll.VCI_InitCAN.argtypes = [wintypes.DWORD, wintypes.DWORD, wintypes.DWORD,
                                ctypes.POINTER(VCI_INIT_CONFIG)]
    dll.VCI_InitCAN.restype = wintypes.DWORD
    dll.VCI_StartCAN.argtypes = [wintypes.DWORD, wintypes.DWORD, wintypes.DWORD]
    dll.VCI_StartCAN.restype = wintypes.DWORD
    dll.VCI_Transmit.argtypes = [wintypes.DWORD, wintypes.DWORD, wintypes.DWORD,
                                 ctypes.POINTER(VCI_CAN_OBJ), wintypes.DWORD]
    dll.VCI_Transmit.restype = wintypes.DWORD
    dll.VCI_Receive.argtypes = [wintypes.DWORD, wintypes.DWORD, wintypes.DWORD,
                                ctypes.POINTER(VCI_CAN_OBJ), wintypes.DWORD,
                                ctypes.c_int]
    dll.VCI_Receive.restype = wintypes.DWORD
    dll.VCI_ClearBuffer.argtypes = [wintypes.DWORD, wintypes.DWORD, wintypes.DWORD]
    dll.VCI_ClearBuffer.restype = wintypes.DWORD
    dll.VCI_ReadBoardInfo.argtypes = [wintypes.DWORD, wintypes.DWORD,
                                      ctypes.POINTER(VCI_BOARD_INFO)]
    dll.VCI_ReadBoardInfo.restype = wintypes.DWORD
    return dll, p


# ============================================================
# 总线对象：接口形状与 can.Bus 一致（send(msg) / recv(timeout) / shutdown()），
# 所以 can_host.py 的协议层一行都不用改。
# ============================================================
class _Msg:
    __slots__ = ("arbitration_id", "data", "is_extended_id")

    def __init__(self, arbitration_id, data):
        self.arbitration_id = arbitration_id
        self.data = bytes(data)
        self.is_extended_id = False


class UsbCanBus:
    def __init__(self, dll_path=None, devtype=4, devindex=0, canindex=0, kbps=500):
        if kbps not in BAUD_TABLE:
            raise ValueError("波特率只支持 %s kbit/s" % sorted(BAUD_TABLE))
        self.dll, self.path = load_dll(dll_path)
        self.devtype, self.devindex, self.canindex = devtype, devindex, canindex
        if not self.dll.VCI_OpenDevice(self.devtype, self.devindex, 0):
            raise RuntimeError(
                "VCI_OpenDevice(DevType=%d) 返回 0：卡不在 / DevType 不对 / 卡被其它程序占用。\n"
                "  ⇒ 先跑 `python tools/can_usbcan.py --scan` 问出正确的 DevType；\n"
                "  ⇒ 关掉厂商 GUI（CanAst.exe）—— 同一块卡只能被一个进程打开。"
                % self.devtype)
        cfg = VCI_INIT_CONFIG(0x00000000, 0xFFFFFFFF, 0, 0,
                              BAUD_TABLE[kbps][0], BAUD_TABLE[kbps][1], 0)
        if not self.dll.VCI_InitCAN(self.devtype, self.devindex, self.canindex,
                                    ctypes.byref(cfg)):
            self.shutdown()
            raise RuntimeError("VCI_InitCAN 失败（波特率 %d kbit/s 不受支持?）" % kbps)
        if not self.dll.VCI_StartCAN(self.devtype, self.devindex, self.canindex):
            self.shutdown()
            raise RuntimeError("VCI_StartCAN 失败")
        self.dll.VCI_ClearBuffer(self.devtype, self.devindex, self.canindex)
        self._kbps = kbps
        self._obj = VCI_CAN_OBJ()
        self._open = True

    # ---- 与 can.Bus 同形 ----
    def send(self, msg):
        o = VCI_CAN_OBJ()
        o.ID = int(msg.arbitration_id)
        o.SendType = 0            # 正常发送
        o.RemoteFlag = 0
        o.ExternFlag = 1 if getattr(msg, "is_extended_id", False) else 0
        o.DataLen = len(msg.data)
        for i, b in enumerate(msg.data[:8]):
            o.Data[i] = b
        n = self.dll.VCI_Transmit(self.devtype, self.devindex, self.canindex,
                                  ctypes.byref(o), 1)
        if not n:
            raise RuntimeError("VCI_Transmit 失败（发送邮箱满 / 总线无 ACK 时也可能返回 0）")

    def recv(self, timeout=0.0):
        """timeout 秒内收一帧。返回 duck-typed msg 或 None。"""
        wait_ms = int(max(0.0, timeout) * 1000)
        n = self.dll.VCI_Receive(self.devtype, self.devindex, self.canindex,
                                 ctypes.byref(self._obj), 1, wait_ms)
        if not n:
            return None
        o = self._obj
        return _Msg(o.ID, bytes(bytearray(o.Data)[:o.DataLen]))

    def shutdown(self):
        if getattr(self, "_open", False):
            try:
                self.dll.VCI_CloseDevice(self.devtype, self.devindex)
            except Exception:                       # noqa: BLE001
                pass
            self._open = False


# ============================================================
# 命令行
# ============================================================
def cmd_selftest(dll_path=None):
    print("== 1) 加载厂商 DLL ==")
    try:
        dll, p = load_dll(dll_path)
    except RuntimeError as e:
        print("  FAIL\n  " + str(e).replace("\n", "\n  "))
        return 1
    print("  PASS 已加载: %s" % p)
    print("== 2) 必需导出 ==")
    miss = []
    for name in REQUIRED_EXPORTS:
        ok = hasattr(dll, name)
        print("  %s %s" % ("PASS" if ok else "FAIL", name))
        if not ok:
            miss.append(name)
    print("\n%s" % ("全部通过 ✅（DLL 可用，插卡后跑 --scan 认 DevType）"
                    if not miss else "有缺失导出 ❌ %s" % miss))
    return 1 if miss else 0


def cmd_scan(dll_path=None, kbps=500, limit=41):
    try:
        dll, p = load_dll(dll_path)
    except RuntimeError as e:
        print(str(e))
        return 1
    print("扫描 %s 的 DevType 0..%d（插着卡才会命中）…\n" % (p, limit - 1))
    hit = []
    for t in range(limit):
        if not dll.VCI_OpenDevice(t, 0, 0):
            continue
        info = VCI_BOARD_INFO()
        hw = fw = dr = ""
        if dll.VCI_ReadBoardInfo(t, 0, ctypes.byref(info)):
            hw = info.str_hw_Type.decode("latin1", "ignore").strip("\0 ")
            fw = "fw=%d.%d" % (info.fw_Version >> 8, info.fw_Version & 0xFF)
            dr = "hw=v%d.%d" % (info.hw_Version >> 8, info.hw_Version & 0xFF)
            sn = info.str_Serial_Num.decode("latin1", "ignore").strip("\0 ")
        else:
            sn = ""
        dll.VCI_CloseDevice(t, 0)
        hit.append(t)
        print("  ✓ DevType=%-3d  通道数=%d  %s  %s  %s  SN=%s"
              % (t, info.can_Num, hw or "(无型号串)", dr or "", fw or "", sn or "-"))
    print()
    if hit:
        print("命中 %d 个。用第一个跑冒烟：" % len(hit))
        print('  python tools/can_host.py --backend usbcan --dll "%s" --devtype %d --smoke'
              % (p, hit[0]))
    else:
        print("一个都没命中。排查：")
        print("  ① 卡插了吗（USB 枚举里要能看到设备；设备管理器看有无未知设备）")
        print("  ② 位数对不对（本 DLL 是 32 位 ⇒ 必须用 32 位 Python）")
        print("  ③ 厂商 GUI/上位机是不是还占着卡（同一块卡只能一个进程打开）")
        print("  ④ 驱动装了吗（LysUsbCanDriver\\usbcanAllInOne.exe）")
    return 0 if hit else 1


def main():
    ap = argparse.ArgumentParser(description="厂商私有 USB-CAN (ZLG VCI 兼容) ctypes 后端")
    ap.add_argument("--dll", help="厂商 DLL 路径（默认自动搜 lyscan.dll）")
    ap.add_argument("--selftest", action="store_true", help="只验证 DLL 加载与导出")
    ap.add_argument("--scan", action="store_true", help="扫 DevType 认卡")
    a = ap.parse_args()
    if a.selftest:
        return cmd_selftest(a.dll)
    if a.scan:
        return cmd_scan(a.dll)
    ap.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())
