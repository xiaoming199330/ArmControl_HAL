#!/usr/bin/env python3
"""
六轴机械臂 CAN 帧生成器
生成 PCAN-View 可直接使用的 CAN 帧数据。

用法:
  python can_frame_generator.py

会输出各种常用命令的 CAN 帧序列，直接复制到 PCAN-View 的 Transmit 窗口发送。
"""
import struct

# ============ 协议常量 ============
CAN_ID_CMD = 0x101   # 上位机 -> 机械臂
CAN_ID_RSP = 0x181   # 机械臂 -> 上位机

CMD_PING        = 0x01
CMD_GET_STATUS  = 0x02
CMD_MOVE_JOINTS = 0x10
CMD_STOP        = 0x11
CMD_SET_TORQUE  = 0x12
CMD_MOVE_CART   = 0x20

# ============ CRC-16/CCITT-FALSE ============
def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

# ============ 定点编码 ============
def enc_deg10(deg: float) -> int:
    """角度 -> int16, 0.1deg"""
    return int(round(deg * 10)) & 0xFFFF

def enc_mm10(mm: float) -> int:
    """位置 -> int16, 0.1mm"""
    return int(round(mm * 10)) & 0xFFFF

def enc_vel10(vel: float) -> int:
    """速度 -> uint16, 0.1deg/s"""
    return int(round(vel * 10)) & 0xFFFF

# ============ CAN 分包 ============
def build_can_frames(seq: int, cmd: int, payload: bytes) -> list:
    """
    将逻辑帧分包为 CAN 2.0A 帧。
    返回 [(can_id, dlc, data_bytes), ...]
    """
    # 逻辑流: SEQ + CMD + LEN + PAYLOAD + CRC_LO + CRC_HI
    stream = bytes([seq, cmd, len(payload)]) + payload
    crc = crc16_ccitt_false(stream)
    stream += struct.pack('<H', crc)  # CRC 小端

    # 分包计算
    tail_total = len(payload) + 2  # payload + CRC(2)
    rest = max(0, tail_total - 4)
    frames_count = 1 + (rest + 6) // 7

    frames = []
    d = bytearray(8)

    # 首帧
    d[0] = 0x80 | frames_count
    d[1] = seq
    d[2] = cmd
    d[3] = len(payload)
    first_n = min(4, tail_total)
    for i in range(first_n):
        d[4 + i] = stream[3 + i]
    frames.append((CAN_ID_CMD, 4 + first_n, bytes(d[:4 + first_n])))

    # 续帧
    off = first_n
    for f in range(1, frames_count):
        n = min(7, tail_total - off)
        d[0] = f
        for i in range(n):
            d[1 + i] = stream[3 + off + i]
        frames.append((CAN_ID_CMD, 1 + n, bytes(d[:1 + n])))
        off += n

    return frames

# ============ 命令构建 ============
def cmd_ping(seq=1):
    return build_can_frames(seq, CMD_PING, b'')

def cmd_get_status(seq=2):
    return build_can_frames(seq, CMD_GET_STATUS, b'')

def cmd_stop(seq=3):
    return build_can_frames(seq, CMD_STOP, b'')

def cmd_set_torque(joint: int, enable: int, seq=4):
    """joint: 0~5 或 0xFF=全部, enable: 0=关 1=开"""
    payload = bytes([joint & 0xFF, enable & 0xFF])
    return build_can_frames(seq, CMD_SET_TORQUE, payload)

def cmd_move_joints(joints_deg: list, v_deg_s=30.0, a_deg_s2=60.0, seq=5):
    """
    joints_deg: [J1,J2,J3,J4,J5,J6] 角度(度)
    v_deg_s: 速度 deg/s
    a_deg_s2: 加速度 deg/s^2
    """
    payload = b''
    for deg in joints_deg:
        payload += struct.pack('<H', enc_deg10(deg))
    payload += struct.pack('<H', enc_vel10(v_deg_s))
    payload += struct.pack('<H', enc_vel10(a_deg_s2))
    return build_can_frames(seq, CMD_MOVE_JOINTS, payload)

def cmd_move_cart(x_mm, y_mm, z_mm, pitch_deg=90.0, v_deg_s=30.0, a_deg_s2=60.0, seq=6):
    payload = struct.pack('<h', enc_mm10(x_mm))
    payload += struct.pack('<h', enc_mm10(y_mm))
    payload += struct.pack('<h', enc_mm10(z_mm))
    payload += struct.pack('<h', enc_deg10(pitch_deg))
    payload += struct.pack('<H', enc_vel10(v_deg_s))
    payload += struct.pack('<H', enc_vel10(a_deg_s2))
    return build_can_frames(seq, CMD_MOVE_CART, payload)

# ============ 格式化输出 ============
def format_frames(frames: list, title: str):
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}")
    for i, (can_id, dlc, data) in enumerate(frames):
        frame_type = "首帧" if i == 0 else f"续帧{i}"
        hex_str = ' '.join(f'{b:02X}' for b in data)
        print(f"  帧{i+1} ({frame_type}):")
        print(f"    CAN-ID: {can_id:03X}h")
        print(f"    Type:   Standard")
        print(f"    DLC:    {dlc}")
        print(f"    Data:   {hex_str}")
        print()

# ============ 主程序 ============
if __name__ == '__main__':
    print("=" * 60)
    print("  六轴机械臂 CAN 帧生成器")
    print("  固件: APROTO v1, CAN1 PA11/PA12, 500kbit/s")
    print("  CAN ID: 命令=0x101, 响应=0x181")
    print("=" * 60)

    # 1. PING
    format_frames(cmd_ping(seq=1), "PING (测试通信)")

    # 2. 使能全部关节扭矩
    format_frames(cmd_set_torque(0xFF, 1, seq=2), "SET_TORQUE 全部关节使能")

    # 3. 回到垂直位姿 (所有关节150°)
    format_frames(cmd_move_joints([150,150,150,150,150,150], 30, 60, seq=3),
                  "MOVE_JOINTS 回垂直位姿 (J1-J6=150°, v=30, a=60)")

    # 4. 关节联动示例 (J1转到100°, 其他保持150°)
    format_frames(cmd_move_joints([100,150,150,150,150,150], 30, 60, seq=4),
                  "MOVE_JOINTS J1转到100° (v=30, a=60)")

    # 5. 停止
    format_frames(cmd_stop(seq=5), "STOP 停止运动")

    # 6. 读状态
    format_frames(cmd_get_status(seq=6), "GET_STATUS 读状态")

    # 7. 关闭全部扭矩
    format_frames(cmd_set_torque(0xFF, 0, seq=7), "SET_TORQUE 全部关节关闭")

    # 8. 自定义命令
    print(f"\n{'='*60}")
    print("  自定义命令生成")
    print(f"{'='*60}")
    print("  修改脚本底部代码来生成其他角度组合的帧。")
    print("  例如: cmd_move_joints([120,140,160,150,150,150], 50, 100)")
    print()

    # 示例: 6关节联动到不同角度
    format_frames(cmd_move_joints([120,140,160,130,150,150], 50, 100, seq=8),
                  "MOVE_JOINTS 6关节联动 (J1=120 J2=140 J3=160 J4=130 J5=150 J6=150, v=50, a=100)")

    print("\n" + "=" * 60)
    print("  PCAN-View 使用说明:")
    print("  1. 每个命令可能由1~3个CAN帧组成，必须按顺序快速发送")
    print("  2. 首帧DLC通常为8，续帧DLC通常为8")
    print("  3. 帧间隔不要超过200ms，否则固件会丢弃重组缓冲")
    print("  4. 发送完成后观察是否收到0x181的ACK/响应")
    print("  5. ACK: 0x81 7F <原CMD> 00 ... (err=0表示成功)")
    print("  6. NACK: 0x81 7E <原CMD> <错误码> ... (失败)")
    print("=" * 60)
