// ===========================================================================
//  ArmLink.cs — 六轴机械臂 网口协议 · C# 最小客户端（单文件，无第三方依赖）
//
//  给 WinForm / WPF / .NET 上位机用。粘进工程即可，或配合同目录 ArmLinkDemo.csproj
//  直接 `dotnet run` 跑一遍冒烟测试。
//
//  链路：UDP → <臂的IP>:5000
//    一个数据报 = 一个逻辑包（无帧头、无分包、无重传）
//    回包发往请求的源端口 ⇒ 本端不必绑固定端口
//
//  报文（3+LEN+2 字节）：
//    [SEQ:1][CMD:1][LEN:1][PAYLOAD:LEN][CRC_LO:1][CRC_HI:1]
//    ⚠ LEN 是 1 字节（0~64），不是 ushort
//    ⚠ CRC = CRC-16/CCITT-FALSE(poly=0x1021, init=0xFFFF, 非反射, 无最终异或)
//      范围 = SEQ+CMD+LEN+PAYLOAD（前 3+LEN 字节），**小端**存
//      自检：Crc16("123456789") == 0x29B1   ← 先过这一条再往下写
//    ⚠ 整包长度必须恰好 3+LEN+2，否则固件**静默丢弃**（只表现为超时）
//    ⚠ CMD ≥ 0x80 的包固件不回（那是回包号段）
//
//  ⚠⚠ 线程模型：UDP 会乱序，且同一个 socket 上并发请求会串包。
//      ArmLink 内部用**一把锁 + seq 匹配**把请求串行化（和 Python/JS SDK 一致）。
//      50 Hz 遥操（STREAM_JOINTS）用 SendOnly()，不等回包。
// ===========================================================================

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;

namespace ArmSdk
{
    /// <summary>错误码（NACK 的 payload[1]）</summary>
    public enum ArmErr : byte
    {
        Ok = 0, Param = 1, Crc = 2, Unreach = 3, Limit = 4,
        Estop = 5, FkVerify = 6, Read = 7, Magic = 8, Busy = 9, NotCal = 10,
    }

    public static class ArmCmd
    {
        public const byte Ping = 0x01, GetStatus = 0x02, LogSetLevel = 0x03;
        public const byte GetJointDetail = 0x04, GetLimits = 0x05, GetInfo = 0x07;
        public const byte GetFlags = 0x0A, TextExec = 0x0B, TextFetch = 0x0C;
        public const byte MoveJoints = 0x10, Stop = 0x11, SetTorque = 0x12;
        public const byte StreamJoints = 0x13, MoveJointOne = 0x14;
        public const byte SetBusMode = 0x17, ServoInit = 0x18, MoveCart = 0x20;
        public const byte FkQuery = 0x21, IkQuery = 0x22;
        public const byte SetGravity = 0x30, SetCorrection = 0x31;
        public const byte GetGravity = 0x32, GetCorrection = 0x33, GetPid = 0x36;
        public const byte ParamRead = 0x40, ParamWrite = 0x41, ParamCount = 0x42;
        public const byte ParamSave = 0x43, ParamList = 0x44;
        public const byte TeachList = 0x50, TeachRead = 0x51, TeachDelete = 0x53;
        public const byte TeachCompact = 0x54, TeachTotal = 0x55;
        public const byte CenterCalib = 0x70, CalibEnter = 0x71;
        public const byte ZeroReset = 0x72, SysReset = 0x73;
    }

    public static class ArmRsp
    {
        public const byte Ack = 0x7F, Nack = 0x7E;
        public const byte Ping = 0x81, Status = 0x82, JointDetail = 0x84;
        public const byte Limits = 0x85, Info = 0x87, Flags = 0x8A;
        public const byte TextOut = 0x8B, TextFetchOut = 0x8C;
        public const byte Fk = 0xA1, Ik = 0xA2;
        public const byte Gravity = 0xB2, Correction = 0xB3, Pid = 0xB6;
        public const byte ParamRead = 0xC0, ParamCount = 0xC2, ParamList = 0xC4;
        public const byte TeachList = 0xD0, TeachRead = 0xD1, TeachTotal = 0xD5;
    }

    public static class ArmConst
    {
        public const int MaxPayload = 64;
        public const uint Magic = 0x5A5A5A5A;
        public const short RawReadFail = -32768;   // GET_STATUS 读失败哨兵（0x8000）
        public const byte TempLost = 0xFF;         // 温度丢读哨兵 —— 不是 0！
        public const byte HwErrLost = 0xFF;        // 硬件错丢读哨兵
        public const int TextBufSz = 2048;
        public const int TextExecMax = 58;
        public const int TextChunk = 58;
        public const byte TextFlagTrunc = 0x01;
        public const byte TextFlagGate = 0x02;
        public const int ParamListMax = 6;
        public const int StatusLen = 21;           // GET_STATUS 响应负载字节数
        public const int DetailSize = 10;          // JOINT_DETAIL 每轴字节数
        public const int LimitsLen = 48;           // GET_LIMITS 响应负载字节数
        public const int InfoLen = 20;             // GET_INFO 响应负载字节数
    }

    public class ArmReply
    {
        public bool Ok;                 // 收到回包（含 ACK），且不是 NACK
        public bool Timeout;            // 根本没回包
        public byte Seq, Cmd;
        public byte[] Payload = new byte[0];
        public ArmErr? Err;             // NACK 时的错误码
        public double LatencyMs;
        public string Desc;             // 人话解释，可直接显示在 UI 上

        public override string ToString()
        {
            if (Timeout) return "超时（无回包）";
            if (Err.HasValue) return string.Format("NACK({0}) {1}", Err, Desc);
            return string.Format("0x{0:X2} {1}B {2:F1}ms", Cmd, Payload.Length, LatencyMs);
        }
    }

    /// <summary>
    /// 单轴遥测（GET_JOINT_DETAIL 每轴 10 字节）。
    /// 逐字段布局：u16 pos_raw, i16 speed, i16 load, u8 volt(0.1V), u8 temp, u8 hw_err, u8 ok
    /// ⚠ 总线是**逐字段瞬时丢读**的（温度约 20% 的拍会丢）—— 丢读**不是故障**，
    ///   下一拍可能就有了。所以 temp/hw_err 用 0xFF 当哨兵，**绝不能拿 0 兜底**。
    /// </summary>
    public class JointDetail
    {
        public int Joint;               // 轴号 0~5（J1..J6，0 基）
        public int PosRaw;              // u16，12 位位置（0~4095）
        public int Speed;               // i16，原始速度读数
        public int Load;                // i16，负载
        public double VoltV;            // u8 / 10.0
        public byte TempRaw;            // u8；255 = 丢读
        public byte HwErrRaw;           // u8；255 = 丢读
        public bool Ok;                 // false ⇒ 这一轴读失败，除哨兵外字段无意义

        public double PosDeg { get { return PosRaw * 360.0 / 4095.0; } }
        public bool TempLost { get { return TempRaw == ArmConst.TempLost; } }
        public bool HwErrLost { get { return HwErrRaw == ArmConst.HwErrLost; } }
        /// <summary>温度（℃）。丢读时返回 null —— 不要把 255 当温度显示出去。</summary>
        public int? TempC { get { return TempLost ? (int?)null : (int)TempRaw; } }
    }

    public sealed class ArmLink : IDisposable
    {
        private readonly UdpClient _udp;
        private readonly IPEndPoint _dst;
        private readonly object _lock = new object();
        private byte _seq;
        private int _timeoutMs;

        /// <param name="ip">臂的 IP，例如 "192.168.7.120"</param>
        /// <param name="port">固定 5000</param>
        public ArmLink(string ip, int port = 5000, int timeoutMs = 500)
        {
            _dst = new IPEndPoint(IPAddress.Parse(ip), port);
            _udp = new UdpClient();
            _timeoutMs = timeoutMs;
        }

        public int TimeoutMs { get { return _timeoutMs; } set { _timeoutMs = value; } }

        // ------------------------------------------------------------ CRC16
        public static ushort Crc16(byte[] d, int off, int n)
        {
            ushort crc = 0xFFFF;
            for (int i = 0; i < n; i++)
            {
                crc ^= (ushort)((ushort)d[off + i] << 8);
                for (int b = 0; b < 8; b++)
                    crc = (ushort)((crc & 0x8000) != 0
                        ? ((crc << 1) ^ 0x1021)
                        : (crc << 1));
            }
            return crc;
        }
        public static ushort Crc16(byte[] d) { return Crc16(d, 0, d.Length); }

        // ------------------------------------------------------------ 组包/解包
        public static byte[] Pack(byte seq, byte cmd, byte[] payload = null)
        {
            payload = payload ?? new byte[0];
            if (payload.Length > ArmConst.MaxPayload)
                throw new ArgumentException("payload 超长 " + payload.Length);
            var pkt = new byte[3 + payload.Length + 2];
            pkt[0] = seq; pkt[1] = cmd; pkt[2] = (byte)payload.Length;
            Buffer.BlockCopy(payload, 0, pkt, 3, payload.Length);
            ushort c = Crc16(pkt, 0, 3 + payload.Length);
            pkt[3 + payload.Length] = (byte)(c & 0xFF);
            pkt[4 + payload.Length] = (byte)(c >> 8);
            return pkt;
        }

        public static bool Unpack(byte[] rx, out byte seq, out byte cmd, out byte[] payload)
        {
            seq = cmd = 0; payload = null;
            if (rx == null || rx.Length < 5) return false;
            int len = rx[2];
            if (len > ArmConst.MaxPayload || rx.Length != 3 + len + 2) return false;
            ushort got = (ushort)(rx[3 + len] | (rx[4 + len] << 8));
            ushort want = Crc16(rx, 0, 3 + len);
            if (got != want) return false;
            seq = rx[0]; cmd = rx[1];
            payload = new byte[len];
            Buffer.BlockCopy(rx, 3, payload, 0, len);
            return true;
        }

        // ------------------------------------------------------------ 发送
        /// <summary>
        /// 发一条命令并等回包。**串行化**：同一时刻只有一条在等。
        /// 回包的 SEQ 必须与请求一致（防串包）。
        /// </summary>
        public ArmReply Request(byte cmd, byte[] payload = null, int? timeoutMs = null)
        {
            lock (_lock)
            {
                var r = new ArmReply();
                _seq = (byte)((_seq + 1) & 0xFF);
                r.Seq = _seq; r.Cmd = cmd;
                byte[] pkt = Pack(_seq, cmd, payload);
                var sw = Stopwatch.StartNew();
                try
                {
                    _udp.Client.ReceiveTimeout = timeoutMs ?? _timeoutMs;
                    _udp.Send(pkt, pkt.Length, _dst);
                    // 收到的包可能不是我们等的（迟到响应/广播）⇒ 丢掉继续等
                    var deadline = DateTime.UtcNow.AddMilliseconds(timeoutMs ?? _timeoutMs);
                    while (true)
                    {
                        int left = (int)(deadline - DateTime.UtcNow).TotalMilliseconds;
                        if (left <= 0) { r.Timeout = true; r.Desc = "无回包"; break; }
                        _udp.Client.ReceiveTimeout = left;
                        IPEndPoint from = null;
                        byte[] rx;
                        try { rx = _udp.Receive(ref from); }
                        catch (SocketException) { r.Timeout = true; r.Desc = "无回包"; break; }

                        byte s2, c2; byte[] pl2;
                        if (!Unpack(rx, out s2, out c2, out pl2)) continue;  // 坏包丢掉
                        if (s2 != _seq) continue;                            // 不是我们等的
                        r.LatencyMs = sw.Elapsed.TotalMilliseconds;
                        r.Cmd = c2;
                        r.Payload = pl2;
                        if (c2 == ArmRsp.Nack)
                        {
                            r.Ok = false;
                            r.Err = pl2.Length >= 2 ? (ArmErr)pl2[1] : ArmErr.Param;
                            r.Desc = Describe(r.Err.Value);
                        }
                        else { r.Ok = true; r.Desc = "成功"; }
                        break;
                    }
                }
                catch (Exception e)
                {
                    r.Timeout = true;
                    r.Desc = "异常：" + e.Message;
                }
                return r;
            }
        }

        /// <summary>只发不等 —— 50 Hz 流控（STREAM_JOINTS）必须用这个。</summary>
        public void SendOnly(byte cmd, byte[] payload = null)
        {
            lock (_lock)
            {
                _seq = (byte)((_seq + 1) & 0xFF);
                byte[] pkt = Pack(_seq, cmd, payload);
                _udp.Send(pkt, pkt.Length, _dst);
            }
        }

        public static string Describe(ArmErr e)
        {
            switch (e)
            {
                case ArmErr.Param: return "参数非法（量纲用错/越界/枚举无此值）";
                case ArmErr.Crc: return "CRC 或长度错";
                case ArmErr.Unreach: return "IK 无解 / 目标不可达";
                case ArmErr.Limit: return "越限（含单帧跳变 >25° 被整帧拒收）";
                case ArmErr.Estop: return "急停生效，现在不能动";
                case ArmErr.FkVerify: return "上报位姿与正解不符，安全拒动";
                case ArmErr.Read: return "舵机读失败（有轴丢读）";
                case ArmErr.Magic: return "危险命令缺/错 MAGIC";
                case ArmErr.Busy: return "轨迹/示教/回放进行中，此刻不受理";
                case ArmErr.NotCal: return "未校准";
                default: return "";
            }
        }

        // ------------------------------------------------------------ 便捷封装
        /// <summary>PING。返回 null = 不通。</summary>
        public int[] Ping()
        {
            var r = Request(ArmCmd.Ping);
            if (!r.Ok || r.Payload.Length < 2) return null;
            return new int[] { r.Payload[0], r.Payload[1] };   // proto_ver, run_id
        }

        /// <summary>
        /// 读状态。返回 6 轴**舵机度**；读失败的轴是 null（**不要当 0 用**）。
        /// ⚠ read_ok 是总闸门：任何一轴丢读，固件就不算 FK，x/y/z/pitch 全 0。
        /// </summary>
        public double?[] GetStatus(out byte flags, out bool estop, out bool traj,
                                   out bool readOk, out double[] pose)
        {
            flags = 0; estop = false; traj = false; readOk = false; pose = null;
            var r = Request(ArmCmd.GetStatus);
            if (!r.Ok || r.Payload.Length < ArmConst.StatusLen)
                return null;
            var pl = r.Payload;
            var q = new double?[6];
            for (int j = 0; j < 6; j++)
            {
                short v = ReadI16(pl, j * 2);
                q[j] = (v == ArmConst.RawReadFail) ? (double?)null : v / 10.0;
            }
            pose = new double[] { ReadI16(pl, 12) / 10.0, ReadI16(pl, 14) / 10.0,
                                  ReadI16(pl, 16) / 10.0, ReadI16(pl, 18) / 10.0 };
            flags = pl[20];
            estop = (pl[20] & 0x01) != 0;
            traj = (pl[20] & 0x02) != 0;
            readOk = (pl[20] & 0x08) != 0;
            return q;
        }

        /// <summary>当前哪些轴读失败（1 基轴号，如 {3}）。空数组 = 全好。</summary>
        public static int[] FailedJoints(double?[] q)
        {
            var l = new List<int>();
            if (q != null)
                for (int i = 0; i < q.Length; i++) if (!q[i].HasValue) l.Add(i + 1);
            return l.ToArray();
        }

        /// <summary>
        /// 绝对角运动，**舵机度**（不是逻辑角）。
        /// ⚠ v/a 是 **u16 且乘 10**：传 100 → 线上写 1000。传 0 = 用臂内默认。
        ///   写成 i16 或不乘 10 会让臂"几乎不动"或"满速冲"，而且**不报错**。
        /// </summary>
        public ArmReply MoveJoints(double[] servoDeg6, double v = 0, double a = 0)
        {
            var pl = new byte[16];
            for (int j = 0; j < 6; j++) WriteI16(pl, j * 2, Enc(servoDeg6[j] * 10.0));
            WriteU16(pl, 12, EncU(v * 10.0));
            WriteU16(pl, 14, EncU(a * 10.0));
            return Request(ArmCmd.MoveJoints, pl);
        }

        /// <summary>
        /// 笛卡尔运动（mm / deg）。⚠ 会归零 J5/J6；想保夹爪开合用 Joints 那条。
        /// </summary>
        public ArmReply MoveCart(double x, double y, double z, double pitch,
                                 double v = 0, double a = 0)
        {
            var pl = new byte[12];
            WriteI16(pl, 0, Enc(x * 10.0));
            WriteI16(pl, 2, Enc(y * 10.0));
            WriteI16(pl, 4, Enc(z * 10.0));
            WriteI16(pl, 6, Enc(pitch * 10.0));
            WriteU16(pl, 8, EncU(v * 10.0));
            WriteU16(pl, 10, EncU(a * 10.0));
            return Request(ArmCmd.MoveCart, pl);
        }

        /// <summary>
        /// 流控帧（50 Hz 遥操/回放用）。**逻辑角增量**，单位度。
        /// ⚠ 单帧单轴跳变 >25° ⇒ 整帧拒收。上游必须限速（240°/s @50Hz = 4.8°/帧）。
        /// 用 SendOnly 发，不等 ACK（等 ACK 会把频率上限压到 ~200 Hz 且更容易堵）。
        /// </summary>
        public void StreamJoints(double[] deltaDeg6)
        {
            var pl = new byte[12];
            for (int j = 0; j < 6; j++) WriteI16(pl, j * 2, Enc(deltaDeg6[j] * 10.0));
            SendOnly(ArmCmd.StreamJoints, pl);
        }

        public ArmReply Stop() { return Request(ArmCmd.Stop); }

        public ArmReply SetTorque(int joint = 0xFF, bool enable = true)
        {
            return Request(ArmCmd.SetTorque, new byte[] { (byte)joint, (byte)(enable ? 1 : 0) });
        }

        /// <summary>关节遥测。joint = -1 表示全部 6 轴（一次 2+60=62 B，一帧装得下）。</summary>
        public List<JointDetail> GetJointDetail(int joint = -1)
        {
            var r = Request(ArmCmd.GetJointDetail,
                            new byte[] { (byte)(joint < 0 ? 0xFF : joint) });
            if (!r.Ok || r.Payload.Length < 2) return null;
            int n = r.Payload[0], j0 = r.Payload[1];
            var list = new List<JointDetail>();
            var pl = r.Payload;
            for (int k = 0; k < n; k++)
            {
                int o = 2 + k * ArmConst.DetailSize;
                if (o + ArmConst.DetailSize > pl.Length) break;
                var d = new JointDetail();
                d.Joint = n > 1 ? j0 + k : j0;
                d.PosRaw = (int)((uint)pl[o] | ((uint)pl[o + 1] << 8));
                d.Speed = ReadI16(pl, o + 2);
                d.Load = ReadI16(pl, o + 4);
                d.VoltV = pl[o + 6] / 10.0;
                d.TempRaw = pl[o + 7];
                d.HwErrRaw = pl[o + 8];
                d.Ok = pl[o + 9] != 0;
                list.Add(d);
            }
            return list;
        }

        // ------------------------------------------------------------ 文本透传
        /// <summary>
        /// 网口下发**任意一条控制台文本命令**（TEXT_EXEC 0x0B）。
        /// 有了它就不必为 113 条控制台命令各写一遍载荷编码。
        /// ⚠ 自动分页：回显超过 58 B 会自动用 TEXT_FETCH 续取，直到拿全。
        /// ⚠ 分页按**字节**拼，最后统一 UTF-8 解码（中文一个字 3 字节，页边界会切开）。
        /// </summary>
        public string Text(string commandLine, out bool truncated, out bool gated,
                           int timeoutMs = 2500)
        {
            truncated = gated = false;
            // ⚠ 正文必须是 ASCII（固件 encode("ascii")，非 ASCII 直接报错），
            //   且末尾要补一个 NUL(0x00)。长度上限 58 是**字符数**。
            var textBytes = Encoding.ASCII.GetBytes(commandLine.Trim());
            if (textBytes.Length > ArmConst.TextExecMax)
                throw new ArgumentException("一条文本命令最多 "
                    + ArmConst.TextExecMax + " 字节（限 ASCII），收到 " + textBytes.Length);

            var pl = new byte[4 + textBytes.Length + 1];
            pl[0] = pl[1] = pl[2] = pl[3] = 0x5A;                   // MAGIC 小端
            Buffer.BlockCopy(textBytes, 0, pl, 4, textBytes.Length);
            pl[4 + textBytes.Length] = 0x00;                        // NUL 结尾

            var r = Request(ArmCmd.TextExec, pl, timeoutMs);
            if (!r.Ok) throw new InvalidOperationException("TEXT_EXEC 失败：" + r.ToString());

            var all = new List<byte>();
            int total, off, n;
            byte flags;
            if (!ParseTextOut(r.Payload, out total, out flags, out off, out n)) return "";
            truncated = (flags & ArmConst.TextFlagTrunc) != 0;
            gated = (flags & ArmConst.TextFlagGate) != 0;
            AppendRange(all, r.Payload, 6, n);
            off += n;

            int guard = ArmConst.TextBufSz / ArmConst.TextChunk + 2;
            while (off < total && all.Count < ArmConst.TextBufSz + 64 && guard-- > 0)
            {
                var pl2 = new byte[] { (byte)(off & 0xFF), (byte)(off >> 8),
                                       (byte)ArmConst.TextChunk };
                var r2 = Request(ArmCmd.TextFetch, pl2, timeoutMs);
                if (!r2.Ok) break;
                if (!ParseTextOut(r2.Payload, out total, out flags, out off, out n)) break;
                if (n == 0) break;
                truncated |= (flags & ArmConst.TextFlagTrunc) != 0;
                AppendRange(all, r2.Payload, 6, n);
                off += n;
            }
            return Encoding.UTF8.GetString(all.ToArray());
        }

        public string Text(string commandLine)
        {
            bool t, g; return Text(commandLine, out t, out g);
        }

        private static bool ParseTextOut(byte[] pl, out int total, out byte flags,
                                         out int off, out int n)
        {
            total = 0; flags = 0; off = 0; n = 0;
            if (pl.Length < 6) return false;
            total = pl[0] | (pl[1] << 8);
            flags = pl[2];
            off = pl[3] | (pl[4] << 8);
            n = pl[5];
            return 6 + n <= pl.Length;
        }
        private static void AppendRange(List<byte> dst, byte[] src, int off, int n)
        {
            for (int i = 0; i < n && off + i < src.Length; i++) dst.Add(src[off + i]);
        }

        // ------------------------------------------------------------ 工具
        public static short ReadI16(byte[] b, int o)
        {
            return (short)(b[o] | (b[o + 1] << 8));
        }
        public static void WriteI16(byte[] b, int o, short v)
        {
            b[o] = (byte)(v & 0xFF); b[o + 1] = (byte)((v >> 8) & 0xFF);
        }
        public static void WriteU16(byte[] b, int o, ushort v)
        {
            b[o] = (byte)(v & 0xFF); b[o + 1] = (byte)((v >> 8) & 0xFF);
        }
        /// <summary>实际值 → i16，四舍五入**远离零**（与固件一致）。</summary>
        public static short Enc(double v) { return (short)(v >= 0 ? v + 0.5 : v - 0.5); }
        /// <summary>实际值 → u16，负数先钳到 0（速度/加速度的语义）。</summary>
        public static ushort EncU(double v) { return (ushort)((v < 0 ? 0 : v) + 0.5); }

        public void Dispose() { try { _udp.Close(); } catch { } }
    }

    // =======================================================================
    //  冒烟测试：dotnet run  →  对着假臂服务跑一遍（不需要真机）
    //     终端 1:  armctl mock --port 5000 --verbose
    //     终端 2:  dotnet run -- 127.0.0.1
    // =======================================================================
    public static class ArmLinkDemo
    {
        public static int Main(string[] args)
        {
            string ip = args.Length > 0 ? args[0] : "127.0.0.1";
            int fails = 0;

            // 0) CRC 自检 —— 先过这一条，否则后面全是废的
            ushort c = ArmLink.Crc16(Encoding.ASCII.GetBytes("123456789"));
            Console.WriteLine("  [0] CRC16(\"123456789\") = 0x{0:X4}  期望 0x29B1  {1}",
                              c, c == 0x29B1 ? "OK" : "FAIL");
            if (c != 0x29B1) fails++;

            using (var arm = new ArmLink(ip, 5000, 800))
            {
                var pi = arm.Ping();
                Console.WriteLine("  [1] PING  {0}", pi == null ? "FAIL 无回包"
                                  : string.Format("proto_ver={0} run_id={1}  OK", pi[0], pi[1]));
                if (pi == null) { fails++; goto done; }

                byte flags; bool estop, traj, readOk; double[] pose;
                var q = arm.GetStatus(out flags, out estop, out traj, out readOk, out pose);
                Console.WriteLine("  [2] STATUS  {0}",
                    q == null ? "FAIL" : string.Format(
                        "{0}  flags=0x{1:X2} read_ok={2} estop={3}  pose=({4:F1},{5:F1},{6:F1},{7:F1})",
                        Join6(q), flags, readOk, estop,
                        pose[0], pose[1], pose[2], pose[3]));
                if (q == null) fails++;
                else
                {
                    var bad = ArmLink.FailedJoints(q);
                    if (bad.Length > 0)
                        Console.WriteLine("      ⚠ 读失败的轴：J{0}（不要当 0 用）",
                                          string.Join(",J", Array.ConvertAll(bad, x => x.ToString())));
                }

                var jd = arm.GetJointDetail(-1);
                Console.WriteLine("  [3] JOINT_DETAIL  {0}",
                    jd == null ? "FAIL" : jd.Count + " 轴  " +
                    string.Format("J1 raw={0} pos={1:F1}° temp={2} 丢读={3} volt={4:F1}V",
                        jd[0].PosRaw, jd[0].PosDeg,
                        jd[0].TempC.HasValue ? jd[0].TempC.Value.ToString() : "--",
                        jd[0].TempLost, jd[0].VoltV));

                try
                {
                    bool tr, ga;
                    string h = arm.Text("h", out tr, out ga);
                    Console.WriteLine("  [4] TEXT_EXEC \"h\"  {0} B, trunc={1} gated={2}",
                                      Encoding.UTF8.GetByteCount(h), tr, ga);
                    Console.WriteLine("      首行: {0}", FirstLine(h));
                }
                catch (Exception e)
                {
                    Console.WriteLine("  [4] TEXT_EXEC FAIL: {0}", e.Message);
                    fails++;
                }

                try
                {
                    bool tr, ga;
                    string out1 = arm.Text("cx", out tr, out ga);   // 抢引脚命令：应被拒
                    Console.WriteLine("  [5] 闸门测试 \"cx\"  gated={0}  输出首行: {1}",
                                      ga, FirstLine(out1));
                }
                catch (Exception e) { Console.WriteLine("  [5] FAIL {0}", e.Message); fails++; }
            }

            done:
            Console.WriteLine();
            Console.WriteLine(fails == 0 ? "  ✅ 全部通过（对着假臂也行）"
                                         : string.Format("  ❌ {0} 项失败", fails));
            return fails == 0 ? 0 : 1;
        }

        private static string Join6(double?[] q)
        {
            var parts = new string[q.Length];
            for (int i = 0; i < q.Length; i++)
                parts[i] = q[i].HasValue ? q[i].Value.ToString("F1") : "--";
            return string.Join(",", parts);
        }

        private static string FirstLine(string s)
        {
            if (string.IsNullOrEmpty(s)) return "";
            int i = s.IndexOfAny(new char[] { '\r', '\n' });
            return (i < 0 ? s : s.Substring(0, i)).Trim();
        }
    }
}
