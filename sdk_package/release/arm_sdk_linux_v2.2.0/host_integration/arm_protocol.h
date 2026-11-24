/* ===========================================================================
 * arm_protocol.h — 六轴机械臂 网口协议 参考实现（纯 C，无依赖，可直接抄进 C/C++/C#）
 *
 * 用途：给上位机（WinForm/WPF/Qt/MFC/组态）工程师一个**能编译、能对照**的实现。
 *       Java/Go/Rust 也一样能照着抄 —— 逻辑就这么点。
 *
 * 链路：UDP，目的端口 **5000**
 *   一个 UDP 数据报 = 一个逻辑包（**没有帧头、没有分包、没有重传**）
 *   回包发往**请求的源 IP:源端口**，所以本端不必绑固定端口
 *
 * 报文格式（三承载共用，共 3+LEN+2 字节）：
 *   ┌─────┬─────┬──────┬───────────────┬──────────────┐
 *   │ SEQ │ CMD │ LEN  │ PAYLOAD(LEN)  │ CRC_LO CRC_HI│
 *   │ 1B  │ 1B  │ 1B   │ LEN ≤ 64      │ 2B 小端      │
 *   └─────┴─────┴──────┴───────────────┴──────────────┘
 *   ⚠ LEN 是 **1 字节**（0~64），不是 u16 —— 这是最容易写错的地方
 *   ⚠ CRC 范围 = SEQ+CMD+LEN+PAYLOAD（即前 3+LEN 字节），**小端**存放
 *   ⚠ 报文长度必须**恰好** 3+LEN+2；不等则固件**静默丢弃**（只表现为超时）
 *
 * ⚠ CMD ≥ 0x80 的包固件**不回**（防死循环）—— 那是回包用的号段
 * ⚠ 必须按 SEQ 匹配回包：UDP 会乱序，同一个 socket 上并发发多条会交叉
 *
 * 用法：
 *     uint8_t buf[80]; int n = arm_pack(buf, sizeof buf, seq, ARM_CMD_PING, NULL, 0);
 *     sendto(sock, buf, n, ...);
 *     // ... recvfrom ...
 *     uint8_t seq2, cmd2, pl[64]; uint8_t pl_len;
 *     if (arm_unpack(rx, rxlen, &seq2, &cmd2, pl, &pl_len) != 0) { 丢弃 }
 *
 * 本文件只依赖 <stdint.h> <string.h>。C99 起可用。
 * =========================================================================== */

#ifndef ARM_PROTOCOL_H
#define ARM_PROTOCOL_H

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ 尺寸 */
#define ARM_MAX_PAYLOAD     64      /* 负载上限，超了固件丢弃 */
#define ARM_MAX_PACKET      (3 + ARM_MAX_PAYLOAD + 2)   /* 69 B */

/* ------------------------------------------------------------------ 命令码 */
/* 查询类（无副作用，可以随时发） */
#define ARM_CMD_PING             0x01
#define ARM_CMD_GET_STATUS       0x02
#define ARM_CMD_LOG_SET_LEVEL    0x03
#define ARM_CMD_GET_JOINT_DETAIL 0x04   /* 关节遥测：温度/电压/电流/负载/移动/硬件错 */
#define ARM_CMD_GET_LIMITS       0x05
#define ARM_CMD_GET_INFO         0x07   /* 固件版本 / 构建 / 链路 / 参数个数 */
#define ARM_CMD_GET_FLAGS        0x0A
#define ARM_CMD_FK_QUERY         0x21
#define ARM_CMD_IK_QUERY         0x22
#define ARM_CMD_PARAM_READ       0x40
#define ARM_CMD_PARAM_COUNT      0x42
#define ARM_CMD_PARAM_LIST       0x44
#define ARM_CMD_GET_GRAVITY      0x32
#define ARM_CMD_GET_CORRECTION   0x33
#define ARM_CMD_GET_PID          0x36
#define ARM_CMD_TEACH_LIST       0x50
#define ARM_CMD_TEACH_READ       0x51
#define ARM_CMD_TEACH_TOTAL      0x55

/* 运动类 */
#define ARM_CMD_MOVE_JOINTS      0x10   /* 绝对角（舵机度） */
#define ARM_CMD_STOP             0x11
#define ARM_CMD_SET_TORQUE       0x12
#define ARM_CMD_STREAM_JOINTS    0x13   /* 50 Hz 流控（逻辑角增量）★ 遥操/回放用这条 */
#define ARM_CMD_MOVE_JOINT_ONE   0x14
#define ARM_CMD_MOVE_CART        0x20   /* 笛卡尔 */

/* 配置类（会改状态/落 Flash，UI 上必须二次确认） */
#define ARM_CMD_SET_BUS_MODE     0x17
#define ARM_CMD_SERVO_INIT       0x18
#define ARM_CMD_SET_GRAVITY      0x30
#define ARM_CMD_SET_CORRECTION   0x31
#define ARM_CMD_PARAM_WRITE      0x41
#define ARM_CMD_PARAM_SAVE       0x43
#define ARM_CMD_TEACH_DELETE     0x53
#define ARM_CMD_TEACH_COMPACT    0x54

/* 危险类（**必须带 MAGIC**，否则回 NACK err=8） */
#define ARM_CMD_CENTER_CALIB     0x70   /* 六轴中位标定 —— 会废掉量纲基准 */
#define ARM_CMD_CALIB_ENTER      0x71   /* 交互式校准向导 —— **一律 NACK**，见下 */
#define ARM_CMD_ZERO_RESET       0x72   /* 清 Home 零点 */
#define ARM_CMD_SYS_RESET        0x73   /* 软复位（延后到回包之后执行） */

/* 文本透传：把整条控制台搬上网口（做上位机时非常有用） */
#define ARM_CMD_TEXT_EXEC        0x0B   /* u32 magic + text，见下 */
#define ARM_CMD_TEXT_FETCH       0x0C   /* u16 off + u8 n → 取下一页回显 */

/* ------------------------------------------------------------------ 回应码 */
#define ARM_RSP_ACK              0x7F   /* 无载荷，纯确认 */
#define ARM_RSP_NACK             0x7E   /* payload = [原cmd, err] 2 字节 */
#define ARM_RSP_PING             0x81
#define ARM_RSP_STATUS           0x82
#define ARM_RSP_JOINT_DETAIL     0x84
#define ARM_RSP_LIMITS           0x85
#define ARM_RSP_INFO             0x87
#define ARM_RSP_FLAGS            0x8A
#define ARM_RSP_TEXT_OUT         0x8B
#define ARM_RSP_TEXT_FETCH       0x8C
#define ARM_RSP_FK               0xA1
#define ARM_RSP_IK               0xA2
#define ARM_RSP_GRAVITY          0xB2
#define ARM_RSP_CORRECTION       0xB3
#define ARM_RSP_PID              0xB6
#define ARM_RSP_PARAM_READ       0xC0   /* = 0x80 + 0x40 */
#define ARM_RSP_PARAM_COUNT      0xC2
#define ARM_RSP_PARAM_LIST       0xC4
#define ARM_RSP_TEACH_LIST       0xD0
#define ARM_RSP_TEACH_READ       0xD1
#define ARM_RSP_TEACH_TOTAL      0xD5
/* 规律：RSP = CMD | 0x80（ACK/NACK 除外） */

/* ------------------------------------------------------------------ 错误码 */
#define ARM_ERR_OK         0    /* 成功 */
#define ARM_ERR_PARAM      1    /* 参数非法（用错量纲/越界/枚举无此值） */
#define ARM_ERR_CRC        2    /* CRC 或长度错 */
#define ARM_ERR_UNREACH    3    /* IK 无解 / 目标不可达 */
#define ARM_ERR_LIMIT      4    /* 越限（含单帧跳变 >25° 被整帧拒收） */
#define ARM_ERR_ESTOP      5    /* 急停生效 */
#define ARM_ERR_FK_VERIFY  6    /* 上报位姿与正解不符（安全拒动） */
#define ARM_ERR_READ       7    /* 舵机读失败（有轴丢读） */
#define ARM_ERR_MAGIC      8    /* 危险命令缺/错 MAGIC */
#define ARM_ERR_BUSY       9    /* 轨迹/示教/回放进行中，此刻不受理 */
#define ARM_ERR_NOTCAL    10    /* 未校准 */

/* ------------------------------------------------------------------ 常量 */
#define ARM_MAGIC          0x5A5A5A5AUL  /* 危险命令的 payload 开头 */
#define ARM_TEMP_LOST      0xFF          /* 温度丢读哨兵 —— **不是 0**！ */
#define ARM_HWERR_LOST     0xFF          /* 硬件错丢读哨兵 */
#define ARM_TEXT_BUF_SZ    2048          /* 文本回显缓冲；超了会被截断 */
#define ARM_TEXT_EXEC_MAX  58            /* 单帧能带的 UTF-8 字节数上限 */
#define ARM_TEXT_CHUNK     58            /* 分页 每页字节数 */
#define ARM_TEXT_FLAG_TRUNC 0x01         /* TEXT_OUT.flags bit0：输出被截断 */
#define ARM_TEXT_FLAG_GATE  0x02         /* TEXT_OUT.flags bit1：被安全闸门拒绝 */
#define ARM_PARAM_LIST_MAX 6             /* PARAM_LIST 每页条数（10B/条 + 2B 头） */

/* ------------------------------------------------------------------ CRC16 */
/* CRC-16/CCITT-FALSE：poly=0x1021, init=0xFFFF，**非反射**，无最终异或
 * 自检向量：arm_crc16("123456789", 9) == 0x29B1   ← 先验这个再往下写 */
static inline uint16_t arm_crc16(const uint8_t *d, int n)
{
    uint16_t crc = 0xFFFF;
    int i, b;
    for (i = 0; i < n; i++) {
        crc ^= (uint16_t)((uint16_t)d[i] << 8);
        for (b = 0; b < 8; b++)
            crc = (crc & 0x8000u) ? (uint16_t)((uint16_t)(crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
    }
    return crc;
}

/* ------------------------------------------------------------------ 组包 */
/* 返回整包长度（成功）或 -1（负载超长）。buf 至少 ARM_MAX_PACKET 字节。 */
static inline int arm_pack(uint8_t *buf, int bufsz, uint8_t seq, uint8_t cmd,
                    const uint8_t *payload, uint8_t len)
{
    uint16_t c;
    if (len > ARM_MAX_PAYLOAD || bufsz < 3 + len + 2)
        return -1;
    buf[0] = seq;
    buf[1] = cmd;
    buf[2] = len;
    if (len && payload)
        memcpy(buf + 3, payload, len);
    c = arm_crc16(buf, 3 + len);
    buf[3 + len]     = (uint8_t)(c & 0xFF);        /* 小端 */
    buf[4 + len]     = (uint8_t)((c >> 8) & 0xFF);
    return 5 + len;
}

/* ------------------------------------------------------------------ 解包 */
/* 返回 0 成功；-1 长度不符；-2 CRC 错。payload 缓冲至少 ARM_MAX_PAYLOAD。 */
static inline int arm_unpack(const uint8_t *rx, int rxlen, uint8_t *seq, uint8_t *cmd,
                      uint8_t *payload, uint8_t *plen)
{
    uint8_t len;
    uint16_t got, want;
    if (rxlen < 5)
        return -1;
    len = rx[2];
    if (len > ARM_MAX_PAYLOAD || rxlen != 3 + len + 2)
        return -1;
    got  = (uint16_t)rx[3 + len] | ((uint16_t)rx[4 + len] << 8);
    want = arm_crc16(rx, 3 + len);
    if (got != want)
        return -2;
    if (seq)   *seq = rx[0];
    if (cmd)   *cmd = rx[1];
    if (plen)  *plen = len;
    if (payload && len)
        memcpy(payload, rx + 3, len);
    return 0;
}

/* ------------------------------------------------------------------ 类型 */
/* 定点约定：角度/位置 = i16，**单位 0.1**（25.3° → 253）；时间 ms = i16 */
#define ARM_DECI(v)  ((double)(v) / 10.0)      /* i16 → 实际值 */
#define ARM_ENC(v)   ((int16_t)((v) >= 0 ? ((v) + 0.5) : ((v) - 0.5)))
                                               /* 实际值 → i16，四舍五入**远离零** */

static inline int16_t arm_rd_i16(const uint8_t *p, int off)
{
    return (int16_t)((uint16_t)p[off] | ((uint16_t)p[off + 1] << 8));
}
static inline void arm_wr_i16(uint8_t *p, int off, int16_t v)
{
    p[off]     = (uint8_t)((uint16_t)v & 0xFF);
    p[off + 1] = (uint8_t)(((uint16_t)v >> 8) & 0xFF);
}
static inline uint16_t arm_rd_u16(const uint8_t *p, int off)
{
    return (uint16_t)((uint16_t)p[off] | ((uint16_t)p[off + 1] << 8));
}
static inline void arm_wr_u16(uint8_t *p, int off, uint16_t v)
{
    p[off]     = (uint8_t)(v & 0xFF);
    p[off + 1] = (uint8_t)((v >> 8) & 0xFF);
}
/* 实际值 → u16，负数先钳到 0（速度/加速度这种量的固件语义就是"0=用默认"） */
static inline uint16_t arm_encu(double v)
{
    if (v < 0) v = 0;
    return (uint16_t)(v + 0.5);
}

/* ============================ 常用载荷构造 ============================ */

/* PING：空载荷。回 ARM_RSP_PING，payload = [proto_ver, run_id] */
#define ARM_PING_LEN 0

/* GET_STATUS：空载荷。回 ARM_RSP_STATUS，21 字节：
 *   6 × i16  舵机角（单位 0.1°）；**读失败 = -1（0xFFFF），不要当 0 用**
 *   4 × i16  X, Y, Z, pitch（单位 0.1）；任一轴读失败 ⇒ 全 0
 *   1 × u8   flags：bit3=read_ok  bit2=corr  bit1=traj  bit0=estop
 */
#define ARM_STATUS_LEN 21
static inline int arm_status_read_ok(const uint8_t *pl, uint8_t *flags)
{
    *flags = pl[20];
    return (pl[20] & 0x08) ? 1 : 0;
}

/* MOVE_JOINTS（0x10）：绝对角，**舵机度**（不是逻辑角！）= 16 字节
 *   6 × i16  目标角（单位 0.1°，如 18000 = 180.0°）
 *   1 × u16  速度 v（**单位 0.1**：v=100 → 写 1000；0 = 用臂内默认）
 *   1 × u16  加速度 a（**单位 0.1**，同上）
 * ⚠ v/a 是 u16 **且乘 10** —— 写成 i16 或不乘 10 是最常见的实现错误：
 *   传 20 结果固件收到 2（几乎不动）或 2000（满速冲），**都不报错**。
 */
#define ARM_MOVE_JOINTS_LEN 16
static inline int arm_make_move_joints(uint8_t *pl, const double *deg6,
                                       double v, double a)
{
    int j;
    for (j = 0; j < 6; j++)
        arm_wr_i16(pl, j * 2, ARM_ENC(deg6[j] * 10.0));
    arm_wr_u16(pl, 12, arm_encu(v * 10.0));
    arm_wr_u16(pl, 14, arm_encu(a * 10.0));
    return ARM_MOVE_JOINTS_LEN;
}

/* MOVE_CART（0x20）：笛卡尔 = 12 字节
 *   i16 x, y, z（mm ×10）  i16 pitch（deg ×10）  u16 v（×10）  u16 a（×10）
 * ⚠ 坐标原点是**夹爪中心 TCP**（定指尖与动指尖的中点），不是指尖
 * ⚠ 会归零 J5/J6 —— 想保夹爪开合请改用 MOVE_JOINTS / STREAM_JOINTS
 */
#define ARM_MOVE_CART_LEN 12
static inline int arm_make_move_cart(uint8_t *pl, double x, double y, double z,
                                     double pitch, double v, double a)
{
    arm_wr_i16(pl, 0, ARM_ENC(x * 10.0));
    arm_wr_i16(pl, 2, ARM_ENC(y * 10.0));
    arm_wr_i16(pl, 4, ARM_ENC(z * 10.0));
    arm_wr_i16(pl, 6, ARM_ENC(pitch * 10.0));
    arm_wr_u16(pl, 8, arm_encu(v * 10.0));
    arm_wr_u16(pl, 10, arm_encu(a * 10.0));
    return ARM_MOVE_CART_LEN;
}

/* STREAM_JOINTS（0x13）：流控帧，**逻辑角增量**，单位 0.1°
 *   6 × i16  = 12 字节
 * ⚠ 语义是"相对进入流的那一帧的增量"，不是绝对角
 * ⚠ 单帧单轴跳变 > 25° ⇒ **整帧拒收且不更新基准**，连续几次就自锁冻结
 *   ⇒ 上游必须限速（50 Hz 下 240°/s = 4.8°/帧，留 5× 余量）
 */
#define ARM_STREAM_JOINTS_LEN 12
static inline int arm_make_stream_joints(uint8_t *pl, const double *delta_deg6)
{
    int j;
    for (j = 0; j < 6; j++)
        arm_wr_i16(pl, j * 2, ARM_ENC(delta_deg6[j] * 10.0));
    return ARM_STREAM_JOINTS_LEN;
}

/* STOP（0x11）：空载荷。随时可发，包括 E-Stop 期间 */
#define ARM_STOP_LEN 0

/* SET_TORQUE（0x12）：[u8 joint, u8 enable]；joint=0xFF 表示全部 */
static inline int arm_make_set_torque(uint8_t *pl, uint8_t joint, int enable)
{
    pl[0] = joint;
    pl[1] = enable ? 1 : 0;
    return 2;
}

/* LOG_SET_LEVEL（0x03）：[u8 level 0~4]。level 越大越啰嗦（会占用网口带宽） */
static inline int arm_make_log_level(uint8_t *pl, uint8_t level)
{
    pl[0] = level > 4 ? 4 : level;
    return 1;
}

/* PARAM_READ（0x40）：[u8 id]。回 ARM_RSP_PARAM_READ：u8 id + i32 值(×1000) */
static inline int arm_make_param_read(uint8_t *pl, uint8_t id)
{
    pl[0] = id;
    return 1;
}

/* MAGIC 前缀（危险命令专用）：4 字节小端 0x5A5A5A5A */
static inline int arm_put_magic(uint8_t *pl)
{
    pl[0] = 0x5A; pl[1] = 0x5A; pl[2] = 0x5A; pl[3] = 0x5A;
    return 4;
}

/* ======================= 文本透传（做上位机强烈推荐） =======================
 *
 * 有了这两条，**38 条二进制之外还有 113 条控制台命令**可以直接用字符串发，
 * 不用为每条命令单独写一遍载荷编码。
 *
 * TEXT_EXEC(0x0B) 载荷：u32 magic(小端 0x5A5A5A5A) + text（UTF-8，**不带 \r**）
 *                      text 最多 ARM_TEXT_EXEC_MAX(58) 字节
 *   回 ARM_RSP_TEXT_OUT，格式：
 *     u16 total(小端，**字节数**)  u8 flags  u16 off  u8 n  + n 字节 UTF-8 正文
 *   若 total > n ⇒ 用 TEXT_FETCH(off+n, n) 继续取，直到取满 total
 *
 * TEXT_FETCH(0x0C) 载荷：u16 off(小端) + u8 n（n ≤ 58）
 *
 * ⚠ 分页必须按**字节**拼接，最后统一按 UTF-8 解码 ——
 *   中文一个字 3 字节，页边界会把它切开。**不要每页单独解码**。
 * ⚠ flags.bit0=1 表示输出超过 2048 B 被截断（要如实告知用户"只看到前 2 KB"）
 * ⚠ flags.bit1=1 表示被**安全闸门**拒绝，正文是原因（例如"急停中只放行 s/h"）
 * ⚠ 抢 ETH 引脚的 11 条命令（cx cp cy cz cw ps can mg mp mo ml）会被固件
 *   主动拦下；交互式命令 `c`（校准向导）从网口进**一律拒绝**，因为它会
 *   死等本机键盘输入，而本工程**没有看门狗** ⇒ 主循环永远回不来。
 */
/* ⚠ 正文必须是 **ASCII**（固件 encode("ascii")，非 ASCII 直接报错），
 *   末尾必须补一个 **NUL(0x00)**，总长 = 4 + n + 1。
 *   长度上限 58 是**字符数**，因为限 ASCII 所以等于字节数。 */
static inline int arm_make_text_exec(uint8_t *pl, const char *ascii_cmd)
{
    int n = 0;
    const char *p = ascii_cmd;
    arm_put_magic(pl);
    if (!p)
        return -1;
    while (*p == ' ' || *p == '\t') p++;            /* 掐前导空白 */
    while (p[n] && p[n] != '\r' && p[n] != '\n') {
        if ((unsigned char)p[n] >= 0x80)
            return -1;                                /* 非 ASCII ⇒ 报错 */
        if (n >= ARM_TEXT_EXEC_MAX)
            return -1;                                /* 超长 ⇒ 报错，不静默截断 */
        pl[4 + n] = (uint8_t)p[n];
        n++;
    }
    pl[4 + n] = 0x00;                                 /* NUL 结尾 */
    return 4 + n + 1;
}

static inline int arm_make_text_fetch(uint8_t *pl, uint16_t off, uint8_t n)
{
    pl[0] = (uint8_t)(off & 0xFF);
    pl[1] = (uint8_t)((off >> 8) & 0xFF);
    pl[2] = n;
    return 3;
}

/* 解析 TEXT_OUT/TEXT_FETCH 的正文（返回正文指针与长度，total 为总字节数） */
typedef struct {
    uint16_t total;
    uint8_t  flags;
    uint16_t off;
    uint8_t  n;
    const uint8_t *text;      /* 指向 payload 内部，不复制 */
} arm_text_out_t;

static inline int arm_parse_text_out(const uint8_t *pl, uint8_t pl_len, arm_text_out_t *o)
{
    if (pl_len < 6)
        return -1;
    o->total = (uint16_t)pl[0] | ((uint16_t)pl[1] << 8);
    o->flags = pl[2];
    o->off   = (uint16_t)pl[3] | ((uint16_t)pl[4] << 8);
    o->n     = pl[5];
    if ((uint8_t)(6 + o->n) > pl_len)
        return -1;
    o->text  = pl + 6;
    return 0;
}

/* ------------------------------------------------------------------ 参数表 */
/* 运行时参数（PARAM_READ / PARAM_WRITE 的 id）。**不要硬编码这些值** ——
 * 从 config/arm_config.json 读，或先用 PARAM_LIST(0x44) 枚举一遍。 */
#define ARM_PARAM_COUNT_RUNTIME  24

#ifdef __cplusplus
}   /* extern "C" */
#endif

#endif /* ARM_PROTOCOL_H */
