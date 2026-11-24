/* ============================================================
 * arm_control_protocol.h - 机械臂二进制控制协议核心
 *
 * 一份协议，两种承载：串口 (arm_proto_serial.c) 与 CAN
 * (arm_proto_can.c) 共用本文件的帧格式、命令码与编解码。
 * 本文件只做纯逻辑（无 HAL 依赖），便于上位机移植与单元测试。
 *
 * 串口帧格式（小端，逐字节流）:
 *   [0]  0xAA          帧头 1（文本控制台不会输出，天然分流）
 *   [1]  0x55          帧头 2
 *   [2]  SEQ           请求序号（响应回显，用于请求-应答配对）
 *   [3]  CMD           命令码（见下表）
 *   [4]  LEN           负载长度 0~APROTO_MAX_PAYLOAD
 *   [5..] PAYLOAD      定点编码负载（见各命令注释）
 *   [-2] CRC16 低字节  CRC-16/CCITT-FALSE (poly 0x1021 init 0xFFFF)
 *   [-1] CRC16 高字节  计算范围: SEQ..PAYLOAD
 *
 * 定点编码（避免 MCU float 序列化与字节序歧义）:
 *   角度    int16, 0.1 deg    位置    int16, 0.1 mm
 *   速度    uint16, 0.1 deg/s 加速度  uint16, 0.1 deg/s^2
 *   增益K   int16, 0.001 deg/mm 参数值 int32, 0.001 (PARAM_READ/WRITE)
 *
 * 响应规则: 数据查询类响应 CMD = 请求 CMD | 0x80；
 *           控制类成功回 ACK(0x7F)，失败回 NACK(0x7E)+错误码
 *           （错误码 = arm_control_api.h 的 ARM_API_ERR_*）。
 * ============================================================ */
#ifndef __ARM_CONTROL_PROTOCOL_H
#define __ARM_CONTROL_PROTOCOL_H

#include <stdint.h>

#define APROTO_HEAD0        0xAA
#define APROTO_HEAD1        0x55
#define APROTO_MAX_PAYLOAD  64
#define APROTO_MAX_FRAME    (5 + APROTO_MAX_PAYLOAD + 2)   /* 71 字节 */

#define APROTO_VERSION      0x01    /* 协议版本，PING 响应携带 */

/* 危险命令的确认字 (2026-09-23)。必须逐字节构造才命中 —— 用途见文件内
 * APROTO_CMD_CENTER_CALIB 等处的说明: 在无连接/无重传通道上, 防"扫描器
 * 误打"或"串包"触发不可逆操作(废量纲基准 / 松全部力矩 / 重置 Home)。 */
#define APROTO_MAGIC        0x5A5A5A5AU

/* ---- 命令码 ---- */
enum {
    /* 系统 */
    APROTO_CMD_PING          = 0x01,  /* -> 0x81: u8 proto_ver, u8 run_id */
    APROTO_CMD_GET_STATUS    = 0x02,  /* -> 0x82: ArmApiStatus 定点快照 */
    APROTO_CMD_LOG_SET_LEVEL = 0x03,  /* u8 level(0~3) -> ACK */

    /* 七段 S 曲线轨迹 + 关节控制 */
    APROTO_CMD_MOVE_JOINTS   = 0x10,  /* 6xi16 角度 + u16 v + u16 a -> ACK */
    APROTO_CMD_STOP          = 0x11,  /* -> ACK（立即停止轨迹） */
    APROTO_CMD_SET_TORQUE    = 0x12,  /* u8 joint(0~5,0xFF=全) u8 en -> ACK */
    APROTO_CMD_STREAM_JOINTS = 0x13,  /* 6xi16 逻辑角*10 -> ACK
                                       * 主从直写: 不做轨迹规划/起点读/settle,
                                       * 供主臂端 20~50Hz 连续跟随。
                                       * 角度是"相对本臂 Home 的逻辑关节角"
                                       * (与 MOVE_JOINTS 的舵机度不同), 两端
                                       * 零点各自校准, 故必须用逻辑角传递。 */

    /* FK/IK-CCD 运动学 */
    APROTO_CMD_MOVE_CART     = 0x20,  /* i16 x,y,z + i16 pitch + u16 v + u16 a */
    APROTO_CMD_FK_QUERY      = 0x21,  /* -> 0xA1: 位姿 + 六关节角 */

    /* 自适应重力补偿 */
    APROTO_CMD_SET_GRAVITY   = 0x30,  /* i16 K(0.001 deg/mm) -> ACK */
    APROTO_CMD_SET_CORRECTION= 0x31,  /* u8 en, u16 thr(0.01deg), u8 rounds */

    /* 统一参数表（arm_params） */
    APROTO_CMD_PARAM_READ    = 0x40,  /* u8 id -> 0xC0: u8 id, i32 val*1000 */
    APROTO_CMD_PARAM_WRITE   = 0x41,  /* u8 id, i32 val*1000 -> ACK */
    APROTO_CMD_PARAM_COUNT   = 0x42,  /* -> 0xC2: u8 count */
    APROTO_CMD_PARAM_SAVE    = 0x43,  /* -> ACK (替代文本 F 的参数落盘) */
    APROTO_CMD_PARAM_LIST    = 0x44,  /* u8 offset -> 0xC4: u8 total, u8 n,
                                       *   n x 10B {u8 id, u8 type, i32 vmin,
                                       *            i32 vmax}
                                       * ⚠ vmin/vmax 是 i32×1000 —— 参数范围里
                                       *   有超过 i16 的值(如 traj.approach_speed
                                       *   上限 4095)，压成 i16 会**静默截断**。
                                       *   10B/条 × 6 = 62B(含 2B 头) <= 64。
                                       * 值用 PARAM_READ 单取(带上 i32 值会超 64B)。
                                       * n == 0 或 offset+n >= total 即读完。 */

    /* ============================================================
     * 2026-09-23 扩展：让网口覆盖文本控制台能力
     * 设计基准 docs/网口指令扩展设计_2026-09-23.md
     * 四条铁律：
     *   ① 命令码必须 < 0x80 —— Dispatch 把 cmd>=0x80 当回包丢弃；
     *   ② 单帧 payload <= APROTO_MAX_PAYLOAD(64) ⇒ 大数组走【分页】；
     *   ③ 危险命令必须带 magic=APROTO_MAGIC, 无连接通道上防误包触发；
     *   ④ 新命令一律经 ArmApi_*, 不许在 protocol.c 里直接碰 HAL。
     * ============================================================ */

    /* ---- 只读：诊断与遥测（替代文本 s/T/O/E/zi/h/ms） ---- */
    APROTO_CMD_GET_JOINT_DETAIL = 0x04,
    /* u8 joint(0~5 | 0xFF=全) -> 0x84: u8 n, u8 joint0, n x 10B
     *   每轴 10 B: u16 pos(raw 0~4095), i16 speed(step/s),
     *              i16 load(有符号), u8 volt(0.1V), u8 temp(°C, 0xFF=丢读),
     *              u8 err(硬件错误位), u8 ok(1=本轴读取成功)
     * ⚠ 6 轴 = 2B 头 + 6 x 10B = 62 B，不超 64。若加 u16 current 会到 74B 超限
     *   ⇒ 电流未含, 需要时用 SERVO_ReadTelemetry 的单轴形态, 见设计文档。 */
    APROTO_CMD_GET_LIMITS       = 0x05,
    /* -> 0x85: 6 x 8B = 48B
     *   每轴 u16 min_raw, pos_raw, max_raw, zero_raw
     *   全部取自 PID_FlashLoad() 的 PID_JointLimits_t；
     *   zero_raw == 0 表示该轴未做 Home 零点校准(固件回落 180°/raw 2048)。 */
    APROTO_CMD_GET_INFO         = 0x07,
    /* -> 0x87: u8 tag_len, char fw_tag[16], u8 build, u8 link, u8 nsrv
     *   build: 0=net 1=can 2=both 3=uart 4=off
     *   link : LinkCfg_ActiveMode() 的【启动冻结】值(不是偏好)。 */
    APROTO_CMD_GET_FLAGS        = 0x0A,
    /* -> 0x8A: u8 flags(ArmMain_Flags 位图), u8 run_id = 2B
     *   flags: bit0=E-Stop bit1=轨迹 bit2=流模式 bit3=修正
     *          bit4=示教录制 bit5=示教回放
     * ⚠ 只读变量, 不碰舵机总线 ⇒ 可以 10~50Hz 轮询(替代文本 ms 的部分)。 */

    /* ---- 文本命令透传（2026-09-23, 全指令覆盖的关键一条） ----
     * 上面那些 opcode 是给热路径用的**类型化**接口; 而控制台有 124 条命令,
     * 逐条加 opcode 既追不上以后新增的、又会造成两套语义慢慢漂移。这里用
     * **一条**命令把整个控制台搬到网口: 负载里就是一行 ASCII 命令。 */
    APROTO_CMD_TEXT_EXEC        = 0x0B,
    /* u32 magic, char text[] (NUL 结尾, 有效长度 <= 58)
     * -> 0x8B: u16 total, u8 flags, u16 off, u8 n, char text[n]
     *    flags bit0 = 1 表示输出被 ARM_TEXT_BUF_SZ(2048) 截断
     * 执行流程 = COM11 上敲同一行的**完全相同**路径:
     *   ArmMain_TextGate(闸门) -> ArmMain_ExecText -> ProcessCommand
     * ⚠ magic 是**无条件**要求的(不像 0x7X 只在那几条上要求): 这条命令能
     *   做的事等于整个控制台, 天然是最危险的一条, 一律要求显式构造。
     * ⚠ 输出 >2048 B 只截断并置 flags.bit0, 绝不静默丢字符。 */
    APROTO_CMD_TEXT_FETCH       = 0x0C,
    /* u16 off, u8 n(1~58) -> 0x8C: 同 0x8B 的负载格式
     * 取上一条 TEXT_EXEC 输出的后续分页(实测单页装得下绝大多数命令;
     * 只有 `h` 的帮助表 5406 B 需要翻页)。 */

    /* ---- 运动：单轴与配置（替代文本 a/b/i） ---- */
    APROTO_CMD_MOVE_JOINT_ONE   = 0x14,
    /* u8 joint(0~5), i16 deg*10, u16 v*10, u16 a*10 -> ACK
     * v/a 传 0 用 AP_SC_DEF_* 默认值(与 MOVE_JOINTS 同规则)。 */
    APROTO_CMD_SET_BUS_MODE     = 0x17,  /* u8 mode(0=正常/1=全A/2=全B) -> ACK */
    APROTO_CMD_SERVO_INIT       = 0x18,  /* u8 joint(0~5 | 0xFF=全) -> ACK
                                          * ⚠ 会重写该轴 PID/限位/中位并 RESET，
                                          *   与文本 i 同副作用。 */

    /* ---- 运动学：只解不算（文本 g 的"先看能不能到"） ---- */
    APROTO_CMD_IK_QUERY         = 0x22,
    /* i16 x,y,z(mm*10), i16 pitch*10 -> 0xA2: 6 x i16 舵机度*10
     * 返回 UNREACH/LIMIT/FK_VERIFY 错误码，不启动运动。 */

    /* ---- 补偿：补读侧（SET_* 早已有，缺 GET_*） ---- */
    APROTO_CMD_GET_GRAVITY      = 0x32,  /* -> 0xB2: i16 K(0.001 deg/mm) */
    APROTO_CMD_GET_CORRECTION   = 0x33,
    /* -> 0xB3: u8 en, u16 threshold*100, u8 max_rounds, u8 pending */
    APROTO_CMD_GET_PID          = 0x36,
    /* u8 joint(0~5) -> 0xB6: u8 p, u8 i, u8 d, u8 from_eeprom
     * from_eeprom=1 表示读自舵机 EEPROM(权威)，0 表示读自固件缓存。 */

    /* ---- 示教/轨迹存储（替代文本 list/play/del/compact/rec list） ---- */
    APROTO_CMD_TEACH_LIST       = 0x50,
    /* u8 idx -> 0xD0: u8 total, u8 idx, u8 valid,
     *                char name[TSTORE_NAME_MAX+1], u32 samples
     * idx 越界 -> ACK 带 ERR_PARAM；valid=0 表示该槽位是 tombstone。 */
    APROTO_CMD_TEACH_READ       = 0x51,
    /* u8 idx, u16 off, u8 n(1~5) -> 0xD1: u8 n, n x 12B (6 x u16 raw)
     * ⭐ 分页读: 12B x 5 = 60 <= 64。PC 侧循环直到 n < 请求数。
     *   1000 点轨迹 = 200 次请求；网口 p50 1.61ms ⇒ 约 0.3s 取完。
     *   off 越界 -> n=0（正常结束，不报错）。 */
    APROTO_CMD_TEACH_DELETE     = 0x53,  /* u8 idx -> ACK (写 tombstone) */
    APROTO_CMD_TEACH_COMPACT    = 0x54,  /* -> ACK (回收 tombstone 空间) */
    APROTO_CMD_TEACH_TOTAL      = 0x55,
    /* -> 0xD5: u8 nfiles, u8 capacity, u32 total_samples, u32 free_bytes */

    /* ---- 危险操作：强制 magic（0x5A5A5A5A） ----
     * 无连接(UDP)/无重传通道上，这几条误触发即不可逆：
     *   M a 会废掉量纲基准、zr 会重置 Home 零点、软复位会中断一切在跑的东西。
     * magic 的作用是让"扫描器误打/串包"无法命中，必须显式构造。
     * 实现上它们**统一走文本透传**(见 0x0B)：M a / zr 就是控制台上那两条,
     * 不另写一份实现 —— 两份实现迟早会分叉, 而这几条的后果是不可逆的。 */
    APROTO_CMD_CENTER_CALIB     = 0x70,  /* u32 magic -> ACK (执行文本 `M a`) */
    APROTO_CMD_CALIB_ENTER      = 0x71,
    /* u32 magic -> **一律 NACK(ERR_PARAM)**。
     * `c` 是相位校准向导, cmdCalibrate() 里是 for(;;) 等人在**本机串口**上敲
     * '0'/'s'/'q'; 从网口进 = 主循环永不返回, 而本工程**没有看门狗** ⇒ 板子
     * 彻底失联。它本来就要求人手摆位, 设计上是控制台专属命令。 */
    APROTO_CMD_ZERO_RESET       = 0x72,
    /* u8 flags(bit0=1 同时落盘, 即文本 `zr save`), u32 magic -> ACK
     * ⚠ 文本 `zr` **没有单轴变体**(一次复位全部六轴), 所以这里也不给关节号。 */
    APROTO_CMD_SYS_RESET        = 0x73,
    /* u32 magic -> ACK 之后再复位(见 ArmMain_RequestReset: 顺序反了上位机
     * 只会看到超时)。远程切链路的完整序列: link can -> link save -> SYS_RESET。 */

    /* 响应 */
    APROTO_RSP_ACK           = 0x7F,  /* u8 orig_cmd, u8 err(=0 OK) */
    APROTO_RSP_NACK          = 0x7E   /* u8 orig_cmd, u8 err(ARM_API_ERR_*) */
};

/* ---- 帧解析状态机（ISR 安全：仅状态迁移与缓冲） ---- */
typedef struct {
    uint8_t  state;         /* 0=等头1 1=等头2 2=SEQ 3=CMD 4=LEN 5=数据 6=CRC0 7=CRC1 */
    uint8_t  seq;
    uint8_t  cmd;
    uint8_t  len;
    uint8_t  idx;
    uint8_t  payload[APROTO_MAX_PAYLOAD];
    uint8_t  crc_lo;        /* 暂存 CRC 低字节 */
    uint32_t last_tick;     /* 最近字节时刻，帧间超时复位用 */
} AprotoParser;

/* CRC-16/CCITT-FALSE（poly 0x1021, init 0xFFFF, 非反射） */
uint16_t Aproto_Crc16(const uint8_t* data, uint16_t len);

/* 复位解析状态机（帧超时/错误恢复用） */
void     Aproto_ParserReset(AprotoParser* p);

/* 逐字节喂入解析。返回:
 *   0 = 该字节不属于协议帧（可交给文本控制台）
 *   1 = 已消费（帧的一部分，需继续喂）
 *   2 = 一帧完整且 CRC 正确（p 中可取 seq/cmd/payload）
 *   3 = 一帧完整但 CRC 错误（状态机已复位）
 * 注意: 仅当返回 >=1 时字节被协议消费；返回 2/3 的那个字节
 *       也已被消费。ISR 可安全调用（无阻塞、无打印）。 */
uint8_t  Aproto_ParserFeed(AprotoParser* p, uint8_t byte, uint32_t now_ms);

/* 组帧：写入 out，返回帧长（out 不足返回 0）。crc 由本函数计算。 */
uint8_t  Aproto_BuildFrame(uint8_t seq, uint8_t cmd,
                           const uint8_t* payload, uint8_t len,
                           uint8_t* out, uint8_t out_max);

/* ---- 定点编解码小工具（inline，协议与 API 共用） ---- */
static inline int16_t  Aproto_EncDeg10(float deg)   { return (int16_t)(deg * 10.0f + (deg >= 0 ? 0.5f : -0.5f)); }
static inline float    Aproto_DecDeg10(int16_t v)   { return v / 10.0f; }
static inline int16_t  Aproto_EncMm10(float mm)     { return (int16_t)(mm * 10.0f + (mm >= 0 ? 0.5f : -0.5f)); }
static inline float    Aproto_DecMm10(int16_t v)    { return v / 10.0f; }
static inline uint16_t Aproto_EncVel10(float v)     { return (uint16_t)(v * 10.0f + 0.5f); }
static inline float    Aproto_DecVel10(uint16_t v)  { return v / 10.0f; }

/* ---- 帧分发（主循环调用） ----
 * 将一条完整请求（解析后的部件）映射为 ArmApi 调用并生成响应帧。
 * seq/cmd/payload/len: 解析后的帧部件（串口状态机或 CAN 重组给出）；
 * rsp: 响应帧输出缓冲；返回响应帧长（0=无需响应）。
 * run_id 用于 PING 应答标识本次运行会话。 */
uint8_t  Aproto_Dispatch(uint8_t seq, uint8_t cmd,
                         const uint8_t* payload, uint8_t len,
                         uint8_t* rsp, uint8_t rsp_max, uint8_t run_id);

#endif /* __ARM_CONTROL_PROTOCOL_H */
