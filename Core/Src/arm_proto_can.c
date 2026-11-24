/* ============================================================
 * arm_proto_can.c - 二进制控制协议的 CAN 承载（实现）
 *
 * 机械臂在这里扮演 **CAN 从站**: 上位机把串口协议那套
 * (SEQ,CMD,LEN,PAYLOAD,CRC16) 逻辑流按 8 字节数据场分包发到
 * 0x100+node, 本模块重组 -> Aproto_Dispatch -> 执行 ArmApi_* ->
 * 把响应分回包发到 0x180+node。
 *
 * 除"请求-响应"之外还补了两件上位机必需的东西（2026-09-17）:
 *   - 0x280+node 周期状态帧: E-Stop/轨迹/流模式/示教标志 + 最近返回码,
 *     让主机不必轮询 GET_STATUS（每轮要读 6 轴舵机总线, 是重操作）。
 *   - 0x280+node 异步事件帧: E-Stop 触发 / 轨迹结束 / 流模式退出 等
 *     边沿事件立即上报, 主机不用靠猜。
 *
 * ARM_CAN_ENABLE=0: 桩模式 —— 与工程一起编译但不产生任何硬件动作。
 * ARM_CAN_ENABLE=1: 完整分包发送 / 重组接收 / 主循环分发。
 * ============================================================ */
#include "arm_proto_can.h"
#include "arm_can_config.h"
#include "arm_control_protocol.h"
#include "arm_control_api.h"
#include "arm_main_exports.h"
#include "arm_runlog.h"
#include "arm_link_cfg.h"   /* LinkCfg_TouchCan: auto 双链路门控(0924CE) */
#include "main.h"       /* HAL_GetTick（重组超时/心跳节拍用） */
#include <string.h>

static uint8_t s_node_id = 1;
static volatile uint32_t s_peer_frames = 0U;   /* 收到的"对端方向"帧数 (0x180+node) */
static volatile uint32_t s_rx_ok = 0U;         /* CRC 通过并已分发的帧数 (CAN 版 LinkProto_StreamCount) */
static CanProtoStats s_stats;

/* ---- 异步事件队列 (单生产者 ISR / 单消费者主循环 的无锁环) ----
 * ⚠ 生产者可能是 TIM2 中断里的示教录制(缓冲写满)或主循环的 E-Stop 检测,
 *   所以两者不能同时改同一个索引: head 只由生产者也只由生产, tail 亦然。
 *   满了丢**新**事件(而不是挤掉旧的): 旧事件往往更关键(E-Stop 不能被覆盖)。 */
#define ARM_CAN_EVT_QUEUE 4U
static volatile uint8_t s_evt_code[ARM_CAN_EVT_QUEUE];
static volatile uint8_t s_evt_param[ARM_CAN_EVT_QUEUE];
static volatile uint8_t s_evt_head = 0U;
static volatile uint8_t s_evt_tail = 0U;
static volatile uint32_t s_evt_drop = 0U;

static uint32_t s_status_last_ms = 0U;   /* 上次状态帧时刻 */
/* "总线对面确实有人"的闩锁: 收到过至少一条合法命令帧才置位。
 * 为什么要它 —— 见 CanProto_Service() 里的说明: 没有对端时主动发心跳
 * 会把 bxCAN 的发送错误计数(TEC)一路堆到 error-passive/bus-off。 */
static volatile uint8_t s_seen_peer = 0U;

/* error-passive 暂停计时（2026-09-18 0918G 修的自锁，见 CanProto_Service 抑制条件②）
 *
 * 病: 原来写的是 `if (tec >= 128) return;` 一个**永久**闸门。可 TEC 只能靠
 *     "发送成功"递减 ⇒ 自己停发就永远赚不到那次递减 ⇒ **永久自锁**。
 *     触发条件极其无害: 宿主程序正常退出(bus.shutdown) 后没人 ACK，臂还在按
 *     1 Hz 发状态帧，16 帧×8 = 128 ⇒ 从此永久静默，只能按 RESET 或 `e` 才回来。
 *     现场实测: `--monitor` 跑完退出，16 s 后 `tec=128 EPVF=1 lec=3`，状态帧停在 34 帧。
 * 药: 闸门保留(不让它继续往 bus-off 堆)，但只是**暂停** —— 每 ARM_CAN_TEC_PAUSE_MS
 *     放一帧出去试(该周期 0918G = 5000 ms, **0918H 起 = 500 ms**; 定义在 arm_proto_can.h)。
 *     有对端: 那一帧被 ACK ⇒ TEC 每次成功发送 -1 ⇒ 若干帧后回到 error-active ⇒ 恢复。
 *  📌 实测到的两条硬事实（2026-09-18 验收 0918G 时用 PASSIVE 旁听测出来的）:
 *     ① **error-passive 下的 ACK 错误不会让 TEC 再涨**(ISO 11898-1 的例外)。
 *        所以"没对端 ⇒ 涨到 255 ⇒ bus-off ⇒ ABOM 自动清零"这条**不成立** ——
 *        TEC 就停在 128~135 不动(实测 tec=134 恒定 25 s，期间约 5 次失败发送)。
 *        ⚠ 别再拿"等它 bus-off 自愈"当恢复手段：**唯一的自愈路径就是探路帧**。
 *     ② 恢复延迟 = (TEC-127) × ARM_CAN_TEC_PAUSE_MS（TEC>127 时每成功发一帧只 -1）。
 *        实测(0918G, 周期 5000 ms) TEC=134 ⇒ 恰好 7 帧 × 5 s ≈ 35 s 后回到 1 Hz
 *        (录到的到达序列: 5.0 5.0 5.0 5.0 5.0 5.0 5.0 然后 1.0 1.0 …)。
 *        ⇒ 0918H 把周期缩到 500 ms, 同一场景约 3.5 s 就恢复。
 *        ⇒ 对端回来"立刻"看不到 1 Hz 属正常，判据是**探路帧一定会到**，不是"立刻 1 Hz"。
 *        ⚠ 命令帧(回包)不受本闸门限制，随时应答；且收到对端命令帧会立刻解除暂停。
 * 另外收到对端命令帧时会立刻解除暂停(见 can_rx_handle)，不必干等这个窗口。
 * ⚠ ARM_CAN_TEC_PAUSE_MS 定义在 arm_proto_can.h（0918H 起 = 500 ms），此处不再重复定义。 */
static uint32_t s_tec_pause_ms = 0U;     /* 上次"试探性放出"的时刻 */
/* 收到对端命令帧 ⇒ 对端确实在 ⇒ 立刻解除暂停, 不必干等那 5 s 窗口。
 * 由 can_rx_handle() 置位, CanProto_Service() 消费后自动清。 */
static volatile uint8_t s_tec_resume = 0U;

/* ---- 驱动钩子 weak 默认（未连接硬件） ---- */
__weak uint8_t ArmCanDrv_HwInit(void) { return 0; }
__weak uint8_t ArmCanDrv_Tx(uint16_t can_id, const uint8_t* data, uint8_t len) {
    (void)can_id; (void)data; (void)len; return 0;
}
__weak uint8_t ArmCanDrv_Rx(uint16_t* can_id, uint8_t* data, uint8_t* len) {
    (void)can_id; (void)data; (void)len; return 0;
}
__weak uint16_t ArmCanDrv_GetBitrate(void) { return 0; }
__weak uint8_t  ArmCanDrv_SetBitrate(uint16_t kbps) { (void)kbps; return 0; }
__weak uint32_t ArmCanDrv_TxCount(void) { return 0; }
__weak uint32_t ArmCanDrv_RxDropCount(void) { return 0; }

/* ============================================================
 * 与承载无关的部分（桩模式也要有正确的行为: 计数/节点号可读）
 * ============================================================ */

void CanProto_SetNodeId(uint8_t node_id) {
    if (node_id < CAN_PROTO_NODE_MIN || node_id > CAN_PROTO_NODE_MAX) return;
    if (node_id == s_node_id) return;
    RLOG_I("CAN", "node id %d -> %d (cmd_id=0x%03X rsp_id=0x%03X evt_id=0x%03X)",
           (int)s_node_id, (int)node_id,
           (unsigned)(CAN_PROTO_ID_CMD_BASE + node_id),
           (unsigned)(CAN_PROTO_ID_RSP_BASE + node_id),
           (unsigned)(CAN_PROTO_ID_EVT_BASE + node_id));
    s_node_id = node_id;
}

uint8_t CanProto_NodeId(void) { return s_node_id; }

void CanProto_SendEvent(uint8_t code, uint8_t param) {
    uint8_t next = (uint8_t)((s_evt_head + 1U) % ARM_CAN_EVT_QUEUE);
    if (next == s_evt_tail) {   /* 满: 丢新事件, 计数备查 */
        s_evt_drop++;
        return;
    }
    s_evt_code[s_evt_head] = code;
    s_evt_param[s_evt_head] = param;
    s_evt_head = next;
}

void CanProto_GetStats(CanProtoStats* out) {
    if (!out) return;
    *out = s_stats;                                   /* 结构体整体取一次快照 */
    out->cmd_frames = s_rx_ok;
    out->rsp_frames = s_peer_frames;
    out->tx_frames  = ArmCanDrv_TxCount();
    out->rx_dropped = ArmCanDrv_RxDropCount();
}

void CanProto_ResetStats(void) {
    memset(&s_stats, 0, sizeof(s_stats));
}

#if ARM_CAN_ENABLE

/* ================= 启用模式：完整分包/重组 ================= */

/* 重组状态 */
static uint8_t  s_rx_active   = 0;
static uint8_t  s_rx_frames   = 0;    /* 总帧数 */
static uint8_t  s_rx_next     = 0;    /* 已见的最大帧号+1(仅作诊断) */
static uint8_t  s_rx_len      = 0;    /* 逻辑流应收长度 */
static uint8_t  s_rx_firstn   = 0;    /* 首帧在 d[4..] 里携带的字节数 */
static uint16_t s_rx_seen     = 0;    /* 位图: bit0=首帧, bit f=第 f 个续帧 */
static uint8_t  s_rx_buf[3 + APROTO_MAX_PAYLOAD + 2];  /* SEQ,CMD,LEN,PAYLOAD,CRC */
static uint32_t s_rx_tick     = 0;

#define CAN_REASM_TIMEOUT_MS  200

void CanProto_Init(uint8_t node_id) {
    if (node_id >= CAN_PROTO_NODE_MIN && node_id <= CAN_PROTO_NODE_MAX) {
        s_node_id = node_id;
    }
    s_rx_active = 0;
    s_status_last_ms = 0U;
    if (ArmCanDrv_HwInit()) {
        RLOG_I("CAN", "hw init ok, node=%d cmd=0x%03X rsp=0x%03X evt=0x%03X @%uk",
               s_node_id,
               CAN_PROTO_ID_CMD_BASE + s_node_id,
               CAN_PROTO_ID_RSP_BASE + s_node_id,
               CAN_PROTO_ID_EVT_BASE + s_node_id,
               (unsigned)ArmCanDrv_GetBitrate());
    } else {
        RLOG_W("CAN", "hw init FAILED (stub driver?)");
    }
}

/* 逻辑流分包发送。base_id 决定方向:
 *   CAN_PROTO_ID_CMD_BASE+node = 命令方向(主臂 -> 从臂)
 *   CAN_PROTO_ID_RSP_BASE+node = 响应方向(从臂 -> 主臂)
 *
 * ⚠ 2026-09-18 现场定案: **每帧之间必须 ArmCanDrv_TxDrain()**。
 *   bxCAN 只有 3 个发送邮箱。若把 4 帧一口气塞进去, F1→MB0/F2→MB1/F3→MB2 挂起后,
 *   F4 会抢走"最先空出来"的 MB0, 而硬件按"ID 相同则邮箱号小者先发"调度 ⇒
 *   实际总线顺序 = **F1, F4, F2, F3**。这在现场表现为"**单帧的 PING 通,
 *   4 帧的 GET_STATUS 永远超时**", 极容易被误判成协议/物理层故障。
 *   ⇒ 一帧发完(邮箱排空)再排下一帧, 顺序就与代码一致。
 *   (对端重组器同时已改为容忍乱序, 双保险。) */
static uint8_t can_send_packed(uint16_t base_id, uint8_t seq, uint8_t cmd,
                               const uint8_t* payload, uint8_t len) {
    uint8_t stream[3 + APROTO_MAX_PAYLOAD + 2];
    stream[0] = seq; stream[1] = cmd; stream[2] = len;
    for (uint8_t i = 0; i < len; i++) stream[3 + i] = payload[i];
    uint16_t crc = Aproto_Crc16(stream, (uint16_t)(3 + len));
    stream[3 + len]     = (uint8_t)(crc & 0xFF);
    stream[3 + len + 1] = (uint8_t)(crc >> 8);
    /* 首帧已独立携带 SEQ/CMD/LEN，d[4..] 只承载 PAYLOAD 与 CRC。 */
    uint8_t tail_total = (uint8_t)(len + 2U);
    uint8_t rest = (tail_total > 4U) ? (uint8_t)(tail_total - 4U) : 0U;
    uint8_t frames = (uint8_t)(1U + (rest + 6U) / 7U);
    if (frames > 15U) return 0U;

    uint8_t d[8];
    d[0] = (uint8_t)(0x80U | frames);
    d[1] = seq;
    d[2] = cmd;
    d[3] = len;
    uint8_t first_n = (tail_total < 4U) ? tail_total : 4U;
    for (uint8_t i = 0; i < first_n; i++) d[4U + i] = stream[3U + i];
    (void)ArmCanDrv_TxDrain(20U);          /* 排空后再发首帧, 保证与后续帧同序 */
    if (!ArmCanDrv_Tx(base_id, d,
                      (uint8_t)(4U + first_n))) return 0U;

    uint8_t off = first_n;
    for (uint8_t f = 1U; f < frames; f++) {
        uint8_t n = (uint8_t)(tail_total - off);
        if (n > 7U) n = 7U;
        d[0] = f;
        for (uint8_t i = 0; i < n; i++) d[1U + i] = stream[3U + off + i];
        (void)ArmCanDrv_TxDrain(20U);      /* ★ 关键: 上一帧真发完才排下一帧 */
        if (!ArmCanDrv_Tx(base_id, d,
                          (uint8_t)(1U + n))) return 0U;
        off = (uint8_t)(off + n);
    }
    return 1;
}

/* 响应方向 (从臂 -> 主臂): 协议层的回包/NACK 走这里 */
uint8_t CanProto_Send(uint8_t seq, uint8_t cmd,
                      const uint8_t* payload, uint8_t len) {
    return can_send_packed((uint16_t)(CAN_PROTO_ID_RSP_BASE + s_node_id),
                           seq, cmd, payload, len);
}

/* 命令方向 (主臂 -> 从臂): 主臂转发 0x13 流帧走这里 */
uint8_t CanProto_SendCmd(uint8_t seq, uint8_t cmd,
                         const uint8_t* payload, uint8_t len) {
    return can_send_packed((uint16_t)(CAN_PROTO_ID_CMD_BASE + s_node_id),
                           seq, cmd, payload, len);
}

uint32_t CanProto_PeerFrames(void) { return s_peer_frames; }
uint32_t CanProto_RxOkCount(void)  { return s_rx_ok; }

/* ---- 0x280+node: 周期状态帧 ---- */
static void can_send_status(void) {
    uint8_t d[8];
    uint8_t flags = ARM_CAN_FLAG_PROTO_RDY;
    uint8_t mf = ArmMain_Flags();
    uint32_t sec;

    if (mf & ARM_MAIN_FLAG_ESTOP)      flags |= ARM_CAN_FLAG_ESTOP;
    if (mf & ARM_MAIN_FLAG_TRAJ)       flags |= ARM_CAN_FLAG_TRAJ;
    if (mf & ARM_MAIN_FLAG_STREAM)     flags |= ARM_CAN_FLAG_STREAM;
    if (mf & ARM_MAIN_FLAG_CORR)       flags |= ARM_CAN_FLAG_CORR;
    if (mf & ARM_MAIN_FLAG_TEACH_REC)  flags |= ARM_CAN_FLAG_TEACH_REC;
    if (mf & ARM_MAIN_FLAG_TEACH_PLAY) flags |= ARM_CAN_FLAG_TEACH_PLAY;
    /* hw 位从驱动层实时取: bus-off 自动恢复期间会短暂为 0, 主机可据此
       区分"臂在忙" 和 "CAN 控制器挂了"。 */
    if (ArmCanDrv_GetBitrate() != 0U) flags |= ARM_CAN_FLAG_HW_OK;

    d[0] = ARM_CAN_EVT_TYPE_STATUS;
    d[1] = s_node_id;
    d[2] = flags;
    d[3] = s_stats.last_err;                 /* 最近一次命令返回码 */
    d[4] = (uint8_t)(s_rx_ok & 0xFFU);       /* 已执行命令计数低 16 位: */
    d[5] = (uint8_t)((s_rx_ok >> 8) & 0xFFU);/*   主机看它是否在推进 */
    sec = HAL_GetTick() / 1000U;
    d[6] = (uint8_t)(sec & 0xFFU);           /* 运行秒数低 16 位(约 18h 回绕) */
    d[7] = (uint8_t)((sec >> 8) & 0xFFU);

    if (ArmCanDrv_Tx((uint16_t)(CAN_PROTO_ID_EVT_BASE + s_node_id), d, 8U)) {
        s_stats.status_tx++;
    }
}

/* ---- 0x280+node: 异步事件帧 ---- */
static void can_send_event_frame(uint8_t code, uint8_t param) {
    uint8_t d[8];
    uint32_t t = HAL_GetTick();
    d[0] = ARM_CAN_EVT_TYPE_EVENT;
    d[1] = s_node_id;
    d[2] = code;
    d[3] = param;
    d[4] = (uint8_t)(t & 0xFFU);
    d[5] = (uint8_t)((t >> 8) & 0xFFU);
    d[6] = (uint8_t)((t >> 16) & 0xFFU);
    d[7] = (uint8_t)((t >> 24) & 0xFFU);
    if (ArmCanDrv_Tx((uint16_t)(CAN_PROTO_ID_EVT_BASE + s_node_id), d, 8U)) {
        s_stats.event_tx++;
    }
}

void CanProto_Service(void) {
    /* 1) 事件帧出队(先发事件: 时效性比心跳高) */
    while (s_evt_tail != s_evt_head) {
        uint8_t t = s_evt_tail;
        uint8_t code = s_evt_code[t];
        uint8_t param = s_evt_param[t];
        s_evt_tail = (uint8_t)((t + 1U) % ARM_CAN_EVT_QUEUE);
        can_send_event_frame(code, param);
    }

    /* 2) 周期状态帧
     * ⚠ 三条抑制条件(2026-09-17 补), 少一条都会在"没接对端"的台子上自伤:
     *   ① 从未收到过任何合法帧 ⇒ 对面根本没人。CAN 的发送错误计数(TEC)
     *      每帧无人 ACK 就 +8(AutoRetransmission 已关, 发一次就放弃),
     *      128 就到 error-passive、256 到 bus-off ⇒ ABOM 恢复后如此循环。
     *      台子上没插 CAN 线时每秒白发一帧毫无意义, 却会把 cn 的诊断读数
     *      刷成一片红, 让人误以为收发器坏了。**臂不主动向空气发帧**。
     *   ② 已在 error-passive 以上 ⇒ 先把周期帧停下来(台子上没必要每秒空发)。
     *      ⚠ 但只能是**暂停**, 不能是永久闸门 —— TEC 只能靠"发送成功"递减,
     *        永久停发 = 永久自锁(0918G 修的就是这个, 详见 s_tec_pause_ms 注释)。
     *        所以这里每 ARM_CAN_TEC_PAUSE_MS 放一帧出去试探, 保证一定能自愈。
     *      📌 注: "继续发就会继续喂 TEC" 这句对 **ACK 错误**并不成立(error-passive 下
     *        ACK 错误不增 TEC, 实测 tec 恒定不动)。停发在这里的理由是**别刷噪声**,
     *        自锁才是真风险 —— 所以闸门必须带重试窗口。
     *   ③ 对端来了命令后就一直发: 主机只要 PING 过一次, 之后只监听也能收到。 */
    {
        const ArmCanCfg* cfg = CanCfg_Get();
        uint32_t now = HAL_GetTick();
        uint8_t tec = 0U, rec = 0U, lec = 0U;

        if (cfg->hb_ms == 0U) return;
        if (!s_seen_peer) return;
        (void)ArmCanDrv_ErrorState(&tec, &rec, &lec);
        if (tec >= 128U) {
            /* error-passive: 暂停周期帧, 但按窗口放行一次"探路帧"。
             * 放行后不更新 s_status_last_ms, 让它照常走自己的节拍。 */
            if (!s_tec_resume &&
                (uint32_t)(now - s_tec_pause_ms) < ARM_CAN_TEC_PAUSE_MS) return;
            s_tec_resume = 0U;
            s_tec_pause_ms = now;
        }

        if (s_status_last_ms == 0U) { s_status_last_ms = now; return; }
        if ((uint32_t)(now - s_status_last_ms) < (uint32_t)cfg->hb_ms) return;
        s_status_last_ms = now;
        can_send_status();
    }
}

/* 从响应帧里取"这次命令的返回码"给状态帧用。
 * 响应帧布局: [0]0xAA [1]0x55 [2]SEQ [3]CMD [4]LEN [5..]PAYLOAD [CRC2]
 * ACK/NACK 的 PAYLOAD = {orig_cmd, err} ⇒ err 落在 rsp_frame[6]。 */
static void can_note_result(const uint8_t* rsp_frame, uint8_t rsp_len) {
    if (!rsp_frame || rsp_len < 7U) return;
    if (rsp_frame[3] == APROTO_RSP_ACK || rsp_frame[3] == APROTO_RSP_NACK) {
        if (rsp_frame[4] >= 2U) s_stats.last_err = rsp_frame[6];
    } else {
        s_stats.last_err = (uint8_t)ARM_API_OK;   /* 数据类响应 = 成功 */
    }
}

/* 收包重组（主循环上下文） */
static void can_rx_handle(uint16_t can_id, const uint8_t* d, uint8_t dlc) {
    /* 对端方向的帧 (0x180+node, 即从臂 -> 主臂的回包/NACK):
     * 只计数, 不参与重组 —— 否则会和命令方向的半截重组互相踩。
     * 主臂用这个数回答"从臂到底回了几帧"。 */
    if (can_id == (uint16_t)(CAN_PROTO_ID_RSP_BASE + s_node_id)) {
        s_peer_frames++;
        return;
    }
    if (can_id != (uint16_t)(CAN_PROTO_ID_CMD_BASE + s_node_id) || dlc < 1)
        return;

    /* 对面有人: 解锁周期状态帧(见 CanProto_Service 的抑制条件①)。
     * 顺手把节拍回拨一个周期, 让主机 PING 完立刻就能看到第一帧状态,
     * 而不是再干等一个 hb 周期才知道"连通了"。 */
    if (!s_seen_peer) {
        s_seen_peer = 1U;
        s_status_last_ms = HAL_GetTick() - (uint32_t)CanCfg_Get()->hb_ms;
        RLOG_I("CAN", "peer detected, status frames enabled");
    }
    /* 每收到一条对端命令帧就解除 TEC 暂停(0918G): 对端明明在, 没有理由还哑着。
     * 不放在上面那个"只进一次"的分支里 —— 自锁往往发生在 peer_seen 早就置位之后。 */
    s_tec_resume = 1U;

    if (d[0] & 0x80) {   /* 首帧 */
        uint8_t frames = d[0] & 0x0F;
        if (frames == 0 || dlc < 4) return;
        uint8_t payload_len = d[3];
        uint8_t tail_total;
        uint8_t expected_frames;
        if (payload_len > APROTO_MAX_PAYLOAD) {
            s_rx_active = 0U;
            return;
        }
        tail_total = (uint8_t)(payload_len + 2U); /* PAYLOAD + CRC */
        expected_frames = (uint8_t)(1U + ((tail_total > 4U ? tail_total - 4U : 0U) + 6U) / 7U);
        if (frames != expected_frames) {
            s_rx_active = 0U;
            s_stats.reasm_err++;
            return;
        }
        s_rx_active = 1U;
        s_rx_frames = frames;
        s_rx_next   = 1U;
        s_rx_len    = (uint8_t)(3U + tail_total);
        s_rx_firstn = (tail_total < 4U) ? tail_total : 4U;
        s_rx_seen   = 0x0001U;            /* bit0 = 首帧已到 */
        /* d[1..3]=SEQ,CMD,LEN; d[4..]=PAYLOAD 和 CRC 的开头。 */
        s_rx_buf[0] = d[1];
        s_rx_buf[1] = d[2];
        s_rx_buf[2] = payload_len;
        {
            uint8_t n = (dlc > 4U) ? (uint8_t)(dlc - 4U) : 0U;
            if (n > 4U) n = 4U;
            if (n != s_rx_firstn) {       /* 首帧长度必须与 LEN 自洽 */
                s_rx_active = 0U;
                s_stats.reasm_err++;
                return;
            }
            for (uint8_t i = 0U; i < n; i++) s_rx_buf[3U + i] = d[4U + i];
        }
        s_rx_tick = HAL_GetTick();
    } else if (s_rx_active && d[0] >= 1U && d[0] < s_rx_frames) {
        /* 续帧。⚠ 这里**不能要求帧号严格递增** —— 详见文件末尾「帧序」注释:
         * 3 个发送邮箱会让 4 帧以上的流乱序到达(F1,F4,F2,F3), 严格递增的判定
         * 会把整条流当"乱序"丢掉, 症状是"1 帧的 PING 通、4 帧的 GET_STATUS 永远超时"。
         * 现在按**帧号定偏移**摆放, 收齐位图后再校验 CRC ⇒ 顺序不再影响正确性。 */
        uint8_t  f   = d[0];
        uint8_t  n   = (dlc > 1U) ? (uint8_t)(dlc - 1U) : 0U;
        uint16_t off = (uint16_t)(3U + s_rx_firstn + (uint16_t)((f - 1U) * 7U));
        if (off >= sizeof(s_rx_buf)) {
            n = 0U;
        } else if ((uint16_t)(off + n) > sizeof(s_rx_buf)) {
            n = (uint8_t)(sizeof(s_rx_buf) - off);
        }
        for (uint8_t i = 0U; i < n; i++) s_rx_buf[off + i] = d[1 + i];
        s_rx_seen |= (uint16_t)(1U << f);
        if ((uint8_t)(f + 1U) > s_rx_next) s_rx_next = (uint8_t)(f + 1U);
        s_rx_tick = HAL_GetTick();
    } else {
        s_rx_active = 0;   /* 帧号越界(不是本流的续帧)，丢弃重组 */
        s_stats.reasm_err++;
        return;
    }

    /* 完整性判定: 首帧 + 全部续帧都到齐(**与到达顺序无关**), 流长 = 3 + LEN + 2
     * ⚠ 这里**不要**再按"序号严格递增"判 —— bxCAN 有 3 个发送邮箱, 第 4 帧会抢先,
     *   到达顺序可以是 F1,F4,F2,F3。用"帧号位图收齐"才对, 也天然容忍重复帧。
     *   (曾经有过一个 `expect = 3+LEN+2` 的变量, 改成位图后就没用了, 已删。) */
    uint16_t all_bits = (uint16_t)((1U << s_rx_frames) - 1U);
    if (s_rx_active && (s_rx_seen & all_bits) == all_bits) {
        s_rx_active = 0;
        uint8_t seq = s_rx_buf[0], cmd = s_rx_buf[1], len = s_rx_buf[2];
        if (len > APROTO_MAX_PAYLOAD) {
            RLOG_W("CAN", "payload len overflow: %d", len);
            return;
        }
        uint16_t crc_got = (uint16_t)s_rx_buf[3 + len] |
                           ((uint16_t)s_rx_buf[3 + len + 1] << 8);
        uint16_t crc_cal = Aproto_Crc16(s_rx_buf, (uint16_t)(3 + len));
        if (crc_got != crc_cal) {
            uint8_t p[2] = { cmd, ARM_API_ERR_CRC };
            CanProto_Send(seq, APROTO_RSP_NACK, p, 2);
            s_stats.crc_err++;
            s_stats.last_err = (uint8_t)ARM_API_ERR_CRC;
            RLOG_W("CAN", "CRC err cmd=0x%02X", cmd);
            return;
        }
        s_rx_ok++;   /* CRC 通过: 这一帧真的可用 (从臂看它以 30/s 涨 = 流帧在到) */
        LinkCfg_TouchCan();   /* auto 门控: 合法包授/刷新 CAN active(0924CE) */
        /* 分发并回包 */
        uint8_t rsp_pl[APROTO_MAX_PAYLOAD];
        /* 借道串口组帧函数生成响应逻辑负载过于耦合，这里直接复用
           Aproto_Dispatch 的串口帧输出再剥离帧头/CRC。 */
        uint8_t rsp_frame[APROTO_MAX_FRAME];
        uint8_t rsp_len = Aproto_Dispatch(seq, cmd, &s_rx_buf[3], len,
                                          rsp_frame, sizeof(rsp_frame),
                                          RunLog_RunId());
        can_note_result(rsp_frame, rsp_len);
        if (rsp_len >= 7) {  /* AA 55 SEQ CMD LEN ... CRC CRC */
            uint8_t rlen = rsp_frame[4];
            if (rlen <= APROTO_MAX_PAYLOAD) {
                memcpy(rsp_pl, &rsp_frame[5], rlen);
                CanProto_Send(rsp_frame[2], rsp_frame[3], rsp_pl, rlen);
            }
        }
    }
}

void CanProto_Poll(void) {
    uint16_t can_id;
    uint8_t  d[8], dlc;
    uint8_t  guard = 8;   /* 单次轮询最多处理 8 帧，防主循环饿死 */
    while (guard-- && ArmCanDrv_Rx(&can_id, d, &dlc)) {
        can_rx_handle(can_id, d, dlc);
    }
    /* 重组超时复位 */
    if (s_rx_active &&
        (uint32_t)(HAL_GetTick() - s_rx_tick) > CAN_REASM_TIMEOUT_MS) {
        s_rx_active = 0;
        s_stats.reasm_err++;
        RLOG_W("CAN", "reassembly timeout, reset");
    }
}

#else  /* ================= 桩模式 ================= */

void CanProto_Init(uint8_t node_id) {
    if (node_id >= CAN_PROTO_NODE_MIN && node_id <= CAN_PROTO_NODE_MAX) {
        s_node_id = node_id;
    }
    /* 桩模式：静默，不打印（避免每次启动误导用户） */
}

void CanProto_Poll(void)    { /* 空操作 */ }
void CanProto_Service(void) { /* 空操作 */ }

uint8_t CanProto_Send(uint8_t seq, uint8_t cmd,
                      const uint8_t* payload, uint8_t len) {
    (void)seq; (void)cmd; (void)payload; (void)len;
    return 0;
}

uint8_t CanProto_SendCmd(uint8_t seq, uint8_t cmd,
                         const uint8_t* payload, uint8_t len) {
    (void)seq; (void)cmd; (void)payload; (void)len;
    return 0;
}

uint32_t CanProto_PeerFrames(void) { return s_peer_frames; }
uint32_t CanProto_RxOkCount(void)  { return s_rx_ok; }

#endif /* ARM_CAN_ENABLE */

/* 是否已探测到对端(收到过发往本节点的帧)。0 ⇒ 周期状态帧停发中
 * （避免无 ACK 白刷 TEC, 见 CanProto_Service）。 */
uint8_t CanProto_PeerSeen(void) { return s_seen_peer; }

/* 事件队列是否有积压(诊断用) */
uint32_t CanProto_EventDropped(void) { return s_evt_drop; }
