/* ============================================================
 * arm_control_protocol.c - 二进制控制协议核心（实现）
 *
 * 纯逻辑模块：帧解析状态机 / CRC / 编解码 / 命令分发。
 * 串口 (arm_proto_serial.c) 与 CAN (arm_proto_can.c) 共用。
 * 仅 Aproto_Dispatch 会调用 ArmApi / ArmParams / RunLog
 * （主循环上下文，允许总线通信与打印）。
 * ============================================================ */
#include "arm_control_protocol.h"
#include "arm_control_api.h"
#include "arm_params.h"
#include "arm_runlog.h"
#include "arm_main_exports.h"   /* 文本透传: ArmMain_TextGate/ExecText/TextRead... */

/* ---- 帧间字节超时：超过则状态机复位（防半截帧卡死） ---- */
#define APROTO_INTERBYTE_TIMEOUT_MS  200

/* ================= CRC-16/CCITT-FALSE ================= */
uint16_t Aproto_Crc16(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* ================= 解析状态机 ================= */
void Aproto_ParserReset(AprotoParser* p) {
    p->state = 0;
    p->idx   = 0;
}

uint8_t Aproto_ParserFeed(AprotoParser* p, uint8_t byte, uint32_t now_ms) {
    /* 帧间超时保护：非空闲状态且距上一字节过久 -> 复位重来 */
    if (p->state != 0 &&
        (uint32_t)(now_ms - p->last_tick) > APROTO_INTERBYTE_TIMEOUT_MS) {
        Aproto_ParserReset(p);
    }

    switch (p->state) {
    case 0: /* 等帧头 1 */
        if (byte == APROTO_HEAD0) {
            p->last_tick = now_ms;
            p->state = 1;
            return 1;
        }
        return 0;   /* 非协议字节，交给文本控制台 */

    case 1: /* 等帧头 2 */
        p->last_tick = now_ms;
        if (byte == APROTO_HEAD1) { p->state = 2; return 1; }
        if (byte == APROTO_HEAD0) { return 1; }   /* 0xAA 0xAA: 保持等 0x55 */
        Aproto_ParserReset(p);
        return 1;   /* 0xAA 后的杂散字节，丢弃（文本命令首字符不可能是 0xAA） */

    case 2: p->seq = byte; p->last_tick = now_ms; p->state = 3; return 1;
    case 3: p->cmd = byte; p->last_tick = now_ms; p->state = 4; return 1;

    case 4: /* LEN */
        p->last_tick = now_ms;
        if (byte > APROTO_MAX_PAYLOAD) { Aproto_ParserReset(p); return 1; }
        p->len = byte;
        p->idx = 0;
        p->state = (byte > 0) ? 5 : 6;
        return 1;

    case 5: /* PAYLOAD */
        p->payload[p->idx++] = byte;
        p->last_tick = now_ms;
        if (p->idx >= p->len) p->state = 6;
        return 1;

    case 6: /* CRC 低字节 */
        p->crc_lo = byte;
        p->last_tick = now_ms;
        p->state = 7;
        return 1;

    case 7: { /* CRC 高字节 -> 校验（范围 SEQ..PAYLOAD 一次性计算） */
        uint16_t expect = (uint16_t)p->crc_lo | ((uint16_t)byte << 8);
        uint8_t tmp[3 + APROTO_MAX_PAYLOAD];
        tmp[0] = p->seq; tmp[1] = p->cmd; tmp[2] = p->len;
        for (uint8_t i = 0; i < p->len; i++) tmp[3 + i] = p->payload[i];
        uint16_t crc = Aproto_Crc16(tmp, (uint16_t)(3 + p->len));
        Aproto_ParserReset(p);
        return (crc == expect) ? 2 : 3;
    }

    default:
        Aproto_ParserReset(p);
        return 1;
    }
}

/* ================= 组帧 ================= */
uint8_t Aproto_BuildFrame(uint8_t seq, uint8_t cmd,
                          const uint8_t* payload, uint8_t len,
                          uint8_t* out, uint8_t out_max) {
    uint8_t total = (uint8_t)(5 + len + 2);
    if (out_max < total || len > APROTO_MAX_PAYLOAD) return 0;
    out[0] = APROTO_HEAD0;
    out[1] = APROTO_HEAD1;
    out[2] = seq;
    out[3] = cmd;
    out[4] = len;
    for (uint8_t i = 0; i < len; i++) out[5 + i] = payload[i];
    uint16_t crc = Aproto_Crc16(&out[2], (uint16_t)(3 + len));
    out[5 + len]     = (uint8_t)(crc & 0xFF);
    out[5 + len + 1] = (uint8_t)(crc >> 8);
    return total;
}

/* ================= 定点负载编解码（小端） ================= */
static int16_t  rd_i16(const uint8_t* p) { return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }
static uint16_t rd_u16(const uint8_t* p) { return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }
static uint32_t rd_u32(const uint8_t* p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                                                  ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static int32_t  rd_i32(const uint8_t* p) { return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                                                            ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24)); }
static void wr_i16(uint8_t* p, int16_t v)  { p[0] = (uint8_t)v; p[1] = (uint8_t)((uint16_t)v >> 8); }
static void wr_u16(uint8_t* p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void wr_i32(uint8_t* p, int32_t v)  { uint32_t u = (uint32_t)v;
                                             p[0]=(uint8_t)u; p[1]=(uint8_t)(u>>8);
                                             p[2]=(uint8_t)(u>>16); p[3]=(uint8_t)(u>>24); }

/* ================= 文本透传应答分页 =================
 * 应答负载统一 6 字节头 + 文本：
 *   u16 total(本次捕获总长), u8 flags(bit0=输出被截断), u16 off, u8 n, char text[n]
 * 6 + 58 = 64 = APROTO_MAX_PAYLOAD 的上限 ⇒ 每页正文最多 58 B。
 * ⚠ 截断必须**显式置位**而不是靠"长度看着不对"：读侧宁可看到 flags，也不许
 *   把半截输出当成完整输出(工程红线：读失败/截断都不许静默兜底)。 */
#define APROTO_TEXT_CHUNK  58U

/* 上一次 TEXT_EXEC 是否被截断 —— TEXT_FETCH 翻页时要接着告诉上位机。 */
static uint8_t s_text_trunc = 0U;

/* 用捕获缓冲(最近一次 TEXT_EXEC 的输出)铺一页应答 */
static uint8_t proto_text_page(uint8_t seq, uint8_t rsp_cmd, uint16_t off,
                               uint8_t* rsp, uint8_t rsp_max) {
    uint8_t  pbuf[APROTO_MAX_PAYLOAD];
    uint16_t total = ArmMain_TextLength();
    if (off > total) off = total;
    uint8_t n = ArmMain_TextRead(off, APROTO_TEXT_CHUNK, (char*)&pbuf[6]);
    wr_u16(&pbuf[0], total);
    pbuf[2] = s_text_trunc ? 0x01U : 0x00U;
    wr_u16(&pbuf[3], off);
    pbuf[5] = n;
    return Aproto_BuildFrame(seq, rsp_cmd, pbuf, (uint8_t)(6U + n), rsp, rsp_max);
}

/* 用一段常量文本铺一页应答(闸门拒绝时把**原因**回给上位机, 而不是干瘪的 err) */
static uint8_t proto_text_lit(uint8_t seq, uint8_t rsp_cmd, const char* s,
                              uint8_t flags, uint8_t* rsp, uint8_t rsp_max) {
    uint8_t pbuf[APROTO_MAX_PAYLOAD];
    uint8_t n = 0U;
    while (n < APROTO_TEXT_CHUNK && s[n] != '\0') {
        pbuf[6U + n] = (uint8_t)s[n];
        n++;
    }
    wr_u16(&pbuf[0], n);
    pbuf[2] = flags;
    wr_u16(&pbuf[3], 0U);
    pbuf[5] = n;
    return Aproto_BuildFrame(seq, rsp_cmd, pbuf, (uint8_t)(6U + n), rsp, rsp_max);
}

/* 控制类命令统一收尾：ACK/NACK 负载 = {原 CMD, 错误码} */
static uint8_t finish_ack(uint8_t seq, uint8_t orig_cmd, int err,
                          uint8_t* rsp, uint8_t rsp_max) {
    uint8_t p[2] = { orig_cmd, (uint8_t)err };
    if (err != ARM_API_OK) {
        /* 限流: 从臂拒 30Hz 主从流帧时会每帧调一次, 而 RunLog 是
         * HAL_UART_Transmit 阻塞写控制台(~7ms/行 @115200) —— 不抑制
         * 会把主循环拖慢到丢帧。只打"每 200 次里的第 1 次", 既能看见
         * 故障在刷、又能看清 cmd/err, 且不占链路时间。
         * (精确统计看 `lk`: last NACK sent err=N / rejects 计数。) */
        static uint16_t nack_seen = 0;
        if ((++nack_seen % 200U) == 1U) {
            RLOG_W("PROTO", "cmd=0x%02X -> NACK err=%d (1/200 shown)",
                   orig_cmd, err);
        }
    }
    return Aproto_BuildFrame(seq,
                             (err == ARM_API_OK) ? APROTO_RSP_ACK : APROTO_RSP_NACK,
                             p, 2, rsp, rsp_max);
}

/* ================= 命令分发（主循环调用） ================= */
uint8_t Aproto_Dispatch(uint8_t seq, uint8_t cmd,
                        const uint8_t* payload, uint8_t len,
                        uint8_t* rsp, uint8_t rsp_max, uint8_t run_id) {
    uint8_t pbuf[APROTO_MAX_PAYLOAD];

    RLOG_D("PROTO", "cmd=0x%02X seq=%d len=%d", cmd, seq, len);

    /* ===== 回包帧不是命令, 一律忽略 =====
     * ACK(0x7F)/NACK(0x7E) 与"请求码|0x80"的查询响应(0x81/0x82/0xA1/0xC0/
     * 0xC2) 属于响应方向。若不挡掉, 它们会落到下面的 default 分支 ->
     * 回一个 NACK -> 对端收到 NACK 又当未知命令… 两端互相回形成死循环
     * (100ms 限流下约 10Hz), 把 reject 统计刷成假告警、掩盖真正故障。
     * 命令码全部 < 0x80, 故用 cmd >= 0x80 一次覆盖所有查询响应。 */
    if (cmd >= 0x80U || cmd == APROTO_RSP_ACK || cmd == APROTO_RSP_NACK) {
        return 0;   /* 无需响应 */
    }

    switch (cmd) {
    /* ---------- 系统 ---------- */
    case APROTO_CMD_PING:
        pbuf[0] = APROTO_VERSION;
        pbuf[1] = run_id;
        return Aproto_BuildFrame(seq, 0x81, pbuf, 2, rsp, rsp_max);

    case APROTO_CMD_GET_STATUS: {
        ArmApiStatus st;
        ArmApi_GetStatus(&st);
        for (uint8_t j = 0; j < 6; j++) {
            int16_t a10 = (st.joint_deg[j] < 0.0f) ? (int16_t)-32768
                                                   : Aproto_EncDeg10(st.joint_deg[j]);
            wr_i16(&pbuf[j * 2], a10);
        }
        wr_i16(&pbuf[12], Aproto_EncMm10(st.x));
        wr_i16(&pbuf[14], Aproto_EncMm10(st.y));
        wr_i16(&pbuf[16], Aproto_EncMm10(st.z));
        wr_i16(&pbuf[18], Aproto_EncDeg10(st.pitch_deg));
        pbuf[20] = (uint8_t)((st.estop ? 1 : 0) | (st.traj_active ? 2 : 0) |
                             (st.corr_active ? 4 : 0) | (st.read_ok ? 8 : 0));
        return Aproto_BuildFrame(seq, 0x82, pbuf, 21, rsp, rsp_max);
    }

    case APROTO_CMD_LOG_SET_LEVEL:
        if (len < 1) return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
        RunLog_SetLevel(payload[0]);
        return finish_ack(seq, cmd, ARM_API_OK, rsp, rsp_max);

    /* ---------- S 曲线 / 关节 ---------- */
    case APROTO_CMD_MOVE_JOINTS: {
        if (len < 16) return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
        float deg[6];
        for (uint8_t j = 0; j < 6; j++) deg[j] = Aproto_DecDeg10(rd_i16(&payload[j * 2]));
        float v = Aproto_DecVel10(rd_u16(&payload[12]));
        float a = Aproto_DecVel10(rd_u16(&payload[14]));
        return finish_ack(seq, cmd, ArmApi_MoveJoints(deg, v, a), rsp, rsp_max);
    }

    case APROTO_CMD_STOP:
        ArmApi_Stop();
        return finish_ack(seq, cmd, ARM_API_OK, rsp, rsp_max);

    case APROTO_CMD_SET_TORQUE:
        if (len < 2) return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
        return finish_ack(seq, cmd, ArmApi_SetTorque(payload[0], payload[1]), rsp, rsp_max);

    /* ---------- 主从直写 (实时遥操作跟随) ---------- */
    case APROTO_CMD_STREAM_JOINTS: {
        if (len < 12) return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
        float deg[6];
        for (uint8_t j = 0; j < 6; j++) deg[j] = Aproto_DecDeg10(rd_i16(&payload[j * 2]));
        return finish_ack(seq, cmd, ArmApi_StreamJoints(deg), rsp, rsp_max);
    }

    /* ---------- FK/IK-CCD ---------- */
    case APROTO_CMD_MOVE_CART: {
        if (len < 12) return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
        float x = Aproto_DecMm10(rd_i16(&payload[0]));
        float y = Aproto_DecMm10(rd_i16(&payload[2]));
        float z = Aproto_DecMm10(rd_i16(&payload[4]));
        float pitch = Aproto_DecDeg10(rd_i16(&payload[6]));
        float v = Aproto_DecVel10(rd_u16(&payload[8]));
        float a = Aproto_DecVel10(rd_u16(&payload[10]));
        return finish_ack(seq, cmd, ArmApi_MoveCartesian(x, y, z, pitch, v, a),
                          rsp, rsp_max);
    }

    case APROTO_CMD_FK_QUERY: {
        ArmApiStatus st;
        ArmApi_GetStatus(&st);
        wr_i16(&pbuf[0], Aproto_EncMm10(st.x));
        wr_i16(&pbuf[2], Aproto_EncMm10(st.y));
        wr_i16(&pbuf[4], Aproto_EncMm10(st.z));
        wr_i16(&pbuf[6], Aproto_EncDeg10(st.pitch_deg));
        for (uint8_t j = 0; j < 6; j++) {
            int16_t a10 = (st.joint_deg[j] < 0.0f) ? (int16_t)-32768
                                                   : Aproto_EncDeg10(st.joint_deg[j]);
            wr_i16(&pbuf[8 + j * 2], a10);
        }
        pbuf[20] = st.read_ok;
        return Aproto_BuildFrame(seq, 0xA1, pbuf, 21, rsp, rsp_max);
    }

    /* ---------- 重力补偿 / 自适应修正 ---------- */
    case APROTO_CMD_SET_GRAVITY: {
        if (len < 2) return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
        float k = rd_i16(&payload[0]) / 1000.0f;
        return finish_ack(seq, cmd, ArmApi_SetGravityComp(k), rsp, rsp_max);
    }

    case APROTO_CMD_SET_CORRECTION: {
        if (len < 4) return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
        float thr = rd_u16(&payload[1]) / 100.0f;
        return finish_ack(seq, cmd,
                          ArmApi_SetCorrection(payload[0], thr, payload[3]),
                          rsp, rsp_max);
    }

    /* ---------- 统一参数表 ---------- */
    case APROTO_CMD_PARAM_READ: {
        if (len < 1) return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
        float val = 0.0f;
        if (ArmParams_Get(payload[0], &val) != 0) {
            return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
        }
        pbuf[0] = payload[0];
        wr_i32(&pbuf[1], (int32_t)(val * 1000.0f + (val >= 0 ? 0.5f : -0.5f)));
        return Aproto_BuildFrame(seq, 0xC0, pbuf, 5, rsp, rsp_max);
    }

    case APROTO_CMD_PARAM_WRITE: {
        if (len < 5) return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
        float val = rd_i32(&payload[1]) / 1000.0f;
        int rc = ArmParams_Set(payload[0], val);
        if (rc == 0) {
            float applied = 0.0f;
            ArmParams_Get(payload[0], &applied);
            RLOG_I("PROTO", "param #%d = %d (x1000)", payload[0],
                   (int)(applied * 1000.0f));
        }
        return finish_ack(seq, cmd,
                          (rc == 0) ? ARM_API_OK : ARM_API_ERR_PARAM, rsp, rsp_max);
    }

    case APROTO_CMD_PARAM_COUNT:
        pbuf[0] = ArmParams_Count();
        return Aproto_BuildFrame(seq, 0xC2, pbuf, 1, rsp, rsp_max);

    /* ============================================================
     * 2026-09-23 扩展：让网口覆盖文本控制台能力
     * 设计基准 docs/网口指令扩展设计_2026-09-23.md
     * 四条铁律：①命令码 < 0x80  ②单帧 ≤ 64B ⇒ 大数组走分页
     *           ③危险命令带 magic  ④一律经 ArmApi_*（本文件不碰 HAL）
     *
     * ⚠ pbuf 只有 APROTO_MAX_PAYLOAD(64) 字节。每个分支的字节数都在
     *   注释里标出来了 —— 加字段前先算账，超了就是静默内存越界。
     * ============================================================ */

    /* ---------- 只读：诊断与遥测 ---------- */

    case APROTO_CMD_GET_JOINT_DETAIL: {
        if (len < 1) return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
        uint8_t j = payload[0];

        if (j == 0xFFU) {
            /* 六轴: 1(n) + 1(joint0) + 6×10 = 62B <= 64 ✓ */
            ArmApiJointDetail det[6];
            ArmApi_GetJointDetailAll(det);
            uint8_t o = 0;
            pbuf[o++] = 6U;
            pbuf[o++] = 0U;
            for (uint8_t k = 0; k < 6U; k++) {
                wr_u16(&pbuf[o], det[k].pos_raw);  o = (uint8_t)(o + 2U);
                wr_i16(&pbuf[o], det[k].speed);    o = (uint8_t)(o + 2U);
                wr_i16(&pbuf[o], det[k].load);     o = (uint8_t)(o + 2U);
                pbuf[o++] = det[k].voltage;
                pbuf[o++] = det[k].temp;      /* 0xFF = 该字段丢读 */
                pbuf[o++] = det[k].hw_err;    /* 0xFF = 硬件错误读失败 */
                pbuf[o++] = det[k].ok;
            }
            return Aproto_BuildFrame(seq, 0x84, pbuf, o, rsp, rsp_max);
        }
        if (j > 5U) return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);

        /* 单轴: 结构与上面完全相同（n=1）—— 上位机可以只写一份解包代码 */
        ArmApiJointDetail d;
        (void)ArmApi_GetJointDetail(j, &d);
        uint8_t o = 0;
        pbuf[o++] = 1U;
        pbuf[o++] = j;
        wr_u16(&pbuf[o], d.pos_raw);  o = (uint8_t)(o + 2U);
        wr_i16(&pbuf[o], d.speed);    o = (uint8_t)(o + 2U);
        wr_i16(&pbuf[o], d.load);     o = (uint8_t)(o + 2U);
        pbuf[o++] = d.voltage;
        pbuf[o++] = d.temp;
        pbuf[o++] = d.hw_err;
        pbuf[o++] = d.ok;
        return Aproto_BuildFrame(seq, 0x84, pbuf, o, rsp, rsp_max);
    }

    case APROTO_CMD_GET_LIMITS: {
        ArmApiLimits L;
        if (ArmApi_GetLimits(&L) != ARM_API_OK)
            return finish_ack(seq, cmd, ARM_API_ERR_READ, rsp, rsp_max);
        /* 6 × 8B = 48B <= 64 ✓
         * 未校准判据: zero_raw[k] == 0（固件此时回落 180°/raw 2048） */
        uint8_t o = 0;
        for (uint8_t k = 0; k < 6U; k++) {
            wr_u16(&pbuf[o], L.min_raw[k]);  o = (uint8_t)(o + 2U);
            wr_u16(&pbuf[o], L.pos_raw[k]);  o = (uint8_t)(o + 2U);
            wr_u16(&pbuf[o], L.max_raw[k]);  o = (uint8_t)(o + 2U);
            wr_u16(&pbuf[o], L.zero_raw[k]); o = (uint8_t)(o + 2U);
        }
        return Aproto_BuildFrame(seq, 0x85, pbuf, o, rsp, rsp_max);
    }

    case APROTO_CMD_GET_INFO: {
        ArmApiInfo info;
        ArmApi_GetInfo(&info);
        /* 自己算长度, 不引 <string.h>（本文件刻意保持零 libc 依赖） */
        uint8_t tl = 0;
        while (tl < 16U && info.fw_tag[tl] != '\0') tl++;
        uint8_t o = 0;
        pbuf[o++] = tl;
        for (uint8_t k = 0; k < 16U; k++)
            pbuf[o++] = (k < tl) ? (uint8_t)info.fw_tag[k] : 0U;
        pbuf[o++] = info.build;
        pbuf[o++] = info.link_mode;
        pbuf[o++] = info.n_params;
        /* 1 + 16 + 3 = 20B <= 64 ✓ */
        return Aproto_BuildFrame(seq, 0x87, pbuf, o, rsp, rsp_max);
    }

    case APROTO_CMD_GET_FLAGS: {
        uint8_t f = 0;
        (void)ArmApi_GetFlags(&f);
        pbuf[0] = f;         /* ARM_MAIN_FLAG_*: E-Stop 位在 0x01 */
        pbuf[1] = run_id;
        /* 只读变量、不碰总线 ⇒ 可以按 10~50Hz 轮询做状态刷新 */
        return Aproto_BuildFrame(seq, 0x8A, pbuf, 2, rsp, rsp_max);
    }

    /* ---------- 运动：单轴与配置 ---------- */

    case APROTO_CMD_MOVE_JOINT_ONE: {
        /* u8 joint + i16 deg + u16 v + u16 a = 7B */
        if (len < 7) return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
        uint8_t j = payload[0];
        float deg = Aproto_DecDeg10(rd_i16(&payload[1]));
        float v   = Aproto_DecVel10(rd_u16(&payload[3]));
        float a   = Aproto_DecVel10(rd_u16(&payload[5]));
        return finish_ack(seq, cmd, ArmApi_MoveJointOne(j, deg, v, a),
                          rsp, rsp_max);
    }

    case APROTO_CMD_SET_BUS_MODE:
        if (len < 1) return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
        return finish_ack(seq, cmd, ArmApi_SetBusMode(payload[0]), rsp, rsp_max);

    case APROTO_CMD_SERVO_INIT:
        if (len < 1) return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
        return finish_ack(seq, cmd, ArmApi_ServoInit(payload[0]), rsp, rsp_max);

    /* ---------- 运动学：只解不算 ---------- */

    case APROTO_CMD_IK_QUERY: {
        /* 4 × i16 = 8B */
        if (len < 8) return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
        float x = Aproto_DecMm10(rd_i16(&payload[0]));
        float y = Aproto_DecMm10(rd_i16(&payload[2]));
        float z = Aproto_DecMm10(rd_i16(&payload[4]));
        float p = Aproto_DecDeg10(rd_i16(&payload[6]));

        float deg[6];
        int rc = ArmApi_IkQuery(x, y, z, p, deg);
        /* 不可达/超限位/FK 超差都回 NACK，错误码区分原因 —— 这条命令
         * 的价值正是"先问能不能到，再决定动不动"。 */
        if (rc != ARM_API_OK) return finish_ack(seq, cmd, rc, rsp, rsp_max);

        for (uint8_t k = 0; k < 6U; k++)
            wr_i16(&pbuf[k * 2], Aproto_EncDeg10(deg[k]));
        return Aproto_BuildFrame(seq, 0xA2, pbuf, 12, rsp, rsp_max);
    }

    /* ---------- 补偿：补读侧 ---------- */

    case APROTO_CMD_GET_GRAVITY: {
        float k = 0.0f;
        if (ArmApi_GetGravity(&k) != ARM_API_OK)
            return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
        wr_i16(pbuf, (int16_t)(k * 1000.0f + (k >= 0.0f ? 0.5f : -0.5f)));
        return Aproto_BuildFrame(seq, 0xB2, pbuf, 2, rsp, rsp_max);
    }

    case APROTO_CMD_GET_CORRECTION: {
        uint8_t en = 0, rounds = 0, pend = 0;
        float thr = 0.0f;
        (void)ArmApi_GetCorrection(&en, &thr, &rounds, &pend);
        pbuf[0] = en;
        wr_u16(&pbuf[1], (uint16_t)(thr * 100.0f + 0.5f));
        pbuf[3] = rounds;
        pbuf[4] = pend;
        return Aproto_BuildFrame(seq, 0xB3, pbuf, 5, rsp, rsp_max);
    }

    case APROTO_CMD_GET_PID: {
        if (len < 1) return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
        uint8_t gp = 0, gi = 0, gd = 0, ee = 0;
        int rc = ArmApi_GetPid(payload[0], &gp, &gi, &gd, &ee);
        if (rc != ARM_API_OK) return finish_ack(seq, cmd, rc, rsp, rsp_max);
        pbuf[0] = gp;   /* 协议顺序固定 p/i/d —— API 层已处理底层 (p,d,i) 形参 */
        pbuf[1] = gi;
        pbuf[2] = gd;
        pbuf[3] = ee;   /* 1 = 值读自舵机 EEPROM（权威） */
        return Aproto_BuildFrame(seq, 0xB6, pbuf, 4, rsp, rsp_max);
    }

    /* ---------- 参数表：只列元数据，值用 PARAM_READ 单取 ---------- */

    case APROTO_CMD_PARAM_LIST: {
        if (len < 1) return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
        ArmApiParamBrief br[6];
        uint8_t n = ArmApi_ParamList(payload[0], br, 6U);
        pbuf[0] = ArmParams_Count();
        pbuf[1] = n;
        uint8_t o = 2U;
        for (uint8_t k = 0; k < n; k++) {
            pbuf[o++] = br[k].id;
            pbuf[o++] = br[k].type;
            /* vmin/vmax 用 i32×1000：参数范围里有超过 i16 的值，
             * 压成 i16 会静默截断。10B/条 × 6 = 62B（含 2B 头）。 */
            wr_i32(&pbuf[o], (int32_t)(br[k].vmin * 1000.0f +
                                       (br[k].vmin >= 0.0f ? 0.5f : -0.5f)));
            o = (uint8_t)(o + 4U);
            wr_i32(&pbuf[o], (int32_t)(br[k].vmax * 1000.0f +
                                       (br[k].vmax >= 0.0f ? 0.5f : -0.5f)));
            o = (uint8_t)(o + 4U);
        }
        return Aproto_BuildFrame(seq, 0xC4, pbuf, o, rsp, rsp_max);
    }

    /* ---------- 示教 / 轨迹存储 ---------- */

    case APROTO_CMD_TEACH_LIST: {
        if (len < 1) return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
        ArmApiTeachInfo ti;
        if (ArmApi_TeachInfo(payload[0], &ti) != ARM_API_OK)
            return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
        uint8_t o = 0;
        pbuf[o++] = (uint8_t)ArmApi_TeachCount();
        pbuf[o++] = payload[0];
        pbuf[o++] = ti.valid;              /* 0 = tombstone（已删待 compact） */
        for (uint8_t k = 0; k < 12U; k++) pbuf[o++] = (uint8_t)ti.name[k];
        /* samples 用 u32 小端手工铺（本文件的 wr_i32 位模式等价） */
        wr_i32(&pbuf[o], (int32_t)ti.samples);
        o = (uint8_t)(o + 4U);
        /* 3 + 12 + 4 = 19B <= 64 ✓ */
        return Aproto_BuildFrame(seq, 0xD0, pbuf, o, rsp, rsp_max);
    }

    case APROTO_CMD_TEACH_READ: {
        /* u8 idx + u16 off + u8 n = 4B */
        if (len < 4) return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
        uint8_t  idx = payload[0];
        uint32_t off = rd_u16(&payload[1]);
        uint8_t  n   = payload[3];
        if (n > 5U) n = 5U;      /* 5 × 12B = 60B, 加 1B 计数 = 61 <= 64 ✓ */

        uint16_t smp[5][6];
        uint32_t got = ArmApi_TeachRead(idx, off, n, smp);

        uint8_t o = 0;
        pbuf[o++] = (uint8_t)got;      /* 0 = off 越界 = 正常读完 */
        for (uint32_t k = 0; k < got; k++) {
            for (uint8_t j = 0; j < 6U; j++) {
                wr_u16(&pbuf[o], smp[k][j]);
                o = (uint8_t)(o + 2U);
            }
        }
        return Aproto_BuildFrame(seq, 0xD1, pbuf, o, rsp, rsp_max);
    }

    case APROTO_CMD_TEACH_DELETE:
        if (len < 1) return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
        return finish_ack(seq, cmd, ArmApi_TeachDelete(payload[0]), rsp, rsp_max);

    case APROTO_CMD_TEACH_COMPACT:
        return finish_ack(seq, cmd, ArmApi_TeachCompact(), rsp, rsp_max);

    case APROTO_CMD_TEACH_TOTAL: {
        uint8_t  nf = 0;
        uint32_t tot = 0, fre = 0;
        ArmApi_TeachTotals(&nf, &tot, &fre);
        pbuf[0] = nf;
        pbuf[1] = 32U;    /* = TSTORE_MAX_FILES，与 teach_store.h 保持一致 */
        wr_i32(&pbuf[2], (int32_t)tot);
        wr_i32(&pbuf[6], (int32_t)fre);
        return Aproto_BuildFrame(seq, 0xD5, pbuf, 10U, rsp, rsp_max);
    }

    /* ---------- 文本命令透传 (2026-09-23) ----------
     * 设计动机（见 arm_control_protocol.h 的 0x0B 说明）：控制台有 124 条命令，
     * 逐条加 opcode 既追不上以后新增的、又会让两套语义慢慢漂移。这一条把
     * **整个控制台**搬上网口，走的是同一条 ProcessCommand。 */

    case APROTO_CMD_TEXT_EXEC: {
        /* u32 magic + char text[](NUL 结尾) = 5+n B */
        if (len < 5U) return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
        if (rd_u32(&payload[0]) != APROTO_MAGIC)
            return finish_ack(seq, cmd, ARM_API_ERR_MAGIC, rsp, rsp_max);

        uint8_t tlen = (uint8_t)(len - 4U);
        if (tlen > 58U) tlen = 58U;          /* line[64]: 58 + NUL 仍有余量 */
        char line[64];
        for (uint8_t i = 0U; i < tlen; i++) line[i] = (char)payload[4U + i];
        line[tlen] = '\0';

        uint8_t gate = ArmMain_TextGate(line);
        if (gate != 0U) {
            /* 闸门拒绝时把**原因文本**回过去 —— 只回一个 err 码的话, 上位机
             * 只能猜"为什么这条命令在串口上好好的、从网口就不行"。 */
            const char* why = (gate == 2U)
                ? "  [GATE] E-Stop active: only 's' and 'h' pass over the link.\r\n"
                : "  [GATE] refused over link: interactive/blocking command.\r\n"
                  "         The calibration wizard ('c') is console-only by design\r\n"
                  "         (it waits for keystrokes on COM11; here it would hang\r\n"
                  "          the main loop, and this firmware has no watchdog).\r\n";
            return proto_text_lit(seq, 0x8B, why, 0x02U, rsp, rsp_max);
        }

        uint8_t trunc = 0U;
        (void)ArmMain_ExecText(line, (int)tlen, &trunc);
        s_text_trunc = trunc;
        return proto_text_page(seq, 0x8B, 0U, rsp, rsp_max);
    }

    case APROTO_CMD_TEXT_FETCH: {
        /* u16 off + u8 n = 3 B */
        if (len < 3U) return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
        uint8_t n = payload[2];
        if (n == 0U) return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
        return proto_text_page(seq, 0x8C, rd_u16(&payload[0]), rsp, rsp_max);
    }

    /* ---------- 危险操作: 强制 magic, 且**统一走文本透传** ----------
     * `M a` / `zr` / `F` 在控制台上早就实现好了, 这里不再写第二份 ——
     * 两份实现迟早分叉, 而这几条的后果是不可逆的(量纲基准 / Home 零点)。 */
    case APROTO_CMD_PARAM_SAVE:     /* 等价文本 `F`        */
    case APROTO_CMD_CENTER_CALIB:   /* 等价文本 `M a`      */
    case APROTO_CMD_ZERO_RESET: {   /* 等价文本 `zr [save]` */
        uint8_t need = (cmd == APROTO_CMD_ZERO_RESET) ? 5U : 4U;
        if (len < need) return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
        if (rd_u32(&payload[need - 4U]) != APROTO_MAGIC)
            return finish_ack(seq, cmd, ARM_API_ERR_MAGIC, rsp, rsp_max);

        char line[16];
        uint8_t o = 0U;
        if (cmd == APROTO_CMD_PARAM_SAVE) {
            line[o++] = 'F';
        } else if (cmd == APROTO_CMD_CENTER_CALIB) {
            line[o++] = 'M'; line[o++] = ' '; line[o++] = 'a';
        } else if (payload[0] == 0U) {
            line[o++] = 'z'; line[o++] = 'r';                 /* zr       */
        } else if (payload[0] == 1U) {
            line[o++] = 'z'; line[o++] = 'r'; line[o++] = ' ';
            line[o++] = 's'; line[o++] = 'a'; line[o++] = 'v';
            line[o++] = 'e';                                  /* zr save  */
        } else {
            /* 文本 `zr` 没有单轴变体(一次复位全部六轴) —— 不猜, 直接报参数错 */
            return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
        }
        line[o] = '\0';

        if (ArmMain_TextGate(line) != 0U)
            return finish_ack(seq, cmd, ARM_API_ERR_BUSY, rsp, rsp_max);
        (void)ArmMain_ExecText(line, (int)o, 0);   /* 本文件无 stddef.h ⇒ 不用 NULL */
        return finish_ack(seq, cmd, ARM_API_OK, rsp, rsp_max);
    }

    case APROTO_CMD_CALIB_ENTER:
        /* 明确拒绝, 而不是假装支持: cmdCalibrate() 里是 for(;;) 等人在**本机
         * 串口**上敲 '0'/'s'/'q' ⇒ 从网口进 = 主循环永不返回, 而本工程没有
         * 看门狗(红线 5)。它本来就要求人手摆位, 设计上是控制台专属命令。 */
        if (len < 4U) return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
        if (rd_u32(&payload[0]) != APROTO_MAGIC)
            return finish_ack(seq, cmd, ARM_API_ERR_MAGIC, rsp, rsp_max);
        RLOG_W("PROTO", "CALIB_ENTER refused: 'c' wizard is console-only");
        return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);

    case APROTO_CMD_SYS_RESET:
        /* 先回 ACK、主循环 150 ms 后才复位 —— 顺序反了上位机只会看到超时,
         * 分不清"复位成功"与"包丢了"。远程切链路: link can → link save → 这条。 */
        if (len < 4U) return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
        if (rd_u32(&payload[0]) != APROTO_MAGIC)
            return finish_ack(seq, cmd, ARM_API_ERR_MAGIC, rsp, rsp_max);
        ArmMain_RequestReset(150U);
        return finish_ack(seq, cmd, ARM_API_OK, rsp, rsp_max);

    default:
        RLOG_W("PROTO", "unknown cmd=0x%02X", cmd);
        return finish_ack(seq, cmd, ARM_API_ERR_PARAM, rsp, rsp_max);
    }

    return 0;   /* 不可达：所有分支均有返回 */
}
