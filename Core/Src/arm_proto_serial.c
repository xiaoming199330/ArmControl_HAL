/* ============================================================
 * arm_proto_serial.c - 二进制控制协议的串口承载（实现）
 *
 * 实例化的解析器：同一份代码服务控制台口 (USART6) 与主从链路口 (UART4)。
 * 接收: 各自 UART 的 RX ISR -> ProtoSer_Feed（状态机，ISR 安全）
 *       完整帧拷贝入该实例的 pending 缓存（单帧深度，忙帧计丢弃）
 * 执行: 主循环 ProtoSer_Dispatch -> Aproto_Dispatch
 * 发送: HAL_UART_Transmit(s->huart)，主循环上下文，短帧阻塞可接受
 *
 * 链路口的 nack_only=1: 流模式下每帧回 ACK 是纯噪声（50Hz × 9B），
 * 故只回 NACK 且限流 100ms —— 既保住“从臂急停/被拒”的可见性，
 * 又不占用链路与主循环时间。
 * ============================================================ */
#include "arm_proto_serial.h"
#include "arm_control_protocol.h"
#include "arm_control_api.h"
#include "arm_runlog.h"
#include "main.h"
#include "usart.h"
#include <string.h>

/* 控制台 UART 句柄（定义在 main.c，非 static） */
extern UART_HandleTypeDef hconsole;

/* NACK 最小间隔（链路口），防 50Hz 连续被拒时刷屏 */
#define LINK_NACK_MIN_INTERVAL_MS   100U

static ProtoSerial s_console;
static ProtoSerial s_link;

/* ================= 通用实现 ================= */
void ProtoSer_Init(ProtoSerial* s, UART_HandleTypeDef* huart) {
    Aproto_ParserReset(&s->parser);
    s->huart       = huart;
    s->pending     = 0;
    s->crc_err     = 0;
    s->seq         = 0;
    s->cmd         = 0;
    s->len         = 0;
    s->dropped     = 0;
    s->nack_only   = 0;
    s->last_err_ms = 0;
    s->count_stats = 0;
    s->stream_count = 0;
    s->reject_count = 0;
    s->bytes_rx = 0;
    s->frames_any = 0;
    s->crc_errs = 0;
    s->rsp_rx = 0;
    s->fe_errs = 0; s->ne_errs = 0; s->ore_errs = 0;
    s->first_ms = 0; s->last_ms = 0;
    s->last_seq = 0; s->last_cmd = 0; s->last_len = 0; s->last_crc_ok = 0;
    s->last_err_code = 0;
    s->last_rsp_err = 0;
    s->last_raw_len = 0;
}

uint8_t ProtoSer_Feed(ProtoSerial* s, uint8_t ch) {
    if (s->count_stats) {
        uint32_t now = HAL_GetTick();
        if (s->bytes_rx == 0U) s->first_ms = now;
        s->last_ms = now;
        s->bytes_rx++;
    }

    uint8_t r = Aproto_ParserFeed(&s->parser, ch, HAL_GetTick());
    if (r == 0) return 0;                 /* 文本字节，交还控制台 */
    if (r == 2 || r == 3) {               /* 帧完成（2=CRC 正确, 3=CRC 错误） */
        if (s->pending) {
            s->dropped++;                 /* 前一帧未处理，覆盖丢弃计数 */
        }
        s->seq = s->parser.seq;
        s->cmd = s->parser.cmd;
        s->len = s->parser.len;
        memcpy(s->payload, s->parser.payload, s->parser.len);
        s->crc_err = (r == 3) ? 1 : 0;
        s->pending = 1;

        /* 诊断快照: frames_any / crc_errs + 复原本帧原始字节。
         * 此刻 ch 就是本帧最后一个字节 = CRC 高字节; 解析器的 payload
         * 在 Reset 里不清零, 故可安全读取。 */
        if (s->count_stats) {
            s->frames_any++;
            if (r == 3) s->crc_errs++;
            s->last_seq    = s->parser.seq;
            s->last_cmd    = s->parser.cmd;
            s->last_len    = s->parser.len;
            s->last_crc_ok = (r == 2) ? 1U : 0U;

            uint8_t n = 0;
            s->last_raw[n++] = APROTO_HEAD0;
            s->last_raw[n++] = APROTO_HEAD1;
            s->last_raw[n++] = s->parser.seq;
            s->last_raw[n++] = s->parser.cmd;
            s->last_raw[n++] = s->parser.len;
            for (uint8_t i = 0; i < s->parser.len; i++) s->last_raw[n++] = s->parser.payload[i];
            s->last_raw[n++] = s->parser.crc_lo;
            s->last_raw[n++] = ch;
            s->last_raw_len = n;
        }
    }
    return 1;
}

void ProtoSer_Dispatch(ProtoSerial* s) {
    if (!s->pending) return;

    /* 取出 pending（短临界区防 ISR 覆盖） */
    __disable_irq();
    uint8_t seq = s->seq, cmd = s->cmd, len = s->len, crc_err = s->crc_err;
    uint8_t payload[APROTO_MAX_PAYLOAD];
    memcpy(payload, s->payload, len);
    s->pending = 0;
    __enable_irq();

    uint8_t rsp[APROTO_MAX_FRAME];
    uint8_t rsp_len;

    if (crc_err) {
        /* CRC 错误：直接 NACK，不进入 API */
        uint8_t p[2] = { cmd, ARM_API_ERR_CRC };
        rsp_len = Aproto_BuildFrame(seq, APROTO_RSP_NACK, p, 2,
                                    rsp, sizeof(rsp));
        if (!s->nack_only) {
            RLOG_W("SER", "frame CRC err, cmd=0x%02X seq=%d", cmd, seq);
        }
    } else if (cmd >= 0x80U || cmd == APROTO_RSP_ACK || cmd == APROTO_RSP_NACK) {
        /* ===== 回包帧：不是命令，只记录，绝不回包 =====
         * ACK(0x7F)/NACK(0x7E) 与"请求码|0x80"的查询响应(0x81/0x82/0xA1/
         * 0xC0/0xC2) 都是本协议的*响应*方向。老代码把它们当未知命令走
         * default 分支 -> 再回一个 NACK -> 对端收到 NACK 又当未知命令…
         * 两端互相回，形成 ~10Hz 的死循环: `lk` 上表现为
         *   stream frames 恒 0、rejects 一直涨
         * 把真正的故障(主臂帧根本没到)完全掩盖。这里直接吞掉。 */
        rsp_len = 0;
        if (s->count_stats) {
            s->rsp_rx++;
            if (cmd == APROTO_RSP_NACK && len >= 2U) {
                s->last_rsp_err = payload[1];   /* NACK 负载 = {原命令码, 错误码} */
            }
        }
    } else {
        rsp_len = Aproto_Dispatch(seq, cmd, payload, len,
                                  rsp, sizeof(rsp), RunLog_RunId());
    }

    if (rsp_len == 0 || s->huart == NULL) return;

    if (s->count_stats && !crc_err && cmd == APROTO_CMD_STREAM_JOINTS) {
        s->stream_count++;
    }

    if (s->count_stats && rsp[3] == APROTO_RSP_NACK && rsp_len >= 7U) {
        s->last_err_code = rsp[6];   /* 我方最近一次拒绝的原因 */
    }

    /* 链路流模式：抑制 ACK，只回限流后的 NACK */
    if (s->nack_only) {
        if (rsp[3] != APROTO_RSP_NACK) return;
        if (s->count_stats) s->reject_count++;
        uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - s->last_err_ms) < LINK_NACK_MIN_INTERVAL_MS) return;
        s->last_err_ms = now;
    }

    HAL_UART_Transmit(s->huart, rsp, rsp_len, 100);
}

/* ================= 控制台口 (USART6) ================= */
void SerialProto_Init(void) {
    ProtoSer_Init(&s_console, &hconsole);
}

uint8_t SerialProto_FeedByte(uint8_t ch) {
    return ProtoSer_Feed(&s_console, ch);
}

void SerialProto_DispatchPending(void) {
    ProtoSer_Dispatch(&s_console);
}

uint16_t SerialProto_DroppedCount(void) { return s_console.dropped; }

/* ================= 主从链路口 (UART4) ================= */
void LinkProto_Init(void) {
    ProtoSer_Init(&s_link, &huart4);
    s_link.nack_only   = 1U;    /* 只回 NACK，不回 ACK */
    s_link.count_stats = 1U;
}

uint8_t LinkProto_FeedByte(uint8_t ch) {
    return ProtoSer_Feed(&s_link, ch);
}

void LinkProto_DispatchPending(void) {
    ProtoSer_Dispatch(&s_link);
}

uint16_t LinkProto_DroppedCount(void) { return s_link.dropped; }
uint32_t LinkProto_StreamCount(void)  { return s_link.stream_count; }
uint32_t LinkProto_RejectCount(void)  { return s_link.reject_count; }

/* UART4 中断登记本字节的硬件错误标志 (FE/NE/ORE)。
 * 注意必须"先读 SR 再读 DR"才能清掉这些标志 —— 顺序写在 ISR 里。 */
void LinkProto_NoteRxErrors(uint8_t fe, uint8_t ne, uint8_t ore) {
    if (!s_link.count_stats) return;
    if (fe)  s_link.fe_errs++;
    if (ne)  s_link.ne_errs++;
    if (ore) s_link.ore_errs++;
}

void LinkProto_GetDiag(LinkDiag* d) {
    if (d == NULL) return;
    __disable_irq();
    d->bytes_rx  = s_link.bytes_rx;
    d->frames    = s_link.frames_any;
    d->crc_errs  = s_link.crc_errs;
    d->rsp_rx    = s_link.rsp_rx;
    d->fe        = s_link.fe_errs;
    d->ne        = s_link.ne_errs;
    d->ore       = s_link.ore_errs;
    d->span_ms   = (s_link.last_ms >= s_link.first_ms)
                 ? (s_link.last_ms - s_link.first_ms) : 0U;
    d->seq       = s_link.last_seq;
    d->cmd       = s_link.last_cmd;
    d->len       = s_link.last_len;
    d->crc_ok    = s_link.last_crc_ok;
    d->nack_err  = s_link.last_err_code;
    d->rsp_err   = s_link.last_rsp_err;
    d->raw_len   = s_link.last_raw_len;
    memcpy(d->raw, s_link.last_raw, sizeof(d->raw));
    __enable_irq();
}
