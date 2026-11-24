/* ============================================================
 * arm_proto_serial.h - 二进制控制协议的串口承载
 *
 * 一个 ProtoSerial 实例 = 一条串口上的解析状态机 + 单帧 pending 缓存。
 * 本工程用到两条，互不干扰：
 *
 *   1) 控制台口 (USART6 @115200, 与文本命令共用)
 *      - 帧头 0xAA 不可能是文本命令首字符，天然无歧义分流
 *      - USART6 ISR 中先经 SerialProto_FeedByte 过滤：
 *          返回 1 = 字节属协议帧（已被状态机消费，不进文本环形缓冲）
 *          返回 0 = 普通文本字节（照原路径进 rx_ring）
 *
 *   2) 主从链路口 (UART4 @115200, PA0/PA1, 纯二进制)
 *      - 主臂板把 0x13 STREAM_JOINTS 帧流式打进来，无文本混流
 *      - 抑制每帧 ACK（50Hz 流下纯属噪声），只回限流后的 NACK，
 *        让主臂能感知“被拒帧 / 从臂急停”，又不淹没链路
 *
 * 两条口都遵循：ISR 只做状态机与帧缓存；解析执行在主循环
 * XxxProto_DispatchPending() 完成（可安全总线通信/打印）。
 *
 * 上位机: tools/arm_run_logger.py --proto 可构造/发送/解析本协议帧。
 * ============================================================ */
#ifndef __ARM_PROTO_SERIAL_H
#define __ARM_PROTO_SERIAL_H

#include <stdint.h>
#include "arm_control_protocol.h"
#include "usart.h"

/* 一条串口上的协议承载实例 */
typedef struct {
    AprotoParser        parser;
    UART_HandleTypeDef* huart;       /* 回包用 (TX) */
    volatile uint8_t    pending;     /* ISR 写 / 主循环读，单帧深度 */
    volatile uint8_t    crc_err;
    uint8_t             seq, cmd, len;
    uint8_t             payload[APROTO_MAX_PAYLOAD];
    uint16_t            dropped;     /* 前一帧未处理即被覆盖的次数 */
    uint8_t             nack_only;   /* 1 = 抑制 ACK，只回限流后的 NACK */
    uint32_t            last_err_ms; /* NACK 限流时间戳 */
    uint8_t             count_stats; /* 1 = 统计流帧/NACK (链路口用) */
    uint32_t            stream_count;/* 成功执行的 0x13 流帧数 */
    uint32_t            reject_count;/* 回 NACK 的次数 (含 CRC 错) */

    /* ---- 链路诊断 (count_stats=1 时维护, 供 `lk` 定位"线上到底来了什么") ----
     * 把"没收到 / 收到但组不成帧 / 组成了但 CRC 错 / 收到的是回包帧"
     * 四种情况分开计数 —— 否则现场只能看到一句 0 frames, 无从下手。 */
    uint32_t            bytes_rx;    /* 收到的原始字节数 */
    uint32_t            frames_any;  /* 结构完整(帧头+长度成立)的帧数, 含 CRC 错 */
    uint32_t            crc_errs;    /* frames_any 中 CRC 校验失败的帧数 */
    uint32_t            rsp_rx;      /* 收到的"回包帧"数 (ACK/NACK/查询响应) */

    /* ---- UART 硬件错误 + 字节率 (2026-09-11, 定位"CRC 全错"的物理层) ----
     * 光看 CRC 错分不清是"电平/波特率"还是"丢字节", 这三个硬件标志能分开:
     *   FE(帧错误) 停止位采样不对  -> 波特率失配 / 电平不够 / 有大干扰
     *   NE(噪声错误) 采样点电平不稳 -> 电气干扰(串扰/地弹/线太长)
     *   ORE(接收溢出) 没及时取走   -> 丢字节; 一旦丢字节, 后面整段帧错位,
     *                                 表现为"帧头难得找到, 但残存字节是对的"
     * first_ms/last_ms 用来算实测字节率, 与"主臂 19B/33ms = 576 B/s"对照:
     * 明显偏低 = 丢字节, 明显偏高 = 两端波特率不成 1:1。 */
    uint32_t            fe_errs, ne_errs, ore_errs;
    uint32_t            first_ms, last_ms;
    uint8_t             last_seq, last_cmd, last_len, last_crc_ok;
    uint8_t             last_err_code;  /* 我方最近发出的 NACK 错误码 */
    uint8_t             last_rsp_err;   /* 最近收到的 NACK 错误码 */
    uint8_t             last_raw[APROTO_MAX_FRAME];  /* 最近一帧原始字节 */
    uint8_t             last_raw_len;
} ProtoSerial;

/* 链路诊断快照 (供 `lk` 打印) */
typedef struct {
    uint32_t bytes_rx;      /* 原始字节数 */
    uint32_t frames;        /* 结构完整的帧数 (含 CRC 错) */
    uint32_t crc_errs;      /* 其中 CRC 校验失败 */
    uint32_t rsp_rx;        /* 其中是回包帧 (ACK/NACK/查询响应) 的数量 */
    uint32_t fe, ne, ore;   /* UART 帧错 / 噪声错 / 接收溢出 计数 */
    uint32_t span_ms;       /* 收字节的时间跨度 (算实测字节率用) */
    uint8_t  cmd, seq, len, crc_ok;   /* 最近一帧的头部与校验结果 */
    uint8_t  nack_err;      /* 我方最后一个 NACK 的错误码 (ARM_API_ERR_*) */
    uint8_t  rsp_err;       /* 收到的最后一个 NACK 的错误码 */
    uint8_t  raw_len;       /* raw[] 有效长度 */
    uint8_t  raw[APROTO_MAX_FRAME];   /* 最近一帧原始字节 (AA 55 SEQ CMD LEN ...) */
} LinkDiag;

/* 通用接口（也供以后新增第三条口复用） */
void    ProtoSer_Init(ProtoSerial* s, UART_HandleTypeDef* huart);
uint8_t ProtoSer_Feed(ProtoSerial* s, uint8_t ch);      /* ISR 安全 */
void    ProtoSer_Dispatch(ProtoSerial* s);              /* 主循环上下文 */

/* ---- 控制台口 (USART6) ---- */
void     SerialProto_Init(void);
uint8_t  SerialProto_FeedByte(uint8_t ch);
void     SerialProto_DispatchPending(void);
uint16_t SerialProto_DroppedCount(void);

/* ---- 主从链路口 (UART4) ---- */
void     LinkProto_Init(void);
uint8_t  LinkProto_FeedByte(uint8_t ch);
void     LinkProto_DispatchPending(void);
uint16_t LinkProto_DroppedCount(void);

/* 链路统计：成功执行的流帧数 / 被拒（NACK）次数，供 `lk` 命令查看 */
uint32_t LinkProto_StreamCount(void);
uint32_t LinkProto_RejectCount(void);

/* 由 UART4 中断把本字节的硬件错误标志登记进来 (ISR 安全, 无阻塞) */
void     LinkProto_NoteRxErrors(uint8_t fe, uint8_t ne, uint8_t ore);

/* 链路诊断快照：字节数/帧数/CRC 错/回包数 + 最近一帧原始字节。
 * `lk` 靠它区分「线上没东西」和「有东西但都不合法」。 */
void     LinkProto_GetDiag(LinkDiag* d);

#endif /* __ARM_PROTO_SERIAL_H */
