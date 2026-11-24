/*
 * Feetech SCS 串行舵机通信协议层。
 * 适配 STS3215 等飞特 SCS/STS 系列舵机。
 *
 * SCS 协议帧格式:
 *   指令帧: [FF][FF][ID][Length][Instruction][Params...][Checksum]
 *   状态帧: [FF][FF][ID][Length][Error][Params...][Checksum]
 *
 *   Length = Instruction/Error(1) + Params(N) + Checksum(1) = N + 2
 *   Checksum = ~(ID + Length + Instruction/Error + Params) & 0xFF
 *
 * 与 Feetech SCS protocol 的区别:
 *   - 2 字节帧头 (非4字节)
 *   - 1 字节 Length (非2字节)
 *   - 1 字节寄存器地址 (非2字节)
 *   - 1 字节简单校验和 (非 CRC-16)
 */
#ifndef __FEETECH_PROTOCOL_H
#define __FEETECH_PROTOCOL_H

#include <stdint.h>
#include "servo_bus_hal.h"

/* ===== Feetech SCS Protocol ===== */
/* Instruction Packet:                                              */
/*   [0xFF][0xFF][ID][Length][Instruction][Params...][Checksum]    */
/* Status Packet:                                                   */
/*   [0xFF][0xFF][ID][Length][Error][Params...][Checksum]           */
/* Length = Instruction/Error(1) + Params(N) + Checksum(1) = N + 2  */

/* Instruction codes */
#define INST_PING           0x01
#define INST_READ           0x02
#define INST_WRITE          0x03
#define INST_REG_WRITE      0x04
#define INST_ACTION         0x05
#define INST_RESET          0x0A  /* 恢复出厂(原 Feetech 的 FACTORY_RESET=0x06 不适用) */
#define INST_SYNC_WRITE     0x83
#define INST_SYNC_READ      0x82

#define DXL_BROADCAST_ID    0xFE

/* Error bits in status packet (Error=0 means OK) */
#define DXL_ERR_RESULT_FAIL    0x01
#define DXL_ERR_INSTRUCTION    0x02
#define DXL_ERR_CRC            0x04
#define DXL_ERR_DATA_RANGE     0x08
#define DXL_ERR_DATA_LENGTH    0x10
#define DXL_ERR_DATA_LIMIT     0x20
#define DXL_ERR_ACCESS         0x40
#define DXL_ERR_ALERT          0x80  /* 硬件告警(过压/过流/过热) 数据仍有效 */

#define DXL_MAX_PARAMS 256

typedef struct {
    uint8_t  id;
    uint8_t  error;
    uint8_t  params[DXL_MAX_PARAMS];
    uint16_t param_len;
    uint8_t  valid;
} DXL_Packet_t;

/* SCS 校验和: ~(ID + Length + Inst + Params) & 0xFF */
uint8_t DXL_CalcChecksum(uint8_t* data, uint16_t len);

/* 组 SCS 指令帧。返回帧总长度(含校验和)。 */
uint16_t DXL_BuildPacket(uint8_t id, uint8_t inst, uint8_t* params,
                          uint16_t param_len, uint8_t* out_buf);

/* 解析 SCS 状态帧。成功返回1。 */
uint8_t  DXL_ParsePacket(uint8_t* raw, uint16_t len, DXL_Packet_t* pkt);

/* 高层协议函数 (SCS 协议 — 1字节地址，addr 取低8位) */
HAL_StatusTypeDef DXL_Ping(uint8_t bus, uint8_t id, uint32_t timeout_ms);
HAL_StatusTypeDef DXL_Read(uint8_t bus, uint8_t id, uint16_t addr, uint16_t len,
                            uint8_t* data, uint32_t timeout_ms);

/* ===== DXL_Read 的带诊断版本 (2026-09-11) =====
 * 读失败时"一个字节都没收到"和"帧合法但舵机告警"在上层看起来完全一样
 * (都是 SERVO_GetPosition 返回 -1), 但现场该查的东西完全不同:
 *   1 = 无应答   -> 线缆/接插/共地/供电/ID 不对
 *   2 = 帧损坏   -> 总线噪声/丢字节 (1Mbps 下中断打断轮询即会触发)
 *   3 = 舵机告警 -> 过载/过温/过压/传感器, 闻声摸温, 查机械与散热
 * (注意: 告警中的舵机 Ping 仍能通, 只有 DXL_Read 会拒绝 —— 见 DXL_Ping 注释) */
#define DXL_RD_OK         0U
#define DXL_RD_NO_REPLY   1U
#define DXL_RD_BAD_FRAME  2U
#define DXL_RD_ERROR_BITS 3U

HAL_StatusTypeDef DXL_ReadDiag(uint8_t bus, uint8_t id, uint16_t addr,
                               uint16_t len, uint8_t* data,
                               uint32_t timeout_ms,
                               uint8_t* reason, uint8_t* err_bits);
HAL_StatusTypeDef DXL_Write(uint8_t bus, uint8_t id, uint16_t addr,
                             uint8_t* data, uint16_t len, uint32_t timeout_ms);

/* 只发不收写入: 实时轨迹控制用。 */
HAL_StatusTypeDef DXL_WriteNoReply(uint8_t bus, uint8_t id, uint16_t addr,
                                    uint8_t* data, uint16_t len);

/* SCS Sync Write (广播, 无状态回复)。
 * 一帧更新多个舵机的同一段寄存器。 */
HAL_StatusTypeDef DXL_SyncWrite(uint8_t bus, uint16_t addr, uint16_t data_len,
                                const uint8_t* ids, const uint8_t* data, uint8_t count);

/* 全量总线扫描: 逐 ID(1..253) Ping, 收集在线舵机。诊断用(慢)。
 * timeout_ms 为每个空 ID 的等待时间(建议 20ms, 舵机应答<2ms)。
 * 常规识别请用 DXL_ScanIds 定点扫描期望 ID。 */
uint8_t DXL_ScanBus(uint8_t bus, uint8_t* found_ids, uint8_t max_ids,
                    uint32_t timeout_ms);

/* 定点识别: 只 Ping 给定的 ID 列表(如 BusA 期望 1/2/3, BusB 期望 4/5/6)。
 * 判定与 ESP32 改 ID 工具一致: 完整 6B 回应 + 帧头 FF FF + ID 回显匹配;
 * 舵机错误/告警位不拒绝(告警中的舵机仍在总线上)。
 * found_ids[] 按 ids[] 顺序填入在线 ID, 返回在线数量。 */
uint8_t DXL_ScanIds(uint8_t bus, const uint8_t* ids, uint8_t n_ids,
                    uint8_t* found_ids, uint8_t max_ids, uint32_t timeout_ms);

#endif
