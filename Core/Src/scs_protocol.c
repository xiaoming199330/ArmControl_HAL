/*
 * Feetech SCS 串行舵机协议层实现。
 * 替代原 Feetech SCS protocol, 适配 STS3215 等 SCS/STS 系列舵机。
 *
 * 帧格式:
 *   指令帧: FF FF | ID | Length | Instruction | Params... | Checksum
 *   状态帧: FF FF | ID | Length | Error | Params... | Checksum
 *   Length = 1(Inst/Error) + N(Params) + 1(Checksum) = N + 2
 *   Checksum = ~(ID + Length + Inst/Error + Params) & 0xFF
 *   线上字节 = 2(Header) + 1(ID) + 1(Length) + Length = 4 + Length
 */
#include "scs_protocol.h"
#include "servo_bus_hal.h"
#include <string.h>

/* ===== SCS Checksum: ~(sum of ID through last param) & 0xFF ===== */
uint8_t DXL_CalcChecksum(uint8_t* data, uint16_t len) {
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum += data[i];  /* 累加会自动取模256 */
    }
    return (uint8_t)(~sum);
}

/* Build SCS instruction packet.
   Layout:
     [FF][FF][ID][Length][Instruction][Params...][Checksum]
      0   1   2    3       4          5..         last
   Length = 1(Inst) + N(Params) + 1(Checksum) = N + 2
   Returns total wire bytes = 4 + Length = N + 6.
*/
uint16_t DXL_BuildPacket(uint8_t id, uint8_t inst, uint8_t* params,
                          uint16_t param_len, uint8_t* out_buf) {
    out_buf[0] = 0xFF;
    out_buf[1] = 0xFF;
    out_buf[2] = id;

    /* Length = Instruction(1) + Params(N) + Checksum(1) */
    uint8_t length = (uint8_t)(param_len + 2U);
    out_buf[3] = length;

    out_buf[4] = inst;

    for (uint16_t i = 0; i < param_len; i++) {
        out_buf[5 + i] = params[i];
    }

    /* Checksum covers ID + Length + Instruction + Params
       = bytes 2..(4+param_len) = param_len + 3 bytes */
    uint16_t cksum_data_len = param_len + 3U;
    out_buf[5 + param_len] = DXL_CalcChecksum(&out_buf[2], cksum_data_len);

    /* Total wire bytes = 2(header) + 1(ID) + 1(Length) + Length
       = 4 + (param_len + 2) = param_len + 6 */
    return param_len + 6U;
}

/* Parse SCS status packet from servo.
   Layout:
     [FF][FF][ID][Length][Error][Params...][Checksum]
      0   1   2    3      4      5..        last
   Length = 1(Error) + N(Params) + 1(Checksum) = N + 2
   Wire bytes = 4 + Length = N + 6.
   Returns 1 on success, 0 on failure.
*/
uint8_t DXL_ParsePacket(uint8_t* raw, uint16_t wire_len, DXL_Packet_t* pkt) {
    /* Need at least 6 bytes (header 2 + ID 1 + Length 1 + Error 1 + Checksum 1) */
    if (wire_len < 6) return 0;

    /* Check header */
    if (raw[0] != 0xFF || raw[1] != 0xFF)
        return 0;

    pkt->id = raw[2];
    uint8_t status_len = raw[3];

    /* Total wire bytes for this packet = 4 + status_len */
    if (wire_len < (uint16_t)(4 + status_len)) return 0;  /* Incomplete */

    /* Error byte */
    pkt->error = raw[4];

    /* status_len = 1(Error) + param_len + 1(Checksum) = param_len + 2
       param_len = status_len - 2 */
    if (status_len < 2) return 0;  /* Minimum: 1+0+1 = 2 */
    pkt->param_len = status_len - 2U;

    if (pkt->param_len > DXL_MAX_PARAMS) return 0;
    for (uint16_t i = 0; i < pkt->param_len; i++) {
        pkt->params[i] = raw[5 + i];
    }

    /* Checksum covers ID + Length + Error + Params
       = bytes 2..(4+param_len) = param_len + 3 bytes */
    uint16_t cksum_data_len = pkt->param_len + 3U;
    uint8_t cksum_calc = DXL_CalcChecksum(&raw[2], cksum_data_len);
    uint8_t cksum_pkt  = raw[5 + pkt->param_len];

    if (cksum_calc != cksum_pkt) return 0;

    pkt->valid = 1;
    return 1;
}

/* ===== High-Level Protocol Functions ===== */

/* 在原始接收缓冲中定位帧头 FF FF。
 * 半双工总线上可能混入残缺回声/噪声字节, 收到的字节流不一定从帧头开始,
 * 必须先同步帧头再解析 —— 否则 1 个前导垃圾字节就让整帧位移报废。
 * 返回帧头起始偏移, 找不到返回 -1。 */
static int16_t find_header(const uint8_t* raw, uint16_t len) {
    for (uint16_t i = 0; (uint16_t)(i + 2) <= len; i++) {
        if (raw[i] == 0xFF && raw[i + 1] == 0xFF) return (int16_t)i;
    }
    return -1;
}

HAL_StatusTypeDef DXL_Ping(uint8_t bus, uint8_t id, uint32_t timeout_ms) {
    /* Ping instruction: no params.
       Length = 1(Inst) + 0(Params) + 1(Checksum) = 2
       Wire bytes = 4 + 2 = 6 */
    uint8_t tx[8];
    uint16_t tx_len = DXL_BuildPacket(id, INST_PING, NULL, 0, tx);

    /* Ping status response: no params.
       Length = 1(Error) + 0(Params) + 1(Checksum) = 2
       Wire bytes = 4 + 2 = 6  (rx 容量留余量给前导噪声) */
    uint8_t rx[16];

    /* 最多 2 次尝试: 偶发首帧截断(方向切换竞态/舵机忙)重试即中 */
    for (uint8_t attempt = 0; attempt < 2; attempt++) {
        if (attempt > 0) HAL_Delay(3);

        uint16_t got = 0;
        HAL_StatusTypeDef st = DXL_TransmitReceive(bus, tx, tx_len,
                                                   rx, sizeof(rx), &got, timeout_ms);
        if (st != HAL_OK) continue;

        int16_t off = find_header(rx, got);
        if (off < 0 || (uint16_t)(got - off) < 6) continue;

        DXL_Packet_t pkt;
        if (!DXL_ParsePacket(&rx[off], (uint16_t)(got - off), &pkt)) continue;
        if (pkt.id != id) continue;
        /* 识别只关心"该 ID 的舵机是否在线": 帧合法 + ID 回显匹配即可,
         * 不因错误/告警位(error)拒绝 —— 告警中的舵机物理上仍在总线上,
         * 与 ESP32 改 ID 工具(ping_servo 只查帧头+ID)判定一致。
         * 错误状态应通过 DXL_Read(HW_ERROR) 由上层读取, 而不是在识别阶段丢掉。 */
        return HAL_OK;
    }

    return HAL_TIMEOUT;
}

HAL_StatusTypeDef DXL_ReadDiag(uint8_t bus, uint8_t id, uint16_t addr,
                               uint16_t read_len, uint8_t* data,
                               uint32_t timeout_ms,
                               uint8_t* reason, uint8_t* err_bits) {
    if (reason   != NULL) *reason   = DXL_RD_OK;
    if (err_bits != NULL) *err_bits = 0;

    /* Read params: [addr][read_len] — 1-byte address! */
    uint8_t params[2] = {
        (uint8_t)(addr & 0xFF),
        (uint8_t)(read_len & 0xFF)
    };

    uint8_t tx[8];
    uint16_t tx_len = DXL_BuildPacket(id, INST_READ, params, 2, tx);

    /* Read response: read_len params
       Length = 1(Error) + read_len + 1(Checksum) = read_len + 2
       Wire bytes = 4 + read_len + 2 = read_len + 6
     * ===== 2026-09-17: 前导噪声余量 +2 -> +8 =====
     * 半双工总线(TX/RX 并在同一根线)下, 自己的 TX 回声只能靠 drain_echo 排掉
     * "此刻已经在 DR 里的那几个字节" —— STM32F4 的 UART 没有 FIFO(DR + 移位
     * 寄存器最多留 2 字节), 其余回声字节被 ORE 丢掉, 所以残留 1~3 字节是常态。
     * 旧余量=2 时: 只要残留 3 字节, 真应答帧尾(CHK)就落在 rx_cap 之外 ⇒
     * DXL_ParsePacket 拿到的是被截断的帧 ⇒ 校验失败 ⇒ 整帧作废。
     * 现场指纹就是这个(报 "corrupt frame (bus noise / dropped byte)")。
     * 放宽到 +8 后, ≤6 个前导噪声字节都不再截断真帧(rx 缓冲 64 字节足够)。 */
    uint16_t rx_cap = (uint16_t)(6 + read_len + 8);
    if (rx_cap > 64) rx_cap = 64;
    uint8_t rx[64];
    uint16_t got = 0;

    /* 已知期望 ID -> 走快路径: 由状态帧 Length 字段定帧长, 收满即返回,
     * 不再等 5ms 静默判停。单次读 ~5ms -> ~0.3ms, 这是"运动中可以
     * 按节拍插入遥测读而不干扰轨迹下发"的前提。 */
    HAL_StatusTypeDef st = DXL_TransmitReceiveEx(bus, tx, tx_len, rx,
                                                rx_cap, &got, timeout_ms, id);
    if (st != HAL_OK) {
        if (reason != NULL) *reason = DXL_RD_NO_REPLY;   /* 一个字节都没收到 */
        return st;
    }

    int16_t off = find_header(rx, got);
    if (off < 0) {
        if (reason != NULL) *reason = DXL_RD_BAD_FRAME;
        return HAL_ERROR;
    }

    DXL_Packet_t pkt;
    if (!DXL_ParsePacket(&rx[off], (uint16_t)(got - off), &pkt)) {
        if (reason != NULL) *reason = DXL_RD_BAD_FRAME;
        return HAL_ERROR;
    }
    if (pkt.id != id) {
        if (reason != NULL) *reason = DXL_RD_BAD_FRAME;
        return HAL_ERROR;
    }
    if (err_bits != NULL) *err_bits = pkt.error;
    /* Alert(0x80) = hardware warning, data still valid */
    if ((pkt.error & 0x7F) != 0) {
        if (reason != NULL) *reason = DXL_RD_ERROR_BITS;
        return HAL_ERROR;
    }
    if (pkt.param_len < read_len) {
        if (reason != NULL) *reason = DXL_RD_BAD_FRAME;
        return HAL_ERROR;
    }

    for (uint16_t i = 0; i < read_len; i++) {
        data[i] = pkt.params[i];
    }
    return HAL_OK;
}

/* 兼容旧调用: 不关心失败原因 */
HAL_StatusTypeDef DXL_Read(uint8_t bus, uint8_t id, uint16_t addr,
                          uint16_t read_len, uint8_t* data, uint32_t timeout_ms) {
    return DXL_ReadDiag(bus, id, addr, read_len, data, timeout_ms, NULL, NULL);
}

HAL_StatusTypeDef DXL_Write(uint8_t bus, uint8_t id, uint16_t addr,
                             uint8_t* wr_data, uint16_t wr_len,
                             uint32_t timeout_ms) {
    /* Write params: [addr][data0]...[dataN] — 1-byte address! */
    uint8_t params[258];
    params[0] = (uint8_t)(addr & 0xFF);
    for (uint16_t i = 0; i < wr_len; i++) {
        params[1 + i] = wr_data[i];
    }
    uint16_t param_len = 1 + wr_len;

    uint8_t tx[270];
    uint16_t tx_len = DXL_BuildPacket(id, INST_WRITE, params, param_len, tx);

    if (id == DXL_BROADCAST_ID) {
        return DXL_Transmit(bus, tx, tx_len);
    }

    /* Write status response: 0 params
       Length = 1(Error) + 0 + 1(Checksum) = 2
       Wire bytes = 4 + 2 = 6 */
    uint8_t rx[16];
    uint16_t got = 0;
    HAL_StatusTypeDef st = DXL_TransmitReceive(bus, tx, tx_len, rx,
                                              sizeof(rx), &got, timeout_ms);

    if (st == HAL_TIMEOUT || got == 0) {
        /* Status Return Level = 1: no response for Write -> treat as OK */
        return HAL_OK;
    }
    if (st != HAL_OK) return st;

    int16_t off = find_header(rx, got);
    if (off < 0) return HAL_ERROR;

    DXL_Packet_t pkt;
    if (!DXL_ParsePacket(&rx[off], (uint16_t)(got - off), &pkt)) return HAL_ERROR;
    if (pkt.id != id) return HAL_ERROR;
    /* Alert(0x80) = hardware warning, write still succeeded */
    if ((pkt.error & 0x7F) != 0) return HAL_ERROR;
    return HAL_OK;
}

/* ===== Fire-and-forget Write: TX only, no RX wait ===== */
HAL_StatusTypeDef DXL_WriteNoReply(uint8_t bus, uint8_t id, uint16_t addr,
                                    uint8_t* wr_data, uint16_t wr_len) {
    uint8_t params[258];
    params[0] = (uint8_t)(addr & 0xFF);
    for (uint16_t i = 0; i < wr_len; i++) {
        params[1 + i] = wr_data[i];
    }
    uint16_t param_len = 1 + wr_len;

    uint8_t tx[270];
    uint16_t tx_len = DXL_BuildPacket(id, INST_WRITE, params, param_len, tx);
    return DXL_Transmit(bus, tx, tx_len);
}

/* ===== SCS Sync Write: one broadcast packet updates many servos =====
 * Params: [addr][data_len]
 *         [ID1][data...][ID2][data...]...
 * Broadcast packets do not return status, so this is always TX-only.
 * NOTE: SCS uses 1-byte address and 1-byte data_len (not 2-byte like P2.0). */
HAL_StatusTypeDef DXL_SyncWrite(uint8_t bus, uint16_t addr, uint16_t data_len,
                                const uint8_t* ids, const uint8_t* data, uint8_t count) {
    if (ids == NULL || data == NULL || count == 0U || data_len == 0U) return HAL_ERROR;

    uint16_t entry_len = (uint16_t)data_len + 1U;  /* ID + data */
    uint16_t param_len = 2U + (uint16_t)count * entry_len;  /* addr + datalen + entries */
    if (param_len > 258U) return HAL_ERROR;

    uint8_t params[258];
    params[0] = (uint8_t)(addr & 0xFFU);
    params[1] = (uint8_t)(data_len & 0xFFU);

    for (uint8_t i = 0; i < count; i++) {
        uint16_t offset = 2U + (uint16_t)i * entry_len;
        params[offset] = ids[i];
        memcpy(&params[offset + 1U], &data[(uint16_t)i * data_len], data_len);
    }

    uint8_t tx[270];
    uint16_t tx_len = DXL_BuildPacket(DXL_BROADCAST_ID, INST_SYNC_WRITE,
                                      params, param_len, tx);
    return DXL_Transmit(bus, tx, tx_len);
}

/* ===== 全量总线扫描: 逐 ID(1..253) Ping, 收集在线舵机 — 诊断用 =====
 * ⚠ 253 个 ID 逐个 Ping, 空总线上每个 ID 都要等满超时(80ms),
 *    空总线约 20s, 两条总线全空约 40s。常规启动/识别请用 DXL_ScanIds
 *    只 Ping 期望 ID(如 BusA 1/2/3), 快 ~80 倍。
 * 范围 1..253 与 ESP32 改 ID 工具一致(从 1 开始, 253 也是合法 ID)。
 * 判定: 有效回应 + ID 回显匹配即在线, 不因错误/告警位拒绝(与 ESP32 一致)。 */
uint8_t DXL_ScanBus(uint8_t bus, uint8_t* found_ids, uint8_t max_ids,
                    uint32_t timeout_ms) {
    uint8_t count = 0;
    uint8_t tx[8];

    for (uint8_t id = 1; id <= 253; id++) {
        uint16_t tx_len = DXL_BuildPacket(id, INST_PING, NULL, 0, tx);
        uint8_t rx[16];
        uint16_t got = 0;

        HAL_StatusTypeDef st = DXL_TransmitReceive(bus, tx, tx_len,
                                                  rx, sizeof(rx), &got,
                                                  timeout_ms);
        if (st != HAL_OK) continue;

        int16_t off = find_header(rx, got);
        if (off < 0 || (uint16_t)(got - off) < 6) continue;

        DXL_Packet_t pkt;
        if (!DXL_ParsePacket(&rx[off], (uint16_t)(got - off), &pkt)) continue;
        if (pkt.id != id) continue;        /* 防串扰: 必须是本人回应 */
        /* 有效回应即视为舵机物理存在 */
        if (count < max_ids) {
            found_ids[count] = id;
        }
        count++;
    }

    return count;
}

/* ===== 定点识别: 只 Ping 给定的 ID 列表 (启动/`n` 命令用) =====
 * 初始化为固定布局: Bus A(ID 1/2/3) + Bus B(ID 4/5/6)。
 * 帧格式与 ESP32 工具的 ping_servo() 完全一致:
 *   TX: FF FF ID 02 01 CK (6B)   RX: FF FF ID 02 00 CK (6B)
 * 判定: 收到完整 6B + 帧头 FF FF + ID 回显匹配 + 校验和正确。
 * 舵机处于错误/告警态(error!=0)也计为在线 —— 物理上它就在总线上。
 * 每个 ID 最多尝试 2 次(偶发首帧截断/舵机忙时重试即中),
 * ID 间留 2ms 总线恢复时间。
 * found_ids[] 按 ids[] 顺序填入在线 ID; 返回在线数量。 */
uint8_t DXL_ScanIds(uint8_t bus, const uint8_t* ids, uint8_t n_ids,
                    uint8_t* found_ids, uint8_t max_ids, uint32_t timeout_ms) {
    uint8_t count = 0;
    uint8_t tx[8];

    if (ids == NULL || n_ids == 0U) return 0;

    for (uint8_t i = 0; i < n_ids; i++) {
        uint8_t id = ids[i];
        uint8_t online = 0;

        for (uint8_t attempt = 0; attempt < 4 && !online; attempt++) {
            if (attempt > 0) HAL_Delay(10);   /* 重试前留恢复时间 */

            uint16_t tx_len = DXL_BuildPacket(id, INST_PING, NULL, 0, tx);
            uint8_t rx[16];
            uint16_t got = 0;

            HAL_StatusTypeDef st = DXL_TransmitReceive(bus, tx, tx_len,
                                                      rx, sizeof(rx), &got, timeout_ms);
            if (st != HAL_OK) continue;       /* 无回应/超时 = 该 ID 不在线 */

            int16_t off = find_header(rx, got);
            if (off < 0 || (uint16_t)(got - off) < 6) continue;

            DXL_Packet_t pkt;
            if (!DXL_ParsePacket(&rx[off], (uint16_t)(got - off), &pkt)) continue;
            if (pkt.id != id) continue;       /* 防串扰: 必须是本人回应 */

            online = 1;
        }

        if (online) {
            if (count < max_ids) {
                found_ids[count] = id;
            }
            count++;
        }

        HAL_Delay(8);   /* ID 间总线恢复: 上一帧收尾/舵机释放总线 + 74HC126 切换抖动 */
    }

    return count;
}
