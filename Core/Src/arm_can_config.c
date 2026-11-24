/* ============================================================
 * arm_can_config.c - CAN 通信配置的 RAM 态与 Flash 持久化
 * 见 arm_can_config.h。
 * ============================================================ */
#include "arm_can_config.h"
#include "flash_cfg.h"
#include "arm_runlog.h"
#include <string.h>
#include <stddef.h>

#define CAN_CFG_MAGIC    0x43414E31UL   /* "CAN1" */
#define CAN_CFG_VERSION  1U

const uint16_t can_cfg_baud_table[CAN_CFG_BAUD_COUNT] = {
    100U, 125U, 250U, 500U, 1000U
};

/* Flash 记录（32 字节, 远小于 4KB 区跨度） */
typedef struct {
    uint32_t  magic;
    uint16_t  version;
    uint16_t  len;
    ArmCanCfg cfg;
    uint32_t  crc;
} CanCfgRecord;

static ArmCanCfg s_cfg;               /* 当前生效 */
static ArmCanCfg s_saved;             /* Flash 里的那份(用于脏判定) */
static uint8_t   s_from_flash = 0U;

/* CRC-32 (poly 0xEDB88320, 反射), 与 pid_flash.c 同一算法 */
static uint32_t cfg_crc(const CanCfgRecord* r)
{
    const uint8_t* b = (const uint8_t*)r;
    uint32_t crc = 0xFFFFFFFFU;
    for (uint32_t n = 0U; n < offsetof(CanCfgRecord, crc); n++) {
        crc ^= b[n];
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            crc = (crc >> 1) ^ (0xEDB88320U & (-(int32_t)(crc & 1U)));
        }
    }
    return ~crc;
}

static void cfg_defaults(ArmCanCfg* c)
{
    memset(c, 0, sizeof(*c));
    c->node_id   = CAN_CFG_DEF_NODE;
    c->iface     = CAN_CFG_DEF_IFACE;   /* 0918H 起默认 = CAN2, 见 arm_can_config.h */
    c->baud_kbps = CAN_CFG_DEF_BAUD;
    c->hb_ms     = CAN_CFG_DEF_HB_MS;
}

uint8_t CanCfg_IfaceOrDefault(const ArmCanCfg* c)
{
    if (c == NULL) return CAN_CFG_DEF_IFACE;
    return (c->iface == CAN_CFG_IF_CAN2) ? CAN_CFG_IF_CAN2 : CAN_CFG_IF_CAN1;
}

/* 合法性: 字段越界一律视为非法（宁愿回落默认, 也不让坏配置进总线） */
static uint8_t cfg_valid(const ArmCanCfg* c)
{
    if (c->node_id < CAN_CFG_NODE_MIN || c->node_id > CAN_CFG_NODE_MAX) return 0U;
    if (!CanCfg_BaudSupported(c->baud_kbps)) return 0U;
    if (c->hb_ms != 0U &&
        (c->hb_ms < CAN_CFG_HB_MIN || c->hb_ms > CAN_CFG_HB_MAX)) return 0U;
    /* iface: 0 是旧记录里的 reserved0, 视为 CAN1; 别的非法值一律回落默认。
     * ⚠ 这里**不判非法**而是靠 CanCfg_IfaceOrDefault() 归一 —— 否则一条
     *   老记录(iface=0)会被判成坏记录, 连带 node/baud 一起丢。 */
    return 1U;
}

uint8_t CanCfg_BaudSupported(uint16_t kbps)
{
    for (uint8_t i = 0U; i < CAN_CFG_BAUD_COUNT; i++) {
        if (can_cfg_baud_table[i] == kbps) return 1U;
    }
    return 0U;
}

void CanCfg_Load(void)
{
    CanCfgRecord r;
    cfg_defaults(&s_cfg);
    s_saved = s_cfg;

    if (CfgFlash_ReadRegion(CFG_OFF_CAN, &r, sizeof(r)) != CFG_FLASH_OK) {
        RLOG_W("CAN", "cfg read failed, use defaults");
        return;
    }
    if (r.magic != CAN_CFG_MAGIC || r.version != CAN_CFG_VERSION ||
        r.len != sizeof(ArmCanCfg) || r.crc != cfg_crc(&r)) {
        /* 未写过 / 版本不符 / 校验失败 —— 都是"没配置过", 用默认值。
         * ⚠ 这里**绝不能**顺手写 Flash: 启动早期写扇区会拖慢启动,
         *   而且会让只读启动变成写操作。落盘只由 `can save` 触发。 */
        return;
    }
    if (!cfg_valid(&r.cfg)) {
        RLOG_W("CAN", "cfg out of range, use defaults");
        return;
    }
    s_cfg = r.cfg;
    s_saved = r.cfg;
    s_from_flash = 1U;
}

const ArmCanCfg* CanCfg_Get(void) { return &s_cfg; }

int CanCfg_SetNode(uint8_t node)
{
    if (node < CAN_CFG_NODE_MIN || node > CAN_CFG_NODE_MAX) return -1;
    s_cfg.node_id = node;
    return 0;
}

int CanCfg_SetBaud(uint16_t kbps)
{
    if (!CanCfg_BaudSupported(kbps)) return -1;
    s_cfg.baud_kbps = kbps;
    return 0;
}

int CanCfg_SetHb(uint16_t ms)
{
    if (ms != 0U && (ms < CAN_CFG_HB_MIN || ms > CAN_CFG_HB_MAX)) return -1;
    s_cfg.hb_ms = ms;
    return 0;
}

int CanCfg_SetIface(uint8_t iface)
{
    if (iface != CAN_CFG_IF_CAN1 && iface != CAN_CFG_IF_CAN2) return -1;
    s_cfg.iface = iface;
    return 0;
}

void CanCfg_Reset(void)
{
    cfg_defaults(&s_cfg);
}

int CanCfg_Save(void)
{
    CanCfgRecord r;
    memset(&r, 0, sizeof(r));
    r.magic = CAN_CFG_MAGIC;
    r.version = CAN_CFG_VERSION;
    r.len = sizeof(ArmCanCfg);
    r.cfg = s_cfg;
    r.crc = cfg_crc(&r);

    if (CfgFlash_WriteRegion(CFG_OFF_CAN, &r, sizeof(r)) != CFG_FLASH_OK) {
        RLOG_E("CAN", "cfg save FAILED");
        return -1;
    }
    s_saved = s_cfg;
    s_from_flash = 1U;
    RLOG_I("CAN", "cfg saved: node=%d iface=CAN%u baud=%u hb=%u",
           (int)s_cfg.node_id, (unsigned)CanCfg_IfaceOrDefault(&s_cfg),
           (unsigned)s_cfg.baud_kbps, (unsigned)s_cfg.hb_ms);
    return 0;
}

uint8_t CanCfg_IsSaved(void)
{
    return (memcmp(&s_cfg, &s_saved, sizeof(s_cfg)) == 0) ? 1U : 0U;
}

uint8_t CanCfg_LoadedFromFlash(void) { return s_from_flash; }
