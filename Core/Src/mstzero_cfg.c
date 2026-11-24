/* ============================================================
 * mstzero_cfg.c - 主臂零点持久化实现 (区 H, 见 mstzero_cfg.h)
 * 结构 32B 自带 magic + 求和校验; 搬运用 CfgFlash 保区写,
 * A~G 区自动原样保住 (见 flash_cfg.c)。
 * ============================================================ */
#include "mstzero_cfg.h"
#include "flash_cfg.h"

#define MSTZERO_MAGIC   0x4D535431U   /* "MST1" little-endian */

typedef struct {
    uint32_t magic;      /* MSTZERO_MAGIC, 区空(0xFF)/坏数据时判无效 */
    uint16_t zero[6];    /* 主臂各关节官方 Home raw 零点 */
    uint16_t pad;        /* 对齐到 4 的倍数 */
    uint32_t sum;        /* 前 28B 按 u32 求和取反 */
} MstZeroRec;

/* 前 (sizeof-4) 字节按 u32 累加取反。记录 4 字节对齐 —— 已保证。 */
static uint32_t mstzero_sum(const MstZeroRec* r) {
    const uint32_t* w = (const uint32_t*)(const void*)r;
    uint32_t s = 0U;
    for (uint32_t i = 0U; i < (sizeof(MstZeroRec) / 4U) - 1U; i++) s += w[i];
    return ~s;
}

uint8_t MstZeroCfg_Save(const uint16_t z[6]) {
    MstZeroRec r;
    if (z == 0) return CFG_FLASH_BADARG;
    for (uint32_t j = 0U; j < 6U; j++) {
        if (z[j] == 0xFFFFU) return CFG_FLASH_BADARG;   /* 哨兵不可入库 */
    }
    r.magic = MSTZERO_MAGIC;
    for (uint32_t j = 0U; j < 6U; j++) r.zero[j] = z[j];
    r.pad = 0U;
    r.sum = mstzero_sum(&r);
    return CfgFlash_WriteRegion(CFG_OFF_MSTZERO, &r, (uint32_t)sizeof(r));
}

uint8_t MstZeroCfg_Load(uint16_t z[6]) {
    MstZeroRec r;
    if (z == 0) return 0U;
    if (CfgFlash_ReadRegion(CFG_OFF_MSTZERO, &r, (uint32_t)sizeof(r)) != CFG_FLASH_OK) {
        return 0U;
    }
    if (r.magic != MSTZERO_MAGIC || r.sum != mstzero_sum(&r)) return 0U;
    for (uint32_t j = 0U; j < 6U; j++) {
        if (r.zero[j] == 0xFFFFU) return 0U;            /* 字段越界: 整条作废 */
        z[j] = r.zero[j];
    }
    return 1U;
}

uint8_t MstZeroCfg_Clear(void) {
    uint8_t zero[32];
    for (uint32_t i = 0U; i < sizeof(zero); i++) zero[i] = 0U;
    /* 写全 0: magic 不为 MSTZERO_MAGIC, 下次 Load 直接判无效。
     * 不能整扇区擦除 —— 那会连带清掉 PID/校准零点。 */
    return CfgFlash_WriteRegion(CFG_OFF_MSTZERO, zero, (uint32_t)sizeof(zero));
}
