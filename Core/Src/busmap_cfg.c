/* ============================================================
 * busmap_cfg.c - 舵机总线路由持久化实现 (区 G, 见 busmap_cfg.h)
 * 结构 16B 自带 magic + 求和校验; 搬运用 CfgFlash 保区写,
 * A~F 区自动原样保住 (见 flash_cfg.c)。
 * ============================================================ */
#include "busmap_cfg.h"
#include "flash_cfg.h"

#define BUSMAP_MAGIC   0x42555331U   /* "BUS1" little-endian */

typedef struct {
    uint32_t magic;      /* BUSMAP_MAGIC, 区空(0xFF)/坏数据时判无效 */
    uint8_t  bus[6];     /* 每轴总线号 (0=Bus A, 1=Bus B) */
    uint8_t  mask;       /* bit j = bus[j] 来自实测, 可以采信 */
    uint8_t  pad;        /* 对齐到 4 的倍数 */
    uint32_t sum;        /* 前 12B 按 u32 求和取反 */
} BusMapRec;

/* 前 (sizeof-4) 字节按 u32 累加取反。记录必须 4 字节对齐 —— 已保证。 */
static uint32_t busmap_sum(const BusMapRec* r) {
    const uint32_t* w = (const uint32_t*)(const void*)r;
    uint32_t s = 0U;
    for (uint32_t i = 0U; i < (sizeof(BusMapRec) / 4U) - 1U; i++) s += w[i];
    return ~s;
}

uint8_t BusMapCfg_Save(const uint8_t bus[6], uint8_t mask) {
    BusMapRec r;
    if (bus == 0 || (mask & 0x3FU) == 0U) return CFG_FLASH_BADARG;

    r.magic = BUSMAP_MAGIC;
    for (uint32_t j = 0U; j < 6U; j++) {
        /* 只接受合法总线号; 越界一律当 Bus B(集成板常态), 不写入垃圾值 */
        r.bus[j] = (bus[j] == 0U) ? 0U : 1U;
    }
    r.mask = (uint8_t)(mask & 0x3FU);
    r.pad  = 0U;
    r.sum  = busmap_sum(&r);
    return CfgFlash_WriteRegion(CFG_OFF_BUSMAP, &r, (uint32_t)sizeof(r));
}

uint8_t BusMapCfg_Load(uint8_t bus[6], uint8_t* mask) {
    BusMapRec r;
    if (bus == 0 || mask == 0) return 0U;
    if (CfgFlash_ReadRegion(CFG_OFF_BUSMAP, &r, (uint32_t)sizeof(r)) != CFG_FLASH_OK) {
        return 0U;
    }
    if (r.magic != BUSMAP_MAGIC || r.sum != busmap_sum(&r)) return 0U;
    if ((r.mask & 0x3FU) == 0U) return 0U;
    for (uint32_t j = 0U; j < 6U; j++) {
        if (r.bus[j] > 1U) return 0U;            /* 字段越界: 整条记录作废 */
        bus[j] = r.bus[j];
    }
    *mask = (uint8_t)(r.mask & 0x3FU);
    return 1U;
}

uint8_t BusMapCfg_Clear(void) {
    uint8_t zero[16];
    for (uint32_t i = 0U; i < sizeof(zero); i++) zero[i] = 0U;
    /* 写全 0: magic 不为 BUSMAP_MAGIC, 下次 Load 直接判无效。
     * 不能整扇区擦除 —— 那会连带清掉 PID/校准零点。 */
    return CfgFlash_WriteRegion(CFG_OFF_BUSMAP, zero, (uint32_t)sizeof(zero));
}
