/* ============================================================
 * geo_cfg.c - 几何偏好持久化实现 (区 F, 见 geo_cfg.h)
 * v1 24B / v2 28B / v3 60B 自带 magic + 求和校验; 搬运用 CfgFlash
 * 保区写, A~E/G/H 区自动原样保住 (见 flash_cfg.c)。
 * v2 (0924BV): +zf_on +margin_raw; Load 兼容旧 v1 记录。
 * v3 (0924CG): +fold_j[6] +fold_valid (P here 折叠位标定)。
 *   前 28B 与 v2 布局逐字节一致; fold 字段追加在 28..55, sum2 在
 *   56..59 覆盖前 56B —— 旧 v2 固件读 v3 记录仍过 v2 校验 (双向兼容)。
 *   s_fold_j/s_fold_valid 缓存最近一次 Load3/Save3 的 fold 字段,
 *   旧包装 Save2/Save 写区时自动带上, 标定不会被顺手抹掉。
 * ============================================================ */
#include "geo_cfg.h"
#include "flash_cfg.h"
#include <string.h>

#define GEO_MAGIC   0x47454F31U   /* "GEO1" little-endian (v1/v2/v3 同 magic) */

/* v2 记录: 28B, sum 在 offset 24 (覆盖前 24B) */
typedef struct {
    uint32_t magic;      /* 0..3   GEO_MAGIC, 区空(0xFF)/坏数据时判无效 */
    float    dx;         /* 4..7   C 偏移 X (actual - command, mm) */
    float    dy;         /* 8..11  C 偏移 Y */
    float    dz;         /* 12..15 C 偏移 Z */
    float    zf_mm;      /* 16..19 zf: 爪尖最低离桌 (mm, 用户语义) */
    uint8_t  zf_valid;   /* 20     1 = 用户设过 zf, zf_mm 有效 */
    uint8_t  zf_on;      /* 21     v2: 流控/遥操桌面限位启用 */
    uint16_t margin;     /* 22..23 v2: fi margin raw (0xFFFF = 未设) */
    uint32_t sum;        /* 24..27 前 24B 按 u32 求和取反 (无隐式 padding) */
} GeoRec;                /* 28B, 字段已手工对齐, 编译器不加隐式 padding */

/* v3 记录 (0924CG): 60B。前 28B == GeoRec (v2), fold 字段追加 */
typedef struct {
    uint32_t magic;
    float    dx, dy, dz, zf_mm;
    uint8_t  zf_valid;
    uint8_t  zf_on;
    uint16_t margin;
    uint32_t sum;        /* 24..27 前 24B 求和取反 (与 v2 同, 兼容旧固件) */
    float    fold_j[6];  /* 28..51 P here 标定: relhome 关节角 (deg) */
    uint8_t  fold_valid; /* 52     1 = 标定过 */
    uint8_t  pad[3];     /* 53..55 对齐填充 */
    uint32_t sum2;       /* 56..59 前 56B 求和取反 (v3 新增) */
} GeoRecV3;              /* 60B */

/* v1 记录 (旧固件): 24B, sum 在 offset 20 */
typedef struct {
    uint32_t magic;
    float    dx, dy, dz, zf_mm;
    uint8_t  zf_valid;
    uint8_t  pad[3];
    uint32_t sum;        /* 前 20B 按 u32 求和取反 */
} GeoRecV1;

/* fold 字段缓存: 旧包装 (Save2/Save) 写区时带上, 防止抹掉 P here 标定 */
static float    s_fold_j[6]   = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static uint8_t  s_fold_valid  = 0U;

/* 前 (bytes/4) 个 u32 累加取反。记录必须 4 字节对齐 —— 已保证。 */
static uint32_t geo_sum(const void* rec, uint32_t bytes) {
    const uint32_t* w = (const uint32_t*)(const void*)rec;
    uint32_t s = 0U;
    for (uint32_t i = 0U; i < bytes / 4U; i++) s += w[i];
    return ~s;
}

/* v3 保存 (全字段)。fold_j6==NULL 时用缓存值。 */
uint8_t GeoCfg_Save3(float dx, float dy, float dz,
                     float zf_mm, uint8_t zf_valid,
                     uint8_t zf_on, uint16_t margin_raw,
                     const float* fold_j6, uint8_t fold_valid) {
    GeoRecV3 r;
    if (fold_j6 != 0) {
        memcpy(s_fold_j, fold_j6, sizeof(s_fold_j));
        s_fold_valid = fold_valid ? 1U : 0U;
    }
    memset(&r, 0, sizeof(r));
    r.magic     = GEO_MAGIC;
    r.dx        = dx;
    r.dy        = dy;
    r.dz        = dz;
    r.zf_mm     = zf_mm;
    r.zf_valid  = zf_valid ? 1U : 0U;
    r.zf_on     = zf_on ? 1U : 0U;
    r.margin    = margin_raw;
    r.sum       = geo_sum(&r, 24U);          /* 与 v2 同口径 (前 24B) */
    memcpy(r.fold_j, s_fold_j, sizeof(r.fold_j));
    r.fold_valid = s_fold_valid;
    r.sum2      = geo_sum(&r, (uint32_t)sizeof(r) - 4U);
    return CfgFlash_WriteRegion(CFG_OFF_GEO, &r, (uint32_t)sizeof(r));
}

/* v3 读取。兼容 v3/v2/v1 记录; 输出参数可为 NULL。 */
uint8_t GeoCfg_Load3(float* dx, float* dy, float* dz,
                     float* zf_mm, uint8_t* zf_valid,
                     uint8_t* zf_on, uint16_t* margin_raw,
                     float* fold_j6, uint8_t* fold_valid) {
    GeoRecV3 r;
    if (CfgFlash_ReadRegion(CFG_OFF_GEO, &r, (uint32_t)sizeof(r)) != CFG_FLASH_OK) {
        return 0U;
    }
    if (r.magic != GEO_MAGIC) return 0U;

    /* 1) v3: sum2 覆盖前 56B */
    if (r.sum2 == geo_sum(&r, (uint32_t)sizeof(r) - 4U)) {
        if (dx)        *dx        = r.dx;
        if (dy)        *dy        = r.dy;
        if (dz)        *dz        = r.dz;
        if (zf_mm)     *zf_mm     = r.zf_mm;
        if (zf_valid)  *zf_valid  = r.zf_valid ? 1U : 0U;
        if (zf_on)     *zf_on     = r.zf_on ? 1U : 0U;
        if (margin_raw)*margin_raw= r.margin;
        if (r.fold_valid) {
            memcpy(s_fold_j, r.fold_j, sizeof(s_fold_j));
        }
        s_fold_valid = r.fold_valid ? 1U : 0U;
        if (fold_j6)   memcpy(fold_j6, s_fold_j, sizeof(s_fold_j));
        if (fold_valid)*fold_valid = s_fold_valid;
        return 1U;
    }
    /* 2) v2: 28B, sum 覆盖前 24B */
    {
        GeoRec v2;
        memcpy(&v2, &r, sizeof(v2));
        if (v2.sum == geo_sum(&v2, 24U)) {
            if (dx)        *dx        = v2.dx;
            if (dy)        *dy        = v2.dy;
            if (dz)        *dz        = v2.dz;
            if (zf_mm)     *zf_mm     = v2.zf_mm;
            if (zf_valid)  *zf_valid  = v2.zf_valid ? 1U : 0U;
            if (zf_on)     *zf_on     = v2.zf_on ? 1U : 0U;
            if (margin_raw)*margin_raw= v2.margin;
            s_fold_valid = 0U;
            if (fold_j6)   memcpy(fold_j6, s_fold_j, sizeof(s_fold_j));
            if (fold_valid)*fold_valid = 0U;
            return 1U;
        }
    }
    /* 3) v1 (旧固件 24B): 按旧布局重新校验, 新字段走默认 */
    {
        GeoRecV1 v1;
        if (CfgFlash_ReadRegion(CFG_OFF_GEO, &v1, (uint32_t)sizeof(v1)) != CFG_FLASH_OK) {
            return 0U;
        }
        if (v1.magic != GEO_MAGIC || v1.sum != geo_sum(&v1, (uint32_t)sizeof(v1) - 4U)) {
            return 0U;
        }
        if (dx)        *dx        = v1.dx;
        if (dy)        *dy        = v1.dy;
        if (dz)        *dz        = v1.dz;
        if (zf_mm)     *zf_mm     = v1.zf_mm;
        if (zf_valid)  *zf_valid  = v1.zf_valid ? 1U : 0U;
        if (zf_on)     *zf_on     = 1U;                 /* v1 无此字段: 默认启用 */
        if (margin_raw)*margin_raw= GEO_MARGIN_UNSET;
        s_fold_valid = 0U;
        if (fold_j6)   memcpy(fold_j6, s_fold_j, sizeof(s_fold_j));
        if (fold_valid)*fold_valid = 0U;
        return 1U;
    }
}

uint8_t GeoCfg_Save2(float dx, float dy, float dz,
                     float zf_mm, uint8_t zf_valid,
                     uint8_t zf_on, uint16_t margin_raw) {
    /* 旧包装: fold 字段走缓存 (Load3/Save3 维护), 不抹标定 */
    return GeoCfg_Save3(dx, dy, dz, zf_mm, zf_valid, zf_on, margin_raw,
                        0, s_fold_valid);
}

uint8_t GeoCfg_Save(float dx, float dy, float dz,
                    float zf_mm, uint8_t zf_valid) {
    return GeoCfg_Save3(dx, dy, dz, zf_mm, zf_valid, 1U, GEO_MARGIN_UNSET,
                        0, s_fold_valid);
}

uint8_t GeoCfg_Load2(float* dx, float* dy, float* dz,
                     float* zf_mm, uint8_t* zf_valid,
                     uint8_t* zf_on, uint16_t* margin_raw) {
    return GeoCfg_Load3(dx, dy, dz, zf_mm, zf_valid, zf_on, margin_raw,
                        0, 0);
}

uint8_t GeoCfg_Load(float* dx, float* dy, float* dz,
                    float* zf_mm, uint8_t* zf_valid) {
    uint8_t zon; uint16_t mrg;
    return GeoCfg_Load3(dx, dy, dz, zf_mm, zf_valid, &zon, &mrg, 0, 0);
}
