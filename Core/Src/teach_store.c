/*
 * teach_store.c —— 命名示教文件的内部 Flash 持久存储（见 teach_store.h）。
 *
 * 读写安全模型：
 *  - 每条记录先写好数据与头，最后才更新“区头”（magic+代次），
 *    任何时刻断电只可能丢“正在写的那条/那次整理”，旧盘数据仍完好。
 *  - 扫描时逐条 CRC 校验，遇到坏尾部即停止，损坏尾部视为空闲。
 *  - 编程/擦除执行期间 CPU 会被 Flash 接口拖住（数秒内不可中断），可接受。
 */
#include "teach_store.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdlib.h>

#define TS_MAGIC         0x54524331UL   /* "TRC1" */
#define TS_HEADER_BYTES  16U            /* 区头: magic|gen|rsvd|rsvd */
#define TS_REGION_SIZE   (256U * 1024U)
#define TS_ENTRY_HDR     24U            /* 条目头: num|flags|crc|name[12] */
#define TS_FLAG_ACTIVE   1U

typedef struct { uint32_t base; uint8_t first_sector; } TsRegion;
static const TsRegion ts_regions[2] = {
    {0x080A0000U, FLASH_SECTOR_9},   /* S9+S10 */
    {0x08060000U, FLASH_SECTOR_7},   /* S7+S8  */
};

typedef struct {
    char     name[TSTORE_NAME_MAX + 1U];
    uint32_t off;      /* 条目头在活动区内的字节偏移 */
    uint32_t num;      /* 样本数 */
} TsFile;

static TsFile   ts_dir[TSTORE_MAX_FILES];
static uint32_t ts_count = 0U;
static uint32_t ts_active_base = 0U;
static uint32_t ts_active_gen = 0U;
static uint32_t ts_tail = TS_HEADER_BYTES;
static uint8_t  ts_tail_corrupt = 0U;   /* 扫描发现坏尾部(断电写中断) */

/* ---------- 内部工具 ---------- */
static uint32_t ts_crc32(const uint8_t *p, uint32_t n) {
    uint32_t crc = 0xFFFFFFFFU;
    for (uint32_t i = 0; i < n; i++) {
        crc ^= p[i];
        for (uint8_t b = 0; b < 8U; b++)
            crc = (crc >> 1) ^ (0xEDB88320UL & (uint32_t)(-(int32_t)(crc & 1U)));
    }
    return ~crc;
}

static void ts_erase_region(uint32_t ridx) {
    FLASH_EraseInitTypeDef e;
    uint32_t err = 0U;
    memset(&e, 0, sizeof(e));
    e.TypeErase = FLASH_TYPEERASE_SECTORS;
    e.Sector = ts_regions[ridx].first_sector;
    e.NbSectors = 2;
    e.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    (void)HAL_FLASHEx_Erase(&e, &err);
}

/* src 只要求 4B 长度、逐字拷贝（兼容任意源对齐） */
static void ts_program_words(uint32_t addr, const uint8_t *src, uint32_t len) {
    for (uint32_t n = 0; n < len; n += 4U) {
        uint32_t w;
        memcpy(&w, src + n, 4U);
        (void)HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + n, w);
    }
}

static uint32_t ts_region_index_of(uint32_t base) {
    for (uint32_t i = 0; i < 2U; i++)
        if (ts_regions[i].base == base) return i;
    return 0U;
}

/* ---------- 上电扫描 ---------- */
static void ts_detect_active(void) {
    ts_active_base = 0U;
    ts_active_gen = 0U;
    for (uint32_t i = 0; i < 2U; i++) {
        const uint32_t *h = (const uint32_t *)ts_regions[i].base;
        if (h[0] == TS_MAGIC) {
            uint32_t g = h[1];
            if (g >= ts_active_gen) { ts_active_gen = g; ts_active_base = ts_regions[i].base; }
        }
    }
}

static void ts_load_dir(void) {
    ts_count = 0U;
    ts_tail = TS_HEADER_BYTES;
    ts_tail_corrupt = 0U;
    if (ts_active_base == 0U) return;

    uint32_t off = TS_HEADER_BYTES;
    while (ts_count < TSTORE_MAX_FILES) {
        if (off + TS_ENTRY_HDR > TS_REGION_SIZE) break;          /* 正常到顶 */
        const uint8_t *p = (const uint8_t *)(ts_active_base + off);
        uint32_t num, flags, crc;
        memcpy(&num, p, 4);
        if (num == 0xFFFFFFFFU) break;                            /* 正常空闲 */
        memcpy(&flags, p + 4, 4);
        memcpy(&crc, p + 8, 4);
        if (num > (TS_REGION_SIZE - off - TS_ENTRY_HDR) / 12U) { ts_tail_corrupt = 1U; break; }
        /* 名字: 12B 域内须有 NUL, 字符只允许字母/数字/_- */
        char nm[TSTORE_NAME_MAX + 1U];
        uint8_t name_ok = 0U;
        memset(nm, 0, sizeof(nm));
        for (uint8_t j = 0; j < 12U; j++) {
            char c = (char)p[12U + j];
            if (c == 0) { name_ok = 1U; break; }
            if (j >= TSTORE_NAME_MAX) break;                      /* 超长 */
            if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') ||
                  (c >= 'A' && c <= 'Z') || c == '_' || c == '-')) break;
            nm[j] = c;
        }
        if (!name_ok) { ts_tail_corrupt = 1U; break; }
        uint32_t crc_calc = ts_crc32((const uint8_t *)(ts_active_base + off + TS_ENTRY_HDR),
                                     num * 12U);
        if (crc_calc != crc) { ts_tail_corrupt = 1U; break; }     /* 坏尾部 */
        if (flags == TS_FLAG_ACTIVE) {
            TsFile *f = &ts_dir[ts_count];
            memcpy(f->name, nm, TSTORE_NAME_MAX);
            f->name[TSTORE_NAME_MAX] = 0;
            f->off = off;
            f->num = num;
            ts_count++;
        }
        off += TS_ENTRY_HDR + num * 12U;
    }
    ts_tail = off;
}

/* 给目录补一条（插入到尾部；仅在确定写入成功后调用） */
static uint8_t ts_dir_add(const char *name, uint32_t off, uint32_t num) {
    if (ts_count >= TSTORE_MAX_FILES) return TSTORE_ERR_LIMIT;
    TsFile *f = &ts_dir[ts_count++];
    memset(f, 0, sizeof(*f));
    size_t l = strlen(name);
    if (l > TSTORE_NAME_MAX) l = TSTORE_NAME_MAX;
    memcpy(f->name, name, l);
    f->off = off;
    f->num = num;
    return TSTORE_OK;
}

/* 名字/序号的合法性与已存在检查 */
static uint8_t ts_name_ok(const char *name) {
    if (!name) return 0U;
    size_t l = strlen(name);
    if (l == 0U || l > TSTORE_NAME_MAX) return 0U;
    for (size_t i = 0; i < l; i++) {
        char c = name[i];
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') ||
              (c >= 'A' && c <= 'Z') || c == '_' || c == '-'))
            return 0U;
    }
    return 1U;
}

/* ---------- 公开接口 ---------- */
/* 扫描某区(只数数, 不写目录): 返回有效条目数与坏尾标志 */
static void ts_probe_region(uint32_t ridx, uint32_t *entries, uint8_t *corrupt, uint32_t *gen) {
    *entries = 0U;
    *corrupt = 0U;
    *gen = 0U;
    uint32_t base = ts_regions[ridx].base;
    const uint32_t *h = (const uint32_t *)base;
    if (h[0] != TS_MAGIC) return;
    *gen = h[1];
    uint32_t off = TS_HEADER_BYTES;
    while (1) {
        if (off + TS_ENTRY_HDR > TS_REGION_SIZE) break;
        const uint8_t *p = (const uint8_t *)(base + off);
        uint32_t num, flags, crc;
        memcpy(&num, p, 4);
        if (num == 0xFFFFFFFFU) break;
        memcpy(&flags, p + 4, 4);
        memcpy(&crc, p + 8, 4);
        if (num > (TS_REGION_SIZE - off - TS_ENTRY_HDR) / 12U) { *corrupt = 1U; break; }
        uint8_t ok = 0U;
        for (uint8_t j = 0; j < 12U; j++) {
            char c = (char)p[12U + j];
            if (c == 0) { ok = 1U; break; }
            if (j >= TSTORE_NAME_MAX) break;
            if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') ||
                  (c >= 'A' && c <= 'Z') || c == '_' || c == '-')) break;
        }
        if (!ok) { *corrupt = 1U; break; }
        if (ts_crc32((const uint8_t *)(base + off + TS_ENTRY_HDR), num * 12U) != crc) {
            *corrupt = 1U; break;
        }
        if (flags == TS_FLAG_ACTIVE) (*entries)++;
        off += TS_ENTRY_HDR + num * 12U;
    }
}

uint8_t TeachStore_Init(void) {
    /* 选"可用"活动区: 有有效条目的优先, 再比代次(避免坏/空区误导代次判断) */
    uint32_t best_idx = 0U, best_entries = 0U, best_gen = 0U;
    uint8_t  found = 0U;
    for (uint32_t i = 0; i < 2U; i++) {
        uint32_t ent, gen; uint8_t corr;
        ts_probe_region(i, &ent, &corr, &gen);
        if (gen == 0U && ent == 0U) continue;   /* 无区头 */
        if (!found || (ent > 0U && best_entries == 0U) ||
            ((ent > 0U) == (best_entries > 0U) && gen > best_gen)) {
            best_idx = i; best_entries = ent; best_gen = gen; found = 1U;
        }
    }
    ts_active_base = found ? ts_regions[best_idx].base : 0U;
    ts_active_gen = found ? best_gen : 0U;
    ts_load_dir();

    if (ts_tail_corrupt) {
        if (ts_count > 0U) {
            /* 断电写中断留下的坏尾: 自动整理到另一区自愈 */
            (void)TeachStore_Compact();
            ts_detect_active();
            ts_load_dir();
        } else {
            ts_active_base = 0U;   /* 无有效文件: 下次保存重新格式化 */
        }
    }
    return (ts_active_base != 0U) ? TSTORE_OK : TSTORE_ERR_FLASH;
}

uint8_t TeachStore_Save(const char *name, const uint16_t (*samples)[6], uint32_t n) {
    if (!ts_name_ok(name)) return TSTORE_ERR_NAME;
    if (!samples || n == 0U) return TSTORE_ERR_DATA;

    /* 活动区未格式化：选区域0, 先擦除并写区头 */
    HAL_FLASH_Unlock();
    if (ts_active_base == 0U) {
        ts_active_base = ts_regions[0].base;
        ts_active_gen = 0U;
        ts_erase_region(0U);
        uint32_t h[4] = {TS_MAGIC, ts_active_gen, 0U, 0U};
        ts_program_words(ts_active_base, (const uint8_t *)h, sizeof(h));
        ts_tail = TS_HEADER_BYTES;
        ts_count = 0U;
    }
    HAL_FLASH_Lock();

    uint32_t need = TS_ENTRY_HDR + n * 12U;
    uint32_t free_now = TS_REGION_SIZE - ts_tail;

    /* 重名覆盖：先墓碑旧文件 */
    uint32_t dup = 0xFFFFFFFFU;
    for (uint32_t i = 0; i < ts_count; i++)
        if (strcmp(ts_dir[i].name, name) == 0) { dup = i; break; }
    if (dup != 0xFFFFFFFFU) {
        (void)TeachStore_Delete(name);   /* 物理尾部不动, 只写墓碑+移目录 */
        free_now = TS_REGION_SIZE - ts_tail;
    }

    if (free_now < need) {
        /* 空间不足: 尝试整理回收墓碑区 */
        uint8_t st = TeachStore_Compact();
        if (st != TSTORE_OK && st != TSTORE_ERR_NFOUND) return TSTORE_ERR_FULL;
        free_now = TS_REGION_SIZE - ts_tail;
        if (free_now < need) return TSTORE_ERR_FULL;
    }

    uint32_t dst = ts_active_base + ts_tail;

    HAL_FLASH_Unlock();
    /* 先写数据, 再写条目头；坏尾部只会被扫描当作空闲 */
    uint8_t head[TS_ENTRY_HDR];
    uint32_t crc = ts_crc32((const uint8_t *)samples, n * 12U);
    uint32_t v;
    v = n;      memcpy(head, &v, 4);
    v = TS_FLAG_ACTIVE; memcpy(head + 4, &v, 4);
    v = crc;    memcpy(head + 8, &v, 4);
    memset(head + 12, 0, 12);
    memcpy(head + 12, name, strlen(name));

    ts_program_words(dst + TS_ENTRY_HDR, (const uint8_t *)samples, n * 12U);
    ts_program_words(dst, head, TS_ENTRY_HDR);

    /* 回读校验 */
    uint32_t rn, rf, rc;
    memcpy(&rn, (const void *)(dst), 4);
    memcpy(&rf, (const void *)(dst + 4), 4);
    memcpy(&rc, (const void *)(dst + 8), 4);
    uint32_t crc2 = ts_crc32((const uint8_t *)(dst + TS_ENTRY_HDR), n * 12U);
    if (rn != n || rf != TS_FLAG_ACTIVE || rc != crc2 || crc2 != crc) {
        HAL_FLASH_Lock();
        return TSTORE_ERR_FLASH;
    }
    HAL_FLASH_Lock();

    if (ts_count >= TSTORE_MAX_FILES) return TSTORE_ERR_LIMIT;
    ts_dir_add(name, ts_tail, n);
    ts_tail += need;
    return TSTORE_OK;
}

uint8_t TeachStore_Delete(const char *name) {
    if (!name || !*name) return TSTORE_ERR_NFOUND;
    uint32_t idx = TeachStore_Resolve(name);
    if (idx == 0xFFFFFFFFU) return TSTORE_ERR_NFOUND;

    uint32_t off = ts_active_base + ts_dir[idx].off;
    uint32_t zero = 0U;
    HAL_FLASH_Unlock();
    (void)HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, off + 4U, zero); /* flags -> 0 */
    HAL_FLASH_Lock();
    uint32_t rf;
    memcpy(&rf, (const void *)(off + 4U), 4);
    if (rf != 0U) return TSTORE_ERR_FLASH;

    /* 目录前移 */
    for (uint32_t i = idx; i + 1U < ts_count; i++) ts_dir[i] = ts_dir[i + 1U];
    ts_count--;
    return TSTORE_OK;
}

uint8_t TeachStore_Compact(void) {
    if (ts_active_base == 0U || ts_count == 0U) {
        /* 无有效盘: 重置为区域0(空) */
        ts_detect_active();
        return ts_count == 0U ? TSTORE_ERR_NFOUND : TSTORE_OK;
    }

    uint32_t src_idx = ts_region_index_of(ts_active_base);
    uint32_t dst_idx = 1U - src_idx;
    uint32_t dst = ts_regions[dst_idx].base;

    HAL_FLASH_Unlock();
    ts_erase_region(dst_idx);

    uint32_t off = TS_HEADER_BYTES;
    for (uint32_t i = 0; i < ts_count; i++) {
        TsFile *f = &ts_dir[i];
        uint32_t sz = TS_ENTRY_HDR + f->num * 12U;
        /* 原样搬条目(含头) */
        ts_program_words(dst + off, (const uint8_t *)(ts_active_base + f->off), sz);
        f->off = off;          /* 目录偏移更新到新区 */
        off += sz;
    }
    /* 最后写区头(代次+1): 断电最多丢本次整理 */
    uint32_t h[4] = {TS_MAGIC, ts_active_gen + 1U, 0U, 0U};
    ts_program_words(dst, (const uint8_t *)h, sizeof(h));
    HAL_FLASH_Lock();

    uint32_t chk;
    memcpy(&chk, (const void *)dst, 4);
    if (chk != TS_MAGIC) return TSTORE_ERR_FLASH;

    /* 切换活动区并重载目录（偏移仍适用: 按同序搬移） */
    ts_active_base = dst;
    ts_active_gen++;
    ts_tail = off;
    return TSTORE_OK;
}

uint32_t TeachStore_Count(void)             { return ts_count; }
const char *TeachStore_FileName(uint32_t idx) {
    return (idx < ts_count) ? ts_dir[idx].name : NULL;
}
uint32_t TeachStore_FileSamples(uint32_t idx) {
    return (idx < ts_count) ? ts_dir[idx].num : 0U;
}
const uint16_t (*TeachStore_FileData(uint32_t idx))[6] {
    if (idx >= ts_count || ts_active_base == 0U) return NULL;
    return (const uint16_t (*)[6])(ts_active_base + ts_dir[idx].off + TS_ENTRY_HDR);
}

uint32_t TeachStore_Resolve(const char *name_or_idx) {
    if (!name_or_idx || !*name_or_idx) return 0xFFFFFFFFU;
    if (ts_name_ok(name_or_idx)) {
        for (uint32_t i = 0; i < ts_count; i++)
            if (strcmp(ts_dir[i].name, name_or_idx) == 0) return i;
    }
    /* 纯数字 = 1-based 序号 */
    char *end = NULL;
    long v = strtol(name_or_idx, &end, 10);
    if (end && *end == 0 && v >= 1 && (unsigned long)v <= ts_count)
        return (uint32_t)(v - 1L);
    return 0xFFFFFFFFU;
}

uint32_t TeachStore_TotalSamples(void) {
    uint32_t s = 0U;
    for (uint32_t i = 0; i < ts_count; i++) s += ts_dir[i].num;
    return s;
}

uint32_t TeachStore_FreeBytes(void) {
    return (ts_active_base == 0U) ? TS_REGION_SIZE - TS_HEADER_BYTES
                                  : TS_REGION_SIZE - ts_tail;
}
