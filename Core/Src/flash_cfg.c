/* ============================================================
 * flash_cfg.c - 配置扇区(sector 11)的保区写入实现
 * 见 flash_cfg.h 的背景说明。
 *
 * 保区模型(2026-09-22 从"两区互保"泛化为"多区互保"):
 *   旧版 other_region() 只认 PID<->CAN 这一对, 加第三个区就崩
 *   (写区 C 时只会保住区 B, 区 A 的校准零点会被静默擦掉)。
 *   现在按 s_region_off[] 枚举所有区, 写任一区时把**其余各区全部**
 *   搬进 s_preserve 再回写。加区只需扩 s_region_off[] + CFG_REGION_COUNT。
 *
 * 一个省时间的判断: 仍是擦除态(全 0xFF)的区**不需要搬也不需要回写**
 *   —— 擦完它本来就是空的, 照原样搬回去等于原地踏步, 白白多烧 1024 次
 *   字编程(约 100ms)。新区第一次写入、或从未用过的区, 走这条路。
 * ============================================================ */
#include "flash_cfg.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* 保区缓冲: 擦除前把"其余各区"整块搬到这里。
 * 用 uint32_t 数组是为了保证 4 字节对齐(HAL_FLASH_Program 的 WORD 模式
 * 要求对齐地址)。
 * ⚠ 必须是 static 而不是栈上局部 —— 单区 4KB, 放栈上会把主栈打穿。
 * ⚠ 数量 = CFG_REGION_COUNT-1: 写任一区时最多有 (N-1) 个别的区要保。 */
static uint32_t s_preserve[CFG_REGION_COUNT - 1U][CFG_REGION_SPAN / 4U];

/* 合法区起点。顺序即 s_preserve 的下标顺序(按"跳过本区"的先后填)。 */
static const uint32_t s_region_off[CFG_REGION_COUNT] = {
    CFG_OFF_PID, CFG_OFF_CAN, CFG_OFF_LINK, CFG_OFF_RCSPD, CFG_OFF_RCMAP,
    CFG_OFF_GEO, CFG_OFF_BUSMAP, CFG_OFF_MSTZERO
};

/* off 是否是一个合法区起点; 是则返回它在 s_region_off 里的下标, 否则 -1。 */
static int region_index(uint32_t off)
{
    for (uint32_t i = 0U; i < CFG_REGION_COUNT; i++) {
        if (s_region_off[i] == off) return (int)i;
    }
    return -1;
}

/* 该区是否仍是擦除态(整块全 0xFF)。
 * 只读 flash, 不解锁 —— 可以随时调用。
 * 用途: 空区不必搬/回写(见文件头说明)。 */
static uint8_t region_is_blank(uint32_t off)
{
    const uint32_t* p = (const uint32_t*)(CFG_SECTOR_ADDR + off);
    for (uint32_t i = 0U; i < (CFG_REGION_SPAN / 4U); i++) {
        if (p[i] != 0xFFFFFFFFU) return 0U;
    }
    return 1U;
}

/* 逐字烧写任意字节流(不要求 src 对齐, 也不需要额外的对齐副本 ——
 * 省下的 4KB RAM 对一个 bss 已经 60KB+ 的工程是值得的)。
 * 尾部不足 4 字节用 0xFF 补齐: 全 1 与擦除态相同, 属于空操作。 */
static uint8_t program_bytes(uint32_t addr, const void* src, uint32_t n)
{
    const uint8_t* b = (const uint8_t*)src;
    uint32_t n_words = (n + 3U) & ~3U;

    for (uint32_t off = 0U; off < n_words; off += 4U) {
        uint32_t w = 0xFFFFFFFFU;
        uint32_t m = n - off;
        if (m > 4U) m = 4U;
        if (m > 0U) memcpy(&w, b + off, m);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + off, w) != HAL_OK) {
            return CFG_FLASH_ERROR;
        }
    }
    return CFG_FLASH_OK;
}

uint8_t CfgFlash_ReadRegion(uint32_t off, void* dst, uint32_t n)
{
    if (!dst || n == 0U || n > CFG_REGION_SPAN) return CFG_FLASH_BADARG;
    if (region_index(off) < 0) return CFG_FLASH_BADARG;
    memcpy(dst, (const void*)(CFG_SECTOR_ADDR + off), n);
    return CFG_FLASH_OK;
}

uint8_t CfgFlash_WriteRegion(uint32_t off, const void* src, uint32_t n)
{
    uint32_t keep[CFG_REGION_COUNT - 1U];   /* 本次要保的区起点 */
    uint32_t nkeep = 0U;
    uint32_t addr;
    uint8_t  rc;
    int      idx;

    if (!src || n == 0U || n > CFG_REGION_SPAN) return CFG_FLASH_BADARG;
    idx = region_index(off);
    if (idx < 0) return CFG_FLASH_BADARG;

    addr = CFG_SECTOR_ADDR + off;

    /* 1) 先把"其余各区"整块搬到 RAM(擦除会连带清掉它们)。
     *    仍是空的区跳过 —— 省一次 4KB 回写。 */
    for (uint32_t i = 0U; i < CFG_REGION_COUNT; i++) {
        if ((int)i == idx) continue;
        if (region_is_blank(s_region_off[i])) continue;
        keep[nkeep] = s_region_off[i];
        memcpy(s_preserve[nkeep], (const void*)(CFG_SECTOR_ADDR + s_region_off[i]),
               CFG_REGION_SPAN);
        nkeep++;
    }

    HAL_FLASH_Unlock();

    /* 2) 擦整扇区 */
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t error = 0U;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = CFG_SECTOR_INDEX;
    erase.NbSectors = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    rc = (HAL_FLASHEx_Erase(&erase, &error) == HAL_OK)
             ? CFG_FLASH_OK : CFG_FLASH_ERROR;

    /* 3) 回写要保的区, 再写本区 */
    for (uint32_t i = 0U; i < nkeep && rc == CFG_FLASH_OK; i++) {
        rc = program_bytes(CFG_SECTOR_ADDR + keep[i],
                           (const void*)s_preserve[i], CFG_REGION_SPAN);
    }
    if (rc == CFG_FLASH_OK) {
        rc = program_bytes(addr, src, n);
    }

    HAL_FLASH_Lock();

    /* 4) 校验: 本区与每个被保的区都要字节一致 */
    if (rc == CFG_FLASH_OK) {
        if (memcmp((const void*)addr, src, n) != 0) rc = CFG_FLASH_ERROR;
    }
    for (uint32_t i = 0U; i < nkeep && rc == CFG_FLASH_OK; i++) {
        if (memcmp((const void*)(CFG_SECTOR_ADDR + keep[i]), s_preserve[i],
                   CFG_REGION_SPAN) != 0) {
            rc = CFG_FLASH_ERROR;
        }
    }
    return rc;
}

uint8_t CfgFlash_EraseAll(void)
{
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t error = 0U;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = CFG_SECTOR_INDEX;
    erase.NbSectors = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&erase, &error);
    HAL_FLASH_Lock();
    return (st == HAL_OK) ? CFG_FLASH_OK : CFG_FLASH_ERROR;
}

uint8_t CfgFlash_Ready(void) { return 1U; }
