/*
 * 命名示教文件存储（STM32 内部 Flash, 断电保留）。
 *
 * 布局：两个 256KB 轮换区（区域A = S9+S10 @0x080A0000，区域B = S7+S8 @0x08060000），
 * 当前盘 = 有效区头(magic + 代次)较大的那个；文件按序连续追加。
 * 每条：24B 头（样本数 | flags | CRC | 名字） + 12B×样本(raw[6])。
 * 删除写墓碑(flags 1->0)；delete 的空间需 compact() 轮换到另一区回收。
 * 注意：pid_flash 已占用 S11(0x080E0000)，不要使用。
 */
#ifndef __TEACH_STORE_H
#define __TEACH_STORE_H

#include <stdint.h>

#define TSTORE_NAME_MAX      11U   /* 最长文件名（不含 NUL） */
#define TSTORE_MAX_FILES     32U

/* 错误码 */
#define TSTORE_OK            0U
#define TSTORE_ERR_NAME      1U    /* 非法名字 */
#define TSTORE_ERR_DUP       2U    /* 重名(内部使用, 保存时自动覆盖) */
#define TSTORE_ERR_DATA      3U    /* 样本指针为空 */
#define TSTORE_ERR_FULL      4U    /* 当前盘空间不足(需 compact) */
#define TSTORE_ERR_FLASH     5U    /* 写/擦 Flash 失败 */
#define TSTORE_ERR_LIMIT     6U    /* 文件数量超限 */
#define TSTORE_ERR_NFOUND    7U    /* 未找到 */

/* 上电扫描两个区, 加载文件目录。返回 OK 表示至少发现一个有效盘。 */
uint8_t TeachStore_Init(void);

/* 保存新文件（重名自动覆盖旧文件）。samples 为 n 条 raw[6]。 */
uint8_t TeachStore_Save(const char *name, const uint16_t (*samples)[6], uint32_t n);

/* 删除：name 或 1-based 序号（如 "3"）。仅写墓碑。 */
uint8_t TeachStore_Delete(const char *name);

/* 整理：把当前盘有效文件搬入另一区并切换代次（回收墓碑空间）。耗时数秒。 */
uint8_t TeachStore_Compact(void);

uint32_t TeachStore_Count(void);
const char *TeachStore_FileName(uint32_t idx);          /* NULL 越界 */
uint32_t TeachStore_FileSamples(uint32_t idx);          /* 0 = 无 */
const uint16_t (*TeachStore_FileData(uint32_t idx))[6]; /* NULL = 无 */
/* 通过 name 或 1-based 序号定位, 返回目录下标或 0xFFFFFFFF */
uint32_t TeachStore_Resolve(const char *name_or_idx);
uint32_t TeachStore_TotalSamples(void);
uint32_t TeachStore_FreeBytes(void);

#endif /* __TEACH_STORE_H */
