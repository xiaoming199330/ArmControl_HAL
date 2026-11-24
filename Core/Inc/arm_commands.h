/* arm_commands.h — 固件所有能发的指令集中配置 / 速查清单
 *
 * 把分散在 main.c 命令解析中的所有可发指令汇总到一个配置文件(本文件)
 * 里: 每条命令作为一个 CmdRegistryEntry 条目, 字段包括主名、别名、用法、
 * 帮助摘要、分组、详细说明/默认值/注意。运行时通过 cmd_registry[] 暴露
 * 给可能的帮助生成/校验工具使用, 但实际命令解析仍走原 ProcessCommand,
 * 避免大改。
 *
 * 维护约定:
 *   - 加新指令时, 在对应分组下追加条目, 同时在 main.c 的 ProcessCommand
 *     中实现并保证 cprint 帮助一致。name 字段用 token0 完整字; 同义短
 *     名放 aliases(分号分隔)。
 *   - notes 字段允许多行(用 \n 换行); 也可在条目上方/下方写 C 注释补充。
 *   - group 用来过滤打印 (如 'h' 想要分组帮助)。
 */
#ifndef __ARM_COMMANDS_H
#define __ARM_COMMANDS_H

#include <stdint.h>

typedef struct {
    const char *name;       /* 主要字(完整 token0),  例 "teach", "t", "Q"  */
    const char *aliases;    /* 同义别名, 分号 ';' 分隔, 可为 NULL            */
    const char *syntax;     /* 用法, 例 "[k k2 k3 k4]"                       */
    const char *help;       /* 一句话摘要                                       */
    const char *group;      /* 分组标签: Teach/Loop/Scan/Safety/...             */
    const char *notes;      /* 详细备注: 默认值 / 范围 / 关联 / 注意事项       */
} CmdRegistryEntry;

extern const CmdRegistryEntry cmd_registry[];   /* 全部指令集中登记     */
extern const uint16_t        cmd_registry_count;/* 元素个数               */

/* 打印本注册表到当前控制台(按 group 分组, 行宽 ~80)。可未来接到 'H' 命令。 */
void CmdRegistry_Print(void);

#endif /* __ARM_COMMANDS_H */