/* ============================================================
 * arm_fault.c — 见 arm_fault.h 顶部的说明(为什么要用 CCM RAM 里的固定地址)
 * ============================================================ */
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"   /* HAL_GetTick: 只给 ArmStall_Tick 用(它不是故障处理器) */
#include <string.h>
#include "arm_fault.h"

/* CCM RAM 64KB: 0x10000000 .. 0x1000FFFF。
 * 取**顶部往下 4KB** 的绝对地址, 而不是用 __attribute__((section)):
 *   - .ccmram 段在本工程恒为空, 这个位置不会被任何东西占;
 *   - 不走链接脚本 ⇒ 万一以后有人往 .ccmram 里塞东西, 这里是显式地址,
 *     一眼能看出冲突, 而不是变成一条"改了 .ld 之后莫名其妙多出 12 字节"。
 * ⚠ 结构体必须 <= 4KB, 且只放这一份记录。 */
#define ARM_FAULT_REC_ADDR  ((ArmFaultRec*)0x1000F000U)

void ArmFault_Mark(uint32_t code, uint32_t exc_return)
{
    ArmFaultRec* r = ARM_FAULT_REC_ADDR;

    /* 先失效再填 —— 万一在填的过程中又来了第二个故障, 读到的会是一条
     * 不完整的记录(magic 没写上), 而不是一条"半新半旧"的假记录。 */
    r->magic = 0U;
    r->inv_magic = 0U;

    r->code       = code;
    r->cfsr       = SCB->CFSR;
    r->hfsr       = SCB->HFSR;
    r->bfar       = SCB->BFAR;
    r->mmfar      = SCB->MMFAR;
    r->exc_return = exc_return;
    r->ipsr       = __get_IPSR();

    /* 最后一行才让它生效。 */
    r->inv_magic = ~ARM_FAULT_MAGIC;
    r->magic     = ARM_FAULT_MAGIC;
}

uint8_t ArmFault_WasFault(void)
{
    const ArmFaultRec* r = ARM_FAULT_REC_ADDR;
    return (uint8_t)((r->magic == ARM_FAULT_MAGIC &&
                      r->inv_magic == ~ARM_FAULT_MAGIC) ? 1U : 0U);
}

void ArmFault_Read(ArmFaultRec* out)
{
    if (out == NULL) return;
    *out = *ARM_FAULT_REC_ADDR;
    if (!ArmFault_WasFault()) {
        out->code = ARM_FAULT_NONE;
    }
}

void ArmFault_Clear(void)
{
    ArmFaultRec* r = ARM_FAULT_REC_ADDR;
    r->magic = 0U;
    r->inv_magic = 0U;
}

const char* ArmFault_Name(uint32_t code)
{
    switch (code) {
    case ARM_FAULT_HARD:  return "HardFault";
    case ARM_FAULT_MEM:   return "MemManage";
    case ARM_FAULT_BUS:   return "BusFault";
    case ARM_FAULT_USAGE: return "UsageFault";
    default:              return "?";
    }
}

const char* ArmFault_AddrHint(uint32_t addr)
{
    /* 只列本次排查真正会用到的几个。命中就基本能定死结论。 */
    if (addr == 0U) {
        return "BFAR 无效(访问被拒但硬件没给出地址), 或出错时未取地址";
    }
    if (addr >= CAN1_BASE && addr < (CAN1_BASE + 0x400U)) {
        return "★ CAN1 寄存器 (0x40006400) ⇒ 读写 CAN1 时出错";
    }
    if (addr >= CAN2_BASE && addr < (CAN2_BASE + 0x400U)) {
        return "★★ CAN2 寄存器 (0x40006800) ⇒ 时钟没开/实例未初始化就去读写 CAN2";
    }
    if (addr >= GPIOA_BASE && addr < (GPIOH_BASE + 0x400U)) {
        return "GPIO 寄存器域 ⇒ 查引脚时钟(__HAL_RCC_GPIOx_CLK_ENABLE)是否开过";
    }
    if (addr >= FLASH_R_BASE && addr < (FLASH_R_BASE + 0x400U)) {
        return "FLASH 控制器 ⇒ 查 flash_cfg 的扇区保护/解锁流程(红线 5)";
    }
    if (addr < 0x20000000U) {
        return "地址低于 SRAM ⇒ 极可能是**野指针**(结构体被写坏 / 未初始化指针)";
    }
    if (addr < 0x20020000U) {
        return "落在 SRAM 内 ⇒ 多为越界写/栈溢出, 而不是外设访问";
    }
    return "地址在 SRAM 之上但不在已列外设内 ⇒ 看 CFSR 细项";
}

uint32_t ArmFault_DecodeCfsr(uint32_t cfsr, char* out, uint32_t out_cap)
{
    /* 只列本工程真会出现的位。★ = 最能一锤定音的两位:
     *   PRECISERR + BFARVALID 一起置位 ⇒ BFAR 里的地址就是出错地址。 */
    static const struct { uint32_t bit; const char* name; } T[] = {
        { (1UL <<  0), "IACCVIOL"    },   /* 取指被拒: 跳到非执行区 */
        { (1UL <<  1), "DACCVIOL"    },   /* 取数被拒 */
        { (1UL <<  3), "MUNSTKERR"   },
        { (1UL <<  4), "MSTKERR"     },   /* 压栈出错 —— 栈指针已经跑飞 */
        { (1UL <<  8), "IBUSERR"     },   /* 取指总线错 */
        { (1UL <<  9), "PRECISERR"   },   /* ★ 精确总线错: BFAR 有效 */
        { (1UL << 10), "IMPRECISERR" },   /* 不精确: 出错地址已不可知 */
        { (1UL << 11), "UNSTKERR"    },
        { (1UL << 12), "STKERR"      },   /* 出栈出错: 多半是**栈溢出** */
        { (1UL << 13), "LSPERR"      },
        { (1UL << 15), "BFARVALID"   },   /* ★ BFAR 里的地址可信 */
        { (1UL << 16), "UNDEFINSTR"  },
        { (1UL << 17), "INVSTATE"    },   /* Thumb 位不对: 函数指针被写坏 */
        { (1UL << 18), "INVPC"       },
        { (1UL << 19), "NOCP"        },   /* 用了本核没有的协处理器 */
        { (1UL << 24), "UNALIGNED"   },
        { (1UL << 25), "DIVBYZERO"   },
    };
    uint32_t n = 0U, i;

    if (out == NULL || out_cap == 0U) return 0U;
    out[0] = '\0';

    for (i = 0U; i < sizeof(T) / sizeof(T[0]); i++) {
        size_t  ln;
        uint32_t need;
        if ((cfsr & T[i].bit) == 0U) continue;
        ln   = strlen(T[i].name);
        need = (uint32_t)ln + (n != 0U ? 1U : 0U);
        if (n + need + 1U > out_cap) break;        /* 写不下就停, 不截半截名字 */
        if (n != 0U) out[n++] = ' ';
        memcpy(out + n, T[i].name, ln);
        n += (uint32_t)ln;
        out[n] = '\0';
    }
    if (n == 0U) {
        const char* s = "none";
        memcpy(out, s, strlen(s) + 1U);
        n = 4U;
    }
    return n;
}

/* ============================================================
 * 主循环停滞记录 —— 见 arm_fault.h 顶部的说明
 * ============================================================ */
#define ARM_STALL_REC_ADDR  ((ArmStallRec*)0x1000F800U)   /* 故障记录占 0x1000F000 */

void ArmStall_Tick(uint32_t main_progress_ms, uint32_t phase, uint32_t cmd)
{
    static uint32_t last_progress = 0xFFFFFFFFU;
    static uint32_t last_change   = 0U;
    static uint8_t  reported      = 0U;
    static uint8_t  armed         = 0U;   /* 0924DD: 见过一次真实推进才允许记卡死 */
    ArmStallRec* r = ARM_STALL_REC_ADDR;
    uint32_t now;

    if (main_progress_ms != last_progress) {   /* 主循环还在推进 */
        if (last_progress != 0xFFFFFFFFU) armed = 1U;
        last_progress = main_progress_ms;
        last_change   = HAL_GetTick();
        reported      = 0U;
        return;
    }
    if (!armed) return;    /* 0924DD: 主循环尚未启动(开机扫描/初始化阶段)不计卡死
                            * —— DB 加主臂扫 7~12 后开机窗可超 3s, 老逻辑把
                            * "开机慢"误记成 phase=1 卡死, 下次 rb 就假报 */
    if (reported) return;                      /* 每次卡死只记一次 */

    now = HAL_GetTick();
    if ((uint32_t)(now - last_change) < ARM_STALL_MS) return;

    reported = 1U;
    r->magic    = 0U;                          /* 先失效再填, 见 ArmFault_Mark 的理由 */
    r->stall_ms = (uint32_t)(now - last_change);
    r->phase    = phase;
    r->cmd      = cmd;
    r->tick     = now;
    r->magic    = ARM_STALL_MAGIC;
}

uint8_t ArmStall_WasStall(void)
{
    return (uint8_t)(ARM_STALL_REC_ADDR->magic == ARM_STALL_MAGIC ? 1U : 0U);
}

void ArmStall_Read(ArmStallRec* out)
{
    if (out == NULL) return;
    *out = *ARM_STALL_REC_ADDR;
    if (ArmStall_WasStall() == 0U) out->magic = 0U;
}

void ArmStall_Clear(void)
{
    ARM_STALL_REC_ADDR->magic = 0U;
}
