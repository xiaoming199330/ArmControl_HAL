/* ============================================================
 * arm_fault.h — 把"一次性静默死亡"变成"下次开机自报"
 *
 * 为什么需要它 (2026-09-18 现场教训):
 *   本工程**没有看门狗**, 所以"固件在某个时刻挂住"和"板子根本没上电"
 *   在现场是**完全同一个现象**: 口在枚举、0 字节输出、bootloader 也不应答。
 *   这一轮为了区分这两者, 只能靠人工去爬桌子按 RESET —— 而两个可能性的
 *   处置方式完全不同(一个是查电源, 一个是查代码)。
 *
 *   真正缺的是一条判据: **"上一次运行是被 CPU 故障打断的吗?"**
 *   本模块用 CCM RAM 里一个不受复位影响的小结构来回答它:
 *     - 进 HardFault / MemManage / BusFault / UsageFault 时, 先把现场写进去;
 *     - 下次开机(控制台一能用)就把这份记录打出来, 然后清掉。
 *
 * 为什么放 CCM RAM (0x10000000 段) 的固定地址:
 *   ① 普通变量在 .bss/.data, 复位时会被启动代码清掉/重填 ⇒ 记不住上一轮;
 *   ② 改链接脚本加 .noinit 段能解决, 但那要动 .ld 和启动文件 ——
 *      为了一个诊断点去碰启动路径, 风险大于收益;
 *   ③ CCM RAM 在本工程里**一个 section 都没占**(.ccmram 段永远是空的),
 *      而且 CPU 复位不清 SRAM ⇒ 天然就是一块"跨复位保留"的干净地方。
 *   ⚠ 它跨**复位**保留, 但**跨掉电不保留** —— 这正好合用: 掉电后 magic
 *     自然读不到, 不会把"上次掉电"误报成"上次故障"。
 *
 * ⚠ 本模块**不依赖控制台**, 也**不做任何初始化**(没有 init 函数可调)。
 *   故障处理器里能安全调用的东西越少越好 —— 这里只写 RAM 和读 SCB 寄存器,
 *   不碰外设、不打印、不等待。
 * ============================================================ */
#ifndef __ARM_FAULT_H
#define __ARM_FAULT_H

#include <stdint.h>

/* 故障类别。数值直接落进记录里, 别改已有取值(改了历史记录就对不上)。 */
#define ARM_FAULT_NONE    0U
#define ARM_FAULT_HARD    1U   /* HardFault  —— 升级上来的, 真因看 CFSR     */
#define ARM_FAULT_MEM     2U   /* MemManage  —— MPU / 取指越权              */
#define ARM_FAULT_BUS     3U   /* BusFault   —— **读写不存在的地址**: BFAR 直说哪儿 */
#define ARM_FAULT_USAGE   4U   /* UsageFault —— 非法指令 / 除零 / 未对齐    */

typedef struct {
    uint32_t magic;      /* == ARM_FAULT_MAGIC 才是有效记录(最后写, 先写它别的都不算) */
    uint32_t inv_magic;  /* ~magic: 双保险, 防上电初值偶然命中                    */
    uint32_t code;       /* ARM_FAULT_*                                          */
    uint32_t cfsr;       /* SCB->CFSR: 真因就在这里面                            */
    uint32_t hfsr;       /* SCB->HFSR: 是否是被升级上来的(bit30 FORCED)          */
    uint32_t bfar;       /* SCB->BFAR: BusFault 的出错地址 ★最有用的一栏         */
    uint32_t mmfar;      /* SCB->MMFAR: MemManage 的出错地址                     */
    uint32_t exc_return; /* 处理器进入本异常时的 EXC_RETURN, 见下                */
    uint32_t ipsr;       /* 出错时在哪个中断里(IPSR); 0 = 线程模式(主循环)       */
} ArmFaultRec;

#define ARM_FAULT_MAGIC 0x41524D46U   /* 'A','R','M','F' */

/* 由故障处理器调用。code = ARM_FAULT_*; exc_return 在处理器入口处读 LR 得到。 */
void ArmFault_Mark(uint32_t code, uint32_t exc_return);

/* 有没有留下有效记录。 */
uint8_t ArmFault_WasFault(void);

/* 取记录副本(不改动)。out 可为 NULL。 */
void ArmFault_Read(ArmFaultRec* out);

/* 清掉记录 —— 打完日志就要调, 否则下次开机还会重复报同一条。 */
void ArmFault_Clear(void);

/* "HardFault" 之类的可读名; 未知返回 "?"。 */
const char* ArmFault_Name(uint32_t code);

/* 把 CFSR 里**置位**的位翻成短名(空格分隔)写进 out, 便于直接看结论。
 * NOCP / PRECISERR / BFARVALID / UNDEFINSTR 这几个最值钱, 所以不做全表,
 * 只做会真正出现在本工程里的那十几个。返回写进去的长度。 */
uint32_t ArmFault_DecodeCfsr(uint32_t cfsr, char* out, uint32_t out_cap);

/* 地址落在哪个外设。用途: BFAR 指向 CAN2(0x40006800) 时, 基本可以断定是
 * "时钟没开就去读写那个控制器" —— 本次排查的核心怀疑点。 */
const char* ArmFault_AddrHint(uint32_t addr);

/* ============================================================
 * 主循环停滞记录 (2026-09-18)
 *
 * 为什么还需要它: 故障记录只能回答"CPU 是不是撞上异常了"。现场还有第三种
 * 死法 —— **主循环卡在某个无界等待里**(例如等一个永远不会来的 UART 标志):
 * 此时 CPU 没故障、USB 照常枚举、CH340 还在, 可板子一个字都不说。它和
 * "掉电"、"BusFault" 三者在现场**表象完全一样**, 只能靠一个**与主循环无关的
 * 观察者**来分辨 —— 这里就用已经在跑的定时器中断(TIM2):
 *   主循环每转一圈更新一次进度时间戳; ISR 发现它超过 ARM_STALL_MS 没动,
 *   就把 phase / cmd / 卡住时长记进 CCM RAM, 下次开机自报。
 *   ⇒ 下次开机三种结论互斥且完备:
 *      有故障记录          = CPU 故障(异常处理器的 while(1))
 *      只有停滞记录        = 主循环卡死, 并已定位到 phase/cmd
 *      两条都没有          = 上一次是掉电/断电(或没跑起来)
 * ============================================================ */
#define ARM_STALL_MAGIC 0x41524D53U   /* 'A','R','M','S' */
#define ARM_STALL_MS    3000U         /* 主循环多久没推进就算卡死 */

typedef struct {
    uint32_t magic;      /* == ARM_STALL_MAGIC 才有效 */
    uint32_t stall_ms;   /* 至少卡了多久 */
    uint32_t phase;      /* 卡在哪个阶段, 见 main.c 的 ARM_PHASE_* */
    uint32_t cmd;        /* 正在处理的命令首字符; 0 = 不在处理命令 */
    uint32_t tick;       /* 记下时的 HAL_GetTick */
} ArmStallRec;

/* 由**定时器中断**调用(不是故障处理器, 所以这里可以用 HAL_GetTick)。
 * main_progress_ms = 主循环最近一次推进时记录的 HAL_GetTick 值。 */
void ArmStall_Tick(uint32_t main_progress_ms, uint32_t phase, uint32_t cmd);

uint8_t ArmStall_WasStall(void);
void    ArmStall_Read(ArmStallRec* out);   /* out 可为 NULL */
void    ArmStall_Clear(void);

#endif /* __ARM_FAULT_H */
