/* ============================================================
 * arm_link_cfg.c - 链路偏好的 RAM 态与 Flash 持久化
 * 见 arm_link_cfg.h。范式与 arm_can_config.c 一致(magic/version/len/CRC)。
 * ============================================================ */
#include "arm_link_cfg.h"
#include "flash_cfg.h"
#include "arm_runlog.h"
#include "main.h"          /* HAL_GetTick: auto 模式活动超时计时(0924CE) */
#include <string.h>
#include <stddef.h>

#define LINK_CFG_MAGIC    0x4C4E4B31UL   /* "LNK1" */
#define LINK_CFG_VERSION  1U

/* Flash 记录(20 字节, 远小于 4KB 区跨度) */
typedef struct {
    uint32_t   magic;
    uint16_t   version;
    uint16_t   len;
    ArmLinkCfg cfg;
    uint32_t   crc;
} LinkCfgRecord;

static ArmLinkCfg s_cfg;                /* 偏好: "下次开机"用哪条 */
static ArmLinkCfg s_saved;              /* Flash 里的那份(用于脏判定) */
static uint8_t    s_active = LINK_MODE_OFF;  /* 手动模式: 冻结; auto 模式: 不看它(见下) */
static uint8_t    s_from_flash = 0U;

/* ---- AUTO 双链路门控状态(0924CE) ---- */
static uint8_t s_auto_active = LINK_MODE_OFF; /* 动态 active: OFF/CAN/NET */
static uint32_t s_auto_last_rx = 0U;          /* 最近一次合法包的 tick */
static uint8_t s_hw_net = 0U;                 /* 本次启动 NET 外设真的初始化过 */
static uint8_t s_hw_can = 0U;                 /* 本次启动 CAN 外设真的初始化过 */

/* CRC-32 (poly 0xEDB88320, 反射), 与 pid_flash.c / arm_can_config.c 同一算法 */
static uint32_t link_crc(const LinkCfgRecord* r)
{
    const uint8_t* b = (const uint8_t*)r;
    uint32_t crc = 0xFFFFFFFFU;
    for (uint32_t n = 0U; n < offsetof(LinkCfgRecord, crc); n++) {
        crc ^= b[n];
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            crc = (crc >> 1) ^ (0xEDB88320U & (-(int32_t)(crc & 1U)));
        }
    }
    return ~crc;
}

static uint8_t mode_valid(uint8_t m)
{
    return (m == LINK_MODE_OFF || m == LINK_MODE_CAN ||
            m == LINK_MODE_NET || m == LINK_MODE_AUTO) ? 1U : 0U;
}

static void link_defaults(ArmLinkCfg* c)
{
    memset(c, 0, sizeof(*c));
    /* ⭐ 2026-11-10 用户明令(覆盖 2026-09-22 的"默认全关"): **未配置默认 = AUTO**。
     *   开机/复位即双链路监听, 谁发合法包谁 active, 5s 无包回静默(仍监听)。
     *   拔线/插线免人工切换。要关必须显式 `link off` + `link save`。 */
    c->mode = LINK_MODE_AUTO;
}

void LinkCfg_Init(void)
{
    LinkCfgRecord r;

    link_defaults(&s_cfg);
    s_saved = s_cfg;
    s_from_flash = 0U;
    s_active = s_cfg.mode;      /* 先钉成"全关"; 只有下面读到有效记录才改 */

    /* ⚠ 本函数在 RunLog_Init() 之前被调用 ⇒ 全程**不使用 RLOG**,
     *   也不写 Flash(只读)。落盘只由 `link save` 触发。 */

    if (CfgFlash_ReadRegion(CFG_OFF_LINK, &r, sizeof(r)) != CFG_FLASH_OK) {
        /* 读都读不到(参数/扇区异常) ⇒ 保守取"未配置" ⇒ 默认 AUTO(0924CF 起) */
        return;
    }
    if (r.magic != LINK_CFG_MAGIC || r.version != LINK_CFG_VERSION ||
        r.len != sizeof(ArmLinkCfg) || r.crc != link_crc(&r)) {
        /* 未写过(magic=0xFFFFFFFF)/版本不符/校验失败 —— 都是"没配置过"。
         * ⇒ 默认 AUTO(2026-11-10 用户明令, 覆盖 09-22 全关语义):
         *   开机即 CAN+网口双监听, 免人工切换。显式关走 `link off`。 */
        return;
    }
    if (!mode_valid(r.cfg.mode)) {
        return;                          /* 字段越界 ⇒ 同样当未配置 */
    }
    s_cfg = r.cfg;
    s_saved = r.cfg;
    s_from_flash = 1U;
    s_active = r.cfg.mode;   /* ⭐ 冻结"本次启动生效值"; 运行期判定只看它 */
}

const ArmLinkCfg* LinkCfg_Get(void) { return &s_cfg; }

/* 开机初始化分支专用: 返回偏好本身(含 AUTO)。
 * ⚠ auto 下 ActiveMode() 开机返回 OFF(动态值), 不能拿去判双初始化。 */
uint8_t LinkCfg_BootMode(void) { return s_cfg.mode; }

/* 开机引脚分配判据: NET 手动与 AUTO 都按"网口形态"分脚(见 .h 注释)。 */
uint8_t LinkCfg_BootNetPins(void)
{
    return (s_cfg.mode == LINK_MODE_NET || s_cfg.mode == LINK_MODE_AUTO) ? 1U : 0U;
}

/* ⭐ 运行期唯一合法判据。
 * 手动 can/net: 冻结值(语义同旧版)。
 * auto: 动态值 —— 合法包授 NET/CAN, 5s 超时回 OFF(静默监听)。 */
uint8_t LinkCfg_ActiveMode(void)
{
    return (s_cfg.mode == LINK_MODE_AUTO) ? s_auto_active : s_active;
}

int LinkCfg_Set(uint8_t mode)
{
    if (!mode_valid(mode)) return -1;
    s_cfg.mode = mode;
    return 0;
}

int LinkCfg_Save(void)
{
    LinkCfgRecord r;
    memset(&r, 0, sizeof(r));
    r.magic   = LINK_CFG_MAGIC;
    r.version = LINK_CFG_VERSION;
    r.len     = sizeof(ArmLinkCfg);
    r.cfg     = s_cfg;
    r.crc     = link_crc(&r);

    if (CfgFlash_WriteRegion(CFG_OFF_LINK, &r, sizeof(r)) != CFG_FLASH_OK) {
        RLOG_E("LINK", "cfg save FAILED");
        return -1;
    }
    s_saved = s_cfg;
    s_from_flash = 1U;
    RLOG_I("LINK", "cfg saved: %s", LinkCfg_NameOf(s_cfg.mode));
    return 0;
}

uint8_t LinkCfg_IsSaved(void)
{
    return (memcmp(&s_cfg, &s_saved, sizeof(s_cfg)) == 0) ? 1U : 0U;
}

uint8_t LinkCfg_LoadedFromFlash(void) { return s_from_flash; }

const char* LinkCfg_NameOf(uint8_t mode)
{
    switch (mode) {
    case LINK_MODE_CAN: return "can";
    case LINK_MODE_NET: return "net";
    case LINK_MODE_OFF: return "off";
    case LINK_MODE_AUTO: return "auto";
    default:            return "?";
    }
}

int LinkCfg_Parse(const char* s, uint8_t* out_mode)
{
    if (!s || !out_mode) return -1;

    if (strcmp(s, "off") == 0 || strcmp(s, "none") == 0 || strcmp(s, "0") == 0) {
        *out_mode = LINK_MODE_OFF;
        return 0;
    }
    if (strcmp(s, "can") == 0 || strcmp(s, "1") == 0) {
        *out_mode = LINK_MODE_CAN;
        return 0;
    }
    if (strcmp(s, "net") == 0 || strcmp(s, "eth") == 0 ||
        strcmp(s, "udp") == 0 || strcmp(s, "2") == 0) {
        *out_mode = LINK_MODE_NET;
        return 0;
    }
    if (strcmp(s, "auto") == 0 || strcmp(s, "3") == 0) {
        *out_mode = LINK_MODE_AUTO;
        return 0;
    }
    return -1;
}

/* ---- AUTO 双链路门控(0924CE) ---------------------------------------- */

void LinkCfg_MarkNetInited(void) { s_hw_net = 1U; }
void LinkCfg_MarkCanInited(void) { s_hw_can = 1U; }
uint8_t LinkCfg_NetHwInited(void) { return s_hw_net; }
uint8_t LinkCfg_CanHwInited(void) { return s_hw_can; }
uint8_t LinkCfg_AutoEnabled(void)
{
    return (s_cfg.mode == LINK_MODE_AUTO) ? 1U : 0U;
}

/* 收到合法包(已过 CRC, 即将执行)时调用: 授 active + 刷时间戳。
 * ⚠ 只有 auto 模式才动状态; 手动模式与单链路构建下是空操作, 收包点可无条件调。 */
void LinkCfg_TouchNet(void)
{
    if (s_cfg.mode != LINK_MODE_AUTO) return;
    s_auto_active  = LINK_MODE_NET;
    s_auto_last_rx = HAL_GetTick();
}

void LinkCfg_TouchCan(void)
{
    if (s_cfg.mode != LINK_MODE_AUTO) return;
    s_auto_active  = LINK_MODE_CAN;
    s_auto_last_rx = HAL_GetTick();
}

/* 主循环每拍调用: 超过 LINK_AUTO_TIMEOUT_MS 无合法包 => 回静默监听态。
 * "静默"的含义(设计权衡, 别误解):
 *   · 外设仍在收(必须常听才能探测"有东西下发");
 *   · 合法包到达即敲门+直接执行(不吞首包 —— 吞包会让上位机每次空闲后
 *     第一条命令超时, 不可靠);
 *   · active=OFF 的实际效果: 0x280 心跳停止(总线上没人听就别喊)、
 *     状态上报显示 off、NET_BLOCK_CAN_CMDS 等门控回到最严格档。 */
void LinkCfg_Tick(void)
{
    if (s_cfg.mode != LINK_MODE_AUTO) return;
    if (s_auto_active == LINK_MODE_OFF) return;
    if ((HAL_GetTick() - s_auto_last_rx) >= LINK_AUTO_TIMEOUT_MS) {
        s_auto_active = LINK_MODE_OFF;
    }
}
