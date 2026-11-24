/* ============================================================
 * arm_rc_ctrl.c - R1: RC 遥控 -> 末端控制 (速度流)
 *
 * 设计见 arm_rc_ctrl.h 顶注与 docs/RC方案复核_2026-09-23.md。
 * 采集层在 arm_rc.c(只读); 本文件是第一层"会动的"代码, 安全闸优先级:
 *   E-Stop > 新鲜度 > 外部运动互斥 > sf 0 > (心跳) > 流层真拒帧熔断。
 * ⭐ 包线越界(桌面/立柱)不走熔断: 发帧前五档缩步长预检 = 顶墙软停不自锁
 *   (2026-09-24, 用户定案; 见 Tick 内注释)。
 * ============================================================ */
#include "stm32f4xx_hal.h"
#include <math.h>
#include <string.h>
#include <stddef.h>
#include "arm_rc_ctrl.h"
#include "arm_rc.h"
#include "arm_main_exports.h"
#include "arm_control_api.h"
#include "arm_runlog.h"
#include "servo_driver.h"      /* SERVO_SetSpeed: RC 流期间自适应 GoalSpeed */
#include "flash_cfg.h"         /* 速度偏好落盘 (扇区 11 区 D) */

/* ⭐ P0-2 互斥标志: 本模块发流帧前置 1, cmdStreamJoints(main.c) 收到
 *   非 RC 来源的流帧(网口 STREAM_JOINTS / 主从 #P 中转)即 disarm RC。
 *   定义在本文件, main.c 经 arm_main_exports.h 的 extern 声明读取。 */
volatile uint8_t arm_rc_ctrl_in_frame = 0U;

#if ARM_MASTER_BUILD
/* ---- 主臂构建: 只读转发角色, 不驱臂 (与 cmdStreamJoints 语义一致) ---- */
void ArmRcCtrl_Tick(void)                       { (void)0; }
void ArmRcCtrl_SetArmed(uint8_t on)             { (void)on; }
uint8_t ArmRcCtrl_IsArmed(void)                 { return 0U; }
void ArmRcCtrl_Disarm(const char* why)          { (void)why; }

/* rc spd 参数接口: 主臂不跑 RC, 空实现防止链接错误 (速度只对从臂有意义)。
 * 2026-09-24 起 main.c 的 `rc spd` 命令树无条件引用这三个符号,
 * 主臂构建自 0924E 后一直链不过 —— 10-30 重建主臂固件时补上。 */
void ArmRcCtrl_SpdGet(float v[4])               { v[0]=v[1]=v[2]=v[3]=0.0f; }
uint8_t ArmRcCtrl_SpdSet(float vxy, float vz, float vj5, float j6r)
    { (void)vxy; (void)vz; (void)vj5; (void)j6r; return 1U; }
uint8_t ArmRcCtrl_SpdSave(void)                 { return 0U; }

#else /* ============ 从臂构建: 正式实现 ============ */

/* ---- 调参宏 (全部保守值; 改完重编, 不需要重新标定) ----
 * ⚠ 速度四项 2026-09-24 起可由 `rc spd` 运行时修改并存 flash(区 D),
 *   下面是**默认值** —— flash 里没有有效记录(或校验不过)时生效。 */
#define RC_VXY_DEF      12.0f   /* X/Y 默认末端速度 mm/s (首版 5 太慢)   */
#define RC_VZ_DEF       8.0f    /* Z  默认末端速度 mm/s (首版 4 太慢)    */
#define RC_VJ5_DEF      150.0f  /* J5 默认角速度 deg/s                   */
/* ⭐0924Y: J5 安全窗改成实际行程 15~350 (全行程 335°) 后, 30°/s 要 11s 走完,
 *   现场反馈"转得很慢"。提到 150°/s ≈ 2.2s/全行程 (5×)。
 *   J5 是腕部回转 (绕小臂轴转爪), 负载轻, 150°/s 只有舵机极速(~299°/s)的一半。
 *   寄存器上限 3400 step/s ≈ 299°/s, 故 vj5 可用区间 5~300 (°/s)。
 *   运行时可调, 不必重编: rc spd <vxy> <vz> <vj5>  (例: rc spd 12 8 200)
 *                          rc spd save               (存 flash 区 D 掉电不丢) */
/* ⭐0924X: 改为关节空间控制后, J1~J4 使用下面独立默认速度 (deg/s)。
 *   关节负载差异大: J1 底座可快, J2/J3 承重放慢, J4 腕部中等。
 *   这些值目前编译期固定; 如需要运行时调节可后续扩 `rc spd`。 */
#define RC_VJ1_DEF      50.0f   /* J1 底座旋转速度 deg/s                  */
#define RC_VJ2_DEF      25.0f   /* J2 大臂俯仰速度 deg/s                  */
#define RC_VJ3_DEF      30.0f   /* J3 小臂俯仰速度 deg/s                  */
#define RC_VJ4_DEF      40.0f   /* J4 腕俯仰速度 deg/s                    */
#define RC_J6_DEF       90.0f   /* 夹爪跟随速率 deg/s —— ⭐0924E 起废弃不用:
                                 *   夹爪=滑杆绝对位置全速实时跟随 (保留字段
                                 *   只为 flash 区 D 记录兼容) */
#define RC_DZ_US        60.0f   /* 杆量死区 (µs, 相对中位)                 */
#define RC_HALF_US      500.0f  /* 标称半行程 (µs)                        */
#define RC_SIGN_X       (+1.0f) /* 方向符号(实测已确认, 别再改)           */
#define RC_SIGN_Y       (+1.0f)  /* 0924X: 左杆前后反了, 取反 */
#define RC_SIGN_Z       (+1.0f)
#define RC_SIGN_J5      (+1.0f)
#define RC_SIGN_J6      (+1)    /* 夹爪: +1=滑杆高电平→J6张开(lim_max)     */
#define RC_BASE_REL     1       /* 1=X/Y 在"底座随动系"解释(杆上=沿臂朝向前移) */
#define RC_J5_SAFE_MIN  15.0f  /* J5 RC 目标安全窗 (L1 servo 度)。机械卡区   */
#define RC_J5_SAFE_MAX  350.0f         
#define RC_REJ_DISARM   15U     /* 连续拒帧熔断 (15 帧 ≈ 300ms)            */
#define RC_STALE_MS     300U    /* 四杆不新鲜超时 ⇒ disarm                  */
#define RC_RESYNC_MS    600U    /* 断流超过它 ⇒ 下一拍重新实测基准          */
/* ⭐ 0924T: UP/DOWN 按键改为直接调 J4 关节目标 (原俯仰姿态功能废弃)。
 *   速率 45°/s; 范围钳到 J4 校准限位 (L 命令), 真到包线由五档软停兜底。 */
#define RC_J4_RATE      45.0f   /* J4 按键调节速率 (deg/s)                 */
#define RC_HB_EN        0       /* ⚠ 心跳判据: 当前 CH6=不复位旋钮, 暂关。  */
#define RC_HB_LOSS_MS   100U    /*   换成专用开关后改 1 (见 arm_rc.h 宏)    */

/* ---- 速度偏好: flash 区 D 记录 (magic/ver/size/FNV 校验) ---- */
#define RCSPD_MAGIC     0x52535031U   /* "RSP1" */
#define RCSPD_VER       1U            /* ⚠ 0924Y: **故意保持 1** —— 现场已存
                                       *   120/80/300 的偏好记录, 升版会把它一并
                                       *   作废(连带 X/Y/Z 回默认)。J5 变慢的根因
                                       *   不在这个数值, 见 req_rate[] 注释。 */
typedef struct {
    uint32_t magic;
    uint16_t ver;
    uint16_t size;
    float    vxy;      /* X/Y 末端速度 mm/s      */
    float    vz;       /* Z   末端速度 mm/s      */
    float    vj5;      /* J5  角速度 deg/s       */
    float    j6r;      /* 夹爪跟随速率 deg/s     */
    uint32_t sum;      /* FNV-1a 到 sum 字段为止 */
} RcSpdCfg;

static RcSpdCfg spd_cfg = {
    RCSPD_MAGIC, RCSPD_VER, (uint16_t)sizeof(RcSpdCfg),
    RC_VXY_DEF, RC_VZ_DEF, RC_VJ5_DEF, RC_J6_DEF, 0U
};
static uint8_t spd_loaded = 0U;

static uint32_t rcspd_sum(const RcSpdCfg* c) {
    const uint8_t* p = (const uint8_t*)c;
    uint32_t s = 2166136261U;
    for (uint32_t i = 0U; i < offsetof(RcSpdCfg, sum); i++) {
        s ^= p[i];
        s *= 16777619U;
    }
    return s;
}

/* 懒加载: 首次用到速度时读 flash 区 D; 记录无效(空/改版/校验不过/
 * 数值出界)就保持默认 —— 永远不给 RC 一个危险速度。 */
static void spd_load(void) {
    RcSpdCfg t;
    if (spd_loaded) return;
    spd_loaded = 1U;
    if (CfgFlash_ReadRegion(CFG_OFF_RCSPD, &t, sizeof(t)) != CFG_FLASH_OK) return;
    if (t.magic != RCSPD_MAGIC || t.ver != RCSPD_VER || t.size != (uint16_t)sizeof(RcSpdCfg)) return;
    if (t.sum != rcspd_sum(&t)) return;
    if (t.vxy < 1.0f || t.vxy > 120.0f) return;   /* 0924E: 60→120 */
    if (t.vz  < 1.0f || t.vz  > 80.0f)  return;   /* 0924E: 40→80  */
    if (t.vj5 < 5.0f || t.vj5 > 300.0f) return;   /* 0924E: 150→300 */
    if (t.j6r < 10.0f || t.j6r > 300.0f) return;  /* 字段兼容保留, 已不用 */
    spd_cfg = t;
}

void ArmRcCtrl_SpdGet(float v[4]) {
    spd_load();
    v[0] = spd_cfg.vxy;  v[1] = spd_cfg.vz;
    v[2] = spd_cfg.vj5;  v[3] = spd_cfg.j6r;
}

uint8_t ArmRcCtrl_SpdSet(float vxy, float vz, float vj5, float j6r) {
    if (vxy < 1.0f || vxy > 120.0f)  return 0U;   /* 0924E: 60→120 */
    if (vz  < 1.0f || vz  > 80.0f)   return 0U;   /* 0924E: 40→80  */
    if (vj5 < 5.0f || vj5 > 300.0f)  return 0U;   /* 0924E: 150→300 */
    if (j6r < 10.0f || j6r > 300.0f) return 0U;   /* 字段兼容保留, 已不用 */
    spd_load();                      /* 先吃 flash 里的既有值, 再覆盖 */
    spd_cfg.vxy = vxy;  spd_cfg.vz = vz;
    spd_cfg.vj5 = vj5;  spd_cfg.j6r = j6r;
    return 1U;
}

uint8_t ArmRcCtrl_SpdSave(void) {
    spd_load();
    spd_cfg.magic = RCSPD_MAGIC;
    spd_cfg.ver   = RCSPD_VER;
    spd_cfg.size  = (uint16_t)sizeof(RcSpdCfg);
    spd_cfg.sum   = rcspd_sum(&spd_cfg);
    return (CfgFlash_WriteRegion(CFG_OFF_RCSPD, &spd_cfg, sizeof(spd_cfg))
            == CFG_FLASH_OK) ? 1U : 0U;
}

/* ---- 会话状态 ---- */
static uint8_t  armed        = 0U;
static uint8_t  session      = 0U;    /* 1=已实测基准, 积分器有效 */
static uint32_t last_tick_ms = 0U;
static float    j1_t, j2_t, j3_t;    /* ⭐0924X: 杆量积分的关节目标 (L1 servo 度) */
static float    j4_t, j4_sent;       /* J4 目标与最近下发值                 */
static float    j5_t, j6_t;          /* 关节空间积分 (L1 servo 度)          */
static float    j6_sent;             /* ⭐0924V: 最近一帧**实际下发**的 J6 (松杆
                                     *   钉位用它, 不再用 act_l1[5] —— 见 Tick) */
static float    lim_min[6], lim_max[6]; /* L1 servo 度 (RAM 校准限位)     */
static float    last_deg[6];         /* 最近一帧被接受的完整解 (L2 域, 纯夹爪帧免 IK) */
static float    prev_l1[6];          /* 最近一帧被接受的 L1 目标 (IK 翻转闸/兜底帧/节奏器基准) */
static uint8_t  have_deg = 0U;
static uint32_t rej_run       = 0U;  /* 连续拒帧计数 (熔断用) */
static uint32_t last_accept_ms= 0U;
static uint32_t last_fresh_ms = 0U;
static uint32_t last_block_log= 0U;
static uint32_t last_try_ms   = 0U;
static uint8_t  prev_intent   = 0U;   /* 上一拍是否有操作意图 (刹车帧判边沿) */
static uint32_t last_brake_ms = 0U;
/* 杆量 ([-1,1], 死区+平方特性): ⭐0924E 起不再做低通滤波 —— 接收机本地
 * 硬件直连延迟≈0, 滤波的 125ms 滞后就是"不灵敏"; 消 PWM 抖动交给 60µs
 * 死区 + 平方特性。摇杆停 = 杆量立即归零 = 立即停。 */
static float    f_s[4]        = {0.0f, 0.0f, 0.0f, 0.0f};
/* ⭐ GoalSpeed 演进史 (STS3215: 1 单位 ≈ 4.39°/s):
 *   0923F 按被接受帧位移 EMA 估速 → 0924C 反馈钳制后自锁低速 (钳制把指令锚在
 *   实际位置上, 估出=舵机当前速度) → 0924D 改差距×K 闭环 → 实机"晃"+松杆还账
 *   (反馈环: 轮询回来差距跳变, 速度跟着抖)。
 *   ⭐ 0924E 终案 = **开环指令速率直通**: GoalSpeed = 指令步长/dt 的 EMA,
 *   与舵机实际位置无关 ⇒ 不晃; 松杆速率归零 ⇒ 立即停; 夹爪恒全速实时跟随。
 *   详见 pace_speeds() 注。
 * ⭐⭐ 0924F 重大修复 —— 速度单位换算错了 50 倍 (0923F 起一直错):
 *   STS3215 Goal Speed(46-47) 寄存器单位 = **1 step/s** (1 step = 360/4096°),
 *   量程 0~3400 step/s ≈ 0~299°/s ≈ 舵机空载极速 (量程与物理极速对上)。
 *   旧换算按 "1 单位 ≈ 4.39°/s" (=50 step/s) ⇒ 写出的值全部比意图慢 ~50×:
 *   J5 设 150°/s 实际只拿到 ~3°/s —— 这才是"rc spd 拉满还是好慢"+"拖尾好几秒"
 *   的真根因 (0924E 里只有夹爪跳过 pace 吃到全速, 所以只有杆轴慢/拖尾)。
 *   佐证: 尾段交接写 GOAL_SPEED=42 是"小值拖慢跟随" (42 step/s = 3.7°/s)。 */
#define RC_STEPS_PER_DEGS  (4096.0f / 360.0f) /* °/s -> 寄存器单位 (step/s) */
static float    cmd_rate[6]  = {0.0f};   /* ⭐0924E 指令关节速率 EMA (deg/s, 开环) */
/* ⭐⭐ 0924Y (J5 提速根因修复) —— 各轴"请求速率" (deg/s, 直接来自杆量/按键,
 *   每拍刷新, 不受任何钳制影响)。pace_speeds 用它给 GoalSpeed 兜底。
 *
 *   为什么必须加: GoalSpeed 原来源自 cmd_rate = |被接受帧的指令步长|/dt,
 *   而**反馈钳制 (act±15°) 会把指令钉在 act+15 上** —— 一旦钉住, 步长就退化成
 *   "舵机自己的实际位移" ⇒ cmd_rate ≈ 舵机实际速率 ⇒ GoalSpeed = 实际速率。
 *   这是一个**自指死循环**: 速度锁死在"指令第一次被钉住"那一刻, 之后无论把
 *   rc spd 的 vj5 加到多少都提不上去 (vj5=300 实测只有 ~150°/s 的机理)。
 *   杆轴 (RC_VJ1~VJ4_DEF = 25~50°/s) 每拍只走 0.5~1°, 永远钉不住 ⇒ 不受影响,
 *   所以"只有 J5 慢"。
 *   ⚠ 现场观感: 限位窗改小的时候 J5 很快就撞到边界, 看着"挺快"; 窗改大 (15~350)
 *   后同样的真实速度要走 335° ⇒ 这个上限才暴露出来 = "把限位改大后就慢了"。
 *   修法: GoalSpeed = max(cmd_rate, 请求速率)。请求速率是"人要的速度", 不受钳制
 *   影响; 轴真被顶死时指令压根不动, 高速只是白给推力 (位置环, 无害)。 */
static float    req_rate[6]  = {0.0f};
static uint16_t reg_written[6]= {0U, 0U, 0U, 0U, 0U, 0U};
static uint32_t last_spd_ms[6]= {0U};
/* ⭐ 反馈钳制锚 (0924C): 每拍轮询 1 轴实际位置 (50Hz ⇒ 120ms 扫完六轴),
 *   指令目标不得超过 act_l1 ± LEAD。0924B 实机踩坑: 目标以指令速率推进而
 *   舵机以 GoalSpeed 追赶, 一旦脱节(软启动 8.8°/s vs 夹爪 90°/s)差距滚雪球,
 *   撞 25° 跳变闸 -> 拒帧 -> 会话重测(actual)又撞旧的流基准 -> 死锁锁死 +
 *   "拖尾 3-4s"(舵机以低速慢慢关差) + "不灵敏"(目标冻结在前面)。
 *   钳死后指令-实际差距恒 ≤ LEAD < 25° 跳变闸, 拒帧从机理上消失。 */
static float    act_l1[6]     = {0.0f};
static uint8_t  act_i         = 0U;
static uint32_t act_last_ms[6]= {0U, 0U, 0U, 0U, 0U, 0U}; /* ⭐0924V: 各轴锚最近
                                          *   成功刷新时刻 (补读饿死轴用) */
#if RC_HB_EN
static uint8_t  hb_loss_now   = 0U;
static uint32_t hb_loss_start = 0U;
#endif

/* ⭐ 0924E 目标节奏器 = **开环指令速率直通** (取代 0924D 的差距×K 闭环):
 *   0924D 的 GoalSpeed=|目标−实际|×K 是一条反馈环 —— 实际位置每次轮询回来
 *   差距跳变, 速度寄存器跟着抖 ⇒ 走起来"晃"; 且松杆后要慢慢"还账"
 *   (差距关账) ⇒ 违背"摇杆停=机械臂停"。
 *   新方案: GoalSpeed_j = **指令速率** (被接受帧的指令步长/dt 的 EMA),
 *   与舵机实际位置无关 ⇒ 无反馈环=不晃; 目标推进速率 = 舵机速率 ⇒
 *   匀速连续不顿挫; 松杆 → 指令速率→0 → 只剩最后一小步 ⇒ 立即停。
 *   ⭐ 夹爪 (j==5) 跳过: 恒全速 (stream_enter 的 0), 滑杆绝对位置实时跟随。
 *   写寄存器: 150ms/轴节流 + ±2 滞回, 控总线负载。 */
static void pace_speeds(uint32_t now) {
    for (uint8_t j = 0; j < 6; j++) {
        /* ⭐0924AC: 夹爪也显式写 GoalSpeed=0(全速)。旧代码 skip(j==5) 导致
         *   夹爪 GoalSpeed 残留之前命令(如 `m ... v=15`) 的低速度 ⇒ 夹爪
         *   闭合异常慢。每 150ms 重写一次 0, 确保恒全速实时跟随。 */
        if (j == 5U) {
            uint16_t want = 0U;           /* 0 = 舵机最大速度 */
            uint16_t cur  = reg_written[5U];
            if (cur != 0U && (cur <= 2U)) continue; /* 已全速, 跳过 */
            if ((uint32_t)(now - last_spd_ms[5U]) > 150U) {
                if (SERVO_SetSpeed(5U, want) == HAL_OK) {
                    reg_written[5U] = want;
                    last_spd_ms[5U] = now;
                }
            }
            continue;
        }
        /* ⭐⭐0924Y: 取「指令速率」与「请求速率」的较大者 (见 req_rate[] 注释)。
         *   只信 cmd_rate 会被 ±15° 反馈钳制钉成"舵机自己的速度"(自指死循环)
         *   ⇒ 无论 vj5 设多大, 实际速度都停在第一次被钉住那一刻。 */
        float want_dps = cmd_rate[j];
        if (want_dps < req_rate[j]) want_dps = req_rate[j];
        uint16_t want = (uint16_t)(want_dps * RC_STEPS_PER_DEGS) + 1U;
        if (want > 3400U) want = 3400U;        /* 寄存器量程 0~3400 step/s */
        uint16_t cur  = reg_written[j];
        uint16_t diff = (want > cur) ? (uint16_t)(want - cur)
                                     : (uint16_t)(cur - want);
        if (cur != 0U && diff <= 2U) continue; /* 滞回: 免总线抖动 */
        if ((uint32_t)(now - last_spd_ms[j]) > 150U) {
            if (SERVO_SetSpeed(j, want) == HAL_OK) {
                reg_written[j] = want;
                last_spd_ms[j] = now;
            }
        }
    }
}

/* ---------------------------------------------------------------- */
static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* 杆量 -> [-1,+1] 平方特性 (中心细调 / 推满全速)。deadzone 外有效。 */
static float stick_norm(const ArmRcState* rc, uint32_t i) {
    float d = (float)rc->ch[i].width_us - (float)rc->mid[i];
    float a = fabsf(d);
    if (a < RC_DZ_US) return 0.0f;
    float n = (a - RC_DZ_US) / (RC_HALF_US - RC_DZ_US);
    if (n > 1.0f) n = 1.0f;
    return (d > 0.0f ? n * n : -(n * n));
}

/* 会话启动: 实测基准 (P0-1: 不许假设 0)。成功返回 1。 */
static uint8_t session_start(const ArmRcState* rc, uint32_t now) {
    ArmApiStatus  st;
    float lmn[6], lmx[6];
    (void)rc;
    if (ArmApi_GetStatus(&st) != ARM_API_OK || !st.read_ok) return 0U;
    /* ⭐ 限位源 = RAM 运行时限位 (Phase2 校准, `L` 同源)。
     *   不用 ArmApi_GetLimits() —— 它读扇区 11 flash 记录, 本板无有效记录。 */
    if (ArmMain_GetJointLimitsDeg(lmn, lmx) != 6U) {
        if ((uint32_t)(now - last_block_log) > 2000U) {
            RLOG_W("RCCTL", "refuse: 限位未全装载 (缺 Work ranges, 敲 L 查)");
            last_block_log = now;
        }
        return 0U;
    }
    for (uint8_t j = 0; j < 6; j++) {
        lim_min[j] = lmn[j];
        lim_max[j] = lmx[j];
        if ((lim_max[j] - lim_min[j]) < 5.0f) {   /* 全 0 / 非法 ⇒ 拒绝 */
            if ((uint32_t)(now - last_block_log) > 2000U) {
                RLOG_W("RCCTL", "refuse: J%u 限位非法 (%d~%d)",
                       (unsigned)(j + 1U), (int)lim_min[j], (int)lim_max[j]);
                last_block_log = now;
            }
            return 0U;
        }
    }
    j1_t = st.joint_deg[0];                       /* ⭐0924X: 关节目标从当前位起 */
    j2_t = st.joint_deg[1];
    j3_t = st.joint_deg[2];
    j4_t = st.joint_deg[3];  j4_sent = j4_t;
    j5_t = st.joint_deg[4];
    j6_t = st.joint_deg[5];  j6_sent = j6_t;      /* ⭐0924V: 松杆钉位基准 */
    if (j5_t < RC_J5_SAFE_MIN || j5_t > RC_J5_SAFE_MAX) {
        RLOG_W("RCCTL", "J5 当前 %.0f 在卡区/安全窗外(%.0f~%.0f): 杆一动将先驶向安全窗",
               (int)j5_t, (int)RC_J5_SAFE_MIN, (int)RC_J5_SAFE_MAX);
    }
    have_deg = 0U;
    session  = 1U;
    last_accept_ms = now;
    /* ⭐ 顿挫根治的前置: 会话开始就把六轴 GoalSpeed 从 0(全速) 压到低速软启动,
     *   之后 Tick 里按每轴实际速度自适应上调。写失败不重试(下一拍会再写)。 */
    /* ⭐ 0924C: 不再写软启动 GoalSpeed=2。0924B 实机教训: 软启动 8.8°/s 撞上
     *   夹爪 90°/s 指令速率 -> 差距 150ms 内拉到 ~12° 且继续滚雪球 -> 跳变闸。
     *   现在 reg_written=0 = "未写", 首个被接受帧立即自适应写入匹配速度;
     *   流层 stream_enter 本来就写 0(全速), 起步阶段目标≈实际, 不会瞬移。 */
    memcpy(prev_l1, st.joint_deg, sizeof(prev_l1));   /* 翻转闸/兜底帧基准 (L1) */
    memcpy(act_l1, st.joint_deg, sizeof(act_l1));     /* 反馈钳制锚 (L1) */
    memset(act_last_ms, 0, sizeof(act_last_ms));      /* ⭐0924V: 锚年龄清零 */
    act_i = 0U;
    for (uint8_t j = 0; j < 6; j++) {
        cmd_rate[j]   = 0.0f;
        req_rate[j]   = 0.0f;     /* ⭐0924Y: 请求速率清零 (松手不残留) */
        reg_written[j] = 0U;      /* 0 = "还没写过", 首个接受帧立即写 */
        last_spd_ms[j] = 0U;
    }
    f_s[0] = f_s[1] = f_s[2] = f_s[3] = 0.0f;     /* 滤波器清零 */
    RLOG_W("RCCTL", "session start J1=%d J2=%d J3=%d J4=%d J5=%d J6=%d",
           (int)j1_t, (int)j2_t, (int)j3_t,
           (int)j4_t, (int)j5_t, (int)j6_t);
    return 1U;
}

/* ---------------------------------------------------------------- */
void ArmRcCtrl_Disarm(const char* why) {
    if (!armed) return;
    armed = 0U;
    session = 0U;
    rej_run = 0U;
    /* ⭐0924AD: 解除武装时把六轴 GoalSpeed 恢复 0(全速)。
     * pace_speeds()/松手刹车帧留在舵机 RAM 里的低速值若不清, 之后
     * m/P 0/TRI 中位/z 等轨迹命令只写 GoalPosition、从不重写 GoalSpeed,
     * 每根轴被残留速度各自封顶: 实测 J2/J3 被钉在 ~4.4°/s 爬行数分钟,
     * J4 先到 ⇒ 折叠<->中位过渡中腕先回直, 爪尖扫桌面 (2026-09-29)。
     * 写失败不重试 (cmdMove 入口的 TrajSpeedEnsureNormal 会再兜底写一遍)。 */
    for (uint8_t j = 0; j < 6U; j++) (void)SERVO_SetSpeed(j, 0U);
    RLOG_W("RCCTL", "DISARM: %s (停发流帧, 舵机自锁保持)", why ? why : "?");
}

void ArmRcCtrl_SetArmed(uint8_t on) {
    if (!on) { ArmRcCtrl_Disarm("rc go off"); return; }
    if (armed) return;
    if (IsEStop())              { RLOG_W("RCCTL", "arm refused: E-STOP");            return; }
    const ArmRcState* rc = ArmRc_Get();
    if (!rc->live)              { RLOG_W("RCCTL", "arm refused: RC 无信号");         return; }
    if (!rc->mid_valid ||
        ((rc->mid_mask & 0x0FU) != 0x0FU)) {
        RLOG_W("RCCTL", "arm refused: CH1~CH4 中位未标全 (先 `rc mid`)");            return;
    }
    session = 0U;            /* 下一个带意图的拍重新实测基准 */
    prev_intent = 0U;        /* 重新武装后首个空闲拍不发刹车帧 */
    /* ⭐0924U: 武装时自动恢复 sf=1。很多用户不记得 sf 0 会锁 RC, 且 home/fold
     *   等按键触发后会自动 disarm RC; 重新武装后应确保跟随开关打开, 否则
     *   下一拍 Tick 会因 sf=0 再次 disarm, 形成"双击 START 闪一下又锁"的现象。 */
    ArmMain_SetStreamFollowEn(1U);
    armed   = 1U;
    RLOG_W("RCCTL", "ARMED (rc go on): 左杆=J1/J2 右杆=J3/J4 L1/R1=J5 L2/R2=夹爪; 确认人不在扫过区");
}

uint8_t ArmRcCtrl_IsArmed(void) { return armed; }

/* ---------------------------------------------------------------- */
void ArmRcCtrl_Tick(void) {
    uint32_t now = HAL_GetTick();
    float dt = (last_tick_ms == 0U) ? 0.02f
                                    : ((float)(now - last_tick_ms) * 0.001f);
    last_tick_ms = now;
    if (dt < 0.005f) dt = 0.005f;
    if (dt > 0.100f) dt = 0.100f;
    if (!armed) return;

    const ArmRcState* rc = ArmRc_Get();

    /* ---- 安全闸 (任何一条不过 = 本拍冻结或解除) ---- */
    if (IsEStop()) { ArmRcCtrl_Disarm("E-STOP"); return; }

    uint8_t fresh4 = rc->live && rc->ch[0].fresh && rc->ch[1].fresh &&
                     rc->ch[2].fresh && rc->ch[3].fresh;
    if (fresh4) {
        last_fresh_ms = now;
    } else if (last_fresh_ms != 0U &&
               (uint32_t)(now - last_fresh_ms) > RC_STALE_MS) {
        ArmRcCtrl_Disarm("四杆信号超时");   /* 真断线 (与丢控不同) */
        return;
    }

    uint32_t flags = ArmMain_Flags();
    if ((flags & (ARM_MAIN_FLAG_TRAJ | ARM_MAIN_FLAG_CORR)) != 0U) {
        ArmRcCtrl_Disarm("外部轨迹/修正活跃");   /* P0-2: 网口永远赢 */
        return;
    }
    if (!ArmMain_StreamFollowEn()) {
        ArmRcCtrl_Disarm("sf 0 (跟随被暂停)");   /* 防 sf 恢复后积分器突进 */
        return;
    }
#if RC_HB_EN
    /* P0-3: CH6 值判据。判据= "读数 ≈正常位" 才是在线; 其余(中档/F-S 位)全算丢 */
    if (rc->ch[ARM_RC_CH_HB].fresh &&
        (fabsf((float)rc->ch[ARM_RC_CH_HB].width_us -
               (float)ARM_RC_HB_W_OK_US) > (float)ARM_RC_HB_TOL_US)) {
        if (!hb_loss_now) { hb_loss_now = 1U; hb_loss_start = now; }
        if ((uint32_t)(now - hb_loss_start) >= RC_HB_LOSS_MS) {
            ArmRcCtrl_Disarm("心跳丢控 (F/S)");  /* 冻结: 夹爪不追 F/S */
            return;
        }
    } else {
        hb_loss_now = 0U;
    }
#endif

    /* ---- 会话维护: 断流久了 ⇒ 重新实测基准 (积分器可能已漂移) ---- */
    if (session && (uint32_t)(now - last_accept_ms) > RC_RESYNC_MS) {
        session = 0U;
    }

    /* ---- 杆量 -> 速度 (⭐0924E: 直连, 不滤波 —— 接收机本地硬件延迟≈0,
     *   低通的 125ms 滞后就是"不灵敏"; 消 PWM 抖动靠死区 + 平方特性) ---- */
    f_s[0] = stick_norm(rc, 0U);
    f_s[1] = stick_norm(rc, 1U);
    f_s[2] = stick_norm(rc, 2U);
    f_s[3] = stick_norm(rc, 3U);

    /* ---- 实际位置轮询 (0924D: 优先读"欠账"最大的轴) ----
     *   谁的 |目标-实际| > 2° 就先读谁 (夹爪高速跟随时几乎每拍轮到它 ⇒
     *   钳制锚只陈旧 ~20ms, 200°/s 也连续不顿挫); 各轴都贴住时轮转保鲜。
     *   失败保旧值 (丢读率 ~5%/字段, 下一拍同轴因欠账仍在会重试)。 ---- */
    {
        float des[6];
        memcpy(des, prev_l1, sizeof(des));
        des[4] = j5_t;  des[5] = j6_t;
        uint8_t pi    = act_i;
        float   gmax  = 2.0f;
        uint8_t gmax_j = 0xFFU;
        for (uint8_t j = 0; j < 6; j++) {
            float gj = fabsf(des[j] - act_l1[j]);
            /* ⭐⭐0924Y: J5 按键按住期间**强制优先刷新它的锚**。J5 是唯一被开到
             *   ~300°/s 的轴 (杆轴只 25~50°/s), 而"欠账最大优先"在杆轴同时运动
             *   时与 J5 并列在 15° 欠账上 ⇒ 平手按轴号小的赢 ⇒ J5 的锚最坏要等
             *   ~200ms 才刷新, ±15° 反馈钳制于是把 J5 钉成"一格一格跳" (等效
             *   上限只有 75°/s, 且顿挫)。加偏置后按住期间 J5 每拍都读数 ⇒
             *   钳制窗相当于 15°/20ms = 750°/s, 不再成为限制。
             *   副作用可控: 杆轴请求速率(≤50°/s) 远低于它们锚 ~200ms 陈旧时的
             *   等效上限(~75°/s), 被"饿"几拍也吃不满、感觉不到。 */
            if (j == 4U && req_rate[4] > 0.0f) gj += 100.0f;
            if (gj > gmax) { gmax = gj; gmax_j = j; }
        }
        if (gmax_j != 0xFFU) pi = gmax_j;
        uint16_t rraw = SERVO_GetPositionRawTO(pi, 10U);
        if (rraw != 0xFFFFU) {
            act_l1[pi] = (float)rraw / 4095.0f * 360.0f;
            act_last_ms[pi] = now;                    /* ⭐0924V */
        }
        act_i = (uint8_t)(pi + 1U);
        if (act_i >= 6U) act_i = 0U;

        /* ⭐0924V 补读 (J4/J5/J6 防饿死): J1/J2/J3 承重轴丢读率 ~25% 且欠账
         *   大, "欠账最大优先"会让它们连续抢占 ⇒ J4/J5/J6 的锚几拍轮不到,
         *   反馈钳制把 J5/夹爪目标按在陈旧锚上 = J5 停滞拍 73%/夹爪 3°/s 慢爬
         *   的主因之一。第一读不是它们且其中有"欠账>5° 且 >100ms 没刷新"的轴,
         *   补读最陈旧的那轴 (最多 +1 次总线读/拍, 只在真的在跟随时发生)。 */
        if (pi < 3U) {
            uint8_t  pj  = 0xFFU;
            uint32_t age = 0U;
            for (uint8_t j = 3U; j < 6U; j++) {
                if (fabsf(des[j] - act_l1[j]) > 5.0f &&
                    (uint32_t)(now - act_last_ms[j]) > age) {
                    age = (uint32_t)(now - act_last_ms[j]);
                    pj  = j;
                }
            }
            if (pj != 0xFFU && age > 100U) {
                uint16_t rraw2 = SERVO_GetPositionRawTO(pj, 10U);
                if (rraw2 != 0xFFFFU) {
                    act_l1[pj] = (float)rraw2 / 4095.0f * 360.0f;
                    act_last_ms[pj] = now;
                }
            }
        }
    }

    spd_load();
    /* ⭐0924X: CSDN 关节空间映射。CH1=LX->J1, CH2=LY->J2, CH3=RY->J3, CH4=RX->J4。
     *   每拍只给微小增量(≈速度×dt)，等价于 Arduino writeMicroseconds()+delay(20)
     *   的平滑思想；舵机 GoalSpeed 由 pace_speeds() 按指令速率自适应。 */
    float vj[4];
    vj[0] = RC_SIGN_X  * f_s[0] * RC_VJ1_DEF;  /* J1 底座旋转 */
    vj[1] = RC_SIGN_Y  * f_s[1] * RC_VJ2_DEF;  /* J2 大臂俯仰 */
    vj[2] = RC_SIGN_Z  * f_s[2] * RC_VJ3_DEF;  /* J3 小臂俯仰 */
    vj[3] = RC_SIGN_J5 * f_s[3] * RC_VJ4_DEF;  /* J4 腕俯仰   */

    /* ⭐⭐0924Y: 把「请求速率」喂给节奏器 (见 req_rate[] 注释)。请求速率只由
     *   杆量/按键决定, 与"帧是否被接受 / 是否被反馈钳制"无关 —— 这才是
     *   "用户要的速度"。缺了它 GoalSpeed 会被钳制钉住, vj5 加到 300 也没用。 */
    for (uint8_t j = 0; j < 4U; j++) req_rate[j] = fabsf(vj[j]);
    req_rate[5] = 0.0f;                        /* 夹爪恒全速, 不进节奏器 */

    /* 夹爪滑杆: 绝对目标 + 限速率跟随 */
    float wmin = (float)rc->ch[ARM_RC_CH_CLAW].w_min;
    float wmax = (float)rc->ch[ARM_RC_CH_CLAW].w_max;
    if ((wmax - wmin) < 300.0f) { wmin = 1000.0f; wmax = 2000.0f; }
    float u = clampf(((float)rc->ch[ARM_RC_CH_CLAW].width_us - wmin) /
                     (wmax - wmin), 0.0f, 1.0f);
#if RC_SIGN_J6 < 0
    u = 1.0f - u;
#endif
    float j6tgt = lim_min[5] + u * (lim_max[5] - lim_min[5]);
    /* ⭐ 0924E: 夹爪改**绝对位置实时跟随**, 无速率限制 (用户定案"不要速度
     *   限制")。滑杆位置 = 夹爪目标; 舵机全速追赶; 反馈钳制 act±18° 保证
     *   永远 < 25° 跳变闸, 快扫滑杆也只表现为舵机全速连续滑, 不会拒帧。
     * ⭐ 0924Q: 松开按键瞬间把目标钉在实际位置, 立即停 (不再滑向滑杆终点)。 */
    uint8_t claw_btn = ArmRc_FnDown(ARM_RC_FN_CLAW_OPEN) ||
                       ArmRc_FnDown(ARM_RC_FN_CLAW_CLOSE);
    if (claw_btn) {
        j6_t = j6tgt;
    } else if (have_deg) {
        /* ⭐0924V: 钉在"最近下发的 J6"而不是 act_l1[5]。act 是低频轮询锚
         *   (J1/J2/J3 欠账时几拍轮不到 J6), 松开瞬间钉陈旧 act 会把目标**往回
         *   拉** = 夹爪按住慢爬/松开回缩。舵机本就朝 j6_sent 走, 钉它 = 真即停。 */
        j6_t = j6_sent;
    }
    float j6n = j6_t;

    /* ⭐0924X: 操作意图。CH1~CH4 直控 J1~J4；J5_CCW/J5_CW 按键控 J5。
     *   笛卡尔意图、J4 按键块全部删除 —— 彻底绕开 IK/FK。 */
    uint8_t stick_intent = (fabsf(vj[0]) > 0.0f) || (fabsf(vj[1]) > 0.0f) ||
                           (fabsf(vj[2]) > 0.0f) || (fabsf(vj[3]) > 0.0f);
    float j5v = 0.0f;
    if (ArmRc_FnDown(ARM_RC_FN_J5_CCW)) j5v -= spd_cfg.vj5;
    if (ArmRc_FnDown(ARM_RC_FN_J5_CW))  j5v += spd_cfg.vj5;
    req_rate[4] = fabsf(j5v);   /* ⭐0924Y: J5 请求速率 = 按键设定的 vj5 */
    uint8_t j5_intent   = (fabsf(j5v) > 0.0f);
    uint8_t claw_intent = (claw_btn && have_deg)
                              ? (fabsf(j6_t - j6_sent) > 0.05f) : 0U; /* ⭐0924V */

    uint8_t intent = stick_intent || j5_intent || claw_intent;
    if (!intent) {
        /* ⭐0924AB 松手帧: 根治"所有舵机松手后回调"。
         * 旧实现(0924Y): 松手时把目标拉回"实际位置" ⇒ 舵机正在向领先 15° 的
         *   目标运动时, 新目标比当前位置还靠后 ⇒ 必须**回调**才能到达。
         * 新实现:
         *   - 目标保持为当前 jN_t (已经过反馈钳制, 最多只领先实际 15°); 不再
         *     把目标往回拉。
         *   - 立即清零 cmd_rate/req_rate, 并把 last_spd_ms 清零, 让本拍末尾的
         *     pace_speeds() 立刻把 GoalSpeed 压到最低。滑行距离 = 正向 LEAD
         *     (最多 2~8°), 且方向始终向前, 绝不回调。
         *   - 更新 prev_l1 到刹车目标, 下次推杆从停止点起步, 无起步跳变。 */
        if (prev_intent && session && have_deg &&
            (uint32_t)(now - last_brake_ms) > 30U) {
            float bl1[6], bd[6];
            bl1[0] = j1_t;  bl1[1] = j2_t;  bl1[2] = j3_t;
            bl1[3] = j4_t;  bl1[4] = j5_t;  bl1[5] = j6_sent;
            for (uint8_t j = 0; j < 6; j++) {
                bl1[j] = clampf(bl1[j], lim_min[j], lim_max[j]);
                bd[j]  = ArmMain_ServoToJoint1(bl1[j], j);
            }
            if (ArmMain_StreamPoseOk(bd)) {
                arm_rc_ctrl_in_frame = 1U;
                int bst = ArmApi_StreamJoints(bd);
                arm_rc_ctrl_in_frame = 0U;
                if (bst == ARM_API_OK) {
                    j4_sent = bl1[3];
                    j6_sent = bl1[5];
                    memcpy(prev_l1, bl1, sizeof(prev_l1));
                }
            }
            /* 立即降速: 打破 pace_speeds 的 150ms 节流, 让 GoalSpeed 本拍就
             * 压到最低 (≈1 step/s), 把滑行距离限制在 LEAD 窗口内。 */
            for (uint8_t j = 0; j < 6; j++) {
                cmd_rate[j]   = 0.0f;
                req_rate[j]   = 0.0f;
                last_spd_ms[j]= 0U;
            }
            last_brake_ms = now;
        }
        /* 0924AP: 不再压保持下限, 让流控直接跑。 */
        prev_intent = 0U;
        return;                      /* 空闲: 不再连发 (看门狗自然收流) */
    }
    prev_intent = 1U;

    if (!session) {
        /* 会话建立尝试限 5Hz: GetStatus 走舵机总线, 50Hz 重试既刷屏又挤总线
         * (本机舵机读成功率 ~95%/字段)。 */
        if ((uint32_t)(now - last_try_ms) < 200U) return;
        last_try_ms = now;
        if (!session_start(rc, now)) return;
    }

    /* ---- ⭐0924X 关节空间直控 + 顶到包线软停 ----
     * 不再调用 IK/FK: CH1~CH4 直接积分到 J1~J4 目标, J5_CCW/CW 按键积分 J5。
     * 五档缩步长 {1, 1/2, 1/4, 1/8, 0} 保留: 朝桌面/立柱推时自动减速软停,
     * 不触发 DISARM。积分器只在帧真被接受时提交, 失败则回退 prev_l1。 */
    float j5lo = (lim_min[4] > RC_J5_SAFE_MIN) ? lim_min[4] : RC_J5_SAFE_MIN;
    float j5hi = (lim_max[4] < RC_J5_SAFE_MAX) ? lim_max[4] : RC_J5_SAFE_MAX;
    float j5n = clampf(j5_t + j5v * dt, j5lo, j5hi);

    static const float scale_tab[5] = {1.0f, 0.5f, 0.25f, 0.125f, 0.0f};
    uint8_t sent = 0U;                    /* 0=全挡不通(软停) 1=已发 2=流层拒 */
    int     rej_code    = 0;              /* 流层拒帧返回码 (E-STOP 判别) */
    for (uint8_t si = 0U; si < 5U && sent == 0U; si++) {
        float s   = scale_tab[si];
        float l1[6], deg[6];
        /* J1~J4 直接按杆量积分; 先钳到 L 限位防 windup */
        l1[0] = clampf(j1_t + vj[0] * s * dt, lim_min[0], lim_max[0]);
        l1[1] = clampf(j2_t + vj[1] * s * dt, lim_min[1], lim_max[1]);
        l1[2] = clampf(j3_t + vj[2] * s * dt, lim_min[2], lim_max[2]);
        l1[3] = clampf(j4_t + vj[3] * s * dt, lim_min[3], lim_max[3]);
        l1[4] = j5n;
        l1[5] = j6n;
        /* ⭐0924AB 反馈钳制: 恢复足够大的双向窗口, 同时保留 0924Z 的刹车帧
         * 逻辑来避免回调。
         * 0924Z 的问题: 正向 LEAD 被压到 2~8°, 舵机目标只比实际超前一点点,
         *   导致"只能动一点点"、速度起不来。
         * 0924Y 的问题: 松手刹车帧把目标拉回"实际位置", 舵机惯性冲过头后必须
         *   **回调**才能到新目标。
         * 0924AB 取两者之长:
         *   - 运动中: 恢复 ±15°(关节)/±18°(夹爪) 大窗口, 让舵机能全力跑;
         *   - 松手时: 刹车帧**不拉回目标**, 保持当前 jN_t 并立即把 GoalSpeed
         *     压到最低, 滑行距离 ≤15° 且始终向前, 绝不回调。 */
        for (uint8_t j = 0; j < 6; j++) {
            float lead_pos = (j == 5U) ? 18.0f : 15.0f;
            float lead_neg = (j == 5U) ? 18.0f : 15.0f;
            float lc = clampf(l1[j], act_l1[j] - lead_neg, act_l1[j] + lead_pos);
            lc = clampf(lc, lim_min[j], lim_max[j]);
            l1[j] = lc;
        }
        for (uint8_t j = 0; j < 6; j++) deg[j] = ArmMain_ServoToJoint1(l1[j], j);

        /* 包线预检 (与流层同一判据): 不过 = 该档越界, 缩步长再试。
         * ⭐0924AJ 带实际位姿参考: 实际已在桌下时只要求目标不低于实际,
         *   修"触限位后六轴全冻" —— 旧绝对判据会把所有方向全拒。 */
        if (!ArmMain_StreamPoseOkAct(deg, act_l1)) continue;

        arm_rc_ctrl_in_frame = 1U;
        int sst = ArmApi_StreamJoints(deg);
        arm_rc_ctrl_in_frame = 0U;
        if (sst != ARM_API_OK) { sent = 2U; rej_code = sst; break; }

        /* ⭐ 提交: 关节积分器始终跟到实际下发的 L1 值 (含限位/反馈钳制),
         *   防止抵住限位或舵机滞后时 windup; J4 随摇杆实时走。 */
        j1_t = l1[0];  j2_t = l1[1];  j3_t = l1[2];  j4_t = l1[3];
        j5_t = l1[4];  j6_t = l1[5];
        j6_sent = l1[5];              /* ⭐0924V: 松杆钉位/夹爪意图基准 */
        j4_sent = l1[3];
        /* ⭐0924E 指令速率 EMA (开环节奏器输入): 用指令步长估计该轴速率,
         *   与舵机实际位置无关 (差距项=反馈环="晃"的根源, 已删)。 */
        for (uint8_t j = 0; j < 6; j++) {
            float inst = fabsf(l1[j] - prev_l1[j]) / dt;
            cmd_rate[j] = cmd_rate[j] * 0.5f + inst * 0.5f;
        }
        memcpy(last_deg, deg, sizeof(deg));
        memcpy(prev_l1, l1, sizeof(prev_l1));   /* 兜底帧/失败回退基准 */
        have_deg = 1U;
        rej_run = 0U;
        last_accept_ms = now;
        sent = 1U;
        break;                        /* 已发出, 不再试更小步长 */
    }

    /* ⭐0924X 抗 windup: 帧没被接受 ⇒ 积分器回退到上一成功帧,
     *   松杆/松键后 intent 立即清零, 下一拍从安全位继续。 */
    if (sent != 1U) {
        j1_t = prev_l1[0];  j2_t = prev_l1[1];  j3_t = prev_l1[2];
        j4_t = prev_l1[3];  j5_t = prev_l1[4];  j6_t = j6_sent;
    }

    if (sent == 2U) {
        if (rej_code == ARM_API_ERR_ESTOP) {
            ArmRcCtrl_Disarm("E-STOP");
        } else {
            /* ⭐ 跳变拒帧自愈 (0924C): 会话作废(下拍实测重基准) + 全轴提速
             *   追差 + 下一拍立刻刷新实际位置锚。0924B 旧行为 = 干等 15 帧
             *   熔断锁死, 且 resync 又把 GoalSpeed 压回 2 ⇒ 关差 3-4s=拖尾。 */
            session = 0U;
            for (uint8_t j = 0; j < 6; j++) {
                (void)SERVO_SetSpeed(j, 0U);     /* 0=全速关差 (0924F 单位修正) */
                reg_written[j] = 0U;             /* 归零=下拍 pace 立即重写正确值 */
            }
            if (++rej_run >= RC_REJ_DISARM) {
                ArmRcCtrl_Disarm("流层连续拒帧 (跳变)");
            }
        }
    } else if (sent == 0U && have_deg) {
        /* ⭐0924X 兜底: 关节五档全被包线拦死时, J5/夹爪帧**不许连坐** ——
         * 用上一帧被接受的位姿 (last_deg) + 新 j5/j6 (已各自钳在
         * 安全窗/限位内) 直接发。last_deg 本来就过过预检, 只有 j5/j6 变;
         * 若预检仍不过 (J5 摆把爪尖压到桌下) = 该轴方向真的不安全, 冻住。 */
        float dg[6], l1b[6];
        memcpy(dg, last_deg, sizeof(dg));
        memcpy(l1b, prev_l1, sizeof(l1b));
        /* 兜底帧同样过反馈钳制 (与主循环一致的小窗口) */
        float j5lead_pos = req_rate[4] * dt * 1.5f;
        if (j5lead_pos < 2.0f) j5lead_pos = 2.0f;
        if (j5lead_pos > 8.0f) j5lead_pos = 8.0f;
        float l5 = clampf(j5n, act_l1[4] - 1.0f, act_l1[4] + j5lead_pos);
        float l6 = clampf(j6n, act_l1[5] - 18.0f, act_l1[5] + 18.0f);
        dg[4] = ArmMain_ServoToJoint1(l5, 4);
        dg[5] = ArmMain_ServoToJoint1(l6, 5);
        l1b[4] = l5;  l1b[5] = l6;
        if (ArmMain_StreamPoseOkAct(dg, act_l1)) {
            arm_rc_ctrl_in_frame = 1U;
            int sst2 = ArmApi_StreamJoints(dg);
            arm_rc_ctrl_in_frame = 0U;
            if (sst2 == ARM_API_OK) {
                j5_t = l5;  j6_t = l6;        /* ⭐0924D: 提交钳制值, 防欠账 */
                j6_sent = l6;                 /* ⭐0924V */
                memcpy(last_deg, dg, sizeof(last_deg));
                memcpy(prev_l1, l1b, sizeof(prev_l1));
                rej_run = 0U;
                last_accept_ms = now;
                sent = 1U;
            } else {
                sent = 2U;  rej_code = sst2;   /* 走上面统一的拒帧自愈 */
            }
        }
    }
    if (sent == 0U) {
        /* 全部尝试失败: 已顶死在包线上 ⇒ 软停本拍不动。
         * 不熔断、不自锁 —— 松杆或反向杆立即恢复。限 2s 一条防刷屏。 */
        if ((uint32_t)(now - last_block_log) > 2000U) {
            RLOG_W("RCCTL", "已到包线/可达边界: 该方向软停 (其他方向仍可动)");
            last_block_log = now;
        }
    }

    /* ⭐ 0924E 目标节奏器 (每拍末尾统一调): GoalSpeed = 指令速率 (开环,
     *   无差距项 = 无反馈环 = 不晃; 松杆速率归零 = 立即停)。have_deg 之前
     *   不写 —— 首帧前保持 stream_enter 的全速 0 = 起步零迟滞。 */
    if (session && have_deg) {
        pace_speeds(now);  /* ⭐0924Y: GoalSpeed = max(指令速率, 请求速率) */
    }
}

#endif /* ARM_MASTER_BUILD */
