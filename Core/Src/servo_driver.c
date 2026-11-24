/*
 * STS3215 高层驱动。
 * 把 STS3215 控制表地址和 SCS 协议细节封装起来,
 * 让 main.c 可以直接按"关节、角度、速度、扭矩"进行控制。
 *
 * 关键差异(vs 舵机):
 *   - AngleToRaw: 0-360° → 0-4095 (was 0-300° → 0-1023)
 *   - PID 增益在 EEPROM(addr 21/22/23), 写入需 Torque OFF
 *   - Mode(33)=0 为舵机/位置模式 (1=恒速模式会自转; 2=PWM; 3=步进)
 *   - 无 CW/CCW Angle Limit, 改用 Min/Max Angle(addr 9/11)
 *   - INST_RESET(0x0A) 替代 INST_REBOOT(0x08)
 */
#include "servo_driver.h"
#include "scs_protocol.h"
#include "busmap_cfg.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

/* 六个逻辑关节到舵机 ID 和物理总线的映射。
 * JOINT_BUS 为运行时可改（bus mode 诊断功能），默认 J1-J3=Bus A, J4-J6=Bus B。 */
const uint8_t JOINT_ID[6]  = {1, 2, 3, 4, 5, 6};
uint8_t JOINT_BUS[6] = {BUS_A, BUS_A, BUS_A, BUS_B, BUS_B, BUS_B};

/* ===== Bus mode: 运行时切换关节-总线映射（诊断用） ===== */
uint8_t SERVO_SetBusMode(uint8_t mode) {
    switch (mode) {
    case 1:
        for (uint8_t j = 0; j < 6; j++) JOINT_BUS[j] = BUS_A;
        break;
    case 2:
        for (uint8_t j = 0; j < 6; j++) JOINT_BUS[j] = BUS_B;
        break;
    default:
        mode = 0;
        JOINT_BUS[0] = BUS_A; JOINT_BUS[1] = BUS_A; JOINT_BUS[2] = BUS_A;
        JOINT_BUS[3] = BUS_B; JOINT_BUS[4] = BUS_B; JOINT_BUS[5] = BUS_B;
        break;
    }
    return mode;
}

uint8_t SERVO_GetBusMode(void) {
    if (JOINT_BUS[0] == BUS_B && JOINT_BUS[5] == BUS_B) return 2;
    if (JOINT_BUS[0] == BUS_A && JOINT_BUS[5] == BUS_A) return 1;
    return 0;
}

/* ===== 逐 ID 自动路由 (2026-11-02, 0924BA) =====
 * 目的: **不改任何接线**的前提下, 让"6 颗舵机插在哪个座子上"不再影响使用。
 *
 * 物理上仍是两条独立总线(74HC126 门1 驱动 Bus A / 门3 驱动 Bus B, 输出没有
 * 短接), 但"关节 -> 总线"的映射由**实测应答**决定, 而不是编译期写死的
 * A/A/A/B/B/B。于是舵机插错座子、换过舵机、改过 ID 都能自动纠正 —— 使用
 * 体验等价于"一条逻辑总线, 按 ID 寻址", 而两条 UART 依旧并行(性能不降)。
 *
 * 三段式, 正常开机的额外开销只有 6 次 PING(亚毫秒级):
 *   pass1 按现有映射逐轴 PING, 6/6 命中直接返回 —— 布局正确时启动时间不变;
 *   pass2 失败的轴用**投票**在"当前路由/上次确认路由"上补测, 命中即保留;
 *   pass3 仍无定论的轴才双总线全扫(带空段短路)。
 *
 * 位掩码 (bit j = 关节 j, 0-based):
 *   *miss  —— 两条总线都不应答 => **保持原路由不变**(见下方 BUGFIX)
 *   *dup   —— 同一 ID 在两条总线都应答(两片舵机 ID 撞了/两段并线) => 保持原路由
 *   *moved —— 本次探测实际改动了路由
 * 返回在线关节数 0..6。
 *
 * ===== 0924BB BUGFIX: 离线时**绝不能**回落到编译期默认 A/A/A/B/B/B =====
 * 0924BA 的 pass2 用 `def = (j<3)?A:B` 做回落, 在"六轴全接 B 段"的板子上
 * 那个 def 本身就是错的。一次 PING 抖动(单帧误码 ~37%) 就把 J1~J3 从 B 段
 * 改到 A 段空线, 之后该轴读位置永远打空段(RC 丢轴), 而且下次 pass1 继续
 * 失败 -> 再进 pass2 -> 再改, 形成自我维持的错误状态。
 * 现场指纹: `b auto` 打出 "J3 -> Bus A  MISSING", 再跑一次又变回 Bus B,
 * 且每次重路由的轴数在 1/7 之间乱跳。
 * 修法: 用 s_map_conf[] 保存"上次实测确认过的路由", MISSING 时保持原路由,
 *       并加入 >=2/3 投票, 单轮抖动不再改变任何路由。 */
#define BUSMAP_PING_MS     30U
#define BUSMAP_VOTE_ROUNDS 3U   /* 每候选投 3 轮 */
#define BUSMAP_VOTE_MIN    2U   /* >=2/3 才认定 */

/* 上次实测确认过的路由。初值只是占位, s_map_known 的位才是"可信"标记。 */
static uint8_t s_map_conf[6] = {BUS_A, BUS_A, BUS_A, BUS_B, BUS_B, BUS_B};
static uint8_t s_map_known  = 0U;   /* bit j: s_map_conf[j] 来自实测, 不是初值 */

static uint8_t busmap_count(uint8_t m) {
    uint8_t n = 0U;
    for (uint8_t i = 0U; i < 6U; i++) if (m & (uint8_t)(1U << i)) n++;
    return n;
}

/* 在一条总线上对某 ID 连投 rounds 轮, 返回命中轮数 */
static uint8_t busmap_vote(uint8_t bus, uint8_t id, uint8_t rounds) {
    uint8_t ok = 0U;
    for (uint8_t r = 0U; r < rounds; r++) {
        if (r != 0U) HAL_Delay(1);
        if (DXL_Ping(bus, id, BUSMAP_PING_MS) == HAL_OK) ok++;
    }
    return ok;
}

/* ===== 路由持久化 (0924BC) =====
 * JOINT_BUS[] 和 s_map_conf[] 都是 RAM, 一复位就回到编译期默认 A/A/A/B/B/B。
 * 集成板"六颗全在 B 段"时那个默认本身是错的, 于是每次上电都要重新赌一次
 * AutoMap; 赌输的那一轴就停在空的 A 段, 表现成"上次好了重启又缺轴"。
 * 把实测确认过的路由写进扇区 11 区 G, 上电先灌回 JOINT_BUS -> pass1 直接
 * 查正确的段, 不再依赖当轮误码运气。 */

/* 上次写进 flash 的内容, 用于"没变就不擦扇区"(擦写寿命 + 启动耗时)。 */
static uint8_t s_saved_bus[6]  = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
static uint8_t s_saved_mask    = 0U;

uint8_t SERVO_LoadSavedRoute(void) {
    uint8_t bus[6];
    uint8_t mask = 0U;
    if (!BusMapCfg_Load(bus, &mask)) return 0U;

    for (uint8_t j = 0U; j < 6U; j++) {
        uint8_t bit = (uint8_t)(1U << j);
        if (!(mask & bit)) continue;
        JOINT_BUS[j]  = bus[j];
        s_map_conf[j] = bus[j];
        s_map_known  |= bit;
    }
    for (uint8_t j = 0U; j < 6U; j++) s_saved_bus[j] = bus[j];
    s_saved_mask = mask;
    return (mask != 0U) ? 1U : 0U;
}

/* 把"已被实测确认"的轴写进 flash。mask=0(一颗都没确认)时不动 flash。
 * ⚠ 阻塞 ~1s —— 只许启动期或控制台命令调。 */
uint8_t SERVO_SaveRoute(void) {
    uint8_t bus[6];
    uint8_t mask = 0U, dirty = 0U;

    for (uint8_t j = 0U; j < 6U; j++) {
        uint8_t bit = (uint8_t)(1U << j);
        bus[j] = JOINT_BUS[j];
        if (!(s_map_known & bit)) continue;
        mask |= bit;
        if (bus[j] != s_saved_bus[j]) dirty = 1U;
    }
    if (mask == 0U) return 0U;
    if ((mask == s_saved_mask) && (dirty == 0U)) return 1U;   /* 没变: 不擦扇区 */

    if (BusMapCfg_Save(bus, mask) != CFG_FLASH_OK) return 0U;
    for (uint8_t j = 0U; j < 6U; j++) s_saved_bus[j] = bus[j];
    s_saved_mask = mask;
    return 1U;
}

/* 无条件把当前 JOINT_BUS[] 全 6 轴写进 flash, 并把 s_map_conf/s_map_known
 * 一并置为"已确认"。用于 `b 0/1/2` —— 用户显式钉死路由时, 意图明确,
 * 就该跨复位保留; 否则重启又回编译期默认, 等于白敲。 */
uint8_t SERVO_SaveRouteAll(void) {
    uint8_t bus[6];
    for (uint8_t j = 0U; j < 6U; j++) bus[j] = JOINT_BUS[j];
    if (BusMapCfg_Save(bus, 0x3FU) != CFG_FLASH_OK) return 0U;
    for (uint8_t j = 0U; j < 6U; j++) {
        s_saved_bus[j] = bus[j];
        s_map_conf[j]  = bus[j];
    }
    s_saved_mask = 0x3FU;
    s_map_known  = 0x3FU;
    return 1U;
}

uint8_t SERVO_ClearSavedRoute(void) {
    for (uint8_t j = 0U; j < 6U; j++) s_saved_bus[j] = 0xFFU;
    s_saved_mask = 0U;
    s_map_known  = 0U;
    return (BusMapCfg_Clear() == CFG_FLASH_OK) ? 1U : 0U;
}

uint8_t SERVO_AutoMapBuses(uint8_t *miss, uint8_t *dup, uint8_t *moved) {
    uint8_t miss_m = 0U, dup_m = 0U, moved_m = 0U;
    uint8_t located = 0U;   /* bit j = 该轴路由已由实测确定 */

    /* ---- pass 1: 现有映射快查 (每轴 1 次 PING, 内部已含 2 次重试) ---- */
    for (uint8_t j = 0U; j < 6U; j++) {
        if (DXL_Ping(JOINT_BUS[j], JOINT_ID[j], BUSMAP_PING_MS) == HAL_OK) {
            located     |= (uint8_t)(1U << j);
            s_map_conf[j] = JOINT_BUS[j];
            s_map_known  |= (uint8_t)(1U << j);
        }
    }
    if (located == 0x3FU) {                 /* 布局正确: 零额外开销返回 */
        if (miss)  *miss  = 0U;
        if (dup)   *dup   = 0U;
        if (moved) *moved = 0U;
        return 6U;
    }

    /* ---- pass 2: 失败轴投票补测 (候选 = 当前路由 / 上次确认路由) ----
     * 命中就保留原路由, 一次都不改 —— 抖动不再产生"改坏路由"的副作用。
     * 上次确认路由与当前不同且命中 => 自愈(把被改坏的路由撤回)。 */
    for (uint8_t j = 0U; j < 6U; j++) {
        uint8_t bit = (uint8_t)(1U << j);
        if (located & bit) continue;

        uint8_t id  = JOINT_ID[j];
        uint8_t c0  = JOINT_BUS[j];                                   /* 当前 */
        uint8_t c1  = (s_map_known & bit) ? s_map_conf[j] : c0;       /* 上次实测 */
        uint8_t alt = (c1 != c0) ? 1U : 0U;

        uint8_t v0 = busmap_vote(c0, id, BUSMAP_VOTE_ROUNDS);
        uint8_t v1 = alt ? busmap_vote(c1, id, BUSMAP_VOTE_ROUNDS) : 0U;

        if (v0 >= BUSMAP_VOTE_MIN && v1 >= BUSMAP_VOTE_MIN) {
            /* 两条总线都应答: 同 ID 两片舵机, 或两段被并线。不猜, 保住
             * 一条已知能用的路由, 只置 dup 告警。 */
            dup_m |= bit;
            located |= bit;
            s_map_conf[j] = c0; s_map_known |= bit;
        } else if (v0 >= BUSMAP_VOTE_MIN) {
            located |= bit;                        /* 原路由对, 只是抖动 */
            s_map_conf[j] = c0; s_map_known |= bit;
        } else if (v1 >= BUSMAP_VOTE_MIN) {
            JOINT_BUS[j] = c1;                     /* 自愈: 撤回被改坏的路由 */
            moved_m |= bit;
            located |= bit;
            s_map_conf[j] = c1; s_map_known |= bit;
        }
        /* 都不够票 -> 留给 pass 3 */
    }

    /* ---- pass 3: 双总线全扫 (只针对仍无定论的轴) ----
     * 带"空段短路": 一条总线上一个 ID 都不应答就认定该段无设备, 剩余轴
     * 不再探它 —— 集成板常见"A 段空着", 不短路每轴要多烧 ~180ms。 */
    uint8_t need = (uint8_t)(~located & 0x3FU);
    if (need != 0U) {
        uint8_t a_live = 0U, b_live = 0U;
        uint8_t va[6] = {0U, 0U, 0U, 0U, 0U, 0U};
        uint8_t vb[6] = {0U, 0U, 0U, 0U, 0U, 0U};

        /* 第 1 轮同时当"该段是否有设备"的探针 */
        for (uint8_t j = 0U; j < 6U; j++) {
            if (!(need & (uint8_t)(1U << j))) continue;
            if (DXL_Ping(BUS_A, JOINT_ID[j], BUSMAP_PING_MS) == HAL_OK) { va[j] = 1U; a_live = 1U; }
            if (DXL_Ping(BUS_B, JOINT_ID[j], BUSMAP_PING_MS) == HAL_OK) { vb[j] = 1U; b_live = 1U; }
        }
        /* 剩余轮次: 空段直接跳过 */
        for (uint8_t r = 1U; (r < BUSMAP_VOTE_ROUNDS) && (a_live || b_live); r++) {
            HAL_Delay(1);
            for (uint8_t j = 0U; j < 6U; j++) {
                if (!(need & (uint8_t)(1U << j))) continue;
                if (a_live && DXL_Ping(BUS_A, JOINT_ID[j], BUSMAP_PING_MS) == HAL_OK) va[j]++;
                if (b_live && DXL_Ping(BUS_B, JOINT_ID[j], BUSMAP_PING_MS) == HAL_OK) vb[j]++;
            }
        }

        for (uint8_t j = 0U; j < 6U; j++) {
            uint8_t bit = (uint8_t)(1U << j);
            if (!(need & bit)) continue;

            if (va[j] >= BUSMAP_VOTE_MIN && vb[j] >= BUSMAP_VOTE_MIN) {
                dup_m |= bit;                      /* 两段都应答: 保当前路由 */
                located |= bit;
                s_map_conf[j] = JOINT_BUS[j]; s_map_known |= bit;
            } else if (va[j] >= BUSMAP_VOTE_MIN) {
                if (JOINT_BUS[j] != BUS_A) moved_m |= bit;
                JOINT_BUS[j]  = BUS_A;
                located      |= bit;
                s_map_conf[j] = BUS_A; s_map_known |= bit;
            } else if (vb[j] >= BUSMAP_VOTE_MIN) {
                if (JOINT_BUS[j] != BUS_B) moved_m |= bit;
                JOINT_BUS[j]  = BUS_B;
                located      |= bit;
                s_map_conf[j] = BUS_B; s_map_known |= bit;
            } else {
                /* 真离线: **不改路由**。上层照常报 MISSING, 但路由原样保留,
                 * 下次抖动恢复就能直接命中, 不会越探越坏。 */
                miss_m |= bit;
            }
        }
    }

    if (miss)  *miss  = miss_m;
    if (dup)   *dup   = dup_m;
    if (moved) *moved = moved_m;

    /* 只在**路由真的被改动**时落盘(moved!=0): 正常布局(pass1 全中)直接
     * 早返回不写; 抖动经投票确认"原路由就对"也不写。这样启动期不会因为
     * 一次探针就擦扇区 11 —— 擦整扇区是有风险的(掉电会连带丢 PID/校准),
     * 必须留给"确实需要改路由"这种明确场景。 */
    if (moved != 0U) (void)SERVO_SaveRoute();

    return busmap_count(located);
}

static uint8_t init_p_gain[6] = {16, 32, 16, 16, 16, 16};
static uint8_t init_i_gain[6] = {4, 4, 4, 4, 4, 4};
static uint8_t init_d_gain[6] = {12, 12, 12, 12, 12, 12};
static uint16_t init_moving_speed = STS_DEFAULT_GOAL_SPEED;

/* ===== 舵机内部加速度 (addr 41) — 运行时可调 (AP_SERVO_ACCEL / SP 44) =====
 * 0924AP: 恢复单一值 50, 不再在轨迹与流控之间切换, 也不额外限制尾段/保持。
 * 50 比旧默认 10 能跟上 25~35°/s 规划, 又比 80 在 RC 流控负载轴上柔和。
 * SP 44 调的是全局加速度; 改后 'i' 重新下发或复位生效。 */
uint8_t sts_accel_value = 50U;
void SERVO_SetAccel(uint8_t a) { sts_accel_value = a; }

/* ===== LED 状态源 (0924CN): 驱动层统一挂钩 =====
 * 放在驱动层而不是 main.c 散布的原因: 力矩开关和目标位置下发的路径太多
 * (t/i/ma/teach/c 向导/RC/fi/轨迹/回放/单轴), 上层各处都记状态必然漏。
 * 驱动层只有这几个汇入点, 挂一次全覆盖。 */
static uint8_t  s_tq_state[6] = {0, 0, 0, 0, 0, 0};  /* 各轴力矩意图 0/1 */
static uint8_t  s_tq_on_cnt   = 0U;                  /* 力矩 ON 轴数 */
static volatile uint32_t s_last_goal_tick = 0U;      /* 最近一次目标位置下发时刻 */
static void LED_TqTrack(uint8_t joint, uint8_t en) {
    if (joint >= 6U) return;
    if (en && !s_tq_state[joint]) { s_tq_state[joint] = 1U; s_tq_on_cnt++; }
    else if (!en && s_tq_state[joint]) { s_tq_state[joint] = 0U; if (s_tq_on_cnt) s_tq_on_cnt--; }
}
uint8_t  SERVO_TorqueOnCount(void) { return s_tq_on_cnt; }      /* 0=全部松力矩 */
uint32_t SERVO_LastGoalTick(void)  { return s_last_goal_tick; } /* 0=开机以来没下发过 */

void SERVO_SetInitPID(const uint8_t p[6], const uint8_t i[6], const uint8_t d[6]) {
    for (uint8_t j = 0; j < 6; j++) {
        init_p_gain[j] = p[j];
        init_i_gain[j] = i[j];
        init_d_gain[j] = d[j];
    }
}

void SERVO_SetInitSpeed(uint16_t speed) {
    init_moving_speed = (speed > 4095U) ? 4095U : speed;
}

/* ===== 角度 <-> 原始值转换 =====
 * STS3215: 0-360° → 0-4095 (12-bit, 0.0879°/step) */
static inline uint16_t AngleToRaw(float angle_deg) {
    if (angle_deg < 0.0f) angle_deg = 0.0f;
    if (angle_deg > 360.0f) angle_deg = 360.0f;
    /* 四舍五入: 加0.5f 再转换为整数, 避免截断导致的系统误差(如180°→2047而不是2048) */
    return (uint16_t)(angle_deg / 360.0f * 4095.0f + 0.5f);
}

static inline float RawToAngle(uint16_t raw) {
    return (raw / 4095.0f) * 360.0f;
}

/* ===== RESET a servo (SCS INST_RESET = 0x0A) =====
 * STS3215 没有 REBOOT 指令, 用 RESET 恢复出厂或清除错误状态。
 * 写 EEPROM 后需要 RESET 才能生效。 */
static HAL_StatusTypeDef STS_Reset(uint8_t bus, uint8_t id) {
    uint8_t tx[8];
    uint16_t tx_len = DXL_BuildPacket(id, INST_RESET, NULL, 0, tx);

    /* Fire-and-forget: servo may not respond before resetting */
    HAL_StatusTypeDef st = DXL_Transmit(bus, tx, tx_len);

    /* Wait for servo to finish resetting (~500ms typical) */
    HAL_Delay(600);
    return st;
}

/* ===== Broadcast RESET (只对"确实挂着轴"的总线广播) =====
 * 尝试恢复卡在错误/告警状态的舵机。
 * 2026-09-11: 改为按 JOINT_BUS[] 判断哪条总线有轴。主臂 6 颗全并联在
 * 一条总线上, 朝另一条空总线广播 RESET 既无意义, 又会在 PB10/PB11 仍
 * 搭在舵机 DATA 节点上时, 把复位帧叠到真帧上把总线搅乱。 */
void SERVO_RebootAll(void) {
    uint8_t tx[8];
    uint16_t tx_len = DXL_BuildPacket(DXL_BROADCAST_ID, INST_RESET, NULL, 0, tx);
    for (uint8_t bus = 0U; bus < 2U; bus++) {
        uint8_t has_joint = 0U;
        for (uint8_t j = 0U; j < 6U; j++) {
            if (JOINT_BUS[j] == bus) { has_joint = 1U; break; }
        }
        if (!has_joint) continue;
        (void)DXL_Transmit(bus, tx, tx_len);
        HAL_Delay(400);
    }
    HAL_Delay(800);   /* 等所有舵机重启 */
}

/* ===== 单字节读 + 重试 =====
 * 2026-09-10: 换 J2/J3 舵机后实测 J3 偶发读超时(开机 WARN、Y 命令连报 FAILED),
 * 单次读失败不足以判定"舵机离线"。这里做 tries 次重试(每次间隔 3ms)再下结论,
 * 避免偶发丢帧被当成"离线/写失败"。
 * 注意: DXL_Read 收到任何告警位都会返回 HAL_ERROR —— 因此重试仍失败只说明
 * "读不到", 不区分"离线"还是"舵机处于过载/过温等告警态", 后者需用 E 命令确认。 */
static uint8_t servo_read_u8_retry(uint8_t bus, uint8_t id, uint16_t addr,
                                   uint8_t *out, uint8_t tries) {
    for (uint8_t t = 0; t < tries; t++) {
        if (DXL_Read(bus, id, addr, 1, out, 50) == HAL_OK) return 1;
        if (t + 1U < tries) HAL_Delay(3);
    }
    return 0;
}

/* ===== Burn one joint's cached PID into servo EEPROM (Lock-protected) =====
 * STS3215 PID 在 EEPROM (P=21, D=22, I=23)。初始化完成后 EEPROM 默认上锁
 * (Lock=1), 此时直接写地址会【静默丢弃】——这就是运行时 K/D/I/N 调参
 * 看起来完全无效的根因。写 EEPROM 必须: Torque OFF -> 解锁(Lock=0) ->
 * 写 P/D/I -> RESET(EEPROM 加载进控制器) -> 重新上锁(Lock=1)。
 * CAUTION: RESET 期间该关节失力 ~0.6s, 必须扶臂; 严禁在轨迹/多点运动中调用。
 * 返回 1 = 回读校验通过 (EEPROM == 请求值), 0 = 失败。 */
uint8_t SERVO_ApplyPidToEeprom(uint8_t joint, uint8_t p, uint8_t i, uint8_t d) {
    if (joint >= 6) return 0;
    uint8_t bus = JOINT_BUS[joint];
    uint8_t id  = JOINT_ID[joint];
    uint8_t byte, torque_was = 1;

    /* ⭐0924DA: 外层整轮重试 (2026-11-24 用户实测 Y 0 偶发 FAIL)。
     * 本总线丢读率高(实测 J1~J3 承重轴 25%+), 内层 3 次重试不够兜底;
     * 回读失败/不匹配时把 解锁->写->RESET->校验 整套重来, 最多 3 轮。 */
    for (uint8_t attempt = 0U; attempt < 3U; attempt++) {

    if (DXL_Ping(bus, id, 50) != HAL_OK) {
        if (attempt + 1U < 3U) { HAL_Delay(100); continue; }
        return 0;
    }

    /* 记住当前扭矩状态, 结束时恢复 (teach 等失力场景不该被强制上力) */
    torque_was = 1;
    (void)DXL_Read(bus, id, STS_ADDR_TORQUE_ENABLE, 1, &torque_was, 50);
    byte = 0;
    SERVO_WriteSimple(bus, id, STS_ADDR_TORQUE_ENABLE, &byte, 1);
    HAL_Delay(10);

    /* 解锁 EEPROM */
    byte = 0;
    SERVO_WriteSimple(bus, id, STS_ADDR_LOCK, &byte, 1);
    HAL_Delay(10);

    SERVO_WriteSimple(bus, id, STS_ADDR_P_GAIN, &p, 1); HAL_Delay(10);
    SERVO_WriteSimple(bus, id, STS_ADDR_D_GAIN, &d, 1); HAL_Delay(10);
    SERVO_WriteSimple(bus, id, STS_ADDR_I_GAIN, &i, 1); HAL_Delay(20);

    /* RESET: 把 EEPROM 新值加载进控制器 (失力窗口 ~0.6s) */
    (void)STS_Reset(bus, id);

    /* RESET 后锁回到 0, 重新上锁 (LOCK 只护 EEPROM 区, 不影响下面的 RAM 写) */
    byte = 1;
    SERVO_WriteSimple(bus, id, STS_ADDR_LOCK, &byte, 1);
    HAL_Delay(10);

    /* RESET 会清空 RAM 区 —— 恢复与 SERVO_InitJoint 相同的 RAM 配置
     * (加速度 / Goal Speed / 力矩上限), 否则该关节速度=0/无限制会跑飞 */
    uint8_t data[2];
    byte = sts_accel_value;
    SERVO_WriteSimple(bus, id, STS_ADDR_ACCELERATION, &byte, 1);
    HAL_Delay(5);
    data[0] = (uint8_t)(init_moving_speed & 0xFFU);
    data[1] = (uint8_t)(init_moving_speed >> 8);
    SERVO_WriteSimple(bus, id, STS_ADDR_GOAL_SPEED, data, 2);
    HAL_Delay(5);
    data[0] = 0xFF; data[1] = 0x0F;   /* Torque Limit = 4095 */
    SERVO_WriteSimple(bus, id, STS_ADDR_TORQUE_LIMIT, data, 2);
    HAL_Delay(5);

    /* 恢复扭矩 */
    byte = torque_was ? 1 : 0;
    SERVO_WriteSimple(bus, id, STS_ADDR_TORQUE_ENABLE, &byte, 1);
    HAL_Delay(5);

    /* 回读校验 (带重试: 偶发丢帧不判失败) */
    {
        uint8_t rp = 0, rd = 0, ri = 0;
        if (servo_read_u8_retry(bus, id, STS_ADDR_P_GAIN, &rp, 5) &&
            servo_read_u8_retry(bus, id, STS_ADDR_D_GAIN, &rd, 5) &&
            servo_read_u8_retry(bus, id, STS_ADDR_I_GAIN, &ri, 5) &&
            rp == p && rd == d && ri == i) {
            return 1;
        }
    }

    /* 本轮失败: 给舵机留喘息, 下一轮整套重来 */
    if (attempt + 1U < 3U) HAL_Delay(120);
    }
    return 0;
}

/* ===== Read PID gains straight from a servo's EEPROM (21/22/23) =====
 * 读不受锁限制, 用于核对缓存(K/D/I/N)与舵机实际运行值是否一致。
 * 返回 0 = 读取失败, 此时 *p/*d/*i 内容无意义(可能仍是调用者的初值),
 * 调用方【必须】检查返回值, 否则会把失败的 0 当成舵机真实值打印出来。 */
uint8_t SERVO_ReadPidEeprom(uint8_t joint, uint8_t *p, uint8_t *d, uint8_t *i) {
    if (joint >= 6 || !p || !d || !i) return 0;
    uint8_t bus = JOINT_BUS[joint];
    uint8_t id  = JOINT_ID[joint];
    if (!servo_read_u8_retry(bus, id, STS_ADDR_P_GAIN, p, 3)) return 0;
    if (!servo_read_u8_retry(bus, id, STS_ADDR_D_GAIN, d, 3)) return 0;
    if (!servo_read_u8_retry(bus, id, STS_ADDR_I_GAIN, i, 3)) return 0;
    return 1;
}

/* ===== Read one register byte from a joint's servo =====
 * 通用单字节读 (PUNCH 等调试/校验用)。返回 1=OK, 0=失败。
 * ⭐2026-11-05: 读失败立即重试最多 10 次(50ms/次), 成功才返回 —— 调用方都是
 * 一次性设置/查看(示教 PUNCH 备份、寄存器转储), 不会卡热路径。 */
uint8_t SERVO_ReadReg(uint8_t joint, uint16_t addr, uint8_t *out) {
    if (joint >= 6 || !out) return 0;
    uint8_t v = 0;
    for (uint8_t t = 0; t < 10U; t++) {
        if (DXL_Read(JOINT_BUS[joint], JOINT_ID[joint], addr, 1, &v, 50) == HAL_OK) {
            *out = v;
            return 1;
        }
    }
    return 0;
}

/* ===== 切换舵机工作模式 (EEPROM addr 33) =====
 * 2026-09-08 teach PWM 重力补偿: mode=2(PWM 开环力控) / 0(位置)。
 * 实测结论: Mode 写入后控制环【必须 RESET 重载】才真正切换 (免 RESET 只改
 * 寄存器值不切控制环, 臂依旧满刚度位置伺服)。流程: Torque OFF -> 解锁 ->
 * 写 Mode -> RESET -> 等 1.2s + 重试验证 -> 上锁 -> Torque ON。
 * 返回码: 1=OK, 2=ping 失败, 3=验证读取失败, 4=RESET 后模式仍不符。
 * CAUTION: RESET 期间该关节失力, 必须扶臂。 */
uint8_t SERVO_SetOperatingMode(uint8_t joint, uint8_t mode) {
    if (joint >= 6) return 2;
    uint8_t bus = JOINT_BUS[joint];
    uint8_t id  = JOINT_ID[joint];
    uint8_t byte, data[2], vm = 0xFF;

    /* ping 重试 3 次: 半双工方向切换/复位竞态下偶发单次失败
     * (2026-09-08 teach pwm J1/J5 ping 失败导致整体回退, 实为偶发) */
    uint8_t ping_ok = 0;
    for (uint8_t att = 0; att < 3; att++) {
        if (DXL_Ping(bus, id, 50) == HAL_OK) { ping_ok = 1; break; }
        HAL_Delay(30);
    }
    if (!ping_ok) return 2;

    byte = 0;
    SERVO_WriteSimple(bus, id, STS_ADDR_TORQUE_ENABLE, &byte, 1);
    HAL_Delay(10);
    byte = 0;
    SERVO_WriteSimple(bus, id, STS_ADDR_LOCK, &byte, 1);
    HAL_Delay(10);
    SERVO_WriteSimple(bus, id, STS_ADDR_MODE, &mode, 1);
    HAL_Delay(50);                 /* EEPROM 写入余量 */

    (void)STS_Reset(bus, id);      /* 控制环重载新模式 (内部已等 600ms) */
    HAL_Delay(600);                /* 额外启动等待 (共 ~1.2s) */

    for (uint8_t att = 0; att < 3; att++) {
        byte = 1;
        SERVO_WriteSimple(bus, id, STS_ADDR_LOCK, &byte, 1);
        HAL_Delay(10);
        if (DXL_Read(bus, id, STS_ADDR_MODE, 1, &vm, 50) == HAL_OK &&
            vm == mode)
            break;
        HAL_Delay(200);
    }
    if (vm != mode) return 4;

    if (mode == 0) {
        byte = sts_accel_value;
        SERVO_WriteSimple(bus, id, STS_ADDR_ACCELERATION, &byte, 1);
        HAL_Delay(5);
        data[0] = (uint8_t)(init_moving_speed & 0xFFU);
        data[1] = (uint8_t)(init_moving_speed >> 8);
        SERVO_WriteSimple(bus, id, STS_ADDR_GOAL_SPEED, data, 2);
        HAL_Delay(5);
        data[0] = 0xFF; data[1] = 0x0F;
        SERVO_WriteSimple(bus, id, STS_ADDR_TORQUE_LIMIT, data, 2);
        HAL_Delay(5);
    } else if (mode == 2) {
        /* 2026-09-08: Torque ON 前先把 duty 清零 —— 否则上电瞬间
         * addr44 RAM 残留值会让该轴突然自发力矩 (乱动来源之一) */
        data[0] = 0; data[1] = 0;
        SERVO_WriteSimple(bus, id, STS_ADDR_GOAL_TIME, data, 2);
        HAL_Delay(5);
    }

    byte = 1;
    SERVO_WriteSimple(bus, id, STS_ADDR_TORQUE_ENABLE, &byte, 1);
    HAL_Delay(5);
    return 1;
}

/* Simplified Write that doesn't fail on HAL_TIMEOUT (Status Return Level=1) */
HAL_StatusTypeDef SERVO_WriteSimple(uint8_t bus, uint8_t id,
                                    uint16_t addr, uint8_t* data,
                                    uint16_t len) {
    return DXL_Write(bus, id, addr, data, len, 100);
}

/* ===== PWM duty output (Mode 2 only) =====
 * 2026-09-08 修正: 官方 SDK SMS_STS::WritePwm 的占空比写【addr 44 GoalTime】,
 * 不是 addr 46 GoalSpeed (写 46 会被舵机当速度限制处理 = 之前 PWM 方案无效根因)。
 * 编码: bit10=方向位, bit0-9=幅度 0~1000 (与 Present Load 同格式)。
 * duty 范围 -1000..+1000, 0=无力矩输出(H 桥不动)。
 * 前提: 该舵机已通过 SERVO_SetOperatingMode(j, 2) 切到 PWM 模式。 */
HAL_StatusTypeDef SERVO_SetPwmDuty(uint8_t joint, int16_t duty) {
    if (joint >= 6) return HAL_ERROR;
    if (duty > 1000) duty = 1000;
    if (duty < -1000) duty = -1000;
    uint16_t v;
    if (duty < 0) {
        v = (uint16_t)(-duty);
        v |= 0x0400U;                 /* bit10 = negative direction */
    } else {
        v = (uint16_t)duty;
    }
    uint8_t data[2] = {(uint8_t)(v & 0xFFU), (uint8_t)(v >> 8)};
    return SERVO_WriteSimple(JOINT_BUS[joint], JOINT_ID[joint],
                             STS_ADDR_GOAL_TIME, data, 2);
}

/* ===== Legacy quick-init (minimal, for backward compat) ===== */
void SERVO_Init(void) {
    SERVO_InitAll();
}

/* ===== Full init ALL 6 joints ===== */
void SERVO_InitAll(void) {
    for (uint8_t i = 0; i < 6; i++) {
        SERVO_InitJoint(i);
    }
}

/* ===== Full servo-mode initialization for one STS3215 =====
   0) Ping to verify servo is present
   1) Torque OFF (required before EEPROM writes)
   2) Read Mode; if already 0 (servo mode) skip EEPROM section entirely
   3) [EEPROM] Unlock (Lock=0, addr 55) — EEPROM 默认锁定, 不解锁写入会静默失败!
   4) [EEPROM] Mode=0, Min/Max Angle, PID
   5) RESET to apply EEPROM changes, then re-lock EEPROM (Lock=1)
   6) Verify Mode == 0
   7) Set RAM: Acceleration, Goal Speed, Torque Limit
   8) Torque ON — hold current position (2026-09-01 修正, 见函数尾部注释)

   Mode 定义(飞特官方, 多个SDK交叉验证):
     Mode=0 位置伺服(舵机)模式, 出厂默认 ← 目标值
     Mode=1 速度闭环(恒速)模式 -> 写 GoalSpeed 会变成转速指令 -> 持续自转
     Mode=2 PWM 开环, Mode=3 步进模式

   自转根因: 旧固件把舵机写成了 Mode=1(恒速模式), 初始化写 GoalSpeed=200
   即开始持续旋转。必须写回 Mode=0。
*/
void SERVO_InitJoint(uint8_t joint) {
    if (joint >= 6) return;

    uint8_t bus = JOINT_BUS[joint];
    uint8_t id  = JOINT_ID[joint];
    uint8_t data[2];
    uint8_t byte;

    /* 0) Quick ping: skip if servo not on bus */
    if (DXL_Ping(bus, id, 50) != HAL_OK) {
        return;
    }

    /* 1) Torque OFF (required before EEPROM writes) */
    byte = 0;
    SERVO_WriteSimple(bus, id, STS_ADDR_TORQUE_ENABLE, &byte, 1);
    HAL_Delay(10);

    /* 2) Read current Mode: 0=舵机(位置)模式, 1=恒速模式(会自转) */
    uint8_t mode = 0xFF;
    if (DXL_Read(bus, id, STS_ADDR_MODE, 1, &mode, 50) != HAL_OK) {
        mode = 0xFF;   /* 读取失败: 保守起见走完整 EEPROM 配置流程 */
    }

    if (mode != 0) {
        /* 3) 解锁 EEPROM (Lock=0, addr 55) */
        byte = 0;
        SERVO_WriteSimple(bus, id, STS_ADDR_LOCK, &byte, 1);
        HAL_Delay(10);

        /* 4) Write Mode = 0 (位置伺服模式, EEPROM) */
        byte = 0;
        SERVO_WriteSimple(bus, id, STS_ADDR_MODE, &byte, 1);
        HAL_Delay(20);

        /* Min Angle Limit = 0 (EEPROM, 2 bytes) */
        data[0] = 0x00; data[1] = 0x00;
        SERVO_WriteSimple(bus, id, STS_ADDR_MIN_ANGLE, data, 2);
        HAL_Delay(20);

        /* Max Angle Limit = 4095 (EEPROM, 2 bytes) */
        data[0] = 0xFF; data[1] = 0x0F;  /* 4095 = 0x0FFF */
        SERVO_WriteSimple(bus, id, STS_ADDR_MAX_ANGLE, data, 2);
        HAL_Delay(20);

        /* PID gains (P=21, D=22, I=23 — D 在 I 前!) */
        byte = init_p_gain[joint];
        SERVO_WriteSimple(bus, id, STS_ADDR_P_GAIN, &byte, 1);
        HAL_Delay(10);

        byte = init_d_gain[joint];
        SERVO_WriteSimple(bus, id, STS_ADDR_D_GAIN, &byte, 1);
        HAL_Delay(10);

        byte = init_i_gain[joint];
        SERVO_WriteSimple(bus, id, STS_ADDR_I_GAIN, &byte, 1);
        HAL_Delay(20);

        /* 5) RESET to apply EEPROM changes */
        STS_Reset(bus, id);
        HAL_Delay(300);  /* extra settling time after reset */

        /* 重新上锁 EEPROM (RESET 后锁默认为 0) */
        byte = 1;
        SERVO_WriteSimple(bus, id, STS_ADDR_LOCK, &byte, 1);
        HAL_Delay(10);

        /* 6) Verify Mode applied */
        uint8_t verify = 0xFF;
        (void)DXL_Read(bus, id, STS_ADDR_MODE, 1, &verify, 50);
        (void)verify;   /* 校验结果由 SERVO_GetMode + 启动打印反馈 */
    }

    /* 7) Set RAM: Acceleration */
    byte = sts_accel_value;
    SERVO_WriteSimple(bus, id, STS_ADDR_ACCELERATION, &byte, 1);
    HAL_Delay(5);

    /* Set Goal Speed from active configuration */
    data[0] = (uint8_t)(init_moving_speed & 0xFFU);
    data[1] = (uint8_t)(init_moving_speed >> 8);
    SERVO_WriteSimple(bus, id, STS_ADDR_GOAL_SPEED, data, 2);
    HAL_Delay(5);

    /* Set Torque Limit = 4095 (maximum) */
    data[0] = 0xFF; data[1] = 0x0F;  /* 4095 = 0x0FFF */
    SERVO_WriteSimple(bus, id, STS_ADDR_TORQUE_LIMIT, data, 2);
    HAL_Delay(5);

    /* 8) 2026-09-01 触地事故修正: 初始化完成后立即恢复扭矩, 保持当前位置。
     * 旧固件这里保持 Torque OFF —— 运动中一旦舵机堵转拉低电源导致 STM32
     * 重启, 重启流程 SERVO_InitAll 把 6 颗舵机全部置为失力, 整臂在重力下
     * 砸向桌面 (J3 被砸到 9° 卡死, 见事故复盘)。上电即"抱闸"。
     * 校准模式 ('c') 入口会显式释放扭矩供手动摆位, 出口再恢复。 */
    uint8_t tq_on = 1;
    SERVO_WriteSimple(bus, id, STS_ADDR_TORQUE_ENABLE, &tq_on, 1);
    LED_TqTrack(joint, 1U);   /* 0924CN: InitJoint 结尾抱闸也算力矩 ON */
    HAL_Delay(5);
}

/* ===== Read operating mode (EEPROM addr 33) =====
 * 0 = 舵机(位置)模式, 1 = 恒速模式(自转!), 2/3 = PWM/步进, 0xFF = 读取失败/离线 */
uint8_t SERVO_GetMode(uint8_t joint) {
    if (joint >= 6) return 0xFF;
    uint8_t data = 0;
    if (DXL_Read(JOINT_BUS[joint], JOINT_ID[joint],
                 STS_ADDR_MODE, 1, &data, 50) != HAL_OK) {
        return 0xFF;
    }
    return data;
}

/* ===== Torque control ===== */
HAL_StatusTypeDef SERVO_SetTorque(uint8_t joint, uint8_t enable) {
    if (joint >= 6) return HAL_ERROR;
    LED_TqTrack(joint, enable ? 1U : 0U);   /* 0924CN: 力矩意图计数 */
    return DXL_Write(JOINT_BUS[joint], JOINT_ID[joint],
                     STS_ADDR_TORQUE_ENABLE, &enable, 1, 100);
}

/* ===== 读"力矩开关"状态 (addr 40) =====
 * 2026-09-17 新增, 给"轨迹一直在下发但某一轴纹丝不动"这类现场做判据:
 * 轨迹通道只写 GOAL_POSITION, **从不碰 Torque_Enable** —— 只要某轴被
 * `t J 0` 松过力(手摆姿态的标准做法), 或被旧版 `z`(结束时把六轴 Torque
 * 写 0)关过力, 那一轴就会安静地停在原地, 而所有命令都"返回成功"。
 * 返回: 1 = 力矩 ON / 0 = OFF / 0xFF = 读不到 */
uint8_t SERVO_GetTorqueEnabled(uint8_t joint) {
    if (joint >= 6) return 0xFF;
    uint8_t data = 0;
    if (DXL_Read(JOINT_BUS[joint], JOINT_ID[joint],
                 STS_ADDR_TORQUE_ENABLE, 1, &data, 50) != HAL_OK)
        return 0xFF;
    return data ? 1U : 0U;
}

/* ===== Set target position (angle in degrees) ===== */
HAL_StatusTypeDef SERVO_SetPosition(uint8_t joint, float angle_deg) {
    if (joint >= 6) return HAL_ERROR;
    uint16_t raw = AngleToRaw(angle_deg);
    uint8_t data[2] = { raw & 0xFF, (raw >> 8) & 0xFF };
    s_last_goal_tick = HAL_GetTick();   /* 0924CN: 运动活动戳 */
    return DXL_Write(JOINT_BUS[joint], JOINT_ID[joint],
                     STS_ADDR_GOAL_POSITION, data, 2, 100);
}

/* ===== Fast position write: fire-and-forget, no RX wait ===== */
void SERVO_SetPositionFast(uint8_t joint, float angle_deg) {
    if (joint >= 6) return;
    uint16_t raw = AngleToRaw(angle_deg);
    uint8_t data[2] = { raw & 0xFF, (raw >> 8) & 0xFF };
    s_last_goal_tick = HAL_GetTick();   /* 0924CN: 运动活动戳 */
    DXL_WriteNoReply(JOINT_BUS[joint], JOINT_ID[joint],
                     STS_ADDR_GOAL_POSITION, data, 2);
}

/* ===== 六轴 GoalPos 同步写 (按 JOINT_BUS[] 实际分组) =====
 * 旧实现硬编码 "Bus A: J1-J3(id 1-3) + Bus B: J4-J6(id 4-6)" 两条总线各发
 * 一帧。从臂上没问题, 但主臂(2026-09-11 起 6 颗舵机全部并联在 UART5 一条
 * 总线上, JOINT_BUS 全是 BUS_B)会把同一批目标也朝根本没接舵机的 Bus A
 * 发一帧 —— 若那块板的 PB10/PB11 仍搭在舵机 DATA 节点上, 这帧就和 Bus B
 * 的真帧叠在同一根线上, 直接搅成乱码。
 * 改法: 按 JOINT_BUS[] 分组, 只对"有轴的总线"各发一次 SyncWrite ——
 *   从臂 A:3 轴 + B:3 轴 (两帧, 与原来完全一致);
 *   主臂 B:6 轴 (一帧, 且不再朝空总线发)。
 * 关键不变量: 同一条总线上的同组轴仍由【一次】SyncWrite 下发 -> 同时起跑,
 * 机械不歪 (把 6 轴拆成逐轴写会破坏这一点, 故不可用逐轴写替代)。 */
static HAL_StatusTypeDef sync_write_goalpos_by_bus(const uint16_t raw[6]) {
    HAL_StatusTypeDef st_all = HAL_OK;
    s_last_goal_tick = HAL_GetTick();   /* 0924CN: 运动活动戳 */

    for (uint8_t bus = 0U; bus < 2U; bus++) {
        uint8_t ids[6];
        uint8_t data[12];
        uint8_t n = 0U;

        for (uint8_t j = 0U; j < 6U; j++) {
            if (JOINT_BUS[j] != bus) continue;
            ids[n] = JOINT_ID[j];
            data[2U * n]      = (uint8_t)(raw[j] & 0xFFU);
            data[2U * n + 1U] = (uint8_t)(raw[j] >> 8);
            n++;
        }
        if (n == 0U) continue;   /* 该总线没有舵机 -> 一帧都不发 */

        if (DXL_SyncWrite(bus, STS_ADDR_GOAL_POSITION, 2U, ids, data, n) != HAL_OK)
            st_all = HAL_ERROR;
    }
    return st_all;
}

/* ===== Six-axis batched goal-position write (角度输入) ===== */
HAL_StatusTypeDef SERVO_SetPositionsSync(const float angles[6]) {
    uint16_t raw[6];
    for (uint8_t j = 0U; j < 6U; j++) raw[j] = AngleToRaw(angles[j]);
    return sync_write_goalpos_by_bus(raw);
}

/* ===== Six-axis batched RAW goal-position write (teach replay) =====
 * 与 SERVO_SetPositionsSync 走同一条分组下发路径, 但直接收 raw 0..4095,
 * 供手控示教回放使用 —— 原样重发当初录到的舵机单位, 不做角度往返换算。 */
HAL_StatusTypeDef SERVO_SetPositionsRawSync(const uint16_t raw[6]) {
    return sync_write_goalpos_by_bus(raw);
}

/* ===== 六轴 GoalPos+GoalSpeed 同步写 (0924BP, fi 遥操比例跟随) =====
 * STS3215 RAM 42~47 连续: GoalPos(2B) + GoalTime(2B) + GoalSpeed(2B)。
 * 一次 SyncWrite(addr=42, len=6) 同时写"去哪 + 多快到" —— 流控帧可以
 * 把主臂速度直传从臂 (旧实现只写 GoalPos, 从臂永远以 i 初始化写入的
 * 固定 GoalSpeed(200 step/s) 匀速走向目标 ⇒ "匀速跟随, 主臂快慢传不过去")。
 * ⚠ STS3215 GoalSpeed=0 语义是"全速"(反向), 调用方必须 clamp 到 ≥1。 */
HAL_StatusTypeDef SERVO_SetPositionsRawSpeedSync(const uint16_t raw[6],
                                                 const uint16_t spd[6]) {
    HAL_StatusTypeDef st_all = HAL_OK;
    s_last_goal_tick = HAL_GetTick();   /* 0924CN: 运动活动戳 */
    for (uint8_t bus = 0U; bus < 2U; bus++) {
        uint8_t ids[6];
        uint8_t data[36];            /* 6 轴 × 6B */
        uint8_t n = 0U;
        for (uint8_t j = 0U; j < 6U; j++) {
            if (JOINT_BUS[j] != bus) continue;
            ids[n] = JOINT_ID[j];
            uint8_t* p = &data[6U * (uint8_t)n];
            p[0] = (uint8_t)(raw[j] & 0xFFU);
            p[1] = (uint8_t)(raw[j] >> 8);
            p[2] = 0U; p[3] = 0U;    /* GoalTime=0: 保持速度控制模式 */
            p[4] = (uint8_t)(spd[j] & 0xFFU);
            p[5] = (uint8_t)(spd[j] >> 8);
            n++;
        }
        if (n == 0U) continue;       /* 该总线无轴, 不发 (同上行注释) */
        if (DXL_SyncWrite(bus, STS_ADDR_GOAL_POSITION, 6U, ids, data, n) != HAL_OK)
            st_all = HAL_ERROR;
    }
    return st_all;
}

/* ===== Set goal speed ===== */
HAL_StatusTypeDef SERVO_SetSpeed(uint8_t joint, uint16_t speed) {
    if (joint >= 6) return HAL_ERROR;
    if (speed > 4095) speed = 4095;
    uint8_t data[2] = { speed & 0xFF, (speed >> 8) & 0xFF };
    return DXL_Write(JOINT_BUS[joint], JOINT_ID[joint],
                     STS_ADDR_GOAL_SPEED, data, 2, 100);
}

/* ===== Get position in degrees =====
 * ⭐2026-11-05 用户指令: 总线丢读实测 50~70% ⇒ "读失败立即重试(不睡眠),
 * 最多 10 次, 读成功才把值交给上层输出"。本函数是显示/决策读的公共入口
 * (p/pv/f/s/L/校准向导/存点/QL 到位判定), 代码里 6+ 处手写的
 * `if (cur<0) cur=SERVO_GetPosition(j) 重试一次` 自动升级为 10 连试。
 * 单次超时 100→30ms: 30ms 对 1M 应答(~0.5ms)仍有 60 倍裕量;
 * 总最坏 300ms(旧行为单次 100ms+上层手写重试≈200ms), 期望 ~45ms;
 * 60% 丢读下单读成功率 40% → 1-0.6^10 ≈ 99.4%。
 * ⚠ 遥测/示教热路径走 GetPositionRawTO(10ms)/ReadTelemetry(10ms) 不经过
 * 这里 —— 那里的设计原则是"可失败、绝不卡主循环", 不适合长重试。 */
float SERVO_GetPosition(uint8_t joint) {
    for (uint8_t t = 0; t < 10U; t++) {
        float v = SERVO_GetPositionDiagTO(joint, 30U, NULL, NULL);
        if (v >= 0.0f) return v;              /* 读成功才输出 */
    }
    return -1.0f;                             /* 10 连败: 上层照常显示 Err */
}

/* 读位置 + 失败原因 (2026-09-11)。
 * 现场意义: "FK feedback failed: Jx read error" 这句话掩盖了三种完全不同的
 * 故障 —— 只报 -1 时用户无法判断该查线缆还是查负载。此处把原因透出去。
 * ⭐2026-11-05: 同 SERVO_GetPosition —— 最多 10 连试, 成功才返回;
 * 10 连败时把最后一次的失败原因透出(不再是笼统 NO_REPLY)。 */
float SERVO_GetPositionDiag(uint8_t joint, uint8_t* diag, uint8_t* err_bits) {
    uint8_t dg = DXL_RD_OK, eb = 0U;
    float v = -1.0f;
    for (uint8_t t = 0; t < 10U; t++) {
        v = SERVO_GetPositionDiagTO(joint, 30U, &dg, &eb);
        if (v >= 0.0f) break;                 /* 读成功才输出 */
    }
    if (diag     != NULL) *diag     = dg;
    if (err_bits != NULL) *err_bits = eb;
    return v;
}

/* 带超时的"读位置 + 失败原因"。
 * 2026-09-17: 为 QL/多点到位判定拆出。该判定在【轨迹结束后的等待期】由主循环
 * 反复调用(见 main.c cmdMultiPointService), 而 DXL_ReadDiag 的超时是 100ms ——
 * 只要某轴连续读失败, 每趟就要烧 100ms×2(含重试), 主循环被拖住 ⇒ 控制台/
 * 遥测/E-STOP 一起停摆。热路径统一用 10ms(与 SERVO_GetPositionRawTO /
 * SERVO_ReadTelemetry 一致)。 */
float SERVO_GetPositionDiagTO(uint8_t joint, uint32_t to_ms,
                              uint8_t* diag, uint8_t* err_bits) {
    if (diag     != NULL) *diag     = DXL_RD_OK;
    if (err_bits != NULL) *err_bits = 0;
    if (joint >= 6) return -1.0f;
    uint8_t data[2] = {0, 0};
    HAL_StatusTypeDef st = DXL_ReadDiag(JOINT_BUS[joint], JOINT_ID[joint],
                                        STS_ADDR_PRESENT_POSITION, 2, data, to_ms,
                                        diag, err_bits);
    if (st != HAL_OK) return -1.0f;
    uint16_t raw = data[0] | (data[1] << 8);
    return RawToAngle(raw);
}

/* ===== Get raw position (0~4095) =====
 * 2026-09-16: 拆出带超时的版本给热路径用。
 * 背景: 示教录音每个 20ms 节拍要连读 6 轴, 而 DXL_Read 的超时是 100ms ——
 * 只要一轴读失败, 那一拍就要烧掉 100ms, 等于吞掉 5 个 50Hz 节拍
 * (节拍标志是 1 bit, 不会被补记) ⇒ 录音里出现"时间空洞",
 * 回放时那一段就成了一条直线抄近路。热路径改用 10ms (与
 * SERVO_ReadTelemetry 的快读一致), 单次失败只损失半个节拍。 */
uint16_t SERVO_GetPositionRawTO(uint8_t joint, uint32_t to_ms) {
    if (joint >= 6) return 0xFFFF;
    uint8_t data[2] = {0, 0};
    HAL_StatusTypeDef st = DXL_Read(JOINT_BUS[joint], JOINT_ID[joint],
                                     STS_ADDR_PRESENT_POSITION, 2, data, to_ms);
    if (st != HAL_OK) return 0xFFFF;
    return (uint16_t)(data[0] | (data[1] << 8));
}

uint16_t SERVO_GetPositionRaw(uint8_t joint) {
    /* ⭐2026-11-05: 同 SERVO_GetPosition —— 读失败立即重试最多 10 次, 成功才
     * 交给上层(校准向导实时显示/r J/存点 q 都用它); 单次超时 100→30ms。
     * 示教热路径走 GetPositionRawTO(TEACH_READ_TO_MS=10ms) 不受影响。 */
    for (uint8_t t = 0; t < 10U; t++) {
        uint16_t r = SERVO_GetPositionRawTO(joint, 30U);
        if (r != 0xFFFFU) return r;           /* 读成功才输出 */
    }
    return 0xFFFFU;
}

/* ===== Check if servo is moving (1=moving, 0=stopped) ===== */
uint8_t SERVO_IsMoving(uint8_t joint) {
    if (joint >= 6) return 2;  /* error */
    uint8_t data = 0;
    HAL_StatusTypeDef st = DXL_Read(JOINT_BUS[joint], JOINT_ID[joint],
                                     STS_ADDR_MOVING, 1, &data, 100);
    if (st != HAL_OK) return 2;
    return data;
}

/* ===== Get present load (direction + magnitude) ===== */
uint16_t SERVO_GetPresentLoad(uint8_t joint) {
    if (joint >= 6) return 0;
    uint8_t data[2] = {0, 0};
    HAL_StatusTypeDef st = DXL_Read(JOINT_BUS[joint], JOINT_ID[joint],
                                     STS_ADDR_PRESENT_LOAD, 2, data, 100);
    if (st != HAL_OK) return 0;
    return data[0] | (data[1] << 8);
}

/* ===== Present load, sign-magnitude decoded (核心: 堵转方向判断) =====
 * STS3215 Present Load(60) 是符号-幅度编码:
 *   低10位 = 负载幅值 (0 ~ 1023), bit10 = 方向 (1=CW, 0=CCW)
 * 返回 signed 值: +1..+1023 (CW), -1023..-1 (CCW), 0 = 无负载
 * 参考 LeRobot / st3215 / 官方样例 readLoad 的实现。 */
int16_t SERVO_GetPresentLoadSigned(uint8_t joint) {
    uint16_t raw = SERVO_GetPresentLoad(joint);
    if (raw == 0) return 0;
    int16_t mag = (int16_t)(raw & 0x3FF);   /* 低10位幅值 */
    if (raw & 0x400) mag = -mag;            /* bit10 = CW(取负) */
    return mag;
}

/* ===== 堵转检测 =====
 * 负载幅值超约 threshold 即判定为堵转/过载。
 * 幅值 = |signed|; 通常 300~500 视为重载, 800+ 视为堵转。 */
uint8_t SERVO_IsStalled(uint8_t joint, uint16_t threshold) {
    if (joint >= 6) return 2;               /* error */
    int16_t load = SERVO_GetPresentLoadSigned(joint);
    if (load == 0 && SERVO_GetHardwareError(joint) == 0) {
        /* 读取失败(load=0且无错误时无法区分) */
        return 2;
    }
    if (load < 0) load = -load;
    return (load >= (int16_t)threshold) ? 1U : 0U;
}

/* ===== Hardware error status (bitfield) =====
 * STS3215 Hardware Error(65): 各 bit 置位表示对应告警。 */
uint8_t SERVO_GetHardwareError(uint8_t joint) {
    if (joint >= 6) return 0;
    uint8_t data = 0;
    HAL_StatusTypeDef st = DXL_Read(JOINT_BUS[joint], JOINT_ID[joint],
                                     STS_ADDR_HW_ERROR, 1, &data, 100);
    if (st != HAL_OK) return 0xFF;          /* 读取失败标记 */
    return data;
}

/* ===== Hardware error bit -> readable string =====
 * 对照 LeRobot / 飞特错误码表:
 *   bit0 过压, bit1 欠压, bit2 过温, bit3 过载/堵转, bit4 传感器失效, bit5 电机失效
 * 为避免依赖 newlib stdio(sprintf), 用静态串拼接实现 (服务端逐个匹配)。 */
static void err_append(char* dst, uint8_t* pos, const char* tag) {
    while (*tag) dst[(*pos)++] = *tag++;
    dst[*pos] = 0;
}
const char* SERVO_ErrorString(uint8_t err) {
    static char buf[64];
    uint8_t i = 0;
    buf[0] = 0;
    if (err & 0x01) err_append(buf, &i, "over_volt ");
    if (err & 0x02) err_append(buf, &i, "under_volt ");
    if (err & 0x04) err_append(buf, &i, "over_temp ");
    if (err & 0x08) err_append(buf, &i, "overload ");
    if (err & 0x10) err_append(buf, &i, "sensor_fail ");
    if (err & 0x20) err_append(buf, &i, "motor_fail ");
    if (buf[0] == 0) err_append(buf, &i, "OK");
    return buf;
}


/* ===== Get temperature ===== */
uint8_t SERVO_GetTemperature(uint8_t joint) {
    if (joint >= 6) return 0;
    uint8_t data = 0;
    HAL_StatusTypeDef st = DXL_Read(JOINT_BUS[joint], JOINT_ID[joint],
                                     STS_ADDR_PRESENT_TEMP, 1, &data, 100);
    if (st != HAL_OK) return 0;
    return data;
}

/* ===== 整组运行状态读回 (2026-09-10) =====
 * 合并事务: 原来 pos/speed/load/voltage/temp/current 要 6 次单寄存器读,
 * 每次在旧接收路径下 ~5ms(静默判停) = 30ms/关节 —— 6 轴轮询一次 180ms,
 * 完全吃掉控制节拍。现在 56 起连续读 8 字节一次拿全 5 项, 电流(69)另一次,
 * 配合 DXL_Read 的帧长快路径, 整组 ~0.7ms。
 * 超时用 10ms(不是默认 50/100ms): 遥测是可失败的低优先级事务, 掉线舵机
 * 绝不能把主循环卡住 —— 那才会真的干扰运动。 */
uint8_t SERVO_ReadTelemetry(uint8_t joint, SERVO_Telemetry_t* t) {
    if (joint >= 6 || t == NULL) return 0;

    t->pos = 0; t->speed = 0; t->load = 0;
    t->voltage = 0; t->temp = 0; t->current = 0;
    t->base_valid = 0; t->cur_valid = 0;

    uint8_t bus = JOINT_BUS[joint];
    uint8_t id  = JOINT_ID[joint];
    uint8_t b[8];

    /* 一次连续读 56..63: Position(2) Speed(2) Load(2) Voltage(1) Temp(1) */
    if (DXL_Read(bus, id, STS_ADDR_PRESENT_POSITION, 8, b, 10) == HAL_OK) {
        t->pos     = (uint16_t)(b[0] | ((uint16_t)b[1] << 8));
        t->speed   = (int16_t)(b[2] | ((uint16_t)b[3] << 8));
        uint16_t ld = (uint16_t)(b[4] | ((uint16_t)b[5] << 8));
        int16_t  mag = (int16_t)(ld & 0x3FFU);          /* 低10位幅值 */
        if (ld & 0x400U) mag = (int16_t)(-mag);         /* bit10 = 方向 */
        t->load    = mag;
        t->voltage = b[6];
        t->temp    = b[7];
        t->base_valid = 1;
    } else {
        /* 兜底: 若该固件不支持长块读/偶发失败, 退化为逐寄存器读(慢但兼容)。
         * 位置是核心字段, 位置都读不到才判定本拍无效。 */
        uint8_t v[2];
        if (DXL_Read(bus, id, STS_ADDR_PRESENT_POSITION, 2, v, 10) != HAL_OK)
            return 0;
        t->pos = (uint16_t)(v[0] | ((uint16_t)v[1] << 8));
        if (DXL_Read(bus, id, STS_ADDR_PRESENT_SPEED, 2, v, 10) == HAL_OK)
            t->speed = (int16_t)(v[0] | ((uint16_t)v[1] << 8));
        if (DXL_Read(bus, id, STS_ADDR_PRESENT_LOAD, 2, v, 10) == HAL_OK) {
            uint16_t ld = (uint16_t)(v[0] | ((uint16_t)v[1] << 8));
            int16_t  mag = (int16_t)(ld & 0x3FFU);
            if (ld & 0x400U) mag = (int16_t)(-mag);
            t->load = mag;
        }
        uint8_t one;
        if (DXL_Read(bus, id, STS_ADDR_PRESENT_VOLTAGE, 1, &one, 10) == HAL_OK)
            t->voltage = one;
        if (DXL_Read(bus, id, STS_ADDR_PRESENT_TEMP, 1, &one, 10) == HAL_OK)
            t->temp = one;
        t->base_valid = 1;
    }

    /* 电流单独一次(69 起 2 字节); 失败不影响其余字段 */
    uint8_t c[2];
    if (DXL_Read(bus, id, STS_ADDR_PRESENT_CURRENT, 2, c, 10) == HAL_OK) {
        t->current   = (uint16_t)(c[0] | ((uint16_t)c[1] << 8));
        t->cur_valid = 1;
    }
    return t->base_valid;
}

/* ===== 写单轴目标 raw（非阻塞、保力矩）—— 供 `z J` / cmdGoHome() 用 =====
 * 2026-09-17 替换原 SERVO_Home()。原实现有四处错，逐条留档：
 *  1) **目标写死 STS_RAW_CENTER(2048)，与固件自己的"0 位"定义脱钩。**
 *     固件里"0 位" = SERVO_OFFSET[j]，由 calib_zero_raw[] 推出
 *     （kinematics.c:119/126: SERVO_OFFSET = raw/4095*360）。
 *     只有"没做过 `c` Phase1"或"零点恰好记在 2048"时两者才相等；
 *     一旦重新锚过零点（历史上出现过 2014/896/3106/2331/2022/1567），
 *     `z a` 推到的就不是 0 位 —— J2 可差 25° 以上。
 *     这也与已发布契约不符：sdk_package/03_INTERFACE 写的是
 *     "单轴 / 全部回中位（**用校准零点**）"，arm_config.json 的 `z` notes="用校准 zero"。
 *  2) 直接 SERVO_WriteSimple(GOAL_POSITION)，是全固件**唯一**绕过
 *     clamp_raw()/raw_in_range()/JOINT_MIN_RAW[] 的运动路径；且开头无条件
 *     上力矩（cmdTorque()/cmdMove() 都会先查 IsEStop()）。
 *  3) 结束把 Torque_Enable 写 0（注释"safe state"）⇒ 归位完六轴无力矩，
 *     重臂自己在重力下沉下去；正在 `#P` 跟随的从臂力矩状态被破坏。
 *  4) 每轴阻塞轮询 ≤15 s（`z a` 最坏 90 s 独占主循环：控制台/遥测/软件
 *     E-STOP 全停摆、流看门狗必报）。第 5 步所谓"恢复速度"写的还是 200
 *     （与第 2 步的慢速同值 = 等于没恢复，也不是 all_servo_moving_speed）。
 * 现在的分工：目标合法性（零点 + 限位）与全部打印留在 main.c 的
 * cmdGoHome()/cmdGoHomeAll()，这里只负责"把 raw 写下去"，不阻塞、不动力矩。
 * 六轴同时归位走 SERVO_SetPositionsRawSync()（同一总线一次 SyncWrite，
 * 保住"同时起跑、机械不歪"这条不变量），不要逐轴调用本函数。 */
HAL_StatusTypeDef SERVO_SetGoalRaw(uint8_t joint, uint16_t raw) {
    if (joint >= 6) return HAL_ERROR;
    uint8_t data[2] = { (uint8_t)(raw & 0xFF), (uint8_t)(raw >> 8) };
    return DXL_Write(JOINT_BUS[joint], JOINT_ID[joint],
                     STS_ADDR_GOAL_POSITION, data, 2, 100);
}
