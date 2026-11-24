/*
 * STS3215 控制表地址和关节级高层驱动接口。
 * STS3215 = 飞特(Feetech) 30KG 12V 串行总线舵机, SCS 协议。
 *
 * 寄存器地址来自 Feetech 官方文档 + 多方交叉验证(deepwiki/SO-101/feetech.js SDK)。
 * STS3215 关键特性:
 *   - 角度范围 0-4095 (0-360°), 中心=2048
 *   - PID 增益在 EEPROM (地址21/22/23), 需 Torque OFF 才能写
 *   - Mode 寄存器(33): 0=舵机/位置模式(出厂默认), 1=恒速模式(会自转)
 *   - 加速度寄存器(41): 控制运动加速度曲线
 *   - Goal Speed(46): 目标速度
 */
#ifndef __SERVO_DRIVER_H
#define __SERVO_DRIVER_H

#include <stdint.h>
#include "servo_bus_hal.h"
#include "scs_protocol.h"   /* DXL_RD_* 读失败原因码 (SERVO_GetPositionDiag 输出) */

/* ===== STS3215 Control Table Addresses (SCS Protocol) ===== */
/* Source: Feetech official docs + deepwiki/feetech + SO-101 project */

/* --- EEPROM area (需 Torque OFF 才能写, 写入后需 RESET 生效) --- */
#define STS_ADDR_ID                    5     /* 1B, RW */
#define STS_ADDR_BAUD_RATE             6     /* 1B, RW (0=1Mbps default) */
#define STS_ADDR_RETURN_DELAY          7     /* 1B, RW (unit: 2us) */
#define STS_ADDR_RETURN_LEVEL          8     /* 1B, RW (0=none,1=ping,2=all) */
#define STS_ADDR_MIN_ANGLE             9     /* 2B, RW */
#define STS_ADDR_MAX_ANGLE             11    /* 2B, RW (default 4095) */
#define STS_ADDR_MAX_TEMP              13    /* 1B, RW (default 85°C) */
#define STS_ADDR_MAX_VOLTAGE           14    /* 1B, RW (0.1V unit, default 160=16.0V) */
#define STS_ADDR_MIN_VOLTAGE           15    /* 1B, RW (0.1V unit, default 60=6.0V) */
#define STS_ADDR_MAX_TORQUE            16    /* 2B, RW (default 4095) */
#define STS_ADDR_P_GAIN                21    /* 1B, RW — 位置环 P 增益 (EEPROM!) */
#define STS_ADDR_D_GAIN                22    /* 1B, RW — 位置环 D 增益 (EEPROM! 注意D在I前) */
#define STS_ADDR_I_GAIN                23    /* 1B, RW — 位置环 I 增益 (EEPROM!) */
#define STS_ADDR_PUNCH                 24    /* 1B, RW */
#define STS_ADDR_MODE                  33    /* 1B, RW (0=servo/position, 1=speed/wheel, 2=PWM, 3=step) */

/* --- RAM area (随时可写) --- */
#define STS_ADDR_TORQUE_ENABLE         40    /* 1B, RW (0=OFF, 1=ON) */
#define STS_ADDR_ACCELERATION          41    /* 1B, RW (0-254, 加速度曲线) */
#define STS_ADDR_GOAL_POSITION         42    /* 2B, RW (0-4095, little-endian) */
#define STS_ADDR_GOAL_TIME             44    /* 2B, RW (ms, 时间控制模式用) */
#define STS_ADDR_GOAL_SPEED            46    /* 2B, RW (0=max, 速度限制) */
#define STS_ADDR_TORQUE_LIMIT          48    /* 2B, RW (0-4095) */
#define STS_ADDR_LOCK                  55    /* 1B, RW */

/* --- RAM feedback (只读) --- */
#define STS_ADDR_PRESENT_POSITION      56    /* 2B, R (0-4095) */
#define STS_ADDR_PRESENT_SPEED         58    /* 2B, R (signed) */
#define STS_ADDR_PRESENT_LOAD          60    /* 2B, R (signed) */
#define STS_ADDR_PRESENT_VOLTAGE       62    /* 1B, R (0.1V unit) */
#define STS_ADDR_PRESENT_TEMP          63    /* 1B, R (°C) */
#define STS_ADDR_HW_ERROR              65    /* 1B, R (bitfield) */
#define STS_ADDR_MOVING                66    /* 1B, R (0=stopped, 1=moving) */
#define STS_ADDR_PRESENT_CURRENT       69    /* 2B, R (signed; 约 6.5mA/LSB) */

/* Default values */
#define STS_DEFAULT_ID                  1
#define STS_DEFAULT_BAUD                0     /* 1 Mbps */
#define STS_DEFAULT_GOAL_SPEED          200   /* RAM register default */
#define STS_DEFAULT_ACCELERATION        10    /* 加速度默认值 */
#define STS_RAW_MAX                     4095  /* 12-bit 分辨率 */
#define STS_ANGLE_MAX                   360.0f  /* 0-360° */
#define STS_RAW_CENTER                  2048  /* 中心位置 */

/* 角度换算常量 */
#define STS_DEG_PER_RAW                 (360.0f / 4095.0f)
#define STS_RAW_PER_DEG                 (4095.0f / 360.0f)

extern const uint8_t JOINT_ID[6];
extern uint8_t JOINT_BUS[6];   /* runtime-configurable via SERVO_SetBusMode */

/* Bus mode (diagnostics): 0=normal(A:AABBB), 1=all Bus A, 2=all Bus B */
uint8_t        SERVO_SetBusMode(uint8_t mode);
uint8_t        SERVO_GetBusMode(void);

/* 逐 ID 自动路由 (0924BA): 按实测应答重填 JOINT_BUS[], 让舵机插任意座子都能用。
 * 返回在线关节数(0..6); miss/dup/moved 为位掩码(bit j=关节 j), 可传 NULL。
 * 必须在扫描之后、任何按关节的事务之前调用一次(上电流程已自动调用)。 */
uint8_t        SERVO_AutoMapBuses(uint8_t *miss, uint8_t *dup, uint8_t *moved);

/* 路由持久化 (0924BC, 扇区 11 区 G)。
 * Load: 上电时把上次实测确认的路由灌进 JOINT_BUS[]/s_map_conf[] —— 必须在
 *       AutoMapBuses 之前调用, 否则 pass1 还在用编译期默认 A/A/A/B/B/B。
 * Save: 写回"已被实测确认"的轴; 内容没变则不擦扇区。阻塞 ~1s, 只许启动期
 *       或控制台命令调。AutoMapBuses 结尾会自动调一次。
 * Clear: 抹掉记录, 下次上电回编译期默认布局。 */
uint8_t        SERVO_LoadSavedRoute(void);
uint8_t        SERVO_SaveRoute(void);
uint8_t        SERVO_SaveRouteAll(void);   /* `b 0/1/2`: 无条件把六轴全钉进 flash */
uint8_t        SERVO_ClearSavedRoute(void);

void            SERVO_Init(void);
void            SERVO_InitJoint(uint8_t joint);      /* Full servo-mode init + RESET */
void            SERVO_InitAll(void);                 /* Init all 6 joints */
void            SERVO_SetInitPID(const uint8_t p[6], const uint8_t i[6], const uint8_t d[6]);
void            SERVO_SetInitSpeed(uint16_t speed); /* Applied by SERVO_InitJoint */
/* PID 在舵机 EEPROM 且默认上锁: 裸写会被静默丢弃 (set_pid_gain 只改缓存!).
 * SERVO_ApplyPidToEeprom = 完整流程 (Torque OFF->解锁->写P/D/I->RESET->上锁->恢复扭矩),
 * 烧写期间该关节失力 ~0.5s, 必须扶臂、且不能在轨迹/多点运行中调用。
 * SERVO_ReadPidEeprom = 回读舵机 EEPROM 21/22/23 用于核对缓存是否真正生效。 */
uint8_t         SERVO_ApplyPidToEeprom(uint8_t joint, uint8_t p, uint8_t i, uint8_t d);
uint8_t         SERVO_ReadPidEeprom(uint8_t joint, uint8_t *p, uint8_t *d, uint8_t *i);
uint8_t         SERVO_ReadReg(uint8_t joint, uint16_t addr, uint8_t *out); /* 通用单字节读 */
uint8_t         SERVO_SetOperatingMode(uint8_t joint, uint8_t mode); /* 0=位置 2=PWM; 返回 1=OK 2=ping 4=RESET后仍不符 5=读失败 */
/* PWM 模式占空比输出 (Mode 2 only): duty -1000..+1000, 写 addr44+bit10 方向。
 * 官方 SDK WritePwm 编码; 写 addr46 是错的(会被当速度限制)。 */
HAL_StatusTypeDef SERVO_SetPwmDuty(uint8_t joint, int16_t duty);

/* 舵机内部加速度 (addr 41) 运行时生效点: 默认 50, SP 44 可调, 'i' 重新下发 */
extern uint8_t  sts_accel_value;
void            SERVO_SetAccel(uint8_t a);
/* 写单轴目标 raw（非阻塞、不碰力矩）。原 SERVO_Home() 已删除 —— 它把目标
 * 写死 2048、绕过限位、结束关力矩、每轴阻塞 15s。`z J` 见 main.c cmdGoHome()。 */
HAL_StatusTypeDef SERVO_SetGoalRaw(uint8_t joint, uint16_t raw);
HAL_StatusTypeDef SERVO_WriteSimple(uint8_t bus, uint8_t id, uint16_t addr,
                                     uint8_t* data, uint16_t len);
HAL_StatusTypeDef SERVO_SetTorque(uint8_t joint, uint8_t enable);
/* 读力矩开关: 1=ON / 0=OFF / 0xFF=读不到 (2026-09-17, 供"轴不动"现场判据) */
uint8_t         SERVO_GetTorqueEnabled(uint8_t joint);
HAL_StatusTypeDef SERVO_SetPosition(uint8_t joint, float angle_deg);
void            SERVO_SetPositionFast(uint8_t joint, float angle_deg); /* Fire-and-forget */
/* Batch-send all six goal positions: one Sync Write packet per physical bus. */
HAL_StatusTypeDef SERVO_SetPositionsSync(const float angles[6]);
/* Teach replay: sync-write raw 0..4095 samples verbatim (no deg conversion),
 * so recorded trajectories keep the exact servo units from teach time. */
HAL_StatusTypeDef SERVO_SetPositionsRawSync(const uint16_t raw[6]);
HAL_StatusTypeDef SERVO_SetPositionsRawSpeedSync(const uint16_t raw[6],
                                                 const uint16_t spd[6]); /* 0924BP */
HAL_StatusTypeDef SERVO_SetSpeed(uint8_t joint, uint16_t speed);

/* ===== LED 状态源 (0924CN): 驱动层挂钩的只读查询 ===== */
uint8_t  SERVO_TorqueOnCount(void);   /* 力矩 ON 意图轴数; 0 = 全部松力矩(软) */
uint32_t SERVO_LastGoalTick(void);    /* 最近一次目标位置下发的 HAL_GetTick; 0=从未 */
float           SERVO_GetPosition(uint8_t joint);
/* 读位置 + 失败原因 (2026-09-11): 返回角度, <0 = 失败。
 * diag 输出 DXL_RD_OK / DXL_RD_NO_REPLY / DXL_RD_BAD_FRAME / DXL_RD_ERROR_BITS,
 * err_bits 输出状态帧 Error 字节原值 (diag == DXL_RD_ERROR_BITS 时有意义)。
 * 用于把"读不到"区分为线缆问题 / 总线噪声 / 舵机告警三种现场处置完全不同的情况。 */
float           SERVO_GetPositionDiag(uint8_t joint, uint8_t* diag, uint8_t* err_bits);
/* 2026-09-17: 可指定超时版本 —— 与 RawTO 同一个理由。多点(QL)到位判定在
 * 运动结束后被主循环反复调用, 必须用短超时(10ms), 否则一轴连续读失败就把
 * 主循环按 100ms/轴 拖住, 控制台与 E-STOP 一起停摆。 */
float           SERVO_GetPositionDiagTO(uint8_t joint, uint32_t to_ms,
                                        uint8_t* diag, uint8_t* err_bits);
uint16_t        SERVO_GetPositionRaw(uint8_t joint); /* Raw 0~4095 */
/* 2026-09-16: 可指定超时版本 —— 示教录音等"每 20ms 连读 6 轴"的热路径必须用
 * 短超时 (建议 10ms), 否则一次总线抖动(100ms 超时)会吞掉 5 个 50Hz 节拍。 */
uint16_t        SERVO_GetPositionRawTO(uint8_t joint, uint32_t to_ms);
uint8_t         SERVO_IsMoving(uint8_t joint);
uint16_t        SERVO_GetPresentLoad(uint8_t joint);         /* Raw 16-bit (bit10=dir) */
int16_t         SERVO_GetPresentLoadSigned(uint8_t joint);   /* Signed -1024..+1023, 符号-幅度解码 */
uint8_t         SERVO_IsStalled(uint8_t joint, uint16_t threshold);
uint8_t         SERVO_GetHardwareError(uint8_t joint);       /* bitfield, 0=ok */
const char*     SERVO_ErrorString(uint8_t err);              /* 错误位 -> 可读字符串 */
uint8_t         SERVO_GetTemperature(uint8_t joint);
uint8_t        SERVO_GetMode(uint8_t joint);           /* 0=servo, 1=wheel(spin!), 0xFF=offline */
void            SERVO_RebootAll(void);   /* 广播 RESET 两条总线上的所有舵机 */

/* ===== 单次事务读回整组运行状态 (2026-09-10) =====
 * 半双工单总线"边发边收"的读侧原语: 把原来 5~6 次单寄存器读
 * (地址 56/58/60/62/63/69, 每次 ~5ms) 合并成 2 次多字节读
 * (56 起连续 8 字节 + 69 起 2 字节), 配合 DXL_Read 的帧长快路径,
 * 整组耗时约 0.7ms。上层按控制节拍轮转调用即可实时采样而不干扰运动。
 * 返回 1 = 基础组(位置/速度/负载/电压/温度)读成功。 */
typedef struct {
    uint16_t pos;        /* Present Position, raw 0~4095 */
    int16_t  speed;      /* Present Speed, 有符号 (step/s) */
    int16_t  load;       /* Present Load, 符号-幅度解码 (bit10=方向) */
    uint8_t  voltage;    /* Present Voltage, 0.1V/LSB (如 74 = 7.4V) */
    uint8_t  temp;       /* Present Temperature, °C */
    uint16_t current;    /* Present Current, raw (约 6.5mA/LSB) */
    uint8_t  base_valid; /* 1 = 上面 5 个基础字段有效 */
    uint8_t  cur_valid;  /* 1 = current 有效 (地址 69, 单独一次读) */
} SERVO_Telemetry_t;

uint8_t         SERVO_ReadTelemetry(uint8_t joint, SERVO_Telemetry_t* t);

#endif /* __SERVO_DRIVER_H */
