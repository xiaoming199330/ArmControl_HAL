/* ============================================================
 * arm_runlog.h - 固件运行日志（结构化、分级、带运行会话 ID）
 *
 * 目标：代码运行调试时，每一次运行测试都有可定位故障的日志。
 *
 * 固件端（本模块）：
 *   - 每次上电输出 RUN-START 头（run_id + 构建时间 + 协议版本）
 *   - 分级 ERROR/WARN/INFO/DEBUG，模块标签，毫秒时间戳
 *   - 输出格式（单行，便于 PC 工具解析/检索）:
 *       [RUN][id][T+00012345][INFO ][PROTO] message
 *
 * PC 端（tools/arm_run_logger.py）：
 *   - 每一次运行采集会话自动保存为独立文件
 *     logs/run_YYYYMMDD_HHMMSS.log，结束输出故障摘要。
 *
 * 纪律（继承工程实时守则）:
 *   - 禁止在 ISR / TIM2 回调 / 200Hz 插值路径调用（阻塞 TX）
 *   - 仅事件级日志：命令、运动起止、IK 失败、修正收敛/超时、
 *     参数修改、E-Stop、协议帧异常
 * ============================================================ */
#ifndef __ARM_RUNLOG_H
#define __ARM_RUNLOG_H

#include <stdint.h>

enum {
    RUNLOG_LVL_NONE  = 0,
    RUNLOG_LVL_ERROR = 1,
    RUNLOG_LVL_WARN  = 2,
    RUNLOG_LVL_INFO  = 3,
    RUNLOG_LVL_DEBUG = 4
};

/* 初始化并输出 RUN-START 头。main() 中 Console_Init 之后调用。 */
void    RunLog_Init(void);

/* 运行会话 ID（本次固件构建标识，PING 协议应答携带） */
uint8_t RunLog_RunId(void);

/* 运行时日志级别（协议 CMD_LOG_SET_LEVEL / 文本 LL 命令可调） */
void    RunLog_SetLevel(uint8_t level);
uint8_t RunLog_GetLevel(void);

/* 分级输出（主循环上下文专用，禁 ISR/200Hz 路径） */
void    RunLog_Msg(uint8_t level, const char* module, const char* fmt, ...);

#define RLOG_E(mod, ...)  RunLog_Msg(RUNLOG_LVL_ERROR, mod, __VA_ARGS__)
#define RLOG_W(mod, ...)  RunLog_Msg(RUNLOG_LVL_WARN,  mod, __VA_ARGS__)
#define RLOG_I(mod, ...)  RunLog_Msg(RUNLOG_LVL_INFO,  mod, __VA_ARGS__)
#define RLOG_D(mod, ...)  RunLog_Msg(RUNLOG_LVL_DEBUG, mod, __VA_ARGS__)

#endif /* __ARM_RUNLOG_H */
