/* ============================================================
 * arm_runlog.c - 固件运行日志（实现）
 *
 * 发送路径：hconsole (USART6) 直接 HAL_UART_Transmit。
 * 阻塞发送，禁止在 ISR / TIM2 回调 / 200Hz 插值路径调用。
 * ============================================================ */
#include "arm_runlog.h"
#include "arm_control_protocol.h"   /* APROTO_VERSION */
#include "arm_control_api.h"        /* ⭐0924DC: ArmApi_FwTag() — runlog 头带 fw-tag,
                                     * 根治 __DATE__ 陈旧误导 (源码没改的 .c 不重编,
                                     * 其 __DATE__ 停在旧日期, 版本判别以 tag 为准) */
#include "main.h"
#include <stdio.h>
#include <stdarg.h>

/* 控制台 UART 句柄（定义在 main.c） */
extern UART_HandleTypeDef hconsole;

static uint8_t s_level = RUNLOG_LVL_INFO;
static uint8_t s_run_id = 0;

static const char* const lvl_tag[] = {
    "NONE", "ERROR", "WARN ", "INFO ", "DEBUG"
};

/* 由构建时间生成稳定 run_id（同一固件镜像恒定） */
static uint8_t runlog_gen_id(void) {
    const char* s = __DATE__ __TIME__;
    uint32_t sum = 0;
    for (const char* p = s; *p; p++) sum = sum * 31u + (uint8_t)*p;
    return (uint8_t)(sum & 0xFF);
}

void RunLog_Init(void) {
    s_run_id = runlog_gen_id();
    char buf[128];
    int n = snprintf(buf, sizeof(buf),
                     "[RUN] RUN-START id=0x%02X tag=%s build=\"%s %s\" proto=%d level=%d\r\n",
                     s_run_id, ArmApi_FwTag(), __DATE__, __TIME__,
                     (int)APROTO_VERSION, (int)s_level);
    if (n > 0) {
        HAL_UART_Transmit(&hconsole, (uint8_t*)buf, (uint16_t)n, 100);
    }
}

uint8_t RunLog_RunId(void) { return s_run_id; }

void RunLog_SetLevel(uint8_t level) {
    if (level > RUNLOG_LVL_DEBUG) level = RUNLOG_LVL_DEBUG;
    s_level = level;
}

uint8_t RunLog_GetLevel(void) { return s_level; }

void RunLog_Msg(uint8_t level, const char* module, const char* fmt, ...) {
    if (level == RUNLOG_LVL_NONE || level > s_level) return;
    if (level > RUNLOG_LVL_DEBUG) level = RUNLOG_LVL_DEBUG;

    char buf[192];
    int n = snprintf(buf, sizeof(buf), "[RUN][%02X][T+%08lu][%s][%s] ",
                     s_run_id, (unsigned long)HAL_GetTick(),
                     lvl_tag[level], module ? module : "-");
    if (n < 0) return;
    if (n > (int)sizeof(buf) - 1) n = (int)sizeof(buf) - 1;  /* 防御: 头部截断 */

    va_list args;
    va_start(args, fmt);
    int m = vsnprintf(buf + n, sizeof(buf) - (size_t)n, fmt, args);
    va_end(args);
    if (m < 0) return;
    n += m;
    if (n > (int)sizeof(buf) - 3) n = (int)sizeof(buf) - 3;

    buf[n++] = '\r';
    buf[n++] = '\n';
    HAL_UART_Transmit(&hconsole, (uint8_t*)buf, (uint16_t)n, 100);
}
