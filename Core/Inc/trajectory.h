/* 六轴同步梯形速度轨迹的公共接口。 */
#ifndef __TRAJECTORY_H
#define __TRAJECTORY_H

#include <stdint.h>

void Trajectory_Plan(float start[6], float end[6], float v_max, float a);
void Trajectory_Interpolate(uint32_t tick, float out_angles[6]);
void Trajectory_Target(float out[6]);   /* 导出轨迹终点目标 (尾段交接用) */
uint8_t Trajectory_IsActive(void);
void Trajectory_Stop(void);
uint32_t Trajectory_TotalTicks(void);   /* 当前轨迹总插值步 */

/* 运行时可调 S 曲线 jerk 过渡时间 s（默认 0.15，范围 0.02~0.50）。
 * arm_params 指针绑定生效点；值越大越柔和，越小越接近梯形。 */
extern float TRAJ_SCURVE_TJ;

#endif
