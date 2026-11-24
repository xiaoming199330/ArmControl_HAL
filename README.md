# ArmControl HAL —— 六轴机械臂 STM32F407 固件

> **版本：V5.1　|　最新更新日期：2026-11-24**

SO-ARM101 六轴机械臂主控固件。MCU 为 STM32F407ZG，驱动 Feetech STS3215 总线舵机，支持网口 UDP / CAN / 串口多链路下发、主从遥操、示教回放、RC 遥控、限位读取（`L` 表）与零点校准。

## 目录结构

| 目录/文件 | 说明 |
|---|---|
| `Core/` | 主程序与应用层：运动学（`kinematics.c`）、舵机驱动（`servo_driver.c`）、CAN/串口/网口协议（`arm_proto_*.c`、`arm_can_driver.c`、`arm_net.c`）、RC 遥控（`arm_rc*.c`）、示教存储（`teach_store.c`）、Flash 配置（`flash_cfg.c`）等 |
| `Drivers/` | STM32F4xx HAL 库与 CMSIS |
| `docs/` | 开发与调试文档（协议规范、联调步骤、排查诊断、调参底稿） |
| `tools/` | 上位机辅助脚本（三链路延迟基准、CAN host、配置生成） |
| `config/` | 参数配置 |
| `sdk_package/` | 上位机 SDK 资料与模型 |
| `机械臂代分析/` | 机械臂代数分析 |
| `ArmControl_HAL.ioc` / `Makefile` / `STM32F407ZGTx_FLASH.ld` / `startup_stm32f407xx.s` | CubeMX 工程与 arm-none-eabi-gcc 构建文件 |

## 构建

```bash
make            # 生成 elf/hex/bin 到 build/
make flash      # 编译并烧写
make clean
```

## 版本说明

- 当前版本线 `fw0924xx_pp`（pp = 网口+CAN 双链路预置），最新源码对应固件 `fw0924de`（2026-11-24）。
- 编译产物（`.elf/.bin/.map`）不入库，源码可随时用 Makefile 复现。
- 固件限位表实测（servo 度）：J1 65~300、J2 75~284、J3 76~271、J4 71~284、J5 12~351、J6 144~264。

## 关联仓库

- 上位机 ROS2 工作区：[GANJSONG/soarm-101-ros2](https://github.com/GANJSONG/soarm-101-ros2)（MoveIt2 规划联动、ACT 训练部署）
