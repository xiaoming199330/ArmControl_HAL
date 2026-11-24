/* ============================================================
 * arm_proto_can.h - 二进制控制协议的 CAN 承载
 *
 * 与串口共用同一协议核心（arm_control_protocol.c）：
 * 串口帧的 (SEQ,CMD,LEN,PAYLOAD,CRC16) 逻辑流在 CAN 上按
 * 8 字节数据场分包传输。
 *
 * CAN 2.0A 标准帧 ID 规划（11 位）:
 *   0x100 + node_id : 上位机 -> 机械臂（命令）
 *   0x180 + node_id : 机械臂 -> 上位机（响应 ACK/NACK/数据）
 *   0x280 + node_id : 机械臂 -> 上位机（周期状态帧/异步事件帧, 8 字节, 无分包）
 *   0x7E0           : 链路自检帧（不经协议层, 见 arm_can_driver.c）
 *   node_id: 1~15，由 `can node` 配置并掉电保存（arm_can_config.c）
 *
 * 【2026-09-14 主从臂复用】主从链路改走 CAN 时, 两个角色直接套用这套
 * "上位机 / 机械臂"寻址:
 *   主臂 = 上位机角色 -> 用 CanProto_SendCmd() 把 0x13 流帧发到 0x100+1
 *   从臂 = 机械臂角色 -> CanProto_Poll() 收 0x100+1, 回 NACK 到 0x180+1
 *   主臂用 CanProto_PeerFrames() 统计"从臂回了几帧" (= 旧 Master RX)
 * 两端 node_id 必须相同 (都 = 1), 否则收发 ID 对不上。
 *
 * 分包格式（逻辑流 = SEQ,CMD,LEN,PAYLOAD...,CRC_lo,CRC_hi）:
 *   首帧: d[0]=0x80|总帧数(1~15) d[1]=SEQ d[2]=CMD d[3]=LEN
 *         d[4..7]=PAYLOAD 前 4 字节（不足时继续放 CRC）
 *   续帧: d[0]=帧序号(1..14) d[1..7]=后续 PAYLOAD/CRC 字节
 *   单帧（PAYLOAD+CRC<=4，即 PAYLOAD<=2）仅首帧，总帧数=1。
 *   示例: MOVE_JOINTS 的逻辑流为 3+16+2=21B；首帧承载固定头
 *         3B 和后续流的 4B，剩余 14B 使用两个续帧，共 3 帧。
 *   ⚠ 分帧**没有帧间握手**: 一整包由同一函数连续发完, 中间不插别的帧。
 *     所以协议帧与 0x280 的状态/事件帧在总线上互不嵌套。
 *
 * 驱动解耦（当前工程 HAL CAN 模块未启用）:
 *   ARM_CAN_ENABLE = 0 (默认): 桩模式，Poll/Init 为空操作，
 *     协议文件可直接编译进工程，不依赖 stm32f4xx_hal_can。
 *   ARM_CAN_ENABLE = 1: 启用完整分包/重组逻辑，需在本工程实现
 *     下列驱动钩子的强定义（bxCAN 或 MCP2515 等）:
 *       ArmCanDrv_HwInit / ArmCanDrv_Tx / ArmCanDrv_Rx
 * ============================================================ */
#ifndef __ARM_PROTO_CAN_H
#define __ARM_PROTO_CAN_H

#include <stdint.h>

/* 编译开关：1=启用 CAN 承载（需提供驱动强定义），0=桩模式 */
#ifndef ARM_CAN_ENABLE
#define ARM_CAN_ENABLE  1
#endif

/* ---- 主从链路承载选择 (2026-09-14) ----
 *   0 = UART4 (PA0/PA1) 二进制帧流  ← 历史方案, 默认
 *   1 = CAN1 (差分总线)             ← `make LINK=can` 置 1
 * 两块板必须用同一个值。协议层 arm_control_protocol.c 完全不受影响。 */
#ifndef ARM_LINK_CAN
#define ARM_LINK_CAN  0
#endif

/* ---- CAN1 引脚选择 (2026-09-14) ----
 * ⚠ CubeMX 工程文件 ArmControl_HAL.ioc 里 CAN1 定的是 **PB8=CAN1_RX /
 *   PB9=CAN1_TX (AF9)**, 而初版 arm_can_driver.c 凭印象写成了 PA11/PA12 ——
 *   正是 UART4 那一轮"引脚凭记忆"踩过的同一个坑 (记忆里那条铁律:
 *   引脚一律以 .ioc 为准)。现在两种都支持, **默认按 .ioc 走 PB8/PB9**:
 *     make LINK=can               -> PB8/PB9   (build_can/)
 *     make LINK=can CANPIN=pa     -> PA11/PA12 (build_can_pa/)
 * 为什么留 PA11/PA12 这个选项: F407 上 PA11/PA12 同时是 USB_DM/USB_DP,
 * 有些板子把这两个脚直接接到 USB 座(有下拉/串阻) → 走 CAN 会打架;
 * 但如果你的板只把 PA11/PA12 引到排针、PB8/PB9 没引出, 就得用 CANPIN=pa。
 * 两组的复用功能号都是 AF9。 */
#ifdef ARM_CAN_PINS_PA
#define ARM_CAN_RX_PORT   GPIOA
#define ARM_CAN_RX_PIN    GPIO_PIN_11
#define ARM_CAN_TX_PORT   GPIOA
#define ARM_CAN_TX_PIN    GPIO_PIN_12
#define ARM_CAN_PIN_DESC  "PA11(RX)/PA12(TX)"
#define ARM_CAN_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#else
#define ARM_CAN_RX_PORT   GPIOB
#define ARM_CAN_RX_PIN    GPIO_PIN_8
#define ARM_CAN_TX_PORT   GPIOB
#define ARM_CAN_TX_PIN    GPIO_PIN_9
#define ARM_CAN_PIN_DESC  "PB8(RX)/PB9(TX)"
#define ARM_CAN_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()
#endif

/* ---- CAN2 引脚（固定, 2026-09-18）----
 * F407 上 CAN2 只能落在这两只脚上: **PB12=CAN2_RX / PB13=CAN2_TX (AF9)**。
 * ⚠ PB14 **没有** CAN 复用功能, 现场若按"CAN2=PB13/PB14"接线, 那一半一定是错的。
 * 这两只脚与本工程 CAN1 那两只(PB8/PB9)同属 GPIOB, 时钟也已开。 */
#define ARM_CAN2_RX_PORT   GPIOB
#define ARM_CAN2_RX_PIN    GPIO_PIN_12
#define ARM_CAN2_TX_PORT   GPIOB
#define ARM_CAN2_TX_PIN    GPIO_PIN_13
#define ARM_CAN2_PIN_DESC  "PB12(RX)/PB13(TX)"

/* ---- 运行时可选的 CAN 接口（2026-09-18）----
 * 为什么把接口从"编译期"改成"运行期":
 *   现场反复出现"帧发得出去、谁也收不到、RX 脚一动不动"。有一个很有力的假设
 *   —— **手里的 USB-CAN 其实挂在板子的另一个 CAN 口上**。F407 有两个 bxCAN
 *   实例(CAN1/CAN2), 各有一对脚, 症状完全相同。
 *   过去接口由 `CANPIN=pa/pb` 在编译期定死, 验一次假设就要拉 BOOT0 重烧一次,
 *   而且烧错版本还会把结果读成"这个口也不通"。
 *   现在做成 `cx 1|2` 运行时可切(+ `cx save` 落盘), **一次烧录就能两个口都试**。
 *
 * `CANPIN=pb/pa` 仍然决定 **CAN1 开机时**用哪一对脚; CAN2 固定 PB12/PB13。
 * 两组的复用功能号都是 AF9, 但 HAL 里是两个不同的宏(AF9_CAN1 / AF9_CAN2)。
 *
 * ⭐ 2026-09-18 现场追加: **CAN1 的那对脚本身也做成运行期可切**(`cx pa` / `cx pb`)。
 *   因为出了一次代价很大的误判 —— 板子的 CAN1 收发器实际接在 PA11(RX)/PA12(TX),
 *   固件却按 .ioc 驱动 PB8/PB9, 现象是"卡在拼命发、臂侧 MSR.RX 恒 1、收帧计数器
 *   全 0", **与"线没接/接错端子"完全一样**; 而改一次引脚假设就要拉 BOOT0 重烧。
 *   ⇒ 现在 `make ... CANPIN=pa` 只决定开机默认值, 之后 `cx pa|pb` 随时切。
 *   (不落盘: 复位后回到编译期默认 —— 不值得为这点便利去动 Flash 配置区布局。) */
#define ARM_CAN_IF_CAN1   1U    /* CAN1 + 当前选定的那对脚(pb/pa 运行期可变) */
#define ARM_CAN_IF_CAN2   2U    /* CAN2 + PB12(RX)/PB13(TX)  */

/* CAN1 的两组候选脚(运行期, 见上) */
#define ARM_CAN1_PINS_PB  0U    /* PB8(RX)/PB9(TX)  —— .ioc 默认 */
#define ARM_CAN1_PINS_PA  1U    /* PA11(RX)/PA12(TX) —— 现有样机的实际接法 */

/* ---- error-passive 下的"探路帧"周期（2026-09-18 0918H）----
 * TEC >= 128 时状态帧**不是永久停发**, 而是降为每 ARM_CAN_TEC_PAUSE_MS 放一帧出去试探
 * （理由见 arm_proto_can.c 的 CanProto_Service：TEC 只能靠"发送成功"递减，永久停发
 *  = 永久自锁）。0918G 是 5000 ms；**0918H 起改 500 ms**，把最坏恢复时间从 ~35 s
 * 压到 ~3.5 s —— 恢复延迟 ≈ (TEC−127) × 本周期，因为 TEC>127 时每成功发一帧只 −1。
 * 代价可忽略：无对端时总线负载从 0.2 帧/s 升到 2 帧/s；而且 error-passive 下
 * ACK 错误**不会**再让 TEC 上涨，所以多试几次不会把情况弄坏。
 * ⚠ 本值**只有这一个定义点** —— 不要在别处再抄一个数字，`main.c` 的提示文案也引用它。 */
#define ARM_CAN_TEC_PAUSE_MS   500U

/* CAN ID 基址（11 位标准帧） */
#define CAN_PROTO_ID_CMD_BASE   0x100   /* 上位机 -> 机械臂 */
#define CAN_PROTO_ID_RSP_BASE   0x180   /* 机械臂 -> 上位机 */
#define CAN_PROTO_ID_EVT_BASE   0x280   /* 机械臂 -> 上位机: 状态/事件, 无分包 */
#define CAN_PROTO_NODE_MAX      15
#define CAN_PROTO_NODE_MIN      1

/* ---- 0x280+node 段: 周期状态帧 / 异步事件帧（2026-09-17 新增） ----
 * d[0] 区分类型。为什么单独开一段而不复用 0x180:
 *   0x180 是"分包重组"的通道, 塞非协议帧进去会被重组逻辑当乱序帧丢掉,
 *   进而把正在重组的那一包也一起废掉。状态帧必须走干净的信道。
 * 主机拿到状态帧就不必轮询(每轮 GET_STATUS 要读 6 轴总线, 是重操作),
 * 也能立刻知道 E-Stop 与"轨迹跑完了没"。 */
#define ARM_CAN_EVT_TYPE_STATUS 0x53U   /* 'S': 周期状态帧 */
#define ARM_CAN_EVT_TYPE_EVENT  0x45U   /* 'E': 异步事件帧 */

/* 状态帧 flags (d[2]) */
#define ARM_CAN_FLAG_ESTOP       0x01U  /* E-Stop 激活 */
#define ARM_CAN_FLAG_TRAJ        0x02U  /* S 曲线轨迹执行中 */
#define ARM_CAN_FLAG_STREAM      0x04U  /* 主从直写流模式激活 */
#define ARM_CAN_FLAG_CORR        0x08U  /* 自适应修正进行中 */
#define ARM_CAN_FLAG_TEACH_REC   0x10U  /* 示教录制中 */
#define ARM_CAN_FLAG_TEACH_PLAY  0x20U  /* 示教回放中 */
#define ARM_CAN_FLAG_HW_OK       0x40U  /* CAN 控制器初始化成功 */
#define ARM_CAN_FLAG_PROTO_RDY   0x80U  /* 协议层就绪(固件恒 1) */

/* 异步事件码 (事件帧 d[2]) */
#define ARM_CAN_EVENT_ESTOP       1U    /* E-Stop 触发(d[3]=0) */
#define ARM_CAN_EVENT_TRAJ_DONE   2U    /* 轨迹结束(d[3]=1 正常/0 中止) */
#define ARM_CAN_EVENT_STREAM_EXIT 3U    /* 流模式退出(d[3]=看门狗超时) */
#define ARM_CAN_EVENT_TEACH_FULL  4U    /* 示教缓冲写满 */
#define ARM_CAN_EVENT_CFG_SAVED   5U    /* CAN 配置已落盘 */

/* 协议层收发统计（`can` 命令显示） */
typedef struct {
    uint32_t cmd_frames;      /* 收到的合法命令逻辑帧(CRC 通过并已执行) */
    uint32_t rsp_frames;      /* 收到的对端响应帧(0x180+node) */
    uint32_t tx_frames;       /* 驱动层成功入邮箱的 CAN 帧 */
    uint32_t rx_dropped;      /* 驱动层接收队列溢出丢帧 */
    uint32_t crc_err;         /* 重组完成但 CRC 错的逻辑帧 */
    uint32_t reasm_err;       /* 重组超时/乱序被丢弃 */
    uint32_t status_tx;       /* 已发出的周期状态帧数 */
    uint32_t event_tx;        /* 已发出的异步事件帧数 */
    uint8_t  last_err;        /* 最近一次命令的返回码(ARM_API_ERR_*) */
} CanProtoStats;

/* 初始化（node_id 1~15）。main() 中调用；桩模式下仅记录 ID。 */
void    CanProto_Init(uint8_t node_id);

/* 运行时改节点号（`can node N`）。只改本机认的 ID 段, 不做硬件重配。
 * ⚠ 改完必须让上位机也换到新 ID, 否则收不到响应。 */
void    CanProto_SetNodeId(uint8_t node_id);
uint8_t CanProto_NodeId(void);

/* 主循环轮询：收包 -> 重组 -> 分发 -> 回包。桩模式为空操作。 */
void    CanProto_Poll(void);

/* 主循环服务：发周期状态帧（arm_can_config 的 hb_ms 节拍）+ 把排队的事件
 * 帧发出去。与 Poll 分开是为了让"收"和"发"各自可以单独开关/计量。 */
void    CanProto_Service(void);

/* 主动上报一个异步事件（入队, 由 Service 发出; 队列满则丢最旧）。
 * code = ARM_CAN_EVENT_*, param = 事件附带的一个字节。 */
void    CanProto_SendEvent(uint8_t code, uint8_t param);

/* 读/清统计。 */
void    CanProto_GetStats(CanProtoStats* out);
void    CanProto_ResetStats(void);

/* 因事件队列满而被丢掉的事件数（诊断用，正常恒 0）。 */
uint32_t CanProto_EventDropped(void);

/* 是否已探测到对端（收到过发往本节点的帧）。
 * 0 ⇒ 周期状态帧被抑制中 —— 没对端时主动发帧会白刷 bxCAN 的 TEC，
 *     十几秒内就把控制器推到 error-passive / bus-off。 */
uint8_t CanProto_PeerSeen(void);

/* 主动组帧发送（响应与主动上报用）。返回 1=成功启动发送。 */
uint8_t CanProto_Send(uint8_t seq, uint8_t cmd,
                      const uint8_t* payload, uint8_t len);

/* 以"命令方向"组帧发送 (发到 0x100+node_id)。
 * 主臂转发 0x13 流帧给从臂时用它 —— CanProto_Send() 发的是 0x180+node_id
 * (响应方向), 主从链路两边都在同一个 node_id 上, 所以主臂必须用这个方向发,
 * 从臂才能在自己的接收 ID 段 (0x100+node_id) 收到。返回 1=成功启动发送。 */
uint8_t CanProto_SendCmd(uint8_t seq, uint8_t cmd,
                         const uint8_t* payload, uint8_t len);

/* 已收到的"对端方向"帧数 (0x180+node_id)。主臂用它显示"从臂回了几帧"。 */
uint32_t CanProto_PeerFrames(void);

/* CRC 通过并已交给 Aproto_Dispatch 的帧数 —— CAN 版的 LinkProto_StreamCount()。
 * 从臂这边看它是否以 ~30/s 增长, 就是"0x13 流帧真的收到了"的直接证据。 */
uint32_t CanProto_RxOkCount(void);

/* ---- 驱动钩子（weak 默认实现 = 未连接；用户工程可覆盖） ---- */

/* 硬件初始化（波特率/过滤器/中断）。返回 1=OK。 */
uint8_t ArmCanDrv_HwInit(void);

/* 发送一帧（标准 ID，len<=8）。返回 1=已入发送邮箱。 */
uint8_t ArmCanDrv_Tx(uint16_t can_id, const uint8_t* data, uint8_t len);
/* 等 3 个发送邮箱全部排空(已挂起帧真发完)。**多帧打包流每帧之间必须调用**,
 * 否则第 4 帧会抢走刚空出的 MB0 ⇒ 总线上帧序被打乱(见 can_send_packed 注释)。 */
uint8_t ArmCanDrv_TxDrain(uint32_t timeout_ms);

/* 非阻塞接收一帧。返回 1=取到帧，0=无帧。 */
uint8_t ArmCanDrv_Rx(uint16_t* can_id, uint8_t* data, uint8_t* len);

/* ---- 波特率（2026-09-17 运行时化） ----
 * 固定 14TQ(BS1=11/BS2=2, 采样点 85.7%), 只改预分频。
 * 支持 100/125/250/500/1000 kbit/s(42MHz PCLK1 下均可整除)。
 * SetBitrate 在 CAN 已初始化时会**停总线重配**并返回结果;
 * 启动流程是先 SetBitrate(配置值) 再 CanProto_Init()。 */
uint16_t ArmCanDrv_GetBitrate(void);
uint8_t  ArmCanDrv_SetBitrate(uint16_t kbps);

/* ---- 接口(控制器+引脚)运行时选择（2026-09-18, `cx` 命令用） ----
 * SetInterface 会把总线停掉、重配 GPIO 复用/时钟/NVIC/过滤器银行, 再以当前
 * 波特率重新上总线; 返回 1=切换完成且已重新初始化成功。可在 HwInit 之前调用,
 * 这样启动时就能按 Flash 里存的接口起来。 */
uint8_t     ArmCanDrv_SetInterface(uint8_t iface);   /* ARM_CAN_IF_CAN1 / CAN2 */
uint8_t     ArmCanDrv_GetInterface(void);

/* ---- CAN1 的引脚对运行时选择（2026-09-18, `cx pa|pb` 命令用） ----
 * 动机: 板子的 CAN1 收发器到底接在哪对脚上, 光看 .ioc 会骗人 —— 猜错时的现象
 * 与"线没接"一模一样。做成运行期后一次烧录就能两组都试。
 * 返回值 1 = 已生效(或当前不在 CAN1 上, 已记为"下次上总线时生效"), 0 = 失败。
 * ⚠ 不落盘: 复位后回到编译期 CANPIN 的默认值。 */
uint8_t     ArmCanDrv_SetCan1Pins(uint8_t which);    /* ARM_CAN1_PINS_PB / PA */
uint8_t     ArmCanDrv_GetCan1Pins(void);
/* 按参数给出引脚描述("PA11(RX)/PA12(TX)" / "PB8(RX)/PB9(TX)"), 静态字符串 */
const char* ArmCanDrv_Can1PinDesc(uint8_t which);

/* 当前接口的人类可读描述, 如 "PB8(RX)/PB9(TX) CAN1" (静态字符串, 可直接打印) */
const char* ArmCanDrv_PinDesc(void);
/* 当前接口的控制器短名: "CAN1" / "CAN2" */
const char* ArmCanDrv_IfName(void);

/* bxCAN 原始寄存器只读访问（`cn` 命令打印用）。
 * 为什么要包一层: 以前 main.c 直接写 `CAN1->MSR`, 接口可切换之后那样会读到
 * **另一个控制器**的寄存器, 是最容易骗过自己的那种错。 */
uint32_t ArmCanDrv_RegMCR(void);
uint32_t ArmCanDrv_RegMSR(void);
uint32_t ArmCanDrv_RegESR(void);
uint32_t ArmCanDrv_RegTSR(void);
/* 当前接口的 CAN_RX 引脚实时电平 (MSR.RX): 1=隐性/空闲, 0=显性/被拉低 */
uint8_t  ArmCanDrv_BusRxLevel(void);

/* 驱动层计数（`can` 命令显示） */
uint32_t ArmCanDrv_TxCount(void);       /* 成功入发送邮箱的帧数 */
uint32_t ArmCanDrv_RxDropCount(void);   /* 接收队列溢出丢帧数 */

/* ---- 链路自检（ct 命令用, 2026-09-14） ----
 * 一块板发 N 帧固定内容到 CAN_PROTO_ID_TEST, 另一块板数收到了几帧。
 * 这是 CAN 版的"帧计数证明": 比字节数可靠, 不会被浮空脚蒙混。
 * 发送是非阻塞的(丢帧无所谓, 看的是对端收数), 不会卡主循环。 */
#define ARM_CAN_TEST_ID        0x7E0U
#define ARM_CAN_TEST_MAGIC0    0xC5U
#define ARM_CAN_TEST_MAGIC1    0x3AU

/* 发 n 帧自检帧 (payload = {MAGIC0,MAGIC1,计数低,计数高,0,0,0,0})。
 * 返回实际成功入邮箱的帧数。 */
uint16_t ArmCanDrv_SendTestFrames(uint16_t n);
/* 已收到的自检帧数。 */
uint32_t ArmCanDrv_TestRxCount(void);
/* 自检帧发送计数。 */
uint32_t ArmCanDrv_TestTxCount(void);
/* bxCAN 错误状态: tec/re*_p 传 NULL 可忽略。返回 1=CAN 已初始化。 */
uint8_t  ArmCanDrv_ErrorState(uint8_t* tec, uint8_t* rec, uint8_t* lec);

/* ---- CAN 引脚探测 (cp 命令用, 2026-09-14) ----
 * 为什么需要它: UART4 那一轮最大的时间黑洞是"分不清是引脚没接还是线不对"。
 * CAN 这边同样会踩 —— 本命令把 CAN 引脚临时切成普通 GPIO 直接量电平:
 *   rx_pd : RX 脚(默认 PB8) 输入 + 内部下拉, 连采 8 次位图
 *   rx_pu : RX 脚 输入 + 内部上拉, 连采 8 次位图
 *   tx_lo : TX 脚(默认 PB9) 推挽输出低后读自己, 8 次位图
 *   tx_hi : TX 脚 推挽输出高后读自己, 8 次位图
 * 判据(收发器已上电且接线正确时): 收发器的 RXD 是**推挽输出且空闲为高**,
 * 所以 rx_pd 会读到 11111111(内部 40k 下拉拉不动它) ⇒ "外面真的有驱动"。
 * 什么都没接时 rx_pd = 00000000 ⇒ 内部下拉轻松赢。探完自动恢复 CAN 正常模式。
 * 返回 1 = 探测完成且 CAN 已恢复。 */
uint8_t ArmCanDrv_PinProbe(uint8_t* rx_pd, uint8_t* rx_pu,
                           uint8_t* tx_lo, uint8_t* tx_hi);

/* ---- cw 外环测试 (2026-09-17) ----
 * cp 的盲区: 它读的是**引脚自己**(RX 有没有被外部驱高 / TX 能不能推挽),
 * 从没验过 "TX -> 收发器 TXD -> 收发器驱动总线 -> 收发器 RXD -> RX"
 * 这条**外环**。TXD 那根线断了 cp 照样全绿, 症状却是"帧发出去了没人收到"。
 *
 * cw 停掉 bxCAN, 把 TX 脚当普通 GPIO 持续驱动显性(低), 直接读 RX:
 *   TX 显性 -> 收发器拉低 CANL/抬高 CANH -> 总线显性 -> RXD 输出低 -> RX 读到低
 * 标准 CAN 收发器在正常模式下就会这样回馈总线状态 ⇒ **单节点自环即可判定**,
 * 不需要万用表, 也不需要总线上有第二个节点。
 *
 * dom/rec = TX 驱动显性/隐性时 RX 脚的 8 次采样位图。
 *   dom=0x00 且 rec=0xFF  ⇒ 外环通
 *   dom=0xFF (仍是隐性高) ⇒ 外环断: TXD 没接 / 模块丝印反 / 模块没驱动 / 器件坏
 *   dom=0x00 且 rec=0x00  ⇒ 总线被外部卡死在显性(短路/对端强驱动)
 * 返回 1 = 探测完成且 CAN 已恢复。 */
uint8_t ArmCanDrv_WireLoopTest(uint8_t* dom, uint8_t* rec);

/* 持续驱动总线显性 ms 毫秒后恢复(CAN 自动重配)。
 * 万用表刷新率抓不到 bxCAN 的间歇发送, 但抓得到"按住不放"的显性:
 * 卡端量到 CANH≈3.5V/CANL≈1.5V = 两根线都通; 仍 ≈2.5V/2.5V = 没接到这对端子上。
 * 返回 1 = 完成且 CAN 已恢复。 */
uint8_t ArmCanDrv_DriveDominant(uint16_t ms);

/* ---- ps 引脚嗅探 (2026-09-18) ----
 * cp / cw 都只认"当前这一组 CAN 脚"(默认 PB8/PB9)。它们回答不了这个更前
 * 面的问题: **板子上是不是还有第二个 CAN 口? 手里的 USB-CAN 到底挂在哪只脚上?**
 * 症状上两者完全一样 —— 帧发得出去、谁也收不到、RX 脚一动不动。
 *
 * ps 把**所有**与 CAN 有关的候选脚逐个切成 GPIO 输入, 在几百微秒内连采
 * ARM_CAN_SCAN_SAMPLES 次, 统计"低电平次数"与"跳变次数", 再用内部下拉
 * 复采一遍区分"悬空"和"被外部驱动"。
 *
 * ⚠ **必须在卡那一端持续发送/灌帧的同时跑本命令**, 否则什么都没有输入源,
 *   悬空脚一律读成"没东西" —— 那样只能证明"没在发", 不能证明"接错脚"。
 *
 * 判读(灌帧期间):
 *   low≈0   且 low_pd≈0            -> 被外部恒定驱高(悬空脚会被内部下拉拉低)
 *   edge≥8                         -> ★有活动信号 ⇒ 对端的 CAN 就在这只脚上
 *   low_pd≈full 且 edge=0          -> 悬空/无外部驱动
 *   其它                           -> 被外部恒定驱低
 * 典型结论: 灌帧时 PB12 出现 ★ 而 PB8 没有 ⇒ 卡接的是板子的 **CAN2 口**
 * (F407: CAN2_RX=PB12 / CAN2_TX=PB13, AF9), 而不是固件在用的 CAN1(PB8/PB9)。 */
#define ARM_CAN_SCAN_SAMPLES  4096U   /* 每只脚每次采样的点数 */

typedef struct {
    uint16_t low;      /* 无上拉时低电平次数 */
    uint16_t edge;     /* 无上拉时电平跳变次数(有信号就会跳) */
    uint16_t low_pd;   /* 内部下拉后低电平次数(悬空脚会被拉满) */
    uint16_t total;    /* 采样总次数 */
} ArmCanPinStat_t;

uint8_t     ArmCanDrv_PinScanCount(void);
const char* ArmCanDrv_PinScanName(uint8_t idx);
/* 一次扫完所有候选脚。st 至少 ARM_CAN_PINSCAN_N 个元素。
 * 返回 1 = 扫描完成且 CAN1 已恢复。 */
uint8_t     ArmCanDrv_PinScan(ArmCanPinStat_t* st);
#define ARM_CAN_PINSCAN_N  11

#endif /* __ARM_PROTO_CAN_H */
