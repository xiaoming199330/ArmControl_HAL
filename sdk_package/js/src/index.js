/**
 * index.js — arm-sdk-js 统一出口
 *
 * 分层（和 Python 版 arm_sdk/ 一一对应）：
 *
 *   protocol.js    逻辑包编解码（三条链路逐字节一致）  ← 唯一的真值源
 *   transport.js   四种承载：UDP / WebSocket 桥 / Web Serial / 离线模拟器
 *   client.js      ArmClient：SEQ 配对 + 超时重发 + 响应归一化
 *   teleop.js      主从臂遥操
 *   teach.js       遥操录制 / 轨迹 / 回放
 *   kinematics.js  FK / IK / 限位（与固件逐位一致）
 *   console.js     串口**文本**控制台（h / s / f / m … 人机调试用）
 *   sim.js         内存假臂（离线开发 / CI）
 *
 * 最常用入口：
 *
 *   import { openArm } from 'arm-sdk-js';
 *   const arm = await openArm('net://192.168.7.120:5000');
 *   console.log((await arm.ping()));
 *
 * ⚠ 浏览器里没有 UDP。网页要用网口，必须先跑 `node src/bridge.js` 边车，
 *   然后 `openArm('ws://127.0.0.1:8765')`。
 */

export * from './protocol.js';
export * from './transport.js';
export * from './client.js';
export * from './teleop.js';
export * from './teach.js';
export * from './kinematics.js';
export * from './sim.js';

// ⚠ console.js 里有个 ArmSDK 类，和 kinematics 无重名，可以整体导出。
//   但它依赖 Web Serial，Node 下加载无害（只在 new 的时候才碰 navigator）。
export {
  ArmSDK as ConsoleClient,
  classify, splitDeviceText, parseTmLine, parseBinStatus,
  parseStatus, parseJoint, parseFk, parseLimits, parseParams,
  parsePidCompare, parseClampLines, parseLimitsSet,
} from './console.js';

import * as kinematics from './kinematics.js';
import { SimTransport } from './sim.js';

// 把 kinematics 注入模拟器，这样 sim 的 FK 也返回真值（而不是 0）
SimTransport.installKinematics(kinematics);

export const VERSION = '2.0.0';

/** 能力摘要（对应 Python 的 `armctl info`）。 */
export function info() {
  return {
    name: 'arm-sdk-js',
    version: VERSION,
    transports: [
      { uri: 'net://<ip>:5000', carrier: 'UDP', p50ms: 1.61, where: 'Node/Electron 主进程', star: true },
      { uri: 'ws://<host>:8765', carrier: 'WebSocket 桥', p50ms: 1.61, where: '浏览器/渲染进程' },
      { uri: 'webserial://', carrier: 'Web Serial', p50ms: 4.78, where: '浏览器/渲染进程' },
      { uri: 'sim://', carrier: '内存', p50ms: 0, where: '任意' },
    ],
    binaryCommands: 14,
    runtimeParams: 24,
    maxPacketBytes: 69,
    notes: [
      '逻辑包三链路完全一致：SEQ|CMD|LEN|PAYLOAD|CRC16-LE',
      'UDP 无帧头；串口加 AA 55 前缀；CAN 8 字节分包',
      '浏览器做不到原始 UDP，必须走 bridge.js 边车',
    ],
  };
}
