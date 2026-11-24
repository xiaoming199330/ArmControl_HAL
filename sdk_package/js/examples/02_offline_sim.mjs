/**
 * 02_offline_sim.mjs — 离线假臂：把固件那几个"阴间行为"复现给你看
 *
 *     node examples/02_offline_sim.mjs
 *
 * 不需要任何硬件。SimTransport 把下列行为逐个复刻（和固件同款）：
 *   ① 静默钳制   MOVE_JOINTS 超限 → 回 err=0，但值被钳过（必须回读才知真相）
 *   ② 读失败哨兵 某轴丢读 → 状态里是 null（-32768），**不兜底**
 *   ③ 流自锁     STREAM_JOINTS 单帧单轴跳变 > 25° → 整帧拒收且基线**不推进**
 *   ④ CRC 错     → NACK err=2
 */

import { ArmClient } from '../src/client.js';
import { SimTransport } from '../src/sim.js';
import { CMD, JointCalib } from '../src/protocol.js';
import { fmtQ } from './_args.mjs';

/* ① 静默钳制 -------------------------------------------------------------- */
{
  // 复刻固件**运行时限位**（关节度）：
  //   J1 63~303 / J2 75~284 / J3 76~271 / J4 71~284 / J5 12~351 / J6 138~266
  // sim 内部按 raw(0~4095) 钳制，这里把角度折成 raw 传进去。
  const toRaw = (d) => Math.round((d / 360) * 4095);
  const pairs = [[63, 303], [75, 284], [76, 271], [71, 284], [12, 351], [138, 266]];

  const arm = new ArmClient(new SimTransport({
    minRaw: pairs.map((p) => toRaw(p[0])),
    maxRaw: pairs.map((p) => toRaw(p[1])),
  }));
  await arm.open();

  console.log('=== ① 静默钳制：J1 发 350（运行时限位 63~303）===');
  const r = await arm.moveJoints([350, 180, 180, 180, 180, 180], 250);
  console.log('  固件回：' + r.toString() + `   err=${r.err}（0 = "成功"，骗你的）`);
  await new Promise((res) => setTimeout(res, 900));      // 等轨迹走完
  const st = await arm.getStatus();
  console.log('  回读真实角度：' + fmtQ(st.joints) + '   ← J1 停在 ~303，不是 350');
  console.log('  ⚠ 结论：**永远不要信 err**，要回读角度对比');
  await arm.close();
}

/* ② 读失败哨兵 ------------------------------------------------------------ */
{
  const arm = new ArmClient(new SimTransport({ failJoints: [2] }));   // J3 丢读
  await arm.open();
  console.log('\n=== ② 读失败哨兵：J3 丢读 ===');
  const st = await arm.getStatus();
  console.log('  joints = ' + fmtQ(st.joints) + '   ← J3 是 null，不是 0');
  console.log('  read_ok = ' + st.read_ok + '   failedJoints = J' + st.failedJoints.join(',J'));
  console.log('  ⚠ SDK 不兜底：绝不用 0 或上次值冒充，防止拿假姿态做逆解');
  await arm.close();
}

/* ③ 流自锁（25° 门限 + 基线不推进）---------------------------------------- */
{
  const arm = new ArmClient(new SimTransport());
  const calib = new JointCalib();
  await arm.open();
  console.log('\n=== ③ STREAM_JOINTS 单帧跳变 > 25° 被整帧拒收 ===');

  // 从 Home（逻辑 0）开始
  const ok1 = await arm.streamJoints([0, 0, 0, 0, 0, 0], true, { timeout: 0.5 });
  console.log('  发逻辑 [0,0,...]（无跳变）：' + ok1.toString());

  // 一帧跳 40°（> 25°）→ 拒收，且基线**不推进**
  const bad = await arm.streamJoints([0, 40, 0, 0, 0, 0], true, { timeout: 0.5 });
  console.log('  发逻辑 [0,40,...]（跳 40°）：' + bad.toString() +
              `   err=${bad.err}（4=LIMIT）`);
  console.log('  → 基线仍停在 0：下一帧必须从 ≤25° 起小步走，否则永远被拒（自锁）');

  // 小步 10° → 通过（基线推进到 10）
  const ok2 = await arm.streamJoints([0, 10, 0, 0, 0, 0], true, { timeout: 0.5 });
  console.log('  发逻辑 [0,10,...]（跳 10°）：' + ok2.toString());
  await arm.close();
  void calib;
}

/* ④ CRC 错 → NACK err=2 --------------------------------------------------- */
{
  const arm = new ArmClient(new SimTransport());
  await arm.open();
  console.log('\n=== ④ CRC 错 → NACK err=2 ===');
  const bad = arm.transport;
  // 直接塞一个坏包（改 CRC 低字节）
  const { packLogical } = await import('../src/protocol.js');
  const pkt = packLogical(0x77, CMD.PING);
  pkt[pkt.length - 1] ^= 0xFF;
  bad._rx.push({
    kind: 'packet', seq: 0x77, cmd: CMD.NACK,
    payload: Uint8Array.from([CMD.PING, 2]),
  });
  await new Promise((r) => setTimeout(r, 30));
  const acks = arm.pollStreamAcks();
  console.log('  注入 NACK 后 pollStreamAcks() = ' + JSON.stringify(
    acks.map((a) => ({ cmd: a.cmd, payload: Array.from(a.payload) }))));
  console.log('  payload[1]=2 → ErrCode.CRC');
  await arm.close();
}

console.log('\n全部演示完。真实固件的同款行为见 docs/网口协议速查.md。');
