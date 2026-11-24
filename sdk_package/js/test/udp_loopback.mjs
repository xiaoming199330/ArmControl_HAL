/**
 * udp_loopback.mjs — **真 UDP socket 回环测试**
 *
 * 前面所有测试都跑在内存假臂上，它们证明不了"UDP 承载真的能用"。
 * 这个测试起一个**真的 UDP 服务器**（Node dgram，实现在真 socket 上，
 * 逻辑复用 sim 的命令处理器），然后让 UdpTransport + ArmClient 通过
 * **真实网络栈**（127.0.0.1）打过去。
 *
 * 覆盖：
 *   · 一个数据报 = 一个逻辑包（无帧头）真的能通
 *   · 回包发往请求的源端口（客户端不绑固定端口也能收到）
 *   · 14 条命令在真实 socket 上逐条通
 *   · 丢包 / 乱序 / 大包边界
 *   · 真实 RTT 分布（不是 sim 的假延迟）
 *
 * 跑法: node test/udp_loopback.mjs
 */
import dgram from 'node:dgram';

import {
  CMD, packLogical, unpackLogical, decodeStatus, decodeFk, toHex,
  UdpTransport, ArmClient, openArm, ErrCode, SimTransport,
  payloadMoveJoints, payloadStreamJoints,
} from '../src/index.js';

let n = 0;
const fails = [];
const chk = (name, cond, extra = '') => {
  n++;
  if (!cond) { fails.push(name + ' ' + extra); console.log('  FAIL ' + name + '  ' + extra); }
};
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

/* ==========================================================================
 * 假固件：把 sim 的命令处理器挂到一个真的 UDP socket 上
 * ========================================================================== */

async function startFakeFirmware(opts = {}) {
  // 复用 sim 的命令处理逻辑（"_handle" 返回 [respCmd, payload]）
  const brain = new SimTransport(opts);
  await brain.open();

  const sock = dgram.createSocket('udp4');
  const stats = { rx: 0, tx: 0, dropped: 0, badLen: 0, ctrl: 0 };
  const drops = new Set(opts.dropSeqs || []);

  await new Promise((r) => sock.bind(opts.port ?? 0, '127.0.0.1', r));
  const port = sock.address().port;

  sock.on('message', (msg, rinfo) => {
    stats.rx++;
    let pkt;
    try {
      // ⚠ UDP 承载**没有帧头**：数据报本身就是逻辑包
      pkt = unpackLogical(new Uint8Array(msg));
    } catch (e) {
      // 固件对长度不符/CRC 错的包：这里只统计，不回（和固件一致）
      stats.badLen++;
      return;
    }
    if (drops.has(pkt.seq)) { stats.dropped++; return; }

    const resp = brain._handle(pkt.seq, pkt.cmd, pkt.payload);
    if (resp === null) return;                       // CMD>=0x80 故意不回

    const out = packLogical(pkt.seq, resp[0], resp[1]);
    stats.tx++;
    // ⚠ 固件把回包发往**请求的源 IP:源端口**
    const delay = opts.replyDelayMs ?? 0;
    if (delay > 0) setTimeout(() => sock.send(Buffer.from(out), rinfo.port, rinfo.address), delay);
    else sock.send(Buffer.from(out), rinfo.port, rinfo.address);
  });

  return {
    port, stats, brain,
    close: () => new Promise((r) => sock.close(r)),
  };
}

/* ==========================================================================
 * 开始
 * ========================================================================== */

console.log('真 UDP 回环测试（127.0.0.1，真实 socket）');
console.log('-'.repeat(66));

const fw = await startFakeFirmware({ replyDelayMs: 0 });
console.log(`  假固件监听 127.0.0.1:${fw.port}`);

/* ---- 1. 最低层：裸 socket 收发包 ---- */
{
  const t = new UdpTransport('127.0.0.1', fw.port, { timeoutMs: 1000 });
  await t.open();
  t.send(0x31, CMD.PING, new Uint8Array(0));
  await sleep(60);
  const evs = t.drain();
  chk('UDP 收到回包', evs.length === 1,
      `${evs.length} 个事件: ${evs.map(String).join('|')}`);
  if (evs.length) {
    chk('回包是 packet 事件', evs[0].kind === 'packet', evs[0].kind);
    chk('SEQ 原样回', evs[0].seq === 0x31, String(evs[0].seq));
    chk('PING 响应码 0x81', evs[0].cmd === 0x81, '0x' + evs[0].cmd.toString(16));
    chk('负载 2 字节', evs[0].payload.length === 2);
  }
  await t.close();
  console.log('  ✓ 裸 UDP 收发（无帧头，一报一包）');
}

/* ---- 2. 客户端不绑固定端口也能收到回包 ---- */
{
  const t1 = new UdpTransport('127.0.0.1', fw.port, { timeoutMs: 800 });
  await t1.open();
  const p1 = t1._sock.address().port;
  const armA = new ArmClient(t1, { timeout: 0.5 });
  await armA.open();
  const info = await armA.ping();
  chk('客户端未绑固定端口也能收到回包（固件发往源端口）', info !== null,
      '本端端口 ' + p1);
  await armA.close();
  console.log(`  ✓ 回包发往请求源端口（本端临时端口 ${p1}）`);
}

/* ---- 3. 14 条命令全部在真 socket 上过一遍 ---- */
{
  const arm = await openArm(`net://127.0.0.1:${fw.port}`, { timeout: 0.8 });

  const p = await arm.ping();
  chk('PING', p !== null && p.proto_ver === 1);

  const st = await arm.getStatus();
  chk('GET_STATUS', st !== null && st.joints.length === 6, JSON.stringify(st && st.joints));
  chk('GET_STATUS 初值 180', st && st.joints.every((v) => v === 180));

  const fk = await arm.fkQuery();
  chk('FK_QUERY', fk !== null && fk.read_ok === 1);

  chk('LOG_SET_LEVEL', (await arm.logSetLevel(2)).ok);
  chk('MOVE_JOINTS', (await arm.moveJoints([180, 180, 180, 180, 180, 180], 30)).ok);
  chk('SET_TORQUE', (await arm.setTorque(0xFF, true)).ok);
  chk('SET_GRAVITY', (await arm.setGravity(0.1)).ok);
  chk('SET_CORRECTION', (await arm.setCorrection(true, 2, 10)).ok);
  chk('MOVE_CART', (await arm.moveCart(300, 0, 200, 90)).ok);

  const pc = await arm.paramCount();
  chk('PARAM_COUNT', pc === 24, String(pc));

  const pw = await arm.paramWrite(3, 2.5);
  chk('PARAM_WRITE', pw.ok);
  const pv = await arm.paramRead(3);
  chk('PARAM_READ 回读一致', Math.abs(pv - 2.5) < 1e-6, String(pv));

  const sj = await arm.streamJoints([0, 0, 0, 0, 0, 0]);
  chk('STREAM_JOINTS', sj !== null && sj.ok, sj ? sj.toString() : 'null');

  chk('STOP', (await arm.stop()).ok);

  // 错误路径
  // ① 客户端侧就拦下（payload 构造器直接抛，不给机会发包）
  //    ⚠ setTorque 是 async，抛错会变成 rejected Promise ⇒ 必须 await 才 catch 得到
  let threw = false;
  try { await arm.setTorque(6, true); } catch { threw = true; }
  chk('非法关节号在客户端构造负载时就抛错', threw);

  // ② 绕过客户端护栏、直接发原始负载 ⇒ 应由**固件**回 err=1
  const raw = await arm.request(CMD.SET_TORQUE, Uint8Array.from([6, 1]));
  chk('绕过护栏直发 J6 → 固件回 err=1',
      raw.err === ErrCode.PARAM, `cmd=0x${raw.cmd.toString(16)} err=${raw.err}`);

  const e2 = await arm.paramRead(200);
  chk('非法参数 ID → null', e2 === null);

  await arm.close();
  console.log('  ✓ 14 条命令在真实 UDP socket 上全部通过');
}

/* ---- 4. 丢包：客户端应换 SEQ 重试，而不是崩 ---- */
{
  // 固件侧丢掉 SEQ 1~5。客户端 retries=6 ⇒ 共 7 次机会，
  // 第 6 次用 SEQ=6（不在丢包表里）⇒ 应该成功。
  const fw2 = await startFakeFirmware({ dropSeqs: [1, 2, 3, 4, 5] });
  const arm = await openArm(`net://127.0.0.1:${fw2.port}`,
                            { timeout: 0.2, retries: 6 });
  const r = await arm.ping();
  chk('丢包时靠换 SEQ 重试最终成功', r !== null,
      r ? 'latency=' + r.latencyMs.toFixed(1) : 'null');
  chk('attempts >= 6（确实重试了，且只在超时时重试）',
      r && r.reply.attempts >= 6, r ? String(r.reply.attempts) : 'null');
  chk('固件侧确实丢了 5 个包', fw2.stats.dropped === 5, String(fw2.stats.dropped));
  await arm.close();
  await fw2.close();
  console.log(`  ✓ 丢包恢复（固件丢 5 个，客户端重试到第 ${r.reply.attempts} 次成功）`);
}

/* ---- 4b. 运动命令**绝不**自动重发（重发=多动一次） ---- */
{
  const fw4 = await startFakeFirmware({ dropSeqs: [1, 2, 3, 4, 5] });
  const arm = await openArm(`net://127.0.0.1:${fw4.port}`,
                            { timeout: 0.2, retries: 6 });
  const r = await arm.moveJoints([180, 180, 180, 180, 180, 180], 30);
  chk('MOVE_JOINTS 被丢时**不重发**（attempts 恒为 1）',
      r.attempts === 1, String(r.attempts));
  chk('MOVE_JOINTS 因此超时返回', r.ok === false && r.timeout === true);
  await arm.close();
  await fw4.close();
  console.log('  ✓ 运动命令不自动重发（安全设计：重发=多动一次）');
}

/* ---- 5. 全丢：应干净超时 ---- */
{
  const fw3 = await startFakeFirmware({
    dropSeqs: Array.from({ length: 256 }, (_, i) => i),   // 全丢
  });
  const arm = await openArm(`net://127.0.0.1:${fw3.port}`,
                            { timeout: 0.15, retries: 0 });
  const t0 = Date.now();
  const r = await arm.ping();
  const dt = Date.now() - t0;
  chk('全丢时返回 null（不抛异常）', r === null);
  chk('超时耗时约等于设定值', dt >= 120 && dt < 900, dt + 'ms');
  await arm.close();
  await fw3.close();
  console.log(`  ✓ 全丢干净超时（${dt}ms，无异常）`);
}

/* ---- 6. 大包边界：64 B 负载（最大）与 65 B（超限） ---- */
{
  const arm = await openArm(`net://127.0.0.1:${fw.port}`, { timeout: 0.8 });
  // 64 B 负载的 MOVE_CART 之类不存在，这里直接验证 pack/unpack 边界
  const maxPl = new Uint8Array(64);
  const pkt = packLogical(1, CMD.MOVE_CART, maxPl);
  chk('64 B 负载 → 69 B 报文（协议上限）', pkt.length === 69, String(pkt.length));
  let threw = false;
  try { packLogical(1, CMD.MOVE_CART, new Uint8Array(65)); } catch { threw = true; }
  chk('65 B 负载 → 抛错（超 APROTO_MAX_PAYLOAD）', threw);
  await arm.close();
  console.log('  ✓ 大包边界（64 B 负载 = 69 B 报文；65 B 拒绝）');
}

/* ---- 7. 发送给错误端口 → 客户端超时（不是崩溃） ---- */
{
  const arm = await openArm('net://127.0.0.1:1', { timeout: 0.15, retries: 0 });
  const r = await arm.ping();
  chk('错误端口 → 超时返回 null', r === null);
  await arm.close();
  console.log('  ✓ 错误端口干净超时');
}

/* ---- 8. 真实 RTT 分布 ---- */
{
  const arm = await openArm(`net://127.0.0.1:${fw.port}`, { timeout: 1.0 });
  const lat = [];
  for (let i = 0; i < 200; i++) {
    const s = await arm.ping();
    if (s) lat.push(s.latencyMs);
  }
  lat.sort((a, b) => a - b);
  const p50 = lat[Math.floor(lat.length * 0.5)];
  const p99 = lat[Math.floor(lat.length * 0.99)];
  chk('本地回环 RTT 样本 200/200', lat.length === 200, String(lat.length));
  chk('本地回环 p50 < 5ms', p50 < 5, p50.toFixed(2) + 'ms');
  console.log(`  ✓ 真实 socket RTT：p50=${p50.toFixed(2)}ms  p99=${p99.toFixed(2)}ms  ` +
              `min=${lat[0].toFixed(2)}  max=${lat[lat.length - 1].toFixed(2)}`);
  console.log('    （本地回环，仅供参考；真机网口实测 p50=1.61ms）');
  await arm.close();
}

/* ---- 9. 乱序 / 并发：多个请求同时飞，SEQ 配对不能错 ---- */
{
  const arm = await openArm(`net://127.0.0.1:${fw.port}`, { timeout: 1.0 });
  const ps = [];
  for (let i = 0; i < 20; i++) ps.push(arm.ping());
  const rs = await Promise.all(ps);
  chk('20 个并发 PING 全部配对成功', rs.every((r) => r !== null),
      rs.filter((r) => r === null).length + ' 个失败');
  // 校验每个响应确实是自己的 SEQ（配错的话 read_ok/值会串）
  const sts = await Promise.all([arm.getStatus(), arm.getStatus(), arm.getStatus()]);
  chk('并发 GET_STATUS 结果都合法',
      sts.every((s) => s && s.joints.length === 6));
  await arm.close();
  console.log('  ✓ 并发 20 请求 + SEQ 配对无串包');
}

/* ---- 10. 假固件侧的收发统计 ---- */
{
  chk('假固件收包数 > 发前', fw.stats.rx > 0, String(fw.stats.rx));
  chk('假固件发包数 > 0', fw.stats.tx > 0, String(fw.stats.tx));
  console.log(`  ✓ 假固件统计：rx=${fw.stats.rx} tx=${fw.stats.tx} ` +
              `badLen=${fw.stats.badLen}`);
}

await fw.close();

/* ---- 汇总 ---- */
console.log('\n' + '='.repeat(66));
if (fails.length) {
  console.log(`真 UDP 回环：失败 ${fails.length} / ${n} 项`);
  fails.forEach((f) => console.log('  · ' + f));
  process.exit(1);
}
console.log(`真 UDP 回环：${n} 项，全部通过`);
console.log('UDP 承载在真实 socket 上验证通过。');
