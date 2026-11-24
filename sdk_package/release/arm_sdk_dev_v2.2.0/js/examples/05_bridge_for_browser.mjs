/**
 * 05_bridge_for_browser.mjs — 起 WebSocket↔UDP 边车，并自证"浏览器那条路"是通的
 *
 * 为什么需要边车：浏览器沙箱**做不到**原始 UDP。要在网页里用 p50 1.61ms 的网口，
 * 必须让一个 Node 进程把 WebSocket 帧翻成 UDP：
 *
 *     网页(WsTransport) ──ws://127.0.0.1:8765──> bridge.js ──UDP──> 机械臂
 *
 * 跑法：
 *     # 有真臂：
 *     node examples/05_bridge_for_browser.mjs --target 192.168.7.120:5000
 *     # 没有真臂：脚本自带一个"假固件"UDP 服务来验证整条链路
 *     node examples/05_bridge_for_browser.mjs --fake
 *
 * 然后把 examples/browser/index.html 用一个静态服务器打开（见文件头注释）。
 */

import dgram from 'node:dgram';
import { createBridge } from '../src/bridge.js';
import { packLogical, unpackLogical, CMD, crc16 } from '../src/protocol.js';
import { arg, flag } from './_args.mjs';

let fake = null;
let target = arg('target', null);

/* ---- 没有真臂时：起一个只会回 PING / GET_STATUS 的假固件 ---- */
if (flag('fake') || !target) {
  fake = dgram.createSocket('udp4');
  await new Promise((r) => fake.bind(0, '127.0.0.1', r));
  const fp = fake.address().port;
  target = `127.0.0.1:${fp}`;
  fake.on('message', (msg, rinfo) => {
    let req;
    try { req = unpackLogical(new Uint8Array(msg)); } catch { return; }
    let payload;
    if (req.cmd === CMD.PING) payload = Uint8Array.from([1, 0x5A]);
    else if (req.cmd === CMD.GET_STATUS) {
      payload = new Uint8Array(21);
      for (let j = 0; j < 6; j++) {
        const v = Math.round(1800);           // 180.0° → 1800（×10）
        payload[j * 2] = v & 0xFF; payload[j * 2 + 1] = (v >> 8) & 0xFF;
      }
      payload[20] = 0x08;                     // read_ok
    } else if (req.cmd & 0x80) return;
    else payload = Uint8Array.from([req.cmd, 0]);
    const rsp = req.cmd >= 0x80 ? req.cmd : (req.cmd | 0x80);
    fake.send(Buffer.from(packLogical(req.seq, rsp, payload)), rinfo.port, rinfo.address);
  });
  console.log('[demo] 已起假固件 UDP 服务 127.0.0.1:' + fp);
}

void crc16;

/* ---- 起边车 ---- */
const br = await createBridge({ port: Number(arg('port', 8765)), target, allow: '127.0.0.1' });
console.log('\n════════════════════════════════════════════════');
console.log('  边车已就绪（Ctrl-C 退出）');
console.log(`  浏览器里用： openArm('ws://127.0.0.1:${br.port}/')`);
console.log(`  指定另一台臂： openArm('ws://127.0.0.1:${br.port}/?target=192.168.7.121:5000')`);
console.log('════════════════════════════════════════════════\n');

/* ---- 用 Node 的全局 WebSocket 冒充浏览器，验证整条链路 ---- */
if (typeof WebSocket !== 'undefined') {
  const ws = new WebSocket(`ws://127.0.0.1:${br.port}/`);
  ws.binaryType = 'arraybuffer';
  await new Promise((res, rej) => {
    const to = setTimeout(() => rej(new Error('连边车超时')), 3000);
    ws.onopen = () => { clearTimeout(to); res(); };
    ws.onerror = () => { clearTimeout(to); rej(new Error('连边车失败')); };
  });

  const pingRsp = await new Promise((resolve) => {
    ws.onmessage = (ev) => {
      if (!(ev.data instanceof ArrayBuffer)) return;      // hello 是文本帧，跳过
      const { seq, cmd, payload } = unpackLogical(new Uint8Array(ev.data));
      if (cmd === (CMD.PING | 0x80)) resolve({ seq, cmd, payload });
    };
    ws.send(Buffer.from(packLogical(0x11, CMD.PING)));
    setTimeout(() => resolve(null), 2000);
  });

  if (pingRsp) {
    console.log(`[自证] 经 WebSocket→UDP 收到 PING 响应：cmd=0x${pingRsp.cmd.toString(16)} ` +
                `payload=[${Array.from(pingRsp.payload)}]`);
    console.log('[自证] ✓ 浏览器那条路已验证通过\n');
  } else {
    console.log('[自证] ✗ 没收到响应\n');
  }
  ws.close();
} else {
  console.log('[提示] 当前 Node 没有全局 WebSocket，跳过自证（Node 21+ 才有）\n');
}

/* ---- 常驻 ---- */
await new Promise((resolve) => {
  process.on('SIGINT', resolve);
  process.on('SIGTERM', resolve);
});
await br.close();
if (fake) fake.close();
console.log('边车已停。');
