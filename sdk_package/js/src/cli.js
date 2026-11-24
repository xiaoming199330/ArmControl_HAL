#!/usr/bin/env node
/**
 * cli.js — `armctl-js` 命令行（Node 18+，**零依赖**）
 *
 * 和 Python 版 `armctl` 同构，但默认承载是 **UDP（net://）**：
 *
 *   node src/cli.js info
 *   node src/cli.js --uri net://192.168.7.120:5000 ping
 *   node src/cli.js --uri net://192.168.7.120:5000 status --json
 *   node src/cli.js move --delta 0,5,0,0,0,0 --v 10
 *   node src/cli.js bridge --target 192.168.7.120:5000     # 给浏览器用
 *
 * 安全约定（照抄 Python 版）
 * -------------------------
 *   · 点动**必须** `--delta`：绝对角会被固件 `clamp_all_angles()` **静默钳制**。
 *   · 每个会动的子命令先读一次状态，把当前真实姿态打出来再动。
 *   · `--dry-run` 是**最底层**出口：一个字节都不发。
 *
 * 串口相关子命令（console / link / relay）
 * ----------------------------------------
 * Node 里没有 Web Serial，也没有内置串口。这几条要么在**浏览器**里用
 * WebSerialTransport（SDK 的 console.js / FirmwareLink），要么用 Python 版
 * `armctl`。此处给出提示，不做半吊子实现。
 */

import { fileURLToPath } from 'node:url';
import process from 'node:process';

import { CMD, AP_PARAM_COUNT, MAX_PACKET, PARAM_TABLE, JointCalib } from './protocol.js';
import { openArm } from './client.js';
import { TeleopConfig, TeleopSession, TeleopAbort, TeleopLinkLost } from './teleop.js';
import {
  Trajectory, TeleopTeachSession, PlaybackSession, saveTrajectory, loadTrajectory,
} from './teach.js';

const VERSION = '2.0.0';
const DEFAULT_URI = process.env.ARM_URI || 'net://192.168.7.120:5000';

/* ==========================================================================
 * 输出小工具
 * ========================================================================== */

const c = {
  ok: (s) => `✓ ${s}`,
  bad: (s) => `✗ ${s}`,
  warn: (s) => `⚠ ${s}`,
};

function die(msg, code = 2) {
  process.stderr.write('armctl-js: ' + msg + '\n');
  const e = new Error(msg); e.exitCode = code; throw e;
}

function fmtQ(q, nd = 1) {
  if (q === null || q === undefined) return '[读失败]';
  return '[' + q.map((v) => (v === null || v === undefined
    ? '--' : Number(v).toFixed(nd))).join(', ') + ']';
}

function parseSix(text, what = '6 个值') {
  const parts = String(text).replace(/,/g, ' ').split(/\s+/).filter(Boolean);
  if (parts.length !== 6) die(`${what}需要 6 个值，收到 ${parts.length} 个：${JSON.stringify(text)}`);
  const out = parts.map(Number);
  if (out.some((v) => !Number.isFinite(v))) die('无法解析数字：' + JSON.stringify(text));
  return out;
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

/* ==========================================================================
 * 参数解析（手写，零依赖）
 * ========================================================================== */

/**
 * 极简解析：位置参数进 `_`，`--k v` / `--flag` / `-x` 进对象。
 * 需要重复的键（如 --cmd）用 `multi` 声明。
 */
function parseArgv(argv, spec = {}) {
  const alias = spec.alias || {};
  const bools = new Set(spec.bools || []);
  const multi = new Set(spec.multi || []);
  const out = { _: [] };
  for (let i = 0; i < argv.length; i++) {
    let s = argv[i];
    if (s === '--') { out._.push(...argv.slice(i + 1)); break; }
    if (s.startsWith('--')) {
      let key = s.slice(2);
      let val = null;
      const eq = key.indexOf('=');
      if (eq >= 0) { val = key.slice(eq + 1); key = key.slice(0, eq); }
      key = alias[key] || key;
      if (bools.has(key)) { out[key] = val === null ? true : val !== 'false'; continue; }
      if (val === null) val = argv[++i];
      if (val === undefined) die('选项 --' + key + ' 缺少值');
      if (multi.has(key)) (out[key] = out[key] || []).push(val);
      else out[key] = val;
      continue;
    }
    if (s.length > 1 && s[0] === '-' && !/^-?\d/.test(s)) {
      const key = alias[s.slice(1)] || s.slice(1);
      if (bools.has(key)) { out[key] = true; continue; }
      out[key] = argv[++i];
      continue;
    }
    out._.push(s);
  }
  return out;
}

/* ==========================================================================
 * 开臂
 * ========================================================================== */

async function openFromArgs(a, uri) {
  const u = uri || a.uri || DEFAULT_URI;
  return openArm(u, {
    timeout: a.timeout !== undefined ? Number(a.timeout) : 0.3,
    dryRun: !!a['dry-run'],
    verbose: !!a.verbose,
    onEvent: a.verbose ? (m) => console.log('  · ' + m) : null,
  });
}

async function prelude(arm) {
  let st;
  try { st = await arm.getStatus(); } catch (e) { console.log('  ' + c.warn('读状态失败：' + e.message)); return null; }
  if (!st) { console.log('  ' + c.warn('读状态无响应')); return null; }
  console.log('  当前舵机度 ' + fmtQ(st.servoDeg ?? st.joints) +
              `  flags(estop=${st.estop} traj=${st.traj} corr=${st.corr} read_ok=${st.read_ok})`);
  if (st.failedJoints && st.failedJoints.length) console.log('  ' + c.warn('读失败的轴：J' + st.failedJoints.join(',J')));
  if (st.estop) console.log('  🛑 E-Stop 激活 —— 现在动不了');
  return st;
}

/** 状态对象的舵机度（兼容两种字段命名）。 */
function servoOf(st) {
  return st.servoDeg ?? st.joints ?? (st.asServoDeg ? st.asServoDeg() : null);
}

/* ==========================================================================
 * 子命令
 * ========================================================================== */

async function cmdInfo() {
  console.log(`arm-sdk-js ${VERSION}`);
  console.log('  零依赖；Node 18+（UDP / 模拟器）；浏览器走 bridge.js 边车或 Web Serial');
  console.log('  四种承载：');
  console.log('    net://<ip>:5000    UDP，一个数据报 = 一个逻辑包     实测 p50 1.61ms  ★首选（Node/Electron 主进程）');
  console.log('    ws://<host>:8765   浏览器走边车（bridge.js），同一份逻辑包');
  console.log('    webserial://       浏览器 Web Serial（需用户手势授权）   实测 p50 4.78ms');
  console.log('    sim://             内存假臂（离线开发 / CI）');
  console.log('                     ⚠ sim 状态只活在**当前进程**：每条命令都是新臂；');
  console.log('                       要连续操作用 SDK API（openArm("sim://")）或一条命令做到底');
  // 真·请求命令 14 条（CMD 里另含 ACK/NACK 两个伪码，不算）
  const nReq = Object.keys(CMD).filter((k) => k !== 'ACK' && k !== 'NACK').length;
  console.log(`  二进制命令 ${nReq} 条；运行时参数 ${AP_PARAM_COUNT} 项；最大逻辑包 ${MAX_PACKET} B`);
  console.log('  参数表节选：' + Object.entries(PARAM_TABLE).slice(0, 6)
    .map(([k, v]) => `${k}=${v.name}`).join(', '));
  return 0;
}

async function cmdSelftest() {
  const here = fileURLToPath(new URL('.', import.meta.url));
  const runner = fileURLToPath(new URL('../test/run_all.mjs', import.meta.url));
  const { spawnSync } = await import('node:child_process');
  const r = spawnSync(process.execPath, [runner], { cwd: fileURLToPath(new URL('..', import.meta.url)), stdio: 'inherit' });
  void here;
  console.log('\n' + (r.status === 0 ? c.ok('全部自检通过') : c.bad('有自检失败')));
  return r.status ?? 1;
}

async function cmdBridge(a) {
  const { createBridge, selftest } = await import('./bridge.js');
  if (a.selftest) {
    const r = await selftest();
    console.log(`bridge 自检：${r.pass} 项通过` +
                (r.fail.length ? `，${r.fail.length} 项失败` : '，全部通过') +
                (r.skipped ? `（${r.skipped}）` : ''));
    for (const f of r.fail) console.log('  ✗ ' + f);
    return r.ok ? 0 : 1;
  }
  const br = await createBridge({
    port: a.port !== undefined ? Number(a.port) : 8765,
    host: a.host || '127.0.0.1',
    target: a.target || null,
    allow: a.allow || '127.0.0.1',
    verbose: !!a.verbose,
  });
  console.log('边车已就绪。Ctrl-C 退出。');
  await new Promise((resolve) => {
    process.on('SIGINT', resolve);
    process.on('SIGTERM', resolve);
  });
  await br.close();
  console.log('\n边车已停。');
  return 0;
}

async function cmdPreflight(a) {
  console.log('=== 上板前置检查 ===');
  const arm = await openFromArgs(a);
  let ok = true;
  try {
    const pi = await arm.ping();
    if (!pi) {
      console.log('  ' + c.bad('PING 无响应 —— 查：①link 偏好(需 link net→save→RESET) ②目的端口 5000'));
      console.log('    ③本构建是否含网口(LINK=net|both) ④网线/IP');
      ok = false;
    } else {
      console.log(`  ${c.ok(`PING proto_ver=${pi.proto_ver} run_id=${pi.run_id}`)}`);
    }
    const st = await arm.getStatus();
    if (!st) { console.log('  ' + c.bad('GET_STATUS 无响应')); ok = false; }
    else {
      const bad = st.failedJoints || [];
      console.log(`  ${bad.length ? '✗' : '✓'} GET_STATUS 舵机度 ${fmtQ(servoOf(st))}`);
      if (bad.length) { console.log('    ' + c.warn('读失败的轴 J' + bad.join(',J') + ' —— 先排总线再运动')); ok = false; }
      if (st.estop) { console.log('    🛑 E-Stop 激活'); ok = false; }
      if (!st.read_ok) { console.log('    ' + c.warn('read_ok=0 —— 位姿字段无效，不要当真值用')); ok = false; }
    }
    try { console.log(`  ${c.ok('PARAM_COUNT = ' + await arm.paramCount() + '（现役 24）')}`); }
    catch (e) { console.log('  ' + c.warn('PARAM_COUNT 失败：' + e.message)); }
    try {
      const fk = await arm.fkQuery();
      if (fk) console.log(`  ${c.ok(`FK_QUERY X=${fk.x.toFixed(1)} Y=${fk.y.toFixed(1)} Z=${fk.z.toFixed(1)} pitch=${fk.pitch.toFixed(1)} read_ok=${fk.read_ok}`)}`);
      else console.log('  ' + c.warn('FK_QUERY 无响应'));
    } catch (e) { console.log('  ' + c.warn('FK_QUERY 失败：' + e.message)); }
    console.log('  提示：串口侧还应跑 `h`(fw-tag) / `zi`(零点是否被污染) / `L`(运行时限位)');
  } finally {
    await arm.close();
  }
  console.log('=== ' + (ok ? '可以动' : '先修上面的问题') + ' ===');
  return ok ? 0 : 1;
}

async function cmdPing(a) {
  const arm = await openFromArgs(a);
  try {
    const t0 = Date.now();
    const pi = await arm.ping();
    const dt = Date.now() - t0;
    if (!pi) die('PING 无响应（见 `armctl-js preflight` 的排查清单）', 1);
    console.log(`✓ PING proto_ver=${pi.proto_ver} run_id=${pi.run_id}  ${dt.toFixed(1)}ms`);
    const repeat = Number(a.repeat || 1);
    if (repeat > 1) {
      const ts = [];
      for (let i = 1; i < repeat; i++) {
        const s = Date.now();
        if (await arm.ping()) ts.push(Date.now() - s);
      }
      if (ts.length) {
        ts.sort((x, y) => x - y);
        console.log(`  ${ts.length} 次：p50=${ts[Math.floor(ts.length / 2)].toFixed(2)}ms ` +
                    `min=${ts[0].toFixed(2)} max=${ts[ts.length - 1].toFixed(2)}`);
      }
    }
  } finally { await arm.close(); }
  return 0;
}

async function cmdStatus(a) {
  const arm = await openFromArgs(a);
  try {
    const watch = Number(a.watch || 0);
    const interval = Number(a.interval !== undefined ? a.interval : 0.2);
    for (let n = 0; ; n++) {
      const t0 = Date.now();
      const st = await arm.getStatus();
      const dt = Date.now() - t0;
      if (!st) { console.log('无响应'); }
      else if (a.json) {
        console.log(JSON.stringify({
          joints_servo_deg: servoOf(st), x: st.x, y: st.y, z: st.z,
          pitch: st.pitch, estop: st.estop, traj: st.traj, corr: st.corr,
          read_ok: st.read_ok, failed_joints_1based: st.failedJoints,
          latency_ms: Math.round(dt * 100) / 100,
        }));
      } else {
        console.log(`${fmtQ(servoOf(st))}  X=${st.x.toFixed(1)} Y=${st.y.toFixed(1)} Z=${st.z.toFixed(1)} ` +
                    `pitch=${st.pitch.toFixed(1)}  ${dt.toFixed(1)}ms${st.estop ? '  🛑E-STOP' : ''}`);
      }
      if (!watch || n + 1 >= watch) break;
      await sleep(interval * 1000);
    }
  } finally { await arm.close(); }
  return 0;
}

async function cmdFk(a) {
  const arm = await openFromArgs(a);
  try {
    const fk = await arm.fkQuery();
    if (!fk) die('FK_QUERY 无响应', 1);
    console.log(`X=${fk.x.toFixed(1)} mm  Y=${fk.y.toFixed(1)} mm  Z=${fk.z.toFixed(1)} mm  ` +
                `pitch=${fk.pitch.toFixed(1)} deg  read_ok=${fk.read_ok}`);
    console.log('舵机度 ' + fmtQ(fk.joints));
    if (fk.read_ok !== 1) console.log(c.warn('read_ok=0 —— 位姿无效（有轴读失败）'));
  } finally { await arm.close(); }
  return 0;
}

async function cmdMove(a) {
  if (a._.length < 1) die('用法：move <6 个角> [--delta] [--v 25] [--a 0] [--wait]');
  const arm = await openFromArgs(a);
  try {
    await prelude(arm);
    let target = parseSix(a._[0], '目标角');
    if (a.delta) {
      const st = await arm.getStatus();
      const cur = st ? servoOf(st) : null;
      if (!cur || cur.some((v) => v === null || v === undefined)) {
        die('要 --delta 但当前有轴读失败（J' + (st?.failedJoints || []).join(',J') + '）—— 不能用未知起点做增量', 1);
      }
      target = cur.map((v, j) => v + target[j]);
      console.log(`  增量模式：当前 ${fmtQ(cur)} → 目标 ${fmtQ(target)}`);
    } else {
      console.log(`  绝对模式：目标 ${fmtQ(target)}（⚠ 超限会**静默钳制**，err=0 不代表到位）`);
    }
    const r = await arm.moveJoints(target, Number(a.v ?? 25), Number(a.a ?? 0));
    console.log('  ' + r.toString());
    if (!r.ok) return 1;
    if (a.wait) {
      await sleep(300);
      const st = await arm.getStatus();
      console.log('  到位检查（err=0 不代表动了，必须看角度）：' + fmtQ(st ? servoOf(st) : null));
    }
  } finally { await arm.close(); }
  return 0;
}

async function cmdCart(a) {
  if (a._.length < 3) die('用法：cart <x> <y> <z> [--pitch 0] [--v 25] [--a 0]');
  const arm = await openFromArgs(a);
  try {
    await prelude(arm);
    const [x, y, z] = a._.slice(0, 3).map(Number);
    const pitch = Number(a.pitch ?? 0);
    console.log(`  MOVE_CART X=${x.toFixed(1)} Y=${y.toFixed(1)} Z=${z.toFixed(1)} pitch=${pitch.toFixed(1)} ⚠ 会归零 J5/J6`);
    const r = await arm.moveCart(x, y, z, pitch, Number(a.v ?? 25), Number(a.a ?? 0));
    console.log('  ' + r.toString());
    return r.ok ? 0 : 1;
  } finally { await arm.close(); }
}

async function cmdStop(a) {
  const arm = await openFromArgs(a);
  try {
    const r = await arm.stop();
    console.log(`${r.toString()}（平滑减速，不是急停断电）`);
  } finally { await arm.close(); }
  return 0;
}

async function cmdTorque(a) {
  const arm = await openFromArgs(a);
  try {
    const j = a.joint === undefined ? 0xFF : Number(a.joint);
    const r = await arm.setTorque(j, !a.off);
    console.log(r.toString());
    return r.ok ? 0 : 1;
  } finally { await arm.close(); }
  return 0;
}

async function cmdStream(a) {
  const arm = await openFromArgs(a);
  try {
    const st = await arm.getStatus();
    const sv = st ? servoOf(st) : null;
    if (!sv || sv.some((v) => v === null || v === undefined)) {
      die('有轴读失败（J' + (st?.failedJoints || []).join(',J') + '），先排总线', 1);
    }
    const calib = new JointCalib();
    const cur = calib.toLogical(sv);
    const hz = Number(a.hz ?? 50);
    const sec = Number(a.sec ?? 2);
    const tgt = a.pose ? parseSix(a.pose, '--pose（逻辑角）') : [...cur];
    console.log('  当前逻辑角 ' + fmtQ(cur));
    console.log(`  目标逻辑角 ${fmtQ(tgt)} @ ${hz.toFixed(0)} Hz for ${sec.toFixed(1)}s`);

    const tr = new Trajectory({ name: 'cli-stream', hz });
    const ramp = Math.max(0.2, sec * 0.3);
    const steps = Math.max(2, Math.round(ramp * hz));
    for (let i = 0; i <= steps; i++) {
      const k = i / steps;
      tr.append(ramp * k, cur.map((v, j) => v + (tgt[j] - v) * k));
    }
    const hold = Math.max(0, sec - ramp);
    for (let i = 0; i < Math.round(hold * hz); i++) tr.append(ramp + i / hz, [...tgt]);

    const pb = new PlaybackSession(arm, {
      cfg: new TeleopConfig({ hz }),
      slewDegPerSec: Number(a.slew ?? 240),
      estopCheckS: 0,
      onEvent: (m) => console.log('  ' + m),
    });
    const stats = await pb.play(tr, { speed: 1.0, approach: false });
    console.log('  ' + stats.summary());
  } finally { await arm.close(); }
  return 0;
}

async function cmdParams(a) {
  const action = a._[0];
  if (!action) die('用法：params <list|get|set|count> [id] [value]');
  const arm = await openFromArgs(a);
  try {
    if (action === 'count') {
      console.log(String(await arm.paramCount()));
    } else if (action === 'list') {
      let n = -1;
      try { n = await arm.paramCount(); } catch { /* 忽略 */ }
      console.log(`现役 ${n} 项：`);
      for (const pid of Object.keys(PARAM_TABLE).map(Number).sort((x, y) => x - y)) {
        const v = await arm.paramRead(pid);
        const { name, unit, min, max, def } = PARAM_TABLE[pid];
        console.log(`  ${String(pid).padStart(2)} ${name.padEnd(22)} ${(v === null ? '读失败' : v.toFixed(3)).padStart(10)} ${unit}  [${min}~${max}] 默认 ${def}`);
      }
    } else if (action === 'get') {
      console.log(String(await arm.paramRead(Number(a._[1]))));
    } else if (action === 'set') {
      const id = Number(a._[1]); const val = Number(a._[2]);
      const r = await arm.paramWrite(id, val);
      const back = await arm.paramRead(id);
      console.log(`${r.toString()}  → 回读 = ${back}（⚠ 超范围会静默钳制，err=0 不代表写成功）`);
      return r.ok ? 0 : 1;
    } else die('未知 action: ' + action);
  } finally { await arm.close(); }
  return 0;
}

async function cmdTeleop(a) {
  if (!a.leader || !a.follower) die('用法：teleop --leader <uri> --follower <uri> [--sec N]');
  const lead = await openFromArgs(a, a.leader);
  const foll = await openFromArgs(a, a.follower);
  const cfg = new TeleopConfig({
    hz: Number(a.hz ?? 50),
    slewDegPerSec: Number(a.slew ?? 240),
    gain: Number(a.gain ?? 1.0),
    sync: a.sync || 'hold',
    waitAck: false,
    verbose: true,
  });
  if (a['soft-limit']) {
    const lim = parseSix(a['soft-limit'], '--soft-limit');
    cfg.softLimitsDeg = lim.map((v) => [-Math.abs(v), Math.abs(v)]);
  }
  console.log(`主从遥操：leader=${a.leader} → follower=${a.follower}  ` +
              `${cfg.hz.toFixed(0)} Hz / 限速 ${cfg.slewDegPerSec.toFixed(0)}°/s / sync=${cfg.sync}`);
  const seconds = a.sec === undefined ? null : Number(a.sec);
  const tp = new TeleopSession(lead, foll, { cfg });
  try {
    await tp.start();
    await tp.run({ durationS: seconds });
  } catch (e) {
    if (e instanceof TeleopLinkLost) { console.log(c.bad(e.message)); return 3; }
    if (e instanceof TeleopAbort) { console.log('🛑 ' + e.message); return 4; }
    throw e;
  } finally {
    await tp.stop();
    await lead.close(); await foll.close();
  }
  console.log(tp.stats.summary());
  return 0;
}

async function cmdRecord(a) {
  if (!a.leader || !a.follower || !a.out) {
    die('用法：record --leader <uri> --follower <uri> --out <file> [--sec N]');
  }
  const lead = await openFromArgs(a, a.leader);
  const foll = await openFromArgs(a, a.follower);
  const cfg = new TeleopConfig({
    hz: Number(a.hz ?? 50),
    slewDegPerSec: Number(a.slew ?? 240),
    sync: a.sync || 'hold',
    verbose: true,
  });
  console.log(`遥操示教：录制到 ${a.out}  ${cfg.hz.toFixed(0)} Hz for ${a.sec ?? 30}s（PC 侧无 80s 上限）`);
  const ts = new TeleopTeachSession(lead, foll, {
    out: a.out, name: a.out.split(/[\\/]/).pop(), hz: cfg.hz, cfg,
  });
  let traj;
  try {
    traj = await ts.record({ durationS: Number(a.sec ?? 30), out: a.out });
  } catch (e) {
    if (e instanceof TeleopLinkLost) { console.log(c.bad(e.message)); return 3; }
    if (e instanceof TeleopAbort) { console.log('🛑 ' + e.message); return 4; }
    throw e;
  } finally {
    await lead.close(); await foll.close();
  }
  const v = traj.validate();
  console.log(c.ok(traj.summary()));
  if (!v.ok) console.log('  ' + c.warn('回放前建议先修：' + JSON.stringify(v.warnings)));
  console.log('  已存 ' + a.out);
  return 0;
}

async function cmdPlay(a) {
  if (!a._.length) die('用法：play <轨迹文件> [--speed 1] [--loop 1] [--force] [--no-approach]');
  const traj = await loadTrajectory(a._[0]);
  const v = traj.validate();
  console.log('轨迹 ' + traj.summary());
  if (!v.ok) {
    console.log('  ' + c.warn(JSON.stringify(v.warnings)));
    if (!a.force) die('有警告，确认无误请加 --force', 1);
  }
  if (a.convert) {
    const out = await saveTrajectory(traj, a.convert);
    console.log('  已导出 ' + out);
    if (a['no-play']) return 0;
  }
  const arm = await openFromArgs(a);
  try {
    await prelude(arm);
    const pb = new PlaybackSession(arm, {
      cfg: new TeleopConfig({ hz: Number(a.hz ?? 50) }),
      estopCheckS: 0.3,
      dryRun: !!a['dry-run'],
      approach: !a['no-approach'],
      approachSpeed: Number(a['approach-speed'] ?? 20),
      slewDegPerSec: Number(a.slew ?? 0),
      onEvent: (m) => console.log('  ' + m),
    });
    console.log(`  回放 ${Number(a.hz ?? 50).toFixed(0)} Hz speed=${Number(a.speed ?? 1)} ` +
                `loop=${Number(a.loop ?? 1)}（单帧步长硬限 20°）`);
    const st = await pb.play(traj, {
      speed: Number(a.speed ?? 1),
      loop: Number(a.loop ?? 1),
      approach: !a['no-approach'],
      approachSpeed: Number(a['approach-speed'] ?? 20),
    });
    console.log('  ' + st.summary());
    if (st.ackLimit) console.log('  ' + c.warn(`有 ${st.ackLimit} 帧被固件拒收(err=4)。` +
      '常见原因：起点未接近（--no-approach 关了接近段）/ 单帧跳变 >25° / 从臂 sf 0 关过跟随'));
  } finally { await arm.close(); }
  return 0;
}

async function cmdTraj(a) {
  const action = a._[0];
  const file = a._[1];
  if (!action || !file) die('用法：traj <info|convert|resample|smooth> <file> [out]');
  const t = await loadTrajectory(file);
  if (action === 'info') {
    const v = t.validate();
    console.log(`name=${t.name}  points=${t.length}  duration=${t.duration.toFixed(3)}s  ` +
                `rate=${t.rate.toFixed(2)}Hz  units=${t.units}  source=${t.source}`);
    console.log('每轴单帧最大跳变 ' + JSON.stringify(t.maxStep().map((x) => Math.round(x * 100) / 100)) +
                '（>25° 会被固件整帧拒收）');
    console.log('warnings: ' + (v.warnings && v.warnings.length ? JSON.stringify(v.warnings) : '无'));
    console.log('首帧 ' + fmtQ(t.startPose) + '\n末帧 ' + fmtQ(t.endPose));
  } else if (action === 'convert') {
    const out = await saveTrajectory(t, a._[2] || a.out);
    console.log(`已导出 ${out}（${t.length} 点）`);
  } else if (action === 'resample') {
    const r = t.resample(Number(a.hz ?? 50));
    const out = await saveTrajectory(r, a._[2] || a.out);
    console.log(`已重采样到 ${Number(a.hz ?? 50).toFixed(1)} Hz → ${out}（${r.length} 点）`);
  } else if (action === 'smooth') {
    const s = t.smooth(Number(a.window ?? 5));
    const out = await saveTrajectory(s, a._[2] || a.out);
    console.log(`已平滑 window=${a.window ?? 5} → ${out}（${s.length} 点）`);
  } else die('未知 action: ' + action);
  return 0;
}

/* ==========================================================================
 * 2026-09-23 扩展：文本透传 / 诊断
 * ========================================================================== */

/** 网口下发**任意一条控制台命令**（等价在 COM11 上敲）。
 *
 * ⚠ 这是"所有指令都支持网口下发"的入口：124 条里 113 条可用，11 条抢 ETH
 *   引脚的命令会被固件接住并把原因当输出回给你，`c` 会被安全闸门拒。
 */
async function cmdText(a) {
  const cmds = a.cmd ? (Array.isArray(a.cmd) ? a.cmd : [a.cmd]) : [];
  if (!cmds.length) die('text 需要至少一个 --cmd <命令>（可重复）');
  const arm = await openFromArgs(a);
  let rc = 0;
  try {
    for (const cmd of cmds) {
      console.log('$ ' + cmd);
      let out;
      try { out = await arm.textExec(cmd); }
      catch (e) { console.log('  ' + c.bad(e.message)); rc = 2; continue; }
      process.stdout.write(out.endsWith('\n') || out.endsWith('\r')
        ? out : out + '\n');
    }
  } finally { await arm.close(); }
  return rc;
}

/** 一趟把设备自述读全：固件 tag / 构建 / 链路 / 状态位 / 校准记录。 */
async function cmdDiag(a) {
  const arm = await openFromArgs(a);
  try {
    const inf = await arm.getInfo();
    console.log('  固件 tag    ' + inf.fw_tag);
    console.log('  构建        ' + inf.build_name + ' (build=' + inf.build + ')');
    console.log('  运行链路    ' + inf.link_mode + '  ⚠ 启动冻结值，不是 Flash 偏好');
    console.log('  参数条目    ' + inf.n_params);

    const fl = await arm.getFlags();
    console.log('  状态位      raw=0x' + fl.raw.toString(16) +
                ` estop=${fl.estop} traj=${fl.traj} stream=${fl.stream}` +
                ` run_id=0x${fl.run_id.toString(16)}`);

    const lm = await arm.getLimits();
    console.log('  raw 限位 / Home 零点（Flash 校准记录）:');
    for (let j = 0; j < 6; j++) {
      console.log(`    J${j + 1}  min=${String(lm.min_raw[j]).padStart(4)}` +
                  `  pos=${String(lm.pos_raw[j]).padStart(4)}` +
                  `  max=${String(lm.max_raw[j]).padStart(4)}` +
                  `  zero=${String(lm.zero_raw[j]).padStart(4)}` +
                  (lm.calibrated[j] ? '' : '   ⚠ 未校准（量纲基准脆）'));
    }

    if (a.detail) {
      const jd = await arm.getJointDetail(
        a.joint !== undefined ? Number(a.joint) : 0xFF);
      console.log('  关节遥测（⚠ 总线逐字段瞬时丢读，单次 0 不等于真值）:');
      for (const d of jd) {
        console.log(`    J${d.joint + 1}  pos=${String(d.pos_raw).padStart(4)}` +
                    `  ${d.voltage.toFixed(1)}V` +
                    `  ${d.temp_lost ? 'temp=--(丢读)' : d.temp + '°C'}` +
                    `  hw_err=${d.hw_err_lost ? '--' : d.hw_err}` +
                    (d.ok ? '' : '   ⚠ 本轴整组读失败'));
      }
    }
    return 0;
  } finally { await arm.close(); }
}

function cmdSerialOnly(name) {
  console.log(c.warn(`\`${name}\` 需要串口（文本控制台 / #P 中转）。`));
  console.log('  Node 里没有 Web Serial，也没有内置 serialport。两条路：');
  console.log(`    · 浏览器：用 SDK 的 console.js / FirmwareLink（${name === 'relay' ? 'PcRelay' : 'ArmSDK'}），WebSocket 桥转发；`);
  console.log(`    · 或直接用 Python 版：armctl ${name} ...`);
  return 2;
}

/* ==========================================================================
 * 帮助 / 主入口
 * ========================================================================== */

const HELP = `armctl-js —— 六轴机械臂上位机 CLI（Node，零依赖，网口优先）

用法: armctl-js [全局选项] <命令> [命令选项]

全局选项:
  --uri <uri>       承载（缺省 ${DEFAULT_URI}，也可用环境变量 ARM_URI）
                    可选 net://<ip>:5000 | ws://<host>:8765 | webserial:// | sim://
  --timeout <秒>    单命令超时（缺省 0.3）
  --dry-run         只打印不发送（最底层安全开关）
  -v, --verbose     打印链路事件

命令:
  info                             能力 / 链路 / 配置摘要（离线）
  selftest                         跑全部离线自检
  bridge [--target ip:port] [--port] [--allow] [--selftest]
                                   起 WebSocket↔UDP 边车（给浏览器用）
  preflight                        上板前置健康检查
  ping [--repeat N]                PING（兼测往返延迟）
  status [--json] [--watch N]      GET_STATUS
  fk                               FK_QUERY 正解位姿
  move <6角> [--delta] [--v] [--a] [--wait]   MOVE_JOINTS（舵机度）
  cart <x> <y> <z> [--pitch] [--v] [--a]      MOVE_CART（⚠ 归零 J5/J6）
  stop                             STOP 平滑停止
  torque [--joint N] [--off]       SET_TORQUE
  stream [--pose 6角] [--hz] [--sec] [--slew] STREAM_JOINTS 限速走到目标并保持
  params <list|get|set|count> [id] [value]    运行时参数
  teleop --leader <uri> --follower <uri> [--hz] [--sec] [--slew] [--gain] [--sync]
  record --leader <uri> --follower <uri> --out <file> [--sec] [--hz]
  play <file> [--speed] [--loop] [--force] [--no-approach] [--convert]
  traj <info|convert|resample|smooth> <file> [out] [--hz] [--window]
  text --cmd <命令> [--cmd ...]    在臂上执行控制台命令（自动翻页，等价 COM11）
  diag [--detail] [--joint N]      固件自述 / 状态位 / 限位 / 关节遥测
  console|link|relay               串口相关（见提示，建议用 Python 版或浏览器）

例子:
  armctl-js info
  armctl-js --uri net://192.168.7.120:5000 ping --repeat 20
  armctl-js --uri net://192.168.7.120:5000 status --json
  armctl-js move --delta 0,5,0,0,0,0 --v 10 --wait
  armctl-js teleop --leader net://192.168.7.121:5000 --follower net://192.168.7.120:5000 --sec 30
  armctl-js bridge --target 192.168.7.120:5000 --port 8765
  armctl-js --uri net://192.168.7.120:5000 text --cmd s --cmd zi
  armctl-js --uri net://192.168.7.120:5000 diag --detail
`;

const GLOBAL_SPEC = {
  alias: { v: 'verbose', t: 'timeout' },
  bools: ['dry-run', 'verbose', 'json', 'help', 'version', 'delta', 'wait',
          'off', 'force', 'no-play', 'no-approach', 'selftest', 'detail'],
  multi: ['cmd'],
};

export async function main(argv = process.argv.slice(2)) {
  const a = parseArgv(argv, GLOBAL_SPEC);
  const cmd = a._[0];
  if (a.version || cmd === 'version') { console.log('arm-sdk-js ' + VERSION); return 0; }
  if (!cmd || a.help || cmd === 'help') { console.log(HELP); return 0; }

  const rest = { ...a, _: a._.slice(1) };
  switch (cmd) {
    case 'info': return cmdInfo();
    case 'selftest': return cmdSelftest();
    case 'bridge': return cmdBridge(rest);
    case 'preflight': return cmdPreflight(rest);
    case 'ping': return cmdPing(rest);
    case 'status': return cmdStatus(rest);
    case 'fk': return cmdFk(rest);
    case 'move': return cmdMove(rest);
    case 'cart': return cmdCart(rest);
    case 'stop': return cmdStop(rest);
    case 'torque': return cmdTorque(rest);
    case 'stream': return cmdStream(rest);
    case 'params': return cmdParams(rest);
    case 'teleop': return cmdTeleop(rest);
    case 'record': return cmdRecord(rest);
    case 'play': return cmdPlay(rest);
    case 'traj': return cmdTraj(rest);
    case 'text': case 'exec': return cmdText(rest);
    case 'diag': return cmdDiag(rest);
    case 'console': case 'link': case 'relay': return cmdSerialOnly(cmd);
    default:
      process.stderr.write(`armctl-js: 未知命令 ${JSON.stringify(cmd)}\n\n` + HELP);
      return 2;
  }
}

const isMain = process.argv[1] && fileURLToPath(import.meta.url) === process.argv[1];
if (isMain) {
  // 被 `| head` / `| grep` 提前关闭管道时别抛 EPIPE 崩掉
  process.stdout.on('error', (e) => { if (e && e.code === 'EPIPE') process.exit(0); });
  main()
    .then((rc) => process.exit(rc ?? 0))
    .catch((e) => {
      if (e && e.exitCode) { process.exit(e.exitCode); }
      if (process.env.ARMCTL_TRACE) throw e;
      process.stderr.write(`armctl-js: ${e.name}: ${e.message}\n`);
      process.exit(1);
    });
}
