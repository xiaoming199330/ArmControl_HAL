/**
 * run_all.mjs — JS SDK 全量离线自检入口（`npm test`）
 *
 * 分两层：
 *   · 库内自检：直接调 `selftest()`（protocol / sim / bridge），最快，且能拿到精确计数
 *   · 脚本级自检：子进程跑 test/*.mjs（跨语言对拍 / 旧版回归 / 端到端 / 真 UDP 回环）
 *
 * 任何一项失败 → 退出码 1。全绿 → 0（可直接接 CI）。
 */

import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import process from 'node:process';

import { selftest as protocolSelftest } from '../src/protocol.js';
import { selftest as simSelftest } from '../src/sim.js';
import { selftest as bridgeSelftest } from '../src/bridge.js';

const HERE = fileURLToPath(new URL('.', import.meta.url));
const ROOT = fileURLToPath(new URL('..', import.meta.url));

const results = [];
function record(name, ok, detail = '') {
  results.push({ name, ok, detail });
  const mark = ok ? '✓' : '✗';
  console.log(`\n${'═'.repeat(72)}\n${mark} ${name}${detail ? '  —  ' + detail : ''}\n${'═'.repeat(72)}`);
}

/* --------------------------------------------------------------------------
 * 1. 库内自检
 * ------------------------------------------------------------------------ */

console.log('\n【1/2】库内自检（直接调用，无进程开销）');

process.stdout.write('\n>>> protocol.js 编解码自检\n');
let rc = 1;
try { rc = protocolSelftest(); } catch (e) { console.log('  FAIL 抛异常: ' + e.stack); rc = 1; }
record('protocol.js 编解码自检', rc === 0, rc === 0 ? '' : '见上方输出');

process.stdout.write('\n>>> sim.js 离线假臂自检\n');
try { rc = await simSelftest(); } catch (e) { console.log('  FAIL 抛异常: ' + e.stack); rc = 1; }
record('sim.js 离线假臂自检', rc === 0, rc === 0 ? '' : '见上方输出');

process.stdout.write('\n>>> bridge.js 边车自检\n');
let br = { ok: false, pass: 0, fail: ['抛异常'] };
try { br = await bridgeSelftest(); } catch (e) { console.log('  FAIL 抛异常: ' + e.stack); }
record('bridge.js 边车自检', br.ok, `${br.pass ?? 0} 项通过` + (br.skipped ? `（${br.skipped}）` : ''));

/* --------------------------------------------------------------------------
 * 2. 脚本级自检（子进程）
 * ------------------------------------------------------------------------ */

console.log('\n【2/2】脚本级自检（子进程）');

const SCRIPTS = [
  ['cross_python.mjs', '跨语言逐字节对拍（JS ↔ Python 向量）'],
  ['legacy_parity.mjs', '旧版 web/arm_sdk.js 回归（225 项）'],
  ['integration.mjs', '端到端离线集成（client/teleop/teach）'],
  ['udp_loopback.mjs', '真实 UDP socket 回环（假固件）'],
];

for (const [file, label] of SCRIPTS) {
  console.log(`\n>>> node test/${file}`);
  const r = spawnSync(process.execPath, [file], { cwd: HERE, stdio: 'inherit' });
  const ok = r.status === 0;
  record(label, ok, ok ? '' : `退出码 ${r.status}${r.signal ? ' 信号 ' + r.signal : ''}`);
}

/* --------------------------------------------------------------------------
 * 汇总
 * ------------------------------------------------------------------------ */

const failed = results.filter((r) => !r.ok);
console.log('\n' + '─'.repeat(72));
console.log(`JS SDK 自检汇总：${results.length - failed.length} / ${results.length} 组通过`);
for (const r of results) console.log(`  ${r.ok ? '✓' : '✗'} ${r.name}${r.detail ? '   ' + r.detail : ''}`);
console.log('─'.repeat(72));
console.log(failed.length ? '✗ 有失败项，见上方各组输出' : '✓ 全部通过');
void ROOT;

process.exit(failed.length ? 1 : 0);
