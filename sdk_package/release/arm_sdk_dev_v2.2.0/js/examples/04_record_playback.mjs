/**
 * 04_record_playback.mjs — 遥操示教：边遥操边录制，然后回放
 *
 *     node examples/04_record_playback.mjs                     # sim 臂，练 API
 *     node examples/04_record_playback.mjs \
 *          --leader net://192.168.7.121:5000 --follower net://192.168.7.120:5000 \
 *          --sec 30 --out demo.json
 *
 * 关键点：
 *   · 录的是**主臂逻辑角**（人手的动作），回放走 STREAM_JOINTS（逻辑角）。
 *   · PC 侧录制**没有固件那条 80s 上限**（那是 `teach` 命令的限制）。
 *   · 回放前先 `validate()`：单帧跳变 > 25° 会被固件整帧拒收，先重采样/平滑。
 *   · 回放默认走"接近段"：先低速把从臂开到轨迹起点，避免第一帧就大跳变。
 */

import { openArm } from '../src/client.js';
import { TeleopConfig, TeleopAbort, TeleopLinkLost } from '../src/teleop.js';
import { TeleopTeachSession, PlaybackSession, Trajectory } from '../src/teach.js';
import { arg, flag } from './_args.mjs';

const leader = arg('leader', 'sim://');
const follower = arg('follower', 'sim://');
const out = arg('out', 'demo_traj.json');
const sec = Number(arg('sec', 5));
const hz = Number(arg('hz', 50));

const [lead, foll] = await Promise.all([
  openArm(leader, { timeout: 0.3 }),
  openArm(follower, { timeout: 0.3 }),
]);

/* ---------------- 录制 ---------------- */
const cfg = new TeleopConfig({ hz, slewDegPerSec: 240, sync: 'hold', verbose: flag('verbose') });
const ts = new TeleopTeachSession(lead, foll, {
  out, name: out, hz, cfg,
  onEvent: (m) => console.log('  · ' + m),
});

console.log(`录制中… ${sec}s → ${out}（真机时请用手拖动主臂）`);
let traj;
try {
  traj = await ts.record({ durationS: sec, out });
} catch (e) {
  if (e instanceof TeleopLinkLost || e instanceof TeleopAbort) {
    console.log('中断：' + e.message);
    traj = ts.lastTraj;
  } else throw e;
}

if (traj) {
  console.log('\n' + traj.summary());
  const v = traj.validate();
  console.log('体检：' + (v.ok ? '通过' : JSON.stringify(v.warnings)));
  console.log(`已存 ${out}（${traj.length} 点，units=${traj.units}）`);
}

/* ---------------- 回放 ---------------- */
const loaded = traj || await Trajectory.load(out);
const v = loaded.validate();
if (!v.ok) {
  console.log('\n⚠ 轨迹有告警：' + JSON.stringify(v.warnings));
  console.log('  真实场景请先 resample/smooth 再回放；此处演示继续。');
}

const pb = new PlaybackSession(foll, {
  cfg: new TeleopConfig({ hz }),
  slewDegPerSec: 60,
  estopCheckS: 0.3,
  approach: true,
  approachSpeed: 20,
  onEvent: (m) => console.log('  · ' + m),
});

console.log('\n回放中…');
const stats = await pb.play(loaded, { speed: 1.0, loop: 1, approach: true, approachSpeed: 20 });
console.log(stats.summary());
if (stats.ackLimit) console.log('⚠ 有帧被拒收（err=4）：查接近段是否收敛 / 单帧跳变是否 >25°');

await lead.close();
await foll.close();
console.log('\n收工。轨迹文件：' + out);
