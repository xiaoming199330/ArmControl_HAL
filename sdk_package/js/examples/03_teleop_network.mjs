/**
 * 03_teleop_network.mjs — 主从遥操（网口，p50 1.61ms）
 *
 * 真机跑法：
 *     # 主臂
 *     node examples/03_teleop_network.mjs --leader net://192.168.7.121:5000 \
 *                                         --follower net://192.168.7.120:5000 --sec 30
 *     # 没有真臂也能跑（两条 sim 臂）：
 *     node examples/03_teleop_network.mjs
 *
 * ⚠ 硬约束（照抄工程红线）：
 *   · 从臂 `sf` 默认是 1；若你之前关过跟随，先 `sf 1` 再遥操。
 *   · 网络路线用 STREAM_JOINTS（逻辑角）；不要用 MOVE_CART（会归零 J5/J6）。
 *   · 丢控后接收机会继续输出预设行程 → X/Y/Z 的 F/S 必须设中位，否则飞车。
 */

import { openArm } from '../src/client.js';
import { TeleopConfig, TeleopSession, TeleopAbort, TeleopLinkLost } from '../src/teleop.js';
import { arg, flag } from './_args.mjs';

const leader = arg('leader', 'sim://');
const follower = arg('follower', 'sim://');
const sec = arg('sec', null);
const hz = Number(arg('hz', 50));
const slew = Number(arg('slew', 240));
const gain = Number(arg('gain', 1));
const sync = arg('sync', 'hold');
const softLimit = arg('soft-limit', null);       // 如 20 → 6 轴各 ±20°

const cfg = new TeleopConfig({
  hz, slewDegPerSec: slew, gain, sync, waitAck: false, verbose: flag('verbose'),
});
if (softLimit !== null) {
  const v = Math.abs(Number(softLimit));
  cfg.softLimitsDeg = Array.from({ length: 6 }, () => [-v, v]);
}
cfg.estopTrips = true;

console.log(`主从遥操  leader=${leader} → follower=${follower}`);
console.log(`  ${hz} Hz / 每轴限速 ${slew}°/s / gain=${gain} / sync=${sync}` +
            (softLimit !== null ? ` / 软限位 ±${softLimit}°` : ''));
console.log('  按 Ctrl-C 停。');

const [lead, foll] = await Promise.all([
  openArm(leader, { timeout: 0.3 }),
  openArm(follower, { timeout: 0.3 }),
]);

const tp = new TeleopSession(lead, foll, {
  cfg,
  onEvent: (m) => console.log('  · ' + m),
});

const stop = () => { tp.requestStop(); };
process.on('SIGINT', stop);

try {
  await tp.start();
  await tp.run({ durationS: sec === null ? undefined : Number(sec) });
} catch (e) {
  if (e instanceof TeleopLinkLost) console.log('✗ 丢控：' + e.message);
  else if (e instanceof TeleopAbort) console.log('🛑 中止：' + e.message);
  else throw e;
} finally {
  await tp.stop();
  await lead.close();
  await foll.close();
}

console.log('\n' + tp.stats.summary());
console.log('提示：从臂侧 `lk` 看流模式统计；`sf 0` 可关跟随。');
