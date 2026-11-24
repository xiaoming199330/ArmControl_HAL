/**
 * 01_quickstart.mjs — 最短路径：连上、读状态、点动、收工
 *
 *     node examples/01_quickstart.mjs                        # 默认 sim://，无需真臂
 *     node examples/01_quickstart.mjs --uri net://192.168.7.120:5000
 *
 * ⚠ 点动**必须**用增量：绝对角会被固件 `clamp_all_angles()` 静默钳制 ——
 *   你发 320，臂停在 300，err 还是 0。这里用 read-modify-write 手工算增量。
 */

import { openArm } from '../src/index.js';
import { arg, fmtQ } from './_args.mjs';

const uri = arg('uri', 'sim://');
const arm = await openArm(uri, { timeout: 0.5 });
console.log('已连接：' + arm.link + '   ' + arm.describe());

// —— 1. PING ——
const pi = await arm.ping();
console.log(`PING  proto_ver=${pi.proto_ver} run_id=${pi.run_id}  ${pi.latencyMs.toFixed(1)}ms`);

// —— 2. 读状态（先读再动）——
let st = await arm.getStatus();
console.log('当前舵机度 ' + fmtQ(st.joints) + `  read_ok=${st.read_ok}` + (st.estop ? '  🛑E-STOP' : ''));
if (st.failedJoints.length) {
  console.log('⚠ 读失败的轴 J' + st.failedJoints.join(',J') + ' —— 先排总线再运动');
}

// —— 3. 点动 J2 +5°（增量 = 当前 + 5）——
const target = st.joints.slice();
target[1] += 5;
console.log('点动 J2：' + fmtQ(st.joints) + ' → ' + fmtQ(target));
console.log('  ' + (await arm.moveJoints(target, 10 /* deg/s */)).toString());

// —— 4. 到位检查（err=0 不代表到位，必须回读角度）——
await new Promise((r) => setTimeout(r, 400));
st = await arm.getStatus();
console.log('到位后 ' + fmtQ(st.joints));

// —— 5. 正解位姿 ——
const fk = await arm.fkQuery();
console.log(`FK  X=${fk.x.toFixed(1)} Y=${fk.y.toFixed(1)} Z=${fk.z.toFixed(1)} pitch=${fk.pitch.toFixed(1)}`);

await arm.close();
console.log('收工。');
