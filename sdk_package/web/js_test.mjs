// js_test.mjs — 对 web/arm_sdk.js 的纯函数做冒烟测试（Node 运行，不需要浏览器）
// 用法: node js_test.mjs
import * as A from './arm_sdk.js';

let fail = 0;
const chk = (c, m, x) => { if (!c) fail++; console.log('  [' + (c ? 'OK ' : 'FAIL') + '] ' + m + (x ? '   ' + x : '')); };

console.log('=== 常量 ===');
chk(Math.abs(A.ARM_MAX_REACH - 418.80) < 1e-9, 'ARM_MAX_REACH = 418.80', A.ARM_MAX_REACH);
chk(Math.abs(A.ROBOT_L_DISTAL - 167.80) < 1e-9, 'ROBOT_L_DISTAL = 167.80', A.ROBOT_L_DISTAL);
chk(Math.abs(A.ROBOT_PSI_UPPER0 * A.R2D - 13.97) < 0.01, 'PSI_UPPER0 = 13.97 deg');
chk(Math.abs(A.ROBOT_PSI_FORE0 * A.R2D - 87.79) < 0.01, 'PSI_FORE0 = 87.79 deg');

console.log('=== FK ===');
const p0 = A.fkFromJointDeg([0, 0, 0, 0, 0, 0]);
chk(Math.abs(p0.x - 361.895) < 0.01 && Math.abs(p0.z - 244.333) < 0.01,
  '零位 TCP (361.895, 0, 244.333)', 'x=' + p0.x.toFixed(3) + ' z=' + p0.z.toFixed(3));
const ph = A.fkFromJointDeg(A.POSE_HOME_J);
chk(Math.abs(A.poseRho(ph) - 131.39) < 0.02 && Math.abs(ph.z - 3.98) < 0.02 &&
  Math.abs(A.posePitchDeg(ph) - 150.03) < 0.02,
  '折叠位对拍 (rho=131.39 z=3.98 pitch=150.03)',
  'rho=' + A.poseRho(ph).toFixed(2) + ' z=' + ph.z.toFixed(2) + ' pitch=' + A.posePitchDeg(ph).toFixed(2));

console.log('=== 角度/raw 换算 ===');
chk(A.degToServoRaw(180) === 2048, 'degToServoRaw(180) = 2048', A.degToServoRaw(180));
// ⚠ 固件固有量化：4095 分辨率的中点是 2047.5，raw 2048 = 180.04396°
chk(Math.abs(A.servoRawToDeg(2048) - 180.04396) < 1e-4,
  'servoRawToDeg(2048) = 180.044 (固件量化, 非 180.000)', A.servoRawToDeg(2048).toFixed(5));
let rtOk = true;
for (let j = 0; j < 6; j++) {
  const b = A.radToServoDeg(A.servoDegToRad(200, j), j);
  if (Math.abs(b - 200) > 1e-9) rtOk = false;
}
chk(rtOk, '舵机度<->关节角 往返一致');

console.log('=== raw 限位判定 ===');
chk(A.rawInRange(2048, 0, 4095) === true, 'rawInRange(2048,0,4095) = true');
chk(A.rawInRange(2048, 0, 0) === false, '未校准 (0,0) -> false (固件会拒一切运动)');
chk(A.rawInRange(10, 4000, 200) === true, '跨 0/4096 包裹 -> true');
chk(A.rawInRange(3000, 4000, 200) === false, '跨 0/4096 包裹 外 -> false');

console.log('=== IK 往返精度 ===');
// ⚠ 用真正"全开放"限位(±180°)，样本才不会被 J1 钳制
//   —— J1 被钳制是限位行为(固件同样静默钳制)，不是 IK 算法误差
const BIG = Math.PI;
const big = new A.JointLimits(new Array(6).fill(-BIG), new Array(6).fill(BIG));
let nOk = 0, nAna = 0, worstAna = 0, worstCcd = 0;
for (let i = 0; i < 300; i++) {
  const jd = [Math.random() * 350 - 175, Math.random() * 320 - 160,
    Math.random() * 320 - 160, Math.random() * 320 - 160, 0, 0];
  const p = A.fkFromJointDeg(jd);
  const r = A.ik({ x: p.x, y: p.y, z: p.z, pitch: p.pitch }, big);
  if (r.ok) {
    nOk++;
    const p2 = A.fk(r.jointsRad);
    const e = Math.hypot(p.x - p2.x, p.y - p2.y, p.z - p2.z);
    if (r.method === 'analytic') { nAna++; worstAna = Math.max(worstAna, e); }
    else worstCcd = Math.max(worstCcd, e);
  }
}
chk(nOk === 300, 'IK 全部可解 ' + nOk + '/300 (解析解 ' + nAna + ' / CCD ' + (300 - nAna) + ')');
chk(worstAna < 1e-6, '解析解往返误差为数值精度级: ' + worstAna.toExponential(2) + ' mm');
chk(worstCcd <= A.KIN_IK_FAIL_MM,
  'CCD 兜底最差误差 ' + worstCcd.toFixed(4) + ' mm <= 固件阈值 ' + A.KIN_IK_FAIL_MM);

console.log('=== ikChecked 闸门 ===');
const minRaw = [1600, 0, 0, 0, 0, 0], maxRaw = [3400, 4095, 4095, 4095, 4095, 4095];
const lim = new A.JointLimits();
lim.setFromRaw(minRaw, maxRaw);
const c1 = A.ikChecked(300, 0, 200, 90, { minRaw, maxRaw, lim });
chk(c1.ok, '正前方 (300,0,200) 接受', 'reason=' + c1.reason);
const c2 = A.ikChecked(900, 0, 200, 90, { minRaw, maxRaw, lim });
chk(!c2.ok && c2.reason === 'unreachable', '超臂展 -> unreachable', 'reason=' + c2.reason);
const c3 = A.ikChecked(-300, 0, 200, 90, { minRaw, maxRaw, lim });
chk(!c3.ok && c3.reason === 'fk_verify', '方位角越 J1 限位 -> fk_verify (静默钳制陷阱)',
  'reason=' + c3.reason + ' posErr=' + c3.posErrMm.toFixed(1) + 'mm');

console.log('=== CRC / 组帧 对称性 ===');
const f = A.buildFrame(7, A.CMD.PING);
chk(f[0] === 0xAA && f[1] === 0x55 && f.length === 7, '帧 = AA 55 SEQ CMD LEN CRC_LO CRC_HI',
  Array.from(f).map(b => b.toString(16).padStart(2, '0')).join(' '));
// ⭐ 跨语言一致性：Python 侧同一输入产出 'aa 55 07 01 00 3d 7a'
chk(Array.from(f).map(b => b.toString(16).padStart(2, '0')).join(' ') === 'aa 55 07 01 00 3d 7a',
  'JS 组帧与 Python 实现逐字节一致（aa 55 07 01 00 3d 7a）');
const pr = new A.FrameParser();
let got = null;
for (const b of f) { const r = pr.feed(b, 1000); if (r && r.type === 'frame') got = r; }
chk(got && got.seq === 7 && got.cmd === A.CMD.PING, '自组帧被自解析器解回');
// CRC 已知向量：CRC-16/CCITT-FALSE("123456789") = 0x29B1
const vec = new TextEncoder().encode('123456789');
chk(A.crc16(vec) === 0x29B1, 'CRC-16/CCITT-FALSE 标准向量 "123456789" = 0x29B1',
  '0x' + A.crc16(vec).toString(16).toUpperCase());
// CRC 错帧要能被识别
const bad = f.slice(); bad[bad.length - 1] ^= 0xFF;
const pr3 = new A.FrameParser();
let crcErr = null;
for (const b of bad) { const r = pr3.feed(b, 1000); if (r && r.type === 'crc_err') crcErr = r; }
chk(crcErr !== null, 'CRC 损坏的帧被判 crc_err');
// MOVE_JOINTS 帧跨语言一致性（Python: aa 55 01 10 10 08 07 e5 05 ca 08 7f 06 08 07 08 07 2c 01 00 00 47 f6）
const mv = A.buildFrame(1, A.CMD.MOVE_JOINTS, (() => {
  const pl = new Uint8Array(16); let o = 0;
  for (const v of [180, 150.9, 225, 166.3, 180, 180]) { pl.set(A.i16Bytes(Math.round(v * 10)), o); o += 2; }
  pl.set(A.u16Bytes(300), o); o += 2; pl.set(A.u16Bytes(0), o);
  return pl;
})());
chk(Array.from(mv).map(b => b.toString(16).padStart(2, '0')).join(' ')
  === 'aa 55 01 10 10 08 07 e5 05 ca 08 7f 06 08 07 08 07 2c 01 00 00 47 f6',
  'MOVE_JOINTS 帧 23 字节，与 Python 逐字节一致');

console.log('=== 文本 / 二进制 混流分流 ===');
const pr2 = new A.FrameParser(); const tb = [];
for (const ch of 'p 0\r\n') {
  const r = pr2.feed(ch.charCodeAt(0), 1000);
  if (r && r.type === 'text') tb.push(String.fromCharCode(r.byte));
}
chk(tb.join('') === 'p 0\r\n', '文本字节原样交还文本通道（天然分流）', JSON.stringify(tb.join('')));
// 文本中夹一帧：帧被吃掉，文本保持完整
const pr4 = new A.FrameParser(); const tb2 = [];
for (const ch of 'ab') { const r = pr4.feed(ch.charCodeAt(0), 1000); if (r && r.type === 'text') tb2.push(String.fromCharCode(r.byte)); }
let midFrame = null;
for (const b of A.buildFrame(9, A.CMD.PING)) { const r = pr4.feed(b, 1000); if (r && r.type === 'frame') midFrame = r; }
for (const ch of 'cd') { const r = pr4.feed(ch.charCodeAt(0), 1000); if (r && r.type === 'text') tb2.push(String.fromCharCode(r.byte)); }
chk(tb2.join('') === 'abcd' && midFrame && midFrame.seq === 9,
  '文本夹二进制帧：帧被分离、文本不丢字', JSON.stringify(tb2.join('')));

console.log('=== 解析器 ===');
const st = A.parseStatus(['  | J1 | 1  | 123.4 |   Yes    |  35C |']);
chk(st.pos[0] === 123.4 && st.moving[0] === true && st.temp[0] === 35, 'parseStatus', JSON.stringify(st));
const jr = A.parseJoint(['  J1: raw=2048 servo=180.04 deg  rel-home=+0.04 deg']);
chk(jr.raw === 2048 && Math.abs(jr.jointDeg - 0.04) < 1e-9, 'parseJoint', JSON.stringify(jr));
chk(A.parseFk(['  FK: x=131 y=0 z=3 mm  pitch=150 deg']) !== null, 'parseFk');
chk(A.parseFk(['  servo  : J1=1.00']) === null, 'parseFk 读失败 -> null（禁止兜底）');
const pc = A.parsePidCompare(['  J3 cache PID = P:16 I:4 D:12',
  '  J3 servo EEPROM = P:16 D:12 I:4  (一致)']);
chk(JSON.stringify(pc.cache) === JSON.stringify([16, 4, 12]) &&
  JSON.stringify(pc.eeprom) === JSON.stringify([16, 4, 12]) && pc.match === true,
  'parsePidCompare（EEPROM 行是 P D I -> 归一化成 P I D）', JSON.stringify(pc));
const pc2 = A.parsePidCompare(['  J2 cache PID = P:32 I:4 D:12',
  '  J2 servo EEPROM = P:16 D:12 I:4  (不一致! 需 Y 烧写)']);
chk(pc2.match === false && pc2.eeprom[0] === 16, 'parsePidCompare 识别"不一致"', JSON.stringify(pc2));
const ps = A.parseParams(['  # 1 ik.converge_mm   +0.500 [ +0.100 ~ +5.000] mm']);
chk(ps.length === 1 && ps[0].id === 1 && ps[0].value === 0.5 && ps[0].unit === 'mm', 'parseParams', JSON.stringify(ps));
const lm = A.parseLimits(['  | J1 |  4000 |  200 |  100 |']);
chk(lm.minDeg[0] === 4000 && lm.maxDeg[0] === 200, 'parseLimits（MIN>MAX 是合法包裹区间）', JSON.stringify(lm));
const tm = A.parseTmLine('TM p:  180.0  150.9   --  180.0  180.0  180.0');
chk(tm && tm.key === 'p' && tm.values.length === 6 && tm.values[2] === null, 'parseTmLine（含 --）', JSON.stringify(tm));
const tm2 = A.parseTmLine('TM i:   12   12   12   12   12   12   (mA)');
chk(tm2 && tm2.values.length === 6 && tm2.values[0] === 12, 'parseTmLine 去掉 (mA) 单位后缀', JSON.stringify(tm2));
chk(A.parseTmLine('  | J1 | 1 |') === null, 'parseTmLine 对非 TM 行返回 null');

console.log('=== classify（关键词判定） ===');
chk(A.classify(['  Move started.']).kind === 'ok', 'ok');
chk(A.classify(['  Usage: K J value  J=1~6, value=0~254']).kind === 'usage', 'usage');
chk(A.classify(['  IK: solution out of joint limits!']).kind === 'safety', 'safety: out of joint limits');
chk(A.classify(["  E-STOP active! Only 's' and 'h' allowed."]).kind === 'estop', 'estop');
chk(A.classify(['  J1 READ FAILED: no reply (cable/connector/ground).']).kind === 'safety', 'read fail');
chk(A.classify(['  Another loop is active. Use S first.']).kind === 'safety', 'loop active');
chk(A.classify(['  IK: tool pitch NOT met (J4 limit?) - move cancelled.']).kind === 'safety', 'pitch not met');
// ⚠ 回归: 固件真实错误串曾经不命中任何关键词 -> 被兜底判成 ok=True（假成功）
chk(A.classify(["  Unknown command 'x'. Type 'h' for help."]).ok === false,
  'Unknown command 必须判失败（回归）', JSON.stringify(A.classify(["  Unknown command 'x'. Type 'h' for help."])));
chk(A.classify(['  Joint must be 0-5']).ok === false, 'Joint must be 0-5 -> 失败');
chk(A.classify(['  Unknown pose 9. Available: 0=HOME 1=FOLD']).ok === false, 'Unknown pose -> 失败');
chk(A.classify(['  Segment index must be 1~8']).ok === false, 'Segment must be -> 失败');
chk(A.classify(['  Cannot move.']).ok === false, 'Cannot move -> 失败');
chk(A.classify(['  Error: min must be < max (for wrap-around range use \'c\' calibration)']).ok === false,
  'Error: min must be < max -> 失败');
chk(A.classify(['  J1: raw=2048 servo=180.04 deg  rel-home=+0.04 deg']).ok === true,
  '只读命令有输出 -> 成功');

console.log('=== 钳制回显解析（静默改值必须能被 UI 抓到） ===');
const cl = A.parseClampLines(['  J3: 320 -> 300 deg (raw 4096 -> 4095 limit)']);
chk(cl.length === 1 && cl[0].joint === 2 && cl[0].jointLabel === 'J3'
    && cl[0].toDeg === 300 && cl[0].toRaw === 4095,
  'parseClampLines: 0 基索引 / 标签 / 目标角', JSON.stringify(cl));
chk(A.parseClampLines(['  Move started.']).length === 0, '无钳制行 -> 空数组');
const ls = A.parseLimitsSet('  J1 limits set: 80 ~ 220 deg (raw 910 ~ 2503)');
chk(ls && ls.minDeg === 80 && ls.maxDeg === 220 && ls.minRaw === 910,
  'parseLimitsSet', JSON.stringify(ls));

console.log('=== 命令封装：本地校验（越界必须先抛，别让固件静默改值） ===');
{
  const sdk = new A.ArmSDK();
  const mustThrow = (label, fn) => {
    let t = false;
    try { fn(); } catch (e) { t = true; }
    chk(t, label);
  };
  mustThrow('moveJoints 含 320 -> 抛错（防静默钳到 300）', () => sdk.moveJoints([180, 150, 225, 166, 180, 320]));
  mustThrow('moveAngle J=7 -> 抛错', () => sdk.moveAngle(7, 100));
  mustThrow('saveWaypoint n=9 -> 抛错', () => sdk.saveWaypoint(9, [180, 180, 180, 180, 180, 180]));
  mustThrow('pointLoop 轴数不足 -> 抛错', () => sdk.pointLoop([180], [200]));
  mustThrow('multiPointLoop n=9 -> 抛错', () => sdk.multiPointLoop(9));
  mustThrow('setLimit J=6 -> 抛错', () => sdk.setLimit(6, 0, 360));
  // 合法值不应抛（还没真发出去，构造命令本身必须成功）
  let okCmd = true;
  try { sdk.moveJoints([180, 150.9, 225, 166.3, 180, 180], 40); } catch (e) { okCmd = false; }
  chk(okCmd, '合法 moveJoints 不抛错');
}

console.log('=== 二进制状态 ===');
const pl = new Uint8Array(21);
const vals = [1800, 1509, 2250, 1663, 1800, 1800];
vals.forEach((v, i) => { pl[i * 2] = v & 0xFF; pl[i * 2 + 1] = (v >> 8) & 0xFF; });
pl[12] = 3000 & 0xFF; pl[13] = (3000 >> 8) & 0xFF;
pl[16] = 2000 & 0xFF; pl[17] = (2000 >> 8) & 0xFF;
pl[18] = 900 & 0xFF; pl[19] = (900 >> 8) & 0xFF;
pl[20] = 0x08;
const bs = A.parseBinStatus(pl);
chk(Math.abs(bs.jointDeg[0] - 180) < 1e-9 && bs.readOk && bs.pose() !== null,
  'parseBinStatus 解 6 轴 + 位姿', JSON.stringify(bs.jointDeg));
pl[20] = 0x00;
chk(A.parseBinStatus(pl).pose() === null, 'readOk=0 -> pose() = null（不兜底）');
const pl2 = new Uint8Array(21); pl2[20] = 0x08;
for (let i = 0; i < 12; i += 2) { pl2[i] = 0x00; pl2[i + 1] = 0x80; }   // -32768
chk(A.parseBinStatus(pl2).pose() === null, '任一轴 = -32768 -> pose() = null');

console.log('=== 无硬件也可跑：命令串行化与行尾 ===');
const arm = new A.ArmSDK();
chk(typeof arm.sendCommand === 'function', 'ArmSDK 构造成功（不需要浏览器）');
chk(A.splitDeviceText('TM p: 1 2 3\r\n  X=1\r\n[RUN] hi\r\n').lines.length === 1,
  'splitDeviceText 把 TM/[RUN] 分到 async（回放日志测试用）');

console.log('\n' + (fail === 0 ? '全部通过 ✅' : fail + ' 项失败 ❌'));
process.exit(fail ? 1 : 0);
