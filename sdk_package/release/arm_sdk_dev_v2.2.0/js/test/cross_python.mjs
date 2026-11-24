/**
 * cross_python.mjs — 拿 **Python 版** protocol.py 生成的向量，逐字节校验 JS 版。
 *
 * 向量由 `python tools/gen_protocol_vectors.py` 生成到 js/test/protocol_vectors.json。
 * 两边任何一边改了协议，这里就会红 —— 比人肉 review 可靠。
 *
 * 跑法：node test/cross_python.mjs
 */
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

import * as P from '../src/protocol.js';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const VEC = JSON.parse(
  fs.readFileSync(path.join(HERE, 'protocol_vectors.json'), 'utf8'));

let n = 0;
const fails = [];
const chk = (name, cond, extra = '') => {
  n++;
  if (!cond) { fails.push(name + ' ' + extra); console.log('  FAIL ' + name + '  ' + extra); }
};

const same = (a, b) => Math.abs(a - b) < 1e-9;
const hx = P.toHex;
const bytes = P.fromHex;

const CMD_BY_NAME = P.CMD;
const RSP_BY_NAME = P.RSP;

console.log('跨语言对拍：JS protocol.js  vs  Python protocol.py');
console.log('-'.repeat(66));

/* ---- 常量 ---- */
const c = VEC.const;
chk('CRC 向量 0x29B1', P.crc16('123456789') === c.crc_123456789);
chk('MAX_PAYLOAD', P.APROTO_MAX_PAYLOAD === c.max_payload);
chk('帧头 AA 55', P.APROTO_HEAD0 === c.head[0] && P.APROTO_HEAD1 === c.head[1]);
for (const [k, v] of Object.entries(c.cmd)) {
  chk('CMD.' + k + '=' + v, CMD_BY_NAME[k] === v, 'js=' + CMD_BY_NAME[k]);
}
for (const [k, v] of Object.entries(c.rsp)) {
  chk('RSP.' + k + '=' + v, RSP_BY_NAME[k] === v, 'js=' + RSP_BY_NAME[k]);
}
for (const [k, v] of Object.entries(c.err)) {
  chk('ErrCode.' + k + '=' + v, P.ErrCode[k] === v);
}
chk('PARAM_TABLE id 集合一致',
    JSON.stringify(Object.keys(P.PARAM_TABLE).map(Number).sort((a, b) => a - b)) ===
    JSON.stringify(c.param_ids));
chk('JOINT_DIR 一致', JSON.stringify(P.JOINT_DIR) === JSON.stringify(c.joint_dir));
chk('DEFAULT_SERVO_OFFSET 一致',
    JSON.stringify(P.DEFAULT_SERVO_OFFSET) === JSON.stringify(c.default_offset));
console.log('  常量：' + n + ' 项已比对');

/* ---- 打包 ---- */
let base = n;
for (const v of VEC.pack) {
  const pl = bytes(v.payload);
  const lg = hx(P.packLogical(v.seq, v.cmd, pl));
  chk('pack seq=' + v.seq + ' cmd=0x' + v.cmd.toString(16) + ' len=' + pl.length,
      lg === v.logical, 'js=[' + lg + '] py=[' + v.logical + ']');
  const fr = hx(P.buildFrame(v.seq, v.cmd, pl));
  chk('frame seq=' + v.seq, fr === v.frame, 'js=[' + fr + '] py=[' + v.frame + ']');
}
console.log('  打包/前缀：' + (n - base) + ' 项逐字节一致');

/* ---- CAN 分片 ---- */
base = n;
for (const v of VEC.can) {
  const got = P.canBuildFrames(v.seq, v.cmd, bytes(v.payload)).map(hx);
  chk('CAN seq=' + v.seq + ' payload=' + v.payload.length + 'B',
      JSON.stringify(got) === JSON.stringify(v.frames),
      'js=' + JSON.stringify(got) + ' py=' + JSON.stringify(v.frames));
}
// 反向：重组
for (const v of VEC.can) {
  const r = new P.CanReassembler();
  let res = null;
  for (const f of v.frames) { const x = r.feed(bytes(f)); if (x) res = x; }
  chk('CAN 重组 seq=' + v.seq,
      res !== null && res.crcOk && res.seq === v.seq &&
      hx(res.payload) === v.payload,
      res ? 'seq=' + res.seq + ' payload=[' + hx(res.payload) + ']' : 'null');
}
console.log('  CAN 分片+重组：' + (n - base) + ' 项一致');

/* ---- decode_status（顺序：先关节后位姿） ---- */
base = n;
for (const v of VEC.status) {
  const st = P.decodeStatus(bytes(v.payload));
  chk('status joints', JSON.stringify(st.joints) === JSON.stringify(v.joints),
      'js=' + JSON.stringify(st.joints) + ' py=' + JSON.stringify(v.joints));
  chk('status x/y/z/pitch',
      same(st.x, v.x) && same(st.y, v.y) && same(st.z, v.z) && same(st.pitch, v.pitch),
      `${st.x},${st.y},${st.z},${st.pitch} vs ${v.x},${v.y},${v.z},${v.pitch}`);
  chk('status flags 位域',
      st.flags === v.flags && st.estop === v.estop && st.traj === v.traj &&
      st.corr === v.corr && st.read_ok === v.read_ok);
  chk('status failed_joints', JSON.stringify(P.failedJoints(st)) ===
      JSON.stringify(v.failed_joints), JSON.stringify(P.failedJoints(st)));
}
console.log('  GET_STATUS 解码：' + (n - base) + ' 项');

/* ---- decode_fk（顺序相反 + 裸 read_ok） ---- */
base = n;
for (const v of VEC.fk) {
  const fk = P.decodeFk(bytes(v.payload));
  chk('fk x/y/z/pitch',
      same(fk.x, v.x) && same(fk.y, v.y) && same(fk.z, v.z) && same(fk.pitch, v.pitch),
      `${fk.x},${fk.y},${fk.z},${fk.pitch}`);
  chk('fk joints', JSON.stringify(fk.joints) === JSON.stringify(v.joints));
  chk('fk read_ok 裸值', fk.read_ok === v.read_ok, String(fk.read_ok));
}
console.log('  FK_QUERY 解码：' + (n - base) + ' 项');

/* ---- param ---- */
base = n;
for (const v of VEC.param) {
  const pr = P.decodeParamRead(bytes(v.read_payload));
  chk('paramRead id', pr.id === v.id);
  chk('paramRead value', same(pr.value, v.value), String(pr.value));
}
chk('paramCount', P.decodeParamCount(bytes(VEC.param_count.payload)) === VEC.param_count.count);
console.log('  参数读写：' + (n - base) + ' 项');

/* ---- 角度换算 ---- */
base = n;
const cal = new P.JointCalib();
for (const v of VEC.calib) {
  const lg = cal.toLogical(v.servo);
  chk('toLogical ' + JSON.stringify(v.servo),
      lg.every((x, i) => same(x, v.logical[i])),
      'js=' + JSON.stringify(lg) + ' py=' + JSON.stringify(v.logical));
  const bk = cal.toServo(lg);
  chk('toServo 往返', bk.every((x, i) => same(P.wrap180(x - v.servo[i]), 0)),
      JSON.stringify(bk));
}
for (const v of VEC.wrap180) {
  chk('wrap180(' + v.in + ')=' + v.out, same(P.wrap180(v.in), v.out),
      String(P.wrap180(v.in)));
}
console.log('  角度换算：' + (n - base) + ' 项');

/* ---- 定点编码（负半轴关键） ---- */
base = n;
for (const v of VEC.round) {
  chk('roundHalfAway(' + v.in + ')=' + v.out, P.roundHalfAway(v.in) === v.out,
      String(P.roundHalfAway(v.in)));
}
console.log('  定点编码：' + (n - base) + ' 项');

/* ==========================================================================
 * 2026-09-23 扩展：新命令（全指令覆盖）
 * ========================================================================== */

/* ---- 扩展常量 ---- */
base = n;
chk('MAGIC', P.MAGIC === c.magic, String(P.MAGIC));
chk('TEXT_BUF_SZ', P.TEXT_BUF_SZ === c.text_buf_sz, String(P.TEXT_BUF_SZ));
chk('TEXT_EXEC_MAX', P.TEXT_EXEC_MAX === c.text_exec_max, String(P.TEXT_EXEC_MAX));
chk('TEXT_CHUNK', P.TEXT_CHUNK === c.text_chunk, String(P.TEXT_CHUNK));
chk('TEXT_FLAG_TRUNC', P.TEXT_FLAG_TRUNC === c.text_flag_trunc);
chk('TEXT_FLAG_GATE', P.TEXT_FLAG_GATE === c.text_flag_gate);
chk('DETAIL_SIZE', P.DETAIL_SIZE === c.detail_size, String(P.DETAIL_SIZE));
chk('TEACH_SAMPLE_SIZE', P.TEACH_SAMPLE_SIZE === c.teach_sample_size);
chk('TEACH_READ_MAX', P.TEACH_READ_MAX === c.teach_read_max);
chk('PARAM_LIST_MAX', P.PARAM_LIST_MAX === c.param_list_max);
chk('TEMP_LOST', P.TEMP_LOST === c.temp_lost);
chk('HWERR_LOST', P.HWERR_LOST === c.hwerr_lost);
console.log('  扩展常量：' + (n - base) + ' 项');

/* ---- payload 构造：JS 构造器 vs Python 构造器（逐字节） ----
 * ⚠ 上面 pack 循环里的 payload 是 Python 造好的 hex ⇒ 只验了"打包"。
 *   这里反向验"构造"：同一个语义参数，两边产出的 payload 必须一模一样。
 *   少一个符号位扩展、少一个 round-half-away，这里就红。
 */
base = n;
const builderCases = [
  ['GET_JOINT_DETAIL(0xFF)', () => P.payloadGetJointDetail(0xFF)],
  ['GET_JOINT_DETAIL(2)', () => P.payloadGetJointDetail(2)],
  ['MOVE_JOINT_ONE(2,100.5,30,80)', () => P.payloadMoveJointOne(2, 100.5, 30, 80)],
  ['MOVE_JOINT_ONE(0,180,0,0)', () => P.payloadMoveJointOne(0, 180, 0, 0)],
  ['SET_BUS_MODE(2)', () => P.payloadSetBusMode(2)],
  ['SERVO_INIT(0xFF)', () => P.payloadServoInit(0xFF)],
  ['IK_QUERY(200,-12.3,150,90)', () => P.payloadIkQuery(200, -12.3, 150, 90)],
  ['GET_PID(3)', () => P.payloadGetPid(3)],
  ['PARAM_SAVE/MAGIC', () => P.payloadMagic()],
  ['PARAM_LIST(6)', () => P.payloadParamList(6)],
  ['TEACH_LIST(2)', () => P.payloadTeachList(2)],
  ['TEACH_READ(1,300,5)', () => P.payloadTeachRead(1, 300, 5)],
  ['TEACH_READ n 钳到 5', () => P.payloadTeachRead(0, 0, 99)],
  ['TEACH_DELETE(7)', () => P.payloadTeachDelete(7)],
  ['ZERO_RESET(no save)', () => P.payloadZeroReset(false)],
  ['ZERO_RESET(save)', () => P.payloadZeroReset(true)],
  ['TEXT_EXEC("s")', () => P.payloadTextExec('s')],
  ['TEXT_EXEC("link can")', () => P.payloadTextExec('link can')],
  ['TEXT_EXEC("M a")', () => P.payloadTextExec('M a')],
  ['TEXT_FETCH(58,58)', () => P.payloadTextFetch(58, 58)],
  ['TEXT_FETCH(0,58)', () => P.payloadTextFetch(0, 58)],
];
// 扩展块在 pack 向量的**末尾**，顺序与 gen_protocol_vectors.py 里的 pack_cases 一致
const PY_EXT_ORDER = [
  'GET_JOINT_DETAIL(0xFF)', 'GET_JOINT_DETAIL(2)', 'GET_LIMITS', 'GET_INFO',
  'GET_FLAGS', 'MOVE_JOINT_ONE(2,100.5,30,80)', 'MOVE_JOINT_ONE(0,180,0,0)',
  'SET_BUS_MODE(2)', 'SERVO_INIT(0xFF)', 'IK_QUERY(200,-12.3,150,90)',
  'GET_GRAVITY', 'GET_CORRECTION', 'GET_PID(3)', 'PARAM_SAVE/MAGIC',
  'PARAM_LIST(6)', 'TEACH_LIST(2)', 'TEACH_READ(1,300,5)', 'TEACH_READ n 钳到 5',
  'TEACH_DELETE(7)', 'TEACH_COMPACT', 'TEACH_TOTAL', 'CENTER_CALIB',
  'CALIB_ENTER', 'ZERO_RESET(no save)', 'ZERO_RESET(save)', 'SYS_RESET',
  'TEXT_EXEC("s")', 'TEXT_EXEC("link can")', 'TEXT_EXEC("M a")',
  'TEXT_FETCH(58,58)', 'TEXT_FETCH(0,58)',
];
const extStart = VEC.pack.length - PY_EXT_ORDER.length;
const pyExt = {};
PY_EXT_ORDER.forEach((name, i) => { pyExt[name] = VEC.pack[extStart + i].payload; });
let builderChecked = 0;
for (const [name, fn] of builderCases) {
  const want = pyExt[name];
  if (want === undefined) continue;
  const got = hx(fn());
  builderChecked++;
  chk('payload 构造 ' + name, got === want, 'js=[' + got + '] py=[' + want + ']');
}
chk('payload 构造用例全覆盖（' + builderChecked + '/' + builderCases.length + '）',
    builderChecked === builderCases.length);
console.log('  payload 构造逐字节：' + (n - base) + ' 项');

/* ---- 扩展响应解码 ---- */
base = n;
for (const v of VEC.joint_detail) {
  const items = P.decodeJointDetail(bytes(v.payload));
  chk('JOINT_DETAIL ' + v.tag + ' 项数', items.length === v.items.length,
      String(items.length));
  items.forEach((d, i) => {
    const e = v.items[i];
    chk('JOINT_DETAIL ' + v.tag + ' #' + i,
        d.joint === e.joint && d.pos_raw === e.pos_raw && d.speed === e.speed &&
        d.load === e.load && same(d.voltage, e.voltage) && d.temp === e.temp &&
        d.hw_err === e.hw_err && d.ok === e.ok &&
        d.temp_lost === e.temp_lost && d.hw_err_lost === e.hw_err_lost,
        JSON.stringify(d) + ' vs ' + JSON.stringify(e));
  });
}
{
  const v = VEC.limits;
  const lm = P.decodeLimits(bytes(v.payload));
  for (const k of ['min_raw', 'pos_raw', 'max_raw', 'zero_raw']) {
    chk('LIMITS ' + k, JSON.stringify(lm[k]) === JSON.stringify(v[k]),
        'js=' + JSON.stringify(lm[k]));
  }
  chk('LIMITS 48 B', bytes(v.payload).length === 48);
}
{
  const v = VEC.info;
  const iv = P.decodeInfo(bytes(v.payload));
  chk('INFO fw_tag', iv.fw_tag === v.fw_tag, iv.fw_tag);
  chk('INFO build/link/n_params',
      iv.build === v.build && iv.link_mode === v.link_mode && iv.n_params === v.n_params);
  chk('INFO 20 B', bytes(v.payload).length === 20);
}
for (const v of VEC.flags) {
  const fl = P.decodeFlags(bytes(v.payload));
  chk('FLAGS raw=0x' + v.raw.toString(16) + ' run_id=' + v.run_id,
      fl.raw === v.raw && fl.run_id === v.run_id);
  chk('FLAGS 位域派生与 raw 一致',
      fl.estop === !!(v.raw & 0x01) && fl.traj === !!(v.raw & 0x02) &&
      fl.stream === !!(v.raw & 0x04));
}
{
  const v = VEC.ik_query;
  const sd = P.decodeIkQuery(bytes(v.payload));
  chk('IK_QUERY servo_deg', sd.every((x, i) => same(x, v.servo_deg[i])),
      JSON.stringify(sd));
}
for (const v of VEC.gravity) {
  chk('GRAVITY ' + v.payload, same(P.decodeGravity(bytes(v.payload)), v.value));
}
{
  const v = VEC.correction;
  const cv = P.decodeCorrection(bytes(v.payload));
  chk('CORRECTION',
      cv.enabled === v.enabled && same(cv.threshold_deg, v.threshold_deg) &&
      cv.max_rounds === v.max_rounds && cv.pending === v.pending,
      JSON.stringify(cv));
}
for (const v of VEC.pid) {
  const pv = P.decodePid(bytes(v.payload));
  chk('PID ' + v.payload,
      pv.p === v.p && pv.i === v.i && pv.d === v.d && pv.from_eeprom === v.from_eeprom,
      JSON.stringify(pv));
}
{
  const v = VEC.param_list;
  const { total, items } = P.decodeParamList(bytes(v.payload));
  chk('PARAM_LIST total', total === v.total);
  chk('PARAM_LIST 单页 ≤6 条（62 B ≤ 64）',
      items.length <= P.PARAM_LIST_MAX && bytes(v.payload).length === 2 + items.length * 10);
  items.forEach((m, i) => {
    const e = v.items[i];
    chk('PARAM_LIST 条目 #' + i,
        m.id === e.id && m.type === e.type && same(m.vmin, e.vmin) && same(m.vmax, e.vmax),
        JSON.stringify(m));
  });
}
{
  const v = VEC.teach_list;
  const tv = P.decodeTeachList(bytes(v.payload));
  chk('TEACH_LIST',
      tv.total === v.n_files && tv.idx === v.idx && tv.valid === v.valid &&
      tv.name === v.name && tv.samples === v.samples, JSON.stringify(tv));
}
{
  const v = VEC.teach_read;
  const pts = P.decodeTeachRead(bytes(v.payload));
  chk('TEACH_READ 样本数', pts.length === v.points.length);
  chk('TEACH_READ 逐点 raw', JSON.stringify(pts) === JSON.stringify(v.points),
      JSON.stringify(pts));
  chk('TEACH_READ 单页上限一致', P.TEACH_READ_MAX === v.max_per_page);
  chk('TEACH_READ 样本尺寸一致', P.TEACH_SAMPLE_SIZE === v.sample_size);
}
{
  const v = VEC.teach_total;
  const tv = P.decodeTeachTotal(bytes(v.payload));
  chk('TEACH_TOTAL',
      tv.n_files === v.n_files && tv.capacity === v.capacity &&
      tv.total_samples === v.total_samples && tv.free_bytes === v.free_bytes,
      JSON.stringify(tv));
}
{
  const v = VEC.text_out;
  const to = P.decodeTextOut(bytes(v.payload));
  chk('TEXT_OUT total/off', to.total === v.total && to.off === v.off);
  chk('TEXT_OUT flags', to.truncated === v.truncated && to.gated === v.gated);
  chk('TEXT_OUT raw 逐字节', hx(to.raw) === v.raw_hex,
      'js=[' + hx(to.raw) + '] py=[' + v.raw_hex + ']');
  chk('TEXT_OUT UTF-8 解码一致（含中文 —— 证明按字节分页不坏字）',
      to.text === v.text, JSON.stringify(to.text) + ' vs ' + JSON.stringify(v.text));
  chk('TEXT_OUT total 是**字节数**（> 字符数）', to.total > to.text.length,
      to.total + ' vs ' + to.text.length);
}
console.log('  扩展响应解码：' + (n - base) + ' 项');

console.log('-'.repeat(66));
if (fails.length) {
  console.log('失败 ' + fails.length + ' / ' + n + ' 项：');
  fails.forEach((f) => console.log('  · ' + f));
  process.exit(1);
}
console.log('跨语言对拍：' + n + ' 项，全部一致 ✅');
console.log('JS 与 Python 的协议实现逐字节等价。');
