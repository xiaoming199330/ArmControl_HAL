/**
 * arm_sdk.js — 六轴机械臂上位机 SDK（浏览器 / Electron，Web Serial API）
 *
 * 与 python/arm_sdk.py 功能对齐：
 *   - 串口 115200 8N1 无流控；**sendCommand() 内部强制追加 `\r\n`**
 *   - 响应按 `\r\n> ` 提示符分帧（不按字节数、不靠超时猜）
 *   - 异步行分流：`TM `/`  Feedback:`/`  Pose:`/`[RUN]`
 *   - 二进制协议（0xAA 0x55 帧 + CRC-16/CCITT-FALSE + SEQ 配对）
 *   - FK/IK 与固件逐位一致（已对拍 rho=131.39 / z=3.98 / pitch=150.03）
 *
 * 浏览器要求：Chrome/Edge 89+ 或 Electron（需支持 Web Serial）。
 * 注意：Web Serial 需要安全上下文（https 或 localhost / file 在 Electron 中）。
 *
 * 用法
 * ----
 *   import { ArmSDK, fkFromJointDeg, ikChecked, POSE_HOME_J } from './arm_sdk.js';
 *
 *   const arm = new ArmSDK();
 *   await arm.open();                       // 弹权限选择框
 *   arm.onTelemetry = (d) => updateChart(d);
 *   console.log(await arm.status());
 *   await arm.moveCart(300, 0, 200, 90);
 *   await arm.close();
 */

/* ==========================================================================
 * 1. 机械几何常量（与 Core/Inc/kinematics.h 完全一致 —— 不要改！）
 * ========================================================================== */

export const ROBOT_BASE_H = 65.30;
export const ROBOT_J2_OFF_Y = 31.32;
export const ROBOT_J2_OFF_Z = 54.80;
export const ROBOT_L_UPPER = 116.00;
export const ROBOT_L_FORE = 135.00;
export const ROBOT_L_WRIST = 62.50;
export const ROBOT_L_TOOL = 105.30;
export const ROBOT_L_DISTAL = ROBOT_L_WRIST + ROBOT_L_TOOL;   // 167.80

export const ROBOT_PSI_UPPER0 = 0.243787;   // +13.97 deg
export const ROBOT_PSI_FORE0 = 1.532268;    // +87.79 deg
export const ROBOT_PSI_DISTAL0 = ROBOT_PSI_FORE0;

export const ARM_MAX_REACH = ROBOT_L_UPPER + ROBOT_L_FORE + ROBOT_L_DISTAL;  // 418.80

const KIN_LEN = [ROBOT_L_UPPER, ROBOT_L_FORE, ROBOT_L_DISTAL];
const KIN_BASE = [ROBOT_PSI_UPPER0, ROBOT_PSI_FORE0, ROBOT_PSI_DISTAL0];

export const JOINT_DIR = [-1, 1, 1, 1, 1, 1];
export const DEFAULT_SERVO_OFFSET = [180, 180, 180, 180, 180, 180];

/** 折叠收纳位（关节角，度）。对应固件 main.c POSE_HOME_J。 */
export const POSE_HOME_J = [0.18, -102.95, 93.54, 71.65, -0.88, -50.29];

export const D2R = Math.PI / 180;
export const R2D = 180 / Math.PI;

export const KIN_IK_FAIL_MM = 1.0;
export const KIN_FK_VERIFY_MM = 5.0;
export const KIN_PITCH_TOL_DEG = 10.0;
export const TRAJ_Z_MARGIN_MM = 3.0;
export const HOME_DOCK_TOL_DEG = 3.0;

/* ==========================================================================
 * 2. 角度 / raw 换算
 * ========================================================================== */

export const deg2rad = (d) => d * D2R;
export const rad2deg = (r) => r * R2D;

export function servoDegToRad(deg, joint, off = DEFAULT_SERVO_OFFSET) {
  return JOINT_DIR[joint] * (deg - off[joint]) * D2R;
}

export function radToServoDeg(rad, joint, off = DEFAULT_SERVO_OFFSET) {
  let deg = off[joint] + JOINT_DIR[joint] * rad * R2D;
  return Math.min(Math.max(deg, 0), 360);
}

export function degToServoRaw(deg) {
  const d = Math.min(Math.max(deg, 0), 360);
  return Math.floor(d / 360 * 4095 + 0.5);
}

export function servoRawToDeg(raw) {
  return raw / 4095 * 360;
}

export function jointDegToServoDeg(jointDeg, joint, off = DEFAULT_SERVO_OFFSET) {
  return radToServoDeg(deg2rad(jointDeg), joint, off);
}

export function servoDegToJointDeg(servoDeg, joint, off = DEFAULT_SERVO_OFFSET) {
  return rad2deg(servoDegToRad(servoDeg, joint, off));
}

/* ==========================================================================
 * 3. FK
 * ========================================================================== */

export function planarFk(ang) {
  let s = 0, y = 0, z = 0;
  for (let i = 0; i < 3; i++) {
    s += ang[i];
    y += KIN_LEN[i] * Math.sin(KIN_BASE[i] + s);
    z += KIN_LEN[i] * Math.cos(KIN_BASE[i] + s);
  }
  return [y, z];
}

/**
 * 正运动学。
 * @param {number[]} jointsRad 6 个关节角（弧度）
 * @param {number[]} worldOffset [dx,dy,dz] mm
 * @returns {{x:number,y:number,z:number,pitch:number}} pitch 为**弧度**
 */
export function fk(jointsRad, worldOffset = [0, 0, 0]) {
  const t1 = jointsRad[0];
  const ang = [jointsRad[1], jointsRad[2], jointsRad[3]];
  const [py0, pz0] = planarFk(ang);
  const py = py0 + ROBOT_J2_OFF_Y;
  const pz = pz0 + ROBOT_BASE_H + ROBOT_J2_OFF_Z;
  return {
    x: py * Math.cos(t1) + worldOffset[0],
    y: py * Math.sin(t1) + worldOffset[1],
    z: pz + worldOffset[2],
    pitch: ROBOT_PSI_FORE0 + jointsRad[1] + jointsRad[2] + jointsRad[3],
  };
}

export function fkFromJointDeg(jointDeg, off = DEFAULT_SERVO_OFFSET, worldOffset = [0, 0, 0]) {
  return fk(jointDeg.map((v) => deg2rad(v)), worldOffset);
}

export function fkFromServoDeg(servoDeg, off = DEFAULT_SERVO_OFFSET, worldOffset = [0, 0, 0]) {
  return fk(servoDeg.map((v, j) => servoDegToRad(v, j, off)), worldOffset);
}

export const poseRho = (p) => Math.hypot(p.x, p.y);
export const posePitchDeg = (p) => rad2deg(p.pitch);

/* ==========================================================================
 * 4. 限位
 * ========================================================================== */

export class JointLimits {
  /** ⚠ 默认是"全开放"(±150°)，**不是**安全限位。上位机必须读回实机限位再设置。 */
  constructor(minRad = null, maxRad = null) {
    this.minRad = minRad ? minRad.slice() : new Array(6).fill(-150 * D2R);
    this.maxRad = maxRad ? maxRad.slice() : new Array(6).fill(150 * D2R);
  }

  /** 对应固件 IK_SetJointLimits() —— 角度制，不支持跨 0/360。 */
  setFromDeg(minDeg, maxDeg, off = DEFAULT_SERVO_OFFSET) {
    for (let i = 0; i < 6; i++) {
      const a = JOINT_DIR[i] * (minDeg[i] - off[i]) * D2R;
      const b = JOINT_DIR[i] * (maxDeg[i] - off[i]) * D2R;
      this.minRad[i] = Math.min(a, b);
      this.maxRad[i] = Math.max(a, b);
    }
  }

  /** 对应固件 IK_SetJointLimitsRaw() —— raw 制，正确处理跨 0/360（推荐）。 */
  setFromRaw(minRaw, maxRaw, off = DEFAULT_SERVO_OFFSET) {
    for (let i = 0; i < 6; i++) {
      const mn = minRaw[i] | 0, mx = maxRaw[i] | 0;
      const span = (mx + 4096 - mn) % 4096;
      if (span === 0) continue;                 // 未校准：跳过（保持原值）
      const spanDeg = span * 360 / 4095;
      const dmn = servoRawToDeg(mn);
      const dmx = dmn + spanDeg;                // 连续，可 > 360
      const a = JOINT_DIR[i] * (dmn - off[i]);
      const b = JOINT_DIR[i] * (dmx - off[i]);
      this.minRad[i] = Math.min(a, b) * D2R;
      this.maxRad[i] = Math.max(a, b) * D2R;
    }
  }

  clamp(q) {
    return q.map((v, j) => Math.min(Math.max(v, this.minRad[j]), this.maxRad[j]));
  }

  inside(q, tol = 0) {
    for (let j = 0; j < 6; j++) {
      if (q[j] < this.minRad[j] - tol || q[j] > this.maxRad[j] + tol) return false;
    }
    return true;
  }

  asDeg(off = DEFAULT_SERVO_OFFSET) {
    const mn = [], mx = [];
    for (let j = 0; j < 6; j++) {
      const a = radToServoDeg(this.minRad[j], j, off);
      const b = radToServoDeg(this.maxRad[j], j, off);
      mn.push(Math.min(a, b)); mx.push(Math.max(a, b));
    }
    return [mn, mx];
  }
}

/* ==========================================================================
 * 5. IK
 * ========================================================================== */

function ikAnalytic(ty, tz, toolPitch, lim, ikFail = KIN_IK_FAIL_MM) {
  const j4y = ty - ROBOT_L_DISTAL * Math.sin(toolPitch);
  const j4z = tz - ROBOT_L_DISTAL * Math.cos(toolPitch);
  const d = Math.hypot(j4y, j4z);

  if (d < 1e-3) return null;
  if (d > ROBOT_L_UPPER + ROBOT_L_FORE + 0.01) return null;
  if (d < Math.abs(ROBOT_L_UPPER - ROBOT_L_FORE)) return null;

  let cosb = (ROBOT_L_UPPER ** 2 + d ** 2 - ROBOT_L_FORE ** 2) / (2 * ROBOT_L_UPPER * d);
  cosb = Math.min(Math.max(cosb, -1), 1);
  const beta = Math.acos(cosb);
  const gamma = Math.atan2(j4y, j4z);

  let best = null, bestErr = Infinity;
  for (let side = 0; side <= 1; side++) {
    const psa = gamma + (side ? -beta : beta);
    const j3y = ROBOT_L_UPPER * Math.sin(psa);
    const j3z = ROBOT_L_UPPER * Math.cos(psa);
    const psb = Math.atan2(j4y - j3y, j4z - j3z);

    const t2 = psa - ROBOT_PSI_UPPER0;
    const t3 = (psb - ROBOT_PSI_FORE0) - t2;
    const t4 = toolPitch - ROBOT_PSI_FORE0 - (t2 + t3);

    if (t2 < lim.minRad[1] || t2 > lim.maxRad[1]) continue;
    if (t3 < lim.minRad[2] || t3 > lim.maxRad[2]) continue;
    if (t4 < lim.minRad[3] || t4 > lim.maxRad[3]) continue;

    const [py, pz] = planarFk([t2, t3, t4]);
    const err = Math.hypot(py - ty, pz - tz);
    if (err > ikFail) continue;
    if (err < bestErr) { bestErr = err; best = [t2, t3, t4]; }
  }
  return best;
}

function ccdRun(ang, ty, tz, lim, convergeMm, maxIter) {
  for (let iter = 0; iter < maxIter; iter++) {
    const [ey, ez] = planarFk(ang);
    if (Math.hypot(ty - ey, tz - ez) < convergeMm) break;

    for (let j = 2; j >= 0; j--) {
      let qy = 0, qz = 0, s = 0;
      for (let i = 0; i < j; i++) {
        s += ang[i];
        qy += KIN_LEN[i] * Math.sin(KIN_BASE[i] + s);
        qz += KIN_LEN[i] * Math.cos(KIN_BASE[i] + s);
      }
      const [ey2, ez2] = planarFk(ang);
      const ver = ey2 - qy, vez = ez2 - qz;
      const vtr = ty - qy, vtz = tz - qz;
      const vl = Math.hypot(ver, vez), tl = Math.hypot(vtr, vtz);
      if (vl < 0.001 || tl < 0.001) continue;

      let dot = (ver * vtr + vez * vtz) / (vl * tl);
      const cross = (ver * vtz - vez * vtr) / (vl * tl);
      dot = Math.min(Math.max(dot, -1), 1);

      ang[j] -= Math.atan2(cross, dot);
      ang[j] = Math.min(Math.max(ang[j], lim.minRad[j + 1]), lim.maxRad[j + 1]);
    }
  }
  const [ey, ez] = planarFk(ang);
  return Math.hypot(ty - ey, tz - ez);
}

/**
 * 逆运动学（对应固件 IK_Solve）。
 * ⚠ J1 方位角会被**静默钳制**并仍返回成功 —— 一定要用 ikChecked() 做最终判定。
 * @returns {{ok:boolean, jointsRad:number[], errMm:number, method:string}}
 */
export function ik(target, lim = null, worldOffset = [0, 0, 0],
                   ikFail = KIN_IK_FAIL_MM, convergeMm = 0.5, maxIter = 100,
                   keepJ5J6 = [0, 0]) {
  const L = lim || new JointLimits();
  const x = target.x - worldOffset[0];
  const y = target.y - worldOffset[1];
  const z = target.z - worldOffset[2];
  const toolPitch = target.pitch;

  const joints = [0, 0, 0, 0, 0, 0];
  let t1 = Math.atan2(y, x);
  t1 = Math.min(Math.max(t1, L.minRad[0]), L.maxRad[0]);
  joints[0] = t1;

  const rho = Math.hypot(x, y);
  const ty = rho - ROBOT_J2_OFF_Y;
  const tz = z - (ROBOT_BASE_H + ROBOT_J2_OFF_Z);
  const planarDist = Math.hypot(ty, tz);

  if (planarDist > ARM_MAX_REACH + 0.5) {
    joints[4] = keepJ5J6[0]; joints[5] = keepJ5J6[1];
    return { ok: false, jointsRad: joints, errMm: planarDist - ARM_MAX_REACH, method: 'fail' };
  }

  const ana = ikAnalytic(ty, tz, toolPitch, L, ikFail);
  if (ana) {
    joints[1] = ana[0]; joints[2] = ana[1]; joints[3] = ana[2];
    joints[4] = keepJ5J6[0]; joints[5] = keepJ5J6[1];
    return { ok: true, jointsRad: joints, errMm: 0, method: 'analytic' };
  }

  const aim = Math.atan2(ty, tz);
  let fold = 0;
  if (planarDist < ARM_MAX_REACH && planarDist > 0.1) {
    fold = Math.acos(Math.min(planarDist / ARM_MAX_REACH, 1)) * 0.5;
  }
  const starts = [
    [0, 0, 0],
    [aim * 0.5, 0, 0],
    [fold, fold, -fold],
    [aim * 0.3, aim * 0.3, aim * 0.4],
  ];

  let bestAng = [0, 0, 0], bestErr = Infinity;
  for (const st of starts) {
    const a = st.map((v, i) => Math.min(Math.max(v, L.minRad[i + 1]), L.maxRad[i + 1]));
    const err = ccdRun(a, ty, tz, L, convergeMm, maxIter);
    if (err < bestErr) { bestErr = err; bestAng = a.slice(); }
    if (err < convergeMm) break;
  }
  joints[1] = bestAng[0]; joints[2] = bestAng[1]; joints[3] = bestAng[2];
  joints[4] = keepJ5J6[0]; joints[5] = keepJ5J6[1];
  if (bestErr > ikFail) {
    return { ok: false, jointsRad: joints, errMm: bestErr, method: 'fail' };
  }
  return { ok: true, jointsRad: joints, errMm: bestErr, method: 'ccd' };
}

export function rawDistFwd(a, b) { return (b + 4096 - a) % 4096; }

/**
 * raw 是否在 [mn,mx]（沿正向弧，可跨 0/4096）。
 * ⚠ span==0（未校准）时**恒为 false** => 未校准的轴会拒绝一切运动（固件行为）。
 */
export function rawInRange(raw, mn, mx) {
  const span = rawDistFwd(mn, mx);
  if (span === 0) return false;
  return rawDistFwd(mn, raw) <= span;
}

export function wrapPitchErr(gotDeg, wantDeg) {
  return ((gotDeg - wantDeg + 540) % 360) - 180;
}

/**
 * **完整复刻固件 cmdGoto/cmdSolveIKPoint 的接受闸门** —— 上位机最该用的接口。
 * reason: ok | unreachable | raw_limit | fk_verify | pitch_not_met
 */
export function ikChecked(x, y, z, pitchDeg = 0, opts = {}) {
  const {
    minRaw = null, maxRaw = null, servoOffset = DEFAULT_SERVO_OFFSET,
    lim = null, worldOffset = [0, 0, 0], keepJ5J6ServoDeg = null,
  } = opts;

  const L = lim || new JointLimits();
  let keep = [0, 0];
  if (keepJ5J6ServoDeg) {
    keep = [servoDegToRad(keepJ5J6ServoDeg[4], 4, servoOffset),
            servoDegToRad(keepJ5J6ServoDeg[5], 5, servoOffset)];
  }

  const r = ik({ x, y, z, pitch: deg2rad(pitchDeg) }, L, worldOffset,
               KIN_IK_FAIL_MM, 0.5, 100, keep);
  if (!r.ok) {
    return { ok: false, reason: 'unreachable', jointsRad: r.jointsRad, servoDeg: [],
             raw: [], posErrMm: 0, pitchErrDeg: 0, method: r.method };
  }

  const joints = r.jointsRad;
  const servoDeg = joints.map((v, j) => radToServoDeg(v, j, servoOffset));
  const raw = servoDeg.map((v) => degToServoRaw(v));

  if (minRaw && maxRaw) {
    for (let j = 0; j < 6; j++) {
      if (!rawInRange(raw[j], minRaw[j] | 0, maxRaw[j] | 0)) {
        return { ok: false, reason: 'raw_limit', jointsRad: joints, servoDeg, raw,
                 posErrMm: 0, pitchErrDeg: 0, method: r.method };
      }
    }
  }

  const verify = fk(joints, worldOffset);
  const posErr = Math.hypot(verify.x - x, verify.y - y, verify.z - z);
  if (posErr > KIN_FK_VERIFY_MM) {
    return { ok: false, reason: 'fk_verify', jointsRad: joints, servoDeg, raw,
             posErrMm: posErr, pitchErrDeg: 0, method: r.method };
  }

  const gotPitch = ((rad2deg(verify.pitch) % 360) + 360) % 360;
  const perr = wrapPitchErr(gotPitch, pitchDeg);
  if (Math.abs(perr) > KIN_PITCH_TOL_DEG) {
    return { ok: false, reason: 'pitch_not_met', jointsRad: joints, servoDeg, raw,
             posErrMm: posErr, pitchErrDeg: perr, method: r.method };
  }

  return { ok: true, reason: 'ok', jointsRad: joints, servoDeg, raw,
           posErrMm: posErr, pitchErrDeg: perr, method: r.method };
}

/** 折叠位白名单判定（6 轴同时满足）。 */
export function isHomeDock(jointDeg, tol = HOME_DOCK_TOL_DEG) {
  for (let j = 0; j < 6; j++) {
    const d = ((jointDeg[j] - POSE_HOME_J[j] + 180) % 360) - 180;
    if (Math.abs(d) > tol) return false;
  }
  return true;
}

/** 桌面下限（基座坐标 mm）= 折叠位模型 z − 3mm。 */
export function deskFloorMm(off = DEFAULT_SERVO_OFFSET) {
  return fkFromJointDeg(POSE_HOME_J, off).z - TRAJ_Z_MARGIN_MM;
}

/* ==========================================================================
 * 6. 二进制协议
 * ========================================================================== */

export const APROTO_HEAD0 = 0xAA;
export const APROTO_HEAD1 = 0x55;
export const APROTO_MAX_PAYLOAD = 64;

export const CMD = {
  PING: 0x01, GET_STATUS: 0x02, LOG_SET_LEVEL: 0x03,
  MOVE_JOINTS: 0x10, STOP: 0x11, SET_TORQUE: 0x12, STREAM_JOINTS: 0x13,
  MOVE_CART: 0x20, FK_QUERY: 0x21,
  SET_GRAVITY: 0x30, SET_CORRECTION: 0x31,
  PARAM_READ: 0x40, PARAM_WRITE: 0x41, PARAM_COUNT: 0x42,
  ACK: 0x7F, NACK: 0x7E,
};

export const ERR_DESC = {
  0: '成功', 1: '参数非法', 2: 'CRC 校验失败', 3: 'IK 不可达',
  4: '超关节限位', 5: '急停激活', 6: 'FK 回代超差', 7: '舵机读取失败',
};

/** CRC-16/CCITT-FALSE：poly 0x1021, init 0xFFFF, 非反射。范围 = SEQ..PAYLOAD。 */
export function crc16(data) {
  let crc = 0xFFFF;
  for (let i = 0; i < data.length; i++) {
    crc ^= (data[i] << 8) & 0xFFFF;
    for (let b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) & 0xFFFF : (crc << 1) & 0xFFFF;
    }
  }
  return crc;
}

export function buildFrame(seq, cmd, payload = new Uint8Array(0)) {
  if (payload.length > APROTO_MAX_PAYLOAD) throw new Error('payload too long');
  const body = new Uint8Array(3 + payload.length);
  body[0] = seq & 0xFF; body[1] = cmd & 0xFF; body[2] = payload.length;
  body.set(payload, 3);
  const crc = crc16(body);
  const out = new Uint8Array(5 + payload.length + 2);
  out[0] = APROTO_HEAD0; out[1] = APROTO_HEAD1;
  out.set(body, 2);
  out[out.length - 2] = crc & 0xFF;
  out[out.length - 1] = (crc >> 8) & 0xFF;
  return out;
}

/** 二进制帧解析状态机 + 从混合流里分流文本字节。 */
export class FrameParser {
  constructor(interByteTimeoutMs = 200) {
    this.buf = [];
    this.timeout = interByteTimeoutMs;
    this.last = 0;
  }
  /** @returns {null | {type:'text',byte:number} | {type:'frame',seq,cmd,payload} | {type:'crc_err',seq,cmd}} */
  feed(byte, now = Date.now()) {
    if (this.buf.length && now - this.last > this.timeout) this.buf = [];
    this.buf.push(byte);
    const b = this.buf, n = b.length;

    if (n === 1) {
      if (b[0] !== APROTO_HEAD0) { this.buf = []; return { type: 'text', byte }; }
      this.last = now; return null;
    }
    if (n === 2) {
      if (b[1] === APROTO_HEAD1) { this.last = now; return null; }
      if (b[1] === APROTO_HEAD0) { this.buf.shift(); return null; }
      this.buf = []; return null;
    }
    this.last = now;
    if (n < 5) return null;
    if (n === 5 && b[4] > APROTO_MAX_PAYLOAD) { this.buf = []; return null; }

    const need = 5 + b[4] + 2;
    if (n < need) return null;

    const seq = b[2], cmd = b[3], len = b[4];
    const payload = new Uint8Array(b.slice(5, 5 + len));
    const got = b[5 + len] | (b[5 + len + 1] << 8);
    const want = crc16(new Uint8Array(b.slice(2, 5 + len)));
    this.buf = [];
    if (got !== want) return { type: 'crc_err', seq, cmd };
    return { type: 'frame', seq, cmd, payload };
  }
}

export const readI16 = (b, o) => {
  const v = b[o] | (b[o + 1] << 8);
  return v & 0x8000 ? v - 0x10000 : v;
};
export const readU16 = (b, o) => b[o] | (b[o + 1] << 8);
export const readI32 = (b, o) =>
  (b[o] | (b[o + 1] << 8) | (b[o + 2] << 16) | (b[o + 3] << 24)) | 0;

export const i16Bytes = (v) => new Uint8Array([v & 0xFF, (v >> 8) & 0xFF]);
export const u16Bytes = (v) => new Uint8Array([v & 0xFF, (v >> 8) & 0xFF]);
export const i32Bytes = (v) => new Uint8Array(
  [v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF]);

export const RAW_READ_FAIL = -32768;

/* ==========================================================================
 * 7. 响应判定
 * ========================================================================== */

const SUCCESS_KW = ['OK', 'Saved', 'Stopped', 'complete', 'Complete', 'set to',
                    'saved', 'Done', 'done', 'started', 'accepted'];
const USAGE_KW = ['Usage:', 'usage:'];
// ⚠ 固件真实错误串（2026-09-11 补）: 此前 "Unknown command 'x'. Type 'h' for help."
//   不命中任何关键词, 被末尾 "有输出即成功" 的兜底判成 ok —— 命令没执行却显示成功。
const FAIL_KW = ['FAIL', 'FAILED', 'failed', 'reject', 'Rejected', 'NACK', 'unreachable',
                 'out of range', 'IK failed', 'cancelled', 'Not ',
                 'Unknown command',   // main.c:5719 未知命令
                 'Unknown pose',      // cmdPose 非法预设位号
                 'must be',           // "Joint must be 0-5" / "Segment must be 1~8"
                 'Cannot ',           // "Cannot move." / "Cannot enter teach mode."
                 'Error:', 'Invalid', 'invalid',
                 'not calibrated', 'uncalibrated'];
const SAFETY_KW = ['path check fail', 'path unsafe', 'read FAILED', 'READ FAIL', 'UNSTABLE',
                   'Another loop is active', 'no reply', 'offline',
                   'out of joint limits', 'NOT met', 'too large'];

/** 钳制回显: "  J3: 320 -> 300 deg (raw 4096 -> 4095 limit)" */
const CLAMP_RE = /J(\d+):\s*(-?[\d.]+)\s*->\s*(-?[\d.]+)\s*deg\s*\(raw\s*(\d+)\s*->\s*(\d+)\s*limit\)/;
/** 限位设置回显: "  J1 limits set: 80 ~ 220 deg (raw 910 ~ 2503)" */
const LIMITS_SET_RE = /J(\d+)\s+limits set:\s*(-?[\d.]+)\s*~\s*(-?[\d.]+)\s*deg\s*\(raw\s*(\d+)\s*~\s*(\d+)\)/;

/** @returns {{ok:boolean, kind:string}} kind: ok|usage|fail|safety|estop|empty */
export function classify(lines) {
  const blob = lines.join('\n');
  if (blob.includes('E-STOP')) return { ok: false, kind: 'estop' };
  for (const k of SAFETY_KW) if (blob.includes(k)) return { ok: false, kind: 'safety' };
  for (const k of USAGE_KW) if (blob.includes(k)) return { ok: false, kind: 'usage' };
  for (const k of FAIL_KW) if (blob.includes(k)) return { ok: false, kind: 'fail' };
  for (const k of SUCCESS_KW) if (blob.includes(k)) return { ok: true, kind: 'ok' };
  if (lines.length) return { ok: true, kind: 'ok' };
  return { ok: false, kind: 'empty' };
}

/**
 * 抽出响应里被**静默钳制**的轴（`clamp_angle()` 的回显）。
 * 返回 [{joint:2, jointLabel:'J3', fromDeg:320, toDeg:300, fromRaw:4096, toRaw:4095}]，
 * `joint` 为 **0 基**（与 `p J`/`L J` 一致）。命令算成功, 但输入被改了 —— UI 标黄。
 */
export function parseClampLines(lines) {
  const out = [];
  for (const ln of lines) {
    const m = CLAMP_RE.exec(ln);
    if (m) {
      out.push({
        joint: Number(m[1]) - 1, jointLabel: 'J' + m[1],
        fromDeg: Number(m[2]), toDeg: Number(m[3]),
        fromRaw: Number(m[4]), toRaw: Number(m[5]),
      });
    }
  }
  return out;
}

/** `L J min max` 回显 -> {minDeg,maxDeg,minRaw,maxRaw} 或 null。 */
export function parseLimitsSet(line) {
  const m = LIMITS_SET_RE.exec(line);
  if (!m) return null;
  return { minDeg: Number(m[2]), maxDeg: Number(m[3]),
           minRaw: Number(m[4]), maxRaw: Number(m[5]) };
}

const ASYNC_PREFIXES = ['TM ', '  Feedback:', '  Pose:', '[RUN]', '  [RUN]'];
const DROP_PREFIXES = ['[RUN]', '  [RUN]', 'TM '];
const isAsyncLine = (l) => ASYNC_PREFIXES.some((p) => l.startsWith(p));

/* ==========================================================================
 * 8. 解析器
 * ========================================================================== */

const RE_STATUS_ROW = /\|\s*J(\d)\s*\|\s*(\d+)\s*\|\s*([\d.]+)\s*\|\s*(\S+)\s*\|\s*(\d+)C\s*\|/;
const RE_JOINT = /raw=(-?\d+)\s+servo=(-?[\d.]+)\s+deg\s+rel-home=([+-]?[\d.]+)\s+deg/;
const RE_FK = /FK:\s*x=(-?\d+)\s+y=(-?\d+)\s+z=(-?\d+)\s+mm\s+pitch=(-?\d+)\s+deg/;
const RE_LIM_ROW = /\|\s*J(\d)\s*\|\s*(\d+)\s*\|\s*(\d+)\s*\|\s*(\d+)\s*\|/;
const RE_PID_CACHE = /J(\d) cache PID = P:(\d+) I:(\d+) D:(\d+)/;
const RE_PID_EE = /J(\d) servo EEPROM = P:(\d+) D:(\d+) I:(\d+)\s*(\(.*?\))?/;
const RE_PARAM = /#\s*(\d+)\s+(\S+)\s+([+-]?\d+\.\d+)\s*\[\s*([+-]?\d+\.\d+)\s*~\s*([+-]?\d+\.\d+)\]\s*(\S*)/;

/** `s` 表格 -> {pos:[],moving:[],temp:[]}。pos 为舵机度（读失败固件打 0.0，无法区分）。 */
export function parseStatus(lines) {
  const pos = new Array(6).fill(null), moving = new Array(6).fill(null);
  const temp = new Array(6).fill(0);
  for (const ln of lines) {
    const m = RE_STATUS_ROW.exec(ln);
    if (m) {
      const j = +m[1] - 1;
      pos[j] = parseFloat(m[3]);
      moving[j] = m[4].startsWith('Err') ? null : m[4].startsWith('Yes');
      temp[j] = +m[5];
    }
  }
  return { pos, moving, temp };
}

/** `p J` -> {raw, servoDeg, jointDeg}。 */
export function parseJoint(lines) {
  for (const ln of lines) {
    const m = RE_JOINT.exec(ln);
    if (m) return { raw: +m[1], servoDeg: +m[2], jointDeg: +m[3] };
  }
  return { raw: null, servoDeg: null, jointDeg: null };
}

/** `f` -> Pose。**返回 null = 读取失败**（固件读失败时不打印任何行），不要用旧值兜底！ */
export function parseFk(lines) {
  for (const ln of lines) {
    const m = RE_FK.exec(ln);
    if (m) return { x: +m[1], y: +m[2], z: +m[3], pitch: deg2rad(+m[4]) };
  }
  return null;
}

/** `L` -> {minDeg,maxDeg,curDeg}。⚠ MIN>MAX 是合法的（跨 0/360 包裹）。 */
export function parseLimits(lines) {
  const minDeg = new Array(6).fill(null), maxDeg = new Array(6).fill(null);
  const curDeg = new Array(6).fill(null);
  for (const ln of lines) {
    const m = RE_LIM_ROW.exec(ln);
    if (m) {
      const j = +m[1] - 1;
      minDeg[j] = +m[2]; maxDeg[j] = +m[3]; curDeg[j] = +m[4];
    }
  }
  return { minDeg, maxDeg, curDeg };
}

/** `N J` -> {cache:[P,I,D], eeprom:[P,I,D], match}。⚠ 固件 EEPROM 行输出顺序是 P D I。 */
export function parsePidCompare(lines) {
  const blob = lines.join('\n');
  let cache = null, eeprom = null, match = null, eepromReadFailed = false;
  let m = RE_PID_CACHE.exec(blob);
  if (m) cache = [+m[2], +m[3], +m[4]];
  m = RE_PID_EE.exec(blob);
  if (m) {
    const p = +m[2], d = +m[3], i = +m[4];
    eeprom = [p, i, d];                   // 归一化成 (P,I,D)
    match = (m[5] || '').includes('一致') && !(m[5] || '').includes('不一致');
  } else if (blob.includes('EEPROM read FAILED')) {
    eepromReadFailed = true;
  }
  return { cache, eeprom, match, eepromReadFailed };
}

/** `LP` -> [{id,name,value,min,max,unit}] */
export function parseParams(lines) {
  const out = [];
  for (const ln of lines) {
    const m = RE_PARAM.exec(ln);
    if (m) out.push({ id: +m[1], name: m[2], value: +m[3], min: +m[4], max: +m[5], unit: m[6] || '' });
  }
  return out;
}

/** 解析一行 `TM <字段>:` 遥测。缺值 `--` -> null。 */
export function parseTmLine(line) {
  if (!line.startsWith('TM ')) return null;
  const body = line.slice(3);
  const ci = body.indexOf(':');
  if (ci < 0) return null;
  const key = body.slice(0, ci).trim();
  if (!'ptivsl'.includes(key)) return null;
  let rest = body.slice(ci + 1).trim().replace(/\((?:C|mA|V)\)\s*$/, '');
  const vals = rest.split(/\s+/).filter((s) => s.length).map((t) => {
    if (t === '--') return null;
    const v = parseFloat(t);
    return Number.isNaN(v) ? null : v;
  });
  return { key, values: vals };
}

/** GET_STATUS(0x02) 响应 -> 结构化状态。readOk=false 时**不得**使用 pose()。 */
export function parseBinStatus(payload) {
  const jointDeg = [];
  for (let j = 0; j < 6; j++) {
    const v = readI16(payload, j * 2);
    jointDeg.push(v === RAW_READ_FAIL ? null : v / 10);
  }
  const flags = payload[20];
  const st = {
    jointDeg,
    x: readI16(payload, 12) / 10,
    y: readI16(payload, 14) / 10,
    z: readI16(payload, 16) / 10,
    pitchDeg: readI16(payload, 18) / 10,
    estop: !!(flags & 1), trajActive: !!(flags & 2),
    correction: !!(flags & 4), readOk: !!(flags & 8),
    pose() {
      if (!this.readOk || this.jointDeg.some((v) => v === null)) return null;
      return { x: this.x, y: this.y, z: this.z, pitch: deg2rad(this.pitchDeg) };
    },
  };
  return st;
}

/* ==========================================================================
 * 9. 主 SDK（Web Serial）
 * ========================================================================== */

export class ArmSDK {
  constructor({ baudRate = 115200, responseTimeoutMs = 3000, longTimeoutMs = 60000 } = {}) {
    this.baudRate = baudRate;
    this.responseTimeout = responseTimeoutMs;
    this.longTimeout = longTimeoutMs;

    this.port = null;
    this.writer = null;
    this.reader = null;
    this._readerLoop = null;

    this._parser = new FrameParser();
    this._textBuf = '';
    this._respLines = [];
    this._asyncLines = [];
    this._pending = null;          // {resolve, reject, cmd, timer}
    this._frames = new Map();      // seq -> {cmd,payload}
    this._seq = 0;
    this._busy = Promise.resolve(); // 串行化

    /** 异步回调：UI 在这里更新曲线/日志 */
    this.onTelemetry = null;       // (key, values) => {}
    this.onLogLine = null;         // (line) => {}
    this.onAsyncLine = null;       // (line) => {}
    this.onRawText = null;         // (chunk) => {}
    this.onError = null;

    this.lastStatus = null;
    this.lastPose = null;
  }

  static get supported() {
    return typeof navigator !== 'undefined' && 'serial' in navigator;
  }

  /** 打开串口（浏览器会弹权限框）。 */
  async open() {
    if (!ArmSDK.supported) {
      throw new Error('本环境不支持 Web Serial。请用 Chrome/Edge 89+ 或 Electron。');
    }
    this.port = await navigator.serial.requestPort();
    // 115200 8N1 无流控 —— 与固件 USART6 配置严格一致
    await this.port.open({ baudRate: this.baudRate, dataBits: 8, stopBits: 1,
                          parity: 'none', flowControl: 'none', bufferSize: 4096 });
    this.writer = this.port.writable.getWriter();
    this._readerLoop = this._readLoop();
    await new Promise((r) => setTimeout(r, 200));   // 吃掉开机横幅
    return this;
  }

  /** 用已授权的串口打开（Electron 里可预选）。 */
  async openWithPort(port) {
    this.port = port;
    await this.port.open({ baudRate: this.baudRate, dataBits: 8, stopBits: 1,
                          parity: 'none', flowControl: 'none', bufferSize: 4096 });
    this.writer = this.port.writable.getWriter();
    this._readerLoop = this._readLoop();
    await new Promise((r) => setTimeout(r, 200));
    return this;
  }

  async close() {
    try {
      if (this.writer) { this.writer.releaseLock(); this.writer = null; }
      if (this.reader) { await this.reader.cancel().catch(() => {}); this.reader = null; }
      if (this.port) { await this.port.close(); }
    } catch (e) { /* 忽略关闭异常 */ }
    this.port = null;
  }

  get isOpen() { return !!this.port; }

  async _readLoop() {
    const decoder = new TextDecoder();
    try {
      while (this.port && this.port.readable) {
        this.reader = this.port.readable.getReader();
        try {
          for (;;) {
            const { value, done } = await this.reader.read();
            if (done) break;
            if (!value) continue;
            if (this.onRawText) this.onRawText(decoder.decode(value, { stream: true }));
            for (const b of value) this._feedByte(b);
          }
        } finally {
          this.reader.releaseLock();
        }
      }
    } catch (e) {
      if (this.onError) this.onError(e);
    }
  }

  _feedByte(byte) {
    const res = this._parser.feed(byte, Date.now());
    if (!res) return;
    if (res.type === 'text') return this._onTextChar(res.byte);
    if (res.type === 'frame') {
      this._frames.set(res.seq, { cmd: res.cmd, payload: res.payload });
      return;
    }
    if (res.type === 'crc_err') {
      this._frames.set('crc' + res.seq, { cmd: res.cmd, payload: new Uint8Array(0) });
    }
  }

  _onTextChar(code) {
    if (code === 0x0D || code === 0x0A) {
      const line = this._textBuf; this._textBuf = '';
      if (line) this._onLine(line);
      return;
    }
    this._textBuf += String.fromCharCode(code);
    // 提示符 "> " 是命令结束标记（固件每处理完一条命令打印 "> "，前面必有 \r\n）
    if (this._textBuf.length >= 2 && this._textBuf.slice(-2) === '> ') {
      const pre = this._textBuf.slice(0, -2); this._textBuf = '';
      if (pre) this._onLine(pre);
      this._onPrompt();
    }
  }

  _onLine(line) {
    if (isAsyncLine(line)) {
      this._asyncLines.push(line);
      if (line.startsWith('TM ')) {
        const tm = parseTmLine(line);
        if (tm && this.onTelemetry) this.onTelemetry(tm.key, tm.values);
      } else if (line.startsWith('[RUN]')) {
        if (this.onLogLine) this.onLogLine(line);
      } else if (this.onAsyncLine) {
        this.onAsyncLine(line);
      }
      return;
    }
    this._respLines.push(line);
  }

  _onPrompt() {
    if (this._pending) {
      const p = this._pending;
      this._pending = null;
      clearTimeout(p.timer);
      p.resolve();
    }
  }

  /** ⚠ 高级/危险：不追加行尾、不等响应（仅用于 `c` 交互式校准之类）。 普通业务用 sendCommand()。 */
  async writeRaw(text) {
    if (!this.writer) throw new Error('串口未打开');
    await this.writer.write(new TextEncoder().encode(text));
  }

  _write(bytes) {
    if (!this.writer) throw new Error('串口未打开');
    return this.writer.write(bytes);
  }

  /**
   * 发送一条文本命令并等响应。
   * **行尾由本函数强制追加 `\r\n`** —— 业务层不要再拼行尾。
   * @param {string} command 例如 'p 0'
   * @param {number} [timeoutMs]
   */
  async sendCommand(command, timeoutMs) {
    const cmd = String(command).trim();
    if (!cmd) throw new Error('空命令');
    if (/[\r\n]$/.test(String(command).replace(/[ \t]+$/, ''))) {
      throw new Error('sendCommand() 会自动追加行尾，请勿再传入 \\r/\\n');
    }

    // 串行化：同一时刻只允许一条命令在途（固件回包无请求 ID）
    const run = async () => {
      this._respLines = [];
      this._asyncLines = [];
      this._frames.clear();
      const waited = new Promise((resolve) => {
        this._pending = {
          resolve,
          timer: setTimeout(() => { this._pending = null; resolve(); },
                            timeoutMs || this.responseTimeout),
          cmd,
        };
      });
      await this._write(new TextEncoder().encode(cmd + '\r\n'));   // ← 强制 CRLF
      await waited;

      const rawLines = this._respLines.slice();
      const asyncLines = this._asyncLines.slice();
      const lines = rawLines.filter((ln) => {
        if (DROP_PREFIXES.some((p) => ln.startsWith(p))) return false;
        if (ln.trim() === cmd.trim()) return false;      // 回显
        if (ln.trim() === '>') return false;
        return ln.trim().length > 0;
      });
      const { ok, kind } = classify(lines);
      return { command: cmd, raw: rawLines, lines, asyncLines, ok, kind };
    };

    const prev = this._busy;
    let release;
    this._busy = new Promise((r) => { release = r; });
    await prev;
    try {
      return await run();
    } finally {
      release();
    }
  }

  /** 发送二进制帧并按 SEQ 等响应。 @returns {{cmd,payload,crcOk}} */
  async sendBinary(cmd, payload = new Uint8Array(0), timeoutMs) {
    const seq = (++this._seq) & 0xFF;
    const frame = buildFrame(seq, cmd, payload);
    this._frames.delete(seq);
    await this._write(frame);

    const deadline = Date.now() + (timeoutMs || this.responseTimeout);
    while (Date.now() < deadline) {
      if (this._frames.has('crc' + seq)) {
        this._frames.delete('crc' + seq);
        return { cmd: null, payload: null, crcOk: false };
      }
      const hit = this._frames.get(seq);
      if (hit) {
        this._frames.delete(seq);
        return { cmd: hit.cmd, payload: hit.payload, crcOk: true };
      }
      await new Promise((r) => setTimeout(r, 5));
    }
    return { cmd: null, payload: null, crcOk: false };
  }

  /* ------------------------- 高级 API ------------------------- */

  help() { return this.sendCommand('h'); }

  async status() {
    const r = await this.sendCommand('s');
    const st = parseStatus(r.lines);
    this.lastStatus = st;
    return st;
  }

  /** `p J`（J=0~5） */
  joint(j) {
    if (j < 0 || j > 5) throw new Error('单轴命令关节号 0~5');
    return this.sendCommand('p ' + j).then((r) => parseJoint(r.lines));
  }

  /** `f` -> Pose 或 **null（读失败）**。绝不用旧值兜底。 */
  async pose() {
    const r = await this.sendCommand('f');
    const p = parseFk(r.lines);
    if (p) this.lastPose = p;
    return p;
  }

  /** `L` -> {minDeg,maxDeg,curDeg}（整数舵机度）。 */
  async limits() {
    const r = await this.sendCommand('L');
    return parseLimits(r.lines);
  }

  /** 由 `L` 的整数舵机度近似反算 raw 限位（误差 <= ~11 raw ≈ 1°）。 */
  limitsToRaw(limits) {
    return {
      minRaw: limits.minDeg.map((v) => (v === null ? 0 : degToServoRaw(v))),
      maxRaw: limits.maxDeg.map((v) => (v === null ? 0 : degToServoRaw(v))),
    };
  }

  /**
   * `L J min max`（J=0~5，舵机度）。⚠ 会同时改写 JOINT_MIN_RAW/MAX_RAW。
   */
  setLimit(j, minDeg, maxDeg) {
    if (j < 0 || j > 5) throw new Error('单轴命令的关节号是 0~5（J1=0）');
    return this.sendCommand(`L ${j} ${fmt(minDeg)} ${fmt(maxDeg)}`);
  }

  /**
   * `m a1..a6 [v]` —— **舵机度，不是关节角！**
   *
   * ⚠ 两个**静默**陷阱：① 每轴被硬钳到 0~300，超 300 会变成 300 —— 这里先
   * throw 拦下；要 300~360 请用 moveAngle()/binMoveJoints()。
   * ② speed 省略时固件用 ARM_SC_DEF_VMAX＝**25** deg/s（帮助文本写 30 是错的），
   * 给定值被钳到 5~300。
   */
  moveJoints(servoDeg, speed) {
    if (!servoDeg || servoDeg.length !== 6) throw new Error('需要 6 个舵机度');
    const bad = servoDeg.filter((v) => v < 0 || v > 300);
    if (bad.length) {
      throw new Error('`m` 的舵机度硬钳制是 0~300，越界值 [' + bad.join(', ') +
        '] 会被固件静默改成 300。超 300 请用 moveAngle() / binMoveJoints()');
    }
    let cmd = 'm ' + servoDeg.map((v) => fmt(v)).join(' ');
    if (speed != null) cmd += ' ' + fmt(speed);
    return this.sendCommand(cmd, this.longTimeout);
  }

  /**
   * `a J deg`（J=0~5）—— 单轴运动到舵机度。**没有 0~300 硬钳制**，上限 360。
   * 仍会被 clamp_angle() 按 raw 限位钳制并回显 `J%d: x -> y deg` ——
   * 用 `parseClampLines(resp.lines)` 抓出来标黄，不要当成正常成功。
   */
  moveAngle(j, servoDeg) {
    if (j < 0 || j > 5) throw new Error('单轴命令的关节号是 0~5（J1=0）');
    return this.sendCommand(`a ${j} ${fmt(servoDeg)}`, this.longTimeout);
  }

  /** `q n j1..j6` —— 存路点（n=1~8）。**舵机度，不是关节角！** */
  saveWaypoint(n, servoDeg) {
    if (n < 1 || n > 8) throw new Error('路点号 n 是 1~8');
    if (!servoDeg || servoDeg.length !== 6) throw new Error('需要 6 个舵机度');
    return this.sendCommand('q ' + n + ' ' + servoDeg.map((v) => fmt(v)).join(' '));
  }

  /**
   * `B a1..a6 b1..b6 rounds [v]` —— 两点往返（**舵机度**）。
   * `rounds`=轮数（**1 轮 = A→B→A**，起手先到 A），0=无限。发 `stop()` 才能解除
   * "另一个循环在跑"的拒命令状态。固件把 rounds 钳到 ≤1000、speed 钳到 5~300。
   */
  pointLoop(a, b, rounds = 0, speed = null) {
    if (!a || !b || a.length !== 6 || b.length !== 6) throw new Error('需要 6+6 个舵机度');
    let cmd = 'B ' + a.map((v) => fmt(v)).join(' ') + ' ' +
      b.map((v) => fmt(v)).join(' ') + ' ' + Math.round(rounds);
    if (speed != null) cmd += ' ' + fmt(speed);
    return this.sendCommand(cmd, this.longTimeout);
  }

  /**
   * `QL n r [v]` —— 多点循环（走 `q` 存的前 n 个点）。⚠ **`Q` 必须大写**。
   * 0 = 无限，用 `stop()` 停。
   */
  multiPointLoop(n, rounds = 0, speed = null) {
    if (n < 1 || n > 8) throw new Error('点数 n 是 1~8');
    let cmd = `QL ${n} ${Math.round(rounds)}`;
    if (speed != null) cmd += ' ' + fmt(speed);
    return this.sendCommand(cmd, this.longTimeout);
  }

  /** `g x y z [pitch]` —— mm / 度。 */
  moveCart(x, y, z, pitchDeg = 0) {
    return this.sendCommand(`g ${fmt(x)} ${fmt(y)} ${fmt(z)} ${fmt(pitchDeg)}`,
                            this.longTimeout);
  }

  /** `P 0..4`（0=HOME 1=FOLD 2=READY 3=WAVE1 4=WAVE2） */
  preset(id) {
    if (id < 0 || id > 4) throw new Error('pose_id 0~4');
    return this.sendCommand('P ' + id, this.longTimeout);
  }

  /** `S` —— 一键停止。安全命令，随时可发。 */
  stop() { return this.sendCommand('S'); }

  torque(j, on) {
    if (j < 0 || j > 5) throw new Error('单轴命令关节号 0~5');
    return this.sendCommand(`t ${j} ${on ? 1 : 0}`);
  }

  /* ---- PID 三步链 ---- */

  pidAll() {
    return this.sendCommand('K').then(async () => {
      const grab = (r) => {
        const m = /:\s*(.*)/.exec(r.lines.join('\n'));
        return m ? (m[1].match(/J\d+=(\d+)/g) || []).map((s) => +s.split('=')[1]) : [];
      };
      const p = grab(await this.sendCommand('K'));
      const i = grab(await this.sendCommand('I'));
      const d = grab(await this.sendCommand('D'));
      return { p, i, d };
    });
  }

  /** ⚠ PID 命令关节号是 **1~6**（不是 0~5）。 */
  pidCompare(j) {
    if (j < 1 || j > 6) throw new Error('PID 命令关节号 1~6');
    return this.sendCommand('N ' + j).then((r) => parsePidCompare(r.lines));
  }

  /**
   * 设 PID。**仅改 STM32 缓存**，必须再 burnPid() 才进舵机 EEPROM。
   * 三个都给 → `N J P I D`；只给部分 → `K/I/D J val`。
   */
  async setPid(j, { p = null, i = null, d = null } = {}) {
    if (j < 1 || j > 6) throw new Error('PID 命令关节号 1~6');
    for (const v of [p, i, d]) {
      if (v != null && (v < 0 || v > 254)) throw new Error('PID 0~254');
    }
    if (p != null && i != null && d != null) return this.sendCommand(`N ${j} ${p} ${i} ${d}`);
    const seq = [];
    if (p != null) seq.push(`K ${j} ${p}`);
    if (i != null) seq.push(`I ${j} ${i}`);
    if (d != null) seq.push(`D ${j} ${d}`);
    let last = null;
    for (const c of seq) last = await this.sendCommand(c);
    return last;
  }

  /**
   * `Y J`（J=1~6）或 `Y 0` —— 烧入舵机 EEPROM。
   * ⚠⚠ **调用前必须让用户托住机械臂**：该关节会失力约 0.6s。
   */
  burnPid(j = null) {
    if (j != null) {
      if (j < 1 || j > 6) throw new Error('PID 命令关节号 1~6');
      return this.sendCommand('Y ' + j, 20000);
    }
    return this.sendCommand('Y 0', 20000);
  }

  restoreDefaultPid() { return this.sendCommand('X'); }
  saveFlash() { return this.sendCommand('F', 20000); }

  /* ---- 参数表 ---- */

  paramList() { return this.sendCommand('LP').then((r) => parseParams(r.lines)); }

  /** ⚠ 仅改 RAM，需 saveFlash()。 */
  setParam(id, value) { return this.sendCommand(`SP ${id} ${fmt(value)}`); }

  /* ---- 遥测 ---- */

  /**
   * `tm <ms> [fields]`。fields: p 角度 / t 温度 / i 电流 / v 电压 / s 速度 / l 负载 / a 全部
   * ⚠ 运动中建议 >= 100ms（后台按 20ms 轮转 1 轴，约 120ms 刷完 6 轴）。
   */
  setTelemetry(periodMs = 200, fields = 'pti') {
    if (periodMs <= 0) return this.sendCommand('tm 0');
    return this.sendCommand(`tm ${periodMs} ${fields}`);
  }

  setAngleStream(periodMs = 200) { return this.sendCommand(`pv ${periodMs}`); }
  setPoseStream(periodMs = 200) { return this.sendCommand(`pf ${periodMs}`); }

  /* ---- 二进制 ---- */

  async ping() {
    const { cmd, payload, crcOk } = await this.sendBinary(CMD.PING);
    if (!crcOk || cmd !== 0x81 || !payload || payload.length < 2) return null;
    return { protoVer: payload[0], runId: payload[1] };
  }

  async binGetStatus() {
    const { cmd, payload, crcOk } = await this.sendBinary(CMD.GET_STATUS);
    if (!crcOk || cmd !== 0x82 || !payload || payload.length < 21) return null;
    return parseBinStatus(payload);
  }

  /** FK_QUERY -> Pose 或 null（read_ok=0 就返回 null，不兜底）。 */
  async binFk() {
    const { cmd, payload, crcOk } = await this.sendBinary(CMD.FK_QUERY);
    if (!crcOk || cmd !== 0xA1 || !payload || payload.length < 21) return null;
    if (payload[20] !== 1) return null;
    return { x: readI16(payload, 0) / 10, y: readI16(payload, 2) / 10,
             z: readI16(payload, 4) / 10, pitch: deg2rad(readI16(payload, 6) / 10) };
  }

  /** MOVE_JOINTS(0x10)：6×i16 舵机度×10 + u16 速度×10 + u16 加速度×10。 */
  async binMoveJoints(servoDeg, speedDps = 0, accelDpss = 0) {
    if (servoDeg.length !== 6) throw new Error('需要 6 个舵机度');
    const pl = new Uint8Array(16);
    let o = 0;
    for (const v of servoDeg) { pl.set(i16Bytes(Math.round(v * 10)), o); o += 2; }
    pl.set(u16Bytes(Math.round(speedDps * 10)), o); o += 2;
    pl.set(u16Bytes(Math.round(accelDpss * 10)), o);
    return this._ackResult(await this.sendBinary(CMD.MOVE_JOINTS, pl));
  }

  /** MOVE_CART(0x20)：4×i16(×10) + u16 速度×10 + u16 加速度×10。 */
  async binMoveCart(x, y, z, pitchDeg = 0, speedDps = 0, accelDpss = 0) {
    const pl = new Uint8Array(12);
    let o = 0;
    for (const v of [x, y, z, pitchDeg]) { pl.set(i16Bytes(Math.round(v * 10)), o); o += 2; }
    pl.set(u16Bytes(Math.round(speedDps * 10)), o); o += 2;
    pl.set(u16Bytes(Math.round(accelDpss * 10)), o);
    return this._ackResult(await this.sendBinary(CMD.MOVE_CART, pl));
  }

  binStop() { return this.sendBinary(CMD.STOP).then((r) => r.cmd === CMD.ACK); }

  async binParamRead(id) {
    const { cmd, payload, crcOk } = await this.sendBinary(CMD.PARAM_READ, new Uint8Array([id]));
    if (!crcOk || cmd !== 0xC0 || !payload || payload.length < 5) return null;
    return readI32(payload, 1) / 1000;
  }

  async binParamWrite(id, value) {
    const pl = new Uint8Array(5);
    pl[0] = id;
    pl.set(i32Bytes(Math.round(value * 1000)), 1);
    return this._ackResult(await this.sendBinary(CMD.PARAM_WRITE, pl));
  }

  _ackResult({ cmd, payload, crcOk }) {
    if (!crcOk) return { ok: false, msg: '超时 / CRC 失败' };
    if (cmd === CMD.ACK) return { ok: true, msg: 'OK' };
    if (cmd === CMD.NACK && payload && payload.length >= 2) {
      return { ok: false, msg: ERR_DESC[payload[1]] || ('err=' + payload[1]), err: payload[1] };
    }
    return { ok: false, msg: 'unexpected rsp 0x' + (cmd == null ? '--' : cmd.toString(16)) };
  }

  /* ---- 危险操作（UI 必须二次确认后调用） ---- */

  /** ⚠⚠ `i` / `i J`：会重写 PID/限位/中位。 */
  initServos(j = null) {
    return this.sendCommand(j == null ? 'i' : 'i ' + j, 30000);
  }

  /** ⚠ `M a`：把当前位置设为中位（raw 2048）。 */
  centerAll() { return this.sendCommand('M a', 30000); }

  /** ⚠ `R 1`：广播 reboot 两条总线所有舵机。 */
  broadcastReboot() { return this.sendCommand('R 1', 30000); }
}

/** 数字格式化：避免 "300.0" 之类多余小数（固件用 atof，都接受）。 */
function fmt(v) {
  const n = Number(v);
  if (!Number.isFinite(n)) throw new Error('非法数值: ' + v);
  return Number.isInteger(n) ? String(n) : String(n);
}

/* ==========================================================================
 * 10. 调试用：解析纯文本（无硬件也能验证 UI 解析逻辑）
 * ========================================================================== */

/**
 * 把一段"设备原始输出"喂给解析器，返回分类后的行。
 * 用于单元测试 / 回放日志。
 * @returns {{lines:string[], asyncLines:string[]}}
 */
export function splitDeviceText(text) {
  const lines = [], asyncLines = [];
  for (const raw of text.split(/\r?\n/)) {
    if (!raw) continue;
    (isAsyncLine(raw) ? asyncLines : lines).push(raw);
  }
  return { lines, asyncLines };
}
