/**
 * kinematics.js — 六轴机械臂运动学（FK / IK / 限位 / 角度换算）
 *
 * 从 web/arm_sdk.js 的 §1~§5 原样切出，**逻辑一字未改**。
 * 几何常量与固件 Core/Inc/kinematics.h 逐位一致 —— 不要改！
 * 已对拍：rho=131.39 / z=3.98 / pitch=150.03
 *
 * 本模块**不依赖任何传输层**，浏览器 / Node / Electron 都能直接用。
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
