/**
 * protocol.js — 六轴机械臂「逻辑包」协议层（纯 JS，零依赖）
 *
 * 一个逻辑包在固件里只有一种定义：
 *
 *     SEQ(1) | CMD(1) | LEN(1) | PAYLOAD(LEN) | CRC16-LE(2)
 *
 * 不同物理承载只是「怎么把这一串字节送上线」不同：
 *
 *     承载       前缀     分包      备注
 *     --------   ------   -------   ----------------------------------------
 *     UDP 网口   无       不需要    一个数据报 = 一个逻辑包（最快，p50 1.6ms）
 *     串口       AA 55    不需要    与控制台文本共口
 *     CAN 总线   无       需要      8 字节/帧，首帧 d[0] = 0x80|帧数
 *
 * ⚠ 本文件是 `arm_sdk/protocol.py` 的 **JS 镜像**，必须与它逐字节一致。
 *   改动任意一边都要同步另一边，并跑两边的自检对拍 hex 例子。
 *
 * 三个最容易错的点（都已在代码里处理）：
 *   1. CRC-16/CCITT-FALSE：poly 0x1021、init 0xFFFF、**非反射**、无最终异或。
 *      校验向量：crc16("123456789") === 0x29B1
 *   2. 定点编码一律「四舍五入、远离零」。JS 的 Math.round 是**向 +∞ 取整**
 *      （Math.round(-0.5) === -0），与固件 `(int)(v + (v>=0?0.5:-0.5))` 在负半轴不一致。
 *      必须用本文件的 roundHalfAway()。
 *   3. GET_STATUS 与 FK_QUERY 的负载**顺序相反、偏移 20 含义不同**（位域 vs 裸值），
 *      绝对不能共用同一个解析函数。
 */

/* ==========================================================================
 * 1. 协议常量
 * ========================================================================== */

export const APROTO_HEAD0 = 0xAA;
export const APROTO_HEAD1 = 0x55;
export const APROTO_MAX_PAYLOAD = 64;
export const MAX_PACKET = 3 + APROTO_MAX_PAYLOAD + 2;

export const NUM_JOINTS = 6;

/** 命令码（请求）。 */
export const CMD = {
  PING: 0x01,
  GET_STATUS: 0x02,
  LOG_SET_LEVEL: 0x03,
  MOVE_JOINTS: 0x10,
  STOP: 0x11,
  SET_TORQUE: 0x12,
  STREAM_JOINTS: 0x13,
  MOVE_CART: 0x20,
  FK_QUERY: 0x21,
  SET_GRAVITY: 0x30,
  SET_CORRECTION: 0x31,
  PARAM_READ: 0x40,
  PARAM_WRITE: 0x41,
  PARAM_COUNT: 0x42,
  ACK: 0x7F,
  NACK: 0x7E,

  /* ---- 2026-09-23 扩展：让网口覆盖文本控制台能力 ----
   * 设计基准 docs/网口指令扩展设计_2026-09-23.md
   * 四条铁律：命令码 < 0x80；单帧 ≤ 64 B ⇒ 大数组分页；危险命令带 MAGIC；
   *           新命令一律经 ArmApi_*。
   * ⭐ TEXT_EXEC 是其中最关键的一条：一把钥匙把整个控制台(124 条)搬上网口，
   *    走的是固件里**同一条** ProcessCommand ⇒ 以后新增的控制台命令自动就
   *    对网口可用，不用再追着加 opcode。 */
  GET_JOINT_DETAIL: 0x04,
  GET_LIMITS: 0x05,
  GET_INFO: 0x07,
  GET_FLAGS: 0x0A,
  TEXT_EXEC: 0x0B,
  TEXT_FETCH: 0x0C,
  MOVE_JOINT_ONE: 0x14,
  SET_BUS_MODE: 0x17,
  SERVO_INIT: 0x18,
  IK_QUERY: 0x22,
  GET_GRAVITY: 0x32,
  GET_CORRECTION: 0x33,
  GET_PID: 0x36,
  PARAM_SAVE: 0x43,
  PARAM_LIST: 0x44,
  TEACH_LIST: 0x50,
  TEACH_READ: 0x51,
  TEACH_DELETE: 0x53,
  TEACH_COMPACT: 0x54,
  TEACH_TOTAL: 0x55,
  CENTER_CALIB: 0x70,
  CALIB_ENTER: 0x71,
  ZERO_RESET: 0x72,
  SYS_RESET: 0x73,
};

/** 查询类：响应码 = 请求码 | 0x80。ACK/NACK 不在这个规律里，但一并放这儿方便查。 */
export const RSP = {
  ACK: 0x7F,
  NACK: 0x7E,
  PING: 0x81,
  STATUS: 0x82,
  FK: 0xA1,
  PARAM_READ: 0xC0,
  PARAM_COUNT: 0xC2,
  /* 2026-09-23 扩展 */
  JOINT_DETAIL: 0x84,
  LIMITS: 0x85,
  INFO: 0x87,
  FLAGS: 0x8A,
  TEXT_OUT: 0x8B,
  TEXT_FETCH: 0x8C,
  IK_QUERY: 0xA2,
  GRAVITY: 0xB2,
  CORRECTION: 0xB3,
  PID: 0xB6,
  PARAM_LIST: 0xC4,
  TEACH_LIST: 0xD0,
  TEACH_READ: 0xD1,
  TEACH_TOTAL: 0xD5,
};

export const CMD_NAME = {
  0x01: 'PING', 0x02: 'GET_STATUS', 0x03: 'LOG_SET_LEVEL',
  0x10: 'MOVE_JOINTS', 0x11: 'STOP', 0x12: 'SET_TORQUE', 0x13: 'STREAM_JOINTS',
  0x20: 'MOVE_CART', 0x21: 'FK_QUERY',
  0x30: 'SET_GRAVITY', 0x31: 'SET_CORRECTION',
  0x40: 'PARAM_READ', 0x41: 'PARAM_WRITE', 0x42: 'PARAM_COUNT',
  0x7F: 'ACK', 0x7E: 'NACK',
  0x81: 'RSP_PING', 0x82: 'RSP_STATUS', 0xA1: 'RSP_FK',
  0xC0: 'RSP_PARAM_READ', 0xC2: 'RSP_PARAM_COUNT',
  /* 2026-09-23 扩展 */
  0x04: 'GET_JOINT_DETAIL', 0x05: 'GET_LIMITS', 0x07: 'GET_INFO',
  0x0A: 'GET_FLAGS', 0x0B: 'TEXT_EXEC', 0x0C: 'TEXT_FETCH',
  0x14: 'MOVE_JOINT_ONE', 0x17: 'SET_BUS_MODE', 0x18: 'SERVO_INIT',
  0x22: 'IK_QUERY', 0x32: 'GET_GRAVITY', 0x33: 'GET_CORRECTION',
  0x36: 'GET_PID', 0x43: 'PARAM_SAVE', 0x44: 'PARAM_LIST',
  0x50: 'TEACH_LIST', 0x51: 'TEACH_READ', 0x53: 'TEACH_DELETE',
  0x54: 'TEACH_COMPACT', 0x55: 'TEACH_TOTAL',
  0x70: 'CENTER_CALIB', 0x71: 'CALIB_ENTER', 0x72: 'ZERO_RESET', 0x73: 'SYS_RESET',
  0x84: 'RSP_JOINT_DETAIL', 0x85: 'RSP_LIMITS', 0x87: 'RSP_INFO',
  0x8A: 'RSP_FLAGS', 0x8B: 'RSP_TEXT_OUT', 0x8C: 'RSP_TEXT_FETCH',
  0xA2: 'RSP_IK_QUERY', 0xB2: 'RSP_GRAVITY', 0xB3: 'RSP_CORRECTION',
  0xB6: 'RSP_PID', 0xC4: 'RSP_PARAM_LIST', 0xD0: 'RSP_TEACH_LIST',
  0xD1: 'RSP_TEACH_READ', 0xD5: 'RSP_TEACH_TOTAL',
};

export function cmdName(cmd) {
  return CMD_NAME[cmd] || ('0x' + (cmd & 0xFF).toString(16).toUpperCase().padStart(2, '0'));
}

/** NACK 负载第 2 字节。 */
export const ErrCode = {
  OK: 0,
  PARAM: 1,
  CRC: 2,
  UNREACH: 3,
  LIMIT: 4,
  ESTOP: 5,
  FK_VERIFY: 6,
  READ: 7,
  MAGIC: 8,      /* 危险命令 magic 不符（防误触发/串包） */
  BUSY: 9,       /* 轨迹/示教/回放进行中，或命令被安全闸门拒绝 */
  NOTCAL: 10,    /* 未做 Home 零点校准，无法换算 */
};

export const ERR_DESC = {
  0: '成功',
  1: '参数非法（长度/范围/ID/未知命令）',
  2: '帧 CRC 校验失败',
  3: 'IK 不可达 / 不收敛',
  4: '超限/不安全（跳变>25°、撞桌面/立柱、总线写失败）',
  5: '急停激活，禁止运动',
  6: 'IK 解 FK 回代验证超差',
  // 码 7 是多命令复用的：舵机总线读失败 或 读 Flash 参数区失败
  7: '读失败（舵机总线或 Flash 参数区，按命令区分）',
  8: '危险命令 magic 不符（必须显式构造 0x5A5A5A5A）',
  9: '忙/被安全闸门拒绝（轨迹或示教进行中；或该命令从网口下发不安全）',
  10: '该关节未做 Home 零点校准（先 zi 检查、zr 修复）',
};

/** STREAM_JOINTS 的 err=4 细分码。 */
export const StreamRc = {
  OK: 0,
  LIMIT: 1,
  ESTOP: 2,
  MASTER_ROLE: 3,
};

export const RAW_READ_FAIL = -32768;

/** 默认舵机零点 / 方向（仅 J1 反向）。未校准时用。 */
export const DEFAULT_SERVO_OFFSET = [180, 180, 180, 180, 180, 180];
export const JOINT_DIR = [-1, 1, 1, 1, 1, 1];

/** 运行时参数表：id -> {name, unit, min, max, def}。与固件 arm_params.c 对齐。 */
export const PARAM_TABLE = {
  1: { name: 'ik.converge_mm', unit: 'mm', min: 0.1, max: 5.0, def: 0.5 },
  2: { name: 'ik.max_iter', unit: 'iter', min: 10, max: 500, def: 100 },
  3: { name: 'ik.fail_mm', unit: 'mm', min: 0.5, max: 10.0, def: 1.0 },
  4: { name: 'ik.fk_verify_mm', unit: 'mm', min: 1.0, max: 20.0, def: 5.0 },
  16: { name: 'sc.tj', unit: 's', min: 0.02, max: 0.50, def: 0.15 },
  17: { name: 'sc.def_vmax', unit: 'deg/s', min: 5, max: 300, def: 25 },
  18: { name: 'sc.def_accel', unit: 'deg/s2', min: 10, max: 600, def: 80 },
  19: { name: 'sc.v_min', unit: 'deg/s', min: 1, max: 50, def: 5 },
  20: { name: 'sc.v_max', unit: 'deg/s', min: 50, max: 300, def: 300 },
  21: { name: 'sc.accel_factor', unit: 'ratio', min: 1.0, max: 4.0, def: 2.0 },
  32: { name: 'gc.k', unit: 'deg/mm', min: 0.0, max: 0.5, def: 0.0 },
  33: { name: 'corr.enabled', unit: '0/1', min: 0, max: 1, def: 1 },
  34: { name: 'corr.threshold', unit: 'deg', min: 0.5, max: 10.0, def: 2.0 },
  35: { name: 'corr.max_rounds', unit: 'rounds', min: 1, max: 20, def: 10 },
  36: { name: 'corr.gain_j1', unit: 'ratio', min: 0.0, max: 0.85, def: 0.2 },
  37: { name: 'corr.gain_j2', unit: 'ratio', min: 0.0, max: 0.85, def: 0.5 },
  38: { name: 'corr.gain_j3', unit: 'ratio', min: 0.0, max: 0.85, def: 0.3 },
  39: { name: 'corr.gain_j4', unit: 'ratio', min: 0.0, max: 0.85, def: 0.2 },
  40: { name: 'corr.gain_j5', unit: 'ratio', min: 0.0, max: 0.85, def: 0.2 },
  41: { name: 'corr.gain_j6', unit: 'ratio', min: 0.0, max: 0.85, def: 0.2 },
  42: { name: 'traj.approach_speed', unit: '1=4.39d/s', min: 0, max: 4095, def: 30 },
  43: { name: 'traj.approach_frac', unit: 'ratio', min: 0.0, max: 0.25, def: 0.05 },
  44: { name: 'servo.accel', unit: '-', min: 0, max: 254, def: 10 },
  45: { name: 'traj.handoff_deg', unit: 'deg', min: 0.0, max: 15.0, def: 0.0 },
};

export const AP_PARAM_COUNT = 24;

/* ---- 2026-09-23 扩展常量（必须与固件 / Python 版一致） ---- */
/** 危险命令确认字。见固件 APROTO_MAGIC。 */
export const MAGIC = 0x5A5A5A5A;

/** 文本透传尺寸。固件出处：arm_control_protocol.c(APROTO_TEXT_CHUNK) +
 *  main.c(ARM_TEXT_BUF_SZ)。改固件要一起改这里。 */
export const TEXT_BUF_SZ = 2048;     /* 单次捕获上限 */
export const TEXT_EXEC_MAX = 58;     /* 一条 TEXT_EXEC 能带的命令字符数 */
export const TEXT_CHUNK = 58;        /* 每页正文上限（64 − 6B 分页头） */
export const TEXT_FLAG_TRUNC = 0x01; /* 输出被 TEXT_BUF_SZ 截断（半截，别当完整） */
export const TEXT_FLAG_GATE = 0x02;  /* 被固件安全闸门拒绝：正文是原因说明 */

export const DETAIL_SIZE = 10;       /* GET_JOINT_DETAIL 每轴字节数 */
export const TEACH_SAMPLE_SIZE = 12; /* 示教样本 6×u16 raw */
export const TEACH_READ_MAX = 5;     /* 单帧最多 5 个样本（5×12 = 60 ≤ 64） */
export const PARAM_LIST_MAX = 6;     /* 单帧最多 6 条元数据（2 + 6×10 = 62 ≤ 64） */
export const TEMP_LOST = 0xFF;       /* 温度丢读哨兵（不是 0 度） */
export const HWERR_LOST = 0xFF;      /* 硬件错误读失败哨兵（不是 0 = 正常） */
export const BUILD_NAME = { 0: 'net', 1: 'can', 2: 'both', 3: 'uart' };

/** UTF-8 编解码（Node ≥18 / 浏览器都有）。
 *  文本透传必须**按字节**拼接、最后统一 decode 一次：分页边界可能正好切在
 *  多字节字符中间，逐页 decode 会在缝上拼出坏字符。 */
export const utf8Encoder = new TextEncoder();
export const utf8Decoder = new TextDecoder('utf-8');

/** 协议层错误（长度/CRC 不符）。 */
export class ProtocolError extends Error {
  constructor(msg) { super(msg); this.name = 'ProtocolError'; }
}

/* ==========================================================================
 * 2. CRC-16/CCITT-FALSE
 * ========================================================================== */

/**
 * CRC-16/CCITT-FALSE：poly 0x1021, init 0xFFFF, **非反射**, 无最终异或。
 * 范围 = SEQ, CMD, LEN, PAYLOAD（逻辑包偏移 0 起，共 3+LEN 字节）。
 * @param {Uint8Array|number[]|string} data
 * @returns {number}
 */
export function crc16(data) {
  if (typeof data === 'string') {
    const enc = new TextEncoder().encode(data);
    return crc16(enc);
  }
  let crc = 0xFFFF;
  for (let i = 0; i < data.length; i++) {
    crc ^= (data[i] & 0xFF) << 8;
    for (let b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) & 0xFFFF : (crc << 1) & 0xFFFF;
    }
  }
  return crc & 0xFFFF;
}

/* ==========================================================================
 * 3. 逻辑包（三承载共用的那一段）
 * ========================================================================== */

/** 组逻辑包（不含任何前缀）。 */
export function packLogical(seq, cmd, payload = new Uint8Array(0)) {
  if (payload.length > APROTO_MAX_PAYLOAD) {
    throw new ProtocolError('payload 超长 ' + payload.length + ' > ' + APROTO_MAX_PAYLOAD);
  }
  const body = new Uint8Array(3 + payload.length);
  body[0] = seq & 0xFF;
  body[1] = cmd & 0xFF;
  body[2] = payload.length;
  body.set(payload, 3);
  const c = crc16(body);
  const out = new Uint8Array(body.length + 2);
  out.set(body, 0);
  out[out.length - 2] = c & 0xFF;
  out[out.length - 1] = (c >> 8) & 0xFF;
  return out;
}

/**
 * 解逻辑包（不含前缀）→ {seq, cmd, payload}。
 * ⚠ 长度必须**恰好** 3+LEN+2，否则抛 ProtocolError（固件对长度不符的包静默丢弃）。
 */
export function unpackLogical(data) {
  if (data.length < 5) throw new ProtocolError('包太短: ' + data.length);
  const seq = data[0], cmd = data[1], ln = data[2];
  if (ln > APROTO_MAX_PAYLOAD) throw new ProtocolError('LEN=' + ln + ' 超上限');
  if (data.length !== 3 + ln + 2) {
    throw new ProtocolError('长度不符: 实收 ' + data.length + ', LEN=' + ln +
                            ' 要求 ' + (3 + ln + 2));
  }
  const got = data[3 + ln] | (data[3 + ln + 1] << 8);
  const want = crc16(data.subarray(0, 3 + ln));
  if (got !== want) {
    throw new ProtocolError('CRC 错: got 0x' + got.toString(16) + ' want 0x' + want.toString(16));
  }
  return { seq, cmd, payload: data.slice(3, 3 + ln) };
}

/** 串口承载：逻辑包前加 `AA 55`。⚠ 这是"带前缀"的版本，UDP 不要用。 */
export function buildFrame(seq, cmd, payload = new Uint8Array(0)) {
  const p = packLogical(seq, cmd, payload);
  const out = new Uint8Array(p.length + 2);
  out[0] = APROTO_HEAD0;
  out[1] = APROTO_HEAD1;
  out.set(p, 2);
  return out;
}

/* ==========================================================================
 * 4. 定点编码（**四舍五入远离零**，与固件一致）
 * ========================================================================== */

/**
 * 四舍五入、远离零。对齐固件 `(int)(v + (v>=0?0.5:-0.5))`。
 * ⚠ 不能直接用 Math.round —— 它是向 +∞ 取整，负半轴会和固件差 1 个 LSB。
 */
export function roundHalfAway(v) {
  return v >= 0 ? Math.floor(v + 0.5) : -Math.floor(-v + 0.5);
}

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

export function i16Bytes(v) {
  const n = clamp(Math.trunc(v), -32768, 32767) & 0xFFFF;
  return [n & 0xFF, (n >> 8) & 0xFF];
}
export function u16Bytes(v) {
  const n = clamp(Math.trunc(v), 0, 65535) & 0xFFFF;
  return [n & 0xFF, (n >> 8) & 0xFF];
}
export function i32Bytes(v) {
  const n = clamp(Math.trunc(v), -2147483648, 2147483647) | 0;
  return [n & 0xFF, (n >>> 8) & 0xFF, (n >>> 16) & 0xFF, (n >>> 24) & 0xFF];
}

export const readI16 = (b, o) => {
  const v = b[o] | (b[o + 1] << 8);
  return v & 0x8000 ? v - 0x10000 : v;
};
export const readU16 = (b, o) => b[o] | (b[o + 1] << 8);
export const readI32 = (b, o) =>
  (b[o] | (b[o + 1] << 8) | (b[o + 2] << 16) | (b[o + 3] << 24)) | 0;

/* ==========================================================================
 * 5. 请求负载构造
 * ========================================================================== */

/**
 * 0x10 MOVE_JOINTS：6×i16 舵机度×10 + u16 速度×10 + u16 加速度×10 = 16 B
 * ⚠ 角度是**舵机度 0~360**（不是逻辑角）。v/a 给 0 = 用臂内默认。
 */
export function payloadMoveJoints(servoDeg, v = 0, a = 0) {
  if (servoDeg.length !== NUM_JOINTS) throw new ProtocolError('需要 6 个舵机度');
  const out = [];
  for (const d of servoDeg) out.push(...i16Bytes(roundHalfAway(d * 10)));
  out.push(...u16Bytes(roundHalfAway(Math.max(0, v) * 10)));
  out.push(...u16Bytes(roundHalfAway(Math.max(0, a) * 10)));
  return Uint8Array.from(out);
}

/** 0x13 STREAM_JOINTS：6×i16 **逻辑角**×10 = 12 B */
export function payloadStreamJoints(logicalDeg) {
  if (logicalDeg.length !== NUM_JOINTS) throw new ProtocolError('需要 6 个逻辑角');
  const out = [];
  for (const d of logicalDeg) out.push(...i16Bytes(roundHalfAway(d * 10)));
  return Uint8Array.from(out);
}

/** 0x20 MOVE_CART：i16 x,y,z(mm×10) + i16 pitch(deg×10) + u16 v×10 + u16 a×10 = 12 B */
export function payloadMoveCart(x, y, z, pitch = 0, v = 0, a = 0) {
  const out = [
    ...i16Bytes(roundHalfAway(x * 10)),
    ...i16Bytes(roundHalfAway(y * 10)),
    ...i16Bytes(roundHalfAway(z * 10)),
    ...i16Bytes(roundHalfAway(pitch * 10)),
    ...u16Bytes(roundHalfAway(Math.max(0, v) * 10)),
    ...u16Bytes(roundHalfAway(Math.max(0, a) * 10)),
  ];
  return Uint8Array.from(out);
}

/** 0x12 SET_TORQUE：u8 joint(0~5 或 0xFF=全部) + u8 enable */
export function payloadSetTorque(joint, enable) {
  if (joint !== 0xFF && !(joint >= 0 && joint <= 5)) {
    throw new ProtocolError('joint 取 0~5 或 0xFF(全部)；6 是非法值');
  }
  return Uint8Array.from([joint & 0xFF, enable ? 1 : 0]);
}

/** 0x30 SET_GRAVITY：i16 K(deg/mm ×1000) */
export function payloadSetGravity(kDegPerMm) {
  return Uint8Array.from(i16Bytes(roundHalfAway(kDegPerMm * 1000)));
}

/** 0x31 SET_CORRECTION：u8 en + u16 thr(deg×100) + u8 rounds */
export function payloadSetCorrection(enable, thresholdDeg, maxRounds) {
  return Uint8Array.from([
    enable ? 1 : 0,
    ...u16Bytes(roundHalfAway(thresholdDeg * 100)),
    Math.trunc(maxRounds) & 0xFF,
  ]);
}

/** 0x40 PARAM_READ：u8 id */
export function payloadParamRead(id) {
  return Uint8Array.from([id & 0xFF]);
}

/** 0x41 PARAM_WRITE：u8 id + i32 value×1000 */
export function payloadParamWrite(id, value) {
  return Uint8Array.from([id & 0xFF, ...i32Bytes(roundHalfAway(value * 1000))]);
}

/** 0x03 LOG_SET_LEVEL：u8 level 0~4 */
export function payloadLogSetLevel(level) {
  return Uint8Array.from([clamp(Math.trunc(level), 0, 4) & 0xFF]);
}

/* ==========================================================================
 * 6. 响应负载解码
 * ========================================================================== */

/**
 * 解 0x82 GET_STATUS 负载（21 B）。
 * 顺序：6×i16 关节(舵机度×10) → i16 X/Y/Z/Pitch → u8 flags 位域
 * @returns {{joints:(number|null)[], x,y,z,pitch, flags, estop, traj, corr, read_ok}}
 */
export function decodeStatus(pl) {
  if (pl.length < 21) throw new ProtocolError('GET_STATUS 负载应 21 字节, 实收 ' + pl.length);
  const joints = [];
  for (let j = 0; j < NUM_JOINTS; j++) {
    const v = readI16(pl, j * 2);
    joints.push(v === RAW_READ_FAIL ? null : v / 10);
  }
  const flags = pl[20];
  return {
    joints,
    x: readI16(pl, 12) / 10,
    y: readI16(pl, 14) / 10,
    z: readI16(pl, 16) / 10,
    pitch: readI16(pl, 18) / 10,
    flags,
    estop: !!(flags & 0x01),
    traj: !!(flags & 0x02),
    corr: !!(flags & 0x04),
    read_ok: !!(flags & 0x08),
  };
}

/**
 * 解 0xA1 FK_QUERY 负载（21 B）。
 * ⚠ 与 Status **顺序相反**（先位姿后关节），偏移 20 是**裸 read_ok(0/1)**，不是位域。
 */
export function decodeFk(pl) {
  if (pl.length < 21) throw new ProtocolError('FK_QUERY 负载应 21 字节, 实收 ' + pl.length);
  const joints = [];
  for (let j = 0; j < NUM_JOINTS; j++) {
    const v = readI16(pl, 8 + j * 2);
    joints.push(v === RAW_READ_FAIL ? null : v / 10);
  }
  return {
    x: readI16(pl, 0) / 10,
    y: readI16(pl, 2) / 10,
    z: readI16(pl, 4) / 10,
    pitch: readI16(pl, 6) / 10,
    joints,
    read_ok: pl[20],
  };
}

/** 解 0xC0 PARAM_READ 负载（5 B）→ {id, value}。 */
export function decodeParamRead(pl) {
  if (pl.length < 5) throw new ProtocolError('PARAM_READ 负载应 5 字节, 实收 ' + pl.length);
  return { id: pl[0], value: readI32(pl, 1) / 1000 };
}

/** 解 0xC2 PARAM_COUNT 负载（1 B）→ 数量。 */
export function decodeParamCount(pl) {
  if (pl.length < 1) throw new ProtocolError('PARAM_COUNT 负载应 1 字节');
  return pl[0];
}

/** 解 0x81 PING 负载（2 B）→ {proto_ver, run_id}。 */
export function decodePing(pl) {
  return { proto_ver: pl[0] ?? 0, run_id: pl[1] ?? 0 };
}

/** GET_STATUS 结果里"读失败轴的 1 基编号"，如 [3]。 */
export const failedJoints = (st) =>
  st.joints.map((v, i) => (v === null ? i + 1 : null)).filter((v) => v !== null);

/* ==========================================================================
 * 7. 角度体系（两套量纲之间的换算）
 * ========================================================================== */

/** 把角度归一化到 (-180, 180]。 */
export function wrap180(a) {
  a = (a + 180) % 360;
  if (a <= 0) a += 360;
  return a - 180;
}

/**
 * 单台臂的关节标定：逻辑角 ⇄ 舵机度。
 *
 * offset : 每个关节"运动学零点(官方 Home)"对应的**舵机度**（出厂默认 180）
 * dir    : JOINT_DIR，仅 J1 反向
 *
 *   舵机度 = (offset[j] + dir[j] * 逻辑角) mod 360
 *   逻辑角 = dir[j] * (舵机度 - offset[j])，归一到 (-180, 180]
 */
export class JointCalib {
  constructor(offset = DEFAULT_SERVO_OFFSET, dir = JOINT_DIR, source = 'default(180.0, 未校准)') {
    if (offset.length !== NUM_JOINTS || dir.length !== NUM_JOINTS) {
      throw new ProtocolError('offset/dir 必须各 6 个');
    }
    this.offset = [...offset];
    this.dir = [...dir];
    this.source = source;
  }

  /** 舵机度 → 逻辑角（STREAM_JOINTS 用）。 */
  toLogical(servoDeg) {
    return servoDeg.map((s, j) => wrap180(this.dir[j] * (s - this.offset[j])));
  }

  /** 逻辑角 → 舵机度（MOVE_JOINTS 用）。 */
  toServo(logicalDeg) {
    return logicalDeg.map((l, j) => {
      const v = (this.offset[j] + this.dir[j] * l) % 360;
      return v < 0 ? v + 360 : v;
    });
  }

  get isDefault() { return this.source.startsWith('default'); }

  describe() {
    return 'offset=[' + this.offset.map((v) => v.toFixed(1)).join(', ') + '] dir=[' +
      this.dir.map((d) => (d >= 0 ? '+' : '') + d).join(',') + '] (' + this.source + ')';
  }
}

/**
 * 从串口 `zi` 的输出里抽出 6 个零点 raw，转成舵机度。
 * 形如 `[ZERO] J1=2048 J2=2048 ...` 或 `Home 零点已记录: 2014 896 3106 2331 2022 1567`
 */
export function parseServoOffsetFromZi(text) {
  const nums = (text.match(/\b\d{2,4}\b/g) || [])
    .map((s) => parseInt(s, 10))
    .filter((n) => n >= 0 && n <= 4095);
  const cand = nums.slice(-6);
  if (cand.length !== 6) return null;
  return cand.map((v) => Math.round((v / 4095) * 360 * 10000) / 10000);
}

/* ==========================================================================
 * 8. CAN 承载（分包 / 重组）
 * ========================================================================== */

/** 返回 {frames, firstN}，与固件 can_send_packed 同公式。 */
export function canPlanFrames(payloadLen) {
  const tail = payloadLen + 2;
  const firstN = Math.min(tail, 4);
  const rest = Math.max(tail - 4, 0);
  return { frames: 1 + Math.ceil(rest / 7), firstN };
}

/** 把一条逻辑包切成 CAN 数据场（Uint8Array 数组，DLC 可变、不补 8）。 */
export function canBuildFrames(seq, cmd, payload = new Uint8Array(0)) {
  if (payload.length > APROTO_MAX_PAYLOAD) throw new ProtocolError('payload 超长');
  const logical = packLogical(seq, cmd, payload);
  const { frames, firstN } = canPlanFrames(payload.length);
  const out = [
    Uint8Array.from([0x80 | frames, logical[0], logical[1], logical[2],
                     ...logical.slice(3, 3 + firstN)]),
  ];
  let off = 3 + firstN;
  for (let f = 1; f < frames; f++) {
    const chunk = logical.slice(off, off + 7);
    out.push(Uint8Array.from([f, ...chunk]));
    off += chunk.length;
  }
  return out;
}

/**
 * 把 CAN 帧还原成逻辑包。
 * ⚠ bxCAN 只有 3 个发送邮箱，**续帧会乱序到达**（硬件行为，不是丢帧）。
 *   实测到达顺序可能是 0,3,1,2。所以这里**按帧号缓冲**，收齐再拼。
 */
export class CanReassembler {
  constructor(strict = false) {
    this.strict = strict;
    this.reset();
  }

  reset() {
    this.active = false;
    this.frames = 0;
    this.next = 0;
    this.head = null;
    this.hdr = null;
    this.parts = new Map();
  }

  /** 喂一帧，收齐则返回 {seq, cmd, payload, crcOk}，否则 null。 */
  feed(data) {
    if (!data || data.length === 0) return null;

    if (data[0] & 0x80) {
      if (data.length < 4) return null;
      const frames = data[0] & 0x0F;
      const ln = data[3];
      if (frames === 0 || ln > APROTO_MAX_PAYLOAD) { this.reset(); return null; }
      const { frames: want, firstN } = canPlanFrames(ln);
      if (frames !== want) { this.reset(); return null; }
      this.active = true;
      this.frames = frames;
      this.next = 1;
      this.hdr = Uint8Array.from([data[1], data[2], data[3]]);
      this.head = data.slice(4, 4 + firstN);
      this.parts = new Map();
    } else if (this.active && data[0] >= 1 && data[0] < this.frames) {
      if (this.strict && data[0] !== this.next) { this.reset(); return null; }
      if (!this.parts.has(data[0])) this.parts.set(data[0], data.slice(1));
      this.next = Math.max(this.next, data[0] + 1);
    } else {
      this.reset();
      return null;
    }

    if (this.active && this.parts.size >= this.frames - 1) {
      const chunks = [this.head];
      for (let i = 1; i < this.frames; i++) chunks.push(this.parts.get(i));
      let total = 0;
      for (const c of chunks) total += c.length;
      const body = new Uint8Array(total);
      let o = 0;
      for (const c of chunks) { body.set(c, o); o += c.length; }

      const seq = this.hdr[0], cmd = this.hdr[1], ln = this.hdr[2];
      if (body.length >= ln + 2) {
        const got = body[ln] | (body[ln + 1] << 8);
        const crcBuf = new Uint8Array(this.hdr.length + ln);
        crcBuf.set(this.hdr, 0);
        crcBuf.set(body.subarray(0, ln), this.hdr.length);
        const ok = got === crc16(crcBuf);
        this.reset();
        return { seq, cmd, payload: body.slice(0, ln), crcOk: ok };
      }
    }
    return null;
  }
}

/* ==========================================================================
 * 9. 串口混合字节流解析（文本 + AA55 二进制）
 * ========================================================================== */

/**
 * 从混合字节流里挑出二进制帧；非 0xAA 的字节原样交还文本通道。
 * 与固件 Aproto_ParserFeed 对称。
 *
 * @returns {null | {type:'text',byte:number} | {type:'frame',seq,cmd,payload}
 *                 | {type:'crc_err',seq,cmd} | {type:'drop'}}
 */
export class FrameParser {
  constructor(interByteTimeoutMs = 200) {
    this.buf = [];
    this.timeout = interByteTimeoutMs;
    this.last = 0;
  }

  feed(byte, now = Date.now()) {
    if (this.buf.length && now - this.last > this.timeout) this.buf = [];
    this.buf.push(byte & 0xFF);
    const b = this.buf, n = b.length;

    if (n === 1) {
      if (b[0] !== APROTO_HEAD0) { this.buf = []; return { type: 'text', byte: byte & 0xFF }; }
      this.last = now;
      return null;
    }
    if (n === 2) {
      if (b[1] === APROTO_HEAD1) { this.last = now; return null; }
      if (b[1] === APROTO_HEAD0) { this.buf.shift(); return null; }
      this.buf = [];
      return { type: 'drop' };
    }
    this.last = now;
    if (n < 5) return null;
    if (n === 5 && b[4] > APROTO_MAX_PAYLOAD) { this.buf = []; return { type: 'drop' }; }

    const need = 5 + b[4] + 2;
    if (n < need) return null;

    const seq = b[2], cmd = b[3], len = b[4];
    const payload = Uint8Array.from(b.slice(5, 5 + len));
    const got = b[5 + len] | (b[5 + len + 1] << 8);
    const want = crc16(Uint8Array.from(b.slice(2, 5 + len)));
    this.buf = [];
    if (got !== want) return { type: 'crc_err', seq, cmd };
    return { type: 'frame', seq, cmd, payload };
  }

  reset() { this.buf = []; }
}

/* ==========================================================================
 * 10. 工具
 * ========================================================================== */

export const toHex = (u8) =>
  Array.from(u8, (b) => (b & 0xFF).toString(16).toUpperCase().padStart(2, '0')).join(' ');

/**
 * "AA 55 31 01" → Uint8Array。
 * ⚠ 空串/纯空白必须返回**零长度**数组 —— 别写成 split 后 parseInt('') = NaN，
 *   那会得到 1 个字节的 [0]，把空负载的包悄悄变成 LEN=1（对拍时真踩过）。
 */
export const fromHex = (s) => {
  const t = String(s).trim();
  if (!t) return new Uint8Array(0);
  const parts = t.split(/\s+/);
  const out = new Uint8Array(parts.length);
  for (let i = 0; i < parts.length; i++) {
    const v = parseInt(parts[i], 16);
    if (!Number.isFinite(v)) throw new ProtocolError('非法 hex: ' + s);
    out[i] = v & 0xFF;
  }
  return out;
};

/* ==========================================================================
 * 10b. 2026-09-23 扩展：请求负载构造 + 响应解码
 *      镜像 arm_sdk/protocol.py 的同名函数，字节必须逐一对齐。
 * ========================================================================== */

const asU8 = (arr) => Uint8Array.from(arr);
const u32le = (v) => [v & 0xFF, (v >>> 8) & 0xFF, (v >>> 16) & 0xFF, (v >>> 24) & 0xFF];
const readU32 = (b, o) =>
  ((b[o] | (b[o + 1] << 8) | (b[o + 2] << 16) | (b[o + 3] << 24)) >>> 0);

/**
 * 0x14 MOVE_JOINT_ONE：u8 joint + i16 deg×10 + u16 v×10 + u16 a×10 = 7 B
 * ⚠ 角度是**舵机度**（与 MOVE_JOINTS 同量纲，不是 STREAM_JOINTS 的逻辑角）。
 * ⚠ 固件会先读全场快照、只改这一轴 —— 任一轴丢读则整条回 ERR_READ
 *   （拿不准位置时不动，比动错强）。
 */
export function payloadMoveJointOne(joint, servoDeg, v = 0, a = 0) {
  if (!(joint >= 0 && joint <= 5)) throw new ProtocolError('joint 取 0~5');
  return asU8([joint & 0xFF,
    ...i16Bytes(roundHalfAway(servoDeg * 10)),
    ...u16Bytes(roundHalfAway(Math.max(0, v) * 10)),
    ...u16Bytes(roundHalfAway(Math.max(0, a) * 10))]);
}

/** 0x04 GET_JOINT_DETAIL：u8 joint（0~5，或 0xFF = 六轴）。 */
export function payloadGetJointDetail(joint = 0xFF) {
  if (joint !== 0xFF && !(joint >= 0 && joint <= 5)) {
    throw new ProtocolError('joint 取 0~5 或 0xFF(全部)');
  }
  return asU8([joint & 0xFF]);
}

/** 0x22 IK_QUERY：i16 x,y,z(mm×10) + i16 pitch(deg×10) = 8 B（**只解不算**）。 */
export function payloadIkQuery(x, y, z, pitch = 0) {
  return asU8([...i16Bytes(roundHalfAway(x * 10)), ...i16Bytes(roundHalfAway(y * 10)),
    ...i16Bytes(roundHalfAway(z * 10)), ...i16Bytes(roundHalfAway(pitch * 10))]);
}

/** 0x36 GET_PID：u8 joint（0~5）。 */
export function payloadGetPid(joint) {
  if (!(joint >= 0 && joint <= 5)) throw new ProtocolError('joint 取 0~5');
  return asU8([joint & 0xFF]);
}

/** 0x17 SET_BUS_MODE：u8 mode（0=正常 A:AABBB / 1=全A / 2=全B）。 */
export function payloadSetBusMode(mode) {
  if (mode !== 0 && mode !== 1 && mode !== 2) throw new ProtocolError('mode 取 0/1/2');
  return asU8([mode]);
}

/** 0x18 SERVO_INIT：u8 joint（0~5 或 0xFF=全部）。
 *  ⚠ 会重写该轴 PID/限位/中位并 RESET，副作用与文本命令 `i` 相同。 */
export function payloadServoInit(joint = 0xFF) {
  if (joint !== 0xFF && !(joint >= 0 && joint <= 5)) {
    throw new ProtocolError('joint 取 0~5 或 0xFF(全部)');
  }
  return asU8([joint & 0xFF]);
}

/** 0x44 PARAM_LIST：u8 offset（从第几条开始；只给元数据，值用 PARAM_READ 取）。 */
export function payloadParamList(offset = 0) { return asU8([offset & 0xFF]); }

/** 0x50 TEACH_LIST：u8 idx（第几个文件，0 基）。 */
export function payloadTeachList(idx) { return asU8([idx & 0xFF]); }

/** 0x51 TEACH_READ：u8 idx + u16 off + u8 n = 4 B。⚠ n 会被固件钳到 5。 */
export function payloadTeachRead(idx, off, n = TEACH_READ_MAX) {
  if (n < 1) throw new ProtocolError('n >= 1');
  return asU8([idx & 0xFF, ...u16Bytes(off & 0xFFFF), Math.min(n, TEACH_READ_MAX)]);
}

/** 0x53 TEACH_DELETE：u8 idx（写 tombstone，空间用 TEACH_COMPACT 回收）。 */
export function payloadTeachDelete(idx) { return asU8([idx & 0xFF]); }

/** 危险命令的 4 字节 MAGIC 负载。 */
export function payloadMagic() { return asU8(u32le(MAGIC)); }

/** 0x72 ZERO_RESET：u8 flags（bit0=1 同时落盘，即文本 `zr save`）+ u32 magic。
 *  ⚠ 文本 `zr` **没有单轴变体**（一次复位全部六轴）⇒ 这里也不给关节号。 */
export function payloadZeroReset(save = false) {
  return asU8([save ? 1 : 0, ...u32le(MAGIC)]);
}

/**
 * 0x0B TEXT_EXEC：u32 magic + 文本命令 + NUL。
 *
 * 这是"所有指令都支持网口下发"的**那一把钥匙**：负载里就是一行 ASCII，
 * 固件走 COM11 上完全相同的 ProcessCommand，输出改道到内存再分页回传。
 *
 * ⚠ 最长 58 字符（TEXT_EXEC_MAX）。超长固件会钳断 —— 与其静默执行半条命令
 *   （很可能是**另一条**命令！），不如在这里就直接抛错。
 * ⚠ 固件侧安全闸门会挡掉 11 条抢 ETH 引脚的命令与交互式的 `c`，被挡时
 *   应答里带 TEXT_FLAG_GATE，正文是**原因说明**。
 */
export function payloadTextExec(text) {
  if (typeof text !== 'string') throw new ProtocolError('text 必须是字符串');
  const body = text.replace(/[\r\n]/g, ' ').trim();
  if (!body) throw new ProtocolError('命令不能为空');
  if (body.length > TEXT_EXEC_MAX) {
    throw new ProtocolError('命令最长 ' + TEXT_EXEC_MAX + ' 字符，实际 ' + body.length);
  }
  const bytes = [...utf8Encoder.encode(body)];
  if (bytes.length > TEXT_EXEC_MAX) {
    throw new ProtocolError('命令编码后 ' + bytes.length + ' B，超过 ' + TEXT_EXEC_MAX);
  }
  return asU8([...u32le(MAGIC), ...bytes, 0]);
}

/** 0x0C TEXT_FETCH：u16 off + u8 n = 3 B。取上一条 TEXT_EXEC 输出的后续页。 */
export function payloadTextFetch(off, n = TEXT_CHUNK) {
  if (n < 1) throw new ProtocolError('n >= 1');
  if (!(off >= 0 && off <= 0xFFFF)) throw new ProtocolError('off 取 0~65535');
  return asU8([...u16Bytes(off), Math.min(n, TEXT_CHUNK)]);
}

/* ---- 响应解码 ---- */

/** 解 0x84：u8 n + u8 joint0 + n×10 B。单轴与六轴**同一份解包代码**。 */
export function decodeJointDetail(pl) {
  if (pl.length < 2) throw new ProtocolError('JOINT_DETAIL 负载过短: ' + pl.length + ' B');
  const nn = pl[0], j0 = pl[1];
  const need = 2 + nn * DETAIL_SIZE;
  if (pl.length < need) {
    throw new ProtocolError('JOINT_DETAIL 需要 ' + need + ' B，实际 ' + pl.length + ' B');
  }
  const out = [];
  for (let k = 0; k < nn; k++) {
    const o = 2 + k * DETAIL_SIZE;
    out.push({
      joint: nn > 1 ? j0 + k : j0,
      pos_raw: readU16(pl, o),
      speed: readI16(pl, o + 2),
      load: readI16(pl, o + 4),
      voltage: pl[o + 6] / 10,
      temp: pl[o + 7],
      hw_err: pl[o + 8],
      ok: pl[o + 9] !== 0,
      /* 总线是**逐字段瞬时丢读**的（实测 Temp ≈20%，各字段独立）
       * ⇒ 哨兵表示"这一拍没读到"，不是"读数就是 0"，也不是故障。 */
      temp_lost: pl[o + 7] === TEMP_LOST,
      hw_err_lost: pl[o + 8] === HWERR_LOST,
    });
  }
  return out;
}

/** 解 0x85：6×8 B（min, pos, max, zero，均 u16 raw）= 48 B。
 *  ⚠ 这 4 组 raw 来自 Flash 校准记录；**运行时限位**（舵机度）不在这条命令里。 */
export function decodeLimits(pl) {
  if (pl.length < 48) throw new ProtocolError('LIMITS 需要 48 B，实际 ' + pl.length + ' B');
  const out = { min_raw: [], pos_raw: [], max_raw: [], zero_raw: [] };
  for (let k = 0; k < 6; k++) {
    const o = k * 8;
    out.min_raw.push(readU16(pl, o));
    out.pos_raw.push(readU16(pl, o + 2));
    out.max_raw.push(readU16(pl, o + 4));
    out.zero_raw.push(readU16(pl, o + 6));
  }
  /* zero_raw === 0 ⇒ 该轴未做 Home 零点校准（固件回落 180° / raw 2048） */
  out.calibrated = out.zero_raw.map((z) => z !== 0);
  return out;
}

/** 解 0x87：u8 tag_len + char[16] + u8 build + u8 link + u8 n_params = 20 B。
 *  ⚠ link_mode 是**启动冻结值**（LinkCfg_ActiveMode），不是 Flash 里的偏好 ——
 *    两者可能在「改了偏好但还没 RESET」时分叉。 */
export function decodeInfo(pl) {
  if (pl.length < 20) throw new ProtocolError('INFO 需要 20 B，实际 ' + pl.length + ' B');
  const tl = Math.min(pl[0], 16);
  let tag = '';
  for (let i = 1; i <= tl; i++) tag += String.fromCharCode(pl[i]);
  const build = pl[17];
  return { fw_tag: tag, build, link_mode: pl[18], n_params: pl[19],
           build_name: BUILD_NAME[build] || '?' };
}

/** 解 0x8A：u8 flags + u8 run_id = 2 B。只读变量、不碰总线 ⇒ 可 10~50Hz 轮询。 */
export function decodeFlags(pl) {
  if (pl.length < 2) throw new ProtocolError('FLAGS 需要 2 B，实际 ' + pl.length + ' B');
  const raw = pl[0];
  return {
    raw, run_id: pl[1],
    estop: !!(raw & 0x01), traj: !!(raw & 0x02), stream: !!(raw & 0x04),
    corr: !!(raw & 0x08), teach_rec: !!(raw & 0x10), teach_play: !!(raw & 0x20),
  };
}

/** 解 0xA2：6×i16 舵机度×10 = 12 B。 */
export function decodeIkQuery(pl) {
  if (pl.length < 12) throw new ProtocolError('IK_QUERY 需要 12 B，实际 ' + pl.length + ' B');
  const out = [];
  for (let k = 0; k < 6; k++) out.push(readI16(pl, k * 2) / 10);
  return out;
}

/** 解 0xB2：i16 K（deg/mm ×1000）= 2 B。 */
export function decodeGravity(pl) {
  if (pl.length < 2) throw new ProtocolError('GRAVITY 需要 2 B，实际 ' + pl.length + ' B');
  return readI16(pl, 0) / 1000;
}

/** 解 0xB3：u8 en + u16 thr×100 + u8 rounds + u8 pending = 5 B。 */
export function decodeCorrection(pl) {
  if (pl.length < 5) throw new ProtocolError('CORRECTION 需要 5 B，实际 ' + pl.length + ' B');
  return { enabled: pl[0] !== 0, threshold_deg: readU16(pl, 1) / 100,
           max_rounds: pl[3], pending: pl[4] !== 0 };
}

/** 解 0xB6：u8 p + u8 i + u8 d + u8 from_eeprom = 4 B。
 *  ⚠ 底层 SERVO_ReadPidEeprom 形参顺序是 (p, d, i)，API 层已换算成协议顺序
 *    p/i/d —— 到这里就是 p/i/d，不要再换一次（换两次等于没换）。
 *  from_eeprom=false 表示读失败，p/i/d 不可信。 */
export function decodePid(pl) {
  if (pl.length < 4) throw new ProtocolError('PID 需要 4 B，实际 ' + pl.length + ' B');
  return { p: pl[0], i: pl[1], d: pl[2], from_eeprom: pl[3] !== 0 };
}

/** 解 0xC4：u8 total + u8 n + n×10 B（id u8, type u8, vmin i32×1000, vmax i32×1000）。
 *  翻页终止：(offset + n) >= total 或 n === 0。 */
export function decodeParamList(pl) {
  if (pl.length < 2) throw new ProtocolError('PARAM_LIST 负载过短: ' + pl.length + ' B');
  const total = pl[0], nn = pl[1];
  const need = 2 + nn * 10;
  if (pl.length < need) {
    throw new ProtocolError('PARAM_LIST 需要 ' + need + ' B，实际 ' + pl.length + ' B');
  }
  const items = [];
  for (let k = 0; k < nn; k++) {
    const o = 2 + k * 10;
    items.push({ id: pl[o], type: pl[o + 1],
                 vmin: readI32(pl, o + 2) / 1000, vmax: readI32(pl, o + 6) / 1000 });
  }
  return { total, items };
}

/** 解 0xD0：u8 total + u8 idx + u8 valid + char[12] + u32 samples = 19 B。 */
export function decodeTeachList(pl) {
  if (pl.length < 19) throw new ProtocolError('TEACH_LIST 需要 19 B，实际 ' + pl.length + ' B');
  let name = '';
  for (let i = 3; i < 15 && pl[i] !== 0; i++) name += String.fromCharCode(pl[i]);
  return { total: pl[0], idx: pl[1], valid: pl[2] !== 0, name,
           samples: readU32(pl, 15) };
}

/** 解 0xD1：u8 n + n×12 B（6×u16 raw）。
 *  ⚠ n === 0 表示 off 越界 —— 那是**正常读完**，不是错误。 */
export function decodeTeachRead(pl) {
  if (pl.length < 1) throw new ProtocolError('TEACH_READ 负载为空');
  const nn = pl[0];
  const need = 1 + nn * TEACH_SAMPLE_SIZE;
  if (pl.length < need) {
    throw new ProtocolError('TEACH_READ 需要 ' + need + ' B，实际 ' + pl.length + ' B');
  }
  const out = [];
  for (let k = 0; k < nn; k++) {
    const o = 1 + k * TEACH_SAMPLE_SIZE;
    const pt = [];
    for (let j = 0; j < 6; j++) pt.push(readU16(pl, o + j * 2));
    out.push(pt);
  }
  return out;
}

/** 解 0xD5：u8 nfiles + u8 capacity + u32 total + u32 free = 10 B。 */
export function decodeTeachTotal(pl) {
  if (pl.length < 10) throw new ProtocolError('TEACH_TOTAL 需要 10 B，实际 ' + pl.length + ' B');
  return { n_files: pl[0], capacity: pl[1],
           total_samples: readU32(pl, 2), free_bytes: readU32(pl, 6) };
}

/** 解 0x8B / 0x8C：u16 total, u8 flags, u16 off, u8 n, text[n]。
 *  raw 是**字节** —— 跨页拼接必须按字节拼（分页边界可能切在多字节字符中间）。 */
export function decodeTextOut(pl) {
  if (pl.length < 6) throw new ProtocolError('TEXT_OUT 负载过短: ' + pl.length + ' B');
  const total = readU16(pl, 0), flags = pl[2], off = readU16(pl, 3), nn = pl[5];
  if (pl.length < 6 + nn) {
    throw new ProtocolError('TEXT_OUT 需要 ' + (6 + nn) + ' B，实际 ' + pl.length + ' B');
  }
  const raw = pl.slice(6, 6 + nn);
  return {
    raw, total, off,
    truncated: !!(flags & TEXT_FLAG_TRUNC),
    gated: !!(flags & TEXT_FLAG_GATE),
    get text() { return utf8Decoder.decode(raw); },
    get done() { return off + raw.length >= total; },
  };
}

/* ==========================================================================
 * 11. 自检
 * ========================================================================== */

export function selftest() {
  const fails = [];
  let n = 0;
  const chk = (name, cond, extra = '') => {
    n++;
    if (!cond) { fails.push(name + ' ' + extra); console.log('  FAIL ' + name + ' ' + extra); }
  };

  // CRC 标准向量
  chk('crc16("123456789")==0x29B1', crc16('123456789') === 0x29B1,
      '0x' + crc16('123456789').toString(16));

  // 文档里的 5 条 hex 例子（与 Python 版对拍）
  chk('PING seq=0x31', toHex(packLogical(0x31, CMD.PING)) === '31 01 00 38 0D',
      toHex(packLogical(0x31, CMD.PING)));
  chk('PARAM_COUNT seq=0x43',
      toHex(packLogical(0x43, CMD.PARAM_COUNT)) === '43 42 00 CF E3',
      toHex(packLogical(0x43, CMD.PARAM_COUNT)));
  chk('STREAM_JOINTS [0,10,20,30,40,50]',
      toHex(payloadStreamJoints([0, 10, 20, 30, 40, 50])) === '00 00 64 00 C8 00 2C 01 90 01 F4 01',
      toHex(payloadStreamJoints([0, 10, 20, 30, 40, 50])));
  chk('MOVE_JOINTS 180x6',
      toHex(payloadMoveJoints([180, 180, 180, 180, 180, 180])).startsWith('08 07 08 07'),
      toHex(payloadMoveJoints([180, 180, 180, 180, 180, 180])));
  chk('PARAM_WRITE(3, 42.5)',
      toHex(payloadParamWrite(3, 42.5)) === '03 04 A6 00 00',
      toHex(payloadParamWrite(3, 42.5)));

  // ---- 2026-09-23 扩展：与 Python 版逐字节对齐的向量 ----
  chk('TEXT_EXEC("s")', toHex(payloadTextExec('s')) === '5A 5A 5A 5A 73 00',
      toHex(payloadTextExec('s')));
  chk('TEXT_EXEC("m 180 ... 15")',
      toHex(payloadTextExec('m 180 180 180 180 180 180 15')) ===
      '5A 5A 5A 5A 6D 20 31 38 30 20 31 38 30 20 31 38 30 20 31 38 30 20 31 38 30 20 31 38 30 20 31 35 00',
      toHex(payloadTextExec('m 180 180 180 180 180 180 15')));
  chk('TEXT_FETCH(58,58)', toHex(payloadTextFetch(58, 58)) === '3A 00 3A',
      toHex(payloadTextFetch(58, 58)));
  chk('ZERO_RESET(false)', toHex(payloadZeroReset(false)) === '00 5A 5A 5A 5A',
      toHex(payloadZeroReset(false)));
  chk('ZERO_RESET(true)', toHex(payloadZeroReset(true)) === '01 5A 5A 5A 5A',
      toHex(payloadZeroReset(true)));
  chk('MOVE_JOINT_ONE(0,180,30,10)',
      toHex(payloadMoveJointOne(0, 180, 30, 10)) === '00 08 07 2C 01 64 00',
      toHex(payloadMoveJointOne(0, 180, 30, 10)));
  chk('GET_JOINT_DETAIL(3)', toHex(payloadGetJointDetail(3)) === '03',
      toHex(payloadGetJointDetail(3)));
  chk('GET_JOINT_DETAIL(0xFF)', toHex(payloadGetJointDetail(0xFF)) === 'FF',
      toHex(payloadGetJointDetail(0xFF)));
  chk('IK_QUERY(100,0,300,0)',
      toHex(payloadIkQuery(100, 0, 300, 0)) === 'E8 03 00 00 B8 0B 00 00',
      toHex(payloadIkQuery(100, 0, 300, 0)));
  chk('TEACH_READ(1,100,5)', toHex(payloadTeachRead(1, 100, 5)) === '01 64 00 05',
      toHex(payloadTeachRead(1, 100, 5)));
  chk('超长命令必须抛错', (() => {
    try { payloadTextExec('x'.repeat(TEXT_EXEC_MAX + 1)); return false; } catch { return true; }
  })());

  /* 解码往返：JOINT_DETAIL 六轴 62 B */
  const detPl = Uint8Array.from([6, 0]);
  {
    const b = [];
    for (let k = 0; k < 6; k++) {
      b.push(...u16Bytes(2048), ...i16Bytes(-30), ...i16Bytes(12), 120, 42, 0, 1);
    }
    const d = decodeJointDetail(Uint8Array.from([...detPl, ...b]));
    chk('decodeJointDetail 6 轴 / 62 B', d.length === 6 && !d[0].temp_lost &&
        Math.abs(d[0].voltage - 12.0) < 1e-9 && d[0].speed === -30,
        'n=' + d.length + ' v=' + d[0].voltage);
  }
  {
    /* 丢读轴：temp/hw_err 都是 0xFF 哨兵 */
    const d = decodeJointDetail(Uint8Array.from([1, 2, 0, 0, 0, 0, 0, 0, 0, 0xFF, 0xFF, 0]));
    chk('decodeJointDetail 丢读哨兵', d[0].temp_lost && d[0].hw_err_lost && !d[0].ok);
  }
  chk('decodeFlags 位图',
      decodeFlags(Uint8Array.from([0x0F, 0x5A])).traj &&
      decodeFlags(Uint8Array.from([0x0F, 0x5A])).corr &&
      decodeFlags(Uint8Array.from([0x0E, 0x5A])).estop === false);
  chk('decodeTextOut 分页头',
      (() => {
        const o = decodeTextOut(Uint8Array.from([0x05, 0x00, 0x01, 0x00, 0x00, 0x03, 0x61, 0x62, 0x63]));
        return o.total === 5 && o.truncated && !o.gated && o.text === 'abc' && !o.done;
      })());
  chk('PARAM_LIST 62 B 上限', decodeParamList(Uint8Array.from([24, 6,
      ...Array.from({ length: 60 }, () => 0)])).items.length === 6);

  // 往返
  const rt = unpackLogical(packLogical(0x11, CMD.STOP));
  chk('往返 seq/cmd/len', rt.seq === 0x11 && rt.cmd === CMD.STOP && rt.payload.length === 0);

  // CRC 错必须抛
  let threw = false;
  try {
    const bad = packLogical(1, CMD.PING);
    bad[4] ^= 0xFF;
    unpackLogical(bad);
  } catch (e) { threw = e instanceof ProtocolError; }
  chk('CRC 错抛 ProtocolError', threw);

  // 长度不符抛
  threw = false;
  try { unpackLogical(Uint8Array.from([1, 2, 3, 4])); } catch (e) { threw = true; }
  chk('长度<5 抛', threw);

  threw = false;
  try { unpackLogical(Uint8Array.from([1, 2, 5, 0, 0])); } catch (e) { threw = true; }
  chk('长度不符抛', threw);

  // roundHalfAway：负半轴必须远离零
  chk('roundHalfAway(0.5)==1', roundHalfAway(0.5) === 1);
  chk('roundHalfAway(-0.5)==-1', roundHalfAway(-0.5) === -1, String(roundHalfAway(-0.5)));
  chk('roundHalfAway(2.5)==3', roundHalfAway(2.5) === 3);
  chk('roundHalfAway(-2.5)==-3', roundHalfAway(-2.5) === -3);
  chk('roundHalfAway(-1.234)==-1', roundHalfAway(-1.234) === -1);
  chk('roundHalfAway(-1.567)==-2', roundHalfAway(-1.567) === -2);

  // 静默钳制
  chk('i16 钳上', readI16(Uint8Array.from(i16Bytes(99999)), 0) === 32767);
  chk('i16 钳下', readI16(Uint8Array.from(i16Bytes(-99999)), 0) === -32768);

  // decodeStatus
  const stPl = new Uint8Array(21);
  [180, 180, -32768, 200, 180, 180].forEach((v, j) => {
    const b = i16Bytes(v * 10 === v * 10 ? (v === -32768 ? -32768 : v * 10) : 0);
    stPl[2 * j] = b[0]; stPl[2 * j + 1] = b[1];
  });
  const st = decodeStatus(stPl);
  chk('status J3 null', st.joints[2] === null, String(st.joints[2]));
  chk('status J1=180', st.joints[0] === 180, String(st.joints[0]));
  chk('failedJoints==[3]', JSON.stringify(failedJoints(st)) === '[3]', JSON.stringify(failedJoints(st)));
  chk('flags 默认 read_ok=false', st.read_ok === false);

  stPl[20] = 0x0B;
  const st2 = decodeStatus(stPl);
  chk('flags 0x0B → estop/traj/read_ok, corr=false',
      st2.estop === true && st2.traj === true && st2.corr === false && st2.read_ok === true);

  // decodeFk 顺序相反 + 偏移 20 裸值
  const fkPl = new Uint8Array(21);
  fkPl[0] = 0x6C; fkPl[1] = 0x02;    // x = 620/10 = 62.0
  fkPl[20] = 0x01;
  const fk = decodeFk(fkPl);
  chk('fk x=62', fk.x === 62, String(fk.x));
  chk('fk read_ok===1 裸值', fk.read_ok === 1);
  chk('fk joints[0]==0', fk.joints[0] === 0, String(fk.joints[0]));

  // param
  const pr = decodeParamRead(Uint8Array.from([3, ...i32Bytes(42500)]));
  chk('paramRead id=3 v=42.5', pr.id === 3 && pr.value === 42.5, JSON.stringify(pr));
  chk('paramCount(24)', decodeParamCount(Uint8Array.from([24])) === 24);

  // JointCalib
  const cal = new JointCalib();
  const lg = cal.toLogical([180, 180, 180, 180, 180, 180]);
  chk('calib 全 180 → 全 0', lg.every((v) => v === 0), JSON.stringify(lg));
  const sv = cal.toServo([0, 0, 0, 0, 0, 0]);
  chk('calib 全 0 → 全 180', sv.every((v) => v === 180), JSON.stringify(sv));
  const lg2 = cal.toLogical([170, 190, 180, 180, 180, 180]);
  chk('J1 dir=-1: 170→+10', Math.abs(lg2[0] - 10) < 1e-9, String(lg2[0]));
  chk('J2 dir=+1: 190→+10', Math.abs(lg2[1] - 10) < 1e-9, String(lg2[1]));
  chk('wrap180(190)==-170', wrap180(190) === -170, String(wrap180(190)));
  chk('wrap180(-180)==180', wrap180(-180) === 180, String(wrap180(-180)));

  // CAN 分片往返
  const payloads = [
    new Uint8Array(0), Uint8Array.from([1]), payloadStreamJoints([0, 0, 0, 0, 0, 0]),
    payloadMoveJoints([1, 2, 3, 4, 5, 6]), payloadParamWrite(3, 42.5),
  ];
  let canOk = true;
  for (const pl of payloads) {
    const frames = canBuildFrames(0x42, CMD.STREAM_JOINTS, pl);
    const r = new CanReassembler();
    let got = null;
    for (const f of frames) { const x = r.feed(f); if (x) got = x; }
    if (!got || !got.crcOk || got.seq !== 0x42 ||
        toHex(got.payload) !== toHex(pl)) canOk = false;
  }
  chk('CAN 分片→重组 5 种负载', canOk);

  // CAN 乱序
  const frames = canBuildFrames(7, CMD.MOVE_JOINTS, payloadMoveJoints([10, 20, 30, 40, 50, 60]));
  const r2 = new CanReassembler();
  let got2 = null;
  for (const f of [frames[0], ...frames.slice(1).reverse()]) {
    const x = r2.feed(f); if (x) got2 = x;
  }
  chk('CAN 乱序重组（bxCAN 三邮箱）', got2 !== null && got2.crcOk && got2.seq === 7,
      got2 ? 'seq=' + got2.seq : 'null');

  // FrameParser 分帧
  const fp = new FrameParser();
  const wire = buildFrame(0x31, CMD.PING);
  const types = [];
  for (const b of wire) { const r = fp.feed(b, 0); if (r) types.push(r.type); }
  chk('FrameParser 认出 frame', types.includes('frame'), JSON.stringify(types));

  const fp2 = new FrameParser();
  const texts = [];
  for (const b of new TextEncoder().encode('OK\r\n')) {
    const r = fp2.feed(b, 0);
    if (r && r.type === 'text') texts.push(r.byte);
  }
  chk('FrameParser 文本原样交还', texts.length === 4, String(texts.length));

  const fp3 = new FrameParser();
  const bad = buildFrame(1, CMD.PING);
  bad[bad.length - 1] ^= 0xFF;
  let crcErr = false;
  for (const b of bad) { const r = fp3.feed(b, 0); if (r && r.type === 'crc_err') crcErr = true; }
  chk('FrameParser 认出 crc_err', crcErr);

  // 参数表
  chk('PARAM_TABLE 24 项', Object.keys(PARAM_TABLE).length === AP_PARAM_COUNT,
      String(Object.keys(PARAM_TABLE).length));
  chk('PARAM_TABLE[42] 存在', !!PARAM_TABLE[42]);

  console.log('\n  protocol.js 自检：' + n + ' 项，' +
              (fails.length ? '失败 ' + fails.length + ' 项' : '全部通过'));
  return fails.length ? 1 : 0;
}

// ⚠ basename 精确比较（别用 endsWith：`my_protocol.js` 会误触发）
const _script = (typeof process !== 'undefined' && process.argv && process.argv[1]) || '';
if (_script.split(/[\\/]/).pop() === 'protocol.js') {
  process.exit(selftest());
}
