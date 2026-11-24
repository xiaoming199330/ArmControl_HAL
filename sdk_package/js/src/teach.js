/**
 * teach.js — 主从臂「遥操 + 录制 + 回放」示教
 *
 * 分工：
 *   TeleopTeachSession  一边遥操一边把**从臂实际轨迹**录下来
 *   Trajectory          轨迹容器（重采样 / 平滑 / 裁剪 / 变速 / 体检 / 四种格式）
 *   PlaybackSession     回放（先接近起点 → 按轨迹流式下发 → E-Stop 轮询）
 *   FirmwareTeach       固件侧录音命令封装（⚠ 有 80s / 4000 点上限，见下）
 *
 * 为什么 PC 侧录制更好
 * -------------------
 *   固件 `teach rec`：50 Hz、**最多 4000 点 ≈ 80 s**，存 Flash 时阻塞 1~3 s，
 *   而且"三种丢录音方式"（`rec clear` / 断电 / 再进 `teach`）都得小心。
 *   PC 侧录制**没有长度上限**，也不占板子 Flash ⇒ 优先用这条。
 *
 * 回放的两个坑（已内置对策）
 * -------------------------
 *   1. 直接下发轨迹第一帧，若与从臂当前位姿差 >25°，固件**整帧拒收** ⇒ 永远回放不动。
 *      对策：**approach** —— 先 MOVE_JOINTS 低速走到轨迹起点，再进流模式。
 *   2. 流断 >500 ms 从臂自锁。对策：进入流前先喂几帧预热，把基准建立起来。
 *
 * 与 Python 版 `arm_sdk/teach.py` 同语义（.npz 仅 Python 版支持，见文件末说明）。
 */

import { NUM_JOINTS, ErrCode, JointCalib, wrap180 } from './protocol.js';
import { TeleopSession, TeleopConfig, FW_MAX_JUMP_DEG } from './teleop.js';

export class TeachError extends Error {
  constructor(msg) { super(msg); this.name = 'TeachError'; }
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const r6 = (v) => Math.round(v * 1e6) / 1e6;

/**
 * 采样率的可测上限（Hz）。
 *
 * 录制器用 `Date.now()` 打时间戳 ⇒ 时钟分辨率 1 ms ⇒ **超过 1000 Hz 的采样率测不出来**。
 * 强行算会得到"时长 1 ms、20 点 ⇒ 19000 Hz"这种荒谬值，并被写进轨迹文件。
 * 所以 `Trajectory.rate` 超过这个上限时回 0（未知），不是夹到 1000 —— 因为
 * "测不出来"和"恰好 1000 Hz"是两回事，回调会让调用方误以为真有 1000 Hz。
 */
export const MAX_MEASURABLE_HZ = 1000;

/* ==========================================================================
 * Trajectory
 * ========================================================================== */

export class Trajectory {
  /**
   * @param {{name?:string, hz?:number, units?:string, frames?:number[][],
   *          source?:string, meta?:object}} [fields]
   */
  constructor(fields = {}) {
    this.name = fields.name ?? 'traj';
    /** 名义采样率（实际时间戳以 frames[i][0] 为准）。 */
    this.hz = fields.hz ?? 50;
    /** 'logical_deg' | 'servo_deg' —— 决定回放走哪条路。 */
    this.units = fields.units ?? 'logical_deg';
    this.source = fields.source ?? 'unknown';
    /** 每项 = [t, [j1..j6]] */
    this.frames = fields.frames ? fields.frames.map((f) => [f[0], [...f[1]]]) : [];
    this.meta = fields.meta ? { ...fields.meta } : {};
    this.created = fields.created ?? new Date().toISOString();
  }

  get length() { return this.frames.length; }
  get duration() { return this.frames.length ? this.frames[this.frames.length - 1][0] : 0; }

  /**
   * 实测采样率（Hz）。**测不出来时回 0 = "未知"，不回假数字。**
   *
   * ⚠ 为什么要有上限：`TeachRecorder` 用 `Date.now()` 打时间戳，分辨率只有 **1 ms**。
   * 如果喂点的速度超过 1000 Hz（例如一次性同步喂 20 点、或从内存重放），
   * 时间跨度会缩到 0 或 1 ms，`(n-1)/duration` 会算出 **19000 这种荒谬值**——
   * 它会被写进轨迹文件，后面回放/加速时就按这个假 hz 排时间。
   * 超过时钟分辨率的测量**在物理上就是不可信的**，所以这里直接判为未知（0），
   * 交给 `TeachRecorder.trajectory()` 回落到名义 `hz`。
   */
  get rate() {
    const d = this.duration;
    if (!(d > 0)) return 0;
    const r = (this.frames.length - 1) / d;
    return r > MAX_MEASURABLE_HZ ? 0 : r;
  }
  get startPose() { return this.frames.length ? [...this.frames[0][1]] : null; }
  get endPose() { return this.frames.length ? [...this.frames[this.frames.length - 1][1]] : null; }
  get times() { return this.frames.map((f) => f[0]); }
  get jointsMatrix() { return this.frames.map((f) => f[1]); }

  append(t, q) {
    if (q.length !== NUM_JOINTS) throw new TeachError('每帧必须 6 个关节');
    this.frames.push([Number(t), Array.from(q, Number)]);
    return this;
  }

  copy() {
    return new Trajectory({
      name: this.name, hz: this.hz, units: this.units, source: this.source,
      frames: this.frames, meta: this.meta, created: this.created,
    });
  }

  /* ---- 变换 ---- */

  /** 线性插值重采样到 hz（或指定 duration）。 */
  resample(hz, duration = null) {
    if (!this.frames.length) return this.copy();
    const dur = duration ?? this.duration;
    const n = Math.max(2, Math.floor(dur * hz) + 1);
    const dt = dur / (n - 1);
    const src = this.frames;
    const out = new Trajectory({ name: this.name, hz, units: this.units,
                                 source: this.source + '+resample', meta: this.meta });
    let k = 0;
    for (let i = 0; i < n; i++) {
      const t = dl(dt * i, dur);
      while (k + 1 < src.length && src[k + 1][0] < t) k++;
      const a = src[k];
      const b = src[Math.min(k + 1, src.length - 1)];
      const span = (b[0] - a[0]) || 1;
      const u = Math.max(0, Math.min(1, (t - a[0]) / span));
      out.append(t, a[1].map((v, j) => v + (b[1][j] - v) * u));
    }
    return out;
  }

  /** 移动平均平滑（window 取奇数，3~7 常用）。首尾保持不变。 */
  smooth(window = 3) {
    if (window < 3 || !this.frames.length) return this.copy();
    let w = window % 2 === 0 ? window + 1 : window;
    const half = Math.floor(w / 2);
    const qs = this.jointsMatrix;
    const n = this.frames.length;
    const out = new Trajectory({ name: this.name + '_smooth', hz: this.hz,
                                 units: this.units, source: this.source, meta: this.meta });
    for (let i = 0; i < n; i++) {
      const lo = Math.max(0, i - half), hi = Math.min(n, i + half + 1);
      const q = [];
      for (let j = 0; j < NUM_JOINTS; j++) {
        let s = 0;
        for (let k = lo; k < hi; k++) s += qs[k][j];
        q.push(s / (hi - lo));
      }
      out.append(this.frames[i][0], q);
    }
    return out;
  }

  /** 按时间裁剪 [t0, t1)（新时间轴从 0 起）。 */
  slice(t0 = 0, t1 = null) {
    const hi = t1 ?? this.duration;
    const out = new Trajectory({ name: this.name + '_slice', hz: this.hz,
                                 units: this.units, source: this.source, meta: this.meta });
    for (const [t, q] of this.frames) {
      if (t >= t0 && t < hi) out.append(t - t0, q);
    }
    return out;
  }

  /** 时间轴缩放（factor>1 变慢）。 */
  scaleTime(factor) {
    const out = new Trajectory({ name: this.name + '_tm', hz: this.hz / factor,
                                 units: this.units, source: this.source, meta: this.meta });
    for (const [t, q] of this.frames) out.append(t * factor, q);
    return out;
  }

  /** 每轴相邻帧最大步长（deg）。 */
  maxStep() {
    const out = new Array(NUM_JOINTS).fill(0);
    for (let i = 1; i < this.frames.length; i++) {
      for (let j = 0; j < NUM_JOINTS; j++) {
        const d = Math.abs(this.frames[i][1][j] - this.frames[i - 1][1][j]);
        if (d > out[j]) out[j] = d;
      }
    }
    return out;
  }

  /** 每轴是否越出 [lo, hi]（按逻辑角）。 */
  checkLimits(limits) {
    const bad = [];
    for (let j = 0; j < NUM_JOINTS && j < limits.length; j++) {
      const [lo, hi] = limits[j];
      for (const [, q] of this.frames) {
        if (q[j] < lo || q[j] > hi) { bad.push(j + 1); break; }
      }
    }
    return bad;
  }

  /** 体检：点数 / 采样率 / 单帧跳变 / 软限位 / 时间单调。 */
  validate(limits = null) {
    const warnings = [];
    if (this.frames.length < 2) warnings.push('轨迹点数 < 2，放不了');
    const rate = this.rate;
    if (this.frames.length >= 2 && (rate < 5 || rate > 200)) {
      warnings.push(`采样率 ${rate.toFixed(1)} Hz 异常（正常 20~100 Hz）`);
    }
    const ms = this.maxStep();
    ms.forEach((d, j) => {
      if (d > FW_MAX_JUMP_DEG) {
        warnings.push(`J${j + 1} 单帧最大跳变 ${d.toFixed(1)}° > ${FW_MAX_JUMP_DEG}°` +
                      '（回放会被固件整帧拒收，需先重采样或平滑）');
      }
    });
    for (let i = 1; i < this.frames.length; i++) {
      if (this.frames[i][0] <= this.frames[i - 1][0]) {
        warnings.push(`时间戳非单调（第 ${i} 帧）`);
        break;
      }
    }
    if (limits && limits.length) {
      const bad = this.checkLimits(limits);
      if (bad.length) warnings.push('越软限位轴：J' + bad.join(',J'));
    }
    return { ok: warnings.length === 0, warnings };
  }

  summary() {
    if (!this.frames.length) return `${this.name}: 空轨迹`;
    const ms = this.maxStep();
    return `${this.name}: ${this.frames.length} 点 / ${this.duration.toFixed(2)}s / ` +
           `${this.rate.toFixed(1)} Hz / ${this.units} / src=${this.source}\n` +
           `           单帧最大步长 ` + ms.map((v, j) => `J${j + 1}=${v.toFixed(2)}°`).join(' ');
  }

  /* ---- 序列化 ---- */

  toJSON() {
    return {
      format: 'arm-trajectory/1', name: this.name, hz: this.hz, units: this.units,
      source: this.source, created: this.created, meta: this.meta,
      frames: this.frames.map(([t, q]) => [r6(t), q.map(r6)]),
    };
  }

  static fromJSON(d) {
    if (!d || !Array.isArray(d.frames)) throw new TeachError('不是合法轨迹 JSON');
    return new Trajectory({
      name: d.name ?? 'traj', hz: d.hz ?? 50, units: d.units ?? 'logical_deg',
      source: d.source ?? 'json', meta: d.meta ?? {}, created: d.created,
      frames: d.frames.map(([t, q]) => [Number(t), q.map(Number)]),
    });
  }

  static fromJSONText(text) { return Trajectory.fromJSON(JSON.parse(text)); }

  /** CSV：`t,j1,j2,j3,j4,j5,j6`（Excel / LeRobot 友好）。 */
  toCSV() {
    const lines = [`# name=${this.name} hz=${this.hz} units=${this.units} ` +
                   `source=${this.source} points=${this.frames.length}`,
                   't,j1,j2,j3,j4,j5,j6'];
    for (const [t, q] of this.frames) {
      lines.push([r6(t), ...q.map(r6)].join(','));
    }
    return lines.join('\n') + '\n';
  }

  static fromCSV(text) {
    const rows = text.split(/\r?\n/).map((s) => s.trim()).filter(Boolean);
    const t = new Trajectory({ source: 'csv' });
    let headerSeen = false;
    for (const row of rows) {
      if (row.startsWith('#')) {
        const m = /name=(\S+)/.exec(row);
        if (m) t.name = m[1];
        const h = /hz=([\d.]+)/.exec(row);
        if (h) t.hz = Number(h[1]);
        const u = /units=(\S+)/.exec(row);
        if (u) t.units = u[1];
        continue;
      }
      const parts = row.split(',').map((s) => s.trim());
      if (!headerSeen && /[a-zA-Z]/.test(row)) { headerSeen = true; continue; }
      if (parts.length < 7) continue;
      const nums = parts.slice(0, 7).map(Number);
      if (nums.some((v) => !Number.isFinite(v))) continue;
      t.append(nums[0], nums.slice(1));
    }
    return t;
  }

  /** JSONL：一行一帧（超大轨迹流式写，不占内存峰值）。 */
  toJSONL() {
    const head = JSON.stringify({ format: 'arm-trajectory-jsonl/1', name: this.name,
      hz: this.hz, units: this.units, source: this.source, meta: this.meta });
    const body = this.frames.map(([t, q]) => JSON.stringify([r6(t), q.map(r6)]));
    return [head, ...body].join('\n') + '\n';
  }

  static fromJSONL(text) {
    const t = new Trajectory({ source: 'jsonl' });
    for (const line of text.split(/\r?\n/)) {
      const s = line.trim();
      if (!s) continue;
      const row = JSON.parse(s);
      if (!Array.isArray(row)) {
        if (row.name) t.name = row.name;
        if (row.hz) t.hz = row.hz;
        if (row.units) t.units = row.units;
        if (row.meta) t.meta = row.meta;
        if (row.source) t.source = row.source;
        continue;
      }
      t.append(row[0], row[1]);
    }
    return t;
  }

  static detectFormat(path) {
    const p = String(path).toLowerCase();
    if (p.endsWith('.json')) return 'json';
    if (p.endsWith('.jsonl')) return 'jsonl';
    if (p.endsWith('.csv')) return 'csv';
    if (p.endsWith('.npz') || p.endsWith('.npy')) return 'npz';
    throw new TeachError('不认识的扩展名: ' + path + '（支持 .json/.jsonl/.csv）');
  }
}

const dl = (a, b) => Math.min(a, b);

/* ==========================================================================
 * 录制器
 * ========================================================================== */

/**
 * PC 侧录制器。可直接当 TeleopSession 的 onFrame 回调。
 *
 * mode:
 *   'follower' 录**从臂目标**（推荐 —— 这是真正会执行的东西）
 *   'leader'   录主臂逻辑角
 *   'both'     两个都录（meta.both 里存主臂）
 */
export class TeachRecorder {
  constructor(opts = {}) {
    this.name = opts.name ?? 'traj';
    this.mode = opts.mode ?? 'follower';
    this.hz = opts.hz ?? 50;
    this.minIntervalS = opts.minIntervalS ?? 0;
    this.maxPoints = opts.maxPoints ?? 0;      // 0 = 不限（PC 侧本就没上限）
    this.onPoint = opts.onPoint || null;
    this.reset();
  }

  reset() {
    this.traj = new Trajectory({ name: this.name, hz: this.hz, units: 'logical_deg',
                                 source: 'teleop+' + this.mode });
    this.leadFrames = [];
    this._t0 = 0;
    this._active = false;
    this._lastT = 0;
    this.dropped = 0;
  }

  start() {
    this.reset();
    this._t0 = Date.now() / 1000;
    this._active = true;
    return this;
  }

  stop() {
    this._active = false;
    if (this.mode === 'both' && this.leadFrames.length) {
      this.traj.meta.leader_frames = this.leadFrames;
    }
    return this;
  }

  get isRecording() { return this._active; }
  get count() { return this.traj.length; }
  get duration() { return this.traj.duration; }

  get summary() {
    return `录 ${this.traj.length} 点 / ${this.traj.duration.toFixed(2)}s / ` +
           `${this.traj.rate.toFixed(1)} Hz` +
           (this.dropped ? ` （跳过 ${this.dropped} 点，低于 ${this.minIntervalS}s）` : '');
  }

  /**
   * 记一点。可当 onFrame(leadLog, target) 直接传进 TeleopSession。
   * @returns {boolean} 是否记下来了
   */
  record(q, t = null) {
    if (!this._active) return false;
    const ts = t ?? (Date.now() / 1000 - this._t0);
    if (this.minIntervalS > 0 && ts - this._lastT < this.minIntervalS) {
      this.dropped++;
      return false;
    }
    if (this.maxPoints > 0 && this.traj.length >= this.maxPoints) return false;
    this._lastT = ts;
    this.traj.append(ts, q);
    if (this.onPoint) { try { this.onPoint(this.traj.length, ts); } catch { /* 忽略 */ } }
    return true;
  }

  /** 当 TeleopSession.onFrame 用。 */
  asCallback() {
    return (leadLog, target) => {
      if (this.mode === 'leader') this.record(leadLog);
      else if (this.mode === 'both') { this.leadFrames.push([...leadLog]); this.record(target); }
      else this.record(target);
    };
  }

  trajectory() {
    const t = this.traj.copy();
    t.hz = this.traj.rate > 0 ? this.traj.rate : this.hz;
    return t;
  }
}

/* ==========================================================================
 * 遥操 + 录制
 * ========================================================================== */

/** 一边遥操一边录。比起单独用 TeleopSession，就多挂了个录制器。 */
export class TeleopTeachSession {
  constructor(leader, follower, opts = {}) {
    this._tp = new TeleopSession(leader, follower, {
      cfg: opts.cfg || new TeleopConfig({ hz: opts.hz ?? 50 }),
      leaderCalib: opts.leaderCalib,
      followerCalib: opts.followerCalib,
      onEvent: opts.onEvent,
    });
    this.rec = new TeachRecorder({
      name: opts.name ?? 'traj',
      mode: opts.mode ?? 'follower',
      hz: opts.hz ?? 50,
      minIntervalS: opts.minIntervalS ?? 0,
      maxPoints: opts.maxPoints ?? 0,
      
    });
    this.out = opts.out || null;
    this.lastTraj = null;
    this._origOnFrame = this._tp.onFrame;
    this._tp.onFrame = (leadLog, target) => {
      if (this._origOnFrame) { try { this._origOnFrame(leadLog, target); } catch { /* 忽略 */ } }
      this.rec.asCallback()(leadLog, target);
    };
  }

  get stats() { return this._tp.stats; }
  get leader() { return this._tp.leader; }
  get follower() { return this._tp.follower; }
  get isRecording() { return this.rec.isRecording; }

  async start() {
    this.rec.start();
    await this._tp.start();
    return this;
  }

  async stop() {
    if (this.rec.isRecording) this.rec.stop();
    await this._tp.stop();
  }

  /** 只遥操不录。 */
  async run(opts = {}) { return this._tp.run(opts); }

  /**
   * 遥操 + 录制。返回轨迹（同时也存到 out）。
   * @param {{durationS?:number, out?:string, until?:()=>boolean, onTick?:Function}} [opts]
   */
  async record(opts = {}) {
    if (!this.rec.isRecording) this.rec.start();
    try {
      await this._tp.run(opts);
    } finally {
      if (this.rec.isRecording) this.rec.stop();
    }
    const traj = this.rec.trajectory();
    this.lastTraj = traj;
    const path = opts.out ?? this.out;
    if (path) await saveTrajectory(traj, path);
    return traj;
  }
}

/* ==========================================================================
 * 回放
 * ========================================================================== */

export class PlaybackStats {
  constructor() {
    this.sent = 0; this.loops = 0; this.slewClamped = 0;
    this.ackLimit = 0; this.ackOk = 0; this.aborted = false;
    this.tStart = 0; this.tEnd = 0;
  }
  get elapsed() { return this.tEnd - this.tStart; }
  get fps() { return this.elapsed > 0 ? this.sent / this.elapsed : 0; }
  summary() {
    return `回放 发送 ${this.sent} 帧 / ${this.elapsed.toFixed(1)}s (${this.fps.toFixed(1)} Hz)` +
           ` / ${this.loops} 轮 | ACK ok=${this.ackOk} limit=${this.ackLimit} | ` +
           `限速 ${this.slewClamped}${this.aborted ? ' | 已中止' : ''}`;
  }
}

export class PlaybackSession {
  /**
   * @param {import('./client.js').ArmClient} arm
   * @param {{cfg?:TeleopConfig, slewDegPerSec?:number, estopCheckS?:number,
   *          dryRun?:boolean, calib?:JointCalib, onFrame?:Function,
   *          onEvent?:(s:string)=>void}} [opts]
   */
  constructor(arm, opts = {}) {
    this.arm = arm;
    this.cfg = opts.cfg || new TeleopConfig({ hz: 50 });
    this.calib = opts.calib || new JointCalib();
    this.slew = opts.slewDegPerSec ?? 0;
    this.estopCheckS = opts.estopCheckS ?? 0.5;
    this.dryRun = !!opts.dryRun;
    this.onFrame = opts.onFrame || null;
    this.onEventCb = opts.onEvent || null;
    this.stats = new PlaybackStats();
    this._abort = false;
    this._last = null;
  }

  _event(m) { if (this.onEventCb) { try { this.onEventCb(m); } catch { /* 忽略 */ } } }
  requestStop() { this._abort = true; }

  /** 低速走到轨迹起点（否则第一帧就 >25° 被拒）。 */
  async _approach(targetLogical, speed, tolDeg, timeoutS) {
    const targetServo = this.calib.toServo(targetLogical);
    this._event('接近段：MOVE_JOINTS 到轨迹起点，v=' + speed.toFixed(0));
    const r = await this.arm.moveJoints(targetServo, speed);
    if (!r.ok) { this._event('⚠ 接近段失败：' + r.toString()); return false; }

    const t0 = Date.now() / 1000;
    let last = null;
    while (Date.now() / 1000 - t0 < timeoutS) {
      await sleep(60);
      const st = await this.arm.getStatus({ timeout: this.cfg.readTimeout });
      if (!st) continue;
      const sv = st.read_ok && st.joints.every((v) => v !== null) ? st.joints : null;
      if (sv === null) continue;
      const err = Math.max(...sv.map((v, j) => Math.abs(wrap180(v - targetServo[j]))));
      if (last !== null && err < tolDeg) return true;
      last = sv;
    }
    this._event(`⚠ 接近段未在 ${timeoutS}s 内收敛，按当前位姿继续`);
    return false;
  }

  /**
   * 回放轨迹。
   * @param {Trajectory} traj
   * @param {{speed?:number, loop?:number, approach?:boolean, approachSpeed?:number,
   *          approachTolDeg?:number, approachTimeoutS?:number, warmupFrames?:number}} [opts]
   */
  async play(traj, opts = {}) {
    const speed = opts.speed ?? 1.0;
    const loop = Math.max(0, opts.loop ?? 1);          // 0 = 无限
    const approach = opts.approach !== false;
    const approachSpeed = opts.approachSpeed ?? 20;
    const approachTol = opts.approachTolDeg ?? 2.0;
    const approachTimeout = opts.approachTimeoutS ?? 8.0;
    const warmup = opts.warmupFrames ?? 3;

    if (traj.units !== 'logical_deg') {
      throw new TeachError(`回放要求 units='logical_deg'，当前是 '${traj.units}'` +
                           '（STREAM_JOINTS 用逻辑角；舵机度请先转换）');
    }
    const v = traj.validate();
    if (!v.ok) {
      this._event('⚠ 轨迹体检有告警：');
      v.warnings.forEach((w) => this._event('   · ' + w));
    }

    const hz = Math.max(this.cfg.hz, traj.rate || this.cfg.hz) * speed;
    const rep = traj.resample(hz);
    const period = 1 / hz;
    const cap = Math.min(
      this.slew > 0 ? this.slew * period : Infinity,
      FW_MAX_JUMP_DEG * 0.8,                            // 留 20% 余量，别贴死门限
    );

    this.stats = new PlaybackStats();
    this._abort = false;
    this.stats.tStart = Date.now() / 1000;

    this._event(`回放 ${traj.name}：${traj.length} 点 → ${rep.length} 点 @ ${hz.toFixed(0)} Hz, ` +
                `speed=${speed.toFixed(2)}, loop=${loop || '∞'}`);

    const startLog = rep.startPose;
    if (approach) {
      const ok = await this._approach(startLog, approachSpeed, approachTol, approachTimeout);
      if (!ok) this._event('按当前位姿继续（若一直拒帧，请手动把臂摆到起点附近）');
    }

    // 预热：先喂几帧把从臂的流基准建立起来（否则第一帧就可能被判大跳变）
    for (let i = 0; i < warmup; i++) {
      await this.arm.streamJoints(startLog, false);
      this.stats.sent++;
      await sleep(period * 1000);
    }
    this._last = [...startLog];

    let lastEstopCheck = Date.now() / 1000;
    let rounds = 0;

    try {
      while (!this._abort && (loop === 0 || rounds < loop)) {
        const t0 = Date.now() / 1000;
        let next = Date.now() / 1000;

        for (let i = 0; i < rep.length; i++) {
          if (this._abort) break;
          const q = rep.frames[i][1];

          // 单帧硬限速（别贴 25° 门限）
          let out = [...q];
          if (this._last) {
            out = q.map((vv, j) => {
              const d = vv - this._last[j];
              if (Math.abs(d) > cap) { this.stats.slewClamped++; return this._last[j] + Math.sign(d) * cap; }
              return vv;
            });
          }
          await this.arm.streamJoints(out, false);
          this.stats.sent++;
          this._last = out;
          if (this.onFrame) { try { this.onFrame(i, out); } catch { /* 忽略 */ } }

          // E-Stop 轮询（别太频繁，否则抢带宽）
          const now = Date.now() / 1000;
          if (this.estopCheckS > 0 && now - lastEstopCheck >= this.estopCheckS) {
            lastEstopCheck = now;
            const st = await this.arm.getStatus({ timeout: this.cfg.readTimeout });
            if (st && st.estop) {
              this._event('🛑 急停激活，中止回放');
              this._abort = true;
              break;
            }
          }

          next += period;
          const slp = (next - Date.now() / 1000) * 1000;
          if (slp > 0) await sleep(slp);
          else next = Date.now() / 1000;
        }

        for (const a of this.arm.pollStreamAcks()) {
          if (a.ok) this.stats.ackOk++;
          else if (a.err === ErrCode.LIMIT) this.stats.ackLimit++;
        }
        rounds++;
        this.stats.loops = rounds;
        if (this._abort) break;
        // 补偿每轮累计的漂移
        const drift = (Date.now() / 1000 - t0) * 1000;
        if (drift > 200) this._event(`一轮耗时 ${(drift / 1000).toFixed(2)}s（目标 ${rep.duration.toFixed(2)}s）`);
      }
    } finally {
      this.stats.tEnd = Date.now() / 1000;
      this.stats.aborted = this._abort;
      this._event('回放结束：' + this.stats.summary());
    }
    return this.stats;
  }
}

/* ==========================================================================
 * 固件侧录音（console 命令封装）
 * ========================================================================== */

/**
 * 固件 `teach rec` 路线的封装。
 *
 * ⚠ 固件限制：50 Hz、**最多 4000 点 ≈ 80 s**、`save` 阻塞 1~3 s、
 *   三种丢录音方式（`rec clear` / 断电 / 再进 `teach`）。
 *   ⇒ 长轨迹请用 PC 侧录制（TeachRecorder）。
 */
export class FirmwareTeach {
  /** @param {import('./console.js').ArmSDK} console_ */
  constructor(console_) { this.c = console_; }

  /** 开始录制（**只读，不碰舵机、不松力矩**）。 */
  async rec() { return this.c.sendCommand('teach rec'); }
  /** 停止录制。 */
  async exit() { return this.c.sendCommand('teach exit'); }
  /** 存到 Flash（阻塞 1~3 s）。 */
  async save(name) { return this.c.sendCommand('teach save' + (name ? ' ' + name : '')); }
  /** 回放。 */
  async play(name) { return this.c.sendCommand('teach play' + (name ? ' ' + name : '')); }
  async stop() { return this.c.sendCommand('teach stop'); }
  async list() { return this.c.sendCommand('files'); }
  async status() { return this.c.sendCommand('teach dump'); }
  /** ⚠ 会清掉当前录音。 */
  async clear() { return this.c.sendCommand('rec clear'); }
}

/* ==========================================================================
 * 文件读写（Node 环境下可用；浏览器里请用 toJSON/fromJSON 自己存）
 * ========================================================================== */

async function fsMod() {
  if (typeof process === 'undefined' || !process.versions?.node) return null;
  return import('node:fs/promises');
}

/** 按扩展名存轨迹（.json / .jsonl / .csv）。 */
export async function saveTrajectory(traj, path) {
  const fs = await fsMod();
  const fmt = Trajectory.detectFormat(path);
  if (fmt === 'npz') {
    throw new TeachError('.npz 只有 Python 版支持（需要 numpy）。' +
                         'JS 侧请存 .json / .jsonl / .csv。');
  }
  const text = fmt === 'json' ? JSON.stringify(traj.toJSON(), null, 1)
    : fmt === 'jsonl' ? traj.toJSONL()
      : traj.toCSV();
  if (!fs) throw new TeachError('当前环境没有文件系统，请用 toJSON()/toCSV() 自行保存');
  await fs.writeFile(path, text, 'utf8');
  return path;
}

/** 按扩展名读轨迹。 */
export async function loadTrajectory(path) {
  const fs = await fsMod();
  if (!fs) throw new TeachError('当前环境没有文件系统');
  const text = await fs.readFile(path, 'utf8');
  const fmt = Trajectory.detectFormat(path);
  if (fmt === 'json') return Trajectory.fromJSONText(text);
  if (fmt === 'jsonl') return Trajectory.fromJSONL(text);
  if (fmt === 'csv') return Trajectory.fromCSV(text);
  throw new TeachError('JS 侧不支持读 .npz，请先用 Python 版转成 .json/.csv');
}

export { sleep };
