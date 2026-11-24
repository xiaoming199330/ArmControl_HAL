/**
 * teleop.js — 主从臂遥操（网口，PC 中转 / 直连都支持）
 *
 * 原理：PC 读主臂 → 算出从臂目标 → 用 STREAM_JOINTS 推给从臂。
 *       **两臂之间一根线都不用接，PC 就是那根线。**
 *
 * 三个必须避开的固件坑（本模块已内置对策）：
 *
 *   1. 单帧单轴跳变 >25° ⇒ 固件**整帧拒收，且不更新基准** ⇒ 永久冻结（自锁）。
 *      对策：发送端**限速** slew_deg_per_s（50 Hz 下 240°/s = 4.8°/帧，余量很足）。
 *
 *   2. 断流 >500 ms ⇒ 从臂退出流模式并原地自锁 ⇒ 频率必须 >2 Hz（推荐 50 Hz）。
 *      对策：主臂偶尔读失败时**重发上一有效目标**（hold），不让流断掉。
 *
 *   3. 两套角度量纲。STREAM_JOINTS 要**逻辑角**，而 GET_STATUS 回**舵机度**。
 *      对策：内部一律用逻辑角，靠 JointCalib 换算，外部不用管。
 *
 * 与 Python 版 `arm_sdk/teleop.py` 同语义。
 */

import { CMD, RSP, ErrCode, NUM_JOINTS, JointCalib, wrap180 } from './protocol.js';
import { ArmCommandError } from './client.js';

/** 固件侧硬编码：断流超时。 */
export const FW_STREAM_TIMEOUT_MS = 500;
/** 单帧单轴跳变拒帧门限。 */
export const FW_MAX_JUMP_DEG = 25;
/** 低于这个频率必掉流（500ms → 2Hz，留余量）。 */
export const FW_MIN_HZ = 3;

export class TeleopError extends Error {
  constructor(msg) { super(msg); this.name = 'TeleopError'; }
}
export class TeleopLinkLost extends TeleopError {
  constructor(msg) { super(msg); this.name = 'TeleopLinkLost'; }
}
export class TeleopAbort extends TeleopError {
  constructor(msg) { super(msg); this.name = 'TeleopAbort'; }
}

export class TeleopConfig {
  constructor(fields = {}) {
    this.hz = fields.hz ?? 50;
    this.slewDegPerSec = fields.slewDegPerSec ?? 240;
    this.maxStepDeg = fields.maxStepDeg ?? 0;
    this.gain = fields.gain ?? 1.0;
    /** false=全轴同向；true=全轴反向；也可以是 6 个 ±1（镜像安装的从臂常要 J1/J4/J5/J6 反）。 */
    this.reverse = fields.reverse ?? false;
    /** 'hold'=增量跟随（两臂起始姿态可不同）；'align'=先把从臂绝对对到主臂姿态。 */
    this.sync = fields.sync ?? 'hold';
    this.syncSpeed = fields.syncSpeed ?? 20;
    this.readTimeout = fields.readTimeout ?? 0.2;
    this.holdOnLeaderLoss = fields.holdOnLeaderLoss ?? true;
    this.linkLossTimeoutS = fields.linkLossTimeoutS ?? 1.0;
    this.maxLeaderFail = fields.maxLeaderFail ?? 50;
    this.estopTrips = fields.estopTrips ?? true;
    this.enableTorque = fields.enableTorque ?? true;
    /** 每轴 [lo, hi] 逻辑角软限位，如 [[-20,20],...]。null=不限。 */
    this.softLimitsDeg = fields.softLimitsDeg ?? null;
    this.waitAck = fields.waitAck ?? false;
    this.verbose = fields.verbose ?? false;
  }
}

export class TeleopStats {
  constructor() {
    this.frames = 0;
    this.sent = 0;
    this.held = 0;
    this.leaderFail = 0;
    this.ackOk = 0;
    this.ackLimit = 0;
    this.ackEstop = 0;
    this.softClamped = 0;
    this.slewClamped = 0;
    this.tStart = 0;
    this.tEnd = 0;
  }
  get elapsed() { return this.tEnd ? this.tEnd - this.tStart : 0; }
  get fps() { return this.elapsed > 0 ? this.sent / this.elapsed : 0; }
  summary() {
    return `发送 ${this.sent} 帧 / ${this.elapsed.toFixed(1)}s ` +
           `(${this.fps.toFixed(1)} Hz) | hold ${this.held} 读失败 ${this.leaderFail} | ` +
           `ACK ok=${this.ackOk} limit=${this.ackLimit} estop=${this.ackEstop} | ` +
           `软限位 ${this.softClamped} 限速 ${this.slewClamped}`;
  }
}

/* ==========================================================================
 * TeleopSession
 * ========================================================================== */

export class TeleopSession {
  /**
   * @param {import('./client.js').ArmClient} leader
   * @param {import('./client.js').ArmClient} follower
   * @param {{cfg?:TeleopConfig, leaderCalib?:JointCalib, followerCalib?:JointCalib,
   *          onEvent?:(s:string)=>void, onFrame?:(lead:number[],target:number[])=>void}} [opts]
   */
  constructor(leader, follower, opts = {}) {
    this.leader = leader;
    this.follower = follower;
    this.cfg = opts.cfg || new TeleopConfig();
    this.leadCalib = opts.leaderCalib || new JointCalib();
    this.follCalib = opts.followerCalib || new JointCalib();
    this.onEventCb = opts.onEvent || null;
    this.onFrame = opts.onFrame || null;

    this.stats = new TeleopStats();
    this._signs = TeleopSession.resolveReverse(this.cfg.reverse);
    this._active = false;
    this.aborted = false;
    this._leadRef = null;
    this._follRef = null;
    this._lastSent = null;
    this._lastGoodT = 0;
    this._lastT = 0;
    this._linkLost = false;
    this.stopRequested = false;
  }

  /** 把 reverse 配置归一化成 6 个 ±1。 */
  static resolveReverse(rev) {
    if (rev === undefined || rev === null || rev === false) return new Array(NUM_JOINTS).fill(1);
    if (rev === true) return new Array(NUM_JOINTS).fill(-1);
    if (Array.isArray(rev) || (rev && typeof rev.length === 'number')) {
      if (rev.length !== NUM_JOINTS) throw new TeleopError('reverse 序列必须 6 个');
      return Array.from(rev, (v) => (v ? -1 : 1));
    }
    throw new TeleopError('reverse 只能是 bool 或 6 元素序列');
  }

  _event(msg) {
    if (this.cfg.verbose) console.log('[teleop] ' + msg);
    if (this.onEventCb) { try { this.onEventCb(msg); } catch { /* 忽略 */ } }
  }

  _syncMode() {
    const s = String(this.cfg.sync || 'hold').toLowerCase();
    if (['relative', 'none', 'off', 'hold'].includes(s)) return 'hold';
    if (['absolute', 'align', 'match'].includes(s)) return 'align';
    throw new TeleopError("sync 只能是 hold / align");
  }

  /* ---- 映射 / 软限位 / 限速 ---- */

  /**
   * 主臂逻辑角 → 从臂目标逻辑角：
   *   target[j] = foll_ref[j] + gain * sign[j] * (lead[j] - lead_ref[j])
   * 递增式，两臂 Home 零点差异被 foll_ref 吸收 ⇒ 不要求起始姿态一致。
   */
  _map(leadLogical) {
    const g = Number(this.cfg.gain);
    const haveRef = this._leadRef !== null && this._follRef !== null;
    const out = [];
    for (let j = 0; j < NUM_JOINTS; j++) {
      out.push(haveRef
        ? this._follRef[j] + g * this._signs[j] * (leadLogical[j] - this._leadRef[j])
        : g * this._signs[j] * leadLogical[j]);
    }
    return out;
  }

  _clampSoft(target) {
    const lim = this.cfg.softLimitsDeg;
    if (!lim) return target;
    const out = [...target];
    for (let j = 0; j < NUM_JOINTS && j < lim.length; j++) {
      const [lo, hi] = lim[j];
      if (out[j] < lo) { out[j] = lo; this.stats.softClamped++; }
      else if (out[j] > hi) { out[j] = hi; this.stats.softClamped++; }
    }
    return out;
  }

  _slew(target, dt) {
    if (this._lastSent === null) return [...target];
    if (dt <= 0) dt = 1 / Math.max(1, this.cfg.hz);
    let cap = Math.max(0, this.cfg.slewDegPerSec) * dt;
    if (this.cfg.maxStepDeg > 0) cap = Math.min(cap, this.cfg.maxStepDeg);
    const out = [];
    for (let j = 0; j < NUM_JOINTS; j++) {
      let d = target[j] - this._lastSent[j];
      if (cap > 0 && Math.abs(d) > cap) {
        d = Math.sign(d) * cap;
        this.stats.slewClamped++;
      }
      out.push(this._lastSent[j] + d);
    }
    return out;
  }

  /* ---- 生命周期 ---- */

  /** 读两臂姿态 + 建参考 + （可选）对齐从臂。失败抛 TeleopError。 */
  async start() {
    if (this.cfg.hz < FW_MIN_HZ) {
      throw new TeleopError(`hz=${this.cfg.hz} 太低（固件断流 500ms，必须 >2Hz）`);
    }
    const mode = this._syncMode();

    this._event('读取主臂姿态…');
    let ls;
    try { ls = await this.leader.getStatus({ timeout: this.cfg.readTimeout }); }
    catch (e) { throw new TeleopError('主臂不可达：' + e.message); }
    if (!ls) throw new TeleopError('主臂无响应');
    const lsv = ls.read_ok && ls.joints.every((v) => v !== null) ? ls.joints : null;
    if (lsv === null) {
      throw new TeleopError(`主臂读轴失败 J${JSON.stringify(ls.failedJoints)} —— ` +
                            '先排总线，别带着瞎读数开遥操');
    }

    let fs;
    try { fs = await this.follower.getStatus({ timeout: this.cfg.readTimeout }); }
    catch (e) { throw new TeleopError('从臂不可达：' + e.message); }
    let fsv = fs && fs.read_ok && fs.joints.every((v) => v !== null) ? fs.joints : null;

    const leadLog = this.leadCalib.toLogical(lsv);

    if (mode === 'align') {
      if (fsv === null) {
        throw new TeleopError(`从臂读轴失败 J${JSON.stringify(fs ? fs.failedJoints : [])} —— 无法对齐`);
      }
      const targetServo = this.follCalib.toServo(this._absMapped(leadLog));
      this._event(`对齐从臂到主臂姿态（MOVE_JOINTS v=${this.cfg.syncSpeed.toFixed(0)}）…`);
      const r = await this.follower.moveJoints(targetServo, this.cfg.syncSpeed);
      if (!r.ok) throw new TeleopError('对齐失败：' + r.toString());
      fsv = await this._waitSettle(targetServo);
    }

    let follLog;
    if (fsv === null) {
      this._event('⚠ 从臂读轴失败，参考取 0（从臂真实姿态未知，先小范围试动）');
      follLog = new Array(NUM_JOINTS).fill(0);
    } else {
      follLog = this.follCalib.toLogical(fsv);
    }

    this._leadRef = [...leadLog];
    this._follRef = [...follLog];
    this._lastSent = [...follLog];
    this._lastGoodT = Date.now() / 1000;
    this._lastT = 0;

    if (this.cfg.enableTorque) await this.follower.setTorque(0xFF, true);

    this._active = true;
    this.aborted = false;
    this.stopRequested = false;
    this._linkLost = false;
    this.stats = new TeleopStats();
    this.stats.tStart = Date.now() / 1000;
    this._event(`遥操开始：${this.cfg.hz.toFixed(0)} Hz，限速 ` +
                `${this.cfg.slewDegPerSec.toFixed(0)}°/s，sync=${mode}`);
    return this;
  }

  _absMapped(leadLog) {
    const g = Number(this.cfg.gain);
    return leadLog.map((v, j) => v * g * this._signs[j]);
  }

  /** 等 MOVE_JOINTS 走完（连续两次读数都收敛），最多 5 s。 */
  async _waitSettle(targetServo) {
    const t0 = Date.now() / 1000;
    let last = null;
    while (Date.now() / 1000 - t0 < 5.0) {
      await sleep(50);
      let st;
      try { st = await this.follower.getStatus({ timeout: this.cfg.readTimeout }); }
      catch { continue; }
      if (!st) continue;
      const sv = st.read_ok && st.joints.every((v) => v !== null) ? st.joints : null;
      if (sv === null) continue;
      const err = Math.max(...sv.map((v, j) => Math.abs(wrap180(v - targetServo[j]))));
      if (last !== null && err < 1.5) return sv;
      last = sv;
    }
    this._event('⚠ 对齐未在 5s 内收敛，按当前读数继续');
    return last;
  }

  /** 请求停止（供 UI 线程 / Ctrl-C 调用）。 */
  requestStop() { this.stopRequested = true; }

  /** 停止遥操。stopFollower=true 时给从臂发 STOP。 */
  async stop(stopFollower = true) {
    if (!this._active) return this.stats;
    this._active = false;
    if (stopFollower) {
      try { await this.follower.stop(); } catch { /* 忽略 */ }
    }
    this.stats.tEnd = Date.now() / 1000;
    this._event('遥操结束：' + this.stats.summary());
    return this.stats;
  }

  get isActive() { return this._active; }

  /* ---- 单帧 ---- */

  /**
   * 走一帧。返回实际发给从臂的 6 个逻辑角；hold/中止时返回 null。
   * @returns {Promise<number[]|null>}
   */
  async step() {
    if (!this._active) return null;
    const now = Date.now() / 1000;
    const dt = this._lastT ? now - this._lastT : 1 / Math.max(1, this.cfg.hz);
    this._lastT = now;
    this.stats.frames++;

    /* 1) 读主臂 */
    let st;
    try {
      st = await this.leader.getStatus({ timeout: this.cfg.readTimeout });
    } catch (e) {
      return this._onLeaderLost(dt, '读超时: ' + e.message);
    }
    if (!st) return this._onLeaderLost(dt, '无响应');
    if (st.estop) { this._onEstop('主臂报 E-Stop'); return null; }
    const lsv = st.read_ok && st.joints.every((v) => v !== null) ? st.joints : null;
    if (lsv === null) {
      return this._onLeaderLost(dt, `读轴失败 J${JSON.stringify(st.failedJoints)}`);
    }

    /* 2) 映射 + 软限位 + 限速 */
    const leadLog = this.leadCalib.toLogical(lsv);
    let target = this._clampSoft(this._map(leadLog));
    target = this._slew(target, dt);

    /* 3) 下发 */
    await this._send(target);
    this._lastSent = target;
    this._lastGoodT = now;
    this.stats.leaderFail = 0;
    this._linkLost = false;

    if (this.onFrame) { try { this.onFrame(leadLog, target); } catch { /* 忽略 */ } }
    return target;
  }

  async _send(target) {
    if (this.cfg.waitAck) {
      const r = await this.follower.streamJoints(target, true,
                                                 { timeout: this.cfg.readTimeout });
      if (r === null) return;
      this._accountAck(r.ok, r.err);
    } else {
      await this.follower.streamJoints(target, false);
      this.stats.sent++;
      for (const a of this.follower.pollStreamAcks()) this._accountAck(a.ok, a.err);
    }
  }

  _accountAck(ok, err) {
    if (ok || err === ErrCode.OK) { this.stats.ackOk++; return; }
    if (err === ErrCode.LIMIT) {
      this.stats.ackLimit++;
      if ([1, 10, 100].includes(this.stats.ackLimit)) {
        this._event(`⚠ 从臂拒帧 err=4（跳变>25° 或超限）第 ${this.stats.ackLimit} 次`);
      }
    } else if (err === ErrCode.ESTOP) {
      this.stats.ackEstop++;
      this._event('⚠ 从臂报 E-Stop');
    } else if (err === ErrCode.UNREACH) {
      // 从臂没进入流模式（常见：`sf 0` 关过跟随，或 500ms 掉流后尚未恢复）
      this._event('⚠ 从臂未接受流帧 err=3（检查 `sf 1` / 断流）');
    }
  }

  /* ---- 异常处理 ---- */

  _onLeaderLost(dt, reason) {
    this.stats.leaderFail++;
    this._event(`主臂 ${reason}（第 ${this.stats.leaderFail} 次）`);

    // 丢控超过阈值 ⇒ 真丢链路，停臂
    const gap = Date.now() / 1000 - this._lastGoodT;
    if (gap > this.cfg.linkLossTimeoutS) {
      this._active = false;
      this._linkLost = true;
      this._event(`⚠ 主臂丢控 ${gap.toFixed(2)}s > ${this.cfg.linkLossTimeoutS}s，停臂`);
      this.follower.stop().catch(() => {});
      throw new TeleopLinkLost(`主臂丢控 ${gap.toFixed(2)}s`);
    }
    if (this.stats.leaderFail > this.cfg.maxLeaderFail) {
      this._active = false;
      this.follower.stop().catch(() => {});
      throw new TeleopAbort(`主臂连续读失败 ${this.stats.leaderFail} 次`);
    }

    // hold：重发上一有效目标，**不让流断掉**（断 500ms 就从臂就自锁）
    if (this.cfg.holdOnLeaderLoss && this._lastSent !== null) {
      this._send(this._lastSent).catch(() => {});
      this.stats.held++;
    }
    return null;
  }

  _onEstop(reason) {
    this._event('🛑 ' + reason);
    if (this.cfg.estopTrips) {
      this._active = false;
      this.aborted = true;
      this.follower.stop().catch(() => {});
      throw new TeleopAbort(reason);
    }
  }

  /* ---- 主循环 ---- */

  /**
   * 按 hz 跑到时长 / until() 为真 / stopRequested。
   *
   * 用**补偿式休眠**（不是固定 sleep(1/hz)）—— 后者在 JS 里实测只有 ~35 Hz。
   * @param {{durationS?:number, until?:()=>boolean, onTick?:(s:TeleopStats)=>void}} [opts]
   */
  async run(opts = {}) {
    if (!this._active) await this.start();
    const period = 1 / Math.max(1, this.cfg.hz);
    const t0 = Date.now() / 1000;
    let next = Date.now() / 1000;

    try {
      for (;;) {
        if (this.stopRequested) break;
        if (opts.until && opts.until()) break;
        if (opts.durationS !== undefined &&
            Date.now() / 1000 - t0 >= opts.durationS) break;

        await this.step();
        if (opts.onTick) opts.onTick(this.stats);

        next += period;
        const slp = (next - Date.now() / 1000) * 1000;
        if (slp > 0) await sleep(slp);
        else next = Date.now() / 1000;    // 落后了放弃追帧，避免雪崩
      }
    } finally {
      await this.stop();
    }
    return this.stats;
  }
}

export const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

/* ==========================================================================
 * 固件直连路线（主从两板之间直接连 UART4，PC 不参与转发）
 * ========================================================================== */

/**
 * 固件侧"主从直连"模式的控制台命令封装。
 *
 * ⚠ 本路线需要两板之间**物理连着 UART4（PA0/PA1）**，且历史上踩过 PA0/PA1 虚焊
 *   （见 docs/主从臂联调步骤_2026-09-11.md §13）。PC 中转路线（TeleopSession）
 *   不需要这根线，是当前推荐的路线。
 */
export class FirmwareLink {
  /** @param {import('./console.js').ArmSDK} console 串口文本控制台 */
  constructor(console_) { this.c = console_; }

  /** 主臂：读两臂相对 Home 的逻辑角。 */
  async ms() { return this.c.sendCommand('ms'); }
  /** 主臂：暂停/恢复转发。 */
  async mf(on) { return this.c.sendCommand('mf ' + (on ? 1 : 0)); }
  /** 主臂：0=只读模式 1=主动模式。 */
  async ma(on) { return this.c.sendCommand('ma ' + (on ? 1 : 0)); }
  /** 主臂：设上行频率 Hz。 */
  async sp(hz) { return this.c.sendCommand('sp ' + hz); }
  /** 从臂：暂停/恢复跟随。**测遥操前必须先 `sf 0`**，否则对端发流会盖掉轨迹。 */
  async sf(on) { return this.c.sendCommand('sf ' + (on ? 1 : 0)); }
  /** 从臂：链路诊断。 */
  async lk() { return this.c.sendCommand('lk'); }
  /** 从臂：链路状态。 */
  async status() { return this.c.sendCommand('zi'); }
}

/* ==========================================================================
 * PC 中转路线（三根线：主臂-USB、从臂-USB，PC 做转发）
 * ========================================================================== */

/** 旧式 PC 中转：用文本 `#P` 行转发（不推荐，仅兼容老固件）。 */
export class PcRelay {
  constructor(leaderConsole, followerConsole, opts = {}) {
    this.lead = leaderConsole;
    this.foll = followerConsole;
    this.hz = opts.hz ?? 30;
  }

  static encodePositionLine(logicalDeg) {
    return '#P ' + logicalDeg.map((v) => v.toFixed(2)).join(' ');
  }
}
