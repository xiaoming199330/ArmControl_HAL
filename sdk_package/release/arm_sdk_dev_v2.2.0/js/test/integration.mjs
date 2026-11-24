/**
 * integration.mjs — 离线端到端（用 sim 假臂，不需要硬件）
 *
 * 覆盖：14 条命令全语义 → 双臂遥操 → 录制 → 轨迹变换 → 回放
 */
import {
  openArm, TeleopSession, TeleopConfig, TeleopTeachSession, TeachRecorder,
  Trajectory, PlaybackSession, JointCalib, ErrCode, failedJoints,
  payloadStreamJoints, CMD,
} from '../src/index.js';

let n = 0;
const fails = [];
const chk = (name, cond, extra = '') => {
  n++;
  if (!cond) { fails.push(name + ' ' + extra); console.log('  FAIL ' + name + '  ' + extra); }
};
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

const arm = await openArm('sim://', { timeout: 0.5 });

/* ---------------- 1. 基础命令 ---------------- */
{
  const p = await arm.ping();
  chk('PING 有响应', p !== null);
  chk('proto_ver=1', p && p.proto_ver === 1, String(p && p.proto_ver));

  const st = await arm.getStatus();
  chk('GET_STATUS read_ok', st && st.read_ok === true);
  chk('初值 180', st && st.joints.every((v) => v === 180), JSON.stringify(st && st.joints));

  const pc = await arm.paramCount();
  chk('PARAM_COUNT=24', pc === 24, String(pc));

  const fk = await arm.fkQuery();
  chk('FK_QUERY read_ok===1（裸值）', fk && fk.read_ok === 1, String(fk && fk.read_ok));

  const r = await arm.logSetLevel(3);
  chk('LOG_SET_LEVEL ACK', r.ok);
}

/* ---------------- 2. 静默钳制必须回读 ---------------- */
{
  // ⚠ param id 3 = ik.fail_mm，合法范围 [0.5, 10.0]。写范围内的值才是"正常"。
  const w = await arm.paramWriteVerified(3, 2.5);
  chk('PARAM_WRITE 范围内值 → ok', w.ok, JSON.stringify({ want: w.want, got: w.got }));

  const bad = await arm.paramWriteVerified(3, 1e9);
  chk('超范围写 → 检测到被钳制（clamped）', bad.clamped === true,
      JSON.stringify({ want: bad.want, got: bad.got }));
  chk('钳到上限 10.0', Math.abs(bad.got - 10.0) < 1e-6, String(bad.got));
}

/* ---------------- 3. 点动必须用增量 ---------------- */
{
  const st0 = await arm.getStatus();
  const base = st0.joints[0];
  // 绝对角会被静默钳制 ⇒ 这里模拟"点动 +5°"
  await arm.moveJoints([base + 5, ...st0.joints.slice(1)], 60);
  await sleep(400);
  const st1 = await arm.getStatus();
  chk('MOVE_JOINTS 真的动了', Math.abs(st1.joints[0] - (base + 5)) < 3,
      `${base} → ${st1.joints[0]}`);
  chk('MOVE_JOINTS 自动上力矩', st1.joints.every((v) => v !== null));
}

/* ---------------- 4. STREAM_JOINTS 与 25° 自锁 ---------------- */
{
  const a2 = await openArm('sim://', { timeout: 0.5 });
  let r = await a2.streamJoints([0, 0, 0, 0, 0, 0]);
  chk('STREAM 正常帧 ACK', r && r.ok);

  r = await a2.streamJoints([0, 0, 0, 0, 0, 60]);
  chk('>25° 整帧拒 err=4(LIMIT)', r && r.err === ErrCode.LIMIT, r ? String(r.err) : 'null');

  r = await a2.streamJoints([0, 0, 0, 0, 0, 60]);
  chk('同一个大目标被持续拒（基准冻结 = 自锁）', r && r.err === ErrCode.LIMIT);

  r = await a2.streamJoints([0, 0, 0, 0, 0, 5]);
  chk('改小步就能过（说明不是"永久锁死"）', r && r.ok);

  await a2.close();
}

/* ---------------- 5. 读失败必须如实上报 ---------------- */
{
  const a3 = await openArm('sim://', { timeout: 0.5 });
  a3.transport.setJointFail([2]);
  const st = await a3.getStatus();
  chk('J3 读失败 → null', st.joints[2] === null, String(st.joints[2]));
  chk('read_ok=0', st.read_ok === false);
  chk('failedJoints=[3]', JSON.stringify(st.failedJoints) === '[3]',
      JSON.stringify(st.failedJoints));
  const lg = await a3.logicalAngles(new JointCalib());
  chk('logicalAngles 不兜底 → null', lg === null);
  await a3.close();
}

/* ---------------- 6. 双臂遥操（递增跟随 + 限速） ---------------- */
{
  const lead = await openArm('sim://', { timeout: 0.5 });
  const foll = await openArm('sim://', { timeout: 0.5 });

  // 两臂起始姿态故意不同 ⇒ 验证 hold 增量跟随能吸收这个差异
  await lead.moveJoints([180, 180, 180, 180, 180, 180], 200);
  await foll.moveJoints([150, 150, 150, 150, 150, 150], 200);
  await sleep(400);

  const tp = new TeleopSession(lead, foll, {
    cfg: new TeleopConfig({ hz: 100, slewDegPerSec: 240, sync: 'hold', waitAck: false }),
  });
  // ⚠ 递增跟随是「从臂从它**自己的**起始姿态开始，按位移量跟随」——
  //   所以判据不是"从臂到 +20"，而是"从臂相对自己起点走了 +20"。先记下起点。
  const follLog0 = await foll.logicalAngles(new JointCalib());
  chk('从臂起点可读', follLog0 !== null, JSON.stringify(follLog0));

  await tp.start();
  chk('遥操已启动', tp.isActive);
  chk('参考已建立（两臂起始姿态不同也能跟随）',
      tp._leadRef !== null && tp._follRef !== null);

  // 主臂 J2 走 +20° 舵机度（dir=+1 ⇒ 逻辑 +20）
  await lead.moveJoints([180, 200, 180, 180, 180, 180], 200);
  await sleep(300);

  for (let i = 0; i < 40; i++) { await tp.step(); await sleep(8); }
  chk('遥操发出帧', tp.stats.sent > 0, String(tp.stats.sent));
  chk('限速生效（已被削过）', tp.stats.slewClamped > 0, String(tp.stats.slewClamped));
  chk('未被固件拒帧', tp.stats.ackLimit === 0, String(tp.stats.ackLimit));

  const follLog = await foll.logicalAngles(new JointCalib());
  const moved = follLog && follLog0 ? follLog[1] - follLog0[1] : 0;
  chk('从臂在自己起点基础上走了 +20 左右（限速中，>5 即算跟随）',
      Math.abs(moved) > 5, `起点 ${follLog0 && follLog0[1]} → ${follLog && follLog[1]} (Δ=${moved.toFixed(1)})`);
  chk('跟随方向正确（递增跟随是加，不是减）', moved > 0, String(moved.toFixed(1)));

  const stats = await tp.stop();
  chk('停止后 stats 有 tEnd', stats.tEnd > 0);
  console.log('  遥操统计：' + stats.summary());
  await lead.close(); await foll.close();
}

/* ---------------- 7. 录制 + 轨迹 + 回放 ---------------- */
{
  const lead2 = await openArm('sim://', { timeout: 0.5 });
  const foll2 = await openArm('sim://', { timeout: 0.5 });

  const ts = new TeleopTeachSession(lead2, foll2, {
    name: 'demo', hz: 100, mode: 'follower',
    cfg: new TeleopConfig({ hz: 100, slewDegPerSec: 600, waitAck: false }),
  });
  await ts.start();
  chk('录制中', ts.isRecording === true);

  // 主臂画个慢速正弦（在 J1 上 ±15° 左右，避免大跳变）
  const t0 = Date.now() / 1000;
  while (Date.now() / 1000 - t0 < 0.8) {
    const el = Date.now() / 1000 - t0;
    const j1 = 180 + 12 * Math.sin(el * 2 * Math.PI);
    await lead2.moveJoints([j1, 180, 180, 180, 180, 180], 300);
    await ts._tp.step();
    await sleep(12);
  }
  await ts.stop();

  const traj = ts.rec.trajectory();
  chk('录到帧', traj.length > 5, String(traj.length));
  chk('units=logical_deg', traj.units === 'logical_deg', traj.units);
  chk('source 含 teleop', traj.source.startsWith('teleop'), traj.source);
  console.log('  ' + traj.summary());

  // 轨迹体检
  const v = traj.validate();
  chk('validate 无告警', v.ok === true, JSON.stringify(v.warnings));

  // 变换
  const rs = traj.resample(50);
  chk('resample 到 50 Hz', Math.abs(rs.rate - 50) < 2, String(rs.rate));
  const sm = traj.smooth(5);
  chk('smooth 不改变点数', sm.length === traj.length);
  chk('smooth 后最大步长不增',
      Math.max(...sm.maxStep()) <= Math.max(...traj.maxStep()) + 1e-6);
  const sc = traj.slice(0, traj.duration / 2);
  chk('slice 点数减少', sc.length < traj.length && sc.length > 0,
      `${sc.length} < ${traj.length}`);
  const st2 = traj.scaleTime(2);
  chk('scaleTime(2) 时长翻倍', Math.abs(st2.duration - traj.duration * 2) < 0.01);

  // 序列化往返
  // ⚠ toJSON 会把数值截到 6 位小数（保证文件稳定/可比），所以往返用容差比，
  //   不能用 JSON.stringify 严格相等。
  const j = Trajectory.fromJSONText(JSON.stringify(traj.toJSON()));
  chk('JSON 往返点数一致', j.length === traj.length);
  let maxDiff = 0;
  for (let i = 0; i < j.frames.length; i++) {
    if (Math.abs(j.frames[i][0] - traj.frames[i][0]) > maxDiff) {
      maxDiff = Math.abs(j.frames[i][0] - traj.frames[i][0]);
    }
    for (let k = 0; k < 6; k++) {
      maxDiff = Math.max(maxDiff, Math.abs(j.frames[i][1][k] - traj.frames[i][1][k]));
    }
  }
  chk('JSON 往返数值一致（6 位小数容差）', maxDiff < 1e-6, 'maxDiff=' + maxDiff);
  const c = Trajectory.fromCSV(traj.toCSV());
  chk('CSV 往返点数一致', c.length === traj.length, `${c.length} vs ${traj.length}`);
  chk('CSV 往返 units 保留', c.units === 'logical_deg', c.units);
  const jl = Trajectory.fromJSONL(traj.toJSONL());
  chk('JSONL 往返点数一致', jl.length === traj.length);
  chk('JSONL 往返 hz 保留', Math.abs(jl.hz - traj.hz) < 1e-6, String(jl.hz));

  // 回放
  const pb = new PlaybackSession(foll2, {
    cfg: new TeleopConfig({ hz: 60, readTimeout: 0.2 }),
    estopCheckS: 0.3,
  });
  const pst = await pb.play(traj, { speed: 2.0, loop: 1, approachSpeed: 60 });
  chk('回放发出帧', pst.sent > 0, String(pst.sent));
  chk('回放完成 1 轮', pst.loops === 1, String(pst.loops));
  chk('回放未中止', pst.aborted === false);
  console.log('  ' + pst.summary());

  await lead2.close(); await foll2.close();
}

/* ---------------- 8. 回放护栏：units 不对要拒绝 ---------------- */
{
  const bad = new Trajectory({ name: 'servo', units: 'servo_deg' });
  bad.append(0, [180, 180, 180, 180, 180, 180]);
  bad.append(0.02, [181, 180, 180, 180, 180, 180]);
  const pb = new PlaybackSession(arm, { cfg: new TeleopConfig({ hz: 50 }) });
  let threw = false;
  try { await pb.play(bad); } catch (e) { threw = /logical_deg/.test(e.message); }
  chk('回放拒绝 units=servo_deg（防止拿错量纲）', threw);
}

/* ---------------- 9. 回放护栏：大跳变被硬削 ---------------- */
{
  const a4 = await openArm('sim://', { timeout: 0.5 });
  const jump = new Trajectory({ name: 'jump', units: 'logical_deg', hz: 50 });
  jump.append(0, [0, 0, 0, 0, 0, 0]);
  jump.append(0.02, [100, 0, 0, 0, 0, 0]);      // 100° 跳变，只有 2 个点

  // ⚠ speed=1 才会触发硬削：回放的 hz = max(cfg.hz, traj.rate) * speed，
  //   重采样点数 = floor(duration*hz)+1。speed 大 ⇒ 点密 ⇒ 每步本来就 <20°，用不着削。
  const pb = new PlaybackSession(a4, { cfg: new TeleopConfig({ hz: 50 }), estopCheckS: 0 });
  await pb.play(jump, { speed: 1, approach: false, warmupFrames: 0 });
  chk('回放单帧硬削在 25° 门限以下', pb._last !== null && Math.abs(pb._last[0]) <= 20.001,
      pb._last ? String(pb._last[0]) : 'null');
  chk('硬削计入 slewClamped', pb.stats.slewClamped > 0, String(pb.stats.slewClamped));

  // 反过来：speed 大 ⇒ 重采样把 100° 摊成小步，压根不需要削
  const pb2 = new PlaybackSession(a4, { cfg: new TeleopConfig({ hz: 50 }), estopCheckS: 0 });
  await pb2.play(jump, { speed: 50, approach: false, warmupFrames: 0 });
  chk('高速回放靠重采样摊平，steps 全部 ≤20°（无需硬削）',
      pb2.stats.slewClamped === 0, String(pb2.stats.slewClamped));
  chk('高速回放末点到位 100°', Math.abs(pb2._last[0] - 100) < 0.5,
      String(pb2._last[0]));
  await a4.close();
}

/* ---------------- 10. 录制器独立使用 ---------------- */
{
  // ⚠ 别用 rec.asCallback() 在同步循环里喂点：那是"瞬时"喂 20 点，
  //   中间没有真实时间流逝，而录制器用 Date.now()（1 ms 分辨率）打时间戳 ⇒
  //   时长为 0 或 1 ms ⇒ rate 抖到 0 或 19000，断言随机红/绿。
  //   这里直接给显式时间戳（1/50 s 间隔），让推断结果确定。
  const rec = new TeachRecorder({ name: 'manual', hz: 50, mode: 'follower' });
  rec.start();
  const cb = rec.asCallback();
  for (let i = 0; i < 20; i++) {
    cb([i, 0, 0, 0, 0, 0], [i * 0.5, 0, 0, 0, 0, 0]);
    rec.traj.frames[rec.traj.frames.length - 1][0] = i / 50;   // 回填确定的时间轴
  }
  rec.stop();
  chk('录制器收到 20 点', rec.count === 20, String(rec.count));
  const t = rec.trajectory();
  chk('录制器轨迹点数一致', t.length === 20);
  // 19 个间隔 / 0.38 s = 50.0 Hz
  chk('录制器按时间戳推出 rate≈50', Math.abs(t.hz - 50) < 0.5, String(t.hz));
}

/* ---------------- 10b. 采样率不可测时必须回 0，不能吐假数字 ---------------- */
{
  // 里程碑式回归：曾经 20 点挤在 1 ms 内 ⇒ rate = 19000 Hz 被写进轨迹文件。
  const t = new Trajectory({ name: 'instant', hz: 50, units: 'logical_deg' });
  for (let i = 0; i < 20; i++) t.append(i * 0.00005, [180, 180, 180, 180, 180, 180]); // 全挤在 0.95 ms
  chk('采样率超时钟分辨率 -> rate=0（未知）', t.rate === 0, String(t.rate));
  const d = new TeachRecorder({ name: 'manual2', hz: 50, mode: 'follower' });
  d.start();
  for (let i = 0; i < 20; i++) d.record([180, 180, 180, 180, 180, 180], i * 0.00005);
  d.stop();
  chk('rate 未知时轨迹回落到名义 hz=50', d.trajectory().hz === 50, String(d.trajectory().hz));

  // 边界：刚好 1 ms 一点（1000 Hz）仍可信；1 kHz 以上判未知
  const ok = new Trajectory({ name: 'k1', hz: 1000, units: 'logical_deg' });
  for (let i = 0; i < 11; i++) ok.append(i * 0.001, [180, 180, 180, 180, 180, 180]);
  chk('1000 Hz 仍可信', Math.abs(ok.rate - 1000) < 1, String(ok.rate));
}

/* ---------------- 11b. 2026-09-23 扩展：全指令覆盖 ---------------- */
{
  const inf = await arm.getInfo();
  chk('GET_INFO fw_tag 非空', typeof inf.fw_tag === 'string' && inf.fw_tag.length > 0,
      inf.fw_tag);
  chk('GET_INFO build_name', inf.build_name === 'net', inf.build_name);
  chk('GET_INFO n_params=24', inf.n_params === 24, String(inf.n_params));

  const fl = await arm.getFlags();
  chk('GET_FLAGS 初态无急停', fl.estop === false, String(fl.raw));

  const lm = await arm.getLimits();
  chk('GET_LIMITS 六轴各 4 组 raw',
      lm.min_raw.length === 6 && lm.zero_raw.length === 6);
  chk('GET_LIMITS zero_raw=0 ⇒ calibrated=false（量纲基准脆）',
      lm.calibrated.every((c) => c === false));

  const jd = await arm.getJointDetail(0xFF);
  chk('GET_JOINT_DETAIL 六轴', jd.length === 6 && jd.every((d) => d.ok), String(jd.length));
  chk('GET_JOINT_DETAIL 温度/电压有值', jd[0].temp === 42 && jd[0].voltage === 12);
  const jd1 = await arm.getJointDetail(1);
  chk('GET_JOINT_DETAIL 单轴 n=1', jd1.length === 1 && jd1[0].joint === 1);

  const pid = await arm.getPid(0);
  chk('GET_PID from_eeprom=true', pid.from_eeprom === true, JSON.stringify(pid));

  chk('GET_GRAVITY 返回数值', typeof (await arm.getGravity()) === 'number');
  const corr = await arm.getCorrection();
  chk('GET_CORRECTION 三参数', typeof corr.threshold_deg === 'number' &&
      typeof corr.max_rounds === 'number', JSON.stringify(corr));

  const metas = await arm.paramList();
  chk('PARAM_LIST 翻页取全 24 条', metas.length === 24, String(metas.length));
  chk('PARAM_LIST 顺序 = 固件表顺序（首条 id=1）', metas[0].id === 1, String(metas[0].id));

  // ---- 示教存储 ----
  const tot = await arm.teachTotal();
  chk('TEACH_TOTAL 文件数/容量', tot.n_files === 2 && tot.capacity === 32,
      JSON.stringify(tot));
  const f0 = await arm.teachList(0);
  chk('TEACH_LIST #0 = demo', f0.name === 'demo' && f0.valid === true, JSON.stringify(f0));
  const all = await arm.teachListAll();
  chk('TEACH_LIST_ALL 列全', all.length === 2, String(all.length));
  const trace = await arm.teachTrace(0);
  chk('TEACH_TRACE 翻页取全 12 点', trace.length === 12, String(trace.length));
  chk('TEACH_TRACE 每点 6 轴', trace.every((p) => p.length === 6));
  const empty = await arm.teachRead(0, 9999, 5);
  chk('TEACH_READ off 越界 ⇒ n=0（正常读完，不抛）', empty.length === 0);
  chk('TEACH_DELETE ACK', (await arm.teachDelete(1)).ok);
  chk('TEACH_COMPACT ACK', (await arm.teachCompact()).ok);

  // ---- IK 试算（只解不算） ----
  const ik = await arm.ikQuery(200, 0, 150, 0);
  chk('IK_QUERY 可解 ⇒ 返回六关节舵机度',
      ik.length === 6 && ik.every((v) => v >= 0 && v <= 360), JSON.stringify(ik));
  let raised = false;
  try { await arm.ikQuery(2000, 2000, 2000, 0); } catch (e) {
    raised = e.name === 'ArmCommandError';
  }
  chk('IK_QUERY 不可达 ⇒ 抛 ArmCommandError（读取类不返回 null）', raised);

  // ---- 单轴运动 / 总线模式 ----
  chk('MOVE_JOINT_ONE ACK', (await arm.moveJointOne(2, 100, 30)).ok);
  chk('SET_BUS_MODE ACK', (await arm.setBusMode(1)).ok);
  let threw = false;
  try { await arm.setBusMode(3); } catch { threw = true; }
  chk('SET_BUS_MODE 3 在客户端就被拦（不发帧）', threw);

  // ---- 危险操作：类型化方法 + MAGIC 由本层自动填 ----
  chk('PARAM_SAVE ACK', (await arm.paramSave()).ok);
  chk('CENTER_CALIB ACK', (await arm.centerCalibDanger()).ok);
  chk('ZERO_RESET(save) ACK', (await arm.zeroResetDanger(true)).ok);
  chk('SYS_RESET ACK', (await arm.sysResetDanger()).ok);
  const ce = await arm.calibEnterDanger();
  chk('CALIB_ENTER 一律 NACK(ERR_PARAM)：交互式命令设计上不可用',
      ce.ok === false && ce.err === ErrCode.PARAM, JSON.stringify(ce.err));
  chk('CALIB_ENTER 不抛异常（返回 Reply，让"不可用"在 API 上可见）', true);

  // ---- 文本透传：整个控制台 ----
  const s = await arm.textExec('s');
  chk('textExec("s") 含 J1 表头', s.includes('J1'), s.replace(/\r\n/g, '|'));
  chk('textExec("s") === COM11 同一份输出（含 CRLF）', s.includes('\r\n'));
  const cx = await arm.textExec('cx');
  chk('textExec("cx") 被固件接住并说明原因（不是静默失败）',
      cx.includes('已停用') && cx.includes('PB12/PB13'));
  const gate = await arm.textExec('c');
  chk('textExec("c") 被安全闸门拒（交互式会卡死主循环）', gate.includes('GATE'));
  const fOut = await arm.textExec('F');
  chk('textExec("F") 等价 paramSave', fOut.includes('saved'));
  const tout = await arm.textOut('s');
  chk('textOut 保留分页元信息 total/off', tout.total > 0 && tout.off === 0,
      String(tout.total));
  chk('textOut 未截断', tout.truncated === false && tout.gated === false);
  let tooLong = false;
  try { await arm.textExec('x'.repeat(200)); } catch { tooLong = true; }
  chk('textExec 超长在客户端就抛错（不让固件静默执行半条命令）', tooLong);

  // ---- 急停闸门：只放 s / h ----
  arm.transport.setEstop(true);
  chk('急停时 textExec("s") 仍放行', !(await arm.textOut('s')).gated);
  chk('急停时 textExec("zi") 被闸门拒', (await arm.textOut('zi')).gated === true);
  arm.transport.setEstop(false);
}

/* ---------------- 11. dry-run 不发包 ---------------- */
{
  const d = await openArm('sim://', { dryRun: true, timeout: 0.3 });
  const r = await d.moveJoints([180, 180, 180, 180, 180, 180]);
  chk('dry-run 回 ok 且标记 dryRun', r.ok === true && r.dryRun === true);
  chk('dry-run 确实没发包', d.stats.sent === 0, String(d.stats.sent));
  await d.close();
}

await arm.close();

/* ---------------- 汇总 ---------------- */
console.log('\n' + '='.repeat(60));
if (fails.length) {
  console.log(`集成测试：失败 ${fails.length} / ${n} 项`);
  fails.forEach((f) => console.log('  · ' + f));
  process.exit(1);
}
console.log(`集成测试：${n} 项，全部通过`);
console.log('离线端到端通过（无硬件）。');
