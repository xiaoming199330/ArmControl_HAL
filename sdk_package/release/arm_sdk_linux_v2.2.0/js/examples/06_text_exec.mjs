/**
 * 06_text_exec.mjs — 用网口下发**任意控制台命令**（等价在 COM11 上敲）
 *
 *   node examples/06_text_exec.mjs            # 用内存假臂
 *   node examples/06_text_exec.mjs net://192.168.7.120:5000
 *
 * 背景：控制台有 124 条命令，网口原来只有 14 条。TEXT_EXEC 把**整个控制台**
 * 搬到了网口 —— 走的是固件里**同一条** ProcessCommand，语义零漂移。
 *
 * 但要理解三条边界（本示例逐条演示）：
 *   1. 11 条抢 ETH 引脚的命令会被固件**接住并说明原因**（不是静默失败）；
 *   2. `c`（校准向导）被安全闸门拒绝 —— 交互式，从网口进会卡死主循环；
 *   3. 输出 > 2048 B 会截断（`h` 帮助表 5406 B 就是这种情况），有 `[截断]` 标记。
 */
import { openArm } from '../src/index.js';
import { TEXT_BUF_SZ, TEXT_EXEC_MAX } from '../src/protocol.js';

const uri = process.argv[2] || 'sim://';
const arm = await openArm(uri, { timeout: 0.5 });
console.log(`已连 ${uri}\n`);

/* ---------- 1. 正常命令：与串口一字不差 ---------- */
console.log('=== 1. textExec("s") ===');
process.stdout.write(await arm.textExec('s'));

console.log('\n=== 2. textExec("zi") —— 前置检查：Home 零点是否污染 ===');
process.stdout.write(await arm.textExec('zi'));

/* ---------- 2. 抢引脚的命令：固件接住并解释 ---------- */
console.log('\n=== 3. textExec("cx") —— 会被接住，把原因当输出回给你 ===');
process.stdout.write(await arm.textExec('cx'));

console.log('\n  解析一下这条拒绝：gated =',
            (await arm.textOut('cx')).gated,
            '（false ⇒ 不是闸门拒绝，是 ProcessCommand 自己接住的）');

/* ---------- 3. 安全闸门：交互式命令不可用 ---------- */
console.log('\n=== 4. textExec("c") —— 安全闸门拒绝 ===');
const gated = await arm.textOut('c');
process.stdout.write(gated.text);
console.log('  gated =', gated.gated,
            '（true ⇒ 闸门拒绝；正文是**原因说明**，不是命令输出）');
console.log('  为什么：cmdCalibrate() 是 for(;;) 等人在本机串口敲键；从网口进');
console.log('          主循环永不返回，而本工程没有看门狗（红线 5）⇒ 板子彻底失联。');

/* ---------- 4. 急停闸门：只放 s / h ---------- */
console.log('\n=== 5. 急停中只放行 s / h ===');
if (arm.transport.setEstop) {
  arm.transport.setEstop(true);
  console.log('  s  gated =', (await arm.textOut('s')).gated, '（应 false）');
  console.log('  zi gated =', (await arm.textOut('zi')).gated, '（应 true）');
  process.stdout.write((await arm.textOut('zi')).text);
  arm.transport.setEstop(false);
} else {
  console.log('  （真机上把 E-Stop 按下去再试；这里跳过）');
}

/* ---------- 5. 分页：大输出怎么取回 ---------- */
console.log('\n=== 6. 分页机制 ===');
const first = await arm.textOut('cx');
console.log(`  0x8B 第一页：total=${first.total} B  off=${first.off}` +
            `  n=${first.raw.length}  done=${first.done}`);
console.log(`  每页正文上限 TEXT_CHUNK=58 B（6 B 分页头 + 58 = 64 B 封顶）`);
console.log(`  输出缓冲上限 TEXT_BUF_SZ=${TEXT_BUF_SZ} B，超了 truncated=true`);
console.log('  ⚠ offset/total 都是**字节**。控制台中文提示是 UTF-8 字节，');
console.log('    所以拼接必须按字节拼，最后统一 decode —— 否则分页缝上会出坏字符。');
const joined = await arm.textExec('cx');
console.log(`  textExec 自动翻页拼回 ${Buffer.byteLength(joined, 'utf8')} B，无坏字符：`,
            !joined.includes('\uFFFD'));
console.log('  含 PB12/PB13 引脚解释：', joined.includes('PB12/PB13'));

/* ---------- 6. 客户端护栏：超长命令直接抛 ---------- */
console.log('\n=== 7. 超长命令在客户端就被拦 ===');
try {
  await arm.textExec('x'.repeat(TEXT_EXEC_MAX + 1));
  console.log('  ✗ 居然发出去了（不该）');
} catch (e) {
  console.log(`  ✓ ${e.message}`);
  console.log(`    原因：固件把超长命令钳到 ${TEXT_EXEC_MAX} 字符后静默执行，`);
  console.log('          而**半条命令往往是另一条命令**（比如 "zr save" 被切成 "zr"）。');
}

/* ---------- 7. 安全提示 ---------- */
console.log('\n=== 8. 会改基准的命令请走类型化方法 ===');
console.log('  ❌ await arm.textExec("M a")              随手拼字符串');
console.log('  ✅ await arm.centerCalibDanger()          MAGIC 自动填，方法名带 Danger');
console.log('  ✅ await arm.zeroResetDanger(true)        = `zr save`');
console.log('  ✅ await arm.paramSave()                  = `F`');
console.log('  命名约定：会改硬件基准 / 会松力矩 / 会改 Flash 的，方法名带 Danger 后缀，');
console.log('            让调用点在 review 时一眼可见。');

await arm.close();
console.log('\n完成。');
