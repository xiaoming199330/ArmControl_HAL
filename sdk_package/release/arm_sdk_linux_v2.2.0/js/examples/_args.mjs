/** _args.mjs — 示例共用的极简取参（不引任何依赖）。 */

export function arg(name, def = null) {
  const i = process.argv.indexOf('--' + name);
  if (i >= 0) return process.argv[i + 1] ?? def;
  const pre = process.argv.find((s) => s.startsWith('--' + name + '='));
  if (pre) return pre.slice(name.length + 3);
  return def;
}

export function flag(name) {
  return process.argv.includes('--' + name);
}

/** 六轴格式化成一行。 */
export function fmtQ(q, nd = 1) {
  if (!q) return '[空]';
  return '[' + q.map((v) => (v === null || v === undefined ? '--' : Number(v).toFixed(nd))).join(', ') + ']';
}
