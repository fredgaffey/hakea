// Minimal SVG charts (no dependencies): line/scatter plots and a phasor diagram.

const NS = 'http://www.w3.org/2000/svg';
const s = (tag, attrs = {}, ...kids) => {
  const n = document.createElementNS(NS, tag);
  for (const [k, v] of Object.entries(attrs)) if (v !== undefined && v !== null) n.setAttribute(k, v);
  for (const c of kids) if (c) n.append(c instanceof Node ? c : document.createTextNode(String(c)));
  return n;
};

export const PALETTE = ['#5aa9e6', '#e9a23b', '#4fb286', '#d36fb0', '#9c8cf0', '#e5534b', '#6fc7c7', '#b8b86a', '#c48a5a', '#8fa3b8'];

function ticks(lo, hi, n = 5) {
  if (!(hi > lo)) return [lo];
  const span = hi - lo;
  const step0 = span / n;
  const mag = 10 ** Math.floor(Math.log10(step0));
  const step = [1, 2, 2.5, 5, 10].map((m) => m * mag).find((x) => x >= step0) || step0;
  const out = [];
  for (let t = Math.ceil(lo / step) * step; t <= hi + 1e-9 * span; t += step) out.push(+t.toPrecision(10));
  return out;
}

const tf = (x) => {
  const a = Math.abs(x);
  if (a === 0) return '0';
  if (a >= 1e5 || a < 1e-2) return x.toExponential(1).replace('e+', 'e');
  return String(+x.toPrecision(3));
};

/**
 * lineChart({series:[{x:[], y:[], color, label, dash, points, width}], xlabel, ylabel,
 *            xlim:[lo,hi], ylim:[lo,hi], width, height, vlines:[{x,label}]})
 */
export function lineChart(opt) {
  const W = opt.width || 420, H = opt.height || 260;
  const m = { l: 48, r: 12, t: 10, b: 34 };
  const all = opt.series.flatMap((sr) => sr.x.map((x, i) => [x, sr.y[i]])).filter(([x, y]) => Number.isFinite(x) && Number.isFinite(y));
  const xs = all.map((p) => p[0]), ys = all.map((p) => p[1]);
  let [x0, x1] = opt.xlim || [Math.min(...xs), Math.max(...xs)];
  let [y0, y1] = opt.ylim || [Math.min(0, ...ys), Math.max(...ys)];
  if (!Number.isFinite(x0)) [x0, x1] = [0, 1];
  if (!Number.isFinite(y0)) [y0, y1] = [0, 1];
  if (x1 === x0) x1 = x0 + 1;
  if (y1 === y0) y1 = y0 + 1;
  if (!opt.ylim) y1 += (y1 - y0) * 0.05;
  const X = (x) => m.l + (x - x0) / (x1 - x0) * (W - m.l - m.r);
  const Y = (y) => H - m.b - (y - y0) / (y1 - y0) * (H - m.t - m.b);
  const svg = s('svg', { viewBox: `0 0 ${W} ${H}`, class: 'chart', role: 'img' });
  const clipId = 'c' + Math.random().toString(36).slice(2);
  svg.append(s('defs', {}, s('clipPath', { id: clipId }, s('rect', { x: m.l, y: m.t, width: W - m.l - m.r, height: H - m.t - m.b }))));
  const g = s('g', { class: 'axis' });
  for (const t of ticks(x0, x1)) {
    g.append(s('line', { x1: X(t), x2: X(t), y1: m.t, y2: H - m.b, class: 'gridl' }));
    g.append(s('text', { x: X(t), y: H - m.b + 13, 'text-anchor': 'middle' }, tf(t)));
  }
  for (const t of ticks(y0, y1)) {
    g.append(s('line', { x1: m.l, x2: W - m.r, y1: Y(t), y2: Y(t), class: 'gridl' }));
    g.append(s('text', { x: m.l - 5, y: Y(t) + 3, 'text-anchor': 'end' }, tf(t)));
  }
  g.append(s('rect', { x: m.l, y: m.t, width: W - m.l - m.r, height: H - m.t - m.b, fill: 'none', stroke: 'var(--line-2)' }));
  if (opt.xlabel) g.append(s('text', { x: (m.l + W - m.r) / 2, y: H - 4, 'text-anchor': 'middle', class: 'lbl' }, opt.xlabel));
  if (opt.ylabel) g.append(s('text', { x: 11, y: (m.t + H - m.b) / 2, 'text-anchor': 'middle', class: 'lbl', transform: `rotate(-90 11 ${(m.t + H - m.b) / 2})` }, opt.ylabel));
  svg.append(g);
  const plot = s('g', { 'clip-path': `url(#${clipId})` });
  for (const v of opt.vlines || []) {
    plot.append(s('line', { x1: X(v.x), x2: X(v.x), y1: m.t, y2: H - m.b, stroke: v.color || 'var(--accent)', 'stroke-dasharray': '2 3' }));
    if (v.label) svg.append(s('text', { x: X(v.x) + 3, y: m.t + 10 }, v.label));
  }
  for (const [i, sr] of opt.series.entries()) {
    const col = sr.color || PALETTE[i % PALETTE.length];
    const pts = sr.x.map((x, k) => [x, sr.y[k]]).filter(([x, y]) => Number.isFinite(x) && Number.isFinite(y));
    if (!sr.points || sr.line) {
      const d = pts.map(([x, y], k) => `${k ? 'L' : 'M'}${X(x).toFixed(1)},${Y(y).toFixed(1)}`).join('');
      plot.append(s('path', { d, fill: 'none', stroke: col, 'stroke-width': sr.width || 1.6, 'stroke-dasharray': sr.dash, opacity: sr.opacity }));
    }
    if (sr.points) for (const [x, y] of pts) plot.append(s('circle', { cx: X(x), cy: Y(y), r: sr.r || 2.2, fill: col, opacity: sr.opacity ?? 0.8 }));
    if (sr.endLabel && pts.length) {
      const [x, y] = pts[pts.length - 1];
      svg.append(s('text', { x: Math.min(X(x), W - m.r) - 2, y: Y(y) - 4, 'text-anchor': 'end', fill: col, style: `fill:${col}` }, sr.endLabel));
    }
  }
  svg.append(plot);
  return svg;
}

export function legend(series) {
  const d = document.createElement('div');
  d.className = 'legend';
  series.forEach((sr, i) => {
    if (!sr.label) return;
    const sp = document.createElement('span');
    const sw = document.createElement('i');
    sw.style.background = sr.color || PALETTE[i % PALETTE.length];
    if (sr.dash) sw.style.opacity = 0.6;
    sp.append(sw, sr.label);
    d.append(sp);
  });
  return d;
}

/** Phasor diagram: arrows [{amp, phase_deg, color, label}] scaled to the largest amplitude. */
export function phasorChart(arrows, { size = 200, title } = {}) {
  const R = size / 2 - 22, c = size / 2;
  const svg = s('svg', { viewBox: `0 0 ${size} ${size}`, class: 'chart', style: `max-width:${size}px` });
  const max = Math.max(1e-12, ...arrows.map((a) => a.amp));
  for (const f of [0.5, 1]) svg.append(s('circle', { cx: c, cy: c, r: R * f, fill: 'none', stroke: 'var(--grid)' }));
  svg.append(s('line', { x1: c - R, x2: c + R, y1: c, y2: c, stroke: 'var(--line-2)' }));
  svg.append(s('line', { y1: c - R, y2: c + R, x1: c, x2: c, stroke: 'var(--line-2)' }));
  svg.append(s('text', { x: c + R + 2, y: c + 3 }, '0°'));
  svg.append(s('text', { x: c - 6, y: c - R - 4 }, '90°'));
  if (title) svg.append(s('text', { x: 4, y: 11, class: 'lbl' }, title));
  svg.append(s('text', { x: size - 4, y: size - 4, 'text-anchor': 'end' }, `|max| ${tf(max)}°`));
  for (const a of arrows) {
    const r = (a.amp / max) * R, th = (a.phase_deg * Math.PI) / 180;
    const x = c + r * Math.cos(th), y = c - r * Math.sin(th);
    svg.append(s('line', { x1: c, y1: c, x2: x, y2: y, stroke: a.color, 'stroke-width': 2, 'stroke-dasharray': a.dash }));
    svg.append(s('circle', { cx: x, cy: y, r: 3, fill: a.color }));
    if (a.label) svg.append(s('text', { x: x + 4, y: y - 3, style: `fill:${a.color}` }, a.label));
  }
  return svg;
}
