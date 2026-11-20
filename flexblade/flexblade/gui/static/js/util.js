// Small DOM / data helpers shared by the GUI modules.

export const $ = (sel, root = document) => root.querySelector(sel);
export const $$ = (sel, root = document) => [...root.querySelectorAll(sel)];

/** el('div.cls#id', {attr: v, on: {click: fn}}, ...children) */
export function el(spec, attrs = {}, ...children) {
  const m = spec.match(/^([a-z0-9]+)?((?:[.#][\w-]+)*)$/i);
  const node = document.createElement((m && m[1]) || 'div');
  if (m && m[2]) {
    for (const part of m[2].match(/[.#][\w-]+/g)) {
      if (part[0] === '.') node.classList.add(part.slice(1));
      else node.id = part.slice(1);
    }
  }
  for (const [k, v] of Object.entries(attrs || {})) {
    if (v === undefined || v === null || v === false) continue;
    if (k === 'on') for (const [ev, fn] of Object.entries(v)) node.addEventListener(ev, fn);
    else if (k === 'text') node.textContent = v;
    else if (k === 'html') node.innerHTML = v;
    else if (k in node && typeof v !== 'string') node[k] = v;
    else node.setAttribute(k, v === true ? '' : v);
  }
  for (const c of children.flat()) {
    if (c === null || c === undefined || c === false) continue;
    node.append(c instanceof Node ? c : document.createTextNode(String(c)));
  }
  return node;
}

export const clone = (x) => (x === undefined ? undefined : JSON.parse(JSON.stringify(x)));

export function splitPath(path) {
  return path.split('.').map((p) => (/^-?\d+$/.test(p) ? Number(p) : p));
}

export function getPath(obj, path) {
  let cur = obj;
  for (const k of splitPath(path)) {
    if (cur === null || cur === undefined) return undefined;
    cur = cur[k];
  }
  return cur;
}

export function setPath(obj, path, value) {
  const keys = splitPath(path);
  let cur = obj;
  for (const k of keys.slice(0, -1)) {
    if (cur[k] === null || cur[k] === undefined) cur[k] = {};
    cur = cur[k];
  }
  cur[keys[keys.length - 1]] = value;
}

/** Leaf-level diff of two JSON trees: [{path, a, b}] */
export function diff(a, b, prefix = '') {
  const out = [];
  const isObj = (x) => x && typeof x === 'object' && !Array.isArray(x);
  if (isObj(a) && isObj(b)) {
    for (const k of [...new Set([...Object.keys(a), ...Object.keys(b)])].sort()) {
      out.push(...diff(a[k], b[k], `${prefix}${k}.`));
    }
  } else if (Array.isArray(a) && Array.isArray(b) && a.length === b.length) {
    a.forEach((x, i) => out.push(...diff(x, b[i], `${prefix}${i}.`)));
  } else if (JSON.stringify(a) !== JSON.stringify(b)) {
    out.push({ path: prefix.slice(0, -1), a, b });
  }
  return out;
}

/** Numeric (non-boolean) leaves as {path: value}. */
export function numericLeaves(obj, prefix = '', out = {}) {
  if (Array.isArray(obj)) obj.forEach((v, i) => numericLeaves(v, `${prefix}${i}.`, out));
  else if (obj && typeof obj === 'object') for (const [k, v] of Object.entries(obj)) numericLeaves(v, `${prefix}${k}.`, out);
  else if (typeof obj === 'number') out[prefix.slice(0, -1)] = obj;
  return out;
}

export function debounce(fn, ms) {
  let t = null;
  return (...a) => {
    clearTimeout(t);
    t = setTimeout(() => fn(...a), ms);
  };
}

/** Number formatting for engineering readouts. */
export function fmt(x, sig = 4) {
  if (x === null || x === undefined || Number.isNaN(x)) return '–';
  if (typeof x !== 'number') return String(x);
  if (x === 0) return '0';
  const a = Math.abs(x);
  if (a >= 1e5 || a < 1e-3) return x.toExponential(sig - 1).replace('e+', 'e');
  return Number(x.toPrecision(sig)).toString();
}

let toastTimer = null;
export function toast(msg, isErr = false, ms = 3500) {
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.className = 'toast' + (isErr ? ' err' : '');
  t.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => (t.hidden = true), ms);
}

export function download(name, blobOrText, type = 'application/json') {
  const blob = blobOrText instanceof Blob ? blobOrText : new Blob([blobOrText], { type });
  const a = el('a', { href: URL.createObjectURL(blob), download: name });
  document.body.append(a);
  a.click();
  setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 1000);
}

/** Colour for a utilisation value: green → amber → red. */
export function utilColour(u) {
  if (u === null || u === undefined || Number.isNaN(u)) return '#9aa3ad';
  if (u > 1) return '#a50f2a';
  // blue (0) -> green (0.35) -> yellow (0.7) -> red (1.0)
  const stops = [[0, [43, 110, 214]], [0.35, [44, 170, 90]], [0.7, [240, 205, 40]], [1, [230, 50, 35]]];
  const t = Math.max(0, u);
  let k = 0;
  while (k < stops.length - 2 && t > stops[k + 1][0]) k++;
  const [t0, c0] = stops[k], [t1, c1] = stops[k + 1];
  const s = Math.min(1, (t - t0) / (t1 - t0));
  return `rgb(${c0.map((a, i) => Math.round(a + (c1[i] - a) * s)).join(',')})`;
}

export const utilClass = (u) => (u > 1 ? 'u-bad' : u > 0.8 ? 'u-warn' : 'u-ok');
