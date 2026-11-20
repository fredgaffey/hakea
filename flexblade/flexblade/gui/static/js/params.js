// Parameter panel generated from the Design JSON schema (pydantic) + display hints.
// Values in the design stay SI; sliders/inputs show display units (hints.fields[path].scale).

import { el, getPath, fmt } from './util.js';

let SCHEMA = null;
let HINTS = null;
let MATERIALS = [];
let onChange = () => {};
let root = null;
let CURRENT = null; // design being edited (for "copy from upper" overrides)

// coaxial.lower_X: null = copy the upper rotor's X
const LOWER_OF = { 'coaxial.lower_blade': 'blade', 'coaxial.lower_flexure': 'flexure', 'coaxial.lower_motor': 'motor' };

export function initParams(container, { schema, hints, materials, change }) {
  root = container;
  SCHEMA = schema;
  HINTS = hints;
  MATERIALS = materials.map((m) => m.name);
  onChange = change;
}

const resolve = (node) => {
  if (node && node.$ref) {
    const name = node.$ref.split('/').pop();
    return { ...SCHEMA.$defs[name], ...Object.fromEntries(Object.entries(node).filter(([k]) => k !== '$ref')) };
  }
  return node || {};
};

/** Strip Optional: returns [innerSchema, nullable] */
function unwrap(node) {
  node = resolve(node);
  if (node.anyOf) {
    const nonNull = node.anyOf.filter((x) => x.type !== 'null');
    const nullable = nonNull.length < node.anyOf.length;
    if (nonNull.length === 1) {
      const inner = resolve(nonNull[0]);
      return [{ ...inner, description: node.description || inner.description, title: node.title || inner.title }, nullable];
    }
    return [node, nullable];
  }
  return [node, false];
}

function hintFor(path) {
  const f = HINTS.fields;
  if (f[path]) return f[path];
  const last = path.split('.').pop();
  return f[`*.${last}`] || null;
}

function kindOf(s) {
  if (s.enum) return 'enum';
  if (s.type === 'number' || s.type === 'integer') return 'number';
  if (s.type === 'boolean') return 'bool';
  if (s.type === 'string') return 'string';
  if (s.type === 'array' && s.prefixItems && s.prefixItems.length === 3) return 'vec3';
  if (s.type === 'array' && s.items && ['number', 'integer'].includes(resolve(s.items).type)) return 'numlist';
  if (s.type === 'object' && s.properties) return 'object';
  return 'json';
}

function label(path, s) {
  const key = path.split('.').pop();
  return el('span.name', { title: `${path}${s.description ? '\n' + s.description : ''}`, 'data-name': path }, key);
}

function rangeFor(path, s, v, h) {
  const scale = h ? h.scale : 1;
  let lo = h?.min, hi = h?.max;
  if (lo === undefined) {
    lo = s.minimum ?? s.exclusiveMinimum ?? (v < 0 ? 3 * v * scale : 0);
    if (s.exclusiveMinimum !== undefined || s.minimum !== undefined) lo *= scale;
  }
  if (hi === undefined) {
    hi = s.maximum !== undefined ? s.maximum * scale : Math.max(Math.abs(v * scale) * 3, 1);
  }
  return { lo, hi, step: h?.step ?? (s.type === 'integer' ? 1 : (hi - lo) / 200), log: !!h?.log };
}

function numberField(path, s, value, nullable) {
  const h = hintFor(path);
  const scale = h ? h.scale : 1;
  const unit = h ? h.unit : '';
  const isInt = s.type === 'integer';
  const cur = value ?? (s.default ?? h?.min ?? 0);
  const r = rangeFor(path, s, cur, h);
  const num = el('input', { type: 'number', step: 'any', value: value === null ? '' : +(cur * scale).toPrecision(6) });
  const slider = el('input', { type: 'range', min: 0, max: 1000, step: 1 });
  const toSlider = (d) => (r.log ? (Math.log10(d / r.lo) / Math.log10(r.hi / r.lo)) : ((d - r.lo) / (r.hi - r.lo))) * 1000;
  const fromSlider = (t) => {
    const u = t / 1000;
    let d = r.log ? r.lo * (r.hi / r.lo) ** u : r.lo + (r.hi - r.lo) * u;
    if (!r.log && r.step) d = Math.round(d / r.step) * r.step;
    return d;
  };
  slider.value = toSlider(cur * scale);
  let auto = null;
  const commit = (disp) => {
    let si = disp / scale;
    if (isInt) si = Math.round(si);
    onChange(path, si);
  };
  num.addEventListener('change', () => {
    if (num.value === '' && nullable) { if (auto) auto.checked = true; onChange(path, null); return; }
    const d = Number(num.value);
    if (!Number.isFinite(d)) return;
    slider.value = toSlider(d);
    if (auto) auto.checked = false;
    commit(d);
  });
  slider.addEventListener('input', () => {
    const d = fromSlider(Number(slider.value));
    num.value = +(isInt ? Math.round(d) : d).toPrecision(6);
    if (auto) auto.checked = false;
    commit(d);
  });
  const ctl = el('span.ctl', {}, num, el('span.unit', {}, unit));
  if (nullable) {
    auto = el('input', { type: 'checkbox', checked: value === null });
    auto.addEventListener('change', () => {
      if (auto.checked) { num.value = ''; onChange(path, null); }
      else { const d = fromSlider(Number(slider.value)); num.value = +d.toPrecision(6); commit(d); }
    });
    ctl.prepend(el('label.auto', { title: 'null = solved automatically' }, auto, 'auto'));
  }
  return el('div.f', { 'data-path': path }, label(path, s), ctl, slider);
}

function enumField(path, s, value, nullable, options) {
  const opts = options || s.enum;
  const sel = el('select', {},
    nullable ? el('option', { value: '' }, '(none)') : null,
    ...opts.map((o) => el('option', { value: o, selected: o === value }, o)));
  if (value !== null && value !== undefined && !opts.includes(value)) sel.append(el('option', { value, selected: true }, `${value} (?)`));
  if (value === null) sel.value = '';
  sel.addEventListener('change', () => onChange(path, sel.value === '' && nullable ? null : sel.value));
  return el('div.f', { 'data-path': path }, label(path, s), el('span.ctl', {}, sel));
}

function boolField(path, s, value) {
  const cb = el('input', { type: 'checkbox', checked: !!value });
  cb.addEventListener('change', () => onChange(path, cb.checked));
  return el('div.f', { 'data-path': path }, label(path, s), el('span.ctl', {}, cb));
}

function stringField(path, s, value, nullable) {
  const key = path.split('.').pop();
  if (HINTS.material_fields.includes(key)) return enumField(path, s, value, nullable, MATERIALS);
  if (key === 'notes') {
    const ta = el('textarea', { rows: 2 }, value || '');
    ta.addEventListener('change', () => onChange(path, ta.value));
    return el('div.f', { 'data-path': path }, label(path, s), ta);
  }
  const inp = el('input', { type: 'text', value: value ?? '' });
  inp.addEventListener('change', () => onChange(path, inp.value === '' && nullable ? null : inp.value));
  return el('div.f', { 'data-path': path }, label(path, s), el('span.ctl', {}, inp));
}

function vec3Field(path, s, value) {
  const v = value || [0, 0, 0];
  const inputs = v.map((x, i) => {
    const inp = el('input', { type: 'number', step: 'any', value: x, title: 'xyz'[i] });
    inp.addEventListener('change', () => onChange(path, inputs.map((q) => Number(q.value))));
    return inp;
  });
  return el('div.f', { 'data-path': path }, label(path, s), el('span.ctl.vec', {}, ...inputs, el('span.unit', {}, 'H')));
}

function numlistField(path, s, value) {
  const inp = el('input', { type: 'text', value: (value || []).join(', '), placeholder: 'empty = auto', class: 'mono' });
  const f = el('div.f', { 'data-path': path }, label(path, s), el('span.ctl', {}, inp));
  inp.addEventListener('change', () => {
    const parts = inp.value.split(/[,\s]+/).filter(Boolean).map(Number);
    if (parts.some((x) => !Number.isFinite(x))) { f.classList.add('invalid'); return; }
    f.classList.remove('invalid');
    onChange(path, parts);
  });
  return f;
}

function jsonField(path, s, value) {
  const ta = el('textarea', { rows: Math.min(10, Math.max(2, JSON.stringify(value ?? null, null, 1).split('\n').length)), spellcheck: false },
    JSON.stringify(value ?? null, null, 1));
  const f = el('div.f', { 'data-path': path }, label(path, s), ta,
    s.description ? el('div.help', {}, s.description) : null);
  ta.addEventListener('change', () => {
    try {
      const v = JSON.parse(ta.value);
      f.classList.remove('invalid');
      onChange(path, v);
    } catch { f.classList.add('invalid'); }
  });
  const n = Array.isArray(value) ? ` [${value.length}]` : value === null ? ' (none)' : '';
  return el('details.sub', {}, el('summary', {}, `${path.split('.').pop()}${n}`), f);
}

function objectFields(path, s, value) {
  const out = [], adv = [];
  const val = value || {};
  for (const [k, sub] of Object.entries(s.properties)) {
    const p = path ? `${path}.${k}` : k;
    ((HINTS.advanced || []).includes(p) ? adv : out).push(field(p, sub, val[k]));
  }
  if (adv.length) out.push(el('details.sub', {}, el('summary', {}, 'advanced (internal, set by solvers)'), ...adv));
  return out;
}

function field(path, node, value) {
  const [s, nullable] = unwrap(node);
  switch (kindOf(s)) {
    case 'number': return numberField(path, s, value, nullable);
    case 'enum': return enumField(path, s, value, nullable);
    case 'bool': return boolField(path, s, value);
    case 'string': return stringField(path, s, value, nullable);
    case 'vec3': return vec3Field(path, s, value);
    case 'numlist': return numlistField(path, s, value);
    case 'object':
      if (LOWER_OF[path]) {
        const key = path.split('.').pop();
        if (value === null || value === undefined) {
          return el('div.f', { 'data-path': path }, el('span.name', { 'data-name': path, title: s.description || '' }, key),
            el('span.ctl', {}, el('span.unit', {}, 'same as upper'),
              el('button.small', { on: { click: () => onChange(path, JSON.parse(JSON.stringify(CURRENT[LOWER_OF[path]])), { rerender: true }) } }, 'override')));
        }
        return el('details.sub', { open: true }, el('summary', {}, `${key} (override)`),
          el('button.small', { on: { click: () => onChange(path, null, { rerender: true }) } }, 'use upper'),
          ...objectFields(path, s, value));
      }
      if (nullable && (value === null || value === undefined)) return jsonField(path, s, value);
      return el('details.sub', { open: false }, el('summary', {}, path.split('.').pop()), ...objectFields(path, s, value));
    default: return jsonField(path, s, value);
  }
}

/** Build the whole panel for `design`; `base` (preset) used for change markers. */
export function renderParams(design, base) {
  CURRENT = design;
  root.replaceChildren();
  const props = SCHEMA.properties;
  if (props.layout) root.append(el('div.group-body.layout-row', {}, field('layout', props.layout, design.layout)));
  for (const g of HINTS.groups) {
    if (!props[g.key]) continue;
    if (g.when && Object.entries(g.when).some(([k, v]) => design[k] !== v)) continue;
    const s = resolve(props[g.key]);
    const open = ['blade', 'flexure', 'op'].includes(g.key);
    root.append(el('details.group', { open, 'data-group': g.key },
      el('summary', {}, g.label),
      el('div.group-body', {}, ...objectFields(g.key, s, design[g.key]))));
  }
  root.append(el('details.group', {}, el('summary', {}, 'Notes'),
    el('div.group-body', {}, field('notes', props.notes, design.notes))));
  markChanged(design, base);
}

export function markChanged(design, base) {
  if (!root) return;
  for (const n of root.querySelectorAll('[data-name]')) {
    const p = n.getAttribute('data-name');
    const changed = base && JSON.stringify(getPath(design, p)) !== JSON.stringify(getPath(base, p));
    n.classList.toggle('changed', !!changed);
  }
}

export { fmt };
