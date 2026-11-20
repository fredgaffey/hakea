// "Compare materials" panel: material sweep over one layup slot of the current design,
// run as a background job; sortable results table; click a row to apply that material.

import { el, fmt, toast, utilClass } from './util.js';
import { api } from './api.js';

const SELECTORS = [
  ['filaments', 'all real filaments'],
  ['class:tpu+tpe', 'TPU / TPE'],
  ['rigid', 'all rigid (non-elastomer)'],
  ['class:pa_cf+ppa_cf+pet_cf+petg_cf+pla_cf+abs_cf+asa_cf+pc_cf+onyx', 'fibre-filled (CF)'],
  ['class:pa+pa_cf+pa_gf+ppa_cf+onyx:cond', 'nylons incl. conditioned'],
  ['inserts', 'inserts (rods / wire)'],
  ['all', 'everything incl. generic'],
];

const COLS = [
  ['material', 'material', (r) => (r.brand && r.brand !== 'generic' ? `${r.brand} ${r.product}` : r.material)],
  ['fitness', 'fitness', (r) => fmt(r.fitness, 3)],
  ['control_ratio', 'ctrl ratio', (r) => fmt(r.control_ratio, 3)],
  ['pitch_1p_deg', 'pitch 1P °', (r) => fmt(r.pitch_1p_deg, 3)],
  ['max_util', 'max util', (r) => fmt(r.max_util, 3)],
  ['governing', 'governs', (r) => r.governing || ''],
  ['lag_per_rev', 'lag /rev', (r) => fmt(r.lag_per_rev, 3)],
  ['pitch_per_rev', 'pitch /rev', (r) => fmt(r.pitch_per_rev, 3)],
  ['pitch_split_deg', 'split °', (r) => fmt(r.pitch_split_deg, 3)],
  ['rotor_mass_g', 'mass g', (r) => fmt(r.rotor_mass_g, 3)],
  ['filament_cost_usd', 'cost $', (r) => fmt(r.filament_cost_usd, 2)],
  ['usd_per_kg', '$/kg', (r) => fmt(r.usd_per_kg, 3)],
];

export class MaterialsPanel {
  constructor(box, { getDesign, apply }) {
    this.box = box;
    this.getDesign = getDesign;
    this.apply = apply;
    this.slot = 'body';
    this.selector = 'filaments';
    this.resize = true;
    this.torque = 'capability';
    this.sortKey = 'fitness';
    this.sortDesc = true;
    this.result = null;
    this.job = null;
    this.slots = {};
    this.render();
  }

  async refreshSlots() {
    try {
      this.slots = (await api.materialSlots(this.getDesign())).slots;
      if (!this.slots[this.slot]) this.slot = Object.keys(this.slots)[0] || 'body';
    } catch { this.slots = {}; }
    this.render();
  }

  async run() {
    try {
      const { job_id, n } = await api.materialSweep({
        design: this.getDesign(), slot: this.slot, materials: this.selector,
        resize: this.resize, torque: this.torque });
      toast(`material sweep: ${n} materials`);
      this.poll(job_id);
    } catch (e) { toast(e.message, true, 6000); }
  }

  async poll(id) {
    const j = await api.job(id);
    this.job = j;
    if (j.status === 'done') this.result = j.result;
    if (j.status === 'error') toast(j.error, true, 8000);
    this.render();
    if (j.status === 'queued' || j.status === 'running') setTimeout(() => this.poll(id), 800);
  }

  sorted() {
    const rows = [...(this.result?.rows || [])];
    const k = this.sortKey;
    const val = (r) => (k === 'material' ? (COLS[0][2](r) || '') : r[k]);
    rows.sort((a, b) => {
      const x = val(a), y = val(b);
      if (x == null && y == null) return 0;
      if (x == null) return 1;
      if (y == null) return -1;
      const c = typeof x === 'string' ? x.localeCompare(y) : x - y;
      return this.sortDesc ? -c : c;
    });
    return rows;
  }

  render() {
    const b = this.box;
    const slotNames = Object.keys(this.slots).length ? Object.keys(this.slots) : ['body', 'blade', 'flexure', 'skin', 'insert'];
    const running = this.job && (this.job.status === 'queued' || this.job.status === 'running');
    const controls = el('div.opt-row', {},
      el('label', {}, 'slot ', el('select', { on: { change: (e) => (this.slot = e.target.value) } },
        ...slotNames.map((s) => el('option', { value: s, selected: s === this.slot },
          this.slots[s] ? `${s} (${this.slots[s].join(', ')})` : s)))),
      el('label', {}, ' materials ', el('select', { on: { change: (e) => (this.selector = e.target.value) } },
        ...SELECTORS.map(([v, t]) => el('option', { value: v, selected: v === this.selector }, t)))),
      el('label.toggle', { title: 'resize geometry to the same section stiffness (fair comparison)' },
        el('input', { type: 'checkbox', checked: this.resize, on: { change: (e) => (this.resize = e.target.checked) } }), 'resize to same stiffness'),
      el('label', {}, ' torque ', el('select', { on: { change: (e) => (this.torque = e.target.value) } },
        ...[['capability', 'motor capability'], ['design', 'design op']].map(([v, t]) => el('option', { value: v, selected: v === this.torque }, t)))),
      el('button', { disabled: running, on: { click: () => this.run() } }, running ? 'running…' : 'Compare materials'),
      el('button.small', { title: 'refresh slots from the current design', on: { click: () => this.refreshSlots() } }, '⟳'));
    const prog = this.job ? el('p.muted', {},
      `${this.job.status} ${this.job.progress?.i ?? 0}/${this.job.progress?.n ?? '?'} · ${fmt(this.job.elapsed_s, 3)} s`) : null;
    let table = null;
    if (this.result) {
      const head = el('tr', {}, ...COLS.map(([k, t]) => el(k === 'material' || k === 'governing' ? 'th' : 'th.r', {
        style: 'cursor:pointer', title: 'sort',
        on: { click: () => { if (this.sortKey === k) this.sortDesc = !this.sortDesc; else { this.sortKey = k; this.sortDesc = k !== 'material'; } this.render(); } },
      }, t + (this.sortKey === k ? (this.sortDesc ? ' ▾' : ' ▴') : ''))), el('th', {}, ''));
      const body = this.sorted().map((r) => el('tr', { title: r.error || r.material },
        ...COLS.map(([k, , f]) => {
          if (k === 'material') return el('td.l', {}, r.url ? el('a', { href: r.url, target: '_blank', rel: 'noopener' }, f(r)) : f(r));
          if (k === 'max_util') return el(`td.r.${utilClass(r.max_util ?? 0)}`, {}, f(r));
          if (k === 'governing') return el('td.l', {}, r.error ? `error: ${r.error}` : f(r));
          return el('td.r', {}, r[k] == null ? '' : f(r));
        }),
        el('td', {}, el('button.small', { title: 'apply this material to the design (geometry unchanged)',
          on: { click: () => this.apply(this.result.slot, r.material) } }, 'use'))));
      table = el('div', { style: 'overflow-x:auto' },
        el('p.muted', {}, `slot ${this.result.slot} · ${this.result.resized ? 'resized to same section stiffness' : 'same geometry'} · `
          + 'fitness = control_ratio × (1 if util ≤ 1 else 1/util²) × min(1, margin/0.15) for lag & pitch'),
        el('table.t', {}, el('thead', {}, head), el('tbody', {}, ...body)));
    }
    b.replaceChildren(el('div', {}, controls, prog, table || el('p.empty', {}, 'Pick a slot and run a material comparison.')));
  }
}
