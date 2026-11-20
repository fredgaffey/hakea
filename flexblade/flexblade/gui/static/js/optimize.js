// Optimisation panel: variables (dotted Design paths + bounds), targets (dotted Results paths),
// background job with polling, history plot, apply best.

import { el, fmt, getPath, numericLeaves, diff, toast } from './util.js';
import { api } from './api.js';
import { lineChart, legend } from './charts.js';

const DEFAULT_VARS = [
  { path: 'flexure.thickness', lo: 0.0004, hi: 0.002, log: false },
  { path: 'flexure.skew_deg', lo: 20, hi: 60, log: false },
];
const DEFAULT_TARGETS = [
  { quantity: 'harmonic.control_ratio', kind: 'max', value: null, weight: 1 },
  { quantity: 'checks.max_utilisation', kind: 'le', value: 0.8, weight: 10 },
];

export class OptimisePanel {
  constructor(box, { getDesign, getResults, apply, suggestions }) {
    this.box = box;
    this.getDesign = getDesign;
    this.getResults = getResults;
    this.apply = apply;
    this.suggestions = suggestions;
    this.vars = DEFAULT_VARS.map((v) => ({ ...v }));
    this.targets = DEFAULT_TARGETS.map((t) => ({ ...t }));
    this.method = 'differential_evolution';
    this.maxEvals = 200;
    this.seed = 0;
    this.job = null;
    this.render();
  }

  datalists() {
    const d = this.getDesign() || {};
    const paths = Object.keys(numericLeaves(d)).filter((p) => !/\.(build_dir|raster_dir|width_dir|p0|p1)\./.test(p));
    const r = this.getResults();
    const rq = new Set(this.suggestions);
    if (r?.summary) for (const k of Object.keys(r.summary)) rq.add(`summary.${k}`);
    return el('div', {},
      el('datalist#dl-vars', {}, ...paths.map((p) => el('option', { value: p }))),
      el('datalist#dl-tgts', {}, ...[...rq].map((p) => el('option', { value: p }))));
  }

  render() {
    const b = this.box;
    b.replaceChildren();
    b.append(this.datalists());
    const varRows = this.vars.map((v, i) => el('div.opt-row.var', {},
      el('input', { type: 'text', value: v.path, list: 'dl-vars', class: 'mono', title: 'dotted path into Design (SI)',
        on: { change: (e) => (v.path = e.target.value.trim()) } }),
      el('input', { type: 'number', step: 'any', value: v.lo, title: 'lower bound (SI)', on: { change: (e) => (v.lo = Number(e.target.value)) } }),
      el('input', { type: 'number', step: 'any', value: v.hi, title: 'upper bound (SI)', on: { change: (e) => (v.hi = Number(e.target.value)) } }),
      el('label.toggle', { title: 'log-scaled' }, el('input', { type: 'checkbox', checked: v.log, on: { change: (e) => (v.log = e.target.checked) } }), 'log'),
      el('button.small', { title: 'remove', on: { click: () => { this.vars.splice(i, 1); this.render(); } } }, '×')));
    const tgtRows = this.targets.map((t, i) => el('div.opt-row.tgt', {},
      el('input', { type: 'text', value: t.quantity, list: 'dl-tgts', class: 'mono', title: 'dotted path into Results',
        on: { change: (e) => (t.quantity = e.target.value.trim()) } }),
      el('select', { on: { change: (e) => (t.kind = e.target.value) } },
        ...['min', 'max', 'eq', 'le', 'ge'].map((k) => el('option', { value: k, selected: k === t.kind }, k))),
      el('input', { type: 'number', step: 'any', value: t.value ?? '', placeholder: 'value',
        on: { change: (e) => (t.value = e.target.value === '' ? null : Number(e.target.value)) } }),
      el('input', { type: 'number', step: 'any', value: t.weight, title: 'weight', on: { change: (e) => (t.weight = Number(e.target.value)) } }),
      el('button.small', { title: 'remove', on: { click: () => { this.targets.splice(i, 1); this.render(); } } }, '×')));
    b.append(
      el('h3', {}, 'Variables'), el('div.opt-rows', {}, ...varRows),
      el('button.small', { on: { click: () => { this.vars.push({ path: '', lo: 0, hi: 1, log: false }); this.render(); } } }, '+ variable'),
      el('h3', {}, 'Targets'), el('div.opt-rows', {}, ...tgtRows),
      el('button.small', { on: { click: () => { this.targets.push({ quantity: '', kind: 'min', value: null, weight: 1 }); this.render(); } } }, '+ target'),
      el('div.opt-ctl', {},
        el('select', { on: { change: (e) => (this.method = e.target.value) } },
          ...['differential_evolution', 'nelder_mead', 'slsqp'].map((m) => el('option', { value: m, selected: m === this.method }, m))),
        el('label.toggle', {}, 'max evals', el('input', { type: 'number', value: this.maxEvals, on: { change: (e) => (this.maxEvals = Number(e.target.value)) } })),
        el('label.toggle', {}, 'seed', el('input', { type: 'number', value: this.seed, on: { change: (e) => (this.seed = Number(e.target.value)) } })),
        (this.runBtn = el('button.primary', { on: { click: () => this.run() } }, 'Run'))),
      el('div.hint', {}, 'Bounds in SI (m, Pa, …; *_deg in degrees). Objective = Σ weight × penalty; le/ge are one-sided.'),
      (this.statusBox = el('div')),
    );
    if (this.job) this.renderJob(this.job);
  }

  async run() {
    const design = this.getDesign();
    for (const v of this.vars) {
      if (getPath(design, v.path) === undefined) { toast(`unknown design path ${v.path}`, true); return; }
    }
    const request = {
      design, method: this.method, max_evals: this.maxEvals, seed: this.seed,
      variables: this.vars.filter((v) => v.path),
      targets: this.targets.filter((t) => t.quantity).map((t) => ({ ...t, value: t.value ?? undefined })),
    };
    try {
      const { job_id } = await api.optimize(request);
      this.runBtn.disabled = true;
      this.poll(job_id);
    } catch (e) { toast(e.message, true, 6000); }
  }

  async poll(id) {
    try {
      const j = await api.job(id);
      this.job = j;
      this.renderJob(j);
      if (j.status === 'queued' || j.status === 'running') setTimeout(() => this.poll(id), 500);
      else this.runBtn.disabled = false;
    } catch (e) { toast(e.message, true); this.runBtn.disabled = false; }
  }

  renderJob(j) {
    const b = this.statusBox;
    b.replaceChildren();
    const p = j.progress || {};
    const frac = p.n ? p.i / p.n : (j.status === 'done' ? 1 : 0);
    b.append(el('h3', {}, `Job ${j.id} · ${j.status}`),
      el('div.progress', {}, el('div', { style: `width:${(frac * 100).toFixed(1)}%` })),
      el('div.hint', {}, `${p.i ?? 0}${p.n ? '/' + p.n : ''} evals · ${fmt(j.elapsed_s, 3)} s` +
        (p.best !== null && p.best !== undefined ? ` · best ${fmt(p.best)}` : '')));
    if (j.history?.length) {
      const x = j.history.map((h, i) => h.eval ?? i);
      const series = [
        { x, y: j.history.map((h) => h.objective), points: true, color: 'var(--faint)', label: 'objective', r: 1.8 },
        { x, y: j.history.map((h) => h.best), color: '#e9a23b', label: 'best so far' },
      ];
      b.append(lineChart({ series, xlabel: 'evaluation', ylabel: 'objective', height: 200 }), legend(series));
    }
    if (j.status === 'error') b.append(el('pre.json', {}, j.error));
    if (j.status === 'done' && j.result) {
      const r = j.result;
      const changes = diff(this.getDesign(), r.design).filter((c) => typeof c.b === 'number');
      b.append(el('div.hint', {}, `objective ${fmt(r.objective)} · ${r.n_evals} evals · ${r.converged ? 'converged' : 'not converged'} · ${r.message || ''}`),
        el('table.t', {}, el('tbody', {}, ...changes.map((c) => el('tr', {}, el('td.l', {}, c.path), el('td.r', {}, fmt(c.a)), el('td.r', {}, '→'), el('td.r', {}, fmt(c.b)))))),
        el('div.opt-ctl', {}, el('button.primary', { on: { click: () => this.apply(r.design, r.results) } }, 'Apply best')),
        r.digest ? el('pre.json', {}, r.digest) : null);
    }
  }
}
