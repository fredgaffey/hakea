// FEA tab: runs POST /api/fea as a background job, then drives the FEA surface view in the viewport.

import { $, el, fmt, toast } from './util.js';
import { api } from './api.js';
import { FEA_FIELDS, rampColour } from './fea-view.js';

const css = (c) => `rgb(${c.map((x) => Math.round(x * 255)).join(',')})`;

export class FeaPanel {
  constructor(box, { getDesign, ensureViewer, viewer, status }) {
    this.box = box;
    this.getDesign = getDesign;
    this.ensureViewer = ensureViewer;
    this.viewerNow = viewer;
    this.status = status;
    this.data = null;
    this.running = false;
    this.ready = false;       // for automation: true once a result is displayed
    this.build();
  }

  build() {
    const b = this.box;
    this.runBtn = el('button.primary', { on: { click: () => this.run() } }, 'Run FEA');
    this.coarse = el('input', { type: 'checkbox', checked: true });
    this.rpm = el('input', { type: 'number', min: 0, step: 100, placeholder: 'design', style: 'width:80px' });
    this.msg = el('div.fea-msg', {}, 'Static centrifugal load case of blade 1 + flexure (hub side clamped). ');
    this.ctl = el('div.fea-ctl', { hidden: true });
    this.sum = el('div.fea-sum');
    b.append(
      el('div.opt-ctl', {}, this.runBtn,
        el('label.toggle', { title: 'Coarse mesh (1.6x element size): seconds instead of a minute' }, this.coarse, ' coarse mesh'),
        el('label', {}, 'rpm ', this.rpm)),
      this.msg, this.ctl, this.sum);
    const sel = el('select', { id: 'fea-field' }, ...Object.entries(FEA_FIELDS).map(([k, f]) => el('option', { value: k }, f.label)));
    const mk = (id, text, checked, on) => el('label.toggle', {}, el('input', { type: 'checkbox', id, checked, on: { change: (e) => on(e.target.checked) } }), ` ${text}`);
    this.scaleIn = el('input', { type: 'range', id: 'fea-scale', min: 1, max: 200, step: 1, value: 1,
      on: { input: (e) => { this.view?.setScale(Number(e.target.value)); this.scaleOut.textContent = `×${e.target.value}`; } } });
    this.scaleOut = el('output', {}, '');
    sel.onchange = () => { this.view?.setField(sel.value); this.legend(); };
    this.ctl.append(
      el('div.row', {}, el('label', {}, 'field ', sel)),
      el('div.row', {},
        mk('fea-clip', 'clip peaks (p99)', true, (v) => { this.view?.setClip(v); this.legend(); }),
        mk('fea-wire', 'wireframe (mesh)', true, (v) => this.view?.setWire(v)),
        mk('fea-geom', 'show geometry', false, () => this.applyMode())),
      el('div.row', {},
        mk('fea-def', 'deformed', true, (v) => this.view?.setDeformed(v)),
        el('label', {}, 'scale ', this.scaleIn, this.scaleOut)),
      el('div.row.hint', {}, 'see-through (viewport toggle) makes the FEA surface translucent. Hover a face for its value.'));
  }

  get v() { return this.viewerNow(); }
  get view() { return this.v?.fea; }

  async run() {
    if (this.running) return;
    const v = await this.ensureViewer();
    if (!v) { toast('3D viewer unavailable', true); return; }
    this.running = true;
    this.ready = false;
    this.runBtn.disabled = true;
    const t0 = performance.now();
    const tick = setInterval(() => this.progress(`meshing + solving… ${((performance.now() - t0) / 1000).toFixed(0)} s`), 500);
    this.progress('submitting…');
    this.status('FEA running…', 'busy');
    try {
      const rpm = this.rpm.value ? Number(this.rpm.value) : undefined;
      const { job_id } = await api.fea(this.getDesign(), { coarse: this.coarse.checked, rpm });
      let j;
      for (;;) {
        j = await api.job(job_id);
        if (j.status === 'done' || j.status === 'error') break;
        await new Promise((r) => setTimeout(r, 400));
      }
      if (j.status === 'error') throw new Error(j.error);
      this.data = j.result;
      this.show(v);
      this.status(`FEA done in ${fmt((performance.now() - t0) / 1000, 3)} s`);
    } catch (e) {
      this.progress(`FEA failed: ${e.message}`, true);
      this.status(`FEA failed: ${e.message}`, 'err');
      toast(e.message, true, 8000);
    } finally {
      clearInterval(tick);
      this.running = false;
      this.runBtn.disabled = false;
    }
  }

  progress(text, err = false) {
    this.msg.textContent = text;
    this.msg.classList.toggle('err', err);
  }

  show(v) {
    const d = this.data;
    v.fea.setData(d);
    this.ctl.hidden = false;
    $('#fea-field').value = v.fea.field;
    $('#fea-wire').checked = v.fea.wire;
    $('#fea-def').checked = v.fea.deformed;
    this.scaleIn.max = Math.max(200, 4 * v.fea.autoScale);
    this.scaleIn.value = v.fea.scale;
    this.scaleOut.textContent = `×${v.fea.scale}`;
    v.fea.setSeeThrough(v.seeThrough);
    this.applyMode();
    v.fitCloud(d.vertices);
    this.progress(`${d.n_tets} tets, ${d.n_faces} surface faces · solved in ${fmt(d.timings?.total ?? 0, 3)} s`);
    this.summary();
    this.ready = true;
  }

  /** (Re)apply FEA/geometry visibility; call after the geometry was reloaded. */
  applyMode() {
    const v = this.v;
    if (!v || !this.data) return;
    v.setFeaMode(this.active !== false, $('#fea-geom')?.checked);
    this.legend();
  }

  setActive(on) {
    this.active = on;
    const v = this.v;
    if (!v) return;
    if (!this.data) return;
    v.setFeaMode(on, $('#fea-geom')?.checked);
    if (on) this.legend(); else $('#vp-legend').dataset.owner = '';
  }

  legend() {
    const v = this.v, box = $('#vp-legend');
    if (!v || !this.data || !v.fea.active) return;
    const r = v.fea.range(), spec = FEA_FIELDS[v.fea.field];
    const s = spec.scale, u = spec.unit ? ` [${spec.unit}]` : '';
    const stops = Array.from({ length: 9 }, (_, i) => `${css(rampColour(i / 8))} ${(i / 8) * 100}%`).join(',');
    const n = (x) => fmt(x * s, 3);
    box.replaceChildren(
      el('div.lg-title', {}, `${spec.label.split(' (')[0]}${u}`),
      el('div.lg-grad', { style: `background:linear-gradient(90deg,${stops})` }),
      el('div.lg-ticks', {}, el('span', {}, n(r.lo)), el('span', {}, n((r.lo + r.hi) / 2)), el('span', {}, `${n(r.hi)}${r.clipped ? '+' : ''}`)),
      el('div.lg-note', {}, `min ${n(r.min)} · max ${n(r.max)}${r.clipped ? ' (clipped at p99)' : ''}`),
      el('div.lg-note', {}, `deformation ${v.fea.deformed ? `×${v.fea.scale}` : 'off'} · max |u| ${fmt(v.fea.maxDisp * 1e3, 3)} mm`));
    box.dataset.owner = 'fea';
  }

  summary() {
    const d = this.data, s = d.summary || {};
    const rows = [['von Mises max', `${fmt((s.max_von_mises ?? 0) / 1e6, 3)} MPa`],
      ['max principal strain', `${fmt(d.fields.strain.max * 100, 3)} %`],
      ['max utilisation', fmt(d.fields.utilisation.max, 3)],
      ['rpm', fmt(s.rpm, 4)], ['tip stretch', `${fmt((s.radial_stretch_tip ?? 0) * 1e3, 3)} mm`],
      ['root force', `${fmt(s.centrifugal_force_root ?? 0, 4)} N`]];
    const perPart = Object.entries(s.parts || {}).map(([k, p]) => el('tr', {}, el('td.l', {}, k),
      el('td.r', {}, fmt(p.max_von_mises / 1e6, 3)), el('td.r', {}, fmt(p.util_principal_vs_Xt ?? 0, 3)), el('td.r', {}, fmt(p.util_interlayer_vs_Zt ?? 0, 3))));
    this.sum.replaceChildren(
      el('table.t', {}, el('tbody', {}, ...rows.map(([k, v]) => el('tr', {}, el('td.l', {}, k), el('td.r', {}, v))))),
      el('table.t', {}, el('thead', {}, el('tr', {}, el('th', {}, 'part'), el('th.r', {}, 'vM MPa'), el('th.r', {}, 'σ1/Xt'), el('th.r', {}, 'σz/Zt'))),
        el('tbody', {}, ...perPart)),
      ...(s.warnings || []).map((w) => el('p.warn', {}, w)));
  }

  markStale() {
    if (this.data && !this.running) this.progress('design changed since this FEA run (re-run to update)');
  }
}
