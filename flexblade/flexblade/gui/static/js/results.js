// Results panels: summary KPIs, checks table, Campbell diagram, harmonic response, raw JSON.

import { el, fmt, utilColour, utilClass } from './util.js';
import { lineChart, legend, phasorChart, PALETTE } from './charts.js';

const kpi = (k, v, u, cls) => el(`div.kpi${cls ? '.' + cls : ''}`, {}, el('div.k', {}, k), el('div', {}, el('span.v', {}, v), el('span.u', {}, u || '')));

export function maxUtil(r) {
  return (r.checks || []).reduce((m, c) => Math.max(m, c.utilisation), -Infinity);
}

function coaxialSummary(box, r) {
  const c = r.coaxial;
  box.append(el('h3', {}, 'Coaxial totals'), el('div.kpis', {},
    kpi('control ratio (both pulsed)', fmt(c.control_ratio, 3)),
    kpi('control moment max', fmt(c.control_moment_max * 1e3, 3), 'mN·m'),
    kpi('yaw authority', fmt(c.yaw_authority * 1e3, 3), 'mN·m'),
    kpi('net yaw torque', fmt(c.yaw_torque_net * 1e3, 3), 'mN·m', Math.abs(c.yaw_torque_net) > 0.2 * Math.abs(c.yaw_authority) ? 'bad' : ''),
    kpi('thrust total', fmt(c.thrust_total, 3), 'N'),
    kpi('power total', fmt(c.power_total, 3), 'W'),
    kpi('rpm upper / lower', `${fmt(c.rpm_upper, 5)} / ${fmt(c.rpm_lower, 5)}`),
    kpi('thrust share upper', fmt(c.thrust_share_upper, 3)),
    kpi('FM (system)', fmt(c.figure_of_merit, 3)),
    kpi('interference power ×', fmt(c.interference_power_factor, 3)),
    kpi('2P combined', fmt(c.moment_2p_combined * 1e3, 3), 'mN·m'),
  ));
  const row = (k, f) => el('tr', {}, el('td.l', {}, k), el('td.r', {}, f(c.upper)), el('td.r', {}, f(c.lower)));
  const mu = (x) => fmt(maxUtil(x), 3);
  box.append(el('h3', {}, 'Per rotor'), el('table.t', {},
    el('thead', {}, el('tr', {}, el('th', {}, ''), el('th.r', {}, 'upper'), el('th.r', {}, 'lower'))),
    el('tbody', {},
      row('rpm', (x) => fmt(x.trim?.rpm, 5)), row('thrust [N]', (x) => fmt(x.trim?.thrust, 3)),
      row('power [W]', (x) => fmt(x.trim?.power, 3)), row('torque [mN·m]', (x) => fmt((x.trim?.torque ?? NaN) * 1e3, 3)),
      row('collective [°]', (x) => fmt(x.trim?.collective_deg, 3)),
      row('hub moment [mN·m]', (x) => fmt((x.harmonic?.hub_moment_mean ?? NaN) * 1e3, 3)),
      row('moment phase [°]', (x) => fmt(x.harmonic?.hub_moment_phase_deg, 3)),
      row('control ratio', (x) => fmt(x.harmonic?.control_ratio, 3)),
      row('max utilisation', mu))));
}

/** One-line verdict: PASSES all checks (cyclic pitch, control ratio) / FAILS: governing check at Z x. */
function verdict(r) {
  const subs = r.coaxial ? [r.coaxial.upper, r.coaxial.lower].filter(Boolean) : [r];
  const checks = subs.flatMap((x) => x.checks || []);
  if (!checks.length) return null;
  const gov = checks.reduce((a, c) => (c.utilisation > a.utilisation ? c : a));
  const bl = subs[0].harmonic?.blades || [];
  const pitch = bl.length ? bl.reduce((a, b) => a + (b.pitch_amp_deg || 0), 0) / bl.length : null;
  const cr = r.coaxial ? r.coaxial.control_ratio : subs[0].harmonic?.control_ratio;
  if (gov.utilisation > 1) {
    return el('div.verdict.bad', {}, `FAILS: ${gov.name}${gov.member ? ` (${gov.member})` : ''} at ${fmt(gov.utilisation, 3)}×`);
  }
  const bits = [pitch !== null ? `cyclic pitch ${fmt(pitch, 3)}°` : null, cr !== undefined ? `control ratio ${fmt(cr, 3)}` : null].filter(Boolean);
  return el('div.verdict.ok', {}, `PASSES all checks${bits.length ? ' — ' + bits.join(', ') : ''}`,
    el('span.sub', {}, `tightest: ${gov.name} at ${fmt(gov.utilisation, 2)}×`));
}

export function renderSummary(box, r) {
  box.replaceChildren();
  if (!r) { box.append(el('p.empty', {}, 'Evaluate to see results.')); return; }
  const vd = verdict(r);
  if (vd) box.append(vd);
  if (r.coaxial) {
    coaxialSummary(box, r);
    if (r.summary && Object.keys(r.summary).length) {
      box.append(el('h3', {}, 'Summary'), el('table.t', {}, el('tbody', {},
        ...Object.entries(r.summary).map(([k, v]) => el('tr', {}, el('td.l', {}, k), el('td.r', {}, fmt(v)))))));
    }
    if (r.warnings?.length) box.append(el('h3', {}, 'Warnings'), el('ul.warnings', {}, ...r.warnings.map((w) => el('li', {}, w))));
    box.append(el('div.hint', {}, `design ${r.design_hash} · model ${r.model_version} · coaxial (use the rotor switch on Checks/Campbell/Harmonic)`));
    return;
  }
  const t = r.trim, h = r.harmonic;
  const mu = maxUtil(r);
  const nfail = (r.checks || []).filter((c) => c.utilisation > 1).length;
  if (h) {
    box.append(el('h3', {}, 'Cyclic control (1/rev)'), el('div.kpis', {},
      kpi('control ratio M/(W·R)', fmt(h.control_ratio, 3)),
      kpi('hub moment mean', fmt(h.hub_moment_mean * 1e3, 3), 'mN·m'),
      kpi('moment phase', fmt(h.hub_moment_phase_deg, 3), '°'),
      kpi('2P ripple', fmt(h.hub_moment_2p * 1e3, 3), 'mN·m'),
      kpi('tilt equivalent', fmt(h.tilt_equiv_deg, 3), '°'),
      kpi('moment / torque', fmt(h.moment_per_torque, 3)),
      kpi('torque amp', fmt(h.torque_amp * 1e3, 3), 'mN·m'),
      kpi('hub speed amp', fmt(h.hub_speed_amp_rpm, 3), 'rpm'),
    ));
  }
  if (t) {
    box.append(el('h3', {}, 'Trim'), el('div.kpis', {},
      kpi('rpm', fmt(t.rpm, 5)), kpi('thrust', fmt(t.thrust, 3), 'N'),
      kpi('power', fmt(t.power, 3), 'W'), kpi('FM', fmt(t.figure_of_merit, 3)),
      kpi('collective', fmt(t.collective_deg, 3), '°'), kpi('CF at root', fmt(t.centrifugal_force_root, 3), 'N'),
      kpi('coning', fmt(t.coning_deg, 3), '°'), kpi('root Δpitch', fmt(t.root_pitch_change_deg, 3), '°'),
    ));
  }
  if (Number.isFinite(mu)) {
    box.append(el('h3', {}, 'Checks'), el('div.kpis', {},
      kpi('max utilisation', fmt(mu, 3), '', mu > 1 ? 'bad' : 'ok'),
      kpi('failing', String(nfail), `of ${r.checks.length}`, nfail ? 'bad' : 'ok')));
  }
  if (r.summary && Object.keys(r.summary).length) {
    box.append(el('h3', {}, 'Summary'), el('table.t', {}, el('tbody', {},
      ...Object.entries(r.summary).map(([k, v]) => el('tr', {}, el('td.l', {}, k), el('td.r', {}, fmt(v)))))));
  }
  if (r.warnings?.length) box.append(el('h3', {}, 'Warnings'), el('ul.warnings', {}, ...r.warnings.map((w) => el('li', {}, w))));
  box.append(el('div.hint', {}, `design ${r.design_hash} · model ${r.model_version}` +
    (r.timings_s?.total !== undefined ? ` · ${fmt(r.timings_s.total, 3)} s` : '')));
}

export function renderChecks(box, r) {
  box.replaceChildren();
  if (!r?.checks?.length) { box.append(el('p.empty', {}, 'No checks.')); return; }
  const rows = [...r.checks].sort((a, b) => b.utilisation - a.utilisation).map((c) => {
    const w = Math.min(100, c.utilisation * 100);
    return el(`tr${c.utilisation > 1 ? '.fail' : ''}`, { title: c.note || '' },
      el('td.l', {}, c.name), el('td.l', {}, c.member),
      el('td.r', {}, fmt(c.value)), el('td.r', {}, fmt(c.allowable)),
      el('td.r', {}, el('div.util', {},
        el('div.bar-bg', {}, el('div.bar-fg', { style: `width:${w}%;background:${utilColour(c.utilisation)}` })),
        el(`span.${utilClass(c.utilisation)}`, {}, fmt(c.utilisation, 3)))));
  });
  box.append(el('table.t', {},
    el('thead', {}, el('tr', {}, el('th', {}, 'check'), el('th', {}, 'member'), el('th.r', {}, 'value'),
      el('th.r', {}, 'allow'), el('th.r', {}, 'util'))),
    el('tbody', {}, ...rows)));
  box.append(el('div.hint', {}, 'utilisation = value / allowable; > 1 fails. Hover a row for notes.'));
}

export function renderCampbell(box, r) {
  box.replaceChildren();
  const c = r?.campbell;
  const rpmOp = r?.trim?.rpm;
  if (!c?.rpm?.length) {
    if (r?.modes?.length) {
      box.append(el('h3', {}, 'Modes at operating rpm'), modesTable(r));
    } else box.append(el('p.empty', {}, 'No Campbell data (evaluate with modes).'));
    return;
  }
  const rpm = c.rpm;
  const nm = Math.max(...c.freqs_hz.map((f) => f.length));
  const series = [];
  for (let j = 0; j < nm; j++) {
    series.push({ x: rpm, y: c.freqs_hz.map((f) => f[j]), label: c.labels[j] || `mode ${j + 1}`, color: PALETTE[j % PALETTE.length] });
  }
  const fmax = Math.max(...series.flatMap((s) => s.y).filter(Number.isFinite));
  const rmax = Math.max(...rpm);
  const yTop = Math.min(fmax * 1.05, Math.max(4 * rmax / 60, rmax / 60 * 1.2));
  for (const n of [1, 2, 3, 4]) {
    series.push({ x: [0, rmax], y: [0, n * rmax / 60], color: 'var(--faint)', dash: '4 3', width: 1, endLabel: `${n}P` });
  }
  box.append(el('h3', {}, 'Campbell diagram'),
    lineChart({ series, xlabel: 'rotor speed [rpm]', ylabel: 'frequency [Hz]', xlim: [0, rmax], ylim: [0, yTop], height: 280,
      vlines: rpmOp ? [{ x: rpmOp, label: 'op' }] : [] }),
    legend(series.filter((s) => !s.dash)));
  if (r.modes?.length) box.append(el('h3', {}, 'Modes at operating rpm'), modesTable(r));
}

function modesTable(r) {
  const per = r.trim?.rpm ? r.trim.rpm / 60 : null;
  return el('table.t', {},
    el('thead', {}, el('tr', {}, el('th', {}, '#'), el('th', {}, 'label'), el('th.r', {}, 'f [Hz]'), el('th.r', {}, '/rev'), el('th.r', {}, 'ζ'))),
    el('tbody', {}, ...r.modes.map((m) => el('tr', {},
      el('td', {}, m.index), el('td.l', {}, m.label), el('td.r', {}, fmt(m.freq_hz)),
      el('td.r', {}, per ? fmt(m.freq_hz / per, 3) : '–'), el('td.r', {}, fmt(m.damping_ratio, 2))))));
}

export function renderHarmonic(box, r) {
  box.replaceChildren();
  const h = r?.harmonic;
  if (!h) { box.append(el('p.empty', {}, 'No harmonic result.')); return; }
  box.append(el('h3', {}, 'Per-blade 1/rev amplitudes'), el('table.t', {},
    el('thead', {}, el('tr', {}, el('th', {}, 'blade'), el('th.r', {}, 'pitch °'), el('th.r', {}, '∠'),
      el('th.r', {}, 'lag °'), el('th.r', {}, '∠'), el('th.r', {}, 'flap °'), el('th.r', {}, '∠'))),
    el('tbody', {}, ...h.blades.map((b) => el('tr', {}, el('td', {}, b.blade),
      el('td.r', {}, fmt(b.pitch_amp_deg, 3)), el('td.r', {}, fmt(b.pitch_phase_deg, 3)),
      el('td.r', {}, fmt(b.lag_amp_deg, 3)), el('td.r', {}, fmt(b.lag_phase_deg, 3)),
      el('td.r', {}, fmt(b.flap_amp_deg, 3)), el('td.r', {}, fmt(b.flap_phase_deg, 3)))))));
  const cols = { pitch: '#e9a23b', lag: '#5aa9e6', flap: '#4fb286' };
  const wrap = el('div', { style: 'display:flex;flex-wrap:wrap;gap:8px' });
  for (const b of h.blades) {
    wrap.append(phasorChart([
      { amp: b.pitch_amp_deg, phase_deg: b.pitch_phase_deg, color: cols.pitch, label: 'θ' },
      { amp: b.lag_amp_deg, phase_deg: b.lag_phase_deg, color: cols.lag, label: 'ζ' },
      { amp: b.flap_amp_deg, phase_deg: b.flap_phase_deg, color: cols.flap, label: 'β' },
    ], { size: 190, title: `blade ${b.blade}` }));
  }
  box.append(el('h3', {}, 'Phasors (relative to torque pulse)'), wrap,
    el('div.legend', {}, ...Object.entries(cols).map(([k, c]) => el('span', {}, el('i', { style: `background:${c}` }), k))));
  // time traces over one revolution
  const psi = Array.from({ length: 73 }, (_, i) => i * 5);
  const series = [];
  h.blades.forEach((b, i) => {
    for (const [q, col] of Object.entries(cols)) {
      const A = b[`${q}_amp_deg`], P = b[`${q}_phase_deg`];
      series.push({ x: psi, y: psi.map((p) => A * Math.cos(((p + P) * Math.PI) / 180)), color: col,
        dash: i ? '5 3' : undefined, label: `${q} b${b.blade}` });
    }
  });
  box.append(el('h3', {}, 'Over one revolution'),
    lineChart({ series, xlabel: 'azimuth ψ [deg]', ylabel: 'angle [deg]', xlim: [0, 360], ylim: undefined, height: 220 }),
    legend(series));
  box.append(el('h3', {}, 'Hub'), el('table.t', {}, el('tbody', {},
    ...[['torque amplitude', h.torque_amp, 'N·m'], ['hub speed amplitude', h.hub_speed_amp_rpm, 'rpm'],
      ['hub moment mean', h.hub_moment_mean, 'N·m'], ['moment phase', h.hub_moment_phase_deg, '°'],
      ['2P ripple', h.hub_moment_2p, 'N·m'], ['1P thrust', h.thrust_1p, 'N'],
      ['control ratio', h.control_ratio, ''], ['tilt equivalent', h.tilt_equiv_deg, '°']]
      .map(([k, v, u]) => el('tr', {}, el('td.l', {}, k), el('td.r', {}, `${fmt(v)} ${u}`))))));
}

export function renderJSON(box, r) {
  box.replaceChildren();
  if (!r) { box.append(el('p.empty', {}, 'Nothing yet.')); return; }
  const compact = JSON.parse(JSON.stringify(r));
  if (compact.harmonic) for (const k of ['nodes', 'displacement_re', 'displacement_im']) {
    if (compact.harmonic[k]) compact.harmonic[k] = `[${compact.harmonic[k].length} rows omitted]`;
  }
  for (const m of compact.modes || []) if (m.shape) m.shape = `[${m.shape.length} rows omitted]`;
  const text = JSON.stringify(compact, null, 1);
  box.append(el('div', { style: 'display:flex;gap:6px;margin-bottom:6px' },
    el('button.small', { on: { click: () => navigator.clipboard.writeText(JSON.stringify(r)) } }, 'copy full'),
    el('button.small', { on: { click: () => navigator.clipboard.writeText(text) } }, 'copy compact')),
  el('pre.json', {}, text));
}
