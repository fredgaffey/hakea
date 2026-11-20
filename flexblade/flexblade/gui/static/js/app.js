// flexblade GUI entry point: state, wiring between panels, viewport and server.
//
// URL parameters: ?preset=NAME  &set=path=value (repeatable)  &eval=1 (evaluate on load)  &tab=campbell  &theme=light|dark

import { $, $$, el, clone, setPath, diff, debounce, fmt, toast, download, utilColour } from './util.js';
import { api } from './api.js';
import { initParams, renderParams, markChanged } from './params.js';
import { renderSummary, renderChecks, renderCampbell, renderHarmonic, renderJSON } from './results.js';
import { OptimisePanel } from './optimize.js';

const S = {
  info: null, schema: null, presets: {}, materials: [], hints: null,
  design: null, base: null, baseName: null,
  results: null, resultsHash: null, geometryOk: false, viewer: null, evaluating: false,
};
const qs = new URLSearchParams(location.search);

function status(msg, kind = '') {
  const s = $('#status');
  s.textContent = msg;
  s.className = `status ${kind}`;
}

// ------------------------------------------------------------------ theme

function setTheme(t) {
  document.documentElement.dataset.theme = t;
  try { localStorage.setItem('flexblade-theme', t); } catch { /* private mode */ }
  S.viewer?.setTheme(t === 'dark');
}
function initTheme() {
  let t = qs.get('theme');
  if (!t) { try { t = localStorage.getItem('flexblade-theme'); } catch { /* ignore */ } }
  if (!t) t = matchMedia('(prefers-color-scheme: light)').matches ? 'light' : 'dark';
  setTheme(t);
  $('#btn-theme').onclick = () => setTheme(document.documentElement.dataset.theme === 'dark' ? 'light' : 'dark');
}

// ------------------------------------------------------------------ tabs

function showTab(name) {
  $$('#tabs button').forEach((b) => b.classList.toggle('on', b.dataset.tab === name));
  $$('.tab-body > section').forEach((s) => s.classList.toggle('on', s.dataset.panel === name));
}

// ------------------------------------------------------------------ design state

const pushSession = debounce(() => api.putSession(S.design).catch(() => {}), 500);
const refreshGeometry = debounce(() => loadGeometry(), 400);
const liveEval = debounce(() => evaluate(), 700);
const validateSoon = debounce(async () => {
  const v = await api.validate(S.design).catch((e) => ({ ok: false, error: e.message }));
  if (!v.ok) status(`invalid: ${v.errors ? v.errors.map((e) => `${e.loc}: ${e.msg}`).join('; ') : v.error}`, 'err');
  else if (!S.evaluating) status(S.results && S.resultsHash === v.design_hash ? 'up to date' : 'modified — evaluate');
  S.validHash = v.ok ? v.design_hash : null;
}, 250);

function setDesign(design, { base = undefined, baseName = undefined } = {}) {
  S.design = clone(design);
  if (base !== undefined) { S.base = clone(base); S.baseName = baseName; }
  $('#design-name').value = S.design.name || '';
  renderParams(S.design, S.base);
  renderDiff();
  refreshGeometry();
  pushSession();
  validateSoon();
}

function onParamChange(path, value, opts = {}) {
  setPath(S.design, path, value);
  if (opts.rerender || path === 'layout') renderParams(S.design, S.base);
  markChanged(S.design, S.base);
  renderDiff();
  refreshGeometry();
  pushSession();
  validateSoon();
  if ($('#live').checked) liveEval();
}

function renderDiff() {
  const box = $('#tab-diff');
  box.replaceChildren();
  if (!S.base) { box.append(el('p.empty', {}, 'No preset loaded to compare against.')); return; }
  const d = diff(S.base, S.design);
  box.append(el('h3', {}, `vs preset ${S.baseName}`));
  if (!d.length) { box.append(el('p.empty', {}, 'Identical to preset.')); return; }
  box.append(el('table.t', {}, el('thead', {}, el('tr', {}, el('th', {}, 'path'), el('th.r', {}, 'preset'), el('th.r', {}, 'now'))),
    el('tbody', {}, ...d.map((c) => el('tr', {}, el('td.l.mono', {}, c.path),
      el('td.r', {}, typeof c.a === 'number' ? fmt(c.a) : JSON.stringify(c.a)),
      el('td.r', {}, typeof c.b === 'number' ? fmt(c.b) : JSON.stringify(c.b)))))),
  el('div.opt-ctl', {}, el('button.small', { on: { click: () => setDesign(S.base) } }, 'revert to preset')));
}

// ------------------------------------------------------------------ geometry / viewer

let viewerPromise = null;
function ensureViewer() {
  viewerPromise ??= (async () => {
    try {
      const { Viewer } = await import('./viewer.js');
      S.viewer = new Viewer($('#viewport'));
      S.viewer.setTheme(document.documentElement.dataset.theme === 'dark');
      S.viewer.partColour = partUtilColour;
      wireViewerControls();
    } catch (e) {
      $('#vp-msg').textContent = `3D viewer unavailable (${e.message}). three.js loads from cdn.jsdelivr.net.`;
    }
    return S.viewer;
  })();
  return viewerPromise;
}

/** Harmonic fields per rotor slot for the viewer. */
function harmonicsOf(r) {
  if (!r) return {};
  if (r.coaxial) return { upper: r.coaxial.upper?.harmonic, lower: r.coaxial.lower?.harmonic };
  return { main: r.harmonic };
}
const hasNodes = (r) => Object.values(harmonicsOf(r)).some((h) => h?.nodes);

function partUtilColour(part) {
  const rotor = part.rotor || ((part.name || '').match(/^(upper|lower)/i) || [])[1]?.toLowerCase();
  const sub = S.results?.coaxial && rotor ? S.results.coaxial[rotor] : S.results;
  const checks = sub?.checks || [];
  const name = (part.name || '').toLowerCase().replace(/^(upper|lower)_/, ''), role = (part.role || '').toLowerCase();
  let u = null;
  for (const c of checks) {
    const m = (c.member || '').toLowerCase();
    const hit = m === name || name.includes(m) || (m.includes(role) && role !== 'other') || ((role === 'flexure' || role === 'member') && m.startsWith('flex'));
    if (hit) u = Math.max(u ?? 0, c.utilisation);
  }
  return utilColour(u);
}

let geoSeq = 0;
function vpMsg(text, err = false) {
  const m = $('#vp-msg');
  m.textContent = text;
  m.className = `vp-msg${text ? (err ? ' err' : ' info') : ''}`;
}

async function loadGeometry() {
  const v = await ensureViewer();
  if (!v) return;
  const seq = ++geoSeq;
  vpMsg('updating geometry…');
  try {
    const geo = await api.geometry(S.design);
    if (seq !== geoSeq) return; // a newer request superseded this one
    v.setGeometry(geo, S.design.blade?.n_blades || 2);
    v.setHarmonics(harmonicsOf(S.results));
    S.updExag?.();
    S.geometryOk = true;
    vpMsg('');
    renderLegend(geo);
  } catch (e) {
    if (seq !== geoSeq) return;
    // keep whatever is on screen; only fall back to the beam model if nothing is shown yet
    if (!v.parts.length && hasNodes(S.results)) {
      v.setBeamOnly(harmonicsOf(S.results), S.design.blade?.n_blades || 2);
      vpMsg(`geometry unavailable (${e.message}); showing beam model`, true);
    } else vpMsg(`geometry error: ${e.message}${v.parts.length ? ' (showing previous geometry)' : ''}`, true);
  }
}

const MAT_NAMES = {
  tpu_95a: 'TPU 95A', tpu_64d: 'TPU 64D', pa6_cf: 'PA6-CF', petg_cf: 'PETG-CF', pla: 'PLA',
  cf_rod_pultruded: 'CF rod', steel_302_spring: 'steel 302',
};

function renderLegend(geo) {
  const box = $('#vp-legend');
  box.replaceChildren();
  if (geo) S.lastGeo = geo; else geo = S.lastGeo;
  if ($('#colour-by').value === 'util') {
    box.append(el('div.lg-title', {}, 'utilisation (value / allowable)'),
      el('div.lg-grad', { style: `background:linear-gradient(90deg,${[0, 0.25, 0.5, 0.75, 1].map((u) => utilColour(u)).join(',')})` }),
      el('div.lg-ticks', {}, el('span', {}, '0'), el('span', {}, '0.5'), el('span', {}, '1.0')),
      el('div', {}, el('span.sw', { style: `background:${utilColour(1.5)}` }), 'over limit (> 1)'),
      el('div', {}, el('span.sw', { style: `background:${utilColour(null)}` }), 'no check'));
    return;
  }
  const seen = new Map();
  for (const p of geo?.parts || []) if (!seen.has(p.material || p.role)) seen.set(p.material || p.role, p.color_base || p.color);
  box.append(el('div.lg-title', {}, 'material'));
  for (const [k, c] of seen) box.append(el('div', {}, el('span.sw', { style: `background:${c}` }), MAT_NAMES[k] || k || '?'));
  box.append(el('div.lg-note', {}, 'hub darker, fittings lighter'));
}

function wireViewerControls() {
  const v = S.viewer;
  const exag = $('#exag'), speed = $('#speed');
  const upd = () => {
    v.exag = Number(exag.value);
    v.speed = Number(speed.value);
    $('#exag-out').textContent = `×${fmt(v.exag * v.autoScale, 3)}`;
    $('#speed-out').textContent = `${fmt(v.speed, 2)} rev/s`;
  };
  exag.oninput = upd;
  speed.oninput = upd;
  upd();
  S.updExag = upd;
  $('#rotating').onchange = (e) => (v.rotating = e.target.checked);
  $('#show-nodes').onchange = (e) => (v.showNodes = e.target.checked);
  $('#see-through').onchange = (e) => v.setSeeThrough(e.target.checked);
  $('#colour-by').onchange = (e) => { v.colourBy = e.target.value; v.recolour(); renderLegend(S.lastGeo); };
  $('#btn-reset-view').onclick = () => v.resetView();
  v.rotating = $('#rotating').checked;
  v.showNodes = $('#show-nodes').checked;
  v.setSeeThrough($('#see-through').checked);
  v.colourBy = $('#colour-by').value;
}

// ------------------------------------------------------------------ evaluate

async function evaluate() {
  if (S.evaluating) return;
  S.evaluating = true;
  $('#btn-eval').disabled = true;
  status('evaluating…', 'busy');
  const t0 = performance.now();
  try {
    const r = await api.evaluate(S.design);
    showResults(r);
    status(`evaluated in ${fmt((performance.now() - t0) / 1000, 3)} s`);
  } catch (e) {
    status(`error: ${e.message}`, 'err');
    toast(e.message, true, 7000);
  } finally {
    S.evaluating = false;
    $('#btn-eval').disabled = false;
  }
}

function rotorPicker(box) {
  if (!S.results?.coaxial) return;
  box.prepend(el('div.rotor-pick', {}, 'rotor',
    ...['upper', 'lower'].map((k) => el(`button.small${S.rotorView === k ? '.on' : ''}`, {
      on: { click: () => { S.rotorView = k; renderRotorTabs(); } } }, k))));
}

function renderRotorTabs() {
  const r = S.results;
  const sub = r.coaxial ? r.coaxial[S.rotorView] : r;
  renderChecks($('#tab-checks'), sub);
  renderCampbell($('#tab-campbell'), sub);
  renderHarmonic($('#tab-harmonic'), sub);
  for (const t of ['checks', 'campbell', 'harmonic']) rotorPicker($(`#tab-${t}`));
}

function showResults(r) {
  S.results = r;
  S.resultsHash = r.design_hash;
  S.rotorView ??= 'upper';
  renderSummary($('#tab-summary'), r);
  renderRotorTabs();
  renderJSON($('#tab-json'), r);
  const allChecks = r.coaxial ? [...(r.coaxial.upper?.checks || []), ...(r.coaxial.lower?.checks || [])] : (r.checks || []);
  const nfail = allChecks.filter((c) => c.utilisation > 1).length;
  $('#tabs [data-tab=checks]').textContent = nfail ? `Checks (${nfail}✕)` : 'Checks';
  if (S.viewer) {
    if (!S.geometryOk && hasNodes(r)) loadGeometry();
    else S.viewer.setHarmonics(harmonicsOf(r));
    S.viewer.recolour();
    S.updExag?.();
  }
}

// ------------------------------------------------------------------ io

function wireIO() {
  $('#btn-eval').onclick = evaluate;
  document.addEventListener('keydown', (e) => { if ((e.ctrlKey || e.metaKey) && e.key === 'Enter') evaluate(); });
  $('#btn-save').onclick = () => download(`${S.design.name || 'design'}.json`, JSON.stringify(S.design, null, 2));
  $('#btn-load').onclick = () => $('#file-load').click();
  $('#file-load').onchange = async (e) => {
    const f = e.target.files[0];
    if (!f) return;
    try {
      const v = await api.validate(JSON.parse(await f.text()));
      if (!v.ok) throw new Error(v.errors ? v.errors.map((x) => `${x.loc}: ${x.msg}`).join('; ') : v.error);
      setDesign(v.design);
      toast(`loaded ${f.name}`);
    } catch (err) { toast(`load failed: ${err.message}`, true, 7000); }
    e.target.value = '';
  };
  $$('.export button').forEach((b) => (b.onclick = async () => {
    const fmtName = b.dataset.fmt;
    status(`exporting ${fmtName}…`, 'busy');
    try {
      download(`${S.design.name || 'flexblade'}.${fmtName}`, await api.export(S.design, fmtName));
      status(`exported ${fmtName}`);
    } catch (e) { status(`export failed: ${e.message}`, 'err'); toast(e.message, true, 6000); }
  }));
  $('#preset').onchange = (e) => {
    const name = e.target.value;
    if (S.presets[name]) setDesign(S.presets[name], { base: S.presets[name], baseName: name });
  };
  $('#design-name').onchange = (e) => onParamChange('name', e.target.value);
  $$('#tabs button').forEach((b) => (b.onclick = () => showTab(b.dataset.tab)));
}

// ------------------------------------------------------------------ boot

async function boot() {
  initTheme();
  status('loading…', 'busy');
  const [info, schema, pres, mats, hints] = await Promise.all([
    api.info(), api.schema(), api.presets(), api.materials(), api.hints()]);
  Object.assign(S, { info, schema: schema.Design, presets: pres.presets || {}, materials: mats.materials, hints });
  $('#ver').textContent = `v${info.model_version}${info.fake_backend ? ' · FAKE backend' : ''}`;
  initParams($('#params'), { schema: S.schema, hints, materials: S.materials, change: onParamChange });
  const sel = $('#preset');
  const names = Object.keys(S.presets);
  sel.replaceChildren(...(names.length ? names : ['(no presets)']).map((n) => el('option', { value: n }, n)));
  if (pres.error) toast(`presets unavailable: ${pres.error}`, true, 6000);
  wireIO();
  S.optimise = new OptimisePanel($('#tab-optimise'), {
    getDesign: () => S.design, getResults: () => S.results, suggestions: hints.target_suggestions,
    apply: (design, results) => { setDesign(design); if (results) showResults(results); toast('applied optimised design'); },
  });
  const want = qs.get('preset');
  const start = want && S.presets[want] ? want : names[0];
  if (start) {
    sel.value = start;
    setDesign(S.presets[start], { base: S.presets[start], baseName: start });
  } else {
    const v = await api.validate({});
    setDesign(v.design, { base: v.design, baseName: 'defaults' });
  }
  for (const kv of qs.getAll('set')) { // ?set=layout=coaxial&set=flexure.thickness=0.001
    const i = kv.indexOf('=');
    if (i < 0) continue;
    let v = kv.slice(i + 1);
    try { v = JSON.parse(v); } catch { /* keep string */ }
    setPath(S.design, kv.slice(0, i), v);
  }
  if (qs.getAll('set').length) setDesign(S.design);
  status('ready');
  if (qs.get('tab')) showTab(qs.get('tab'));
  if (qs.get('eval') === '1') await evaluate();
  S.setDesign = setDesign;
  S.showResults = showResults;
  window.flexblade = S; // for debugging / automation
  document.body.dataset.ready = '1';
}

boot().catch((e) => {
  console.error(e);
  status(`failed to start: ${e.message}`, 'err');
});
