// Flexblade panel for Keel / SolidCraft (OWNER: flexblade ui).
//
// Runs in the Keel *host page* (same origin as the SolidCraft iframe) and talks to a local
// `flexblade gui` server (default http://127.0.0.1:8765, CORS-allowed for the Keel origin).
//
//   import('http://127.0.0.1:8765/keel/flexblade-keel.js').then((m) => m.install());
//   // or inside Keel: m.install({ app: S.cad, frame: document.getElementById('cad') })
//
// Actions:
//   Insert STEP      GET  {server}/api/export?fmt=step (the GUI's current "session" design),
//                    then hand the file to SolidCraft. SolidCraft's file.insert_step needs a
//                    path in its in-memory file system, which the browser API cannot write, so
//                    we drop the file on the SolidCraft canvas (same as a user drag & drop,
//                    which inserts STEP into a design that has features / opens it otherwise).
//                    If the design revision does not move, we fall back to downloading the
//                    file for a manual drag.
//   Push parameters  GET  {server}/api/session/design, then parameters.add / parameters.change
//                    for a curated set of design values (fb_* names, with units).
//   Follow           poll the session design and push parameters whenever it changes.

const STORE_KEY = 'flexblade.server';
const DEFAULT_SERVER = 'http://127.0.0.1:8765';

/** Design values mirrored into CAD parameters: [dotted path, parameter name, unit]. */
export const PARAMS = [
  ['blade.diameter_in', 'fb_diameter', 'in'],
  ['blade.n_blades', 'fb_n_blades', ''],
  ['blade.root_cutout_frac', 'fb_root_cutout_frac', ''],
  ['flexure.skew_deg', 'fb_skew', 'deg'],
  ['flexure.hinge_radius', 'fb_hinge_radius', 'm'],
  ['flexure.length', 'fb_flex_length', 'm'],
  ['flexure.thickness', 'fb_flex_thickness', 'm'],
  ['flexure.width', 'fb_flex_width', 'm'],
  ['flexure.cross_angle_deg', 'fb_cross_angle', 'deg'],
  ['flexure.taper', 'fb_flex_taper', ''],
  ['flexure.twist_deg', 'fb_flex_twist', 'deg'],
  ['flexure.hub_radius', 'fb_hub_radius', 'm'],
  ['op.collective_deg', 'fb_collective', 'deg'],
];

const get = (o, p) => p.split('.').reduce((x, k) => (x == null ? x : x[k]), o);

/** CAD expression for a value: SI metres become millimetres. */
export function expressionFor(value, unit) {
  if (unit === 'm') return `${+(value * 1000).toPrecision(6)} mm`;
  if (unit === 'in') return `${+value.toPrecision(6)} in`;
  if (unit === 'deg') return `${+value.toPrecision(6)} deg`;
  return String(+value.toPrecision(6));
}

/** {name: {expression, unit, path}} for a Design JSON. */
export function parameterMap(design) {
  const out = {};
  for (const [path, name, unit] of PARAMS) {
    const v = get(design, path);
    if (typeof v === 'number' && Number.isFinite(v)) out[name] = { expression: expressionFor(v, unit), unit: unit === 'm' ? 'mm' : unit, path };
  }
  return out;
}

/** Create or update CAD parameters. Returns {added, changed, errors}. */
export async function pushParameters(app, design) {
  const map = parameterMap(design);
  const added = [], existing = {}, errors = [];
  for (const [name, p] of Object.entries(map)) {
    try {
      await app.execute('parameters.add', { name, expression: p.expression, unit: p.unit, comment: `flexblade ${p.path}` });
      added.push(name);
    } catch (e) {
      existing[name] = p.expression; // most likely "already exists": change it below
    }
  }
  let changed = [];
  if (Object.keys(existing).length) {
    try {
      await app.execute('parameters.change', { changes: existing });
      changed = Object.keys(existing);
    } catch (e) {
      // fall back to one by one so one bad name doesn't block the rest
      for (const [name, expression] of Object.entries(existing)) {
        try { await app.execute('parameters.change', { name, expression }); changed.push(name); }
        catch (e2) { errors.push(`${name}: ${e2.message}`); }
      }
    }
  }
  return { added, changed, errors };
}

/** Hand a STEP file to SolidCraft by a synthetic drop on its canvas. Resolves true if the design changed. */
export async function insertStep(app, frame, text, name = 'flexblade.step') {
  const w = frame.contentWindow, doc = frame.contentDocument;
  const target = doc && (doc.querySelector('canvas') || doc.body);
  if (!target) return false;
  const before = await app.request('document.revision').catch(() => null);
  const dt = new w.DataTransfer();
  dt.items.add(new w.File([text], name, { type: 'model/step' }));
  const rect = target.getBoundingClientRect();
  const at = { clientX: rect.left + rect.width / 2, clientY: rect.top + rect.height / 2, bubbles: true, cancelable: true, dataTransfer: dt };
  for (const type of ['dragenter', 'dragover', 'drop']) target.dispatchEvent(new w.DragEvent(type, at));
  for (let i = 0; i < 40; i++) { // up to ~8 s: STEP import can take a moment
    await new Promise((r) => setTimeout(r, 200));
    const now = await app.request('document.revision').catch(() => null);
    if (now && before && JSON.stringify(now) !== JSON.stringify(before)) return true;
  }
  return false;
}

function saveFile(name, text) {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([text], { type: 'model/step' }));
  a.download = name;
  document.body.append(a);
  a.click();
  setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 1000);
}

const CSS = `
:host { all: initial; }
.p { position: fixed; right: 14px; top: 56px; width: 290px; z-index: 2147483000; font: 12px/1.4 system-ui, sans-serif;
  color: #d8dee6; background: #161a1f; border: 1px solid #2a313a; border-radius: 6px; box-shadow: 0 8px 28px #0007; }
.h { display: flex; align-items: center; gap: 6px; padding: 7px 10px; border-bottom: 1px solid #2a313a; font-weight: 600; }
.h b { color: #e9a23b; } .h .x { margin-left: auto; cursor: pointer; color: #8592a1; background: none; border: 0; font-size: 14px; }
.b { padding: 8px 10px; display: flex; flex-direction: column; gap: 7px; }
input[type=text] { width: 100%; box-sizing: border-box; background: #1c2128; color: inherit; border: 1px solid #38414c; border-radius: 4px; padding: 3px 6px; font: 11px ui-monospace, monospace; }
.row { display: flex; gap: 6px; flex-wrap: wrap; align-items: center; }
button { background: #1c2128; color: inherit; border: 1px solid #38414c; border-radius: 4px; padding: 4px 9px; cursor: pointer; font: inherit; }
button.pri { background: #e9a23b; border-color: #e9a23b; color: #1a1208; font-weight: 600; }
button:disabled { opacity: .5; }
.s { color: #8592a1; font: 11px ui-monospace, monospace; white-space: pre-wrap; max-height: 140px; overflow: auto; }
.s.err { color: #e5534b; } a { color: #e9a23b; }
label { color: #8592a1; display: inline-flex; gap: 4px; align-items: center; }
`;

/**
 * Add the panel. Options: app (window.solidcraft of the frame), frame (the SolidCraft iframe),
 * server (flexblade URL), run (fn => promise: the host's queue for engine work, e.g. Keel's
 * `serial`, so our commands never interleave with its document import/export).
 * Returns {toggle, el}.
 */
export function install(opts = {}) {
  // Run engine work through the host's queue; errors come back as values (a queue may swallow them).
  const engine = async (fn) => {
    const run = opts.run || ((f) => f());
    const r = await run(async () => { try { return { value: await fn() }; } catch (error) { return { error }; } });
    if (!r) throw new Error('engine queue returned nothing');
    if (r.error) throw r.error;
    return r.value;
  };
  const frame = opts.frame || document.getElementById('cad') || document.querySelector('iframe');
  const appOf = () => opts.app || (frame && frame.contentWindow && frame.contentWindow.solidcraft) || window.solidcraft;
  let server = opts.server;
  if (!server) { try { server = localStorage.getItem(STORE_KEY); } catch { /* ignore */ } }
  server = server || DEFAULT_SERVER;

  const host = document.createElement('div');
  host.id = 'flexblade-panel';
  const root = host.attachShadow({ mode: 'open' });
  root.innerHTML = `<style>${CSS}</style>
  <div class="p"><div class="h"><b>◇</b> Flexblade <button class="x" title="close">×</button></div>
  <div class="b">
    <input type="text" class="url" spellcheck="false" title="flexblade gui server">
    <div class="row"><button class="pri ins">Insert STEP</button><button class="par">Push parameters</button></div>
    <div class="row"><label><input type="checkbox" class="fol"> follow design</label><a class="gui" target="_blank">open simulator</a></div>
    <div class="s">…</div>
  </div></div>`;
  document.body.append(host);
  const $ = (s) => root.querySelector(s);
  const url = $('.url'), st = $('.s');
  url.value = server;
  $('.gui').href = server;
  const say = (m, err = false) => { st.textContent = m; st.className = 's' + (err ? ' err' : ''); };
  url.addEventListener('change', () => {
    server = url.value.replace(/\/+$/, '');
    $('.gui').href = server;
    try { localStorage.setItem(STORE_KEY, server); } catch { /* ignore */ }
    ping();
  });

  const fetchJSON = async (path) => {
    const r = await fetch(server + path);
    if (!r.ok) throw new Error(`${path}: ${r.status} ${(await r.text()).slice(0, 200)}`);
    return r.json();
  };

  async function ping() {
    try {
      const info = await fetchJSON('/api/info');
      const d = await fetchJSON('/api/session/design');
      say(`connected · flexblade ${info.model_version}${info.fake_backend ? ' (fake backend)' : ''}\ndesign: ${d.name}`);
      return d;
    } catch (e) {
      say(`can't reach ${server}: ${e.message}\nRun: flexblade gui --cors-origin ${location.origin}`, true);
      return null;
    }
  }

  $('.ins').addEventListener('click', async () => {
    const app = appOf();
    if (!app) return say('SolidCraft API not found (is the modeller loaded?)', true);
    try {
      say('exporting STEP…');
      const r = await fetch(`${server}/api/export?fmt=step`);
      if (!r.ok) throw new Error(`${r.status} ${(await r.text()).slice(0, 300)}`);
      const text = await r.text();
      const d = await fetchJSON('/api/session/design');
      const name = `${(d.name || 'flexblade').replace(/[^\w.-]+/g, '_')}.step`;
      say(`inserting ${name} (${(text.length / 1024).toFixed(0)} kB)…`);
      const ok = frame ? await engine(() => insertStep(app, frame, text, name)) : false;
      if (ok) say(`inserted ${name} as a new component`);
      else { saveFile(name, text); say(`couldn't insert automatically: downloaded ${name}, drag it onto the model (or SOLID › INSERT › Insert STEP).`, true); }
    } catch (e) { say(`insert failed: ${e.message}`, true); }
  });

  let lastHash = null;
  async function push(quiet = false) {
    const app = appOf();
    if (!app) { say('SolidCraft API not found', true); return; }
    try {
      const d = await fetchJSON('/api/session/design');
      const r = await engine(() => pushParameters(app, d));
      lastHash = JSON.stringify(d);
      if (!quiet || r.errors.length) {
        say(`parameters: ${r.added.length} added, ${r.changed.length} updated` + (r.errors.length ? `\n${r.errors.join('\n')}` : ''), !!r.errors.length);
      }
    } catch (e) { say(`push failed: ${e.message}`, true); }
  }
  $('.par').addEventListener('click', () => push(false));

  let timer = null;
  $('.fol').addEventListener('change', (e) => {
    clearInterval(timer);
    if (!e.target.checked) return;
    timer = setInterval(async () => {
      try {
        const d = await fetchJSON('/api/session/design');
        if (JSON.stringify(d) !== lastHash) { await push(true); say(`following · updated from "${d.name}" ${new Date().toLocaleTimeString()}`); }
      } catch { /* server away: keep trying */ }
    }, 2000);
  });

  const toggle = (show) => { host.style.display = (show ?? host.style.display === 'none') ? '' : 'none'; };
  $('.x').addEventListener('click', () => toggle(false));
  ping();
  return { toggle, el: host };
}
