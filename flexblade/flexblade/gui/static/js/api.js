// REST client for the flexblade server (same origin).

async function req(method, url, body) {
  const opt = { method, headers: {} };
  if (body !== undefined) {
    opt.headers['Content-Type'] = 'application/json';
    opt.body = JSON.stringify(body);
  }
  const r = await fetch(url, opt);
  if (!r.ok) {
    let detail;
    try { detail = await r.json(); } catch { detail = { error: r.statusText }; }
    const msg = detail.error || detail.detail || r.statusText;
    const extra = detail.errors ? ' — ' + detail.errors.map((e) => `${e.loc}: ${e.msg}`).join('; ') : '';
    const err = new Error(`${msg}${extra}${detail.hint ? ` (${detail.hint})` : ''}`);
    err.status = r.status;
    err.detail = detail;
    throw err;
  }
  return r;
}

const json = async (method, url, body) => (await req(method, url, body)).json();

export const api = {
  info: () => json('GET', '/api/info'),
  schema: () => json('GET', '/api/schema'),
  presets: () => json('GET', '/api/presets'),
  materials: () => json('GET', '/api/materials'),
  hints: () => json('GET', '/api/hints'),
  validate: (design) => json('POST', '/api/validate', { design }),
  evaluate: (design, what) => json('POST', '/api/evaluate', { design, what }),
  geometry: (design) => json('POST', '/api/geometry', { design }),
  optimize: (request) => json('POST', '/api/optimize', request),
  job: (id) => json('GET', `/api/jobs/${id}`),
  putSession: (design) => json('PUT', '/api/session/design', { design }),
  export: async (design, fmt) => {
    const r = await req('POST', `/api/export?fmt=${fmt}`, { design });
    return r.blob();
  },
};
