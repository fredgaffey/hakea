# Flexblade ↔ Keel bridge

A small panel inside Keel (Fred's browser CAD, which embeds SolidCraft) that talks to the
local `flexblade gui` server. The local GUI stays the main simulator. The panel can:

- **Insert STEP.** Fetches the flexblade GUI's current design as STEP and inserts it into
  the open Keel document as a new component.
- **Push parameters.** Creates or updates SolidCraft user parameters (`fb_skew`,
  `fb_flex_thickness`, …) from the current design, so sketches and features can reference
  them.
- **Follow design.** Polls the GUI every 2 s and pushes the parameters again whenever the
  design changes.

The "current design" is whatever the flexblade GUI last loaded or edited. Every edit there
is sent to `PUT /api/session/design`, and the panel reads it back with
`GET /api/session/design` and `GET /api/export?fmt=step`.

## Files

| File | Role |
|---|---|
| `flexblade/keel/flexblade-keel.js` | The bridge module and source of truth. It is an ES module with no dependencies and uses a shadow DOM, so Keel's CSS can't leak into it. It exports `install()`, `pushParameters()`, `insertStep()`, `parameterMap()` and `expressionFor()`. |
| `/keel/flexblade-keel.js` on the flexblade server | The same file, served with CORS, for quick tries without changing Keel. |
| Keel: `public/js/flexblade.js` | A copy of the module. Copy it again after you change the source. |

## Install into Keel

This is the change in the Keel PR on branch `flexblade-panel`:

1. Copy `flexblade-keel.js` to Keel `public/js/flexblade.js`.
2. In `public/doc.html`, add a button to the top bar:
   ```html
   <button class="btn hide-narrow" id="flexblade-btn" title="…">Flexblade</button>
   ```
3. In `public/js/editor.js`, inside `wireBar()`:
   ```js
   $('flexblade-btn').hidden = S.role === 'view';
   $('flexblade-btn').onclick = async () => {
     if (!S.cad) return toast('The modeller is still loading', 'error');
     if (S.flexblade) return S.flexblade.toggle();
     const { install } = await import('./flexblade.js');
     S.flexblade = install({ app: S.cad, frame: $('cad'), run: serial });
   };
   ```
   Passing `run: serial` puts the panel's engine calls in Keel's existing queue, so they never
   interleave with Keel's own `document.import` / `document.export` sync. The changes the
   panel makes are then ordinary design edits, which Keel syncs and versions as usual.
4. In `public/sw.js`, add `/js/flexblade.js` to `SHELL_FILES` so it works offline.
5. In `tests/flexblade.test.js`, unit-test the pure helpers (`npm test`).

The module only loads when someone clicks the button, so it costs nothing otherwise.

## Use

On the computer running Keel in the browser:

```bash
flexblade gui                                  # local Keel dev (127.0.0.1:8787) is allowed by default
flexblade gui --cors-origin https://keel.<your-domain>     # deployed Keel
# or: FLEXBLADE_CORS_ORIGINS=https://keel.a,https://keel.b flexblade gui
```

Then open a document in Keel, click **Flexblade**, and check that the server URL is
`http://127.0.0.1:8765`. The URL is kept in localStorage. Then click **Insert STEP** or
**Push parameters**.

To try it without changing Keel, paste this into the devtools console of a Keel document
page:

```js
const m = await import('http://127.0.0.1:8765/keel/flexblade-keel.js');
m.install({ app: document.getElementById('cad').contentWindow.solidcraft, frame: document.getElementById('cad') });
```

### Browser notes

- A deployed Keel page served over https may call `http://127.0.0.1`. Chrome and Edge treat
  loopback as potentially trustworthy, so this is not blocked as mixed content. Chrome's
  Private Network Access preflight is answered: the server sends
  `Access-Control-Allow-Private-Network: true`, but only to allowed origins.
- Safari has historically blocked https→http-loopback requests. If it does, run Keel
  locally, or serve flexblade over https.

## Parameters pushed

| CAD parameter | Design path | Expression |
|---|---|---|
| `fb_diameter` | `blade.diameter_in` | `5 in` |
| `fb_n_blades`, `fb_root_cutout_frac`, `fb_flex_taper` | unitless | `2` |
| `fb_skew`, `fb_cross_angle`, `fb_flex_twist`, `fb_collective` | `*_deg` | `45 deg` |
| `fb_hinge_radius`, `fb_flex_length`, `fb_flex_thickness`, `fb_flex_width`, `fb_hub_radius` | metres | `1.2 mm` |

The panel tries `parameters.add {name, expression, unit, comment}` first. If that is refused
(normally because the parameter already exists), it falls back to one
`parameters.change {changes: {name: expression}}`, which rebuilds once, all or nothing.
If that fails too, it changes each parameter on its own.

## What was verified, and what wasn't

Verified against a mock Keel host and a mock SolidCraft iframe on another origin
(`tests/keel_mock/`, driven by headless Chrome), with the real flexblade server:

- the module loads cross-origin through CORS;
- parameters are added, then changed;
- the real geometry module exports a 1.3 MB STEP, and the panel delivers it as a dropped file;
- follow mode works;
- the `run` queue works.

Keel's `npm test` passes, including the two new tests.

Not verified against real SolidCraft, because there is no SolidCraft build on this laptop:

1. **STEP insertion.** `file.insert_step {path}` reads from SolidCraft's in-memory file
   system. The browser API (`window.solidcraft`) has no call that writes a file there:
   `apps/solidcraft-web/src/files.rs` only fills `/upload/` from a file picker or a drop.
   So the panel imitates a user drag-and-drop. It dispatches `dragenter`, `dragover` and
   `drop` with a `DataTransfer` that holds a `File`, on the SolidCraft canvas, from the
   same-origin parent. SolidCraft's drop path (eframe's web drag-and-drop, then
   `files::take_drops`, then `Intent::Insert`) should take this exactly like a real drop.
   That's an inference from reading the source, not a test, and two things could stop it:
   - eframe might attach its drop listener to a different element from the first `<canvas>`;
   - `take_drops` gets the name from `f.path().file_name()`. If a web drop has no path,
     the name becomes `"dropped"`, and the `.step` extension may be lost.

   The panel checks that `document.revision` moves. If it doesn't, it downloads the file
   and tells you to drag it in by hand (or use SOLID › INSERT › Insert STEP).

   A cleaner long-term route would be a small SolidCraft browser-API addition, for example
   `request('file.write', {path, text})` or `file.insert_step {text, name}`. Whether to
   propose that upstream is Fred's call.
2. **The `parameters.add` / `parameters.change` schemas** were read from
   `crates/engine/src/schema/parameters.rs`, and look right. The exact error text for a
   parameter that already exists was not checked; the code treats any refusal of `add` as
   "exists".
3. **SolidCraft's plugin system** (WASM components, `plugin.install`) was not used. A
   plugin can't add host-page UI or reach `http://127.0.0.1` without file and network
   grants, and the iframe-host route is much simpler for Keel.
4. **Units.** The parameters assume the design's length unit can parse `mm`. Expressions
   carry explicit units, so a design set to inches should still convert.
