// three.js viewport: rotor geometry, spin animation and exaggerated 1/rev deformation.
//
// Deformation: harmonic.nodes (frame H, m) with complex 6-DOF displacements
// [ux,uy,uz,rx,ry,rz]; x(t) = Re{X e^{iψ}}, ψ = |Ω|t. Each surface vertex follows its beam:
// blade parts interpolate linearly between the two bracketing nodes of that blade (by radial
// coordinate along the blade axis); u_v = u_n + θ_n × (p − x_n). Hub parts follow the node
// nearest the axis. If a geometry part carries `node_ids` + `node_weights` (per vertex) those
// are used instead.
//
// Coaxial: parts named upper_* / lower_* (or with `rotor`) go to separate rotor slots, each with
// its own harmonic field (Results.coaxial.upper/lower.harmonic, nodes assumed in frame H incl.
// the lower rotor's z offset). The lower slot spins −Ω.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const MM = 1000; // scene units are mm

class RotorSlot {
  constructor(name, sign, parent) {
    this.name = name;
    this.sign = sign;
    this.group = new THREE.Group();
    parent.add(this.group);
    this.parts = [];
    this.harm = null;
    this.nodes = null; // harmonic nodes, possibly z-shifted to match geometry
    this.nodePts = null;
    this.beamLines = null;
    this.nBlades = 2;
  }

  clear() {
    for (const p of this.parts) { p.mesh.geometry.dispose(); p.mesh.material.dispose(); this.group.remove(p.mesh); }
    this.parts = [];
    for (const k of ['nodePts', 'beamLines']) {
      if (this[k]) { this.group.remove(this[k]); this[k].geometry.dispose(); this[k] = null; }
    }
  }

  addPart(part) {
    const pos = new Float32Array(part.vertices.flat());
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(pos.slice(), 3));
    g.setIndex(part.faces.flat());
    g.computeVertexNormals();
    const mat = new THREE.MeshStandardMaterial({ color: part.color || '#a070dc', metalness: 0.05, roughness: 0.6, side: THREE.DoubleSide });
    mat.userData.baseOpacity = part.role === 'skin' || part.role === 'member_skin' ? 0.5 : 1;
    const mesh = new THREE.Mesh(g, mat);
    mesh.userData.part = part;
    this.group.add(mesh);
    this.parts.push({ part, mesh, base: pos, map: null });
  }

  bladeOf(p, rmin) {
    const r = Math.hypot(p[0], p[1]);
    if (r < rmin) return 0;
    const n = this.nBlades;
    let k = Math.round((Math.atan2(p[1], p[0]) / (2 * Math.PI)) * n);
    k = ((k % n) + n) % n;
    return k + 1;
  }

  groupNodes(nodes) {
    const rs = nodes.map((p) => Math.hypot(p[0], p[1]));
    const rmin = Math.max(...rs) * 0.02;
    const groups = Array.from({ length: this.nBlades + 1 }, () => []);
    nodes.forEach((p, i) => groups[this.bladeOf(p, rmin)].push(i));
    for (let k = 1; k <= this.nBlades; k++) groups[k].sort((a, b) => rs[a] - rs[b]);
    if (!groups[0].length && nodes.length) groups[0].push(rs.indexOf(Math.min(...rs)));
    return groups;
  }

  setHarmonic(h) {
    this.harm = h && h.nodes && h.displacement_re ? h : null;
    this.nodes = null;
    if (this.harm) {
      // align node z with geometry z if they disagree (e.g. lower rotor reported at z = 0)
      let nodes = h.nodes.map((p) => [...p]);
      if (this.parts.length) {
        let gz = 0, gn = 0;
        for (const P of this.parts) for (let v = 2; v < P.base.length; v += 3) { gz += P.base[v]; gn++; }
        gz /= gn || 1;
        const nz = nodes.reduce((a, p) => a + p[2], 0) / nodes.length;
        const R = Math.max(...nodes.map((p) => Math.hypot(p[0], p[1])));
        if (Math.abs(gz - nz) > 0.2 * R) nodes = nodes.map((p) => [p[0], p[1], p[2] + gz - nz]);
      }
      this.nodes = nodes;
    }
    this.remap();
    this.buildNodePoints();
  }

  maxDisp() {
    const h = this.harm;
    if (!h) return { u: 0, R: 0 };
    let u = 0;
    for (const row of [...h.displacement_re, ...h.displacement_im]) u = Math.max(u, Math.hypot(row[0], row[1], row[2]));
    return { u, R: Math.max(...h.nodes.map((p) => Math.hypot(p[0], p[1]))) };
  }

  remap() {
    if (!this.harm) { for (const p of this.parts) p.map = null; return; }
    const N = this.nodes;
    const groups = this.groupNodes(N);
    for (const P of this.parts) {
      const part = P.part, nv = P.base.length / 3;
      const map = { i0: new Int32Array(nv), i1: new Int32Array(nv), w: new Float32Array(nv) };
      if (part.node_ids && part.node_weights) {
        for (let v = 0; v < nv; v++) {
          map.i0[v] = part.node_ids[v][0]; map.i1[v] = part.node_ids[v][1] ?? part.node_ids[v][0];
          map.w[v] = part.node_weights[v][1] ?? 0;
        }
        P.map = map;
        continue;
      }
      let blade = part.role === 'hub' ? 0 : part.blade;
      if (blade === null || blade === undefined) {
        let cx = 0, cy = 0;
        for (let v = 0; v < nv; v++) { cx += P.base[3 * v]; cy += P.base[3 * v + 1]; }
        blade = this.bladeOf([cx / nv, cy / nv, 0], 1e-4);
      }
      const list = groups[blade]?.length ? groups[blade] : groups[0];
      const ang = (2 * Math.PI * (Math.max(blade, 1) - 1)) / this.nBlades;
      const ex = Math.cos(ang), ey = Math.sin(ang);
      const sN = list.map((i) => N[i][0] * ex + N[i][1] * ey);
      for (let v = 0; v < nv; v++) {
        if (blade === 0 || list.length === 1) { map.i0[v] = map.i1[v] = list[0]; map.w[v] = 0; continue; }
        const s = P.base[3 * v] * ex + P.base[3 * v + 1] * ey;
        let j = 0;
        while (j < list.length - 2 && sN[j + 1] < s) j++;
        const t = Math.max(0, Math.min(1, (s - sN[j]) / ((sN[j + 1] - sN[j]) || 1)));
        map.i0[v] = list[j]; map.i1[v] = list[j + 1]; map.w[v] = t;
      }
      P.map = map;
    }
  }

  buildNodePoints() {
    if (this.nodePts) { this.group.remove(this.nodePts); this.nodePts.geometry.dispose(); this.nodePts = null; }
    if (!this.harm) return;
    const pos = new Float32Array(this.nodes.flat());
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(pos.slice(), 3));
    this.nodePts = new THREE.Points(g, new THREE.PointsMaterial({ color: 0x5aa9e6, size: 5, sizeAttenuation: false, depthTest: false }));
    this.nodePts.userData.base = pos;
    this.nodePts.renderOrder = 2;
    this.group.add(this.nodePts);
  }

  beamOnly(h) {
    this.clear();
    const pos = new Float32Array(h.nodes.flat());
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(pos.slice(), 3));
    const idx = [];
    const byBlade = this.groupNodes(h.nodes);
    for (const list of byBlade.slice(1)) {
      const chain = [byBlade[0][0], ...list].filter((x) => x !== undefined);
      for (let i = 0; i + 1 < chain.length; i++) idx.push(chain[i], chain[i + 1]);
    }
    g.setIndex(idx);
    this.beamLines = new THREE.LineSegments(g, new THREE.LineBasicMaterial({ color: 0xe9a23b }));
    this.beamLines.userData.base = pos;
    this.group.add(this.beamLines);
  }

  deform(psi, k) {
    const h = this.harm;
    if (!h) return;
    const N = this.nodes, n = N.length, c = Math.cos(psi), s = Math.sin(psi);
    const U = new Float32Array(6 * n);
    for (let i = 0; i < n; i++) {
      const re = h.displacement_re[i], im = h.displacement_im[i];
      for (let d = 0; d < 6; d++) U[6 * i + d] = k * (re[d] * c - im[d] * s);
    }
    const disp = (i, x, y, z, o) => {
      const dx = x - N[i][0], dy = y - N[i][1], dz = z - N[i][2];
      const rx = U[6 * i + 3], ry = U[6 * i + 4], rz = U[6 * i + 5];
      o[0] = U[6 * i] + ry * dz - rz * dy;
      o[1] = U[6 * i + 1] + rz * dx - rx * dz;
      o[2] = U[6 * i + 2] + rx * dy - ry * dx;
    };
    const a = [0, 0, 0], b = [0, 0, 0];
    for (const P of this.parts) {
      if (!P.map) continue;
      const attr = P.mesh.geometry.attributes.position, arr = attr.array, base = P.base;
      const { i0, i1, w } = P.map;
      for (let v = 0; v < i0.length; v++) {
        const x = base[3 * v], y = base[3 * v + 1], z = base[3 * v + 2];
        disp(i0[v], x, y, z, a);
        const t = w[v];
        if (t > 0 && i1[v] !== i0[v]) {
          disp(i1[v], x, y, z, b);
          arr[3 * v] = x + a[0] * (1 - t) + b[0] * t;
          arr[3 * v + 1] = y + a[1] * (1 - t) + b[1] * t;
          arr[3 * v + 2] = z + a[2] * (1 - t) + b[2] * t;
        } else {
          arr[3 * v] = x + a[0]; arr[3 * v + 1] = y + a[1]; arr[3 * v + 2] = z + a[2];
        }
      }
      attr.needsUpdate = true;
    }
    for (const obj of [this.nodePts, this.beamLines]) {
      if (!obj) continue;
      const arr = obj.geometry.attributes.position.array, base = obj.userData.base;
      for (let i = 0; i < n; i++) for (let d = 0; d < 3; d++) arr[3 * i + d] = base[3 * i + d] + U[6 * i + d];
      obj.geometry.attributes.position.needsUpdate = true;
    }
  }
}

export class Viewer {
  constructor(container) {
    this.container = container;
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, preserveDrawingBuffer: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    container.append(this.renderer.domElement);
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(35, 1, 0.1, 10000);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(110, -150, 90);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    // hemisphere (sky/ground) for ambient shape cues + key and fill directional lights
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x7a828c, 1.9));
    const key = new THREE.DirectionalLight(0xffffff, 2.2);
    key.position.set(100, -80, 200);
    this.scene.add(key);
    const fill = new THREE.DirectionalLight(0xffffff, 0.9);
    fill.position.set(-120, 140, 60);
    this.scene.add(fill);
    this.setTheme(true);
    this.root = new THREE.Group();
    this.root.scale.setScalar(MM);
    this.scene.add(this.root);
    this.slots = {};
    this.psi = 0;
    this.speed = 0.5;       // visual rev/s
    this.exag = 1;          // multiplies auto scale
    this.autoScale = 1;
    this.rotating = true;
    this.showNodes = false;
    this.seeThrough = false;
    this.fitted = false;
    this.hover = null;
    this.initHover(container);
    this.colourBy = 'material';
    this.partColour = null; // (part) => css colour, for utilisation mode
    this.last = performance.now();
    new ResizeObserver(() => this.resize()).observe(container);
    this.resize();
    this.renderer.setAnimationLoop(() => this.frame());
  }

  setTheme(dark) {
    const z = this.grid ? this.grid.position.z : -12;
    if (this.grid) this.scene.remove(this.grid);
    this.grid = new THREE.GridHelper(400, 40, 0x9aa2ab, 0xc2c8ce);
    this.grid.rotation.x = Math.PI / 2;
    this.grid.position.z = z;
    this.scene.add(this.grid);
  }

  resize() {
    const w = this.container.clientWidth || 1, h = this.container.clientHeight || 1;
    this.renderer.setSize(w, h, false);
    this.renderer.domElement.style.width = '100%';
    this.renderer.domElement.style.height = '100%';
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  slot(name) {
    if (!this.slots[name]) this.slots[name] = new RotorSlot(name, name === 'lower' ? -1 : 1, this.root);
    return this.slots[name];
  }

  get parts() { return Object.values(this.slots).flatMap((s) => s.parts); }

  clear() { for (const s of Object.values(this.slots)) s.clear(); }

  static rotorOf(part) {
    if (part.rotor) return part.rotor;
    const n = (part.name || '').toLowerCase();
    return n.startsWith('upper') ? 'upper' : n.startsWith('lower') ? 'lower' : 'main';
  }

  /** geometry: {parts:[{name, role, blade, rotor?, material, color, vertices, faces}]} in metres. */
  setGeometry(geo, nBlades) {
    this.clear(); // synchronous swap: the new meshes are added in the same tick, so nothing blinks
    const hasLower = geo.parts.some((p) => Viewer.rotorOf(p) === 'lower');
    for (const part of geo.parts) {
      let r = Viewer.rotorOf(part);
      if (!hasLower) r = 'main';
      else if (r === 'main') r = 'upper';
      const s = this.slot(r);
      s.nBlades = nBlades || 2;
      s.addPart(part);
    }
    const R = geo.radius || Math.max(...geo.parts.flatMap((p) => p.vertices.map((v) => Math.hypot(v[0], v[1]))));
    if (!this.fitted || Math.abs(R - (this.fitR || R)) > 0.2 * R) this.fit();   // auto-frame on first / very different rotor
    this.fitR = R;
    this.fitted = true;
    this.recolour();
    this.applyOpacity();
  }

  /** harmonics: {main: h} or {upper: h, lower: h}. */
  setHarmonics(hs) {
    let best = { u: 0, R: 0 };
    for (const [name, s] of Object.entries(this.slots)) {
      const h = hs[name] ?? (name === 'upper' ? hs.main : null) ?? (name === 'main' ? hs.upper : null);
      s.setHarmonic(h || null);
      const m = s.maxDisp();
      if (m.u > best.u) best = m;
    }
    this.autoScale = best.u > 0 ? (0.08 * best.R) / best.u : 1;
  }

  /** Fallback when no surface geometry is available: beam lines through harmonic nodes. */
  setBeamOnly(hs, nBlades) {
    this.clear();
    for (const [name, h] of Object.entries(hs)) {
      if (!h?.nodes) continue;
      const s = this.slot(name);
      s.nBlades = nBlades || 2;
      s.beamOnly(h);
    }
    this.setHarmonics(hs);
    this.fit(Object.values(hs).flatMap((h) => h?.nodes || []));
  }

  fit(points) {
    const box = new THREE.Box3();
    if (points) for (const p of points) box.expandByPoint(new THREE.Vector3(p[0] * MM, p[1] * MM, p[2] * MM));
    else for (const P of this.parts) for (let v = 0; v < P.base.length; v += 3) {
      box.expandByPoint(new THREE.Vector3(P.base[v] * MM, P.base[v + 1] * MM, P.base[v + 2] * MM));
    }
    if (box.isEmpty()) return;
    if (!points) { // the rotor sweeps a disk: frame the full radius
      const R = Math.max(Math.abs(box.min.x), box.max.x, Math.abs(box.min.y), box.max.y);
      box.expandByPoint(new THREE.Vector3(-R, -R, box.min.z)); box.expandByPoint(new THREE.Vector3(R, R, box.max.z));
    }
    const size = box.getSize(new THREE.Vector3()).length();
    const c = box.getCenter(new THREE.Vector3());
    c.x = 0; c.y = 0;
    this.controls.target.copy(c);
    this.camera.position.set(c.x + size * 0.34, c.y - size * 0.8, c.z + size * 1.0);
    this.camera.near = size / 1000;
    this.camera.far = size * 50;
    this.camera.updateProjectionMatrix();
    this.grid.position.z = box.min.z - size * 0.08;
  }

  resetView() { this.fit(); }

  recolour() {
    for (const P of this.parts) {
      const c = this.colourBy === 'util' && this.partColour ? this.partColour(P.part) : (P.part.color || '#a070dc');
      P.mesh.material.color.set(c);
    }
  }

  /** See-through: everything except rods (and other stiffeners) goes translucent. */
  applyOpacity() {
    for (const P of this.parts) {
      const m = P.mesh.material, solid = P.part.role === 'rod' || P.part.role === 'wire';
      const o = this.seeThrough && !solid ? 0.22 : m.userData.baseOpacity;
      m.transparent = o < 1; m.opacity = o; m.depthWrite = o >= 0.99;
      m.needsUpdate = true;
    }
  }

  setSeeThrough(on) { this.seeThrough = on; this.applyOpacity(); }

  initHover(container) {
    this.ray = new THREE.Raycaster();
    this.tip = document.createElement('div');
    this.tip.className = 'vp-tip';
    this.tip.hidden = true;
    container.parentElement.append(this.tip);
    const dom = this.renderer.domElement;
    dom.addEventListener('pointermove', (e) => {
      const r = dom.getBoundingClientRect();
      this.pointer = { x: ((e.clientX - r.left) / r.width) * 2 - 1, y: -((e.clientY - r.top) / r.height) * 2 + 1,
        px: e.clientX - r.left, py: e.clientY - r.top };
    });
    dom.addEventListener('pointerleave', () => { this.pointer = null; });
  }

  updateHover() {
    let hit = null;
    if (this.pointer) {
      this.ray.setFromCamera(this.pointer, this.camera);
      const meshes = this.parts.map((P) => P.mesh);
      const hits = this.ray.intersectObjects(meshes, false);
      // when see-through, ignore translucent parts if a solid one is under the cursor too
      hit = (this.seeThrough ? hits.find((h) => h.object.userData.part.role === 'rod') : null) || hits[0] || null;
    }
    const part = hit ? hit.object.userData.part : null;
    if (part !== this.hover) {
      if (this.hover) for (const P of this.parts) if (P.part === this.hover) P.mesh.material.emissive.setHex(0);
      if (part) for (const P of this.parts) if (P.part === part) P.mesh.material.emissive.setHex(0x333333);
      this.hover = part;
    }
    if (part) {
      this.tip.hidden = false;
      this.tip.textContent = `${part.name} · ${part.material || '?'} · ${part.role}`;
      this.tip.style.transform = `translate(${this.pointer.px + 14}px, ${this.pointer.py + 14}px)`;
    } else this.tip.hidden = true;
  }

  frame() {
    const now = performance.now();
    const dt = Math.min(0.1, (now - this.last) / 1000);
    this.last = now;
    this.psi = (this.psi + dt * this.speed * 2 * Math.PI) % (2 * Math.PI);
    for (const s of Object.values(this.slots)) {
      s.group.rotation.z = this.rotating ? s.sign * this.psi : 0;
      if (s.nodePts) s.nodePts.visible = this.showNodes;
      s.deform(this.psi, this.exag * this.autoScale);
    }
    this.updateHover();
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}
