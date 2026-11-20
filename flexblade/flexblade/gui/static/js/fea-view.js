// FEA surface-field view: boundary triangles of the solid tet mesh (frame H, metres) coloured by a
// per-face result, optional wireframe (the mesh), deformed shape, hover read-out.
// Data: result of POST /api/fea (see flexblade.fea.surface_field).

import * as THREE from 'three';

const MM = 1000;
// blue -> cyan -> green -> yellow -> red
const RAMP = [[0, [49, 54, 149]], [0.25, [69, 160, 220]], [0.5, [80, 190, 120]], [0.75, [245, 215, 60]], [1, [215, 48, 39]]];

export function rampColour(t) {
  t = Math.max(0, Math.min(1, t));
  let k = 0;
  while (k < RAMP.length - 2 && t > RAMP[k + 1][0]) k++;
  const [t0, c0] = RAMP[k], [t1, c1] = RAMP[k + 1];
  const s = (t - t0) / (t1 - t0);
  return c0.map((a, i) => (a + (c1[i] - a) * s) / 255);
}

export const FEA_FIELDS = {
  utilisation: { label: 'utilisation (max of strain, stress)', unit: '', scale: 1, fixed: [0, 1] },
  util_strain: { label: 'utilisation: strain / creep allowable', unit: '', scale: 1, fixed: [0, 1] },
  util_stress: { label: 'utilisation: stress / strength', unit: '', scale: 1, fixed: [0, 1] },
  von_mises: { label: 'von Mises stress', unit: 'MPa', scale: 1e-6 },
  strain: { label: 'max principal strain', unit: '%', scale: 100 },
  interlayer: { label: 'interlayer (build-dir) normal stress', unit: 'MPa', scale: 1e-6 },
};

export class FeaView {
  constructor(viewer) {
    this.viewer = viewer;
    this.group = new THREE.Group();
    this.group.visible = false;
    viewer.root.add(this.group);
    this.data = null;
    this.field = 'utilisation';
    this.clip = true;
    this.wire = true;
    this.deformed = true;
    this.scale = 1;
    this.opacity = 1;
    this.active = false;
    this.ray = new THREE.Raycaster();
  }

  setActive(on) {
    this.active = on && !!this.data;
    this.group.visible = this.active;
  }

  clear() {
    for (const o of [this.mesh, this.lines]) {
      if (!o) continue;
      this.group.remove(o);
      o.geometry.dispose();
      o.material.dispose();
    }
    this.mesh = this.lines = null;
  }

  setData(d) {
    this.clear();
    this.data = d;
    const nf = d.faces.length;
    this.nf = nf;
    const base = new Float32Array(nf * 9), disp = new Float32Array(nf * 9);
    for (let f = 0; f < nf; f++) {
      for (let c = 0; c < 3; c++) {
        const v = d.faces[f][c];
        for (let k = 0; k < 3; k++) {
          base[9 * f + 3 * c + k] = d.vertices[v][k];
          disp[9 * f + 3 * c + k] = d.disp[v][k];
        }
      }
    }
    this.base = base;
    this.dispArr = disp;
    let um = 0;
    for (const p of d.disp) um = Math.max(um, Math.hypot(p[0], p[1], p[2]));
    this.maxDisp = um;
    let ext = 0;
    for (const p of d.vertices) ext = Math.max(ext, Math.hypot(p[0], p[1], p[2]));
    // auto deformation scale: largest displacement drawn as 8 % of the model extent
    this.autoScale = um > 0 ? Math.max(1, Math.round((0.08 * ext) / um)) : 1;
    this.scale = this.autoScale;
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(base.slice(), 3));
    g.setAttribute('color', new THREE.BufferAttribute(new Float32Array(nf * 9), 3));
    this.mesh = new THREE.Mesh(g, new THREE.MeshStandardMaterial({
      vertexColors: true, side: THREE.DoubleSide, metalness: 0, roughness: 0.8, flatShading: true,
      polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1 }));
    this.group.add(this.mesh);
    const lg = new THREE.BufferGeometry();
    lg.setAttribute('position', g.attributes.position);   // shared buffer: follows deformation
    this.lines = new THREE.Mesh(lg, new THREE.MeshBasicMaterial({
      color: 0x10151a, wireframe: true, transparent: true, opacity: 0.55, depthWrite: false }));
    this.group.add(this.lines);
    this.lines.visible = this.wire;
    this.applyOpacity();
    this.refreshColours();
    this.refreshShape();
  }

  /** Colour range [lo, hi] of the current field in SI units, and the clip value actually used. */
  range() {
    const F = this.data.fields[this.field], spec = FEA_FIELDS[this.field];
    if (spec.fixed) return { lo: spec.fixed[0], hi: spec.fixed[1], max: F.max, min: F.min, clipped: false };
    const lo = Math.min(0, F.min);
    let hi = F.max, clipped = false;
    if (this.clip) {
      const v = Float32Array.from(F.values).sort();
      const p = v[Math.min(v.length - 1, Math.floor(0.99 * v.length))];
      if (p > lo && p < 0.98 * F.max) { hi = p; clipped = true; }
    }
    return { lo, hi: hi > lo ? hi : lo + 1e-12, max: F.max, min: F.min, clipped };
  }

  refreshColours() {
    if (!this.mesh) return;
    const F = this.data.fields[this.field], r = this.range();
    const col = this.mesh.geometry.attributes.color.array;
    for (let f = 0; f < this.nf; f++) {
      const c = rampColour((F.values[f] - r.lo) / (r.hi - r.lo));
      for (let k = 0; k < 3; k++) col.set(c, 9 * f + 3 * k);
    }
    this.mesh.geometry.attributes.color.needsUpdate = true;
    this.rng = r;
  }

  refreshShape() {
    if (!this.mesh) return;
    const pos = this.mesh.geometry.attributes.position, a = pos.array;
    const k = this.deformed ? this.scale : 0;
    for (let i = 0; i < a.length; i++) a[i] = this.base[i] + k * this.dispArr[i];
    pos.needsUpdate = true;
    this.mesh.geometry.computeVertexNormals();
    this.mesh.geometry.computeBoundingSphere();
  }

  applyOpacity() {
    if (!this.mesh) return;
    const m = this.mesh.material, o = this.opacity;
    m.transparent = o < 1; m.opacity = o; m.depthWrite = o >= 0.99; m.needsUpdate = true;
  }

  setSeeThrough(on) { this.opacity = on ? 0.35 : 1; this.applyOpacity(); }
  setWire(on) { this.wire = on; if (this.lines) this.lines.visible = on; }
  setField(f) { this.field = f; this.refreshColours(); }
  setClip(on) { this.clip = on; this.refreshColours(); }
  setDeformed(on) { this.deformed = on; this.refreshShape(); }
  setScale(s) { this.scale = s; this.refreshShape(); }

  fmtValue(face) {
    const spec = FEA_FIELDS[this.field], v = this.data.fields[this.field].values[face] * spec.scale;
    const a = Math.abs(v);
    return `${v.toFixed(a >= 100 ? 0 : a >= 10 ? 1 : a >= 0.1 ? 2 : 3)}${spec.unit ? ' ' + spec.unit : ''}`;
  }

  /** Hover read-out; returns true when it handled the tooltip. */
  updateHover(pointer, camera, tip) {
    if (!this.active || !this.mesh) return false;
    let hit = null;
    if (pointer) {
      this.ray.setFromCamera(pointer, camera);
      hit = this.ray.intersectObject(this.mesh, false)[0] || null;
    }
    if (!hit) { tip.hidden = true; return true; }
    const f = hit.faceIndex, d = this.data, p = d.face_part[f];
    const util = d.fields.utilisation.values[f];
    tip.hidden = false;
    tip.textContent = `${d.part_names[p]} · ${d.part_materials[p]} · ${FEA_FIELDS[this.field].label.split(' (')[0]} ${this.fmtValue(f)}`
      + (this.field === 'utilisation' ? '' : ` · util ${util.toFixed(2)}`);
    tip.style.transform = `translate(${pointer.px + 14}px, ${pointer.py + 14}px)`;
    return true;
  }

  bounds() {
    const box = new THREE.Box3().setFromObject(this.group);
    return box;
  }
}
