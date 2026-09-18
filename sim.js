/* Google Earth Flight Tracker - visual layer
 *
 * Adds a real aircraft and an MSFS-style glass panel on top of Google Earth's
 * flight simulator, driven by the true 6-DOF state (lat, lon, altitude, heading,
 * pitch, bank) that tracker.js reads out of Earth's WebAssembly heap.
 *
 * Views (cycle with V):
 *   exterior - low-poly aircraft rendered in a transparent WebGL overlay, with a
 *              lagging chase camera, deflecting control surfaces and a spinning prop
 *   cockpit  - windscreen frame + six-pack glass panel (ASI, AI, ALT, TC, HSI, VSI)
 *   off      - Earth's own HUD only
 *
 * Earth's camera already rolls and pitches with the aircraft, so in the exterior
 * view the model sits rigid in the frame while the world moves behind it - exactly
 * how an external chase camera behaves.
 */
(function () {
  'use strict';
  if (window.__geftSimCleanup) { try { window.__geftSimCleanup(); } catch (e) { /* ignore */ } }

  const MODES = ['exterior', 'cockpit', 'off'];
  let mode = localStorage.getItem('geft.view');
  if (MODES.indexOf(mode) < 0) mode = 'exterior';

  const KT = 1.94384, FT = 3.28084, FPM = 196.85;
  const clamp = (v, a, b) => (v < a ? a : v > b ? b : v);
  const lerp = (a, b, t) => a + (b - a) * t;
  const wrap180 = (d) => ((d + 540) % 360) - 180;
  const D2R = Math.PI / 180;

  // ======================================================================= mat4
  const m4 = {
    ident: () => new Float32Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]),
    mul(a, b) {
      const o = new Float32Array(16);
      for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) {
        o[c * 4 + r] = a[r] * b[c * 4] + a[4 + r] * b[c * 4 + 1] + a[8 + r] * b[c * 4 + 2] + a[12 + r] * b[c * 4 + 3];
      }
      return o;
    },
    perspective(fovy, aspect, near, far) {
      const f = 1 / Math.tan(fovy / 2), nf = 1 / (near - far);
      return new Float32Array([f / aspect, 0, 0, 0, 0, f, 0, 0, 0, 0, (far + near) * nf, -1, 0, 0, 2 * far * near * nf, 0]);
    },
    lookAt(eye, center, up) {
      const z = norm(sub(eye, center)), x = norm(cross(up, z)), y = cross(z, x);
      return new Float32Array([
        x[0], y[0], z[0], 0, x[1], y[1], z[1], 0, x[2], y[2], z[2], 0,
        -dot(x, eye), -dot(y, eye), -dot(z, eye), 1]);
    },
    trans(x, y, z) { const m = m4.ident(); m[12] = x; m[13] = y; m[14] = z; return m; },
    rotX(a) { const c = Math.cos(a), s = Math.sin(a), m = m4.ident(); m[5] = c; m[6] = s; m[9] = -s; m[10] = c; return m; },
    rotY(a) { const c = Math.cos(a), s = Math.sin(a), m = m4.ident(); m[0] = c; m[2] = -s; m[8] = s; m[10] = c; return m; },
    rotZ(a) { const c = Math.cos(a), s = Math.sin(a), m = m4.ident(); m[0] = c; m[1] = s; m[4] = -s; m[5] = c; return m; },
    mat3(m) { return new Float32Array([m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]]); },
  };
  const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const norm = (a) => { const l = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };

  // ================================================================ mesh helper
  function Mesh() { this.p = []; this.n = []; this.c = []; }
  Mesh.prototype.quad = function (a, b, c, d, col) {
    const u = sub(b, a), v = sub(d, a); const nn = norm(cross(u, v));
    const push = (p) => { this.p.push(p[0], p[1], p[2]); this.n.push(nn[0], nn[1], nn[2]); this.c.push(col[0], col[1], col[2]); };
    push(a); push(b); push(c); push(a); push(c); push(d);
  };
  const AXMAP = {
    z: (u, v, t) => [u, v, t],
    x: (u, v, t) => [t, v, u],
    y: (u, v, t) => [u, t, v],
  };
  // cross-section profiles, unit space (-0.5 .. 0.5)
  const P_RECT = [[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]];
  const P_OCT = [[-0.5, -0.21], [-0.21, -0.5], [0.21, -0.5], [0.5, -0.21],
    [0.5, 0.21], [0.21, 0.5], [-0.21, 0.5], [-0.5, 0.21]];
  const P_ROUND = (() => { const o = []; for (let i = 0; i < 12; i++) { const a = (i / 12) * Math.PI * 2; o.push([Math.cos(a) * 0.5, Math.sin(a) * 0.5]); } return o; })();
  // airfoil: u = chord (-0.5 leading .. +0.5 trailing), v = thickness
  const P_FOIL = (() => {
    const up = [], lo = [];
    for (let i = 0; i <= 8; i++) {
      const x = i / 8;                       // 0 at LE, 1 at TE
      const th = 2.4 * (0.2969 * Math.sqrt(x) - 0.126 * x - 0.3516 * x * x + 0.2843 * x * x * x - 0.1015 * x * x * x * x);
      up.push([x - 0.5, th * 0.5]); lo.push([x - 0.5, -th * 0.42]);
    }
    return up.concat(lo.reverse());
  })();

  // secs: [{t, u, v, du, dv}] - profiles swept along `axis`
  Mesh.prototype.loft = function (axis, secs, col, profile) {
    const map = AXMAP[axis];
    const pr = profile || P_RECT;
    const ring = (s) => pr.map((p) => map(s.u + p[0] * s.du, s.v + p[1] * s.dv, s.t));
    const N = pr.length;
    for (let i = 0; i < secs.length - 1; i++) {
      const a = ring(secs[i]), b = ring(secs[i + 1]);
      for (let k = 0; k < N; k++) { const k2 = (k + 1) % N; this.quad(a[k], a[k2], b[k2], b[k], col); }
    }
    const cap = (r, flip) => {
      const c = r.reduce((s, p) => [s[0] + p[0] / r.length, s[1] + p[1] / r.length, s[2] + p[2] / r.length], [0, 0, 0]);
      for (let k = 0; k < N; k++) {
        const k2 = (k + 1) % N;
        if (flip) this.quad(c, r[k2], r[k], r[k], col); else this.quad(c, r[k], r[k2], r[k2], col);
      }
    };
    cap(ring(secs[0]), true);
    cap(ring(secs[secs.length - 1]), false);
  };
  Mesh.P_RECT = P_RECT; Mesh.P_OCT = P_OCT; Mesh.P_ROUND = P_ROUND; Mesh.P_FOIL = P_FOIL;

  // ================================================================== aircraft
  // High-wing single-engine tourer. Nose points -Z, +Y up, +X right. Units ~metres.
  const C = {
    body: [0.93, 0.94, 0.96],
    accent: [0.16, 0.36, 0.60],
    dark: [0.13, 0.14, 0.16],
    glass: [0.10, 0.15, 0.21],
    metal: [0.52, 0.55, 0.58],
    tyre: [0.09, 0.09, 0.10],
    prop: [0.15, 0.15, 0.17],
    tip: [0.85, 0.24, 0.20],
  };

  function buildAircraft() {
    const parts = {};

    // ---- fuselage
    const f = new Mesh();
    f.loft('z', [
      { t: -4.15, u: 0, v: 0.10, du: 0.34, dv: 0.34 },
      { t: -3.70, u: 0, v: 0.08, du: 0.90, dv: 0.88 },
      { t: -2.90, u: 0, v: 0.06, du: 1.06, dv: 1.10 },
      { t: -1.30, u: 0, v: 0.04, du: 1.14, dv: 1.28 },
      { t: 0.30, u: 0, v: 0.02, du: 1.08, dv: 1.18 },
      { t: 1.60, u: 0, v: 0.08, du: 0.78, dv: 0.86 },
      { t: 3.00, u: 0, v: 0.18, du: 0.44, dv: 0.50 },
      { t: 3.95, u: 0, v: 0.24, du: 0.22, dv: 0.30 },
    ], C.body, Mesh.P_OCT);
    // accent stripe along the flanks
    f.loft('z', [
      { t: -3.55, u: 0, v: -0.28, du: 1.09, dv: 0.15 },
      { t: 0.30, u: 0, v: -0.32, du: 1.12, dv: 0.18 },
      { t: 3.10, u: 0, v: -0.10, du: 0.46, dv: 0.13 },
    ], C.accent, Mesh.P_RECT);
    // cabin glass
    f.loft('z', [
      { t: -2.78, u: 0, v: 0.38, du: 1.00, dv: 0.30 },
      { t: -2.20, u: 0, v: 0.42, du: 1.12, dv: 0.46 },
      { t: -0.75, u: 0, v: 0.42, du: 1.14, dv: 0.48 },
      { t: -0.05, u: 0, v: 0.38, du: 1.04, dv: 0.34 },
    ], C.glass, Mesh.P_OCT);
    // cowling
    f.loft('z', [
      { t: -3.98, u: 0, v: 0.10, du: 0.66, dv: 0.62 },
      { t: -3.55, u: 0, v: 0.08, du: 0.94, dv: 0.88 },
    ], C.dark, Mesh.P_OCT);
    parts.body = f;

    // ---- wing (high, slight dihedral via stepped sections)
    const w = new Mesh();
    const wingSec = (x, y, chord, thick) => ({ t: x, u: -0.45 + chord * 0.12, v: y, du: chord, dv: thick });
    w.loft('x', [
      { t: -5.55, u: -0.30, v: 1.16, du: 1.06, dv: 0.30 },
      { t: -4.20, u: -0.34, v: 1.12, du: 1.30, dv: 0.38 },
      { t: -1.60, u: -0.38, v: 1.06, du: 1.58, dv: 0.46 },
      { t: 1.60, u: -0.38, v: 1.06, du: 1.58, dv: 0.46 },
      { t: 4.20, u: -0.34, v: 1.12, du: 1.30, dv: 0.38 },
      { t: 5.55, u: -0.30, v: 1.16, du: 1.06, dv: 0.30 },
    ], C.body, Mesh.P_FOIL);
    void wingSec;
    // wing tips
    w.loft('x', [{ t: -5.74, u: -0.30, v: 1.17, du: 0.94, dv: 0.24 }, { t: -5.52, u: -0.30, v: 1.16, du: 1.06, dv: 0.30 }], C.tip, Mesh.P_FOIL);
    w.loft('x', [{ t: 5.52, u: -0.30, v: 1.16, du: 1.06, dv: 0.30 }, { t: 5.74, u: -0.30, v: 1.17, du: 0.94, dv: 0.24 }], C.tip, Mesh.P_FOIL);
    // struts
    w.loft('y', [{ t: -0.42, u: -1.05, v: -0.35, du: 0.09, dv: 0.22 }, { t: 1.02, u: -2.95, v: -0.42, du: 0.09, dv: 0.22 }], C.metal, Mesh.P_OCT);
    w.loft('y', [{ t: -0.42, u: 1.05, v: -0.35, du: 0.09, dv: 0.22 }, { t: 1.02, u: 2.95, v: -0.42, du: 0.09, dv: 0.22 }], C.metal, Mesh.P_OCT);
    parts.wing = w;

    // ---- ailerons (hinged, outboard trailing edge)
    const al = new Mesh();
    al.loft('x', [{ t: -5.28, u: 0.30, v: 1.15, du: 0.34, dv: 0.07 }, { t: -3.15, u: 0.34, v: 1.09, du: 0.42, dv: 0.09 }], C.body, Mesh.P_OCT);
    parts.aileronL = al; parts.aileronL.hinge = [0, 1.12, 0.14];
    const ar = new Mesh();
    ar.loft('x', [{ t: 3.15, u: 0.34, v: 1.09, du: 0.42, dv: 0.09 }, { t: 5.28, u: 0.30, v: 1.15, du: 0.34, dv: 0.07 }], C.body, Mesh.P_OCT);
    parts.aileronR = ar; parts.aileronR.hinge = [0, 1.12, 0.14];

    // ---- tailplane
    const t = new Mesh();
    t.loft('x', [
      { t: -1.85, u: 3.62, v: 0.30, du: 0.62, dv: 0.18 },
      { t: -0.55, u: 3.48, v: 0.28, du: 0.92, dv: 0.26 },
      { t: 0.55, u: 3.48, v: 0.28, du: 0.92, dv: 0.26 },
      { t: 1.85, u: 3.62, v: 0.30, du: 0.62, dv: 0.18 },
    ], C.body, Mesh.P_FOIL);
    // fin - swept, loft along Y with chord in Z
    t.loft('y', [
      { t: 0.30, u: 0, v: 3.30, du: 0.22, dv: 1.10 },
      { t: 1.20, u: 0, v: 3.52, du: 0.18, dv: 0.86 },
      { t: 1.95, u: 0, v: 3.72, du: 0.13, dv: 0.52 },
    ], C.body, Mesh.P_FOIL);
    t.loft('y', [
      { t: 1.35, u: 0, v: 3.58, du: 0.19, dv: 0.72 },
      { t: 1.96, u: 0, v: 3.73, du: 0.14, dv: 0.50 },
    ], C.accent, Mesh.P_FOIL);
    parts.tail = t;

    const el = new Mesh();
    el.loft('x', [
      { t: -1.82, u: 4.02, v: 0.30, du: 0.34, dv: 0.09 },
      { t: 1.82, u: 4.02, v: 0.30, du: 0.34, dv: 0.09 },
    ], C.body, Mesh.P_OCT);
    parts.elevator = el; parts.elevator.hinge = [0, 0.29, 3.86];

    const rd = new Mesh();
    rd.loft('y', [
      { t: 0.32, u: 0, v: 4.02, du: 0.11, dv: 0.36 },
      { t: 1.90, u: 0, v: 4.02, du: 0.09, dv: 0.26 },
    ], C.body, Mesh.P_OCT);
    parts.rudder = rd; parts.rudder.hinge = [0, 3.30, 3.86];

    // ---- gear
    const g = new Mesh();
    const leg = (x) => {
      g.loft('y', [{ t: -0.62, u: x * 1.34, v: -0.25, du: 0.10, dv: 0.14 }, { t: -0.05, u: x * 0.42, v: -0.28, du: 0.12, dv: 0.16 }], C.metal, Mesh.P_OCT);
      g.loft('x', [{ t: x * 1.22, u: -0.25, v: -0.80, du: 0.46, dv: 0.46 }, { t: x * 1.46, u: -0.25, v: -0.80, du: 0.46, dv: 0.46 }], C.tyre, Mesh.P_ROUND);
    };
    leg(-1); leg(1);
    g.loft('y', [{ t: -0.80, u: 0, v: -3.15, du: 0.11, dv: 0.11 }, { t: -0.20, u: 0, v: -3.05, du: 0.13, dv: 0.13 }], C.metal, Mesh.P_OCT);
    g.loft('x', [{ t: -0.12, u: -3.15, v: -0.94, du: 0.38, dv: 0.38 }, { t: 0.12, u: -3.15, v: -0.94, du: 0.38, dv: 0.38 }], C.tyre, Mesh.P_ROUND);
    parts.gear = g;

    // ---- spinner + blades (rotate about Z at the nose)
    const sp = new Mesh();
    sp.loft('z', [
      { t: -4.46, u: 0, v: 0.10, du: 0.07, dv: 0.07 },
      { t: -4.34, u: 0, v: 0.10, du: 0.24, dv: 0.24 },
      { t: -4.16, u: 0, v: 0.10, du: 0.32, dv: 0.32 },
    ], C.dark, Mesh.P_ROUND);
    parts.spinner = sp;

    const bl = new Mesh();
    bl.loft('y', [
      { t: 0.14, u: 0, v: -4.31, du: 0.17, dv: 0.05 },
      { t: 0.55, u: 0, v: -4.31, du: 0.19, dv: 0.05 },
      { t: 0.94, u: 0, v: -4.31, du: 0.11, dv: 0.04 },
    ], C.prop, Mesh.P_FOIL);
    bl.loft('y', [
      { t: -0.94, u: 0, v: -4.31, du: 0.11, dv: 0.04 },
      { t: -0.55, u: 0, v: -4.31, du: 0.19, dv: 0.05 },
      { t: -0.14, u: 0, v: -4.31, du: 0.17, dv: 0.05 },
    ], C.prop, Mesh.P_FOIL);
    parts.blades = bl; parts.blades.spin = true;

    // ---- prop disc (translucent, gives the blur)
    const dc = new Mesh();
    const R = 0.95, seg = 28;
    for (let i = 0; i < seg; i++) {
      const a0 = (i / seg) * Math.PI * 2, a1 = ((i + 1) / seg) * Math.PI * 2;
      dc.quad([0, 0.10, -4.28], [Math.cos(a0) * R, 0.10 + Math.sin(a0) * R, -4.28],
        [Math.cos(a1) * R, 0.10 + Math.sin(a1) * R, -4.28], [0, 0.10, -4.28], [0.62, 0.64, 0.68]);
    }
    parts.disc = dc; parts.disc.alpha = 0.16;

    return parts;
  }

  // ==================================================================== WebGL
  const VS = `
    attribute vec3 aPos; attribute vec3 aNormal; attribute vec3 aColor;
    uniform mat4 uMVP; uniform mat4 uModel; uniform mat3 uNM;
    varying vec3 vN; varying vec3 vC; varying vec3 vW;
    void main(){ gl_Position = uMVP*vec4(aPos,1.0); vN = uNM*aNormal; vC = aColor;
      vW = (uModel*vec4(aPos,1.0)).xyz; }`;
  const FS = `
    precision mediump float;
    varying vec3 vN; varying vec3 vC; varying vec3 vW;
    uniform vec3 uLight; uniform vec3 uEye; uniform float uAlpha;
    void main(){
      vec3 n = normalize(vN); if(!gl_FrontFacing) n = -n;
      vec3 l = normalize(uLight);
      vec3 v = normalize(uEye - vW);
      float diff = max(dot(n,l),0.0);
      vec3 h = normalize(l+v);
      float spec = pow(max(dot(n,h),0.0), 42.0)*0.30;
      float rim = pow(1.0-max(dot(n,v),0.0), 3.0)*0.16;
      float sky = 0.40 + 0.60*clamp(n.y*0.5+0.5,0.0,1.0);
      vec3 col = vC*(0.34*sky + 0.80*diff) + vec3(spec) + vec3(0.55,0.68,0.88)*rim;
      gl_FragColor = vec4(col, uAlpha);
    }`;

  let gl = null, prog = null, loc = null, gpu = null, cv3d = null;

  function initGL() {
    cv3d = document.createElement('canvas');
    cv3d.id = 'geft-3d';
    Object.assign(cv3d.style, {
      position: 'fixed', inset: '0', width: '100%', height: '100%',
      pointerEvents: 'none', zIndex: '2147483500',
    });
    document.body.appendChild(cv3d);
    gl = cv3d.getContext('webgl', { alpha: true, antialias: true, premultipliedAlpha: false });
    if (!gl) return false;
    const sh = (t, src) => { const s = gl.createShader(t); gl.shaderSource(s, src); gl.compileShader(s); return s; };
    prog = gl.createProgram();
    gl.attachShader(prog, sh(gl.VERTEX_SHADER, VS));
    gl.attachShader(prog, sh(gl.FRAGMENT_SHADER, FS));
    gl.linkProgram(prog);
    if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) { console.warn('[geft]', gl.getProgramInfoLog(prog)); return false; }
    loc = {
      aPos: gl.getAttribLocation(prog, 'aPos'),
      aNormal: gl.getAttribLocation(prog, 'aNormal'),
      aColor: gl.getAttribLocation(prog, 'aColor'),
      uMVP: gl.getUniformLocation(prog, 'uMVP'),
      uModel: gl.getUniformLocation(prog, 'uModel'),
      uNM: gl.getUniformLocation(prog, 'uNM'),
      uLight: gl.getUniformLocation(prog, 'uLight'),
      uEye: gl.getUniformLocation(prog, 'uEye'),
      uAlpha: gl.getUniformLocation(prog, 'uAlpha'),
    };
    const parts = buildAircraft();
    gpu = {};
    for (const k in parts) {
      const m = parts[k];
      const buf = (arr) => { const b = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, b); gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(arr), gl.STATIC_DRAW); return b; };
      gpu[k] = { p: buf(m.p), n: buf(m.n), c: buf(m.c), count: m.p.length / 3, hinge: m.hinge, spin: m.spin, alpha: m.alpha };
    }
    return true;
  }

  function bindPart(P) {
    gl.bindBuffer(gl.ARRAY_BUFFER, P.p); gl.enableVertexAttribArray(loc.aPos); gl.vertexAttribPointer(loc.aPos, 3, gl.FLOAT, false, 0, 0);
    gl.bindBuffer(gl.ARRAY_BUFFER, P.n); gl.enableVertexAttribArray(loc.aNormal); gl.vertexAttribPointer(loc.aNormal, 3, gl.FLOAT, false, 0, 0);
    gl.bindBuffer(gl.ARRAY_BUFFER, P.c); gl.enableVertexAttribArray(loc.aColor); gl.vertexAttribPointer(loc.aColor, 3, gl.FLOAT, false, 0, 0);
  }

  // =================================================================== HUD 2D
  let cvHud = null, ctx = null;
  function initHud() {
    cvHud = document.createElement('canvas');
    cvHud.id = 'geft-hud';
    Object.assign(cvHud.style, {
      position: 'fixed', inset: '0', width: '100%', height: '100%',
      pointerEvents: 'none', zIndex: '2147483540',
    });
    document.body.appendChild(cvHud);
    ctx = cvHud.getContext('2d');
  }

  let W = 0, H = 0, DPR = 1;
  function resize() {
    DPR = Math.min(window.devicePixelRatio || 1, 2);
    W = window.innerWidth; H = window.innerHeight;
    for (const c of [cv3d, cvHud]) {
      if (!c) continue;
      c.width = Math.round(W * DPR); c.height = Math.round(H * DPR);
    }
    if (ctx) ctx.setTransform(DPR, 0, 0, DPR, 0, 0);
  }

  // ------------------------------------------------------------- instruments
  const FONT = '600 {s}px ui-monospace, SFMono-Regular, Menlo, monospace';
  const fs = (s) => FONT.replace('{s}', s);

  function bezel(x, y, r, label) {
    ctx.save();
    const g = ctx.createLinearGradient(x, y - r, x, y + r);
    g.addColorStop(0, '#2b2e33'); g.addColorStop(1, '#15171a');
    ctx.beginPath(); ctx.arc(x, y, r + r * 0.11, 0, 7); ctx.fillStyle = g; ctx.fill();
    ctx.lineWidth = 1; ctx.strokeStyle = 'rgba(255,255,255,.10)'; ctx.stroke();
    ctx.beginPath(); ctx.arc(x, y, r, 0, 7); ctx.fillStyle = '#0b0d0f'; ctx.fill();
    ctx.restore();
    if (label) {
      ctx.fillStyle = 'rgba(190,200,210,.45)'; ctx.font = fs(Math.round(r * 0.16));
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText(label, x, y + r * 1.30);
    }
  }
  function clipCircle(x, y, r, fn) { ctx.save(); ctx.beginPath(); ctx.arc(x, y, r, 0, 7); ctx.clip(); fn(); ctx.restore(); }
  function needle(x, y, ang, len, w, col, back) {
    ctx.save(); ctx.translate(x, y); ctx.rotate(ang);
    ctx.beginPath(); ctx.moveTo(-w, back || -len * 0.18); ctx.lineTo(0, -len); ctx.lineTo(w, back || -len * 0.18);
    ctx.closePath(); ctx.fillStyle = col; ctx.fill(); ctx.restore();
  }

  // Earth's simulator happily flies at jet speeds, so the dials pick a range
  // instead of pegging. Hysteresis keeps them from flickering between scales.
  const SPD_RANGES = [200, 400, 800, 1600, 3200];
  const VS_RANGES = [2000, 6000, 12000];
  function pickRange(list, v, cur) {
    let want = list[list.length - 1];
    for (const r of list) if (v <= r * 0.92) { want = r; break; }
    if (cur && want < cur && v > cur * 0.42) return cur;   // only scale down well clear
    return want;
  }
  let asiRange = 200, vsiRange = 2000;

  function drawASI(x, y, r, kts) {
    asiRange = pickRange(SPD_RANGES, kts, asiRange);
    const M = asiRange;
    bezel(x, y, r, 'AIRSPEED');
    ctx.save(); ctx.translate(x, y);
    const a = (v) => (-135 + (clamp(v, 0, M) / M) * 320) * D2R;
    const arc = (f0, f1, col) => {
      ctx.beginPath(); ctx.arc(0, 0, r * 0.84, a(M * f0) - Math.PI / 2, a(M * f1) - Math.PI / 2);
      ctx.lineWidth = r * 0.07; ctx.strokeStyle = col; ctx.stroke();
    };
    arc(0.20, 0.64, '#3fae52');
    arc(0.64, 0.815, '#d8b02e');
    arc(0.815, 1.0, '#cc3b30');
    ctx.font = fs(Math.round(r * (M >= 1000 ? 0.15 : 0.19))); ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    for (let i = 0; i <= 10; i++) {
      const v = (M / 10) * i, ang = a(v);
      ctx.save(); ctx.rotate(ang);
      ctx.beginPath(); ctx.moveTo(0, -r * 0.92); ctx.lineTo(0, -r * 0.78);
      ctx.lineWidth = 2; ctx.strokeStyle = '#d8dee6'; ctx.stroke(); ctx.restore();
      ctx.fillStyle = '#cfd6dd';
      ctx.fillText(String(v), Math.sin(ang) * r * 0.60, -Math.cos(ang) * r * 0.60);
    }
    needle(0, 0, a(kts), r * 0.80, r * 0.045, '#f2f5f8');
    ctx.beginPath(); ctx.arc(0, 0, r * 0.06, 0, 7); ctx.fillStyle = '#e9edf2'; ctx.fill();
    ctx.restore();
  }

  function drawAI(x, y, r, pitch, roll) {
    bezel(x, y, r, 'ATTITUDE');
    clipCircle(x, y, r, () => {
      ctx.save(); ctx.translate(x, y); ctx.rotate(-roll * D2R);
      const ppd = r / 22;                       // pixels per degree of pitch
      const o = clamp(pitch, -60, 60) * ppd;
      const sky = ctx.createLinearGradient(0, o - r * 2, 0, o);
      sky.addColorStop(0, '#1f6fc4'); sky.addColorStop(1, '#5aa8e8');
      ctx.fillStyle = sky; ctx.fillRect(-r * 2, o - r * 2.4, r * 4, r * 2.4);
      const gnd = ctx.createLinearGradient(0, o, 0, o + r * 2);
      gnd.addColorStop(0, '#9a6a34'); gnd.addColorStop(1, '#5c3f20');
      ctx.fillStyle = gnd; ctx.fillRect(-r * 2, o, r * 4, r * 2.4);
      ctx.beginPath(); ctx.moveTo(-r * 2, o); ctx.lineTo(r * 2, o);
      ctx.lineWidth = 2; ctx.strokeStyle = '#ffffff'; ctx.stroke();
      ctx.font = fs(Math.round(r * 0.13)); ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      for (let d = -30; d <= 30; d += 5) {
        if (!d) continue;
        const wq = (d % 10 === 0) ? r * 0.34 : r * 0.17;
        const yy = o - d * ppd;
        ctx.beginPath(); ctx.moveTo(-wq, yy); ctx.lineTo(wq, yy);
        ctx.lineWidth = 1.5; ctx.strokeStyle = 'rgba(255,255,255,.85)'; ctx.stroke();
        if (d % 10 === 0) { ctx.fillStyle = '#fff'; ctx.fillText(String(Math.abs(d)), -wq - r * 0.12, yy); }
      }
      ctx.restore();
      // bank scale
      ctx.save(); ctx.translate(x, y);
      ctx.strokeStyle = '#e8edf2'; ctx.fillStyle = '#e8edf2';
      [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60].forEach((b) => {
        ctx.save(); ctx.rotate(b * D2R);
        const big = (b % 30 === 0);
        ctx.beginPath(); ctx.moveTo(0, -r * 0.97); ctx.lineTo(0, -r * (big ? 0.84 : 0.90));
        ctx.lineWidth = big ? 2.4 : 1.4; ctx.stroke(); ctx.restore();
      });
      ctx.save(); ctx.rotate(-roll * D2R);
      ctx.beginPath(); ctx.moveTo(0, -r * 0.82); ctx.lineTo(-r * 0.06, -r * 0.71); ctx.lineTo(r * 0.06, -r * 0.71);
      ctx.closePath(); ctx.fillStyle = '#ffcf4a'; ctx.fill(); ctx.restore();
      ctx.restore();
    });
    // fixed aircraft symbol
    ctx.save(); ctx.translate(x, y);
    ctx.strokeStyle = '#ffcf4a'; ctx.lineWidth = 3; ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.moveTo(-r * 0.52, 0); ctx.lineTo(-r * 0.18, 0); ctx.lineTo(-r * 0.10, r * 0.09);
    ctx.moveTo(r * 0.52, 0); ctx.lineTo(r * 0.18, 0); ctx.lineTo(r * 0.10, r * 0.09);
    ctx.stroke();
    ctx.beginPath(); ctx.arc(0, 0, r * 0.035, 0, 7); ctx.fillStyle = '#ffcf4a'; ctx.fill();
    ctx.restore();
  }

  function drawALT(x, y, r, ft) {
    bezel(x, y, r, 'ALTITUDE');
    ctx.save(); ctx.translate(x, y);
    ctx.font = fs(Math.round(r * 0.20)); ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    for (let i = 0; i < 10; i++) {
      const ang = (i / 10) * Math.PI * 2;
      ctx.save(); ctx.rotate(ang);
      ctx.beginPath(); ctx.moveTo(0, -r * 0.94); ctx.lineTo(0, -r * 0.80);
      ctx.lineWidth = 2.2; ctx.strokeStyle = '#d8dee6'; ctx.stroke();
      ctx.restore();
      ctx.fillStyle = '#cfd6dd';
      ctx.fillText(String(i), Math.sin(ang) * r * 0.64, -Math.cos(ang) * r * 0.64);
      for (let j = 1; j < 5; j++) {
        const a2 = ang + (j / 50) * Math.PI * 2;
        ctx.save(); ctx.rotate(a2);
        ctx.beginPath(); ctx.moveTo(0, -r * 0.94); ctx.lineTo(0, -r * 0.88);
        ctx.lineWidth = 1; ctx.strokeStyle = 'rgba(216,222,230,.6)'; ctx.stroke(); ctx.restore();
      }
    }
    const h = ft / 1000, k = (ft % 1000) / 100;
    ctx.fillStyle = 'rgba(190,200,210,.5)'; ctx.font = fs(Math.round(r * 0.13));
    ctx.fillText('FEET', 0, -r * 0.42);
    // digital drum
    ctx.fillStyle = '#0f1216'; ctx.strokeStyle = 'rgba(255,255,255,.18)';
    const bw = r * 0.86, bh = r * 0.26;
    ctx.beginPath(); ctx.rect(-bw / 2, r * 0.30, bw, bh); ctx.fill(); ctx.stroke();
    ctx.fillStyle = '#e9edf2'; ctx.font = fs(Math.round(r * 0.21));
    ctx.fillText(Math.round(ft).toLocaleString('en-US'), 0, r * 0.30 + bh / 2 + 1);
    needle(0, 0, (h / 10) * Math.PI * 2, r * 0.52, r * 0.055, '#c9d2db');
    needle(0, 0, (k / 10) * Math.PI * 2, r * 0.86, r * 0.038, '#f2f5f8');
    ctx.beginPath(); ctx.arc(0, 0, r * 0.06, 0, 7); ctx.fillStyle = '#e9edf2'; ctx.fill();
    ctx.restore();
  }

  function drawTC(x, y, r, roll, turnRate) {
    bezel(x, y, r, 'TURN COORD');
    ctx.save(); ctx.translate(x, y);
    ctx.strokeStyle = 'rgba(216,222,230,.75)'; ctx.lineWidth = 2;
    [-30, 30].forEach((b) => {
      ctx.save(); ctx.rotate(b * D2R);
      ctx.beginPath(); ctx.moveTo(0, -r * 0.92); ctx.lineTo(0, -r * 0.76); ctx.stroke(); ctx.restore();
    });
    ctx.fillStyle = 'rgba(190,200,210,.5)'; ctx.font = fs(Math.round(r * 0.14));
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText('L', -r * 0.56, -r * 0.62); ctx.fillText('R', r * 0.56, -r * 0.62);
    // little aeroplane, banks with the turn rate
    const tr = clamp(turnRate / 3, -1.6, 1.6);
    ctx.save(); ctx.rotate(tr * 20 * D2R);
    ctx.strokeStyle = '#e9edf2'; ctx.lineWidth = 3.4; ctx.lineCap = 'round';
    ctx.beginPath(); ctx.moveTo(-r * 0.62, 0); ctx.lineTo(r * 0.62, 0); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0, -r * 0.08); ctx.lineTo(0, r * 0.26); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(-r * 0.20, r * 0.26); ctx.lineTo(r * 0.20, r * 0.26); ctx.stroke();
    ctx.restore();
    // inclinometer
    ctx.beginPath(); ctx.rect(-r * 0.40, r * 0.52, r * 0.80, r * 0.22);
    ctx.fillStyle = '#12151a'; ctx.fill(); ctx.strokeStyle = 'rgba(255,255,255,.15)'; ctx.lineWidth = 1; ctx.stroke();
    const slip = clamp(-roll / 45 + tr * 0.55, -1, 1);
    ctx.beginPath(); ctx.arc(slip * r * 0.26, r * 0.63, r * 0.085, 0, 7);
    ctx.fillStyle = '#1b1d21'; ctx.fill();
    ctx.beginPath(); ctx.arc(slip * r * 0.26, r * 0.63, r * 0.075, 0, 7);
    ctx.fillStyle = '#d6dbe1'; ctx.fill();
    ctx.restore();
  }

  function drawHSI(x, y, r, hdg, track) {
    bezel(x, y, r, 'HEADING');
    clipCircle(x, y, r, () => {
      ctx.save(); ctx.translate(x, y); ctx.rotate(-hdg * D2R);
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      for (let d = 0; d < 360; d += 5) {
        const ang = d * D2R, big = d % 30 === 0;
        ctx.save(); ctx.rotate(ang);
        ctx.beginPath(); ctx.moveTo(0, -r * 0.93); ctx.lineTo(0, -r * (big ? 0.78 : 0.86));
        ctx.lineWidth = big ? 2.2 : 1.2; ctx.strokeStyle = big ? '#e2e8ee' : 'rgba(216,222,230,.55)'; ctx.stroke();
        ctx.restore();
        if (big) {
          const lbl = d === 0 ? 'N' : d === 90 ? 'E' : d === 180 ? 'S' : d === 270 ? 'W' : String(d / 10);
          ctx.save(); ctx.translate(Math.sin(ang) * r * 0.62, -Math.cos(ang) * r * 0.62); ctx.rotate(hdg * D2R);
          ctx.fillStyle = (d % 90 === 0) ? '#ffcf4a' : '#cfd6dd';
          ctx.font = fs(Math.round(r * (d % 90 === 0 ? 0.24 : 0.19)));
          ctx.fillText(lbl, 0, 0); ctx.restore();
        }
      }
      if (track != null) {
        ctx.save(); ctx.rotate(track * D2R);
        ctx.beginPath(); ctx.moveTo(0, -r * 0.72); ctx.lineTo(-r * 0.05, -r * 0.60); ctx.lineTo(r * 0.05, -r * 0.60);
        ctx.closePath(); ctx.fillStyle = '#4fd35a'; ctx.fill(); ctx.restore();
      }
      ctx.restore();
    });
    ctx.save(); ctx.translate(x, y);
    ctx.strokeStyle = '#e9edf2'; ctx.lineWidth = 3; ctx.lineCap = 'round';
    ctx.beginPath(); ctx.moveTo(-r * 0.30, r * 0.02); ctx.lineTo(r * 0.30, r * 0.02); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0, -r * 0.24); ctx.lineTo(0, r * 0.26); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0, -r * 0.99); ctx.lineTo(-r * 0.07, -r * 0.86); ctx.lineTo(r * 0.07, -r * 0.86);
    ctx.closePath(); ctx.fillStyle = '#ffcf4a'; ctx.fill();
    ctx.fillStyle = '#0f1216'; ctx.strokeStyle = 'rgba(255,255,255,.18)'; ctx.lineWidth = 1;
    const bw = r * 0.72, bh = r * 0.26;
    ctx.beginPath(); ctx.rect(-bw / 2, -r * 0.62, bw, bh); ctx.fill(); ctx.stroke();
    ctx.fillStyle = '#e9edf2'; ctx.font = fs(Math.round(r * 0.21)); ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText(String(Math.round(hdg)).padStart(3, '0') + '\u00b0', 0, -r * 0.62 + bh / 2 + 1);
    ctx.restore();
  }

  function drawVSI(x, y, r, fpm) {
    vsiRange = pickRange(VS_RANGES, Math.abs(fpm), vsiRange);
    const M = vsiRange, step = M / 4;
    bezel(x, y, r, 'VERT SPEED');
    ctx.save(); ctx.translate(x, y);
    const a = (v) => (clamp(v, -M, M) / M) * 160 * D2R;
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    for (let v = -M; v <= M; v += step / 2) {
      const ang = a(v) + Math.PI / 2;
      ctx.save(); ctx.rotate(ang);
      ctx.beginPath(); ctx.moveTo(0, -r * 0.93); ctx.lineTo(0, -r * 0.79);
      ctx.lineWidth = Math.abs(v % step) < 1 ? 2.2 : 1.2; ctx.strokeStyle = '#d8dee6'; ctx.stroke(); ctx.restore();
      if (Math.abs(v % (step * 2)) < 1) {
        ctx.fillStyle = '#cfd6dd'; ctx.font = fs(Math.round(r * 0.19));
        ctx.fillText(String(Math.abs(v / 1000)), Math.sin(ang) * r * 0.62, -Math.cos(ang) * r * 0.62);
      }
    }
    ctx.fillStyle = 'rgba(190,200,210,.45)'; ctx.font = fs(Math.round(r * 0.11));
    ctx.fillText('x1000 FT/MIN', 0, r * 0.52);
    ctx.fillStyle = fpm > 60 ? '#4fd35a' : fpm < -60 ? '#ff8f6a' : '#cfd6dd';
    ctx.font = fs(Math.round(r * 0.17));
    ctx.fillText((fpm > 0 ? '+' : '') + Math.round(fpm), 0, -r * 0.44);
    needle(0, 0, a(fpm) + Math.PI / 2, r * 0.84, r * 0.042, '#f2f5f8');
    ctx.beginPath(); ctx.arc(0, 0, r * 0.06, 0, 7); ctx.fillStyle = '#e9edf2'; ctx.fill();
    ctx.restore();
  }

  function drawWindscreen() {
    const g = ctx.createLinearGradient(0, 0, 0, H * 0.16);
    g.addColorStop(0, 'rgba(14,15,17,.92)'); g.addColorStop(1, 'rgba(14,15,17,0)');
    ctx.fillStyle = g; ctx.fillRect(0, 0, W, H * 0.16);
    ctx.fillStyle = 'rgba(17,18,21,.95)';
    // left pillar
    ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(W * 0.14, 0); ctx.lineTo(0, H * 0.52); ctx.closePath(); ctx.fill();
    // right pillar
    ctx.beginPath(); ctx.moveTo(W, 0); ctx.lineTo(W * 0.86, 0); ctx.lineTo(W, H * 0.52); ctx.closePath(); ctx.fill();
  }

  function drawPanel(st) {
    const ph = clamp(H * 0.30, 190, 280);
    const py = H - ph;
    const g = ctx.createLinearGradient(0, py, 0, H);
    g.addColorStop(0, '#22252a'); g.addColorStop(0.06, '#1a1d21'); g.addColorStop(1, '#0d0f11');
    ctx.fillStyle = g; ctx.fillRect(0, py, W, ph);
    ctx.fillStyle = 'rgba(255,255,255,.10)'; ctx.fillRect(0, py, W, 1);

    const r = clamp(ph * 0.27, 44, 76);
    const gap = r * 2.70;
    const cy = py + ph * 0.40;
    const cx0 = W / 2 - gap * 2.5;
    const kts = st.ias, ft = st.altm * FT, fpm = st.vsms * FPM;
    drawASI(cx0 + gap * 0, cy, r, kts);
    drawAI(cx0 + gap * 1, cy, r, st.pitch, st.roll);
    drawALT(cx0 + gap * 2, cy, r, ft);
    drawTC(cx0 + gap * 3, cy, r, st.roll, st.turnRate);
    drawHSI(cx0 + gap * 4, cy, r, st.hdg, st.track);
    drawVSI(cx0 + gap * 5, cy, r, fpm);

    // digital strip
    const dy = H - ph * 0.075;
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    const cell = (cx, k, v, col) => {
      ctx.fillStyle = 'rgba(170,180,192,.55)'; ctx.font = fs(9);
      ctx.fillText(k, cx, dy - 9);
      ctx.fillStyle = col || '#dfe5ec'; ctx.font = fs(13);
      ctx.fillText(v, cx, dy + 5);
    };
    const items = [
      ['IAS', Math.round(kts) + ' KT'],
      ['TRK', st.track == null ? '---' : String(Math.round(st.track)).padStart(3, '0') + '\u00b0'],
      ['ALT', Math.round(ft).toLocaleString('en-US') + ' FT'],
      ['BANK', Math.round(Math.abs(st.roll)) + '\u00b0 ' + (st.roll > 1 ? 'R' : st.roll < -1 ? 'L' : '')],
      ['POS', st.lat.toFixed(3) + ', ' + st.lon.toFixed(3)],
      ['REC', st.recording ? (st.paused ? 'PAUSED' : 'ON ' + st.points) : 'OFF'],
    ];
    const step = Math.min(150, W / (items.length + 2));
    const x0 = W / 2 - (step * (items.length - 1)) / 2;
    items.forEach((it, i) => cell(x0 + step * i, it[0], it[1], it[0] === 'REC' && st.recording && !st.paused ? '#ff6b60' : null));
  }

  // ------------------------------------------------- flight-model instruments
  function drawFlightOverlay(st) {
    const f = st.fl;
    if (!f) return;
    const pad = 16, w = 128;
    const top = Math.max(70, H * 0.12);
    ctx.save();
    ctx.textBaseline = 'middle';

    // aircraft + autopilot header
    ctx.fillStyle = 'rgba(10,12,14,.58)';
    ctx.beginPath(); ctx.roundRect(pad, top, w, 26, 7); ctx.fill();
    ctx.strokeStyle = 'rgba(255,255,255,.08)'; ctx.lineWidth = 1; ctx.stroke();
    ctx.fillStyle = '#e9edf2'; ctx.font = fs(11); ctx.textAlign = 'center';
    ctx.fillText(f.name, pad + w / 2, top + 13);

    // vertical throttle bar
    const bx = pad, by = top + 38, bw = 26, bh = Math.min(190, H * 0.24);
    ctx.fillStyle = 'rgba(10,12,14,.58)';
    ctx.beginPath(); ctx.roundRect(bx, by, bw, bh, 6); ctx.fill();
    ctx.strokeStyle = 'rgba(255,255,255,.08)'; ctx.stroke();
    const th = bh * clamp(f.thr, 0, 1);
    const tg = ctx.createLinearGradient(0, by + bh, 0, by);
    tg.addColorStop(0, '#2fd2ff'); tg.addColorStop(1, '#7ce0ff');
    ctx.fillStyle = tg;
    ctx.beginPath(); ctx.roundRect(bx + 3, by + bh - th + 3 - (th ? 3 : 0), bw - 6, Math.max(th - 3, 0), 4); ctx.fill();
    ctx.fillStyle = 'rgba(190,200,210,.65)'; ctx.font = fs(9); ctx.textAlign = 'center';
    ctx.fillText(Math.round(f.thr * 100) + '%', bx + bw / 2, by + bh + 12);

    // readout column
    const cx = bx + bw + 10, cw = w - bw - 10;
    const rows = [
      ['AOA', f.alpha.toFixed(1) + '\u00b0', Math.abs(f.alpha) > 14 ? '#ffcf4a' : null],
      ['G', f.nz.toFixed(2), (f.nz > f.gLim[1] || f.nz < f.gLim[0]) ? '#ff6b60' : null],
      ['MACH', f.mach.toFixed(2), null],
      ['AGL', f.agl == null ? '---' : Math.round(f.agl * FT) + ' ft', f.agl != null && f.agl < 300 ? '#ffcf4a' : null],
      ['FLAP', Math.round(f.flap * 100) + '%', f.flap > 0 ? '#2fd2ff' : null],
      ['GEAR', f.gear > 0.5 ? 'DOWN' : 'UP', f.gear > 0.5 ? '#4fd35a' : null],
      ['TRIM', (f.trim >= 0 ? '+' : '') + f.trim.toFixed(2), null],
    ];
    ctx.fillStyle = 'rgba(10,12,14,.58)';
    ctx.beginPath(); ctx.roundRect(cx, by, cw, rows.length * 22 + 6, 7); ctx.fill();
    ctx.strokeStyle = 'rgba(255,255,255,.08)'; ctx.stroke();
    rows.forEach((r, i) => {
      const y = by + 14 + i * 22;
      ctx.textAlign = 'left'; ctx.fillStyle = 'rgba(170,180,192,.6)'; ctx.font = fs(9);
      ctx.fillText(r[0], cx + 8, y);
      ctx.textAlign = 'right'; ctx.fillStyle = r[2] || '#dfe5ec'; ctx.font = fs(11);
      ctx.fillText(r[1], cx + cw - 8, y);
    });

    // autopilot strip
    const apOn = f.ap.on && (f.ap.hdg || f.ap.alt || f.ap.spd);
    if (apOn) {
      const ay = by + rows.length * 22 + 14;
      ctx.fillStyle = 'rgba(10,12,14,.58)';
      ctx.beginPath(); ctx.roundRect(pad, ay, w, 40, 7); ctx.fill();
      ctx.strokeStyle = 'rgba(79,211,90,.35)'; ctx.stroke();
      ctx.textAlign = 'left'; ctx.fillStyle = '#4fd35a'; ctx.font = fs(10);
      ctx.fillText('AUTOPILOT', pad + 8, ay + 12);
      ctx.font = fs(10); ctx.fillStyle = '#dfe5ec';
      const bits = [];
      if (f.ap.hdg) bits.push('HDG ' + String(Math.round(f.ap.tgtHdg)).padStart(3, '0'));
      if (f.ap.alt) bits.push('ALT ' + Math.round(f.ap.tgtAlt * FT));
      if (f.ap.spd) bits.push('SPD ' + Math.round(f.ap.tgtSpd));
      ctx.fillText(bits.join('  '), pad + 8, ay + 28);
    }

    // warnings
    const warn = [];
    if (f.crashed) warn.push(['CRASHED - press R', '#ff6b60']);
    if (f.stall) warn.push(['STALL', '#ff6b60']);
    if (f.over) warn.push(['OVERSPEED', '#ffcf4a']);
    if (f.gpws) warn.push([f.gpws, '#ff6b60']);
    if (f.onGround && !f.crashed) warn.push(['ON GROUND', '#8b93a0']);
    warn.forEach((wn, i) => {
      ctx.textAlign = 'center'; ctx.font = fs(22);
      ctx.fillStyle = wn[1];
      ctx.fillText(wn[0], W / 2, H * 0.22 + i * 30);
    });
    if (f.msg) {
      ctx.textAlign = 'center'; ctx.font = fs(12); ctx.fillStyle = 'rgba(233,237,242,.85)';
      ctx.fillText(f.msg, W / 2, H * 0.16);
    }
    if (f.pad) {
      ctx.textAlign = 'left'; ctx.font = fs(9); ctx.fillStyle = 'rgba(79,211,90,.8)';
      ctx.fillText('\u2699 ' + f.pad, pad, top - 12);
    }
    ctx.restore();
  }

  function drawExteriorHud(st) {
    // thin top-corner readouts only; Earth draws its own ladder and compass
    ctx.textBaseline = 'middle';
    const box = (x, y, w, h) => {
      ctx.fillStyle = 'rgba(10,12,14,.55)';
      ctx.beginPath(); ctx.roundRect(x, y, w, h, 7); ctx.fill();
      ctx.strokeStyle = 'rgba(255,255,255,.08)'; ctx.lineWidth = 1; ctx.stroke();
    };
    const kts = Math.round(st.ias), ft = Math.round(st.altm * FT), fpm = Math.round(st.vsms * FPM);
    const B = 132;                              // clear of Earth's attribution bar
    ctx.textAlign = 'left';
    box(18, H - B, 158, 80);
    ctx.fillStyle = 'rgba(170,180,192,.6)'; ctx.font = fs(9);
    ctx.fillText('AIRSPEED', 31, H - B + 16); ctx.fillText('ALTITUDE', 31, H - B + 52);
    ctx.fillStyle = '#e9edf2'; ctx.font = fs(19);
    ctx.fillText(kts + ' kt', 31, H - B + 33);
    ctx.fillText(ft.toLocaleString('en-US') + ' ft', 31, H - B + 69);
    box(W - 176, H - B, 158, 80);
    ctx.fillStyle = 'rgba(170,180,192,.6)'; ctx.font = fs(9);
    ctx.fillText('VERT SPEED', W - 163, H - B + 16); ctx.fillText('BANK', W - 163, H - B + 52);
    ctx.fillStyle = fpm > 60 ? '#4fd35a' : fpm < -60 ? '#ff8f6a' : '#e9edf2'; ctx.font = fs(19);
    ctx.fillText((fpm > 0 ? '+' : '') + fpm + ' fpm', W - 163, H - B + 33);
    ctx.fillStyle = '#e9edf2';
    ctx.fillText(Math.round(Math.abs(st.roll)) + '\u00b0 ' + (st.roll > 1 ? 'R' : st.roll < -1 ? 'L' : '\u2014'), W - 163, H - B + 69);
  }

  // ==================================================================== loop
  const S = { lagRoll: 0, lagPitch: 0, lagHdg: null, prop: 0, turnRate: 0, last: 0 };
  let raf = 0;

  function frame(now) {
    raf = requestAnimationFrame(frame);
    const g = window.GEFT;
    const dt = S.last ? Math.min((now - S.last) / 1000, 0.1) : 0.016;
    S.last = now;

    if (ctx) ctx.clearRect(0, 0, W, H);
    if (gl) { gl.viewport(0, 0, cv3d.width, cv3d.height); gl.clearColor(0, 0, 0, 0); gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT); }
    if (mode === 'off' || !g) return;

    // attitude lag -> what the chase camera has not caught up with yet
    S.lagRoll = lerp(S.lagRoll, g.roll, clamp(dt * 3.2, 0, 1));
    S.lagPitch = lerp(S.lagPitch, g.pitch, clamp(dt * 3.2, 0, 1));
    if (S.lagHdg == null) S.lagHdg = g.hdg;
    const dh = wrap180(g.hdg - S.lagHdg);
    S.lagHdg = (S.lagHdg + dh * clamp(dt * 2.4, 0, 1) + 360) % 360;
    S.turnRate = lerp(S.turnRate, dh / Math.max(dt, 0.001) * 0.35, clamp(dt * 4, 0, 1));

    const fl = window.GEFT_FLIGHT && window.GEFT_FLIGHT.on ? window.GEFT_FLIGHT : null;
    const st = {
      lat: g.lat, lon: g.lon, alt: g.alt, hdg: g.hdg, pitch: g.pitch, roll: g.roll,
      gs: g.gs, vs: g.vs, track: g.track, recording: g.recording, paused: g.paused,
      points: g.points, turnRate: S.turnRate, fl,
      // the flight model is authoritative when it is flying
      ias: fl ? fl.ias : g.gs * KT,
      vsms: fl ? fl.vs : g.vs,
      altm: fl ? fl.alt : g.alt,
    };
    if (fl) { st.pitch = fl.pitch; st.roll = fl.roll; st.hdg = fl.hdg; }

    if (mode === 'cockpit') {
      drawWindscreen();
      drawPanel(st);
      drawFlightOverlay(st);
      return;
    }

    // ---- exterior
    drawExteriorHud(st);
    drawFlightOverlay(st);
    if (!gl) return;

    const rollOff = clamp((g.roll - S.lagRoll) * 0.70, -18, 18);
    const pitchOff = clamp((g.pitch - S.lagPitch) * 0.70, -13, 13);
    const yawOff = clamp(wrap180(g.hdg - S.lagHdg) * 1.2, -11, 11);
    S.prop = (S.prop + dt * (26 + Math.min(g.gs, 400) * 0.10)) % (Math.PI * 2);

    const aspect = cv3d.width / cv3d.height;
    const proj = m4.perspective(33 * D2R, aspect, 0.2, 400);
    const eye = [0, 5.4 + pitchOff * 0.03, 27.5];
    const view = m4.lookAt(eye, [0, 2.0, 0], [0, 1, 0]);
    const pv = m4.mul(proj, view);

    // aircraft body transform: lag offsets only (Earth's camera already carries
    // the absolute attitude, so the model stays rigid in frame)
    const base = m4.mul(
      m4.mul(m4.rotZ(-rollOff * D2R), m4.rotX(pitchOff * D2R)),
      m4.rotY(-yawOff * D2R));

    gl.useProgram(prog);
    gl.enable(gl.DEPTH_TEST);
    gl.disable(gl.CULL_FACE);
    gl.enable(gl.BLEND);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    gl.uniform3f(loc.uLight, -0.45, 0.80, 0.42);
    gl.uniform3f(loc.uEye, eye[0], eye[1], eye[2]);

    const rollRate = (g.roll - S.lagRoll);
    const ail = clamp(rollRate * 1.5, -22, 22);
    const elev = clamp((g.pitch - S.lagPitch) * -1.8, -18, 18);
    const rud = clamp(wrap180(g.hdg - S.lagHdg) * -2.2, -20, 20);

    const drawPart = (key, local, alpha, depthWrite) => {
      const P = gpu[key]; if (!P) return;
      const model = local ? m4.mul(base, local) : base;
      gl.uniformMatrix4fv(loc.uMVP, false, m4.mul(pv, model));
      gl.uniformMatrix4fv(loc.uModel, false, model);
      gl.uniformMatrix3fv(loc.uNM, false, m4.mat3(model));
      gl.uniform1f(loc.uAlpha, alpha == null ? 1 : alpha);
      gl.depthMask(depthWrite === false ? false : true);
      bindPart(P);
      gl.drawArrays(gl.TRIANGLES, 0, P.count);
    };

    const hinged = (h, axisFn, deg) => m4.mul(m4.mul(m4.trans(h[0], h[1], h[2]), axisFn(deg * D2R)), m4.trans(-h[0], -h[1], -h[2]));

    drawPart('body');
    drawPart('wing');
    drawPart('tail');
    drawPart('gear');
    drawPart('aileronL', hinged(gpu.aileronL.hinge, m4.rotX, -ail));
    drawPart('aileronR', hinged(gpu.aileronR.hinge, m4.rotX, ail));
    drawPart('elevator', hinged(gpu.elevator.hinge, m4.rotX, elev));
    drawPart('rudder', hinged(gpu.rudder.hinge, m4.rotY, rud));
    drawPart('spinner', m4.mul(m4.trans(0, 0.10, 0), m4.mul(m4.rotZ(S.prop), m4.trans(0, -0.10, 0))));
    drawPart('blades', m4.mul(m4.trans(0, 0.10, 0), m4.mul(m4.rotZ(S.prop), m4.trans(0, -0.10, 0))), 0.9);
    drawPart('disc', null, 0.14, false);
    gl.depthMask(true);
  }

  // ====================================================================== UI
  function addControl() {
    const panel = document.getElementById('geft');
    if (!panel || document.getElementById('geft-view')) return;
    const rows = panel.querySelectorAll('.row');
    const row = document.createElement('div');
    row.className = 'row';
    row.innerHTML = '<button id="geft-view"></button>';
    const anchor = rows[rows.length - 1];
    if (anchor && anchor.parentNode) anchor.parentNode.insertBefore(row, anchor.nextSibling);
    else panel.querySelector('.bd').appendChild(row);
    const b = row.querySelector('#geft-view');
    const label = () => { b.textContent = 'View: ' + mode[0].toUpperCase() + mode.slice(1) + '  (V)'; b.classList.toggle('on', mode !== 'off'); };
    b.onclick = () => { cycle(); label(); };
    window.__geftViewLabel = label;
    label();
  }
  function cycle() {
    mode = MODES[(MODES.indexOf(mode) + 1) % MODES.length];
    localStorage.setItem('geft.view', mode);
    if (window.__geftViewLabel) window.__geftViewLabel();
  }
  const onKey = (e) => {
    if (e.metaKey || e.ctrlKey || e.altKey) return;
    const tag = (e.target && e.target.tagName) || '';
    if (tag === 'INPUT' || tag === 'TEXTAREA') return;
    if (e.code === 'KeyV') { e.preventDefault(); cycle(); }
  };

  // =================================================================== boot
  initHud();
  const ok = initGL();
  if (!ok) console.warn('[geft] WebGL overlay unavailable, cockpit view still works');
  resize();
  window.addEventListener('resize', resize);
  window.addEventListener('keydown', onKey, true);
  raf = requestAnimationFrame(frame);
  const uiTimer = setInterval(addControl, 1200);
  addControl();

  window.__geftSimCleanup = () => {
    cancelAnimationFrame(raf);
    clearInterval(uiTimer);
    window.removeEventListener('resize', resize);
    window.removeEventListener('keydown', onKey, true);
    for (const c of [cv3d, cvHud]) if (c && c.parentNode) c.parentNode.removeChild(c);
    const r = document.getElementById('geft-view');
    if (r && r.parentNode && r.parentNode.parentNode) r.parentNode.parentNode.removeChild(r.parentNode);
    window.__geftSimCleanup = null;
  };
  window.__geftView = (m) => { if (MODES.indexOf(m) >= 0) { mode = m; localStorage.setItem('geft.view', m); if (window.__geftViewLabel) window.__geftViewLabel(); } };
})();
