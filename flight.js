/* Google Earth Flight Tracker - flight dynamics
 *
 * Google Earth's own flight simulator is a toy: no aerodynamics, no stall, no
 * mass, and it happily cruises at 1,500 knots. The camera struct in Earth's
 * WebAssembly heap turns out to be *writable*, so this replaces Earth's physics
 * entirely and flies the camera itself, using Earth only as the renderer.
 *
 * The model is a conventional stability-derivative rigid-body simulation:
 *   - body-axis velocities (u, v, w) and rates (p, q, r), Euler attitude
 *   - CL(alpha) with a real stall break, CD = CD0 + induced + gear/flap
 *   - roll/pitch/yaw moments from control surfaces, damping and static stability
 *     (dihedral effect, weathercock, adverse yaw)
 *   - ISA atmosphere, so density altitude, service ceiling, IAS vs TAS and Mach
 *     all fall out of the model rather than being faked
 *   - propeller thrust (falls off with airspeed) or turbofan thrust (falls off
 *     with density)
 *   - wind and turbulence, ground handling, gear, flaps and brakes
 *
 * Integrated at 200 Hz with an accumulator so the handling does not change with
 * frame rate.
 */
(function () {
  'use strict';
  if (window.__geftFlightCleanup) { try { window.__geftFlightCleanup(); } catch (e) { /* ignore */ } }

  const D2R = Math.PI / 180, R2D = 180 / Math.PI;
  const RE = 6371008.8, G = 9.80665;
  const clamp = (v, a, b) => (v < a ? a : v > b ? b : v);
  const lerp = (a, b, t) => a + (b - a) * t;
  const wrap360 = (d) => ((d % 360) + 360) % 360;
  const wrap180 = (d) => ((d + 540) % 360) - 180;

  // ----------------------------------------------------------- ISA atmosphere
  function atmos(h) {
    h = clamp(h, -500, 84000);
    let T, p;
    if (h < 11000) { T = 288.15 - 0.0065 * h; p = 101325 * Math.pow(T / 288.15, 5.25588); }
    else if (h < 20000) { T = 216.65; p = 22632.1 * Math.exp(-G * (h - 11000) / (287.053 * T)); }
    else if (h < 32000) { T = 216.65 + 0.001 * (h - 20000); p = 5474.89 * Math.pow(T / 216.65, -34.1632); }
    else { T = 228.65 + 0.0028 * (h - 32000); p = 868.019 * Math.pow(T / 228.65, -12.2011); }
    const rho = p / (287.053 * T);
    return { T, p, rho, a: Math.sqrt(1.4 * 287.053 * T), sigma: rho / 1.225 };
  }

  // ------------------------------------------------------------- aircraft set
  // Coefficients are per-radian, conventional sign convention.
  const AIRCRAFT = {
    c172: {
      name: 'Cessna 172', kind: 'prop',
      m: 1111, S: 16.17, b: 11.0, c: 1.49, Ix: 1285, Iy: 1825, Iz: 2667,
      CL0: 0.31, CLa: 5.14, CLq: 3.9, CLde: 0.43, aStall: 16 * D2R, CLmaxTail: 1.0,
      CD0: 0.031, e: 0.78,
      Cm0: 0.04, Cma: -0.89, Cmq: -12.4, Cmde: -1.28,
      CYb: -0.31, CYdr: 0.187,
      Clb: -0.089, Clp: -0.47, Clr: 0.096, Clda: 0.178, Cldr: 0.0147,
      Cnb: 0.065, Cnp: -0.03, Cnr: -0.099, Cnda: -0.053, Cndr: -0.0657,
      deMax: 25 * D2R, daMax: 20 * D2R, drMax: 16 * D2R,
      power: 134000, propEta: 0.80, vMaxProp: 90,      // W, efficiency, thrust knee
      flapCL: 0.9, flapCD: 0.045, flapStall: -3 * D2R,
      gearCD: 0.015, gearFixed: true,
      vne: 163, vfe: 85, vs: 47,                        // knots
      gLim: [-1.5, 3.8],
      startAlt: 300, cruise: 110,
    },
    extra300: {
      name: 'Extra 300', kind: 'prop',
      m: 950, S: 10.84, b: 8.0, c: 1.4, Ix: 780, Iy: 1100, Iz: 1500,
      CL0: 0.05, CLa: 5.5, CLq: 4.2, CLde: 0.62, aStall: 18 * D2R,
      CD0: 0.028, e: 0.85,
      Cm0: 0.0, Cma: -0.55, Cmq: -11.0, Cmde: -1.9,
      CYb: -0.34, CYdr: 0.23,
      Clb: -0.06, Clp: -0.58, Clr: 0.10, Clda: 0.52, Cldr: 0.02,
      Cnb: 0.07, Cnp: -0.02, Cnr: -0.11, Cnda: -0.02, Cndr: -0.10,
      deMax: 30 * D2R, daMax: 32 * D2R, drMax: 28 * D2R,
      power: 224000, propEta: 0.82, vMaxProp: 100,
      flapCL: 0, flapCD: 0, flapStall: 0,
      gearCD: 0.012, gearFixed: true,
      vne: 220, vfe: 0, vs: 58,
      gLim: [-10, 10],
      startAlt: 500, cruise: 180,
    },
    airliner: {
      name: 'Jet Airliner', kind: 'jet',
      m: 62000, S: 122.6, b: 34.1, c: 4.29, Ix: 1.4e6, Iy: 3.0e6, Iz: 4.3e6,
      CL0: 0.21, CLa: 5.6, CLq: 6.6, CLde: 0.36, aStall: 15 * D2R,
      CD0: 0.021, e: 0.80,
      Cm0: 0.02, Cma: -0.75, Cmq: -22.0, Cmde: -1.45,
      CYb: -0.55, CYdr: 0.14,
      Clb: -0.12, Clp: -0.52, Clr: 0.14, Clda: 0.18, Cldr: 0.012,
      Cnb: 0.12, Cnp: -0.04, Cnr: -0.20, Cnda: -0.02, Cndr: -0.10,
      deMax: 22 * D2R, daMax: 18 * D2R, drMax: 20 * D2R,
      thrust: 240000, jetLapse: 0.75,
      flapCL: 1.05, flapCD: 0.075, flapStall: -4 * D2R,
      gearCD: 0.022, gearFixed: false,
      vne: 350, vfe: 200, vs: 125, mmo: 0.82,
      gLim: [-1.0, 2.5],
      startAlt: 1000, cruise: 280,
    },
    glider: {
      name: 'Glider', kind: 'glider',
      m: 600, S: 17.95, b: 17.0, c: 0.95, Ix: 2500, Iy: 1800, Iz: 3800,
      CL0: 0.25, CLa: 5.7, CLq: 5.5, CLde: 0.5, aStall: 15 * D2R,
      CD0: 0.013, e: 0.92,
      Cm0: 0.03, Cma: -0.9, Cmq: -16.0, Cmde: -1.5,
      CYb: -0.28, CYdr: 0.16,
      Clb: -0.10, Clp: -0.55, Clr: 0.12, Clda: 0.19, Cldr: 0.015,
      Cnb: 0.06, Cnp: -0.03, Cnr: -0.09, Cnda: -0.06, Cndr: -0.07,
      deMax: 25 * D2R, daMax: 22 * D2R, drMax: 20 * D2R,
      thrust: 0,
      flapCL: 0.3, flapCD: 0.02, flapStall: -2 * D2R,
      gearCD: 0.01, gearFixed: true,
      vne: 135, vfe: 90, vs: 38,
      gLim: [-2.5, 5.3],
      startAlt: 1200, cruise: 60,
    },
    hypersonic: {
      name: 'Hypersonic', kind: 'jet',
      m: 30000, S: 70, b: 14, c: 8, Ix: 3e5, Iy: 1.2e6, Iz: 1.4e6,
      CL0: 0.02, CLa: 3.0, CLq: 3.0, CLde: 0.25, aStall: 24 * D2R,
      CD0: 0.016, e: 0.60,
      Cm0: 0, Cma: -0.45, Cmq: -14, Cmde: -1.0,
      CYb: -0.4, CYdr: 0.10,
      Clb: -0.08, Clp: -0.40, Clr: 0.10, Clda: 0.16, Cldr: 0.01,
      Cnb: 0.10, Cnp: -0.03, Cnr: -0.16, Cnda: -0.01, Cndr: -0.08,
      deMax: 20 * D2R, daMax: 18 * D2R, drMax: 16 * D2R,
      thrust: 900000, jetLapse: 0.35,
      flapCL: 0, flapCD: 0, flapStall: 0,
      gearCD: 0.02, gearFixed: false,
      vne: 1400, vfe: 0, vs: 190, mmo: 6.0,
      gLim: [-3, 9],
      startAlt: 12000, cruise: 900,
    },
  };
  const ORDER = ['c172', 'extra300', 'airliner', 'glider', 'hypersonic'];

  // ----------------------------------------------------------------- the state
  const F = {
    on: false, paused: false,
    type: localStorage.getItem('geft.aircraft') || 'c172',
    lat: 0, lon: 0, alt: 1000,
    u: 60, v: 0, w: 0,            // body velocities, m/s
    p: 0, q: 0, r: 0,             // body rates, rad/s
    phi: 0, theta: 0, psi: 0,     // roll, pitch, heading, radians
    thr: 0.7, flap: 0, gear: 1, brake: 0, trim: 0,
    de: 0, da: 0, dr: 0,          // control deflections, -1..1
    onGround: false, crashed: false,
    nz: 1, alpha: 0, beta: 0, tas: 0, ias: 0, mach: 0, vs: 0, vsRaw: 0,
    agl: null, ground: null, gpws: '',
    stall: false, over: false,
    // Terrain height is screen-picked and approximate, so collision is opt-in (K).
    // AGL and the ground-proximity warnings are always live and advisory.
    terrain: localStorage.getItem('geft.terrain') === '1',
    nSamp: 0, nearest: null, groundRate: 0, deep: 0,
    ap: { on: false, hdg: false, alt: false, spd: false, tgtHdg: 0, tgtAlt: 1000, tgtSpd: 0 },
    wind: { dir: 240, kt: 8, gust: 0 },
    msg: '',
  };
  const A = () => AIRCRAFT[F.type];

  // ------------------------------------------------------------------- input
  const keys = Object.create(null);
  const IN = { pitch: 0, roll: 0, yaw: 0 };   // -1..1 demanded
  let pad = null;

  function readGamepad() {
    const gps = navigator.getGamepads ? navigator.getGamepads() : [];
    pad = null;
    for (const g of gps) if (g && g.connected) { pad = g; break; }
    if (!pad) return false;
    const ax = pad.axes, dz = (x) => (Math.abs(x) < 0.08 ? 0 : x);
    IN.roll = dz(ax[0] || 0);
    IN.pitch = dz(ax[1] || 0);      // stick forward (negative) = nose down
    if (ax.length > 2) IN.yaw = dz(ax[2] || 0);
    if (ax.length > 3) F.thr = clamp((1 - ax[3]) / 2, 0, 1);
    const b = pad.buttons || [];
    if (b[0] && b[0].pressed) toggleGear();
    if (b[1] && b[1].pressed) F.brake = 1; else if (!keys.KeyB) F.brake = 0;
    return true;
  }

  function keyAxes(dt) {
    const rate = 2.6, center = 3.4;
    const ax = (neg, pos, cur) => {
      let d = 0;
      if (keys[neg]) d -= 1;
      if (keys[pos]) d += 1;
      if (d) return clamp(cur + d * rate * dt, -1, 1);
      if (cur > 0) return Math.max(0, cur - center * dt);
      return Math.min(0, cur + center * dt);
    };
    // convention: +1 = nose up, +1 = right bank, +1 = right rudder
    IN.pitch = ax('ArrowDown', 'ArrowUp', IN.pitch);
    IN.roll = ax('ArrowLeft', 'ArrowRight', IN.roll);
    IN.yaw = ax('KeyQ', 'KeyE', IN.yaw);
    if (keys.KeyW) IN.pitch = clamp(IN.pitch - rate * dt, -1, 1);   // W = nose down
    if (keys.KeyS) IN.pitch = clamp(IN.pitch + rate * dt, -1, 1);   // S = nose up
    if (keys.KeyA) IN.roll = clamp(IN.roll - rate * dt, -1, 1);
    if (keys.KeyD) IN.roll = clamp(IN.roll + rate * dt, -1, 1);
    if (keys.PageUp || keys.Equal) F.thr = clamp(F.thr + 0.45 * dt, 0, 1);
    if (keys.PageDown || keys.Minus) F.thr = clamp(F.thr - 0.45 * dt, 0, 1);
    if (keys.Comma) F.trim = clamp(F.trim - 0.25 * dt, -1, 1);
    if (keys.Period) F.trim = clamp(F.trim + 0.25 * dt, -1, 1);
    F.brake = keys.KeyB ? 1 : F.brake;
  }

  // Control surfaces cannot snap from stop to stop instantly; limiting the rate
  // keeps both the pilot and the autopilot from commanding step inputs.
  const CMD = { pitch: 0, roll: 0, yaw: 0 };
  function rateLimit(dt) {
    const mx = 3.2 * dt;
    for (const k of ['pitch', 'roll', 'yaw']) {
      CMD[k] += clamp(IN[k] - CMD[k], -mx, mx);
      IN[k] = CMD[k];
    }
  }

  function toggleGear() {
    if (A().gearFixed) { flash('gear is fixed on this aircraft'); return; }
    F.gear = F.gear > 0.5 ? 0 : 1;
    flash(F.gear ? 'gear down' : 'gear up');
  }
  let flashT = 0;
  function flash(m) { F.msg = m; flashT = performance.now() + 2200; }

  // --------------------------------------------------------------- autopilot
  // Cascaded and deliberately gentle: altitude -> vertical speed -> pitch attitude
  // -> elevator, heading -> bank angle -> aileron, each stage rate- and
  // authority-limited. An autopilot allowed full deflection will loop the
  // aeroplane, which is exactly what the first version did.
  function autopilot(dt) {
    const ap = F.ap;
    if (!ap.on) return;

    // upset recovery takes priority over everything else
    if (Math.abs(F.phi) > 75 * D2R || Math.abs(F.theta) > 35 * D2R) { levelOff(); return; }

    if (ap.hdg) {
      const err = wrap180(ap.tgtHdg - wrap360(F.psi * R2D));
      const tgtBank = clamp(err * 0.9, -22, 22) * D2R;
      IN.roll = clamp((tgtBank - F.phi) * 1.4 - F.p * 0.85, -0.5, 0.5);
      IN.yaw = clamp(-F.beta * 3, -0.35, 0.35);
    }
    if (ap.alt) {
      const errM = ap.tgtAlt - F.alt;
      const tgtVs = clamp(errM * 0.05, -7, 7);                     // m/s
      const vsErr = clamp(tgtVs - F.vs, -8, 8);
      const tgtPitch = clamp(F.theta + vsErr * 0.02, -10 * D2R, 12 * D2R);
      IN.pitch = clamp((tgtPitch - F.theta) * 2.6 - F.q * 1.2, -0.35, 0.35);
    }
    if (ap.spd && A().kind !== 'glider') {
      F.thr = clamp(F.thr + (ap.tgtSpd - F.ias) * 0.0012, 0, 1);
    }
  }

  // Wings level, nose on the horizon. Unloads first when inverted, because
  // pulling while upside down only makes the dive steeper.
  function levelOff() {
    IN.roll = clamp(-F.phi * 1.6 - F.p * 0.9, -1, 1);
    const inverted = Math.abs(F.phi) > 100 * D2R;
    const tgtPitch = inverted ? 0 : 2 * D2R;
    const gain = inverted ? 1.2 : 3.0;
    IN.pitch = clamp((tgtPitch - F.theta) * gain - F.q * 1.3, inverted ? -0.2 : -0.75, 0.75);
    IN.yaw = clamp(-F.beta * 3, -0.4, 0.4);
  }

  // ----------------------------------------------------------------- physics
  let turbState = [0, 0, 0];
  let simT = 0, wallT0 = 0;
  function step(dt) {
    simT += dt;
    const ac = A();
    const at = atmos(F.alt);

    // --- wind in NED
    const wDir = (F.wind.dir + 180) * D2R, wSpd = F.wind.kt * 0.514444;
    for (let i = 0; i < 3; i++) {
      turbState[i] = turbState[i] * 0.985 + (Math.random() - 0.5) * F.wind.gust * 0.35;
    }
    const wN = Math.cos(wDir) * wSpd + turbState[0];
    const wE = Math.sin(wDir) * wSpd + turbState[1];
    const wD = turbState[2] * 0.5;

    const cphi = Math.cos(F.phi), sphi = Math.sin(F.phi);
    const cth = Math.cos(F.theta), sth = Math.sin(F.theta);
    const cpsi = Math.cos(F.psi), spsi = Math.sin(F.psi);
    // body <- NED rotation matrix rows
    const R11 = cth * cpsi, R12 = cth * spsi, R13 = -sth;
    const R21 = sphi * sth * cpsi - cphi * spsi, R22 = sphi * sth * spsi + cphi * cpsi, R23 = sphi * cth;
    const R31 = cphi * sth * cpsi + sphi * spsi, R32 = cphi * sth * spsi - sphi * cpsi, R33 = cphi * cth;

    // wind in body axes
    const wu = R11 * wN + R12 * wE + R13 * wD;
    const wv = R21 * wN + R22 * wE + R23 * wD;
    const ww = R31 * wN + R32 * wE + R33 * wD;
    const ua = F.u - wu, va = F.v - wv, wa = F.w - ww;      // airmass-relative

    const V = Math.max(Math.hypot(ua, va, wa), 0.5);
    const alpha = Math.atan2(wa, Math.max(ua, 0.5));
    const beta = Math.asin(clamp(va / V, -1, 1));
    F.alpha = alpha; F.beta = beta; F.tas = V;
    F.ias = V * Math.sqrt(at.sigma) * 1.94384;               // knots
    F.mach = V / at.a;

    // --- controls
    // elevator deflection is negative for a nose-up command (Cmde is negative)
    const de = -clamp(IN.pitch + F.trim * 0.6, -1, 1) * ac.deMax;
    const da = clamp(IN.roll, -1, 1) * ac.daMax;
    const dr = clamp(IN.yaw, -1, 1) * ac.drMax;
    F.de = IN.pitch; F.da = IN.roll; F.dr = IN.yaw;

    // --- lift with a real stall break
    const aStall = ac.aStall + (ac.flapStall || 0) * F.flap;
    let CL = ac.CL0 + ac.CLa * alpha + ac.CLde * de
      + ac.CLq * F.q * ac.c / (2 * V) + (ac.flapCL || 0) * F.flap;
    const CLstall = ac.CL0 + ac.CLa * aStall;
    let stalled = false;
    if (Math.abs(alpha) > aStall) {
      stalled = true;
      const over = Math.abs(alpha) - aStall;
      // post-stall: lift collapses then behaves like a flat plate
      const decay = Math.exp(-over * 7);
      const flat = 1.1 * Math.sin(2 * alpha);
      CL = Math.sign(alpha) * (Math.abs(CLstall) * decay) + flat * (1 - decay);
    }
    F.stall = stalled;

    const AR = ac.b * ac.b / ac.S;
    let CD = ac.CD0 + (CL * CL) / (Math.PI * AR * ac.e)
      + (ac.flapCD || 0) * F.flap + ac.gearCD * F.gear;
    if (stalled) CD += 0.9 * Math.pow(Math.abs(alpha) - aStall, 1.4);
    if (ac.mmo) { const dm = F.mach - (ac.mmo - 0.05); if (dm > 0) CD += 0.055 * dm * dm * 60; }

    const CY = ac.CYb * beta + ac.CYdr * dr;
    const qbar = 0.5 * at.rho * V * V;
    const L = qbar * ac.S * CL, D = qbar * ac.S * CD, Yf = qbar * ac.S * CY;
    F.nz = (L * Math.cos(alpha) + D * Math.sin(alpha)) / (ac.m * G);   // aerodynamic load factor

    // --- thrust
    let T = 0;
    if (ac.kind === 'prop') {
      const pw = ac.power * F.thr * Math.pow(at.sigma, 0.8);
      T = pw * ac.propEta / Math.max(V, ac.vMaxProp * 0.35);
      T = Math.min(T, pw * ac.propEta / (ac.vMaxProp * 0.35));
    } else if (ac.kind === 'jet') {
      T = ac.thrust * F.thr * Math.pow(at.sigma, ac.jetLapse || 0.8);
    }

    // --- forces in body axes
    const sa = Math.sin(alpha), ca = Math.cos(alpha);
    let X = T + L * sa - D * ca;
    let Y = Yf;
    let Z = -L * ca - D * sa;
    // gravity
    X += -G * sth * ac.m;
    Y += G * sphi * cth * ac.m;
    Z += G * cphi * cth * ac.m;

    // --- moments
    const b2v = ac.b / (2 * V), c2v = ac.c / (2 * V);
    let Cl = ac.Clb * beta + ac.Clp * F.p * b2v + ac.Clr * F.r * b2v + ac.Clda * da + ac.Cldr * dr;
    let Cm = ac.Cm0 + ac.Cma * alpha + ac.Cmq * F.q * c2v + ac.Cmde * de;
    let Cn = ac.Cnb * beta + ac.Cnp * F.p * b2v + ac.Cnr * F.r * b2v + ac.Cnda * da + ac.Cndr * dr;
    if (stalled) {
      Cl *= 0.35; Cn *= 0.6;                                  // controls go soft
      Cl += (Math.random() - 0.5) * 0.02;                     // wing drop / buffet
      Cm -= 0.05;
    }
    const l = qbar * ac.S * ac.b * Cl;
    const mM = qbar * ac.S * ac.c * Cm;
    const n = qbar * ac.S * ac.b * Cn;

    // --- ground
    // The terrain height is interpolated from screen-picked samples, so it is an
    // estimate, not a height map. Contact is handled generously and a crash needs
    // sustained penetration rather than one bad sample.
    groundUpdate();
    const gh = F.ground;
    const agl = gh == null ? null : F.alt - gh;
    F.agl = agl;
    const wheelH = 1.6;
    const groundSettled = F.groundRate != null && Math.abs(F.groundRate) < 8 && F.nSamp >= 4;
    F.onGround = F.terrain && agl != null && agl <= wheelH + 0.6 && groundSettled;

    if (F.terrain && agl != null && agl < -60 && F.nSamp >= 4) { F.deep = (F.deep || 0) + dt; } else F.deep = 0;
    if (F.deep > 1.0) { F.crashed = true; flash('terrain impact - press R'); F.deep = 0; }

    if (F.onGround) {
      const vGround = Math.hypot(F.u, F.v);
      if (F.w > 0) {
        if (F.w > 5.0 && vGround > 25) { F.crashed = true; flash('hard landing - press R'); }
        F.w = 0;
      }
      Z = Math.min(Z, 0);
      // rolling friction + brakes
      const mu = 0.02 + F.brake * 0.45;
      X -= Math.sign(F.u) * mu * ac.m * G;
      // wheels resist sideslip, nosewheel steers with rudder
      F.v *= 0.80;
      if (vGround > 0.5) F.r += dr * 0.55 * clamp(vGround / 30, 0, 1) * dt * 10;
      // keep the aircraft on its wheels until it flies
      if (L < ac.m * G) {
        F.phi *= 0.85;
        F.theta = lerp(F.theta, clamp(F.theta, -1 * D2R, 12 * D2R), 0.3);
        F.p *= 0.6; F.q *= 0.6;
      }
      // rate-limited so a jumpy terrain estimate cannot teleport the aircraft
      const want = gh + wheelH;
      F.alt += clamp(want - F.alt, -2.5 * dt, 2.5 * dt);
    }

    // --- integrate rates
    const pd = (l + (ac.Iy - ac.Iz) * F.q * F.r) / ac.Ix;
    const qd = (mM + (ac.Iz - ac.Ix) * F.r * F.p) / ac.Iy;
    const rd = (n + (ac.Ix - ac.Iy) * F.p * F.q) / ac.Iz;
    F.p += pd * dt; F.q += qd * dt; F.r += rd * dt;

    // --- integrate body velocities
    const ud = X / ac.m - F.q * F.w + F.r * F.v;
    const vd = Y / ac.m - F.r * F.u + F.p * F.w;
    const wd = Z / ac.m - F.p * F.v + F.q * F.u;
    F.u += ud * dt; F.v += vd * dt; F.w += wd * dt;
    if (F.u < 1) F.u = 1;

    // --- integrate attitude
    const tth = Math.tan(clamp(F.theta, -85 * D2R, 85 * D2R));
    F.phi += (F.p + (F.q * sphi + F.r * cphi) * tth) * dt;
    F.theta += (F.q * cphi - F.r * sphi) * dt;
    F.psi += ((F.q * sphi + F.r * cphi) / Math.max(cth, 0.02)) * dt;
    F.theta = clamp(F.theta, -89 * D2R, 89 * D2R);
    F.phi = Math.atan2(Math.sin(F.phi), Math.cos(F.phi));
    F.psi = (F.psi % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI);

    // --- position from NED velocity (transpose of the body<-NED matrix)
    const vN = R11 * F.u + R21 * F.v + R31 * F.w;
    const vE = R12 * F.u + R22 * F.v + R32 * F.w;
    const vD = R13 * F.u + R23 * F.v + R33 * F.w;
    F.vsRaw = vD;
    F.vs = -vD;
    const rr = RE + F.alt;
    F.lat += (vN / rr) * R2D * dt;
    F.lon += (vE / (rr * Math.cos(F.lat * D2R))) * R2D * dt;
    if (!F.onGround) F.alt += -vD * dt;
    F.lat = clamp(F.lat, -84.9, 84.9);
    if (F.lon > 180) F.lon -= 360; if (F.lon < -180) F.lon += 360;

    // --- envelope flags
    F.over = F.ias > ac.vne || (ac.mmo && F.mach > ac.mmo);
    if (agl != null) {
      if (agl < 150 && F.vs < -12 && !F.onGround) F.gpws = 'SINK RATE';
      else if (agl < 120 && !F.onGround && F.gear < 0.5 && !ac.gearFixed) F.gpws = 'TOO LOW - GEAR';
      else if (agl < 90 && F.vs < -6 && !F.onGround) F.gpws = 'PULL UP';
      else F.gpws = '';
      if (agl < -25) { F.crashed = true; }
    }
  }

  // --------------------------------------------------------- terrain probing
  // Earth reports the terrain elevation under the mouse pointer to its
  // accessibility tree. We move a synthetic pointer near the bottom of the
  // screen and keep the samples whose reported lat/lon land near the aircraft.
  const samples = [];
  let lastProbe = 0, semOn = false, probeN = 0;
  function enableSemantics() {
    if (semOn) return;
    const ph = document.querySelector('flt-semantics-placeholder');
    if (ph) { ph.click(); ph.dispatchEvent(new MouseEvent('click', { bubbles: true })); semOn = true; }
  }
  function parseDMS(s) {
    const m = s.match(/(\d+)°(\d+)'([\d.]+)"([NSEW])/g);
    if (!m || m.length < 2) return null;
    const one = (t) => {
      const p = t.match(/(\d+)°(\d+)'([\d.]+)"([NSEW])/);
      let v = +p[1] + +p[2] / 60 + +p[3] / 3600;
      if (p[4] === 'S' || p[4] === 'W') v = -v;
      return v;
    };
    return [one(m[0]), one(m[1])];
  }
  function readSemantics() {
    const host = document.querySelector('flt-semantics-host');
    if (!host) return null;
    let elev = null, coord = null;
    for (const e of host.querySelectorAll('*')) {
      const t = (e.textContent || '').trim();
      if (!coord && /^Cursor latitude and longitude/i.test(t)) coord = t.split('\n').pop();
      else if (!elev && /^Cursor elevation relative to sea level/i.test(t)) elev = t.split('\n').pop();
    }
    if (!elev || !coord) return null;
    const ll = parseDMS(coord);
    const h = parseFloat(elev.replace(/[^\d.-]/g, ''));
    if (!ll || !isFinite(h)) return null;
    return { lat: ll[0], lon: ll[1], h };
  }
  function probeTick(now) {
    if (now - lastProbe < 150) return;
    lastProbe = now;
    enableSemantics();

    const s = readSemantics();
    if (s && isFinite(s.h)) {
      const key = s.lat.toFixed(3) + ',' + s.lon.toFixed(3);
      if (!samples.length || samples[samples.length - 1].key !== key) {
        samples.push({ key, lat: s.lat, lon: s.lon, h: s.h, t: now });
        if (samples.length > 400) samples.shift();
      }
    }

    // Flutter only re-picks the terrain on a real PointerEvent, so sweep a
    // synthetic pointer across the lower half of the screen. Whatever it lands
    // on goes into the cache with its own reported lat/lon, and the aircraft
    // picks up those samples as it flies over them.
    const gp = document.querySelector('flt-glass-pane') || document.getElementById('earth-canvas');
    if (!gp) return;
    probeN++;
    // aim low on the screen: that is the terrain closest to the aircraft
    const cols = 5;
    const cx = (0.28 + 0.11 * (probeN % cols)) * window.innerWidth;
    const cy = (0.74 + 0.07 * ((probeN / cols | 0) % 3)) * window.innerHeight;
    try {
      gp.dispatchEvent(new PointerEvent('pointermove', {
        bubbles: true, cancelable: true, composed: true,
        pointerId: 1, pointerType: 'mouse', isPrimary: true,
        clientX: cx, clientY: cy, buttons: 0,
      }));
    } catch (e) { /* ignore */ }
  }

  // Screen-picked samples occasionally miss the terrain and come back as 0 m, so
  // the estimate is the median of the nearest few rather than a mean - one bad
  // sample must never be able to yank the aircraft into the ground.
  function groundUpdate() {
    if (samples.length < 4) { F.ground = null; F.nSamp = samples.length; return; }
    const cosLat = Math.cos(F.lat * D2R);
    const radius = clamp(F.alt * 2.6, 1200, 30000);
    const near = [];
    let nearest = Infinity;
    for (const s of samples) {
      const dN = (s.lat - F.lat) * 111320;
      const dE = (s.lon - F.lon) * 111320 * cosLat;
      const d = Math.hypot(dN, dE);
      if (d < nearest) nearest = d;
      if (d < radius) near.push({ d, h: s.h });
    }
    F.nearest = Math.round(nearest);
    F.nSamp = near.length;
    if (near.length < 4) { if (nearest > radius * 2) F.ground = null; return; }
    near.sort((a, b) => a.d - b.d);
    const use = near.slice(0, 9).map((x) => x.h).sort((a, b) => a - b);
    const h = use.length % 2 ? use[(use.length - 1) / 2]
      : (use[use.length / 2 - 1] + use[use.length / 2]) / 2;
    const prev = F.ground;
    F.ground = prev == null ? h : lerp(prev, h, 0.12);
    F.groundRate = prev == null ? 0 : (F.ground - prev) * 200;
  }

  // ---------------------------------------------------------------- takeover
  function syncFromEarth() {
    const g = window.GEFT;
    if (!g) return false;
    F.lat = g.lat; F.lon = g.lon; F.alt = Math.max(g.alt, 150);
    F.psi = g.hdg * D2R; F.theta = clamp(g.pitch, -20, 20) * D2R; F.phi = 0;
    const v = Math.max(A().cruise * 0.514444, 30);
    F.u = v; F.v = 0; F.w = 0; F.p = F.q = F.r = 0;
    F.thr = A().kind === 'glider' ? 0 : 0.75;
    F.crashed = false;
    IN.pitch = IN.roll = IN.yaw = 0;
    CMD.pitch = CMD.roll = CMD.yaw = 0;
    F.ap.tgtHdg = wrap360(g.hdg); F.ap.tgtAlt = F.alt; F.ap.tgtSpd = A().cruise;
    return true;
  }
  function respawn() {
    const base = F.ground == null ? (F.alt - A().startAlt) : F.ground;
    F.alt = Math.max(base + A().startAlt, 400);
    F.u = Math.max(A().cruise * 0.514444, 30); F.v = 0; F.w = 0;
    F.p = F.q = F.r = 0; F.phi = 0; F.theta = 3 * D2R; F.psi = F.psi || 0;
    F.flap = 0; F.brake = 0;
    CMD.pitch = CMD.roll = CMD.yaw = 0;
    F.thr = A().kind === 'glider' ? 0 : 0.8;
    F.crashed = false; F.trim = 0; F.deep = 0;
    IN.pitch = IN.roll = IN.yaw = 0;
    flash('reset');
  }

  function writeCamera() {
    const core = window.GEFT_CORE;
    if (!core || core.idx == null) return false;
    return core.write({
      lat: F.lat, lon: F.lon, alt: F.alt,
      hdg: wrap360(F.psi * R2D),
      pitch: clamp(F.theta * R2D, -89, 89),
      roll: clamp(F.phi * R2D, -179, 179),
    });
  }

  // -------------------------------------------------------------------- loop
  // Driven by a timer, not requestAnimationFrame: Chrome throttles rAF hard when
  // the window is occluded, which silently starves the integrator and makes the
  // aircraft fly in slow motion. A fixed-step accumulator keeps handling identical
  // whatever the frame rate.
  let acc = 0, last = 0, timer = 0;
  const HZ = 200, H = 1 / HZ;
  function loop() {
    const now = performance.now();
    const dt = last ? Math.min((now - last) / 1000, 2.0) : 0.016;
    last = now;
    if (!F.on || F.paused) return;
    if (flashT && now > flashT) { F.msg = ''; flashT = 0; }
    if (F.crashed) { publish(); return; }     // frozen until R

    probeTick(now);
    const hasPad = readGamepad();

    acc = Math.min(acc + dt, 2.0);
    let guard = 0;
    let ctlAcc = 0;
    const CH = 1 / 50;                       // control laws at a fixed 50 Hz
    while (acc >= H && guard++ < 1200) {
      ctlAcc += H;
      if (ctlAcc >= CH) {
        if (!hasPad) keyAxes(ctlAcc);
        if (keys.KeyL) levelOff();
        autopilot(ctlAcc);
        rateLimit(ctlAcc);
        ctlAcc = 0;
      }
      step(H); acc -= H;
    }
    writeCamera();
    publish();
  }

  function publish() {
    window.GEFT_FLIGHT = {
      on: F.on, paused: F.paused, crashed: F.crashed, type: F.type, name: A().name,
      ias: F.ias, tas: F.tas * 1.94384, mach: F.mach, alt: F.alt, agl: F.agl,
      vs: F.vs, hdg: wrap360(F.psi * R2D), pitch: F.theta * R2D, roll: F.phi * R2D,
      alpha: F.alpha * R2D, beta: F.beta * R2D, nz: F.nz,
      thr: F.thr, flap: F.flap, gear: F.gear, brake: F.brake, trim: F.trim,
      stall: F.stall, over: F.over, onGround: F.onGround, gpws: F.gpws,
      ap: { ...F.ap }, wind: { ...F.wind }, msg: F.msg,
      hasGround: F.ground != null, terrain: F.terrain,
      simT, wallT: wallT0 ? (performance.now() - wallT0) / 1000 : 0,
      ground: F.ground, probe: { n: samples.length, near: F.nSamp, nearest: F.nearest },
      pad: pad ? (pad.id || 'gamepad').slice(0, 28) : null,
      vne: A().vne, vs0: A().vs, gLim: A().gLim,
    };
  }

  // ------------------------------------------------------------------ enable
  function enable() {
    if (!window.GEFT_CORE || window.GEFT_CORE.idx == null) { flash('no camera lock yet'); return; }
    if (!syncFromEarth()) { flash('no flight state yet'); return; }
    F.on = true; F.crashed = false;
    simT = 0; wallT0 = performance.now(); last = 0; acc = 0;
    flash('flight model engaged - ' + A().name);
  }
  function disable() { F.on = false; flash('Earth camera released'); }
  function toggle() { F.on ? disable() : enable(); }
  function cycleAircraft(dir) {
    const i = ORDER.indexOf(F.type);
    F.type = ORDER[(i + (dir || 1) + ORDER.length) % ORDER.length];
    localStorage.setItem('geft.aircraft', F.type);
    F.ap.tgtSpd = A().cruise;
    if (A().gearFixed) F.gear = 1;
    flash(A().name);
  }

  // ----------------------------------------------------------------- keyboard
  const OURS = new Set(['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'PageUp', 'PageDown',
    'KeyW', 'KeyA', 'KeyS', 'KeyD', 'KeyQ', 'KeyE', 'KeyB', 'KeyG', 'KeyF', 'KeyL',
    'Equal', 'Minus', 'Comma', 'Period', 'Digit1', 'Digit2', 'Digit3', 'Digit0', 'KeyT', 'KeyM', 'KeyR', 'KeyK']);
  function onDown(e) {
    const tag = (e.target && e.target.tagName) || '';
    if (tag === 'INPUT' || tag === 'TEXTAREA') return;
    if (e.code === 'KeyM' && !e.shiftKey && !e.metaKey && !e.ctrlKey) { e.preventDefault(); e.stopPropagation(); toggle(); return; }
    if (!F.on) return;
    if (!OURS.has(e.code)) return;
    e.preventDefault(); e.stopPropagation();
    keys[e.code] = true;
    if (e.repeat) return;
    switch (e.code) {
      case 'KeyG': toggleGear(); break;
      case 'KeyF':
        F.flap = e.shiftKey ? Math.max(0, F.flap - 0.5) : Math.min(1, F.flap + 0.5);
        flash('flaps ' + Math.round(F.flap * 100) + '%'); break;
      case 'KeyT': cycleAircraft(e.shiftKey ? -1 : 1); respawn(); break;
      case 'KeyR': respawn(); break;
      case 'KeyK':
        F.terrain = !F.terrain;
        localStorage.setItem('geft.terrain', F.terrain ? '1' : '0');
        flash('terrain collision ' + (F.terrain ? 'on' : 'off')); break;
      case 'Digit1': F.ap.on = true; F.ap.hdg = !F.ap.hdg; F.ap.tgtHdg = wrap360(F.psi * R2D); flash('AP HDG ' + (F.ap.hdg ? 'on' : 'off')); break;
      case 'Digit2': F.ap.on = true; F.ap.alt = !F.ap.alt; F.ap.tgtAlt = F.alt; flash('AP ALT ' + (F.ap.alt ? 'on' : 'off')); break;
      case 'Digit3': F.ap.on = true; F.ap.spd = !F.ap.spd; F.ap.tgtSpd = Math.round(F.ias); flash('AP SPD ' + (F.ap.spd ? 'on' : 'off')); break;
      case 'Digit0': F.ap.on = false; F.ap.hdg = F.ap.alt = F.ap.spd = false; flash('AP off'); break;
      default: break;
    }
  }
  function onUp(e) {
    if (!OURS.has(e.code)) return;
    keys[e.code] = false;
    if (F.on) { e.preventDefault(); e.stopPropagation(); }
    if (e.code === 'KeyB') F.brake = 0;
  }
  // stop Earth dragging the camera out from under us
  function swallow(e) {
    if (!F.on) return;
    const t = e.target;
    if (t && (t.id === 'earth-canvas' || (t.tagName && /FLT-|CANVAS/i.test(t.tagName)))) {
      e.stopPropagation();
    }
  }

  window.addEventListener('keydown', onDown, true);
  window.addEventListener('keyup', onUp, true);
  window.addEventListener('mousedown', swallow, true);
  window.addEventListener('wheel', swallow, { capture: true, passive: true });

  // Earth re-asserts its own camera between our writes, so push the camera on
  // every animation frame as well as every physics tick.
  let wraf = 0;
  const rafWrite = () => { wraf = requestAnimationFrame(rafWrite); if (F.on && !F.paused) writeCamera(); };
  wraf = requestAnimationFrame(rafWrite);
  timer = setInterval(loop, 10);
  publish();

  window.GEFT_SIM = {
    enable, disable, toggle, respawn, cycleAircraft,
    set: (k, v) => { F[k] = v; },
    state: () => F,
    aircraft: AIRCRAFT, order: ORDER,
  };
  window.__geftFlightCleanup = () => {
    clearInterval(timer);
    cancelAnimationFrame(wraf);
    window.removeEventListener('keydown', onDown, true);
    window.removeEventListener('keyup', onUp, true);
    window.removeEventListener('mousedown', swallow, true);
    window.removeEventListener('wheel', swallow, true);
    F.on = false;
    window.__geftFlightCleanup = null;
  };
})();
