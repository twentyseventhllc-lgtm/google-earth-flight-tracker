/* Google Earth Flight Tracker
 * Records the route you fly in the Google Earth web Flight Simulator and
 * exports it as KML / GPX / GeoJSON.
 *
 * How it reads your position:
 *   Google Earth web runs its C++ engine as multi-threaded WebAssembly on a
 *   SharedArrayBuffer. The flight simulator never writes the plane position to
 *   the URL, the DOM, or any JS API. But the engine's camera struct
 *   (lon, lat, altitude, heading, tilt, roll) lives as consecutive float64s in
 *   that shared heap. We grab a handle on the heap from a WebGL upload call,
 *   then find the camera struct with a 3-pass scan (plausible field ranges ->
 *   values that change -> values that change consistently), and poll it.
 */
(function () {
  'use strict';
  if (window.__geftCleanup) { try { window.__geftCleanup(); } catch (e) { /* ignore */ } }
  if (window.__geFlightTracker) return;
  window.__geFlightTracker = true;

  // ------------------------------------------------------------------ config
  const CFG = {
    pollMs: 150,          // how often we read the camera
    minMoveM: 20,         // record a point after this much movement
    maxGapMs: 2000,       // ...or at least this often
    minTurnDeg: 4,        // ...or after this much course change
    autosaveMs: 4000,
    jumpKm: 40,           // bigger single-step jump => teleport / crash-restart
  };
  const LS_FLIGHTS = 'geft.flights.v1';
  const LS_ACTIVE = 'geft.active.v1';

  // ------------------------------------------------------------------- state
  let heap = null, f64 = null, camIdx = null;
  let locking = false, lastRead = null, badReads = 0;
  let recording = false, paused = false;
  let active = null;      // { id, name, started, points:[], segments:[] }
  let lastPoint = null;

  // --------------------------------------------------------------- geo utils
  const R = 6371008.8;
  const rad = (d) => (d * Math.PI) / 180;
  function haversine(a, b) {
    const dLat = rad(b.lat - a.lat), dLon = rad(b.lon - a.lon);
    const s = Math.sin(dLat / 2) ** 2 +
      Math.cos(rad(a.lat)) * Math.cos(rad(b.lat)) * Math.sin(dLon / 2) ** 2;
    return 2 * R * Math.asin(Math.min(1, Math.sqrt(s)));
  }
  function dist3d(a, b) {
    const g = haversine(a, b), dz = (b.alt - a.alt);
    return Math.sqrt(g * g + dz * dz);
  }
  function bearing(a, b) {
    const y = Math.sin(rad(b.lon - a.lon)) * Math.cos(rad(b.lat));
    const x = Math.cos(rad(a.lat)) * Math.sin(rad(b.lat)) -
      Math.sin(rad(a.lat)) * Math.cos(rad(b.lat)) * Math.cos(rad(b.lon - a.lon));
    return (Math.atan2(y, x) * 180 / Math.PI + 360) % 360;
  }
  const angDiff = (a, b) => Math.abs(((a - b + 540) % 360) - 180);

  // ------------------------------------------------------- heap + camera lock
  function captureHeap(ms = 2000) {
    return new Promise((resolve) => {
      const protos = [WebGLRenderingContext.prototype];
      if (window.WebGL2RenderingContext) protos.push(WebGL2RenderingContext.prototype);
      const names = ['bufferData', 'bufferSubData', 'texImage2D', 'texSubImage2D', 'compressedTexImage2D'];
      const saved = [];
      let best = null;
      for (const P of protos) {
        for (const fn of names) {
          const orig = P[fn];
          if (typeof orig !== 'function') continue;
          saved.push([P, fn, orig]);
          P[fn] = function (...a) {
            for (const x of a) {
              try {
                const b = x && x.buffer;
                if (b && b.byteLength > 8e6 && b.byteLength > (best ? best.byteLength : 0)) best = b;
              } catch (e) { /* ignore */ }
            }
            return orig.apply(this, a);
          };
        }
      }
      setTimeout(() => {
        for (const [P, fn, orig] of saved) P[fn] = orig;
        resolve(best);
      }, ms);
    });
  }

  const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

  function plausible(i) {
    const lon = f64[i - 1], lat = f64[i], alt = f64[i + 1];
    if (!(lat > -85 && lat < 85) || Math.abs(lat) < 1e-4) return false;
    if (!(lon > -180 && lon < 180) || Math.abs(lon) < 1e-4) return false;
    if (Math.hypot(lat, lon) < 0.05) return false; // null-island scratch values
    if (!(alt > 1 && alt < 2e7)) return false;
    const hd = f64[i + 2], tl = f64[i + 3], rl = f64[i + 4];
    if (!(hd >= -360 && hd <= 360)) return false;
    if (!(tl >= -180 && tl <= 180)) return false;
    if (!(rl >= -180 && rl <= 180)) return false;
    return true;
  }

  async function lockCamera() {
    if (locking) return false;
    locking = true;
    try {
      if (!heap) {
        setStatus('looking for the engine…');
        for (let a = 0; a < 6 && !heap; a++) heap = await captureHeap(2500);
        if (!heap) {
          setStatus('engine not found — move the view, then Re-lock');
          setTimeout(() => { if (!camIdx) lockCamera(); }, 8000);
          return false;
        }
        f64 = new Float64Array(heap);
      }
      setStatus('locating the aircraft…');
      const n = f64.length;
      const c = [];
      for (let i = 1; i < n - 5; i++) { if (plausible(i)) { c.push(i); if (c.length > 3e6) break; } }
      if (!c.length) {
        setStatus('waiting for a moving aircraft…');
        setTimeout(() => { if (!camIdx) lockCamera(); }, 6000);
        return false;
      }

      // Sample every candidate a few times: the real camera moves smoothly at a
      // plausible flight speed. Most decoys either sit still or jump around.
      const SAMPLES = 5, DT = 0.3;
      const trk = [];
      for (let s = 0; s < SAMPLES; s++) {
        trk.push(c.map((i) => [f64[i - 1], f64[i]]));
        if (s < SAMPLES - 1) await sleep(DT * 1000);
      }
      const survivors = [];
      for (let k = 0; k < c.length; k++) {
        const i = c[k];
        if (!plausible(i)) continue;
        let ok = true; const sp = [];
        for (let s = 1; s < SAMPLES; s++) {
          const dla = trk[s][k][1] - trk[s - 1][k][1];
          const dlo = trk[s][k][0] - trk[s - 1][k][0];
          if (dla === 0 && dlo === 0) { ok = false; break; }
          if (Math.abs(dla) > 0.25 || Math.abs(dlo) > 0.25) { ok = false; break; }
          const m = Math.hypot(dla * 111320, dlo * 111320 * Math.cos(trk[s][k][1] * Math.PI / 180));
          sp.push((m / DT) * 3.6); // km/h
        }
        if (!ok) continue;
        const mn = Math.min(...sp), mx = Math.max(...sp);
        if (mn < 20 || mx > 15000) continue;      // not a flying aircraft
        if (mx / Math.max(mn, 1e-6) > 4) continue; // erratic => not the camera
        survivors.push({ i, mn, mx, alt: f64[i + 1], tilt: f64[i + 3] });
      }
      if (!survivors.length) {
        setStatus('waiting for a moving aircraft…');
        setTimeout(() => { if (!camIdx) lockCamera(); }, 6000);
        return false;
      }

      // The engine keeps several identical copies of the camera struct.
      // Group by value, then score.
      const groups = new Map();
      for (const s of survivors) {
        const key = f64[s.i].toFixed(5) + ',' + f64[s.i - 1].toFixed(5) + ',' + Math.round(s.alt);
        const g = groups.get(key) || { n: 0, best: s };
        g.n++; groups.set(key, g);
      }
      let win = null;
      for (const g of groups.values()) {
        const s = g.best;
        const at = Math.abs(s.tilt);
        let sc = g.n * 10;
        if (at > 20 && at < 160) sc += 25;       // flight-sim camera sits near the horizon
        if (s.alt > 200) sc += 12;
        if (s.mx / Math.max(s.mn, 1e-6) < 2.5) sc += 15;
        if (s.mn > 50 && s.mx < 8000) sc += 15;
        if (Math.abs(f64[s.i]) < 1 && Math.abs(f64[s.i - 1]) < 2) sc -= 30; // null-island noise
        if (!win || sc > win.sc) win = { sc, idx: s.i };
      }
      camIdx = win.idx;
      badReads = 0;
      setStatus(`locked on · ${f64[camIdx].toFixed(3)}, ${f64[camIdx - 1].toFixed(3)}`);
      return true;
    } finally { locking = false; }
  }

  function readCam() {
    if (camIdx == null || !f64) return null;
    const i = camIdx;
    const lat = f64[i], lon = f64[i - 1], alt = f64[i + 1];
    if (!isFinite(lat) || !isFinite(lon) || !isFinite(alt)) return null;
    if (!(lat > -85 && lat < 85 && lon > -180 && lon < 180 && alt > -500 && alt < 2e7)) return null;
    // the engine zeroes / parks the struct while the sim is paused or crashed
    if (lat === 0 || (alt === 0 && Number.isInteger(lon))) return null;
    if (Math.hypot(lat, lon) < 0.05) return null;
    return { lat, lon, alt, tilt: f64[i + 3], roll: f64[i + 4], t: Date.now() };
  }

  // ----------------------------------------------------------- flight storage
  const loadFlights = () => { try { return JSON.parse(localStorage.getItem(LS_FLIGHTS)) || []; } catch (e) { return []; } };
  const saveFlights = (f) => localStorage.setItem(LS_FLIGHTS, JSON.stringify(f));

  function newFlight() {
    const d = new Date();
    return {
      id: 'f' + d.getTime(),
      name: 'Flight ' + d.toLocaleDateString() + ' ' + d.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }),
      started: d.getTime(), ended: null, points: [],
    };
  }
  function persistActive() { if (active) localStorage.setItem(LS_ACTIVE, JSON.stringify(active)); }

  function stats(f) {
    const p = f.points;
    if (p.length < 2) return { dist: 0, dur: 0, maxAlt: p[0] ? p[0].alt : 0, maxSpd: 0, avgSpd: 0, pts: p.length };
    let dist = 0, maxAlt = -1e9, maxSpd = 0;
    for (let i = 1; i < p.length; i++) {
      if (p[i].brk) { maxAlt = Math.max(maxAlt, p[i].alt); continue; }
      const d = dist3d(p[i - 1], p[i]);
      const dt = (p[i].t - p[i - 1].t) / 1000;
      if (d < CFG.jumpKm * 1000) dist += d;
      if (dt > 0 && d < CFG.jumpKm * 1000) maxSpd = Math.max(maxSpd, d / dt);
      maxAlt = Math.max(maxAlt, p[i].alt);
    }
    maxAlt = Math.max(maxAlt, p[0].alt);
    const dur = (p[p.length - 1].t - p[0].t) / 1000;
    return { dist, dur, maxAlt, maxSpd, avgSpd: dur > 0 ? dist / dur : 0, pts: p.length };
  }

  // ------------------------------------------------------------------ exports
  const xmlEsc = (s) => String(s).replace(/[<>&'"]/g, (c) => ({ '<': '&lt;', '>': '&gt;', '&': '&amp;', "'": '&apos;', '"': '&quot;' }[c]));
  const iso = (t) => new Date(t).toISOString().replace(/\.\d{3}Z$/, 'Z');

  function toKML(f) {
    const p = f.points;
    const s = stats(f);
    const legs = [];
    let leg = [];
    for (const q of p) { if (q.brk && leg.length) { legs.push(leg); leg = []; } leg.push(q); }
    if (leg.length) legs.push(leg);
    const legXml = legs.filter((l) => l.length > 1).map((l, k) => `  <Placemark>
    <name>${legs.length > 1 ? 'Leg ' + (k + 1) : 'Route'}</name>
    <styleUrl>#routeLine</styleUrl>
    <LineString>
      <extrude>1</extrude>
      <tessellate>1</tessellate>
      <altitudeMode>absolute</altitudeMode>
      <coordinates>${l.map((q) => `${q.lon.toFixed(7)},${q.lat.toFixed(7)},${q.alt.toFixed(1)}`).join(' ')}</coordinates>
    </LineString>
  </Placemark>`).join('\n');
    const when = p.map((q) => `      <when>${iso(q.t)}</when>`).join('\n');
    const gx = p.map((q) => `      <gx:coord>${q.lon.toFixed(7)} ${q.lat.toFixed(7)} ${q.alt.toFixed(1)}</gx:coord>`).join('\n');
    const first = p[0], last = p[p.length - 1];
    return `<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:gx="http://www.google.com/kml/ext/2.2">
<Document>
  <name>${xmlEsc(f.name)}</name>
  <description><![CDATA[Distance ${(s.dist / 1000).toFixed(1)} km · Duration ${fmtDur(s.dur)} · Max altitude ${Math.round(s.maxAlt)} m · Top speed ${Math.round(s.maxSpd * 3.6)} km/h · ${s.pts} track points. Recorded in the Google Earth flight simulator.]]></description>
  <Style id="routeLine">
    <LineStyle><color>ff2fd2ff</color><width>3</width></LineStyle>
    <PolyStyle><color>332fd2ff</color></PolyStyle>
  </Style>
  <Style id="ptStart"><IconStyle><color>ff4fd35a</color><scale>1.1</scale>
    <Icon><href>http://maps.google.com/mapfiles/kml/shapes/airports.png</href></Icon></IconStyle></Style>
  <Style id="ptEnd"><IconStyle><color>ff4f4fff</color><scale>1.1</scale>
    <Icon><href>http://maps.google.com/mapfiles/kml/shapes/flag.png</href></Icon></IconStyle></Style>
${legXml}
  <Placemark>
    <name>Takeoff</name><styleUrl>#ptStart</styleUrl>
    <Point><altitudeMode>absolute</altitudeMode><coordinates>${first.lon},${first.lat},${first.alt.toFixed(1)}</coordinates></Point>
  </Placemark>
  <Placemark>
    <name>End</name><styleUrl>#ptEnd</styleUrl>
    <Point><altitudeMode>absolute</altitudeMode><coordinates>${last.lon},${last.lat},${last.alt.toFixed(1)}</coordinates></Point>
  </Placemark>
  <Folder>
    <name>Animated track</name>
    <Placemark>
      <name>${xmlEsc(f.name)} (playback)</name>
      <styleUrl>#routeLine</styleUrl>
      <gx:Track>
        <altitudeMode>absolute</altitudeMode>
${when}
${gx}
      </gx:Track>
    </Placemark>
  </Folder>
</Document>
</kml>`;
  }

  function toGPX(f) {
    const pts = f.points.map((q) =>
      `      <trkpt lat="${q.lat.toFixed(7)}" lon="${q.lon.toFixed(7)}"><ele>${q.alt.toFixed(1)}</ele><time>${iso(q.t)}</time></trkpt>`
    ).join('\n');
    return `<?xml version="1.0" encoding="UTF-8"?>
<gpx version="1.1" creator="Google Earth Flight Tracker" xmlns="http://www.topografix.com/GPX/1/1">
  <metadata><name>${xmlEsc(f.name)}</name><time>${iso(f.started)}</time></metadata>
  <trk><name>${xmlEsc(f.name)}</name><trkseg>
${pts}
  </trkseg></trk>
</gpx>`;
  }

  function toGeoJSON(f) {
    const s = stats(f);
    return JSON.stringify({
      type: 'Feature',
      properties: {
        name: f.name, started: iso(f.started),
        distance_km: +(s.dist / 1000).toFixed(3), duration_s: Math.round(s.dur),
        max_alt_m: Math.round(s.maxAlt), max_speed_kmh: Math.round(s.maxSpd * 3.6), points: s.pts,
        times: f.points.map((q) => iso(q.t)),
      },
      geometry: { type: 'LineString', coordinates: f.points.map((q) => [+q.lon.toFixed(7), +q.lat.toFixed(7), +q.alt.toFixed(1)]) },
    }, null, 1);
  }

  function download(name, text, mime) {
    const b = new Blob([text], { type: mime });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(b); a.download = name;
    document.body.appendChild(a); a.click();
    setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 1000);
  }
  const slug = (s) => s.replace(/[^a-z0-9]+/gi, '-').replace(/^-|-$/g, '').toLowerCase();

  // --------------------------------------------------------------------- loop
  function tick() {
    const fix = readCam();
    if (!fix) {
      badReads++;
      if (badReads > 20 && !locking) { camIdx = null; lockCamera(); }
      return;
    }
    if (lastRead && (Math.abs(fix.lat - lastRead.lat) > 5 || Math.abs(fix.lon - lastRead.lon) > 5)) {
      // struct probably moved in memory -> re-lock
      badReads += 10;
      if (badReads > 20 && !locking) { camIdx = null; lockCamera(); return; }
    } else badReads = 0;
    lastRead = fix;
    paintHud(fix);

    if (!recording || paused || !active) return;
    const pts = active.points;
    if (!pts.length) { pts.push(fix); lastPoint = fix; persistActive(); return; }
    const prev = pts[pts.length - 1];
    const d = dist3d(prev, fix);
    const dt = fix.t - prev.t;
    if (d > CFG.jumpKm * 1000) {      // crash-restart or teleport: start a new leg
      fix.brk = true; pts.push(fix); lastPoint = fix; persistActive(); return;
    }
    let push = false;
    if (d >= CFG.minMoveM) push = true;
    else if (dt >= CFG.maxGapMs && d > 1) push = true;
    if (!push && pts.length > 1) {
      const b1 = bearing(pts[pts.length - 2], prev);
      const b2 = bearing(prev, fix);
      if (d > 5 && angDiff(b1, b2) >= CFG.minTurnDeg) push = true;
    }
    if (push) { pts.push(fix); lastPoint = fix; }
  }

  // ----------------------------------------------------------------------- UI
  const css = `
  #geft{position:fixed;top:64px;right:16px;z-index:2147483600;width:268px;
    font:12px/1.45 -apple-system,BlinkMacSystemFont,"SF Pro Text",Inter,system-ui,sans-serif;
    color:#e8eaed;background:rgba(18,20,23,.88);backdrop-filter:blur(18px) saturate(140%);
    -webkit-backdrop-filter:blur(18px) saturate(140%);
    border:1px solid rgba(255,255,255,.09);border-radius:12px;
    box-shadow:0 10px 34px rgba(0,0,0,.45);overflow:hidden;user-select:none}
  #geft .hd{display:flex;align-items:center;gap:8px;padding:9px 11px;cursor:move;
    border-bottom:1px solid rgba(255,255,255,.07)}
  #geft .dot{width:7px;height:7px;border-radius:50%;background:#5f6368;flex:none;transition:.2s}
  #geft.rec .dot{background:#ff5f56;box-shadow:0 0 0 3px rgba(255,95,86,.18);animation:geftp 1.4s infinite}
  @keyframes geftp{50%{opacity:.35}}
  #geft .ttl{font-weight:600;letter-spacing:.2px;font-size:12px;flex:1}
  #geft .min{cursor:pointer;opacity:.5;padding:0 3px;font-size:14px;line-height:1}
  #geft .min:hover{opacity:1}
  #geft .bd{padding:10px 11px 11px}
  #geft.col .bd{display:none}
  #geft .st{font-size:10.5px;color:#9aa0a6;margin-bottom:9px;min-height:14px;letter-spacing:.2px}
  #geft .grid{display:grid;grid-template-columns:1fr 1fr;gap:7px 10px;margin-bottom:10px}
  #geft .k{font-size:9.5px;text-transform:uppercase;letter-spacing:.7px;color:#80868b}
  #geft .v{font-size:13px;font-weight:600;font-variant-numeric:tabular-nums;
    font-family:ui-monospace,SFMono-Regular,Menlo,monospace}
  #geft svg.map{width:100%;height:92px;display:block;background:rgba(255,255,255,.035);
    border:1px solid rgba(255,255,255,.06);border-radius:7px;margin-bottom:10px}
  #geft .row{display:flex;gap:6px;margin-bottom:6px}
  #geft button{flex:1;appearance:none;border:1px solid rgba(255,255,255,.1);
    background:rgba(255,255,255,.055);color:#e8eaed;border-radius:7px;padding:6px 4px;
    font:600 11px/1 inherit;cursor:pointer;transition:.13s;letter-spacing:.2px}
  #geft button:hover{background:rgba(255,255,255,.12)}
  #geft button.pri{background:#ff5f56;border-color:transparent;color:#fff}
  #geft button.pri:hover{background:#ff7169}
  #geft button.on{background:#2fd2ff;border-color:transparent;color:#07222b}
  #geft .sep{height:1px;background:rgba(255,255,255,.07);margin:9px 0}
  #geft .lst{max-height:150px;overflow:auto;margin-top:2px}
  #geft .it{display:flex;align-items:center;gap:6px;padding:5px 6px;border-radius:6px;font-size:11px}
  #geft .it:hover{background:rgba(255,255,255,.06)}
  #geft .it .nm{flex:1;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
  #geft .it .mi{font-size:9.5px;color:#80868b;font-variant-numeric:tabular-nums}
  #geft .it i{cursor:pointer;opacity:.55;font-style:normal;font-size:11px;padding:0 2px}
  #geft .it i:hover{opacity:1}
  #geft .hint{font-size:9.5px;color:#6e7378;margin-top:7px;line-height:1.4}`;

  let el, elStatus, elMap, elList, elBtnRec, elBtnPause;

  function buildUI() {
    const st = document.createElement('style'); st.textContent = css; document.head.appendChild(st);
    el = document.createElement('div');
    el.id = 'geft';
    el.innerHTML = `
      <div class="hd"><span class="dot"></span><span class="ttl">Flight Tracker</span><span class="min">–</span></div>
      <div class="bd">
        <div class="st">starting…</div>
        <div class="grid">
          <div><div class="k">Latitude</div><div class="v" id="geft-lat">–</div></div>
          <div><div class="k">Longitude</div><div class="v" id="geft-lon">–</div></div>
          <div><div class="k">Altitude</div><div class="v" id="geft-alt">–</div></div>
          <div><div class="k">Speed</div><div class="v" id="geft-spd">–</div></div>
          <div><div class="k">Heading</div><div class="v" id="geft-hdg">–</div></div>
          <div><div class="k">Distance</div><div class="v" id="geft-dst">–</div></div>
        </div>
        <svg class="map" viewBox="0 0 240 92" preserveAspectRatio="none"><path id="geft-path" fill="none" stroke="#2fd2ff" stroke-width="1.6" stroke-linejoin="round" stroke-linecap="round"/><circle id="geft-cur" r="2.6" fill="#ff5f56"/></svg>
        <div class="row">
          <button class="pri" id="geft-rec">● Record</button>
          <button id="geft-pause">Pause</button>
        </div>
        <div class="row">
          <button id="geft-kml">KML</button>
          <button id="geft-gpx">GPX</button>
          <button id="geft-json">GeoJSON</button>
        </div>
        <div class="row">
          <button id="geft-relock">Re-lock aircraft</button>
        </div>
        <div class="sep"></div>
        <div class="k">Logbook</div>
        <div class="lst" id="geft-list"></div>
        <div class="hint">Shift+R record/stop · Shift+P pause · drag header to move. Import the KML back into Earth via Data Layers → Import to see the route in 3D.</div>
      </div>`;
    document.body.appendChild(el);
    elStatus = el.querySelector('.st');
    elMap = el.querySelector('#geft-path');
    elList = el.querySelector('#geft-list');
    elBtnRec = el.querySelector('#geft-rec');
    elBtnPause = el.querySelector('#geft-pause');

    el.querySelector('.min').onclick = () => el.classList.toggle('col');
    elBtnRec.onclick = () => (recording ? stopRec() : startRec());
    elBtnPause.onclick = togglePause;
    el.querySelector('#geft-kml').onclick = () => exportCur('kml');
    el.querySelector('#geft-gpx').onclick = () => exportCur('gpx');
    el.querySelector('#geft-json').onclick = () => exportCur('json');
    el.querySelector('#geft-relock').onclick = () => { camIdx = null; lastRead = null; lockCamera(); };

    // keep keystrokes inside the panel away from the plane
    el.addEventListener('keydown', (e) => e.stopPropagation(), true);

    // drag
    const hd = el.querySelector('.hd');
    let dx = 0, dy = 0, drag = false;
    hd.addEventListener('mousedown', (e) => {
      drag = true; const r = el.getBoundingClientRect();
      dx = e.clientX - r.left; dy = e.clientY - r.top;
      el.style.right = 'auto'; e.preventDefault();
    });
    window.addEventListener('mousemove', (e) => {
      if (!drag) return;
      el.style.left = Math.max(0, e.clientX - dx) + 'px';
      el.style.top = Math.max(0, e.clientY - dy) + 'px';
    });
    window.addEventListener('mouseup', () => { drag = false; });

    window.addEventListener('keydown', (e) => {
      if (!e.shiftKey || e.metaKey || e.ctrlKey || e.altKey) return;
      if (e.code === 'KeyR') { e.preventDefault(); recording ? stopRec() : startRec(); }
      if (e.code === 'KeyP') { e.preventDefault(); togglePause(); }
    }, true);
  }

  const setStatus = (s) => { if (elStatus) elStatus.textContent = s; };
  function fmtDur(s) {
    s = Math.round(s); const h = (s / 3600) | 0, m = ((s % 3600) / 60) | 0, x = s % 60;
    return h ? `${h}h ${m}m` : m ? `${m}m ${x}s` : `${x}s`;
  }
  const dms = (v, ns) => {
    const h = v < 0 ? ns[1] : ns[0]; v = Math.abs(v);
    const d = v | 0, m = ((v - d) * 60) | 0, s = ((v - d - m / 60) * 3600).toFixed(1);
    return `${d}°${String(m).padStart(2, '0')}'${s}"${h}`;
  };

  function paintHud(fix) {
    el.querySelector('#geft-lat').textContent = fix.lat.toFixed(4) + '°';
    el.querySelector('#geft-lon').textContent = fix.lon.toFixed(4) + '°';
    el.querySelector('#geft-alt').textContent = fix.alt > 9999
      ? (fix.alt / 1000).toFixed(1) + ' km' : Math.round(fix.alt) + ' m';
    let spd = 0, hdg = null;
    if (lastRead2 && fix.t > lastRead2.t) {
      const d = dist3d(lastRead2, fix), dt = (fix.t - lastRead2.t) / 1000;
      if (d < CFG.jumpKm * 1000) { spd = d / dt; if (d > 2) hdg = bearing(lastRead2, fix); }
    }
    if (hdg != null) lastHdg = hdg;
    smoothSpd = smoothSpd == null ? spd : smoothSpd * 0.75 + spd * 0.25;
    el.querySelector('#geft-spd').textContent = Math.round((smoothSpd || 0) * 3.6) + ' km/h';
    el.querySelector('#geft-hdg').textContent = lastHdg == null ? '–' : Math.round(lastHdg) + '°';
    const s = active ? stats(active) : null;
    el.querySelector('#geft-dst').textContent = s ? (s.dist / 1000).toFixed(1) + ' km' : '–';
    lastRead2 = fix;
    drawMap(fix);
  }
  let lastRead2 = null, smoothSpd = null, lastHdg = null;

  function drawMap(cur) {
    const p = active ? active.points : [];
    if (p.length < 2) { elMap.setAttribute('d', ''); return; }
    let minx = 1e9, maxx = -1e9, miny = 1e9, maxy = -1e9;
    const proj = p.map((q) => {
      // eslint-disable-next-line no-unused-expressions
      const x = q.lon * Math.cos(rad(q.lat)), y = -q.lat;
      minx = Math.min(minx, x); maxx = Math.max(maxx, x);
      miny = Math.min(miny, y); maxy = Math.max(maxy, y);
      return [x, y];
    });
    const w = Math.max(maxx - minx, 1e-6), h = Math.max(maxy - miny, 1e-6);
    const sc = Math.min(224 / w, 76 / h);
    const ox = 120 - ((minx + maxx) / 2) * sc, oy = 46 - ((miny + maxy) / 2) * sc;
    const d = proj.map(([x, y], i) => ((i && !p[i].brk) ? 'L' : 'M') + (x * sc + ox).toFixed(1) + ' ' + (y * sc + oy).toFixed(1)).join(' ');
    elMap.setAttribute('d', d);
    const last = proj[proj.length - 1];
    const c = el.querySelector('#geft-cur');
    c.setAttribute('cx', (last[0] * sc + ox).toFixed(1));
    c.setAttribute('cy', (last[1] * sc + oy).toFixed(1));
  }

  function renderList() {
    const fl = loadFlights();
    if (!fl.length) { elList.innerHTML = '<div class="hint">No saved flights yet.</div>'; return; }
    elList.innerHTML = '';
    fl.slice().reverse().forEach((f) => {
      const s = stats(f);
      const d = document.createElement('div');
      d.className = 'it';
      d.innerHTML = `<span class="nm">${f.name.replace(/^Flight /, '')}</span>
        <span class="mi">${(s.dist / 1000).toFixed(0)}km</span>
        <i title="Export KML" data-a="k">⭳</i><i title="Delete" data-a="d">✕</i>`;
      d.querySelector('[data-a="k"]').onclick = () => download(slug(f.name) + '.kml', toKML(f), 'application/vnd.google-earth.kml+xml');
      d.querySelector('[data-a="d"]').onclick = () => {
        saveFlights(loadFlights().filter((x) => x.id !== f.id)); renderList();
      };
      elList.appendChild(d);
    });
  }

  function startRec() {
    active = newFlight(); recording = true; paused = false;
    el.classList.add('rec');
    elBtnRec.textContent = '■ Stop'; elBtnRec.classList.remove('pri');
    setStatus('recording');
  }
  function stopRec() {
    recording = false; paused = false; el.classList.remove('rec');
    elBtnRec.textContent = '● Record'; elBtnRec.classList.add('pri');
    elBtnPause.classList.remove('on'); elBtnPause.textContent = 'Pause';
    if (active && active.points.length > 1) {
      active.ended = Date.now();
      const fl = loadFlights(); fl.push(active); saveFlights(fl);
      const s = stats(active);
      setStatus(`saved · ${(s.dist / 1000).toFixed(1)} km in ${fmtDur(s.dur)}`);
      renderList();
    } else setStatus('nothing recorded');
    localStorage.removeItem(LS_ACTIVE);
  }
  function togglePause() {
    if (!recording) return;
    paused = !paused;
    elBtnPause.classList.toggle('on', paused);
    elBtnPause.textContent = paused ? 'Resume' : 'Pause';
    setStatus(paused ? 'paused' : 'recording');
  }
  function exportCur(kind) {
    let f = active && active.points.length > 1 ? active : null;
    if (!f) { const fl = loadFlights(); f = fl[fl.length - 1]; }
    if (!f || f.points.length < 2) { setStatus('no track to export yet'); return; }
    if (kind === 'kml') download(slug(f.name) + '.kml', toKML(f), 'application/vnd.google-earth.kml+xml');
    if (kind === 'gpx') download(slug(f.name) + '.gpx', toGPX(f), 'application/gpx+xml');
    if (kind === 'json') download(slug(f.name) + '.geojson', toGeoJSON(f), 'application/geo+json');
    setStatus('exported ' + kind.toUpperCase());
  }

  // --------------------------------------------------------------------- boot
  buildUI();
  renderList();
  // restore an interrupted recording
  try {
    const a = JSON.parse(localStorage.getItem(LS_ACTIVE));
    if (a && a.points && a.points.length > 1) {
      active = a; recording = true; el.classList.add('rec');
      elBtnRec.textContent = '■ Stop'; elBtnRec.classList.remove('pri');
      setStatus('resumed previous recording');
    }
  } catch (e) { /* ignore */ }

  lockCamera();
  const iv1 = setInterval(tick, CFG.pollMs);
  const iv2 = setInterval(persistActive, CFG.autosaveMs);
  window.__geftCleanup = () => {
    clearInterval(iv1); clearInterval(iv2);
    const e = document.getElementById('geft'); if (e) e.remove();
    window.__geFlightTracker = false; window.__geftCleanup = null;
  };
  window.__geftRelock = lockCamera;
  window.__geftExport = { toKML, toGPX, toGeoJSON, stats, loadFlights };
  window.__geftState = () => ({ camIdx, recording, points: active ? active.points.length : 0 });
})();
