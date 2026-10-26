// Aerodynamics: body panels, quick estimate, and a 2D lattice-Boltzmann wind tunnel.
//
// The tunnel is a D2Q9 LBM (BGK + Smagorinsky LES) of the car's side section in a
// rolling-road tunnel. Forces come from momentum exchange on every solid link and are
// attributed to the part that owns the cell (each body panel, each wing, cockpit, engine).
// 2D section forces are converted to whole-car coefficient·area with a 3D relief factor.
(function (GT) {
  'use strict';
  const clamp = (v, a, b) => (v < a ? a : v > b ? b : v);

  GT.CAR_WIDTH = 1.8;     // m, used to turn 2D section forces into a 3D car
  GT.PANEL_SPAN = 1.6;    // m of skin each side-view panel stands for (mass)
  GT.WHEEL_CDA = 0.12;    // m² per axle pair: wheels are not part of the 2D section
  // 2D section → 3D car. Bodies: air escapes round the sides, so drag and especially lift relax.
  // Wings: nearly 2D, but the tunnel runs at low Reynolds number, so lift is boosted and drag cut
  // to land on real-world lift/drag ratios.
  GT.K3D = { body: { x: 0.2, y: 0.07 }, wing: { x: 0.47, y: 2.0 } };

  GT.PANEL_MATS = {
    grp:    { name: 'Fibreglass (GRP) 3 mm',    short: 'GRP', areal: 5.5, costM2: 60,  EA: 3e6, tear: 6000,  color: '#e9edf1' },
    alu:    { name: 'Aluminium sheet 1.2 mm',   short: 'Alu', areal: 3.3, costM2: 90,  EA: 6e6, tear: 9000,  color: '#c3cad2' },
    carbon: { name: 'Carbon prepreg 1.5 mm',    short: 'CF',  areal: 2.2, costM2: 600, EA: 9e6, tear: 14000, color: '#3a3f47' },
  };

  GT.panelProps = function (nm, p) {
    const A = nm.get(p.a), B = nm.get(p.b); if (!A || !B) return null;
    const m = GT.PANEL_MATS[p.mat] || GT.PANEL_MATS.grp;
    const L = Math.hypot(B.x - A.x, B.y - A.y), area = L * GT.PANEL_SPAN;
    return { L, area, mass: area * m.areal, cost: area * m.costM2, k: m.EA / Math.max(L, 0.05), tear: m.tear, mat: m };
  };

  // Wing / splitter geometry in local coords (y up, car faces +x), relative to the mount node.
  GT.wingShape = function (spec) {
    if (spec.kind === 'splitter') return { strut: null, line: [[-0.05, -0.03], [0.18, -0.045], [0.42, -0.05]], thick: 0.02 };
    const big = spec.ClA > 1.2 ? 1.25 : spec.ClA < 0.4 ? 0.6 : 1;
    const h = spec.ClA < 0.4 ? 0.08 : 0.5 * big, ch = 0.5 * big, aoa = -0.16 - (spec.ClA > 1.2 ? 0.08 : 0);
    const cx = -0.1, cy = h + 0.02, ca = Math.cos(aoa), sa = Math.sin(aoa), line = [];
    for (let i = 0; i <= 8; i++) {
      const t = i / 8, x = (t - 0.5) * ch, y = -0.09 * ch * 4 * t * (1 - t);   // camber bulges DOWN: inverted airfoil
      line.push([cx + x * ca - y * sa, cy + x * sa + y * ca]);
    }
    return { strut: [[0, 0], [-0.05, h]], line, thick: 0.035 };
  };

  // Blobs that the air sees when they are not hidden inside the body.
  GT.driverBlobs = () => ({ segs: [[0, 0.06, -0.12, 0.45, 0.12], [0.2, 0.18, 0.55, 0.12, 0.08]], circles: [[-0.14, 0.6, 0.14]] });
  GT.engineBlob = spec => (spec && spec.ev ? [-0.45, -0.08, 0.9, 0.16] : [-0.31, -0.13, 0.62, 0.42]);   // x, y, w, h

  // Closed loops of panels (components where every node has exactly two panels).
  GT.panelLoops = function (d) {
    const adj = new Map();
    for (const p of d.panels || []) {
      if (!adj.has(p.a)) adj.set(p.a, []); if (!adj.has(p.b)) adj.set(p.b, []);
      adj.get(p.a).push(p.b); adj.get(p.b).push(p.a);
    }
    const seen = new Set(), loops = [];
    for (const start of adj.keys()) {
      if (seen.has(start)) continue;
      const comp = [], q = [start]; seen.add(start);
      while (q.length) { const c = q.pop(); comp.push(c); for (const o of adj.get(c)) if (!seen.has(o)) { seen.add(o); q.push(o); } }
      if (comp.length < 3 || comp.some(c => adj.get(c).length !== 2)) continue;
      const loop = [start]; let prev = null, cur = start;
      for (;;) { const nx = adj.get(cur).find(o => o !== prev); if (nx === start || nx == null) break; loop.push(nx); prev = cur; cur = nx; if (loop.length > comp.length) break; }
      if (loop.length === comp.length) loops.push(loop);
    }
    return loops;
  };

  GT.pointInPoly = function (x, y, P) {
    let inside = false;
    for (let i = 0, j = P.length - 1; i < P.length; j = i++) {
      const xi = P[i][0], yi = P[i][1], xj = P[j][0], yj = P[j][1];
      if ((yi > y) !== (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) inside = !inside;
    }
    return inside;
  };

  function loopPolys(d, nm) { return GT.panelLoops(d).map(l => l.map(id => { const n = nm.get(id); return [n.x, n.y]; })); }
  const inBody = (polys, x, y) => polys.some(P => GT.pointInPoly(x, y, P));

  // Geometry fingerprint: a stored tunnel measurement is valid only while this matches.
  GT.aeroHash = function (d) {
    const nm = GT.nodeMapOf(d), P = id => { const n = nm.get(id); return n ? n.x.toFixed(3) + ',' + n.y.toFixed(3) : '-'; };
    const parts = [];
    (d.panels || []).forEach(p => parts.push('p' + P(p.a) + ';' + P(p.b)));
    d.wings.forEach(w => parts.push('w' + w.type + P(w.node)));
    d.wheels.forEach(w => parts.push('h' + w.size + P(w.node)));
    if (d.driver) parts.push('d' + P(d.driver.node));
    if (d.engine) parts.push('e' + d.engine.type + P(d.engine.node));
    let h = 5381; const s = parts.sort().join('|');
    for (let i = 0; i < s.length; i++) h = ((h << 5) + h + s.charCodeAt(i)) | 0;
    return (h >>> 0).toString(36);
  };

  // Items the tunnel can't see: exposed tubes (crossflow drag) and wheels.
  function fixedItems(d, nm, polys) {
    const items = [];
    for (const b of d.beams) {
      const A = nm.get(b.a), B = nm.get(b.b); if (!A || !B) continue;
      const mx = (A.x + B.x) / 2, my = (A.y + B.y) / 2;
      if (inBody(polys, mx, my)) continue;
      const L = Math.hypot(B.x - A.x, B.y - A.y) || 1, s2 = ((B.y - A.y) / L) ** 2;
      const D = GT.tube(b.tube).D;
      items.push({ kind: 'tube', ref: b.id, nodes: [b.a, b.b], cx: -2 * D * L * (0.08 + 0.92 * s2), cy: 0, label: 'Exposed tubes' });
    }
    for (const w of d.wheels) items.push({ kind: 'node', ref: 'wheel' + w.id, nodes: [w.node], cx: -GT.WHEEL_CDA, cy: 0, label: 'Wheels & tyres' });
    if (d.engine && nm.has(d.engine.node)) { const rad = GT.RADIATORS[d.setup.radiator] || GT.RADIATORS.std; items.push({ kind: 'node', ref: 'cooling', nodes: [d.engine.node], cx: -rad.CdA, cy: 0, label: 'Cooling airflow' }); }
    return items;
  }

  GT.aeroEstimate = function (d) {
    const nm = GT.nodeMapOf(d), polys = loopPolys(d, nm), items = [];
    for (const p of d.panels || []) {
      const A = nm.get(p.a), B = nm.get(p.b); if (!A || !B) continue;
      items.push({ kind: 'panel', ref: p.id, nodes: [p.a, p.b], cx: -0.3 * GT.CAR_WIDTH * Math.abs(B.y - A.y) / 2, cy: 0.04 * Math.abs(B.x - A.x), label: 'Body panels' });
    }
    for (const w of d.wings) { const s = GT.WINGS[w.type]; if (s) items.push({ kind: 'wing', ref: w.id, nodes: [w.node], cx: -s.CdA, cy: -s.ClA, label: s.name }); }
    if (d.driver) { const n = nm.get(d.driver.node); if (n && !inBody(polys, n.x - 0.14, n.y + 0.6)) items.push({ kind: 'node', ref: 'driver', nodes: [d.driver.node], cx: -0.15, cy: 0, label: 'Driver / cockpit' }); }
    if (d.engine) { const n = nm.get(d.engine.node); if (n && !inBody(polys, n.x, n.y + 0.1)) items.push({ kind: 'node', ref: 'engine', nodes: [d.engine.node], cx: -0.12, cy: 0, label: 'Engine' }); }
    return items.concat(fixedItems(d, nm, polys));
  };

  // CdA: drag area. ClA: downforce area (negative = lift). ClAf / ClAr: what each axle feels.
  GT.aeroTotals = function (d, items) {
    const nm = GT.nodeMapOf(d), mm = GT.massModel(d), ax = GT.axleLoads(d, mm);
    let CdA = 0, ClA = 0, ClAf = 0;
    for (const it of items) {
      CdA -= it.cx; ClA -= it.cy;
      const ns = it.nodes.map(id => nm.get(id)).filter(Boolean);
      if (ax && ns.length) { const x = ns.reduce((s, n) => s + n.x, 0) / ns.length; ClAf += -it.cy * clamp((x - ax.rearX) / ax.wheelbase, -0.5, 1.5); }
    }
    return { CdA, ClA, ClAf: ax ? ClAf : null, ClAr: ax ? ClA - ClAf : null, LD: CdA > 0 ? ClA / CdA : 0 };
  };

  // What the driving sim uses: the tunnel measurement while geometry matches, else the estimate.
  GT.aeroModel = function (d) {
    const fresh = d.aero && d.aero.hash === GT.aeroHash(d);
    let items;
    if (fresh) {
      const nm = GT.nodeMapOf(d);
      items = d.aero.items.filter(it => it.nodes.every(id => nm.has(id))).concat(fixedItems(d, nm, loopPolys(d, nm)));
    } else items = GT.aeroEstimate(d);
    return { source: fresh ? 'measured' : 'estimate', items, totals: GT.aeroTotals(d, items), stale: !!d.aero && !fresh };
  };

  // ------------------------------------------------------------------ LBM tunnel
  const CX = [0, 1, 0, -1, 0, 1, -1, -1, 1], CY = [0, 0, 1, 0, -1, 1, 1, -1, -1];
  const W = [4 / 9, 1 / 9, 1 / 9, 1 / 9, 1 / 9, 1 / 36, 1 / 36, 1 / 36, 1 / 36];
  const SPEED_RAMP = [[0, 8, 10, 18], [0.5, 22, 44, 92], [0.95, 24, 100, 126], [1.2, 40, 146, 146], [1.4, 242, 190, 80], [1.7, 255, 80, 45], [2.2, 255, 40, 90]];
  const OPP = [0, 3, 4, 1, 2, 7, 8, 5, 6], FLIPY = [0, 1, 4, 3, 2, 8, 7, 6, 5];

  class Tunnel {
    constructor(design, opts) {
      opts = opts || {};
      const d = design, nm = GT.nodeMapOf(d);
      this.U = 0.1; this.tau0 = 0.51; this.Cs = 0.17;
      this.speedKmh = 200;
      // sit the car on the rolling road: lowest tyre touches y = 0
      let low = Infinity;
      for (const w of d.wheels) { const n = nm.get(w.node); if (n) low = Math.min(low, n.y - (GT.WHEELS[w.size] || GT.WHEELS.r18).r); }
      if (!isFinite(low)) low = Math.min(0, ...d.nodes.map(n => n.y)) - 0.15;
      const dy = this.dy = -low;
      const P = id => { const n = nm.get(id); return [n.x, n.y + dy]; };

      const owners = this.owners = [null];
      const own = o => { owners.push(o); return owners.length - 1; };
      const segs = [], circles = [], rects = [];
      for (const p of d.panels || []) { if (!nm.has(p.a) || !nm.has(p.b)) continue; const a = P(p.a), b = P(p.b); segs.push([a[0], a[1], b[0], b[1], 0, own({ kind: 'panel', ref: p.id, nodes: [p.a, p.b], label: 'Body panels' })]); }
      for (const w of d.wings) {
        const s = GT.WINGS[w.type]; if (!s || !nm.has(w.node)) continue;
        const o = own({ kind: 'wing', ref: w.id, nodes: [w.node], label: s.name, splitter: s.kind === 'splitter' }), base = P(w.node), sh = GT.wingShape(s);
        const pts = sh.line.map(([x, y]) => [base[0] + x, base[1] + y]);
        // probes a little above and below the wing measure the dynamic pressure it actually sees
        const probes = [];
        for (const [px, py] of [pts[0], pts[pts.length >> 1], pts[pts.length - 1]]) for (const oy of [0.22, -0.22]) probes.push([px, py + oy]);
        owners[o].probes = probes;
        owners[o].spec = s;
        for (let i = 0; i < pts.length - 1; i++) segs.push([pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1], sh.thick / 2, o]);
        // struts are thin pillars in 3D; in a 2D section they would become walls, so leave them out
      }
      // cockpit / engine only count when the shell doesn't enclose them
      const polys = loopPolys(d, nm);
      const hidden = (id, ox, oy) => { const n = nm.get(id); return inBody(polys, n.x + ox, n.y + oy); };
      if (d.driver && nm.has(d.driver.node) && !hidden(d.driver.node, -0.14, 0.6)) {
        const o = own({ kind: 'node', ref: 'driver', nodes: [d.driver.node], label: 'Driver / cockpit' }), b = P(d.driver.node), bl = GT.driverBlobs();
        bl.segs.forEach(s => segs.push([b[0] + s[0], b[1] + s[1], b[0] + s[2], b[1] + s[3], s[4], o]));
        bl.circles.forEach(c => circles.push([b[0] + c[0], b[1] + c[1], c[2], o]));
      }
      if (d.engine && nm.has(d.engine.node) && !hidden(d.engine.node, 0, 0.1)) {
        const o = own({ kind: 'node', ref: 'engine', nodes: [d.engine.node], label: 'Engine' }), b = P(d.engine.node), r = GT.engineBlob(GT.ENGINES[d.engine.type]);
        rects.push([b[0] + r[0], b[1] + r[1], r[2], r[3], o]);
      }
      this.interior = own({ kind: 'interior' });

      let xmin = Infinity, xmax = -Infinity, ymax = 0.5;
      const ext = (x, y) => { xmin = Math.min(xmin, x); xmax = Math.max(xmax, x); ymax = Math.max(ymax, y); };
      segs.forEach(s => { ext(s[0], s[1]); ext(s[2], s[3]); });
      circles.forEach(c => ext(c[0] + c[2], c[1] + c[2]));
      rects.forEach(r => { ext(r[0], r[1] + r[3]); ext(r[0] + r[2], r[1]); });
      d.nodes.forEach(n => ext(n.x, n.y + dy));
      if (!isFinite(xmin)) { xmin = 0; xmax = 4; }
      const L = Math.max(1, xmax - xmin);
      let dx = clamp(L / 70, 0.04, 0.1);
      let x0 = xmin - Math.max(5, 1.5 * L), x1 = xmax + Math.max(2.5, 0.6 * L), ytop = Math.max(ymax + 4.5, ymax * 4.8);   // tall test section keeps blockage low
      dx = Math.max(dx, (x1 - x0) / 460, ytop / 170);
      this.dx = dx; this.x0 = x0;
      const NX = this.NX = Math.ceil((x1 - x0) / dx), NY = this.NY = Math.ceil(ytop / dx), N = this.N = NX * NY;
      this.carL = L / dx; this.ymax = ymax; this.xmin = xmin; this.xmax = xmax;

      // rasterise
      const solid = this.solid = new Uint8Array(N), owner = this.owner = new Uint16Array(N);
      const cell = (x, y) => [(x - x0) / dx - 0.5, y / dx - 0.5];
      const mark = (i, j, o) => { if (i >= 0 && j >= 0 && i < NX && j < NY) { solid[j * NX + i] = 1; owner[j * NX + i] = o; } };
      for (const [ax, ay, bx, by, rad, o] of segs) {
        const r = Math.max(rad / dx, 0.8), [i1, j1] = cell(ax, ay), [i2, j2] = cell(bx, by);
        const ex = i2 - i1, ey = j2 - j1, L2 = ex * ex + ey * ey || 1e-9;
        for (let j = Math.floor(Math.min(j1, j2) - r); j <= Math.ceil(Math.max(j1, j2) + r); j++)
          for (let i = Math.floor(Math.min(i1, i2) - r); i <= Math.ceil(Math.max(i1, i2) + r); i++) {
            const t = clamp(((i - i1) * ex + (j - j1) * ey) / L2, 0, 1);
            if (Math.hypot(i - i1 - t * ex, j - j1 - t * ey) < r) mark(i, j, o);
          }
      }
      for (const [x, y, rad, o] of circles) {
        const [ci, cj] = cell(x, y), r = rad / dx;
        for (let j = Math.floor(cj - r); j <= Math.ceil(cj + r); j++) for (let i = Math.floor(ci - r); i <= Math.ceil(ci + r); i++) if (Math.hypot(i - ci, j - cj) < r) mark(i, j, o);
      }
      for (const [x, y, w, h, o] of rects) {
        const [ia, ja] = cell(x, y), [ib, jb] = cell(x + w, y + h);
        for (let j = Math.round(ja); j <= Math.round(jb); j++) for (let i = Math.round(ia); i <= Math.round(ib); i++) mark(i, j, o);
      }
      // anything the outside air can't reach is inside the body
      const ext2 = new Uint8Array(N), q = new Int32Array(N); let qh = 0, qt = 0;
      const push = k => { if (!solid[k] && !ext2[k]) { ext2[k] = 1; q[qt++] = k; } };
      for (let i = 0; i < NX; i++) { push(i); push((NY - 1) * NX + i); }
      for (let j = 0; j < NY; j++) { push(j * NX); push(j * NX + NX - 1); }
      while (qh < qt) {
        const k = q[qh++], i = k % NX, j = (k - i) / NX;
        if (i > 0) push(k - 1); if (i < NX - 1) push(k + 1); if (j > 0) push(k - NX); if (j < NY - 1) push(k + NX);
      }
      for (let k = 0; k < N; k++) if (!solid[k] && !ext2[k]) { solid[k] = 1; owner[k] = this.interior; }

      // boundary cells of each owner, for surface pressure
      this.surf = [];
      for (let k = 0; k < N; k++) {
        if (!solid[k]) continue;
        const i = k % NX, j = (k - i) / NX;
        for (let n = 1; n < 9; n++) {
          const ni = i + CX[n], nj = j + CY[n];
          if (ni >= 0 && nj >= 0 && ni < NX && nj < NY && !solid[nj * NX + ni]) { this.surf.push(owner[k], nj * NX + ni); break; }
        }
      }

      this.f = new Float32Array(9 * N); this.g = new Float32Array(9 * N);
      this.rho = new Float32Array(N); this.ux = new Float32Array(N); this.uy = new Float32Array(N);
      this.fx = new Float64Array(owners.length); this.fy = new Float64Array(owners.length);
      this.sumX = new Float64Array(owners.length); this.sumY = new Float64Array(owners.length);
      this.cpSum = new Float64Array(owners.length); this.cpCnt = new Float64Array(owners.length);
      this.samples = 0; this.step = 0;
      this.probeQ = new Float64Array(owners.length); this.probeN = new Float64Array(owners.length);
      owners.forEach(ow => { if (ow && ow.probes) ow.probes = ow.probes.map(([x, y]) => [(x - x0) / dx - 0.5, y / dx - 0.5]); });
      this.warmup = Math.round(Math.max(2000, 2.6 * this.carL / this.U));
      this.total = this.warmup + 4000;   // average over several vortex-shedding cycles
      this.history = [];
      this.smoke = []; this.smokeTick = 0;
      this.reset();
    }

    reset() {
      const { N, f, U } = this, uw = -U;
      for (let k = 0; k < N; k++) {
        this.rho[k] = 1; this.ux[k] = this.solid[k] ? 0 : uw; this.uy[k] = 0;
        for (let n = 0; n < 9; n++) {
          if (this.solid[k]) { f[k * 9 + n] = 0; continue; }
          const cu = 3 * CX[n] * uw;
          f[k * 9 + n] = W[n] * (1 + cu + 0.5 * cu * cu - 1.5 * uw * uw);
        }
      }
    }

    get done() { return this.step >= this.total; }
    get progress() { return Math.min(1, this.step / this.total); }

    run(steps) { for (let s = 0; s < steps; s++) this.stepOnce(); }

    stepOnce() {
      const { NX, NY, solid, owner, rho: R, ux: UX, uy: UY, tau0, fx, fy } = this;
      const f = this.f, g = this.g, uw = -this.U, cs2 = 18 * Math.SQRT2 * this.Cs * this.Cs;
      fx.fill(0); fy.fill(0);
      for (let j = 0; j < NY; j++) {
        for (let i = 0; i < NX; i++) {
          const idx = j * NX + i;
          if (solid[idx]) continue;
          const b = idx * 9;
          const f0 = f[b], f1 = f[b + 1], f2 = f[b + 2], f3 = f[b + 3], f4 = f[b + 4], f5 = f[b + 5], f6 = f[b + 6], f7 = f[b + 7], f8 = f[b + 8];
          let rho = f0 + f1 + f2 + f3 + f4 + f5 + f6 + f7 + f8;
          if (!(rho > 0.2)) rho = 1;   // guard against blow-up
          const ir = 1 / rho;
          const ux = (f1 - f3 + f5 - f6 - f7 + f8) * ir, uy = (f2 - f4 + f5 + f6 - f7 - f8) * ir;
          R[idx] = rho; UX[idx] = ux; UY[idx] = uy;
          const usq = 1.5 * (ux * ux + uy * uy);
          const w1 = rho / 9, w5 = rho / 36;
          const e0 = 4 / 9 * rho * (1 - usq);
          const e1 = w1 * (1 + 3 * ux + 4.5 * ux * ux - usq), e3 = w1 * (1 - 3 * ux + 4.5 * ux * ux - usq);
          const e2 = w1 * (1 + 3 * uy + 4.5 * uy * uy - usq), e4 = w1 * (1 - 3 * uy + 4.5 * uy * uy - usq);
          const a = ux + uy, c = -ux + uy;
          const e5 = w5 * (1 + 3 * a + 4.5 * a * a - usq), e7 = w5 * (1 - 3 * a + 4.5 * a * a - usq);
          const e6 = w5 * (1 + 3 * c + 4.5 * c * c - usq), e8 = w5 * (1 - 3 * c + 4.5 * c * c - usq);
          const n1 = f1 - e1, n2 = f2 - e2, n3 = f3 - e3, n4 = f4 - e4, n5 = f5 - e5, n6 = f6 - e6, n7 = f7 - e7, n8 = f8 - e8;
          const pxx = n1 + n3 + n5 + n6 + n7 + n8, pyy = n2 + n4 + n5 + n6 + n7 + n8, pxy = n5 - n6 + n7 - n8;
          const Q = Math.sqrt(pxx * pxx + pyy * pyy + 2 * pxy * pxy);
          const om = 2 / (tau0 + Math.sqrt(tau0 * tau0 + cs2 * Q * ir));
          const post0 = f0 - om * (f0 - e0), post1 = f1 - om * n1, post2 = f2 - om * n2, post3 = f3 - om * n3, post4 = f4 - om * n4;
          const post5 = f5 - om * n5, post6 = f6 - om * n6, post7 = f7 - om * n7, post8 = f8 - om * n8;
          g[b] = post0;
          // push-stream the 8 moving populations
          this._push(i, j, b, 1, post1, rho, uw); this._push(i, j, b, 2, post2, rho, uw);
          this._push(i, j, b, 3, post3, rho, uw); this._push(i, j, b, 4, post4, rho, uw);
          this._push(i, j, b, 5, post5, rho, uw); this._push(i, j, b, 6, post6, rho, uw);
          this._push(i, j, b, 7, post7, rho, uw); this._push(i, j, b, 8, post8, rho, uw);
        }
      }
      // inlet (right, x = NX-1): freestream equilibrium. outlet (left): zero gradient.
      for (let j = 0; j < NY; j++) {
        const kin = j * NX + NX - 1;
        if (!solid[kin]) for (let n = 0; n < 9; n++) { const cu = 3 * CX[n] * uw; g[kin * 9 + n] = W[n] * (1 + cu + 0.5 * cu * cu - 1.5 * uw * uw); }
        const ko = j * NX, k1 = ko + 1;
        if (!solid[ko] && !solid[k1]) for (let n = 0; n < 9; n++) g[ko * 9 + n] = g[k1 * 9 + n];
      }
      this.f = g; this.g = f;
      this.step++;
      // statistics
      if (this.step > this.warmup) {
        for (let o = 0; o < fx.length; o++) { this.sumX[o] += fx[o]; this.sumY[o] += fy[o]; }
        this.samples++;
        if (this.step % 5 === 0) {
          for (let o = 1; o < this.owners.length; o++) {
            const ow = this.owners[o]; if (!ow || !ow.probes) continue;
            for (const [px, py] of ow.probes) { const u = this.sampleU(px, py); if (u) { this.probeQ[o] += (u[0] * u[0] + u[1] * u[1]) / (this.U * this.U); this.probeN[o]++; } }
          }
          const S = this.surf, qd = 0.5 * this.U * this.U;
          for (let s = 0; s < S.length; s += 2) { this.cpSum[S[s]] += (this.rho[S[s + 1]] - 1) / 3 / qd; this.cpCnt[S[s]]++; }
        }
      }
      if (this.step % 20 === 0) {
        let X = 0, Y = 0;
        for (let o = 1; o < fx.length; o++) { const ow = this.owners[o]; if (ow && ow.spec) continue; const k = this.k3(o); X += fx[o] * k.x; Y += fy[o] * k.y; }
        const sc = this.dx * GT.CAR_WIDTH / (0.5 * this.U * this.U);
        this.history.push([this.step, -X * sc, -Y * sc]);
        if (this.history.length > 600) this.history.splice(0, this.history.length - 600);
      }
    }

    _push(i, j, b, n, v, rho, uw) {
      const ni = i + CX[n], nj = j + CY[n], NX = this.NX;
      if (nj < 0) { this.g[b + OPP[n]] = v - 6 * W[n] * rho * CX[n] * uw; return; }          // rolling road
      if (nj >= this.NY) { if (ni >= 0 && ni < NX) this.g[(j * NX + ni) * 9 + FLIPY[n]] = v; return; }   // slip ceiling
      if (ni < 0 || ni >= NX) return;
      const k = nj * NX + ni;
      if (this.solid[k]) {                              // half-way bounce-back + momentum exchange
        this.g[b + OPP[n]] = v;
        const o = this.owner[k];
        const gv = 2 * (v - W[n]);                       // gauge: subtract ambient so per-part forces are meaningful
        this.fx[o] += gv * CX[n]; this.fy[o] += gv * CY[n];
      } else this.g[k * 9 + n] = v;
    }

    k3(o) { const ow = this.owners[o]; return ow && ow.kind === 'wing' && !ow.splitter ? GT.K3D.wing : GT.K3D.body; }

    // Per-part coefficient·area (m²) averaged after warm-up, as items for the driving sim.
    results() {
      const n = Math.max(1, this.samples), sc = this.dx * GT.CAR_WIDTH / (0.5 * this.U * this.U), items = [];
      for (let o = 1; o < this.owners.length; o++) {
        const ow = this.owners[o]; if (!ow || ow.kind === 'interior') continue;
        const k = this.k3(o);
        if (ow.spec) {   // wings & splitters: catalogue coefficients × the dynamic pressure they actually see
          const qr = this.probeN[o] ? clamp(this.probeQ[o] / this.probeN[o], 0, 1.6) : 1;
          items.push({ kind: ow.kind, ref: ow.ref, nodes: ow.nodes, label: ow.label, cx: -ow.spec.CdA * qr, cy: -ow.spec.ClA * qr, q: qr, cp: this.cpCnt[o] ? this.cpSum[o] / this.cpCnt[o] : null });
          continue;
        }
        items.push({ kind: ow.kind, ref: ow.ref, nodes: ow.nodes, label: ow.label, cx: this.sumX[o] / n * sc * k.x, cy: this.sumY[o] / n * sc * k.y, cp: this.cpCnt[o] ? this.cpSum[o] / this.cpCnt[o] : null });
      }
      return items;
    }

    // Live surface pressure per owner (for colouring panels while it runs).
    liveCp() {
      const S = this.surf, qd = 0.5 * this.U * this.U, sum = new Float64Array(this.owners.length), cnt = new Float64Array(this.owners.length);
      for (let s = 0; s < S.length; s += 2) { sum[S[s]] += (this.rho[S[s + 1]] - 1) / 3 / qd; cnt[S[s]]++; }
      const out = new Map();
      for (let o = 1; o < this.owners.length; o++) { const ow = this.owners[o]; if (ow && cnt[o]) out.set(ow.kind + ':' + ow.ref, sum[o] / cnt[o]); }
      return out;
    }

    sampleU(x, y) {   // bilinear velocity at cell coords
      const { NX, NY } = this;
      if (x < 0 || y < 0 || x >= NX - 1 || y >= NY - 1) return null;
      const i = x | 0, j = y | 0, tx = x - i, ty = y - j, k = j * NX + i;
      if (this.solid[k] && this.solid[k + 1] && this.solid[k + NX] && this.solid[k + NX + 1]) return null;
      const a = (1 - tx) * (1 - ty), b = tx * (1 - ty), c = (1 - tx) * ty, d = tx * ty;
      return [this.ux[k] * a + this.ux[k + 1] * b + this.ux[k + NX] * c + this.ux[k + NX + 1] * d,
              this.uy[k] * a + this.uy[k + 1] * b + this.uy[k + NX] * c + this.uy[k + NX + 1] * d];
    }

    // Smoke streaklines injected at the inlet, advected by the live field.
    advectSmoke(steps) {
      const S = this.smoke;
      this.smokeTick += steps;
      while (this.smokeTick >= 9) {
        this.smokeTick -= 9;
        const top = Math.min(this.NY - 2, (this.ymax * 1.8) / this.dx);
        for (let y = 0.6, i = 0; y < top; y += 1.6, i++) S.push(this.NX - 1.5, y, i);
      }
      for (let p = S.length - 3; p >= 0; p -= 3) {
        const u = this.sampleU(S[p], S[p + 1]);
        if (!u || this.solid[(S[p + 1] | 0) * this.NX + (S[p] | 0)]) { S.splice(p, 3); continue; }
        S[p] += u[0] * steps; S[p + 1] += u[1] * steps;
      }
      if (S.length > 60000) S.splice(0, S.length - 60000);
    }

    // Colour the field into an RGBA buffer (row 0 = top of the image).
    paint(img, view) {
      const { NX, NY, solid, U } = this, data = img.data, qd = 0.5 * U * U;
      for (let j = 0; j < NY; j++) {
        const row = (NY - 1 - j) * NX;
        for (let i = 0; i < NX; i++) {
          const k = j * NX + i, p = (row + i) * 4;
          if (solid[k]) { data[p] = 20; data[p + 1] = 24; data[p + 2] = 30; data[p + 3] = 255; continue; }
          let r, g, b;
          if (view === 'pressure') {
            const cp = clamp((this.rho[k] - 1) / 3 / qd, -2, 1);
            const t = cp < 0 ? cp / 2 : cp;   // -1..1
            if (t < 0) { r = 255 * (1 + t) * 0.85 + 20; g = 255 * (1 + t) * 0.85 + 30; b = 240; } else { r = 240; g = 255 * (1 - t) * 0.85 + 30; b = 255 * (1 - t) * 0.85 + 20; }
            r *= 0.75; g *= 0.75; b *= 0.75;
          } else if (view === 'vorticity') {
            const i0 = i > 0 ? k - 1 : k, i1 = i < NX - 1 ? k + 1 : k, j0 = j > 0 ? k - NX : k, j1 = j < NY - 1 ? k + NX : k;
            const w = clamp(((this.uy[i1] - this.uy[i0]) - (this.ux[j1] - this.ux[j0])) / U * 2.5, -1, 1);
            const a = Math.abs(w);
            r = 14 + (w > 0 ? 240 * a : 40 * a); g = 18 + 60 * a; b = 24 + (w < 0 ? 240 * a : 40 * a);
          } else {
            // speed / freestream: wake near-black, freestream calm teal, accelerated air hot
            const v = Math.hypot(this.ux[k], this.uy[k]) / U;
            let a = 0; while (a < SPEED_RAMP.length - 2 && v > SPEED_RAMP[a + 1][0]) a++;
            const A = SPEED_RAMP[a], B = SPEED_RAMP[a + 1], t = clamp((v - A[0]) / (B[0] - A[0]), 0, 1);
            r = A[1] + (B[1] - A[1]) * t; g = A[2] + (B[2] - A[2]) * t; b = A[3] + (B[3] - A[3]) * t;
          }
          data[p] = r; data[p + 1] = g; data[p + 2] = b; data[p + 3] = 255;
        }
      }
    }
  }
  GT.Tunnel = Tunnel;
})(window.GT);
