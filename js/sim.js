// Physics: XPBD (extended position-based dynamics) with small substeps.
//  - Tubes are stiff compliant distance constraints; the Lagrange multiplier gives the
//    axial force, which is checked against yield, Euler buckling and fracture.
//  - Coilovers: compliant spring + velocity damper + bump stop / droop strap.
//  - Tyres: compliant vertical contact, rigid wheel rotation, Coulomb friction impulse.
//  - Drivetrain: torque curve, launch clutch, auto gearbox, reflected flywheel inertia.
(function (GT) {
  'use strict';
  const G = GT.G;
  const clamp = (v, a, b) => (v < a ? a : v > b ? b : v);
  const RPM = 60 / (2 * Math.PI);
  const TAU_FORCE = 0.006;   // low-pass on tube forces before failure checks (s)

  GT.curveAt = function (c, rpm) {
    if (rpm <= c[0][0]) return c[0][1];
    for (let i = 1; i < c.length; i++) {
      if (rpm <= c[i][0]) { const a = c[i - 1], b = c[i]; return a[1] + (b[1] - a[1]) * (rpm - a[0]) / (b[0] - a[0]); }
    }
    return c[c.length - 1][1];
  };
  GT.peakTorque = spec => Math.max(...spec.curve.map(p => p[1]));
  GT.peakPower = function (spec) {
    let best = 0, at = 0;
    for (let r = 500; r <= spec.redline; r += 50) { const p = GT.curveAt(spec.curve, r) * r / RPM; if (p > best) { best = p; at = r; } }
    return { kw: best / 1000, rpm: at };
  };

  class Sim {
    constructor(design, terrain, opts) {
      opts = opts || {};
      this.terrain = terrain;
      this.sub = opts.substeps || 40;
      this.t = 0;
      this.events = []; this.particles = [];
      this.broken = 0; this.crashed = null;
      this.scrapes = 0; this.bumpHits = 0; this.rimHits = 0;
      const d = design, mm = GT.massModel(d), nm = mm.nm;
      this.mm = mm; this.M = mm.total;
      this.idx = new Map();
      this.nodes = d.nodes.map((n, i) => {
        this.idx.set(n.id, i);
        const m = mm.mass.get(n.id);
        return { id: n.id, x: n.x, y: n.y, px: n.x, py: n.y, vx: 0, vy: 0, m, w: 1 / m, fx: 0, fy: 0, hub: false, cd: 0, cnx: 0, cny: 1, scr: false, scrPrev: false };
      });
      const I = id => this.idx.get(id);

      this.beams = [];
      for (const b of d.beams) {
        const p = GT.beamProps(nm, b);
        if (!p || p.L < 1e-3) continue;
        this.beams.push({ id: b.id, a: I(b.a), b: I(b.b), L0: p.L, Lorig: p.L, alpha: 1 / p.k, Ft: p.Ft, Fc: p.Fc, buckling: p.buckling,
          brittle: p.mat.brittle, lambda: 0, T: 0, ratio: 0, peak: 0, broken: false, yielded: false, color: p.mat.color, D: p.tube.D });
      }

      this.tubeCount = this.beams.length;
      // body panels: soft links that carry aero load into the frame and can tear off
      this.panelIdx = new Map();
      for (const p of d.panels || []) {
        const pp = GT.panelProps(nm, p);
        if (!pp || pp.L < 1e-3) continue;
        this.panelIdx.set(p.id, this.beams.length);
        this.beams.push({ id: p.id, panel: true, a: I(p.a), b: I(p.b), L0: pp.L, Lorig: pp.L, alpha: 1 / pp.k, Ft: pp.tear, Fc: pp.tear, buckling: false,
          brittle: true, lambda: 0, T: 0, ratio: 0, peak: 0, broken: false, yielded: false, color: pp.mat.color, D: 0.01 });
      }

      this.wheels = [];
      for (const w of d.wheels) {
        if (!this.idx.has(w.node)) continue;
        const ws = GT.WHEELS[w.size] || GT.WHEELS.r18, ty = GT.TIRES[w.tire] || GT.TIRES.street, br = GT.BRAKES[w.brake] || GT.BRAKES.street;
        this.nodes[I(w.node)].hub = true;
        const Iw = 2 * 0.8 * (ws.mass * 0.75) * ws.r * ws.r;
        this.wheels.push({ id: w.id, n: I(w.node), r: ws.r, mu: ty.mu, I: Iw, Ieff: Iw, omega: 0, ang: 0, driven: !!w.driven, brakeT: br.torque, brakeName: w.brake,
          contact: false, nx: 0, ny: 1, Nimp: 0, load: 0, slip: 0, sliding: false, rimLimit: ws.r * 0.13, lastRim: -1 });
      }
      this.driven = this.wheels.filter(w => w.driven);

      // Coilovers, with automatic preload so the drawn pose is roughly the static ride height.
      const ax = GT.axleLoads(d, mm);
      const hubLoad = new Map();
      if (ax) {
        const near = x => d.wheels.filter(w => nm.get(w.node) && Math.abs(nm.get(w.node).x - x) < 0.05);
        const rw = near(ax.rearX), fw = near(ax.frontX);
        rw.forEach(w => hubLoad.set(w.node, clamp(ax.rear, 0, 1) * mm.total * G / rw.length));
        fw.forEach(w => hubLoad.set(w.node, clamp(ax.front, 0, 1) * mm.total * G / fw.length));
        for (const [id, L] of hubLoad) hubLoad.set(id, Math.max(0, L - mm.mass.get(id) * G));
      }
      const perHub = new Map();
      for (const s of d.springs) for (const e of [s.a, s.b]) if (hubLoad.has(e)) perHub.set(e, (perHub.get(e) || 0) + 1);
      this.springs = [];
      for (const s of d.springs) {
        const A = nm.get(s.a), B = nm.get(s.b);
        if (!A || !B) continue;
        const Ld = Math.hypot(B.x - A.x, B.y - A.y);
        if (Ld < 0.02) continue;
        let pre = (s.adj || 0) / 1000;
        if (s.auto !== false) {
          const hub = hubLoad.has(s.a) ? s.a : hubLoad.has(s.b) ? s.b : null;
          if (hub != null) pre += hubLoad.get(hub) / perHub.get(hub) / Math.max(0.35, Math.abs(B.y - A.y) / Ld) / s.k;
        }
        pre = clamp(pre, -0.2, 0.3);
        this.springs.push({ id: s.id, a: I(s.a), b: I(s.b), Ld, L0: Ld + pre, Lmin: Ld - s.travel, Lmax: Ld + s.travel * 0.6,
          alpha: 1 / s.k, c: s.c, travel: s.travel, lambda: 0, F: 0, len: Ld, bump: false, bumpPrev: false });
      }

      if (d.engine && this.idx.has(d.engine.node)) {
        const pt = GT.powertrain(d), spec = pt.spec;
        this.engine = { spec, pt, n: I(d.engine.node), final: pt.final, gear: 1, shiftT: 0, rpm: spec.idle, rpmC: 0, tq: 0, power: 0,
          peakT: GT.peakTorque(spec), flywheel: spec.ev ? 0.04 : 0.13,
          temp: 90, peakTemp: 90, derate: 1, wear: 0, knock: 0, work: 0, ersWork: 0, regen: 0, tcCut: 0, tcActive: false, tcLight: false,
          ers: spec.ers ? spec.ers.store : 0, ersOn: false, starving: false, dead: null, outFuel: false };
      } else this.engine = null;
      // fuel / battery
      const mmFuel = mm.fuelKg || 0;
      this.fuelKg = mmFuel; this.fuel0 = mmFuel; this.fuelUsed = 0;
      this.tankN = d.tank && this.idx.has(d.tank.node) ? I(d.tank.node) : -1;
      this.tankType = d.tank ? (GT.TANK_TYPES[d.tank.type] || GT.TANK_TYPES.steel) : null;
      this.tankL = d.tank ? (GT.TANKS[d.tank.size] || GT.TANKS.l40).L : 0;
      this.battery = this.engine && this.engine.spec.ev ? this.engine.spec.battery * 3.6e6 : 0;
      this.battery0 = this.battery;
      this.absActive = false; this.absLight = false;
      this.driver = d.driver && this.idx.has(d.driver.node) ? I(d.driver.node) : 0;
      this.wings = d.wings.filter(w => this.idx.has(w.node) && GT.WINGS[w.type]).map(w => Object.assign({ n: I(w.node), type: w.type }, GT.WINGS[w.type]));
      this.paint = d.setup.paint || '#d8452b';
      this.loops = GT.panelLoops(d).map(l => l.map(I));
      const am = GT.aeroModel(d);
      this.aeroSource = am.source; this.aeroTotals = am.totals;
      this.aeroItems = am.items.map(it => ({ label: it.label, kind: it.kind, ref: it.ref, cx: it.cx, cy: it.cy, n: it.nodes.map(I),
        beam: it.kind === 'panel' ? this.panelIdx.get(it.ref) : undefined, fx: 0, fy: 0 }));

      // Pitch reference: extreme non-hub nodes.
      let lo = -1, hi = -1;
      this.nodes.forEach((n, i) => { if (n.hub) return; if (lo < 0 || n.x < this.nodes[lo].x) lo = i; if (hi < 0 || n.x > this.nodes[hi].x) hi = i; });
      this.refA = lo; this.refB = hi;
      this.ang0 = lo >= 0 && hi !== lo ? Math.atan2(this.nodes[hi].y - this.nodes[lo].y, this.nodes[hi].x - this.nodes[lo].x) : 0;
      this.pitch = 0;

      this.place(opts.spawnX || 0);
      this.vf = { x: 0, y: 0 }; this.af = { x: 0, y: 0 };
      this.speed = 0; this.gLong = 0; this.gVert = 1; this.maxUtil = 0; this.downforce = 0;
    }

    place(sx) {
      const N = this.nodes, T = this.terrain;
      if (!N.length) return;
      const dx = sx - N[this.driver].x;
      let low = Infinity;
      for (const n of N) if (!n.hub) low = Math.min(low, n.y - T.height(n.x + dx));
      for (const w of this.wheels) { const n = N[w.n]; low = Math.min(low, n.y - w.r - T.height(n.x + dx)); }
      if (!isFinite(low)) low = 0;
      const dy = 0.01 - low;
      for (const n of N) { n.x += dx; n.y += dy; n.px = n.x; n.py = n.y; }
    }

    driverX() { return this.nodes.length ? this.nodes[this.driver].x : 0; }

    ratio() {
      const e = this.engine, g = e.pt.ratios;
      return e.gear === -1 ? -g[0] * 0.95 * e.final : g[e.gear - 1] * e.final;
    }

    step(dt, inp) {
      inp = inp || {};
      this.inp = inp;
      this.aero();
      this.gearbox(dt, inp);
      const h = dt / this.sub;
      for (let s = 0; s < this.sub; s++) this.substep(h, inp);
      this.t += dt;
      this.post(dt);
    }

    // Aero: each part (panel, wing, cockpit, exposed tube, wheel) pushes on its own nodes.
    aero() {
      const N = this.nodes;
      let mv = 0; for (const n of N) mv += n.m * n.vx;
      const v = mv / this.M, q = 0.5 * GT.RHO * v * v, sg = v < 0 ? -1 : 1;
      for (const n of N) { n.fx = 0; n.fy = 0; }
      let df = 0, drag = 0;
      for (const it of this.aeroItems) {
        if (it.beam !== undefined && this.beams[it.beam].broken) { it.fx = it.fy = 0; continue; }
        it.fx = q * it.cx * sg; it.fy = q * it.cy;
        const k = 1 / it.n.length;
        for (const i of it.n) { N[i].fx += it.fx * k; N[i].fy += it.fy * k; }
        df -= it.fy; drag -= it.fx * sg;
      }
      this.downforce = df; this.drag = drag;
    }

    gearbox(dt, inp) {
      const e = this.engine; if (!e) return;
      if (e.shiftT > 0) e.shiftT -= dt;
      if (inp.reverse) e.gear = -1; else if (e.gear === -1) e.gear = 1;
      const pt = e.pt, g = pt.ratios, rl = pt.redline;
      if (!e.spec.ev && e.gear > 0 && e.shiftT <= 0 && this.driven.length) {
        // shift on road speed, not wheel speed, so wheelspin bounces off the limiter instead of hunting gears
        const rpmG = Math.max(0, this.speed) / this.driven[0].r * this.ratio() * RPM;
        if (rpmG > rl * pt.shiftFrac && e.gear < g.length) { e.gear++; e.shiftT = pt.shift; }
        else if (e.gear > 1 && rpmG < rl * 0.5 && rpmG * g[e.gear - 2] / g[e.gear - 1] < rl * 0.85) { e.gear--; e.shiftT = pt.shift; }
      }
      const r = this.ratio(), nd = this.driven.length || 1, locked = e.shiftT <= 0;
      for (const w of this.driven) w.Ieff = w.I + (locked ? e.flywheel * r * r / nd : 0);
    }

    substep(h, inp) {
      const N = this.nodes, h2 = h * h;
      for (const n of N) { n.vx += n.fx * n.w * h; n.vy += (n.fy * n.w - G) * h; }
      this.driveTorque(h, inp);
      for (const n of N) { n.px = n.x; n.py = n.y; n.x += n.vx * h; n.y += n.vy * h; }
      for (const b of this.beams) b.lambda = 0;
      for (const s of this.springs) s.lambda = 0;
      for (let it = 0; it < 2; it++) { this.solveBeams(h2); this.solveSprings(h2); }
      this.collide(h);
      const ih = 1 / h;
      for (const n of N) { n.vx = (n.x - n.px) * ih; n.vy = (n.y - n.py) * ih; }
      this.velocityPass(h);
      this.checkBeams(h, h2);
      for (const w of this.wheels) w.ang += w.omega * h;
    }

    driveTorque(h, inp) {
      const thr = clamp(inp.throttle || 0, 0, 1), brk = clamp(inp.brake || 0, 0, 1);
      const e = this.engine, v = Math.abs(this.speed);
      if (e && this.driven.length) {
        let om = 0; for (const w of this.driven) om += w.omega; om /= this.driven.length;
        const ratio = this.ratio(), sp = e.spec, pt = e.pt;
        const rpmC = om * ratio * RPM;
        e.rpmC = rpmC;
        const alive = !e.dead && (sp.ev ? this.battery > 0 : this.fuelKg > 0);
        const th = alive ? thr : 0;
        let rpm, tq;
        if (sp.ev) {
          rpm = Math.abs(rpmC);
          tq = th > 0.01 ? th * GT.ptTorque(pt, rpm) : -0.08 * e.peakT * Math.min(1, rpm / 1000) * Math.sign(rpmC);  // regen
        } else {
          const launch = sp.idle + th * (pt.launchRpm - sp.idle);
          if (rpmC < launch) { rpm = Math.max(sp.idle, launch); tq = th > 0.01 ? th * GT.ptTorque(pt, rpm) : 0; }   // clutch slipping / open
          else { rpm = rpmC; tq = th > 0.01 ? th * GT.ptTorque(pt, rpm) : -0.1 * e.peakT * (rpm / pt.redline); }  // engine braking
          if (!alive) rpm = Math.max(0, rpmC);
        }
        if (rpm >= pt.redline && tq > 0) tq = 0;   // limiter
        if (e.shiftT > 0) tq = 0;
        if (tq > 0) tq *= e.derate * (e.starving ? 0.25 : 1);
        // hybrid: electric boost from the energy store
        e.ersOn = false;
        if (sp.ers && th > 0.9 && e.ers > 0 && e.shiftT <= 0 && rpm > 1000) {
          const w = rpm / RPM, add = Math.min(sp.ers.kw * 1000 / w, 250);
          tq += add; e.ers -= add * w * h; e.ersWork += add * w * h; e.ersOn = true;
        }
        // traction control (or launch control below 20 km/h): cut torque while driven wheels over-slip
        const target = pt.tc || (pt.launchCtl && v < 6 ? 0.15 : 0);
        if (target && tq > 0) {
          let slip = 0;
          for (const w of this.driven) if (w.contact) slip = Math.max(slip, (w.omega * w.r - v) / Math.max(v, 3));
          e.tcCut = slip > target ? Math.min(0.95, e.tcCut + 60 * h) : Math.max(0, e.tcCut - 25 * h);
          tq *= 1 - e.tcCut;
          if (e.tcCut > 0.05) e.tcActive = true;
        } else e.tcCut = 0;
        e.rpm = rpm; e.tq = tq;
        if (tq > 0) e.work += tq * rpm / RPM * h; else if (sp.ev) e.regen += -tq * rpm / RPM * h;
        const Tw = tq * ratio * 0.9 / this.driven.length;
        for (const w of this.driven) w.omega += Tw * h / w.Ieff;
        if (sp.ers && brk > 0 && v > 5) e.ers = Math.min(sp.ers.store, e.ers + sp.ers.kw * 1000 * brk * 0.6 * h);   // harvest under braking
      }
      const abs = e && e.pt.abs;
      for (const w of this.wheels) {
        let bt = brk * w.brakeT;
        if (abs && bt > 0 && w.contact && v > 2 && (v - w.omega * w.r) / v > 0.12) { bt *= 0.2; this.absActive = true; }   // ABS dumps pressure
        let dw = bt * h / w.Ieff;
        if (w.contact) dw += 0.012 * w.load * w.r * h / w.Ieff;   // rolling resistance
        if (Math.abs(w.omega) <= dw) w.omega = 0; else w.omega -= Math.sign(w.omega) * dw;
      }
    }

    solveBeams(h2) {
      const N = this.nodes;
      for (const b of this.beams) {
        if (b.broken) continue;
        const A = N[b.a], B = N[b.b];
        let dx = B.x - A.x, dy = B.y - A.y;
        const len = Math.sqrt(dx * dx + dy * dy);
        if (len < 1e-9) continue;
        dx /= len; dy /= len;
        const at = b.alpha / h2;
        const dl = (b.L0 - len - at * b.lambda) / (A.w + B.w + at);
        b.lambda += dl;
        A.x -= A.w * dl * dx; A.y -= A.w * dl * dy;
        B.x += B.w * dl * dx; B.y += B.w * dl * dy;
      }
    }

    solveSprings(h2) {
      const N = this.nodes;
      for (const s of this.springs) {
        const A = N[s.a], B = N[s.b];
        let dx = B.x - A.x, dy = B.y - A.y;
        const len = Math.sqrt(dx * dx + dy * dy);
        if (len < 1e-9) continue;
        dx /= len; dy /= len;
        const ws = A.w + B.w;
        const at = s.alpha / h2;
        const dl = (s.L0 - len - at * s.lambda) / (ws + at);
        s.lambda += dl;
        A.x -= A.w * dl * dx; A.y -= A.w * dl * dy;
        B.x += B.w * dl * dx; B.y += B.w * dl * dy;
        const len2 = len + ws * dl;
        let c = 0;
        if (len2 < s.Lmin) { c = (s.Lmin - len2) / (ws + (1 / GT.BUMPSTOP_K) / h2); s.bump = true; }
        else if (len2 > s.Lmax) c = (s.Lmax - len2) / ws;
        if (c) { A.x -= A.w * c * dx; A.y -= A.w * c * dy; B.x += B.w * c * dx; B.y += B.w * c * dy; }
        s.len = len2 + ws * c;
      }
    }

    collide(h) {
      const N = this.nodes, T = this.terrain, h2 = h * h;
      for (const n of N) {
        if (n.hub) continue;
        const g = T.height(n.x);
        if (n.y < g) {
          const s = T.slope(n.x), inv = 1 / Math.sqrt(1 + s * s), nx = -s * inv, ny = inv;
          const dpt = (g - n.y) * ny;
          n.x += nx * dpt; n.y += ny * dpt; n.cd = dpt; n.cnx = nx; n.cny = ny;
        } else n.cd = 0;
      }
      const at = (1 / GT.TIRE_K) / h2;
      for (const w of this.wheels) {
        const n = N[w.n];
        const c = T.circleContact(n.x, n.y, w.r);
        if (!c) { w.contact = false; w.Nimp = 0; w.load = 0; continue; }
        let move = n.w * c.depth / (n.w + at);
        const rest = c.depth - move;
        if (rest > w.rimLimit) {   // tyre fully compressed: rim strike
          move += rest - w.rimLimit;
          if (this.t - w.lastRim > 0.3) { this.rimHits++; this.event('rim', `Rim strike (${(w.load / 1000).toFixed(0)} kN)`); }
          w.lastRim = this.t;
        }
        n.x += c.nx * move; n.y += c.ny * move;
        w.contact = true; w.nx = c.nx; w.ny = c.ny;
        w.Nimp = move * n.m / h; w.load = w.Nimp / h;
      }
    }

    velocityPass(h) {
      const N = this.nodes;
      for (const s of this.springs) {
        const A = N[s.a], B = N[s.b];
        let ux = B.x - A.x, uy = B.y - A.y;
        const L = Math.sqrt(ux * ux + uy * uy) || 1; ux /= L; uy /= L;
        const vr = (B.vx - A.vx) * ux + (B.vy - A.vy) * uy;
        const mx = Math.abs(vr) / (A.w + B.w);
        const imp = clamp(-s.c * vr * h, -mx, mx);
        A.vx -= imp * A.w * ux; A.vy -= imp * A.w * uy;
        B.vx += imp * B.w * ux; B.vy += imp * B.w * uy;
      }
      for (const w of this.wheels) {
        if (!w.contact) { w.slip = 0; w.sliding = false; continue; }
        const n = N[w.n];
        const vn = n.vx * w.nx + n.vy * w.ny;
        const dvn = -vn * Math.min(1, GT.TIRE_C * h * n.w);
        n.vx += dvn * w.nx; n.vy += dvn * w.ny;
        const tx = w.ny, ty = -w.nx;
        const vs = n.vx * tx + n.vy * ty - w.omega * w.r;
        let j = -vs / (n.w + w.r * w.r / w.Ieff);
        const jmax = w.mu * w.Nimp;
        w.sliding = Math.abs(j) > jmax;
        if (w.sliding) j = Math.sign(j) * jmax * 0.9;
        n.vx += j * n.w * tx; n.vy += j * n.w * ty;
        w.omega -= j * w.r / w.Ieff;
        w.slip = vs;
      }
      for (const n of N) {
        if (n.hub || n.cd <= 0) continue;
        const tx = n.cny, ty = -n.cnx;
        const vt = n.vx * tx + n.vy * ty, dv = 0.6 * n.cd / h;
        const nv = Math.abs(vt) <= dv ? 0 : vt - Math.sign(vt) * dv;
        n.vx += (nv - vt) * tx; n.vy += (nv - vt) * ty;
        if (Math.abs(vt) > 2) n.scr = true;
      }
    }

    checkBeams(h, h2) {
      const k = Math.min(1, h / TAU_FORCE);
      for (const b of this.beams) {
        if (b.broken) continue;
        b.T += (-b.lambda / h2 - b.T) * k;          // + tension, - compression
        const cap = b.T >= 0 ? b.Ft : b.Fc;
        const r = Math.abs(b.T) / cap;
        b.ratio = r; if (r > b.peak) b.peak = r;
        if (r <= 1) continue;
        const kN = (Math.abs(b.T) / 1000).toFixed(1);
        if (b.panel) { this.breakBeam(b, 'tear', `Body panel #${b.id} TORE OFF (${kN} kN)`); continue; }
        if (b.T < 0 && b.buckling) this.breakBeam(b, 'buckle', `Tube #${b.id} BUCKLED at ${kN} kN compression`);
        else if (b.brittle) this.breakBeam(b, 'snap', `Carbon tube #${b.id} SNAPPED at ${kN} kN`);
        else {
          // plastic flow: permanent change of rest length, fracture past ~3.5 % strain
          b.L0 += Math.sign(b.T) * (Math.abs(b.T) - cap) * b.alpha * 0.3;
          if (!b.yielded) { b.yielded = true; this.event('yield', `Tube #${b.id} yielded (${b.T > 0 ? 'stretched' : 'crushed'}, ${kN} kN) — permanent set`); }
          if (Math.abs(b.L0 - b.Lorig) / b.Lorig > 0.035 || r > 1.4) this.breakBeam(b, 'fracture', `Tube #${b.id} FRACTURED (${kN} kN)`);
        }
      }
    }

    // Fuel burn, battery, engine temperature, mechanical abuse — once per frame.
    powertrainFrame(dt) {
      const e = this.engine, N = this.nodes;
      this.absLight = this.absActive; this.absActive = false;
      if (!e) return;
      e.tcLight = e.tcActive; e.tcActive = false;
      const sp = e.spec, pt = e.pt, P = (e.work - e.ersWork) / dt, Pall = e.work / dt;
      e.work = 0; e.ersWork = 0; e.power = Pall;
      if (sp.ev) {
        this.battery = Math.max(0, this.battery - (P / 0.92 + 300) * dt + e.regen * 0.7);
        e.regen = 0;
        if (this.battery <= 0 && !e.outFuel) { e.outFuel = true; this.event('fuel', 'BATTERY FLAT — no drive'); }
      } else if (!e.dead && this.fuelKg > 0) {
        const kg = (Math.max(0, P) / 1000 * pt.bsfc / 3.6e6 + 0.0003) * dt;   // + idle flow
        const burn = Math.min(kg, this.fuelKg);
        this.fuelKg -= burn; this.fuelUsed += burn;
        if (this.tankN >= 0) { const n = N[this.tankN]; n.m = Math.max(0.5, n.m - burn); n.w = 1 / n.m; }
        if (this.fuelKg <= 0 && !e.outFuel) { e.outFuel = true; this.event('fuel', 'OUT OF FUEL — engine dead'); }
        // fuel slosh: a low tank unports the pickup under hard g
        const L = this.fuelKg / GT.FUEL_DENSITY, slosh = Math.abs(this.gLong) > 0.6 || Math.abs(this.gVert - 1) > 0.9;
        const starving = L > 0 && L < this.tankType.starveL && slosh && this.t > 1;
        if (starving && !e.starving && this.t - (e.lastStarve || -9) > 2) { this.event('fuel', `Fuel starvation (${L.toFixed(1)} L sloshing) — engine coughs`); e.lastStarve = this.t; }
        e.starving = starving;
      }
      // coolant temperature: heat in ≈ shaft power, radiator rejects with airflow
      const Pk = pt.peakKw * 1000, C = Math.max(3e4, 0.3 * Pk), kRad = pt.radiator.factor * Pk / 180;
      const qin = (sp.ev ? 0.12 : 0.9) * Math.max(0, Pall) * pt.heatMul + (sp.ev ? 0 : 2500);
      let qout = kRad * (e.temp - 25) * (0.3 + Math.abs(this.speed) / 32);
      qout *= clamp((e.temp - 87) / 8, 0, 1);   // thermostat opens 87–95 °C
      e.temp += (qin - qout) / C * dt;
      e.peakTemp = Math.max(e.peakTemp, e.temp);
      e.derate = e.temp > 115 ? clamp(1 - (e.temp - 115) / 15, 0.2, 1) : 1;
      if (e.temp > 112 && !e.hotWarned) { e.hotWarned = true; this.event('heat', `Engine at ${e.temp.toFixed(0)} °C — derating soon`); }
      const kill = msg => { if (e.dead) return; e.dead = msg; this.event('engine', msg); if (!this.crashed) this.crashed = msg; };
      if (e.temp > 130) kill('Engine overheated and seized (130 °C)');
      if (!sp.ev && e.rpm > pt.stockRedline + 50) { e.wear += ((e.rpm - pt.stockRedline) / 500) ** 2 * dt; if (e.wear > 6) kill('Over-revved — valve met piston'); }
      if (pt.knockRisk && e.temp > 105 && Pall > 0.7 * Pk) { e.knock += dt; if (e.knock > 2.5) kill('Detonation — holed a piston (race map + lean mix running hot)'); }
      if (this.tankN >= 0 && this.tankType.rupture && N[this.tankN].cd > 0 && Math.abs(this.speed) > 3 && !this.crashed)
        this.crashed = 'Fuel tank scraped open — FIRE (fit an FIA bag cell or mount it higher)';
    }

    breakBeam(b, kind, msg) {
      b.broken = true;
      if (b.panel) this.torn = (this.torn || 0) + 1; else this.broken++;
      const A = this.nodes[b.a], B = this.nodes[b.b];
      const x = (A.x + B.x) / 2, y = (A.y + B.y) / 2;
      for (let i = 0; i < 18; i++) this.emit(x, y, (Math.random() - 0.5) * 8, Math.random() * 6, 0.5 + Math.random() * 0.5, 'spark', 2);
      this.event(kind, msg);
    }

    event(kind, msg) {
      this.events.push({ t: this.t, kind, msg });
      if (this.events.length > 60) this.events.shift();
    }

    emit(x, y, vx, vy, life, type, size) {
      if (this.particles.length > 500) this.particles.shift();
      this.particles.push({ x, y, vx, vy, life, max: life, type, size });
    }

    post(dt) {
      const N = this.nodes;
      if (!N.length) return;
      const dn = N[this.driver];
      if (!isFinite(dn.x + dn.y)) { this.crashed = 'Simulation diverged (degenerate geometry)'; return; }
      const k1 = Math.min(1, dt / 0.03), k2 = Math.min(1, dt / 0.06);
      const pvx = this.vf.x, pvy = this.vf.y;
      this.vf.x += (dn.vx - this.vf.x) * k1; this.vf.y += (dn.vy - this.vf.y) * k1;
      if (this.t > 0.1) {
        this.af.x += ((this.vf.x - pvx) / dt - this.af.x) * k2;
        this.af.y += ((this.vf.y - pvy) / dt - this.af.y) * k2;
      }
      this.speed = this.vf.x; this.gLong = this.af.x / G; this.gVert = this.af.y / G + 1;
      if (this.refA >= 0 && this.refB !== this.refA) {
        const a = N[this.refA], b = N[this.refB];
        this.pitch = Math.atan2(b.y - a.y, b.x - a.x) - this.ang0;
      }
      if (dn.cd > 0 && !this.crashed && this.t > 0.2) this.crashed = 'Driver hit the ground (rollover / frame collapse)';
      this.powertrainFrame(dt);

      let mu = 0;
      for (const b of this.beams) if (!b.panel && !b.broken && b.ratio > mu) mu = b.ratio;
      this.maxUtil = mu;
      for (const s of this.springs) { if (s.bump && !s.bumpPrev) this.bumpHits++; s.bumpPrev = s.bump; s.bump = false; }
      for (const n of N) {
        if (n.scr) {
          if (!n.scrPrev) { this.scrapes++; }
          if (Math.random() < 0.7) this.emit(n.x, n.y, -n.vx * 0.3 + (Math.random() - 0.5) * 3, Math.random() * 3, 0.35, 'spark', 1.5);
        }
        n.scrPrev = n.scr; n.scr = false;
      }
      for (const w of this.wheels) {
        if (w.contact && Math.abs(w.slip) > 2.5 && Math.random() < 0.45) {
          const n = N[w.n];
          this.emit(n.x - w.nx * w.r, n.y - w.ny * w.r + 0.05, -w.slip * 0.1 + (Math.random() - 0.5), 0.4 + Math.random() * 0.6, 1.1, 'smoke', 0.12 + Math.random() * 0.1);
        }
      }
      const P = this.particles;
      for (let i = P.length - 1; i >= 0; i--) {
        const p = P[i];
        p.life -= dt;
        if (p.life <= 0) { P.splice(i, 1); continue; }
        if (p.type === 'spark') p.vy -= G * dt; else { p.vx *= 0.97; p.vy *= 0.97; p.size += dt * 0.35; }
        p.x += p.vx * dt; p.y += p.vy * dt;
      }
    }
  }

  GT.Sim = Sim;
})(window.GT);
