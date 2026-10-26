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
        const spec = GT.ENGINES[d.engine.type] || GT.ENGINES.i4t;
        this.engine = { spec, n: I(d.engine.node), final: d.setup.finalDrive || spec.final, gear: 1, shiftT: 0, rpm: spec.idle, rpmC: 0, tq: 0,
          peakT: GT.peakTorque(spec), flywheel: spec.ev ? 0.04 : 0.13 };
      } else this.engine = null;
      this.driver = d.driver && this.idx.has(d.driver.node) ? I(d.driver.node) : 0;
      this.wings = d.wings.filter(w => this.idx.has(w.node) && GT.WINGS[w.type]).map(w => Object.assign({ n: I(w.node), type: w.type }, GT.WINGS[w.type]));
      this.body = GT.BODIES[d.setup.body] || GT.BODIES.none;

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
      const e = this.engine;
      return e.gear === -1 ? -e.spec.gears[0] * 0.95 * e.final : e.spec.gears[e.gear - 1] * e.final;
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

    aero() {
      const N = this.nodes;
      let mv = 0; for (const n of N) mv += n.m * n.vx;
      const v = mv / this.M, q = 0.5 * GT.RHO * v * v, sg = Math.sign(v);
      const dragA = q * this.body.CdA / this.M * sg, liftA = q * (this.body.ClA || 0) / this.M;
      for (const n of N) { n.fx = -dragA * n.m; n.fy = -liftA * n.m; }
      let df = q * (this.body.ClA || 0);
      for (const w of this.wings) { const n = N[w.n]; n.fx -= q * w.CdA * sg; n.fy -= q * w.ClA; df += q * w.ClA; }
      this.downforce = df;
    }

    gearbox(dt, inp) {
      const e = this.engine; if (!e) return;
      if (e.shiftT > 0) e.shiftT -= dt;
      if (inp.reverse) e.gear = -1; else if (e.gear === -1) e.gear = 1;
      const g = e.spec.gears, rl = e.spec.redline;
      if (!e.spec.ev && e.gear > 0 && e.shiftT <= 0 && this.driven.length) {
        // shift on road speed, not wheel speed, so wheelspin bounces off the limiter instead of hunting gears
        const rpmG = Math.max(0, this.speed) / this.driven[0].r * this.ratio() * RPM;
        if (rpmG > rl * 0.95 && e.gear < g.length) { e.gear++; e.shiftT = 0.16; }
        else if (e.gear > 1 && rpmG < rl * 0.5 && rpmG * g[e.gear - 2] / g[e.gear - 1] < rl * 0.85) { e.gear--; e.shiftT = 0.12; }
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
      const e = this.engine;
      if (e && this.driven.length) {
        let om = 0; for (const w of this.driven) om += w.omega; om /= this.driven.length;
        const ratio = this.ratio(), sp = e.spec;
        const rpmC = om * ratio * RPM;
        e.rpmC = rpmC;
        let rpm, tq;
        if (sp.ev) {
          rpm = Math.abs(rpmC);
          tq = thr > 0.01 ? thr * GT.curveAt(sp.curve, rpm) : -0.08 * e.peakT * Math.min(1, rpm / 1000) * Math.sign(rpmC);  // regen
        } else {
          const launch = sp.idle + thr * (0.45 * sp.redline - sp.idle);
          if (rpmC < launch) { rpm = Math.max(sp.idle, launch); tq = thr > 0.01 ? thr * GT.curveAt(sp.curve, rpm) : 0; }   // clutch slipping / open
          else { rpm = rpmC; tq = thr > 0.01 ? thr * GT.curveAt(sp.curve, rpm) : -0.1 * e.peakT * (rpm / sp.redline); }  // engine braking
        }
        if (rpm >= sp.redline && tq > 0) tq = 0;   // limiter
        if (e.shiftT > 0) tq = 0;
        e.rpm = rpm; e.tq = tq;
        const Tw = tq * ratio * 0.9 / this.driven.length;
        for (const w of this.driven) w.omega += Tw * h / w.Ieff;
      }
      for (const w of this.wheels) {
        let dw = brk * w.brakeT * h / w.Ieff;
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

    breakBeam(b, kind, msg) {
      b.broken = true; this.broken++;
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

      let mu = 0;
      for (const b of this.beams) if (!b.broken && b.ratio > mu) mu = b.ratio;
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
