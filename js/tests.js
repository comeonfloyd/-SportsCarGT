// Terrain heightfields, the proving-ground tests and the test runner.
(function (GT) {
  'use strict';
  const clamp = (v, a, b) => (v < a ? a : v > b ? b : v);
  const KMH = 3.6;

  function rng(seed) {
    return function () {
      seed = (seed + 0x6D2B79F5) | 0;
      let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }
  const bump = (x, c, w, hgt) => (Math.abs(x - c) < w / 2 ? hgt * 0.5 * (1 + Math.cos(2 * Math.PI * (x - c) / w)) : 0);
  const hill = (x, a, len, amp) => (x > a && x < a + len ? amp * 0.5 * (1 - Math.cos(2 * Math.PI * (x - a) / len)) : 0);
  const kicker = (x, a, len, hgt) => (x >= a && x < a + len ? hgt * (x - a) / len : 0);

  class Terrain {
    constructor(x0, len, dx) {
      this.x0 = x0; this.dx = dx || 0.25;
      this.n = Math.ceil(len / this.dx) + 1;
      this.h = new Float32Array(this.n);
      this.marks = [];
    }
    add(fn) { for (let i = 0; i < this.n; i++) this.h[i] += fn(this.x0 + i * this.dx); return this; }
    mark(x, label, color) { this.marks.push({ x, label, color }); return this; }
    height(x) {
      const f = (x - this.x0) / this.dx;
      if (f <= 0) return this.h[0];
      if (f >= this.n - 1) return this.h[this.n - 1];
      const i = f | 0, t = f - i;
      return this.h[i] * (1 - t) + this.h[i + 1] * t;
    }
    slope(x) {
      const i = clamp(Math.floor((x - this.x0) / this.dx), 0, this.n - 2);
      return (this.h[i + 1] - this.h[i]) / this.dx;
    }
    circleContact(cx, cy, r) {
      const hs = this.height(cx);
      if (cy < hs) {   // centre below the surface: push out along the local normal
        const s = this.slope(cx), inv = 1 / Math.sqrt(1 + s * s);
        return { depth: (hs - cy) * inv + r, nx: -s * inv, ny: inv };
      }
      const i0 = Math.max(0, Math.floor((cx - r - this.x0) / this.dx));
      const i1 = Math.min(this.n - 1, Math.ceil((cx + r - this.x0) / this.dx));
      let best = r, bx = 0, by = 0, found = false;
      for (let i = i0; i < i1; i++) {
        const ax = this.x0 + i * this.dx, ay = this.h[i];
        const ex = this.dx, ey = this.h[i + 1] - ay;
        const t = clamp(((cx - ax) * ex + (cy - ay) * ey) / (ex * ex + ey * ey), 0, 1);
        const px = ax + t * ex, py = ay + t * ey;
        const d = Math.hypot(cx - px, cy - py);
        if (d < best) { best = d; bx = px; by = py; found = true; }
      }
      if (!found || best < 1e-6) return null;
      return { depth: r - best, nx: (cx - bx) / best, ny: (cy - by) / best };
    }
  }
  GT.Terrain = Terrain;
  GT.flatTerrain = () => new Terrain(-100, 400);

  // Simple cruise controller
  function cruise(r, target) {
    const err = target - r.speed;
    return { throttle: clamp(err * 0.35, 0, 1), brake: err < -3 ? clamp(-err * 0.08, 0, 1) : 0 };
  }
  const fmtT = t => (t ? t.toFixed(2) + ' s' : '—');

  GT.TESTS = {
    drag: {
      name: 'Drag strip', icon: '⏱', sub: '0–100, ¼ mile, 1 km',
      desc: 'Flat-out from a standstill. Launch clutch at 45 % redline, automatic shifts. Measures traction, gearing and power-to-weight.',
      maxTime: 70,
      terrain() {
        const t = new Terrain(-80, 1500);
        return t.mark(0, 'START', '#ff7a2f').mark(100, '100 m').mark(201.2, '⅛ mile').mark(402.3, '¼ MILE', '#ff7a2f').mark(1000, '1 KM', '#ff7a2f');
      },
      control: () => ({ throttle: 1, brake: 0 }),
      update(r) {
        const s = r.st, v = r.speed;
        if (!s.t100 && v >= 100 / KMH) s.t100 = r.t;
        if (!s.t200 && v >= 200 / KMH) s.t200 = r.t;
        if (!s.q && r.dist >= 402.3) { s.q = r.t; s.qv = v; }
        if (r.dist >= 1000) { s.k = r.t; s.kv = v; r.finish(); }
      },
      results(r) {
        const s = r.st;
        return [
          ['0–100 km/h', fmtT(s.t100)], ['0–200 km/h', fmtT(s.t200)],
          ['¼ mile', s.q ? `${s.q.toFixed(2)} s @ ${(s.qv * KMH).toFixed(0)} km/h` : '—'],
          ['1 km', s.k ? `${s.k.toFixed(2)} s @ ${(s.kv * KMH).toFixed(0)} km/h` : '—'],
          ['Top speed', `${(r.m.top * KMH).toFixed(0)} km/h`],
        ];
      },
      headline: r => (r.st.t100 ? `0–100 ${r.st.t100.toFixed(2)} s` : 'no 0–100'),
    },

    brake: {
      name: 'Brake test', icon: '🛑', sub: '100–0 km/h',
      desc: 'Accelerate to 100 km/h, then full brakes (no ABS). Nose-dive, lock-ups and stopping distance.',
      maxTime: 45,
      terrain() { return new Terrain(-80, 900).mark(0, 'START', '#ff7a2f'); },
      control(r) {
        const s = r.st;
        if (!s.braking) {
          if (r.speed >= 100 / KMH) { s.braking = true; s.x0 = r.dist; s.v0 = r.speed; s.allowStop = true; r.terrain.mark(r.sim.driverX(), 'BRAKE', '#ff3b30'); }
          else return { throttle: 1, brake: 0 };
        }
        return { throttle: 0, brake: 1 };
      },
      update(r) {
        const s = r.st;
        if (!s.braking) { if (r.t > 30) r.finish('Could not reach 100 km/h in 30 s'); return; }
        s.decel = Math.max(s.decel || 0, -r.sim.gLong);
        s.dive = Math.max(s.dive || 0, Math.abs(r.sim.pitch));
        if (r.speed > 3) for (const w of r.sim.wheels) if (w.contact && Math.abs(w.omega) < 0.5) s.locked = true;
        if (r.speed < 0.15) { s.d = (r.dist - s.x0) * Math.pow((100 / KMH) / s.v0, 2); r.finish(); }
      },
      results(r) {
        const s = r.st;
        return [
          ['100–0 km/h', s.d ? `${s.d.toFixed(1)} m` : '—'],
          ['Peak deceleration', s.decel ? `${s.decel.toFixed(2)} g` : '—'],
          ['Max pitch (dive)', s.dive ? `${(s.dive * 180 / Math.PI).toFixed(1)}°` : '—'],
          ['Wheel lock-up', s.locked ? 'YES — brakes overpower grip' : 'no'],
        ];
      },
      headline: r => (r.st.d ? `100–0 ${r.st.d.toFixed(1)} m` : 'no stop'),
    },

    rough: {
      name: 'Rough road', icon: '〰', sub: 'durability @ 70 km/h',
      desc: '780 m of broken tarmac, potholes, speed bumps and a whoops section at a steady 70 km/h. Fatigue, bottoming and ride comfort.',
      maxTime: 90,
      terrain() {
        const t = new Terrain(-80, 1000), R = rng(7), B = [];
        for (let x = 60; x < 760; x += 6 + R() * 10) B.push([x, 1.0 + R() * 1.6, (R() < 0.3 ? -1 : 1) * (0.03 + R() * 0.07)]);
        t.add(x => {
          if (x < 40 || x > 780) return 0;
          const fade = Math.min(1, (x - 40) / 20, (780 - x) / 20);
          let y = 0.03 * Math.sin(x * 0.7) + 0.02 * Math.sin(x * 1.9 + 1.3);
          for (const b of B) if (Math.abs(x - b[0]) < b[1]) y += bump(x, b[0], b[1], b[2]);
          if (x > 300 && x < 384) y += 0.12 * Math.pow(Math.sin(Math.PI * (x - 300) / 6), 2);
          return y * fade;
        });
        return t.mark(0, 'START', '#ff7a2f').mark(300, 'WHOOPS').mark(780, 'FINISH', '#ff7a2f');
      },
      control: r => cruise(r, 70 / KMH),
      update(r) {
        const s = r.st;
        s.n = (s.n || 0) + 1; s.sq = (s.sq || 0) + (r.sim.gVert - 1) ** 2;
        if (r.dist >= 780) { s.done = true; r.finish(); }
      },
      results(r) {
        const s = r.st;
        return [
          ['Average speed', `${(r.dist / r.t * KMH).toFixed(1)} km/h`],
          ['Ride harshness (RMS)', `${Math.sqrt(s.sq / s.n).toFixed(2)} g`],
          ['Frame scrapes', `${r.sim.scrapes}`],
          ['Bump-stop hits', `${r.sim.bumpHits}`],
          ['Rim strikes', `${r.sim.rimHits}`],
        ];
      },
      pass: r => r.sim.broken === 0,
      headline: r => `RMS ${Math.sqrt(r.st.sq / r.st.n).toFixed(2)} g, ${r.sim.broken} broken`,
    },

    jump: {
      name: 'Jump', icon: '⤴', sub: '1.1 m kicker @ 70 km/h',
      desc: 'A 1.1 m kicker with a flat landing. Takes off around 70 km/h. Landing loads are brutal — this is where weak frames die.',
      maxTime: 40,
      terrain() {
        const t = new Terrain(-80, 600);
        t.add(x => kicker(x, 200, 7, 1.1));
        return t.mark(0, 'START', '#ff7a2f').mark(200, 'KICKER').mark(330, 'FINISH', '#ff7a2f');
      },
      control: r => cruise(r, 70 / KMH),
      update(r) {
        const s = r.st, air = r.sim.wheels.every(w => !w.contact);
        if (air && r.dist > 150) { s.cur = (s.cur || 0) + r.dt; s.flew = true; }
        else { if (s.cur) s.air = Math.max(s.air || 0, s.cur); s.cur = 0; }
        if (s.flew && !air) s.landG = Math.max(s.landG || 0, r.sim.gVert);
        if (r.dist >= 330) r.finish();
      },
      results(r) {
        const s = r.st;
        return [
          ['Airtime', s.air ? `${s.air.toFixed(2)} s` : '—'],
          ['Landing peak (driver)', s.landG ? `${s.landG.toFixed(1)} g` : '—'],
          ['Bump-stop hits', `${r.sim.bumpHits}`],
          ['Rim strikes', `${r.sim.rimHits}`],
        ];
      },
      pass: r => r.sim.broken === 0,
      headline: r => `landing ${(r.st.landG || 0).toFixed(1)} g, ${r.sim.broken} broken`,
    },

    hill: {
      name: 'Hill climb', icon: '⛰', sub: 'grade ramps 0 → 80 %',
      desc: 'A climb that keeps getting steeper, from flat to an 80 % wall at the top. How steep can it go before traction or torque runs out?',
      maxTime: 60,
      terrain() {
        const t = new Terrain(-80, 600), a = 60, len = 300, top = 0.8;
        t.add(x => (x < a ? 0 : x < a + len ? top / (2 * len) * (x - a) ** 2 : top * len / 2));   // grade rises linearly, flat plateau on top
        t.mark(0, 'START', '#ff7a2f');
        for (const g of [20, 40, 60]) t.mark(a + len * g / 80, g + ' %');
        return t.mark(a + len, '80 % TOP', '#ff7a2f');
      },
      grade: x => Math.max(0, Math.min(0.8, 0.8 * (x - 60) / 300)),
      control: r => cruise(r, 40 / KMH),
      update(r) {
        r.st.maxGrade = Math.max(r.st.maxGrade || 0, GT.TESTS.hill.grade(r.dist));
        if (r.dist >= 360) r.finish();
      },
      results(r) {
        return [['Time to the top', r.done && !r.reason ? `${r.t.toFixed(2)} s` : '—'], ['Steepest grade reached', `${((r.st.maxGrade || 0) * 100).toFixed(0)} %`]];
      },
      headline: r => (r.reason ? `stalled at ${((r.st.maxGrade || 0) * 100).toFixed(0)} %` : `top in ${r.t.toFixed(2)} s`),
    },

    endurance: {
      name: 'Endurance stint', icon: '⛽', sub: '5 km: heat, fuel, brakes',
      desc: '5 km at race pace: long straights, three big climbs and six hairpin brake zones. Tests cooling, fuel load vs. burn, starvation and brakes.',
      maxTime: 300,
      corners: [700, 1500, 2200, 3000, 3700, 4400],
      terrain() {
        const t = new Terrain(-80, 5300);
        t.add(x => hill(x, 850, 500, 18) + hill(x, 2350, 400, 12) + hill(x, 3800, 550, 26));
        t.mark(0, 'START', '#ff7a2f');
        this.corners.forEach(c => t.mark(c, 'HAIRPIN', '#ff3b30'));
        return t.mark(5000, 'FINISH', '#ff7a2f');
      },
      control(r) {
        const v = r.speed, d = r.dist;
        let target = 400 / KMH;
        for (const c of GT.TESTS.endurance.corners) {
          const brakeDist = Math.max(0, v * v - (70 / KMH) ** 2) / (2 * 9) + 25;
          if (d > c - brakeDist && d < c + 30) target = 70 / KMH;
        }
        const err = target - v;
        return { throttle: clamp(err * 0.4, 0, 1), brake: err < -2 ? clamp(-err * 0.15, 0, 1) : 0 };
      },
      update(r) {
        const e = r.sim.engine;
        if (e && e.derate < 1) r.st.derateT = (r.st.derateT || 0) + r.dt;
        if (r.dist >= 5000) r.finish();
      },
      results(r) {
        return [
          ['Stint time', r.done && !r.reason ? `${r.t.toFixed(1)} s` : '—'],
          ['Average speed', `${(Math.max(0, r.dist) / r.t * KMH).toFixed(1)} km/h`],
          ['Time power-derated (hot)', `${(r.st.derateT || 0).toFixed(1)} s`],
        ];
      },
      headline: r => (r.reason ? 'DNF' : `${r.t.toFixed(1)} s, ${(r.sim.fuelUsed / GT.FUEL_DENSITY).toFixed(2)} L`),
    },

    free: {
      name: 'Free drive', icon: '🎮', sub: '2.4 km mixed terrain', free: true,
      desc: 'You drive. → / D / W throttle, ← / A / S brake & reverse, Space brake. Hills, bumps, a jump and a whoops section.',
      maxTime: 1e9,
      terrain() {
        const t = new Terrain(-80, 2600), R = rng(42), B = [];
        for (let x = 450; x < 650; x += 4 + R() * 8) B.push([x, 0.5 + R(), (R() < 0.3 ? -1 : 1) * (0.05 + R() * 0.12)]);
        t.add(x => {
          let y = hill(x, 150, 100, 4) + hill(x, 250, 100, 5) + hill(x, 350, 100, 3);
          for (const b of B) if (Math.abs(x - b[0]) < b[1]) y += bump(x, b[0], b[1], b[2]);
          y += kicker(x, 700, 8, 1.2);
          if (x > 760 && x < 900) y += 0.3 * Math.pow(Math.sin(Math.PI * (x - 760) / 7), 2);
          y += hill(x, 1000, 300, 25) + hill(x, 1400, 120, 6) + hill(x, 1600, 60, 2);
          y += kicker(x, 1800, 10, 2.0) + kicker(x, 1950, 6, 0.8);
          return y;
        });
        return t.mark(0, 'START', '#ff7a2f').mark(700, 'JUMP').mark(760, 'WHOOPS').mark(1000, 'THE WALL').mark(1800, 'BIG AIR').mark(2400, 'END', '#ff7a2f');
      },
      control(r, k) {
        const fwd = k.ArrowRight || k.KeyD || k.ArrowUp || k.KeyW, back = k.ArrowLeft || k.KeyA || k.ArrowDown || k.KeyS, v = r.speed;
        if (k.Space) return { throttle: 0, brake: 1 };
        if (fwd) return v < -0.5 ? { throttle: 0, brake: 1 } : { throttle: 1, brake: 0 };
        if (back) return v > 0.5 ? { throttle: 0, brake: 1 } : { throttle: 0.7, brake: 0, reverse: true };
        return { throttle: 0, brake: 0 };
      },
      update(r) { if (r.dist >= 2400) r.finish(); },
      results(r) { return [['Distance', `${Math.max(0, r.dist).toFixed(0)} m`], ['Time', `${r.t.toFixed(1)} s`], ['Top speed', `${(r.m.top * KMH).toFixed(0)} km/h`]]; },
      headline: r => `${Math.max(0, r.dist).toFixed(0)} m`,
    },
  };

  class TestRunner {
    constructor(design, id) {
      this.id = id; this.test = GT.TESTS[id];
      this.design = JSON.parse(JSON.stringify(design));
      this.terrain = this.test.terrain();
      this.sim = new GT.Sim(this.design, this.terrain);
      this.t = 0; this.dt = 0; this.st = {};
      this.m = { top: 0, gl: 0, gb: 0, gv: 0, util: 0 };
      this.samples = []; this.sampleT = 0;
      this.done = false; this.result = null; this.reason = null; this.stuckT = 0;
      this.x0 = this.sim.driverX();
      this.input = { throttle: 0, brake: 0 };
    }
    get speed() { return this.sim.speed; }
    get dist() { return this.sim.driverX() - this.x0; }

    finish(reason) {
      if (this.done) return;
      this.done = true; this.reason = reason || null;
      const t = this.test, s = this.sim;
      const ok = !this.reason && (t.pass ? t.pass(this) : true);
      const rows = t.results(this).concat([
        ['Peak frame utilisation', `${(this.m.util * 100).toFixed(0)} %`],
        ['Tubes broken', `${s.broken} / ${s.tubeCount}`],
        ['Body panels torn', `${s.torn || 0}`],
        ['Aero model', s.aeroSource === 'measured' ? `wind-tunnel (CdA ${s.aeroTotals.CdA.toFixed(2)}, ClA ${s.aeroTotals.ClA.toFixed(2)})` : `estimate (CdA ${s.aeroTotals.CdA.toFixed(2)})`],
        ['Peak long. g (accel / brake)', `${this.m.gl.toFixed(2)} / ${this.m.gb.toFixed(2)}`],
        ['Peak vertical g (driver)', `${this.m.gv.toFixed(1)}`],
      ]);
      const e = s.engine;
      if (e) {
        const km = Math.max(0.001, this.dist / 1000);
        if (e.spec.ev) rows.push(['Energy used', `${((s.battery0 - s.battery) / 3.6e6).toFixed(2)} kWh (${((s.battery0 - s.battery) / 3.6e6 / km * 100).toFixed(1)} kWh/100 km)`]);
        else { const L = s.fuelUsed / GT.FUEL_DENSITY; rows.push(['Fuel used', `${L.toFixed(2)} L (${(L / km * 100).toFixed(1)} L/100 km)`], ['Fuel left', `${(s.fuelKg / GT.FUEL_DENSITY).toFixed(1)} L`]); }
        rows.push(['Peak coolant temp', `${e.peakTemp.toFixed(0)} °C`]);
      }
      this.result = { id: this.id, name: t.name, ok, free: !!t.free, reason: this.reason, rows, headline: t.headline(this), events: s.events.slice() };
    }

    step(dt, keys) {
      if (this.done) return;
      this.dt = dt;
      const inp = this.test.control(this, keys || {});
      this.input = inp;
      this.sim.step(dt, inp);
      this.t += dt;
      const s = this.sim, m = this.m;
      m.top = Math.max(m.top, s.speed);
      if (this.t > 0.3) {
        m.gl = Math.max(m.gl, s.gLong); m.gb = Math.max(m.gb, -s.gLong); m.gv = Math.max(m.gv, s.gVert);
        m.util = Math.max(m.util, s.maxUtil);
      }
      this.test.update(this);
      if (s.crashed) this.finish(s.crashed);
      if (!this.test.free && !this.done) {
        if (this.t > 3 && Math.abs(this.speed) < 0.4 && !this.st.allowStop) this.stuckT += dt; else this.stuckT = 0;
        if (this.stuckT > 4) {
          const grounded = s.nodes.some(n => !n.hub && n.cd > 0);
          this.finish(grounded ? 'Frame collapsed onto the ground — it is a mechanism or the suspension bottomed out' : 'Car stopped — no traction, or the drivetrain lost contact');
        }
        if (this.t > this.test.maxTime) this.finish('Time limit');
      }
      this.sampleT += dt;
      if (this.sampleT >= 0.05) {
        this.sampleT = 0;
        this.samples.push({ t: this.t, v: s.speed * KMH, u: s.maxUtil, rpm: s.engine ? s.engine.rpm : 0 });
      }
    }
  }
  GT.TestRunner = TestRunner;
})(window.GT);
