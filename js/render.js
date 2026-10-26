// Canvas renderer: garage blueprint view and proving-ground view.
(function (GT) {
  'use strict';
  const TAU = Math.PI * 2;
  const lerp = (a, b, t) => a + (b - a) * t;

  GT.stressColor = function (r) {
    if (r >= 1) return '#ff3df2';
    if (r < 0.5) { const t = r / 0.5; return `rgb(${lerp(61, 255, t) | 0},${lerp(220, 214, t) | 0},${lerp(132, 64, t) | 0})`; }
    const t = (r - 0.5) / 0.5;
    return `rgb(255,${lerp(214, 59, t) | 0},${lerp(64, 48, t) | 0})`;
  };

  class Renderer {
    constructor(cv) {
      this.cv = cv; this.ctx = cv.getContext('2d');
      this.cam = { x: 1.3, y: 0.6, s: 170 };
      this.resize();
    }
    resize() {
      const r = this.cv.getBoundingClientRect();
      this.w = Math.max(1, r.width); this.h = Math.max(1, r.height);
      this.dpr = window.devicePixelRatio || 1;
      this.cv.width = Math.round(this.w * this.dpr); this.cv.height = Math.round(this.h * this.dpr);
    }
    sx(x) { return (x - this.cam.x) * this.cam.s + this.w / 2; }
    sy(y) { return this.h / 2 - (y - this.cam.y) * this.cam.s; }
    wx(px) { return (px - this.w / 2) / this.cam.s + this.cam.x; }
    wy(py) { return (this.h / 2 - py) / this.cam.s + this.cam.y; }

    begin(test) {
      const c = this.ctx;
      c.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
      if (test) {
        const g = c.createLinearGradient(0, 0, 0, this.h);
        g.addColorStop(0, '#0b0f15'); g.addColorStop(1, '#1b2431');
        c.fillStyle = g;
      } else c.fillStyle = '#0d1116';
      c.fillRect(0, 0, this.w, this.h);
    }

    // ---------- garage ----------
    grid() {
      const c = this.ctx, s = this.cam.s;
      const x0 = this.wx(0), x1 = this.wx(this.w), y0 = this.wy(this.h), y1 = this.wy(0);
      const line = (step, color, width) => {
        if (step * s < 7) return;
        c.strokeStyle = color; c.lineWidth = width; c.beginPath();
        for (let x = Math.floor(x0 / step) * step; x <= x1; x += step) { const X = Math.round(this.sx(x)) + 0.5; c.moveTo(X, 0); c.lineTo(X, this.h); }
        for (let y = Math.floor(y0 / step) * step; y <= y1; y += step) { const Y = Math.round(this.sy(y)) + 0.5; c.moveTo(0, Y); c.lineTo(this.w, Y); }
        c.stroke();
      };
      line(0.1, '#141a22', 1); line(0.5, '#18202a', 1); line(1, '#212c39', 1);
      c.fillStyle = '#3b4a5c'; c.font = '10px ui-monospace, Menlo, monospace'; c.textAlign = 'left';
      for (let x = Math.ceil(x0); x <= x1; x++) c.fillText(x + ' m', this.sx(x) + 3, this.h - 6);
      for (let y = Math.ceil(y0); y <= y1; y++) if (y !== 0) c.fillText(y + ' m', 4, this.sy(y) - 3);
      // ground reference
      const Y = this.sy(0);
      c.fillStyle = 'rgba(255,122,47,0.04)'; c.fillRect(0, Y, this.w, this.h - Y);
      c.strokeStyle = '#ff7a2f55'; c.setLineDash([8, 6]); c.lineWidth = 1.5;
      c.beginPath(); c.moveTo(0, Y); c.lineTo(this.w, Y); c.stroke(); c.setLineDash([]);
      c.fillStyle = '#ff7a2f88'; c.fillText('GROUND  y = 0', 8, Y + 14);
    }

    drawDesign(d, o) {
      const c = this.ctx, nm = GT.nodeMapOf(d), s = this.cam.s;
      const sel = o.sel || {}, util = o.util;
      const isSel = (k, id) => sel.kind === k && sel.id === id;
      for (const w of d.wheels) {
        const n = nm.get(w.node); if (!n) continue;
        const ws = GT.WHEELS[w.size] || GT.WHEELS.r18;
        this.wheel(n.x, n.y, ws.r, 0, { ghost: true, driven: w.driven, sel: isSel('node', w.node), brake: w.brake });
      }
      for (const b of d.beams) {
        const A = nm.get(b.a), B = nm.get(b.b); if (!A || !B) continue;
        const m = GT.MATERIALS[b.mat] || GT.MATERIALS.mild, t = GT.tube(b.tube);
        const col = util ? GT.stressColor(util.get(b.id) || 0) : m.color;
        this.tube(A.x, A.y, B.x, B.y, t.D, col, isSel('beam', b.id) || o.hoverLink === b, m.brittle);
      }
      for (const sp of d.springs) {
        const A = nm.get(sp.a), B = nm.get(sp.b); if (!A || !B) continue;
        this.coil(A.x, A.y, B.x, B.y, isSel('spring', sp.id) || o.hoverLink === sp, false);
      }
      for (const w of d.wings) { const n = nm.get(w.node); if (n) this.wing(n.x, n.y, 0, GT.WINGS[w.type]); }
      if (d.engine) { const n = nm.get(d.engine.node); if (n) this.engine(n.x, n.y, 0, GT.ENGINES[d.engine.type]); }
      if (d.driver) { const n = nm.get(d.driver.node); if (n) this.driver(n.x, n.y, 0); }
      const r = Math.max(3, Math.min(6, s * 0.025));
      for (const n of d.nodes) {
        const hv = o.hover && o.hover.id === n.id, sl = isSel('node', n.id);
        c.beginPath(); c.arc(this.sx(n.x), this.sy(n.y), sl || hv ? r + 2 : r, 0, TAU);
        c.fillStyle = sl ? '#ff7a2f' : hv ? '#ffd2b8' : '#e8edf2'; c.fill();
        c.lineWidth = 1.5; c.strokeStyle = '#0d1116'; c.stroke();
        if (o.labels) { c.fillStyle = '#5d6d80'; c.font = '9px ui-monospace, Menlo, monospace'; c.fillText('#' + n.id, this.sx(n.x) + 6, this.sy(n.y) - 6); }
      }
      if (o.mm && o.mm.cog) this.cog(o.mm.cog.x, o.mm.cog.y);
      if (o.axle) this.dim(o.axle.rearX, o.axle.frontX, -0.12, `wheelbase ${(o.axle.wheelbase * 1000).toFixed(0)} mm`);
      if (o.preview) {
        const p = o.preview;
        c.setLineDash([6, 4]); c.lineWidth = 2; c.strokeStyle = p.ok ? (p.type === 'spring' ? '#f4c542' : '#ff7a2f') : '#ff3b30';
        c.beginPath(); c.moveTo(this.sx(p.x1), this.sy(p.y1)); c.lineTo(this.sx(p.x2), this.sy(p.y2)); c.stroke(); c.setLineDash([]);
        const L = Math.hypot(p.x2 - p.x1, p.y2 - p.y1), ang = Math.atan2(p.y2 - p.y1, p.x2 - p.x1) * 180 / Math.PI;
        this.label(`${(L * 1000).toFixed(0)} mm  ${ang.toFixed(0)}°`, this.sx(p.x2) + 12, this.sy(p.y2) - 12, p.ok ? '#ffd2b8' : '#ff3b30');
      }
      if (o.cursor) {
        const X = this.sx(o.cursor.x), Y = this.sy(o.cursor.y);
        c.strokeStyle = '#ff7a2f99'; c.lineWidth = 1;
        c.beginPath(); c.moveTo(X - 7, Y); c.lineTo(X + 7, Y); c.moveTo(X, Y - 7); c.lineTo(X, Y + 7); c.stroke();
      }
    }

    label(text, x, y, color) {
      const c = this.ctx;
      c.font = '11px ui-monospace, Menlo, monospace';
      const w = c.measureText(text).width + 10;
      c.fillStyle = 'rgba(10,13,18,0.85)'; c.fillRect(x - 5, y - 12, w, 17);
      c.fillStyle = color || '#cfd8e3'; c.textAlign = 'left'; c.fillText(text, x, y);
    }

    cog(x, y) {
      const c = this.ctx, X = this.sx(x), Y = this.sy(y), r = 9;
      c.save();
      for (let i = 0; i < 4; i++) { c.beginPath(); c.moveTo(X, Y); c.arc(X, Y, r, i * TAU / 4, (i + 1) * TAU / 4); c.closePath(); c.fillStyle = i % 2 ? '#0d1116' : '#f4f6f8'; c.fill(); }
      c.beginPath(); c.arc(X, Y, r, 0, TAU); c.strokeStyle = '#f4f6f8'; c.lineWidth = 1.5; c.stroke();
      c.fillStyle = '#f4f6f8'; c.font = '10px ui-monospace, Menlo, monospace'; c.fillText('CoG', X + 12, Y + 4);
      c.restore();
    }

    dim(x1, x2, y, text) {
      const c = this.ctx, Y = this.sy(y), X1 = this.sx(x1), X2 = this.sx(x2);
      c.strokeStyle = '#4f6378'; c.lineWidth = 1;
      c.beginPath(); c.moveTo(X1, Y); c.lineTo(X2, Y); c.moveTo(X1, Y - 5); c.lineTo(X1, Y + 5); c.moveTo(X2, Y - 5); c.lineTo(X2, Y + 5); c.stroke();
      c.fillStyle = '#7d90a5'; c.font = '10px ui-monospace, Menlo, monospace'; c.textAlign = 'center';
      c.fillText(text, (X1 + X2) / 2, Y + 14); c.textAlign = 'left';
    }

    // ---------- parts ----------
    tube(x1, y1, x2, y2, D, color, hi, carbon) {
      const c = this.ctx, w = Math.max(2, D * this.cam.s * 1.3);
      const X1 = this.sx(x1), Y1 = this.sy(y1), X2 = this.sx(x2), Y2 = this.sy(y2);
      c.lineCap = 'round';
      if (hi) { c.strokeStyle = '#ff7a2f'; c.lineWidth = w + 6; c.beginPath(); c.moveTo(X1, Y1); c.lineTo(X2, Y2); c.stroke(); }
      c.strokeStyle = '#06080b'; c.lineWidth = w + 2; c.beginPath(); c.moveTo(X1, Y1); c.lineTo(X2, Y2); c.stroke();
      c.strokeStyle = color; c.lineWidth = w; c.beginPath(); c.moveTo(X1, Y1); c.lineTo(X2, Y2); c.stroke();
      if (carbon && w > 3) { c.strokeStyle = 'rgba(0,0,0,0.35)'; c.setLineDash([3, 3]); c.lineWidth = w * 0.5; c.beginPath(); c.moveTo(X1, Y1); c.lineTo(X2, Y2); c.stroke(); c.setLineDash([]); }
      else if (w > 3) { c.strokeStyle = 'rgba(255,255,255,0.25)'; c.lineWidth = Math.max(1, w * 0.25); c.beginPath(); c.moveTo(X1, Y1 - w * 0.2); c.lineTo(X2, Y2 - w * 0.2); c.stroke(); }
    }

    coil(x1, y1, x2, y2, hi, bump) {
      const c = this.ctx, s = this.cam.s;
      const X1 = this.sx(x1), Y1 = this.sy(y1), X2 = this.sx(x2), Y2 = this.sy(y2);
      const L = Math.hypot(X2 - X1, Y2 - Y1); if (L < 1) return;
      const ux = (X2 - X1) / L, uy = (Y2 - Y1) / L, nx = -uy, ny = ux;
      const amp = Math.max(3, 0.045 * s);
      c.save();
      if (hi) { c.strokeStyle = '#ff7a2f'; c.lineWidth = amp * 2 + 6; c.globalAlpha = 0.35; c.beginPath(); c.moveTo(X1, Y1); c.lineTo(X2, Y2); c.stroke(); c.globalAlpha = 1; }
      // damper body
      c.strokeStyle = '#3b4552'; c.lineWidth = Math.max(3, 0.03 * s); c.lineCap = 'butt';
      c.beginPath(); c.moveTo(X1 + ux * L * 0.1, Y1 + uy * L * 0.1); c.lineTo(X1 + ux * L * 0.6, Y1 + uy * L * 0.6); c.stroke();
      c.strokeStyle = '#c8d0d9'; c.lineWidth = Math.max(1.5, 0.012 * s);
      c.beginPath(); c.moveTo(X1, Y1); c.lineTo(X2, Y2); c.stroke();
      // coil
      const turns = 7, a = 0.18, b = 0.82;
      c.strokeStyle = bump ? '#ff3b30' : '#f4c542'; c.lineWidth = Math.max(1.5, 0.014 * s); c.lineJoin = 'round';
      c.beginPath();
      for (let i = 0; i <= turns * 8; i++) {
        const t = a + (b - a) * i / (turns * 8), k = Math.sin(i / 8 * TAU) * amp;
        const X = X1 + ux * L * t + nx * k, Y = Y1 + uy * L * t + ny * k;
        if (i === 0) c.moveTo(X, Y); else c.lineTo(X, Y);
      }
      c.stroke();
      c.restore();
    }

    wheel(x, y, r, ang, o) {
      const c = this.ctx, X = this.sx(x), Y = this.sy(y), R = r * this.cam.s;
      c.save();
      c.globalAlpha = o.ghost ? 0.6 : 1;
      c.beginPath(); c.arc(X, Y, R, 0, TAU); c.fillStyle = '#15181c'; c.fill();
      c.lineWidth = Math.max(1, R * 0.06); c.strokeStyle = o.sel ? '#ff7a2f' : o.sliding ? '#ff3b30' : '#2d333b'; c.stroke();
      // tread blocks
      c.strokeStyle = '#22272e'; c.lineWidth = Math.max(1, R * 0.05);
      for (let i = 0; i < 24; i++) { const a = ang + i * TAU / 24; c.beginPath(); c.moveTo(X + Math.cos(a) * R * 0.86, Y + Math.sin(a) * R * 0.86); c.lineTo(X + Math.cos(a) * R * 0.97, Y + Math.sin(a) * R * 0.97); c.stroke(); }
      // brake disc + caliper
      c.beginPath(); c.arc(X, Y, R * 0.5, 0, TAU); c.fillStyle = '#5b5f66'; c.fill();
      c.fillStyle = o.brake === 'ccm' ? '#e8d13a' : o.brake === 'race' ? '#d8352a' : o.brake === 'sport' ? '#c0392b' : '#6d747d';
      c.beginPath(); c.arc(X, Y, R * 0.52, -Math.PI * 0.85, -Math.PI * 0.55); c.arc(X, Y, R * 0.36, -Math.PI * 0.55, -Math.PI * 0.85, true); c.closePath(); c.fill();
      // rim
      c.beginPath(); c.arc(X, Y, R * 0.66, 0, TAU); c.strokeStyle = '#aeb7c2'; c.lineWidth = Math.max(1, R * 0.06); c.stroke();
      c.strokeStyle = '#c9d1da'; c.lineWidth = Math.max(1, R * 0.08); c.lineCap = 'round';
      for (let k = 0; k < 5; k++) {
        const a = ang + k * TAU / 5;
        c.beginPath(); c.moveTo(X + Math.cos(a) * R * 0.14, Y + Math.sin(a) * R * 0.14); c.lineTo(X + Math.cos(a) * R * 0.62, Y + Math.sin(a) * R * 0.62); c.stroke();
      }
      c.beginPath(); c.arc(X, Y, R * 0.12, 0, TAU); c.fillStyle = o.driven ? '#ff7a2f' : '#8a939e'; c.fill();
      c.restore();
    }

    // draw in local, y-up world units around (x, y) rotated by ang
    local(x, y, ang, fn) {
      const c = this.ctx, s = this.cam.s;
      c.save(); c.translate(this.sx(x), this.sy(y)); c.rotate(-ang); c.scale(s, -s);
      fn(c, 1 / s); c.restore();
    }

    engine(x, y, ang, spec) {
      if (!spec) return;
      this.local(x, y, ang, (c, px) => {
        const w = spec.ev ? 0.9 : 0.62, h = spec.ev ? 0.16 : 0.42, oy = spec.ev ? 0 : 0.08;
        c.fillStyle = spec.ev ? '#1f6f8b' : '#a8322a'; c.strokeStyle = '#06080b'; c.lineWidth = 2 * px;
        c.beginPath(); c.rect(-w / 2, -h / 2 + oy, w, h); c.fill(); c.stroke();
        if (!spec.ev) {
          c.fillStyle = '#7d1f19'; c.fillRect(-w / 2 + 0.05, h / 2 + oy - 0.02, w - 0.1, 0.09);
          c.fillStyle = '#2a2f36'; for (let i = 0; i < 4; i++) c.fillRect(-w / 2 + 0.08 + i * 0.13, -h / 2 + oy + 0.06, 0.07, 0.12);
        }
        c.scale(1, -1); c.fillStyle = '#fff'; c.font = `bold ${12 * px}px ui-monospace, Menlo, monospace`; c.textAlign = 'center';
        c.fillText(spec.tag, 0, -oy + 4 * px);
      });
      const c = this.ctx; c.beginPath(); c.arc(this.sx(x), this.sy(y), 3, 0, TAU); c.fillStyle = '#ffd2b8'; c.fill();
    }

    driver(x, y, ang) {
      this.local(x, y, ang, (c, px) => {
        c.lineCap = 'round'; c.lineJoin = 'round';
        c.strokeStyle = '#2c6fd1'; c.lineWidth = 0.11;
        c.beginPath(); c.moveTo(0.55, 0.12); c.lineTo(0.2, 0.18); c.lineTo(0, 0.06); c.lineTo(-0.12, 0.45); c.stroke();   // legs + torso
        c.lineWidth = 0.07; c.beginPath(); c.moveTo(-0.08, 0.38); c.lineTo(0.18, 0.3); c.lineTo(0.32, 0.38); c.stroke();      // arms
        c.strokeStyle = '#222'; c.lineWidth = 0.025; c.beginPath(); c.arc(0.36, 0.42, 0.11, 0, TAU); c.stroke();          // steering wheel
        c.fillStyle = '#f2f2f2'; c.beginPath(); c.arc(-0.14, 0.6, 0.13, 0, TAU); c.fill();
        c.fillStyle = '#ff7a2f'; c.beginPath(); c.arc(-0.14, 0.6, 0.13, -0.3, 0.9); c.lineTo(-0.14, 0.6); c.fill();
        c.fillStyle = '#1b2633'; c.fillRect(-0.06, 0.58, 0.07, 0.05);
        c.strokeStyle = '#555'; c.lineWidth = 0.04; c.beginPath(); c.moveTo(0.1, -0.02); c.lineTo(-0.1, -0.02); c.lineTo(-0.25, 0.55); c.stroke();  // seat
      });
    }

    wing(x, y, ang, spec) {
      if (!spec) return;
      this.local(x, y, ang, (c) => {
        c.fillStyle = '#1d2229'; c.strokeStyle = '#ff7a2f'; c.lineWidth = 0.012;
        if (spec.kind === 'splitter') {
          c.beginPath(); c.moveTo(-0.05, -0.03); c.lineTo(0.42, -0.06); c.lineTo(0.42, -0.04); c.lineTo(-0.05, 0); c.closePath(); c.fill(); c.stroke();
          return;
        }
        const big = spec.ClA > 1.2 ? 1.25 : spec.ClA < 0.4 ? 0.6 : 1;
        const h = spec.ClA < 0.4 ? 0.08 : 0.32 * big, ch = 0.5 * big;
        c.strokeStyle = '#3b4552'; c.lineWidth = 0.03; c.beginPath(); c.moveTo(0, 0); c.lineTo(-0.05, h); c.stroke();
        c.save(); c.translate(-0.1, h + 0.02); c.rotate(-0.16);
        c.beginPath(); c.moveTo(-ch / 2, 0); c.quadraticCurveTo(0, 0.09 * big, ch / 2, 0.01); c.quadraticCurveTo(0, -0.02, -ch / 2, 0); c.fill();
        c.strokeStyle = '#ff7a2f'; c.lineWidth = 0.012; c.stroke();
        c.restore();
      });
    }

    // ---------- proving ground ----------
    backdrop() {
      const c = this.ctx, s = this.cam.s;
      const layers = [[0.08, '#121821', 140, 0.004], [0.2, '#16202b', 90, 0.011]];
      for (const [f, col, amp, freq] of layers) {
        const off = this.cam.x * s * f;
        c.fillStyle = col; c.beginPath(); c.moveTo(0, this.h);
        for (let px = 0; px <= this.w + 8; px += 8) {
          const X = px + off;
          c.lineTo(px, this.h * 0.62 - amp * (0.55 + 0.3 * Math.sin(X * freq) + 0.15 * Math.sin(X * freq * 2.7 + 1.3)));
        }
        c.lineTo(this.w, this.h); c.closePath(); c.fill();
      }
    }

    terrain(T) {
      const c = this.ctx, s = this.cam.s;
      const xa = this.wx(-10), xb = this.wx(this.w + 10);
      const i0 = Math.max(0, Math.floor((xa - T.x0) / T.dx)), i1 = Math.min(T.n - 1, Math.ceil((xb - T.x0) / T.dx));
      const step = Math.max(1, Math.floor(2 / (T.dx * s)));
      const pts = [];
      for (let i = i0; i <= i1; i += step) pts.push([this.sx(T.x0 + i * T.dx), this.sy(T.h[i])]);
      if (pts.length < 2) return;
      const g = c.createLinearGradient(0, this.sy(this.cam.y), 0, this.h + 200);
      g.addColorStop(0, '#2b2420'); g.addColorStop(1, '#14100e');
      c.fillStyle = g; c.beginPath(); c.moveTo(pts[0][0], this.h + 10);
      for (const p of pts) c.lineTo(p[0], p[1]);
      c.lineTo(pts[pts.length - 1][0], this.h + 10); c.closePath(); c.fill();
      c.lineJoin = 'round';
      c.strokeStyle = '#3a3d42'; c.lineWidth = Math.max(3, 0.12 * s); c.beginPath();
      pts.forEach((p, i) => (i ? c.lineTo(p[0], p[1] + Math.max(1.5, 0.06 * s)) : c.moveTo(p[0], p[1] + Math.max(1.5, 0.06 * s)))); c.stroke();
      c.strokeStyle = '#8b8f96'; c.lineWidth = 1.5; c.beginPath();
      pts.forEach((p, i) => (i ? c.lineTo(p[0], p[1]) : c.moveTo(p[0], p[1]))); c.stroke();
      // distance ticks every 10 m
      c.fillStyle = '#4a4f57'; c.font = '10px ui-monospace, Menlo, monospace'; c.textAlign = 'center';
      if (s > 25) for (let x = Math.ceil(xa / 10) * 10; x < xb; x += 10) { const X = this.sx(x), Y = this.sy(T.height(x)); c.fillRect(X - 0.5, Y + 4, 1, 6); if (s > 50 || x % 50 === 0) c.fillText(x + '', X, Y + 22); }
      for (const m of T.marks) {
        if (m.x < xa || m.x > xb) continue;
        const X = this.sx(m.x), Y = this.sy(T.height(m.x));
        c.strokeStyle = m.color || '#cfd8e3'; c.lineWidth = 2;
        c.beginPath(); c.moveTo(X, Y); c.lineTo(X, Y - 1.8 * s); c.stroke();
        c.fillStyle = m.color || '#cfd8e3'; c.font = 'bold 11px ui-monospace, Menlo, monospace';
        c.fillText(m.label, X, Y - 1.8 * s - 6);
      }
      c.textAlign = 'left';
    }

    drawSim(sim, o) {
      const N = sim.nodes, ang = sim.pitch;
      for (const w of sim.wheels) { const n = N[w.n]; this.wheel(n.x, n.y, w.r, w.ang, { driven: w.driven, sliding: w.sliding && Math.abs(w.slip) > 1.5, brake: w.brakeName }); }
      for (const b of sim.beams) {
        if (b.broken) continue;
        const A = N[b.a], B = N[b.b];
        this.tube(A.x, A.y, B.x, B.y, b.D, o.stress ? GT.stressColor(b.ratio) : b.color, false, b.brittle);
        if (b.yielded) { const c = this.ctx; c.fillStyle = '#ff9f0a'; c.beginPath(); c.arc(this.sx((A.x + B.x) / 2), this.sy((A.y + B.y) / 2), 3, 0, TAU); c.fill(); }
      }
      for (const s of sim.springs) { const A = N[s.a], B = N[s.b]; this.coil(A.x, A.y, B.x, B.y, false, s.len < s.Lmin + 0.004); }
      for (const w of sim.wings) { const n = N[w.n]; this.wing(n.x, n.y, ang, w); }
      if (sim.engine) { const n = N[sim.engine.n]; this.engine(n.x, n.y, ang, sim.engine.spec); }
      const dn = N[sim.driver]; if (dn) this.driver(dn.x, dn.y, ang);
      const c = this.ctx, r = Math.max(2, Math.min(4, this.cam.s * 0.02));
      c.fillStyle = '#e8edf2';
      for (const n of N) { c.beginPath(); c.arc(this.sx(n.x), this.sy(n.y), r, 0, TAU); c.fill(); }
      this.particles(sim.particles);
    }

    particles(P) {
      const c = this.ctx, s = this.cam.s;
      for (const p of P) {
        const a = p.life / p.max, X = this.sx(p.x), Y = this.sy(p.y);
        if (p.type === 'smoke') {
          c.fillStyle = `rgba(190,196,204,${0.14 * a})`; c.beginPath(); c.arc(X, Y, p.size * s, 0, TAU); c.fill();
        } else {
          c.strokeStyle = `rgba(255,${(160 + 80 * a) | 0},60,${a})`; c.lineWidth = p.size;
          c.beginPath(); c.moveTo(X, Y); c.lineTo(X - p.vx * 0.02 * s, Y + p.vy * 0.02 * s); c.stroke();
        }
      }
    }
  }
  GT.Renderer = Renderer;
})(window.GT);
