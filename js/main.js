// UI glue: garage editor, inspector, proving ground, persistence.
(function (GT) {
  'use strict';
  const $ = s => document.querySelector(s);
  const clamp = (v, a, b) => (v < a ? a : v > b ? b : v);
  const r3 = v => Math.round(v * 1000) / 1000;
  const esc = s => String(s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
  const LS_CUR = 'chassislab.current', LS_GARAGE = 'chassislab.garage';

  const canvas = $('#view');
  const R = GT.R = new GT.Renderer(canvas);
  const App = GT.App = {
    mode: 'build', tool: 'beam', mat: 'mild', tube: '32x2.0', snap: true, labels: false,
    sel: null, hover: null, hoverLink: null, drag: null, pan: null, mouse: null, cursor: null,
    undo: [], redo: [], design: null, staticCheck: null,
    runner: null, testId: null, paused: false, slow: false, stress: true, keys: {}, history: [], buildCam: null,
  };

  const TOOLS = [
    ['select', 'V', '⌖', 'Select', 'Click a node or tube to inspect it. Drag nodes to move them.'],
    ['beam', 'B', '╱', 'Tube', 'Drag from point to point to cut a tube. Starts/ends snap to existing nodes.'],
    ['spring', 'S', '⌇', 'Coilover', 'Drag between two nodes to fit a spring-damper. Typical: wheel hub → frame, with a rigid arm as the other link.'],
    ['wheel', 'W', '◎', 'Axle', 'Click a node to mount an axle (pair of wheels) on it. That node becomes the hub.'],
    ['engine', 'E', '▣', 'Engine', 'Click a node to mount the engine + gearbox there. Heavy — mind the CoG.'],
    ['driver', 'D', '☺', 'Driver', 'Click a node to place the driver seat (80 kg). If the driver touches the ground, the run is over.'],
    ['wing', 'A', '⌒', 'Aero', 'Click a node to bolt on a wing or splitter. Downforce loads that node.'],
    ['node', 'N', '•', 'Node', 'Click to place a free joint.'],
    ['erase', 'X', '✕', 'Erase', 'Click a tube/coilover to remove it. Click a node: first removes parts mounted on it, then the node.'],
  ];

  // ---------- design state ----------
  const snapshot = () => JSON.stringify(App.design);
  function mutate(fn) {
    const s = snapshot();
    fn(App.design);
    App.undo.push(s); if (App.undo.length > 200) App.undo.shift();
    App.redo = [];
    changed();
  }
  function changed() {
    App.staticCheck = null;
    try { localStorage.setItem(LS_CUR, snapshot()); } catch (e) { /* storage unavailable */ }
    refreshPanels();
  }
  function setDesign(d, keepUndo) {
    if (keepUndo !== false && App.design) { App.undo.push(snapshot()); App.redo = []; }
    App.design = GT.normalize(d);
    App.sel = null;
    changed();
  }
  function undo() { if (!App.undo.length) return; App.redo.push(snapshot()); App.design = JSON.parse(App.undo.pop()); App.sel = null; changed(); toast('Undo'); }
  function redo() { if (!App.redo.length) return; App.undo.push(snapshot()); App.design = JSON.parse(App.redo.pop()); App.sel = null; changed(); toast('Redo'); }

  const D = () => App.design;
  function addNode(d, x, y) { const n = { id: d.nextId++, x: r3(x), y: r3(y) }; d.nodes.push(n); return n; }
  const linkBetween = (d, a, b) => d.beams.concat(d.springs).find(l => (l.a === a && l.b === b) || (l.a === b && l.b === a));
  function removeNode(d, id) {
    d.nodes = d.nodes.filter(n => n.id !== id);
    d.beams = d.beams.filter(l => l.a !== id && l.b !== id);
    d.springs = d.springs.filter(l => l.a !== id && l.b !== id);
    d.wheels = d.wheels.filter(w => w.node !== id);
    d.wings = d.wings.filter(w => w.node !== id);
    if (d.engine && d.engine.node === id) d.engine = null;
    if (d.driver && d.driver.node === id) d.driver = null;
  }
  function partsOn(d, id) {
    return { wheel: d.wheels.find(w => w.node === id), wings: d.wings.filter(w => w.node === id), engine: d.engine && d.engine.node === id, driver: d.driver && d.driver.node === id };
  }
  function selected() {
    const s = App.sel, d = D(); if (!s) return null;
    if (s.kind === 'node') return d.nodes.find(n => n.id === s.id);
    if (s.kind === 'beam') return d.beams.find(b => b.id === s.id);
    if (s.kind === 'spring') return d.springs.find(b => b.id === s.id);
    return null;
  }

  // ---------- hit testing ----------
  function nodeAt(x, y, px) {
    const r = (px || 11) / R.cam.s; let best = null, bd = r;
    for (const n of D().nodes) { const d = Math.hypot(n.x - x, n.y - y); if (d < bd) { bd = d; best = n; } }
    return best;
  }
  function linkAt(x, y) {
    const d = D(), nm = GT.nodeMapOf(d), r = 7 / R.cam.s; let best = null, bd = r;
    const test = (l, kind) => {
      const A = nm.get(l.a), B = nm.get(l.b); if (!A || !B) return;
      const ex = B.x - A.x, ey = B.y - A.y, L2 = ex * ex + ey * ey || 1;
      const t = clamp(((x - A.x) * ex + (y - A.y) * ey) / L2, 0, 1);
      const dd = Math.hypot(x - A.x - t * ex, y - A.y - t * ey);
      if (dd < bd) { bd = dd; best = { kind, link: l }; }
    };
    d.springs.forEach(l => test(l, 'spring')); d.beams.forEach(l => test(l, 'beam'));
    return best;
  }
  function wheelNodeAt(x, y) {
    const d = D(), nm = GT.nodeMapOf(d);
    for (const w of d.wheels) { const n = nm.get(w.node), ws = GT.WHEELS[w.size]; if (n && ws && Math.hypot(n.x - x, n.y - y) < ws.r) return n; }
    return null;
  }
  const mpos = e => { const r = canvas.getBoundingClientRect(); return { x: e.clientX - r.left, y: e.clientY - r.top }; };
  const world = p => ({ x: R.wx(p.x), y: R.wy(p.y) });
  function snapPt(w, e) {
    if (!App.snap || (e && e.shiftKey)) return { x: r3(w.x), y: r3(w.y) };
    const g = GT.SNAP; return { x: r3(Math.round(w.x / g) * g), y: r3(Math.round(w.y / g) * g) };
  }

  // ---------- editor tools ----------
  function defaultWheel(d, node) {
    return { id: d.nextId++, node, size: 'r18', tire: 'sport', brake: 'sport', driven: !d.wheels.some(w => w.driven) };
  }
  function toolDown(p, e) {
    const w = world(p), sp = snapPt(w, e), hn = nodeAt(w.x, w.y);
    const target = hn || null;
    const placeOn = (fn) => mutate(d => { const n = target || addNode(d, sp.x, sp.y); fn(d, n); App.sel = { kind: 'node', id: n.id }; });
    switch (App.tool) {
      case 'select': {
        if (hn) { App.sel = { kind: 'node', id: hn.id }; App.drag = { type: 'move', id: hn.id, snap: snapshot(), moved: false, ox: hn.x - w.x, oy: hn.y - w.y }; }
        else {
          const l = linkAt(w.x, w.y), wn = wheelNodeAt(w.x, w.y);
          App.sel = l ? { kind: l.kind, id: l.link.id } : wn ? { kind: 'node', id: wn.id } : null;
        }
        refreshPanels();
        break;
      }
      case 'node':
        if (hn) { App.sel = { kind: 'node', id: hn.id }; refreshPanels(); } else mutate(d => { const n = addNode(d, sp.x, sp.y); App.sel = { kind: 'node', id: n.id }; });
        break;
      case 'beam': case 'spring':
        App.drag = { type: 'link', from: hn ? hn.id : null, x: hn ? hn.x : sp.x, y: hn ? hn.y : sp.y, tx: hn ? hn.x : sp.x, ty: hn ? hn.y : sp.y };
        break;
      case 'wheel': {
        const n = hn || wheelNodeAt(w.x, w.y);
        if (n && partsOn(D(), n.id).wheel) { App.sel = { kind: 'node', id: n.id }; refreshPanels(); break; }
        mutate(d => { const nn = n || addNode(d, sp.x, sp.y); d.wheels.push(defaultWheel(d, nn.id)); App.sel = { kind: 'node', id: nn.id }; });
        toast('Axle mounted. Link the hub to the frame with a rigid arm (Tube) + a Coilover.');
        break;
      }
      case 'engine': placeOn((d, n) => { d.engine = { node: n.id, type: d.engine ? d.engine.type : 'i4t' }; }); break;
      case 'driver': placeOn((d, n) => { d.driver = { node: n.id }; }); break;
      case 'wing': placeOn((d, n) => { d.wings.push({ id: d.nextId++, node: n.id, type: d.wings.length ? 'splitter' : 'gt' }); }); break;
      case 'erase': {
        const l = !hn && linkAt(w.x, w.y);
        const n = hn || (!l && wheelNodeAt(w.x, w.y));
        if (n) {
          mutate(d => {
            const pp = partsOn(d, n.id);
            if (pp.wheel || pp.wings.length || pp.engine || pp.driver) {
              d.wheels = d.wheels.filter(x => x.node !== n.id); d.wings = d.wings.filter(x => x.node !== n.id);
              if (pp.engine) d.engine = null; if (pp.driver) d.driver = null;
            } else removeNode(d, n.id);
          });
        } else if (l) mutate(d => { d[l.kind === 'beam' ? 'beams' : 'springs'] = d[l.kind === 'beam' ? 'beams' : 'springs'].filter(x => x.id !== l.link.id); });
        App.sel = null; refreshPanels();
        break;
      }
    }
  }
  function toolMove(p, e) {
    const w = world(p), sp = snapPt(w, e);
    App.cursor = sp;
    App.hover = nodeAt(w.x, w.y);
    App.hoverLink = (App.tool === 'select' || App.tool === 'erase') && !App.hover ? (linkAt(w.x, w.y) || {}).link : null;
    const dr = App.drag; if (!dr) return;
    if (dr.type === 'move') {
      const n = D().nodes.find(x => x.id === dr.id); if (!n) return;
      const np = snapPt({ x: w.x + dr.ox, y: w.y + dr.oy }, e);
      if (np.x !== n.x || np.y !== n.y) { n.x = np.x; n.y = np.y; dr.moved = true; App.staticCheck = null; throttledPanels(); }
    } else if (dr.type === 'link') {
      const hn = App.hover;
      dr.tx = hn ? hn.x : sp.x; dr.ty = hn ? hn.y : sp.y;
    }
  }
  function toolUp(p, e) {
    const dr = App.drag; App.drag = null; if (!dr) return;
    if (dr.type === 'move') {
      if (dr.moved) { App.undo.push(dr.snap); App.redo = []; changed(); }
      return;
    }
    if (dr.type === 'link') {
      const w = world(p), hn = nodeAt(w.x, w.y), sp = snapPt(w, e);
      const tx = hn ? hn.x : sp.x, ty = hn ? hn.y : sp.y;
      const L = Math.hypot(tx - dr.x, ty - dr.y);
      if (L < 0.02 || (hn && hn.id === dr.from)) {
        if (dr.from == null) mutate(d => { const n = addNode(d, dr.x, dr.y); App.sel = { kind: 'node', id: n.id }; });
        return;
      }
      if (L > GT.MAX_BEAM) { toast(`Too long: ${L.toFixed(2)} m (max ${GT.MAX_BEAM} m stock)`, true); return; }
      if (dr.from != null && hn && linkBetween(D(), dr.from, hn.id)) { toast('Those nodes are already linked.', true); return; }
      mutate(d => {
        const a = dr.from != null ? dr.from : addNode(d, dr.x, dr.y).id;
        const b = hn ? hn.id : addNode(d, tx, ty).id;
        if (App.tool === 'spring') { const s = { id: d.nextId++, a, b, k: 60000, c: 3000, travel: 0.1, auto: true, adj: 0 }; d.springs.push(s); App.sel = { kind: 'spring', id: s.id }; }
        else { const bm = { id: d.nextId++, a, b, mat: App.mat, tube: App.tube }; d.beams.push(bm); App.sel = { kind: 'beam', id: bm.id }; }
      });
    }
  }

  // ---------- input ----------
  canvas.addEventListener('contextmenu', e => e.preventDefault());
  canvas.addEventListener('mousedown', e => {
    const p = mpos(e); App.mouse = p;
    if (e.button === 1 || e.button === 2 || App.keys.Space) { App.pan = { x: p.x, y: p.y, cx: R.cam.x, cy: R.cam.y }; e.preventDefault(); return; }
    if (App.mode === 'build' && e.button === 0) toolDown(p, e);
  });
  window.addEventListener('mousemove', e => {
    const p = mpos(e); App.mouse = p;
    if (App.pan) {
      R.cam.x = App.pan.cx - (p.x - App.pan.x) / R.cam.s;
      R.cam.y = App.pan.cy + (p.y - App.pan.y) / R.cam.s;
      if (App.mode === 'test') App.follow = false;
      return;
    }
    if (App.mode === 'build') toolMove(p, e);
  });
  window.addEventListener('mouseup', e => {
    if (App.pan) { App.pan = null; return; }
    if (App.mode === 'build') toolUp(mpos(e), e);
  });
  canvas.addEventListener('mouseleave', () => { App.cursor = null; });
  canvas.addEventListener('wheel', e => {
    e.preventDefault();
    const p = mpos(e), w0 = world(p);
    R.cam.s = clamp(R.cam.s * Math.exp(-e.deltaY * 0.0015), 12, 1200);
    if (App.mode === 'build') { R.cam.x = w0.x - (p.x - R.w / 2) / R.cam.s; R.cam.y = w0.y - (R.h / 2 - p.y) / R.cam.s; }
  }, { passive: false });

  window.addEventListener('keydown', e => {
    if (e.target.matches('input, select, textarea')) return;
    App.keys[e.code] = true;
    const mod = e.ctrlKey || e.metaKey;
    if (mod && e.code === 'KeyZ') { e.preventDefault(); if (App.mode === 'build') (e.shiftKey ? redo : undo)(); return; }
    if (mod && e.code === 'KeyY') { e.preventDefault(); if (App.mode === 'build') redo(); return; }
    if (mod) return;
    if (App.mode === 'build') {
      const t = TOOLS.find(t => 'Key' + t[1] === e.code);
      if (t) { setTool(t[0]); return; }
      if (e.code === 'Delete' || e.code === 'Backspace') { deleteSelected(); e.preventDefault(); }
      else if (e.code === 'KeyG') { App.snap = !App.snap; $('#snapChk').checked = App.snap; toast('Grid snap ' + (App.snap ? 'on' : 'off')); }
      else if (e.code === 'KeyF') fitView();
      else if (e.code === 'Escape') { App.drag = null; App.sel = null; refreshPanels(); }
      else if (e.code === 'Enter') setMode('test');
      else if (e.code === 'Space') e.preventDefault();
    } else {
      if (['ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown', 'Space'].includes(e.code)) e.preventDefault();
      if (e.code === 'KeyR' && App.testId) runTest(App.testId);
      else if (e.code === 'KeyP') App.paused = !App.paused;
      else if (e.code === 'KeyM') { App.slow = !App.slow; $('#slowChk').checked = App.slow; }
      else if (e.code === 'KeyC') App.follow = true;
      else if (e.code === 'Escape') setMode('build');
    }
  });
  window.addEventListener('keyup', e => { App.keys[e.code] = false; });
  window.addEventListener('blur', () => { App.keys = {}; });

  function deleteSelected() {
    const s = App.sel; if (!s) return;
    mutate(d => {
      if (s.kind === 'node') removeNode(d, s.id);
      if (s.kind === 'beam') d.beams = d.beams.filter(b => b.id !== s.id);
      if (s.kind === 'spring') d.springs = d.springs.filter(b => b.id !== s.id);
    });
    App.sel = null; refreshPanels();
  }

  function setTool(t) {
    App.tool = t; App.drag = null;
    document.querySelectorAll('.tool').forEach(b => b.classList.toggle('active', b.dataset.tool === t));
    const def = TOOLS.find(x => x[0] === t);
    $('#hint').textContent = def ? `${def[3]}: ${def[4]}` : '';
  }

  function fitView() {
    const d = D(); if (!d.nodes.length) { R.cam.x = 1.3; R.cam.y = 0.6; R.cam.s = 170; return; }
    let x0 = Infinity, x1 = -Infinity, y0 = 0, y1 = -Infinity;
    const nm = GT.nodeMapOf(d);
    d.nodes.forEach(n => { x0 = Math.min(x0, n.x); x1 = Math.max(x1, n.x); y1 = Math.max(y1, n.y + 0.7); });
    d.wheels.forEach(w => { const n = nm.get(w.node), r = (GT.WHEELS[w.size] || GT.WHEELS.r18).r; if (n) { x0 = Math.min(x0, n.x - r); x1 = Math.max(x1, n.x + r); } });
    R.cam.x = (x0 + x1) / 2; R.cam.y = (y0 + y1) / 2;
    R.cam.s = clamp(Math.min(R.w / (x1 - x0 + 1.2), R.h / (y1 - y0 + 1.2)), 30, 600);
  }

  // ---------- panels ----------
  let panelTimer = 0;
  function throttledPanels() { if (panelTimer) return; panelTimer = setTimeout(() => { panelTimer = 0; refreshPanels(); }, 80); }

  const opts = (obj, cur) => Object.entries(obj).map(([k, v]) => `<option value="${k}"${k === cur ? ' selected' : ''}>${esc(v.name)}</option>`).join('');
  const sel = (f, obj, cur) => `<select data-f="${f}">${opts(obj, cur)}</select>`;
  const num = (f, v, step, scale, min, max) => `<input type="number" data-f="${f}" value="${+(v / (scale || 1)).toFixed(4)}" step="${step}" data-scale="${scale || 1}"${min != null ? ` min="${min}"` : ''}${max != null ? ` max="${max}"` : ''}>`;
  const fld = (label, html) => `<label class="field">${label}${html}</label>`;
  const kv = rows => `<div class="kv">${rows.map(([k, v, cls]) => `<span>${k}</span><span class="${cls || ''}">${v}</span>`).join('')}</div>`;
  const kN = f => (f / 1000).toFixed(1) + ' kN';

  function refreshPanels() {
    if (App.mode !== 'build') return;
    refreshInspector(); refreshStats(); refreshWarnings(); refreshTubeInfo();
  }

  function refreshTubeInfo() {
    const m = GT.MATERIALS[App.mat], t = GT.tube(App.tube);
    const kgm = 2 * t.A * m.rho;
    const pcr = L => 2 * Math.PI * Math.PI * m.E * t.I / Math.pow(GT.K_EFF * L, 2);
    $('#tubeInfo').innerHTML = `pair: ${kgm.toFixed(2)} kg/m · $${(kgm * m.costKg).toFixed(0)}/m<br>tension ${kN(2 * m.yield * t.A)}<br>buckling @1 m ${kN(Math.min(pcr(1), 2 * (m.compYield || m.yield) * t.A))} · @2 m ${kN(Math.min(pcr(2), 2 * (m.compYield || m.yield) * t.A))}${m.brittle ? '<br><span style="color:#b07cff">brittle: no yielding, it snaps</span>' : ''}`;
  }

  function refreshInspector() {
    const d = D(), s = App.sel, obj = selected();
    let h = '';
    if (!obj) {
      const e = d.engine ? GT.ENGINES[d.engine.type] : null;
      h += `<h4>Car setup <span class="tag">nothing selected</span></h4>`;
      h += fld('Name', `<input type="text" data-f="setup:0:name" value="${esc(d.name)}">`);
      h += fld('Bodywork (drag & mass)', sel('setup:0:body', GT.BODIES, d.setup.body));
      h += fld(`Final drive ${e ? `(engine default ${e.final.toFixed(2)})` : ''}`, `<input type="number" data-f="setup:0:finalDrive" step="0.05" min="1" max="12" placeholder="default" value="${d.setup.finalDrive || ''}">`);
      if (e) h += `<div class="small dim mono">gears: ${e.gears.map(g => g.toFixed(2)).join(' · ')}</div>`;
      h += `<p class="small dim">Select a node, tube or coilover to edit it.</p>`;
    } else if (s.kind === 'node') {
      const pp = partsOn(d, obj.id);
      const links = d.beams.concat(d.springs).filter(l => l.a === obj.id || l.b === obj.id).length;
      h += `<h4>Node #${obj.id} <span class="tag">${links} link${links === 1 ? '' : 's'}</span></h4>`;
      h += `<div class="grid2">${fld('x (m)', num(`node:${obj.id}:x`, obj.x, 0.01))}${fld('y (m)', num(`node:${obj.id}:y`, obj.y, 0.01))}</div>`;
      if (pp.wheel) {
        const w = pp.wheel;
        h += `<div class="sub">AXLE (pair of wheels)<button data-act="rm-wheel" data-id="${w.id}">remove</button></div>`;
        h += fld('Wheel', sel(`wheel:${w.id}:size`, GT.WHEELS, w.size));
        h += fld('Tyre', sel(`wheel:${w.id}:tire`, GT.TIRES, w.tire));
        h += fld('Brakes', sel(`wheel:${w.id}:brake`, GT.BRAKES, w.brake));
        h += `<label class="check"><input type="checkbox" data-f="wheel:${w.id}:driven"${w.driven ? ' checked' : ''}> Driven (engine torque)</label>`;
        const ws = GT.WHEELS[w.size], br = GT.BRAKES[w.brake], ty = GT.TIRES[w.tire];
        h += kv([['Unsprung (pair)', `${(2 * (ws.mass + br.mass)).toFixed(0)} kg`], ['Tyre μ', ty.mu.toFixed(2)], ['Brake torque', `${br.torque} N·m`]]);
      }
      if (pp.engine) {
        const e = GT.ENGINES[d.engine.type], pk = GT.peakPower(e);
        h += `<div class="sub">ENGINE<button data-act="rm-engine">remove</button></div>`;
        h += fld('Type', sel('engine:0:type', GT.ENGINES, d.engine.type));
        h += kv([['Peak power', `${pk.kw.toFixed(0)} kW @ ${pk.rpm} rpm`], ['Peak torque', `${GT.peakTorque(e)} N·m`], ['Redline', `${e.redline} rpm`], ['Mass', `${e.mass} kg`]]);
      }
      if (pp.driver) h += `<div class="sub">DRIVER SEAT<button data-act="rm-driver">remove</button></div><div class="small dim">80 kg. Run ends if this node touches the ground.</div>`;
      for (const w of pp.wings) {
        const sp = GT.WINGS[w.type];
        h += `<div class="sub">AERO<button data-act="rm-wing" data-id="${w.id}">remove</button></div>`;
        h += fld('Type', sel(`wing:${w.id}:type`, GT.WINGS, w.type));
        h += kv([['Downforce @200 km/h', `${(0.5 * GT.RHO * 55.56 ** 2 * sp.ClA / 1000).toFixed(2)} kN`], ['Drag @200 km/h', `${(0.5 * GT.RHO * 55.56 ** 2 * sp.CdA).toFixed(0)} N`]]);
      }
    } else if (s.kind === 'beam') {
      const p = GT.beamProps(GT.nodeMapOf(d), obj);
      h += `<h4>Tube #${obj.id} <span class="tag">${p ? (p.L * 1000).toFixed(0) + ' mm' : ''}</span></h4>`;
      h += fld('Material', sel(`beam:${obj.id}:mat`, GT.MATERIALS, obj.mat));
      h += fld('Section', `<select data-f="beam:${obj.id}:tube">${GT.TUBES.map(t => `<option${t.id === obj.tube ? ' selected' : ''}>${t.id}</option>`).join('')}</select>`);
      if (p) {
        const st = App.staticCheck && App.staticCheck.util.get(obj.id);
        h += kv([
          ['Mass (pair)', `${p.mass.toFixed(2)} kg`],
          ['Axial stiffness', `${(p.k / 1e6).toFixed(1)} MN/m`],
          ['Tension limit', kN(p.Ft)],
          ['Compression limit', kN(p.Fc) + (p.buckling ? ' (buckling)' : ' (yield)'), p.buckling ? 'warnc' : ''],
          ['Cost', `$${p.cost.toFixed(0)}`],
        ].concat(st != null ? [['Static utilisation', `${(st * 100).toFixed(1)} %`, st > 0.5 ? 'badc' : 'good']] : []));
      }
    } else if (s.kind === 'spring') {
      const nm = GT.nodeMapOf(d), A = nm.get(obj.a), B = nm.get(obj.b), L = Math.hypot(B.x - A.x, B.y - A.y);
      h += `<h4>Coilover #${obj.id} <span class="tag">${(L * 1000).toFixed(0)} mm</span></h4>`;
      h += `<div class="grid2">${fld('Spring (kN/m)', num(`spring:${obj.id}:k`, obj.k, 5, 1000, 5, 500))}${fld('Damping (N·s/m)', num(`spring:${obj.id}:c`, obj.c, 250, 1, 0, 30000))}</div>`;
      h += `<div class="grid2">${fld('Bump travel (mm)', num(`spring:${obj.id}:travel`, obj.travel, 5, 0.001, 20, 300))}${fld('Preload adj. (mm)', num(`spring:${obj.id}:adj`, obj.adj, 5, 1, -100, 100))}</div>`;
      h += `<label class="check"><input type="checkbox" data-f="spring:${obj.id}:auto"${obj.auto !== false ? ' checked' : ''}> Auto preload (hold drawn ride height)</label>`;
      h += `<p class="small dim">Droop limit = 60 % of bump travel. Running out of bump travel hits a polyurethane bump stop.</p>`;
      const sc = App.staticCheck && App.staticCheck.sag.get(obj.id);
      if (sc != null) h += kv([['Static sag', `${(sc * 1000).toFixed(0)} mm of ${(obj.travel * 1000).toFixed(0)}`, sc > obj.travel * 0.7 ? 'badc' : 'good']]);
    }
    $('#inspector').innerHTML = h;
  }

  $('#inspector').addEventListener('change', e => {
    const el = e.target, f = el.dataset.f; if (!f) return;
    const [kind, idS, key] = f.split(':'), id = +idS;
    let v;
    if (el.type === 'checkbox') v = el.checked;
    else if (el.type === 'number') { v = parseFloat(el.value) * parseFloat(el.dataset.scale || '1'); if (!isFinite(v)) { if (key !== 'finalDrive') return refreshPanels(); v = null; } }
    else v = el.value;
    if (el.type === 'number' && v != null) {
      if (el.min !== '') v = Math.max(v, parseFloat(el.min) * parseFloat(el.dataset.scale || '1'));
      if (el.max !== '') v = Math.min(v, parseFloat(el.max) * parseFloat(el.dataset.scale || '1'));
    }
    mutate(d => {
      let o = null;
      if (kind === 'node') o = d.nodes.find(n => n.id === id);
      else if (kind === 'beam') o = d.beams.find(n => n.id === id);
      else if (kind === 'spring') o = d.springs.find(n => n.id === id);
      else if (kind === 'wheel') o = d.wheels.find(n => n.id === id);
      else if (kind === 'wing') o = d.wings.find(n => n.id === id);
      else if (kind === 'engine') o = d.engine;
      else if (kind === 'setup') o = key === 'name' ? d : d.setup;
      if (!o) return;
      if (kind === 'node') v = r3(v);
      if (key === 'driven' && v === false && !d.wheels.some(w => w.driven && w.id !== id)) toast('No driven axle left — the car won\'t move.', true);
      o[key] = v;
      if (kind === 'beam') { if (key === 'mat') App.mat = v; if (key === 'tube') App.tube = v; syncStock(); }
    });
  });
  $('#inspector').addEventListener('click', e => {
    const b = e.target.closest('[data-act]'); if (!b) return;
    const id = +b.dataset.id;
    mutate(d => {
      if (b.dataset.act === 'rm-wheel') d.wheels = d.wheels.filter(w => w.id !== id);
      if (b.dataset.act === 'rm-wing') d.wings = d.wings.filter(w => w.id !== id);
      if (b.dataset.act === 'rm-engine') d.engine = null;
      if (b.dataset.act === 'rm-driver') d.driver = null;
    });
  });

  const MASS_COLORS = { frame: '#8d99a6', joints: '#5d6977', suspension: '#f4c542', wheels: '#444b55', engine: '#c0392b', driver: '#2c6fd1', aero: '#ff7a2f', body: '#3ddc84', systems: '#1f8a5b' };
  function refreshStats() {
    const d = D(), mm = GT.massModel(d), ax = GT.axleLoads(d, mm);
    const e = d.engine ? GT.ENGINES[d.engine.type] : null, pk = e ? GT.peakPower(e) : null;
    let h = `<h4>Spec sheet</h4>`;
    const B = mm.breakdown, tot = mm.total || 1;
    h += `<div class="massbar">${Object.entries(B).filter(([, v]) => v > 0).map(([k, v]) => `<i style="width:${v / tot * 100}%;background:${MASS_COLORS[k]}" title="${k}: ${v.toFixed(1)} kg"></i>`).join('')}</div>`;
    h += `<div class="masslegend">${Object.entries(B).filter(([, v]) => v > 0.5).map(([k, v]) => `<span><b style="background:${MASS_COLORS[k]}"></b>${k} ${v.toFixed(0)}</span>`).join('')}</div>`;
    const rows = [['Kerb mass', `${mm.total.toFixed(0)} kg`]];
    if (ax) {
      const f = ax.front * 100, good = f > 35 && f < 55;
      rows.push(['Weight split F / R', `${f.toFixed(0)} / ${(100 - f).toFixed(0)}`, good ? 'good' : 'warnc']);
      rows.push(['Wheelbase', `${(ax.wheelbase * 1000).toFixed(0)} mm`]);
    }
    if (mm.cog) rows.push(['CoG height', `${(mm.cog.y * 1000).toFixed(0)} mm`, mm.cog.y > 0.55 ? 'warnc' : '']);
    rows.push(['Unsprung', `${mm.unsprung.toFixed(0)} kg (${(mm.unsprung / tot * 100).toFixed(0)} %)`]);
    if (pk) {
      rows.push(['Power', `${pk.kw.toFixed(0)} kW / ${(pk.kw * 1.341).toFixed(0)} hp`]);
      rows.push(['Power-to-weight', `${(pk.kw * 1.341 / (tot / 1000)).toFixed(0)} hp/t`]);
    }
    const body = GT.BODIES[d.setup.body] || GT.BODIES.none;
    const aero = d.wings.reduce((a, w) => { const s = GT.WINGS[w.type]; return s ? [a[0] + s.ClA, a[1] + s.CdA] : a; }, [body.ClA || 0, body.CdA]);
    rows.push(['Aero ClA / CdA', `${aero[0].toFixed(2)} / ${aero[1].toFixed(2)} m²`]);
    rows.push(['Frame', `${d.beams.length} tubes · ${mm.tubeLen.toFixed(1)} m · ${B.frame.toFixed(1)} kg`]);
    rows.push(['Build cost', `$${Math.round(mm.cost).toLocaleString()}`]);
    h += kv(rows);
    const sc = App.staticCheck;
    if (sc) {
      h += `<div class="sub">STATIC LOAD CHECK</div>`;
      h += kv([
        ['Max tube utilisation', `${(sc.maxUtil * 100).toFixed(1)} %`, sc.maxUtil > 0.5 ? 'badc' : sc.maxUtil > 0.25 ? 'warnc' : 'good'],
        ['Settled ride height', `${(sc.ride * 1000).toFixed(0)} mm`, sc.ride < 0.04 ? 'badc' : 'good'],
        ['Settled pitch', `${(sc.pitch * 180 / Math.PI).toFixed(2)}°`],
        ['Broken while parked', `${sc.broken}`, sc.broken ? 'badc' : 'good'],
      ]);
      h += `<div class="small dim" style="margin-top:6px">Tubes are coloured by utilisation. Any edit clears this.</div>`;
    }
    $('#stats').innerHTML = h;
  }

  function refreshWarnings() {
    const v = GT.validate(D());
    let h = `<h4>Engineering review</h4>`;
    h += v.errors.map(m => `<div class="msg err">${esc(m)}</div>`).join('');
    h += v.warns.map(m => `<div class="msg wrn">${esc(m)}</div>`).join('');
    if (!v.errors.length && !v.warns.length) h += `<div class="msg okm">Frame is statically determinate or better. Ready for the proving ground.</div>`;
    $('#warnings').innerHTML = h;
  }

  function runStatic() {
    const v = GT.validate(D());
    if (v.errors.length) { toast(v.errors[0], true); return; }
    const sim = new GT.Sim(D(), GT.flatTerrain());
    const util = new Map(), sag = new Map();
    const F = 150, AV = 40;
    for (let i = 0; i < F; i++) {
      sim.step(1 / 60, { throttle: 0, brake: 1 });
      if (i >= F - AV) for (const b of sim.beams) util.set(b.id, (util.get(b.id) || 0) + (b.broken ? 2 : b.ratio) / AV);
    }
    for (const s of sim.springs) sag.set(s.id, s.Ld - s.len);
    let ride = Infinity;
    for (const n of sim.nodes) if (!n.hub) ride = Math.min(ride, n.y - sim.terrain.height(n.x));
    let mx = 0; util.forEach(u => { mx = Math.max(mx, u); });
    App.staticCheck = { util, sag, maxUtil: mx, ride: isFinite(ride) ? ride : 0, pitch: sim.pitch, broken: sim.broken };
    refreshPanels();
    toast(sim.broken ? `${sim.broken} tube(s) failed just sitting there!` : `Parked fine. Max tube utilisation ${(mx * 100).toFixed(1)} %.`, !!sim.broken);
  }

  // ---------- proving ground ----------
  function setMode(m) {
    if (m === 'test') {
      const v = GT.validate(D());
      if (v.errors.length) { toast('Not road-legal yet: ' + v.errors[0], true); return; }
      App.buildCam = Object.assign({}, R.cam);
    }
    App.mode = m;
    $('#modeBuild').classList.toggle('active', m === 'build');
    $('#modeTest').classList.toggle('active', m === 'test');
    $('#buildLeft').hidden = m !== 'build'; $('#testLeft').hidden = m !== 'test';
    $('#buildRight').hidden = m !== 'build'; $('#testRight').hidden = m !== 'test';
    $('#hud').hidden = m !== 'test';
    canvas.style.cursor = m === 'build' ? 'crosshair' : 'default';
    if (m === 'build') {
      App.runner = null; if (App.buildCam) Object.assign(R.cam, App.buildCam);
      setTool(App.tool); refreshPanels();
    } else {
      $('#hint').textContent = 'Pick a test on the left. R restart · P pause · M slow-mo · wheel zoom · drag to pan (C re-centres)';
      renderTestList(); renderResults(); renderHistory();
      runTest(App.testId || 'drag');
    }
  }

  function renderTestList() {
    $('#testList').innerHTML = Object.entries(GT.TESTS).map(([id, t]) =>
      `<button class="testcard${id === App.testId ? ' active' : ''}" data-test="${id}" title="${esc(t.desc)}"><span class="ic">${t.icon}</span><b>${esc(t.name)}</b><small>${esc(t.sub)}</small></button>`).join('');
  }
  $('#testList').addEventListener('click', e => { const b = e.target.closest('[data-test]'); if (b) runTest(b.dataset.test); });

  function runTest(id) {
    App.testId = id;
    App.runner = new GT.TestRunner(D(), id);
    App.paused = false; App.follow = true; App.reported = false;
    R.cam.s = 75;
    const dn = App.runner.sim.nodes[App.runner.sim.driver];
    R.cam.x = dn.x + 2; R.cam.y = dn.y + 0.8;
    renderTestList(); renderResults();
  }

  function renderResults() {
    const r = App.runner, t = r ? r.test : null;
    if (!r) { $('#results').innerHTML = '<h4>Results</h4><p class="small dim">No run yet.</p>'; return; }
    let h = `<h4>${t.icon} ${esc(t.name)} ${r.done ? (r.result.free ? '<span class="badge run">ENDED</span>' : `<span class="badge ${r.result.ok ? 'pass' : 'fail'}">${r.result.ok ? 'PASS' : 'FAIL'}</span>`) : '<span class="badge run">RUNNING</span>'}</h4>`;
    h += `<p class="small dim" style="margin-top:0">${esc(t.desc)}</p>`;
    if (r.done) {
      const res = r.result;
      if (res.reason) h += `<div class="msg err">${esc(res.reason)}</div>`;
      else if (!res.ok) h += `<div class="msg err">Frame damaged — ${r.sim.broken} tube(s) failed.</div>`;
      h += `<canvas id="chart"></canvas>`;
      h += kv(res.rows);
      const ev = res.events.filter(e => e.kind !== 'rim');
      if (ev.length) h += `<div class="sub">STRUCTURAL EVENTS</div><div class="evlist">${ev.map(e => `<div>${e.t.toFixed(2)} s · ${esc(e.msg)}</div>`).join('')}</div>`;
      h += `<button class="wide primary" data-act="rerun">Run again <kbd>R</kbd></button><button class="wide" data-act="garage">← Back to garage to tweak</button>`;
    } else if (t.free) {
      h += `<button class="wide" data-act="endfree">End drive &amp; show results</button>`;
    }
    $('#results').innerHTML = h;
    if (r.done) drawChart($('#chart'), r.samples);
  }
  $('#results').addEventListener('click', e => {
    const b = e.target.closest('[data-act]'); if (!b) return;
    if (b.dataset.act === 'rerun') runTest(App.testId);
    if (b.dataset.act === 'garage') setMode('build');
    if (b.dataset.act === 'endfree' && App.runner) App.runner.finish();
  });

  function drawChart(cv, S) {
    if (!cv || S.length < 2) return;
    const dpr = window.devicePixelRatio || 1, w = cv.clientWidth, h = cv.clientHeight;
    cv.width = w * dpr; cv.height = h * dpr;
    const c = cv.getContext('2d'); c.scale(dpr, dpr);
    const T = S[S.length - 1].t, vmax = Math.max(50, ...S.map(s => s.v)) * 1.1;
    const X = t => 30 + (t / T) * (w - 36), Yv = v => h - 14 - v / vmax * (h - 24), Yu = u => h - 14 - Math.min(u, 1.2) / 1.2 * (h - 24);
    c.strokeStyle = '#1e2733'; c.lineWidth = 1; c.font = '9px ui-monospace, Menlo, monospace'; c.fillStyle = '#5d6d80';
    for (let v = 0; v < vmax; v += 50) { c.beginPath(); c.moveTo(30, Yv(v)); c.lineTo(w - 6, Yv(v)); c.stroke(); c.fillText(v, 4, Yv(v) + 3); }
    c.fillText(T.toFixed(1) + ' s', w - 40, h - 3);
    c.strokeStyle = '#ff3b3066'; c.beginPath(); S.forEach((s, i) => (i ? c.lineTo(X(s.t), Yu(s.u)) : c.moveTo(X(s.t), Yu(s.u)))); c.stroke();
    c.strokeStyle = '#ff7a2f'; c.lineWidth = 1.8; c.beginPath(); S.forEach((s, i) => (i ? c.lineTo(X(s.t), Yv(s.v)) : c.moveTo(X(s.t), Yv(s.v)))); c.stroke();
    c.fillStyle = '#ff7a2f'; c.fillText('km/h', 34, 10); c.fillStyle = '#ff6b63'; c.fillText('frame utilisation', 70, 10);
  }

  function renderHistory() {
    const H = App.history;
    $('#history').innerHTML = `<h4>Run log</h4>` + (H.length ? `<table class="hist">${H.slice().reverse().map(x => `<tr><td>${x.ok == null ? '·' : x.ok ? '✅' : '❌'} ${esc(x.test)}</td><td class="dim">${esc(x.car)}</td><td>${esc(x.head)}</td></tr>`).join('')}</table>` : '<p class="small dim">Completed runs land here so you can compare tweaks.</p>');
  }

  // ---------- HUD ----------
  const hud = $('#hud');
  hud.innerHTML = `
    <div class="hudbox speedo"><span class="gear" id="hGear">1</span><div class="v"><span id="hSpd">0</span><small>km/h</small></div>
      <div class="rpm"><i id="hRpm"></i></div><div class="rpmt" id="hRpmT"></div>
      <div class="pedals"><div><i id="hThr" style="background:#3ddc84"></i></div><div><i id="hBrk" style="background:#ff3b30"></i></div></div></div>
    <div class="hudbox hudgrid" id="hGrid"></div>
    <div class="hudbox events" id="hEv"></div>
    <div id="hBanner"></div>`;
  function updateHud() {
    const r = App.runner; if (!r) return;
    const s = r.sim, e = s.engine;
    $('#hSpd').textContent = Math.abs(s.speed * 3.6).toFixed(0);
    $('#hGear').textContent = e ? (e.gear === -1 ? 'R' : e.shiftT > 0 ? '–' : e.gear) : '-';
    if (e) {
      $('#hRpm').style.width = clamp(e.rpm / e.spec.redline * 100, 0, 100) + '%';
      $('#hRpmT').textContent = `${e.rpm.toFixed(0)} rpm · ${Math.max(0, e.tq).toFixed(0)} N·m`;
    }
    $('#hThr').style.width = ((r.input.throttle || 0) * 100) + '%';
    $('#hBrk').style.width = ((r.input.brake || 0) * 100) + '%';
    const intact = s.beams.filter(b => !b.broken).length;
    const sliding = s.wheels.some(w => w.contact && Math.abs(w.slip) > 1.5);
    $('#hGrid').innerHTML = [
      ['time', r.t.toFixed(2) + ' s'], ['distance', Math.max(0, r.dist).toFixed(1) + ' m'],
      ['long g', s.gLong.toFixed(2)], ['vert g', s.gVert.toFixed(2)],
      ['frame load', `<span style="color:${GT.stressColor(s.maxUtil)}">${(s.maxUtil * 100).toFixed(0)} %</span>`],
      ['integrity', `<span style="color:${intact < s.beams.length ? '#ff4d4d' : '#3ddc84'}">${intact}/${s.beams.length}</span>`],
      ['downforce', (s.downforce / 1000).toFixed(2) + ' kN'],
      ['tyres', sliding ? '<span style="color:#ff4d4d">SLIP</span>' : (s.wheels.every(w => !w.contact) ? '<span style="color:#ffb020">AIR</span>' : 'grip')],
    ].map(([k, v]) => `<span>${k}</span><span>${v}</span>`).join('');
    $('#hEv').innerHTML = s.events.slice(-6).reverse().map(ev => `<div class="${ev.kind}">${ev.t.toFixed(2)}s ${esc(ev.msg)}</div>`).join('') || '<div style="color:#5d6d80">no structural events</div>';
    const b = $('#hBanner');
    if (r.done) {
      const res = r.result;
      b.innerHTML = res.free ? `<div class="banner">DRIVE ENDED<small>${esc(res.headline)}</small></div>`
        : `<div class="banner${res.ok ? '' : ' fail'}">${res.ok ? 'PASS' : 'FAIL'}<small>${esc(res.reason || res.headline)}</small></div>`;
    } else b.innerHTML = App.paused ? '<div class="banner" style="border-color:#ffb020;color:#ffb020">PAUSED</div>' : '';
  }

  // ---------- main loop ----------
  let last = performance.now(), acc = 0;
  function frame(now) {
    const dt = Math.min(0.05, (now - last) / 1000); last = now;
    const r = App.runner;
    if (App.mode === 'test' && r && !App.paused && !r.done) {
      acc += dt * (App.slow ? 0.25 : 1);
      let n = 0;
      while (acc >= 1 / 60 && n < 4) { r.step(1 / 60, App.keys); acc -= 1 / 60; n++; }
      if (n === 4) acc = 0;
    }
    draw(dt);
    requestAnimationFrame(frame);
  }

  function draw(dt) {
    if (App.mode === 'test' && App.runner) {
      const r = App.runner;
      if (r.done && !App.reported) {
        App.reported = true;
        App.history.push({ test: r.test.name, car: D().name, head: r.result.headline, ok: r.result.free ? null : r.result.ok });
        if (App.history.length > 30) App.history.shift();
        renderResults(); renderHistory();
      }
      if (App.follow) {
        const dn = r.sim.nodes[r.sim.driver];
        const k = Math.abs(dn.x - R.cam.x) * R.cam.s > R.w ? 1 : Math.min(1, dt * 4);
        R.cam.x += (dn.x + clamp(r.sim.speed * 0.12, -4, 6) - R.cam.x) * k;
        R.cam.y += (dn.y + 0.6 - R.cam.y) * k;
      }
      R.begin(true); R.backdrop(); R.terrain(r.terrain); R.drawSim(r.sim, { stress: App.stress });
      updateHud();
    } else {
      R.begin(false); R.grid();
      const d = D(), mm = GT.massModel(d);
      const dr = App.drag;
      let preview = null;
      if (dr && dr.type === 'link') {
        const L = Math.hypot(dr.tx - dr.x, dr.ty - dr.y);
        if (L > 0.01) preview = { x1: dr.x, y1: dr.y, x2: dr.tx, y2: dr.ty, type: App.tool, ok: L <= GT.MAX_BEAM };
      }
      R.drawDesign(d, { sel: App.sel, hover: App.hover, hoverLink: App.hoverLink, util: App.staticCheck && App.staticCheck.util, mm, axle: GT.axleLoads(d, mm), preview, labels: App.labels, cursor: App.tool !== 'select' && App.tool !== 'erase' ? App.cursor : null });
    }
  }
  App.draw = draw;

  // ---------- toast / persistence ----------
  let toastT = 0;
  function toast(msg, bad) {
    const t = $('#toast'); t.textContent = msg; t.classList.toggle('bad', !!bad); t.classList.add('show');
    clearTimeout(toastT); toastT = setTimeout(() => t.classList.remove('show'), 2600);
  }
  function garage() { try { return JSON.parse(localStorage.getItem(LS_GARAGE) || '{}'); } catch (e) { return {}; } }
  function refreshLoadSel() {
    const g = garage();
    $('#loadSel').innerHTML = `<option value="">— Load design… —</option><optgroup label="Presets">${Object.entries(GT.PRESETS).map(([k, p]) => `<option value="p:${k}">${esc(p.name)}</option>`).join('')}</optgroup>` +
      (Object.keys(g).length ? `<optgroup label="Your garage">${Object.keys(g).map(n => `<option value="g:${esc(n)}">${esc(n)}</option>`).join('')}</optgroup>` : '');
  }
  $('#loadSel').addEventListener('change', e => {
    const v = e.target.value; e.target.value = '';
    if (!v) return;
    if (v.startsWith('p:')) setDesign(GT.PRESETS[v.slice(2)].make());
    else { const g = garage()[v.slice(2)]; if (g) setDesign(g); }
    if (App.mode !== 'build') setMode('build');
    fitView(); toast('Loaded ' + D().name);
  });
  $('#btnNew').onclick = () => { setDesign(GT.emptyDesign('My chassis')); if (App.mode !== 'build') setMode('build'); toast('Fresh sheet. Ctrl+Z brings the old one back.'); };
  $('#btnSave').onclick = () => {
    const name = (D().name || '').trim() || 'Untitled';
    try { const g = garage(); g[name] = D(); localStorage.setItem(LS_GARAGE, JSON.stringify(g)); refreshLoadSel(); toast(`Saved “${name}” to your garage`); }
    catch (e) { toast('Could not save: browser storage unavailable', true); }
  };
  $('#btnExport').onclick = () => {
    const a = document.createElement('a');
    a.href = URL.createObjectURL(new Blob([JSON.stringify(D(), null, 1)], { type: 'application/json' }));
    a.download = (D().name || 'chassis').replace(/[^\w-]+/g, '_') + '.chassis.json';
    a.click(); setTimeout(() => URL.revokeObjectURL(a.href), 1000);
  };
  $('#btnImport').onclick = () => $('#fileInput').click();
  $('#fileInput').onchange = e => {
    const f = e.target.files[0]; if (!f) return;
    f.text().then(t => { setDesign(JSON.parse(t)); setMode('build'); fitView(); toast('Imported ' + D().name); }).catch(() => toast('Not a valid chassis file', true));
    e.target.value = '';
  };

  // ---------- left panel wiring ----------
  $('#tools').innerHTML = TOOLS.map(([id, key, ic, name]) => `<button class="tool" data-tool="${id}" title="${esc(name)}"><span class="ic">${ic}</span>${esc(name)}<kbd>${key}</kbd></button>`).join('');
  $('#tools').addEventListener('click', e => { const b = e.target.closest('[data-tool]'); if (b) setTool(b.dataset.tool); });
  $('#matSel').innerHTML = opts(GT.MATERIALS, App.mat);
  $('#tubeSel').innerHTML = GT.TUBES.map(t => `<option>${t.id}</option>`).join('');
  function syncStock() { $('#matSel').value = App.mat; $('#tubeSel').value = App.tube; refreshTubeInfo(); }
  const applyStockToSel = () => { const s = App.sel; if (s && s.kind === 'beam') mutate(d => { const b = d.beams.find(x => x.id === s.id); if (b) { b.mat = App.mat; b.tube = App.tube; } }); else refreshTubeInfo(); };
  $('#matSel').onchange = e => { App.mat = e.target.value; applyStockToSel(); };
  $('#tubeSel').onchange = e => { App.tube = e.target.value; applyStockToSel(); };
  $('#btnApplyAll').onclick = () => { mutate(d => d.beams.forEach(b => { b.mat = App.mat; b.tube = App.tube; })); toast(`All tubes → ${GT.MATERIALS[App.mat].short} ${App.tube}`); };
  $('#snapChk').onchange = e => { App.snap = e.target.checked; };
  $('#labelChk').onchange = e => { App.labels = e.target.checked; };
  $('#btnFit').onclick = fitView;
  $('#btnStatic').onclick = runStatic;
  $('#stressChk').onchange = e => { App.stress = e.target.checked; };
  $('#slowChk').onchange = e => { App.slow = e.target.checked; };
  $('#btnBack').onclick = () => setMode('build');
  $('#modeBuild').onclick = () => setMode('build');
  $('#modeTest').onclick = () => setMode('test');
  new ResizeObserver(() => R.resize()).observe(canvas);

  // ---------- boot ----------
  let start = null;
  try { const s = localStorage.getItem(LS_CUR); if (s) start = JSON.parse(s); } catch (e) { start = null; }
  App.design = GT.normalize(start && start.nodes && start.nodes.length ? start : GT.PRESETS.starter.make());
  refreshLoadSel(); syncStock(); setTool('beam'); refreshPanels();
  requestAnimationFrame(() => { R.resize(); fitView(); requestAnimationFrame(frame); });
})(window.GT);
