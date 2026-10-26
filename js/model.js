// Design model: section properties, mass model, validation, presets.
(function (GT) {
  'use strict';

  GT.tube = function (id) {
    const t = GT.TUBES.find(x => x.id === id) || GT.TUBES[2];
    const d = t.D - 2 * t.t;
    return { id: t.id, D: t.D, t: t.t, A: Math.PI / 4 * (t.D * t.D - d * d), I: Math.PI / 64 * (t.D ** 4 - d ** 4) };
  };

  GT.nodeMapOf = d => new Map(d.nodes.map(n => [n.id, n]));

  // Properties of a tube PAIR (left + right side).
  GT.beamProps = function (nodeMap, b) {
    const A = nodeMap.get(b.a), B = nodeMap.get(b.b);
    if (!A || !B) return null;
    const L = Math.hypot(B.x - A.x, B.y - A.y);
    const m = GT.MATERIALS[b.mat] || GT.MATERIALS.mild;
    const t = GT.tube(b.tube);
    const EA2 = 2 * m.E * t.A;
    const Ft = 2 * m.yield * t.A;
    const Fyc = 2 * (m.compYield || m.yield) * t.A;
    const Le = GT.K_EFF * Math.max(L, 1e-3);
    const Pcr = 2 * Math.PI * Math.PI * m.E * t.I / (Le * Le);   // Euler buckling
    const mass = 2 * t.A * m.rho * L;
    return { L, mass, EA2, k: EA2 / Math.max(L, 1e-3), Ft, Fc: Math.min(Fyc, Pcr), Pcr, buckling: Pcr < Fyc, mat: m, tube: t, cost: mass * m.costKg + 25 * L };
  };

  GT.emptyDesign = function (name) {
    return { version: 2, name: name || 'Untitled', nodes: [], beams: [], springs: [], panels: [], wheels: [], engine: null, driver: null, tank: null, wings: [], aero: null, setup: Object.assign({ paint: '#d8452b', finalDrive: null }, GT.SETUP_DEFAULTS), nextId: 1 };
  };

  // Fill in anything missing from an imported / old design.
  GT.normalize = function (d) {
    const e = GT.emptyDesign(d && d.name);
    d = Object.assign(e, d || {});
    d.setup = Object.assign({ paint: '#d8452b', finalDrive: null }, GT.SETUP_DEFAULTS, d.setup || {});
    if (d.tank) d.tank = Object.assign({ size: 'l40', type: 'steel', fill: 0.6 }, d.tank);
    delete d.setup.body;   // v1 prefab bodies are replaced by drawn panels
    for (const k of ['nodes', 'beams', 'springs', 'panels', 'wheels', 'wings']) if (!Array.isArray(d[k])) d[k] = [];
    d.panels.forEach(p => { if (!p.mat) p.mat = 'grp'; });
    d.springs.forEach(s => { if (s.k == null) s.k = 60000; if (s.c == null) s.c = 3000; if (s.travel == null) s.travel = 0.1; if (s.auto == null) s.auto = true; if (s.adj == null) s.adj = 0; });
    let max = 0;
    for (const k of ['nodes', 'beams', 'springs', 'panels', 'wheels', 'wings']) d[k].forEach(o => { max = Math.max(max, o.id || 0); });
    d.nextId = Math.max(d.nextId || 1, max + 1);
    return d;
  };

  GT.massModel = function (d) {
    const mass = new Map();
    d.nodes.forEach(n => mass.set(n.id, GT.JOINT_MASS));
    const add = (id, m) => { if (mass.has(id)) mass.set(id, mass.get(id) + m); };
    const nm = GT.nodeMapOf(d);
    const B = { frame: 0, joints: d.nodes.length * GT.JOINT_MASS, suspension: 0, wheels: 0, engine: 0, driver: 0, aero: 0, body: 0, systems: 0, drivetrain: 0, fuel: 0 };
    let bodyArea = 0;
    let cost = d.nodes.length * GT.JOINT_COST, tubeLen = 0;
    for (const b of d.beams) {
      const p = GT.beamProps(nm, b); if (!p) continue;
      B.frame += p.mass; tubeLen += p.L; add(b.a, p.mass / 2); add(b.b, p.mass / 2); cost += p.cost;
    }
    for (const p of d.panels || []) {
      const pp = GT.panelProps(nm, p); if (!pp) continue;
      B.body += pp.mass; bodyArea += pp.area; add(p.a, pp.mass / 2); add(p.b, pp.mass / 2); cost += pp.cost;
    }
    for (const s of d.springs) { B.suspension += GT.SPRING.mass; add(s.a, GT.SPRING.mass / 2); add(s.b, GT.SPRING.mass / 2); cost += GT.SPRING.cost; }
    const hubs = new Set();
    for (const w of d.wheels) {
      const ws = GT.WHEELS[w.size] || GT.WHEELS.r18, ty = GT.TIRES[w.tire] || GT.TIRES.street, br = GT.BRAKES[w.brake] || GT.BRAKES.street;
      const m = 2 * (ws.mass + br.mass);
      B.wheels += m; add(w.node, m); hubs.add(w.node);
      cost += 2 * (ws.cost + ty.cost) + br.cost;
    }
    if (d.engine && mass.has(d.engine.node)) { const e = GT.ENGINES[d.engine.type] || GT.ENGINES.i4t; B.engine = e.mass; add(d.engine.node, e.mass); cost += e.cost; }
    const pm = GT.powertrainMass(d);
    cost += pm.cost;
    if (d.engine && mass.has(d.engine.node)) { B.drivetrain = pm.gearbox + pm.electronics + pm.cooling; add(d.engine.node, B.drivetrain); }
    if (d.tank && mass.has(d.tank.node)) { B.fuel = pm.tankDry + pm.fuel; add(d.tank.node, B.fuel); }
    if (d.driver && mass.has(d.driver.node)) { B.driver = GT.DRIVER_MASS; add(d.driver.node, GT.DRIVER_MASS); }
    for (const w of d.wings) { const s = GT.WINGS[w.type]; if (!s || !mass.has(w.node)) continue; B.aero += s.mass; add(w.node, s.mass); cost += s.cost; }
    const carriers = d.nodes.filter(n => !hubs.has(n.id));
    if (carriers.length) {
      B.systems = GT.SYSTEMS.mass; cost += GT.SYSTEMS.cost;
      carriers.forEach(n => add(n.id, GT.SYSTEMS.mass / carriers.length));
    }
    let total = 0, cx = 0, cy = 0, unsprung = 0;
    for (const n of d.nodes) { const m = mass.get(n.id); total += m; cx += m * n.x; cy += m * n.y; if (hubs.has(n.id)) unsprung += m; }
    return { mass, total, unsprung, cog: total ? { x: cx / total, y: cy / total } : null, breakdown: B, cost, hubs, tubeLen, bodyArea, nm, fuelKg: pm.fuel };
  };

  // Static axle split from the extreme axles (lever rule).
  GT.axleLoads = function (d, mm) {
    if (d.wheels.length < 2 || !mm.cog) return null;
    const nm = mm.nm || GT.nodeMapOf(d);
    const xs = d.wheels.map(w => nm.get(w.node)).filter(Boolean).map(n => n.x);
    if (xs.length < 2) return null;
    const rearX = Math.min(...xs), frontX = Math.max(...xs);
    if (frontX - rearX < 0.3) return null;
    const f = (mm.cog.x - rearX) / (frontX - rearX);
    return { rearX, frontX, wheelbase: frontX - rearX, front: f, rear: 1 - f };
  };

  GT.validate = function (d) {
    const errors = [], warns = [];
    if (!d.nodes.length) { errors.push('Empty design — drag out some tubes with the Tube tool (B).'); return { errors, warns, dof: 0 }; }
    if (!d.driver) errors.push('No driver seat (D). Someone has to drive this thing.');
    if (!d.engine) errors.push('No engine (E).');
    const espec = d.engine && GT.ENGINES[d.engine.type];
    if (espec && !espec.ev && !d.tank) errors.push('No fuel tank (T). Petrol engines need somewhere to drink from.');
    if (espec && !espec.ev && d.tank && !(d.tank.fill > 0.005)) errors.push('Fuel tank is empty — set a fill level.');
    if (espec && espec.ev && d.tank) warns.push('Fuel tank on an EV: dead weight.');
    if (d.wheels.length < 2) errors.push('Needs at least two axles (W).');
    else if (!d.wheels.some(w => w.driven)) errors.push('No driven axle — select a wheel node and tick "Driven".');

    const adj = new Map(d.nodes.map(n => [n.id, []]));
    const links = d.beams.concat(d.springs);
    for (const l of links) { if (adj.has(l.a) && adj.has(l.b)) { adj.get(l.a).push(l.b); adj.get(l.b).push(l.a); } }
    for (const p of d.panels || []) { if (adj.has(p.a) && adj.has(p.b)) { adj.get(p.a).push(p.b); adj.get(p.b).push(p.a); } }
    const root = d.driver ? d.driver.node : d.nodes[0].id;
    const seen = new Set([root]), q = [root];
    while (q.length) { const c = q.pop(); for (const o of adj.get(c) || []) if (!seen.has(o)) { seen.add(o); q.push(o); } }
    if (d.engine && !seen.has(d.engine.node)) errors.push('Engine is not connected to the driver\'s frame.');
    if (d.wheels.some(w => !seen.has(w.node))) errors.push('An axle is not connected to the main frame.');
    if (d.tank && !seen.has(d.tank.node)) errors.push('Fuel tank is not connected to the frame.');
    const loose = d.nodes.filter(n => !seen.has(n.id));
    if (loose.length) warns.push(`${loose.length} node(s) are not connected to the main frame — they will fall off.`);

    // Maxwell's rule for planar pin-jointed frames: m >= 2j - 3
    const n = seen.size;
    let m = 0; for (const l of links) if (seen.has(l.a) && seen.has(l.b)) m++;
    const dof = Math.max(0, 2 * n - 3 - m);
    if (n >= 3 && dof > 0) warns.push(`Frame is a mechanism: ${dof} free degree(s) of freedom. Tubes are pin-jointed and only carry axial load — triangulate or it will fold.`);
    for (const nd of d.nodes) if (seen.has(nd.id) && (adj.get(nd.id) || []).length === 1) warns.push(`Node #${nd.id} hangs on a single tube — it will flail.`);

    const held = new Set(); links.forEach(l => { held.add(l.a); held.add(l.b); });
    const skinOnly = new Set();
    (d.panels || []).forEach(p => [p.a, p.b].forEach(id => { if (!held.has(id)) skinOnly.add(id); }));
    if (skinOnly.size) warns.push(`${skinOnly.size} body node(s) are held only by panels — skin is floppy, add a mounting tube or it will flap and tear.`);
    if (d.panels && d.panels.length && !GT.panelLoops(d).length) warns.push('Body panels don\'t form a closed outline — the wind tunnel will see open sheets, not a shell.');

    const sprung = new Set(); d.springs.forEach(s => { sprung.add(s.a); sprung.add(s.b); });
    if (d.wheels.some(w => !sprung.has(w.node))) warns.push('An axle is rigid-mounted (no coilover): only the tyre sidewall is suspension. Kart rules.');

    const nm = GT.nodeMapOf(d);
    let slender = 0, longs = 0;
    for (const b of d.beams) { const p = GT.beamProps(nm, b); if (!p) continue; if (p.buckling && p.Pcr < 20000) slender++; if (p.L > GT.MAX_BEAM) longs++; }
    if (slender) warns.push(`${slender} slender tube(s) will buckle below 20 kN compression. Use a bigger section or shorter members.`);
    if (longs) errors.push(`${longs} tube(s) longer than ${GT.MAX_BEAM} m — stock length exceeded.`);

    const mm = GT.massModel(d), ax = GT.axleLoads(d, mm);
    if (ax && (ax.front < 0.05 || ax.front > 0.95)) warns.push('Centre of gravity sits outside / at the edge of the wheelbase — it will tip.');
    return { errors, warns, dof };
  };

  // ---------- presets ----------
  function build(name, spec) {
    const d = GT.emptyDesign(name);
    spec.nodes.forEach(([x, y]) => d.nodes.push({ id: d.nextId++, x, y }));
    const id = i => d.nodes[i].id;
    spec.beams.forEach(([a, b, mat, tube]) => d.beams.push({ id: d.nextId++, a: id(a), b: id(b), mat: mat || spec.mat, tube: tube || spec.tube }));
    spec.springs.forEach(([a, b]) => d.springs.push({ id: d.nextId++, a: id(a), b: id(b), k: spec.k, c: spec.c, travel: spec.travel || 0.1, auto: true, adj: 0 }));
    spec.wheels.forEach(([i, driven, brake]) => d.wheels.push({ id: d.nextId++, node: id(i), size: spec.wheel, tire: spec.tire, brake: brake || spec.brake, driven }));
    d.engine = { node: id(spec.engine[0]), type: spec.engine[1] };
    d.driver = { node: id(spec.driver) };
    if (spec.tank) d.tank = { node: id(spec.tank[0]), size: spec.tank[1], type: spec.tank[2], fill: spec.tank[3] };
    Object.assign(d.setup, spec.setup || {});
    (spec.wings || []).forEach(([i, type]) => d.wings.push({ id: d.nextId++, node: id(i), type }));
    (spec.panels || []).forEach(([a, b]) => d.panels.push({ id: d.nextId++, a: id(a), b: id(b), mat: spec.panelMat || 'grp' }));
    if (spec.paint) d.setup.paint = spec.paint;
    return d;
  }

  const ATOM_NODES = [
    [-0.6, 0.25], [0.4, 0.15], [1.4, 0.15], [2.2, 0.15], [3.2, 0.30],   // 0-4 lower rail
    [-0.5, 0.75], [0.5, 0.90], [1.1, 1.15], [1.9, 0.85], [3.0, 0.65],   // 5-9 upper rail / roll hoop
    [0.0, 0.33], [2.5, 0.33],                                             // 10-11 hubs
  ];
  const ATOM_BEAMS = [
    [0, 1], [1, 2], [2, 3], [3, 4], [5, 6], [6, 7], [7, 8], [8, 9], [0, 5], [4, 9],
    [1, 6], [2, 7], [3, 8], [5, 1], [6, 2], [7, 3], [8, 4],
    [10, 1], [11, 3],   // trailing arms
  ];

  // outer loop of the frame: floor, nose, bonnet, roof, rear deck
  const ATOM_SHELL = [[0, 1], [1, 2], [2, 3], [3, 4], [4, 9], [9, 8], [8, 7], [7, 6], [6, 5], [5, 0]];

  GT.PRESETS = {
    starter: {
      name: 'Starter: I4T space-frame roadster',
      make: () => build('Starter roadster', {
        nodes: ATOM_NODES, beams: ATOM_BEAMS, springs: [[10, 6], [11, 8]],
        mat: 'mild', tube: '32x2.0', k: 60000, c: 3000, travel: 0.1,
        wheels: [[10, true, 'sport'], [11, false, 'sport']], wheel: 'r18', tire: 'sport', brake: 'sport',
        engine: [1, 'i4t'], driver: 2, panels: ATOM_SHELL, panelMat: 'grp', paint: '#d8452b',
        tank: [3, 'l40', 'steel', 0.6],
      }),
    },
    track: {
      name: 'Track: V8 chromoly with wing',
      make: () => build('V8 track car', {
        nodes: ATOM_NODES, beams: ATOM_BEAMS.concat([[0, 6], [9, 3]]), springs: [[10, 6], [11, 8]],
        mat: 'cromo', tube: '32x2.0', k: 90000, c: 4200, travel: 0.09,
        wheels: [[10, true, 'race'], [11, false, 'race']], wheel: 'r19', tire: 'semi', brake: 'race',
        engine: [1, 'v8'], driver: 2, wings: [[5, 'ta'], [4, 'splitter']], panels: ATOM_SHELL, panelMat: 'carbon', paint: '#f2c230',
        tank: [2, 'l60', 'cell', 0.5],
        setup: { gearbox: 'seq6', ecu: 'standalone', map: 'sport', tc: 4, abs: true, launch: true, launchPct: 50, flatShift: true, radiator: 'race' },
      }),
    },
    ladder: {
      name: 'Lesson: untriangulated ladder (folds!)',
      make: () => build('Ladder frame (bad)', {
        nodes: [[-0.5, 0.2], [0.6, 0.2], [1.8, 0.2], [3.0, 0.2], [-0.5, 0.8], [0.6, 0.8], [1.8, 0.8], [3.0, 0.8], [0, 0.33], [2.5, 0.33]],
        beams: [[0, 1], [1, 2], [2, 3], [4, 5], [5, 6], [6, 7], [0, 4], [1, 5], [2, 6], [3, 7], [8, 1], [9, 2]],
        springs: [[8, 5], [9, 6]],
        mat: 'mild', tube: '38x2.5', k: 60000, c: 3000,
        wheels: [[8, true], [9, false]], wheel: 'r18', tire: 'street', brake: 'street',
        engine: [1, 'i4na'], driver: 2, tank: [2, 'l40', 'steel', 0.5],
      }),
    },
  };
})(window.GT);
