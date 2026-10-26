// Powertrain: gearbox, ECU & tune, driver-aid electronics, fuel tank, cooling.
// GT.powertrain(design) resolves all of it into one spec the sim and the UI share.
(function (GT) {
  'use strict';
  const clamp = (v, a, b) => (v < a ? a : v > b ? b : v);
  const RPM = 60 / (2 * Math.PI);

  GT.FUEL_DENSITY = 0.745;   // kg/L petrol

  // shift: seconds of torque interruption. mass: delta against the engine's stock box.
  GT.GEARBOXES = {
    m5:   { name: '5-speed manual',             n: 5, shift: 0.30, mass: -5, cost: 0 },
    m6:   { name: '6-speed manual',             n: 6, shift: 0.25, mass: 0,  cost: 1500 },
    seq6: { name: '6-speed sequential dog-box', n: 6, shift: 0.08, mass: 4,  cost: 18000 },
    dct7: { name: '7-speed dual-clutch',        n: 7, shift: 0.05, mass: 14, cost: 9000 },
    at8:  { name: '8-speed torque-converter auto', n: 8, shift: 0.20, mass: 22, cost: 4000 },
  };

  GT.ECUS = {
    stock:      { name: 'Stock ECU (locked)',   cost: 0,    maps: ['eco', 'stock'],                  boost: 1.0,  rev: 0,   launch: false, tc: false, flat: false },
    piggyback:  { name: 'Piggyback tuner',      cost: 1500, maps: ['eco', 'stock', 'sport'],         boost: 1.15, rev: 0,   launch: true,  tc: false, flat: false },
    standalone: { name: 'Standalone race ECU',  cost: 6500, maps: ['eco', 'stock', 'sport', 'race'], boost: 1.4,  rev: 500, launch: true,  tc: true,  flat: true },
  };
  // tq: torque multiplier (turbo engines get 1.5× the delta), bsfc: fuel per kWh, heat: heat-to-coolant multiplier
  GT.MAPS = {
    eco:   { name: 'Economy',     tq: 0.85, bsfc: 0.93, heat: 0.90 },
    stock: { name: 'Stock',       tq: 1.00, bsfc: 1.00, heat: 1.00 },
    sport: { name: 'Sport',       tq: 1.06, bsfc: 1.03, heat: 1.07 },
    race:  { name: 'Race (100 RON)', tq: 1.12, bsfc: 1.08, heat: 1.16 },
  };
  GT.MIXES = {
    lean:   { name: 'Lean (economy, hot)',  tq: 0.97, bsfc: 0.94, heat: 1.10 },
    stoich: { name: 'Stoichiometric',       tq: 1.00, bsfc: 1.00, heat: 1.00 },
    rich:   { name: 'Rich (power, cool)',   tq: 1.02, bsfc: 1.10, heat: 0.92 },
  };
  GT.TC_LEVELS = [null, 0.30, 0.24, 0.19, 0.15, 0.12, 0.10, 0.08, 0.065, 0.05, 0.04];   // target slip ratio per level
  GT.ABS = { mass: 3, cost: 2200 };

  GT.TANKS = {
    l20: { name: '20 L', L: 20 }, l40: { name: '40 L', L: 40 }, l60: { name: '60 L', L: 60 }, l80: { name: '80 L', L: 80 }, l100: { name: '100 L', L: 100 },
  };
  GT.TANK_TYPES = {
    steel: { name: 'Pressed-steel tank',        kgPerL: 0.24, cost: 300,  costPerL: 4,  starveL: 6,   rupture: true },
    cell:  { name: 'FIA bag cell + foam + swirl pot', kgPerL: 0.16, cost: 2500, costPerL: 40, starveL: 1.2, rupture: false },
  };
  GT.RADIATORS = {
    small: { name: 'Small (light, low drag)', factor: 0.7, mass: 6,  CdA: 0.02, cost: 200 },
    std:   { name: 'Standard',                factor: 1.0, mass: 9,  CdA: 0.035, cost: 400 },
    race:  { name: 'Race core + oil cooler',  factor: 1.45, mass: 13, CdA: 0.06, cost: 1800 },
  };

  GT.SETUP_DEFAULTS = { gearbox: 'm6', ecu: 'stock', map: 'stock', boost: 1.0, revAdj: 0, launchPct: 45, shiftPct: 96, mix: 'stoich', tc: 0, abs: false, launch: false, flatShift: false, radiator: 'std' };

  GT.gearRatios = function (spec, gb) {
    if (spec.ev) return spec.gears.slice();
    const a = spec.gears[0], b = spec.gears[spec.gears.length - 1], n = gb.n, out = [];
    for (let i = 0; i < n; i++) out.push(a * Math.pow(b / a, i / (n - 1)));
    return out;
  };

  // Everything the ECU and hardware decide, clamped to what the hardware allows.
  GT.powertrain = function (d) {
    const st = Object.assign({}, GT.SETUP_DEFAULTS, d.setup);
    const spec = d.engine ? (GT.ENGINES[d.engine.type] || GT.ENGINES.i4t) : null;
    if (!spec) return null;
    const gb = GT.GEARBOXES[st.gearbox] || GT.GEARBOXES.m6;
    const ecu = GT.ECUS[st.ecu] || GT.ECUS.stock;
    const mapId = ecu.maps.includes(st.map) ? st.map : 'stock';
    const map = GT.MAPS[mapId], mix = ecu === GT.ECUS.stock ? GT.MIXES.stoich : (GT.MIXES[st.mix] || GT.MIXES.stoich);
    const boost = spec.turbo ? clamp(st.boost, 0.8, ecu.boost) : 1;
    const revAdj = spec.ev ? 0 : clamp(st.revAdj, -1000, ecu.rev);
    const tcLevel = ecu.tc ? clamp(Math.round(st.tc) || 0, 0, 10) : 0;
    const turboK = spec.turbo ? 1.5 : 1;
    const pk = GT.peakPower(spec);
    return {
      spec, gb, ecu, mapId, map, mix,
      ratios: GT.gearRatios(spec, gb),
      final: d.setup.finalDrive || spec.final,
      shift: spec.ev ? 0 : gb.shift * (ecu.flat && st.flatShift ? 0.4 : 1),
      tqMul: (1 + (map.tq - 1) * turboK) * mix.tq,
      boost, revAdj,
      redline: spec.redline + revAdj, stockRedline: spec.redline,
      launchRpm: spec.ev ? 0 : spec.idle + (spec.redline - spec.idle) * (ecu.launch ? clamp(st.launchPct, 20, 85) : 45) / 100,
      shiftFrac: clamp(st.shiftPct, 70, 100) / 100,
      launchCtl: ecu.launch && !!st.launch,
      tc: tcLevel ? GT.TC_LEVELS[tcLevel] : 0, tcLevel,
      abs: !!st.abs,
      bsfc: (spec.bsfc || 260) * map.bsfc * mix.bsfc * (1 + (boost - 1) * 0.3),
      heatMul: (spec.heat || 1) * map.heat * mix.heat * (1 + (boost - 1) * 0.8),
      radiator: GT.RADIATORS[st.radiator] || GT.RADIATORS.std,
      peakKw: pk.kw,
      knockRisk: mapId === 'race' && mix === GT.MIXES.lean,
    };
  };

  // Crank torque at rpm with the tune applied (boost needs the turbo spooled).
  GT.ptTorque = function (pt, rpm) {
    let t = GT.curveAt(pt.spec.curve, rpm) * pt.tqMul;
    if (pt.boost !== 1) t *= 1 + (pt.boost - 1) * clamp((rpm - 1500) / 1500, 0, 1);
    return t;
  };

  GT.dyno = function (pt) {
    const out = [];
    for (let r = Math.max(800, pt.spec.idle); r <= pt.redline; r += 100) {
      const tq = GT.ptTorque(pt, r), st = GT.curveAt(pt.spec.curve, r);
      out.push({ rpm: r, tq, kw: tq * r / RPM / 1000, stockTq: r <= pt.stockRedline ? st : null, stockKw: r <= pt.stockRedline ? st * r / RPM / 1000 : null });
    }
    return out;
  };

  // Mass/cost of everything powertrain beyond the bare engine.
  GT.powertrainMass = function (d) {
    const pt = GT.powertrain(d), out = { gearbox: 0, electronics: 0, cooling: 0, tankDry: 0, fuel: 0, cost: 0 };
    if (pt) {
      if (!pt.spec.ev) { out.gearbox = pt.gb.mass; out.cost += pt.gb.cost; }
      out.cooling = pt.radiator.mass; out.cost += pt.radiator.cost + pt.ecu.cost;
      out.electronics = 1 + (pt.abs ? GT.ABS.mass : 0); if (pt.abs) out.cost += GT.ABS.cost;
    }
    if (d.tank) {
      const tk = GT.TANKS[d.tank.size] || GT.TANKS.l40, ty = GT.TANK_TYPES[d.tank.type] || GT.TANK_TYPES.steel;
      out.tankDry = tk.L * ty.kgPerL; out.fuel = tk.L * clamp(d.tank.fill, 0, 1) * GT.FUEL_DENSITY;
      out.cost += ty.cost + ty.costPerL * tk.L;
    }
    return out;
  };
})(window.GT);
