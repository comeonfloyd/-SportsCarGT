// Chassis Lab GT — part catalogue and physical constants.
// Everything is SI: metres, kilograms, newtons, seconds.
// The world is a 2D side view: every tube, coilover and wheel stands for a mirrored left/right PAIR.
window.GT = window.GT || {};
(function (GT) {
  'use strict';

  GT.G = 9.81;
  GT.RHO = 1.225;            // air density
  GT.JOINT_MASS = 1.0;       // kg per node: gussets + weld for the L/R pair
  GT.JOINT_COST = 60;        // $ of fabrication per joint
  GT.SPRING = { mass: 7, cost: 1400 };     // pair of coilover dampers
  GT.DRIVER_MASS = 80;
  GT.SYSTEMS = { mass: 110, cost: 6000 };  // fuel, cooling, wiring, seat, steering, pedals
  GT.TIRE_K = 5.2e5;         // vertical tyre rate, axle pair (N/m)
  GT.TIRE_C = 2500;          // tyre carcass damping (N·s/m)
  GT.BUMPSTOP_K = 3e6;       // polyurethane bump stop rate (N/m)
  GT.K_EFF = 0.8;            // buckling effective-length factor (welded ends ≈ partially fixed)
  GT.MAX_BEAM = 4.0;         // longest tube you can cut
  GT.SNAP = 0.05;

  GT.MATERIALS = {
    mild:   { name: 'Mild steel S235',            short: 'Steel', E: 200e9, yield: 250e6, rho: 7850, costKg: 3,   brittle: false, color: '#8d99a6' },
    cromo:  { name: 'Chromoly 4130 (normalised)', short: 'CrMo',  E: 205e9, yield: 460e6, rho: 7850, costKg: 14,  brittle: false, color: '#4f8fd6' },
    alu:    { name: 'Aluminium 6061 (welded, HAZ)', short: 'Alu', E: 69e9,  yield: 150e6, rho: 2700, costKg: 11,  brittle: false, color: '#d7dde4' },
    ti:     { name: 'Titanium Ti-6Al-4V',         short: 'Ti',    E: 114e9, yield: 830e6, rho: 4430, costKg: 95,  brittle: false, color: '#c9a36b' },
    carbon: { name: 'Carbon fibre (pultruded)',   short: 'CFRP',  E: 135e9, yield: 1300e6, compYield: 650e6, rho: 1600, costKg: 160, brittle: true, color: '#b07cff' },
  };

  // Round tube sections: outer diameter D, wall t
  GT.TUBES = [
    { id: '19x1.2', D: 0.019, t: 0.0012 },
    { id: '25x1.5', D: 0.025, t: 0.0015 },
    { id: '32x2.0', D: 0.032, t: 0.0020 },
    { id: '38x2.5', D: 0.038, t: 0.0025 },
    { id: '45x2.5', D: 0.045, t: 0.0025 },
    { id: '50x3.0', D: 0.050, t: 0.0030 },
    { id: '60x3.0', D: 0.060, t: 0.0030 },
  ];

  // Torque curves: [rpm, N·m]. Mass includes a stock gearbox / battery pack.
  // gears: first & top ratios (the gearbox picks how many in between). bsfc: g/kWh at load.
  const G6 = [3.36, 2.18, 1.62, 1.27, 1.03, 0.85], G6R = [3.10, 2.05, 1.55, 1.24, 1.02, 0.86];
  GT.ENGINES = {
    i3t:  { name: '1.0 I3 Turbo (eco)',          tag: 'I3T', mass: 95,  cost: 6000,  idle: 900,  redline: 6500, turbo: true, bsfc: 255,
      curve: [[0, 90], [1000, 110], [2000, 200], [4500, 200], [6500, 150]], gears: G6, final: 4.2 },
    i4na: { name: '2.0 I4 NA (high-revving)',    tag: 'I4',  mass: 135, cost: 9000,  idle: 900,  redline: 8000, bsfc: 250,
      curve: [[0, 120], [1000, 150], [3000, 185], [4500, 210], [6500, 205], [8000, 170]], gears: G6, final: 4.3 },
    i4t:  { name: '2.0 I4 Turbo',                tag: 'I4T', mass: 150, cost: 14000, idle: 900,  redline: 7000, turbo: true, bsfc: 265,
      curve: [[0, 150], [1000, 180], [2500, 400], [5500, 400], [7000, 300]], gears: G6, final: 3.9 },
    r2:   { name: '1.3 twin-rotor Wankel',       tag: 'ROT', mass: 115, cost: 16000, idle: 1000, redline: 9000, bsfc: 330, heat: 1.25,
      curve: [[0, 100], [1000, 110], [4000, 170], [7500, 190], [9000, 170]], gears: G6, final: 4.4 },
    f6:   { name: '4.0 flat-6 NA',               tag: 'F6',  mass: 175, cost: 38000, idle: 900,  redline: 9000, bsfc: 250,
      curve: [[0, 250], [1000, 280], [4000, 400], [6500, 470], [8000, 460], [9000, 420]], gears: G6R, final: 3.9 },
    v6tt: { name: '3.8 V6 Twin-turbo',           tag: 'V6TT', mass: 190, cost: 32000, idle: 850, redline: 7100, turbo: true, bsfc: 260,
      curve: [[0, 200], [1000, 250], [3000, 630], [5800, 630], [7100, 520]], gears: [3.20, 0.84], final: 3.7 },
    v8:   { name: '5.2 V8 flat-plane NA',        tag: 'V8',  mass: 205, cost: 42000, idle: 950,  redline: 8800, bsfc: 255,
      curve: [[0, 250], [1000, 300], [3500, 480], [6200, 560], [8000, 520], [8800, 470]], gears: G6R, final: 3.9 },
    v8pr: { name: '7.0 V8 pushrod NA (torque)',  tag: 'V8PR', mass: 190, cost: 22000, idle: 700, redline: 7000, bsfc: 270,
      curve: [[0, 450], [1000, 500], [4800, 640], [6300, 600], [7000, 540]], gears: [2.97, 0.67], final: 3.4 },
    v8tt: { name: '4.0 V8 Twin-turbo',           tag: 'V8TT', mass: 210, cost: 55000, idle: 800, redline: 7500, turbo: true, bsfc: 262,
      curve: [[0, 300], [1000, 350], [2500, 800], [6000, 800], [7500, 650]], gears: G6R, final: 3.5 },
    v10:  { name: '5.2 V10 NA',                  tag: 'V10', mass: 215, cost: 70000, idle: 1000, redline: 8700, bsfc: 258,
      curve: [[0, 320], [1000, 360], [4500, 520], [6500, 560], [8000, 530], [8700, 500]], gears: G6R, final: 3.8 },
    v12:  { name: '6.5 V12 NA',                  tag: 'V12', mass: 245, cost: 90000, idle: 1000, redline: 9200, bsfc: 265,
      curve: [[0, 300], [1000, 350], [4000, 600], [7000, 720], [8500, 690], [9200, 640]], gears: G6R, final: 3.6 },
    hyb:  { name: '1.6 V6 turbo-hybrid (ERS)',   tag: 'HYB', mass: 175, cost: 150000, idle: 4000, redline: 12000, turbo: true, bsfc: 225,
      curve: [[0, 200], [3000, 260], [7000, 330], [10500, 380], [12000, 350]], gears: [3.5, 1.05], final: 3.4,
      ers: { kw: 120, store: 4e6 } },
    ev:   { name: 'Dual e-motor + 60 kWh pack',  tag: 'EV',  mass: 470, cost: 38000, idle: 0, redline: 17000, ev: true, battery: 60,
      curve: [[0, 620], [6500, 620], [10000, 420], [17000, 240]], gears: [9.1], final: 1.0 },
  };

  // Per corner. Each axle entry = 2 corners.
  GT.WHEELS = {
    r15: { name: '15" street (r 290)',  r: 0.290, mass: 16, cost: 350 },
    r17: { name: '17" sport (r 315)',   r: 0.315, mass: 19, cost: 600 },
    r18: { name: '18" sport (r 330)',   r: 0.330, mass: 21, cost: 800 },
    r19: { name: '19" forged (r 345)',  r: 0.345, mass: 20, cost: 2200 },
    r20: { name: '20" show rim (r 360)', r: 0.360, mass: 26, cost: 1400 },
    off: { name: '33" off-road (r 420)', r: 0.420, mass: 34, cost: 900 },
  };
  GT.TIRES = {
    eco:    { name: 'Eco touring',      mu: 0.85, cost: 90 },
    street: { name: 'Street all-season', mu: 1.00, cost: 150 },
    sport:  { name: 'Sport UHP',        mu: 1.15, cost: 260 },
    semi:   { name: 'Semi-slick R-comp', mu: 1.35, cost: 380 },
    slick:  { name: 'Racing slick',     mu: 1.60, cost: 520 },
  };
  // torque = brake torque for the whole axle (both corners)
  GT.BRAKES = {
    street: { name: 'Street 2-pot',   torque: 1600, mass: 4,   cost: 500 },
    sport:  { name: 'Sport 4-pot',    torque: 2600, mass: 5,   cost: 1600 },
    race:   { name: 'Race 6-pot',     torque: 3800, mass: 5.5, cost: 4200 },
    ccm:    { name: 'Carbon-ceramic', torque: 4600, mass: 2.5, cost: 12000 },
  };
  GT.WINGS = {
    lip:      { name: 'Lip spoiler',      kind: 'wing',     ClA: 0.25, CdA: 0.03, mass: 3,  cost: 300 },
    splitter: { name: 'Front splitter',   kind: 'splitter', ClA: 0.45, CdA: 0.04, mass: 6,  cost: 900 },
    gt:       { name: 'GT wing',          kind: 'wing',     ClA: 1.00, CdA: 0.16, mass: 9,  cost: 2500 },
    ta:       { name: 'Time-attack wing', kind: 'wing',     ClA: 1.80, CdA: 0.32, mass: 13, cost: 5500 },
  };
})(window.GT);
