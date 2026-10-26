# Chassis Lab GT

A hardcore 2D sports-car chassis construction simulator. Weld a tube space frame, hang suspension, wheels, engine and aero on it, then break it on the proving ground.

## Run

No build step. Serve the folder and open it in a browser:

```bash
cd ~/Documents/SportsCarGT && python3 -m http.server 8737
```

Then open http://localhost:8737. Opening `index.html` directly from disk also works in most browsers.

## What's simulated

- **Frame**: pin-jointed tubes (each one is a left/right pair) solved with XPBD. The constraint force in each tube is checked against tensile yield, compressive yield and Euler buckling (effective length factor 0.8). Ductile metals yield plastically and take a permanent set, then fracture at about 3.5 % strain. Carbon snaps.
- **Materials**: mild steel, chromoly, welded 6061 aluminium (yield derated for the heat-affected zone), Ti-6Al-4V, and pultruded CFRP with weaker compression. Seven round tube sections.
- **Structure check**: Maxwell's rule (m ≥ 2j − 3) flags mechanisms. An untriangulated frame really does fold.
- **Suspension**: coilovers with spring rate, damping, bump travel, polyurethane bump stops, droop limit and automatic preload.
- **Tyres**: compliant vertical rate with carcass damping, rim strikes, Coulomb friction with a static/kinetic drop, and rolling resistance.
- **Drivetrain**: torque curves, launch clutch, rev limiter, engine braking or regen, an auto gearbox that shifts on road speed, and flywheel inertia reflected through the gearing.
- **Engines & drivetrain**: 13 engines, including a turbo I3, a Wankel rotary, a flat-6, a pushrod V8, a twin-turbo V8, a V10, a V12, a 1.6 turbo-hybrid with an ERS energy store, and an EV. Five gearboxes (5/6-speed manual, sequential dog-box, 7-speed DCT, 8-speed auto) set the gear count, ratio spread, shift time and mass.
- **ECU & tune**: the stock ECU is locked; a piggyback tuner and a standalone race ECU unlock more. Options are map (eco/stock/sport/race), mixture, turbo boost, rev limit, upshift point and launch rpm, shown on a tuned-vs-stock dyno chart. Tunes change fuel use and heat. Over-revving wears the valvetrain, and a race map with a lean mixture running hot detonates.
- **Electronics**: traction control (10 levels, needs the race ECU), ABS, launch control and flat-shift.
- **Fuel tank**: you place it on the frame like any part. Fuel weight burns off as you drive, based on fuel efficiency (BSFC). A low tank starves the engine under hard g, and a steel tank that scrapes the ground ruptures. An FIA bag cell avoids both but costs more.
- **Cooling**: a coolant temperature model with heat from power output, airflow-dependent radiator rejection and a thermostat. Above 115 °C power derates; at 130 °C the engine seizes. Radiator size trades cooling against mass and drag.
- **Bodywork**: draw body panels node-to-node in fibreglass, aluminium or carbon. Or use **Wrap frame in panels**, which builds a convex hull to start from. A closed outline becomes a filled shell. Panels are soft skin: they add mass, carry aero load into their two nodes, and tear off if overloaded.
- **Wind tunnel**: a 2D lattice-Boltzmann CFD run (D2Q9, BGK + Smagorinsky LES) of the car's side section at 200 km/h, with a rolling road and a slip ceiling.
  - You can view air speed, pressure (Cp) or vorticity, with smoke streaklines on top.
  - Momentum exchange gives the force on every panel. Wings and splitters use catalogue coefficients scaled by the dynamic pressure the tunnel measures around them, so a wing in the roof's wake loses its downforce.
  - 2D section forces are scaled to a 1.8 m wide car with calibrated 3D relief factors.
  - Results are saved into the design and drive the proving-ground physics until the geometry changes. Until you run the tunnel, a rough estimate is used instead.
- **Aero on the road**: each part pushes on its own nodes, so downforce loads the frame and pitches the car. The proving ground can draw force arrows for this.
- **Tests**: drag strip, 100–0 braking, rough road, a 1.1 m jump, a hill climb up to 80 %, a 5 km endurance stint (heat, fuel, brakes), and free drive. Fast-forward 4× and slow-mo are available.

## Files

- `js/data.js`: part catalogue and constants
- `js/powertrain.js`: gearboxes, ECU and tune, electronics, tanks, cooling
- `js/model.js`: section properties, mass model, validation, presets
- `js/aero.js`: body panels, aero estimate, LBM wind tunnel
- `js/sim.js`: physics
- `js/tests.js`: terrain and the proving-ground tests
- `js/render.js`: canvas drawing
- `js/main.js`: editor and UI
