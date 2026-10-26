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
- **Aero**: body CdA/ClA plus wings and splitters that load the node they're bolted to.
- **Tests**: drag strip, 100–0 braking, rough road, a 1.1 m jump, a hill climb up to 80 %, and free drive.

## Files

- `js/data.js`: part catalogue and constants
- `js/model.js`: section properties, mass model, validation, presets
- `js/sim.js`: physics
- `js/tests.js`: terrain and the proving-ground tests
- `js/render.js`: canvas drawing
- `js/main.js`: editor and UI
