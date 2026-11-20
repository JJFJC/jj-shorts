# KESTREL: original quad tilt-rotor VTOL

A ~1 m, ~1.2 kg 3D-printed plane that **takes off and hovers like a drone**, then **tilts all four rotors forward and flies like a plane**. It's an original design inspired by the look of FLIGHTORY's "Super Stingray" (a blended-body flying wing), but with **two wings**.

| | |
|---|---|
| Layout | Blended wing-body main wing + **front canard** (second wing), winglets |
| Rotors | **4 tilting:** 2 pullers on the canard tips, 2 **pushers behind the main-wing trailing edge** |
| Size / weight | 1000 mm span, ~740 mm long, ~1.18 kg (4S 2200 mAh) |
| Hover | 4× 2207 1700 KV, 7" props, thrust/weight 4.2, load split 54/46 front/rear |
| Speeds (estimated) | stall 29 km/h · efficient cruise 45 km/h · 72 km/h cruise · top ≈ 120 km/h |
| Flight time (est.) | ~24 min at 45 km/h, ~8 min at 72 km/h (4S 2200; a 3300 mAh pack adds ~45%) |
| Flight control | ArduPilot QuadPlane, tilt-rotor mode (hover, transition, cruise) |
| Printing | 24 printed pieces, all fit the Bambu P1S bed; LW-PLA for the airframe, PETG for the tilt pods |

Full numbers and assumptions: [`docs/SIZING.md`](docs/SIZING.md). These are textbook first estimates, so treat them as ±20% until flight tests.

## How it aims to beat the reference

I couldn't watch the video, only its thumbnail and title. So these are design intentions, not a measured comparison:
- **All four rotors tilt.** Every motor pushes in cruise, which gives more top speed, and none sits as dead weight in fast flight.
- **The canard** gives strong pitch control during the transition (the hardest phase), and the plane is less prone to stalling.
- **Trailing-edge pushers** put the rear props in the wing's wake, which is efficient, and even out the hover load.
- **Fully parametric:** change `params.py` and the CAD, sizing and renders all update.

## Files

| Path | What |
|---|---|
| `params.py` | every dimension, mass and component |
| `sim/sizing.py` → `docs/SIZING.md` | geometry, CG/neutral point, hover/cruise power, speeds, flight time |
| `cad/kestrel.py` → `cad/out/{hover,cruise}/` | CadQuery airframe; STLs in both rotor positions |
| `render/render.py` → `render/out/` | Blender renders: hover 3/4, cruise 3/4, top view |

## Not done yet (next steps)

1. **Internal structure:** spar channels (8 mm main, 6 mm canard), battery bay and hatch, servo pockets, elevons and canard elevators, tilt-servo linkages. Printing is vase-mode / thin-wall LW-PLA, so the CAD solids become skins with ribs.
2. **ArduPilot SITL (simulator)** model and parameters: tilt-rotor setup, transition speed, motor layout.
3. **Parts list (BOM)** in CAD $ with Canadian sources.
4. **Canada:** at 1.2 kg it must be **registered with Transport Canada** and flown with at least a **Basic pilot certificate** (online exam). Fly in open areas away from people, airports and parks with bylaws.
