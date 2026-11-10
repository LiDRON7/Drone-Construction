# Drone Construction

LiDRON's drone models and build requirements.

| I want to... | Go to |
|---|---|
| Print a part | `3D Models/Widow/Printed Parts/<part>/`, open the `.3mf` in Anycubic Slicer Next |
| See the whole drone | `3D Models/Widow/Full Drone/widow-full-drone.f3d`, open in Fusion |
| Know Widow's parts, sizes and weights | [`3D Models/Widow/README.md`](3D%20Models/Widow/README.md) |
| Know Huntsman's parts, sizes, weights and LiDAR wiring | [`3D Models/Huntsman/README.md`](3D%20Models/Huntsman/README.md) |
| Know what to build next | [Issues](https://github.com/LiDRON7/Drone-Construction/issues): one user story or task each |

<img src="3D%20Models/Widow/Full%20Drone/widow-full-drone-preview.png" width="600" alt="Widow, the full drone model">

## Printed parts (Widow)

| | Part | Print | Material |
|---|---|---|---|
| <img src="3D%20Models/Widow/Printed%20Parts/Propeller%20Guard/propeller-guard-preview.png" width="160"> | [Propeller Guard](3D%20Models/Widow/Printed%20Parts/Propeller%20Guard) | 6 | PLA or PETG |
| <img src="3D%20Models/Widow/Printed%20Parts/Landing%20Leg/landing-leg-preview.png" width="160"> | [Landing Leg](3D%20Models/Widow/Printed%20Parts/Landing%20Leg) (one mount on the battery plate for both legs, the battery and the camera, plus the feet) | 1 mount, 2 feet | PETG |
| <img src="3D%20Models/Widow/Printed%20Parts/GPS%20Mast/gps-mast-preview.png" width="160"> | [GPS Mast](3D%20Models/Widow/Printed%20Parts/GPS%20Mast) | 1 | PETG |

## Printed parts (Huntsman)

| | Part | Print | Material |
|---|---|---|---|
| <img src="3D%20Models/Huntsman/Printed%20Parts/Payload%20Carrier/payload-carrier-preview.png" width="160"> | [Payload Carrier](3D%20Models/Huntsman/Printed%20Parts/Payload%20Carrier) (hangs from the Tarot's payload rods: holds the Puck LITE, battery, camera and LiDAR interface box) | 1 carrier, 6 rod clamps | PETG |
| <img src="3D%20Models/Huntsman/Printed%20Parts/Propeller%20Guard/propeller-guard-preview.png" width="160"> | [Propeller Guard](3D%20Models/Huntsman/Printed%20Parts/Propeller%20Guard) (13" props; clamps to the arm tube) | 6 guards, 6 clamps | PETG |
| | GPS Mast: same as Widow's | 1 | PETG |

Each part's folder has the same five files:

- `.3mf`: print job for our Kobra 2 Max, settings already set
- `.gcode`: goes straight to the Kobra 2 Max
- `.stl`: print on another printer, or click it on GitHub to spin it in 3D
- `.step`: edit it in Fusion or any CAD program
- `-preview.png`: the picture above

## Upload a new or changed file

1. Open the folder the file belongs in.
2. Click **Add file**, then **Upload files**, and drag your files in.
3. Write one line about what changed, then click **Commit changes**.

Changing a part? Upload it with **the same name**. Don't add `v2` or `final`, and don't make an "old" folder.

Need an earlier version? Open the file, click **History**, pick the change, download.

## Where things are

```
3D Models/
├── Widow/
│   ├── Printed Parts/     parts we print (above)
│   ├── Reference Parts/   models of parts we buy, to design holders around
│   └── Full Drone/        everything put together
├── Huntsman/              same layout (Reference Parts has the Tarot 680 PRO frame model)
└── Fusion Scripts/        the scripts that build the models (huntsman_*.py for Huntsman)
```

Reference parts start with their kind, so they sort together: `frame-`, `propulsion-`, `power-`, `flight-controller-`,
`gps-`, `radio-`, `computer-`, `camera-`, `misc-`. Anything named `placeholder` is a rough block until we measure it.

## Changing a model's shape

Only needed to change sizes. Each model has a script in `Fusion Scripts` with the same name (`landing_leg.py`
builds the landing leg). With Fusion open and the
[Fusion360MCP add-in](https://github.com/faust-machines/fusion360-mcp-server) running:

```
cd "3D Models/Fusion Scripts"
python3 run_in_fusion.py execute_code -code landing_leg.py
```

Sizes are at the top of each script. The scripts check strength and fit, and stop with a message if something's off.
Then run `full_drone.py` so the whole drone picks up the change.

Stuck? Ask Alex or Devlin.
