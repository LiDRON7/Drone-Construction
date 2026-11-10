# Drone Construction

LiDRON's drone models and the parts we 3D print for them. Two drones: Widow (test drone) and Huntsman (carries the LiDAR).

## Widow

Storm Drone 6 V3. Parts, sizes and weights: [Widow README](3D%20Models/Widow/README.md).

<img src="3D%20Models/Widow/Full%20Drone/widow-full-drone-preview.png" width="420" alt="Widow, the whole drone"> <img src="3D%20Models/Widow/widow-top-view.png" width="300" alt="Widow from above, arms numbered">

| Picture | Part | Print | What it does |
|---|---|---|---|
| <img src="3D%20Models/Widow/Printed%20Parts/Landing%20Leg/landing-leg-preview.png" width="220"> | [Landing Leg](3D%20Models/Widow/Printed%20Parts/Landing%20Leg) | 1 mount, 2 feet | Holds both legs, the battery and the camera |
| <img src="3D%20Models/Widow/Printed%20Parts/GPS%20Mast/gps-mast-preview.png" width="220"> | [GPS Mast](3D%20Models/Widow/Printed%20Parts/GPS%20Mast) | 1 | Lifts the GPS above the electronics |
| <img src="3D%20Models/Widow/Printed%20Parts/Propeller%20Guard/propeller-guard-preview.png" width="220"> | [Propeller Guard](3D%20Models/Widow/Printed%20Parts/Propeller%20Guard) | 6 | One per prop, held by the motor screws |

## Huntsman

Tarot 680 PRO with the Velodyne Puck LITE. Parts, sizes, weights and LiDAR wiring: [Huntsman README](3D%20Models/Huntsman/README.md).

<img src="3D%20Models/Huntsman/Full%20Drone/huntsman-full-drone-preview.png" width="600" alt="Huntsman, the whole drone">

<img src="3D%20Models/Huntsman/Full%20Drone/huntsman-payload-closeup.png" width="360" alt="Under Huntsman: LiDAR, battery and camera on the payload carrier"> <img src="3D%20Models/Huntsman/Full%20Drone/huntsman-guard-closeup.png" width="360" alt="Huntsman propeller guards on the arms">

| Picture | Part | Print | What it does |
|---|---|---|---|
| <img src="3D%20Models/Huntsman/Printed%20Parts/Payload%20Carrier/payload-carrier-preview.png" width="220"> | [Payload Carrier](3D%20Models/Huntsman/Printed%20Parts/Payload%20Carrier) | 1 carrier, 6 rod clamps | Hangs under the frame and holds the LiDAR, battery, camera and LiDAR interface box |
| <img src="3D%20Models/Huntsman/Printed%20Parts/Propeller%20Guard/propeller-guard-preview.png" width="220"> | [Propeller Guard](3D%20Models/Huntsman/Printed%20Parts/Propeller%20Guard) | 6 guards, 6 clamps | One per 13" prop, clamped to the arm tube |
| | GPS Mast | 1 | Same print as Widow's |

Print everything in PETG. Use PLA only for test prints.

## Print a part

Open the part's folder and pick a file:

- `.gcode` goes straight to our Kobra 2 Max
- `.3mf` opens in Anycubic Slicer Next with the settings already set
- `.stl` is for any other printer (click it on GitHub to spin it in 3D)
- `.step` opens in Fusion or any CAD program

## Upload a new or changed file

1. Open the folder the file belongs in.
2. Click Add file, then Upload files, and drag your files in.
3. Write one line about what changed, then click Commit changes.

Keep the same file name when you change a part. No `v2`, no `final`, no "old" folders. GitHub keeps every earlier version: open the file, click History, pick the change.

## Where things are

```
3D Models/
├── Widow/
│   ├── Printed Parts/     parts we print
│   ├── Reference Parts/   models of parts we buy, to design holders around
│   └── Full Drone/        everything put together
├── Huntsman/              same folders (Reference Parts has the Tarot 680 PRO frame)
└── Fusion Scripts/        the scripts that build the models
```

Reference parts start with their kind so they sort together: `frame-`, `propulsion-`, `power-`, `flight-controller-`,
`gps-`, `radio-`, `computer-`, `camera-`, `misc-`. Anything named `placeholder` is a rough block until we measure it.

## Change a part's size

Each part has a script in `Fusion Scripts` (`landing_leg.py` builds Widow's landing leg, `huntsman_*.py` build
Huntsman's parts). Sizes are at the top of each script. With Fusion open and the
[Fusion360MCP add-in](https://github.com/faust-machines/fusion360-mcp-server) running:

```
cd "3D Models/Fusion Scripts"
python3 run_in_fusion.py execute_code -code landing_leg.py
```

The scripts check strength and fit and stop with a message if something is off. Then run `full_drone.py` (or
`huntsman_full_drone.py`) so the whole drone picks up the change.

Stuck? Ask Alex or Devlin.
