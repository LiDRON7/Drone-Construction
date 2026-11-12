# Huntsman: components, sizes and weights

Huntsman is the Tarot 680 PRO hexacopter that carries the LiDAR, flown as a **hexa X (PX4 airframe 6001)**. The
front is the gap between two arms that the two payload rods under the frame point at. The camera looks out of it,
and the Pixhawk and GPS arrows point the same way.

The LiDAR is the **Velodyne Puck LITE** (590 g). Some team docs and issues call it the VLP-16; the Puck LITE is the
lighter version of the VLP-16, same size, same data and driver.

Sizes are mm, weights g. The frame numbers come from an online Tarot 680 PRO model
(`Reference Parts/frame-tarot-680-pro.step`), not calipers yet, so they are EST until measured. The full-drone model
(`Fusion Scripts/huntsman_full_drone.py`) uses these numbers.

## Bought parts

| Part | Where it sits | Size | Weight |
|---|---|---|---|
| Velodyne Puck LITE | payload carrier, front, on its side (top facing right), cable up and forward | 103.3 across x 71.7 (datasheet) | 590 (datasheet) |
| Velodyne interface box | under the interface-box plate (behind the carrier), velcro | 75 x 53 x 28 (EST) | 80 (EST) |
| 12 V step-down for the LiDAR (6S in, 3 A out) | top of the interface-box plate, between the rods, velcro | 45 x 26 x 14 (EST) | 25 (EST) |
| PowerHobby 6S 5200 mAh 50C LiPo | carrier, across the drone behind the LiDAR, 2 straps | 133 x 43.5 x 49.5 (listing) | 658 (listing) |
| Luxonis OAK-D Pro camera | cradle on the carrier's front, 10° down | **96.96** x **29.45** x **23.15** | **89.69** |
| Pixhawk 6X Mini | top plate, centre, on its foam, arrow forward | 72.8 x 43.4 x 31 (spec) | **54.49** |
| Raspberry Pi 5 in its case, with cooler | top plate, right | 85 x 56 x 20 (EST) | **99.92** |
| Holybro H-RTK F9P Helical | GPS mast (same print as Widow's), top plate, back left | module **53.86** long | **57.25** |
| SiK 915 MHz telemetry radio | top plate, front left | 53 x 28 x 10.7 (EST) | 15 (EST) |
| RadioLink R9DS receiver | top plate, front right | 43 x 24 x 15 (spec) | **12.53** |
| Holybro PM02D | top plate, back | 20 x 16 x 10 (EST) | 20 (EST) |
| 5 V step-down for the Pi | top plate, back right | 20 x 16 x 8 (EST) | 15 (EST) |
| Tarot 4108 380 KV motor (x6), Aikon AK8 80A ESC (x6), 1355 props | arms | | 90 each; ESC 50 each (EST); prop 30 each (EST) |

## Frame (Tarot 680 PRO, from the online model)

| What | Size |
|---|---|
| Top and bottom plates | 195 across; the bottom plate is also the power board |
| Motors | about 690 apart across |
| Payload rods | 2 x 10 mm carbon, 60 apart, 280 long, running fore-aft, on 4 dampers; rod centres 35 below the top plate's underside |
| Skids | 245 below the top plate's underside |

## Printed parts (weights as sliced for the Bambu Lab X2D)

The payload carrier, its rod clamps and the LiDAR strap are **PAHT-CF**: they touch the LiDAR (8 W) and sit next to
the 6S battery in the sun, and PETG-CF goes soft at about 70 °C and slowly lets screws loosen. PAHT-CF takes far more
heat and holds screws better. Dry it before printing (it soaks up water fast here). Propeller guards are **PETG**
(they have to bend, not snap). The GPS mast stays PETG-CF.

| Part | Material | Weight | Holds | Fixed by |
|---|---|---|---|---|
| Payload carrier | PAHT-CF | 161 | LiDAR, battery, camera | 4 rod clamps |
| Interface-box plate | PAHT-CF | 15 | interface box, 12 V step-down | 2 rod clamps (the back pair) |
| LiDAR strap | PAHT-CF | 7.4 | the LiDAR's top cap | 2 x M2.5 x 10 up into 2 inserts in the carrier's saddle |
| Rod clamp (x6) | PAHT-CF | 6.3 each | the carrier and the interface-box plate, on the payload rods | 2 x M2.5 x 6 up through the carrier into 2 inserts; 1 x M2.5 x 16 pinch screw (below) |
| Propeller guard (x6, 2 halves each) | PETG | 58.2 each | one 13" prop each | its hub sits under the motor: the motor's 3 screws (3 mm longer than stock) hold both halves; halves joined at the ring by 1 x M2.5 x 12 into 1 insert in the right half |
| GPS mast | PETG-CF | 38.1 | GPS module | Widow's mast; 4 x M2.5 x 6 from under the top plate (drill a 17 mm square of 4 holes if the Tarot plate has none there) |

Carrier, interface-box plate, LiDAR strap and 6 rod clamps print together: 221 g, 9 h 38 min
(`Printed Parts/Payload Carrier/payload-carrier.gcode.3mf`). Each guard half fills a plate: 29 g, 1 h 28 min
(`propeller-guard-left.gcode.3mf`, `propeller-guard-right.gcode.3mf`, print each 6 times).

**Rod clamps.** Each one pinches its rod instead of pressing a grub screw into it (a grub screw crushes carbon tube).
The clamp's inner wall is slit at the rod's height. An M2.5 x 16 goes up through the hole in the carrier next to the
clamp, sits in a pocket in the clamp, crosses the slit and screws into an insert pressed into the clamp's top, so you
can tighten it with everything fitted. Its tip ends flush with the clamp top (2 mm under the frame).

**Safety cord.** The whole payload (about 1.7 kg) hangs from the frame's 4 rubber dampers. If a damper ball pops out,
everything falls. Loop a cord (Dyneema or Kevlar, about 1 m, 50 kg or more) through the 2 holes just behind the LiDAR
and tie it round the frame's centre, slack enough that the dampers still work.

**Propeller guards.** They go under the motor, like Widow's: a 3 mm hub sits between the motor and the Tarot motor
plate and the motor's own 3 screws hold it, so it can't turn on the arm (the old guard clamped the round arm tube, and
only friction held it there, 12 mm under the prop). There is no arm clamp any more. The ring wraps the outer 190° of
each prop with an 8 mm tip gap: a solid band at the prop tips with a 5 mm lip round its top edge, and a diagonal
lattice of 45° bars below. Four T-section struts run from the hub to the ring, 17 mm under the prop's lowest point,
with a fin outside the ring at each strut end. The struts stay 3 mm thin under the motor bell (it sits 2 mm above the
plate) and widen to 18 mm where they meet the hub. A whole guard is 349 mm across, too big for the X2D, so it prints
as a left and a right half (looking out along the arm); the motor screws clamp both hubs and a block under the ring at
the outer point joins them with one cross bolt. Print with 3 walls and 15 % infill (the profile in the `.gcode.3mf`).

Before printing the guards, check two things on the drone (both from the online model, so EST):
- Where the motor's 3 screws sit. The Tarot 4108 uses 3 x M3 on a 27 mm triangle; the hub's slots take that pattern
  whether 27 mm is the circle or the side, with one screw straight across the arm (`HOLE_ANG0 = 90`). If yours sits
  another way, set `HOLE_ANG0` in `huntsman_propeller_guard.py` and rerun it.
- That nothing sticks up from the motor plate where the struts cross it (2 frame bolts beside each motor, 26 mm from
  its axis). The model shows nothing there.

Motor screws get 3 mm longer than stock (the hub is 3 mm). Use blue threadlocker and check them after the first 3
flights: PETG settles a little under screws.

Hardware: 30 M2.5 heat-set inserts (6 rod clamps x 3, 6 guard right halves x 1, LiDAR saddle 2, mast 4),
12 x M2.5 x 6 (clamps to carrier), 6 x M2.5 x 16 (clamp pinch screws), 2 x M2.5 x 10 (LiDAR strap),
6 x M2.5 x 12 (guard halves), 18 x M3 motor screws 3 mm longer than stock, blue threadlocker,
1 x 1/4-20 x 1/2 in screw + washer (LiDAR), 2 x 5/32 in x 3/8 in steel dowel pins (LiDAR), 2 x 20 mm velcro straps
(battery), 1 x 15 mm strap (camera), about 1 m of Dyneema or Kevlar cord (safety cord).

**Fitting it:** the stock battery deck and its clips come off. Loosen the damper clamps, slide the rods out, put 3
clamps on each rod (slits facing the centre), slide the rods back, then screw the carrier (front 4 clamps) and the
interface-box plate (back 2 clamps) to them from below. The clamps sit at 112 mm behind, 40 behind and 56 in front of
the frame centre, clear of the dampers. Tighten the 6 pinch screws through the holes in the carrier, then tie the
safety cord.

## Rain covers (ASA, collar TPU; weights as sliced)

Same idea as Widow's: fly, land and sit in light to moderate rain, by shedding water, not sealing it out. **Not tested
yet**, so they are rain covers, not waterproofing. `Fusion Scripts/rain_cover.py` builds them; files are in
`Printed Parts/Rain Cover` (the collar and visor are Widow's prints).

| Part | Weight | Covers | Fixed by |
|---|---|---|---|
| Rain canopy | 91.1 | everything on the top plate: Pixhawk, Pi, radios, step-down, power module, GPS module. Only the GPS antenna pokes out | sits on 3 arm tubes (left, front right, back right); a 12 mm velcro strap through each strap block and round the tube |
| Antenna collar (TPU) | 4.0 | the roof hole round the GPS antenna | pushed down the antenna onto the roof |
| Camera visor | 5.7 | the lenses, from above and ahead | VHB tape on the carrier's top, over the cradle |
| Interface-box cover | 15.4 | the LiDAR interface box, on all four sides; open underneath so the cables drop out in a drip loop | its sheet goes between the plate and the box; the 4 rear clamp screws go through it (use M2.5 x 10 there instead of x 6) |
| ESC cover (x6) | 11.0 each | each ESC, from above and the sides | roof on the arm tube, 2 zip ties |
| Motor cap (x6) | 2.7 each | the holes in the top of each motor bell | under the prop, on the prop's 2 screws; it spins with it |

The canopy prints roof-down with no supports (2 h 30 min, `rain-canopy.gcode.3mf`). The ESC covers, motor caps and
interface-box cover print together (2 h 42 min, `esc-covers-motor-caps-box-cover.gcode.3mf`).

What the rest of the payload relies on:
- The Puck LITE is sealed by Velodyne (IP67 on the datasheet). Its plug and the interface box are not: the cover
  handles the box, and the plug gets self-amalgamating tape.
- The battery hangs under the carrier plate, which is its roof. It stays open at the sides on purpose (heat, gas).
- 12 V step-down: heat-shrink it. It sits on top of the interface-box plate, half under the frame.
- The Tarot's bottom plate is the power board. Conformal coat its solder joints and the ESCs; keep it off connectors.
- Every cable into the canopy comes up from below with a drip loop. Dielectric grease in the battery and ESC plugs.

**Guesses to fix before printing** (marked EST in `rain_cover.py`): I don't have the AK8 ESC's size or where the ESCs
sit, so the ESC cover is drawn for a 68 x 34 x 18 ESC zip-tied under the arm tube. The motor cap assumes 2 x M3 prop
screws 12 mm apart and a 46 mm bell. The interface box size is still the 75 x 53 x 28 guess. Measure those three,
change the numbers, rerun the script.

What it costs: 198 g, 4.7 % more weight, about 7 % less hover time (EST). The LiDAR still sees from straight down to
past horizontal both ways (103° forward, 96° back; the box cover takes 5° off the back). Camera view unchanged.

Testing is the same five steps as in Widow's README (fit, dry hover with vibration and GPS compared, heat, bench
spray with indicator paper, then rain). The motor caps spin with the props: if the vibration in the log goes up
with them on, take them off.

## FEA

`FEA/fea.py` meshes the STEP files the Fusion scripts export and solves them (linear static, quadratic tetrahedra;
`pip install gmsh scikit-fem pyamg matplotlib`, then `python3 fea.py guard` or `python3 fea.py carrier`). Results and
stress pictures land in `FEA/`. Parts are modelled solid; strengths are conservative (PETG 30 MPa, PAHT-CF 25 MPa
across layers).

**Payload carrier, 8 g hard landing** (LiDAR 46 N on its wall, battery 52 N in its straps, its own weight; held at
the 4 clamps): peak 11.5 MPa where the LiDAR wall and its gussets meet the plate, safety factor 2.2. It moves 0.8 mm
at most, so the LiDAR stays where its static transform says.

**Propeller guard** (held under the motor), against the old arm-clamp guard under the same loads (the old one held
rigidly at its clamp, which flatters it):

| Load on one guard | New: ring moves toward prop | New: peak stress | Old: ring moves toward prop | Old: peak stress |
|---|---|---|---|---|
| 1 kg hand push at the outer point | 5.0 mm | 17 MPa | 27.5 mm | 32 MPa |
| 1 g (4.3 kg) bump at the outer point | 22 mm | 74 MPa | 118 mm | 137 MPa |
| 1 g bump from the side | 16 mm | 85 MPa | 107 mm | 130 MPa |
| 1 g bump between struts | 33 mm | 109 MPa | 76 mm | 97 MPa |
| lifted by the ring, 1 kg | 9 mm | 50 MPa | 37 mm | 52 MPa |

What it means: the new guard is about 5 times stiffer than the old one for the same weight, and a firm push stays
under the 8 mm tip gap (5 mm). A full-weight hit on one guard still bends it into the prop and overloads the PETG,
as it did the old one: the guards stop brushes and pushes, not crashes, and are cheap to reprint. Don't lift the drone
by the guards. Big deflections are past where a linear model is accurate; read them as "a lot".

## The LiDAR

It hangs on its side with its spin axis across the drone, so the scan sweeps front to back through the ground
straight under the drone, where the landing spot is. Mounted upright it would only see a ring about 11 m away at
3 m up. From the model, nothing blocks the scan from straight down to just past horizontal, forward and back. It
sees ±15° across the drone: a strip 1.6 m wide at 3 m up. Its lowest point is 88 mm above the ground when landed.

- Base bolts to the carrier's side wall (on the drone's left): one 1/4-20 screw in the centre, two dowel pins.
- A printed strap round the top cap (outside the scan window) bolts to a saddle on the carrier, so the screw is not
  the only thing holding it.
- Turn it so the cable leaves up and forward at 45°; it goes up through the slot in the carrier. Velodyne's 0° mark
  (opposite the cable) then points down and back at 45°, and the top of the LiDAR faces right. Check in RViz before
  setting the static transform.
- Optical centre from the centre of mass (PX4 frame, forward/right/down): 0.044 / 0.002 / 0.072 m.
- Data & Comms is still testing orientations (Sensor-Control-Data-Gathering #61). Only the carrier's wall decides
  it, so a different orientation means reprinting the carrier, nothing else.

## Wiring

- **LiDAR power:** battery (from the Tarot's power board) -> 12 V step-down -> interface box power input. The LiDAR
  takes 9-18 V at about 8 W; never feed it the 6S directly (up to 25.2 V).
- **LiDAR data:** LiDAR cable -> interface box -> Ethernet -> Raspberry Pi 5's Ethernet port on the top plate. The
  LiDAR's factory address is 192.168.1.201, so give the Pi's `eth0` a static 192.168.1.x address.
- **Camera:** OAK-D USB-C up through the carrier's window to the Pi. Short, shielded cable (USB 3 noise reaches the GPS).
- **Pi power:** 5 V step-down on the top plate.

## Whole drone

About **4462** g with the guards and rain covers (4264 without the covers). The centre of mass is 2.3 mm behind the
frame centre and 0.2 mm right, fine to fly. The
interface box weight is a guess; weigh it and the real centre moves.

Same interference rules as Widow: fly on the GPS compass, turn the Pixhawk's internal compass off, keep the SiK
antenna 10 cm from the GPS. The Tarot's bottom plate carries all the motor current, so check the compass against
throttle before trusting it.

| Parameter | Value |
|---|---|
| `EKF2_IMU_POS_X` / `_Y` / `_Z` | 0.002 / 0.000 / -0.064 |
| `EKF2_GPS_POS_X` / `_Y` / `_Z` | -0.053 / -0.040 / -0.126 |

The camera lens is 134 mm forward of the centre of mass and 37 mm below it. It is tilted 10° down so it still sees
above level when the drone tips forward to fly. The front props, guards and arms cover about 14 % of its view, all in
the top corners. They are always the same close pixels: mask them in software (or drop depth under 0.5 m).

## Still to measure

Payload rod size and positions, the top plate holes, the interface box, the 12 V step-down, the battery, the prop
tip height above the motor plate (`PROP_Z`, 31.8 mm in the model before the 3 mm hub), where the motor screws sit
(`HOLE_ANG0`), anything sticking up from the motor plate.
