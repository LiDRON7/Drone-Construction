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
| Velodyne interface box | under the carrier, back, velcro | 75 x 53 x 28 (EST) | 80 (EST) |
| 12 V step-down for the LiDAR (6S in, 3 A out) | top of the carrier, between the rods, velcro | 45 x 26 x 14 (EST) | 25 (EST) |
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

## Printed parts (PETG, weights as sliced)

| Part | Weight | Holds | Fixed by |
|---|---|---|---|
| Payload carrier | 213 | LiDAR, battery, camera, interface box, 12 V step-down | 6 rod clamps |
| LiDAR strap | 10.7 | the LiDAR's top cap | 2 x M2.5 x 10 up into 2 inserts in the carrier's saddle |
| Rod clamp (x6) | 6.8 each | the carrier, on the payload rods | slides onto a rod; 1 x M2.5 x 6 grub screw into the rod; 2 x M2.5 x 6 up through the carrier |
| Propeller guard (x6) | 46.5 each | one 13" prop each | its arm clamp: 2 x M2.5 x 8 down into 2 inserts |
| Arm clamp (x6) | 5.5 each | a guard, on the 16 mm arm tube, 88 mm inside the motor | 1 x M2.5 x 25 cross bolt under the tube into 1 insert |
| GPS mast | 38.1 | GPS module | Widow's mast; 4 x M2.5 x 6 from under the top plate (drill a 17 mm square of 4 holes if the Tarot plate has none there) |

Carrier, LiDAR strap and 6 rod clamps print together: 263 g, 9 h 27 min (`Printed Parts/Payload Carrier/payload-carrier.gcode`).
One guard + one arm clamp per plate: 52 g, 1 h 53 min (`Printed Parts/Propeller Guard/propeller-guard.gcode`, print 6).

**Propeller guards.** They clamp to the arm tube instead of going under the motor, so the motors and their screws
stay untouched. The ring wraps the outer 190° of each prop with an 8 mm tip gap. It is solid at the prop tips and
a 45° diamond lattice below them, which prints without supports and saves weight. Four triangular struts run back
to the clamp, 12 mm under the prop. The struts are sized so the drone's whole weight pushing sideways on one guard
doesn't buckle them (safety factor 1.7). Print them with 3 walls and 15 % infill (the profile in the `.3mf`): the
parts are thin enough that the walls do the work. Before printing, measure how high the prop tips sit above the
motor mount (`PROP_Z`, 25 mm in the model); the ring's solid band is centred on it.

Hardware: 42 M2.5 heat-set inserts (6 rod clamps x 3, 6 arm clamps x 3, LiDAR saddle 2, mast 4), 12 x M2.5 x 6 screws,
6 x M2.5 x 6 grub screws, 2 x M2.5 x 10 (LiDAR strap),
12 x M2.5 x 8 (guards), 6 x M2.5 x 25 (arm clamp cross bolts),
1 x 1/4-20 x 1/2 in screw + washer (LiDAR), 2 x 5/32 in x 3/8 in steel dowel pins (LiDAR), 2 x 20 mm velcro straps
(battery), 1 x 15 mm strap (camera).

**Fitting it:** the stock battery deck and its clips come off. Loosen the damper clamps, slide the rods out, put 3
clamps on each rod (grub screws facing out), slide the rods back, then screw the carrier to the clamps from below.
The clamps sit at 112 mm behind, 40 behind and 56 in front of the frame centre, clear of the dampers.

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
- Optical centre from the centre of mass (PX4 frame, forward/right/down): 0.043 / 0.002 / 0.069 m.
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

About **4269** g with the guards. The centre of mass is 1.7 mm behind the frame centre and 0.4 mm right, fine to fly. The
interface box weight is a guess; weigh it and the real centre moves.

Same interference rules as Widow: fly on the GPS compass, turn the Pixhawk's internal compass off, keep the SiK
antenna 10 cm from the GPS. The Tarot's bottom plate carries all the motor current, so check the compass against
throttle before trusting it.

| Parameter | Value |
|---|---|
| `EKF2_IMU_POS_X` / `_Y` / `_Z` | 0.002 / 0.000 / -0.067 |
| `EKF2_GPS_POS_X` / `_Y` / `_Z` | -0.053 / -0.040 / -0.129 |

The camera lens is 133 mm forward of the centre of mass and 34 mm below it. It is tilted 10° down so it still sees
above level when the drone tips forward to fly. The front props, guards and arms cover about 9 % of its view, all in
the top corners. They are always the same close pixels: mask them in software (or drop depth under 0.5 m).

## Still to measure

Payload rod size and positions, the top plate holes, the interface box, the 12 V step-down, the battery, the prop
tip height above the motor mounts, the arm tube diameter.
