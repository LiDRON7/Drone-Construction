# Widow: components, sizes and weights

Widow is the Storm Drone 6 V3 hexacopter, flown as a **hexa X (PX4 airframe 6001)**. The battery plate's pointed
tips, which run along the battery, point at the gaps between arms 1 and 2 (the front) and between arms 4 and 5 (the
back). Its flat sides, which run along the feet, face arms 3 and 6, so the leg holes line up with those arms and the
legs sit under them. The GPS stand
sits between arms 5 and 6. The camera looks out of the front gap, and the Pixhawk and GPS arrows point the same way.

Arm numbers, seen from above with the camera at the top: 1 front right, 2 front left, 3 left, 4 back left, 5 back
right, 6 right (they go down going clockwise). See `widow-top-view.png` in this folder.

Sizes are mm, weights g. **Bold** means measured with calipers or the lab scale on 2026-11-09. Anything else is the
maker's spec or an estimate (EST), still to measure. The full-drone model (`Fusion Scripts/full_drone.py`) uses these
numbers.

## Bought parts

| Part | Where it sits | Size | Weight |
|---|---|---|---|
| Pixhawk 6X Mini (FMU + baseboard) | top plate, centre (14 mm left), on its foam pads, arrow forward | 72.8 x 43.4 x 31 (spec) | **54.49** |
| Holybro H-RTK F9P Helical (module + antenna) | on the metal GPS stand at the back right of the top plate (between arms 5 and 6), arrow forward | module **53.86** long, 35.3 wide (EST), 12.9 tall; antenna 27.5 x 59 (spec) | **57.25** |
| Luxonis OAK-D Pro camera | cradle on the front of the landing-leg mount, 10° down | **96.96** x **29.45** x **23.15** deep (**14.28** body + fins); USB-C **9.45** behind the front face | **89.69** |
| Storm 3S 5500 mAh 11.1 V LiPo | tray in the landing-leg mount, under the battery plate | **135.41** x **43.21** x **31.11** | **361.98** |
| Raspberry Pi 5 in its case, with active cooler | between the top and bottom plates | case not measured | **99.92** |
| RadioLink R9DS receiver (with lead) | top plate, front right lobe (over arm 1), velcro | 43 x 24 x 15 (spec) | **12.53** |
| SiK 915 MHz telemetry radio | top plate, left lobe (over arm 3), velcro; antenna out along arm 3, away from the GPS | 53 x 28 x 10.7 (EST) | 15 (EST) |
| Power distribution board | under the bottom plate, next to the battery lead | 45 x 35 x 6 (EST) | 15 (EST) |
| Storm M2208 motor (x6) | arm tips, 216 apart | **27.47** across | 44 each (EST) |
| uxcell 8045 nylon prop (x6) | on the motors | 203 across | 8 each (EST) |
| F35A ESC (x6) | inside the arms | 30 x 16 x 5 (spec) | 7.5 each (spec) |
| Metal GPS stand | top plate, back right, over the 17 mm square of holes | module sits **170** above the plate; base 22 x 25 and 5 mm rod (EST) | 30 (EST) |

## Frame

Three carbon plates and six truss arms.

| What | Size |
|---|---|
| Top plate | six-pointed star, a lobe over each arm, a window over the Pi. Each lobe has 2 arm screws, about 16 apart and 83 from the centre (EST, photo). Its only other holes are a 17 mm square of 4 M3 holes, about 33 from the centre, between arms 5 and 6 (EST), on a strip **22.94**-**30.74** wide |
| Top to bottom plate | **39.11** with the **9.12** spacers in (30.0 without) |
| Bottom plate | same star as the top |
| Battery plate | hexagon **120.09** across the flats, **105.38** tip to tip (**90.73** tip to far corner), **12** under the bottom plate, tips at the 1-2 and 4-5 gaps, legs under arms 3 and 6 |
| Leg screw squares (battery plate) | 2 squares of 4 M3, **24.62** fore-aft x **25.95** across, **75.05** apart, left and right of centre |
| Motor spacing | **216** between neighbours |
| Leg tubes / skid tubes / skid foam | **16.04** / **9.89** OD; foam 12 ID, 18 OD |

## Printed parts (PETG-CF, propeller guards PETG; weights as sliced)

Every screw that goes into a printed part goes into an M2.5 heat-set insert (holes are 3.6 mm wide, 4 mm deep inserts;
change `INSERT_D`/`INSERT_L` in the scripts if yours differ).

| Part | Weight | Holds | Fixed by |
|---|---|---|---|
| Propeller guard (x6) | 33.7 each | one prop each | the motor screws (no insert: they go into the motor) |
| Landing-leg mount | 79.2 | both legs, the battery and the OAK-D Pro | 8 x M2.5 x 6 from above the battery plate into 8 inserts; 2 x M2.5 x 25 cross bolts into 2 inserts |
| Landing-leg foot (x2) | 22.4 each | skid tube | M2.5 x 25 cross bolt into 1 insert; 2 x M2.5 x 6 grub screws in 2 inserts |

Inserts for one drone: 8 + 2 (mount) + 2 x 3 (feet) = **16**.

**Propeller guards** are the same style as Huntsman's: a 2 mm hub under the motor, four T-section struts with wide
roots, and a ring with a solid band at the prop tips, a lip round its top edge, a diagonal lattice below and a fin at
each strut end. One piece, 1 h 26 min, PETG, 3 walls 15 % infill (`propeller-guard.gcode.3mf`). FEA (`FEA/fea.py
widow_guard`, results in `FEA/`) against the old solid-ring guard, same loads:

| Load on one guard | New: ring moves toward prop | New: peak stress | Old: ring moves toward prop | Old: peak stress |
|---|---|---|---|---|
| 1 kg hand push at the outer point | 2.9 mm | 18 MPa | 4.1 mm | 31 MPa |
| 1 g (1.9 kg) bump at the outer point | 5.5 mm | 34 MPa | 7.8 mm | 58 MPa |
| 1 g bump from the side | 3.3 mm | 32 MPa | 6.2 mm | 122 MPa |
| 1 g bump between struts | 7.1 mm | 45 MPa | 15.6 mm | 72 MPa |
| lifted by the ring, 1 kg | 6.8 mm | 75 MPa | 12.7 mm | 146 MPa |

The tip gap is 6.4 mm, so a full-weight hit at the outer point now stays clear of the prop (the old one reached it).
PETG is good for about 30 MPa, so the 1 g bumps sit right at its limit: it bends there, it doesn't shatter. Don't
lift the drone by the guards. 1.7 g heavier per guard than the old one.

## Whole drone

About **1878** g with the battery, guards and printed parts. The centre of mass
is 1.7 mm forward of the frame centre and 0.7 mm to the right, which is fine; sliding the battery about 9 mm back centres it.

Why the camera sits where it does: under the front gap, tilted 10° down. No arm is straight ahead. Flying forward
tips the drone nose-down about 10°, so at 10° the camera still sees 7.5° above level; at 15° the horizon sat on the
top edge. No arm or prop is in its view (80° x 55°, OAK-D Pro stereo). The guard edges nick the top corners, 1.6 %
of the view: always the same close pixels, so mask them in software. The skids stay outside its width, and it sits
inside the guard rings if the drone hits something. The cradle holds it by its ends,
so nothing covers the lenses. A 15 mm strap under it keeps it in, and the USB-C cable goes up through a window to
the Pi.

## Before flying

1. Mount the battery plate with its tips between arms 1-2 and 4-5 and its flat sides facing arms 3 and 6; the leg
   mount and camera turn with it. The camera goes at the 1-2 end. In QGroundControl set the airframe to Generic Hexarotor x (6001). In the
   Actuators tab, assign each ESC output to the motor position PX4 shows and run the motor test. Spin directions
   don't change.
2. Mount the Pixhawk and the GPS with their arrows toward the camera, and recalibrate the sensors and compass.
   The propeller guards sit under the motors, inside the motor screws' grip: put blue threadlocker on those screws
   and check them after the first 3 flights (PETG settles a little under screws).
3. Set the sensor offsets from the centre of mass:

| Parameter | Value |
|---|---|
| `EKF2_IMU_POS_X` / `_Y` / `_Z` | -0.002 / -0.015 / -0.062 |
| `EKF2_GPS_POS_X` / `_Y` / `_Z` | -0.015 / 0.030 / -0.246 |

The camera lens is 83 mm forward of the centre of mass and 48 mm below it (for the OAK-D's extrinsics).

## Interference

The GPS module's compass, the GPS receiver and the radios sit close together, so:

- Fly on the GPS module's compass. Turn the Pixhawk's internal compass off (set its `CAL_MAGx_PRIO` to 0).
- The power board carries all the motor current, so it stays under the bottom plate, away from both compasses. Twist
  the battery and ESC power leads in pairs and keep them short.
- Keep the SiK antenna at least 10 cm from the GPS (out along arm 3 or hanging under the drone).
- Hold a magnet to the GPS stand. If it sticks, the stand is steel and bends the compass reading: swap it for an
  aluminium or carbon one.
- Use a short shielded USB-C cable from the OAK-D to the Pi and keep it away from the GPS stand: USB 3 noise lands in the
  GPS band.
- Check: in PX4 Flight Review the magnetic field strength should stay flat as throttle rises, and the GPS satellite
  count shouldn't drop when the Pi and camera start.
