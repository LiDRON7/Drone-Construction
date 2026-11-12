# Widow (Storm Drone 6 V3, flown as hexa X: PX4 6001) layout model. Runs inside Fusion via the
# Fusion360MCP add-in: python3 run_in_fusion.py execute_code -code full_drone.py
# Saves the assembly into ../Widow/Full Drone/ and one model per bought part into ../Widow/Reference Parts/
# (HERE is set by run_in_fusion.py).
# Frame: x forward = the camera side, the gap between the two front arms. The battery plate's tips point at the front
# and back gaps; its flat sides (and the legs) face the arms at +-90 deg. The GPS mast sits in the front-right gap.
# y left, z up, mm. Origin = frame centre, bottom face of the bottom plate.
# Three carbon plates: top star, bottom star, and the battery plate (a hexagon) hanging 12 mm under the bottom star.
# Every part is its own component with its real (or flagged estimated) mass, so
# Fusion's Inspect > Center of Mass works on the whole drone.
# "EST" = not published or not measured yet; measure and update the number.
import adsk.core, adsk.fusion, math

import os
PROP_STL = os.path.join(HERE, '..', 'Widow', 'Reference Parts', 'propulsion-propeller-8x4.5.stl')
PROP_HUB = (27.72, 5.01, 101.61)   # hub centre in the blade file, mm (spin axis = file y)
PROP_HUB_G = 1.0                   # share of the prop weight on the hub body
GUARD_STEP = os.path.join(HERE, '..', 'Widow', 'Printed Parts', 'Propeller Guard', 'propeller-guard.step')
BATT_X = -13.0  # battery fore/aft position; the balance trim
MAST_G = 38.1                 # printed GPS mast, g as sliced

R_MOTOR   = 216.0   # motor centre radius (user-measured neighbour spacing; hexagon side = radius)
TOP_D     = 150.0   # carbon top plate, scaled from photo 4 (EST +-15 mm): tip-to-tip size of the star below
STAR_R_TIP, STAR_R_IN, STAR_LOBE_W = 91.0, 45.0, 32.0   # six-lobed star, a lobe over each arm; arm screws 83 out (EST from photos)
PLATE_T   = 2.0
GAP       = 39.11 - 9.12   # arm height at the root: plate-to-plate 39.11 measured with the 9.12 spacers in
SPACER    = 9.12    # measured spacers between the arms and the top plate; set 0 if they come out
BP_GAP    = 12.0    # measured: battery plate this far under the bottom plate
BP_W, BP_L, BP_SIDE = 120.09, 105.38, 30.6   # battery plate hexagon: across the flats (y), tip to tip (x), flat side
# (BP_SIDE from the 90.73 tip-to-far-corner measurement)
ARM_W     = 24.0    # truss arm width (photos)
ARM_R0    = 55.0    # EST where the arm root starts, measured from the frame centre
H_TIP     = 14.0    # EST arm height at the motor; the arm underside tapers up from root to tip
ARM_WALL, ARM_FLOOR = 3.0, 2.0   # open-top channel: two side walls + a floor, ESC rides inside
ARM_TOP   = PLATE_T + GAP        # top of the arms, where the motors sit
TOP_Z     = ARM_TOP + SPACER     # underside of the top plate
ARM0      = -30.0                # arm 1 at front right; numbers rise counter-clockwise seen from above (the team's numbering:
                                 # mast between arms 5 and 6, legs under 6 and 3, battery tips between 1-2 (front) and 4-5)
BP_Z      = -BP_GAP - PLATE_T    # underside of the battery plate
WITH_GUARDS = True

# name, shape, dims (mm), centre (x, y, z) mm, mass g, rot_z deg, rot_y deg, rot_x deg (applied x, then y, then z)
PARTS = []
def add(name, shape, dims, c, m, rz=0.0, ry=0.0, rx=0.0):
    PARTS.append((name, shape, dims, c, m, rz, ry, rx))

ARM_L = R_MOTOR + 15 - ARM_R0
arm_floor_z = lambda r: PLATE_T + (r - ARM_R0) / ARM_L * (GAP - H_TIP) + ARM_FLOOR  # inside floor at radius r

# --- frame (EST split of the 870 g body weight minus motors, ESCs, props, stock electronics)
def star_points():
    pts = []
    for i in range(6):
        a = math.radians(ARM0 + 60 * i); d = math.asin(STAR_LOBE_W / 2 / STAR_R_TIP)
        v = a - math.radians(30)
        pts += [(STAR_R_IN*math.cos(v), STAR_R_IN*math.sin(v)),
                (STAR_R_TIP*math.cos(a-d), STAR_R_TIP*math.sin(a-d)), (STAR_R_TIP*math.cos(a+d), STAR_R_TIP*math.sin(a+d))]
    return pts
add('frame bottom plate', 'poly', (star_points(), PLATE_T), (0, 0, PLATE_T/2), 45)            # same star as the top plate (user)
bp_pts = [(BP_L/2, 0), (BP_SIDE/2, -BP_W/2), (-BP_SIDE/2, -BP_W/2), (-BP_L/2, 0), (-BP_SIDE/2, BP_W/2), (BP_SIDE/2, BP_W/2)]
add('frame battery plate', 'poly', (bp_pts, PLATE_T), (0, 0, BP_Z + PLATE_T/2), 15)            # EST 15 g (taken off the bottom plate's EST)
add('frame top plate', 'poly', (star_points(), PLATE_T), (0, 0, TOP_Z + PLATE_T/2), 60)
for i in range(6):
    a = ARM0 + 60.0 * i               # hexa X: arms at 30, 90, ... deg; none straight ahead
    t = math.radians(a); cx, cy = math.cos(t), math.sin(t)
    rm = ARM_R0 + ARM_L/2
    add(f'frame arm #{i+1}', 'arm', (ARM_L, ARM_W, GAP, H_TIP), (rm*cx, rm*cy, PLATE_T + GAP/2), 30, rz=a)
    mz = ARM_TOP + (2.0 if WITH_GUARDS else 0.0)   # motor base; the guard hub (2 mm) sits between arm and motor
    add(f'propulsion motor Storm M2208 #{i+1}', 'cyl', (27.5, 20.0), (R_MOTOR*cx, R_MOTOR*cy, mz + 10), 44)        # EST 44 g; prop seat 20 mm above the base (measured)
    add(f'propulsion propeller 8x4.5 #{i+1}', 'prop', (19.6, 7.0), (R_MOTOR*cx, R_MOTOR*cy, mz + 20 + 3.5), 8, rz=a)  # real blade mesh + 8 g hub body (EST weight)
    add(f'propulsion ESC F35A #{i+1}', 'box', (30, 16, 5), (140*cx, 140*cy, arm_floor_z(140) + 2.5 + 1.5), 7.5, rz=a)  # +1.5: flat ESC on a sloped floor  # runs inside the arm
    if WITH_GUARDS:
        add(f'printed propeller guard #{i+1}', 'step', (GUARD_STEP,), (R_MOTOR*cx, R_MOTOR*cy, ARM_TOP), 33.7, rz=a)   # the real printed guard, hub on the arm pad
# landing gear: one printed mount takes all 8 screws under the battery plate, holds both legs (leaning out LEAN deg)
# and the battery tray between them (Printed Parts/Landing Leg, landing_leg.py). The mount is drawn sockets-up, so rx=180
# hangs it under the plate; the foot is a straight T drawn with its skid along x at its origin, turned on the skid to
# line up with the leaning tube.
LEG_DIR = os.path.join(HERE, '..', 'Widow', 'Printed Parts', 'Landing Leg')
LEG_Y, FL_T, LEAN, TUBE_L = 37.52, 6.0, 20.0, 184.0   # must match leg_y_mm, FL_T, LEAN, tube_l_mm in landing_leg.py
FOOT_FLOOR = 13.1                                     # foot_floor_mm in landing_leg.py: socket floor from the skid axis
sa, ca = math.sin(math.radians(LEAN)), math.cos(math.radians(LEAN))
add('printed landing leg mount', 'step', (os.path.join(LEG_DIR, 'landing-leg-mount.step'),), (0, 0, BP_Z), 79.2, rx=180)   # as sliced
for sy in (1, -1):
    side = "left" if sy > 0 else "right"
    top = (0, sy*LEG_Y, BP_Z - FL_T)                          # where the tube's top end sits in its socket
    end = (0, top[1] + sy*TUBE_L*sa, top[2] - TUBE_L*ca)
    skid = (0, end[1] + sy*FOOT_FLOOR*sa, end[2] - FOOT_FLOOR*ca)
    add(f'frame landing leg tube #{side}', 'cyl', (16, TUBE_L - 0.2), (0, (top[1] + end[1])/2, (top[2] + end[2])/2), 18, rx=sy*LEAN)   # carbon, EST g. 0.2 short: ends touch the floors
    add(f'printed landing leg foot #{side}', 'step', (os.path.join(LEG_DIR, 'landing-leg-foot.step'),), skid, 22.4, rx=sy*LEAN)  # as sliced
    add(f'frame landing skid tube #{side}', 'cyl', (10, 220), skid, 15, ry=90)        # stock carbon tube, EST 10 mm x 220 mm

# --- electronics: the Pi between the plates, the battery and power board under them, the flight parts on the top
# plate (velcro, Pixhawk on its foam). Kept apart for the compass and GPS: the power board's motor current stays
# under the bottom plate, the SiK sits on the far lobe from the GPS mast with its antenna out along arm 3.
FC_Z = TOP_Z + PLATE_T                # top of the top plate: the parts sit here
SQ_X, SQ_Y = -13.2, -30.5             # EST: centre of the top plate's 17 mm hole square = the mast axis
add('flight controller foam mount', 'box', (75, 46, 6), (0, 14, FC_Z + 3), 8)
add('flight controller Pixhawk 6X Mini baseboard', 'box', (72.8, 43.4, 14.2), (0, 14, FC_Z + 6 + 7.1), 31.5)   # 54.49 g measured with the FMU
add('flight controller Pixhawk 6X Mini FMU', 'box', (38.8, 31.8, 16.8), (0, 14, FC_Z + 6 + 14.2 + 8.4), 23)
# printed GPS mast (Printed Parts/GPS Mast, gps_mast.py) on the top plate, module in the cradle 48 mm up
MAST_STEP = os.path.join(HERE, '..', 'Widow', 'Printed Parts', 'GPS Mast', 'gps-mast.step')
GPS_Z = FC_Z + 48.0                   # cradle floor top; must match CRADLE_Z + FLOOR in gps_mast.py
add('printed GPS mast', 'step', (MAST_STEP,), (SQ_X, SQ_Y, FC_Z), MAST_G)
add('gps H-RTK F9P Helical module', 'box', (53.86, 35.3, 12.9), (SQ_X, SQ_Y, GPS_Z + 6.45), 38.25)
add('gps H-RTK F9P Helical antenna', 'cyl', (27.5, 59), (SQ_X, SQ_Y, GPS_Z + 12.9 + 29.5), 19)      # 57.25 g total measured, split EST
add('radio telemetry SiK V3 915MHz', 'box', (28, 53, 10.7), (0, 66, FC_Z + 5.35), 15)             # left lobe, over arm 3, ~100 mm from the GPS; EST weight
add('radio RC receiver RadioLink R9DS', 'box', (43, 24, 15), (46, -28, FC_Z + 7.5), 12.53)   # front right lobe; measured with its lead
add('power distribution board', 'box', (45, 35, 6), (0, 0, -3), 15)       # under the bottom plate, next to the battery lead, away from the compass; EST size and weight
# between the plates: Pi on 3 mm standoffs on the bottom plate, step-down beside it, wiring above
add('computer Raspberry Pi 5 with cooler', 'box', (85, 56, 20), (0, 0, PLATE_T + 3 + 10), 99.92)    # measured in its case with the active cooler; size EST
add('power step-down 5V', 'box', (20, 16, 8), (0, 40, PLATE_T + 4), 15)                # EST size
add('power module Holybro PM02D', 'box', (20, 16, 10), (0, -40, PLATE_T + 5), 20)              # EST size
add('misc wiring bundle (placeholder)', 'box', (60, 60, 5), (0, 0, TOP_Z - 3.5), 50)                  # EST
# under the battery plate: battery strapped in the leg mount's tray, OAK-D Pro in the cradle on the mount's front
# (landing_leg.py), facing the front (between the two front arms) and TILT deg down. The Pixhawk and GPS arrows point this way too.
CAM_C, CAM_LENS, TILT = (72.42, 21.97), (84.21, 24.05), 10.0   # cam_centre_print_mm, cam_lens_print_mm, TILT in landing_leg.py
CAM_BACK_X = 58.0                                               # CAM_BACK_X in landing_leg.py
ct, st = math.cos(math.radians(TILT)), math.sin(math.radians(TILT))
cam_at = lambda k: (CAM_C[0] + k*ct, 0, BP_Z - CAM_C[1] - k*st)   # k mm forward along the camera axis from the pocket centre
add('camera Luxonis OAK-D Pro', 'box', (14.28, 96.96, 29.45), cam_at((23.55 - 14.28)/2 - 0.2), 77.69, ry=TILT)        # body; 89.69 g measured in all
add('camera Luxonis OAK-D Pro heatsink', 'box', (8.87, 79, 25), cam_at((23.55 - 8.87)/2 - 0.2 - 14.28), 12, ry=TILT)   # fins stop ~9 mm from each end (photo); split EST
add('power battery 3S 5500mAh LiPo', 'box', (135.4, 43.2, 31.1), (BATT_X, 0, BP_Z - FL_T - 31.1/2), 361.98)    # measured; x is the balance trim
# self-check: rays over the camera's view (OAK-D Pro stereo 80 x 55 deg, spec sheet). No prop or arm may show;
# the guard rings may nick the top corners (GUARD_MAX % of the view): fixed, close pixels the software masks.
HFOV, VFOV, GUARD_MAX = 80.0, 55.0, 3.0
lx, lz = CAM_LENS[0], BP_Z - CAM_LENS[1]
def _seg_hit(o, d, a, b, r):          # ray o + s d (s >= 0) passes within r of segment ab
    dot = lambda p, q: sum(p[k]*q[k] for k in range(3))
    v = [b[k]-a[k] for k in range(3)]; w = [o[k]-a[k] for k in range(3)]
    A_, B_, C_, D_, E_ = dot(d, d), dot(d, v), dot(v, v), dot(d, w), dot(v, w)
    den = A_*C_ - B_*B_
    s, t = ((B_*E_ - C_*D_)/den, (A_*E_ - B_*D_)/den) if den > 1e-9 else (0.0, E_/C_)
    s, t = max(s, 0.0), min(max(t, 0.0), 1.0)
    p = [w[k] + s*d[k] - t*v[k] for k in range(3)]
    return dot(p, p) <= r*r
def _cam_ray(d):
    o = (lx, 0.0, lz)
    for i in range(6):
        a = ARM0 + 60*i; c, s_ = math.cos(math.radians(a)), math.sin(math.radians(a)); cx, cy = R_MOTOR*c, R_MOTOR*s_
        if _seg_hit(o, d, (ARM_R0*c, ARM_R0*s_, ARM_TOP - GAP/2), ((R_MOTOR + 15)*c, (R_MOTOR + 15)*s_, ARM_TOP - H_TIP/2), H_TIP/2 + 2):
            return 'arm'
        if d[2] > 1e-9:
            t = (ARM_TOP + 2 + 23.5 - lz)/d[2]
            if math.hypot(lx + t*d[0] - cx, t*d[1] - cy) <= 101.6:
                return 'prop'
        px, py = lx - cx, -cy; A_ = d[0]**2 + d[1]**2; B_ = 2*(px*d[0] + py*d[1]); C_ = px*px + py*py - 110.0**2
        disc = B_*B_ - 4*A_*C_
        if A_ > 1e-12 and disc >= 0:
            for t in ((-B_ - math.sqrt(disc))/(2*A_), (-B_ + math.sqrt(disc))/(2*A_)):
                hz = lz + t*d[2]; rel = (math.degrees(math.atan2(t*d[1] - cy, lx + t*d[0] - cx)) - a + 180) % 360 - 180
                if t > 0 and ARM_TOP <= hz <= ARM_TOP + 30 and abs(rel) <= 95:
                    return 'guard'
    return None
_hits, _n = {}, 0
for _az in range(-int(HFOV/2), int(HFOV/2) + 1, 2):
    for _el in range(-int(VFOV/2), int(VFOV/2) + 1, 2):
        e, z_ = math.radians(-TILT + _el), math.radians(_az); _n += 1
        k = _cam_ray((math.cos(e)*math.cos(z_), math.cos(e)*math.sin(z_), math.sin(e)))
        if k:
            _hits[k] = _hits.get(k, 0) + 1
assert not _hits.get('arm') and not _hits.get('prop'), f'an arm or prop is in the camera view: {_hits}'
assert 100*_hits.get('guard', 0)/_n <= GUARD_MAX, f'guards cover {100*_hits["guard"]/_n:.1f} % of the camera view'
CAM_BLOCKED_PCT = round(100*sum(_hits.values())/_n, 1)
skid_front = (110.0, LEG_Y + TUBE_L*sa + FOOT_FLOOR*sa)                         # skid end: x, |y|
assert math.degrees(math.atan2(skid_front[1], skid_front[0] - lx)) > HFOV/2 + 5, 'a skid is in the camera view'
assert BATT_X + 135.4/2 + 2 <= CAM_BACK_X, 'battery runs into the camera'

# self-check: the Pi must fit between the plates and inside the arm roots
assert math.hypot(85/2, 56/2) < ARM_R0, 'Pi corners hit the arm roots'
assert 3 + 20 <= GAP - 5, 'Pi + standoffs do not fit under the wiring'

# ---------------------------------------------------------------------------
cm = lambda v: v / 10.0
for d in [d for d in app.documents if d.name.lower().startswith(('widow assembly', 'widow-full-drone', 'widow full drone'))]:
    d.close(False)
doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
doc.name = 'Widow full drone'
des = adsk.fusion.Design.cast(app.activeProduct)
root = des.rootComponent
des.designType = adsk.fusion.DesignTypes.DirectDesignType  # no timeline, so occurrence moves stick
lib = app.materialLibraries.itemByName('Fusion Material Library')
base_mat = lib.materials.itemByName('ABS Plastic')

def build_body(comp, shape, dims):
    sk = comp.sketches.add(comp.xYConstructionPlane)
    P = lambda x, y: adsk.core.Point3D.create(cm(x), cm(y), 0)
    if shape == 'box':
        lx, ly, h = dims
        sk.sketchCurves.sketchLines.addTwoPointRectangle(P(-lx/2, -ly/2), P(lx/2, ly/2))
    elif shape == 'cyl':
        d, h = dims
        sk.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0), cm(d/2))
    elif shape == 'poly':  # closed polygon in x-y, extruded h
        pts, h = dims
        for i in range(len(pts)):
            sk.sketchCurves.sketchLines.addByTwoPoints(P(*pts[i]), P(*pts[(i + 1) % len(pts)]))
    elif shape == 'arm':  # local x -L/2 (root) .. L/2 (tip); z 0..H top flat, underside tapers up to H-h_tip at the tip
        L, W, H, ht = dims; h = H
        skx = comp.sketches.add(comp.xZConstructionPlane)
        Q = lambda x, z: skx.modelToSketchSpace(adsk.core.Point3D.create(cm(x), 0, cm(z)))
        bot = lambda x: (x + L/2) / L * (H - ht)
        def poly(sketch, pts):
            for i in range(len(pts)):
                sketch.sketchCurves.sketchLines.addByTwoPoints(pts[i], pts[(i+1) % len(pts)])
        poly(skx, [Q(-L/2, 0), Q(L/2, H - ht), Q(L/2, H), Q(-L/2, H)])
        inp = comp.features.extrudeFeatures.createInput(skx.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        inp.setSymmetricExtent(adsk.core.ValueInput.createByReal(cm(W)), True)
        body = comp.features.extrudeFeatures.add(inp).bodies.item(0)
        # hollow it: open top, ARM_FLOOR floor, solid 30 mm motor pad at the tip, 3 mm at the root
        x0, x1 = -L/2 + 3, L/2 - 30
        sk2 = comp.sketches.add(comp.xZConstructionPlane)
        Q2 = lambda x, z: sk2.modelToSketchSpace(adsk.core.Point3D.create(cm(x), 0, cm(z)))
        poly(sk2, [Q2(x0, bot(x0) + ARM_FLOOR), Q2(x1, bot(x1) + ARM_FLOOR), Q2(x1, H + 1), Q2(x0, H + 1)])
        ci = comp.features.extrudeFeatures.createInput(sk2.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
        ci.setSymmetricExtent(adsk.core.ValueInput.createByReal(cm(W - 2*ARM_WALL)), True)
        ci.participantBodies = [body]
        comp.features.extrudeFeatures.add(ci)
        return body, h, None
    elif shape == 'prop':  # hub cylinder carries the weight; the blade mesh is display only (meshes have no mass in Fusion)
        d, h = dims
        sk.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0), cm(d/2))
        sk.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0), cm(2.5))   # 5 mm shaft hole, as in the blade file
        prof = max(sk.profiles, key=lambda q: q.areaProperties().area)
        inp = comp.features.extrudeFeatures.createInput(prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(cm(h)))
        body = comp.features.extrudeFeatures.add(inp).bodies.item(0)
        # blade file: span along z, spin axis along y, hub centre PROP_HUB. Centre it, turn y up, lift to mid-hub.
        M = adsk.core.Matrix3D.create()
        M.translation = adsk.core.Vector3D.create(-cm(PROP_HUB[0]), -cm(PROP_HUB[1]), -cm(PROP_HUB[2]))
        Rx = adsk.core.Matrix3D.create(); Rx.setToRotation(math.pi/2, adsk.core.Vector3D.create(1, 0, 0), adsk.core.Point3D.create(0, 0, 0))
        M.transformBy(Rx)
        T = adsk.core.Matrix3D.create(); T.translation = adsk.core.Vector3D.create(0, 0, cm(h/2)); M.transformBy(T)
        child = comp.occurrences.addNewComponent(M); child.component.name = 'blades (mesh)'
        child.component.meshBodies.add(PROP_STL, adsk.fusion.MeshUnits.MillimeterMeshUnit)
        return body, h, child.component
    elif shape == 'step':  # a real part imported from its STEP (guard, leg connectors, GPS mast); used in its own coordinates
        im = app.importManager
        im.importToTarget(im.createSTEPImportOptions(dims[0]), comp)
        target = root.allOccurrencesByComponent(comp).item(0)
        for ch in list(comp.occurrences):  # STEP import lands in a child occurrence; pull the body up
            for b in list(ch.bRepBodies):
                b.moveToComponent(target)
            ch.deleteMe()
        assert comp.bRepBodies.count == 1, f'{dims[0]} import gave {comp.bRepBodies.count} bodies'
        return comp.bRepBodies.item(0), 0.0, None
    prof = sk.profiles.item(0)
    inp = comp.features.extrudeFeatures.createInput(prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(cm(h)))
    return comp.features.extrudeFeatures.add(inp).bodies.item(0), h, None

report = []
for name, shape, dims, c, m, rz, ry, rx in PARTS:
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component; comp.name = f'{name} [{m:g} g]'
    body, h, mesh_comp = build_body(comp, shape, dims)
    # density so the body weighs exactly m grams (volume cm^3 -> kg/m^3)
    mat = des.materials.addByCopy(base_mat, f'mat {name}')
    m_body = PROP_HUB_G if mesh_comp else m   # a prop's weight is split: hub body + blade mesh
    mat.materialProperties.itemById('structural_Density').value = (m_body / body.volume) * 1000.0
    body.material = mat
    if mesh_comp:  # Fusion weighs meshes with the component material, so give the blades the rest
        mm = des.materials.addByCopy(base_mat, f'mat {name} blades')
        mm.materialProperties.itemById('structural_Density').value = ((m - PROP_HUB_G) / mesh_comp.physicalProperties.volume) * 1000.0
        mesh_comp.material = mm
    # body sits on local z 0..h: centre it, rotate, then move into place
    M = adsk.core.Matrix3D.create()
    M.translation = adsk.core.Vector3D.create(0, 0, -cm(h)/2)
    O = adsk.core.Point3D.create(0, 0, 0)
    if rx:
        Rx = adsk.core.Matrix3D.create(); Rx.setToRotation(math.radians(rx), adsk.core.Vector3D.create(1, 0, 0), O); M.transformBy(Rx)
    if ry:
        Ry = adsk.core.Matrix3D.create(); Ry.setToRotation(math.radians(ry), adsk.core.Vector3D.create(0, 1, 0), O); M.transformBy(Ry)
    if rz:
        Rz = adsk.core.Matrix3D.create(); Rz.setToRotation(math.radians(rz), adsk.core.Vector3D.create(0, 0, 1), O); M.transformBy(Rz)
    T = adsk.core.Matrix3D.create(); T.translation = adsk.core.Vector3D.create(cm(c[0]), cm(c[1]), cm(c[2]))
    M.transformBy(T)
    occ.transform = M
    report.append((name, round((body.physicalProperties.mass + (mesh_comp.physicalProperties.mass if mesh_comp else 0)) * 1000, 1), m))

app.activeViewport.fit()

# exports: one STEP + STL per distinct part (one arm, not six), then the whole drone as Fusion archive, STEP and STL
import os, re
OUT = os.path.join(HERE, '..', 'Widow', 'Full Drone')
PARTS_DIR = os.path.join(HERE, '..', 'Widow', 'Reference Parts')
os.makedirs(PARTS_DIR, exist_ok=True)
for f in os.listdir(PARTS_DIR):  # generated folder: clear last run so renamed parts don't linger
    if f.endswith(('.stl', '.step')) and f != os.path.basename(PROP_STL):   # the prop file is a source, not output
        os.remove(os.path.join(PARTS_DIR, f))
em = des.exportManager
done = set()
for occ, p in zip(root.occurrences, PARTS):
    base = re.sub(r'\s+#\S+$', '', p[0])            # 'frame arm #3' -> 'frame arm'; model numbers like M2208 stay
    if base in done or p[1] in ('step', 'prop'):  # printed parts live in Printed Parts; the prop file is the source itself
        continue
    done.add(base)
    fn = os.path.join(PARTS_DIR, re.sub(r'[^a-z0-9]+', '-', base.lower()).strip('-'))
    em.execute(em.createSTLExportOptions(occ, fn + '.stl'))
    em.execute(em.createSTEPExportOptions(fn + '.step', occ.component))
o = em.createSTLExportOptions(root, os.path.join(OUT, 'widow-full-drone.stl')); o.isOneFilePerBody = False; em.execute(o)
em.execute(em.createSTEPExportOptions(os.path.join(OUT, 'widow-full-drone.step'), root))
em.execute(em.createFusionArchiveExportOptions(os.path.join(OUT, 'widow-full-drone.f3d'), root))
vp = app.activeViewport; cam = vp.camera
cam.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation; cam.isFitView = True; vp.camera = cam
vp.saveAsImageFile(os.path.join(OUT, 'widow-full-drone-preview.png'), 1600, 1000)

# self-check: every part weighs what the table says and sits where the table says
# (guards skipped for position: an open arc's centroid is not its placement point)
pp = root.getPhysicalProperties(adsk.fusion.CalculationAccuracy.HighCalculationAccuracy)
tm = sum(p[4] for p in PARTS)
bad_mass = [r for r in report if abs(r[1]-r[2]) > 0.02*r[2] + 0.1]
bad_pos = []
for occ, p in zip(root.occurrences, PARTS):
    if p[1] in ('step', 'arm', 'poly'):  # centroid is not the placement point for these shapes
        continue
    c = occ.bRepBodies.item(0).physicalProperties.centerOfMass
    if max(abs(c.x*10-p[3][0]), abs(c.y*10-p[3][1]), abs(c.z*10-p[3][2])) > 0.5:
        bad_pos.append((p[0], [round(c.x*10, 1), round(c.y*10, 1), round(c.z*10, 1)], p[3]))
bodies = adsk.core.ObjectCollection.create()
for occ in root.occurrences:
    bodies.add(occ.bRepBodies.item(0))
ii = des.createInterferenceInput(bodies); ii.areCoincidentFacesIncluded = False
cname = lambda e: (e.assemblyContext.component if e.assemblyContext else e.parentComponent).name.split(' [')[0]
clashes = sorted({tuple(sorted((cname(r.entityOne), cname(r.entityTwo)))) for r in des.analyzeInterference(ii)})
def inside(pt, poly):  # ray casting
    x, y = pt; c = False
    for i in range(len(poly)):
        (x1, y1), (x2, y2) = poly[i], poly[i - 1]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            c = not c
    return c
star = star_points()
off = lambda parts: sorted({p[0] for p in parts for sx in (-1, 1) for sy in (-1, 1)
                            if not inside((p[3][0] + sx*p[2][0]/2, p[3][1] + sy*p[2][1]/2), star)})
# parts standing on the bottom plate (between the plates) or hanging from its underside
on_bottom = [p for p in PARTS if p[1] == 'box' and (abs(p[3][2] - p[2][2]/2 - PLATE_T) < 3.5 or abs(p[3][2] + p[2][2]/2) < 3.5)]
on_battery_plate = [('landing leg mount plate', 'box', (34.6, 2*LEG_Y + 36, 6), (0, 0, BP_Z - 3), 0)]
off_bp = sorted({p[0] for p in on_battery_plate for sx in (-1, 1) for sy in (-1, 1)
                 if not inside((p[3][0] + sx*p[2][0]/2, p[3][1] + sy*p[2][1]/2), bp_pts)})
{'camera_view_blocked_pct': CAM_BLOCKED_PCT, 'off_bottom_plate': off(on_bottom), 'off_battery_plate': off_bp, 'clashes': clashes, 'total_g': round(pp.mass*1000, 1), 'table_total_g': tm,
 'cg_mm': [round(pp.centerOfMass.x*10, 1), round(pp.centerOfMass.y*10, 1), round(pp.centerOfMass.z*10, 1)],
 'mass_mismatches': bad_mass, 'position_mismatches': bad_pos[:5], 'n_bad_pos': len(bad_pos), 'parts': len(PARTS), 'part_files': len(done)}
