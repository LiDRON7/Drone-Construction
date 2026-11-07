# Widow (Storm Drone 6 V3, hexa +, PX4 7001) layout model. Runs inside Fusion via the
# Fusion360MCP add-in: python3 run_in_fusion.py execute_code -code full_drone.py
# Saves the assembly into ../Widow/Full Drone/ and one model per bought part into ../Widow/Reference Parts/
# (HERE is set by run_in_fusion.py).
# Frame: x forward, y left, z up, mm. Origin = frame centre, bottom face of the bottom plate.
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

R_MOTOR   = 216.0   # motor centre radius (user-measured neighbour spacing; hexagon side = radius)
TOP_D     = 150.0   # carbon top plate, scaled from photo 4 (EST +-15 mm): tip-to-tip size of the star below
STAR_R_TIP, STAR_R_IN, STAR_LOBE_W = 77.0, 45.0, 34.0   # top plate is a six-lobed star, a lobe over each arm (EST from photos)
PLATE_T   = 2.0
GAP       = 30.0    # EST space between the plates = arm height at the root (the Pi sits in here)
ARM_W     = 24.0    # truss arm width (photos)
ARM_R0    = 55.0    # EST where the arm root starts, measured from the frame centre
H_TIP     = 14.0    # EST arm height at the motor; the arm underside tapers up from root to tip
ARM_WALL, ARM_FLOOR = 3.0, 2.0   # open-top channel: two side walls + a floor, ESC rides inside
TOP_Z     = PLATE_T + GAP        # underside of the top plate = top of the arms
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
        a = math.radians(60 * i); d = math.asin(STAR_LOBE_W / 2 / STAR_R_TIP)
        v = a - math.radians(30)
        pts += [(STAR_R_IN*math.cos(v), STAR_R_IN*math.sin(v)),
                (STAR_R_TIP*math.cos(a-d), STAR_R_TIP*math.sin(a-d)), (STAR_R_TIP*math.cos(a+d), STAR_R_TIP*math.sin(a+d))]
    return pts
add('frame bottom plate', 'poly', (star_points(), PLATE_T), (0, 0, PLATE_T/2), 60)            # same star as the top plate (user)
add('frame top plate', 'poly', (star_points(), PLATE_T), (0, 0, TOP_Z + PLATE_T/2), 60)
for i in range(6):
    a = 60.0 * i                      # motor 1 forward (hexa +)
    t = math.radians(a); cx, cy = math.cos(t), math.sin(t)
    rm = ARM_R0 + ARM_L/2
    add(f'frame arm #{i+1}', 'arm', (ARM_L, ARM_W, GAP, H_TIP), (rm*cx, rm*cy, PLATE_T + GAP/2), 30, rz=a)
    mz = TOP_Z + (2.0 if WITH_GUARDS else 0.0)   # motor base; the guard hub (2 mm) sits between arm and motor
    add(f'propulsion motor Storm M2208 #{i+1}', 'cyl', (27.5, 20.0), (R_MOTOR*cx, R_MOTOR*cy, mz + 10), 44)        # EST 44 g; prop seat 20 mm above the base (measured)
    add(f'propulsion propeller 8x4.5 #{i+1}', 'prop', (19.6, 7.0), (R_MOTOR*cx, R_MOTOR*cy, mz + 20 + 3.5), 8, rz=a)  # real blade mesh + 8 g hub body (EST weight)
    add(f'propulsion ESC F35A #{i+1}', 'box', (30, 16, 5), (140*cx, 140*cy, arm_floor_z(140) + 2.5 + 1.5), 7.5, rz=a)  # +1.5: flat ESC on a sloped floor  # runs inside the arm
    if WITH_GUARDS:
        add(f'printed propeller guard #{i+1}', 'step', (GUARD_STEP,), (R_MOTOR*cx, R_MOTOR*cy, TOP_Z), 33.5, rz=a)   # the real printed guard, hub on the arm pad
# landing gear: two printed legs (Printed Parts/Landing Leg, landing_leg.py) bolted under the bottom plate, stock carbon
# skid tubes through their feet. The leg file is drawn for the left side in drone coordinates (profile in x-y = drone
# y-z), so rx=90 stands it up and rz=+-90 turns it to each side.
LEG_STEP = os.path.join(HERE, '..', 'Widow', 'Printed Parts', 'Landing Leg', 'landing-leg.step')
SKID_Y, SKID_Z = 125.0, -85.0       # must match FOOT_U, FOOT_V in landing_leg.py
LEG_G = 37.0                        # g per printed leg, as sliced
for sy in (1, -1):
    add(f'printed landing leg #{"left" if sy > 0 else "right"}', 'step', (LEG_STEP,), (0, 0, 0), LEG_G, rz=90*sy, rx=90)     # printed PETG, as sliced
    add(f'frame landing skid tube #{"left" if sy > 0 else "right"}', 'cyl', (10, 220), (0, sy*SKID_Y, SKID_Z), 15, ry=90)        # stock carbon tube, EST 10 mm x 220 mm

# --- electronics, as Widow is built: everything but the Pi and battery on the top plate
FC_Z = TOP_Z + PLATE_T
add('flight controller foam mount', 'box', (75, 46, 6), (0, 0, FC_Z + 3), 8)
add('flight controller Pixhawk 6X Mini baseboard', 'box', (72.8, 43.4, 14.2), (0, 0, FC_Z + 6 + 7.1), 26.5)
add('flight controller Pixhawk 6X Mini FMU', 'box', (38.8, 31.8, 16.8), (0, 0, FC_Z + 6 + 14.2 + 8.4), 23)
# printed GPS mast (Printed Parts/GPS Mast, gps_mast.py): 4 x M3 to the top plate, module in the cradle 63 mm up
MAST_X, MAST_STEP = -55.0, os.path.join(HERE, '..', 'Widow', 'Printed Parts', 'GPS Mast', 'gps-mast.step')
GPS_Z = FC_Z + 63.0                   # cradle floor top; must match CRADLE_Z + FLOOR in gps_mast.py
add('printed GPS mast', 'step', (MAST_STEP,), (MAST_X, 0, FC_Z), 46)                 # 46 g as sliced
add('gps H-RTK F9P Helical module', 'box', (52.7, 34.8, 12.9), (MAST_X, 0, GPS_Z + 6.45), 30)
add('gps H-RTK F9P Helical antenna', 'cyl', (27.5, 59), (MAST_X, 0, GPS_Z + 12.9 + 29.5), 19)      # 49 g total, split EST
add('radio telemetry SiK V3 915MHz', 'box', (28, 53, 10.7), (52, 0, FC_Z + 5.35), 15)              # front of the top plate; EST weight
add('radio RC receiver RadioLink R9DS', 'box', (43, 24, 15), (-20, -44, FC_Z + 7.5), 9)
add('power distribution board', 'box', (45, 35, 6), (-18, 44, FC_Z + 3), 15)          # feeds the ESCs; EST size and weight; clears the mast foot
# between the plates: Pi on 3 mm standoffs on the bottom plate, step-down beside it, wiring above
add('computer Raspberry Pi 5 with cooler', 'box', (85, 56, 20), (0, 0, PLATE_T + 3 + 10), 66)       # 46 g + EST 20 g cooler
# both moved off the centreline so the landing-leg nuts (x = 0, 33 and 41 mm out) have room
add('power step-down 5V', 'box', (20, 16, 8), (14, 38, PLATE_T + 4), 15)                # EST size
add('power module Holybro PM02D', 'box', (20, 16, 10), (14, -38, PLATE_T + 5), 20)              # EST size
add('misc wiring bundle (placeholder)', 'box', (60, 60, 5), (0, 0, TOP_Z - 3.5), 50)                  # EST
# under the bottom plate: camera below the front arm facing forward, battery on a strap tray behind it
OAK_X = 78.0
add('camera Luxonis OAK-D Pro', 'box', (29.5, 97, 22.9), (OAK_X, 0, -3 - 22.9/2), 91)          # 3 mm bracket under the plate
add('misc battery tray (placeholder)', 'box', (160, 55, 3), (0, 0, -1.5), 25)                        # EST
add('power battery 3S 5500mAh LiPo', 'box', (150, 48, 30), (BATT_X, 0, -18), 420)                  # 1290 - 870 g (HeliPal AUW); x is the balance trim

# self-check: the Pi must fit between the plates and inside the arm roots
assert math.hypot(85/2, 56/2) < ARM_R0, 'Pi corners hit the arm roots'
assert 3 + 20 <= GAP - 5, 'Pi + standoffs do not fit under the wiring'
assert BATT_X + 150/2 < OAK_X - 29.5/2, 'battery runs into the OAK-D'

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
    elif shape == 'step':  # a real part imported from its STEP (guard, landing leg); used in its own coordinates
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
on_top = [p for p in PARTS if p[1] == 'box' and abs(p[3][2] - p[2][2]/2 - FC_Z) < 0.5]   # boxes resting on the top plate
on_top.append(('GPS mast foot', 'box', (26, 60, 5), (MAST_X, 0, FC_Z), 0))
off = lambda parts: sorted({p[0] for p in parts for sx in (-1, 1) for sy in (-1, 1)
                            if not inside((p[3][0] + sx*p[2][0]/2, p[3][1] + sy*p[2][1]/2), star)})
# parts standing on the bottom plate (between the plates) or hanging from its underside
on_bottom = [p for p in PARTS if p[1] == 'box' and (abs(p[3][2] - p[2][2]/2 - PLATE_T) < 3.5 or abs(p[3][2] + p[2][2]/2) < 3.5)]
on_bottom += [(f'landing leg block #{"left" if sy > 0 else "right"}', 'box', (24, 29, 12), (0, sy*43.5, -6), 0) for sy in (1, -1)]
{'off_top_plate': off(on_top), 'off_bottom_plate': off(on_bottom), 'clashes': clashes, 'total_g': round(pp.mass*1000, 1), 'table_total_g': tm,
 'cg_mm': [round(pp.centerOfMass.x*10, 1), round(pp.centerOfMass.y*10, 1), round(pp.centerOfMass.z*10, 1)],
 'mass_mismatches': bad_mass, 'position_mismatches': bad_pos[:5], 'n_bad_pos': len(bad_pos), 'parts': len(PARTS), 'part_files': len(done)}
