# Huntsman (Tarot 680 PRO) put together: the online Tarot model, the printed payload carrier + clamps, and every
# bought part as a block. Runs inside Fusion via the add-in (run huntsman_payload.py first):
#   python3 run_in_fusion.py execute_code -code huntsman_full_drone.py
# Reports clashes between our parts and the frame, the centre of mass (from the weight table below, not Fusion's
# materials), PX4 sensor offsets, how far the LiDAR can see sideways under the drone, and what the camera sees.
# Flight coords: x forward (along the payload rods, the gap between two arms: hexa X), y left, z up, mm, origin =
# centre of the top plate's underside (as in the Tarot model). "EST" = not measured yet.
import adsk.core, adsk.fusion, math, os
cm = lambda mm: mm / 10.0
HUNT = os.path.join(HERE, '..', 'Huntsman')
TAROT_STEP = os.path.join(HUNT, 'Reference Parts', 'frame-tarot-680-pro.step')
CARRIER = os.path.join(HUNT, 'Printed Parts', 'Payload Carrier')
MAST_STEP = os.path.join(HERE, '..', 'Widow', 'Printed Parts', 'GPS Mast', 'gps-mast.step')   # same mast as Widow
GROUND_Z = -245.3                  # bottom of the skid feet in the model
TOP = 1.6                          # top face of the top plate
# must match huntsman_payload.py
Z_PT, ROD_Y, CLAMP_X, LID_OPT, LID_D, LID_H = -48.0, 30.0, (-112.0, -40.0, 40.0), 37.8, 103.3, 71.7
LID_AX_Z, CAM_C, CAM_LENS, TILT = -105.65, (91.24, 21.46), (102.61, 24.51), 15.0   # camera points are print coords
BAT_X0, BAT_X1 = -24.0, -68.5
CARRIER_G, CLAMP_G = 163.1, 6.8     # PETG as sliced: payload carrier, each rod clamp
HFOV, VFOV = 72.0, 56.0            # OAK-D Pro RGB, deg
# name: (shape, size, centre, grams). Sizes x, y, z. 'cyl_x' = (diameter, length) along x.
PARTS = {
    'lidar Velodyne Puck LITE':          ('cyl_x', (LID_D, LID_H), (LID_H / 2, 0, LID_AX_Z), 590),       # datasheet
    'camera Luxonis OAK-D Pro':          ('cam', (14.28, 96.96, 29.45), (23.55 - 14.28) / 2 - 0.2, 77.69),   # body; 89.69 g measured in all
    'camera Luxonis OAK-D Pro heatsink': ('cam', (8.87, 79, 25), (23.55 - 8.87) / 2 - 0.2 - 14.28, 12),     # fins stop ~9 mm from each end (as Widow)
    'power battery 6S 5200 mAh':         ('box', (43.5, 133, 49.5), ((BAT_X0 + BAT_X1 + 1) / 2, 0, Z_PT - 4 - 24.75), 658),  # listing
    'lidar Velodyne interface box':      ('box', (53, 75, 28), (-102.5, 0, Z_PT - 4 - 14), 80),        # EST size and weight
    'power step-down 12V (LiDAR)':       ('box', (45, 26, 14), (-97.5, 0, Z_PT + 7), 25),              # EST, on top of the carrier
    'flight controller foam mount':      ('box', (75, 46, 6), (0, 0, TOP + 3), 8),
    'flight controller Pixhawk 6X Mini': ('box', (72.8, 43.4, 31), (0, 0, TOP + 6 + 15.5), 54.49),    # measured, baseboard + FMU
    'computer Raspberry Pi 5 with cooler': ('box', (85, 56, 20), (0, -60, TOP + 10), 99.92),          # measured weight, EST size
    'radio telemetry SiK V3 915MHz':     ('box', (53, 28, 10.7), (35, 55, TOP + 5.35), 15),
    'radio RC receiver RadioLink R9DS':  ('box', (24, 43, 15), (55, -42, TOP + 7.5), 12.53),
    'power step-down 5V (Pi)':           ('box', (20, 16, 8), (-55, -55, TOP + 4), 15),
    'power module Holybro PM02D':        ('box', (20, 16, 10), (-55, 0, TOP + 5), 20),
    'gps H-RTK F9P Helical module':      ('box', (53.86, 35.3, 12.9), (-55, 40, TOP + 48 + 6.45), 38.25),
    'gps H-RTK F9P Helical antenna':     ('cyl_z', (27.5, 59), (-55, 40, TOP + 48 + 12.9 + 29.5), 19),
}
# not modelled as blocks (the Tarot model has them, or they spread evenly): weight only, centred on the frame
SPREAD = {'printed propeller guards + arm clamps x6': (6 * (46.5 + 5.5), 15), 'frame Tarot 680 PRO with landing gear (EST)': (790, -40), 'motors Tarot 4108 380KV x6': (540, 10),
          'ESC Aikon AK8 80A x6 (EST)': (300, -10), 'props 1355 x6 (EST)': (180, 25), 'wiring (EST)': (120, -20),
          'printed GPS mast': (38.1, TOP + 25)}

for d in [d for d in app.documents if d.name.startswith('Huntsman full drone')]:
    d.close(False)
doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType); doc.name = 'Huntsman full drone'
des = adsk.fusion.Design.cast(app.activeProduct); des.designType = adsk.fusion.DesignTypes.DirectDesignType
root = des.rootComponent
P3 = lambda x, y, z: adsk.core.Point3D.create(cm(x), cm(y), cm(z))
V3 = lambda x, y, z: adsk.core.Vector3D.create(x, y, z)
tbm = adsk.fusion.TemporaryBRepManager.get()
cam_at = lambda k: (CAM_C[0] + k * math.cos(math.radians(TILT)), 0, Z_PT - CAM_C[1] - k * math.sin(math.radians(TILT)))

def new_occ(name, M=None):
    occ = root.occurrences.addNewComponent(M or adsk.core.Matrix3D.create()); occ.component.name = name; return occ

def import_step(path, name, M):
    occ = new_occ(name)
    app.importManager.importToTarget(app.importManager.createSTEPImportOptions(path), occ.component)
    occ.transform = M
    return occ

def rot(axis, deg, t=(0, 0, 0)):
    M = adsk.core.Matrix3D.create(); M.setToRotation(math.radians(deg), V3(*axis), P3(0, 0, 0))
    T = adsk.core.Matrix3D.create(); T.translation = V3(cm(t[0]), cm(t[1]), cm(t[2])); M.transformBy(T); return M

# frame: the model has the rods along its y; turn it so they run along our x (forward)
frame = import_step(TAROT_STEP, 'Tarot 680 PRO frame', rot((0, 0, 1), -90))
for o in [o for o in frame.component.allOccurrences if 'payload-deck' in o.name]:
    o.deleteMe()                       # the stock battery deck and its clips: the carrier replaces them
frame.transform = rot((0, 0, 1), -90)  # again: deleting resets it
rod = [o for o in root.allOccurrences if 'small-tube v3:3' in o.name][0].boundingBox
assert (rod.maxPoint.x - rod.minPoint.x) * 10 > 250, 'frame not turned: payload rods should run fore-aft'
carrier = import_step(os.path.join(CARRIER, 'payload-carrier.step'), 'printed payload carrier', rot((1, 0, 0), 180, (0, 0, Z_PT)))
clamps = []
for x in CLAMP_X:
    for sy in (-1, 1):
        M = rot((0, 0, 1), 0 if sy > 0 else 180, (x, sy * ROD_Y, Z_PT))
        clamps.append(import_step(os.path.join(CARRIER, 'rod-clamp.step'), f'printed rod clamp', M))
mast = import_step(MAST_STEP, 'printed GPS mast', rot((0, 0, 1), 0, (-55, 40, TOP)))
ours = [carrier, mast] + clamps
GUARD = os.path.join(HUNT, 'Printed Parts', 'Propeller Guard')
ARM_ANGS, ARM_R, RING_R, RING_Z, RING_HALF = (30, 90, 150, -150, -90, -30), 343.0, 173.75, (6.0, 31.0), 95.0   # huntsman_propeller_guard.py
for a in ARM_ANGS:                     # guard local coords: motor axis at 0, x outward, z = motor mount top
    M = rot((0, 0, 1), a, (ARM_R * math.cos(math.radians(a)), ARM_R * math.sin(math.radians(a)), 0))
    ours.append(import_step(os.path.join(GUARD, 'propeller-guard.step'), 'printed propeller guard', M))
    ours.append(import_step(os.path.join(GUARD, 'arm-clamp.step'), 'printed arm clamp', M))
for name, (shape, size, c, g) in PARTS.items():
    if shape == 'box':
        b = tbm.createBox(adsk.core.OrientedBoundingBox3D.create(P3(*c), V3(1, 0, 0), V3(0, 1, 0), cm(size[0]), cm(size[1]), cm(size[2])))
    elif shape == 'cam':                # c = mm forward along the camera axis from the pocket centre
        t = math.radians(TILT); c = cam_at(c); PARTS[name] = (shape, size, c, g)
        b = tbm.createBox(adsk.core.OrientedBoundingBox3D.create(P3(*c), V3(math.cos(t), 0, -math.sin(t)), V3(0, 1, 0), cm(size[0]), cm(size[1]), cm(size[2])))
    elif shape == 'cyl_x':
        b = tbm.createCylinderOrCone(P3(c[0] - size[1] / 2, c[1], c[2]), cm(size[0] / 2), P3(c[0] + size[1] / 2, c[1], c[2]), cm(size[0] / 2))
    else:
        b = tbm.createCylinderOrCone(P3(c[0], c[1], c[2] - size[1] / 2), cm(size[0] / 2), P3(c[0], c[1], c[2] + size[1] / 2), cm(size[0] / 2))
    occ = new_occ(name); occ.component.bRepBodies.add(b); ours.append(occ)

# clashes: only pairs that involve one of our parts (the frame model's own screws sit in their holes)
bodies = adsk.core.ObjectCollection.create()
for o in root.allOccurrences:          # every level, as proxies in the root's context
    for bd in o.bRepBodies:
        bodies.add(bd)
ii = des.createInterferenceInput(bodies); ii.areCoincidentFacesIncluded = False
def path(e):                           # 'Tarot 680 PRO frame:1+...+part:1' -> ['Tarot 680 PRO frame', ..., 'part']
    occ = e.assemblyContext
    return [q.split(':')[0] for q in occ.fullPathName.split('+')] if occ else [e.parentComponent.name]
OURS = {o.component.name for o in ours}
clashes = set()
for r in des.analyzeInterference(ii):
    a, b = path(r.entityOne), path(r.entityTwo)
    mine = lambda q: next((n for n in q if n in OURS), None)
    if not (mine(a) or mine(b)):
        continue                       # the frame model's own screws in their holes
    clashes.add(tuple(sorted((mine(a) or a[-1], mine(b) or b[-1]))))

# centre of mass from the table
items = [(n, g, c) for n, (s, z_, c, g) in PARTS.items()]
cc = carrier.physicalProperties.centerOfMass
items += [('printed payload carrier', CARRIER_G, (cc.x * 10, cc.y * 10, cc.z * 10)), ('printed rod clamps x6', 6 * CLAMP_G, (-37, 0, Z_PT + 10))]
items += [(n, g, (0, 0, z)) for n, (g, z) in SPREAD.items() if n != 'printed GPS mast'] + [('printed GPS mast', 38.1, (-55, 40, TOP + 25))]
M_tot = sum(g for _, g, _ in items)
cg = [sum(g * c[i] for _, g, c in items) / M_tot for i in range(3)]

# AABBs of everything (flight coords) for the view checks
def aabb(o):
    bb = o.boundingBox
    return [bb.minPoint.x * 10, bb.minPoint.y * 10, bb.minPoint.z * 10, bb.maxPoint.x * 10, bb.maxPoint.y * 10, bb.maxPoint.z * 10]
leaves = [o for o in root.allOccurrences if o.bRepBodies.count and o.fullPathName.startswith('Tarot 680 PRO frame')]
boxes = [(o.name, aabb(o)) for o in leaves if 'propeller' not in o.name]
props = [aabb(o) for o in leaves if 'propeller' in o.name]
boxes += [(o.component.name, aabb(o)) for o in ours if 'guard' not in o.component.name and 'arm clamp' not in o.component.name and o.component.name not in ('lidar Velodyne Puck LITE', 'printed payload carrier', 'camera Luxonis OAK-D Pro', 'camera Luxonis OAK-D Pro heatsink')]
def hit(o, d, box, tmax=30000.0):
    t0, t1 = 0.0, tmax
    for i in range(3):
        if abs(d[i]) < 1e-12:
            if not box[i] <= o[i] <= box[i + 3]:
                return None
            continue
        a, b = (box[i] - o[i]) / d[i], (box[i + 3] - o[i]) / d[i]
        t0, t1 = max(t0, min(a, b)), min(t1, max(a, b))
        if t0 > t1:
            return None
    return t0
def blocked(o, d):
    for n, bx in boxes:
        if hit(o, d, bx) is not None:
            return n
    for a in ARM_ANGS:              # guard rings: vertical cylinder bands, outer RING_HALF deg only
        cx, cy = ARM_R * math.cos(math.radians(a)), ARM_R * math.sin(math.radians(a))
        px, py = o[0] - cx, o[1] - cy
        A = d[0] ** 2 + d[1] ** 2; Bq = 2 * (px * d[0] + py * d[1]); C = px * px + py * py - RING_R ** 2
        disc = Bq * Bq - 4 * A * C
        if A > 1e-12 and disc >= 0:
            for t in ((-Bq - math.sqrt(disc)) / (2 * A), (-Bq + math.sqrt(disc)) / (2 * A)):
                if t > 0:
                    hx, hy, hz = o[0] + t * d[0], o[1] + t * d[1], o[2] + t * d[2]
                    rel = (math.degrees(math.atan2(hy - cy, hx - cx)) - a + 180) % 360 - 180
                    if RING_Z[0] <= hz <= RING_Z[1] and abs(rel) <= RING_HALF:
                        return 'propeller guard ring'
    for p in props:                 # props: discs at their AABB's mid height
        zc = (p[2] + p[5]) / 2; cx, cy, r = (p[0] + p[3]) / 2, (p[1] + p[4]) / 2, (p[3] - p[0]) / 2
        if abs(d[2]) > 1e-9:
            t = (zc - o[2]) / d[2]
            if t > 0 and math.hypot(o[0] + t * d[0] - cx, o[1] + t * d[1] - cy) <= r:
                return 'propeller'
    return None
# LiDAR: scan plane x = optical centre, beams +-15 deg along x; angle th from straight down, + to the left
O = (LID_OPT, 0.0, LID_AX_Z)
def clear(th):
    return all(blocked(O, (math.sin(math.radians(f)), math.sin(math.radians(th)) * math.cos(math.radians(f)),
                           -math.cos(math.radians(th)) * math.cos(math.radians(f)))) is None for f in (-15, -7.5, 0, 7.5, 15))
left = next(th for th in range(0, 181) if not clear(th)) - 1
right = next(th for th in range(0, 181) if not clear(-th)) - 1
h3 = 3000.0
strip = (h3 * math.tan(math.radians(min(left, right))) * 2 / 1000, 2 * h3 * math.tan(math.radians(15)) / 1000)
# camera: rays over its view
L = (CAM_LENS[0], 0.0, Z_PT - CAM_LENS[1])
seen = {}
n_rays = 0
for a in range(-int(HFOV / 2), int(HFOV / 2) + 1, 4):
    for e in range(-int(VFOV / 2), int(VFOV / 2) + 1, 4):
        el = math.radians(-TILT + e); az = math.radians(a); n_rays += 1
        k = blocked(L, (math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)))
        if k:
            seen[k] = seen.get(k, 0) + 1
# PX4 offsets (FRD: x forward, y right, z down) from the centre of mass
fmu = (0, 0, TOP + 6 + 14.2 + 8.4); ant = PARTS['gps H-RTK F9P Helical antenna'][2]
frd = lambda p: [round((p[0] - cg[0]) / 1000, 3), round(-(p[1] - cg[1]) / 1000, 3), round(-(p[2] - cg[2]) / 1000, 3)]

OUT = os.path.join(HUNT, 'Full Drone'); os.makedirs(OUT, exist_ok=True)
vp = app.activeViewport; cam = vp.camera
cam.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation; cam.isFitView = True; vp.camera = cam
vp.saveAsImageFile(os.path.join(OUT, 'huntsman-full-drone-preview.png'), 1600, 1000)
cam.viewOrientation = adsk.core.ViewOrientations.FrontViewOrientation; cam.isFitView = True; vp.camera = cam
vp.saveAsImageFile(os.path.join(OUT, 'huntsman-side-view.png'), 1600, 1000)
em = des.exportManager
em.execute(em.createFusionArchiveExportOptions(os.path.join(OUT, 'huntsman-full-drone.f3d'), root))
res = {'clashes': sorted(clashes), 'total_g': round(M_tot, 1), 'cg_mm': [round(v, 1) for v in cg],
       'lidar_clear_deg_left_right': [left, right], 'lidar_ground_strip_at_3m_m': [round(v, 2) for v in strip],
       'lidar_ground_clearance_mm': round(LID_AX_Z - LID_D / 2 - GROUND_Z, 1),
       'camera_rays_blocked': seen, 'camera_rays': n_rays,
       'EKF2_IMU_POS': frd(fmu), 'EKF2_GPS_POS': frd(ant), 'lidar_optical_from_cg_m': frd(O), 'camera_lens_from_cg_m': frd(L)}
import json
open(os.path.join(OUT, 'huntsman-checks.json'), 'w').write(json.dumps(res, indent=1))
res
