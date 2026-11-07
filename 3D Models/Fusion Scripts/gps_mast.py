# Widow GPS mast for the Holybro H-RTK F9P Helical. Runs inside Fusion via the add-in:
#   python3 run_in_fusion.py execute_code -code gps_mast.py
# Saves STEP/STL/preview into ../Widow/Printed Parts/GPS Mast/ (HERE is set by run_in_fusion.py).
#
# Foot (4 x M3 to the top plate) -> flared hollow column (GPS cable runs inside, exits at the foot toward the
# Pixhawk) -> 45 deg loft -> cradle pocket for the module, held by a strap through two slots.
# Printed upright, foot on the bed: the 45 deg loft needs no supports.
# The mast is sized by its sway frequency, not just strength: a printed mast fails by fatigue when it sways
# at the motors' frequency, so it must sway faster than the motors ever spin. All mm, z = height above the
# top plate. "EST" = not measured yet.
FOOT_X, FOOT_Y, FOOT_T = 26.0, 60.0, 5.0
BOLTS = [(-8, -24), (8, -24), (-8, 24), (8, 24)]   # EST: M3, drill 3.2 mm holes in the top plate if none line up
BOLT_D = 3.4
FLARE_R, FLARE_H = 17.0, 8.0          # base flare from the bed up to FOOT_T+FLARE_H: no sharp corner, no overhang
COL_OD, COL_ID = 30.0, 22.0           # column; the bore is the cable channel (fits the GH connector)
COL_TOP = 45.0
CRADLE_Z = 60.0                       # loft from the column to the cradle between COL_TOP and here
MOD_L, MOD_W, MOD_H = 52.7, 34.8, 12.9   # H-RTK F9P Helical module (Holybro spec); long side along drone x
FIT, WALL, FLOOR, POCKET = 0.35, 2.8, 3.0, 8.0
STRAP_W, STRAP_T, STRAP_X = 12.0, 3.0, 20.0   # strap slots through the long walls at x = +-STRAP_X: between the antenna and the module ends
ANT_D = 27.5                                  # helical antenna standing on the middle of the module
CABLE_SLOT_W, CABLE_SLOT_H = 10.0, 12.0       # cable exit at the foot, facing +x (toward the Pixhawk)

# self-checks
GPS_G, ANT_G, CRADLE_G = 30.0, 19.0, 8.0     # module + antenna = 49 g (Holybro); cradle EST
E_PETG_UPRIGHT = 1.5e9                        # Pa, printed upright (layers across the column): low end, on purpose
KV, V_MAX, LOAD = 1260, 12.6, 0.8             # motor KV, full 3S, loaded rpm fraction
G_CRASH, SIGMA_LAYER_MPA = 20.0, 15.0         # 20 g sideways hit; PETG strength across layers, conservative

import adsk.core, adsk.fusion, math, os
cm = lambda mm: mm / 10.0
outer_x, outer_y = MOD_L + 2*FIT + 2*WALL, MOD_W + 2*FIT + 2*WALL
loft_h = CRADLE_Z - COL_TOP
assert (outer_x/2 - COL_OD/2) <= loft_h and (outer_y/2 - COL_OD/2) <= loft_h, 'loft steeper than 45 deg: needs supports'
assert (COL_OD - COL_ID)/2 >= 1.6 and WALL >= 1.6, 'walls under the FDM minimum'
assert STRAP_X - STRAP_W/2 > ANT_D/2 and STRAP_X + STRAP_W/2 < MOD_L/2, 'strap would cross the antenna or miss the module'
assert min(math.hypot(bx, by) for bx, by in BOLTS) - 2.75 > FLARE_R, 'flare covers a bolt head'
m = (GPS_G + ANT_G + CRADLE_G) / 1000
z_cg = ((CRADLE_Z + FLOOR + MOD_H/2)*GPS_G + (CRADLE_Z + FLOOR + MOD_H + 29.5)*ANT_G + (CRADLE_Z + 4)*CRADLE_G) / (GPS_G + ANT_G + CRADLE_G)
I = math.pi * (COL_OD**4 - COL_ID**4) / 64 * 1e-12            # m^4, the slimmest section (flare ignored: conservative)
k = 3 * E_PETG_UPRIGHT * I / (z_cg / 1000)**3
f_mast = math.sqrt(k / m) / (2 * math.pi)
f_motor = KV * V_MAX * LOAD / 60                               # Hz, highest loaded motor speed
assert f_mast >= 1.4 * f_motor, f'mast sways at {f_mast:.0f} Hz, too close to the motors ({f_motor:.0f} Hz)'
M = m * 9.81 * G_CRASH * z_cg                                  # N*mm at the base
S = math.pi * (COL_OD**4 - COL_ID**4) / (32 * COL_OD)
sigma = M / S
assert SIGMA_LAYER_MPA / sigma >= 4, f'base stress {sigma:.1f} MPa too high'

for d in [d for d in app.documents if d.name.startswith('Widow GPS mast')]:
    d.close(False)
doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
doc.name = 'Widow GPS mast'
des = adsk.fusion.Design.cast(app.activeProduct)
des.designType = adsk.fusion.DesignTypes.DirectDesignType
root = des.rootComponent
ext, lofts = root.features.extrudeFeatures, root.features.loftFeatures
NEW, JOIN, CUT = (adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
                  adsk.fusion.FeatureOperations.JoinFeatureOperation,
                  adsk.fusion.FeatureOperations.CutFeatureOperation)
P = lambda x, y: adsk.core.Point3D.create(cm(x), cm(y), 0)

def sketch_at(z):
    if z == 0:
        return root.sketches.add(root.xYConstructionPlane)
    pi = root.constructionPlanes.createInput()
    pi.setByOffset(root.xYConstructionPlane, adsk.core.ValueInput.createByReal(cm(z)))
    return root.sketches.add(root.constructionPlanes.add(pi))

def rect(sk, x0, y0, x1, y1):
    sk.sketchCurves.sketchLines.addTwoPointRectangle(P(x0, y0), P(x1, y1))
    return sk

def circle(sk, r, cx=0, cy=0):
    sk.sketchCurves.sketchCircles.addByCenterRadius(P(cx, cy), cm(r))
    return sk

def extrude(sk, h, op, body=None):
    profs = adsk.core.ObjectCollection.create()
    for p in sk.profiles:
        profs.add(p)
    inp = ext.createInput(profs, op)
    inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(cm(h)))
    if body:
        inp.participantBodies = [body]
    return ext.add(inp)

def loft(sk_a, sk_b, body):
    li = lofts.createInput(JOIN)
    li.loftSections.add(sk_a.profiles.item(0)); li.loftSections.add(sk_b.profiles.item(0))
    li.isSolid = True
    li.participantBodies = [body]
    return lofts.add(li)

body = extrude(rect(sketch_at(0), -FOOT_X/2, -FOOT_Y/2, FOOT_X/2, FOOT_Y/2), FOOT_T, NEW).bodies.item(0)
body.name = 'gps mast'
loft(circle(sketch_at(0), FLARE_R), circle(sketch_at(FOOT_T + FLARE_H), COL_OD/2), body)            # flare, starts on the bed
extrude(circle(sketch_at(FOOT_T + FLARE_H), COL_OD/2), COL_TOP - FOOT_T - FLARE_H, JOIN, body)      # column
loft(circle(sketch_at(COL_TOP), COL_OD/2), rect(sketch_at(CRADLE_Z), -outer_x/2, -outer_y/2, outer_x/2, outer_y/2), body)
extrude(rect(sketch_at(CRADLE_Z), -outer_x/2, -outer_y/2, outer_x/2, outer_y/2), FLOOR + POCKET, JOIN, body)  # cradle
if root.bRepBodies.count > 1:  # lofts/joins can leave separate bodies; merge
    tools = adsk.core.ObjectCollection.create()
    for b in list(root.bRepBodies)[1:]:
        tools.add(b)
    ci = root.features.combineFeatures.createInput(body, tools); ci.operation = JOIN
    root.features.combineFeatures.add(ci)
    body = root.bRepBodies.item(0)

pw, pl = MOD_W + 2*FIT, MOD_L + 2*FIT
extrude(rect(sketch_at(CRADLE_Z + FLOOR), -pl/2, -pw/2, pl/2, pw/2), POCKET + 1, CUT, body)             # module pocket
extrude(circle(sketch_at(FOOT_T), COL_ID/2), CRADLE_Z + FLOOR - FOOT_T + 0.5, CUT, body)              # cable bore up into the pocket
extrude(rect(sketch_at(FOOT_T), 0, -CABLE_SLOT_W/2, FLARE_R + 1, CABLE_SLOT_W/2), CABLE_SLOT_H, CUT, body)  # cable exit
sk = sketch_at(CRADLE_Z + FLOOR + 1)
for sx in (-STRAP_X, STRAP_X):
    rect(sk, sx - STRAP_W/2, -outer_y/2 - 1, sx + STRAP_W/2, outer_y/2 + 1)
extrude(sk, STRAP_T, CUT, body)                                                                     # strap slots
sk = sketch_at(-1)
for bx, by in BOLTS:
    circle(sk, BOLT_D/2, bx, by)
extrude(sk, FOOT_T + 2, CUT, body)                                                                  # bolt holes

assert root.bRepBodies.count == 1 and body.lumps.count == 1, 'mast is not one solid'
vol = body.physicalProperties.volume
bb = body.boundingBox
OUT = os.path.join(HERE, '..', 'Widow', 'Printed Parts', 'GPS Mast')
os.makedirs(OUT, exist_ok=True)
em = des.exportManager
o = em.createSTLExportOptions(body, os.path.join(OUT, 'gps-mast.stl'))
o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
em.execute(o)
em.execute(em.createSTEPExportOptions(os.path.join(OUT, 'gps-mast.step'), root))
vp = app.activeViewport
cam = vp.camera; cam.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation; cam.isFitView = True; vp.camera = cam
vp.saveAsImageFile(os.path.join(OUT, 'gps-mast-preview.png'), 1200, 900)
{'volume_cm3': round(vol, 2), 'mass_g_PETG_solid': round(vol * 1.27, 1),
 'bbox_mm': [round((bb.maxPoint.x - bb.minPoint.x)*10, 1), round((bb.maxPoint.y - bb.minPoint.y)*10, 1), round((bb.maxPoint.z - bb.minPoint.z)*10, 1)],
 'sway_Hz': round(f_mast), 'motor_max_Hz': round(f_motor), 'base_stress_MPa_20g': round(sigma, 2), 'gps_cg_height_mm': round(z_cg, 1)}
