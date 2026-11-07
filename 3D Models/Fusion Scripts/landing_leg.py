# Widow landing leg ("feet"). Runs inside Fusion via the add-in:
#   python3 run_in_fusion.py execute_code -code landing_leg.py
# Saves STEP/STL/preview into ../Widow/Printed Parts/Landing Leg/ (HERE is set by run_in_fusion.py). Print two.
# One leg per side, mounted close in beside the battery, where the star-shaped bottom plate is solid
# (the middle of each side is a gap further out). Bolted from below, nuts between the plates.
#
# One flat profile, extruded LEG_T thick, so it prints lying on the bed with every layer parallel to the
# plane the leg bends in (layers never get pulled apart on landing). Local coords = drone coords for the
# left leg: profile u = sideways (y), v = up (z), extrusion along x. The assembly turns it into place.
# Top block bolts under the bottom plate with 2 x M3; the foot boss takes the stock carbon skid tube.
# All mm. "EST" = guessed from photos, measure before printing.
LEG_T      = 24.0    # extrusion thickness (front-to-back on the drone)
BLOCK_U    = (29.0, 58.0)   # top block, distance from the centreline: starts just outside the battery tray (27.5)
BLOCK_H    = 12.0
BOLT_U     = (33.0, 41.0)   # M3 bolts, EST: inside the star's solid centre (edge at 45 here); drill if no holes
ROOT_U, ROOT_V = 51.0, -6.0   # strut root starts inside the block so its corners don't stick out as a notch
BOLT_D, CB_D, CB_DEPTH = 3.4, 6.5, 6.0   # through hole, counterbore for the head from below, its depth
FOOT_U, FOOT_V = 125.0, -85.0           # skid centre: 250 mm stance, ground 90 mm under the plate
TUBE_D     = 10.2    # EST stock skid tube 10 mm + fit; measure the tube
BOSS_R     = 11.0
SET_D      = 2.8     # M3 self-tapping set screw into the tube from the top of the boss
W_ROOT, W_MID, W_FOOT = 16.0, 10.0, 14.0   # strut width in the profile: wide at both joints, slim in the middle

# strength self-check (hand calc, PETG printed flat): hard landing at G_LAND on one side's share
AUW_KG, G_LAND, SIGMA_PETG_MPA, SF_MIN, N_LEGS = 1.91, 8.0, 40.0, 4.0, 2

import adsk.core, adsk.fusion, math, os
cm = lambda mm: mm / 10.0

F = AUW_KG * 9.81 * G_LAND / N_LEGS             # N per leg
arm = FOOT_U - BLOCK_U[0]                       # horizontal lever from the foot to the block's inner edge
M = F * arm                                     # N*mm at the root
S = LEG_T * W_ROOT**2 / 6                       # section modulus at the root, bending in the profile plane
sigma = M / S
sf = SIGMA_PETG_MPA / sigma
assert sf >= SF_MIN, f'leg root stress {sigma:.1f} MPa, safety factor {sf:.1f} < {SF_MIN}'
bolt_tension = M / (BOLT_U[1] - BOLT_U[0])      # the two bolts resist the root moment as a couple
assert bolt_tension < 1500, f'bolt tension {bolt_tension:.0f} N too high for M3 in a 2 mm plate'
_dx, _dy = FOOT_U - ROOT_U, FOOT_V - ROOT_V; _l = math.hypot(_dx, _dy); _n = (-_dy/_l, _dx/_l)
for _s in (1, -1):  # strut root corners must sit inside the block (no notch)
    _cu, _cv = ROOT_U + _s*_n[0]*W_ROOT/2, ROOT_V + _s*_n[1]*W_ROOT/2
    assert BLOCK_U[0] <= _cu <= BLOCK_U[1] and -BLOCK_H <= _cv <= 0, f'strut root corner ({_cu:.1f}, {_cv:.1f}) outside the block'
_inner_u = min(ROOT_U + _s*_n[0]*W_ROOT/2 for _s in (1, -1))   # strut's inner edge only moves outward below the block
assert max(BOLT_U) + CB_D/2 < _inner_u, 'strut sits under a bolt head: no screwdriver access from below'
assert min(BOLT_U) - CB_D/2 >= BLOCK_U[0] + 0.5, 'head pocket breaks out of the block'
assert BOSS_R - TUBE_D/2 >= 1.6 and W_MID >= 1.6 * 4, 'walls under the FDM minimum'

for d in [d for d in app.documents if d.name.startswith('Widow landing leg')]:
    d.close(False)
doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
doc.name = 'Widow landing leg'
des = adsk.fusion.Design.cast(app.activeProduct)
des.designType = adsk.fusion.DesignTypes.DirectDesignType
root = des.rootComponent
ext = root.features.extrudeFeatures
NEW, JOIN, CUT = (adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
                  adsk.fusion.FeatureOperations.JoinFeatureOperation,
                  adsk.fusion.FeatureOperations.CutFeatureOperation)
P = lambda u, v: adsk.core.Point3D.create(cm(u), cm(v), 0)

def poly(sk, pts):
    L = sk.sketchCurves.sketchLines
    for i in range(len(pts)):
        L.addByTwoPoints(P(*pts[i]), P(*pts[(i + 1) % len(pts)]))

def extrude_sym(sk, op, body=None, prof=None):
    profs = adsk.core.ObjectCollection.create()
    for p in ([prof] if prof else sk.profiles):
        profs.add(p)
    inp = ext.createInput(profs, op)
    inp.setSymmetricExtent(adsk.core.ValueInput.createByReal(cm(LEG_T)), True)
    if body:
        inp.participantBodies = [body]
    return ext.add(inp)

xy = root.xYConstructionPlane
# top block
sk = root.sketches.add(xy); sk.name = 'block'
poly(sk, [(BLOCK_U[0], -BLOCK_H), (BLOCK_U[1], -BLOCK_H), (BLOCK_U[1], 0), (BLOCK_U[0], 0)])
body = extrude_sym(sk, NEW).bodies.item(0); body.name = 'landing leg'

# strut: from the block's underside to the foot boss, tapering W_ROOT -> W_MID -> W_FOOT
r0 = (ROOT_U, ROOT_V)
r1 = (FOOT_U, FOOT_V)
dx, dy = r1[0] - r0[0], r1[1] - r0[1]; ln = math.hypot(dx, dy); ux, uy = dx/ln, dy/ln; nx, ny = -uy, ux
pt = lambda t, w, s: (r0[0] + ux*ln*t + s*nx*w/2, r0[1] + uy*ln*t + s*ny*w/2)
sk = root.sketches.add(xy); sk.name = 'strut'
poly(sk, [pt(0, W_ROOT, 1), pt(0.5, W_MID, 1), pt(1, W_FOOT, 1), pt(1, W_FOOT, -1), pt(0.5, W_MID, -1), pt(0, W_ROOT, -1)])
extrude_sym(sk, JOIN, body)

# foot boss
sk = root.sketches.add(xy); sk.name = 'boss'
sk.sketchCurves.sketchCircles.addByCenterRadius(P(FOOT_U, FOOT_V), cm(BOSS_R))
extrude_sym(sk, JOIN, body)
if root.bRepBodies.count > 1:  # joins of separately-sketched solids can land as extra bodies; merge them
    tools = adsk.core.ObjectCollection.create()
    for b in list(root.bRepBodies)[1:]:
        tools.add(b)
    ci = root.features.combineFeatures.createInput(body, tools); ci.operation = JOIN
    root.features.combineFeatures.add(ci)
    body = root.bRepBodies.item(0)

# skid tube hole along the extrusion (vertical when printed, so it prints round)
sk = root.sketches.add(xy); sk.name = 'tube hole'
sk.sketchCurves.sketchCircles.addByCenterRadius(P(FOOT_U, FOOT_V), cm(TUBE_D / 2))
extrude_sym(sk, CUT, body)

# holes that run up/down on the drone = along local v: sketch on the XZ plane (normal along v) and cut through
def v_hole(u, d, v_from, v_to):
    plane_in = root.constructionPlanes.createInput()
    plane_in.setByOffset(root.xZConstructionPlane, adsk.core.ValueInput.createByReal(cm(v_from)))
    pl = root.constructionPlanes.add(plane_in)
    s = root.sketches.add(pl)
    c = s.modelToSketchSpace(adsk.core.Point3D.create(cm(u), cm(v_from), 0))
    s.sketchCurves.sketchCircles.addByCenterRadius(c, cm(d / 2))
    inp = ext.createInput(s.profiles.item(0), CUT)
    inp.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal(cm(abs(v_to - v_from)))),
                         adsk.fusion.ExtentDirections.PositiveExtentDirection if v_to > v_from else adsk.fusion.ExtentDirections.NegativeExtentDirection)
    inp.participantBodies = [body]
    ext.add(inp)

for u in BOLT_U:
    v_hole(u, BOLT_D, -BLOCK_H - 1, 1)                    # through the block
    v_hole(u, CB_D, -BLOCK_H - 1, -BLOCK_H + CB_DEPTH)     # head pocket from below
v_hole(FOOT_U, SET_D, FOOT_V + BOSS_R + 1, FOOT_V)        # set screw from the top of the boss into the tube

assert root.bRepBodies.count == 1 and body.lumps.count == 1, 'leg is not one solid'
vol = body.physicalProperties.volume
bb = body.boundingBox
OUT = os.path.join(HERE, '..', 'Widow', 'Printed Parts', 'Landing Leg')
os.makedirs(OUT, exist_ok=True)
em = des.exportManager
o = em.createSTLExportOptions(body, os.path.join(OUT, 'landing-leg.stl'))
o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
em.execute(o)
em.execute(em.createSTEPExportOptions(os.path.join(OUT, 'landing-leg.step'), root))
vp = app.activeViewport
cam = vp.camera; cam.viewOrientation = adsk.core.ViewOrientations.TopViewOrientation; cam.isFitView = True; vp.camera = cam
vp.saveAsImageFile(os.path.join(OUT, 'landing-leg-preview.png'), 1200, 800)
{'volume_cm3': round(vol, 2), 'mass_g_PETG_solid': round(vol * 1.27, 1),
 'bbox_mm': [round((bb.maxPoint.x - bb.minPoint.x)*10, 1), round((bb.maxPoint.y - bb.minPoint.y)*10, 1), round((bb.maxPoint.z - bb.minPoint.z)*10, 1)],
 'load_per_leg_N': round(F, 1), 'root_stress_MPa': round(sigma, 2), 'safety_factor': round(sf, 1), 'bolt_tension_N': round(bolt_tension)}
