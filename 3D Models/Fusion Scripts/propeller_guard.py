# Widow 8" propeller guard. Runs inside Fusion via the add-in's execute_code:
#   python3 run_in_fusion.py execute_code -code propeller_guard.py
# Saves the STL, STEP and a preview into ../Widow/Printed Parts/Propeller Guard/ (HERE is set by run_in_fusion.py).
# All mm; Fusion API works in cm, so every value goes through cm().
# z=0 is the guard bottom (sits on the arm's motor pad); the motor sits on z=HUB_T.
# Motor holes are the 22xx cross pattern (16 mm pair + 19 mm pair, read from photos),
# not a square, so the holes are kidney slots that take either pair on either diagonal.
PROP_D      = 203.2   # 8 in
TIP_GAP     = 6.4     # prop tip to ring inner wall
RING_T      = 2.0     # ring wall thickness
RING_H      = 28.0    # prop plane is HUB_T+20 = 22, so the ring rises 6 mm above it
ARC_HALF    = 95.0    # deg each side of outward; neighbours at 216 mm overlap past ~103
HUB_R       = 15.0    # motor base is 27.47 dia
HUB_T       = 2.0     # sandwiched: 10 mm screws become 12 mm, same 5 mm thread engagement
CENTER_D    = 10.5    # clears the bearing boss and circlip (about 10.3 mm in the photo)
HOLE_D      = 3.4     # M3 clearance
HOLE_R_MIN  = 8.0     # 16 mm pair
HOLE_R_MAX  = 9.5     # 19 mm pair
HOLE_ANG0   = 45.0    # pairs sit on the diagonals to the arm
HOLE_ANG_TOL = 6.0    # photo shows the pattern ~5 deg off the arm; slot covers +-6 deg
SPOKE_ANGS  = (0.0, 65.0, -65.0)
SPOKE_W     = 5.0
RIB_R0      = 17.0    # tall rib starts outside the bell (r 13.7)
RIB_H       = 6.0

import adsk.core, adsk.fusion, math
cm = lambda mm: mm / 10.0
P = lambda x, y: adsk.core.Point3D.create(cm(x), cm(y), 0)

# self-check: the slot must leave a web to the centre hole and stay inside the hub
assert HOLE_R_MIN - HOLE_D/2 - CENTER_D/2 >= 1.0, 'web to centre hole under 1 mm'
assert HOLE_R_MAX + HOLE_D/2 <= HUB_R - 1.5, 'slot too close to hub edge'

for d in [d for d in app.documents if d.name.startswith(('Widow prop guard', 'Widow propeller guard'))]:
    d.close(False)  # rerun replaces the unsaved previous build
doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
doc.name = 'Widow propeller guard'
des = adsk.fusion.Design.cast(app.activeProduct)
root = des.rootComponent
xy = root.xYConstructionPlane
ext = root.features.extrudeFeatures
NEW, JOIN, CUT = (adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
                  adsk.fusion.FeatureOperations.JoinFeatureOperation,
                  adsk.fusion.FeatureOperations.CutFeatureOperation)

def extrude(sketch, h, op, body=None):
    profs = adsk.core.ObjectCollection.create()
    for p in sketch.profiles:
        profs.add(p)
    inp = ext.createInput(profs, op)
    inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(cm(h)))
    if body:
        inp.participantBodies = [body]
    return ext.add(inp)

def spoke_rects(sk, r0, r1):
    L = sk.sketchCurves.sketchLines
    for a in SPOKE_ANGS:
        t = math.radians(a); ux, uy = math.cos(t), math.sin(t); nx, ny = -uy, ux
        w = SPOKE_W / 2
        pts = [P(r0*ux + w*nx, r0*uy + w*ny), P(r1*ux + w*nx, r1*uy + w*ny),
               P(r1*ux - w*nx, r1*uy - w*ny), P(r0*ux - w*nx, r0*uy - w*ny)]
        for i in range(4):
            L.addByTwoPoints(pts[i], pts[(i + 1) % 4])

r_in = PROP_D / 2 + TIP_GAP
r_out = r_in + RING_T

# hub
sk = root.sketches.add(xy); sk.name = 'hub'
sk.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0), cm(HUB_R))
body = extrude(sk, HUB_T, NEW).bodies.item(0)
body.name = 'prop guard'

# spokes: flat 2 mm from the hub, tall rib outside the bell; both run into the ring wall
sk = root.sketches.add(xy); sk.name = 'spokes flat'
spoke_rects(sk, HUB_R - 3, r_in + 1)
extrude(sk, HUB_T, JOIN, body)
sk = root.sketches.add(xy); sk.name = 'spoke ribs'
spoke_rects(sk, RIB_R0, r_in + 1)
extrude(sk, RIB_H, JOIN, body)

# ring arc (outward side only)
sk = root.sketches.add(xy); sk.name = 'ring'
A = sk.sketchCurves.sketchArcs; L = sk.sketchCurves.sketchLines
sweep = math.radians(2 * ARC_HALF); a0 = math.radians(-ARC_HALF); a1 = -a0
A.addByCenterStartSweep(P(0, 0), P(r_in*math.cos(a0), r_in*math.sin(a0)), sweep)
A.addByCenterStartSweep(P(0, 0), P(r_out*math.cos(a0), r_out*math.sin(a0)), sweep)
L.addByTwoPoints(P(r_in*math.cos(a0), r_in*math.sin(a0)), P(r_out*math.cos(a0), r_out*math.sin(a0)))
L.addByTwoPoints(P(r_in*math.cos(a1), r_in*math.sin(a1)), P(r_out*math.cos(a1), r_out*math.sin(a1)))
extrude(sk, RING_H, JOIN, body)
# the ring join lands as its own body anyway; merge it explicitly
if root.bRepBodies.count > 1:
    tools = adsk.core.ObjectCollection.create()
    for b in list(root.bRepBodies)[1:]:
        tools.add(b)
    ci = root.features.combineFeatures.createInput(body, tools)
    ci.operation = JOIN
    root.features.combineFeatures.add(ci)
    body = root.bRepBodies.item(0)

# holes: centre hole + 4 kidney slots (annular sector r MIN..MAX widened by the hole radius, +-TOL deg)
sk = root.sketches.add(xy); sk.name = 'holes'
C = sk.sketchCurves.sketchCircles; A = sk.sketchCurves.sketchArcs; L = sk.sketchCurves.sketchLines
C.addByCenterRadius(P(0, 0), cm(CENTER_D / 2))
ri, ro = HOLE_R_MIN - HOLE_D/2, HOLE_R_MAX + HOLE_D/2
pol = lambda r, a: P(r*math.cos(a), r*math.sin(a))
for k in range(4):
    c = math.radians(HOLE_ANG0 + 90 * k); d = math.radians(HOLE_ANG_TOL) + (HOLE_D/2) / HOLE_R_MIN
    s, e = c - d, c + d
    A.addByCenterStartSweep(P(0, 0), pol(ri, s), 2*d)
    A.addByCenterStartSweep(P(0, 0), pol(ro, s), 2*d)
    L.addByTwoPoints(pol(ri, s), pol(ro, s))
    L.addByTwoPoints(pol(ri, e), pol(ro, e))
profs = adsk.core.ObjectCollection.create()
for p in sk.profiles:
    profs.add(p)
inp = ext.createInput(profs, CUT)
inp.setAllExtent(adsk.fusion.ExtentDirections.PositiveExtentDirection)
inp.participantBodies = [body]
ext.add(inp)

import os
OUT = os.path.join(HERE, '..', 'Widow', 'Printed Parts', 'Propeller Guard')
out = os.path.join(OUT, 'propeller-guard.stl')
o = des.exportManager.createSTLExportOptions(body, out)
o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
des.exportManager.execute(o)
des.exportManager.execute(des.exportManager.createSTEPExportOptions(os.path.join(OUT, 'propeller-guard.step'), root))  # the assembly imports this
vp = app.activeViewport
cam = vp.camera; cam.viewOrientation = adsk.core.ViewOrientations.TopViewOrientation; cam.isFitView = True; vp.camera = cam
vp.saveAsImageFile(os.path.join(OUT, 'propeller-guard-preview.png'), 1200, 800)
pr = body.physicalProperties
{'bodies': root.bRepBodies.count, 'lumps': body.lumps.count, 'volume_cm3': round(pr.volume, 2),
 'mass_g_solid_PLA': round(pr.volume * 1.24, 1), 'stl': out}
