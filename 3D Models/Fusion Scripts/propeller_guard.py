# Widow 8" propeller guard, same style as Huntsman's. Runs inside Fusion via the add-in:
#   python3 run_in_fusion.py execute_code -code propeller_guard.py
# Saves the STL, STEP and a preview into ../Widow/Printed Parts/Propeller Guard/ (HERE is set by run_in_fusion.py).
#
# A thin hub sits between the motor and the arm's motor pad, held by the motor's 4 screws.
#   hub    - disc under the motor base; the 22xx cross pattern (16 mm pair + 19 mm pair, read from photos) is not a
#            square, so the holes are kidney slots that take either pair on either diagonal.
#   struts - 4 T ribs printed flat (flange on the bed, web up the middle), roots widened at the hub. Hub thick under
#            the motor bell, then up a 45 deg ramp to full height.
#   ring   - open ring round the outer 190 deg of the prop: solid band at the prop tips with a lip round its top edge,
#            a diagonal lattice of 45 deg bars below it, a narrow inward flange on the bed, a fin outside the ring at
#            each strut end. Wall = 3 lines of 0.43.
# Why this shape (FEA, ../FEA/fea.py widow_guard): the old guard's solid 2 mm ring carried almost no stress while its
# 2 mm flat spoke roots were at the PETG limit under a 1 kg push. Material moved from the ring to the roots and struts.
# Prints in one piece, flat (as built, hub on the bed), 3 walls, 15 % infill, PETG.
# Local coords: motor axis at the origin, x outward along the arm, z = 0 the guard bottom on the arm's motor pad, mm.
PROP_D = 203.2                     # 8 in
HUB_T = 2.0                        # sandwiched: 10 mm motor screws become 12 mm, same 5 mm thread engagement
PROP_Z = 25.0                      # prop above the pad: 25.5 in full_drone.py (mid-hub), 22 in the old script; band covers both
PROP_LOW = 20.0                    # EST: lowest point of the prop (its hub) above the pad
BELL_R = 13.74                     # motor bell radius (27.47 measured)
TIP_GAP, RING_T = 6.4, 1.7         # tip to ring wall; ring wall (4 extrusion lines: FEA, a 1.3 band let a push reach the prop)
BAND_UP, BAND_DN = 5.0, 5.0        # solid band from PROP_Z - BAND_DN to PROP_Z + BAND_UP (top of the guard)
ARC_HALF = 95.0                    # deg each side of outward; neighbours at 216 mm overlap past ~103
FL_W, FL_T = 3.0, 1.2              # inward flange along the ring bottom (on the bed)
HUB_R, CENTER_D = 15.0, 10.5       # hub disc (motor base 27.47); hole for the bearing boss and circlip
HOLE_D, HOLE_R_MIN, HOLE_R_MAX = 3.4, 8.0, 9.5   # M3; 16 mm pair, 19 mm pair
HOLE_ANG0, HOLE_ANG_TOL = 45.0, 6.0   # pairs sit on the diagonals to the arm; photo shows ~5 deg off, slots cover +-6
STRUTS = ((40.0, 20.0), (-40.0, -20.0), (100.0, 75.0), (-100.0, -75.0))   # (angle at the hub, angle at the ring), deg
STRUT_W, STRUT_FT, STRUT_WEB, STRUT_H = 7.0, 1.2, 1.3, 11.0  # strut flange width, flange thickness, web (3 lines), height
ROOT_W, ROOT_L = 12.0, 7.0         # strut roots widen to this at the hub, over this length (hub thick, under the bell)
TOP_LIP = 5.0                      # triangular lip outside the ring's top edge, 45 deg underneath (prints without supports)
BUT_D, BUT_T = 12.0, 1.5           # buttress at each strut end: radial depth at the bed (to 0 at the top), thickness
DIA_H, DIA_WEB = 9.0, 2.0          # diagonal lattice below the band: diamond size, bar width between diamonds
NEIGHBOUR = 216.0                  # between neighbouring motors
BED, BED_MARGIN = 256.0, 3.0       # Bambu Lab X2D bed (main nozzle); room for a 3 mm brim
# strength self-check
AUW_KG, G_SIDE, E_MPA, K_END, SF_MIN = 1.885, 1.0, 1500.0, 0.7, 2.0   # E: PETG, conservative; SF 2 for the 15 % infill

import adsk.core, adsk.fusion, math, os
cm = lambda mm: mm / 10.0
r_in = PROP_D / 2 + TIP_GAP
r_mid = r_in + RING_T / 2
top = PROP_Z + BAND_UP
band0 = PROP_Z - BAND_DN                       # lattice below this, solid band above
lat0, lat1 = FL_T + 1.0, band0
hb = DIA_H
R_LOW = BELL_R + 3.3                           # struts stay hub-thick inside this radius
pol = lambda r, a: (r * math.cos(math.radians(a)), r * math.sin(math.radians(a)))
struts = [(pol(HUB_R - 1, ah), pol(r_in - 0.8, ar)) for ah, ar in STRUTS]   # ends just inside the wall

# --- self-checks
assert r_in - PROP_D / 2 >= 6, 'prop tips too close to the ring'
assert STRUT_H <= PROP_LOW - 8, 'struts must stay 8 mm under the prop'
assert lat1 - lat0 >= hb and hb <= 15, 'lattice band too short, or openings bridge too far'
assert HOLE_R_MIN - HOLE_D / 2 - CENTER_D / 2 >= 1.0 and HOLE_R_MAX + HOLE_D / 2 <= HUB_R - 1.5, 'screw slots: web to the centre hole or hub edge'
nb = pol(NEIGHBOUR, 120); end = pol(r_in + RING_T + TOP_LIP, ARC_HALF)
d_end = math.hypot(end[0] - nb[0], end[1] - nb[1])
assert d_end - PROP_D / 2 >= 15, 'ring end too close to the neighbouring prop'
(p0, p1) = struts[0]
L1 = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
F_strut = AUW_KG * 9.81 * G_SIDE / (2 * (p1[0] - p0[0]) / L1)
I_min = STRUT_FT * STRUT_W**3 / 12 + (STRUT_H - STRUT_FT) * STRUT_WEB**3 / 12   # sideways: the T's weak axis
P_cr = math.pi**2 * E_MPA * I_min / (K_END * L1) ** 2
assert P_cr / F_strut >= SF_MIN, f'struts buckle: {P_cr:.0f} N vs {F_strut:.0f} N'

for d in [d for d in app.documents if d.name.startswith(('Widow prop guard', 'Widow propeller guard'))]:
    d.close(False)  # rerun replaces the unsaved previous build
doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
doc.name = 'Widow propeller guard'
des = adsk.fusion.Design.cast(app.activeProduct)
des.designType = adsk.fusion.DesignTypes.DirectDesignType
root = des.rootComponent
NEW, JOIN, CUT = (adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
                  adsk.fusion.FeatureOperations.JoinFeatureOperation,
                  adsk.fusion.FeatureOperations.CutFeatureOperation)
V = lambda mm: adsk.core.ValueInput.createByReal(cm(mm))

def sketch(comp, plane, off=0.0):
    base = {'xy': comp.xYConstructionPlane, 'xz': comp.xZConstructionPlane, 'yz': comp.yZConstructionPlane}[plane]
    if off:
        pi = comp.constructionPlanes.createInput(); pi.setByOffset(base, V(off)); base = comp.constructionPlanes.add(pi)
    return comp.sketches.add(base)

def pt(sk, x, y, z):
    return sk.modelToSketchSpace(adsk.core.Point3D.create(cm(x), cm(y), cm(z)))

def poly(sk, pts):
    L = sk.sketchCurves.sketchLines
    for i in range(len(pts)):
        L.addByTwoPoints(pt(sk, *pts[i]), pt(sk, *pts[(i + 1) % len(pts)]))
    return sk

def circle(sk, c, d):
    sk.sketchCurves.sketchCircles.addByCenterRadius(pt(sk, *c), cm(d / 2)); return sk

def extrude(comp, sk, op, h=None, sym=None, body=None):
    profs = adsk.core.ObjectCollection.create()
    for p in sk.profiles:
        profs.add(p)
    inp = comp.features.extrudeFeatures.createInput(profs, op)
    if sym:
        inp.setSymmetricExtent(V(sym), True)
    else:
        inp.setDistanceExtent(False, V(h))
    if body:
        inp.participantBodies = [body]
    return comp.features.extrudeFeatures.add(inp)

def move(comp, bodies, deg, t):
    M = adsk.core.Matrix3D.create()
    M.setToRotation(math.radians(deg), adsk.core.Vector3D.create(0, 0, 1), adsk.core.Point3D.create(0, 0, 0))
    T = adsk.core.Matrix3D.create(); T.translation = adsk.core.Vector3D.create(cm(t[0]), cm(t[1]), cm(t[2])); M.transformBy(T)
    coll = adsk.core.ObjectCollection.create()
    for b in bodies:
        coll.add(b)
    mi = comp.features.moveFeatures.createInput2(coll); mi.defineAsFreeMove(M); comp.features.moveFeatures.add(mi)

def combine(comp, target, tools, op):
    coll = adsk.core.ObjectCollection.create()
    for t in tools:
        coll.add(t)
    ci = comp.features.combineFeatures.createInput(target, coll); ci.operation = op
    comp.features.combineFeatures.add(ci)

def revolve(comp, pts, op, full=False, bodies=None):
    sk = poly(sketch(comp, 'xz'), [(x, 0, z) for x, z in pts])
    ri = comp.features.revolveFeatures.createInput(sk.profiles.item(0), comp.zConstructionAxis, op)
    if full:
        ri.setAngleExtent(False, adsk.core.ValueInput.createByReal(2 * math.pi))
    else:
        ri.setAngleExtent(True, adsk.core.ValueInput.createByReal(math.radians(ARC_HALF)))   # symmetric: angle is per side
    if bodies:
        ri.participantBodies = bodies
    return comp.features.revolveFeatures.add(ri)

c = root
ring = revolve(c, [(r_in - FL_W, 0), (r_in + RING_T, 0), (r_in + RING_T, top), (r_in, top), (r_in, FL_T), (r_in - FL_W, FL_T)], NEW).bodies.item(0)
parts_ = []
for (p0, p1) in struts:                # T ribs built along x, turned and moved onto their line
    L = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    w, f_, t = STRUT_W / 2, STRUT_FT, STRUT_WEB / 2
    b = extrude(c, poly(sketch(c, 'yz'), [(0, -w, 0), (0, w, 0), (0, w, f_), (0, t, f_), (0, t, STRUT_H), (0, -t, STRUT_H),
                                          (0, -t, f_), (0, -w, f_)]), NEW, L).bodies.item(0)
    move(c, [b], math.degrees(math.atan2(p1[1] - p0[1], p1[0] - p0[0])), (p0[0], p0[1], 0)); parts_.append(b)
    ux, uy = (p1[0] - p0[0]) / L, (p1[1] - p0[1]) / L          # wide root: trapezoid from inside the hub, hub thick
    s0, s1 = (p0[0] - 3 * ux, p0[1] - 3 * uy), (p0[0] + ROOT_L * ux, p0[1] + ROOT_L * uy)
    extrude(c, poly(sketch(c, 'xy'), [(s0[0] - uy * ROOT_W / 2, s0[1] + ux * ROOT_W / 2, 0), (s1[0] - uy * w, s1[1] + ux * w, 0),
                                      (s1[0] + uy * w, s1[1] - ux * w, 0), (s0[0] + uy * ROOT_W / 2, s0[1] - ux * ROOT_W / 2, 0)]), NEW, HUB_T)
revolve(c, [(0, HUB_T), (R_LOW, HUB_T), (R_LOW + STRUT_H - HUB_T + 1, STRUT_H + 1), (0, STRUT_H + 1)], CUT, full=True, bodies=parts_)
for _, ar in STRUTS:                   # buttress: a triangular fin outside the ring, in line with the strut
    b = extrude(c, poly(sketch(c, 'xz'), [(r_in + RING_T - 0.5, 0, 0), (r_in + RING_T + BUT_D, 0, 0), (r_in + RING_T - 0.5, 0, top)]),
                NEW, sym=BUT_T).bodies.item(0)
    move(c, [b], ar, (0, 0, 0))
revolve(c, [(r_in + RING_T - 0.3, top), (r_in + RING_T + TOP_LIP, top), (r_in + RING_T - 0.3, top - TOP_LIP)], NEW)   # top lip
extrude(c, circle(sketch(c, 'xy'), (0, 0, 0), 2 * HUB_R), NEW, HUB_T)
combine(c, ring, [b for b in c.bRepBodies if b != ring], JOIN)
body = c.bRepBodies.item(0)
# lattice: a diagonal grid of 45 deg bars below the solid band (diamonds on staggered rows, clipped flat at the band
# and the flange), skipping strut joints and the ring ends
def clip(pts, z, keep_above):          # Sutherland-Hodgman against one horizontal line, in (t, z)
    out = []
    for i in range(len(pts)):
        a, b = pts[i], pts[(i + 1) % len(pts)]
        ina, inb = (a[1] >= z) == keep_above, (b[1] >= z) == keep_above
        if ina:
            out.append(a)
        if ina != inb:
            out.append((a[0] + (b[0] - a[0]) * (z - a[1]) / (b[1] - a[1]), z))
    return out
cutters, pitch = [], hb + DIA_WEB * math.sqrt(2)
t_max = math.radians(ARC_HALF) * r_mid - hb / 2 - 3
avoid = [(math.radians(ar) * r_mid, STRUT_W / 2 + BUT_T + 2) for _, ar in STRUTS]
for row in range(int((lat1 - lat0) // (pitch / 2)) + 2):
    zc = lat0 + row * pitch / 2
    for k in range(-int(t_max // pitch) - 1, int(t_max // pitch) + 2):
        tc = k * pitch + (pitch / 2 if row % 2 else 0.0)
        if abs(tc) > t_max or any(abs(tc - t0) < hb / 2 + d for t0, d in avoid):
            continue
        pts = clip(clip([(tc, zc - hb / 2), (tc + hb / 2, zc), (tc, zc + hb / 2), (tc - hb / 2, zc)], lat0, True), lat1, False)
        if len(pts) < 3 or max(p[1] for p in pts) - min(p[1] for p in pts) < 2.5:
            continue                   # sliver: not worth a cut
        b = extrude(c, poly(sketch(c, 'yz'), [(0, t - tc, z) for t, z in pts]), NEW, sym=4 * RING_T + 2).bodies.item(0)
        m = adsk.core.Matrix3D.create(); m.translation = adsk.core.Vector3D.create(cm(r_in + RING_T / 2), 0, 0)
        R = adsk.core.Matrix3D.create(); R.setToRotation(tc / r_mid, adsk.core.Vector3D.create(0, 0, 1), adsk.core.Point3D.create(0, 0, 0))
        m.transformBy(R)               # out to the ring, then round (one move: an identity move is refused)
        coll = adsk.core.ObjectCollection.create(); coll.add(b)
        mi = c.features.moveFeatures.createInput2(coll); mi.defineAsFreeMove(m); c.features.moveFeatures.add(mi)
        cutters.append(b)
combine(c, body, cutters, CUT)
# hub holes: centre hole + 4 kidney slots (annular sector r MIN..MAX widened by the hole radius, +-TOL deg)
sk = sketch(c, 'xy', -1)
Pp = lambda r, a: pt(sk, r * math.cos(a), r * math.sin(a), -1)
sk.sketchCurves.sketchCircles.addByCenterRadius(pt(sk, 0, 0, -1), cm(CENTER_D / 2))
A, Ln = sk.sketchCurves.sketchArcs, sk.sketchCurves.sketchLines
ri_, ro_ = HOLE_R_MIN - HOLE_D / 2, HOLE_R_MAX + HOLE_D / 2
for k in range(4):
    a = math.radians(HOLE_ANG0 + 90 * k); d = math.radians(HOLE_ANG_TOL) + (HOLE_D / 2) / HOLE_R_MIN
    A.addByCenterStartSweep(pt(sk, 0, 0, -1), Pp(ri_, a - d), 2 * d)
    A.addByCenterStartSweep(pt(sk, 0, 0, -1), Pp(ro_, a - d), 2 * d)
    Ln.addByTwoPoints(Pp(ri_, a - d), Pp(ro_, a - d)); Ln.addByTwoPoints(Pp(ri_, a + d), Pp(ro_, a + d))
extrude(c, sk, CUT, HUB_T + 2, body=body)
assert c.bRepBodies.count == 1 and body.lumps.count == 1, 'guard is not one solid'
body.name = 'prop guard'
bb = body.boundingBox
span = [round((bb.maxPoint.x - bb.minPoint.x) * 10, 1), round((bb.maxPoint.y - bb.minPoint.y) * 10, 1)]
assert max(span) <= BED - 2 * BED_MARGIN, f'guard {span} mm does not fit the bed'

OUT = os.path.join(HERE, '..', 'Widow', 'Printed Parts', 'Propeller Guard')
o = des.exportManager.createSTLExportOptions(body, os.path.join(OUT, 'propeller-guard.stl'))
o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
des.exportManager.execute(o)
des.exportManager.execute(des.exportManager.createSTEPExportOptions(os.path.join(OUT, 'propeller-guard.step'), root))  # the assembly and FEA import this
for s in c.sketches:
    s.isVisible = False
for p in c.constructionPlanes:
    p.isLightBulbOn = False
vp = app.activeViewport
cam = vp.camera; cam.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation; cam.isFitView = True; vp.camera = cam
vp.saveAsImageFile(os.path.join(OUT, 'propeller-guard-preview.png'), 1200, 800)
{'strut_buckling_sf': round(P_cr / F_strut, 2), 'strut_len_mm': round(L1, 1), 'ring_r_in': r_in, 'ring_z': [0, top],
 'diamonds': len(cutters), 'ring_end_to_neighbour_prop_mm': round(d_end - PROP_D / 2, 1),
 'volume_cm3': round(body.physicalProperties.volume, 2), 'span_mm': span}
