# Huntsman (Tarot 680 PRO) 13" propeller guard. Runs inside Fusion via the add-in:
#   python3 run_in_fusion.py execute_code -code huntsman_propeller_guard.py
# Saves STEP/STL/preview into ../Huntsman/Printed Parts/Propeller Guard/ (HERE is set by run_in_fusion.py).
#
# Like Widow's guard, it goes under the motor: a thin hub between the motor and the Tarot motor plate, held by the
# motor's own 3 screws, so it cannot turn on the arm (the old guard clamped the round arm tube and only friction held it).
#   hub    - disc under the motor base, a slot for each motor screw, a hole for the shaft circlip.
#   struts - 4 T ribs printed flat, straight from the hub edge to the ring, roots widened at the hub. Hub thick under
#            the motor bell, which sits only 2 mm above the plate, then up a 45 deg ramp to full height. Sized so the
#            whole drone's weight pushing on the ring's outer point does not buckle them (Euler, fixed-pinned).
#   ring   - open ring round the outer 190 deg of the prop: solid band at the prop tips with a lip round its top edge,
#            a diagonal lattice of 45 deg bars below it, a narrow inward flange on the bed, a fin outside the ring at
#            each strut end. Wall = 3 lines of 0.43. Checked with FEA (../FEA/fea.py guard).
# It is 349 mm across, too big for the X2D, so it splits down its middle (y = 0) into a left and a right half. The motor
# screws clamp both hub halves; a block under the ring at the outer point takes an M2.5 cross bolt into an insert.
# Print flat (as built, hub on the bed), 3 walls, 15 % infill, PETG.
# Local coords: motor axis at the origin, x outward along the arm, z = 0 the top of the Tarot motor plate (where the
# motor sits), mm. Frame and motor numbers come from the online Tarot 680 PRO model: EST until measured.
PROP_D = 330.2                     # 13 in (1355)
HUB_T = 3.0                        # hub under the motor: the motor and prop rise this much; motor screws get this much longer
PROP_Z = 31.8 + HUB_T              # EST (model): prop 23.8-39.8 above the plate, tips taken at the middle; measure
PROP_LOW = 23.8 + HUB_T            # EST (model): lowest point of the prop (its hub)
BELL_R, BELL_Z = 23.05, 2.0 + HUB_T   # EST (model): motor bell radius; its rim this high above the plate
TIP_GAP, RING_T = 8.0, 1.3         # tip to ring wall; ring wall (3 extrusion lines)
BAND_UP, BAND_DN = 6.0, 4.0        # solid band from PROP_Z - BAND_DN to PROP_Z + BAND_UP (top of the guard)
ARC_HALF = 95.0                    # deg each side of outward: neighbour rings would meet near 108 deg
FL_W, FL_T = 4.0, 1.6              # inward flange along the ring bottom (on the bed)
HUB_R, CENTER_D = 22.0, 12.0       # hub disc (inside the frame's motor-plate bolts at r 25.9); circlip hole (EST)
# Tarot 4108 (TL68P07): 3 x M3 on a "27 mm equilateral triangle" (retailer listing): 27 across the circle or 27 a side,
# so each slot runs from r 13.5 to r 15.6. HOLE_ANG0 = where one screw sits, deg from outward (EST: look under the mount).
PAT_R0, PAT_R1, HOLE_D, HOLE_ANG0, HOLE_ANG_TOL = 13.5, 15.6, 3.4, 90.0, 8.0
STRUTS = ((50.0, 20.0), (-50.0, -20.0), (100.0, 75.0), (-100.0, -75.0))   # (angle at the hub, angle at the ring), deg
# What FEA showed (FEA/fea.py): a bump lands on the band ~30 mm above the struts, so (1) the band itself bends in toward
# the prop between struts, (2) the struts bend up and down, hardest at their thin roots under the motor bell. Hence:
# a lip round the top edge (stiffens the band), T-section struts (flange on the bed against sideways buckling, web for
# up-and-down bending), wide roots, and a buttress outside the ring at each strut end.
STRUT_W, STRUT_FT, STRUT_WEB, STRUT_H = 8.0, 1.2, 1.3, 10.0   # strut flange width, flange thickness, web (3 lines), height
ROOT_W, ROOT_L = 18.0, 10.0        # strut roots widen to this at the hub, over this length (hub thick, under the bell)
TOP_LIP = 5.0                      # triangular lip outside the ring's top edge, 45 deg underneath (prints without supports)
BUT_D, BUT_T = 16.0, 1.7           # buttress at each strut end: radial depth at the bed (to 0 at the top), thickness
DIA_H, DIA_WEB = 12.0, 2.2         # diagonal lattice below the band: diamond size, bar width between diamonds
LIP = (26.0, 33.5, 13.8, 2.92 + 0.5)   # EST (model): raised lip at the motor plate's outer end: x0, x1, half width, top
NEIGHBOUR = 343.0                  # between neighbouring motors (hexagon)
JB_D, JB_W, JB_H = 10.0, 16.0, 9.0 # joint block under the ring at the split: radial depth, width across the split, height
BED, BED_MARGIN = 256.0, 3.0       # Bambu Lab X2D bed (main nozzle); room for a 3 mm brim
INSERT_D, INSERT_L, SCREW_D = 3.6, 4.0, 2.9      # M2.5 heat-set inserts (same EST as Widow)
# strength self-check
AUW_KG, G_SIDE, E_MPA, K_END, SF_MIN = 4.3, 1.0, 1500.0, 0.7, 2.0   # E: PETG, conservative; SF 2 for the 15 % infill

import adsk.core, adsk.fusion, math, os
cm = lambda mm: mm / 10.0
r_in = PROP_D / 2 + TIP_GAP
r_mid = r_in + RING_T / 2
top = PROP_Z + BAND_UP
band0 = PROP_Z - BAND_DN                       # lattice below this, solid band above
lat0, lat1 = FL_T + 1.0, band0                 # lattice band
hb = DIA_H                                     # diamond height = width (45 deg sides)
R_LOW = BELL_R + 1.5                           # struts stay hub-thick inside this radius
pol = lambda r, a: (r * math.cos(math.radians(a)), r * math.sin(math.radians(a)))
struts = [(pol(HUB_R - 1, ah), pol(r_in - 0.8, ar)) for ah, ar in STRUTS]   # ends just inside the wall: an angled end would poke out

# --- self-checks: clearances (EST geometry) and strength
assert r_in - PROP_D / 2 >= 6, 'prop tips too close to the ring'
assert STRUT_H <= PROP_LOW - 8, 'struts must stay 8 mm under the prop'
assert JB_H <= PROP_LOW - 8, 'joint block must stay 8 mm under the prop'
assert HUB_T <= BELL_Z - 1.5, 'hub-thick strut roots must clear the motor bell rim'
assert lat1 - lat0 >= hb, 'lattice band too short'
assert hb <= 15, 'lattice openings bridge their flat tops: keep them short'
assert PAT_R0 - HOLE_D / 2 - CENTER_D / 2 >= 1.5 and PAT_R1 + HOLE_D / 2 <= HUB_R - 2.5, 'screw slots: web to the centre hole or hub edge'
for (p0, p1) in struts:                        # the lip at the plate's outer end: sample the strut's footprint
    for k in range(101):
        x = p0[0] + (p1[0] - p0[0]) * k / 100; y = p0[1] + (p1[1] - p0[1]) * k / 100
        assert not (LIP[0] - 1 <= x <= LIP[1] + 1 and abs(y) - STRUT_W / 2 <= LIP[2] + 1), 'a strut runs over the motor plate lip'
# neighbour: its prop and ring must stay clear of our ring's ends (neighbour centre 120 deg round from outward)
nb = pol(NEIGHBOUR, 120); end = pol(r_in + RING_T, ARC_HALF)
d_end = math.hypot(end[0] - nb[0], end[1] - nb[1])
assert d_end - PROP_D / 2 >= 20 and d_end - (r_in + RING_T) >= 10, 'ring end too close to the neighbouring prop or ring'
# struts: a sideways bump of G_SIDE x the drone's weight on the ring's outer point, taken by the two struts nearest it
(p0, p1) = struts[0]
L1 = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
cos_b = (p1[0] - p0[0]) / L1
F_strut = AUW_KG * 9.81 * G_SIDE / (2 * cos_b)
I_min = STRUT_FT * STRUT_W**3 / 12 + (STRUT_H - STRUT_FT) * STRUT_WEB**3 / 12   # sideways: the T's weak axis
P_cr = math.pi**2 * E_MPA * I_min / (K_END * L1) ** 2
assert P_cr / F_strut >= SF_MIN, f'struts buckle: {P_cr:.0f} N vs {F_strut:.0f} N'
assert JB_D >= INSERT_D + 4 and JB_H >= INSERT_D + 4 and JB_W / 2 >= INSERT_L + 1.5, 'joint block too small for its insert'
# each half fits the bed as built (no turn): ring arc + hub
_hull = [pol(r_in + RING_T, t / 2) for t in range(0, int(2 * ARC_HALF) + 1)] + [pol(HUB_R, a) for a in range(0, 181, 5)]
span = (max(x for x, _ in _hull) - min(x for x, _ in _hull), max(y for _, y in _hull))
assert max(span) <= BED - 2 * BED_MARGIN, f'half guard {span} does not fit the bed'

for d in [d for d in app.documents if d.name.startswith('Huntsman propeller guard')]:
    d.close(False)
doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
doc.name = 'Huntsman propeller guard'
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

g_occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create()); g_occ.component.name = 'propeller guard'
c = g_occ.component
# ring
sk = poly(sketch(c, 'xz'), [(r_in - FL_W, 0, 0), (r_in + RING_T, 0, 0), (r_in + RING_T, 0, top), (r_in, 0, top),
                            (r_in, 0, FL_T), (r_in - FL_W, 0, FL_T)])
ri = c.features.revolveFeatures.createInput(sk.profiles.item(0), c.zConstructionAxis, NEW)
ri.setAngleExtent(True, adsk.core.ValueInput.createByReal(math.radians(ARC_HALF)))   # symmetric: angle is per side
ring = c.features.revolveFeatures.add(ri).bodies.item(0)
# struts: triangles built along x, turned and moved onto their line; then cut down to hub thickness under the bell
parts_ = []
for (p0, p1) in struts:
    L = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    w, f_, t = STRUT_W / 2, STRUT_FT, STRUT_WEB / 2
    b = extrude(c, poly(sketch(c, 'yz'), [(0, -w, 0), (0, w, 0), (0, w, f_), (0, t, f_), (0, t, STRUT_H), (0, -t, STRUT_H),
                                          (0, -t, f_), (0, -w, f_)]), NEW, L).bodies.item(0)
    move(c, [b], math.degrees(math.atan2(p1[1] - p0[1], p1[0] - p0[0])), (p0[0], p0[1], 0)); parts_.append(b)
    ux, uy = (p1[0] - p0[0]) / L, (p1[1] - p0[1]) / L          # wide root: trapezoid from inside the hub, hub thick
    s0, s1 = (p0[0] - 3 * ux, p0[1] - 3 * uy), (p0[0] + ROOT_L * ux, p0[1] + ROOT_L * uy)
    extrude(c, poly(sketch(c, 'xy'), [(s0[0] - uy * ROOT_W / 2, s0[1] + ux * ROOT_W / 2, 0), (s1[0] - uy * w, s1[1] + ux * w, 0),
                                      (s1[0] + uy * w, s1[1] - ux * w, 0), (s0[0] + uy * ROOT_W / 2, s0[1] - ux * ROOT_W / 2, 0)]), NEW, HUB_T)
sk = poly(sketch(c, 'xz'), [(0, 0, HUB_T), (R_LOW, 0, HUB_T), (R_LOW + STRUT_H - HUB_T + 1, 0, STRUT_H + 1), (0, 0, STRUT_H + 1)])
rc = c.features.revolveFeatures.createInput(sk.profiles.item(0), c.zConstructionAxis, CUT)
rc.setAngleExtent(False, adsk.core.ValueInput.createByReal(2 * math.pi)); rc.participantBodies = parts_
c.features.revolveFeatures.add(rc)
for _, ar in STRUTS:                   # buttress: a triangular fin outside the ring, in line with the strut
    b = extrude(c, poly(sketch(c, 'xz'), [(r_in + RING_T - 0.5, 0, 0), (r_in + RING_T + BUT_D, 0, 0), (r_in + RING_T - 0.5, 0, top)]),
                NEW, sym=BUT_T).bodies.item(0)
    move(c, [b], ar, (0, 0, 0))
sk = poly(sketch(c, 'xz'), [(r_in + RING_T - 0.3, 0, top), (r_in + RING_T + TOP_LIP, 0, top), (r_in + RING_T - 0.3, 0, top - TOP_LIP)])
li = c.features.revolveFeatures.createInput(sk.profiles.item(0), c.zConstructionAxis, NEW)
li.setAngleExtent(True, adsk.core.ValueInput.createByReal(math.radians(ARC_HALF)))
c.features.revolveFeatures.add(li)    # top lip
hub = extrude(c, circle(sketch(c, 'xy'), (0, 0, 0), 2 * HUB_R), NEW, HUB_T).bodies.item(0)
jb = extrude(c, poly(sketch(c, 'xy'), [(r_in - JB_D, -JB_W / 2, 0), (r_in + RING_T / 2, -JB_W / 2, 0),
                                       (r_in + RING_T / 2, JB_W / 2, 0), (r_in - JB_D, JB_W / 2, 0)]), NEW, JB_H).bodies.item(0)
combine(c, ring, [b for b in c.bRepBodies if b != ring], JOIN)
guard = c.bRepBodies.item(0)
# lattice: a diagonal grid of 45 deg bars below the solid band (diamonds on staggered rows, clipped flat at the band
# and the flange, so the openings run right up to both), skipping strut joints, the joint block and the ring ends
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
avoid = [(math.radians(ar) * r_mid, STRUT_W / 2 + BUT_T + 2) for _, ar in STRUTS] + [(0.0, JB_W / 2 + 1)]
for row in range(int((lat1 - lat0) // (pitch / 2)) + 2):
    zc = lat0 + row * pitch / 2
    for k in range(-int(t_max // pitch) - 1, int(t_max // pitch) + 2):
        tc = k * pitch + (pitch / 2 if row % 2 else 0.0)
        if abs(tc) > t_max or any(abs(tc - t0) < hb / 2 + d for t0, d in avoid):
            continue
        pts = clip(clip([(tc, zc - hb / 2), (tc + hb / 2, zc), (tc, zc + hb / 2), (tc - hb / 2, zc)], lat0, True), lat1, False)
        if len(pts) < 3 or max(p[1] for p in pts) - min(p[1] for p in pts) < 2.5:
            continue                   # sliver: not worth a cut
        phi = math.degrees(tc / r_mid)
        b = extrude(c, poly(sketch(c, 'yz'), [(0, t - tc, z) for t, z in pts]), NEW, sym=4 * RING_T + 2).bodies.item(0)
        m = adsk.core.Matrix3D.create(); m.translation = adsk.core.Vector3D.create(cm(r_in + RING_T / 2), 0, 0)
        R = adsk.core.Matrix3D.create(); R.setToRotation(math.radians(phi), adsk.core.Vector3D.create(0, 0, 1), adsk.core.Point3D.create(0, 0, 0))
        m.transformBy(R)               # out to the ring, then round to phi (one move: an identity move is refused)
        coll = adsk.core.ObjectCollection.create(); coll.add(b)
        mi = c.features.moveFeatures.createInput2(coll); mi.defineAsFreeMove(m); c.features.moveFeatures.add(mi)
        cutters.append(b)
combine(c, guard, cutters, CUT)
# hub holes: circlip hole + 3 kidney slots (annular sector r PAT_R0..PAT_R1 widened by the hole radius, +-TOL deg)
sk = sketch(c, 'xy', -1)
P = lambda r, a: pt(sk, r * math.cos(a), r * math.sin(a), -1)
sk.sketchCurves.sketchCircles.addByCenterRadius(pt(sk, 0, 0, -1), cm(CENTER_D / 2))
A, Ln = sk.sketchCurves.sketchArcs, sk.sketchCurves.sketchLines
ri_, ro_ = PAT_R0 - HOLE_D / 2, PAT_R1 + HOLE_D / 2
for k in range(3):
    a = math.radians(HOLE_ANG0 + 120 * k); d = math.radians(HOLE_ANG_TOL) + (HOLE_D / 2) / PAT_R0
    A.addByCenterStartSweep(pt(sk, 0, 0, -1), P(ri_, a - d), 2 * d)
    A.addByCenterStartSweep(pt(sk, 0, 0, -1), P(ro_, a - d), 2 * d)
    Ln.addByTwoPoints(P(ri_, a - d), P(ro_, a - d)); Ln.addByTwoPoints(P(ri_, a + d), P(ro_, a + d))
extrude(c, sk, CUT, HUB_T + 2, body=guard)
jx, jz = r_in - JB_D / 2, JB_H / 2
extrude(c, circle(sketch(c, 'xz'), (jx, 0, jz), SCREW_D), CUT, sym=JB_W + 2, body=guard)   # M2.5 x 12 cross bolt, head on the left
ins = extrude(c, circle(sketch(c, 'xz'), (jx, 0, jz), INSERT_D), NEW, -(INSERT_L + 0.5)).bodies.item(0)   # insert in the right half (-y)
combine(c, guard, [ins], CUT)
assert c.bRepBodies.count == 1 and guard.lumps.count == 1, 'guard is not one solid before the split'
whole_cm3 = guard.physicalProperties.volume
sp = c.features.splitBodyFeatures.createInput(guard, c.xZConstructionPlane, True)
c.features.splitBodyFeatures.add(sp)
assert c.bRepBodies.count == 2, 'guard did not split in two'
halves = {}
for b in c.bRepBodies:
    side = 'left' if b.boundingBox.maxPoint.y > 0.01 else 'right'   # looking out along the arm: +y is left
    b.name = f'propeller guard {side}'; halves[side] = b

OUT = os.path.join(HERE, '..', 'Huntsman', 'Printed Parts', 'Propeller Guard')
os.makedirs(OUT, exist_ok=True)
for f in ('arm-clamp.step', 'arm-clamp.stl', 'arm-clamp.gcode.3mf'):   # the old arm-tube clamp is gone
    if os.path.exists(os.path.join(OUT, f)):
        os.remove(os.path.join(OUT, f))
em = des.exportManager
res = {'strut_buckling_sf': round(P_cr / F_strut, 2), 'strut_load_N': round(F_strut, 1), 'strut_len_mm': round(L1, 1),
       'ring_r_in': r_in, 'ring_z': [0, top], 'lattice_band': [round(lat0, 1), round(lat1, 1)], 'diamonds': len(cutters),
       'ring_end_to_neighbour_prop_mm': round(d_end - PROP_D / 2, 1), 'whole_cm3': round(whole_cm3, 2)}
em.execute(em.createSTEPExportOptions(os.path.join(OUT, 'propeller-guard.step'), c))   # both halves, local coords (assembly, FEA)
for side in ('left', 'right'):
    bd = halves[side]
    assert bd.lumps.count == 1, f'{side} half is not one solid'
    bb = bd.boundingBox
    sp_ = [round((bb.maxPoint.x - bb.minPoint.x) * 10, 1), round((bb.maxPoint.y - bb.minPoint.y) * 10, 1)]
    assert max(sp_) <= BED - 2 * BED_MARGIN, f'{side} half {sp_} mm does not fit the bed'
    o = em.createSTLExportOptions(bd, os.path.join(OUT, f'propeller-guard-{side}.stl'))
    o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
    em.execute(o)
    res[f'propeller-guard-{side}'] = {'cm3': round(bd.physicalProperties.volume, 2), 'print_span_mm': sp_}
des.activateRootComponent(); app.userInterface.activeSelections.clear()
for s in c.sketches:
    s.isVisible = False
for p in c.constructionPlanes:
    p.isLightBulbOn = False
vp = app.activeViewport
cam = vp.camera; cam.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation; cam.isFitView = True; vp.camera = cam
vp.saveAsImageFile(os.path.join(OUT, 'propeller-guard-preview.png'), 1200, 800)
res
