# Huntsman (Tarot 680 PRO) 13" propeller guard + arm clamp. Runs inside Fusion via the add-in:
#   python3 run_in_fusion.py execute_code -code huntsman_propeller_guard.py
# Saves STEP/STL/preview into ../Huntsman/Printed Parts/Propeller Guard/ (HERE is set by run_in_fusion.py).
#
# The guard clamps to the 16 mm arm tube just inside the motor mount, so the motor, its screws and its alignment
# stay untouched. Guard: an open ring around the outer 190 deg of the prop, 4 triangular struts back to a pad on the
# clamp. Clamp: a U over the tube, closed under it by an M2.5 cross bolt; the guard pad screws onto it.
#   ring   - solid band at the prop tips (where it stops things), diamond lattice below it (45 deg sides print
#            without supports), a narrow inward flange on the bed for in-plane stiffness. Wall = 3 lines of 0.43.
#   struts - solid triangles printed flat: the most buckling stiffness per gram that prints without supports.
#            Sized so the whole drone's weight pushing sideways on one guard does not buckle them (Euler,
#            fixed-pinned, safety factor 1.5).
# Print the guard flat (as built: strut side on the bed), 3 walls, 15 % infill; the clamp on its end (rod hole up).
# Local coords: motor axis at the origin, x outward along the arm, z = 0 the top of the Tarot motor mount, mm.
# Frame numbers are from the online Tarot 680 PRO model (EST): measure PROP_Z and TUBE_Z on the drone.
PROP_D, PROP_Z = 330.2, 25.0       # 13 in (1355) prop; EST height of the prop tips above the motor mount top
TIP_GAP, RING_T = 8.0, 1.3         # tip to ring wall; ring wall (3 extrusion lines)
BAND_UP, BAND_DN = 6.0, 4.0        # solid band from PROP_Z - BAND_DN to PROP_Z + BAND_UP (top of the guard)
ARC_HALF = 95.0                    # deg each side of outward: neighbour rings would meet near 108 deg
FL_W, FL_T = 4.0, 1.6              # inward flange along the ring bottom (on the bed)
Z0 = 6.0                           # bed plane: struts and ring bottom, this far above the motor mount top
STRUT_ANGS = (35.0, -35.0, 80.0, -80.0)
STRUT_B, STRUT_H = 8.5, 6.8        # strut triangle: base width, height (sized by the buckling check below)
DIA_WEB = 2.2                      # web between lattice diamonds (measured along the ring)
MOTOR_R, MOUNT_HW, MOUNT_IN = 23.0, 21.8, 72.7   # 4108 bell radius; motor mount half width; mount's inboard end (EST model)
ARM_R, NEIGHBOUR = 343.0, 343.0    # motor distance from the frame centre; between neighbouring motors (hexagon)
TUBE_D, TUBE_Z, FIT = 16.0, -11.0, 0.3   # arm tube (EST model)
CLAMP_DX, CL_L, CL_LEG, CL_BELOW = 88.0, 16.0, 5.0, 6.5   # clamp centre inboard of the motor axis; length; leg; below the tube
PAD_L, PAD_W, PAD_T = 26.0, 24.0, 3.0
INSERT_D, INSERT_L, SCREW_D = 3.6, 4.0, 2.9      # M2.5 heat-set inserts (same EST as Widow)
# strength self-check
AUW_KG, G_SIDE, E_MPA, K_END, SF_MIN = 3.9, 1.0, 1500.0, 0.7, 1.5   # E: PETG with 15 % infill, conservative

import adsk.core, adsk.fusion, math, os
cm = lambda mm: mm / 10.0
r_in = PROP_D / 2 + TIP_GAP
r_mid = r_in + RING_T / 2
top = PROP_Z + BAND_UP
band0 = PROP_Z - BAND_DN                       # lattice below this, solid band above
lat0, lat1 = Z0 + FL_T + 1.0, band0             # lattice band
hb = lat1 - lat0                               # diamond height = width (45 deg sides)
pad = (-CLAMP_DX, 0.0)
struts = [((r_in - 0.8) * math.cos(math.radians(a)), (r_in - 0.8) * math.sin(math.radians(a))) for a in STRUT_ANGS]   # end just inside the wall: an angled end would poke out

def line_dist(p, a, b):                        # distance from p to segment ab
    ax, ay = a; bx, by = b; px, py = p
    t = max(0, min(1, ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / ((bx - ax) ** 2 + (by - ay) ** 2)))
    return math.hypot(ax + t * (bx - ax) - px, ay + t * (by - ay) - py)

# --- self-checks: clearances (EST geometry) and strength
assert r_in - PROP_D / 2 >= 6, 'prop tips too close to the ring'
assert Z0 >= 3 and Z0 + STRUT_H <= PROP_Z - 8, 'struts must clear the motor mount top and stay 8 mm under the prop'
assert hb >= 6, 'lattice band too short: raise PROP_Z - BAND_DN or lower Z0'
for s in struts:
    assert line_dist((0, 0), pad, s) - STRUT_B / 2 >= MOTOR_R + 4, 'a strut passes too close to the motor bell'
# neighbour: its prop and ring must stay clear of our ring's ends (neighbour centre 120 deg round from outward)
nb = (NEIGHBOUR * math.cos(math.radians(120)), NEIGHBOUR * math.sin(math.radians(120)))
end = ((r_in + RING_T) * math.cos(math.radians(ARC_HALF)), (r_in + RING_T) * math.sin(math.radians(ARC_HALF)))
d_end = math.hypot(end[0] - nb[0], end[1] - nb[1])
assert d_end - PROP_D / 2 >= 20 and d_end - (r_in + RING_T) >= 10, 'ring end too close to the neighbouring prop or ring'
# struts: a sideways bump of G_SIDE x the drone's weight on the ring's outer point, taken by the two +-35 deg struts
L35 = math.hypot(struts[0][0] - pad[0], struts[0][1] - pad[1])
cos_b = (struts[0][0] - pad[0]) / L35
F_strut = AUW_KG * 9.81 * G_SIDE / (2 * cos_b)
I_min = min(STRUT_B * STRUT_H**3 / 36, STRUT_H * STRUT_B**3 / 48)
P_cr = math.pi**2 * E_MPA * I_min / (K_END * L35) ** 2
assert P_cr / F_strut >= SF_MIN, f'struts buckle: {P_cr:.0f} N vs {F_strut:.0f} N'
cl_top = Z0 - (TUBE_Z + (TUBE_D + FIT) / 2)
assert cl_top >= INSERT_L + 1.5 and CL_LEG >= INSERT_L + 0.8, 'clamp walls too thin for the inserts'
assert CLAMP_DX - CL_L / 2 >= MOUNT_IN + 5, 'clamp runs into the motor mount'
assert PAD_W / 2 <= CL_L / 2 + 6 and PAD_L >= 2 * 6 + INSERT_D + 4, 'pad'

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

def part(name):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create()); occ.component.name = name; return occ

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

# ---- guard
g_occ = part('propeller guard'); c = g_occ.component
sk = poly(sketch(c, 'xz'), [(r_in - FL_W, 0, Z0), (r_in + RING_T, 0, Z0), (r_in + RING_T, 0, top), (r_in, 0, top),
                            (r_in, 0, Z0 + FL_T), (r_in - FL_W, 0, Z0 + FL_T)])
ri = c.features.revolveFeatures.createInput(sk.profiles.item(0), c.zConstructionAxis, NEW)
ri.setAngleExtent(True, adsk.core.ValueInput.createByReal(math.radians(ARC_HALF)))   # symmetric: angle is per side
ring = c.features.revolveFeatures.add(ri).bodies.item(0)
parts_ = []
for (sx, sy) in struts:                                       # triangular struts, built along x then turned and moved
    L = math.hypot(sx - pad[0], sy - pad[1])
    b = extrude(c, poly(sketch(c, 'yz'), [(0, -STRUT_B / 2, Z0), (0, STRUT_B / 2, Z0), (0, 0, Z0 + STRUT_H)]), NEW, L).bodies.item(0)
    move(c, [b], math.degrees(math.atan2(sy - pad[1], sx - pad[0])), (pad[0], pad[1], 0)); parts_.append(b)
pb = extrude(c, poly(sketch(c, 'xy', Z0), [(pad[0] - PAD_L / 2, -PAD_W / 2, Z0), (pad[0] + PAD_L / 2, -PAD_W / 2, Z0),
                                          (pad[0] + PAD_L / 2, PAD_W / 2, Z0), (pad[0] - PAD_L / 2, PAD_W / 2, Z0)]), NEW, PAD_T).bodies.item(0)
combine(c, ring, parts_ + [pb], JOIN)
guard = c.bRepBodies.item(0)
# lattice: diamonds through the ring wall below the solid band, skipping strut joints and the ring ends
cutters, pitch = [], hb + DIA_WEB * math.sqrt(2)
dphi = math.degrees(pitch / r_mid)
n = int((2 * ARC_HALF - 2 * math.degrees((hb / 2 + 3) / r_mid)) // dphi)
start = -(n - 1) * dphi / 2
for k in range(n):
    phi = start + k * dphi
    if any(abs(phi - a) < math.degrees((hb / 2 + STRUT_B / 2 + 2) / r_mid) for a in STRUT_ANGS):
        continue
    zc = (lat0 + lat1) / 2
    b = extrude(c, poly(sketch(c, 'yz'), [(0, 0, lat0), (0, hb / 2, zc), (0, 0, lat1), (0, -hb / 2, zc)]), NEW, sym=4 * RING_T + 2).bodies.item(0)
    m = adsk.core.Matrix3D.create(); m.translation = adsk.core.Vector3D.create(cm(r_in + RING_T / 2), 0, 0)
    R = adsk.core.Matrix3D.create(); R.setToRotation(math.radians(phi), adsk.core.Vector3D.create(0, 0, 1), adsk.core.Point3D.create(0, 0, 0))
    m.transformBy(R)                   # out to the ring, then round to phi (one move: an identity move is refused)
    coll = adsk.core.ObjectCollection.create(); coll.add(b)
    mi = c.features.moveFeatures.createInput2(coll); mi.defineAsFreeMove(m); c.features.moveFeatures.add(mi)
    cutters.append(b)
combine(c, guard, cutters, CUT)
sk = sketch(c, 'xy', Z0 - 1)
for dx in (-6, 6):
    circle(sk, (pad[0] + dx, 0, Z0 - 1), SCREW_D)
extrude(c, sk, CUT, PAD_T + STRUT_H + 2, body=guard)          # 2 x M2.5 x 8 down into the clamp's inserts
guard = c.bRepBodies.item(0); guard.name = 'propeller guard'

# ---- arm clamp (same local coords; U over the tube, cross bolt under it)
k_occ = part('arm clamp'); c = k_occ.component
yo, zb = (TUBE_D + FIT) / 2 + CL_LEG, TUBE_Z - (TUBE_D + FIT) / 2 - CL_BELOW
x0, x1 = pad[0] - CL_L / 2, pad[0] + CL_L / 2
cl = extrude(c, poly(sketch(c, 'xy', zb), [(x0, -yo, zb), (x1, -yo, zb), (x1, yo, zb), (x0, yo, zb)]), NEW, Z0 - zb).bodies.item(0)
extrude(c, circle(sketch(c, 'yz', pad[0]), (pad[0], 0, TUBE_Z), TUBE_D + FIT), CUT, sym=CL_L + 2, body=cl)
gap = (TUBE_D + FIT) / 2
extrude(c, poly(sketch(c, 'xy', zb - 1), [(x0 - 1, -gap, zb - 1), (x1 + 1, -gap, zb - 1), (x1 + 1, gap, zb - 1), (x0 - 1, gap, zb - 1)]),
        CUT, TUBE_Z - zb + 1, body=cl)                        # open under the tube: the legs pinch it
zbolt = zb + CL_BELOW / 2
extrude(c, circle(sketch(c, 'xz'), (pad[0], 0, zbolt), SCREW_D), CUT, sym=2 * yo + 2, body=cl)
ins = extrude(c, circle(sketch(c, 'xz', -yo), (pad[0], -yo, zbolt), INSERT_D), NEW, INSERT_L + 0.5).bodies.item(0)   # insert in the -y leg
combine(c, cl, [ins], CUT)
sk = sketch(c, 'xy', Z0 - INSERT_L - 1)
for dx in (-6, 6):
    circle(sk, (pad[0] + dx, 0, Z0 - INSERT_L - 1), INSERT_D)
extrude(c, sk, CUT, INSERT_L + 2, body=cl)                    # inserts for the guard screws
clamp = c.bRepBodies.item(0); clamp.name = 'arm clamp'

OUT = os.path.join(HERE, '..', 'Huntsman', 'Printed Parts', 'Propeller Guard')
os.makedirs(OUT, exist_ok=True)
em = des.exportManager
res = {'strut_buckling_sf': round(P_cr / F_strut, 2), 'strut_load_N': round(F_strut, 1), 'ring_r_in': r_in,
       'ring_z': [Z0, top], 'lattice_band': [round(lat0, 1), round(lat1, 1)], 'diamonds': len(cutters),
       'ring_end_to_neighbour_prop_mm': round(d_end - PROP_D / 2, 1)}
for name, occ, bd in (('propeller-guard', g_occ, guard), ('arm-clamp', k_occ, clamp)):
    assert occ.component.bRepBodies.count == 1 and bd.lumps.count == 1, f'{name} is not one solid'
    em.execute(em.createSTEPExportOptions(os.path.join(OUT, f'{name}.step'), occ.component))   # local coords (assembly)
    if name == 'arm-clamp':            # print standing on its end: tube hole vertical, no supports
        R = adsk.core.Matrix3D.create(); R.setToRotation(math.radians(-90), adsk.core.Vector3D.create(0, 1, 0), adsk.core.Point3D.create(0, 0, 0))
        coll = adsk.core.ObjectCollection.create(); coll.add(bd)
        mi = occ.component.features.moveFeatures.createInput2(coll); mi.defineAsFreeMove(R); occ.component.features.moveFeatures.add(mi)
        bd = occ.component.bRepBodies.item(0)
    o = em.createSTLExportOptions(bd, os.path.join(OUT, f'{name}.stl'))
    o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
    em.execute(o)
    res[name + '_cm3'] = round(bd.physicalProperties.volume, 2)
k_occ.isLightBulbOn = False
des.activateRootComponent(); app.userInterface.activeSelections.clear()
for occ in (g_occ, k_occ):
    for s in occ.component.sketches:
        s.isVisible = False
    for p in occ.component.constructionPlanes:
        p.isLightBulbOn = False
vp = app.activeViewport
cam = vp.camera; cam.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation; cam.isFitView = True; vp.camera = cam
vp.saveAsImageFile(os.path.join(OUT, 'propeller-guard-preview.png'), 1200, 800)
res
