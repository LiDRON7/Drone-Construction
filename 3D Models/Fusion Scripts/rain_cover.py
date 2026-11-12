# Rain covers for Widow and Huntsman. Runs inside Fusion via the add-in:
#   python3 run_in_fusion.py execute_code -code rain_cover.py
# Saves STEP/STL/previews into ../Widow/Printed Parts/Rain Cover/ and ../Huntsman/Printed Parts/Rain Cover/
# (HERE is set by run_in_fusion.py).
#
# Goal: fly, land and sit in light to moderate rain. Not immersion, not a pressure wash. Nothing here is tested yet,
# so these are rain covers, not waterproofing.
# How: shed the water, don't seal it out. Each cover is a roof with an open bottom, so heat, battery gas and
# condensation get out the way the air gets in.
#   canopy  - one per drone. A thin hat over everything on the top plate (Pixhawk, radios, GPS module, and on
#             Huntsman the Pi). Its skirt hangs past the plate edge. Only the GPS antenna pokes out of the roof.
#             It rests on 3 arms and a velcro strap round each of those arms holds it: no screws, no new holes.
#   collar  - TPU ring that grips the GPS antenna and lies on the roof over the antenna hole. Same on both drones.
#   visor   - strip stuck on the camera cradle's top with VHB tape, 20 mm out over the lenses. Same on both drones.
#   ESC cover - a roof over each ESC, saddled on the arm with skirts down both sides, held by 2 zip ties. The ends
#             (Widow) or the bottom (Huntsman) stay open so the prop wash still cools the ESC.
#   motor cap - a thin disc under each prop with a rim down round the top of the motor bell, so rain can't fall
#             straight into the bell's holes. It spins with the prop: see the README before you fly with them.
#   interface-box cover (Huntsman) - sheet between the interface-box plate and the box, with walls down round the
#             box. The 4 rear clamp screws hold it. Open at the bottom: the cables drop out and make a drip loop.
# Not printed (see the READMEs): conformal coat on the ESCs and power board, heat-shrink on the step-downs, tape on
# plugs, drip loops. The LiDAR is sealed by Velodyne; the batteries stay open on purpose (heat, gas).
# Canopy coords: x forward, y left, z up from the TOP FACE of the top plate, origin at the frame centre.
# All mm. "EST" = not measured yet: measure it, change the number, print in PLA first.
T = 1.2                      # shell wall: 3 lines of a 0.4 nozzle
ROOF = 65.0                  # inside height over the plate: the GPS module's top (48 + 12.9, gps_mast.py) + 4
ANT_D, HOLE_D = 27.5, 33.5   # helical antenna; roof hole (3 mm slack each side: the plate sizes are EST)
TAB_L, TAB_T = 18.5, 2.4                    # strap block on the skirt: how far out, floor that sits on the arm
STRAP_W, STRAP_T = 14.0, 3.5                # tunnel through it for a 12 mm velcro strap
MAX_LEAN = 52.0              # deg from vertical the wall may lean: it prints roof-down with no supports
BED, BED_MARGIN = 256.0, 3.0  # Bambu Lab X2D
ASA, TPU_D = 1.07, 1.21      # g/cm3
# per drone: r_in = inside radius at the plate (plate tip radius + slack), drop = skirt below the plate top,
# arm_top = arm top below the plate top (the strap blocks sit on it), band = straight wall up to here, then it leans
# in to the roof circle (centre roof_c, radius roof_r). tabs = arms that carry a strap block (deg, 0 = forward,
# counter-clockwise from above), tab_w = block width = the arm's width, so it sits on the arm and the strap lies flat. notches = (deg, width, top) slots up from the skirt edge, for the SiK antenna.
# parts = what has to fit inside: (x0, y0, x1, y1, z0, z1), numbers from full_drone.py / huntsman_full_drone.py.
DRONES = {
    'Widow': dict(
        plate_r=91.0, r_in=96.0,              # EST plate (photo); the SiK overhangs the lobe tip, hence +5
        drop=10.1, arm_top=11.12,             # plate 2 + spacers 9.12 (measured); skirt stops 1 mm above the arms
        band=17.0, roof_c=(-8.0, -13.0), roof_r=57.0, ant=(-13.2, -30.5),
        tab_w=24.0, tabs=(30, 150, 270),      # arms 2, 4 and 6: arm 1 has the receiver's leads, arm 3 the SiK antenna
        notches=((90, 12.0, 12.0),),          # SiK antenna out along arm 3
        parts={'Pixhawk': (-36.4, -7.7, 36.4, 35.7, 0, 37), 'SiK radio': (-14, 39.5, 14, 92.5, 0, 10.7),
               'R9DS receiver': (24.5, -40, 67.5, -16, 0, 15), 'GPS cradle': (-43.45, -51.45, 17.05, -9.55, 45, 56),
               'GPS module': (-40.13, -48.15, 13.73, -12.85, 48, 60.9)},
        # ESC inside the open-top arm (24 wide, photos: EST): the roof closes the channel over it, ends open
        esc=dict(length=44.0, inner_w=24.6, skirt=10.0, tube=None),
        motor=dict(bell_d=27.47, hole_d=5.2, bolts=())),                # measured bell; 5 mm prop shaft
    'Huntsman': dict(
        plate_r=101.0, r_in=103.0,            # EST: plate corners in the online Tarot model
        drop=3.6, arm_top=4.6,                # EST (model): arm tube top 3 mm under the plate's underside
        band=24.0, roof_c=(-28.0, 19.0), roof_r=88.0, ant=(-55.0, 40.0),
        tab_w=16.0, tabs=(90, -30, -150),     # EST: 16 mm arm tubes
        notches=((33, 12.0, 12.0),),          # SiK antenna, pointing forward (EST: where the radio really ends up)
        parts={'Pixhawk': (-37.5, -23, 37.5, 23, 0, 37), 'Raspberry Pi 5': (-42.5, -88, 42.5, -32, 0, 20),
               'SiK radio': (8.5, 41, 61.5, 69, 0, 10.7), 'R9DS receiver': (43, -63.5, 67, -20.5, 0, 15),
               '5 V step-down': (-65, -63, -45, -47, 0, 8), 'PM02D': (-65, -8, -45, 8, 0, 10),
               'GPS cradle': (-85.25, 19.05, -24.75, 60.95, 45, 56), 'GPS module': (-81.93, 22.35, -28.07, 57.65, 48, 60.9)},
        # EST, all of it: I don't have the AK8's size or where the ESCs sit yet. Drawn for an ESC of 68 x 34 x 18
        # zip-tied under the 16 mm arm tube: roof on the tube, skirts down past the ESC, end flaps either side of the tube.
        esc=dict(length=72.0, inner_w=36.0, skirt=36.0, tube=16.0),
        motor=dict(bell_d=46.0, hole_d=8.0, bolts=(2, 12.0, 3.2))),      # EST: bell from the online model; prop bolts 2 x M3, 12 apart
}
COLLAR_ID, COLLAR_WALL, COLLAR_H, FLANGE_D, FLANGE_T = ANT_D - 0.5, 2.0, 8.0, 48.0, 1.6   # stretches onto the antenna
# visor: x = 0 is the cradle's top front edge (Ft in landing_leg.py / huntsman_payload.py). Print it upside down.
VIS_W, VIS_BACK, VIS_OUT, VIS_LIP = 104.0, 28.0, 20.0, 3.0   # width; length stuck to the cradle; reach over the lenses; drip lip
VIS_SLOT = (6.0, 22.0, -2.7)   # the cradle's USB-C windows: |y| from..to, front edge x. Slots run to the back edge.
CAM_LENS_DROP, CAM_TILT, CAM_VFOV = 24.05, 10.0, 56.0   # lens below the cradle top (landing_leg.py), tilt down, widest view
CAP_T, CAP_RIM, CAP_GAP = 1.0, 4.0, 0.5      # motor cap: disc under the prop, rim down round the bell, gap to the bell
# interface-box cover: numbers from huntsman_payload.py (box 75 across x 53 long x 28 EST, centred at x -102.5; rear
# clamp screws at x -112 +- 6, y +- 30; pinch screws at y +- 21.1; plate half width 40, carrier's back rail at x -70.9)
IFB_L, IFB_W, IFB_H, IFB_X = 53.0, 75.0, 28.0, -102.5
IFB_SLACK, IFB_DROP = 1.0, 4.0               # room round the box; walls reach this far below it (velcro included)
IFB_SCREWS, IFB_PINCH, SCREW_D = [(-118.0, 30.0), (-106.0, 30.0)], (-112.0, 21.1), 2.9

import math
# self-checks (plain numbers, before Fusion draws anything)
def section(d, h):               # canopy inside at height h: centre and radius
    f = max(0.0, (h - d['band']) / (ROOF - d['band']))
    return (f * d['roof_c'][0], f * d['roof_c'][1]), d['r_in'] + f * (d['roof_r'] - d['r_in'])
for name, d in DRONES.items():
    assert d['r_in'] >= d['plate_r'] + 2, f'{name}: skirt too tight on the plate'
    lean = math.degrees(math.atan((abs(d['r_in'] - d['roof_r']) + math.hypot(*d['roof_c'])) / (ROOF - d['band'])))
    assert lean <= MAX_LEAN, f'{name}: wall leans {lean:.0f} deg, needs supports'
    d['lean'] = round(lean, 1)
    for part_, (x0, y0, x1, y1, z0, z1) in d['parts'].items():
        assert z1 + 2 <= ROOF, f'{name}: {part_} touches the roof'
        for h in (z0, z1):
            (cx, cy), r = section(d, h)
            gap = r - max(math.hypot(x - cx, y - cy) for x in (x0, x1) for y in (y0, y1))
            assert gap >= 1.5, f'{name}: {part_} is {-gap:.1f} mm outside the canopy at {h} mm up'
    assert math.hypot(d['ant'][0] - d['roof_c'][0], d['ant'][1] - d['roof_c'][1]) + FLANGE_D / 2 + 3 <= d['roof_r'], f'{name}: antenna collar runs off the roof'
    assert all(abs((a - n[0] + 180) % 360 - 180) >= 20 for a in d['tabs'] for n in d['notches']), f'{name}: a notch cuts a strap block'
    assert -d['arm_top'] + TAB_T + STRAP_T + 2.5 + TAB_L + T <= d['band'], f'{name}: strap block taller than the straight wall'
    assert d['esc']['tube'] is None or d['esc']['inner_w'] - d['esc']['tube'] >= 8, f'{name}: ESC cover end flaps too narrow'
assert IFB_W / 2 + IFB_SLACK + T <= 40.0 and IFB_X + IFB_L / 2 + IFB_SLACK + T <= -70.9 - 1, 'interface-box cover wider than its plate or into the battery rail'
assert HOLE_D >= ANT_D + 4 and FLANGE_D >= HOLE_D + 2 * 3 + 6, 'collar flange must cover the hole wherever the canopy ends up'
# the visor must stay out of the picture: top edge of the view, from the lens, at the lip's bottom corner
_top_ray = math.tan(math.radians(CAM_VFOV / 2 - CAM_TILT)) * (VIS_OUT + 3)
assert CAM_LENS_DROP - VIS_LIP - _top_ray >= 5, 'visor shows in the camera view'

import adsk.core, adsk.fusion, os
cm = lambda mm: mm / 10.0
for d_ in [d_ for d_ in app.documents if d_.name.startswith('Rain cover')]:
    d_.close(False)
doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
doc.name = 'Rain cover'
des = adsk.fusion.Design.cast(app.activeProduct)
des.designType = adsk.fusion.DesignTypes.DirectDesignType
root = des.rootComponent
NEW, JOIN, CUT = (adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
                  adsk.fusion.FeatureOperations.JoinFeatureOperation,
                  adsk.fusion.FeatureOperations.CutFeatureOperation)
V = lambda mm: adsk.core.ValueInput.createByReal(cm(mm))

def part(name):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ

def sketch(comp, plane, off=0.0):
    base = {'xy': comp.xYConstructionPlane, 'xz': comp.xZConstructionPlane}[plane]
    if off:
        pi = comp.constructionPlanes.createInput(); pi.setByOffset(base, V(off)); base = comp.constructionPlanes.add(pi)
    return comp.sketches.add(base)

def pt(sk, x, y, z):
    return sk.modelToSketchSpace(adsk.core.Point3D.create(cm(x), cm(y), cm(z)))

def circle(sk, c, dia):
    sk.sketchCurves.sketchCircles.addByCenterRadius(pt(sk, *c), cm(dia / 2)); return sk

def poly(sk, pts):
    L = sk.sketchCurves.sketchLines
    for i in range(len(pts)):
        L.addByTwoPoints(pt(sk, *pts[i]), pt(sk, *pts[(i + 1) % len(pts)]))
    return sk

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

def combine(comp, target, tools, op):
    coll = adsk.core.ObjectCollection.create()
    for t in tools:
        coll.add(t)
    ci = comp.features.combineFeatures.createInput(target, coll); ci.operation = op
    comp.features.combineFeatures.add(ci)

def move(comp, body, M):
    coll = adsk.core.ObjectCollection.create(); coll.add(body)
    mi = comp.features.moveFeatures.createInput2(coll); mi.defineAsFreeMove(M); comp.features.moveFeatures.add(mi)

def rot(axis, deg):
    M = adsk.core.Matrix3D.create()
    M.setToRotation(math.radians(deg), adsk.core.Vector3D.create(*axis), adsk.core.Point3D.create(0, 0, 0)); return M

def canopy(name, d):
    occ = part(f'{name} rain canopy'); c = occ.component
    r_o, z0 = d['r_in'] + T, -d['drop']
    body = extrude(c, circle(sketch(c, 'xy', z0), (0, 0, z0), 2 * r_o), NEW, d['band'] - z0).bodies.item(0)
    li = c.features.loftFeatures.createInput(JOIN)
    li.loftSections.add(circle(sketch(c, 'xy', d['band']), (0, 0, d['band']), 2 * r_o).profiles.item(0))
    li.loftSections.add(circle(sketch(c, 'xy', ROOF + T), (*d['roof_c'], ROOF + T), 2 * (d['roof_r'] + T)).profiles.item(0))
    li.isSolid = True; li.participantBodies = [body]
    c.features.loftFeatures.add(li)
    if c.bRepBodies.count > 1:
        combine(c, c.bRepBodies.item(0), list(c.bRepBodies)[1:], JOIN)
    body = c.bRepBodies.item(0)
    bottom = min(body.faces, key=lambda f: f.boundingBox.maxPoint.z)          # the open side
    faces = adsk.core.ObjectCollection.create(); faces.add(bottom)
    si = c.features.shellFeatures.createInput(faces, False); si.insideThickness = V(T)
    c.features.shellFeatures.add(si)
    body = c.bRepBodies.item(0)
    extrude(c, circle(sketch(c, 'xy', ROOF - 1), (*d['ant'], ROOF - 1), HOLE_D), CUT, T + 2, body=body)   # antenna hole
    # strap blocks: floor on the arm, tunnel for the strap, then a 45 deg ramp back to the wall (prints roof-down)
    zt = -d['arm_top']; x0, x1 = d['r_in'] + 0.3, r_o + TAB_L
    z_end = zt + TAB_T + STRAP_T + 2.5
    for a in d['tabs']:
        blk = extrude(c, poly(sketch(c, 'xz'), [(x0, 0, zt), (x1, 0, zt), (x1, 0, z_end), (x0, 0, z_end + (x1 - x0))]), NEW, sym=d['tab_w']).bodies.item(0)
        extrude(c, poly(sketch(c, 'xz'), [(r_o + 1.5, 0, zt + TAB_T), (r_o + 1.5 + STRAP_W, 0, zt + TAB_T),
                                           (r_o + 1.5 + STRAP_W, 0, zt + TAB_T + STRAP_T), (r_o + 1.5, 0, zt + TAB_T + STRAP_T)]),
                CUT, sym=d['tab_w'] + 2, body=blk)
        move(c, [b for b in c.bRepBodies if b != body][-1], rot((0, 0, 1), a))
        combine(c, body, [[b for b in c.bRepBodies if b != body][-1]], JOIN)
        body = c.bRepBodies.item(0)
    for a, w, top in d['notches']:
        ca, sa = math.cos(math.radians(a)), math.sin(math.radians(a))
        P = lambda r, s: (r * ca - s * sa, r * sa + s * ca, z0 - 1)
        extrude(c, poly(sketch(c, 'xy', z0 - 1), [P(d['r_in'] - 4, -w / 2), P(r_o + 4, -w / 2), P(r_o + 4, w / 2), P(d['r_in'] - 4, w / 2)]),
                CUT, top - z0 + 1, body=body)
    body = c.bRepBodies.item(0); body.name = 'rain canopy'
    return occ, body

def collar():
    occ = part('antenna collar'); c = occ.component
    body = extrude(c, circle(sketch(c, 'xy'), (0, 0, 0), FLANGE_D), NEW, FLANGE_T).bodies.item(0)
    extrude(c, circle(sketch(c, 'xy'), (0, 0, 0), COLLAR_ID + 2 * COLLAR_WALL), JOIN, COLLAR_H, body=body)
    extrude(c, circle(sketch(c, 'xy', -1), (0, 0, -1), COLLAR_ID), CUT, COLLAR_H + 2, body=body)
    body = c.bRepBodies.item(0); body.name = 'antenna collar'
    return occ, body

def visor():     # drawn as it prints: the face that sticks to the cradle is up, the drip lip grows up
    occ = part('camera visor'); c = occ.component
    w = VIS_W / 2
    body = extrude(c, poly(sketch(c, 'xy'), [(-VIS_BACK, -w, 0), (VIS_OUT, -w, 0), (VIS_OUT, w, 0), (-VIS_BACK, w, 0)]), NEW, T).bodies.item(0)
    extrude(c, poly(sketch(c, 'xy', T), [(VIS_OUT - T, -w, T), (VIS_OUT, -w, T), (VIS_OUT, w, T), (VIS_OUT - T, w, T)]), JOIN, VIS_LIP, body=body)
    sk = sketch(c, 'xy', -1)
    for s in (-1, 1):
        y0, y1 = sorted((s * VIS_SLOT[0], s * VIS_SLOT[1]))
        poly(sk, [(-VIS_BACK - 1, y0, -1), (VIS_SLOT[2], y0, -1), (VIS_SLOT[2], y1, -1), (-VIS_BACK - 1, y1, -1)])
    extrude(c, sk, CUT, T + 2, body=body)                    # the USB-C cable comes up through one of these
    body = c.bRepBodies.item(0); body.name = 'camera visor'
    return occ, body

def esc_cover(name, e):   # drawn as it prints: roof on the bed, skirts up. x runs along the arm.
    occ = part(f'{name} ESC cover'); c = occ.component
    L, W, S = e['length'], e['inner_w'], e['skirt']; wo = W / 2 + T
    body = extrude(c, poly(sketch(c, 'xy'), [(-L / 2, -wo, 0), (L / 2, -wo, 0), (L / 2, wo, 0), (-L / 2, wo, 0)]), NEW, T + S).bodies.item(0)
    extrude(c, poly(sketch(c, 'xy', T), [(-L / 2 - 1, -W / 2, T), (L / 2 + 1, -W / 2, T), (L / 2 + 1, W / 2, T), (-L / 2 - 1, W / 2, T)]), CUT, S + 1, body=body)
    if e['tube']:             # end flaps either side of the arm tube: they keep the roof centred and the spray out
        sk = sketch(c, 'xy', T)
        for sx in (-1, 1):
            for sy in (-1, 1):
                x0, x1 = sorted((sx * (L / 2 - T), sx * L / 2)); y0, y1 = sorted((sy * (e['tube'] / 2 + 0.2), sy * (W / 2 + 0.5)))
                poly(sk, [(x0, y0, T), (x1, y0, T), (x1, y1, T), (x0, y1, T)])
        extrude(c, sk, JOIN, S, body=body)
    body = c.bRepBodies.item(0); body.name = 'ESC cover'
    return occ, body

def motor_cap(name, m):   # drawn as it sits: disc on the bell's top (z = 0), rim hanging down round the bell
    occ = part(f'{name} motor cap'); c = occ.component
    r_in = m['bell_d'] / 2 + CAP_GAP
    body = extrude(c, circle(sketch(c, 'xy', -CAP_RIM), (0, 0, -CAP_RIM), 2 * (r_in + T)), NEW, CAP_RIM + CAP_T).bodies.item(0)
    extrude(c, circle(sketch(c, 'xy', -CAP_RIM - 1), (0, 0, -CAP_RIM - 1), 2 * r_in), CUT, CAP_RIM + 1, body=body)   # room for the bell
    sk = circle(sketch(c, 'xy', -1), (0, 0, -1), m['hole_d'])
    if m['bolts']:
        n, sp, dia = m['bolts']
        for k in range(n):
            a = 2 * math.pi * k / n
            circle(sk, (sp / 2 * math.cos(a), sp / 2 * math.sin(a), -1), dia)
    extrude(c, sk, CUT, CAP_T + 2, body=c.bRepBodies.item(0))
    body = c.bRepBodies.item(0); body.name = 'motor cap'
    return occ, body

def ifb_cover():          # drawn as it prints: the sheet that goes against the plate is on the bed, walls up. Flight x, y.
    occ = part('interface box cover'); c = occ.component
    x0, x1, w = IFB_X - IFB_L / 2 - IFB_SLACK, IFB_X + IFB_L / 2 + IFB_SLACK, IFB_W / 2 + IFB_SLACK
    box = lambda g, z: [(x0 - g, -w - g, z), (x1 + g, -w - g, z), (x1 + g, w + g, z), (x0 - g, w + g, z)]
    body = extrude(c, poly(sketch(c, 'xy'), box(T, 0)), NEW, T + IFB_H + IFB_DROP).bodies.item(0)
    extrude(c, poly(sketch(c, 'xy', T), box(0, T)), CUT, IFB_H + IFB_DROP + 1, body=body)
    sk = sketch(c, 'xy', -1)
    for sy in (-1, 1):
        for x, y in IFB_SCREWS:
            circle(sk, (x, sy * y, -1), SCREW_D)
        circle(sk, (IFB_PINCH[0], sy * IFB_PINCH[1], -1), 6.0)      # the pinch screws stay reachable
    extrude(c, sk, CUT, T + 2, body=body)
    body = c.bRepBodies.item(0); body.name = 'interface box cover'
    return occ, body

em = des.exportManager
res = {}
def save(folder, fname, occ, body, density, flip=False):
    out = os.path.join(HERE, '..', folder, 'Printed Parts', 'Rain Cover'); os.makedirs(out, exist_ok=True)
    assert occ.component.bRepBodies.count == 1 and body.lumps.count == 1, f'{fname} is not one solid'
    em.execute(em.createSTEPExportOptions(os.path.join(out, fname + '.step'), occ.component))   # as it sits on the drone
    if flip:
        move(occ.component, body, rot((1, 0, 0), 180)); body = occ.component.bRepBodies.item(0)
    o = em.createSTLExportOptions(body, os.path.join(out, fname + '.stl'))                      # as it prints
    o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
    em.execute(o)
    bb = body.boundingBox
    box = [round((bb.maxPoint.x - bb.minPoint.x) * 10, 1), round((bb.maxPoint.y - bb.minPoint.y) * 10, 1), round((bb.maxPoint.z - bb.minPoint.z) * 10, 1)]
    assert max(box[:2]) <= BED - 2 * BED_MARGIN and box[2] <= 256, f'{fname} does not fit the X2D'
    vol = body.physicalProperties.volume
    res[f'{folder} {fname}'] = {'volume_cm3': round(vol, 2), 'solid_g': round(vol * density, 1), 'bbox_mm': box}
    if flip:
        move(occ.component, body, rot((1, 0, 0), 180))       # back the right way up for the picture

def picture(folder, show):
    for o in root.occurrences:
        o.isLightBulbOn = o in show
        for s in o.component.sketches:
            s.isVisible = False
        for p in o.component.constructionPlanes:
            p.isLightBulbOn = False
    vp = app.activeViewport
    cam = vp.camera; cam.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation; cam.isFitView = True; vp.camera = cam
    vp.saveAsImageFile(os.path.join(HERE, '..', folder, 'Printed Parts', 'Rain Cover', 'rain-cover-preview.png'), 1200, 800)

col_occ, col = collar()
vis_occ, vis = visor()
save('Widow', 'antenna-collar', col_occ, col, TPU_D)
save('Widow', 'camera-visor', vis_occ, vis, ASA)
ifb_occ, ifb = ifb_cover()
save('Huntsman', 'interface-box-cover', ifb_occ, ifb, ASA)
shown = {}
for name, d in DRONES.items():
    occ, body = canopy(name, d)
    save(name, 'rain-canopy', occ, body, ASA, flip=True)
    res[f'{name} rain-canopy']['wall_lean_deg'] = d['lean']
    e_occ, e_body = esc_cover(name, d['esc']); save(name, 'esc-cover', e_occ, e_body, ASA)
    m_occ, m_body = motor_cap(name, d['motor']); save(name, 'motor-cap', m_occ, m_body, ASA, flip=True)
    for o_, x_ in ((e_occ, 150), (m_occ, 230)):                   # beside the canopy, for the pictures only
        M_ = adsk.core.Matrix3D.create(); M_.translation = adsk.core.Vector3D.create(cm(x_), cm(150), 0); o_.transform = M_
    shown[name] = [occ, e_occ, m_occ] + ([ifb_occ] if name == 'Huntsman' else [])
M_ = adsk.core.Matrix3D.create(); M_.translation = adsk.core.Vector3D.create(cm(330), cm(-130), 0); ifb_occ.transform = M_
# the small parts beside the canopy, for the pictures only
T1 = adsk.core.Matrix3D.create(); T1.translation = adsk.core.Vector3D.create(cm(150), cm(-60), 0); col_occ.transform = T1
T2 = adsk.core.Matrix3D.create(); T2.translation = adsk.core.Vector3D.create(cm(150), cm(40), 0); vis_occ.transform = T2
des.activateRootComponent(); app.userInterface.activeSelections.clear()
for name, occs in shown.items():
    picture(name, occs + [col_occ, vis_occ])
for o in root.occurrences:
    o.isLightBulbOn = True
res
