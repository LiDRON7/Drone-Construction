# Huntsman payload carrier (Tarot 680 PRO). Runs inside Fusion via the add-in:
#   python3 run_in_fusion.py execute_code -code huntsman_payload.py
# Saves STEP/STL/preview into ../Huntsman/Printed Parts/Payload Carrier/ (HERE is set by run_in_fusion.py).
#
# One printed plate under the Tarot's two 10 mm payload rods (they hang on the frame's 4 vibration dampers), held by
# 6 printed rod clamps. Front to back it carries:
#   OAK-D Pro  - cradle at the front, looking forward and TILT deg down (same cradle as Widow's leg mount)
#   Puck LITE  - on its side: spin axis pointing forward, so it sweeps a line straight down under the drone (the
#                landing spot). Its base bolts to a wall with the 1/4-20 screw and 2 pins; its cable goes up
#                through a slot in the plate, Velodyne's 0 deg mark (opposite the cable) points straight down.
#   battery    - 6S 5200 across the drone behind the wall, 2 x 20 mm straps through the plate
#   interface box (under the plate, velcro) and the 12 V step-down for the LiDAR (on top of the plate, between the rods)
# Print coords: the plate's top (the rod side) on the bed, everything else grows up (= down in flight). x forward.
# Flight coords = (x, -y, Z_PT - z). All mm. Frame numbers come from the online Tarot 680 PRO model (EST: measure
# the real rods); LiDAR numbers from Velodyne's datasheet; battery from the PowerHobby listing.
ROD_D, ROD_Y, ROD_Z = 10.0, 30.0, -34.85    # EST (model): payload rod diameter, distance off the centre line, axis height
DAMP_Z = -46.2                              # EST (model): lowest point of the damper clamps on the rods (x 68-78)
FIT = 0.3
PL_T, Z_PT = 4.0, -48.0                     # plate thickness; flight height of its top face (under the damper clamps)
INSERT_D, INSERT_L, SCREW_D = 3.6, 4.0, 2.9  # M2.5 heat-set inserts (same EST as Widow) and M2.5 clearance
# Puck LITE (Velodyne datasheet 63-9229 / 63-9286): body, base height, optical centre above the base, pins, 1/4-20
LID_D, LID_H, LID_OPT = 103.3, 71.7, 37.8
PIN_SP, PIN_HOLE = 88.9, 4.0                # 2 pins on the 90 deg line, 5/32 in (3.97 mm) steel dowels pressed in
QTR_D = 6.6                                 # 1/4-20 clearance: 1/4-20 x 1/2 in screw from behind the wall
LID_GAP, WALL_T, WALL_HW, WALL_BELOW = 2.0, 6.0, 55.0, 14.0
GUS_Y, GUS_T, GUS_X = 40.0, 5.0, 12.0       # 2 gussets behind the wall
CABLE_SLOT = (0.0, 20.0, 9.0)               # x from, x to, half width: the LiDAR cable goes up through the plate
# battery: PowerHobby 6S 5200 50C (listing: 133 x 43.5 x 49.5, 658 g), across the drone
BAT_L, BAT_W, BAT_H, BAT_FIT = 133.0, 43.5, 49.5, 1.0
BAT_X0 = -24.0                              # battery front face (x)
RAIL_T, RAIL_H, RAIL_HW = 2.4, 12.0, 36.0
STRAP_W, STRAP_T, STRAP_Y = 22.0, 3.5, 53.0  # 20 mm velcro straps, centred this far off the centre line
# plate outline: front part (LiDAR + camera), battery wings, back part (interface box)
PL_X_BACK, WING_X, WING_HW, FRONT_HW, BACK_HW = -134.0, (-17.0, -78.0), 66.0, 56.0, 40.0
IFB = (75.0, 53.0, 28.0)                    # EST: Velodyne interface box (y, x, z): measure it
CLAMP_X = (-112.0, -40.0, 40.0)             # clamps on each rod, clear of the dampers (x 66-80)
CL_L, CL_IN, CL_OUT, CL_TOP = 20.0, 3.0, 5.5, 2.5   # clamp length, wall toward the centre, outer wall (grub insert), over the rod
# OAK-D Pro cradle: identical numbers to Widow's landing_leg.py
CAM_L, CAM_H, CAM_D, CAM_BODY, CAM_USB = 96.96, 29.45, 23.15, 14.28, 9.45
CAM_FIT, TILT = 0.4, 15.0
CAM_BACK_X = LID_H + 4.3                    # camera's lowest back corner: just ahead of the LiDAR's top face
NOSE_T, END_T, END_BLOCK = PL_T, 3.0, 16.0
LIP, LIP_T, LENS_EDGE = 3.5, 2.0, 8.0
CAM_STRAP_W, CAM_STRAP_T = 17.0, 3.5
# strength self-check: hard landing at G_LAND on the LiDAR wall, across the print layers
LID_G, G_LAND = 0.590, 8.0
SIGMA_LAYER_MPA, SF_MIN = 15.0, 3.5

import adsk.core, adsk.fusion, math, os
cm = lambda mm: mm / 10.0
z_ax = PL_T + LID_GAP + LID_D / 2             # LiDAR axis, print z
wall_bot = z_ax + WALL_BELOW
bat_x1 = BAT_X0 - BAT_W - BAT_FIT
cl_w = ROD_D + FIT + CL_IN + CL_OUT
cl_h = (ROD_Z + ROD_D / 2 + CL_TOP) - Z_PT     # clamp height above the plate's top face
bore_z = ROD_Z - Z_PT                          # rod axis above the plate's top face (clamp coords)

assert Z_PT < DAMP_Z - 1.5 and Z_PT < ROD_Z - ROD_D / 2 - 1.5, 'plate runs into the dampers or rods'
assert ROD_Z + ROD_D / 2 + CL_TOP <= -25.2 - 1.5, 'clamps hit the bottom plate (its underside is at -25.2)'
assert all(abs(abs(x) - 73) > CL_L / 2 + 6 for x in CLAMP_X), 'a clamp sits on a damper'
assert bore_z - (ROD_D + FIT) / 2 >= INSERT_L + 1.5, 'clamp inserts run into the rod bore'
assert CL_OUT >= INSERT_L + 0.8, 'grub screw insert does not fit in the clamp wall'
assert WALL_HW >= PIN_SP / 2 + PIN_HOLE / 2 + 4, 'pin holes too close to the wall edge'
assert BAT_X0 <= -WALL_T - GUS_X - 2, 'battery runs into the gussets'
assert WING_X[0] >= BAT_X0 + 3.5 + 2 and WING_X[1] <= bat_x1 - STRAP_T - 2, 'battery wings too short for the straps'
assert STRAP_Y - STRAP_W / 2 >= ROD_Y + (ROD_D + FIT) / 2 + CL_OUT + 1, 'strap slots run into the clamps'
assert STRAP_Y + STRAP_W / 2 <= WING_HW - 2, 'strap slots too close to the wing edge'
assert PL_X_BACK <= bat_x1 - STRAP_T - 4 - IFB[1], 'no room for the interface box behind the battery'
# LiDAR wall: the sensor's weight under a hard landing bends the wall root (layers run across it)
M = LID_G * 9.81 * G_LAND * (LID_H / 2)
sigma_wall = M / (2 * WALL_HW * WALL_T**2 / 6)
assert SIGMA_LAYER_MPA / sigma_wall >= SF_MIN, f'LiDAR wall stress {sigma_wall:.1f} MPa'
assert LIP <= LENS_EDGE - 3 and CAM_STRAP_W <= CAM_D + CAM_FIT - 4, 'camera cradle'
t_ = math.radians(TILT)
f_, u_ = (math.cos(t_), math.sin(t_)), (math.sin(t_), -math.cos(t_))   # print coords (x, z): camera forward, camera up
cH, cD, cL = CAM_H + CAM_FIT, CAM_D + CAM_FIT, CAM_L + CAM_FIT
mv = lambda p, v, k: (p[0] + k * v[0], p[1] + k * v[1])
Bk = (CAM_BACK_X + cH * math.sin(t_), NOSE_T)
Ft = mv(Bk, f_, cD)
cam_c = mv(mv(Bk, f_, cD / 2), u_, -cH / 2)
E = CAM_STRAP_T + 3.0
assert mv(Bk, u_, -(cH + E))[0] >= LID_H + 1.5, 'camera cradle runs into the LiDAR'
assert cL / 2 + END_T <= FRONT_HW, 'camera wider than the plate'

for d in [d for d in app.documents if d.name.startswith('Huntsman payload')]:
    d.close(False)
doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
doc.name = 'Huntsman payload carrier'
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
    base = {'xy': comp.xYConstructionPlane, 'xz': comp.xZConstructionPlane, 'yz': comp.yZConstructionPlane}[plane]
    if off:
        pi = comp.constructionPlanes.createInput(); pi.setByOffset(base, V(off)); base = comp.constructionPlanes.add(pi)
    return comp.sketches.add(base)

def pt(sk, x, y, z):
    return sk.modelToSketchSpace(adsk.core.Point3D.create(cm(x), cm(y), cm(z)))

def circle(sk, c, d):
    sk.sketchCurves.sketchCircles.addByCenterRadius(pt(sk, *c), cm(d / 2)); return sk

def poly(sk, pts):
    L = sk.sketchCurves.sketchLines
    for i in range(len(pts)):
        L.addByTwoPoints(pt(sk, *pts[i]), pt(sk, *pts[(i + 1) % len(pts)]))
    return sk

def box_xy(sk, x0, y0, x1, y1, z):
    return poly(sk, [(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)])

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

def merge(comp):
    bodies = list(comp.bRepBodies)
    if len(bodies) > 1:
        combine(comp, bodies[0], bodies[1:], JOIN)
    return comp.bRepBodies.item(0)

# ---- carrier
car_occ = part('payload carrier'); c = car_occ.component
xf = Ft[0] + LIP_T                                             # plate front end
outline = [(xf, -FRONT_HW), (xf, FRONT_HW), (WING_X[0], FRONT_HW), (WING_X[0], WING_HW), (WING_X[1], WING_HW),
           (WING_X[1], BACK_HW), (PL_X_BACK, BACK_HW), (PL_X_BACK, -BACK_HW), (WING_X[1], -BACK_HW),
           (WING_X[1], -WING_HW), (WING_X[0], -WING_HW), (WING_X[0], -FRONT_HW)]
body = extrude(c, poly(sketch(c, 'xy'), [(x, y, 0) for x, y in outline]), NEW, PL_T).bodies.item(0)
extrude(c, box_xy(sketch(c, 'xy', PL_T), -WALL_T, -WALL_HW, 0, WALL_HW, PL_T), JOIN, wall_bot - PL_T, body=body)   # LiDAR wall
for sy in (-1, 1):                                             # gussets behind the wall
    sk = poly(sketch(c, 'xz'), [(-WALL_T + 0.5, 0, PL_T - 0.5), (-WALL_T - GUS_X, 0, PL_T - 0.5), (-WALL_T + 0.5, 0, z_ax)])
    g = extrude(c, sk, NEW, sym=GUS_T).bodies.item(0)
    mi = adsk.core.Matrix3D.create(); mi.translation = adsk.core.Vector3D.create(0, cm(sy * GUS_Y), 0)
    coll = adsk.core.ObjectCollection.create(); coll.add(g)
    m = c.features.moveFeatures.createInput2(coll); m.defineAsFreeMove(mi); c.features.moveFeatures.add(m)
sk = sketch(c, 'xy', PL_T)
for x0 in (BAT_X0 + RAIL_T, bat_x1):                           # battery rails, front and back
    box_xy(sk, x0 - RAIL_T, -RAIL_HW, x0, RAIL_HW, PL_T)
extrude(c, sk, JOIN, RAIL_H, body=body)
# camera cradle (as on Widow's leg mount; there the tray stood where our plate is)
def slab(pts, w_out, w_in=0.0):
    xz = lambda: poly(sketch(c, 'xz'), [(x, 0, z) for x, z in pts])
    b = extrude(c, xz(), NEW, sym=2 * w_out).bodies.item(0)
    if w_in:
        combine(c, b, [extrude(c, xz(), NEW, sym=2 * w_in).bodies.item(0)], CUT)
    return b
Bb = mv(Ft, f_, -(CAM_BODY + CAM_FIT))
slab([(Bk[0], 0), (Ft[0], 0), Ft, mv(Ft, f_, LIP_T), mv(mv(Ft, f_, LIP_T), u_, -(cH + E)), mv(Bk, u_, -(cH + E)), Bk], cL / 2 + END_T, cL / 2)
slab([(Bk[0], 0), (Ft[0], 0), Ft, Bk], cL / 2 + END_T, cL / 2 - END_BLOCK)
slab([(Ft[0], 0), (mv(Ft, f_, LIP_T)[0], 0), mv(Ft, f_, LIP_T), mv(mv(Ft, f_, LIP_T), u_, -cH), mv(Ft, u_, -cH), Ft], cL / 2 + END_T, cL / 2 - LIP)
slab([Bb, mv(Bb, f_, -LIP_T), mv(mv(Bb, f_, -LIP_T), u_, -cH), mv(Bb, u_, -cH)], cL / 2 + END_T, cL / 2 - LIP)
body = merge(c)
p0 = mv(mv(Bk, u_, -(cH + 0.3)), f_, (cD - CAM_STRAP_W) / 2); p1 = mv(p0, f_, CAM_STRAP_W)
extrude(c, poly(sketch(c, 'xz'), [(x, 0, z) for x, z in (p0, p1, mv(p1, u_, -CAM_STRAP_T), mv(p0, u_, -CAM_STRAP_T))]),
        CUT, sym=cL + 2 * END_T + 2, body=body)               # camera strap slots through the end plates
usb_x = Ft[0] - (CAM_USB + CAM_FIT / 2) * math.cos(t_)
sk = sketch(c, 'xy', -1)
for sy in (-1, 1):
    box_xy(sk, usb_x - 7, sy * 6, usb_x + 7, sy * 22, -1)      # camera USB-C windows (plug either side)
x0, x1, hw = CABLE_SLOT
box_xy(sk, x0, -hw, x1, hw, -1)                                # LiDAR cable slot
for x in (BAT_X0 + 3.5, bat_x1):                               # strap slots in front of and behind the battery
    for sy in (-1, 1):
        box_xy(sk, x - STRAP_T, sy * STRAP_Y - STRAP_W / 2, x, sy * STRAP_Y + STRAP_W / 2, -1)
extrude(c, sk, CUT, PL_T + 2, body=body)
sk = sketch(c, 'yz', 1)
circle(sk, (1, 0, z_ax), QTR_D)
for sy in (-1, 1):
    circle(sk, (1, sy * PIN_SP / 2, z_ax), PIN_HOLE)
extrude(c, sk, CUT, -(WALL_T + 2), body=body)                  # 1/4-20 screw and the 2 pins through the wall
sk = sketch(c, 'xy', -1)
for x in CLAMP_X:
    for sy in (-1, 1):
        for dx in (-6, 6):
            circle(sk, (x + dx, sy * ROD_Y, -1), SCREW_D)
extrude(c, sk, CUT, PL_T + 2, body=body)                       # M2.5 x 8 up through the plate into the clamps
sk = sketch(c, 'xy', PL_T - 1.8)
for x in CLAMP_X:
    for sy in (-1, 1):
        for dx in (-6, 6):
            circle(sk, (x + dx, sy * ROD_Y, PL_T - 1.8), 5.2)
extrude(c, sk, CUT, 2.0, body=body)                            # counterbores: heads sit flush under the plate
carrier = c.bRepBodies.item(0); carrier.name = 'payload carrier'

# ---- rod clamp (clamp coords: plate face at z = 0, rod along x at y = 0, z = bore_z; outer side +y)
cl_occ = part('rod clamp'); c = cl_occ.component
y_in, y_out = -((ROD_D + FIT) / 2 + CL_IN), (ROD_D + FIT) / 2 + CL_OUT
cl = extrude(c, box_xy(sketch(c, 'xy'), -CL_L / 2, y_in, CL_L / 2, y_out, 0), NEW, cl_h).bodies.item(0)
extrude(c, circle(sketch(c, 'yz'), (0, 0, bore_z), ROD_D + FIT), CUT, sym=CL_L + 2, body=cl)
sk = sketch(c, 'xy')
for dx in (-6, 6):
    circle(sk, (dx, 0, 0), INSERT_D)
extrude(c, sk, CUT, INSERT_L + 1.0, body=cl)                   # inserts for the plate screws
f = extrude(c, circle(sketch(c, 'xz'), (0, 0, bore_z), INSERT_D), NEW, sym=2 * (y_out + 1)).bodies.item(0)
keep = extrude(c, box_xy(sketch(c, 'xy'), -CL_L, -50, CL_L, (ROD_D + FIT) / 2 + 0.5, 0), NEW, cl_h + 2).bodies.item(0)
combine(c, f, [keep], CUT)                                     # grub screw insert hole in the outer wall only
combine(c, cl, [f], CUT)
clamp = c.bRepBodies.item(0); clamp.name = 'rod clamp'

OUT = os.path.join(HERE, '..', 'Huntsman', 'Printed Parts', 'Payload Carrier')
os.makedirs(OUT, exist_ok=True)
em = des.exportManager
res = {'wall_stress_MPa': round(sigma_wall, 2), 'wall_sf': round(SIGMA_LAYER_MPA / sigma_wall, 1),
       'lidar_axis_flight_z': round(Z_PT - z_ax, 2), 'lidar_bottom_flight_z': round(Z_PT - z_ax - LID_D / 2, 2),
       'cam_centre_print': [round(v, 2) for v in cam_c], 'cam_lens_print': [round(v, 2) for v in mv(Ft, u_, -cH / 2)],
       'battery_x': [BAT_X0, round(bat_x1, 2)], 'plate_front_x': round(xf, 2), 'clamp_h': round(cl_h, 2)}
for name, occ, bd in (('payload-carrier', car_occ, carrier), ('rod-clamp', cl_occ, clamp)):
    assert occ.component.bRepBodies.count == 1 and bd.lumps.count == 1, f'{name} is not one solid'
    em.execute(em.createSTEPExportOptions(os.path.join(OUT, f'{name}.step'), occ.component))   # mounting coords (the assembly uses these)
    if name == 'rod-clamp':            # print it standing on its end: the rod hole goes up, no supports
        R = adsk.core.Matrix3D.create(); R.setToRotation(math.radians(-90), adsk.core.Vector3D.create(0, 1, 0), adsk.core.Point3D.create(0, 0, 0))
        coll = adsk.core.ObjectCollection.create(); coll.add(bd)
        mi = occ.component.features.moveFeatures.createInput2(coll); mi.defineAsFreeMove(R); occ.component.features.moveFeatures.add(mi)
        bd = occ.component.bRepBodies.item(0)
    o = em.createSTLExportOptions(bd, os.path.join(OUT, f'{name}.stl'))
    o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
    em.execute(o)
    bb = bd.boundingBox
    res[name] = {'volume_cm3': round(bd.physicalProperties.volume, 2),
                 'bbox_mm': [round((bb.maxPoint.x - bb.minPoint.x) * 10, 1), round((bb.maxPoint.y - bb.minPoint.y) * 10, 1), round((bb.maxPoint.z - bb.minPoint.z) * 10, 1)]}
T = adsk.core.Matrix3D.create(); T.translation = adsk.core.Vector3D.create(cm(0), cm(110), 0); cl_occ.transform = T   # picture only
des.activateRootComponent(); app.userInterface.activeSelections.clear()
for occ in (car_occ, cl_occ):
    for s in occ.component.sketches:
        s.isVisible = False
    for p in occ.component.constructionPlanes:
        p.isLightBulbOn = False
vp = app.activeViewport
cam = vp.camera; cam.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation; cam.isFitView = True; vp.camera = cam
vp.saveAsImageFile(os.path.join(OUT, 'payload-carrier-preview.png'), 1200, 800)
res
