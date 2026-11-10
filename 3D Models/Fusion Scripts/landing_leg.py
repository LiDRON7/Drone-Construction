# Widow landing gear + battery tray. Runs inside Fusion via the add-in:
#   python3 run_in_fusion.py execute_code -code landing_leg.py
# Saves STEP/STL/preview into ../Widow/Printed Parts/Landing Leg/ (HERE is set by run_in_fusion.py).
# The battery plate (the hexagon under the bottom star) only has screws at the two leg spots (2 squares of 4 x M3),
# so one printed part takes all 8:
#   mount - plate on both squares, a socket for each leg tube leaning out LEAN deg, and a battery tray between them
#           (battery held by a 20 mm strap through windows in the side rails). Every screw goes into an M2.5 heat-set insert.
#   nose  - cradle for the OAK-D Pro camera on the front of the tray, looking forward and TILT deg down, out of the gap
#           between the two front arms, inside the guards' reach. Camera goes in from below, a 15 mm strap under it holds it.
#   foot  - straight T: socket for the leg tube on a barrel the skid tube slides through (print 2). It turns on the
#           skid to meet the leaning leg, so it needs no lean of its own; 2 grub screws lock it.
# Print coords: the mount's face that meets the drone is on the bed, sockets up; feet socket up. Leg tubes are carbon.
# Fit 0.3 mm on diameter (snug PETG). All mm. Measured with calipers on the drone (2026-11-09) unless "EST".
LEG_TUBE, SKID_TUBE = 16.04, 9.89       # measured tube outside diameters
FIT = 0.3
LEAN = 20.0                             # deg the legs lean out from vertical (stock leg, from a photo: rough)
SOCK_D, SOCK_WALL = 25.0, 5.0           # tube depth in each socket, socket wall
INSERT_D, INSERT_L = 3.6, 4.0           # M2.5 heat-set insert: hole diameter and length (EST: use your inserts' spec)
SCREW_D = 2.9                           # M2.5 clearance
CROSS_D = SCREW_D                       # M2.5 cross bolt through socket and tube, into an insert in the far wall
# mount: 2 squares of 4 holes. Holes 24.62 along x (fore-aft) and 25.95 across; squares 75.05 apart centre to centre
LEG_Y = (100.59 + 49.51) / 4            # measured over the far holes and the near holes, across the drone
HOLE_X, HOLE_Y = 24.62, (35.77**2 - 24.62**2) ** 0.5
FL_T, FL_EDGE, FL_CHAMF = 6.0, 5.0, 2.5
GUS_R, GUS_H = 15.0, 5.0                # gusset cone where each socket meets the plate
BAT_W, BAT_L, BAT_H = 43.21, 135.41, 31.11   # measured 3S 5500 mAh
TRAY_X, NECK_X = 48.0, 22.0             # tray ends reach x = +-TRAY_X; narrow neck up to NECK_X
RAIL_T, RAIL_H = 2.4, 12.0
STRAP_W, STRAP_H = 22.0, 3.5            # window for a 20 mm velcro strap, one at each tray end
# 2 orange M3 button-head screws stick out of the battery plate under the tray ends, on the centre line near each tip
# (photo IMG_8836, scaled off the leg squares: EST). The tray face gets a pocket over each head, slotted +-SCREW_SLOT.
SCREW_X, HEAD_D, HEAD_H, SCREW_SLOT = 46.8, 5.7, 1.65, 2.0
POCKET_D, POCKET_H = HEAD_D + 1.5, HEAD_H + 0.7
STRAP_XC = SCREW_X - SCREW_SLOT - POCKET_D / 2 - 1.5 - STRAP_W / 2   # strap groove centre: stops short of the pocket
# OAK-D Pro (measured): length, height, depth with fins, body depth without fins, USB-C centre behind the front face
CAM_L, CAM_H, CAM_D, CAM_BODY, CAM_USB = 96.96, 29.45, 23.15, 14.28, 9.45
CAM_FIT, TILT = 0.4, 15.0               # TILT: deg down; keeps the front arm, prop and guard out of the top of the view
CAM_BACK_X = 58.0                       # camera's lowest back corner; the battery's front end is at 54.7 (BATT_X in full_drone.py)
NOSE_T, END_T, END_BLOCK = 4.0, 3.0, 16.0   # plate over the camera, end plates, blocks between plate and camera top at each end
LIP, LIP_T, LENS_EDGE = 3.5, 2.0, 8.0   # front/back lips over the camera's ends; outer lens edge 8 mm from the end (photo, EST)
CAM_STRAP_W, CAM_STRAP_T = 17.0, 3.5    # slot in each end plate for a 15 mm velcro strap under the camera
CAM_G, G_CRASH = 0.0897, 20.0           # kg measured; sideways crash load on the nose
# foot T: stock T is 38.9 long along the skid; foam sleeves (12 ID, 18 OD) sit either side and must touch ground first
FOOT_L, SKID_WALL, BOT_WALL, BOT_HW = 38.9, 5.5, 1.6, 5.0   # SKID_WALL holds the grub screw inserts
FOAM_OD = 18.0
SET_X = 15.8                            # 2 x M2.5 grub screws from the top, in inserts, into the skid tube
FOOT_S0 = 2.5                           # socket floor this far above the barrel top

# strength self-check: hard landing at G_LAND, a side load of SIDE x that, bending the leg at a socket mouth
AUW_KG, G_LAND, SIDE, N_LEGS, LEG_LEN = 1.91, 8.0, 0.3, 2, 134.0   # LEG_LEN: measured exposed stock tube
SIGMA_LAYER_MPA, SF_MIN = 15.0, 3.5     # PETG pulled across layers (sockets print upright): conservative

import adsk.core, adsk.fusion, math, os
cm = lambda mm: mm / 10.0
sa, ca, ta = math.sin(math.radians(LEAN)), math.cos(math.radians(LEAN)), math.tan(math.radians(LEAN))
bore, skid_bore = LEG_TUBE + FIT, SKID_TUBE + FIT
sock_od = bore + 2 * SOCK_WALL
r_s = skid_bore / 2
tear_top = r_s + 0.6                        # teardrop cut flat here: bridge = 2*(r_s*sqrt2 - tear_top)
fl_x, fl_y = HOLE_X + 2 * FL_EDGE, 2 * LEG_Y + HOLE_Y + 2 * FL_EDGE
rail_in = BAT_W / 2 + 0.4
zb = r_s + SKID_WALL                        # foot barrel top
bot = r_s + BOT_WALL                        # foot flat bottom, below the skid axis
foot_w = sock_od / 2 + 0.3                  # foot block half width: holds the socket's footprint
tube_l = LEG_LEN + 2 * SOCK_D                # carbon leg tube length to cut

F = AUW_KG * 9.81 * G_LAND / N_LEGS
M = F * LEG_LEN * (sa + SIDE * ca)          # leaning leg: the landing load itself bends it
sigma = M / (math.pi * (sock_od**4 - bore**4) / (32 * sock_od))
assert SIGMA_LAYER_MPA / sigma >= SF_MIN, f'socket stress {sigma:.1f} MPa, safety factor < {SF_MIN}'
r_bolt = math.hypot(HOLE_X / 2, HOLE_Y / 2)
assert sock_od / 2 / ca <= GUS_R - 0.5, 'socket footprint wider than its gusset'
assert GUS_R + 0.8 <= r_bolt - INSERT_D / 2, 'inserts run into the gusset'
assert LEG_Y - GUS_R >= rail_in, 'battery runs into a socket'
assert FL_T >= INSERT_L + 1.5 and HOLE_X / 2 + INSERT_D / 2 + 1.6 <= fl_x / 2, 'plate inserts: walls too thin'
assert SOCK_WALL >= INSERT_L + 0.8, 'cross bolt insert does not fit in the socket wall'
assert r_s + SKID_WALL - INSERT_L - 0.3 >= r_s + 0.6 + 0.5, 'grub screw insert runs into the skid bore'
assert 2 * (r_s * math.sqrt(2) - tear_top) <= 10, 'teardrop bridge too long'
assert max(math.hypot(BOT_HW, bot), (BOT_HW + bot) / math.sqrt(2)) <= FOAM_OD / 2 - 0.5, 'foot reaches the ground before the foam'
assert zb + FOOT_S0 >= tear_top + 2.5, 'foot socket floor too close to the skid bore'
assert sock_od / 2 + INSERT_D / 2 + 0.6 <= SET_X <= FOOT_L / 2 - INSERT_D / 2 - 1.5, 'grub screw insert too close to an edge'
assert LIP <= LENS_EDGE - 3, 'camera lips cover the outer lenses'
assert CAM_STRAP_W <= CAM_D + CAM_FIT - 4, 'camera strap slot longer than the camera is deep'
t_ = math.radians(TILT)
f_, u_ = (math.cos(t_), math.sin(t_)), (math.sin(t_), -math.cos(t_))   # print coords (x, z): camera forward, camera up
cH, cD, cL = CAM_H + CAM_FIT, CAM_D + CAM_FIT, CAM_L + CAM_FIT
mv = lambda p, v, k: (p[0] + k * v[0], p[1] + k * v[1])
Bk = (CAM_BACK_X + cH * math.sin(t_), NOSE_T)                          # pocket top-back corner
Ft = mv(Bk, f_, cD)                                                     # pocket top-front corner
cam_c = mv(mv(Bk, f_, cD / 2), u_, -cH / 2)                             # camera centre (print coords)
nose_sigma = CAM_G * 9.81 * G_CRASH * (cam_c[0] - TRAY_X) / ((cL + 2 * END_T) * NOSE_T**2 / 6)
assert SIGMA_LAYER_MPA / nose_sigma >= SF_MIN, f'camera nose stress {nose_sigma:.1f} MPa'
assert FL_T - POCKET_H >= 2.4, 'tray floor too thin over the screw pockets'
assert STRAP_XC - STRAP_W / 2 >= fl_x / 2 and STRAP_XC + STRAP_W / 2 <= SCREW_X - SCREW_SLOT - POCKET_D / 2 - 1.0, 'strap groove runs into the screw pocket or the leg plate'
assert SOCK_WALL >= 2.4 and SKID_WALL >= 2.4 and BOT_WALL >= 1.2 and RAIL_T >= 1.6, 'walls under the FDM minimum'

for d in [d for d in app.documents if d.name.startswith('Widow landing leg')]:
    d.close(False)
doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
doc.name = 'Widow landing leg'
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
    # plane 'xy' (normal z), 'xz' (normal y), 'yz' (normal x); points are given in model coords
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
        inp.setDistanceExtent(False, V(h))      # along the plane normal: +z for 'xy' sketches
    if body:
        inp.participantBodies = [body]
    return comp.features.extrudeFeatures.add(inp)

def cone(comp, z0, r0, z1, r1, body, y0=0.0, y1=0.0):
    li = comp.features.loftFeatures.createInput(JOIN if body else NEW)
    li.loftSections.add(circle(sketch(comp, 'xy', z0), (0, y0, z0), 2 * r0).profiles.item(0))
    li.loftSections.add(circle(sketch(comp, 'xy', z1), (0, y1, z1), 2 * r1).profiles.item(0))
    li.isSolid = True
    if body:
        li.participantBodies = [body]
    comp.features.loftFeatures.add(li)

def combine(comp, target, tools, op):
    coll = adsk.core.ObjectCollection.create()
    for t in tools:
        coll.add(t)
    ci = comp.features.combineFeatures.createInput(target, coll); ci.operation = op
    comp.features.combineFeatures.add(ci)

def merge(comp):
    bodies = [b for b in comp.bRepBodies if b.name != 'cutter']   # cutters stay separate
    if len(bodies) > 1:  # lofts/joins of separately-sketched solids can land as extra bodies
        combine(comp, bodies[0], bodies[1:], JOIN)
    return bodies[0]

def cut_all(comp):
    body = merge(comp)
    combine(comp, body, [b for b in comp.bRepBodies if b.name == 'cutter'], CUT)
    return comp.bRepBodies.item(0)

def hexagon(sk, cx, cy, z, af):
    r = af / math.sqrt(3)
    return poly(sk, [(cx + r * math.cos(math.radians(60 * i)), cy + r * math.sin(math.radians(60 * i)), z) for i in range(6)])

def leaning_socket(comp, body, py, pz, side, s0, embed, lean=LEAN):
    # socket on an axis through (0, py, pz) leaning lean deg toward side*y; tube floor s0 up the axis.
    # Built upright at the origin, then tilted and moved; merge() joins the socket, cut_all() cuts the bore and cross hole.
    sock = extrude(comp, circle(sketch(comp, 'xy', -embed), (0, 0, -embed), sock_od), NEW, embed + s0 + SOCK_D).bodies.item(0)
    hole = extrude(comp, circle(sketch(comp, 'xy', s0), (0, 0, s0), bore), NEW, SOCK_D + 1).bodies.item(0)
    cross = extrude(comp, circle(sketch(comp, 'yz'), (0, 0, s0 + SOCK_D / 2), CROSS_D), NEW, sym=sock_od + 2).bodies.item(0)
    xo = sock_od / 2                                                     # insert for the cross bolt in the +x wall
    ins = extrude(comp, circle(sketch(comp, 'yz', xo), (xo, 0, s0 + SOCK_D / 2), INSERT_D), NEW, sym=2 * (INSERT_L + 0.2)).bodies.item(0)
    m = adsk.core.Matrix3D.create()
    m.setToRotation(-side * math.radians(lean), adsk.core.Vector3D.create(1, 0, 0), adsk.core.Point3D.create(0, 0, 0))
    t = adsk.core.Matrix3D.create(); t.translation = adsk.core.Vector3D.create(0, cm(py), cm(pz)); m.transformBy(t)
    coll = adsk.core.ObjectCollection.create()
    for b in (sock, hole, cross, ins):
        coll.add(b)
    hole.name = cross.name = ins.name = 'cutter'
    mi = comp.features.moveFeatures.createInput2(coll); mi.defineAsFreeMove(m)
    comp.features.moveFeatures.add(mi)

# ---- mount (print coords: face against the drone's bottom plate on the bed, sockets and tray up; y across the drone)
mount_occ = part('landing leg mount'); c = mount_occ.component
a, b, k = fl_x / 2, fl_y / 2, FL_CHAMF
body = extrude(c, poly(sketch(c, 'xy'), [(-a + k, -b, 0), (a - k, -b, 0), (a, -b + k, 0), (a, b - k, 0),
                                          (a - k, b, 0), (-a + k, b, 0), (-a, b - k, 0), (-a, -b + k, 0)]), NEW, FL_T).bodies.item(0)
extrude(c, box_xy(sketch(c, 'xy'), -TRAY_X, -20, TRAY_X, 20, 0), JOIN, FL_T, body=body)                    # tray floor + neck
sk = sketch(c, 'xy')
for sx in (-1, 1):
    box_xy(sk, sx * NECK_X, -rail_in - RAIL_T, sx * TRAY_X, rail_in + RAIL_T, 0)
extrude(c, sk, JOIN, FL_T, body=body)                                                                     # tray ends
sk = sketch(c, 'xy', FL_T)
for sx in (-1, 1):
    for sy in (-1, 1):
        box_xy(sk, sx * NECK_X, sy * rail_in, sx * TRAY_X, sy * (rail_in + RAIL_T), FL_T)
extrude(c, sk, JOIN, RAIL_H, body=body)                                                                   # side rails
for side in (-1, 1):
    leaning_socket(c, body, side * LEG_Y, FL_T, side, 0.0, 1.0)
    cone(c, FL_T, GUS_R, FL_T + GUS_H, sock_od / 2 - 0.2, None, side * LEG_Y, side * (LEG_Y + GUS_H * ta))
    body = cut_all(c)                                                                                     # one side at a time
def slab(pts, w_out, w_in=0.0):  # x-z profile (print coords) extruded over w_in <= |y| <= w_out
    xz = lambda: poly(sketch(c, 'xz'), [(x, 0, z) for x, z in pts])
    b = extrude(c, xz(), NEW, sym=2 * w_out).bodies.item(0)
    if w_in:
        combine(c, b, [extrude(c, xz(), NEW, sym=2 * w_in).bodies.item(0)], CUT)
    return b
E = CAM_STRAP_T + 3.0                                                   # end plates reach this far under the camera
Bb = mv(Ft, f_, -(CAM_BODY + CAM_FIT))                                  # back of the camera body, at its ends (fins start ~9 mm in)
nose = [extrude(c, box_xy(sketch(c, 'xy'), TRAY_X - 1, -(cL / 2 + END_T), Ft[0], cL / 2 + END_T, 0), NEW, NOSE_T).bodies.item(0),
        slab([(Bk[0], 0), (Ft[0], 0), Ft, mv(Ft, f_, LIP_T), mv(mv(Ft, f_, LIP_T), u_, -(cH + E)), mv(Bk, u_, -(cH + E)), Bk], cL / 2 + END_T, cL / 2),   # LIP_T longer: the front lips join it on a face
        slab([(Bk[0], 0), (Ft[0], 0), Ft, Bk], cL / 2 + END_T, cL / 2 - END_BLOCK),
        slab([(Ft[0], 0), (mv(Ft, f_, LIP_T)[0], 0), mv(Ft, f_, LIP_T), mv(mv(Ft, f_, LIP_T), u_, -cH), mv(Ft, u_, -cH), Ft], cL / 2 + END_T, cL / 2 - LIP),
        slab([Bb, mv(Bb, f_, -LIP_T), mv(mv(Bb, f_, -LIP_T), u_, -cH), mv(Bb, u_, -cH)], cL / 2 + END_T, cL / 2 - LIP)]   # lips run into the end plates: solid joins
body = merge(c)                                                         # the cuts above split each slab into two bodies
p0 = mv(mv(Bk, u_, -(cH + 0.3)), f_, (cD - CAM_STRAP_W) / 2)
p1 = mv(p0, f_, CAM_STRAP_W)
extrude(c, poly(sketch(c, 'xz'), [(x, 0, z) for x, z in (p0, p1, mv(p1, u_, -CAM_STRAP_T), mv(p0, u_, -CAM_STRAP_T))]),
        CUT, sym=cL + 2 * END_T + 2, body=body)                         # strap slots through the end plates
usb_x = Ft[0] - (CAM_USB + CAM_FIT / 2) * math.cos(t_)
sk = sketch(c, 'xy', -1)
for sy in (-1, 1):                                                      # the plug can be on either side: camera fits both ways up
    box_xy(sk, usb_x - 7, sy * 6, usb_x + 7, sy * 22, -1)
extrude(c, sk, CUT, NOSE_T + 2, body=body)                              # USB-C windows: cable goes up to the Pi
sk = sketch(c, 'xy', -1)
for sx in (-1, 1):
    for sy in (-1, 1):
        for side in (-1, 1):
            circle(sk, (sx * HOLE_X / 2, side * LEG_Y + sy * HOLE_Y / 2, -1), INSERT_D)
extrude(c, sk, CUT, INSERT_L + 1.5, body=body)                          # insert holes in the face against the plate: M2.5 x 6 from above
sk = sketch(c, 'xy', FL_T - 2)
for sx in (-1, 1):
    xc = sx * STRAP_XC
    box_xy(sk, xc - STRAP_W / 2, -fl_y, xc + STRAP_W / 2, fl_y, FL_T - 2)
extrude(c, sk, CUT, 2 + STRAP_H, body=body)                     # strap groove under the battery + windows through both rails
sk = sketch(c, 'xy', -1)
for sx in (-1, 1):                                              # pockets for the orange screw heads: a slot along x
    x0, x1, r = sx * SCREW_X - SCREW_SLOT, sx * SCREW_X + SCREW_SLOT, POCKET_D / 2
    L, A = sk.sketchCurves.sketchLines, sk.sketchCurves.sketchArcs
    L.addByTwoPoints(pt(sk, x0, -r, -1), pt(sk, x1, -r, -1)); L.addByTwoPoints(pt(sk, x0, r, -1), pt(sk, x1, r, -1))
    A.addByThreePoints(pt(sk, x1, -r, -1), pt(sk, x1 + r, 0, -1), pt(sk, x1, r, -1))
    A.addByThreePoints(pt(sk, x0, -r, -1), pt(sk, x0 - r, 0, -1), pt(sk, x0, r, -1))
extrude(c, sk, CUT, POCKET_H + 1, body=body)
mount = body; mount.name = 'landing leg mount'

# ---- foot T (print coords: skid axis along x at z = 0, flat bottom on the bed, socket straight up)
foot_occ = part('landing leg foot'); c = foot_occ.component
w, d45 = foot_w, foot_w - BOT_HW
body = extrude(c, poly(sketch(c, 'yz'), [(0, -BOT_HW, -bot), (0, BOT_HW, -bot), (0, w, -bot + d45), (0, w, zb),
                                         (0, -w, zb), (0, -w, -bot + d45)]), NEW, sym=FOOT_L).bodies.item(0)  # 45 deg sides: foam lands first
leaning_socket(c, body, 0.0, zb, 1, FOOT_S0, 3.0, lean=0.0)
body = cut_all(c)
extrude(c, circle(sketch(c, 'yz'), (0, 0, 0), skid_bore), CUT, sym=FOOT_L + 2, body=body)
q, wt = r_s / math.sqrt(2), r_s * math.sqrt(2) - tear_top
extrude(c, poly(sketch(c, 'yz'), [(0, -q, q), (0, q, q), (0, wt, tear_top), (0, -wt, tear_top)]), CUT, sym=FOOT_L + 2, body=body)
sk = sketch(c, 'xy', r_s - 1)
for sx in (-SET_X, SET_X):
    circle(sk, (sx, 0, r_s - 1), SCREW_D)
extrude(c, sk, CUT, zb - r_s + 2, body=body)                                                    # grub screws down into the skid tube
sk = sketch(c, 'xy', zb - INSERT_L - 0.3)
for sx in (-SET_X, SET_X):
    circle(sk, (sx, 0, zb - INSERT_L - 0.3), INSERT_D)
extrude(c, sk, CUT, INSERT_L + 1.3, body=body)                                                  # their inserts
foot = body; foot.name = 'landing leg foot'

OUT = os.path.join(HERE, '..', 'Widow', 'Printed Parts', 'Landing Leg')
os.makedirs(OUT, exist_ok=True)
em = des.exportManager
res = {'socket_stress_MPa': round(sigma, 2), 'socket_sf': round(SIGMA_LAYER_MPA / sigma, 1),
       'leg_y_mm': round(LEG_Y, 2), 'nose_stress_MPa': round(nose_sigma, 2), 'cam_centre_print_mm': [round(cam_c[0], 2), round(cam_c[1], 2)],
       'cam_lens_print_mm': [round(v, 2) for v in mv(Ft, u_, -cH / 2)], 'foot_floor_mm': round(zb + FOOT_S0, 2), 'tube_l_mm': tube_l}
for name, occ, bd in (('landing-leg-mount', mount_occ, mount), ('landing-leg-foot', foot_occ, foot)):
    assert occ.component.bRepBodies.count == 1 and bd.lumps.count == 1, f'{name} is not one solid'
    assert bd.faces.count and all(e.faces.count == 2 for e in bd.edges), f'{name} has parts touching only on an edge'
    o = em.createSTLExportOptions(bd, os.path.join(OUT, f'{name}.stl'))
    o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
    em.execute(o)
    em.execute(em.createSTEPExportOptions(os.path.join(OUT, f'{name}.step'), occ.component))
    bb = bd.boundingBox
    res[name] = {'volume_cm3': round(bd.physicalProperties.volume, 2),
                 'bbox_mm': [round((bb.maxPoint.x - bb.minPoint.x) * 10, 1), round((bb.maxPoint.y - bb.minPoint.y) * 10, 1), round((bb.maxPoint.z - bb.minPoint.z) * 10, 1)]}
for occ, (x, y) in ((foot_occ, (-85, 0)),):   # side by side for the picture only
    T = adsk.core.Matrix3D.create(); T.translation = adsk.core.Vector3D.create(cm(x), cm(y), 0); occ.transform = T
des.activateRootComponent(); app.userInterface.activeSelections.clear()   # else the last part draws ghosted
for occ in (mount_occ, foot_occ):
    for s in occ.component.sketches:
        s.isVisible = False
    for p in occ.component.constructionPlanes:
        p.isLightBulbOn = False
vp = app.activeViewport
cam = vp.camera; cam.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation; cam.isFitView = True; vp.camera = cam
vp.saveAsImageFile(os.path.join(OUT, 'landing-leg-preview.png'), 1200, 800)
res
