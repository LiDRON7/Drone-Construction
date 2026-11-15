# Linear static FEA of the printed parts, from the STEP files the Fusion scripts export.
# Needs: pip install gmsh scikit-fem pyamg matplotlib   (Python 3.10+)
# Run:   python3 fea.py guard      |   python3 fea.py carrier      (writes <part>-results.json and <part>-<case>.png here)
#
# Solid, isotropic, quadratic tetrahedra. Printed parts are weaker across layers and are not solid inside (walls +
# infill), so stresses are compared with conservative strengths and a safety factor of 2 is the bar.
# The hub and the clamp pads are held fixed where the motor or the rod clamps squeeze them; stresses right at those
# edges are partly an artefact of a perfectly rigid hold, so the report gives both the peak and the peak 3 mm away.
import sys, os, json, math
import numpy as np, gmsh
from skfem import MeshTet, Basis, FacetBasis, ElementVector, ElementTetP2, LinearForm, asm, condense
from skfem.models.elasticity import linear_elasticity, lame_parameters
from skfem.helpers import dot
import pyamg
from scipy.sparse.linalg import cg

HERE = os.path.dirname(os.path.abspath(__file__))
HUNT = os.path.join(HERE, '..', 'Huntsman', 'Printed Parts')
G = 9.81
# guard load cases: (deg round from outward, direction, N). The push is the stiffness check (ring must stay under half
# the tip gap from the prop); the 1 g bumps are strength checks (the whole drone's weight on one point of one guard).
GUARD_CASES = lambda auw, between: {'hand push 1 kg': (0.0, 'radial', 1.0 * G), 'front bump 1 g': (0.0, 'radial', auw * G),
               'side bump 1 g': (0.0, 'tangential', auw * G), 'bump between struts 1 g': (between, 'radial', auw * G),
               'lifted by the ring 1 kg': (0.0, 'up', 1.0 * G)}
# --- the parts. Numbers must match the Fusion scripts named here.
PARTS = {
    'guard': dict(   # huntsman_propeller_guard.py: motor axis at 0, x outward, z = 0 the motor plate top
        step=os.path.join(HUNT, 'Propeller Guard', 'propeller-guard.step'), keep='all', h=(0.7, 3.0),
        E=1500.0, nu=0.38, rho=1.27e-6, strength=30.0, material='PETG (XY, conservative)',
        held=lambda x: (np.hypot(x[0], x[1]) <= 19.0) & (x[2] <= 3.1),   # under the motor base (hub 3 mm)
        r_in=173.1, ring_t=1.3, band=(30.8, 40.8), tip_gap=8.0, auw=4.3, between=47.5),
    # the old arm-clamp guard, for comparison: python3 fea.py guard_old <its propeller-guard.step from git history>.
    # Held rigidly at its pad on the arm clamp (kinder than the real clamp, which can turn on the tube).
    'guard_old': dict(
        step=None, keep='all', h=(0.7, 3.0), E=1500.0, nu=0.38, rho=1.27e-6, strength=30.0, material='PETG (XY, conservative)',
        held=lambda x: (abs(x[0] + 88.0) <= 13.0) & (abs(x[1]) <= 12.0) & (x[2] <= 9.1),
        r_in=173.1, ring_t=1.3, band=(21.0, 31.0), tip_gap=8.0, auw=4.3, between=47.5),
    # Widow's 8 in guard (propeller_guard.py): z = 0 its bottom on the motor pad, held under the motor base (27.5 mm)
    'widow_guard': dict(
        step=os.path.join(HERE, '..', 'Widow', 'Printed Parts', 'Propeller Guard', 'propeller-guard.step'), keep='all',
        h=(0.7, 2.5), E=1500.0, nu=0.38, rho=1.27e-6, strength=30.0, material='PETG (XY, conservative)',
        held=lambda x: (np.hypot(x[0], x[1]) <= 13.0) & (x[2] <= 2.1),
        r_in=108.0, ring_t=1.7, band=(20.0, 30.0), tip_gap=6.4, auw=1.885, between=32.5),
    'carrier': dict(  # huntsman_payload.py, print coords: plate top (rod side) on z = 0, +z = down in flight
        step=os.path.join(HUNT, 'Payload Carrier', 'payload-carrier.step'), keep='max_x', h=(1.0, 4.0),
        E=4000.0, nu=0.35, rho=1.2e-6, strength=25.0, material='PAHT-CF (across layers, conservative)',
        clamps=[(x, s * 30.0) for x in (-40.0, 56.0) for s in (-1, 1)], clamp_box=(20.0, 22.0),
        wall=dict(y=-35.85, x=(41.65 - 50.5, 41.65 + 50.5), lidar_kg=0.590, offset=35.85),
        straps=dict(x=(-68.5, -20.5), y=53.0, w=22.0, battery_kg=0.658), g_land=8.0),
}


def mesh_step(step, keep, h):
    gmsh.initialize(); gmsh.option.setNumber('General.Terminal', 0)
    gmsh.option.setString('Geometry.OCCTargetUnit', 'MM')
    vols = gmsh.model.occ.importShapes(step); gmsh.model.occ.synchronize()
    vols = [v for v in vols if v[0] == 3]
    if keep == 'max_x':
        vols = [max(vols, key=lambda v: gmsh.model.getBoundingBox(*v)[3])]
        gmsh.model.occ.remove([v for v in gmsh.model.getEntities(3) if v not in vols], recursive=True)
    elif len(vols) > 1:   # bolted halves: treat as bonded
        gmsh.model.occ.fuse(vols[:1], vols[1:])
    gmsh.model.occ.synchronize()
    gmsh.option.setNumber('Mesh.MeshSizeMin', h[0]); gmsh.option.setNumber('Mesh.MeshSizeMax', h[1])
    gmsh.option.setNumber('Mesh.MeshSizeFromCurvature', 12)
    gmsh.model.mesh.generate(3)
    tags, xyz, _ = gmsh.model.mesh.getNodes()
    idx = {t: i for i, t in enumerate(tags)}
    et, en = gmsh.model.mesh.getElementsByType(4)
    tets = np.vectorize(idx.get)(en.reshape(-1, 4))
    gmsh.finalize()
    used = np.unique(tets); remap = -np.ones(len(tags), int); remap[used] = np.arange(len(used))
    return MeshTet(xyz.reshape(-1, 3)[used].T, remap[tets].T)


def von_mises(basis, u, E, nu):
    lam, mu = lame_parameters(E, nu)
    grads = basis.interpolate(u).grad                    # (3, 3, nel, nqp)
    eps = 0.5 * (grads + grads.transpose(1, 0, 2, 3))
    tr = eps[0, 0] + eps[1, 1] + eps[2, 2]
    s = 2 * mu * eps + lam * tr * np.eye(3)[:, :, None, None]
    vm = np.sqrt(0.5 * ((s[0, 0] - s[1, 1])**2 + (s[1, 1] - s[2, 2])**2 + (s[2, 2] - s[0, 0])**2)
                 + 3 * (s[0, 1]**2 + s[1, 2]**2 + s[0, 2]**2))
    return vm.max(axis=1)                                # per element


def solve(m, fixed_nodes, loads, P, body=None):
    """loads: list of (facet selector on facet midpoints, traction fn(x) -> (3, ...)) in N/mm^2."""
    e = ElementVector(ElementTetP2()); ib = Basis(m, e)
    lam, mu = lame_parameters(P['E'], P['nu'])
    K = asm(linear_elasticity(lam, mu), ib)
    f = np.zeros(K.shape[0])
    fmid = m.p[:, m.facets].mean(axis=1)
    for sel, trac in loads:
        facets = np.intersect1d(m.boundary_facets(), np.nonzero(sel(fmid))[0])
        assert facets.size, 'load patch found no faces'
        fb = FacetBasis(m, e, facets=facets)
        @LinearForm
        def lf(v, w):
            return dot(trac(w.x), v)
        f += asm(lf, fb)
    if body is not None:
        @LinearForm
        def bf(v, w):
            return dot(np.array(body)[:, None, None] * np.ones_like(w.x[0]), v)
        f += asm(bf, ib)
    D = ib.get_dofs(lambda x: fixed_nodes(x)).all()
    assert D.size, 'no fixed nodes'
    A, b, x0, I = condense(K, f, D=D)
    X = ib.doflocs[:, I]; comp = np.arange(K.shape[0])[I] % 3   # vector dofs are interleaved x, y, z
    B = np.zeros((len(I), 6))                                  # rigid body modes: what multigrid needs for elasticity
    for k in range(3):
        B[comp == k, k] = 1.0
    B[comp == 0, 3], B[comp == 1, 3] = -X[1, comp == 0], X[0, comp == 1]
    B[comp == 1, 4], B[comp == 2, 4] = -X[2, comp == 1], X[1, comp == 2]
    B[comp == 0, 5], B[comp == 2, 5] = X[2, comp == 0], -X[0, comp == 2]
    print(f'  {len(I)} unknowns', flush=True)
    ml = pyamg.smoothed_aggregation_solver(A, B=B, max_coarse=3000)
    x, info = cg(A, b, M=ml.aspreconditioner(), rtol=1e-8, maxiter=3000)
    assert info == 0, f'solver did not converge ({info})'
    u = np.zeros(K.shape[0]); u[I] = x
    return ib, u


def picture(m, ib, u, vm, title, path, scale):
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    bf = m.boundary_facets(); tri = m.facets[:, bf]; el = m.f2t[0, bf]
    p = m.p + scale * u[ib.nodal_dofs[:, :m.p.shape[1]]]
    fig = plt.figure(figsize=(9, 6)); ax = fig.add_subplot(projection='3d')
    cmap = plt.get_cmap('viridis'); vmax = np.percentile(vm, 99.5)
    ax.add_collection3d(Poly3DCollection(p[:, tri].transpose(2, 1, 0), facecolors=cmap(np.clip(vm[el] / vmax, 0, 1)), linewidths=0))
    lo, hi = p.min(axis=1), p.max(axis=1); c = (lo + hi) / 2; r = (hi - lo).max() / 2
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r); ax.set_axis_off()
    fig.colorbar(plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(0, vmax)), ax=ax, shrink=0.6, label='von Mises, MPa')
    ax.set_title(f'{title} (displacement x{scale:.0f})'); fig.savefig(path, dpi=110, bbox_inches='tight'); plt.close(fig)


def report(name, case, m, ib, u, vm, P, fixed_xyz, extra):
    from scipy.spatial import cKDTree
    far = cKDTree(fixed_xyz.T).query(m.p[:, m.t].mean(axis=1).T)[0] > 3.0
    U = u[ib.nodal_dofs[:, :m.p.shape[1]]]
    r = {'max_vm_MPa': round(float(vm.max()), 2), 'max_vm_3mm_from_supports_MPa': round(float(vm[far].max()), 2),
         'sf_vs_strength': round(P['strength'] / float(vm[far].max()), 2), 'max_disp_mm': round(float(np.linalg.norm(U, axis=0).max()), 2)}
    r.update(extra(U))
    slug = '-'.join(case.split(' ')[:2])
    picture(m, ib, u, vm, f'{name}: {case}', os.path.join(HERE, f'{name}-{slug}.png'), scale=max(1.0, 10.0 / max(r['max_disp_mm'], 1e-6)))
    return r


def run_guard(P):
    m = mesh_step(P['step'], P['keep'], P['h'])
    res = {'material': P['material'], 'E_MPa': P['E'], 'nodes': m.p.shape[1], 'tets': m.t.shape[1]}
    clamp = P['held']
    fixed_xyz = m.p[:, clamp(m.p)]
    r_out = P['r_in'] + P['ring_t']
    for case, (phi, kind, F) in GUARD_CASES(P['auw'], P['between']).items():
        a = math.radians(phi); half = math.radians(4.0)      # load patch: 4 deg either side, on the solid band
        def sel(x, a=a):
            r = np.hypot(x[0], x[1]); ang = np.arctan2(x[1], x[0])
            return (abs(r - r_out) < 0.2) & (abs(((ang - a) + np.pi) % (2 * np.pi) - np.pi) < half) & (x[2] >= P['band'][0]) & (x[2] <= P['band'][1])
        area = 2 * half * r_out * (P['band'][1] - P['band'][0])
        d = {'radial': (-math.cos(a), -math.sin(a), 0.0), 'tangential': (-math.sin(a), math.cos(a), 0.0), 'up': (0.0, 0.0, 1.0)}[kind]
        trac = lambda x, d=d: np.array(d)[:, None, None] * (F / area) * np.ones_like(x[0])
        ib, u = solve(m, clamp, [(sel, trac)], P)
        vm = von_mises(ib, u, P['E'], P['nu'])
        def extra(U):
            th = np.arctan2(m.p[1], m.p[0]); ring = (np.hypot(m.p[0], m.p[1]) > P['r_in'] - 0.5) & (m.p[2] >= P['band'][0])
            inward = -(U[0] * np.cos(th) + U[1] * np.sin(th))
            return {'ring_moves_toward_prop_mm': round(float(inward[ring].max()), 2), 'tip_gap_mm': P['tip_gap'], 'load_N': round(F, 1)}
        res[case] = report(P['name'], case, m, ib, u, vm, P, fixed_xyz, extra)
        print(case, res[case], flush=True)
    return res


def run_carrier(P):
    m = mesh_step(P['step'], P['keep'], P['h'])
    res = {'material': P['material'], 'E_MPa': P['E'], 'nodes': m.p.shape[1], 'tets': m.t.shape[1]}
    L, W = P['clamp_box']
    clamp = lambda x: (x[2] <= 0.05) & np.any([(abs(x[0] - cx) <= L / 2) & (abs(x[1] - cy) <= W / 2) for cx, cy in P['clamps']], axis=0)
    fixed_xyz = m.p[:, clamp(m.p)]
    g = P['g_land']; w = P['wall']; s = P['straps']
    # LiDAR on the wall face (plane y = wall y, facing +y): shear F along +z plus the couple from its centre of mass sitting
    # `offset` off the face, as a linear normal traction over the face (z about the face centroid)
    F = w['lidar_kg'] * G * g
    on_wall = lambda x: (abs(x[1] - w['y']) < 0.05) & (x[0] >= w['x'][0]) & (x[0] <= w['x'][1]) & (x[2] > 4.05)
    fmid = m.p[:, m.facets].mean(axis=1); wf = np.intersect1d(m.boundary_facets(), np.nonzero(on_wall(fmid))[0])
    zs = fmid[2, wf]; zbar = zs.mean(); A = (w['x'][1] - w['x'][0]) * (zs.max() - zs.min()); Iz = A * zs.var()
    alpha = -w['offset'] * F / Iz
    lidar = (on_wall, lambda x: np.array([0 * x[0], alpha * (x[2] - zbar), F / A + 0 * x[0]]))
    # battery hangs in 2 straps that run over the plate's top face between the slots in front of and behind it
    Fb = s['battery_kg'] * G * g; Ab = 2 * (s['x'][1] - s['x'][0]) * s['w']
    on_strap = lambda x: (x[2] <= 0.05) & (x[0] >= s['x'][0]) & (x[0] <= s['x'][1]) & (abs(abs(x[1]) - s['y']) <= s['w'] / 2)
    batt = (on_strap, lambda x: np.array([0 * x[0], 0 * x[0], Fb / Ab + 0 * x[0]]))
    case = f'hard landing {g:g} g'
    ib, u = solve(m, clamp, [lidar, batt], P, body=(0.0, 0.0, P['rho'] * G * g))
    vm = von_mises(ib, u, P['E'], P['nu'])
    extra = lambda U: {'lidar_N': round(F, 1), 'battery_N': round(Fb, 1), 'max_sag_mm': round(float(U[2].max()), 2)}
    res[case] = report('carrier', case, m, ib, u, vm, P, fixed_xyz, extra)
    print(case, res[case], flush=True)
    return res


if __name__ == '__main__':
    name = sys.argv[1]
    P = dict(PARTS[name], name=name)
    if len(sys.argv) > 2:
        P['step'] = sys.argv[2]
    res = (run_carrier if name == 'carrier' else run_guard)(P)
    json.dump(res, open(os.path.join(HERE, f'{name}-results.json'), 'w'), indent=1)
