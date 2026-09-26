"""
Local finite element verification (entirely on this computer: gmsh + scikit-fem, no internet).

Experiment: one link as a cantilever. The O4 hole (x = 0) is fixed, the B hole (x = L) carries a total
force F in +y (spread over the hole surface). In-plane bending.

Comparison (hand calculation, same relations as strength.py):
  tip deflection  delta = F L^3 / (3 E I) + F L / (kappa G A)   (Euler-Bernoulli + Timoshenko shear)
  mid section     sigma = M c / I,  M = F (L - x0),  c = w/2    (far from hole and support: a clean comparison)

Units: mm, N, MPa. Element: quadratic tetrahedron (P2), which avoids the artificial stiffness (locking)
of linear elements in thin plates.
Note: this module runs in normal Python, not inside Fusion (needs numpy / scipy / gmsh).
"""

import math
import os
import tempfile

import numpy as np


def mesh_link(L, w, t, d, h, d1=None, D0=None, D1=None):
    """Link geometry with gmsh (two end bosses + web + two holes) and a tetrahedral mesh. Returns the msh path.
    d: hole at x = 0 (shaft), d1: hole at x = L (pin; d if not given).
    D0, D1: boss diameters (w if not given: a plain bar with round ends)."""
    import gmsh
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.add("link")
        occ = gmsh.model.occ
        body = occ.addBox(0, -w / 2, 0, L, w, t)
        end0 = occ.addCylinder(0, 0, 0, 0, 0, t, (D0 or w) / 2)
        end1 = occ.addCylinder(L, 0, 0, 0, 0, t, (D1 or w) / 2)
        fused, _ = occ.fuse([(3, body)], [(3, end0), (3, end1)])
        hole0 = occ.addCylinder(0, 0, 0, 0, 0, t, d / 2)
        hole1 = occ.addCylinder(L, 0, 0, 0, 0, t, (d1 or d) / 2)
        occ.cut(fused, [(3, hole0), (3, hole1)])
        occ.synchronize()
        gmsh.option.setNumber("Mesh.MeshSizeMax", h)
        gmsh.option.setNumber("Mesh.MeshSizeMin", h / 3)
        gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 12)   # fine mesh around the holes
        gmsh.model.mesh.generate(3)
        path = os.path.join(tempfile.gettempdir(), f"link_{os.getpid()}.msh")
        gmsh.option.setNumber("Mesh.MshFileVersion", 2.2)
        gmsh.write(path)
        return path
    finally:
        gmsh.finalize()


def hand_calc(L, w, t, d, F, E, nu, x0=None):
    x0 = L / 2 if x0 is None else x0
    I = t * w ** 3 / 12.0
    G = E / (2 * (1 + nu))
    d_b = F * L ** 3 / (3 * E * I)
    d_s = F * L / ((5 / 6) * G * w * t)
    return {"delta_bending": d_b, "delta_shear": d_s, "delta": d_b + d_s,
            "sigma_mid": F * (L - x0) * (w / 2) / I, "x0": x0}


def solve_cantilever(L, w, t, d, F, E, nu, h=None, d_load=None, D0=None, D1=None):
    """P2 linear elastic solution. d: fixed (shaft) hole, d_load: loaded (pin) hole,
    D0/D1: boss diameters. Returns tip deflection, mid-section stress, largest von Mises, mesh info."""
    import skfem
    from scipy.sparse.linalg import spsolve
    from skfem.models.elasticity import lame_parameters, linear_elasticity

    d_load = d_load or d
    h = h or min(t / 1.5, min(d, d_load) / 2, w / 3)
    msh = mesh_link(L, w, t, d, h, d_load, D0, D1)
    mesh = skfem.MeshTet.load(msh)
    os.remove(msh)

    elem = skfem.ElementVector(skfem.ElementTetP2())
    basis = skfem.Basis(mesh, elem, intorder=3)
    lam, mu = lame_parameters(E, nu)
    K = skfem.asm(linear_elasticity(lam, mu), basis)

    def on_hole(xc, dh):
        tol = 0.12 * dh                      # flat triangles sit slightly inside the curved hole surface
        return lambda x: np.abs(np.hypot(x[0] - xc, x[1]) - dh / 2) < tol

    fixed = mesh.facets_satisfying(on_hole(0.0, d), boundaries_only=True)
    loaded = mesh.facets_satisfying(on_hole(L, d_load), boundaries_only=True)

    fb = skfem.FacetBasis(mesh, elem, facets=loaded, intorder=3)

    @skfem.Functional
    def area(w_):
        return 1.0 + 0.0 * w_.x[0]

    A_hole = area.assemble(fb)

    @skfem.LinearForm
    def traction(v, w_):
        return (F / A_hole) * v[1]           # total F in +y, spread over the hole surface

    f = traction.assemble(fb)
    D = basis.get_dofs(fixed).all()
    u = skfem.solve(*skfem.condense(K, f, D=D), solver=spsolve)

    # Tip deflection: mean y displacement of the nodes on the loaded hole surface
    load_nodes = np.unique(mesh.facets[:, loaded])
    uy = u[basis.nodal_dofs[1, load_nodes]]
    tip = float(np.mean(uy))

    # Stress at the quadrature points: sigma = 2 mu eps + lam tr(eps) I
    ui = basis.interpolate(u)
    g = ui.grad                               # (3, 3, element, point)
    eps = 0.5 * (g + np.swapaxes(g, 0, 1))
    tr = eps[0, 0] + eps[1, 1] + eps[2, 2]
    sig = 2 * mu * eps
    for i in range(3):
        sig[i, i] = sig[i, i] + lam * tr
    vm = np.sqrt(0.5 * ((sig[0, 0] - sig[1, 1]) ** 2 + (sig[1, 1] - sig[2, 2]) ** 2 +
                        (sig[2, 2] - sig[0, 0]) ** 2) +
                 3 * (sig[0, 1] ** 2 + sig[1, 2] ** 2 + sig[0, 2] ** 2))
    X = basis.global_coordinates().value      # (3, element, point)

    x0 = L / 2
    band = (np.abs(X[0] - x0) < max(h, 1.0))
    sigma_mid = float(np.max(np.abs(sig[0, 0][band])))       # largest |sigma_xx| at the mid section
    far = np.hypot(X[0], X[1]) > 2.0 * d                      # region away from the fixed hole
    return {"tip_delta": tip, "sigma_mid": sigma_mid,
            "vm_max_far": float(np.max(vm[far])), "vm_max_all": float(np.max(vm)),
            "nodes": int(mesh.p.shape[1]), "elements": int(mesh.t.shape[1]), "dofs": int(basis.N),
            "h": float(h)}


def compare(L, w, t, d, F, E, nu, h=None, d_load=None, D0=None, D1=None):
    fea = solve_cantilever(L, w, t, d, F, E, nu, h, d_load, D0, D1)
    hand = hand_calc(L, w, t, d, F, E, nu)
    return {"fea": fea, "hand": hand,
            "err_delta_pct": 100 * (fea["tip_delta"] - hand["delta"]) / hand["delta"],
            "err_delta_eb_pct": 100 * (fea["tip_delta"] - hand["delta_bending"]) / hand["delta_bending"],
            "err_sigma_pct": 100 * (fea["sigma_mid"] - hand["sigma_mid"]) / hand["sigma_mid"]}


def verify_design(lengths, section, load, material, crank_torque=None):
    """For the panel: compare FEA with the hand calculation for the rocker and the crank.
    lengths: {a,b,c,d} mm; section: {hole, links:{Crank,Rocker:{width,thickness}}};
    load: tip load N; material: {E, nu} MPa; crank_torque: largest drive torque N*mm."""
    out = {}
    cases = {"Rocker": (lengths["c"], load)}
    if crank_torque:
        cases["Crank"] = (lengths["a"], crank_torque / lengths["a"])   # perpendicular force at the tip
    from strength import HUB_C, section_with_bosses
    section = section_with_bosses(section)
    for name, (L, F) in cases.items():
        s = section["links"][name]
        d_pin = section["hole"]
        d = section.get("shaft", d_pin)          # the crank and rocker are held by shafts (O2, O4)
        r = compare(L, s["width"], s["thickness"], d, F, material["E"], material["nu"], d_load=d_pin,
                    D0=s["boss0"], D1=s["boss1"])
        # Design model (same as strength.py): bending + shear + hub (shaft) rotation
        d_hub = F * L ** 2 / (HUB_C * material["E"] * s["thickness"] * (d / 2) ** 2)
        model = r["hand"]["delta"] + d_hub
        out[name] = {"L": L, "F": F, "width": s["width"], "thickness": s["thickness"], "hole": d_pin,
                     "shaft": d, "boss0": s["boss0"], "boss1": s["boss1"],
                     "fea_delta": r["fea"]["tip_delta"], "hand_delta": model,
                     "beam_only_delta": r["hand"]["delta"],
                     "fea_sigma_mid": r["fea"]["sigma_mid"], "hand_sigma_mid": r["hand"]["sigma_mid"],
                     "err_delta_pct": 100 * (r["fea"]["tip_delta"] - model) / model,
                     "err_sigma_pct": r["err_sigma_pct"],
                     "vm_max_far": r["fea"]["vm_max_far"], "dofs": r["fea"]["dofs"]}
    return out


if __name__ == "__main__":
    import json
    import sys
    import time
    if "--json" in sys.argv:          # called by the add-in: JSON in on stdin, JSON out
        req = json.loads(sys.stdin.read())
        print(json.dumps(verify_design(**req)))
        sys.exit(0)
    cases = [("Al 6061 rocker (slender)", 60, 9, 3, 3, 49.05, 68900, 0.33),
             ("PETG rocker (short, wide)", 60, 29, 3, 4, 49.05, 2000, 0.38)]
    for name, *args in cases:
        t0 = time.time()
        r = compare(*args)
        f, hc = r["fea"], r["hand"]
        print(f"\n{name}: {f['elements']} elements, {f['dofs']} DOF, {time.time() - t0:.1f} s")
        print(f"  Tip deflection  FEA {f['tip_delta']:.4f} mm | hand (EB) {hc['delta_bending']:.4f} "
              f"| hand (EB+shear) {hc['delta']:.4f}  -> diff {r['err_delta_pct']:+.1f}% "
              f"(vs EB only {r['err_delta_eb_pct']:+.1f}%)")
        print(f"  Mid section     FEA {f['sigma_mid']:.2f} MPa | hand {hc['sigma_mid']:.2f} MPa -> diff {r['err_sigma_pct']:+.1f}%")
        print(f"  von Mises       largest (away from the fixed hole) {f['vm_max_far']:.1f} MPa, "
              f"whole part {f['vm_max_all']:.1f} MPa")
