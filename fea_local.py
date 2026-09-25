"""
Yerel sonlu eleman dogrulamasi (tamamen bu bilgisayarda: gmsh + scikit-fem, internet yok).

Deney: tek uzuv konsol kiris. O4 deligi (x=0) sabit, B deligine (x=L) +y yonde
toplam F (delik yuzeyine yayili). Uzuv duzleminde egilme.

Karsilastirma (el hesabi, strength.py ile ayni bagintilar):
  uc sehimi    delta = F L^3 / (3 E I) + F L / (kappa G A)   (Euler-Bernoulli + Timoshenko kayma)
  orta kesit   sigma = M c / I,  M = F (L - x0),  c = w/2    (delik ve kisittan uzak: temiz karsilastirma)

Birimler: mm, N, MPa. Eleman: kuadratik tetrahedron (P2) — ince plakada dogrusal
elemanlarin yapay sertligini (locking) onler.
Not: Bu modul Fusion icinde degil, normal Python'da calisir (numpy/scipy/gmsh gerekir).
"""

import math
import os
import tempfile

import numpy as np


def mesh_link(L, w, t, d, h):
    """gmsh ile uzuv geometrisi (yuvarlak uclu cubuk + iki delik) ve tetrahedral ag. Dondurur: msh yolu."""
    import gmsh
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.add("link")
        occ = gmsh.model.occ
        body = occ.addBox(0, -w / 2, 0, L, w, t)
        end0 = occ.addCylinder(0, 0, 0, 0, 0, t, w / 2)
        end1 = occ.addCylinder(L, 0, 0, 0, 0, t, w / 2)
        fused, _ = occ.fuse([(3, body)], [(3, end0), (3, end1)])
        hole0 = occ.addCylinder(0, 0, 0, 0, 0, t, d / 2)
        hole1 = occ.addCylinder(L, 0, 0, 0, 0, t, d / 2)
        occ.cut(fused, [(3, hole0), (3, hole1)])
        occ.synchronize()
        gmsh.option.setNumber("Mesh.MeshSizeMax", h)
        gmsh.option.setNumber("Mesh.MeshSizeMin", h / 3)
        gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 12)   # delik cevresinde sik ag
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


def solve_cantilever(L, w, t, d, F, E, nu, h=None):
    """P2 dogrusal elastik cozum. Dondurur: uc sehimi, orta kesit gerilmesi, en buyuk von Mises, ag bilgisi."""
    import skfem
    from scipy.sparse.linalg import spsolve
    from skfem.models.elasticity import lame_parameters, linear_elasticity

    h = h or min(t / 1.5, d / 2)
    msh = mesh_link(L, w, t, d, h)
    mesh = skfem.MeshTet.load(msh)
    os.remove(msh)

    elem = skfem.ElementVector(skfem.ElementTetP2())
    basis = skfem.Basis(mesh, elem, intorder=3)
    lam, mu = lame_parameters(E, nu)
    K = skfem.asm(linear_elasticity(lam, mu), basis)

    tol = 0.12 * d                           # duz ucgenler egri delik yuzeyinden biraz iceride

    def on_hole(xc):
        return lambda x: np.abs(np.hypot(x[0] - xc, x[1]) - d / 2) < tol

    fixed = mesh.facets_satisfying(on_hole(0.0), boundaries_only=True)
    loaded = mesh.facets_satisfying(on_hole(L), boundaries_only=True)

    fb = skfem.FacetBasis(mesh, elem, facets=loaded, intorder=3)

    @skfem.Functional
    def area(w_):
        return 1.0 + 0.0 * w_.x[0]

    A_hole = area.assemble(fb)

    @skfem.LinearForm
    def traction(v, w_):
        return (F / A_hole) * v[1]           # toplam F, +y yonunde, delik yuzeyine yayili

    f = traction.assemble(fb)
    D = basis.get_dofs(fixed).all()
    u = skfem.solve(*skfem.condense(K, f, D=D), solver=spsolve)

    # Uc sehimi: yuklu delik yuzeyindeki dugumlerin ortalama y yer degistirmesi
    load_nodes = np.unique(mesh.facets[:, loaded])
    uy = u[basis.nodal_dofs[1, load_nodes]]
    tip = float(np.mean(uy))

    # Gerilme: kuadratur noktalarinda sigma = 2 mu eps + lam tr(eps) I
    ui = basis.interpolate(u)
    g = ui.grad                               # (3, 3, eleman, nokta)
    eps = 0.5 * (g + np.swapaxes(g, 0, 1))
    tr = eps[0, 0] + eps[1, 1] + eps[2, 2]
    sig = 2 * mu * eps
    for i in range(3):
        sig[i, i] = sig[i, i] + lam * tr
    vm = np.sqrt(0.5 * ((sig[0, 0] - sig[1, 1]) ** 2 + (sig[1, 1] - sig[2, 2]) ** 2 +
                        (sig[2, 2] - sig[0, 0]) ** 2) +
                 3 * (sig[0, 1] ** 2 + sig[1, 2] ** 2 + sig[0, 2] ** 2))
    X = basis.global_coordinates().value      # (3, eleman, nokta)

    x0 = L / 2
    band = (np.abs(X[0] - x0) < max(h, 1.0))
    sigma_mid = float(np.max(np.abs(sig[0, 0][band])))       # orta kesitte en buyuk |sigma_xx|
    far = np.hypot(X[0], X[1]) > 2.0 * d                      # sabit delikten uzak bolge
    return {"tip_delta": tip, "sigma_mid": sigma_mid,
            "vm_max_far": float(np.max(vm[far])), "vm_max_all": float(np.max(vm)),
            "nodes": int(mesh.p.shape[1]), "elements": int(mesh.t.shape[1]), "dofs": int(basis.N),
            "h": float(h)}


def compare(L, w, t, d, F, E, nu, h=None):
    fea = solve_cantilever(L, w, t, d, F, E, nu, h)
    hand = hand_calc(L, w, t, d, F, E, nu)
    return {"fea": fea, "hand": hand,
            "err_delta_pct": 100 * (fea["tip_delta"] - hand["delta"]) / hand["delta"],
            "err_delta_eb_pct": 100 * (fea["tip_delta"] - hand["delta_bending"]) / hand["delta_bending"],
            "err_sigma_pct": 100 * (fea["sigma_mid"] - hand["sigma_mid"]) / hand["sigma_mid"]}


def verify_design(lengths, section, load, material, crank_torque=None):
    """Panel icin: kol ve krank icin FEA ile el hesabini karsilastir.
    lengths: {a,b,c,d} mm; section: {hole, links:{Krank,Kol:{width,thickness}}};
    load: uctaki yuk N; material: {E, nu} MPa; crank_torque: en buyuk tahrik torku N*mm."""
    out = {}
    cases = {"Kol": (lengths["c"], load)}
    if crank_torque:
        cases["Krank"] = (lengths["a"], crank_torque / lengths["a"])   # uctaki dik kuvvet
    from strength import HUB_C
    for name, (L, F) in cases.items():
        s = section["links"][name]
        d = section["hole"]
        r = compare(L, s["width"], s["thickness"], d, F, material["E"], material["nu"])
        # Tasarim modeli (strength.py ile ayni): egilme + kayma + gobek donmesi
        d_hub = F * L ** 2 / (HUB_C * material["E"] * s["thickness"] * (d / 2) ** 2)
        model = r["hand"]["delta"] + d_hub
        out[name] = {"L": L, "F": F, "width": s["width"], "thickness": s["thickness"], "hole": d,
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
    if "--json" in sys.argv:          # eklentiden cagrilir: girdi stdin'den JSON, cikti JSON
        req = json.loads(sys.stdin.read())
        print(json.dumps(verify_design(**req)))
        sys.exit(0)
    cases = [("Al 6061 kol (ince)", 60, 9, 3, 3, 49.05, 68900, 0.33),
             ("PETG kol (kisa-genis)", 60, 29, 3, 4, 49.05, 2000, 0.38)]
    for name, *args in cases:
        t0 = time.time()
        r = compare(*args)
        f, hc = r["fea"], r["hand"]
        print(f"\n{name}: {f['elements']} eleman, {f['dofs']} serbestlik, {time.time() - t0:.1f} s")
        print(f"  Uc sehimi   FEA {f['tip_delta']:.4f} mm | el (EB) {hc['delta_bending']:.4f} "
              f"| el (EB+kayma) {hc['delta']:.4f}  -> fark {r['err_delta_pct']:+.1f}% "
              f"(sadece EB'ye gore {r['err_delta_eb_pct']:+.1f}%)")
        print(f"  Orta kesit  FEA {f['sigma_mid']:.2f} MPa | el {hc['sigma_mid']:.2f} MPa -> fark {r['err_sigma_pct']:+.1f}%")
        print(f"  von Mises   en buyuk (sabit delikten uzak) {f['vm_max_far']:.1f} MPa, "
              f"tum parca {f['vm_max_all']:.1f} MPa")
