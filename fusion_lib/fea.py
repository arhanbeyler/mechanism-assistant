"""
Fusion FEA ile el hesabi dogrulamasi (sadece Fusion icinde calisir).

Deney: tek uzuv konsol kiris gibi. O4 deligi sabit, B deligine uzva dik F.
El hesabi (strength.py ile ayni bagintilar):
    sehim   delta = F L^3 / (3 E I),  I = t w^3 / 12   (Euler-Bernoulli, kayma ihmal)
    kayma katkisi (Timoshenko): delta_s = F L / (kappa G A),  kappa = 5/6, G = E / (2 (1 + nu))
    gerilme sigma = F L / S_net,  S_net = t (w^3 - d^3) / (6 w)   (delikli kesit, O4'te)
Birimler: Fusion ici uzunluk cm; sim ici kuvvet 0.01 N (kg*cm/s^2); malzeme ozellikleri kPa.
"""

import math

import adsk.core
import adsk.fusion
import adsk.sim

from fusion_lib.fourbar import build_link_body, mm


def hand_calc(L, w, t, d, F, E_mpa, nu):
    I = t * w ** 3 / 12.0
    delta_b = F * L ** 3 / (3 * E_mpa * I)
    G = E_mpa / (2 * (1 + nu))
    delta_s = F * L / ((5.0 / 6.0) * G * w * t)
    S_net = t * (w ** 3 - d ** 3) / (6 * w)
    return {"delta_bending_mm": delta_b, "delta_shear_mm": delta_s,
            "delta_total_mm": delta_b + delta_s, "sigma_root_MPa": F * L / S_net}


def _hole_face(body, x_mm, d):
    """Yerel x konumundaki pim deliginin silindirik yuzeyi."""
    for face in body.faces:
        g = face.geometry
        if isinstance(g, adsk.core.Cylinder) and abs(g.radius - mm(d / 2)) < 1e-4 \
                and abs(g.origin.x - mm(x_mm)) < 1e-4:
            return face
    raise RuntimeError(f"x={x_mm} mm'de delik yuzeyi bulunamadi")


def create_cantilever_study(L, d, F, body=None):
    """Aktif (KAYDEDILMIS) belgede statik gerilme calismasi: O4 deligi sabit, B deligine +Y yonde F.
    Cozum BASLATILMAZ: kullanici Fusion'da Solve -> Local secer (bulut yok)."""
    app = adsk.core.Application.get()
    ui = app.userInterface
    if body is None:
        design = adsk.fusion.Design.cast(app.activeDocument.products.itemByProductType("DesignProductType"))
        body = design.rootComponent.bRepBodies.item(0)
    ui.workspaces.itemById("SimulationEnvironment").activate()
    adsk.doEvents()
    sims = adsk.sim.Simulations.get(app.activeDocument)
    if sims is None:
        raise RuntimeError("Simulations alinamadi (belge kaydedilmis mi, acik diyalog var mi?)")
    study = sims.createStudy("Kol dogrulama", adsk.sim.StudyTypes.LinearStaticStressStudyType)
    case = study.loadCases.item(0)
    case.constraints.add(case.constraints.createFixedConstraintInput([_hole_face(body, 0.0, d)]))
    force = adsk.sim.VectorDefinition.createByVector(adsk.core.Vector3D.create(0, F * 100.0, 0))  # N -> 0.01 N
    case.loads.add(case.loads.createForceLoadInput([_hole_face(body, L, d)], force))
    return study


def setup_cantilever(L, w, t, d, F, material_name="Aluminum 6061"):
    """Yeni belgede uzvu cizer, malzemeyi atar, el hesabini yapar. (Calisma: create_cantilever_study,
    belge kaydedildikten sonra.) Dondurur: (body, el_hesabi)."""
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    body = build_link_body(root, L, {"width": w, "thickness": t, "hole": d})
    body.name = "Kol"

    mat = app.materialLibraries.itemByName("Fusion Material Library").materials.itemByName(material_name)
    body.material = mat
    props = {mat.materialProperties.item(i).name: mat.materialProperties.item(i).value
             for i in range(mat.materialProperties.count)}
    E_mpa = props["Young's Modulus"] / 1000.0          # kPa -> MPa
    nu = props["Poisson's Ratio"]

    calc = hand_calc(L, w, t, d, F, E_mpa, nu)
    calc.update(E_MPa=E_mpa, nu=nu)
    return body, calc
