"""
Kinematik motor v0 — krank-kol (crank-rocker) dort cubuk mekanizmasi.

Isimlendirme (Norton):
    a = krank (O2-A)      b = biyel / coupler (A-B)
    c = kol / rocker (O4-B)   d = sabit uzuv / ground (O2-O4)

Sentez: Krank uzunlugu (a) ve kol salinim acisi (psi) verilir,
zaman orani Q = 1 (hizli donus yok). Norton'un grafik yontemi:
    c = a / sin(psi/2)
    O2, kolun iki uc konumu B1-B2'den gecen dogru uzerinde;
    |O2B1| = b + a (uzanmis), |O2B2| = b - a (katlanmis).
Kalan serbest parametre b; iletim acisi 90 dereceye en yakin kalacak
sekilde taranarak secilir.
"""

import math
import sys

import numpy as np

MU_MIN_DEG = 40.0
MU_MAX_DEG = 140.0


# ---------------------------------------------------------------- kontroller

def grashof(a, b, c, d):
    """Grashof kontrolu ve sinifi. Dondurur: (saglaniyor_mu, sinif)."""
    links = sorted([a, b, c, d])
    s, p, q, l = links[0], links[1], links[2], links[3]
    if s + l > p + q:
        return False, "Grashof DEGIL (hicbir uzuv tam donemez)"
    if math.isclose(s + l, p + q):
        return True, "Ozel Grashof (degisim noktasi var)"
    shortest = min((a, "krank"), (b, "biyel"), (c, "kol"), (d, "sabit"))[1]
    kind = {
        "krank": "krank-kol (crank-rocker)",
        "sabit": "cift krank (double-crank)",
        "biyel": "cift kol (double-rocker)",
        "kol": "krank-kol (kol tarafindan)",
    }[shortest]
    return True, kind


def transmission_extremes(a, b, c, d):
    """Iletim acisinin uc degerleri (derece). Krank sabit uzuvla dogrusal iken olusur."""
    def mu(diag):
        cos_mu = (b * b + c * c - diag * diag) / (2 * b * c)
        return math.degrees(math.acos(max(-1.0, min(1.0, cos_mu))))
    return mu(abs(d - a)), mu(d + a)


# ---------------------------------------------------------------- sentez

def synthesize_crank_rocker(a, psi_deg, mu_design=45.0, b_steps=2000):
    """
    Krank (a) ve salinim (psi) icin dort cubuk sentezi.

    Odunlesim: b uzadikca iletim acisi 90'a yaklasir ama mekanizma buyur.
    Iletim acisini [mu_design, 180 - mu_design] icinde tutan EN KISA b secilir.
    Hicbiri saglamazsa iletim acisi en iyi olan dondurulur.
    """
    psi = math.radians(psi_deg)
    c = a / math.sin(psi / 2)

    # Koordinatlar: O4 orijinde, B1/B2 kolun uc konumlari (x eksenine simetrik)
    bx = c * math.cos(psi / 2)
    by = c * math.sin(psi / 2)

    best = None
    for b in np.linspace(1.05 * a, 15 * a, b_steps):   # kisadan uzuna
        o2 = (bx, -by - (b - a))            # B1-B2 dogrusu uzerinde, B2'nin altinda
        d = math.hypot(*o2)
        mu1, mu2 = transmission_extremes(a, b, c, d)
        worst = max(abs(90 - mu1), abs(90 - mu2))  # 90'dan en buyuk sapma
        cand = dict(a=a, b=b, c=c, d=d, o2=o2, b_ref=(bx, by), mu_min=min(mu1, mu2),
                    mu_max=max(mu1, mu2), worst_dev=worst, size=b + c + d, Q=1.0)
        if worst <= 90 - mu_design:
            return cand                     # hedefi saglayan ilk (en kompakt) cozum
        if best is None or worst < best["worst_dev"]:
            best = cand
    return best


def _cross2(u, v):
    """2B capraz carpim (z bileseni): isaret, donus yonunu verir."""
    return u[0] * v[1] - u[1] * v[0]


def synthesize_quick_return(a, psi_deg, Q, mu_design=45.0, n=3600):
    """
    Hizli donuslu (Q > 1) krank-kol sentezi (Norton).

    alpha = 180 (Q-1)/(Q+1). O2, B1-B2 kirisini alpha acisiyla goren
    cember uzerindedir. Uzanmis konum: |O2B1| = b + a, katlanmis: |O2B2| = b - a.
    Birim kol (c=1) ile tasarlanir, sonra krank = a olacak sekilde olceklenir.
    Cember uzerinde taranir; iletim hedefini saglayan EN KOMPAKT secilir.
    """
    if Q <= 1.0:
        return synthesize_crank_rocker(a, psi_deg, mu_design)
    psi = math.radians(psi_deg)
    alpha = math.pi * (Q - 1) / (Q + 1)
    B1 = np.array([math.cos(psi / 2), math.sin(psi / 2)])
    B2 = np.array([math.cos(psi / 2), -math.sin(psi / 2)])
    chord = np.linalg.norm(B1 - B2)
    R = chord / (2 * math.sin(alpha))
    mid = (B1 + B2) / 2

    best = best_ok = None
    for side in (1, -1):
        center = mid + side * np.array([R * math.cos(alpha), 0.0])
        for t in np.linspace(0, 2 * math.pi, n, endpoint=False):
            o2 = center + R * np.array([math.cos(t), math.sin(t)])
            L1, L2 = np.linalg.norm(B1 - o2), np.linalg.norm(B2 - o2)
            if min(L1, L2) < 1e-9:
                continue
            v1, v2 = (B1 - o2) / L1, (B2 - o2) / L2
            ang = math.acos(max(-1.0, min(1.0, float(np.dot(v1, v2)))))
            if abs(ang - alpha) > 1e-6 or abs(L1 - L2) < 1e-6:
                continue                     # cemberin yanlis yayi
            a_u, b_u = abs(L1 - L2) / 2, (L1 + L2) / 2
            # Uzanmis (ext) ve katlanmis (fold) konumlar AYNI montaj kolunda olmali:
            # biyel-kol arasindaki isaretli aci iki konumda da ayni isarette.
            ext, fold = (B1, B2) if L1 > L2 else (B2, B1)
            A_ext = o2 + a_u * (ext - o2) / np.linalg.norm(ext - o2)
            A_fold = o2 - a_u * (fold - o2) / np.linalg.norm(fold - o2)
            s_ext = _cross2(ext - A_ext, -ext)
            s_fold = _cross2(fold - A_fold, -fold)
            if s_ext * s_fold <= 0:
                continue                     # farkli montaj kollari: gercekte olusmaz
            k = a / a_u                      # olcek: krank = a
            b, c, d = b_u * k, k, float(np.linalg.norm(o2)) * k
            if not grashof(a, b, c, d)[0] or min(a, b, c, d) != a:
                continue                     # krank-kol olmali (en kisa = krank)
            mu1, mu2 = transmission_extremes(a, b, c, d)
            worst = max(abs(90 - mu1), abs(90 - mu2))
            cand = dict(a=a, b=b, c=c, d=d, o2=tuple(o2 * k), b_ref=tuple(ext * k),
                        mu_min=min(mu1, mu2), mu_max=max(mu1, mu2),
                        worst_dev=worst, size=b + c + d, Q=Q)
            if worst <= 90 - mu_design and (best_ok is None or cand["size"] < best_ok["size"]):
                best_ok = cand
            if best is None or worst < best["worst_dev"]:
                best = cand
    return best_ok or best


# ---------------------------------------------------------------- konum analizi

def _circle_intersections(p0, r0, p1, r1):
    x0, y0 = p0
    x1, y1 = p1
    dx, dy = x1 - x0, y1 - y0
    dist = math.hypot(dx, dy)
    if dist > r0 + r1 or dist < abs(r0 - r1):
        return []  # mekanizma bu konuma ulasamiyor (kilitlenme)
    k = (r0 * r0 - r1 * r1 + dist * dist) / (2 * dist)
    h = math.sqrt(max(0.0, r0 * r0 - k * k))
    mx, my = x0 + k * dx / dist, y0 + k * dy / dist
    return [(mx + h * dy / dist, my - h * dx / dist),
            (mx - h * dy / dist, my + h * dx / dist)]


def simulate(m, n=360):
    """Krank 360 derece doner; her adimda A, B noktalari ve iletim acisi."""
    o2 = np.array(m["o2"])
    o4 = np.array([0.0, 0.0])
    ground_angle = math.atan2(-o2[1], -o2[0])  # O2 -> O4 yonu

    # Baslangic: sentezdeki uzanmis konum (krank, kol ucu B_ref'e bakar) -> dogru montaj kolu
    b_ref = m.get("b_ref")
    th0 = 0.0
    if b_ref is not None:
        th0 = math.atan2(b_ref[1] - o2[1], b_ref[0] - o2[0]) - ground_angle

    frames, prev_b = [], b_ref
    for step in np.linspace(0, 2 * math.pi, n, endpoint=False):
        th = (th0 + step) % (2 * math.pi)
        A = o2 + m["a"] * np.array([math.cos(ground_angle + th), math.sin(ground_angle + th)])
        sols = _circle_intersections(A, m["b"], o4, m["c"])
        if not sols:
            raise RuntimeError(f"Mekanizma krank acisi {math.degrees(th):.1f} derecede kilitleniyor")
        # Sureklilik: onceki B'ye en yakin cozumu sec (montaj kolu degismesin)
        if prev_b is None:
            B = np.array(max(sols, key=lambda s: s[0]))
        else:
            B = np.array(min(sols, key=lambda s: math.dist(s, prev_b)))
        prev_b = B
        ab, ob = B - A, B - o4
        cos_mu = np.dot(-ab, -ob) / (np.linalg.norm(ab) * np.linalg.norm(ob))
        mu = math.degrees(math.acos(max(-1.0, min(1.0, cos_mu))))
        rocker = math.degrees(math.atan2(B[1], B[0]))
        frames.append(dict(theta=math.degrees(th), A=A, B=B, mu=mu, rocker=rocker))
    return frames


# ---------------------------------------------------------------- rapor

def measure(frames):
    """Simulasyondan olculen degerler: salinim, zaman orani, iletim acisi araligi."""
    rock = [f["rocker"] for f in frames]
    mus = [f["mu"] for f in frames]
    # Zaman orani: kolun iki uc konumu arasindaki krank donusleri
    n = len(frames)
    i_max, i_min = rock.index(max(rock)), rock.index(min(rock))
    go = ((i_min - i_max) % n) * 360.0 / n
    return dict(swing=max(rock) - min(rock),
                Q=max(go, 360 - go) / min(go, 360 - go),
                mu_min=min(mus), mu_max=max(mus))


def report(m, frames, psi_target):
    ok, kind = grashof(m["a"], m["b"], m["c"], m["d"])
    meas = measure(frames)
    swing, q_meas = meas["swing"], meas["Q"]
    mus = (meas["mu_min"], meas["mu_max"])
    mu_ok = MU_MIN_DEG <= min(mus) and max(mus) <= MU_MAX_DEG

    print("=" * 52)
    print(f"  Krank a  = {m['a']:8.2f} mm")
    print(f"  Biyel b  = {m['b']:8.2f} mm")
    print(f"  Kol   c  = {m['c']:8.2f} mm")
    print(f"  Sabit d  = {m['d']:8.2f} mm")
    print("-" * 52)
    print(f"  Grashof         : {'✓' if ok else '✗'} {kind}")
    print(f"  Salinim (hedef) : {psi_target:.2f} deg")
    print(f"  Salinim (simul.): {swing:.2f} deg")
    print(f"  Zaman orani Q   : hedef {m.get('Q', 1.0):.2f} / simul. {q_meas:.2f}")
    print(f"  Toplam boy      : {m['b'] + m['c'] + m['d']:.1f} mm (b+c+d)")
    print(f"  Iletim acisi    : {min(mus):.1f} .. {max(mus):.1f} deg "
          f"{'✓' if mu_ok else '✗'} (sinir {MU_MIN_DEG:g}..{MU_MAX_DEG:g})")
    print("=" * 52)
    return ok and mu_ok


def export_json(m, frames, passed, path=None):
    """Fusion script'inin okuyacagi dosya: uzuv boylari + baslangic konumu (theta=0)."""
    import json
    import os

    if path is None:
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output", "mechanism.json")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    f0 = frames[0]
    data = {
        "type": "crank_rocker",
        "passed_checks": passed,
        "lengths_mm": {k: round(m[k], 3) for k in ("a", "b", "c", "d")},
        "points_mm": {
            "O2": [round(v, 3) for v in m["o2"]],
            "O4": [0.0, 0.0],
            "A": [round(float(v), 3) for v in f0["A"]],
            "B": [round(float(v), 3) for v in f0["B"]],
        },
    }
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=2)
    print(f"  Kaydedildi: {path}")


def animate(m, frames, save_path=None):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    o2 = m["o2"]
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(12, 5.5))
    xs = [o2[0], 0] + [f["A"][0] for f in frames] + [f["B"][0] for f in frames]
    ys = [o2[1], 0] + [f["A"][1] for f in frames] + [f["B"][1] for f in frames]
    pad = 0.15 * (max(xs) - min(xs))
    ax.set_xlim(min(xs) - pad, max(xs) + pad)
    ax.set_ylim(min(ys) - pad, max(ys) + pad)
    ax.set_aspect("equal")
    ax.set_title("Dort cubuk mekanizmasi")
    ax.plot([o2[0], 0], [o2[1], 0], "k--", lw=1)
    ax.plot([f["B"][0] for f in frames], [f["B"][1] for f in frames], ":", color="gray")
    ax.plot(*o2, "ks", ms=8)
    ax.plot(0, 0, "ks", ms=8)
    crank, = ax.plot([], [], "-o", lw=4, color="tab:red", label="krank a")
    coupler, = ax.plot([], [], "-o", lw=4, color="tab:blue", label="biyel b")
    rocker, = ax.plot([], [], "-o", lw=4, color="tab:green", label="kol c")
    ax.legend(loc="upper left")

    curve = sorted((f["theta"], f["mu"]) for f in frames)
    ax2.plot([p[0] for p in curve], [p[1] for p in curve], color="tab:purple")
    ax2.axhspan(MU_MIN_DEG, MU_MAX_DEG, color="tab:green", alpha=0.1, label="guvenli bolge")
    ax2.set_xlabel("Krank acisi (deg)")
    ax2.set_ylabel("Iletim acisi mu (deg)")
    ax2.set_xlim(0, 360)
    ax2.set_ylim(0, 180)
    ax2.legend()
    marker, = ax2.plot([], [], "o", color="tab:purple")

    def update(i):
        f = frames[i]
        A, B = f["A"], f["B"]
        crank.set_data([o2[0], A[0]], [o2[1], A[1]])
        coupler.set_data([A[0], B[0]], [A[1], B[1]])
        rocker.set_data([0, B[0]], [0, B[1]])
        marker.set_data([f["theta"]], [f["mu"]])
        return crank, coupler, rocker, marker

    anim = FuncAnimation(fig, update, frames=len(frames), interval=20, blit=True)
    fig.tight_layout()
    if save_path:
        update(len(frames) // 8)
        fig.savefig(save_path, dpi=90)
    else:
        plt.show()
    return anim


if __name__ == "__main__":
    # Kullanim: python kinematics.py [krank_mm] [salinim_deg] [--png cikti.png]
    argv = sys.argv[1:]
    if "--png" in argv:
        i = argv.index("--png")
        argv = argv[:i] + argv[i + 2:]
    args = [x for x in argv if not x.startswith("--")]
    a = float(args[0]) if len(args) > 0 else 20.0
    psi = float(args[1]) if len(args) > 1 else 60.0
    png = sys.argv[sys.argv.index("--png") + 1] if "--png" in sys.argv else None

    q = float(args[2]) if len(args) > 2 else 1.0
    mech = synthesize_quick_return(a, psi, q)
    frames = simulate(mech)
    passed = report(mech, frames, psi)
    export_json(mech, frames, passed)
    animate(mech, frames, save_path=png)
