"""
B9: Sinirli acida calisan dort cubuk — fonksiyon ureteci sentezi (Freudenstein).

Istek: giris uzvu (krank yerine "giris kolu") delta_theta kadar donerken cikis kolu delta_phi kadar
donsun (dogrusal iliski). Giris tam tur ATMAK ZORUNDA DEGIL: sonuc cift kol, uclu kol (Grashof degil)
ya da krank-kol olabilir.

Freudenstein (O2 orijinde, O4 = (d, 0); theta: giris acisi, phi: cikis acisi, x ekseninden):
    K1 cos(phi) - K2 cos(theta) + K3 = cos(theta - phi)
    K1 = d/a,  K2 = d/c,  K3 = (a^2 - b^2 + c^2 + d^2) / (2 a c)
3 hassas nokta (Chebyshev araligi) -> 3x3 dogrusal sistem -> K1, K2, K3 -> uzuv oranlari (d = 1),
sonra giris kolu = a olacak sekilde olceklenir. Serbest baslangic acilari (theta0, phi0) taranir;
her aday BAGIMSIZ simulasyonla dogrulanir (montaj, ayni kol, yapisal hata, iletim acisi).
Sadece standart kutuphane (Fusion icinde de calisir).
"""

import math

import kinematics as kin


def _solve3(M, v):
    """3x3 dogrusal sistem (Cramer)."""
    def det(m):
        return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    D = det(M)
    if abs(D) < 1e-12:
        return None
    out = []
    for j in range(3):
        Mj = [row[:] for row in M]
        for i in range(3):
            Mj[i][j] = v[i]
        out.append(det(Mj) / D)
    return out


def chebyshev(x0, dx, n=3):
    return [x0 + dx / 2 * (1 - math.cos((2 * j - 1) * math.pi / (2 * n))) for j in range(1, n + 1)]


def _position(a, b, c, d, th, prev_B):
    """Giris acisi th icin A ve B (B: onceki B'ye en yakin cozum -> ayni montaj kolu)."""
    A = (a * math.cos(th), a * math.sin(th))
    sols = kin._circle_intersections(A, b, (d, 0.0), c)
    if not sols:
        return A, None
    B = min(sols, key=lambda s: math.dist(s, prev_B)) if prev_B else sols
    return A, B


def simulate_range(a, b, c, d, th0, dth, phi0, n=61):
    """Giris th0 -> th0 + dth. Ilk B, phi0'a en yakin kol. Dondurur: kareler ya da None (kilitlenme)."""
    frames, prev = [], (d + c * math.cos(phi0), c * math.sin(phi0))
    for k in range(n):
        th = th0 + dth * k / (n - 1)
        A, B = _position(a, b, c, d, th, prev)
        if B is None:
            return None
        if math.dist(B, prev) > 0.5 * c and k > 0:      # kol atladi (tekil konumdan gecis)
            return None
        prev = B
        ab = (A[0] - B[0], A[1] - B[1])
        cb = (d - B[0], -B[1])
        cos_mu = (ab[0] * cb[0] + ab[1] * cb[1]) / (b * c)
        mu = math.degrees(math.acos(max(-1.0, min(1.0, cos_mu))))
        phi = math.atan2(B[1], B[0] - d)
        frames.append({"theta": math.degrees(th), "A": A, "B": B, "mu": mu, "phi": phi})
    return frames


def _unwrap(angles):
    out = [angles[0]]
    for x in angles[1:]:
        out.append(out[-1] + (x - out[-1] + math.pi) % (2 * math.pi) - math.pi)
    return out


def candidates(a_len, in_swing_deg, out_swing_deg, mu_design=45.0, step_deg=5, max_ratio=8.0, top=40):
    """Kinematik olarak uygun (hata <= 1 derece, iletim acisi tasarim araliginda) adaylar,
    dengeli orandan kotuye sirali. Mukavemet/katman cakismasi gibi sonraki kontrollerden gecemeyen
    olursa siradaki denenir (pipeline)."""
    out = []
    synthesize(a_len, in_swing_deg, out_swing_deg, mu_design, step_deg, max_ratio, collect=out)
    out.sort(key=lambda m: (round(m["ratio"], 2), m["size"]))
    # birbirine cok benzeyenleri ayikla (komsu tarama noktalari)
    distinct = []
    for m in out:
        if all(max(abs(m[k] - n[k]) for k in "abcd") > 0.05 * m["a"] for n in distinct):
            distinct.append(m)
        if len(distinct) >= top:
            break
    return distinct


def synthesize(a_len, in_swing_deg, out_swing_deg, mu_design=45.0, step_deg=5, max_ratio=8.0, collect=None):
    """
    Dondurur: en iyi tasarim (dict) ya da None.
    Secim: yapisal hata <= 1 derece ve iletim acisi [mu_design, 180-mu_design] saglayanlar icinde
    en DENGELI oranli (en uzun / en kisa uzuv en kucuk; esitlikte kucuk olan). "En kompakt"
    secimi gobeklerin sigmayacagi cok kisa uzuvlar uretiyordu. Uygun yoksa iletim acisi en iyi olan.
    """
    dth, dphi_abs = math.radians(in_swing_deg), math.radians(out_swing_deg)
    best = best_ok = None
    for sign in (1, -1):                                  # cikis ayni ya da ters yonde donebilir
        dphi = sign * dphi_abs
        for i in range(0, 360, step_deg):
            th0 = math.radians(i)
            ths = chebyshev(th0, dth)
            for j in range(0, 360, step_deg):
                phi0 = math.radians(j)
                phis = [phi0 + dphi * (t - th0) / dth for t in ths]
                K = _solve3([[math.cos(p), -math.cos(t), 1.0] for t, p in zip(ths, phis)],
                            [math.cos(t - p) for t, p in zip(ths, phis)])
                if not K or K[0] <= 0 or K[1] <= 0:
                    continue
                d = 1.0
                a, c = d / K[0], d / K[1]
                b2 = a * a + c * c + d * d - 2 * a * c * K[2]
                if b2 <= 0:
                    continue
                b = math.sqrt(b2)
                if max(a, b, c, d) / min(a, b, c, d) > max_ratio:
                    continue
                fr = simulate_range(a, b, c, d, th0, dth, phi0)
                if not fr:
                    continue
                phi_sim = _unwrap([f["phi"] for f in fr])
                # Yapisal hata: simule cikis ile istenen dogrusal iliski arasindaki en buyuk fark
                # (phi_sim[0] ile phi0 arasinda 2*pi farki olabilir: kaydirilir)
                shift = phi_sim[0] - phi0
                err = max(abs(ps - shift - (phi0 + dphi * k / (len(fr) - 1))) for k, ps in enumerate(phi_sim))
                mus = [f["mu"] for f in fr]
                worst = max(abs(90 - min(mus)), abs(90 - max(mus)))
                s = a_len / a
                ratio = max(a, b, c, d) / min(a, b, c, d)
                cand = {"a": a * s, "b": b * s, "c": c * s, "d": d * s, "th0": th0, "phi0": phi0,
                        "dth": dth, "dphi": dphi, "err_deg": math.degrees(err),
                        "mu_min": min(mus), "mu_max": max(mus), "worst_dev": worst,
                        "size": (b + c + d) * s, "ratio": ratio}
                if math.degrees(err) <= 1.0 and worst <= 90 - mu_design:
                    if collect is not None:
                        collect.append(cand)
                    if best_ok is None or (round(ratio, 2), cand["size"]) < (round(best_ok["ratio"], 2), best_ok["size"]):
                        best_ok = cand
                if best is None or (math.degrees(err), worst) < (best["err_deg"], best["worst_dev"]):
                    best = cand
    return best_ok or best


def frames_for(m, n=61):
    """Tasarimi bizim koordinatlara (O4 orijinde) cevirip simule eder.
    Dondurur: (kinematics uyumlu mech, kareler)."""
    a, b, c, d = m["a"], m["b"], m["c"], m["d"]
    fr = simulate_range(a, b, c, d, m["th0"], m["dth"], m["phi0"], n)
    shift = (-d, 0.0)                                     # O4 -> orijin
    out = []
    for f in fr:
        out.append({"theta": f["theta"], "mu": f["mu"],
                    "A": (f["A"][0] + shift[0], f["A"][1] + shift[1]),
                    "B": (f["B"][0] + shift[0], f["B"][1] + shift[1]),
                    "rocker": math.degrees(f["phi"])})
    mech = dict(m, o2=(-d, 0.0), b_ref=out[0]["B"], Q=1.0)
    return mech, out


def measure_range(frames):
    """Simulasyondan: giris ve cikis salinimi, iletim acisi araligi."""
    th = [f["theta"] for f in frames]
    ph = [math.degrees(x) for x in _unwrap([math.radians(f["rocker"]) for f in frames])]
    return {"in_swing": th[-1] - th[0], "out_swing": ph[-1] - ph[0],
            "mu_min": min(f["mu"] for f in frames), "mu_max": max(f["mu"] for f in frames)}
