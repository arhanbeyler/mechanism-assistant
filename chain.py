"""
G3: roller chain drive (ANSI B29.1 / ISO 606 A series), Shigley ch. 17.

Rated power per strand of a chain with pitch p (in) on a small sprocket of N1 teeth at n1 rev/min (hp):
    link-plate fatigue      H1 = 0.004 N1^1.08 n1^0.9 p^(3 - 0.07 p)
    roller-bushing impact   H2 = 1000 K_r N1^1.5 p^0.8 / n1^1.5      (K_r = 29 for no. 25 and 35, else 17)
    H_tab = min(H1, H2)  (these reproduce Shigley Table 17-20; with the real N1 no K1 correction is needed)
Allowed: H_a = K2 H_tab, K2 = 1, 1.7, 2.5 for 1, 2, 3 strands.  Design power: H_d = K_s H_nom
(service factor: smooth 1.0, moderate shock 1.3, heavy shock 1.5).
Geometry: pitch diameter D = p / sin(180/N), outside diameter p (0.6 + cot(180/N));
    chain length in pitches  L/p = 2C/p + (N1 + N2)/2 + (N2 - N1)^2 / (4 pi^2 C/p), rounded UP to an even number,
    then the exact centre distance  C = p/4 [-A + sqrt(A^2 - 8 ((N2 - N1)/(2 pi))^2)],  A = (N1 + N2)/2 - L/p
Also: small sprocket 17..25 teeth (odd), large at most 120, ratio within 2 %, wrap on the small sprocket
>= 120 deg, chordal speed variation (pi/N)(1/sin(180/N) - 1/tan(180/N)), chain pull vs breaking load (>= 7,
common practice), both steel shafts in torsion and a web between bore and tooth root.
Choice: the smallest sprocket pair (pitch-circle areas x chain width) that passes.
Units outside: mm, N, W, N*mm, rev/min.
"""

import math

import strength

IN = 25.4
HP = 745.7                                        # W
K2 = {1: 1.0, 2: 1.7, 3: 2.5}
SERVICE = {"smooth": 1.0, "moderate": 1.3, "heavy": 1.5}
N1_CHOICES = (17, 19, 21, 23, 25)
N2_MAX = 120
C_PITCHES = 40                                    # default centre distance: 30..50 pitches recommended
STATIC_MIN = 7.0
WRAP_MIN = 120.0
BORES = [8, 10, 12, 15, 17, 20, 25, 30, 35, 40, 45, 50, 60, 70, 80, 90, 100]
# ANSI no.: pitch, roller diameter, width between inner plates, transverse pitch (in), min tensile strength (lbf)
CHAINS = [
    {"no": 25, "iso": "04C", "p": 0.250, "Dr": 0.130, "W": 0.125, "pt": 0.252, "Fu": 780, "Kr": 29},
    {"no": 35, "iso": "06C", "p": 0.375, "Dr": 0.200, "W": 0.188, "pt": 0.399, "Fu": 1760, "Kr": 29},
    {"no": 40, "iso": "08A", "p": 0.500, "Dr": 0.312, "W": 0.312, "pt": 0.566, "Fu": 3130, "Kr": 17},
    {"no": 50, "iso": "10A", "p": 0.625, "Dr": 0.400, "W": 0.375, "pt": 0.713, "Fu": 4880, "Kr": 17},
    {"no": 60, "iso": "12A", "p": 0.750, "Dr": 0.469, "W": 0.500, "pt": 0.897, "Fu": 7030, "Kr": 17},
    {"no": 80, "iso": "16A", "p": 1.000, "Dr": 0.625, "W": 0.625, "pt": 1.153, "Fu": 12500, "Kr": 17},
    {"no": 100, "iso": "20A", "p": 1.250, "Dr": 0.750, "W": 0.750, "pt": 1.409, "Fu": 19500, "Kr": 17},
    {"no": 120, "iso": "24A", "p": 1.500, "Dr": 0.875, "W": 1.000, "pt": 1.789, "Fu": 28000, "Kr": 17},
    {"no": 140, "iso": "28A", "p": 1.750, "Dr": 1.000, "W": 1.000, "pt": 1.924, "Fu": 38000, "Kr": 17},
    {"no": 160, "iso": "32A", "p": 2.000, "Dr": 1.125, "W": 1.250, "pt": 2.305, "Fu": 50000, "Kr": 17},
]


def chain(no):
    return next(c for c in CHAINS if c["no"] == no)


def rated_hp(ch, N1, n1):
    """Rated power of one strand (hp) and which limit governs."""
    p = ch["p"]
    h1 = 0.004 * N1 ** 1.08 * n1 ** 0.9 * p ** (3 - 0.07 * p)
    h2 = 1000 * ch["Kr"] * N1 ** 1.5 * p ** 0.8 / n1 ** 1.5
    return (h1, "link plate") if h1 <= h2 else (h2, "roller-bushing")


def pitch_diameter(p, N):
    return p / math.sin(math.pi / N)


def outside_diameter(p, N):
    return p * (0.6 + 1 / math.tan(math.pi / N))


def links(C, p, N1, N2):
    """Chain length in pitches for centre distance C (same unit as p), not rounded."""
    c = C / p
    return 2 * c + (N1 + N2) / 2 + (N2 - N1) ** 2 / (4 * math.pi ** 2 * c)


def centre(L, p, N1, N2):
    """Exact centre distance for a chain of L pitches."""
    A = (N1 + N2) / 2 - L
    return p / 4 * (-A + math.sqrt(A * A - 8 * ((N2 - N1) / (2 * math.pi)) ** 2))


def chordal(N):
    a = math.pi / N
    return a * (1 / math.sin(a) - 1 / math.tan(a))


def wrap_deg(D1, D2, C):
    return 180 - 2 * math.degrees(math.asin((D2 - D1) / (2 * C)))


def shaft_bore(T_Nmm):
    st = strength.MATERIALS["steel"]
    allow = 0.5 * st["sy"] / st["n"]
    return next((float(d) for d in BORES if 16 * T_Nmm / (math.pi * d ** 3) <= allow), None)


def _web(D, Dr, bore):
    """Bore plus a web of max(3 mm, bore/4) each side must fit inside the root diameter."""
    return (bore + 2 * max(3.0, bore / 4)) / (D - Dr)


def evaluate(ch, strands, N1, N2, n1, P_W, Ks, C_mm=None):
    """All checks of one candidate (small sprocket N1 at n1). Returns (info, checks)."""
    p = ch["p"] * IN
    Dr = ch["Dr"] * IN
    D1, D2 = pitch_diameter(p, N1), pitch_diameter(p, N2)
    L = links(C_mm or C_PITCHES * p, p, N1, N2)
    L = int(math.ceil(L - 1e-9))
    L += L % 2
    C = centre(L, p, N1, N2)
    tab, limit = rated_hp(ch, N1, n1)
    Ha = K2[strands] * tab * HP
    Hd = Ks * P_W
    V = N1 * p * n1 / 60000                               # m/s
    F = Hd / V                                            # N
    Fu = ch["Fu"] * 4.448 * K2[strands]                   # multi-strand breaking load taken like the rating
    T1 = Hd / (2 * math.pi * n1 / 60) * 1000              # N*mm
    T2 = T1 * N2 / N1
    b1, b2 = shaft_bore(T1), shaft_bore(T2)
    wrap = wrap_deg(D1, D2, C)
    checks = [
        {"group": "Rating", "name": "Chain rating (Shigley 17-20)", "util": Hd / Ha,
         "detail": f"{Hd / 1000:.2f} kW design vs {Ha / 1000:.2f} kW allowed ({limit} limit)"},
        {"group": "Rating", "name": "Chain pull vs breaking load", "util": STATIC_MIN * F / Fu,
         "detail": f"{F:.0f} N, breaking {Fu / 1000:.1f} kN (x{Fu / F:.1f}, at least {STATIC_MIN:g})"},
        {"group": "Geometry", "name": "Wrap on the small sprocket", "util": WRAP_MIN / wrap,
         "detail": f"{wrap:.0f}° (at least {WRAP_MIN:g}°)"},
        {"group": "Geometry", "name": "Sprockets clear each other", "util": (outside_diameter(p, N1) + outside_diameter(p, N2)) / 2 / (C - p),
         "detail": f"C = {C:.1f} mm"},
    ]
    for name, T, b, D in (("Small", T1, b1, D1), ("Large", T2, b2, D2)):
        if b is None:
            checks.append({"group": "Shafts", "name": f"{name} sprocket shaft", "util": 9.9,
                           "detail": f"{T / 1000:.0f} N·m is beyond Ø{BORES[-1]}"})
        else:
            checks.append({"group": "Shafts", "name": f"{name} sprocket shaft and hub", "util": _web(D, Dr, b),
                           "detail": f"Ø{b:g} steel shaft, {T / 1000:.1f} N·m, root Ø{D - Dr:.1f} mm"})
    info = {"no": ch["no"], "iso": ch["iso"], "p": p, "Dr": Dr, "W": ch["W"] * IN, "pt": ch["pt"] * IN,
            "strands": strands, "N1": N1, "N2": N2, "D1": D1, "D2": D2,
            "OD1": outside_diameter(p, N1), "OD2": outside_diameter(p, N2), "links": L, "C": C,
            "V": V, "F": F, "H_d": Hd, "H_a": Ha, "limit": limit, "T1": T1, "T2": T2, "bore1": b1, "bore2": b2,
            "wrap": wrap, "chordal": chordal(N1), "Ks": Ks}
    return info, checks


def design(P_W, n_in, ratio, Ks=1.0, C_mm=None, strands_max=3):
    """Chain drive for power P_W at n_in rev/min, speed ratio n_in / n_out. Returns (design, checks)."""
    small_in = ratio >= 1
    i = ratio if small_in else 1 / ratio
    n1 = n_in if small_in else n_in * i                  # the small sprocket turns i times faster
    best = best_any = None
    for ch in CHAINS:
        for strands in range(1, strands_max + 1):
            for N1 in N1_CHOICES:
                N2 = int(round(N1 * i))
                if N2 > N2_MAX or N2 < N1 or abs(N2 / N1 - i) > 0.02 * i:
                    continue
                info, checks = evaluate(ch, strands, N1, N2, n1, P_W, Ks, C_mm)
                worst = max(c["util"] for c in checks)
                size = (info["D1"] ** 2 + info["D2"] ** 2) * (info["W"] + (strands - 1) * info["pt"])
                cand = (dict(info, size=size, small_is_input=small_in, n1=n1, ratio=N2 / N1 if small_in else N1 / N2),
                        checks, worst)
                if worst <= 1.0:
                    if best is None or size < best[0]["size"]:
                        best = cand
                elif best_any is None or worst < best_any[2]:
                    best_any = cand
    if best:
        return best[0], best[1]
    return None, (best_any[1] if best_any else [])


# ---------------------------------------------------------------- geometry for drawing
FLANK = math.radians(20.0)                        # flank opening of the tooth gap (simplified ANSI form)
PLATE_H = 0.95                                    # link plate height, in pitches
PLATE_T = 0.125                                   # one link plate's thickness, in pitches


def seat_radius(Dr):
    """ANSI roller seat radius 0.5025 Dr + 0.0015 in."""
    return 0.5025 * Dr + 0.0015 * IN


def tooth_gap(p, N, Dr, flank=FLANK):
    """One tooth gap centred on +x (pitch point (R, 0)): the roller seat arc and two straight flanks
    tangent to it, opening outward, run past the outside circle. Returns a dict of points (mm) and the
    tooth's top land width at the outside circle."""
    R, Ro, rs = pitch_diameter(p, N) / 2, outside_diameter(p, N) / 2, seat_radius(Dr)
    s, c = math.sin(flank), math.cos(flank)
    ends = {}
    for side in (1, -1):
        q = (R - rs * s, side * rs * c)                       # where the seat arc meets the flank
        d = (c, side * s)                                     # flank direction (outward, away from the gap centre)
        # |q + t d| = r  ->  t^2 + 2 t (q.d) + |q|^2 - r^2 = 0
        qd, qq = q[0] * d[0] + q[1] * d[1], q[0] ** 2 + q[1] ** 2
        at = lambda r: -qd + math.sqrt(qd * qd - qq + r * r)
        t_o, t_x = at(Ro), at(Ro + 0.3 * p)
        ends[side] = (q, (q[0] + t_o * d[0], q[1] + t_o * d[1]), (q[0] + t_x * d[0], q[1] + t_x * d[1]))
    half = math.atan2(ends[1][1][1], ends[1][1][0])           # half the gap's angle at the outside circle
    land = (2 * math.pi / N - 2 * half) * Ro
    return {"R": R, "Ro": Ro, "rs": rs, "bottom": (R - rs, 0.0),
            "seat_plus": ends[1][0], "seat_minus": ends[-1][0],
            "out_plus": ends[1][2], "out_minus": ends[-1][2], "land": land}


def open_path(r1, r2, C):
    """Open (uncrossed) drive around circles r1 at (0, 0) and r2 at (C, 0): tangent points of the upper and
    lower straight spans, and the far points of both arcs. Offsetting both radii by the same amount keeps
    the spans parallel (the chain's plate band, a belt's faces)."""
    b = math.asin((r2 - r1) / C)
    nx, ny = -math.sin(b), math.cos(b)
    return {"u1": (r1 * nx, r1 * ny), "u2": (C + r2 * nx, r2 * ny), "l1": (r1 * nx, -r1 * ny),
            "l2": (C + r2 * nx, -r2 * ny), "far1": (-r1, 0.0), "far2": (C + r2, 0.0),
            "length": 2 * C * math.cos(b) + r1 * (math.pi - 2 * b) + r2 * (math.pi + 2 * b)}
