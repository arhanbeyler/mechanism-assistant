"""Pure geometry of the frame animations (fusion_lib.flexure / snap_fit), tested without Fusion (adsk stubbed)."""

import math
import sys
import types

import pytest

for name in ("adsk", "adsk.core", "adsk.fusion"):
    sys.modules.setdefault(name, types.ModuleType(name))
sys.modules["adsk"].core, sys.modules["adsk"].fusion = sys.modules["adsk.core"], sys.modules["adsk.fusion"]

from fusion_lib import flexure, snap_fit  # noqa: E402

SNAP = {"l": 10.0, "h": 1.2, "w": 6.0, "y": 1.0, "alpha_in": 30.0, "alpha_out": 90.0}


def test_hinge_flat_as_printed():
    pts = flexure.outline(18.0, 0.4, 0.0)
    assert min(z for _, z in pts) == 0.0 and max(z for _, z in pts) == flexure.PANEL_T
    assert max(x for x, _ in pts) == 9.0 + flexure.PANEL


@pytest.mark.parametrize("deg", [45, 90, 180])
def test_hinge_strip_keeps_its_length_and_the_panels_turn(deg):
    L, t, phi = 18.0, 0.4, math.radians(deg)
    pts = flexure.outline(L, t, phi)
    bottom = pts[1:flexure.SEG + 2]                                      # chords of the outer (bottom) arc
    length = sum(math.dist(a, b) for a, b in zip(bottom, bottom[1:]))
    rho = L / phi
    assert length == pytest.approx(2 * (rho + t / 2) * math.sin(phi / (2 * flexure.SEG)) * flexure.SEG, rel=1e-9)
    # right panel's bottom edge turned by phi / 2 from the x axis, left one mirrored
    (x0, z0), (x1, z1) = pts[flexure.SEG + 1], pts[flexure.SEG + 2]
    assert math.degrees(math.atan2(z1 - z0, x1 - x0)) == pytest.approx(deg / 2, abs=1e-9)
    assert pts[0][0] == pytest.approx(-pts[flexure.SEG + 2][0]) and pts[0][1] == pytest.approx(pts[flexure.SEG + 2][1])


def test_snap_beam_bends_enough_for_the_hook_and_springs_back():
    x_ramp, x_top, x_r, x0 = snap_fit.geometry(SNAP)
    assert snap_fit.deflection(SNAP, 0.0) == 0.0                         # home: the catch behind the hook
    assert snap_fit.deflection(SNAP, SNAP["l"] - x0 + 1.0) == 0.0        # clear of the tip
    s = 0.2                                                              # the catch's edge over the hook's top
    d = snap_fit.deflection(SNAP, s)
    for x in (x_r + 0.01, x_top, x_ramp):                                # every hook point under the catch
        if x0 + s <= x <= x_r + s:
            # sampled maximum: the builder adds CLEAR for what the samples miss
            assert snap_fit.hook_height(SNAP, x) - (d + snap_fit.CLEAR) * snap_fit.shape(x / SNAP["l"]) <= 0
    assert d >= SNAP["y"]
