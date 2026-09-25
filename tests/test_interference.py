"""Katman cakismasi testleri (B8b)."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import interference  # noqa: E402
import kinematics as kin  # noqa: E402
import strength  # noqa: E402


def test_region_distance_geometry():
    """Govde seridi ve gobeklere mesafe: elle bilinen noktalar."""
    a, b = (0.0, 0.0), (10.0, 0.0)
    assert interference.region_distance((5.0, 4.0), a, b, 2.0, 1.0, 1.0) == pytest.approx(3.0)   # seride
    assert interference.region_distance((-4.0, 0.0), a, b, 2.0, 1.5, 1.0) == pytest.approx(2.5)  # gobege
    assert interference.region_distance((5.0, 0.0), a, b, 2.0, 1.0, 1.0) < 0                      # icinde


def test_typical_design_has_collision_free_order():
    m = kin.synthesize_crank_rocker(30, 60)
    frames = kin.simulate(m)
    order, res, ok = interference.choose_layer_order(m, frames, strength.DEFAULT_SECTION)
    assert ok
    assert all(margin >= 0 for _, _, margin in res)
    assert order[0] == "Sabit"


def test_detects_collision_when_boss_covers_shaft():
    """Yapay durum: krankin gobegi O4'u orter -> ilk sira (O4 krank katmanindan gecer) cakismali."""
    m = kin.synthesize_crank_rocker(30, 60)
    frames = kin.simulate(m)
    d = math.dist(m["o2"], (0.0, 0.0))
    sec = strength.section_with_bosses(strength.DEFAULT_SECTION)
    sec["links"]["Krank"]["boss0"] = 2 * d + 10          # O2 gobegi O4'a kadar uzanir
    res = interference.check_layers(m, frames, sec, interference.LAYER_ORDERS[0])
    o4 = [r for r in res if r[0] == "O4"][0]
    assert o4[2] < 0
