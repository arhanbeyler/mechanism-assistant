"""C10: yay modulu testleri."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import springs  # noqa: E402

MW = springs.SPRING_MATERIALS["music_wire"]


def test_conical_rate_reduces_to_helical():
    """Konik formul, taper = 1'de silindirik k = d^4 G / (8 D^3 Na) olmali."""
    d, D, Na, G = 2.0, 16.0, 8, 81000.0
    assert springs.rate(d, G, Na, D, 1.0) == pytest.approx(d ** 4 * G / (8 * D ** 3 * Na))


def test_conical_is_stiffer_than_cylinder_of_large_diameter():
    """Kucuk sarimlar daha sert: ayni buyuk capta konik yay silindirikten serttir."""
    assert springs.rate(2.0, 81000.0, 8, 16.0, 0.5) > springs.rate(2.0, 81000.0, 8, 16.0, 1.0)


def test_conical_rate_matches_numeric_integration():
    """k = G d^4 / (16 Na (R1+R2)(R1^2+R2^2)): sarim sarim toplamla (sayisal integral) ayni."""
    d, G, Na, R1, R2 = 2.0, 81000.0, 6, 10.0, 5.0
    n = 60000
    compliance = sum(64 * (R1 + (R2 - R1) * (i + 0.5) / n) ** 3 / (G * d ** 4) * (Na / n) for i in range(n))
    assert springs.rate(d, G, Na, 2 * R1, R2 / R1) == pytest.approx(1 / compliance, rel=1e-6)


def test_bergstrasser_known_value():
    assert springs.bergstrasser(8.0) == pytest.approx(34 / 29)


def test_design_meets_checks_and_target():
    geo, checks = springs.design(100.0, 20.0, MW)
    assert geo is not None
    assert all(c["util"] <= 1.0 for c in checks)
    assert geo["k"] == pytest.approx(5.0, rel=0.05)
    assert 4 <= geo["D1"] / geo["d"] <= 12


def test_solid_compression_does_not_yield():
    """Tam sikismada bile akma olmamali (Shigley'nin 'solid-safe' tasarim ilkesi)."""
    geo, checks = springs.design(100.0, 20.0, MW)
    solid = [c for c in checks if c["name"].startswith("Tam sıkışmada")][0]
    assert solid["util"] <= 1.0


def test_conical_design():
    geo, checks = springs.design(50.0, 15.0, MW, taper=0.5)
    assert geo is not None and geo["D2"] == pytest.approx(0.5 * geo["D1"])
    assert all(c["util"] <= 1.0 for c in checks)


def test_helix_points_geometry():
    geo, _ = springs.design(100.0, 20.0, MW)
    pts = springs.helix_points(geo)
    assert pts[0][2] == pytest.approx(0.0)
    assert pts[-1][2] == pytest.approx(geo["L0"], rel=1e-6)    # taslanmis uclar: serbest boy = L0
    assert all(abs(math.hypot(x, y) - geo["D1"] / 2) < 1e-9 for x, y, _ in pts)


def test_compressed_helix_is_shorter_by_deflection():
    """Animasyon karesi: delta kadar sikismis yayin boyu L0 - delta; en fazla calisma cokmesinde
    aktif sarimlar arasinda hala bosluk kalir (tam sikisma degil)."""
    geo, _ = springs.design(100.0, 20.0, MW)
    pts = springs.helix_points(geo, compression=geo["y_work"])
    assert pts[-1][2] == pytest.approx(geo["L0"] - geo["y_work"], rel=1e-6)
    assert geo["pitch"] - geo["y_work"] / geo["Na"] > geo["d"]      # teller birbirine degmiyor
