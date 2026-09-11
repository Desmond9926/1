"""Offline tests: covering geometry, S2 receive, and full clearance on fake worlds."""

from __future__ import annotations

import math
import sys

import geometry as g
from offline_sim import FakeClient, random_world, worst_outer_world
from robot_q3 import RobotDog


def test_covering():
    d = g.covering_worst_outer_distance()
    assert d < g.R_MIN, d
    assert g.disks_cover_arena(g.survey_points()), "survey points must cover the arena"
    print(f"[ok] covering worst outer distance = {d:.2f} m < 1000")


def test_s2_receive():
    s1 = (0.0, 0.0)
    plus, minus = g.s2_candidates(s1, 0.0)
    assert g.s2_receive_ok(s1, 0.0, plus)
    assert g.s2_receive_ok(s1, 0.0, minus)
    dmax = max(
        g.dist(plus, s1),
        g.dist(plus, (1500.0 * math.cos(math.radians(1)), 1500.0 * math.sin(math.radians(1)))),
        g.dist(plus, (1500.0 * math.cos(math.radians(1)), -1500.0 * math.sin(math.radians(1)))),
    )
    print(f"[ok] S2+ = {plus} dmax_to_fan = {dmax:.2f} m")


def _point_in_poly(p, poly, eps=1e-4):
    # convex CCW: left of every edge
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        if g.cross((b[0] - a[0], b[1] - a[1]), (p[0] - a[0], p[1] - a[1])) < -eps * g.dist(a, b):
            return False
    return True


def test_polygon_contains_source():
    s1 = (0.0, 0.0)
    gsrc = (800.0, 50.0)
    th1 = g.atan2_deg(gsrc[1] - s1[1], gsrc[0] - s1[0]) + 0.4
    s2 = g.choose_s2(s1, th1, s1)
    th2 = g.atan2_deg(gsrc[1] - s2[1], gsrc[0] - s2[0]) - 0.3
    poly = g.localization_polygon([(s1, th1), (s2, th2)])
    assert poly, "empty localization polygon"
    assert _point_in_poly(gsrc, poly), f"source {gsrc} not in poly, C={g.centroid(poly)}"
    c, r = g.smallest_enclosing_circle(poly)
    print(f"[ok] loc poly n={len(poly)} area={g.polygon_area(poly):.1f} MEC r={r:.2f}")


def run_world(name, world):
    client = FakeClient(world)
    summary = RobotDog(client).run()
    missing = [s for s in world.sources if not s["cleared"]]
    n = len(world.sources)
    print(
        f"[{name}] cleared {summary['cleared']}/{n}  "
        f"T={summary['virtual_time_s']:.1f}s  "
        f"avg={summary.get('avg_time_s', float('nan')):.1f}s  "
        f"missing={[(s['channel'], round(s['x'],1), round(s['y'],1)) for s in missing]}"
    )
    return missing


def test_missions():
    failed = []
    miss = run_world("worst_outer", worst_outer_world())
    if miss:
        failed.append("worst_outer")
    for seed in (1, 2, 3, 7, 11, 42):
        world = random_world(n=10 + (seed % 7), seed=seed)
        miss = run_world(f"random seed={seed} n={len(world.sources)}", world)
        if miss:
            failed.append(f"seed{seed}")
    return failed


if __name__ == "__main__":
    test_covering()
    test_s2_receive()
    test_polygon_contains_source()
    failed = test_missions()
    if failed:
        print("FAILED", failed)
        sys.exit(1)
    print("ALL TESTS PASSED")
