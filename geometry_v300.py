"""Problem 3 geometry: sector half-planes, convex intersection, diameter, MEC, S2."""

from __future__ import annotations

import math
from typing import Iterable, List, Optional, Sequence, Tuple

Point = Tuple[float, float]
Poly = List[Point]

R0 = 1800.0
R_MIN = 1000.0
R_MAX = 1500.0
DELTA_DEG = 1.0
OPTICAL_R = 20.0
NEAR_R = 5.0
RING_RHO = 1250.0
N_RING = 6
NGON = 64
S2_ALONG = 750.0
S2_LAT = 600.0
EPS = 1e-9


def hypot(p: Point) -> float:
    return math.hypot(p[0], p[1])


def dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def add(a: Point, b: Point) -> Point:
    return (a[0] + b[0], a[1] + b[1])


def sub(a: Point, b: Point) -> Point:
    return (a[0] - b[0], a[1] - b[1])


def scale(a: Point, s: float) -> Point:
    return (a[0] * s, a[1] * s)


def dot(a: Point, b: Point) -> float:
    return a[0] * b[0] + a[1] * b[1]


def cross(a: Point, b: Point) -> float:
    return a[0] * b[1] - a[1] * b[0]


def unit_from_deg(theta_deg: float) -> Point:
    t = math.radians(theta_deg)
    return (math.cos(t), math.sin(t))


def left_normal(u: Point) -> Point:
    return (-u[1], u[0])


def wrap_deg(theta: float) -> float:
    x = theta % 360.0
    return x if x >= 0.0 else x + 360.0


def atan2_deg(y: float, x: float) -> float:
    return wrap_deg(math.degrees(math.atan2(y, x)))


def ring_points(rho: float = RING_RHO, n: int = N_RING) -> List[Point]:
    return [
        (rho * math.cos(2.0 * math.pi * k / n), rho * math.sin(2.0 * math.pi * k / n))
        for k in range(n)
    ]


def survey_points() -> List[Point]:
    return [(0.0, 0.0)] + ring_points()


def covering_worst_outer_distance(rho: float = RING_RHO, n: int = N_RING) -> float:
    """Distance from a ring station to the outer-boundary midpoint."""
    ang = math.pi / n
    return math.hypot(
        R0 - rho * math.cos(ang),
        rho * math.sin(ang),
    )


def regular_ngon(center: Point, radius: float, n: int = NGON) -> Poly:
    """Convex n-gon containing the disk of the given radius."""
    r = radius / math.cos(math.pi / n)
    cx, cy = center
    return [
        (cx + r * math.cos(2.0 * math.pi * i / n), cy + r * math.sin(2.0 * math.pi * i / n))
        for i in range(n)
    ]


def _clip_halfplane(poly: Poly, a: float, b: float, c: float) -> Poly:
    """Keep ax + by + c >= 0."""
    if len(poly) < 3:
        return []
    out: Poly = []
    n = len(poly)

    def val(p: Point) -> float:
        return a * p[0] + b * p[1] + c

    for i in range(n):
        s = poly[i]
        e = poly[(i + 1) % n]
        vs, ve = val(s), val(e)
        s_in = vs >= -EPS
        e_in = ve >= -EPS
        if s_in and e_in:
            out.append(e)
        elif s_in and not e_in:
            t = vs / (vs - ve) if abs(vs - ve) > EPS else 1.0
            t = min(1.0, max(0.0, t))
            out.append((s[0] + t * (e[0] - s[0]), s[1] + t * (e[1] - s[1])))
        elif (not s_in) and e_in:
            t = vs / (vs - ve) if abs(vs - ve) > EPS else 0.0
            t = min(1.0, max(0.0, t))
            out.append((s[0] + t * (e[0] - s[0]), s[1] + t * (e[1] - s[1])))
            out.append(e)
    return _dedup(out)


def _clip_left_of_edge(poly: Poly, a: Point, b: Point) -> Poly:
    """Keep points to the left of directed edge a->b (CCW interior)."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    return _clip_halfplane(poly, -dy, dx, dy * a[0] - dx * a[1])


def _dedup(poly: Poly, tol: float = 1e-7) -> Poly:
    if not poly:
        return []
    out: Poly = []
    for p in poly:
        if not out or dist(p, out[-1]) > tol:
            out.append(p)
    if len(out) >= 2 and dist(out[0], out[-1]) <= tol:
        out.pop()
    return out if len(out) >= 3 else []


def convex_clip(poly: Poly, cutter: Poly) -> Poly:
    if len(poly) < 3 or len(cutter) < 3:
        return []
    q = poly
    m = len(cutter)
    for i in range(m):
        q = _clip_left_of_edge(q, cutter[i], cutter[(i + 1) % m])
        if len(q) < 3:
            return []
    return q


def sector_halfplanes(sensor: Point, theta_deg: float, delta_deg: float = DELTA_DEG):
    """Two half-planes of the closed 2Δ sector. Each is (a, b, c) with ax+by+c >= 0."""
    th_l = theta_deg - delta_deg
    th_r = theta_deg + delta_deg
    cl, sl = math.cos(math.radians(th_l)), math.sin(math.radians(th_l))
    cr, sr = math.cos(math.radians(th_r)), math.sin(math.radians(th_r))
    sx, sy = sensor
    h1 = (-sl, cl, sl * sx - cl * sy)
    h2 = (sr, -cr, -sr * sx + cr * sy)
    return h1, h2


def clip_sector(poly: Poly, sensor: Point, theta_deg: float, delta_deg: float = DELTA_DEG) -> Poly:
    q = poly
    for a, b, c in sector_halfplanes(sensor, theta_deg, delta_deg):
        q = _clip_halfplane(q, a, b, c)
        if len(q) < 3:
            return []
    return q


def clip_disk(poly: Poly, center: Point, radius: float) -> Poly:
    return convex_clip(poly, regular_ngon(center, radius))


def localization_polygon(
    bearings: Sequence[Tuple[Point, float]],
    extra_disks: Optional[Sequence[Tuple[Point, float]]] = None,
    delta_deg: float = DELTA_DEG,
) -> Poly:
    """Intersection of error sectors and range disks, clipped to the arena."""
    poly = regular_ngon((0.0, 0.0), R0)
    for sensor, theta in bearings:
        poly = clip_sector(poly, sensor, theta, delta_deg)
        if not poly:
            return []
        poly = clip_disk(poly, sensor, R_MAX)
        if not poly:
            return []
    if extra_disks:
        for c, r in extra_disks:
            poly = clip_disk(poly, c, r)
            if not poly:
                return []
    return poly


def polygon_area(poly: Poly) -> float:
    if len(poly) < 3:
        return 0.0
    s = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return 0.5 * abs(s)


def centroid(poly: Poly) -> Point:
    if not poly:
        return (0.0, 0.0)
    if len(poly) == 1:
        return poly[0]
    if len(poly) == 2:
        return ((poly[0][0] + poly[1][0]) / 2.0, (poly[0][1] + poly[1][1]) / 2.0)
    a = 0.0
    cx = 0.0
    cy = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        cr = x1 * y2 - x2 * y1
        a += cr
        cx += (x1 + x2) * cr
        cy += (y1 + y2) * cr
    if abs(a) < 1e-12:
        sx = sum(p[0] for p in poly) / n
        sy = sum(p[1] for p in poly) / n
        return (sx, sy)
    return (cx / (3.0 * a), cy / (3.0 * a))


def diameter_and_pair(poly: Poly) -> Tuple[float, Optional[Tuple[Point, Point]]]:
    if len(poly) < 2:
        return 0.0, None
    best = -1.0
    pair = (poly[0], poly[0])
    n = len(poly)
    for i in range(n):
        for j in range(i + 1, n):
            d = dist(poly[i], poly[j])
            if d > best:
                best = d
                pair = (poly[i], poly[j])
    return best, pair


def _circumcircle(a: Point, b: Point, c: Point) -> Optional[Tuple[Point, float]]:
    d = 2.0 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]))
    if abs(d) < 1e-14:
        return None
    a2 = a[0] * a[0] + a[1] * a[1]
    b2 = b[0] * b[0] + b[1] * b[1]
    c2 = c[0] * c[0] + c[1] * c[1]
    ux = (a2 * (b[1] - c[1]) + b2 * (c[1] - a[1]) + c2 * (a[1] - b[1])) / d
    uy = (a2 * (c[0] - b[0]) + b2 * (a[0] - c[0]) + c2 * (b[0] - a[0])) / d
    cen = (ux, uy)
    return cen, dist(cen, a)


def smallest_enclosing_circle(points: Sequence[Point]) -> Tuple[Point, float]:
    pts = [p for p in points]
    n = len(pts)
    if n == 0:
        return (0.0, 0.0), 0.0
    if n == 1:
        return pts[0], 0.0
    best_c, best_r = pts[0], 1e100
    for i in range(n):
        for j in range(i + 1, n):
            c = ((pts[i][0] + pts[j][0]) / 2.0, (pts[i][1] + pts[j][1]) / 2.0)
            r = dist(pts[i], pts[j]) / 2.0
            if r < best_r and all(dist(p, c) <= r + 1e-6 for p in pts):
                best_c, best_r = c, r
    for i in range(n):
        for j in range(i + 1, n):
            for k in range(j + 1, n):
                circ = _circumcircle(pts[i], pts[j], pts[k])
                if circ is None:
                    continue
                c, r = circ
                if r < best_r and all(dist(p, c) <= r + 1e-6 for p in pts):
                    best_c, best_r = c, r
    return best_c, best_r


def s2_candidates(s1: Point, theta_deg: float) -> Tuple[Point, Point]:
    u = unit_from_deg(theta_deg)
    n = left_normal(u)
    plus = add(s1, add(scale(u, S2_ALONG), scale(n, S2_LAT)))
    minus = add(s1, add(scale(u, S2_ALONG), scale(n, -S2_LAT)))
    return plus, minus


def s2_receive_ok(s1: Point, theta_deg: float, s2: Point, dmax: float = R_MAX) -> bool:
    """Conservative: S2 within 1000 m of S1 and of both far sector corners."""
    if dist(s1, s2) > R_MIN + 1e-6:
        return False
    u = unit_from_deg(theta_deg)
    n = left_normal(u)
    cd = math.cos(math.radians(DELTA_DEG))
    sd = math.sin(math.radians(DELTA_DEG))
    for sgn in (1.0, -1.0):
        far = add(s1, add(scale(u, dmax * cd), scale(n, sgn * dmax * sd)))
        if dist(s2, far) > R_MIN + 1e-6:
            return False
    return True


def heard_s2_candidates(s1: Point, theta_deg: float) -> Tuple[Point, Point]:
    """Shorter lateral S2 used AFTER the source was already heard at S1.

    Because R >= dist(S1, G), a 540 m dogleg still reaches every G in the
    1500 m sector; the conservative 1000 m fan-covering point is unnecessary.
    """
    u = unit_from_deg(theta_deg)
    n = left_normal(u)
    plus = add(s1, add(scale(u, 250.0), scale(n, 480.0)))
    minus = add(s1, add(scale(u, 250.0), scale(n, -480.0)))
    return plus, minus


def choose_s2(s1: Point, theta_deg: float, now: Point) -> Point:
    cand = list(heard_s2_candidates(s1, theta_deg)) + list(s2_candidates(s1, theta_deg))
    cand.sort(key=lambda p: (dist(p, now), hypot(p)))
    return cand[0]


def closer_s2_offsets(s1: Point, theta_deg: float, now: Point) -> List[Point]:
    """Heard-source dogleg first, then the conservative 1000 m-safe point."""
    pts: List[Point] = []
    for p in heard_s2_candidates(s1, theta_deg) + s2_candidates(s1, theta_deg):
        if all(dist(p, q) > 5.0 for q in pts):
            pts.append(p)
    pts.sort(key=lambda p: dist(p, now))
    return pts


def ang_diff_deg(a: float, b: float) -> float:
    d = abs(wrap_deg(a) - wrap_deg(b))
    return min(d, 360.0 - d)


def circular_mean_deg(thetas: Sequence[float]) -> float:
    s = sum(math.sin(math.radians(t)) for t in thetas)
    c = sum(math.cos(math.radians(t)) for t in thetas)
    return atan2_deg(s, c)


def cluster_by_angle(items: Sequence[Tuple[float, int]], gap_deg: float = 22.0):
    """Circular clustering of (theta, id). Split at gaps larger than gap_deg."""
    if not items:
        return []
    ordered = sorted(((wrap_deg(th), i) for th, i in items), key=lambda x: x[0])
    n = len(ordered)
    if n == 1:
        return [list(ordered)]
    gaps = []
    for i in range(n):
        a = ordered[i][0]
        b = ordered[(i + 1) % n][0]
        gaps.append((b - a) % 360.0)
    start = (max(range(n), key=lambda i: gaps[i]) + 1) % n
    clusters = []
    cur = [ordered[start]]
    for k in range(1, n):
        prev = (start + k - 1) % n
        i = (start + k) % n
        if gaps[prev] > gap_deg:
            clusters.append(cur)
            cur = [ordered[i]]
        else:
            cur.append(ordered[i])
    clusters.append(cur)
    return clusters


def shared_s2(s1: Point, thetas: Sequence[float], now: Point) -> Optional[Point]:
    """One S2 for several nearby bearings. Always returns a point for a singleton."""
    if not thetas:
        return None
    if len(thetas) == 1:
        return choose_s2(s1, thetas[0], now)
    mean = circular_mean_deg(thetas)
    cand = list(heard_s2_candidates(s1, mean)) + list(s2_candidates(s1, mean))
    cand.sort(key=lambda p: dist(p, now))
    return cand[0]


def nn_order(start: Point, pts: Sequence[Point]) -> List[int]:
    """Nearest-neighbor order of indices, starting from `start`."""
    rem = list(range(len(pts)))
    order: List[int] = []
    cur = start
    while rem:
        j = min(rem, key=lambda i: dist(cur, pts[i]))
        order.append(j)
        cur = pts[j]
        rem.remove(j)
    return order


def third_station(poly: Poly, s_ref: Point) -> Optional[Point]:
    if len(poly) < 3:
        return None
    c = centroid(poly)
    v = sub(c, s_ref)
    if hypot(v) < 1e-6:
        v = (1.0, 0.0)
    n = left_normal(scale(v, 1.0 / hypot(v)))
    return add(c, scale(n, 150.0))


def optical_points(poly: Poly, radius: float = OPTICAL_R, max_pts: int = 10) -> List[Point]:
    """Cover a convex localization polygon by 20 m clearance disks."""
    if not poly:
        return []
    c, r = smallest_enclosing_circle(poly)
    pts: List[Point] = [c]
    if r <= radius + 1e-6:
        return pts
    dlen, pair = diameter_and_pair(poly)
    if pair is not None and dlen > 1e-6:
        a, b = pair
        nsteps = max(1, int(math.ceil(dlen / radius)))
        for i in range(nsteps + 1):
            t = i / nsteps
            p = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
            if all(dist(p, q) > 0.4 * radius for q in pts):
                pts.append(p)
    cen = centroid(poly)
    if all(dist(cen, q) > 0.4 * radius for q in pts):
        pts.append(cen)
    for v in poly:
        if all(dist(v, q) > radius for q in pts):
            pts.append(v)
    return pts[:max_pts]


def disks_cover_arena(centers: Iterable[Point], radius: float = R_MIN, samples: int = 36) -> bool:
    """Check that origin + ring covering is implied, plus a polar sample of the arena."""
    pts = list(survey_points())
    for k in range(samples):
        ang = 2.0 * math.pi * k / samples
        pts.append((R0 * math.cos(ang), R0 * math.sin(ang)))
        pts.append((0.5 * R0 * math.cos(ang), 0.5 * R0 * math.sin(ang)))
        pts.append((1500.0 * math.cos(ang), 1500.0 * math.sin(ang)))
    for p in pts:
        if hypot(p) > R0 + 1e-6:
            continue
        if all(dist(p, c) > radius + 1e-6 for c in centers):
            return False
    return True
