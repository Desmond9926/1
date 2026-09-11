"""Local fake simulator for Problem 3 strategy tests. Physics matches the statement."""

from __future__ import annotations

import math
import random
from typing import Dict, List, Tuple

from geometry import R0, atan2_deg, dist, wrap_deg

Point = Tuple[float, float]


class World:
    def __init__(self, sources: List[dict]):
        self.sources = sources  # channel, x, y, R, cleared

    def source_on(self, channel: int):
        for s in self.sources:
            if s["channel"] == channel and not s["cleared"]:
                return s
        return None


def _error_deg(channel: int, x: float, y: float) -> float:
    """Deterministic location-dependent error in [-1, 1], fixed at a given point."""
    # round to 1 cm so retries at the same command position share the error
    xr = round(x, 2)
    yr = round(y, 2)
    u = math.sin(channel * 12.989 + xr * 0.017 + yr * 0.031)
    return max(-1.0, min(1.0, u))


class FakeClient:
    def __init__(self, world: World):
        self.world = world
        self.pos: Point = (0.0, 0.0)
        self.channel = 1
        self.virtual_time = 0.0
        self.log: List[dict] = []

    def enter(self) -> dict:
        self.pos = (0.0, 0.0)
        self.channel = 1
        self.virtual_time = 0.0
        return {
            "accepted": True,
            "virtual_time_s": 0.0,
            "remaining_real_duration_s": 1200,
        }

    def measure(self, x: float, y: float, channel: int) -> dict:
        p = (float(x), float(y))
        move = dist(self.pos, p) / 5.0
        sw = 1.0 if int(channel) != self.channel else 0.0
        self.virtual_time += move + sw + 5.0
        self.pos = p
        self.channel = int(channel)
        src = self.world.source_on(int(channel))
        if src is None:
            res = {"accepted": True, "virtual_time_s": self.virtual_time, "measure_result": "no_signal"}
            self.log.append(res)
            return res
        d = dist(p, (src["x"], src["y"]))
        if d <= 5.0:
            res = {"accepted": True, "virtual_time_s": self.virtual_time, "measure_result": "near"}
            self.log.append(res)
            return res
        if d <= src["R"] + 1e-9:
            true = atan2_deg(src["y"] - p[1], src["x"] - p[0])
            svd = wrap_deg(true + _error_deg(int(channel), p[0], p[1]))
            res = {
                "accepted": True,
                "virtual_time_s": self.virtual_time,
                "measure_result": "direction",
                "svd_deg": round(svd, 2),
            }
            self.log.append(res)
            return res
        res = {"accepted": True, "virtual_time_s": self.virtual_time, "measure_result": "no_signal"}
        self.log.append(res)
        return res

    def clear(self, x: float, y: float, channel: int) -> dict:
        p = (float(x), float(y))
        move = dist(self.pos, p) / 5.0
        src = self.world.source_on(int(channel))
        if src is not None and dist(p, (src["x"], src["y"])) <= 20.0 + 1e-9:
            src["cleared"] = True
            self.virtual_time += move + 5.0
            self.pos = p
            return {"accepted": True, "virtual_time_s": self.virtual_time, "clear_result": "success"}
        self.virtual_time += move + 3.0
        self.pos = p
        return {
            "accepted": True,
            "virtual_time_s": self.virtual_time,
            "clear_result": "no_target_in_range",
        }

    def exit(self) -> dict:
        return {
            "accepted": True,
            "virtual_time_s": self.virtual_time,
            "exit_reason": "user_exit",
        }

    def dump_log(self, path: str) -> None:
        pass


def random_world(n: int = 13, seed: int = 1) -> World:
    rng = random.Random(seed)
    n = max(10, min(16, n))
    channels = rng.sample(range(1, 21), n)
    sources = []
    for ch in channels:
        # uniform in the disk
        r = R0 * math.sqrt(rng.random())
        a = rng.random() * 2.0 * math.pi
        x, y = r * math.cos(a), r * math.sin(a)
        R = rng.uniform(1000.0, 1500.0)
        sources.append({"channel": ch, "x": x, "y": y, "R": R, "cleared": False})
    return World(sources)


def worst_outer_world() -> World:
    """Six sources near the boundary, midway between ring stations, plus inner ones."""
    sources = []
    # outer midpoints at 1790 m, angle 30+60k
    for k in range(6):
        ang = math.radians(30 + 60 * k)
        sources.append({
            "channel": k + 1,
            "x": 1790.0 * math.cos(ang),
            "y": 1790.0 * math.sin(ang),
            "R": 1000.0,
            "cleared": False,
        })
    # inner
    for k, (x, y) in enumerate([(100.0, 0.0), (0.0, 400.0), (-500.0, 200.0), (300.0, -700.0)]):
        sources.append({
            "channel": 10 + k,
            "x": x,
            "y": y,
            "R": 1200.0 + 50 * k,
            "cleared": False,
        })
    return World(sources)
