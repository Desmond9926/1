"""CUMCM 2026 B Q3: omnidirectional interferer search, DF localization, and clearance."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import traceback
import uuid
from typing import Any, Dict, List, Optional, Tuple
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from geometry import (
    OPTICAL_R,
    R0,
    ang_diff_deg,
    choose_s2,
    closer_s2_offsets,
    cluster_by_angle,
    disks_cover_arena,
    dist,
    localization_polygon,
    nn_order,
    optical_points,
    ring_points,
    shared_s2,
    smallest_enclosing_circle,
    s2_candidates,
    third_station,
    wrap_deg,
)

Point = Tuple[float, float]


def _log(msg: str) -> None:
    print(msg, flush=True)


class HttpClient:
    def __init__(self, base_url: str, robot_id: str, timeout: float = 10.0):
        self.base_url = base_url.rstrip("/")
        self.robot_id = robot_id
        self.timeout = timeout
        self.seq = 0
        self.pos: Point = (0.0, 0.0)
        self.channel = 1
        self.virtual_time = 0.0
        self.log: List[dict] = []

    def _rid(self, prefix: str) -> str:
        self.seq += 1
        return f"{prefix}-{self.seq}-{uuid.uuid4().hex[:8]}"

    def _post(self, path: str, payload: dict, retries: int = 8) -> dict:
        data = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        last_err: Optional[Exception] = None
        for attempt in range(retries):
            req = Request(
                self.base_url + path,
                data=data,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            try:
                with urlopen(req, timeout=self.timeout) as resp:
                    body = resp.read().decode("utf-8")
                    obj = json.loads(body)
                    self.log.append({"path": path, "payload": payload, "response": obj})
                    return obj
            except HTTPError as e:
                raw = e.read().decode("utf-8", errors="replace") if e.fp else ""
                last_err = e
                self.log.append(
                    {"path": path, "payload": payload, "http_error": e.code, "body": raw}
                )
                if e.code in (409, 400, 415):
                    raise
                time.sleep(min(0.5 * (attempt + 1), 3.0))
            except (URLError, TimeoutError, ConnectionError, json.JSONDecodeError) as e:
                last_err = e
                time.sleep(min(0.5 * (attempt + 1), 3.0))
        raise RuntimeError(f"POST {path} failed after retries: {last_err}")

    def _base(self, request_id: str) -> dict:
        return {
            "arena_id": "default",
            "robot_id": self.robot_id,
            "request_id": request_id,
        }

    def enter(self) -> dict:
        rid = self._rid("enter")
        obj = self._post("/enter", self._base(rid))
        if obj.get("accepted") is not True:
            raise RuntimeError(f"enter rejected: {obj}")
        self.pos = (0.0, 0.0)
        self.channel = 1
        self.virtual_time = float(obj.get("virtual_time_s") or 0.0)
        return obj

    def measure(self, x: float, y: float, channel: int) -> dict:
        rid = self._rid("measure")
        payload = self._base(rid)
        payload["position"] = {"x": float(x), "y": float(y)}
        payload["channel"] = int(channel)
        obj = self._post("/measure", payload)
        if obj.get("accepted") is not True:
            raise RuntimeError(f"measure rejected: {obj}")
        self.pos = (float(x), float(y))
        self.channel = int(channel)
        self.virtual_time = float(obj.get("virtual_time_s", self.virtual_time))
        return obj

    def clear(self, x: float, y: float, channel: int) -> dict:
        rid = self._rid("clear")
        payload = self._base(rid)
        payload["position"] = {"x": float(x), "y": float(y)}
        payload["channel"] = int(channel)
        obj = self._post("/clear", payload)
        if obj.get("accepted") is not True:
            raise RuntimeError(f"clear rejected: {obj}")
        self.pos = (float(x), float(y))
        self.virtual_time = float(obj.get("virtual_time_s", self.virtual_time))
        return obj

    def exit(self) -> dict:
        rid = self._rid("exit")
        obj = self._post("/exit", self._base(rid))
        return obj

    def dump_log(self, path: str) -> None:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.log, f, ensure_ascii=False, indent=2)


class ChannelInfo:
    __slots__ = ("status", "bearings", "theta0", "surveyed", "ns_points")

    def __init__(self) -> None:
        self.status = "unknown"  # unknown / tracked / cleared / empty
        self.bearings: List[Tuple[Point, float]] = []
        self.theta0: Optional[float] = None
        self.surveyed: set = set()
        self.ns_points: List[Point] = []


class RobotDog:
    def __init__(self, client: Any):
        self.client = client
        self.ch = {c: ChannelInfo() for c in range(1, 21)}
        self.cleared = 0
        self.ring = ring_points()
        self.survey = [(0.0, 0.0)] + self.ring
        self.ring_done = [False] * 6

    @property
    def pos(self) -> Point:
        return self.client.pos

    def _unknown(self) -> List[int]:
        return [c for c, info in self.ch.items() if info.status == "unknown"]

    def _tracked(self) -> List[int]:
        return [c for c, info in self.ch.items() if info.status == "tracked"]

    def _one_bearing(self) -> List[int]:
        return [c for c, info in self.ch.items() if info.status == "tracked" and len(info.bearings) == 1]

    def _done(self) -> bool:
        if self.cleared >= 16:
            return True
        return all(info.status in ("cleared", "empty") for info in self.ch.values())

    def _circular_channels(self, channels: List[int]) -> List[int]:
        if not channels:
            return []
        cur = getattr(self.client, "channel", 1)
        ordered = sorted(channels)
        start = 0
        for i, c in enumerate(ordered):
            if c >= cur:
                start = i
                break
        else:
            start = 0
        return ordered[start:] + ordered[:start]

    def _credit_survey(self, pos: Point) -> Optional[int]:
        if dist(pos, (0.0, 0.0)) < 5.0:
            return 0
        best, best_d = None, 1e9
        for k, sk in enumerate(self.ring):
            d = dist(pos, sk)
            if d < best_d:
                best, best_d = k, d
        if best is not None and best_d < 380.0:
            return best + 1
        return None

    def _apply_measure(self, channel: int, pos: Point, obj: dict, survey_idx: Optional[int] = None) -> str:
        info = self.ch[channel]
        if survey_idx is None:
            survey_idx = self._credit_survey(pos)
        if survey_idx is not None:
            info.surveyed.add(survey_idx)
            if survey_idx >= 1:
                # a nearby ring point is effectively scanned for this channel
                pass
        result = obj.get("measure_result")
        if info.status in ("cleared", "empty"):
            return result
        if result == "no_signal":
            info.ns_points.append(pos)
            if info.status == "unknown" and (
                disks_cover_arena(info.ns_points) or len(info.surveyed) >= 7
            ):
                info.status = "empty"
                _log(f"  ch{channel} empty")
            return result
        if result == "near":
            info.status = "tracked"
            if info.theta0 is None:
                info.theta0 = 0.0
            return result
        if result == "direction":
            theta = float(obj["svd_deg"])
            info.bearings.append((pos, theta))
            if info.theta0 is None:
                info.theta0 = theta
            info.status = "tracked"
            return result
        return result

    def _measure(self, pos: Point, channel: int, survey_idx: Optional[int] = None) -> str:
        obj = self.client.measure(pos[0], pos[1], channel)
        res = self._apply_measure(channel, pos, obj, survey_idx)
        extra = f" {obj.get('svd_deg')}" if res == "direction" else ""
        _log(
            f"  measure ch{channel} at ({pos[0]:.1f},{pos[1]:.1f}) -> {res}{extra}"
            f" t={self.client.virtual_time:.1f}"
        )
        return res

    def _try_clear(self, pos: Point, channel: int) -> bool:
        obj = self.client.clear(pos[0], pos[1], channel)
        ok = obj.get("clear_result") == "success"
        _log(
            f"  clear ch{channel} at ({pos[0]:.2f},{pos[1]:.2f}) -> "
            f"{obj.get('clear_result')} t={self.client.virtual_time:.1f}"
        )
        if ok:
            self.ch[channel].status = "cleared"
            self.cleared += 1
        return ok

    def _optical_clear(self, channel: int) -> bool:
        info = self.ch[channel]
        if info.status == "cleared":
            return True
        if not info.bearings:
            return self._try_clear(self.pos, channel)
        poly = localization_polygon(info.bearings)
        if poly:
            c, r = smallest_enclosing_circle(poly)
            _log(f"  ch{channel} Omega n={len(poly)} MEC r={r:.2f} C=({c[0]:.1f},{c[1]:.1f})")
            if r > 80.0 and len(info.bearings) < 3:
                s3 = third_station(poly, info.bearings[-1][0])
                if s3 is not None:
                    res = self._measure(s3, channel)
                    if res == "near":
                        return self._try_clear(s3, channel)
                    if res == "direction":
                        poly = localization_polygon(info.bearings)
                        if poly:
                            c, r = smallest_enclosing_circle(poly)
                            _log(f"  after 3rd DF MEC r={r:.2f}")
        shots = optical_points(poly) if poly else [self.pos]
        for p in shots:
            if self._try_clear(p, channel):
                return True
            if poly:
                poly = [v for v in poly if dist(v, p) > OPTICAL_R]
        _log(f"  WARNING: failed to clear ch{channel}")
        return False

    def _second_df_then_clear(self, channel: int, s1: Point, th1: float) -> bool:
        info = self.ch[channel]
        if info.status == "cleared":
            return True
        if len(info.bearings) >= 2:
            return self._optical_clear(channel)
        tried = [p for p, _ in info.bearings]
        for p in closer_s2_offsets(s1, th1, self.pos):
            if any(dist(p, q) < 5.0 for q in tried):
                continue
            res = self._measure(p, channel)
            tried.append(p)
            if res == "near":
                return self._try_clear(p, channel)
            if res == "direction":
                return self._optical_clear(channel)
        return self._optical_clear(channel) if info.bearings else False

    def _batch_from_station(self, s1: Point, channels: List[int]) -> None:
        """Shared second DF for channels that already have a bearing at s1, then TSP-clear."""
        todo = []
        for c in channels:
            info = self.ch[c]
            if info.status != "tracked":
                continue
            if len(info.bearings) >= 2:
                todo.append(c)
                continue
            th = info.theta0 if info.theta0 is not None else 0.0
            todo.append(c)
        if not todo:
            return
        items = []
        for c in todo:
            info = self.ch[c]
            th = info.theta0 if info.theta0 is not None else 0.0
            items.append((th, c))
        clusters = cluster_by_angle(items, gap_deg=35.0)
        ready: List[int] = []
        for cl in clusters:
            if self._done():
                return
            ids = [c for _, c in cl]
            need_df = [c for c in ids if self.ch[c].status == "tracked" and len(self.ch[c].bearings) < 2]
            already = [c for c in ids if self.ch[c].status == "tracked" and len(self.ch[c].bearings) >= 2]
            ready.extend(already)
            if not need_df:
                continue
            thetas = [self.ch[c].theta0 or 0.0 for c in need_df]
            s2 = shared_s2(s1, thetas, self.pos)
            if s2 is None:
                for c in need_df:
                    if self._done():
                        return
                    th = self.ch[c].theta0 or 0.0
                    self._second_df_then_clear(c, s1, th)
                continue
            _log(f"  shared S2 {s2} for {need_df}")
            for c in self._circular_channels(need_df):
                if self.ch[c].status != "tracked" or len(self.ch[c].bearings) >= 2:
                    continue
                res = self._measure(s2, c)
                if res == "near":
                    self._try_clear(s2, c)
                elif res == "no_signal":
                    th = self.ch[c].theta0 or 0.0
                    plus, minus = s2_candidates(s1, th)
                    alt = minus if dist(s2, plus) < dist(s2, minus) else plus
                    res2 = self._measure(alt, c)
                    if res2 == "near":
                        self._try_clear(alt, c)
                    elif res2 != "direction":
                        self._second_df_then_clear(c, s1, th)
            ready.extend(
                [c for c in need_df if self.ch[c].status == "tracked" and len(self.ch[c].bearings) >= 2]
            )
        ready = [c for c in dict.fromkeys(ready) if self.ch[c].status == "tracked"]
        if not ready:
            return
        pts = []
        ids = []
        for c in ready:
            poly = localization_polygon(self.ch[c].bearings)
            if not poly:
                ids.append(c)
                pts.append(self.pos)
                continue
            cen, _ = smallest_enclosing_circle(poly)
            ids.append(c)
            pts.append(cen)
        order = nn_order(self.pos, pts)
        for j in order:
            if self._done():
                return
            c = ids[j]
            if self.ch[c].status != "tracked":
                continue
            self._optical_clear(c)

    def localize_clear(self, channel: int) -> bool:
        info = self.ch[channel]
        if info.status == "cleared":
            return True
        _log(f"LocalizeClear ch{channel} from {self.pos} bearings={len(info.bearings)}")
        if not info.bearings:
            if self._try_clear(self.pos, channel):
                return True
            s1, th1 = self.pos, (info.theta0 or 0.0)
        else:
            s1, th1 = info.bearings[0]
        return self._second_df_then_clear(channel, s1, th1)

    def phase_origin_scan(self) -> None:
        _log("=== Phase I: origin scan ===")
        for c in range(1, 21):
            if self._done():
                return
            res = self._measure((0.0, 0.0), c, survey_idx=0)
            if res == "near":
                self._try_clear((0.0, 0.0), c)

    def _tsp_clear(self, channels: List[int]) -> None:
        ready = [c for c in channels if self.ch[c].status == "tracked" and len(self.ch[c].bearings) >= 2]
        if not ready:
            return
        pts = []
        ids = []
        for c in ready:
            poly = localization_polygon(self.ch[c].bearings)
            cen = smallest_enclosing_circle(poly)[0] if poly else self.pos
            ids.append(c)
            pts.append(cen)
        for j in nn_order(self.pos, pts):
            if self._done():
                return
            c = ids[j]
            if self.ch[c].status == "tracked":
                self._optical_clear(c)

    def phase_cluster_s2(self) -> None:
        _log("=== Phase II: clustered second DF then TSP clear ===")
        tracked = self._one_bearing()
        if not tracked:
            return
        items = [(self.ch[c].theta0 or 0.0, c) for c in tracked]
        clusters = cluster_by_angle(items, gap_deg=35.0)
        s1 = (0.0, 0.0)
        waypoints = []
        for cl in clusters:
            ids = [c for _, c in cl]
            thetas = [self.ch[c].theta0 or 0.0 for c in ids]
            s2 = shared_s2(s1, thetas, self.pos)
            waypoints.append((s2 if s2 is not None else choose_s2(s1, thetas[0], self.pos), ids))
        order = nn_order(self.pos, [w[0] for w in waypoints])
        for j in order:
            if self._done():
                return
            s2, ids = waypoints[j]
            ids = [c for c in ids if self.ch[c].status == "tracked" and len(self.ch[c].bearings) < 2]
            if not ids:
                continue
            _log(f"  cluster {ids} DF at {s2}")
            for c in self._circular_channels(ids):
                if self.ch[c].status != "tracked" or len(self.ch[c].bearings) >= 2:
                    continue
                res = self._measure(s2, c)
                if res == "near":
                    self._try_clear(s2, c)
                elif res == "no_signal":
                    th = self.ch[c].theta0 or 0.0
                    for alt in closer_s2_offsets(s1, th, self.pos):
                        if dist(alt, s2) < 5.0:
                            continue
                        res2 = self._measure(alt, c)
                        if res2 == "near":
                            self._try_clear(alt, c)
                            break
                        if res2 == "direction":
                            break
        # all second DFs done — clear in one TSP
        self._tsp_clear(self._tracked())
        for c in list(self._one_bearing()):
            if self._done():
                return
            self.localize_clear(c)

    def _ring_tour_order(self) -> List[int]:
        d = [dist(self.pos, sk) for sk in self.ring]
        k0 = min(range(6), key=lambda k: d[k])

        def clen(step: int) -> float:
            s = d[k0]
            idx = k0
            for _ in range(5):
                nxt = (idx + step) % 6
                s += dist(self.ring[idx], self.ring[nxt])
                idx = nxt
            return s

        step = 1 if clen(1) <= clen(-1) else -1
        return [(k0 + i * step) % 6 for i in range(6)]

    def phase_ring(self) -> None:
        _log("=== Phase III: single-pass ring ===")
        for k in self._ring_tour_order():
            if self._done():
                return
            if self.ring_done[k]:
                continue
            unknown = self._unknown()
            need = [c for c in unknown if k + 1 not in self.ch[c].surveyed]
            # also take 2nd DF here for 1-bearing tracks whose bearing faces this ring
            sk = self.ring[k]
            extra_df = []
            for c in self._one_bearing():
                th = self.ch[c].theta0 or 0.0
                ring_ang = wrap_deg(math.degrees(math.atan2(sk[1], sk[0])))
                # skip nearly collinear (bad geometry) and opposite-side far misses
                ad = ang_diff_deg(th, ring_ang)
                if 25.0 <= ad <= 150.0:
                    extra_df.append(c)
            if not need and not extra_df:
                self.ring_done[k] = True
                continue
            _log(f"  ring[{k}] {sk} scan {need} extra_df={extra_df}")
            found = []
            for c in self._circular_channels(need):
                if self.ch[c].status != "unknown":
                    continue
                res = self._measure(sk, c, survey_idx=k + 1)
                if res == "near":
                    self._try_clear(sk, c)
                    if self._done():
                        return
                elif res == "direction":
                    found.append(c)
            for c in self._circular_channels(extra_df):
                if self.ch[c].status != "tracked" or len(self.ch[c].bearings) != 1:
                    continue
                res = self._measure(sk, c, survey_idx=k + 1)
                if res == "near":
                    self._try_clear(sk, c)
                elif res == "direction":
                    found.append(c)
            self.ring_done[k] = True
            new_from_here = [c for c in found if self.ch[c].status == "tracked"]
            if new_from_here:
                # 2nd DF relative to THIS ring station for newly heard sources
                fresh = [c for c in new_from_here if len(self.ch[c].bearings) == 1]
                if fresh:
                    self._batch_from_station(sk, fresh)
                ready = [c for c in new_from_here if self.ch[c].status == "tracked" and len(self.ch[c].bearings) >= 2]
                for c in ready:
                    if self._done():
                        return
                    if self.ch[c].status == "tracked":
                        self._optical_clear(c)
            if self._done():
                return
        for c in self._unknown():
            if disks_cover_arena(self.ch[c].ns_points) or len(self.ch[c].surveyed) >= 7:
                self.ch[c].status = "empty"

    def _clear_nearby(self, radius: float = 750.0) -> None:
        ready = []
        pts = []
        for c in self._tracked():
            if len(self.ch[c].bearings) < 2:
                continue
            poly = localization_polygon(self.ch[c].bearings)
            if not poly:
                continue
            cen, r = smallest_enclosing_circle(poly)
            if dist(self.pos, cen) <= radius or r <= 22.0:
                ready.append(c)
                pts.append(cen)
        if not ready:
            return
        for j in nn_order(self.pos, pts):
            if self._done():
                return
            c = ready[j]
            if self.ch[c].status == "tracked":
                self._optical_clear(c)

    def phase_sweep(self) -> None:
        """One angular tour: origin-cluster S2s and ring points together."""
        _log("=== Phase II: unified angular sweep ===")
        s1 = (0.0, 0.0)
        wps = []
        tracked = self._one_bearing()
        if tracked:
            items = [(self.ch[c].theta0 or 0.0, c) for c in tracked]
            for cl in cluster_by_angle(items, gap_deg=35.0):
                ids = [c for _, c in cl]
                thetas = [self.ch[c].theta0 or 0.0 for c in ids]
                s2 = shared_s2(s1, thetas, self.pos)
                if s2 is None:
                    s2 = choose_s2(s1, thetas[0], self.pos)
                wps.append(("s2", s2, ids, None))
        for k, sk in enumerate(self.ring):
            wps.append(("ring", sk, [], k))
        if not wps:
            return
        # circular sweep: sort by polar angle, start at the nearest waypoint
        wps.sort(key=lambda w: math.atan2(w[1][1], w[1][0]))
        d0 = [dist(self.pos, w[1]) for w in wps]
        i0 = min(range(len(wps)), key=lambda i: d0[i])
        # pick CW vs CCW by looking at the neighbour closer to us
        nwp = len(wps)
        cw = dist(self.pos, wps[(i0 + 1) % nwp][1])
        ccw = dist(self.pos, wps[(i0 - 1) % nwp][1])
        step = 1 if cw <= ccw else -1
        order = [(i0 + step * t) % nwp for t in range(nwp)]
        for j in order:
            if self._done():
                return
            kind, p, ids, k = wps[j]
            if kind == "s2":
                ids = [c for c in ids if self.ch[c].status == "tracked" and len(self.ch[c].bearings) < 2]
                if not ids:
                    continue
                _log(f"  sweep S2 {p} ch={ids}")
                for c in self._circular_channels(ids):
                    if self.ch[c].status != "tracked" or len(self.ch[c].bearings) >= 2:
                        continue
                    res = self._measure(p, c)
                    if res == "near":
                        self._try_clear(p, c)
                    elif res == "no_signal":
                        th = self.ch[c].theta0 or 0.0
                        for alt in closer_s2_offsets(s1, th, self.pos):
                            if dist(alt, p) < 5.0:
                                continue
                            res2 = self._measure(alt, c)
                            if res2 in ("near", "direction"):
                                if res2 == "near":
                                    self._try_clear(alt, c)
                                break
                self._clear_nearby(900.0)
            else:
                if k is None or self.ring_done[k]:
                    continue
                unknown = self._unknown()
                need = [c for c in unknown if k + 1 not in self.ch[c].surveyed]
                extra_df = []
                for c in self._one_bearing():
                    th = self.ch[c].theta0 or 0.0
                    ring_ang = wrap_deg(math.degrees(math.atan2(p[1], p[0])))
                    if 15.0 <= ang_diff_deg(th, ring_ang) <= 160.0:
                        extra_df.append(c)
                if not need and not extra_df:
                    self.ring_done[k] = True
                    continue
                _log(f"  sweep ring[{k}] {p} need={need} extra={extra_df}")
                found = []
                for c in self._circular_channels(need):
                    if self.ch[c].status != "unknown":
                        continue
                    res = self._measure(p, c, survey_idx=k + 1)
                    if res == "near":
                        self._try_clear(p, c)
                    elif res == "direction":
                        found.append(c)
                for c in self._circular_channels(extra_df):
                    if self.ch[c].status != "tracked" or len(self.ch[c].bearings) != 1:
                        continue
                    res = self._measure(p, c, survey_idx=k + 1)
                    if res == "near":
                        self._try_clear(p, c)
                    elif res == "direction":
                        found.append(c)
                self.ring_done[k] = True
                fresh = [c for c in found if self.ch[c].status == "tracked" and len(self.ch[c].bearings) == 1]
                if fresh:
                    self._batch_from_station(p, fresh)
                self._clear_nearby(900.0)
        self._tsp_clear(self._tracked())

    def run(self) -> Dict[str, Any]:
        enter_obj = self.client.enter()
        remaining = enter_obj.get("remaining_real_duration_s")
        _log(f"entered remaining_real_duration_s={remaining}")
        try:
            self.phase_origin_scan()
            if not self._done():
                self.phase_sweep()
            for c in list(self._tracked()):
                if self._done():
                    break
                self.localize_clear(c)
            for c in self._unknown():
                if disks_cover_arena(self.ch[c].ns_points) or len(self.ch[c].surveyed) >= 7:
                    self.ch[c].status = "empty"
        finally:
            try:
                ex = self.client.exit()
                _log(f"exit {ex}")
            except Exception as e:
                _log(f"exit failed: {e}")
        summary = {
            "cleared": self.cleared,
            "virtual_time_s": self.client.virtual_time,
            "status": {c: self.ch[c].status for c in range(1, 21)},
        }
        if self.cleared:
            summary["avg_time_s"] = self.client.virtual_time / self.cleared
        _log(f"SUMMARY {summary}")
        return summary


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="CUMCM 2026 B Q3 robot dog")
    p.add_argument("--url", default=os.environ.get("CUMCM_URL", "http://127.0.0.1:2026"))
    p.add_argument("--robot-id", default=os.environ.get("CUMCM_ROBOT_ID", "TEAM_ID"))
    p.add_argument("--log", default="q3_run_log.json")
    p.add_argument("--offline", action="store_true", help="run against the local fake world")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--n-sources", type=int, default=13)
    return p.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    if args.offline:
        from offline_sim import FakeClient, random_world

        world = random_world(n=args.n_sources, seed=args.seed)
        client = FakeClient(world)
        dog = RobotDog(client)
        summary = dog.run()
        missing = [s["channel"] for s in world.sources if not s["cleared"]]
        _log(f"offline missing={missing} n={len(world.sources)}")
        return 0 if not missing else 2

    client = HttpClient(args.url, args.robot_id)
    try:
        summary = RobotDog(client).run()
        return 0
    except Exception:
        traceback.print_exc()
        return 1
    finally:
        try:
            client.dump_log(args.log)
            _log(f"wrote {args.log}")
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
