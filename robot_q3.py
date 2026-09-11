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
    choose_s2,
    closer_s2_offsets,
    dist,
    localization_polygon,
    optical_points,
    ring_points,
    smallest_enclosing_circle,
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
    __slots__ = ("status", "bearings", "theta0", "surveyed")

    def __init__(self) -> None:
        self.status = "unknown"  # unknown / tracked / cleared / empty
        self.bearings: List[Tuple[Point, float]] = []
        self.theta0: Optional[float] = None
        self.surveyed: set = set()  # survey-point indices that measured this channel


class RobotDog:
    def __init__(self, client: Any):
        self.client = client
        self.ch = {c: ChannelInfo() for c in range(1, 21)}
        self.cleared = 0
        self.ring = ring_points()
        self.survey = [(0.0, 0.0)] + self.ring

    @property
    def pos(self) -> Point:
        return self.client.pos

    def _unknown(self) -> List[int]:
        return [c for c, info in self.ch.items() if info.status == "unknown"]

    def _tracked(self) -> List[int]:
        return [c for c, info in self.ch.items() if info.status == "tracked"]

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

    def _apply_measure(self, channel: int, pos: Point, obj: dict, survey_idx: Optional[int] = None) -> str:
        info = self.ch[channel]
        if survey_idx is not None:
            info.surveyed.add(survey_idx)
        result = obj.get("measure_result")
        if info.status in ("cleared", "empty"):
            return result
        if result == "no_signal":
            if info.status == "unknown" and len(info.surveyed) >= 7:
                info.status = "empty"
                _log(f"  ch{channel} empty (covered by 7 survey points)")
            return result
        if result == "near":
            info.status = "tracked"
            info.theta0 = info.theta0 if info.theta0 is not None else 0.0
            return result
        if result == "direction":
            theta = float(obj["svd_deg"])
            info.bearings.append((pos, theta))
            if info.theta0 is None:
                info.theta0 = theta
            info.status = "tracked"
            return result
        return result

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

        tried_stations: List[Point] = [p for p, _ in info.bearings]
        candidates = closer_s2_offsets(s1, th1, self.pos)
        if choose_s2(s1, th1, self.pos) not in candidates:
            candidates.insert(0, choose_s2(s1, th1, self.pos))

        def measure_here(p: Point) -> str:
            obj = self.client.measure(p[0], p[1], channel)
            res = self._apply_measure(channel, p, obj)
            _log(
                f"  measure ch{channel} at ({p[0]:.2f},{p[1]:.2f}) -> {res}"
                + (f" {obj.get('svd_deg')}" if res == "direction" else "")
                + f" t={self.client.virtual_time:.1f}"
            )
            return res

        got_second = len(info.bearings) >= 2
        for p in candidates:
            if any(dist(p, q) < 5.0 for q in tried_stations):
                continue
            res = measure_here(p)
            tried_stations.append(p)
            if res == "near":
                return self._try_clear(p, channel)
            if res == "direction":
                got_second = True
                break
            if res == "no_signal":
                continue

        if info.status == "cleared":
            return True

        poly = localization_polygon(info.bearings) if info.bearings else []
        if poly:
            c, r = smallest_enclosing_circle(poly)
            _log(f"  Omega verts={len(poly)} MEC r={r:.2f} C=({c[0]:.1f},{c[1]:.1f})")
            if r > 40.0 and len(info.bearings) < 3:
                s3 = third_station(poly, info.bearings[-1][0])
                if s3 is not None and dist(s3, self.pos) + r > 1.0:
                    res = measure_here(s3)
                    if res == "near":
                        return self._try_clear(s3, channel)
                    if res == "direction":
                        poly = localization_polygon(info.bearings)
                        if poly:
                            c, r = smallest_enclosing_circle(poly)
                            _log(f"  after 3rd DF MEC r={r:.2f}")

        if info.status == "cleared":
            return True

        shots = optical_points(poly) if poly else []
        if not shots:
            if info.bearings:
                s, th = info.bearings[0]
                u = (math.cos(math.radians(th)), math.sin(math.radians(th)))
                shots = [
                    (s[0] + d * u[0], s[1] + d * u[1])
                    for d in (200.0, 500.0, 800.0, 1100.0, 1400.0)
                    if math.hypot(s[0] + d * u[0], s[1] + d * u[1]) <= R0 + 50.0
                ]
            else:
                shots = [self.pos]

        for p in shots:
            if self._try_clear(p, channel):
                return True
            if poly:
                poly = [v for v in poly if dist(v, p) > OPTICAL_R]
                extra = optical_points(poly) if poly else []
                for q in extra:
                    if dist(q, p) < 1.0:
                        continue
                    if self._try_clear(q, channel):
                        return True

        _log(f"  WARNING: failed to clear ch{channel}")
        return False

    def phase_origin_scan(self) -> None:
        _log("=== Phase I: origin scan ===")
        for c in range(1, 21):
            if self._done():
                return
            obj = self.client.measure(0.0, 0.0, c)
            res = self._apply_measure(c, (0.0, 0.0), obj, survey_idx=0)
            extra = f" {obj.get('svd_deg')}" if res == "direction" else ""
            _log(f"  origin ch{c} -> {res}{extra} t={self.client.virtual_time:.1f}")
            if res == "near":
                self._try_clear((0.0, 0.0), c)

    def phase_clear_tracked(self) -> None:
        _log("=== Phase II: clear origin-tracked sources ===")
        targets = []
        for c in self._tracked():
            th = self.ch[c].theta0
            if th is None:
                th = 0.0
            targets.append((wrap_deg(th), c))
        targets.sort()
        if not targets:
            return
        # start from the target whose S2 is closest to current position
        best_i = 0
        best_d = 1e100
        now = self.pos
        for i, (th, c) in enumerate(targets):
            s1 = (0.0, 0.0)
            p = choose_s2(s1, th, now)
            d = dist(p, now)
            if d < best_d:
                best_d = d
                best_i = i
        ordered = [c for _, c in (targets[best_i:] + targets[:best_i])]
        for c in ordered:
            if self.ch[c].status != "tracked":
                continue
            self.localize_clear(c)
            if self._done():
                return

    def phase_ring(self) -> None:
        _log("=== Phase III: ring survey ===")
        finished = [False] * 6
        guard = 0
        while self._unknown() and not self._done() and guard < 40:
            guard += 1
            now = self.pos
            unknown = self._unknown()
            # pick nearest unfinished ring point that still needs a scan
            cand = []
            for k, sk in enumerate(self.ring):
                if finished[k]:
                    continue
                need = [c for c in unknown if k + 1 not in self.ch[c].surveyed]
                if not need:
                    finished[k] = True
                    continue
                cand.append((dist(now, sk), k, sk, need))
            if not cand:
                # mark remaining unknown as empty if all survey points done
                for c in list(self._unknown()):
                    if len(self.ch[c].surveyed) >= 7:
                        self.ch[c].status = "empty"
                    else:
                        # force remaining unfinished points
                        finished = [False] * 6
                if not self._unknown():
                    break
                if all(finished):
                    for c in self._unknown():
                        self.ch[c].status = "empty"
                    break
                continue
            cand.sort()
            _, k, sk, need = cand[0]
            _log(f"  ring[{k}] {sk} scan {need}")
            hit = False
            for c in self._circular_channels(need):
                if self.ch[c].status != "unknown":
                    continue
                obj = self.client.measure(sk[0], sk[1], c)
                res = self._apply_measure(c, sk, obj, survey_idx=k + 1)
                extra = f" {obj.get('svd_deg')}" if res == "direction" else ""
                _log(f"    ch{c} -> {res}{extra} t={self.client.virtual_time:.1f}")
                if res == "near":
                    self._try_clear(sk, c)
                    hit = True
                    break
                if res == "direction":
                    self.localize_clear(c)
                    hit = True
                    break
            if not hit:
                still = [c for c in self._unknown() if k + 1 not in self.ch[c].surveyed]
                if not still:
                    finished[k] = True
            if self._done():
                return
        for c in self._unknown():
            if len(self.ch[c].surveyed) >= 7:
                self.ch[c].status = "empty"

    def run(self) -> Dict[str, Any]:
        enter_obj = self.client.enter()
        remaining = enter_obj.get("remaining_real_duration_s")
        _log(f"entered remaining_real_duration_s={remaining}")
        try:
            self.phase_origin_scan()
            if not self._done():
                self.phase_clear_tracked()
            if not self._done():
                self.phase_ring()
            # leftover tracked (found during ring but not cleared)
            for c in list(self._tracked()):
                if self._done():
                    break
                self.localize_clear(c)
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
