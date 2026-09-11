import statistics

import robot_q3
from offline_sim import FakeClient, random_world, worst_outer_world
from robot_q3 import RobotDog

robot_q3._log = lambda m: None


def run(name, world):
    client = FakeClient(world)
    s = RobotDog(client).run()
    miss = [x["channel"] for x in world.sources if not x["cleared"]]
    n = len(world.sources)
    avg = s["virtual_time_s"] / s["cleared"] if s["cleared"] else 1e9
    print(
        f"{name:16s} n={n:2d} cleared={s['cleared']:2d} miss={miss} "
        f"T={s['virtual_time_s']:7.1f} avg={avg:6.1f}"
    )
    return avg, miss


def main():
    avgs = []
    a, m = run("worst_outer", worst_outer_world())
    avgs.append(a)
    assert not m
    for seed in range(1, 21):
        n = 10 + (seed % 7)
        a, m = run(f"seed{seed}", random_world(n=n, seed=seed))
        avgs.append(a)
        assert not m, (seed, m)
    print(
        "MEAN AVG",
        round(statistics.mean(avgs), 1),
        "MEDIAN",
        round(statistics.median(avgs), 1),
        "MIN",
        round(min(avgs), 1),
        "MAX",
        round(max(avgs), 1),
    )


if __name__ == "__main__":
    main()
