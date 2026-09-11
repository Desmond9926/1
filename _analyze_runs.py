import re
from collections import Counter
from pathlib import Path

text = Path("../test-3.txt").read_text(encoding="utf-8")
runs = re.split(r"python robot_q3.py", text)[1:]
print("n_runs", len(runs))
for i, run in enumerate(runs, 1):
    print("=" * 70)
    print("RUN", i)
    origin_dir = re.findall(r"origin ch(\d+) -> direction", run)
    origin_near = re.findall(r"origin ch(\d+) -> near", run)
    loc = re.findall(r"LocalizeClear ch(\d+)", run)
    success = re.findall(r"clear ch(\d+) at .* -> success", run)
    miss = re.findall(r"clear ch(\d+) at .* -> no_target_in_range", run)
    third = len(re.findall(r"after 3rd DF", run))
    empty = re.findall(r"ch(\d+) empty", run)
    mec = [float(x) for x in re.findall(r"MEC r=([0-9.]+)", run)]
    third_r = [float(x) for x in re.findall(r"after 3rd DF MEC r=([0-9.]+)", run)]
    m = re.search(r"SUMMARY (\{.*\})", run)
    summary = eval(m.group(1)) if m else {}
    st = summary.get("status", {})
    print("origin direction", origin_dir, "n=", len(origin_dir))
    print("origin near", origin_near)
    print("success order", [int(x) for x in success])
    print("missed clears", len(miss), "channels", miss)
    print("3rd DF count", third, "radii after", third_r)
    if mec:
        sm = sorted(mec)
        print("MEC n", len(mec), "min", sm[0], "median", sm[len(sm)//2], "max", sm[-1])
        print("first-shot MEC >20", sum(1 for x in mec if x > 20), ">40", sum(1 for x in mec if x > 40))
    print("empty unique", sorted({int(x) for x in empty}))
    print(
        "SUMMARY cleared",
        summary.get("cleared"),
        "T",
        round(float(summary.get("virtual_time_s", 0)), 1),
        "avg",
        round(float(summary.get("avg_time_s", 0)), 1),
    )
    print("status counts", dict(Counter(st.values())))
    print("leftover", {c: s for c, s in st.items() if s not in ("cleared", "empty")})
    before_p3, _, after_p3 = run.partition("=== Phase III")
    suc2 = re.findall(r"success t=([0-9.]+)", before_p3)
    suc3 = re.findall(r"success t=([0-9.]+)", after_p3)
    print("phase2 successes", len(suc2), "last t", suc2[-1] if suc2 else None)
    print("phase3 successes", len(suc3), "last t", suc3[-1] if suc3 else None)
    rings = re.findall(r"ring\[(\d+)\]", after_p3)
    print("ring visits", rings, "n=", len(rings))
    print("phase3 no_signal measures", len(re.findall(r"-> no_signal", after_p3)))
    print("phase3 direction measures", len(re.findall(r"-> direction", after_p3)))
