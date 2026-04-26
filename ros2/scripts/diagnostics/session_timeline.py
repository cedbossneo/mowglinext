#!/usr/bin/env python3
import json, sys
with open(sys.argv[1]) as f:
    lines = f.readlines()
samples = [json.loads(l) for l in lines[1:-1] if l.strip()]

# Every ~5s, print fusion sigma and position
print(f"{'t(s)':>6} {'fx':>8} {'fy':>8} {'sigma_xy':>10} {'gps_ok':>6}")
for i in range(0, len(samples), 100):  # 100 samples = 5s at 20Hz
    s = samples[i]
    t = s.get("session_elapsed_sec", 0)
    fx = s.get("fusion", {}).get("x") or 0
    fy = s.get("fusion", {}).get("y") or 0
    sxy = s.get("fusion", {}).get("sigma_xy_m") or 0
    rtk = s.get("cross_checks", {}).get("rtk_cov_check", {})
    arrivals = rtk.get("arrivals", 0)
    ok = rtk.get("ok", 0)
    violations = rtk.get("violations", 0)
    print(f"{t:6.1f} {fx:8.4f} {fy:8.4f} {sxy*100:9.2f}cm {arrivals}/{ok}/{violations}")
