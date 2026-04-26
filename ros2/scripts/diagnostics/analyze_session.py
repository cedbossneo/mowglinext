#!/usr/bin/env python3
import json, sys

path = sys.argv[1]
with open(path) as f:
    lines = f.readlines()

meta = json.loads(lines[0])
summary = json.loads(lines[-1])
samples = [json.loads(l) for l in lines[1:-1] if l.strip()]

print(f"samples: {len(samples)}, duration: {summary['duration_sec']:.0f}s")
print(f"sample rate: {len(samples)/summary['duration_sec']:.1f} Hz")

# Position drift
xs = [s.get("fusion", {}).get("x") for s in samples if s.get("fusion", {}).get("x") is not None]
ys = [s.get("fusion", {}).get("y") for s in samples if s.get("fusion", {}).get("y") is not None]
yaws = [s.get("fusion", {}).get("yaw_deg") for s in samples if s.get("fusion", {}).get("yaw_deg") is not None]
print(f"\nfusion.x: min={min(xs):.4f} max={max(xs):.4f} range={max(xs)-min(xs):.4f} m")
print(f"fusion.y: min={min(ys):.4f} max={max(ys):.4f} range={max(ys)-min(ys):.4f} m")
print(f"fusion.yaw: min={min(yaws):.2f} deg, max={max(yaws):.2f} deg, range={max(yaws)-min(yaws):.2f} deg")

# Covariance trend
sxy = [s.get("fusion", {}).get("sigma_xy_m") for s in samples if s.get("fusion", {}).get("sigma_xy_m") is not None]
print(f"\nfusion.sigma_xy: first={sxy[0]*100:.2f}cm mid={sxy[len(sxy)//2]*100:.2f}cm last={sxy[-1]*100:.2f}cm peak={max(sxy)*100:.2f}cm")
below_3cm = sum(1 for x in sxy if x < 0.03)
below_10cm = sum(1 for x in sxy if x < 0.10)
print(f"sigma_xy < 3cm: {100*below_3cm/len(sxy):.1f}% | < 10cm: {100*below_10cm/len(sxy):.1f}%")

# GPS sigma
gsxy_mm = [s.get("gps", {}).get("sigma_xy_mm") for s in samples if s.get("gps", {}).get("sigma_xy_mm") is not None]
if gsxy_mm:
    print(f"\ngps.sigma_xy_mm: mean={sum(gsxy_mm)/len(gsxy_mm):.1f}mm max={max(gsxy_mm):.1f}mm ({len(gsxy_mm)} samples)")

# Wheel / gyro integrated yaw
w_yaws = [s.get("wheel", {}).get("yaw_integrated_deg") for s in samples if s.get("wheel", {}).get("yaw_integrated_deg") is not None]
g_yaws = [s.get("imu", {}).get("gyro_yaw_integrated_deg") for s in samples if s.get("imu", {}).get("gyro_yaw_integrated_deg") is not None]
if w_yaws:
    print(f"\nwheel.yaw_integrated: first={w_yaws[0]:.2f} deg, last={w_yaws[-1]:.2f} deg, net drift={w_yaws[-1]-w_yaws[0]:.2f} deg")
if g_yaws:
    print(f"gyro.yaw_integrated: first={g_yaws[0]:.2f} deg, last={g_yaws[-1]:.2f} deg, net drift={g_yaws[-1]-g_yaws[0]:.2f} deg")

# GPS abs pose check
gps_xs = [s.get("gps_absolute_pose", {}).get("x") for s in samples if s.get("gps_absolute_pose", {}).get("x") is not None]
print(f"\ngps_absolute_pose samples received: {len(gps_xs)}/{len(samples)}")

# RTK cov check progression
rtk_finals = [s.get("cross_checks", {}).get("rtk_cov_check", {}) for s in samples if s.get("cross_checks", {}).get("rtk_cov_check")]
if rtk_finals:
    final = rtk_finals[-1]
    print(f"\nRTK cov check final: arrivals={final.get('arrivals')}, ok={final.get('ok')}, violations={final.get('violations')}")
    # when did arrivals saturate?
    print(f"Last arrival age: {final.get('last_rtk_fixed_age_sec')}")

# gyro samples: check magnitude of gyro_z
gzs = [s.get("imu", {}).get("gyro", [None, None, None])[2] for s in samples]
gzs = [g for g in gzs if g is not None]
if gzs:
    mean_gz = sum(gzs) / len(gzs)
    print(f"\ngyro_z: mean={mean_gz:+.6f} rad/s ({mean_gz*180/3.14159:+.3f} deg/s), samples={len(gzs)}")
    # integrated over 300s, expected drift = mean_gz * 300 * 180/pi
    print(f"  integrated over {summary['duration_sec']:.0f}s @ mean rate: {mean_gz*summary['duration_sec']*180/3.14159:+.1f} deg")

# wheel vx check
wvs = [s.get("wheel", {}).get("vx") for s in samples if s.get("wheel", {}).get("vx") is not None]
if wvs:
    print(f"\nwheel.vx: mean={sum(wvs)/len(wvs):+.5f} m/s, max_abs={max(abs(v) for v in wvs):.5f} m/s")
