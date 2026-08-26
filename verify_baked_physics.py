"""
Verify that an exported FBX still carries the physics the converter computed.

Baking motion into keyframes is lossy: linear keys have zero acceleration inside
an interval, so too low a frame rate silently flattens G-force peaks. This script
closes the loop by reading the exported animation back, differentiating it the
same way CoasterAnalyzer does, and comparing against the analytic timeline.

Run via Blender:
blender -b -P verify_baked_physics.py -- --fbx ".../CoasterCartAnimated.fbx"
    --bundle ".../coaster_ue5_bundle.json" --fps 120
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import bpy
from mathutils import Vector

G0 = 9.80665


def savitzky_golay_derivatives(values, dt, half_window):
    """Least-squares quadratic fit per point. Returns (first, second) derivatives.

    This mirrors CoasterAnalyzer's differentiator rather than using plain finite
    differences, so a disagreement points at the bake and not at the method.
    """
    n = len(values)
    first = [0.0] * n
    second = [0.0] * n

    for i in range(n):
        lo = max(0, i - half_window)
        hi = min(n - 1, i + half_window)
        if hi - lo + 1 < 3:
            continue

        s0 = s1 = s2 = s3 = s4 = 0.0
        b0 = b1 = b2 = 0.0
        for j in range(lo, hi + 1):
            x = (j - i) * dt
            y = values[j]
            x2 = x * x
            s0 += 1.0
            s1 += x
            s2 += x2
            s3 += x2 * x
            s4 += x2 * x2
            b0 += y
            b1 += x * y
            b2 += x2 * y

        # Solve the normal equations for y = c0 + c1*x + c2*x^2 by Gauss-Jordan.
        a = [[s0, s1, s2, b0], [s1, s2, s3, b1], [s2, s3, s4, b2]]
        singular = False
        for col in range(3):
            piv = max(range(col, 3), key=lambda r: abs(a[r][col]))
            if abs(a[piv][col]) < 1e-18:
                singular = True
                break
            a[col], a[piv] = a[piv], a[col]
            for r in range(3):
                if r == col:
                    continue
                f = a[r][col] / a[col][col]
                for c in range(col, 4):
                    a[r][c] -= f * a[col][c]
        if singular:
            continue

        first[i] = a[1][3] / a[1][1]
        second[i] = 2.0 * (a[2][3] / a[2][2])

    return first, second


def iter_action_fcurves(action):
    """Yield an action's fcurves across Blender's old and slotted action APIs."""
    fcurves = getattr(action, "fcurves", None)
    if fcurves is not None:
        for fcurve in fcurves:
            yield fcurve
        return

    # Blender 4.4+ moved fcurves into layers -> strips -> channelbags.
    for layer in getattr(action, "layers", None) or ():
        for strip in getattr(layer, "strips", None) or ():
            for bag in getattr(strip, "channelbags", None) or ():
                for fcurve in getattr(bag, "fcurves", None) or ():
                    yield fcurve


def count_action_keys(obj):
    anim = obj.animation_data
    if anim is None:
        return 0

    actions = []
    if getattr(anim, "action", None) is not None:
        actions.append(anim.action)
    for slot in getattr(anim, "action_slots", None) or ():
        act = getattr(slot, "action", None)
        if act is not None and act not in actions:
            actions.append(act)

    return sum(
        len(fcurve.keyframe_points) for act in actions for fcurve in iter_action_fcurves(act)
    )


def action_frame_range(obj):
    """True keyframe extent of an object's animation.

    The FBX importer does not reliably update scene.frame_start/frame_end, which
    left an earlier version of this script measuring Blender's default 1..250
    window and reporting nonsense. The keys themselves are authoritative.
    """
    anim = obj.animation_data
    if anim is None:
        return None

    actions = []
    if getattr(anim, "action", None) is not None:
        actions.append(anim.action)
    for slot in getattr(anim, "action_slots", None) or ():
        act = getattr(slot, "action", None)
        if act is not None and act not in actions:
            actions.append(act)

    lo = hi = None
    for act in actions:
        for fcurve in iter_action_fcurves(act):
            for key in fcurve.keyframe_points:
                x = float(key.co[0])
                lo = x if lo is None else min(lo, x)
                hi = x if hi is None else max(hi, x)

    if lo is None:
        return None
    return int(math.floor(lo)), int(math.ceil(hi))


def find_animated_object(probe_frames=24):
    """Pick the object that actually moves.

    Chosen over counting keyframes because the action API has changed shape
    across Blender versions, whereas "which object changes position" does not.
    """
    scene = bpy.context.scene
    f0, f1 = scene.frame_start, scene.frame_end
    frames = sorted({
        int(round(f0 + (f1 - f0) * i / max(probe_frames - 1, 1)))
        for i in range(probe_frames)
    })

    # Only objects that own keyframes are candidates. A cart model's child
    # meshes travel exactly as far as the animated root that carries them, so
    # ranking purely by travel can select a child that has no curves of its own.
    candidates = [obj for obj in scene.objects if action_frame_range(obj) is not None]
    if not candidates:
        candidates = [obj for obj in scene.objects if obj.type in {"MESH", "EMPTY"}]

    tracks = {obj: [] for obj in candidates}
    if not tracks:
        return None, 0

    for frame in frames:
        scene.frame_set(frame)
        for obj in tracks:
            tracks[obj].append(obj.matrix_world.translation.copy())

    best = None
    best_travel = -1.0
    for obj, positions in tracks.items():
        travel = sum(
            (positions[i] - positions[i - 1]).length for i in range(1, len(positions))
        )
        if travel > best_travel:
            best_travel = travel
            best = obj

    if best is None or best_travel <= 1e-9:
        return None, 0
    return best, count_action_keys(best)


def percentile(sorted_vals, q):
    if not sorted_vals:
        return 0.0
    idx = min(len(sorted_vals) - 1, max(0, int(round(q * (len(sorted_vals) - 1)))))
    return sorted_vals[idx]


def build_sampler(samples):
    times = [float(s["time_s"]) for s in samples]

    def is_suspect(t):
        """True if t falls in a region the converter flagged as defective.

        Those samples sit on cusps and bridged gaps where curvature genuinely
        diverges, so comparing against them measures the source data's defects
        rather than the fidelity of the bake.
        """
        if t <= times[0]:
            return bool(samples[0].get("suspect"))
        if t >= times[-1]:
            return bool(samples[-1].get("suspect"))
        lo, hi = 0, len(times) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if times[mid] <= t:
                lo = mid
            else:
                hi = mid
        return bool(samples[lo].get("suspect") or samples[hi].get("suspect"))

    def sample_at(t, field):
        if t <= times[0]:
            return float(samples[0][field])
        if t >= times[-1]:
            return float(samples[-1][field])
        lo, hi = 0, len(times) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if times[mid] <= t:
                lo = mid
            else:
                hi = mid
        span = times[hi] - times[lo]
        a = 0.0 if span <= 0 else (t - times[lo]) / span
        return float(samples[lo][field]) * (1.0 - a) + float(samples[hi][field]) * a

    return sample_at, is_suspect


def detect_unit_scale(pos, samples):
    """Return metres-per-baked-unit, inferred from overall size.

    A cm/m mix-up multiplies every acceleration by 100, so it has to be caught
    here rather than showing up as a mysteriously large G reading later.
    """
    baked_span = max(max(p[k] for p in pos) - min(p[k] for p in pos) for k in range(3))
    analytic_span = max(
        max(s["pos_m"][k] for s in samples) - min(s["pos_m"][k] for s in samples)
        for k in range(3)
    )
    ratio = baked_span / analytic_span if analytic_span > 1e-9 else 0.0

    print("")
    print(f"baked/analytic size ratio: {ratio:.5f}")
    if abs(ratio - 1.0) < 0.01:
        print("  -> FBX round-trip preserved metres 1:1")
        return 1.0, True
    if abs(ratio - 100.0) < 1.0:
        print("  -> FBX round-trip is in centimetres")
        return 1.0 / 100.0, True
    print(f"  -> WARNING: unexpected scale factor {ratio:.5f}")
    return (1.0 / ratio if ratio > 1e-9 else 1.0), False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--fbx", required=True)
    parser.add_argument("--bundle", required=True)
    parser.add_argument("--fps", type=int, default=120)
    parser.add_argument("--window-seconds", type=float, default=0.15)
    parser.add_argument("--speed-tolerance-pct", type=float, default=2.0)
    # Acceleration is compared between two different estimators on purpose: the
    # converter fits a circumcircle to the geometry, while this script runs the
    # Savitzky-Golay differentiator CoasterAnalyzer will actually use in Unreal.
    # The question being answered is "what will the analyzer read", so the
    # estimators are not unified, and a p95 gap of roughly 20% where curvature
    # changes quickly is inherent rather than a defect in the bake.
    parser.add_argument("--accel-tolerance-pct", type=float, default=25.0)
    # Peaks are the reading that matters, so they are bounded on both sides:
    # too low means the bake flattened them, too high means it invented them.
    parser.add_argument("--peak-retention-min-pct", type=float, default=85.0)
    parser.add_argument("--peak-retention-max-pct", type=float, default=120.0)
    argv = sys.argv
    argv = argv[argv.index("--") + 1:] if "--" in argv else []
    args = parser.parse_args(argv)

    bundle = json.loads(Path(args.bundle).read_text(encoding="utf-8"))
    samples = bundle["samples"]

    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.fbx(filepath=args.fbx)

    obj, key_count = find_animated_object()
    if obj is None:
        raise RuntimeError("No animated object found in the FBX")

    scene = bpy.context.scene

    key_range = action_frame_range(obj)
    if key_range is None:
        raise RuntimeError(f"{obj.name} moves but exposes no keyframes")
    f0, f1 = key_range
    n_frames = f1 - f0 + 1

    # The FBX declares its own playback rate. That rate, not the rate the caller
    # expected, determines how much real time each key interval represents.
    fbx_fps = float(scene.render.fps) / float(scene.render.fps_base or 1.0)
    dt = 1.0 / fbx_fps

    analytic_duration = float(samples[-1]["time_s"])
    baked_duration = (n_frames - 1) * dt

    print(f"animated object  : {obj.name} ({key_count} keys)")
    print(f"frame range      : {f0}..{f1} ({n_frames} frames)")
    print(f"fbx playback rate: {fbx_fps:g}fps (expected {args.fps}fps)")
    print(f"baked duration   : {baked_duration:.2f}s")
    print(f"analytic duration: {analytic_duration:.2f}s")

    timing_ok = True
    if abs(fbx_fps - args.fps) > 0.51:
        print(
            f"  -> WARNING: FBX declares {fbx_fps:g}fps but keys were authored "
            f"for {args.fps}fps. Every acceleration is scaled by "
            f"{(fbx_fps / args.fps) ** 2:.4f}."
        )
        timing_ok = False
    if analytic_duration > 1e-6:
        drift = abs(baked_duration - analytic_duration) / analytic_duration
        if drift > 0.01:
            print(
                f"  -> WARNING: baked duration differs from analytic by "
                f"{drift * 100.0:.1f}%."
            )
            timing_ok = False

    # Sample the evaluated world transform, which is what a game engine sees.
    depsgraph = bpy.context.evaluated_depsgraph_get()
    pos = []
    for f in range(f0, f1 + 1):
        scene.frame_set(f)
        depsgraph.update()
        pos.append(obj.evaluated_get(depsgraph).matrix_world.translation.copy())

    to_m, scale_ok = detect_unit_scale(pos, samples)

    half = max(1, int(round(args.window_seconds / dt / 2.0)))
    vel, acc = [], []
    for k in range(3):
        d1, d2 = savitzky_golay_derivatives([p[k] * to_m for p in pos], dt, half)
        vel.append(d1)
        acc.append(d2)

    sample_at, is_suspect = build_sampler(samples)
    speed_err, accel_err = [], []
    peak_baked_g = 0.0
    peak_analytic_g = 0.0
    skipped = 0

    # Trim one window at each end: the quadratic fit is one-sided there.
    for i in range(half, n_frames - half):
        t = i * dt
        if is_suspect(t):
            skipped += 1
            continue
        v_vec = Vector((vel[0][i], vel[1][i], vel[2][i]))
        v_ref = sample_at(t, "speed_mps")
        if v_ref > 1.0:
            speed_err.append(abs(v_vec.length - v_ref) / v_ref * 100.0)

        a_vec = Vector((acc[0][i], acc[1][i], acc[2][i]))
        if v_vec.length > 1e-6:
            tangent = v_vec.normalized()
            a_normal = (a_vec - tangent * a_vec.dot(tangent)).length
        else:
            a_normal = a_vec.length

        a_ref = sample_at(t, "normal_acc_mps2")
        if a_ref > 2.0:
            accel_err.append(abs(a_normal - a_ref) / a_ref * 100.0)

        peak_baked_g = max(peak_baked_g, a_normal / G0)
        peak_analytic_g = max(peak_analytic_g, a_ref / G0)

    speed_err.sort()
    accel_err.sort()
    retention = 100.0 * peak_baked_g / max(peak_analytic_g, 1e-9)

    print("")
    if skipped:
        print(
            f"skipped {skipped} frames in regions the converter flagged as "
            "defective source geometry"
        )
    print(f"--- speed recovered from baked keys ({len(speed_err)} frames) ---")
    print(
        f"  median {percentile(speed_err, 0.5):6.3f}%   "
        f"p95 {percentile(speed_err, 0.95):6.3f}%   "
        f"max {(speed_err[-1] if speed_err else 0.0):6.3f}%"
    )
    print("")
    print(f"--- normal acceleration recovered ({len(accel_err)} frames) ---")
    print(
        f"  median {percentile(accel_err, 0.5):6.2f}%   "
        f"p95 {percentile(accel_err, 0.95):6.2f}%   "
        f"max {(accel_err[-1] if accel_err else 0.0):6.2f}%"
    )
    print(
        f"  peak normal G: baked {peak_baked_g:.3f}  "
        f"analytic {peak_analytic_g:.3f}  retention {retention:.1f}%"
    )

    failures = []
    if not timing_ok:
        failures.append(
            "FBX playback timing does not match the analytic timeline "
            "(see warnings above)"
        )
    if not scale_ok:
        failures.append("FBX round-trip scale is not a recognised unit ratio")
    if percentile(speed_err, 0.95) > args.speed_tolerance_pct:
        failures.append(
            f"speed p95 error {percentile(speed_err, 0.95):.2f}% "
            f"exceeds {args.speed_tolerance_pct}%"
        )
    if percentile(accel_err, 0.95) > args.accel_tolerance_pct:
        failures.append(
            f"acceleration p95 error {percentile(accel_err, 0.95):.2f}% "
            f"exceeds {args.accel_tolerance_pct}%"
        )
    if retention < args.peak_retention_min_pct:
        failures.append(
            f"peak G retention {retention:.1f}% below "
            f"{args.peak_retention_min_pct}% - the bake flattened the peaks, "
            "raise --fps in the exporter"
        )
    elif retention > args.peak_retention_max_pct:
        failures.append(
            f"peak G retention {retention:.1f}% above "
            f"{args.peak_retention_max_pct}% - the bake is producing forces the "
            "analytic path does not contain, check --resample-spacing-m"
        )

    print("")
    if failures:
        for f in failures:
            print(f"FAIL: {f}")
        print("RESULT: FAIL")
        sys.exit(2)

    print("RESULT: PASS - baked animation reproduces the analytic physics")


if __name__ == "__main__":
    main()
