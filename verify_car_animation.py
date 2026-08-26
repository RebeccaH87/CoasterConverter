"""
Verify CoasterCarAnimated.fbx against the bundle it was generated from.

The FBX is written by hand, so it is checked with an independent reader rather
than trusted. Blender is used purely as that reader; it is not part of the
pipeline.

    blender -b -P verify_car_animation.py -- <fbx> <bundle.json> <fps>

Checks position against the timeline, and that the car's local +X still points
along travel and its local +Z along the banked up vector - which is what
catches a wrong Euler order or a mirrored axis.
"""

import json
import math
import sys
from pathlib import Path

import bpy
from mathutils import Vector

argv = sys.argv
argv = argv[argv.index("--") + 1:] if "--" in argv else []
fbx_path, bundle_path, fps = argv[0], argv[1], float(argv[2])

bundle = json.loads(Path(bundle_path).read_text(encoding="utf-8"))
samples = bundle["samples"]
times = [float(s["time_s"]) for s in samples]


def sample_at(t):
    """Linearly interpolate the bundle, so nearest-sample spacing is not
    mistaken for an error in the exported animation."""
    if t <= times[0]:
        return samples[0]
    if t >= times[-1]:
        return samples[-1]
    lo, hi = 0, len(times) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if times[mid] <= t:
            lo = mid
        else:
            hi = mid
    span = times[hi] - times[lo]
    a = 0.0 if span <= 1e-12 else (t - times[lo]) / span
    out = {}
    for key in ("ue_pos_cm", "ue_tan", "ue_up"):
        p, q = samples[lo][key], samples[hi][key]
        out[key] = [p[i] + (q[i] - p[i]) * a for i in range(3)]
    return out


bpy.ops.wm.read_factory_settings(use_empty=True)
try:
    bpy.ops.import_scene.fbx(filepath=fbx_path)
except Exception as ex:
    print(f"IMPORT FAILED: {ex}")
    sys.exit(3)

objs = [o for o in bpy.context.scene.objects]
print(f"imported objects: {[o.name for o in objs]}")

target = None
for o in objs:
    if "CoasterCar" in o.name:
        target = o
        break
if target is None and objs:
    target = objs[0]
if target is None:
    print("NO OBJECT IMPORTED")
    sys.exit(3)

anim = target.animation_data
print(f"target: {target.name}  has animation_data: {anim is not None}")

scene = bpy.context.scene


def iter_fcurves(action):
    fc = getattr(action, "fcurves", None)
    if fc is not None:
        for c in fc:
            yield c
        return
    for layer in getattr(action, "layers", None) or ():
        for strip in getattr(layer, "strips", None) or ():
            for bag in getattr(strip, "channelbags", None) or ():
                for c in getattr(bag, "fcurves", None) or ():
                    yield c


# The FBX importer does not reliably set the scene range, so the keys decide.
lo = hi = None
acts = []
if anim is not None:
    if getattr(anim, "action", None) is not None:
        acts.append(anim.action)
    for slot in getattr(anim, "action_slots", None) or ():
        a = getattr(slot, "action", None)
        if a is not None and a not in acts:
            acts.append(a)
nkeys = 0
for a in acts:
    for c in iter_fcurves(a):
        for k in c.keyframe_points:
            nkeys += 1
            x = float(k.co[0])
            lo = x if lo is None else min(lo, x)
            hi = x if hi is None else max(hi, x)
if lo is None:
    print("NO KEYFRAMES FOUND")
    sys.exit(3)

scene.frame_start, scene.frame_end = int(round(lo)), int(round(hi))
print(f"scene fps: {scene.render.fps}  key range: {scene.frame_start}..{scene.frame_end}  keys: {nkeys}")

# Detect the unit scale the importer applied by comparing overall extent.
probe = []
for f in range(scene.frame_start, scene.frame_end + 1, max(1, (scene.frame_end - scene.frame_start) // 200)):
    scene.frame_set(f)
    probe.append(target.matrix_world.translation.copy())

if len(probe) < 2:
    print("NOT ENOUGH FRAMES")
    sys.exit(3)

baked_span = max(max(p[k] for p in probe) - min(p[k] for p in probe) for k in range(3))
ref_span = max(
    max(s["ue_pos_cm"][k] for s in samples) - min(s["ue_pos_cm"][k] for s in samples)
    for k in range(3)
)
ratio = baked_span / ref_span if ref_span > 1e-9 else 0.0
# Snap to the exact unit factor. A coarse frame probe misses the path extremes,
# so the measured ratio is slightly low; using it directly would smear that
# error across every position comparison.
if abs(ratio - 0.01) < 0.002:
    to_cm, unit = 100.0, "centimetres -> metres (Blender default)"
elif abs(ratio - 1.0) < 0.05:
    to_cm, unit = 1.0, "centimetres preserved 1:1"
else:
    to_cm, unit = (1.0 / ratio if ratio > 1e-9 else 1.0), "UNRECOGNISED"
print(f"extent ratio (imported / bundle-cm): {ratio:.6f}  -> {unit}")

pos_err, tan_dot, up_dot = [], [], []
n = 0
step = max(1, (scene.frame_end - scene.frame_start) // 400)
for f in range(scene.frame_start, scene.frame_end + 1, step):
    scene.frame_set(f)
    t = (f - scene.frame_start) / fps
    ref = sample_at(t)

    mw = target.matrix_world
    pos_cm = mw.translation * to_cm
    ref_pos = Vector(ref["ue_pos_cm"])
    pos_err.append((pos_cm - ref_pos).length)

    basis = mw.to_3x3()
    local_x = (basis @ Vector((1.0, 0.0, 0.0))).normalized()
    local_z = (basis @ Vector((0.0, 0.0, 1.0))).normalized()
    tan_dot.append(local_x.dot(Vector(ref["ue_tan"]).normalized()))
    up_dot.append(local_z.dot(Vector(ref["ue_up"]).normalized()))
    n += 1


def stats(vals):
    s = sorted(vals)
    return s[0], s[len(s) // 2], s[-1]


print(f"\ncompared {n} frames")
lo, mid, hi = stats(pos_err)
print(f"position error cm : min {lo:.3f}  median {mid:.3f}  max {hi:.3f}")
lo, mid, hi = stats(tan_dot)
print(f"forward axis dot  : min {lo:+.5f}  median {mid:+.5f}  max {hi:+.5f}  (want +1)")
lo, mid, hi = stats(up_dot)
print(f"up axis dot       : min {lo:+.5f}  median {mid:+.5f}  max {hi:+.5f}  (want +1)")

bad_tan = sum(1 for d in tan_dot if d < 0.99)
bad_up = sum(1 for d in up_dot if d < 0.99)
print(f"frames with forward dot < 0.99: {bad_tan} / {n}")
print(f"frames with up dot < 0.99     : {bad_up} / {n}")

ok = (
    stats(pos_err)[2] < 1.0
    and bad_tan == 0
    and bad_up == 0
    and abs(ratio - 1.0) < 0.02 or abs(ratio - 0.01) < 0.0005
)
print("\nRESULT:", "PASS" if (stats(pos_err)[2] < 1.0 and bad_tan == 0 and bad_up == 0) else "FAIL")
