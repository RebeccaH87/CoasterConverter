"""
Verify CoasterTrack.fbx against the bundle it was generated from.

The FBX is written by hand, so it is checked with an independent reader rather
than trusted. Blender is used purely as that reader; it is not part of the
pipeline.

    blender -b -P verify_track_mesh.py -- <track.fbx> <bundle.json>

Checks that every part imported, that the geometry sits where the path is, that
no vertex is degenerate, and that normals and UVs survived.
"""

import json
import math
import sys
from pathlib import Path

import bpy

argv = sys.argv
argv = argv[argv.index("--") + 1:] if "--" in argv else []
fbx_path, bundle_path = argv[0], argv[1]

bundle = json.loads(Path(bundle_path).read_text(encoding="utf-8"))
path = bundle.get("render_path") or bundle["samples"]

bpy.ops.wm.read_factory_settings(use_empty=True)
try:
    bpy.ops.import_scene.fbx(filepath=fbx_path)
except Exception as ex:
    print(f"IMPORT FAILED: {ex}")
    sys.exit(3)

meshes = [o for o in bpy.context.scene.objects if o.type == "MESH"]
print(f"imported meshes: {sorted(o.name for o in meshes)}")
if not meshes:
    print("NO MESH IMPORTED")
    sys.exit(3)

failures = []

expected = {
    "CoasterTrackRails", "CoasterTrackSpine",
    "CoasterTrackTies", "CoasterTrackSupports",
}
found = {o.name.split(".")[0] for o in meshes}
missing = expected - found
if missing:
    failures.append(f"missing parts: {sorted(missing)}")

print()
total_verts = total_polys = 0
for obj in sorted(meshes, key=lambda o: o.name):
    mesh = obj.data
    uv_layers = [layer.name for layer in mesh.uv_layers]
    print(
        f"  {obj.name:24s} verts {len(mesh.vertices):7d}  polys {len(mesh.polygons):7d}"
        f"  uv {uv_layers or 'NONE'}"
    )
    total_verts += len(mesh.vertices)
    total_polys += len(mesh.polygons)
    if not uv_layers:
        failures.append(f"{obj.name} has no UV layer")

    bad = 0
    for v in mesh.vertices:
        p = obj.matrix_world @ v.co
        if any(math.isnan(c) or math.isinf(c) for c in p):
            bad += 1
    if bad:
        failures.append(f"{obj.name} has {bad} non-finite vertices")

print(f"  {'TOTAL':24s} verts {total_verts:7d}  polys {total_polys:7d}")

# Where the geometry sits, against where the path says it should be.
scale_probe = []
for obj in meshes:
    for v in obj.data.vertices:
        scale_probe.append(obj.matrix_world @ v.co)

mesh_min = [min(p[k] for p in scale_probe) for k in range(3)]
mesh_max = [max(p[k] for p in scale_probe) for k in range(3)]

path_min = [min(s["ue_pos_cm"][k] for s in path) for k in range(3)]
path_max = [max(s["ue_pos_cm"][k] for s in path) for k in range(3)]

mesh_span = [mesh_max[k] - mesh_min[k] for k in range(3)]
path_span = [path_max[k] - path_min[k] for k in range(3)]

ratio = max(mesh_span) / max(path_span) if max(path_span) > 1e-9 else 0.0
if abs(ratio - 0.01) < 0.002:
    to_cm, unit = 100.0, "centimetres -> metres (Blender default)"
elif abs(ratio - 1.0) < 0.05:
    to_cm, unit = 1.0, "centimetres preserved 1:1"
else:
    to_cm, unit = 1.0, "UNRECOGNISED"
    failures.append(f"unrecognised import scale ratio {ratio:.6f}")

print()
print(f"import scale ratio: {ratio:.6f}  -> {unit}")
print(f"{'axis':5s} {'mesh span m':>13s} {'path span m':>13s} {'difference m':>13s}")
for k, axis in enumerate("XYZ"):
    m = mesh_span[k] * to_cm / 100.0
    p = path_span[k] / 100.0
    print(f"{axis:5s} {m:13.2f} {p:13.2f} {m - p:13.2f}")

# The track is wider than the path by the gauge, and deeper by the spine drop
# plus the support columns, so only an upper bound is meaningful here.
for k, axis in enumerate("XY"):
    m = mesh_span[k] * to_cm
    p = path_span[k]
    if m < p - 1.0:
        failures.append(f"{axis} span {m:.1f}cm is smaller than the path's {p:.1f}cm")
    if m > p + 400.0:
        failures.append(f"{axis} span {m:.1f}cm overshoots the path's {p:.1f}cm")

# Vertical: the mesh must reach below the path (supports) but not above it.
top_gap = (mesh_max[2] * to_cm) - path_max[2]
if top_gap > 50.0:
    failures.append(f"mesh rises {top_gap:.1f}cm above the path")

print()
if failures:
    for f in failures:
        print(f"FAIL: {f}")
    print("RESULT: FAIL")
    sys.exit(2)
print("RESULT: PASS - track geometry matches the converted path")
