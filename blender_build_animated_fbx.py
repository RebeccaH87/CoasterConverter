"""
Build separate UE-friendly FBX outputs for track and animated cart.

Run via Blender:
blender -b -P blender_build_animated_fbx.py -- --mesh ".../CoasterModel.3ds" --bundle ".../coaster_ue5_bundle.json" --track-out ".../CoasterTrack.fbx" --cart-out ".../CoasterCartAnimated.fbx" --fps 30
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import bpy
from mathutils import Matrix, Vector


def load_bundle(bundle_path: Path):
    data = json.loads(bundle_path.read_text(encoding="utf-8"))
    samples = data.get("samples", [])
    if not samples:
        raise RuntimeError("Bundle contains no samples")

    # "samples" carries the un-edited analytic path that the physics timeline
    # was derived from. "render_path" is the spike-filtered copy and is the only
    # one safe to build display geometry from. Older v1 bundles have no
    # render_path, in which case samples are all we have.
    render_path = data.get("render_path") or samples
    if len(render_path) != len(samples):
        print(
            f"WARNING: render_path has {len(render_path)} points but samples has "
            f"{len(samples)}. Using render_path for geometry regardless."
        )
    return data, samples, render_path


def make_rotation_from_tangent_up(tangent, up):
    fwd = Vector(tangent).normalized()
    up_v = Vector(up).normalized()
    right = fwd.cross(up_v)
    if right.length < 1e-8:
        right = Vector((1.0, 0.0, 0.0))
    right.normalize()
    up_v = right.cross(fwd)
    up_v.normalize()

    # Local axes: X=right, Y=forward, Z=up
    rot_m = Matrix((right, fwd, up_v)).transposed()
    return rot_m.to_euler("XYZ")


def apply_linear_interpolation(obj):
    ad = obj.animation_data
    if not ad:
        return

    actions = []
    if getattr(ad, "action", None) is not None:
        actions.append(ad.action)

    slots = getattr(ad, "action_slots", None)
    if slots:
        for slot in slots:
            act = getattr(slot, "action", None)
            if act is not None:
                actions.append(act)

    for action in actions:
        fcurves = getattr(action, "fcurves", None)
        if not fcurves:
            continue
        for fcurve in fcurves:
            for point in fcurve.keyframe_points:
                point.interpolation = "LINEAR"


def v_lerp(a, b, t):
    return a + (b - a) * t


def sample_at_time(samples, t):
    def normalize_sample(s):
        return {
            "pos": Vector(s["pos_m"]),
            "tan": Vector(s["tan"]).normalized(),
            "up": Vector(s["up"]).normalized(),
        }

    if t <= samples[0]["time_s"]:
        return normalize_sample(samples[0])
    if t >= samples[-1]["time_s"]:
        return normalize_sample(samples[-1])

    lo = 0
    hi = len(samples) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if samples[mid]["time_s"] <= t:
            lo = mid
        else:
            hi = mid

    a = samples[lo]
    b = samples[hi]
    ta = float(a["time_s"])
    tb = float(b["time_s"])
    alpha = 0.0 if tb <= ta else (t - ta) / (tb - ta)

    pos_a = Vector(a["pos_m"])
    pos_b = Vector(b["pos_m"])
    tan_a = Vector(a["tan"])
    tan_b = Vector(b["tan"])
    up_a = Vector(a["up"])
    up_b = Vector(b["up"])

    return {
        "pos": v_lerp(pos_a, pos_b, alpha),
        "tan": v_lerp(tan_a, tan_b, alpha).normalized(),
        "up": v_lerp(up_a, up_b, alpha).normalized(),
    }


def build_rider_mesh_object(cart_scale: float):
    # Small cart-like marker mesh that UE will always import as a real animated object.
    verts = [
        (-0.25, -0.45, -0.15),
        (0.25, -0.45, -0.15),
        (0.25, 0.45, -0.15),
        (-0.25, 0.45, -0.15),
        (-0.25, -0.45, 0.15),
        (0.25, -0.45, 0.15),
        (0.25, 0.45, 0.15),
        (-0.25, 0.45, 0.15),
    ]
    faces = [
        (0, 1, 2, 3),
        (4, 5, 6, 7),
        (0, 1, 5, 4),
        (1, 2, 6, 5),
        (2, 3, 7, 6),
        (3, 0, 4, 7),
    ]

    mesh = bpy.data.meshes.new("CoasterRiderMesh")
    mesh.from_pydata(verts, [], faces)
    mesh.update()

    obj = bpy.data.objects.new("CoasterRider", mesh)
    bpy.context.collection.objects.link(obj)
    obj.rotation_mode = "XYZ"
    obj.scale = (cart_scale, cart_scale, cart_scale)
    return obj


def world_bounds_size(objects):
    min_v = Vector((1.0e18, 1.0e18, 1.0e18))
    max_v = Vector((-1.0e18, -1.0e18, -1.0e18))
    has_mesh = False

    for obj in objects:
        if obj.type != "MESH":
            continue
        has_mesh = True
        for corner in obj.bound_box:
            p = obj.matrix_world @ Vector(corner)
            min_v.x = min(min_v.x, p.x)
            min_v.y = min(min_v.y, p.y)
            min_v.z = min(min_v.z, p.z)
            max_v.x = max(max_v.x, p.x)
            max_v.y = max(max_v.y, p.y)
            max_v.z = max(max_v.z, p.z)

    if not has_mesh:
        return None
    return max_v - min_v


def try_import_cart_model(cart_model_path: Path, cart_scale: float, fit_mode: str, target_length_m: float):
    importer = getattr(bpy.ops.import_scene, "gltf", None)
    if importer is None or not cart_model_path.exists():
        return None

    pre = set(bpy.context.scene.objects)
    try:
        importer(filepath=str(cart_model_path))
    except Exception:
        return None

    imported = [o for o in bpy.context.scene.objects if o not in pre]
    if not imported:
        return None

    imported_set = set(imported)
    imported_roots = [o for o in imported if o.parent not in imported_set]
    mesh_objs = [o for o in imported if o.type == "MESH"]
    if not mesh_objs:
        return None

    visual_root = bpy.data.objects.new("CoasterCartVisual", None)
    bpy.context.collection.objects.link(visual_root)
    visual_root.rotation_mode = "XYZ"

    for obj in imported_roots:
        obj.parent = visual_root

    bounds = world_bounds_size(mesh_objs)
    max_dim = max(bounds.x, bounds.y, bounds.z) if bounds is not None else 0.0

    # "preserve" keeps the model's authored real-world size, so a 4.5m car
    # imports as 4.5m. Forcing every model to a fixed nominal length destroys
    # real-world scale, which is exactly what has to stay correct here, so
    # "normalize" is opt-in for models authored in arbitrary units.
    normalize = 1.0
    if fit_mode == "normalize":
        if max_dim > 1.0e-6:
            normalize = max(target_length_m, 1.0e-6) / max_dim
    elif fit_mode != "preserve":
        raise RuntimeError(f"Unknown cart fit mode: {fit_mode}")

    final_scale = max(cart_scale, 0.01) * normalize
    visual_root.scale = (final_scale, final_scale, final_scale)

    print(
        f"Imported cart model: {cart_model_path}"
    )
    print(
        f"  fit_mode={fit_mode} authored_max_dim={max_dim:.3f}m "
        f"normalize={normalize:.5f} cart_scale={cart_scale:.3f} "
        f"final_scale={final_scale:.5f}"
    )
    if max_dim > 1.0e-6:
        print(f"  resulting cart length: {max_dim * final_scale:.3f} m")
    if fit_mode == "preserve" and abs(cart_scale - 1.0) > 1e-6:
        print(
            f"  NOTE: cart_scale={cart_scale:.3f} != 1.0, so the cart is no "
            "longer at its authored real-world size."
        )

    return visual_root


def build_calibration_cube(edge_m: float, samples):
    """A cube of exactly known size, so the delivered scale can be asserted in UE.

    The converter can verify its own output against a reference path, but it
    cannot see what the FBX importer does to units. This cube travels through
    the same export and import as the track, under the same root transform, so
    measuring it in Unreal measures the whole chain end to end.
    """
    if edge_m <= 0.0:
        return None

    xs = [s["pos_m"][0] for s in samples]
    ys = [s["pos_m"][1] for s in samples]
    zs = [s["pos_m"][2] for s in samples]

    # Park it clear of the track so it never intersects the geometry.
    origin = Vector((min(xs) - edge_m * 3.0, min(ys), min(zs) - edge_m * 3.0))

    h = edge_m * 0.5
    verts = [
        (-h, -h, -h), (h, -h, -h), (h, h, -h), (-h, h, -h),
        (-h, -h, h), (h, -h, h), (h, h, h), (-h, h, h),
    ]
    faces = [
        (0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4),
        (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7),
    ]

    mesh = bpy.data.meshes.new("CoasterScaleRefMesh")
    mesh.from_pydata(verts, [], faces)
    mesh.update()

    # The expected size is encoded in the name so the UE-side check needs no
    # out-of-band knowledge of what it is measuring.
    name = f"CoasterScaleRef_{int(round(edge_m * 1000.0))}mm"
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    obj.location = origin
    print(f"Added scale calibration cube '{name}': {edge_m:.4f} m edge at {origin[:]}")
    return obj


def build_fallback_track_mesh(samples):
    points_raw = samples[::4] if len(samples) > 4000 else samples
    points_raw = [{"pos": Vector(s["pos_m"]), "up": Vector(s["up"]).normalized()} for s in points_raw]
    if len(points_raw) < 2:
        raise RuntimeError("Not enough sampled points to build fallback track mesh")

    # Remove near-duplicate and needle-like reversal points to avoid bevel spikes.
    seg_lengths = [(points_raw[i]["pos"] - points_raw[i - 1]["pos"]).length for i in range(1, len(points_raw))]
    avg_seg = (sum(seg_lengths) / len(seg_lengths)) if seg_lengths else 0.1
    min_step = max(avg_seg * 0.2, 0.03)

    filtered = [points_raw[0]]
    dropped_close = 0
    for p in points_raw[1:]:
        if (p["pos"] - filtered[-1]["pos"]).length < min_step:
            dropped_close += 1
            continue
        filtered.append(p)

    dropped_needle = 0
    i = 1
    while i < len(filtered) - 1:
        a = filtered[i - 1]["pos"]
        b = filtered[i]["pos"]
        c = filtered[i + 1]["pos"]
        ab = b - a
        bc = c - b
        d1 = ab.length
        d2 = bc.length

        if d1 < 1e-9 or d2 < 1e-9:
            filtered.pop(i)
            dropped_needle += 1
            continue

        dot_dir = ab.normalized().dot(bc.normalized())
        if min(d1, d2) < (min_step * 1.5) and dot_dir < -0.25:
            filtered.pop(i)
            dropped_needle += 1
            continue
        i += 1

    if len(filtered) < 2:
        filtered = points_raw

    tube_radius = 0.04
    tube_sides = 10
    verts = []
    faces = []
    prev_right = None

    for i, p in enumerate(filtered):
        pos = p["pos"]
        if i == 0:
            tan = (filtered[i + 1]["pos"] - pos).normalized()
        elif i == len(filtered) - 1:
            tan = (pos - filtered[i - 1]["pos"]).normalized()
        else:
            tan = (filtered[i + 1]["pos"] - filtered[i - 1]["pos"]).normalized()

        up = p["up"]
        right = tan.cross(up)
        if right.length < 1e-8:
            right = Vector((1.0, 0.0, 0.0))
        right.normalize()
        up = right.cross(tan)
        if up.length < 1e-8:
            up = Vector((0.0, 1.0, 0.0))
        up.normalize()

        if prev_right is not None and right.dot(prev_right) < 0.0:
            right = -right
            up = -up
        prev_right = right.copy()

        ring_start = len(verts)
        for side in range(tube_sides):
            ang = (2.0 * math.pi * side) / tube_sides
            radial = right * math.cos(ang) + up * math.sin(ang)
            v = pos + radial * tube_radius
            verts.append((v.x, v.y, v.z))

        if i > 0:
            prev_ring = ring_start - tube_sides
            for side in range(tube_sides):
                a = prev_ring + side
                b = prev_ring + ((side + 1) % tube_sides)
                c = ring_start + ((side + 1) % tube_sides)
                d = ring_start + side
                faces.append((a, b, c, d))

    mesh = bpy.data.meshes.new("CoasterTrackFallbackMesh")
    mesh.from_pydata(verts, [], faces)
    mesh.update()

    obj = bpy.data.objects.new("CoasterTrackFallback", mesh)
    bpy.context.collection.objects.link(obj)

    print(
        f"Fallback mesh cleanup: raw={len(points_raw)} kept={len(filtered)} "
        f"drop_close={dropped_close} drop_needle={dropped_needle} "
        f"verts={len(verts)} faces={len(faces)}"
    )
    return obj


def collect_mesh_objects():
    return [o for o in bpy.context.scene.objects if o.type in {"MESH", "CURVE", "EMPTY"}]


def create_root_node(scale_multiplier: float, root_rot_x_deg: float):
    root = bpy.data.objects.new("CoasterRoot", None)
    bpy.context.collection.objects.link(root)
    root.empty_display_type = "PLAIN_AXES"
    root.rotation_mode = "XYZ"
    root.rotation_euler = (math.radians(root_rot_x_deg), 0.0, 0.0)
    root.scale = (scale_multiplier, scale_multiplier, scale_multiplier)
    return root


def collect_hierarchy(objects):
    result = set()
    stack = list(objects)
    while stack:
        obj = stack.pop()
        if obj in result:
            continue
        result.add(obj)
        stack.extend(list(obj.children))
    return list(result)


def export_fbx_selection(path: Path, selected_objects):
    all_objs = list(bpy.context.scene.objects)
    for o in all_objs:
        o.select_set(False)

    expanded = collect_hierarchy(selected_objects)
    if not expanded:
        raise RuntimeError(f"No objects selected for export: {path}")

    for o in expanded:
        o.select_set(True)
    bpy.context.view_layer.objects.active = expanded[0]

    path.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.export_scene.fbx(
        filepath=str(path),
        use_selection=True,
        apply_unit_scale=True,
        bake_space_transform=False,
        add_leaf_bones=False,
        bake_anim=True,
        bake_anim_use_all_actions=True,
        bake_anim_force_startend_keying=True,
        bake_anim_step=1.0,
        axis_forward="-Z",
        axis_up="Y",
    )


def parent_objects_to_root(objects, root):
    for obj in objects:
        if obj == root:
            continue
        obj.parent = root


def try_import_3ds(mesh_path: Path) -> bool:
    importer = getattr(bpy.ops.import_scene, "autodesk_3ds", None)
    if importer is None:
        return False

    try:
        importer(filepath=str(mesh_path))
        return True
    except Exception:
        return False


def build_animation(
    samples,
    fps,
    speed_multiplier,
    cart_scale,
    cart_model_path: Path | None,
    cart_fit_mode: str = "preserve",
    cart_target_length_m: float = 0.9,
):
    cart = None
    if cart_model_path is not None:
        cart = try_import_cart_model(
            cart_model_path, cart_scale, cart_fit_mode, cart_target_length_m
        )
    if cart is None:
        cart = build_rider_mesh_object(cart_scale)

    start_frame = 1
    max_time = float(samples[-1]["time_s"])
    speed = max(float(speed_multiplier), 1e-3)
    end_frame = max(int(round((max_time / speed) * fps)) + start_frame, start_frame + 1)

    # Write one key per frame for reliable import into DCC/game engines.
    for frame in range(start_frame, end_frame + 1):
        t = ((frame - start_frame) / float(fps)) * speed
        sample = sample_at_time(samples, t)
        pos = sample["pos"]
        rot = make_rotation_from_tangent_up(sample["tan"], sample["up"])
        cart.location = pos
        cart.rotation_euler = rot
        cart.keyframe_insert(data_path="location", frame=frame)
        cart.keyframe_insert(data_path="rotation_euler", frame=frame)

    scene = bpy.context.scene
    # Blender's FBX exporter converts frame numbers to seconds using the scene
    # render fps, NOT the spacing the keys were authored at. Leaving this at the
    # factory default of 24 makes the FBX declare a ride that lasts fps/24 times
    # too long, which scales every acceleration by (24/fps)^2. It must match the
    # rate the keys were written at.
    scene.render.fps = int(round(fps))
    scene.render.fps_base = 1.0
    scene.frame_start = start_frame
    scene.frame_end = end_frame
    print(
        f"Scene timing: {scene.render.fps}fps, frames {start_frame}..{end_frame} "
        f"= {(end_frame - start_frame) / float(scene.render.fps):.2f}s"
    )

    apply_linear_interpolation(cart)
    return cart


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mesh", required=True)
    parser.add_argument("--bundle", required=True)
    parser.add_argument("--out")
    parser.add_argument("--track-out")
    parser.add_argument("--cart-out")
    parser.add_argument("--cart-model")
    # 120fps, not 30. Position keys are linear, so acceleration inside a key
    # interval is zero and the whole second derivative lives in the joints
    # between keys. The analyzer fits a quadratic over a 0.15s window; at 30fps
    # that window holds ~4 keys, which cannot resolve a G peak. At 120fps it
    # holds ~18. Linear interpolation is kept deliberately - at this key density
    # it is accurate, and Bezier would overshoot on tight curvature.
    parser.add_argument("--fps", type=int, default=120)
    parser.add_argument("--speed-multiplier", type=float, default=1.0)
    parser.add_argument("--scale-multiplier", type=float, default=1.0)
    parser.add_argument("--cart-scale", type=float, default=1.0)
    parser.add_argument(
        "--cart-fit-mode",
        choices=["preserve", "normalize"],
        default="preserve",
        help="preserve: keep the model's authored real-world size (default). "
             "normalize: rescale its longest axis to --cart-target-length.",
    )
    parser.add_argument("--cart-target-length", type=float, default=0.9)
    parser.add_argument(
        "--calibration-cube-m",
        type=float,
        default=1.0,
        help="Edge length in metres of a known-size cube added to the track FBX "
             "so import scale can be asserted in Unreal. 0 disables it.",
    )
    parser.add_argument("--root-rot-x-deg", type=float, default=90.0)
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1 :]
    else:
        argv = []
    args = parser.parse_args(argv)

    mesh_path = Path(args.mesh)
    bundle_path = Path(args.bundle)
    out_path = Path(args.out) if args.out else None
    track_out = Path(args.track_out) if args.track_out else None
    cart_out = Path(args.cart_out) if args.cart_out else None
    cart_model_path = Path(args.cart_model) if args.cart_model else None

    if out_path is None and (track_out is None or cart_out is None):
        raise RuntimeError("Provide --out for combined export or both --track-out and --cart-out for split export")

    if out_path is not None and track_out is None:
        track_out = out_path.with_name("CoasterTrack.fbx")
    if out_path is not None and cart_out is None:
        cart_out = out_path.with_name("CoasterCartAnimated.fbx")

    if not mesh_path.exists():
        raise FileNotFoundError(f"Mesh not found: {mesh_path}")
    if not bundle_path.exists():
        raise FileNotFoundError(f"Bundle not found: {bundle_path}")

    # Reset scene.
    bpy.ops.wm.read_factory_settings(use_empty=True)

    # Both multipliers silently corrupt any force derived from the animation.
    # Position scales linearly with scale_multiplier, and time scales inversely
    # with speed_multiplier, so acceleration goes as scale/speed^2.
    if abs(args.scale_multiplier - 1.0) > 1e-6 or abs(args.speed_multiplier - 1.0) > 1e-6:
        factor = args.scale_multiplier / (args.speed_multiplier ** 2)
        print("=" * 72)
        print("WARNING: PHYSICS ACCURACY COMPROMISED")
        print(f"  scale_multiplier = {args.scale_multiplier}  (world size x{args.scale_multiplier})")
        print(f"  speed_multiplier = {args.speed_multiplier}  (time x{1.0 / args.speed_multiplier:.4f})")
        print(f"  => every acceleration and G-force reading will be x{factor:.4f}")
        print("  Set both to 1.0 for physically accurate output.")
        print("=" * 72)

    bundle_data, samples, render_path = load_bundle(bundle_path)
    cleanup = (bundle_data.get("cleanup") or {})
    if cleanup.get("applies_to") != "render_path" and cleanup.get("spike_filter_enabled"):
        print(
            "WARNING: this bundle was produced by an older converter that applied "
            "the spike filter to the analytic path. Its physics timeline contains "
            "smoothing artifacts. Re-run the converter to fix."
        )

    track_root = create_root_node(args.scale_multiplier, args.root_rot_x_deg)
    track_root.name = "CoasterTrackRoot"

    pre_import_objs = set(bpy.context.scene.objects)
    imported = try_import_3ds(mesh_path)
    if not imported:
        print("WARNING: Blender 3DS importer is unavailable. Building fallback track mesh from sampled path.")
        build_fallback_track_mesh(render_path)
    build_calibration_cube(args.calibration_cube_m, render_path)
    post_import_objs = [o for o in bpy.context.scene.objects if o not in pre_import_objs]
    parent_objects_to_root(post_import_objs, track_root)

    cart_root = create_root_node(args.scale_multiplier, args.root_rot_x_deg)
    cart_root.name = "CoasterCartRoot"

    rider = build_animation(
        samples,
        args.fps,
        args.speed_multiplier,
        max(args.cart_scale, 0.01),
        cart_model_path,
        args.cart_fit_mode,
        args.cart_target_length,
    )
    print(
        f"Baked {args.fps}fps keys over {float(samples[-1]['time_s']):.2f}s "
        f"of ride time ({len(samples)} analytic samples)"
    )
    rider.parent = cart_root

    export_fbx_selection(track_out, [track_root])
    print(f"Exported track FBX: {track_out}")

    export_fbx_selection(cart_out, [cart_root])
    print(f"Exported animated cart FBX: {cart_out}")

    if out_path is not None:
        # Legacy combined output for compatibility.
        export_fbx_selection(out_path, [track_root, cart_root])
        print(f"Exported combined animated FBX: {out_path}")


if __name__ == "__main__":
    main()
