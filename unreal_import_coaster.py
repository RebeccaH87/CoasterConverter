"""
Run inside Unreal Editor Python.

Builds, from one JSON bundle:
- a track spline actor from the bundle's render_path
- the coaster car, spawned from a mesh asset or imported from a mesh file
- a Level Sequence driving the car along the ride at true scale and true timing

The car mesh is recorded in the bundle by convert_nlelem_to_ue.py, so the usual
call needs nothing but the bundle path. car_mesh_asset overrides it when you
want to try a different car without re-running the converter.

Usage in the UE Python console:

import unreal
exec(open(r"C:/Users/rhutto2/Documents/TestCoaster/UE5_CoasterPipeline/unreal_import_coaster.py").read())

import_coaster_bundle(
    bundle_path=r"C:/Users/rhutto2/Documents/TestCoaster/CoasterRawExportData/UE5/coaster_ue5_bundle.json",
)

# Pick a different car, and also fly a camera along the same path:
import_coaster_bundle(
    bundle_path=r".../coaster_ue5_bundle.json",
    car_mesh_asset="/Game/Coaster/SM_MyCoasterCar",
    also_animate_actor_name="CineCameraActor_0",
)
"""

import json
import math
import os
import unreal


def _find_actor_by_name(name: str):
    for actor in unreal.EditorLevelLibrary.get_all_level_actors():
        if actor.get_name() == name:
            return actor
    return None


def _sample_axes_ue(sample):
    """Return (tangent, up) in Unreal space.

    v2 bundles carry ue_tan/ue_up, already axis-converted. v1 bundles only have
    source-space tan/up, which must not be combined with ue_pos_cm: mixing an
    Unreal-space position with a source-space orientation leaves the cart facing
    the wrong way relative to the path it follows.
    """
    if "ue_tan" in sample and "ue_up" in sample:
        return sample["ue_tan"], sample["ue_up"]

    raise RuntimeError(
        "Bundle predates ue_tan/ue_up (format ue5_coaster_bundle_v1). Re-run "
        "convert_nlelem_to_ue.py so orientation is expressed in Unreal space."
    )


def report_bundle_extent(samples, label="track"):
    """Print the real-world size of what is about to be created.

    Without an FBX in the loop there is no importer to negotiate units with, so
    the bundle's centimetres are the delivered centimetres. Printing the extent
    lets it be checked against the source design in one glance.
    """
    axes = []
    for k, name in enumerate("XYZ"):
        vals = [s["ue_pos_cm"][k] for s in samples]
        axes.append((name, min(vals), max(vals), max(vals) - min(vals)))

    unreal.log(f"{label} extent in Unreal world space:")
    for name, lo, hi, span in axes:
        unreal.log(
            f"  {name}: {lo:11.1f} to {hi:11.1f} cm   span {span:9.1f} cm "
            f"= {span / 100.0:7.2f} m"
        )
    unreal.log("  (Z span is ride height; 1 Unreal unit = 1 cm = 0.01 m)")


def report_force_envelope(samples):
    """Summarise the analytic forces, separating flagged geometry from real."""
    g0 = 9.80665
    clean = sorted(
        s["normal_acc_mps2"] / g0 for s in samples if not s.get("suspect")
    )
    suspect = sum(1 for s in samples if s.get("suspect"))

    def pct(vals, q):
        if not vals:
            return 0.0
        return vals[min(len(vals) - 1, max(0, int(round(q * (len(vals) - 1)))))]

    unreal.log(
        f"Analytic normal G (excluding {suspect} flagged samples): "
        f"median {pct(clean, 0.5):.2f}  p95 {pct(clean, 0.95):.2f}  "
        f"peak {clean[-1] if clean else 0.0:.2f}"
    )
    if suspect:
        unreal.log_warning(
            f"{suspect} of {len(samples)} samples sit on defective source "
            "geometry (gaps or cusps in the .nlelem export). Forces there are "
            "artefacts. See source_defects in the bundle."
        )


def _norm(v):
    length = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    return [v[0] / length, v[1] / length, v[2] / length] if length > 1e-12 else [0.0, 0.0, 1.0]


def _cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _basis_from_tangent_up(tangent, up):
    """Orthonormal basis: local X along travel, local Z along the banked up."""
    ax = _norm(tangent)
    az = _norm(up)
    ay = _norm(_cross(az, ax))
    az = _norm(_cross(ax, ay))
    return ax, ay, az


def _rotator_from_tangent_up(tangent, up):
    """Build an FRotator from a basis, in pure Python.

    The obvious route - unreal.Vector.get_safe_normal and Matrix.rotator - is not
    exposed in the Python binding and raised AttributeError at the first sample,
    so this reimplements FMatrix::Rotator directly.
    """
    ax, ay, az = _basis_from_tangent_up(tangent, up)

    pitch = math.degrees(math.atan2(ax[2], math.sqrt(ax[0] * ax[0] + ax[1] * ax[1])))
    yaw = math.degrees(math.atan2(ax[1], ax[0]))

    # Y axis of a rotation built from (pitch, yaw, roll=0), which is horizontal.
    yaw_rad = math.radians(yaw)
    sy_axis = [-math.sin(yaw_rad), math.cos(yaw_rad), 0.0]
    roll = math.degrees(math.atan2(_dot(az, sy_axis), _dot(ay, sy_axis)))

    return unreal.Rotator(roll=roll, pitch=pitch, yaw=yaw)


def _quat_from_tangent_up(tangent, up):
    """Quaternion from the same basis, for animation bone tracks."""
    ax, ay, az = _basis_from_tangent_up(tangent, up)
    m = (
        (ax[0], ay[0], az[0]),
        (ax[1], ay[1], az[1]),
        (ax[2], ay[2], az[2]),
    )
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w, x, y, z = (
            0.25 * s,
            (m[2][1] - m[1][2]) / s,
            (m[0][2] - m[2][0]) / s,
            (m[1][0] - m[0][1]) / s,
        )
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w, x, y, z = (
            (m[2][1] - m[1][2]) / s,
            0.25 * s,
            (m[0][1] + m[1][0]) / s,
            (m[0][2] + m[2][0]) / s,
        )
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w, x, y, z = (
            (m[0][2] - m[2][0]) / s,
            (m[0][1] + m[1][0]) / s,
            0.25 * s,
            (m[1][2] + m[2][1]) / s,
        )
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w, x, y, z = (
            (m[1][0] - m[0][1]) / s,
            (m[0][2] + m[2][0]) / s,
            (m[1][2] + m[2][1]) / s,
            0.25 * s,
        )
    return unreal.Quat(x, y, z, w)


def build_track_spline(render_path, actor_label: str):
    """Add a SplineComponent actor following the track, if the engine allows it.

    A convenience for snapping other actors to the ride, not a deliverable, so a
    failure here is reported and stepped over. The obvious construction -
    unreal.SplineComponent(actor) with add_instance_component - is not exposed in
    the Python binding and raised AttributeError, which used to abort the whole
    import before the car or the animation were built.
    """
    try:
        actor = unreal.EditorLevelLibrary.spawn_actor_from_class(
            unreal.Actor, unreal.Vector(0.0, 0.0, 0.0)
        )
        actor.set_actor_label(actor_label)
        spline = actor.add_component_by_class(
            unreal.SplineComponent, False, unreal.Transform(), False
        )
        if spline is None:
            raise RuntimeError("add_component_by_class returned nothing")

        spline.clear_spline_points(False)
        world_space = unreal.SplineCoordinateSpace.WORLD
        for row in render_path:
            pos = row["ue_pos_cm"]
            spline.add_spline_point(
                unreal.Vector(pos[0], pos[1], pos[2]), world_space, False
            )
        spline.update_spline()
        unreal.log(
            f"Track spline '{actor_label}': "
            f"{spline.get_number_of_spline_points()} points"
        )
        return actor
    except Exception as ex:
        unreal.log_warning(
            f"Could not build the track spline ({ex}). The track mesh, car and "
            "animation are unaffected."
        )
        return None


def import_car_animation_fbx(bundle_path: str, data: dict, destination: str):
    """Import CoasterCarAnimated.fbx and return (skeleton, skeletal mesh).

    That file is a single-bone skeletal mesh of the car, so this yields a
    Skeleton to hang an AnimSequence on and a SkeletalMesh shaped like the real
    vehicle.
    """
    car = data.get("car") or {}
    name = (car.get("animation_fbx") or "").strip()
    if not name:
        return None, None

    fbx = os.path.join(os.path.dirname(os.path.abspath(bundle_path)), name)
    if not os.path.isfile(fbx):
        unreal.log_warning(f"Animated car FBX not found beside the bundle: {fbx}")
        return None, None

    unreal.log(f"Importing animated car {name} -> {destination}")
    task = unreal.AssetImportTask()
    task.set_editor_property("filename", fbx)
    task.set_editor_property("destination_path", destination)
    task.set_editor_property("automated", True)
    task.set_editor_property("replace_existing", True)
    task.set_editor_property("save", True)
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])

    skeleton = None
    skeletal_mesh = None
    for path in task.get_editor_property("imported_object_paths") or []:
        asset = unreal.load_asset(path)
        if isinstance(asset, unreal.Skeleton):
            skeleton = asset
        elif isinstance(asset, unreal.SkeletalMesh):
            skeletal_mesh = asset

    if skeleton is None:
        unreal.log_warning(
            f"{name} imported without a Skeleton; no AnimSequence can be built."
        )
    return skeleton, skeletal_mesh


def _animation_sampling_rate(fallback=(30, 1)):
    """The frame rate Unreal will actually store animation data at.

    Keys written at any other rate are resampled to the project's Animation
    "Default Frame Rate", and if the play length is then not a whole number of
    frames at that rate the compressor hits
    check(IsNearlyZero(SampleFrameTime.GetSubFrame())) and takes the editor down.
    So read the rate first and key straight onto it - writing at a higher rate
    buys nothing, since the resample throws the extra keys away.
    """
    try:
        rate = unreal.get_default_object(unreal.AnimationSettings).get_editor_property(
            "default_frame_rate"
        )
        num, den = int(rate.numerator), int(rate.denominator)
        if num > 0 and den > 0:
            return num, den
    except Exception as ex:
        unreal.log_warning(f"Could not read the project animation frame rate ({ex}).")
    return fallback


def create_car_anim_sequence(
    skeleton,
    samples,
    fps: int,
    destination: str,
    asset_name: str = "CoasterCarAnim",
    bone_name: str = "",
):
    """Bake the ride onto the car's bone as an AnimSequence asset.

    Written here rather than relying on the FBX: Unreal's FBX translator would
    not produce an AnimSequence from the exported file, and building it through
    the animation data controller is both reliable and verifiable - the keys come
    straight from the bundle, so the asset matches the analytic path to well
    under a centimetre.
    """
    if skeleton is None or not samples:
        return None

    duration = float(samples[-1]["time_s"])
    if duration <= 0.0:
        return None

    rate_num, rate_den = _animation_sampling_rate()
    key_fps = rate_num / float(rate_den)
    if abs(key_fps - float(fps)) > 1e-6:
        unreal.log(
            f"  keying at the project animation rate {rate_num}/{rate_den} fps; "
            f"the requested {fps}fps would be resampled to it anyway"
        )

    # Floor, so the last frame lands on or before the end of the ride. Rounding up
    # would put it past the end, its key would be clamped back to the source
    # duration, and the play length would no longer be a whole number of frames -
    # which is exactly what the compressor asserts on. Costs under one frame of tail.
    frame_span = max(int(math.floor(duration * key_fps)), 1)
    frame_count = frame_span + 1

    times = [float(s["time_s"]) for s in samples]

    def sample_at(t):
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

    positions = []
    rotations = []
    scales = []
    for frame in range(frame_count):
        row = sample_at(frame * rate_den / float(rate_num))
        pos = row["ue_pos_cm"]
        positions.append(unreal.Vector(pos[0], pos[1], pos[2]))
        rotations.append(_quat_from_tangent_up(row["ue_tan"], row["ue_up"]))
        scales.append(unreal.Vector(1.0, 1.0, 1.0))

    factory = unreal.AnimSequenceFactory()
    factory.set_editor_property("target_skeleton", skeleton)
    anim = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
        asset_name, destination, unreal.AnimSequence, factory
    )
    if anim is None:
        unreal.log_warning("Could not create the AnimSequence asset.")
        return None

    # The bone the exporter skins to, with fallbacks in case it was renamed.
    candidates = [bone_name] if bone_name else []
    candidates += ["CoasterCarBone", "CoasterCarRig", "root"]
    chosen = ""
    for candidate in candidates:
        if not candidate:
            continue
        try:
            if unreal.AnimationLibrary.does_bone_name_exist(anim, candidate):
                chosen = candidate
                break
        except Exception:
            continue
    if not chosen:
        unreal.log_warning(
            "Could not find the car bone on the skeleton; AnimSequence left empty."
        )
        return anim

    # Ancestors of the driven bone need tracks too, or compression walks a bone
    # with no keys. They are all identity - the car is one rigid body.
    try:
        chain = [
            str(b)
            for b in unreal.AnimationLibrary.find_bone_path_to_root(anim, chosen)
        ]
    except Exception:
        chain = [chosen]
    rest_pos = [unreal.Vector(0.0, 0.0, 0.0)] * frame_count
    rest_rot = [unreal.Quat(0.0, 0.0, 0.0, 1.0)] * frame_count

    controller = anim.controller
    controller.open_bracket("Coaster ride import")
    controller.set_frame_rate(unreal.FrameRate(rate_num, rate_den))
    # Frames are intervals, so one fewer than the number of keys.
    controller.set_number_of_frames(unreal.FrameNumber(frame_span))
    for bone in chain:
        controller.add_bone_curve(bone)
        if bone == chosen:
            controller.set_bone_track_keys(bone, positions, rotations, scales)
        else:
            controller.set_bone_track_keys(bone, rest_pos, rest_rot, scales)
    controller.close_bracket()

    stored = anim.get_editor_property("number_of_sampled_frames")
    if int(stored) != frame_span:
        unreal.log_warning(
            f"Unreal stored {stored} frames where {frame_span} were written; the "
            "animation may have been resampled and could be misaligned."
        )

    unreal.EditorAssetLibrary.save_loaded_asset(anim)
    unreal.log(
        f"AnimSequence '{asset_name}': {frame_count} keys at {rate_num}/{rate_den}fps "
        f"on bone '{chosen}' ({len(chain)} track(s)), {anim.get_play_length():.2f}s "
        f"of {duration:.2f}s"
    )
    return anim



# Which way to yaw a mesh so its own forward axis ends up along +X, because the
# path frame this script builds always puts travel along +X.
FORWARD_AXIS_YAW = {"+X": 0.0, "-Y": 90.0, "-X": 180.0, "+Y": -90.0}


def locate_car_mesh_file(car: dict, bundle_path: str):
    """Find the staged car mesh, which sits next to the bundle.

    The converter copies it there and records only the filename, so moving the
    whole export folder keeps working. mesh_file_source is the original location
    and is only consulted if the staged copy has gone missing.
    """
    name = (car.get("mesh_file") or "").strip()
    bundle_dir = os.path.dirname(os.path.abspath(bundle_path))

    candidates = []
    if name:
        # A bare filename is the normal case; an absolute path is honoured too,
        # so bundles written before staging existed still import.
        candidates.append(name if os.path.isabs(name) else os.path.join(bundle_dir, name))
    source = (car.get("mesh_file_source") or "").strip()
    if source:
        candidates.append(source)

    for path in candidates:
        if os.path.isfile(path):
            return path

    if candidates:
        unreal.log_warning(
            "Car mesh file not found. Looked in: " + ", ".join(candidates)
        )
    return ""


def resolve_car_mesh(car: dict, bundle_path: str, destination: str):
    """Return (mesh_asset, description) for the configured coaster car.

    Prefers an existing asset. Falls back to importing the staged file, so a car
    that only exists on disk still works without a manual import step first.
    """
    asset_path = (car.get("mesh_asset") or "").strip()
    if asset_path:
        mesh = unreal.load_asset(asset_path)
        if mesh:
            return mesh, asset_path
        unreal.log_warning(f"Car mesh asset not found: {asset_path}")

    mesh_file = locate_car_mesh_file(car, bundle_path)
    if not mesh_file:
        return None, ""

    # Reuse the asset if this mesh was already imported on a previous run,
    # rather than piling up SM_Car, SM_Car_1, SM_Car_2 in the content folder.
    stem = os.path.splitext(os.path.basename(mesh_file))[0]
    existing = unreal.load_asset(f"{destination}/{stem}")
    if existing is not None and isinstance(
        existing, (unreal.StaticMesh, unreal.SkeletalMesh)
    ):
        unreal.log(f"Reusing already-imported car mesh {destination}/{stem}")
        return existing, f"{destination}/{stem}"

    unreal.log(f"Importing car mesh {mesh_file} -> {destination}")

    task = unreal.AssetImportTask()
    task.set_editor_property("filename", mesh_file)
    task.set_editor_property("destination_path", destination)
    task.set_editor_property("automated", True)
    task.set_editor_property("replace_existing", True)
    task.set_editor_property("save", True)
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])

    for path in task.get_editor_property("imported_object_paths") or []:
        obj = unreal.load_asset(path)
        if isinstance(obj, (unreal.StaticMesh, unreal.SkeletalMesh)):
            return obj, path

    unreal.log_warning(f"Import produced no usable mesh from {mesh_file}")
    return None, ""


def _assign_mesh(component, mesh, property_names):
    """Set a mesh onto a component, tolerating property renames across versions."""
    for name in property_names:
        try:
            component.set_editor_property(name, mesh)
            return
        except Exception:
            continue
    raise RuntimeError(
        f"Could not assign the mesh: none of {property_names} were settable on "
        f"{type(component).__name__}"
    )


def _mesh_bounds_extent_cm(mesh):
    try:
        box = mesh.get_bounding_box()
        size = box.max - box.min
        return [abs(size.x), abs(size.y), abs(size.z)]
    except Exception:
        return None


def detect_forward_axis(mesh) -> str:
    """Guess which way the car model faces from its bounding box.

    A coaster car is always longer than it is wide, so the longer horizontal
    axis is the one it faces down. Worth detecting rather than defaulting,
    because getting it wrong mounts the car sideways on the track and that is
    easy to mistake for the mesh itself being broken.
    """
    extent = _mesh_bounds_extent_cm(mesh)
    if not extent:
        unreal.log_warning("  could not measure the car; assuming it faces +X")
        return "+X"

    axis = "+X" if extent[0] >= extent[1] else "+Y"
    unreal.log(
        f"  auto forward axis: {axis} (mesh is {extent[0] / 100.0:.2f} m on X, "
        f"{extent[1] / 100.0:.2f} m on Y)"
    )
    return axis


def spawn_car_actor(mesh, car: dict, label: str):
    """Spawn the car and push orientation, offset and scale onto its component.

    Keeping those on the component leaves the actor's own transform as the pure
    path frame, so the Level Sequence keys stay physically meaningful and
    CoasterAnalyzer reads the same motion the converter computed.
    """
    if isinstance(mesh, unreal.SkeletalMesh):
        actor = unreal.EditorLevelLibrary.spawn_actor_from_class(
            unreal.SkeletalMeshActor, unreal.Vector(0.0, 0.0, 0.0)
        )
        component = actor.skeletal_mesh_component
        # Renamed to skeletal_mesh_asset in UE 5.1; try both so this works
        # across engine versions rather than failing on a property name.
        _assign_mesh(component, mesh, ("skeletal_mesh_asset", "skeletal_mesh"))
    else:
        actor = unreal.EditorLevelLibrary.spawn_actor_from_class(
            unreal.StaticMeshActor, unreal.Vector(0.0, 0.0, 0.0)
        )
        component = actor.static_mesh_component
        try:
            component.set_static_mesh(mesh)
        except Exception:
            _assign_mesh(component, mesh, ("static_mesh",))

    actor.set_actor_label(label)

    roll, pitch, yaw = car.get("rotation_offset_deg") or [0.0, 0.0, 0.0]
    forward_axis = (car.get("forward_axis") or "auto").strip()
    if forward_axis == "auto":
        forward_axis = detect_forward_axis(mesh)
    base_yaw = FORWARD_AXIS_YAW.get(forward_axis, 0.0)
    component.set_editor_property(
        "relative_rotation",
        unreal.Rotator(roll=float(roll), pitch=float(pitch), yaw=base_yaw + float(yaw)),
    )

    off = car.get("offset_cm") or [0.0, 0.0, 0.0]
    component.set_editor_property(
        "relative_location", unreal.Vector(float(off[0]), float(off[1]), float(off[2]))
    )

    scale = float(car.get("scale") or 1.0)
    component.set_editor_property(
        "relative_scale3d", unreal.Vector(scale, scale, scale)
    )

    unreal.log(
        f"Car '{label}': forward {forward_axis} "
        f"(yaw {base_yaw:+.0f}), offset {off}, scale {scale}"
    )

    extent = _mesh_bounds_extent_cm(mesh)
    expected_m = float(car.get("expected_length_m") or 0.0)
    if extent:
        longest_cm = max(extent) * scale
        unreal.log(
            f"  mesh size {extent[0] * scale:.1f} x {extent[1] * scale:.1f} x "
            f"{extent[2] * scale:.1f} cm (longest {longest_cm / 100.0:.2f} m)"
        )
        if expected_m > 0.0:
            error = 100.0 * (longest_cm / 100.0 - expected_m) / expected_m
            if abs(error) <= 5.0:
                unreal.log(
                    f"  SCALE OK: within {error:+.1f}% of the expected "
                    f"{expected_m:.2f} m"
                )
            else:
                unreal.log_warning(
                    f"  car is {longest_cm / 100.0:.2f} m but {expected_m:.2f} m "
                    f"was expected ({error:+.1f}%). Set --car-scale to "
                    f"{expected_m / max(longest_cm / 100.0 / scale, 1e-6):.4f} "
                    "to correct it."
                )

    return actor


def build_placeholder_car(label: str):
    """A visible stand-in so the motion can be checked before a car is chosen."""
    cube = unreal.load_asset("/Engine/BasicShapes/Cube")
    if cube is None:
        unreal.log_warning("Engine cube not available; no placeholder car spawned.")
        return None

    actor = unreal.EditorLevelLibrary.spawn_actor_from_class(
        unreal.StaticMeshActor, unreal.Vector(0.0, 0.0, 0.0)
    )
    actor.set_actor_label(label)
    component = actor.static_mesh_component
    component.set_static_mesh(cube)
    # The engine cube is 100cm; make it read as a ~4.5m car body.
    component.set_editor_property(
        "relative_scale3d", unreal.Vector(4.5, 1.6, 1.2)
    )
    unreal.log(f"No car configured; spawned placeholder '{label}' (4.5 x 1.6 x 1.2 m)")
    return actor


def import_track_mesh(bundle_path: str, data: dict, destination: str):
    """Import the procedural track and place it at the origin.

    The geometry is written in absolute Unreal coordinates, so the actors carry
    an identity transform: the track lands exactly where the ride is, with no
    alignment step.

    Prefers CoasterTrack.glb. Unreal's FBX importer mirrors the scene in Y,
    which puts an FBX track nowhere near the glTF car while still passing every
    span and bounds check, because a mirror preserves both.
    """
    info = data.get("track_mesh") or {}
    name = (data.get("track_mesh_glb") or "").strip() or (info.get("file") or "").strip()
    if not name:
        return []
    if not name.lower().endswith(".glb"):
        unreal.log_warning(
            f"Importing the track from {name}; Unreal mirrors FBX in Y, so it "
            "will not line up with the car. Re-run the converter to get "
            "CoasterTrack.glb."
        )

    fbx = os.path.join(os.path.dirname(os.path.abspath(bundle_path)), name)
    if not os.path.isfile(fbx):
        unreal.log_warning(f"Track mesh not found beside the bundle: {fbx}")
        return []

    unreal.log(f"Importing track mesh {name} -> {destination}")
    task = unreal.AssetImportTask()
    task.set_editor_property("filename", fbx)
    task.set_editor_property("destination_path", destination)
    task.set_editor_property("automated", True)
    task.set_editor_property("replace_existing", True)
    task.set_editor_property("save", True)

    # Keep the four parts separate so each can take its own material. Wrapped
    # because the options object differs between engine versions and a failure
    # here should cost the material split, not the import.
    try:
        options = unreal.FbxImportUI()
        options.set_editor_property("import_mesh", True)
        options.set_editor_property("import_as_skeletal", False)
        options.set_editor_property("import_animations", False)
        options.static_mesh_import_data.set_editor_property("combine_meshes", False)
        options.static_mesh_import_data.set_editor_property(
            "convert_scene", False
        )
        task.set_editor_property("options", options)
    except Exception as ex:
        unreal.log_warning(f"Could not set FBX import options ({ex}); using defaults.")

    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])

    actors = []
    for path in task.get_editor_property("imported_object_paths") or []:
        asset = unreal.load_asset(path)
        if not isinstance(asset, unreal.StaticMesh):
            continue
        actor = unreal.EditorLevelLibrary.spawn_actor_from_class(
            unreal.StaticMeshActor, unreal.Vector(0.0, 0.0, 0.0)
        )
        actor.static_mesh_component.set_static_mesh(asset)
        actor.set_actor_label(asset.get_name())
        actors.append(actor)

    if actors:
        unreal.log(
            f"Track: {len(actors)} part(s) placed - "
            + ", ".join(a.get_actor_label() for a in actors)
        )
        unreal.log(
            f"  {info.get('polygons', '?')} polygons, "
            f"{info.get('supports', '?')} support columns, "
            f"gauge {info.get('gauge_cm', '?')}cm"
        )
    else:
        unreal.log_warning("Track FBX imported but produced no static meshes.")
    return actors


def add_transform_track(seq, actor, samples, ticks_per_second, label=""):
    """Key an actor's transform along the ride. Returns the created binding."""
    binding = seq.add_possessable(actor)
    track = binding.add_track(unreal.MovieScene3DTransformTrack)
    section = track.add_section()
    section.set_range_seconds(0.0, float(samples[-1]["time_s"]))

    channels = section.get_all_channels()
    tx, ty, tz, rx, ry, rz = channels[:6]

    for s in samples:
        p = s["ue_pos_cm"]
        tan_ue, up_ue = _sample_axes_ue(s)
        rot = _rotator_from_tangent_up(tan_ue, up_ue)
        frame = unreal.FrameNumber(
            int(round(float(s["time_s"]) * ticks_per_second))
        )
        tx.add_key(frame, p[0])
        ty.add_key(frame, p[1])
        tz.add_key(frame, p[2])
        rx.add_key(frame, rot.roll)
        ry.add_key(frame, rot.pitch)
        rz.add_key(frame, rot.yaw)

    unreal.log(
        f"Keyed {len(samples)} transforms onto "
        f"{label or actor.get_actor_label()}"
    )
    return binding


def import_coaster_bundle(
    bundle_path: str,
    spline_actor_name: str = "BP_CoasterSplineActor",
    level_sequence_path: str = "/Game/Coaster/LS_CoasterRide",
    display_fps: int = 120,
    car_mesh_asset: str = "",
    car_actor_name: str = "CoasterCar",
    spawn_placeholder_car: bool = True,
    also_animate_actor_name: str = "",
    import_track: bool = True,
    build_anim_sequence: bool = True,
):
    """Build the track spline and a Level Sequence that drives the coaster car.

    car_mesh_asset overrides whatever the bundle recorded, so a different car can
    be tried without re-running the converter.

    also_animate_actor_name keys an existing actor - typically a camera for a POV
    pass - along the identical path. It is optional: a missing actor is reported
    and skipped rather than aborting an otherwise good import.

    import_track brings in CoasterTrack.fbx if the converter wrote one.

    build_anim_sequence imports CoasterCarAnimated.fbx - a single-bone skeletal
    mesh of the car - and bakes the ride onto its bone as an AnimSequence asset.
    """
    data = json.load(open(bundle_path, "r", encoding="utf-8"))
    samples = data["samples"]

    if not samples:
        raise RuntimeError("No samples found in bundle")

    units = data.get("units") or {}
    scale = units.get("metres_to_unreal_units")
    if scale is not None and abs(float(scale) - 100.0) > 1e-6:
        raise RuntimeError(
            f"Bundle declares {scale} Unreal units per metre; expected 100."
        )

    handedness = data.get("handedness") or {}
    if handedness and not handedness.get("preserves_handedness", True):
        unreal.log_error(
            f"Bundle used axis mapping '{data.get('source', {}).get('axis_mapping')}' "
            f"with determinant {handedness.get('mapping_determinant')}. The track "
            "is MIRRORED and lateral-G signs are inverted. Re-run the converter "
            "with --axis-mapping nl2_to_ue_swap_yz."
        )

    validation = data.get("validation")
    if validation and not validation.get("selected_is_best", True):
        unreal.log_warning(
            "Converter validation reported that "
            f"'{validation.get('best_mapping')}' fits the reference path better "
            f"than the mapping used ('{validation.get('selected_mapping')}')."
        )

    # Geometry comes from render_path (spike-filtered, safe to look at) while
    # motion comes from samples (never geometry-edited, so forces stay true).
    render_path = data.get("render_path") or samples
    report_bundle_extent(render_path, "track")

    build_track_spline(render_path, spline_actor_name)

    # Create or load the level sequence.
    seq = unreal.load_asset(level_sequence_path)
    if not seq:
        asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
        package_path, asset_name = level_sequence_path.rsplit("/", 1)
        seq = asset_tools.create_asset(
            asset_name,
            package_path,
            unreal.LevelSequence,
            unreal.LevelSequenceFactoryNew(),
        )

    # Key times must be expressed in the sequence's tick resolution, not in
    # display frames. Hardcoding 30 quantised the whole ride to 30 ticks per
    # second regardless of what the asset actually used.
    tick = seq.get_tick_resolution()
    ticks_per_second = float(tick.numerator) / float(tick.denominator)
    seq.set_display_rate(unreal.FrameRate(numerator=int(display_fps), denominator=1))
    unreal.log(
        f"Sequence tick resolution {ticks_per_second:.0f}/s, "
        f"display rate {display_fps}fps"
    )

    # ---- the coaster car ----
    car = dict(data.get("car") or {})
    if car_mesh_asset:
        car["mesh_asset"] = car_mesh_asset

    # Everything this script creates goes in one content folder: the one the
    # level sequence lives in. That is why there is no import-folder setting.
    content_dir = level_sequence_path.rsplit("/", 1)[0]

    if import_track:
        import_track_mesh(bundle_path, data, content_dir)

    if build_anim_sequence:
        skeleton, skeletal_car = import_car_animation_fbx(
            bundle_path, data, content_dir
        )
        if skeleton is not None:
            create_car_anim_sequence(
                skeleton,
                samples,
                int((data.get("car") or {}).get("animation_fbx_fps") or 60),
                content_dir,
            )

    car_actor = None
    mesh, mesh_desc = resolve_car_mesh(car, bundle_path, content_dir)
    if mesh is not None:
        car_actor = spawn_car_actor(mesh, car, car_actor_name)
        unreal.log(f"Car mesh: {mesh_desc}")
    elif spawn_placeholder_car:
        car_actor = build_placeholder_car(car_actor_name)
        unreal.log_warning(
            "No car mesh was resolved, so a placeholder is animated instead. "
            "Set the car mesh in the GUI, pass --car-mesh-asset / "
            "--car-mesh-file to the converter, or call this function with "
            "car_mesh_asset=... to use a real car."
        )

    if car_actor is not None:
        add_transform_track(seq, car_actor, samples, ticks_per_second, car_actor_name)
    else:
        unreal.log_warning("No car actor to animate.")

    # ---- optional extra rider, usually a POV camera ----
    if also_animate_actor_name:
        extra = _find_actor_by_name(also_animate_actor_name)
        if extra is None:
            unreal.log_warning(
                f"Actor '{also_animate_actor_name}' not found; skipping it. "
                "The car was still animated."
            )
        else:
            add_transform_track(
                seq, extra, samples, ticks_per_second, also_animate_actor_name
            )

    unreal.EditorAssetLibrary.save_loaded_asset(seq)
    unreal.log(f"Imported coaster spline + sequence from {bundle_path}")
    unreal.log(
        f"Ride duration {float(samples[-1]['time_s']):.2f}s over "
        f"{float(samples[-1]['distance_m']):.1f}m, "
        f"peak speed {max(float(s['speed_mps']) for s in samples):.1f} m/s"
    )
    report_force_envelope(samples)
    if car_actor is not None:
        unreal.log(
            f"Add a CoasterAnalyzer component to '{car_actor_name}' with "
            "Use Live Actor Tracking enabled to read these forces during PIE."
        )

