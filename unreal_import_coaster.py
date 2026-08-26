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


def _rotator_from_tangent_up(tangent, up):
    x_axis = unreal.Vector(tangent[0], tangent[1], tangent[2]).get_safe_normal()
    z_axis = unreal.Vector(up[0], up[1], up[2]).get_safe_normal()
    y_axis = unreal.Vector.cross_product(z_axis, x_axis).get_safe_normal()
    z_axis = unreal.Vector.cross_product(x_axis, y_axis).get_safe_normal()
    m = unreal.Matrix(
        x_plane=x_axis,
        y_plane=y_axis,
        z_plane=z_axis,
        w_plane=unreal.Vector(0.0, 0.0, 0.0),
    )
    return m.rotator()


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
    base_yaw = FORWARD_AXIS_YAW.get(car.get("forward_axis") or "+X", 0.0)
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
        f"Car '{label}': forward {car.get('forward_axis', '+X')} "
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
):
    """Build the track spline and a Level Sequence that drives the coaster car.

    car_mesh_asset overrides whatever the bundle recorded, so a different car can
    be tried without re-running the converter.

    also_animate_actor_name keys an existing actor - typically a camera for a POV
    pass - along the identical path. It is optional: a missing actor is reported
    and skipped rather than aborting an otherwise good import.
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

    # Spawn an Empty Actor with SplineComponent.
    world = unreal.EditorLevelLibrary.get_editor_world()
    actor = unreal.EditorLevelLibrary.spawn_actor_from_class(unreal.Actor, unreal.Vector(0, 0, 0))
    actor.set_actor_label(spline_actor_name)

    spline = unreal.SplineComponent(actor)
    actor.add_instance_component(spline)
    spline.register_component()

    # Geometry comes from render_path (spike-filtered, safe to look at) while
    # motion comes from samples (never geometry-edited, so forces stay true).
    render_path = data.get("render_path") or samples

    spline.clear_spline_points(False)
    for s in render_path:
        pos = s["ue_pos_cm"]
        tan = s["ue_tan_cm"]
        spline.add_spline_point(unreal.Vector(pos[0], pos[1], pos[2]), unreal.SplineCoordinateSpace.WORLD, False)
        spline.set_tangent_at_spline_point(
            spline.get_number_of_spline_points() - 1,
            unreal.Vector(tan[0], tan[1], tan[2]),
            unreal.SplineCoordinateSpace.WORLD,
            False,
        )

    spline.update_spline()
    unreal.log(
        f"Track spline: {spline.get_number_of_spline_points()} points from "
        f"{'render_path' if data.get('render_path') else 'samples'}"
    )
    report_bundle_extent(render_path, "track spline")

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

