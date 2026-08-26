"""
Run inside Unreal Editor Python.

Creates:
- A spline actor from ue_pos_cm / ue_tan_cm samples.
- A Level Sequence animating a target actor along the sampled transform timeline.

Usage in UE Python console:
import unreal
exec(open(r"C:/Users/rhutto2/Documents/TestCoaster/UE5_CoasterPipeline/unreal_import_coaster.py").read())
import_coaster_bundle(
    bundle_path=r"C:/Users/rhutto2/Documents/TestCoaster/CoasterRawExportData/UE5/coaster_ue5_bundle.json",
    spline_actor_name="BP_CoasterSplineActor",
    actor_to_animate_name="CineCameraActor_0",
    level_sequence_path="/Game/Coaster/LS_CoasterRide"
)
"""

import json
import math
import unreal


def _find_actor_by_name(name: str):
    for actor in unreal.EditorLevelLibrary.get_all_level_actors():
        if actor.get_name() == name:
            return actor
    return None


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


def import_coaster_bundle(
    bundle_path: str,
    spline_actor_name: str = "BP_CoasterSplineActor",
    actor_to_animate_name: str = "CineCameraActor_0",
    level_sequence_path: str = "/Game/Coaster/LS_CoasterRide",
):
    data = json.load(open(bundle_path, "r", encoding="utf-8"))
    samples = data["samples"]

    if not samples:
        raise RuntimeError("No samples found in bundle")

    # Spawn an Empty Actor with SplineComponent.
    world = unreal.EditorLevelLibrary.get_editor_world()
    actor = unreal.EditorLevelLibrary.spawn_actor_from_class(unreal.Actor, unreal.Vector(0, 0, 0))
    actor.set_actor_label(spline_actor_name)

    spline = unreal.SplineComponent(actor)
    actor.add_instance_component(spline)
    spline.register_component()

    spline.clear_spline_points(False)
    for s in samples:
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

    target_actor = _find_actor_by_name(actor_to_animate_name)
    if not target_actor:
        raise RuntimeError(f"Actor not found: {actor_to_animate_name}")

    binding = seq.add_possessable(target_actor)
    transform_track = binding.add_track(unreal.MovieScene3DTransformTrack)
    section = transform_track.add_section()
    section.set_range_seconds(0.0, samples[-1]["time_s"])

    channels = section.get_all_channels()
    tx, ty, tz, rx, ry, rz = channels[0], channels[1], channels[2], channels[3], channels[4], channels[5]

    for s in samples:
        t = s["time_s"]
        p = s["ue_pos_cm"]
        rot = _rotator_from_tangent_up(s["tan"], s["up"])

        tx.add_key(unreal.FrameNumber.from_float(t * 30.0), p[0])
        ty.add_key(unreal.FrameNumber.from_float(t * 30.0), p[1])
        tz.add_key(unreal.FrameNumber.from_float(t * 30.0), p[2])
        rx.add_key(unreal.FrameNumber.from_float(t * 30.0), rot.roll)
        ry.add_key(unreal.FrameNumber.from_float(t * 30.0), rot.pitch)
        rz.add_key(unreal.FrameNumber.from_float(t * 30.0), rot.yaw)

    unreal.EditorAssetLibrary.save_loaded_asset(seq)
    unreal.log(f"Imported coaster spline + sequence from {bundle_path}")
