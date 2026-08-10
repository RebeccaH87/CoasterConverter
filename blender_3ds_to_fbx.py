"""
Run with Blender in background mode:
blender -b -P blender_3ds_to_fbx.py -- --in ".../CoasterModel.3ds" --out ".../CoasterModel.fbx"
"""

import argparse
import sys
import bpy


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--in", dest="in_path", required=True)
    parser.add_argument("--out", dest="out_path", required=True)
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1 :]
    else:
        argv = []
    args = parser.parse_args(argv)

    bpy.ops.wm.read_factory_settings(use_empty=True)
    importer = getattr(bpy.ops.import_scene, "autodesk_3ds", None)
    if importer is None:
        raise RuntimeError(
            "Blender 3DS importer is unavailable in this installation. "
            "Disable 'Also convert 3DS to FBX' or install/enable 3DS import support."
        )

    importer(filepath=args.in_path)

    # Keep source transform but export in UE-friendly axis convention.
    bpy.ops.export_scene.fbx(
        filepath=args.out_path,
        use_selection=False,
        apply_unit_scale=True,
        bake_space_transform=True,
        axis_forward="X",
        axis_up="Z",
    )

    print(f"Exported FBX: {args.out_path}")


if __name__ == "__main__":
    main()
