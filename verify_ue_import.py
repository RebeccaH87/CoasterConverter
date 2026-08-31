"""
Verify the exported FBX files against Unreal's own importer.

The other verifiers use Blender as an independent reader, which is useful but
more forgiving than Autodesk's FBX SDK - the SDK is what Unreal's Interchange
importer uses, and it rejected files Blender read happily. This is the only
check that exercises the real thing.

Imports without saving, so nothing is written into the project.

    UnrealEditor-Cmd.exe <project.uproject> ^
        -ExecutePythonScript="verify_ue_import.py" ^
        -unattended -nopause -nosplash -stdout

Set FILES below to the exports you want checked, then read the LogPython lines.
A healthy run reports one object per mesh and no warnings.
"""

import unreal

FILES = [
    r"C:/Users/rhutto2/Documents/Digital Humans Class/Project1/CoasterTrack.fbx",
    r"C:/Users/rhutto2/Documents/Digital Humans Class/Project1/CoasterCarAnimated.fbx",
]

unreal.log("=========== COASTER FBX IMPORT TEST START ===========")

tools = unreal.AssetToolsHelpers.get_asset_tools()

for path in FILES:
    name = path.rsplit("/", 1)[-1]
    unreal.log(f"--- TESTING {name}")

    task = unreal.AssetImportTask()
    task.set_editor_property("filename", path)
    task.set_editor_property("destination_path", "/Game/CoasterImportTest")
    task.set_editor_property("automated", True)
    task.set_editor_property("replace_existing", True)
    # Deliberately not saving: this is a read-only validation of the parser.
    task.set_editor_property("save", False)

    try:
        options = unreal.FbxImportUI()
        options.set_editor_property("import_mesh", True)
        options.set_editor_property("import_as_skeletal", False)
        options.set_editor_property("import_animations", False)
        options.static_mesh_import_data.set_editor_property("combine_meshes", False)
        task.set_editor_property("options", options)
        unreal.log("    options: combine_meshes=False accepted")
    except Exception as ex:
        unreal.log_warning(f"    options rejected ({ex}); using defaults")

    try:
        tools.import_asset_tasks([task])
    except Exception as ex:
        unreal.log_error(f"    IMPORT THREW: {ex}")
        continue

    paths = task.get_editor_property("imported_object_paths") or []
    unreal.log(f"    RESULT {name}: {len(paths)} object(s) imported")
    for p in paths:
        obj = unreal.load_asset(p)
        kind = type(obj).__name__ if obj else "unresolved"
        detail = ""
        if isinstance(obj, unreal.StaticMesh):
            try:
                box = obj.get_bounding_box()
                size = box.max - box.min
                detail = (
                    f" bounds {size.x/100.0:.1f} x {size.y/100.0:.1f} x "
                    f"{size.z/100.0:.1f} m"
                )
            except Exception:
                pass
        unreal.log(f"      {kind}: {p}{detail}")

    if not paths:
        unreal.log_error(f"    {name} PRODUCED NOTHING")

unreal.log("=========== COASTER FBX IMPORT TEST END ===========")
