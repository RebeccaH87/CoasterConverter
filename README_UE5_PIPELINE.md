# UE5 Coaster Pipeline From FVD++ / NoLimits Export

This folder converts your exported FVD++ data into a UE5-ready pipeline bundle and can export a single animated FBX.

## What this supports

- Reads binary `.nlelem` files exported by FVD++ (format decoded from OpenFVD source).
- Produces one JSON file containing:
  - sampled spline points
  - tangent/up vectors
  - roll
  - gravity-based speed/time timeline
- Optionally writes a CSV timeline.
- Optionally converts `CoasterModel.3ds` to `FBX` if Blender is installed.
- Optionally exports `CoasterAnimated.fbx` that contains track mesh and keyframed cart transform animation.

## Input files expected

- `CoasterSpline.nlelem`
- `CoasterTangent.nlelem`
- `CoasterModel.3ds`

## Quick run (GUI)

Use one app instead of calling multiple scripts.

```powershell
C:/Users/rhutto2/.local/bin/python3.15.exe "C:\Users\rhutto2\Documents\TestCoaster\UE5_CoasterPipeline\coaster_pipeline_gui.py"
```

In the GUI:

- Select `CoasterSpline.nlelem`
- Select `CoasterTangent.nlelem`
- Select `CoasterModel.3ds`
- Select output folder (example: `...\CoasterRawExportData\UE5`)
- Enable `Export animated FBX (track + cart animation)`
- Click **Run Conversion**

Optional:

- Enable `Also convert 3DS to FBX` if Blender is installed and configured.
- Set Blender executable path for animated FBX export.
- Use `Animation speed multiplier` to make the rider/coaster animation faster or slower.
: `1.0` = original, `2.0` = twice as fast, `0.5` = half speed.
- Use `Track/motion scale multiplier` to match UE scene scale.
- `Root X rotation (deg)` defaults to `90` so UE import orientation is corrected automatically.
- `Smooth extreme spline spikes/outliers` removes sharp local protrusions where path flow reverses abruptly.
- Tune `Spike angle threshold (deg)` and `Spike deviation multiplier` if smoothing is too weak or too aggressive.

## Quick run (PowerShell)

```powershell
$base = "C:\Users\rhutto2\Documents\TestCoaster\CoasterRawExportData"
$out = Join-Path $base "UE5"

# If Blender is installed, set this:
$blender = "C:\Program Files\Blender Foundation\Blender 4.2\blender.exe"

powershell -ExecutionPolicy Bypass -File "C:\Users\rhutto2\Documents\TestCoaster\UE5_CoasterPipeline\run_pipeline.ps1" `
  -SplineNLElem (Join-Path $base "CoasterSpline.nlelem") `
  -TangentNLElem (Join-Path $base "CoasterTangent.nlelem") `
  -Mesh3ds (Join-Path $base "CoasterModel.3ds") `
  -OutDir $out `
  -PythonExe "python" `
  -BlenderExe $blender `
  -SamplesPerSegment 20 `
  -AxisMapping "nl2_to_ue" `
  -InitialSpeed 6.0
```

## Output

- `coaster_ue5_bundle.json` (single import bundle for UE tooling)
- `coaster_timeline.csv` (debug/import table)
- `CoasterModel.fbx` (if Blender conversion is enabled)
- `CoasterAnimated.fbx` (if animated FBX export is enabled)

## Blender requirement

If you need FBX with stored animation, Blender executable path is required in the GUI.

## Unreal import

Use `unreal_import_coaster.py` inside Unreal Editor Python. It creates:

- a spline actor
- a Level Sequence that animates a chosen camera/cart actor over time

Edit the actor names/asset path in the example call at the top of that script.

## Notes on physics

The timeline uses an energy model with gravity, rolling friction, and drag. It is tunable from command line:

- `--initial-speed`
- `--rolling-friction`
- `--drag-coeff`

If your speed profile is too fast/slow, tune those first before changing sampling density.
