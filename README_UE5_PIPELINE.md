# UE5 Coaster Pipeline From OpenFVD / NoLimits Export

Converts exported OpenFVD / NoLimits `.nlelem` track data into a JSON bundle that
Unreal drives directly, with correct real-world scale and physically meaningful
forces.

## How it works now

```
CoasterSpline.nlelem  ->  convert_nlelem_to_ue.py  ->  coaster_ue5_bundle.json
   your car mesh ---------------^                    ->  CoasterCarAnimated.fbx
                                                     ->  coaster_timeline.csv
                                                                |
                                                      unreal_import_coaster.py
                                                      (run inside UE Editor)
                                                                |
                                        track spline + coaster car + Level Sequence
```

**Blender is not part of the path.** The bundle is already in Unreal's
coordinate system and units, so Unreal reads it straight in, and the animated
FBX is written directly in Python. That removes every failure mode that came
from round-tripping through Blender:

- no dependency on Blender's scene frame rate, which silently timed keys at 24fps
- no dependency on Blender's 3DS importer, which was not available here anyway
- no FBX unit negotiation: the file declares Z-up centimetres, matching Unreal

The old Blender scripts remain on disk but nothing calls them. See "Legacy FBX
scripts" below.

## Scale

Scale is exact by construction rather than by convention:

- Source data is metres. `ue_pos_cm` is centimetres. **1 Unreal unit = 1 cm.**
- The conversion factor is recorded in the bundle as
  `units.metres_to_unreal_units` and `unreal_import_coaster.py` refuses to
  import if it is not 100.
- On import, the script logs the track's extent in metres so it can be checked
  against the source design at a glance.

For this track that is **260.44 m x 163.98 m x 44.65 m** (X x Y x height).

### Axis mapping

The correct mapping was established empirically, not assumed. Pass
`--validate-reference-csv` with a known-good UE-space spline export and the
converter scores every candidate mapping against it:

```
mapping                 det     span err          RMS  len ratio
nl2_to_ue_swap_yz        -1      0.00 cm    488.05 cm    0.98075
identity                 +1  11939.02 cm   4193.54 cm    0.98075
nl2_to_ue                +1   9645.09 cm   4652.23 cm    0.98075
nl2_to_ue_flip_y         -1   9645.09 cm   7206.19 cm    0.98075
```

`nl2_to_ue_swap_yz` — `(x, y, z) -> (x, z, y)` — is the default. It matches the
reference to **0.00 cm** on all three axis spans, and it is the only default
with determinant **-1**.

The determinant matters: the source is right-handed and Unreal is left-handed,
so a correct conversion must have determinant -1. A mapping with +1 is a pure
rotation, cannot change handedness, and silently yields a **mirrored** track —
left-hand helices become right-hand and every lateral-G sign flips.

The previous default, `nl2_to_ue`, has determinant +1 and was mirrored.

## Physics

### Speed

There is no velocity data in the `.nlelem` export, so speed comes from an energy
model: gravity, rolling friction, and drag. Tune with `--initial-speed`,
`--rolling-friction`, `--drag-coeff`. **The speed profile is only as good as
those three numbers** — if you can export real velocities from OpenFVD, they would
replace this entirely and should.

### Curvature and force

Three things make the force numbers meaningful:

1. **Uniform arc-length resampling** (`--resample-spacing-m`, default 0.10).
   Bezier segments are sampled in uniform parameter `t`, which produces spacing
   that varies by more than 10x. Differentiating unevenly spaced points biases
   the result by the spacing itself.

2. **Circumcircle curvature** rather than `|d(tangent)|/ds`, which divides by a
   length that can approach zero.

3. **A time-matched measurement window** (`--curvature-window-s`, default 0.15).
   Accelerometers, and CoasterAnalyzer, filter over *time*, so the curvature
   baseline is `speed * window` rather than a fixed distance. This makes the
   converter and the in-engine analyzer measure the same physical scale.

The spike filter now runs **only** on the render path. It rewrites positions, and
every position edit changes curvature — a straight-line replacement reads as
zero curvature (phantom airtime) bracketed by kinks (phantom spikes). The
analytic path in `samples` is never geometry-edited. `render_path` is the
filtered copy, and is what the track spline is built from.

## Source data defects in this export

The converter reports defects rather than smoothing them away, because smoothing
geometry to make forces look reasonable is how you get plausible wrong answers.

This export has real damage, confirmed independently by `Coaster_UE_spline.csv`
having discontinuities in the same places:

- **4 gaps / missing track**: nodes 16 (45.95 m), 316 (10.01 m), 333 (25.85 m),
  341 (13.36 m), against a median node spacing of 2.00 m.
- **12 malformed segments** whose Bezier control polygon folds back on itself,
  clustered around those same gaps. Segment 14's polygon is 8.71x its chord with
  a 180-degree internal turn. A fold means a cusp: the tangent passes through
  zero and curvature genuinely diverges, so no estimator can produce a sensible
  force there.

Samples in these regions are flagged `"suspect": true` in the bundle, and the
converter reports the force envelope both including and excluding them:

```
Normal G, all samples      : median 1.21  p95 5.50  peak 188.68
Normal G, excl. 1733 suspect: median 1.65  p95 5.49  peak 7.24
```

**21.3% of samples are flagged.** The clean figures are plausible for an
aggressive design; the 188 G peak is an artefact of the damaged geometry. Fixing
this properly means re-exporting from OpenFVD / NoLimits.

## The coaster car

Pick any mesh you already have. Set it on the **Car** tab, or from the command
line:

```powershell
--car-mesh-file  "C:/path/to/CoasterCar.glb"    # imported into Unreal on first use
--car-mesh-asset "/Game/Coaster/SM_CoasterCar"    # or an asset already in the project
```

The mesh is **copied into the output folder** next to the bundle, so a finished
export is self-contained:

```
CoasterRawExportData/UE5/
  coaster_ue5_bundle.json     analytic path, timeline, forces, car settings
  coaster_timeline.csv        the same timeline as a table
  CoasterCarAnimated.fbx      the car's motion as real keyframes
  KexLSMfoSketchfab.glb       staged car mesh
```

The bundle records only the filename, resolved next to itself, so moving or
sending the whole folder keeps working. The original path is kept as a fallback
in case the staged copy is deleted, and a bundle written before staging existed
still imports from its absolute path.

`.glb` embeds its textures. `.fbx` and `.obj` can reference external ones, and
the converter says so when you use them; an `.obj`'s sibling `.mtl` is copied
automatically.

The Unreal side then needs nothing but the bundle path.
`unreal_import_coaster.py`:

1. loads the asset, or imports the staged file if the asset is not there yet
2. spawns it as `CoasterCar` and applies the facing correction, height offset
   and scale
3. keys its transform along the whole ride in the Level Sequence
4. reports the mesh's real size, and checks it against
   `--car-expected-length-m` if you set one

The car is presentation only. Nothing on that tab changes a single number in
the physics timeline.

### There is no import-folder setting

Unreal assets live inside the project's `Content` directory, addressed as
`/Game/...`. That is a content-browser path, not a disk folder, so a car asset
cannot live in `CoasterRawExportData\UE5` alongside the bundle.

Rather than expose a setting for it, the importer derives the content folder
from the level sequence's own folder, so **everything it creates lands in one
place**: `/Game/Coaster/LS_CoasterRide` puts the car in `/Game/Coaster`. Point
`level_sequence_path` somewhere else and the car follows.

Re-running the import reuses an already-imported mesh rather than piling up
`SM_Car`, `SM_Car_1`, `SM_Car_2`.

### Facing axis

The path frame puts travel along **+X**. A mesh authored facing +Y will drive
sideways until **Faces along** is set to match, which is the one setting worth
checking first if the car looks wrong. `Extra yaw` handles a mesh that is not
quite square to its own axes.

### Why the orientation lives on the component

The facing correction, offset and scale are applied to the mesh *component*,
not the actor. The actor's own transform stays the pure path frame, so the
Level Sequence keys remain physically meaningful and CoasterAnalyzer reads
exactly the motion the converter computed. Baking the correction into the
actor transform would have quietly corrupted every force reading.

### Scale

Scale defaults to **1.0**, which keeps the mesh at its authored real-world
size. That is deliberate: the previous pipeline forced every car to a fixed
nominal length, so a correctly-built 4.5 m car arrived as 2.25 m next to a
true-scale track. Set `--car-expected-length-m` and the import will tell you
the measured length and the exact scale factor needed to correct it.

### No car set

A placeholder box roughly the size of a coaster car (4.5 x 1.6 x 1.2 m) is
animated instead, so the motion is still visible and checkable. The import logs
a warning saying so.

### The animation is a real file

`CoasterCarAnimated.fbx` carries the car's motion as keyframes, so the export
contains the animation instead of depending on a script to rebuild it. It is a
**binary FBX 7.4** written directly by `export_car_animation.py` - no Blender,
no FBX SDK.

Binary rather than ASCII on purpose: most readers, Blender included, refuse
ASCII FBX outright, so an ASCII file could not even be verified.

The scene declares itself Z-up in centimetres, which is Unreal's own
convention, so the importer has no axis or unit conversion to perform.

| Property | Value |
|---|---|
| Node name | `CoasterCar` |
| Channels | `Lcl Translation` and `Lcl Rotation`, XYZ each |
| Rate | `--car-fbx-fps`, default 60 |
| Axes / units | Z-up, X-forward, centimetres, `UnitScaleFactor` 1 |

Two details that are easy to get wrong and are handled:

- **Euler order.** FBX's default `eEulerXYZ` composes as `Rz * Ry * Rx`, not the
  more familiar `Rx * Ry * Rz`. Decomposing for the wrong order yields a car
  that is subtly mis-banked everywhere rather than obviously broken.
- **Angle unwrapping.** Each rotation channel is unwrapped, so a roll sweeping
  past +-180 degrees does not read as an instant spin backwards. Left alone this
  shows as a visible snap, and makes anything differentiating the curve report a
  huge false angular velocity.

### Verifying it

The FBX is hand-written, so it is checked rather than trusted:

```powershell
blender -b -P verify_car_animation.py -- "...\CoasterCarAnimated.fbx" "...\coaster_ue5_bundle.json" 60
```

Blender is used only as an independent reader here. The check compares position
against the timeline and confirms the car's local +X still points along travel
and its local +Z along the banked up vector, which is what catches a wrong Euler
order or a mirrored axis. Current result on this export:

```
position error cm : min 0.000  median 0.000  max 0.002
forward axis dot  : min +1.00000  median +1.00000  max +1.00000
up axis dot       : min +0.99995  median +1.00000  max +1.00000
RESULT: PASS
```

`fbx_inspect.py` dumps any binary FBX's record tree if you need to look inside
one:

```powershell
python fbx_inspect.py CoasterCarAnimated.fbx AnimationCurve
```

### FBX or bundle?

Both describe the same motion; use whichever suits the job.

| | FBX | Bundle + `unreal_import_coaster.py` |
|---|---|---|
| Portable outside Unreal | yes | no |
| Carries your car mesh | no, a car-sized box | yes, the staged mesh |
| Time resolution | quantised to the chosen fps | every analytic sample |
| Builds the track spline | no | yes |
| Reports forces on import | no | yes |

The bundle route stays the higher-fidelity one, because its keys sit on the
analytic samples rather than on a fixed frame clock. The FBX is what makes the
export self-contained.

### Reading forces off the car

Add a **CoasterAnalyzer** component to the `CoasterCar` actor with **Use Live
Actor Tracking** enabled, then press Play. The import prints this reminder.

## Run it

### GUI

```powershell
python "C:\Users\rhutto2\Documents\TestCoaster\UE5_CoasterPipeline\coaster_pipeline_gui.py"
```

Four tabs, LSU purple and gold, dark throughout:

| Tab | Holds |
|---|---|
| **Files** | Spline (required), tangent, output folder, validation reference CSV |
| **Car** | Car mesh, facing axis, yaw, scale, height offset, expected length, baked FBX |
| **Physics** | Initial speed, rolling friction, drag, curvature window and floor |
| **Geometry** | Axis mapping, samples per segment, resample spacing, track mesh cleanup |
| **Advanced** | Python interpreter, source-defect thresholds, spike filter tuning |

Behaviour worth knowing:

- Picking a determinant +1 axis mapping prompts for confirmation, explaining
  that the track will come out mirrored.
- The run log colour-codes warnings and errors, so a flagged source defect is
  visible without reading every line.
- The status pill reads Ready / Running / Done / Failed.
- Ctrl+Enter runs. Every field carries a one-line explanation of what it does.

Settings saved by an older build are migrated on load and each change is
reported in the run log. Removed entirely, because they only existed to drive
the Blender/FBX export:

`mesh_3ds`, `blender_exe`, `cart_model_glb`, `run_fbx_conversion`,
`export_animated_fbx`, `animated_fbx_fps`, `animated_fbx_speed_multiplier`,
`animated_fbx_scale_multiplier`, `animated_fbx_cart_scale`,
`animated_fbx_cart_fit_mode`, `animated_fbx_cart_target_length`,
`animated_fbx_calibration_cube_m`, `animated_fbx_root_rot_x_deg`,
`physics_accurate_mode`.

`physics_accurate_mode` went with them: it existed only to force the FBX time
and scale multipliers to 1.0 and the bake rate to 120fps. With no bake in the
pipeline there is nothing left for it to force, and no way to express those
errors in the first place.

Two corrections are applied to old saved state:

- `axis_mapping` on any determinant +1 value is reset to `nl2_to_ue_swap_yz`.
  The previous value mirrored the track.
- `python_exe` is reset if it does not name a Python binary. It had come to
  point at the packaged GUI executable, which would have launched a second app
  window instead of converting.

### Command line

```powershell
python convert_nlelem_to_ue.py `
  --spline "...\CoasterSpline.nlelem" `
  --tangent "...\CoasterTangent.nlelem" `
  --output "...\UE5\coaster_ue5_bundle.json" `
  --csv "...\UE5\coaster_timeline.csv" `
  --initial-speed 10.0 `
  --validate-reference-csv "...\Coaster_UE_spline.csv"
```

### Unreal

```python
import unreal
exec(open(r"C:/Users/rhutto2/Documents/TestCoaster/UE5_CoasterPipeline/unreal_import_coaster.py").read())
import_coaster_bundle(
    bundle_path=r"C:/Users/rhutto2/Documents/TestCoaster/CoasterRawExportData/UE5/coaster_ue5_bundle.json",
    spline_actor_name="BP_CoasterSplineActor",
    actor_to_animate_name="CineCameraActor_0",
    level_sequence_path="/Game/Coaster/LS_CoasterRide",
)
```

This builds the track spline from `render_path`, spawns the coaster car, and
keys a Level Sequence from `samples`, then logs the extent, duration, car size
and force envelope.

To try a different car, or to fly a camera along the same path, without
re-running the converter:

```python
import_coaster_bundle(
    bundle_path=r".../coaster_ue5_bundle.json",
    car_mesh_asset="/Game/Coaster/SM_MyOtherCar",
    also_animate_actor_name="CineCameraActor_0",
)
```

`also_animate_actor_name` is optional and a missing actor is reported and
skipped. Previously a missing actor raised, which meant the default value of
`"CineCameraActor_0"` aborted the entire import for anyone without a camera by
that exact name.

Two bugs fixed here that mattered:

- Orientation was built from **source-space** `tan`/`up` while position used
  **Unreal-space** `ue_pos_cm`, so the cart faced the wrong way relative to the
  path it followed. The bundle now carries `ue_tan` / `ue_up` and the script
  uses them.
- Key times were written as `t * 30.0` regardless of the sequence's actual tick
  resolution, quantising the whole ride to 30 ticks per second. The script now
  reads `get_tick_resolution()` and keys against it.

## Legacy FBX scripts (not called by the GUI)

`blender_build_animated_fbx.py`, `blender_3ds_to_fbx.py` and
`verify_baked_physics.py` are no longer referenced by the pipeline and are
no longer bundled into the packaged executable. They are kept only so the
FBX route can be reconstructed by hand if it is ever needed.

`verify_baked_physics.py` stays useful in that case. `verify_baked_physics.py` reads an
exported FBX back, differentiates it with the same Savitzky-Golay filter
CoasterAnalyzer uses, and compares against the analytic timeline:

```powershell
blender -b -P verify_baked_physics.py -- --fbx "...\CoasterCartAnimated.fbx" --bundle "...\coaster_ue5_bundle.json" --fps 120
```

It reports speed error, acceleration error, and peak-G retention, skipping the
flagged defect regions. It found the bug where `scene.render.fps` was never set,
so Blender timed the keys at its default 24fps — at `--fps 120` that stretched a
101 s ride to 506 s and scaled every acceleration by 1/25.

Acceleration is compared between two deliberately different estimators (the
converter's circumcircle vs the analyzer's SG differentiator), so a p95 gap
around 20% is inherent. Peak retention is the tight gate.

## Files

| File | Role |
|---|---|
| `convert_nlelem_to_ue.py` | `.nlelem` -> JSON bundle. The core tool. |
| `unreal_import_coaster.py` | Run in UE Editor. Builds spline + Level Sequence. |
| `coaster_pipeline_gui.py` | GUI front end. |
| `export_car_animation.py` | Writes CoasterCarAnimated.fbx. Pure Python. |
| `verify_car_animation.py` | Checks that FBX against the bundle (needs Blender). |
| `fbx_inspect.py` | Dumps a binary FBX record tree. Pure Python. |
| `verify_baked_physics.py` | Legacy. FBX fidelity check (needs Blender). |
| `blender_build_animated_fbx.py` | Legacy. FBX export (needs Blender). |
| `blender_3ds_to_fbx.py` | Legacy. 3DS to FBX (needs Blender). |
| `*.bak` | Backups of the pre-change versions. |
