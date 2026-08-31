# UE5 Coaster Pipeline From OpenFVD / NoLimits 2 Export

Converts exported OpenFVD or NoLimits 2 track data into a JSON bundle that
Unreal drives directly, with correct real-world scale and physically meaningful
forces.

## How it works now

```
 OpenFVD .nlelem  \
 NoLimits .nl2elem >->  convert_nlelem_to_ue.py  ->  coaster_ue5_bundle.json
 NoLimits .csv    /            ^                  ->  CoasterCarAnimated.glb
   your car mesh --------------'                  ->  CoasterCarAnimated.fbx
                                                  ->  CoasterTrack.fbx
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

## Input formats

Three, auto-detected from the file extension. Pick one; the others are not
needed.

| File | From | Tangent file needed | Accuracy |
|---|---|---|---|
| **`.csv`** spline export | NoLimits 2 | no | **exact** |
| `.nl2elem` | NoLimits 2 | no | exact via its sibling CSV |
| `.nlelem` | OpenFVD | yes, for banking | exact |

### NoLimits 2 spline CSV - the best input

Tab-separated, one row per station, with a position **and a complete
orientation frame** per row:

```
"No."  "PosX" "PosY" "PosZ"  "FrontX".. "LeftX".. "UpX"..
```

Nothing is reconstructed. Positions come straight from the file and the up
vector is authored rather than rebuilt from a roll angle, which is why the
exported animation validates a shade tighter than the OpenFVD route: up-axis dot
**min +1.00000** against +0.99995.

Metres, Y up - the same convention as the element formats - so the same verified
axis mapping applies. `Left = Up x Front` was confirmed on the sample.

### NoLimits 2 `.nl2elem`

XML, and self-contained in the sense you would expect: one file holds both the
path (`<vertex>`) and the banking (`<roll>`, an orthonormal up/right pair at a
normalised `coord`). No tangent file.

**Point the converter at it and it works** - because it looks for the spline CSV
that NoLimits exports alongside and uses that instead. The element's own
`<description>` is checked first, since NoLimits names the CSV after the track
rather than after the element file: `Coaster.nl2elem` with description
`IndoorCoaster` finds `IndoorCoasterSpline.csv`.

```
Read Coaster.nl2elem -> using its spline export IndoorCoasterSpline.csv
```

#### Why the CSV is preferred, in detail

The `.nl2elem` geometry itself could not be decoded reliably, and the reader
says so rather than pretending otherwise. What was established from the sample:

- The vertex count is an exact multiple of three (57 = 19 x 3).
- After an X-Z swap the vertex bounds **contain** the resolved path's on every
  axis (76 >= 75, 68 >= 41, 149 >= 140), and the vertex polyline is longer than
  the path (1561 m against 1223 m). Both are the signature of control points
  enclosing their curve, so the vertices are controls, not points on the track.
- The roll frames are orthonormal, and `u` and `r` both sit ~90 degrees from the
  tangent, which identifies `u x r` as the travel direction. At `coord` 0 and 1
  that reproduces the tangent to **0.1 degrees**.

What could not be established is the spline basis. Read as cubic Bezier triples
- the layout the older binary format uses - the chain is **not C1 continuous**:
corners of 72 to 145 degrees at the joints, where a real track has none. No
offset or ordering tested fixes it, and only 1 of 55 consecutive vertex triples
is even near-collinear. An earlier fit that looked promising (median 10.8
degrees tangent error, against 27 to 49 for the alternatives) turned out to be
coincidence, not structure.

So with no CSV beside it the reader still runs, but it warns loudly, and the
defect detection flags the reconstructed path as unusable on its own terms -
every sample comes back `"suspect": true`. `--ignore-sibling-csv` forces that
path for diagnosis. Do not ship from it.

### The reference CSV is not one of these

`--validate-reference-csv` is an optional cross-check, and it wants something
different from all three inputs above: a spline exported **from Unreal, in
centimetres**, of the same ride. It is what pinned the axis mapping down.

Pointing it at a NoLimits source export is the easy mistake, since that is also
a CSV with `PosX` columns. The converter now detects that and says so, rather
than reporting a mapping failure:

```
NOTE: skipping axis validation. IndoorCoasterSpline.csv is a NoLimits 2 source
export, not an Unreal-space reference. It is the converter's input, in metres,
so comparing the conversion against it proves nothing.
```

It also refuses a reference whose extent looks like metres rather than
centimetres, and checks that the reference is plausibly the same ride at all -
a reference from a different track used to read as "your axis mapping is wrong",
which would have led you to mirror a correct track.

Validation is advisory throughout: a bad reference is reported and skipped, and
never costs you the export.

### OpenFVD `.nlelem`

The original binary format. Needs the companion tangent file for banking; see
"Source data defects" below for what this particular export contains.

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

Neither the `.nlelem` export nor the NoLimits spline CSV carries velocity - the
CSV is `PosX/Y/Z` plus the Front/Left/Up basis and nothing else - so speed comes
from an energy model: gravity, rolling friction, and drag. Tune with
`--initial-speed`, `--rolling-friction`, `--drag-coeff`. **The speed profile is
only as good as those three numbers.** If you can export real velocities from
either tool, they would replace this entirely and should.

#### The lift hill is driven, not coasted

Gravity cannot get a train up a lift, so a purely gravity-driven model has to
catch the climb with a floor, and the train crawls. That was the single largest
error in the ride's timing - on the reference ride it cost 19.5 s, 22% of the
whole ride, spent at 1 m/s.

So the model drives it. Wherever **the track is climbing and free-rolling would
be slower than `--lift-speed`** (default 4.0 m/s), the train is taken to be on
the chain or LSM and holds that speed exactly, which is what a real lift does.
Everywhere else it rolls on energy alone. Longitudinal acceleration goes to zero
across a driven section, which is also correct - a chain lift is constant speed.

The converter prints what it decided, so the driven sections are inspectable
rather than silently changing the ride time:

```
Duration 73.46s, length 1222.9m, speed 21.0/28.4 m/s (median/peak)
Lift: 1 driven section(s) at 4.0 m/s, 5.1s of the ride (7%)
      657-677 m:   5.1s, climbing +14.9 m
```

Set `--lift-speed` to your lift's real speed if you know it. `--lift-speed 0`
models the ride as purely gravity-driven, which is physically honest but will
make every climb crawl. `--min-speed` is now only a numerical floor for flat,
undriven stretches - it is no longer what carries the train uphill.

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
  CoasterCarAnimated.glb      the car + its motion; drop this into Unreal
  CoasterCarAnimated.fbx      the same motion for other tools
  CoasterTrack.glb            the track; drop this into Unreal too
  CoasterTrack.fbx            procedural rails, spine, ties and supports
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

`auto` measures the mesh, and has to answer two questions. The long horizontal
axis is easy. Which *end* of it is the nose is not - a bounding box cannot tell
you, and guessing wrong drives the car round the track backwards while every
numeric check still passes, because the path and the banking are both still
correct. So `auto` also compares the two halves: a coaster car's nose is low and
tapered while its back carries the seat backs, so the **taller half is the
rear**. The converter prints what it decided (`facing -Y`); if it gets your car
wrong, name the axis explicitly and it will not second-guess you.

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

### The animated FBX carries your actual car

`CoasterCarAnimated.fbx` is a **single-bone skeletal mesh of the car you
selected**, with the ride baked onto that bone as keyframes. The car's triangles
are read straight out of the `.glb` by `read_glb.py` and written into the FBX, so
the file is self-contained - it does not reference the GLB.

For the reference car that is 56,052 triangles at 5.21 x 2.28 x 2.83 m, and the
mesh is auto-rotated so the car faces along travel (`--car-forward-axis auto`
measures the bounds; pass `+X`/`+Y`/`-X`/`-Y` to override).

**If no car mesh is given**, a box roughly the size of a coaster car is skinned
and animated instead, so the motion is still visible and checkable. The import
logs a warning saying so.

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

### Verified against Unreal, not just Blender

Blender is a convenient independent reader but a forgiving one. Autodesk's FBX
SDK - which Unreal's Interchange importer uses - is stricter, and rejected files
Blender read without complaint:

```
Cannot open FBX file 'CoasterTrack.fbx'.
FInterchangeFbxParser::LoadFbxFile: ... error when parsing the file.
There was nothing to import from the provided source data.
```

The cause was the document preamble, all of which the SDK requires and Blender
ignores: missing `FileId`, `CreationTime`, `CreationTimeStamp` and `SceneInfo`,
an unnamed scene `Document`, and `RootNode` written as an int32 where it must be
an int64.

`verify_ue_import.py` now checks the exports against the real importer, headless
and without saving anything:

```powershell
UnrealEditor-Cmd.exe "YourProject.uproject" -ExecutePythonScript="verify_ue_import.py" -unattended -nopause -nosplash -stdout
```

Current result on this export - four separate meshes, correct real-world bounds,
no warnings:

```
RESULT CoasterTrack.fbx: 4 object(s) imported
  StaticMesh: CoasterTrackRails     bounds 142.1 x 76.2 x 41.2 m
  StaticMesh: CoasterTrackSpine     bounds 142.0 x 75.3 x 41.3 m
  StaticMesh: CoasterTrackTies      bounds 142.3 x 76.2 x 41.4 m
  StaticMesh: CoasterTrackSupports  bounds 141.5 x 75.3 x 40.6 m
RESULT CoasterCarAnimated.fbx: 3 object(s) imported
  SkeletalMesh: CoasterCarAnimated   bounds 5.21 x 2.28 x 2.83 m
  Skeleton:     CoasterCarAnimated_Skeleton
  PhysicsAsset: CoasterCarAnimated_PhysicsAsset
```

Unreal also warned that the meshes carried no smoothing groups. Harmless in
itself, but it meant Unreal might compute normals rather than use the supplied
ones - which would have rounded the crossties, whose corner vertices were
shared. Box faces are now built independently so they read flat either way, and
the warning is gone.

### Import the two .glb files, not the .fbx files

`CoasterCarAnimated.glb` and `CoasterTrack.glb` are written in the same space and
land on top of each other with identity transforms. Place both actors at the
world origin and the car runs on the track - measured at **13.6 cm median**
between the car and the track centre line, which is just the sampling step.

**Do not import CoasterTrack.fbx into Unreal.** Unreal's FBX importer mirrors
the scene in Y. The track comes in as a mirror image of itself, which lines up
with the glTF car nowhere - the two end up hundreds of metres apart. Worse, a
mirror preserves every length, so the track passes every span and bounds check
while being wrong; that is how it survived earlier verification. The `.fbx`
files stay in the export for tools that read FBX correctly.

### Just drag CoasterCarAnimated.glb into Unreal

That single file gives you the car and the whole ride as an `AnimSequence`, with
no script, no import settings and no project changes:

| Asset | Type |
|---|---|
| `CoasterCarAnimated` | SkeletalMesh - your car, 5.21 x 2.28 x 2.83 m |
| `CoasterCarAnimated_Skeleton` | Skeleton - `CoasterCarRig` -> `CoasterCarBone` |
| `CoasterCarAnimated_Anim` | **AnimSequence** - the ride, 89.83 s |
| `CoasterCarAnimated_PhysicsAsset` | PhysicsAsset |
| `CoasterCarMaterial` | Material |

Measured against the analytic path over 101 probes:

| | median | p95 | max |
|---|---|---|---|
| position | 0.067 cm | 0.400 cm | 0.809 cm |
| facing | 0.052 deg | 0.635 deg | 1.107 deg |
| bank | 0.049 deg | 0.402 deg | 1.091 deg |

The residual is Unreal resampling the 60 fps keys down to the project's 30 fps
animation rate, not error in the export.

**Why glTF and not FBX.** Unreal's FBX translator would not carry the animation
across. With a correct skeleton, correct bind pose, Autodesk's own tangent
encoding and a frame-aligned take, the legacy FBX importer does now build an
`AnimSequence` from `CoasterCarAnimated.fbx` - but every key reads back as
identity, and Interchange (the default in 5.8) builds no `AnimSequence` at all.
The animation graph in that file is byte-for-byte equivalent to one Unreal
exports itself, so the remaining difference is somewhere the records do not
show. glTF has no such problem: Interchange reads skinned glTF animation
natively, and the format is plain JSON plus typed buffers, so the file can be
checked before it ever reaches the engine.

Two things that are not optional in either format:

- **The take must be frame-border aligned.** Unreal rejects any animation whose
  length is not a whole number of frames at the project's animation rate. The
  legacy importer says so - *"animation has to be frame-border aligned"* -
  Interchange just drops the take silently. `--car-fbx-import-fps` (default 30)
  is the rate to align to; set it to your project's Default Frame Rate if you
  have changed it.
- **The bind pose must agree with the skin.** Seeding the bone with the first
  frame while the cluster matrices stay identity displaces the whole animation
  by that offset - 55 m, in the reference ride.

### What the import script gives you

Measured, not assumed. Importing the bundle produces all of this:

| Asset | Type | What it is |
|---|---|---|
| `CoasterCarAnimated` | SkeletalMesh | your car, one bone, 5.21 x 2.28 x 2.83 m |
| `CoasterCarAnimated_Skeleton` | Skeleton | `CoasterCarRig` -> `CoasterCarBone` |
| `CoasterCarAnim` | **AnimSequence** | the whole ride baked onto the bone |
| `CoasterTrackRails` etc. | StaticMesh x4 | the procedural track |
| `LS_CoasterRide` | LevelSequence | the same ride driving a placed actor |
| `CoasterCar` | StaticMesh | the textured GLB import |

The `AnimSequence` is **built by `unreal_import_coaster.py`, not read out of the
FBX.** Unreal's FBX translator would not produce one from the exported file, and
building it through `unreal.AnimationDataController` is both reliable and
checkable, since the keys come straight from the bundle. Verified against the
analytic path over 41 probes: **median 0.009 cm, p95 0.397 cm, max 0.686 cm.**

Two details there were not optional:

- **Every bone in the chain gets a track**, not just the driven one. A bone with
  no keys is walked during compression anyway.
- **Keys are written at the project's animation frame rate**, read from
  `AnimationSettings.default_frame_rate` (30 fps by default), and the frame count
  is floored so the play length is an exact whole number of frames. Anything else
  is resampled to that rate, and a play length that is not frame-aligned at it
  fails `check(IsNearlyZero(SampleFrameTime.GetSubFrame()))` in
  `AnimCompressionTypes.cpp` - which crashes the editor on save, on a background
  worker, several seconds after the import reports success. Writing at a higher
  rate buys nothing, because the resample discards the extra keys.

Both the `AnimSequence` and the Level Sequence describe the same motion; the
Level Sequence keys every analytic sample rather than a fixed frame clock, so it
stays the higher-resolution of the two.

### FBX or bundle?

Both describe the same motion; use whichever suits the job.

| | FBX | Bundle + `unreal_import_coaster.py` |
|---|---|---|
| Portable outside Unreal | yes | no |
| Carries your car mesh | yes, embedded triangles | yes, the staged mesh |
| Yields an AnimSequence | .glb yes, .fbx no | yes |
| Time resolution | quantised to the chosen fps | every analytic sample |
| Builds the track spline | no | yes |
| Reports forces on import | no | yes |

The bundle route stays the higher-fidelity one, because its keys sit on the
analytic samples rather than on a fixed frame clock. The FBX is what makes the
export self-contained.

### Reading forces off the car

Add a **CoasterAnalyzer** component to the `CoasterCar` actor with **Use Live
Actor Tracking** enabled, then press Play. The import prints this reminder.

## The track mesh

`CoasterTrack.fbx` is generated track geometry swept along the converted path -
not a rip of the original model, which the `.nlelem` export does not contain.
Written by `export_track_mesh.py` through the same binary FBX writer as the
animation.

Four separate meshes, so each can take its own material in Unreal instead of
arriving as one unassignable blob:

| Mesh | What it is |
|---|---|
| `CoasterTrackRails` | The two running rails, swept tubes at the gauge |
| `CoasterTrackSpine` | The central box beam below the rails |
| `CoasterTrackTies` | Crossties from the spine out to both rails |
| `CoasterTrackSupports` | Vertical columns down to ground level |

For this ride that is 2,017 cross-sections over 806 m: **55,124 polygons**,
57,848 vertices, 505 ties and 85 columns, in an 11 MB file.

Every part carries smooth per-corner normals and a UV layer, so the rails shade
as round tubes rather than faceted prisms and textures tile at a real-world rate
(V advances one unit per metre of track).

### The track is fitted to your car

`--track-rail-drop-cm` and `--track-gauge-cm` both default to `auto`, which
measures the car's bogies rather than assuming a value.

A bogie grips the rail from both sides - road wheels on top, upstop wheels
underneath - so the outboard running gear is spread roughly symmetrically about
the rail line. Its vertical centre gives the rail height, and whatever sits at
that height gives the gauge. The converter reports what it found:

```
track fit: rails -1.6 cm from the path, gauge 174.0 cm (measured from the car's bogies)
```

The old fixed defaults (110 cm drop, 100 cm gauge) assumed the path was a
heartline and the car was narrow-gauge. For the reference car both are wrong: its
rails belong at the path, 174 cm apart. The result was a car hovering above rails
that were too low and too narrow - and nothing caught it, because the track and
the animation were each individually correct and only disagreed with each other.

Pass a number to either flag to override the measurement.

### Where it sits

The geometry is written in **absolute Unreal coordinates**, the same ones the
car animation uses, so the track needs no alignment: `unreal_import_coaster.py`
imports it and places each part at the origin with an identity transform, and
the car runs on it. Verified by comparing the mesh bounds against the path:

```
axis    mesh span m   path span m  difference m
X            260.49        260.44          0.05
Y            164.59        163.98          0.61
Z             45.17         44.65          0.52
```

X matches to 5 cm. Y is wider by the rail gauge, and Z is taller because the
rails sit *above* the path through inversions and the columns run below it -
both expected.

### What the geometry assumes

The path is treated as the **heartline**, which is what NoLimits and OpenFVD
export, so the rails are placed below it by `--track-rail-drop-cm` (default
110 cm) and the spine 35 cm below that. Adjust the drop and
`--track-gauge-cm` to match the coaster type you are modelling.

Support columns scale their radius with height, targeting a height-to-diameter
ratio near 35. A fixed radius made a 44 m column read as a wire at ratio 160,
where real coaster columns sit nearer 35.

### What it is not

The supports are plain verticals dropped to the lowest point of the ride. Real
supports are angled bents chosen per element, so treat these as massing and
shadow rather than an engineering claim - they can intersect track that passes
underneath. Rails are round tubes; box-section and triangular-truss spines are
not modelled.

### Verifying it

```powershell
blender -b -P verify_track_mesh.py -- "...\CoasterTrack.fbx" "...\coaster_ue5_bundle.json"
```

Confirms every part imported, that the bounds match the path, that no vertex is
non-finite, and that normals and UVs survived. Blender is used only as an
independent reader.

## Run it

### GUI

```powershell
python "C:\Users\rhutto2\Documents\TestCoaster\UE5_CoasterPipeline\coaster_pipeline_gui.py"
```

Four tabs, LSU purple and gold, dark throughout:

| Tab | Holds |
|---|---|
| **Files** | Track file (required), tangent, output folder, validation reference CSV |
| **Car** | Car mesh, facing axis, yaw, scale, height offset, expected length, baked FBX |
| **Track** | Procedural track: gauge, rail drop, detail and tie spacing, supports |
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
| `fbx_writer.py` | Shared binary FBX 7.4 writer. Pure Python. |
| `export_car_glb.py` | Writes CoasterCarAnimated.glb - the file Unreal imports as an animation. Pure Python. |
| `export_car_animation.py` | Writes CoasterCarAnimated.fbx. Pure Python. |
| `export_track_mesh.py` | Writes CoasterTrack.fbx. Pure Python. |
| `verify_track_mesh.py` | Checks that track against the bundle (needs Blender). |
| `verify_car_animation.py` | Checks that FBX against the bundle (needs Blender). |
| `fbx_inspect.py` | Dumps a binary FBX record tree. Pure Python. |
| `verify_baked_physics.py` | Legacy. FBX fidelity check (needs Blender). |
| `blender_build_animated_fbx.py` | Legacy. FBX export (needs Blender). |
| `blender_3ds_to_fbx.py` | Legacy. 3DS to FBX (needs Blender). |
| `*.bak` | Backups of the pre-change versions. |
