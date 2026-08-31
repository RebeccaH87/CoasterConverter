"""
Write the baked car motion as a glTF 2.0 binary (.glb).

This exists because Unreal's FBX path would not carry the animation across.
Interchange reads skinned glTF animation natively, so this is the file to drag
into the content browser when you want the ride as an AnimSequence without
running the import script.

glTF is right-handed, Y-up, in metres. Everything upstream of here is in
Unreal's left-handed Z-up centimetres, so positions and directions swap Y and Z
on the way out - the same swap read_glb.py applies coming in, which is its own
inverse. The swap has determinant -1, so it also reverses triangle winding and
turns cross products around; both are handled below rather than left to chance.
"""

from __future__ import annotations

import json
import math
import struct
from pathlib import Path
from typing import Dict, List, Optional, Sequence

from export_car_animation import resample_uniform_fps, trim_to_frame_border
from read_glb import GlbError, orient_forward, read_glb_triangles

CM_TO_M = 0.01

# accessor.componentType
FLOAT = 5126
UNSIGNED_INT = 5125
UNSIGNED_BYTE = 5121

# bufferView.target
ARRAY_BUFFER = 34962
ELEMENT_ARRAY_BUFFER = 34963

GLB_MAGIC = 0x46546C67
CHUNK_JSON = 0x4E4F534A
CHUNK_BIN = 0x004E4942


def _swap(v: Sequence[float]) -> List[float]:
    """Unreal (x, y, z) -> glTF (x, z, y). Its own inverse."""
    return [v[0], v[2], v[1]]


def _norm(v: Sequence[float]) -> List[float]:
    length = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    return [v[0] / length, v[1] / length, v[2] / length] if length > 1e-12 else [1.0, 0.0, 0.0]


def _cross(a: Sequence[float], b: Sequence[float]) -> List[float]:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def quat_from_tangent_up(tangent: Sequence[float], up: Sequence[float]) -> List[float]:
    """Orientation as a glTF quaternion (x, y, z, w).

    The car's local axes in Unreal are X along travel and Z along the banked up.
    Swapping into glTF puts travel on +X and up on +Y; the third column is
    forward x up rather than up x forward, because the swap reverses handedness.
    """
    # Straight from the source vectors rather than via basis_from_tangent_up,
    # which returns a matrix whose *columns* are the axes - unpacking it gives
    # you rows, and the orientation comes out silently wrong.
    forward = _norm(_swap(tangent))
    upward = _norm(_swap(up))
    # re-orthogonalise, then complete a right-handed basis
    side = _norm(_cross(forward, upward))
    upward = _norm(_cross(side, forward))

    m = (
        (forward[0], upward[0], side[0]),
        (forward[1], upward[1], side[1]),
        (forward[2], upward[2], side[2]),
    )
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s

    length = math.sqrt(x * x + y * y + z * z + w * w)
    return [x / length, y / length, z / length, w / length]


def _box_mesh(size_cm: Sequence[float]) -> Dict:
    """A car-sized box in Unreal centimetres, used when no car mesh is given."""
    hx, hy, hz = (max(float(v), 1.0) * 0.5 for v in size_cm)
    faces = (
        ((-1, 0, 0), ((-hx, -hy, -hz), (-hx, hy, -hz), (-hx, hy, hz), (-hx, -hy, hz))),
        ((1, 0, 0), ((hx, -hy, -hz), (hx, -hy, hz), (hx, hy, hz), (hx, hy, -hz))),
        ((0, -1, 0), ((-hx, -hy, -hz), (-hx, -hy, hz), (hx, -hy, hz), (hx, -hy, -hz))),
        ((0, 1, 0), ((-hx, hy, -hz), (hx, hy, -hz), (hx, hy, hz), (-hx, hy, hz))),
        ((0, 0, -1), ((-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz))),
        ((0, 0, 1), ((-hx, -hy, hz), (-hx, hy, hz), (hx, hy, hz), (hx, -hy, hz))),
    )
    corner_uvs = ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0))

    positions: List[float] = []
    normals: List[float] = []
    uvs: List[float] = []
    indices: List[int] = []
    for normal, corners in faces:
        base = len(positions) // 3
        for corner, uv in zip(corners, corner_uvs):
            positions.extend(corner)
            normals.extend(normal)
            uvs.extend(uv)
        indices.extend((base, base + 1, base + 2, base, base + 2, base + 3))

    return {
        "positions": positions,
        "normals": normals,
        "uvs": uvs,
        "indices": indices,
        "vertex_count": len(positions) // 3,
        "triangle_count": len(indices) // 3,
        "extent_cm": [max(positions[k::3]) - min(positions[k::3]) for k in range(3)],
    }


class _Blob:
    """Accumulates the binary chunk and hands back bufferView indices."""

    def __init__(self) -> None:
        self.parts: List[bytes] = []
        self.views: List[Dict] = []
        self.length = 0

    def add(self, payload: bytes, target: Optional[int] = None) -> int:
        # Every view starts 4-byte aligned, which satisfies each component type
        # used here and keeps readers that mmap the chunk happy.
        pad = (-self.length) % 4
        if pad:
            self.parts.append(b"\x00" * pad)
            self.length += pad
        view = {"buffer": 0, "byteOffset": self.length, "byteLength": len(payload)}
        if target is not None:
            view["target"] = target
        self.views.append(view)
        self.parts.append(payload)
        self.length += len(payload)
        return len(self.views) - 1

    def data(self) -> bytes:
        return b"".join(self.parts)


def _minmax(values: Sequence[float], stride: int) -> Dict:
    lo = [min(values[k::stride]) for k in range(stride)]
    hi = [max(values[k::stride]) for k in range(stride)]
    return {"min": [float(v) for v in lo], "max": [float(v) for v in hi]}


def mesh_from_builder(builder) -> Dict:
    """Flatten a MeshBuilder's polygon soup into glTF-ready triangles.

    MeshBuilder stores FBX-style polygons: the last index of each polygon is
    bitwise-negated to mark the end, and normals and UVs are per polygon-corner
    rather than per vertex. glTF wants indexed triangles with one attribute set
    per vertex, so every corner becomes its own vertex - which is also what keeps
    the flat-shaded boxes flat.
    """
    positions: List[float] = []
    normals: List[float] = []
    uvs: List[float] = []
    indices: List[int] = []

    loop: List[int] = []
    corner = 0
    corner_of_loop: List[int] = []
    for raw in builder.indices:
        end = raw < 0
        index = ~raw if end else raw
        loop.append(index)
        corner_of_loop.append(corner)
        corner += 1
        if not end:
            continue

        base = len(positions) // 3
        for vert, c in zip(loop, corner_of_loop):
            positions.extend(builder.verts[vert * 3:vert * 3 + 3])
            normals.extend(builder.normals[c * 3:c * 3 + 3])
            uvs.extend(builder.uvs[c * 2:c * 2 + 2])
        # fan-triangulate; every polygon here is a triangle or a quad
        for k in range(1, len(loop) - 1):
            indices.extend((base, base + k, base + k + 1))
        loop = []
        corner_of_loop = []

    return {
        "positions": positions,
        "normals": normals,
        "uvs": uvs,
        "indices": indices,
        "vertex_count": len(positions) // 3,
        "triangle_count": len(indices) // 3,
        "extent_cm": [
            (max(positions[k::3]) - min(positions[k::3])) if positions else 0.0
            for k in range(3)
        ],
    }


def write_static_glb(path, parts: Sequence, generator: str = "UE5 Coaster Pipeline") -> Dict:
    """Write named static meshes to one .glb, in Unreal's own space.

    parts is a sequence of (name, mesh dict) in Unreal centimetres. Used for the
    track: sending it through the same glTF path as the car is what keeps the two
    in the same place, because Unreal's FBX importer mirrors Y on the way in and
    its glTF importer does not.
    """
    blob = _Blob()
    accessors: List[Dict] = []
    meshes: List[Dict] = []
    nodes: List[Dict] = []
    summary: Dict = {}

    def accessor(payload: bytes, count: int, comp: int, kind: str,
                 target: Optional[int] = None, bounds: Optional[Dict] = None) -> int:
        view = blob.add(payload, target)
        entry = {"bufferView": view, "componentType": comp, "count": count, "type": kind}
        if bounds:
            entry.update(bounds)
        accessors.append(entry)
        return len(accessors) - 1

    def floats(values: Sequence[float]) -> bytes:
        return struct.pack("<" + "f" * len(values), *values)

    for name, mesh in parts:
        count = mesh["vertex_count"]
        if not count:
            continue
        pos: List[float] = []
        nrm: List[float] = []
        for i in range(count):
            p = _swap(mesh["positions"][i * 3:i * 3 + 3])
            pos.extend((p[0] * CM_TO_M, p[1] * CM_TO_M, p[2] * CM_TO_M))
            nrm.extend(_swap(mesh["normals"][i * 3:i * 3 + 3]))

        src = mesh["indices"]
        idx: List[int] = []
        for t in range(0, len(src) - 2, 3):
            idx.extend((src[t + 2], src[t + 1], src[t]))

        a_pos = accessor(floats(pos), count, FLOAT, "VEC3", ARRAY_BUFFER, _minmax(pos, 3))
        a_nrm = accessor(floats(nrm), count, FLOAT, "VEC3", ARRAY_BUFFER)
        a_uv = accessor(floats(mesh["uvs"]), count, FLOAT, "VEC2", ARRAY_BUFFER)
        a_idx = accessor(struct.pack("<" + "I" * len(idx), *idx), len(idx),
                         UNSIGNED_INT, "SCALAR", ELEMENT_ARRAY_BUFFER)

        meshes.append({
            "name": name,
            "primitives": [{
                "attributes": {"POSITION": a_pos, "NORMAL": a_nrm, "TEXCOORD_0": a_uv},
                "indices": a_idx,
                "material": 0,
                "mode": 4,
            }],
        })
        nodes.append({"name": name, "mesh": len(meshes) - 1})
        summary[name] = len(idx) // 3

    if not meshes:
        raise ValueError("no geometry to write")

    gltf = {
        "asset": {"version": "2.0", "generator": generator},
        "scene": 0,
        "scenes": [{"nodes": list(range(len(nodes)))}],
        "nodes": nodes,
        "meshes": meshes,
        "materials": [{
            "name": "CoasterTrackMaterial",
            "pbrMetallicRoughness": {
                "baseColorFactor": [0.62, 0.62, 0.65, 1.0],
                "metallicFactor": 0.35,
                "roughnessFactor": 0.55,
            },
        }],
        "bufferViews": blob.views,
        "accessors": accessors,
        "buffers": [{"byteLength": blob.length}],
    }

    json_chunk = json.dumps(gltf, separators=(",", ":")).encode("utf-8")
    json_chunk += b" " * ((-len(json_chunk)) % 4)
    bin_chunk = blob.data()
    bin_chunk += b"\x00" * ((-len(bin_chunk)) % 4)

    total = 12 + 8 + len(json_chunk) + 8 + len(bin_chunk)
    out = bytearray()
    out += struct.pack("<III", GLB_MAGIC, 2, total)
    out += struct.pack("<II", len(json_chunk), CHUNK_JSON) + json_chunk
    out += struct.pack("<II", len(bin_chunk), CHUNK_BIN) + bin_chunk

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(bytes(out))
    return {"path": str(path), "parts": summary, "size_bytes": path.stat().st_size}


def write_car_glb(
    path,
    samples: List[Dict],
    fps: int = 60,
    node_name: str = "CoasterCar",
    box_size_cm: Sequence[float] = (450.0, 160.0, 120.0),
    car_mesh_file: Optional[str] = None,
    car_forward_axis: str = "auto",
    import_fps: int = 30,
    generator: str = "UE5 Coaster Pipeline",
) -> Dict:
    """Write the ride as a skinned, animated .glb. Returns a summary dict.

    One joint carries the motion and the car is skinned to it at full weight -
    a coaster car is a rigid body, so there is no weighting subtlety. The mesh
    node sits at the scene root, which is what the glTF spec asks for on a
    skinned mesh.
    """
    frames = resample_uniform_fps(samples, fps)
    frames = trim_to_frame_border(frames, fps, import_fps)
    if len(frames) < 2:
        raise ValueError("Not enough samples to write an animation")

    # ---- geometry, in Unreal centimetres, facing +X
    if car_mesh_file:
        mesh = read_glb_triangles(car_mesh_file)
        axis = orient_forward(mesh, car_forward_axis)
        extent = mesh["extent_cm"]
        note = (
            f"{Path(car_mesh_file).name}: {mesh['triangle_count']} triangles, "
            f"{extent[0] / 100.0:.2f} x {extent[1] / 100.0:.2f} x "
            f"{extent[2] / 100.0:.2f} m, facing {axis}"
        )
    else:
        mesh = _box_mesh(box_size_cm)
        note = "placeholder box"

    vertex_count = mesh["vertex_count"]

    positions: List[float] = []
    normals: List[float] = []
    for i in range(vertex_count):
        p = _swap(mesh["positions"][i * 3:i * 3 + 3])
        positions.extend((p[0] * CM_TO_M, p[1] * CM_TO_M, p[2] * CM_TO_M))
        normals.extend(_swap(mesh["normals"][i * 3:i * 3 + 3]))

    uvs = list(mesh["uvs"]) if mesh.get("uvs") else [0.0] * (vertex_count * 2)

    # The swap flips winding, so put each triangle back the way round it was.
    src = mesh["indices"]
    indices: List[int] = []
    for t in range(0, len(src) - 2, 3):
        indices.extend((src[t + 2], src[t + 1], src[t]))

    # Every vertex rides the one bone, which is joint 1 in the skin.
    joints = bytes([1, 0, 0, 0]) * vertex_count
    weights = struct.pack("<" + "f" * (vertex_count * 4), *([1.0, 0.0, 0.0, 0.0] * vertex_count))

    # ---- animation
    times: List[float] = []
    translations: List[float] = []
    rotations: List[float] = []
    for frame in frames:
        times.append(float(frame["time_s"]))
        pos = _swap(frame["ue_pos_cm"])
        translations.extend((pos[0] * CM_TO_M, pos[1] * CM_TO_M, pos[2] * CM_TO_M))
        rotations.extend(quat_from_tangent_up(frame["ue_tan"], frame["ue_up"]))

    # ---- pack
    blob = _Blob()
    accessors: List[Dict] = []

    def accessor(payload: bytes, count: int, comp: int, kind: str,
                 target: Optional[int] = None, bounds: Optional[Dict] = None) -> int:
        view = blob.add(payload, target)
        entry = {
            "bufferView": view,
            "componentType": comp,
            "count": count,
            "type": kind,
        }
        if bounds:
            entry.update(bounds)
        accessors.append(entry)
        return len(accessors) - 1

    def floats(values: Sequence[float]) -> bytes:
        return struct.pack("<" + "f" * len(values), *values)

    a_pos = accessor(floats(positions), vertex_count, FLOAT, "VEC3",
                     ARRAY_BUFFER, _minmax(positions, 3))
    a_nrm = accessor(floats(normals), vertex_count, FLOAT, "VEC3", ARRAY_BUFFER)
    a_uv = accessor(floats(uvs), vertex_count, FLOAT, "VEC2", ARRAY_BUFFER)
    a_joint = accessor(joints, vertex_count, UNSIGNED_BYTE, "VEC4", ARRAY_BUFFER)
    a_weight = accessor(weights, vertex_count, FLOAT, "VEC4", ARRAY_BUFFER)
    a_index = accessor(struct.pack("<" + "I" * len(indices), *indices),
                       len(indices), UNSIGNED_INT, "SCALAR", ELEMENT_ARRAY_BUFFER)

    # Both joints sit at the origin at bind time, so the inverse bind matrices
    # are identity. glTF stores matrices column-major.
    identity = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    a_ibm = accessor(floats(identity * 2), 2, FLOAT, "MAT4")

    a_time = accessor(floats(times), len(times), FLOAT, "SCALAR",
                      bounds={"min": [float(min(times))], "max": [float(max(times))]})
    a_trans = accessor(floats(translations), len(times), FLOAT, "VEC3")
    a_rot = accessor(floats(rotations), len(times), FLOAT, "VEC4")

    rig_name = f"{node_name}Rig"
    bone_name = f"{node_name}Bone"

    gltf = {
        "asset": {"version": "2.0", "generator": generator},
        "scene": 0,
        "scenes": [{"nodes": [0, 2]}],
        "nodes": [
            {"name": rig_name, "children": [1]},
            {"name": bone_name},
            {"name": node_name, "mesh": 0, "skin": 0},
        ],
        "meshes": [{
            "name": node_name,
            "primitives": [{
                "attributes": {
                    "POSITION": a_pos,
                    "NORMAL": a_nrm,
                    "TEXCOORD_0": a_uv,
                    "JOINTS_0": a_joint,
                    "WEIGHTS_0": a_weight,
                },
                "indices": a_index,
                "material": 0,
                "mode": 4,
            }],
        }],
        # A plain grey PBR material. Without one Interchange warns that the
        # primitive has no material and assigns the world grid default; the
        # textured look comes from importing the car's own .glb separately.
        "materials": [{
            "name": f"{node_name}Material",
            "pbrMetallicRoughness": {
                "baseColorFactor": [0.75, 0.75, 0.78, 1.0],
                "metallicFactor": 0.1,
                "roughnessFactor": 0.6,
            },
        }],
        "skins": [{
            "name": f"{node_name}Skin",
            "skeleton": 0,
            "joints": [0, 1],
            "inverseBindMatrices": a_ibm,
        }],
        "animations": [{
            "name": "CoasterRide",
            "samplers": [
                {"input": a_time, "output": a_trans, "interpolation": "LINEAR"},
                {"input": a_time, "output": a_rot, "interpolation": "LINEAR"},
            ],
            "channels": [
                {"sampler": 0, "target": {"node": 1, "path": "translation"}},
                {"sampler": 1, "target": {"node": 1, "path": "rotation"}},
            ],
        }],
        "bufferViews": blob.views,
        "accessors": accessors,
        "buffers": [{"byteLength": blob.length}],
    }

    json_chunk = json.dumps(gltf, separators=(",", ":")).encode("utf-8")
    json_chunk += b" " * ((-len(json_chunk)) % 4)
    bin_chunk = blob.data()
    bin_chunk += b"\x00" * ((-len(bin_chunk)) % 4)

    total = 12 + 8 + len(json_chunk) + 8 + len(bin_chunk)
    out = bytearray()
    out += struct.pack("<III", GLB_MAGIC, 2, total)
    out += struct.pack("<II", len(json_chunk), CHUNK_JSON) + json_chunk
    out += struct.pack("<II", len(bin_chunk), CHUNK_BIN) + bin_chunk

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(bytes(out))

    return {
        "path": str(path),
        "fps": fps,
        "import_fps": import_fps,
        "frames": len(frames),
        "duration_s": frames[-1]["time_s"],
        "node_name": node_name,
        "bone_name": bone_name,
        "geometry": note,
        "vertices": vertex_count,
        "triangles": len(indices) // 3,
        "size_bytes": path.stat().st_size,
    }
