"""
Read triangle geometry out of a .glb / .gltf file.

Only what the pipeline needs: positions, normals and triangle indices, with node
transforms applied, converted into the same Z-up centimetre space the FBX
exporters use. Materials and textures are deliberately not carried across - the
GLB itself is still imported separately in Unreal for the textured visual, and
this exists so the animated FBX can contain the real car instead of a box.

glTF is right-handed and Y-up in metres, the same convention as the NoLimits and
OpenFVD sources, so the axis mapping verified for those applies unchanged.
"""

from __future__ import annotations

import json
import math
import struct
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

GLB_MAGIC = 0x46546C67  # "glTF"
CHUNK_JSON = 0x4E4F534A
CHUNK_BIN = 0x004E4942

# accessor.componentType -> (struct code, byte size)
COMPONENT = {
    5120: ("b", 1),
    5121: ("B", 1),
    5122: ("h", 2),
    5123: ("H", 2),
    5125: ("I", 4),
    5126: ("f", 4),
}

COMPONENT_COUNT = {
    "SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4,
    "MAT2": 4, "MAT3": 9, "MAT4": 16,
}

M_TO_CM = 100.0


class GlbError(RuntimeError):
    pass


def _spin_180(mesh: Dict) -> None:
    """Turn the mesh end for end about Z, in place."""
    for array in (mesh["positions"], mesh["normals"]):
        for i in range(0, len(array), 3):
            array[i] = -array[i]
            array[i + 1] = -array[i + 1]
    mesh["extent_cm"] = [
        max(mesh["positions"][k::3]) - min(mesh["positions"][k::3]) for k in range(3)
    ]


def _load(path: Path) -> Tuple[Dict, bytes]:
    """Return (gltf json, binary blob) for either .glb or .gltf."""
    raw = path.read_bytes()
    if len(raw) >= 12 and struct.unpack_from("<I", raw, 0)[0] == GLB_MAGIC:
        _, version, _ = struct.unpack_from("<III", raw, 0)
        if version != 2:
            raise GlbError(f"{path.name} is glTF version {version}; only 2 is read")
        offset = 12
        gltf: Optional[Dict] = None
        blob = b""
        while offset + 8 <= len(raw):
            length, kind = struct.unpack_from("<II", raw, offset)
            offset += 8
            chunk = raw[offset:offset + length]
            offset += length
            if kind == CHUNK_JSON:
                gltf = json.loads(chunk.decode("utf-8"))
            elif kind == CHUNK_BIN:
                blob = chunk
        if gltf is None:
            raise GlbError(f"{path.name} has no JSON chunk")
        return gltf, blob

    # Plain .gltf: the buffer may be embedded as a data URI or sit alongside.
    gltf = json.loads(raw.decode("utf-8"))
    blob = b""
    buffers = gltf.get("buffers") or []
    if buffers:
        uri = buffers[0].get("uri", "")
        if uri.startswith("data:"):
            import base64

            blob = base64.b64decode(uri.split(",", 1)[1])
        elif uri:
            from urllib.parse import unquote

            blob = (path.parent / unquote(uri)).read_bytes()
    return gltf, blob


def _read_accessor(gltf: Dict, blob: bytes, index: int) -> List:
    """Decode one accessor into a flat list, honouring byteStride."""
    accessor = gltf["accessors"][index]
    count = accessor["count"]
    comp_code, comp_size = COMPONENT[accessor["componentType"]]
    per_item = COMPONENT_COUNT[accessor["type"]]

    if "bufferView" not in accessor:
        # Sparse-only accessors are legal and read as zeros.
        return [0] * (count * per_item)

    view = gltf["bufferViews"][accessor["bufferView"]]
    base = view.get("byteOffset", 0) + accessor.get("byteOffset", 0)
    stride = view.get("byteStride") or (comp_size * per_item)

    out: List = []
    item = struct.Struct("<" + comp_code * per_item)
    for i in range(count):
        out.extend(item.unpack_from(blob, base + i * stride))
    return out


def _node_matrix(node: Dict) -> List[float]:
    """Local transform as a 4x4, row-major."""
    if "matrix" in node:
        m = node["matrix"]
        # glTF stores column-major; transpose into row-major.
        return [
            m[0], m[4], m[8], m[12],
            m[1], m[5], m[9], m[13],
            m[2], m[6], m[10], m[14],
            m[3], m[7], m[11], m[15],
        ]

    tx, ty, tz = node.get("translation", (0.0, 0.0, 0.0))
    qx, qy, qz, qw = node.get("rotation", (0.0, 0.0, 0.0, 1.0))
    sx, sy, sz = node.get("scale", (1.0, 1.0, 1.0))

    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    rot = [
        1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy),
        2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx),
        2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy),
    ]
    return [
        rot[0] * sx, rot[1] * sy, rot[2] * sz, tx,
        rot[3] * sx, rot[4] * sy, rot[5] * sz, ty,
        rot[6] * sx, rot[7] * sy, rot[8] * sz, tz,
        0.0, 0.0, 0.0, 1.0,
    ]


def _mat_mul(a: Sequence[float], b: Sequence[float]) -> List[float]:
    out = [0.0] * 16
    for r in range(4):
        for c in range(4):
            out[r * 4 + c] = sum(a[r * 4 + k] * b[k * 4 + c] for k in range(4))
    return out


def _transform_point(m: Sequence[float], p: Sequence[float]) -> Tuple[float, float, float]:
    return (
        m[0] * p[0] + m[1] * p[1] + m[2] * p[2] + m[3],
        m[4] * p[0] + m[5] * p[1] + m[6] * p[2] + m[7],
        m[8] * p[0] + m[9] * p[1] + m[10] * p[2] + m[11],
    )


def _transform_direction(m: Sequence[float], v: Sequence[float]) -> Tuple[float, float, float]:
    # Good enough for normals here: the car's transforms carry no shear.
    out = (
        m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
        m[4] * v[0] + m[5] * v[1] + m[6] * v[2],
        m[8] * v[0] + m[9] * v[1] + m[10] * v[2],
    )
    length = math.sqrt(sum(c * c for c in out))
    return tuple(c / length for c in out) if length > 1e-12 else (0.0, 0.0, 1.0)


IDENTITY = [
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
]


def read_glb_triangles(path) -> Dict:
    """Flatten a glTF file into one triangle soup in Unreal centimetres.

    Returns positions, per-vertex normals and triangle indices. Every mesh in
    the file is merged: a coaster car arrives as several parts and they all
    belong to the same rigid body here.
    """
    path = Path(path)
    gltf, blob = _load(path)

    meshes = gltf.get("meshes") or []
    nodes = gltf.get("nodes") or []
    if not meshes:
        raise GlbError(f"{path.name} contains no meshes")

    scene_index = gltf.get("scene", 0)
    scenes = gltf.get("scenes") or [{"nodes": list(range(len(nodes)))}]
    roots = scenes[scene_index].get("nodes", [])

    positions: List[float] = []
    normals: List[float] = []
    uvs: List[float] = []
    indices: List[int] = []
    primitives_read = 0
    skipped_modes = set()

    def visit(node_index: int, parent: Sequence[float]) -> None:
        nonlocal primitives_read
        node = nodes[node_index]
        world = _mat_mul(parent, _node_matrix(node))

        if "mesh" in node:
            for prim in meshes[node["mesh"]].get("primitives", []):
                mode = prim.get("mode", 4)
                if mode != 4:
                    skipped_modes.add(mode)
                    continue
                attrs = prim.get("attributes") or {}
                if "POSITION" not in attrs:
                    continue

                flat = _read_accessor(gltf, blob, attrs["POSITION"])
                vertex_count = len(flat) // 3
                base = len(positions) // 3

                has_normals = "NORMAL" in attrs
                flat_n = (
                    _read_accessor(gltf, blob, attrs["NORMAL"]) if has_normals else []
                )
                has_uv = "TEXCOORD_0" in attrs
                flat_uv = (
                    _read_accessor(gltf, blob, attrs["TEXCOORD_0"]) if has_uv else []
                )

                for i in range(vertex_count):
                    p = _transform_point(world, flat[i * 3:i * 3 + 3])
                    # glTF is right-handed Y-up; (x, y, z) -> (x, z, y) converts
                    # to Unreal's left-handed Z-up. Determinant -1, which is what
                    # makes it a conversion rather than a mirror.
                    positions.extend((p[0] * M_TO_CM, p[2] * M_TO_CM, p[1] * M_TO_CM))
                    if has_normals:
                        n = _transform_direction(world, flat_n[i * 3:i * 3 + 3])
                        normals.extend((n[0], n[2], n[1]))
                    else:
                        normals.extend((0.0, 0.0, 1.0))

                    if has_uv:
                        # glTF's V runs top-down; FBX and Unreal expect it flipped.
                        uvs.extend((flat_uv[i * 2], 1.0 - flat_uv[i * 2 + 1]))
                    else:
                        uvs.extend((0.0, 0.0))

                if "indices" in prim:
                    local = _read_accessor(gltf, blob, prim["indices"])
                else:
                    local = list(range(vertex_count))
                # The axis swap flips winding, so reverse each triangle to keep
                # the faces pointing outwards.
                for t in range(0, len(local) - 2, 3):
                    indices.extend(
                        (base + local[t + 2], base + local[t + 1], base + local[t])
                    )
                primitives_read += 1

        for child in node.get("children", []) or []:
            visit(child, world)

    for root in roots:
        visit(root, IDENTITY)

    if not indices:
        raise GlbError(
            f"{path.name} yielded no triangles"
            + (f" (skipped primitive modes {sorted(skipped_modes)})" if skipped_modes else "")
        )

    extent = [
        max(positions[k::3]) - min(positions[k::3]) for k in range(3)
    ]
    return {
        "positions": positions,
        "normals": normals,
        "uvs": uvs,
        "indices": indices,
        "vertex_count": len(positions) // 3,
        "triangle_count": len(indices) // 3,
        "primitives": primitives_read,
        "extent_cm": extent,
        "skipped_modes": sorted(skipped_modes),
    }


def _nose_is_at_positive_x(mesh: Dict) -> bool:
    """Guess which end of an X-aligned car is the front.

    Bounding boxes give you the long axis but not which way along it the car
    faces, and getting that backwards drives the car round the track in reverse
    - visibly wrong, and invisible to every other check. A coaster car's nose is
    low and tapered while its back carries the seat backs and headrests, so the
    taller half is the rear. --car-forward-axis overrides this outright.
    """
    xs = mesh["positions"][0::3]
    zs = mesh["positions"][2::3]
    lo, hi = min(xs), max(xs)
    mid = (lo + hi) * 0.5

    def height(front: bool) -> float:
        picked = [zs[i] for i in range(len(xs)) if (xs[i] >= mid) == front]
        return (max(picked) - min(picked)) if picked else 0.0

    return height(True) <= height(False)


def detect_bogie_rails(mesh: Dict) -> Optional[Dict]:
    """Measure where a car's bogies expect the rails: height and gauge.

    A bogie grips the rail from both sides - road wheels on top, upstop wheels
    underneath - so the outboard running gear is spread roughly symmetrically
    about the rail line, and its vertical centre estimates that line. Whatever
    sits at that height then gives the gauge.

    Measuring both off the model is what snaps the car onto its own track. The
    defaults could not: this car's rails belong at the path, 175 cm apart, and
    the old fixed 110 cm drop and 100 cm gauge left it hovering above rails that
    were both too low and too narrow.

    Returns {"slot_z_cm", "gauge_cm"} or None if there is too little geometry.
    """
    positions = mesh.get("positions") or []
    if len(positions) < 600:
        return None
    ys = positions[1::3]
    zs = positions[2::3]

    # Bogies are outboard and low; the seats and bodywork are neither.
    widest = max(abs(y) for y in ys)
    if widest <= 1e-6:
        return None
    ceiling = sorted(zs)[int(len(zs) * 0.75)]
    gear = [i for i in range(len(zs)) if zs[i] <= ceiling and abs(ys[i]) >= widest * 0.4]
    if len(gear) < 200:
        return None

    heights = sorted(zs[i] for i in gear)
    slot_z = heights[len(heights) // 2]

    lateral = [abs(ys[i]) for i in gear if abs(zs[i] - slot_z) <= 9.0]
    if len(lateral) < 100:
        return None
    outboard = [y for y in lateral if y >= max(lateral) * 0.5]
    if not outboard:
        return None

    return {
        "slot_z_cm": slot_z,
        "gauge_cm": 2.0 * sum(outboard) / len(outboard),
    }


def detect_rail_slot_cm(mesh: Dict, gauge_cm: float = 100.0) -> Optional[float]:
    """Find the height, in car-local cm, where the rail passes through a bogie.

    A coaster bogie grips the rail from both sides: road wheels ride on top of
    it, upstop wheels grip underneath, and between them is a gap the rail runs
    through. That gap shows up as a collapse in vertex density at the rail's
    lateral offset, so the rail line can be measured off the model instead of
    guessed - which is what stops the car floating above its own track.

    Returns None when there is no clear gap, e.g. a car with no modelled bogies.
    """
    positions = mesh.get("positions") or []
    if not positions:
        return None

    xs = positions[0::3]
    ys = positions[1::3]
    zs = positions[2::3]
    band = gauge_cm * 0.5
    picked = [
        zs[i] for i in range(len(zs))
        if band - 28.0 <= abs(ys[i]) <= band + 28.0
    ]
    if len(picked) < 200:
        return None

    # Only the running gear matters, so ignore the seats and bodywork above it.
    picked.sort()
    ceiling = picked[int(len(picked) * 0.75)]
    picked = [z for z in picked if z <= ceiling]
    if len(picked) < 200:
        return None

    lo, hi = min(picked), max(picked)
    if hi - lo < 20.0:
        return None

    step = 4.0
    bins = int((hi - lo) // step) + 1
    counts = [0] * bins
    for z in picked:
        counts[min(int((z - lo) // step), bins - 1)] += 1

    ordered = sorted(c for c in counts if c)
    if not ordered:
        return None
    median = ordered[len(ordered) // 2]
    threshold = median * 0.6

    # Longest run of sparse bins that still has real geometry on both sides:
    # that is the slot, rather than open air below the wheels.
    total = sum(counts)
    best = None
    start = None
    for k in range(bins + 1):
        sparse = k < bins and counts[k] <= threshold
        if sparse and start is None:
            start = k
        elif not sparse and start is not None:
            below = sum(counts[:start])
            above = sum(counts[k:])
            if below >= 0.2 * total and above >= 0.2 * total:
                length = k - start
                if best is None or length > best[0]:
                    best = (length, start, k)
            start = None

    if best is None:
        return None
    _, a, b = best
    return lo + (a + b) * 0.5 * step


def orient_forward(mesh: Dict, forward_axis: str = "auto") -> str:
    """Rotate the mesh in place so it faces +X, and report the axis assumed.

    The exporters drive everything along +X, so a car modelled facing another
    way has to be turned once here rather than corrected at every use.
    """
    extent = mesh["extent_cm"]
    axis = forward_axis
    if axis == "auto":
        axis = "+X" if extent[0] >= extent[1] else "+Y"

    # Yaw about Z that brings the named axis onto +X.
    yaw = {"+X": 0.0, "-Y": 90.0, "-X": 180.0, "+Y": -90.0}.get(axis, 0.0)
    if abs(yaw) < 1e-9:
        if forward_axis == "auto" and not _nose_is_at_positive_x(mesh):
            _spin_180(mesh)
            return "-X"
        return axis

    angle = math.radians(yaw)
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    for array in (mesh["positions"], mesh["normals"]):
        for i in range(0, len(array), 3):
            x, y = array[i], array[i + 1]
            array[i] = x * cos_a - y * sin_a
            array[i + 1] = x * sin_a + y * cos_a

    mesh["extent_cm"] = [
        max(mesh["positions"][k::3]) - min(mesh["positions"][k::3]) for k in range(3)
    ]

    if forward_axis == "auto" and not _nose_is_at_positive_x(mesh):
        _spin_180(mesh)
        axis = {"+X": "-X", "-X": "+X", "+Y": "-Y", "-Y": "+Y"}[axis]
    return axis
