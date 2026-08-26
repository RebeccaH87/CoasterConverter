"""
Write the coaster car's baked motion as an FBX that Unreal can import directly.

No Blender and no FBX SDK: binary FBX 7.4 is emitted straight from the timeline.
That puts the animation inside the exported folder as a real file rather than
something a script has to reconstruct inside the editor.

Binary rather than ASCII deliberately - most readers, Blender included, refuse
ASCII FBX outright, so an ASCII file could not even be verified.

The scene is declared Z-up in centimetres, matching Unreal exactly, so the
importer has no unit or axis conversion to get wrong.
"""

from __future__ import annotations

import math
import struct
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

FBX_VERSION = 7400

# FBX stores time as an integer count; this many units per second.
FBX_TIME_ONE_SECOND = 46186158000

# Opaque tangent payload accompanying every key. The third entry is a float
# whose *bit pattern* is this integer, not the integer's value; the number was
# taken from a reference file rather than guessed.
KEY_ATTR_FLAGS = 24836
KEY_ATTR_DATA = (
    0.0,
    0.0,
    struct.unpack("<f", struct.pack("<i", 255790911))[0],
    0.0,
)

HEADER_MAGIC = b"Kaydara FBX Binary  " + bytes([0x00, 0x1A, 0x00])
FOOTER_ID = bytes.fromhex("fabcab09d0c8d466b176fb831cf7267e")
FOOTER_EXT = bytes.fromhex("f85a8c6adef5d97eece90ce3758f290b")

# FbxTime::EMode values for the rates worth naming; anything else goes custom.
TIME_MODES = {120: 1, 100: 2, 60: 3, 50: 4, 48: 5, 30: 6, 24: 11, 1000: 12}
TIME_MODE_CUSTOM = 14


# --------------------------------------------------------------------------
# binary record plumbing
# --------------------------------------------------------------------------

class Node:
    """One FBX record: a name, typed properties, and nested records."""

    def __init__(self, name: str, props=None, children=None):
        self.name = name
        self.props = list(props or [])
        self.children = list(children or [])

    def add(self, child: "Node") -> "Node":
        self.children.append(child)
        return child


def _i(v):
    return ("I", int(v))


def _l(v):
    return ("L", int(v))


def _d(v):
    return ("D", float(v))


def _b(v):
    return ("C", bool(v))


def _s(v):
    return ("S", v if isinstance(v, bytes) else str(v).encode("utf-8"))


def _arr(kind, values):
    return (kind + "[]", list(values))


def obj_name(name: str, klass: str) -> bytes:
    """Binary FBX packs an object name as name<NUL><SOH>Class.

    Note this is the reverse of the ASCII form's "Class::name".
    """
    return name.encode("utf-8") + bytes([0x00, 0x01]) + klass.encode("utf-8")


_ARRAY_FMT = {"f": "f", "d": "d", "l": "q", "i": "i", "b": "b"}


def encode_prop(prop) -> bytes:
    kind, value = prop
    if kind == "I":
        return b"I" + struct.pack("<i", value)
    if kind == "L":
        return b"L" + struct.pack("<q", value)
    if kind == "D":
        return b"D" + struct.pack("<d", value)
    if kind == "F":
        return b"F" + struct.pack("<f", value)
    if kind == "C":
        return b"C" + bytes([1 if value else 0])
    if kind == "S":
        return b"S" + struct.pack("<I", len(value)) + value
    if kind == "R":
        return b"R" + struct.pack("<I", len(value)) + value
    if kind.endswith("[]"):
        code = kind[0]
        fmt = _ARRAY_FMT[code]
        payload = struct.pack("<" + fmt * len(value), *value)
        # Encoding 0 is uncompressed. Deflate would be smaller but adds a
        # failure mode for no real benefit at this size.
        return (
            code.encode("ascii")
            + struct.pack("<III", len(value), 0, len(payload))
            + payload
        )
    raise ValueError(f"unsupported property kind {kind!r}")


def serialize_node(node: Node, offset: int) -> bytes:
    """Serialise one record. EndOffset is absolute, hence the offset argument."""
    props = b"".join(encode_prop(p) for p in node.props)
    name = node.name.encode("ascii")
    header_len = 4 + 4 + 4 + 1 + len(name)

    body = b""
    cursor = offset + header_len + len(props)
    if node.children:
        for child in node.children:
            chunk = serialize_node(child, cursor)
            body += chunk
            cursor += len(chunk)
        # A record with children is closed by a null record.
        body += bytes(13)
        cursor += 13

    return (
        struct.pack("<III", cursor, len(node.props), len(props))
        + bytes([len(name)])
        + name
        + props
        + body
    )


def serialize_document(roots: List[Node]) -> bytes:
    out = bytearray(HEADER_MAGIC + struct.pack("<I", FBX_VERSION))
    for root in roots:
        out += serialize_node(root, len(out))
    out += bytes(13)

    out += FOOTER_ID
    # The version field in the footer sits on a 16-byte boundary.
    out += bytes((-len(out)) % 16)
    out += struct.pack("<I", FBX_VERSION)
    out += bytes(120)
    out += FOOTER_EXT
    return bytes(out)


# --------------------------------------------------------------------------
# motion maths
# --------------------------------------------------------------------------

def _norm(v):
    length = math.sqrt(sum(c * c for c in v))
    return [c / length for c in v] if length > 1e-12 else [0.0, 0.0, 1.0]


def _cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def basis_from_tangent_up(tangent: Sequence[float], up: Sequence[float]):
    """Orthonormal basis with local X along travel and local Z along up.

    Columns are the local axes in world space, which is what the Euler
    decomposition below expects.
    """
    x_axis = _norm(tangent)
    z_axis = _norm(up)
    # Unreal is left-handed, so Y completes the basis as Z cross X.
    y_axis = _norm(_cross(z_axis, x_axis))
    z_axis = _norm(_cross(x_axis, y_axis))
    return [
        [x_axis[0], y_axis[0], z_axis[0]],
        [x_axis[1], y_axis[1], z_axis[1]],
        [x_axis[2], y_axis[2], z_axis[2]],
    ]


def euler_xyz_degrees(m) -> Tuple[float, float, float]:
    """Decompose to the Euler triple FBX's default rotation order expects.

    FBX eEulerXYZ composes as M = Rz * Ry * Rx, so the angles come out in that
    order rather than the more familiar Rx * Ry * Rz.
    """
    b = math.asin(max(-1.0, min(1.0, -m[2][0])))
    if abs(m[2][0]) < 0.999999:
        a = math.atan2(m[2][1], m[2][2])
        c = math.atan2(m[1][0], m[0][0])
    else:
        # Gimbal lock: a degree of freedom is lost, so fold it into a.
        a = math.atan2(-m[1][2], m[1][1])
        c = 0.0
    return math.degrees(a), math.degrees(b), math.degrees(c)


def unwrap_degrees(channel: List[float]) -> None:
    """Remove 360-degree jumps in place.

    Without this, a roll sweeping past +-180 reads as an instant spin backwards:
    visible as a snap, and enough to make any curve fitted to it report a huge
    false angular velocity.
    """
    for i in range(1, len(channel)):
        while channel[i] - channel[i - 1] > 180.0:
            channel[i] -= 360.0
        while channel[i] - channel[i - 1] < -180.0:
            channel[i] += 360.0


def resample_uniform_fps(samples: List[Dict], fps: int) -> List[Dict]:
    """Pick evenly spaced frames from the timeline.

    Keys land on exact frame boundaries because an FBX consumer plays them on a
    fixed frame clock; the converter's arc-length spacing would put keys at
    irregular times for no benefit.
    """
    if not samples:
        return []

    duration = float(samples[-1]["time_s"])
    frame_count = max(int(round(duration * fps)) + 1, 2)
    times = [float(s["time_s"]) for s in samples]

    out = []
    cursor = 0
    for frame in range(frame_count):
        t = min(frame / float(fps), duration)
        while cursor < len(times) - 2 and times[cursor + 1] < t:
            cursor += 1

        lo = samples[cursor]
        hi = samples[min(cursor + 1, len(samples) - 1)]
        span = float(hi["time_s"]) - float(lo["time_s"])
        alpha = 0.0 if span <= 1e-12 else (t - float(lo["time_s"])) / span

        def blend(key):
            a, b = lo[key], hi[key]
            return [a[i] + (b[i] - a[i]) * alpha for i in range(3)]

        out.append(
            {
                "time_s": t,
                "ue_pos_cm": blend("ue_pos_cm"),
                "ue_tan": blend("ue_tan"),
                "ue_up": blend("ue_up"),
            }
        )
    return out


# --------------------------------------------------------------------------
# document assembly
# --------------------------------------------------------------------------

def _prop70(name, type_a, type_b, flags, *values):
    props = [_s(name), _s(type_a), _s(type_b), _s(flags)]
    for v in values:
        props.append(v)
    return Node("P", props)


def _curve_node(object_id, label, first_values):
    node = Node(
        "AnimationCurveNode",
        [_l(object_id), _s(obj_name(label, "AnimCurveNode")), _s("")],
    )
    props = node.add(Node("Properties70"))
    for axis, value in zip("XYZ", first_values):
        props.add(_prop70(f"d|{axis}", "Number", "", "A", _d(value)))
    return node


def _curve(object_id, times_ticks, values):
    node = Node(
        "AnimationCurve",
        [_l(object_id), _s(obj_name("", "AnimCurve")), _s("")],
    )
    node.add(Node("Default", [_d(values[0] if values else 0.0)]))
    node.add(Node("KeyVer", [_i(4008)]))
    node.add(Node("KeyTime", [_arr("l", times_ticks)]))
    node.add(Node("KeyValueFloat", [_arr("f", values)]))
    node.add(Node("KeyAttrFlags", [_arr("i", [KEY_ATTR_FLAGS])]))
    node.add(Node("KeyAttrDataFloat", [_arr("f", KEY_ATTR_DATA)]))
    node.add(Node("KeyAttrRefCount", [_arr("i", [len(times_ticks)])]))
    return node


def _box_geometry(object_id, size_cm):
    """A car-sized box so the FBX is visibly animated on its own.

    The real car mesh travels beside the bundle and is bound in Unreal; this
    exists so the motion can be inspected without any other asset.
    """
    hx, hy, hz = (max(float(s), 1.0) * 0.5 for s in size_cm)
    verts = [
        (-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),
        (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz),
    ]
    faces = [
        (0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
        (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7),
    ]

    indices = []
    for face in faces:
        indices.extend(face[:-1])
        # FBX marks a polygon's last index by bitwise negation.
        indices.append(~face[-1])

    node = Node(
        "Geometry",
        [_l(object_id), _s(obj_name("CoasterCar", "Geometry")), _s("Mesh")],
    )
    node.add(Node("GeometryVersion", [_i(124)]))
    node.add(Node("Vertices", [_arr("d", [c for v in verts for c in v])]))
    node.add(Node("PolygonVertexIndex", [_arr("i", indices)]))

    normals = [
        (0, 0, -1), (0, 0, 1), (0, -1, 0), (1, 0, 0), (0, 1, 0), (-1, 0, 0),
    ]
    layer = node.add(Node("LayerElementNormal", [_i(0)]))
    layer.add(Node("Version", [_i(102)]))
    layer.add(Node("Name", [_s("")]))
    layer.add(Node("MappingInformationType", [_s("ByPolygon")]))
    layer.add(Node("ReferenceInformationType", [_s("Direct")]))
    layer.add(Node("Normals", [_arr("d", [float(c) for n in normals for c in n])]))
    return node


def write_car_animation_fbx(
    path,
    samples: List[Dict],
    fps: int = 60,
    node_name: str = "CoasterCar",
    box_size_cm: Sequence[float] = (450.0, 160.0, 120.0),
    creator: str = "UE5 Coaster Pipeline",
) -> Dict:
    """Write the baked car motion to a binary FBX. Returns a summary dict."""
    frames = resample_uniform_fps(samples, fps)
    if len(frames) < 2:
        raise ValueError("Not enough samples to write an animation")

    times_ticks = [
        int(round(f["time_s"] * FBX_TIME_ONE_SECOND)) for f in frames
    ]

    translations = [[], [], []]
    rotations = [[], [], []]
    for frame in frames:
        for axis in range(3):
            translations[axis].append(float(frame["ue_pos_cm"][axis]))
        rot = euler_xyz_degrees(
            basis_from_tangent_up(frame["ue_tan"], frame["ue_up"])
        )
        for axis in range(3):
            rotations[axis].append(rot[axis])
    for channel in rotations:
        unwrap_degrees(channel)

    model_id, geom_id = 1000000, 1100000
    stack_id, layer_id = 2000000, 2100000
    node_t, node_r = 3000000, 3100000
    curves_t = [4000000, 4000001, 4000002]
    curves_r = [4100000, 4100001, 4100002]

    first_pos = [translations[a][0] for a in range(3)]
    first_rot = [rotations[a][0] for a in range(3)]
    stop_ticks = times_ticks[-1]

    roots: List[Node] = []

    header = Node("FBXHeaderExtension")
    header.add(Node("FBXHeaderVersion", [_i(1003)]))
    header.add(Node("FBXVersion", [_i(FBX_VERSION)]))
    header.add(Node("EncryptionType", [_i(0)]))
    header.add(Node("Creator", [_s(creator)]))
    roots.append(header)
    roots.append(Node("Creator", [_s(creator)]))

    settings = Node("GlobalSettings")
    settings.add(Node("Version", [_i(1000)]))
    props = settings.add(Node("Properties70"))
    # Z-up, X-forward, centimetres: Unreal's own convention, declared so the
    # importer has no conversion to perform.
    for name, value in (
        ("UpAxis", 2), ("UpAxisSign", 1),
        ("FrontAxis", 1), ("FrontAxisSign", -1),
        ("CoordAxis", 0), ("CoordAxisSign", 1),
        ("OriginalUpAxis", 2), ("OriginalUpAxisSign", 1),
    ):
        props.add(_prop70(name, "int", "Integer", "", _i(value)))
    props.add(_prop70("UnitScaleFactor", "double", "Number", "", _d(1.0)))
    props.add(_prop70("OriginalUnitScaleFactor", "double", "Number", "", _d(1.0)))
    props.add(
        _prop70("TimeMode", "enum", "", "", _i(TIME_MODES.get(fps, TIME_MODE_CUSTOM)))
    )
    props.add(_prop70("TimeSpanStart", "KTime", "Time", "", _l(0)))
    props.add(_prop70("TimeSpanStop", "KTime", "Time", "", _l(stop_ticks)))
    props.add(_prop70("CustomFrameRate", "double", "Number", "", _d(float(fps))))
    roots.append(settings)

    documents = Node("Documents")
    documents.add(Node("Count", [_i(1)]))
    doc = documents.add(
        Node("Document", [_l(1), _s(""), _s("Scene")])
    )
    doc.add(Node("RootNode", [_i(0)]))
    roots.append(documents)
    roots.append(Node("References"))

    definitions = Node("Definitions")
    definitions.add(Node("Version", [_i(100)]))
    definitions.add(Node("Count", [_i(13)]))
    for type_name, count in (
        ("GlobalSettings", 1), ("Model", 1), ("Geometry", 1),
        ("AnimationStack", 1), ("AnimationLayer", 1),
        ("AnimationCurveNode", 2), ("AnimationCurve", 6),
    ):
        entry = definitions.add(Node("ObjectType", [_s(type_name)]))
        entry.add(Node("Count", [_i(count)]))
    roots.append(definitions)

    objects = Node("Objects")

    model = objects.add(
        Node("Model", [_l(model_id), _s(obj_name(node_name, "Model")), _s("Mesh")])
    )
    model.add(Node("Version", [_i(232)]))
    mprops = model.add(Node("Properties70"))
    mprops.add(
        _prop70("Lcl Translation", "Lcl Translation", "", "A+", *[_d(v) for v in first_pos])
    )
    mprops.add(
        _prop70("Lcl Rotation", "Lcl Rotation", "", "A+", *[_d(v) for v in first_rot])
    )
    mprops.add(
        _prop70("Lcl Scaling", "Lcl Scaling", "", "A+", _d(1.0), _d(1.0), _d(1.0))
    )
    mprops.add(_prop70("DefaultAttributeIndex", "int", "Integer", "", _i(0)))
    mprops.add(_prop70("InheritType", "enum", "", "", _i(1)))
    model.add(Node("MultiLayer", [_i(0)]))
    model.add(Node("MultiTake", [_i(0)]))
    model.add(Node("Shading", [_b(True)]))
    model.add(Node("Culling", [_s("CullingOff")]))

    objects.add(_box_geometry(geom_id, box_size_cm))

    stack = objects.add(
        Node("AnimationStack", [_l(stack_id), _s(obj_name("CoasterRide", "AnimStack")), _s("")])
    )
    sprops = stack.add(Node("Properties70"))
    sprops.add(_prop70("LocalStop", "KTime", "Time", "", _l(stop_ticks)))
    sprops.add(_prop70("ReferenceStop", "KTime", "Time", "", _l(stop_ticks)))

    objects.add(
        Node("AnimationLayer", [_l(layer_id), _s(obj_name("BaseLayer", "AnimLayer")), _s("")])
    )
    objects.add(_curve_node(node_t, "T", first_pos))
    objects.add(_curve_node(node_r, "R", first_rot))
    for axis in range(3):
        objects.add(_curve(curves_t[axis], times_ticks, translations[axis]))
    for axis in range(3):
        objects.add(_curve(curves_r[axis], times_ticks, rotations[axis]))
    roots.append(objects)

    connections = Node("Connections")
    connections.add(Node("C", [_s("OO"), _l(model_id), _l(0)]))
    connections.add(Node("C", [_s("OO"), _l(geom_id), _l(model_id)]))
    connections.add(Node("C", [_s("OO"), _l(layer_id), _l(stack_id)]))
    for curve_node, prop, curve_ids in (
        (node_t, "Lcl Translation", curves_t),
        (node_r, "Lcl Rotation", curves_r),
    ):
        connections.add(Node("C", [_s("OO"), _l(curve_node), _l(layer_id)]))
        connections.add(Node("C", [_s("OP"), _l(curve_node), _l(model_id), _s(prop)]))
        for axis, letter in enumerate("XYZ"):
            connections.add(
                Node("C", [_s("OP"), _l(curve_ids[axis]), _l(curve_node), _s(f"d|{letter}")])
            )
    roots.append(connections)

    takes = Node("Takes")
    takes.add(Node("Current", [_s("")]))
    roots.append(takes)

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(serialize_document(roots))

    return {
        "path": str(path),
        "fps": fps,
        "frames": len(frames),
        "duration_s": frames[-1]["time_s"],
        "node_name": node_name,
        "size_bytes": path.stat().st_size,
    }
