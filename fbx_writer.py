"""
Minimal binary FBX 7.4 writer.

Shared plumbing for the exporters. Binary rather than ASCII because most
readers - Blender included - refuse ASCII FBX outright, so an ASCII file could
not be verified.

Every scene written here declares itself Z-up in centimetres, which is Unreal's
own convention, so the importer has no axis or unit conversion to perform.

The record layout, the name encoding and the key attribute payload were all
taken from a reference file produced by a known-good exporter rather than
guessed; see the notes on obj_name() and KEY_ATTR_DATA.
"""

from __future__ import annotations

import struct
from typing import Dict, List, Sequence

FBX_VERSION = 7400

# FBX stores time as an integer count; this many units per second.
FBX_TIME_ONE_SECOND = 46186158000

# Tangent payload accompanying animation keys, matching byte for byte what
# Autodesk's own exporter writes.
#
# Keys are described in blocks: each KeyAttrFlags entry pairs with a 4-float
# group in KeyAttrDataFloat, and KeyAttrRefCount says how many consecutive keys
# that block covers. The counts must sum to the number of keys.
#
# KeyVer must be 4009. At 4008 the FBX SDK reads the tangent block with the
# older layout and discards the curve, which is why a file with complete,
# correct curves imported into Unreal as a skeletal mesh and no AnimSequence -
# the animation was there, and the reader was throwing it away.
KEY_VER = 4009
KEY_FLAG_LINEAR = 0x2104    # linear interpolation, auto tangents
KEY_FLAG_CONSTANT = 0x2002  # the last key has nothing to interpolate towards
# Default weights and velocities. The third entry is a float whose *bit pattern*
# is this integer, not the integer's value.
KEY_ATTR_BLOCK = (
    0.0,
    0.0,
    struct.unpack("<f", struct.pack("<I", 0x0D050D05))[0],
    0.0,
)


def key_attr_blocks(key_count: int):
    """(flags, refcounts, tangent data) for a curve of key_count keys."""
    if key_count > 1:
        flags = [KEY_FLAG_LINEAR, KEY_FLAG_CONSTANT]
        refs = [key_count - 1, 1]
    else:
        flags = [KEY_FLAG_CONSTANT]
        refs = [max(key_count, 1)]
    return flags, refs, KEY_ATTR_BLOCK * len(flags)

HEADER_MAGIC = b"Kaydara FBX Binary  " + bytes([0x00, 0x1A, 0x00])
FOOTER_ID = bytes.fromhex("fabcab09d0c8d466b176fb831cf7267e")
FOOTER_EXT = bytes.fromhex("f85a8c6adef5d97eece90ce3758f290b")

# FbxTime::EMode values for the rates worth naming; anything else goes custom.
TIME_MODES = {120: 1, 100: 2, 60: 3, 50: 4, 48: 5, 30: 6, 24: 11, 1000: 12}
TIME_MODE_CUSTOM = 14


# --------------------------------------------------------------------------
# records
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
        payload = struct.pack("<" + _ARRAY_FMT[code] * len(value), *value)
        # Encoding 0 is uncompressed. Deflate would be smaller but adds a
        # failure mode for no real benefit at these sizes.
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
# standard scene scaffolding
# --------------------------------------------------------------------------

def prop70(name, type_a, type_b, flags, *values) -> Node:
    return Node("P", [_s(name), _s(type_a), _s(type_b), _s(flags), *values])


# A fixed identifier and timestamp keep output byte-reproducible. The values are
# not meaningful to any consumer; only their presence is.
_FILE_ID = bytes.fromhex("28b32aebb624ccc2bfc8b02aa92bfcf1")
_CREATION_TIME = "1970-01-01 10:00:00:000"


def header_nodes(creator: str) -> List[Node]:
    """The document preamble.

    Autodesk's own FBX SDK - which Unreal's Interchange importer uses - refuses
    to open a file that omits any of this, where more forgiving readers such as
    Blender's do not care. So the whole preamble is written even though most of
    it carries no information: FileId, CreationTime, CreationTimeStamp and
    SceneInfo are all required in practice.
    """
    header = Node("FBXHeaderExtension")
    header.add(Node("FBXHeaderVersion", [_i(1003)]))
    header.add(Node("FBXVersion", [_i(FBX_VERSION)]))
    header.add(Node("EncryptionType", [_i(0)]))

    stamp = header.add(Node("CreationTimeStamp"))
    stamp.add(Node("Version", [_i(1000)]))
    for field, value in (
        ("Year", 1970), ("Month", 1), ("Day", 1),
        ("Hour", 10), ("Minute", 0), ("Second", 0), ("Millisecond", 0),
    ):
        stamp.add(Node(field, [_i(value)]))

    header.add(Node("Creator", [_s(creator)]))

    scene_info = header.add(
        Node("SceneInfo", [_s(obj_name("GlobalInfo", "SceneInfo")), _s("UserData")])
    )
    scene_info.add(Node("Type", [_s("UserData")]))
    scene_info.add(Node("Version", [_i(100)]))
    meta = scene_info.add(Node("MetaData"))
    meta.add(Node("Version", [_i(100)]))
    for field in ("Title", "Subject", "Author", "Keywords", "Revision", "Comment"):
        meta.add(Node(field, [_s("")]))
    info_props = scene_info.add(Node("Properties70"))
    info_props.add(prop70("DocumentUrl", "KString", "Url", "", _s("/foobar.fbx")))
    info_props.add(prop70("SrcDocumentUrl", "KString", "Url", "", _s("/foobar.fbx")))
    info_props.add(prop70("Original", "Compound", "", ""))
    info_props.add(
        prop70("Original|ApplicationVendor", "KString", "", "", _s("LSU"))
    )
    info_props.add(
        prop70("Original|ApplicationName", "KString", "", "", _s(creator))
    )
    info_props.add(prop70("LastSaved", "Compound", "", ""))
    info_props.add(
        prop70("LastSaved|ApplicationVendor", "KString", "", "", _s("LSU"))
    )
    info_props.add(
        prop70("LastSaved|ApplicationName", "KString", "", "", _s(creator))
    )

    return [
        header,
        Node("FileId", [("R", _FILE_ID)]),
        Node("CreationTime", [_s(_CREATION_TIME)]),
        Node("Creator", [_s(creator)]),
    ]


def global_settings(fps: float | None = None, stop_ticks: int = 0) -> Node:
    settings = Node("GlobalSettings")
    settings.add(Node("Version", [_i(1000)]))
    props = settings.add(Node("Properties70"))
    # Z-up, X-forward, centimetres: Unreal's own convention.
    for name, value in (
        ("UpAxis", 2), ("UpAxisSign", 1),
        ("FrontAxis", 1), ("FrontAxisSign", -1),
        ("CoordAxis", 0), ("CoordAxisSign", 1),
        ("OriginalUpAxis", 2), ("OriginalUpAxisSign", 1),
    ):
        props.add(prop70(name, "int", "Integer", "", _i(value)))
    props.add(prop70("UnitScaleFactor", "double", "Number", "", _d(1.0)))
    props.add(prop70("OriginalUnitScaleFactor", "double", "Number", "", _d(1.0)))
    if fps is not None:
        rate = int(round(fps))
        props.add(
            prop70("TimeMode", "enum", "", "", _i(TIME_MODES.get(rate, TIME_MODE_CUSTOM)))
        )
        props.add(prop70("TimeSpanStart", "KTime", "Time", "", _l(0)))
        props.add(prop70("TimeSpanStop", "KTime", "Time", "", _l(stop_ticks)))
        props.add(prop70("CustomFrameRate", "double", "Number", "", _d(float(fps))))
    return settings


def document_nodes() -> List[Node]:
    """The scene document.

    Two details matter to the FBX SDK and were wrong at first: the document has
    to be named ("Scene", not empty), and RootNode is an int64, not an int32. A
    reader that indexes by type rather than validating will not notice either.
    """
    documents = Node("Documents")
    documents.add(Node("Count", [_i(1)]))
    doc = documents.add(
        Node("Document", [_l(626207883), _s("Scene"), _s("Scene")])
    )
    props = doc.add(Node("Properties70"))
    props.add(prop70("SourceObject", "object", "", ""))
    props.add(prop70("ActiveAnimStackName", "KString", "", "", _s("")))
    doc.add(Node("RootNode", [_l(0)]))
    return [documents, Node("References")]


def definitions_node(counts: Dict[str, int]) -> Node:
    definitions = Node("Definitions")
    definitions.add(Node("Version", [_i(100)]))
    definitions.add(Node("Count", [_i(sum(counts.values()) + 1)]))
    entry = definitions.add(Node("ObjectType", [_s("GlobalSettings")]))
    entry.add(Node("Count", [_i(1)]))
    for type_name, count in counts.items():
        entry = definitions.add(Node("ObjectType", [_s(type_name)]))
        entry.add(Node("Count", [_i(count)]))
    return definitions


def takes_node(take_name: str = "", stop_ticks: int = 0) -> Node:
    """The legacy take table.

    FBX 7 keeps animation in AnimationStack objects, but Autodesk's SDK still
    enumerates takes from here, and Unreal follows it: with no Take entry the
    import yields a Skeleton and a SkeletalMesh but no AnimSequence at all.
    """
    takes = Node("Takes")
    takes.add(Node("Current", [_s(take_name)]))
    if take_name:
        take = takes.add(Node("Take", [_s(take_name)]))
        take.add(Node("FileName", [_s(f"{take_name}.tak")]))
        take.add(Node("LocalTime", [_l(0), _l(stop_ticks)]))
        take.add(Node("ReferenceTime", [_l(0), _l(stop_ticks)]))
    return takes


def mesh_model_node(object_id: int, name: str) -> Node:
    """A plain static mesh node sitting at the origin.

    Track geometry is written in absolute Unreal coordinates, so the node
    carries an identity transform and the mesh lands exactly where the
    converter put it.
    """
    model = Node("Model", [_l(object_id), _s(obj_name(name, "Model")), _s("Mesh")])
    model.add(Node("Version", [_i(232)]))
    props = model.add(Node("Properties70"))
    props.add(prop70("Lcl Translation", "Lcl Translation", "", "A+", _d(0), _d(0), _d(0)))
    props.add(prop70("Lcl Rotation", "Lcl Rotation", "", "A+", _d(0), _d(0), _d(0)))
    props.add(prop70("Lcl Scaling", "Lcl Scaling", "", "A+", _d(1), _d(1), _d(1)))
    props.add(prop70("DefaultAttributeIndex", "int", "Integer", "", _i(0)))
    props.add(prop70("InheritType", "enum", "", "", _i(1)))
    model.add(Node("MultiLayer", [_i(0)]))
    model.add(Node("MultiTake", [_i(0)]))
    model.add(Node("Shading", [_b(True)]))
    model.add(Node("Culling", [_s("CullingOff")]))
    return model


# --------------------------------------------------------------------------
# mesh building
# --------------------------------------------------------------------------

def triangle_geometry(
    object_id: int,
    name: str,
    positions: Sequence[float],
    normals: Sequence[float],
    uvs: Sequence[float],
    indices: Sequence[int],
) -> Node:
    """A mesh from flat triangle arrays, with per-corner normals and UVs.

    Vertices stay shared, and the normals supplied per corner are what give the
    surface its shading, so a mesh imported from elsewhere keeps the smoothing it
    was authored with.
    """
    poly_indices: List[int] = []
    corner_normals: List[float] = []
    corner_uvs: List[float] = []

    for t in range(0, len(indices) - 2, 3):
        tri = (indices[t], indices[t + 1], indices[t + 2])
        # FBX marks a polygon's last index by bitwise negation.
        poly_indices.extend((tri[0], tri[1], ~tri[2]))
        for v in tri:
            corner_normals.extend(normals[v * 3:v * 3 + 3])
            corner_uvs.extend(uvs[v * 2:v * 2 + 2])

    geom = Node(
        "Geometry", [_l(object_id), _s(obj_name(name, "Geometry")), _s("Mesh")]
    )
    geom.add(Node("GeometryVersion", [_i(124)]))
    geom.add(Node("Vertices", [_arr("d", list(positions))]))
    geom.add(Node("PolygonVertexIndex", [_arr("i", poly_indices)]))

    layer = geom.add(Node("LayerElementNormal", [_i(0)]))
    layer.add(Node("Version", [_i(102)]))
    layer.add(Node("Name", [_s("")]))
    layer.add(Node("MappingInformationType", [_s("ByPolygonVertex")]))
    layer.add(Node("ReferenceInformationType", [_s("Direct")]))
    layer.add(Node("Normals", [_arr("d", corner_normals)]))

    uv_layer = geom.add(Node("LayerElementUV", [_i(0)]))
    uv_layer.add(Node("Version", [_i(101)]))
    uv_layer.add(Node("Name", [_s("UVMap")]))
    uv_layer.add(Node("MappingInformationType", [_s("ByPolygonVertex")]))
    uv_layer.add(Node("ReferenceInformationType", [_s("Direct")]))
    uv_layer.add(Node("UV", [_arr("d", corner_uvs)]))

    layer_node = geom.add(Node("Layer", [_i(0)]))
    layer_node.add(Node("Version", [_i(100)]))
    for element in ("LayerElementNormal", "LayerElementUV"):
        entry = layer_node.add(Node("LayerElement"))
        entry.add(Node("Type", [_s(element)]))
        entry.add(Node("TypedIndex", [_i(0)]))
    return geom


class MeshBuilder:
    """Accumulates polygons with per-corner normals and UVs.

    Normals are stored per polygon-vertex rather than per polygon so a swept
    tube can carry its true radial normals and shade as a smooth cylinder
    instead of a faceted prism.
    """

    def __init__(self):
        self.verts: List[float] = []
        self.indices: List[int] = []
        self.normals: List[float] = []
        self.uvs: List[float] = []
        self._count = 0

    def add_vertex(self, p: Sequence[float]) -> int:
        self.verts.extend((float(p[0]), float(p[1]), float(p[2])))
        self._count += 1
        return self._count - 1

    @property
    def vertex_count(self) -> int:
        return self._count

    @property
    def polygon_count(self) -> int:
        # Every polygon contributes exactly one negated terminal index.
        return sum(1 for i in self.indices if i < 0)

    def add_polygon(self, loop, normals, uvs) -> None:
        for k, index in enumerate(loop):
            # FBX marks a polygon's last index by bitwise negation.
            self.indices.append(~index if k == len(loop) - 1 else index)
        for n in normals:
            self.normals.extend((float(n[0]), float(n[1]), float(n[2])))
        for uv in uvs:
            self.uvs.extend((float(uv[0]), float(uv[1])))

    def is_empty(self) -> bool:
        return not self.indices

    def geometry(self, object_id: int, name: str) -> Node:
        geom = Node(
            "Geometry", [_l(object_id), _s(obj_name(name, "Geometry")), _s("Mesh")]
        )
        geom.add(Node("GeometryVersion", [_i(124)]))
        geom.add(Node("Vertices", [_arr("d", self.verts)]))
        geom.add(Node("PolygonVertexIndex", [_arr("i", self.indices)]))

        layer = geom.add(Node("LayerElementNormal", [_i(0)]))
        layer.add(Node("Version", [_i(102)]))
        layer.add(Node("Name", [_s("")]))
        layer.add(Node("MappingInformationType", [_s("ByPolygonVertex")]))
        layer.add(Node("ReferenceInformationType", [_s("Direct")]))
        layer.add(Node("Normals", [_arr("d", self.normals)]))

        uv_layer = geom.add(Node("LayerElementUV", [_i(0)]))
        uv_layer.add(Node("Version", [_i(101)]))
        uv_layer.add(Node("Name", [_s("UVMap")]))
        uv_layer.add(Node("MappingInformationType", [_s("ByPolygonVertex")]))
        uv_layer.add(Node("ReferenceInformationType", [_s("Direct")]))
        uv_layer.add(Node("UV", [_arr("d", self.uvs)]))

        # Some importers only look for layer elements listed here.
        layer_node = geom.add(Node("Layer", [_i(0)]))
        layer_node.add(Node("Version", [_i(100)]))
        for element in ("LayerElementNormal", "LayerElementUV"):
            entry = layer_node.add(Node("LayerElement"))
            entry.add(Node("Type", [_s(element)]))
            entry.add(Node("TypedIndex", [_i(0)]))
        return geom
