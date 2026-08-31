"""
Build a coaster track mesh from the converted path and write it as an FBX.

This is procedural geometry, not a rip of the original model: rails, a spine,
crossties and support columns swept along the path the converter produced. It
lands in Unreal at the same absolute coordinates as the ride, so the car runs on
it without any alignment step.

Four separate meshes are written - rails, spine, ties, supports - so each can
take its own material in Unreal rather than arriving as one unassignable blob.

Geometry comes from the bundle's render_path, which is the spike-filtered copy.
The analytic path used for the physics is never touched by any of this.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, List, Sequence

from fbx_writer import (
    MeshBuilder,
    Node,
    definitions_node,
    document_nodes,
    global_settings,
    header_nodes,
    mesh_model_node,
    serialize_document,
    takes_node,
    _l,
    _s,
    obj_name,
)


def _norm(v):
    length = math.sqrt(sum(c * c for c in v))
    return [c / length for c in v] if length > 1e-12 else [0.0, 0.0, 1.0]


def _cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _add(a, b, scale=1.0):
    return [a[i] + b[i] * scale for i in range(3)]


def _dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


class Station:
    """One cross-section frame along the track, in Unreal centimetres."""

    __slots__ = ("pos", "fwd", "right", "up", "distance")

    def __init__(self, pos, fwd, up, distance):
        self.pos = pos
        self.fwd = _norm(fwd)
        up_n = _norm(up)
        # Unreal is left-handed with X forward and Z up, so right = up x forward.
        self.right = _norm(_cross(up_n, self.fwd))
        self.up = _norm(_cross(self.fwd, self.right))
        self.distance = distance


def build_stations(render_path: List[Dict], spacing_cm: float) -> List[Station]:
    """Resample the path into evenly spaced cross-section frames.

    Sweeping a ring at every 10cm sample would be tens of thousands of rings for
    no visible gain, so the path is walked at a chosen spacing instead.
    """
    points = [s["ue_pos_cm"] for s in render_path]
    ups = [s.get("ue_up") or [0.0, 0.0, 1.0] for s in render_path]
    if len(points) < 2:
        raise ValueError("render_path needs at least two points")

    cumulative = [0.0]
    for i in range(1, len(points)):
        cumulative.append(cumulative[-1] + _dist(points[i], points[i - 1]))
    total = cumulative[-1]

    spacing = max(float(spacing_cm), 1.0)
    count = max(int(round(total / spacing)) + 1, 2)

    stations: List[Station] = []
    cursor = 0
    for step in range(count):
        target = total * step / (count - 1)
        while cursor < len(cumulative) - 2 and cumulative[cursor + 1] < target:
            cursor += 1

        span = cumulative[cursor + 1] - cumulative[cursor]
        alpha = 0.0 if span <= 1e-9 else (target - cumulative[cursor]) / span

        pos = [
            points[cursor][k] + (points[cursor + 1][k] - points[cursor][k]) * alpha
            for k in range(3)
        ]
        up = [
            ups[cursor][k] + (ups[cursor + 1][k] - ups[cursor][k]) * alpha
            for k in range(3)
        ]

        nxt = points[min(cursor + 1, len(points) - 1)]
        prv = points[cursor]
        fwd = [nxt[k] - prv[k] for k in range(3)]
        if _dist(nxt, prv) < 1e-9 and stations:
            fwd = stations[-1].fwd

        stations.append(Station(pos, fwd, up, target))
    return stations


def sweep_tube(
    mesh: MeshBuilder,
    stations: Sequence[Station],
    radius: float,
    sides: int,
    offset_right: float = 0.0,
    offset_up: float = 0.0,
    uv_metres_per_tile: float = 1.0,
) -> None:
    """Sweep a closed tube of `radius` along the stations, offset from the path."""
    if len(stations) < 2 or radius <= 0.0:
        return

    sides = max(int(sides), 3)
    ring_starts = []
    ring_normals = []

    for station in stations:
        centre = _add(_add(station.pos, station.right, offset_right),
                      station.up, offset_up)
        start = mesh.vertex_count
        normals = []
        for side in range(sides):
            angle = 2.0 * math.pi * side / sides
            radial = _add(
                [station.right[k] * math.cos(angle) for k in range(3)],
                station.up,
                math.sin(angle),
            )
            radial = _norm(radial)
            mesh.add_vertex(_add(centre, radial, radius))
            normals.append(radial)
        ring_starts.append(start)
        ring_normals.append(normals)

    tile = max(uv_metres_per_tile, 1e-6) * 100.0
    for i in range(1, len(stations)):
        a, b = ring_starts[i - 1], ring_starts[i]
        v0 = stations[i - 1].distance / tile
        v1 = stations[i].distance / tile
        for side in range(sides):
            nxt = (side + 1) % sides
            loop = (a + side, a + nxt, b + nxt, b + side)
            normals = (
                ring_normals[i - 1][side], ring_normals[i - 1][nxt],
                ring_normals[i][nxt], ring_normals[i][side],
            )
            u0 = side / float(sides)
            u1 = (side + 1) / float(sides)
            mesh.add_polygon(loop, normals, ((u0, v0), (u1, v0), (u1, v1), (u0, v1)))


# Each face lists its outward axis, that axis's sign, and its four corners as
# (fwd, right, up) signs. Faces do not share vertices: Unreal warns that these
# meshes carry no smoothing groups, and if it responds by computing normals
# rather than importing the supplied ones, shared corners would smooth a tie
# into something rounded. Independent faces are flat either way.
_BOX_FACES = (
    ("up", -1, ((-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1))),
    ("up", +1, ((-1, 1, 1), (1, 1, 1), (1, -1, 1), (-1, -1, 1))),
    ("fwd", -1, ((-1, -1, -1), (-1, 1, -1), (-1, 1, 1), (-1, -1, 1))),
    ("fwd", +1, ((1, -1, -1), (1, -1, 1), (1, 1, 1), (1, 1, -1))),
    ("right", -1, ((-1, -1, -1), (-1, -1, 1), (1, -1, 1), (1, -1, -1))),
    ("right", +1, ((-1, 1, -1), (1, 1, -1), (1, 1, 1), (-1, 1, 1))),
)

_BOX_UVS = ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0))


def add_box(mesh: MeshBuilder, centre, right, up, fwd, half) -> None:
    """A box aligned to a station frame; used for ties and support footings."""
    hx, hy, hz = half
    axes = {"fwd": fwd, "right": right, "up": up}

    for axis, sign, corners in _BOX_FACES:
        normal = _norm([c * sign for c in axes[axis]])
        loop = []
        for sx, sy, sz in corners:
            point = _add(
                _add(_add(centre, fwd, sx * hx), right, sy * hy), up, sz * hz
            )
            loop.append(mesh.add_vertex(point))
        mesh.add_polygon(loop, (normal,) * 4, _BOX_UVS)


def add_ties(
    mesh: MeshBuilder,
    stations: Sequence[Station],
    spacing_cm: float,
    gauge_cm: float,
    rail_drop_cm: float,
    spine_drop_cm: float,
    thickness_cm: float,
) -> int:
    """Cross members from the spine out to both rails, at a fixed spacing."""
    if not stations:
        return 0

    step = max(float(spacing_cm), 10.0)
    placed = 0
    next_at = 0.0
    half_thick = max(thickness_cm, 1.0) * 0.5

    for station in stations:
        if station.distance < next_at:
            continue
        next_at = station.distance + step

        rail_centre = _add(station.pos, station.up, -rail_drop_cm)
        spine_centre = _add(station.pos, station.up, -spine_drop_cm)

        # Horizontal bar spanning the gauge at rail height.
        add_box(
            mesh, rail_centre, station.right, station.up, station.fwd,
            (half_thick, gauge_cm * 0.5 + half_thick, half_thick),
        )
        # Vertical web joining that bar down to the spine.
        mid = [(rail_centre[k] + spine_centre[k]) * 0.5 for k in range(3)]
        drop = abs(spine_drop_cm - rail_drop_cm)
        add_box(
            mesh, mid, station.right, station.up, station.fwd,
            (half_thick, half_thick, max(drop * 0.5, half_thick)),
        )
        placed += 1
    return placed


def add_supports(
    mesh: MeshBuilder,
    stations: Sequence[Station],
    spacing_cm: float,
    radius_cm: float,
    sides: int,
    spine_drop_cm: float,
    ground_z_cm: float,
    min_height_cm: float,
    slenderness: float = 35.0,
) -> int:
    """Vertical columns from the spine down to ground level.

    Deliberately simple: a straight column wherever the track is high enough to
    need one. Real supports are angled bents chosen per element, so these are a
    stand-in for massing and shadow rather than an engineering claim. They can
    intersect track that passes underneath.

    Column thickness scales with height. A fixed radius makes a 44m column read
    as a wire - height over diameter around 160, where real coaster columns sit
    nearer 35 - so the radius is whichever is larger of the floor and the radius
    that hits the target ratio.
    """
    if not stations:
        return 0

    step = max(float(spacing_cm), 50.0)
    placed = 0
    next_at = 0.0
    sides = max(int(sides), 3)

    for station in stations:
        if station.distance < next_at:
            continue

        top = _add(station.pos, station.up, -spine_drop_cm)
        height = top[2] - ground_z_cm
        if height < max(min_height_cm, 1.0):
            continue
        next_at = station.distance + step

        column_radius = max(
            radius_cm, height / (2.0 * max(slenderness, 1.0))
        )

        # A column is a plain vertical prism, so it is built directly rather
        # than swept along the (banked) station frame.
        base = [top[0], top[1], ground_z_cm]
        ring_top, ring_base, normals = [], [], []
        for side in range(sides):
            angle = 2.0 * math.pi * side / sides
            radial = [math.cos(angle), math.sin(angle), 0.0]
            ring_top.append(mesh.add_vertex(_add(top, radial, column_radius)))
            ring_base.append(mesh.add_vertex(_add(base, radial, column_radius)))
            normals.append(radial)

        v_top = height / 100.0
        for side in range(sides):
            nxt = (side + 1) % sides
            loop = (ring_base[side], ring_base[nxt], ring_top[nxt], ring_top[side])
            u0 = side / float(sides)
            u1 = (side + 1) / float(sides)
            mesh.add_polygon(
                loop,
                (normals[side], normals[nxt], normals[nxt], normals[side]),
                ((u0, 0.0), (u1, 0.0), (u1, v_top), (u0, v_top)),
            )
        placed += 1
    return placed


def build_track_parts(
    render_path: List[Dict],
    station_spacing_cm: float = 40.0,
    gauge_cm: float = 100.0,
    rail_drop_cm: float = 110.0,
    rail_radius_cm: float = 5.5,
    spine_drop_cm: float = 145.0,
    spine_radius_cm: float = 12.0,
    tube_sides: int = 8,
    tie_spacing_cm: float = 150.0,
    tie_thickness_cm: float = 7.0,
    supports: bool = True,
    support_spacing_cm: float = 900.0,
    support_radius_cm: float = 12.0,
    support_slenderness: float = 35.0,
    support_min_height_cm: float = 150.0,
    ground_z_cm: float | None = None,
):
    """Generate the track geometry. Returns (parts, stations, counts).

    Split out of write_track_fbx so the glTF writer builds byte-identical
    geometry rather than a second implementation that can drift from it.
    """
    stations = build_stations(render_path, station_spacing_cm)

    if ground_z_cm is None:
        # Below the lowest point of the spine, not the path: with the rails on
        # the path the old formula put the ground above the spine it was meant
        # to hold up, giving the lowest columns a negative length.
        ground_z_cm = min(s.pos[2] for s in stations) - spine_drop_cm - 30.0

    rails = MeshBuilder()
    for side_sign in (-1.0, 1.0):
        sweep_tube(
            rails, stations, rail_radius_cm, tube_sides,
            offset_right=side_sign * gauge_cm * 0.5,
            offset_up=-rail_drop_cm,
        )

    spine = MeshBuilder()
    sweep_tube(
        spine, stations, spine_radius_cm, tube_sides, offset_up=-spine_drop_cm
    )

    ties = MeshBuilder()
    tie_count = add_ties(
        ties, stations, tie_spacing_cm, gauge_cm, rail_drop_cm,
        spine_drop_cm, tie_thickness_cm,
    )

    columns = MeshBuilder()
    support_count = 0
    if supports:
        support_count = add_supports(
            columns, stations, support_spacing_cm, support_radius_cm,
            tube_sides, spine_drop_cm, ground_z_cm, support_min_height_cm,
            support_slenderness,
        )

    parts = [
        ("CoasterTrackRails", rails),
        ("CoasterTrackSpine", spine),
        ("CoasterTrackTies", ties),
        ("CoasterTrackSupports", columns),
    ]
    parts = [(name, mesh) for name, mesh in parts if not mesh.is_empty()]
    if not parts:
        raise ValueError("track generation produced no geometry")
    return parts, stations, tie_count, support_count, ground_z_cm


def write_track_glb(path, render_path: List[Dict], **kwargs) -> Dict:
    """Write the track as a .glb, which is what Unreal should import.

    Unreal's FBX importer mirrors the scene in Y on the way in; its glTF
    importer does not. The car already arrives as glTF, so sending the track the
    same way is what keeps the two in the same place - a mirrored track lines up
    on every span check and still sits nowhere near the car.
    """
    from export_car_glb import mesh_from_builder, write_static_glb

    parts, stations, tie_count, support_count, ground_z_cm = build_track_parts(
        render_path, **kwargs
    )
    info = write_static_glb(path, [(n, mesh_from_builder(m)) for n, m in parts])
    info.update({
        "stations": len(stations),
        "length_m": stations[-1].distance / 100.0,
        "ties": tie_count,
        "supports": support_count,
        "ground_z_cm": ground_z_cm,
    })
    return info


def write_track_fbx(
    path,
    render_path: List[Dict],
    station_spacing_cm: float = 40.0,
    gauge_cm: float = 100.0,
    rail_drop_cm: float = 110.0,
    rail_radius_cm: float = 5.5,
    spine_drop_cm: float = 145.0,
    spine_radius_cm: float = 12.0,
    tube_sides: int = 8,
    tie_spacing_cm: float = 150.0,
    tie_thickness_cm: float = 7.0,
    supports: bool = True,
    support_spacing_cm: float = 900.0,
    support_radius_cm: float = 12.0,
    support_slenderness: float = 35.0,
    support_min_height_cm: float = 150.0,
    ground_z_cm: float | None = None,
    creator: str = "UE5 Coaster Pipeline",
) -> Dict:
    """Generate the track and write it as FBX. Returns a summary dict.

    Note this file imports into Unreal mirrored in Y - use the .glb for Unreal
    and keep this for tools that read FBX correctly.
    """
    parts, stations, tie_count, support_count, ground_z_cm = build_track_parts(
        render_path,
        station_spacing_cm=station_spacing_cm,
        gauge_cm=gauge_cm,
        rail_drop_cm=rail_drop_cm,
        rail_radius_cm=rail_radius_cm,
        spine_drop_cm=spine_drop_cm,
        spine_radius_cm=spine_radius_cm,
        tube_sides=tube_sides,
        tie_spacing_cm=tie_spacing_cm,
        tie_thickness_cm=tie_thickness_cm,
        supports=supports,
        support_spacing_cm=support_spacing_cm,
        support_radius_cm=support_radius_cm,
        support_slenderness=support_slenderness,
        support_min_height_cm=support_min_height_cm,
        ground_z_cm=ground_z_cm,
    )

    roots: List[Node] = []
    roots.extend(header_nodes(creator))
    roots.append(global_settings())
    roots.extend(document_nodes())
    roots.append(
        definitions_node({"Model": len(parts), "Geometry": len(parts)})
    )

    objects = Node("Objects")
    connections = Node("Connections")
    model_id, geom_id = 1000000, 2000000
    for name, mesh in parts:
        objects.add(mesh_model_node(model_id, name))
        objects.add(mesh.geometry(geom_id, name))
        connections.add(Node("C", [_s("OO"), _l(model_id), _l(0)]))
        connections.add(Node("C", [_s("OO"), _l(geom_id), _l(model_id)]))
        model_id += 1
        geom_id += 1
    roots.append(objects)
    roots.append(connections)
    roots.append(takes_node())

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(serialize_document(roots))

    return {
        "path": str(path),
        "stations": len(stations),
        "length_m": stations[-1].distance / 100.0,
        "parts": {name: mesh.polygon_count for name, mesh in parts},
        "vertices": sum(mesh.vertex_count for _, mesh in parts),
        "ties": tie_count,
        "supports": support_count,
        "ground_z_cm": ground_z_cm,
        "size_bytes": path.stat().st_size,
    }
