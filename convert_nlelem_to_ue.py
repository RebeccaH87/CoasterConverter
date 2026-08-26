#!/usr/bin/env python3
"""
Convert OpenFVD/NoLimits .nlelem coaster exports to a UE5-friendly bundle.

Outputs:
- Single JSON bundle with spline, sampled path, and gravity-based motion timeline.
- Optional CSV timeline for quick inspection/import.
"""

from __future__ import annotations

import argparse
import copy
import csv
import shutil
import json
import math
import struct
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple


@dataclass
class NLElemNode:
    kp1: Tuple[float, float, float]
    kp2: Tuple[float, float, float]
    p1: Tuple[float, float, float]
    roll: float
    cont_roll: int
    rel_roll: int
    equal_dist_cp: int


@dataclass
class NLElemData:
    path: Path
    data_length: int
    node_count: int
    nodes: List[NLElemNode]


def parse_nlelem(path: Path) -> NLElemData:
    raw = path.read_bytes()
    if len(raw) < 76:
        raise ValueError(f"{path} is too small to be a valid .nlelem file")

    magic = raw[0:4]
    if magic != b"ELEM":
        raise ValueError(f"{path} has unexpected magic {magic!r}, expected b'ELEM'")

    data_length = struct.unpack(">i", raw[4:8])[0]
    node_count = struct.unpack(">i", raw[72:76])[0]

    expected_data_length = node_count * 50 + 132
    if data_length != expected_data_length:
        raise ValueError(
            f"{path} header mismatch: data_length={data_length}, expected={expected_data_length}"
        )

    required_len = 76 + node_count * 50
    if len(raw) < required_len:
        raise ValueError(
            f"{path} truncated: got {len(raw)} bytes, expected at least {required_len}"
        )

    nodes: List[NLElemNode] = []
    offset = 76
    for _ in range(node_count):
        vals = struct.unpack(">10f", raw[offset : offset + 40])
        kp1 = (vals[0], vals[1], vals[2])
        kp2 = (vals[3], vals[4], vals[5])
        p1 = (vals[6], vals[7], vals[8])
        roll = vals[9]
        cont_roll = raw[offset + 40]
        rel_roll = raw[offset + 41]
        equal_dist_cp = raw[offset + 42]

        nodes.append(
            NLElemNode(
                kp1=kp1,
                kp2=kp2,
                p1=p1,
                roll=roll,
                cont_roll=cont_roll,
                rel_roll=rel_roll,
                equal_dist_cp=equal_dist_cp,
            )
        )
        offset += 50

    return NLElemData(path=path, data_length=data_length, node_count=node_count, nodes=nodes)


def v_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def v_mul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def v_len(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def v_norm(a):
    n = v_len(a)
    if n < 1e-9:
        return (0.0, 0.0, 1.0)
    return (a[0] / n, a[1] / n, a[2] / n)


def v_lerp(a, b, t):
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t)


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def rotate_around_axis(vec, axis, angle_rad):
    # Rodrigues' rotation formula.
    k = v_norm(axis)
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return v_add(
        v_add(v_mul(vec, c), v_mul(cross(k, vec), s)),
        v_mul(k, dot(k, vec) * (1.0 - c)),
    )


def cubic_bezier(p0, p1, p2, p3, t):
    u = 1.0 - t
    uu = u * u
    uuu = uu * u
    tt = t * t
    ttt = tt * t
    return v_add(
        v_add(v_mul(p0, uuu), v_mul(p1, 3.0 * uu * t)),
        v_add(v_mul(p2, 3.0 * u * tt), v_mul(p3, ttt)),
    )


def cubic_bezier_derivative(p0, p1, p2, p3, t):
    u = 1.0 - t
    a = v_mul(v_sub(p1, p0), 3.0 * u * u)
    b = v_mul(v_sub(p2, p1), 6.0 * u * t)
    c = v_mul(v_sub(p3, p2), 3.0 * t * t)
    return v_add(v_add(a, b), c)


M_TO_CM = 100.0

# Axis conversion matrices. Source is X-right, Y-up, Z-forward (NoLimits/FVD);
# Unreal is X-forward, Y-right, Z-up.
#
# The source basis is right-handed and Unreal's is left-handed. Converting a
# real object between bases of opposite handedness requires a net determinant
# of -1. A mapping whose determinant is +1 is a pure rotation, so it cannot
# change handedness -- it silently produces a MIRRORED track: left-hand
# helices come out right-hand and every lateral-G sign flips.
# "nl2_to_ue_swap_yz" was established empirically against a known-good
# UE-space spline export of this same track (Coaster_UE_spline.csv): the
# per-axis bounding-box spans matched to within 0.01m on all three axes with no
# sign flips, and arc-length-aligned RMS was ~9x lower than any alternative.
# Verify with --validate-reference-csv after changing source tooling.
AXIS_MAPPINGS: Dict[str, Tuple[Tuple[float, float, float], ...]] = {
    # (x, y, z) -> (x,  z, y)   det = -1   handedness corrected. VERIFIED.
    "nl2_to_ue_swap_yz": ((1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, 1.0, 0.0)),
    # (x, y, z) -> (z, -x, y)   det = -1   handedness ok, wrong axis assignment
    "nl2_to_ue_flip_y": ((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    # (x, y, z) -> (z,  x, y)   det = +1   mirrored; kept for compatibility
    "nl2_to_ue": ((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    # (x, y, z) -> (x,  y, z)   det = +1   mirrored; debug only
    "identity": ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
}


def mapping_matrix(mapping: str) -> Tuple[Tuple[float, float, float], ...]:
    try:
        return AXIS_MAPPINGS[mapping]
    except KeyError:
        raise ValueError(f"Unsupported mapping: {mapping}") from None


def mapping_determinant(mapping: str) -> float:
    m = mapping_matrix(mapping)
    return (
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    )


def apply_axis_mapping(vec, mapping: str) -> Tuple[float, float, float]:
    m = mapping_matrix(mapping)
    return (
        m[0][0] * vec[0] + m[0][1] * vec[1] + m[0][2] * vec[2],
        m[1][0] * vec[0] + m[1][1] * vec[1] + m[1][2] * vec[2],
        m[2][0] * vec[0] + m[2][1] * vec[1] + m[2][2] * vec[2],
    )


def to_ue_cm(pos_m: Tuple[float, float, float], mapping: str) -> Tuple[float, float, float]:
    """Source position in metres -> Unreal world position in centimetres."""
    return v_mul(apply_axis_mapping(pos_m, mapping), M_TO_CM)


def to_ue_dir(vec: Tuple[float, float, float], mapping: str) -> Tuple[float, float, float]:
    """Source unit direction -> Unreal unit direction. No unit conversion."""
    return v_norm(apply_axis_mapping(vec, mapping))


def build_sampled_path(
    elem: NLElemData,
    samples_per_segment: int,
    axis_mapping: str,
    initial_roll: float,
) -> List[Dict]:
    out = []
    p0 = (0.0, 0.0, 0.0)
    last_roll = initial_roll
    idx = 0

    for seg_i, node in enumerate(elem.nodes):
        c1, c2, p3 = node.kp1, node.kp2, node.p1
        for i in range(samples_per_segment + (0 if seg_i else 1)):
            if seg_i > 0 and i == 0:
                continue
            t = i / float(samples_per_segment)
            pos = cubic_bezier(p0, c1, c2, p3, t)
            deriv = cubic_bezier_derivative(p0, c1, c2, p3, t)
            tan = v_norm(deriv)

            roll = last_roll + (node.roll - last_roll) * t

            world_up = (0.0, 1.0, 0.0)
            right = v_norm(cross(tan, world_up))
            if v_len(right) < 1e-6:
                right = (1.0, 0.0, 0.0)
            up = v_norm(cross(right, tan))
            up = rotate_around_axis(up, tan, roll)

            out.append(
                {
                    "index": idx,
                    "segment": seg_i,
                    "t": t,
                    "pos_m": [pos[0], pos[1], pos[2]],
                    "tan": [tan[0], tan[1], tan[2]],
                    "up": [up[0], up[1], up[2]],
                    "roll_rad": roll,
                    "ue_pos_cm": list(to_ue_cm(pos, axis_mapping)),
                    "ue_tan_cm": list(v_mul(to_ue_cm(tan, axis_mapping), 1.0)),
                    "ue_tan": list(to_ue_dir(tan, axis_mapping)),
                    "ue_up": list(to_ue_dir(up, axis_mapping)),
                }
            )
            idx += 1

        p0 = p3
        last_roll = node.roll

    return out


def recompute_samples_orientation(samples: List[Dict], axis_mapping: str) -> None:
    if len(samples) < 2:
        return

    for i in range(len(samples)):
        p = tuple(samples[i]["pos_m"])
        if i == 0:
            nxt = tuple(samples[i + 1]["pos_m"])
            tan = v_norm(v_sub(nxt, p))
        elif i == len(samples) - 1:
            prev = tuple(samples[i - 1]["pos_m"])
            tan = v_norm(v_sub(p, prev))
        else:
            prev = tuple(samples[i - 1]["pos_m"])
            nxt = tuple(samples[i + 1]["pos_m"])
            tan = v_norm(v_sub(nxt, prev))

        roll = float(samples[i]["roll_rad"])
        world_up = (0.0, 1.0, 0.0)
        right = v_norm(cross(tan, world_up))
        if v_len(right) < 1e-6:
            right = (1.0, 0.0, 0.0)
        up = v_norm(cross(right, tan))
        up = rotate_around_axis(up, tan, roll)

        samples[i]["tan"] = [tan[0], tan[1], tan[2]]
        samples[i]["up"] = [up[0], up[1], up[2]]
        samples[i]["ue_pos_cm"] = list(to_ue_cm(p, axis_mapping))
        samples[i]["ue_tan_cm"] = list(v_mul(to_ue_cm(tan, axis_mapping), 1.0))
        samples[i]["ue_tan"] = list(to_ue_dir(tan, axis_mapping))
        samples[i]["ue_up"] = list(to_ue_dir(up, axis_mapping))


def smooth_extreme_spikes(
    samples: List[Dict],
    axis_mapping: str,
    angle_threshold_deg: float = 70.0,
    deviation_multiplier: float = 0.25,
    max_passes: int = 5,
) -> int:
    if len(samples) < 5:
        return 0

    total_changes = 0
    cos_threshold = math.cos(math.radians(angle_threshold_deg))

    for _ in range(max(1, int(max_passes))):
        seg_lengths = []
        for i in range(1, len(samples)):
            a = tuple(samples[i - 1]["pos_m"])
            b = tuple(samples[i]["pos_m"])
            seg_lengths.append(v_len(v_sub(b, a)))
        avg_seg = max(sum(seg_lengths) / max(len(seg_lengths), 1), 1e-6)

        pass_changes = 0
        for i in range(1, len(samples) - 1):
            p_prev = tuple(samples[i - 1]["pos_m"])
            p_curr = tuple(samples[i]["pos_m"])
            p_next = tuple(samples[i + 1]["pos_m"])

            v_in = v_sub(p_curr, p_prev)
            v_out = v_sub(p_next, p_curr)
            d_in = v_len(v_in)
            d_out = v_len(v_out)
            if d_in < 1e-6 or d_out < 1e-6:
                continue

            u_in = v_norm(v_in)
            u_out = v_norm(v_out)
            turn_dot = dot(u_in, u_out)
            if turn_dot > cos_threshold:
                continue

            # Needle-kink guard: if one leg is tiny and the local direction
            # flips, smooth it even if perpendicular deviation is small.
            short_leg = min(d_in, d_out)
            long_leg = max(d_in, d_out)
            force_micro_fix = (
                long_leg > 1e-6
                and short_leg / long_leg < 0.2
                and turn_dot < -0.6
            )

            span = v_sub(p_next, p_prev)
            span_len2 = dot(span, span)
            if span_len2 < 1e-8:
                continue

            rel = v_sub(p_curr, p_prev)
            t = max(0.0, min(1.0, dot(rel, span) / span_len2))
            proj = v_add(p_prev, v_mul(span, t))
            deviation = v_len(v_sub(p_curr, proj))
            if (not force_micro_fix) and deviation < avg_seg * max(deviation_multiplier, 0.05):
                continue

            blend = d_in / (d_in + d_out)
            replacement = v_lerp(p_prev, p_next, blend)
            samples[i]["pos_m"] = [replacement[0], replacement[1], replacement[2]]
            pass_changes += 1

        if pass_changes == 0:
            break
        total_changes += pass_changes

    # Secondary pass: remove short detour-loops where the path doubles back and
    # returns to nearly the same flow direction over a small neighborhood.
    window = 8
    i = window + 2
    while i < len(samples) - window - 2:
        a = i - window
        b = i + window

        p_a = tuple(samples[a]["pos_m"])
        p_b = tuple(samples[b]["pos_m"])

        path_len = 0.0
        for k in range(a + 1, b + 1):
            path_len += v_len(v_sub(tuple(samples[k]["pos_m"]), tuple(samples[k - 1]["pos_m"])))
        chord_len = v_len(v_sub(p_b, p_a))
        if path_len < 1e-6 or chord_len < 1e-6:
            i += 1
            continue

        ratio = chord_len / path_len
        if ratio > 0.45:
            i += 1
            continue

        dir_in = v_norm(v_sub(tuple(samples[a]["pos_m"]), tuple(samples[a - 2]["pos_m"])))
        dir_out = v_norm(v_sub(tuple(samples[b + 2]["pos_m"]), tuple(samples[b]["pos_m"])))
        if dot(dir_in, dir_out) < 0.5:
            i += 1
            continue

        # Replace detour with smooth interpolation between neighborhood endpoints.
        for k in range(a + 1, b):
            t = (k - a) / float(b - a)
            rep = v_lerp(p_a, p_b, t)
            samples[k]["pos_m"] = [rep[0], rep[1], rep[2]]
            total_changes += 1

        i = b + 1

    # Tertiary pass: detect short non-local self-returns (double-back detours).
    # This catches smooth protrusions that bend out and come back near the same line.
    if len(samples) > 64:
        pts = [tuple(s["pos_m"]) for s in samples]
        seg = [0.0]
        for k in range(1, len(pts)):
            seg.append(v_len(v_sub(pts[k], pts[k - 1])))
        avg_seg = max(sum(seg[1:]) / max(len(seg) - 1, 1), 1e-6)

        cum = [0.0] * len(pts)
        for k in range(1, len(pts)):
            cum[k] = cum[k - 1] + seg[k]

        best = None
        for a in range(12, len(pts) - 48):
            max_b = min(len(pts) - 12, a + 120)
            for b in range(a + 24, max_b):
                chord = v_len(v_sub(pts[b], pts[a]))
                arc = cum[b] - cum[a]
                if arc < avg_seg * 60.0:
                    continue
                if chord > avg_seg * 8.0 or chord < 1e-6:
                    continue

                detour_ratio = arc / chord
                if detour_ratio < 7.5:
                    continue

                dir_in = v_norm(v_sub(pts[a], pts[a - 2]))
                dir_out = v_norm(v_sub(pts[b + 2], pts[b])) if (b + 2) < len(pts) else v_norm(v_sub(pts[b], pts[b - 2]))
                if dot(dir_in, dir_out) < 0.35:
                    continue

                if best is None or detour_ratio > best[0]:
                    best = (detour_ratio, a, b)

        if best is not None:
            _, a, b = best
            p_a = pts[a]
            p_b = pts[b]
            for k in range(a + 1, b):
                t = (k - a) / float(b - a)
                rep = v_lerp(p_a, p_b, t)
                samples[k]["pos_m"] = [rep[0], rep[1], rep[2]]
                total_changes += 1

    # Final pass: collapse needle-like micro segments that can produce visible
    # protrusions in generated fallback rail geometry.
    if len(samples) >= 3:
        seg_lengths = []
        for i in range(1, len(samples)):
            a = tuple(samples[i - 1]["pos_m"])
            b = tuple(samples[i]["pos_m"])
            seg_lengths.append(v_len(v_sub(b, a)))
        avg_seg = max(sum(seg_lengths) / max(len(seg_lengths), 1), 1e-6)

        tiny_abs = avg_seg * 0.03
        for i in range(1, len(samples) - 1):
            p_prev = tuple(samples[i - 1]["pos_m"])
            p_curr = tuple(samples[i]["pos_m"])
            p_next = tuple(samples[i + 1]["pos_m"])

            d_in = v_len(v_sub(p_curr, p_prev))
            d_out = v_len(v_sub(p_next, p_curr))
            short_leg = min(d_in, d_out)
            long_leg = max(d_in, d_out)

            # Strongly asymmetric local spacing is almost always a sampling glitch.
            if short_leg < tiny_abs and long_leg > avg_seg * 0.25:
                rep = v_lerp(p_prev, p_next, 0.5)
                samples[i]["pos_m"] = [rep[0], rep[1], rep[2]]
                total_changes += 1
            elif long_leg > 1e-6 and short_leg / long_leg < 0.04 and short_leg < avg_seg * 0.08:
                rep = v_lerp(p_prev, p_next, d_in / (d_in + d_out + 1e-12))
                samples[i]["pos_m"] = [rep[0], rep[1], rep[2]]
                total_changes += 1

    if total_changes > 0:
        recompute_samples_orientation(samples, axis_mapping)
    return total_changes


def segment_endpoints(elem: NLElemData, index: int):
    """Return (p0, kp1, kp2, p3) for segment `index`, matching build_sampled_path."""
    node = elem.nodes[index]
    p0 = (0.0, 0.0, 0.0) if index == 0 else elem.nodes[index - 1].p1
    return p0, node.kp1, node.kp2, node.p1


def detect_tangent_breaks(elem: NLElemData, angle_threshold_deg: float = 5.0) -> List[Dict]:
    """Find node boundaries where consecutive Bezier segments are not C1.

    build_sampled_path chains each segment onto the previous endpoint, which
    guarantees positional continuity but nothing about direction. Where the
    outgoing control point is not colinear with the incoming one, the path has a
    corner. A corner is infinite curvature, so it produces an unbounded force
    reading no matter how the curvature is estimated - the geometry is simply not
    differentiable there, and the only honest response is to say so.
    """
    breaks = []
    for i in range(elem.node_count - 1):
        _, _, kp2_in, p_join = segment_endpoints(elem, i)
        _, kp1_out, _, _ = segment_endpoints(elem, i + 1)

        tan_in = v_sub(p_join, kp2_in)
        tan_out = v_sub(kp1_out, p_join)
        if v_len(tan_in) < 1e-9 or v_len(tan_out) < 1e-9:
            continue

        cos_angle = max(-1.0, min(1.0, dot(v_norm(tan_in), v_norm(tan_out))))
        angle_deg = math.degrees(math.acos(cos_angle))
        if angle_deg > angle_threshold_deg:
            breaks.append(
                {
                    "node_index": i + 1,
                    "segment_before": i,
                    "segment_after": i + 1,
                    "angle_deg": angle_deg,
                }
            )
    return breaks


def detect_malformed_segments(
    elem: NLElemData,
    distortion_ratio: float = 1.25,
    internal_turn_deg: float = 60.0,
) -> List[Dict]:
    """Find segments whose Bezier control polygon folds back on itself.

    A healthy track segment has a control polygon barely longer than its chord
    and no sharp internal turn. When the polygon is much longer, or one leg
    reverses against the next, the curve contains a cusp or a loop: the tangent
    passes through zero and curvature diverges. No curvature estimator can
    recover a sensible force there, because the curve genuinely has a spike.

    Cheaper and more reliable than hunting for cusps in the sampled output,
    since the control polygon states the problem directly.
    """
    flagged = []
    for i in range(elem.node_count):
        p0, kp1, kp2, p3 = segment_endpoints(elem, i)

        leg1 = v_sub(kp1, p0)
        leg2 = v_sub(kp2, kp1)
        leg3 = v_sub(p3, kp2)
        polygon = v_len(leg1) + v_len(leg2) + v_len(leg3)
        chord = v_len(v_sub(p3, p0))
        if chord < 1e-9 or polygon < 1e-9:
            continue

        ratio = polygon / chord
        turns = []
        for a, b in ((leg1, leg2), (leg2, leg3)):
            if v_len(a) < 1e-9 or v_len(b) < 1e-9:
                continue
            cos_angle = max(-1.0, min(1.0, dot(v_norm(a), v_norm(b))))
            turns.append(math.degrees(math.acos(cos_angle)))
        max_turn = max(turns) if turns else 0.0

        if ratio > distortion_ratio or max_turn > internal_turn_deg:
            flagged.append(
                {
                    "segment": i,
                    "node_index": i + 1,
                    "polygon_over_chord": ratio,
                    "max_internal_turn_deg": max_turn,
                    "chord_m": chord,
                }
            )
    return flagged


def detect_source_gaps(elem: NLElemData, threshold_multiple: float = 5.0) -> List[Dict]:
    """Find nodes whose spacing is a large multiple of the median.

    A well-formed export steps along the track at a near-constant node spacing.
    A node that sits far from its predecessor means the export dropped the track
    in between, and the converter can only bridge it with one long Bezier. The
    sharp joins at each end of that bridge are geometry the ride never had, so
    forces there are meaningless and have to be labelled rather than reported.
    """
    if elem.node_count < 8:
        return []

    points = [(0.0, 0.0, 0.0)] + [n.p1 for n in elem.nodes]
    spacing = [v_len(v_sub(points[i], points[i - 1])) for i in range(1, len(points))]
    ordered = sorted(spacing)
    median = ordered[len(ordered) // 2]
    if median < 1e-9:
        return []

    limit = median * threshold_multiple
    gaps = []
    for i, step in enumerate(spacing):
        if step <= limit:
            continue
        gaps.append(
            {
                "node_index": i + 1,
                "gap_m": step,
                "median_spacing_m": median,
                "ratio": step / median,
                # Node 1 is the synthetic leading segment from the local origin
                # to the first exported node, not a hole in the middle of a ride.
                "kind": "leading_origin_segment" if i == 0 else "missing_track",
            }
        )
    return gaps


def mark_suspect_samples(samples: List[Dict], suspect_segments, margin_m: float) -> int:
    """Flag samples whose forces come from defective geometry, not real track."""
    for row in samples:
        row["suspect"] = False
    suspect_segments = set(suspect_segments)
    if not suspect_segments:
        return 0

    cum = cumulative_arclength(samples)

    flagged_spans = []
    for i, row in enumerate(samples):
        if int(row.get("segment", -1)) in suspect_segments:
            flagged_spans.append(cum[i])

    if not flagged_spans:
        return 0

    count = 0
    for i, row in enumerate(samples):
        for centre in flagged_spans:
            if abs(cum[i] - centre) <= margin_m:
                row["suspect"] = True
                count += 1
                break
    return count


def cumulative_arclength(samples) -> List[float]:
    cum = [0.0]
    for i in range(1, len(samples)):
        cum.append(cum[-1] + v_len(v_sub(tuple(samples[i]["pos_m"]), tuple(samples[i - 1]["pos_m"]))))
    return cum


def resample_uniform_arclength(samples: List[Dict], spacing_m: float, axis_mapping: str) -> List[Dict]:
    """Re-space samples evenly along the path.

    build_sampled_path steps each Bezier in uniform parameter t, which produces
    spacing that varies by an order of magnitude with |P'(t)|. Finite-difference
    curvature over unevenly spaced points is biased by the spacing itself, so the
    stations are levelled out before anything is differentiated.
    """
    if spacing_m <= 0.0 or len(samples) < 3:
        return samples

    cum = cumulative_arclength(samples)
    total = cum[-1]
    if total < spacing_m * 2.0:
        return samples

    n_out = max(int(round(total / spacing_m)) + 1, 3)
    out: List[Dict] = []
    src = 0

    for j in range(n_out):
        target = total * j / (n_out - 1)
        while src < len(cum) - 2 and cum[src + 1] < target:
            src += 1
        seg = cum[src + 1] - cum[src]
        t = 0.0 if seg < 1e-12 else (target - cum[src]) / seg

        a, b = samples[src], samples[src + 1]
        pos = v_lerp(tuple(a["pos_m"]), tuple(b["pos_m"]), t)

        # Roll is a scalar angle along the path, so it interpolates directly.
        # Orientation frames are rebuilt from position and roll afterwards.
        roll = float(a["roll_rad"]) + (float(b["roll_rad"]) - float(a["roll_rad"])) * t

        out.append(
            {
                "index": j,
                "segment": a["segment"],
                "t": float(a["t"]) + (float(b["t"]) - float(a["t"])) * t,
                "pos_m": [pos[0], pos[1], pos[2]],
                "roll_rad": roll,
                "tan": list(a["tan"]),
                "up": list(a["up"]),
            }
        )

    recompute_samples_orientation(out, axis_mapping)
    return out


def compute_curvature(samples: List[Dict], baseline_m, min_baseline_m: float = 0.0) -> List[float]:
    """Menger (circumcircle) curvature over a fixed physical baseline.

    Two deliberate choices:

    * The circumcircle of three points, kappa = 4*Area/(a*b*c), is well
      conditioned for uneven spacing, unlike |d(tangent)|/ds which divides by a
      length that may be near zero.
    * The three points straddle a real distance rather than being adjacent
      samples. A rider feels the curvature the vehicle traverses over its own
      length; sampling it at 10cm resolution instead turns every tangent kink
      between Bezier segments into an impulse, which is where readings of
      thousands of G came from.
    """
    n = len(samples)
    if n < 3:
        return [0.0] * n

    cum = cumulative_arclength(samples)

    # baseline_m may be a single value or one value per sample, so that the
    # measurement scale can follow local speed.
    if isinstance(baseline_m, (int, float)):
        baselines = [float(baseline_m)] * n
    else:
        baselines = list(baseline_m)
        if len(baselines) != n:
            raise ValueError("per-sample baseline length must match samples")

    out = [0.0] * n

    for i in range(n):
        half = max(baselines[i], min_baseline_m, 1e-6) * 0.5
        lo = i
        while lo > 0 and cum[i] - cum[lo] < half:
            lo -= 1
        hi = i
        while hi < n - 1 and cum[hi] - cum[i] < half:
            hi += 1
        if lo == i or hi == i:
            continue

        p_a = tuple(samples[lo]["pos_m"])
        p_b = tuple(samples[i]["pos_m"])
        p_c = tuple(samples[hi]["pos_m"])

        side_a = v_len(v_sub(p_b, p_a))
        side_b = v_len(v_sub(p_c, p_b))
        side_c = v_len(v_sub(p_c, p_a))
        if side_a < 1e-9 or side_b < 1e-9 or side_c < 1e-9:
            continue

        area = 0.5 * v_len(cross(v_sub(p_b, p_a), v_sub(p_c, p_a)))
        out[i] = 4.0 * area / (side_a * side_b * side_c)

    # Endpoints have no straddling window; hold the nearest interior value.
    for i in range(n):
        if out[i] != 0.0:
            for j in range(i):
                out[j] = out[i]
            break
    for i in range(n - 1, -1, -1):
        if out[i] != 0.0:
            for j in range(i + 1, n):
                out[j] = out[i]
            break

    return out


def simulate_gravity_timeline(
    samples: List[Dict],
    g: float,
    initial_speed: float,
    min_speed: float,
    rolling_friction: float,
    drag_coeff: float,
    curvature: List[float] | None = None,
) -> List[Dict]:
    if not samples:
        return []

    if curvature is None:
        curvature = [0.0] * len(samples)

    timeline = []
    t_acc = 0.0
    s_acc = 0.0
    v_prev = max(initial_speed, min_speed)

    first = dict(samples[0])
    first.update(
        {
            "time_s": 0.0,
            "distance_m": 0.0,
            "speed_mps": v_prev,
            "curvature_1pm": curvature[0],
            "normal_acc_mps2": v_prev * v_prev * curvature[0],
            "tangential_acc_mps2": 0.0,
        }
    )
    timeline.append(first)

    for i in range(1, len(samples)):
        p0 = tuple(samples[i - 1]["pos_m"])
        p1 = tuple(samples[i]["pos_m"])
        ds = max(v_len(v_sub(p1, p0)), 1e-6)
        s_acc += ds

        # Height is source Y (NoLimits/FVD convention). This is deliberately
        # independent of --axis-mapping: gravity acts along the source vertical
        # regardless of which Unreal axis that later becomes.
        dh = p0[1] - p1[1]

        # Energy update with simple rolling+drag losses.
        v_sq = max(
            v_prev * v_prev + 2.0 * g * dh - 2.0 * rolling_friction * g * ds - drag_coeff * ds * v_prev * v_prev,
            min_speed * min_speed,
        )
        v_cur = math.sqrt(v_sq)
        v_avg = max(0.5 * (v_prev + v_cur), min_speed)
        dt = ds / v_avg
        t_acc += dt

        k = curvature[i]
        row = dict(samples[i])
        row.update(
            {
                "time_s": t_acc,
                "distance_m": s_acc,
                "speed_mps": v_cur,
                "curvature_1pm": k,
                "normal_acc_mps2": v_cur * v_cur * k,
                # Longitudinal acceleration: what a rider feels as launch/brake.
                "tangential_acc_mps2": (v_cur * v_cur - v_prev * v_prev) / (2.0 * ds),
            }
        )
        timeline.append(row)
        v_prev = v_cur

    return timeline


def write_csv_timeline(path: Path, timeline: List[Dict]) -> None:
    fieldnames = [
        "index",
        "time_s",
        "distance_m",
        "speed_mps",
        "segment",
        "t",
        "x_m",
        "y_m",
        "z_m",
        "tan_x",
        "tan_y",
        "tan_z",
        "roll_rad",
        "ue_x_cm",
        "ue_y_cm",
        "ue_z_cm",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in timeline:
            writer.writerow(
                {
                    "index": row["index"],
                    "time_s": row["time_s"],
                    "distance_m": row["distance_m"],
                    "speed_mps": row["speed_mps"],
                    "segment": row["segment"],
                    "t": row["t"],
                    "x_m": row["pos_m"][0],
                    "y_m": row["pos_m"][1],
                    "z_m": row["pos_m"][2],
                    "tan_x": row["tan"][0],
                    "tan_y": row["tan"][1],
                    "tan_z": row["tan"][2],
                    "roll_rad": row["roll_rad"],
                    "ue_x_cm": row["ue_pos_cm"][0],
                    "ue_y_cm": row["ue_pos_cm"][1],
                    "ue_z_cm": row["ue_pos_cm"][2],
                }
            )


def _resample_by_arclength(pts: List[Tuple[float, float, float]], n_out: int):
    """Resample a polyline at n_out equal arc-length stations. Returns (pts, total)."""
    cum = [0.0]
    for i in range(1, len(pts)):
        cum.append(cum[-1] + v_len(v_sub(pts[i], pts[i - 1])))
    total = cum[-1]
    if total < 1e-9 or n_out < 2:
        return list(pts), total

    out = []
    i = 0
    for j in range(n_out):
        s = total * j / (n_out - 1)
        while i < len(cum) - 2 and cum[i + 1] < s:
            i += 1
        seg = cum[i + 1] - cum[i]
        t = 0.0 if seg < 1e-12 else (s - cum[i]) / seg
        out.append(v_lerp(pts[i], pts[i + 1], t))
    return out, total


def load_ue_reference_csv(path: Path) -> List[Tuple[float, float, float]]:
    """Load a UE-space spline CSV (Index,PosX,PosY,PosZ,...) in centimetres."""
    with path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    missing = {"PosX", "PosY", "PosZ"} - set(rows[0].keys() if rows else ())
    if missing:
        raise ValueError(f"{path} is missing required columns: {sorted(missing)}")
    return [(float(r["PosX"]), float(r["PosY"]), float(r["PosZ"])) for r in rows]


def validate_axis_mapping(
    source_points_m: List[Tuple[float, float, float]],
    reference_cm: List[Tuple[float, float, float]],
    selected_mapping: str,
    stations: int = 2000,
) -> Dict:
    """Score every axis mapping against a known-good UE-space reference path.

    Two independent checks, because each fails differently:
      * span match  - alignment-free, so gaps in the reference cannot skew it.
      * arc-length RMS - catches sign flips and axis swaps that spans alone miss.
    """
    ref_rs, ref_len = _resample_by_arclength(reference_cm, stations)
    ref_centroid = [sum(p[k] for p in ref_rs) / len(ref_rs) for k in range(3)]

    ref_span = [
        max(p[k] for p in reference_cm) - min(p[k] for p in reference_cm) for k in range(3)
    ]

    results = []
    for name in sorted(AXIS_MAPPINGS):
        conv = [to_ue_cm(p, name) for p in source_points_m]
        conv_span = [max(p[k] for p in conv) - min(p[k] for p in conv) for k in range(3)]
        span_err = max(abs(conv_span[k] - ref_span[k]) for k in range(3))

        conv_rs, conv_len = _resample_by_arclength(conv, stations)
        conv_centroid = [sum(p[k] for p in conv_rs) / len(conv_rs) for k in range(3)]
        sq = 0.0
        for i in range(min(len(conv_rs), len(ref_rs))):
            for k in range(3):
                d = (conv_rs[i][k] - conv_centroid[k]) - (ref_rs[i][k] - ref_centroid[k])
                sq += d * d
        n = max(min(len(conv_rs), len(ref_rs)) * 3, 1)
        results.append(
            {
                "mapping": name,
                "determinant": mapping_determinant(name),
                "span_error_cm": span_err,
                "rms_cm": math.sqrt(sq / n),
                "length_ratio": (ref_len / conv_len) if conv_len > 1e-9 else 0.0,
            }
        )

    results.sort(key=lambda r: r["rms_cm"])
    best = results[0]
    return {
        "reference_length_m": ref_len / M_TO_CM,
        "selected_mapping": selected_mapping,
        "best_mapping": best["mapping"],
        "selected_is_best": best["mapping"] == selected_mapping,
        "scores": results,
    }


def report_axis_validation(report: Dict) -> None:
    print("")
    print("--- axis mapping / scale validation ---")
    print(f"reference polyline length: {report['reference_length_m']:.2f} m")
    print(f"{'mapping':22s} {'det':>4s} {'span err':>12s} {'RMS':>12s} {'len ratio':>10s}")
    for r in report["scores"]:
        print(
            f"{r['mapping']:22s} {r['determinant']:+4.0f} "
            f"{r['span_error_cm']:9.2f} cm {r['rms_cm']:9.2f} cm "
            f"{r['length_ratio']:10.5f}"
        )

    if report["selected_is_best"]:
        print(f"OK: selected mapping '{report['selected_mapping']}' is the best fit.")
    else:
        print(
            f"WARNING: selected mapping '{report['selected_mapping']}' is NOT the "
            f"best fit. '{report['best_mapping']}' fits the reference better. "
            "The exported track is probably rotated or mirrored.",
            file=sys.stderr,
        )


def stage_car_mesh(mesh_file: Path | None, output_dir: Path) -> Dict:
    """Copy the car mesh in beside the bundle and return how to find it again.

    The bundle would otherwise carry an absolute path to wherever the mesh
    happened to live, which breaks the moment that file is moved or the export
    is handed to someone else. Copying it into the output folder makes the
    export self-contained, and the path recorded is relative to the bundle so it
    survives the whole folder being moved.
    """
    if mesh_file is None:
        return {"mesh_file": "", "mesh_file_source": "", "staged": False}

    source = Path(mesh_file)
    if not source.is_file():
        print(
            f"WARNING: car mesh not found, so nothing was staged: {source}",
            file=sys.stderr,
        )
        return {"mesh_file": "", "mesh_file_source": str(source), "staged": False}

    output_dir.mkdir(parents=True, exist_ok=True)
    destination = output_dir / source.name

    already_there = destination.exists() and destination.samefile(source)
    if not already_there:
        shutil.copy2(source, destination)

        # .obj keeps its materials in a sibling .mtl, which would be left behind.
        if source.suffix.lower() == ".obj":
            mtl = source.with_suffix(".mtl")
            if mtl.is_file():
                shutil.copy2(mtl, output_dir / mtl.name)
                print(f"Staged car material: {mtl.name}")

        print(f"Staged car mesh: {source.name} -> {output_dir}")
    else:
        print(f"Car mesh already in the output folder: {source.name}")

    if source.suffix.lower() not in (".glb", ".gltf"):
        print(
            f"NOTE: {source.suffix} can reference external textures. If the car "
            "imports untextured, copy those alongside it or use .glb, which "
            "embeds them."
        )

    return {
        "mesh_file": source.name,
        "mesh_file_source": str(source),
        "staged": True,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert OpenFVD .nlelem data into UE5-ready motion bundle")
    parser.add_argument("--spline", required=True, type=Path, help="Path to spline .nlelem")
    parser.add_argument("--tangent", type=Path, help="Path to tangent .nlelem (optional metadata/validation)")
    parser.add_argument("--mesh", type=Path, help="Path to coaster mesh .3ds (optional metadata)")
    parser.add_argument("--output", required=True, type=Path, help="Output bundle JSON path")
    parser.add_argument("--csv", type=Path, help="Optional output CSV timeline path")
    parser.add_argument("--samples-per-segment", type=int, default=20)
    parser.add_argument(
        "--resample-spacing-m",
        type=float,
        default=0.10,
        help="Re-space the analytic path at this arc-length interval before "
             "differentiating. 0 disables it and keeps uniform-in-t spacing.",
    )
    parser.add_argument(
        "--segment-distortion-ratio",
        type=float,
        default=1.25,
        help="Control-polygon length over chord length above which a segment is "
             "treated as containing a cusp or loop and its forces flagged.",
    )
    parser.add_argument(
        "--tangent-break-threshold-deg",
        type=float,
        default=5.0,
        help="Corner angle between consecutive Bezier segments above which the "
             "path is treated as non-differentiable and its forces flagged.",
    )
    parser.add_argument(
        "--curvature-window-s",
        type=float,
        default=0.15,
        help="Time window the curvature measurement spans, matching "
             "CoasterAnalyzer's differentiation window. The baseline becomes "
             "speed * window, so both measure the same physical scale. "
             "0 uses a fixed --curvature-baseline-m instead.",
    )
    parser.add_argument(
        "--curvature-baseline-m",
        type=float,
        default=1.0,
        help="Minimum curvature baseline in metres. Acts as a floor for the "
             "speed-scaled window so slow sections are not measured over a "
             "near-zero distance.",
    )
    parser.add_argument(
        "--axis-mapping",
        choices=sorted(AXIS_MAPPINGS.keys()),
        default="nl2_to_ue_swap_yz",
        help=(
            "Source-to-Unreal axis conversion. Default 'nl2_to_ue_swap_yz' is "
            "verified against a known-good UE spline export. Mappings with "
            "determinant +1 cannot convert right-handed source data to "
            "left-handed Unreal space and produce a mirrored track."
        ),
    )
    # The coaster car is presentation, not physics: none of these affect a single
    # number in the timeline. They travel in the bundle so that the Unreal side
    # needs one argument - the bundle path - to build the whole scene.
    car = parser.add_argument_group("coaster car (passed through to Unreal)")
    car.add_argument(
        "--car-mesh-asset",
        default="",
        help="Unreal asset path of the car mesh, e.g. /Game/Coaster/SM_Car.",
    )
    car.add_argument(
        "--car-mesh-file",
        type=Path,
        help="Local mesh file (.fbx/.glb/.obj) imported into Unreal if the "
             "asset above does not already exist.",
    )
    car.add_argument(
        "--car-forward-axis",
        choices=["+X", "-X", "+Y", "-Y"],
        default="+X",
        help="Which axis the car mesh faces in its own space. The path frame "
             "puts travel along +X, so anything else needs a yaw correction or "
             "the car drives sideways.",
    )
    car.add_argument(
        "--car-rotation-offset-deg",
        type=float,
        nargs=3,
        metavar=("ROLL", "PITCH", "YAW"),
        default=[0.0, 0.0, 0.0],
        help="Extra rotation applied after the forward-axis correction.",
    )
    car.add_argument(
        "--car-offset-cm",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=[0.0, 0.0, 0.0],
        help="Car offset from the track spine, in centimetres. Raise Z to sit "
             "the body on the rails instead of through them.",
    )
    car.add_argument("--car-scale", type=float, default=1.0)
    car.add_argument(
        "--car-fbx-fps",
        type=int,
        default=60,
        help="Frame rate of the exported CoasterCarAnimated.fbx.",
    )
    car.add_argument(
        "--no-car-fbx",
        action="store_true",
        help="Skip writing CoasterCarAnimated.fbx.",
    )
    car.add_argument(
        "--car-expected-length-m",
        type=float,
        default=0.0,
        help="Real-world car length for a scale check on import. 0 skips it.",
    )

    parser.add_argument(
        "--validate-reference-csv",
        type=Path,
        help=(
            "Optional UE-space spline CSV (Index,PosX,PosY,PosZ,... in cm) used "
            "to verify the axis mapping and scale. Reports the best-fitting "
            "mapping and warns if it is not the one selected."
        ),
    )
    parser.add_argument("--g", type=float, default=9.81)
    parser.add_argument("--initial-speed", type=float, default=6.0)
    parser.add_argument("--min-speed", type=float, default=1.0)
    parser.add_argument("--rolling-friction", type=float, default=0.004)
    parser.add_argument("--drag-coeff", type=float, default=0.0004)
    parser.add_argument("--disable-spike-filter", action="store_true")
    parser.add_argument("--spike-angle-threshold-deg", type=float, default=70.0)
    parser.add_argument("--spike-deviation-multiplier", type=float, default=0.25)
    parser.add_argument("--spike-max-passes", type=int, default=5)

    args = parser.parse_args()

    det = mapping_determinant(args.axis_mapping)
    if det > 0.0:
        print(
            f"WARNING: axis mapping '{args.axis_mapping}' has determinant "
            f"{det:+.0f}. It cannot convert right-handed source data into "
            "left-handed Unreal space, so the track will be MIRRORED: "
            "left-hand turns become right-hand and lateral-G signs flip. "
            "Use --axis-mapping nl2_to_ue_flip_y unless you specifically "
            "want a mirrored track.",
            file=sys.stderr,
        )

    spline = parse_nlelem(args.spline)
    tangent = parse_nlelem(args.tangent) if args.tangent else None

    if tangent and tangent.node_count != spline.node_count:
        raise ValueError(
            f"Node mismatch: spline has {spline.node_count}, tangent has {tangent.node_count}."
        )

    sampled = build_sampled_path(
        elem=spline,
        samples_per_segment=max(args.samples_per_segment, 2),
        axis_mapping=args.axis_mapping,
        initial_roll=spline.nodes[0].roll if spline.nodes else 0.0,
    )

    # The spike filter rewrites sample positions. Every position edit changes
    # local curvature, and curvature is what the forces are derived from: a
    # straight-line replacement reads as kappa=0 (phantom airtime) bracketed by
    # C1 kinks (phantom spikes). So it runs on a copy used only for building
    # render geometry, and the analytic path stays exactly as sampled.
    raw_sample_count = len(sampled)
    sampled = resample_uniform_arclength(
        sampled, args.resample_spacing_m, args.axis_mapping
    )
    if len(sampled) != raw_sample_count:
        print(
            f"Resampled analytic path: {raw_sample_count} -> {len(sampled)} "
            f"points at {args.resample_spacing_m * 100:.1f}cm spacing"
        )

    render_samples = copy.deepcopy(sampled)
    spike_edits = 0
    if not args.disable_spike_filter:
        spike_edits = smooth_extreme_spikes(
            render_samples,
            axis_mapping=args.axis_mapping,
            angle_threshold_deg=args.spike_angle_threshold_deg,
            deviation_multiplier=args.spike_deviation_multiplier,
            max_passes=args.spike_max_passes,
        )

    validation = None
    if args.validate_reference_csv:
        node_points_m = [(0.0, 0.0, 0.0)] + [n.p1 for n in spline.nodes]
        validation = validate_axis_mapping(
            source_points_m=node_points_m,
            reference_cm=load_ue_reference_csv(args.validate_reference_csv),
            selected_mapping=args.axis_mapping,
        )
        report_axis_validation(validation)

    gaps = detect_source_gaps(spline)
    breaks = detect_tangent_breaks(spline, args.tangent_break_threshold_deg)
    malformed = detect_malformed_segments(spline, args.segment_distortion_ratio)
    real_gaps = [g for g in gaps if g["kind"] == "missing_track"]
    if gaps:
        print("")
        print("--- source geometry defects ---")
        for g in gaps:
            note = (
                "synthetic leading segment from origin"
                if g["kind"] == "leading_origin_segment"
                else "MISSING TRACK in the export"
            )
            print(
                f"  node {g['node_index']:4d}: {g['gap_m']:7.2f}m gap "
                f"({g['ratio']:5.1f}x median {g['median_spacing_m']:.2f}m) - {note}"
            )
        if real_gaps:
            print(
                f"WARNING: {len(real_gaps)} gap(s) in the source export. The path "
                "is bridged across them with a single long curve, so forces near "
                "those joins are not real. Affected samples are flagged "
                '"suspect": true in the bundle. Re-export from OpenFVD/NoLimits to '
                "fix this properly.",
                file=sys.stderr,
            )

    if malformed:
        worst_seg = sorted(malformed, key=lambda m: -m["polygon_over_chord"])
        print("")
        print(f"--- malformed segments: {len(malformed)} contain a cusp or loop ---")
        for m in worst_seg[:8]:
            print(
                f"  segment {m['segment']:4d}: control polygon "
                f"{m['polygon_over_chord']:6.2f}x its {m['chord_m']:.2f}m chord, "
                f"max internal turn {m['max_internal_turn_deg']:6.1f} deg"
            )
        if len(worst_seg) > 8:
            print(f"  ... and {len(worst_seg) - 8} more")
        print(
            f"WARNING: {len(malformed)} segment(s) have a self-folding control "
            "polygon. The curve reverses inside them, so curvature genuinely "
            "diverges and no force reading there is meaningful. Affected samples "
            'are flagged "suspect": true.',
            file=sys.stderr,
        )

    if breaks:
        worst = sorted(breaks, key=lambda b: -b["angle_deg"])
        print("")
        print(
            f"--- tangent discontinuities: {len(breaks)} node boundaries exceed "
            f"{args.tangent_break_threshold_deg:.1f} deg ---"
        )
        for b in worst[:8]:
            print(
                f"  node {b['node_index']:4d}: {b['angle_deg']:6.2f} deg corner "
                f"between segments {b['segment_before']} and {b['segment_after']}"
            )
        if len(worst) > 8:
            print(f"  ... and {len(worst) - 8} more")
        print(
            f"WARNING: the source path is not C1 continuous at {len(breaks)} node "
            "boundaries. A corner has undefined curvature, so force readings "
            "there are artefacts of the geometry, not of the ride. Affected "
            'samples are flagged "suspect": true.',
            file=sys.stderr,
        )

    # Speed does not depend on curvature in this energy model, so the timeline
    # can be solved first and its speeds used to size the curvature baseline.
    def run_timeline(curv):
        return simulate_gravity_timeline(
            sampled,
            g=args.g,
            initial_speed=args.initial_speed,
            min_speed=args.min_speed,
            rolling_friction=args.rolling_friction,
            drag_coeff=args.drag_coeff,
            curvature=curv,
        )

    if args.curvature_window_s > 0.0:
        speeds = [row["speed_mps"] for row in run_timeline(None)]
        # An accelerometer, and CoasterAnalyzer, filter over a time window. The
        # distance that window covers depends on how fast the car is moving, so
        # matching it means a baseline of v*window rather than a fixed length.
        baselines = [v * args.curvature_window_s for v in speeds]
        curvature = compute_curvature(
            sampled, baselines, min_baseline_m=args.curvature_baseline_m
        )
        print(
            f"Curvature baseline: {args.curvature_window_s:.3f}s of travel "
            f"({min(baselines):.2f}-{max(baselines):.2f}m, floor "
            f"{args.curvature_baseline_m:.2f}m)"
        )
    else:
        curvature = compute_curvature(sampled, args.curvature_baseline_m)

    timeline = run_timeline(curvature)

    suspect_segments = {g["node_index"] - 1 for g in gaps}
    for b in breaks:
        suspect_segments.add(b["segment_before"])
        suspect_segments.add(b["segment_after"])
    for m in malformed:
        suspect_segments.add(m["segment"])
    staged_car = stage_car_mesh(args.car_mesh_file, args.output.parent)

    suspect_count = mark_suspect_samples(
        timeline,
        suspect_segments,
        margin_m=max(args.curvature_baseline_m * 2.0, 2.0),
    )

    bundle = {
        "format": "ue5_coaster_bundle_v2",
        "source": {
            "spline_nlelem": str(args.spline),
            "tangent_nlelem": str(args.tangent) if args.tangent else None,
            "mesh_3ds": str(args.mesh) if args.mesh else None,
            "axis_mapping": args.axis_mapping,
        },
        "units": {
            "pos_m": "metres, source axes (X-right, Y-up, Z-forward)",
            "ue_pos_cm": "centimetres, Unreal axes (X-forward, Y-right, Z-up)",
            "ue_tan_cm": "Unreal-space tangent, magnitude 100cm (spline tangent)",
            "ue_tan": "Unreal-space unit tangent",
            "ue_up": "Unreal-space unit up (banked)",
            "time_s": "seconds",
            "speed_mps": "metres per second",
            "metres_to_unreal_units": M_TO_CM,
        },
        # Consumed by unreal_import_coaster.py. Nothing here feeds the physics.
        # mesh_file is a bare filename resolved next to this bundle; the Unreal
        # content folder it imports into is derived there from the level
        # sequence's own folder, so everything created lands together.
        "car": {
            "mesh_asset": args.car_mesh_asset,
            **staged_car,
            "forward_axis": args.car_forward_axis,
            "rotation_offset_deg": list(args.car_rotation_offset_deg),
            "offset_cm": list(args.car_offset_cm),
            "scale": args.car_scale,
            "expected_length_m": args.car_expected_length_m,
        },
        "validation": validation,
        "source_defects": {
            "gaps": gaps,
            "malformed_segments": malformed,
            "tangent_breaks": breaks,
            "tangent_break_threshold_deg": args.tangent_break_threshold_deg,
            "suspect_sample_count": suspect_count,
            "suspect_sample_fraction": (
                suspect_count / len(timeline) if timeline else 0.0
            ),
        },
        "handedness": {
            "source": "right-handed",
            "target": "left-handed (Unreal)",
            "mapping_determinant": mapping_determinant(args.axis_mapping),
            "preserves_handedness": mapping_determinant(args.axis_mapping) < 0.0,
        },
        "nlelem": {
            "spline": {
                "data_length": spline.data_length,
                "node_count": spline.node_count,
            },
            "tangent": None
            if tangent is None
            else {
                "data_length": tangent.data_length,
                "node_count": tangent.node_count,
            },
        },
        "physics": {
            "resample_spacing_m": args.resample_spacing_m,
            "curvature_window_s": args.curvature_window_s,
            "curvature_baseline_m": args.curvature_baseline_m,
            "g": args.g,
            "initial_speed": args.initial_speed,
            "min_speed": args.min_speed,
            "rolling_friction": args.rolling_friction,
            "drag_coeff": args.drag_coeff,
        },
        "cleanup": {
            # Applies to "render_path" only. "samples" is never geometry-edited,
            # so the physics timeline is derived from the path as sampled.
            "applies_to": "render_path",
            "spike_filter_enabled": not args.disable_spike_filter,
            "spike_angle_threshold_deg": args.spike_angle_threshold_deg,
            "spike_deviation_multiplier": args.spike_deviation_multiplier,
            "spike_max_passes": args.spike_max_passes,
            "spike_points_smoothed": spike_edits,
        },
        "samples": timeline,
        # Carries orientation as well as position so Unreal can build the track
        # spline directly, with correct tangents, without an FBX in the loop.
        "render_path": [
            {
                "index": s["index"],
                "pos_m": s["pos_m"],
                "up": s["up"],
                "ue_pos_cm": s["ue_pos_cm"],
                "ue_tan_cm": s["ue_tan_cm"],
                "ue_tan": s["ue_tan"],
                "ue_up": s["ue_up"],
            }
            for s in render_samples
        ],
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(bundle, indent=2), encoding="utf-8")

    if args.csv:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        write_csv_timeline(args.csv, timeline)

    if not args.no_car_fbx:
        # The animation has to exist as a real file in the export, not only as
        # something the Unreal script reconstructs from the timeline.
        try:
            from export_car_animation import write_car_animation_fbx

            length_cm = (
                args.car_expected_length_m * 100.0
                if args.car_expected_length_m > 0.0
                else 450.0
            )
            fbx_info = write_car_animation_fbx(
                args.output.parent / "CoasterCarAnimated.fbx",
                timeline,
                fps=max(int(args.car_fbx_fps), 1),
                box_size_cm=(length_cm, length_cm * 0.36, length_cm * 0.27),
            )
            bundle["car"]["animation_fbx"] = Path(fbx_info["path"]).name
            bundle["car"]["animation_fbx_fps"] = fbx_info["fps"]
            args.output.write_text(json.dumps(bundle, indent=2), encoding="utf-8")
            print(
                f"Wrote animation: {Path(fbx_info['path']).name} "
                f"({fbx_info['frames']} frames at {fbx_info['fps']}fps, "
                f"{fbx_info['size_bytes'] / 1024:.0f} KB)"
            )
        except Exception as ex:
            print(
                f"WARNING: could not write the animated FBX: {ex}",
                file=sys.stderr,
            )

    print(f"Wrote bundle: {args.output}")
    if args.csv:
        print(f"Wrote csv: {args.csv}")
    print(f"Nodes: {spline.node_count}, sampled points: {len(timeline)}")
    print(
        f"Axis mapping: {args.axis_mapping} (determinant "
        f"{mapping_determinant(args.axis_mapping):+.0f})"
    )
    if args.car_mesh_asset or staged_car["mesh_file"]:
        print(
            f"Car: {args.car_mesh_asset or staged_car['mesh_file']} "
            f"(forward {args.car_forward_axis}, scale {args.car_scale})"
        )
    else:
        print(
            "Car: none set. Unreal will animate a placeholder cube so the "
            "motion is still visible; set --car-mesh-asset or --car-mesh-file "
            "to use a real car."
        )
    if not args.disable_spike_filter:
        print(f"Spike smoothing edits (render path only): {spike_edits}")

    def pct(vals, q):
        if not vals:
            return 0.0
        return vals[min(len(vals) - 1, max(0, int(round(q * (len(vals) - 1)))))]

    speeds = sorted(r["speed_mps"] for r in timeline)
    all_g = sorted(r["normal_acc_mps2"] / 9.80665 for r in timeline)
    clean_g = sorted(
        r["normal_acc_mps2"] / 9.80665 for r in timeline if not r.get("suspect")
    )

    print("")
    print(
        f"Duration {timeline[-1]['time_s']:.2f}s, length "
        f"{timeline[-1]['distance_m']:.1f}m, speed "
        f"{pct(speeds, 0.5):.1f}/{speeds[-1]:.1f} m/s (median/peak)"
    )
    print(
        f"Normal G, all samples      : median {pct(all_g, 0.5):.2f}  "
        f"p95 {pct(all_g, 0.95):.2f}  peak {all_g[-1]:.2f}"
    )
    if suspect_count:
        print(
            f"Normal G, excl. {suspect_count} suspect: median "
            f"{pct(clean_g, 0.5):.2f}  p95 {pct(clean_g, 0.95):.2f}  "
            f"peak {clean_g[-1] if clean_g else 0.0:.2f}"
        )

    reference = clean_g if clean_g else all_g
    if reference and reference[-1] > 10.0:
        print(
            f"WARNING: peak normal load {reference[-1]:.1f}G outside the flagged "
            "regions still exceeds anything a real coaster produces. Raise "
            "--curvature-baseline-m, or check the source for tangent "
            "discontinuities between elements.",
            file=sys.stderr,
        )


if __name__ == "__main__":
    main()
