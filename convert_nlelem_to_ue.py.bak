#!/usr/bin/env python3
"""
Convert FVD++/NoLimits .nlelem coaster exports to a UE5-friendly bundle.

Outputs:
- Single JSON bundle with spline, sampled path, and gravity-based motion timeline.
- Optional CSV timeline for quick inspection/import.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import struct
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


def to_ue_cm(pos_m: Tuple[float, float, float], mapping: str) -> Tuple[float, float, float]:
    x, y, z = pos_m
    if mapping == "nl2_to_ue":
        # Assumes source is X-right, Y-up, Z-forward. UE is X-forward, Y-right, Z-up.
        return (z * 100.0, x * 100.0, y * 100.0)
    if mapping == "nl2_to_ue_flip_y":
        return (z * 100.0, -x * 100.0, y * 100.0)
    if mapping == "identity":
        return (x * 100.0, y * 100.0, z * 100.0)
    raise ValueError(f"Unsupported mapping: {mapping}")


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


def simulate_gravity_timeline(
    samples: List[Dict],
    g: float,
    initial_speed: float,
    min_speed: float,
    rolling_friction: float,
    drag_coeff: float,
) -> List[Dict]:
    if not samples:
        return []

    timeline = []
    t_acc = 0.0
    s_acc = 0.0
    v_prev = max(initial_speed, min_speed)
    prev_tan = samples[0]["tan"]

    first = dict(samples[0])
    first.update({"time_s": 0.0, "distance_m": 0.0, "speed_mps": v_prev, "curvature_1pm": 0.0, "normal_acc_mps2": 0.0})
    timeline.append(first)

    for i in range(1, len(samples)):
        p0 = tuple(samples[i - 1]["pos_m"])
        p1 = tuple(samples[i]["pos_m"])
        ds = max(v_len(v_sub(p1, p0)), 1e-6)
        s_acc += ds

        # Height axis is source Y (NoLimits/FVD convention).
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

        cur_tan = samples[i]["tan"]
        d_tan = v_sub(tuple(cur_tan), tuple(prev_tan))
        curvature = v_len(d_tan) / ds
        normal_acc = v_cur * v_cur * curvature

        row = dict(samples[i])
        row.update(
            {
                "time_s": t_acc,
                "distance_m": s_acc,
                "speed_mps": v_cur,
                "curvature_1pm": curvature,
                "normal_acc_mps2": normal_acc,
            }
        )
        timeline.append(row)
        v_prev = v_cur
        prev_tan = cur_tan

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


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert FVD++ .nlelem data into UE5-ready motion bundle")
    parser.add_argument("--spline", required=True, type=Path, help="Path to spline .nlelem")
    parser.add_argument("--tangent", type=Path, help="Path to tangent .nlelem (optional metadata/validation)")
    parser.add_argument("--mesh", type=Path, help="Path to coaster mesh .3ds (optional metadata)")
    parser.add_argument("--output", required=True, type=Path, help="Output bundle JSON path")
    parser.add_argument("--csv", type=Path, help="Optional output CSV timeline path")
    parser.add_argument("--samples-per-segment", type=int, default=20)
    parser.add_argument("--axis-mapping", choices=["nl2_to_ue", "nl2_to_ue_flip_y", "identity"], default="nl2_to_ue")
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

    spike_edits = 0
    if not args.disable_spike_filter:
        spike_edits = smooth_extreme_spikes(
            sampled,
            axis_mapping=args.axis_mapping,
            angle_threshold_deg=args.spike_angle_threshold_deg,
            deviation_multiplier=args.spike_deviation_multiplier,
            max_passes=args.spike_max_passes,
        )

    timeline = simulate_gravity_timeline(
        sampled,
        g=args.g,
        initial_speed=args.initial_speed,
        min_speed=args.min_speed,
        rolling_friction=args.rolling_friction,
        drag_coeff=args.drag_coeff,
    )

    bundle = {
        "format": "ue5_coaster_bundle_v1",
        "source": {
            "spline_nlelem": str(args.spline),
            "tangent_nlelem": str(args.tangent) if args.tangent else None,
            "mesh_3ds": str(args.mesh) if args.mesh else None,
            "axis_mapping": args.axis_mapping,
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
            "g": args.g,
            "initial_speed": args.initial_speed,
            "min_speed": args.min_speed,
            "rolling_friction": args.rolling_friction,
            "drag_coeff": args.drag_coeff,
        },
        "cleanup": {
            "spike_filter_enabled": not args.disable_spike_filter,
            "spike_angle_threshold_deg": args.spike_angle_threshold_deg,
            "spike_deviation_multiplier": args.spike_deviation_multiplier,
            "spike_max_passes": args.spike_max_passes,
            "spike_points_smoothed": spike_edits,
        },
        "samples": timeline,
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(bundle, indent=2), encoding="utf-8")

    if args.csv:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        write_csv_timeline(args.csv, timeline)

    print(f"Wrote bundle: {args.output}")
    if args.csv:
        print(f"Wrote csv: {args.csv}")
    print(f"Nodes: {spline.node_count}, sampled points: {len(timeline)}")
    if not args.disable_spike_filter:
        print(f"Spike smoothing edits: {spike_edits}")


if __name__ == "__main__":
    main()
