"""Minimal binary FBX reader, used to learn and verify record layout."""

import struct
import sys
import zlib

HEADER = b"Kaydara FBX Binary  " + bytes([0x00, 0x1A, 0x00])


def read_prop(buf, off):
    kind = chr(buf[off])
    off += 1
    if kind == "Y":
        return kind, struct.unpack_from("<h", buf, off)[0], off + 2
    if kind == "C":
        return kind, bool(buf[off]), off + 1
    if kind == "I":
        return kind, struct.unpack_from("<i", buf, off)[0], off + 4
    if kind == "F":
        return kind, struct.unpack_from("<f", buf, off)[0], off + 4
    if kind == "D":
        return kind, struct.unpack_from("<d", buf, off)[0], off + 8
    if kind == "L":
        return kind, struct.unpack_from("<q", buf, off)[0], off + 8
    if kind in "SR":
        n = struct.unpack_from("<I", buf, off)[0]
        off += 4
        return kind, buf[off:off + n], off + n
    if kind in "fdlib":
        length, encoding, comp_len = struct.unpack_from("<III", buf, off)
        off += 12
        data = buf[off:off + comp_len]
        off += comp_len
        if encoding == 1:
            data = zlib.decompress(data)
        fmt = {"f": "f", "d": "d", "l": "q", "i": "i", "b": "b"}[kind]
        vals = list(struct.unpack("<" + fmt * length, data)) if length else []
        return kind + "[]", (vals, encoding), off
    raise ValueError(f"unknown property type {kind!r} at {off}")


def read_node(buf, off, use64):
    if use64:
        end, nprops, plen = struct.unpack_from("<QQQ", buf, off)
        off += 24
    else:
        end, nprops, plen = struct.unpack_from("<III", buf, off)
        off += 12
    namelen = buf[off]
    off += 1
    name = buf[off:off + namelen].decode("ascii", "replace")
    off += namelen

    props = []
    for _ in range(nprops):
        kind, val, off = read_prop(buf, off)
        props.append((kind, val))

    # Children exist only when the node has them, closed by a null record.
    null_len = 25 if use64 else 13
    children = []
    while off + null_len <= end:
        if buf[off:off + null_len] == bytes(null_len):
            off += null_len
            break
        child, off = read_node(buf, off, use64)
        children.append(child)
    return {"name": name, "props": props, "children": children}, end


def parse(path):
    buf = open(path, "rb").read()
    assert buf[:23] == HEADER, "not a binary FBX"
    version = struct.unpack_from("<I", buf, 23)[0]
    use64 = version >= 7500
    null_len = 25 if use64 else 13
    off = 27
    nodes = []
    while off + null_len <= len(buf):
        if buf[off:off + null_len] == bytes(null_len):
            break
        node, off = read_node(buf, off, use64)
        nodes.append(node)
    return version, nodes


def brief(kind, val):
    if kind.endswith("[]"):
        vals, enc = val
        head = ", ".join(f"{v:.6g}" if isinstance(v, float) else str(v) for v in vals[:6])
        return f"{kind} len={len(vals)} enc={enc} [{head}{' ...' if len(vals) > 6 else ''}]"
    if isinstance(val, bytes):
        return f'{kind} "{val.decode("ascii", "replace")}"'
    if isinstance(val, float):
        return f"{kind} {val:.6g}"
    return f"{kind} {val}"


def dump(node, depth=0, maxdepth=99):
    pad = "  " * depth
    props = "  ".join(brief(k, v) for k, v in node["props"])
    print(f"{pad}{node['name']}: {props}")
    if depth < maxdepth:
        for c in node["children"]:
            dump(c, depth + 1, maxdepth)


def find(node, name, out):
    if node["name"] == name:
        out.append(node)
    for c in node["children"]:
        find(c, name, out)
    return out


if __name__ == "__main__":
    path = sys.argv[1]
    only = sys.argv[2] if len(sys.argv) > 2 else None
    version, nodes = parse(path)
    print(f"FBX version {version}   top-level: {[n['name'] for n in nodes]}\n")
    if only:
        hits = []
        for n in nodes:
            find(n, only, hits)
        print(f"{len(hits)} node(s) named {only!r}\n")
        for h in hits[:3]:
            dump(h)
            print()
    else:
        for n in nodes:
            dump(n, maxdepth=1)
            print()
