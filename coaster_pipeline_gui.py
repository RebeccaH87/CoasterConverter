#!/usr/bin/env python3
"""
Single-window GUI for the UE5 coaster conversion pipeline.

Wraps convert_nlelem_to_ue.py, which produces a JSON bundle that Unreal reads
directly via unreal_import_coaster.py. Blender and FBX are not involved.
"""

from __future__ import annotations

import argparse
import json
import os
import queue
import subprocess
import sys
import threading
from dataclasses import asdict, dataclass, field
from pathlib import Path


APP_NAME = "UE5 Coaster Pipeline"
IS_FROZEN = getattr(sys, "frozen", False)
SCRIPT_DIR = Path(__file__).resolve().parent
RESOURCE_DIR = Path(getattr(sys, "_MEIPASS", SCRIPT_DIR))


def _default_app_state_dir() -> Path:
    if sys.platform == "win32":
        return Path(os.environ.get("LOCALAPPDATA", Path.home() / "AppData" / "Local"))
    if sys.platform == "darwin":
        return Path.home() / "Library" / "Application Support"
    return Path(os.environ.get("XDG_DATA_HOME", Path.home() / ".local" / "share"))


APP_STATE_DIR = _default_app_state_dir() / "UE5_CoasterPipeline"
CONVERTER_SCRIPT = RESOURCE_DIR / "convert_nlelem_to_ue.py"
STATE_FILE = APP_STATE_DIR / ".gui_state.json"

CONFIG_VERSION = 7

# LSU brand colours: Purple PMS 268 and Gold PMS 123.
LSU_PURPLE = "#461D7C"
LSU_GOLD = "#FDD023"

THEME = {
    "bg": "#14111C",
    "surface": "#1F1930",
    "surface_alt": "#2A2240",
    "surface_hover": "#241D38",
    "field": "#171223",
    "field_focus": "#1E1731",
    "line": "#2C2440",
    "hairline": "#3A3154",
    "brand": LSU_PURPLE,
    "brand_light": "#5C2BA0",
    "pill": "#6E39BE",
    "brand_deep": "#38175F",
    "accent": LSU_GOLD,
    "accent_hover": "#FFE05C",
    "accent_ink": "#2A1E00",
    "fg": "#F5F2FB",
    "muted": "#A79BC0",
    "faint": "#7E7299",
    "ok": "#77D992",
    "warn": LSU_GOLD,
    "error": "#FF8A9D",
}

# Windows 11 ships Segoe UI Variable in optical sizes: Display for headings,
# Text for body, Small for captions. Using the right one at each size is most of
# what makes the type look considered rather than uniformly scaled.
FONT_STACKS = {
    "display": ["Segoe UI Variable Display", "Segoe UI Semibold", "Segoe UI"],
    "text": ["Segoe UI Variable Text", "Segoe UI"],
    "small": ["Segoe UI Variable Small", "Segoe UI Variable Text", "Segoe UI"],
    "mono": ["Cascadia Mono", "Cascadia Code", "Consolas", "Courier New"],
}

# Resolved at startup by resolve_fonts().
FONT = "Segoe UI"
FONT_TEXT = "Segoe UI"
FONT_SMALL = "Segoe UI"
MONO = "Consolas"


@dataclass
class PipelineConfig:
    config_version: int = CONFIG_VERSION

    # Files
    spline_nlelem: str = ""
    tangent_nlelem: str = ""
    output_dir: str = ""
    validate_reference_csv: str = ""
    python_exe: str = sys.executable

    # Physics
    initial_speed: float = 6.0
    lift_speed: float = 4.0
    rolling_friction: float = 0.004
    drag_coeff: float = 0.0004
    curvature_window_s: float = 0.15
    curvature_baseline_m: float = 1.0

    # Coaster car. Presentation only: none of it affects the physics timeline.
    car_mesh_asset: str = ""
    car_mesh_file: str = ""
    car_forward_axis: str = "auto"
    car_scale: float = 1.0
    car_offset_z_cm: float = 0.0
    car_yaw_offset_deg: float = 0.0
    car_expected_length_m: float = 0.0
    car_export_fbx: bool = True
    car_export_glb: bool = True
    car_fbx_fps: int = 60
    car_import_fps: int = 30

    # Procedural track mesh. Presentation only, like the car.
    track_export_fbx: bool = True
    track_station_spacing_cm: float = 40.0
    track_gauge_cm: float = 100.0
    track_rail_drop_cm: float = 110.0
    track_tie_spacing_cm: float = 150.0
    track_supports: bool = True
    track_support_spacing_cm: float = 900.0

    # Geometry / sampling
    axis_mapping: str = "nl2_to_ue_swap_yz"
    samples_per_segment: int = 20
    resample_spacing_m: float = 0.10

    # Render-path cleanup (does not affect the physics timeline)
    spike_filter_enabled: bool = True
    spike_angle_threshold_deg: float = 70.0
    spike_deviation_multiplier: float = 0.25
    spike_max_passes: int = 5

    # Source-defect detection thresholds
    segment_distortion_ratio: float = 1.25
    tangent_break_threshold_deg: float = 5.0

    # Populated by migrate_state for logging. Excluded from the saved file.
    state_migrations: list = field(default_factory=list)


# Settings that no longer exist. Kept only so a saved file that still contains
# them can be reported and cleaned rather than silently carrying dead weight.
REMOVED_KEYS = {
    "mesh_3ds",
    "blender_exe",
    "cart_model_glb",
    "run_fbx_conversion",
    "export_animated_fbx",
    "animated_fbx_fps",
    "animated_fbx_speed_multiplier",
    "animated_fbx_scale_multiplier",
    "animated_fbx_cart_scale",
    "animated_fbx_cart_fit_mode",
    "animated_fbx_cart_target_length",
    "animated_fbx_calibration_cube_m",
    "animated_fbx_root_rot_x_deg",
    "physics_accurate_mode",
    # An Unreal content-browser path, not a disk folder, and never a choice worth
    # making: the importer now uses the level sequence's own folder so that
    # everything it creates lands together.
    "car_import_dir",
}

# Axis mappings that cannot convert right-handed source data into left-handed
# Unreal space, so they silently mirror the track.
MIRRORED_MAPPINGS = {"nl2_to_ue", "nl2_to_ue_flip_y", "identity"}


def default_config() -> PipelineConfig:
    cfg = PipelineConfig()
    base = Path.home() / "Documents" / "TestCoaster" / "CoasterRawExportData"
    if base.exists():
        cfg.spline_nlelem = str(base / "CoasterSpline.nlelem")
        cfg.tangent_nlelem = str(base / "CoasterTangent.nlelem")
        cfg.output_dir = str(base / "UE5")

    for ref in (
        Path.home() / "Documents" / "TestCoaster" / "Coaster_UE_spline.csv",
        base / "Coaster_UE_spline.csv",
    ):
        if ref.exists():
            cfg.validate_reference_csv = str(ref)
            break

    home = Path.home()
    for car in (
        SCRIPT_DIR / "KexLSMfoSketchfab.glb",
        home / "Documents" / "TestCoaster" / "KexLSMfoSketchfab.glb",
        base / "KexLSMfoSketchfab.glb",
        home / "Downloads" / "kexedit-lsm-rollercoaster-trains" / "source"
        / "KexLSMfoSketchfab.glb",
    ):
        if car.exists():
            cfg.car_mesh_file = str(car)
            break

    return cfg


def load_state() -> PipelineConfig:
    cfg = default_config()
    if not STATE_FILE.exists():
        return cfg
    try:
        data = json.loads(STATE_FILE.read_text(encoding="utf-8"))
    except Exception:
        return cfg

    for key, value in data.items():
        if hasattr(cfg, key) and key != "state_migrations":
            setattr(cfg, key, value)

    return migrate_state(cfg, data)


def migrate_state(cfg: PipelineConfig, data: dict) -> PipelineConfig:
    """Bring a saved config forward, recording each change for the run log.

    Saved state overrides dataclass defaults, so a corrected default alone would
    leave every existing install running the old, wrong value.
    """
    notes: list[str] = []
    from_version = int(data.get("config_version", 1))

    if from_version < 3:
        if cfg.axis_mapping in MIRRORED_MAPPINGS:
            notes.append(
                f"axis_mapping {cfg.axis_mapping!r} -> 'nl2_to_ue_swap_yz' "
                "(the old value mirrored the track)"
            )
            cfg.axis_mapping = "nl2_to_ue_swap_yz"

        # A stale value here points at the packaged GUI often enough that it is
        # worth repairing rather than silently working around at run time.
        if cfg.python_exe and "python" not in Path(cfg.python_exe).name.lower():
            notes.append(
                f"python_exe {cfg.python_exe!r} does not look like Python; "
                f"reset to {sys.executable!r}"
            )
            cfg.python_exe = sys.executable

        # The old Blender-only cart path is still a perfectly good car mesh, so
        # it is carried over rather than dropped with the rest.
        legacy_car = (data.get("cart_model_glb") or "").strip()
        if legacy_car and not cfg.car_mesh_file:
            cfg.car_mesh_file = legacy_car
            notes.append(f"car_mesh_file adopted from cart_model_glb: {legacy_car}")

    if from_version < 5:
        # "+X" was the old default rather than a considered choice, and it mounts
        # a car sideways whenever the model is longer on Y - which is common.
        # "auto" measures the mesh instead, and still resolves to +X for a car
        # that really is longest on X.
        if cfg.car_forward_axis == "+X":
            notes.append(
                "car_forward_axis: '+X' -> 'auto' (measures the mesh; the old "
                "default mounted a Y-long car across the track)"
            )
            cfg.car_forward_axis = "auto"

    if from_version < 4:
        dropped = sorted(REMOVED_KEYS & set(data))
        if dropped:
            notes.append(
                f"dropped {len(dropped)} obsolete setting(s): {', '.join(dropped)}"
            )

    if from_version < 7 and "lift_speed" not in data:
        # New in 7: climbs used to be caught by the min_speed floor, which made
        # the lift crawl and every ride read slow.
        notes.append(f"lift_speed: {cfg.lift_speed} m/s (lift hills are now driven, not coasted)")

    if from_version < 6 and "car_export_glb" not in data:
        # New in 6: the .glb is the only file Unreal imports as an AnimSequence,
        # so it is on by default even for configs saved before it existed.
        notes.append("car_export_glb: on (the .glb is what Unreal imports as animation)")

    if from_version < CONFIG_VERSION:
        cfg.config_version = CONFIG_VERSION

    cfg.state_migrations = notes
    return cfg


def save_state(cfg: PipelineConfig) -> None:
    APP_STATE_DIR.mkdir(parents=True, exist_ok=True)
    data = asdict(cfg)
    data.pop("state_migrations", None)
    STATE_FILE.write_text(json.dumps(data, indent=2), encoding="utf-8")


# Which converter flag each track-file extension maps to. NoLimits 2 formats
# carry their own banking, so neither needs a tangent file.
SOURCE_FLAGS = {
    ".nlelem": "--spline",
    ".nl2elem": "--nl2elem",
    ".csv": "--nl2-csv",
}


def source_flag_for(path: str) -> str:
    """The converter argument that suits this track file."""
    return SOURCE_FLAGS.get(Path(path).suffix.lower(), "--spline")


def needs_tangent(path: str) -> bool:
    """Only OpenFVD's binary format takes a separate tangent file."""
    return source_flag_for(path) == "--spline"


def build_converter_command(
    cfg: PipelineConfig, bundle_path: Path, csv_path: Path, python_exe: Path
) -> list[str]:
    args = [
        source_flag_for(cfg.spline_nlelem), str(cfg.spline_nlelem),
        "--output", str(bundle_path),
        "--csv", str(csv_path),
        "--samples-per-segment", str(cfg.samples_per_segment),
        "--resample-spacing-m", str(cfg.resample_spacing_m),
        "--axis-mapping", cfg.axis_mapping,
        "--initial-speed", str(cfg.initial_speed),
        "--lift-speed", str(cfg.lift_speed),
        "--rolling-friction", str(cfg.rolling_friction),
        "--drag-coeff", str(cfg.drag_coeff),
        "--curvature-window-s", str(cfg.curvature_window_s),
        "--curvature-baseline-m", str(cfg.curvature_baseline_m),
        "--segment-distortion-ratio", str(cfg.segment_distortion_ratio),
        "--tangent-break-threshold-deg", str(cfg.tangent_break_threshold_deg),
    ]

    if not cfg.car_export_fbx:
        args.append("--no-car-fbx")
    if not cfg.car_export_glb:
        args.append("--no-car-glb")
    if not cfg.track_export_fbx:
        args.append("--no-track-fbx")
    if not cfg.track_supports:
        args.append("--no-track-supports")
    args.extend([
        "--track-station-spacing-cm", str(cfg.track_station_spacing_cm),
        "--track-gauge-cm", str(cfg.track_gauge_cm),
        "--track-rail-drop-cm", str(cfg.track_rail_drop_cm),
        "--track-tie-spacing-cm", str(cfg.track_tie_spacing_cm),
        "--track-support-spacing-cm", str(cfg.track_support_spacing_cm),
    ])
    args.extend([
        "--car-fbx-fps", str(cfg.car_fbx_fps),
        "--car-fbx-import-fps", str(cfg.car_import_fps),
        "--car-forward-axis", cfg.car_forward_axis,
        "--car-scale", str(cfg.car_scale),
        "--car-expected-length-m", str(cfg.car_expected_length_m),
        "--car-offset-cm", "0", "0", str(cfg.car_offset_z_cm),
        "--car-rotation-offset-deg", "0", "0", str(cfg.car_yaw_offset_deg),
    ])
    if cfg.car_mesh_asset.strip():
        args.extend(["--car-mesh-asset", cfg.car_mesh_asset.strip()])
    if cfg.car_mesh_file.strip():
        args.extend(["--car-mesh-file", cfg.car_mesh_file.strip()])

    tangent = cfg.tangent_nlelem.strip()
    if tangent and needs_tangent(cfg.spline_nlelem):
        args.extend(["--tangent", tangent])

    reference = cfg.validate_reference_csv.strip()
    if reference and Path(reference).exists():
        args.extend(["--validate-reference-csv", reference])

    if not cfg.spike_filter_enabled:
        args.append("--disable-spike-filter")
    args.extend([
        "--spike-angle-threshold-deg", str(cfg.spike_angle_threshold_deg),
        "--spike-deviation-multiplier", str(cfg.spike_deviation_multiplier),
        "--spike-max-passes", str(cfg.spike_max_passes),
    ])

    if IS_FROZEN:
        return [str(sys.executable), "--internal-convert", *args]
    return [str(python_exe), str(CONVERTER_SCRIPT), *args]


def run_pipeline(cfg: PipelineConfig, log) -> int:
    for note in getattr(cfg, "state_migrations", None) or []:
        log(f"Migrated setting: {note}")

    spline = Path(cfg.spline_nlelem)
    out_dir = Path(cfg.output_dir)
    python_exe = Path(cfg.python_exe)

    if not cfg.spline_nlelem.strip():
        raise ValueError("Select a spline .nlelem file first.")
    if not spline.exists():
        raise FileNotFoundError(f"Spline file not found: {spline}")
    if not cfg.output_dir.strip():
        raise ValueError("Select an output folder first.")

    flag = source_flag_for(cfg.spline_nlelem)
    label = {
        "--spline": "OpenFVD element (.nlelem)",
        "--nl2elem": "NoLimits 2 element (.nl2elem)",
        "--nl2-csv": "NoLimits 2 spline export (.csv)",
    }[flag]
    log(f"Input format: {label}")

    tangent = cfg.tangent_nlelem.strip()
    if needs_tangent(cfg.spline_nlelem):
        if tangent and not Path(tangent).exists():
            raise FileNotFoundError(f"Tangent file not found: {tangent}")
    elif tangent:
        log("Tangent file ignored: this format carries its own banking.")

    if not IS_FROZEN:
        # Guard against a stale setting pointing at the packaged GUI itself,
        # which would spawn a second app window instead of converting.
        if "python" not in python_exe.name.lower():
            python_exe = Path(sys.executable)
            log(f"Python path did not look like Python; using {python_exe}")
        if not python_exe.exists():
            raise FileNotFoundError(f"Python executable not found: {python_exe}")

    out_dir.mkdir(parents=True, exist_ok=True)
    bundle_path = out_dir / "coaster_ue5_bundle.json"
    csv_path = out_dir / "coaster_timeline.csv"

    command = build_converter_command(cfg, bundle_path, csv_path, python_exe)
    log("Running converter...")
    log(" ".join(command))

    proc = subprocess.run(
        command, capture_output=True, text=True, encoding="utf-8", errors="replace"
    )
    for stream in (proc.stdout, proc.stderr):
        if stream and stream.strip():
            for line in stream.strip().splitlines():
                log(line)

    if proc.returncode != 0:
        raise RuntimeError(f"Conversion failed with exit code {proc.returncode}")

    log(f"Bundle:   {bundle_path}")
    log(f"Timeline: {csv_path}")
    log("")
    log("Next: run unreal_import_coaster.py inside the Unreal Editor and point")
    log("import_coaster_bundle() at the bundle above.")
    return 0


def resolve_fonts(root) -> None:
    """Pick the best available family for each role."""
    global FONT, FONT_TEXT, FONT_SMALL, MONO
    from tkinter import font as tkfont

    available = set(tkfont.families(root))

    def pick(role):
        for name in FONT_STACKS[role]:
            if name in available:
                return name
        return FONT_STACKS[role][-1]

    FONT = pick("display")
    FONT_TEXT = pick("text")
    FONT_SMALL = pick("small")
    MONO = pick("mono")


def _colorref(hex_colour: str) -> int:
    """Win32 COLORREF is 0x00BBGGRR, not RGB."""
    h = hex_colour.lstrip("#")
    r, g, b = int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
    return (b << 16) | (g << 8) | r


def style_window_chrome(root, caption: str, text: str, border: str) -> bool:
    """Recolour the native title bar so it continues the header band.

    The default light title bar is the "white border" around an otherwise dark
    window. Windows 11 lets it be recoloured directly, which keeps the real
    system window controls and snapping rather than faking a title bar.
    """
    if sys.platform != "win32":
        return False
    try:
        import ctypes
        from ctypes import wintypes

        root.update_idletasks()
        hwnd = ctypes.windll.user32.GetParent(root.winfo_id()) or root.winfo_id()
        dwm = ctypes.windll.dwmapi

        def attr(which: int, value: int) -> int:
            data = ctypes.c_int(value)
            return dwm.DwmSetWindowAttribute(
                wintypes.HWND(hwnd), ctypes.c_int(which),
                ctypes.byref(data), ctypes.c_int(ctypes.sizeof(data)),
            )

        # 20 = immersive dark mode, 34 = border, 35 = caption, 36 = caption text.
        # Dark mode works on Windows 10 2004+; the colours need Windows 11.
        attr(20, 1)
        ok = attr(35, _colorref(caption)) == 0
        attr(36, _colorref(text))
        attr(34, _colorref(border))
        return ok
    except Exception:
        return False


def build_app_icon(root):
    """A small purple-and-gold mark, so the window is not the Python feather."""
    import tkinter as tk

    size = 32
    img = tk.PhotoImage(master=root, width=size, height=size)
    bg = THEME["brand"]
    gold = THEME["accent"]
    deep = THEME["brand_deep"]

    rows = []
    for y in range(size):
        row = []
        for x in range(size):
            # Round the corners very slightly.
            corner = (
                (x < 2 and y < 2) or (x > size - 3 and y < 2)
                or (x < 2 and y > size - 3) or (x > size - 3 and y > size - 3)
            )
            row.append(deep if corner else bg)
        rows.append(row)

    # A coaster hill: rise, crest, drop.
    import math

    for x in range(3, size - 3):
        t = (x - 3) / float(size - 7)
        hill = 0.5 * (1.0 - math.cos(math.pi * min(t * 1.45, 1.0)))
        y = int(round(size - 8 - hill * (size - 17)))
        # Three pixels thick: a hairline vanishes at the 16px title-bar size.
        for dy in (-2, -1, 0):
            yy = max(0, min(size - 1, y + dy))
            rows[yy][x] = gold
    # A solid gold rail along the base anchors the mark at small sizes.
    for x in range(3, size - 3):
        for y in (size - 6, size - 5):
            rows[y][x] = gold

    img.put(" ".join("{" + " ".join(r) + "}" for r in rows))
    return img


def enable_hidpi() -> float:
    """Opt into real-pixel rendering and return the display scale factor.

    Without this, Windows bitmap-stretches a DPI-unaware window on a scaled
    display, which makes every glyph and border soft. Declaring awareness lets
    Tk draw at native resolution; the returned factor is used to scale the
    layout back up so it occupies the same physical size.
    """
    if sys.platform != "win32":
        return 1.0
    try:
        import ctypes

        try:
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except (AttributeError, OSError):
            ctypes.windll.user32.SetProcessDPIAware()

        dc = ctypes.windll.user32.GetDC(0)
        try:
            # LOGPIXELSX = 88
            dpi = ctypes.windll.gdi32.GetDeviceCaps(dc, 88)
        finally:
            ctypes.windll.user32.ReleaseDC(0, dc)
        return max(1.0, dpi / 96.0)
    except Exception:
        return 1.0


def headless_smoke_test() -> int:
    cfg = load_state()
    print("GUI smoke test passed")
    print(f"Spline path: {cfg.spline_nlelem}")
    print(f"Axis mapping: {cfg.axis_mapping}")
    return 0


def launch_gui() -> None:
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk

    cfg = load_state()
    c = THEME
    scale = enable_hidpi()

    def px(value: int) -> int:
        """Scale a pixel measurement to the display."""
        return max(1, int(round(value * scale)))

    root = tk.Tk()
    root.title(APP_NAME)
    # Point-sized fonts follow this; pixel measurements go through px().
    root.tk.call("tk", "scaling", 96.0 * scale / 72.0)
    resolve_fonts(root)
    root.configure(bg=c["bg"])

    style = ttk.Style()
    style.theme_use("clam")

    style.configure("TFrame", background=c["bg"])
    style.configure("Surface.TFrame", background=c["surface"])
    style.configure("Brand.TFrame", background=c["brand"])

    style.configure(
        "Title.TLabel",
        background=c["brand"], foreground="#FFFFFF", font=(FONT, 20, "bold"),
    )
    style.configure(
        "Subtitle.TLabel",
        background=c["brand"], foreground="#D6C4EE", font=(FONT_TEXT, 10),
    )
    style.configure(
        "Section.TLabel",
        background=c["surface"], foreground=c["accent"], font=(FONT_SMALL, 9, "bold"),
    )
    style.configure(
        "Field.TLabel",
        background=c["surface"], foreground=c["fg"], font=(FONT_TEXT, 10),
    )
    style.configure(
        "Hint.TLabel",
        background=c["surface"], foreground=c["muted"], font=(FONT_SMALL, 9),
    )
    style.configure(
        "Footer.TLabel",
        background=c["bg"], foreground=c["faint"], font=(FONT_SMALL, 9),
    )


    style.configure(
        "Field.TEntry",
        fieldbackground=c["field"], foreground=c["fg"], insertcolor=c["accent"],
        bordercolor=c["line"], lightcolor=c["line"], darkcolor=c["line"],
        borderwidth=1, padding=px(9), font=(FONT_TEXT, 10),
    )
    style.map(
        "Field.TEntry",
        fieldbackground=[("focus", c["field_focus"])],
        # Gold focus ring, so the active field is obvious without a heavy border.
        bordercolor=[("focus", c["accent"])],
        lightcolor=[("focus", c["accent"])],
        darkcolor=[("focus", c["accent"])],
    )
    style.configure(
        "Field.TCombobox",
        fieldbackground=c["field"], background=c["field"], foreground=c["fg"],
        bordercolor=c["line"], lightcolor=c["line"], darkcolor=c["line"],
        arrowcolor=c["accent"], arrowsize=px(15), padding=px(8),
        borderwidth=1, font=(FONT_TEXT, 10),
    )
    style.map(
        "Field.TCombobox",
        fieldbackground=[("readonly", c["field"])],
        selectbackground=[("readonly", c["field"])],
        selectforeground=[("readonly", c["fg"])],
        bordercolor=[("focus", c["accent"])],
        lightcolor=[("focus", c["accent"])],
        darkcolor=[("focus", c["accent"])],
    )
    style.configure(
        "Field.TSpinbox",
        fieldbackground=c["field"], background=c["field"], foreground=c["fg"],
        bordercolor=c["line"], lightcolor=c["line"], darkcolor=c["line"],
        arrowcolor=c["accent"], arrowsize=px(13), padding=px(7),
        borderwidth=1, font=(FONT_TEXT, 10),
    )
    style.configure(
        "Ghost.TButton",
        background=c["surface_alt"], foreground=c["fg"],
        bordercolor=c["hairline"], lightcolor=c["hairline"],
        darkcolor=c["hairline"],
        borderwidth=1, focusthickness=0, padding=(px(15), px(9)),
        font=(FONT_TEXT, 10),
    )
    style.map(
        "Ghost.TButton",
        background=[("pressed", c["brand_deep"]), ("active", c["brand"]),
                    ("disabled", c["surface"])],
        bordercolor=[("active", c["brand_light"])],
        lightcolor=[("active", c["brand_light"])],
        darkcolor=[("active", c["brand_light"])],
        foreground=[("disabled", c["faint"])],
    )
    style.configure(
        "Accent.TButton",
        background=c["accent"], foreground=c["accent_ink"],
        bordercolor=c["accent"], lightcolor=c["accent"], darkcolor=c["accent"],
        borderwidth=0, focusthickness=0, padding=(px(28), px(11)),
        font=(FONT, 11, "bold"),
    )
    style.map(
        "Accent.TButton",
        background=[("pressed", "#E8BE18"), ("active", c["accent_hover"]),
                    ("disabled", c["surface_alt"])],
        foreground=[("disabled", c["faint"])],
    )
    style.configure(
        "Brand.Horizontal.TProgressbar",
        background=c["accent"], troughcolor=c["surface"],
        bordercolor=c["surface"], lightcolor=c["accent"], darkcolor=c["accent"],
    )

    # ---------------- header ----------------
    header = tk.Frame(root, bg=c["brand"])
    header.pack(fill=tk.X)
    header_inner = tk.Frame(header, bg=c["brand"])
    header_inner.pack(fill=tk.X, padx=px(28), pady=(px(20), px(18)))

    titles = tk.Frame(header_inner, bg=c["brand"])
    titles.pack(side=tk.LEFT)
    ttk.Label(titles, text="UE5 Coaster Pipeline", style="Title.TLabel").pack(anchor="w")
    ttk.Label(
        titles,
        text="NoLimits 2 / OpenFVD  →  Unreal Engine 5",
        style="Subtitle.TLabel",
    ).pack(anchor="w", pady=(3, 0))

    status_var = tk.StringVar(value="Ready")
    status_pill = tk.Label(
        header_inner,
        textvariable=status_var,
        bg=c["pill"], fg="#FFFFFF",
        font=(FONT_SMALL, 9, "bold"), padx=px(15), pady=px(7),
    )
    status_pill.pack(side=tk.RIGHT)

    # LSU gold rule under the purple band.
    tk.Frame(root, bg=c["accent"], height=px(3)).pack(fill=tk.X)
    tk.Frame(root, bg=c["brand_deep"], height=px(1)).pack(fill=tk.X)

    body = tk.Frame(root, bg=c["bg"])
    body.pack(fill=tk.BOTH, expand=True, padx=px(22), pady=px(18))

    vars_dict = {
        "spline_nlelem": tk.StringVar(value=cfg.spline_nlelem),
        "tangent_nlelem": tk.StringVar(value=cfg.tangent_nlelem),
        "output_dir": tk.StringVar(value=cfg.output_dir),
        "validate_reference_csv": tk.StringVar(value=cfg.validate_reference_csv),
        "python_exe": tk.StringVar(value=cfg.python_exe),
        "initial_speed": tk.DoubleVar(value=cfg.initial_speed),
        "lift_speed": tk.DoubleVar(value=cfg.lift_speed),
        "rolling_friction": tk.DoubleVar(value=cfg.rolling_friction),
        "drag_coeff": tk.DoubleVar(value=cfg.drag_coeff),
        "curvature_window_s": tk.DoubleVar(value=cfg.curvature_window_s),
        "curvature_baseline_m": tk.DoubleVar(value=cfg.curvature_baseline_m),
        "car_mesh_asset": tk.StringVar(value=cfg.car_mesh_asset),
        "car_mesh_file": tk.StringVar(value=cfg.car_mesh_file),
        "car_forward_axis": tk.StringVar(value=cfg.car_forward_axis),
        "car_scale": tk.DoubleVar(value=cfg.car_scale),
        "car_offset_z_cm": tk.DoubleVar(value=cfg.car_offset_z_cm),
        "car_yaw_offset_deg": tk.DoubleVar(value=cfg.car_yaw_offset_deg),
        "car_expected_length_m": tk.DoubleVar(value=cfg.car_expected_length_m),
        "car_export_fbx": tk.BooleanVar(value=cfg.car_export_fbx),
        "car_export_glb": tk.BooleanVar(value=cfg.car_export_glb),
        "car_fbx_fps": tk.IntVar(value=cfg.car_fbx_fps),
        "car_import_fps": tk.IntVar(value=cfg.car_import_fps),
        "track_export_fbx": tk.BooleanVar(value=cfg.track_export_fbx),
        "track_station_spacing_cm": tk.DoubleVar(value=cfg.track_station_spacing_cm),
        "track_gauge_cm": tk.DoubleVar(value=cfg.track_gauge_cm),
        "track_rail_drop_cm": tk.DoubleVar(value=cfg.track_rail_drop_cm),
        "track_tie_spacing_cm": tk.DoubleVar(value=cfg.track_tie_spacing_cm),
        "track_supports": tk.BooleanVar(value=cfg.track_supports),
        "track_support_spacing_cm": tk.DoubleVar(value=cfg.track_support_spacing_cm),
        "axis_mapping": tk.StringVar(value=cfg.axis_mapping),
        "samples_per_segment": tk.IntVar(value=cfg.samples_per_segment),
        "resample_spacing_m": tk.DoubleVar(value=cfg.resample_spacing_m),
        "spike_filter_enabled": tk.BooleanVar(value=cfg.spike_filter_enabled),
        "spike_angle_threshold_deg": tk.DoubleVar(value=cfg.spike_angle_threshold_deg),
        "spike_deviation_multiplier": tk.DoubleVar(value=cfg.spike_deviation_multiplier),
        "spike_max_passes": tk.IntVar(value=cfg.spike_max_passes),
        "segment_distortion_ratio": tk.DoubleVar(value=cfg.segment_distortion_ratio),
        "tangent_break_threshold_deg": tk.DoubleVar(value=cfg.tangent_break_threshold_deg),
    }

    # A hand-rolled tab strip rather than ttk.Notebook: clam shifts the selected
    # tab's baseline and draws a client border that cannot be styled away.
    tab_bar = tk.Frame(body, bg=c["bg"])
    tab_bar.pack(fill=tk.X)
    # The card absorbs spare height so it reads the same on every tab; letting
    # the log take it instead left a large empty well on the shorter tabs.
    tab_host = tk.Frame(body, bg=c["surface"])
    tab_host.pack(fill=tk.BOTH, expand=True)

    tabs: list[dict] = []

    def select_tab(index: int):
        for i, tab in enumerate(tabs):
            active = i == index
            tab["label"].configure(
                fg=c["accent"] if active else c["muted"],
                bg=c["surface"] if active else c["bg"],
            )
            tab["rule"].configure(bg=c["accent"] if active else c["bg"])
            if active:
                tab["page"].pack(fill=tk.BOTH, expand=True)
            else:
                tab["page"].pack_forget()

    def make_tab(title):
        index = len(tabs)
        holder = tk.Frame(tab_bar, bg=c["bg"])
        holder.pack(side=tk.LEFT)
        label = tk.Label(
            holder, text=title, bg=c["bg"], fg=c["muted"],
            font=(FONT_TEXT, 10), padx=px(21), pady=px(11), cursor="hand2",
        )
        label.pack(fill=tk.X)
        rule = tk.Frame(holder, bg=c["bg"], height=px(3))
        rule.pack(fill=tk.X)

        def on_enter(_e, i=index):
            if tabs[i]["page"].winfo_ismapped():
                return
            tabs[i]["label"].configure(bg=c["surface_hover"], fg=c["fg"])

        def on_leave(_e, i=index):
            if tabs[i]["page"].winfo_ismapped():
                return
            tabs[i]["label"].configure(bg=c["bg"], fg=c["muted"])

        label.bind("<Enter>", on_enter)
        label.bind("<Leave>", on_leave)

        page = ttk.Frame(tab_host, style="Surface.TFrame", padding=(px(24), px(18)))
        page.columnconfigure(0, weight=1, uniform="col")
        page.columnconfigure(1, weight=1, uniform="col")

        label.bind("<Button-1>", lambda _e, i=index: select_tab(i))
        tabs.append({"label": label, "rule": rule, "page": page})
        return page

    def field_cell(parent, row, col, label, hint):
        """One label + hint slot. Returns the frame the control goes into."""
        cell = ttk.Frame(parent, style="Surface.TFrame")
        cell.grid(
            row=row, column=col, sticky="new",
            padx=(0, px(20)) if col == 0 else (px(20), 0), pady=(0, px(12)),
        )
        cell.columnconfigure(0, weight=1)
        ttk.Label(cell, text=label, style="Field.TLabel").grid(
            row=0, column=0, sticky="w", pady=(0, px(5))
        )
        holder = ttk.Frame(cell, style="Surface.TFrame")
        holder.grid(row=1, column=0, sticky="ew")
        ttk.Label(cell, text=hint, style="Hint.TLabel", wraplength=px(430)).grid(
            row=2, column=0, sticky="w", pady=(px(5), 0)
        )
        return holder

    def add_path_row(parent, row, label, key, hint, mode="file", types=None):
        holder = ttk.Frame(parent, style="Surface.TFrame")
        holder.grid(row=row, column=0, columnspan=2, sticky="ew", pady=(0, px(12)))
        holder.columnconfigure(0, weight=1)

        ttk.Label(holder, text=label, style="Field.TLabel").grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, px(5))
        )
        ttk.Entry(
            holder, textvariable=vars_dict[key], style="Field.TEntry"
        ).grid(row=1, column=0, sticky="ew")

        def browse():
            picked = (
                filedialog.askdirectory()
                if mode == "dir"
                else filedialog.askopenfilename(filetypes=types or [("All", "*.*")])
            )
            if picked:
                vars_dict[key].set(picked)

        ttk.Button(holder, text="Browse", command=browse, style="Ghost.TButton").grid(
            row=1, column=1, padx=(px(10), 0)
        )
        ttk.Label(holder, text=hint, style="Hint.TLabel", wraplength=px(900)).grid(
            row=2, column=0, columnspan=2, sticky="w", pady=(px(5), 0)
        )

    def add_value_row(parent, row, col, label, key, hint, widget="entry", **kw):
        holder = field_cell(parent, row, col, label, hint)
        if widget == "combo":
            ttk.Combobox(
                holder, textvariable=vars_dict[key], values=kw["values"],
                state="readonly", style="Field.TCombobox",
            ).pack(fill=tk.X)
        elif widget == "spin":
            ttk.Spinbox(
                holder, textvariable=vars_dict[key], from_=kw["from_"], to=kw["to"],
                increment=kw.get("increment", 1), style="Field.TSpinbox",
            ).pack(fill=tk.X)
        else:
            ttk.Entry(
                holder, textvariable=vars_dict[key], style="Field.TEntry"
            ).pack(fill=tk.X)

    def add_toggle(parent, row, text, key):
        """A hand-drawn checkbox: clam's indicator does not scale cleanly."""
        var = vars_dict[key]
        holder = tk.Frame(parent, bg=c["surface"], cursor="hand2")
        holder.grid(
            row=row, column=0, columnspan=2, sticky="w", pady=(0, px(14))
        )

        box = tk.Label(
            holder, text="", width=2, font=(FONT, 10, "bold"),
            bd=0, highlightthickness=0,
        )
        box.pack(side=tk.LEFT)
        label = tk.Label(
            holder, text=text, bg=c["surface"], fg=c["fg"], font=(FONT_TEXT, 10)
        )
        label.pack(side=tk.LEFT, padx=(px(10), 0))

        def refresh():
            on = bool(var.get())
            box.configure(
                text="✓" if on else "",
                bg=c["accent"] if on else c["field"],
                fg=c["accent_ink"] if on else c["field"],
            )

        def toggle(_event=None):
            var.set(not var.get())
            refresh()

        for widget in (holder, box, label):
            widget.bind("<Button-1>", toggle)
        refresh()

    def add_section(parent, row, title, blurb):
        head = ttk.Frame(parent, style="Surface.TFrame")
        head.grid(row=row, column=0, columnspan=2, sticky="ew", pady=(px(4), px(12)))
        ttk.Label(head, text=title, style="Section.TLabel").pack(anchor="w")
        ttk.Label(head, text=blurb, style="Hint.TLabel", wraplength=px(900)).pack(
            anchor="w", pady=(px(3), 0)
        )

    # ---------------- Files ----------------
    files = make_tab("Files")
    add_path_row(
        files, 0, "Track file", "spline_nlelem",
        "The track from OpenFVD (.nlelem) or NoLimits 2 (.nl2elem or a spline "
        ".csv). Required.",
        "file",
        [
            ("Track files", "*.nlelem *.nl2elem *.csv"),
            ("OpenFVD element", "*.nlelem"),
            ("NoLimits 2 element", "*.nl2elem"),
            ("NoLimits 2 spline CSV", "*.csv"),
            ("All", "*.*"),
        ],
    )
    add_path_row(
        files, 1, "Tangent file", "tangent_nlelem",
        "Only used with an OpenFVD .nlelem. NoLimits 2 files carry their own "
        "banking. Optional.",
        "file", [("NLElem", "*.nlelem"), ("All", "*.*")],
    )
    add_path_row(
        files, 2, "Output folder", "output_dir",
        "Where the converted files are saved.", "dir",
    )
    add_path_row(
        files, 3, "Reference CSV", "validate_reference_csv",
        "Optional check. A spline exported from Unreal, in centimetres, of this "
        "same ride. Not the NoLimits CSV above - that is the input. Leave blank "
        "if unsure.",
        "file", [("CSV", "*.csv"), ("All", "*.*")],
    )

    # ---------------- Car ----------------
    car = make_tab("Car")
    add_path_row(
        car, 0, "Car mesh file", "car_mesh_file",
        "The 3D model file for the coaster car (.fbx, .glb or .obj).",
        "file",
        [("Mesh", "*.fbx *.glb *.gltf *.obj"), ("All", "*.*")],
    )
    add_value_row(
        car, 1, 0, "Car mesh asset", "car_mesh_asset",
        "The car's path inside Unreal, if it is already in your project. "
        "Optional.",
    )
    add_value_row(
        car, 1, 1, "Faces along", "car_forward_axis",
        "Which way the car model faces. Leave on auto unless the car drives "
        "sideways.",
        "combo", values=["auto", "+X", "-X", "+Y", "-Y"],
    )
    add_value_row(
        car, 2, 0, "Extra yaw (deg)", "car_yaw_offset_deg",
        "Extra turn for the car, if it sits crooked.",
    )
    add_value_row(
        car, 2, 1, "Scale", "car_scale",
        "Size multiplier for the car. 1.0 is its original size.",
    )
    add_value_row(
        car, 3, 0, "Height offset (cm)", "car_offset_z_cm",
        "Raises or lowers the car relative to the track.",
    )
    add_value_row(
        car, 3, 1, "Expected length (m)", "car_expected_length_m",
        "The car's real length, used as a size check. 0 to skip.",
    )
    add_section(
        car, 4, "BAKED ANIMATION",
        "Saves the car's movement along the track as an animation file.",
    )
    add_toggle(
        car, 5, "Export CoasterCarAnimated.glb  (import this one into Unreal)",
        "car_export_glb",
    )
    add_toggle(car, 6, "Export CoasterCarAnimated.fbx  (for other tools)", "car_export_fbx")
    add_value_row(
        car, 7, 0, "Animation FPS", "car_fbx_fps",
        "Frames per second for the animation file.",
        "spin", from_=15, to=240, increment=15,
    )
    add_value_row(
        car, 8, 0, "Unreal animation FPS", "car_import_fps",
        "Your Unreal project's Default Frame Rate, under Project Settings > "
        "Animation. The animation is cut to end on a whole frame at this rate, "
        "because Unreal rejects one that is not frame-aligned.",
        "spin", from_=15, to=240, increment=15,
    )

    # ---------------- Track ----------------
    track = make_tab("Track")
    add_section(
        track, 0, "PROCEDURAL TRACK",
        "Builds rails, a spine, crossties and support columns along the ride "
        "and saves them as CoasterTrack.fbx. Generated geometry, not the "
        "original model.",
    )
    add_toggle(track, 1, "Export CoasterTrack.fbx", "track_export_fbx")
    add_value_row(
        track, 2, 0, "Rail gauge (cm)", "track_gauge_cm",
        "Distance between the two rails.",
    )
    add_value_row(
        track, 2, 1, "Rail drop (cm)", "track_rail_drop_cm",
        "How far the rails sit below the ride path.",
    )
    add_value_row(
        track, 3, 0, "Detail spacing (cm)", "track_station_spacing_cm",
        "Distance between cross-sections. Smaller is smoother and heavier.",
    )
    add_value_row(
        track, 3, 1, "Tie spacing (cm)", "track_tie_spacing_cm",
        "Distance between crossties.",
    )
    add_toggle(track, 4, "Include support columns", "track_supports")
    add_value_row(
        track, 5, 0, "Support spacing (cm)", "track_support_spacing_cm",
        "Distance between support columns.",
    )

    # ---------------- Physics ----------------
    physics = make_tab("Physics")
    add_value_row(
        physics, 0, 0, "Initial speed (m/s)", "initial_speed",
        "How fast the car is moving at the start of the ride.",
    )
    add_value_row(
        physics, 0, 1, "Rolling friction", "rolling_friction",
        "Speed lost to the wheels. Higher slows the ride down.",
    )
    add_value_row(
        physics, 1, 0, "Drag coefficient", "drag_coeff",
        "Speed lost to air resistance. Higher slows the ride down.",
    )
    add_value_row(
        physics, 1, 1, "Curvature window (s)", "curvature_window_s",
        "Time window used to measure how sharply the track bends.",
    )
    add_value_row(
        physics, 2, 0, "Curvature floor (m)", "curvature_baseline_m",
        "Shortest distance used to measure how sharply the track bends.",
    )
    add_value_row(
        physics, 2, 1, "Lift speed (m/s)", "lift_speed",
        "How fast the chain or launch drives the car up a hill. Gravity alone "
        "cannot get a car up a lift, so without this the climb crawls and the "
        "whole ride comes out slower than it should. Set 0 for gravity only.",
    )

    # ---------------- Geometry ----------------
    geometry = make_tab("Geometry")
    add_value_row(
        geometry, 0, 0, "Axis mapping", "axis_mapping",
        "How the source axes line up with Unreal's. Leave this alone unless "
        "the track comes in mirrored.",
        "combo",
        values=["nl2_to_ue_swap_yz", "nl2_to_ue", "nl2_to_ue_flip_y", "identity"],
    )
    add_value_row(
        geometry, 0, 1, "Samples per segment", "samples_per_segment",
        "How many points to take per piece of track. Higher is more detailed.",
        "spin", from_=4, to=200,
    )
    add_value_row(
        geometry, 1, 0, "Resample spacing (m)", "resample_spacing_m",
        "Spacing between points along the track.",
    )
    add_section(
        geometry, 2, "TRACK MESH CLEANUP",
        "Smooths rough spots out of the track shape used for display. The "
        "physics numbers are not affected. Settings are under Advanced.",
    )
    add_toggle(
        geometry, 3, "Smooth extreme spline spikes and outliers",
        "spike_filter_enabled",
    )

    # ---------------- Advanced ----------------
    advanced = make_tab("Advanced")
    adv = 0
    if not IS_FROZEN:
        add_path_row(
            advanced, adv, "Python executable", "python_exe",
            "The Python program used to run the converter.",
            "file", [("Python", "python*.exe"), ("All", "*.*")],
        )
        adv += 1

    add_section(
        advanced, adv, "SOURCE DEFECT DETECTION",
        "Parts of the track that look broken in the source file are marked, "
        "not silently repaired.",
    )
    adv += 1
    add_value_row(
        advanced, adv, 0, "Segment distortion", "segment_distortion_ratio",
        "How bent a piece of track must be before it is marked as broken.",
    )
    add_value_row(
        advanced, adv, 1, "Tangent break (deg)", "tangent_break_threshold_deg",
        "How sharp a corner must be before it is marked as broken.",
    )
    adv += 1

    add_section(
        advanced, adv, "SPIKE FILTER TUNING",
        "Used only when track mesh cleanup is turned on, on the Geometry tab.",
    )
    adv += 1
    add_value_row(
        advanced, adv, 0, "Angle threshold (deg)", "spike_angle_threshold_deg",
        "How sharp a kink must be before it is smoothed.",
    )
    add_value_row(
        advanced, adv, 1, "Deviation multiplier", "spike_deviation_multiplier",
        "How far a point must sit off the track before it is smoothed.",
    )
    adv += 1
    add_value_row(
        advanced, adv, 0, "Max passes", "spike_max_passes",
        "How many times to repeat the smoothing.", "spin", from_=1, to=10,
    )

    # ---------------- actions ----------------
    actions = tk.Frame(body, bg=c["bg"])
    actions.pack(fill=tk.X, pady=(18, 12))

    run_btn = ttk.Button(actions, text="Run Conversion", style="Accent.TButton")
    run_btn.pack(side=tk.LEFT)
    save_btn = ttk.Button(actions, text="Save Settings", style="Ghost.TButton")
    save_btn.pack(side=tk.LEFT, padx=(10, 0))
    open_btn = ttk.Button(actions, text="Open Output", style="Ghost.TButton")
    open_btn.pack(side=tk.LEFT, padx=(10, 0))
    ttk.Label(actions, text="Ctrl+Enter to run", style="Footer.TLabel").pack(
        side=tk.RIGHT, pady=(10, 0)
    )

    progress = ttk.Progressbar(
        body, mode="indeterminate", style="Brand.Horizontal.TProgressbar"
    )

    log_head = tk.Frame(body, bg=c["bg"])
    log_head.pack(fill=tk.X)
    tk.Label(
        log_head, text="RUN LOG", bg=c["bg"], fg=c["accent"],
        font=(FONT_SMALL, 9, "bold"),
    ).pack(side=tk.LEFT)
    clear_btn = ttk.Button(log_head, text="Clear", style="Ghost.TButton")
    clear_btn.pack(side=tk.RIGHT)

    log_wrap = tk.Frame(body, bg=c["hairline"], padx=1, pady=1)  # hairline border
    log_wrap.pack(fill=tk.X, pady=(px(8), 0))

    log_text = tk.Text(
        log_wrap, wrap="word", height=7,
        bg=c["field"], fg=c["fg"], insertbackground=c["accent"],
        selectbackground=c["brand"], selectforeground="#FFFFFF",
        relief=tk.FLAT, borderwidth=0, highlightthickness=0,
        font=(MONO, 10), padx=px(16), pady=px(13),
        spacing1=px(1), spacing3=px(2),
    )
    log_scroll = tk.Scrollbar(
        log_wrap, command=log_text.yview, bg=c["surface_alt"],
        troughcolor=c["field"], activebackground=c["accent"],
        highlightthickness=0, bd=0, width=px(12), relief=tk.FLAT,
    )
    log_text.configure(yscrollcommand=log_scroll.set)
    log_scroll.pack(side=tk.RIGHT, fill=tk.Y)
    log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    log_text.tag_configure("warn", foreground=c["warn"])
    log_text.tag_configure("error", foreground=c["error"])
    log_text.tag_configure("ok", foreground=c["ok"])
    log_text.tag_configure("muted", foreground=c["muted"])

    def classify(line: str) -> str:
        upper = line.upper()
        if "ERROR" in upper or "FAIL" in upper:
            return "error"
        if "WARNING" in upper or upper.startswith("MIGRATED"):
            return "warn"
        if upper.startswith("OK:") or "-> PASS" in upper or "RESULT: PASS" in upper:
            return "ok"
        if line.startswith("  ") or line.startswith("---"):
            return "muted"
        return ""

    def append_log(msg: str):
        for line in str(msg).splitlines() or [""]:
            log_text.insert(tk.END, line + "\n", classify(line))
        log_text.see(tk.END)

    queue_out = queue.Queue()
    running = {"busy": False}

    def set_status(text: str, colour: str):
        status_var.set(text)
        status_pill.configure(bg=colour)

    set_status("Ready", c["pill"])

    def collect_cfg() -> PipelineConfig:
        def num(key, cast):
            try:
                return cast(vars_dict[key].get())
            except (tk.TclError, ValueError):
                raise ValueError(f"'{key.replace('_', ' ')}' is not a valid number.")

        return PipelineConfig(
            spline_nlelem=vars_dict["spline_nlelem"].get().strip(),
            tangent_nlelem=vars_dict["tangent_nlelem"].get().strip(),
            output_dir=vars_dict["output_dir"].get().strip(),
            validate_reference_csv=vars_dict["validate_reference_csv"].get().strip(),
            python_exe=vars_dict["python_exe"].get().strip() or sys.executable,
            initial_speed=num("initial_speed", float),
            lift_speed=num("lift_speed", float),
            rolling_friction=num("rolling_friction", float),
            drag_coeff=num("drag_coeff", float),
            curvature_window_s=num("curvature_window_s", float),
            curvature_baseline_m=num("curvature_baseline_m", float),
            car_mesh_asset=vars_dict["car_mesh_asset"].get().strip(),
            car_mesh_file=vars_dict["car_mesh_file"].get().strip(),
            car_forward_axis=vars_dict["car_forward_axis"].get().strip(),
            car_scale=num("car_scale", float),
            car_offset_z_cm=num("car_offset_z_cm", float),
            car_yaw_offset_deg=num("car_yaw_offset_deg", float),
            car_expected_length_m=num("car_expected_length_m", float),
            car_export_fbx=bool(vars_dict["car_export_fbx"].get()),
            car_export_glb=bool(vars_dict["car_export_glb"].get()),
            car_fbx_fps=num("car_fbx_fps", int),
            car_import_fps=num("car_import_fps", int),
            track_export_fbx=bool(vars_dict["track_export_fbx"].get()),
            track_station_spacing_cm=num("track_station_spacing_cm", float),
            track_gauge_cm=num("track_gauge_cm", float),
            track_rail_drop_cm=num("track_rail_drop_cm", float),
            track_tie_spacing_cm=num("track_tie_spacing_cm", float),
            track_supports=bool(vars_dict["track_supports"].get()),
            track_support_spacing_cm=num("track_support_spacing_cm", float),
            axis_mapping=vars_dict["axis_mapping"].get().strip(),
            samples_per_segment=num("samples_per_segment", int),
            resample_spacing_m=num("resample_spacing_m", float),
            spike_filter_enabled=bool(vars_dict["spike_filter_enabled"].get()),
            spike_angle_threshold_deg=num("spike_angle_threshold_deg", float),
            spike_deviation_multiplier=num("spike_deviation_multiplier", float),
            spike_max_passes=num("spike_max_passes", int),
            segment_distortion_ratio=num("segment_distortion_ratio", float),
            tangent_break_threshold_deg=num("tangent_break_threshold_deg", float),
        )

    def background_run(run_cfg: PipelineConfig):
        try:
            save_state(run_cfg)
            run_pipeline(run_cfg, lambda m: queue_out.put(("log", m)))
            queue_out.put(("done", "Success"))
        except Exception as ex:
            queue_out.put(("error", str(ex)))

    def on_run():
        if running["busy"]:
            return
        try:
            run_cfg = collect_cfg()
        except ValueError as ex:
            messagebox.showerror(APP_NAME, str(ex))
            return

        if run_cfg.axis_mapping in MIRRORED_MAPPINGS and not messagebox.askyesno(
            APP_NAME,
            f"Axis mapping '{run_cfg.axis_mapping}' has determinant +1.\n\n"
            "It cannot convert right-handed source data into left-handed Unreal "
            "space, so the track will come out mirrored: left-hand turns become "
            "right-hand and lateral-G signs flip.\n\nContinue anyway?",
        ):
            return

        running["busy"] = True
        run_btn.configure(state=tk.DISABLED)
        set_status("Running", "#7B4AC6")
        progress.pack(fill=tk.X, pady=(0, 12), before=log_head)
        progress.start(12)
        append_log("Starting conversion...")
        threading.Thread(target=background_run, args=(run_cfg,), daemon=True).start()

    def finish(status_text: str, colour: str):
        running["busy"] = False
        run_btn.configure(state=tk.NORMAL)
        set_status(status_text, colour)
        progress.stop()
        progress.pack_forget()

    def on_save():
        try:
            save_state(collect_cfg())
        except ValueError as ex:
            messagebox.showerror(APP_NAME, str(ex))
            return
        append_log("Settings saved.")

    def on_open_output():
        out = Path(vars_dict["output_dir"].get().strip())
        if not out.exists():
            messagebox.showwarning(APP_NAME, "Output folder does not exist yet.")
            return
        if sys.platform == "win32":
            subprocess.Popen(["explorer", str(out)])
        elif sys.platform == "darwin":
            subprocess.Popen(["open", str(out)])
        else:
            subprocess.Popen(["xdg-open", str(out)])

    run_btn.configure(command=on_run)
    save_btn.configure(command=on_save)
    open_btn.configure(command=on_open_output)
    clear_btn.configure(command=lambda: log_text.delete("1.0", tk.END))

    def poll_queue():
        while True:
            try:
                kind, payload = queue_out.get_nowait()
            except queue.Empty:
                break

            if kind == "log":
                append_log(payload)
            elif kind == "done":
                append_log("Finished successfully.")
                finish("Done", "#2E7A4E")
            elif kind == "error":
                append_log(f"ERROR: {payload}")
                finish("Failed", "#8C2C3E")
                messagebox.showerror(APP_NAME, payload)

        root.after(120, poll_queue)

    for note in cfg.state_migrations:
        append_log(f"Migrated setting: {note}")

    # Measure every tab, not just the first, so switching to a taller one later
    # cannot squeeze the run log down to a single line.
    tallest = 0
    for i in range(len(tabs)):
        select_tab(i)
        root.update_idletasks()
        tallest = max(tallest, root.winfo_reqheight())
    select_tab(0)
    root.update_idletasks()

    screen_w, screen_h = root.winfo_screenwidth(), root.winfo_screenheight()
    win_w = max(px(900), min(px(1120), screen_w - px(80)))
    win_h = max(px(620), min(tallest, screen_h - px(120)))
    root.geometry(
        f"{win_w}x{win_h}+{max(0, (screen_w - win_w) // 2)}"
        f"+{max(0, (screen_h - win_h) // 3)}"
    )
    root.minsize(px(880), px(600))

    # Match the native title bar to the header band, and replace the default
    # interpreter icon. Both are cosmetic and silently skipped if unsupported.
    try:
        root.iconphoto(True, build_app_icon(root))
    except Exception:
        pass
    if not style_window_chrome(root, c["brand"], "#FFFFFF", c["brand"]):
        # Windows 10 gets dark mode but not caption colours; nothing to do.
        pass

    root.bind("<Control-Return>", lambda _e: on_run())
    root.after(120, poll_queue)
    root.mainloop()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="UE5 coaster pipeline GUI")
    parser.add_argument("--internal-convert", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument(
        "--smoke-test", action="store_true",
        help="Validate startup without opening a window",
    )
    parser.add_argument(
        "--headless-run", action="store_true",
        help="Convert using saved settings without opening a window",
    )
    args, extra = parser.parse_known_args()
    setattr(args, "extra_args", extra)
    return args


def main() -> int:
    args = parse_args()
    if args.internal_convert:
        import convert_nlelem_to_ue as converter_module

        sys.argv = [str(CONVERTER_SCRIPT), *args.extra_args]
        converter_module.main()
        return 0

    if args.smoke_test:
        return headless_smoke_test()

    if args.headless_run:
        return run_pipeline(load_state(), print)

    launch_gui()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
