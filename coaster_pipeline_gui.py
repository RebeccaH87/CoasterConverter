#!/usr/bin/env python3
"""
Single-window GUI for the UE5 coaster conversion pipeline.

This app wraps:
- convert_nlelem_to_ue.py
- optional blender_3ds_to_fbx.py execution
"""

from __future__ import annotations

import argparse
import json
import os
import queue
import subprocess
import sys
import threading
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional


APP_NAME = "UE5 Coaster Pipeline"
IS_FROZEN = getattr(sys, "frozen", False)
SCRIPT_DIR = Path(__file__).resolve().parent
RESOURCE_DIR = Path(getattr(sys, "_MEIPASS", SCRIPT_DIR))
APP_STATE_DIR = Path(os.environ.get("LOCALAPPDATA", Path.home() / "AppData" / "Local")) / "UE5_CoasterPipeline"
CONVERTER_SCRIPT = RESOURCE_DIR / "convert_nlelem_to_ue.py"
BLENDER_HELPER_SCRIPT = RESOURCE_DIR / "blender_3ds_to_fbx.py"
BLENDER_ANIMATED_FBX_SCRIPT = RESOURCE_DIR / "blender_build_animated_fbx.py"
STATE_FILE = APP_STATE_DIR / ".gui_state.json"


@dataclass
class PipelineConfig:
    spline_nlelem: str = ""
    tangent_nlelem: str = ""
    mesh_3ds: str = ""
    output_dir: str = ""
    python_exe: str = sys.executable
    blender_exe: str = ""
    cart_model_glb: str = ""
    samples_per_segment: int = 20
    axis_mapping: str = "nl2_to_ue"
    initial_speed: float = 6.0
    rolling_friction: float = 0.004
    drag_coeff: float = 0.0004
    run_fbx_conversion: bool = False
    export_animated_fbx: bool = True
    animated_fbx_fps: int = 30
    animated_fbx_speed_multiplier: float = 1.0
    animated_fbx_scale_multiplier: float = 1.0
    animated_fbx_cart_scale: float = 2.5
    animated_fbx_root_rot_x_deg: float = 90.0
    spike_filter_enabled: bool = True
    spike_angle_threshold_deg: float = 70.0
    spike_deviation_multiplier: float = 0.25
    spike_max_passes: int = 5


def default_config() -> PipelineConfig:
    cfg = PipelineConfig()
    base = Path.home() / "Documents" / "TestCoaster" / "CoasterRawExportData"
    if base.exists():
        cfg.spline_nlelem = str(base / "CoasterSpline.nlelem")
        cfg.tangent_nlelem = str(base / "CoasterTangent.nlelem")
        cfg.mesh_3ds = str(base / "CoasterModel.3ds")
        cfg.output_dir = str(base / "UE5")

    cart_base = base if base.exists() else Path.home() / "Documents" / "TestCoaster"
    cart_candidates = [
        SCRIPT_DIR / "KexLSMfoSketchfab.glb",
        Path.home() / "Documents" / "TestCoaster" / "KexLSMfoSketchfab.glb",
        cart_base / "KexLSMfoSketchfab.glb",
    ]
    for candidate in cart_candidates:
        if candidate.exists():
            cfg.cart_model_glb = str(candidate)
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
        if hasattr(cfg, key):
            setattr(cfg, key, value)
    return cfg


def save_state(cfg: PipelineConfig) -> None:
    STATE_FILE.parent.mkdir(parents=True, exist_ok=True)
    STATE_FILE.write_text(json.dumps(asdict(cfg), indent=2), encoding="utf-8")


def find_blender() -> str:
    candidates = [
        Path("C:/Program Files/Blender Foundation/Blender 5.1/blender.exe"),
        Path("C:/Program Files/Blender Foundation/Blender 4.2/blender.exe"),
        Path("C:/Program Files/Blender Foundation/Blender 4.1/blender.exe"),
        Path("C:/Program Files/Blender Foundation/Blender 4.0/blender.exe"),
        Path("C:/Program Files/Blender Foundation/Blender 3.6/blender.exe"),
    ]
    for candidate in candidates:
        if candidate.exists():
            return str(candidate)
    return ""


def build_converter_command(cfg: PipelineConfig, bundle_path: Path, csv_path: Path, python_exe: Path) -> list[str]:
    base_args = [
        "--spline",
        str(cfg.spline_nlelem),
        "--tangent",
        str(cfg.tangent_nlelem),
        "--mesh",
        str(cfg.mesh_3ds),
        "--output",
        str(bundle_path),
        "--csv",
        str(csv_path),
        "--samples-per-segment",
        str(cfg.samples_per_segment),
        "--axis-mapping",
        cfg.axis_mapping,
        "--initial-speed",
        str(cfg.initial_speed),
        "--rolling-friction",
        str(cfg.rolling_friction),
        "--drag-coeff",
        str(cfg.drag_coeff),
    ]
    if not cfg.spike_filter_enabled:
        base_args.append("--disable-spike-filter")
    base_args.extend(
        [
            "--spike-angle-threshold-deg",
            str(cfg.spike_angle_threshold_deg),
            "--spike-deviation-multiplier",
            str(cfg.spike_deviation_multiplier),
            "--spike-max-passes",
            str(cfg.spike_max_passes),
        ]
    )

    if IS_FROZEN:
        return [str(sys.executable), "--internal-convert", *base_args]

    return [
        str(python_exe),
        str(CONVERTER_SCRIPT),
        *base_args,
    ]


def run_pipeline(cfg: PipelineConfig, log) -> int:
    spline = Path(cfg.spline_nlelem)
    tangent = Path(cfg.tangent_nlelem)
    mesh = Path(cfg.mesh_3ds)
    out_dir = Path(cfg.output_dir)
    python_exe = Path(cfg.python_exe)

    if not IS_FROZEN:
        # Guard against stale settings where users accidentally point this at the
        # packaged GUI executable. That would spawn a second app window on Run.
        exe_name = python_exe.name.lower()
        if "python" not in exe_name:
            fallback = Path(sys.executable)
            log(
                "WARNING: Python executable setting does not look like Python. "
                f"Using current interpreter instead: {fallback}"
            )
            python_exe = fallback

    if not spline.exists():
        raise FileNotFoundError(f"Spline file not found: {spline}")
    if not tangent.exists():
        raise FileNotFoundError(f"Tangent file not found: {tangent}")
    if not mesh.exists():
        raise FileNotFoundError(f"Mesh file not found: {mesh}")
    if not IS_FROZEN and not python_exe.exists():
        raise FileNotFoundError(f"Python executable not found: {python_exe}")

    out_dir.mkdir(parents=True, exist_ok=True)
    bundle_path = out_dir / "coaster_ue5_bundle.json"
    csv_path = out_dir / "coaster_timeline.csv"

    convert_cmd = build_converter_command(cfg, bundle_path, csv_path, python_exe)

    log("Running converter...")
    log(" ".join(convert_cmd))
    convert_proc = subprocess.run(
        convert_cmd,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    if convert_proc.stdout:
        log(convert_proc.stdout.strip())
    if convert_proc.stderr:
        log(convert_proc.stderr.strip())
    if convert_proc.returncode != 0:
        raise RuntimeError(f"Conversion failed with exit code {convert_proc.returncode}")

    if cfg.run_fbx_conversion:
        blender = Path(cfg.blender_exe)
        if not blender.exists():
            raise FileNotFoundError(
                f"Blender path not found but FBX conversion is enabled: {blender}"
            )

        fbx_path = out_dir / "CoasterModel.fbx"
        blender_cmd = [
            str(blender),
            "-b",
            "--python-exit-code",
            "1",
            "-P",
            str(BLENDER_HELPER_SCRIPT),
            "--",
            "--in",
            str(mesh),
            "--out",
            str(fbx_path),
        ]
        log("Running Blender 3DS->FBX conversion...")
        log(" ".join(blender_cmd))
        blender_proc = subprocess.run(
            blender_cmd,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
        )
        if blender_proc.stdout:
            log(blender_proc.stdout.strip())
        if blender_proc.stderr:
            log(blender_proc.stderr.strip())
        if blender_proc.returncode != 0:
            log(
                "WARNING: Optional 3DS->FBX conversion failed. "
                "Continuing with animated FBX export. "
                "If needed, uncheck 'Also convert 3DS to FBX'."
            )
            log(f"WARNING: FBX conversion exit code {blender_proc.returncode}")
        else:
            log(f"Done. Static FBX: {fbx_path}")

    if cfg.export_animated_fbx:
        blender = Path(cfg.blender_exe)
        if not blender.exists():
            raise FileNotFoundError(
                "Blender executable is required for animated FBX export. "
                "Set Blender executable in the GUI."
            )

        track_fbx_path = out_dir / "CoasterTrack.fbx"
        cart_anim_fbx_path = out_dir / "CoasterCartAnimated.fbx"
        animated_cmd = [
            str(blender),
            "-b",
            "--python-exit-code",
            "1",
            "-P",
            str(BLENDER_ANIMATED_FBX_SCRIPT),
            "--",
            "--mesh",
            str(mesh),
            "--bundle",
            str(bundle_path),
            "--track-out",
            str(track_fbx_path),
            "--cart-out",
            str(cart_anim_fbx_path),
            "--fps",
            str(cfg.animated_fbx_fps),
            "--speed-multiplier",
            str(cfg.animated_fbx_speed_multiplier),
            "--scale-multiplier",
            str(cfg.animated_fbx_scale_multiplier),
            "--cart-scale",
            str(cfg.animated_fbx_cart_scale),
            "--root-rot-x-deg",
            str(cfg.animated_fbx_root_rot_x_deg),
        ]
        cart_model_glb = cfg.cart_model_glb.strip()
        if cart_model_glb:
            animated_cmd.extend(["--cart-model", cart_model_glb])
        log("Running Blender animated FBX export...")
        log(" ".join(animated_cmd))
        anim_proc = subprocess.run(
            animated_cmd,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
        )
        if anim_proc.stdout:
            log(anim_proc.stdout.strip())
        if anim_proc.stderr:
            log(anim_proc.stderr.strip())
        if anim_proc.returncode != 0:
            raise RuntimeError(f"Animated FBX export failed with exit code {anim_proc.returncode}")
        if not track_fbx_path.exists() or track_fbx_path.stat().st_size == 0:
            raise RuntimeError("Track FBX export did not produce a valid output file")
        if not cart_anim_fbx_path.exists() or cart_anim_fbx_path.stat().st_size == 0:
            raise RuntimeError("Cart animated FBX export did not produce a valid output file")
        log(f"Done. Track FBX: {track_fbx_path}")
        log(f"Done. Cart Animated FBX: {cart_anim_fbx_path}")

    log(f"Done. Bundle: {bundle_path}")
    log(f"Done. Timeline CSV: {csv_path}")
    return 0


def headless_smoke_test() -> int:
    cfg = load_state()
    print("GUI smoke test passed")
    print(f"Default spline path: {cfg.spline_nlelem}")
    return 0


def launch_gui() -> None:
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk

    cfg = load_state()
    if not cfg.blender_exe:
        cfg.blender_exe = find_blender()

    root = tk.Tk()
    root.title(APP_NAME)
    root.geometry("1180x780")
    root.minsize(860, 560)

    colors = {
        "bg": "#1b1625",
        "panel": "#241d33",
        "panel_alt": "#302646",
        "border": "#b89a4a",
        "border_soft": "#8d7740",
        "fg": "#f3edf9",
        "muted": "#c5b8d9",
        "accent": "#d8b75a",
        "accent_hover": "#e4c979",
        "success": "#d4c070",
        "entry": "#211a2f",
        "entry_focus": "#2b2140",
        "danger": "#ef8ea7",
    }

    ui_font = "Segoe UI Variable Text"
    ui_font_bold = "Segoe UI Variable Display"
    mono_font = "Cascadia Code"

    root.configure(bg=colors["bg"])
    style = ttk.Style()
    style.theme_use("clam")
    style.configure("App.TFrame", background=colors["bg"])
    style.configure("Card.TFrame", background=colors["panel"], relief="flat")
    style.configure("Card.TLabelframe", background=colors["panel"], borderwidth=1, relief="solid", bordercolor=colors["border_soft"], lightcolor=colors["border_soft"], darkcolor=colors["border_soft"])
    style.configure("Card.TLabelframe.Label", background=colors["panel"], foreground=colors["accent"], font=(ui_font_bold, 11))
    style.configure("HeaderTitle.TLabel", background=colors["bg"], foreground=colors["fg"], font=(ui_font_bold, 22))
    style.configure("HeaderSub.TLabel", background=colors["bg"], foreground=colors["muted"], font=(ui_font, 10))
    style.configure("Body.TLabel", background=colors["panel"], foreground=colors["fg"], font=(ui_font, 10))
    style.configure("Muted.TLabel", background=colors["bg"], foreground=colors["muted"], font=(ui_font, 9))
    style.configure("Card.TCheckbutton", background=colors["panel"], foreground=colors["fg"], font=(ui_font, 10))
    style.map("Card.TCheckbutton", background=[("active", colors["panel"])], foreground=[("active", colors["fg"])])
    style.configure("Card.TEntry", fieldbackground=colors["entry"], background=colors["entry"], foreground=colors["fg"], bordercolor=colors["border"], lightcolor=colors["border"], darkcolor=colors["border"], insertcolor=colors["fg"], padding=7)
    style.map("Card.TEntry", fieldbackground=[("focus", colors["entry_focus"])])
    style.configure("Card.TCombobox", fieldbackground=colors["entry"], background=colors["entry"], foreground=colors["fg"], bordercolor=colors["border"], lightcolor=colors["border"], darkcolor=colors["border"], arrowsize=16, padding=6)
    style.map("Card.TCombobox", fieldbackground=[("readonly", colors["entry"]), ("focus", colors["entry_focus"])], selectbackground=[("readonly", colors["entry_focus"])] , selectforeground=[("readonly", colors["fg"])])
    style.configure("Card.TSpinbox", fieldbackground=colors["entry"], background=colors["entry"], foreground=colors["fg"], bordercolor=colors["border"], lightcolor=colors["border"], darkcolor=colors["border"], arrowsize=14, padding=4)
    style.configure("Ghost.TButton", background=colors["panel_alt"], foreground=colors["fg"], borderwidth=1, focusthickness=0, bordercolor=colors["border_soft"], lightcolor=colors["border_soft"], darkcolor=colors["border_soft"], padding=(14, 8), font=(ui_font_bold, 10))
    style.map("Ghost.TButton", background=[("active", colors["border"])], foreground=[("active", colors["fg"])])

    shell = tk.Frame(root, bg=colors["bg"])
    shell.pack(fill=tk.BOTH, expand=True)

    canvas = tk.Canvas(
        shell,
        bg=colors["bg"],
        highlightthickness=0,
        bd=0,
        relief=tk.FLAT,
    )
    scrollbar = tk.Scrollbar(
        shell,
        orient=tk.VERTICAL,
        command=canvas.yview,
        bg=colors["panel_alt"],
        activebackground=colors["accent_hover"],
        troughcolor=colors["bg"],
        highlightthickness=0,
        bd=0,
        width=12,
        relief=tk.FLAT,
    )
    canvas.configure(yscrollcommand=scrollbar.set)
    scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
    canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    main = ttk.Frame(canvas, padding=18, style="App.TFrame")
    canvas_window = canvas.create_window((0, 0), window=main, anchor="nw")

    def sync_scroll_region(_event=None):
        canvas.configure(scrollregion=canvas.bbox("all"))

    def resize_embedded_frame(event):
        canvas.itemconfigure(canvas_window, width=event.width)

    def on_mousewheel(event):
        delta = event.delta
        if delta:
            canvas.yview_scroll(int(-delta / 120), "units")

    main.bind("<Configure>", sync_scroll_region)
    canvas.bind("<Configure>", resize_embedded_frame)
    canvas.bind_all("<MouseWheel>", on_mousewheel)

    header = ttk.Frame(main, style="App.TFrame")
    header.pack(fill=tk.X, pady=(0, 14))
    ttk.Label(header, text="UE5 Coaster Pipeline", style="HeaderTitle.TLabel").pack(anchor="w")
    ttk.Label(
        header,
        text="Dark-mode pipeline launcher for conversion, cleanup, and animated FBX export.",
        style="HeaderSub.TLabel",
    ).pack(anchor="w", pady=(4, 0))

    path_frame = ttk.LabelFrame(main, text="Input / Output", style="Card.TLabelframe", padding=14)
    path_frame.pack(fill=tk.X, pady=(0, 10))

    vars_dict = {
        "spline_nlelem": tk.StringVar(value=cfg.spline_nlelem),
        "tangent_nlelem": tk.StringVar(value=cfg.tangent_nlelem),
        "mesh_3ds": tk.StringVar(value=cfg.mesh_3ds),
        "output_dir": tk.StringVar(value=cfg.output_dir),
        "python_exe": tk.StringVar(value=cfg.python_exe),
        "blender_exe": tk.StringVar(value=cfg.blender_exe),
        "cart_model_glb": tk.StringVar(value=cfg.cart_model_glb),
        "samples_per_segment": tk.IntVar(value=cfg.samples_per_segment),
        "axis_mapping": tk.StringVar(value=cfg.axis_mapping),
        "initial_speed": tk.DoubleVar(value=cfg.initial_speed),
        "rolling_friction": tk.DoubleVar(value=cfg.rolling_friction),
        "drag_coeff": tk.DoubleVar(value=cfg.drag_coeff),
        "run_fbx_conversion": tk.BooleanVar(value=cfg.run_fbx_conversion),
        "export_animated_fbx": tk.BooleanVar(value=cfg.export_animated_fbx),
        "animated_fbx_fps": tk.IntVar(value=cfg.animated_fbx_fps),
        "animated_fbx_speed_multiplier": tk.DoubleVar(value=cfg.animated_fbx_speed_multiplier),
        "animated_fbx_scale_multiplier": tk.DoubleVar(value=cfg.animated_fbx_scale_multiplier),
        "animated_fbx_cart_scale": tk.DoubleVar(value=cfg.animated_fbx_cart_scale),
        "animated_fbx_root_rot_x_deg": tk.DoubleVar(value=cfg.animated_fbx_root_rot_x_deg),
        "spike_filter_enabled": tk.BooleanVar(value=cfg.spike_filter_enabled),
        "spike_angle_threshold_deg": tk.DoubleVar(value=cfg.spike_angle_threshold_deg),
        "spike_deviation_multiplier": tk.DoubleVar(value=cfg.spike_deviation_multiplier),
        "spike_max_passes": tk.IntVar(value=cfg.spike_max_passes),
    }

    row = 0

    def add_row(label, key, file_mode="file", file_types=None):
        nonlocal row
        ttk.Label(path_frame, text=label, width=24, style="Body.TLabel").grid(row=row, column=0, sticky="w", padx=6, pady=6)
        ttk.Entry(path_frame, textvariable=vars_dict[key], width=88, style="Card.TEntry").grid(
            row=row, column=1, sticky="ew", padx=6, pady=4
        )

        def browse():
            if file_mode == "dir":
                picked = filedialog.askdirectory()
            else:
                picked = filedialog.askopenfilename(filetypes=file_types)
            if picked:
                vars_dict[key].set(picked)

        ttk.Button(path_frame, text="Browse", command=browse, style="Ghost.TButton").grid(row=row, column=2, sticky="e", padx=6, pady=4)
        row += 1

    add_row("Spline .nlelem", "spline_nlelem", "file", [("NLElem", "*.nlelem"), ("All", "*.*")])
    add_row("Tangent .nlelem", "tangent_nlelem", "file", [("NLElem", "*.nlelem"), ("All", "*.*")])
    add_row("Mesh .3ds", "mesh_3ds", "file", [("3DS", "*.3ds"), ("All", "*.*")])
    add_row("Output folder", "output_dir", "dir")
    if not IS_FROZEN:
        add_row("Python executable", "python_exe", "file", [("Python", "python*.exe"), ("All", "*.*")])
    add_row("Blender executable", "blender_exe", "file", [("Blender", "blender.exe"), ("All", "*.*")])
    add_row("Cart model .glb", "cart_model_glb", "file", [("GLB", "*.glb"), ("glTF", "*.gltf"), ("All", "*.*")])

    path_frame.columnconfigure(1, weight=1)

    settings_frame = ttk.LabelFrame(main, text="Settings", style="Card.TLabelframe", padding=14)
    settings_frame.pack(fill=tk.X, pady=(0, 10))

    ttk.Label(settings_frame, text="Samples per segment", style="Body.TLabel").grid(row=0, column=0, sticky="w", padx=6, pady=6)
    ttk.Spinbox(settings_frame, from_=4, to=200, textvariable=vars_dict["samples_per_segment"], width=12, style="Card.TSpinbox").grid(
        row=0, column=1, sticky="w", padx=6, pady=6
    )

    ttk.Label(settings_frame, text="Axis mapping", style="Body.TLabel").grid(row=0, column=2, sticky="w", padx=6, pady=6)
    ttk.Combobox(
        settings_frame,
        textvariable=vars_dict["axis_mapping"],
        values=["nl2_to_ue", "nl2_to_ue_flip_y", "identity"],
        width=20,
        state="readonly",
        style="Card.TCombobox",
    ).grid(row=0, column=3, sticky="w", padx=6, pady=6)

    ttk.Label(settings_frame, text="Initial speed (m/s)", style="Body.TLabel").grid(row=1, column=0, sticky="w", padx=6, pady=6)
    ttk.Entry(settings_frame, textvariable=vars_dict["initial_speed"], width=12, style="Card.TEntry").grid(
        row=1, column=1, sticky="w", padx=6, pady=6
    )

    ttk.Label(settings_frame, text="Rolling friction", style="Body.TLabel").grid(row=1, column=2, sticky="w", padx=6, pady=6)
    ttk.Entry(settings_frame, textvariable=vars_dict["rolling_friction"], width=12, style="Card.TEntry").grid(
        row=1, column=3, sticky="w", padx=6, pady=6
    )

    ttk.Checkbutton(
        settings_frame,
        text="Also convert 3DS to FBX (requires Blender)",
        variable=vars_dict["run_fbx_conversion"],
        style="Card.TCheckbutton",
    ).grid(row=2, column=0, columnspan=2, sticky="w", padx=6, pady=6)

    ttk.Label(settings_frame, text="Drag coefficient", style="Body.TLabel").grid(row=2, column=2, sticky="w", padx=6, pady=6)
    ttk.Entry(settings_frame, textvariable=vars_dict["drag_coeff"], width=12, style="Card.TEntry").grid(
        row=2, column=3, sticky="w", padx=6, pady=6
    )

    ttk.Checkbutton(
        settings_frame,
        text="Export animated FBX (track + cart animation)",
        variable=vars_dict["export_animated_fbx"],
        style="Card.TCheckbutton",
    ).grid(row=3, column=0, columnspan=2, sticky="w", padx=6, pady=6)

    ttk.Label(settings_frame, text="Animated FBX FPS", style="Body.TLabel").grid(row=3, column=2, sticky="w", padx=6, pady=6)
    ttk.Spinbox(settings_frame, from_=12, to=120, textvariable=vars_dict["animated_fbx_fps"], width=12, style="Card.TSpinbox").grid(
        row=3, column=3, sticky="w", padx=6, pady=6
    )

    ttk.Label(settings_frame, text="Animation speed multiplier", style="Body.TLabel").grid(row=4, column=0, sticky="w", padx=6, pady=6)
    ttk.Entry(settings_frame, textvariable=vars_dict["animated_fbx_speed_multiplier"], width=12, style="Card.TEntry").grid(
        row=4, column=1, sticky="w", padx=6, pady=6
    )

    ttk.Label(settings_frame, text="Track/motion scale multiplier", style="Body.TLabel").grid(row=4, column=2, sticky="w", padx=6, pady=6)
    ttk.Entry(settings_frame, textvariable=vars_dict["animated_fbx_scale_multiplier"], width=12, style="Card.TEntry").grid(
        row=4, column=3, sticky="w", padx=6, pady=6
    )

    ttk.Label(settings_frame, text="Cart scale relative to track", style="Body.TLabel").grid(row=5, column=0, sticky="w", padx=6, pady=6)
    ttk.Entry(settings_frame, textvariable=vars_dict["animated_fbx_cart_scale"], width=12, style="Card.TEntry").grid(
        row=5, column=1, sticky="w", padx=6, pady=6
    )

    ttk.Label(settings_frame, text="Root X rotation (deg)", style="Body.TLabel").grid(row=5, column=2, sticky="w", padx=6, pady=6)
    ttk.Entry(settings_frame, textvariable=vars_dict["animated_fbx_root_rot_x_deg"], width=12, style="Card.TEntry").grid(
        row=5, column=3, sticky="w", padx=6, pady=6
    )

    ttk.Checkbutton(
        settings_frame,
        text="Smooth extreme spline spikes/outliers",
        variable=vars_dict["spike_filter_enabled"],
        style="Card.TCheckbutton",
    ).grid(row=6, column=0, columnspan=2, sticky="w", padx=6, pady=6)

    ttk.Label(settings_frame, text="Spike angle threshold (deg)", style="Body.TLabel").grid(row=6, column=2, sticky="w", padx=6, pady=6)
    ttk.Entry(settings_frame, textvariable=vars_dict["spike_angle_threshold_deg"], width=12, style="Card.TEntry").grid(
        row=6, column=3, sticky="w", padx=6, pady=6
    )

    ttk.Label(settings_frame, text="Spike deviation multiplier", style="Body.TLabel").grid(row=7, column=0, sticky="w", padx=6, pady=6)
    ttk.Entry(settings_frame, textvariable=vars_dict["spike_deviation_multiplier"], width=12, style="Card.TEntry").grid(
        row=7, column=1, sticky="w", padx=6, pady=6
    )

    ttk.Label(settings_frame, text="Spike max passes", style="Body.TLabel").grid(row=7, column=2, sticky="w", padx=6, pady=6)
    ttk.Spinbox(settings_frame, from_=1, to=10, textvariable=vars_dict["spike_max_passes"], width=12, style="Card.TSpinbox").grid(
        row=7, column=3, sticky="w", padx=6, pady=6
    )

    queue_out = queue.Queue()
    running = {"busy": False}

    def append_log(msg: str):
        log_text.insert(tk.END, msg + "\n")
        log_text.see(tk.END)

    def collect_cfg() -> PipelineConfig:
        python_exe_value = vars_dict["python_exe"].get().strip() if "python_exe" in vars_dict else cfg.python_exe
        return PipelineConfig(
            spline_nlelem=vars_dict["spline_nlelem"].get().strip(),
            tangent_nlelem=vars_dict["tangent_nlelem"].get().strip(),
            mesh_3ds=vars_dict["mesh_3ds"].get().strip(),
            output_dir=vars_dict["output_dir"].get().strip(),
            python_exe=python_exe_value,
            blender_exe=vars_dict["blender_exe"].get().strip(),
            cart_model_glb=vars_dict["cart_model_glb"].get().strip(),
            samples_per_segment=int(vars_dict["samples_per_segment"].get()),
            axis_mapping=vars_dict["axis_mapping"].get().strip(),
            initial_speed=float(vars_dict["initial_speed"].get()),
            rolling_friction=float(vars_dict["rolling_friction"].get()),
            drag_coeff=float(vars_dict["drag_coeff"].get()),
            run_fbx_conversion=bool(vars_dict["run_fbx_conversion"].get()),
            export_animated_fbx=bool(vars_dict["export_animated_fbx"].get()),
            animated_fbx_fps=int(vars_dict["animated_fbx_fps"].get()),
            animated_fbx_speed_multiplier=float(vars_dict["animated_fbx_speed_multiplier"].get()),
            animated_fbx_scale_multiplier=float(vars_dict["animated_fbx_scale_multiplier"].get()),
            animated_fbx_cart_scale=float(vars_dict["animated_fbx_cart_scale"].get()),
            animated_fbx_root_rot_x_deg=float(vars_dict["animated_fbx_root_rot_x_deg"].get()),
            spike_filter_enabled=bool(vars_dict["spike_filter_enabled"].get()),
            spike_angle_threshold_deg=float(vars_dict["spike_angle_threshold_deg"].get()),
            spike_deviation_multiplier=float(vars_dict["spike_deviation_multiplier"].get()),
            spike_max_passes=int(vars_dict["spike_max_passes"].get()),
        )

    def background_run(run_cfg: PipelineConfig):
        def _log(msg: str):
            queue_out.put(("log", msg))

        try:
            save_state(run_cfg)
            run_pipeline(run_cfg, _log)
            queue_out.put(("done", "Success"))
        except Exception as ex:
            queue_out.put(("error", str(ex)))

    def on_run_clicked():
        if running["busy"]:
            return
        cfg_local = collect_cfg()
        running["busy"] = True
        run_btn.config(state=tk.DISABLED)
        append_log("Starting conversion...")
        thread = threading.Thread(target=background_run, args=(cfg_local,), daemon=True)
        thread.start()

    def on_save_clicked():
        cfg_local = collect_cfg()
        save_state(cfg_local)
        append_log("Settings saved.")

    def on_open_output_clicked():
        out = Path(vars_dict["output_dir"].get().strip())
        if not out.exists():
            messagebox.showwarning("Output folder", "Output folder does not exist yet.")
            return
        subprocess.Popen(["explorer", str(out)])

    button_row = tk.Frame(main, bg=colors["bg"], highlightthickness=0, bd=0)
    button_row.pack(fill=tk.X, pady=(0, 10))

    run_btn = tk.Button(
        button_row,
        text="Run",
        command=on_run_clicked,
        bg=colors["accent"],
        fg="#241b08",
        activebackground=colors["accent_hover"],
        activeforeground="#241b08",
        font=(ui_font_bold, 11),
        padx=20,
        pady=10,
        relief=tk.FLAT,
        bd=0,
        highlightthickness=0,
        cursor="hand2",
    )
    run_btn.pack(side=tk.LEFT, padx=(0, 10))
    ttk.Button(button_row, text="Save Settings", command=on_save_clicked, style="Ghost.TButton").pack(side=tk.LEFT, padx=(0, 6))
    ttk.Button(button_row, text="Open Output Folder", command=on_open_output_clicked, style="Ghost.TButton").pack(side=tk.LEFT, padx=(0, 6))
    ttk.Label(button_row, text="Shortcut: Ctrl+Enter", style="Muted.TLabel").pack(side=tk.RIGHT, pady=(8, 0))

    log_frame = ttk.LabelFrame(main, text="Run Log", style="Card.TLabelframe", padding=10)
    log_frame.pack(fill=tk.BOTH, expand=True)

    log_text = tk.Text(
        log_frame,
        wrap="word",
        height=18,
        bg=colors["entry"],
        fg=colors["fg"],
        insertbackground=colors["fg"],
        selectbackground=colors["accent"],
        selectforeground="#241b08",
        relief=tk.FLAT,
        borderwidth=0,
        highlightthickness=1,
        highlightbackground=colors["border"],
        font=(mono_font, 10),
        padx=10,
        pady=10,
    )
    log_text.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

    def poll_queue():
        while True:
            try:
                kind, payload = queue_out.get_nowait()
            except queue.Empty:
                break

            if kind == "log":
                append_log(payload)
            elif kind == "done":
                running["busy"] = False
                run_btn.config(state=tk.NORMAL)
                append_log("Finished successfully.")
                messagebox.showinfo(APP_NAME, "Conversion completed successfully.")
            elif kind == "error":
                running["busy"] = False
                run_btn.config(state=tk.NORMAL)
                append_log(f"ERROR: {payload}")
                messagebox.showerror(APP_NAME, payload)

        root.after(120, poll_queue)

    root.bind("<Control-Return>", lambda _evt: on_run_clicked())
    root.after(120, poll_queue)
    root.mainloop()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="UE5 coaster pipeline GUI")
    parser.add_argument(
        "--internal-convert",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--smoke-test",
        action="store_true",
        help="Run a quick startup validation without launching a window",
    )
    parser.add_argument(
        "--headless-run",
        action="store_true",
        help="Run conversion without GUI using saved/default paths",
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
        cfg = load_state()
        if not cfg.blender_exe:
            cfg.blender_exe = find_blender()
        run_pipeline(cfg, print)
        return 0

    launch_gui()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())