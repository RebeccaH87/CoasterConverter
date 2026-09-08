# -*- mode: python ; coding: utf-8 -*-
"""PyInstaller spec for the nl2elem -> UE5 converter, Windows and macOS.

One spec for both platforms because the two builds differ only in packaging
shape: Windows ships a single self-contained .exe, macOS ships a .app bundle
(a one-file binary cannot carry an Info.plist, so it gets no Dock icon and no
Retina rendering).

Build with:
    pyinstaller --noconfirm UE5_CoasterPipeline.spec

Environment overrides:
    MACOS_TARGET_ARCH   arm64 | x86_64 | universal2 (default: host arch)
"""

import os
import re
import sys
from pathlib import Path

SPEC_DIR = Path(SPECPATH).resolve()
PACKAGING = SPEC_DIR / "packaging"
IS_MAC = sys.platform == "darwin"
IS_WIN = sys.platform == "win32"

APP_NAME = "UE5_CoasterPipeline"
BUNDLE_ID = "edu.lsu.digitalart.ue5coasterpipeline"

# The version lives in the app source so the GUI, the installer and the bundle
# can never disagree about it.
_gui_src = (SPEC_DIR / "coaster_pipeline_gui.py").read_text(encoding="utf-8")
_match = re.search(r'^APP_VERSION\s*=\s*"([^"]+)"', _gui_src, re.MULTILINE)
if not _match:
    raise SystemExit("APP_VERSION not found in coaster_pipeline_gui.py")
APP_VERSION = _match.group(1)

# Converter modules the GUI reaches only through lazy or re-exec imports, so
# static analysis does not see all of them.
HIDDEN = [
    "convert_nlelem_to_ue",
    "export_car_animation",
    "export_car_glb",
    "export_track_mesh",
    "fbx_writer",
    "read_glb",
]

# The pipeline is standard library only; excluding the heavyweight scientific
# and dev packages keeps the download from tripling if any of them is installed
# in the build environment.
EXCLUDES = [
    "numpy", "scipy", "matplotlib", "pandas", "PIL", "IPython",
    "pytest", "setuptools", "pip", "unittest",
]


def _windows_version_file() -> str:
    """Write a VERSIONINFO resource so the .exe has real file properties."""
    parts = [int(p) for p in APP_VERSION.split(".")]
    while len(parts) < 4:
        parts.append(0)
    quad = ", ".join(str(p) for p in parts[:4])
    text = f"""VSVersionInfo(
  ffi=FixedFileInfo(
    filevers=({quad}), prodvers=({quad}),
    mask=0x3f, flags=0x0, OS=0x40004, fileType=0x1, subtype=0x0,
  ),
  kids=[
    StringFileInfo([StringTable('040904B0', [
      StringStruct('CompanyName', 'LSU Digital Art'),
      StringStruct('FileDescription', 'UE5 Coaster Pipeline - nl2elem to UE5 converter'),
      StringStruct('FileVersion', '{APP_VERSION}'),
      StringStruct('InternalName', '{APP_NAME}'),
      StringStruct('OriginalFilename', '{APP_NAME}.exe'),
      StringStruct('ProductName', 'UE5 Coaster Pipeline'),
      StringStruct('ProductVersion', '{APP_VERSION}'),
    ])]),
    VarFileInfo([VarStruct('Translation', [1033, 1200])]),
  ],
)
"""
    out = Path(workpath) / "version_info.txt"
    out.write_text(text, encoding="utf-8")
    return str(out)


icon = None
if IS_WIN and (PACKAGING / "app.ico").exists():
    icon = str(PACKAGING / "app.ico")
elif IS_MAC and (PACKAGING / "app.icns").exists():
    icon = str(PACKAGING / "app.icns")

target_arch = os.environ.get("MACOS_TARGET_ARCH") or None if IS_MAC else None

a = Analysis(
    ["coaster_pipeline_gui.py"],
    pathex=[str(SPEC_DIR)],
    binaries=[],
    datas=[],
    hiddenimports=HIDDEN,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=EXCLUDES,
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

if IS_MAC:
    # onedir + BUNDLE: the .app is the shippable unit on macOS.
    exe = EXE(
        pyz,
        a.scripts,
        [],
        exclude_binaries=True,
        name=APP_NAME,
        debug=False,
        bootloader_ignore_signals=False,
        strip=False,
        upx=False,
        console=False,
        disable_windowed_traceback=False,
        argv_emulation=False,
        target_arch=target_arch,
        codesign_identity=os.environ.get("MACOS_CODESIGN_IDENTITY") or None,
        entitlements_file=None,
        icon=icon,
    )
    coll = COLLECT(
        exe,
        a.binaries,
        a.datas,
        strip=False,
        upx=False,
        upx_exclude=[],
        name=APP_NAME,
    )
    app = BUNDLE(
        coll,
        name=f"{APP_NAME}.app",
        icon=icon,
        bundle_identifier=BUNDLE_ID,
        version=APP_VERSION,
        info_plist={
            "CFBundleName": "UE5 Coaster Pipeline",
            "CFBundleDisplayName": "UE5 Coaster Pipeline",
            "CFBundleShortVersionString": APP_VERSION,
            "CFBundleVersion": APP_VERSION,
            # Without this the Tk window is bitmap-stretched on Retina displays.
            "NSHighResolutionCapable": True,
            "LSMinimumSystemVersion": "11.0",
            "LSApplicationCategoryType": "public.app-category.developer-tools",
            "NSRequiresAquaSystemAppearance": False,
            # No CFBundleDocumentTypes on purpose. Receiving a double-clicked
            # file in a .app needs PyInstaller's argv_emulation, which hooks
            # AppleEvents and has a history of stalling launch -- not something
            # to ship untested. The Windows installer offers the association
            # because it is a plain argv hand-off. macOS users open files from
            # the picker inside the app.
        },
    )
else:
    # Windows: single self-contained executable, as the tool has always shipped.
    exe = EXE(
        pyz,
        a.scripts,
        a.binaries,
        a.datas,
        [],
        name=APP_NAME,
        debug=False,
        bootloader_ignore_signals=False,
        strip=False,
        upx=True,
        upx_exclude=[],
        runtime_tmpdir=None,
        console=False,
        disable_windowed_traceback=False,
        argv_emulation=False,
        target_arch=None,
        codesign_identity=None,
        entitlements_file=None,
        icon=icon,
        version=_windows_version_file() if IS_WIN else None,
    )
