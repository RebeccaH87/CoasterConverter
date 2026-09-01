# -*- mode: python ; coding: utf-8 -*-

import sys

a = Analysis(
    ['coaster_pipeline_gui.py'],
    pathex=[],
    binaries=[],
    datas=[],
    hiddenimports=[
        'convert_nlelem_to_ue', 'export_car_animation', 'export_track_mesh',
        'fbx_writer', 'read_glb',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

if sys.platform == 'darwin':
    # Onedir, not onefile: a notarized app must have its hardened runtime
    # validate every loaded library's code signature, but onefile only
    # unpacks binaries (e.g. libpython) to a temp dir at launch, where they
    # keep their *original* signer (e.g. Anaconda) instead of ours. That
    # Team ID mismatch makes the process refuse to load them. Onedir ships
    # the real files inside the .app, so `codesign --deep` re-signs them
    # all under our own identity and the Team IDs match at launch.
    exe = EXE(
        pyz,
        a.scripts,
        [],
        exclude_binaries=True,
        name='UE5_CoasterPipeline',
        debug=False,
        bootloader_ignore_signals=False,
        strip=False,
        upx=True,
        console=False,
        disable_windowed_traceback=False,
        argv_emulation=False,
        target_arch=None,
        codesign_identity=None,
        entitlements_file=None,
    )
    coll = COLLECT(
        exe,
        a.binaries,
        a.datas,
        strip=False,
        upx=True,
        upx_exclude=[],
        name='UE5_CoasterPipeline',
    )
    app = BUNDLE(
        coll,
        name='UE5_CoasterPipeline.app',
        icon=None,
        bundle_identifier='edu.lsu.ue5coasterpipeline',
        info_plist={
            'CFBundleName': 'UE5 Coaster Pipeline',
            'CFBundleShortVersionString': '1.0.0',
            'NSHighResolutionCapable': True,
        },
    )
else:
    exe = EXE(
        pyz,
        a.scripts,
        a.binaries,
        a.datas,
        [],
        name='UE5_CoasterPipeline',
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
    )
