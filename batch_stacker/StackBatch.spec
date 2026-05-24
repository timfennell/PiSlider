# -*- mode: python ; coding: utf-8 -*-

# StackBatch.spec
#
# shinestacker is NOT imported in-process.  The app spawns a system-Python
# subprocess for each stack job, which loads shinestacker from
# /Applications/shinestacker.app/Contents/Resources at runtime.
# This means we bundle only tkinter + stdlib — nothing from shinestacker.

a = Analysis(
    ['gui_stacker.py'],
    pathex=[],
    binaries=[],
    datas=[],
    hiddenimports=[],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        # Keep the bundle tiny — shinestacker and all scientific stack run
        # in a subprocess; none of these are needed here.
        'shinestacker',
        'rawpy', 'cv2', 'scipy', 'numpy', 'PIL', 'Pillow',
        'matplotlib', 'imagecodecs', 'PySide6', 'tqdm',
        'IPython', 'jedi', 'parso', 'prompt_toolkit',
        'pygments', 'traitlets', 'ipywidgets',
        'psutil', 'setuptools', 'pkg_resources',
    ],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='StackBatch',
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
    name='StackBatch',
)

app = BUNDLE(
    coll,
    name='StackBatch.app',
    icon=None,
    bundle_identifier='com.pislider.stackbatch',
)
