"""
Runtime hook: add the PyInstaller Resources/Contents folder to sys.path
so that shinestacker (copied as real files via datas) can be imported.
"""
import sys, os

# In a macOS .app bundle PyInstaller sets sys._MEIPASS to the Resources dir.
# Adding it to the front of sys.path makes 'import shinestacker' find the
# copied package directory instead of relying on the frozen PYZ archive.
if hasattr(sys, '_MEIPASS'):
    meipass = sys._MEIPASS
    if meipass not in sys.path:
        sys.path.insert(0, meipass)
