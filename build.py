#!/usr/bin/env python3
"""
build.py -- Build bouncing_glyphs into a standalone .exe and package as a .zip.

Usage:
    python build.py

Produces:
    dist/bouncing_glyphs/bouncing_glyphs.exe   (the runnable program)
    bouncing_glyphs.zip                        (everything needed to distribute)

Requires:
    pip install pyinstaller
"""

import os
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent.resolve()
MAIN_SCRIPT = SCRIPT_DIR / "bouncing_glyphs.py"
DIST_DIR = SCRIPT_DIR / "dist"
BUILD_DIR = SCRIPT_DIR / "build"
SPEC_FILE = SCRIPT_DIR / "bouncing_glyphs.spec"
OUTPUT_ZIP = SCRIPT_DIR / "bouncing_glyphs.zip"
EXE_DIR_NAME = "bouncing_glyphs"


def run(cmd, **kwargs):
    print(f"  > {' '.join(cmd)}")
    result = subprocess.run(cmd, **kwargs)
    if result.returncode != 0:
        sys.exit(f"Command failed with exit code {result.returncode}")
    return result


def main():
    os.chdir(SCRIPT_DIR)

    # ── Clean previous build artifacts ──
    for path in [BUILD_DIR, DIST_DIR, SPEC_FILE]:
        if path.is_dir():
            print(f"Cleaning {path}")
            shutil.rmtree(path)
        elif path.is_file():
            path.unlink()
    if OUTPUT_ZIP.is_file():
        OUTPUT_ZIP.unlink()

    # ── Run PyInstaller ──
    print("\n=== Building with PyInstaller ===\n")
    run([
        sys.executable, "-m", "PyInstaller",
        "--noconfirm",
        "--name", EXE_DIR_NAME,
        # One-folder mode (faster startup than one-file, easier to debug)
        "--onedir",
        # No console window (it's a pygame GUI app)
        "--windowed",
        # Collect fonttools data files (they include some required data)
        "--collect-data", "fonttools",
        # Hidden imports that PyInstaller sometimes misses
        "--hidden-import", "pygame",
        "--hidden-import", "pygame.freetype",
        "--hidden-import", "fontTools.ttLib",
        "--hidden-import", "fontTools.pens.recordingPen",
        str(MAIN_SCRIPT),
    ])

    exe_dir = DIST_DIR / EXE_DIR_NAME
    if not exe_dir.is_dir():
        sys.exit(f"Expected output directory not found: {exe_dir}")

    # ── Copy README into the dist folder ──
    readme = SCRIPT_DIR / "README.md"
    if readme.is_file():
        shutil.copy2(readme, exe_dir / "README.md")
        print(f"Copied README.md into {exe_dir}")

    # ── Package into .zip ──
    print(f"\n=== Packaging into {OUTPUT_ZIP.name} ===\n")
    with zipfile.ZipFile(OUTPUT_ZIP, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as zf:
        for root, dirs, files in os.walk(exe_dir):
            for file in files:
                file_path = Path(root) / file
                arc_name = Path(EXE_DIR_NAME) / file_path.relative_to(exe_dir)
                zf.write(file_path, arc_name)

    zip_size_mb = OUTPUT_ZIP.stat().st_size / (1024 * 1024)
    print(f"Created: {OUTPUT_ZIP}  ({zip_size_mb:.1f} MB)")

    # ── Summary ──
    exe_path = exe_dir / f"{EXE_DIR_NAME}.exe"
    print(f"""
=== Done ===

  Executable:  {exe_path}
  Zip archive: {OUTPUT_ZIP}

To run directly:
  {exe_path}
  {exe_path} --gravity 500 --friction 0.3 --unicode --colorful

To distribute:
  Send {OUTPUT_ZIP.name} -- unzip and run {EXE_DIR_NAME}.exe
""")


if __name__ == "__main__":
    main()
