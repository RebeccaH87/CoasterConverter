#!/usr/bin/env bash
# Build the macOS .app bundle and a distributable .dmg.
#
# Two things about this build are not obvious:
#
#   1. It needs a Python with a working tkinter. Homebrew's python has no
#      _tkinter unless python-tk is installed too; the python.org installer
#      and conda both ship Tk. The script checks rather than failing deep
#      inside PyInstaller.
#   2. It stages into a temp dir instead of building in place. On a Mac where
#      this repo lives under an iCloud-synced folder, the file provider keeps
#      re-adding a com.apple.FinderInfo xattr to the bundle, and `codesign
#      --verify --strict` rejects that as "detritus". Building outside the
#      synced tree keeps the shipped artifacts signature-clean.
#
# Usage:
#   ./build_macos.sh                 # build for the host architecture
#   MACOS_TARGET_ARCH=universal2 ./build_macos.sh
#   MACOS_CODESIGN_IDENTITY="Developer ID Application: ..." ./build_macos.sh
#
# The app is signed ad-hoc unless MACOS_CODESIGN_IDENTITY is set. An ad-hoc
# app runs fine on the machine that built it; on any other Mac Gatekeeper
# needs a right-click -> Open once, or:
#   xattr -dr com.apple.quarantine /Applications/UE5_CoasterPipeline.app

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="${PYTHON:-python3}"
APP_NAME="UE5_CoasterPipeline"

VERSION="$(sed -n 's/^APP_VERSION = "\([^"]*\)".*/\1/p' "$REPO/coaster_pipeline_gui.py" | head -1)"
[ -n "$VERSION" ] || { echo "could not read APP_VERSION from coaster_pipeline_gui.py" >&2; exit 1; }

ARCH="${MACOS_TARGET_ARCH:-$(uname -m)}"
DMG="$REPO/dist/${APP_NAME}-${VERSION}-macOS-${ARCH}.dmg"

if ! "$PYTHON" -c "import tkinter" 2>/dev/null; then
    echo "error: $PYTHON has no tkinter, so the GUI cannot be frozen." >&2
    echo "       Use the python.org build, or conda, or 'brew install python-tk'." >&2
    exit 1
fi

STAGE="$(mktemp -d)"
trap 'rm -rf "$STAGE"' EXIT

echo "==> build env"
"$PYTHON" -m venv "$STAGE/venv"
"$STAGE/venv/bin/pip" install --quiet --upgrade pip pyinstaller

echo "==> freezing $APP_NAME $VERSION ($ARCH)"
"$STAGE/venv/bin/pyinstaller" --noconfirm \
    --workpath "$STAGE/work" --distpath "$STAGE/dist" \
    "$REPO/UE5_CoasterPipeline.spec"

echo "==> verifying the bundle"
"$STAGE/dist/$APP_NAME.app/Contents/MacOS/$APP_NAME" --smoke-test
codesign --verify --deep --strict "$STAGE/dist/$APP_NAME.app"

echo "==> packaging dmg"
mkdir -p "$STAGE/dmgroot" "$REPO/dist"
ditto "$STAGE/dist/$APP_NAME.app" "$STAGE/dmgroot/$APP_NAME.app"
ln -s /Applications "$STAGE/dmgroot/Applications"
rm -f "$DMG"
hdiutil create -volname "UE5 Coaster Pipeline" -srcfolder "$STAGE/dmgroot" \
    -ov -format UDZO "$DMG" >/dev/null

# The .app is copied back for convenience; the .dmg is the shippable artifact.
rm -rf "$REPO/dist/$APP_NAME.app"
ditto "$STAGE/dist/$APP_NAME.app" "$REPO/dist/$APP_NAME.app"

echo
echo "app: $REPO/dist/$APP_NAME.app"
echo "dmg: $DMG"
