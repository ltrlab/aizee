#!/usr/bin/env bash
# install_itmux.sh — Download and install itmux (portable tmux for Windows).
#
# Downloads the latest itmux release ZIP, extracts it to /c/itmux (C:\itmux),
# locates the tmux binary, and optionally adds it to PATH via ~/.bashrc.
#
# Usage:
#   ./scripts/install_itmux.sh           # install to /c/itmux  (default)
#   ./scripts/install_itmux.sh /d/itmux  # custom install dir
#
# On Linux/Mac this script just prints the equivalent package manager command
# and exits cleanly — safe to call unconditionally in setup workflows.

set -euo pipefail

ITMUX_VERSION="1.1.0"
ITMUX_ZIP="itmux_${ITMUX_VERSION}_x64_free.zip"
ITMUX_URL="https://github.com/itefixnet/itmux/releases/download/v${ITMUX_VERSION}/${ITMUX_ZIP}"

INSTALL_DIR="${1:-/c/itmux}"

# ---------------------------------------------------------------------------
# Non-Windows: just advise and exit
# ---------------------------------------------------------------------------
case "$(uname -s)" in
    Linux*)
        echo "Linux detected — install tmux via your package manager:"
        echo "  Debian/Ubuntu:  sudo apt install tmux"
        echo "  Fedora/RHEL:    sudo dnf install tmux"
        exit 0
        ;;
    Darwin*)
        echo "macOS detected — install tmux via Homebrew:"
        echo "  brew install tmux"
        exit 0
        ;;
    MINGW*|MSYS*|CYGWIN*)
        : # continue below
        ;;
    *)
        echo "Unknown OS — cannot install itmux automatically."
        exit 1
        ;;
esac

# ---------------------------------------------------------------------------
# Windows: download + extract
# ---------------------------------------------------------------------------
echo "Installing itmux ${ITMUX_VERSION} to ${INSTALL_DIR} ..."
echo ""

# Check dependencies
for tool in curl unzip; do
    if ! command -v "$tool" &>/dev/null; then
        echo "ERROR: '$tool' not found. It should be bundled with Git for Windows."
        echo "Make sure you're running this from a Git Bash terminal."
        exit 1
    fi
done

# Already installed?
if [[ -d "$INSTALL_DIR" ]]; then
    for sub in "usr/bin" "bin" "."; do
        for name in "tmux.exe" "tmux"; do
            candidate="${INSTALL_DIR}/${sub}/${name}"
            if [[ -x "$candidate" ]]; then
                echo "itmux is already installed at ${INSTALL_DIR}"
                echo "tmux binary: ${candidate}"
                echo ""
                echo "To reinstall, delete ${INSTALL_DIR} first and re-run."
                exit 0
            fi
        done
    done
fi

# Download
TMPDIR="$(mktemp -d)"
trap 'rm -rf "$TMPDIR"' EXIT

echo "Downloading ${ITMUX_URL} ..."
curl -L --progress-bar "$ITMUX_URL" -o "${TMPDIR}/${ITMUX_ZIP}"
echo ""

# Extract into a staging directory
STAGE="${TMPDIR}/stage"
mkdir -p "$STAGE"
echo "Extracting..."
unzip -q "${TMPDIR}/${ITMUX_ZIP}" -d "$STAGE"

# The ZIP may put everything under a single top-level folder or at the root.
# Detect and normalise.
TOPLEVEL=("$STAGE"/*/)
if [[ ${#TOPLEVEL[@]} -eq 1 && -d "${TOPLEVEL[0]}" ]]; then
    # Single top-level directory — use it as the source
    SRC="${TOPLEVEL[0]}"
else
    SRC="$STAGE"
fi

# Move into place
mkdir -p "$INSTALL_DIR"
cp -r "$SRC"/* "$INSTALL_DIR/"
echo "Extracted to ${INSTALL_DIR}"

# ---------------------------------------------------------------------------
# Locate the tmux binary
# ---------------------------------------------------------------------------
TMUX_BIN=""
for sub in "usr/bin" "bin" "."; do
    for name in "tmux.exe" "tmux"; do
        candidate="${INSTALL_DIR}/${sub}/${name}"
        if [[ -x "$candidate" ]]; then
            TMUX_BIN="$candidate"
            break 2
        fi
    done
done

echo ""
if [[ -z "$TMUX_BIN" ]]; then
    echo "WARNING: tmux binary not found in ${INSTALL_DIR}."
    echo "Contents of ${INSTALL_DIR}:"
    ls -la "$INSTALL_DIR"
    echo ""
    echo "The ZIP structure may differ from expected. Locate tmux manually"
    echo "and set ITMUX_HOME to that directory, then re-run the start script."
    exit 1
fi

echo "Found tmux: ${TMUX_BIN}"

# ---------------------------------------------------------------------------
# Version check
# ---------------------------------------------------------------------------
"$TMUX_BIN" -V 2>/dev/null || true

# ---------------------------------------------------------------------------
# PATH / ITMUX_HOME setup
# ---------------------------------------------------------------------------
TMUX_BIN_DIR="$(dirname "$TMUX_BIN")"
BASHRC="$HOME/.bashrc"

# Check if already in PATH
if echo "$PATH" | tr ':' '\n' | grep -qxF "$TMUX_BIN_DIR"; then
    echo ""
    echo "PATH already contains ${TMUX_BIN_DIR} — no changes to ${BASHRC}."
else
    echo ""
    echo "Adding to ${BASHRC}..."
    cat >> "$BASHRC" <<EOF

# itmux (portable tmux for Windows) — added by install_itmux.sh
export ITMUX_HOME="${INSTALL_DIR}"
export PATH="${TMUX_BIN_DIR}:\${PATH}"
EOF
    echo "Done. Run 'source ~/.bashrc' or open a new terminal to pick up PATH."
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo " itmux installed successfully!"
echo ""
echo " Binary : ${TMUX_BIN}"
echo " Launch : ${INSTALL_DIR}/tmux.cmd   (Mintty + tmux)"
echo ""
echo " To use the SO-101 teleop session:"
echo "   ./scripts/start_so101_teleop.sh COM14"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
