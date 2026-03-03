#!/usr/bin/env bash
# start_so101_teleop.sh — Launch a tmux session for SO-101 leader-arm teleop + Rerun viewer.
#
# Layout:
#   ┌───────────────────────────────────────────┬─────────────────────────┐
#   │  so101_teleop.py            (left, 60%)   │  so101_rerun.py         │
#   │  terminal UI — joint readout, state       │  (top-right, 70%)       │
#   │  machine, torque/temp warnings            │  Rerun GUI auto-spawns  │
#   │                                           ├─────────────────────────┤
#   │                                           │  shell  (bottom-right)  │
#   │                                           │  handy commands below   │
#   └───────────────────────────────────────────┴─────────────────────────┘
#
# Usage:
#   ./scripts/start_so101_teleop.sh <port>
#   ./scripts/start_so101_teleop.sh <port> <jetson-ip>
#   ./scripts/start_so101_teleop.sh <port> <jetson-ip> --record
#   ./scripts/start_so101_teleop.sh <port> <jetson-ip> --max-delta 0.05
#
# Arguments:
#   <port>       SO-101 serial port  (required, e.g. COM14 or /dev/ttyACM0)
#   <jetson-ip>  optional Jetson IP  (default: 192.168.0.27)
#   --record     save a timestamped MCAP via so101_rerun.py
#   remaining    forwarded to so101_teleop.py
#
# Windows (itmux):
#   itmux creates Cygwin panes which use /cygdrive/X/ paths and have their own
#   PATH. This script resolves both automatically — just run it from Git Bash.

set -euo pipefail

SESSION="so101-teleop"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# ---------------------------------------------------------------------------
# OS detection
# ---------------------------------------------------------------------------
_os() {
    case "$(uname -s)" in
        Linux*)               echo "linux"   ;;
        Darwin*)              echo "mac"     ;;
        MINGW*|MSYS*|CYGWIN*) echo "windows" ;;
        *)                    echo "unknown" ;;
    esac
}
OS="$(_os)"

# ---------------------------------------------------------------------------
# Path conversion: MSYS2 /x/foo  →  Cygwin /cygdrive/x/foo
# itmux's tmux is a Cygwin binary; its panes use Cygwin bash which maps
# drives at /cygdrive/X/ rather than MSYS2's /X/ shorthand.
# On Linux the function is a no-op.
# ---------------------------------------------------------------------------
_to_pane_path() {
    if [[ "$OS" == "windows" ]]; then
        # /p/Workspace/aizee  →  /cygdrive/p/Workspace/aizee
        echo "$1" | sed 's|^/\([a-zA-Z]\)/|/cygdrive/\1/|'
    else
        echo "$1"
    fi
}

# ---------------------------------------------------------------------------
# Python detection
# Resolves the full path from the CURRENT shell (MSYS2/Git Bash) and converts
# it to a Cygwin-accessible path so itmux panes can call the same interpreter.
# ---------------------------------------------------------------------------
_find_python() {
    local py_path=""
    for name in python python3 py; do
        if command -v "$name" &>/dev/null; then
            py_path="$(command -v "$name")"
            break
        fi
    done

    if [[ -z "$py_path" ]]; then
        echo ""
        return 1
    fi

    _to_pane_path "$py_path"
}

# ---------------------------------------------------------------------------
# tmux detection (unchanged from before)
# ---------------------------------------------------------------------------
_find_tmux() {
    if command -v tmux &>/dev/null; then
        command -v tmux; return 0
    fi
    if [[ "$OS" == "windows" ]]; then
        local search_bases=()
        [[ -n "${ITMUX_HOME:-}" ]] && search_bases+=("$ITMUX_HOME")
        search_bases+=(
            "/c/itmux" "/d/itmux"
            "$HOME/itmux" "$USERPROFILE/itmux"
            "$USERPROFILE/Downloads/itmux"
        )
        for base in "${search_bases[@]}"; do
            for sub in "usr/bin" "bin" "."; do
                for name in "tmux.exe" "tmux"; do
                    local c="$base/$sub/$name"
                    [[ -x "$c" ]] && { echo "$c"; return 0; }
                done
            done
        done
    fi
    return 1
}

# ---------------------------------------------------------------------------
# Parse arguments
# ---------------------------------------------------------------------------
if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <serial-port> [jetson-ip] [--record] [extra teleop args...]"
    echo "  e.g. $0 COM14"
    echo "       $0 /dev/ttyACM0 192.168.0.27 --record"
    exit 1
fi

PORT="$1"; shift

HOST="192.168.0.27"
if [[ $# -ge 1 && "$1" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    HOST="$1"; shift
fi

RECORD=0
TELEOP_EXTRA=()
for arg in "$@"; do
    if [[ "$arg" == "--record" ]]; then
        RECORD=1
    else
        TELEOP_EXTRA+=("$arg")
    fi
done

CMD_ADDR="tcp://${HOST}:5555"
TELEM_ADDR="tcp://${HOST}:5556"
UPS_ADDR="tcp://${HOST}:5562"
TELEOP_PUB="tcp://*:5570"
TELEOP_SUB="tcp://localhost:5570"

# ---------------------------------------------------------------------------
# Resolve paths + python for use inside tmux panes
# ---------------------------------------------------------------------------
PANE_REPO="$(_to_pane_path "$REPO_ROOT")"

PYTHON="$(_find_python)" || true
if [[ -z "$PYTHON" ]]; then
    echo "ERROR: python not found in current environment."
    echo "Make sure Python is installed and accessible from this shell."
    exit 1
fi
echo "Python  : ${PYTHON}"

# ---------------------------------------------------------------------------
# Build command strings
# ---------------------------------------------------------------------------
TELEOP_CMD="'${PYTHON}' python/scripts/so101_teleop.py \
    --port ${PORT} \
    --cmd  ${CMD_ADDR} \
    --telem ${TELEM_ADDR} \
    --ups   ${UPS_ADDR} \
    --teleop-pub ${TELEOP_PUB}"
if [[ ${#TELEOP_EXTRA[@]} -gt 0 ]]; then
    TELEOP_CMD="${TELEOP_CMD} ${TELEOP_EXTRA[*]}"
fi

RERUN_CMD="'${PYTHON}' python/scripts/so101_rerun.py \
    --host  ${HOST} \
    --teleop ${TELEOP_SUB}"
if [[ $RECORD -eq 1 ]]; then
    STAMP="$(date +%Y%m%d_%H%M%S)"
    MCAP_PATH="logs/so101_teleop_${STAMP}.mcap"
    RERUN_CMD="${RERUN_CMD} --save ${MCAP_PATH}"
fi

# ---------------------------------------------------------------------------
# Find tmux — or explain how to get it
# ---------------------------------------------------------------------------
TMUX_BIN=""
if ! TMUX_BIN="$(_find_tmux)"; then
    echo ""
    echo "ERROR: tmux not found."
    if [[ "$OS" == "windows" ]]; then
        echo ""
        echo "Install itmux (https://github.com/itefixnet/itmux):"
        echo "  ./scripts/install_itmux.sh"
        echo ""
        echo "Or set ITMUX_HOME to your itmux folder:"
        echo "  ITMUX_HOME=/c/itmux ./scripts/start_so101_teleop.sh ${PORT}"
    else
        echo "  sudo apt install tmux   # Debian/Ubuntu"
    fi
    echo ""
    echo "Manual fallback — run in separate terminals:"
    echo "  cd ${REPO_ROOT} && ${TELEOP_CMD}"
    echo "  cd ${REPO_ROOT} && ${RERUN_CMD}"
    exit 1
fi

echo "tmux    : ${TMUX_BIN}"
echo "Repo    : ${PANE_REPO}"
echo ""

# ---------------------------------------------------------------------------
# Attach if session already exists
# ---------------------------------------------------------------------------
if "$TMUX_BIN" has-session -t "$SESSION" 2>/dev/null; then
    echo "Session '$SESSION' already running — attaching."
    exec "$TMUX_BIN" attach-session -t "$SESSION"
fi

# ---------------------------------------------------------------------------
# Create session  (no -c flag — we cd explicitly via send-keys below,
# using the Cygwin-converted path that itmux panes understand)
# ---------------------------------------------------------------------------
"$TMUX_BIN" new-session -d -s "$SESSION"

# Right column (40% of width)
"$TMUX_BIN" split-window -h -p 40 -t "${SESSION}:0"

# Bottom-right shell pane (30% of right-column height)
"$TMUX_BIN" split-window -v -p 30 -t "${SESSION}:0.1"

# ---------------------------------------------------------------------------
# Populate panes  — each pane does:
#   1. cd to repo (Cygwin path)
#   2. run the actual command
# ---------------------------------------------------------------------------

# Pane 0.2 — shell with usage hints
"$TMUX_BIN" send-keys -t "${SESSION}:0.2" "cd '${PANE_REPO}'" Enter
"$TMUX_BIN" send-keys -t "${SESSION}:0.2" "clear" Enter
"$TMUX_BIN" send-keys -t "${SESSION}:0.2" "cat <<'HINTS'

  ── so101 teleop — quick reference ──────────────────────────────────

  Re-attach after detaching (Ctrl-b d):
    ${TMUX_BIN} attach-session -t ${SESSION}

  Kill everything:
    ${TMUX_BIN} kill-session -t ${SESSION}

  Start a fresh recording (MCAP):
    '${PYTHON}' python/scripts/so101_rerun.py \\
        --teleop ${TELEOP_SUB} --host ${HOST} \\
        --save logs/so101_\$(date +%Y%m%d_%H%M%S).mcap

  Restart motor service on Jetson (if arm is unresponsive):
    ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@${HOST} \\
        'sudo systemctl restart aizee-motor-control-rover'

  ────────────────────────────────────────────────────────────────────
HINTS" Enter

# Pane 0.1 — so101_rerun.py (Rerun GUI auto-spawns as a separate window)
"$TMUX_BIN" send-keys -t "${SESSION}:0.1" "cd '${PANE_REPO}'" Enter
"$TMUX_BIN" send-keys -t "${SESSION}:0.1" "${RERUN_CMD}" Enter

# Pane 0.0 — so101_teleop.py (main interactive terminal UI)
"$TMUX_BIN" select-pane -t "${SESSION}:0.0"
"$TMUX_BIN" send-keys -t "${SESSION}:0.0" "cd '${PANE_REPO}'" Enter
"$TMUX_BIN" send-keys -t "${SESSION}:0.0" "${TELEOP_CMD}" Enter

# ---------------------------------------------------------------------------
# Attach
# ---------------------------------------------------------------------------
exec "$TMUX_BIN" attach-session -t "$SESSION"
