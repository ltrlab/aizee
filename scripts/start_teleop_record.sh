#!/usr/bin/env bash
# start_teleop_record.sh — Launch a tmux session for teleop + recording.
#
# Layout:
#   ┌──────────────────────────────────┬─────────────────────────┐
#   │  teleop_record.py  (left, 62%)   │  shell  (right, 38%)    │
#   │  drive arm + record trajectories │  run visualize / replay │
#   └──────────────────────────────────┴─────────────────────────┘
#
# Usage:
#   ./scripts/start_teleop_record.sh                              # local mock
#   ./scripts/start_teleop_record.sh 192.168.0.27                 # real hardware
#   ./scripts/start_teleop_record.sh 192.168.0.27 --max-delta 0.03
#
# Arguments:
#   $1  optional IP (default: localhost) — sets --cmd and --telem addresses
#   remaining args forwarded to teleop_record.py

set -euo pipefail

SESSION="aizee-teleop"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# --- Resolve host from optional first argument ---
HOST="localhost"
EXTRA_ARGS=()
if [[ $# -ge 1 && "$1" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    HOST="$1"
    shift
fi
EXTRA_ARGS=("$@")

CMD_ADDR="tcp://${HOST}:5555"
TELEM_ADDR="tcp://${HOST}:5556"

TELEOP_CMD="python python/scripts/teleop_record.py \
    --cmd ${CMD_ADDR} \
    --telem ${TELEM_ADDR} \
    ${EXTRA_ARGS[*]+"${EXTRA_ARGS[@]}"}"

# --- If session already exists, attach and exit ---
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Session '$SESSION' already running — attaching."
    exec tmux attach-session -t "$SESSION"
fi

# --- Create new detached session (main pane = teleop_record.py) ---
tmux new-session -d -s "$SESSION" -c "$REPO_ROOT"

# Right pane (38% width) — interactive shell with usage hints
tmux split-window -h -p 38 -t "${SESSION}:0" -c "$REPO_ROOT"
tmux send-keys -t "${SESSION}:0.1" "clear" Enter
tmux send-keys -t "${SESSION}:0.1" "cat <<'HINTS'

  ── Useful commands ──────────────────────────────────────────────

  Visualize a recording in Rerun:
    python python/scripts/record_replay.py visualize \\
        recordings/recording_XXXX.hdf5

  Visualize a collected episode:
    python python/scripts/record_replay.py visualize \\
        episodes/episode_XXXX.hdf5

  Dry-run replay (no hardware):
    python python/scripts/record_replay.py replay \\
        recordings/recording_XXXX.hdf5 --dry-run

  Live replay on hardware:
    python python/scripts/record_replay.py replay \\
        recordings/recording_XXXX.hdf5 \\
        --live --goto-start \\
        --cmd ${CMD_ADDR} --telem ${TELEM_ADDR}

  ─────────────────────────────────────────────────────────────────
HINTS" Enter

# Left pane — run teleop_record.py
tmux select-pane -t "${SESSION}:0.0"
tmux send-keys -t "${SESSION}:0.0" "$TELEOP_CMD" Enter

# Attach
exec tmux attach-session -t "$SESSION"
