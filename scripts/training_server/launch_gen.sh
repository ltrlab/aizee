#!/usr/bin/env bash
# Start a generation run in a detached tmux session named $NAME (nice 10, MuJoCo on GPU $EGL through EGL, default 0).
#   NAME=gen-v3 EPISODES=300 [WORKERS=6] [TREE=$AIZEE_SIM/tree-01e2198] [SEED=1000] [EGL=0] \
#       $AIZEE_SIM/bin/launch_gen.sh [more generate options, e.g. --grid 6x5 --jitter 1.0 --yaw 0.785398]
# Episodes land in $AIZEE_SIM/gen/$NAME/episodes (manifest.jsonl, summary.json beside them; w<k>.log per worker),
# the run's own output in $AIZEE_SIM/logs/$NAME.log. Progress: $AIZEE_SIM/bin/gen_status.sh $NAME
set -euo pipefail
: "${NAME:?set NAME}" "${EPISODES:?set EPISODES}"
S=${AIZEE_SIM:-$HOME/aizee-sim}
WORKERS=${WORKERS:-6}; TREE=${TREE:-$S/tree-01e2198}; SEED=${SEED:-1000}; EGL=${EGL:-0}
if [ -e "$S/gen/$NAME" ]; then echo "$S/gen/$NAME exists: pick another NAME" >&2; exit 1; fi
SESSION=${NAME//./_}  # tmux turns a dot in a session name into an underscore.
if tmux has-session -t "=$SESSION" 2>/dev/null; then echo "tmux session $SESSION exists" >&2; exit 1; fi
[ -d "$TREE/src/robot_platform" ] || { echo "no source tree at $TREE" >&2; exit 1; }
mkdir -p "$S/gen" "$S/logs" "$S/tmp"
cmd=("$S/.venv/bin/python" -u "$S/bin/gen_parallel.py" --out "$S/gen/$NAME" --episodes "$EPISODES"
     --workers "$WORKERS" --tree "$TREE" --seed "$SEED" --egl-device "$EGL" -- "$@")
printf -v line '%q ' "${cmd[@]}"
tmux new-session -d -s "$SESSION" "cd $S && { echo \"started \$(date -Is): $line\"; $line; echo \"exit \$? at \$(date -Is)\"; } 2>&1 | tee -a $S/logs/$NAME.log"
echo "started tmux session $SESSION (tmux attach -t $SESSION); log $S/logs/$NAME.log; episodes $S/gen/$NAME/episodes"
