#!/usr/bin/env bash
# Start a training run in a detached tmux session named $NAME, on GPU $GPU (nvidia-smi index, default 0), nice 10.
#   NAME=train-v3 DATA=$AIZEE_SIM/gen/gen-v3/episodes [GPU=0] \
#       $AIZEE_SIM/bin/launch_train.sh --epochs 15 --num-workers 2 --save-every 5 --no-tensorboard
# AIZEE_TRAIN_SRC picks the copied training source (default $AIZEE_SIM/train-0a4d8ef); SIM_TRAIN_STATS_FROM=<checkpoint>
# keeps that checkpoint's normalization (a fine-tune with --resume).
# Checkpoints and sim_training.json / timing.jsonl land in $AIZEE_SIM/ckpt/$NAME, the log in $AIZEE_SIM/logs/$NAME.log.
set -euo pipefail
: "${NAME:?set NAME}" "${DATA:?set DATA}"
S=${AIZEE_SIM:-$HOME/aizee-sim}
GPU=${GPU:-0}; TRAIN_SRC=${AIZEE_TRAIN_SRC:-$S/train-0a4d8ef}; STATS_FROM=${SIM_TRAIN_STATS_FROM:-}
if [ -e "$S/ckpt/$NAME" ]; then echo "$S/ckpt/$NAME exists: pick another NAME" >&2; exit 1; fi
SESSION=${NAME//./_}  # tmux turns a dot in a session name into an underscore.
if tmux has-session -t "=$SESSION" 2>/dev/null; then echo "tmux session $SESSION exists" >&2; exit 1; fi
[ -d "$DATA" ] || { echo "no episodes folder $DATA" >&2; exit 1; }
[ -f "$TRAIN_SRC/python/training/train_minerva.py" ] || { echo "no training source at $TRAIN_SRC" >&2; exit 1; }
mkdir -p "$S/ckpt" "$S/logs" "$S/tmp"
cmd=(env CUDA_DEVICE_ORDER=PCI_BUS_ID CUDA_VISIBLE_DEVICES="$GPU" TORCH_HOME="$S/torch-home" TMPDIR="$S/tmp" AIZEE_TRAIN_SRC="$TRAIN_SRC" SIM_TRAIN_STATS_FROM="$STATS_FROM"
     OMP_NUM_THREADS=4 PYTHONUNBUFFERED=1 nice -n 10 "$S/.venv-train/bin/python" -u "$S/bin/sim_train.py"
     --data-dir "$DATA" --output-dir "$S/ckpt/$NAME" "$@")
printf -v line '%q ' "${cmd[@]}"
tmux new-session -d -s "$SESSION" "cd $S && { echo \"started \$(date -Is): $line\"; $line; echo \"exit \$? at \$(date -Is)\"; } 2>&1 | tee -a $S/logs/$NAME.log"
echo "started tmux session $SESSION (tmux attach -t $SESSION); log $S/logs/$NAME.log; checkpoints $S/ckpt/$NAME"
