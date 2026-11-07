#!/usr/bin/env bash
# Train LeRobot ACT on an exported Minerva dataset (python -m robot_platform.data.lerobot_export) in the .venv-lerobot
# that setup_lerobot.sh builds, in a detached tmux session named $NAME (nice 10) on GPU $GPU (nvidia-smi index).
#   NAME=act-demo1 DATA=$AIZEE_SIM/data/lerobot/minerva-handoff-demo1 [GPU=0] [STEPS=200] [BATCH=8] \
#       train_act.sh [more LeRobot train options, e.g. --policy.chunk_size=50]
# The defaults are the 2026-10-22 smoke run's (act-demo1-smoke: batch 8, 200 steps, seed 1000, ACT's chunk of 100 and
# learning rate 1e-5). Checkpoints land in $AIZEE_SIM/ckpt/$NAME, the log in $AIZEE_SIM/logs/$NAME.log. Export the
# result for the robot with: python -m robot_platform.data.lerobot_policy export <ckpt>/checkpoints/<step>/pretrained_model ...
set -euo pipefail
: "${NAME:?set NAME}" "${DATA:?set DATA}"
S=${AIZEE_SIM:-$HOME/aizee-sim}
GPU=${GPU:-0}
STEPS=${STEPS:-200}
BATCH=${BATCH:-8}
if [ -e "$S/ckpt/$NAME" ]; then
    echo "$S/ckpt/$NAME exists: pick another NAME" >&2
    exit 1
fi
SESSION=${NAME//./_} # tmux turns a dot in a session name into an underscore.
if tmux has-session -t "=$SESSION" 2>/dev/null; then
    echo "tmux session $SESSION exists" >&2
    exit 1
fi
[ -f "$DATA/meta/info.json" ] || {
    echo "no LeRobot dataset at $DATA" >&2
    exit 1
}
[ -x "$S/.venv-lerobot/bin/python" ] || {
    echo "no $S/.venv-lerobot: run setup_lerobot.sh" >&2
    exit 1
}
mkdir -p "$S/ckpt" "$S/logs" "$S/tmp"
cmd=(env CUDA_DEVICE_ORDER=PCI_BUS_ID CUDA_VISIBLE_DEVICES="$GPU" TORCH_HOME="$S/torch-home" TMPDIR="$S/tmp"
    PYTHONUNBUFFERED=1 nice -n 10 "$S/.venv-lerobot/bin/python" -m lerobot.scripts.train
    --policy.type=act --policy.device=cuda --dataset.repo_id="minerva/$(basename "$DATA")" --dataset.root="$DATA"
    --dataset.video_backend=torchcodec --batch_size="$BATCH" --steps="$STEPS" --save_freq="$STEPS" --log_freq=25
    --seed=1000 --num_workers=4 --eval_freq=0 --wandb.enable=false --job_name=act --output_dir="$S/ckpt/$NAME" "$@")
printf -v line '%q ' "${cmd[@]}"
tmux new-session -d -s "$SESSION" "cd $S && { echo \"started \$(date -Is): $line\"; $line; echo \"exit \$? at \$(date -Is)\"; } 2>&1 | tee -a $S/logs/$NAME.log"
echo "started tmux session $SESSION (tmux attach -t $SESSION); log $S/logs/$NAME.log; checkpoints $S/ckpt/$NAME"
