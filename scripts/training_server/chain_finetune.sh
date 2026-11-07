#!/usr/bin/env bash
# Fine-tune a finished run's last checkpoint on a newer generation (v3.1: v3's epoch 15 on the room-look episodes).
# Waits for both:
#   - generation $GEN finished (its tmux session ended) with at least $MIN episodes (its summary.json);
#   - training $BASE finished: ckpt/$BASE/$BASE_FILE exists and its tmux session has ended (a dot in a name is an
#     underscore in tmux: train-v3.1 runs in tmux train-v3_1).
# Then writes an init checkpoint from $BASE_FILE (model, EMA, EMA count, dataset_stats and config kept; optimizer,
# scheduler and losses left out; epoch -1, so --resume starts a fresh AdamW, a fresh cosine schedule over this run and
# a reset best score), trains $NAME from it on GPU $GPU with the base's normalization (SIM_TRAIN_STATS_FROM), and
# starts a second tb_sync for $NAME (tmux tb-sync-$NAME).
#   GEN=gen-v3.1 BASE=train-v3 NAME=train-v3.1 MIN=250 [GPU=0] [BASE_FILE=minerva_epoch_0015.pt] \
#       $AIZEE_SIM/bin/chain_finetune.sh --epochs 5 --lr 3e-5 --lr-backbone 3e-6 --num-workers 6 --save-every 1 --no-tensorboard
# Its log: $AIZEE_SIM/logs/chain-$NAME.log (also on its tmux pane).
set -uo pipefail
: "${GEN:?set GEN}" "${BASE:?set BASE}" "${NAME:?set NAME}" "${MIN:?set MIN}"
S=${AIZEE_SIM:-$HOME/aizee-sim}
GPU=${GPU:-0}
BASE_FILE=${BASE_FILE:-minerva_epoch_0015.pt}
# tmux turns a dot in a session name into an underscore: gen-v3.1 runs in tmux gen-v3_1.
GEN_SESSION=${GEN//./_}; BASE_SESSION=${BASE//./_}; TB_SESSION=tb-sync-${NAME//./_}
LOG=$S/logs/chain-$NAME.log
say() { echo "$(date -Is) $*" | tee -a "$LOG"; }
say "waiting: $GEN finished (tmux $GEN_SESSION) with >= $MIN episodes, and $BASE finished ($S/ckpt/$BASE/$BASE_FILE, tmux $BASE_SESSION ended); then $NAME on GPU $GPU: $*"

if ! tmux has-session -t "=$GEN_SESSION" 2>/dev/null && [ ! -f "$S/gen/$GEN/summary.json" ]; then
  say "no generation $GEN running or finished: nothing to wait for"
  exit 1
fi
while tmux has-session -t "=$GEN_SESSION" 2>/dev/null; do sleep 60; done
summary=$S/gen/$GEN/summary.json
if [ -f "$summary" ]; then
  made=$("$S/.venv/bin/python" -c "import json, sys; print(json.load(open(sys.argv[1]))['episodes'])" "$summary")
else
  made=$(find "$S/gen/$GEN/episodes" -maxdepth 1 -name '*.hdf5' 2>/dev/null | wc -l)
  say "no $summary (did the run stop early?); counted $made episode files"
fi
say "generation $GEN finished with $made episodes"
if [ "${made:-0}" -lt "$MIN" ]; then
  say "fewer than $MIN episodes: not fine-tuning; see $S/logs/$GEN.log and $AIZEE_SIM/bin/gen_status.sh $GEN"
  exit 1
fi

while [ ! -f "$S/ckpt/$BASE/$BASE_FILE" ] || tmux has-session -t "=$BASE_SESSION" 2>/dev/null; do sleep 60; done
say "$BASE finished: $(ls -l "$S/ckpt/$BASE/$BASE_FILE")"

INIT_DIR=$S/ckpt/$NAME-init
INIT=$INIT_DIR/init_from_${BASE}_${BASE_FILE}
mkdir -p "$INIT_DIR"
if [ ! -f "$INIT" ]; then
  nice -n 10 "$S/.venv-train/bin/python" - "$S/ckpt/$BASE/$BASE_FILE" "$INIT" <<'EOF' 2>&1 | tee -a "$LOG"
import hashlib, os, sys, time
import torch
source, target = sys.argv[1], sys.argv[2]
checkpoint = torch.load(source, map_location='cpu', weights_only=False)
keep = ('model_state_dict', 'ema_state_dict', 'ema_num_updates', 'dataset_stats', 'config')
missing = [key for key in keep if key not in checkpoint]
if missing:
    raise SystemExit(f'{source} lacks {missing}')
digest = hashlib.sha256()
with open(source, 'rb') as handle:
    for block in iter(lambda: handle.read(1 << 24), b''):
        digest.update(block)
init = {key: checkpoint[key] for key in keep}
init['epoch'] = -1  # train_minerva resumes at epoch + 1 = 0.
init['init_from'] = {'path': source, 'sha256': digest.hexdigest(), 'epoch': int(checkpoint.get('epoch', -1)) + 1,
                     'left_out': sorted(set(checkpoint) - set(keep)), 'written': time.strftime('%Y-%m-%dT%H:%M:%S%z')}
torch.save(init, target + '.tmp')
os.replace(target + '.tmp', target)
print(f"init checkpoint {target}: from epoch {init['init_from']['epoch']} of {source}; kept {list(keep)}; "
      f"left out {init['init_from']['left_out']}; ema_num_updates {init['ema_num_updates']}")
EOF
fi
[ -f "$INIT" ] || { say "no init checkpoint written"; exit 1; }

# Loader workers: the --num-workers asked for, if memory holds them over the 8 GB floor: the trainer about 2 GB and each
# loader worker about 0.5 GB (v3's held 0.49-0.51 GB), seen on 3 readings 20 s apart; up to 20 minutes for that, else
# 2 workers (the eval runs only start with 21 GB free, and stop their workers under the floor).
args=("$@")
want=2
for ((i = 0; i < ${#args[@]}; i++)); do [ "${args[i]}" = --num-workers ] && want=${args[i+1]}; done
need_gb() { echo $(( 8 + 2 + ($1 + 1) / 2 )); }
avail_gb() { awk '/MemAvailable/ {printf "%d", $2/1048576}' /proc/meminfo; }
workers=$want
if [ "$want" -gt 2 ]; then
  steady=0
  for _ in $(seq 1 60); do
    if [ "$(avail_gb)" -ge "$(need_gb "$want")" ]; then steady=$((steady + 1)); else steady=0; fi
    [ "$steady" -ge 3 ] && break
    sleep 20
  done
  if [ "$steady" -lt 3 ]; then
    workers=2
    say "memory short for $want loader workers ($(avail_gb) GB available, $(need_gb "$want") GB wanted): using 2"
  fi
fi
for ((i = 0; i < ${#args[@]}; i++)); do [ "${args[i]}" = --num-workers ] && args[i+1]=$workers; done
say "loader workers: $workers ($(avail_gb) GB available)"
say "GPU before training: $(nvidia-smi --query-gpu=index,memory.used,utilization.gpu --format=csv,noheader | tr '\n' ';')"
say "memory before training: $(free -g | sed -n 2p)"
NAME=$NAME DATA=$S/gen/$GEN/episodes GPU=$GPU SIM_TRAIN_STATS_FROM=$INIT "$S/bin/launch_train.sh" --resume "$INIT" \
  "${args[@]}" 2>&1 | tee -a "$LOG"
say "launch_train.sh exit ${PIPESTATUS[0]}"
if ! tmux has-session -t "=$TB_SESSION" 2>/dev/null; then
  tmux new-session -d -s "$TB_SESSION" \
    "cd $S && nice -n 10 .venv-train/bin/python -u bin/tb_sync.py --run $NAME 2>&1 | tee -a logs/tb-sync-$NAME.log"
  say "started tb_sync for $NAME (tmux $TB_SESSION): TensorBoard run $NAME beside train-v3 and v2"
fi
