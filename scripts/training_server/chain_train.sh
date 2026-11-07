#!/usr/bin/env bash
# Wait for a generation run (launch_gen.sh) to finish, then start training on its episodes with launch_train.sh, only
# if it made at least MIN episodes. Run it in its own tmux session:
#   GEN=gen-v3 NAME=train-v3 MIN=250 [GPU=0] $AIZEE_SIM/bin/chain_train.sh --epochs 15 --num-workers 2 --save-every 5 --no-tensorboard
# Its log: $AIZEE_SIM/logs/chain-$NAME.log (also on its tmux pane).
set -uo pipefail
: "${GEN:?set GEN}" "${NAME:?set NAME}" "${MIN:?set MIN}"
S=${AIZEE_SIM:-$HOME/aizee-sim}
GPU=${GPU:-0}
LOG=$S/logs/chain-$NAME.log
say() { echo "$(date -Is) $*" | tee -a "$LOG"; }
say "waiting for generation $GEN (tmux session $GEN, $S/gen/$GEN/summary.json); then train $NAME on GPU $GPU if >= $MIN episodes: $*"
if ! tmux has-session -t "=$GEN" 2>/dev/null && [ ! -f "$S/gen/$GEN/summary.json" ]; then
    say "no generation $GEN running or finished: nothing to wait for"
    exit 1
fi
while tmux has-session -t "=$GEN" 2>/dev/null; do sleep 60; done
summary=$S/gen/$GEN/summary.json
if [ -f "$summary" ]; then
    made=$("$S/.venv/bin/python" -c "import json, sys; print(json.load(open(sys.argv[1]))['episodes'])" "$summary")
else
    made=$(find "$S/gen/$GEN/episodes" -maxdepth 1 -name '*.hdf5' 2>/dev/null | wc -l)
    say "no $summary (did the run stop early?); counted $made episode files"
fi
say "generation $GEN finished with $made episodes"
if [ "${made:-0}" -lt "$MIN" ]; then
    say "fewer than $MIN episodes: not training; see $S/logs/$GEN.log and $AIZEE_SIM/bin/gen_status.sh $GEN"
    exit 1
fi
say "GPU before training: $(nvidia-smi --query-gpu=index,memory.used,utilization.gpu --format=csv,noheader | tr '\n' ';')"
NAME=$NAME DATA=$S/gen/$GEN/episodes GPU=$GPU "$S/bin/launch_train.sh" "$@" 2>&1 | tee -a "$LOG"
say "launch_train.sh exit ${PIPESTATUS[0]}"
