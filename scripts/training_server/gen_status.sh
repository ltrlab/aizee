#!/usr/bin/env bash
# Progress of a generation run started by launch_gen.sh: gen_status.sh NAME
d=${AIZEE_SIM:-$HOME/aizee-sim}/gen/${1:?give the run NAME}
total=0
ok=0
for f in "$d"/w*.log; do
    [ -e "$f" ] || continue
    a=$(grep -c '] attempt ' "$f" || true)
    s=$(grep -c '] attempt [0-9]*: success' "$f" || true)
    total=$((total + a))
    ok=$((ok + s))
    printf '%s: %s attempts, %s successes; last: %s\n' "$(basename "$f" .log)" "$a" "$s" "$(grep '] attempt ' "$f" | tail -1)"
done
echo "all: $ok successes from $total attempts; $(du -sh "$d" 2>/dev/null | cut -f1) on disk"
[ -f "$d/summary.json" ] && echo "finished: $(head -c 400 "$d/summary.json")"
