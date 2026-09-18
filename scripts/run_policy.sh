#!/usr/bin/env bash
# Aizee ACT inference with an explicitly selected compatible checkpoint.
# Local:  ./scripts/run_policy.sh --local checkpoints/aizee/act_best.pt --dry-run
# Jetson: ./scripts/run_policy.sh checkpoints/aizee/act_best.pt --dry-run
# Set JETSON=user@host, JETSON_IP=host and SSH_KEY=path for your deployment.
set -euo pipefail

LOCAL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
JETSON="${JETSON:-ltr@192.168.0.27}"
JETSON_IP="${JETSON_IP:-${JETSON#*@}}"
SSH_KEY="${SSH_KEY:-$LOCAL_DIR/../ssh-keys/aizee_rover_id}"
REMOTE_DIR="${AIZEE_REMOTE_DIR:-aizee}"
LOCAL_MODE=false
CHECKPOINT=""
EXTRA_ARGS=()
for arg in "$@"; do
    if [[ "$arg" == "--local" ]]; then
        LOCAL_MODE=true
    elif [[ -z "$CHECKPOINT" && "$arg" == *.pt ]]; then
        CHECKPOINT="$arg"
    else
        EXTRA_ARGS+=("$arg")
    fi
done

if [[ -z "$CHECKPOINT" ]]; then
    echo "Usage: $0 [--local] path/to/checkpoint.pt [policy options]" >&2
    echo "Train or supply a compatible checkpoint; old weights are no longer bundled locally." >&2
    exit 2
fi

if $LOCAL_MODE; then
    if [[ ! -f "$CHECKPOINT" ]]; then
        echo "Checkpoint not found: $CHECKPOINT" >&2
        exit 2
    fi
    exec python "$LOCAL_DIR/python/nodes/act_policy_node.py" \
        --checkpoint "$CHECKPOINT" --device cpu \
        --telem "tcp://$JETSON_IP:5556" \
        --gripper-cam "tcp://$JETSON_IP:5563" \
        --cmd "tcp://$JETSON_IP:5555" "${EXTRA_ARGS[@]}"
else
    # Preserve argument boundaries through the remote shell, including spaces
    # in checkpoint paths. Remote checkpoint paths are relative to the checkout.
    printf -v remote_dir '%q' "$REMOTE_DIR"
    printf -v remote_command '%q ' python3 python/nodes/act_policy_node.py \
        --checkpoint "$CHECKPOINT" --device cuda --no-rerun "${EXTRA_ARGS[@]}"
    exec ssh -t -i "$SSH_KEY" "$JETSON" "cd \"\$HOME\"/$remote_dir && $remote_command"
fi
