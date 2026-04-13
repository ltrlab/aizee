#!/bin/bash
# Run ACT policy inference.
#
# Default: runs on the Jetson via SSH with --no-rerun.
# With --local: runs on the dev machine (CPU) with Rerun visualization,
#               connecting to the Jetson's ZMQ streams over the network.
#
# Usage:
#   ./scripts/run_policy.sh                          # Jetson, no rerun
#   ./scripts/run_policy.sh --dry-run                # Jetson dry run
#   ./scripts/run_policy.sh --local                  # dev machine + rerun
#   ./scripts/run_policy.sh --local --dry-run        # dev machine dry run + rerun
#   ./scripts/run_policy.sh act_epoch_0200.pt        # specific checkpoint

set -euo pipefail

JETSON="ltr@192.168.0.27"
JETSON_IP="192.168.0.27"
SSH_KEY="/p/Workspace/ssh-keys/aizee_rover_id"
REMOTE_DIR="aizee"
LOCAL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

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

echo "=== AIZEE Policy Inference ==="
echo ""
echo "Controls:"
echo "  SPACE  — pause/resume (hold current position)"
echo "  Q      — quit (press twice to confirm, ramps to zero)"
echo ""

if $LOCAL_MODE; then
    # Run locally with Rerun, connecting to Jetson ZMQ over network
    if [[ -z "$CHECKPOINT" ]]; then
        CHECKPOINT="$LOCAL_DIR/checkpoints/act_epoch_0200.pt"
    fi
    echo "Mode: LOCAL (dev machine + Rerun)"
    echo "Checkpoint: $CHECKPOINT"
    echo ""
    exec python "$LOCAL_DIR/python/nodes/act_policy_node.py" \
        --checkpoint "$CHECKPOINT" \
        --device cpu \
        --telem "tcp://$JETSON_IP:5556" \
        --cam-left "tcp://$JETSON_IP:5563" \
        --cam-right "tcp://$JETSON_IP:5564" \
        --cmd "tcp://$JETSON_IP:5555" \
        "${EXTRA_ARGS[@]}"
else
    # Run on Jetson via SSH
    if [[ -z "$CHECKPOINT" ]]; then
        echo "Finding latest checkpoint on Jetson..."
        CHECKPOINT=$(ssh -i "$SSH_KEY" "$JETSON" \
            "ls -t ~/$REMOTE_DIR/checkpoints/act_epoch_*.pt 2>/dev/null | head -1")
        if [[ -z "$CHECKPOINT" ]]; then
            echo "ERROR: No checkpoints found on Jetson"
            exit 1
        fi
    fi
    echo "Mode: JETSON (GPU inference, no Rerun)"
    echo "Checkpoint: $CHECKPOINT"
    echo ""
    exec ssh -t -i "$SSH_KEY" "$JETSON" \
        "cd ~/$REMOTE_DIR && python3 python/nodes/act_policy_node.py \
            --checkpoint $CHECKPOINT --device cuda --no-rerun \
            ${EXTRA_ARGS[*]+${EXTRA_ARGS[*]}}"
fi
