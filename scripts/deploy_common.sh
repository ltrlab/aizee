# Shared target/key resolution for the AIZEE deploy scripts. Source, don't run:
#   source "$(dirname "$0")/deploy_common.sh"
#
# Provides:
#   AIZEE_TARGET  — user@host default for the Jetson. Tries, in order, any
#                   already-set $AIZEE_TARGET, then the first reachable of the
#                   known addresses (USB-C tether, WiFi AP, legacy LAN).
#   SSH_KEY       — private key path, auto-detected across the git-bash /c
#                   path, the P: mapped-drive path, and ~/.ssh.
#
# Every deploy script still accepts an explicit target as $1, which wins.

# --- SSH key -----------------------------------------------------------------
if [[ -z "${SSH_KEY:-}" || ! -f "${SSH_KEY:-}" ]]; then
    for _aizee_k in /c/Users/ltr/Workspace/ssh-keys/aizee_rover_id \
                    /p/Workspace/ssh-keys/aizee_rover_id \
                    "$HOME/.ssh/aizee_rover_id"; do
        if [[ -f "$_aizee_k" ]]; then SSH_KEY="$_aizee_k"; break; fi
    done
fi
if [[ -z "${SSH_KEY:-}" ]]; then
    echo "ERROR: aizee_rover_id SSH key not found (set SSH_KEY=<path>)." >&2
    return 1 2>/dev/null || exit 1
fi

# --- target ------------------------------------------------------------------
# 10.42.0.1  = USB-C shared link, 192.168.50.1 = WiFi AP "aizee",
# 192.168.0.27 = legacy LAN static address.
if [[ -z "${AIZEE_TARGET:-}" ]]; then
    for _aizee_h in 10.42.0.1 192.168.50.1 192.168.0.27; do
        if ssh -i "$SSH_KEY" -o BatchMode=yes -o ConnectTimeout=2 \
               -o StrictHostKeyChecking=accept-new "ltr@$_aizee_h" true 2>/dev/null; then
            AIZEE_TARGET="ltr@$_aizee_h"
            break
        fi
    done
    AIZEE_TARGET="${AIZEE_TARGET:-ltr@10.42.0.1}"
fi
