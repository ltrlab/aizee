#!/bin/bash
# Bootstrap a new Raspberry Pi onto the PoE Ethernet subnet (10.42.0.0/24)
# Run this when a Pi first connects to the PoE switch and gets a DHCP address.
#
# Usage: ./setup_pi_ethernet.sh <pi_number> [pi_dhcp_ip]
#   pi_number   : 1-4  (determines static IP 10.42.0.1{1-4} and hostname PI-{1-4})
#   pi_dhcp_ip  : DHCP address assigned by Jetson (check `arp -n` on Jetson)
#
# Prerequisites:
#   - Jetson must be reachable at 192.168.0.27 with SSH key
#   - Pi must have gotten a DHCP lease from Jetson dnsmasq (10.42.0.10-20)
#   - Pi SSH password is 'changeme' (default for fresh images)
#
# Steps performed:
#   1. Install SSH public key on Pi (via sshpass from Jetson)
#   2. Configure static IP on Pi Ethernet interface
#   3. Verify SSH key auth works

set -e

SSH_KEY="/p/Workspace/ssh-keys/aizee_rover_id"
JETSON_IP="192.168.0.27"
JETSON_SSH="ssh -i $SSH_KEY ltr@$JETSON_IP"

PI_NUM="${1:-}"
DHCP_IP="${2:-}"

if [[ -z "$PI_NUM" ]]; then
    echo "Usage: $0 <pi_number> [pi_dhcp_ip]"
    echo ""
    echo "  pi_number: 1=cam_front(10.42.0.11), 2=cam_rear(10.42.0.12),"
    echo "             3=cam_left(10.42.0.13),  4=cam_right(10.42.0.14)"
    echo ""
    echo "  To find DHCP IP: ssh to Jetson and run: arp -n | grep 10.42.0"
    echo "  Or check dnsmasq leases: cat /var/lib/misc/dnsmasq.leases"
    exit 1
fi

case "$PI_NUM" in
    1) STATIC_IP="10.42.0.11"; CAM_ID="cam_front";  HOSTNAME="AIZEE-ROVER-PI-1" ;;
    2) STATIC_IP="10.42.0.12"; CAM_ID="cam_rear";   HOSTNAME="AIZEE-ROVER-PI-2" ;;
    3) STATIC_IP="10.42.0.13"; CAM_ID="cam_left";   HOSTNAME="AIZEE-ROVER-PI-3" ;;
    4) STATIC_IP="10.42.0.14"; CAM_ID="cam_right";  HOSTNAME="AIZEE-ROVER-PI-4" ;;
    *)
        echo "ERROR: pi_number must be 1-4"
        exit 1
        ;;
esac

# If DHCP IP not provided, try to discover it
if [[ -z "$DHCP_IP" ]]; then
    echo "No DHCP IP provided. Checking Jetson ARP table..."
    DHCP_IP=$($JETSON_SSH ltr@$JETSON_IP "arp -n | grep '10\.42\.0\.' | grep -v $STATIC_IP | awk '{print \$1}' | head -1" 2>/dev/null || true)
    if [[ -z "$DHCP_IP" ]]; then
        echo "ERROR: Could not auto-detect Pi DHCP IP."
        echo "Run on Jetson: arp -n | grep 10.42.0"
        echo "Then retry: $0 $PI_NUM <dhcp_ip>"
        exit 1
    fi
    echo "Found Pi at DHCP IP: $DHCP_IP"
fi

echo "=== Setting up Pi $PI_NUM ==="
echo "DHCP IP:   $DHCP_IP"
echo "Static IP: $STATIC_IP"
echo "Camera:    $CAM_ID"
echo "Hostname:  $HOSTNAME"
echo ""

# Step 1: Copy SSH public key from dev machine to Jetson (if not already there)
PUB_KEY="${SSH_KEY}.pub"
if [[ ! -f "$PUB_KEY" ]]; then
    echo "ERROR: SSH public key not found: $PUB_KEY"
    exit 1
fi

echo "1. Copying SSH public key to Jetson..."
scp -i $SSH_KEY "$PUB_KEY" ltr@$JETSON_IP:/tmp/aizee_rover_id.pub

echo ""
echo "2. Installing SSH key on Pi $PI_NUM via sshpass (password: changeme)..."
$JETSON_SSH "
    sshpass -p changeme ssh -o StrictHostKeyChecking=no ltr@$DHCP_IP \
        'mkdir -p ~/.ssh && chmod 700 ~/.ssh && cat >> ~/.ssh/authorized_keys' \
        < /tmp/aizee_rover_id.pub
    rm /tmp/aizee_rover_id.pub
"

echo ""
echo "3. Configuring static IP $STATIC_IP on Pi..."
$JETSON_SSH "ssh -i ~/.ssh/aizee_rover_id -o StrictHostKeyChecking=no ltr@$DHCP_IP \"
    nmcli con mod 'Wired connection 1' ipv4.method manual ipv4.addresses $STATIC_IP/24 ipv4.gateway '' ipv4.dns '' && nmcli con up 'Wired connection 1'
\""

echo ""
echo "4. Verifying SSH key auth at static IP (waiting 5s for interface to come up)..."
sleep 5

if $JETSON_SSH "ssh -i ~/.ssh/aizee_rover_id -o StrictHostKeyChecking=no ltr@$STATIC_IP 'echo SSH key auth: OK'" 2>/dev/null; then
    echo ""
    echo "=== Pi $PI_NUM ($CAM_ID) setup complete! ==="
    echo ""
    echo "Pi is now reachable via:"
    echo "  ssh -i $SSH_KEY ltr@$JETSON_IP \"ssh -i ~/.ssh/aizee_rover_id ltr@$STATIC_IP\""
    echo ""
    echo "Next: deploy camera node"
    echo "  ./scripts/deploy_rpi4_camera_scp.sh $CAM_ID"
else
    echo ""
    echo "WARNING: Could not verify SSH at $STATIC_IP — interface may still be coming up."
fi
