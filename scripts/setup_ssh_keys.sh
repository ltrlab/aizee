#!/bin/bash
# Setup SSH key authentication for all 4 Raspberry Pis
# You'll need to enter the password once for each Pi

echo "=== Setting up SSH key authentication for AIZEE camera Pis ==="
echo "You'll need to enter password 'changeme' once for each Pi"
echo ""

# Copy SSH key to each Pi
for ip in 22 23 24 25; do
    echo "Copying SSH key to 192.168.0.$ip..."
    ssh-copy-id -i ~/.ssh/id_ed25519.pub ltr@192.168.0.$ip
    echo ""
done

echo "=== SSH key setup complete! ==="
echo "You can now run commands without entering passwords."
