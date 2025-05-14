#!/bin/bash

echo "Starting custom entrypoint script..."
echo "Setting permissions for /config/.XDG..."
mkdir -p /config/.XDG
chown -R $PUID:$PGID /config/.XDG
chmod 0700 /config/.XDG

# Start the SSH server in the background
echo "Starting SSH Server..."
/usr/sbin/sshd -D &

# Execute the original entrypoint to start the WebTop server
exec /init
