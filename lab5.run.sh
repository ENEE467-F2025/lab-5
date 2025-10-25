#!/bin/bash
set -e

LAB_NUM=5

cd ~/Labs || exit 1

if [ ! -d "lab-$LAB_NUM" ]; then
    git clone https://github.com/ENEE467-F2025/lab-$LAB_NUM.git
fi

cd lab-$LAB_NUM/docker || exit 1

echo "Checking for lab-$LAB_NUM Docker image..."
docker image ls | grep -q "lab-$LAB_NUM-image" || {
    echo "ERROR: Docker image lab-$LAB_NUM-image not found."
    echo "Please ask your instructor to rebuild using lab$LAB_NUM.build.sh"
    exit 1
}

echo "Launching container..."
userid=$(id -u) groupid=$(id -g) docker compose -f lab-$LAB_NUM-compose.yml up -d

# Allow local container to access X server (Linux host). Safe on lab machines.
if command -v xhost >/dev/null 2>&1; then
    xhost +local: 1>/dev/null 2>&1 || true
fi

# VS Code extension check
if ! code --list-extensions | grep -q ms-vscode-remote.remote-containers; then
    echo "Installing VSCode Dev Containers extension..."
    code --install-extension ms-vscode-remote.remote-containers
else
    echo "VSCode Dev Containers extension already installed."
fi

echo "Attaching VSCode to container..."
# Wait for the container to be up
for i in {1..20}; do
    CONTAINER_ID=$(docker ps -q --filter name=lab-$LAB_NUM-container)
    if [ -n "$CONTAINER_ID" ]; then
        break
    fi
    sleep 0.5
done

if [ -z "$CONTAINER_ID" ]; then
    echo "ERROR: lab-$LAB_NUM container did not start. Check Docker logs."
    exit 1
fi

ENCODED=$(printf "%s" "$CONTAINER_ID" | od -A n -t x1 | sed 's/ *//g' | tr -d '\n')
code --folder-uri vscode-remote://attached-container+$ENCODED/home/robot/ros2_ws
echo "All packages for Lab $LAB_NUM found. Docker setup is correct."
echo "Container launched and VSCode attached."
echo "You can now start working on Lab $LAB_NUM!"