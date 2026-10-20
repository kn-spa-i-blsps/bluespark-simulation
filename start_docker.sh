#!/usr/bin/env bash
set -e

COMPOSE_FILE="docker-compose.yaml"
CONTAINER_NAME="bs_sim"

xhost +local:docker > /dev/null 2>&1

# check container status
STATUS=$(docker inspect -f '{{.State.Status}}' "$CONTAINER_NAME" 2>/dev/null || true)

if [ "$STATUS" = "running" ]; then
    echo "Container is already working. Connecting..."
elif [ -n "$STATUS" ]; then
    echo "Container exists, but is exited. Starting..."
    docker compose -f "$COMPOSE_FILE" start
else
    echo "Creating and staring container..."
    docker compose -f "$COMPOSE_FILE" up -d
    
    # permissions to FUEL 
    docker compose -f "$COMPOSE_FILE" exec -u root gazebo_dev chown -R ubuntu:ubuntu /home/ubuntu/.gz
fi

docker compose -f "$COMPOSE_FILE" exec -it gazebo_dev bash
