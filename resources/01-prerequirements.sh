#!/bin/bash
set -e

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
RESOURCE_DIR="$SCRIPT_DIR/resources"

# NVIDIA 드라이버, Docker, ROS2 설치
bash "$RESOURCE_DIR/nvidia-driver-install.sh"
bash "$RESOURCE_DIR/docker-install.sh"
bash "$RESOURCE_DIR/ros2-install.sh"

# 반드시 재부팅 필요 (CUDA, ROS2, Docker 적용)
sudo reboot
