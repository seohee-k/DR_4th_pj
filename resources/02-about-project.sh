#!/bin/bash
set -e

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
RESOURCE_DIR="$SCRIPT_DIR/resources"

# CUDA, PyTorch, Doosan ROS2 프로젝트 설치
bash "$RESOURCE_DIR/cuda-pytorch-install.sh"
bash "$RESOURCE_DIR/python-dependency.sh"
bash "$RESOURCE_DIR/dsr-project-install.sh"
