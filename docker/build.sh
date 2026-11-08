#!/usr/bin/env bash
# Build the maze-sim image. Run from anywhere; the build context is docker/.
set -euo pipefail
cd "$(dirname "$0")"
podman build -t maze-sim -f Containerfile .
