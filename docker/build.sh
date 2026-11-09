#!/usr/bin/env bash
# Build the maze-sim image: ./docker/build.sh [tag]   (default tag: maze-sim)
# The build context is the repo root, filtered by .containerignore down to the package.xml files.
set -euo pipefail
cd "$(dirname "$0")/.."
podman build -t "${1:-maze-sim}" -f docker/Containerfile .
