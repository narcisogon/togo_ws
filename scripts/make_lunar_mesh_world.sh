#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 HEIGHTMAP_IMAGE [make_lunar_mesh_world.py options]" >&2
  echo "Example: $0 /path/to/lunar_heightmap.png --size-m 350 --height-m 25 --resolution 257" >&2
  exit 2
fi

IMAGE_PATH="$1"
shift

python3 "$SCRIPT_DIR/make_lunar_mesh_world.py" --image "$IMAGE_PATH" "$@"
