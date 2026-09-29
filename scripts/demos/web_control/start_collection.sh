#!/usr/bin/env bash
# 30 FPS requested on the simulation clock; wall-clock throughput is measured separately.
set -eo pipefail
collection_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$collection_dir/start_simulation.sh" --camera-fps 30 --capture-clock simulation "$@"
