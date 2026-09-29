#!/usr/bin/env bash
# Start the physical execution/recording environment after offline cuRobo planning.
set -eo pipefail
planning_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$planning_dir/../start_collection.sh" --robot franka --grasp-demo --port 8083 "$@"
