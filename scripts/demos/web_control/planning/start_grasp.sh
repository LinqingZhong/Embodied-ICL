#!/usr/bin/env bash
# Run only AFTER the offline planning worker has exited and released GPU memory.
set -eo pipefail
planning_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
robot="${1:-x5}"
shift || true
case "$robot" in
    x5|x5_dual|cobot_magic) ;;
    *) echo '用法: bash start_grasp.sh x5|x5_dual|cobot_magic [--grasp-arm left|right|fl|fr]'; exit 2 ;;
esac
exec bash "$planning_dir/../start_collection.sh" --robot "$robot" --grasp-demo --port 8083 "$@"
