#!/usr/bin/env bash
# 独立启动仿真、相机和 RobotServer；网页保留观察功能。
set -eo pipefail
SIM_ROBOT="cobot_magic"
SIM_PORT=8080
SIM_SCENE_CONFIG=""  # 留空使用机器人默认场景
SIM_WIDTH=480
SIM_HEIGHT=360
SIM_CAMERA_FPS=3
simulation_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
simulation_args=(--mode observe --robot "$SIM_ROBOT" --port "$SIM_PORT"
    --width "$SIM_WIDTH" --height "$SIM_HEIGHT" --camera-fps "$SIM_CAMERA_FPS")
if [[ -n "$SIM_SCENE_CONFIG" ]]; then
    simulation_args+=(--scene-config "$SIM_SCENE_CONFIG")
fi
exec bash "$simulation_script_dir/start_web_control.sh" "${simulation_args[@]}" "$@"
