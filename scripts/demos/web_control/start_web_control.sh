#!/usr/bin/env bash
# Start the browser simulation panel using the existing Conda environment.
set -eo pipefail

# ===== 常用配置：修改这里，然后直接 bash start_web_control.sh =====
WEB_ROBOT="cobot_magic"                         # 可选：cobot_magic（松灵四臂）、x5_dual、x5、a2d、franka
WEB_PORT=8080                           # SSH 隧道两端端口需一致
WEB_WIDTH=480
WEB_HEIGHT=360
WEB_CAMERA_FPS=3
WEB_OUTPUT_DIR="output/web_control"     # 相对仓库根目录，或填写绝对路径
WEB_SCENE_CONFIG=""                     # 留空：使用机器人默认场景
# 自定义场景示例（相对仓库根目录，或填写绝对路径）：
# WEB_SCENE_CONFIG="scripts/demos/web_control/scenes/a2d_table_bottle.json"
# Conda 可在下方修改默认值，也可通过同名环境变量覆盖。
web_control_conda_root="${ISAACLAB_CONDA_ROOT:-${HOME}/anaconda3}"
web_control_conda_env="${ISAACLAB_CONDA_ENV:-env_isaaclab}"
# ================================================================

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    cat <<'HELP'
启动 Isaac Lab 浏览器仿真控制台。

用法：
  bash start_web_control.sh
  ./start_web_control.sh --port 8081
  ./start_web_control.sh --robot cobot_magic
  ./start_web_control.sh --robot a2d
  ./start_web_control.sh --robot franka
  ./start_web_control.sh --width 320 --height 240 --camera-fps 5
  ./start_web_control.sh --duration 30
  ./start_web_control.sh --scene-config scripts/demos/web_control/scenes/table_bottle.json

自动激活 env_isaaclab，并启用 --headless --enable_cameras。
默认值在脚本顶部配置区修改；命令行同名参数会覆盖顶部配置。
默认机器人见脚本顶部 WEB_ROBOT，端口 8080，分辨率 480×360，相机更新上限 3 FPS。
按 Ctrl+C 停止；快照默认保存在仓库的 output/web_control/。

可选环境变量：
  ISAACLAB_CONDA_ROOT   Conda 安装目录，默认 ~/anaconda3
  ISAACLAB_CONDA_ENV    环境名称，默认 env_isaaclab
  OMNI_KIT_ACCEPT_EULA  若已同意 NVIDIA EULA，可设为 YES 用于无人值守启动

远程电脑执行（端口需与启动参数一致）：
  ssh -N -L 8080:127.0.0.1:8080 agiuser@服务器地址
随后浏览器打开 http://127.0.0.1:8080
HELP
    exit 0
fi

web_control_repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Locate the repository rather than assuming the launcher lives at its root.
while [[ ! -f "${web_control_repo_root}/isaaclab.sh" ||
         ! -f "${web_control_repo_root}/scripts/demos/web_control/run.py" ]]; do
    if [[ "${web_control_repo_root}" == "/" ]]; then
        echo "找不到 Isaac Lab 仓库根目录：请将启动脚本保留在仓库内。" >&2
        exit 1
    fi
    web_control_repo_root="$(dirname -- "${web_control_repo_root}")"
done


if [[ ! -f "${web_control_conda_root}/etc/profile.d/conda.sh" ]]; then
    echo "找不到 Conda：${web_control_conda_root}/etc/profile.d/conda.sh" >&2
    echo "请通过 ISAACLAB_CONDA_ROOT 指定 Conda 安装目录。" >&2
    exit 1
fi

# Conda activation also loads this environment's Vulkan driver configuration.
source "${web_control_conda_root}/etc/profile.d/conda.sh"
conda activate "${web_control_conda_env}"
cd -- "${web_control_repo_root}"

# isaaclab.sh uses tabs, which needs a terminal type even when run over SSH.
if [[ -z "${TERM:-}" || "${TERM}" == "dumb" ]]; then
    export TERM=xterm
fi

web_control_args=(
    --robot "${WEB_ROBOT}"
    --port "${WEB_PORT}"
    --width "${WEB_WIDTH}"
    --height "${WEB_HEIGHT}"
    --camera-fps "${WEB_CAMERA_FPS}"
    --output-dir "${WEB_OUTPUT_DIR}"
)
if [[ -n "${WEB_SCENE_CONFIG}" ]]; then
    web_control_args+=(--scene-config "${WEB_SCENE_CONFIG}")
fi

echo "Conda 环境：${CONDA_PREFIX}"
echo "正在启动网页控制台；服务地址将在 WEB_CONTROL_URL 行显示。"
exec "${web_control_repo_root}/isaaclab.sh" -p \
    "${web_control_repo_root}/scripts/demos/web_control/run.py" \
    --headless --enable_cameras "${web_control_args[@]}" "$@"
