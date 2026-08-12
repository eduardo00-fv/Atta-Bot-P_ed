#!/usr/bin/env bash
# Levanta el contenedor de ROS 2 Jazzy con GPU Intel y X11 via XWayland.
#
#   ./run.sh              -> shell interactiva
#   ./run.sh rviz2        -> corre un comando y sale
#   LIBGL_ALWAYS_SOFTWARE=1 ./run.sh rviz2   -> fuerza llvmpipe si falla la GPU
set -euo pipefail

IMAGE=attabot-ros2:jazzy
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_REPO="${HOME}/Documents/AttaBot-Sim"

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    echo "Falta la imagen ${IMAGE}. Construila con:"
    echo "    docker build -t ${IMAGE} ${REPO_ROOT}/ros2"
    exit 1
fi

# La sesion es Wayland; las apps X11 del contenedor entran por XWayland.
# Se abre el acceso local y se revierte al salir.
if command -v xhost >/dev/null 2>&1; then
    xhost +local: >/dev/null
    trap 'xhost -local: >/dev/null 2>&1 || true' EXIT
fi

# El repo del simulador es opcional: solo se monta si existe.
sim_mount=()
if [ -d "${SIM_REPO}" ]; then
    sim_mount=(-v "${SIM_REPO}:/ws/AttaBot-Sim")
fi

# -it solo si hay terminal de verdad, para que el script sirva igual corrido
# a mano que desde otro script (o desde un agente, que no tiene TTY).
tty_flags=()
if [ -t 0 ] && [ -t 1 ]; then
    tty_flags=(-it)
fi

exec docker run "${tty_flags[@]}" --rm \
    --network=host \
    --ipc=host \
    --device /dev/dri \
    -e DISPLAY="${DISPLAY}" \
    -e QT_X11_NO_MITSHM=1 \
    -e LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-0}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v "${REPO_ROOT}:/ws/Atta-Bot-P_ed" \
    "${sim_mount[@]}" \
    "${IMAGE}" "$@"
