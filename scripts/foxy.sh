#!/bin/bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export LOCAL_UID="$(id -u)"
export LOCAL_GID="$(id -g)"
cd "$repo_dir"
case "${1:-shell}" in
  image) docker compose build foxy ;;
  build) docker compose run --rm foxy bash -c 'colcon build --symlink-install' ;;
  test) docker compose run --rm foxy bash -c 'colcon test --event-handlers console_direct+ && colcon test-result --verbose' ;;
  smoke) docker compose run --rm foxy python3 /ws/src/Robominton2026/scripts/smoke_test.py ;;
  sim) docker compose run --rm foxy ros2 launch robom1_bringup sim.launch.py rviz:=false "${@:2}" ;;
  gui)
    if [[ -z "${DISPLAY:-}" ]]; then
      echo 'DISPLAY is not set. Run this on the desktop host terminal.' >&2
      exit 1
    fi
    command -v xauth >/dev/null || { echo 'Install xauth on the host first.' >&2; exit 1; }
    export FOXY_XAUTHORITY="$(mktemp /tmp/robominton-foxy-xauth.XXXXXX)"
    trap 'rm -f "$FOXY_XAUTHORITY"' EXIT
    # Wildcard family lets this display cookie work with the container hostname.
    # The cookie is private and only this temporary file is mounted into Docker.
    xauth nlist "$DISPLAY" | sed 's/^..../ffff/' | xauth -f "$FOXY_XAUTHORITY" nmerge -
    if [[ ! -s "$FOXY_XAUTHORITY" ]]; then
      echo 'No X11 authentication cookie found for DISPLAY.' >&2
      exit 1
    fi
    docker compose -f compose.yaml -f compose.gui.yaml run --rm -T foxy \
      ros2 launch --noninteractive robom1_bringup sim.launch.py rviz:=true "${@:2}"
    ;;
  shell) docker compose run --rm foxy bash --rcfile /usr/local/bin/robominton-entrypoint ;;
  *) echo 'Usage: bash scripts/foxy.sh {image|build|test|smoke|sim|gui|shell}' >&2; exit 2 ;;
esac
