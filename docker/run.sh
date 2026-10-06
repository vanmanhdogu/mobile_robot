#!/usr/bin/env bash
# Convenience wrapper: sets up X11 auth, then builds/starts/enters the container.
#
#   ./run.sh build     rebuild the image
#   ./run.sh up        start the container in the background
#   ./run.sh shell     open a shell inside it (default)
#   ./run.sh down      stop and remove it
#   ./run.sh logs      follow container output
set -euo pipefail

cd "$(dirname "$(readlink -f "$0")")"
CMD="${1:-shell}"
SERVICE=ros2_humble

# X11 cookie the container can use. Under Wayland the real Xauthority file is
# named unpredictably (mutter regenerates it per session), so copy the cookie
# into a fixed path with the host part wildcarded.
setup_xauth() {
  # Host-side cookie lives under $HOME, not /tmp. systemd-tmpfiles sweeps /tmp
  # periodically; when the file disappears while compose still lists it as a
  # bind-mount source, Docker recreates it root-owned, and the touch below then
  # fails with "Permission denied" for every later run. $HOME is not swept.
  local xauth="${HOME}/.docker.xauth"
  [ -e "$xauth" ] && [ ! -f "$xauth" ] && { echo "warn: $xauth exists but is not a file" >&2; return; }
  touch "$xauth"
  if command -v xauth >/dev/null && [ -n "${DISPLAY:-}" ]; then
    xauth nlist "$DISPLAY" 2>/dev/null | sed -e 's/^..../ffff/' | xauth -f "$xauth" nmerge - 2>/dev/null || true
  fi
  chmod 0644 "$xauth"
}

case "$CMD" in
  build) setup_xauth; docker compose build ;;
  up)    setup_xauth; docker compose up -d ;;
  shell)
    setup_xauth
    docker compose up -d
    docker compose exec "$SERVICE" bash
    ;;
  down)  docker compose down ;;
  logs)  docker compose logs -f ;;
  *)     echo "usage: $0 {build|up|shell|down|logs}" >&2; exit 1 ;;
esac
