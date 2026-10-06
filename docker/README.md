# ROS 2 Humble container

Ubuntu 24.04 can't run Humble natively (Humble targets 22.04), so this runs it
in a container with your `src/` mounted from the host.

Targets this machine: **Jetson, aarch64, JetPack R38 / L4T**. The image also
builds on x86-64 — the base image is multi-arch and the apt architecture is
detected rather than hardcoded.

## One-time setup

```bash
# 1. Install Docker Engine + the compose plugin (asks for your password).
#    Docker itself is already present here, but `docker compose` is not.
sudo bash docker/install-docker.sh

# 2. Pick up the docker group without logging out
newgrp docker

# 3. Build the image (~15-30 min on Jetson, several GB: nav2 dominates)
cd docker && ./run.sh build
```

## Daily use

```bash
cd docker
./run.sh shell      # start container (if needed) and open a shell
./run.sh down       # stop it
```

Inside the container:

| alias    | does                                              |
|----------|---------------------------------------------------|
| `rdep`   | `rosdep install` deps for everything in `src/`    |
| `rbuild` | `colcon build --symlink-install`                  |
| `rsource`| source `install/setup.bash`                       |
| `rtest`  | `colcon test` + show results                      |
| `rclean` | delete `build/ install/ log/`                     |

## Verify it works

```bash
# talker/listener in two shells -> confirms DDS
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener

# GUI + GL
rviz2
glxinfo | grep "OpenGL renderer"
```

## Notes

- **Base image is `ros:humble-ros-base`, not `osrf/ros:humble-desktop`.** The
  `osrf/ros` desktop tags are published for `linux/amd64` only and cannot run
  on this Jetson; the official `ros` images are multi-arch. The desktop tooling
  those tags would have provided is installed via `ros-humble-desktop` in the
  Dockerfile instead.
- **No Gazebo on arm64.** `packages.ros.org` has no aarch64 build of
  `ros-humble-gazebo-ros-pkgs` (nor of the `ros-gz` metapackage) for jammy, so
  the Dockerfile skips that step on this host rather than failing the build.
  `INSTALL_SIM=yes` in `.env` forces the attempt; on amd64 `auto` installs it.
  Everything else — nav2, slam_toolbox, rviz2, rqt, robot_localization — is
  published for arm64 and is installed.
- **Same UID/GID as you (1000).** Files you create in the mounted workspace stay
  owned by `dogu` on the host — no root-owned build artifacts.
- **No roscore.** ROS 2 discovers peers over DDS. Anything that should talk to
  this container needs the same `ROS_DOMAIN_ID` (see `.env`). Set
  `ROS_LOCALHOST_ONLY=1` to keep traffic off the LAN.
- **GUI apps** use the host X server (Xwayland) via `/tmp/.X11-unix`. `run.sh`
  refreshes `/tmp/.docker.xauth` each time because mutter renames its
  Xauthority file every session.
- **GPU on Jetson.** `docker-compose.yml` sets `runtime: nvidia`, which
  bind-mounts the L4T driver stack (per
  `/etc/nvidia-container-runtime/host-files-for-container.d/*.csv`) into the
  container. `/etc/docker/daemon.json` on this host already registers that
  runtime and `nvidia-container-toolkit` is installed. On a machine without it,
  comment the `runtime:` line out and fall back to `/dev/dri` + mesa.
- **If rviz2 renders black or crashes**, set `LIBGL_ALWAYS_SOFTWARE=1` in `.env`
  and restart — falls back to llvmpipe.
- **Humble is supported until May 2027**, so unlike the old Foxy image there is
  no `--include-eol-distros` on `rosdep` and no EOL apt workaround. The
  Dockerfile does still refresh the ROS apt signing key, because ROS rotated it
  in 2025 and image layers built before then carry the expired one.
- **The apt-source cleanup must sweep `*.sources`, not just `*.list`.** The
  `ros:humble-*` images moved to deb822 format: `sources.list.d/ros2.sources`,
  a symlink into `/usr/share/ros-apt-source/`, with the signing key inlined
  under `Signed-By:`. If step 1 removes only `*.list`, that file survives, and
  the fresh `ros2.list` written next declares packages.ros.org a second time
  with a different `Signed-By`. apt treats a conflicting `Signed-By` as fatal,
  so *every* later apt command dies with `E: The list of sources could not be
  read.` -- the build fails at step 1 with exit 100. Note that the base image's
  own inlined key is already the rotated one and carries no `arch=` pin, so the
  refresh is now a safety net for stale layers rather than a requirement.
- **Don't upgrade pip/setuptools in the image.** Humble's `ament_python` builds
  expect jammy's setuptools 59.6.0; the Foxy pin of 58.2.0 was removed rather
  than bumped.
- **Serial/USB hardware:** uncomment the `devices:` and `group_add:` block in
  `docker-compose.yml`.
- **"Cannot connect to the Docker daemon" right after installing Docker.**
  Swapping distro `docker.io` for `docker-ce` rewrites `docker.socket` while it
  is running, so `dockerd -H fd://` finds no listener to inherit and exits with
  `failed to load listeners: no sockets found via socket activation`. systemd
  then rate-limits the retries and leaves both units `failed`. Fix:

  ```bash
  sudo systemctl daemon-reload
  sudo systemctl reset-failed docker.socket docker.service
  sudo systemctl start docker.socket
  sudo systemctl start docker
  ```

  `reset-failed` is the part that matters — without it systemd won't retry.
  `install-docker.sh` now does this itself.
- **"permission denied ... /var/run/docker.sock"** is a different problem: the
  daemon is up but your shell predates your `docker` group membership. Run
  `newgrp docker`, or log out and back in.
- `legacy-ros1-backup/` holds your previous catkin-based files.

## What changed from the Foxy setup

| | Foxy | Humble |
|---|---|---|
| base image | `osrf/ros:foxy-desktop` (focal, amd64-only) | `ros:humble-ros-base` (jammy, multi-arch) + `ros-humble-desktop` |
| apt suite | `focal` | `jammy` |
| apt arch | hardcoded `amd64` — **wrong on this Jetson** | `$(dpkg --print-architecture)` |
| `rosdep update` | needed `--include-eol-distros` | plain |
| setuptools | pinned to 58.2.0 | left at jammy's 59.6.0 |
| user / workspace | `dogux` / `ros2_foxy_ws` | `dogu` / `ros2_humble_ws` |
| `RENDER_GID` | 992 | 993 (this host's `render` group) |
| GPU | `/dev/dri` only | `runtime: nvidia` + `/dev/dri` |
| Gazebo | unconditional | `INSTALL_SIM`, auto-skipped on arm64 |
| extras | — | `rmw_cyclonedds_cpp` alongside FastRTPS |

Three of those were hard breakages on this machine rather than cosmetic
updates: the amd64-only base image, the `arch=amd64` apt line (which makes
every `ros-humble-*` install fail on aarch64), and the unconditional Gazebo
package (which has no aarch64 build). The `dogux` / GID 992 values were the
softer kind — the build succeeds but files in the mounted workspace end up
owned by a user that isn't you.
