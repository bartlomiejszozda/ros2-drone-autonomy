# Collider dev container

The image holds only the toolchain: ROS 2 Humble, ArduPilot build tools, Gazebo Harmonic,
MAVProxy, and Ultralytics with PyTorch and the YOLO weights for `car_detector_node`. The
workspace lives on the host and is mounted at `/ws`, so code, git history and builds survive
rebuilding the container. Gazebo runs headless; camera images are watched in the browser
through `web_video_server`, as they would be from a real drone.

| File | Role |
|---|---|
| `collider.repos` | which repos make up the workspace (forks on `collider` branch, upstream pinned) |
| `Dockerfile` | the toolchain image |
| `Dockerfile.dockerignore` | sends only `package.xml` files to the build (for `rosdep`) |
| `docker-compose.yml` | how the container runs: mounts, ports |
| `compose.nvidia.yaml` | optional: NVIDIA GPU for Gazebo rendering |
| `entrypoint.sh` | starts sshd |

## Install on a new machine

Linux (Ubuntu). Use Docker Engine, not Docker Desktop: Desktop runs a VM, which breaks GPU
access and UID matching. Plan for about 30 GB free (ArduPilot with submodules + builds).

### 1. Host prerequisites (once per machine)

Check whether Docker Engine and the Compose plugin are already installed:

```bash
docker --version
docker compose version
```

If either command is missing, install Docker Engine and the Compose plugin (official script;
or follow `docs.docker.com/engine/install/ubuntu`):

```bash
curl -fsSL https://get.docker.com | sh
```

Give the current user access to Docker, then **log out of the desktop session and back in**
(or reboot). Group membership does not take effect in the existing login session.

```bash
sudo usermod -aG docker "$USER"
# After logging in again:
id -nG                                 # must include docker
docker run --rm hello-world            # must work without sudo
```

Install Git and `vcstool`. `vcstool 0.3.0` still imports `pkg_resources`, which was removed
from newer `setuptools`; pinning `setuptools` inside this pipx environment keeps it working
on current Ubuntu/Python releases without changing system Python packages.

```bash
sudo apt install -y git pipx
pipx install vcstool
pipx runpip vcstool install 'setuptools<81'
pipx ensurepath                           # then open a new terminal (or source ~/.profile)
vcs --version                             # check
```

Repositories are cloned over HTTPS, so no GitHub account is needed. An SSH key is only
needed to push; afterwards switch a repo with `git remote set-url origin git@github.com:...`.

```bash
ssh-keygen -t ed25519                  # skip if you have one
cat ~/.ssh/id_ed25519.pub              # add at github.com: Settings, SSH and GPG keys
ssh -T git@github.com                  # check
```

#### NVIDIA GPU (optional)

Skip this subsection on a machine without an NVIDIA GPU. The host driver must work before
Docker GPU support can work:

```bash
nvidia-smi
```

If that fails on a new installation, install Ubuntu's recommended driver and reboot. If
Secure Boot asks for MOK enrollment, complete it during the reboot.

```bash
sudo apt update
sudo ubuntu-drivers install
sudo reboot
```

Do not continue until `nvidia-smi` works on the host. Then install and configure the NVIDIA
Container Toolkit. `--yes` makes the keyring command safe to rerun without an overwrite
prompt.

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
docker run --rm --gpus all ubuntu nvidia-smi    # check: GPU listed
```

### 2. Workspace

```bash
mkdir -p ~/collider_ws/src && cd ~/collider_ws
git clone https://github.com/bartlomiejszozda/ros2-drone-autonomy.git src/collider
vcs import --recursive src < src/collider/docker/collider.repos
```

The `vcs import` command clones every additional repository declared in `collider.repos`
under `src/`, including ArduPilot and its recursive submodules. Do not clone those repositories
one by one. Clone/import everything before building the image: the image installs the
dependencies declared in their `package.xml` files.

These instructions use `~/collider_ws`; if the workspace already exists elsewhere (for
example `~/projects/collider_ws`), use that path consistently in this and later steps.

### 3. Container

```bash
cd ~/collider_ws/src/collider/docker
cat > .env <<EOF
COMPOSE_FILE=docker-compose.yml:compose.nvidia.yaml
HOST_UID=$(id -u)
HOST_GID=$(id -g)
EOF
docker compose up -d --build            # first build takes a while
docker compose exec -u dev dev bash     # -u dev: as root, build files would be root-owned
```

No NVIDIA GPU: use `COMPOSE_FILE=docker-compose.yml` (Gazebo renders on the CPU, slower).

### 4. Build and run (inside the container)

```bash
colcon build --symlink-install --base-paths src src/collider/interfaces/collider_interfaces
source install/setup.bash
ros2 launch collider collider.launch.py with_sim:=true use_gz_sim_gui:=False rviz:=False
```

The first build compiles ArduPilot and takes a while. The second base path is required:
`collider_interfaces` is nested inside the `collider` repository, and colcon does not search
inside a package. Day-to-day build, run and test commands are in
[docs/development.md](../docs/development.md).

Second shell (`docker compose exec -u dev dev bash`):

```bash
ros2 run web_video_server web_video_server --ros-args -p port:=8765
```

Browser on the same machine: `http://localhost:8765/stream?topic=/iris/static_camera_sensor/image`

### If something breaks

- Docker reports `permission denied` for `/var/run/docker.sock`: confirm `id -nG` includes
  `docker`. If it does not, run the `usermod` command from step 1 and fully log out and back in.
- `vcs` fails with `ModuleNotFoundError: No module named 'pkg_resources'`: run
  `pipx runpip vcstool install 'setuptools<81'`, then retry the import.
- `rosdep` errors in the image build: look for `ERROR` lines, the build continues past them.
- `colcon build` misses a dependency: add the package to the `Dockerfile` tools list, then
  `docker compose up -d --build`.
- `nvidia-smi` fails on the host after a kernel update: the running kernel may not have a
  matching NVIDIA module. Run `uname -r` and `modinfo nvidia`; update the installed NVIDIA
  driver and its `linux-modules-nvidia` meta-package, reboot, and make the host check pass first.
- `nvidia-smi` works on the host but fails inside: check `COMPOSE_FILE` in `.env` and rerun
  the `docker run --rm --gpus all ubuntu nvidia-smi` check from step 1. On
  `Failed to initialize NVML: Unknown Error`, restart the container on the host
  (`docker restart <container>`). The container can lose GPU access when the host reloads
  its services; if it keeps happening, check the cgroup settings of the NVIDIA Container
  Toolkit (`/etc/nvidia-container-runtime/config.toml`) and Docker.
- Gazebo is slow although `nvidia-smi` works inside: it may be rendering in software, which
  also caps the simulation at about 1.2x real time. Check which EGL library it loaded:

  ```bash
  for P in $(pgrep -f "gz sim"); do grep -oE 'libEGL_nvidia|libnvidia-eglcore|swrast_dri' \
    /proc/$P/maps 2>/dev/null; done | sort -u
  ```

  `libEGL_nvidia` and `libnvidia-eglcore` mean GPU rendering, `swrast_dri` means software.
  The image registers the NVIDIA EGL driver; the container also needs
  `NVIDIA_DRIVER_CAPABILITIES=all`, which `compose.nvidia.yaml` sets.

## Remote machine (GPU server)

The container's ports listen only on the server's `127.0.0.1`; reach them through a tunnel:

```bash
# laptop
ssh -N -L 10023:localhost:10023 -L 8765:localhost:8765 <user>@<host>
ssh -A -p 10023 dev@localhost
```

`-A` forwards your SSH agent, so `git push` works inside the container without copying keys
into it. Login uses the server's `~/.ssh/authorized_keys` (already has your laptop's key if
you SSH to the server). Other file: set `AUTHORIZED_KEYS` in `.env`.

On a local machine SSH isn't needed: `docker compose exec -u dev dev bash` (or VS Code
"Dev Containers: Attach to Running Container").

## Daily use

- `docker compose up -d` starts it, `docker compose down` stops it; the workspace and the
  `home`/`ccache` volumes stay.
- Rebuild the image (`docker compose up -d --build`) after dependencies change in a
  `package.xml` or in the `Dockerfile`. Code changes only need `colcon build` inside.
- `assistant_node` reads `OPENROUTER_API_KEY`. Export it in `~/.bashrc` inside the container:
  the home directory is a volume and survives rebuilds, while variables set in
  `docker-compose.yml` do not reach SSH sessions.

## Settings (`.env` next to `docker-compose.yml`, not committed)

```bash
COMPOSE_FILE=docker-compose.yml:compose.nvidia.yaml   # drop the second file without NVIDIA
SSH_PORT=10023
VIDEO_PORT=8765
ROS_DOMAIN_ID=0
HOST_UID=1000                       # `id -u` on the host
HOST_GID=1000                       # `id -g` on the host
AUTHORIZED_KEYS=/home/me/.ssh/authorized_keys   # public keys allowed to SSH in
```

Running next to another ROS container: use other ports and another `ROS_DOMAIN_ID`. Compose
puts this container on its own network, so it doesn't share the default Docker bridge.

## How the image is built

- The base image, `ardupilot/ardupilot-dev-ros` with ROS 2 Humble and the ArduPilot build
  tools, is pinned by digest. Tags move; digests do not.
- Layers that change rarely come first. Each `RUN` removes its apt lists, because a later
  layer cannot shrink an earlier one.
- The `rosdep` layer sees only the workspace's `package.xml` files
  (`Dockerfile.dockerignore`). It reruns when a declared dependency changes, not on code
  edits, and it is the reason all repositories must be imported before the first build.
- User `dev` gets the host's UID and GID (`HOST_UID`, `HOST_GID`), so files written into the
  mounted workspace belong to you on the host.
- Shell setup (ROS, the workspace overlay, ArduPilot tools) lives in `/etc/bash.bashrc`, not in
  `~/.bashrc`, because `/home/dev` is a volume and would hide changes made in the image.
- sshd starts every session with an empty environment. `entrypoint.sh` therefore writes the
  container's `PATH`, `GZ_*`, `ROS_*` and similar variables to `/etc/container.env`, which
  the shell sources, and keeps the SSH host keys in a volume so they survive rebuilds.
- `/usr/share/glvnd/egl_vendor.d/10_nvidia.json` registers NVIDIA's EGL driver, so Gazebo
  renders on the GPU.
- Ultralytics brings PyTorch with CUDA (several GB). `numpy` is pinned to 1.24 because ROS
  Humble and apt's SciPy need numpy 1.x, and `setuptools` is pinned because newer versions
  break `colcon build --symlink-install`. The VisDrone-tuned YOLO weights are downloaded at a
  fixed revision with a checksum into `/opt/models/yolo11n-visdrone.pt`; nothing is
  downloaded at runtime.
- `shm_size` is 1 GB because the DDS shared-memory transport carries camera images and
  Docker's default is 64 MB.
- The GPU settings are a separate compose file because a device reservation is a hard
  requirement: without NVIDIA, `docker compose up` would fail.

`docker compose config` prints the merged configuration with every variable filled in.
