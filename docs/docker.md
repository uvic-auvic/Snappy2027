# Docker workflow

Snappy2027 supports an x86_64 ROS 2 Humble development container and an
NVIDIA Jetson Orin container based on JetPack 6 / L4T r36.4.

## Laptop development

```bash
./scripts/setup_docker.sh
```

This starts `snappy2027-dev`, imports the pinned repositories from
`src/drivers/dependencies.repos`, installs dependencies with rosdep, and builds
the workspace.

## Jetson

The host must run JetPack 6.x and have Docker plus NVIDIA Container Toolkit.

```bash
./scripts/setup_docker_jetson.sh
```

The Jetson build enables CUDA/TensorRT vision with `WITH_CUDA=ON`. Hardware is
made available through the NVIDIA runtime, host networking, and `/dev` bind.

Verify the running Jetson container with:

```bash
docker compose -f docker/docker-compose.yml --profile jetson exec snappy-jetson \
  bash /ros2_ws/scripts/verify_jetson_container.sh
```

## Manual commands

```bash
docker compose -f docker/docker-compose.yml up -d snappy-dev
docker compose -f docker/docker-compose.yml exec snappy-dev bash
```

Inside either container:

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch snappy_launch snappy_realsense.launch.py
```

Container build/install/log directories use named Docker volumes. This keeps
host and container build artifacts from contaminating each other. Source and
downloaded external repositories remain in the bind-mounted checkout.

## Camera images and detections GUI (Linux desktop)

The OpenCV `displayFeed` viewer displays the front D455 and bottom D405 feeds
side by side (front on the left, bottom on the right) in one window, with detection boxes, masks, class names, confidence, and distance.
Press **Esc** to exit. Run these commands from the checkout root. The
`display_width` ROS parameter sets the width of each camera panel (default 960).

The repository's model class mappings are:

| Camera | Recognized classes |
| --- | --- |
| Front D455 (`ffc_rs_26.engine`) | blood, buoy, compass, circle, fire, hammer_and_wrench, slalom, sos |
| Bottom D405 (`dfc_rs_26.engine`) | bandage, blood, fire, helmet, nut_and_bolt, pill, plug, warning |

These labels come from `src/vision/include/snappy_computer_vision/detection_classes.hpp`.


First rebuild the viewer in the running Jetson container:

```bash
docker compose -f docker/docker-compose.yml --profile jetson exec snappy-jetson \
  bash -lc 'source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; colcon build --symlink-install --packages-select snappy_computer_vision --cmake-args -DWITH_CUDA=ON -DBUILD_TESTING=OFF'
```

Start the cameras and TensorRT detection nodes in one terminal:

```bash
docker compose -f docker/docker-compose.yml --profile jetson exec snappy-jetson \
  bash -lc 'source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; ros2 launch snappy_computer_vision snappy_computer_vision.launch.py'
```

This requires both connected cameras and the models at
`models/ffc_rs_26.engine` and `models/dfc_rs_26.engine`. The vision launch records
video by default. The main `snappy_realsense.launch.py` currently has its vision
include commented out, so use the vision launch above for this workflow.

From a second **host desktop terminal**, open the viewer:

```bash
./scripts/docker_vision_gui.sh snappy-jetson
```

The script runs the viewer in the initialized service container and copies a
temporary Xauthority cookie into it. Compose mounts the desktop's X11 socket; it does not require disabling X11 access control with `xhost`.
Install `xauth` on the host if missing. A Wayland desktop needs XWayland and a
valid `DISPLAY`. A headless SSH session needs a desktop session available on the
Docker host; this script does not set up a remote desktop.

The viewer also builds without CUDA, so an x86 development container can display
images and detection messages produced elsewhere (or replayed from a bag):

```bash
# Recreate snappy-dev once to apply the X11 socket mount:
docker compose -f docker/docker-compose.yml up -d snappy-dev
# After building snappy_computer_vision in snappy-dev:
./scripts/docker_vision_gui.sh snappy-dev
# For a bag played with ros2 bag play --clock:
./scripts/docker_vision_gui.sh snappy-dev --ros-args -p use_sim_time:=true
```

The default topics are `/d455/d455/color/image_raw`,
`/d405/d405/color/image_raw`, `/d455/detections`, and `/d405/detections`.
Use ROS remapping if your recording uses other names, for example:

```bash
./scripts/docker_vision_gui.sh snappy-jetson --ros-args \
  -r /d455/d455/color/image_raw:=/d455/color/image_raw
```

If a panel shows **no signal**, check `ros2 topic list` and the corresponding
image publisher inside the running container. **det: waiting...** means no
DetectionArray messages have arrived; **STALE** means their timestamps are older
than the viewer's default one-second timeout. For slower inference, pass
`--ros-args -p detection_timeout_s:=3.0`. Overlays use the latest detections on
the latest image, so moving objects can have some alignment lag.

## Switching underwater and above-water camera settings

The vision launch accepts `camera_mode:=underwater` (default) or
`camera_mode:=above_water`. Stop the current camera launch with Ctrl+C before
restarting it with the other mode; leave the display viewer running.

Inside the initialized container:

```bash
# Above-water demo:
ros2 launch snappy_computer_vision snappy_computer_vision.launch.py camera_mode:=above_water
# Underwater operation:
ros2 launch snappy_computer_vision snappy_computer_vision.launch.py camera_mode:=underwater
```

The same argument is forwarded by the full robot launch:

```bash
ros2 launch snappy_launch snappy_realsense.launch.py camera_mode:=above_water
```

The profile files live in `src/vision/config/d455_{underwater,above_water}.json`
and `d405_{underwater,above_water}.json`. The RealSense driver loads them via
`json_file_path` into the camera's advanced-mode settings at startup; this is
camera-side processing configured by the repo, before images reach inference
or the GUI. It does not replace the camera's geometric calibration.

The D455 underwater profile disables automatic white balance and fixes it at
6000 K (`controls-color-white-balance-*`). Its above-water profile enables
automatic white balance and automatic color/depth exposure. The D405 already
uses automatic white balance in both profiles; above-water mode enables
automatic exposure for its stereo sensor, which also supplies its color image.
Both above-water profiles preserve the existing depth settings and
color-correction matrices. The D405 matrix matches the SDK's default D405
matrix, so it should not be treated as a custom underwater tint merely because
it appears in the underwater JSON.

Switching explicitly loads the selected profile, rather than relying on a
camera reboot or an empty JSON path to clear previous settings. Let automatic
white balance settle under the demo lighting, using a neutral white/gray target.
The saved manual white-balance value is inactive when automatic balance is on.
If the bottom image remains orange, check lighting and the camera's active
settings; its repo profile does not disable automatic white balance. Visual
color correction still needs verification with the actual cameras.

After updating these files, rebuild/install `snappy_computer_vision` and
`snappy_launch` in the container so the new files and launch arguments are
available. The source of the SDK's D405 defaults is
[librealsense presets.cpp](https://github.com/realsenseai/librealsense/blob/master/src/ds/advanced_mode/presets.cpp).

## Jetson troubleshooting

Run manual Compose commands from the checkout root (`cd ~/Snappy2027`).
The Jetson setup script can be invoked from any directory.

### NVIDIA runtime configured, but Docker still cannot find it

A socket such as `~/.docker/desktop/docker.sock` or a container hostname of
`docker-desktop` indicates that the CLI is using Docker Desktop. Its daemon
runs in a VM; configuring `/etc/docker/daemon.json` and restarting the host
Docker service does not configure that VM's daemon. Jetson hardware access
requires the host Docker Engine.

On the Jetson host, select and inspect the host daemon:

```bash
unset DOCKER_HOST DOCKER_CONTEXT
docker context use default
docker context inspect default --format '{{.Endpoints.docker.Host}}'
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
docker info --format '{{json .Runtimes}}'
```

The endpoint should be `unix:///var/run/docker.sock`, and the runtimes should
include `nvidia`. If the endpoint differs, inspect `docker context ls` and
select the context for the host Engine. If the default context's endpoint is
correct but `nvidia` is still missing, inspect `sudo systemctl status docker`
and `sudo journalctl -u docker -n 50 --no-pager` for configuration errors.
Do not overwrite the entire daemon configuration; `nvidia-ctk` updates it.

Then build and initialize the workspace:

```bash
cd ~/Snappy2027
./scripts/setup_docker_jetson.sh
```

Docker Desktop and the host Engine have separate images, containers, and
volumes, so the host Engine may need to build the image and workspace again.
See [Docker's Linux Desktop documentation](https://docs.docker.com/desktop/setup/install/linux/)
and [NVIDIA's runtime configuration instructions](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#configuring-docker).

### `colcon build` cannot find `mavros_msgs`

Sourcing ROS sets up the environment; it does not install dependencies.
`ntrip` and the Xsens driver declare `mavros_msgs` in their package manifests.
Run the dependency installation and build workflow in the running container:

```bash
docker compose -f docker/docker-compose.yml --profile jetson exec \
  -e SNAPPY_WITH_CUDA=ON snappy-jetson \
  bash /ros2_ws/scripts/container_setup.sh
```

This imports the external repositories, installs their dependencies with
rosdep, and builds sequentially with CUDA enabled. For the specific missing
package, an immediate repair inside the container is:

```bash
apt-get update
apt-get install -y ros-humble-mavros-msgs
source /opt/ros/humble/setup.bash
colcon build --symlink-install --executor sequential \
  --cmake-args -DBUILD_TESTING=OFF -DWITH_CUDA=ON
```

Prefer the setup workflow so all workspace dependencies are installed.

### `snappy_interfaces` cannot replace a directory with a symbolic link

Building once with plain `colcon build` creates a copied Python package in
the build directory. A later build with `--symlink-install` needs a symlink
at that same path and cannot replace the directory automatically.

`scripts/container_setup.sh` now detects this generated directory and moves
it into a uniquely named `build/snappy_interfaces/python-copy-backup.*`
directory before building. Rerun the setup script with the updated checkout:

```bash
./scripts/setup_docker_jetson.sh
```

The CMake deprecation, CMP0148, and unused `WITH_CUDA` messages shown alongside
this failure are warnings; the directory/symlink collision is the blocking
error. `WITH_CUDA` is used by the vision package, not the interfaces package.

### Shell startup reports missing paths under `snappy-nano/install`

These messages refer to an old host workspace, independently of Docker.
Inspect the host shell configuration for a stale workspace setup line:

```bash
rg -n 'snappy-nano|install/.*setup' ~/.bashrc ~/.profile ~/.bash_profile 2>/dev/null
```

Remove or comment out the obsolete workspace source line. If another sourced
setup file chains to the old workspace, inspect that workspace's generated
`install/setup.bash` as well. Source the new container workspace only after
its build succeeds, from inside the container:

```bash
source /ros2_ws/install/setup.bash
```
