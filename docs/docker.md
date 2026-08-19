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
