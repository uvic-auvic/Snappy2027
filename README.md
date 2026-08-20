# Snappy2027

AUVic's ROS 2 software stack for the 2027 RoboSub competition cycle.

## Migration

Initial software is being migrated from:

- Repository: `uvic-auvic/snappy-nano`
- Branch: `semiFinalDay2`
- Commit: `660ddbd64e6ae88d4afc5b7a9956ab6ca13089da`
- Historical tag: `robosub-2026-latest`

Simulation and embedded firmware are maintained separately from this repository.

## Docker

For an x86_64 Linux development machine:

```bash
./scripts/setup_docker.sh
```

For the Jetson Orin running JetPack 6:

```bash
./scripts/setup_docker_jetson.sh
```

See [docs/docker.md](docs/docker.md) for manual commands, hardware access, and
container verification.

## Setup

### Source the setup files

Run this command on every new shell to access ROS 2 commands.

```bash
source /opt/ros/humble/setup.bash
```

### Build the workspace

In the root of the workspace, run `colcon build`

### Source the environment

After colcon build is completed successfully, the output will be in the `install` directory. colcon will have generated bash/bat files in the `install` directory to help set up the environment. Run the following command to source the environment:

```bash
source install/setup.bash
```
