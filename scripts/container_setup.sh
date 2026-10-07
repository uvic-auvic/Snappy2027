#!/usr/bin/env bash
set -euo pipefail

cd /ros2_ws
export ROS_DISTRO=humble
export ROS_PYTHON_VERSION=3

# ROS 2 Humble's generated setup scripts probe optional environment variables
# that may be unset, which is incompatible with nounset while they are sourced.
set +u
source /opt/ros/humble/setup.bash
set -u

echo "--- Importing pinned external repositories ---"
mkdir -p src/external
vcs import --skip-existing src < src/drivers/dependencies.repos

waterlinked_manifest="src/external/waterlinked_dvl/ros2.repos"
if [[ -f "${waterlinked_manifest}" ]]; then
  echo "--- Importing Water Linked dependencies ---"
  vcs import --skip-existing src/external < "${waterlinked_manifest}"
fi

echo "--- Installing rosdep dependencies ---"
apt-get update -q
rosdep update --rosdistro humble
rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro humble \
  --skip-keys nlohmann_json \
  -r -y

echo "--- Building Snappy2027 ---"
# A previous build without --symlink-install leaves a copied Python package
# where ament now needs to create a symlink. Preserve it outside that path.
interfaces_python_dir="build/snappy_interfaces/ament_cmake_python/snappy_interfaces/snappy_interfaces"
if [[ -d "${interfaces_python_dir}" && ! -L "${interfaces_python_dir}" ]]; then
  backup_dir="$(mktemp -d build/snappy_interfaces/python-copy-backup.XXXXXX)"
  mv -- "${interfaces_python_dir}" "${backup_dir}/snappy_interfaces"
  echo "Preserved previous generated Python package in ${backup_dir}"
fi

build_args=(--symlink-install --executor sequential --cmake-args -DBUILD_TESTING=OFF)
if [[ "${SNAPPY_WITH_CUDA:-OFF}" == "ON" ]]; then
  build_args+=( -DWITH_CUDA=ON )
fi
colcon build "${build_args[@]}"

echo "--- Build complete ---"
echo "Run: source /ros2_ws/install/setup.bash"
