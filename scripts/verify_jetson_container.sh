#!/usr/bin/env bash
set -u

fail=0
check() {
  local name="$1"
  shift
  if "$@" >/dev/null 2>&1; then
    echo "PASS  ${name}"
  else
    echo "FAIL  ${name}"
    fail=1
  fi
}

set +u
source /opt/ros/humble/setup.bash
if [[ -f /ros2_ws/install/setup.bash ]]; then
  source /ros2_ws/install/setup.bash
fi
set -u

check "aarch64 architecture" test "$(uname -m)" = aarch64
check "ROS 2 CLI" ros2 --help
check "Snappy launch package" ros2 pkg prefix snappy_launch
check "CUDA compiler" /usr/local/cuda/bin/nvcc --version
check "TensorRT" sh -c 'ldconfig -p | grep -q libnvinfer'
check "GPIO device" test -c /dev/gpiochip0
check "libgpiod tools" gpiodetect
check "USB enumeration" lsusb
check "camera devices" sh -c 'ls /dev/video* >/dev/null 2>&1'

exit "${fail}"
