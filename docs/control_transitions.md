# Controller and planner behavior

These changes are software-tested, not a hardware qualification. Verify sensor
frames, motor mapping/direction, ESC response, buoyancy compensation and tuned
gains in a restrained, supervised water test before an ocean mission. Keep an
independent physical motor-power cutoff available.

## Mission targets and transitions

YAML x/y/z values are **relative increments along fixed mission/world axes**,
not body-relative translations. Four `z: 1.0` tasks produce targets 1, 2, 3, 4 m.
A subsequent `z: 0.0` holds the previous depth. Targets use the depth sensor's
zero; the planner does not add the measured startup depth. Roll/pitch/yaw remain
absolute mission-frame angles: degrees in YAML, radians on /planner/task.

The controller applies planner depth and preserves integral history between
tasks. A changed position coordinate suppresses that PID's derivative for one
update; any orientation target change suppresses all three angular derivatives.
The first PID update also suppresses derivative. Proportional action remains
immediate; this is not a target ramp. Orientation uses Rz(yaw)*Ry(pitch)*Rx(roll),
with angular control error expressed in the body frame.

Completion requires position error below 0.1 m and orientation error below 5°,
speed below 0.1 m/s and angular rate below 10°/s continuously for 0.5 s of fresh
state samples. These are configurable startup settings. Without DVL,
`enable_xy_control: false` ignores estimated X/Y drift and rejects X/Y missions.

The planner advances on matching acknowledgements and retries every 0.5 s.
Retries do not accumulate movement, reset PID history, restart settling or
extend deadlines. The controller repeats a lost completion acknowledgement.
After the final task it keeps holding that target until a stop condition.
Restart planner and controller together for a new mission: the protocol has
sequence numbers, not a unique mission ID.

## Allocation and integral feedback

Depth PID output plus `heave_feedforward` (existing value 5) defines a world-vertical
force. The controller rotates that into the body frame, so it is not always
body Z when tilted. Allocation reserves vertical force before fitting attitude
and horizontal requests into remaining motor headroom.

`allocate_with_priority()` uses wrench order Fx,Fy,Fz,Tx,Ty,Tz. It computes a
pseudoinverse priority allocation, reducing it only if that solution alone exceeds
motor limits. It adds the largest uniformly scaled remainder that fits.
Feasible full requests match ordinary pseudoinverse allocation. This is
conservative: it does not search the thruster nullspace for extra headroom.
Attitude/horizontal response can be reduced to preserve depth. Insufficient
physical thrust can still prevent depth hold.

Each PID rejects its latest integral increment when it would push farther into
an unachievable output. Integration that unwinds saturation is retained. Its own
output clamp has the same protection. Force feedback is rotated back to world
coordinates; depth feedback subtracts feedforward. Body-frame requested and
allocated wrenches are published on /controller/wrench_requested and
/controller/wrench_allocated.

Allocated wrench is a model prediction before integer command conversion, not
measured thrust. It does not model ESC deadband or calibrated thrust curves.

## Stop conditions and sensors

The controller sends zeros until valid, fresh estimator state arrives. Planner
mode also waits for its first task. Once active, invalid or missing state latches
commands at zero. `state_timeout_s` defaults to 0.5 s. The estimator withholds
state if depth is absent or older than `sensor_timeout_s` (default 1 s), or IMU2
input is invalid. Depth loss can therefore take approximately 1.5 s plus scheduling
delay to stop control. Freshness uses steady-clock receipt time; depth messages
have no sensor timestamp.

The kill deadline starts at **controller startup**, including time waiting for
sensors/tasks, not the first motor command. It is 120 s in the launch config
(30 s when running the executable without that config). Tasks have a separate
60 s deadline in both planner and controller. Kill expiry, active-state loss,
invalid tasks, task timeout or planner abort latch commands at zero.
New tasks/restored sensors cannot clear the latch. Process restart clears it:
do not automatically restart control to recover during a test.

This is not a microcontroller watchdog or a guaranteed hardware shutdown.
It cannot deliver zeros if the controller/Jetson crashes or the motor link
fails. Stopping the process also ends its repeated zero publications.
No firmware watchdog was added. Use the physical cutoff when ending tests.

The depth driver accepts `serial_port`, reports setup errors and rejects
non-finite/malformed values. Reconnection is not automatic. Launch pins the IMU
to `imu_port`, disables device scanning, and rejects duplicate motor/IMU/depth
paths, including symlink aliases. Prefer verified /dev/serial/by-id paths;
ttyUSB numbers can change after reboot.

## Launching after merging and rebuilding

Inside the ROS container:

```bash
colcon build --packages-up-to snappy_launch --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

Launch defaults to sensors/micro-ROS only; controller, planner and DVL are disabled.
`enable_controller:=true` starts fixed-target control; `enable_planner:=true`
starts controller and planner together. Both can move motors. Set verified,
distinct `serial_dev`, `imu_port` and `depth_port` arguments.

For a depth-only mission, after hardware safety checks:

```bash
ros2 launch snappy_launch snappy_realsense.launch.py \
  enable_planner:=true \
  task_file:=/ros2_ws/src/autonomy/config/depth_hold_example.yaml
```

This commands 1 m, then holds 1 m; the second task is not a timed dwell.
The older tasks_example.yaml demands roll/pitch maneuvers, but this checkout has
zero roll/pitch gains. Nonzero targets on disabled attitude axes fail explicitly.
Do not invent gains or use that mission before tuning those axes. Controller and
planner parameters are startup-only: edit config and restart, not ros2 param set.
Inspect /controller/status, /planner/status and /controller/target for progress
and stop reasons.

## Software verification without hardware commands

```bash
colcon build --packages-up-to snappy_control --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ctest --test-dir build/snappy_control --output-on-failure
```

C++ regression tests cover derivative transitions, integral retention,
saturation/unwinding, allocation limits, tilted vertical-force priority,
orientation math and invalid numeric inputs. Python tests run real controller,
planner, estimator and depth-driver processes with simulated input, a virtual
serial port, a private DDS domain and remapped motor topics. They never publish
on /motor_cmd. Tests cover task handshakes, depth hold, retries/timeouts, kill
latching, invalid/stale state, serial parsing and port-alias collision.
