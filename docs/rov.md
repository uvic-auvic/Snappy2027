# PS4 ROV operation

ROV is a separate launch configuration. It never starts the mission planner,
autonomous navigation, cameras, or a second thruster driver. The normal
`snappy_realsense.launch.py` and autonomous controller remain available.

## Architecture

The existing `controller.cpp` contains all six PIDs, the vehicle's 6-by-8
allocation matrix, the `ThrusterAllocator`, and the only `/motor_cmd` publisher.
There is no `controller.hpp`, separate depth node, manual effort topic, or
general emergency-stop service in the original code. Autonomous `Task.z` sets
the depth target; the controller computes depth error and passes it through
`pid_z_`. The existing allocator converts a body wrench into individual forces,
then the controller converts those to `ThrusterCommand` percentages.

The estimator publishes `snappy_interfaces/msg/Pose` on
`/state_estimator/state`. World +Z means deeper; body +X is forward, +Y right,
and positive yaw turns right. It initializes from pressure and Xsens data.
Pressure is `/depth_data` (`std_msgs/msg/Float32`). The pose has no timestamp
or validity field, and IMU-driven predictions continue after pressure loss.

The new path is:

```text
/joy -> rov_controller -> /rov/command -> rov_depth_controller
                           ^                    |
/depth_data and /state_estimator/state ---------+
                                                |
                         existing depth PID + allocator
                                                |
                        /rov/allocated_thrusters (preview)
                                                |
                   output disabled: /rov/disabled_motor_cmd (zeros)
                   output enabled:  /motor_cmd (ThrusterCommand, best effort)
```

`rov_depth_controller` compiles the existing controller with a fixed ROV startup
configuration. It omits planner subscriptions and uses `RovControl` to combine
direct normalized surge, sway and yaw effort with the existing `pid_z_` and
existing 5-unit buoyancy compensation. It introduces no yaw PID or heading
hold. Depth force is rotated from world into body coordinates before allocation.
The PID gained a history-reset method for safe re-entry after a fault; its
autonomous update behavior is unchanged.

`RovCommand` is the only new message. The existing `Task` represents an
autonomous pose goal and cannot represent direct effort or input safety state.
The new message carries normalized effort, depth target, enable state and raw
L1/Circle state atomically. Motor commands retain the existing message and
allocation interface. The input node never publishes motor commands.

## Configuration and mapping

Edit `src/launch/config/rov_params.yaml`, then rebuild/reinstall launch files.
All ROV parameters are read-only after startup; restart to change configuration.

| Input | Default index | Action |
| --- | --- | --- |
| R2 | axis 5 | Forward, released +1 / fully pressed -1 |
| L2 | axis 2 | Reverse, released +1 / fully pressed -1 |
| Left stick horizontal | axis 0, inverted | Strafe |
| Right stick horizontal | axis 3, inverted | Direct yaw effort |
| D-pad | axis 7, +1 up / -1 down | Change depth target by 0.25 m |
| L1 | button 4 | Manual deadman |
| Circle | button 1 | Latched emergency stop |

Vertical sticks are unused. The 10% stick deadzone is rescaled smoothly to
full travel. Set `trigger_released`/`trigger_pressed` to `0.0`/`1.0` for
zero-based triggers, or swap endpoints to invert trigger polarity. Configure
`invert_strafe`, `invert_yaw`, and `invert_dpad` as needed. For a driver exposing
D-pad buttons, set `dpad_axis: -1` and assign both button indices.

Normalized horizontal and yaw limits default to 0.30. For example, R2=80% and
L2=30% request surge `0.30 * (0.80 - 0.30) = 0.15`. The controller uses the
existing geometry and asymmetric -4/+5 force limits to scale each axis.
Combined requests are additionally capped at the horizontal motors to the
larger of the configured horizontal/yaw percentage limits. Vertical allocation
continues using the existing allocator limits.

`min_depth_m` defaults to 0.0. UP is ignored while measured depth is within
`surface_tolerance_m` (default 0.05 m) of that minimum. Targets cannot go below
the minimum or above `max_depth_m`. **No approved maximum was found.** The
default `max_depth_m: -1.0` disables depth control and all thrust. Enabling
physical output without explicitly configuring a valid maximum fails startup.
The 0–5 m limits in `rov_test_params.yaml` are synthetic test bounds only.

On initialization and after a depth fault, the target is the first healthy
estimated depth, with fresh pressure required. D-pad changes require L1 and
a direction release between presses. A direction pressed while L1 is released
does not become a new press when L1 is subsequently held.

## Safety behavior

* L1 must first be observed released. Releasing it zeros manual surge, sway and
  yaw; depth stabilization continues. D-pad input also requires L1.
* Joy loss for 0.5 s disarms manual control. Reconnecting while L1 is held
  cannot resume movement. Release and press L1 again. Healthy depth hold
  continues during controller disconnection.
* Empty/short arrays, nonfinite/out-of-range axes, invalid button values, and
  stale/future Joy timestamps cannot arm movement. Circle is still honored
  when other fields are malformed. Simulated messages must carry current stamps.
* The depth controller independently validates command limits, command freshness,
  fresh pressure, a fresh finite estimator depth and a valid orientation. Its
  watchdog stops **all** allocation if the input process stops publishing for
  `command_timeout_s` (default 0.5 s). A hanging/crashed depth controller stops
  sending commands; the motor board's command-loss stop is required for that
  case. Board firmware is maintained elsewhere and was not tested here.
* Invalid/stale pressure or estimator data stops all allocation and resets PID
  history. Recovery waits for pressure and a subsequent estimator sample,
  captures current depth, and requires another L1 release/press for manual use.
* Circle immediately latches zero in the depth controller, including vertical
  output. The controller continuously publishes zeros until an explicit reset.
  The latch survives input-node loss/restart. Reset requires fresh commands and
  depth data, L1 and Circle released, and an unexpired kill timer.
* The existing timed kill remains in force: `controller_params.yaml` currently
  sets **20 seconds** from controller startup. It latches all output at zero
  and cannot be cleared by the emergency-stop reset service. Choose and document
  an appropriate finite timeout in an approved deployment configuration. The
  simulation override uses 120 seconds.

Joy timers run every 20 ms; the controller watchdog/output timer runs every
100 ms. Allow one timer period beyond a configured timeout for enforcement.
Watchdogs use steady time even when simulated ROS time pauses. Header age is
also checked. Multiple `/rov/command` publishers inhibit allocation.

The physical kill switch remains the independent hardware stop. Run only one
vehicle controller in a ROS domain. ROV launch creates one motor-command owner;
it does not stop an independently launched autonomous controller. Sensor nodes,
Linux joy and the micro-ROS agent are opt-in to avoid duplicates. An existing
agent can serve the board; no additional ROS thruster driver is introduced.

## Build and automated tests

Use the normal Docker setup described in [docker.md](docker.md), or an installed
ROS 2 Humble development environment. Inside the workspace/container:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to snappy_control snappy_launch --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select snappy_control --event-handlers console_direct+
colcon test-result --verbose
```

Building `snappy_launch` also builds its existing autonomous/vision/driver
dependencies, even though ROV simulation does not launch them. For a minimal
Humble environment that only needs ROV simulation, build the control packages
first, then discover just the launch package:

```bash
colcon build --packages-up-to snappy_control --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon build --base-paths src/launch --packages-select snappy_launch
source install/setup.bash
```

Tests have no hardware nodes and hard-disable physical output. Pure C++ tests
cover conversion, mapping variants, D-pad edge handling, limits, surface guard,
deadman arming, malformed data and safety state transitions. The DDS integration
test uses a separate local ROS domain and checks both C++ processes, PID response,
all input axes, Joy timeout, stale/malformed input, pressure loss despite fresh
estimator data, invalid pressure, recovery, an input process paused with SIGSTOP,
latched stop and guarded reset. It asserts there is no `/motor_cmd` publisher
and disabled output remains zero.

## Controller-free simulation

Use a dedicated ROS domain in every terminal below:

```bash
export ROS_DOMAIN_ID=77
ros2 launch snappy_launch snappy_rov_sim.launch.py scenario:=half_forward
```

This starts both controllers and a fixed 2 m pressure/estimated-depth fixture.
It never starts hardware nodes or a micro-ROS agent. The synthetic sensor fixture
is not a vehicle physics simulator. Inspect the request and allocator preview:

```bash
export ROS_DOMAIN_ID=77
ros2 topic echo /rov/command
ros2 topic echo /rov/allocated_thrusters
ros2 topic echo /rov/disabled_motor_cmd   # Always zeros
ros2 topic info /motor_cmd               # No publisher from this launch
```

Available scenarios: `neutral`, `half_forward`, `full_reverse`, `both_triggers`,
`strafe_left`, `strafe_right`, `yaw_left`, `yaw_right`, `depth_up`, `depth_down`,
`held_dpad`, `deadman_release`, `emergency_stop`, `disconnect`, and `malformed`.
Each starts with L1 released for one second. Depth up/down scenarios pulse once
per second; `held_dpad` steps only once. The disconnect scenario stops Joy after
two seconds while depth fixtures continue. The stop scenario pulses Circle and
then releases it, demonstrating that allocation stays zero.

For interactive reset testing, start the controllers without the fixture:

```bash
ros2 launch snappy_launch snappy_rov_sim.launch.py start_simulator:=false
```

In a second terminal in the same domain, run
`ros2 run snappy_launch rov_simulator.py --scenario emergency_stop`. After the
stop latches, Ctrl+C that fixture and replace it with a neutral fixture, then
request reset while L1/Circle are released:

```bash
ros2 run snappy_launch rov_simulator.py --scenario neutral
# In another terminal:
ros2 service call /rov/reset_emergency_stop std_srvs/srv/Trigger '{}'
```

For individually stamped Joy messages, use a small rclpy publisher or the fixture
script as a template. Simultaneously running multiple Joy publishers is unsuitable
for manual control. Never run synthetic depth fixtures in the vehicle's live ROS
domain.

## Hardware configuration

First verify the real `/joy` layout with physical output disabled. Supply an
approved parameter file containing explicit operating depth bounds. The default
launch accepts external sensors/estimator and does not start an agent or joy node:

```bash
ros2 launch snappy_launch snappy_rov.launch.py rov_params:=/absolute/path/approved_rov.yaml
```

For a complete ROV hardware configuration, `start_sensors:=true` starts pressure,
Xsens and estimation; `start_joy:=true` starts Linux `joy_node`. Use
`start_micro_ros_agent:=true serial_dev:=/dev/ttyUSB1` only if the board has no
agent running. Only the explicit `motor_output_enabled:=true` launch argument
connects the controller's effective output to `/motor_cmd`. The default remains
false, including with a custom YAML file. Do not run this beside the autonomous
controller. Physical controller layout, motor directions, pressure reference,
approved depth bounds, depth PID tuning and firmware command-loss timeout still
require validation on the vehicle before operation.
