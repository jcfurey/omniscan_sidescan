# omniscan_sidescan (ROS 2)

ROS 2 driver for the [Cerulean Omniscan 450 Side Scan
Sonar](https://bluerobotics.com/store/the-reef/cerulean-sidescan-sonar/).

> **This branch is the ROS 2 version.** The original ROS 1 (Noetic) driver
> lives on the `main` branch. The two are kept on separate branches; neither
> contains the other.

Standalone: the only dependencies are a ROS 2 install (`rclpy`, `std_msgs`)
and the Python standard library. The parts of
[ping-python](https://github.com/bluerobotics/ping-python) the sonar needs are
vendored under `omniscan_sidescan/brping/`, so there is nothing to `pip
install` and no source checkout to keep in sync.

Built and tested on ROS 2 Jazzy.

## Packages

| Package | Build type | Contents |
| --- | --- | --- |
| `omniscan_sidescan_msgs` | `ament_cmake` | the four message definitions |
| `omniscan_sidescan` | `ament_python` | driver, nodes, launch files, simulator |

They are split so the messages can be depended on without pulling in the
driver, which is the usual ROS 2 arrangement.

## Building

Clone this branch into a ROS 2 workspace and build. Both packages sit at the
repository root, so colcon finds them directly:

```
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone -b <this branch> <this repository>
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Running

Adjust the addresses for your vehicle; the defaults match the deployment in
[`notes`](notes) (port `.94`, starboard `.95`, bow `.93`, stern `.92`).

One pair of sonars:

```
ros2 launch omniscan_sidescan sidescansonar.launch.py
```

All four sonars:

```
ros2 launch omniscan_sidescan sidescansonarall.launch.py
```

Override any argument on the command line, for example:

```
ros2 launch omniscan_sidescan sidescansonarall.launch.py \
    port_ip_address:=192.168.2.94 length_mm:=8000 gain_index:=3
```

A parameter file is also provided, see `config/omniscan450.yaml`:

```
ros2 run omniscan_sidescan sidescansonarall_node \
    --ros-args --params-file <path>/config/omniscan450.yaml
```

### Recording

```
ros2 launch omniscan_sidescan logger.launch.py bag_name:=my_run
```

Records every topic by default. For just the sonar topics:

```
ros2 launch omniscan_sidescan logger.launch.py bag_name:=my_run all_topics:=false
```

`all_sonar_and_record.launch.py` brings up the four sonars and the recorder
together. Unlike the ROS 1 version it does not include the `ping_sonar`
echosounder, which is not part of this repository.

### Without hardware

A simulator that speaks the sonar's own protocol is included, so the nodes and
launch files can be exercised with nothing attached:

```
ros2 run omniscan_sidescan omniscan450_simulator \
    --port 51200 --port 51201 --port 51202 --port 51203

ros2 launch omniscan_sidescan sidescansonarall.launch.py \
    port_ip_address:=127.0.0.1 port_port:=51200 \
    starboard_ip_address:=127.0.0.1 starboard_port:=51201 \
    bow_ip_address:=127.0.0.1 bow_port:=51202 \
    stern_ip_address:=127.0.0.1 stern_port:=51203
```

## Output

### Topics

Topic names are unchanged from the ROS 1 package:

- `/omniscan450/range_raw` — raw, unscaled `uint16` power results
- `/omniscan450/range` — the same profiles scaled to dB

Both carry identical metadata; they differ only in the sonar results. The
message types are `omniscan_sidescan_msgs/msg/SideScanSonar[Raw]` for the
two-sonar node and `.../SideScanSonarAll[Raw]` for the four-sonar node, with
the same fields as the ROS 1 messages.

### Parameters

ROS 2 parameters replace ROS 1's `dynamic_reconfigure`. The tunables below can
be changed while the node runs and apply to every sonar the node owns. Ranges
are declared on each parameter, so out-of-range values are rejected. See the
[ping-protocol definitions](https://github.com/bluerobotics/ping-protocol/blob/master/src/definitions/omniscan450.json).

```
ros2 param list /omniscan450all
ros2 param describe /omniscan450all gain_index
ros2 param set /omniscan450all gain_index 3
```

| Parameter | Range | Default |
| --- | --- | --- |
| `speed_of_sound_mm` | 1482000–1500000 | 1482000 |
| `start_mm` | 0–5000 | 0 |
| `length_mm` | 0–300000 | 5000 (6000 for four sonars) |
| `msec_per_ping` | 0–10 | 0 |
| `pulse_len_percent` | 0.002–0.004 | 0.002 |
| `filter_duration_percent` | 0.0015–0.0016 | 0.0015 |
| `gain_index` | -1–7 | -1 (auto) |
| `num_results` | 200–1200 | 600 |

`frame_id` and `read_timeout` are also settable at run time. The sonar
addresses (`<channel>_ip_address`, `<channel>_port`) and `sensor_number` are
read-only, since changing them would mean reconnecting.

## Testing

```
colcon test --packages-select omniscan_sidescan
colcon test-result --all
```

The suite covers the vendored protocol encoder/decoder and runs the driver
against the simulator, including a regression test for the shared-buffer bug
described below. It needs no hardware.

## Differences from the ROS 1 package

Behaviour is otherwise a straight port; these are the deliberate changes.

- **Per-sonar receive buffers.** Upstream ping-python declares
  `PingDevice._input_buffer` at class scope, so every sonar object in a
  process shares one buffer. These nodes open two or four sonars at once, so
  bytes from one sonar were fed to another's parser and profiles could be
  attributed to the wrong channel. The vendored copy makes the buffer
  per-instance.
- **Reads take the newest profile.** The sonar streams continuously whether or
  not anything is reading it. Taking the oldest buffered profile each time let
  the published data fall progressively further behind, so a read now drains
  the backlog and returns the most recent profile.
- **No `rospy.Rate(msec_per_ping)`.** The ROS 1 nodes used a sonar parameter
  as the loop rate, which with the default of `0` is not a valid rate. The read
  loop now runs at whatever rate the sonars deliver.
- **Errors are reported, not swallowed.** A bare `except:` hid every read
  failure; failures now raise `SonarError`, are logged once, and recovery is
  logged too.
- **`start_mm` is applied.** The ROS 1 `set_parameters` assigned `length_mm`
  when `start_mm` changed, so `start_mm` never took effect.
- **`dynamic_reconfigure` → ROS 2 parameters**, with the same names and ranges.
- **A simulator** is included so the stack runs without hardware.
- Serial transport and the `.svlog` recording helpers were dropped from the
  vendored subset; the sonar is reached over TCP and ROS 2 bags cover
  recording.

## Vendored code

`omniscan_sidescan/brping/` is a trimmed copy of ping-python (Blue Robotics,
MIT). Its licence is kept alongside it in `omniscan_sidescan/brping/LICENSE`.
Only the common and Omniscan 450 messages are retained. Note that current
ping-python releases on PyPI *do* ship the Omniscan 450 definitions, contrary
to the ROS 1 README's note — vendoring is a deliberate choice to keep this
package dependency-free, not a workaround.

## Authors

[Monika Roznere](https://monikaroznere.com) — Marine Robotics Lab, Binghamton
University, original ROS 1 driver.

## License

MIT — see [LICENSE](LICENSE).
