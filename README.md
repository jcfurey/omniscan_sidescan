# omniscan_sidescan (ROS 2)

ROS 2 driver for the [Cerulean Omniscan 450 Side Scan
Sonar](https://bluerobotics.com/store/the-reef/cerulean-sidescan-sonar/).

> **This branch is the ROS 2 version.** The original ROS 1 (Noetic) driver
> lives on the `main` branch. The two are kept on separate branches; neither
> contains the other.

The runtime driver is C++20, using `rclcpp`, lifecycle nodes, and composable
components. Each sonar has its own TCP worker, parser, configuration state, and
reconnect backoff. Network waits run outside the ROS executor. The native
protocol/transport code uses the C++ standard library and Linux/POSIX sockets;
it has no Python or vendor SDK dependency.

Python remains available for the TCP simulator, integration tests, launch
files, and the previous driver as a comparison baseline. The simulator and
reference driver use a vendored subset of
[ping-python](https://github.com/bluerobotics/ping-python); no `pip` installation
or separate source checkout is required.

Built and tested on ROS 2 Lyrical with GCC 15.2. A C++20 compiler is required;
other ROS distributions have not been validated for this conversion.

## Packages

| Package | Build type | Contents |
| --- | --- | --- |
| `omniscan_sidescan_msgs` | `ament_cmake_auto` | four legacy message definitions and per-head `SonarProfile` |
| `omniscan_sidescan` | `ament_cmake_auto` | C++ driver/components, launch files, Python simulator/reference driver |

They are split so the messages can be depended on without pulling in the
driver, which is the usual ROS 2 arrangement.

## Building

Clone this branch into a ROS 2 workspace and build. Both packages sit at the
repository root, so colcon finds them directly:

```
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone -b <this branch> <this repository>
cd ~/ros2_ws
colcon build --packages-up-to omniscan_sidescan --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
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

The existing executable and launch names now start C++. Standalone executables
configure and activate automatically, including when a device is unavailable.
For comparison, the previous Python executables are installed as
`sidescansonar_node_python` and `sidescansonarall_node_python`; they accept the
same ROS parameters and publish the same messages.

### Lifecycle and composition

Both native nodes provide the standard lifecycle services. `deactivate` cancels
network waits, stops streaming, and joins all workers; `activate` reconnects
using the latest committed settings. `cleanup` releases publishers and channel
resources. Shutdown and component destruction also join workers before their
ROS resources are destroyed.

The registered components are `omniscan_sidescan::SonarNode` (two heads) and
`omniscan_sidescan::SonarAllNode` (four heads). Components start **unconfigured**,
so a lifecycle manager controls when hardware connections begin. For example,
start a container in one terminal:

```
ros2 run rclcpp_components component_container --ros-args -r __node:=sonar_container
```

Then load and activate the four-head component:

```
ros2 component load /sonar_container omniscan_sidescan omniscan_sidescan::SonarAllNode \
    --node-name omniscan450all
ros2 lifecycle set /omniscan450all configure
ros2 lifecycle set /omniscan450all activate
```

Supply addresses and other read-only parameters when loading the component.
Namespaces, remappings, and `use_intra_process_comms` through component extra
arguments are supported.

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

The combined topics retain the ROS 1 names and message fields:

- `/omniscan450/range_raw` — raw, unscaled `uint16` power results
- `/omniscan450/range` — the same profiles scaled to dB

Both carry identical metadata; they differ only in the sonar results. The
message types are `omniscan_sidescan_msgs/msg/SideScanSonar[Raw]` for the
two-sonar node and `.../SideScanSonarAll[Raw]` for the four-sonar node, with
the same fields as the ROS 1 messages. Profile arrays are bounded to the
device maximum of 1200 samples. Both topics use the ROS 2 sensor-data QoS
profile (best effort, volatile, depth 5) to favor current sonar data over an
increasing reliable backlog.

Each sonar also publishes `omniscan_sidescan_msgs/msg/SonarProfile` on
`/omniscan450/<channel>/profile`, where `<channel>` is `port`, `starboard`,
`bow`, or `stern`. This message carries both raw counts and scaled dB values,
all metadata from that device, and the confirmed configuration generation.
Healthy channels continue publishing when another channel is unavailable.
Topics are relative to the node namespace; `sensor_number` changes `450`.

Each profile header records its **host receive time** in the ROS clock and its
physical `<channel>_frame_id` (default `sonar_<channel>`). Supply mounting
transforms in your vehicle's TF configuration. `timestamp_ms` remains the
device's independent uptime counter; it is not converted into synchronized
acquisition time. Receive-time skew checks cannot establish acoustic
synchronization or compensate for network latency.

Combined output requires a new profile from every channel in the same
configuration generation, with matching geometry, age within `max_profile_age`
(default 0.5 s), and host receive skew within `max_profile_skew` (default 0.05 s).
Its header uses the oldest contributing receive timestamp and the rig
`frame_id`. Incomplete, stale, or skewed sets are withheld without interrupting
the independent profile topics. The combined message retains the first
channel's `channel_number` and vehicle heading for compatibility; use the
per-channel topics when those fields matter.

`/diagnostics` reports connection state, failures, retry counts, duplicate
profiles, parser errors, profile age, and requested versus profile-verified
configuration. It also reports the current availability of combined output.

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
| `length_mm` | 1–300000 | 5000 (6000 in the four-sonar launch file) |
| `msec_per_ping` | 0–10 | 0 |
| `pulse_len_percent` | 0.002–0.004 | 0.002 |
| `filter_duration_percent` | 0.0015–0.0016 | 0.0015 |
| `gain_index` | -1–7 | -1 (auto) |
| `num_results` | 200–1200 | 600 |

`frame_id`, `read_timeout`, `max_profile_age`, and `max_profile_skew` are also
settable at run time. Addresses (`<channel>_ip_address`, `<channel>_port`),
physical `<channel>_frame_id`, connection/retry settings, and `sensor_number`
are read-only. Direct `ros2 run` uses node names `omniscan` or `omniscanall`;
the launch files use `omniscan450` or `omniscan450all` by default.

Parameter success means **desired state was accepted by ROS**. Validation has
no side effects. After commit, each worker applies the settings independently.
Diagnostics expose `desired_generation`, `applied_generation`, requested values,
commanded values, and the profile's reported geometry, sound speed, and gain.
An unavailable or rejecting device remains visibly unconfirmed and retries;
it does not block another channel's configuration or publication. There is no
cross-device hardware transaction or automatic rollback of ROS desired state.

The driver uses speed-of-sound command ID `116`, surfaces NACKs, and waits for
a subsequent profile matching range, result count, sound speed, and manual gain.
An ordered device-information reply separates old buffered profiles from this
confirmation. Ping period and pulse/filter fractions are not reported in the
profile and cannot be read back by this API; they remain commanded values.

| Connection parameter | Default | Meaning |
| --- | --- | --- |
| `connect_timeout` | 5.0 s | TCP connection deadline for each device |
| `configuration_timeout` | 2.0 s | Device-information and profile-confirmation deadlines |
| `read_timeout` | 0.5 s | Profile wait deadline for one device |
| `max_consecutive_failures` | 10 | Reconnect this channel after persistent read/data failures; 0 disables this threshold |
| `reconnect_initial_delay` | 0.25 s | Initial delay before reconnecting a failed channel |
| `reconnect_max_delay` | 10.0 s | Maximum backoff for repeated connection/configuration failures |

Socket failures, NACKs, and configuration drift reconnect the affected channel
without waiting for the read-failure threshold. Retry settings are launch
arguments and YAML parameters. Shutdown cancels connection/configuration/read
waits. Hostname resolution follows the operating system's resolver policy;
use device IP addresses when predictable cancellation is required.

## Testing

```
colcon test --packages-select omniscan_sidescan_msgs omniscan_sidescan
colcon test-result --all
```

Native GTests cover independent command/profile fixtures, framing recovery,
bounded buffering, power scaling, parameter validation, and lifecycle setup.
Python integration tests run the installed C++ executables through TCP and DDS,
checking both legacy message variants, all per-head topics, oldest receipt
stamps, rejected/ignored settings, independent outages, malformed profiles,
configuration drift, counter wrap/reboots/duplicates, lifecycle reactivation,
component load/unload with intra-process enabled, and cancellation of long
waits. The Python reference driver's regression suite is also retained.

These tests use simulated devices; on-hardware firmware and timing validation
is still required.

### Comparing runtime cost

After building and sourcing the workspace, run from `omniscan_sidescan/`:

```
python3 test/benchmark_driver.py --seconds 10 --runs 3
```

This alternates the installed C++ and Python drivers against four separate
TCP simulators, with 1200 samples per head at a requested 20 Hz and six DDS
subscriptions (four profiles and both legacy topics). JSON output includes
per-run CPU time, resident memory, and observed publication rates. Linux
`/proc` accounting measures only the driver process and its threads; simulator
and observer CPU are excluded. This measures host processing cost, not the
physical sonar's achievable ping rate or acoustic synchronization.

On a Threadripper 3970X running ROS 2 Lyrical, GCC 15.2, and a
`RelWithDebInfo` build, three 10-second runs of that workload measured these
medians:

| Driver | CPU (% of one core) | Resident memory | Combined sets/s |
| --- | --- | --- | --- |
| C++ | 1.30% | 20.3 MiB | 19.50 |
| Python reference | 10.50% | 65.3 MiB | 19.50 |

The simulator's requested rate is 20 Hz; its scheduling and packet-generation
overhead account for the observed rate being slightly lower. These are local
host measurements, not a guarantee for other hardware or networks.

## Differences from the ROS 1 package

The ROS 2 implementation includes these deliberate changes.

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
  failure; timeouts, malformed profiles, and broken connections now have
  distinct errors. Independent workers reconnect only failed channels and
  publish health through standard ROS diagnostics.
- **Complete, low-latency TCP I/O.** Writes cannot silently truncate protocol
  packets, reads block efficiently on the socket using monotonic deadlines,
  and the ROS executor handles callbacks while workers acquire profiles.
- **Consistent multi-sonar output.** A combined message is held back while
  channels disagree on scan geometry, configuration generation, freshness, or
  receive-time skew. Independent channel topics remain available.
- **`start_mm` is applied.** The ROS 1 `set_parameters` assigned `length_mm`
  when `start_mm` changed, so `start_mm` never took effect.
- **`dynamic_reconfigure` → ROS 2 parameters**, with asynchronous device
  confirmation after ROS commits the desired values.
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
